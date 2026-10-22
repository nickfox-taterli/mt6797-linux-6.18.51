// SPDX-License-Identifier: GPL-2.0-only
/* BSP idle uses APXGPT GPT4 when the architectural tick is stopped. */
#include <linux/clk.h>
#include <linux/clocksource.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeirq.h>
#include <linux/pm_wakeup.h>
#include <linux/spinlock.h>

#define GPT4_BIT BIT(3)
#define GPT4_CON 0x40
#define GPT4_CLK 0x44
#define GPT4_CNT 0x48
#define GPT4_CMP 0x4c

struct mt6797_gpt_wakeup {
	struct device *dev;
	void __iomem *base;
	spinlock_t lock;
	u32 saved[3];
	u32 saved_gpt2[3];
	struct clocksource sleep_clock;
	u32 delay_ms, fired;
};

static u64 gpt_sleep_read(struct clocksource *clock)
{
	struct mt6797_gpt_wakeup *s = container_of(clock, struct mt6797_gpt_wakeup,
						 sleep_clock);

	return readl(s->base + 0x28);
}

static int gpt_sleep_enable(struct clocksource *clock)
{
	struct mt6797_gpt_wakeup *s = container_of(clock, struct mt6797_gpt_wakeup,
						 sleep_clock);

	writel(0, s->base + 0x20);
	writel(BIT(4), s->base + 0x24);
	writel(BIT(1), s->base + 0x20);
	writel(0x31, s->base + 0x20);
	return 0;
}

static void gpt_sleep_disable(struct clocksource *clock)
{
	struct mt6797_gpt_wakeup *s = container_of(clock, struct mt6797_gpt_wakeup,
						 sleep_clock);

	writel(0, s->base + 0x20);
}

static irqreturn_t gpt_wake_irq(int irq, void *data)
{
	struct mt6797_gpt_wakeup *s = data;
	unsigned long flags;

	if (!(readl(s->base + 4) & GPT4_BIT))
		return IRQ_NONE;
	spin_lock_irqsave(&s->lock, flags);
	writel(0, s->base + GPT4_CON);
	writel(readl(s->base) & ~GPT4_BIT, s->base);
	writel(GPT4_BIT, s->base + 8);
	s->fired++;
	s->delay_ms = 0;
	spin_unlock_irqrestore(&s->lock, flags);
	pm_wakeup_hard_event(s->dev);
	return IRQ_HANDLED;
}

static ssize_t wake_after_ms_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct mt6797_gpt_wakeup *s = dev_get_drvdata(dev);
	unsigned long flags;
	ssize_t len;

	spin_lock_irqsave(&s->lock, flags);
	len = sysfs_emit(buf, "delay_ms=%u fired=%u counter=%u compare=%u control=%x\n",
			 s->delay_ms, s->fired, readl(s->base + GPT4_CNT),
			 readl(s->base + GPT4_CMP), readl(s->base + GPT4_CON));
	spin_unlock_irqrestore(&s->lock, flags);
	return len;
}

static ssize_t wake_after_ms_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct mt6797_gpt_wakeup *s = dev_get_drvdata(dev);
	unsigned long flags;
	u32 delay;

	if (kstrtou32(buf, 0, &delay) || delay > 600000)
		return -EINVAL;
	spin_lock_irqsave(&s->lock, flags);
	writel(0, s->base + GPT4_CON);
	writel(readl(s->base) & ~GPT4_BIT, s->base);
	writel(GPT4_BIT, s->base + 8);
	s->delay_ms = delay;
	if (delay) {
		/* 32kHz always-on source, one-shot, division by one. */
		writel(BIT(4), s->base + GPT4_CLK);
		writel(BIT(1), s->base + GPT4_CON);
		writel(DIV_ROUND_UP_ULL((u64)delay * 32768, 1000), s->base + GPT4_CMP);
		writel(readl(s->base) | GPT4_BIT, s->base);
		writel(BIT(0), s->base + GPT4_CON);
	}
	spin_unlock_irqrestore(&s->lock, flags);
	return count;
}
static DEVICE_ATTR_RW(wake_after_ms);

static struct attribute *gpt_attrs[] = {
	&dev_attr_wake_after_ms.attr, NULL,
};
ATTRIBUTE_GROUPS(gpt);

static void gpt_restore(void *data)
{
	struct mt6797_gpt_wakeup *s = data;
	unsigned long flags;

	spin_lock_irqsave(&s->lock, flags);
	writel(0, s->base + GPT4_CON);
	writel(readl(s->base) & ~GPT4_BIT, s->base);
	writel(GPT4_BIT, s->base + 8);
	writel(s->saved[1], s->base + GPT4_CLK);
	writel(s->saved[2], s->base + GPT4_CMP);
	writel(s->saved[0], s->base + GPT4_CON);
	writel(0, s->base + 0x20);
	writel(s->saved_gpt2[1], s->base + 0x24);
	writel(s->saved_gpt2[2], s->base + 0x2c);
	writel(s->saved_gpt2[0], s->base + 0x20);
	spin_unlock_irqrestore(&s->lock, flags);
}

static int gpt_wakeup_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mt6797_gpt_wakeup *s;
	struct clk *clock;
	int irq, ret;

	s = devm_kzalloc(dev, sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;
	s->dev = dev;
	spin_lock_init(&s->lock);
	s->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(s->base))
		return PTR_ERR(s->base);
	clock = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(clock))
		return PTR_ERR(clock);
	/* The bootloader leaves GPT4 free-running for boot timestamps. The DT
	 * reserves this page for Linux wakeups after arch_sys_counter takes over.
	 * Clear bootloader interrupt masks: no other Linux timer owns this page.
	 * Refuse a different GPT4 active operating mode.
	 */
	if (((readl(s->base + GPT4_CON) & BIT(0)) &&
	     (readl(s->base + GPT4_CON) & 0x30) != 0x30))
		return -EBUSY;
	s->saved[0] = readl(s->base + GPT4_CON);
	s->saved[1] = readl(s->base + GPT4_CLK);
	s->saved[2] = readl(s->base + GPT4_CMP);
	s->saved_gpt2[0] = readl(s->base + 0x20);
	s->saved_gpt2[1] = readl(s->base + 0x24);
	s->saved_gpt2[2] = readl(s->base + 0x2c);
	writel(0, s->base);
	writel(readl(s->base + 4), s->base + 8);
	writel(0, s->base + GPT4_CON);
	writel(GPT4_BIT, s->base + 8);
	ret = devm_add_action_or_reset(dev, gpt_restore, s);
	if (ret)
		return ret;
	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	ret = devm_request_irq(dev, irq, gpt_wake_irq, 0, dev_name(dev), s);
	if (ret)
		return ret;
	ret = device_init_wakeup(dev, true);
	if (ret)
		return ret;
	ret = dev_pm_set_wake_irq(dev, irq);
	if (ret) {
		device_init_wakeup(dev, false);
		return ret;
	}
	platform_set_drvdata(pdev, s);
	s->sleep_clock.name = "mt6797-gpt2-sleep";
	s->sleep_clock.rating = 50;
	s->sleep_clock.read = gpt_sleep_read;
	s->sleep_clock.enable = gpt_sleep_enable;
	s->sleep_clock.disable = gpt_sleep_disable;
	s->sleep_clock.mask = CLOCKSOURCE_MASK(32);
	s->sleep_clock.flags = CLOCK_SOURCE_IS_CONTINUOUS | CLOCK_SOURCE_SUSPEND_NONSTOP;
	s->sleep_clock.owner = THIS_MODULE;
	ret = clocksource_register_hz(&s->sleep_clock, 32768);
	if (ret) {
		dev_pm_clear_wake_irq(dev);
		device_init_wakeup(dev, false);
		return ret;
	}
	dev_info(dev, "BSP GPT4 wake timer, 32kHz, initially disarmed\n");
	return 0;
}

static void gpt_wakeup_remove(struct platform_device *pdev)
{
	struct mt6797_gpt_wakeup *s = platform_get_drvdata(pdev);

	clocksource_unregister(&s->sleep_clock);
	dev_pm_clear_wake_irq(&pdev->dev);
	device_init_wakeup(&pdev->dev, false);
}

static const struct of_device_id gpt_wakeup_of_match[] = {
	{ .compatible = "mediatek,mt6797-gpt-wakeup" }, {},
};
MODULE_DEVICE_TABLE(of, gpt_wakeup_of_match);

static struct platform_driver gpt_wakeup_driver = {
	.probe = gpt_wakeup_probe,
	.remove = gpt_wakeup_remove,
	.driver = {
		.name = "mt6797-gpt-wakeup",
		.of_match_table = gpt_wakeup_of_match,
		.dev_groups = gpt_groups,
	},
};
module_platform_driver(gpt_wakeup_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MT6797 BSP APXGPT4 always-on suspend wake timer");
