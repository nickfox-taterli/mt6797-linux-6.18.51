// SPDX-License-Identifier: GPL-2.0+
/*
 * Mediatek Watchdog Driver
 *
 * Copyright (C) 2014 Matthias Brugger
 *
 * Matthias Brugger <matthias.bgg@gmail.com>
 *
 * Based on sunxi_wdt.c
 */

#include <dt-bindings/reset/mt2712-resets.h>
#include <dt-bindings/reset/mediatek,mt6735-wdt.h>
#include <dt-bindings/reset/mediatek,mt6795-resets.h>
#include <dt-bindings/reset/mt7986-resets.h>
#include <dt-bindings/reset/mt8183-resets.h>
#include <dt-bindings/reset/mt8186-resets.h>
#include <dt-bindings/reset/mt8188-resets.h>
#include <dt-bindings/reset/mt8192-resets.h>
#include <dt-bindings/reset/mt8195-resets.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <linux/types.h>
#include <linux/watchdog.h>
#include <linux/interrupt.h>
#include <linux/soc/mediatek/mt6797-wdt.h>
#include <linux/sysfs.h>

#define WDT_MAX_TIMEOUT		31
#define WDT_MIN_TIMEOUT		2
#define WDT_LENGTH_TIMEOUT(n)	((n) << 5)

#define WDT_LENGTH		0x04
#define WDT_LENGTH_KEY		0x8

#define WDT_RST			0x08
#define WDT_RST_RELOAD		0x1971

#define WDT_STATUS		0x0c

#define WDT_MODE		0x00
#define WDT_MODE_EN		(1 << 0)
#define WDT_MODE_EXT_POL_LOW	(0 << 1)
#define WDT_MODE_EXT_POL_HIGH	(1 << 1)
#define WDT_MODE_EXRST_EN	(1 << 2)
#define WDT_MODE_IRQ_EN		(1 << 3)
#define WDT_MODE_AUTO_START	(1 << 4)
#define WDT_MODE_DUAL_EN	(1 << 6)
#define WDT_MODE_CNT_SEL	(1 << 8)
#define WDT_MODE_KEY		0x22000000

#define WDT_SWRST		0x14
#define WDT_SWRST_KEY		0x1209

#define WDT_SWSYSRST		0x18U
#define WDT_SWSYS_RST_KEY	0x88000000

#define WDT_SWSYSRST_EN		0xfc

#define WDT_INTERVAL		0x10
#define WDT_NONRST_REG		0x20
#define WDT_NONRST_REG2		0x24
#define WDT_REQ_MODE_REG	0x30
#define WDT_REQ_IRQ_EN_REG	0x34
#define WDT_DEBUG_CTL		0x40
#define WDT_LATCH_CTL		0x44

/*
 * MT6797 vendor boot chain: the preloader decodes the TOPRGU raw status
 * before the boot chain clears it and keeps the decoded reason (plus the
 * previous boot's copy) in its SRAM ram console.  Layout per preloader
 * ram_console.c: 64-byte header (sig "DBGC", off_pl, off_lpl, sz_pl, ...),
 * then the records whose first word is the decoded reason.
 */
#define MT6797_RAM_CONSOLE_SIG		0x43474244	/* "DBGC" */
#define MT6797_RAM_CONSOLE_HDR_SIZE	64

/* Decoded reboot reasons stored by the preloader (preloader wdt.h). */
#define MT6797_PL_WDT_HW			BIT(0)
#define MT6797_PL_WDT_SW			BIT(1)
#define MT6797_PL_INTERRUPT			BIT(2)
#define MT6797_PL_SPM_THERMAL			BIT(3)
#define MT6797_PL_SPM				BIT(4)
#define MT6797_PL_THERMAL_DIRECT		BIT(5)
#define MT6797_PL_DEBUG				BIT(6)
#define MT6797_PL_SECURITY			BIT(7)
#define MT6797_PL_EINT				BIT(8)
#define MT6797_PL_SYSRST			BIT(9)

#define DRV_NAME		"mtk-wdt"
#define DRV_VERSION		"1.0"

#define MT7988_TOPRGU_SW_RST_NUM	24

static bool nowayout = WATCHDOG_NOWAYOUT;
static unsigned int timeout;
#ifdef CONFIG_FAULT_INJECTION
/* Explicit one-shot failures for the board's independent-RAM tests only.
 * Defaults are zero and reboot clears them.  No request is changed by a
 * failed hold; failed restore attempts exercise the real bounded retry.
 */
static unsigned int tc_fail_hold;
static unsigned int tc_fail_restore;
module_param(tc_fail_hold, uint, 0600);
module_param(tc_fail_restore, uint, 0600);
MODULE_PARM_DESC(tc_fail_hold, "One-shot TC hold failures (RAM fault tests)");
MODULE_PARM_DESC(tc_fail_restore, "One-shot TC restore failures (RAM fault tests)");
#endif

struct mtk_wdt_dev {
	struct watchdog_device wdt_dev;
	void __iomem *wdt_base;
	spinlock_t lock; /* protects WDT_SWSYSRST reg */
	struct reset_controller_dev rcdev;
	bool disable_wdt_extrst;
	bool reset_by_toprgu;
	bool has_swsysrst_en;
	bool vendor_reset;
	/* MT6797 SRAM ram console: decoded preloader reboot reasons, if valid */
	bool ram_console;
	u32 pl_reason;
	u32 prev_pl_reason;
};

struct mtk_wdt_data {
	int toprgu_sw_rst_num;
	bool has_swsysrst_en;
	bool vendor_reset;
	resource_size_t ram_console_phys;
	resource_size_t ram_console_size;
};

static const struct mtk_wdt_data mt2712_data = {
	.toprgu_sw_rst_num = MT2712_TOPRGU_SW_RST_NUM,
};

static const struct mtk_wdt_data mt6735_data = {
	.toprgu_sw_rst_num = MT6735_TOPRGU_RST_NUM,
};

static const struct mtk_wdt_data mt6797_data = {
	.vendor_reset = true,
	.ram_console_phys = 0x0012d000,
	.ram_console_size = 0xc00,
};

int mt6797_wdt_thermal_direct(struct device *dev, bool enable)
{
	struct mtk_wdt_dev *wdt = dev_get_drvdata(dev);
	unsigned long flags;
	u32 mode, irq, reset;
	int ret = 0;

	if (!wdt || !wdt->vendor_reset)
		return -ENODEV;
	spin_lock_irqsave(&wdt->lock, flags);
	mode = readl(wdt->wdt_base + 0x30) & 0x00ffffff;
	irq = readl(wdt->wdt_base + 0x34) & 0x00ffffff;
	mode = enable ? mode | BIT(18) : mode & ~BIT(18);
	if (enable) {
		/* BSP mtk_wdt_mode_config(): propagate reset to the PMIC and
		 * carry the bypass-power-key flag into the vendor boot chain.
		 * This does not enable the watchdog countdown or change its IRQ.
		 */
		reset = readl(wdt->wdt_base + WDT_MODE);
		reset |= WDT_MODE_EXRST_EN | WDT_MODE_AUTO_START;
		writel(WDT_MODE_KEY | reset, wdt->wdt_base + WDT_MODE);
		if ((readl(wdt->wdt_base + WDT_MODE) &
		     (WDT_MODE_EXRST_EN | WDT_MODE_AUTO_START)) !=
		    (WDT_MODE_EXRST_EN | WDT_MODE_AUTO_START)) {
			ret = -EIO;
			goto unlock;
		}
	}
	/* BSP THERMAL_DIRECT_MODE, RESET_MODE: IRQ is not the actuator. */
	irq &= ~BIT(18);
	writel(0x44000000 | irq, wdt->wdt_base + 0x34);
	writel(0x33000000 | mode, wdt->wdt_base + 0x30);
	if ((readl(wdt->wdt_base + 0x30) & BIT(18)) != (mode & BIT(18)) ||
	    (readl(wdt->wdt_base + 0x34) & BIT(18)))
		ret = -EIO;
unlock:
	spin_unlock_irqrestore(&wdt->lock, flags);
	return ret;
}
EXPORT_SYMBOL_GPL(mt6797_wdt_thermal_direct);

/**
 * mt6797_wdt_thermal_request() - toggle only the TC direct-reset request
 * @dev: watchdog device
 * @enable: arm (true) or disarm (false) REQ_MODE bit 18
 *
 * Unlike mt6797_wdt_thermal_direct(), this never writes WDT_MODE: the
 * preloader documents that MODE writes clear WDT_STATUS, and the per-switch
 * thermal bank guard must not keep clearing the latch that bark/forensics
 * paths will depend on.  MODE-side setup (EXTEN/AUTO_START) stays with the
 * one-shot mt6797_wdt_thermal_direct() at protection-setup time.  The
 * IRQ-route side (REQ_IRQ_EN bit 18) is left as configured by that setup.
 */
int mt6797_wdt_thermal_request(struct device *dev, bool enable)
{
	struct mtk_wdt_dev *wdt = dev_get_drvdata(dev);
	unsigned long flags;
	u32 mode;
	int ret = 0;

	if (!wdt || !wdt->vendor_reset)
		return -ENODEV;

	spin_lock_irqsave(&wdt->lock, flags);
#ifdef CONFIG_FAULT_INJECTION
	if (enable && READ_ONCE(tc_fail_restore)) {
		WRITE_ONCE(tc_fail_restore, tc_fail_restore - 1);
		spin_unlock_irqrestore(&wdt->lock, flags);
		return -EAGAIN;
	}
#endif
	mode = readl(wdt->wdt_base + WDT_REQ_MODE_REG) & 0x00ffffff;
	mode = enable ? mode | BIT(18) : mode & ~BIT(18);
	writel(0x33000000 | mode, wdt->wdt_base + WDT_REQ_MODE_REG);
	if ((readl(wdt->wdt_base + WDT_REQ_MODE_REG) & BIT(18)) != (mode & BIT(18)))
		ret = -EIO;
	spin_unlock_irqrestore(&wdt->lock, flags);

	return ret;
}
EXPORT_SYMBOL_GPL(mt6797_wdt_thermal_request);

/* Called with the shared thermal/EEM bank lock held.  Preserve an
 * intentionally disarmed request, and never clear WDT_STATUS here.
 */
int mt6797_wdt_thermal_hold(struct device *dev, bool *was_armed)
{
	struct mtk_wdt_dev *wdt = dev_get_drvdata(dev);
	unsigned long flags;
	u32 original, value;
	int ret = 0;

	if (!wdt || !wdt->vendor_reset)
		return -ENODEV;
	spin_lock_irqsave(&wdt->lock, flags);
#ifdef CONFIG_FAULT_INJECTION
	if (READ_ONCE(tc_fail_hold)) {
		WRITE_ONCE(tc_fail_hold, tc_fail_hold - 1);
		spin_unlock_irqrestore(&wdt->lock, flags);
		return -EAGAIN;
	}
#endif
	original = readl(wdt->wdt_base + WDT_REQ_MODE_REG) & 0x00ffffff;
	*was_armed = !!(original & BIT(18));
	if (*was_armed) {
		value = original & ~BIT(18);
		writel(0x33000000 | value, wdt->wdt_base + WDT_REQ_MODE_REG);
		if (readl(wdt->wdt_base + WDT_REQ_MODE_REG) & BIT(18)) {
			/* Abort without changing banks; restore the entry state. */
			writel(0x33000000 | original,
			       wdt->wdt_base + WDT_REQ_MODE_REG);
			ret = readl(wdt->wdt_base + WDT_REQ_MODE_REG) & BIT(18) ?
				-EIO : -EREMOTEIO;
		}
	}
	spin_unlock_irqrestore(&wdt->lock, flags);
	return ret;
}
EXPORT_SYMBOL_GPL(mt6797_wdt_thermal_hold);

static const struct mtk_wdt_data mt6795_data = {
	.toprgu_sw_rst_num = MT6795_TOPRGU_SW_RST_NUM,
};

static const struct mtk_wdt_data mt7986_data = {
	.toprgu_sw_rst_num = MT7986_TOPRGU_SW_RST_NUM,
};

static const struct mtk_wdt_data mt7988_data = {
	.toprgu_sw_rst_num = MT7988_TOPRGU_SW_RST_NUM,
	.has_swsysrst_en = true,
};

static const struct mtk_wdt_data mt8183_data = {
	.toprgu_sw_rst_num = MT8183_TOPRGU_SW_RST_NUM,
};

static const struct mtk_wdt_data mt8186_data = {
	.toprgu_sw_rst_num = MT8186_TOPRGU_SW_RST_NUM,
};

static const struct mtk_wdt_data mt8188_data = {
	.toprgu_sw_rst_num = MT8188_TOPRGU_SW_RST_NUM,
};

static const struct mtk_wdt_data mt8192_data = {
	.toprgu_sw_rst_num = MT8192_TOPRGU_SW_RST_NUM,
};

static const struct mtk_wdt_data mt8195_data = {
	.toprgu_sw_rst_num = MT8195_TOPRGU_SW_RST_NUM,
};

/**
 * toprgu_reset_sw_en_unlocked() - enable/disable software control for reset bit
 * @data: Pointer to instance of driver data.
 * @id: Bit number identifying the reset to be enabled or disabled.
 * @enable: If true, enable software control for that bit, disable otherwise.
 *
 * Context: The caller must hold lock of struct mtk_wdt_dev.
 */
static void toprgu_reset_sw_en_unlocked(struct mtk_wdt_dev *data,
					unsigned long id, bool enable)
{
	u32 tmp;

	tmp = readl(data->wdt_base + WDT_SWSYSRST_EN);
	if (enable)
		tmp |= BIT(id);
	else
		tmp &= ~BIT(id);

	writel(tmp, data->wdt_base + WDT_SWSYSRST_EN);
}

static int toprgu_reset_update(struct reset_controller_dev *rcdev,
			       unsigned long id, bool assert)
{
	unsigned int tmp;
	unsigned long flags;
	struct mtk_wdt_dev *data =
		 container_of(rcdev, struct mtk_wdt_dev, rcdev);

	spin_lock_irqsave(&data->lock, flags);

	if (assert && data->has_swsysrst_en)
		toprgu_reset_sw_en_unlocked(data, id, true);

	tmp = readl(data->wdt_base + WDT_SWSYSRST);
	if (assert)
		tmp |= BIT(id);
	else
		tmp &= ~BIT(id);
	tmp |= WDT_SWSYS_RST_KEY;
	writel(tmp, data->wdt_base + WDT_SWSYSRST);

	if (!assert && data->has_swsysrst_en)
		toprgu_reset_sw_en_unlocked(data, id, false);

	spin_unlock_irqrestore(&data->lock, flags);

	return 0;
}

static int toprgu_reset_assert(struct reset_controller_dev *rcdev,
			       unsigned long id)
{
	return toprgu_reset_update(rcdev, id, true);
}

static int toprgu_reset_deassert(struct reset_controller_dev *rcdev,
				 unsigned long id)
{
	return toprgu_reset_update(rcdev, id, false);
}

static int toprgu_reset(struct reset_controller_dev *rcdev,
			unsigned long id)
{
	int ret;

	ret = toprgu_reset_assert(rcdev, id);
	if (ret)
		return ret;

	return toprgu_reset_deassert(rcdev, id);
}

static const struct reset_control_ops toprgu_reset_ops = {
	.assert = toprgu_reset_assert,
	.deassert = toprgu_reset_deassert,
	.reset = toprgu_reset,
};

static int toprgu_register_reset_controller(struct platform_device *pdev,
					    int rst_num)
{
	int ret;
	struct mtk_wdt_dev *mtk_wdt = platform_get_drvdata(pdev);

	spin_lock_init(&mtk_wdt->lock);

	mtk_wdt->rcdev.owner = THIS_MODULE;
	mtk_wdt->rcdev.nr_resets = rst_num;
	mtk_wdt->rcdev.ops = &toprgu_reset_ops;
	mtk_wdt->rcdev.of_node = pdev->dev.of_node;
	ret = devm_reset_controller_register(&pdev->dev, &mtk_wdt->rcdev);
	if (ret != 0)
		dev_err(&pdev->dev,
			"couldn't register wdt reset controller: %d\n", ret);
	return ret;
}

static int mtk_wdt_restart(struct watchdog_device *wdt_dev,
			   unsigned long action, void *data)
{
	struct mtk_wdt_dev *mtk_wdt = watchdog_get_drvdata(wdt_dev);
	void __iomem *wdt_base;
	unsigned long flags;
	u32 reg;

	wdt_base = mtk_wdt->wdt_base;

	spin_lock_irqsave(&mtk_wdt->lock, flags);
	reg = readl(wdt_base + WDT_MODE);
	if (mtk_wdt->vendor_reset) {
		/* Match the MT6797 vendor wdt_arch_reset() sequence. */
		iowrite32(WDT_RST_RELOAD, wdt_base + WDT_RST);
		reg = readl(wdt_base + WDT_MODE);
		reg &= ~(WDT_MODE_AUTO_START | WDT_MODE_IRQ_EN |
			 WDT_MODE_EN | WDT_MODE_DUAL_EN);
		reg |= WDT_MODE_EXRST_EN;
		if (action)
			reg |= WDT_MODE_AUTO_START;
	} else {
		/* Enable reset in order to issue a system reset instead of an IRQ. */
		reg &= ~WDT_MODE_IRQ_EN;
	}
	writel(reg | WDT_MODE_KEY, wdt_base + WDT_MODE);
	spin_unlock_irqrestore(&mtk_wdt->lock, flags);
	if (mtk_wdt->vendor_reset) {
		mb();
		udelay(100);
	}

	while (1) {
		writel(WDT_SWRST_KEY, wdt_base + WDT_SWRST);
		if (mtk_wdt->vendor_reset)
			mb();
		mdelay(5);
	}

	return 0;
}

static int mtk_wdt_ping(struct watchdog_device *wdt_dev)
{
	struct mtk_wdt_dev *mtk_wdt = watchdog_get_drvdata(wdt_dev);
	void __iomem *wdt_base = mtk_wdt->wdt_base;

	iowrite32(WDT_RST_RELOAD, wdt_base + WDT_RST);

	return 0;
}

static int mtk_wdt_set_timeout(struct watchdog_device *wdt_dev,
				unsigned int timeout)
{
	struct mtk_wdt_dev *mtk_wdt = watchdog_get_drvdata(wdt_dev);
	void __iomem *wdt_base = mtk_wdt->wdt_base;
	u32 reg;

	wdt_dev->timeout = timeout;
	/*
	 * In dual mode, irq will be triggered at timeout / 2
	 * the real timeout occurs at timeout
	 */
	if (wdt_dev->pretimeout)
		wdt_dev->pretimeout = timeout / 2;

	/*
	 * One bit is the value of 512 ticks
	 * The clock has 32 KHz
	 */
	reg = WDT_LENGTH_TIMEOUT((timeout - wdt_dev->pretimeout) << 6)
			| WDT_LENGTH_KEY;
	iowrite32(reg, wdt_base + WDT_LENGTH);

	mtk_wdt_ping(wdt_dev);

	return 0;
}

static void mtk_wdt_init(struct watchdog_device *wdt_dev)
{
	struct mtk_wdt_dev *mtk_wdt = watchdog_get_drvdata(wdt_dev);
	void __iomem *wdt_base;

	wdt_base = mtk_wdt->wdt_base;

	if (readl(wdt_base + WDT_MODE) & WDT_MODE_EN) {
		set_bit(WDOG_HW_RUNNING, &wdt_dev->status);
		mtk_wdt_set_timeout(wdt_dev, wdt_dev->timeout);
	}
}

static int mtk_wdt_stop(struct watchdog_device *wdt_dev)
{
	struct mtk_wdt_dev *mtk_wdt = watchdog_get_drvdata(wdt_dev);
	void __iomem *wdt_base = mtk_wdt->wdt_base;
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(&mtk_wdt->lock, flags);
	reg = readl(wdt_base + WDT_MODE);
	reg &= ~WDT_MODE_EN;
	reg |= WDT_MODE_KEY;
	iowrite32(reg, wdt_base + WDT_MODE);
	spin_unlock_irqrestore(&mtk_wdt->lock, flags);

	return 0;
}

static int mtk_wdt_start(struct watchdog_device *wdt_dev)
{
	u32 reg;
	struct mtk_wdt_dev *mtk_wdt = watchdog_get_drvdata(wdt_dev);
	void __iomem *wdt_base = mtk_wdt->wdt_base;
	unsigned long flags;
	int ret;

	ret = mtk_wdt_set_timeout(wdt_dev, wdt_dev->timeout);
	if (ret < 0)
		return ret;

	spin_lock_irqsave(&mtk_wdt->lock, flags);
	reg = ioread32(wdt_base + WDT_MODE);
	if (wdt_dev->pretimeout)
		reg |= (WDT_MODE_IRQ_EN | WDT_MODE_DUAL_EN);
	else
		reg &= ~(WDT_MODE_IRQ_EN | WDT_MODE_DUAL_EN);
	if (mtk_wdt->disable_wdt_extrst)
		reg &= ~WDT_MODE_EXRST_EN;
	if (mtk_wdt->reset_by_toprgu)
		reg |= WDT_MODE_CNT_SEL;
	reg |= (WDT_MODE_EN | WDT_MODE_KEY);
	iowrite32(reg, wdt_base + WDT_MODE);
	spin_unlock_irqrestore(&mtk_wdt->lock, flags);

	return 0;
}

static int mtk_wdt_set_pretimeout(struct watchdog_device *wdd,
				  unsigned int timeout)
{
	struct mtk_wdt_dev *mtk_wdt = watchdog_get_drvdata(wdd);
	void __iomem *wdt_base = mtk_wdt->wdt_base;
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(&mtk_wdt->lock, flags);
	reg = ioread32(wdt_base + WDT_MODE);
	if (timeout && !wdd->pretimeout) {
		wdd->pretimeout = wdd->timeout / 2;
		reg |= (WDT_MODE_IRQ_EN | WDT_MODE_DUAL_EN);
	} else if (!timeout && wdd->pretimeout) {
		wdd->pretimeout = 0;
		reg &= ~(WDT_MODE_IRQ_EN | WDT_MODE_DUAL_EN);
	} else {
		spin_unlock_irqrestore(&mtk_wdt->lock, flags);
		return 0;
	}

	reg |= WDT_MODE_KEY;
	iowrite32(reg, wdt_base + WDT_MODE);
	spin_unlock_irqrestore(&mtk_wdt->lock, flags);

	return mtk_wdt_set_timeout(wdd, wdd->timeout);
}

static irqreturn_t mtk_wdt_isr(int irq, void *arg)
{
	struct watchdog_device *wdd = arg;

	watchdog_notify_pretimeout(wdd);

	return IRQ_HANDLED;
}

static const char * const mt6797_pl_reason_names[] = {
	"wdt_hw", "wdt_sw", "interrupt", "spm_thermal", "spm", "thermal_direct",
	"debug", "security", "eint", "sysrst",
};

static int mt6797_pl_reason_str(char *buf, size_t size, u32 reason)
{
	int len = 0, i;

	for (i = 0; i < ARRAY_SIZE(mt6797_pl_reason_names); i++)
		if (reason & BIT(i))
			len += scnprintf(buf + len, size - len, "%s%s",
					 len ? "|" : "",
					 mt6797_pl_reason_names[i]);
	if (!len)
		len = scnprintf(buf, size, "unknown(0x%x)", reason);

	return len;
}

static void mtk_wdt_read_boot_reason(struct mtk_wdt_dev *mtk_wdt,
				     const struct mtk_wdt_data *wdt_data)
{
	void __iomem *sram = ioremap(wdt_data->ram_console_phys,
				      wdt_data->ram_console_size);
	u32 sig, off_pl, off_lpl, sz_pl, size = wdt_data->ram_console_size;

	if (!sram)
		return;

	sig = readl(sram);
	off_pl = readl(sram + 4);
	off_lpl = readl(sram + 8);
	sz_pl = readl(sram + 12);

	/* Same bounds discipline as the validated diagnostic: aligned offsets
	 * inside the console, and a record that starts with the reason word.
	 */
	if (sig == MT6797_RAM_CONSOLE_SIG && sz_pl >= sizeof(u32) &&
	    !(off_pl & 3) && off_pl >= MT6797_RAM_CONSOLE_HDR_SIZE &&
	    off_pl <= size - sizeof(u32) &&
	    !(off_lpl & 3) && off_lpl >= MT6797_RAM_CONSOLE_HDR_SIZE &&
	    off_lpl <= size - sizeof(u32)) {
		mtk_wdt->ram_console = true;
		mtk_wdt->pl_reason = readl(sram + off_pl);
		mtk_wdt->prev_pl_reason = readl(sram + off_lpl);
	}
	iounmap(sram);
}

static ssize_t boot_reason_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct mtk_wdt_dev *mtk_wdt = dev_get_drvdata(dev);
	char cur[96], prev[96];

	if (!mtk_wdt->ram_console)
		return sysfs_emit(buf, "unavailable\n");

	mt6797_pl_reason_str(cur, sizeof(cur), mtk_wdt->pl_reason);
	mt6797_pl_reason_str(prev, sizeof(prev), mtk_wdt->prev_pl_reason);

	/* Decoded by the preloader; the raw TOPRGU status is cleared by the
	 * boot chain before Linux starts, so these words are not raw bits.
	 */
	return sysfs_emit(buf,
			  "pl=0x%08x (%s)\nprev_pl=0x%08x (%s)\nbootstatus=0x%x\n",
			  mtk_wdt->pl_reason, cur,
			  mtk_wdt->prev_pl_reason, prev,
			  mtk_wdt->wdt_dev.bootstatus);
}
static DEVICE_ATTR_RO(boot_reason);

static ssize_t registers_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct mtk_wdt_dev *mtk_wdt = dev_get_drvdata(dev);
	void __iomem *base = mtk_wdt->wdt_base;
	struct watchdog_device *wdd = &mtk_wdt->wdt_dev;
	u32 mode, length;
	ssize_t len = 0;

	mode = readl(base + WDT_MODE);
	length = readl(base + WDT_LENGTH);
	len += sysfs_emit_at(buf, len,
			     "mode=0x%08x en=%d irq=%d dual=%d exten=%d auto_restart=%d\n",
			     mode, !!(mode & WDT_MODE_EN),
			     !!(mode & WDT_MODE_IRQ_EN),
			     !!(mode & WDT_MODE_DUAL_EN),
			     !!(mode & WDT_MODE_EXRST_EN),
			     !!(mode & WDT_MODE_AUTO_START));
	/* LENGTH bits [15:5] hold seconds << 6; the low bits are the key. */
	len += sysfs_emit_at(buf, len, "length=0x%08x countdown=%d sec\n",
			     length, (length & 0xffe0) >> 11);
	len += sysfs_emit_at(buf, len,
			     "status=0x%08x interval=0x%08x swsysrst=0x%08x\n",
			     readl(base + WDT_STATUS), readl(base + WDT_INTERVAL),
			     readl(base + WDT_SWSYSRST));
	len += sysfs_emit_at(buf, len, "nonrst=0x%08x nonrst2=0x%08x\n",
			     readl(base + WDT_NONRST_REG),
			     readl(base + WDT_NONRST_REG2));
	len += sysfs_emit_at(buf, len, "req_mode=0x%08x req_irq_en=0x%08x\n",
			     readl(base + WDT_REQ_MODE_REG),
			     readl(base + WDT_REQ_IRQ_EN_REG));
	len += sysfs_emit_at(buf, len, "debug_ctl=0x%08x latch_ctl=0x%08x\n",
			     readl(base + WDT_DEBUG_CTL),
			     readl(base + WDT_LATCH_CTL));
	len += sysfs_emit_at(buf, len,
			     "sw: active=%d hw_running=%d timeout=%d pretimeout=%d bootstatus=0x%x\n",
			     watchdog_active(wdd), watchdog_hw_running(wdd),
			     wdd->timeout, wdd->pretimeout, wdd->bootstatus);

	return len;
}
static DEVICE_ATTR_RO(registers);

static struct attribute *mtk_wdt_vendor_attrs[] = {
	&dev_attr_boot_reason.attr,
	&dev_attr_registers.attr,
	NULL
};
static const struct attribute_group mtk_wdt_vendor_group = {
	.attrs = mtk_wdt_vendor_attrs,
};

static const struct watchdog_info mtk_wdt_info = {
	.identity	= DRV_NAME,
	.options	= WDIOF_SETTIMEOUT |
			  WDIOF_KEEPALIVEPING |
			  WDIOF_MAGICCLOSE,
};

static const struct watchdog_info mtk_wdt_pt_info = {
	.identity	= DRV_NAME,
	.options	= WDIOF_SETTIMEOUT |
			  WDIOF_PRETIMEOUT |
			  WDIOF_KEEPALIVEPING |
			  WDIOF_MAGICCLOSE,
};

static const struct watchdog_ops mtk_wdt_ops = {
	.owner		= THIS_MODULE,
	.start		= mtk_wdt_start,
	.stop		= mtk_wdt_stop,
	.ping		= mtk_wdt_ping,
	.set_timeout	= mtk_wdt_set_timeout,
	.set_pretimeout	= mtk_wdt_set_pretimeout,
	.restart	= mtk_wdt_restart,
};

static int mtk_wdt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_wdt_dev *mtk_wdt;
	const struct mtk_wdt_data *wdt_data;
	int err, irq;
	u32 reg;

	mtk_wdt = devm_kzalloc(dev, sizeof(*mtk_wdt), GFP_KERNEL);
	if (!mtk_wdt)
		return -ENOMEM;
	spin_lock_init(&mtk_wdt->lock);

	platform_set_drvdata(pdev, mtk_wdt);

	mtk_wdt->wdt_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(mtk_wdt->wdt_base))
		return PTR_ERR(mtk_wdt->wdt_base);

	wdt_data = of_device_get_match_data(dev);
	if (wdt_data) {
		mtk_wdt->vendor_reset = wdt_data->vendor_reset;
		if (wdt_data->ram_console_size)
			mtk_wdt_read_boot_reason(mtk_wdt, wdt_data);
	}

	irq = platform_get_irq_optional(pdev, 0);
	if (irq > 0) {
		err = devm_request_irq(&pdev->dev, irq, mtk_wdt_isr, 0, "wdt_bark",
				       &mtk_wdt->wdt_dev);
		if (err)
			return err;

		mtk_wdt->wdt_dev.info = &mtk_wdt_pt_info;
		mtk_wdt->wdt_dev.pretimeout = WDT_MAX_TIMEOUT / 2;
	} else {
		if (irq == -EPROBE_DEFER)
			return -EPROBE_DEFER;

		mtk_wdt->wdt_dev.info = &mtk_wdt_info;
	}

	mtk_wdt->wdt_dev.ops = &mtk_wdt_ops;
	mtk_wdt->wdt_dev.timeout = WDT_MAX_TIMEOUT;
	mtk_wdt->wdt_dev.max_hw_heartbeat_ms = WDT_MAX_TIMEOUT * 1000;
	mtk_wdt->wdt_dev.min_timeout = WDT_MIN_TIMEOUT;
	mtk_wdt->wdt_dev.parent = dev;

	watchdog_init_timeout(&mtk_wdt->wdt_dev, timeout, dev);
	watchdog_set_nowayout(&mtk_wdt->wdt_dev, nowayout);
	watchdog_set_restart_priority(&mtk_wdt->wdt_dev,
				     mtk_wdt->vendor_reset ? 254 : 128);

	watchdog_set_drvdata(&mtk_wdt->wdt_dev, mtk_wdt);

	mtk_wdt_init(&mtk_wdt->wdt_dev);

	if (mtk_wdt->ram_console) {
		char cur[96], prev[96];
		u32 r = mtk_wdt->pl_reason;

		/* Map the decoded preloader reason onto the standard
		 * watchdog boot flags.  A software reboot is a normal boot
		 * and keeps bootstatus clear.
		 */
		if (r & MT6797_PL_WDT_HW)
			mtk_wdt->wdt_dev.bootstatus |= WDIOF_CARDRESET;
		if (r & (MT6797_PL_SPM_THERMAL | MT6797_PL_THERMAL_DIRECT))
			mtk_wdt->wdt_dev.bootstatus |= WDIOF_OVERHEAT;

		mt6797_pl_reason_str(cur, sizeof(cur), r);
		mt6797_pl_reason_str(prev, sizeof(prev),
				     mtk_wdt->prev_pl_reason);
		dev_info(dev,
			 "preloader reboot reason (SRAM ram console, decoded): this boot 0x%x (%s), previous boot 0x%x (%s)\n",
			 r, cur, mtk_wdt->prev_pl_reason, prev);
	} else if (wdt_data && wdt_data->ram_console_size) {
		dev_warn(dev,
			 "preloader SRAM ram console missing or invalid; no boot reason available\n");
	}

	watchdog_stop_on_reboot(&mtk_wdt->wdt_dev);
	err = devm_watchdog_register_device(dev, &mtk_wdt->wdt_dev);
	if (unlikely(err))
		return err;

	/* State at probe: nobody has started the countdown unless the boot
	 * chain left MODE.EN set; report the hardware as it really is.
	 */
	reg = readl(mtk_wdt->wdt_base + WDT_MODE);
	dev_info(dev,
		 "Watchdog registered (timeout=%d sec, nowayout=%d, pretimeout=%d): hardware countdown %s\n",
		 mtk_wdt->wdt_dev.timeout, nowayout, mtk_wdt->wdt_dev.pretimeout,
		 (reg & WDT_MODE_EN) ? "running (MODE.EN=1)" :
				       "not running (MODE.EN=0)");

	if (mtk_wdt->ram_console) {
		err = devm_device_add_group(dev, &mtk_wdt_vendor_group);
		if (err)
			return err;
	}

	wdt_data = of_device_get_match_data(dev);
	if (wdt_data && wdt_data->toprgu_sw_rst_num) {
		err = toprgu_register_reset_controller(pdev,
						       wdt_data->toprgu_sw_rst_num);
		if (err)
			return err;
	}
	if (wdt_data) {
		mtk_wdt->has_swsysrst_en = wdt_data->has_swsysrst_en;
	}

	mtk_wdt->disable_wdt_extrst =
		of_property_read_bool(dev->of_node, "mediatek,disable-extrst");

	mtk_wdt->reset_by_toprgu =
		of_property_read_bool(dev->of_node, "mediatek,reset-by-toprgu");

	return 0;
}

static int mtk_wdt_suspend(struct device *dev)
{
	struct mtk_wdt_dev *mtk_wdt = dev_get_drvdata(dev);

	if (watchdog_active(&mtk_wdt->wdt_dev))
		mtk_wdt_stop(&mtk_wdt->wdt_dev);

	return 0;
}

static int mtk_wdt_resume(struct device *dev)
{
	struct mtk_wdt_dev *mtk_wdt = dev_get_drvdata(dev);

	if (watchdog_active(&mtk_wdt->wdt_dev)) {
		mtk_wdt_start(&mtk_wdt->wdt_dev);
		mtk_wdt_ping(&mtk_wdt->wdt_dev);
	}

	return 0;
}

static const struct of_device_id mtk_wdt_dt_ids[] = {
	{ .compatible = "mediatek,mt2712-wdt", .data = &mt2712_data },
	{ .compatible = "mediatek,mt6589-wdt" },
	{ .compatible = "mediatek,mt6735-wdt", .data = &mt6735_data },
	{ .compatible = "mediatek,mt6797-wdt", .data = &mt6797_data },
	{ .compatible = "mediatek,mt6795-wdt", .data = &mt6795_data },
	{ .compatible = "mediatek,mt7986-wdt", .data = &mt7986_data },
	{ .compatible = "mediatek,mt7988-wdt", .data = &mt7988_data },
	{ .compatible = "mediatek,mt8183-wdt", .data = &mt8183_data },
	{ .compatible = "mediatek,mt8186-wdt", .data = &mt8186_data },
	{ .compatible = "mediatek,mt8188-wdt", .data = &mt8188_data },
	{ .compatible = "mediatek,mt8192-wdt", .data = &mt8192_data },
	{ .compatible = "mediatek,mt8195-wdt", .data = &mt8195_data },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mtk_wdt_dt_ids);

static DEFINE_SIMPLE_DEV_PM_OPS(mtk_wdt_pm_ops,
				mtk_wdt_suspend, mtk_wdt_resume);

static struct platform_driver mtk_wdt_driver = {
	.probe		= mtk_wdt_probe,
	.driver		= {
		.name		= DRV_NAME,
		.pm		= pm_sleep_ptr(&mtk_wdt_pm_ops),
		.of_match_table	= mtk_wdt_dt_ids,
	},
};

module_platform_driver(mtk_wdt_driver);

module_param(timeout, uint, 0);
MODULE_PARM_DESC(timeout, "Watchdog heartbeat in seconds");

module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started (default="
			__MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Matthias Brugger <matthias.bgg@gmail.com>");
MODULE_DESCRIPTION("Mediatek WatchDog Timer Driver");
MODULE_VERSION(DRV_VERSION);
