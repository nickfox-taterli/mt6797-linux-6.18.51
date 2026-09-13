// SPDX-License-Identifier: GPL-2.0-only
/*
 * MT6797 reboot-to-Fastboot support.
 *
 * The vendor kernel marks Fastboot in the MT6351 RTC spare register before
 * asking the watchdog to reset the SoC. Mainline uses PSCI for the reset,
 * but the boot ROM/LK contract is the same: set RTC_PDN1[13], trigger the
 * PMIC RTC write, then let the normal restart handler reset the machine.
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/regmap.h>

#define MT6351_RTC_BASE		0x4000
#define MT6351_RTC_BBPU		(MT6351_RTC_BASE + 0x0000)
#define MT6351_RTC_BBPU_CBUSY	BIT(6)
#define MT6351_RTC_PROT		(MT6351_RTC_BASE + 0x0036)
#define MT6351_RTC_PROT_UNLOCK1	0x586a
#define MT6351_RTC_PROT_UNLOCK2	0x9136
#define MT6351_RTC_WRTGR	(MT6351_RTC_BASE + 0x003c)
#define MT6351_RTC_PDN1		(MT6351_RTC_BASE + 0x002c)
#define MT6351_RTC_FAST_BOOT	BIT(13)

struct mt6797_fastboot {
	struct device *dev;
	struct regmap *regmap;
	struct notifier_block restart_handler;
};

static int mt6797_rtc_trigger(struct regmap *regmap)
{
	unsigned int value;
	int ret;
	int i;

	ret = regmap_write(regmap, MT6351_RTC_WRTGR, 1);
	if (ret)
		return ret;

	for (i = 0; i < 100; i++) {
		ret = regmap_read(regmap, MT6351_RTC_BBPU, &value);
		if (ret)
			return ret;
		if (!(value & MT6351_RTC_BBPU_CBUSY))
			return 0;
		udelay(10);
	}

	return -ETIMEDOUT;
}

static int mt6797_rtc_write(struct regmap *regmap, unsigned int reg,
				unsigned int value)
{
	int ret;

	ret = regmap_write(regmap, reg, value);
	if (ret)
		return ret;

	return mt6797_rtc_trigger(regmap);
}

static int mt6797_fastboot_mark(struct regmap *regmap)
{
	unsigned int pdn1;
	int ret;

	/* The vendor LK unlocks the protected RTC write interface first. */
	ret = mt6797_rtc_write(regmap, MT6351_RTC_PROT,
				       MT6351_RTC_PROT_UNLOCK1);
	if (ret)
		return ret;

	ret = mt6797_rtc_write(regmap, MT6351_RTC_PROT,
				       MT6351_RTC_PROT_UNLOCK2);
	if (ret)
		return ret;

	/* Keep this identical to the vendor LK read/OR/write sequence. */
	ret = regmap_read(regmap, MT6351_RTC_PDN1, &pdn1);
	if (ret)
		return ret;

	pdn1 |= MT6351_RTC_FAST_BOOT;
	ret = regmap_write(regmap, MT6351_RTC_PDN1, pdn1);
	if (ret)
		return ret;

	return mt6797_rtc_trigger(regmap);
}

static int mt6797_fastboot_restart(struct notifier_block *nb,
					   unsigned long mode, void *cmd)
{
	struct mt6797_fastboot *fastboot =
		container_of(nb, struct mt6797_fastboot, restart_handler);
	int ret;

	if (!cmd || (strcmp(cmd, "bootloader") && strcmp(cmd, "fastboot")))
		return NOTIFY_DONE;

	ret = mt6797_fastboot_mark(fastboot->regmap);
	if (ret)
		dev_emerg(fastboot->dev, "failed to mark Fastboot: %d\n", ret);

	return NOTIFY_DONE;
}

static int mt6797_fastboot_probe(struct platform_device *pdev)
{
	struct mt6797_fastboot *fastboot;
	int ret;

	fastboot = devm_kzalloc(&pdev->dev, sizeof(*fastboot), GFP_KERNEL);
	if (!fastboot)
		return -ENOMEM;

	fastboot->dev = &pdev->dev;
	fastboot->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!fastboot->regmap)
		return -EPROBE_DEFER;

	fastboot->restart_handler.notifier_call = mt6797_fastboot_restart;
	fastboot->restart_handler.priority = 255;
	ret = register_restart_handler(&fastboot->restart_handler);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, fastboot);
	return 0;
}

static const struct of_device_id mt6797_fastboot_of_match[] = {
	{ .compatible = "mediatek,mt6351" },
	{ .compatible = "mediatek,mt6351-pmic" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6797_fastboot_of_match);

static struct platform_driver mt6797_fastboot_driver = {
	.probe = mt6797_fastboot_probe,
	.driver = {
		.name = "mt6797-fastboot",
		.of_match_table = mt6797_fastboot_of_match,
	},
};
module_platform_driver(mt6797_fastboot_driver);

MODULE_DESCRIPTION("MT6797 MT6351 reboot-to-Fastboot support");
MODULE_LICENSE("GPL");
