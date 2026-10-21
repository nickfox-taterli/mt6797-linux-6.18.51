// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * MT6797 INFRA/PERI DCM. Register masks, presets and enable/disable values
 * follow the BSP mt_dcm.c. Linux owns only these always-on bus registers;
 * CPU sync DCM remains with cpufreq, and memory/DDR DCM is not touched.
 */
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/string.h>

#define TOP_DCMCTL		0x010
#define INFRA_BUS_DCM_CTRL	0x070
#define PERI_BUS_DCM_CTRL	0x074
#define INFRA_MISC_2		0xf18

#define INFRA_MASK		(GENMASK(21, 0) | BIT(30))
#define INFRA_ON			(3 | (0x10 << 5) | (1 << 10) | \
				 (0x10 << 15) | BIT(20) | BIT(21) | BIT(30))
#define INFRA_OFF		((0x10 << 5) | (0x10 << 10) | \
				 (0x10 << 15) | BIT(21))
#define INFRA_PRESET_MASK	GENMASK(19, 5)
#define INFRA_PRESET		((0x10 << 5) | (1 << 10) | (0x10 << 15))
#define INFRA_MISC_MASK		(GENMASK(28, 22) | GENMASK(31, 30))

#define PERI_MASK		(GENMASK(29, 0) | BIT(31))
#define PERI_ON			(3 | (0x10 << 5) | (1 << 10) | \
				 (0x10 << 15) | BIT(20) | BIT(21) | \
				 BIT(28) | BIT(29) | BIT(31))
#define PERI_OFF			((0x10 << 5) | (0x10 << 10) | (0x10 << 15))
#define PERI_PRESET_MASK		GENMASK(14, 5)
#define PERI_PRESET		((0x10 << 5) | (1 << 10))

/* A probe-only option permits independent real-board qualification. */
static bool enable = true;
module_param(enable, bool, 0444);
MODULE_PARM_DESC(enable, "Apply BSP INFRA/PERI defaults at probe");

static const struct {
	u32 reg;
	u32 mask;
} owned[] = {
	{ TOP_DCMCTL, BIT(0) },
	{ INFRA_BUS_DCM_CTRL, INFRA_MASK },
	{ PERI_BUS_DCM_CTRL, PERI_MASK },
	{ INFRA_MISC_2, INFRA_MISC_MASK },
};

struct mt6797_dcm {
	struct device *dev;
	struct regmap *infra;
	struct clk *axi;
	struct mutex lock;
	u32 saved[ARRAY_SIZE(owned)];
	unsigned long changes;
};

static int dcm_update(struct mt6797_dcm *dcm, u32 reg, u32 mask, u32 value)
{
	u32 actual;
	int ret;

	ret = regmap_update_bits(dcm->infra, reg, mask, value);
	if (ret)
		return ret;
	/* Flush the write and verify just our fields, preserving other owners. */
	ret = regmap_read(dcm->infra, reg, &actual);
	if (ret)
		return ret;
	if ((actual & mask) != (value & mask)) {
		dev_err(dcm->dev, "DCM readback mismatch at %#x: %#x mask=%#x expected=%#x\n",
			reg, actual, mask, value & mask);
		return -EIO;
	}
	return 0;
}

static void dcm_restore(void *arg)
{
	struct mt6797_dcm *dcm = arg;

	/* Restore before releasing the clock/syscon suppliers, including errors. */
	for (int i = ARRAY_SIZE(owned) - 1; i >= 0; i--)
		if (dcm_update(dcm, owned[i].reg, owned[i].mask, dcm->saved[i]))
			dev_err(dcm->dev, "failed to restore DCM register %#x\n", owned[i].reg);
}

/* Caller serializes preset + configuration, as in the BSP dcm_lock. */
static int dcm_set(struct mt6797_dcm *dcm, bool peri, bool on)
{
	u32 old[ARRAY_SIZE(owned)];
	int ret;

	/* The BSP enables TOP_DCMCTL[0] when the bus leaves the 26 MHz source. */
	if (clk_get_rate(dcm->axi) <= 26000000)
		return -EBUSY;
	for (unsigned int i = 0; i < ARRAY_SIZE(owned); i++) {
		ret = regmap_read(dcm->infra, owned[i].reg, &old[i]);
		if (ret)
			return ret;
	}
	ret = dcm_update(dcm, TOP_DCMCTL, BIT(0), BIT(0));
	if (ret)
		goto restore;
	if (peri) {
		/* BSP dcm_peri_preset(), then dcm_peri(). */
		if (on) {
			ret = dcm_update(dcm, PERI_BUS_DCM_CTRL,
					 PERI_PRESET_MASK, PERI_PRESET);
			if (ret)
				goto restore;
		}
		ret = dcm_update(dcm, PERI_BUS_DCM_CTRL, PERI_MASK,
				 on ? PERI_ON : PERI_OFF);
	} else {
		/* BSP dcm_infra_preset(), then dcm_infra(). */
		if (on) {
			ret = dcm_update(dcm, INFRA_BUS_DCM_CTRL,
					 INFRA_PRESET_MASK, INFRA_PRESET);
			if (ret)
				goto restore;
		}
		ret = dcm_update(dcm, INFRA_BUS_DCM_CTRL, INFRA_MASK,
				 on ? INFRA_ON : INFRA_OFF);
		if (ret)
			goto restore;
		/* BSP deliberately leaves P2P_RX_CLK_ON alone (DRAM access WA). */
		ret = dcm_update(dcm, INFRA_MISC_2, INFRA_MISC_MASK,
				 on ? 0 : INFRA_MISC_MASK);
	}
	if (!ret) {
		dcm->changes++;
		return 0;
	}
restore:
	for (int i = ARRAY_SIZE(owned) - 1; i >= 0; i--)
		if (dcm_update(dcm, owned[i].reg, owned[i].mask, old[i]))
			dev_err(dcm->dev, "DCM transition rollback failed at %#x\n", owned[i].reg);
	return ret;
}

static ssize_t state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mt6797_dcm *dcm = dev_get_drvdata(dev);
	u32 value[ARRAY_SIZE(owned)];
	int ret;
	ssize_t len;

	mutex_lock(&dcm->lock);
	for (unsigned int i = 0; i < ARRAY_SIZE(owned); i++) {
		ret = regmap_read(dcm->infra, owned[i].reg, &value[i]);
		if (ret) {
			len = ret;
			goto out;
		}
	}
	len = sysfs_emit(buf, "axi_hz=%lu changes=%lu\n"
			 "top=%08x infra=%08x peri=%08x misc2=%08x\n"
			 "initial_top=%08x initial_infra=%08x initial_peri=%08x initial_misc2=%08x\n"
			 "infra_enabled=%u peri_enabled=%u\n",
			 clk_get_rate(dcm->axi), dcm->changes,
			 value[0], value[1], value[2], value[3],
			 dcm->saved[0], dcm->saved[1], dcm->saved[2], dcm->saved[3],
			 !!(value[0] & BIT(0)) && (value[1] & INFRA_MASK) == INFRA_ON &&
			 !(value[3] & INFRA_MISC_MASK),
			 !!(value[0] & BIT(0)) && (value[2] & PERI_MASK) == PERI_ON);
out:
	mutex_unlock(&dcm->lock);
	return len;
}

static ssize_t state_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct mt6797_dcm *dcm = dev_get_drvdata(dev);
	bool peri, on;
	int ret;

	if (sysfs_streq(buf, "infra 1")) {
		peri = false; on = true;
	} else if (sysfs_streq(buf, "infra 0")) {
		peri = false; on = false;
	} else if (sysfs_streq(buf, "peri 1")) {
		peri = true; on = true;
	} else if (sysfs_streq(buf, "peri 0")) {
		peri = true; on = false;
	} else {
		return -EINVAL;
	}
	mutex_lock(&dcm->lock);
	ret = dcm_set(dcm, peri, on);
	mutex_unlock(&dcm->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_RW(state);

static struct attribute *mt6797_dcm_attrs[] = { &dev_attr_state.attr, NULL };
ATTRIBUTE_GROUPS(mt6797_dcm);

static int mt6797_dcm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mt6797_dcm *dcm;
	int ret;

	dcm = devm_kzalloc(dev, sizeof(*dcm), GFP_KERNEL);
	if (!dcm)
		return -ENOMEM;
	dcm->dev = dev;
	dcm->infra = syscon_regmap_lookup_by_phandle(dev->of_node, "mediatek,infracfg");
	if (IS_ERR(dcm->infra))
		return dev_err_probe(dev, PTR_ERR(dcm->infra), "missing INFRACFG\n");
	dcm->axi = devm_clk_get(dev, "axi");
	if (IS_ERR(dcm->axi))
		return dev_err_probe(dev, PTR_ERR(dcm->axi), "missing AXI clock\n");
	mutex_init(&dcm->lock);
	platform_set_drvdata(pdev, dcm);
	for (unsigned int i = 0; i < ARRAY_SIZE(owned); i++) {
		ret = regmap_read(dcm->infra, owned[i].reg, &dcm->saved[i]);
		if (ret)
			return ret;
	}
	ret = devm_add_action_or_reset(dev, dcm_restore, dcm);
	if (ret)
		return ret;
	if (enable) {
		ret = dcm_set(dcm, false, true);
		if (!ret)
			ret = dcm_set(dcm, true, true);
		if (ret)
			return dev_err_probe(dev, ret, "failed to apply BSP bus DCM\n");
	}
	dev_info(dev, "BSP INFRA/PERI DCM %s, AXI=%lu Hz\n",
		 enable ? "enabled" : "probe only", clk_get_rate(dcm->axi));
	return 0;
}

static const struct of_device_id mt6797_dcm_of_match[] = {
	{ .compatible = "mediatek,mt6797-dcm" }, { }
};
MODULE_DEVICE_TABLE(of, mt6797_dcm_of_match);

static struct platform_driver mt6797_dcm_driver = {
	.probe = mt6797_dcm_probe,
	.driver = {
		.name = "mt6797-dcm",
		.of_match_table = mt6797_dcm_of_match,
		.dev_groups = mt6797_dcm_groups,
	},
};
module_platform_driver(mt6797_dcm_driver);
MODULE_DESCRIPTION("MT6797 BSP INFRA/PERI dynamic clock management");
MODULE_LICENSE("GPL");
