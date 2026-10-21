// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * MT6797 INFRA/PERI, static MCUSYS, ARMCORE and MEM DCM. Masks and values
 * follow the BSP mt_dcm.c. Linux owns only these always-on registers;
 * CPU sync DCM remains with cpufreq; DRAM PLLs and DDR PHY are not touched.
 */
#include <linux/arm-smccc.h>
#include <linux/clk.h>
#include <linux/cpu.h>
#include <linux/err.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/soc/mediatek/mt6797-armcore-dcm.h>
#include <linux/string.h>

#define TOP_DCMCTL		0x010
#define INFRA_BUS_DCM_CTRL	0x070
#define PERI_BUS_DCM_CTRL	0x074
#define MEM_DCM_CTRL		0x078
#define DFS_MEM_DCM_CTRL		0x07c
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

#define MT6797_SIP_OCP_WRITE	0xc200035e
#define MT6797_SIP_OCP_READ	0xc200035f

#define MEM_CTRL_MASK		(GENMASK(26, 6) | BIT(29) | BIT(31))
#define MEM_ON			(BIT(7) | BIT(8) | (0x10 << 9) | (0x1f << 21))
#define MEM_OFF			((0x2f << 9) | (0x1f << 21) | BIT(31))
#define MEM_RATE_MASK		GENMASK(25, 16)
#define MEM_RATE_DEFAULT		(0x1f << 21)
#define MEM_SELECT		GENMASK(5, 1)
#define MEM_TOGGLE		BIT(0)
#define DFS_RATE_MASK		GENMASK(1, 0)
#define DFS_TOGGLE		BIT(2)
#define DFS_SLOWDOWN		BIT(9)
#define DFS_GATE_MASK		(BIT(3) | BIT(4) | BIT(8) | BIT(9))

/* A probe-only option permits independent real-board qualification. */
static bool enable = true;
module_param(enable, bool, 0444);
MODULE_PARM_DESC(enable, "Apply BSP INFRA/PERI defaults at probe");

static bool mcusys = true;
module_param(mcusys, bool, 0444);
MODULE_PARM_DESC(mcusys, "Apply BSP static MCUSYS defaults at probe");

static bool armcore = true;
module_param(armcore, bool, 0444);
MODULE_PARM_DESC(armcore, "Apply BSP ARMCORE MODE1 at probe");

static bool memory = true;
module_param(memory, bool, 0444);
MODULE_PARM_DESC(memory, "Apply BSP MEM/DFS_MEM defaults at probe");

static const struct {
	u32 reg;
	u32 mask;
} owned[] = {
	{ TOP_DCMCTL, BIT(0) },
	{ INFRA_BUS_DCM_CTRL, INFRA_MASK },
	{ PERI_BUS_DCM_CTRL, PERI_MASK },
	{ INFRA_MISC_2, INFRA_MISC_MASK },
};

/* These fields need latch pulses on restoration; never restore them raw. */
static const struct {
	u32 reg;
	u32 mask;
} memory_owned[] = {
	{ MEM_DCM_CTRL, MEM_CTRL_MASK | MEM_SELECT | MEM_TOGGLE },
	{ DFS_MEM_DCM_CTRL, DFS_RATE_MASK | DFS_TOGGLE | DFS_GATE_MASK },
};

/*
 * BSP dcm_mcusys_little(), in its original order. All four are AO registers.
 * The subsequent BSP sync DCM step belongs to cpufreq, which updates its
 * divider on each frequency transition; do not replace that with a fixed /4.
 */
static const struct {
	const char *name;
	u32 address;
	u32 mask;
} mcu_owned[] = {
	{ "l2",     0x10220648, BIT(0) },
	{ "cci",    0x10220660, BIT(8) },
	{ "fabric", 0x10220668, GENMASK(11, 0) | GENMASK(21, 16) },
	{ "adb400", 0x10220740, GENMASK(7, 0) },
};

struct mt6797_dcm {
	struct device *dev;
	struct regmap *infra;
	struct clk *axi;
	struct device *cpufreq;
	struct mutex lock;
	u32 saved[ARRAY_SIZE(owned)];
	u32 mcu_saved[ARRAY_SIZE(mcu_owned)];
	u32 armcore_saved;
	u32 memory_saved[ARRAY_SIZE(memory_owned)];
	bool memory_touched;
	unsigned long changes;
};

static int mcu_read(u32 address, u32 *value)
{
	struct arm_smccc_res res;

	/* Existing MT6797 firmware whitelists aligned MCUCFG reads/writes. */
	arm_smccc_smc(MT6797_SIP_OCP_READ, address, 0, 0, 0, 0, 0, 0, &res);
	if ((long)res.a0 < 0)
		return -EIO;
	*value = res.a0;
	return 0;
}

static int mcu_update(struct mt6797_dcm *dcm, unsigned int i, u32 value)
{
	struct arm_smccc_res res;
	u32 old, actual, address = mcu_owned[i].address, mask = mcu_owned[i].mask;
	int ret;

	ret = mcu_read(address, &old);
	if (ret)
		return ret;
	value = (old & ~mask) | (value & mask);
	arm_smccc_smc(MT6797_SIP_OCP_WRITE, address, value, 0, 0, 0, 0, 0, &res);
	if (res.a0)
		return -EIO;
	ret = mcu_read(address, &actual);
	if (ret)
		return ret;
	if ((actual & mask) != (value & mask)) {
		dev_err(dcm->dev, "MCUSYS DCM readback mismatch at %#x: %#x expected=%#x\n",
			address, actual, value);
		return -EIO;
	}
	return 0;
}

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

static int memory_write(struct mt6797_dcm *dcm, u32 reg, u32 mask, u32 value)
{
	u32 actual;
	int ret;

	/* Force every BSP write, including repeated values around latch pulses. */
	ret = regmap_write_bits(dcm->infra, reg, mask, value);
	if (ret)
		return ret;
	ret = regmap_read(dcm->infra, reg, &actual);
	if (ret)
		return ret;
	if ((actual & mask) != (value & mask)) {
		dev_err(dcm->dev, "MEM DCM readback mismatch at %#x: %#x mask=%#x expected=%#x\n",
			reg, actual, mask, value & mask);
		return -EIO;
	}
	return 0;
}

static int memory_latch(struct mt6797_dcm *dcm)
{
	int ret;

	ret = memory_write(dcm, MEM_DCM_CTRL, MEM_SELECT, MEM_SELECT);
	if (!ret)
		ret = memory_write(dcm, MEM_DCM_CTRL, MEM_TOGGLE, MEM_TOGGLE);
	if (!ret)
		ret = memory_write(dcm, MEM_DCM_CTRL, MEM_TOGGLE, 0);
	return ret;
}

static int dfs_rate(struct mt6797_dcm *dcm, u32 rate, bool slowdown)
{
	int ret;

	/* BSP dcm_dfs_mem_rate(): free-clock ratio, pipe off, pulse, pipe on. */
	ret = memory_write(dcm, DFS_MEM_DCM_CTRL, DFS_RATE_MASK, rate);
	if (!ret)
		ret = memory_write(dcm, DFS_MEM_DCM_CTRL, DFS_SLOWDOWN, 0);
	if (!ret)
		ret = memory_write(dcm, DFS_MEM_DCM_CTRL, DFS_TOGGLE, DFS_TOGGLE);
	if (!ret)
		ret = memory_write(dcm, DFS_MEM_DCM_CTRL, DFS_TOGGLE, 0);
	if (!ret)
		ret = memory_write(dcm, DFS_MEM_DCM_CTRL, DFS_SLOWDOWN,
				   slowdown ? DFS_SLOWDOWN : 0);
	return ret;
}

static int memory_restore(struct mt6797_dcm *dcm, const u32 *saved)
{
	int ret, dfs_ret;

	/* A failed pulse may have left a latch high; establish a fresh edge. */
	ret = memory_write(dcm, MEM_DCM_CTRL, MEM_TOGGLE, 0);
	if (!ret)
		ret = memory_write(dcm, MEM_DCM_CTRL, MEM_CTRL_MASK, saved[0]);
	if (!ret)
		ret = memory_latch(dcm);
	if (!ret)
		ret = memory_write(dcm, MEM_DCM_CTRL, MEM_SELECT, saved[0]);
	/* Attempt the second domain even if the first domain's restore failed. */
	dfs_ret = memory_write(dcm, DFS_MEM_DCM_CTRL, DFS_TOGGLE, 0);
	if (!dfs_ret)
		dfs_ret = dfs_rate(dcm, saved[1] & DFS_RATE_MASK, saved[1] & DFS_SLOWDOWN);
	if (!dfs_ret)
		dfs_ret = memory_write(dcm, DFS_MEM_DCM_CTRL, DFS_GATE_MASK, saved[1]);
	return ret ? ret : dfs_ret;
}

static int memory_set(struct mt6797_dcm *dcm, bool on)
{
	u32 old[ARRAY_SIZE(memory_owned)];
	int ret;

	for (unsigned int i = 0; i < ARRAY_SIZE(memory_owned); i++) {
		ret = regmap_read(dcm->infra, memory_owned[i].reg, &old[i]);
		if (ret)
			return ret;
	}
	/* Do not reinterpret an unfinished external latch transaction. */
	if ((old[0] & MEM_TOGGLE) || (old[1] & DFS_TOGGLE))
		return -EBUSY;
	dcm->memory_touched = true;
	if (on) {
		/* BSP dcm_mem_preset(): active 1:1, idle 1:32. */
		ret = memory_write(dcm, MEM_DCM_CTRL, MEM_RATE_MASK, MEM_RATE_DEFAULT);
		if (ret)
			goto restore;
	}
	ret = memory_write(dcm, MEM_DCM_CTRL, MEM_CTRL_MASK, on ? MEM_ON : MEM_OFF);
	if (!ret)
		ret = memory_latch(dcm);
	if (!ret && on)
		ret = dfs_rate(dcm, 0, true);
	if (!ret)
		ret = memory_write(dcm, DFS_MEM_DCM_CTRL, DFS_GATE_MASK,
				   on ? DFS_GATE_MASK : 0);
	if (!ret) {
		dcm->changes++;
		return 0;
	}
restore:
	if (memory_restore(dcm, old))
		dev_err(dcm->dev, "MEM/DFS_MEM DCM rollback failed\n");
	return ret;
}

static void dcm_restore(void *arg)
{
	struct mt6797_dcm *dcm = arg;

	cpus_read_lock();
	mutex_lock(&dcm->lock);
	if (dcm->memory_touched && memory_restore(dcm, dcm->memory_saved))
		dev_err(dcm->dev, "failed to restore MEM/DFS_MEM DCM\n");
	if (mt6797_armcore_dcm_access(dcm->cpufreq, &dcm->armcore_saved, true))
		dev_err(dcm->dev, "failed to restore ARMCORE DCM\n");
	for (int i = ARRAY_SIZE(mcu_owned) - 1; i >= 0; i--)
		if (mcu_update(dcm, i, dcm->mcu_saved[i]))
			dev_err(dcm->dev, "failed to restore MCUSYS DCM %s\n", mcu_owned[i].name);
	/* Restore before releasing the clock/syscon suppliers, including errors. */
	for (int i = ARRAY_SIZE(owned) - 1; i >= 0; i--)
		if (dcm_update(dcm, owned[i].reg, owned[i].mask, dcm->saved[i]))
			dev_err(dcm->dev, "failed to restore DCM register %#x\n", owned[i].reg);
	mutex_unlock(&dcm->lock);
	cpus_read_unlock();
}

static int armcore_set(struct mt6797_dcm *dcm, bool on)
{
	u32 value = on ? MT6797_ARMCORE_DCM_MODE1 : 0;
	int ret = mt6797_armcore_dcm_access(dcm->cpufreq, &value, true);

	if (!ret)
		dcm->changes++;
	return ret;
}

/* Caller holds cpus_read_lock and dcm->lock: no hotplug during the sequence. */
static int mcu_set(struct mt6797_dcm *dcm, unsigned int selection, bool on)
{
	u32 old[ARRAY_SIZE(mcu_owned)];
	int ret;

	for (unsigned int i = 0; i < ARRAY_SIZE(mcu_owned); i++) {
		if (!(selection & BIT(i)))
			continue;
		ret = mcu_read(mcu_owned[i].address, &old[i]);
		if (ret)
			return ret;
	}
	for (unsigned int i = 0; i < ARRAY_SIZE(mcu_owned); i++) {
		if (!(selection & BIT(i)))
			continue;
		ret = mcu_update(dcm, i, on ? mcu_owned[i].mask : 0);
		if (ret) {
			/* Include the failed write; it may already have reached hardware. */
			for (int j = i; j >= 0; j--)
				if ((selection & BIT(j)) && mcu_update(dcm, j, old[j]))
					dev_err(dcm->dev, "MCUSYS DCM rollback failed: %s\n",
						mcu_owned[j].name);
			return ret;
		}
	}
	dcm->changes++;
	return 0;
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
	u32 mcu[ARRAY_SIZE(mcu_owned)];
	u32 arm;
	u32 mem[ARRAY_SIZE(memory_owned)];
	int ret;
	ssize_t len;

	cpus_read_lock();
	mutex_lock(&dcm->lock);
	ret = mt6797_armcore_dcm_access(dcm->cpufreq, &arm, false);
	if (ret) {
		len = ret;
		goto out;
	}
	for (unsigned int i = 0; i < ARRAY_SIZE(owned); i++) {
		ret = regmap_read(dcm->infra, owned[i].reg, &value[i]);
		if (ret) {
			len = ret;
			goto out;
		}
	}
	for (unsigned int i = 0; i < ARRAY_SIZE(mcu_owned); i++) {
		ret = mcu_read(mcu_owned[i].address, &mcu[i]);
		if (ret) {
			len = ret;
			goto out;
		}
	}
	for (unsigned int i = 0; i < ARRAY_SIZE(memory_owned); i++) {
		ret = regmap_read(dcm->infra, memory_owned[i].reg, &mem[i]);
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
	for (unsigned int i = 0; i < ARRAY_SIZE(mcu_owned); i++)
		len += sysfs_emit_at(buf, len, "%s=%08x initial_%s=%08x %s_enabled=%u\n",
			mcu_owned[i].name, mcu[i], mcu_owned[i].name, dcm->mcu_saved[i],
			mcu_owned[i].name, (mcu[i] & mcu_owned[i].mask) == mcu_owned[i].mask);
	len += sysfs_emit_at(buf, len, "armcore=%08x initial_armcore=%08x armcore_enabled=%u\n",
		arm, dcm->armcore_saved,
		(arm & MT6797_ARMCORE_DCM_MASK) == MT6797_ARMCORE_DCM_MODE1);
	len += sysfs_emit_at(buf, len, "mem=%08x dfs_mem=%08x mem_enabled=%u\n",
		mem[0], mem[1],
		(mem[0] & memory_owned[0].mask) == (MEM_ON | MEM_SELECT) &&
		(mem[1] & memory_owned[1].mask) == DFS_GATE_MASK);
	len += sysfs_emit_at(buf, len, "initial_mem=%08x initial_dfs_mem=%08x\n",
		dcm->memory_saved[0], dcm->memory_saved[1]);
out:
	mutex_unlock(&dcm->lock);
	cpus_read_unlock();
	return len;
}

static ssize_t state_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct mt6797_dcm *dcm = dev_get_drvdata(dev);
	unsigned int selection = 0;
	bool peri = false, on = false, arm = false, mem = false;
	char command[24];
	int ret;

	if (sysfs_streq(buf, "infra 1")) {
		peri = false; on = true;
	} else if (sysfs_streq(buf, "infra 0")) {
		peri = false; on = false;
	} else if (sysfs_streq(buf, "peri 1")) {
		peri = true; on = true;
	} else if (sysfs_streq(buf, "peri 0")) {
		peri = true; on = false;
	} else if (sysfs_streq(buf, "armcore 1") || sysfs_streq(buf, "armcore 0")) {
		arm = true;
		on = sysfs_streq(buf, "armcore 1");
	} else if (sysfs_streq(buf, "mem 1") || sysfs_streq(buf, "mem 0")) {
		mem = true;
		on = sysfs_streq(buf, "mem 1");
	} else if (sysfs_streq(buf, "mcusys 1") || sysfs_streq(buf, "mcusys 0")) {
		selection = GENMASK(ARRAY_SIZE(mcu_owned) - 1, 0);
		on = sysfs_streq(buf, "mcusys 1");
	} else {
		for (unsigned int i = 0; i < ARRAY_SIZE(mcu_owned); i++) {
			for (unsigned int v = 0; v <= 1; v++) {
				snprintf(command, sizeof(command), "%s %u", mcu_owned[i].name, v);
				if (sysfs_streq(buf, command)) {
					selection = BIT(i);
					on = v;
				}
			}
		}
		if (!selection)
			return -EINVAL;
	}
	cpus_read_lock();
	mutex_lock(&dcm->lock);
	if (mem)
		ret = memory_set(dcm, on);
	else if (arm)
		ret = armcore_set(dcm, on);
	else
		ret = selection ? mcu_set(dcm, selection, on) : dcm_set(dcm, peri, on);
	mutex_unlock(&dcm->lock);
	cpus_read_unlock();
	return ret ? ret : count;
}
static DEVICE_ATTR_RW(state);

static struct attribute *mt6797_dcm_attrs[] = { &dev_attr_state.attr, NULL };
ATTRIBUTE_GROUPS(mt6797_dcm);

static void dcm_put_supplier(void *arg)
{
	put_device(arg);
}

static int mt6797_dcm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct platform_device *supplier;
	struct device_node *node;
	struct mt6797_dcm *dcm;
	int ret;

	dcm = devm_kzalloc(dev, sizeof(*dcm), GFP_KERNEL);
	if (!dcm)
		return -ENOMEM;
	dcm->dev = dev;
	node = of_parse_phandle(dev->of_node, "mediatek,cpufreq", 0);
	if (!node)
		return -EINVAL;
	supplier = of_find_device_by_node(node);
	of_node_put(node);
	if (!supplier)
		return -EPROBE_DEFER;
	ret = devm_add_action_or_reset(dev, dcm_put_supplier, &supplier->dev);
	if (ret)
		return ret;
	if (!device_link_add(dev, &supplier->dev, DL_FLAG_AUTOREMOVE_CONSUMER))
		return -ENOMEM;
	if (!device_is_bound(&supplier->dev))
		return -EPROBE_DEFER;
	dcm->cpufreq = &supplier->dev;
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
	for (unsigned int i = 0; i < ARRAY_SIZE(mcu_owned); i++) {
		ret = mcu_read(mcu_owned[i].address, &dcm->mcu_saved[i]);
		if (ret)
			return dev_err_probe(dev, ret, "MCUSYS secure read unavailable\n");
	}
	for (unsigned int i = 0; i < ARRAY_SIZE(memory_owned); i++) {
		ret = regmap_read(dcm->infra, memory_owned[i].reg, &dcm->memory_saved[i]);
		if (ret)
			return ret;
	}
	cpus_read_lock();
	ret = mt6797_armcore_dcm_access(dcm->cpufreq, &dcm->armcore_saved, false);
	cpus_read_unlock();
	if (ret)
		return dev_err_probe(dev, ret, "ARMCORE DCM read unavailable\n");
	ret = devm_add_action_or_reset(dev, dcm_restore, dcm);
	if (ret)
		return ret;
	cpus_read_lock();
	mutex_lock(&dcm->lock);
	if (enable) {
		ret = dcm_set(dcm, false, true);
		if (!ret)
			ret = dcm_set(dcm, true, true);
	}
	if (!ret && mcusys)
		ret = mcu_set(dcm, GENMASK(ARRAY_SIZE(mcu_owned) - 1, 0), true);
	if (!ret && armcore)
		ret = armcore_set(dcm, true);
	if (!ret && memory)
		ret = memory_set(dcm, true);
	mutex_unlock(&dcm->lock);
	cpus_read_unlock();
	if (ret)
		return dev_err_probe(dev, ret, "failed to apply BSP DCM\n");
	dev_info(dev, "BSP INFRA/PERI=%u static MCUSYS=%u ARMCORE=%u MEM=%u, AXI=%lu Hz\n",
		 enable, mcusys, armcore, memory, clk_get_rate(dcm->axi));
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
MODULE_DESCRIPTION("MT6797 BSP bus, static MCUSYS, ARMCORE and MEM dynamic clock management");
MODULE_LICENSE("GPL");
