// SPDX-License-Identifier: GPL-2.0-only
/*
 * MT6797 CPU DVFS
 *
 * The 6.18 clock driver does not describe the MCUSYS ARM PLLs used by this
 * SoC.  This driver therefore keeps the small, board-independent part of the
 * vendor implementation here: the FY OPP tables are in DT, while the PLL
 * divider/clock-switch sequence remains explicit.  The Cortex-A72 PLL is
 * changed through the ATF IDVFS SMC, as required by the vendor firmware.
 */

#include <linux/arm-smccc.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/limits.h>
#include <linux/math64.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/regulator/consumer.h>

#define MT6797_SIP_IDVFS_PLL_SET_FREQ	0xC20003B8
#define MT6797_SIP_IDVFS_SRAM_LDO_SET	0xC20003BF
#define MT6797_SIP_OCP_READ		0xC200035F

#define MT6797_PLL_LL_CON1		0x204
#define MT6797_PLL_L_CON1		0x214
#define MT6797_PLL_CCI_CON1		0x224
#define MT6797_PLL_MUXSEL		0x270
#define MT6797_PLL_CKDIV		0x274
#define MT6797_TOP_CLK_MISC_CFG_0	0x104

#define MT6797_SEMA_POWERON_CONFIG	0x000
#define MT6797_SEMA3_M0			0x440
#define MT6797_SEMA_PROJECT_CODE		0x0b16
#define MT6797_SEMA_TIMEOUT_US		2000

#define MT6797_B_PLL_CON0		0x102224a0
#define MT6797_B_PLL_CON1		0x102224a4
#define MT6797_B_SRAM_LDO		0x102222b0

#define MT6797_CPULDO_CTRL_0		0xf98
#define MT6797_CPULDO_CTRL_1		0xf9c
#define MT6797_CPULDO_CTRL_2		0xfa0
#define MT6797_VSRAM_MIN_UV		1000000
#define MT6797_VSRAM_MAX_UV		1200000
#define MT6797_VSRAM_STEP_UV		25000
#define MT6797_VSRAM_MARGIN_UV		100000
#define MT6797_VSRAM_STEP_LIMIT_UV	275000

struct mt6797_freq_method {
	unsigned int khz;
	unsigned int pos_div;
	unsigned int clk_div;
};

struct mt6797_cluster {
	unsigned int id;
	unsigned int first_cpu;
	struct cpumask cpus;
	struct device *cpu_dev;
	struct regulator *vproc;
	const struct mt6797_freq_method *methods;
	unsigned int method_count;
	struct cpufreq_frequency_table *freq_table;
	unsigned int current_khz;
	unsigned int current_uv;
	bool vproc_owner;
	bool opp_added;
};

struct mt6797_cpufreq {
	struct device *dev;
	void __iomem *mcumixed;
	void __iomem *topckgen;
	void __iomem *semaphore;
	void __iomem *infracfg;
	struct mutex lock;
	struct mt6797_cluster cluster[3];
	struct mt6797_cluster cci;
};

static struct mt6797_cpufreq *mt6797_data;

#define MT6797_METHOD(_khz, _pos, _clk) \
	{ .khz = (_khz), .pos_div = (_pos), .clk_div = (_clk) }

static const struct mt6797_freq_method mt6797_ll_methods[] = {
	MT6797_METHOD(1391000, 1, 1), MT6797_METHOD(1339000, 1, 1),
	MT6797_METHOD(1287000, 1, 1), MT6797_METHOD(1222000, 1, 1),
	MT6797_METHOD(1118000, 2, 1), MT6797_METHOD(1066000, 2, 1),
	MT6797_METHOD( 949000, 2, 1), MT6797_METHOD( 897000, 2, 1),
	MT6797_METHOD( 806000, 2, 1), MT6797_METHOD( 715000, 2, 1),
	MT6797_METHOD( 624000, 2, 1), MT6797_METHOD( 559000, 2, 1),
	MT6797_METHOD( 481000, 2, 2), MT6797_METHOD( 416000, 2, 2),
	MT6797_METHOD( 338000, 2, 2), MT6797_METHOD( 221000, 2, 4),
};

static const struct mt6797_freq_method mt6797_l_methods[] = {
	MT6797_METHOD(1846000, 1, 1), MT6797_METHOD(1781000, 1, 1),
	MT6797_METHOD(1703000, 1, 1), MT6797_METHOD(1625000, 1, 1),
	MT6797_METHOD(1495000, 1, 1), MT6797_METHOD(1417000, 1, 1),
	MT6797_METHOD(1274000, 1, 1), MT6797_METHOD(1209000, 1, 1),
	MT6797_METHOD(1079000, 2, 1), MT6797_METHOD( 962000, 2, 1),
	MT6797_METHOD( 832000, 2, 1), MT6797_METHOD( 741000, 2, 1),
	MT6797_METHOD( 650000, 2, 2), MT6797_METHOD( 559000, 2, 2),
	MT6797_METHOD( 468000, 2, 2), MT6797_METHOD( 325000, 2, 2),
};

static const struct mt6797_freq_method mt6797_b_methods[] = {
	MT6797_METHOD(2249000, 1, 1), MT6797_METHOD(2197000, 1, 1),
	MT6797_METHOD(2171000, 1, 1), MT6797_METHOD(2119000, 1, 1),
	MT6797_METHOD(2093000, 1, 1), MT6797_METHOD(1989000, 1, 1),
	MT6797_METHOD(1781000, 1, 1), MT6797_METHOD(1677000, 1, 1),
	MT6797_METHOD(1495000, 1, 1), MT6797_METHOD(1378000, 1, 1),
	MT6797_METHOD(1248000, 1, 1), MT6797_METHOD(1131000, 2, 1),
	MT6797_METHOD(1001000, 2, 1), MT6797_METHOD( 845000, 2, 1),
	MT6797_METHOD( 676000, 2, 1), MT6797_METHOD( 338000, 2, 2),
};

static const struct mt6797_freq_method mt6797_cci_methods[] = {
	MT6797_METHOD(923000, 2, 1), MT6797_METHOD(884000, 2, 1),
	MT6797_METHOD(858000, 2, 1), MT6797_METHOD(819000, 2, 1),
	MT6797_METHOD(754000, 2, 1), MT6797_METHOD(715000, 2, 1),
	MT6797_METHOD(637000, 2, 1), MT6797_METHOD(611000, 2, 1),
	MT6797_METHOD(533000, 2, 2), MT6797_METHOD(481000, 2, 2),
	MT6797_METHOD(416000, 2, 2), MT6797_METHOD(377000, 2, 2),
	MT6797_METHOD(325000, 2, 2), MT6797_METHOD(286000, 2, 2),
	MT6797_METHOD(234000, 2, 4), MT6797_METHOD(169000, 2, 4),
};

/* The FY CCI OPPs share VPROC1 with LL and L. */
static const unsigned int mt6797_cci_voltages[] = {
	1200000, 1180000, 1150000, 1130000,
	1090000, 1070000, 1020000, 1000000,
	 970000,  940000,  900000,  880000,
	 850000,  830000,  800000,  780000,
};

static struct mt6797_cluster *mt6797_cluster_for_cpu(unsigned int cpu)
{
	unsigned int i;

	if (!mt6797_data)
		return NULL;

	for (i = 0; i < ARRAY_SIZE(mt6797_data->cluster); i++)
		if (cpumask_test_cpu(cpu, &mt6797_data->cluster[i].cpus))
			return &mt6797_data->cluster[i];

	return NULL;
}

static const struct mt6797_freq_method *
mt6797_cci_target(struct mt6797_cpufreq *data,
			  const struct mt6797_cluster *changed,
			  unsigned int target_khz, unsigned int target_uv,
			  unsigned int *vproc1_uv)
{
	unsigned int max_khz = 0, required_uv = 0, requested_khz;
	int i;

	for (i = 0; i < ARRAY_SIZE(data->cluster); i++) {
		const struct mt6797_cluster *cluster = &data->cluster[i];
		unsigned int khz = cluster == changed ? target_khz :
			cluster->current_khz;
		unsigned int uv = cluster == changed ? target_uv :
			cluster->current_uv;

		max_khz = max(max_khz, khz);
		if (i < 2)
			required_uv = max(required_uv, uv);
	}

	requested_khz = min(max_khz / 2, mt6797_cci_methods[0].khz);
	for (i = ARRAY_SIZE(mt6797_cci_methods) - 1; i > 0; i--)
		if (requested_khz <= mt6797_cci_methods[i].khz)
			break;
	required_uv = max(required_uv, mt6797_cci_voltages[i]);
	*vproc1_uv = required_uv;

	/* BSP chooses the fastest CCI OPP allowed by the shared voltage. */
	for (i = 0; i < ARRAY_SIZE(mt6797_cci_methods) - 1; i++)
		if (mt6797_cci_voltages[i] <= required_uv)
			break;
	return &mt6797_cci_methods[i];
}

static const struct mt6797_freq_method *
mt6797_method_for_rate(const struct mt6797_cluster *cluster, unsigned int khz)
{
	const struct mt6797_freq_method *best = &cluster->methods[0];
	unsigned int best_delta = UINT_MAX;
	unsigned int i;

	for (i = 0; i < cluster->method_count; i++) {
		unsigned int rate = cluster->methods[i].khz;
		unsigned int delta = rate > khz ? rate - khz : khz - rate;

		if (rate == khz)
			return &cluster->methods[i];
		if (delta < best_delta) {
			best = &cluster->methods[i];
			best_delta = delta;
		}
	}

	return best;
}

static unsigned int mt6797_ckdiv_selector(unsigned int clk_div)
{
	switch (clk_div) {
	case 1:
		return 8;
	case 2:
		return 10;
	case 4:
		return 11;
	default:
		return 8;
	}
}

static unsigned int mt6797_ckdiv_ratio(unsigned int selector)
{
	switch (selector) {
	case 10:
		return 2;
	case 11:
		return 4;
	default:
		return 1;
	}
}

static unsigned int mt6797_ckdiv_shift(const struct mt6797_cluster *cluster)
{
	return cluster->id == 0 ? 5 : cluster->id == 1 ? 10 :
		cluster->id == 3 ? 15 : 0;
}

static unsigned int mt6797_mux_shift(const struct mt6797_cluster *cluster)
{
	return cluster->id == 0 ? 2 : cluster->id == 1 ? 4 :
		cluster->id == 3 ? 6 : 0;
}

static void mt6797_clock_switch(struct mt6797_cpufreq *data,
				const struct mt6797_cluster *cluster,
				unsigned int source)
{
	u32 value;
	u32 shift = mt6797_mux_shift(cluster);

	value = readl_relaxed(data->topckgen + MT6797_TOP_CLK_MISC_CFG_0);
	value &= ~GENMASK(5, 4);
	if (source >= 2)
		value |= FIELD_PREP(GENMASK(5, 4), 3);
	writel_relaxed(value, data->topckgen + MT6797_TOP_CLK_MISC_CFG_0);

	value = readl_relaxed(data->mcumixed + MT6797_PLL_MUXSEL);
	value &= ~(0x3U << shift);
	value |= source << shift;
	writel_relaxed(value, data->mcumixed + MT6797_PLL_MUXSEL);
	readl_relaxed(data->mcumixed + MT6797_PLL_MUXSEL);
}

static int mt6797_hw_semaphore_get(struct mt6797_cpufreq *data)
{
	unsigned int i;

	writel_relaxed((MT6797_SEMA_PROJECT_CODE << 16) | BIT(0),
		       data->semaphore + MT6797_SEMA_POWERON_CONFIG);
	for (i = 0; i < MT6797_SEMA_TIMEOUT_US / 10; i++) {
		writel_relaxed(1, data->semaphore + MT6797_SEMA3_M0);
		if (readl_relaxed(data->semaphore + MT6797_SEMA3_M0) & BIT(0))
			return 0;
		udelay(10);
	}

	return -EBUSY;
}

static void mt6797_hw_semaphore_put(struct mt6797_cpufreq *data)
{
	writel_relaxed((MT6797_SEMA_PROJECT_CODE << 16) | BIT(0),
		       data->semaphore + MT6797_SEMA_POWERON_CONFIG);
	if (readl_relaxed(data->semaphore + MT6797_SEMA3_M0) & BIT(0))
		writel_relaxed(1, data->semaphore + MT6797_SEMA3_M0);
}

static unsigned int mt6797_calc_freq(u32 con1, u32 ckdiv)
{
	u64 freq;
	unsigned int pos_div = 1U << FIELD_GET(GENMASK(26, 24), con1);

	freq = ((u64)(con1 & GENMASK(20, 0)) * 26 >> 14) * 1000;
	freq = div_u64(freq, pos_div);
	freq = div_u64(freq, mt6797_ckdiv_ratio(ckdiv));

	return min_t(u64, freq, UINT_MAX);
}

static int mt6797_smc_read(u32 address, u32 *value)
{
	struct arm_smccc_res res;

	arm_smccc_smc(MT6797_SIP_OCP_READ, address, 0, 0, 0, 0, 0, 0, &res);
	if ((long)res.a0 < 0)
		return (int)res.a0;

	*value = (u32)res.a0;
	return 0;
}

static int mt6797_vsram_decode(u32 selector, unsigned int *uv)
{
	if (selector > 15)
		return -EINVAL;

	switch (selector) {
	case 0:
		*uv = 1050000;
		break;
	case 1:
		*uv = 600000;
		break;
	case 2:
		*uv = 700000;
		break;
	default:
		*uv = 900000 + (selector - 3) * MT6797_VSRAM_STEP_UV;
	}
	return 0;
}

static int mt6797_vsram_read(struct mt6797_cpufreq *data,
			      const struct mt6797_cluster *cluster,
			      unsigned int *uv)
{
	u32 value, mirror;
	int ret;

	if (cluster->id == 2) {
		ret = mt6797_smc_read(MT6797_B_SRAM_LDO, &value);
		if (ret)
			return ret;
		return mt6797_vsram_decode(value & 0xf, uv);
	}

	value = readl_relaxed(data->infracfg + MT6797_CPULDO_CTRL_1);
	mirror = readl_relaxed(data->infracfg + MT6797_CPULDO_CTRL_2);
	if (value != mirror || (value & 0xf0f0f0f0) ||
	    (value & 0xff) != ((value >> 8) & 0xff) ||
	    (value & 0xff) != ((value >> 16) & 0xff) ||
	    (value & 0xff) != ((value >> 24) & 0xff))
		return -EIO;

	return mt6797_vsram_decode(value & 0xf, uv);
}

static int mt6797_vsram_write(struct mt6797_cpufreq *data,
			       const struct mt6797_cluster *cluster,
			       unsigned int uv)
{
	struct arm_smccc_res res;
	unsigned int actual, old_uv, settle_us;
	u32 selector, value;
	int ret;

	if (uv < MT6797_VSRAM_MIN_UV || uv > MT6797_VSRAM_MAX_UV ||
	    (uv - 900000) % MT6797_VSRAM_STEP_UV)
		return -EINVAL;
	ret = mt6797_vsram_read(data, cluster, &old_uv);
	if (ret)
		return ret;

	if (cluster->id == 2) {
		/* The MT6797 BL31 API accepts mV * 100, as in the BSP. */
		arm_smccc_smc(MT6797_SIP_IDVFS_SRAM_LDO_SET, uv / 10, 0, 0,
			      0, 0, 0, 0, &res);
		if ((long)res.a0 < 0)
			return (int)res.a0;
	} else {
		selector = (uv - 900000) / MT6797_VSRAM_STEP_UV + 3;
		value = selector * 0x01010101;
		writel_relaxed(readl_relaxed(data->infracfg + MT6797_CPULDO_CTRL_0) |
			       0xff, data->infracfg + MT6797_CPULDO_CTRL_0);
		writel_relaxed(value, data->infracfg + MT6797_CPULDO_CTRL_1);
		writel_relaxed(value, data->infracfg + MT6797_CPULDO_CTRL_2);
		readl_relaxed(data->infracfg + MT6797_CPULDO_CTRL_2);
	}

	/* The BSP's SRAM LDO ramp needs longer settling on a downward step. */
	if (uv >= old_uv)
		settle_us = 5 + DIV_ROUND_UP(uv - old_uv, 12500U);
	else
		settle_us = 5 + DIV_ROUND_UP(old_uv - uv, 3125U);
	udelay(max(settle_us, 25U));
	ret = mt6797_vsram_read(data, cluster, &actual);
	if (ret)
		return ret;
	return actual == uv ? 0 : -EIO;
}

static unsigned int mt6797_vsram_target(unsigned int vproc_uv)
{
	unsigned int uv = clamp(vproc_uv + MT6797_VSRAM_MARGIN_UV,
				MT6797_VSRAM_MIN_UV, MT6797_VSRAM_MAX_UV);

	return DIV_ROUND_UP(uv, MT6797_VSRAM_STEP_UV) *
		MT6797_VSRAM_STEP_UV;
}

/* BSP ordering: raise SRAM before VPROC; lower VPROC before SRAM. */
static int mt6797_set_voltages(struct mt6797_cpufreq *data,
			       struct mt6797_cluster *cluster,
			       unsigned int target_uv)
{
	unsigned int vsram_uv, target_vsram, next_uv;
	int vproc_uv, ret;

	if (target_uv < 780000 || target_uv > MT6797_VSRAM_MAX_UV)
		return -EINVAL;

	vproc_uv = regulator_get_voltage(cluster->vproc);
	if (vproc_uv < 0)
		return vproc_uv;
	ret = mt6797_vsram_read(data, cluster, &vsram_uv);
	if (ret)
		return ret;
	target_vsram = mt6797_vsram_target(target_uv);

	while (vproc_uv < target_uv) {
		if (vsram_uv < target_vsram) {
			next_uv = min(target_vsram,
				      (vproc_uv + MT6797_VSRAM_STEP_LIMIT_UV) /
					      MT6797_VSRAM_STEP_UV * MT6797_VSRAM_STEP_UV);
			if (next_uv > vsram_uv) {
				ret = mt6797_vsram_write(data, cluster, next_uv);
				if (ret)
					return ret;
				vsram_uv = next_uv;
			}
		}

		next_uv = min(target_uv, (vsram_uv -
					(vsram_uv == target_vsram ? 0 :
					 MT6797_VSRAM_MARGIN_UV)) / 10000U * 10000U);
		if (next_uv <= vproc_uv)
			return -ERANGE;
		ret = regulator_set_voltage(cluster->vproc, next_uv, next_uv);
		if (ret)
			return ret;
		vproc_uv = regulator_get_voltage(cluster->vproc);
		if (vproc_uv != next_uv)
			return -EIO;
		udelay(25);
	}

	while (vproc_uv > target_uv) {
		next_uv = max(target_uv,
			      DIV_ROUND_UP(vsram_uv - MT6797_VSRAM_STEP_LIMIT_UV,
					   10000U) * 10000U);
		if (next_uv >= vproc_uv)
			return -ERANGE;
		ret = regulator_set_voltage(cluster->vproc, next_uv, next_uv);
		if (ret)
			return ret;
		vproc_uv = regulator_get_voltage(cluster->vproc);
		if (vproc_uv != next_uv)
			return -EIO;
		udelay(25);

		next_uv = mt6797_vsram_target(vproc_uv);
		if (next_uv < vsram_uv) {
			ret = mt6797_vsram_write(data, cluster, next_uv);
			if (ret)
				return ret;
			vsram_uv = next_uv;
		}
	}

	if (vsram_uv != target_vsram)
		return mt6797_vsram_write(data, cluster, target_vsram);
	return 0;
}

static unsigned int mt6797_read_rate(struct mt6797_cpufreq *data,
				      const struct mt6797_cluster *cluster)
{
	u32 ckdiv = readl_relaxed(data->mcumixed + MT6797_PLL_CKDIV);

	ckdiv >>= mt6797_ckdiv_shift(cluster);
	ckdiv &= 0x1f;

	if (cluster->id == 2) {
		u32 con0, con1;
		u64 freq;
		unsigned int pos_div;

		if (mt6797_smc_read(MT6797_B_PLL_CON0, &con0) ||
		    mt6797_smc_read(MT6797_B_PLL_CON1, &con1))
			return cluster->current_khz;

		pos_div = 1U << ((con0 >> 12) & 0x7);
		freq = ((u64)(con1 & 0x7fffffff) * 26 * 1000) >> 24;
		freq = div_u64(freq, pos_div);
		freq = div_u64(freq, mt6797_ckdiv_ratio(ckdiv));
		return min_t(u64, freq, UINT_MAX);
	}

	return mt6797_calc_freq(readl_relaxed(data->mcumixed +
					    (cluster->id == 0 ? MT6797_PLL_LL_CON1 :
					     cluster->id == 1 ? MT6797_PLL_L_CON1 :
					     MT6797_PLL_CCI_CON1)), ckdiv);
}

static void mt6797_set_clkdiv(struct mt6797_cpufreq *data,
				      const struct mt6797_cluster *cluster,
				      unsigned int clk_div)
{
	u32 shift = mt6797_ckdiv_shift(cluster);
	u32 selector = mt6797_ckdiv_selector(clk_div);
	u32 value;

	value = readl_relaxed(data->mcumixed + MT6797_PLL_CKDIV);
	value &= ~(0x1fU << shift);
	value |= selector << shift;
	writel_relaxed(value, data->mcumixed + MT6797_PLL_CKDIV);
	udelay(2);

	value = readl_relaxed(data->mcumixed + MT6797_PLL_CKDIV);
	if (((value >> shift) & 0x1f) != selector) {
		value &= ~(0x1fU << shift);
		value |= selector << shift;
		writel_relaxed(value, data->mcumixed + MT6797_PLL_CKDIV);
		udelay(2);
	}
}

static void mt6797_set_posdiv(struct mt6797_cpufreq *data,
				      const struct mt6797_cluster *cluster,
				      unsigned int pos_div)
{
	u32 con1_offset = cluster->id == 0 ? MT6797_PLL_LL_CON1 :
			  cluster->id == 1 ? MT6797_PLL_L_CON1 :
			  MT6797_PLL_CCI_CON1;
	u32 value;

	mt6797_clock_switch(data, cluster, 2);
	value = readl_relaxed(data->mcumixed + con1_offset);
	value &= ~GENMASK(26, 24);
	value |= ilog2(pos_div) << 24;
	writel_relaxed(value | BIT(31), data->mcumixed + con1_offset);
	udelay(2);
	mt6797_clock_switch(data, cluster, 1);
}

static int mt6797_set_direct_pll(struct mt6797_cpufreq *data,
					const struct mt6797_cluster *cluster,
					const struct mt6797_freq_method *method)
{
	u32 con1_offset = cluster->id == 0 ? MT6797_PLL_LL_CON1 :
			  cluster->id == 1 ? MT6797_PLL_L_CON1 :
			  MT6797_PLL_CCI_CON1;
	u32 value;
	u64 vco = (u64)method->khz * method->pos_div * method->clk_div;
	u32 dds = ((vco / 1000) << 14) / 26;

	mt6797_clock_switch(data, cluster, 2);
	value = readl_relaxed(data->mcumixed + con1_offset);
	value &= ~(GENMASK(26, 24) | GENMASK(20, 0));
	value |= ilog2(method->pos_div) << 24;
	value |= dds & GENMASK(20, 0);
	writel_relaxed(value | BIT(31), data->mcumixed + con1_offset);
	udelay(20);
	mt6797_clock_switch(data, cluster, 1);

	return 0;
}

static int mt6797_set_big_pll(const struct mt6797_freq_method *method)
{
	struct arm_smccc_res res;
	u64 freq_mhz = (u64)method->khz * method->clk_div / 1000;

	if (freq_mhz < 250 || freq_mhz > 3000)
		return -EINVAL;

	arm_smccc_smc(MT6797_SIP_IDVFS_PLL_SET_FREQ, freq_mhz, 0, 0,
			      0, 0, 0, 0, &res);
	return (long)res.a0 < 0 ? (int)res.a0 : 0;
}

static int mt6797_set_rate_locked(struct mt6797_cpufreq *data,
					 struct mt6797_cluster *cluster,
					 const struct mt6797_freq_method *old,
					 const struct mt6797_freq_method *target)
{
	int ret;

	if (old->khz == target->khz &&
	    (cluster->id != 3 ||
	     abs((int)mt6797_read_rate(data, cluster) - (int)target->khz) < 1000))
		return 0;

	ret = mt6797_hw_semaphore_get(data);
	if (ret)
		return ret;

	/* This is the ordering used by the vendor set_cur_freq(). */
	if (cluster->id != 2 && old->pos_div < target->pos_div) {
		mt6797_set_posdiv(data, cluster, target->pos_div);
	}
	if (old->clk_div < target->clk_div) {
		mt6797_set_clkdiv(data, cluster, target->clk_div);
	}

	if (cluster->id == 2) {
		mt6797_clock_switch(data, cluster, 2);
		ret = mt6797_set_big_pll(target);
		udelay(20);
		mt6797_clock_switch(data, cluster, 1);
	} else {
		ret = mt6797_set_direct_pll(data, cluster, target);
	}
	if (!ret && old->clk_div > target->clk_div)
		mt6797_set_clkdiv(data, cluster, target->clk_div);
	if (!ret && cluster->id != 2 && old->pos_div > target->pos_div)
		mt6797_set_posdiv(data, cluster, target->pos_div);

	mt6797_hw_semaphore_put(data);
	return ret;
}

static unsigned int mt6797_cpufreq_get(unsigned int cpu)
{
	struct mt6797_cluster *cluster = mt6797_cluster_for_cpu(cpu);
	unsigned int rate;

	if (!cluster)
		return 0;

	mutex_lock(&mt6797_data->lock);
	rate = mt6797_read_rate(mt6797_data, cluster);
	mutex_unlock(&mt6797_data->lock);

	return rate;
}

static int mt6797_cpufreq_target_index(struct cpufreq_policy *policy,
					       unsigned int index)
{
	struct mt6797_cluster *cluster = policy->driver_data;
	struct mt6797_cpufreq *data = mt6797_data;
	const struct mt6797_freq_method *old, *target, *old_cci, *target_cci;
	struct dev_pm_opp *opp;
	unsigned long target_hz;
	unsigned int target_uv, vproc1_uv, vproc2_uv;
	int old_vproc1, old_vproc2, ret;

	if (!cluster || !policy->freq_table)
		return -EINVAL;

	target_hz = (unsigned long)policy->freq_table[index].frequency * 1000;
	opp = dev_pm_opp_find_freq_exact(cluster->cpu_dev, target_hz, true);
	if (IS_ERR(opp))
		return PTR_ERR(opp);
	target_uv = dev_pm_opp_get_voltage(opp);
	dev_pm_opp_put(opp);
	if (!target_uv)
		return -EINVAL;

	target = mt6797_method_for_rate(cluster, target_hz / 1000);
	mutex_lock(&data->lock);
	old = mt6797_method_for_rate(cluster, cluster->current_khz);
	old_cci = mt6797_method_for_rate(&data->cci, data->cci.current_khz);
	old_vproc1 = regulator_get_voltage(data->cluster[0].vproc);
	if (old_vproc1 < 0) {
		ret = old_vproc1;
		goto out_unlock;
	}
	old_vproc2 = regulator_get_voltage(data->cluster[2].vproc);
	if (old_vproc2 < 0) {
		ret = old_vproc2;
		goto out_unlock;
	}
	target_cci = mt6797_cci_target(data, cluster, target->khz,
					 target_uv, &vproc1_uv);
	vproc2_uv = cluster->id == 2 ? target_uv : data->cluster[2].current_uv;

	if (vproc1_uv > old_vproc1) {
		ret = mt6797_set_voltages(data, &data->cluster[0], vproc1_uv);
		if (ret)
			goto out_unlock;
	}
	if (vproc2_uv > old_vproc2) {
		ret = mt6797_set_voltages(data, &data->cluster[2], vproc2_uv);
		if (ret)
			goto out_unlock;
	}

	ret = mt6797_set_rate_locked(data, cluster, old, target);
	if (ret)
		goto out_unlock;

	cluster->current_khz = target->khz;
	cluster->current_uv = target_uv;
	ret = mt6797_set_rate_locked(data, &data->cci, old_cci, target_cci);
	if (ret) {
		dev_warn(data->dev, "CCI transition from %u to %ukHz failed: %d\n",
			 old_cci->khz, target_cci->khz, ret);
		/* Keep both rails raised until the CCI state is known again. */
		ret = 0;
		goto out_unlock;
	}
	data->cci.current_khz = target_cci->khz;
	if (vproc2_uv < old_vproc2) {
		ret = mt6797_set_voltages(data, &data->cluster[2], vproc2_uv);
		if (ret) {
			dev_warn(data->dev,
				 "VPROC2 reduction incomplete at %ukHz: %d\n",
				 target->khz, ret);
			ret = 0;
		}
	}
	if (vproc1_uv < old_vproc1) {
		ret = mt6797_set_voltages(data, &data->cluster[0], vproc1_uv);
		if (ret) {
			dev_warn(data->dev,
				 "VPROC1 reduction incomplete at CCI %ukHz: %d\n",
				 target_cci->khz, ret);
			ret = 0;
		}
	}

out_unlock:
	mutex_unlock(&data->lock);
	return ret;
}

static int mt6797_cpufreq_init(struct cpufreq_policy *policy)
{
	struct mt6797_cluster *cluster = mt6797_cluster_for_cpu(policy->cpu);
	struct mt6797_cpufreq *data = mt6797_data;
	unsigned int latency;
	int ret;

	if (!cluster)
		return -ENODEV;

	if (!cluster->opp_added) {
		ret = dev_pm_opp_of_cpumask_add_table(&cluster->cpus);
		if (ret)
			return ret;
		cluster->opp_added = true;
	}

	if (!cluster->freq_table) {
		ret = dev_pm_opp_init_cpufreq_table(cluster->cpu_dev,
						    &cluster->freq_table);
		if (ret) {
			dev_pm_opp_of_cpumask_remove_table(&cluster->cpus);
			cluster->opp_added = false;
			return ret;
		}
	}

	cpumask_copy(policy->cpus, &cluster->cpus);
	policy->shared_type = CPUFREQ_SHARED_TYPE_ANY;
	policy->driver_data = cluster;
	policy->freq_table = cluster->freq_table;
	latency = dev_pm_opp_get_max_transition_latency(cluster->cpu_dev);
	policy->cpuinfo.transition_latency = latency ? latency : 30000;
	policy->dvfs_possible_from_any_cpu = true;
	policy->cur = mt6797_read_rate(data, cluster);
	return 0;
}

static void mt6797_cpufreq_exit(struct cpufreq_policy *policy)
{
	struct mt6797_cluster *cluster = policy->driver_data;

	if (!cluster)
		return;

	dev_pm_opp_free_cpufreq_table(cluster->cpu_dev, &cluster->freq_table);
	if (cluster->opp_added) {
		dev_pm_opp_of_cpumask_remove_table(&cluster->cpus);
		cluster->opp_added = false;
	}
}

static struct cpufreq_driver mt6797_cpufreq_driver = {
	.name = "mt6797-cpufreq",
	.flags = CPUFREQ_HAVE_GOVERNOR_PER_POLICY | CPUFREQ_IS_COOLING_DEV,
	.verify = cpufreq_generic_frequency_table_verify,
	.target_index = mt6797_cpufreq_target_index,
	.get = mt6797_cpufreq_get,
	.init = mt6797_cpufreq_init,
	.exit = mt6797_cpufreq_exit,
};

/* TOPCKGEN is already owned by the common clock driver; share the mapping. */
static void __iomem *mt6797_iomap_shared(struct platform_device *pdev,
						 const char *name)
{
	struct resource *resource;
	void __iomem *base;

	resource = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	if (!resource)
		return ERR_PTR(-EINVAL);

	base = devm_ioremap(&pdev->dev, resource->start, resource_size(resource));
	return base ? base : ERR_PTR(-ENOMEM);
}

static int mt6797_cpufreq_probe(struct platform_device *pdev)
{
	struct mt6797_cpufreq *data;
	const struct mt6797_freq_method *cci_target, *cci_old;
	unsigned int vproc1_uv;
	unsigned int i;
	int ret;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->dev = &pdev->dev;
	mutex_init(&data->lock);
	data->mcumixed = mt6797_iomap_shared(pdev, "mcumixed");
	if (IS_ERR(data->mcumixed))
		return PTR_ERR(data->mcumixed);
	data->topckgen = mt6797_iomap_shared(pdev, "topckgen");
	if (IS_ERR(data->topckgen))
		return PTR_ERR(data->topckgen);
	data->semaphore = mt6797_iomap_shared(pdev, "semaphore");
	if (IS_ERR(data->semaphore))
		return PTR_ERR(data->semaphore);
	data->infracfg = mt6797_iomap_shared(pdev, "infracfg");
	if (IS_ERR(data->infracfg))
		return PTR_ERR(data->infracfg);

	data->cluster[0] = (struct mt6797_cluster) {
		.id = 0, .first_cpu = 0, .methods = mt6797_ll_methods,
		.method_count = ARRAY_SIZE(mt6797_ll_methods),
	};
	data->cluster[1] = (struct mt6797_cluster) {
		.id = 1, .first_cpu = 4, .methods = mt6797_l_methods,
		.method_count = ARRAY_SIZE(mt6797_l_methods),
	};
	data->cluster[2] = (struct mt6797_cluster) {
		.id = 2, .first_cpu = 8, .methods = mt6797_b_methods,
		.method_count = ARRAY_SIZE(mt6797_b_methods),
	};
	data->cci = (struct mt6797_cluster) {
		.id = 3, .methods = mt6797_cci_methods,
		.method_count = ARRAY_SIZE(mt6797_cci_methods),
	};

	for (i = 0; i < ARRAY_SIZE(data->cluster); i++) {
		struct mt6797_cluster *cluster = &data->cluster[i];
		int voltage_uv;

		cpumask_clear(&cluster->cpus);
		cpumask_set_cpu(cluster->first_cpu, &cluster->cpus);
		cpumask_set_cpu(cluster->first_cpu + 1, &cluster->cpus);
		if (i < 2) {
			cpumask_set_cpu(cluster->first_cpu + 2, &cluster->cpus);
			cpumask_set_cpu(cluster->first_cpu + 3, &cluster->cpus);
		}

		cluster->cpu_dev = get_cpu_device(cluster->first_cpu);
		if (!cluster->cpu_dev) {
			ret = -EPROBE_DEFER;
			goto err_regulators;
		}
		if (i == 1) {
			/* LL and L share DA9214 BUCKA/VPROC1. */
			cluster->vproc = data->cluster[0].vproc;
		} else {
			cluster->vproc = regulator_get(cluster->cpu_dev, "cpu");
			if (IS_ERR(cluster->vproc)) {
				ret = dev_err_probe(&pdev->dev, PTR_ERR(cluster->vproc),
						    "cluster%u: failed to get Vproc\n", i);
				cluster->vproc = NULL;
				goto err_regulators;
			}
			cluster->vproc_owner = true;
			ret = regulator_enable(cluster->vproc);
			if (ret) {
				dev_err_probe(&pdev->dev, ret,
					      "cluster%u: failed to enable Vproc\n", i);
				regulator_put(cluster->vproc);
				cluster->vproc = NULL;
				goto err_regulators;
			}
		}

		cluster->current_khz = mt6797_method_for_rate(cluster,
						      mt6797_read_rate(data, cluster))->khz;
		voltage_uv = regulator_get_voltage(cluster->vproc);
		if (voltage_uv < 0) {
			ret = voltage_uv;
			i++;
			goto err_regulators;
		}
		cluster->current_uv = voltage_uv;
		dev_info(&pdev->dev, "cluster%u: current rate %ukHz, Vproc %duV\n",
			 i, cluster->current_khz, cluster->current_uv);
	}

	/* Preloader voltage is not necessarily suitable for the CPU OPP governor. */
	ret = mt6797_set_voltages(data, &data->cluster[0],
				    data->cluster[0].current_uv);
	if (ret)
		goto err_regulators;
	ret = mt6797_set_voltages(data, &data->cluster[2],
				    data->cluster[2].current_uv);
	if (ret)
		goto err_regulators;

	data->cci.current_khz = mt6797_read_rate(data, &data->cci);
	cci_old = mt6797_method_for_rate(&data->cci, data->cci.current_khz);
	cci_target = mt6797_cci_target(data, NULL, 0, 0, &vproc1_uv);
	if (vproc1_uv > data->cluster[0].current_uv) {
		ret = mt6797_set_voltages(data, &data->cluster[0], vproc1_uv);
		if (ret)
			goto err_regulators;
	}
	ret = mt6797_set_rate_locked(data, &data->cci, cci_old, cci_target);
	if (ret)
		goto err_regulators;
	data->cci.current_khz = cci_target->khz;
	if (vproc1_uv < data->cluster[0].current_uv) {
		ret = mt6797_set_voltages(data, &data->cluster[0], vproc1_uv);
		if (ret)
			goto err_regulators;
	}
	dev_info(&pdev->dev, "CCI: %ukHz, shared Vproc target %uuV\n",
		 data->cci.current_khz, vproc1_uv);

	mt6797_data = data;
	platform_set_drvdata(pdev, data);
	ret = cpufreq_register_driver(&mt6797_cpufreq_driver);
	if (ret) {
		mt6797_data = NULL;
		platform_set_drvdata(pdev, NULL);
		goto err_regulators;
	}

	return 0;

err_regulators:
	while (i--) {
		if (data->cluster[i].vproc && data->cluster[i].vproc_owner) {
			regulator_disable(data->cluster[i].vproc);
			regulator_put(data->cluster[i].vproc);
		}
	}
	return ret;
}

static void mt6797_cpufreq_remove(struct platform_device *pdev)
{
	struct mt6797_cpufreq *data = platform_get_drvdata(pdev);
	unsigned int i;

	cpufreq_unregister_driver(&mt6797_cpufreq_driver);
	mt6797_data = NULL;
	for (i = 0; i < ARRAY_SIZE(data->cluster); i++) {
		if (data->cluster[i].vproc && data->cluster[i].vproc_owner) {
			regulator_disable(data->cluster[i].vproc);
			regulator_put(data->cluster[i].vproc);
		}
	}
}

static const struct of_device_id mt6797_cpufreq_of_match[] = {
	{ .compatible = "mediatek,mt6797-cpufreq" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6797_cpufreq_of_match);

static struct platform_driver mt6797_cpufreq_platform_driver = {
	.probe = mt6797_cpufreq_probe,
	.remove = mt6797_cpufreq_remove,
	.driver = {
		.name = "mt6797-cpufreq",
		.of_match_table = mt6797_cpufreq_of_match,
	},
};
module_platform_driver(mt6797_cpufreq_platform_driver);

MODULE_AUTHOR("X20 bring-up");
MODULE_DESCRIPTION("MT6797 vendor PLL CPU frequency scaling");
MODULE_LICENSE("GPL");
