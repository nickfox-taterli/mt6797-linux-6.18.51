// SPDX-License-Identifier: GPL-2.0-only
/*
 * MT6797 CPU DVFS
 *
 * Coordinates the CPU and CCI clock provider with shared VPROC/VSRAM votes.
 * Board operating points and manual voltage policy are described in DT.
 */

#include <linux/soc/mediatek/mt6797-sip.h>
#include <linux/arm-smccc.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/cpuhotplug.h>
#include <linux/cpumask.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/iopoll.h>
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
#include <linux/soc/mediatek/mt6797-armcore-dcm.h>
#include <linux/soc/mediatek/mt6797-cpufreq.h>
#include <linux/soc/mediatek/mt6797-mcuclk.h>
#include <linux/suspend.h>

#define MT6797_B_SRAM_LDO		0x102222b0

#define MT6797_CPULDO_CTRL_0		0xf98
#define MT6797_CPULDO_CTRL_1		0xf9c
#define MT6797_CPULDO_CTRL_2		0xfa0
#define MT6797_VSRAM_MIN_UV		1000000
#define MT6797_VSRAM_MAX_UV		1200000
#define MT6797_VSRAM_STEP_UV		25000
#define MT6797_VSRAM_MARGIN_UV		100000
#define MT6797_VSRAM_STEP_LIMIT_UV	275000

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
	bool active;
};

struct mt6797_cpufreq {
	struct device *dev;
	struct mt6797_mcuclk *clock;
	void __iomem *infracfg;
	struct mutex lock;
	struct mt6797_cluster cluster[3];
	struct mt6797_cluster cci;
	struct cpumask prepared_cpus;
	int hp_state;
	struct notifier_block pm_notifier;
	bool pm_rails_held;
	bool pm_prepare_failed;
	u32 pm_active;
	unsigned int pm_khz[3], pm_cluster_uv[3], pm_cci_khz;
	unsigned int pm_uv[2], pm_sram[2];
	unsigned long pm_cycles, pm_failures;
};

static struct mt6797_cpufreq *mt6797_data;

/* Board experiment policy; do not exceed the verified 2.405 GHz endpoint. */
static unsigned int big_max_khz = 2405000;
module_param(big_max_khz, uint, 0444);
MODULE_PARM_DESC(big_max_khz, "BIG policy ceiling in kHz (0: full DT table)");

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
	/* /2 requires a 2.158 GHz VCO that did not latch on this board. */
	MT6797_METHOD(1079000, 1, 1), MT6797_METHOD( 962000, 2, 1),
	MT6797_METHOD( 832000, 2, 1), MT6797_METHOD( 741000, 2, 1),
	MT6797_METHOD( 650000, 2, 1), MT6797_METHOD( 559000, 2, 1),
	MT6797_METHOD( 468000, 2, 2), MT6797_METHOD( 325000, 2, 2),
};

static const struct mt6797_freq_method mt6797_b_methods[] = {
	/* Board experiment points, bounded by the verified 2.405 GHz endpoint. */
	MT6797_METHOD(2405000, 1, 1),
	MT6797_METHOD(2366000, 1, 1),
	MT6797_METHOD(2314000, 1, 1),
	MT6797_METHOD(2249000, 1, 1),
	MT6797_METHOD(2197000, 1, 1),
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
	1180000, 1180000, 1150000, 1130000,
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

		if (!cluster->active)
			continue;
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

/* DCM and CPUFreq share the clock provider's mutex and hardware semaphore. */
int mt6797_armcore_dcm_access(struct device *supplier, u32 *value, bool write)
{
	struct mt6797_cpufreq *data = dev_get_drvdata(supplier);

	if (!data)
		return -EPROBE_DEFER;
	return mt6797_mcuclk_armcore(data->clock, value, write);
}
EXPORT_SYMBOL_GPL(mt6797_armcore_dcm_access);

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
static int mt6797_sync_dcm(struct mt6797_cpufreq *data,
			   const struct mt6797_cluster *cluster, unsigned int khz)
{
	return mt6797_mcuclk_sync(data->clock, cluster->id, khz);
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
	return mt6797_mcuclk_rate(data->clock, cluster->id);
}

/* Keep the rail vote stable while the clock provider performs the transition. */
static int mt6797_set_rate_locked(struct mt6797_cpufreq *data,
				  struct mt6797_cluster *cluster,
				  const struct mt6797_freq_method *old,
				  const struct mt6797_freq_method *target)
{
	return mt6797_mcuclk_transition(data->clock, cluster->id, old, target);
}

static unsigned int mt6797_cpufreq_get(unsigned int cpu)
{
	struct mt6797_cluster *cluster = mt6797_cluster_for_cpu(cpu);
	unsigned int rate;

	if (!cluster)
		return 0;

	mutex_lock(&mt6797_data->lock);
	rate = cluster->active ? mt6797_read_rate(mt6797_data, cluster) : 0;
	mutex_unlock(&mt6797_data->lock);

	return rate;
}

static int mt6797_pm_restore_locked(struct mt6797_cpufreq *data);

/* One target-frequency transaction, from OPP lookup through the final rail
 * reduction.  The fields are filled in by the steps below, in order, all
 * under data->lock. */
struct mt6797_cpufreq_transition {
	struct mt6797_cpufreq *data;
	struct mt6797_cluster *cluster;
	const struct mt6797_freq_method *old, *target, *old_cci, *target_cci;
	unsigned int target_uv, vproc1_uv, vproc2_uv;
	int old_vproc1, old_vproc2;
};

/* Validate the request and decode it: exact OPP for the table frequency,
 * its voltage, and the board clock method entry that reaches that rate. */
static int mt6797_cpufreq_resolve_target(struct cpufreq_policy *policy,
					 unsigned int index,
					 struct mt6797_cpufreq_transition *tr)
{
	struct dev_pm_opp *opp;
	unsigned long target_hz;

	tr->cluster = policy->driver_data;
	tr->data = mt6797_data;
	if (!tr->cluster || !policy->freq_table)
		return -EINVAL;

	target_hz = (unsigned long)policy->freq_table[index].frequency * 1000;
	opp = dev_pm_opp_find_freq_exact(tr->cluster->cpu_dev, target_hz, true);
	if (IS_ERR(opp))
		return PTR_ERR(opp);
	tr->target_uv = dev_pm_opp_get_voltage(opp);
	dev_pm_opp_put(opp);
	if (!tr->target_uv)
		return -EINVAL;

	tr->target = mt6797_method_for_rate(tr->cluster, target_hz / 1000);
	return 0;
}

/* Snapshot the shared state the transition decision depends on: the clock
 * method entries in effect, both shared rail levels, the CCI target derived
 * from all clusters, and the VPROC2 level requested by this transition. */
static int mt6797_cpufreq_capture_current(struct mt6797_cpufreq_transition *tr)
{
	struct mt6797_cpufreq *data = tr->data;

	tr->old = mt6797_method_for_rate(tr->cluster, tr->cluster->current_khz);
	tr->old_cci = mt6797_method_for_rate(&data->cci, data->cci.current_khz);
	tr->old_vproc1 = regulator_get_voltage(data->cluster[0].vproc);
	if (tr->old_vproc1 < 0)
		return tr->old_vproc1;
	tr->old_vproc2 = regulator_get_voltage(data->cluster[2].vproc);
	if (tr->old_vproc2 < 0)
		return tr->old_vproc2;
	tr->target_cci = mt6797_cci_target(data, tr->cluster, tr->target->khz,
					   tr->target_uv, &tr->vproc1_uv);
	tr->vproc2_uv = tr->cluster->id == 2 ? tr->target_uv :
			data->cluster[2].current_uv;
	return 0;
}

/* Frequency increases require the shared rails at the target level first:
 * the CCI vote decides how far VPROC1 rises, VPROC2 follows its own OPP. */
static int mt6797_cpufreq_raise_voltages(struct mt6797_cpufreq_transition *tr)
{
	int ret;

	if (tr->vproc1_uv > tr->old_vproc1) {
		ret = mt6797_set_voltages(tr->data, &tr->data->cluster[0],
					  tr->vproc1_uv);
		if (ret)
			return ret;
	}
	if (tr->cluster->id == 2 && tr->vproc2_uv > tr->old_vproc2) {
		ret = mt6797_set_voltages(tr->data, &tr->data->cluster[2],
					  tr->vproc2_uv);
		if (ret)
			return ret;
	}
	return 0;
}

/* Switch the cluster clock, record the new cluster state, then switch CCI.
 * Sets *clocks_stable only when both clocks reached the target; a failed CCI
 * transition keeps the current stable state unknown, so both rails must stay
 * raised.  A failed cluster switch leaves the recorded state untouched. */
static int mt6797_cpufreq_switch_clocks(struct mt6797_cpufreq_transition *tr,
					bool *clocks_stable)
{
	struct mt6797_cpufreq *data = tr->data;
	int ret;

	*clocks_stable = false;
	ret = mt6797_set_rate_locked(data, tr->cluster, tr->old, tr->target);
	if (ret)
		return ret;

	tr->cluster->current_khz = tr->target->khz;
	tr->cluster->current_uv = tr->target_uv;
	ret = mt6797_set_rate_locked(data, &data->cci, tr->old_cci,
				     tr->target_cci);
	if (ret) {
		dev_warn(data->dev,
			 "CCI transition from %u to %ukHz failed: %d\n",
			 tr->old_cci->khz, tr->target_cci->khz, ret);
		return 0;
	}
	data->cci.current_khz = tr->target_cci->khz;
	*clocks_stable = true;
	return 0;
}

/* Reductions happen only after both clocks reached the target.  A failed
 * reduction keeps the conservative higher voltage and is reported but not
 * propagated: the request itself succeeded. */
static void mt6797_cpufreq_lower_voltages(struct mt6797_cpufreq_transition *tr)
{
	int ret;

	if (tr->cluster->id == 2 && tr->vproc2_uv < tr->old_vproc2) {
		ret = mt6797_set_voltages(tr->data, &tr->data->cluster[2],
					  tr->vproc2_uv);
		if (ret)
			dev_warn(tr->data->dev,
				 "VPROC2 reduction incomplete at %ukHz: %d\n",
				 tr->target->khz, ret);
	}
	if (tr->vproc1_uv < tr->old_vproc1) {
		ret = mt6797_set_voltages(tr->data, &tr->data->cluster[0],
					  tr->vproc1_uv);
		if (ret)
			dev_warn(tr->data->dev,
				 "VPROC1 reduction incomplete at CCI %ukHz: %d\n",
				 tr->target_cci->khz, ret);
	}
}

static int mt6797_cpufreq_target_index(struct cpufreq_policy *policy,
				       unsigned int index)
{
	struct mt6797_cpufreq_transition tr;
	bool clocks_stable;
	int ret;

	ret = mt6797_cpufreq_resolve_target(policy, index, &tr);
	if (ret)
		return ret;

	mutex_lock(&tr.data->lock);
	if (tr.data->pm_rails_held) {
		ret = tr.data->pm_prepare_failed ?
			mt6797_pm_restore_locked(tr.data) : -EBUSY;
		if (ret)
			goto out_unlock;
	}
	if (!tr.cluster->active) {
		ret = -ENODEV;
		goto out_unlock;
	}
	ret = mt6797_cpufreq_capture_current(&tr);
	if (ret)
		goto out_unlock;

	ret = mt6797_cpufreq_raise_voltages(&tr);
	if (ret)
		goto out_unlock;

	ret = mt6797_cpufreq_switch_clocks(&tr, &clocks_stable);
	if (ret)
		goto out_unlock;
	if (clocks_stable)
		mt6797_cpufreq_lower_voltages(&tr);
	ret = 0;

out_unlock:
	mutex_unlock(&tr.data->lock);
	return ret;
}

/* INIT01 needs stable 1V rails. The caller first requests low frequencies;
 * the manual OPPs are never modified, and both rails/modes are restored before
 * releasing the same mutex used by DVFS, CCI and hotplug preparation.
 */
int mt6797_cpufreq_eem_measure(struct device *dev,
			      int (*measure)(void *), void *context)
{
	struct mt6797_cpufreq *data = dev_get_drvdata(dev);
	struct mt6797_cluster *rail[2];
	unsigned int saved_mode[2], saved_sram[2];
	int saved_uv[2], ret = 0, restore;
	unsigned int i;

	if (!data || !measure)
		return -ENODEV;
	rail[0] = &data->cluster[0];
	rail[1] = &data->cluster[2];
	cpus_read_lock();
	mutex_lock(&data->lock);
	if (data->pm_rails_held) {
		ret = -EBUSY;
		goto unlock;
	}
	for (i = 0; i < 3; i++) {
		if (!data->cluster[i].active || data->cluster[i].current_khz > 750000) {
			ret = -EBUSY;
			goto unlock;
		}
	}
	if (data->cci.current_khz > 750000) {
		ret = -EBUSY;
		goto unlock;
	}
	for (i = 0; i < 2; i++) {
		saved_uv[i] = regulator_get_voltage(rail[i]->vproc);
		saved_mode[i] = regulator_get_mode(rail[i]->vproc);
		ret = mt6797_vsram_read(data, rail[i], &saved_sram[i]);
		if (ret || saved_uv[i] < 0 || !saved_mode[i] ||
		    saved_mode[i] > REGULATOR_MODE_STANDBY) {
			ret = ret ?: -EIO;
			goto unlock;
		}
	}
	for (i = 0; i < 2; i++) {
		ret = regulator_set_mode(rail[i]->vproc, REGULATOR_MODE_FAST);
		if (ret)
			goto restore_rails;
		ret = mt6797_set_voltages(data, rail[i], 1000000);
		if (ret)
			goto restore_rails;
	}
	usleep_range(100, 200);
	ret = measure(context);
restore_rails:
	for (i = 0; i < 2; i++) {
		restore = mt6797_set_voltages(data, rail[i], saved_uv[i]);
		if (!restore)
			restore = mt6797_vsram_write(data, rail[i], saved_sram[i]);
		if (restore) {
			dev_err(dev, "EEM rail%u restoration failed: %d\n", i, restore);
			ret = restore;
		}
		restore = regulator_set_mode(rail[i]->vproc, saved_mode[i]);
		if (restore)
			ret = restore;
	}
unlock:
	mutex_unlock(&data->lock);
	cpus_read_unlock();
	return ret;
}
EXPORT_SYMBOL_GPL(mt6797_cpufreq_eem_measure);

static int mt6797_cpufreq_init(struct cpufreq_policy *policy)
{
	struct mt6797_cluster *cluster = mt6797_cluster_for_cpu(policy->cpu);
	struct mt6797_cpufreq *data = mt6797_data;
	unsigned int latency;
	struct dev_pm_opp *opp;
	unsigned long hz;
	unsigned int rate, uv;
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
	/* CPU_ON may have reset the PLL (BIG starts at 750/500 MHz). */
	mutex_lock(&data->lock);
	rate = mt6797_read_rate(data, cluster);
	if (!rate) {
		ret = -EIO;
		goto out_unlock;
	}
	hz = (unsigned long)rate * 1000;
	opp = dev_pm_opp_find_freq_ceil(cluster->cpu_dev, &hz);
	if (IS_ERR(opp)) {
		ret = PTR_ERR(opp);
		goto out_unlock;
	}
	uv = dev_pm_opp_get_voltage(opp);
	dev_pm_opp_put(opp);
	/* Only the BIG SRAM LDO is reset by cluster power cycling. */
	if (cluster->id == 2 && data->pm_rails_held) {
		/* CPU_ON resets the internal BIG SRAM LDO. The external rail was
		 * prepared before I2C suspended; only secure SRAM access is valid
		 * in this early CPU restore phase.
		 */
		ret = mt6797_vsram_write(data, cluster, MT6797_VSRAM_MAX_UV);
		if (ret)
			goto out_unlock;
	} else if (cluster->id == 2) {
		int vproc = regulator_get_voltage(cluster->vproc);

		ret = vproc < 0 ? vproc : mt6797_set_voltages(data, cluster, vproc);
		if (ret)
			goto out_unlock;
	}
	cluster->current_khz = rate;
	cluster->current_uv = uv;
	policy->cur = rate;
	ret = mt6797_sync_dcm(data, cluster, rate);
out_unlock:
	mutex_unlock(&data->lock);
	if (ret)
		dev_pm_opp_free_cpufreq_table(cluster->cpu_dev, &cluster->freq_table);
	return ret;
}

static void mt6797_cpufreq_exit(struct cpufreq_policy *policy)
{
	struct mt6797_cluster *cluster = policy->driver_data;

	if (!cluster)
		return;

	dev_pm_opp_free_cpufreq_table(cluster->cpu_dev, &cluster->freq_table);
	/* Keep OPPs available for the pre-CPU_ON voltage preparation. */
}

static int mt6797_cpufreq_offline(struct cpufreq_policy *policy)
{
	/* Retain the table and policy bounds for lightweight re-online. The BP
	 * callback releases the shared rail only after physical power-down.
	 */
	return 0;
}

/* Caller holds data->lock; change CCI before reducing the shared rail. */
static int mt6797_refresh_cci(struct mt6797_cpufreq *data)
{
	const struct mt6797_freq_method *old, *target;
	unsigned int uv;
	int previous, ret;

	target = mt6797_cci_target(data, NULL, 0, 0, &uv);
	previous = regulator_get_voltage(data->cluster[0].vproc);
	if (previous < 0)
		return previous;
	if (uv > previous) {
		ret = mt6797_set_voltages(data, &data->cluster[0], uv);
		if (ret)
			return ret;
	}
	old = mt6797_method_for_rate(&data->cci, data->cci.current_khz);
	ret = mt6797_set_rate_locked(data, &data->cci, old, target);
	if (ret)
		return ret;
	data->cci.current_khz = target->khz;
	if (uv < previous)
		return mt6797_set_voltages(data, &data->cluster[0], uv);
	return 0;
}

static unsigned int mt6797_max_manual_uv(struct mt6797_cluster *cluster)
{
	struct dev_pm_opp *opp;
	unsigned long hz = 0;
	unsigned int uv = 0;

	while (!IS_ERR(opp = dev_pm_opp_find_freq_ceil(cluster->cpu_dev, &hz))) {
		if (cluster->id == 2 && big_max_khz && hz > big_max_khz * 1000UL) {
			dev_pm_opp_put(opp);
			break;
		}
		uv = max(uv, (unsigned int)dev_pm_opp_get_voltage(opp));
		dev_pm_opp_put(opp);
		hz++;
	}
	return uv;
}

/* Prepare before device suspend, while the external I2C rails still work. */
static int mt6797_pm_prepare_locked(struct mt6797_cpufreq *data)
{
	unsigned int maximum[2] = { 0, 0 };
	int ret, restore;

	if (data->pm_rails_held) {
		if (!data->pm_prepare_failed)
			return -EBUSY;
		ret = mt6797_pm_restore_locked(data);
		if (ret)
			return ret;
	}
	data->pm_prepare_failed = false;
	data->pm_active = 0;
	data->pm_cci_khz = data->cci.current_khz;
	for (unsigned int i = 0; i < ARRAY_SIZE(data->cluster); i++) {
		struct mt6797_cluster *c = &data->cluster[i];
		unsigned int uv;

		if (!c->active)
			continue;
		uv = mt6797_max_manual_uv(c);
		if (!uv)
			return -EINVAL;
		maximum[i == 2] = max(maximum[i == 2], uv);
		data->pm_active |= BIT(i);
		data->pm_khz[i] = c->current_khz;
		data->pm_cluster_uv[i] = c->current_uv;
	}
	/* Save both supplies before making either change. */
	for (unsigned int i = 0; i < 2; i++) {
		struct mt6797_cluster *c = &data->cluster[i ? 2 : 0];
		int uv;

		if (!(data->pm_active & (i ? BIT(2) : BIT(0) | BIT(1))))
			continue;
		uv = regulator_get_voltage(c->vproc);
		if (uv < 0)
			return uv;
		data->pm_uv[i] = uv;
		maximum[i] = max_t(unsigned int, maximum[i], uv);
		ret = mt6797_vsram_read(data, c, &data->pm_sram[i]);
		if (ret)
			return ret;
	}
	for (unsigned int i = 0; i < 2; i++) {
		struct mt6797_cluster *c = &data->cluster[i ? 2 : 0];

		if (!(data->pm_active & (i ? BIT(2) : BIT(0) | BIT(1))))
			continue;
		ret = mt6797_set_voltages(data, c, maximum[i]);
		if (ret)
			goto rollback;
	}
	data->pm_rails_held = true;
	return 0;
rollback:
	for (unsigned int i = 0; i < 2; i++) {
		struct mt6797_cluster *c = &data->cluster[i ? 2 : 0];

		if (!(data->pm_active & (i ? BIT(2) : BIT(0) | BIT(1))))
			continue;
		restore = mt6797_set_voltages(data, c, data->pm_uv[i]);
		if (!restore)
			restore = mt6797_vsram_write(data, c, data->pm_sram[i]);
		if (restore) {
			dev_err(data->dev, "PM rail%u rollback failed: %d\n", i, restore);
			/* Keep DVFS and a later sleep blocked until restoration succeeds. */
			data->pm_rails_held = true;
			data->pm_prepare_failed = true;
		}
	}
	return ret;
}

/* Restore clocks at the held maximum rails before restoring the user's rails. */
static int mt6797_pm_restore_locked(struct mt6797_cpufreq *data)
{
	const struct mt6797_freq_method *old, *target;
	int ret;

	if (!data->pm_rails_held)
		return 0;
	/* A failed prepare never changed the clocks, only the rails. */
	if (data->pm_prepare_failed)
		goto restore_rails;
	for (unsigned int i = 0; i < ARRAY_SIZE(data->cluster); i++) {
		struct mt6797_cluster *c = &data->cluster[i];

		if (!(data->pm_active & BIT(i)))
			continue;
		if (!c->active)
			return -ENODEV;
		old = mt6797_method_for_rate(c, c->current_khz);
		target = mt6797_method_for_rate(c, data->pm_khz[i]);
		ret = mt6797_set_rate_locked(data, c, old, target);
		if (ret)
			return ret;
		c->current_khz = target->khz;
		c->current_uv = data->pm_cluster_uv[i];
	}
	old = mt6797_method_for_rate(&data->cci, data->cci.current_khz);
	target = mt6797_method_for_rate(&data->cci, data->pm_cci_khz);
	ret = mt6797_set_rate_locked(data, &data->cci, old, target);
	if (ret)
		return ret;
	data->cci.current_khz = target->khz;
restore_rails:
	for (unsigned int i = 0; i < 2; i++) {
		struct mt6797_cluster *c = &data->cluster[i ? 2 : 0];

		if (!(data->pm_active & (i ? BIT(2) : BIT(0) | BIT(1))))
			continue;
		ret = mt6797_set_voltages(data, c, data->pm_uv[i]);
		if (!ret)
			ret = mt6797_vsram_write(data, c, data->pm_sram[i]);
		if (ret)
			return ret;
	}
	data->pm_rails_held = false;
	if (!data->pm_prepare_failed)
		data->pm_cycles++;
	data->pm_prepare_failed = false;
	return 0;
}

static int mt6797_pm_notify(struct notifier_block *nb, unsigned long event, void *unused)
{
	struct mt6797_cpufreq *data = container_of(nb, struct mt6797_cpufreq, pm_notifier);
	int ret;

	if (event != PM_SUSPEND_PREPARE && event != PM_POST_SUSPEND)
		return NOTIFY_DONE;
	cpus_read_lock();
	mutex_lock(&data->lock);
	if (event == PM_SUSPEND_PREPARE)
		ret = mt6797_pm_prepare_locked(data);
	else
		ret = mt6797_pm_restore_locked(data);
	if (ret) {
		data->pm_failures++;
		dev_err(data->dev, "PM rail preparation/restoration failed: %d\n", ret);
	}
	mutex_unlock(&data->lock);
	cpus_read_unlock();
	/* Abort sleep on prepare failure, before the I2C adapters suspend. */
	return notifier_from_errno(ret);
}

static int mt6797_cpufreq_resume(struct cpufreq_policy *policy)
{
	struct mt6797_cpufreq *data = mt6797_data;
	int ret;

	mutex_lock(&data->lock);
	ret = mt6797_pm_restore_locked(data);
	if (!ret)
		policy->cur = mt6797_read_rate(data, policy->driver_data);
	else
		data->pm_failures++;
	mutex_unlock(&data->lock);
	return ret;
}

static int mt6797_cpu_prepare(unsigned int cpu)
{
	struct mt6797_cluster *cluster = mt6797_cluster_for_cpu(cpu);
	struct mt6797_cpufreq *data = mt6797_data;
	struct dev_pm_opp *opp;
	unsigned long hz = 0;
	unsigned int uv = 0;
	int ret = 0;

	if (!cluster)
		return 0;
	mutex_lock(&data->lock);
	if (data->pm_prepare_failed) {
		ret = mt6797_pm_restore_locked(data);
		if (ret)
			goto out;
	}
	if (cluster->active)
		goto mark_prepared;
	if (data->pm_rails_held) {
		if (!(data->pm_active & BIT(cluster->id))) {
			ret = -EBUSY;
			goto out;
		}
		cluster->active = true;
		goto mark_prepared;
	}

	/* The first CPU must have power before PSCI CPU_ON.  Cover all manual
	 * OPP voltages during the short reset-clock -> governor transition;
	 * do not assume this experimental voltage table is monotonic.
	 */
	while (!IS_ERR(opp = dev_pm_opp_find_freq_ceil(cluster->cpu_dev, &hz))) {
		if (cluster->id == 2 && big_max_khz && hz > big_max_khz * 1000UL) {
			dev_pm_opp_put(opp);
			break;
		}
		uv = max(uv, (unsigned int)dev_pm_opp_get_voltage(opp));
		dev_pm_opp_put(opp);
		hz++;
	}
	if (!uv) {
		ret = -EINVAL;
		goto out;
	}
	if (cluster->id == 2) {
		/* Restore its reset SRAM LDO after CPU_ON, before changing PLL. */
		ret = regulator_set_voltage(cluster->vproc, uv, uv);
		udelay(25);
	} else {
		ret = mt6797_set_voltages(data, cluster, uv);
	}
	if (ret)
		goto out;
	cluster->active = true;
	cluster->current_uv = uv;
	ret = mt6797_refresh_cci(data);
	if (ret) {
		cluster->active = false;
		/* Retaining raised rails on failure is intentional. */
		goto out;
	}
	dev_info(data->dev, "cluster%u: prepared CPU%u, Vproc %uuV\n",
		 cluster->id, cpu, uv);
mark_prepared:
	cpumask_set_cpu(cpu, &data->prepared_cpus);
out:
	mutex_unlock(&data->lock);
	return ret;
}

static int mt6797_cpu_unprepare(unsigned int cpu)
{
	struct mt6797_cluster *cluster = mt6797_cluster_for_cpu(cpu);
	struct mt6797_cpufreq *data = mt6797_data;
	int ret;

	if (!cluster)
		return 0;
	mutex_lock(&data->lock);
	cpumask_clear_cpu(cpu, &data->prepared_cpus);
	if (!cpumask_intersects(&cluster->cpus, &data->prepared_cpus)) {
		/* BP teardown runs after the target's physical power-off, also
		 * when a failed bring-up is rolled back by the hotplug core.
		 */
		cluster->active = false;
		mt6797_mcuclk_forget(data->clock, cluster->id);
		ret = data->pm_rails_held ? 0 : mt6797_refresh_cci(data);
		if (ret)
			dev_warn(data->dev, "cluster%u: CCI/rail release failed: %d\n",
				 cluster->id, ret);
		dev_info(data->dev, "cluster%u: last CPU off, CCI %ukHz\n",
			 cluster->id, data->cci.current_khz);
	}
	mutex_unlock(&data->lock);
	/* A physically dead CPU cannot be resurrected by failing teardown. */
	return 0;
}

static ssize_t runtime_state_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct mt6797_cpufreq *data = dev_get_drvdata(dev);
	struct mt6797_mcuclk_snapshot clocks;
	ssize_t len = 0;
	unsigned int i;

	/* Serialize the hardware reads with the whole PSCI power transition. */
	cpus_read_lock();
	mutex_lock(&data->lock);
	mt6797_mcuclk_snapshot(data->clock, data->cluster[2].active, &clocks);
	len += sysfs_emit_at(buf, len, "prepared_cpus=%*pbl mux=0x%x ckdiv=0x%x\n",
			     cpumask_pr_args(&data->prepared_cpus), clocks.mux, clocks.ckdiv);
	for (i = 0; i < ARRAY_SIZE(data->cluster); i++) {
		struct mt6797_cluster *cluster = &data->cluster[i];
		unsigned int vsram = 0;
		int ret = cluster->active ? mt6797_vsram_read(data, cluster, &vsram) :
			-ENODEV;

		len += sysfs_emit_at(buf, len,
			"cluster%u active=%u requested_khz=%u pll_khz=%u requested_uv=%u vproc_uv=%d vsram_uv=%u vsram_error=%d\n",
			i, cluster->active, cluster->current_khz,
			cluster->active ? mt6797_read_rate(data, cluster) : 0,
			cluster->current_uv, regulator_get_voltage(cluster->vproc), vsram, ret);
	}
	len += sysfs_emit_at(buf, len, "cci requested_khz=%u pll_khz=%u\n",
			     data->cci.current_khz, mt6797_read_rate(data, &data->cci));
	len += sysfs_emit_at(buf, len, "pm_rails_held=%u pm_cycles=%lu pm_failures=%lu\n",
			     data->pm_rails_held, data->pm_cycles, data->pm_failures);
	len += sysfs_emit_at(buf, len,
		"clock_flow fhctl=%u ready=%u hops=%lu failures=%lu sync_dcm=%u updates=%lu little_dcm=0x%x error=%d big_dcm=0x%x error=%d\n",
		clocks.fhctl, clocks.ready, clocks.hops, clocks.hop_failures,
		clocks.sync_dcm, clocks.dcm_updates, clocks.little_dcm,
		clocks.little_error, clocks.big_dcm, clocks.big_error);
	mutex_unlock(&data->lock);
	cpus_read_unlock();
	return len;
}
static DEVICE_ATTR_RO(runtime_state);

/* BSP abist channels 34/35/36/37 measure the clock outputs after CKDIV.
 * data->lock, the HW semaphore and cpus_read_lock exclude DVFS and PSCI
 * meter users. Restore every writable control field even on timeout.
 */
static ssize_t frequency_meter_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	static const unsigned int selectors[] = { 34, 35, 37, 36, 45 };
	static const char * const names[] = {
		"cluster0", "cluster1", "cluster2", "cci", "abist45",
	};
	struct mt6797_cpufreq *data = dev_get_drvdata(dev);
	struct mt6797_mcuclk_measurement results[5];
	unsigned int active = 0;
	ssize_t len = 0;
	int ret;

	cpus_read_lock();
	mutex_lock(&data->lock);
	for (unsigned int i = 0; i < ARRAY_SIZE(data->cluster); i++)
		if (data->cluster[i].active)
			active |= BIT(i);
	ret = mt6797_mcuclk_measure(data->clock, active, results);
	if (!ret)
		for (unsigned int i = 0; i < ARRAY_SIZE(selectors); i++)
			len += sysfs_emit_at(buf, len, "%s selector=%u khz=%u error=%d\n",
					     names[i], selectors[i], results[i].khz,
					     results[i].error);
	mutex_unlock(&data->lock);
	cpus_read_unlock();
	return ret ? ret : len;
}
static DEVICE_ATTR_RO(frequency_meter);

static struct attribute *mt6797_cpufreq_attrs[] = {
	&dev_attr_runtime_state.attr,
	&dev_attr_frequency_meter.attr,
	NULL,
};
ATTRIBUTE_GROUPS(mt6797_cpufreq);

static int mt6797_cpufreq_verify(struct cpufreq_policy_data *policy)
{
	struct mt6797_cluster *cluster = mt6797_cluster_for_cpu(policy->cpu);
	int ret = cpufreq_generic_frequency_table_verify(policy);

	if (!ret && cluster && cluster->id == 2 && big_max_khz) {
		cpufreq_verify_within_limits(policy, 0, big_max_khz);
		ret = cpufreq_generic_frequency_table_verify(policy);
	}
	return ret;
}

static struct cpufreq_driver mt6797_cpufreq_driver = {
	.name = "mt6797-cpufreq",
	.flags = CPUFREQ_HAVE_GOVERNOR_PER_POLICY | CPUFREQ_IS_COOLING_DEV,
	.verify = mt6797_cpufreq_verify,
	.target_index = mt6797_cpufreq_target_index,
	.get = mt6797_cpufreq_get,
	.init = mt6797_cpufreq_init,
	.online = mt6797_cpufreq_init,
	.offline = mt6797_cpufreq_offline,
	.exit = mt6797_cpufreq_exit,
	.resume = mt6797_cpufreq_resume,
};

/* INFRACFG is shared with its syscon provider. */
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
	data->clock = devm_mt6797_mcuclk_create(pdev);
	if (IS_ERR(data->clock))
		return PTR_ERR(data->clock);
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
		ret = dev_pm_opp_of_cpumask_add_table(&cluster->cpus);
		if (ret)
			goto err_regulators;
		cluster->opp_added = true;
		cluster->active = cpumask_intersects(&cluster->cpus, cpu_online_mask);
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
	ret = mt6797_sync_dcm(data, &data->cci, data->cci.current_khz);
	if (ret)
		goto err_regulators;
	if (vproc1_uv < data->cluster[0].current_uv) {
		ret = mt6797_set_voltages(data, &data->cluster[0], vproc1_uv);
		if (ret)
			goto err_regulators;
	}
	dev_info(&pdev->dev, "CCI: %ukHz, shared Vproc target %uuV\n",
		 data->cci.current_khz, vproc1_uv);

	mt6797_data = data;
	platform_set_drvdata(pdev, data);
	cpus_read_lock();
	cpumask_copy(&data->prepared_cpus, cpu_online_mask);
	ret = cpuhp_setup_state_nocalls_cpuslocked(CPUHP_BP_PREPARE_DYN,
						 "cpufreq/mt6797:prepare",
						 mt6797_cpu_prepare,
						 mt6797_cpu_unprepare);
	cpus_read_unlock();
	if (ret < 0)
		goto err_data;
	data->hp_state = ret;
	ret = cpufreq_register_driver(&mt6797_cpufreq_driver);
	if (ret) {
		cpuhp_remove_state_nocalls(data->hp_state);
		goto err_data;
	}
	data->pm_notifier.notifier_call = mt6797_pm_notify;
	ret = register_pm_notifier(&data->pm_notifier);
	if (ret) {
		cpufreq_unregister_driver(&mt6797_cpufreq_driver);
		cpuhp_remove_state_nocalls(data->hp_state);
		goto err_data;
	}

	return 0;

err_data:
	mt6797_data = NULL;
	platform_set_drvdata(pdev, NULL);
err_regulators:
	for (unsigned int j = 0; j < ARRAY_SIZE(data->cluster); j++)
		if (data->cluster[j].opp_added)
			dev_pm_opp_of_cpumask_remove_table(&data->cluster[j].cpus);
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

	unregister_pm_notifier(&data->pm_notifier);
	cpufreq_unregister_driver(&mt6797_cpufreq_driver);
	cpuhp_remove_state_nocalls(data->hp_state);
	mt6797_data = NULL;
	for (i = 0; i < ARRAY_SIZE(data->cluster); i++) {
		if (data->cluster[i].opp_added)
			dev_pm_opp_of_cpumask_remove_table(&data->cluster[i].cpus);
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
		.dev_groups = mt6797_cpufreq_groups,
	},
};
module_platform_driver(mt6797_cpufreq_platform_driver);

MODULE_AUTHOR("X20 bring-up");
MODULE_DESCRIPTION("MT6797 vendor PLL CPU frequency scaling");
MODULE_LICENSE("GPL");
