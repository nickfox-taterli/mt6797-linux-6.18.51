// SPDX-License-Identifier: GPL-2.0-only
/* MT6797 MCUSYS CPU/CCI clock provider. Voltage votes remain in CPUFreq. */
#include <linux/arm-smccc.h>
#include <linux/bitfield.h>
#include <linux/clk-provider.h>
#include <linux/cpu.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/limits.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_clk.h>
#include <linux/platform_device.h>
#include <linux/soc/mediatek/mt6797-armcore-dcm.h>
#include <linux/soc/mediatek/mt6797-mcuclk.h>
#include <linux/soc/mediatek/mt6797-sip.h>

#define MT6797_PLL_LL_CON1 0x204
#define MT6797_PLL_L_CON1 0x214
#define MT6797_PLL_CCI_CON1 0x224
#define MT6797_PLL_MUXSEL 0x270
#define MT6797_PLL_CKDIV 0x274
#define MT6797_ARMPLLDIV_DCMCTL 0x278
#define MT6797_TOP_CLK_MISC_CFG_0 0x104
#define MT6797_SEMA_POWERON_CONFIG 0x000
#define MT6797_SEMA3_M0 0x440
#define MT6797_SEMA_PROJECT_CODE 0x0b16
#define MT6797_SEMA_TIMEOUT_US 2000
#define MT6797_B_PLL_CON0 0x102224a0
#define MT6797_B_PLL_CON1 0x102224a4

struct mt6797_mcuclk_cluster { unsigned int id; };
struct mt6797_mcuclk_output {
	struct clk_hw hw;
	struct mt6797_mcuclk *provider;
	unsigned int id;
};
struct mt6797_mcuclk {
	struct device *dev;
	void __iomem *mcumixed, *topckgen, *semaphore;
	struct mutex lock;
	bool fhctl_ready;
	unsigned long hops, hop_failures, dcm_updates;
	unsigned long cached_rate[4];
	struct mt6797_mcuclk_output outputs[4];
	struct clk_hw_onecell_data *clock_data;
};
static bool fhctl = true;
module_param(fhctl, bool, 0444);
static bool sync_dcm = true;
module_param(sync_dcm, bool, 0444);


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


static unsigned int mt6797_ckdiv_shift(const struct mt6797_mcuclk_cluster *cluster)
{
	return cluster->id == 0 ? 5 : cluster->id == 1 ? 10 :
		cluster->id == 3 ? 15 : 0;
}


static unsigned int mt6797_mux_shift(const struct mt6797_mcuclk_cluster *cluster)
{
	return cluster->id == 0 ? 2 : cluster->id == 1 ? 4 :
		cluster->id == 3 ? 6 : 0;
}


static void mt6797_clock_switch(struct mt6797_mcuclk *data,
				const struct mt6797_mcuclk_cluster *cluster,
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


static int mt6797_hw_semaphore_get(struct mt6797_mcuclk *data)
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


static void mt6797_hw_semaphore_put(struct mt6797_mcuclk *data)
{
	writel_relaxed((MT6797_SEMA_PROJECT_CODE << 16) | BIT(0),
		       data->semaphore + MT6797_SEMA_POWERON_CONFIG);
	if (readl_relaxed(data->semaphore + MT6797_SEMA3_M0) & BIT(0))
		writel_relaxed(1, data->semaphore + MT6797_SEMA3_M0);
}

/* Consecutive reads must remain uninterrupted on the MCUMIXED bridge. */
static u32 mt6797_mcu_read(void __iomem *reg)
{
	unsigned long flags;
	u32 value;
	unsigned int i;

	local_irq_save(flags);
	ndelay(200);
	for (i = 0; i < 3; i++) {
		readl(reg);
		value = readl(reg);
		if (value)
			break;
	}
	mb();
	local_irq_restore(flags);
	return value;
}


static int mt6797_mcu_write(void __iomem *reg, u32 value, u32 mask)
{
	unsigned int i;

	for (i = 0; i < 4; i++) {
		writel(value, reg);
		ndelay(200);
		if ((mt6797_mcu_read(reg) & mask) == (value & mask))
			return 0;
	}
	pr_err_ratelimited("mt6797-mcuclk: MCU write failed value=%08x mask=%08x readback=%08x\n",
			   value, mask, mt6797_mcu_read(reg));
	return -EIO;
}

/* PLL0=LL, PLL1=L, PLL2=CCI; PLL3 is the untouched backup. */
static int mt6797_fhctl_init(struct mt6797_mcuclk *data)
{
	void __iomem *base = data->mcumixed + 0xf00;
	unsigned int id;
	u32 value;
	int ret;

	if (data->fhctl_ready)
		return 0;
	if (mt6797_mcu_read(base) & 7)
		return -EBUSY;
	for (id = 0; id < 3; id++) {
		void __iomem *cfg = base + 0x34 + 0x14 * id;

		value = mt6797_mcu_read(base + 4) | BIT(id);
		ret = mt6797_mcu_write(base + 4, value, U32_MAX);
		if (ret)
			return ret;
		value = mt6797_mcu_read(base + 8) & ~BIT(id);
		ret = mt6797_mcu_write(base + 8, value, U32_MAX);
		if (!ret)
			ret = mt6797_mcu_write(base + 8, value | BIT(id), U32_MAX);
		if (!ret)
			ret = mt6797_mcu_write(cfg, 0, U32_MAX);
		if (!ret)
			ret = mt6797_mcu_write(cfg + 4, 0, U32_MAX);
		if (!ret)
			ret = mt6797_mcu_write(cfg + 8, 0, GENMASK(20, 0));
		if (ret)
			return ret;
	}
	data->fhctl_ready = true;
	return 0;
}


static int mt6797_set_hopping_pll(struct mt6797_mcuclk *data,
				 const struct mt6797_mcuclk_cluster *cluster,
				 const struct mt6797_freq_method *method)
{
	unsigned int id = cluster->id == 3 ? 2 : cluster->id;
	void __iomem *base = data->mcumixed + 0xf00;
	void __iomem *cfg = base + 0x34 + 0x14 * id;
	void __iomem *con1 = data->mcumixed + 0x204 + id * 0x10;
	u64 vco = (u64)method->khz * method->pos_div * method->clk_div;
	u32 dds = div_u64((vco / 1000) << 14, 26);
	u32 saved, hp, mon = 0;
	unsigned int i;
	int ret, cleanup;

	if (id > 2 || cluster->id == 2 || dds > GENMASK(20, 0))
		return -EINVAL;
	ret = mt6797_fhctl_init(data);
	if (ret)
		return ret;
	hp = mt6797_mcu_read(base);
	if (hp & BIT(id))
		return -EBUSY;
	saved = mt6797_mcu_read(con1);
	ret = mt6797_mcu_write(cfg, 0, U32_MAX);
	if (!ret)
		ret = mt6797_mcu_write(cfg + 8,
				      (saved & GENMASK(20, 0)) | BIT(31), GENMASK(20, 0));
	if (!ret)
		ret = mt6797_mcu_write(cfg, BIT(2) | BIT(0), U32_MAX);
	if (!ret)
		ret = mt6797_mcu_write(base + 0xc, 0x6003c97, U32_MAX);
	if (!ret)
		ret = mt6797_mcu_write(base, hp | BIT(id), U32_MAX);
	wmb();
	if (!ret)
		ret = mt6797_mcu_write(cfg + 0xc, dds | BIT(31), GENMASK(20, 0));
	if (!ret) {
		for (i = 0; i < 100; i++) {
			mon = mt6797_mcu_read(cfg + 0x10) & GENMASK(20, 0);
			if (mon == dds)
				break;
			udelay(10);
		}
		if (mon != dds)
			ret = -ETIMEDOUT;
	}
	if (ret)
		/* Stay parked until the caller restores the complete old method. */
		mt6797_clock_switch(data, cluster, 2);
	cleanup = mt6797_mcu_write(con1,
				  ((ret ? saved : mon) & GENMASK(20, 0)) |
				  (saved & ~GENMASK(20, 0)) | BIT(31), GENMASK(20, 0));
	if (!ret)
		ret = cleanup;
	cleanup = mt6797_mcu_write(base, hp, U32_MAX);
	if (!ret)
		ret = cleanup;
	cleanup = mt6797_mcu_write(cfg, 0, U32_MAX);
	if (!ret)
		ret = cleanup;
	if (ret)
		data->hop_failures++;
	else
		data->hops++;
	return ret;
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


static int mt6797_sync_dcm(struct mt6797_mcuclk *data,
			   const struct mt6797_mcuclk_cluster *cluster, unsigned int khz)
{
	unsigned int div = min(khz / 70000, 31U);
	u32 address = cluster->id == 2 ? 0x10222274 : 0x10220744;
	unsigned int shift = cluster->id == 0 ? 8 : cluster->id == 1 ? 16 : 0;
	u32 toggle = cluster->id == 2 ? BIT(1) : BIT(shift + 6);
	u32 mask = cluster->id == 2 ? GENMASK(6, 2) : GENMASK(shift + 5, shift + 1);
	u32 value, actual;
	struct arm_smccc_res res;
	int ret;

	if (!sync_dcm)
		return 0;
	ret = mt6797_smc_read(address, &value);
	if (ret)
		return ret;
	arm_smccc_smc(MT6797_SIP_OCP_WRITE, address, value & ~toggle,
			0, 0, 0, 0, 0, &res);
	if ((long)res.a0 < 0)
		return (int)res.a0;
	value = (value & ~(mask | toggle)) | toggle | BIT(shift) |
		(div << (cluster->id == 2 ? 2 : shift + 1));
	arm_smccc_smc(MT6797_SIP_OCP_WRITE, address, value, 0, 0, 0, 0, 0, &res);
	if ((long)res.a0 < 0)
		return (int)res.a0;
	ret = mt6797_smc_read(address, &actual);
	if (ret)
		return ret;
	if (actual != value)
		return -EIO;
	data->dcm_updates++;
	return 0;
}


static unsigned int mt6797_read_rate(struct mt6797_mcuclk *data,
				      const struct mt6797_mcuclk_cluster *cluster)
{
	u32 ckdiv = readl_relaxed(data->mcumixed + MT6797_PLL_CKDIV);
	u32 mux = readl_relaxed(data->mcumixed + MT6797_PLL_MUXSEL);

	mux = (mux >> mt6797_mux_shift(cluster)) & 3;
	if (!mux)
		return 26000 / mt6797_ckdiv_ratio(
			(ckdiv >> mt6797_ckdiv_shift(cluster)) & 0x1f);
	if (mux != 1) {
		dev_warn_ratelimited(data->dev, "cluster%u: unexpected clock mux %u\n",
				     cluster->id, mux);
		return 0;
	}

	ckdiv >>= mt6797_ckdiv_shift(cluster);
	ckdiv &= 0x1f;

	if (cluster->id == 2) {
		u32 con0, con1;
		u64 freq;
		unsigned int pos_div;

		if (mt6797_smc_read(MT6797_B_PLL_CON0, &con0) ||
		    mt6797_smc_read(MT6797_B_PLL_CON1, &con1))
			return 0;

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


static void mt6797_set_clkdiv(struct mt6797_mcuclk *data,
				      const struct mt6797_mcuclk_cluster *cluster,
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


static void mt6797_set_posdiv(struct mt6797_mcuclk *data,
				      const struct mt6797_mcuclk_cluster *cluster,
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


static int mt6797_set_direct_pll(struct mt6797_mcuclk *data,
					const struct mt6797_mcuclk_cluster *cluster,
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

/* BIG stays on the backup clock unless the old PLL and DCM are restored. */
static int mt6797_restore_big_pll(struct mt6797_mcuclk *data,
				  const struct mt6797_mcuclk_cluster *cluster,
				  const struct mt6797_freq_method *old)
{
	int ret = mt6797_set_big_pll(old);

	if (ret)
		return ret;
	udelay(20);
	mt6797_set_clkdiv(data, cluster, old->clk_div);
	ret = mt6797_sync_dcm(data, cluster, old->khz);
	if (!ret)
		mt6797_clock_switch(data, cluster, 1);
	return ret;
}


static int mt6797_set_rate_locked(struct mt6797_mcuclk *data,
					 struct mt6797_mcuclk_cluster *cluster,
					 const struct mt6797_freq_method *old,
					 const struct mt6797_freq_method *target)
{
	int ret;

	if (old->khz == target->khz &&
	    abs((int)mt6797_read_rate(data, cluster) - (int)target->khz) < 1000)
		return 0;

	ret = mt6797_hw_semaphore_get(data);
	if (ret)
		return ret;
	/* BSP lowers sync DCM before downclocking; CCI uses divider zero. */
	if (old->khz > target->khz) {
		ret = mt6797_sync_dcm(data, cluster,
					cluster->id == 3 ? 0 : target->khz);
		if (ret)
			goto out_semaphore;
	}

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
		if (ret) {
			int restore = mt6797_restore_big_pll(data, cluster, old);

			if (restore)
				dev_err(data->dev, "BIG PLL rollback failed: %d; kept on backup clock\n",
					restore);
			goto out_semaphore;
		}
		udelay(20);
		mt6797_clock_switch(data, cluster, 1);
	} else if (fhctl) {
		ret = mt6797_set_hopping_pll(data, cluster, target);
		if (ret) {
			mt6797_set_direct_pll(data, cluster, old);
			mt6797_set_clkdiv(data, cluster, old->clk_div);
			mt6797_sync_dcm(data, cluster, old->khz);
			goto out_semaphore;
		}
	} else {
		ret = mt6797_set_direct_pll(data, cluster, target);
	}
	if (!ret && old->clk_div > target->clk_div)
		mt6797_set_clkdiv(data, cluster, target->clk_div);
	if (!ret && cluster->id != 2 && old->pos_div > target->pos_div)
		mt6797_set_posdiv(data, cluster, target->pos_div);
	if (!ret && old->khz < target->khz) {
		int dcm_ret = mt6797_sync_dcm(data, cluster, target->khz);

		if (dcm_ret)
			dev_warn_ratelimited(data->dev, "sync DCM update failed: %d\n", dcm_ret);
	}

out_semaphore:
	mt6797_hw_semaphore_put(data);
	return ret;
}

static int mt6797_measure_rate(struct mt6797_mcuclk *data, unsigned int selector,
			       unsigned int *khz)
{
	u32 arm_k1 = readl(data->mcumixed + 0x27c);
	u32 mon_en = readl(data->mcumixed + 0x284);
	u32 dbg = readl(data->topckgen + 0x10c);
	u32 misc = readl(data->topckgen + 0x104);
	u32 cal0 = readl(data->topckgen + 0x220);
	u32 cal1 = readl(data->topckgen + 0x224);
	u32 value;
	int ret;

	if (cal0 & BIT(4))
		return -EBUSY;
	writel(0, data->mcumixed + 0x27c);
	writel(U32_MAX, data->mcumixed + 0x284);
	writel((dbg & 0xffc0fffc) | (selector << 16), data->topckgen + 0x10c);
	/* Divide GHz inputs before counting, retaining the 1024-cycle gate. */
	writel((misc & 0x00ffffff) | 0x07000000, data->topckgen + 0x104);
	writel(0x03ff0000, data->topckgen + 0x224);
	writel(0x1000, data->topckgen + 0x220);
	writel(0x1010, data->topckgen + 0x220);
	ret = readl_poll_timeout_atomic(data->topckgen + 0x220, value,
					!(value & BIT(4)), 1, 1000);
	if (!ret)
		*khz = div_u64((u64)(readl(data->topckgen + 0x224) & 0xffff) *
				26000 * 8, 1024);
	writel(0, data->topckgen + 0x220);
	writel(cal1, data->topckgen + 0x224);
	writel(dbg, data->topckgen + 0x10c);
	writel(misc, data->topckgen + 0x104);
	writel(mon_en, data->mcumixed + 0x284);
	writel(arm_k1, data->mcumixed + 0x27c);
	writel(cal0, data->topckgen + 0x220);
	/* The meter result bits in CALI1 need not be writable. */
	if (readl(data->mcumixed + 0x27c) != arm_k1 ||
	    readl(data->mcumixed + 0x284) != mon_en ||
	    readl(data->topckgen + 0x10c) != dbg ||
	    readl(data->topckgen + 0x104) != misc ||
	    readl(data->topckgen + 0x220) != cal0 ||
	    ((readl(data->topckgen + 0x224) ^ cal1) & 0xffff0000))
		return -EIO;
	return ret;
}

unsigned int mt6797_mcuclk_rate(struct mt6797_mcuclk *clk, unsigned int id)
{
	struct mt6797_mcuclk_cluster cluster = { .id = id };
	unsigned int khz;

	if (id >= ARRAY_SIZE(clk->outputs))
		return 0;
	mutex_lock(&clk->lock);
	khz = mt6797_read_rate(clk, &cluster);
	WRITE_ONCE(clk->cached_rate[id], (unsigned long)khz * 1000);
	mutex_unlock(&clk->lock);
	return khz;
}
EXPORT_SYMBOL_GPL(mt6797_mcuclk_rate);

void mt6797_mcuclk_forget(struct mt6797_mcuclk *clk, unsigned int id)
{
	if (id < ARRAY_SIZE(clk->outputs))
		WRITE_ONCE(clk->cached_rate[id], 0);
}
EXPORT_SYMBOL_GPL(mt6797_mcuclk_forget);

int mt6797_mcuclk_transition(struct mt6797_mcuclk *clk, unsigned int id,
			     const struct mt6797_freq_method *old,
			     const struct mt6797_freq_method *target)
{
	struct mt6797_mcuclk_cluster cluster = { .id = id };
	int ret;

	if (id >= ARRAY_SIZE(clk->outputs) || !old || !target)
		return -EINVAL;
	mutex_lock(&clk->lock);
	ret = mt6797_set_rate_locked(clk, &cluster, old, target);
	if (!ret)
		WRITE_ONCE(clk->cached_rate[id], (unsigned long)target->khz * 1000);
	mutex_unlock(&clk->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mt6797_mcuclk_transition);

int mt6797_mcuclk_sync(struct mt6797_mcuclk *clk, unsigned int id,
			unsigned int khz)
{
	struct mt6797_mcuclk_cluster cluster = { .id = id };
	int ret;

	if (id >= ARRAY_SIZE(clk->outputs))
		return -EINVAL;
	mutex_lock(&clk->lock);
	ret = mt6797_hw_semaphore_get(clk);
	if (!ret) {
		ret = mt6797_sync_dcm(clk, &cluster, khz);
		mt6797_hw_semaphore_put(clk);
	}
	mutex_unlock(&clk->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mt6797_mcuclk_sync);

int mt6797_mcuclk_armcore(struct mt6797_mcuclk *clk, u32 *value, bool write)
{
	void __iomem *reg = clk->mcumixed + MT6797_ARMPLLDIV_DCMCTL;
	u32 old;
	int ret;

	lockdep_assert_cpus_held();
	mutex_lock(&clk->lock);
	ret = mt6797_hw_semaphore_get(clk);
	if (ret)
		goto out;
	old = mt6797_mcu_read(reg);
	if (write) {
		u32 next = (old & ~MT6797_ARMCORE_DCM_MASK) |
			   (*value & MT6797_ARMCORE_DCM_MASK);

		ret = mt6797_mcu_write(reg, next, MT6797_ARMCORE_DCM_MASK);
		if (ret && mt6797_mcu_write(reg, old, MT6797_ARMCORE_DCM_MASK))
			dev_err(clk->dev, "ARMCORE DCM rollback failed\n");
	} else {
		*value = old;
	}
	mt6797_hw_semaphore_put(clk);
out:
	mutex_unlock(&clk->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mt6797_mcuclk_armcore);

void mt6797_mcuclk_snapshot(struct mt6797_mcuclk *clk, bool big_active,
			    struct mt6797_mcuclk_snapshot *snapshot)
{
	mutex_lock(&clk->lock);
	*snapshot = (struct mt6797_mcuclk_snapshot) {
		.mux = readl_relaxed(clk->mcumixed + MT6797_PLL_MUXSEL),
		.ckdiv = readl_relaxed(clk->mcumixed + MT6797_PLL_CKDIV),
		.fhctl = fhctl, .ready = clk->fhctl_ready,
		.sync_dcm = sync_dcm, .hops = clk->hops,
		.hop_failures = clk->hop_failures, .dcm_updates = clk->dcm_updates,
	};
	snapshot->little_error = mt6797_smc_read(0x10220744, &snapshot->little_dcm);
	snapshot->big_error = big_active ?
		mt6797_smc_read(0x10222274, &snapshot->big_dcm) : -ENODEV;
	mutex_unlock(&clk->lock);
}
EXPORT_SYMBOL_GPL(mt6797_mcuclk_snapshot);

int mt6797_mcuclk_measure(struct mt6797_mcuclk *clk, unsigned int active,
			  struct mt6797_mcuclk_measurement results[5])
{
	static const unsigned int selectors[] = { 34, 35, 37, 36, 45 };
	int ret;

	mutex_lock(&clk->lock);
	ret = mt6797_hw_semaphore_get(clk);
	if (!ret) {
		for (unsigned int i = 0; i < ARRAY_SIZE(selectors); i++) {
			results[i].khz = 0;
			results[i].error = i < 3 && !(active & BIT(i)) ? -ENODEV :
				mt6797_measure_rate(clk, selectors[i], &results[i].khz);
		}
		mt6797_hw_semaphore_put(clk);
	}
	mutex_unlock(&clk->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mt6797_mcuclk_measure);

/* Read-only CCF outputs cannot bypass CPUFreq's voltage-vote transaction. */
static unsigned long mt6797_mcuclk_recalc(struct clk_hw *hw,
					  unsigned long parent_rate)
{
	struct mt6797_mcuclk_output *output =
		container_of(hw, struct mt6797_mcuclk_output, hw);

	return READ_ONCE(output->provider->cached_rate[output->id]);
}

static const struct clk_ops mt6797_mcuclk_ops = {
	.recalc_rate = mt6797_mcuclk_recalc,
};

static void __iomem *mt6797_mcuclk_iomap(struct platform_device *pdev,
					 const char *name)
{
	struct resource *res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	void __iomem *base;

	if (!res)
		return ERR_PTR(-EINVAL);
	base = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	return base ?: ERR_PTR(-ENOMEM);
}

struct mt6797_mcuclk *devm_mt6797_mcuclk_create(struct platform_device *pdev)
{
	static const char * const names[] = {
		"mt6797-cpu-ll", "mt6797-cpu-l", "mt6797-cpu-big", "mt6797-cci",
	};
	struct device *dev = &pdev->dev;
	struct mt6797_mcuclk *clk;
	int ret;

	clk = devm_kzalloc(dev, sizeof(*clk), GFP_KERNEL);
	if (!clk)
		return ERR_PTR(-ENOMEM);
	clk->dev = dev;
	mutex_init(&clk->lock);
	clk->mcumixed = mt6797_mcuclk_iomap(pdev, "mcumixed");
	if (IS_ERR(clk->mcumixed))
		return ERR_CAST(clk->mcumixed);
	clk->topckgen = mt6797_mcuclk_iomap(pdev, "topckgen");
	if (IS_ERR(clk->topckgen))
		return ERR_CAST(clk->topckgen);
	clk->semaphore = mt6797_mcuclk_iomap(pdev, "semaphore");
	if (IS_ERR(clk->semaphore))
		return ERR_CAST(clk->semaphore);
	clk->clock_data = devm_kzalloc(dev, struct_size(clk->clock_data, hws,
						       ARRAY_SIZE(names)), GFP_KERNEL);
	if (!clk->clock_data)
		return ERR_PTR(-ENOMEM);
	clk->clock_data->num = ARRAY_SIZE(names);
	for (unsigned int i = 0; i < ARRAY_SIZE(names); i++) {
		struct clk_init_data init = { .name = names[i],
			.ops = &mt6797_mcuclk_ops, .flags = CLK_GET_RATE_NOCACHE };
		struct mt6797_mcuclk_output *output = &clk->outputs[i];

		output->provider = clk;
		output->id = i;
		output->hw.init = &init;
		ret = devm_clk_hw_register(dev, &output->hw);
		if (ret)
			return ERR_PTR(ret);
		clk->clock_data->hws[i] = &output->hw;
	}
	if (of_property_present(dev->of_node, "#clock-cells")) {
		ret = devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get,
						 clk->clock_data);
		if (ret)
			return ERR_PTR(ret);
	}
	return clk;
}
EXPORT_SYMBOL_GPL(devm_mt6797_mcuclk_create);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MediaTek MT6797 CPU and CCI clock provider");
