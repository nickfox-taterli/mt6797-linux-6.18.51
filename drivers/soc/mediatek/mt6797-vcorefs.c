// SPDX-License-Identifier: GPL-2.0-only
/*
 * MT6797 SPM VCORE/DRAM DVFS, adapted from MediaTek's 2015 spm_v2 and
 * mt_vcorefs_governor. CPU VPROC/VSRAM remains owned by CPUfreq.
 */
#include <linux/devfreq.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mfd/syscon.h>
#include <linux/mutex.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/regmap.h>
#include <linux/soc/mediatek/mt6797-dcm.h>
#include <linux/unaligned.h>

#include "mt6797-spm-regs.h"

#define SPM_KEY			0x0b160000
#define SPM_WAIT_US		10000
#define SPM_RESET_STATE		0x48490
#define PCM_COMMON		BIT(10)
#define PCM_DVFS_BUSY		BIT(22)
#define PCM_DVFS_STATE		GENMASK(24, 23)
#define PWRAP_COMMANDS		16
#define FW_CODE_LIMIT		8192

static const u32 saved_offsets[] = {
	SPM_SRC_REQ, SPM_SRC_MASK, SPM_SRC2_MASK, SPM_AP_STANDBY_CON,
	SPM_CLK_CON, SPM_IRQ_MASK, SPM_WAKEUP_EVENT_MASK, CPU_DVFS_REQ,
	SPM_SW_FLAG, SPM_SW_RSV_0, SPM_SW_RSV_1, SPM_SW_RSV_4,
	SPM_SW_RSV_5, SPM_PASR_DPD_1, SPM_PASR_DPD_2, SPM_EMI_BW_MODE,
	SPM_POWER_ON_VAL1, SPM_MAS_PAUSE_MASK_B, SPM_MAS_PAUSE2_MASK_B,
	MP0_CPU0_WFI_EN, MP0_CPU1_WFI_EN, MP0_CPU2_WFI_EN, MP0_CPU3_WFI_EN,
	MP1_CPU0_WFI_EN, MP1_CPU1_WFI_EN, MP1_CPU2_WFI_EN, MP1_CPU3_WFI_EN,
	MP2_CPU0_WFI_EN, MP2_CPU1_WFI_EN,
};

struct mt6797_vcorefs {
	struct device *dev;
	void __iomem *spm, *phy, *dram, *pwrap;
	void __iomem *cpu_spm, *big_spm;
	struct mutex lock;
	struct devfreq *devfreq;
	struct regmap *pmic;
	struct device *dcm;
	struct clk *axi, *axi_high, *axi_low, *saved_axi;
	void *code;
	dma_addr_t code_dma;
	u16 words;
	u32 vectors[16];
	char version[128];
	u32 saved_pwrap[PWRAP_COMMANDS * 2];
	u32 saved_vcore[3];
	u32 saved_gpu_mode, saved_im_ptr, saved_im_len;
	u32 saved_con0, saved_con1;
	u32 saved_spm[ARRAY_SIZE(saved_offsets)];
	unsigned int initial_opp, current_opp;
	u32 dummy_address;
	u32 pcm_address, pcm_flags;
	bool resume_running;
	unsigned int resume_opp;
	bool running;
	bool stopped;
	unsigned long changes, failures;
	struct devfreq_dev_profile profile;
};

/* BSP HPM/LPM, nominal data rates 1600/1270 MT/s. The third ULPM
 * transition has not completed on this board's vendor DRAM initialization;
 * do not advertise a frequency which cannot be acknowledged by the PHY.
 */
static const unsigned long dram_rates[] = { 800000000, 635000000 };
static const u32 state_codes[] = { 3, 2 };

static u32 spm_read(struct mt6797_vcorefs *s, u32 reg)
{
	return readl(s->spm + reg);
}

static void spm_write(struct mt6797_vcorefs *s, u32 reg, u32 value)
{
	writel(value, s->spm + reg);
}

static int spm_wait(struct mt6797_vcorefs *s, u32 reg, u32 mask, u32 expect)
{
	u32 value;

	int ret = readl_poll_timeout(s->spm + reg, value,
				     (value & mask) == expect, 1, SPM_WAIT_US);

	if (ret)
		dev_err(s->dev, "PCM timeout reg=%x value=%08x mask=%08x expected=%08x pc=%x r6=%08x r13=%08x mode=%08x shuffle=%u\n",
			reg, value, mask, expect, readl(s->spm + PCM_REG15_DATA),
			readl(s->spm + PCM_REG6_DATA), readl(s->spm + 0x134),
			readl(s->spm + SPM_SW_RSV_1), readl(s->phy + 0x63c) & 3);
	return ret;
}

static unsigned int current_opp(struct mt6797_vcorefs *s)
{
	return readl(s->phy + 0x63c) & 3;
}

/* BSP get_dram_data_rate_from_reg(), before its advertised-rate rounding. */
static unsigned int dram_data_rate(struct mt6797_vcorefs *s, bool highest)
{
	static const u32 div_regs[] = { 0x42c, 0x448, 0x9c4 };
	static const u32 pll_regs[] = { 0x408, 0x644, 0xa44 };
	u32 div_index = highest ? 0 : (readl(s->dram + 0x28) >> 8) & 7;
	u32 pll_index = highest ? 0 : current_opp(s);
	u32 divider, pll, prediv, frequency;

	if ((readl(s->dram + 0x28) & BIT(17)) || div_index >= 3 || pll_index >= 3)
		return 0;
	divider = readl(s->phy + div_regs[div_index]);
	if (!(divider & BIT(17))) {
		pll = readl(s->phy + 0x400);
		prediv = (pll >> 18) & 3;
		if (prediv == 3)
			prediv = 2;
		frequency = (((((pll >> 24) & 0x7f) + 1) * 52) <<
			     ((pll >> 20) & 3)) >> prediv;
	} else {
		pll = readl(s->phy + pll_regs[pll_index]);
		frequency = (((((pll >> 24) & 0x7f) * 52) <<
			      ((readl(s->phy + 0x43c) >> 12) & 1)) >>
			     ((pll >> 18) & 3)) >> ((pll >> 16) & 3);
		frequency >>= 2;
	}
	return (frequency >> ((divider >> 10) & 3)) * 2;
}

static int load_firmware(struct mt6797_vcorefs *s)
{
	const struct firmware *fw;
	size_t offset, version_len;
	unsigned int i;
	int ret;

	ret = request_firmware(&fw, "mediatek/mt6797/pcm_vcorefs_lpm.bin", s->dev);
	if (ret)
		return ret;
	if (fw->size < 75) {
		ret = -EINVAL;
		goto release;
	}
	s->words = get_unaligned_le16(fw->data);
	offset = 2 + s->words * sizeof(u32);
	if (!s->words || offset > FW_CODE_LIMIT + 2 ||
	    offset + 72 >= fw->size ||
	    get_unaligned_le16(fw->data + offset) != s->words ||
	    fw->data[offset + 2] != 2 || fw->data[offset + 3] != 0 ||
	    get_unaligned_le32(fw->data + offset + 4)) {
		ret = -EINVAL;
		goto release;
	}
	version_len = strnlen(fw->data + offset + 72, fw->size - offset - 72);
	if (!version_len || version_len >= sizeof(s->version) ||
	    offset + 72 + version_len + 1 != fw->size ||
	    !str_has_prefix((const char *)fw->data + offset + 72,
			    "pcm_vcorefs_lpm_v10p7_")) {
		ret = -EINVAL;
		goto release;
	}
	for (i = 0; i < ARRAY_SIZE(s->vectors); i++)
		s->vectors[i] = get_unaligned_le32(fw->data + offset + 8 + i * 4);
	s->code = dmam_alloc_coherent(s->dev, FW_CODE_LIMIT, &s->code_dma, GFP_KERNEL);
	if (!s->code) {
		ret = -ENOMEM;
		goto release;
	}
	memcpy(s->code, fw->data + 2, s->words * sizeof(u32));
	memcpy(s->version, fw->data + offset + 72, version_len + 1);
	ret = 0;
release:
	release_firmware(fw);
	return ret;
}

static void program_pwrap(struct mt6797_vcorefs *s)
{
	/* BSP normal phase, VCORE only; no CPU VPROC/VSRAM commands. */
	static const u32 microvolts[] = { 1000000, 983330, 966660, 949990,
		933330, 916660, 900000, 900000, 900000, 900000 };
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(microvolts); i++) {
		writel(0x060a, s->pwrap + 0x10c + i * 8);
		writel(DIV_ROUND_UP(microvolts[i] - 600000, 6250),
		       s->pwrap + 0x110 + i * 8);
	}
	writel(0x0462, s->pwrap + 0x10c + 10 * 8);
	writel(6, s->pwrap + 0x110 + 10 * 8);
	writel(0x0462, s->pwrap + 0x10c + 11 * 8);
	writel(4, s->pwrap + 0x110 + 11 * 8);
}

static int stop_firmware(struct mt6797_vcorefs *s)
{
	u32 value;
	int ret;

	value = spm_read(s, SPM_SW_RSV_1);
	spm_write(s, SPM_SW_RSV_1, (value & ~0xf) | 0xf);
	spm_write(s, SPM_CPU_WAKEUP_EVENT, 1);
	ret = spm_wait(s, SPM_IRQ_STA, BIT(3), BIT(3));
	if (ret)
		return ret;
	spm_write(s, SPM_CPU_WAKEUP_EVENT, 0);
	/* Keep the current memory controls when the PCM relinquishes r0/r7. */
	spm_write(s, SPM_POWER_ON_VAL0, spm_read(s, PCM_REG0_DATA));
	spm_write(s, PCM_PWR_IO_EN, 0);
	spm_write(s, PCM_CON1, SPM_KEY | (spm_read(s, PCM_CON1) & ~BIT(5)));
	spm_write(s, SPM_IRQ_STA, BIT(3));
	/* Offload acknowledgement alone does not stop instruction DMA. */
	spm_write(s, PCM_CON0, SPM_KEY | BIT(2) | BIT(15));
	ret = spm_wait(s, PCM_FSM_STA, 0x7fffff, SPM_RESET_STATE);
	if (ret)
		return ret;
	spm_write(s, PCM_CON0, SPM_KEY | BIT(2));
	return 0;
}

static int restore_state(struct mt6797_vcorefs *s)
{
	unsigned int i;
	int ret;

	ret = regmap_write(s->pmic, 0x0608, s->saved_vcore[0]);
	if (ret)
		return ret;
	ret = regmap_write(s->pmic, 0x0600, s->saved_vcore[2]);
	if (ret)
		return ret;
	ret = regmap_write(s->pmic, 0x060a, s->saved_vcore[1]);
	if (ret)
		return ret;
	ret = regmap_write(s->pmic, 0x0462, s->saved_gpu_mode);
	if (ret)
		return ret;
	for (i = 0; i < ARRAY_SIZE(s->saved_pwrap); i++)
		writel(s->saved_pwrap[i], s->pwrap + 0x10c + i * 4);
	for (i = 0; i < ARRAY_SIZE(saved_offsets); i++)
		spm_write(s, saved_offsets[i], s->saved_spm[i]);
	spm_write(s, PCM_IM_PTR, s->saved_im_ptr);
	spm_write(s, PCM_IM_LEN, s->saved_im_len);
	spm_write(s, PCM_CON0, SPM_KEY | (s->saved_con0 & 0xffff));
	spm_write(s, PCM_CON1, SPM_KEY | (s->saved_con1 & 0xffff));
	return 0;
}

static int start_firmware(struct mt6797_vcorefs *s)
{
	u32 value;
	unsigned int i;
	int ret;

	/* Refuse to replace an unknown active PCM program. */
	if ((spm_read(s, PCM_FSM_STA) & 0x7fffff) != SPM_RESET_STATE)
		return -EBUSY;
	/* Common PCM only uses CPU SPM semaphore ports. Refuse an active CPU
	 * DVFS firmware, which would be a second owner of the manual rails.
	 */
	if ((readl(s->cpu_spm + 0x178) & 0x7fffff) != SPM_RESET_STATE ||
	    (readl(s->big_spm + 0x178) & 0x7fffff) != SPM_RESET_STATE)
		return -EBUSY;
	ret = regmap_read(s->pmic, 0x0608, &s->saved_vcore[0]);
	if (ret)
		return ret;
	ret = regmap_read(s->pmic, 0x060a, &s->saved_vcore[1]);
	if (ret)
		return ret;
	ret = regmap_read(s->pmic, 0x0600, &s->saved_vcore[2]);
	if (ret)
		return ret;
	ret = regmap_read(s->pmic, 0x0462, &s->saved_gpu_mode);
	if (ret)
		return ret;
	s->saved_im_ptr = spm_read(s, PCM_IM_PTR);
	s->saved_im_len = spm_read(s, PCM_IM_LEN);
	s->saved_con0 = spm_read(s, PCM_CON0);
	s->saved_con1 = spm_read(s, PCM_CON1);
	for (i = 0; i < ARRAY_SIZE(saved_offsets); i++)
		s->saved_spm[i] = spm_read(s, saved_offsets[i]);
	for (i = 0; i < ARRAY_SIZE(s->saved_pwrap); i++)
		s->saved_pwrap[i] = readl(s->pwrap + 0x10c + i * 4);
	spm_write(s, POWERON_CONFIG_EN, SPM_KEY | 1);
	spm_write(s, SPM_POWER_ON_VAL1, 0x00015820);
	spm_write(s, PCM_PWR_IO_EN, 0);
	spm_write(s, SPM_CLK_CON, spm_read(s, SPM_CLK_CON) |
		  BIT(2) | BIT(3) | BIT(6) | BIT(12) | BIT(27) | BIT(16));
	spm_write(s, SPM_IRQ_STA, BIT(2) | BIT(3));
	spm_write(s, SPM_SW_INT_CLEAR, GENMASK(9, 0));
	spm_write(s, PCM_CON0, SPM_KEY | BIT(2) | BIT(15));
	ret = spm_wait(s, PCM_FSM_STA, 0x7fffff, SPM_RESET_STATE);
	if (ret)
		goto restore;
	spm_write(s, PCM_CON0, SPM_KEY | BIT(2));
	spm_write(s, PCM_CON0, SPM_KEY | BIT(2) | BIT(3));
	spm_write(s, PCM_CON1, SPM_KEY | BIT(12) | BIT(11) | BIT(10) |
		  BIT(6) | BIT(3) | BIT(14) |
		  (spm_read(s, PCM_CON1) & (BIT(8) | BIT(9))));
	program_pwrap(s);
	/* Select VOSEL_ON only after matching the active software selector. */
	if (!(s->saved_vcore[2] & BIT(1))) {
		ret = regmap_update_bits(s->pmic, 0x060a, 0x7f,
					 s->saved_vcore[0] & 0x7f);
		if (ret)
			goto restore;
	}
	ret = regmap_update_bits(s->pmic, 0x0600, BIT(1), BIT(1));
	if (ret)
		goto restore;
	/* BSP initializes the wrapper acknowledgement before starting the PCM. */
	spm_write(s, SPM_DVFS_CON, 0);
	ret = spm_wait(s, SPM_DVFS_CON, BIT(31), BIT(31));
	if (ret)
		goto restore;
	spm_write(s, SPM_PASR_DPD_1, s->dummy_address);
	spm_write(s, SPM_PASR_DPD_2, s->dummy_address);
	/* The BSP preserves R6 twice: voltage state in [1:0], DRAM state in
	 * [24:23]. The vendor boot chain can leave stale HPM in the latter
	 * while actually booting shuffle 1. Seed both from the physical PHY.
	 */
	value = state_codes[current_opp(s)];
	spm_write(s, SPM_SW_RSV_5,
		  (s->saved_spm[12] & ~(3 | PCM_DVFS_STATE)) | value | (value << 23));
	dma_wmb();
	spm_write(s, PCM_IM_PTR, s->pcm_address);
	spm_write(s, PCM_IM_LEN, s->words - 1);
	spm_write(s, PCM_CON0, SPM_KEY | BIT(2) | BIT(3) | BIT(1));
	spm_write(s, PCM_CON0, SPM_KEY | BIT(2) | BIT(3));
	spm_write(s, PCM_REG_DATA_INI, spm_read(s, SPM_POWER_ON_VAL0));
	spm_write(s, PCM_PWR_IO_EN, BIT(16));
	spm_write(s, PCM_PWR_IO_EN, 0);
	spm_write(s, PCM_REG_DATA_INI, spm_read(s, SPM_POWER_ON_VAL1));
	spm_write(s, PCM_PWR_IO_EN, BIT(23));
	spm_write(s, PCM_PWR_IO_EN, 0);
	for (i = 0; i < ARRAY_SIZE(s->vectors); i++)
		spm_write(s, PCM_EVENT_VECTOR0 + i * 4, s->vectors[i]);
	/* BSP vcorefs pwr_ctrl; its command bank has no CPU rail commands. */
	spm_write(s, SPM_AP_STANDBY_CON, BIT(20) | BIT(19) |
		  (s->saved_spm[3] & BIT(23)));
	spm_write(s, SPM_SRC_REQ, PCM_COMMON | BIT(6) | BIT(5));
	spm_write(s, SPM_SRC_MASK, BIT(28));
	spm_write(s, SPM_SRC2_MASK, BIT(26) | BIT(8) | 7);
	for (i = 0; i < 10; i++)
		spm_write(s, MP0_CPU0_WFI_EN + i * 4, 0);
	spm_write(s, SPM_EMI_BW_MODE, spm_read(s, SPM_EMI_BW_MODE) & ~3);
	spm_write(s, PCM_TIMER_VAL, 0xffffffff - 30 * 32768);
	spm_write(s, PCM_CON1, spm_read(s, PCM_CON1) | SPM_KEY | BIT(5));
	spm_write(s, SPM_WAKEUP_EVENT_MASK, ~(u32)(BIT(0) | BIT(3) | BIT(10)));
	spm_write(s, SPM_IRQ_MASK, (spm_read(s, SPM_IRQ_MASK) & BIT(2)) | GENMASK(17, 9));
	spm_write(s, SPM_MAS_PAUSE_MASK_B, 0xffffffff);
	spm_write(s, SPM_MAS_PAUSE2_MASK_B, 0xffffffff);
	spm_write(s, PCM_REG_DATA_INI, 0);
	spm_write(s, SPM_SW_FLAG, s->pcm_flags);
	spm_write(s, SPM_SW_RSV_0, 0);
	spm_write(s, SPM_CLK_CON, spm_read(s, SPM_CLK_CON) & ~(BIT(5) | BIT(6)));
	spm_write(s, PCM_PWR_IO_EN, BIT(0) | BIT(7));
	/* Hold the module and DMA buffer even if the start handshake times out. */
	s->running = true;
	__module_get(THIS_MODULE);
	value = spm_read(s, PCM_CON0) & ~(BIT(0) | BIT(1));
	spm_write(s, PCM_CON0, value | SPM_KEY | BIT(2) | BIT(0));
	spm_write(s, PCM_CON0, value | SPM_KEY | BIT(2));
	ret = spm_wait(s, PCM_REG1_DATA, 0xffffffff, 1);
	if (ret)
		return ret;
	return 0;
restore:
	if (restore_state(s))
		dev_err(s->dev, "VCORE/PCM start rollback failed\n");
	return ret;
}

static int request_opp(struct mt6797_vcorefs *s, unsigned int opp)
{
	u32 value;
	int ret;

	ret = spm_wait(s, PCM_REG6_DATA, PCM_DVFS_BUSY, 0);
	if (ret)
		return ret;
	value = spm_read(s, SPM_SW_RSV_1);
	spm_write(s, SPM_SW_RSV_1, (value & ~0xf) | 1);
	spm_write(s, SPM_CPU_WAKEUP_EVENT, 1);
	ret = spm_wait(s, SPM_SW_RSV_1, 0xf, 0xb);
	if (ret)
		return ret;
	/* BSP NORMAL uses MASK_MD_DVFS_REQ=0. */
	value = spm_read(s, SPM_SW_RSV_1) & ~BIT(4);
	spm_write(s, SPM_SW_RSV_1, value);
	spm_write(s, SPM_CPU_WAKEUP_EVENT, 0);
	spm_write(s, SPM_SW_RSV_1, (spm_read(s, SPM_SW_RSV_1) & ~0xf) | 0xa);
	value = spm_read(s, SPM_SRC_REQ) & ~BIT(5);
	spm_write(s, SPM_SRC_REQ, value | (opp == 0 ? BIT(5) : 0));
	/* Confirm both the PCM acknowledgement and the physical PHY shuffle. */
	ret = read_poll_timeout(spm_read, value,
		!(value & PCM_DVFS_BUSY) &&
		(value & PCM_DVFS_STATE) == (state_codes[opp] << 23) &&
		current_opp(s) == opp, 1, SPM_WAIT_US, false,
		s, PCM_REG6_DATA);
	if (ret)
		dev_err(s->dev, "DRAM transition incomplete: target=%u r6=%08x shuffle=%u ddr=%u MT/s\n",
			opp, value, current_opp(s), dram_data_rate(s, false));
	return ret;
}

struct vcorefs_transition {
	struct mt6797_vcorefs *s;
	unsigned int index;
	bool release;
};

static int release_firmware_locked(struct mt6797_vcorefs *s);

static int vcorefs_transition(void *data, bool *quiescent)
{
	struct vcorefs_transition *transition = data;
	struct mt6797_vcorefs *s = transition->s;
	unsigned int index = transition->index;
	int ret;

	if (transition->release) {
		ret = release_firmware_locked(s);
		goto done;
	}
	if (s->stopped) {
		ret = -EIO;
		goto done;
	}
	if (!s->running)
		s->saved_axi = clk_get_parent(s->axi);
	/* BSP: lower AXI before lowering VCORE; raise after VCORE/DDR. */
	if (index) {
		ret = clk_set_parent(s->axi, s->axi_low);
		if (ret)
			goto done;
	}
	if (!s->running)
		ret = start_firmware(s);
	else
		ret = 0;
	if (!ret)
		ret = request_opp(s, index);
	if (!ret && !index)
		ret = clk_set_parent(s->axi, s->axi_high);
	if (!ret) {
		s->current_opp = index;
		s->changes++;
	} else {
		s->failures++;
		if (s->running && request_opp(s, s->current_opp))
			dev_err(s->dev, "DRAM DVFS recovery failed\n");
		else if (clk_set_parent(s->axi, s->running ?
				 (s->current_opp ? s->axi_low : s->axi_high) :
				 s->saved_axi))
			dev_err(s->dev, "AXI DVFS recovery failed\n");
	}
done:
	*quiescent = !s->running || s->stopped ||
		(!(spm_read(s, PCM_REG6_DATA) & PCM_DVFS_BUSY) &&
		 current_opp(s) == s->current_opp &&
		 (spm_read(s, PCM_REG6_DATA) & PCM_DVFS_STATE) ==
			(state_codes[s->current_opp] << 23));
	return ret;
}

static int vcorefs_target(struct device *dev, unsigned long *freq, u32 flags)
{
	struct mt6797_vcorefs *s = dev_get_drvdata(dev);
	struct vcorefs_transition transition = { .s = s };
	struct dev_pm_opp *opp;
	int ret;

	opp = devfreq_recommended_opp(dev, freq, flags);
	if (IS_ERR(opp))
		return PTR_ERR(opp);
	dev_pm_opp_put(opp);
	for (; transition.index < ARRAY_SIZE(dram_rates); transition.index++)
		if (dram_rates[transition.index] == *freq)
			break;
	if (transition.index == ARRAY_SIZE(dram_rates))
		return -EINVAL;
	mutex_lock(&s->lock);
	if (!s->running &&
	    (spm_read(s, PCM_FSM_STA) & 0x7fffff) != SPM_RESET_STATE)
		ret = -EBUSY;
	else
		ret = mt6797_dcm_memory_transaction(s->dcm, vcorefs_transition, &transition);
	mutex_unlock(&s->lock);
	return ret;
}

static int vcorefs_get_cur_freq(struct device *dev, unsigned long *freq)
{
	struct mt6797_vcorefs *s = dev_get_drvdata(dev);
	unsigned int index;

	mutex_lock(&s->lock);
	index = current_opp(s);
	mutex_unlock(&s->lock);
	if (index >= ARRAY_SIZE(dram_rates))
		return -EIO;
	*freq = dram_rates[index];
	return 0;
}

static ssize_t state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mt6797_vcorefs *s = dev_get_drvdata(dev);
	ssize_t len;

	mutex_lock(&s->lock);
	len = sysfs_emit(buf, "running=%u stopped=%u shuffle=%u ddr_MT_s=%u highest_ddr_MT_s=%u axi_Hz=%lu initial=%u changes=%lu failures=%lu cpu_voltage_owner=cpufreq firmware=%s r1=%08x r6=%08x flags=%08x im_ptr=%08x im_len=%u\n",
			 s->running, s->stopped, current_opp(s), dram_data_rate(s, false),
			 dram_data_rate(s, true), clk_get_rate(s->axi), s->initial_opp, s->changes,
			 s->failures, s->version, spm_read(s, PCM_REG1_DATA),
			 spm_read(s, PCM_REG6_DATA), spm_read(s, SPM_SW_FLAG),
			 spm_read(s, PCM_IM_PTR), spm_read(s, PCM_IM_LEN));
	mutex_unlock(&s->lock);
	return len;
}
static DEVICE_ATTR_RO(state);

static int release_firmware_locked(struct mt6797_vcorefs *s)
{
	int ret;

	lockdep_assert_held(&s->devfreq->lock);
	lockdep_assert_held(&s->lock);
	if (!s->running)
		return 0;
	if (!s->stopped) {
		ret = request_opp(s, s->initial_opp);
		if (ret)
			return ret;
		ret = stop_firmware(s);
		if (ret)
			return ret;
		s->stopped = true;
	}
	/* Both normal parents are valid at the saved baseline VCORE. */
	ret = clk_set_parent(s->axi, s->saved_axi);
	if (!ret)
		ret = restore_state(s);
	if (ret)
		return ret;
	s->running = false;
	s->stopped = false;
	s->current_opp = s->initial_opp;
	s->devfreq->previous_freq = dram_rates[s->initial_opp];
	module_put(THIS_MODULE);
	return 0;
}

static ssize_t release_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct mt6797_vcorefs *s = dev_get_drvdata(dev);
	struct vcorefs_transition transition = { .s = s, .release = true };
	int ret;

	if (!sysfs_streq(buf, "1"))
		return -EINVAL;
	mutex_lock(&s->devfreq->lock);
	mutex_lock(&s->lock);
	ret = mt6797_dcm_memory_transaction(s->dcm, vcorefs_transition, &transition);
	mutex_unlock(&s->lock);
	mutex_unlock(&s->devfreq->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(release);

static struct attribute *vcorefs_attrs[] = {
	&dev_attr_state.attr, &dev_attr_release.attr, NULL,
};
ATTRIBUTE_GROUPS(vcorefs);

static void unmap_spm(void *data)
{
	iounmap(data);
}

static void put_provider(void *data)
{
	put_device(data);
}

static int vcorefs_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mt6797_vcorefs *s;
	struct device_node *np;
	struct resource res;
	struct resource *pwrap_res;
	struct platform_device *wrapper;
	struct platform_device *dcm;
	struct regmap *infra;
	struct clk_bulk_data *clocks;
	void __iomem *chip;
	u32 mapping, revision;
	unsigned int i;
	int ret;

	s = devm_kzalloc(dev, sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;
	s->dev = dev;
	mutex_init(&s->lock);
	ret = devm_clk_bulk_get_all_enabled(dev, &clocks);
	if (ret < 0)
		return ret;
	if (ret != 8)
		return -EINVAL;
	s->axi = devm_clk_get(dev, "axi");
	s->axi_high = devm_clk_get(dev, "axi-high");
	s->axi_low = devm_clk_get(dev, "axi-low");
	if (IS_ERR(s->axi) || IS_ERR(s->axi_high) || IS_ERR(s->axi_low))
		return -EINVAL;
	if (clk_get_rate(s->axi_low) <= 26000000 ||
	    clk_get_rate(s->axi_low) > clk_get_rate(s->axi_high))
		return dev_err_probe(dev, -EINVAL, "unresolved AXI clock parents\n");
	np = of_parse_phandle(dev->of_node, "mediatek,dcm", 0);
	if (!np)
		return -EINVAL;
	dcm = of_find_device_by_node(np);
	of_node_put(np);
	if (!dcm)
		return -EPROBE_DEFER;
	ret = devm_add_action_or_reset(dev, put_provider, &dcm->dev);
	if (ret)
		return ret;
	if (!device_is_bound(&dcm->dev))
		return -EPROBE_DEFER;
	s->dcm = &dcm->dev;
	if (!device_link_add(dev, s->dcm, DL_FLAG_AUTOREMOVE_CONSUMER))
		return -ENOMEM;
	infra = syscon_regmap_lookup_by_phandle(dev->of_node, "mediatek,infracfg");
	if (IS_ERR(infra))
		return PTR_ERR(infra);
	ret = regmap_read(infra, 0xf00, &mapping);
	if (ret)
		return ret;
	chip = devm_platform_ioremap_resource_byname(pdev, "chip-version");
	if (IS_ERR(chip))
		return PTR_ERR(chip);
	revision = readl(chip);
	if (revision != 0xca00 && revision != 0xca01)
		return -ENODEV;
	s->pcm_flags = PCM_COMMON |
		(revision == 0xca01 ? BIT(18) : 0);
	np = of_parse_phandle(dev->of_node, "mediatek,pmic-wrapper", 0);
	if (!np)
		return -EINVAL;
	wrapper = of_find_device_by_node(np);
	of_node_put(np);
	if (!wrapper)
		return -EPROBE_DEFER;
	ret = devm_add_action_or_reset(dev, put_provider, &wrapper->dev);
	if (ret)
		return ret;
	s->pmic = dev_get_regmap(&wrapper->dev, NULL);
	if (!s->pmic)
		return -EPROBE_DEFER;
	if (!device_link_add(dev, &wrapper->dev, DL_FLAG_AUTOREMOVE_CONSUMER))
		return -ENOMEM;
	np = of_parse_phandle(dev->of_node, "mediatek,spm", 0);
	if (!np)
		return -EINVAL;
	s->spm = of_iomap(np, 0);
	of_node_put(np);
	if (!s->spm)
		return -ENOMEM;
	ret = devm_add_action_or_reset(dev, unmap_spm, s->spm);
	if (ret)
		return ret;
	s->dram = devm_platform_ioremap_resource_byname(pdev, "dramc");
	s->phy = devm_platform_ioremap_resource_byname(pdev, "ddrphy");
	s->cpu_spm = devm_platform_ioremap_resource_byname(pdev, "cpu-spm");
	s->big_spm = devm_platform_ioremap_resource_byname(pdev, "big-spm");
	if (IS_ERR(s->cpu_spm) || IS_ERR(s->big_spm))
		return -ENODEV;
	/* PWRAP DVFS command bank is disjoint from its driver's WACS accesses. */
	pwrap_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "pwrap-dvfs");
	if (!pwrap_res || pwrap_res->start < 0x10c)
		return -EINVAL;
	res = *pwrap_res;
	s->pwrap = devm_ioremap(dev, res.start - 0x10c, 0x200);
	if (IS_ERR(s->dram) || IS_ERR(s->phy) || !s->pwrap)
		return -ENODEV;
	/* This firmware/profile is for the board's LPDDR3-1600 configuration. */
	if (dram_data_rate(s, true) != 1560 && dram_data_rate(s, true) != 1586)
		return dev_err_probe(dev, -ENODEV, "unsupported DRAM PLL profile\n");
	np = of_parse_phandle(dev->of_node, "memory-region", 0);
	ret = np ? of_address_to_resource(np, 0, &res) : -EINVAL;
	of_node_put(np);
	if (ret || resource_size(&res) < PAGE_SIZE || upper_32_bits(res.start))
		return -EINVAL;
	s->dummy_address = res.start;
	s->initial_opp = current_opp(s);
	if (s->initial_opp >= ARRAY_SIZE(dram_rates))
		return -EINVAL;
	s->current_opp = s->initial_opp;
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;
	ret = load_firmware(s);
	if (ret)
		return dev_err_probe(dev, ret, "SPM firmware rejected\n");
	s->pcm_address = lower_32_bits(s->code_dma);
	/* BSP enable_4G()/MAPPING_DRAM_ACCESS_ADDR(), SPM's bus address. */
	if (mapping & BIT(13)) {
		if (s->pcm_address >= 0x40000000 && s->pcm_address < 0xc0000000)
			s->pcm_address += 0x40000000;
		if (s->dummy_address >= 0x40000000 && s->dummy_address < 0xc0000000)
			s->dummy_address += 0x40000000;
	}
	platform_set_drvdata(pdev, s);
	for (i = 0; i < ARRAY_SIZE(dram_rates); i++) {
		ret = dev_pm_opp_add(dev, dram_rates[i], i ? 900000 : 1000000);
		if (ret)
			goto remove_opps;
	}
	s->profile.initial_freq = dram_rates[s->initial_opp];
	s->profile.target = vcorefs_target;
	s->profile.get_cur_freq = vcorefs_get_cur_freq;
	s->devfreq = devm_devfreq_add_device(dev, &s->profile, "userspace", NULL);
	if (IS_ERR(s->devfreq)) {
		ret = PTR_ERR(s->devfreq);
		goto remove_opps;
	}
	dev_info(dev, "%s; shuffle=%u, CPU voltages retained\n",
		 s->version, s->initial_opp);
	return 0;
remove_opps:
	dev_pm_opp_remove_all_dynamic(dev);
	return ret;
}

static void vcorefs_remove(struct platform_device *pdev)
{
	struct mt6797_vcorefs *s = platform_get_drvdata(pdev);

	devm_devfreq_remove_device(&pdev->dev, s->devfreq);
	dev_pm_opp_remove_all_dynamic(&pdev->dev);
}

static int vcorefs_suspend(struct device *dev)
{
	struct mt6797_vcorefs *s = dev_get_drvdata(dev);
	struct vcorefs_transition transition = { .s = s, .release = true };
	int ret;

	ret = devfreq_suspend_device(s->devfreq);
	if (ret)
		return ret;
	mutex_lock(&s->devfreq->lock);
	mutex_lock(&s->lock);
	s->resume_running = s->running;
	s->resume_opp = s->current_opp;
	ret = mt6797_dcm_memory_transaction(s->dcm, vcorefs_transition, &transition);
	mutex_unlock(&s->lock);
	mutex_unlock(&s->devfreq->lock);
	if (ret)
		devfreq_resume_device(s->devfreq);
	return ret;
}

static int vcorefs_resume(struct device *dev)
{
	struct mt6797_vcorefs *s = dev_get_drvdata(dev);
	unsigned long frequency = dram_rates[s->resume_opp];
	int ret = 0;

	mutex_lock(&s->devfreq->lock);
	if (s->resume_running) {
		ret = vcorefs_target(dev, &frequency, 0);
		if (!ret)
			s->devfreq->previous_freq = frequency;
	}
	mutex_unlock(&s->devfreq->lock);
	if (!ret)
		ret = devfreq_resume_device(s->devfreq);
	return ret;
}
static DEFINE_SIMPLE_DEV_PM_OPS(vcorefs_pm_ops, vcorefs_suspend, vcorefs_resume);

static void vcorefs_shutdown(struct platform_device *pdev)
{
	struct mt6797_vcorefs *s = platform_get_drvdata(pdev);
	struct vcorefs_transition transition = { .s = s, .release = true };
	int ret;

	mutex_lock(&s->devfreq->lock);
	mutex_lock(&s->lock);
	ret = mt6797_dcm_memory_transaction(s->dcm, vcorefs_transition, &transition);
	mutex_unlock(&s->lock);
	mutex_unlock(&s->devfreq->lock);
	if (ret)
		dev_err(&pdev->dev, "PCM release at shutdown failed: %d\n", ret);
}

static const struct of_device_id vcorefs_of_match[] = {
	{ .compatible = "mediatek,mt6797-vcorefs" }, {},
};
MODULE_DEVICE_TABLE(of, vcorefs_of_match);

static struct platform_driver vcorefs_driver = {
	.probe = vcorefs_probe,
	.remove = vcorefs_remove,
	.shutdown = vcorefs_shutdown,
	.driver = {
		.name = "mt6797-vcorefs",
		.of_match_table = vcorefs_of_match,
		.dev_groups = vcorefs_groups,
		.pm = pm_sleep_ptr(&vcorefs_pm_ops),
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(vcorefs_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MT6797 BSP SPM VCORE and DRAM DVFS");
