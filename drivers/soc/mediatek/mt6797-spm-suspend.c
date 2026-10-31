// SPDX-License-Identifier: GPL-2.0-only
/* MT6797 BSP spm_v2 suspend, with manual CPU rails retained.
 * VCORE relinquishes its common PCM before this driver borrows SPM.
 */
#include <linux/soc/mediatek/mt6797-sip.h>
#include <linux/arm-smccc.h>
#include <linux/clk.h>
#include <linux/cpu.h>
#include <linux/cpu_pm.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeup.h>
#include <linux/psci.h>
#include <linux/regmap.h>
#include <linux/soc/mediatek/mt6797-dcm.h>
#include <linux/soc/mediatek/mt6797-vcorefs.h>
#include <linux/soc/mediatek/mt6797-wdt.h>
#include <linux/suspend.h>
#include <linux/unaligned.h>

#include "mt6797-spm-regs.h"

#define RESET_STATE 0x48490
#define WAIT_US 10000
/* CPU/DDRPHY/INFRA powerdown requested. Keep manual VCORE/VPROC policy,
 * SYSRAM and DPD disabled; bus clocks may stop as in the BSP suspend path.
 */
#define SLEEP_FLAGS (BIT(3) | BIT(4) | BIT(5) | BIT(9) | BIT(16) | BIT(18))
#define FW_LIMIT 8192

static const u32 offsets[] = {
	SPM_POWER_ON_VAL1,     SPM_CLK_CON,	      0x010,
	SPM_AP_STANDBY_CON,    SPM_SRC_REQ,	      SPM_SRC_MASK,
	SPM_SRC2_MASK,	       SPM_WAKEUP_EVENT_MASK, SPM_IRQ_MASK,
	CPU_DVFS_REQ,	       SPM_SW_FLAG,	      SPM_SW_RSV_0,
	SPM_SW_RSV_1,	       SPM_SW_RSV_4,	      SPM_SW_RSV_5,
	SPM_PASR_DPD_1,	       SPM_PASR_DPD_2,	      SPM_MAS_PAUSE_MASK_B,
	SPM_MAS_PAUSE2_MASK_B, PCM_TIMER_VAL,	      PCM_WDT_VAL,
	PCM_REG_DATA_INI,
	MP0_CPU0_WFI_EN,       MP0_CPU1_WFI_EN,	      MP0_CPU2_WFI_EN,
	MP0_CPU3_WFI_EN,       MP1_CPU0_WFI_EN,	      MP1_CPU1_WFI_EN,
	MP1_CPU2_WFI_EN,       MP1_CPU3_WFI_EN,	      MP2_CPU0_WFI_EN,
	MP2_CPU1_WFI_EN,
};
static const u32 pmic_regs[] = {
	0x0600, 0x0608, 0x060a, 0x0204, 0x045c, 0x06a8
};
static struct {
	struct device *dev, *vcorefs, *dcm, *watchdog;
	void __iomem *spm, *pwrap, *phy, *cirq, *gic;
	phys_addr_t polarity[2];
	u32 cirq_saved[3][7], cirq_control, cirq_pending[7];
	unsigned long cirq_cycles, cirq_flushed;
	bool cirq_enabled;
	struct regmap *pmic;
	void *code;
	dma_addr_t dma;
	u32 pcm_address, dummy_address;
	unsigned int words;
	u32 vectors[16], regs[ARRAY_SIZE(offsets)];
	u32 pmic_saved[ARRAY_SIZE(pmic_regs)], pwrap_saved[32], vector_saved[16];
	u32 con0, con1, ptr, len;
	u32 r12, raw, debug, timer, r13, flags;
	u32 wake_after_ms, cycle_wake_ms;
	u32 wdt_value, wdt_ticks, wdt_con1;
	unsigned long cycles, resumes, failures;
	int irq, last_error;
	bool prepared, active, safe, owned, dcm_lease, module_held, pinned;
	bool irq_enabled, wdt_armed, wdt_request_owned;
} s;

static u32 rd(u32 offset)
{
	return readl(s.spm + offset);
}
static void wr(u32 offset, u32 value)
{
	writel(value, s.spm + offset);
}
static int wait(u32 offset, u32 mask, u32 value)
{
	u32 actual;

	return readl_poll_timeout_atomic(s.spm + offset, actual,
					 (actual & mask) == value, 1, WAIT_US);
}

static int pcm_wdt_disarm(void)
{
	if (!s.wdt_armed)
		return 0;

	s.wdt_con1 = rd(PCM_CON1);
	s.wdt_ticks = rd(PCM_WDT_OUT);
	wr(PCM_CON1, SPM_KEY | (s.wdt_con1 & ~PCM_WDT_EN));
	if (rd(PCM_CON1) & PCM_WDT_EN)
		return -EIO;
	s.wdt_armed = false;
	return 0;
}

static int pcm_wdt_arm(void)
{
	u32 con1, timer, value;
	int ret;

	if (s.wdt_armed || s.wdt_request_owned)
		return -EBUSY;
	con1 = rd(PCM_CON1) & ~(PCM_WDT_EN | PCM_WDT_WAKE_MODE);
	wr(PCM_CON1, SPM_KEY | con1);
	if (rd(PCM_CON1) & (PCM_WDT_EN | PCM_WDT_WAKE_MODE))
		return -EIO;
	timer = rd(PCM_TIMER_VAL);
	if (timer > PCM_TIMER_MAX) {
		timer = PCM_TIMER_MAX;
		wr(PCM_TIMER_VAL, timer);
	}
	value = timer + PCM_WDT_TIMEOUT;
	wr(PCM_WDT_VAL, value);
	wr(PCM_CON1, SPM_KEY | con1 | PCM_WDT_EN);
	s.wdt_armed = true;
	s.wdt_value = value;
	s.wdt_ticks = rd(PCM_WDT_OUT);
	s.wdt_con1 = rd(PCM_CON1);
	if (rd(PCM_TIMER_VAL) != timer || rd(PCM_WDT_VAL) != value ||
	    (s.wdt_con1 & (PCM_WDT_EN | PCM_WDT_WAKE_MODE)) != PCM_WDT_EN) {
		ret = pcm_wdt_disarm();
		return ret ?: -EIO;
	}
	return 0;
}

static int spm_wdt_request(bool enable)
{
	int ret;

	if (enable) {
		if (s.wdt_request_owned)
			return -EBUSY;
		s.wdt_request_owned = true;
		return mt6797_wdt_reset_request(s.watchdog, BIT(1), true);
	}
	if (!s.wdt_request_owned)
		return 0;
	ret = mt6797_wdt_reset_request(s.watchdog, BIT(1), false);
	if (!ret)
		s.wdt_request_owned = false;
	return ret;
}

static int stop_pcm(void)
{
	int ret;
	/* Retain the current R0 memory controls, then stop instruction fetch. */
	wr(SPM_POWER_ON_VAL0, rd(PCM_REG0_DATA));
	wr(PCM_PWR_IO_EN, 0);
	wr(PCM_CON1, SPM_KEY | (rd(PCM_CON1) & ~BIT(5)));
	wr(PCM_CON0, SPM_KEY | BIT(2) | BIT(15));
	ret = wait(PCM_FSM_STA, 0x7fffff, RESET_STATE);
	if (ret)
		return ret;
	wr(PCM_CON0, SPM_KEY | BIT(2));
	wr(SPM_CPU_WAKEUP_EVENT, 0);
	wr(SPM_IRQ_STA, BIT(3));
	wr(SPM_SW_INT_CLEAR, GENMASK(9, 0));
	s.active = false;
	s.safe = true;
	return 0;
}
static void restore(void)
{
	if (!s.prepared || !s.safe)
		return;
	for (unsigned int i = 0; i < ARRAY_SIZE(pmic_regs); i++) {
		int ret = regmap_write(s.pmic, pmic_regs[i], s.pmic_saved[i]);

		if (ret) {
			s.last_error = ret;
			pr_err("mt6797-spm: PMIC recovery failed at %x\n",
			       pmic_regs[i]);
		}
	}
	for (unsigned int i = 0; i < ARRAY_SIZE(s.pwrap_saved); i++)
		writel(s.pwrap_saved[i], s.pwrap + 0x10c + i * 4);
	for (unsigned int i = 0; i < ARRAY_SIZE(offsets); i++)
		wr(offsets[i], s.regs[i]);
	for (unsigned int i = 0; i < ARRAY_SIZE(s.vectors); i++)
		wr(PCM_EVENT_VECTOR0 + i * 4, s.vector_saved[i]);
	wr(PCM_IM_PTR, s.ptr);
	wr(PCM_IM_LEN, s.len);
	wr(PCM_CON0, SPM_KEY | (s.con0 & 0xffff));
	wr(PCM_CON1, SPM_KEY | (s.con1 & 0xffff));
	s.prepared = false;
}
static int prepare(void)
{
	u32 selector, value;
	int ret;

	if ((rd(PCM_FSM_STA) & 0x7fffff) != RESET_STATE ||
	    (readl(s.phy + 0x63c) & 3) != 1)
		return -EBUSY;
	for (unsigned int i = 0; i < ARRAY_SIZE(pmic_regs); i++) {
		ret = regmap_read(s.pmic, pmic_regs[i], &s.pmic_saved[i]);
		if (ret)
			return ret;
	}
	for (unsigned int i = 0; i < ARRAY_SIZE(offsets); i++)
		s.regs[i] = rd(offsets[i]);
	for (unsigned int i = 0; i < ARRAY_SIZE(s.pwrap_saved); i++)
		s.pwrap_saved[i] = readl(s.pwrap + 0x10c + i * 4);
	for (unsigned int i = 0; i < ARRAY_SIZE(s.vectors); i++)
		s.vector_saved[i] = rd(PCM_EVENT_VECTOR0 + i * 4);
	s.ptr = rd(PCM_IM_PTR);
	s.len = rd(PCM_IM_LEN);
	s.con0 = rd(PCM_CON0);
	s.con1 = rd(PCM_CON1);
	s.prepared = true;
	s.safe = true;
	wr(POWERON_CONFIG_EN, SPM_KEY | 1);
	wr(SPM_POWER_ON_VAL1, 0x15820);
	wr(PCM_PWR_IO_EN, 0);
	wr(PCM_CON0, SPM_KEY | BIT(2) | BIT(15));
	ret = wait(PCM_FSM_STA, 0x7fffff, RESET_STATE);
	if (ret)
		goto failed;
	s.safe = false;
	wr(PCM_CON0, SPM_KEY | BIT(2) | BIT(3));
	wr(PCM_CON1,
	   SPM_KEY | BIT(12) | BIT(11) | BIT(10) | BIT(6) | BIT(3) | BIT(14));
	/* Keep the manual rail policy even if a firmware command executes.
	 * Indices follow BSP SUSPEND; VCORE and VSRAM selectors stay at baseline.
	 */
	selector = (s.pmic_saved[0] & BIT(1)) ? s.pmic_saved[2] :
						s.pmic_saved[1];
	for (unsigned int i = 0; i < 16; i++) {
		writel(0x060a, s.pwrap + 0x10c + i * 8);
		writel(selector, s.pwrap + 0x110 + i * 8);
	}
	for (unsigned int i = 7; i <= 8; i++) {
		writel(0x0204, s.pwrap + 0x10c + i * 8);
		writel(s.pmic_saved[3], s.pwrap + 0x110 + i * 8);
	}
	for (unsigned int i = 10; i <= 11; i++) {
		writel(0x06a8, s.pwrap + 0x10c + i * 8);
		writel(s.pmic_saved[5], s.pwrap + 0x110 + i * 8);
	}
	for (unsigned int i = 12; i <= 13; i++) {
		writel(0x045c, s.pwrap + 0x10c + i * 8);
		writel(s.pmic_saved[4], s.pwrap + 0x110 + i * 8);
	}
	wr(SPM_DVFS_CON, 0);
	ret = wait(SPM_DVFS_CON, BIT(31), BIT(31));
	if (ret)
		goto failed;
	wr(SPM_SW_RSV_5,
	   (rd(SPM_SW_RSV_5) & ~(3 | GENMASK(24, 23))) | 2 | (2 << 23));
	wr(SPM_PASR_DPD_1, s.dummy_address);
	wr(SPM_PASR_DPD_2, s.dummy_address);
	dma_wmb();
	wr(PCM_IM_PTR, s.pcm_address);
	wr(PCM_IM_LEN, s.words - 1);
	wr(PCM_CON0, SPM_KEY | BIT(2) | BIT(3) | BIT(1));
	wr(PCM_CON0, SPM_KEY | BIT(2) | BIT(3));
	wr(PCM_REG_DATA_INI, rd(SPM_POWER_ON_VAL0));
	wr(PCM_PWR_IO_EN, BIT(16));
	wr(PCM_PWR_IO_EN, 0);
	wr(PCM_REG_DATA_INI, rd(SPM_POWER_ON_VAL1));
	wr(PCM_PWR_IO_EN, BIT(23));
	wr(PCM_PWR_IO_EN, 0);
	for (unsigned int i = 0; i < ARRAY_SIZE(s.vectors); i++)
		wr(PCM_EVENT_VECTOR0 + i * 4, s.vectors[i]);
	/* BSP suspend_ctrl (no SCP), preserving the firmware's SRCCLKENI field. */
	wr(SPM_AP_STANDBY_CON, (rd(SPM_AP_STANDBY_CON) & BIT(23)) | BIT(26) |
				       BIT(19) | BIT(4) | BIT(0));
	wr(SPM_SRC_REQ, BIT(9) | BIT(6));
	wr(SPM_SRC_MASK,
	   BIT(28) | BIT(23) | BIT(21) | BIT(19) | BIT(17) | GENMASK(6, 1));
	wr(SPM_SRC2_MASK, GENMASK(23, 20) | BIT(16) | GENMASK(4, 0));
	for (unsigned int i = 0; i < 10; i++)
		wr(MP0_CPU0_WFI_EN + i * 4, 1);
	value = rd(SPM_CLK_CON) | BIT(2) | BIT(3) | BIT(4) | BIT(5) | BIT(6) |
		BIT(12) | BIT(16) | BIT(27);
	wr(SPM_CLK_CON, value);
	wr(0x010, 99);
	wr(PCM_TIMER_VAL, DIV_ROUND_UP((u64)s.cycle_wake_ms * 32768, 1000));
	wr(PCM_CON1, SPM_KEY | (rd(PCM_CON1) & ~BIT(5)) |
	   (s.cycle_wake_ms ? BIT(5) : 0));
	/* PCM timer, AP watchdog, keypad, EINT and USB; no modem dependency. */
	wr(SPM_WAKEUP_EVENT_MASK,
	   ~(u32)(BIT(0) | BIT(3) | BIT(2) | BIT(6) | BIT(14) | BIT(15)));
	wr(SPM_IRQ_STA, BIT(3));
	wr(SPM_IRQ_MASK, (rd(SPM_IRQ_MASK) & BIT(2)) | GENMASK(17, 9));
	wr(SPM_SW_INT_CLEAR, GENMASK(9, 0));
	wr(SPM_MAS_PAUSE_MASK_B, 0xffffffff);
	wr(SPM_MAS_PAUSE2_MASK_B, 0xffffffff);
	wr(SPM_SW_FLAG, SLEEP_FLAGS);
	wr(SPM_SW_RSV_0, 0);
	return 0;
failed:
	if (!s.safe)
		stop_pcm();
	restore();
	return ret;
}
static int memory_lease(void *ending, bool *quiescent)
{
	/* Retain the snapshot, not the DCM mutex, over CPU hotplug and PM. */
	*quiescent = ending && s.safe;
	return 0;
}

static int sleep_begin(suspend_state_t state)
{
	/* The suspend core falls back to platform ->begin() for s2idle when
	 * no platform_s2idle_ops are installed. No PCM/lease is needed there.
	 */
	if (state == PM_SUSPEND_TO_IDLE)
		return 0;
	if (state != PM_SUSPEND_MEM || !try_module_get(THIS_MODULE))
		return -ENODEV;
	s.module_held = true;
	s.cycle_wake_ms = READ_ONCE(s.wake_after_ms);
	/* Device/noirq failures may abort before our ->enter() is called. */
	s.last_error = -ECANCELED;
	s.cycles++;
	return 0;
}

static int sleep_prepare(void)
{
	int ret;

	ret = mt6797_vcorefs_claim_sleep(s.vcorefs, s.dev);
	if (ret)
		goto failed;
	s.owned = true;
	ret = mt6797_dcm_memory_transaction(s.dcm, memory_lease, NULL);
	if (ret)
		goto failed;
	s.dcm_lease = true;
	ret = prepare();
	if (!ret) {
		enable_irq(s.irq);
		s.irq_enabled = true;
	}
failed:
	if (ret)
		s.last_error = ret;
	return ret;
}

/* BSP describes 220 CIRQ inputs, but this silicon implements 219 (SPI72..290).
 * The final bit for the CIRQ aggregate IRQ291 reads zero in MASK/SENS/POL.
 * It retains edges while GIC
 * loses power; polarity comes from MT6797's secure SYSIRQ register banks.
 */
static u32 cirq_valid(unsigned int bank)
{
	return bank == 6 ? GENMASK(26, 0) : U32_MAX;
}

static void cirq_bank(unsigned int base, unsigned int bank, u32 value)
{
	u32 mask = cirq_valid(bank);

	writel(value & mask, s.cirq + base + 0x40 + bank * 4);
	writel(~value & mask, s.cirq + base + 0x80 + bank * 4);
}

static int cirq_prepare(void)
{
	u32 polarity[10], enabled[7] = {}, sense[7] = {}, positive[7] = {};
	u32 pending[7] = {};
	struct arm_smccc_res result;

	if (readl(s.cirq + 0x300) & BIT(0))
		return -EBUSY;
	/* Fetch every secure word before changing CIRQ. No partial clone on error. */
	for (unsigned int i = 2; i < 10; i++) {
		phys_addr_t address = i < 8 ? s.polarity[0] + i * 4 :
					     s.polarity[1] + (i - 8) * 4;

		arm_smccc_smc(MT6797_SIP_OCP_READ, address, 0, 0, 0, 0, 0, 0, &result);
		if ((long)result.a0 < 0)
			return -EIO;
		polarity[i] = result.a0;
	}
	for (unsigned int i = 0; i < 219; i++) {
		unsigned int spi = i + 72, id = spi + 32, bank = i / 32;
		u32 bit = BIT(i % 32);

		if (readl(s.gic + 0x100 + id / 32 * 4) & BIT(id % 32))
			enabled[bank] |= bit;
		if (readl(s.gic + 0x200 + id / 32 * 4) & BIT(id % 32))
			pending[bank] |= bit;
		if (!(readl(s.gic + 0xc00 + id / 16 * 4) & BIT(id % 16 * 2 + 1)))
			sense[bank] |= bit;
		if (!(polarity[spi / 32] & BIT(spi % 32)))
			positive[bank] |= bit;
	}
	s.cirq_control = readl(s.cirq + 0x300);
	for (unsigned int i = 0; i < 7; i++) {
		s.cirq_saved[0][i] = readl(s.cirq + 0x080 + i * 4);
		s.cirq_saved[1][i] = readl(s.cirq + 0x140 + i * 4);
		s.cirq_saved[2][i] = readl(s.cirq + 0x200 + i * 4);
		cirq_bank(0x200, i, positive[i]);
		cirq_bank(0x140, i, sense[i]);
		cirq_bank(0x080, i, ~enabled[i]);
		/* Keep edges already pending and unmasked at GIC. This closes the
		 * ACK race noted by the mainline CIRQ syscore suspend implementation.
		 */
		writel(cirq_valid(i) & ~(pending[i] & enabled[i]),
		       s.cirq + 0x040 + i * 4);
	}
	for (unsigned int i = 0; i < 7; i++) {
		u32 mask = cirq_valid(i);

		if (((readl(s.cirq + 0x080 + i * 4) ^ ~enabled[i]) & mask) ||
		    ((readl(s.cirq + 0x140 + i * 4) ^ sense[i]) & mask) ||
		    ((readl(s.cirq + 0x200 + i * 4) ^ positive[i]) & mask)) {
			dev_err(s.dev, "CIRQ bank%u mask=%08x/%08x sens=%08x/%08x pol=%08x/%08x\n",
				i, readl(s.cirq + 0x080 + i * 4), ~enabled[i],
				readl(s.cirq + 0x140 + i * 4), sense[i],
				readl(s.cirq + 0x200 + i * 4), positive[i]);
			for (unsigned int j = 0; j < 7; j++) {
				cirq_bank(0x200, j, s.cirq_saved[2][j]);
				cirq_bank(0x140, j, s.cirq_saved[1][j]);
				cirq_bank(0x080, j, s.cirq_saved[0][j]);
			}
			return -EIO;
		}
	}
	writel((s.cirq_control & 7) | BIT(0) | BIT(1), s.cirq + 0x300);
	s.cirq_enabled = true;
	s.cirq_cycles++;
	return 0;
}

static void cirq_finish(void)
{
	if (!s.cirq_enabled)
		return;
	/* BSP flush: replay latched lines into GIC, then mask/ACK/disable. */
	for (unsigned int i = 0; i < 7; i++)
		writel(cirq_valid(i), s.cirq + 0x100 + i * 4);
	for (unsigned int i = 0; i < 7; i++)
		s.cirq_pending[i] = readl(s.cirq + i * 4) & cirq_valid(i);
	for (unsigned int i = 0; i < 219; i++) {
		unsigned int id = i + 72 + 32;

		if (s.cirq_pending[i / 32] & BIT(i % 32)) {
			writel(BIT(id % 32), s.gic + 0x200 + id / 32 * 4);
			s.cirq_flushed++;
		}
	}
	for (unsigned int i = 0; i < 7; i++) {
		writel(cirq_valid(i), s.cirq + 0x0c0 + i * 4);
		writel(cirq_valid(i), s.cirq + 0x040 + i * 4);
	}
	writel(s.cirq_control & 7, s.cirq + 0x300);
	for (unsigned int i = 0; i < 7; i++) {
		cirq_bank(0x200, i, s.cirq_saved[2][i]);
		cirq_bank(0x140, i, s.cirq_saved[1][i]);
		cirq_bank(0x080, i, s.cirq_saved[0][i]);
	}
	s.cirq_enabled = false;
}

static int sleep_enter(suspend_state_t state)
{
	int ret, stopped;

	if (state != PM_SUSPEND_MEM || !s.prepared ||
	    num_online_cpus() != 1 || smp_processor_id() != 0) {
		s.last_error = -EINVAL;
		return -EINVAL;
	}
	ret = cirq_prepare();
	if (ret) {
		s.last_error = ret;
		return ret;
	}
	ret = pcm_wdt_arm();
	if (!ret)
		ret = spm_wdt_request(true);
	if (!ret) {
		dev_info(s.dev, "PCM WDT armed: timer=%u compare=%u timeout=30s\n",
			 rd(PCM_TIMER_VAL), s.wdt_value);
		wr(PCM_REG_DATA_INI, 0);
		wr(PCM_PWR_IO_EN, BIT(0) | BIT(7));
		s.active = true;
		s.safe = false;
		wr(PCM_CON0, SPM_KEY | BIT(2) | BIT(3) | BIT(0));
		wr(PCM_CON0, SPM_KEY | BIT(2) | BIT(3));
		ret = cpu_pm_enter();
		if (!ret) {
			ret = psci_cpu_suspend_enter(MT6797_SLEEP_STATE);
			cpu_pm_exit();
		}
	}
	/* A zero return from cpu_suspend requires the warm cpu_resume path. */
	if (!ret)
		s.resumes++;
	s.r12 = rd(SPM_SW_RSV_0);
	s.raw = rd(0x15c);
	s.debug = rd(0x604);
	s.timer = rd(0x150);
	s.r13 = rd(0x134);
	s.flags = rd(SPM_SW_FLAG);
	stopped = pcm_wdt_disarm();
	if (!ret)
		ret = stopped;
	stopped = spm_wdt_request(false);
	if (!ret)
		ret = stopped;
	cirq_finish();
	stopped = stop_pcm();
	if (!ret)
		ret = stopped;
	s.last_error = ret;
	dev_dbg(s.dev, "resume=%d wake=%08x raw=%08x debug=%08x ticks=%u flags=%08x\n",
		 ret, s.r12, s.raw, s.debug, s.timer, s.flags);
	return ret;
}

static void sleep_finish(void)
{
	int ret;

	if (s.irq_enabled) {
		disable_irq(s.irq);
		s.irq_enabled = false;
	}
	ret = pcm_wdt_disarm();
	if (ret)
		s.last_error = ret;
	ret = spm_wdt_request(false);
	if (ret)
		s.last_error = ret;
	if (s.prepared && !s.safe) {
		ret = stop_pcm();
		if (ret)
			s.last_error = ret;
	}
	if (!s.wdt_armed && !s.wdt_request_owned)
		restore();
	if (s.dcm_lease) {
		ret = mt6797_dcm_memory_transaction(s.dcm, memory_lease, &s);
		if (ret)
			s.last_error = ret;
		if (s.safe && !ret && !s.wdt_armed && !s.wdt_request_owned)
			s.dcm_lease = false;
	}
	if (s.owned && s.safe && !s.dcm_lease &&
	    !s.wdt_armed && !s.wdt_request_owned) {
		ret = mt6797_vcorefs_release_sleep(s.vcorefs, s.dev);
		if (ret)
			s.last_error = ret;
		else
			s.owned = false;
	}
	/* Incomplete PCM or TOPRGU cleanup retains all exclusive resources. */
	if ((s.owned || s.wdt_armed || s.wdt_request_owned) && !s.pinned) {
		__module_get(THIS_MODULE);
		s.pinned = true;
		dev_err(s.dev, "PCM/TOPRGU not released: %d; resources retained\n",
			s.last_error);
	}
}

static void sleep_end(void)
{
	if (s.module_held) {
		if (s.last_error)
			s.failures++;
		s.module_held = false;
		module_put(THIS_MODULE);
	}
}

static const struct platform_suspend_ops sleep_ops = {
	.valid = suspend_valid_only_mem,
	.begin = sleep_begin,
	.prepare_late = sleep_prepare,
	.enter = sleep_enter,
	.finish = sleep_finish,
	.end = sleep_end,
};

static irqreturn_t sleep_interrupt(int irq, void *data)
{
	if (!READ_ONCE(s.owned) || !(rd(SPM_IRQ_STA) & BIT(3)))
		return IRQ_NONE;
	wr(SPM_IRQ_STA, BIT(3));
	pm_wakeup_hard_event(s.dev);
	return IRQ_HANDLED;
}

static ssize_t wake_after_ms_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%u\n", READ_ONCE(s.wake_after_ms));
}

static ssize_t wake_after_ms_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	u32 value;
	int ret = kstrtou32(buf, 0, &value);

	if (ret)
		return ret;
	if (value && (value < 100 || value > 600000))
		return -ERANGE;
	WRITE_ONCE(s.wake_after_ms, value);
	return count;
}
static DEVICE_ATTR_RW(wake_after_ms);

static ssize_t state_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	return sysfs_emit(buf,
		"cycles=%lu resumes=%lu failures=%lu last_error=%d owned=%u safe=%u dcm_lease=%u\n"
		"wake=%08x raw=%08x debug=%08x timer_ticks=%u r13=%08x flags=%08x\n"
		"pcm_wdt=armed:%u request_owned:%u compare:%08x ticks:%u con1:%08x\n"
		"cpu_powerdown=1 infra_powerdown=requested infra_off_verified=0 manual_cpu_rails=retained\n"
		"cirq_cycles=%lu cirq_flushed=%lu cirq_enabled=%u pending=%08x,%08x,%08x,%08x,%08x,%08x,%08x\n",
		s.cycles, s.resumes, s.failures, s.last_error, s.owned, s.safe,
		s.dcm_lease, s.r12, s.raw, s.debug, s.timer, s.r13, s.flags,
		s.wdt_armed, s.wdt_request_owned, s.wdt_value, s.wdt_ticks, s.wdt_con1,
		s.cirq_cycles, s.cirq_flushed, s.cirq_enabled,
		s.cirq_pending[0], s.cirq_pending[1], s.cirq_pending[2],
		s.cirq_pending[3], s.cirq_pending[4], s.cirq_pending[5],
		s.cirq_pending[6]);
}
static DEVICE_ATTR_RO(state);
static struct attribute *sleep_attrs[] = {
	&dev_attr_wake_after_ms.attr, &dev_attr_state.attr, NULL,
};
ATTRIBUTE_GROUPS(sleep);

static int sleep_firmware(void)
{
	const struct firmware *fw;
	size_t offset;
	int ret;

	ret = request_firmware(&fw, "mediatek/mt6797/pcm_suspend.bin", s.dev);
	if (ret)
		return ret;
	ret = -EINVAL;
	if (fw->size < 75)
		goto release;
	s.words = get_unaligned_le16(fw->data);
	offset = 2 + s.words * 4;
	if (s.words != 1704 ||
	    offset + 72 + sizeof("pcm_suspend_fpb_v15_fix_pmicw_cmd") != fw->size ||
	    get_unaligned_le16(fw->data + offset) != s.words ||
	    fw->data[offset + 2] != 2 || fw->data[offset + 3] != 0 ||
	    get_unaligned_le32(fw->data + offset + 4) ||
	    strcmp((const char *)fw->data + offset + 72,
		   "pcm_suspend_fpb_v15_fix_pmicw_cmd"))
		goto release;
	s.code = dmam_alloc_coherent(s.dev, FW_LIMIT, &s.dma, GFP_KERNEL);
	if (!s.code) {
		ret = -ENOMEM;
		goto release;
	}
	memcpy(s.code, fw->data + 2, s.words * 4);
	for (unsigned int i = 0; i < ARRAY_SIZE(s.vectors); i++)
		s.vectors[i] = get_unaligned_le32(fw->data + offset + 8 + i * 4);
	ret = 0;
release:
	release_firmware(fw);
	return ret;
}

static void sleep_unmap(void *base)
{
	iounmap(base);
}

static void sleep_put_device(void *dev)
{
	put_device(dev);
}

static void sleep_disable_wake(void *dev)
{
	disable_irq_wake(s.irq);
	device_init_wakeup(dev, false);
}

static int sleep_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mt6797_spm_resources resources;
	struct platform_device *provider;
	struct device_node *np;
	struct arm_smccc_res result;
	struct resource res;
	struct clk *cirq_clock;
	int ret;

	if (s.dev)
		return -EBUSY;
	arm_smccc_smc(MT6797_SIP_SLEEP_CAPS, 0, 0, 0, 0, 0, 0, 0, &result);
	if (result.a0 != MT6797_SLEEP_STATE || result.a1 != 1)
		return dev_err_probe(dev, -EOPNOTSUPP, "TF-A sleep ABI v1 required\n");
	np = of_parse_phandle(dev->of_node, "mediatek,vcorefs", 0);
	provider = np ? of_find_device_by_node(np) : NULL;
	of_node_put(np);
	if (!provider)
		return -EPROBE_DEFER;
	ret = devm_add_action_or_reset(dev, sleep_put_device, &provider->dev);
	if (ret)
		return ret;
	if (!device_is_bound(&provider->dev))
		return -EPROBE_DEFER;
	if (!device_link_add(dev, &provider->dev, DL_FLAG_AUTOREMOVE_CONSUMER))
		return -ENOMEM;
	ret = mt6797_vcorefs_sleep_resources(&provider->dev, &resources);
	if (ret)
		return ret;
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;
	memset(&s, 0, sizeof(s));
	s.dev = dev;
	s.vcorefs = &provider->dev;
	s.dcm = resources.dcm;
	s.pmic = resources.pmic;
	s.spm = resources.spm;
	s.pwrap = resources.pwrap;
	s.phy = resources.phy;
	s.dummy_address = resources.dummy_address;
	s.safe = true;
	np = of_parse_phandle(dev->of_node, "mediatek,watchdog", 0);
	provider = np ? of_find_device_by_node(np) : NULL;
	of_node_put(np);
	if (!provider) {
		ret = -EPROBE_DEFER;
		goto failed;
	}
	ret = devm_add_action_or_reset(dev, sleep_put_device, &provider->dev);
	if (ret)
		goto failed;
	if (!device_is_bound(&provider->dev)) {
		ret = -EPROBE_DEFER;
		goto failed;
	}
	s.watchdog = &provider->dev;
	if (!device_link_add(dev, s.watchdog, DL_FLAG_AUTOREMOVE_CONSUMER)) {
		ret = -ENOMEM;
		goto failed;
	}
	cirq_clock = devm_clk_get_enabled(dev, "cirq");
	if (IS_ERR(cirq_clock)) {
		ret = PTR_ERR(cirq_clock);
		goto failed;
	}
	s.cirq = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(s.cirq)) {
		ret = PTR_ERR(s.cirq);
		goto failed;
	}
	np = of_parse_phandle(dev->of_node, "mediatek,gic", 0);
	s.gic = np ? of_iomap(np, 0) : NULL;
	of_node_put(np);
	if (!s.gic) {
		ret = -ENODEV;
		goto failed;
	}
	ret = devm_add_action_or_reset(dev, sleep_unmap, s.gic);
	if (ret)
		goto failed;
	np = of_parse_phandle(dev->of_node, "mediatek,sysirq", 0);
	for (unsigned int i = 0; i < 2; i++) {
		ret = np ? of_address_to_resource(np, i, &res) : -EINVAL;
		if (ret)
			break;
		s.polarity[i] = res.start;
	}
	of_node_put(np);
	if (ret)
		goto failed;
	ret = sleep_firmware();
	if (ret)
		goto failed;
	s.pcm_address = lower_32_bits(s.dma);
	if (resources.mapping_4g && s.pcm_address >= 0x40000000 &&
	    s.pcm_address < 0xc0000000)
		s.pcm_address += 0x40000000;
	s.irq = platform_get_irq(pdev, 0);
	if (s.irq < 0) {
		ret = s.irq;
		goto failed;
	}
	ret = devm_request_irq(dev, s.irq, sleep_interrupt,
			       IRQF_NO_SUSPEND | IRQF_NO_AUTOEN,
			       dev_name(dev), NULL);
	if (ret)
		goto failed;
	device_init_wakeup(dev, true);
	ret = enable_irq_wake(s.irq);
	if (ret) {
		device_init_wakeup(dev, false);
		goto failed;
	}
	ret = devm_add_action_or_reset(dev, sleep_disable_wake, dev);
	if (ret)
		goto failed;
	suspend_set_ops(&sleep_ops);
	dev_info(dev, "BSP sleep PCM ready; manual CPU rails retained\n");
	return 0;
failed:
	s.dev = NULL;
	return ret;
}

static void sleep_remove(struct platform_device *pdev)
{
	suspend_set_ops(NULL);
	/* Device links prevent provider removal while this driver is bound. */
	s.dev = NULL;
}

static const struct of_device_id sleep_of_match[] = {
	{ .compatible = "mediatek,mt6797-spm-suspend" }, {},
};
MODULE_DEVICE_TABLE(of, sleep_of_match);

static struct platform_driver sleep_driver = {
	.probe = sleep_probe,
	.remove = sleep_remove,
	.driver = {
		.name = "mt6797-spm-suspend",
		.of_match_table = sleep_of_match,
		.dev_groups = sleep_groups,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(sleep_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MT6797 BSP SPM CPU powerdown system sleep");
MODULE_FIRMWARE("mediatek/mt6797/pcm_suspend.bin");
