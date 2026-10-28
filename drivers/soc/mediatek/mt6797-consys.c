// SPDX-License-Identifier: GPL-2.0-only
//
// MT6797 ConSys power/bring-up driver for the X20-96 board.
//
// The MT6797 connectivity subsystem (ConSys) hosts the WLAN/BT/GPS baseband
// and drives the external MT6631 radio through dedicated CONN_IF pins.  This
// driver implements the vendor power-on sequence from the MT6797 BSP
// (conn_soc/mt6797/mtk_wcn_consys_hw.c): MT6351 connectivity LDOs, MTCMOS
// power-up through SPM, AXI bus-protect release, and the ConSys MCU chip-ID
// handshake.  It exposes the resulting state through sysfs for bring-up and
// testing; protocol stacks (WMT/STP, WLAN) are out of scope here.
//
//       radio_power - 1 to run the full power-on sequence, 0 to power off
//       chipid - read of CONN_MCU_CFG + 0x08 (0x0279 when the MCU is alive)

#include <linux/soc/mediatek/mt6797-sip.h>
#include <linux/bitops.h>
#include <linux/arm-smccc.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kstrtox.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>

#include <linux/mt6797-consys.h>
#include <linux/mt6797-stp-session.h>
#include <linux/soc/mediatek/mt6797-toprgu.h>

#define CONSYS_CHIP_ID			0x0279

/* CONN_MCU_CONFIG_BASE */
#define CONSYS_MCU_CHIP_ID_OFF		0x0008
#define CONSYS_MCU_CFG_ACR_OFF		0x0110
#define CONSYS_MCU_CFG_ACR_MBIST	BIT(18)
/* AP_RGU_BASE */
#define CONSYS_RGU_RST_ID		12
/* TOPCKGEN_BASE */
#define CONSYS_TOPAXI_PROT_EN_OFF	0x1220
#define CONSYS_TOPAXI_PROT_STA1_OFF	0x1228
#define CONSYS_TOPAXI_PROT_MASK		(BIT(18) | BIT(19))
#define CONSYS_CONN2AP_SLEEP_MASK_OFF	0x1350
#define CONSYS_CONN2AP_SLEEP_MASK_BIT	BIT(8)
#define CONSYS_EMI_MAPPING_OFF		0x1340
#define CONSYS_EMI_MAPPING_ENABLE	0x1000
/* SPM_BASE */
#define CONSYS_SPM_PWRON_CONFG_EN_OFF	0x0000
#define CONSYS_SPM_PWRON_CONFG_EN_VAL	0x0b160001
#define CONSYS_SPM_CONN_PWR_CTRL_OFF	0x032c
#define CONSYS_SPM_PWR_RST_BIT		BIT(0)
#define CONSYS_SPM_PWR_ISO_S_BIT	BIT(1)
#define CONSYS_SPM_PWR_ON_BIT		BIT(2)
#define CONSYS_SPM_PWR_ON_S_BIT		BIT(3)
#define CONSYS_SPM_CLK_DIS_BIT		BIT(4)
#define CONSYS_SPM_PWR_ACK_OFF		0x0180
#define CONSYS_SPM_PWR_ACK_S_OFF	0x0184
#define CONSYS_SPM_PWR_ACK_BIT		BIT(1)

/*
 * These bits describe an attempted stage, rather than only a successful
 * stage.  That distinction matters when a register write succeeds but its
 * following ACK poll times out: cleanup must still undo the write.
 */
#define CONSYS_STAGE_VCN18		BIT(0)
#define CONSYS_STAGE_VCN28		BIT(1)
#define CONSYS_STAGE_SLEEP_MASK		BIT(2)
#define CONSYS_STAGE_RGU_RESET		BIT(3)
#define CONSYS_STAGE_PWR_CFG		BIT(4)
#define CONSYS_STAGE_PWR_ON		BIT(5)
#define CONSYS_STAGE_PWR_ON_S		BIT(6)
#define CONSYS_STAGE_CLK_ON		BIT(7)
#define CONSYS_STAGE_ISO_RELEASED	BIT(8)
#define CONSYS_STAGE_SPM_RESET		BIT(9)
#define CONSYS_STAGE_AXI		BIT(10)
#define CONSYS_STAGE_CHIP_ID		BIT(11)
#define CONSYS_STAGE_RGU_RELEASED	BIT(12)

#define CONSYS_STAGE_POWERED		(CONSYS_STAGE_CHIP_ID | \
						 CONSYS_STAGE_RGU_RELEASED)

#define MT6797_EMI_REGION_SHIFT		27U
#define MT6797_EMI_REGION18		18U
#define MT6797_EMI_REGION19		19U
#define MT6797_EMI_REGION18_OPEN	0x00000000U
#define MT6797_EMI_REGION18_RESTORE	0x00b6da2dU
#define MT6797_EMI_REGION19_HOST	0x00b6da28U
#define MT6797_EMI_WINDOW_SIZE		0x00080000U
#define MT6797_EMI_MAPPED_SIZE		0x00100000U

struct mt6797_consys {
	struct device *dev;
	void __iomem *base[4];
	void __iomem *mcu;
	void __iomem *rgu;
	void __iomem *topckgen;
	void __iomem *spm;
	struct regulator *vcn18;
	struct regulator *vcn28;
	struct regulator *vcn33_bt;
	struct regulator *vcn33_wifi;
	struct mutex lock;
	u32 stages;
	u32 stp_mode;
	int last_error;
	bool powered;
	bool stp_claimed;
	bool stp_power_owned;
	bool stp_enabled;
	bool bt_radio_on;
	bool wifi_radio_on;
	phys_addr_t emi_base;
	phys_addr_t emi_size;
	void __iomem *emi_io;
	u32 emi_mapping_saved;
	bool emi_mapping_active;
	bool emi_patch_active;
};

static int consys_emi_prepare_locked(struct mt6797_consys *cs);
static int consys_emi_restore_locked(struct mt6797_consys *cs);

static int consys_wait_ack(struct mt6797_consys *cs, u32 off, u32 mask)
{
	u32 val;
	int ret;

	ret = readl_poll_timeout(cs->spm + off, val, val & mask, 10, 100000);
	if (ret)
		dev_err(cs->dev, "timeout waiting SPM ack @0x%x (val=0x%08x)\n",
			off, val);
	return ret;
}

static int consys_wait_ack_clear(struct mt6797_consys *cs, u32 off, u32 mask)
{
	u32 val;
	int ret;

	ret = readl_poll_timeout(cs->spm + off, val, !(val & mask), 10, 100000);
	if (ret)
		dev_err(cs->dev, "timeout clearing SPM ack @0x%x (val=0x%08x)\n",
			off, val);
	return ret;
}

static int consys_rgu_sw_rst(struct mt6797_consys *cs, bool assert)
{
	int ret = mt6797_toprgu_update(CONSYS_RGU_RST_ID, assert);

	if (ret)
		dev_err(cs->dev, "TOPRGU MCU reset %s failed: %d\n",
			assert ? "assert" : "deassert", ret);
	return ret;
}

static int consys_axi_protect(struct mt6797_consys *cs, bool enable)
{
	u32 val;

	val = readl(cs->topckgen + CONSYS_TOPAXI_PROT_EN_OFF);
	if (enable)
		val |= CONSYS_TOPAXI_PROT_MASK;
	else
		val &= ~CONSYS_TOPAXI_PROT_MASK;
	writel(val, cs->topckgen + CONSYS_TOPAXI_PROT_EN_OFF);

	if (enable)
		return readl_poll_timeout(cs->topckgen + CONSYS_TOPAXI_PROT_STA1_OFF,
					  val, (val & CONSYS_TOPAXI_PROT_MASK) ==
					       CONSYS_TOPAXI_PROT_MASK,
					  10, 100000);

	return readl_poll_timeout(cs->topckgen + CONSYS_TOPAXI_PROT_STA1_OFF,
				  val, !(val & CONSYS_TOPAXI_PROT_MASK),
				  10, 100000);
}

/* Called with cs->lock held. */
static int consys_cleanup_locked(struct mt6797_consys *cs)
{
	int ret = 0;
	int tmp;
	u32 val;

	/* WMT function rails are children of the ConSys power owner. */
	if (cs->wifi_radio_on) {
		tmp = regulator_disable(cs->vcn33_wifi);
		if (tmp && !ret)
			ret = tmp;
		cs->wifi_radio_on = false;
	}
	if (cs->bt_radio_on) {
		tmp = regulator_disable(cs->vcn33_bt);
		if (tmp && !ret)
			ret = tmp;
		cs->bt_radio_on = false;
	}

	if (cs->stages & CONSYS_STAGE_RGU_RELEASED) {
		tmp = consys_rgu_sw_rst(cs, true);
		if (tmp)
			return tmp;
		cs->stages &= ~CONSYS_STAGE_RGU_RELEASED;
	}

	if (cs->stages & CONSYS_STAGE_AXI) {
		tmp = consys_axi_protect(cs, true);
		if (tmp && !ret) {
			ret = tmp;
			dev_err(cs->dev, "failed to re-enable AXI protection: %d\n", tmp);
		}
		cs->stages &= ~CONSYS_STAGE_AXI;
	}
	/* The MCU must be reset before its DRAM mapping can be removed. */
	tmp = consys_emi_restore_locked(cs);
	if (tmp && !ret)
		ret = tmp;

	if (cs->stages & CONSYS_STAGE_SPM_RESET) {
		val = readl(cs->spm + CONSYS_SPM_CONN_PWR_CTRL_OFF);
		writel(val & ~CONSYS_SPM_PWR_RST_BIT,
		       cs->spm + CONSYS_SPM_CONN_PWR_CTRL_OFF);
		cs->stages &= ~CONSYS_STAGE_SPM_RESET;
	}

	if (cs->stages & CONSYS_STAGE_ISO_RELEASED) {
		val = readl(cs->spm + CONSYS_SPM_CONN_PWR_CTRL_OFF);
		writel(val | CONSYS_SPM_PWR_ISO_S_BIT,
		       cs->spm + CONSYS_SPM_CONN_PWR_CTRL_OFF);
		cs->stages &= ~CONSYS_STAGE_ISO_RELEASED;
	}

	if (cs->stages & CONSYS_STAGE_CLK_ON) {
		val = readl(cs->spm + CONSYS_SPM_CONN_PWR_CTRL_OFF);
		writel(val | CONSYS_SPM_CLK_DIS_BIT,
		       cs->spm + CONSYS_SPM_CONN_PWR_CTRL_OFF);
		udelay(1);
		cs->stages &= ~CONSYS_STAGE_CLK_ON;
	}

	if (cs->stages & (CONSYS_STAGE_PWR_ON | CONSYS_STAGE_PWR_ON_S)) {
		val = readl(cs->spm + CONSYS_SPM_CONN_PWR_CTRL_OFF);
		writel(val & ~(CONSYS_SPM_PWR_ON_BIT | CONSYS_SPM_PWR_ON_S_BIT),
		       cs->spm + CONSYS_SPM_CONN_PWR_CTRL_OFF);
		tmp = consys_wait_ack_clear(cs, CONSYS_SPM_PWR_ACK_OFF,
					     CONSYS_SPM_PWR_ACK_BIT);
		if (tmp && !ret)
			ret = tmp;
		tmp = consys_wait_ack_clear(cs, CONSYS_SPM_PWR_ACK_S_OFF,
					     CONSYS_SPM_PWR_ACK_BIT);
		if (tmp && !ret)
			ret = tmp;
		cs->stages &= ~(CONSYS_STAGE_PWR_ON | CONSYS_STAGE_PWR_ON_S);
	}

	if (cs->stages & CONSYS_STAGE_PWR_CFG) {
		writel(CONSYS_SPM_PWRON_CONFG_EN_VAL & ~0x1,
		       cs->spm + CONSYS_SPM_PWRON_CONFG_EN_OFF);
		cs->stages &= ~CONSYS_STAGE_PWR_CFG;
	}

	/* The chip ID is an observation, not a power-off action. */
	cs->stages &= ~CONSYS_STAGE_CHIP_ID;

	if (cs->stages & CONSYS_STAGE_RGU_RESET)
		cs->stages &= ~CONSYS_STAGE_RGU_RESET;

	if (cs->stages & CONSYS_STAGE_SLEEP_MASK) {
		val = readl(cs->topckgen + CONSYS_CONN2AP_SLEEP_MASK_OFF);
		writel(val & ~CONSYS_CONN2AP_SLEEP_MASK_BIT,
		       cs->topckgen + CONSYS_CONN2AP_SLEEP_MASK_OFF);
		cs->stages &= ~CONSYS_STAGE_SLEEP_MASK;
	}

	if (cs->stages & CONSYS_STAGE_VCN28) {
		tmp = regulator_disable(cs->vcn28);
		if (tmp && !ret)
			ret = tmp;
		cs->stages &= ~CONSYS_STAGE_VCN28;
	}
	if (cs->stages & CONSYS_STAGE_VCN18) {
		tmp = regulator_disable(cs->vcn18);
		if (tmp && !ret)
			ret = tmp;
		cs->stages &= ~CONSYS_STAGE_VCN18;
	}

	cs->powered = false;
	cs->stp_enabled = false;
	cs->stp_mode = 0;
	return ret;
}

/* Called with cs->lock held. */
static int consys_power_on_locked(struct mt6797_consys *cs)
{
	u32 val, chipid;
	int ret, retry;

	if (cs->powered)
		return 0;
	if (cs->stages || cs->emi_mapping_active || cs->emi_patch_active) {
		dev_warn(cs->dev, "recovering incomplete state before power-on (0x%x)\n",
			 cs->stages);
		ret = consys_cleanup_locked(cs);
		if (ret)
			return ret;
	}

	/* Step 1: connectivity LDOs (vendor order: VCN18, settle, VCN28). */
	ret = regulator_enable(cs->vcn18);
	if (ret) {
		dev_err(cs->dev, "failed to enable vcn18: %d\n", ret);
		return ret;
	}
	cs->stages |= CONSYS_STAGE_VCN18;
	udelay(240);
	ret = regulator_enable(cs->vcn28);
	if (ret) {
		dev_err(cs->dev, "failed to enable vcn28: %d\n", ret);
		goto err_cleanup;
	}
	cs->stages |= CONSYS_STAGE_VCN28;

	/* Mask ConSys sleep requests while the MCU boots. */
	val = readl(cs->topckgen + CONSYS_CONN2AP_SLEEP_MASK_OFF);
	writel(val | CONSYS_CONN2AP_SLEEP_MASK_BIT,
	       cs->topckgen + CONSYS_CONN2AP_SLEEP_MASK_OFF);
	cs->stages |= CONSYS_STAGE_SLEEP_MASK;

	/* Step 2: hold the ConSys MCU in software reset during power-up. */
	ret = consys_rgu_sw_rst(cs, true);
	if (ret)
		goto err_cleanup;
	cs->stages |= CONSYS_STAGE_RGU_RESET;

	/* Step 3: unmask SPM power-on control for the CONN domain. */
	writel(CONSYS_SPM_PWRON_CONFG_EN_VAL,
	       cs->spm + CONSYS_SPM_PWRON_CONFG_EN_OFF);
	cs->stages |= CONSYS_STAGE_PWR_CFG;

	/* Step 4: MTCMOS power-up with ISO release and clock enable. */
	val = readl(cs->spm + CONSYS_SPM_CONN_PWR_CTRL_OFF);
	writel(val | CONSYS_SPM_PWR_ON_BIT, cs->spm + CONSYS_SPM_CONN_PWR_CTRL_OFF);
	cs->stages |= CONSYS_STAGE_PWR_ON;
	ret = consys_wait_ack(cs, CONSYS_SPM_PWR_ACK_OFF, CONSYS_SPM_PWR_ACK_BIT);
	if (ret)
		goto err_cleanup;

	val = readl(cs->spm + CONSYS_SPM_CONN_PWR_CTRL_OFF);
	writel(val | CONSYS_SPM_PWR_ON_S_BIT, cs->spm + CONSYS_SPM_CONN_PWR_CTRL_OFF);
	cs->stages |= CONSYS_STAGE_PWR_ON_S;
	ret = consys_wait_ack(cs, CONSYS_SPM_PWR_ACK_S_OFF, CONSYS_SPM_PWR_ACK_BIT);
	if (ret)
		goto err_cleanup;

	val = readl(cs->spm + CONSYS_SPM_CONN_PWR_CTRL_OFF);
	writel(val & ~CONSYS_SPM_CLK_DIS_BIT, cs->spm + CONSYS_SPM_CONN_PWR_CTRL_OFF);
	cs->stages |= CONSYS_STAGE_CLK_ON;
	udelay(1);

	val = readl(cs->spm + CONSYS_SPM_CONN_PWR_CTRL_OFF);
	writel(val & ~CONSYS_SPM_PWR_ISO_S_BIT, cs->spm + CONSYS_SPM_CONN_PWR_CTRL_OFF);
	cs->stages |= CONSYS_STAGE_ISO_RELEASED;

	val = readl(cs->spm + CONSYS_SPM_CONN_PWR_CTRL_OFF);
	writel(val | CONSYS_SPM_PWR_RST_BIT, cs->spm + CONSYS_SPM_CONN_PWR_CTRL_OFF);
	cs->stages |= CONSYS_STAGE_SPM_RESET;

	/* Step 5: release the ConSys AXI bus protection. */
	cs->stages |= CONSYS_STAGE_AXI;
	ret = consys_axi_protect(cs, false);
	if (ret) {
		dev_err(cs->dev, "AXI bus protect stuck (sta1=0x%08x)\n",
			readl(cs->topckgen + CONSYS_TOPAXI_PROT_STA1_OFF));
		goto err_cleanup;
	}

	/* 26 MHz is running by now; allow memory power-down de-assert. */
	udelay(30);

	/* Step 6: poll the ConSys MCU chip ID (vendor handshake). */
	chipid = 0;
	for (retry = 10; retry > 0; retry--) {
		chipid = readl(cs->mcu + CONSYS_MCU_CHIP_ID_OFF);
		if (chipid == CONSYS_CHIP_ID)
			break;
		msleep(20);
	}
	if (chipid != CONSYS_CHIP_ID) {
		dev_err(cs->dev, "ConSys chip ID 0x%08x != 0x%04x, power-on issue\n",
			chipid, CONSYS_CHIP_ID);
		ret = -EIO;
		goto err_cleanup;
	}
	dev_info(cs->dev, "ConSys chip ID 0x%04x confirmed\n", chipid);
	cs->stages |= CONSYS_STAGE_CHIP_ID;

	/* Reserve and clear shared DRAM while the ConSys MCU is still reset. */
	ret = consys_emi_prepare_locked(cs);
	if (ret) {
		dev_err(cs->dev, "ConSys EMI setup failed (matching BL31 required): %d\n", ret);
		goto err_cleanup;
	}

	/* Step 7: run MBIST at real speed, then release the MCU reset. */
	val = readl(cs->mcu + CONSYS_MCU_CFG_ACR_OFF);
	writel(val | CONSYS_MCU_CFG_ACR_MBIST, cs->mcu + CONSYS_MCU_CFG_ACR_OFF);

	ret = consys_rgu_sw_rst(cs, false);
	if (ret)
		goto err_cleanup;
	cs->stages &= ~CONSYS_STAGE_RGU_RESET;
	cs->stages |= CONSYS_STAGE_RGU_RELEASED;
	msleep(20);

	cs->powered = (cs->stages & CONSYS_STAGE_POWERED) == CONSYS_STAGE_POWERED;
	dev_dbg(cs->dev, "ConSys powered on\n");
	return 0;

err_cleanup:
	cs->last_error = ret;
	if (consys_cleanup_locked(cs))
		dev_err(cs->dev, "cleanup after power-on failure also failed\n");
	return ret;
}

/* Called with cs->lock held. */
static int consys_power_off_locked(struct mt6797_consys *cs)
{
	int ret;

	if (cs->stp_claimed)
		return -EBUSY;
	if (!cs->stages)
		return 0;

	ret = consys_cleanup_locked(cs);
	cs->last_error = ret;
	if (ret)
		dev_err(cs->dev, "ConSys power-off cleanup failed: %d\n", ret);
	else
		dev_dbg(cs->dev, "ConSys powered off\n");
	return ret;
}

int mt6797_consys_stp_set_mode(void *context, u32 mode)
{
	struct mt6797_consys *cs = context;
	int ret = 0;

	if (!cs || (mode & ~GENMASK(4, 0)))
		return -EINVAL;
	mutex_lock(&cs->lock);
	if (!cs->powered)
		ret = -EHOSTDOWN;
	else if (!cs->stp_claimed)
		ret = -EPERM;
	else
		cs->stp_mode = mode;
	mutex_unlock(&cs->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mt6797_consys_stp_set_mode);

int mt6797_consys_stp_set_enabled(void *context, bool enabled)
{
	struct mt6797_consys *cs = context;
	int ret = 0;

	if (!cs)
		return -EINVAL;
	mutex_lock(&cs->lock);
	if (enabled && !cs->powered)
		ret = -EHOSTDOWN;
	else if (!cs->stp_claimed)
		ret = -EPERM;
	else if (enabled && !cs->stp_mode)
		ret = -EINVAL;
	else
		cs->stp_enabled = enabled;
	mutex_unlock(&cs->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mt6797_consys_stp_set_enabled);

static int consys_stp_acquire(struct device *consys_dev,
			      struct mt6797_stp_host_ops *ops,
			      void **context, bool manage_power)
{
	struct mt6797_consys *cs;
	int ret = 0;

	if (!consys_dev || !ops || !context)
		return -EINVAL;
	cs = dev_get_drvdata(consys_dev);
	if (!cs)
		return -EPROBE_DEFER;
	mutex_lock(&cs->lock);
	if (cs->stp_claimed)
		ret = -EBUSY;
	else if (!cs->powered)
		ret = manage_power ? consys_power_on_locked(cs) : -EHOSTDOWN;
	if (!ret) {
		cs->stp_claimed = true;
		cs->stp_power_owned = manage_power;
		ops->set_mode = mt6797_consys_stp_set_mode;
		ops->set_enabled = mt6797_consys_stp_set_enabled;
		*context = cs;
	}
	mutex_unlock(&cs->lock);
	return ret;
}

int mt6797_consys_stp_acquire(struct device *consys_dev,
			      struct mt6797_stp_host_ops *ops, void **context)
{
	return consys_stp_acquire(consys_dev, ops, context, false);
}
EXPORT_SYMBOL_GPL(mt6797_consys_stp_acquire);

int mt6797_consys_stp_open(struct device *consys_dev,
			  struct mt6797_stp_host_ops *ops, void **context)
{
	return consys_stp_acquire(consys_dev, ops, context, true);
}
EXPORT_SYMBOL_GPL(mt6797_consys_stp_open);

int mt6797_consys_stp_release(void *context)
{
	struct mt6797_consys *cs = context;
	int ret = 0;

	if (!cs)
		return -EINVAL;
	mutex_lock(&cs->lock);
	if (!cs->stp_claimed)
		ret = -EINVAL;
	else if (cs->stp_enabled)
		ret = -EBUSY;
	else {
		cs->stp_claimed = false;
		cs->stp_mode = 0;
		if (cs->stp_power_owned) {
			cs->stp_power_owned = false;
			ret = consys_power_off_locked(cs);
		}
	}
	mutex_unlock(&cs->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mt6797_consys_stp_release);

int mt6797_consys_radio_power(void *context, u8 function, bool enabled,
				      bool *changed)
{
	struct mt6797_consys *cs = context;
	struct regulator *supply;
	bool *state;
	int ret = 0;

	if (!cs)
		return -EINVAL;
	if (changed)
		*changed = false;
	switch (function) {
	case MT6797_CONSYS_FUNC_BT:
		supply = cs->vcn33_bt;
		state = &cs->bt_radio_on;
		break;
	case MT6797_CONSYS_FUNC_WIFI:
		supply = cs->vcn33_wifi;
		state = &cs->wifi_radio_on;
		break;
	default:
		return -EINVAL;
	}

	mutex_lock(&cs->lock);
	if (!cs->powered)
		ret = -EHOSTDOWN;
	else if (!cs->stp_claimed)
		ret = -EPERM;
	else if (*state == enabled)
		ret = 0;
	else if (enabled)
		ret = regulator_enable(supply);
	else
		ret = regulator_disable(supply);
	if (!ret && *state != enabled) {
		*state = enabled;
		if (changed)
			*changed = true;
	}
	mutex_unlock(&cs->lock);

	if (ret)
		dev_err(cs->dev, "VCN33 function %u %s failed: %d\n",
			function, enabled ? "on" : "off", ret);
	else
		dev_dbg(cs->dev, "VCN33 function %u %s\n",
			 function, enabled ? "on" : "off");
	return ret;
}
EXPORT_SYMBOL_GPL(mt6797_consys_radio_power);

int mt6797_consys_get_emi(void *context, phys_addr_t *base,
				  phys_addr_t *size)
{
	struct mt6797_consys *cs = context;
	int ret = 0;

	if (!cs || !base || !size)
		return -EINVAL;

	mutex_lock(&cs->lock);
	if (!cs->stp_claimed)
		ret = -EPERM;
	else if (!cs->emi_size || !cs->emi_mapping_active || !cs->powered)
		ret = -ENODEV;
	else {
		*base = cs->emi_base;
		*size = cs->emi_size;
	}
	mutex_unlock(&cs->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mt6797_consys_get_emi);

int mt6797_consys_read_cpupcr(void *context, u32 *value)
{
	struct mt6797_consys *cs = context;
	int ret = 0;

	if (!cs || !value)
		return -EINVAL;
	mutex_lock(&cs->lock);
	if (!cs->stp_claimed || !cs->powered)
		ret = -EHOSTDOWN;
	else
		*value = readl(cs->mcu + 0x160);
	mutex_unlock(&cs->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mt6797_consys_read_cpupcr);

static int consys_emi_smc_set_locked(struct mt6797_consys *cs,
					     phys_addr_t start, phys_addr_t end,
					     unsigned int region, u32 permission)
{
	struct arm_smccc_res res;
	u32 packed;
	s64 result;

	if (!cs->emi_size || !cs->emi_io)
		return -ENODEV;
	packed = (region << MT6797_EMI_REGION_SHIFT) | permission;
	arm_smccc_smc(MT6797_SIP_EMIMPU_SET,
			      (u64)start, (u64)end, packed, 0, 0, 0, 0, &res);
	result = (s64)res.a0;
	if (!result)
		return 0;
	if (result == -1 || result == -2)
		return -EOPNOTSUPP;
	if (result == -3)
		return -ERANGE;
	if (result == -4)
		return -EACCES;
	return -EIO;
}

/* Called with cs->lock held. */
static int consys_emi_prepare_locked(struct mt6797_consys *cs)
{
	phys_addr_t base = cs->emi_base;
	u32 map_bits;
	u32 old_map;
	u32 readback;
	int ret;

	if (!cs->emi_size)
		return 0;
	if (!cs->emi_io)
		return -ENODEV;
	if (!IS_ALIGNED(base, MT6797_CONSYS_EMI_REGION_SIZE) ||
	    cs->emi_size < MT6797_CONSYS_EMI_REGION_SIZE ||
	    base > U32_MAX)
		return -ERANGE;

	/* Replace the address field rather than OR-ing a previous mapping into it. */
	map_bits = ((u32)base & 0xfff00000U) >> 20;
	map_bits |= CONSYS_EMI_MAPPING_ENABLE;
	old_map = readl(cs->topckgen + CONSYS_EMI_MAPPING_OFF);
	cs->emi_mapping_saved = old_map;
	cs->emi_mapping_active = true;
	map_bits |= old_map & ~GENMASK(12, 0);
	writel(map_bits, cs->topckgen + CONSYS_EMI_MAPPING_OFF);
	readback = readl(cs->topckgen + CONSYS_EMI_MAPPING_OFF);
	if (readback != map_bits) {
		ret = -EIO;
		goto err_map;
	}

	/*
	 * Region 19 is the AP/ConSys host window; region 18 is opened only for
	 * the bounded firmware transaction.
	 */
	ret = consys_emi_smc_set_locked(cs, base + MT6797_EMI_WINDOW_SIZE,
					base + MT6797_EMI_MAPPED_SIZE - 1,
					MT6797_EMI_REGION19, MT6797_EMI_REGION19_HOST);
	if (ret)
		goto err_map;
	ret = consys_emi_smc_set_locked(cs, base, base + MT6797_EMI_WINDOW_SIZE - 1,
					MT6797_EMI_REGION18, MT6797_EMI_REGION18_OPEN);
	if (ret)
		goto err_map;
	cs->emi_patch_active = true;

	/* This only runs before MCU reset release, never between patch images. */
	memset_io(cs->emi_io, 0, MT6797_EMI_MAPPED_SIZE);
	/* Complete the clear before readback and removal of AP write access. */
	wmb();
	if (readl(cs->emi_io) ||
	    readl(cs->emi_io + MT6797_EMI_MAPPED_SIZE - sizeof(u32))) {
		ret = -EIO;
		goto err_region;
	}
	ret = consys_emi_smc_set_locked(cs, base, base + MT6797_EMI_WINDOW_SIZE - 1,
					MT6797_EMI_REGION18, MT6797_EMI_REGION18_RESTORE);
	if (ret)
		goto err_region;
	cs->emi_patch_active = false;
	dev_dbg(cs->dev, "ConSys EMI owner active: remap=0x%08x base=%pa\n",
		 readback, &base);
	return 0;

err_region:
	if (!consys_emi_smc_set_locked(cs, base, base + MT6797_EMI_WINDOW_SIZE - 1,
					 MT6797_EMI_REGION18, MT6797_EMI_REGION18_RESTORE))
		cs->emi_patch_active = false;
err_map:
	writel(cs->emi_mapping_saved, cs->topckgen + CONSYS_EMI_MAPPING_OFF);
	cs->emi_mapping_active = readl(cs->topckgen + CONSYS_EMI_MAPPING_OFF) !=
				 cs->emi_mapping_saved;
	return ret;
}

/* Called with cs->lock held. */
static int consys_emi_restore_locked(struct mt6797_consys *cs)
{
	int ret;

	if (!cs->emi_patch_active && !cs->emi_mapping_active)
		return 0;
	ret = consys_emi_smc_set_locked(cs, cs->emi_base,
					cs->emi_base + MT6797_EMI_WINDOW_SIZE - 1,
					MT6797_EMI_REGION18, MT6797_EMI_REGION18_RESTORE);
	if (ret)
		dev_warn(cs->dev, "failed to restore ConSys EMI region 18: %d\n", ret);
	if (cs->emi_mapping_active) {
		writel(cs->emi_mapping_saved,
		       cs->topckgen + CONSYS_EMI_MAPPING_OFF);
		cs->emi_mapping_active = readl(cs->topckgen + CONSYS_EMI_MAPPING_OFF) !=
					 cs->emi_mapping_saved;
		if (cs->emi_mapping_active) {
			dev_warn(cs->dev, "failed to restore ConSys EMI remap\n");
			if (!ret)
				ret = -EIO;
		}
	}
	if (!ret)
		cs->emi_patch_active = false;
	return ret;
}

int mt6797_consys_emi_patch_begin(void *context)
{
	struct mt6797_consys *cs = context;
	int ret;

	if (!cs)
		return -EINVAL;

	mutex_lock(&cs->lock);
	if (!cs->stp_claimed)
		ret = -EPERM;
	else if (!cs->powered)
		ret = -EHOSTDOWN;
	else if (cs->emi_patch_active)
		ret = -EBUSY;
	else if (!cs->emi_size || !cs->emi_mapping_active)
		ret = -ENODEV;
	else
		ret = consys_emi_smc_set_locked(cs, cs->emi_base,
					cs->emi_base + MT6797_EMI_WINDOW_SIZE - 1,
					MT6797_EMI_REGION18, MT6797_EMI_REGION18_OPEN);
	if (!ret)
		cs->emi_patch_active = true;
	mutex_unlock(&cs->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mt6797_consys_emi_patch_begin);

int mt6797_consys_emi_patch_end(void *context)
{
	struct mt6797_consys *cs = context;
	int ret;

	if (!cs)
		return -EINVAL;
	mutex_lock(&cs->lock);
	if (!cs->stp_claimed || !cs->emi_patch_active) {
		ret = -EPERM;
	} else {
		ret = consys_emi_smc_set_locked(cs, cs->emi_base,
					cs->emi_base + MT6797_EMI_WINDOW_SIZE - 1,
					MT6797_EMI_REGION18, MT6797_EMI_REGION18_RESTORE);
		if (!ret)
			cs->emi_patch_active = false;
	}
	/* Firmware keeps using the remap until power-off asserts MCU reset. */
	mutex_unlock(&cs->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mt6797_consys_emi_patch_end);

int mt6797_consys_wifi_emi_write(void *context, u32 offset,
			       const void *data, size_t length)
{
	struct mt6797_consys *cs = context;
	u8 verify[64];
	size_t position, count;
	int ret, restore_ret;

	if (!cs || !data || !length || offset >= MT6797_CONSYS_WIFI_EMI_SIZE ||
	    length > MT6797_CONSYS_WIFI_EMI_SIZE - offset)
		return -EINVAL;
	ret = mt6797_consys_emi_patch_begin(context);
	if (ret)
		return ret;
	mutex_lock(&cs->lock);
	memcpy_toio(cs->emi_io + offset, data, length);
	wmb();
	for (position = 0; position < length; position += count) {
		count = min_t(size_t, sizeof(verify), length - position);
		memcpy_fromio(verify, cs->emi_io + offset + position, count);
		if (memcmp(verify, (const u8 *)data + position, count)) {
			ret = -EIO;
			break;
		}
	}
	mutex_unlock(&cs->lock);
	restore_ret = mt6797_consys_emi_patch_end(context);
	return ret ? ret : restore_ret;
}
EXPORT_SYMBOL_GPL(mt6797_consys_wifi_emi_write);

static ssize_t radio_power_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct mt6797_consys *cs = dev_get_drvdata(dev);
	bool powered;

	mutex_lock(&cs->lock);
	powered = cs->powered;
	mutex_unlock(&cs->lock);

	return sysfs_emit(buf, "%d\n", powered);
}

static ssize_t radio_power_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct mt6797_consys *cs = dev_get_drvdata(dev);
	bool on;
	int ret;

	ret = kstrtobool(buf, &on);
	if (ret)
		return ret;

	mutex_lock(&cs->lock);
	if (on)
		ret = consys_power_on_locked(cs);
	else
		ret = consys_power_off_locked(cs);
	mutex_unlock(&cs->lock);

	return ret ? ret : count;
}

static ssize_t chipid_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct mt6797_consys *cs = dev_get_drvdata(dev);
	u32 chipid;

	mutex_lock(&cs->lock);
	if (!cs->powered) {
		mutex_unlock(&cs->lock);
		return sysfs_emit(buf, "off\n");
	}
	chipid = readl(cs->mcu + CONSYS_MCU_CHIP_ID_OFF);
	mutex_unlock(&cs->lock);

	return sysfs_emit(buf, "0x%04x\n", chipid);
}

static ssize_t state_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct mt6797_consys *cs = dev_get_drvdata(dev);
	u32 stages;

	mutex_lock(&cs->lock);
	stages = cs->stages;
	mutex_unlock(&cs->lock);

	return sysfs_emit(buf, "0x%08x\n", stages);
}

static ssize_t stp_mode_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct mt6797_consys *cs = dev_get_drvdata(dev);
	u32 mode;

	mutex_lock(&cs->lock);
	mode = cs->stp_mode;
	mutex_unlock(&cs->lock);
	return sysfs_emit(buf, "0x%08x\n", mode);
}

static ssize_t stp_enabled_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct mt6797_consys *cs = dev_get_drvdata(dev);
	bool enabled;

	mutex_lock(&cs->lock);
	enabled = cs->stp_enabled;
	mutex_unlock(&cs->lock);
	return sysfs_emit(buf, "%d\n", enabled);
}

static ssize_t last_error_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct mt6797_consys *cs = dev_get_drvdata(dev);
	int error;

	mutex_lock(&cs->lock);
	error = cs->last_error;
	mutex_unlock(&cs->lock);

	return sysfs_emit(buf, "%d\n", error);
}

static ssize_t emi_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct mt6797_consys *cs = dev_get_drvdata(dev);
	phys_addr_t base, size;

	mutex_lock(&cs->lock);
	base = cs->emi_base;
	size = cs->emi_size;
	mutex_unlock(&cs->lock);

	if (!size)
		return sysfs_emit(buf, "none\n");
	return sysfs_emit(buf, "base=0x%pa size=0x%pa wifi=0x%x\n",
			  &base, &size, MT6797_CONSYS_WIFI_EMI_SIZE);
}

/* "power" is the device PM directory when CONFIG_PM is enabled. */
static DEVICE_ATTR_RW(radio_power);
static DEVICE_ATTR_RO(chipid);
static DEVICE_ATTR_RO(state);
static DEVICE_ATTR_RO(stp_mode);
static DEVICE_ATTR_RO(stp_enabled);
static DEVICE_ATTR_RO(last_error);
static DEVICE_ATTR_RO(emi);

static struct attribute *consys_attrs[] = {
	&dev_attr_radio_power.attr,
	&dev_attr_chipid.attr,
	&dev_attr_state.attr,
	&dev_attr_stp_mode.attr,
	&dev_attr_stp_enabled.attr,
	&dev_attr_last_error.attr,
	&dev_attr_emi.attr,
	NULL,
};
ATTRIBUTE_GROUPS(consys);

static int mt6797_consys_probe(struct platform_device *pdev)
{
	struct mt6797_consys *cs;
	struct resource *r;
	struct resource emi;
	int ret;
	int i;

	cs = devm_kzalloc(&pdev->dev, sizeof(*cs), GFP_KERNEL);
	if (!cs)
		return -ENOMEM;

	cs->dev = &pdev->dev;
	mutex_init(&cs->lock);
	/*
	 * Every window here is (partly) owned by another driver: the RGU by
	 * the watchdog, TOPCKGEN by the clock driver.  Map them shared
	 * (devm_ioremap, no request_mem_region) and touch only the ConSys
	 * registers defined above.
	 */
	for (i = 0; i < ARRAY_SIZE(cs->base); i++) {
		r = platform_get_resource(pdev, IORESOURCE_MEM, i);
		if (!r)
			return -ENODEV;
		cs->base[i] = devm_ioremap(&pdev->dev, r->start, resource_size(r));
		if (!cs->base[i])
			return -ENOMEM;
	}
	cs->mcu = cs->base[0];
	cs->rgu = cs->base[1];
	cs->topckgen = cs->base[2];
	cs->spm = cs->base[3];

	/*
	 * The vendor MT6797 layout reserves a 2 MiB, 2 MiB-aligned region for
	 * ConSys.  Wi-Fi uses its first 512 KiB; later offsets belong to ConSys
	 * diagnostics/GPS. Remap/MPU setup and clearing happen at power-on before
	 * MCU reset release; firmware retains that mapping until power-off.
	 */
	ret = of_reserved_mem_region_to_resource(pdev->dev.of_node, 0, &emi);
	if (!ret) {
		if (!IS_ALIGNED(emi.start, MT6797_CONSYS_EMI_REGION_SIZE) ||
		    resource_size(&emi) < MT6797_CONSYS_EMI_REGION_SIZE)
			return dev_err_probe(&pdev->dev, -EINVAL,
					     "invalid ConSys EMI reservation %pa/%pa\n",
					     &emi.start, &emi.end);
		cs->emi_base = emi.start;
		cs->emi_size = resource_size(&emi);
		cs->emi_io = devm_ioremap(&pdev->dev, cs->emi_base,
					  MT6797_CONSYS_EMI_REGION_SIZE);
		if (!cs->emi_io)
			return dev_err_probe(&pdev->dev, -ENOMEM,
					     "failed to map ConSys EMI reservation\n");
		dev_info(&pdev->dev,
			 "ConSys EMI reservation base=%pa size=%pa (Wi-Fi=%x)\n",
			 &cs->emi_base, &cs->emi_size, MT6797_CONSYS_WIFI_EMI_SIZE);
	} else if (ret != -ENOENT && ret != -ENODEV) {
		return dev_err_probe(&pdev->dev, ret,
				     "failed to resolve ConSys EMI reservation\n");
	} else {
		dev_warn(&pdev->dev,
			 "no ConSys EMI reservation; WLAN firmware cannot start\n");
	}

	cs->vcn18 = devm_regulator_get_exclusive(&pdev->dev, "vcn18");
	if (IS_ERR(cs->vcn18))
		return dev_err_probe(&pdev->dev, PTR_ERR(cs->vcn18),
				     "vcn18 supply missing\n");
	cs->vcn28 = devm_regulator_get_exclusive(&pdev->dev, "vcn28");
	if (IS_ERR(cs->vcn28))
		return dev_err_probe(&pdev->dev, PTR_ERR(cs->vcn28),
				     "vcn28 supply missing\n");
	cs->vcn33_bt = devm_regulator_get_exclusive(&pdev->dev, "vcn33-bt");
	if (IS_ERR(cs->vcn33_bt))
		return dev_err_probe(&pdev->dev, PTR_ERR(cs->vcn33_bt),
				     "vcn33-bt supply missing\n");
	cs->vcn33_wifi = devm_regulator_get_exclusive(&pdev->dev, "vcn33-wifi");
	if (IS_ERR(cs->vcn33_wifi))
		return dev_err_probe(&pdev->dev, PTR_ERR(cs->vcn33_wifi),
				     "vcn33-wifi supply missing\n");

	platform_set_drvdata(pdev, cs);
	ret = device_add_groups(&pdev->dev, consys_groups);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to create sysfs attributes\n");

	dev_info(&pdev->dev, "MT6797 ConSys bring-up driver probed\n");
	return 0;
}

static void mt6797_consys_remove(struct platform_device *pdev)
{
	struct mt6797_consys *cs = platform_get_drvdata(pdev);

	device_remove_groups(&pdev->dev, consys_groups);
	mutex_lock(&cs->lock);
	consys_power_off_locked(cs);
	mutex_unlock(&cs->lock);
}

static const struct of_device_id mt6797_consys_of_match[] = {
	{ .compatible = "mediatek,mt6797-consys" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mt6797_consys_of_match);

static struct platform_driver mt6797_consys_driver = {
	.probe = mt6797_consys_probe,
	.remove = mt6797_consys_remove,
	.driver = {
		.name = "mt6797-consys",
		.of_match_table = mt6797_consys_of_match,
	},
};
module_platform_driver(mt6797_consys_driver);

MODULE_DESCRIPTION("MT6797 ConSys power/bring-up driver");
MODULE_LICENSE("GPL");
