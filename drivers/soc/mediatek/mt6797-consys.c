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
//       power  - 1 to run the full power-on sequence, 0 to power off
//       chipid - read of CONN_MCU_CFG + 0x08 (0x0279 when the MCU is alive)
//       wl_dump- 8 words read back from the WLAN MAC window at 0x180f0000

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kstrtox.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>

#define CONSYS_CHIP_ID			0x0279

/* CONN_MCU_CONFIG_BASE */
#define CONSYS_MCU_CHIP_ID_OFF		0x0008
#define CONSYS_MCU_CFG_ACR_OFF		0x0110
#define CONSYS_MCU_CFG_ACR_MBIST	BIT(18)
/* AP_RGU_BASE */
#define CONSYS_RGU_SWSYSRST_OFF		0x0018
#define CONSYS_RGU_KEY			(0x88u << 24)
#define CONSYS_RGU_RST_BIT		BIT(12)
/* TOPCKGEN_BASE */
#define CONSYS_TOPAXI_PROT_EN_OFF	0x1220
#define CONSYS_TOPAXI_PROT_STA1_OFF	0x1228
#define CONSYS_TOPAXI_PROT_MASK		(BIT(18) | BIT(19))
#define CONSYS_CONN2AP_SLEEP_MASK_OFF	0x1350
#define CONSYS_CONN2AP_SLEEP_MASK_BIT	BIT(8)
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

/* WLAN MAC window, read back only as bring-up evidence. */
#define CONSYS_WLAN_MAC_BASE		0x180f0000
#define CONSYS_WLAN_DUMP_WORDS		8

struct mt6797_consys {
	struct device *dev;
	void __iomem *mcu;
	void __iomem *rgu;
	void __iomem *topckgen;
	void __iomem *spm;
	void __iomem *wlan;
	struct regulator *vcn18;
	struct regulator *vcn28;
	struct regulator *vcn33_bt;
	struct regulator *vcn33_wifi;
	bool powered;
};

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

static void consys_rgu_sw_rst(struct mt6797_consys *cs, bool assert)
{
	u32 val = readl(cs->rgu + CONSYS_RGU_SWSYSRST_OFF);

	if (assert)
		val |= CONSYS_RGU_RST_BIT;
	else
		val &= ~CONSYS_RGU_RST_BIT;
	val |= CONSYS_RGU_KEY;
	writel(val, cs->rgu + CONSYS_RGU_SWSYSRST_OFF);
}

static int consys_power_on(struct mt6797_consys *cs)
{
	u32 val, chipid;
	int ret, retry;

	if (cs->powered)
		return 0;

	/* Step 1: connectivity LDOs (vendor order: VCN18, settle, VCN28). */
	ret = regulator_enable(cs->vcn18);
	if (ret) {
		dev_err(cs->dev, "failed to enable vcn18: %d\n", ret);
		return ret;
	}
	udelay(240);
	ret = regulator_enable(cs->vcn28);
	if (ret) {
		dev_err(cs->dev, "failed to enable vcn28: %d\n", ret);
		goto err_vcn28;
	}

	/* Mask ConSys sleep requests while the MCU boots. */
	val = readl(cs->topckgen + CONSYS_CONN2AP_SLEEP_MASK_OFF);
	writel(val | CONSYS_CONN2AP_SLEEP_MASK_BIT,
	       cs->topckgen + CONSYS_CONN2AP_SLEEP_MASK_OFF);

	/* Step 2: hold the ConSys MCU in software reset during power-up. */
	consys_rgu_sw_rst(cs, true);

	/* Step 3: unmask SPM power-on control for the CONN domain. */
	writel(CONSYS_SPM_PWRON_CONFG_EN_VAL,
	       cs->spm + CONSYS_SPM_PWRON_CONFG_EN_OFF);

	/* Step 4: MTCMOS power-up with ISO release and clock enable. */
	val = readl(cs->spm + CONSYS_SPM_CONN_PWR_CTRL_OFF);
	writel(val | CONSYS_SPM_PWR_ON_BIT, cs->spm + CONSYS_SPM_CONN_PWR_CTRL_OFF);
	ret = consys_wait_ack(cs, CONSYS_SPM_PWR_ACK_OFF, CONSYS_SPM_PWR_ACK_BIT);
	if (ret)
		goto err_pwr;

	val = readl(cs->spm + CONSYS_SPM_CONN_PWR_CTRL_OFF);
	writel(val | CONSYS_SPM_PWR_ON_S_BIT, cs->spm + CONSYS_SPM_CONN_PWR_CTRL_OFF);
	ret = consys_wait_ack(cs, CONSYS_SPM_PWR_ACK_S_OFF, CONSYS_SPM_PWR_ACK_BIT);
	if (ret)
		goto err_pwr;

	val = readl(cs->spm + CONSYS_SPM_CONN_PWR_CTRL_OFF);
	writel(val & ~CONSYS_SPM_CLK_DIS_BIT, cs->spm + CONSYS_SPM_CONN_PWR_CTRL_OFF);
	udelay(1);

	val = readl(cs->spm + CONSYS_SPM_CONN_PWR_CTRL_OFF);
	writel(val & ~CONSYS_SPM_PWR_ISO_S_BIT, cs->spm + CONSYS_SPM_CONN_PWR_CTRL_OFF);

	val = readl(cs->spm + CONSYS_SPM_CONN_PWR_CTRL_OFF);
	writel(val | CONSYS_SPM_PWR_RST_BIT, cs->spm + CONSYS_SPM_CONN_PWR_CTRL_OFF);

	/* Step 5: release the ConSys AXI bus protection. */
	val = readl(cs->topckgen + CONSYS_TOPAXI_PROT_EN_OFF);
	writel(val & ~CONSYS_TOPAXI_PROT_MASK, cs->topckgen + CONSYS_TOPAXI_PROT_EN_OFF);
	ret = readl_poll_timeout(cs->topckgen + CONSYS_TOPAXI_PROT_STA1_OFF,
				 val, !(val & CONSYS_TOPAXI_PROT_MASK),
				 10, 100000);
	if (ret) {
		dev_err(cs->dev, "AXI bus protect stuck (sta1=0x%08x)\n", val);
		goto err_pwr;
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
		goto err_pwr;
	}
	dev_info(cs->dev, "ConSys chip ID 0x%04x confirmed\n", chipid);

	/* Step 7: run MBIST at real speed, then release the MCU reset. */
	val = readl(cs->mcu + CONSYS_MCU_CFG_ACR_OFF);
	writel(val | CONSYS_MCU_CFG_ACR_MBIST, cs->mcu + CONSYS_MCU_CFG_ACR_OFF);

	consys_rgu_sw_rst(cs, false);
	msleep(20);

	cs->powered = true;
	dev_info(cs->dev, "ConSys powered on\n");
	return 0;

err_pwr:
	regulator_disable(cs->vcn28);
err_vcn28:
	regulator_disable(cs->vcn18);
	return ret;
}

static void consys_power_off(struct mt6797_consys *cs)
{
	u32 val;

	if (!cs->powered)
		return;

	/* Hold the MCU in reset, then reverse the MTCMOS sequence. */
	consys_rgu_sw_rst(cs, true);

	val = readl(cs->topckgen + CONSYS_TOPAXI_PROT_EN_OFF);
	writel(val | CONSYS_TOPAXI_PROT_MASK, cs->topckgen + CONSYS_TOPAXI_PROT_EN_OFF);

	val = readl(cs->spm + CONSYS_SPM_CONN_PWR_CTRL_OFF);
	writel(val & ~CONSYS_SPM_PWR_RST_BIT, cs->spm + CONSYS_SPM_CONN_PWR_CTRL_OFF);

	val = readl(cs->spm + CONSYS_SPM_CONN_PWR_CTRL_OFF);
	writel(val | CONSYS_SPM_PWR_ISO_S_BIT, cs->spm + CONSYS_SPM_CONN_PWR_CTRL_OFF);

	val = readl(cs->spm + CONSYS_SPM_CONN_PWR_CTRL_OFF);
	writel(val | CONSYS_SPM_CLK_DIS_BIT, cs->spm + CONSYS_SPM_CONN_PWR_CTRL_OFF);
	udelay(1);

	val = readl(cs->spm + CONSYS_SPM_CONN_PWR_CTRL_OFF);
	writel(val & ~(CONSYS_SPM_PWR_ON_BIT | CONSYS_SPM_PWR_ON_S_BIT),
	       cs->spm + CONSYS_SPM_CONN_PWR_CTRL_OFF);

	writel(CONSYS_SPM_PWRON_CONFG_EN_VAL & ~0x1,
	       cs->spm + CONSYS_SPM_PWRON_CONFG_EN_OFF);

	val = readl(cs->topckgen + CONSYS_CONN2AP_SLEEP_MASK_OFF);
	writel(val & ~CONSYS_CONN2AP_SLEEP_MASK_BIT,
	       cs->topckgen + CONSYS_CONN2AP_SLEEP_MASK_OFF);

	regulator_disable(cs->vcn28);
	regulator_disable(cs->vcn18);
	cs->powered = false;
	dev_info(cs->dev, "ConSys powered off\n");
}

static ssize_t power_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct mt6797_consys *cs = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", cs->powered);
}

static ssize_t power_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct mt6797_consys *cs = dev_get_drvdata(dev);
	bool on;
	int ret;

	ret = kstrtobool(buf, &on);
	if (ret)
		return ret;

	if (on)
		ret = consys_power_on(cs);
	else
		consys_power_off(cs);

	return ret ? ret : count;
}

static ssize_t chipid_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct mt6797_consys *cs = dev_get_drvdata(dev);

	if (!cs->powered)
		return sysfs_emit(buf, "off\n");

	return sysfs_emit(buf, "0x%04x\n",
			  readl(cs->mcu + CONSYS_MCU_CHIP_ID_OFF));
}

static ssize_t wl_dump_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct mt6797_consys *cs = dev_get_drvdata(dev);
	ssize_t len = 0;
	int i;

	if (!cs->powered)
		return sysfs_emit(buf, "off\n");

	for (i = 0; i < CONSYS_WLAN_DUMP_WORDS; i++)
		len += sysfs_emit_at(buf, len, "%08x%s",
				     readl(cs->wlan + i * 4),
				     (i % 4 == 3) ? "\n" : " ");

	return len;
}

static DEVICE_ATTR_RW(power);
static DEVICE_ATTR_RO(chipid);
static DEVICE_ATTR_RO(wl_dump);

static struct attribute *consys_attrs[] = {
	&dev_attr_power.attr,
	&dev_attr_chipid.attr,
	&dev_attr_wl_dump.attr,
	NULL,
};
ATTRIBUTE_GROUPS(consys);

static int mt6797_consys_probe(struct platform_device *pdev)
{
	struct mt6797_consys *cs;
	int ret;

	cs = devm_kzalloc(&pdev->dev, sizeof(*cs), GFP_KERNEL);
	if (!cs)
		return -ENOMEM;

	cs->dev = &pdev->dev;
	cs->mcu = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(cs->mcu))
		return PTR_ERR(cs->mcu);
	cs->rgu = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(cs->rgu))
		return PTR_ERR(cs->rgu);
	cs->topckgen = devm_platform_ioremap_resource(pdev, 2);
	if (IS_ERR(cs->topckgen))
		return PTR_ERR(cs->topckgen);
	cs->spm = devm_platform_ioremap_resource(pdev, 3);
	if (IS_ERR(cs->spm))
		return PTR_ERR(cs->spm);

	/*
	 * The WLAN MAC window has no DT node of its own; map it directly for
	 * bring-up evidence.  Failures here are not fatal for power control.
	 */
	cs->wlan = devm_ioremap(&pdev->dev, CONSYS_WLAN_MAC_BASE, 0x100);
	if (!cs->wlan)
		dev_warn(cs->dev, "WLAN MAC window map failed; wl_dump unavailable\n");

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
	consys_power_off(cs);
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
