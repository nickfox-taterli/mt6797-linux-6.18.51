// SPDX-License-Identifier: GPL-2.0-only
/* MT6797 gen3 glue using the owned mainline AHB and WMT providers.
 * Protocol/queue/STA/AP/P2P code remains in the BSP gen3 core.
 */
#include "precomp.h"
#include <linux/err.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/mt6797-stp-session.h>
#include <linux/mt6797-wlan-hif.h>
#include <linux/mt6797-wmt.h>

struct mt6797_ahb {
	struct device *dev;
	struct mt6797_wmt_client *wmt;
	struct mt6797_wifi_nvram nvram;
	P_GLUE_INFO_T glue;
	bool function;
	bool irq;
};
static DEFINE_MUTEX(current_lock);
static struct device *current_device;
static u32 chip_versions[2];
static probe_card wlan_probe;
static remove_card wlan_remove;
UINT_32 IsrCnt, IsrPassCnt;

static struct mt6797_ahb *glue_bus(P_GLUE_INFO_T glue)
{
	return dev_get_drvdata(glue->rHifInfo.Dev);
}

VOID glSetHifInfo(GLUE_INFO_T *glue, ULONG cookie)
{
	struct device *dev = (struct device *)cookie;
	struct mt6797_ahb *bus = dev_get_drvdata(dev);

	glue->rHifInfo.Dev = dev;
	glue->rHifInfo.wmt = bus->wmt;
	glue->rHifInfo.nvram = &bus->nvram;
	glue->rHifInfo.ChipID = 0x6797;
	glue->rHifInfo.fgDmaEnable = FALSE;
	bus->glue = glue;
	SET_NETDEV_DEV(glue->prDevHandler, dev);
}

VOID glClearHifInfo(P_GLUE_INFO_T glue)
{
	glue_bus(glue)->glue = NULL;
	glue->rHifInfo.wmt = NULL;
}

BOOLEAN glBusInit(PVOID data)
{
	struct mt6797_ahb *bus = dev_get_drvdata(data);

	return bus && bus->function;
}

VOID glBusRelease(PVOID data)
{
	/* Platform remove owns teardown after protocol threads have stopped. */
}

BOOLEAN kalDevRegRead(GLUE_INFO_T *glue, UINT_32 address, PUINT_32 value)
{
	return glue_bus(glue)->function &&
	       !mt6797_wlan_hif_pio_read(glue->rHifInfo.Dev, address, value);
}

BOOLEAN kalDevRegWrite(GLUE_INFO_T *glue, UINT_32 address, UINT_32 value)
{
	int ret;

	if (!glue_bus(glue)->function)
		return FALSE;
	ret = mt6797_wlan_hif_pio_write(glue->rHifInfo.Dev, address, value);
	if (!ret && address == MCR_WHLPCR && (value & WHLPCR_INT_EN_SET))
		ret = mt6797_wlan_hif_mask_irq(glue->rHifInfo.Dev, false);
	return !ret;
}

BOOLEAN kalDevPortRead(P_GLUE_INFO_T glue, UINT_16 port, UINT_32 length,
		      PUINT_8 buffer, UINT_32 capacity)
{
	if (!glue_bus(glue)->function || !buffer || length > capacity ||
	    HifIsFwOwn(glue->prAdapter))
		return FALSE;
	return !mt6797_wlan_hif_transfer(glue->rHifInfo.Dev, port, false, buffer, length);
}

BOOLEAN kalDevPortWrite(P_GLUE_INFO_T glue, UINT_16 port, UINT_32 length,
		       PUINT_8 buffer, UINT_32 capacity)
{
	if (!glue_bus(glue)->function || !buffer || length > capacity ||
	    HifIsFwOwn(glue->prAdapter))
		return FALSE;
	return !mt6797_wlan_hif_transfer(glue->rHifInfo.Dev, port, true, buffer, length);
}

static irqreturn_t mt6797_ahb_irq(int irq, void *data)
{
	P_GLUE_INFO_T glue = data;

	mt6797_wlan_hif_mask_irq(glue->rHifInfo.Dev, true);
	IsrCnt++;
	glue->IsrCnt++;
	if (test_bit(GLUE_FLAG_HALT_BIT, &glue->ulFlag))
		return IRQ_HANDLED;
	set_bit(GLUE_FLAG_INT_BIT, &glue->ulFlag);
#if CFG_SUPPORT_MULTITHREAD
	wake_up_interruptible(&glue->waitq_hif);
#else
	wake_up_interruptible(&glue->waitq);
#endif
	IsrPassCnt++;
	glue->IsrPassCnt++;
	return IRQ_HANDLED;
}

INT_32 glBusSetIrq(PVOID data, PVOID handler, PVOID context)
{
	P_GLUE_INFO_T glue = context;
	struct mt6797_ahb *bus = glue_bus(glue);
	int ret;

	ret = mt6797_wlan_hif_request_threaded_irq(bus->dev, mt6797_ahb_irq,
						  "mt6797-wlan", glue);
	if (ret)
		return ret;
	bus->irq = true;
	ret = mt6797_wlan_hif_enable_irq(bus->dev);
	if (ret) {
		mt6797_wlan_hif_free_irq(bus->dev, glue);
		bus->irq = false;
	}
	return ret;
}

VOID glBusFreeIrq(PVOID data, PVOID context)
{
	P_GLUE_INFO_T glue = context;
	struct mt6797_ahb *bus = glue_bus(glue);

	if (bus->irq) {
		if (bus->function)
			mt6797_wlan_hif_mask_irq(bus->dev, true);
		mt6797_wlan_hif_free_irq(bus->dev, glue);
		bus->irq = false;
	}
}

VOID glResetHif(P_GLUE_INFO_T glue)
{
	/* PIO has no in-flight AP-DMA transaction to reset. */
}

VOID glGetChipInfo(P_GLUE_INFO_T glue, UINT_8 *buffer)
{
	scnprintf(buffer, 32, "MT6797 AHB chip=%04x", glue->rHifInfo.ChipID);
}

INT_32 glSetPowerState(P_GLUE_INFO_T glue, UINT_32 mode)
{
	struct mt6797_ahb *bus = glue_bus(glue);
	int ret = 0;

	if (mode == ParamDeviceStateD3 && bus->function) {
		if (bus->irq)
			ret = mt6797_wlan_hif_disable_irq(bus->dev);
		if (!ret)
			ret = mt6797_wlan_hif_function(bus->dev, false);
		if (!ret) {
			bus->function = false;
			ret = mt6797_wmt_client_close(bus->wmt);
		}
	} else if (mode == ParamDeviceStateD0 && !bus->function) {
		ret = mt6797_wmt_client_open(bus->wmt);
		if (!ret)
			ret = mt6797_wlan_hif_function(bus->dev, true);
		if (!ret && bus->irq)
			ret = mt6797_wlan_hif_enable_irq(bus->dev);
		if (!ret)
			bus->function = true;
		else
			mt6797_wmt_client_close(bus->wmt);
	}
	if (ret)
		dev_err(bus->dev, "power state %u failed: %d\n", mode, ret);
	return ret;
}

u32 mt6797_wlan_chip_version(bool firmware)
{
	return READ_ONCE(chip_versions[firmware ? 1 : 0]);
}

VOID glDumpConnSysCpuInfo(P_GLUE_INFO_T glue)
{
	u32 value;

	if (!mt6797_wmt_client_cpupcr(glue->rHifInfo.wmt, &value))
		dev_info(glue->rHifInfo.Dev, "ConSys CPUPCR=%08x\n", value);
}

PUINT_8 glRemapConnsysAddr(P_GLUE_INFO_T glue, UINT_32 address, UINT_32 length)
{
	/* The old WTBL dump repurposes the live ConSys EMI remap register.
	 * Keep it unavailable until a separate, owned diagnostic window exists.
	 */
	dev_warn_ratelimited(glue->rHifInfo.Dev,
			     "diagnostic remap %08x/%u unavailable\n", address, length);
	return NULL;
}

VOID glUnmapConnsysAddr(P_GLUE_INFO_T glue, PUINT_8 mapping, UINT_32 address)
{
	WARN_ON(mapping);
}

static int mt6797_ahb_probe(struct platform_device *pdev)
{
	struct mt6797_ahb *bus;
	u32 chip;
	int ret;

	bus = devm_kzalloc(&pdev->dev, sizeof(*bus), GFP_KERNEL);
	if (!bus)
		return -ENOMEM;
	bus->dev = &pdev->dev;
	ret = mt6797_nvram_init(bus->dev, &bus->nvram);
	if (ret)
		return dev_err_probe(bus->dev, ret, "WiFi calibration unavailable\n");
	bus->wmt = mt6797_wmt_client_get(bus->dev, MT6797_STP_TASK_WIFI, NULL, NULL);
	if (IS_ERR(bus->wmt)) {
		ret = PTR_ERR(bus->wmt);
		goto release_nvram;
	}
	ret = mt6797_wmt_client_open(bus->wmt);
	if (ret)
		goto put_wmt;
	ret = mt6797_wmt_client_version(bus->wmt, &chip_versions[0], &chip_versions[1]);
	if (ret)
		goto close_wmt;
	ret = mt6797_wlan_hif_claim(bus->dev);
	if (ret)
		goto close_wmt;
	ret = mt6797_wlan_hif_function(bus->dev, true);
	if (ret)
		goto release_hif;
	bus->function = true;
	ret = mt6797_wlan_hif_wcir_read(bus->dev, &chip);
	if (ret || (chip & 0xffff) != 0x0279) {
		ret = ret ?: -ENODEV;
		goto disable_function;
	}
	platform_set_drvdata(pdev, bus);
	if (wlan_probe(bus->dev) == WLAN_STATUS_SUCCESS) {
		mutex_lock(&current_lock);
		current_device = get_device(bus->dev);
		mutex_unlock(&current_lock);
		return 0;
	}
	ret = -EIO;
disable_function:
	mt6797_wlan_hif_function(bus->dev, false);
release_hif:
	mt6797_wlan_hif_release(bus->dev);
close_wmt:
	mt6797_wmt_client_close(bus->wmt);
put_wmt:
	platform_set_drvdata(pdev, NULL);
	mt6797_wmt_client_put(bus->wmt);
release_nvram:
	mt6797_nvram_release(&bus->nvram);
	return dev_err_probe(bus->dev, ret, "WLAN initialization failed\n");
}

static void mt6797_ahb_remove(struct platform_device *pdev)
{
	struct mt6797_ahb *bus = platform_get_drvdata(pdev);

	mutex_lock(&current_lock);
	put_device(current_device);
	current_device = NULL;
	mutex_unlock(&current_lock);
	if (bus->glue)
		wlan_remove();
	if (bus->function)
		mt6797_wlan_hif_function(bus->dev, false);
	mt6797_wlan_hif_release(bus->dev);
	mt6797_wmt_client_put(bus->wmt);
	kalBoostCpu(0);
	mt6797_nvram_release(&bus->nvram);
	platform_set_drvdata(pdev, NULL);
}

int mt6797_wlan_restart(void)
{
	struct mt6797_ahb *bus;
	struct device *dev;
	int ret;

	mutex_lock(&current_lock);
	dev = get_device(current_device);
	mutex_unlock(&current_lock);
	if (!dev)
		return -ENODEV;
	device_lock(dev);
	bus = dev_get_drvdata(dev);
	if (!bus) {
		ret = -ENODEV;
		goto out;
	}
	if (bus->glue)
		wlan_remove();
	if (bus->function) {
		ret = mt6797_wlan_hif_function(dev, false);
		if (ret)
			goto out;
		bus->function = false;
	}
	ret = mt6797_wmt_client_close(bus->wmt);
	if (ret)
		goto out;
	ret = mt6797_wmt_client_open(bus->wmt);
	if (ret)
		goto out;
	ret = mt6797_wlan_hif_function(dev, true);
	if (ret)
		goto close;
	bus->function = true;
	WRITE_ONCE(fgIsResetting, FALSE);
	if (wlan_probe(dev) == WLAN_STATUS_SUCCESS) {
		ret = 0;
		goto out;
	}
	mt6797_wlan_hif_function(dev, false);
	bus->function = false;
	ret = -EIO;
close:
	mt6797_wmt_client_close(bus->wmt);
out:
	device_unlock(dev);
	put_device(dev);
	return ret;
}

static const struct of_device_id mt6797_ahb_match[] = {
	{ .compatible = "mediatek,mt6797-wlan" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6797_ahb_match);
static struct platform_driver mt6797_ahb_driver = {
	.probe = mt6797_ahb_probe,
	.remove = mt6797_ahb_remove,
	.driver = {
		.name = "mt6797-wlan",
		.of_match_table = mt6797_ahb_match,
	},
};

WLAN_STATUS glRegisterBus(probe_card probe, remove_card remove)
{
	wlan_probe = probe;
	wlan_remove = remove;
	return platform_driver_register(&mt6797_ahb_driver) ? WLAN_STATUS_FAILURE : WLAN_STATUS_SUCCESS;
}

VOID glUnregisterBus(remove_card remove)
{
	platform_driver_unregister(&mt6797_ahb_driver);
}
