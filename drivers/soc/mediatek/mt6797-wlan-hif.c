// SPDX-License-Identifier: GPL-2.0-only
//
// MT6797 WLAN AHB sdioLike PIO host window.
//
// The vendor block is an indirect SDIO-like command port.  It must not be
// accessed merely because a platform device exists: the ConSys/WMT WLAN
// function has to be on before the HIF is claimed.  This first slice maps the
// resource, records the DT IRQ, and provides an owner-gated CMD53 PIO API.
// AP-DMA, remap programming, firmware download, and cfg80211 are intentionally
// outside this file.

#include <linux/errno.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include <linux/mt6797-wlan-hif.h>

#define MT6797_WLAN_HIF_LENGTH		0x1100
#define MT6797_WLAN_HIF_CMD_SETUP	0x0000
#define MT6797_WLAN_HIF_CMD53_DATA	0x1000

/* CMD53 descriptor: count[8:0], addr[25:9], op[26], block[27], func[30:28], rw[31]. */
#define CMD53_ADDR_MASK			GENMASK(16, 0)
#define CMD53_ADDR_SHIFT		9
#define CMD53_FUNC_WIFI			1
#define CMD53_FUNC_SHIFT		28
#define CMD53_RW_SHIFT			31
#define CMD53_COUNT			4

struct mt6797_wlan_hif {
	struct device *dev;
	void __iomem *base;
	resource_size_t size;
	int irq;
	struct mutex lock;
	struct device *owner;
	bool claimed;
	bool irq_requested;
	bool irq_enabled;
	irq_handler_t irq_handler;
	void *irq_data;
};

static struct mt6797_wlan_hif *mt6797_wlan_hif_singleton;
static DEFINE_MUTEX(mt6797_wlan_hif_singleton_lock);

/* Return with the HIF lock held.  The singleton lock prevents remove from
 * freeing the devm mapping while an API call is using it. */
static int mt6797_wlan_hif_lock_client(struct device *client,
					struct mt6797_wlan_hif **out)
{
	struct mt6797_wlan_hif *hif;

	if (!client || !out)
		return -EINVAL;

	mutex_lock(&mt6797_wlan_hif_singleton_lock);
	hif = mt6797_wlan_hif_singleton;
	if (!hif) {
		mutex_unlock(&mt6797_wlan_hif_singleton_lock);
		return -ENODEV;
	}
	mutex_lock(&hif->lock);
	mutex_unlock(&mt6797_wlan_hif_singleton_lock);

	if (!hif->claimed || hif->owner != client) {
		mutex_unlock(&hif->lock);
		return -EPERM;
	}
	*out = hif;
	return 0;
}

static u32 mt6797_wlan_hif_cmd53(bool write, u32 address)
{
	u32 command;

	command = CMD53_COUNT;
	command |= (address & CMD53_ADDR_MASK) << CMD53_ADDR_SHIFT;
	command |= CMD53_FUNC_WIFI << CMD53_FUNC_SHIFT;
	if (write)
		command |= BIT(CMD53_RW_SHIFT);
	return command;
}

int mt6797_wlan_hif_claim(struct device *client)
{
	struct mt6797_wlan_hif *hif;
	int ret = 0;

	if (!client)
		return -EINVAL;
	mutex_lock(&mt6797_wlan_hif_singleton_lock);
	hif = mt6797_wlan_hif_singleton;
	if (!hif)
		ret = -ENODEV;
	else {
		mutex_lock(&hif->lock);
		if (hif->claimed)
			ret = hif->owner == client ? -EALREADY : -EBUSY;
		else {
			hif->owner = client;
			hif->claimed = true;
			dev_info(hif->dev,
				 "WLAN HIF claimed by %s; PIO access is now permitted\n",
				 dev_name(client));
		}
		mutex_unlock(&hif->lock);
	}
	mutex_unlock(&mt6797_wlan_hif_singleton_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mt6797_wlan_hif_claim);

int mt6797_wlan_hif_release(struct device *client)
{
	struct mt6797_wlan_hif *hif;
	int ret = 0;

	if (!client)
		return -EINVAL;
	mutex_lock(&mt6797_wlan_hif_singleton_lock);
	hif = mt6797_wlan_hif_singleton;
	if (!hif)
		ret = -ENODEV;
	else {
		mutex_lock(&hif->lock);
		if (!hif->claimed || hif->owner != client)
			ret = -EPERM;
		else if (hif->irq_requested)
			ret = -EBUSY;
		else {
			hif->owner = NULL;
			hif->claimed = false;
			dev_info(hif->dev, "WLAN HIF released\n");
		}
		mutex_unlock(&hif->lock);
	}
	mutex_unlock(&mt6797_wlan_hif_singleton_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mt6797_wlan_hif_release);

int mt6797_wlan_hif_pio_read(struct device *client, u32 address, u32 *value)
{
	struct mt6797_wlan_hif *hif;
	int ret;

	if (!value || (address & 3) || address > CMD53_ADDR_MASK)
		return -EINVAL;
	ret = mt6797_wlan_hif_lock_client(client, &hif);
	if (ret)
		return ret;

	/* The function register address is indirect; never use base + address. */
	writel(mt6797_wlan_hif_cmd53(false, address),
	       hif->base + MT6797_WLAN_HIF_CMD_SETUP);
	*value = readl(hif->base + MT6797_WLAN_HIF_CMD53_DATA);
	mutex_unlock(&hif->lock);
	return 0;
}
EXPORT_SYMBOL_GPL(mt6797_wlan_hif_pio_read);

int mt6797_wlan_hif_pio_write(struct device *client, u32 address, u32 value)
{
	struct mt6797_wlan_hif *hif;
	int ret;

	if ((address & 3) || address > CMD53_ADDR_MASK)
		return -EINVAL;
	ret = mt6797_wlan_hif_lock_client(client, &hif);
	if (ret)
		return ret;

	writel(mt6797_wlan_hif_cmd53(true, address),
	       hif->base + MT6797_WLAN_HIF_CMD_SETUP);
	writel(value, hif->base + MT6797_WLAN_HIF_CMD53_DATA);
	mutex_unlock(&hif->lock);
	return 0;
}
EXPORT_SYMBOL_GPL(mt6797_wlan_hif_pio_write);

int mt6797_wlan_hif_wcir_read(struct device *client, u32 *value)
{
	return mt6797_wlan_hif_pio_read(client, MT6797_WLAN_HIF_WCIR, value);
}
EXPORT_SYMBOL_GPL(mt6797_wlan_hif_wcir_read);

int mt6797_wlan_hif_request_irq(struct device *client, irq_handler_t handler,
				unsigned long flags, const char *name, void *data)
{
	struct mt6797_wlan_hif *hif;
	int ret;

	if (!handler)
		return -EINVAL;
	ret = mt6797_wlan_hif_lock_client(client, &hif);
	if (ret)
		return ret;
	if (hif->irq_requested)
		ret = -EBUSY;
	else {
		/* The DT interrupt type is authoritative; callers pass only sharing
		 * or other request flags.  Keep the line disabled until the owner has
		 * finished its HIF/firmware initialization. */
		ret = request_irq(hif->irq, handler, flags | IRQF_NO_AUTOEN,
				  name ?: dev_name(hif->dev), data);
		if (!ret) {
			hif->irq_requested = true;
			hif->irq_enabled = false;
			hif->irq_handler = handler;
			hif->irq_data = data;
		}
	}
	mutex_unlock(&hif->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mt6797_wlan_hif_request_irq);

int mt6797_wlan_hif_free_irq(struct device *client, void *data)
{
	struct mt6797_wlan_hif *hif;
	int ret;

	ret = mt6797_wlan_hif_lock_client(client, &hif);
	if (ret)
		return ret;
	if (!hif->irq_requested)
		ret = -EINVAL;
	else if (data != hif->irq_data)
		ret = -EPERM;
	else {
		if (hif->irq_enabled) {
			disable_irq(hif->irq);
			hif->irq_enabled = false;
		}
		free_irq(hif->irq, hif->irq_data);
		hif->irq_requested = false;
		hif->irq_handler = NULL;
		hif->irq_data = NULL;
	}
	mutex_unlock(&hif->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mt6797_wlan_hif_free_irq);

int mt6797_wlan_hif_enable_irq(struct device *client)
{
	struct mt6797_wlan_hif *hif;
	int ret;

	ret = mt6797_wlan_hif_lock_client(client, &hif);
	if (ret)
		return ret;
	if (!hif->irq_requested)
		ret = -EINVAL;
	else if (!hif->irq_enabled) {
		enable_irq(hif->irq);
		hif->irq_enabled = true;
	}
	mutex_unlock(&hif->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mt6797_wlan_hif_enable_irq);

int mt6797_wlan_hif_disable_irq(struct device *client)
{
	struct mt6797_wlan_hif *hif;
	int ret;

	ret = mt6797_wlan_hif_lock_client(client, &hif);
	if (ret)
		return ret;
	if (!hif->irq_requested)
		ret = -EINVAL;
	else if (hif->irq_enabled) {
		disable_irq(hif->irq);
		hif->irq_enabled = false;
	}
	mutex_unlock(&hif->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mt6797_wlan_hif_disable_irq);

static int mt6797_wlan_hif_probe(struct platform_device *pdev)
{
	struct mt6797_wlan_hif *hif;
	struct resource *resource;
	int ret;

	resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!resource || resource_size(resource) < MT6797_WLAN_HIF_LENGTH)
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "HIF resource must be at least 0x%x bytes\n",
				     MT6797_WLAN_HIF_LENGTH);

	hif = devm_kzalloc(&pdev->dev, sizeof(*hif), GFP_KERNEL);
	if (!hif)
		return -ENOMEM;
	hif->dev = &pdev->dev;
	hif->size = resource_size(resource);
	hif->base = devm_ioremap_resource(&pdev->dev, resource);
	if (IS_ERR(hif->base))
		return PTR_ERR(hif->base);
	hif->irq = platform_get_irq(pdev, 0);
	if (hif->irq < 0)
		return hif->irq;
	mutex_init(&hif->lock);

	mutex_lock(&mt6797_wlan_hif_singleton_lock);
	if (mt6797_wlan_hif_singleton)
		ret = -EBUSY;
	else {
		mt6797_wlan_hif_singleton = hif;
		platform_set_drvdata(pdev, hif);
		ret = 0;
	}
	mutex_unlock(&mt6797_wlan_hif_singleton_lock);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "only one WLAN HIF is supported\n");

	dev_info(&pdev->dev,
		 "WLAN HIF mapped (%pa+0x%llx), irq=%d; dormant until WMT function-on\n",
		 &resource->start, (unsigned long long)hif->size, hif->irq);
	return 0;
}

static void mt6797_wlan_hif_remove(struct platform_device *pdev)
{
	struct mt6797_wlan_hif *hif = platform_get_drvdata(pdev);

	if (!hif)
		return;
	mutex_lock(&mt6797_wlan_hif_singleton_lock);
	mutex_lock(&hif->lock);
	if (hif->irq_requested) {
		if (hif->irq_enabled)
			disable_irq(hif->irq);
		free_irq(hif->irq, hif->irq_data);
		hif->irq_requested = false;
	}
	if (hif->claimed)
		dev_warn(hif->dev, "removing WLAN HIF while owner is still claimed\n");
	hif->owner = NULL;
	hif->claimed = false;
	mutex_unlock(&hif->lock);
	if (mt6797_wlan_hif_singleton == hif)
		mt6797_wlan_hif_singleton = NULL;
	mutex_unlock(&mt6797_wlan_hif_singleton_lock);
}

static const struct of_device_id mt6797_wlan_hif_of_match[] = {
	{ .compatible = "mediatek,mt6797-wlan-hif" },
	/* Temporary board bring-up alias for the generic-resource A/B node. */
	{ .compatible = "x20,diagnostic-hif" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6797_wlan_hif_of_match);

static struct platform_driver mt6797_wlan_hif_driver = {
	.probe = mt6797_wlan_hif_probe,
	.remove = mt6797_wlan_hif_remove,
	.driver = {
		.name = "mt6797-wlan-hif",
		.of_match_table = mt6797_wlan_hif_of_match,
	},
};
module_platform_driver(mt6797_wlan_hif_driver);

MODULE_DESCRIPTION("MT6797 WLAN AHB sdioLike PIO HIF owner gate");
MODULE_LICENSE("GPL");
