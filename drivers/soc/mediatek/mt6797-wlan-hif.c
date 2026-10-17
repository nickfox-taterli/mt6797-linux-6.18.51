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
#include <linux/iopoll.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/unaligned.h>

#include <linux/mt6797-wlan-hif.h>

#define MT6797_WLAN_HIF_LENGTH		0x1100
#define MT6797_WLAN_HIF_CMD_SETUP	0x0000
#define MT6797_WLAN_HIF_CMD53_DATA	0x1000
#define MT6797_WLAN_HIF_CMD52_DATA	0x0004
#define MT6797_WLAN_HIF_IRQ_MASK		0x0200

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
	bool irq_tearing_down;
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
		else if (!device_link_add(client, hif->dev, DL_FLAG_AUTOREMOVE_CONSUMER))
			ret = -ENOMEM;
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

static u8 mt6797_wlan_hif_cccr_read(struct mt6797_wlan_hif *hif, u32 address)
{
	writel(address << CMD53_ADDR_SHIFT, hif->base + MT6797_WLAN_HIF_CMD_SETUP);
	return readl(hif->base + MT6797_WLAN_HIF_CMD52_DATA);
}

static void mt6797_wlan_hif_cccr_write(struct mt6797_wlan_hif *hif,
				     u32 address, u8 value)
{
	writel(BIT(31) | (address << CMD53_ADDR_SHIFT) | value,
	       hif->base + MT6797_WLAN_HIF_CMD_SETUP);
	writel(value, hif->base + MT6797_WLAN_HIF_CMD52_DATA);
}

int mt6797_wlan_hif_function(struct device *client, bool enabled)
{
	struct mt6797_wlan_hif *hif;
	u8 value;
	int ret;

	ret = mt6797_wlan_hif_lock_client(client, &hif);
	if (ret)
		return ret;
	/* Keep the AHB interrupt masked while CCCR/function state changes. */
	writel(readl(hif->base + MT6797_WLAN_HIF_IRQ_MASK) | BIT(0),
	       hif->base + MT6797_WLAN_HIF_IRQ_MASK);
	value = mt6797_wlan_hif_cccr_read(hif, 0x02);
	mt6797_wlan_hif_cccr_write(hif, 0x02,
				    enabled ? value | BIT(1) : value & ~BIT(1));
	if (enabled) {
		ret = read_poll_timeout(mt6797_wlan_hif_cccr_read, value,
					value & BIT(1), 100, 100000, false, hif, 0x03);
		if (ret) {
			value = mt6797_wlan_hif_cccr_read(hif, 0x02);
			mt6797_wlan_hif_cccr_write(hif, 0x02, value & ~BIT(1));
			goto out;
		}
		/* Function 1 FBR block size = 512, matching the BSP HIF. */
		mt6797_wlan_hif_cccr_write(hif, 0x110, 0);
		mt6797_wlan_hif_cccr_write(hif, 0x111, 2);
	}
	value = mt6797_wlan_hif_cccr_read(hif, 0x04);
	if (enabled)
		value |= BIT(0) | BIT(1);
	else {
		value &= ~BIT(1);
		if (!(value & ~BIT(0)))
			value = 0;
	}
	mt6797_wlan_hif_cccr_write(hif, 0x04, value);
out:
	mutex_unlock(&hif->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mt6797_wlan_hif_function);

int mt6797_wlan_hif_transfer(struct device *client, u32 port, bool write,
			     void *buffer, size_t length)
{
	struct mt6797_wlan_hif *hif;
	size_t count, position, valid;
	u32 command, value;
	u8 tail[4];
	int ret;

	if (!buffer || !length || port > CMD53_ADDR_MASK || (port & 3) ||
	    length > 511 * 512)
		return -EINVAL;
	count = ALIGN(length, 4);
	command = (port << CMD53_ADDR_SHIFT) | (CMD53_FUNC_WIFI << CMD53_FUNC_SHIFT);
	if (count >= 512) {
		count = ALIGN(count, 512);
		command |= BIT(27) | (count / 512);
	} else {
		command |= count;
	}
	if (write)
		command |= BIT(31);
	ret = mt6797_wlan_hif_lock_client(client, &hif);
	if (ret)
		return ret;
	writel(command, hif->base + MT6797_WLAN_HIF_CMD_SETUP);
	for (position = 0; position < count; position += sizeof(value)) {
		valid = position < length ? min(sizeof(value), length - position) : 0;
		if (write) {
			memset(tail, 0, sizeof(tail));
			if (valid)
				memcpy(tail, (u8 *)buffer + position, valid);
			writel(get_unaligned_le32(tail), hif->base + MT6797_WLAN_HIF_CMD53_DATA);
		} else {
			value = readl(hif->base + MT6797_WLAN_HIF_CMD53_DATA);
			put_unaligned_le32(value, tail);
			if (valid)
				memcpy((u8 *)buffer + position, tail, valid);
		}
	}
	mutex_unlock(&hif->lock);
	return 0;
}
EXPORT_SYMBOL_GPL(mt6797_wlan_hif_transfer);

int mt6797_wlan_hif_mask_irq(struct device *client, bool masked)
{
	struct mt6797_wlan_hif *hif;
	u32 value;
	int ret;

	ret = mt6797_wlan_hif_lock_client(client, &hif);
	if (ret)
		return ret;
	value = readl(hif->base + MT6797_WLAN_HIF_IRQ_MASK);
	writel(masked ? value | BIT(0) : value & ~BIT(0),
	       hif->base + MT6797_WLAN_HIF_IRQ_MASK);
	mutex_unlock(&hif->lock);
	return 0;
}
EXPORT_SYMBOL_GPL(mt6797_wlan_hif_mask_irq);

int mt6797_wlan_hif_request_threaded_irq(struct device *client,
			irq_handler_t thread, const char *name, void *data)
{
	struct mt6797_wlan_hif *hif;
	int ret;

	if (!thread)
		return -EINVAL;
	ret = mt6797_wlan_hif_lock_client(client, &hif);
	if (ret)
		return ret;
	if (hif->irq_requested)
		ret = -EBUSY;
	else {
		ret = request_threaded_irq(hif->irq, NULL, thread,
					   IRQF_ONESHOT | IRQF_NO_AUTOEN, name, data);
		if (!ret) {
			hif->irq_requested = true;
			hif->irq_enabled = false;
			hif->irq_handler = thread;
			hif->irq_data = data;
		}
	}
	mutex_unlock(&hif->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mt6797_wlan_hif_request_threaded_irq);

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
	else if (hif->irq_tearing_down)
		ret = -EBUSY;
	else if (data != hif->irq_data)
		ret = -EPERM;
	else {
		hif->irq_tearing_down = true;
		mutex_unlock(&hif->lock);
		/* The IRQ thread may need the command lock while being drained. */
		free_irq(hif->irq, hif->irq_data);
		mutex_lock(&hif->lock);
		hif->irq_requested = false;
		hif->irq_enabled = false;
		hif->irq_tearing_down = false;
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
	else if (hif->irq_tearing_down)
		ret = -EBUSY;
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
	else if (hif->irq_tearing_down)
		ret = -EBUSY;
	else if (hif->irq_enabled) {
		hif->irq_tearing_down = true;
		hif->irq_enabled = false;
		mutex_unlock(&hif->lock);
		disable_irq(hif->irq);
		mutex_lock(&hif->lock);
		hif->irq_tearing_down = false;
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
