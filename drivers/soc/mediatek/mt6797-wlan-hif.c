// SPDX-License-Identifier: GPL-2.0-only
//
// MT6797 WLAN AHB sdioLike PIO host window.
//
// The vendor block is an indirect SDIO-like command port.  It must not be
// accessed merely because a platform device exists: the ConSys/WMT WLAN
// function has to be on before the HIF is claimed.  This first slice maps the
// resource, records the DT IRQ, and provides an owner-gated CMD53 PIO API.
// AP-DMA uses the dedicated BSP HIF channel; register accesses remain PIO.

#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
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

#define HIF_DMA_SIZE			(512 * 512)
#define HIF_DMA_FLAG			0x00
#define HIF_DMA_INT_EN			0x04
#define HIF_DMA_EN			0x08
#define HIF_DMA_RST			0x0c
#define HIF_DMA_CON			0x18
#define HIF_DMA_SRC			0x1c
#define HIF_DMA_DST			0x20
#define HIF_DMA_LEN			0x24
#define HIF_DMA_SRC2			0x54
#define HIF_DMA_DST2			0x58

struct mt6797_wlan_hif {
	struct device *dev;
	void __iomem *base;
	resource_size_t size;
	resource_size_t data_phys;
	void __iomem *dma_base;
	struct clk *dma_clk;
	void *dma_buffer;
	dma_addr_t dma_address;
	bool dma_enabled;
	bool dma_clocked;
	bool dma_fault;
	u64 dma_transfers;
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

/* Caller holds hif->lock and the channel clock.  Never reuse a timed-out
 * channel or free its buffer until reset has made the engine inactive.
 */
static int mt6797_wlan_hif_dma_reset(struct mt6797_wlan_hif *hif)
{
	u32 value;
	int ret;

	writel(0, hif->dma_base + HIF_DMA_INT_EN);
	writel(BIT(0), hif->dma_base + HIF_DMA_RST);
	ret = readl_poll_timeout(hif->dma_base + HIF_DMA_EN, value,
				 !(value & BIT(0)), 10, 10000);
	if (ret) {
		writel(BIT(1), hif->dma_base + HIF_DMA_RST);
		udelay(1000);
	}
	writel(0, hif->dma_base + HIF_DMA_RST);
	ret = readl_poll_timeout(hif->dma_base + HIF_DMA_EN, value,
				 !(value & BIT(0)), 10, 10000);
	/* This channel clears its interrupt by writing zero, as in the BSP. */
	writel(0, hif->dma_base + HIF_DMA_FLAG);
	return ret;
}

static int mt6797_wlan_hif_dma_transfer(struct mt6797_wlan_hif *hif,
				      bool write, void *buffer,
				      size_t length, size_t count)
{
	u32 value, con;
	int ret, reset_ret;

	if (hif->dma_fault || !hif->dma_clocked)
		return -EIO;
	if (write) {
		memcpy(hif->dma_buffer, buffer, length);
		memset(hif->dma_buffer + length, 0, count - length);
	}
	/* MT6797 HIF channel: incrementing bus, 4/4 burst, TX=0 / RX=1. */
	con = readl(hif->dma_base + HIF_DMA_CON);
	con &= ~(GENMASK(17, 16) | BIT(1) | BIT(0));
	con |= BIT(31) | (3 << 16) | !write;
	writel(con, hif->dma_base + HIF_DMA_CON);
	writel(write ? lower_32_bits(hif->dma_address) : hif->data_phys,
	       hif->dma_base + HIF_DMA_SRC);
	writel(write ? hif->data_phys : lower_32_bits(hif->dma_address),
	       hif->dma_base + HIF_DMA_DST);
	writel(count, hif->dma_base + HIF_DMA_LEN);
	/* Register Table Part 1, pp.418-419: bit 0 is address bit 32 on
	 * both memory and FIFO sides. The BSP's forced 1 assumes its 4G
	 * address layout; this driver's DMA API and HIF resource are 32-bit.
	 */
	writel(write ? upper_32_bits(hif->dma_address) : upper_32_bits(hif->data_phys),
	       hif->dma_base + HIF_DMA_SRC2);
	writel(write ? upper_32_bits(hif->data_phys) : upper_32_bits(hif->dma_address),
	       hif->dma_base + HIF_DMA_DST2);
	writel(0, hif->dma_base + HIF_DMA_FLAG);
	dma_wmb();
	writel(BIT(0), hif->dma_base + HIF_DMA_INT_EN);
	writel(BIT(0), hif->dma_base + HIF_DMA_EN);
	/* Normal FIFO transfers finish in microseconds. Without high-resolution
	 * timers even a 10-us sleep costs a scheduler tick per transfer. Keep
	 * that fast path bounded and retain sleeping polls for a slow engine.
	 */
	ret = readl_poll_timeout_atomic(hif->dma_base + HIF_DMA_FLAG, value,
					value & BIT(0), 1, 2000);
	if (ret)
		ret = readl_poll_timeout(hif->dma_base + HIF_DMA_FLAG, value,
					 value & BIT(0), 10, 100000);
	if (!ret) {
		ret = readl_poll_timeout_atomic(hif->dma_base + HIF_DMA_EN, value,
						!(value & BIT(0)), 1, 100);
		if (ret)
			ret = readl_poll_timeout(hif->dma_base + HIF_DMA_EN, value,
						 !(value & BIT(0)), 10, 10000);
	}
	writel(0, hif->dma_base + HIF_DMA_INT_EN);
	writel(0, hif->dma_base + HIF_DMA_FLAG);
	if (ret) {
		/* Fail the owning WLAN session, even if reset recovered the engine.
		 * Replaying a partly delivered CMD53 would corrupt the firmware stream.
		 */
		hif->dma_fault = true;
		reset_ret = mt6797_wlan_hif_dma_reset(hif);
		dev_err(hif->dev, "HIF DMA %s timeout, reset=%d\n",
			write ? "TX" : "RX", reset_ret);
		return ret;
	}
	dma_rmb();
	if (!write)
		memcpy(buffer, hif->dma_buffer, length);
	hif->dma_transfers++;
	return 0;
}

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
			if (hif->dma_enabled) {
				if (!hif->dma_clocked) {
					ret = clk_prepare_enable(hif->dma_clk);
					if (ret)
						goto unlock;
					hif->dma_clocked = true;
				}
				ret = mt6797_wlan_hif_dma_reset(hif);
				if (ret) {
					hif->dma_fault = true;
					goto unlock;
				}
				hif->dma_fault = false;
			}
			if (!device_link_add(client, hif->dev, DL_FLAG_AUTOREMOVE_CONSUMER)) {
				ret = -ENOMEM;
				if (hif->dma_clocked) {
					clk_disable_unprepare(hif->dma_clk);
					hif->dma_clocked = false;
				}
			} else {
				hif->owner = client;
				hif->claimed = true;
				dev_info(hif->dev, "WLAN HIF claimed by %s; ports use %s\n",
					 dev_name(client), hif->dma_enabled ? "AP-DMA" : "PIO");
			}
		}
unlock:
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
			if (hif->dma_clocked && !mt6797_wlan_hif_dma_reset(hif)) {
				clk_disable_unprepare(hif->dma_clk);
				hif->dma_clocked = false;
				hif->dma_fault = false;
			}
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
	bool dma;
	int ret;

	if (!buffer || !length || port > CMD53_ADDR_MASK || (port & 3) ||
	    length > 511 * 512)
		return -EINVAL;
	ret = mt6797_wlan_hif_lock_client(client, &hif);
	if (ret)
		return ret;
	if (hif->dma_fault) {
		ret = -EIO;
		goto out;
	}
	dma = hif->dma_enabled && ((write && port == 0x34) ||
				 (!write && (port == 0x50 || port == 0x54)));
	count = ALIGN(length, 4);
	command = (port << CMD53_ADDR_SHIFT) | (CMD53_FUNC_WIFI << CMD53_FUNC_SHIFT);
	if (count >= 512) {
		count = ALIGN(count, 512);
		command |= BIT(27) | (count / 512);
	} else {
		/* BSP RX byte-mode DMA requires 8-byte FIFO reads. A byte
		 * count of 512 is encoded as zero in CMD53's 9-bit count.
		 */
		if (dma && !write)
			count = ALIGN(count, 8);
		command |= count & GENMASK(8, 0);
	}
	if (write)
		command |= BIT(31);
	writel(command, hif->base + MT6797_WLAN_HIF_CMD_SETUP);
	/* Match the BSP: only WLAN packet FIFO ports use AP-DMA. */
	if (dma) {
		ret = mt6797_wlan_hif_dma_transfer(hif, write, buffer, length, count);
		goto out;
	}
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
out:
	mutex_unlock(&hif->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mt6797_wlan_hif_transfer);

static ssize_t transfer_mode_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct mt6797_wlan_hif *hif = dev_get_drvdata(dev);
	ssize_t length;

	mutex_lock(&hif->lock);
	length = sysfs_emit(buf, "%s transfers=%llu fault=%u\n",
			hif->dma_enabled ? "dma" : "pio", hif->dma_transfers,
			hif->dma_fault);
	mutex_unlock(&hif->lock);
	return length;
}

static ssize_t transfer_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct mt6797_wlan_hif *hif = dev_get_drvdata(dev);
	bool enabled;
	int ret = 0;

	if (sysfs_streq(buf, "dma"))
		enabled = true;
	else if (sysfs_streq(buf, "pio"))
		enabled = false;
	else
		return -EINVAL;
	mutex_lock(&hif->lock);
	if (hif->claimed || hif->dma_clocked)
		ret = -EBUSY;
	else if (enabled && !hif->dma_buffer)
		ret = -EOPNOTSUPP;
	else
		hif->dma_enabled = enabled;
	mutex_unlock(&hif->lock);
	return ret ?: count;
}
static DEVICE_ATTR_RW(transfer_mode);

static struct attribute *mt6797_wlan_hif_attrs[] = {
	&dev_attr_transfer_mode.attr,
	NULL,
};
ATTRIBUTE_GROUPS(mt6797_wlan_hif);

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
	hif->data_phys = resource->start + MT6797_WLAN_HIF_CMD53_DATA;
	hif->base = devm_ioremap_resource(&pdev->dev, resource);
	if (IS_ERR(hif->base))
		return PTR_ERR(hif->base);
	hif->irq = platform_get_irq(pdev, 0);
	if (hif->irq < 0)
		return hif->irq;
	resource = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (resource) {
		if (resource_size(resource) < 0x80 || hif->data_phys > U32_MAX)
			return -EINVAL;
		hif->dma_base = devm_ioremap_resource(&pdev->dev, resource);
		if (IS_ERR(hif->dma_base))
			return PTR_ERR(hif->dma_base);
		hif->dma_clk = devm_clk_get(&pdev->dev, "wifi-dma");
		if (IS_ERR(hif->dma_clk))
			return PTR_ERR(hif->dma_clk);
		ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
		if (ret)
			return ret;
		/* Keep one buffer for the channel lifetime, including fault teardown. */
		hif->dma_buffer = dma_alloc_coherent(&pdev->dev, HIF_DMA_SIZE,
						    &hif->dma_address, GFP_KERNEL);
		if (!hif->dma_buffer)
			return -ENOMEM;
	}
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
	if (ret) {
		if (hif->dma_buffer)
			dma_free_coherent(hif->dev, HIF_DMA_SIZE, hif->dma_buffer,
					  hif->dma_address);
		return dev_err_probe(&pdev->dev, ret, "only one WLAN HIF is supported\n");
	}

	dev_info(&pdev->dev,
		 "WLAN HIF mapped (data=%pa, size=0x%llx), irq=%d; dormant until WMT function-on\n",
		 &hif->data_phys,
		 (unsigned long long)hif->size, hif->irq);
	return 0;
}

static void mt6797_wlan_hif_remove(struct platform_device *pdev)
{
	struct mt6797_wlan_hif *hif = platform_get_drvdata(pdev);

	if (!hif)
		return;
	mutex_lock(&mt6797_wlan_hif_singleton_lock);
	mutex_lock(&hif->lock);
	if (mt6797_wlan_hif_singleton == hif)
		mt6797_wlan_hif_singleton = NULL;
	if (hif->claimed)
		dev_warn(hif->dev, "removing WLAN HIF while owner is still claimed\n");
	hif->owner = NULL;
	hif->claimed = false;
	mutex_unlock(&hif->lock);
	mutex_unlock(&mt6797_wlan_hif_singleton_lock);
	/* No new client can acquire this provider.  Drain the thread without
	 * either API lock, since it may itself be completing a HIF access.
	 */
	if (hif->irq_requested)
		free_irq(hif->irq, hif->irq_data);
	if (hif->dma_clocked && !mt6797_wlan_hif_dma_reset(hif)) {
		clk_disable_unprepare(hif->dma_clk);
		hif->dma_clocked = false;
	}
	if (hif->dma_buffer) {
		if (hif->dma_clocked)
			dev_err(hif->dev, "DMA reset failed; retaining active channel buffer\n");
		else
			dma_free_coherent(hif->dev, HIF_DMA_SIZE, hif->dma_buffer,
					  hif->dma_address);
	}
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
		.dev_groups = mt6797_wlan_hif_groups,
	},
};
module_platform_driver(mt6797_wlan_hif_driver);

MODULE_DESCRIPTION("MT6797 WLAN AHB sdioLike PIO HIF owner gate");
MODULE_LICENSE("GPL");
