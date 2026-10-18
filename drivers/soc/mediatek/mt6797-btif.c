// SPDX-License-Identifier: GPL-2.0-only
//
// MT6797 BTIF PIO and dedicated DMA/VFIFO transport for ConSys STP/WMT.
//
// The block has UART-like FIFOs but is not a serial console.  This driver
// deliberately exposes only an in-kernel byte transport: STP framing,
// function ownership and HCI demultiplexing belong to its client.

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kfifo.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/wait.h>

#include <linux/mt6797-btif.h>
#include "mt6797-btif-ring.h"

static bool trace;
module_param(trace, bool, 0600);
MODULE_PARM_DESC(trace, "Log raw BTIF bytes and initial IRQ state");

#define BTIF_RBR_THR		0x00
#define BTIF_IER		0x04
#define BTIF_IIR_FIFOCTRL	0x08
#define BTIF_FAKELCR		0x0c
#define BTIF_LSR		0x14
#define BTIF_SLEEP_EN		0x48
#define BTIF_DMA_EN		0x4c
#define BTIF_TRI_LVL		0x60
#define BTIF_WAK		0x64
#define BTIF_HANDSHAKE		0x6c

#define BTIF_IER_RXFEN		BIT(0)
#define BTIF_IER_TXEEN		BIT(1)
#define BTIF_IIR_NINT		BIT(0)
#define BTIF_IIR_TX_EMPTY	BIT(1)
#define BTIF_IIR_RX		BIT(2)
#define BTIF_IIR_RX_TIMEOUT	(0x11 << 2)
#define BTIF_LSR_THRE		BIT(5)
#define BTIF_LSR_TEMT		BIT(6)
#define BTIF_FIFOCTRL_CLR_RX	BIT(1)
#define BTIF_FIFOCTRL_CLR_TX	BIT(2)
#define BTIF_DMA_AUTORST	BIT(2)
#define BTIF_HANDSHAKE_ENABLE	BIT(0)
#define BTIF_WAK_SIGNAL		BIT(0)
#define BTIF_TRI_TX_MASK		GENMASK(3, 0)
#define BTIF_TRI_RX_MASK		GENMASK(6, 4)
#define BTIF_TRI_TX_THRESHOLD	8
#define BTIF_TRI_RX_THRESHOLD	1
#define BTIF_TX_FIFO_SIZE	16
#define BTIF_RX_BUDGET		256
#define BTIF_TX_QUEUE_SIZE	4096

#define VFF_FLAG 0x00
#define VFF_INT_EN 0x04
#define VFF_EN 0x08
#define VFF_RST 0x0c
#define VFF_STOP 0x10
#define VFF_FLUSH 0x14
#define VFF_ADDR 0x1c
#define VFF_LEN 0x24
#define VFF_THRE 0x28
#define VFF_WPT 0x2c
#define VFF_RPT 0x30
#define VFF_VALID 0x3c
#define VFF_LEFT 0x40
#define VFF_ADDR2 0x54

struct btif_dma_channel {
	void __iomem *base;
	void *buffer;
	dma_addr_t address;
	int irq;
	u32 pointer;
	u64 bytes;
};

struct mt6797_btif {
	struct device *dev;
	void __iomem *base;
	struct clk *clk;
	int irq;
	struct clk *dma_clk;
	struct btif_dma_channel tx_dma, rx_dma;
	bool dma_mode, dma_clocked, dma_fault;

	struct mutex client_lock;
	struct device *owner;
	const struct mt6797_btif_client_ops *ops;
	void *context;
	bool running;
	unsigned int debug_irq_count;

	spinlock_t tx_lock;
	struct kfifo tx_fifo;
	wait_queue_head_t tx_wait;
};

static struct mt6797_btif *mt6797_btif_singleton;
static DEFINE_MUTEX(mt6797_btif_singleton_lock);

static void btif_kfifo_free(void *data)
{
	kfifo_free((struct kfifo *)data);
}

static void btif_update_bits(struct mt6797_btif *btif, unsigned int offset,
			     u32 mask, u32 value)
{
	u32 reg = readl(btif->base + offset);

	reg = (reg & ~mask) | (value & mask);
	writel(reg, btif->base + offset);
}

static void btif_fifo_reset(struct mt6797_btif *btif)
{
	/* IIR/FIFOCTRL share an offset; FIFOCTRL is write-only on this block. */
	writel(BTIF_FIFOCTRL_CLR_RX, btif->base + BTIF_IIR_FIFOCTRL);
	writel(0, btif->base + BTIF_IIR_FIFOCTRL);
	writel(BTIF_FIFOCTRL_CLR_TX, btif->base + BTIF_IIR_FIFOCTRL);
	writel(0, btif->base + BTIF_IIR_FIFOCTRL);
}

static int btif_dma_reset(struct btif_dma_channel *channel)
{
	u32 value;
	int ret;

	writel(0, channel->base + VFF_INT_EN);
	writel(BIT(0), channel->base + VFF_RST);
	ret = readl_poll_timeout(channel->base + VFF_EN, value,
				 !(value & 1), 10, 10000);
	if (ret) {
		writel(BIT(1), channel->base + VFF_RST);
		udelay(100);
	}
	writel(0, channel->base + VFF_RST);
	writel(0, channel->base + VFF_STOP);
	return readl_poll_timeout(channel->base + VFF_EN, value,
				  !(value & 1), 10, 10000);
}

static void btif_dma_free(void *data)
{
	struct mt6797_btif *btif = data;
	struct btif_dma_channel *channels[] = { &btif->tx_dma, &btif->rx_dma };
	int i;

	/* Never free memory while an engine could still write into it. */
	if (btif->dma_clocked) {
		dev_err(btif->dev, "retaining DMA buffers and clock after failed stop\n");
		return;
	}
	for (i = 0; i < ARRAY_SIZE(channels); i++)
		if (channels[i]->buffer)
			dma_free_coherent(btif->dev, BTIF_VFIFO_SIZE,
					  channels[i]->buffer, channels[i]->address);
}

static irqreturn_t btif_dma_rx_irq(int irq, void *data)
{
	struct mt6797_btif *btif = data;
	struct btif_dma_channel *channel = &btif->rx_dma;
	const struct mt6797_btif_client_ops *ops;
	u32 write, offset, first;
	int count, round;

	if (!READ_ONCE(btif->running))
		return IRQ_NONE;
	/* RX flags are W1C; the TX flag below is write-zero-to-clear. */
	writel(3, channel->base + VFF_FLAG);
	ops = READ_ONCE(btif->ops);
	for (round = 0; round < 32; round++) {
		write = readl(channel->base + VFF_WPT);
		count = btif_vfifo_pending(write, channel->pointer);
		if (count < 0) {
			WRITE_ONCE(btif->dma_fault, true);
			writel(0, channel->base + VFF_INT_EN);
			dev_err_ratelimited(btif->dev, "invalid RX DMA ring pointers %x/%x\n",
					    write, channel->pointer);
			break;
		}
		if (!count)
			break;
		dma_rmb();
		offset = channel->pointer & 0xffff;
		first = min_t(u32, count, BTIF_VFIFO_SIZE - offset);
		if (ops && ops->rx_bytes) {
			ops->rx_bytes(READ_ONCE(btif->context), channel->buffer + offset, first);
			if (count > first)
				ops->rx_bytes(READ_ONCE(btif->context), channel->buffer, count - first);
		}
		channel->pointer = btif_vfifo_advance(channel->pointer, count);
		channel->bytes += count;
		/* The callback has copied every byte before returning ownership. */
		dma_mb();
		writel(channel->pointer, channel->base + VFF_RPT);
	}
	return IRQ_HANDLED;
}

static irqreturn_t btif_dma_tx_irq(int irq, void *data)
{
	struct mt6797_btif *btif = data;
	struct btif_dma_channel *channel = &btif->tx_dma;
	unsigned long flags;
	u32 valid;

	if (!READ_ONCE(btif->running))
		return IRQ_NONE;
	spin_lock_irqsave(&btif->tx_lock, flags);
	writel(0, channel->base + VFF_INT_EN);
	writel(0, channel->base + VFF_FLAG);
	valid = readl(channel->base + VFF_VALID);
	if (valid && valid < 8 && !readl(channel->base + VFF_FLUSH))
		writel(1, channel->base + VFF_FLUSH);
	if (valid >= 8)
		writel(1, channel->base + VFF_INT_EN);
	spin_unlock_irqrestore(&btif->tx_lock, flags);
	wake_up_all(&btif->tx_wait);
	return IRQ_HANDLED;
}

static int btif_dma_start(struct mt6797_btif *btif)
{
	struct btif_dma_channel *channels[] = { &btif->tx_dma, &btif->rx_dma };
	struct btif_dma_channel *channel;
	int i, ret;

	if (!btif->dma_clocked) {
		ret = clk_prepare_enable(btif->dma_clk);
		if (ret)
			return ret;
		btif->dma_clocked = true;
	}
	for (i = 0; i < ARRAY_SIZE(channels); i++) {
		channel = channels[i];
		ret = btif_dma_reset(channel);
		if (ret) {
			btif->dma_fault = true;
			return ret;
		}
		memset(channel->buffer, 0, BTIF_VFIFO_SIZE);
		channel->pointer = 0;
		writel(lower_32_bits(channel->address), channel->base + VFF_ADDR);
		writel(upper_32_bits(channel->address), channel->base + VFF_ADDR2);
		writel(BTIF_VFIFO_SIZE, channel->base + VFF_LEN);
		writel(i ? BTIF_VFIFO_SIZE * 3 / 4 : BTIF_VFIFO_SIZE - 7,
		       channel->base + VFF_THRE);
		writel(0, channel->base + VFF_WPT);
		writel(0, channel->base + VFF_RPT);
		writel(i ? 3 : 0, channel->base + VFF_FLAG);
	}
	btif->dma_fault = false;
	return 0;
}

static int btif_dma_stop(struct mt6797_btif *btif)
{
	int tx, rx;

	btif_update_bits(btif, BTIF_DMA_EN, BIT(0) | BIT(1), 0);
	tx = btif_dma_reset(&btif->tx_dma);
	rx = btif_dma_reset(&btif->rx_dma);
	if (tx || rx) {
		btif->dma_fault = true;
		dev_err(btif->dev, "DMA stop/reset failed: tx=%d rx=%d\n", tx, rx);
		return tx ?: rx;
	}
	clk_disable_unprepare(btif->dma_clk);
	btif->dma_clocked = false;
	return 0;
}

/* The caller holds the singleton lock, keeping ownership and clocks stable. */
static ssize_t btif_dma_write(struct mt6797_btif *btif, const u8 *data,
			     size_t length, unsigned long timeout)
{
	struct btif_dma_channel *channel = &btif->tx_dma;
	unsigned long flags;
	size_t written = 0;
	u32 value, room, count, offset, first;
	long remaining;
	int ret;

	while (written < length) {
		if (READ_ONCE(btif->dma_fault))
			return -EIO;
		ret = readl_poll_timeout(channel->base + VFF_FLUSH, value,
					 !value, 10, 20000);
		if (ret) {
			btif->dma_fault = true;
			return ret;
		}
		spin_lock_irqsave(&btif->tx_lock, flags);
		if (readl(channel->base + VFF_FLUSH)) {
			spin_unlock_irqrestore(&btif->tx_lock, flags);
			continue;
		}
		room = readl(channel->base + VFF_LEFT);
		if (room > BTIF_VFIFO_SIZE) {
			spin_unlock_irqrestore(&btif->tx_lock, flags);
			btif->dma_fault = true;
			return -EPROTO;
		}
		count = min_t(size_t, length - written, room);
		if (count) {
			writel(0, channel->base + VFF_INT_EN);
			offset = channel->pointer & 0xffff;
			first = min(count, BTIF_VFIFO_SIZE - offset);
			memcpy(channel->buffer + offset, data + written, first);
			if (count > first)
				memcpy(channel->buffer, data + written + first, count - first);
			channel->pointer = btif_vfifo_advance(channel->pointer, count);
			dma_wmb();
			writel(1, channel->base + VFF_EN);
			writel(channel->pointer, channel->base + VFF_WPT);
			value = readl(channel->base + VFF_VALID);
			if (value && value < 8)
				writel(1, channel->base + VFF_FLUSH);
			writel(1, channel->base + VFF_INT_EN);
			channel->bytes += count;
			written += count;
		}
		spin_unlock_irqrestore(&btif->tx_lock, flags);
		if (written == length || !timeout)
			break;
		remaining = wait_event_interruptible_timeout(btif->tx_wait,
			readl(channel->base + VFF_LEFT) > 0, timeout);
		if (remaining <= 0)
			return remaining ? -ERESTARTSYS : -ETIMEDOUT;
		timeout = remaining;
	}
	return written;
}

static void btif_debug_state(struct mt6797_btif *btif, const char *where)
{
	if (!trace)
		return;
	dev_info(btif->dev,
		 "BTIF debug %s: ier=%08x iir=%08x lsr=%08x tri=%08x dma=%08x handshake=%08x wak=%08x clk=%lu\n",
		 where, readl(btif->base + BTIF_IER),
		 readl(btif->base + BTIF_IIR_FIFOCTRL),
		 readl(btif->base + BTIF_LSR), readl(btif->base + BTIF_TRI_LVL),
		 readl(btif->base + BTIF_DMA_EN),
		 readl(btif->base + BTIF_HANDSHAKE),
		 readl(btif->base + BTIF_WAK), clk_get_rate(btif->clk));
}

static void btif_hw_init(struct mt6797_btif *btif)
{
	u32 tri;

	writel(0, btif->base + BTIF_FAKELCR);
	btif_update_bits(btif, BTIF_HANDSHAKE, BTIF_HANDSHAKE_ENABLE,
			 BTIF_HANDSHAKE_ENABLE);
	btif_fifo_reset(btif);
	tri = readl(btif->base + BTIF_TRI_LVL);
	tri &= ~(BTIF_TRI_TX_MASK | BTIF_TRI_RX_MASK);
	tri |= BTIF_TRI_TX_THRESHOLD | (BTIF_TRI_RX_THRESHOLD << 4);
	writel(tri, btif->base + BTIF_TRI_LVL);
	btif_update_bits(btif, BTIF_DMA_EN, BTIF_DMA_AUTORST, BTIF_DMA_AUTORST);
	btif_update_bits(btif, BTIF_DMA_EN, BIT(0) | BIT(1), 0);
	/* Vendor PIO init leaves RX interrupts enabled; TX is armed per write. */
	btif_update_bits(btif, BTIF_IER, BTIF_IER_RXFEN | BTIF_IER_TXEEN,
			 BTIF_IER_RXFEN);
}

static irqreturn_t mt6797_btif_irq(int irq, void *data)
{
	struct mt6797_btif *btif = data;
	const struct mt6797_btif_client_ops *ops;
	unsigned long flags;
	u8 rx[BTIF_RX_BUDGET];
	unsigned int rx_len = 0;
	u32 iir;

	/* Release sets running false before disable_irq() synchronizes this IRQ. */
	if (!READ_ONCE(btif->running))
		return IRQ_NONE;
	ops = READ_ONCE(btif->ops);

	iir = readl(btif->base + BTIF_IIR_FIFOCTRL);
	if (trace && btif->debug_irq_count < 32) {
		btif->debug_irq_count++;
		dev_info(btif->dev,
			 "BTIF debug irq#%u: iir=%08x ier=%08x lsr=%08x\n",
			 btif->debug_irq_count, iir,
			 readl(btif->base + BTIF_IER), readl(btif->base + BTIF_LSR));
	}
	while ((iir & (BTIF_IIR_RX | BTIF_IIR_RX_TIMEOUT)) &&
	       rx_len < ARRAY_SIZE(rx)) {
		rx[rx_len++] = readb(btif->base + BTIF_RBR_THR);
		iir = readl(btif->base + BTIF_IIR_FIFOCTRL);
	}

	if (iir & BTIF_IIR_TX_EMPTY) {
		u8 tx[BTIF_TX_FIFO_SIZE];
		unsigned int count;
		unsigned int index;
		u32 lsr = readl(btif->base + BTIF_LSR);
		unsigned int room = lsr & BTIF_LSR_TEMT ? BTIF_TX_FIFO_SIZE :
			(lsr & BTIF_LSR_THRE ? BTIF_TX_FIFO_SIZE - BTIF_TRI_TX_THRESHOLD : 0);

		spin_lock_irqsave(&btif->tx_lock, flags);
		count = min_t(unsigned int, room, kfifo_out_peek(&btif->tx_fifo,
							 tx, sizeof(tx)));
		if (count)
			kfifo_skip_count(&btif->tx_fifo, count);
		if (kfifo_is_empty(&btif->tx_fifo))
			btif_update_bits(btif, BTIF_IER, BTIF_IER_TXEEN, 0);
		spin_unlock_irqrestore(&btif->tx_lock, flags);

		for (index = 0; index < count; index++)
			writeb(tx[index], btif->base + BTIF_RBR_THR);
		wake_up_all(&btif->tx_wait);
	}

	if (trace && rx_len && ops && ops->rx_bytes)
		dev_info(btif->dev, "BTIF debug rx len=%u data=%*phN\n", rx_len,
			 rx_len, rx);
	if (rx_len && ops && ops->rx_bytes)
		ops->rx_bytes(READ_ONCE(btif->context), rx, rx_len);
	return (rx_len || (iir & (BTIF_IIR_TX_EMPTY | BTIF_IIR_RX |
					 BTIF_IIR_RX_TIMEOUT))) ? IRQ_HANDLED : IRQ_NONE;
}

int mt6797_btif_claim(struct device *client,
			      const struct mt6797_btif_client_ops *ops,
			      void *context)
{
	struct mt6797_btif *btif;
	int ret;

	if (!client || !ops || !ops->rx_bytes)
		return -EINVAL;
	mutex_lock(&mt6797_btif_singleton_lock);
	btif = mt6797_btif_singleton;
	if (!btif) {
		ret = -ENODEV;
		goto out;
	}
	mutex_lock(&btif->client_lock);
	if (btif->ops) {
		ret = -EBUSY;
		goto unlock_client;
	}
	ret = clk_prepare_enable(btif->clk);
	if (ret)
		goto unlock_client;
	btif_hw_init(btif);
	if (btif->dma_mode) {
		ret = btif_dma_start(btif);
		if (ret) {
			clk_disable_unprepare(btif->clk);
			goto unlock_client;
		}
		btif_update_bits(btif, BTIF_IER, BTIF_IER_RXFEN | BTIF_IER_TXEEN, 0);
		btif_update_bits(btif, BTIF_DMA_EN, BIT(0) | BIT(1), BIT(0) | BIT(1));
	}
	btif->debug_irq_count = 0;
	btif_debug_state(btif, "claim");
	WRITE_ONCE(btif->ops, ops);
	btif->owner = client;
	WRITE_ONCE(btif->context, context);
	WRITE_ONCE(btif->running, true);
	if (btif->dma_mode) {
		enable_irq(btif->tx_dma.irq);
		enable_irq(btif->rx_dma.irq);
		writel(3, btif->rx_dma.base + VFF_INT_EN);
		writel(1, btif->rx_dma.base + VFF_EN);
	} else {
		enable_irq(btif->irq);
	}
	dev_info(btif->dev, "BTIF claimed: %s\n", btif->dma_mode ? "DMA/VFIFO" : "PIO");
	ret = 0;
unlock_client:
	mutex_unlock(&btif->client_lock);
out:
	mutex_unlock(&mt6797_btif_singleton_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mt6797_btif_claim);

void mt6797_btif_release(struct device *client)
{
	struct mt6797_btif *btif;

	if (!client)
		return;
	mutex_lock(&mt6797_btif_singleton_lock);
	btif = mt6797_btif_singleton;
	if (!btif)
		goto out;
	mutex_lock(&btif->client_lock);
	if (btif->ops && btif->owner == client)
		WRITE_ONCE(btif->running, false);
	else if (btif->ops) {
		dev_warn(btif->dev, "release from non-owner device\n");
		goto unlock_client;
	} else
		goto unlock_client;
	if (btif->dma_mode) {
		disable_irq(btif->tx_dma.irq);
		disable_irq(btif->rx_dma.irq);
		btif_dma_stop(btif);
	} else {
		disable_irq(btif->irq);
	}
	btif_update_bits(btif, BTIF_IER, BTIF_IER_RXFEN | BTIF_IER_TXEEN, 0);
	WRITE_ONCE(btif->ops, NULL);
	btif->owner = NULL;
	WRITE_ONCE(btif->context, NULL);
	spin_lock_irq(&btif->tx_lock);
	kfifo_reset(&btif->tx_fifo);
	spin_unlock_irq(&btif->tx_lock);
	clk_disable_unprepare(btif->clk);
unlock_client:
	mutex_unlock(&btif->client_lock);
out:
	mutex_unlock(&mt6797_btif_singleton_lock);
}
EXPORT_SYMBOL_GPL(mt6797_btif_release);

ssize_t mt6797_btif_write(struct device *client, const u8 *data,
			   size_t length, unsigned long timeout_jiffies)
{
	struct mt6797_btif *btif;
	ssize_t written = 0;

	if (!client || !data)
		return -EINVAL;
	mutex_lock(&mt6797_btif_singleton_lock);
	btif = mt6797_btif_singleton;
	if (!btif || btif->owner != client || !btif->running) {
		mutex_unlock(&mt6797_btif_singleton_lock);
		return -ENODEV;
	}
	if (btif->dma_mode) {
		written = btif_dma_write(btif, data, length, timeout_jiffies);
		mutex_unlock(&mt6797_btif_singleton_lock);
		return written;
	}
	while (written < length) {
		unsigned int count;
		long wait_ret;

		spin_lock_irq(&btif->tx_lock);
		count = kfifo_in(&btif->tx_fifo, data + written, length - written);
		if (count)
			btif_update_bits(btif, BTIF_IER, BTIF_IER_TXEEN,
					 BTIF_IER_TXEEN);
		spin_unlock_irq(&btif->tx_lock);
		written += count;
		if (trace && count)
			dev_info(btif->dev,
				 "BTIF debug write queued=%u/%zu data=%*phN ier=%08x lsr=%08x\n",
				 count, length, (int)min_t(size_t, count, 32), data + written - count,
				 readl(btif->base + BTIF_IER), readl(btif->base + BTIF_LSR));
		if (written == length)
			break;
		if (!timeout_jiffies)
			break;
		wait_ret = wait_event_interruptible_timeout(btif->tx_wait,
							!kfifo_is_full(&btif->tx_fifo),
							timeout_jiffies);
		if (wait_ret <= 0) {
			if (!wait_ret && written)
				break;
			written = wait_ret ? -ERESTARTSYS : -ETIMEDOUT;
			break;
		}
		timeout_jiffies = wait_ret;
	}
	mutex_unlock(&mt6797_btif_singleton_lock);
	return written;
}
EXPORT_SYMBOL_GPL(mt6797_btif_write);

int mt6797_btif_wake(struct device *client)
{
	struct mt6797_btif *btif;

	mutex_lock(&mt6797_btif_singleton_lock);
	btif = mt6797_btif_singleton;
	if (!btif || btif->owner != client || !btif->running) {
		mutex_unlock(&mt6797_btif_singleton_lock);
		return -ENODEV;
	}
	writel(BTIF_WAK_SIGNAL, btif->base + BTIF_WAK);
	mutex_unlock(&mt6797_btif_singleton_lock);
	return 0;
}
EXPORT_SYMBOL_GPL(mt6797_btif_wake);

static ssize_t transfer_mode_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct mt6797_btif *btif = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%s tx=%llu rx=%llu fault=%u running=%u\n",
		btif->dma_mode ? "dma" : "pio", READ_ONCE(btif->tx_dma.bytes),
		READ_ONCE(btif->rx_dma.bytes), READ_ONCE(btif->dma_fault),
		READ_ONCE(btif->running));
}

static ssize_t transfer_mode_store(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct mt6797_btif *btif = dev_get_drvdata(dev);
	bool dma;
	int ret = 0;

	if (sysfs_streq(buf, "dma"))
		dma = true;
	else if (sysfs_streq(buf, "pio"))
		dma = false;
	else
		return -EINVAL;
	mutex_lock(&mt6797_btif_singleton_lock);
	mutex_lock(&btif->client_lock);
	if (btif->running || btif->dma_clocked)
		ret = -EBUSY;
	else if (dma && !btif->rx_dma.buffer)
		ret = -EOPNOTSUPP;
	else
		btif->dma_mode = dma;
	mutex_unlock(&btif->client_lock);
	mutex_unlock(&mt6797_btif_singleton_lock);
	return ret ?: count;
}
static DEVICE_ATTR_RW(transfer_mode);
static struct attribute *btif_attrs[] = { &dev_attr_transfer_mode.attr, NULL };
ATTRIBUTE_GROUPS(btif);

static int btif_dma_probe(struct platform_device *pdev, struct mt6797_btif *btif)
{
	struct btif_dma_channel *channels[] = { &btif->tx_dma, &btif->rx_dma };
	irq_handler_t handlers[] = { btif_dma_tx_irq, btif_dma_rx_irq };
	const char *names[] = { "mt6797-btif-tx", "mt6797-btif-rx" };
	int i, ret;

	if (!platform_get_resource(pdev, IORESOURCE_MEM, 1))
		return 0;
	btif->dma_clk = devm_clk_get(&pdev->dev, "dma");
	if (IS_ERR(btif->dma_clk))
		return PTR_ERR(btif->dma_clk);
	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(&pdev->dev, btif_dma_free, btif);
	if (ret)
		return ret;
	for (i = 0; i < ARRAY_SIZE(channels); i++) {
		channels[i]->base = devm_platform_ioremap_resource(pdev, i + 1);
		if (IS_ERR(channels[i]->base))
			return PTR_ERR(channels[i]->base);
		channels[i]->irq = platform_get_irq(pdev, i + 1);
		if (channels[i]->irq < 0)
			return channels[i]->irq;
		channels[i]->buffer = dma_alloc_coherent(&pdev->dev, BTIF_VFIFO_SIZE,
							&channels[i]->address, GFP_KERNEL);
		if (!channels[i]->buffer)
			return -ENOMEM;
		ret = devm_request_threaded_irq(&pdev->dev, channels[i]->irq, NULL,
						 handlers[i], IRQF_ONESHOT | IRQF_NO_AUTOEN,
						 names[i], btif);
		if (ret)
			return ret;
	}
	return 0;
}

static int mt6797_btif_probe(struct platform_device *pdev)
{
	struct mt6797_btif *btif;
	int ret;

	mutex_lock(&mt6797_btif_singleton_lock);
	if (mt6797_btif_singleton) {
		mutex_unlock(&mt6797_btif_singleton_lock);
		return -EBUSY;
	}
	mutex_unlock(&mt6797_btif_singleton_lock);

	btif = devm_kzalloc(&pdev->dev, sizeof(*btif), GFP_KERNEL);
	if (!btif)
		return -ENOMEM;
	btif->dev = &pdev->dev;
	btif->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(btif->base))
		return PTR_ERR(btif->base);
	btif->irq = platform_get_irq(pdev, 0);
	if (btif->irq < 0)
		return btif->irq;
	btif->clk = devm_clk_get(&pdev->dev, "btif");
	if (IS_ERR(btif->clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(btif->clk),
				     "BTIF clock missing\n");
	mutex_init(&btif->client_lock);
	spin_lock_init(&btif->tx_lock);
	init_waitqueue_head(&btif->tx_wait);
	ret = kfifo_alloc(&btif->tx_fifo, BTIF_TX_QUEUE_SIZE, GFP_KERNEL);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(&pdev->dev, btif_kfifo_free,
				       &btif->tx_fifo);
	if (ret)
		return ret;
	ret = devm_request_irq(&pdev->dev, btif->irq, mt6797_btif_irq,
				IRQF_NO_AUTOEN, dev_name(&pdev->dev), btif);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "BTIF IRQ request failed\n");
	ret = btif_dma_probe(pdev, btif);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "BTIF DMA setup failed\n");
	platform_set_drvdata(pdev, btif);
	mutex_lock(&mt6797_btif_singleton_lock);
	mt6797_btif_singleton = btif;
	mutex_unlock(&mt6797_btif_singleton_lock);
	dev_info(&pdev->dev, "MT6797 BTIF transport ready (PIO%s; STP client required)\n",
		 btif->rx_dma.buffer ? ", DMA/VFIFO available" : "");
	return 0;
}

static void mt6797_btif_remove(struct platform_device *pdev)
{
	struct mt6797_btif *btif = platform_get_drvdata(pdev);

	mt6797_btif_release(btif ? btif->owner : NULL);
	mutex_lock(&mt6797_btif_singleton_lock);
	if (mt6797_btif_singleton == btif)
		mt6797_btif_singleton = NULL;
	mutex_unlock(&mt6797_btif_singleton_lock);
}

static const struct of_device_id mt6797_btif_of_match[] = {
	{ .compatible = "mediatek,mt6797-btif" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6797_btif_of_match);

static struct platform_driver mt6797_btif_driver = {
	.probe = mt6797_btif_probe,
	.remove = mt6797_btif_remove,
	.driver = {
		.name = "mt6797-btif",
		.of_match_table = mt6797_btif_of_match,
		.dev_groups = btif_groups,
	},
};
module_platform_driver(mt6797_btif_driver);

MODULE_DESCRIPTION("MT6797 ConSys BTIF PIO and DMA/VFIFO transport");
MODULE_LICENSE("GPL");
