// SPDX-License-Identifier: GPL-2.0-only
//
// MT6797 BTIF PIO transport for the ConSys STP/WMT stack.
//
// The block has UART-like FIFOs but is not a serial console.  This driver
// deliberately exposes only an in-kernel byte transport: STP framing,
// function ownership and HCI demultiplexing belong to its client.

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kfifo.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/wait.h>

#include <linux/mt6797-btif.h>

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

struct mt6797_btif {
	struct device *dev;
	void __iomem *base;
	struct clk *clk;
	int irq;

	struct mutex client_lock;
	struct device *owner;
	const struct mt6797_btif_client_ops *ops;
	void *context;
	bool running;

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
	btif_update_bits(btif, BTIF_IER, BTIF_IER_RXFEN | BTIF_IER_TXEEN, 0);
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
	WRITE_ONCE(btif->ops, ops);
	btif->owner = client;
	WRITE_ONCE(btif->context, context);
	WRITE_ONCE(btif->running, true);
	enable_irq(btif->irq);
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
	disable_irq(btif->irq);
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
	platform_set_drvdata(pdev, btif);
	mutex_lock(&mt6797_btif_singleton_lock);
	mt6797_btif_singleton = btif;
	mutex_unlock(&mt6797_btif_singleton_lock);
	dev_info(&pdev->dev, "MT6797 BTIF PIO transport ready (STP client required)\n");
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
	},
};
module_platform_driver(mt6797_btif_driver);

MODULE_DESCRIPTION("MT6797 ConSys BTIF PIO transport");
MODULE_LICENSE("GPL");
