/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * MT6797 BTIF byte transport contract (PIO or dedicated DMA/VFIFO).
 *
 * BTIF is the ConSys host byte transport.  It is not a tty or a 16550 UART;
 * callers must put STP/WMT framing above this interface.  The RX callback is
 * invoked from IRQ or threaded IRQ context and must only copy/schedule work.
 */
#ifndef __LINUX_MT6797_BTIF_H
#define __LINUX_MT6797_BTIF_H

#include <linux/device.h>
#include <linux/types.h>

struct mt6797_btif_client_ops {
	void (*rx_bytes)(void *context, const u8 *data, size_t length);
};

int mt6797_btif_claim(struct device *client,
			      const struct mt6797_btif_client_ops *ops,
			      void *context);
void mt6797_btif_release(struct device *client);
ssize_t mt6797_btif_write(struct device *client, const u8 *data,
			   size_t length, unsigned long timeout_jiffies);
int mt6797_btif_wake(struct device *client);
/* WMT sleep is acknowledged before entry; retain FIFO/register state. */
int mt6797_btif_idle(struct device *client, bool idle);

#endif /* __LINUX_MT6797_BTIF_H */
