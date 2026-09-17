/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * MT6797 WLAN AHB sdioLike host interface.
 *
 * The HIF is not an ARM register file and it is not an MMC SDIO host.  A
 * caller must first obtain the WLAN function through the ConSys/WMT owner;
 * only then may it claim this interface and issue the indirect CMD53 PIO
 * operation below.  Probe and claim deliberately do not touch the HIF
 * window.
 */
#ifndef __LINUX_MT6797_WLAN_HIF_H
#define __LINUX_MT6797_WLAN_HIF_H

#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/types.h>

#define MT6797_WLAN_HIF_WCIR		0x0000

/* Call only after the WMT WLAN function-on transaction succeeded. */
int mt6797_wlan_hif_claim(struct device *client);
int mt6797_wlan_hif_release(struct device *client);

/* 32-bit CMD53 byte-mode access to the WLAN function's register window. */
int mt6797_wlan_hif_pio_read(struct device *client, u32 address, u32 *value);
int mt6797_wlan_hif_pio_write(struct device *client, u32 address, u32 value);
int mt6797_wlan_hif_wcir_read(struct device *client, u32 *value);

/* IRQ ownership is separate from the HIF claim and is explicitly deferred. */
int mt6797_wlan_hif_request_irq(struct device *client, irq_handler_t handler,
				unsigned long flags, const char *name, void *data);
int mt6797_wlan_hif_free_irq(struct device *client, void *data);
int mt6797_wlan_hif_enable_irq(struct device *client);
int mt6797_wlan_hif_disable_irq(struct device *client);

#endif /* __LINUX_MT6797_WLAN_HIF_H */
