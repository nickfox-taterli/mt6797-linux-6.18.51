/* SPDX-License-Identifier: GPL-2.0-only */
/* MT6797 ConSys host-state callbacks used by STP/WMT clients. */
#ifndef __LINUX_MT6797_CONSYS_H
#define __LINUX_MT6797_CONSYS_H

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/types.h>

#define MT6797_CONSYS_EMI_REGION_SIZE	0x200000
#define MT6797_CONSYS_WIFI_EMI_SIZE	0x080000

#define MT6797_CONSYS_STP_BTIF_FULL_MODE	BIT(2)

/* WMT function type values from the MT6797 vendor protocol. */
#define MT6797_CONSYS_FUNC_BT		0
#define MT6797_CONSYS_FUNC_WIFI		3

struct mt6797_stp_host_ops;

/*
 * Reserve the ConSys host parser before claiming BTIF.  The reservation is
 * separate from set_enabled(): it keeps a concurrent sysfs power-off from
 * tearing down MMIO while a STP transaction is in flight.
 */
int mt6797_consys_stp_acquire(struct device *consys_dev,
				      struct mt6797_stp_host_ops *ops,
				      void **context);
int mt6797_consys_stp_release(void *context);

/* Functional clients own power through this reservation; release powers off.
 * Diagnostic acquire above retains its explicit manual power contract.
 */
int mt6797_consys_stp_open(struct device *consys_dev,
			 struct mt6797_stp_host_ops *ops, void **context);

/* The context is the MT6797 ConSys platform driver's private state. */
int mt6797_consys_stp_set_mode(void *context, u32 mode);
int mt6797_consys_stp_set_enabled(void *context, bool enabled);

/* Own the external MT6631 VCN33 rail for a live WMT function. */
int mt6797_consys_radio_power(void *context, u8 function, bool enabled,
				     bool *changed);

/* Read the reserved ConSys EMI region while the caller owns the STP context. */
int mt6797_consys_get_emi(void *context, phys_addr_t *base,
				  phys_addr_t *size);
int mt6797_consys_read_cpupcr(void *context, u32 *value);

/* Reserve/release the shared EMI transaction used by firmware download. */
int mt6797_consys_emi_patch_begin(void *context);
int mt6797_consys_emi_patch_end(void *context);

/* Bounded, verified WLAN firmware copy with MPU restoration on every exit. */
int mt6797_consys_wifi_emi_write(void *context, u32 offset,
			       const void *data, size_t length);

#endif /* __LINUX_MT6797_CONSYS_H */
