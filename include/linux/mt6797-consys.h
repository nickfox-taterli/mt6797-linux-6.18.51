/* SPDX-License-Identifier: GPL-2.0-only */
/* MT6797 ConSys host-state callbacks used by STP/WMT clients. */
#ifndef __LINUX_MT6797_CONSYS_H
#define __LINUX_MT6797_CONSYS_H

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/types.h>

#define MT6797_CONSYS_STP_BTIF_FULL_MODE	BIT(2)

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

/* The context is the MT6797 ConSys platform driver's private state. */
int mt6797_consys_stp_set_mode(void *context, u32 mode);
int mt6797_consys_stp_set_enabled(void *context, bool enabled);

#endif /* __LINUX_MT6797_CONSYS_H */
