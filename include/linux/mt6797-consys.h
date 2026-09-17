/* SPDX-License-Identifier: GPL-2.0-only */
/* MT6797 ConSys host-state callbacks used by STP/WMT clients. */
#ifndef __LINUX_MT6797_CONSYS_H
#define __LINUX_MT6797_CONSYS_H

#include <linux/bitops.h>
#include <linux/types.h>

#define MT6797_CONSYS_STP_BTIF_FULL_MODE	BIT(2)

/* The context is the MT6797 ConSys platform driver's private state. */
int mt6797_consys_stp_set_mode(void *context, u32 mode);
int mt6797_consys_stp_set_enabled(void *context, bool enabled);

#endif /* __LINUX_MT6797_CONSYS_H */
