/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __LINUX_SOC_MEDIATEK_MT6797_THERMAL_EEM_H
#define __LINUX_SOC_MEDIATEK_MT6797_THERMAL_EEM_H

#include <linux/spinlock.h>

/* MT6797 thermal and EEM share the PTPCORESEL bank-select register. */
extern spinlock_t mt6797_thermal_eem_lock;

#endif
