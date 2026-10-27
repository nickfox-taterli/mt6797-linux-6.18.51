/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_SOC_MEDIATEK_MT6797_TOPRGU_H
#define _LINUX_SOC_MEDIATEK_MT6797_TOPRGU_H

#include <linux/errno.h>
#include <linux/kconfig.h>
#include <linux/types.h>

#if IS_ENABLED(CONFIG_ARCH_MEDIATEK)
int mt6797_toprgu_update(unsigned int id, bool assert);
#else
static inline int mt6797_toprgu_update(unsigned int id, bool assert)
{
	return -EOPNOTSUPP;
}
#endif

#endif
