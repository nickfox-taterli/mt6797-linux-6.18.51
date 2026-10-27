/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_SOC_MEDIATEK_MT6797_WDT_H
#define _LINUX_SOC_MEDIATEK_MT6797_WDT_H

#include <linux/errno.h>
#include <linux/kconfig.h>
#include <linux/types.h>

struct device;
#if IS_REACHABLE(CONFIG_MEDIATEK_WATCHDOG)
int mt6797_wdt_thermal_direct(struct device *dev, bool enable);
int mt6797_wdt_thermal_request(struct device *dev, bool enable);
int mt6797_wdt_thermal_hold(struct device *dev, bool *was_armed);
int mt6797_wdt_reset_request(struct device *dev, u32 request, bool enable);
#else
static inline int mt6797_wdt_thermal_direct(struct device *dev, bool enable)
{
	return -EOPNOTSUPP;
}
static inline int mt6797_wdt_thermal_request(struct device *dev, bool enable)
{
	return -EOPNOTSUPP;
}
static inline int mt6797_wdt_thermal_hold(struct device *dev, bool *was_armed)
{
	return -EOPNOTSUPP;
}

static inline int mt6797_wdt_reset_request(struct device *dev, u32 request,
					   bool enable)
{
	return -EOPNOTSUPP;
}
#endif
#endif
