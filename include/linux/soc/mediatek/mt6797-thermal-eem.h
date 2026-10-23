/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __LINUX_SOC_MEDIATEK_MT6797_THERMAL_EEM_H
#define __LINUX_SOC_MEDIATEK_MT6797_THERMAL_EEM_H

#include <linux/spinlock.h>
#include <linux/types.h>

/* MT6797 thermal and EEM share the PTPCORESEL bank-select register. */
extern spinlock_t mt6797_thermal_eem_lock;

struct device;
struct mt6797_thermal_bank_cookie {
	unsigned long flags;
	bool was_armed;
	u64 hold_started_ns;
};

int mt6797_thermal_bank_begin(struct device *dev, unsigned int bank,
			      struct mt6797_thermal_bank_cookie *cookie);
int mt6797_thermal_bank_end(struct device *dev,
			    struct mt6797_thermal_bank_cookie *cookie);
int mt6797_thermal_otp_calibration(struct device *dev, u32 *mts, u32 *bts);
int mt6797_thermal_eem_calibration(struct device *dev, unsigned int bank,
				 u32 *mts, u32 *bts);
int mt6797_thermal_bank_temp(struct device *dev, unsigned int bank, int *temp);

#endif
