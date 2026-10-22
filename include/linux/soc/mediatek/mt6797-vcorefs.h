/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_SOC_MEDIATEK_MT6797_VCOREFS_H
#define _LINUX_SOC_MEDIATEK_MT6797_VCOREFS_H

#include <linux/types.h>

struct device;
struct regmap;

struct mt6797_spm_resources {
	void __iomem *spm, *pwrap, *phy;
	struct regmap *pmic;
	struct device *dcm;
	u32 dummy_address;
	bool mapping_4g;
};

int mt6797_vcorefs_sleep_resources(struct device *dev,
				 struct mt6797_spm_resources *resources);

/* System sleep may borrow SPM only after devfreq has suspended and offloaded
 * its PCM. Release requires the sleep PCM to be reset before devfreq resumes.
 */
int mt6797_vcorefs_claim_sleep(struct device *dev, struct device *owner);
int mt6797_vcorefs_release_sleep(struct device *dev, struct device *owner);

#endif
