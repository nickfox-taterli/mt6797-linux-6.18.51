/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_SOC_MEDIATEK_MT6797_CPUFREQ_H
#define _LINUX_SOC_MEDIATEK_MT6797_CPUFREQ_H

struct device;

int mt6797_cpufreq_eem_measure(struct device *dev,
			      int (*measure)(void *), void *context);

#endif
