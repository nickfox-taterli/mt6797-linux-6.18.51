/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __MT6797_MCUCLK_H
#define __MT6797_MCUCLK_H

#include <linux/types.h>

struct platform_device;
struct mt6797_mcuclk;

struct mt6797_freq_method {
	unsigned int khz;
	unsigned int pos_div;
	unsigned int clk_div;
};

struct mt6797_mcuclk_snapshot {
	u32 mux, ckdiv, little_dcm, big_dcm;
	int little_error, big_error;
	bool fhctl, ready, sync_dcm;
	unsigned long hops, hop_failures, dcm_updates;
};

struct mt6797_mcuclk_measurement {
	unsigned int khz;
	int error;
};

/* CPUFreq must hold its rail-vote lock around a rate transition. */
struct mt6797_mcuclk *devm_mt6797_mcuclk_create(struct platform_device *pdev);
unsigned int mt6797_mcuclk_rate(struct mt6797_mcuclk *clk, unsigned int id);
void mt6797_mcuclk_forget(struct mt6797_mcuclk *clk, unsigned int id);
int mt6797_mcuclk_transition(struct mt6797_mcuclk *clk, unsigned int id,
			     const struct mt6797_freq_method *old,
			     const struct mt6797_freq_method *target);
int mt6797_mcuclk_sync(struct mt6797_mcuclk *clk, unsigned int id,
			unsigned int khz);
int mt6797_mcuclk_armcore(struct mt6797_mcuclk *clk, u32 *value, bool write);
void mt6797_mcuclk_snapshot(struct mt6797_mcuclk *clk, bool big_active,
			    struct mt6797_mcuclk_snapshot *snapshot);
int mt6797_mcuclk_measure(struct mt6797_mcuclk *clk, unsigned int active,
			  struct mt6797_mcuclk_measurement results[5]);
#endif
