/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef MTK_SD_MT6797_AUTOK_H
#define MTK_SD_MT6797_AUTOK_H

#include <linux/types.h>

struct mmc_host;

/* Fixed result-record size: see MT6797_SDAUTOK_RESULT_SIZE in
 * mt6797-sd-autok/mt6797-sd-autok.h, which mtk-sd.c includes. */

int mt6797_sd_autok_execute(struct mmc_host *mmc, void __iomem *base,
			    u32 hclk, u32 sclk, u8 *result);
void mt6797_sd_autok_apply(struct mmc_host *mmc, void __iomem *base,
			   u32 hclk, u32 sclk, const u8 *result);

#endif
