/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef MTK_SD_MT6797_AUTOK_H
#define MTK_SD_MT6797_AUTOK_H

#include <linux/types.h>

struct mmc_host;

/* TUNING_PARAM_COUNT in the original BSP autok.h. */
#define MT6797_BSP_AUTOK_RESULT_SIZE 18

int mt6797_bsp_autok_execute(struct mmc_host *mmc, void __iomem *base,
			    u32 hclk, u32 sclk, u8 *result);
void mt6797_bsp_autok_apply(struct mmc_host *mmc, void __iomem *base,
			   u32 hclk, u32 sclk, u8 *result);

#endif
