// SPDX-License-Identifier: GPL-2.0-only
/*
 * mtk-sd interface adapter for the MT6797 MSDC1 SDR104 tuning engine.
 *
 * Bridges the kernel MMC core to the board tuning flow.  The MMC core
 * owns the policy (when to tune, what opcode); this module owns the
 * Linux-side calling convention and failure propagation.  The stored
 * result record feeds the replay on later clock changes.
 */
#include <linux/module.h>

#include "mtk-sd-mt6797-autok.h"
#include "mt6797-sd-autok/mt6797-sd-autok.h"

int mt6797_sd_autok_execute(struct mmc_host *mmc, void __iomem *base,
			    u32 hclk, u32 sclk, u8 *result)
{
	struct mt6797_sdautok_ctx ctx = {
		.base = base,
		.hclk = hclk, .sclk = sclk,
	};
	int ret;

	if (!sclk || hclk < sclk)
		return -EINVAL;

	ret = mt6797_sdautok_run(&ctx, result);

	/* Propagate failures to the modern MMC core; never substitute a point. */
	if (ret || ctx.error || result[AUTOK_INT_DAT_LATCH_CK] > 7) {
		dev_err(mmc_dev(mmc), "SDR104 tuning failed (ret=%d, error=%d, latch=%u)\n",
			ret, ctx.error, result[AUTOK_INT_DAT_LATCH_CK]);
		return -EIO;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(mt6797_sd_autok_execute);

void mt6797_sd_autok_apply(struct mmc_host *mmc, void __iomem *base,
			   u32 hclk, u32 sclk, const u8 *result)
{
	struct mt6797_sdautok_ctx ctx = {
		.base = base,
		.hclk = hclk, .sclk = sclk,
	};

	/* Replay the stored window; the timing setup precedes it. */
	mt6797_sdautok_apply(&ctx, result);
	mt6797_sdautok_tx_setting(&ctx, true);
}
EXPORT_SYMBOL_GPL(mt6797_sd_autok_apply);

MODULE_DESCRIPTION("MT6797 MSDC1 SDR104 tuning engine, mtk-sd adapter");
MODULE_LICENSE("GPL");
