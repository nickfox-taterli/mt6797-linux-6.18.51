// SPDX-License-Identifier: GPL-2.0-only
/* Linux 6.18 interface adapter; scanning and selection stay in BSP autok.c. */
#include <linux/module.h>

#include "mtk-sd-mt6797-autok.h"
#include "mt6797-bsp-autok/autok.h"

#include "mt6797-bsp-autok/tune-path.h"

int mt6797_bsp_autok_execute(struct mmc_host *mmc, void __iomem *base,
			    u32 hclk, u32 sclk, u8 *result)
{
	struct mt6797_autok_ctx host = {
		.base = base,
		.hclk = hclk, .sclk = sclk,
	};
	u32 pio;
	int ret;

	BUILD_BUG_ON(TUNING_PARAM_COUNT != MT6797_BSP_AUTOK_RESULT_SIZE);
	if (!sclk || hclk < sclk)
		return -EINVAL;

	/* No MMC request is active: BSP tuning polls the controller directly. */
	pio = autok_get_field(MSDC_CFG, MSDC_CFG_PIO);
	autok_set_field(MSDC_CFG, MSDC_CFG_PIO, 1);
	msdc_init_tune_path(&host, mmc->ios.timing);
	autok_msdc_tx_setting(&host, &mmc->ios);
	ret = autok_execute_tuning(&host, result);
	autok_set_field(MSDC_CFG, MSDC_CFG_PIO, pio);

	/* Propagate BSP failures to the modern MMC core; never substitute a point. */
	if (ret || host.autok_error || result[INT_DAT_LATCH_CK] > 7) {
		dev_err(mmc_dev(mmc), "SDR104 tuning failed (ret=%d, error=%d, latch=%u)\n",
			ret, host.autok_error, result[INT_DAT_LATCH_CK]);
		return -EIO;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(mt6797_bsp_autok_execute);

void mt6797_bsp_autok_apply(struct mmc_host *mmc, void __iomem *base,
			   u32 hclk, u32 sclk, u8 *result)
{
	struct mt6797_autok_ctx host = {
		.base = base,
		.hclk = hclk, .sclk = sclk,
	};

	/* Match the BSP's saved-result application path. */
	autok_init_sdr104(&host);
	autok_tuning_parameter_init(&host, result);
	autok_msdc_tx_setting(&host, &mmc->ios);
}
EXPORT_SYMBOL_GPL(mt6797_bsp_autok_apply);

MODULE_DESCRIPTION("Original MT6797 BSP AutoK with Linux 6.18 interface adapter");
MODULE_LICENSE("GPL");
