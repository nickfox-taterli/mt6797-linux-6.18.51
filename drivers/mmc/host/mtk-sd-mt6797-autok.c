// SPDX-License-Identifier: GPL-2.0-only
/* Linux 6.18 interface adapter; scanning and selection stay in BSP autok.c. */
#include <linux/module.h>
#include <linux/timekeeping.h>
#include <linux/time_types.h>

#include "mtk-sd-mt6797-autok.h"
#include "mt6797-bsp-autok/autok.h"

/* Declarations missing from the old BSP header; definitions remain untouched. */
int execute_online_tuning_hs400(struct msdc_host *host, u8 *res);
int execute_cmd_online_tuning(struct msdc_host *host, u8 *res);
int autok_execute_tuning_latch_ck(struct msdc_host *host, unsigned int opcode,
				unsigned int latch_ck_initail_value);
int execute_online_tuning_hs200(struct msdc_host *host, u8 *res);
int execute_online_tuning(struct msdc_host *host, u8 *res);
int execute_online_tuning_stress(struct msdc_host *host);
int execute_offline_tuning_hs400(struct msdc_host *host);
int execute_offline_tuning(struct msdc_host *host);
void msdc_init_tune_path(struct msdc_host *host, unsigned char timing);

/* The BSP uses timeval only for its elapsed-time log. */
static void mt6797_autok_gettimeofday(struct __kernel_old_timeval *tv)
{
	struct timespec64 now;

	ktime_get_real_ts64(&now);
	tv->tv_sec = now.tv_sec;
	tv->tv_usec = now.tv_nsec / NSEC_PER_USEC;
}

#define timeval __kernel_old_timeval
#define do_gettimeofday mt6797_autok_gettimeofday
#include "mt6797-bsp-autok/autok.c"
#include "mt6797-bsp-autok/tune-path.h"
#undef do_gettimeofday
#undef timeval

int mt6797_bsp_autok_execute(struct mmc_host *mmc, void __iomem *base,
			    u32 hclk, u32 sclk, u8 *result)
{
	struct msdc_hw hw = { .host_function = MSDC_SD };
	struct msdc_host host = {
		.base = base, .mmc = mmc, .hw = &hw, .id = 1,
		.hclk = hclk, .sclk = sclk,
	};
	u32 pio;
	int ret;

	BUILD_BUG_ON(TUNING_PARAM_COUNT != MT6797_BSP_AUTOK_RESULT_SIZE);
	if (!sclk || hclk < sclk)
		return -EINVAL;

	/* No MMC request is active: BSP tuning polls the controller directly. */
	MSDC_GET_FIELD(MSDC_CFG, MSDC_CFG_PIO, pio);
	MSDC_SET_FIELD(MSDC_CFG, MSDC_CFG_PIO, 1);
	msdc_init_tune_path(&host, mmc->ios.timing);
	autok_msdc_tx_setting(&host, &mmc->ios);
	ret = autok_execute_tuning(&host, result);
	MSDC_SET_FIELD(MSDC_CFG, MSDC_CFG_PIO, pio);

	/* Propagate BSP failures to the modern MMC core; never substitute a point. */
	if (ret || host.autok_error || result[INT_DAT_LATCH_CK] > 7)
		return -EIO;
	return 0;
}
EXPORT_SYMBOL_GPL(mt6797_bsp_autok_execute);

void mt6797_bsp_autok_apply(struct mmc_host *mmc, void __iomem *base,
			   u32 hclk, u32 sclk, u8 *result)
{
	struct msdc_hw hw = { .host_function = MSDC_SD };
	struct msdc_host host = {
		.base = base, .mmc = mmc, .hw = &hw, .id = 1,
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
