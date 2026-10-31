/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * MT6797 MSDC1 SDR104 tuning: result layout shared between the scan engine,
 * the stored replay window and mtk-sd.c.
 *
 * The 18-byte result record is a fixed interface: mtk-sd.c stores it across
 * clock changes and the apply path writes it back register by register.
 * Indices 0..17 follow the scan order of the tuning parameters below.
 */
#ifndef MT6797_SDAUTOK_H
#define MT6797_SDAUTOK_H

#include <linux/bits.h>
#include <linux/io.h>
#include <linux/mmc/host.h>
#include <linux/types.h>

/* Tuned parameters carried in the result record. */
enum mt6797_sdautok_result {
	AUTOK_CMD_EDGE,
	AUTOK_RDATA_EDGE,
	AUTOK_RD_FIFO_EDGE,
	AUTOK_WD_FIFO_EDGE,
	AUTOK_CMD_RD_D_DLY1,
	AUTOK_CMD_RD_D_DLY1_SEL,
	AUTOK_CMD_RD_D_DLY2,
	AUTOK_CMD_RD_D_DLY2_SEL,
	AUTOK_DAT_RD_DLY1,
	AUTOK_DAT_RD_DLY1_SEL,
	AUTOK_DAT_RD_DLY2,
	AUTOK_DAT_RD_DLY2_SEL,
	AUTOK_INT_DAT_LATCH_CK,
	AUTOK_EMMC50_DS_Z_DLY1,
	AUTOK_EMMC50_DS_Z_DLY1_SEL,
	AUTOK_EMMC50_DS_Z_DLY2,
	AUTOK_EMMC50_DS_Z_DLY2_SEL,
	AUTOK_EMMC50_DS_ZDLY_DLY,
	MT6797_SDAUTOK_RESULT_SIZE,
};

/* Additional controller parameters fixed by the tune-path setup.  These
 * index the shared parameter table, so they continue after the result
 * record instead of restarting at zero. */
enum mt6797_sdautok_path_param {
	AUTOK_PATH_READ_DATA_SMPL_SEL = MT6797_SDAUTOK_RESULT_SIZE,
	AUTOK_PATH_WRITE_DATA_SMPL_SEL,
	AUTOK_PATH_DATA_DLYLINE_SEL,
	AUTOK_PATH_DAT_TUNE_SEL,
	AUTOK_PATH_WCRC_ASYNC_FIFO_SEL,
	AUTOK_PATH_RESP_ASYNC_FIFO_SEL,
	AUTOK_PATH_EMMC50_WDATA_MUX_EN,
	AUTOK_PATH_EMMC50_CMD_MUX_EN,
	AUTOK_PATH_EMMC50_WDATA_EDGE,
	AUTOK_PATH_CKGEN_MSDC_DLY_SEL,
	AUTOK_PATH_CMD_RSP_TA_CNTR,
	AUTOK_PATH_WRDAT_CRCS_TA_CNTR,
	AUTOK_PATH_PAD_CLK_TXDLY,
};

/* Failure classification of one tuning command (bit per class). */
enum mt6797_sdautok_error {
	AUTOK_ERR_NONE		= 0,
	AUTOK_ERR_CMD_TMO	= BIT(0),
	AUTOK_ERR_RSP_CRC	= BIT(1),
	AUTOK_ERR_DAT_CRC	= BIT(2),
	AUTOK_ERR_DAT_TMO	= BIT(3),
	AUTOK_ERR_FATAL		= BIT(5),
};

/* Per-tuning-operation state; no request is active while tuning runs. */
struct mt6797_sdautok_ctx {
	void __iomem *base;
	unsigned int hclk;
	unsigned int sclk;
	unsigned int latch_blk_cnt;
	int error;
};

int mt6797_sdautok_execute(struct mt6797_sdautok_ctx *ctx, u8 *result);
void mt6797_sdautok_apply(struct mt6797_sdautok_ctx *ctx, const u8 *result);
void sdautok_tune_path_init(const struct mt6797_sdautok_ctx *ctx);
void mt6797_sdautok_tx_setting(struct mt6797_sdautok_ctx *ctx, bool sdr104);

/* Whole run with PIO isolation bracketing tune-path setup and scan. */
int mt6797_sdautok_run(struct mt6797_sdautok_ctx *ctx, u8 *result);

#endif /* MT6797_SDAUTOK_H */
