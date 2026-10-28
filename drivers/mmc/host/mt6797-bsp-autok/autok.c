/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include "autok.h"
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/io.h>

#include <linux/time.h>
#include <linux/delay.h>

#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sdio.h>

#include <linux/completion.h>
#include <linux/scatterlist.h>

#define AUTOK_CMD_TIMEOUT               (HZ / 10) /* 100ms */
#define AUTOK_DAT_TIMEOUT               (HZ * 3) /* 1s x 3 */
#define HS400_ONE_CYCLE                 (50)
#define HS400_ACTUAL_CYCLE              (50 - 10 * 2)
#define MSDC_FIFO_THD_1K                (1024)
#define TUNE_TX_CNT                     (100)
#define TUNE_DATA_TX_ADDR               (0x358000)
#define CMDQ
#define AUTOK_LATCH_CK_EMMC_TUNE_TIMES     (10) /* 5.0IP eMMC 1KB fifo ZIZE */
#define AUTOK_LATCH_CK_SDIO_TUNE_TIMES  (3) /* 4.5IP 128B fifo CMD19 need send 3 times  */
#define AUTOK_LATCH_CK_SD_TUNE_TIMES    (3) /* 4.5IP 128B fifo CMD19 need send 3 times  */
#define AUTOK_CMD_TIMES                 (20)
#define AUTOK_TUNING_INACCURACY         (3) /* scan result may find xxxxooxxx */
#define AUTOK_MARGIN_THOLD              (5)
#define AUTOK_BD_WIDTH_REF              (3)

#define AUTOK_READ                      0
#define AUTOK_WRITE                     1

#define AUTOK_FINAL_CKGEN_SEL           (0)
#define SCALE_TA_CNTR                   (8)
#define SCALE_CMD_RSP_TA_CNTR           (8)
#define SCALE_WDAT_CRC_TA_CNTR          (8)
#define SCALE_INT_DAT_LATCH_CK_SEL      (8)
#define SCALE_INTERNAL_DLY_CNTR         (32)
#define SCALE_PAD_DAT_DLY_CNTR          (32)

#define TUNING_INACCURACY (2)

/* autok platform specific setting */
#define AUTOK_CKGEN_VALUE                       (0)
#define AUTOK_CMD_LATCH_EN_HS400_PORT0_VALUE    (5)
#define AUTOK_CRC_LATCH_EN_HS400_PORT0_VALUE    (5)
#define AUTOK_CMD_LATCH_EN_HS200_PORT0_VALUE    (3)
#define AUTOK_CRC_LATCH_EN_HS200_PORT0_VALUE    (3)
#define AUTOK_CMD_LATCH_EN_SDR104_PORT1_VALUE   (4)
#define AUTOK_CRC_LATCH_EN_SDR104_PORT1_VALUE   (4)
#define AUTOK_CMD_LATCH_EN_HS_VALUE             (2)
#define AUTOK_CRC_LATCH_EN_HS_VALUE            (2)
#define AUTOK_LATCH_CK_VALUE                    (1)
#define AUTOK_CMD_TA_VALUE                      (0)
#define AUTOK_CRC_TA_VALUE                      (0)
#define AUTOK_CRC_MA_VALUE                      (1)
#define AUTOK_BUSY_MA_VALUE                     (1)

/* autok msdc TX init setting */
#define AUTOK_MSDC0_HS400_CLKTXDLY            0
#define AUTOK_MSDC0_HS400_CMDTXDLY            9
#define AUTOK_MSDC0_HS400_DAT0TXDLY           0
#define AUTOK_MSDC0_HS400_DAT1TXDLY           0
#define AUTOK_MSDC0_HS400_DAT2TXDLY           0
#define AUTOK_MSDC0_HS400_DAT3TXDLY           0
#define AUTOK_MSDC0_HS400_DAT4TXDLY           0
#define AUTOK_MSDC0_HS400_DAT5TXDLY           0
#define AUTOK_MSDC0_HS400_DAT6TXDLY           0
#define AUTOK_MSDC0_HS400_DAT7TXDLY           0
#define AUTOK_MSDC0_HS400_TXSKEW              0

#define AUTOK_MSDC0_DDR50_DDRCKD              1
#define AUTOK_MSDC_DDRCKD                     0

#define AUTOK_MSDC0_CLKTXDLY                  0
#define AUTOK_MSDC0_CMDTXDLY                  0
#define AUTOK_MSDC0_DAT0TXDLY                 0
#define AUTOK_MSDC0_DAT1TXDLY                 0
#define AUTOK_MSDC0_DAT2TXDLY                 0
#define AUTOK_MSDC0_DAT3TXDLY                 0
#define AUTOK_MSDC0_DAT4TXDLY                 0
#define AUTOK_MSDC0_DAT5TXDLY                 0
#define AUTOK_MSDC0_DAT6TXDLY                 0
#define AUTOK_MSDC0_DAT7TXDLY                 0

#define AUTOK_MSDC0_TXSKEW                    0

#define AUTOK_MSDC1_CLK_TX_VALUE              0
#define AUTOK_MSDC1_CLK_SDR104_TX_VALUE       0

#define AUTOK_MSDC2_CLK_TX_VALUE              0

/* #define PORT0_PB0_RD_DAT_SEL_VALID */
#define PORT1_PB0_RD_DAT_SEL_VALID
#define PORT2_PB0_RD_DAT_SEL_VALID

enum TUNE_TYPE {
	TUNE_CMD = 0,
	TUNE_DATA,
	TUNE_LATCH_CK,
};

#define autok_msdc_retry(expr, retry, cnt) \
	do { \
		int backup = cnt; \
		while (retry) { \
			if (!(expr)) \
				break; \
			if (cnt-- == 0) { \
				retry--; cnt = backup; \
			} \
		} \
	WARN_ON(retry == 0); \
} while (0)

#define autok_msdc_reset() \
	do { \
		int retry = 3, cnt = 1000; \
		autok_set_bits(MSDC_CFG, MSDC_CFG_RST); \
		/* ensure reset operation be sequential  */ \
		mb(); \
		autok_msdc_retry(autok_read(MSDC_CFG) & MSDC_CFG_RST, retry, cnt); \
	} while (0)

#define msdc_rxfifocnt() \
	((autok_read(MSDC_FIFOCS) & MSDC_FIFOCS_RXCNT) >> 0)
#define msdc_txfifocnt() \
	((autok_read(MSDC_FIFOCS) & MSDC_FIFOCS_TXCNT) >> 16)

#define wait_cond_tmo(cond, tmo) \
	do { \
		unsigned long timeout = jiffies + tmo; \
		while (1) { \
			if ((cond) || (tmo == 0)) \
				break; \
			if (time_after(jiffies, timeout)) \
				tmo = 0; \
		} \
	} while (0)

#define msdc_clear_fifo() \
	do { \
		int retry = 5, cnt = 1000; \
		autok_set_bits(MSDC_FIFOCS, MSDC_FIFOCS_CLR); \
		/* ensure fifo clear operation be sequential  */ \
		mb(); \
		autok_msdc_retry(autok_read(MSDC_FIFOCS) & MSDC_FIFOCS_CLR, retry, cnt); \
	} while (0)

struct AUTOK_PARAM_RANGE {
	unsigned int start;
	unsigned int end;
};

struct AUTOK_PARAM_INFO {
	struct AUTOK_PARAM_RANGE range;
	char *param_name;
};

#include "analysis.h"

unsigned int autok_debug_level = AUTOK_DBG_RES;

const struct AUTOK_PARAM_INFO autok_param_info[] = {
	{{0, 1}, "CMD_EDGE"},
	{{0, 1}, "RDATA_EDGE"},         /* async fifo mode Pad dat edge must fix to 0 */
	{{0, 1}, "RD_FIFO_EDGE"},
	{{0, 1}, "WD_FIFO_EDGE"},

	{{0, 31}, "CMD_RD_D_DLY1"},     /* Cmd Pad Tune Data Phase */
	{{0, 1}, "CMD_RD_D_DLY1_SEL"},
	{{0, 31}, "CMD_RD_D_DLY2"},
	{{0, 1}, "CMD_RD_D_DLY2_SEL"},

	{{0, 31}, "DAT_RD_D_DLY1"},     /* Data Pad Tune Data Phase */
	{{0, 1}, "DAT_RD_D_DLY1_SEL"},
	{{0, 31}, "DAT_RD_D_DLY2"},
	{{0, 1}, "DAT_RD_D_DLY2_SEL"},

	{{0, 7}, "INT_DAT_LATCH_CK"},   /* Latch CK Delay for data read when clock stop */

	{{0, 31}, "EMMC50_DS_Z_DLY1"},	/* eMMC50 Related tuning param */
	{{0, 1}, "EMMC50_DS_Z_DLY1_SEL"},
	{{0, 31}, "EMMC50_DS_Z_DLY2"},
	{{0, 1}, "EMMC50_DS_Z_DLY2_SEL"},
	{{0, 31}, "EMMC50_DS_ZDLY_DLY"},

	/* ================================================= */
	/* Timming Related Mux & Common Setting Config */
	{{0, 1}, "READ_DATA_SMPL_SEL"},         /* all data line path share sample edge */
	{{0, 1}, "WRITE_DATA_SMPL_SEL"},
	{{0, 1}, "DATA_DLYLINE_SEL"},           /* clK tune all data Line share dly */
	{{0, 1}, "MSDC_WCRC_ASYNC_FIFO_SEL"},   /* data tune mode select */
	{{0, 1}, "MSDC_RESP_ASYNC_FIFO_SEL"},   /* data tune mode select */
	/* eMMC50 Function Mux */
	{{0, 1}, "EMMC50_WDATA_MUX_EN"},        /* write path switch to emmc45 */
	{{0, 1}, "EMMC50_CMD_MUX_EN"},          /* response path switch to emmc45 */
	{{0, 1}, "EMMC50_WDATA_EDGE"},
	/* Common Setting Config */
	{{0, 31}, "CKGEN_MSDC_DLY_SEL"},
	{{1, 7}, "CMD_RSP_TA_CNTR"},
	{{1, 7}, "WRDAT_CRCS_TA_CNTR"},
	{{0, 31}, "PAD_CLK_TXDLY"},             /* tx clk dly fix to 0 for HQA res */
};

/**********************************************************
* AutoK Basic Interface Implenment                        *
**********************************************************/

static int autok_send_tune_cmd(struct mt6797_autok_ctx *host, unsigned int opcode, enum TUNE_TYPE tune_type_value)
{
	void __iomem *base = host->base;
	unsigned int value;
	unsigned int rawcmd = 0;
	unsigned int arg = 0;
	unsigned int sts = 0;
	unsigned int wints = 0;
	unsigned long tmo = 0;
	unsigned long write_tmo = 0;
	unsigned int left = 0;
	unsigned int fifo_have = 0;
	unsigned int fifo_1k_cnt = 0;
	unsigned int i = 0;
	int ret = E_RESULT_PASS;

	switch (opcode) {
	case MMC_SEND_EXT_CSD:
		rawcmd =  (512 << 16) | (0 << 13) | (1 << 11) | (1 << 7) | (8);
		arg = 0;
		if (tune_type_value == TUNE_LATCH_CK)
			autok_write(SDC_BLK_NUM, host->tune_latch_ck_cnt);
		else
			autok_write(SDC_BLK_NUM, 1);
		break;
	case MMC_STOP_TRANSMISSION:
		rawcmd = (1 << 14)  | (7 << 7) | (12);
		arg = 0;
		break;
	case MMC_SEND_STATUS:
		rawcmd = (1 << 7) | (13);
		arg = (1 << 16);
		break;
	case MMC_READ_SINGLE_BLOCK:
		left = 512;
		rawcmd =  (512 << 16) | (0 << 13) | (1 << 11) | (1 << 7) | (17);
		arg = 0;
		if (tune_type_value == TUNE_LATCH_CK)
			autok_write(SDC_BLK_NUM, host->tune_latch_ck_cnt);
		else
			autok_write(SDC_BLK_NUM, 1);
		break;
	case MMC_SEND_TUNING_BLOCK:
		left = 64;
		rawcmd =  (64 << 16) | (0 << 13) | (1 << 11) | (1 << 7) | (19);
		arg = 0;
		if (tune_type_value == TUNE_LATCH_CK)
			autok_write(SDC_BLK_NUM, host->tune_latch_ck_cnt);
		else
			autok_write(SDC_BLK_NUM, 1);
		break;
	case MMC_SEND_TUNING_BLOCK_HS200:
		left = 128;
		rawcmd =  (128 << 16) | (0 << 13) | (1 << 11) | (1 << 7) | (21);
		arg = 0;
		if (tune_type_value == TUNE_LATCH_CK)
			autok_write(SDC_BLK_NUM, host->tune_latch_ck_cnt);
		else
			autok_write(SDC_BLK_NUM, 1);
		break;
	case MMC_WRITE_BLOCK:
		rawcmd =  (512 << 16) | (1 << 13) | (1 << 11) | (1 << 7) | (24);
		arg = TUNE_DATA_TX_ADDR;
		break;
	case SD_IO_RW_DIRECT:
		break;
	case SD_IO_RW_EXTENDED:
		break;
	}

	tmo = AUTOK_DAT_TIMEOUT;
	wait_cond_tmo(!(autok_read(SDC_STS) & SDC_STS_SDCBUSY), tmo);
	if (tmo == 0) {
		AUTOK_RAWPRINT("[AUTOK]MSDC busy tmo1 cmd%d goto end...\n", opcode);
		ret = E_RESULT_FATAL_ERR;
		goto end;
	}

	/* clear fifo */
	if ((tune_type_value == TUNE_CMD) || (tune_type_value == TUNE_DATA)) {
		autok_msdc_reset();
		msdc_clear_fifo();
		autok_write(MSDC_INT, 0xffffffff);
	}

	/* start command */
	autok_write(SDC_ARG, arg);
	autok_write(SDC_CMD, rawcmd);

	/* wait interrupt status */
	wints = MSDC_INT_CMDTMO | MSDC_INT_CMDRDY | MSDC_INT_RSPCRCERR;
	tmo = AUTOK_CMD_TIMEOUT;
	wait_cond_tmo(((sts = autok_read(MSDC_INT)) & wints), tmo);
	if (tmo == 0) {
		AUTOK_RAWPRINT("[AUTOK]CMD%d wait int tmo\r\n", opcode);
		ret = E_RESULT_CMD_TMO;
		goto end;
	}

	autok_write(MSDC_INT, (sts & wints));
	if (sts == 0) {
		ret = E_RESULT_CMD_TMO;
		goto end;
	}

	if (sts & MSDC_INT_CMDRDY) {
		if (tune_type_value == TUNE_CMD) {
			ret = E_RESULT_PASS;
			goto end;
		}
	} else if (sts & MSDC_INT_RSPCRCERR) {
		ret = E_RESULT_RSP_CRC;
		goto end;
	} else if (sts & MSDC_INT_CMDTMO) {
		AUTOK_RAWPRINT("[AUTOK]CMD%d HW tmo\r\n", opcode);
		ret = E_RESULT_CMD_TMO;
		goto end;
	}

	if ((tune_type_value != TUNE_LATCH_CK) && (tune_type_value != TUNE_DATA))
		goto skip_tune_latch_ck_and_tune_data;
	tmo = jiffies + AUTOK_DAT_TIMEOUT;
	while ((autok_read(SDC_STS) & SDC_STS_SDCBUSY) && (tmo != 0)) {
		if (time_after(jiffies, tmo))
			tmo = 0;
		if (tune_type_value == TUNE_LATCH_CK) {
			fifo_have = msdc_rxfifocnt();
			if ((opcode == MMC_SEND_TUNING_BLOCK_HS200) || (opcode == MMC_READ_SINGLE_BLOCK)
				|| (opcode == MMC_SEND_EXT_CSD)) {
				autok_set_field(MSDC_DBG_SEL, 0xffff << 0, 0x0b);
				fifo_1k_cnt = autok_get_field(MSDC_DBG_OUT, 0x7ff << 0);
				if ((fifo_1k_cnt >= MSDC_FIFO_THD_1K) && (fifo_have >= MSDC_FIFO_SZ)) {
					value = autok_read(MSDC_RXDATA);
					value = autok_read(MSDC_RXDATA);
					value = autok_read(MSDC_RXDATA);
					value = autok_read(MSDC_RXDATA);
				}
			} else if (opcode == MMC_SEND_TUNING_BLOCK) {
				if (fifo_have >= MSDC_FIFO_SZ) {
					value = autok_read(MSDC_RXDATA);
					value = autok_read(MSDC_RXDATA);
					value = autok_read(MSDC_RXDATA);
					value = autok_read(MSDC_RXDATA);
				}
			}
		} else if ((tune_type_value == TUNE_DATA) && (opcode == MMC_WRITE_BLOCK)) {
			for (i = 0; i < 64; i++) {
				autok_write(MSDC_TXDATA, 0x5af00fa5);
				autok_write(MSDC_TXDATA, 0x33cc33cc);
			}

			write_tmo = AUTOK_DAT_TIMEOUT;
			wait_cond_tmo(!(autok_read(SDC_STS) & SDC_STS_SDCBUSY), write_tmo);
			if (write_tmo == 0) {
				AUTOK_RAWPRINT("[AUTOK]MSDC busy tmo2 while write cmd%d goto end...\n", opcode);
				ret = E_RESULT_FATAL_ERR;
				goto end;
			}
		}
	}
	if (tmo == 0) {
		AUTOK_RAWPRINT("[AUTOK]MSDC busy tmo3 cmd%d goto end...\n", opcode);
		ret = E_RESULT_FATAL_ERR;
		goto end;
	}

	sts = autok_read(MSDC_INT);
	wints = MSDC_INT_XFER_COMPL | MSDC_INT_DATCRCERR | MSDC_INT_DATTMO;
	if (sts) {
		/* clear status */
		autok_write(MSDC_INT, (sts & wints));
		if (sts & MSDC_INT_XFER_COMPL)
			ret = E_RESULT_PASS;
		if (MSDC_INT_DATCRCERR & sts)
			ret = E_RESULT_DAT_CRC;
		if (MSDC_INT_DATTMO & sts)
			ret = E_RESULT_DAT_TMO;
	}

skip_tune_latch_ck_and_tune_data:
	tmo = AUTOK_DAT_TIMEOUT;
	wait_cond_tmo(!(autok_read(SDC_STS) & SDC_STS_SDCBUSY), tmo);
	if (tmo == 0) {
		AUTOK_RAWPRINT("[AUTOK]MSDC busy tmo4 cmd%d goto end...\n", opcode);
		ret = E_RESULT_FATAL_ERR;
		goto end;
	}
	if ((tune_type_value == TUNE_CMD) || (tune_type_value == TUNE_DATA))
		msdc_clear_fifo();

end:
	if (opcode == MMC_STOP_TRANSMISSION) {
		tmo = AUTOK_DAT_TIMEOUT;
		wait_cond_tmo(((autok_read(MSDC_PS) & 0x10000) == 0x10000), tmo);
		if (tmo == 0) {
			AUTOK_RAWPRINT("[AUTOK]DTA0 busy tmo cmd%d goto end...\n", opcode);
			ret = E_RESULT_FATAL_ERR;
		}
	}

	return ret;
}


/*************************************************************************
* FUNCTION
*  msdc_autok_adjust_param
*
* DESCRIPTION
*  This function for auto-K, adjust msdc parameter
*
* PARAMETERS
*    host: msdc host manipulator pointer
*    param: enum of msdc parameter
*    value: value of msdc parameter
*    rw: AUTOK_READ/AUTOK_WRITE
*
* RETURN VALUES
*    error code: 0 success,
*               -1 parameter input error
*               -2 read/write fail
*               -3 else error
*************************************************************************/
static int msdc_autok_adjust_param(struct mt6797_autok_ctx *host, enum AUTOK_PARAM param, u32 *value,
				   int rw)
{
	void __iomem *base = host->base;
	void __iomem *reg;
	u32 field = 0;

	switch (param) {
	case READ_DATA_SMPL_SEL:
		if ((rw == AUTOK_WRITE) && (*value > 1)) {
			pr_debug
			    ("[%s] Input value(%d) for READ_DATA_SMPL_SEL is out of range, it should be [0~1]\n",
			     __func__, *value);
			return -1;
		}

		reg = MSDC_IOCON;
		field = (u32) (MSDC_IOCON_R_D_SMPL_SEL);
		break;
	case WRITE_DATA_SMPL_SEL:
		if ((rw == AUTOK_WRITE) && (*value > 1)) {
			pr_debug
			    ("[%s] Input value(%d) for WRITE_DATA_SMPL_SEL is out of range, it should be [0~1]\n",
			     __func__, *value);
			return -1;
		}

		reg = MSDC_IOCON;
		field = (u32) (MSDC_IOCON_W_D_SMPL_SEL);
		break;
	case DATA_DLYLINE_SEL:
		if ((rw == AUTOK_WRITE) && (*value > 1)) {
			pr_debug
			    ("[%s] Input value(%d) for DATA_DLYLINE_SEL is out of range, it should be [0~1]\n",
			     __func__, *value);
			return -1;
		}

		reg = MSDC_IOCON;
		field = (u32) (MSDC_IOCON_DDLSEL);
		break;
	case MSDC_DAT_TUNE_SEL:	/* 0-Dat tune 1-CLk tune ; */
		if ((rw == AUTOK_WRITE) && (*value > 1)) {
			pr_debug
			    ("[%s] Input value(%d) for DATA_DLYLINE_SEL is out of range, it should be [0~1]\n",
			     __func__, *value);
			return -1;
		}
		reg = MSDC_PAD_TUNE0;
		field = (u32) (MSDC_PAD_TUNE0_RXDLYSEL);
		break;
	case MSDC_WCRC_ASYNC_FIFO_SEL:
		if ((rw == AUTOK_WRITE) && (*value > 1)) {
			pr_debug
			    ("[%s] Input value(%d) for DATA_DLYLINE_SEL is out of range, it should be [0~1]\n",
			     __func__, *value);
			return -1;
		}
		reg = MSDC_PATCH_BIT2;
		field = (u32) (MSDC_PB2_CFGCRCSTS);
		break;
	case MSDC_RESP_ASYNC_FIFO_SEL:
		if ((rw == AUTOK_WRITE) && (*value > 1)) {
			pr_debug
			    ("[%s] Input value(%d) for DATA_DLYLINE_SEL is out of range, it should be [0~1]\n",
			     __func__, *value);
			return -1;
		}
		reg = MSDC_PATCH_BIT2;
		field = (u32) (MSDC_PB2_CFGRESP);
		break;
	case CMD_EDGE:
		if ((rw == AUTOK_WRITE) && (*value > 1)) {
			pr_debug
			    ("[%s] Input value(%d) for CMD_EDGE is out of range, it should be [0~1]\n",
			     __func__, *value);
			return -1;
		}
		reg = MSDC_IOCON;
		field = (u32) (MSDC_IOCON_RSPL);
		break;
	case RDATA_EDGE:
		if ((rw == AUTOK_WRITE) && (*value > 1)) {
			pr_debug
			    ("[%s] Input value(%d) for RDATA_EDGE is out of range, it should be [0~1]\n",
			     __func__, *value);
			return -1;
		}
		reg = MSDC_IOCON;
		field = (u32) (MSDC_IOCON_R_D_SMPL);
		break;
	case RD_FIFO_EDGE:
		if ((rw == AUTOK_WRITE) && (*value > 1)) {
			pr_debug
			    ("[%s] Input value(%d) for RDATA_EDGE is out of range, it should be [0~1]\n",
			     __func__, *value);
			return -1;
		}
		reg = MSDC_PATCH_BIT0;
		field = (u32) (MSDC_PB0_RD_DAT_SEL);
		break;
	case WD_FIFO_EDGE:
		if ((rw == AUTOK_WRITE) && (*value > 1)) {
			pr_debug
			    ("[%s] Input value(%d) for WDATA_EDGE is out of range, it should be [0~1]\n",
			     __func__, *value);
			return -1;
		}
		reg = MSDC_PATCH_BIT2;
		field = (u32) (MSDC_PB2_CFGCRCSTSEDGE);
		break;
	case CMD_RD_D_DLY1:
		if ((rw == AUTOK_WRITE) && (*value > 31)) {
			pr_debug
			    ("[%s] Input value(%d) for CMD_RD_DLY is out of range, it should be [0~31]\n",
			     __func__, *value);
			return -1;
		}
		reg = MSDC_PAD_TUNE0;
		field = (u32) (MSDC_PAD_TUNE0_CMDRDLY);
		break;
	case CMD_RD_D_DLY1_SEL:
		if ((rw == AUTOK_WRITE) && (*value > 1)) {
			pr_debug
			    ("[%s] Input value(%d) for CMD_RD_DLY is out of range, it should be [0~31]\n",
			     __func__, *value);
			return -1;
		}
		reg = MSDC_PAD_TUNE0;
		field = (u32) (MSDC_PAD_TUNE0_CMDRRDLYSEL);
		break;
	case CMD_RD_D_DLY2:
		if ((rw == AUTOK_WRITE) && (*value > 31)) {
			pr_debug
			    ("[%s] Input value(%d) for CMD_RD_DLY is out of range, it should be [0~31]\n",
			     __func__, *value);
			return -1;
		}
		reg = MSDC_PAD_TUNE1;
		field = (u32) (MSDC_PAD_TUNE1_CMDRDLY2);
		break;
	case CMD_RD_D_DLY2_SEL:
		if ((rw == AUTOK_WRITE) && (*value > 1)) {
			pr_debug
			    ("[%s] Input value(%d) for CMD_RD_DLY is out of range, it should be [0~31]\n",
			     __func__, *value);
			return -1;
		}
		reg = MSDC_PAD_TUNE1;
		field = (u32) (MSDC_PAD_TUNE1_CMDRRDLY2SEL);
		break;
	case DAT_RD_D_DLY1:
		if ((rw == AUTOK_WRITE) && (*value > 31)) {
			pr_debug
			    ("[%s] Input value(%d) for DAT0_RD_DLY is out of range, it should be [0~31]\n",
			     __func__, *value);
			return -1;
		}
		reg = MSDC_PAD_TUNE0;
		field = (u32) (MSDC_PAD_TUNE0_DATRRDLY);
		break;
	case DAT_RD_D_DLY1_SEL:
		if ((rw == AUTOK_WRITE) && (*value > 1)) {
			pr_debug
			    ("[%s] Input value(%d) for CMD_RD_DLY is out of range, it should be [0~31]\n",
			     __func__, *value);
			return -1;
		}
		reg = MSDC_PAD_TUNE0;
		field = (u32) (MSDC_PAD_TUNE0_DATRRDLYSEL);
		break;
	case DAT_RD_D_DLY2:
		if ((rw == AUTOK_WRITE) && (*value > 31)) {
			pr_debug
			    ("[%s] Input value(%d) for DAT1_RD_DLY is out of range, it should be [0~31]\n",
			     __func__, *value);
			return -1;
		}
		reg = MSDC_PAD_TUNE1;
		field = (u32) (MSDC_PAD_TUNE1_DATRRDLY2);
		break;
	case DAT_RD_D_DLY2_SEL:
		if ((rw == AUTOK_WRITE) && (*value > 1)) {
			pr_debug
			    ("[%s] Input value(%d) for CMD_RD_DLY is out of range, it should be [0~31]\n",
			     __func__, *value);
			return -1;
		}
		reg = MSDC_PAD_TUNE1;
		field = (u32) (MSDC_PAD_TUNE1_DATRRDLY2SEL);
		break;
	case INT_DAT_LATCH_CK:
		if ((rw == AUTOK_WRITE) && (*value > 7)) {
			pr_debug
			    ("[%s] Input value(%d) for INT_DAT_LATCH_CK is out of range, it should be [0~7]\n",
			     __func__, *value);
			return -1;
		}
		reg = MSDC_PATCH_BIT0;
		field = (u32) (MSDC_PB0_INT_DAT_LATCH_CK_SEL);
		break;
	case CKGEN_MSDC_DLY_SEL:
		if ((rw == AUTOK_WRITE) && (*value > 31)) {
			pr_debug
			    ("[%s] Input value(%d) for CKGEN_MSDC_DLY_SEL is out of range, it should be [0~31]\n",
			     __func__, *value);
			return -1;
		}
		reg = MSDC_PATCH_BIT0;
		field = (u32) (MSDC_PB0_CKGEN_MSDC_DLY_SEL);
		break;
	case CMD_RSP_TA_CNTR:
		if ((rw == AUTOK_WRITE) && (*value > 7)) {
			pr_debug
			    ("[%s] Input value(%d) for CMD_RSP_TA_CNTR is out of range, it should be [0~7]\n",
			     __func__, *value);
			return -1;
		}
		reg = MSDC_PATCH_BIT1;
		field = (u32) (MSDC_PB1_CMD_RSP_TA_CNTR);
		break;
	case WRDAT_CRCS_TA_CNTR:
		if ((rw == AUTOK_WRITE) && (*value > 7)) {
			pr_debug
			    ("[%s] Input value(%d) for WRDAT_CRCS_TA_CNTR is out of range, it should be [0~7]\n",
			     __func__, *value);
			return -1;
		}
		reg = MSDC_PATCH_BIT1;
		field = (u32) (MSDC_PB1_WRDAT_CRCS_TA_CNTR);
		break;
	case PAD_CLK_TXDLY:
		if ((rw == AUTOK_WRITE) && (*value > 31)) {
			pr_debug
			    ("[%s] Input value(%d) for PAD_CLK_TXDLY is out of range, it should be [0~31]\n",
			     __func__, *value);
			return -1;
		}
		reg = MSDC_PAD_TUNE0;
		field = (u32) (MSDC_PAD_TUNE0_CLKTXDLY);
		break;
	case EMMC50_WDATA_MUX_EN:
		if ((rw == AUTOK_WRITE) && (*value > 1)) {
			pr_debug
			    ("[%s] Input value(%d) for EMMC50_WDATA_MUX_EN is out of range, it should be [0~1]\n",
			     __func__, *value);
			return -1;
		}
		reg = EMMC50_CFG0;
		field = (u32) (MSDC_EMMC50_CFG_CRC_STS_SEL);
		break;
	case EMMC50_CMD_MUX_EN:
		if ((rw == AUTOK_WRITE) && (*value > 1)) {
			pr_debug
			    ("[%s] Input value(%d) for EMMC50_CMD_MUX_EN is out of range, it should be [0~1]\n",
			     __func__, *value);
			return -1;
		}
		reg = EMMC50_CFG0;
		field = (u32) (MSDC_EMMC50_CFG_CMD_RESP_SEL);
		break;
	case EMMC50_WDATA_EDGE:
		if ((rw == AUTOK_WRITE) && (*value > 1)) {
			pr_debug
			    ("[%s] Input value(%d) for EMMC50_WDATA_EDGE is out of range, it should be [0~1]\n",
			     __func__, *value);
			return -1;
		}
		reg = EMMC50_CFG0;
		field = (u32) (MSDC_EMMC50_CFG_CRC_STS_EDGE);
		break;
	case EMMC50_DS_Z_DLY1:
		if ((rw == AUTOK_WRITE) && (*value > 31)) {
			pr_debug
			    ("[%s] Input value(%d) for EMMC50_DS_Z_DLY1 is out of range, it should be [0~1]\n",
			     __func__, *value);
			return -1;
		}
		reg = EMMC50_PAD_DS_TUNE;
		field = (u32) (MSDC_EMMC50_PAD_DS_TUNE_DLY1);
		break;
	case EMMC50_DS_Z_DLY1_SEL:
		if ((rw == AUTOK_WRITE) && (*value > 1)) {
			pr_debug
			    ("[%s] Input value(%d) for EMMC50_DS_Z_DLY1_SEL is out of range, it should be [0~1]\n",
			     __func__, *value);
			return -1;
		}
		reg = EMMC50_PAD_DS_TUNE;
		field = (u32) (MSDC_EMMC50_PAD_DS_TUNE_DLYSEL);
		break;
	case EMMC50_DS_Z_DLY2:
		if ((rw == AUTOK_WRITE) && (*value > 31)) {
			pr_debug
			    ("[%s] Input value(%d) for EMMC50_DS_Z_DLY2 is out of range, it should be [0~1]\n",
			     __func__, *value);
			return -1;
		}
		reg = EMMC50_PAD_DS_TUNE;
		field = (u32) (MSDC_EMMC50_PAD_DS_TUNE_DLY2);
		break;
	case EMMC50_DS_Z_DLY2_SEL:
		if ((rw == AUTOK_WRITE) && (*value > 1)) {
			pr_debug
			    ("[%s] Input value(%d) for EMMC50_DS_Z_DLY1_SEL is out of range, it should be [0~1]\n",
			     __func__, *value);
			return -1;
		}
		reg = EMMC50_PAD_DS_TUNE;
		field = (u32) (MSDC_EMMC50_PAD_DS_TUNE_DLY2SEL);
		break;
	case EMMC50_DS_ZDLY_DLY:
		if ((rw == AUTOK_WRITE) && (*value > 31)) {
			pr_debug
			    ("[%s] Input value(%d) for EMMC50_DS_Z_DLY2 is out of range, it should be [0~1]\n",
			     __func__, *value);
			return -1;
		}
		reg = EMMC50_PAD_DS_TUNE;
		field = (u32) (MSDC_EMMC50_PAD_DS_TUNE_DLY3);
		break;
	default:
		pr_debug("[%s] Value of [enum AUTOK_PARAM param] is wrong\n", __func__);
		return -1;
	}

	if (rw == AUTOK_READ)
		*value = autok_get_field(reg, field);
	else if (rw == AUTOK_WRITE) {
		autok_set_field(reg, field, *value);

		if (param == CKGEN_MSDC_DLY_SEL)
			mdelay(1);
	} else {
		pr_debug("[%s] Value of [int rw] is wrong\n", __func__);
		return -1;
	}

	return 0;
}

static int autok_param_update(enum AUTOK_PARAM param_id, unsigned int result, u8 *autok_tune_res)
{
	if (param_id < TUNING_PARAM_COUNT) {
		if ((result > autok_param_info[param_id].range.end) ||
		    (result < autok_param_info[param_id].range.start)) {
			AUTOK_RAWPRINT("[AUTOK]param outof range : %d not in [%d,%d]\r\n",
				       result, autok_param_info[param_id].range.start,
				       autok_param_info[param_id].range.end);
			return -1;
		} else {
			autok_tune_res[param_id] = (u8) result;
			return 0;
		}
	}
	AUTOK_RAWPRINT("[AUTOK]param not found\r\n");

	return -1;
}

static int autok_param_apply(struct mt6797_autok_ctx *host, u8 *autok_tune_res)
{
	unsigned int i = 0;
	unsigned int value = 0;

	for (i = 0; i < TUNING_PARAM_COUNT; i++) {
		value = (u8) autok_tune_res[i];
		msdc_autok_adjust_param(host, i, &value, AUTOK_WRITE);
	}

	return 0;
}

static int autok_result_dump(struct mt6797_autok_ctx *host, u8 *autok_tune_res)
{
	AUTOK_RAWPRINT("[AUTOK]CMD [EDGE:%d DLY1:%d DLY2:%d ]\r\n",
		autok_tune_res[0], autok_tune_res[4], autok_tune_res[6]);
	AUTOK_RAWPRINT("[AUTOK]DAT [RDAT_EDGE:%d RD_FIFO_EDGE:%d WD_FIFO_EDGE:%d]\r\n",
		autok_tune_res[1], autok_tune_res[2], autok_tune_res[3]);
	AUTOK_RAWPRINT("[AUTOK]DAT [LATCH_CK:%d DLY1:%d DLY2:%d ]\r\n",
		autok_tune_res[12], autok_tune_res[8], autok_tune_res[10]);
	AUTOK_RAWPRINT("[AUTOK]DS  [DLY1:%d DLY2:%d DLY3:%d]\r\n",
		autok_tune_res[13], autok_tune_res[15], autok_tune_res[17]);

	return 0;
}

#if AUTOK_PARAM_DUMP_ENABLE
static int autok_register_dump(struct mt6797_autok_ctx *host)
{
	unsigned int i = 0;
	unsigned int value = 0;
	u8 autok_tune_res[TUNING_PARAM_COUNT];
	for (i = 0; i < TUNING_PARAM_COUNT; i++) {
		msdc_autok_adjust_param(host, i, &value, AUTOK_READ);
		autok_tune_res[i] = value;
	}
	AUTOK_RAWPRINT("[AUTOK]CMD [EDGE:%d DLY1:%d DLY2:%d ]\r\n",
		autok_tune_res[0], autok_tune_res[4], autok_tune_res[6]);
	AUTOK_RAWPRINT("[AUTOK]DAT [RDAT_EDGE:%d RD_FIFO_EDGE:%d WD_FIFO_EDGE:%d]\r\n",
		autok_tune_res[1], autok_tune_res[2], autok_tune_res[3]);
	AUTOK_RAWPRINT("[AUTOK]DAT [LATCH_CK:%d DLY1:%d DLY2:%d ]\r\n",
		autok_tune_res[12], autok_tune_res[8], autok_tune_res[10]);
	AUTOK_RAWPRINT("[AUTOK]DS  [DLY1:%d DLY2:%d DLY3:%d]\r\n",
		autok_tune_res[13], autok_tune_res[15], autok_tune_res[17]);

	return 0;
}
#endif

void autok_tuning_parameter_init(struct mt6797_autok_ctx *host, u8 *res)
{
	unsigned int ret = 0;
	/* void __iomem *base = host->base; */

	/* autok_set_field(MSDC_PATCH_BIT2, 7<<29, 2); */
	/* autok_set_field(MSDC_PATCH_BIT2, 7<<16, 4); */

	ret = autok_param_apply(host, res);
}

/*******************************************************
* Function: msdc_autok_adjust_paddly                   *
* Param : value - delay cnt from 0 to 63               *
*         pad_sel - 0 for cmd pad and 1 for data pad   *
*******************************************************/
#define CMD_PAD_RDLY 0
#define DAT_PAD_RDLY 1
#define DS_PAD_RDLY 2
static void msdc_autok_adjust_paddly(struct mt6797_autok_ctx *host, unsigned int *value,
				     unsigned int pad_sel)
{
	unsigned int uCfgL = 0;
	unsigned int uCfgLSel = 0;
	unsigned int uCfgH = 0;
	unsigned int uCfgHSel = 0;
	unsigned int dly_cnt = *value;

	uCfgL = (dly_cnt > 31) ? (31) : dly_cnt;
	uCfgH = (dly_cnt > 31) ? (dly_cnt - 32) : 0;

	uCfgLSel = (uCfgL > 0) ? 1 : 0;
	uCfgHSel = (uCfgH > 0) ? 1 : 0;
	switch (pad_sel) {
	case CMD_PAD_RDLY:
		msdc_autok_adjust_param(host, CMD_RD_D_DLY1, &uCfgL, AUTOK_WRITE);
		msdc_autok_adjust_param(host, CMD_RD_D_DLY2, &uCfgH, AUTOK_WRITE);

		msdc_autok_adjust_param(host, CMD_RD_D_DLY1_SEL, &uCfgLSel, AUTOK_WRITE);
		msdc_autok_adjust_param(host, CMD_RD_D_DLY2_SEL, &uCfgHSel, AUTOK_WRITE);
		break;
	case DAT_PAD_RDLY:
		msdc_autok_adjust_param(host, DAT_RD_D_DLY1, &uCfgL, AUTOK_WRITE);
		msdc_autok_adjust_param(host, DAT_RD_D_DLY2, &uCfgH, AUTOK_WRITE);

		msdc_autok_adjust_param(host, DAT_RD_D_DLY1_SEL, &uCfgLSel, AUTOK_WRITE);
		msdc_autok_adjust_param(host, DAT_RD_D_DLY2_SEL, &uCfgHSel, AUTOK_WRITE);
		break;
	case DS_PAD_RDLY:
		msdc_autok_adjust_param(host, EMMC50_DS_Z_DLY1, &uCfgL, AUTOK_WRITE);
		msdc_autok_adjust_param(host, EMMC50_DS_Z_DLY2, &uCfgH, AUTOK_WRITE);

		msdc_autok_adjust_param(host, EMMC50_DS_Z_DLY1_SEL, &uCfgLSel, AUTOK_WRITE);
		msdc_autok_adjust_param(host, EMMC50_DS_Z_DLY2_SEL, &uCfgHSel, AUTOK_WRITE);
		break;
	}
}

static void autok_paddly_update(unsigned int pad_sel, unsigned int dly_cnt, u8 *autok_tune_res)
{
	unsigned int uCfgL = 0;
	unsigned int uCfgLSel = 0;
	unsigned int uCfgH = 0;
	unsigned int uCfgHSel = 0;

	uCfgL = (dly_cnt > 31) ? (31) : dly_cnt;
	uCfgH = (dly_cnt > 31) ? (dly_cnt - 32) : 0;

	uCfgLSel = (uCfgL > 0) ? 1 : 0;
	uCfgHSel = (uCfgH > 0) ? 1 : 0;
	switch (pad_sel) {
	case CMD_PAD_RDLY:
		autok_param_update(CMD_RD_D_DLY1, uCfgL, autok_tune_res);
		autok_param_update(CMD_RD_D_DLY2, uCfgH, autok_tune_res);

		autok_param_update(CMD_RD_D_DLY1_SEL, uCfgLSel, autok_tune_res);
		autok_param_update(CMD_RD_D_DLY2_SEL, uCfgHSel, autok_tune_res);
		break;
	case DAT_PAD_RDLY:
		autok_param_update(DAT_RD_D_DLY1, uCfgL, autok_tune_res);
		autok_param_update(DAT_RD_D_DLY2, uCfgH, autok_tune_res);

		autok_param_update(DAT_RD_D_DLY1_SEL, uCfgLSel, autok_tune_res);
		autok_param_update(DAT_RD_D_DLY2_SEL, uCfgHSel, autok_tune_res);
		break;
	case DS_PAD_RDLY:
		autok_param_update(EMMC50_DS_Z_DLY1, uCfgL, autok_tune_res);
		autok_param_update(EMMC50_DS_Z_DLY2, uCfgH, autok_tune_res);

		autok_param_update(EMMC50_DS_Z_DLY1_SEL, uCfgLSel, autok_tune_res);
		autok_param_update(EMMC50_DS_Z_DLY2_SEL, uCfgHSel, autok_tune_res);
		break;
	}
}

/*******************************************************
* Exectue tuning IF Implenment                         *
*******************************************************/
static int autok_write_param(struct mt6797_autok_ctx *host, enum AUTOK_PARAM param, u32 value)
{
	msdc_autok_adjust_param(host, param, &value, AUTOK_WRITE);

	return 0;
}

int autok_path_sel(struct mt6797_autok_ctx *host)
{
	void __iomem *base = host->base;
	autok_write_param(host, READ_DATA_SMPL_SEL, 0);
	autok_write_param(host, WRITE_DATA_SMPL_SEL, 0);

	/* clK tune all data Line share dly */
	autok_write_param(host, DATA_DLYLINE_SEL, 0);

	/* data tune mode select */
#if defined(CHIP_DENALI_3_DAT_TUNE)
	autok_write_param(host, MSDC_DAT_TUNE_SEL, 1);
#else
	autok_write_param(host, MSDC_DAT_TUNE_SEL, 0);
#endif
	autok_write_param(host, MSDC_WCRC_ASYNC_FIFO_SEL, 1);
	autok_write_param(host, MSDC_RESP_ASYNC_FIFO_SEL, 0);

	/* eMMC50 Function Mux */
	/* write path switch to emmc45 */
	autok_write_param(host, EMMC50_WDATA_MUX_EN, 0);

	/* response path switch to emmc45 */
	autok_write_param(host, EMMC50_CMD_MUX_EN, 0);
	autok_write_param(host, EMMC50_WDATA_EDGE, 0);

	/* Common Setting Config */
	autok_write_param(host, CKGEN_MSDC_DLY_SEL, AUTOK_CKGEN_VALUE);
	autok_write_param(host, CMD_RSP_TA_CNTR, AUTOK_CMD_TA_VALUE);
	autok_write_param(host, WRDAT_CRCS_TA_CNTR, AUTOK_CRC_TA_VALUE);

	autok_set_field(MSDC_PATCH_BIT1, MSDC_PB1_GET_BUSY_MA, AUTOK_BUSY_MA_VALUE);
	autok_set_field(MSDC_PATCH_BIT1, MSDC_PB1_GET_CRC_MA, AUTOK_CRC_MA_VALUE);

	return 0;
}

int autok_init_sdr104(struct mt6797_autok_ctx *host)
{
	void __iomem *base = host->base;

	/* driver may miss data tune path setting in the interim */
	autok_path_sel(host);

	/* if any specific config need modify add here */
	if (host->sclk <= 100000000) {
		/* LATCH_TA_EN Config for WCRC Path HS FS mode */
		autok_set_field(MSDC_PATCH_BIT2, MSDC_PB2_CRCSTSENSEL, AUTOK_CRC_LATCH_EN_HS_VALUE);
		/* LATCH_TA_EN Config for CMD Path HS FS mode */
		autok_set_field(MSDC_PATCH_BIT2, MSDC_PB2_RESPSTENSEL, AUTOK_CMD_LATCH_EN_HS_VALUE);
	} else {
		/* LATCH_TA_EN Config for WCRC Path SDR104 mode */
		autok_set_field(MSDC_PATCH_BIT2, MSDC_PB2_CRCSTSENSEL, AUTOK_CRC_LATCH_EN_SDR104_PORT1_VALUE);
		/* LATCH_TA_EN Config for CMD Path SDR104 mode */
		autok_set_field(MSDC_PATCH_BIT2, MSDC_PB2_RESPSTENSEL, AUTOK_CMD_LATCH_EN_SDR104_PORT1_VALUE);
	}
	/* enable dvfs feature */
	/* Board SD tuning setup. */
	/*	autok_set_field(MSDC_CFG, MSDC_CFG_DVFS_EN, 1); */

	return 0;
}


/* online tuning for latch ck */
int autok_execute_tuning_latch_ck(struct mt6797_autok_ctx *host, unsigned int opcode,
	unsigned int latch_ck_initail_value)
{
	unsigned int ret = 0;
	unsigned int j, k;
	void __iomem *base = host->base;
	unsigned int tune_time;

	autok_write(MSDC_INT, 0xffffffff);
	tune_time = AUTOK_LATCH_CK_SD_TUNE_TIMES;
	for (j = latch_ck_initail_value; j < 8; j += (host->hclk / host->sclk)) {
		host->tune_latch_ck_cnt = 0;
		msdc_clear_fifo();
		autok_set_field(MSDC_PATCH_BIT0, MSDC_PB0_INT_DAT_LATCH_CK_SEL, j);
		for (k = 0; k < tune_time; k++) {
			if (opcode == MMC_SEND_TUNING_BLOCK_HS200) {
				switch (k) {
				case 0:
					host->tune_latch_ck_cnt = 1;
					break;
				default:
					host->tune_latch_ck_cnt = k;
					break;
				}
			} else if (opcode == MMC_SEND_TUNING_BLOCK) {
				switch (k) {
				case 0:
				case 1:
				case 2:
					host->tune_latch_ck_cnt = 1;
					break;
				default:
					host->tune_latch_ck_cnt = k - 1;
					break;
				}
			} else if (opcode == MMC_SEND_EXT_CSD) {
					host->tune_latch_ck_cnt = k + 1;
			} else
				host->tune_latch_ck_cnt++;
			ret = autok_send_tune_cmd(host, opcode, TUNE_LATCH_CK);
			if ((ret & (E_RESULT_CMD_TMO | E_RESULT_RSP_CRC)) != 0) {
				AUTOK_RAWPRINT("[AUTOK]Error Autok CMD Failed while tune LATCH CK\r\n");
				break;
			} else if ((ret & (E_RESULT_DAT_CRC | E_RESULT_DAT_TMO)) != 0) {
				AUTOK_RAWPRINT("[AUTOK]Error Autok  tune LATCH_CK error %d\r\n", j);
				break;
			}
		}
		if (ret == 0) {
			autok_set_field(MSDC_PATCH_BIT0, MSDC_PB0_INT_DAT_LATCH_CK_SEL, j);
			break;
		}
	}
	host->tune_latch_ck_cnt = 0;

	return j;

}

/* online tuning for eMMC4.5(hs200) */


/* online tuning for SDIO/SD */
int execute_online_tuning(struct mt6797_autok_ctx *host, u8 *res)
{
	void __iomem *base = host->base;
	unsigned int ret = 0;
	unsigned int uCmdEdge = 0;
	unsigned int uDatEdge = 0;
	u64 RawData64 = 0LL;
	unsigned int score = 0;
	unsigned int j, k;
	unsigned int opcode = MMC_SEND_TUNING_BLOCK;
	struct AUTOK_REF_INFO uCmdDatInfo;
	struct AUTOK_SCAN_RES *pBdInfo;
	char tune_result_str64[65];
	u8 p_autok_tune_res[TUNING_PARAM_COUNT];

	autok_init_sdr104(host);
	memset((void *)p_autok_tune_res, 0, sizeof(p_autok_tune_res) / sizeof(u8));

	/* Step1 : Tuning Cmd Path */
	autok_tuning_parameter_init(host, p_autok_tune_res);
	memset(&uCmdDatInfo, 0, sizeof(struct AUTOK_REF_INFO));

	uCmdEdge = 0;
	do {
		pBdInfo = (struct AUTOK_SCAN_RES *)&(uCmdDatInfo.scan_info[uCmdEdge]);
		msdc_autok_adjust_param(host, CMD_EDGE, &uCmdEdge, AUTOK_WRITE);
		RawData64 = 0LL;
		for (j = 0; j < 64; j++) {
			msdc_autok_adjust_paddly(host, &j, CMD_PAD_RDLY);
			for (k = 0; k < AUTOK_CMD_TIMES / 2; k++) {
				ret = autok_send_tune_cmd(host, opcode, TUNE_CMD);
				if ((ret & E_RESULT_RSP_CRC) != 0) {
					RawData64 |= (u64) (1LL << j);
					break;
				} else if ((ret & E_RESULT_CMD_TMO) != 0) {
					autok_msdc_reset();
					msdc_clear_fifo();
					autok_write(MSDC_INT, 0xffffffff);
					RawData64 |= (u64) (1LL << j);
					break;
				} else if (ret == E_RESULT_FATAL_ERR)
					return -1;
			}
		}
		score = autok_simple_score64(tune_result_str64, RawData64);
		AUTOK_DBGPRINT(AUTOK_DBG_RES, "[AUTOK]CMD %d \t %d \t %s\r\n", uCmdEdge, score,
			       tune_result_str64);
		if (autok_check_scan_res64(RawData64, pBdInfo) != 0) {
			host->autok_error = -1;
			return -1;
		}

		uCmdEdge ^= 0x1;
	} while (uCmdEdge);

	if (autok_pad_dly_sel(&uCmdDatInfo) == 0) {
		autok_param_update(CMD_EDGE, uCmdDatInfo.opt_edge_sel, p_autok_tune_res);
		autok_paddly_update(CMD_PAD_RDLY, uCmdDatInfo.opt_dly_cnt, p_autok_tune_res);
	} else {
		AUTOK_DBGPRINT(AUTOK_DBG_RES,
			       "[AUTOK][Error]=============Analysis Failed!!=======================\r\n");
	}

	/* Step2 : Tuning Data Path */
	autok_tuning_parameter_init(host, p_autok_tune_res);
	memset(&uCmdDatInfo, 0, sizeof(struct AUTOK_REF_INFO));

	uDatEdge = 0;
	do {
		pBdInfo = (struct AUTOK_SCAN_RES *)&(uCmdDatInfo.scan_info[uDatEdge]);
		msdc_autok_adjust_param(host, RD_FIFO_EDGE, &uDatEdge, AUTOK_WRITE);
		RawData64 = 0LL;
		for (j = 0; j < 64; j++) {
			msdc_autok_adjust_paddly(host, &j, DAT_PAD_RDLY);
			for (k = 0; k < AUTOK_CMD_TIMES / 2; k++) {
				ret = autok_send_tune_cmd(host, opcode, TUNE_DATA);
				if ((ret & (E_RESULT_CMD_TMO | E_RESULT_RSP_CRC)) != 0) {
					AUTOK_RAWPRINT
					    ("[AUTOK]Error Autok CMD Failed while tune Read\r\n");
					host->autok_error = -1;
					return -1;
				} else if ((ret & (E_RESULT_DAT_CRC | E_RESULT_DAT_TMO)) != 0) {
					RawData64 |= (u64) (1LL << j);
					break;
				} else if (ret == E_RESULT_FATAL_ERR)
					return -1;
			}
		}
		score = autok_simple_score64(tune_result_str64, RawData64);
		AUTOK_DBGPRINT(AUTOK_DBG_RES, "[AUTOK]DAT %d \t %d \t %s\r\n", uDatEdge, score,
			       tune_result_str64);
		if (autok_check_scan_res64(RawData64, pBdInfo) != 0) {
			host->autok_error = -1;
			return -1;
		}

		uDatEdge ^= 0x1;
	} while (uDatEdge);

	if (autok_pad_dly_sel(&uCmdDatInfo) == 0) {
		autok_param_update(RD_FIFO_EDGE, uCmdDatInfo.opt_edge_sel, p_autok_tune_res);
		autok_paddly_update(DAT_PAD_RDLY, uCmdDatInfo.opt_dly_cnt, p_autok_tune_res);
		autok_param_update(WD_FIFO_EDGE, uCmdDatInfo.opt_edge_sel, p_autok_tune_res);
	} else {
		AUTOK_DBGPRINT(AUTOK_DBG_RES,
			       "[AUTOK][Error]=============Analysis Failed!!=======================\r\n");
	}

	autok_tuning_parameter_init(host, p_autok_tune_res);

	/* Step3 : Tuning LATCH CK */

	opcode = MMC_SEND_TUNING_BLOCK;
	p_autok_tune_res[INT_DAT_LATCH_CK] = autok_execute_tuning_latch_ck(host, opcode,
		p_autok_tune_res[INT_DAT_LATCH_CK]);

	autok_result_dump(host, p_autok_tune_res);
#if AUTOK_PARAM_DUMP_ENABLE
	autok_register_dump(host);
#endif
	if (res != NULL) {
		memcpy((void *)res, (void *)p_autok_tune_res,
		       sizeof(p_autok_tune_res) / sizeof(u8));
	}
	host->autok_error = 0;

	return 0;
}


/********************************************************************
* Offline tune for SDIO/eMMC/SD                                     *
* Common Interface                                                  *
********************************************************************/


void autok_msdc_tx_setting(struct mt6797_autok_ctx *host, struct mmc_ios *ios)
{
	void __iomem *base = host->base;

	autok_set_field(MSDC_IOCON, MSDC_IOCON_DDR50CKD, AUTOK_MSDC_DDRCKD);
	if (ios->timing == MMC_TIMING_UHS_SDR104)
		autok_set_field(MSDC_PAD_TUNE0, MSDC_PAD_TUNE0_CLKTXDLY,
			       AUTOK_MSDC1_CLK_SDR104_TX_VALUE);
	else
		autok_set_field(MSDC_PAD_TUNE0, MSDC_PAD_TUNE0_CLKTXDLY,
			       AUTOK_MSDC1_CLK_TX_VALUE);
}


int autok_execute_tuning(struct mt6797_autok_ctx *host, u8 *res)
{
	int ret = 0;
	unsigned int clk_pwdn = 0;
	unsigned int int_en = 0;
	void __iomem *base = host->base;


	int_en = autok_read(MSDC_INTEN);
	autok_write(MSDC_INTEN, 0);
	clk_pwdn = autok_get_field(MSDC_CFG, MSDC_CFG_CKPDN);
	autok_set_field(MSDC_CFG, MSDC_CFG_CKPDN, 1);

	ret = execute_online_tuning(host, res);
	if (ret != 0)
		AUTOK_RAWPRINT("[AUTOK] ========Error: Autok Failed========");

	autok_msdc_reset();
	msdc_clear_fifo();
	autok_write(MSDC_INT, 0xffffffff);
	autok_write(MSDC_INTEN, int_en);
	autok_set_field(MSDC_CFG, MSDC_CFG_CKPDN, clk_pwdn);


	return ret;
}
