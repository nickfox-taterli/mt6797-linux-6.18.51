// SPDX-License-Identifier: GPL-2.0-only
/*
 * MT6797 MSDC1 SDR104 tuning engine for the X20 board.
 *
 * This is the board-specific rework of the vendor "AutoK" for MSDC1: the
 * same tuning operations, scan counts, latch checks and failure handling
 * that were verified on this board, expressed against the board's own
 * register map and the kernel's MMIO helpers.  The register sequence and
 * scan structure are hardware facts of the MT6797 MSDC1 SD path; the old
 * vendor multi-IP code paths (eMMC/SDIO/offline tuning) this board never
 * exercised are gone.  eMMC HS400 tuning stays in mtk-sd.c.
 *
 * The engine polls the controller directly; the caller holds the host IRQ
 * off and guarantees no MMC request is active.  MMIO uses raw 32-bit
 * accesses with a full barrier after each write, as the verified flow did.
 *
 * Scan sequence (all step boundaries are load-bearing for the final
 * register state):
 *   - INTEN=0, clock power up;
 *   - tune-path init, then a first apply of the zeroed result;
 *   - command-path scan over both sample edges, result update, apply;
 *   - data-path scan over both sample edges, result update, apply;
 *   - latch-CK candidate scan starting from the applied latch value;
 *   - restore INTEN and clock power regardless of the outcome.
 */
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sd.h>
#include <linux/string.h>
#include <linux/time.h>

#include "analysis.h"
#include "mt6797-sd-autok.h"
#include "regs.h"

/* Scan diagnostics follow the analysis layer: debug-only, off by default. */
#define AUTOK_DBG_RES	2
#define AUTOK_DBGPRINT(_level, _fmt, ...) \
	do { if (_level <= AUTOK_DBG_RES) pr_debug(_fmt, ##__VA_ARGS__); } while (0)

/* Scan and latch-CK retry structure of the verified flow. */
#define AUTOK_CMD_TIMES		20	/* per-edge attempts, split in half */
#define AUTOK_LATCH_CK_TIMES	3	/* tuning blocks per latch candidate */
#define AUTOK_LATCH_CK_MAX	8	/* INT_DAT_LATCH_CK candidates */
#define SDAUTOK_TUNING_BLOCK_SZ	64	/* CMD19 returns 64 bytes */
#define SDAUTOK_CMD_TIMEOUT	(HZ / 10)
#define SDAUTOK_DATA_TIMEOUT	(3 * HZ)
#define SDAUTOK_FIFO_SZ		128	/* RX FIFO watermark for the drain */

/* Latch-enable and turn-around values fixed by the verified setup. */
#define AUTOK_LATCH_EN_HS	2
#define AUTOK_LATCH_EN_SDR104	4
#define AUTOK_CMD_TA_CNTR	0
#define AUTOK_CRC_TA_CNTR	0
#define AUTOK_BUSY_MA		1
#define AUTOK_CRC_MA		1

/* Pad delay groups of the result record. */
enum {
	CMD_PAD_RDLY,
	DAT_PAD_RDLY,
	DS_PAD_RDLY,
};

/* Scan command purpose: bare command (CMD19 response only), data read
 * (CMD19 with the 64-byte tuning block), or latch-CK burst (block count
 * from ctx->latch_blk_cnt, FIFO kept across commands). */
enum sdautok_tune_type {
	TUNE_CMD,
	TUNE_DATA,
	TUNE_LATCH_CK,
};

/* --- MMIO helpers: raw 32-bit access, full barrier after every write --- */

static u32 autok_read(const struct mt6797_sdautok_ctx *ctx, unsigned int reg)
{
	return __raw_readl(ctx->base + reg);
}

static void autok_write(const struct mt6797_sdautok_ctx *ctx, unsigned int reg,
			u32 value)
{
	__raw_writel(value, ctx->base + reg);
	mb();
}

static void autok_set_bits(const struct mt6797_sdautok_ctx *ctx,
			   unsigned int reg, u32 bits)
{
	autok_write(ctx, reg, autok_read(ctx, reg) | bits);
}

static void autok_clear_bits(const struct mt6797_sdautok_ctx *ctx,
			     unsigned int reg, u32 bits)
{
	autok_write(ctx, reg, autok_read(ctx, reg) & ~bits);
}

static void autok_set_field(const struct mt6797_sdautok_ctx *ctx,
			    unsigned int reg, u32 mask, u32 value)
{
	autok_write(ctx, reg, (autok_read(ctx, reg) & ~mask) |
		    (value << __ffs(mask)));
}

static u32 autok_get_field(const struct mt6797_sdautok_ctx *ctx,
			   unsigned int reg, u32 mask)
{
	return (autok_read(ctx, reg) & mask) >> __ffs(mask);
}

/* Bounded wait for the controller to leave reset / clear the FIFO. */
static void autok_poll_clear(const struct mt6797_sdautok_ctx *ctx,
			     unsigned int reg, u32 bits)
{
	int retry = 5, count = 1000;

	while (retry && (autok_read(ctx, reg) & bits)) {
		if (count-- == 0) {
			retry--;
			count = 1000;
		}
	}
	WARN_ON(!retry);
}

static void autok_msdc_reset(const struct mt6797_sdautok_ctx *ctx)
{
	autok_set_bits(ctx, SDAUTOK_MSDC_CFG, MSDC_CFG_RST);
	/* The reset is asynchronous; make the write visible before polling. */
	mb();
	autok_poll_clear(ctx, SDAUTOK_MSDC_CFG, MSDC_CFG_RST);
}

static void autok_clear_fifo(const struct mt6797_sdautok_ctx *ctx)
{
	autok_set_bits(ctx, SDAUTOK_MSDC_FIFOCS, MSDC_FIFOCS_CLR);
	mb();
	autok_poll_clear(ctx, SDAUTOK_MSDC_FIFOCS, MSDC_FIFOCS_CLR);
}

/* Bounded poll on an interrupt status; returns the last status read, 0 on
 * timeout.  The returned value is the one the caller acknowledges. */
static u32 autok_wait_int(const struct mt6797_sdautok_ctx *ctx, u32 wanted,
			  unsigned long timeout)
{
	unsigned long deadline = jiffies + timeout;
	u32 sts;

	while (1) {
		sts = autok_read(ctx, SDAUTOK_MSDC_INT);
		if (sts & wanted)
			return sts;
		if (!timeout || time_after(jiffies, deadline))
			return 0;
	}
}

/* Bounded wait for the controller to go idle; false on timeout. */
static bool autok_wait_idle(const struct mt6797_sdautok_ctx *ctx,
			    unsigned long timeout)
{
	unsigned long deadline = jiffies + timeout;

	while (autok_read(ctx, SDAUTOK_SDC_STS) & SDC_STS_SDCBUSY) {
		if (!timeout || time_after(jiffies, deadline))
			return false;
	}
	return true;
}

/* --- Tuned-parameter register table -----------------------------------
 * reg/mask are MT6797 MSDC1 hardware facts; max is the field width.  The
 * CKGEN delay write needs a settling delay before the next access.
 * ---------------------------------------------------------------------- */
enum {
	PREG_IOCON,
	PREG_PAD_TUNE0,
	PREG_PAD_TUNE1,
	PREG_PATCH0,
	PREG_PATCH1,
	PREG_PATCH2,
	PREG_EMMC50_CFG0,
	PREG_DS_TUNE,
};

static const u16 param_reg_base[] = {
	[PREG_IOCON]		= SDAUTOK_MSDC_IOCON,
	[PREG_PAD_TUNE0]	= SDAUTOK_MSDC_PAD_TUNE0,
	[PREG_PAD_TUNE1]	= SDAUTOK_MSDC_PAD_TUNE1,
	[PREG_PATCH0]		= SDAUTOK_MSDC_PATCH_BIT0,
	[PREG_PATCH1]		= SDAUTOK_MSDC_PATCH_BIT1,
	[PREG_PATCH2]		= SDAUTOK_MSDC_PATCH_BIT2,
	[PREG_EMMC50_CFG0]	= SDAUTOK_EMMC50_CFG0,
	[PREG_DS_TUNE]		= SDAUTOK_EMMC50_PAD_DS_TUNE,
};

struct sdautok_param {
	u8 preg;
	u32 mask;
	u8 max;
	bool settle;	/* writes need a 1 ms settling delay */
};

static const struct sdautok_param sdautok_params[] = {
	/* Result record: indices follow enum mt6797_sdautok_result. */
	[AUTOK_CMD_EDGE]		= { PREG_IOCON, MSDC_IOCON_RSPL, 1 },
	[AUTOK_RDATA_EDGE]		= { PREG_IOCON, MSDC_IOCON_R_D_SMPL, 1 },
	[AUTOK_RD_FIFO_EDGE]		= { PREG_PATCH0, MSDC_PB0_RD_DAT_SEL, 1 },
	[AUTOK_WD_FIFO_EDGE]		= { PREG_PATCH2, MSDC_PB2_CFGCRCSTSEDGE, 1 },
	[AUTOK_CMD_RD_D_DLY1]		= { PREG_PAD_TUNE0, MSDC_PAD_TUNE0_CMDRDLY, 31 },
	[AUTOK_CMD_RD_D_DLY1_SEL]	= { PREG_PAD_TUNE0, MSDC_PAD_TUNE0_CMDRRDLYSEL, 1 },
	[AUTOK_CMD_RD_D_DLY2]		= { PREG_PAD_TUNE1, MSDC_PAD_TUNE1_CMDRDLY2, 31 },
	[AUTOK_CMD_RD_D_DLY2_SEL]	= { PREG_PAD_TUNE1, MSDC_PAD_TUNE1_CMDRRDLY2SEL, 1 },
	[AUTOK_DAT_RD_DLY1]		= { PREG_PAD_TUNE0, MSDC_PAD_TUNE0_DATRRDLY, 31 },
	[AUTOK_DAT_RD_DLY1_SEL]		= { PREG_PAD_TUNE0, MSDC_PAD_TUNE0_DATRRDLYSEL, 1 },
	[AUTOK_DAT_RD_DLY2]		= { PREG_PAD_TUNE1, MSDC_PAD_TUNE1_DATRRDLY2, 31 },
	[AUTOK_DAT_RD_DLY2_SEL]		= { PREG_PAD_TUNE1, MSDC_PAD_TUNE1_DATRRDLY2SEL, 1 },
	[AUTOK_INT_DAT_LATCH_CK]	= { PREG_PATCH0, MSDC_PB0_INT_DAT_LATCH_CK_SEL, 7 },
	[AUTOK_EMMC50_DS_Z_DLY1]	= { PREG_DS_TUNE, MSDC_EMMC50_DS_DLY1, 31 },
	[AUTOK_EMMC50_DS_Z_DLY1_SEL]	= { PREG_DS_TUNE, MSDC_EMMC50_DS_DLYSEL, 1 },
	[AUTOK_EMMC50_DS_Z_DLY2]	= { PREG_DS_TUNE, MSDC_EMMC50_DS_DLY2, 31 },
	[AUTOK_EMMC50_DS_Z_DLY2_SEL]	= { PREG_DS_TUNE, MSDC_EMMC50_DS_DLY2SEL, 1 },
	[AUTOK_EMMC50_DS_ZDLY_DLY]	= { PREG_DS_TUNE, MSDC_EMMC50_DS_DLY3, 31 },
	/* Path parameters: fixed by the tune-path setup, never scanned. */
	[AUTOK_PATH_READ_DATA_SMPL_SEL]	= { PREG_IOCON, MSDC_IOCON_R_D_SMPL_SEL, 1 },
	[AUTOK_PATH_WRITE_DATA_SMPL_SEL] = { PREG_IOCON, MSDC_IOCON_W_D_SMPL_SEL, 1 },
	[AUTOK_PATH_DATA_DLYLINE_SEL]	= { PREG_IOCON, MSDC_IOCON_DDLSEL, 1 },
	[AUTOK_PATH_DAT_TUNE_SEL]	= { PREG_PAD_TUNE0, MSDC_PAD_TUNE0_RXDLYSEL, 1 },
	[AUTOK_PATH_WCRC_ASYNC_FIFO_SEL] = { PREG_PATCH2, MSDC_PB2_CFGCRCSTS, 1 },
	[AUTOK_PATH_RESP_ASYNC_FIFO_SEL] = { PREG_PATCH2, MSDC_PB2_CFGRESP, 1 },
	[AUTOK_PATH_EMMC50_WDATA_MUX_EN] = { PREG_EMMC50_CFG0, MSDC_EMMC50_CFG_CRC_STS_SEL, 1 },
	[AUTOK_PATH_EMMC50_CMD_MUX_EN]	= { PREG_EMMC50_CFG0, MSDC_EMMC50_CFG_CMD_RESP_SEL, 1 },
	[AUTOK_PATH_EMMC50_WDATA_EDGE]	= { PREG_EMMC50_CFG0, MSDC_EMMC50_CFG_CRC_STS_EDGE, 1 },
	[AUTOK_PATH_CKGEN_MSDC_DLY_SEL]	= { PREG_PATCH0, MSDC_PB0_CKGEN_MSDC_DLY_SEL, 31, true },
	[AUTOK_PATH_CMD_RSP_TA_CNTR]	= { PREG_PATCH1, MSDC_PB1_CMD_RSP_TA_CNTR, 7 },
	[AUTOK_PATH_WRDAT_CRCS_TA_CNTR]	= { PREG_PATCH1, MSDC_PB1_WRDAT_CRCS_TA_CNTR, 7 },
	[AUTOK_PATH_PAD_CLK_TXDLY]	= { PREG_PAD_TUNE0, MSDC_PAD_TUNE0_CLKTXDLY, 31 },
};

static const u8 sdautok_result_max[] = {
	[AUTOK_CMD_EDGE] = 1,
	[AUTOK_RDATA_EDGE] = 1,
	[AUTOK_RD_FIFO_EDGE] = 1,
	[AUTOK_WD_FIFO_EDGE] = 1,
	[AUTOK_CMD_RD_D_DLY1] = 31,
	[AUTOK_CMD_RD_D_DLY1_SEL] = 1,
	[AUTOK_CMD_RD_D_DLY2] = 31,
	[AUTOK_CMD_RD_D_DLY2_SEL] = 1,
	[AUTOK_DAT_RD_DLY1] = 31,
	[AUTOK_DAT_RD_DLY1_SEL] = 1,
	[AUTOK_DAT_RD_DLY2] = 31,
	[AUTOK_DAT_RD_DLY2_SEL] = 1,
	[AUTOK_INT_DAT_LATCH_CK] = 7,
	[AUTOK_EMMC50_DS_Z_DLY1] = 31,
	[AUTOK_EMMC50_DS_Z_DLY1_SEL] = 1,
	[AUTOK_EMMC50_DS_Z_DLY2] = 31,
	[AUTOK_EMMC50_DS_Z_DLY2_SEL] = 1,
	[AUTOK_EMMC50_DS_ZDLY_DLY] = 31,
};

static void sdautok_write_param(const struct mt6797_sdautok_ctx *ctx,
				unsigned int param, u32 value)
{
	const struct sdautok_param *p = &sdautok_params[param];

	autok_set_field(ctx, param_reg_base[p->preg], p->mask, value);
	if (p->settle)
		mdelay(1);
}

/* Store a scan outcome in the result record, rejecting out-of-range hits. */
static int sdautok_result_update(u8 *result, unsigned int param, u32 value)
{
	if (value > sdautok_result_max[param]) {
		pr_debug("[AUTOK]param out of range: %u > %u\n",
			 value, sdautok_result_max[param]);
		return -1;
	}
	result[param] = value;
	return 0;
}

/* Write every result-record parameter to its register. */
static void sdautok_result_apply(const struct mt6797_sdautok_ctx *ctx,
				 const u8 *result)
{
	unsigned int i;

	for (i = 0; i < MT6797_SDAUTOK_RESULT_SIZE; i++)
		sdautok_write_param(ctx, i, result[i]);
}

/* Split a 0..63 delay across the two-stage delay line and record it. */
static void sdautok_result_paddly(u8 *result, unsigned int pad,
				  unsigned int delay)
{
	unsigned int lo = min(delay, 31u);
	unsigned int hi = (delay > 31) ? delay - 32 : 0;
	unsigned int lo_sel = lo ? 1 : 0;
	unsigned int hi_sel = hi ? 1 : 0;
	unsigned int base = (pad == CMD_PAD_RDLY) ? AUTOK_CMD_RD_D_DLY1 :
			    (pad == DAT_PAD_RDLY) ? AUTOK_DAT_RD_DLY1 :
						    AUTOK_EMMC50_DS_Z_DLY1;

	sdautok_result_update(result, base, lo);
	sdautok_result_update(result, base + 2, hi);
	sdautok_result_update(result, base + 1, lo_sel);
	sdautok_result_update(result, base + 3, hi_sel);
}

/* Same split, written straight to the controller during a scan. */
static void sdautok_write_paddly(const struct mt6797_sdautok_ctx *ctx,
				 unsigned int pad, unsigned int delay)
{
	unsigned int lo = min(delay, 31u);
	unsigned int hi = (delay > 31) ? delay - 32 : 0;
	unsigned int lo_sel = lo ? 1 : 0;
	unsigned int hi_sel = hi ? 1 : 0;
	unsigned int base = (pad == CMD_PAD_RDLY) ? AUTOK_CMD_RD_D_DLY1 :
			    (pad == DAT_PAD_RDLY) ? AUTOK_DAT_RD_DLY1 :
						    AUTOK_EMMC50_DS_Z_DLY1;

	sdautok_write_param(ctx, base, lo);
	sdautok_write_param(ctx, base + 2, hi);
	sdautok_write_param(ctx, base + 1, lo_sel);
	sdautok_write_param(ctx, base + 3, hi_sel);
}

/* --- Tuning command ---------------------------------------------------- */

static u32 sdautok_send_tune_cmd(struct mt6797_sdautok_ctx *ctx,
				 enum sdautok_tune_type type)
{
	u32 rawcmd = (SDAUTOK_TUNING_BLOCK_SZ << 16) | BIT(11) | BIT(7) | 19;
	unsigned long deadline;
	u32 sts, status = AUTOK_ERR_NONE;

	/* CMD19 (SEND_TUNING_BLOCK); the latch-CK burst carries its own
	 * block count, everything else reads one tuning block. */
	autok_write(ctx, SDAUTOK_SDC_BLK_NUM,
		    (type == TUNE_LATCH_CK) ? ctx->latch_blk_cnt : 1);

	if (!autok_wait_idle(ctx, SDAUTOK_DATA_TIMEOUT)) {
		pr_debug("[AUTOK]MSDC busy timeout before cmd19\n");
		return AUTOK_ERR_FATAL;
	}

	/* Reset and clear FIFO so each scan point starts from a known state.
	 * The latch-CK scan deliberately keeps the FIFO across commands. */
	if (type != TUNE_LATCH_CK) {
		autok_msdc_reset(ctx);
		autok_clear_fifo(ctx);
		autok_write(ctx, SDAUTOK_MSDC_INT, 0xffffffff);
	}

	autok_write(ctx, SDAUTOK_SDC_ARG, 0);
	autok_write(ctx, SDAUTOK_SDC_CMD, rawcmd);

	sts = autok_wait_int(ctx, MSDC_INT_CMD_DONE, SDAUTOK_CMD_TIMEOUT);
	if (!sts) {
		pr_debug("[AUTOK]CMD19 wait int timeout\n");
		return AUTOK_ERR_CMD_TMO;
	}

	autok_write(ctx, SDAUTOK_MSDC_INT, sts & MSDC_INT_CMD_DONE);

	if (sts & MSDC_INT_CMDRDY) {
		if (type == TUNE_CMD)
			return AUTOK_ERR_NONE;
	} else if (sts & MSDC_INT_RSPCRCERR) {
		return AUTOK_ERR_RSP_CRC;
	} else if (sts & MSDC_INT_CMDTMO) {
		return AUTOK_ERR_CMD_TMO;
	}

	/* The data phase is running: wait it out, keeping the RX FIFO from
	 * stalling during latch-CK bursts. */
	deadline = jiffies + SDAUTOK_DATA_TIMEOUT;
	while (autok_read(ctx, SDAUTOK_SDC_STS) & SDC_STS_SDCBUSY) {
		if (time_after(jiffies, deadline)) {
			pr_debug("[AUTOK]MSDC busy timeout after cmd19\n");
			return AUTOK_ERR_FATAL;
		}
		if (type == TUNE_LATCH_CK &&
		    autok_get_field(ctx, SDAUTOK_MSDC_FIFOCS,
				    MSDC_FIFOCS_RXCNT) >= SDAUTOK_FIFO_SZ) {
			autok_read(ctx, SDAUTOK_MSDC_RXDATA);
			autok_read(ctx, SDAUTOK_MSDC_RXDATA);
			autok_read(ctx, SDAUTOK_MSDC_RXDATA);
			autok_read(ctx, SDAUTOK_MSDC_RXDATA);
		}
	}

	sts = autok_read(ctx, SDAUTOK_MSDC_INT);
	if (sts) {
		autok_write(ctx, SDAUTOK_MSDC_INT, sts & MSDC_INT_DATA_DONE);
		/* Priority as the verified flow reports it: timeout wins
		 * over CRC error, which wins over completion. */
		if (sts & MSDC_INT_XFER_COMPL)
			status = AUTOK_ERR_NONE;
		if (sts & MSDC_INT_DATCRCERR)
			status = AUTOK_ERR_DAT_CRC;
		if (sts & MSDC_INT_DATTMO)
			status = AUTOK_ERR_DAT_TMO;
	}

	if (!autok_wait_idle(ctx, SDAUTOK_DATA_TIMEOUT)) {
		pr_debug("[AUTOK]MSDC busy timeout tail cmd19\n");
		return AUTOK_ERR_FATAL;
	}
	if (type != TUNE_LATCH_CK)
		autok_clear_fifo(ctx);

	return status;
}

/* --- Path and timing setup --------------------------------------------- */

/* Path selection shared by the execute and replay paths: shared sample
 * edge/delay line, async FIFO selection and the common turn-around and
 * margin settings of the verified SD configuration. */
static void sdautok_path_select(const struct mt6797_sdautok_ctx *ctx)
{
	/* Shared sample edge and delay line for every data line. */
	sdautok_write_param(ctx, AUTOK_PATH_READ_DATA_SMPL_SEL, 0);
	sdautok_write_param(ctx, AUTOK_PATH_WRITE_DATA_SMPL_SEL, 0);
	sdautok_write_param(ctx, AUTOK_PATH_DATA_DLYLINE_SEL, 0);
	sdautok_write_param(ctx, AUTOK_PATH_DAT_TUNE_SEL, 0);

	/* Async FIFO for the write-CRC path, sync response path. */
	sdautok_write_param(ctx, AUTOK_PATH_WCRC_ASYNC_FIFO_SEL, 1);
	sdautok_write_param(ctx, AUTOK_PATH_RESP_ASYNC_FIFO_SEL, 0);

	/* eMMC50 muxes off: the SD slot runs the 4.5 IP paths. */
	sdautok_write_param(ctx, AUTOK_PATH_EMMC50_WDATA_MUX_EN, 0);
	sdautok_write_param(ctx, AUTOK_PATH_EMMC50_CMD_MUX_EN, 0);
	sdautok_write_param(ctx, AUTOK_PATH_EMMC50_WDATA_EDGE, 0);

	/* Common settings of the verified SD configuration. */
	sdautok_write_param(ctx, AUTOK_PATH_CKGEN_MSDC_DLY_SEL, 0);
	sdautok_write_param(ctx, AUTOK_PATH_CMD_RSP_TA_CNTR, AUTOK_CMD_TA_CNTR);
	sdautok_write_param(ctx, AUTOK_PATH_WRDAT_CRCS_TA_CNTR, AUTOK_CRC_TA_CNTR);

	autok_set_field(ctx, SDAUTOK_MSDC_PATCH_BIT1, MSDC_PB1_GET_BUSY_MA,
			AUTOK_BUSY_MA);
	autok_set_field(ctx, SDAUTOK_MSDC_PATCH_BIT1, MSDC_PB1_GET_CRC_MA,
			AUTOK_CRC_MA);
}

/* Tune-path baseline for a fresh tuning run.  This is the SD (4.5 IP)
 * path of MSDC1; the HS400 alternates in the old code belongs to MSDC0
 * eMMC, which mtk-sd.c tunes itself. */
void sdautok_tune_path_init(const struct mt6797_sdautok_ctx *ctx)
{
	/* Start the pad tuning registers from the SD path baseline. */
	autok_write(ctx, SDAUTOK_MSDC_PAD_TUNE0, 0);
	autok_clear_bits(ctx, SDAUTOK_MSDC_IOCON, MSDC_IOCON_DDLSEL);
	autok_clear_bits(ctx, SDAUTOK_MSDC_IOCON, MSDC_IOCON_R_D_SMPL_SEL);
	autok_clear_bits(ctx, SDAUTOK_MSDC_IOCON, MSDC_IOCON_R_D_SMPL);
	autok_set_bits(ctx, SDAUTOK_MSDC_PAD_TUNE0, MSDC_PAD_TUNE0_DATRRDLYSEL);
	autok_clear_bits(ctx, SDAUTOK_MSDC_PAD_TUNE1, MSDC_PAD_TUNE1_DATRRDLY2SEL);
	autok_set_bits(ctx, SDAUTOK_MSDC_PATCH_BIT2, MSDC_PB2_CFGCRCSTS);
	autok_clear_bits(ctx, SDAUTOK_MSDC_IOCON, MSDC_IOCON_W_D_SMPL_SEL);

	autok_clear_bits(ctx, SDAUTOK_MSDC_PATCH_BIT2, MSDC_PB2_CFGRESP);
	autok_set_bits(ctx, SDAUTOK_MSDC_PAD_TUNE0, MSDC_PAD_TUNE0_CMDRRDLYSEL);
	autok_clear_bits(ctx, SDAUTOK_MSDC_PAD_TUNE1, MSDC_PAD_TUNE1_CMDRRDLY2SEL);
	autok_clear_bits(ctx, SDAUTOK_EMMC50_CFG0, MSDC_EMMC50_CFG_CMD_RESP_SEL);

	sdautok_path_select(ctx);
}

static void sdautok_timing_init(struct mt6797_sdautok_ctx *ctx)
{
	u32 latch_en = (ctx->sclk <= 100000000) ? AUTOK_LATCH_EN_HS :
						  AUTOK_LATCH_EN_SDR104;

	/* The data tune path may have been changed between sessions. */
	sdautok_path_select(ctx);

	autok_set_field(ctx, SDAUTOK_MSDC_PATCH_BIT2, MSDC_PB2_CRCSTSENSEL,
			latch_en);
	autok_set_field(ctx, SDAUTOK_MSDC_PATCH_BIT2, MSDC_PB2_RESPSTENSEL,
			latch_en);
}

void mt6797_sdautok_tx_setting(struct mt6797_sdautok_ctx *ctx, bool sdr104)
{
	/* The board's SDR104 and slower modes share a 0 TX delay; DDR50 uses
	 * the internal clock division. */
	autok_set_field(ctx, SDAUTOK_MSDC_IOCON, MSDC_IOCON_DDR50CKD, 0);
	autok_set_field(ctx, SDAUTOK_MSDC_PAD_TUNE0, MSDC_PAD_TUNE0_CLKTXDLY, 0);
}

/* --- Tuning execution --------------------------------------------------- */

/* Scan one pad delay range over both sample edges; mark failures per delay.
 *
 * bad_mask errors record the delay as failed (command timeouts recover via
 * a controller reset when reset_on_cmd_tmo is set); fail_mask errors abort
 * the whole tuning.  Returns 0 with *opt_edge filled when the analysis
 * selected a window, 1 when the window analysis declined (the verified
 * flow keeps the previous settings then), -EIO on fatal errors.
 */
static int sdautok_scan_edge(struct mt6797_sdautok_ctx *ctx, u8 *result,
			     unsigned int param, unsigned int pad,
			     enum sdautok_tune_type type,
			     u32 bad_mask, u32 fail_mask,
			     bool reset_on_cmd_tmo,
			     unsigned int *opt_edge)
{
	struct AUTOK_REF_INFO info;
	struct AUTOK_SCAN_RES *scan;
	u64 failures;
	u64 score;
	unsigned int edge = 0;
	unsigned int i, k;
	int ret;

	memset(&info, 0, sizeof(info));

	do {
		char text[65];

		scan = &info.scan_info[edge];
		sdautok_write_param(ctx, param, edge);
		failures = 0;

		for (i = 0; i < 64; i++) {
			sdautok_write_paddly(ctx, pad, i);
			for (k = 0; k < AUTOK_CMD_TIMES / 2; k++) {
				ret = sdautok_send_tune_cmd(ctx, type);
				if (ret & bad_mask) {
					if ((ret & AUTOK_ERR_CMD_TMO) &&
					    reset_on_cmd_tmo) {
						autok_msdc_reset(ctx);
						autok_clear_fifo(ctx);
						autok_write(ctx, SDAUTOK_MSDC_INT,
							    0xffffffff);
					}
					failures |= BIT_ULL(i);
					break;
				}
				if (ret & fail_mask) {
					ctx->error = -1;
					return -EIO;
				}
			}
		}

		score = autok_simple_score64(text, failures);
		AUTOK_DBGPRINT(AUTOK_DBG_RES, "[AUTOK]edge %u score %llu %s\n",
			       edge, score, text);
		if (autok_check_scan_res64(failures, scan) != 0) {
			ctx->error = -1;
			return -EIO;
		}

		edge ^= 0x1;
	} while (edge);

	if (autok_pad_dly_sel(&info) != 0) {
		AUTOK_DBGPRINT(AUTOK_DBG_RES, "[AUTOK]analysis failed\n");
		return 1;
	}

	sdautok_result_update(result, param, info.opt_edge_sel);
	sdautok_result_paddly(result, pad, info.opt_dly_cnt);
	*opt_edge = info.opt_edge_sel;

	return 0;
}

/* Online latch-CK search: burst tuning blocks per candidate, first stable
 * candidate wins.  Returns the chosen latch value. */
static unsigned int sdautok_tune_latch_ck(struct mt6797_sdautok_ctx *ctx,
					  unsigned int initial)
{
	unsigned int j, k;
	int ret = 0;

	autok_write(ctx, SDAUTOK_MSDC_INT, 0xffffffff);

	for (j = initial; j < AUTOK_LATCH_CK_MAX; j += ctx->hclk / ctx->sclk) {
		ctx->latch_blk_cnt = 0;
		autok_clear_fifo(ctx);
		autok_set_field(ctx, SDAUTOK_MSDC_PATCH_BIT0,
				MSDC_PB0_INT_DAT_LATCH_CK_SEL, j);
		for (k = 0; k < AUTOK_LATCH_CK_TIMES; k++) {
			/* SD latch-CK burst: one tuning block per attempt. */
			ctx->latch_blk_cnt = 1;
			ret = sdautok_send_tune_cmd(ctx, TUNE_LATCH_CK);
			if (ret & (AUTOK_ERR_CMD_TMO | AUTOK_ERR_RSP_CRC)) {
				pr_debug("[AUTOK]CMD failed during latch-CK scan\n");
				break;
			}
			if (ret & (AUTOK_ERR_DAT_CRC | AUTOK_ERR_DAT_TMO)) {
				pr_debug("[AUTOK]latch-CK data error at %u\n", j);
				break;
			}
		}
		if (ret == 0) {
			autok_set_field(ctx, SDAUTOK_MSDC_PATCH_BIT0,
					MSDC_PB0_INT_DAT_LATCH_CK_SEL, j);
			break;
		}
	}
	ctx->latch_blk_cnt = 0;

	return j;
}

static void sdautok_build_checks(void)
{
	BUILD_BUG_ON(ARRAY_SIZE(sdautok_result_max) !=
		     MT6797_SDAUTOK_RESULT_SIZE);
	BUILD_BUG_ON(ARRAY_SIZE(sdautok_params) <
		     ARRAY_SIZE(sdautok_result_max));
}

int mt6797_sdautok_execute(struct mt6797_sdautok_ctx *ctx, u8 *result)
{
	unsigned int opt_edge = 0;
	unsigned int latch;
	int ret;
	u32 int_en, clk_pwdn;

	sdautok_build_checks();

	/* Save the request-path state; tuning polls the controller directly. */
	int_en = autok_read(ctx, SDAUTOK_MSDC_INTEN);
	autok_write(ctx, SDAUTOK_MSDC_INTEN, 0);
	clk_pwdn = autok_get_field(ctx, SDAUTOK_MSDC_CFG, MSDC_CFG_CKPDN);
	autok_set_field(ctx, SDAUTOK_MSDC_CFG, MSDC_CFG_CKPDN, 1);

	memset(result, 0, MT6797_SDAUTOK_RESULT_SIZE);
	sdautok_timing_init(ctx);
	sdautok_result_apply(ctx, result);

	/* Step 1: command response path.  A command timeout is recoverable
	 * (the scan resets the controller); anything else fails the tuning. */
	ret = sdautok_scan_edge(ctx, result, AUTOK_CMD_EDGE, CMD_PAD_RDLY,
				TUNE_CMD,
				AUTOK_ERR_RSP_CRC | AUTOK_ERR_CMD_TMO,
				AUTOK_ERR_FATAL, true, &opt_edge);
	if (ret < 0)
		goto out;

	/* The command-path selection feeds the data scan; apply it even when
	 * the window analysis declined, as the verified flow does. */
	sdautok_result_apply(ctx, result);

	/* Step 2: data read path.  A command failure here is fatal. */
	ret = sdautok_scan_edge(ctx, result, AUTOK_RD_FIFO_EDGE, DAT_PAD_RDLY,
				TUNE_DATA,
				AUTOK_ERR_DAT_CRC | AUTOK_ERR_DAT_TMO,
				AUTOK_ERR_CMD_TMO | AUTOK_ERR_RSP_CRC |
				AUTOK_ERR_FATAL, false, &opt_edge);
	if (ret < 0)
		goto out;

	/* Both write-path selections share the chosen read FIFO edge. */
	if (ret == 0)
		sdautok_result_update(result, AUTOK_WD_FIFO_EDGE, opt_edge);
	sdautok_result_apply(ctx, result);

	/* Step 3: internal data latch clock, started from the applied value. */
	latch = result[AUTOK_INT_DAT_LATCH_CK];
	result[AUTOK_INT_DAT_LATCH_CK] = sdautok_tune_latch_ck(ctx, latch);
	ret = 0;
	ctx->error = 0;

out:
	if (ret)
		pr_debug("[AUTOK]tuning failed\n");

	/* Restore the request path regardless of the outcome. */
	autok_msdc_reset(ctx);
	autok_clear_fifo(ctx);
	autok_write(ctx, SDAUTOK_MSDC_INT, 0xffffffff);
	autok_write(ctx, SDAUTOK_MSDC_INTEN, int_en);
	autok_set_field(ctx, SDAUTOK_MSDC_CFG, MSDC_CFG_CKPDN, clk_pwdn);

	return ret;
}

void mt6797_sdautok_apply(struct mt6797_sdautok_ctx *ctx, const u8 *result)
{
	sdautok_timing_init(ctx);
	sdautok_result_apply(ctx, result);
}

int mt6797_sdautok_run(struct mt6797_sdautok_ctx *ctx, u8 *result)
{
	u32 pio;
	int ret;

	sdautok_build_checks();

	/* No MMC request is active: isolate PIO around the whole flow. */
	pio = autok_get_field(ctx, SDAUTOK_MSDC_CFG, MSDC_CFG_PIO);
	autok_set_field(ctx, SDAUTOK_MSDC_CFG, MSDC_CFG_PIO, 1);
	sdautok_tune_path_init(ctx);
	mt6797_sdautok_tx_setting(ctx, true);
	ret = mt6797_sdautok_execute(ctx, result);
	autok_set_field(ctx, SDAUTOK_MSDC_CFG, MSDC_CFG_PIO, pio);

	return ret;
}
