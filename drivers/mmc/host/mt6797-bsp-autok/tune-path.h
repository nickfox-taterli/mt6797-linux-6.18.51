/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (C) 2015 MediaTek Inc.
 * Verbatim msdc_init_tune_path() from the MT6797 BSP msdc_tune.c.
 */
static void msdc_init_tune_path(struct mt6797_autok_ctx *host, unsigned char timing)
{
	void __iomem *base = host->base;

	autok_write(MSDC_PAD_TUNE0,   0x00000000);

	autok_clear_bits(MSDC_IOCON, MSDC_IOCON_DDLSEL);
	autok_clear_bits(MSDC_IOCON, MSDC_IOCON_R_D_SMPL_SEL);
	autok_clear_bits(MSDC_IOCON, MSDC_IOCON_R_D_SMPL);
	if (timing == MMC_TIMING_MMC_HS400) {
		autok_clear_bits(MSDC_PAD_TUNE0, MSDC_PAD_TUNE0_DATRRDLYSEL);
		autok_clear_bits(MSDC_PAD_TUNE1, MSDC_PAD_TUNE1_DATRRDLY2SEL);
	} else {
		autok_set_bits(MSDC_PAD_TUNE0, MSDC_PAD_TUNE0_DATRRDLYSEL);
		autok_clear_bits(MSDC_PAD_TUNE1, MSDC_PAD_TUNE1_DATRRDLY2SEL);
	}

	if (timing == MMC_TIMING_MMC_HS400)
		autok_clear_bits(MSDC_PATCH_BIT2, MSDC_PB2_CFGCRCSTS);
	else
		autok_set_bits(MSDC_PATCH_BIT2, MSDC_PB2_CFGCRCSTS);
	autok_clear_bits(MSDC_IOCON, MSDC_IOCON_W_D_SMPL_SEL);

	autok_clear_bits(MSDC_PATCH_BIT2, MSDC_PB2_CFGRESP);
	autok_set_bits(MSDC_PAD_TUNE0, MSDC_PAD_TUNE0_CMDRRDLYSEL);
	autok_clear_bits(MSDC_PAD_TUNE1, MSDC_PAD_TUNE1_CMDRRDLY2SEL);

	autok_clear_bits(EMMC50_CFG0, MSDC_EMMC50_CFG_CMD_RESP_SEL);
	/* tune path and related key hw fixed setting should be wrappered follow interface */
	autok_path_sel(host);
}
