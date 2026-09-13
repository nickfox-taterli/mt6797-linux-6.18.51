// SPDX-License-Identifier: GPL-2.0
/*
 * Based on pinctrl-mt6765.c
 *
 * Copyright (C) 2018 MediaTek Inc.
 *
 * Author: ZH Chen <zh.chen@mediatek.com>
 *
 * Copyright (C) Manivannan Sadhasivam <manivannan.sadhasivam@linaro.org>
 *
 */

#include "pinctrl-mtk-mt6797.h"
#include "pinctrl-paris.h"

/*
 * MT6797 have multiple bases to program pin configuration listed as the below:
 * gpio:0x10005000, iocfg[l]:0x10002000, iocfg[b]:0x10002400,
 * iocfg[r]:0x10002800, iocfg[t]:0x10002C00.
 * _i_base could be used to indicate what base the pin should be mapped into.
 */

#define PIN_FIELD_BASE(s_pin, e_pin, i_base, s_addr, x_addrs, s_bit, x_bits) \
	PIN_FIELD_CALC(s_pin, e_pin, i_base, s_addr, x_addrs, s_bit, x_bits, \
		       32, 0)

#define PINS_FIELD_BASE(s_pin, e_pin, i_base, s_addr, x_addrs, s_bit, x_bits) \
	PIN_FIELD_CALC(s_pin, e_pin, i_base, s_addr, x_addrs, s_bit, x_bits, \
		       32, 1)

static const struct mtk_pin_field_calc mt6797_pin_mode_range[] = {
	PIN_FIELD(0, 261, 0x300, 0x10, 0, 4),
};

static const struct mtk_pin_field_calc mt6797_pin_dir_range[] = {
	PIN_FIELD(0, 261, 0x0, 0x10, 0, 1),
};

static const struct mtk_pin_field_calc mt6797_pin_di_range[] = {
	PIN_FIELD(0, 261, 0x200, 0x10, 0, 1),
};

static const struct mtk_pin_field_calc mt6797_pin_do_range[] = {
	PIN_FIELD(0, 261, 0x100, 0x10, 0, 1),
};

/*
 * MT6797 MSDC0 uses the IO_CFG_B register layout from the vendor driver.
 * GPIOs 114..125 are DAT[0..7], CMD, CLK, DSL and RSTB respectively.
 * Several controls are shared by a group of pads, hence the fixed ranges.
 */
static const struct mtk_pin_field_calc mt6797_pin_ies_range[] = {
	PIN_FIELD_BASE(114, 125, 2, 0x020, 0x10, 0, 1),
};

static const struct mtk_pin_field_calc mt6797_pin_smt_range[] = {
	PINS_FIELD_BASE(114, 117, 2, 0x060, 0x10, 0, 1),
	PINS_FIELD_BASE(118, 121, 2, 0x060, 0x10, 1, 1),
	PIN_FIELD_BASE(122, 122, 2, 0x060, 0x10, 2, 1),
	PIN_FIELD_BASE(123, 123, 2, 0x060, 0x10, 3, 1),
	PINS_FIELD_BASE(124, 125, 2, 0x060, 0x10, 2, 1),
};

static const struct mtk_pin_field_calc mt6797_pin_pupd_range[] = {
	PIN_FIELD_BASE(114, 125, 2, 0x100, 0x10, 6, 1),
};

static const struct mtk_pin_field_calc mt6797_pin_r0_range[] = {
	PIN_FIELD_BASE(114, 125, 2, 0x110, 0x10, 6, 1),
};

static const struct mtk_pin_field_calc mt6797_pin_r1_range[] = {
	PIN_FIELD_BASE(114, 125, 2, 0x120, 0x10, 6, 1),
};

static const struct mtk_pin_field_calc mt6797_pin_sr_range[] = {
	PINS_FIELD_BASE(114, 117, 2, 0x030, 0x10, 15, 1),
	PINS_FIELD_BASE(118, 121, 2, 0x030, 0x10, 16, 1),
	PINS_FIELD_BASE(122, 122, 2, 0x030, 0x10, 17, 1),
	PIN_FIELD_BASE(123, 123, 2, 0x030, 0x10, 18, 1),
	PINS_FIELD_BASE(124, 125, 2, 0x030, 0x10, 17, 1),
};

static const struct mtk_pin_field_calc mt6797_pin_tdsel_range[] = {
	PINS_FIELD_BASE(114, 117, 2, 0x0d0, 0x10, 0, 4),
	PINS_FIELD_BASE(118, 121, 2, 0x0d0, 0x10, 4, 4),
	PIN_FIELD_BASE(122, 122, 2, 0x0d0, 0x10, 8, 4),
	PIN_FIELD_BASE(123, 123, 2, 0x0d0, 0x10, 12, 4),
	PINS_FIELD_BASE(124, 125, 2, 0x0d0, 0x10, 8, 4),
};

static const struct mtk_pin_field_calc mt6797_pin_rdsel_range[] = {
	PINS_FIELD_BASE(114, 117, 2, 0x080, 0x10, 0, 6),
	PINS_FIELD_BASE(118, 121, 2, 0x080, 0x10, 6, 6),
	PINS_FIELD_BASE(122, 125, 2, 0x080, 0x10, 12, 6),
};

static const struct mtk_pin_field_calc mt6797_pin_drv_range[] = {
	PINS_FIELD_BASE(114, 117, 2, 0x1a0, 0x10, 0, 3),
	PINS_FIELD_BASE(118, 121, 2, 0x1a0, 0x10, 3, 3),
	PIN_FIELD_BASE(122, 122, 2, 0x1a0, 0x10, 6, 3),
	PIN_FIELD_BASE(123, 123, 2, 0x1a0, 0x10, 9, 3),
	PINS_FIELD_BASE(124, 125, 2, 0x1a0, 0x10, 6, 3),
};

static const unsigned int mt6797_pull_type[ARRAY_SIZE(mtk_pins_mt6797)] = {
	[114 ... 125] = MTK_PULL_PUPD_R1R0_TYPE,
};

static const struct mtk_pin_reg_calc mt6797_reg_cals[PINCTRL_PIN_REG_MAX] = {
	[PINCTRL_PIN_REG_MODE] = MTK_RANGE(mt6797_pin_mode_range),
	[PINCTRL_PIN_REG_DIR] = MTK_RANGE(mt6797_pin_dir_range),
	[PINCTRL_PIN_REG_DI] = MTK_RANGE(mt6797_pin_di_range),
	[PINCTRL_PIN_REG_DO] = MTK_RANGE(mt6797_pin_do_range),
	[PINCTRL_PIN_REG_IES] = MTK_RANGE(mt6797_pin_ies_range),
	[PINCTRL_PIN_REG_SMT] = MTK_RANGE(mt6797_pin_smt_range),
	[PINCTRL_PIN_REG_PUPD] = MTK_RANGE(mt6797_pin_pupd_range),
	[PINCTRL_PIN_REG_R0] = MTK_RANGE(mt6797_pin_r0_range),
	[PINCTRL_PIN_REG_R1] = MTK_RANGE(mt6797_pin_r1_range),
	[PINCTRL_PIN_REG_SR] = MTK_RANGE(mt6797_pin_sr_range),
	[PINCTRL_PIN_REG_TDSEL] = MTK_RANGE(mt6797_pin_tdsel_range),
	[PINCTRL_PIN_REG_RDSEL] = MTK_RANGE(mt6797_pin_rdsel_range),
	[PINCTRL_PIN_REG_DRV] = MTK_RANGE(mt6797_pin_drv_range),
};

static const char * const mt6797_pinctrl_register_base_names[] = {
	"gpio", "iocfgl", "iocfgb", "iocfgr", "iocfgt",
};

static const struct mtk_pin_soc mt6797_data = {
	.reg_cal = mt6797_reg_cals,
	.pins = mtk_pins_mt6797,
	.npins = ARRAY_SIZE(mtk_pins_mt6797),
	.ngrps = ARRAY_SIZE(mtk_pins_mt6797),
	.gpio_m = 0,
	.ies_present = true,
	.base_names = mt6797_pinctrl_register_base_names,
	.nbase_names = ARRAY_SIZE(mt6797_pinctrl_register_base_names),
	.pull_type = mt6797_pull_type,
	.bias_set_combo = mtk_pinconf_bias_set_combo,
	.bias_get_combo = mtk_pinconf_bias_get_combo,
	.drive_set = mtk_pinconf_drive_set_rev1,
	.drive_get = mtk_pinconf_drive_get_rev1,
};

static const struct of_device_id mt6797_pinctrl_of_match[] = {
	{ .compatible = "mediatek,mt6797-pinctrl", .data = &mt6797_data },
	{ }
};

static struct platform_driver mt6797_pinctrl_driver = {
	.driver = {
		.name = "mt6797-pinctrl",
		.of_match_table = mt6797_pinctrl_of_match,
	},
	.probe = mtk_paris_pinctrl_probe,
};

static int __init mt6797_pinctrl_init(void)
{
	return platform_driver_register(&mt6797_pinctrl_driver);
}
arch_initcall(mt6797_pinctrl_init);
