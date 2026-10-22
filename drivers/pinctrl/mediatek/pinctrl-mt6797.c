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

#include <linux/delay.h>

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
	PIN_FIELD_BASE(43, 50, 1, 0x000, 0x10, 16, 1),
	PIN_FIELD_BASE(52, 52, 1, 0x000, 0x10, 25, 1),
	PIN_FIELD_BASE(57, 60, 2, 0x000, 0x10, 8, 1),
	PIN_FIELD_BASE(65, 67, 3, 0x010, 0x10, 12, 1),
	PIN_FIELD_BASE(114, 125, 2, 0x020, 0x10, 0, 1),
	PIN_FIELD_BASE(129, 134, 2, 0x020, 0x10, 25, 1),
	PIN_FIELD_BASE(238, 241, 2, 0x010, 0x10, 4, 1),
};

static const struct mtk_pin_field_calc mt6797_pin_smt_range[] = {
	PIN_FIELD_BASE(57, 58, 2, 0x050, 0x10, 3, 1),
	PINS_FIELD_BASE(59, 60, 2, 0x050, 0x10, 5, 1),
	/* GPIO65..68 share this bit; GPIO68 is not exposed by this work. */
	PINS_FIELD_BASE(65, 67, 3, 0x030, 0x10, 27, 1),
	PINS_FIELD_BASE(114, 117, 2, 0x060, 0x10, 0, 1),
	PINS_FIELD_BASE(118, 121, 2, 0x060, 0x10, 1, 1),
	PIN_FIELD_BASE(122, 122, 2, 0x060, 0x10, 2, 1),
	PIN_FIELD_BASE(123, 123, 2, 0x060, 0x10, 3, 1),
	PINS_FIELD_BASE(124, 125, 2, 0x060, 0x10, 2, 1),
	PINS_FIELD_BASE(129, 129, 3, 0x030, 0x10, 0, 1),
	PINS_FIELD_BASE(130, 133, 3, 0x030, 0x10, 1, 1),
	PINS_FIELD_BASE(134, 134, 3, 0x030, 0x10, 2, 1),
	PINS_FIELD_BASE(238, 239, 2, 0x050, 0x10, 13, 1),
	PINS_FIELD_BASE(240, 241, 2, 0x050, 0x10, 14, 1),
};

static const struct mtk_pin_field_calc mt6797_pin_pd_range[] = {
	PIN_FIELD_BASE(43, 50, 1, 0x0d0, 0x10, 16, 1),
	PIN_FIELD_BASE(52, 52, 1, 0x0d0, 0x10, 25, 1),
	PIN_FIELD_BASE(57, 60, 2, 0x130, 0x10, 2, 1),
	PIN_FIELD_BASE(65, 67, 3, 0x0c0, 0x10, 12, 1),
	PIN_FIELD_BASE(238, 241, 2, 0x130, 0x10, 27, 1),
};

static const struct mtk_pin_field_calc mt6797_pin_pu_range[] = {
	PIN_FIELD_BASE(43, 50, 1, 0x0b0, 0x10, 16, 1),
	PIN_FIELD_BASE(52, 52, 1, 0x0b0, 0x10, 25, 1),
	PIN_FIELD_BASE(57, 60, 2, 0x160, 0x10, 0, 1),
	PIN_FIELD_BASE(65, 67, 3, 0x0e0, 0x10, 12, 1),
};

static const struct mtk_pin_field_calc mt6797_pin_pupd_range[] = {
	PIN_FIELD_BASE(114, 125, 2, 0x100, 0x10, 6, 1),
	PIN_FIELD_BASE(129, 134, 2, 0x100, 0x10, 18, 1),
};

static const struct mtk_pin_field_calc mt6797_pin_r0_range[] = {
	PIN_FIELD_BASE(114, 125, 2, 0x110, 0x10, 6, 1),
	PIN_FIELD_BASE(129, 134, 2, 0x110, 0x10, 18, 1),
};

static const struct mtk_pin_field_calc mt6797_pin_r1_range[] = {
	PIN_FIELD_BASE(114, 125, 2, 0x120, 0x10, 6, 1),
	PIN_FIELD_BASE(129, 134, 2, 0x120, 0x10, 18, 1),
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
	PINS_FIELD_BASE(129, 129, 2, 0x0c0, 0x10, 16, 4),
	PINS_FIELD_BASE(130, 133, 2, 0x0c0, 0x10, 20, 4),
	PINS_FIELD_BASE(134, 134, 2, 0x0c0, 0x10, 24, 4),
};

static const struct mtk_pin_field_calc mt6797_pin_rdsel_range[] = {
	PINS_FIELD_BASE(114, 117, 2, 0x080, 0x10, 0, 6),
	PINS_FIELD_BASE(118, 121, 2, 0x080, 0x10, 6, 6),
	PINS_FIELD_BASE(122, 125, 2, 0x080, 0x10, 12, 6),
	PINS_FIELD_BASE(129, 129, 2, 0x0a0, 0x10, 8, 6),
	PINS_FIELD_BASE(130, 133, 2, 0x0a0, 0x10, 14, 6),
	PINS_FIELD_BASE(134, 134, 2, 0x0a0, 0x10, 20, 6),
};

static const struct mtk_pin_field_calc mt6797_pin_drv_range[] = {
	PINS_FIELD_BASE(57, 60, 2, 0x190, 0x10, 6, 2),
	PINS_FIELD_BASE(114, 117, 2, 0x1a0, 0x10, 0, 3),
	PINS_FIELD_BASE(118, 121, 2, 0x1a0, 0x10, 3, 3),
	PIN_FIELD_BASE(122, 122, 2, 0x1a0, 0x10, 6, 3),
	PIN_FIELD_BASE(123, 123, 2, 0x1a0, 0x10, 9, 3),
	PINS_FIELD_BASE(124, 125, 2, 0x1a0, 0x10, 6, 3),
	PINS_FIELD_BASE(129, 129, 2, 0x1b0, 0x10, 21, 3),
	PINS_FIELD_BASE(130, 133, 2, 0x1b0, 0x10, 24, 3),
	PINS_FIELD_BASE(134, 134, 2, 0x1b0, 0x10, 27, 3),
};

static const unsigned int mt6797_pull_type[ARRAY_SIZE(mtk_pins_mt6797)] = {
	[43 ... 50] = MTK_PULL_PU_PD_TYPE,
	[52] = MTK_PULL_PU_PD_TYPE,
	[57 ... 60] = MTK_PULL_PU_PD_TYPE,
	[65 ... 67] = MTK_PULL_PU_PD_TYPE,
	[114 ... 125] = MTK_PULL_PUPD_R1R0_TYPE,
	[129 ... 134] = MTK_PULL_PUPD_R1R0_TYPE,
	/* I2C4/5 pads have only a programmable pull-down in the BSP. */
	[238 ... 241] = MTK_PULL_PD_TYPE,
};

static const struct mtk_pin_reg_calc mt6797_reg_cals[PINCTRL_PIN_REG_MAX] = {
	[PINCTRL_PIN_REG_MODE] = MTK_RANGE(mt6797_pin_mode_range),
	[PINCTRL_PIN_REG_DIR] = MTK_RANGE(mt6797_pin_dir_range),
	[PINCTRL_PIN_REG_DI] = MTK_RANGE(mt6797_pin_di_range),
	[PINCTRL_PIN_REG_DO] = MTK_RANGE(mt6797_pin_do_range),
	[PINCTRL_PIN_REG_PD] = MTK_RANGE(mt6797_pin_pd_range),
	[PINCTRL_PIN_REG_PU] = MTK_RANGE(mt6797_pin_pu_range),
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

/* Board BSP mt6797.dtsi: exact debounce periods in microseconds. */
static const unsigned int mt6797_debounce_time[] = {
	128, 256, 512, 1024, 16384, 32768, 65536, 131072, 262144, 524288, 0
};

static void mt6797_eint_sync_write(u32 value, void __iomem *reg)
{
	/* BSP mt_reg_sync_writel() completes writes before the polarity delay. */
	writel(value, reg);
	/* Match the BSP write-completion barrier before touching EINT state. */
	mb();
}

/* Preserve irq-mt-eic.c's RAW_STA sampling after selecting the EINT mux. */
static int mt6797_eint_get_raw_level(struct mtk_eint *eint, unsigned int eint_num)
{
	const struct mtk_eint_regs *regs = eint->regs;
	unsigned int idx = eint->pins[eint_num].index;
	u32 bit = BIT(idx % 32), masked, pol, sens, level;
	void __iomem *base = eint->base[eint->pins[eint_num].instance] +
			     (idx / 32) * 4;

	masked = readl(base + regs->mask) & bit;
	pol = readl(base + regs->pol) & bit;
	sens = readl(base + regs->sens) & bit;
	if (!masked)
		mt6797_eint_sync_write(bit, base + regs->mask_set);

	mt6797_eint_sync_write(bit, base + regs->pol_set);
	/* BSP mt_eint_set_polarity(): longest hardware latency is 250 ns. */
	ndelay(300);
	mt6797_eint_sync_write(bit, base + regs->ack);
	mt6797_eint_sync_write(bit, base + regs->sens_set);
	mt6797_eint_sync_write(bit, base + regs->ack);
	level = !!(readl(base + 0xa00) & bit);

	mt6797_eint_sync_write(bit, base + (pol ? regs->pol_set : regs->pol_clr));
	ndelay(300);
	mt6797_eint_sync_write(bit, base + regs->ack);
	mt6797_eint_sync_write(bit, base + (sens ? regs->sens_set : regs->sens_clr));
	mt6797_eint_sync_write(bit, base + regs->ack);
	if (!masked)
		mt6797_eint_sync_write(bit, base + regs->mask_clr);

	return level;
}

/* GPIO DI avoids changing/acknowledging a live physical edge to sample it. */
static int mt6797_eint_get_level(struct mtk_eint *eint, unsigned int eint_num)
{
	int level = eint->gpio_xlate->get_gpio_state(eint->pctl, eint_num);

	if (level >= 0)
		return level;
	return mt6797_eint_get_raw_level(eint, eint_num);
}

static int mt6797_eint_flip_edge(struct mtk_eint *eint, unsigned int eint_num)
{
	const struct mtk_eint_regs *regs = eint->regs;
	unsigned int idx = eint->pins[eint_num].index, retries = 0;
	void __iomem *base = eint->base[eint->pins[eint_num].instance] +
			     (idx / 32) * 4;
	u32 bit = BIT(idx % 32), masked;
	int level, next;

	masked = readl(base + regs->mask) & bit;
	if (!masked)
		mt6797_eint_sync_write(bit, base + regs->mask_set);
	/*
	 * RAW_STA sampling acknowledges pending edges while restoring state.
	 * The final check must use pad DI, without clearing a newly arrived edge.
	 */
	next = eint->gpio_xlate->get_gpio_state(eint->pctl, eint_num);
	do {
		level = next;
		mt6797_eint_sync_write(bit, base + (level ? regs->pol_clr : regs->pol_set));
		ndelay(300);
		mt6797_eint_sync_write(bit, base + regs->ack);
		/* Opposite level remains pending even if a contact changes at ACK. */
		mt6797_eint_sync_write(bit, base + regs->sens_set);
		mt6797_eint_sync_write(bit, base + regs->ack);
		next = eint->gpio_xlate->get_gpio_state(eint->pctl, eint_num);
	} while (level != next && ++retries < 32);
	if (level != next)
		mt6797_eint_sync_write(bit, base + regs->soft_set);
	if (!masked)
		mt6797_eint_sync_write(bit, base + regs->mask_clr);

	return next;
}

/* Caller masks the line and restores its mask after this BSP sequence. */
static void mt6797_eint_set_debounce(struct mtk_eint *eint,
				    unsigned int eint_num, unsigned int debounce)
{
	const struct mtk_eint_regs *regs = eint->regs;
	unsigned int idx = eint->pins[eint_num].index;
	unsigned int shift = (idx % 4) * 8, setting;
	void __iomem *base = eint->base[eint->pins[eint_num].instance] +
			     (idx / 4) * 4;

	writel(0xffU << shift, base + regs->dbnc_clr);
	/* Standard GPIO debounce=0 disables the hardware filter. */
	if (!debounce)
		return;

	for (setting = 0; setting < ARRAY_SIZE(mt6797_debounce_time) - 2;
	     setting++)
		if (debounce <= mt6797_debounce_time[setting])
			break;

	/* BSP saturates requests above 524288 us at the last valid encoding. */
	writel(((setting << 4) | BIT(0)) << shift, base + regs->dbnc_set);
	udelay(500);
	writel(BIT(1) << shift, base + regs->dbnc_set);
	udelay(500);
}

static const struct mtk_eint_hw mt6797_eint_hw = {
	.port_mask = 7,
	.ports = 6,
	.ap_num = 192,
	.db_cnt = 16,
	.db_time = mt6797_debounce_time,
	.polarity_delay_ns = 300,
	.dual_edge_level = true,
	.get_level = mt6797_eint_get_level,
	.flip_edge = mt6797_eint_flip_edge,
	.set_debounce = mt6797_eint_set_debounce,
};

static const struct mtk_pin_soc mt6797_data = {
	.reg_cal = mt6797_reg_cals,
	.pins = mtk_pins_mt6797,
	.npins = ARRAY_SIZE(mtk_pins_mt6797),
	.ngrps = ARRAY_SIZE(mtk_pins_mt6797),
	.eint_hw = &mt6797_eint_hw,
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
