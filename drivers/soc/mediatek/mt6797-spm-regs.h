/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (C) 2015 MediaTek Inc.
 * Register offsets from BSP spm_v2/mt_spm_reg_mt6797.h.
 * See docs/power/20261022-cpu-remaining for provenance.
 */
#ifndef _MT6797_SPM_REGS_H
#define _MT6797_SPM_REGS_H

#define POWERON_CONFIG_EN	0x000
/* SPM PCM access key, written with the register address in POWERON_CONFIG_EN
 * (BSP spm_v2 SPM_KEY); shared by the vcorefs and spm-suspend drivers. */
#define SPM_KEY			0x0b160000
#define SPM_POWER_ON_VAL0	0x004
#define SPM_POWER_ON_VAL1	0x008
#define SPM_CLK_CON	0x00C
#define SPM_AP_STANDBY_CON	0x014
#define PCM_CON0	0x018
#define PCM_CON1	0x01C
#define PCM_IM_PTR	0x020
#define PCM_IM_LEN	0x024
#define PCM_REG_DATA_INI	0x028
#define PCM_PWR_IO_EN	0x02C
#define PCM_TIMER_VAL	0x030
#define PCM_WDT_VAL	0x034
#define PCM_WDT_TIMEOUT	(30 * 32768)
#define PCM_TIMER_MAX	(U32_MAX - PCM_WDT_TIMEOUT)
#define PCM_WDT_EN	BIT(8)
#define PCM_WDT_WAKE_MODE	BIT(9)
#define PCM_REG0_DATA	0x100
#define PCM_REG1_DATA	0x104
#define PCM_REG6_DATA	0x118
#define PCM_REG15_DATA	0x13C
#define PCM_FSM_STA	0x17C
#define SPM_SRC_REQ	0x0B8
#define SPM_SRC_MASK	0x0BC
#define SPM_SRC2_MASK	0x0C0
#define SPM_WAKEUP_EVENT_MASK	0x0C4
#define SPM_CPU_WAKEUP_EVENT	0x0B0
#define SPM_IRQ_MASK	0x0B4
#define SPM_IRQ_STA	0x158
#define PCM_WDT_OUT	0x154
#define SPM_SW_INT_CLEAR	0x094
#define SPM_SW_FLAG	0x600
#define SPM_SW_RSV_0	0x608
#define SPM_SW_RSV_1	0x60C
#define SPM_SW_RSV_4	0x618
#define SPM_SW_RSV_5	0x61C
#define SPM_PASR_DPD_1	0x634
#define SPM_PASR_DPD_2	0x638
#define SPM_DVFS_CON	0x400
#define SPM_EMI_BW_MODE	0x450
#define CPU_DVFS_REQ	0x448
#define SPM_MAS_PAUSE_MASK_B	0x408
#define SPM_MAS_PAUSE2_MASK_B	0x40C
#define PCM_EVENT_VECTOR0	0x040
#define PCM_EVENT_VECTOR1	0x044
#define PCM_EVENT_VECTOR2	0x048
#define PCM_EVENT_VECTOR3	0x04C
#define PCM_EVENT_VECTOR4	0x050
#define PCM_EVENT_VECTOR5	0x054
#define PCM_EVENT_VECTOR6	0x058
#define PCM_EVENT_VECTOR7	0x05C
#define PCM_EVENT_VECTOR8	0x060
#define PCM_EVENT_VECTOR9	0x064
#define PCM_EVENT_VECTOR10	0x068
#define PCM_EVENT_VECTOR11	0x06C
#define PCM_EVENT_VECTOR12	0x070
#define PCM_EVENT_VECTOR13	0x074
#define PCM_EVENT_VECTOR14	0x078
#define PCM_EVENT_VECTOR15	0x07C
#define MP0_CPU0_WFI_EN	0x540
#define MP0_CPU1_WFI_EN	0x544
#define MP0_CPU2_WFI_EN	0x548
#define MP0_CPU3_WFI_EN	0x54C
#define MP1_CPU0_WFI_EN	0x550
#define MP1_CPU1_WFI_EN	0x554
#define MP1_CPU2_WFI_EN	0x558
#define MP1_CPU3_WFI_EN	0x55C
#define MP2_CPU0_WFI_EN	0x560
#define MP2_CPU1_WFI_EN	0x564

#endif
