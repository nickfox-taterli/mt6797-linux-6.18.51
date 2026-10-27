/* SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause) */
#ifndef _DT_BINDINGS_RESET_MT6797_H
#define _DT_BINDINGS_RESET_MT6797_H

/* INFRACFG_AO RST0_SET/CLR at 0x120/0x124, BSP tscpu_reset_thermal(). */
#define MT6797_INFRA_THERM_SW_RST 0

/* TOPRGU SWSYSRST, shared with the secure power sequencer. */
#define MT6797_TOPRGU_MFG_SW_RST		2
#define MT6797_TOPRGU_PWRAP_SPI_CTL_RST		11
#define MT6797_TOPRGU_CONN_MCU_SW_RST		12
#define MT6797_TOPRGU_SW_RST_NUM		18

#endif
