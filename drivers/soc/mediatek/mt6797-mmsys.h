/* SPDX-License-Identifier: GPL-2.0-only */
/* MT6797 BSP ddp_path.c: external RDMA1 -> DSC bypass -> DPI. */
#ifndef __SOC_MEDIATEK_MT6797_MMSYS_H
#define __SOC_MEDIATEK_MT6797_MMSYS_H
static const struct mtk_mmsys_routes mt6797_mmsys_routing_table[] = {
	MMSYS_ROUTE(RDMA1, DSC0, 0x094, BIT(0), BIT(0)),
	MMSYS_ROUTE(RDMA1, DSC0, 0x078, BIT(0), BIT(0)),
	MMSYS_ROUTE(DSC0, DPI0, 0x044, BIT(2), BIT(2)),
	MMSYS_ROUTE(DSC0, DPI0, 0x084, GENMASK(1, 0), 2),
};
#endif
