// SPDX-License-Identifier: GPL-2.0-only
/* MT6797 BSP MFG_BG3D gate, exported through the standard clock framework. */
#include <linux/module.h>
#include <linux/platform_device.h>
#include "clk-gate.h"
#include "clk-mtk.h"

static const struct mtk_gate_regs mfg_regs = {
	.sta_ofs = 0, .set_ofs = 4, .clr_ofs = 8,
};
static const struct mtk_gate mfg_clks[] = {
	GATE_MTK(0, "mfg_bg3d", "mfg_sel", &mfg_regs, 0, &mtk_clk_gate_ops_setclr),
};
static const struct mtk_clk_desc mfg_desc = {
	.clks = mfg_clks, .num_clks = ARRAY_SIZE(mfg_clks),
};
static const struct of_device_id mfg_match[] = {
	{ .compatible = "mediatek,mt6797-mfgcfg", .data = &mfg_desc }, { }
};
MODULE_DEVICE_TABLE(of, mfg_match);
static struct platform_driver mfg_driver = {
	.probe = mtk_clk_simple_probe,
	.remove = mtk_clk_simple_remove,
	.driver = { .name = "clk-mt6797-mfg", .of_match_table = mfg_match },
};
module_platform_driver(mfg_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MediaTek MT6797 GPU clock");
