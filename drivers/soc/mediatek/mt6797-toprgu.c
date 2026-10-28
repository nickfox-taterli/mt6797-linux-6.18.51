// SPDX-License-Identifier: GPL-2.0-only
#include <linux/soc/mediatek/mt6797-sip.h>
#include <dt-bindings/reset/mediatek,mt6797-resets.h>
#include <linux/arm-smccc.h>
#include <linux/errno.h>
#include <linux/export.h>
#include <linux/soc/mediatek/mt6797-toprgu.h>


int mt6797_toprgu_update(unsigned int id, bool assert)
{
	struct arm_smccc_res res;

	if (id >= MT6797_TOPRGU_SW_RST_NUM)
		return -EINVAL;
	arm_smccc_smc(MT6797_SIP_TOPRGU_UPDATE, id, assert,
		      0, 0, 0, 0, 0, &res);
	switch ((long)res.a0) {
	case 0:
		return 0;
	case -1:
		return -EOPNOTSUPP;
	case -2:
		return -EINVAL;
	default:
		return -EIO;
	}
}
EXPORT_SYMBOL_GPL(mt6797_toprgu_update);
