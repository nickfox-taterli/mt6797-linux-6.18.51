// SPDX-License-Identifier: GPL-2.0-only
//
// Minimal MT6351 regulator support for the MT6797 X20 board.

#include <linux/bitops.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>

#define MT6351_LDO_VMCH_CON0	0x0a2e
#define MT6351_VMCH_ANA_CON0	0x0ace
#define MT6351_LDO_VEMC_CON0	0x0a28
#define MT6351_VEMC_ANA_CON0	0x0ad2
#define MT6351_LDO_VMC_CON0	0x0aaa
#define MT6351_VMC_ANA_CON0	0x0ae2
#define MT6351_LDO_VCN18_CON0	0x0a52
#define MT6351_LDO_VCN28_CON0	0x0a0c
#define MT6351_LDO_VCN33_CON3	0x0a98
#define MT6351_LDO_VCN33_CON4	0x0a9a

static const unsigned int mt6351_vmch_voltages[] = {
	3000000,
	3300000,
};

static const struct regulator_ops mt6351_vmch_ops = {
	.list_voltage = regulator_list_voltage_table,
	.map_voltage = regulator_map_voltage_iterate,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
};

static const unsigned int mt6351_vmc_voltages[] = {
	1200000,
	1300000,
	1500000,
	1800000,
	2000000,
	2900000,
	3000000,
	3300000,
};

static const struct regulator_ops mt6351_vmc_ops = {
	.list_voltage = regulator_list_voltage_table,
	.map_voltage = regulator_map_voltage_iterate,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
};

static const struct regulator_desc mt6351_vmch_desc = {
	.name = "vmch",
	.of_match = "ldo_vmch",
	.regulators_node = of_match_ptr("regulators"),
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.n_voltages = ARRAY_SIZE(mt6351_vmch_voltages),
	.volt_table = mt6351_vmch_voltages,
	.vsel_reg = MT6351_VMCH_ANA_CON0,
	.vsel_mask = BIT(8),
	.enable_reg = MT6351_LDO_VMCH_CON0,
	.enable_mask = BIT(1),
	.ops = &mt6351_vmch_ops,
};

/* MT6797 BSP VEMC33: the same 3.0/3.3-V selector encoding as VMCH. */
static const struct regulator_desc mt6351_vemc_desc = {
	.name = "vemc",
	.of_match = "ldo_vemc",
	.regulators_node = of_match_ptr("regulators"),
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.n_voltages = ARRAY_SIZE(mt6351_vmch_voltages),
	.volt_table = mt6351_vmch_voltages,
	.vsel_reg = MT6351_VEMC_ANA_CON0,
	.vsel_mask = BIT(8),
	.enable_reg = MT6351_LDO_VEMC_CON0,
	.enable_mask = BIT(1),
	.ops = &mt6351_vmch_ops,
};

static const struct regulator_desc mt6351_vmc_desc = {
	.name = "vmc",
	.of_match = "ldo_vmc",
	.regulators_node = of_match_ptr("regulators"),
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.n_voltages = ARRAY_SIZE(mt6351_vmc_voltages),
	.volt_table = mt6351_vmc_voltages,
	.vsel_reg = MT6351_VMC_ANA_CON0,
	.vsel_mask = 0x7 << 8,
	.enable_reg = MT6351_LDO_VMC_CON0,
	.enable_mask = BIT(1),
	.ops = &mt6351_vmc_ops,
};

/*
 * Connectivity rails for the MT6797 ConSys / MT6631 combo. Each output has a
 * hardware-fixed voltage and a single software enable bit; VCN33 has separate
 * BT and WiFi outputs (LDO_VCN33_CON3/CON4).
 */
#define MT6351_FIXED_LDO(_name, _id, _of, _reg)				\
static const struct regulator_desc mt6351_##_name##_desc = {		\
	.name = #_name,							\
	.of_match = of_match_ptr(_of),					\
	.regulators_node = of_match_ptr("regulators"),			\
	.type = REGULATOR_VOLTAGE,					\
	.owner = THIS_MODULE,						\
	.n_voltages = 1,						\
	.fixed_uV = _id##_UV,						\
	.enable_reg = _reg,						\
	.enable_mask = BIT(1),						\
	.ops = &mt6351_vcn_ops,						\
}

static const struct regulator_ops mt6351_vcn_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
};

#define MT6351_VCN18_UV 1800000
#define MT6351_VCN28_UV 2800000
#define MT6351_VCN33_BT_UV 3300000
#define MT6351_VCN33_WIFI_UV 3300000

MT6351_FIXED_LDO(vcn18, MT6351_VCN18, "ldo_vcn18", MT6351_LDO_VCN18_CON0);
MT6351_FIXED_LDO(vcn28, MT6351_VCN28, "ldo_vcn28", MT6351_LDO_VCN28_CON0);
MT6351_FIXED_LDO(vcn33_bt, MT6351_VCN33_BT, "ldo_vcn33_bt", MT6351_LDO_VCN33_CON3);
MT6351_FIXED_LDO(vcn33_wifi, MT6351_VCN33_WIFI, "ldo_vcn33_wifi", MT6351_LDO_VCN33_CON4);

static int mt6351_regulator_probe(struct platform_device *pdev)
{
	struct regulator_config config = {
		.dev = &pdev->dev,
		.regmap = dev_get_regmap(pdev->dev.parent, NULL),
	};
	const struct regulator_desc *descs[] = {
		&mt6351_vemc_desc,
		&mt6351_vmch_desc,
		&mt6351_vmc_desc,
		&mt6351_vcn18_desc,
		&mt6351_vcn28_desc,
		&mt6351_vcn33_bt_desc,
		&mt6351_vcn33_wifi_desc,
	};
	unsigned int i;
	struct regulator_dev *rdev;
	struct device_node *regulators, *vemc;
	bool has_vemc;

	if (!config.regmap)
		return dev_err_probe(&pdev->dev, -ENODEV,
				     "PMIC wrapper regmap is unavailable\n");
	regulators = of_get_child_by_name(pdev->dev.of_node, "regulators");
	vemc = of_get_child_by_name(regulators, "ldo_vemc");
	has_vemc = !!vemc;
	of_node_put(vemc);
	of_node_put(regulators);

	for (i = 0; i < ARRAY_SIZE(descs); i++) {
		/*
		 * Legacy DTs model VCC as fixed: do not register an unused,
		 * switchable VEMC that regulator late init could turn off.
		 */
		if (descs[i] == &mt6351_vemc_desc && !has_vemc)
			continue;
		rdev = devm_regulator_register(&pdev->dev, descs[i], &config);
		if (IS_ERR(rdev))
			return dev_err_probe(&pdev->dev, PTR_ERR(rdev),
					     "failed to register MT6351 rail %s\n",
					     descs[i]->name);
	}

	return 0;
}

static const struct of_device_id mt6351_regulator_of_match[] = {
	{ .compatible = "mediatek,mt6351" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mt6351_regulator_of_match);

static struct platform_driver mt6351_regulator_driver = {
	.probe = mt6351_regulator_probe,
	.driver = {
		.name = "mt6351-regulator",
		.of_match_table = mt6351_regulator_of_match,
	},
};
module_platform_driver(mt6351_regulator_driver);

MODULE_DESCRIPTION("MT6351 VEMC/VMC/VMCH/VCN regulator driver");
MODULE_LICENSE("GPL");
