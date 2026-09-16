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

static int mt6351_regulator_probe(struct platform_device *pdev)
{
	struct regulator_config config = {
		.dev = &pdev->dev,
		.regmap = dev_get_regmap(pdev->dev.parent, NULL),
	};
	struct regulator_dev *rdev;

	if (!config.regmap)
		return dev_err_probe(&pdev->dev, -ENODEV,
				     "PMIC wrapper regmap is unavailable\n");

	rdev = devm_regulator_register(&pdev->dev, &mt6351_vmch_desc,
				       &config);
	if (IS_ERR(rdev))
		return dev_err_probe(&pdev->dev, PTR_ERR(rdev),
				     "failed to register VMCH\n");

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

MODULE_DESCRIPTION("Minimal MT6351 VMCH regulator driver");
MODULE_LICENSE("GPL");
