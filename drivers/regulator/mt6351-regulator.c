// SPDX-License-Identifier: GPL-2.0-only
//
// Minimal MT6351 regulator support for the MT6797 X20 board.

#include <linux/bitops.h>
#include <linux/bitfield.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>

#define MT6351_LDO_VMCH_CON0	0x0a2e
#define MT6351_VMCH_ANA_CON0	0x0ace
#define MT6351_LDO_VMC_CON0	0x0aaa
#define MT6351_VMC_ANA_CON0	0x0ae2

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
	1800000,
	3000000,
};

/* VMC uses non-linear PMIC selector codes: 3 = 1.8 V, 6 = 3.0 V. */
static const unsigned int mt6351_vmc_selectors[] = {
	3,
	6,
};

static int mt6351_vmc_list_voltage(struct regulator_dev *rdev,
				   unsigned int selector)
{
	(void)rdev;

	if (selector >= ARRAY_SIZE(mt6351_vmc_voltages))
		return -EINVAL;

	return mt6351_vmc_voltages[selector];
}

static int mt6351_vmc_map_voltage(struct regulator_dev *rdev,
				  int min_uV, int max_uV)
{
	unsigned int i;

	(void)rdev;

	for (i = 0; i < ARRAY_SIZE(mt6351_vmc_voltages); i++)
		if (mt6351_vmc_voltages[i] >= min_uV &&
		    mt6351_vmc_voltages[i] <= max_uV)
			return i;

	return -EINVAL;
}

static int mt6351_vmc_set_voltage_sel(struct regulator_dev *rdev,
				      unsigned int selector)
{
	if (selector >= ARRAY_SIZE(mt6351_vmc_selectors))
		return -EINVAL;

	return regmap_update_bits(rdev->regmap, MT6351_VMC_ANA_CON0,
				  GENMASK(10, 8),
				  mt6351_vmc_selectors[selector] << 8);
}

static int mt6351_vmc_get_voltage_sel(struct regulator_dev *rdev)
{
	unsigned int val;
	int ret;

	ret = regmap_read(rdev->regmap, MT6351_VMC_ANA_CON0, &val);
	if (ret)
		return ret;

	val = FIELD_GET(GENMASK(10, 8), val);
	if (val == mt6351_vmc_selectors[0])
		return 0;
	if (val == mt6351_vmc_selectors[1])
		return 1;

	return -EINVAL;
}

static const struct regulator_ops mt6351_vmc_ops = {
	.list_voltage = mt6351_vmc_list_voltage,
	.map_voltage = mt6351_vmc_map_voltage,
	.set_voltage_sel = mt6351_vmc_set_voltage_sel,
	.get_voltage_sel = mt6351_vmc_get_voltage_sel,
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

static const struct regulator_desc mt6351_vmc_desc = {
	.name = "vmc",
	.of_match = "ldo_vmc",
	.regulators_node = of_match_ptr("regulators"),
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.n_voltages = ARRAY_SIZE(mt6351_vmc_voltages),
	.vsel_reg = MT6351_VMC_ANA_CON0,
	.vsel_mask = GENMASK(10, 8),
	.enable_reg = MT6351_LDO_VMC_CON0,
	.enable_mask = BIT(1),
	.ops = &mt6351_vmc_ops,
};

static int mt6351_regulator_probe(struct platform_device *pdev)
{
	struct regulator_config config = {
		.dev = &pdev->dev,
		.regmap = dev_get_regmap(pdev->dev.parent, NULL),
	};
	const struct regulator_desc *descs[] = {
		&mt6351_vmch_desc,
		&mt6351_vmc_desc,
	};
	unsigned int i;
	struct regulator_dev *rdev;

	if (!config.regmap)
		return dev_err_probe(&pdev->dev, -ENODEV,
				     "PMIC wrapper regmap is unavailable\n");

	for (i = 0; i < ARRAY_SIZE(descs); i++) {
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

MODULE_DESCRIPTION("Minimal MT6351 VMC/VMCH regulator driver");
MODULE_LICENSE("GPL");
