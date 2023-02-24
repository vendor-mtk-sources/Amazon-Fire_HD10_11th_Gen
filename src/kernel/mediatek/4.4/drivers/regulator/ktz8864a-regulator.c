/*
 * KINETIC Regulator Driver
 *
 * Author: Bruce pu <bruce.xm.pu@enskytech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/mfd/kinetic-lmu.h>
#include <linux/mfd/kinetic-lmu-register.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/slab.h>

/* KTZ8864A */
#define KTZ8864A_BOOST_VSEL_MAX		0x34
#define KTZ8864A_LDO_VSEL_MAX		0x2E
#define KTZ8864A_VOLTAGE_MIN		4000000

#define KTZ8864A_STEP_50mV		50000
#define KTZ8864A_STEP_500mV		500000

static int ktz8864a_regulator_enable_time(struct regulator_dev *rdev)
{
	return 0;
}

static const struct regulator_ops ktz8864a_boost_voltage_table_ops = {
	.list_voltage     = regulator_list_voltage_linear,
	.set_voltage_sel  = regulator_set_voltage_sel_regmap,
	.get_voltage_sel  = regulator_get_voltage_sel_regmap,
};

static const struct regulator_ops ktz8864a_regulator_voltage_table_ops = {
	.list_voltage     = regulator_list_voltage_linear,
	.set_voltage_sel  = regulator_set_voltage_sel_regmap,
	.get_voltage_sel  = regulator_get_voltage_sel_regmap,
	.enable           = regulator_enable_regmap,
	.disable          = regulator_disable_regmap,
	.is_enabled       = regulator_is_enabled_regmap,
	.enable_time      = ktz8864a_regulator_enable_time,
};

static const struct regulator_desc ktz8864a_regulator_desc[] = {
	/* KTZ8864A */
	{
		.name           = "vboost",
		.of_match	= "vboost",
		.id             = KTZ8864A_BOOST,
		.ops            = &ktz8864a_boost_voltage_table_ops,
		.n_voltages     = KTZ8864A_BOOST_VSEL_MAX,
		.min_uV         = KTZ8864A_VOLTAGE_MIN,
		.uV_step        = KTZ8864A_STEP_50mV,
		.type           = REGULATOR_VOLTAGE,
		.owner          = THIS_MODULE,
		.vsel_reg       = KTZ8864A_REG_VOUT_BOOST,
		.vsel_mask      = KTZ8864A_VOUT_MASK,
	},
	{
		.name           = "ldo_vpos",
		.of_match	= "vpos",
		.id             = KTZ8864A_LDO_POS,
		.ops            = &ktz8864a_regulator_voltage_table_ops,
		.n_voltages     = KTZ8864A_LDO_VSEL_MAX,
		.min_uV         = KTZ8864A_VOLTAGE_MIN,
		.uV_step        = KTZ8864A_STEP_50mV,
		.type           = REGULATOR_VOLTAGE,
		.owner          = THIS_MODULE,
		.vsel_reg       = KTZ8864A_REG_VOUT_POS,
		.vsel_mask      = KTZ8864A_VOUT_MASK,
		.enable_reg     = KTZ8864A_REG_BIAS_CONFIG_1,
		.enable_mask    = KTZ8864A_EN_VPOS_MASK,
	},
	{
		.name           = "ldo_vneg",
		.of_match	= "vneg",
		.id             = KTZ8864A_LDO_NEG,
		.ops            = &ktz8864a_regulator_voltage_table_ops,
		.n_voltages     = KTZ8864A_LDO_VSEL_MAX,
		.min_uV         = KTZ8864A_VOLTAGE_MIN,
		.uV_step        = KTZ8864A_STEP_50mV,
		.type           = REGULATOR_VOLTAGE,
		.owner          = THIS_MODULE,
		.vsel_reg       = KTZ8864A_REG_VOUT_NEG,
		.vsel_mask      = KTZ8864A_VOUT_MASK,
		.enable_reg     = KTZ8864A_REG_BIAS_CONFIG_1,
		.enable_mask    = KTZ8864A_EN_VNEG_MASK,
	},
};

static int ktz8864a_regulator_of_get_enable_gpio(struct device_node *np, int id)
{
	/*
	 * Check LCM_EN1/2_GPIO is configured.
	 * Those pins are used for enabling VPOS/VNEG LDOs.
	 * Do not use devm* here: the regulator core takes over the
	 * lifecycle management of the GPIO descriptor.
	 */
	switch (id) {
	case KTZ8864A_LDO_POS:
		return of_get_named_gpio(np, "enable-gpios", 0);
	case KTZ8864A_LDO_NEG:
		return of_get_named_gpio(np, "enable-gpios", 1);
	default:
		return -EINVAL;
	}
}

static int ktz8864a_regulator_set_ext_en(struct regmap *regmap, int id)
{
	int ext_en_mask = 0;

	switch (id) {
	case KTZ8864A_LDO_POS:
	case KTZ8864A_LDO_NEG:
		ext_en_mask = KTZ8864A_EXT_EN_MASK;
		break;
	default:
		return -ENODEV;
	}

	return regmap_update_bits(regmap, ktz8864a_regulator_desc[id].enable_reg,
				 ext_en_mask, ext_en_mask);
}

static int ktz8864a_regulator_probe(struct platform_device *pdev)
{
	struct kinetic_lmu *lmu = dev_get_drvdata(pdev->dev.parent);
	struct regmap *regmap = lmu->regmap;
	struct regulator_config cfg = { };
	struct regulator_dev *rdev;
	struct device *dev = &pdev->dev;
	int id = pdev->id;
	int ret, ena_gpio;

	cfg.dev = dev;
	cfg.regmap = regmap;

	ena_gpio = ktz8864a_regulator_of_get_enable_gpio(dev->of_node, id);
	if (gpio_is_valid(ena_gpio)) {
		cfg.ena_gpio = ena_gpio;
		if (of_property_read_bool(dev->of_node, "enable-init-high"))
			cfg.ena_gpio_flags = GPIOF_OUT_INIT_HIGH;
		else
			cfg.ena_gpio_flags = GPIOF_OUT_INIT_LOW;

		ret = ktz8864a_regulator_set_ext_en(regmap, id);
		if (ret) {
			dev_err(dev, "External pin err: %d\n", ret);
			return ret;
		}
	}

	rdev = devm_regulator_register(dev, &ktz8864a_regulator_desc[id], &cfg);
	if (IS_ERR(rdev)) {
		ret = PTR_ERR(rdev);
		dev_err(dev, "[%d] regulator register err: %d\n", id, ret);
		return ret;
	}

	return 0;
}

static struct platform_driver ktz8864a_regulator_driver = {
	.probe = ktz8864a_regulator_probe,
	.driver = {
		.name = "ktz8864a-regulator",
	},
};

module_platform_driver(ktz8864a_regulator_driver);

MODULE_DESCRIPTION("KINETIC KTZ8864A Regulator Driver");
MODULE_AUTHOR("Bruce Pu <bruce.xm.pu@enskytech.com>");
MODULE_LICENSE("GPL v2");
