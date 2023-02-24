/*
 * KINETIC LMU (Lighting Management Unit) Core Driver
 *
 * Author: Bruce Pu <bruce@enskytech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/mfd/core.h>
#include <linux/mfd/kinetic-lmu.h>
#include <linux/mfd/kinetic-lmu-register.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/slab.h>

#define KTZ8864A_ID 0x2

struct kinetic_lmu_data {
	const struct mfd_cell *cells;
	int num_cells;
	unsigned int max_register;
};

static int kinetic_lmu_check_id(struct kinetic_lmu *lmu, enum kinetic_lmu_id id)
{
	int ret = 0;
	unsigned int val;

	if (lmu->en_gpio)
		gpiod_set_value(lmu->en_gpio, 1);

	if (id == KTZ8864A) {
		ret = regmap_read(lmu->regmap, KTZ8864A_REG_REV, &val);

		if(ret == 0)
		{
			if((val & 3) != KTZ8864A_ID)
			{
				dev_err(lmu->dev, "Check ID fail: id = 0x%x, expected value is %d.\n", val, KTZ8864A_ID);
				ret = -ENODEV;
			}
		}
	}

	return ret;
}

static int kinetic_lmu_enable_hw(struct kinetic_lmu *lmu, enum kinetic_lmu_id id)
{
	if (lmu->en_gpio)
		gpiod_set_value(lmu->en_gpio, 1);

	/* Delay about 1ms after HW enable pin control */
	usleep_range(1000, 1500);

	return 0;
}

static void kinetic_lmu_disable_hw(void *data)
{
	struct kinetic_lmu *lmu = data;
	if (lmu->en_gpio)
		gpiod_set_value(lmu->en_gpio, 0);
}

#define KTZ8864A_REGULATOR(_id)			\
{						\
	.name          = "ktz8864a-regulator",	\
	.id            = _id,			\
	.of_compatible = "kinetic,ktz8864a-regulator",	\
}						\

static const struct mfd_cell ktz8864a_devices[] = {
	KTZ8864A_REGULATOR(KTZ8864A_BOOST),
	KTZ8864A_REGULATOR(KTZ8864A_LDO_POS),
	KTZ8864A_REGULATOR(KTZ8864A_LDO_NEG),
	{
		.name          = "ktz8864a-leds",
		.id            = KTZ8864A,
		.of_compatible = "kinetic,ktz8864a-backlight",
	},
};

#define KINETIC_LMU_DATA(chip, max_reg)		\
static const struct kinetic_lmu_data chip##_data =	\
{						\
	.cells = chip##_devices,		\
	.num_cells = ARRAY_SIZE(chip##_devices),\
	.max_register = max_reg,		\
}

KINETIC_LMU_DATA(ktz8864a, KTZ8864A_MAX_REG);

static int kinetic_lmu_probe(struct i2c_client *cl, const struct i2c_device_id *id)
{
	struct device *dev = &cl->dev;
	const struct kinetic_lmu_data *data;
	struct regmap_config regmap_cfg;
	struct kinetic_lmu *lmu;
	int ret;

	/*
	 * The client addr of lm36274 and ktz8864a is same, thus use pseudo addr for ktz8864a.
	 * Change to real addr at here.
	 */

	cl->addr = 0x11;

	/*
	 * Get device specific data from of_match table.
	 * This data is defined by using TI_LMU_DATA() macro.
	 */
	data = of_device_get_match_data(dev);
	if (!data)
		return -ENODEV;

	lmu = devm_kzalloc(dev, sizeof(*lmu), GFP_KERNEL);
	if (!lmu)
		return -ENOMEM;

	lmu->dev = &cl->dev;

	/* Setup regmap */
	memset(&regmap_cfg, 0, sizeof(struct regmap_config));
	regmap_cfg.reg_bits = 8;
	regmap_cfg.val_bits = 8;
	regmap_cfg.name = id->name;
	regmap_cfg.max_register = data->max_register;

	lmu->regmap = devm_regmap_init_i2c(cl, &regmap_cfg);
	if (IS_ERR(lmu->regmap))
		return PTR_ERR(lmu->regmap);

	/* HW enable pin control and additional power up sequence if required */
	lmu->en_gpio = devm_gpiod_get_optional(dev, "enable", GPIOD_OUT_HIGH);
	if (IS_ERR(lmu->en_gpio)) {
		ret = PTR_ERR(lmu->en_gpio);
		dev_err(dev, "Can not request enable GPIO: %d\n", ret);
		return ret;
	}

	ret = kinetic_lmu_check_id(lmu, id->driver_data);
	if (ret)
	{
		gpiod_put(lmu->en_gpio);
		return ret;
	}

	ret = kinetic_lmu_enable_hw(lmu, id->driver_data);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(dev, kinetic_lmu_disable_hw, lmu);
	if (ret)
		return ret;

	/*
	 * Fault circuit(open/short) can be detected by kinetic-lmu-fault-monitor.
	 * After fault detection is done, some devices should re-initialize
	 * configuration. The notifier enables such kind of handling.
	 */
	BLOCKING_INIT_NOTIFIER_HEAD(&lmu->notifier);

	i2c_set_clientdata(cl, lmu);

	return devm_mfd_add_devices(lmu->dev, 0, data->cells,
				    data->num_cells, NULL, 0, NULL);
}

static const struct of_device_id kinetic_lmu_of_match[] = {
	{ .compatible = "kinetic,ktz8864a", .data = &ktz8864a_data },
	{ }
};
MODULE_DEVICE_TABLE(of, kinetic_lmu_of_match);

static const struct i2c_device_id kinetic_lmu_ids[] = {
	{ "ktz8864a", KTZ8864A },
	{ }
};
MODULE_DEVICE_TABLE(i2c, kinetic_lmu_ids);

static struct i2c_driver kinetic_lmu_driver = {
	.probe = kinetic_lmu_probe,
	.driver = {
		.name = "kinetic-lmu",
		.of_match_table = kinetic_lmu_of_match,
	},
	.id_table = kinetic_lmu_ids,
};

module_i2c_driver(kinetic_lmu_driver);

MODULE_DESCRIPTION("KINETIC LMU MFD Core Driver");
MODULE_AUTHOR("Bruce Pu <bruce.xm.pu@enskytech.com>");
MODULE_LICENSE("GPL v2");
