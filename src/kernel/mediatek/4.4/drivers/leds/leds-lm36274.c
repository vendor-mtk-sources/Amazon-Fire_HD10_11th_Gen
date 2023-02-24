// SPDX-License-Identifier: GPL-2.0
// TI LM36274 LED chip family driver
// Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com/

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/leds.h>
#include <linux/leds-ti-lmu-common.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>

#include <linux/mfd/ti-lmu.h>
#include <linux/mfd/ti-lmu-register.h>

#include <uapi/linux/uleds.h>

#define LM36274_MAX_STRINGS	4
#define LM36274_BL_EN		BIT(4)

/**
 * struct lm36274
 * @pdev: platform device
 * @led_dev: led class device
 * @lmu_data: Register and setting values for common code
 * @regmap: Devices register map
 * @dev: Pointer to the devices device struct
 * @led_sources - The LED strings supported in this array
 * @num_leds - Number of LED strings are supported in this array
 */
struct lm36274 {
	struct platform_device *pdev;
	struct led_classdev led_dev;
	struct ti_lmu_bank lmu_data;
	struct regmap *regmap;
	struct device *dev;

	u32 led_sources[LM36274_MAX_STRINGS];
	int num_leds;
	u8 ovp_v;
	u8 pwm_mode;
	bool pwm_en_high_sample_rate;
	u8 boost_cur_limit;
};

static int lm36274_brightness_set(struct led_classdev *led_cdev,
				enum led_brightness brt_val)
{
	struct lm36274 *led = container_of(led_cdev, struct lm36274, led_dev);

	return ti_lmu_common_set_brightness(&led->lmu_data, brt_val);
}

static int lm36274_init_led(struct lm36274 *lm36274_data)
{
	return lm36274_brightness_set(&lm36274_data->led_dev, lm36274_data->led_dev.brightness);
}

static int lm36274_init(struct lm36274 *lm36274_data)
{
	int enable_val = 0;
	int i, ret;

	/* I2C PWM Pin Enable */
	switch (lm36274_data->pwm_mode << LM36274_PWM_SHIFT) {
	case LM36274_PWM_DISABLE:
	case LM36274_PWM_ENABLE:
		ret = regmap_update_bits(lm36274_data->regmap,
					  LM36274_REG_BL_CFG_1, LM36274_PWM_MASK,
					  lm36274_data->pwm_mode << LM36274_PWM_SHIFT);
		if (ret) {
			dev_err(lm36274_data->dev, "Failed to update BL_CFG_1 Reg\n");
			return ret;
		}
		break;
	default:
		dev_err(lm36274_data->dev, "invalid pwm value: %d\n", lm36274_data->pwm_mode);
		return -EINVAL;
	}

	/* Enable pwm high sample rate */
	if (lm36274_data->pwm_en_high_sample_rate) {
		ret = regmap_update_bits(lm36274_data->regmap,
					  LM36274_REG_OPTION_1, LM36274_HIGH_PWM_MASK,
					  LM36274_HIGH_PWM);
		if (ret) {
			dev_err(lm36274_data->dev, "Failed to update OPTION_1 Reg\n");
			return ret;
		}
	}

	/* Set backlight boost current limit */
	switch (lm36274_data->boost_cur_limit << LM36274_BOOST_CUR_LIMIT_SHIFT) {
	case LM36274_BOOST_CUR_LIMIT_0_9_A:
	case LM36274_BOOST_CUR_LIMIT_1_2_A:
	case LM36274_BOOST_CUR_LIMIT_1_5_A:
	case LM36274_BOOST_CUR_LIMIT_1_8_A:
		ret = regmap_update_bits(lm36274_data->regmap,
					  LM36274_REG_OPTION_2, LM36274_BOOST_CUR_LIMIT_MASK,
					  lm36274_data->boost_cur_limit << LM36274_BOOST_CUR_LIMIT_SHIFT);
		if (ret) {
			dev_err(lm36274_data->dev, "Failed to update OPTION_2 Reg\n");
			return ret;
		}
		break;
	default:
		dev_err(lm36274_data->dev, "invalid boost cur limit value: %d\n",
			lm36274_data->boost_cur_limit);
		return -EINVAL;
	}

	/*	I2C LED Strings Enable */
	for (i = 0; i < lm36274_data->num_leds; i++)
		enable_val |= (1 << lm36274_data->led_sources[i]);

	if (!enable_val) {
		dev_err(lm36274_data->dev, "No LEDs were enabled\n");
		return -EINVAL;
	}

	enable_val |= LM36274_BL_EN;

	ret = regmap_write(lm36274_data->regmap, LM36274_REG_BL_EN,
			   enable_val);
	if (ret)
		return ret;

	/* Set LED Brightness */
	ret = lm36274_init_led(lm36274_data);
	if (ret)
		return ret;

	/* Set Display OVP */
	switch (lm36274_data->ovp_v << LM36274_OVP_SHIFT) {
	case LM36274_OVP_17V:
	case LM36274_OVP_21V:
	case LM36274_OVP_25V:
	case LM36274_OVP_29V:
		return regmap_update_bits(lm36274_data->regmap,
					  LM36274_REG_BL_CFG_1, LM36274_OVP_MASK,
					  lm36274_data->ovp_v << LM36274_OVP_SHIFT);
	}

	return -EINVAL;
}

/**
 * As the flags register will be cleaned after readback, we have to read and parse
 * each flag value at the same readback.
 */
static ssize_t lm36274_flags_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct lm36274 *lm36274_data = container_of(led_cdev, struct lm36274, led_dev);
	struct regmap *regmap = lm36274_data->regmap;
	unsigned int value = 0;
	int ret;

	ret = regmap_read(regmap, LM36274_REG_FLAGS, &value);
	if (ret) {
		dev_err(lm36274_data->dev, "Failed to read FLAGS Reg\n");
		return ret;
	}
	buf[0] = 0;

	/* For sysfs node, the buffer will always be PAGE_SIZE bytes in length */
	if ((value & LM36274_BL_OCP_MASK) && ((strlen(buf) + strlen("bl_ocp ")) < PAGE_SIZE))
		strncat(buf, "bl_ocp ", strlen("bl_ocp "));
	if ((value & LM36274_BL_OVP_MASK) && (((strlen(buf) + strlen("bl_ovp ")) < PAGE_SIZE)))
		strncat(buf, "bl_ovp ", strlen("bl_ovp "));
	if ((value & LM36274_VNEG_SHORT_MASK) && (((strlen(buf) + strlen("vneg_short ")) < PAGE_SIZE)))
		strncat(buf, "vneg_short ", strlen("vneg_short "));
	if ((value & LM36274_VPOS_SHORT_MASK) && (((strlen(buf) + strlen("vpos_short ")) < PAGE_SIZE)))
		strncat(buf, "vpos_short ", strlen("vpos_short "));
	if ((value & LM36274_LCM_OVP_MASK) && (((strlen(buf) + strlen("lcm_ovp ")) < PAGE_SIZE)))
		strncat(buf, "lcm_ovp ", strlen("lcm_ovp "));
	if ((value & LM36274_TSD_MASK) && (((strlen(buf) + strlen("tsd ")) < PAGE_SIZE)))
		strncat(buf, "tsd ", strlen("tsd "));

	if (((strlen(buf) + strlen("\n")) < PAGE_SIZE))
		strncat(buf, "\n", strlen("\n"));

	return strlen(buf);
}

static DEVICE_ATTR_RO(lm36274_flags);

static struct attribute *lm36274_attrs[] = {
	&dev_attr_lm36274_flags.attr,
	NULL
};
ATTRIBUTE_GROUPS(lm36274);

static int lm36274_parse_dt(struct lm36274 *lm36274_data)
{
	struct fwnode_handle *child = NULL;
	char label[LED_MAX_NAME_SIZE];
	struct device *dev = &lm36274_data->pdev->dev;
	const char *name;
	int child_cnt;
	int ret = -EINVAL;

	/* There should only be 1 node */
	child_cnt = device_get_child_node_count(dev);
	if (child_cnt != 1)
		return -EINVAL;

	ret = of_property_read_u8(dev->of_node, "ovp-voltage", &lm36274_data->ovp_v);
	if (ret)
		lm36274_data->ovp_v = LM36274_OVP_17V;

	ret = of_property_read_u8(dev->of_node, "pwm-mode", &lm36274_data->pwm_mode);
	if (ret)
		lm36274_data->pwm_mode = LM36274_PWM_DISABLE;

	ret = of_property_read_u8(dev->of_node, "boost-current-limit",
				&lm36274_data->boost_cur_limit);
	if (ret)
		lm36274_data->boost_cur_limit = LM36274_BOOST_CUR_LIMIT_1_2_A;

	lm36274_data->pwm_en_high_sample_rate = of_property_read_bool(dev->of_node,
					"pwm-enable-high-sample-rate");

	device_for_each_child_node(dev, child) {
		ret = fwnode_property_read_string(child, "label", &name);
		if (ret)
			snprintf(label, sizeof(label),
				"%s::", lm36274_data->pdev->name);
		else
			snprintf(label, sizeof(label),
				 "%s:%s", lm36274_data->pdev->name, name);

		lm36274_data->num_leds = fwnode_property_read_u32_array(child,
							  "led-sources",
							  NULL, 0);
		if (lm36274_data->num_leds <= 0)
			return -ENODEV;

		ret = fwnode_property_read_u32_array(child, "led-sources",
						     lm36274_data->led_sources,
						     lm36274_data->num_leds);
		if (ret) {
			dev_err(dev, "led-sources property missing\n");
			return ret;
		}

		fwnode_property_read_string(child, "linux,default-trigger",
					&lm36274_data->led_dev.default_trigger);
	}

	lm36274_data->lmu_data.regmap = lm36274_data->regmap;
	lm36274_data->lmu_data.max_brightness = MAX_BRIGHTNESS_11BIT;
	lm36274_data->lmu_data.msb_brightness_reg = LM36274_REG_BRT_MSB;
	lm36274_data->lmu_data.lsb_brightness_reg = LM36274_REG_BRT_LSB;

	lm36274_data->led_dev.name = label;
	lm36274_data->led_dev.max_brightness = MAX_BRIGHTNESS_11BIT;

	ret = of_property_read_u32(dev->of_node, "default-brightness", &lm36274_data->led_dev.brightness);
	if (ret || lm36274_data->led_dev.brightness > lm36274_data->led_dev.max_brightness)
		lm36274_data->led_dev.brightness = lm36274_data->led_dev.max_brightness;
	dev_info(lm36274_data->dev, "set default-brightness as %d\n", lm36274_data->led_dev.brightness);

	lm36274_data->led_dev.brightness_set_blocking = lm36274_brightness_set;
	lm36274_data->led_dev.groups = lm36274_groups;

	return 0;
}

static int lm36274_probe(struct platform_device *pdev)
{
	struct ti_lmu *lmu = dev_get_drvdata(pdev->dev.parent);
	struct lm36274 *lm36274_data;
	int ret;

	lm36274_data = devm_kzalloc(&pdev->dev, sizeof(*lm36274_data),
				    GFP_KERNEL);
	if (!lm36274_data)
		return -ENOMEM;

	lm36274_data->pdev = pdev;
	lm36274_data->dev = lmu->dev;
	lm36274_data->regmap = lmu->regmap;
	dev_set_drvdata(&pdev->dev, lm36274_data);

	ret = lm36274_parse_dt(lm36274_data);
	if (ret) {
		dev_err(lm36274_data->dev, "Failed to parse DT node\n");
		return ret;
	}

	ret = lm36274_init(lm36274_data);
	if (ret) {
		dev_err(lm36274_data->dev, "Failed to init the device\n");
		return ret;
	}

	return devm_led_classdev_register(lm36274_data->dev,
					 &lm36274_data->led_dev);
}

static const struct of_device_id of_lm36274_leds_match[] = {
	{ .compatible = "ti,lm36274-backlight", },
	{},
};
MODULE_DEVICE_TABLE(of, of_lm36274_leds_match);

static struct platform_driver lm36274_driver = {
	.probe  = lm36274_probe,
	.driver = {
		.name = "lm36274-leds",
	},
};
module_platform_driver(lm36274_driver)

MODULE_DESCRIPTION("Texas Instruments LM36274 LED driver");
MODULE_AUTHOR("Dan Murphy <dmurphy@ti.com>");
MODULE_LICENSE("GPL v2");
