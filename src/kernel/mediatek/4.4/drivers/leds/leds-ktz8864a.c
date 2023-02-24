/*
 * KINETIC KTZ8864A LED chip family driver
 *
 * Author: Bruce pu <bruce.xm.pu@enskytech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>


#include <linux/leds-ktz8864a.h>
#include <linux/mfd/kinetic-lmu.h>
#include <linux/mfd/kinetic-lmu-register.h>

#include <uapi/linux/uleds.h>

#define KTZ8864A_MAX_STRINGS	4
#define KTZ8864A_BL_EN		BIT(4)

/**
 * struct ktz8864a
 * @pdev: platform device
 * @led_dev: led class device
 * @lmu_data: Register and setting values for common code
 * @regmap: Devices register map
 * @dev: Pointer to the devices device struct
 * @led_sources - The LED strings supported in this array
 * @num_leds - Number of LED strings are supported in this array
 */
struct ktz8864a {
	struct platform_device *pdev;
	struct led_classdev led_dev;
	struct kinetic_lmu_bank lmu_data;
	struct regmap *regmap;
	struct device *dev;

	u32 led_sources[KTZ8864A_MAX_STRINGS];
	int num_leds;
	u8 ovp_v;
	u8 pwm_mode;
	bool pwm_en_high_sample_rate;
	u8 boost_cur_limit;
};

static int kinetic_lmu_common_update_brightness(struct kinetic_lmu_bank *lmu_bank,
					   int brightness)
{
	struct regmap *regmap = lmu_bank->regmap;
	u8 reg, val;
	int ret;

	/*
	 * Brightness register update
	 *
	 * 11 bit dimming: update LSB bits and write MSB byte.
	 *		   MSB brightness should be shifted.
	 *  8 bit dimming: write MSB byte.
	 */
	if (lmu_bank->max_brightness == MAX_BRIGHTNESS_11BIT) {
		reg = lmu_bank->lsb_brightness_reg;
		ret = regmap_update_bits(regmap, reg,
					 LMU_11BIT_LSB_MASK,
					 brightness);
		if (ret)
			return ret;

		val = brightness >> LMU_11BIT_MSB_SHIFT;
	} else {
		val = brightness;
	}

	reg = lmu_bank->msb_brightness_reg;

	return regmap_write(regmap, reg, val);
}

static int kinetic_lmu_common_set_brightness(struct kinetic_lmu_bank *lmu_bank, int brightness)
{
	return kinetic_lmu_common_update_brightness(lmu_bank, brightness);
}

static int ktz8864a_brightness_set(struct led_classdev *led_cdev,
				enum led_brightness brt_val)
{
	struct ktz8864a *led = container_of(led_cdev, struct ktz8864a, led_dev);

	return kinetic_lmu_common_set_brightness(&led->lmu_data, brt_val);
}

static int ktz8864a_init_led(struct ktz8864a *ktz8864a_data)
{
	return ktz8864a_brightness_set(&ktz8864a_data->led_dev, ktz8864a_data->led_dev.brightness);
}

static int ktz8864a_init(struct ktz8864a *ktz8864a_data)
{
	int enable_val = 0;
	int i, ret;

	/* I2C PWM Pin Enable */
	switch (ktz8864a_data->pwm_mode << KTZ8864A_PWM_SHIFT) {
	case KTZ8864A_PWM_DISABLE:
	case KTZ8864A_PWM_ENABLE:
		ret = regmap_update_bits(ktz8864a_data->regmap,
					  KTZ8864A_REG_BL_CFG_1, KTZ8864A_PWM_MASK,
					  ktz8864a_data->pwm_mode << KTZ8864A_PWM_SHIFT);
		if (ret) {
			dev_err(ktz8864a_data->dev, "Failed to update BL_CFG_1 Reg\n");
			return ret;
		}
		break;
	default:
		dev_err(ktz8864a_data->dev, "invalid pwm value: %d\n", ktz8864a_data->pwm_mode);
		return -EINVAL;
	}

	/* Set backlight boost current limit */
	switch (ktz8864a_data->boost_cur_limit << KTZ8864A_BOOST_CUR_LIMIT_SHIFT) {
	case KTZ8864A_BOOST_CUR_LIMIT_1_2_A:
	case KTZ8864A_BOOST_CUR_LIMIT_1_5_A:
	case KTZ8864A_BOOST_CUR_LIMIT_1_8_A:
	case KTZ8864A_BOOST_CUR_LIMIT_2_1_A:
		ret = regmap_update_bits(ktz8864a_data->regmap,
					  KTZ8864A_REG_OPTION_2, KTZ8864A_BOOST_CUR_LIMIT_MASK,
					  ktz8864a_data->boost_cur_limit << KTZ8864A_BOOST_CUR_LIMIT_SHIFT);
		if (ret) {
			dev_err(ktz8864a_data->dev, "Failed to update OPTION_2 Reg\n");
			return ret;
		}
		break;
	default:
		dev_err(ktz8864a_data->dev, "invalid boost cur limit value: %d\n",
			ktz8864a_data->boost_cur_limit);
		return -EINVAL;
	}

	/*	I2C LED Strings Enable */
	for (i = 0; i < ktz8864a_data->num_leds; i++)
		enable_val |= (1 << ktz8864a_data->led_sources[i]);

	if (!enable_val) {
		dev_err(ktz8864a_data->dev, "No LEDs were enabled\n");
		return -EINVAL;
	}

	enable_val |= KTZ8864A_BL_EN;

	ret = regmap_write(ktz8864a_data->regmap, KTZ8864A_REG_BL_EN,
			   enable_val);
	if (ret)
		return ret;

	/* Set LED Brightness */
	ret = ktz8864a_init_led(ktz8864a_data);
	if (ret)
		return ret;

	/* Set Display OVP */
	switch (ktz8864a_data->ovp_v << KTZ8864A_OVP_SHIFT) {
	case KTZ8864A_OVP_17V:
	case KTZ8864A_OVP_21V:
	case KTZ8864A_OVP_25V:
	case KTZ8864A_OVP_29V:
		return regmap_update_bits(ktz8864a_data->regmap,
					  KTZ8864A_REG_BL_CFG_1, KTZ8864A_OVP_MASK,
					  ktz8864a_data->ovp_v << KTZ8864A_OVP_SHIFT);
	}

	return -EINVAL;
}

/**
 * As the flags register will be cleaned after readback, we have to read and parse
 * each flag value at the same readback.
 */
static ssize_t ktz8864a_flags_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct ktz8864a *ktz8864a_data = container_of(led_cdev, struct ktz8864a, led_dev);
	struct regmap *regmap = ktz8864a_data->regmap;
	unsigned int value = 0;
	int ret;

	ret = regmap_read(regmap, KTZ8864A_REG_FLAGS, &value);
	if (ret) {
		dev_err(ktz8864a_data->dev, "Failed to read FLAGS Reg\n");
		return ret;
	}
	buf[0] = 0;

	/* For sysfs node, the buffer will always be PAGE_SIZE bytes in length */
	if ((value & KTZ8864A_BL_OCP_MASK) && ((strlen(buf) + strlen("bl_ocp ")) < PAGE_SIZE))
		strncat(buf, "bl_ocp ", strlen("bl_ocp "));
	if ((value & KTZ8864A_BL_OVP_MASK) && (((strlen(buf) + strlen("bl_ovp ")) < PAGE_SIZE)))
		strncat(buf, "bl_ovp ", strlen("bl_ovp "));
	if ((value & KTZ8864A_VNEG_SHORT_MASK) && (((strlen(buf) + strlen("vneg_short ")) < PAGE_SIZE)))
		strncat(buf, "vneg_short ", strlen("vneg_short "));
	if ((value & KTZ8864A_VPOS_SHORT_MASK) && (((strlen(buf) + strlen("vpos_short ")) < PAGE_SIZE)))
		strncat(buf, "vpos_short ", strlen("vpos_short "));
	if ((value & KTZ8864A_TSD_MASK) && (((strlen(buf) + strlen("tsd ")) < PAGE_SIZE)))
		strncat(buf, "tsd ", strlen("tsd "));
	if ((value & KTZ8864A_LED_SHORT_MASK) && (((strlen(buf) + strlen("led_short ")) < PAGE_SIZE)))
		strncat(buf, "led_short ", strlen("led_short "));

	if (((strlen(buf) + strlen("\n")) < PAGE_SIZE))
		strncat(buf, "\n", strlen("\n"));

	return strlen(buf);
}

static DEVICE_ATTR_RO(ktz8864a_flags);

static struct attribute *ktz8864a_attrs[] = {
	&dev_attr_ktz8864a_flags.attr,
	NULL
};
ATTRIBUTE_GROUPS(ktz8864a);

static int ktz8864a_parse_dt(struct ktz8864a *ktz8864a_data)
{
	struct fwnode_handle *child = NULL;
	char label[LED_MAX_NAME_SIZE];
	struct device *dev = &ktz8864a_data->pdev->dev;
	const char *name;
	int child_cnt;
	int ret = -EINVAL;

	/* There should only be 1 node */
	child_cnt = device_get_child_node_count(dev);
	if (child_cnt != 1)
		return -EINVAL;

	ret = of_property_read_u8(dev->of_node, "ovp-voltage", &ktz8864a_data->ovp_v);
	if (ret)
		ktz8864a_data->ovp_v = KTZ8864A_OVP_17V;

	ret = of_property_read_u8(dev->of_node, "pwm-mode", &ktz8864a_data->pwm_mode);
	if (ret)
		ktz8864a_data->pwm_mode = KTZ8864A_PWM_DISABLE;

	ret = of_property_read_u8(dev->of_node, "boost-current-limit",
				&ktz8864a_data->boost_cur_limit);
	if (ret)
		ktz8864a_data->boost_cur_limit = KTZ8864A_BOOST_CUR_LIMIT_1_2_A;

	device_for_each_child_node(dev, child) {
		ret = fwnode_property_read_string(child, "label", &name);
		if (ret)
			snprintf(label, sizeof(label),
				"%s::", ktz8864a_data->pdev->name);
		else
			snprintf(label, sizeof(label),
				 "%s:%s", ktz8864a_data->pdev->name, name);

		ktz8864a_data->num_leds = fwnode_property_read_u32_array(child,
							  "led-sources",
							  NULL, 0);
		if (ktz8864a_data->num_leds <= 0)
			return -ENODEV;

		ret = fwnode_property_read_u32_array(child, "led-sources",
						     ktz8864a_data->led_sources,
						     ktz8864a_data->num_leds);
		if (ret) {
			dev_err(dev, "led-sources property missing\n");
			return ret;
		}

		fwnode_property_read_string(child, "linux,default-trigger",
					&ktz8864a_data->led_dev.default_trigger);
	}

	ktz8864a_data->lmu_data.regmap = ktz8864a_data->regmap;
	ktz8864a_data->lmu_data.max_brightness = MAX_BRIGHTNESS_11BIT;
	ktz8864a_data->lmu_data.msb_brightness_reg = KTZ8864A_REG_BRT_MSB;
	ktz8864a_data->lmu_data.lsb_brightness_reg = KTZ8864A_REG_BRT_LSB;

	ktz8864a_data->led_dev.name = label;
	ktz8864a_data->led_dev.max_brightness = MAX_BRIGHTNESS_11BIT;

	ret = of_property_read_u32(dev->of_node, "default-brightness", &ktz8864a_data->led_dev.brightness);
	if (ret || ktz8864a_data->led_dev.brightness > ktz8864a_data->led_dev.max_brightness)
		ktz8864a_data->led_dev.brightness = ktz8864a_data->led_dev.max_brightness;
	dev_info(ktz8864a_data->dev, "set default-brightness as %d\n", ktz8864a_data->led_dev.brightness);

	ktz8864a_data->led_dev.brightness_set_blocking = ktz8864a_brightness_set;
	ktz8864a_data->led_dev.groups = ktz8864a_groups;

	return 0;
}

static int ktz8864a_probe(struct platform_device *pdev)
{
	struct kinetic_lmu *lmu = dev_get_drvdata(pdev->dev.parent);
	struct ktz8864a *ktz8864a_data;
	int ret;

	ktz8864a_data = devm_kzalloc(&pdev->dev, sizeof(*ktz8864a_data),
				    GFP_KERNEL);
	if (!ktz8864a_data)
		return -ENOMEM;

	ktz8864a_data->pdev = pdev;
	ktz8864a_data->dev = lmu->dev;
	ktz8864a_data->regmap = lmu->regmap;
	dev_set_drvdata(&pdev->dev, ktz8864a_data);

	ret = ktz8864a_parse_dt(ktz8864a_data);
	if (ret) {
		dev_err(ktz8864a_data->dev, "Failed to parse DT node\n");
		return ret;
	}

	ret = ktz8864a_init(ktz8864a_data);
	if (ret) {
		dev_err(ktz8864a_data->dev, "Failed to init the device\n");
		return ret;
	}

	return devm_led_classdev_register(ktz8864a_data->dev,
					 &ktz8864a_data->led_dev);
}

static const struct of_device_id of_ktz8864a_leds_match[] = {
	{ .compatible = "kinetic,ktz8864a-backlight", },
	{},
};
MODULE_DEVICE_TABLE(of, of_ktz8864a_leds_match);

static struct platform_driver ktz8864a_driver = {
	.probe  = ktz8864a_probe,
	.driver = {
		.name = "ktz8864a-leds",
	},
};
module_platform_driver(ktz8864a_driver)

MODULE_DESCRIPTION("Kinetic LED driver");
MODULE_AUTHOR("Bruce Pu <bruce.xm.pu@enskytech.com>");
MODULE_LICENSE("GPL v2");
