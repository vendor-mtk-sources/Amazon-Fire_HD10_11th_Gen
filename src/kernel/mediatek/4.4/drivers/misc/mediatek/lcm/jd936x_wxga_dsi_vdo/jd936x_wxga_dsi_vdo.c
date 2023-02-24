/*
 * Copyright (C) 2020 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#else
#include <linux/string.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#include <asm-generic/gpio.h>

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#endif
#endif

#include <mtk_leds_drv.h>
#include "lcm_drv.h"

#ifndef BUILD_LK
static unsigned int GPIO_LCD_RST;
static struct regulator *lcm_bias_pos;
static struct regulator *lcm_bias_neg;
static int lcm_bias_voltage;
static unsigned char vendor_id;
static unsigned char DDIC_id;
static unsigned char LCM_id;

#define DDIC_ID_JD			0x0
#define DDIC_ID_ILI			0x1
#define DDIC_NULL			0xF
#define LCM_VENDOR_ID_MASK		0xF
#define LCM_VENDOR_ID_SHIFT		0
#define LCM_DDIC_ID_MASK		0xF0
#define LCM_DDIC_ID_SHIFT		4
#define LCM_BIAS_DEFAULT_VOLTAGE_UV	5000000

static void lcm_request_gpio_control(struct device *dev)
{
	int ret = 0;

	dev_dbg(dev, "[Kernel/LCM] %s enter\n", __func__);
	GPIO_LCD_RST = of_get_named_gpio(dev->of_node, "gpio_lcd_rst", 0);
	ret = gpio_request(GPIO_LCD_RST, "GPIO_LCD_RST");
	if (ret)
		dev_warn(dev, "gpio request reset pin = 0x%x fail\n", GPIO_LCD_RST);
}

static int lcm_bias_regulator_probe(struct device *dev)
{
	struct device_node *np = dev_of_node(dev);
	int val;
	lcm_bias_pos = devm_regulator_get(dev, "avdd");

	if (IS_ERR(lcm_bias_pos))
	{
		dev_info(dev, "Get regulator avdd fail, find regulator avdd_alt\n");
		lcm_bias_pos = devm_regulator_get(dev, "avdd_alt");
	}

	if (IS_ERR(lcm_bias_pos))
		dev_warn(dev, "get regulator avdd fail, error: %ld\n", PTR_ERR(lcm_bias_pos));

	lcm_bias_neg = devm_regulator_get(dev, "avee");

	if (IS_ERR(lcm_bias_neg))
	{
		dev_info(dev, "Get regulator avee fail, find regulator avee_alt\n");
		lcm_bias_neg = devm_regulator_get(dev, "avee_alt");
	}

	if (IS_ERR(lcm_bias_neg))
		dev_warn(dev, "get regulator avee fail, error: %ld\n", PTR_ERR(lcm_bias_neg));

	if (!of_property_read_u32(np, "lcm-bias-voltage-uv", &val)) {
		lcm_bias_voltage = val;
	} else {
		lcm_bias_voltage = LCM_BIAS_DEFAULT_VOLTAGE_UV;
	}
	dev_info(dev, "lcm bias voltage %d uV", lcm_bias_voltage);
	return 0;
}

static int lcm_bias_enable(void)
{
	int ret = 0;

	if (!IS_ERR(lcm_bias_pos)) {
		ret = regulator_set_voltage(lcm_bias_pos, lcm_bias_voltage, lcm_bias_voltage);
		if (ret) {
			pr_err("set voltage lcm_bias_pos fail, ret = %d\n", ret);
			return ret;
		}

		ret = regulator_enable(lcm_bias_pos);
		if (ret) {
			pr_err("enable regulator lcm_bias_pos fail, ret = %d\n", ret);
			return ret;
		}
	}

	if (!IS_ERR(lcm_bias_neg)) {
		ret = regulator_set_voltage(lcm_bias_neg, lcm_bias_voltage, lcm_bias_voltage);
		if (ret) {
			pr_err("set voltage lcm_bias_neg fail, ret = %d\n", ret);
			return ret;
		}

		ret = regulator_enable(lcm_bias_neg);
		if (ret) {
			pr_err("enable regulator lcm_bias_neg fail, ret = %d\n", ret);
			return ret;
		}
	}

	return 0;
}

static void lcm_bias_disable(void)
{
	if(!IS_ERR(lcm_bias_neg))
		regulator_disable(lcm_bias_neg);

	if(!IS_ERR(lcm_bias_pos))
		regulator_disable(lcm_bias_pos);
}
static int __init setup_lcm_id(char *str)
{
	int id;

	if (get_option(&str, &id))
	{
		LCM_id = (unsigned char)id;
	}
	return 0;
}
__setup("jd936x_id=", setup_lcm_id);
static void get_lcm_id(void)
{
	vendor_id = (LCM_id & LCM_VENDOR_ID_MASK) >> LCM_VENDOR_ID_SHIFT;
	DDIC_id = (LCM_id & LCM_DDIC_ID_MASK) >> LCM_DDIC_ID_SHIFT;
}

static bool is_jd936x_lcm_ic(void)
{
	get_lcm_id();
	if(DDIC_id == DDIC_ID_JD)
		return true;
	else
		return false;
}

static int lcm_driver_probe(struct device *dev, void const *data)
{

	dev_dbg(dev, "[Kernel/LCM] %s enter\n", __func__);
	if(is_jd936x_lcm_ic() == true)
	{
		lcm_request_gpio_control(dev);
		lcm_bias_regulator_probe(dev);
		lcm_bias_enable();
	}
	return 0;
}

static const struct of_device_id lcm_platform_of_match[] = {
	{
		.compatible = "jd,jd936x",
		.data = 0,
	}, {
		/* sentinel */
	}
};


MODULE_DEVICE_TABLE(of, platform_of_match);

static int lcm_platform_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;
	dev_dbg(&pdev->dev, "[Kernel/LCM] %s enter\n", __func__);

	id = of_match_node(lcm_platform_of_match, pdev->dev.of_node);
	if (!id)
		return -ENODEV;

	return lcm_driver_probe(&pdev->dev, id->data);
}

static struct platform_driver lcm_driver = {
	.probe = lcm_platform_probe,
	.driver = {
		.name = "jd936x_wxga_dsi_vdo_hawk",
		.owner = THIS_MODULE,
		.of_match_table = lcm_platform_of_match,
	},
};

static int __init lcm_drv_init(void)
{
	pr_notice("[Kernel/LCM] %s enter\n", __func__);
	if (platform_driver_register(&lcm_driver)) {
		pr_notice("Failed to register this driver!");
		return -ENODEV;
	}

	return 0;
}

static void __exit lcm_drv_exit(void)
{
	platform_driver_unregister(&lcm_driver);
}

late_initcall(lcm_drv_init);
module_exit(lcm_drv_exit);
MODULE_AUTHOR("Mediatek");
MODULE_DESCRIPTION("LCM display subsystem driver");
MODULE_LICENSE("GPL");
#endif

/* ----------------------------------------------------------------- */
/* Local Constants */
/* ----------------------------------------------------------------- */

#define FRAME_WIDTH		(800)
#define FRAME_HEIGHT		(1280)
#define GPIO_OUT_ONE		1
#define GPIO_OUT_ZERO		0

#define REGFLAG_DELAY		0xFE
#define REGFLAG_END_OF_TABLE		0x00

#define ST_BOE		0x0
#define ST_INX		0x2
#define ST_TG	    0x3

/* ----------------------------------------------------------------- */
/*  Local Variables */
/* ----------------------------------------------------------------- */
static LCM_UTIL_FUNCS lcm_util = { 0 };
#define SET_RESET_PIN(v)		(lcm_util.set_reset_pin((v)))
#define UDELAY(n)				(lcm_util.udelay(n))
#define MDELAY(n)				(lcm_util.mdelay(n))
static unsigned char vendor_id;

/* ----------------------------------------------------------------- */
/* Local Functions */
/* ----------------------------------------------------------------- */
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
		 (lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update))
#define dsi_set_cmdq(pdata, queue_size, force_update) \
		 (lcm_util.dsi_set_cmdq(pdata, queue_size, force_update))
#define wrtie_cmd(cmd) \
		 (lcm_util.dsi_write_cmd(cmd))
#define write_regs(addr, pdata, byte_nums) \
		 (lcm_util.dsi_write_regs(addr, pdata, byte_nums))
#define read_reg \
		 (lcm_util.dsi_read_reg())
#define read_reg_v2(cmd, buffer, buffer_size) \
		 (lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size))

static void lcm_set_gpio_output(unsigned int GPIO, unsigned int output)
{
	pr_notice("[Kernel/LCM] %s enter\n", __func__);
#ifdef BUILD_LK
	mt_set_gpio_mode(GPIO, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO, output);
#else
	gpio_direction_output(GPIO, output);
	gpio_set_value(GPIO, output);
#endif
}

static void init_jd936x_inx_lcm(void)
{
	unsigned int data_array[64];

	/* password */
	data_array[0] = 0x00E01500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x93E11500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x65E21500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xF8E31500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x03801500;
	dsi_set_cmdq(data_array, 1, 1);

	/* Page1 */
	data_array[0] = 0x01E01500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00001500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x69011500;
	dsi_set_cmdq(data_array, 1, 1);

	/* Set vcom_reverse*/
	data_array[0] = 0x00031500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x6C041500;
	dsi_set_cmdq(data_array, 1, 1);

	/* Set Gamma power */
	data_array[0] = 0x00171500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xBF181500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x01191500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x001A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xBF1B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x011C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x701F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x2D201500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x2D211500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x7E221500;
	dsi_set_cmdq(data_array, 1, 1);

	/* Set SAP */
	data_array[0] = 0x28351500;
	dsi_set_cmdq(data_array, 1, 1);

	/* Set Panel */
	data_array[0] = 0x19371500;
	dsi_set_cmdq(data_array, 1, 1);

	/*SET RGBCYC*/
	data_array[0] = 0x05381500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00391500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x013A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x7C3C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xFF3D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xFF3E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x7F3F1500;
	dsi_set_cmdq(data_array, 1, 1);

	/* Set TCON */
	data_array[0] = 0x06401500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xA0411500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1E431500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0F441500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x32451500;
	dsi_set_cmdq(data_array, 1, 1);


	/* power voltage */
	data_array[0] = 0x0F551500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x68571500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0A591500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x2E5A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1A5B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x155C1500;
	dsi_set_cmdq(data_array, 1, 1);

	/* Gamma */
	data_array[0] = 0x7F5D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x645E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x535F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x47601500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x43611500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x33621500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x37631500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x21641500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x39651500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x37661500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x34671500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x50681500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x3D691500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x446A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x366B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x346C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x256D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x156E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x026F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x7F701500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x64711500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x53721500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x47731500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x43741500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x33751500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x37761500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x21771500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x39781500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x37791500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x347A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x507B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x3D7C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x447D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x367E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x347F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x25801500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x15811500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02821500;
	dsi_set_cmdq(data_array, 1, 1);
	/* Page2, for GIP */
	data_array[0] = 0x02E01500;
	dsi_set_cmdq(data_array, 1, 1);
	/* GIP_L Pin mapping */
	data_array[0] = 0x52001500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x55011500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x55021500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x50031500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x77041500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x57051500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x55061500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x4E071500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x4C081500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x5F091500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x4A0A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x480B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x550C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x460D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x440E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x400F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x55101500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x55111500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x55121500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x55131500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x55141500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x55151500;
	dsi_set_cmdq(data_array, 1, 1);
	/* GIP_R Pin mapping */
	data_array[0] = 0x53161500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x55171500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x55181500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x51191500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x771A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x571B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x551C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x4F1D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x4D1E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x5F1F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x4B201500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x49211500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x55221500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x47231500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x45241500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x41251500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x55261500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x55271500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x55281500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x55291500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x552A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x552B1500;
	dsi_set_cmdq(data_array, 1, 1);

	/*GIP_L_GS Pin mapping*/
	data_array[0] = 0x132C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x152D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x152E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x012F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x17301500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x17311500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x15321500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0D331500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0F341500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x15351500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x05361500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x07371500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x15381500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x09391500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0B3A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x113B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x153C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x153D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x153E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x153F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x15401500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x15411500;
	dsi_set_cmdq(data_array, 1, 1);

	/*GIP_R_GS Pin mapping*/
	data_array[0] = 0x12421500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x15431500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x15441500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00451500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x17461500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x17471500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x15481500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0C491500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0E4A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x154B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x044C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x064D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x154E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x084F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0A501500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x10511500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x15521500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x15531500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x15541500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x15551500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x15561500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x15571500;
	dsi_set_cmdq(data_array, 1, 1);

	/* GIP Timing */
	data_array[0] = 0x40581500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x105B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0A5C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x405D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x005E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x005F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x40601500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x03611500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x04621500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x6C631500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x6C641500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x75651500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0C661500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xB4671500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0C681500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x6C691500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x6C6A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0C6B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x006D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x006E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x886F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xBB751500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00761500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x05771500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x2E781500;
	dsi_set_cmdq(data_array, 1, 1);


	data_array[0] = 0x04E01500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x11091500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x480E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x2B2B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x032D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x442E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x05E01500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x72121500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00E01500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02E61500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0CE71500;
	dsi_set_cmdq(data_array, 1, 1);

	/*SLP OUT*/
	data_array[0] = 0x00110500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);
	/*DISP ON */
	data_array[0] = 0x00290500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(5);

	/*TE*/
	data_array[0] = 0x00351500;
	dsi_set_cmdq(data_array, 1, 1);
}

static void init_jd936x_tg_lcm(void)
{
	unsigned int data_array[64];

	data_array[0] = 0x00E01500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x93E11500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x65E21500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xF8E31500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x03801500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x01E01500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00001500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x59011500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00031500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x58041500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00171500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xEF181500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x01191500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x001A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xEF1B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x011C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x7A1F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x24201500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x24211500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x4E221500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x2A351500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x59371500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x05381500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x08391500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x103A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x7C3C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xFF3D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xFF3E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x7F3F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x06401500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xA0411500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1E431500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0F441500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x32451500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x044B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0F551500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xA5571500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0A591500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x385A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x105B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x165C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x7A5D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x605E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x525F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x44601500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x3E611500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x2E621500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x30631500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x18641500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x2F651500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x2C661500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x2B671500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x47681500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x34691500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x3A6A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x2D6B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x2A6C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F6D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x116E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x046F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x7A701500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x60711500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x52721500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x44731500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x3E741500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x2E751500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x30761500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x18771500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x2F781500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x2C791500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x2B7A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x477B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x347C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x3A7D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x2D7E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x2A7F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F801500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x11811500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x04821500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02E01500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1E001500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F011500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x57021500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x58031500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x48041500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x4A051500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x44061500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x46071500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x40081500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F091500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F0A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F0B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F0C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F0D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F0E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x420F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F101500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F111500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F121500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F131500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F141500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F151500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1E161500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F171500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x57181500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x58191500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x491A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x4B1B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x451C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x471D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x411E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F1F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F201500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F211500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F221500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F231500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F241500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x43251500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F261500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F271500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F281500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F291500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F2A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F2B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F2C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1E2D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x172E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x182F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x07301500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x05311500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0B321500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x09331500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x03341500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F351500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F361500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F371500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F381500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F391500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F3A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x013B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F3C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F3D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F3E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F3F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F401500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F411500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F421500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1E431500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x17441500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x18451500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x06461500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x04471500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0A481500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x08491500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x024A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F4B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F4C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F4D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F4E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F4F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F501500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00511500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F521500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F531500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F541500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F551500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F561500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F571500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x40581500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x305B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x055C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x305D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x015E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x025F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x30601500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x03611500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x04621500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x6A631500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x6A641500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x75651500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0D661500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x73671500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x09681500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x6A691500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x6A6A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x086B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x046D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x006E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x886F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xBC751500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00761500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0D771500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x2D781500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x04E01500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x10091500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x480E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x2B2B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x032D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x442E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x05E01500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x72121500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00E01500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02E61500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0CE71500;
	dsi_set_cmdq(data_array, 1, 1);

	/*SLP OUT*/
	data_array[0] = 0x00110500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);

	/*DISP ON */
	data_array[0] = 0x00290500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(5);

	/*TE*/
	data_array[0] = 0x00351500;
	dsi_set_cmdq(data_array, 1, 1);
}

static void init_jd936x_boe_lcm(void)
{
	unsigned int data_array[64];

	data_array[0] = 0x00E01500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x93E11500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x65E21500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xF8E31500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x03801500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x01E01500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00001500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x6F011500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00031500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xA0041500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00171500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xAF181500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x01191500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x001A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xAF1B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x011C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x3E1F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x28201500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x28211500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x7E221500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x26351500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x09371500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x04381500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00391500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x013A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x7C3C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xFF3D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xFF3E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x7F3F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x06401500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xA0411500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1E431500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0F441500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x32451500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0F551500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x69571500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0A591500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x285A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x145B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x155C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x7C5D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x655E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x555F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x47601500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x43611500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x32621500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x34631500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1C641500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x33651500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x31661500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x30671500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x4E681500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x3C691500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x446A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x356B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x316C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x236D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x116E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x006F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x7C701500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x65711500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x55721500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x47731500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x43741500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x32751500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x34761500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1C771500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x33781500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x31791500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x307A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x4E7B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x3C7C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x447D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x357E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x317F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x23801500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x11811500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00821500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02E01500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1E001500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1E011500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x41021500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x41031500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x43041500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x43051500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F061500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F071500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x35081500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F091500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x150A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x150B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F0C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x470D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x470E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x450F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x45101500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x4B111500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x4B121500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x49131500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x49141500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F151500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1E161500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1E171500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x40181500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x40191500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x421A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x421B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F1C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F1D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x351E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F1F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x15201500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x15211500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F221500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x46231500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x46241500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x44251500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x44261500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x4A271500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x4A281500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x48291500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x482A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F2B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F2C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F2D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x022E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x022F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00301500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00311500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1E321500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1E331500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x35341500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F351500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x15361500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x15371500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F381500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x08391500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x083A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0A3B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0A3C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x043D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x043E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x063F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x06401500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F411500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F421500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F431500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x03441500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x03451500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x01461500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x01471500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1E481500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1E491500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x354A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F4B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x154C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x154D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F4E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x094F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x09501500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0B511500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0B521500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x05531500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x05541500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x07551500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x07561500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F571500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x40581500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x305B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x075C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x305D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x015E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x025F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x14631500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x6A641500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x73671500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x09681500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x14691500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x6A6A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x086B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x006C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x006D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x006E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x886F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xDD771500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0E791500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x077A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x147D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x6A7E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x04E01500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x11091500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x480E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x2B2B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x032D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x442E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x05E01500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x72121500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00E01500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02E61500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0CE71500;
	dsi_set_cmdq(data_array, 1, 1);

	/*SLP OUT*/
	data_array[0] = 0x00110500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);

	/*DISP ON */
	data_array[0] = 0x00290500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(5);

	/*TE*/
	data_array[0] = 0x00351500;
	dsi_set_cmdq(data_array, 1, 1);
}

static void init_jd936x_lcm(void)
{
	get_lcm_id();
	if (vendor_id == ST_INX)
		init_jd936x_inx_lcm();		/* JD936X INX panel */
	else if (vendor_id == ST_TG)
		init_jd936x_tg_lcm();		/* JD936X KD panel, INX glass */
	else
		init_jd936x_boe_lcm();			/* Default JD936X BOE panel */

	pr_debug("[Kernel/LCM] jd936x vendor_id %d\n", vendor_id);
}

/* ----------------------------------------------------------------- */
/* LCM Driver Implementations */
/* ----------------------------------------------------------------- */
static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));
	get_lcm_id();
	params->type   = LCM_TYPE_DSI;

	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	params->dsi.mode   = SYNC_EVENT_VDO_MODE;

	params->dsi.LANE_NUM				= LCM_FOUR_LANE;
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding	= LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format	= LCM_DSI_FORMAT_RGB888;

	params->dsi.packet_size = 256;

	params->dsi.intermediat_buffer_num = 0;

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	if (vendor_id == ST_TG) {
		pr_notice("[Kernel/LCM] %s panel\n", "TG");
		params->dsi.vertical_sync_active		= 4;
		params->dsi.vertical_backporch			= 12;
		params->dsi.vertical_frontporch 		= 30;
		params->dsi.vertical_active_line		= FRAME_HEIGHT;

		params->dsi.horizontal_sync_active		= 20;
		params->dsi.horizontal_backporch		= 30;
		params->dsi.horizontal_frontporch		= 30;
		params->dsi.horizontal_active_pixel	= FRAME_WIDTH;

		params->dsi.PLL_CLOCK = 234;

	} else if (vendor_id == ST_INX) {
		pr_notice("[Kernel/LCM] %s panel\n", "INX");
		params->dsi.vertical_sync_active		= 4;
		params->dsi.vertical_backporch			= 12;
		params->dsi.vertical_frontporch 		= 30;
		params->dsi.vertical_active_line		= FRAME_HEIGHT;

		params->dsi.horizontal_sync_active		= 20;
		params->dsi.horizontal_backporch		= 30;
		params->dsi.horizontal_frontporch		= 30;
		params->dsi.horizontal_active_pixel	= FRAME_WIDTH;

		params->dsi.PLL_CLOCK = 234;
	} else {
		pr_notice("[Kernel/LCM] %s panel\n", "BOE");
		params->dsi.vertical_sync_active		= 4;
		params->dsi.vertical_backporch			= 12;
		params->dsi.vertical_frontporch			= 30;
		params->dsi.vertical_active_line                = FRAME_HEIGHT;

		params->dsi.horizontal_sync_active              = 20;
		params->dsi.horizontal_backporch                = 30;
		params->dsi.horizontal_frontporch               = 30;
		params->dsi.horizontal_active_pixel     = FRAME_WIDTH;
		params->dsi.PLL_CLOCK = 234;
	}
	params->dsi.vertical_frontporch_for_low_power = 295;
	params->dsi.clk_lp_per_line_enable = 1;
	params->dsi.cont_clock = 0;
	params->dsi.DA_HS_EXIT = 10;
	params->dsi.CLK_ZERO = 16;
	params->dsi.HS_ZERO = 10;
	params->dsi.HS_TRAIL = 5;
	params->dsi.CLK_TRAIL = 5;
	params->dsi.CLK_HS_POST = 10;
	params->dsi.CLK_HS_EXIT = 6;
	params->dsi.CLK_HS_PRPR = 4;

	params->dsi.TA_GO = 8;
	params->dsi.TA_GET = 10;

	params->physical_width = 108;
	params->physical_height = 172;
}

static void lcm_init(void)
{
	pr_notice("[Kernel/LCM] %s enter\n", __func__);
	init_jd936x_lcm();
}

static void lcm_resume(void)
{
	pr_notice("[Kernel/LCM] %s enter\n", __func__);
	if (vendor_id == ST_TG) {
		lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ONE);
		MDELAY(5);
		lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ZERO);
		MDELAY(10);
		lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ONE);
		MDELAY(200);
	} else if(vendor_id == ST_BOE) {
		lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ONE);
		MDELAY(10);
		lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ZERO);
		MDELAY(2);
		lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ONE);
		MDELAY(200);
	} else {
		lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ONE);
		MDELAY(10);
		lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ZERO);
		MDELAY(2);
		lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ONE);
		MDELAY(6);
	}
	init_jd936x_lcm(); /* TPV panel */
}

static void lcm_init_power(void)
{
	pr_notice("[Kernel/LCM] %s enter\n", __func__);
	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ONE);
	MDELAY(10);
	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ZERO);
	MDELAY(2);
	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ONE);
	MDELAY(5);
}

static void lcm_resume_power(void)
{
	pr_notice("[Kernel/LCM] %s enter\n", __func__);
	lcm_bias_enable();
	/* tRPWIRES + tRESETL = 5ms + 1ms = 6ms */
	MDELAY(6);
	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ONE);
}

static void lcm_suspend(void)
{
	unsigned int data_array[16];


	pr_notice("[Kernel/LCM] %s enter\n", __func__);

	data_array[0] = 0x00280500; /* Display Off */
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(50);

	data_array[0] = 0x00100500; /* Sleep In */
	dsi_set_cmdq(data_array, 1, 1);

	MDELAY(120);
}

static void lcm_suspend_power(void)
{
	pr_notice("[Kernel/LCM] %s enter\n", __func__);

	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ZERO);
	MDELAY(120);

	lcm_bias_disable();
	MDELAY(20);
}

static void lcm_set_backlight_cmdq(void *handle, unsigned int level)
{
	backlight_brightness_set(level);
}


LCM_DRIVER jd936x_wxga_dsi_vdo_lcm_drv = {
	.name		= "jd936x_wxga_dsi_vdo",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.init_power	= lcm_init_power,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.resume_power	= lcm_resume_power,
	.suspend_power	= lcm_suspend_power,
	.set_backlight_cmdq	= lcm_set_backlight_cmdq,
};


