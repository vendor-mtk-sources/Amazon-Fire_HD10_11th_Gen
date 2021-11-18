/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/string.h>
#include <linux/regulator/consumer.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/syscore_ops.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#include <linux/i2c.h>
#else
#include <string.h>
#endif

#include "lcm_drv.h"

#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <platform/mt_pmic.h>
#endif



#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#endif

#define LOG_TAG "LCM-NT51021-MVK"

#define LCM_LOGI(fmt, args...)  pr_notice("[KERNEL/"LOG_TAG"]"fmt, ##args)
#define LCM_DBG(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#define LCM_ERR(fmt, args...)  pr_err("[KERNEL/"LOG_TAG"]"fmt, ##args)

/*
* 1. lcm_i2c_driver
* 2. lcm_platform_driver
* 3. lcm_driver
*/

#define VID_INX_OTA7290B	0x0		/* Innolux with OTA7290B */
#define VID_BOE_NT51021B	0x1		/* BOE with NT51021B*/
#define VID_KD_OTA7290B		0x2		/* KD with OTA7290B */
#define VID_NULL			0xff	/* NULL */

#ifdef BUILD_LK
#else

static unsigned int GPIO_LCD_RST_EN;
static unsigned int LCM_ID1_GPIO;
static unsigned int LCM_ID0_GPIO;

static struct regulator *reg_lcm_rst;
static struct regulator *reg_lcm_vcc3v3;

static unsigned int g_vendor_id = VID_NULL;
static int cmd_sel = -1;


/*
* unsigned int idme_get_board_type(void);
* unsigned int idme_get_board_rev(void);
* #ifdef WITH_ENABLE_IDME
* char *board_version(void);
*/

#define GetField(Var, Mask, Shift) \
	(((Var) >> (Shift)) & (Mask))

#define SetField(Var, Mask, Shift, Val) \
	((Var) = (((Var) & ~((Mask) << (Shift))) | \
				(((Val) & (Mask)) << (Shift))))

#define NT51021_CABC_ENB_ADDR		0x89
#define NT51021_CABC_ENB_MASK		0x03
#define NT51021_CABC_ENB_SHIFT		0x06

static unsigned char nt51021_cabc_regs[][2] = {
	{0x83, 0xBB,},		/* Page Selection1 */
	{0x84, 0x22,},		/* Page Selection2 */
	{0x90, 0xC0,},		/* CABC_ENB, 0xC0 is CABC_OFf */
};

static unsigned char CABC_mapping_table[] = {
	0x03,	/* CABC_OFF		HH */
	0x00,	/* CABC_MOVIE	LL */
	0x01,	/* CABC_STILL	LH */
	0x02,	/* CABC_UI		HL not support in panel*/
};

#endif		/* #ifdef BUILD_LK */


static unsigned int get_lcm_id(void)
{
	unsigned int vendor_id = VID_NULL;

#ifdef BUILD_LK
	unsigned int ret = 0;

	ret = mt_set_gpio_mode(LCM_ID1_GPIO, GPIO_MODE_00);
	if (ret != 0)
		LCM_LOGI("ID1 mt_set_gpio_mode fail");

	ret = mt_set_gpio_dir(LCM_ID1_GPIO, GPIO_DIR_IN);
	if (ret != 0)
		LCM_LOGI("ID1 mt_set_gpio_dir fail");

	ret = mt_set_gpio_mode(LCM_ID0_GPIO, GPIO_MODE_00);
	if (ret != 0)
		LCM_LOGI("ID0 mt_set_gpio_mode fail");

	ret = mt_set_gpio_dir(LCM_ID0_GPIO, GPIO_DIR_IN);
	if (ret != 0)
		LCM_LOGI("ID0 mt_set_gpio_dir fail");

	/* get ID pin status */
	vendor_id = mt_get_gpio_in(LCM_ID1_GPIO);
	vendor_id |= (mt_get_gpio_in(LCM_ID0_GPIO) << 1);
#else
	gpio_direction_input(LCM_ID1_GPIO);
	vendor_id = gpio_get_value(LCM_ID1_GPIO);
	gpio_direction_input(LCM_ID0_GPIO);
	vendor_id |= (gpio_get_value(LCM_ID0_GPIO) << 1);
#endif	/* #ifdef BUILD_LK */
	LCM_LOGI("vendor_id = 0x%x", vendor_id);
	return vendor_id;
}

/* lcm_i2c_driver start */
#ifdef BUILD_LK
#else

#define I2C_ID_NAME "nt51021"

static int nt51021_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int nt51021_remove(struct i2c_client *client);

static const struct of_device_id lcm_of_match[] = {
		{.compatible = "novatek,nt51021_i2c"},
		{.compatible = "common,lcm_i2c"},
		{},
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

struct i2c_client *nt51021_i2c_client;

static const struct i2c_device_id nt51021_id[] = {
	{I2C_ID_NAME, 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, nt51021_id);

static struct i2c_driver nt51021_iic_driver = {
	.id_table = nt51021_id,
	.probe = nt51021_probe,
	.remove = nt51021_remove,
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "nt51021",
#ifdef CONFIG_OF
			.of_match_table = of_match_ptr(lcm_of_match),
#endif
		   },
};

static int nt51021_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	LCM_DBG("name=%s, addr=0x%x", client->name, client->addr);
	nt51021_i2c_client = client;
	return 0;
}

static int nt51021_remove(struct i2c_client *client)
{
	nt51021_i2c_client = NULL;
	i2c_unregister_device(client);
	return 0;
}

static int nt51021_read_byte(unsigned char addr, unsigned char *value)
{
	int ret = 0;
	struct i2c_client *client;
	struct i2c_adapter *adap;
	struct i2c_msg msg[2];

	if (nt51021_i2c_client == NULL) {
		LCM_DBG("nt51021_i2c_client = NULL !!");
		return -1;
	}

	client = nt51021_i2c_client;
	adap = client->adapter;

	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN;
	msg[0].len = 1;
	msg[0].buf = &addr;

	msg[1].addr = client->addr;
	msg[1].flags = client->flags & I2C_M_TEN;
	msg[1].flags |= I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = value;

	ret = i2c_transfer(adap, msg, 2);

	if (ret <= 0)
		LCM_DBG("nt51021 read data fail !!");

	return ret;
}

static int nt51021_write_byte(unsigned char addr, unsigned char value)
{
	int ret = 0;
	struct i2c_client *client;
	char write_data[2] = { 0 };

	if (nt51021_i2c_client == NULL) {
		LCM_DBG("nt51021_i2c_client is NULL!!");
		return -1;
	}

	client = nt51021_i2c_client;

	write_data[0] = addr;
	write_data[1] = value;
	ret = i2c_master_send(client, write_data, 2);
	if (ret < 0)
		LCM_DBG("nt51021 write data fail !!");
	return ret;
}

static int __init nt51021_iic_init(void)
{
	LCM_DBG("nt51021_iic_init");
	i2c_add_driver(&nt51021_iic_driver);

	return 0;
}

static void __exit nt51021_iic_exit(void)
{
	LCM_DBG("nt51021_iic_exit");
	i2c_del_driver(&nt51021_iic_driver);
}

#endif	/* #ifdef BUILD_LK */
/* lcm_i2c_driver end */



/* lcm_platform_driver start */
#ifdef BUILD_LK
#else

static int lcm_driver_probe(struct device *dev, void const *data)
{
	int ret = 0;

	GPIO_LCD_RST_EN = of_get_named_gpio(dev->of_node, "gpio_lcm_rst_en", 0);
	ret = gpio_request(GPIO_LCD_RST_EN, "GPIO_LCD_RST_EN");
	if (ret) {
		LCM_ERR("gpio request GPIO_LCD_RST_EN = 0x%x fail with %d",
			GPIO_LCD_RST_EN, ret);

		goto err_rst_gpio;
	}

	LCM_ID0_GPIO = of_get_named_gpio(dev->of_node, "lcm_id0_gpio", 0);
	ret = gpio_request(LCM_ID0_GPIO, "LCM_ID0_GPIO");
	if (ret) {
		LCM_ERR("gpio request LCM_ID0_GPIO = 0x%x fail with %d",
			LCM_ID0_GPIO, ret);

		goto err_id0;
	}

	LCM_ID1_GPIO = of_get_named_gpio(dev->of_node, "lcm_id1_gpio", 0);
	ret = gpio_request(LCM_ID1_GPIO, "LCM_ID1_GPIO");
	if (ret) {
		LCM_ERR("gpio request LCM_ID1_GPIO = 0x%x fail with %d",
			LCM_ID1_GPIO, ret);

		goto err_id1;
	}

	g_vendor_id = get_lcm_id();

	if (g_vendor_id == VID_BOE_NT51021B) {
		LCM_LOGI("lcm probe success, Panel ID = %x", g_vendor_id);
	} else {
		LCM_ERR("lcm probe fail, Panel ID = %x", g_vendor_id);
		goto err_not_match;
	}

	reg_lcm_vcc3v3 = regulator_get(dev, "lcm_vcc3v3");
	if (IS_ERR(reg_lcm_vcc3v3)) {
		LCM_ERR("Failed to request lcm_vcc3v3 power supply: %ld\n",
			PTR_ERR(reg_lcm_vcc3v3));

		goto err_vcc3v3_1;
	}

	ret = regulator_enable(reg_lcm_vcc3v3);
	if (ret) {
		LCM_ERR("Failed to enable lcm_vcc3v3 power supply: %d\n", ret);
		goto err_vcc3v3_2;
	}

	reg_lcm_rst = regulator_get(dev, "lcm_rst");
	if (IS_ERR(reg_lcm_rst)) {
		LCM_ERR("Failed to request lcm_rst gpio power supply: %ld\n",
			PTR_ERR(reg_lcm_rst));

		goto err_rst_pwr_1;
	}

	ret = regulator_enable(reg_lcm_rst);
	if (ret) {
		LCM_ERR("Failed to enable lcm_rst gpio power supply: %d\n",
			ret);

		goto err_rst_pwr_2;
	}

	nt51021_iic_init();
	cmd_sel = 0;

	return 0;

err_rst_pwr_2:
	regulator_put(reg_lcm_rst);
	reg_lcm_rst = NULL;
err_rst_pwr_1:
	regulator_disable(reg_lcm_vcc3v3);
err_vcc3v3_2:
	regulator_put(reg_lcm_vcc3v3);
	reg_lcm_vcc3v3 = NULL;
err_vcc3v3_1:
err_not_match:
	gpio_free(LCM_ID1_GPIO);
err_id1:
	gpio_free(LCM_ID0_GPIO);
err_id0:
	gpio_free(GPIO_LCD_RST_EN);
err_rst_gpio:

	return -1;
}

static const struct of_device_id lcm_platform_of_match[] = {
	{
		.compatible = "novatek,nt51021_mvk",
		.data = 0,
	}, {
		/* sentinel */
	}
};


MODULE_DEVICE_TABLE(of, platform_of_match);

static int lcm_platform_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;

	id = of_match_node(lcm_platform_of_match, pdev->dev.of_node);
	if (!id)
		return -ENODEV;

	return lcm_driver_probe(&pdev->dev, id->data);
}

static int lcm_platform_remove(struct platform_device *pdev)
{
	gpio_free(LCM_ID0_GPIO);
	gpio_free(LCM_ID1_GPIO);
	gpio_free(GPIO_LCD_RST_EN);
	if (reg_lcm_rst) {
		regulator_disable(reg_lcm_rst);
		regulator_put(reg_lcm_rst);
		reg_lcm_rst = NULL;
	}
	if (reg_lcm_vcc3v3) {
		regulator_disable(reg_lcm_vcc3v3);
		regulator_put(reg_lcm_vcc3v3);
		reg_lcm_vcc3v3 = NULL;
	}

	LCM_DBG("lcm_platform_remove() done");

	return 0;
}
static struct platform_driver lcm_driver = {
	.probe = lcm_platform_probe,
	.remove = lcm_platform_remove,
	.driver = {
		.name = "nt51021_wuxga_dsi_vdo_mvk",
		.owner = THIS_MODULE,
		.of_match_table = lcm_platform_of_match,
	},
};

static int __init lcm_platform_init(void)
{
	LCM_DBG("Register panel driver for nt51021_wuxga_dsi_vdo_mvk");
	if (platform_driver_register(&lcm_driver)) {
		LCM_ERR("Failed to register this driver!");
		return -ENODEV;
	}

	return 0;
}

static void __exit lcm_platform_exit(void)
{
	platform_driver_unregister(&lcm_driver);
	nt51021_iic_exit();
	LCM_DBG("Unregister this driver done");
}

late_initcall(lcm_platform_init);
module_exit(lcm_platform_exit);
MODULE_AUTHOR("Compal");
MODULE_DESCRIPTION("LCM display subsystem driver");
MODULE_LICENSE("GPL");

#endif /* #ifdef BUILD_LK */
/* lcm_platform_driver end */


/* lcm_driver start */

/* --------------------------------------------------------------------------- */
/* Local Constants */
/* --------------------------------------------------------------------------- */

#if defined(MTK_ALPS_BOX_SUPPORT)
#define HDMI_SUB_PATH 1
#else
#define HDMI_SUB_PATH 0
#endif

#if HDMI_SUB_PATH
#define FRAME_WIDTH	 (1920)
#define FRAME_HEIGHT (1080)
#else
#define FRAME_WIDTH	 (1200)
#define FRAME_HEIGHT (1920)
#endif

#define GPIO_OUT_ONE 1
#define GPIO_OUT_ZERO 0
/* --------------------------------------------------------------------------- */
/* Local Variables */
/* --------------------------------------------------------------------------- */



static LCM_UTIL_FUNCS lcm_util = {
	.set_reset_pin = NULL,
	.udelay = NULL,
	.mdelay = NULL,
};

#define SET_RESET_PIN(v) lcm_set_gpio_output(GPIO_LCD_RST, v)	/* (lcm_util.set_reset_pin((v))) */

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))



/* --------------------------------------------------------------------------- */
/* Local Functions */
/* --------------------------------------------------------------------------- */

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)	 lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)									 lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)				 lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)						  lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#define LCM_DSI_CMD_MODE			0

static void init_lcm_registers(void)
{
	unsigned int data_array[16];

	switch (g_vendor_id) {
	case VID_BOE_NT51021B:
		LCM_LOGI("Init VID_BOE_NT51021B registers");

		if (cmd_sel == 0) {
			LCM_DBG("Force sending by MIPI");
			/* Command interface selection */
			data_array[0] = 0xA58F1500;
			dsi_set_cmdq(data_array, 1, 1);
			MDELAY(1);
		}

		data_array[0] = 0x00011500;	/* Software reset */
		dsi_set_cmdq(data_array, 1, 1);
		MDELAY(20);

		if (cmd_sel == 0) {
			LCM_DBG("Force sending by MIPI");
			data_array[0] = 0xA58F1500; /* Command interface selection */
			dsi_set_cmdq(data_array, 1, 1);
			MDELAY(1);
		}

		data_array[0] = 0x00831500; /* Page setting */
		dsi_set_cmdq(data_array, 1, 1);
		data_array[0] = 0x00841500; /* Page setting */
		dsi_set_cmdq(data_array, 1, 1);

		/* BIST test */
		/*data_array[0] = 0x06911500;*/
		/*dsi_set_cmdq(data_array, 1, 1);*/

		data_array[0] = 0x808c1500; /* GOP setting */
		dsi_set_cmdq(data_array, 1, 1);

		data_array[0] = 0x6ccd1500; /* 3Dummy */
		dsi_set_cmdq(data_array, 1, 1);

		data_array[0] = 0x8bc01500;
		dsi_set_cmdq(data_array, 1, 1);

		data_array[0] = 0xf0c81500; /* GCH */
		dsi_set_cmdq(data_array, 1, 1);

		/* Resistor setting 100ohm */
		data_array[0] = 0x00971500;
		dsi_set_cmdq(data_array, 1, 1);

		data_array[0] = 0x108b1500;
		dsi_set_cmdq(data_array, 1, 1);

		data_array[0] = 0x20A91500; /* Enable TP_SYNC */
		dsi_set_cmdq(data_array, 1, 1);

		data_array[0] = 0xaa831500; /* Page setting */
		dsi_set_cmdq(data_array, 1, 1);
		data_array[0] = 0x11841500; /* Page setting */
		dsi_set_cmdq(data_array, 1, 1);

		/* IC MIPI Rx drivering setting 85% */
		data_array[0] = 0x4ba91500;
		dsi_set_cmdq(data_array, 1, 1);
		data_array[0] = 0x04851500; /* Test mode1 */
		dsi_set_cmdq(data_array, 1, 1);
		data_array[0] = 0x08861500; /* Test mode2 */
		dsi_set_cmdq(data_array, 1, 1);
		data_array[0] = 0x109c1500; /* Test mode3 */
		dsi_set_cmdq(data_array, 1, 1);

		data_array[0] = 0x00110500; /* Sleep Out */
		dsi_set_cmdq(data_array, 1, 1);

		if (cmd_sel == 0) {
			LCM_DBG("Exit force sending");
			/* Command interface selection */
			data_array[0] = 0x008F1500;
			dsi_set_cmdq(data_array, 1, 1);
		}
		MDELAY(5);
		break;

	default:
		LCM_LOGI("Not supported yet, g_vendor_id = 0x%x",
			g_vendor_id);

		break;
	}
}

/* --------------------------------------------------------------------------- */
/* LCM Driver Implementations */
/* --------------------------------------------------------------------------- */
static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_set_gpio_output(unsigned int GPIO, unsigned int output)
{
	LCM_DBG("GPIO = 0x%x, Output= 0x%x", GPIO, output);

#ifdef BUILD_LK
	mt_set_gpio_mode(GPIO, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO, (output > 0) ? GPIO_OUT_ONE : GPIO_OUT_ZERO);
#else
#if HDMI_SUB_PATH
#else
	gpio_direction_output(GPIO, (output > 0) ? GPIO_OUT_ONE : GPIO_OUT_ZERO);
	gpio_set_value(GPIO, (output > 0) ? GPIO_OUT_ONE : GPIO_OUT_ZERO);
#endif
#endif	/* #ifdef BUILD_LK */
}

static void lcm_get_params(LCM_PARAMS *params)
{

	memset(params, 0, sizeof(LCM_PARAMS));

	params->type = LCM_TYPE_DSI;

	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode = CMD_MODE;
#else
	params->dsi.mode = SYNC_EVENT_VDO_MODE;
#endif

	params->dsi.LANE_NUM = LCM_FOUR_LANE;

	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

	params->dsi.packet_size = 256;
	params->dsi.intermediat_buffer_num = 0;
	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.word_count = FRAME_WIDTH * 3;

	params->dsi.vertical_sync_active = 20;
	params->dsi.vertical_backporch = 25;
	params->dsi.vertical_frontporch = 25;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 15;
	params->dsi.horizontal_backporch = 55;
	params->dsi.horizontal_frontporch = 125;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;

	params->dsi.PLL_CLOCK = 497;
	params->dsi.clk_lp_per_line_enable = 1;
	params->dsi.ssc_disable = 1;
	params->dsi.cont_clock = 1; /* Keep HS serial clock running */
	params->dsi.DA_HS_EXIT = 1;
	params->dsi.CLK_ZERO = 16;
	params->dsi.HS_ZERO = 25;
	params->dsi.HS_TRAIL = 10;
	params->dsi.CLK_TRAIL = 1;
	params->dsi.CLK_HS_POST = 8;
	params->dsi.CLK_HS_EXIT = 6;
	/* params->dsi.CLK_HS_PRPR = 1; */

	params->dsi.TA_GO = 8;
	params->dsi.TA_GET = 10;

	/*params->dsi.cust_clk_impendence = 0xf;*/

	/* The values are in mm.*/
	/* Add these values for reporting correct xdpi and ydpi values */
	params->physical_width = 135;
	params->physical_height = 217;
}

static void lcm_init(void)
{
#ifdef BUILD_LK
	cmd_sel = 0;
	gpio_rst = GPIO_LCD_RST_EVT;

	lcm_set_gpio_output(gpio_rst, GPIO_OUT_ZERO);
	MDELAY(20);

	lcm_set_gpio_output(gpio_rst, GPIO_OUT_ONE);
	MDELAY(5);

	init_lcm_registers();
#else
	cmd_sel = 0;

	LCM_LOGI("Skip power_on & init lcm since it's done by lk");
#endif	/* #ifdef BUILD_LK */
	LCM_LOGI("cmd_sel = 0x%x", cmd_sel);
}

static void lcm_suspend(void)
{
	unsigned int data_array[16];

	LCM_LOGI("lcm_suspend");

#ifdef BUILD_LK
#else
	if (cmd_sel == 0) {
		LCM_DBG("Force sending by MIPI");
		/* Command interface selection */
		data_array[0] = 0xA58F1500;
		dsi_set_cmdq(data_array, 1, 1);
		MDELAY(1);
	}

	data_array[0] = 0x00100500; /* Sleep In */
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(80);

#endif	/* #ifdef BUILD_LK */
}


static void lcm_suspend_power(void)
{
	int ret = 1;

	LCM_LOGI("lcm_suspend_power");

#ifdef BUILD_LK
#else

	MDELAY(5);	/* 1ms < T6*/
	lcm_set_gpio_output(GPIO_LCD_RST_EN, GPIO_OUT_ZERO);

	MDELAY(5);	/* 0.1ms < T8 < 20ms */
	if (reg_lcm_rst) {
		ret = regulator_disable(reg_lcm_rst);
		if (ret != 0)
			LCM_ERR("disable reg_lcm_rst fail");
	}

	if (reg_lcm_vcc3v3) {
		ret = regulator_disable(reg_lcm_vcc3v3);
		if (ret != 0)
			LCM_ERR("disable reg_lcm_vcc3v3 fail");
	}

#endif	/* #ifdef BUILD_LK */
}

static void lcm_resume_power(void)
{
	int ret = 1;

	LCM_LOGI("lcm_resume_power");

#ifdef BUILD_LK
#else
	if (reg_lcm_vcc3v3) {
		ret = regulator_enable(reg_lcm_vcc3v3);
		if (ret != 0)
			LCM_ERR("enable reg_lcm_vcc3v3 fail");
	}

	if (reg_lcm_rst) {
		ret = regulator_enable(reg_lcm_rst);
		if (ret != 0)
			LCM_ERR("enable reg_lcm_rst fail");
	}

	lcm_set_gpio_output(GPIO_LCD_RST_EN, GPIO_OUT_ZERO);

	MDELAY(5);	/* 1ms < T5 < 20ms */

	lcm_set_gpio_output(GPIO_LCD_RST_EN, GPIO_OUT_ONE);

	MDELAY(5);	/* 1ms < T2 < 20ms */

#endif	/* #ifdef BUILD_LK */
}

static void lcm_resume(void)
{

	LCM_LOGI("lcm_resume");

#ifdef BUILD_LK
#else

	init_lcm_registers();

#endif	/* #ifdef BUILD_LK */
}

#if (LCM_DSI_CMD_MODE)
static void lcm_update(unsigned int x, unsigned int y, unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0 >> 8) & 0xFF);
	unsigned char x0_LSB = (x0 & 0xFF);
	unsigned char x1_MSB = ((x1 >> 8) & 0xFF);
	unsigned char x1_LSB = (x1 & 0xFF);
	unsigned char y0_MSB = ((y0 >> 8) & 0xFF);
	unsigned char y0_LSB = (y0 & 0xFF);
	unsigned char y1_MSB = ((y1 >> 8) & 0xFF);
	unsigned char y1_LSB = (y1 & 0xFF);

	unsigned int data_array[16];

	data_array[0] = 0x00053902;
	data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
	data_array[2] = (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x00053902;
	data_array[1] = (y1_MSB << 24) | (y0_LSB << 16) | (y0_MSB << 8) | 0x2b;
	data_array[2] = (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x00290508;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);
}
#endif

static void lcm_setbacklight_mode(unsigned int mode)
{
	int ret = 0;
	unsigned char value;

	LCM_DBG("Send CABC mode 0x%x via i2c", mode);

	if (mode > CABC_STILL) {
		LCM_ERR("CABC mode %x is not supported now", mode);
		goto err_unsupport;
	}

	ret = nt51021_write_byte(nt51021_cabc_regs[0][0],
				nt51021_cabc_regs[0][1]);

	if (ret < 0)
		goto err_i2c;

	ret = nt51021_write_byte(nt51021_cabc_regs[1][0],
				nt51021_cabc_regs[1][1]);
	if (ret < 0)
		goto err_i2c;

	ret = nt51021_read_byte(nt51021_cabc_regs[2][0], &value);
	if (ret < 0)
		goto err_i2c;
	else
		LCM_LOGI("Current CABC value is %x", value);

	SetField(value, NT51021_CABC_ENB_MASK, NT51021_CABC_ENB_SHIFT,
				CABC_mapping_table[mode]);

	ret = nt51021_write_byte(nt51021_cabc_regs[2][0], value);
	if (ret < 0)
		goto err_i2c;

	ret = nt51021_read_byte(nt51021_cabc_regs[2][0], &value);
	if (ret < 0)
		goto err_i2c;
	else
		LCM_LOGI("Update CABC value to %x", value);

	return;

err_i2c:
	LCM_ERR("Read/Write CABC value via I2C fail");
err_unsupport:
	return;
}

LCM_DRIVER nt51021_wuxga_dsi_vdo_mvk_lcm_drv = {
	.name = "nt51021_wuxga_dsi_vdo_mvk",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend_power = lcm_suspend_power,
	.suspend = lcm_suspend,
	.resume_power = lcm_resume_power,
	.resume = lcm_resume,
	.set_backlight_mode = lcm_setbacklight_mode,
#if (LCM_DSI_CMD_MODE)
	.update = lcm_update,
#endif
};
/* lcm_driver end */
