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
#define LOG_TAG "LCM-OTA7290B-MVK"

#define LCM_LOGI(fmt, args...)  pr_notice("[KERNEL/"LOG_TAG"]"fmt, ##args)
#define LCM_DBG(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#define LCM_ERR(fmt, args...)  pr_err("[KERNEL/"LOG_TAG"]"fmt, ##args)


#define HVT_OTA7290B_CABC_WORKAROUND
#define OTA7290B_CABC_SMOOTH_EXPIRES (2*HZ)
extern unsigned int idme_get_board_rev(void);
extern unsigned int idme_get_board_type(void);

#ifdef CONFIG_FADE_IN_OUT_FEATURE_FOR_CABC
extern void mt6370_bled_set_ramptime(uint32_t ramptime);
extern void mt6370_bled_set_pwm_hys(uint32_t hys_en,
		uint32_t hys_val);
static void cabc_init_timer(void);
static void cabc_timer_callback(unsigned long ramptime);
static int cabc_timer_work_init(void);
static void cabc_work_handler(struct work_struct *data);

#define BLED_RAMPTIME_1MS		0x03
#define BLED_RAMPTIME_2MS		0x04
#define BLED_RAMPTIME_5MS		0x05
#define BLED_RAMPTIME_10MS		0x06
#define BLED_RAMPTIME_20MS		0x07
#define BLED_RAMPTIME_50MS		0x08
#define BLED_RAMPTIME_100MS		0x09
#define BLED_RAMPTIME_250MS		0x0A
#define BLED_RAMPTIME_800MS		0x0B
#define BLED_RAMPTIME_1000MS	0x0C
#define BLED_RAMPTIME_2000MS	0x0D
#define BLED_RAMPTIME_4000MS	0x0E
#define BLED_RAMPTIME_8000MS	0x0F

#define BLED_PWMHYS_EN_ON		0x01
#define BLED_PWMHYS_EN_OFF		0x00
#define BLED_PWMHYS_1BIT		0x00
#define BLED_PWMHYS_2BIT		0x01
#define BLED_PWMHYS_4BIT		0x02
#define BLED_PWMHYS_6BIT		0x03

struct timer_list cabc_report_timer;
static struct workqueue_struct		*cabc_timer_workqueue;
static struct work_struct			cabc_work;
#endif

/*
* 1. lcm_i2c_driver
* 2. lcm_platform_driver
* 3. lcm_driver
*/

#define VID_INX_OTA7290B	0x0		/* Innolux with OTA7290B */
#define VID_BOE_NT51021B	0x1		/* BOE with NT51021B */
#define VID_KD1_OTA7290B	0x2		/* KD(CPT) with OTA7290B */
#define VID_KD2_OTA7290B	0x3		/* KD(INX) with OTA7290B */
#define VID_NULL			0xff	/* NULL */

#define OTA_I2C_OTP_RELOAD_ADDR				0xB0
#define OTA_I2C_PAGE_SELECT_ADDR			0xB1
#define OTA_I2C_SLEEP_IN_ADDR				0x89
#define OTA_I2C_SLEEP_IN_MASK				0x01
#define OTA_I2C_SLEEP_IN_SHIFT				0x01
#define DELAY_CMD		0xFF
/* BANK 0 */
#define OTA_I2C_CABC_ENB_ADDR				0x91
#define OTA_I2C_CABC_ENB_DCE_EN_MASK		0x01
#define OTA_I2C_CABC_ENB_DCE_EN_SHIFT		0x06
#define OTA_I2C_CABC_ENB_DIM_EN_MASK		0x01
#define OTA_I2C_CABC_ENB_DIM_EN_SHIFT		0x05
#define OTA_I2C_CABC_ENB_PWM_OUT_EN_MASK	0x01
#define OTA_I2C_CABC_ENB_PWM_OUT_EN_SHIFT	0x03

#define BOARD_TYPE_MVK		0x3F

#define BOARD_REV_PROTO		0x00
#define BOARD_REV_HVT1p0	0x10
#define BOARD_REV_HVT1p1	0x11
#define BOARD_REV_EVT		0x20
#define BOARD_REV_DVT		0x30
#define BOARD_REV_PVT		0x40

#ifdef BUILD_LK
#else

static unsigned int GPIO_LCD_RST_EN;
static unsigned int LCM_ID1_GPIO;
static unsigned int LCM_ID0_GPIO;

static struct regulator *reg_lcm_rst;
static struct regulator *reg_lcm_vcc3v3;

static unsigned int g_vendor_id = VID_NULL;
static unsigned int g_board_type;
static unsigned int g_board_rev;

#define GetField(Var, Mask, Shift) \
	(((Var) >> (Shift)) & (Mask))

#define SetField(Var, Mask, Shift, Val) \
	((Var) = (((Var) & ~((Mask) << (Shift))) | \
				(((Val) & (Mask)) << (Shift))))

static LCM_UTIL_FUNCS lcm_util = {
	.set_reset_pin = NULL,
	.udelay = NULL,
	.mdelay = NULL,
};
#define SET_RESET_PIN(v) lcm_set_gpio_output(GPIO_LCD_RST, v)
#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

static unsigned char CABC_mapping_table_DCE[] = {
	0x00,	/* CABC_OFF		*/
	0x01,	/* CABC_MOVIE	*/
	0x01,	/* CABC_STILL, only support movie mode on OTA7290B	*/
	0x01,	/* CABC_UI	 , only support movie mode on OTA7290B	*/
};

static unsigned char CABC_mapping_table_DIM[] = {
	0x01,	/* CABC_OFF		*/
	0x01,	/* CABC_MOVIE	*/
	0x01,	/* CABC_STILL, only support movie mode on OTA7290B	*/
	0x01,	/* CABC_UI	 , only support movie mode on OTA7290B	*/
};

static unsigned char CABC_mapping_table_PWM_OUT[] = {
	0x00,	/* CABC_OFF		*/
	0x01,	/* CABC_MOVIE	*/
	0x01,	/* CABC_STILL, only support movie mode on OTA7290B	*/
	0x01,	/* CABC_UI	 , only support movie mode on OTA7290B	*/
};

#endif		/* #ifdef BUILD_LK */

#ifdef HVT_OTA7290B_CABC_WORKAROUND
static unsigned char cabc_on_parms[][2] = {
	{0xB0, 0x5A,},
	{0xB1, 0x03,},
	{0x40, 0x28,},
	{0x41, 0xFC,},
	{0x42, 0x01,},
	{0x43, 0x08,},
	{0x44, 0x05,},
	{0x45, 0xF0,},
	{0x46, 0x01,},
	{0x47, 0x02,},
	{0x48, 0x00,},
	{0x49, 0x58,},
	{0x4A, 0x00,},
	{0x4B, 0x05,},
	{0x4C, 0x03,},
	{0x4D, 0xD0,},
	{0x4E, 0x13,},
	{0x4F, 0xFF,},

	{0xB1, 0x00,},
	{0x91, 0x3F,},
	{0xB0, 0x5A,},
	{0xB3, 0xA5,},
	{0xC0, 0x8A,},
	{0xC1, 0x87,},
	{0xC2, 0x82,},
	{0xC3, 0x82,},
	{0xC4, 0x80,},
	{0xC5, 0x80,},
	{0xC6, 0x80,},
	{0xC7, 0x80,},
	{0xC8, 0x7C,},
	{0xC9, 0x7C,},
	{0xCA, 0x7C,},
	{0xCB, 0x7C,},
	{0xCC, 0x78,},
	{0xCD, 0x78,},
	{0xCE, 0x78,},
	{0xCF, 0x78,},

	{0xB1, 0x00,},
	{0x91, 0x7F,},
};

static unsigned char cabc_off_parms[][2] = {
	{0xB1, 0x00,},
	{0x91, 0x77,},
	{0x91, 0x37,},
	{DELAY_CMD, 0x15,},
	{0xB0, 0x00,},
};
#endif

typedef unsigned char DISP_CMD[2];
static DISP_CMD g_init_setting[] = {
	{0xB1, 0x00,}, /* select page 0*/
	{0x89, 0x03,}, /* sleep out */
};

static DISP_CMD g_init_setting_hvt1p1[] = {
	{0xB1, 0x00,}, /* select page 0*/
	{0x89, 0x03,}, /* sleep out */
	{0x91, 0x37,}, /*force cabc off */
};

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
#define I2C_ID_NAME "ota7290b"
static int ota7290b_probe(struct i2c_client *client,
						const struct i2c_device_id *id);
static int ota7290b_remove(struct i2c_client *client);

static const struct of_device_id lcm_of_match[] = {
		{.compatible = "focaltech,ota7290b_i2c"},
		{.compatible = "common,lcm_i2c"},
		{},
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

struct i2c_client *ota7290b_i2c_client;


static const struct i2c_device_id ota7290b_id[] = {
	{I2C_ID_NAME, 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, ota7290b_id);

static struct i2c_driver ota7290b_iic_driver = {
	.id_table = ota7290b_id,
	.probe = ota7290b_probe,
	.remove = ota7290b_remove,
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "ota7290b",
#ifdef CONFIG_OF
			.of_match_table = of_match_ptr(lcm_of_match),
#endif
		   },
};

static int ota7290b_probe(struct i2c_client *client,
						const struct i2c_device_id *id)
{
	LCM_DBG("name=%s, addr=0x%x", client->name, client->addr);
	ota7290b_i2c_client = client;
	return 0;
}

static int ota7290b_remove(struct i2c_client *client)
{
	ota7290b_i2c_client = NULL;
	i2c_unregister_device(client);
	return 0;
}

static int ota7290b_read_byte(unsigned char addr, unsigned char *value)
{
	int ret = 0;
	struct i2c_client *client;
	struct i2c_adapter *adap;
	struct i2c_msg msg[2];

	if (ota7290b_i2c_client == NULL) {
		LCM_ERR("ota7290b_i2c_client = NULL !!");
		return -1;
	}

	client = ota7290b_i2c_client;
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
	if (ret < 0)
		LCM_ERR("ota7290b read data fail !!");

	return ret;
}

static int ota7290b_write_byte(unsigned char addr, unsigned char value)
{
	int ret = 0;
	struct i2c_client *client;
	char write_data[2] = { 0 };

	if (ota7290b_i2c_client == NULL) {
		LCM_ERR("ota7290b_i2c_client is NULL!!");
		return -1;
	}

	if (addr == DELAY_CMD) {
		MDELAY(value);
	} else {
		client = ota7290b_i2c_client;

		write_data[0] = addr;
		write_data[1] = value;
		ret = i2c_master_send(client, write_data, 2);
		if (ret < 0)
			LCM_ERR("ota7290b write data fail !!");
	}
	return ret;
}

static int __init ota7290b_iic_init(void)
{
	LCM_DBG("ota7290b_iic_init");
	i2c_add_driver(&ota7290b_iic_driver);
	return 0;
}

static void __exit ota7290b_iic_exit(void)
{
	LCM_DBG("ota7290b_iic_exit");
	i2c_del_driver(&ota7290b_iic_driver);
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

	if ((g_vendor_id == VID_INX_OTA7290B) ||
		(g_vendor_id == VID_KD1_OTA7290B) ||
		(g_vendor_id == VID_KD2_OTA7290B)) {
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

	ota7290b_iic_init();

	g_board_type = idme_get_board_type();
	g_board_rev = idme_get_board_rev();

#ifdef CONFIG_FADE_IN_OUT_FEATURE_FOR_CABC
	cabc_init_timer();
	ret = cabc_timer_work_init();
	if (ret) {
		LCM_ERR("Failed on cabc_timer_work_init with %d\n", ret);
		goto err_cabc_init;
	}
#endif

	return 0;

#ifdef CONFIG_FADE_IN_OUT_FEATURE_FOR_CABC
err_cabc_init:
	regulator_disable(reg_lcm_rst);
#endif
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
		.compatible = "focaltech,ota7290b_mvk",
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
		.name = "ota7290b_wuxga_dsi_vdo_mvk",
		.owner = THIS_MODULE,
		.of_match_table = lcm_platform_of_match,
	},
};

static int __init lcm_platform_init(void)
{
	LCM_DBG("Register panel driver for ota7290b_wuxga_dsi_vdo_mvk");
	if (platform_driver_register(&lcm_driver)) {
		LCM_ERR("Failed to register this driver!");
		return -ENODEV;
	}

	return 0;
}

static void __exit lcm_platform_exit(void)
{
	platform_driver_unregister(&lcm_driver);
	ota7290b_iic_exit();
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

/* --------------- */
/* Local Constants */
/* --------------- */

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
/* --------------- */
/* Local Variables */
/* --------------- */

/* --------------- */
/* Local Functions */
/* ----------------*/

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)	 \
	lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)				\
	lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)				 \
	lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)				\
	lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)			 \
	lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#define LCM_DSI_CMD_MODE			0

static void init_lcm_registers(void)
{
	int i = 0;
	int ret = 0;
	int arr_size = 0;
	DISP_CMD *init_setting;

	if ((g_board_type == BOARD_TYPE_MVK) &&
			(g_board_rev == BOARD_REV_HVT1p1)) {
		init_setting = g_init_setting_hvt1p1;
		arr_size = ARRAY_SIZE(g_init_setting_hvt1p1);
		LCM_LOGI("workaround for CABC off");
	} else {
		init_setting = g_init_setting;
		arr_size = ARRAY_SIZE(g_init_setting);
	}


	switch (g_vendor_id) {
	case VID_INX_OTA7290B:
	case VID_KD1_OTA7290B:
	case VID_KD2_OTA7290B:
		LCM_LOGI("Init panel-id(0x%x, 0x%x) registers(v2)",
			g_vendor_id, g_board_rev);
		/* Initial code already preburn to otp */
		for (i = 0; i < arr_size; i++) {
			ret = ota7290b_write_byte(
					init_setting[i][0],
					init_setting[i][1]);
			if (ret < 0)
				LCM_ERR("i2c fail on %xH=0x%x",
					init_setting[i][0], init_setting[i][1]);
		}
		break;

	default:
		LCM_LOGI("The panel-id(%x) is not supported yet",
				g_vendor_id);
		break;
	}

}

/* ---------------------------*/
/* LCM Driver Implementations */
/* ---------------------------*/
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
	gpio_direction_output(GPIO, (output > 0)
					? GPIO_OUT_ONE : GPIO_OUT_ZERO);
	gpio_set_value(GPIO, (output > 0)
					? GPIO_OUT_ONE : GPIO_OUT_ZERO);
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

	params->dsi.vertical_sync_active = 18;
	params->dsi.vertical_backporch = 25;
	params->dsi.vertical_frontporch = 35;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 35;
	params->dsi.horizontal_backporch = 32;
	params->dsi.horizontal_frontporch = 42;
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
	gpio_rst = GPIO_LCD_RST_EVT;

	lcm_set_gpio_output(gpio_rst, GPIO_OUT_ZERO);
	MDELAY(20);

	lcm_set_gpio_output(gpio_rst, GPIO_OUT_ONE);
	MDELAY(5);

	init_lcm_registers();
#else

	LCM_LOGI("Skip power_on & init lcm since it's done by lk");
#endif	/* #ifdef BUILD_LK */
}

static void lcm_suspend(void)
{
	int ret = 0;
	unsigned char value;

	LCM_LOGI("lcm_suspend");

#ifdef BUILD_LK
#else

	value = 0x01;

#ifdef CONFIG_FADE_IN_OUT_FEATURE_FOR_CABC
	if (del_timer(&cabc_report_timer)) {
		LCM_LOGI("stop timer(cabc_report_timer)");
		LCM_DBG("cabc_timer setup ramptime=1ms\n");
		mt6370_bled_set_ramptime((uint32_t) BLED_RAMPTIME_1MS);
	}
	if (cancel_work_sync(&cabc_work))
		LCM_ERR("cabc_work was pending");
#endif

	ret = ota7290b_write_byte(OTA_I2C_SLEEP_IN_ADDR, value); /* Sleep In */
	if (ret < 0)
		LCM_DBG("set Sleep_In register value fail");

#endif	/* #ifdef BUILD_LK */
}

static void lcm_suspend_power(void)
{
	int ret = 1;

	LCM_LOGI("lcm_suspend_power");

#ifdef BUILD_LK
#else

	MDELAY(80);		/* TODO: wait for 4 frame */

	lcm_set_gpio_output(GPIO_LCD_RST_EN, GPIO_OUT_ZERO);
	/* MDELAY(10); */
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
	MDELAY(10);

	lcm_set_gpio_output(GPIO_LCD_RST_EN, GPIO_OUT_ONE);
	MDELAY(20);

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
static void lcm_update(unsigned int x, unsigned int y,
				unsigned int width, unsigned int height)
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

#ifdef CONFIG_FADE_IN_OUT_FEATURE_FOR_CABC
static void cabc_init_timer(void)
{
	init_timer(&cabc_report_timer);
	cabc_report_timer.function = cabc_timer_callback;
	cabc_report_timer.data = ((unsigned long) BLED_RAMPTIME_1MS);
	cabc_report_timer.expires = jiffies + HZ;
}

static void cabc_timer_callback(unsigned long ramptime)
{
	LCM_DBG("cabc_timer trigger callback\n");
	if (!queue_work(cabc_timer_workqueue, &cabc_work))
		LCM_ERR("queue_work failed\n");
}

static int cabc_timer_work_init(void)
{
	cabc_timer_workqueue = alloc_workqueue("cabc_timer_wq",
			WQ_UNBOUND | WQ_HIGHPRI, 1);

	if (!cabc_timer_workqueue) {
		LCM_ERR("workqueue create failed\n");
		return 1;
	}

	INIT_WORK(&cabc_work, cabc_work_handler);
	LCM_DBG("create cabc_timer_workqueue done\n");

	return 0;
}

static void cabc_work_handler(struct work_struct *data)
{
	LCM_DBG("cabc_timer setup ramptime=1ms\n");
	mt6370_bled_set_ramptime((uint32_t) BLED_RAMPTIME_1MS);
}

#endif

static void lcm_setbacklight_mode(unsigned int mode)
{
	int ret = 0;
	unsigned char value;
	static unsigned last_mode = CABC_OFF;
	int i;

	LCM_LOGI("%s setup CABC mode=%d via i2c.\n", __func__, mode);

	if (mode > CABC_MOVIE) {
		LCM_ERR("CABC mode %x is not supported now", mode);
		goto err_unsupport;
	}

	if (mode == last_mode) {
		LCM_LOGI("exit due to no change on cabc mode(%d)", mode);
		goto err_nochange;
	} else {
		last_mode = mode;
	}

	value = 0x00;
	ret = ota7290b_write_byte(OTA_I2C_PAGE_SELECT_ADDR, value);
	if (ret < 0)
		goto err_i2c;
	ret = ota7290b_read_byte(OTA_I2C_CABC_ENB_ADDR, &value);
	if (ret < 0)
		goto err_i2c;
	LCM_DBG("Old CABC value is 0x%x\n", value);

#ifdef CONFIG_FADE_IN_OUT_FEATURE_FOR_CABC
	LCM_LOGI("CABC setup bled/ramp=800ms, duration=%d ticks\n",
			OTA7290B_CABC_SMOOTH_EXPIRES);
	mt6370_bled_set_ramptime(BLED_RAMPTIME_800MS);
	mod_timer(&cabc_report_timer, jiffies + OTA7290B_CABC_SMOOTH_EXPIRES);
#endif
	i = 0;
#ifdef HVT_OTA7290B_CABC_WORKAROUND
	/* HVT panel didn't write CABC parameter into OTP correctly	  */
	/* below is SW solution */
	if ((g_board_type == BOARD_TYPE_MVK) &&
		(g_board_rev == BOARD_REV_HVT1p0 ||
		(g_board_rev == BOARD_REV_HVT1p1 && g_vendor_id == VID_INX_OTA7290B))) {

		LCM_LOGI("CABC workaround on HVT1.0/HVT1.1 ota7290 panels"
			" (0x%x,0x%x,%d).\n",
			g_board_type, g_board_rev, g_vendor_id);

		switch (mode) {
		case CABC_MOVIE:
			mt6370_bled_set_pwm_hys(BLED_PWMHYS_EN_ON,
					BLED_PWMHYS_2BIT);
			LCM_DBG("Disable OTP reloading, apply new"
						" CABC params..");
			for (i = 0; i < ARRAY_SIZE(cabc_on_parms); i++) {
				ret = ota7290b_write_byte(
						cabc_on_parms[i][0],
						cabc_on_parms[i][1]);
				if (ret < 0)
					goto err_i2c;
			}
			break;
		case CABC_OFF:
			for (i = 0; i < ARRAY_SIZE(cabc_off_parms); i++) {
				ret = ota7290b_write_byte(
						cabc_off_parms[i][0],
						cabc_off_parms[i][1]);
				if (ret < 0)
					goto err_i2c;
			}
			LCM_DBG("Enable OTP reloading");
			mt6370_bled_set_pwm_hys(BLED_PWMHYS_EN_OFF,
					BLED_PWMHYS_2BIT);
			break;
		default:
			LCM_ERR("CABC mode 0x%x is not supported now\n",
					mode);
			break;
		}
		value = 0x00;
		ret = ota7290b_write_byte(OTA_I2C_PAGE_SELECT_ADDR, value);
		if (ret < 0)
			goto err_i2c;
		ret = ota7290b_read_byte(OTA_I2C_CABC_ENB_ADDR, &value);
		if (ret < 0)
			goto err_i2c;
		LCM_LOGI("New CABC value: 0x%x\n", value);
		return;
	}
#endif

	switch (mode) {
	case CABC_MOVIE:
		mt6370_bled_set_pwm_hys(BLED_PWMHYS_EN_ON,
				BLED_PWMHYS_2BIT);

		/* 37 -> 7f */
		SetField(value, OTA_I2C_CABC_ENB_DCE_EN_MASK,
				OTA_I2C_CABC_ENB_DCE_EN_SHIFT,
				CABC_mapping_table_DCE[CABC_MOVIE]);

		SetField(value, OTA_I2C_CABC_ENB_DIM_EN_MASK,
				OTA_I2C_CABC_ENB_DIM_EN_SHIFT,
				CABC_mapping_table_DIM[CABC_MOVIE]);

		SetField(value, OTA_I2C_CABC_ENB_PWM_OUT_EN_MASK,
				OTA_I2C_CABC_ENB_PWM_OUT_EN_SHIFT,
				CABC_mapping_table_PWM_OUT[CABC_MOVIE]);

		ret = ota7290b_write_byte(OTA_I2C_CABC_ENB_ADDR,
				value);
		if (ret < 0)
			goto err_i2c;

		break;

	case CABC_OFF:
		/* 7f -> 77 -> 37 to prevent screen flicker */
		SetField(value, OTA_I2C_CABC_ENB_DCE_EN_MASK,
				OTA_I2C_CABC_ENB_DCE_EN_SHIFT,
				CABC_mapping_table_DCE[CABC_MOVIE]);

		SetField(value, OTA_I2C_CABC_ENB_DIM_EN_MASK,
				OTA_I2C_CABC_ENB_DIM_EN_SHIFT,
				CABC_mapping_table_DIM[CABC_OFF]);

		SetField(value, OTA_I2C_CABC_ENB_PWM_OUT_EN_MASK,
				OTA_I2C_CABC_ENB_PWM_OUT_EN_SHIFT,
				CABC_mapping_table_PWM_OUT[CABC_OFF]);

		ret = ota7290b_write_byte(OTA_I2C_CABC_ENB_ADDR,
				value);
		if (ret < 0)
			goto err_i2c;

		SetField(value, OTA_I2C_CABC_ENB_DCE_EN_MASK,
				OTA_I2C_CABC_ENB_DCE_EN_SHIFT,
				CABC_mapping_table_DCE[CABC_OFF]);

		SetField(value, OTA_I2C_CABC_ENB_DIM_EN_MASK,
				OTA_I2C_CABC_ENB_DIM_EN_SHIFT,
				CABC_mapping_table_DIM[CABC_OFF]);

		SetField(value, OTA_I2C_CABC_ENB_PWM_OUT_EN_MASK,
				OTA_I2C_CABC_ENB_PWM_OUT_EN_SHIFT,
				CABC_mapping_table_PWM_OUT[CABC_OFF]);

		ret = ota7290b_write_byte(OTA_I2C_CABC_ENB_ADDR,
				value);
		if (ret < 0)
			goto err_i2c;

		mt6370_bled_set_pwm_hys(BLED_PWMHYS_EN_OFF,
				BLED_PWMHYS_2BIT);
		break;
	default:
		LCM_ERR("CABC mode %x is not supported now\n", mode);
		break;

	}

	ret = ota7290b_read_byte(OTA_I2C_CABC_ENB_ADDR, &value);
	if (ret < 0)
		goto err_i2c;
	LCM_LOGI("New CABC value: 0x%x\n", value);

	return;

err_i2c:
	LCM_ERR("%s: Read/Write CABC value via I2C fail.\n",
			__func__);
err_unsupport:
err_nochange:
	return;
}

LCM_DRIVER ota7290b_wuxga_dsi_vdo_mvk_lcm_drv = {
	.name = "ota7290b_wuxga_dsi_vdo_mvk",
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
}; /*lcm_driver end*/
