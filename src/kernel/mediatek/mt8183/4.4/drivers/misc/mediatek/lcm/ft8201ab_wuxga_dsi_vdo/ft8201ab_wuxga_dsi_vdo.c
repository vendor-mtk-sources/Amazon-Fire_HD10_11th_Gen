/*
 * Copyright (C) 2021 MediaTek Inc.
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

#define LOG_TAG "LCM-FT8201AB"

#define LCM_LOGI(fmt, args...)              \
		pr_notice("["LOG_TAG"]" fmt "\n", ##args)
#define LCM_DBG(fmt, args...)               \
		pr_debug("["LOG_TAG"]" fmt "\n", ##args)
#define LCM_ERR(fmt, args...)               \
		pr_err("["LOG_TAG"][ERR]" fmt "\n", ##args)

#define LCM_FUNC_ENTRY(fmt, args...)        \
		pr_notice("["LOG_TAG"]%s entry " fmt "\n", __func__, ##args)
#define LCM_FUNC_EXIT(fmt, args...)         \
		pr_notice("["LOG_TAG"]%s exit " fmt "\n", __func__, ##args)

#define LCM_FUNC_DBG_ENTRY(fmt, args...)    \
		pr_debug("["LOG_TAG"]%s entry " fmt "\n", __func__, ##args)
#define LCM_FUNC_DBG_EXIT(fmt, args...)     \
		pr_debug("["LOG_TAG"]%s exit " fmt "\n", __func__, ##args)


static LCM_UTIL_FUNCS lcm_util = {
	.set_reset_pin = NULL,
	.udelay = NULL,
	.mdelay = NULL,
};

/* (lcm_util.set_reset_pin((v))) */
#define SET_RESET_PIN(v) lcm_set_gpio_output(GPIO_LCD_RST, v)

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

#define dsi_set_cmdq_V4(para_tbl, size, force_update) \
	lcm_util.dsi_set_cmdq_V4(para_tbl, size, force_update)
#define dsi_get_cmdq_V4(para_tbl, size, force_update) \
	lcm_util.dsi_get_cmdq_V4(para_tbl, size, force_update)
#define dsi_set_cmdq_backlight(para_tbl, size, force_update) \
			lcm_util.dsi_set_cmdq_backlight(para_tbl, size, force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)	 \
	lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)                                   \
	lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)               \
	lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)                                    \
	lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)            \
	lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#define VID_FTAB_STARRY_BOE   0x10  /* Starry, using FT8201AB IC */
#define VID_FTAB_KD_INX       0x17  /* KD, using FT8201AB IC */
#define VID_RSV_EXT_CODE      0x02  /* ID extension code */
#define VID_NULL              0xff	/* NULL */
#define RSVD_CODE             0x81	/* NULL */

/*
 * power on sequnce: vrf18/AVDD/AVEE -> LP11 -> RESET high
 * the LP11 will be enable after resume_power() and before resume()
 * so   set vrf18/AVDD/AVEE at resume_power() //TRS_ON_PHASE1
 * then set RESET at resume()                 //TRS_ON_PHASE2
*/
#define TRS_SLP_TO_OFF       0
#define TRS_OFF_TO_ON        1
#define TRS_SLP              2
#define TRS_SLP_TO_ON        3    /* sleep out */
#define TRS_OFF              4
#define TRS_ON_PHASE1        5
#define TRS_ON_PHASE2        6
#define TRS_DSB              7
#define TRS_DSB_TO_ON        8
#define TRS_DSB_TO_OFF       9
#define TRS_KEEP_CUR         0xFD /* keep current state */
#define TRS_NOT_PSB          0xFE /* not possible */

#define LCM_OFF             0
#define LCM_ON              1
#define LCM_ON1             2
#define LCM_ON2             3
#define LCM_SLP             4
#define LCM_DSB             5

/* trans_table[old_state][new_state]; */
static int trans_table[][6] = {
/*            LCM_OFF         LCM_ON         LCM_ON1        LCM_ON2        LCM_SLP       LCM_DSB   */
/* LCM_OFF */{TRS_KEEP_CUR,   TRS_OFF_TO_ON, TRS_ON_PHASE1, TRS_NOT_PSB,   TRS_NOT_PSB,  TRS_NOT_PSB},
/* LCM_ON  */{TRS_OFF,        TRS_KEEP_CUR,  TRS_KEEP_CUR,  TRS_KEEP_CUR,  TRS_SLP,      TRS_DSB},
/* LCM_ON1 */{TRS_NOT_PSB,    TRS_NOT_PSB,   TRS_KEEP_CUR,  TRS_ON_PHASE2, TRS_NOT_PSB,  TRS_NOT_PSB},
/* LCM_ON2 */{TRS_OFF,        TRS_KEEP_CUR,  TRS_NOT_PSB,   TRS_KEEP_CUR,  TRS_SLP,      TRS_DSB},
/* LCM_SLP */{TRS_SLP_TO_OFF, TRS_SLP_TO_ON, TRS_KEEP_CUR,  TRS_SLP_TO_ON, TRS_KEEP_CUR, TRS_NOT_PSB},
/* LCM_DSB */{TRS_DSB_TO_OFF, TRS_DSB_TO_ON, TRS_KEEP_CUR,  TRS_DSB_TO_ON, TRS_NOT_PSB,  TRS_KEEP_CUR}
};

static char *lcm_state_str[] = {
	"LCM_OFF",
	"LCM_ON",
	"LCM_ON1",
	"LCM_ON2",
	"LCM_SLP",
	"LCM_DSB",
};

#define LCM_Tdb 4                  /* display bias ready */
#define LCM_TVDDI_TP_RST   2       /* spec: > 1ms */
#define LCM_TVDDI_AVDD     5       /* spec: > 3ms */
#define LCM_TAVDD_AVEE     5       /* spec: > 3ms */
#define LCM_TAVEE_RESX_H   5       /* spec: > 3ms */
#define LCM_TRESX_H_L      5       /* spec: > 3ms */
#define LCM_TRESX_L_H      2       /* spec: > 10us */
#define LCM_TINIT_READY    80      /* vendor: > 80ms */
#define LCM_TSLPOUT_DSPON  85      /* spec: > 81ms */
#define LCM_TRESX_AVDD     5       /* spec: > 3ms */
#define LCM_TAVEE_VDDI     5       /* spec: > 3ms */
#define LCM_TRESX_ID_READ  12      /* spec: > 10ms */
#define LCM_TSLPIN_RESX_L  60      /* spec: > 51.6ms */
#define LCM_TIC_DSTB_READY 17      /* vendor: > 16.6ms */

#define LCM_ID1_VENDOR_SHIFT    5
#define LCM_ID1_RSV_CODE_MASK	0x18
#define LCM_ID1_RSV_CODE_SHIFT	3
#define LCM_ID_RETRY_CNT        3
#define LCM_AVDD_VOL            6000000
#define LCM_AVEE_VOL            6000000
#define LCM_DELAY_SHUTDOWN_TO_DISPLAY_SHUTDOWN

#define GPIO_OUT_ONE 1
#define GPIO_OUT_ZERO 0

#define GetField(Var, Mask, Shift) \
	(((Var) >> (Shift)) & (Mask))

#define SetField(Var, Mask, Shift, Val) \
	((Var) = (((Var) & ~((Mask) << (Shift))) | \
				(((Val) & (Mask)) << (Shift))))

#define REGFLAG_DELAY         0xFFFC
#define REGFLAG_UDELAY        0xFFFB
#define REGFLAG_END_OF_TABLE  0xFFFD

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static struct LCM_setting_table register_CMD2_EN[] = {
	{0x00, 1, {0x00} },
	{0xFF, 3, {0x82, 0x01, 0x01} },
};

static struct LCM_setting_table register_FOCAL_CMD_EN[] = {
	{0x00, 1, {0x80} },
	{0xFF, 2, {0x82, 0x01} },
};

static struct LCM_setting_table register_READ_EN[] = {
	{0x00, 1, {0x88} },
	{0xB0, 1, {0x07} },
};

static struct LCM_setting_table register_READ_DIS[] = {
	{0x00, 1, {0x88} },
	{0xB0, 1, {0x01} },
};

static struct LCM_setting_table register_SELECT_MASTER[] = {
	{0x00, 1, {0x00} },
	{0xFA, 1, {0x02} },
};

static struct LCM_setting_table register_SELECT_M_AND_S[] = {
	{0x00, 1, {0x00} },
	{0xFA, 1, {0x5A} },
};

/* reserve
static struct LCM_setting_table register_FOCAL_CMD_DIS[] = {
	{0x00, 1, {0x80} },
	{0xFF, 2, {0x00, 0x00} },
};

static struct LCM_setting_table register_CMD2_DIS[] = {
	{0x00, 1, {0x00} },
	{0xFF, 3, {0x00, 0x00, 0x00} },
};
*/

static struct LCM_setting_table slp_out_disp_on_setting[] = {
	{0x35, 1, {0x0} },
	{0x11, 0, {} },
	{REGFLAG_DELAY, LCM_TSLPOUT_DSPON, {} },
	{0x29, 0, {} },
	/* Backlight control on */
	{0x53, 1, {0x2C} }
};

static struct LCM_setting_table sleep_in_setting[] = {
	{0x28, 0, {} },
	{0x10, 0, {} }
};

static struct LCM_setting_table register_DSTB_IN[] = {
	{0x00, 1, {0x00} },
	{0xF7, 4, {0x5A, 0xA5, 0x95, 0x27} },
};

static struct LCM_setting_table init_setting[] = {
};

/* Using I2C command to prevent half screen happen */
#define WORKAROUND_FT8201_SETUP_LCM_CABC_THROUGH_TOUCH_I2C

#ifdef WORKAROUND_FT8201_SETUP_LCM_CABC_THROUGH_TOUCH_I2C

#define CABC_ON_VAL  0x03
#define CABC_OFF_VAL 0x00
extern int ft8201_share_cabc_write(u8 mode);
extern int ft8201_share_cabc_read(u8 *mode);

#else

#define	DSI_DCS_SHORT_PACKET_ID_0	0x05
#define	DSI_DCS_SHORT_PACKET_ID_1	0x15
#define	DSI_DCS_LONG_PACKET_ID		0x39
#define	DSI_DCS_READ_PACKET_ID		0x06

static struct LCM_setting_table_V4 cabc_on_setting[] = {
	{DSI_DCS_SHORT_PACKET_ID_1, 0x55, 1, {0x03}, 0 }
};

static struct LCM_setting_table_V4 cabc_off_setting[] = {
	{DSI_DCS_SHORT_PACKET_ID_1, 0x55, 1, {0x00}, 0 }
};

#endif /* WORKAROUND_FT8201_SETUP_LCM_CABC_THROUGH_TOUCH_I2C */

static DEFINE_MUTEX(ft8201ab_share_power_mutex);

static struct regulator *reg_lcm_vcc1v8;
static unsigned int LCM_RST_GPIO;
static unsigned int TP_RST_GPIO;
static unsigned int g_vendor_id = VID_NULL;
static int old_state = LCM_OFF;
static atomic_t g_power_request = ATOMIC_INIT(0);

static void lcm_power_manager(int);
static void lcm_power_manager_lock(int);
static void lcm_set_gpio_output(unsigned int GPIO, unsigned int output);
static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update);

/*
* 1. export function
* 2. lcm_platform_driver
* 3. lcm_driver
*/

/* export function start */
int ft8201ab_share_request_powerpin_resource(struct device *dev)
{
	struct device_node *lcm_common_resource = NULL;
	int ret = 0;

	LCM_FUNC_ENTRY();
	mutex_lock(&ft8201ab_share_power_mutex);
	if (atomic_read(&g_power_request) > 0) {
		LCM_LOGI("lcm resource already init before");
		goto init_done;
	}

	lcm_common_resource = of_parse_phandle(dev->of_node,
			"lcm_common_resource", 0);
	if (!lcm_common_resource)
		goto err_dts_parsing;

	LCM_RST_GPIO = of_get_named_gpio(lcm_common_resource,
			"lcm_rst_gpio", 0);
	ret = gpio_request(LCM_RST_GPIO, "LCM_RST_GPIO");
	if (ret != 0) {
		LCM_ERR("gpio request LCM_RST_GPIO = 0x%x fail with %d",
			LCM_RST_GPIO, ret);
		goto err_lcm_rst;
	}

	TP_RST_GPIO = of_get_named_gpio(lcm_common_resource,
			"tp_rst_gpio", 0);
	ret = gpio_request(TP_RST_GPIO, "TP_RST_GPIO");
	if (ret != 0) {
		LCM_ERR("gpio request TP_RST_GPIO = 0x%x fail with %d",
			TP_RST_GPIO, ret);
		goto err_tp_rst;
	}

	reg_lcm_vcc1v8 = regulator_get(dev, "lcm_common_vcc1v8");
	if (IS_ERR(reg_lcm_vcc1v8)) {
		LCM_ERR("Failed to request lcm_common_vcc1v8 power supply: %ld",
			PTR_ERR(reg_lcm_vcc1v8));
		goto err_vcc1v8_1;
	}

	ret = regulator_enable(reg_lcm_vcc1v8);
	if (ret != 0) {
		LCM_ERR("Failed to enable lcm_vcc1v8 power supply: %d", ret);
		goto err_vcc1v8_2;
	}

	ret = display_bias_regulator_init();
	if (ret != 0) {
		LCM_ERR("Failed to enable display bias supply: %d", ret);
		goto err_disp_bias;
	}

	ret = display_bias_enable_vol(LCM_AVDD_VOL, LCM_AVEE_VOL, LCM_TAVDD_AVEE);
	if (ret != 0) {
		LCM_ERR("Failed to config display bias supply: %d", ret);
		goto err_disp_bias;
	}
	old_state = LCM_ON;

init_done:
	atomic_inc(&g_power_request);
	mutex_unlock(&ft8201ab_share_power_mutex);
	LCM_FUNC_EXIT("cnt(%d)", atomic_read(&g_power_request));
	return 0;

err_disp_bias:
	regulator_disable(reg_lcm_vcc1v8);
err_vcc1v8_2:
	regulator_put(reg_lcm_vcc1v8);
	reg_lcm_vcc1v8 = NULL;
err_vcc1v8_1:
	gpio_free(TP_RST_GPIO);
err_tp_rst:
	gpio_free(LCM_RST_GPIO);
err_lcm_rst:
err_dts_parsing:
	mutex_unlock(&ft8201ab_share_power_mutex);
	LCM_FUNC_EXIT("fail to probe, cnt(%d)", atomic_read(&g_power_request));
	return -1;
}
EXPORT_SYMBOL(ft8201ab_share_request_powerpin_resource);

void ft8201ab_share_powerpin_on(void)
{
	mutex_lock(&ft8201ab_share_power_mutex);
	/*LCM resume power with ON1->ON2, touch resume with ON.*/
	if (atomic_read(&g_power_request) == 0)
		LCM_ERR("Touch poweron should NOT before LCM poweron");
	lcm_power_manager(LCM_ON);
	mutex_unlock(&ft8201ab_share_power_mutex);
}
EXPORT_SYMBOL(ft8201ab_share_powerpin_on);

void ft8201ab_share_powerpin_off(void)
{
	mutex_lock(&ft8201ab_share_power_mutex);
	lcm_power_manager(LCM_OFF);
	mutex_unlock(&ft8201ab_share_power_mutex);
}
EXPORT_SYMBOL(ft8201ab_share_powerpin_off);

void ft8201ab_share_tp_resetpin_high(void)
{
	LCM_LOGI("%s", __func__);
	mutex_lock(&ft8201ab_share_power_mutex);
	if (atomic_read(&g_power_request) <= 0) {
		LCM_ERR("%s skip, bceause res_cnt(%d) <= 0",
				__func__, atomic_read(&g_power_request));
	} else {
		lcm_set_gpio_output(TP_RST_GPIO, GPIO_OUT_ONE);
	}
	mutex_unlock(&ft8201ab_share_power_mutex);
}
EXPORT_SYMBOL(ft8201ab_share_tp_resetpin_high);

void ft8201ab_share_tp_resetpin_low(void)
{
	LCM_LOGI("%s", __func__);
	mutex_lock(&ft8201ab_share_power_mutex);
	if (atomic_read(&g_power_request) <= 0) {
		LCM_ERR("%s skip, bceause res_cnt(%d) <= 0",
				__func__, atomic_read(&g_power_request));
	} else {
		lcm_set_gpio_output(TP_RST_GPIO, GPIO_OUT_ZERO);
	}
	mutex_unlock(&ft8201ab_share_power_mutex);
}
EXPORT_SYMBOL(ft8201ab_share_tp_resetpin_low);

unsigned int ft8201ab_incell_compare_id(void)
{
	LCM_LOGI("%s", __func__);

	/* lcm id get from Lk */
	if ((g_vendor_id == VID_FTAB_STARRY_BOE) ||
		(g_vendor_id == VID_FTAB_KD_INX))
		return g_vendor_id;
	else
		return VID_NULL;
}
EXPORT_SYMBOL(ft8201ab_incell_compare_id);

/* export function end */


/* lcm_platform_driver start */
static int lcm_driver_probe(struct device *dev, void const *data)
{
	int ret = 0;

	if (ft8201ab_incell_compare_id() != VID_NULL) {
		if (ft8201ab_share_request_powerpin_resource(dev) < 0) {
			LCM_ERR("fail to request power pins, Panel ID = 0x%x",
					g_vendor_id);
			ret = -1;
		} else {
			LCM_LOGI("lcm probe success, Panel ID = 0x%x",
					g_vendor_id);
			ret = 0;
		}
	} else {
		LCM_ERR("lcm probe fail, mismatch Panel ID = 0x%x", g_vendor_id);
		ret = -ENODEV;
	}

	return ret;
}

static const struct of_device_id lcm_platform_of_match[] = {
	{
		.compatible = "focal,ft8201ab",
		.data = 0,
	}, {
		/* sentinel */
	}
};

MODULE_DEVICE_TABLE(of, platform_of_match);

static int lcm_platform_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;

	LCM_LOGI("%s", __func__);
	id = of_match_node(lcm_platform_of_match, pdev->dev.of_node);
	if (!id)
		return -ENODEV;

	return lcm_driver_probe(&pdev->dev, id->data);
}

static int lcm_platform_remove(struct platform_device *pdev)
{
	LCM_LOGI("%s", __func__);
	return 0;
}

static void lcm_platform_shutdown(struct platform_device *pdev)
{
	LCM_LOGI("%s", __func__);
#ifdef LCM_DELAY_SHUTDOWN_TO_DISPLAY_SHUTDOWN
	mutex_lock(&ft8201ab_share_power_mutex);
	atomic_dec(&g_power_request);
	mutex_unlock(&ft8201ab_share_power_mutex);
#endif
}

static struct platform_driver lcm_driver = {
	.probe = lcm_platform_probe,
	.remove = lcm_platform_remove,
	.shutdown = lcm_platform_shutdown,
	.driver = {
		.name = "ft8201ab_wuxga_dsi_vdo",
		.owner = THIS_MODULE,
		.of_match_table = lcm_platform_of_match,
	},
};

static int __init lcm_platform_init(void)
{
	LCM_DBG("Register panel driver for ft8201ab_wuxga_dsi_vdo");
	if (ft8201ab_incell_compare_id() != VID_NULL) {
		if (platform_driver_register(&lcm_driver)) {
			LCM_ERR("Failed to register this driver!");
			return -ENODEV;
		}
	} else {
		LCM_LOGI("mismatch Panel ID = 0x%x", g_vendor_id);
		return -ENODEV;
	}

	return 0;
}


static void __exit lcm_platform_exit(void)
{
	platform_driver_unregister(&lcm_driver);
	LCM_DBG("Unregister this driver done");
}

late_initcall(lcm_platform_init);
module_exit(lcm_platform_exit);
MODULE_AUTHOR("Compal");
MODULE_DESCRIPTION("LCM display subsystem driver");
MODULE_LICENSE("GPL");
/* lcm_platform_driver end */


/* lcm_driver start */
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
	gpio_direction_output(GPIO, (output > 0) ?
			GPIO_OUT_ONE : GPIO_OUT_ZERO);
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
	params->lcm_if = LCM_INTERFACE_DSI0;
	params->lcm_cmd_if = LCM_INTERFACE_DSI0;

	params->dsi.mode = SYNC_EVENT_VDO_MODE;

	params->dsi.LANE_NUM = LCM_FOUR_LANE;

	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

	params->dsi.packet_size = 256;
	params->dsi.intermediat_buffer_num = 0;
	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.word_count = FRAME_WIDTH * 3;

	params->dsi.vertical_sync_active = 6;
	params->dsi.vertical_backporch = 10;
	params->dsi.vertical_frontporch = 205;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 6;
	params->dsi.horizontal_backporch = 8;
	params->dsi.horizontal_frontporch = 8;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;

	LCM_LOGI("LCM(id=0x%x, VFP=%d)", g_vendor_id,
		params->dsi.vertical_frontporch);

	params->dsi.PLL_CLOCK = 497;

	params->dsi.clk_lp_per_line_enable = 0;
	params->dsi.ssc_disable = 1;
	params->dsi.cont_clock = 1; /* Keep HS serial clock running */
	params->dsi.DA_HS_EXIT = 10;
	params->dsi.CLK_ZERO = 16;
	params->dsi.HS_ZERO = 25;
	params->dsi.HS_TRAIL = 10;
	params->dsi.CLK_TRAIL = 1;
	params->dsi.CLK_HS_POST = 8;
	params->dsi.CLK_HS_EXIT = 6;
	/* params->dsi.CLK_HS_PRPR = 1; */

	params->dsi.TA_GO = 8;
	params->dsi.TA_GET = 10;

	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
	params->dsi.lcm_esd_check_table[1].cmd = 0x0E;
	params->dsi.lcm_esd_check_table[1].count = 1;
	params->dsi.lcm_esd_check_table[1].para_list[0] = 0x80;
	params->dsi.lcm_esd_check_table[2].cmd = 0xAC;
	params->dsi.lcm_esd_check_table[2].count = 1;
	params->dsi.lcm_esd_check_table[2].para_list[0] = 0x00;
	/*params->dsi.cust_clk_impendence = 0xf;*/

	/* The values are in mm.*/
	/* Add these values for reporting correct xdpi and ydpi values */
	params->physical_width = 135;
	params->physical_height = 217;
}

static void init_lcm_registers(void)
{

	switch (g_vendor_id) {
	case VID_FTAB_STARRY_BOE:
	case VID_FTAB_KD_INX:
		LCM_LOGI("init lcm(id=0x%x) registers", g_vendor_id);

		push_table(init_setting,
			sizeof(init_setting) / sizeof(struct LCM_setting_table), 1);
		push_table(slp_out_disp_on_setting,
			sizeof(slp_out_disp_on_setting) / sizeof(struct LCM_setting_table), 1);
		push_table(register_CMD2_EN,
			sizeof(register_CMD2_EN) / sizeof(struct LCM_setting_table), 1);
		push_table(register_FOCAL_CMD_EN,
			sizeof(register_FOCAL_CMD_EN) / sizeof(struct LCM_setting_table), 1);
		push_table(register_READ_EN,
			sizeof(register_READ_EN) / sizeof(struct LCM_setting_table), 1);
		push_table(register_SELECT_MASTER,
			sizeof(register_SELECT_MASTER) / sizeof(struct LCM_setting_table), 1);

		break;

	default:
		LCM_LOGI("Not supported yet, g_vendor_id = 0x%x",
			g_vendor_id);

		break;
	}
}

static void lcm_init_power(void)
{
	LCM_LOGI("%s", __func__);
	lcm_power_manager_lock(LCM_ON1);
}

static void lcm_init(void)
{
	LCM_LOGI("%s", __func__);
	lcm_power_manager_lock(LCM_ON2);
}

static void lcm_resume_power(void)
{
	LCM_LOGI("%s", __func__);
	lcm_power_manager_lock(LCM_ON1);
}

static void lcm_resume(void)
{
	LCM_LOGI("%s", __func__);
#if defined(CONFIG_AMAZON_METRICS_LOG) || defined(CONFIG_AMZN_METRICS_LOG)
	snprintf(metric_buf, sizeof(metric_buf),
		"%s:lcd:resume=1;CT;1:NR", __func__);
	log_to_metrics(ANDROID_LOG_INFO, "lcd", metric_buf);
#endif
#if defined(CONFIG_AMZN_MINERVA_METRICS_LOG) || defined(CONFIG_AMAZON_MINERVA_METRICS_LOG)
	minerva_metrics_log(metric_buf, 512, "%s:%s:100:%s,%s,%s,%s,lcm_state=lcm_resume;SY,"
			"ESD_Recovery=0;IN:us-east-1",
			METRICS_LCD_GROUP_ID, METRICS_LCD_SCHEMA_ID,
			PREDEFINED_ESSENTIAL_KEY, PREDEFINED_MODEL_KEY,
			PREDEFINED_TZ_KEY, PREDEFINED_DEVICE_LANGUAGE_KEY);
#endif
	lcm_power_manager_lock(LCM_ON2);
}

static void lcm_suspend(void)
{
	LCM_LOGI("%s", __func__);
#if defined(CONFIG_AMAZON_METRICS_LOG) || defined(CONFIG_AMZN_METRICS_LOG)
	snprintf(metric_buf, sizeof(metric_buf),
		"%s:lcd:suspend=1;CT;1:NR", __func__);
	log_to_metrics(ANDROID_LOG_INFO, "lcd", metric_buf);
#endif
#if defined(CONFIG_AMZN_MINERVA_METRICS_LOG) || defined(CONFIG_AMAZON_MINERVA_METRICS_LOG)
	minerva_metrics_log(metric_buf, 512, "%s:%s:100:%s,%s,%s,%s,lcm_state=lcm_suspend;SY,"
			"ESD_Recovery=0;IN:us-east-1",
			METRICS_LCD_GROUP_ID, METRICS_LCD_SCHEMA_ID,
			PREDEFINED_ESSENTIAL_KEY, PREDEFINED_MODEL_KEY,
			PREDEFINED_TZ_KEY, PREDEFINED_DEVICE_LANGUAGE_KEY);
#endif
	lcm_power_manager_lock(LCM_DSB);
}

static void lcm_suspend_power(void)
{
	LCM_LOGI("%s", __func__);
	lcm_power_manager_lock(LCM_OFF);
}

static int lcm_power_controller(int state)
{
	int ret = 0;

	LCM_FUNC_DBG_ENTRY();
	switch (state) {
	case TRS_SLP:
		push_table(register_SELECT_M_AND_S,
			sizeof(register_SELECT_M_AND_S) / sizeof(struct LCM_setting_table), 1);
		push_table(register_READ_DIS,
			sizeof(register_READ_DIS) / sizeof(struct LCM_setting_table), 1);
		push_table(sleep_in_setting,
			sizeof(sleep_in_setting) / sizeof(struct LCM_setting_table), 1);
		MDELAY(LCM_TSLPIN_RESX_L);

		break;

	case TRS_DSB:
		push_table(register_SELECT_M_AND_S,
			sizeof(register_SELECT_M_AND_S) / sizeof(struct LCM_setting_table), 1);
		push_table(register_READ_DIS,
			sizeof(register_READ_DIS) / sizeof(struct LCM_setting_table), 1);
		push_table(sleep_in_setting,
			sizeof(sleep_in_setting) / sizeof(struct LCM_setting_table), 1);
		MDELAY(LCM_TSLPIN_RESX_L);
		push_table(register_DSTB_IN,
			sizeof(register_DSTB_IN) / sizeof(struct LCM_setting_table), 1);

		break;

	case TRS_SLP_TO_ON:
		init_lcm_registers();
		break;

	case TRS_DSB_TO_ON:
		lcm_set_gpio_output(LCM_RST_GPIO, GPIO_OUT_ZERO);
		MDELAY(LCM_TRESX_L_H);
		lcm_set_gpio_output(LCM_RST_GPIO, GPIO_OUT_ONE);
		MDELAY(LCM_TINIT_READY);
		init_lcm_registers();
		break;

	case TRS_SLP_TO_OFF:
	case TRS_DSB_TO_OFF:
		MDELAY(LCM_TIC_DSTB_READY);
		lcm_set_gpio_output(TP_RST_GPIO, GPIO_OUT_ZERO);
		lcm_set_gpio_output(LCM_RST_GPIO, GPIO_OUT_ZERO);
		MDELAY(LCM_TRESX_AVDD);
		ret = display_bias_disable();
		if (ret < 0)
			LCM_ERR("disable display bias power fail: %d",
				ret);
		MDELAY(LCM_Tdb);
		MDELAY(LCM_TAVEE_VDDI);
		if (reg_lcm_vcc1v8) {
			ret = regulator_disable(reg_lcm_vcc1v8);
			if (ret != 0)
				LCM_ERR("disable reg_lcm_vcc1v8 fail");
		}
		break;

	case TRS_ON_PHASE1:
		if (reg_lcm_vcc1v8) {
			ret = regulator_enable(reg_lcm_vcc1v8);
			if (ret != 0)
				LCM_ERR("enable reg_lcm_vcc1v8 fail");
		}

		MDELAY(LCM_TVDDI_TP_RST);
		lcm_set_gpio_output(TP_RST_GPIO, GPIO_OUT_ONE);

		MDELAY(LCM_TVDDI_AVDD - LCM_TVDDI_TP_RST);

		ret = display_bias_enable_vol(LCM_AVDD_VOL, LCM_AVEE_VOL, LCM_TAVDD_AVEE);
		if (ret < 0)
			LCM_ERR("enable display bias power fail: %d", ret);
		MDELAY(LCM_Tdb);
		break;

	case TRS_ON_PHASE2:
		lcm_set_gpio_output(LCM_RST_GPIO, GPIO_OUT_ZERO);
		MDELAY(LCM_TAVEE_RESX_H);
		lcm_set_gpio_output(LCM_RST_GPIO, GPIO_OUT_ONE);
		MDELAY(LCM_TRESX_H_L);
		lcm_set_gpio_output(LCM_RST_GPIO, GPIO_OUT_ZERO);
		MDELAY(LCM_TRESX_L_H);
		lcm_set_gpio_output(LCM_RST_GPIO, GPIO_OUT_ONE);
		MDELAY(LCM_TINIT_READY);
		init_lcm_registers();
		break;

	case TRS_OFF_TO_ON:
		if (reg_lcm_vcc1v8) {
			ret = regulator_enable(reg_lcm_vcc1v8);
			if (ret != 0)
				LCM_ERR("enable reg_lcm_vcc1v8 fail");
		}
		MDELAY(LCM_TVDDI_TP_RST);
		lcm_set_gpio_output(TP_RST_GPIO, GPIO_OUT_ONE);

		MDELAY(LCM_TVDDI_AVDD - LCM_TVDDI_TP_RST);
		ret = display_bias_enable_vol(LCM_AVDD_VOL, LCM_AVEE_VOL, LCM_TAVDD_AVEE);
		if (ret < 0)
			LCM_ERR("enable display bias power fail: %d", ret);
		MDELAY(LCM_Tdb);
		MDELAY(LCM_TAVEE_RESX_H);
		lcm_set_gpio_output(LCM_RST_GPIO, GPIO_OUT_ONE);
		MDELAY(LCM_TRESX_H_L);
		lcm_set_gpio_output(LCM_RST_GPIO, GPIO_OUT_ZERO);
		MDELAY(LCM_TRESX_L_H);
		lcm_set_gpio_output(LCM_RST_GPIO, GPIO_OUT_ONE);
		MDELAY(LCM_TINIT_READY);
		init_lcm_registers();
		break;

	case TRS_OFF:
		push_table(register_SELECT_M_AND_S,
			sizeof(register_SELECT_M_AND_S) / sizeof(struct LCM_setting_table), 1);
		push_table(register_READ_DIS,
			sizeof(register_READ_DIS) / sizeof(struct LCM_setting_table), 1);
		push_table(sleep_in_setting,
			sizeof(sleep_in_setting) / sizeof(struct LCM_setting_table), 1);
		MDELAY(LCM_TSLPIN_RESX_L);
		push_table(register_DSTB_IN,
			sizeof(register_DSTB_IN) / sizeof(struct LCM_setting_table), 1);

		MDELAY(LCM_TIC_DSTB_READY);
		lcm_set_gpio_output(TP_RST_GPIO, GPIO_OUT_ZERO);
		lcm_set_gpio_output(LCM_RST_GPIO, GPIO_OUT_ZERO);
		MDELAY(LCM_TRESX_AVDD);
		ret = display_bias_disable();
		if (ret < 0)
			LCM_ERR("disable display bias power fail: %d", ret);
		MDELAY(LCM_Tdb);
		MDELAY(LCM_TAVEE_VDDI);
		if (reg_lcm_vcc1v8) {
			ret = regulator_disable(reg_lcm_vcc1v8);
			if (ret != 0)
				LCM_ERR("disable reg_lcm_vcc1v8 fail");
		}
		break;
	case TRS_KEEP_CUR:
		ret = -1;
		LCM_LOGI("keep current state");
		break;
	case TRS_NOT_PSB:
		ret = -1;
		LCM_ERR("Unexpected state change");
		break;
	default:
		ret = -1;
		LCM_ERR("%s undefined state (%d)", __func__, state);
		break;
	}
	LCM_FUNC_DBG_EXIT();
	return ret;
}

static void lcm_power_manager_lock(int pwr_state)
{
	LCM_FUNC_DBG_ENTRY();
	mutex_lock(&ft8201ab_share_power_mutex);
	lcm_power_manager(pwr_state);
	mutex_unlock(&ft8201ab_share_power_mutex);
	LCM_FUNC_DBG_EXIT();
}

static void lcm_power_manager(int new_state)
{
	int tmp_state = LCM_ON;
	int new_trns;
	int ret = 0;

	LCM_FUNC_DBG_ENTRY();

	tmp_state = old_state;

	switch (new_state) {
	case LCM_ON:
	case LCM_ON1:
		atomic_inc(&g_power_request);
		break;
	case LCM_ON2:
	case LCM_SLP:
	case LCM_DSB:
		break;
	case LCM_OFF:
		atomic_dec(&g_power_request);
		if (atomic_read(&g_power_request) <= 0)
			new_state = LCM_OFF;
		else
			new_state = old_state;
		break;
	}

	new_trns = trans_table[old_state][new_state];

	ret = lcm_power_controller(new_trns);

	if (ret == 0)
		old_state = new_state;

	LCM_LOGI("%s requesting %s, changed from %s to %s, count=%d",
		__func__, lcm_state_str[new_state],
		lcm_state_str[tmp_state], lcm_state_str[old_state],
		atomic_read(&g_power_request));

	LCM_FUNC_DBG_EXIT();
}

static int __init get_lk_parameter(char *str)
{
	int lcm_id;

	if (get_option(&str, &lcm_id)) {
		g_vendor_id = lcm_id;
		LCM_LOGI("lk lcm_id value=0x%x", lcm_id);
	}
	return 0;
}
__setup("lcm_id=", get_lk_parameter);

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;
	unsigned cmd;

	for (i = 0; i < count; i++) {
		cmd = table[i].cmd;

		switch (cmd) {

		case REGFLAG_DELAY:
			MDELAY(table[i].count);
			break;

		case REGFLAG_UDELAY:
			UDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE:
			break;

		default:
			dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}

static void lcm_setbacklight_mode(unsigned int mode)
{
	static unsigned last_mode = CABC_OFF;
#ifndef WORKAROUND_FT8201_SETUP_LCM_CABC_THROUGH_TOUCH_I2C
	int ret;
#endif

	LCM_LOGI("%s setup CABC mode=0x%x", __func__, mode);

	if (mode > CABC_MOVIE) {
		LCM_ERR("CABC mode 0x%x is not supported now", mode);
		goto err_unsupport;
	}

	if (mode == last_mode) {
		LCM_LOGI("exit due to no change on cabc mode(%d)", mode);
		goto err_nochange;
	} else {
		last_mode = mode;
	}

	switch (mode) {
	case CABC_MOVIE:
		LCM_DBG("CABC_MOVIE");
#ifdef WORKAROUND_FT8201_SETUP_LCM_CABC_THROUGH_TOUCH_I2C
		if (ft8201_share_cabc_write(CABC_ON_VAL) < 0)
			LCM_ERR("CABC write via I2C fail");
#else
		dsi_set_cmdq_V4(cabc_on_setting,
			sizeof(cabc_on_setting) / sizeof(struct LCM_setting_table_V4), 1);
#endif /* WORKAROUND_FT8201_SETUP_LCM_CABC_THROUGH_TOUCH_I2C */

		break;
	case CABC_OFF:
		LCM_DBG("CABC_OFF");
#ifdef WORKAROUND_FT8201_SETUP_LCM_CABC_THROUGH_TOUCH_I2C
		if (ft8201_share_cabc_write(CABC_OFF_VAL) < 0)
			LCM_ERR("CABC Write via I2C fail");
#else
		dsi_set_cmdq_V4(cabc_off_setting,
			sizeof(cabc_off_setting) / sizeof(struct LCM_setting_table_V4), 1);
#endif /* WORKAROUND_FT8201_SETUP_LCM_CABC_THROUGH_TOUCH_I2C */
		break;
	default:
		LCM_ERR("CABC mode 0x%x is not supported now", mode);
		break;
	}

err_unsupport:
err_nochange:
	return;
}

LCM_DRIVER ft8201ab_wuxga_dsi_vdo_lcm_drv = {
	.name = "ft8201ab_wuxga_dsi_vdo",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.init_power = lcm_init_power,
	.suspend_power = lcm_suspend_power,
	.suspend = lcm_suspend,
	.resume_power = lcm_resume_power,
	.resume = lcm_resume,
	.set_backlight_mode = lcm_setbacklight_mode,
};
/* lcm_driver end */
