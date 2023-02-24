/*
 * Copyright (C) 2020 MediaTek Inc.
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
#include <linux/leds.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#endif

#include <upmu_common.h>

/* mt8183 set backlight */
#include "mtk_leds_sw.h"

#include "lcm_drv.h"
#define FRAME_WIDTH  (800)
#define FRAME_HEIGHT (1280)
#define REGFLAG_DELAY 0xFE
/* END OF REGISTERS MARKER */
#define REGFLAG_END_OF_TABLE 0xFC


#define PROJECT_ID_PROTO	0x2	/* abc123 use abc123's display in ptoto build, proiect_id is 0x2 */
#define PROJECT_ID	0x3	/* abc123's own project_id is 0x3 */

#define LCM_ID_LOW_MASK  0x0F
#define LCM_ID_HIGH_MASK 0xF0

#define ILI9881C_KD_HSD	0x3	/* KD, using ILI9881C IC, HSD cell */
#define ILI9881C_BOE	0x6	/* BOE, using ILI9881C IC */
#define ILI9881C_INX	0x8	/* INX, using ILI9881C IC */
#define ILI9881C_KD_HSD_ONYX	0x9	/* HSD, using ILI9881C IC */
#define ILI9881C_KD_INX_O	0xA	/* INX-O, using ILI9881C IC */

#define ST_UNKNOWN	0xFF

#define GPIO_OUT_ONE 1
#define GPIO_OUT_ZERO 0
#define GPIO_OUT_HIGH 1
#define GPIO_OUT_LOW 0
#define TRUE 1
#define FALSE 0

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {
	.set_reset_pin = NULL,
	.set_gpio_out = NULL,
	.udelay = NULL,
	.mdelay = NULL,
};

#define SET_RESET_PIN(v)      					(lcm_util.set_reset_pin((v)))
#define SET_GPIO_OUT(n, v)	        			(lcm_util.set_gpio_out((n), (v)))
#define UDELAY(n) 						(lcm_util.udelay(n))
#define MDELAY(n) 						(lcm_util.mdelay(n))

static unsigned char project_id;	/* Indentify abc123 hvt lcm(KD_HSD+ILI9881C) vendor_id(0x3) and proto lcm(KD_HSD+JD936X)(0x3) */
static unsigned char vendor_id;
static unsigned char lcm_id;

static struct regulator *lcm_vio;

static unsigned int GPIO_LCD_RST_EN;
static unsigned int GPIO_LCD_PWR_P_EN;
static unsigned int GPIO_LCD_PWR_N_EN;

extern int backlight_brightness_set(int level);
static void lcm_get_gpio_infor(struct device *dev);
static int lcm_get_vio_supply(struct device *dev);

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)						lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)			lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)						lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   		lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#define   LCM_DSI_CMD_MODE	0
struct LCM_setting_table
{
	unsigned char cmd;
	unsigned char count;
	unsigned char para_list[64];
};

#ifdef BUILD_LK

#ifdef GPIO_LCM_RST
#define GPIO_LCD_RST_EN		GPIO_LCM_RST
#else
#define GPIO_LCD_RST_EN		(0xFFFFFFFF)
#endif
#endif

static int __init setup_lcm_id(char *str)
{
	int id;

	if (get_option(&str, &id))
	{
		lcm_id = (unsigned char)id;
	}
	return 0;
}
__setup("jd936x_id=", setup_lcm_id);

static void get_lcm_id(void)
{
	vendor_id = (unsigned int)(lcm_id & LCM_ID_LOW_MASK);
	project_id = (unsigned int)((lcm_id & LCM_ID_HIGH_MASK) >> 4);
	pr_debug("LCM: vendor_id = 0x%x,project_id = 0x%x\n",vendor_id,project_id);
}

static void lcm_get_gpio_infor(struct device *dev)
{
	pr_debug("LCM: lcm_get_gpio_infor is going\n");

	GPIO_LCD_RST_EN = of_get_named_gpio(dev->of_node, "lcm_reset_gpio", 0);
	gpio_request(GPIO_LCD_RST_EN, "GPIO_LCD_RST_EN");
	pr_debug("LCM: GPIO_LCD_RST_EN=%d.\n",GPIO_LCD_RST_EN);

	GPIO_LCD_PWR_P_EN = of_get_named_gpio(dev->of_node, "lcm_power_p_gpio", 0);
	gpio_request(GPIO_LCD_PWR_P_EN, "GPIO_LCD_PWR_P_EN");
	pr_debug("LCM: GPIO_LCD_PWR_P_EN=%d.\n",GPIO_LCD_PWR_P_EN);

	GPIO_LCD_PWR_N_EN = of_get_named_gpio(dev->of_node, "lcm_power_n_gpio", 0);
	gpio_request(GPIO_LCD_PWR_N_EN, "GPIO_LCD_PWR_N_EN");
	pr_debug("LCM: GPIO_LCD_PWR_N_EN=%d.\n",GPIO_LCD_PWR_N_EN);
}

static void lcm_set_gpio_output(unsigned int GPIO, unsigned int output)
{
	gpio_direction_output(GPIO, output);
	gpio_set_value(GPIO, output);
}

static int lcm_get_vio_supply(struct device *dev)
{
        int ret = 0;
        struct regulator *lcm_vio_ldo;

        pr_debug("LCM: lcm_get_vio_supply is going\n");

        lcm_vio_ldo = devm_regulator_get(dev, "lcm-vio");
        if (IS_ERR(lcm_vio_ldo)) {
                ret = PTR_ERR(lcm_vio_ldo);
                dev_err(dev, "failed to get lcm-vio LDO, %d\n", ret);
                devm_regulator_put(lcm_vio_ldo);
                return ret;
        }

        lcm_vio = lcm_vio_ldo;
        pr_debug("LCM: lcm get lcm-vio supply ok.\n");

        return ret;
}

static int lcm_vio_supply_enable(void)
{
	int ret = 0;
	unsigned int volt;

	pr_debug("LCM: Enter lcm_vio_supply_enable\n");

	if (NULL == lcm_vio) {
		pr_err("LCM: Oops, do nothing and leave lcm_vio_supply_enable\n");
		return 0;
	}

	pr_debug("LCM: set regulator voltage lcm_vio voltage to 1.8V\n");

	ret = regulator_set_voltage(lcm_vio, 1800000, 1800000);

	if (ret != 0) {
		pr_err("LCM: lcm failed to set lcm_vio voltage: %d\n", ret);
		return ret;
	}

	/* get voltage settings again */
	volt = regulator_get_voltage(lcm_vio);
	if (volt == 1800000)
		pr_debug("LCM: check regulator voltage=1800000 pass!\n");
	else
		pr_err("LCM: check regulator voltage=1800000 fail! (voltage: %d)\n", volt);

	ret = regulator_enable(lcm_vio);

	if (ret != 0)
		pr_err("LCM: Failed to enable lcm_vio: %d\n", ret);

	return ret;
}

static int lcm_vio_supply_disable(void)
{
	int ret = 0;
	unsigned int isenable;

	pr_debug("LCM: lcm_vio_supply_disable\n");

	if (NULL == lcm_vio) {
		pr_err("LCM: Oops, do nothing and leave lcm_vio_supply_disable\n");
		return 0;
	}

	pr_debug("LCM: close supply lcm_vio(1V8)\n");

	isenable = regulator_is_enabled(lcm_vio);

	pr_debug("LCM: lcm_vio query regulator enable status[%d]\n", isenable);

	if (isenable) {
		ret = regulator_disable(lcm_vio);

		if (ret != 0) {
			pr_err("LCM: lcm failed to disable lcm_vio: %d\n", ret);
			return ret;
		}
		else {
			pr_debug("LCM: lcm disable lcm_vio\n");
		}

		/* verify */
		isenable = regulator_is_enabled(lcm_vio);
		if (!isenable)
			pr_debug("LCM: lcm_vio regulator disable pass\n");
		else
			pr_err("LCM: lcm_vio regulator disable fail\n");
	}

	return ret;
}

static bool is_iliteck_lcm_ic(void)
{
	get_lcm_id();
	/*abc123 ILITECK lcm vendor id is 3, 6, 8, 9*/
	if ((vendor_id == ILI9881C_BOE)||(vendor_id == ILI9881C_INX)||(vendor_id == ILI9881C_KD_HSD_ONYX)||(vendor_id == ILI9881C_KD_INX_O)||((project_id == PROJECT_ID) && (vendor_id == ILI9881C_KD_HSD))) {
		pr_debug("LCM: vendor_id is 0x%x\n",vendor_id);
		return true;
	} else {
		pr_err("LCM: there is no suitable vendor id for iliteck found\n");
		return false;
	}
}

static int lcm_driver_probe(struct device *dev, void const *data)
{
	pr_debug("LCM: lcm_driver_probe \n");

	lcm_get_vio_supply(dev);

	if (is_iliteck_lcm_ic()) {
		lcm_vio_supply_enable();
	}

	lcm_get_gpio_infor(dev);

	return 0;
}
static const struct of_device_id lcm_platform_of_match[] = {
	{
		.compatible = "lcm,lcm_dts_ili9881c",
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



static struct platform_driver lcm_driver = {
	.probe = lcm_platform_probe,
	.driver = {
			.name = "lcm_dts_ili9881c",
			.owner = THIS_MODULE,
			.of_match_table = lcm_platform_of_match,
			},
};

static int __init lcm_init(void)
{
	if (platform_driver_register(&lcm_driver)) {
		pr_err("LCM: failed to register disp driver\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit lcm_exit(void)
{
	platform_driver_unregister(&lcm_driver);
	pr_err("LCM: Unregister lcm driver done\n");
}
late_initcall(lcm_init);
module_exit(lcm_exit);
MODULE_AUTHOR("mediatek");
MODULE_DESCRIPTION("Display subsystem Driver");
MODULE_LICENSE("GPL");

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_st_kd_hsd_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;
	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	/* SYNC_PULSE_VDO_MODE;BURST_VDO_MODE;SYNC_EVENT_VDO_MODE; */
	params->dsi.mode		     = BURST_VDO_MODE;
	/* Command mode setting */
	params->dsi.LANE_NUM		     = LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order  = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq    = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding	     = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format	     = LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;
	// Video mode setting
	params->dsi.intermediat_buffer_num   = 0;
	params->dsi.PS			     = LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.word_count		     = FRAME_WIDTH*3;

	/* lcm optimize:reset voltage during print screen */
	params->dsi.cont_clock		     = 0;
	params->dsi.clk_lp_per_line_enable   = 1;

	params->dsi.vertical_sync_active     = 4;
	params->dsi.vertical_backporch	     = 18;
	params->dsi.vertical_frontporch	     = 18;
	params->dsi.vertical_active_line     = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active   = 10;
	params->dsi.horizontal_backporch     = 46;
	params->dsi.horizontal_frontporch    = 65;
	params->dsi.horizontal_active_pixel  = FRAME_WIDTH;

	/* PLL CLOCK 1314 X 889 X 24 X 60 % 4 X 1.15 % 2 = 241 */
	params->dsi.PLL_CLOCK	 = 239;
	params->dsi.ssc_disable	 = 0; /* 1:disable ssc , 0:enable ssc */
	params->dsi.HS_TRAIL = 5;
	params->dsi.CLK_HS_PRPR = 4;
}

static void lcm_get_st_boe_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;
	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	/* SYNC_PULSE_VDO_MODE;BURST_VDO_MODE;SYNC_EVENT_VDO_MODE; */
	params->dsi.mode		     = BURST_VDO_MODE;
	/* Command mode setting */
	params->dsi.LANE_NUM		     = LCM_FOUR_LANE;
	/* The following defined the fomat for data coming from LCD engine. */
	params->dsi.data_format.color_order  = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq    = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding	     = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format	     = LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;

	/* Video mode setting */
	params->dsi.intermediat_buffer_num   = 0;
	params->dsi.PS			     = LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.word_count		     = FRAME_WIDTH*3;

	/* lcm optimize:reset voltage during print screen */
	params->dsi.cont_clock		     = 0;
	params->dsi.clk_lp_per_line_enable   = 1;

	params->dsi.vertical_sync_active     = 8;
	params->dsi.vertical_backporch	     = 24;
	params->dsi.vertical_frontporch	     = 16;
	params->dsi.vertical_active_line     = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active   = 20;
	params->dsi.horizontal_backporch     = 80;
	params->dsi.horizontal_frontporch    = 100;
	params->dsi.horizontal_active_pixel  = FRAME_WIDTH;

	/* PLL_CLOCK: 1336 X 900 X 24 X 60 % 4 X 1.1 % 2 = 238 */
	params->dsi.PLL_CLOCK	 = 239;
	params->dsi.ssc_disable	 = 0; /* 1:disable ssc , 0:enable ssc */
	params->dsi.HS_TRAIL = 5;
	params->dsi.CLK_HS_PRPR = 4;
}

static void lcm_get_st_inx_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;
	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	/* SYNC_PULSE_VDO_MODE;BURST_VDO_MODE;SYNC_EVENT_VDO_MODE; */
	params->dsi.mode		     = BURST_VDO_MODE;
	/* Command mode setting */
	params->dsi.LANE_NUM		     = LCM_FOUR_LANE;
	/* The following defined the fomat for data coming from LCD engine. */
	params->dsi.data_format.color_order  = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq    = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding	     = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format	     = LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;

	/* Video mode setting */
	params->dsi.intermediat_buffer_num   = 0;
	params->dsi.PS			     = LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.word_count		     = FRAME_WIDTH*3;

	/* lcm optimize:reset voltage during print screen */
	params->dsi.cont_clock		     = 0;
	params->dsi.clk_lp_per_line_enable   = 1;

	params->dsi.vertical_sync_active     = 8;
	params->dsi.vertical_backporch	     = 24;
	params->dsi.vertical_frontporch	     = 16;
	params->dsi.vertical_active_line     = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active   = 20;
	params->dsi.horizontal_backporch     = 80;
	params->dsi.horizontal_frontporch    = 100;
	params->dsi.horizontal_active_pixel  = FRAME_WIDTH;

	/* PLL_CLOCK: 1336 X 900 X 24 X 60 % 4 X 1.1 % 2 = 238 */
	params->dsi.PLL_CLOCK	 = 239;
	params->dsi.ssc_disable	 = 0; /* 1:disable ssc , 0:enable ssc */
	params->dsi.HS_TRAIL = 5;
	params->dsi.CLK_HS_PRPR = 4;
}

static void lcm_get_st_kd_hsd_onyx_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;
	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	/* SYNC_PULSE_VDO_MODE;BURST_VDO_MODE;SYNC_EVENT_VDO_MODE; */
	params->dsi.mode		     = BURST_VDO_MODE;
	/* Command mode setting */
	params->dsi.LANE_NUM		     = LCM_FOUR_LANE;
	/* The following defined the fomat for data coming from LCD engine. */
	params->dsi.data_format.color_order  = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq    = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding	     = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format	     = LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;

	/* Video mode setting */
	params->dsi.intermediat_buffer_num   = 0;
	params->dsi.PS			     = LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.word_count		     = FRAME_WIDTH*3;

	/* lcm optimize:reset voltage during print screen */
	params->dsi.cont_clock		     = 0;
	params->dsi.clk_lp_per_line_enable   = 1;

	params->dsi.vertical_sync_active     = 4;
	params->dsi.vertical_backporch	     = 18;
	params->dsi.vertical_frontporch	     = 18;
	params->dsi.vertical_active_line     = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active   = 10;
	params->dsi.horizontal_backporch     = 46;
	params->dsi.horizontal_frontporch    = 65;
	params->dsi.horizontal_active_pixel  = FRAME_WIDTH;

	/* PLL_CLOCK: 1336 X 900 X 24 X 60 % 4 X 1.1 % 2 = 238 */
	params->dsi.PLL_CLOCK	 = 239;
	params->dsi.ssc_disable	 = 0; /* 1:disable ssc , 0:enable ssc */
	params->dsi.HS_TRAIL = 5;
	params->dsi.CLK_HS_PRPR = 4;
}

static void lcm_get_st_kd_inx_o_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;
	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	/* SYNC_PULSE_VDO_MODE;BURST_VDO_MODE;SYNC_EVENT_VDO_MODE; */
	params->dsi.mode		     = BURST_VDO_MODE;
	/* Command mode setting */
	params->dsi.LANE_NUM		     = LCM_FOUR_LANE;
	/* The following defined the fomat for data coming from LCD engine. */
	params->dsi.data_format.color_order  = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq    = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding	     = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format	     = LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;

	/* Video mode setting */
	params->dsi.intermediat_buffer_num   = 0;
	params->dsi.PS			     = LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.word_count		     = FRAME_WIDTH*3;

	/* lcm optimize:reset voltage during print screen */
	params->dsi.cont_clock		     = 0;
	params->dsi.clk_lp_per_line_enable   = 1;

	params->dsi.vertical_sync_active     = 8;
	params->dsi.vertical_backporch	     = 24;
	params->dsi.vertical_frontporch	     = 16;
	params->dsi.vertical_active_line     = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active   = 20;
	params->dsi.horizontal_backporch     = 80;
	params->dsi.horizontal_frontporch    = 80;
	params->dsi.horizontal_active_pixel  = FRAME_WIDTH;

	/* PLL_CLOCK: 1336 X 900 X 24 X 60 % 4 X 1.1 % 2 = 238 */
	params->dsi.PLL_CLOCK	 = 239;
	params->dsi.ssc_disable	 = 0; /* 1:disable ssc , 0:enable ssc */
	params->dsi.HS_TRAIL = 5;
	params->dsi.CLK_HS_PRPR = 4;
}

static void lcm_reset(void)
{
	pr_info("[KERNEL/LCM] %s, - Reset\n", __func__);

	lcm_set_gpio_output(GPIO_LCD_RST_EN,GPIO_OUT_HIGH);
	UDELAY(50);
	lcm_set_gpio_output(GPIO_LCD_RST_EN,GPIO_OUT_LOW);
	UDELAY(50);
	lcm_set_gpio_output(GPIO_LCD_RST_EN,GPIO_OUT_HIGH);
	MDELAY(20);
}

static char *lcm_get_vendor_type(void)
{
	if ((project_id == PROJECT_ID) && (vendor_id == ILI9881C_KD_HSD))
		return "ILI9881C_KD_HSD\0";
	else if (vendor_id == ILI9881C_BOE)
		return "ILI9881C_BOE\0";
	else if (vendor_id == ILI9881C_INX)
		return "ILI9881C_INX\0";
	else if (vendor_id == ILI9881C_KD_HSD_ONYX)
		return "ILI9881C_HD_HSD_ONYX\0";
	else if (vendor_id == ILI9881C_KD_INX_O)
		return "ILI9881C_KD_INX_O\0";
	else
		return "UNKNOWN\0";
}

static void init_ili9881c_kd_hsd_lcm(void)
{
	unsigned int data_array[64];

	data_array[0] = 0x00043902;
	data_array[1] = 0x038198FF;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00011500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00021500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x73031500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00041500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00051500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0A061500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00071500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00081500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x20091500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x200A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x000B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x000C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x000D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x000E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1E0F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1E101500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00111500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00121500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00131500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00141500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x10151500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x10161500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x03171500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x03181500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00191500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x001A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x001B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x001C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x001D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x401E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x801F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x06201500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x01211500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00221500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00231500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00241500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00251500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00261500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00271500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x33281500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x03291500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x002A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x002B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x002C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x002D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x002E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x002F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00301500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00311500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00321500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00331500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x04341500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00351500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00361500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00371500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x3C381500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00391500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x003A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x003B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x003C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x003D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x003E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x003F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00401500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00411500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00421500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00431500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00441500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x10501500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x32511500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x54521500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x76531500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x98541500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xBA551500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x10561500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x32571500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x54581500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x76591500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x985A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xBA5B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xDC5C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xFE5D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x005E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x015F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00601500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x15611500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x14621500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0E631500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0F641500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0C651500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0D661500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x06671500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02681500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02691500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x026A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x026B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x026C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x026D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x076E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x026F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02701500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02711500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02721500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02731500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02741500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x01751500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00761500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x14771500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x15781500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0E791500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0F7A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0C7B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0D7C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x067D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x027E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x027F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02801500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02811500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02821500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02831500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x07841500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02851500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02861500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02871500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02881500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02891500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x028A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00043902;
	data_array[1] = 0x048198FF;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x156C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x2A6E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x356F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x243A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F8D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xBA871500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x76261500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xD1B21500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x27B51500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x75311500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x03301500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x983B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F351500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x14331500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0F7A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02381500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00391500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00043902;
	data_array[1] = 0x018198FF;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x0A221500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00311500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x46531500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xC7501500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xC2511500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x26601500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00631500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x08A01500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x13A11500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x2AA21500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x04A31500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x25A41500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1BA51500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x12A61500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1CA71500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x8EA81500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x19A91500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x26AA1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x87AB1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x24AC1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1EAD1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x5BAE1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x29AF1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x2BB01500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x52B11500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x5DB21500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x39B31500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x08C01500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x23C11500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x2AC21500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x20C31500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x07C41500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x34C51500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x26C61500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x21C71500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x9EC81500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x20C91500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x2CCA1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x8ECB1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1ACC1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x23CD1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x4DCE1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x27CF1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x2DD01500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x52D11500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x5DD21500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x39D31500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00043902;
	data_array[1] = 0x008198FF;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00110500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);

	data_array[0] = 0x00290500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(20);
}

static void init_ili9881c_boe_lcm(void)
{
	unsigned int data_array[64];

	data_array[0] = 0x00043902;
	data_array[1] = 0x038198FF;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00011500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00021500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x57031500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xD3041500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00051500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x11061500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x08071500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00081500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00091500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x3F0A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x000B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x000C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x000D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x000E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x3F0F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x3F101500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00111500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00121500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00131500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00141500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00151500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00161500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00171500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00181500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00191500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x001A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x001B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x001C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x001D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x401E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x801F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x06201500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x01211500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00221500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00231500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00241500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00251500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00261500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00271500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x33281500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x33291500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x002A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x002B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x002C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x002D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x002E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x002F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00301500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00311500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00321500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00331500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00341500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00351500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00361500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00371500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x78381500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00391500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x003A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x003B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x003C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x003D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x003E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x003F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00401500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00411500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00421500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00431500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00441500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00501500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x23511500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x45521500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x67531500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x89541500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xAB551500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x01561500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x23571500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x45581500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x67591500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x895A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xAB5B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xCD5C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xEF5D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x005E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0D5F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0D601500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0C611500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0C621500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0F631500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0F641500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0E651500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0E661500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x08671500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02681500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02691500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x026A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x026B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x026C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x026D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x026E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x026F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x14701500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x15711500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x06721500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02731500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02741500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0D751500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0D761500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0C771500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0C781500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0F791500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0F7A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0E7B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1E7C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x087D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x027E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x027F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02801500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02811500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02821500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02831500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02841500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02851500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x14861500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x15871500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x06881500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02891500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x028A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00043902;
	data_array[1] = 0x048198FF;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x3B6E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x576F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x243A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F8D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xBA871500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xD1B21500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0B881500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x01381500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00391500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x07B51500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x75311500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x983B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00043902;
	data_array[1] = 0x018198FF;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x0A221500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x09311500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x07351500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x86501500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x82511500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x10601500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00621500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00A01500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x12A11500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1FA21500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x12A31500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x16A41500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x29A51500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1EA61500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1FA71500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x7EA81500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1BA91500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x28AA1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x6DAB1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x19AC1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x18AD1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x4CAE1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1EAF1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x23B01500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x52B11500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x6DB21500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x3FB31500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00C01500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x12C11500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x20C21500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x10C31500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x13C41500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x27C51500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1BC61500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1DC71500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x75C81500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1FC91500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x28CA1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x68CB1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1ACC1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x18CD1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x4DCE1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x25CF1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x2ED01500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x53D11500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x60D21500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x3FD31500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00043902;
	data_array[1] = 0x008198FF;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00351500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00110500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);

	data_array[0] = 0x00290500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(20);
}

static void init_ili9881c_inx_lcm(void)
{
	unsigned int data_array[64];

	data_array[0] = 0x00043902;
	data_array[1] = 0x038198FF;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00011500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00021500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x53031500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x53041500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x13051500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x04061500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02071500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02081500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00091500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x000A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x000B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x000C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x000D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x000E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x000F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00101500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00111500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00121500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00131500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00141500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00151500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00161500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00171500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00181500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00191500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x001A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x001B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x001C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x001D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xC01E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x801F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02201500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x09211500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00221500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00231500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00241500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00251500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00261500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00271500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x55281500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x03291500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x002A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x002B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x002C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x002D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x002E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x002F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00301500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00311500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00321500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00331500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00341500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00351500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00361500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00371500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x3C381500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00391500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x003A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x003B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x003C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x003D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x003E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x003F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00401500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00411500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00421500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00431500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00441500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x01501500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x23511500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x45521500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x67531500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x89541500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xAB551500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x01561500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x23571500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x45581500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x67591500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x895A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xAB5B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xCD5C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xEF5D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x015E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x085F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02601500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02611500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0A621500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x15631500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x14641500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02651500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x11661500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x10671500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02681500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0F691500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0E6A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x026B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0D6C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0C6D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x066E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x026F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02701500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02711500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02721500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02731500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02741500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x06751500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02761500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02771500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0A781500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x15791500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x147A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x027B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x107C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x117D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x027E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0C7F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0D801500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02811500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0E821500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0F831500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x08841500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02851500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02861500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02871500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02881500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02891500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x028A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00043902;
	data_array[1] = 0x048198FF;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x156C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x306E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x376F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1F8D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xBA871500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x76261500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xD1B21500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x07B51500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x17351500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x14331500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x75311500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x853A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x983B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x01381500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00391500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x107A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00043902;
	data_array[1] = 0x018198FF;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x0A221500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00311500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xE9501500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xE4511500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x28601500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xC82E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x01341500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00A01500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x11A11500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1FA21500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x14A31500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x18A41500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x2DA51500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x21A61500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x21A71500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x7DA81500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1BA91500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x25AA1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x69AB1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1BAC1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1AAD1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x50AE1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x24AF1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x29B01500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x4DB11500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x5AB21500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x23B31500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00C01500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x14C11500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x22C21500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x14C31500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x17C41500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x2AC51500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1FC61500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x20C71500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x7DC81500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1CC91500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x28CA1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x68CB1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1DCC1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1DCD1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x52CE1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x27CF1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x2DD01500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x4CD11500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x59D21500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x23D31500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00043902;
	data_array[1] = 0x008198FF;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00351500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00110500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);

	data_array[0] = 0x00290500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(20);
}

static void init_ili9881c_kd_hsd_onyx_lcm(void)
{
	unsigned int data_array[64];

	data_array[0] = 0x00043902;
	data_array[1] = 0x038198FF;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00011500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00021500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x53031500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x13041500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00051500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x04061500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00071500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00081500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x20091500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x200A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x000B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x000C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x000D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x000E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1E0F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1E101500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00111500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00121500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00131500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00141500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x10151500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x10161500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x03171500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x03181500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00191500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x001A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x001B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x001C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x001D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x441E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x801F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02201500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x03211500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00221500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00231500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00241500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00251500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00261500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00271500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x33281500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x03291500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x002A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x002B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x002C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x002D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x002E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x002F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00301500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00311500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00321500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00331500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x04341500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00351500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00361500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00371500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x3C381500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00391500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x003A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x003B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x003C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x003D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x003E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x003F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00401500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00411500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00421500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00431500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00441500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x01501500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x23511500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x45521500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x67531500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x89541500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xAB551500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x01561500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x23571500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x45581500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x67591500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x895A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xAB5B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xCD5C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xEF5D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x115E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x015F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00601500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x15611500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x14621500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0C631500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0D641500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0E651500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0F661500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x06671500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02681500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02691500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x026A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x026B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x026C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x026D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x086E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x026F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02701500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02711500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02721500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02731500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02741500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x01751500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00761500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x14771500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x15781500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0C791500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0D7A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0E7B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0F7C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x087D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x027E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x027F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02801500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02811500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02821500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02831500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x06841500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02851500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02861500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02871500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02881500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02891500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x028A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00043902;
	data_array[1] = 0x048198FF;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x156C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x2A6E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x336F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x243A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x148D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xBA871500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x76261500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xD1B21500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x27B51500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x75311500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x03301500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x983B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1f351500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x14331500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0F7A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02381500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00391500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00043902;
	data_array[1] = 0x018198FF;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x0A221500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00311500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x3E531500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xE9501500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xE5511500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x19601500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00631500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x08A01500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x10A11500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x26A21500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x03A31500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x25A41500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1BA51500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x13A61500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1CA71500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x83A81500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x19A91500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x24AA1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x79AB1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x23AC1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1EAD1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x5CAE1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x28AF1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x29B01500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x56B11500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x63B21500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x39B31500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x08C01500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x20C11500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x26C21500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x20C31500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x06C41500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x35C51500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x27C61500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x22C71500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x92C81500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x20C91500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x2BCA1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x81CB1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1ACC1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x22CD1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x4ECE1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x26CF1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x2DD01500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x56D11500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x63D21500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x39D31500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00043902;
	data_array[1] = 0x008198FF;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00110500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);

	data_array[0] = 0x00290500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(20);
}

static void init_ili9881c_kd_inx_o_lcm(void)
{
	unsigned int data_array[64];

	data_array[0] = 0x00043902;
	data_array[1] = 0x038198FF;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00011500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00021500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x53031500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x53041500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x13051500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x04061500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02071500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02081500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00091500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x000A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x000B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x000C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x000D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x000E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x000F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00101500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00111500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00121500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00131500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00141500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00151500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00161500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00171500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00181500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00191500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x001A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x001B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x001C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x001D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xC01E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x801F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02201500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x09211500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00221500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00231500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00241500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00251500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00261500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00271500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x55281500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x03291500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x002A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x002B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x002C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x002D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x002E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x002F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00301500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00311500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00321500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00331500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00341500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00351500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00361500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00371500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x3C381500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00391500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x003A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x003B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x003C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x003D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x003E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x003F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00401500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00411500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00421500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00431500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00441500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x01501500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x23511500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x45521500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x67531500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x89541500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xAB551500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x01561500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x23571500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x45581500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x67591500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x895A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xAB5B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xCD5C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xEF5D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x015E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x085F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02601500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02611500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0A621500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x15631500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x14641500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02651500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x11661500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x10671500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02681500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0F691500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0E6A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x026B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0D6C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0C6D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x066E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x026F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02701500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02711500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02721500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02731500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02741500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x06751500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02761500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02771500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0A781500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x15791500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x147A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x027B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x107C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x117D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x027E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0C7F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0D801500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02811500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0E821500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0F831500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x08841500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02851500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02861500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02871500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02881500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x02891500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x028A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00043902;
	data_array[1] = 0x048198FF;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x156C1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x2E6E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x356F1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1E8D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xBA871500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x76261500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xD1B21500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x07B51500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x17351500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x14331500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x75311500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x853A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x983B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x01381500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00391500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x107A1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00043902;
	data_array[1] = 0x018198FF;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x0A221500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00311500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xCF501500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xCA511500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x3D531500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x48551500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x28601500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0xC82E1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x34011500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00A01500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0CA11500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1AA21500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x12A31500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x17A41500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x2CA51500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1FA61500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1FA71500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x6FA81500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1AA91500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x25AA1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x5DAB1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1AAC1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1BAD1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x4EAE1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x21AF1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x25B01500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x4DB11500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x62B21500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x3FB31500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00C01500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x13C11500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1FC21500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x14C31500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x15C41500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x28C51500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1FC61500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x21C71500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x70C81500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1CC91500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x28CA1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x5ECB1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1CCC1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x1DCD1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x53CE1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x28CF1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x2FD01500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x4CD11500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x5AD21500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x3FD31500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00043902;
	data_array[1] = 0x008198FF;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00110500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);

	data_array[0] = 0x00290500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(20);
}

static void lcm_init_lcm(void)
{
	get_lcm_id();

	if ((project_id == PROJECT_ID) && (vendor_id == ILI9881C_KD_HSD))
		init_ili9881c_kd_hsd_lcm();	/* ILI9881C KD panel, HSD glass */
	else if (vendor_id == ILI9881C_BOE)
		init_ili9881c_boe_lcm();	/*ILI9881C BOE panel */
	else if (vendor_id == ILI9881C_INX)
		init_ili9881c_inx_lcm();	/*ILI9881C INX panel */
	else if (vendor_id == ILI9881C_KD_HSD_ONYX)
		init_ili9881c_kd_hsd_onyx_lcm();	/*ILI9881C KD panel, HSDonyx glass */
	else
		init_ili9881c_kd_hsd_lcm();	    /* Default ILI9881C KD_HSD panel */

	pr_debug("[ILI9881C_lcm]lcm_init_lcm func:Kernel ili9881c %s lcm init ok!\n",lcm_get_vendor_type());
}

static void lcm_suspend(void)
{
	unsigned int data_array[64];

	data_array[0] = 0x00280500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(20);

	data_array[0] = 0x00100500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);
}

static void lcm_suspend_power(void)
{
	pr_debug("LCM: lcm try to turn off supply\n");

	lcm_set_gpio_output(GPIO_LCD_RST_EN,GPIO_OUT_LOW);
	MDELAY(50);
	lcm_set_gpio_output(GPIO_LCD_PWR_N_EN,GPIO_OUT_LOW);
	MDELAY(10);
	lcm_set_gpio_output(GPIO_LCD_PWR_P_EN,GPIO_OUT_LOW);
	MDELAY(10);

	if (lcm_vio_supply_disable())
		pr_err(KERN_ERR "<%s>JD936X lcm(lcm_vio) failed to turn off supply", __func__);
}

static void lcm_resume_power(void)
{
	pr_debug("LCM: lcm try to turn on supply\n");

	if(lcm_vio_supply_enable())
		pr_err(KERN_ERR "<%s>JD936X lcm(lcm_vio) failed to turn on supply", __func__);
	MDELAY(10);
	lcm_set_gpio_output(GPIO_LCD_PWR_P_EN,GPIO_OUT_HIGH);
	MDELAY(10);
	lcm_set_gpio_output(GPIO_LCD_PWR_N_EN,GPIO_OUT_HIGH);
	MDELAY(10);
}

static void lcm_resume(void)
{
	get_lcm_id();

	MDELAY(10);
	lcm_reset();

	if ((project_id == PROJECT_ID) && (vendor_id == ILI9881C_KD_HSD))
		init_ili9881c_kd_hsd_lcm();	/* ILI9881C KD panel, HSD glass */
	else if (vendor_id == ILI9881C_BOE)
		init_ili9881c_boe_lcm();	/*ILI9881C BOE panel */
	else if (vendor_id == ILI9881C_INX)
		init_ili9881c_inx_lcm();	/*ILI9881C INX panel */
	else if (vendor_id == ILI9881C_KD_HSD_ONYX)
		init_ili9881c_kd_hsd_onyx_lcm();	/*ILI9881C KD panel, HSDonyx glass */
	else if (vendor_id == ILI9881C_KD_INX_O)
		init_ili9881c_kd_inx_o_lcm();	/*ILI9881C KD INX */
	else
		init_ili9881c_kd_hsd_lcm();	    /* Default ILI9881C KD_HSD panel */
}

static void lcm_set_backlight(unsigned int level)
{
	backlight_brightness_set(level);
}


LCM_DRIVER ili9881c_wxga_dsi_vdo_st_kd_hsd_lcm_drv = {
	.name		 = "ili9881c_wxga_dsi_vdo_st_kd_hsd",
	.set_util_funcs	 = lcm_set_util_funcs,
	.get_params	 = lcm_get_st_kd_hsd_params,
	.init		 = lcm_init_lcm,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.resume_power	= lcm_resume_power,
	.suspend_power	= lcm_suspend_power,
	.set_backlight  = lcm_set_backlight,
};

LCM_DRIVER ili9881c_wxga_dsi_vdo_st_boe_lcm_drv = {
	.name		 = "ili9881c_wxga_dsi_vdo_st_boe",
	.set_util_funcs	 = lcm_set_util_funcs,
	.get_params	 = lcm_get_st_boe_params,
	.init		 = lcm_init_lcm,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.resume_power	= lcm_resume_power,
	.suspend_power	= lcm_suspend_power,
	.set_backlight  = lcm_set_backlight,
};

LCM_DRIVER ili9881c_wxga_dsi_vdo_st_inx_lcm_drv = {
	.name		 = "ili9881c_wxga_dsi_vdo_st_inx",
	.set_util_funcs	 = lcm_set_util_funcs,
	.get_params	 = lcm_get_st_inx_params,
	.init		 = lcm_init_lcm,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.resume_power	= lcm_resume_power,
	.suspend_power	= lcm_suspend_power,
	.set_backlight  = lcm_set_backlight,
};

LCM_DRIVER ili9881c_wxga_dsi_vdo_st_kd_hsd_onyx_lcm_drv = {
	.name		 = "ili9881c_wxga_dsi_vdo_st_kd_hsd_onyx",
	.set_util_funcs	 = lcm_set_util_funcs,
	.get_params	 = lcm_get_st_kd_hsd_onyx_params,
	.init		 = lcm_init_lcm,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.resume_power	= lcm_resume_power,
	.suspend_power	= lcm_suspend_power,
	.set_backlight  = lcm_set_backlight,
};

LCM_DRIVER ili9881c_wxga_dsi_vdo_st_kd_inx_o_lcm_drv = {
	.name		 = "ili9881c_wxga_dsi_vdo_st_kd_inx_o",
	.set_util_funcs	 = lcm_set_util_funcs,
	.get_params	 = lcm_get_st_kd_inx_o_params,
	.init		 = lcm_init_lcm,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.resume_power	= lcm_resume_power,
	.suspend_power	= lcm_suspend_power,
	.set_backlight  = lcm_set_backlight,
};
