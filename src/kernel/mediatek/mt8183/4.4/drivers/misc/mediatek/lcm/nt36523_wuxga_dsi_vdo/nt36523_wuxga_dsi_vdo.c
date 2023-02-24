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

#define LOG_TAG "LCM-NT36523"

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

#define VID_NT_TG_BOE   0x6		/* TG, using NT36523 IC */

#define VID_NULL        0xff	/* NULL */
#define RSVD_CODE       0x81	/* NULL */

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

#define LCM_Tdb 4       /* display bias ready */
#define LCM_TSVSP  2       /* spec: > 1ms */
#define LCM_TPON1  0       /* spec: > 0ms */
#define LCM_TPON2  0       /* spec: > 0ms */
#define LCM_TRW    12      /* spec: > 10ms */
#define LCM_TRT2   0      /* spec: > 0ms */
#define LCM_TRT2BRST   12      /* spec: > 10ms */
#define LCM_TRT1   12      /* spec: > 10ms */
#define LCM_TSLPOUT   110      /* spec: > 100ms */
#define LCM_TBLON   45     /* spec: > 40ms */
#define LCM_TPD2SOFF   22     /* spec: > 20ms */
#define LCM_TSOFF1   0     /* spec: > 0ms */
#define LCM_TBLOFf   0     /* spec: > 0ms */
#define LCM_TSLPIN   66     /* spec: > 60ms */
#define LCM_TSOFF2   0     /* spec: > 0ms */
#define LCM_TROFF   0     /* spec: > 0ms */
#define LCM_TPOFF2   0     /* spec: > 0ms */
#define LCM_TPOFF1   0     /* spec: > 0ms */
#define LCM_THVSP   0     /* spec: > 0ms */
#define LCM_TVDDOFF   90     /* spec: > 80ms */

#define LCM_ID1_VENDOR_SHIFT    5
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

static struct LCM_setting_table sleep_out_setting[] = {
	{0x11, 0, {} },
	{REGFLAG_DELAY, LCM_TSLPOUT, {} },
	{0x29, 0, {} }
};

static struct LCM_setting_table sleep_in_setting[] = {
	{0x28, 0, {} },
	{0x10, 0, {} }
};

static struct LCM_setting_table register_lock[] = {
	{0xF0, 1, {0x55} },
	{0xF1, 1, {0xAA} },
	{0xF2, 1, {0x66} }
};

static struct LCM_setting_table register_unlock[] = {
	{0xF0, 1, {0xAA} },
	{0xF1, 1, {0x55} },
	{0xF2, 1, {0x99} }
};

/* 2020/11/24 release v2 CABC=50%*/
static struct LCM_setting_table init_setting[] = {
	{0xFF, 1, {0x20} },
	{0xFB, 1, {0x01} },
	{0x05, 1, {0xD1} },
	/* VGH=19V */
	{0x07, 1, {0x8C} },
	/* VGL=-12V */
	{0x08, 1, {0x46} },
	/* EN_VMODGATE2=1 */
	{0x0D, 1, {0x63} },
	/* VGHO=18.7V */
	{0x0E, 1, {0xAC} },
	/* VGLO=-11V */
	{0x0F, 1, {0x5F} },
	/* VCOM=-0.9V */
	/* GVDD=5.4V */
	{0x95, 1, {0xFF} },
	{0x96, 1, {0xFF} },
	/* Disable VDDI LV */
	{0x30, 1, {0x11} },
	{0x58, 1, {0x60} },
	/* ISOP */
	{0x6D, 1, {0x66} },
	/* EN_GMACP */
	{0x75, 1, {0xA2} },
	/* V128 */
	{0x77, 1, {0xB3} },
	/* R(+) */
	{0xB0, 16, {0x00, 0xB6, 0x00, 0xBF, 0x00, 0xD3, 0x00, 0xE4, 0x00, 0xF2, 0x01, 0x01, 0x01, 0x0D, 0x01, 0x1A} },
	{0xB1, 16, {0x01, 0x27, 0x01, 0x4F, 0x01, 0x71, 0x01, 0xA6, 0x01, 0xD3, 0x02, 0x19, 0x02, 0x52, 0x02, 0x54} },
	{0xB2, 16, {0x02, 0x8D, 0x02, 0xCF, 0x02, 0xF7, 0x03, 0x2C, 0x03, 0x4D, 0x03, 0x79, 0x03, 0x86, 0x03, 0x95} },
	{0xB3, 12, {0x03, 0xA4, 0x03, 0xB4, 0x03, 0xD5, 0x03, 0xED, 0x03, 0xFE, 0x03, 0xFF} },
	/* G(+) */
	{0xB4, 16, {0x00, 0x16, 0x00, 0x30, 0x00, 0x57, 0x00, 0x77, 0x00, 0x93, 0x00, 0xAB, 0x00, 0xC1, 0x00, 0xD4} },
	{0xB5, 16, {0x00, 0xE5, 0x01, 0x1F, 0x01, 0x4B, 0x01, 0x8E, 0x01, 0xC0, 0x02, 0x0D, 0x02, 0x4B, 0x02, 0x4D} },
	{0xB6, 16, {0x02, 0x88, 0x02, 0xCB, 0x02, 0xF2, 0x03, 0x27, 0x03, 0x4A, 0x03, 0x74, 0x03, 0x83, 0x03, 0x91} },
	{0xB7, 12, {0x03, 0xA1, 0x03, 0xB2, 0x03, 0xD2, 0x03, 0xEC, 0x03, 0xFE, 0x03, 0xFF} },
	/* B(+) */
	{0xB8, 16, {0x00, 0x08, 0x00, 0x2C, 0x00, 0x57, 0x00, 0x7A, 0x00, 0x99, 0x00, 0xB1, 0x00, 0xC7, 0x00, 0xDC} },
	{0xB9, 16, {0x00, 0xEE, 0x01, 0x28, 0x01, 0x54, 0x01, 0x96, 0x01, 0xC7, 0x02, 0x13, 0x02, 0x4F, 0x02, 0x51} },
	{0xBA, 16, {0x02, 0x8B, 0x02, 0xCD, 0x02, 0xF4, 0x03, 0x2A, 0x03, 0x4D, 0x03, 0x78, 0x03, 0x86, 0x03, 0x94} },
	{0xBB, 12, {0x03, 0xA4, 0x03, 0xB4, 0x03, 0xD5, 0x03, 0xEC, 0x03, 0xFE, 0x03, 0xFF} },
	/* CMD2_Page1 */
	{0xFF, 1, {0x21} },
	{0xFB, 1, {0x01} },
	/* R(-) */
	{0xB0, 16, {0x00, 0xAE, 0x00, 0xB7, 0x00, 0xCB, 0x00, 0xDC, 0x00, 0xEA, 0x00, 0xF9, 0x01, 0x05, 0x01, 0x12} },
	{0xB1, 16, {0x01, 0x1F, 0x01, 0x47, 0x01, 0x69, 0x01, 0x9E, 0x01, 0xCB, 0x02, 0x11, 0x02, 0x4A, 0x02, 0x4C} },
	{0xB2, 16, {0x02, 0x85, 0x02, 0xC7, 0x02, 0xEF, 0x03, 0x24, 0x03, 0x45, 0x03, 0x71, 0x03, 0x7E, 0x03, 0x8D} },
	{0xB3, 12, {0x03, 0x9C, 0x03, 0xAC, 0x03, 0xCD, 0x03, 0xE5, 0x03, 0xF6, 0x03, 0xF7} },
	/* G(-) */
	{0xB4, 16, {0x00, 0x0E, 0x00, 0x28, 0x00, 0x4F, 0x00, 0x6F, 0x00, 0x8B, 0x00, 0xA3, 0x00, 0xB9, 0x00, 0xCC} },
	{0xB5, 16, {0x00, 0xDD, 0x01, 0x17, 0x01, 0x43, 0x01, 0x86, 0x01, 0xB8, 0x02, 0x05, 0x02, 0x43, 0x02, 0x45} },
	{0xB6, 16, {0x02, 0x80, 0x02, 0xC3, 0x02, 0xEA, 0x03, 0x1F, 0x03, 0x42, 0x03, 0x6C, 0x03, 0x7B, 0x03, 0x89} },
	{0xB7, 12, {0x03, 0x99, 0x03, 0xAA, 0x03, 0xCA, 0x03, 0xE4, 0x03, 0xF6, 0x03, 0xF7} },
	/* B(-) */
	{0xB8, 16, {0x00, 0x00, 0x00, 0x24, 0x00, 0x4F, 0x00, 0x72, 0x00, 0x91, 0x00, 0xA9, 0x00, 0xBF, 0x00, 0xD4} },
	{0xB9, 16, {0x00, 0xE6, 0x01, 0x20, 0x01, 0x4C, 0x01, 0x8E, 0x01, 0xBF, 0x02, 0x0B, 0x02, 0x47, 0x02, 0x49} },
	{0xBA, 16, {0x02, 0x83, 0x02, 0xC5, 0x02, 0xEC, 0x03, 0x22, 0x03, 0x45, 0x03, 0x70, 0x03, 0x7E, 0x03, 0x8C} },
	{0xBB, 12, {0x03, 0x9C, 0x03, 0xAC, 0x03, 0xCD, 0x03, 0xE4, 0x03, 0xF6, 0x03, 0xF7} },
	{0xFF, 1, {0x24} },
	{0xFB, 1, {0x01} },
	/* VGL */
	{0x00, 1, {0x00} },
	/* VGL */
	{0x01, 1, {0x00} },
	/* VDS */
	{0x02, 1, {0x22} },
	/* STV3 */
	{0x03, 1, {0x08} },
	/* STV4 */
	{0x04, 1, {0x04} },
	/* VSD */
	{0x05, 1, {0x23} },
	/* GCL */
	{0x06, 1, {0x21} },
	/* GCH */
	{0x07, 1, {0x20} },
	/* CLK8 */
	{0x08, 1, {0x0F} },
	{0x09, 1, {0x0F} },
	/* CLK6 */
	{0x0A, 1, {0x0E} },
	{0x0B, 1, {0x0E} },
	/* CLK4 */
	{0x0C, 1, {0x0D} },
	{0x0D, 1, {0x0D} },
	/* CLK2 */
	{0x0E, 1, {0x0C} },
	{0x0F, 1, {0x0C} },
	{0x10, 1, {0x00} },
	{0x11, 1, {0x00} },
	{0x12, 1, {0x00} },
	{0x13, 1, {0x00} },
	{0x14, 1, {0x00} },
	{0x15, 1, {0x00} },
	/* VGL */
	{0x16, 1, {0x00} },
	/* VGL */
	{0x17, 1, {0x00} },
	/* VDS */
	{0x18, 1, {0x22} },
	/* STV1 */
	{0x19, 1, {0x08} },
	/* STV2 */
	{0x1A, 1, {0x04} },
	/* VSD */
	{0x1B, 1, {0x23} },
	/* GCL */
	{0x1C, 1, {0x21} },
	/* GCH */
	{0x1D, 1, {0x20} },
	/* CLK7 */
	{0x1E, 1, {0x0F} },
	{0x1F, 1, {0x0F} },
	/* CLK5 */
	{0x20, 1, {0x0E} },
	{0x21, 1, {0x0E} },
	/* CLK3 */
	{0x22, 1, {0x0D} },
	{0x23, 1, {0x0D} },
	/* CLK1 */
	{0x24, 1, {0x0C} },
	{0x25, 1, {0x0C} },
	{0x26, 1, {0x00} },
	{0x27, 1, {0x00} },
	{0x28, 1, {0x00} },
	{0x29, 1, {0x00} },
	{0x2A, 1, {0x00} },
	{0x2B, 1, {0x00} },
	{0x2F, 1, {0x54} },
	{0x33, 1, {0x54} },
	{0x3A, 1, {0x9A} },
	{0x3B, 1, {0xA0} },
	{0x3D, 1, {0x91} },
	/* STV1, STV3 */
	{0x3F, 1, {0x08} },
	{0x43, 1, {0x08} },
	{0x4A, 1, {0x9A} },
	{0x4B, 1, {0xA0} },
	{0x4C, 1, {0x91} },
	/* GCK */
	{0x4D, 1, {0x43} },
	{0x4E, 1, {0x21} },
	{0x51, 1, {0x12} },
	{0x52, 1, {0x34} },
	{0x55, 2, {0x86, 0x07} },
	{0x56, 1, {0x04} },
	{0x58, 1, {0x21} },
	{0x59, 1, {0x30} },
	{0x5A, 1, {0xBA} },
	{0x5B, 1, {0x96} },
	{0x5E, 2, {0x00, 0x12} },
	{0x5F, 1, {0x00} },
	/* EN_LFD_SOURCE=0 */
	{0x65, 1, {0x82} },
	/* GCH,GCL */
	{0xAD, 1, {0x00} },
	{0xAE, 1, {0x01} },
	{0xAF, 1, {0x0C} },
	{0xB0, 1, {0x16} },
	{0xB1, 1, {0x0B} },
	{0xB2, 1, {0x15} },
	{0xB3, 1, {0x00} },
	/* VDS,VSD */
	{0xC4, 1, {0x20} },
	{0xB6, 12, {0x05, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x05, 0x00, 0x00} },
	/* Resolution (1200RGBx1920) */
	{0x60, 1, {0x96} },
	{0x61, 1, {0x80} },
	{0x63, 1, {0x70} },
	{0x92, 1, {0xDF} },
	{0x93, 1, {0x1A} },
	{0x94, 1, {0x0A} },
	/* SOG_HBP */
	{0xD7, 1, {0x55} },
	{0xDA, 1, {0x0A} },
	{0xDE, 1, {0x08} },
	/* Normal */
	{0xDB, 1, {0x05} },
	{0xDC, 1, {0xDF} },
	{0xDD, 1, {0x22} },
	/* Line N */
	{0xDF, 1, {0x05} },
	{0xE0, 1, {0xDF} },
	/* Line N+1 */
	{0xE1, 1, {0x05} },
	{0xE2, 1, {0xDF} },
	/* TP0 */
	{0xE3, 1, {0x05} },
	{0xE4, 1, {0xDF} },
	/* TP3 */
	{0xE5, 1, {0x05} },
	{0xE6, 1, {0xDF} },
	/* Gate EQ */
	/* Normal */
	{0x5C, 1, {0x88} },
	{0x5D, 1, {0x08} },
	/* TP3 */
	{0x8D, 1, {0x88} },
	{0x8E, 1, {0x08} },
	/* No Sync @ TP */
	{0xB5, 1, {0x90} },
	{0xFF, 1, {0x25} },
	{0xFB, 1, {0x01} },
	/* disable auto_vbp_vfp */
	{0x05, 1, {0x00} },
	/* ESD_DET_ERR_SEL */
	{0x19, 1, {0x07} },
	/* DP_N_GCK */
	{0x1F, 1, {0xBA} },
	{0x20, 1, {0x96} },
	/* DP_N_1_GCK */
	{0x26, 1, {0xBA} },
	{0x27, 1, {0x96} },
	/* TP0_GCK */
	{0x33, 1, {0xBA} },
	{0x34, 1, {0x96} },
	/* TP3 GCK/MUX=1 */
	{0x3F, 1, {0xE0} },
	/* TP3_GCK_START_LINE */
	{0x40, 1, {0x00} },
	/* TP3_STV */
	{0x44, 1, {0x00} },
	{0x45, 1, {0x40} },
	/* TP3_GCK */
	{0x48, 1, {0xBA} },
	{0x49, 1, {0x96} },
	/* LSTP0 */
	{0x5B, 1, {0x00} },
	{0x5C, 1, {0x00} },
	{0x5D, 1, {0x00} },
	{0x5E, 1, {0xD0} },
	{0x61, 1, {0xBA} },
	{0x62, 1, {0x96} },
	/* en_vfp_addvsync */
	{0xF1, 1, {0x10} },
	/* CMD2,Page10 */
	{0xFF, 1, {0x2A} },
	{0xFB, 1, {0x01} },
	/* PWRONOFF */
	/* STV */
	{0x64, 1, {0x16} },
	/* CLR */
	{0x67, 1, {0x16} },
	/* GCK */
	{0x6A, 1, {0x16} },
	/* GCH */
	{0x73, 1, {0x9E} },
	/* GCL */
	{0x76, 1, {0x16} },
	/* VDS */
	{0x79, 1, {0x9E} },
	/* VSD */
	{0x7C, 1, {0x16} },
	/* ABOFF */
	{0xA2, 1, {0xF3} },
	{0xA3, 1, {0xFF} },
	{0xA4, 1, {0xFF} },
	{0xA5, 1, {0xFF} },
	/* Long_V_TIMING disable */
	{0xD6, 1, {0x08} },
	/* CMD2,Page6 */
	{0xFF, 1, {0x26} },
	{0xFB, 1, {0x01} },
	/* TPEN */
	{0x00, 1, {0xA1} },
	{0x0A, 1, {0xF2} },
	/* Table A (60Hz,106*17+118*1=1920) */
	{0x04, 1, {0x28} },
	{0x06, 1, {0x2A} },
	{0x0C, 1, {0x11} },
	{0x0D, 1, {0x00} },
	{0x0F, 1, {0x09} },
	{0x11, 1, {0x10} },
	{0x12, 1, {0x50} },
	{0x13, 1, {0x6A} },
	{0x14, 1, {0x76} },
	{0x15, 1, {0x00} },
	{0x16, 1, {0x90} },
	{0x17, 1, {0xA0} },
	{0x18, 1, {0x86} },
	{0x19, 1, {0x0F} },
	{0x1A, 1, {0x9C} },
	{0x1B, 1, {0x0E} },
	{0x1C, 1, {0xDC} },
	{0x22, 1, {0x00} },
	{0x23, 1, {0x00} },
	{0x2A, 1, {0x0F} },
	{0x2B, 1, {0x9C} },
	{0x1D, 1, {0x00} },
	{0x1E, 1, {0xDF} },
	{0x1F, 1, {0xDF} },
	{0x24, 1, {0x00} },
	{0x25, 1, {0xDF} },
	{0x2F, 1, {0x05} },
	{0x30, 1, {0xDF} },
	{0x31, 1, {0x00} },
	{0x32, 1, {0xDF} },
	{0x39, 1, {0x00} },
	{0x3A, 1, {0xDF} },
	/* PRZ1 */
	{0x20, 1, {0x01} },
	/* Rescan=3 */
	{0x33, 1, {0x11} },
	{0x34, 1, {0x78} },
	{0x35, 1, {0x16} },
	/* DLH  */
	{0xC8, 1, {0x00} },
	{0xC9, 1, {0x00} },
	{0xCA, 1, {0x4E} },
	{0xCB, 1, {0x00} },
	{0xA9, 1, {0x6D} },
	{0xAA, 1, {0x71} },
	{0xAB, 1, {0x73} },
	{0xAC, 1, {0x76} },
	/* CMD2,Page7 */
	{0xFF, 1, {0x27} },
	{0xFB, 1, {0x01} },
	{0x58, 1, {0x00} },
	{0x00, 1, {0x00} },
	{0x78, 1, {0x00} },
	{0xC3, 1, {0x00} },
	/* FTE output TE, LEDPWM output LEDPWM */
	{0xD1, 1, {0x24} },
	{0xD2, 1, {0x30} },
	/* FTE output TE, FTE1 output TSVD, LEDPWM output TSHD_NVT */
	/* FTE output TE, FTE1 output Internal H, LEDPWM output TSHD */
	/* CMD2,Page3 */
	{0xFF, 1, {0x23} },
	{0xFB, 1, {0x01} },
	/* DBV=12 bit */
	{0x00, 1, {0x80} },
	/* CABC Dimming */
	{0x05, 1, {0x2D} },
	/* PWM frequency */
	{0x07, 1, {0x00} },
	/* Resolution=1200x1920 */
	{0x11, 1, {0x01} },
	{0x12, 1, {0x68} },
	{0x15, 1, {0xE9} },
	{0x16, 1, {0x0C} },
	{0x2B, 1, {0x10} },
	/* MOV_PWM DUTY */
	{0x58, 1, {0xFF} },
	{0x59, 1, {0xFF} },
	{0x5A, 1, {0xFE} },
	{0x5B, 1, {0xFD} },
	{0x5C, 1, {0xFD} },
	{0x5D, 1, {0xFC} },
	{0x5E, 1, {0xFB} },
	{0x5F, 1, {0xF9} },
	{0x60, 1, {0xF6} },
	{0x61, 1, {0xEF} },
	{0x62, 1, {0xDF} },
	{0x63, 1, {0xCF} },
	{0x64, 1, {0xBF} },
	{0x65, 1, {0xAF} },
	{0x66, 1, {0x9F} },
	{0x67, 1, {0x7F} },
	/* GAMMA SEL */
	{0x19, 1, {0x00} },
	{0x1A, 1, {0x00} },
	{0x1B, 1, {0x08} },
	{0x1C, 1, {0x0A} },
	{0x1D, 1, {0x0C} },
	{0x1E, 1, {0x12} },
	{0x1F, 1, {0x16} },
	{0x20, 1, {0x1A} },
	{0x21, 1, {0x20} },
	{0x22, 1, {0x24} },
	{0x23, 1, {0x2A} },
	{0x24, 1, {0x30} },
	{0x25, 1, {0x34} },
	{0x26, 1, {0x3A} },
	{0x27, 1, {0x3C} },
	{0x28, 1, {0x3F} },
	/* CMD3,PageA */
	{0xFF, 1, {0xE0} },
	{0xFB, 1, {0x01} },
	/* VCOM Driving Ability */
	{0x14, 1, {0x60} },
	{0x16, 1, {0xC0} },
	/* CMD3,PageB */
	{0xFF, 1, {0xF0} },
	{0xFB, 1, {0x01} },
	/* slave osc workaround */
	{0x3A, 1, {0x08} },
	/* CMD1 */
	{0xFF, 1, {0x10} },
	{0xFB, 1, {0x01} },
	/* Only Write Slave */
	{0xB9, 1, {0x01} },
	/* CMD2,Page0 */
	{0xFF, 1, {0x20} },
	{0xFB, 1, {0x01} },
	{0x18, 1, {0x40} },
	/* CMD1 */
	{0xFF, 1, {0x10} },
	{0xFB, 1, {0x01} },
	/* Write Master & Slave */
	{0xB9, 1, {0x02} },
	{0x35, 1, {0x00} },
	{0x51, 2, {0x0F, 0xFF} },
	{0x53, 1, {0x2C} },
	/* 0X00: CABC OFF; 0X01: UI_mode; 0x02: Still-Mode; 0x03: Moving-Mode */
	{0x55, 1, {0x00} },
	{0xBB, 1, {0x13} },
	/* VBP+VFP=36 */
	{0x3B, 5, {0x03, 0x0A, 0x1A, 0x04, 0x04} },
	/* CMD2,Page5 */
	{0xFF, 1, {0x25} },
	/* FRM */
	{0xEC, 1, {0x00} },
	/* PAUSE */
	/* CMD1 */
	{0xFF, 1, {0x10} },
	{0xFB, 1, {0x01} }
};

#define	DSI_DCS_SHORT_PACKET_ID_0	0x05
#define	DSI_DCS_SHORT_PACKET_ID_1	0x15
#define	DSI_DCS_LONG_PACKET_ID		0x39
#define	DSI_DCS_READ_PACKET_ID		0x06

static struct LCM_setting_table_V4 cabc_on_setting[] = {
	{DSI_DCS_SHORT_PACKET_ID_1, 0xF0, 1, {0xAA}, 0 },
	{DSI_DCS_SHORT_PACKET_ID_1, 0xF1, 1, {0x55}, 0 },
	{DSI_DCS_SHORT_PACKET_ID_1, 0xF2, 1, {0x99}, 0 },
	{DSI_DCS_SHORT_PACKET_ID_1, 0x55, 1, {0x03}, 0 },
	{DSI_DCS_SHORT_PACKET_ID_1, 0xF0, 1, {0x55}, 0 },
	{DSI_DCS_SHORT_PACKET_ID_1, 0xF1, 1, {0xAA}, 0 },
	{DSI_DCS_SHORT_PACKET_ID_1, 0xF2, 1, {0x66}, 0 }
};

static struct LCM_setting_table_V4 cabc_off_setting[] = {
	{DSI_DCS_SHORT_PACKET_ID_1, 0xF0, 1, {0xAA}, 0 },
	{DSI_DCS_SHORT_PACKET_ID_1, 0xF1, 1, {0x55}, 0 },
	{DSI_DCS_SHORT_PACKET_ID_1, 0xF2, 1, {0x99}, 0 },
	{DSI_DCS_SHORT_PACKET_ID_1, 0xB9, 1, {0x00}, 0 },
	{DSI_DCS_SHORT_PACKET_ID_1, 0x55, 1, {0x00}, 0 },
	{DSI_DCS_SHORT_PACKET_ID_1, 0xB9, 1, {0x02}, 0 },
	{DSI_DCS_SHORT_PACKET_ID_1, 0xF0, 1, {0x55}, 0 },
	{DSI_DCS_SHORT_PACKET_ID_1, 0xF1, 1, {0xAA}, 0 },
	{DSI_DCS_SHORT_PACKET_ID_1, 0xF2, 1, {0x66}, 0 }
};

static DEFINE_MUTEX(nt36523_share_power_mutex);

static struct regulator *reg_lcm_vcc1v8;
static unsigned int LCM_RST_GPIO;
static unsigned int g_vendor_id = VID_NULL;
static int old_state = LCM_OFF;
static atomic_t g_power_request = ATOMIC_INIT(0);

static void lcm_power_manager(int);
static void lcm_power_manager_lock(int);
static unsigned int get_lcm_id(void);
static void lcm_set_gpio_output(unsigned int GPIO, unsigned int output);
static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update);

/*
* 1. export function
* 2. lcm_platform_driver
* 3. lcm_driver
*/

/* export function start */
int nt36523_share_request_powerpin_resource(struct device *dev)
{
	struct device_node *lcm_common_resource = NULL;
	int ret = 0;

	LCM_FUNC_ENTRY();
	mutex_lock(&nt36523_share_power_mutex);
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

	ret = display_bias_enable_vol(LCM_AVDD_VOL, LCM_AVEE_VOL, 0);
	if (ret != 0) {
		LCM_ERR("Failed to config display bias supply: %d", ret);
		goto err_disp_bias;
	}
	old_state = LCM_ON;

init_done:
	atomic_inc(&g_power_request);
	mutex_unlock(&nt36523_share_power_mutex);
	LCM_FUNC_EXIT("cnt(%d)", atomic_read(&g_power_request));
	return 0;

err_disp_bias:
	regulator_disable(reg_lcm_vcc1v8);
err_vcc1v8_2:
	regulator_put(reg_lcm_vcc1v8);
	reg_lcm_vcc1v8 = NULL;
err_vcc1v8_1:
	gpio_free(LCM_RST_GPIO);
err_lcm_rst:
err_dts_parsing:
	mutex_unlock(&nt36523_share_power_mutex);
	LCM_FUNC_EXIT("fail to probe, cnt(%d)", atomic_read(&g_power_request));
	return -1;
}
EXPORT_SYMBOL(nt36523_share_request_powerpin_resource);

void nt36523_share_powerpin_on(void)
{
	mutex_lock(&nt36523_share_power_mutex);
	/*LCM resume power with ON1->ON2, touch resume with ON.*/
	if (atomic_read(&g_power_request) == 0)
		LCM_ERR("Touch poweron should NOT before LCM poweron");
	lcm_power_manager(LCM_ON);
	mutex_unlock(&nt36523_share_power_mutex);
}
EXPORT_SYMBOL(nt36523_share_powerpin_on);

void nt36523_share_powerpin_off(void)
{
	mutex_lock(&nt36523_share_power_mutex);
	lcm_power_manager(LCM_OFF);
	mutex_unlock(&nt36523_share_power_mutex);
}
EXPORT_SYMBOL(nt36523_share_powerpin_off);

unsigned int nt36523_incell_compare_id(void)
{
	LCM_LOGI("%s", __func__);

	/* lcm id get from Lk */
	if (g_vendor_id == VID_NT_TG_BOE)
		return g_vendor_id;
	else
		return VID_NULL;
}
EXPORT_SYMBOL(nt36523_incell_compare_id);

/* export function end */


/* lcm_platform_driver start */
static int lcm_driver_probe(struct device *dev, void const *data)
{
	int ret = 0;

	if (nt36523_incell_compare_id() != VID_NULL) {
		if (nt36523_share_request_powerpin_resource(dev) < 0) {
			LCM_ERR("fail to request power pins, Panel ID = %x",
					g_vendor_id);
			ret = -1;
		} else {
			LCM_LOGI("lcm probe success, Panel ID = %x",
					g_vendor_id);
			ret = 0;
		}
	} else {
		LCM_ERR("lcm probe fail, mismatch Panel ID = %x", g_vendor_id);
		ret = -ENODEV;
	}

	return ret;
}

static const struct of_device_id lcm_platform_of_match[] = {
	{
		.compatible = "novatek,nt36523",
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
	mutex_lock(&nt36523_share_power_mutex);
	atomic_dec(&g_power_request);
	mutex_unlock(&nt36523_share_power_mutex);
#endif
}

static struct platform_driver lcm_driver = {
	.probe = lcm_platform_probe,
	.remove = lcm_platform_remove,
	.shutdown = lcm_platform_shutdown,
	.driver = {
		.name = "nt36523_wuxga_dsi_vdo",
		.owner = THIS_MODULE,
		.of_match_table = lcm_platform_of_match,
	},
};

static int __init lcm_platform_init(void)
{
	LCM_DBG("Register panel driver for nt36523_wuxga_dsi_vdo");
	if (nt36523_incell_compare_id() != VID_NULL) {
		if (platform_driver_register(&lcm_driver)) {
			LCM_ERR("Failed to register this driver!");
			return -ENODEV;
		}
	} else {
		LCM_LOGI("mismatch Panel ID = %x", g_vendor_id);
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

	params->dsi.vertical_sync_active = 2;
	params->dsi.vertical_backporch = 8;
	params->dsi.vertical_frontporch = 26;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 14;
	params->dsi.horizontal_backporch = 52;
	params->dsi.horizontal_frontporch = 84;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;

	LCM_LOGI("LCM(id=%x, VFP=%d)", g_vendor_id,
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
	params->dsi.lcm_esd_check_table[1].cmd = 0xAB;
	params->dsi.lcm_esd_check_table[1].count = 1;
	params->dsi.lcm_esd_check_table[1].para_list[0] = 0x00;
	/*params->dsi.cust_clk_impendence = 0xf;*/

	/* The values are in mm.*/
	/* Add these values for reporting correct xdpi and ydpi values */
	params->physical_width = 135;
	params->physical_height = 217;
}

static unsigned int get_lcm_id(void)
{
	unsigned int vendor_id = VID_NULL;
	unsigned int vendor_id_tmp = VID_NULL;
	unsigned char buffer[3] = {0};
	unsigned int data_array[16];
	unsigned int retry_cnt;
	int skiplcm = 0;

	LCM_FUNC_DBG_ENTRY();
	retry_cnt = LCM_ID_RETRY_CNT;

	while (retry_cnt > 0) {
		data_array[0] = 0x00013700;
		dsi_set_cmdq(data_array, 1, 1);

		read_reg_v2(0xDA, buffer, 1);
		read_reg_v2(0xDB, buffer+1, 1);
		read_reg_v2(0xDC, buffer+2, 1);

		LCM_LOGI("Display ID1(%x), ID2(%x), ID3(%x)",
				buffer[0], buffer[1], buffer[2]);

		vendor_id_tmp = buffer[0] > LCM_ID1_VENDOR_SHIFT;
		LCM_LOGI("vendor_id_tmp = 0x%x", vendor_id_tmp);

		/* ID2(addr=0x81) value is reserved as 0x81. use this to check if it's a nv36523 driver IC */
		skiplcm = (buffer[1] != RSVD_CODE);

		if ((vendor_id_tmp == VID_NT_TG_BOE) && (!skiplcm)) {
			vendor_id = vendor_id_tmp;
			LCM_LOGI("vendor_id = 0x%x", vendor_id);
			break;
		}
		retry_cnt--;
		LCM_ERR("read id fail, retry_cnt(%d)",
			retry_cnt);
		MDELAY(3);
	}

	LCM_FUNC_DBG_EXIT();
	return vendor_id;
}

static void init_lcm_registers(void)
{

	switch (g_vendor_id) {
	case VID_NT_TG_BOE:
		LCM_LOGI("init lcm(id=%x) registers", g_vendor_id);
		push_table(init_setting,
			sizeof(init_setting) / sizeof(struct LCM_setting_table), 1);
		push_table(sleep_out_setting,
			sizeof(sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
		push_table(register_lock,
			sizeof(register_lock) / sizeof(struct LCM_setting_table), 1);
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
	if (g_vendor_id == VID_NULL)
		g_vendor_id = get_lcm_id();
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
		push_table(register_unlock,
			sizeof(register_unlock) / sizeof(struct LCM_setting_table), 1);
		push_table(sleep_in_setting,
			sizeof(sleep_in_setting) / sizeof(struct LCM_setting_table), 1);
		push_table(register_lock,
			sizeof(register_lock) / sizeof(struct LCM_setting_table), 1);
		MDELAY(LCM_TSLPIN);
		/* RESX LOW require before LP-00 in spec */
		/* g_power_request = 1 means everyone else has */
		/* voted off and will be followed with actual poweroff */
		if (atomic_read(&g_power_request) == 1)
			lcm_set_gpio_output(LCM_RST_GPIO, GPIO_OUT_ZERO);

		break;

	case TRS_DSB:
		push_table(register_unlock,
			sizeof(register_unlock) / sizeof(struct LCM_setting_table), 1);
		push_table(sleep_in_setting,
			sizeof(sleep_in_setting) / sizeof(struct LCM_setting_table), 1);
		push_table(register_lock,
			sizeof(register_lock) / sizeof(struct LCM_setting_table), 1);
		MDELAY(LCM_TSLPIN);
		/* RESX LOW require before LP-00 in spec */
		/* g_power_request = 1 means everyone else has */
		/* voted off and will be followed with actual poweroff */
		if (atomic_read(&g_power_request) == 1)
			lcm_set_gpio_output(LCM_RST_GPIO, GPIO_OUT_ZERO);
		else
			MDELAY(LCM_TVDDOFF);
		break;

	case TRS_SLP_TO_ON:
		push_table(register_unlock,
			sizeof(register_unlock) / sizeof(struct LCM_setting_table), 1);
		push_table(sleep_out_setting,
			sizeof(sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
		push_table(register_lock,
			sizeof(register_lock) / sizeof(struct LCM_setting_table), 1);
		break;

	case TRS_DSB_TO_ON:
		lcm_set_gpio_output(LCM_RST_GPIO, GPIO_OUT_ZERO);
		MDELAY(5);
		lcm_set_gpio_output(LCM_RST_GPIO, GPIO_OUT_ONE);
		MDELAY(1);
		lcm_set_gpio_output(LCM_RST_GPIO, GPIO_OUT_ZERO);
		MDELAY(1);
		lcm_set_gpio_output(LCM_RST_GPIO, GPIO_OUT_ONE);
		MDELAY(1);
		MDELAY(LCM_TRT1);
		init_lcm_registers();
		break;

	case TRS_SLP_TO_OFF:
	case TRS_DSB_TO_OFF:
		ret = display_bias_disable();
		if (ret < 0)
			LCM_ERR("disable display bias power fail: %d",
				ret);
		MDELAY(LCM_Tdb);
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
		MDELAY(LCM_TSVSP);
		ret = display_bias_enable_vol(LCM_AVDD_VOL, LCM_AVEE_VOL, 0);
		if (ret < 0)
			LCM_ERR("enable display bias power fail: %d", ret);
		MDELAY(LCM_Tdb);
		break;

	case TRS_ON_PHASE2:
		MDELAY(LCM_TRT2);
		MDELAY(LCM_TRW);
		lcm_set_gpio_output(LCM_RST_GPIO, GPIO_OUT_ZERO);
		MDELAY(1);
		lcm_set_gpio_output(LCM_RST_GPIO, GPIO_OUT_ONE);
		MDELAY(1);
		lcm_set_gpio_output(LCM_RST_GPIO, GPIO_OUT_ZERO);
		MDELAY(1);
		lcm_set_gpio_output(LCM_RST_GPIO, GPIO_OUT_ONE);
		MDELAY(LCM_TRT1);
		init_lcm_registers();
		break;

	case TRS_OFF_TO_ON:
		if (reg_lcm_vcc1v8) {
			ret = regulator_enable(reg_lcm_vcc1v8);
			if (ret != 0)
				LCM_ERR("enable reg_lcm_vcc1v8 fail");
		}
		MDELAY(LCM_TSVSP);
		ret = display_bias_enable_vol(LCM_AVDD_VOL, LCM_AVEE_VOL, 0);
		if (ret < 0)
			LCM_ERR("enable display bias power fail: %d", ret);
		MDELAY(LCM_Tdb);
		MDELAY(LCM_TRT2);
		MDELAY(LCM_TRW);
		lcm_set_gpio_output(LCM_RST_GPIO, GPIO_OUT_ONE);
		MDELAY(1);
		lcm_set_gpio_output(LCM_RST_GPIO, GPIO_OUT_ZERO);
		MDELAY(1);
		lcm_set_gpio_output(LCM_RST_GPIO, GPIO_OUT_ONE);
		MDELAY(LCM_TRT1);
		init_lcm_registers();
		break;

	case TRS_OFF:
		push_table(register_unlock,
			sizeof(register_unlock) / sizeof(struct LCM_setting_table), 1);
		push_table(sleep_in_setting,
			sizeof(sleep_in_setting) / sizeof(struct LCM_setting_table), 1);
		push_table(register_lock,
			sizeof(register_lock) / sizeof(struct LCM_setting_table), 1);
		MDELAY(LCM_TSLPIN);
		lcm_set_gpio_output(LCM_RST_GPIO, GPIO_OUT_ZERO);
		ret = display_bias_disable();
		if (ret < 0)
			LCM_ERR("disable display bias power fail: %d", ret);
		MDELAY(LCM_Tdb);
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
	mutex_lock(&nt36523_share_power_mutex);
	lcm_power_manager(pwr_state);
	mutex_unlock(&nt36523_share_power_mutex);
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
		LCM_LOGI("lk lcm_id value=%x", lcm_id);
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

	LCM_LOGI("%s setup CABC mode=%d\n", __func__, mode);

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

	switch (mode) {
	case CABC_MOVIE:
		LCM_DBG("CABC_MOVIE\n");
		dsi_set_cmdq_V4(cabc_on_setting,
			sizeof(cabc_on_setting) / sizeof(struct LCM_setting_table_V4), 1);
		break;
	case CABC_OFF:
		LCM_DBG("CABC_OFF\n");
		dsi_set_cmdq_V4(cabc_off_setting,
			sizeof(cabc_off_setting) / sizeof(struct LCM_setting_table_V4), 1);
		break;
	default:
		LCM_ERR("CABC mode %x is not supported now\n", mode);
		break;
	}

err_unsupport:
err_nochange:
	return;
}

LCM_DRIVER nt36523_wuxga_dsi_vdo_lcm_drv = {
	.name = "nt36523_wuxga_dsi_vdo",
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
