/*
 * Device driver for monitoring ambient light intensity in (lux), RGB, and
 * color temperature (in kelvin) within the AMS-TAOS TCS family of devices.
 *
 * Copyright (c) 2016, AMS-TAOS USA, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */

#ifndef __TCS3400_H
#define __TCS3400_H

#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/wait.h>

#define INTEGRATION_CYCLE 2780
#define SCALER 1000
#define TCS3400_MASK_AGAIN 0x03
#define INDOOR_LUX_TRIGGER	6000
#define OUTDOOR_LUX_TRIGGER	10000
#define TCS3400_MAX_LUX		0xffff
#define TCS3400_MAX_ALS_VALUE	0xffff
#define TCS3400_MIN_ALS_VALUE	3

#define MAX_REGS 256
/* Default LUX and Color coefficients */
#define D_Factor	446
#define R_Coef		90
#define G_Coef		1000
#define B_Coef		(-460)
#define CT_Coef		4128
#define CT_Offset	1448

#define TCS3400_CMD_ALS_INT_CLR  0xE6
#define TCS3400_CMD_ALL_INT_CLR  0xE7

#define CENTI_MSEC_PER_MSEC	100

#define I2C_ADDR_OFFSET		0X00

#define GAIN1	0
#define GAIN4	1
#define GAIN16	2
#define GAIN64	3

#define ALS_PERSIST(p) (((p) & 0xf) << 3)
#define AW_TIME_MS(p)  ((((p) * 1000) + (INTEGRATION_CYCLE - 1)) / INTEGRATION_CYCLE)

struct device;


enum tcs3400_pwr_state {
	POWER_ON,
	POWER_OFF,
	POWER_STANDBY,
};

enum tcs3400_ctrl_reg {
	AGAIN_1        = (0 << 0),
	AGAIN_4        = (1 << 0),
	AGAIN_16       = (2 << 0),
	AGAIN_64       = (3 << 0),
};

struct tcs3400_als_info {
	u32 cpl;
	u32 saturation;
	u16 clear_raw;
	u16 red_raw;
	u16 green_raw;
	u16 blue_raw;
	u16 lux;
	u16 cct;
	s16 ir;
	s16 ir_passband;
};

struct tcs3400_parameters {
	u8 als_time;
	u8 als_gain;
	u8 persist;
	u32 als_deltap;
	u32 als_auto_gain;
	s32 d_factor;
	s32 r_coef;
	s32 g_coef;
	s32 b_coef;
	s32 ct_coef;
	s32 ct_offset;
};

enum tcs3400_regs {
	TCS3400_ENABLE = 0x80,
	TCS3400_ALS_TIME = 0x81,
	TCS3400_WAIT_TIME = 0x83,
	TCS3400_ALS_MINTHRESHLO = 0x84,
	TCS3400_ALS_MINTHRESHHI = 0x85,
	TCS3400_ALS_MAXTHRESHLO = 0x86,
	TCS3400_ALS_MAXTHRESHHI = 0x87,
	TCS3400_PERSISTENCE = 0x8C,
	TCS3400_CONFIG = 0x8D,
	TCS3400_CONTROL = 0x8F,
	TCS3400_REG_AUX = 0x90,
	TCS3400_REVID = 0x91,
	TCS3400_CHIPID = 0x92,
	TCS3400_STATUS = 0x93,
	TCS3400_CLR_CHANLO = 0x94,
	TCS3400_CLR_CHANHI = 0x95,
	TCS3400_RED_CHANLO = 0x96,
	TCS3400_RED_CHANHI = 0x97,
	TCS3400_GRN_CHANLO = 0x98,
	TCS3400_GRN_CHANHI = 0x99,
	TCS3400_BLU_CHANLO = 0x9A,
	TCS3400_BLU_CHANHI = 0x9B,
	TCS3400_IR_TOGGLE = 0xC0,
	TCS3400_IFORCE = 0xE4,
	TCS3400_CLCLEAR = 0xE6,
	TCS3400_AICLEAR = 0xE7,
};

enum tcs3400_en_reg {
	TCS3400_EN_PWR_ON      = (1 << 0),
	TCS3400_EN_ALS         = (1 << 1),
	TCS3400_EN_WAIT        = (1 << 3),
	TCS3400_EN_ALS_IRQ     = (1 << 4),
	TCS3400_EN_ALS_SAT_IRQ = (1 << 5),
	TCS3400_EN_IRQ_PWRDN   = (1 << 6),
};

enum tcs3400_status {
	TCS3400_ST_ALS_VALID  = (1 << 0),
	TCS3400_ST_ALS_IRQ    = (1 << 4),
	TCS3400_ST_ALS_SAT    = (1 << 7),
};

enum {
	TCS3400_ALS_GAIN_MASK = (3 << 0),
	TCS3400_ATIME_DEFAULT_MS = 50,
	MAX_ALS_VALUE = 0xffff,
	MIN_ALS_VALUE = 1,
};

struct tcs3400_chip {
	struct mutex lock;
	struct i2c_client *client;
	struct tcs3400_als_info als_inf;
	struct tcs3400_parameters params;
	struct tcs3400_i2c_platform_data *pdata;
	u8 shadow[MAX_REGS];
	u16 bins_shadow;
	u16 atime_ms;
	struct input_dev *a_idev;
	int in_suspend;
	int wake_irq;
	int irq_pending;
	bool unpowered;
	bool als_enabled;
	bool in_asat;
	u8 device_index;
	u8 ir_toggle_mode;
	u8 ir_toggle_state;
	struct work_struct work_thread;
};

struct tcs3400_i2c_platform_data {
	/* The following callback for power events received and handled by
	   the driver.  Currently only for SUSPEND and RESUME */
	int (*platform_power)(struct device *dev, enum tcs3400_pwr_state state);
	int (*platform_init)(void);
	void (*platform_teardown)(struct device *dev);
	char const *als_name;
	struct tcs3400_parameters parameters;
	bool als_can_wake;
	u32 ams_irq_gpio; /* as per DTS */
#ifdef CONFIG_OF
	struct device_node *of_node;
#endif

};

#endif /* __TCS3400_H */
