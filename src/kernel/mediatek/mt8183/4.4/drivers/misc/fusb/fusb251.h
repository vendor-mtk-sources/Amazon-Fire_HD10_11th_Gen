/*
 * Copyright 2018-2019 ON Semiconductor
 * Copyright 2020 Compal Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

#ifndef __FUSB251_INC__
#define __FUSB251_INC__

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/list.h>
#include <linux/semaphore.h>
#include <linux/workqueue.h>
#include <linux/switch.h>
#if defined(CONFIG_AMAZON_METRICS_LOG) || defined(CONFIG_AMAZON_MINERVA_METRICS_LOG)
#include <linux/metricslog.h>
#endif

#if defined(CONFIG_AMZN_METRICS_LOG) || defined(CONFIG_AMZN_MINERVA_METRICS_LOG)
#include <linux/amzn_metricslog.h>
#endif

#define FUSB251_DEVICE_ID       0x80

/* Register map for the FUSB251 */
#define FUSB251_PRODUCT_ID      0x01
#define FUSB251_CONTROL         0x02
#define FUSB251_INTERRUPT       0x03
#define FUSB251_INT_MASK        0x04
#define FUSB251_STATUS          0x05
#define FUSB251_MOISTATUS       0x06
#define FUSB251_SWITCHCONTROL   0x07
#define FUSB251_THRESHOLD1      0x08
#define FUSB251_THRESHOLD2      0x09
#define FUSB251_TIMER1          0x0A
#define FUSB251_RESET           0x0B
#define FUSB251_TIMER2          0x0C
#define FUSB251_REG_NUM         12

/* for interrupt */
#define MSK_I_OVP               1
#define MSK_I_OVP_REC           (1 << 1)
#define MSK_I_MOS_DET           (1 << 2)
#define MSK_I_DRY_DET           (1 << 3)
#define MSK_I_CC1_TIMER         (1 << 4)
#define MSK_I_CC2_TIMER         (1 << 5)
#define MSK_I_MASK_ALL          0x3F
/* for control */
#define CTL_EN_MAN_MODE         (1 << 0)
#define CTL_EN_CC_DET           (1 << 1)
#define CTL_EN_SBU_DET          (1 << 2)
#define CTL_EN_SBUFT_DET        (1 << 3)
#define CTL_EN_AUTO_SBU_DET     (1 << 4)
#define CTL_EN_DRY_DET          (1 << 5)
#define CTL_DISABLE_ALL         0x00
#define EN_CC_DET_MASK          0x01
#define EN_CC_DET_SHIFT         1
#define EN_SBU_DET_MASK         0x01
#define EN_SBU_DET_SHIFT        2
#define EN_SBUFT_DET_MASK       0x01
#define EN_SBUFT_DET_SHIFT      3
#define EN_AUTO_SBU_DET_MASK    0x01
#define EN_AUTO_SBU_DET_SHIFT   4
#define EN_DRY_DET_MASK         0x01
#define EN_DRY_DET_SHIFT        5
/* for status */
#define STATUS_OVP_CC_MASK      0x01
#define STATUS_OVP_CC_SHIFT     0
#define STATUS_OVP_SBU_MASK     0x01
#define STATUS_OVP_SBU_SHIFT    1
#define STATUS_LOOK4DRY_MASK    0x01
#define STATUS_LOOK4DRY_SHIFT   5
#define STATUS_LOOK4SBU_MASK    0x01
#define STATUS_LOOK4SBU_SHIFT   6
#define STATUS_LOOK4CC_MASK     0x01
#define STATUS_LOOK4CC_SHIFT    7
/* for moisture status */
#define MOISTATUS_CC1MOS_MASK   0x01
#define MOISTATUS_CC1MOS_SHIFT  0
#define MOISTATUS_CC2MOS_MASK   0x01
#define MOISTATUS_CC2MOS_SHIFT  1
#define MOISTATUS_SBU1MOS_MASK  0x01
#define MOISTATUS_SBU1MOS_SHIFT 2
#define MOISTATUS_SBU2MOS_MASK  0x01
#define MOISTATUS_SBU2MOS_SHIFT 3
#define MOISTATUS_SBU1FT_MASK   0x01
#define MOISTATUS_SBU1FT_SHIFT  4
#define MOISTATUS_SBU2FT_MASK   0x01
#define MOISTATUS_SBU2FT_SHIFT  5
/* for switch */
#define SWITCH_CC_MASK          0x01
#define SWITCH_CC_SHIFT         0
#define SWITCH_SBU_MASK         0x03
#define SWITCH_SBU_SHIFT        1
/* for reset */
#define RESET_DEVICE            0x01
#define RESET_MOISTURE          0x02
/* for timer1 */
#define TIMER_DRY_DET_1S        0x03
#define TIMER_DRY_DET_2S        0x04
#define TIMER_DRY_DET_4S        0x05
#define TIMER_DRY_DET_8S        0x06
#define TIMER_DRY_DET_10S       0x07
/* for timer2 */
#define ADC_READ_TIMES_1        0x00
#define ADC_READ_TIMES_2        0x04
#define ADC_READ_TIMES_3        0x08
#define CC_SETTLE_400US         0x00
#define CC_SETTLE_300US         0x01
#define CC_SETTLE_500US         0x02
#define CC_SETTLE_600US         0x03
/* for threshold1 */
#define SBU_MOS_R_DET_MASK      0x0F
#define SBU_MOS_R_DET_SHIFT     4
#define SBU_MOS_R_DET_480K      0x0B
#define SBU_MOS_R_DET_1280K     0x0F
#define CC_MOS_R_DET_MASK       0x0F
#define CC_MOS_R_DET_SHIFT      0
#define CC_MOS_R_DET_480K       0x0B
#define CC_MOS_R_DET_1280K      0x0F
/* for threshold2 */
#define SBU_FLOAT_DET_100MV     0x00
#define SBU_FLOAT_DET_200MV     0x10
#define SBU_FLOAT_DET_300MV     0x20
#define SBU_FLOAT_DET_400MV     0x30
#define SBU_FLOAT_DET_500MV     0x40
#define SBU_FLOAT_DET_600MV     0x50
#define SBU_FLOAT_DET_700MV     0x60
#define SBU_FLOAT_DET_800MV     0x70
#define SBU_FLOAT_DET_MASK      0x07
#define SBU_FLOAT_DET_SHIFT     4
#define VDRY_R_DET_MASK         0x0F
#define VDRY_R_DET_SHIFT        0
#define VDRY_DET_480K           0x0B
#define VDRY_DET_747K           0x0D
#define VDRY_DET_1280K          0x0F

/* Register unions */
union product_id_reg_t {
	unsigned char byte;
	struct {
		unsigned char rev_id               : 2;
		unsigned char prd_id               : 2;
		unsigned char dev_id               : 4;
	};
}; /* R */

union control_reg_t {
	unsigned char byte;
	struct {
		unsigned char man_sw               : 1;
		unsigned char en_cc_mos            : 1;
		unsigned char en_sbu_mos           : 1;
		unsigned char en_sbuft             : 1;
		unsigned char auto_en_sbu          : 1;
		unsigned char en_dry_check         : 1;
		unsigned char reserved             : 1;
		unsigned char dbg_acc_det          : 1;
	};
}; /* R/W */

union interrupt_reg_t {
	unsigned char byte;
	struct {
		unsigned char i_ovp                : 1;
		unsigned char i_ovp_rec            : 1;
		unsigned char i_mos_chg            : 1;
		unsigned char i_dry_chg            : 1;
		unsigned char i_cc1_timer          : 1;
		unsigned char i_cc2_timer          : 1;
		unsigned char reserved             : 2;
	};
}; /* RC */

union mask_reg_t {
	unsigned char byte;
	struct {
		unsigned char m_ovp                : 1;
		unsigned char m_ovp_rec            : 1;
		unsigned char m_mos_det            : 1;
		unsigned char m_dry_det            : 1;
		unsigned char m_cc1_timer          : 1;
		unsigned char m_cc2_timer          : 1;
		unsigned char reserved             : 2;
	};
}; /* R/W */

union status_reg_t {
	unsigned char byte;
	struct {
		unsigned char ovp_cc               : 1;
		unsigned char ovp_sbu              : 1;
		unsigned char reserved             : 3;
		unsigned char look4dry             : 1;
		unsigned char look4sbu             : 1;
		unsigned char look4cc              : 1;
	};
}; /* R */

union moisture_status_t {
	unsigned char byte;
	struct {
		unsigned char cc1_mos              : 1;
		unsigned char cc2_mos              : 1;
		unsigned char sbu1_mos             : 1;
		unsigned char sbu2_mos             : 1;
		unsigned char sbu1_ft              : 1;
		unsigned char sbu2_ft              : 1;
		unsigned char fault                : 2;
	};
}; /* R */

union switch_control_t {
	unsigned char byte;
	struct {
		unsigned char cc_switch            : 1;
		unsigned char sbu_switch           : 2;
		unsigned char reserved             : 5;
	};
}; /* R/W */

union threshold1_reg_t {
	unsigned char byte;
	struct {
		unsigned char cc_mos_det           : 4;
		unsigned char sbu_mos_det          : 4;
	};
}; /* R/W */

union threshold2_reg_t {
	unsigned char byte;
	struct {
		unsigned char vdry                 : 4;
		unsigned char sbu_float_det        : 3;
		unsigned char reserved             : 1;
	};
}; /* R/W */

union timer_reg_t {
	unsigned char byte;
	struct {
		unsigned char tdry                 : 3;
		unsigned char reserved             : 5;
	};
}; /* R/W */

union reset_reg_t {
	unsigned char byte;
	struct {
		unsigned char reset                : 1;
		unsigned char mos_reset            : 1;
		unsigned char reserved             : 6;
		/* [7..2] reserved */
	};
}; /* R/W */

union timer2_reg_t {
	unsigned char byte;
	struct {
		unsigned char cc_settle_time       : 2;
		unsigned char num_adc_read         : 2;
		unsigned char reserved             : 4;
	};
}; /* R/W */

struct device_reg_t {
	union product_id_reg_t          productid;
	union control_reg_t             control;
	union interrupt_reg_t           interrupt;
	union mask_reg_t                mask;
	union status_reg_t              status;
	union moisture_status_t         mos_status;
	union switch_control_t          switch_control;
	union threshold1_reg_t          threshold1;
	union threshold2_reg_t          threshold2;
	union timer_reg_t               timer;
	union reset_reg_t               reset;
	union timer2_reg_t              timer2;
};

struct fusb251 {
	struct i2c_client *i2c;
	struct mutex fusb251_i2c_mutex;
	struct device *dev;
	struct tcpc_device *tcpc;
	struct power_supply *usb_psy;
	struct power_supply *batt_psy;
	struct device_reg_t registers;
	struct switch_dev st_switch;
	struct delayed_work mos_det_work;
	struct wakeup_source fusb251_wake_lock;
	struct timespec event_ts;

	int threshold_sbu_dry2wet;
	int threshold_sbu_wet2dry;
	int threshold_sbuft;
	int task_timer[3];

	unsigned char sbu1_mos_status;
	unsigned char sbu2_mos_status;
	unsigned char sbu1_ft_status;
	unsigned char sbu2_ft_status;
	unsigned char ovp_cc_status;
	unsigned char ovp_sbu_status;
};

/* Voltage table for sbu float detection */
int sbuft_det_MV_table[] = {100, 200, 300, 400, 500, 600,
	700, 800};

/* Resistance table for moisture detection */
int mos_det_R_table[] = {17, 36, 56, 80, 107, 137, 172, 213,
	262, 320, 391, 480, 594, 747, 960, 1280};

#if defined(CONFIG_AMAZON_METRICS_LOG) || defined(CONFIG_AMZN_METRICS_LOG) || defined(CONFIG_AMZN_MINERVA_METRICS_LOG) || defined(CONFIG_AMAZON_MINERVA_METRICS_LOG)
#define METRICS_BUFF_SIZE_FUSB251 512
char g_m_buf_fusb251[METRICS_BUFF_SIZE_FUSB251];

#define fusb251_metrics_log(domain, fmt, ...)				\
do {	\
	memset(g_m_buf_fusb251, 0, METRICS_BUFF_SIZE_FUSB251);		\
	snprintf(g_m_buf_fusb251, sizeof(g_m_buf_fusb251), fmt, ##__VA_ARGS__);	\
	log_to_metrics(ANDROID_LOG_INFO, domain, g_m_buf_fusb251);	\
} while (0)
#else
static inline void fusb251_metrics_log(void) {}
#endif
#define MAX_RESISTANCE              1280
#define MAX_VOLTAGE                 800
#define IUSB_LIMITATION_UA          10000
#define VBUS_INVALID_VOL            2600000
#define THRESHOLD_SBU_DRY2WET       747
#define THRESHOLD_SBU_WET2DRY       960
#define DRY_WORK_PERIOD             15
#define WET_WORK_PERIOD             30
#define WET_AND_VBUS_WORK_PERIOD    60
#define SBUFT_DELAY                 100
#define SBU_DELAY                   700
#define TASK_DELAY_WAKEUP_SEC       5
#define THRESHOLD_SBUFT             100

#endif
