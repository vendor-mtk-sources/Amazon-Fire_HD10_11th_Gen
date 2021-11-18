/*
 * Copyright (C) 2012-2015, Focaltech Systems (R)，All Rights Reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/************************************************************************
* File Name: focaltech_driver.c
*
* Author:
*
* Created: 2015-01-01
*
* Abstract: Function for driver initial, report point, resume, suspend
*
************************************************************************/
#include "tpd_custom_fts.h"
#include "focaltech_ex_fun.h"
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
/* #include <linux/rtpm_prio.h> */
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#ifdef CONFIG_AMAZON_METRICS_LOG
#include <linux/metricslog.h>
#endif

#ifdef CONFIG_AMZN_METRICS_LOG
#include <linux/amzn_metricslog.h>
#endif

#if defined(CONFIG_AMAZON_METRICS_LOG) || defined(CONFIG_AMZN_METRICS_LOG)
#include <linux/power_supply.h>
#endif

#ifdef GTP_ESD_PROTECT
#define TPD_ESD_CHECK_CIRCLE	200
static struct delayed_work gtp_esd_check_work;
static struct workqueue_struct *gtp_esd_check_workqueue;
static void gtp_esd_check_func(struct work_struct *);
static int count_irq;
static unsigned long esd_check_circle = TPD_ESD_CHECK_CIRCLE;
static u8 run_check_91_register;
#endif
#if defined(CONFIG_AMAZON_METRICS_LOG) || defined(CONFIG_AMZN_METRICS_LOG)
static char buffer[128];
#endif

#ifdef FTS_CTL_IIC
#include "focaltech_ctl.h"
#endif

/* gesture wakeup*/
int g_fts_gesture_wakeup_delayed; /*-1:no update, 0:disable, 1: enable */
int g_fts_slide_wakeup;
struct mutex g_fts_gesture_lock;

struct fts_touch_info {
	u8 x_msb:4, rev:2, event:2;
	u8 x_lsb;
	u8 y_msb:4, id:4;
	u8 y_lsb;
	u8 weight;
	u8 speed:2, direction:2, area:4;
};

struct fts_packet_info {
	u8 gesture;
	u8 fingers:4, frame:4;
	struct fts_touch_info touch[CFG_MAX_TOUCH_POINTS];
};

/*touch event info*/
struct ts_event {
	/*x coordinate */
	u16 au16_x[CFG_MAX_TOUCH_POINTS];
	/*y coordinate */
	u16 au16_y[CFG_MAX_TOUCH_POINTS];
	/*touch event: 0 -- down; 1-- up; 2 -- contact */
	u8 au8_touch_event[CFG_MAX_TOUCH_POINTS];
	/*touch ID */
	u8 au8_finger_id[CFG_MAX_TOUCH_POINTS];
	u16 pressure[CFG_MAX_TOUCH_POINTS];
	u16 area[CFG_MAX_TOUCH_POINTS];
	u8 touch_point;
	u8 touch_point_num;
};

static DECLARE_WAIT_QUEUE_HEAD(waiter);
static DEFINE_MUTEX(i2c_access);
static DEFINE_MUTEX(i2c_rw_access);
static irqreturn_t tpd_eint_interrupt_handler(int irq, void *dev_id);
static int tpd_focal_probe(struct i2c_client *client,
			   const struct i2c_device_id *id);
static int tpd_detect(struct i2c_client *client, struct i2c_board_info *info);
static int tpd_focal_remove(struct i2c_client *client);
static int touch_event_handler(void *unused);
static void tpd_focal_shutdown(struct i2c_client *client);

struct i2c_client *i2c_client;
struct task_struct *thread;
static int tpd_halt;
static int tpd_rst_gpio_number;
static int tpd_int_gpio_number;
unsigned int touch_irq;
static int IICErrorCountor;
bool focal_suspend_flag;
static int tpd_flag;
bool is_update;
int current_touchs;
unsigned char hw_rev;
unsigned char ft_vendor_id;
unsigned char ft_routing_type;

/************************************************************************
* Name: ftxxxx_i2c_Read
* Brief: i2c read
* Input: i2c info, write buf, write len, read buf, read len
* Output: get data in the 3rd buf
* Return: fail <0
***********************************************************************/
int ftxxxx_i2c_Read_p(struct i2c_client *client, char *writebuf, int writelen,
		    char *readbuf, int readlen, int retrycnt, int delayms)
{
	int ret = -EIO;
	int retry = 0;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
			 },
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		for (retry = 0; retry < retrycnt; retry++) {
			ret = i2c_transfer(client->adapter, msgs, 2);
			if (ret >= 0)
				break;

			msleep(delayms);
		}
	} else {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		for (retry = 0; retry < retrycnt; retry++) {
			ret = i2c_transfer(client->adapter, msgs, 1);
			if (ret >= 0)
				break;

			msleep(delayms);
		}
	}

	if (retry == retrycnt) {
		dev_err(&client->dev,
			"[focal] %s: i2c read error. err code = %d\n", __func__,
			ret);
		IICErrorCountor += 1;
		if (IICErrorCountor >= 10) {
			dev_err(&client->dev,
				"[focal] %s: i2c read/write error over 10 times\n",
				__func__);
			dev_err(&client->dev,
				"[focal] %s: excute reset IC process\n",
				__func__);
			return ret;
		}
		return ret;
	}
	IICErrorCountor = 0;
	return ret;
}

int ftxxxx_i2c_Read(struct i2c_client *client, char *writebuf, int writelen,
		char *readbuf, int readlen)
{
	/*vendor version*/
	return ftxxxx_i2c_Read_p(client, writebuf, writelen, readbuf, readlen,
			IICReadWriteRetryTime, 20);
}
/************************************************************************
* Name: ftxxxx_i2c_Write
* Brief: i2c write
* Input: i2c info, write buf, write len
* Output: no
* Return: fail <0
***********************************************************************/
int ftxxxx_i2c_Write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret;
	int retry = 0;

	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = writelen,
		 .buf = writebuf,
		 },
	};

	for (retry = 0; retry < IICReadWriteRetryTime; retry++) {
		ret = i2c_transfer(client->adapter, msg, 1);
		if (ret >= 0)
			break;
		msleep(30);
	}

	if (retry == IICReadWriteRetryTime) {
		dev_err(&client->dev,
			"[focal] %s: i2c write error. err code = %d\n",
			__func__, ret);

		IICErrorCountor += 1;

		if (IICErrorCountor >= 10) {
			pr_err
			    ("[focal] %s: i2c read/write error over 10 times\n",
			     __func__);
			pr_err("[focal] %s: excute reset IC process\n",
			       __func__);
			return ret;
		}
		return ret;
	}

	IICErrorCountor = 0;

	return ret;
}

/************************************************************************
* Name: fts_write_reg
* Brief: write register
* Input: i2c info, reg address, reg value
* Output: no
* Return: fail <0
***********************************************************************/
int fts_write_reg(struct i2c_client *client, u8 regaddr, u8 regvalue)
{
	unsigned char buf[2] = { 0 };

	buf[0] = regaddr;
	buf[1] = regvalue;
	return ftxxxx_i2c_Write(client, buf, sizeof(buf));
}

/************************************************************************
* Name: fts_read_reg
* Brief: read register
* Input: i2c info, reg address, reg value
* Output: get reg value
* Return: fail <0
***********************************************************************/
int fts_read_reg(struct i2c_client *client, u8 regaddr, u8 *regvalue)
{
	return ftxxxx_i2c_Read(client, &regaddr, 1, regvalue, 1);
}

int fts_i2c_ping(struct i2c_client *client)
{
	unsigned char reg_addr = FTS_REG_VENDOR_ID;
	unsigned char reg_value[2] = { 0 };
	int ret = 0;

	ret = ftxxxx_i2c_Read_p(client, &reg_addr, 1, reg_value, 1, 2, 5);
	if (ret >= 0) {
		/*check supported devices*/
		switch (reg_value[0]) {
		case FT_TopGroup_ID:
		case FT_LENS_ID:
			pr_debug("[focal] Vendor ID:0x%x\n", reg_value[0]);
			break;
		default:
			ret = -ENODEV;
			pr_warn("[focal] Invalid Vendor ID:0x%x\n",
				reg_value[0]);
		}
	}
	return ret;
}
/*register driver and device info*/
static const struct of_device_id focal_dt_match[] = {
	{.compatible = "mediatek,cap_touch"},
	{},
};

MODULE_DEVICE_TABLE(of, focal_dt_match);

static const struct i2c_device_id tpd_id[] = { {TPD_DEVICE, 0}, {} };
static unsigned short force[] = { 0, 0x70, I2C_CLIENT_END, I2C_CLIENT_END };
static const unsigned short *const forces[] = { force, NULL };

static struct i2c_driver tpd_i2c_driver = {

	.driver = {
		   .name = "ft5726",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(focal_dt_match),
		   },
	.probe = tpd_focal_probe,
	.remove = tpd_focal_remove,
	.shutdown = tpd_focal_shutdown,
	.id_table = tpd_id,
	.detect = tpd_detect,
	.address_list = (const unsigned short *)forces,
};

#if defined(CONFIG_AMAZON_METRICS_LOG) || defined(CONFIG_AMZN_METRICS_LOG)
static int fts_charger_exist(void)
{
	int ret;
	struct power_supply *psy;
	union power_supply_propval online_usb, online_ac;

	psy = power_supply_get_by_name("usb");
	if (!psy)
		return -ENODEV;
	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_PRESENT,
			&online_usb);
	if (ret)
		return -ENOENT;

	psy = power_supply_get_by_name("ac");
	if (!psy)
		return -ENODEV;
	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_PRESENT,
			&online_ac);
	if (ret)
		return -ENOENT;

	if (online_usb.intval || online_ac.intval)
		return 1;
	else
		return 0;
}
#endif

/***********************************************************************
* Name: fts_report_value
* Brief: report the point information
* Input: event info
* Output: no
* Return: success is zero
***********************************************************************/
static int fts_report_value(struct ts_event *data)
{
	int ret = -1;
	int touchs = 0;
	u8 addr = 0x01;
	struct fts_packet_info buf;
	struct fts_touch_info *touch;
	int i, x, y;
	u8 gesture_buf[1] = { 0 };
#if defined(CONFIG_AMAZON_METRICS_LOG) || defined(CONFIG_AMZN_METRICS_LOG)
	int charger_flag = 0;
#endif

	mutex_lock(&i2c_access);
	ret = ftxxxx_i2c_Read(i2c_client, &addr, 1, (char *)&buf,
			      sizeof(struct fts_packet_info));
	if (ret < 0) {
		pr_err("[focal] %s read touchdata failed.\n", __func__);
		mutex_unlock(&i2c_access);
		return ret;
	}
	mutex_unlock(&i2c_access);

	if (g_fts_slide_wakeup && focal_suspend_flag) {
		ret = fts_read_reg(i2c_client, FTS_REG_GESTURE_OUTPUT,
			&gesture_buf[0]);
		if (ret < 0) {
			pr_err("[focal] read gesture data failed\n");
			return ret;
		}
		pr_info("[focal] gesture id = 0x%X\n", gesture_buf[0]);
		if (gesture_buf[0] == FTS_GESTURE_ID_TAP) {
			/* report power key event */
			input_report_key(tpd->dev, KEY_POWER, 1);
			input_sync(tpd->dev);
			input_report_key(tpd->dev, KEY_POWER, 0);
			input_sync(tpd->dev);
			focal_suspend_flag = false;

#if defined(CONFIG_AMAZON_METRICS_LOG) || defined(CONFIG_AMZN_METRICS_LOG)
			charger_flag = fts_charger_exist();
			if (charger_flag >= 0) {
				snprintf(buffer, sizeof(buffer),
				"%s:touchwakup:%s_charger_wakeup=1;CT;1:NR",
				__func__, (charger_flag ? "with" : "without"));
				log_to_metrics(ANDROID_LOG_INFO,
					"TouchWakeup", buffer);
			} else
				pr_err("%s charger_exist error: %d\n",
						__func__, charger_flag);
#endif
			return 0;
		}
	}

	touch = &buf.touch[0];
	for (i = 0; i < (buf.fingers) && (i < CFG_MAX_TOUCH_POINTS);
	     i++, touch++) {

		x = (u16) (touch->x_msb << 8) | (u16) touch->x_lsb;
		y = (u16) (touch->y_msb << 8) | (u16) touch->y_lsb;
		input_mt_slot(tpd->dev, touch->id);

		if (touch->event == 0 || touch->event == 2) {
			input_mt_report_slot_state(tpd->dev,
						   MT_TOOL_FINGER, true);
			input_report_abs(tpd->dev, ABS_MT_PRESSURE,
					 touch->weight);
			input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR,
					 touch->area);
			input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
			input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
			touchs |= BIT(touch->id);
			current_touchs |= BIT(touch->id);

			/*
			  * pr_debug
			  * ("[focal]%d: id=%d, x=%d, y=%d, p=0x%x, m=%d\n",
			  * i, touch->id, x, y, touch->weight, touch->area);
			 */
		} else {
			input_mt_report_slot_state(tpd->dev,
						   MT_TOOL_FINGER, false);
			current_touchs &= ~BIT(touch->id);
		}
	}

	if (unlikely(current_touchs ^ touchs)) {
		for (i = 0; i < CFG_MAX_TOUCH_POINTS; i++) {
			if (BIT(i) & (current_touchs ^ touchs)) {
				input_mt_slot(tpd->dev, i);
				input_mt_report_slot_state(tpd->dev,
							   MT_TOOL_FINGER,
							   false);
			}
		}
	}
	current_touchs = touchs;
	if (touchs)
		input_report_key(tpd->dev, BTN_TOUCH, 1);
	else
		input_report_key(tpd->dev, BTN_TOUCH, 0);

	input_sync(tpd->dev);
	return 0;
}

/************************************************************************
* Name: touch_event_handler
* Brief: interrupt event from TP, and read/report data to Android system
* Input: no use
* Output: no
* Return: 0
***********************************************************************/
static int touch_event_handler(void *unused)
{
	struct ts_event pevent;
	struct sched_param param = {.sched_priority = 4 };

	sched_setscheduler(current, SCHED_RR, &param);

	do {
		set_current_state(TASK_INTERRUPTIBLE);
		wait_event_interruptible(waiter, tpd_flag != 0);
		tpd_flag = 0;
		set_current_state(TASK_RUNNING);
		fts_report_value(&pevent);
	} while (!kthread_should_stop());

	return 0;
}

/************************************************************************
* Name: fts_reset_tp
* Brief: reset TP
* Input: pull low or high
* Output: no
* Return: 0
***********************************************************************/
void fts_reset_tp(int HighOrLow)
{
	if (HighOrLow)
		gpio_set_value(tpd_rst_gpio_number, 1);
	else
		gpio_set_value(tpd_rst_gpio_number, 0);
}

/************************************************************************
* Name: tpd_detect
* Brief: copy device name
* Input: i2c info, board info
* Output: no
* Return: 0
***********************************************************************/
static int tpd_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, TPD_DEVICE);
	return 0;
}

/************************************************************************
* Name: tpd_eint_interrupt_handler
* Brief: deal with the interrupt event
* Input: no
* Output: no
* Return: no
***********************************************************************/
static irqreturn_t tpd_eint_interrupt_handler(int irq, void *dev_id)
{
	TPD_DEBUG_PRINT_INT;
	tpd_flag = 1;
	wake_up_interruptible(&waiter);
	return IRQ_HANDLED;
}

/************************************************************************
* Name: fts_init_gpio_hw
* Brief: initial gpio
* Input: no
* Output: no
* Return: 0
***********************************************************************/
static int fts_init_gpio_hw(void)
{
	int ret = 0;

	ret = gpio_request(tpd_rst_gpio_number, "ft5726-rst");
	if (ret) {
		pr_err("[focal] reset-gpio:%d request fail.\n",
		       tpd_rst_gpio_number);
		return ret;
	}

	ret = gpio_request(tpd_int_gpio_number, "ft5726-int");
	if (ret) {
		pr_err("[focal] int-gpio:%d request fail.\n",
						tpd_int_gpio_number);
		gpio_free(tpd_rst_gpio_number);
		return ret;
	}

	gpio_direction_output(tpd_rst_gpio_number, 0);
	/*
	* pr_debug("[focal] rst-gpio = %d, int-gpio = %d\n",
	* gpio_get_value(tpd_rst_gpio_number),
	* gpio_get_value(tpd_int_gpio_number));
	*/

	return ret;
}

static void fts_reset_hw(void)
{
	gpio_set_value(tpd_rst_gpio_number, 0);
	usleep_range(2000, 3000);/*trst*/

	gpio_set_value(tpd_rst_gpio_number, 1);

	msleep(210);/*trsi*/
	gpio_direction_input(tpd_int_gpio_number);
	/*
	 * pr_debug("[focal] rst-gpio = %d, int-gpio = %d\n",
	 * gpio_get_value(tpd_rst_gpio_number),
	 * gpio_get_value(tpd_int_gpio_number));
	 */
}
#ifdef FTS_POWER_DOWN_IN_SUSPEND
static void tpd_power_down(void)
{
	int ret = 0;

	gpio_set_value(tpd_rst_gpio_number, 0);

	usleep_range(2000, 3000);/*Tvdr*/

#ifdef CONFIG_TOUCHSCREEN_DUAL_POWER
		ret = regulator_disable(tpd->io_reg);
		if (ret != 0)
			pr_debug("[focal]Failed to disable VIO: %d\n", ret);
#endif

	ret = regulator_disable(tpd->reg);
	if (ret != 0)
		pr_debug("[focal]Failed to disable VTP: %d\n", ret);
	usleep_range(10000, 11000);/*Tpdt*/
}
#endif

static void tpd_power_on(void)
{
	int ret = 0;

	/* pr_debug("[focal] %s-%d\n", __func__, __LINE__); */

	gpio_set_value(tpd_rst_gpio_number, 0);

	usleep_range(200, 300); /*Trtp*/

#ifdef CONFIG_TOUCHSCREEN_DUAL_POWER
	ret = regulator_enable(tpd->io_reg);
	if (ret != 0)
		pr_debug("[focal] Failed to enable VIO: %d\n", ret);
	usleep_range(20, 30); /*Tivd*/
#endif

	ret = regulator_enable(tpd->reg);
	if (ret != 0)
		pr_err("[focal] Failed to enable vtp: %d\n", ret);

	usleep_range(2000, 3000); /*Tvdr*/
	fts_reset_hw();
}

static void gesture_wakeup_enable(void)
{
	int retval = 0;

	retval = irq_set_irq_wake(touch_irq, true);
	if (retval)
		pr_err("[focal]irq_set_irq_wake err = %d\n", retval);

	mutex_lock(&i2c_access);
	fts_write_reg(i2c_client, 0xD0, 0x01);
	fts_write_reg(i2c_client, 0xD1, 0x10);
	mutex_unlock(&i2c_access);
}

static int of_get_focal_platform_data(struct device *dev)
{
	if (dev->of_node) {
		const struct of_device_id *match;

		match = of_match_device(of_match_ptr(focal_dt_match), dev);
		if (!match) {
			pr_err("[focal] %s: No device match found\n", __func__);
			return -ENODEV;
		}
	}

	tpd_rst_gpio_number = of_get_named_gpio(dev->of_node, "rst-gpio", 0);
	if (tpd_rst_gpio_number < 0) {
		pr_err("[focal] Can't find rst-gpio\n");
		return tpd_rst_gpio_number;
	}

	tpd_int_gpio_number = of_get_named_gpio(dev->of_node, "int-gpio", 0);
	if (tpd_int_gpio_number < 0) {
		pr_err("[focal] Can't find int-gpio\n");
		return tpd_int_gpio_number;
	}

	touch_irq = irq_of_parse_and_map(dev->of_node, 0);
	if (touch_irq < 0) {
		pr_err("[focal] get touch_irq failed!\n");
		return -ENOENT;
	}

	pr_debug("[focal] tpd_rst_gpio= %d, tpd_int_gpio= %d, irq = %d\n",
	       tpd_rst_gpio_number, tpd_int_gpio_number, touch_irq);
	return 0;
}
#if 0
static char get_hw_rev(struct device *dev)
{
	unsigned int hwid_gpio[3] = { 0 };
	struct pinctrl *hwid_gpio_pinctrl;
	struct pinctrl_state *set_state;
	char retval = 0;

	if (dev->of_node) {
		const struct of_device_id *match;

		match = of_match_device(of_match_ptr(focal_dt_match), dev);
		if (!match) {
			pr_err("[focal] No of-device match found\n");
			return 0;
		}
	}

	hwid_gpio_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(hwid_gpio_pinctrl)) {
		pr_err("[focal] get pinctrl failed, return hw_rev 0\n");
		return 0;
	}

	set_state =
	    pinctrl_lookup_state(hwid_gpio_pinctrl, "hwid_gpio_default");
	if (IS_ERR(set_state)) {
		pr_err
		 ("[focal]hwid_gpio_default not found, return hw_rev 0\n");
		return 0;
	}

	retval = pinctrl_select_state(hwid_gpio_pinctrl, set_state);
	if (retval < 0) {
		pr_err("[focal] set pinctrl state failed, return hw_rev 0\n");
		return 0;
	}

	retval = 0;
	hwid_gpio[0] = of_get_named_gpio(dev->of_node, "hwid0-gpio", 0);
	if (hwid_gpio[0] < 0) {
		pr_err("[focal] Can't find hwid0-gpio\n");
		return 0;
	}
	gpio_direction_input(hwid_gpio[0]);
	retval |= gpio_get_value(hwid_gpio[0]);

	hwid_gpio[1] = of_get_named_gpio(dev->of_node, "hwid1-gpio", 0);
	if (hwid_gpio[1] < 0) {
		pr_err("[focal] Can't find hwid1-gpio\n");
		return 0;
	}
	gpio_direction_input(hwid_gpio[1]);
	retval |= (gpio_get_value(hwid_gpio[1]) << 1);

	hwid_gpio[2] = of_get_named_gpio(dev->of_node, "hwid2-gpio", 0);
	if (hwid_gpio[2] < 0) {
		pr_err("[focal] Can't find hwid2-gpio\n");
		return 0;
	}
	gpio_direction_input(hwid_gpio[2]);
	retval |= (gpio_get_value(hwid_gpio[2]) << 2);

	return retval;
}
#endif

/***********************************************************************
* Name: tpd_probe
* Brief: driver entrance function for initial/power on/create channel
* Input: i2c info, device id
* Output: no
* Return: 0
***********************************************************************/
static int tpd_focal_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	int retval = 0, fs = 0, chdev = 0, proc = 0;
	unsigned char reg_value[2] = { 0 };
	unsigned char reg_addr;

	pr_info("[focal] %s-%d probe start.\n", __func__, __LINE__);

	i2c_client = client;

	retval = of_get_focal_platform_data(&i2c_client->dev);
	if (retval) {
		pr_err("[focal] GPIO configuration failed\n");
		return retval;
	}

	retval = fts_init_gpio_hw();
	if (retval) {
		pr_err("[focal] Init_gpio_hw failed\n");
		return retval;
	}
	/*get tpd regulator */
	tpd->reg = regulator_get(tpd->tpd_dev, "vtouch");
	if (IS_ERR(tpd->reg)) {
		pr_err("[focal] Can't request VTP power supply: %ld\n",
			   PTR_ERR(tpd->reg));
		goto err_free_gpio;
	}

	retval = regulator_set_voltage(tpd->reg, 3000000, 3000000);
	if (retval != 0) {
		pr_err
			("[focal]Failed to set VTP voltage:%d\n", retval);
		goto err_free_vtp;
	}
#ifdef CONFIG_TOUCHSCREEN_DUAL_POWER
	tpd->io_reg = devm_regulator_get_optional(tpd->tpd_dev, "vtouchio");
	if (IS_ERR(tpd->io_reg))
		pr_err("[focal] Can't request VIO power supply\n");

	retval = regulator_set_voltage(tpd->io_reg, 1800000, 1800000);
	if (retval != 0) {
		pr_err("[focal] Failed to set VIO voltage: %d\n",
			retval);
		goto err_free_vio;
	}
#endif

	/* hw_rev = get_hw_rev(&i2c_client->dev);	*/	/*No need */
	/* pr_debug("[focal] HW rev:%d\n", hw_rev); */

	tpd_power_on();

	retval = fts_i2c_ping(client);
	if (retval < 0) {
		pr_err("Failed to ping touch device with %d.\n", retval);
		goto err_free_hw;
	}

	/* Check if TP is connectted, if not return -1 */
	retval = fts_read_reg(i2c_client, 0x00, &reg_value[0]);
	if (retval < 0 || reg_value[0] != 0) {
		/* reg0 data running state is 0; other state is not 0 */
		pr_err("[focal] Chip does not exits. I2C transfer error:%d, Reg0:%d\n",
			retval, reg_value[0]);
		goto err_free_hw;
	}

	reg_addr = FTS_REG_FW_VER;
	retval = ftxxxx_i2c_Read(i2c_client, &reg_addr, 1, reg_value, 1);
	if (retval < 0) {
		pr_err("[focal] Read FW ver error!\n");
		goto err_free_hw;
	} else
		pr_info("[focal] FW ver:0x%x\n", reg_value[0]);

	reg_addr = FTS_REG_VENDOR_ID;
	retval = ftxxxx_i2c_Read(i2c_client, &reg_addr, 1, reg_value, 1);
	if (retval < 0) {
		pr_err("[focal] Read Vendor ID error!\n");
		goto err_free_hw;
	} else {
		ft_vendor_id = reg_value[0];
	}

	reg_addr = FTS_REG_ROUTING_TYPE;
	retval = ftxxxx_i2c_Read(i2c_client, &reg_addr, 1, reg_value, 1);
	if (retval < 0) {
		pr_err("[focal]Read routing type error!\n");
		goto err_free_hw;
	} else {
		ft_routing_type = reg_value[0];
		pr_info("[focal]Routing type:0x%x\n", reg_value[0]);
	}

	reg_addr = FTS_REG_CHIP_ID;
	retval = ftxxxx_i2c_Read(i2c_client, &reg_addr, 1, &reg_value[1], 1);
	if (retval < 0) {
		pr_err("[focal] Read Chip ID error!\n");
		goto err_free_hw;
	} else
		pr_info("[focal] Chip ID:0x%x\n", reg_value[1]);

	retval = request_irq(touch_irq, tpd_eint_interrupt_handler,
			     IRQF_TRIGGER_FALLING, TPD_DEVICE, NULL);
	if (retval < 0) {
		pr_err("[focal] request_irq faled.");
		goto err_free_hw;
	}

	thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
	if (IS_ERR(thread)) {
		retval = PTR_ERR(thread);
		pr_err("[focal] failed to create kernel thread: %d\n", retval);
		goto err_free_irq;
	}

	pr_debug("[focal] %s-%d\n", __func__, __LINE__);

#ifdef SYSFS_DEBUG
	fs = fts_create_sysfs(i2c_client);
	if (fs != 0)
		pr_err("[focal] %s - ERROR: create sysfs failed.\n", __func__);
#endif

#ifdef FTS_CTL_IIC
	chdev = fts_rw_iic_drv_init(i2c_client);
	if (chdev < 0)
		pr_err("[focal] %s: create fts control iic driver failed\n",
		       __func__);
#endif

#ifdef FTS_APK_DEBUG
	proc = fts_create_apk_debug_channel(i2c_client);
	if (proc != 0)
		pr_err("[focal] Couldn't create apk debug channel!\n");
#endif

#ifdef TPD_AUTO_UPGRADE
	pr_info("[focal] *****Enter Focal Touch Auto Upgrade*****\n");

	is_update = true;
	retval = fts_ctpm_auto_upgrade(i2c_client);
	if (retval != 0)
		pr_err("[focal] Auto upgrade failed\n");

	is_update = false;
#endif

#ifdef GTP_ESD_PROTECT
	pr_debug("[focal] %s-%d\n", __func__, __LINE__);
	INIT_DELAYED_WORK(&gtp_esd_check_work, gtp_esd_check_func);

	gtp_esd_check_workqueue = create_workqueue("gtp_esd_check");
	if (gtp_esd_check_workqueue == NULL) {
		pr_err("[focal] %s: couldn't create ESD workqueue\n", __func__);
		retval = -ENOMEM;
		goto err_free;
	}

	queue_delayed_work(gtp_esd_check_workqueue,
			   &gtp_esd_check_work, TPD_ESD_CHECK_CIRCLE);
#endif

	input_mt_init_slots(tpd->dev, CFG_MAX_TOUCH_POINTS, 0);
	input_set_abs_params(tpd->dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(tpd->dev, ABS_MT_POSITION_X, 0, TPD_RES_X, 0, 0);
	input_set_abs_params(tpd->dev, ABS_MT_POSITION_Y, 0, TPD_RES_Y, 0, 0);
	input_set_abs_params(tpd->dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
	input_set_capability(tpd->dev, EV_KEY, KEY_POWER);
	g_fts_gesture_wakeup_delayed = -1;
	mutex_init(&g_fts_gesture_lock);

	pr_info("[focal] Touch driver probe %s\n",
	       (retval < 0) ? "FAIL" : "PASS");

	focal_suspend_flag = false;
	tpd_load_status = 1;
	return 0;

#ifdef GTP_ESD_PROTECT
err_free:
	if (gtp_esd_check_workqueue != NULL)
		destroy_workqueue(gtp_esd_check_workqueue);

	kthread_stop(thread);
#endif
#ifdef SYSFS_DEBUG
	if (fs == 0)
		fts_release_sysfs(client);
#endif
#ifdef FTS_CTL_IIC
	if (chdev >= 0)
		fts_rw_iic_drv_exit();
#endif
#ifdef FTS_APK_DEBUG
	if (proc == 0)
		fts_release_apk_debug_channel();
#endif
err_free_irq:
	free_irq(touch_irq, NULL);
err_free_hw:
#ifdef CONFIG_TOUCHSCREEN_DUAL_POWER
	if (regulator_disable(tpd->io_reg))
		pr_err("[focal]Failed to disable VIO: %d\n", retval);
#endif
	if (regulator_disable(tpd->reg))
		pr_err("[focal]Failed to disable VTP: %d\n", retval);

#ifdef CONFIG_TOUCHSCREEN_DUAL_POWER
err_free_vio:
	regulator_put(tpd->io_reg);
#endif

err_free_vtp:
	regulator_put(tpd->reg);

err_free_gpio:
	gpio_free(tpd_rst_gpio_number);
	gpio_free(tpd_int_gpio_number);

	return retval;
}

/***********************************************************************
* Name: tpd_focal_remove
* Brief: remove driver/channel
* Input: i2c info
* Output: no
* Return: 0
***********************************************************************/
static int tpd_focal_remove(struct i2c_client *client)
{
	int ret = 0;
#ifdef GTP_ESD_PROTECT
	destroy_workqueue(gtp_esd_check_workqueue);
	kthread_stop(thread);
#endif

#ifdef SYSFS_DEBUG
	fts_release_sysfs(client);
#endif

#ifdef FTS_CTL_IIC
	fts_rw_iic_drv_exit();
#endif

#ifdef FTS_APK_DEBUG
	fts_release_apk_debug_channel();
#endif

	free_irq(touch_irq, NULL);

	gpio_free(tpd_rst_gpio_number);
	gpio_free(tpd_int_gpio_number);

#ifdef CONFIG_TOUCHSCREEN_DUAL_POWER
		ret = regulator_disable(tpd->io_reg);
		if (ret != 0)
			pr_err("[focal]Failed to disable VIO: %d\n", ret);
		regulator_put(tpd->io_reg);
#endif

	ret = regulator_disable(tpd->reg);
	if (ret != 0)
		pr_err("[focal]Failed to disable VTP: %d\n", ret);

	regulator_put(tpd->reg);

	pr_debug("[focal] TPD removed\n");
	return 0;
}

#ifdef GTP_ESD_PROTECT
/***********************************************************************
* Name: force_reset_guitar
* Brief: reset
* Input: no
* Output: no
* Return: 0
***********************************************************************/
static void force_reset_guitar(void)
{
	int ret;

	pr_debug("[focal] force_reset_guitar\n");

	ret = regulator_disable(tpd->reg);
	if (ret != 0)
		pr_err("[focal] Failed to disable VTP: %d\n", ret);

#ifdef CONFIG_TOUCHSCREEN_DUAL_POWER
		ret = regulator_disable(tpd->io_reg);
		if (ret != 0)
			pr_err("[focal] Failed to disable VIO: %d\n", ret);
	}
#endif

	msleep(200);

#ifdef CONFIG_TOUCHSCREEN_DUAL_POWER
		ret = regulator_enable(tpd->io_reg);
		if (ret != 0)
			pr_err("[focal] Failed to enable VIO: %d\n", ret);
	}
#endif

	ret = regulator_enable(tpd->reg);
	if (ret != 0)
		pr_debug("[focal] Failed to enable vtp: %d\n", ret);

	msleep(20);

	gpio_direction_output(tpd_rst_gpio_number, 0);

	msleep(20);

	pr_debug("[focal] ic reset\n");

	gpio_set_value(tpd_rst_gpio_number, 1);

	msleep(300);
}

/* 0 for no apk upgrade, 1 for apk upgrade */

#define A3_REG_VALUE			0x58
#define RESET_91_REGVALUE_SAMECOUNT	5
static u8 g_old_91_Reg_Value;
static u8 g_first_read_91 = 0x01;
static u8 g_91value_same_count;

/***********************************************************************
* Name: gtp_esd_check_func
* Brief: esd check function
* Input: struct work_struct
* Output: no
* Return: 0
***********************************************************************/
static void gtp_esd_check_func(struct work_struct *work)
{
	int i = 0;
	int ret = -1;
	u8 data = 0;
	u8 flag_error = 0;
	int reset_flag = 0;

	if (tpd_halt || is_update)
		return;

	if (apk_debug_flag) {
		queue_delayed_work(gtp_esd_check_workqueue,
				   &gtp_esd_check_work, esd_check_circle);
		return;
	}

	run_check_91_register = 0;

	for (i = 0; i < 3; i++) {
		ret = fts_read_reg(i2c_client, 0xA3, &data);
		if (ret < 0)
			pr_err("[focal] read chip ID fail (%d)\n", i);

		if (ret >= 1 && A3_REG_VALUE == data)
			break;
	}

	if (i >= 3) {
		force_reset_guitar();
		pr_err("[focal] ret = %d, A3_Reg_Value = 0x%02x\n", ret, data);
		reset_flag = 1;
		goto FOCAL_RESET_A3_REGISTER;
	}

	ret = fts_read_reg(i2c_client, 0x8F, &data);
	if (ret < 0)
		pr_err("[focal] read value fail");

	pr_debug("[focal] 0x8F:%d, count_irq is %d\n", data, count_irq);
	flag_error = 0;

	if (((count_irq - data) > 10) && ((data + 200) > (count_irq + 10)))
		flag_error = 1;

	if ((data - count_irq) > 10)
		flag_error = 1;

	if (flag_error == 1) {
		pr_err("[focal] tpd reset. data=%d, count_irq=%d\n",
		       data, count_irq);
		force_reset_guitar();
		reset_flag = 1;
		goto FOCAL_RESET_INT;
	}

	run_check_91_register = 1;
	ret = fts_read_reg(i2c_client, 0x91, &data);
	if (ret < 0)
		pr_err("[focal] read value fail");

	pr_debug("[focal] 91 register value = 0x%02x old value = 0x%02x\n",
	       data, g_old_91_Reg_Value);

	if (g_first_read_91 == 0x01) {
		g_old_91_Reg_Value = data;
		g_first_read_91 = 0x00;
	} else {
		if (g_old_91_Reg_Value == data) {
			g_91value_same_count++;
			pr_debug("[focal] g_91value_same_count=%d\n",
			       g_91value_same_count);

			if (RESET_91_REGVALUE_SAMECOUNT ==
					g_91value_same_count) {
				force_reset_guitar();
				pr_err("[focal] g_91value_same_count = 5\n");
				g_91value_same_count = 0;
				reset_flag = 1;
			}

			esd_check_circle = TPD_ESD_CHECK_CIRCLE / 2;
			g_old_91_Reg_Value = data;
		} else {
			g_old_91_Reg_Value = data;
			g_91value_same_count = 0;
			esd_check_circle = TPD_ESD_CHECK_CIRCLE;
		}
	}

FOCAL_RESET_INT:
FOCAL_RESET_A3_REGISTER:
	count_irq = 0;
	data = 0;
	ret = fts_write_reg(i2c_client, 0x8F, data);
	if (ret < 0)
		pr_err("[focal] write value fail");

	if (run_check_91_register == 0)
		g_91value_same_count = 0;

	if (!tpd_halt) {
		queue_delayed_work(gtp_esd_check_workqueue,
				   &gtp_esd_check_work, esd_check_circle);
	}
}
#endif

/************************************************************************
* Name: tpd_local_init
* Brief: add driver info
* Input: no
* Output: no
* Return: fail <0
***********************************************************************/
static int tpd_local_init(void)
{
	/* pr_debug("[focal] fts I2C Driver init.\n"); */

	if (i2c_add_driver(&tpd_i2c_driver) != 0) {
		pr_err("[Foacl] fts unable to add i2c driver.\n");
		return -1;
	}

	input_set_abs_params(tpd->dev, ABS_MT_TRACKING_ID,
			     0, (TINNO_TOUCH_TRACK_IDS - 1), 0, 0);

	tpd_type_cap = 1;

	pr_debug("[focal] fts add i2c driver ok.\n");
	return 0;
}

/************************************************************************
* Name: tpd_resume
* Brief: system wake up
* Input: no use
* Output: no
* Return: no
***********************************************************************/
static void tpd_resume(struct device *h)
{
	int retval;
	pr_debug("[focal] %s-%d TOUCH RESUME.\n", __func__, __LINE__);

	if (g_fts_slide_wakeup) {
		retval = disable_irq_wake(touch_irq);
		if (retval)
			pr_err("[focal] disable_irq_wake err = %d\n", retval);
	}

#ifdef FTS_POWER_DOWN_IN_SUSPEND
	if (!g_fts_slide_wakeup)
		tpd_power_on();
	else
		fts_reset_hw();
#else
	fts_reset_hw();
#endif
	if (!g_fts_slide_wakeup)
		enable_irq(touch_irq);

	msleep(30);
	tpd_halt = 0;
#ifdef GTP_ESD_PROTECT
	queue_delayed_work(gtp_esd_check_workqueue,
			   &gtp_esd_check_work, TPD_ESD_CHECK_CIRCLE);
#endif
	focal_suspend_flag = false;
	pr_debug("[focal] %s-%d TOUCH RESUME DONE.\n", __func__, __LINE__);
}

/************************************************************************
* Name: tpd_suspend
* Brief: system sleep
* Input: no use
* Output: no
* Return: no
***********************************************************************/
static void tpd_suspend(struct device *h)
{
	int i = 0;

	if (focal_suspend_flag) {
		pr_debug("[focal] IC already suspend\n");
		return;
	}
	pr_debug("[focal] %s-%d TOUCH SUSPEND.\n", __func__, __LINE__);

#ifdef GTP_ESD_PROTECT
	cancel_delayed_work_sync(&gtp_esd_check_work);
#endif

	tpd_halt = 1;
	if (unlikely(current_touchs)) {
		for (i = 0; i < CFG_MAX_TOUCH_POINTS; i++) {
			if (BIT(i) & current_touchs) {
				input_mt_slot(tpd->dev, i);
				input_mt_report_slot_state(tpd->dev,
							   MT_TOOL_FINGER,
							   false);
			}
		}
		current_touchs = 0;
		input_report_key(tpd->dev, BTN_TOUCH, 0);
		input_sync(tpd->dev);
	}
	mutex_lock(&g_fts_gesture_lock);
	if (g_fts_gesture_wakeup_delayed >= 0) {
		pr_debug("Update geture wakeup %d to %d\n",
			g_fts_slide_wakeup, g_fts_gesture_wakeup_delayed);
		g_fts_slide_wakeup = g_fts_gesture_wakeup_delayed;
		g_fts_gesture_wakeup_delayed = -1;
	}
	mutex_unlock(&g_fts_gesture_lock);

	if (!g_fts_slide_wakeup)
		disable_irq(touch_irq);

#ifdef FTS_POWER_DOWN_IN_SUSPEND
	if (!g_fts_slide_wakeup)
		tpd_power_down();
	else
		gesture_wakeup_enable();
#else
	if (g_fts_slide_wakeup)
		gesture_wakeup_enable();
	else {
		mutex_lock(&i2c_access);
		/* write i2c command to make IC into deepsleep */
		fts_write_reg(i2c_client, 0xa5, 0x03);
		mutex_unlock(&i2c_access);
	}
#endif

	focal_suspend_flag = true;
	pr_debug("[focal] %s-%d TOUCH SUSPEND DONE.\n", __func__, __LINE__);
}

static void tpd_focal_shutdown(struct i2c_client *client)
{
	int ret = 0;
	unsigned int isenable;

	tpd_halt = 1;

#ifdef GTP_ESD_PROTECT
	cancel_delayed_work(&gtp_esd_check_work);
	flush_workqueue(gtp_esd_check_workqueue);
	destroy_workqueue(gtp_esd_check_workqueue);
#endif

#ifdef SYSFS_DEBUG
	fts_release_sysfs(client);
#endif

#ifdef FTS_CTL_IIC
	fts_rw_iic_drv_exit();
#endif

#ifdef FTS_APK_DEBUG
	fts_release_apk_debug_channel();
#endif

	if (g_fts_slide_wakeup == 1 && focal_suspend_flag)
		irq_set_irq_wake(touch_irq, false);

	free_irq(touch_irq, NULL);

	gpio_set_value(tpd_rst_gpio_number, 0);

	usleep_range(2000, 3000);/*Tvdr*/

	gpio_free(tpd_rst_gpio_number);
	gpio_free(tpd_int_gpio_number);

#ifdef CONFIG_TOUCHSCREEN_DUAL_POWER
	isenable = regulator_is_enabled(tpd->io_reg);
	if (isenable) {
		ret = regulator_disable(tpd->io_reg);
		if (ret != 0)
			pr_err("[focal]Failed to disable VIO: %d\n", ret);
	}
	regulator_put(tpd->io_reg);
#endif

	isenable = regulator_is_enabled(tpd->reg);
	if (isenable) {
		ret = regulator_disable(tpd->reg);
		if (ret != 0)
			pr_err("[focal]Failed to disable VTP: %d\n", ret);
	}
	regulator_put(tpd->reg);

	pr_debug("[focal] TPD shutdown\n");
}



static struct tpd_driver_t tpd_device_driver = {

	.tpd_device_name = "ft5726",
	.tpd_local_init = tpd_local_init,
	.suspend = tpd_suspend,
	.resume = tpd_resume,
	.tpd_have_button = 0,
};

/************************************************************************
* Name: tpd_suspend
* Brief:  called when loaded into kernel
* Input: no
* Output: no
* Return: 0
***********************************************************************/
static int __init tpd_driver_init(void)
{
	/* pr_debug("[focal] MediaTek fts touch panel driver init\n"); */

	if (tpd_driver_add(&tpd_device_driver) < 0)
		pr_err("[focal] MediaTek add fts driver failed\n");

	return 0;
}

/************************************************************************
* Name: tpd_driver_exit
* Brief:  should never be called
* Input: no
* Output: no
* Return: 0
***********************************************************************/
static void __exit tpd_driver_exit(void)
{
	/* pr_debug("[focal] MediaTek fts touch panel driver exit\n"); */
	tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);
