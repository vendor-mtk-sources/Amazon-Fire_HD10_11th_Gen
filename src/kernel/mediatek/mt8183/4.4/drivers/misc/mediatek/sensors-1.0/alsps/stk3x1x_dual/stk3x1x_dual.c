/*
 * Author: EdwardCW Lin <EdwardCW_Lin@compal.com>
 * Copyright 2020 Compal Inc.
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/timekeeping.h>
#include <linux/time.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/atomic.h>
#include <linux/version.h>
#include <linux/fs.h>
#include <linux/wakelock.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <hwmsensor.h>
#include "cust_alsps.h"
#include "alsps.h"
#include "stk3x1x.h"

#define DRIVER_VERSION  "3.9.2.1 20200521"
#define MTK_AUTO_DETECT_ALSPS
/******************************************************************************
 * configuration
*******************************************************************************/

/*----------------------------------------------------------------------------*/
#define stk3x1x_DEV_NAME "stk3x1x"
/*----------------------------------------------------------------------------*/
#define APS_TAG                "[ALS/PS] "
#define APS_FUN(f)             pr_debug(APS_TAG"%s\n", __func__)
#define APS_ERR(fmt, args...)  pr_err(APS_TAG"%s %d : "fmt, __func__, __LINE__, ##args)
#define APS_DBG(fmt, args...)  pr_debug(APS_TAG fmt, ##args)
#define APS_LOG(fmt, args...)  pr_info(APS_TAG fmt, ##args)
/******************************************************************************
 * extern functions
*******************************************************************************/

#define STK_IRC_MAX_ALS_CODE        20000
#define STK_IRC_MIN_ALS_CODE        25
#define STK_IRC_MIN_IR_CODE         50
#define STK_IRC_ALS_DENOMI          2
#define STK_IRC_ALS_NUMERA          5
#define STK_IRC_ALS_CORREC          748

#define STK_IRS_IT_REDUCE           3
#define STK_ALS_READ_IRS_IT_REDUCE  5
#define STK_ALS_THRESHOLD           30

#define STK3310SA_PID               0x17
#define STK3311SA_PID               0x1E
#define STK3311WV_PID               0x1D
#define STK3311X_PID                0x12
#define STK33119_PID                0x11

#define ALS_CALIBRATION_LUX_HIGH    400
#define ALS_CALIBRATION_LUX_LOW     20
#define MASTER                      0
#define SLAVE                       1
#define STK_IRS                     1

#define IDME_OF_ALSCAL              "/idme/alscal"
#define IDME_OF_CG_COLOR            "/idme/tp_cg_color"
#define IDME_MAX_LEN                14
#define BLACK                       1
#define WHITE                       2
#define ALS_NOT_EXSIT               "als not exsit!\n"

#define IR_INTERVAL_500MS           500000000

static unsigned int  position_1st = 1;
static unsigned int  position_2nd = 1;

static struct timespec now;
static struct timespec last_time;
static struct timespec stk_diff;
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id stk3x1x_i2c_id[] = {
	{"stk3x1x", 0},
	{},
};
/*----------------------------------------------------------------------------*/
static int stk3x1x_i2c_probe(struct i2c_client *client,
				const struct i2c_device_id *id);
static int stk3x1x_i2c_remove(struct i2c_client *client);
/*----------------------------------------------------------------------------*/

#ifndef C_I2C_FIFO_SIZE
#define C_I2C_FIFO_SIZE		8
#endif
static DEFINE_MUTEX(STK3X1X_i2c_mutex);
static int stk3x1x_init_flag = -1;	/* 0 <==> OK -1 <==> fail */
static int stk3x1x_local_init(void);
static int stk3x1x_local_uninit(void);
static struct alsps_init_info stk3x1x_init_info = {
	.name = "stk3x1x",
	.init = stk3x1x_local_init,
	.uninit = stk3x1x_local_uninit,
};

struct platform_device *alspsPltFmDev;
/*----------------------------------------------------------------------------*/
static int als_get_data(int *value, int *status);
/*----------------------------------------------------------------------------*/
typedef enum {
	STK_TRC_DEBUG = 0x0001,
	STK_TRC_ALS_DATA = 0x0002,
} STK_TRC;
/*----------------------------------------------------------------------------*/
typedef enum {
	STK_BIT_ALS = 1,
	STK_BIT_PS = 2,
} STK_BIT;
/*----------------------------------------------------------------------------*/
struct stk3x1x_i2c_addr {
	/*define a series of i2c slave address*/
	u8 state;		/* enable/disable state */
	u8 psctrl;		/* PS control */
	u8 alsctrl;		/* ALS control */
	u8 ledctrl;		/* LED control */
	u8 intmode;		/* INT mode */
	u8 wait;		/* wait time */
	u8 thdh1_ps;		/* PS INT threshold high 1 */
	u8 thdh2_ps;		/* PS INT threshold high 2 */
	u8 thdl1_ps;		/* PS INT threshold low 1 */
	u8 thdl2_ps;		/* PS INT threshold low 2 */
	u8 thdh1_als;		/* ALS INT threshold high 1 */
	u8 thdh2_als;		/* ALS INT threshold high 2 */
	u8 thdl1_als;		/* ALS INT threshold low 1 */
	u8 thdl2_als;		/* ALS INT threshold low 2 */
	u8 flag;		/* int flag */
	u8 data1_ps;		/* ps data1 */
	u8 data2_ps;		/* ps data2 */
	u8 data1_als;		/* als data1 */
	u8 data2_als;		/* als data2 */
	u8 data1_offset;	/* offset data1 */
	u8 data2_offset;	/* offset data2 */
	u8 data1_ir;		/* ir data1 */
	u8 data2_ir;		/* ir data2 */
	u8 soft_reset;		/* software reset */
};
/*----------------------------------------------------------------------------*/
struct stk3x1x_priv {
	struct alsps_hw	 *hw;
	struct i2c_client *client;
	struct delayed_work eint_work;

	struct device_node *irq_node;
	atomic_t state_val;

	/*data*/
	u16 als;
	u8 _align;

	ulong enable;
	/*enable mask*/
	ulong pending_intr;
	/*pending interrupt*/

	bool first_boot;
	uint16_t ir_code;
	uint16_t als_correct_factor;
	bool als_last;
	bool re_enable_als;

	uint8_t pid;
	uint8_t	p_wv_r_bd_with_co;
	uint32_t als_code_last;
	uint16_t als_data_index;
	uint8_t ps_distance_last;
	uint8_t	p_1x_r_bd_with_co;
	uint8_t	p_19_r_bc;

	uint32_t als_cal_low;
	uint32_t als_cal_high;
	int als_cali_compen_20;
	int als_cali_compen_400;

	int sensor_num;
};
/*----------------------------------------------------------------------------*/
struct dual_als {
	u_int8_t dual_als_enable;
	int als_count;
	int tp_cg_color;
	/*i2c address group*/
	struct stk3x1x_i2c_addr addr;
	struct stk3x1x_priv *als[2];

	/*misc*/
	atomic_t trace;
	atomic_t i2c_retry;
	atomic_t als_suspend;
	atomic_t als_debounce;	/*debounce time after enabling als*/
	atomic_t als_deb_on;	/*indicates if the debounce is on*/
	atomic_t als_deb_end;	/*the jiffies representing the end of debounce*/

	atomic_t alsctrl_val;
	u8 wait_val;
	u8 ledctrl_val;
	u8 int_val;
} stk3x1x_dual;
/*----------------------------------------------------------------------------*/
#ifdef CONFIG_PM_SLEEP
static int stk3x1x_suspend(struct device *dev);
static int stk3x1x_resume(struct device *dev);
#endif
/*----------------------------------------------------------------------------*/
#ifdef CONFIG_OF
static const struct of_device_id alsps_of_match[] = {
	{.compatible = "mediatek,alsps"},
	{},
};
#endif
#ifdef CONFIG_PM_SLEEP
static const struct dev_pm_ops stk3x1x_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(stk3x1x_suspend, stk3x1x_resume)
};
#endif

static struct i2c_driver stk3x1x_i2c_driver = {
	.probe = stk3x1x_i2c_probe,
	.remove = stk3x1x_i2c_remove,
	.id_table = stk3x1x_i2c_id,
	.driver = {
		.name = stk3x1x_DEV_NAME,
#ifdef CONFIG_PM_SLEEP
		.pm = &stk3x1x_pm_ops,
#endif

#ifdef CONFIG_OF
		.of_match_table = alsps_of_match,
#endif
	},
};


static int stk3x1x_read_als(struct i2c_client *client, u16 *data);
static int32_t stk3x1x_get_ir_value(struct stk3x1x_priv *obj);
static int get_cali_compensation(struct device_node *node, char *name);
static int stk3x1x_init_client(struct i2c_client *client);
static int als_get_lux(int *value);
struct wake_lock mps_lock;

static DEFINE_MUTEX(run_cali_mutex);
/*----------------------------------------------------------------------------*/
int stk3x1x_get_addr(struct stk3x1x_i2c_addr *addr)
{

	if (!addr)
		return -EFAULT;

	addr->state = STK_STATE_REG;
	addr->psctrl = STK_PSCTRL_REG;
	addr->alsctrl = STK_ALSCTRL_REG;
	addr->ledctrl = STK_LEDCTRL_REG;
	addr->intmode = STK_INT_REG;
	addr->wait = STK_WAIT_REG;
	addr->thdh1_ps = STK_THDH1_PS_REG;
	addr->thdh2_ps = STK_THDH2_PS_REG;
	addr->thdl1_ps = STK_THDL1_PS_REG;
	addr->thdl2_ps = STK_THDL2_PS_REG;
	addr->thdh1_als = STK_THDH1_ALS_REG;
	addr->thdh2_als = STK_THDH2_ALS_REG;
	addr->thdl1_als = STK_THDL1_ALS_REG;
	addr->thdl2_als = STK_THDL2_ALS_REG;
	addr->flag = STK_FLAG_REG;
	addr->data1_ps = STK_DATA1_PS_REG;
	addr->data2_ps = STK_DATA2_PS_REG;
	addr->data1_als = STK_DATA1_ALS_REG;
	addr->data2_als = STK_DATA2_ALS_REG;
	addr->data1_offset = STK_DATA1_OFFSET_REG;
	addr->data2_offset = STK_DATA2_OFFSET_REG;
	addr->data1_ir = STK_DATA1_IR_REG;
	addr->data2_ir = STK_DATA2_IR_REG;
	addr->soft_reset = STK_SW_RESET_REG;
	return 0;
}
/*----------------------------------------------------------------------------*/
int stk3x1x_i2c_read_block(struct i2c_client *client,
				u8 addr, u8 *data, u8 len)
{
	int err;


	if (!client)
		return -EINVAL;
	else if (len > C_I2C_FIFO_SIZE) {
		APS_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}

	mutex_lock(&STK3X1X_i2c_mutex);
	err = i2c_smbus_read_i2c_block_data(client, addr, len, data);
	mutex_unlock(&STK3X1X_i2c_mutex);

	if (err < 0) {
		APS_ERR("i2c_smbus_read_i2c_block_data error: (%d %p %d) %d\n",
			addr, data, len, err);
		err = -EIO;
	} else {
		err = 0;/*no error*/
	}

	return err;
}
/*----------------------------------------------------------------------------*/
int stk3x1x_get_timing(void)
{
	return 200;
}

/*----------------------------------------------------------------------------*/
int stk3x1x_register_read(struct i2c_client *client, u16 addr,
						u8 *buf, int count)
{
	int ret = 0, retry = 0;
	int trc = atomic_read(&stk3x1x_dual.trace);
	int max_try = atomic_read(&stk3x1x_dual.i2c_retry);

	while (retry++ < max_try) {
		ret = stk3x1x_i2c_read_block(client, addr, buf, count);

		if (ret == 0)
			break;

		usleep_range(100, 110);
	}

	if (unlikely(trc)) {
		if ((retry != 1) && (trc & STK_TRC_DEBUG))
			APS_DBG("(recv) %d/%d\n", retry - 1, max_try);
	}

	/*
	 * If everything went ok (i.e. 1 msg transmitted), return #bytes
	 * transmitted, else error code.
	 */
	return (ret == 0) ? count : ret;
}
/*----------------------------------------------------------------------------*/
int stk3x1x_register_write(struct i2c_client *client, u16 addr,
						u8 *buf, int count)
{
	int ret = 0, retry = 0;
	int trc = atomic_read(&stk3x1x_dual.trace);
	int max_try = atomic_read(&stk3x1x_dual.i2c_retry);

	while (retry++ < max_try) {
		ret = i2c_smbus_write_i2c_block_data(client, addr, count, buf);

		if (ret == 0)
			break;

		usleep_range(100, 110);
	}

	if (unlikely(trc)) {
		if ((retry != 1) && (trc & STK_TRC_DEBUG))
			APS_DBG("(send) %d/%d\n", retry - 1, max_try);
	}

	/* If everything went ok (i.e. 1 msg transmitted) */
	/* return bytes transmitted, else error code. */
	return (ret == 0) ? count : ret;
}
/*----------------------------------------------------------------------------*/
static int stk3x1x_otp_read_byte_data(struct i2c_client *client, u8 command)
{
	int ret;
	int value;
	u8 data;

	data = 0x2;
	ret = stk3x1x_register_write(client, 0x0, &data, 1);
	if (ret < 0) {
		APS_ERR("write 0x0 = %d\n", ret);
		return -EFAULT;
	}

	data = command;
	ret = stk3x1x_register_write(client, 0x90, &data, 1);
	if (ret < 0) {
		APS_ERR("write 0x90 = %d\n", ret);
		return -EFAULT;
	}

	data = 0x82;
	ret = stk3x1x_register_write(client, 0x92, &data, 1);
	if (ret < 0) {
		APS_ERR("write 0x0 = %d\n", ret);
		return -EFAULT;
	}

	usleep_range(2000, 4000);
	ret = stk3x1x_register_read(client, 0x91, &data, 1);
	if (ret < 0) {
		APS_ERR("error: %d\n", ret);
		return -EFAULT;
	}

	value = data;
	APS_DBG("%s: 0x%x=0x%x\n", __func__, command, value);
	data = 0x0;
	ret = stk3x1x_register_write(client, 0x0, &data, 1);
	if (ret < 0) {
		APS_ERR("write 0x0 = %d\n", ret);
		return -EFAULT;
	}

	return value;
}
/*----------------------------------------------------------------------------*/
int stk3x1x_write_led(struct i2c_client *client, u8 data)
{
	int ret = 0;

	ret = stk3x1x_register_write(client, stk3x1x_dual.addr.ledctrl,
								&data,
								1);
	if (ret < 0) {
		APS_ERR("write led = %d\n", ret);
		return -EFAULT;
	}

	return 0;
}
uint16_t stk_als_ir_formula_black(uint16_t ir_code, u32 als_data)
{
	if (ir_code == 0 || (als_data / ir_code) * 100 > 40)
		return 737;
	else if ((als_data / ir_code) * 100 > 15)
		return 726;
	else
		return 658;
}

uint16_t stk_als_ir_formula_white(uint16_t ir_code, u32 als_data)
{
	if (ir_code == 0 || (als_data / ir_code) * 100 > 40)
		return 2072;
	else if ((als_data / ir_code) * 100 > 15)
		return 2116;
	else
		return 1943;
}
/*----------------------------------------------------------------------------*/
int stk_als_ir_get_formula(struct stk3x1x_priv *obj, u32 als_data)
{
	if (stk3x1x_dual.tp_cg_color == WHITE)
		obj->als_correct_factor =
			stk_als_ir_formula_white(obj->ir_code,
					als_data);
	else if (stk3x1x_dual.tp_cg_color == BLACK)
		obj->als_correct_factor =
			stk_als_ir_formula_black(obj->ir_code,
					als_data);
	else {
		APS_DBG("tp_cg_color is 0!!! use black as default\n");
		obj->als_correct_factor =
			stk_als_ir_formula_black(obj->ir_code,
					als_data);
	}

	APS_DBG("%s: bus=%d, als_num=%d, als=%d, ir=%d, als_correct_factor=%d",
				__func__, obj->hw->i2c_num,
				obj->sensor_num,
				als_data,
				obj->ir_code,
				obj->als_correct_factor);

	return 0;
}

/*----------------------------------------------------------------------------*/
int stk3x1x_get_als_adc(struct i2c_client *client)
{
	int ret = 0;
	u8 buf[2];
	u32 als_data;
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);

	ret = stk3x1x_register_read(client, stk3x1x_dual.addr.data1_als,
			buf, 0x02);
	if (ret < 0) {
		APS_ERR("error: %d\n", ret);
		return -EFAULT;
	}

	als_data = (buf[0] << 8) | (buf[1]);

	if (obj->p_1x_r_bd_with_co == 0x07 || obj->p_19_r_bc == 0x03) {
		als_data = als_data * 16 / 10;

		if (als_data > 65535) {
			APS_LOG("65535=%d\n", als_data);
			als_data = 65535;
		}

	}

	obj->als_code_last = als_data;

	return als_data;
}

#define STK_ALS_THRESHOLD	30

int stk3x1x_read_als(struct i2c_client *client, u16 *data)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);
	u32 als_data;

	if (!client)
		return -EINVAL;

	if (atomic_read(&stk3x1x_dual.als_suspend)) {
		als_data = 0;
		goto out;
	}

	als_data = stk3x1x_get_als_adc(client);

	stk3x1x_get_ir_value(obj);

	stk_als_ir_get_formula(obj, als_data);
	als_data = ((als_data * obj->als_correct_factor) + 500) / 1000;

out:
	*data = (u16)als_data;

	if (atomic_read(&stk3x1x_dual.trace) & STK_TRC_ALS_DATA)
		APS_DBG("ALS: 0x%04X\n", (u32)(*data));

	return 0;
}
/*----------------------------------------------------------------------------*/
int stk3x1x_write_als(struct i2c_client *client, u8 data)
{
	int ret = 0;

	ret = stk3x1x_register_write(client, stk3x1x_dual.addr.alsctrl,
			&data, 1);
	if (ret < 0) {
		APS_ERR("write als = %d\n", ret);
		return -EFAULT;
	}

	return 0;
}
/*----------------------------------------------------------------------------*/
int stk3x1x_read_state(struct i2c_client *client, u8 *data)
{
	int ret = 0;
	u8 buf;

	if (!client)
		return -EINVAL;

	ret = stk3x1x_register_read(client, STK_STATE_REG, &buf, 0x01);
	if (ret < 0) {
		APS_ERR("error: %d\n", ret);
		return -EFAULT;
	}

	*data = buf;

	return 0;
}
/*----------------------------------------------------------------------------*/
int stk3x1x_read_flag(struct i2c_client *client, u8 *data)
{
	int ret = 0;
	u8 buf;

	if (!client)
		return -EINVAL;

	ret = stk3x1x_register_read(client, stk3x1x_dual.addr.flag,
			&buf, 0x01);
	if (ret < 0) {
		APS_ERR("error: %d\n", ret);
		return -EFAULT;
	}
	*data = buf;

	if (atomic_read(&stk3x1x_dual.trace) & STK_TRC_ALS_DATA)
		APS_DBG("PS NF flag: 0x%04X\n", (u32)(*data));

	return 0;
}
/*----------------------------------------------------------------------------*/
int stk3x1x_read_id(struct i2c_client *client)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);
	int ret = 0;
	u8 buf[2];
	u8 pid_msb;
	int otp25 = 0;

	if (!client)
		return -EINVAL;

	obj->p_wv_r_bd_with_co = 0;
	obj->p_1x_r_bd_with_co = 0;
	obj->p_19_r_bc = 0;
	ret = stk3x1x_register_read(client, STK_PDT_ID_REG, buf, 0x02);

	if (ret < 0) {
		APS_ERR("error: %d\n", ret);
		return -EFAULT;
	}

	obj->pid = buf[0];
	APS_DBG("%s: PID=0x%x, VID=0x%x\n", __func__, buf[0], buf[1]);

	if (obj->pid == STK3310SA_PID || obj->pid == STK3311SA_PID)
		stk3x1x_dual.ledctrl_val &= 0x3F;

	if (buf[0] == STK3311WV_PID)
		obj->p_wv_r_bd_with_co |= 0x04;
	else if (buf[0] == STK3311X_PID)
		obj->p_1x_r_bd_with_co |= 0x04;
	else if (buf[0] == STK33119_PID)
		obj->p_19_r_bc |= 0x02;

	if (buf[1] == 0xC3) {
		obj->p_wv_r_bd_with_co |= 0x02;
		obj->p_1x_r_bd_with_co |= 0x02;
	} else if (buf[1] == 0xC2) {
		obj->p_19_r_bc |= 0x01;
	}

	ret = stk3x1x_otp_read_byte_data(client, 0x25);

	if (ret < 0)
		return ret;

	otp25 = ret;

	if (otp25 & 0x80)
		obj->p_wv_r_bd_with_co |= 0x01;

	APS_DBG("%s: p_wv_r_bd_with_co = 0x%x\n",
		__func__, obj->p_wv_r_bd_with_co);

	if (otp25 & 0x40)
		obj->p_1x_r_bd_with_co |= 0x01;

	APS_DBG("%s: p_1x_r_bd_with_co = 0x%x\n",
		__func__, obj->p_1x_r_bd_with_co);
	APS_DBG("%s: p_19_r_bc = 0x%x\n",
		__func__, obj->p_19_r_bc);

	if (buf[0] == 0) {
		APS_ERR("PID=0x0, please make sure the chip is stk3x1x!\n");
		return -2;
	}

	pid_msb = buf[0] & 0xF0;

	switch (pid_msb) {
	case 0x10:
	case 0x20:
	case 0x30:
		return 0;

	default:
		APS_ERR("invalid PID(%#x)\n", buf[0]);
		return -1;
	}

	return 0;
}
/*----------------------------------------------------------------------------*/
int stk3x1x_write_ps(struct i2c_client *client, u8 data)
{
	int ret = 0;

	ret = stk3x1x_register_write(client, stk3x1x_dual.addr.psctrl,
			&data, 1);
	if (ret < 0) {
		APS_ERR("write ps = %d\n", ret);
		return -EFAULT;
	}

	return 0;
}

/*----------------------------------------------------------------------------*/
int stk3x1x_write_wait(struct i2c_client *client, u8 data)
{
	int ret = 0;

	ret = stk3x1x_register_write(client, stk3x1x_dual.addr.wait,
			&data, 1);
	if (ret < 0) {
		APS_ERR("write wait = %d\n", ret);
		return -EFAULT;
	}

	return 0;
}

/*----------------------------------------------------------------------------*/
int stk3x1x_write_int(struct i2c_client *client, u8 data)
{
	int ret = 0;

	ret = stk3x1x_register_write(client, stk3x1x_dual.addr.intmode,
			&data, 1);
	if (ret < 0) {
		APS_ERR("write intmode = %d\n", ret);
		return -EFAULT;
	}

	return 0;
}
/*----------------------------------------------------------------------------*/
int stk3x1x_write_state(struct i2c_client *client, u8 data)
{
	int ret = 0;

	ret = stk3x1x_register_write(client, stk3x1x_dual.addr.state,
			&data, 1);
	if (ret < 0) {
		APS_ERR("write state = %d\n", ret);
		return -EFAULT;
	}

	return 0;
}
/*----------------------------------------------------------------------------*/
int stk3x1x_write_flag(struct i2c_client *client, u8 data)
{
	int ret = 0;

	ret = stk3x1x_register_write(client, stk3x1x_dual.addr.flag,
			&data, 1);
	if (ret < 0) {
		APS_ERR("write ps = %d\n", ret);
		return -EFAULT;
	}

	return 0;
}
/*----------------------------------------------------------------------------*/
int stk3x1x_write_sw_reset(struct i2c_client *client)
{
	u8 buf = 0, r_buf = 0;
	int ret = 0;

	APS_LOG("%s: In\n", __func__);
	buf = 0x7F;
	ret = stk3x1x_register_write(client, stk3x1x_dual.addr.wait,
				(char *)&buf, sizeof(buf));

	if (ret < 0) {
		APS_ERR("i2c write test error = %d\n", ret);
		return -EFAULT;
	}

	ret = stk3x1x_register_read(client, stk3x1x_dual.addr.wait,
			&r_buf, 1);

	if (ret < 0) {
		APS_ERR("i2c read test error = %d\n", ret);
		return -EFAULT;
	}

	if (buf != r_buf) {
		APS_ERR("read-back is not the same, write=0x%x, read=0x%x\n",
			buf, r_buf);
		return -EIO;
	}

	buf = 0;
	ret = stk3x1x_register_write(client, stk3x1x_dual.addr.soft_reset,
				(char *)&buf, sizeof(buf));
	if (ret < 0) {
		APS_ERR("write software reset error = %d\n", ret);
		return -EFAULT;
	}

	msleep(20);
	return 0;
}
/*----------------------------------------------------------------------------*/
int stk3x1x_write_als_high_thd(struct i2c_client *client, u16 thd)
{
	u8 buf[2];
	int ret = 0;

	buf[0] = (u8)((0xFF00 & thd) >> 8);
	buf[1] = (u8)(0x00FF & thd);
	ret = stk3x1x_register_write(client, stk3x1x_dual.addr.thdh1_als,
			&buf[0], 1);
	if (ret < 0) {
		APS_ERR("WARNING: %s: %d\n", __func__, ret);
		return -EFAULT;
	}

	ret = stk3x1x_register_write(client, stk3x1x_dual.addr.thdh2_als,
			&(buf[1]), 1);
	if (ret < 0) {
		APS_ERR("WARNING: %s: %d\n", __func__, ret);
		return -EFAULT;
	}

	return 0;
}
/*----------------------------------------------------------------------------*/
int stk3x1x_write_als_low_thd(struct i2c_client *client, u16 thd)
{
	u8 buf[2];
	int ret = 0;

	buf[0] = (u8)((0xFF00 & thd) >> 8);
	buf[1] = (u8)(0x00FF & thd);
	ret = stk3x1x_register_write(client, stk3x1x_dual.addr.thdl1_als,
			&buf[0], 1);
	if (ret < 0) {
		APS_ERR("WARNING: %s: %d\n", __func__, ret);
		return -EFAULT;
	}

	ret = stk3x1x_register_write(client, stk3x1x_dual.addr.thdl2_als,
			&(buf[1]), 1);
	if (ret < 0) {
		APS_ERR("WARNING: %s: %d\n", __func__, ret);
		return -EFAULT;
	}

	return 0;
}
/*----------------------------------------------------------------------------*/
static int stk3x1x_enable_als(struct i2c_client *client, int enable)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);
	int err, cur = 0, old = atomic_read(&obj->state_val);
	int trc = atomic_read(&stk3x1x_dual.trace);

	APS_LOG("%s: i2cbus=%d als_num=%d, enable als=%d\n",
			__func__, obj->hw->i2c_num, obj->sensor_num, enable);

	cur = old & (~(STK_STATE_EN_ALS_MASK | STK_STATE_EN_WAIT_MASK));

	if (enable)
		cur |= STK_STATE_EN_ALS_MASK | STK_STATE_EN_IRS_MASK;
	else if (old & STK_STATE_EN_PS_MASK)
		cur |= STK_STATE_EN_WAIT_MASK;

	if (trc & STK_TRC_DEBUG)
		APS_DBG("%s: %08X, %08X, %d\n", __func__, cur, old, enable);

	if (0 == (cur ^ old))
		return 0;

	if (enable && obj->hw->polling_mode_als == 0) {
		stk3x1x_write_als_high_thd(client, 0x0);
		stk3x1x_write_als_low_thd(client, 0xFFFF);
	}

	err = stk3x1x_write_state(client, cur);
	if (err < 0)
		return err;

	atomic_set(&obj->state_val, cur);
	if (enable) {
		obj->als_last = 0;

		if (obj->hw->polling_mode_als) {
			atomic_set(&stk3x1x_dual.als_deb_on, 1);
			atomic_set(&stk3x1x_dual.als_deb_end, jiffies +
			atomic_read(&stk3x1x_dual.als_debounce)*HZ / 1000);
		} else {
			schedule_delayed_work(&obj->eint_work, 220 * HZ / 1000);
		}
	}

	obj->als_data_index = 0;

	if (trc & STK_TRC_DEBUG)
		APS_DBG("enable als (%d)\n", enable);

	return err;
}
/*----------------------------------------------------------------------------*/
static int stk3x1x_clear_intr(struct i2c_client *client,
				u8 status, u8 disable_flag)
{
	int err = 0;

	status = status | (STK_FLG_ALSINT_MASK | STK_FLG_PSINT_MASK
			| STK_FLG_OUI_MASK | STK_FLG_IR_RDY_MASK);
	status &= (~disable_flag);

	err = stk3x1x_write_flag(client, status);
	if (err)
		APS_ERR("stk3x1x_write_flag failed, err=%d\n", err);

	return err;
}
/******************************************************************************
 * Function Configuration
******************************************************************************/
uint32_t als_calibration(struct stk3x1x_priv *obj, uint32_t alscode)
{
	int alslux = 0, cal_high = 0, cal_low = 0;
	int def_high = 0, def_low = 0;

	if (alscode == 0)
		return 0;

	if (alscode > 56000)
		alscode = 56000;

	alslux = (int)alscode * 10;
	cal_high = (int)obj->als_cal_high * 10;
	cal_low = (int)obj->als_cal_low * 10;
	def_high = (int)ALS_CALIBRATION_LUX_HIGH * 10;
	def_low = (int)ALS_CALIBRATION_LUX_LOW * 10;

	/*
	* Lux =
	* 400 - 20 * (ALS ADC - als cal low) / (als cal high - als cal low) +20
	* Formula for ADC to LUX
	**/
	alslux = (def_high - def_low) *
			(alslux - cal_low) / (cal_high - cal_low) +
			(int)def_low;

	alslux = (alslux + 5) / 10;

	return (uint32_t)alslux;
}
/*----------------------------------------------------------------------------*/
static int als_cali_set(char *buf)
{
	struct stk3x1x_priv *obj;
	char *alscal;
	char *token;
	const char delim[] = ",";
	/*default value format is 1st low, 1st high, 2nd low, 2nd high*/
	int dual_cal[4] = {20, 400, 20, 400};
	int i = 0;
	int err = 0;

	alscal = kmalloc(IDME_MAX_LEN, GFP_KERNEL);

	if (!alscal)
		goto error;

	strscpy(alscal, buf, IDME_MAX_LEN);

	for (token = strsep(&alscal, delim); token != NULL; token = strsep(&alscal, delim)) {

		APS_DBG("strsep=%s dest=%s\n", token, alscal);
		err = kstrtoint(token, 10, &dual_cal[i++]);

		if (err)
			goto buf_error;
	}

	obj = stk3x1x_dual.als[position_1st];

	if (obj) {
		obj->als_cal_low = ((dual_cal[0] *
				obj->als_cali_compen_20 + 500) / 1000);
		obj->als_cal_high = ((dual_cal[1] *
				obj->als_cali_compen_400 + 500) / 1000);

		APS_LOG("cal values bus=%d 1st=low:%d high:%d\n",
			stk3x1x_dual.als[position_1st]->hw->i2c_num,
			stk3x1x_dual.als[position_1st]->als_cal_low,
			stk3x1x_dual.als[position_1st]->als_cal_high);
	}

	obj = stk3x1x_dual.als[position_2nd];

	if (obj) {
		obj->als_cal_low = ((dual_cal[2] *
				obj->als_cali_compen_20 + 500) / 1000);
		obj->als_cal_high = ((dual_cal[3] *
				obj->als_cali_compen_400 + 500) / 1000);

		APS_LOG("cal values bus=%d 2nd=low:%d high:%d\n",
			stk3x1x_dual.als[position_2nd]->hw->i2c_num,
			stk3x1x_dual.als[position_2nd]->als_cal_low,
			stk3x1x_dual.als[position_2nd]->als_cal_high);
	}

buf_error:
	kfree(alscal);

error:
	return err;
}
/*----------------------------------------------------------------------------*/
static int stk3x1x_check_id(struct i2c_client *client)
{
	int err;

	err = stk3x1x_write_sw_reset(client);
	if (err) {
		APS_ERR("software reset error, err=%d", err);
		return err;
	}

	err = stk3x1x_read_id(client);
	if (err) {
		APS_ERR("stk3x1x_read_id error, err=%d", err);
		return err;
	}

	return 0;
}
/*----------------------------------------------------------------------------*/
static int stk3x1x_init_client(struct i2c_client *client)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);
	int err;

	APS_LOG("%s: In\n", __func__);

	err = stk3x1x_write_state(client, atomic_read(&obj->state_val));
	if (err) {
		APS_ERR("write stete error: %d\n", err);
		return err;
	}

	err = stk3x1x_write_als(client, atomic_read(&stk3x1x_dual.alsctrl_val));
	if (err) {
		APS_ERR("write als error: %d\n", err);
		return err;
	}

	err = stk3x1x_write_wait(client, stk3x1x_dual.wait_val);
	if (err) {
		APS_ERR("write wait error: %d\n", err);
		return err;
	}

	obj->re_enable_als = false;
	obj->als_code_last = 100;

	get_monotonic_boottime(&last_time);

	return 0;
}

/*----------------------------------------------------------------------------*/
static inline uint32_t stk3x1x_get_als_reading_avg(struct stk3x1x_priv *obj,
		int sSampleNo)
{
	int res;
	uint16_t ALSData = 0;
	uint16_t DataCount = 0;
	uint32_t sAveAlsData = 0;

	while (DataCount < sSampleNo) {
		msleep(120);

		res = stk3x1x_read_als(obj->client, &obj->als);
		if (res)
			APS_ERR("get_als_reading_avg: stk3x1x_read_als!!\n");

		ALSData = obj->als;
		APS_DBG("%s: [STK]als code = %d\n", __func__, ALSData);
		sAveAlsData += ALSData;
		DataCount++;
	}
	sAveAlsData /= sSampleNo;
	return sAveAlsData;
}
/******************************************************************************
 * Sysfs attributes
*******************************************************************************/
static ssize_t stk3x1x_show_als_cali_1st(struct device_driver *ddri, char *buf)
{
	int als_adc_cali;
	struct stk3x1x_priv *obj = stk3x1x_dual.als[position_1st];

	if (!obj)
		return scnprintf(buf, PAGE_SIZE, ALS_NOT_EXSIT);

	als_adc_cali = stk3x1x_get_als_reading_avg(obj, 5);

	return scnprintf(buf, PAGE_SIZE,
		"adc_cali=%d\n",
		als_adc_cali);
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_show_als_cali_2nd(struct device_driver *ddri, char *buf)
{
	int als_adc_cali;
	struct stk3x1x_priv *obj = stk3x1x_dual.als[position_2nd];

	if (!obj)
		return scnprintf(buf, PAGE_SIZE, ALS_NOT_EXSIT);

	als_adc_cali = stk3x1x_get_als_reading_avg(obj, 5);

	return scnprintf(buf, PAGE_SIZE,
		"adc_cali=%d\n",
		 als_adc_cali);
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_show_config(struct device_driver *ddri, char *buf)
{
	ssize_t res = 0;
	struct stk3x1x_priv *obj;

	for (int i = 0; i < stk3x1x_dual.als_count; i++) {
		obj = stk3x1x_dual.als[i];
		if (!obj) {
			APS_ERR("stk3x1x_obj is null!!\n");
			return -1;
		}

		res += scnprintf(buf + res, PAGE_SIZE - res,
				"(%d %d %d %d %d %d)\n",
				atomic_read(&stk3x1x_dual.i2c_retry),
				atomic_read(&stk3x1x_dual.als_debounce));
	}

	return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_show_trace(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct stk3x1x_priv *obj;

	for (int i = 0; i < stk3x1x_dual.als_count; i++) {
		obj = stk3x1x_dual.als[i];
		if (!obj) {
			APS_ERR("stk3x1x_obj is null!!\n");
			return -1;
		}

		res = scnprintf(buf, PAGE_SIZE,
				"0x%04X\n",
				atomic_read(&stk3x1x_dual.trace));
	}
	return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_store_trace(struct device_driver *ddri,
					const char *buf, size_t count)
{
	int trace;

	if (sysfs_streq(buf, "1")) {
		trace = 1;
	} else if (sysfs_streq(buf, "2")) {
		trace = 2;
	} else if (sysfs_streq(buf, "0")) {
		trace = 0;
	} else {
		APS_ERR("%s, invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}

	atomic_set(&stk3x1x_dual.trace, trace);
	APS_LOG("set trace =%d, count = %d\n", trace, (int)count);

	return count;
}
/*----------------------------------------------------------------------------*/
static uint32_t stk3x1x_show_ir(struct stk3x1x_priv *obj)
{
	if (!obj) {
		APS_ERR("stk3x1x_obj is null!!\n");
		return -1;
	}

	return obj->ir_code;
}

static ssize_t show_ir_1st(struct device_driver *ddri, char *buf)
{
	int32_t reading;
	struct stk3x1x_priv *obj = stk3x1x_dual.als[position_1st];

	if (!obj)
		return scnprintf(buf, PAGE_SIZE, ALS_NOT_EXSIT);

	reading = stk3x1x_show_ir(obj);

	if (reading > 0)
		return scnprintf(buf, PAGE_SIZE, "%d\n", reading);
	else
		return scnprintf(buf, PAGE_SIZE, "ERROR: %d\n", reading);

}

static ssize_t show_ir_2nd(struct device_driver *ddri, char *buf)
{
	int32_t reading;
	struct stk3x1x_priv *obj = stk3x1x_dual.als[position_2nd];

	if (!obj)
		return scnprintf(buf, PAGE_SIZE, ALS_NOT_EXSIT);

	reading = stk3x1x_show_ir(obj);

	if (reading > 0)
		return scnprintf(buf, PAGE_SIZE, "%d\n", obj->ir_code);
	else
		return scnprintf(buf, PAGE_SIZE, "ERROR: %d\n", reading);

}

/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_show_als(struct stk3x1x_priv *obj)
{
	int res;

	if (!obj) {
		APS_ERR("stk3x1x_obj is null!!\n");
		return -1;
	}

	res = stk3x1x_read_als(obj->client, &obj->als);

	return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_als_1st(struct device_driver *ddri, char *buf)
{
	int res = -1;
	struct stk3x1x_priv *obj = stk3x1x_dual.als[position_1st];

	if (!obj)
		return scnprintf(buf, PAGE_SIZE, ALS_NOT_EXSIT);

	res = stk3x1x_show_als(obj);

	if (res)
		return scnprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
	else
		return scnprintf(buf, PAGE_SIZE,
					"%d\n", obj->als);

	return scnprintf(buf, PAGE_SIZE, "%d\n", obj->als_code_last);

}
static ssize_t show_als_2nd(struct device_driver *ddri, char *buf)
{
	int res = 0;
	struct stk3x1x_priv *obj = stk3x1x_dual.als[position_2nd];

	if (!obj)
		return scnprintf(buf, PAGE_SIZE, ALS_NOT_EXSIT);

	res = stk3x1x_show_als(obj);

	if (res)
		return scnprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
	else
		return scnprintf(buf, PAGE_SIZE,
					"%d\n", obj->als);

	return scnprintf(buf, PAGE_SIZE, "%d\n", obj->als_code_last);
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_show_allreg(struct stk3x1x_priv *stk3x1x_obj, char *buf)
{
	int ret = 0;
	u8 rbuf[0x22];
	int cnt;
	int len = 0;

	memset(rbuf, 0, sizeof(rbuf));

	ret = stk3x1x_register_read(stk3x1x_obj->client, 0, &rbuf[0], 7);

	if (ret < 0) {
		APS_ERR("error: %d\n", ret);
		return -EFAULT;
	}

	ret = stk3x1x_register_read(stk3x1x_obj->client, 7, &rbuf[7], 7);

	if (ret < 0) {
		APS_ERR("error: %d\n", ret);
		return -EFAULT;
	}

	ret = stk3x1x_register_read(stk3x1x_obj->client, 14, &rbuf[14], 7);

	if (ret < 0) {
		APS_ERR("error: %d\n", ret);
		return -EFAULT;
	}

	ret = stk3x1x_register_read(stk3x1x_obj->client, 21, &rbuf[21], 7);

	if (ret < 0) {
		APS_ERR("error: %d\n", ret);
		return -EFAULT;
	}

	ret = stk3x1x_register_read(stk3x1x_obj->client, 28, &rbuf[28], 4);

	if (ret < 0) {
		APS_ERR("error: %d\n", ret);
		return -EFAULT;
	}

	ret = stk3x1x_register_read(stk3x1x_obj->client,
					STK_PDT_ID_REG, &rbuf[32], 2);

	if (ret < 0) {
		APS_ERR("error: %d\n", ret);
		return -EFAULT;
	}

	for (cnt = 0; cnt < 0x20; cnt++) {
		APS_LOG("reg[0x%x]=0x%x\n", cnt, rbuf[cnt]);
		len += scnprintf(buf + len, PAGE_SIZE - len,
					"reg[0x%x]=0x%x\n", cnt, rbuf[cnt]);
	}

	APS_DBG("reg[0x3E]=0x%x\n", rbuf[cnt]);
	APS_DBG("reg[0x3F]=0x%x\n", rbuf[cnt++]);
	len += scnprintf(buf + len, PAGE_SIZE - len, "[0x3e]=%2X\n[0x3f]=%2X\n",
			 rbuf[cnt - 1], rbuf[cnt]);
	return len;
}

static ssize_t show_allreg_1st(struct device_driver *ddri, char *buf)
{
	int len = 0;
	struct stk3x1x_priv *obj = stk3x1x_dual.als[position_1st];

	if (!obj)
		return scnprintf(buf, PAGE_SIZE, ALS_NOT_EXSIT);

	len = stk3x1x_show_allreg(obj, buf);
	return len;
}

static ssize_t show_allreg_2nd(struct device_driver *ddri, char *buf)
{
	int len = 0;
	struct stk3x1x_priv *obj = stk3x1x_dual.als[position_2nd];

	if (!obj)
		return scnprintf(buf, PAGE_SIZE, ALS_NOT_EXSIT);

	len = stk3x1x_show_allreg(obj, buf);
	return len;
}

/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_show_status(struct stk3x1x_priv *stk3x1x_obj, char *buf)
{
	ssize_t len = 0;
	u8 rbuf[25];
	int ret = 0;

	if (stk3x1x_obj->hw) {
	len += scnprintf(buf + len, PAGE_SIZE - len,
	"CUST: %d, (%d %d) (%02X)(%02X %02X %02X) (%02X %02X %02X %02X)\n",
				stk3x1x_obj->hw->i2c_num,
				stk3x1x_obj->hw->power_id,
				stk3x1x_obj->hw->power_vol,
				stk3x1x_dual.addr.flag,
				stk3x1x_dual.addr.alsctrl,
				stk3x1x_dual.addr.data1_als,
				stk3x1x_dual.addr.data2_als,
				stk3x1x_dual.addr.psctrl,
				stk3x1x_dual.addr.data1_ps,
				stk3x1x_dual.addr.data2_ps,
				stk3x1x_dual.addr.thdh1_ps);
	} else {
		len += scnprintf(buf + len, PAGE_SIZE - len, "CUST: NULL\n");
	}

	len += scnprintf(buf + len, PAGE_SIZE - len,
		"REGS: %02X %02X %02X %02X %02X %02X %02X %02X %02lX %02lX\n",
			atomic_read(&stk3x1x_obj->state_val),
			atomic_read(&stk3x1x_dual.alsctrl_val),
			stk3x1x_dual.ledctrl_val, stk3x1x_dual.int_val,
			stk3x1x_dual.wait_val,
			stk3x1x_obj->enable,
			stk3x1x_obj->pending_intr);

	len += scnprintf(buf + len, PAGE_SIZE - len, "MISC: %d\n",
			 atomic_read(&stk3x1x_dual.als_suspend));
	len += scnprintf(buf + len, PAGE_SIZE - len, "VER.: %s\n",
			 DRIVER_VERSION);
	memset(rbuf, 0, sizeof(rbuf));
	ret = stk3x1x_register_read(stk3x1x_obj->client, 0, &rbuf[0], 7);

	if (ret < 0) {
		APS_ERR("error: %d\n", ret);
		return -EFAULT;
	}

	ret = stk3x1x_register_read(stk3x1x_obj->client, 7, &rbuf[7], 7);

	if (ret < 0) {
		APS_ERR("error: %d\n", ret);
		return -EFAULT;
	}

	ret = stk3x1x_register_read(stk3x1x_obj->client, 14, &rbuf[14], 7);

	if (ret < 0) {
		APS_ERR("error: %d\n", ret);
		return -EFAULT;
	}
	len += scnprintf(buf + len, PAGE_SIZE - len,
			"[PS=%2X] [ALS=%2X] [WAIT=%4Xms]\n",
			rbuf[0] & 0x01, (rbuf[0] & 0x02) >> 1,
			((rbuf[0] & 0x04) >> 2) * rbuf[5] * 6);
	len += scnprintf(buf + len, PAGE_SIZE - len,
			"[EN_ASO=%2X] [EN_AK=%2X] [NEAR/FAR=%2X]\n",
			(rbuf[0] & 0x20) >> 5, (rbuf[0] & 0x40) >> 6,
			rbuf[16] & 0x01);
	len += scnprintf(buf + len, PAGE_SIZE - len,
			"[FLAG_OUI=%2X] [FLAG_PSINT=%2X] [FLAG_ALSINT=%2X]\n",
			(rbuf[16] & 0x04) >> 2,
			(rbuf[16] & 0x10) >> 4, (rbuf[16] & 0x20) >> 5);
	return len;
}

static ssize_t show_status_1st(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	struct stk3x1x_priv *obj = stk3x1x_dual.als[position_1st];

	if (!obj)
		return scnprintf(buf, PAGE_SIZE, ALS_NOT_EXSIT);

	len = stk3x1x_show_status(obj, buf);

	return len;
}
static ssize_t show_status_2nd(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	struct stk3x1x_priv *obj = stk3x1x_dual.als[position_2nd];

	if (!obj)
		return scnprintf(buf, PAGE_SIZE, ALS_NOT_EXSIT);

	len = stk3x1x_show_status(obj, buf);

	return len;
}
/*----------------------------------------------------------------------------*/
#define IS_SPACE(CH) (((CH) == ' ') || ((CH) == '\n'))
/*----------------------------------------------------------------------------*/
static int stk3x1x_show_als_enable(struct stk3x1x_priv *obj, int32_t *enable)
{
	u8 r_buf;
	int ret;

	ret = stk3x1x_register_read(obj->client,
					STK_STATE_REG, &r_buf, 0x01);
	if (ret < 0) {
		APS_ERR("error: %d\n", ret);
		return -EFAULT;
	}

	*enable = (r_buf & STK_STATE_EN_ALS_MASK) ? 1 : 0;

	return ret;
}

static ssize_t show_als_enable_1st(struct device_driver *ddri, char *buf)
{
	int32_t enable;
	int ret;
	struct stk3x1x_priv *obj = stk3x1x_dual.als[position_1st];

	if (!obj)
		return scnprintf(buf, PAGE_SIZE, ALS_NOT_EXSIT);

	ret = stk3x1x_show_als_enable(obj, &enable);

	if (ret == -EFAULT)
		return ret;

	return scnprintf(buf, PAGE_SIZE, "%d\n", enable);
}

static ssize_t show_als_enable_2nd(struct device_driver *ddri, char *buf)
{
	int32_t enable;
	int ret;
	struct stk3x1x_priv *obj = stk3x1x_dual.als[position_2nd];

	if (!obj)
		return scnprintf(buf, PAGE_SIZE, ALS_NOT_EXSIT);

	ret = stk3x1x_show_als_enable(obj, &enable);

	if (ret == -EFAULT)
		return ret;

	return scnprintf(buf, PAGE_SIZE, "%d\n", enable);
}

static int stk3x1x_store_als_enable(struct stk3x1x_priv *obj,
		const char *buf)
{
	uint8_t en;

	if (sysfs_streq(buf, "1")) {
		en = 1;
	} else if (sysfs_streq(buf, "0")) {
		en = 0;
	} else {
		APS_ERR("%s, invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}

	return stk3x1x_enable_als(obj->client, en);
}


static ssize_t store_als_enable_1st(struct device_driver *ddri,
		const char *buf, size_t size)
{
	struct stk3x1x_priv *obj = stk3x1x_dual.als[position_1st];

	if (!obj)
		return -1;

	if (stk3x1x_store_als_enable(obj, buf))
		return -EINVAL;
	else
		APS_LOG("1st ALS enable\n");

	return size;
}

static ssize_t store_als_enable_2nd(struct device_driver *ddri,
		const char *buf, size_t size)
{
	struct stk3x1x_priv *obj = stk3x1x_dual.als[position_2nd];

	if (!obj)
		return -1;

	if (stk3x1x_store_als_enable(obj, buf))
		return -EINVAL;
	else
		APS_LOG("2nd ALS enable\n");

	return size;
}
/*----------------------------------------------------------------------------*/

static void stk3x1x_show_als_test(struct stk3x1x_priv *obj,
				uint32_t *als_reading, uint32_t *als_value)
{
	APS_LOG("%s: [STK]Start testing light...\n", __func__);

	msleep(150);
	*als_reading = stk3x1x_get_als_reading_avg(obj, 5);
	APS_LOG("%s: [STK]als_reading = %d\n", __func__, *als_reading);

	*als_value = als_calibration(obj, *als_reading);

	APS_LOG("%s: [STK]Start testing light done!!! als_value = %d\n",
			__func__, *als_value);
	APS_LOG("%s: [STK]Start testing light done!!! als_test_adc = %d\n",
			__func__, *als_reading);

}

static ssize_t show_als_test_1st(struct device_driver *ddri, char *buf)
{
	int als_reading = 0;
	unsigned int als_value_1st;		/* lux value in test level*/
	struct stk3x1x_priv *obj = stk3x1x_dual.als[position_1st];

	if (!obj)
		return scnprintf(buf, PAGE_SIZE, ALS_NOT_EXSIT);

	stk3x1x_show_als_test(obj,
					&als_reading, &als_value_1st);

	return scnprintf(buf, PAGE_SIZE,
			"1st als_value = %5d lux, cci_als_test_adc = %5d\n",
			als_value_1st, als_reading);
}

static ssize_t show_als_test_2nd(struct device_driver *ddri, char *buf)
{
	int als_reading = 0;
	unsigned int als_value_2nd;		/* lux value in test level*/
	struct stk3x1x_priv *obj = stk3x1x_dual.als[position_2nd];

	if (!obj)
		return scnprintf(buf, PAGE_SIZE, ALS_NOT_EXSIT);

	stk3x1x_show_als_test(obj,
					&als_reading, &als_value_2nd);

	return scnprintf(buf, PAGE_SIZE,
			"2nd als_value = %5d lux, cci_als_test_adc = %5d\n",
			als_value_2nd, als_reading);
}
static ssize_t show_als_cali_value(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;

	if (stk3x1x_dual.als[position_1st]) {
		len += scnprintf(buf, PAGE_SIZE,
			"1st 20lux cali = %d, 1st 400lux cali = %d\n",
			stk3x1x_dual.als[position_1st]->als_cal_low,
			stk3x1x_dual.als[position_1st]->als_cal_high);
	}

	if (stk3x1x_dual.als[position_2nd]) {
		len += scnprintf(buf + len, PAGE_SIZE - len,
			"2nd 20lux cali = %d, 2nd 400lux cali = %d\n",
			stk3x1x_dual.als[position_2nd]->als_cal_low,
			stk3x1x_dual.als[position_2nd]->als_cal_high);

	}
	return len;

}

static ssize_t store_als_cali_value(struct device_driver *ddri,
		const char *buf, size_t size)
{
	int ret = 0;

	if (size > 0)
		ret = als_cali_set((char *)buf);
	else {
		APS_ERR("%s, invalid value: size too small\n", __func__);
		return -EINVAL;
	}

	if (ret)
		APS_ERR("%s, invalid value %s\n", __func__, buf);

	return size;
}
/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(als_1, S_IWUSR | S_IRUGO, show_als_1st, NULL);
static DRIVER_ATTR(als_2, S_IWUSR | S_IRUGO, show_als_2nd, NULL);
static DRIVER_ATTR(ir_1, S_IWUSR | S_IRUGO, show_ir_1st, NULL);
static DRIVER_ATTR(ir_2, S_IWUSR | S_IRUGO, show_ir_2nd, NULL);
static DRIVER_ATTR(config, S_IWUSR | S_IRUGO, stk3x1x_show_config, NULL);
static DRIVER_ATTR(trace, S_IWUSR | S_IRUGO, stk3x1x_show_trace,
		stk3x1x_store_trace);
static DRIVER_ATTR(calival, S_IWUSR | S_IRUGO, show_als_cali_value,
		store_als_cali_value);
static DRIVER_ATTR(status_1, S_IWUSR | S_IRUGO, show_status_1st, NULL);
static DRIVER_ATTR(status_2, S_IWUSR | S_IRUGO, show_status_2nd, NULL);
static DRIVER_ATTR(allreg_1, S_IWUSR | S_IRUGO,
		show_allreg_1st, NULL);
static DRIVER_ATTR(allreg_2, S_IWUSR | S_IRUGO,
		show_allreg_2nd, NULL);
/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(enable_1, S_IWUSR | S_IRUGO,
		show_als_enable_1st, store_als_enable_1st);
static DRIVER_ATTR(enable_2, S_IWUSR | S_IRUGO,
		show_als_enable_2nd, store_als_enable_2nd);
static DRIVER_ATTR(alstest_1, S_IWUSR | S_IRUGO, show_als_test_1st, NULL);
static DRIVER_ATTR(alstest_2, S_IWUSR | S_IRUGO, show_als_test_2nd, NULL);
static DRIVER_ATTR(alscali_1, S_IWUSR | S_IRUGO,
		stk3x1x_show_als_cali_1st, NULL);
static DRIVER_ATTR(alscali_2, S_IWUSR | S_IRUGO,
		stk3x1x_show_als_cali_2nd, NULL);
/*----------------------------------------------------------------------------*/
static struct driver_attribute *stk3x1x_attr_list_common[] = {

	&driver_attr_trace,		   /*trace log*/
	&driver_attr_config,
	&driver_attr_calival,
};

static struct driver_attribute *stk3x1x_attr_list_als_1[] = {
	&driver_attr_als_1,
	&driver_attr_ir_1,
	&driver_attr_enable_1,
	&driver_attr_status_1,
	&driver_attr_allreg_1,
	&driver_attr_alstest_1,
	&driver_attr_alscali_1,
};

static struct driver_attribute *stk3x1x_attr_list_als_2[] = {
	&driver_attr_als_2,
	&driver_attr_ir_2,
	&driver_attr_enable_2,
	&driver_attr_status_2,
	&driver_attr_allreg_2,
	&driver_attr_alstest_2,
	&driver_attr_alscali_2,
};

/*----------------------------------------------------------------------------*/
static int stk3x1x_create_attr(struct device_driver *driver,
				struct driver_attribute **attr_list, int num)
{
	int idx, err = 0;

	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++) {
		err = driver_create_file(driver, attr_list[idx]);

		if (err) {
			APS_ERR("driver_create_file (%s) = %d\n",
					attr_list[idx]->attr.name, err);
			break;
		}
	}

	return err;
}
/*----------------------------------------------------------------------------*/
static int stk3x1x_delete_attr(struct device_driver *driver,
				struct driver_attribute **attr_list, int num)
{
	int idx, err = 0;

	if (!driver)
		return -EINVAL;

	for (idx = 0; idx < num; idx++)
		driver_remove_file(driver, attr_list[idx]);

	return err;
}
/*----------------------------------------------------------------------------*/
static int dualals_create_node(int sensor_num)
{
	int err = 0;
	int num = 0;

	if (sensor_num == 1) {
		num = (int)(sizeof(stk3x1x_attr_list_als_1) /
			sizeof(stk3x1x_attr_list_als_1[0]));

		err =
		stk3x1x_create_attr(
			&(stk3x1x_init_info.platform_diver_addr->driver),
			stk3x1x_attr_list_als_1,
			num);
	} else {
		num = (int)(sizeof(stk3x1x_attr_list_als_2) /
			sizeof(stk3x1x_attr_list_als_2[0]));

		err =
		stk3x1x_create_attr(
			&(stk3x1x_init_info.platform_diver_addr->driver),
			stk3x1x_attr_list_als_2,
			num);
	}

	if (err) {
		APS_ERR("create attribute err = %d\n", err);
		return -ENAVAIL;
	}

	return 0;
}
/*----------------------------------------------------------------------------*/
static int dualals_delete_node(void)
{
	int err = 0;
	int num = 0;
	struct stk3x1x_priv *obj;

	obj = stk3x1x_dual.als[position_1st];
	if (obj) {
		num = (int)(sizeof(stk3x1x_attr_list_als_1) /
			sizeof(stk3x1x_attr_list_als_1[0]));

		err =
		stk3x1x_delete_attr(
			&(stk3x1x_init_info.platform_diver_addr->driver),
			stk3x1x_attr_list_als_1,
			num);

		if (err)
			APS_ERR("delete als node 1 attribute err = %d\n", err);
	}

	obj = stk3x1x_dual.als[position_2nd];
	if (obj) {
		num = (int)(sizeof(stk3x1x_attr_list_als_2) /
			sizeof(stk3x1x_attr_list_als_2[0]));

		err =
		stk3x1x_delete_attr(
			&(stk3x1x_init_info.platform_diver_addr->driver),
			stk3x1x_attr_list_als_2,
			num);

		if (err) {
			APS_ERR("delete als node 1 attribute err = %d\n", err);
			return -EIO;
		}
	}

	return 0;
}
/*----------------------------------------------------------------------------*/
static int common_delete_node(void)
{
	int num = 0;
	int err = 0;

	APS_LOG("delete common node\n");

	num = (int)(sizeof(stk3x1x_attr_list_common) /
			sizeof(stk3x1x_attr_list_common[0]));

	err =
	stk3x1x_delete_attr(
		&(stk3x1x_init_info.platform_diver_addr->driver),
		stk3x1x_attr_list_common,
		num);

	if (err) {
		APS_ERR("delete common attribute err = %d\n", err);
		return -EIO;
	}

	return 0;
}
/*----------------------------------------------------------------------------*/
static void als_position_init(struct stk3x1x_priv *obj, uint8_t sensors)
{
	if (obj->sensor_num == 1)
		position_1st = sensors;
	else
		position_2nd = sensors;
}
/*----------------------------------------------------------------------------*/
static int32_t stk3x1x_get_ir_value(struct stk3x1x_priv *obj)
{
	int32_t word_data, ret;
	uint8_t w_reg;

	u8 flag;
	u8 buf[2];
	u8 state_buf;

	ret = stk3x1x_read_flag(obj->client, &flag);

	if (ret < 0) {
		APS_ERR("WARNING: read flag reg error: %d\n", ret);
		goto irs_err_i2c_rw;
	}

	if ((flag & STK_FLG_IR_RDY_MASK)) {
		APS_DBG("IR flag ready!\n");
		ret = stk3x1x_clear_intr(obj->client, flag,
							STK_FLG_IR_RDY_MASK);

		if (ret < 0) {
			APS_ERR("%s: write i2c error\n", __func__);
			goto irs_err_i2c_rw;
		}

		ret = stk3x1x_register_read(obj->client,
							STK_DATA1_IR_REG,
							buf,
							2);

		if (ret < 0) {
			APS_ERR("%s fail, ret=0x%x", __func__, ret);
			goto irs_err_i2c_rw;
		}

		word_data = (buf[0] << 8) | buf[1];
		obj->ir_code = word_data;

	}

	get_monotonic_boottime(&now);
	stk_diff = timespec_sub(now, last_time);

	APS_DBG("now=%lu, last=%lu, diff = %lu\n",
			now.tv_nsec, last_time.tv_nsec, stk_diff.tv_nsec);

	if (stk_diff.tv_nsec > IR_INTERVAL_500MS) {
		stk3x1x_register_read(obj->client,
						STK_STATE_REG, &state_buf, 1);

		if (!(state_buf & STK_STATE_EN_IRS_MASK)) {
			w_reg =
			atomic_read(&obj->state_val) | STK_STATE_EN_IRS_MASK;

			ret = i2c_smbus_write_byte_data(obj->client,
								STK_STATE_REG,
								w_reg);

			if (ret < 0) {
				APS_ERR("%s: write i2c error\n", __func__);
				goto irs_err_i2c_rw;
			}
		}
		get_monotonic_boottime(&last_time);
	}

irs_err_i2c_rw:
	return ret;
}

/******************************************************************************
 * Function Configuration
******************************************************************************/
static int stk3x1x_open(struct inode *inode, struct file *file)
{
	file->private_data = stk3x1x_dual.als[MASTER]->client;

	if (!file->private_data) {
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}

	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int stk3x1x_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}
/*----------------------------------------------------------------------------*/
static long stk3x1x_unlocked_ioctl(struct file *file,
				unsigned int cmd, unsigned long arg)
{
	struct stk3x1x_priv *obj;
	long err = 0;
	void __user *ptr = (void __user *) arg;
	int dat;
	uint32_t enable;

	switch (cmd) {
	case ALSPS_SET_ALS_MODE:
		if (copy_from_user(&enable, ptr, sizeof(enable))) {
			err = -EFAULT;
			goto err_out;
		}

		if (enable) {
			for (int i = 0; i < stk3x1x_dual.als_count; i++) {
				obj = stk3x1x_dual.als[i];
				if (obj)
					err = stk3x1x_enable_als(obj->client, 1);

				if (err) {
					APS_ERR("enable als fail: %ld\n", err);
					goto err_out;
				}

				set_bit(STK_BIT_ALS, &obj->enable);
			}
		} else {
			for (int i = 0; i < stk3x1x_dual.als_count; i++) {
				obj = stk3x1x_dual.als[i];

				err = stk3x1x_enable_als(obj->client, 0);

				if (err) {
					APS_ERR("disable als fail: %ld\n", err);
					goto err_out;
				}

				clear_bit(STK_BIT_ALS, &obj->enable);
			}
		}

		break;

	case ALSPS_GET_ALS_MODE:
		enable = test_bit(STK_BIT_ALS, &obj->enable) ? (1) : (0);

		if (copy_to_user(ptr, &enable, sizeof(enable))) {
			err = -EFAULT;
			goto err_out;
		}

		break;

	case ALSPS_GET_ALS_DATA:
		err = als_get_lux(&dat);

		if (err)
			goto err_out;


		if (copy_to_user(ptr, &dat, sizeof(dat))) {
			err = -EFAULT;
			goto err_out;
		}

		break;

	case ALSPS_GET_ALS_RAW_DATA:
			obj = stk3x1x_dual.als[MASTER];
			err = stk3x1x_read_als(obj->client, &obj->als);
			if (err)
				goto err_out;

			dat = obj->als;

			if (stk3x1x_dual.als[SLAVE]) {
				obj = stk3x1x_dual.als[SLAVE];

				err = stk3x1x_read_als(obj->client, &obj->als);
				if (err)
					goto err_out;

				if (dat < obj->als)
					dat = obj->als;

			}

			if (copy_to_user(ptr, &dat, sizeof(dat))) {
				err = -EFAULT;
				goto err_out;
			}
		break;

	default:
		APS_ERR("%s not supported = 0x%04x", __func__, cmd);
		err = -ENOIOCTLCMD;
		break;
	}

err_out:
	return err;
}
/*----------------------------------------------------------------------------*/
static const struct file_operations stk3x1x_fops = {
	.open = stk3x1x_open,
	.release = stk3x1x_release,
	.unlocked_ioctl = stk3x1x_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice stk3x1x_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &stk3x1x_fops,
};
/*----------------------------------------------------------------------------*/

#ifdef CONFIG_PM_SLEEP
static int stk3x1x_suspend(struct device *dev)
{
	int err = 0;

	APS_FUN();

	if (atomic_read(&stk3x1x_dual.als_suspend))
		return 0;

	alsps_driver_pause_polling(1);
	atomic_set(&stk3x1x_dual.als_suspend, 1);

	for (int i = 0; i < stk3x1x_dual.als_count; i++) {
		struct i2c_client *client =
			stk3x1x_dual.als[i]->client;

		err &= stk3x1x_enable_als(client, 0);
	}

	if (err) {
		atomic_set(&stk3x1x_dual.als_suspend, 0);
		alsps_driver_pause_polling(0);
		APS_ERR("disable als fail: %d\n", err);
		return err;
	}

	return 0;
}

static int stk3x1x_resume(struct device *dev)
{
	int err = 0;

	APS_FUN();

	if (!atomic_read(&stk3x1x_dual.als_suspend))
		return 0;

	for (int i = 0; i < stk3x1x_dual.als_count; i++) {
		struct i2c_client *client =
				stk3x1x_dual.als[i]->client;

		if (alsps_driver_query_polling_state(ID_LIGHT) == 1) {
			err = stk3x1x_enable_als(client, 1);
			if (err) {
				APS_ERR("enable als fail: %d\n", err);
				return err;
			}
		}
	}

	atomic_set(&stk3x1x_dual.als_suspend, 0);
	alsps_driver_pause_polling(0);

	return 0;
}
#endif
/*----------------------------------------------------------------------------*/
static int als_open_report_data(int open)
{
	return 0;
}

static int als_enable_nodata(int en)
{
	int res = 0;

	APS_LOG("stk3x1x_obj als enable value = %d\n", en);

	for (int i = 0; i < stk3x1x_dual.als_count; i++) {

		res = stk3x1x_enable_als(stk3x1x_dual.als[i]->client, en);

		if (res) {
			APS_ERR("als_enable_nodata is failed!!\n");
			return -1;
		}
	}
	return 0;
}

static int als_set_delay(u64 ns)
{
	/* TODO */
	return 0;
}

static int als_batch(int flag, int64_t samplingPeriodNs,
					int64_t maxBatchReportLatencyNs)
{
	return als_set_delay(samplingPeriodNs);
}

static int als_flush(void)
{
	return als_flush_report();
}

static int als_select_data(int alslux1, int alslux2)
{
	if (alslux1 > alslux2)
		return alslux1;
	else
		return alslux2;

}
/*----------------------------------------------------------------------------*/
static void als_debug_log(int *lux_cali, int als_report)
{
	if (atomic_read(&stk3x1x_dual.trace) & STK_TRC_DEBUG) {
		if (stk3x1x_dual.als[position_1st])
			APS_LOG("ALS_1_DATA=%d,%d,%d\n",
				lux_cali[position_1st],
				stk3x1x_dual.als[position_1st]->als,
				stk3x1x_dual.als[position_1st]->als_code_last);

		if (stk3x1x_dual.als[position_2nd])
			APS_LOG("ALS_2_DATA=%d,%d,%d\n",
				lux_cali[position_2nd],
				stk3x1x_dual.als[position_2nd]->als,
				stk3x1x_dual.als[position_2nd]->als_code_last);
	}

	if (atomic_read(&stk3x1x_dual.trace) & STK_TRC_ALS_DATA) {
		if (stk3x1x_dual.dual_als_enable) {
			APS_LOG(
				"ALS_DATA=%d,%s,%d,%d,%d,%d,%d,%d,%d,%s,%d,%d,%d,%d,%d,%d,%d\n",
				als_report,
				"1st",
				lux_cali[position_1st],
				stk3x1x_dual.als[position_1st]->als,
				stk3x1x_dual.als[position_1st]->als_code_last,
				stk3x1x_dual.als[position_1st]->ir_code,
				stk3x1x_dual.als[position_1st]->als_correct_factor,
				stk3x1x_dual.als[position_1st]->als_cal_low,
				stk3x1x_dual.als[position_1st]->als_cal_high,
				"2nd",
				lux_cali[position_2nd],
				stk3x1x_dual.als[position_2nd]->als,
				stk3x1x_dual.als[position_2nd]->als_code_last,
				stk3x1x_dual.als[position_2nd]->ir_code,
				stk3x1x_dual.als[position_2nd]->als_correct_factor,
				stk3x1x_dual.als[position_2nd]->als_cal_low,
				stk3x1x_dual.als[position_2nd]->als_cal_high);
		} else {
			if (stk3x1x_dual.als[position_1st])
				APS_LOG("ALS_1_DATA=%s,%d,%d,%d,%d,%d,%d,%d\n",
					"1st",
					lux_cali[position_1st],
					stk3x1x_dual.als[position_1st]->als,
					stk3x1x_dual.als[position_1st]->als_code_last,
					stk3x1x_dual.als[position_1st]->ir_code,
					stk3x1x_dual.als[position_1st]->als_correct_factor,
					stk3x1x_dual.als[position_1st]->als_cal_low,
					stk3x1x_dual.als[position_1st]->als_cal_high);

			if (stk3x1x_dual.als[position_2nd])
				APS_LOG("ALS_2_DATA=%s,%d,%d,%d,%d,%d,%d,%d\n",
					"2nd",
					lux_cali[position_2nd],
					stk3x1x_dual.als[position_2nd]->als,
					stk3x1x_dual.als[position_2nd]->als_code_last,
					stk3x1x_dual.als[position_2nd]->ir_code,
					stk3x1x_dual.als[position_2nd]->als_correct_factor,
					stk3x1x_dual.als[position_2nd]->als_cal_low,
					stk3x1x_dual.als[position_2nd]->als_cal_high);
		}
	}

}
/*----------------------------------------------------------------------------*/
static int als_get_lux(int *value)
{
	int err = 0;
	int alslux[2] = {0};

	for (int i = MASTER; i < stk3x1x_dual.als_count; i++) {
		struct stk3x1x_priv *obj = stk3x1x_dual.als[i];

		if (!obj) {
			APS_ERR("stk3x1x_obj is null!!\n");
			err = -1;
			goto out;
		}

		err = stk3x1x_read_als(obj->client, &obj->als);

		APS_DBG("stk3x1x_read_als bus=%d als_num=%d data = %d ",
				obj->hw->i2c_num,
				obj->sensor_num,
				obj->als);

		if (err) {
			err = -1;
			goto out;
		}

		alslux[i] = als_calibration(obj, obj->als);
	}

	*value = als_select_data(alslux[MASTER], alslux[SLAVE]);

	als_debug_log(alslux, *value);

out:
	return err;
}

static int als_get_data(int *value, int *status)
{
	int err = 0;

	err = als_get_lux(value);

	if (!err)
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	return err;

}

static bool check_is_master(struct device_node *node)
{
	u32 is_master[] = {0};
	int ret = 0;

	ret = of_property_read_u32_array(node, "is_master", is_master,
	ARRAY_SIZE(is_master));

	if (is_master[0])
		return true;

	return false;
}

static int read_sensor_num(struct device_node *node, int *sensor_num)
{
	u32 buf = 0;
	int ret = 0;

	ret = of_property_read_u32(node, "sensor_num", &buf);

	if (ret)
		return -ENAVAIL;

	*sensor_num = (int)buf;

	APS_LOG("sensor number is %d\n", *sensor_num);

	return 0;
}

static int get_cali_compensation(struct device_node *node, char *name)
{
	u32 buf = 0;
	int ret = 0;
	int cali_compensation = 1000;

	ret = of_property_read_u32(node, name, &buf);

	if (ret) {
		APS_ERR("read cali compensation from dts failed!!\n");
		return cali_compensation;
	}

	cali_compensation = (int)buf;

	return cali_compensation;
}

static void set_chip_common_setting(void)
{
	stk3x1x_get_addr(&stk3x1x_dual.addr);

	atomic_set(&stk3x1x_dual.als_debounce, 200);
	atomic_set(&stk3x1x_dual.als_deb_on, 0);
	atomic_set(&stk3x1x_dual.als_deb_end, 0);
	atomic_set(&stk3x1x_dual.trace, 0x00);
	atomic_set(&stk3x1x_dual.als_suspend, 0);
	atomic_set(&stk3x1x_dual.alsctrl_val, 0x39);
	atomic_set(&stk3x1x_dual.i2c_retry, 3);
	stk3x1x_dual.ledctrl_val = 0xFF;
	stk3x1x_dual.wait_val = 0xF;
	stk3x1x_dual.int_val = 0;
}

/*----------------------------------------------------------------------------*/
static int stk3x1x_i2c_probe(struct i2c_client *client,
						const struct i2c_device_id *id)
{
	struct stk3x1x_priv *obj = NULL;
	struct alsps_hw *alsps_cust = NULL;
	int err = 0;
	struct als_control_path als_ctl = { 0 };
	struct als_data_path als_data = { 0 };

	if (stk3x1x_dual.als_count >= 2) {
		err = -ENAVAIL;
		goto exit;
	}

	alsps_cust = kzalloc(sizeof(*alsps_cust), GFP_KERNEL);
	if (!alsps_cust) {
		err = -ENOMEM;
		goto exit;
	}
	memset(alsps_cust, 0, sizeof(*alsps_cust));

	err = get_alsps_dts_func(client->dev.of_node, alsps_cust);
	if (err < 0) {
		APS_ERR("get customization info from dts failed\n");
		goto exit;
	}

	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (!obj) {
		err = -ENOMEM;
		goto exit_obj_failed;
	}

	memset(obj, 0, sizeof(*obj));

	obj->hw = alsps_cust;
	client->addr = *alsps_cust->i2c_addr;
	obj->client = client;

	i2c_set_clientdata(client, obj);
	/*set device data to dual als struct*/

	if (stk3x1x_check_id(client)) {
		err = -1;
		goto exit_init_failed;
	}

	err = read_sensor_num(client->dev.of_node, &obj->sensor_num);
	if (err) {
		APS_ERR("read sensor number fail! %d\n", err);
		goto exit_init_failed;
	}

	if (check_is_master(client->dev.of_node)) {
		stk3x1x_dual.als[MASTER] = obj;
		als_position_init(obj, MASTER);

	} else {
		stk3x1x_dual.als[SLAVE] = obj;
		als_position_init(obj, SLAVE);
	}

	obj->als_cali_compen_20 =
		get_cali_compensation(client->dev.of_node, "cali_compen_20");
	APS_LOG("cali_compen_20 = %d\n", obj->als_cali_compen_20);
	obj->als_cali_compen_400 =
		get_cali_compensation(client->dev.of_node, "cali_compen_400");
	APS_LOG("cali_compen_400 = %d\n", obj->als_cali_compen_400);

	obj->als_cal_low = 0;
	obj->als_cal_high = 0;

	obj->irq_node = client->dev.of_node;
	obj->als_correct_factor = 1000;

	obj->hw->polling_mode_als = 1;
	APS_LOG("%s: force PS = polling and ALS = polling\n", __func__);

	obj->enable = 0;
	obj->pending_intr = 0;

	atomic_set(&obj->state_val, 0);

	if (atomic_read(&obj->state_val) & STK_STATE_EN_ALS_MASK)
		set_bit(STK_BIT_ALS, &obj->enable);

	err = stk3x1x_init_client(client);
	if (err) {
		APS_ERR("stk3x1x init client failed\n");
		goto exit_init_failed;
	}

	stk3x1x_dual.als_count++;

	if (stk3x1x_dual.als_count > 1) {
		stk3x1x_dual.dual_als_enable = 1;

		err = dualals_create_node(obj->sensor_num);
		if (err)
			goto exit_2nd_dualals_node_fail;

		return 0;
	}

	err = misc_register(&stk3x1x_device);
	if (err) {
		APS_ERR("stk3x1x_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	err =
	stk3x1x_create_attr(
		&(stk3x1x_init_info.platform_diver_addr->driver),
		stk3x1x_attr_list_common,
		3);

	if (err) {
		APS_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}

	err = dualals_create_node(obj->sensor_num);
	if (err) {
		APS_ERR("create dualals note fail! err = %d\n", err);
		goto exit_dualals_node_fail;
	}

	als_ctl.open_report_data = als_open_report_data;
	als_ctl.enable_nodata = als_enable_nodata;
	als_ctl.set_delay = als_set_delay;
	als_ctl.batch = als_batch;
	als_ctl.flush = als_flush;
	als_ctl.is_report_input_direct = false;
	als_ctl.is_use_common_factory = false;

	if (obj->hw->polling_mode_als == 1)
		als_ctl.is_polling_mode = true;
	else
		als_ctl.is_polling_mode = false;

	als_ctl.is_support_batch = false;

	err = als_register_control_path(&als_ctl);

	if (err) {
		APS_ERR("als_control register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	als_data.get_data = als_get_data;
	als_data.vender_div = 100;
	err = als_register_data_path(&als_data);
	if (err) {
		APS_ERR("als_data register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}


	stk3x1x_init_flag = 0;

	APS_LOG("%s: OK\n", __func__);

	return 0;

exit_2nd_dualals_node_fail:
exit_sensor_obj_attach_fail:
	dualals_delete_node();
exit_dualals_node_fail:
	common_delete_node();
exit_create_attr_failed:
	misc_deregister(&stk3x1x_device);
exit_misc_device_register_failed:
exit_init_failed:
	kfree(obj);
exit_obj_failed:
	kfree(alsps_cust);
exit:

	stk3x1x_init_flag = -1;

	APS_ERR("%s: err = %d\n", __func__, err);
	return err;
}
/*----------------------------------------------------------------------------*/
static int stk3x1x_i2c_remove(struct i2c_client *client)
{
	int err = 0;

	if (stk3x1x_dual.als_count == 1) {
		err = dualals_delete_node();
		if (err)
			APS_ERR("dualals_delete_node fail: %d\n", err);

		err = common_delete_node();
		if (err)
			APS_ERR("common_delete_node fail: %d\n", err);

		misc_deregister(&stk3x1x_device);
	}

	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));

	stk3x1x_dual.als_count--;

	return 0;
}

static int stk3x1x_local_uninit(void)
{
	APS_FUN();

	i2c_del_driver(&stk3x1x_i2c_driver);

	return 0;
}

/*----------------------------------------------------------------------------*/
unsigned char *idme_get_dual_alscal_value(void)
{
	struct device_node *ap = NULL;
	char *alscal = NULL;

	ap = of_find_node_by_path(IDME_OF_ALSCAL);
	if (ap) {
		alscal = (char *)of_get_property(ap, "value", NULL);
		pr_info("alscal %s\n", alscal);

	} else
		pr_err("of_find_node_by_path failed\n");

	return alscal;
}

int idme_get_tp_cg_color(void)
{
	struct device_node *ap = NULL;
	char *cg_color = NULL;
	int res = -1;

	ap = of_find_node_by_path(IDME_OF_CG_COLOR);
	if (ap) {
		cg_color = (char *)of_get_property(ap, "value", NULL);
		pr_info("tp_cg_color %s\n", cg_color);

	} else
		pr_err("of_find_node_by_path failed\n");

	if (unlikely(kstrtouint(cg_color, 10, &res)))
		pr_err("idme_get_board_rev kstrtouint failed!\v");

	return res;
}

static int stk3x1x_get_dual_cal(void)
{
	char *alscal;
	int err = 0;

	alscal = idme_get_dual_alscal_value();

	if (!alscal) {
		err = -1;
		goto idme_err;
	}

	if (!(strlen(alscal) > 0)) {
		err = -1;
		APS_ERR("alscal get failed! use default value\n", err);

		goto idme_err;
	}

	err = als_cali_set(alscal);

idme_err:
	return err;
}
/*----------------------------------------------------------------------------*/

static int stk3x1x_local_init(void)
{

	APS_FUN();

	stk3x1x_dual.dual_als_enable = 0;
	stk3x1x_dual.als[0] = NULL;
	stk3x1x_dual.als[1] = NULL;
	stk3x1x_dual.als_count = 0;

	set_chip_common_setting();

	if (i2c_add_driver(&stk3x1x_i2c_driver)) {
		APS_ERR("add driver error\n");
		return -1;
	}

	if (-1 == stk3x1x_init_flag) {
		APS_ERR("%s fail with stk3x1x_init_flag=%d\n",
			__func__, stk3x1x_init_flag);
		return -1;
	}

	if (stk3x1x_get_dual_cal())
		APS_ERR("get idme for alscal failed!!\n ");

	stk3x1x_dual.tp_cg_color = idme_get_tp_cg_color();

	return 0;
}

/*----------------------------------------------------------------------------*/
static int __init stk3x1x_init(void)
{
	alsps_driver_add(&stk3x1x_init_info);
	/* hwmsen_alsps_add(&stk3x1x_init_info);*/
	pr_info("%s done\n", __func__);
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit stk3x1x_exit(void)
{
	APS_FUN();
}
/*----------------------------------------------------------------------------*/
module_init(stk3x1x_init);
module_exit(stk3x1x_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("EdwardCW Lin");
MODULE_DESCRIPTION("stk3x1x dual light sensor driver");
MODULE_LICENSE("GPLv2");
