/*
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

#include "cust_alsps.h"
#include "ltrx130a.h"
#include "alsps.h"

/*#define DEMO_BOARD*/
/*#define LTRX130A_DEBUG*/

/******************************************************************************
 * configuration
*******************************************************************************/
/*--------------------------------------------------------------------------*/
#define LTRX130A_DEV_NAME       "ltrx130a"

/*--------------------------------------------------------------------------*/
#define APS_TAG                 "[ALS] "
#define APS_FUN(f)              pr_debug(APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)   pr_err(APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_DBG(fmt, args...)   pr_debug(APS_TAG fmt, ##args)
#define APS_LOG(fmt, args...)   pr_info(APS_TAG fmt, ##args)

/*--------------------------------------------------------------------------*/
static const struct i2c_device_id ltrx130a_i2c_id[] = {
	{LTRX130A_DEV_NAME, 0},
	{}
};
/*--------------------------------------------------------------------------*/
static int ltrx130a_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int ltrx130a_i2c_remove(struct i2c_client *client);
static int ltrx130a_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
static int ltrx130a_i2c_suspend(struct device *dev);
static int ltrx130a_i2c_resume(struct device *dev);

#ifdef LTRX130A_DEBUG
static int ltrx130a_dump_reg(void);
#endif

static int als_gainrange;
static int final_lux_val;

static unsigned int  position_1st = 1;
static unsigned int  position_2nd = 1;

#define ALS_CALI_VALUE 400

#define MASTER 0
#define SLAVE 1
#define IDME_OF_ALSCAL			"/idme/alscal"
#define IDME_OF_CG_COLOR		"/idme/tp_cg_color"
#define IDME_MAX_LEN 14
#define BLACK 1
#define WHITE 2
#define ALS_NOT_EXSIT		"als not exsit!\n"
#define ALS_CALIBRATION_LUX_HIGH 400
#define ALS_CALIBRATION_LUX_LOW 20
#define LTR_STATE_EN_ALS_MASK 0x02
/*--------------------------------------------------------------------------*/
typedef enum {
	LTR_TRC_DEBUG = 0x0001,
	LTR_TRC_ALS_DATA = 0x0002,
} LTR_TRC;
/*--------------------------------------------------------------------------*/
typedef enum {
	CMC_BIT_ALS    = 1,
} CMC_BIT;

/*--------------------------------------------------------------------------*/
struct ltrx130a_priv {
	struct alsps_hw  *hw;
	struct i2c_client *client;
	struct work_struct	eint_work;

	/*misc*/
	u16 als_modulus;

#ifdef CONFIG_OF
	struct device_node *irq_node;
	int irq;
#endif

	/*data*/
	u32 als;
	int als_code;
	int mix_code;
	u8 _align;
	u16 als_level_num;
	u16 als_value_num;
	u32 als_level[C_CUST_ALS_LEVEL-1];
	u32 als_value[C_CUST_ALS_LEVEL];

	atomic_t als_cmd_val;       /*the cmd value can't be read, stored in ram*/
	atomic_t als_thd_val_high;  /*the cmd value can't be read, stored in ram*/
	atomic_t als_thd_val_low;   /*the cmd value can't be read, stored in ram*/
	ulong enable;               /*enable mask*/
	ulong pending_intr;         /*pending interrupt*/

	uint16_t als_correct_factor;
	uint32_t als_cal_low;
	uint32_t als_cal_high;

	int sensor_num;
};

/*----------------------------------------------------------------------------*/
struct dual_als {
	u_int8_t dual_als_enable;
	int als_count;
	int tp_cg_color;
	int als_win_factor;

	struct ltrx130a_priv *als[2];

	/*misc*/
	atomic_t i2c_retry;
	atomic_t als_suspend;
	atomic_t als_debounce;	/*debounce time after enabling als*/
	atomic_t als_deb_on;	/*indicates if the debounce is on*/
	atomic_t als_deb_end;	/*the jiffies representing the end of debounce*/
	atomic_t trace;

} ltrx130a_dual;

static DEFINE_MUTEX(ltrx130a_mutex);
static DEFINE_MUTEX(ltrx130a_i2c_mutex);

static int ltrx130a_local_init(void);
static int ltrx130a_local_uninit(void);
static int ltrx130a_init_flag; /* 0<==>OK -1 <==> fail */

static struct alsps_init_info ltrx130a_init_info = {
	.name = "ltrx130a",
	.init = ltrx130a_local_init,
	.uninit = ltrx130a_local_uninit,
};

#ifdef CONFIG_OF
static const struct of_device_id alsps_of_match[] = {
	{.compatible = "mediatek,ltrx130a"},
	{},
};
#endif

#ifdef CONFIG_PM_SLEEP
static const struct dev_pm_ops ltrx130a_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ltrx130a_i2c_suspend, ltrx130a_i2c_resume)
};
#endif

static struct i2c_driver ltrx130a_i2c_driver = {
	.probe      = ltrx130a_i2c_probe,
	.remove     = ltrx130a_i2c_remove,
	.detect     = ltrx130a_i2c_detect,
	.id_table   = ltrx130a_i2c_id,
	.driver = {
		.name           = LTRX130A_DEV_NAME,
#ifdef CONFIG_OF
		.of_match_table = alsps_of_match,
#endif

#ifdef CONFIG_PM_SLEEP
		.pm = &ltrx130a_pm_ops,
#endif
	},
};

/*
 * #########
 * ## I2C ##
 * #########
 */
static int ltrx130a_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
	int err = 0;

	if (!client) {
		return -EINVAL;
	} else if (len > C_I2C_FIFO_SIZE) {
		APS_LOG(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}

	mutex_lock(&ltrx130a_i2c_mutex);
	err = i2c_smbus_read_i2c_block_data(client, addr, len, data);
	mutex_unlock(&ltrx130a_i2c_mutex);

	if (err < 0) {
		APS_ERR("i2c_smbus_read_i2c_block_data error: (%d %p %d) %d\n",
			addr, data, len, err);
		err = -EIO;
	} else {
		err = 0;	/*no error */
	}
	return err;
}

static int ltrx130a_i2c_write_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
	int err = 0;

	if (!client) {
		return -EINVAL;
	} else if (len >= C_I2C_FIFO_SIZE) {
		APS_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}

	mutex_lock(&ltrx130a_i2c_mutex);
	err = i2c_smbus_write_i2c_block_data(client, addr, len, data);
	mutex_unlock(&ltrx130a_i2c_mutex);

	if (err < 0) {
		APS_ERR("send command error!!\n");
		return -EFAULT;
	}

	return err;
}

/*--------------------------------------------------------------------------*/
static int ltrx130a_register_read(struct i2c_client *client, u16 addr,
						u8 *buf, int count)
{
	int ret = 0, retry = 0;
	int trc = atomic_read(&ltrx130a_dual.trace);
	int max_try = atomic_read(&ltrx130a_dual.i2c_retry);

	while (retry++ < max_try) {
		ret = ltrx130a_i2c_read_block(client, addr, buf, count);
		if (ret == 0)
			break;
		udelay(100);
	}

	if (unlikely(trc)) {
		if ((retry != 1) && (trc & 0x8000))
			APS_DBG("(recv) %d/%d\n", retry - 1, max_try);
	}

	/* If everything went ok (i.e. 1 msg transmitted), return #bytes */
	/* transmitted, else error code. */
	return (ret == 0) ? count : ret;
}

/*----------------------------------------------------------------------------*/
static int ltrx130a_register_write(struct i2c_client *client, u16 addr,
						u8 *buf, int count)
{
	int ret = 0, retry = 0;
	int trc = atomic_read(&ltrx130a_dual.trace);
	int max_try = atomic_read(&ltrx130a_dual.i2c_retry);

	while (retry++ < max_try) {
		ret = ltrx130a_i2c_write_block(client, addr, buf, count);
		if (ret == 0)
			break;
		udelay(100);
	}

	if (unlikely(trc)) {
		if ((retry != 1) && (trc & 0x8000))
			APS_LOG("(send) %d/%d\n", retry - 1, max_try);
	}
	/* If everything went ok (i.e. 1 msg transmitted), return #bytes */
	/* transmitted, else error code. */
	return (ret == 0) ? count : ret;
}

/********************************************************************/
/*
 * ################
 * ## ALS CONFIG ##
 * ################
 */

static int ltrx130a_als_enable(struct i2c_client *client, int enable)
{
	int err = 0;
	u8 regdata = 0;

	err = ltrx130a_register_read(client, LTRX130A_MAIN_CTRL,
							&regdata, 0x01);
	if (err < 0)
		APS_ERR("i2c error: %d\n", err);

	regdata &= 0xEF;	/* Clear reset bit */

	if (enable != 0) {
		APS_LOG("ALS(1): enable als only\n");
		regdata |= 0x02;

	} else {
		APS_LOG("ALS(1): disable als only\n");
		regdata &= 0xFD;
	}

	err = ltrx130a_register_write(client, LTRX130A_MAIN_CTRL,
							(char *)&regdata, 1);
	if (err < 0) {
		APS_ERR("ALS: enable als err: %d en: %d\n", err, enable);
		return err;
	}

	if (enable != 0)
		msleep(WAKEUP_DELAY);

	return 0;
}
/*--------------------------------------------------------------------------*/
static int ltrx130a_als_read_alsval(struct i2c_client *client)
{
	u8 buf[3];
	int alsval = 0;
	int ret;

	ret = ltrx130a_register_read(client, LTRX130A_ALS_DATA_0, buf, 0x03);
	if (ret < 0) {
		APS_ERR("i2c error: %d\n", ret);
		return 0;
	}

	alsval = (buf[2] * 256 * 256) + (buf[1] * 256) + buf[0];
	APS_DBG("alsval_0 = %d,alsval_1=%d,alsval_2=%d,alsval=%d\n",
			buf[0], buf[1], buf[2], alsval);

	return alsval;
}
/*--------------------------------------------------------------------------*/
static int ltrx130a_als_read_clearval(struct i2c_client *client)
{
	u8 buf[3];
	int clearval = 0;
	int ret;

	ret = ltrx130a_register_read(client, LTRX130A_CLEAR_DATA_0, buf, 0x03);
	if (ret < 0) {
		APS_ERR("i2c error: %d\n", ret);
		return 0;
	}

	clearval = (buf[2] * 256 * 256) + (buf[1] * 256) + buf[0];
	APS_DBG("clearval_0 = %d,clearval_1=%d,clearval_2=%d,clearval=%d\n",
			buf[0], buf[1], buf[2], clearval);

	return clearval;
}


static int ltrx130a_get_winfactor(int ratio)
{
	if (ltrx130a_dual.tp_cg_color == WHITE) {
		if (ratio > 60)
			return ALS_WIN_FACTOR_WHITE_IR;
		else
			return ALS_WIN_FACTOR_WHITE;
	} else {
		if (ratio > 60)
			return ALS_WIN_FACTOR_BLACK_IR;
		else
			return ALS_WIN_FACTOR_BLACK;
	}
}

static int ltrx130a_formula(int alsval, int clearval)
{
	int ratio = 0;
	/*
	 * LTRX13A formula
	 * vendor suggest use alsval * ALS_WIN_FACTOR
	 * because ADC value close to lux value
	 * ALS_MIX_DATA_FORMULA is calculate with clear data.
	 * note. clearval is mix data include ir and visible data
	 */
	if (alsval <= 0)
		return 0;

	if (ALS_MIX_DATA_FORMULA == 1) {

		ratio = (clearval * 100) / (alsval + clearval);

		APS_DBG("(%d) * (%d + %d), ratio = %d\n",
			clearval, alsval, clearval, ratio);

		ltrx130a_dual.als_win_factor = ltrx130a_get_winfactor(ratio);
	}

	return (alsval * ltrx130a_dual.als_win_factor + 50) / 100;
}

/*----------------------------------------------------------------------------*/
static unsigned char *idme_get_alscal_value(void)
{
	struct device_node *ap = NULL;
	char *alscal = NULL;

	ap = of_find_node_by_path(IDME_OF_ALSCAL);
	if (ap) {
		alscal = (char *)of_get_property(ap, "value", NULL);
		APS_LOG("alscal %s\n", alscal);

	} else
		APS_ERR("of_find_node_by_path failed\n");

	return alscal;
}
/*--------------------------------------------------------------------------*/
static int idme_get_tp_cg_color(void)
{
	struct device_node *ap = NULL;
	char *cg_color = NULL;
	int res = -1;

	ap = of_find_node_by_path(IDME_OF_CG_COLOR);
	if (ap) {
		cg_color = (char *)of_get_property(ap, "value", NULL);
		pr_info("tp_cg_color %s\n", cg_color);

	} else {
		pr_err("of_find_node_by_path failed\n");
		return 0;
	}

	if (unlikely(kstrtouint(cg_color, 10, &res)))
		pr_err("idme_get_board_rev kstrtouint failed!\v");

	return res;
}
/*----------------------------------------------------------------------------*/
static int als_cali_set(char *buf)
{
	struct ltrx130a_priv *obj;
	char *token;
	char *alscal;
	const char delim[] = ",";
	/*default value format is 1st low, 1st high, 2nd low, 2nd high*/
	int dual_cal[4] = {20, 400, 20, 400};
	int err = 0;
	int i = 0;

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

	obj = ltrx130a_dual.als[position_1st];

	if (obj) {
		obj->als_cal_low = dual_cal[0];
		obj->als_cal_high = dual_cal[1];

		APS_DBG("cal values bus=%d 1st=low:%d high:%d\n",
			obj->hw->i2c_num,
			obj->als_cal_low,
			obj->als_cal_high);
	}

	obj = ltrx130a_dual.als[position_2nd];

	if (obj) {
		obj->als_cal_low = dual_cal[2];
		obj->als_cal_high = dual_cal[3];

		APS_DBG("cal values bus=%d 2nd=low:%d high%d\n",
			obj->hw->i2c_num,
			obj->als_cal_low,
			obj->als_cal_high);
	}

buf_error:
	kfree(alscal);

error:
	return err;
}
/*----------------------------------------------------------------------------*/
static int ltrx130a_get_dual_cal(void)
{
	char *alscal;
	int err = 0;

	alscal = idme_get_alscal_value();

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
/*--------------------------------------------------------------------------*/
static uint32_t als_calibration(struct ltrx130a_priv *obj, uint32_t alscode)
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
/*--------------------------------------------------------------------------*/
static int ltrx130a_als_read(struct ltrx130a_priv *obj, u32 *data)
{
	int luxdata_int;

	obj->als_code = ltrx130a_als_read_alsval(obj->client);

	obj->mix_code = ltrx130a_als_read_clearval(obj->client);

	if (obj->als_code == 0) {
		luxdata_int = 0;
		goto out;
	}

	luxdata_int = ltrx130a_formula(obj->als_code, obj->mix_code);

	APS_DBG("ltrx130a_als_read: als_value_lux = %d\n", luxdata_int);
out:
	*data = luxdata_int;
	final_lux_val = luxdata_int;
	return luxdata_int;
}

/********************************************************************/
static int ltrx130a_get_als_value(struct ltrx130a_priv *obj, u16 als)
{
	int idx;
	int invalid = 0;

	APS_DBG("als  = %d\n", als);
	for (idx = 0; idx < obj->als_level_num; idx++) {
		if (als < obj->hw->als_level[idx])
			break;
	}

	if (obj->als_value_num > 0 && idx >= obj->als_value_num) {
		APS_ERR("exceed range\n");
		idx = obj->als_value_num - 1;
	}

	if (atomic_read(&ltrx130a_dual.als_deb_on) == 1) {
		unsigned long endt = atomic_read(&ltrx130a_dual.als_deb_end);

		if (time_after(jiffies, endt))
			atomic_set(&ltrx130a_dual.als_deb_on, 0);

		if (atomic_read(&ltrx130a_dual.als_deb_on) == 1)
			invalid = 1;
	}

	if (!invalid) {
		APS_DBG("ALS: %05d => %05d\n", als, obj->hw->als_value[idx]);
		return obj->hw->als_value[idx];
	}
	APS_ERR("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);
	return -1;
}
/*--------------------------------------------------------------------------*/
static inline uint32_t ltrx130a_get_als_reading_avg(struct ltrx130a_priv *obj,
		int sSampleNo)
{
	int res;
	uint16_t ALSData = 0;
	uint16_t DataCount = 0;
	uint32_t sAveAlsData = 0;

	while (DataCount < sSampleNo) {
		msleep(110);

		res = ltrx130a_als_read(obj, &obj->als);
		if (res)
			APS_ERR("get_als_reading_avg:ltrx130x_read_als!!\n");

		ALSData = obj->als;
		APS_DBG("%s: [LTR]als code = %d\n", __func__, ALSData);
		sAveAlsData += ALSData;
		DataCount++;
	}
	sAveAlsData /= sSampleNo;
	return sAveAlsData;
}

/*---------------------attribute file for debugging-------------------------*/

/*****************************************************************************
 * Sysfs attributes
******************************************************************************/
static ssize_t ltrx130a_show_config(struct device_driver *ddri, char *buf)
{
	ssize_t res;

	res = snprintf(buf, PAGE_SIZE, "(%d %d %d %d %d)\n",
		atomic_read(&ltrx130a_dual.i2c_retry),
		atomic_read(&ltrx130a_dual.als_debounce));
	return res;
}
/*--------------------------------------------------------------------------*/
static ssize_t ltrx130a_show_trace(struct device_driver *ddri, char *buf)
{
	ssize_t res;

	res = snprintf(buf, PAGE_SIZE, "%d\n",
		atomic_read(&ltrx130a_dual.trace));
	return res;
}
/*--------------------------------------------------------------------------*/
static ssize_t ltrx130a_store_trace(struct device_driver *ddri, const char *buf, size_t count)
{
	int trace;

	if (sysfs_streq(buf, "1")) {
		trace = 1;
	} else if (sysfs_streq(buf, "2")) {
		trace = 2;
	} else if (sysfs_streq(buf, "0")) {
		trace = 0;
	} else {
		APS_ERR("invalid content: '%s', length = %zu\n", buf, count);
		return -EINVAL;
	}

	atomic_set(&ltrx130a_dual.trace, trace);

	return count;
}
/*--------------------------------------------------------------------------*/
static int ltrx130a_show_als(struct ltrx130a_priv *obj)
{
	int res;

	if (!obj) {
		APS_ERR("obj is null!!\n");
		return 0;
	}
	res = ltrx130a_als_read(obj, &obj->als);

	return res;
}

static ssize_t als_show_1st(struct device_driver *ddri, char *buf)
{
	struct ltrx130a_priv *obj = ltrx130a_dual.als[position_1st];
	int res = -1;

	if (!obj)
		return scnprintf(buf, PAGE_SIZE, ALS_NOT_EXSIT);

	res = ltrx130a_show_als(obj);

	return scnprintf(buf, PAGE_SIZE,
		"als_value = %5d, als_clear = NA(LTRX130ANA)\n",
		res, obj->mix_code);
}

static ssize_t als_show_2nd(struct device_driver *ddri, char *buf)
{
	struct ltrx130a_priv *obj = ltrx130a_dual.als[position_2nd];
	int res = -1;

	if (!obj)
		return scnprintf(buf, PAGE_SIZE, ALS_NOT_EXSIT);

	res = ltrx130a_show_als(obj);

	return scnprintf(buf, PAGE_SIZE,
		"als_value = %5d, als_clear = NA(LTRX130ANA)\n",
		res, obj->mix_code);
}
/*--------------------------------------------------------------------------*/
static void ltrx130a_show_als_test(struct ltrx130a_priv *obj,
				uint32_t *als_reading, uint32_t *als_value)
{
	APS_LOG("%s: [LTR]Start testing light...\n", __func__);

	msleep(110);
	*als_reading = ltrx130a_get_als_reading_avg(obj, 5);
	APS_LOG("%s: [LTR]als_reading = %d\n", __func__, *als_reading);

	*als_value = als_calibration(obj, *als_reading);

	APS_LOG("%s: [LTR]Start testing light done!!! als_value = %d\n",
			__func__, *als_value);
	APS_LOG("%s: [LTR]Start testing light done!!! als_test_adc = %d\n",
			__func__, *als_reading);

}

static ssize_t als_show_test_1st(struct device_driver *ddri, char *buf)
{
	int als_reading = 0;
	unsigned int als_value_1st;		/* lux value in test level*/
	struct ltrx130a_priv *obj =
		ltrx130a_dual.als[position_1st];

	if (!obj)
		return scnprintf(buf, PAGE_SIZE, ALS_NOT_EXSIT);

	ltrx130a_show_als_test(obj,
					&als_reading, &als_value_1st);

	return scnprintf(buf, PAGE_SIZE,
			"1st als_value = %5d lux, cci_als_test_adc = %5d\n",
			als_value_1st, als_reading);
}

static ssize_t als_show_test_2nd(struct device_driver *ddri, char *buf)
{
	int als_reading = 0;
	unsigned int als_value_2nd;		/* lux value in test level*/
	struct ltrx130a_priv *obj =
		ltrx130a_dual.als[position_2nd];

	if (!obj)
		return scnprintf(buf, PAGE_SIZE, ALS_NOT_EXSIT);

	ltrx130a_show_als_test(obj,
					&als_reading, &als_value_2nd);

	return scnprintf(buf, PAGE_SIZE,
			"2nd als_value = %5d lux, cci_als_test_adc = %5d\n",
			als_value_2nd, als_reading);
}
/*--------------------------------------------------------------------------*/
static ssize_t ltrx130a_show_reg(struct ltrx130a_priv *obj, char *buf)
{
	int i, len = 0;
	int reg[] = {0x00, 0x04, 0x05, 0x06, 0x07, 0x0a, 0x0b, 0x0c,
				0x0d, 0x0e, 0x0f, 0x19, 0x1A, 0x21, 0x22, 0x23, 0x24,
				0x25, 0x26};
	int ret;
	u8 buffer;

	for (i = 0; i < 19; i++) {
		ret = ltrx130a_register_read(obj->client, reg[i],
								&buffer, 0x01);
		if (ret < 0)
			APS_ERR("i2c error: %d\n", ret);

		len += snprintf(buf + len, PAGE_SIZE - len,
				"reg:0x%04X value: 0x%04X\n", reg[i], buffer);
	}
	return len;
}

static ssize_t als_show_reg_1st(struct device_driver *ddri, char *buf)
{
	int len = 0;
	struct ltrx130a_priv *obj = ltrx130a_dual.als[position_1st];

	if (!obj)
		return scnprintf(buf, PAGE_SIZE, ALS_NOT_EXSIT);

	len = ltrx130a_show_reg(obj, buf);
	return len;
}

static ssize_t als_show_reg_2nd(struct device_driver *ddri, char *buf)
{
	int len = 0;
	struct ltrx130a_priv *obj = ltrx130a_dual.als[position_2nd];

	if (!obj)
		return scnprintf(buf, PAGE_SIZE, ALS_NOT_EXSIT);

	len = ltrx130a_show_reg(obj, buf);
	return len;
}

#ifdef LTRX130A_DEBUG
static int ltrx130a_dump_reg(struct ltrx130a_priv *obj)
{
	int i = 0;
	int reg[] = {0x00, 0x04, 0x05, 0x06, 0x07, 0x0a, 0x0b, 0x0c,
				0x0d, 0x0e, 0x0f, 0x19, 0x1A, 0x21, 0x22, 0x23,
				0x24, 0x25, 0x26};
	int ret;
	u8 buffer;

	for (i = 0; i < 19; i++) {
		ret = ltrx130a_register_read(obj->client, reg[i],
								&buffer, 0x01);
		if (ret < 0)
			APS_ERR("i2c error: %d\n", ret);

		APS_DBG("reg:0x%04X value: 0x%04X\n", reg[i], buffer);
	}
	return 0;
}
#endif

/*--------------------------------------------------------------------------*/
static ssize_t ltrx130a_show_status(struct ltrx130a_priv *obj, char *buf)
{
	ssize_t len = 0;

	if (!obj) {
		APS_ERR("obj is null!!\n");
		return 0;
	}

	if (obj->hw)
		len += snprintf(buf+len, PAGE_SIZE-len,
			"CUST: %d, %d, (%d %d)\n",
			obj->hw->i2c_num, obj->sensor_num,
			obj->hw->power_id, obj->hw->power_vol);
	else
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");

	len += snprintf(buf+len, PAGE_SIZE-len, "MISC: %d\n",
			atomic_read(&ltrx130a_dual.als_suspend));

	return len;
}

static ssize_t als_show_status_1st(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	struct ltrx130a_priv *obj = ltrx130a_dual.als[position_1st];

	if (!obj)
		return scnprintf(buf, PAGE_SIZE, ALS_NOT_EXSIT);

	len = ltrx130a_show_status(obj, buf);

	return len;

}
static ssize_t als_show_status_2nd(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	struct ltrx130a_priv *obj = ltrx130a_dual.als[position_2nd];

	if (!obj)
		return scnprintf(buf, PAGE_SIZE, ALS_NOT_EXSIT);

	len = ltrx130a_show_status(obj, buf);

	return len;
}

/*--------------------------------------------------------------------------*/
#define IS_SPACE(CH) (((CH) == ' ') || ((CH) == '\n'))
/*--------------------------------------------------------------------------*/
static int read_int_from_buf(struct ltrx130a_priv *obj, const char *buf, size_t count, u32 data[], int len)
{
	int idx = 0;
	char *cur = (char *)buf, *end = (char *)(buf+count);

	while (idx < len) {
		while ((cur < end) && IS_SPACE(*cur))
			cur++;

		if (sscanf(cur, "%d", &data[idx]) != 1)
			break;

		idx++;
		while ((cur < end) && !IS_SPACE(*cur))
			cur++;
	}
	return idx;
}
/*--------------------------------------------------------------------------*/
static ssize_t ltrx130a_show_alslv(struct ltrx130a_priv *obj, char *buf)
{
	ssize_t len = 0;
	int idx;

	if (!obj) {
		APS_ERR("obj is null!!\n");
		return 0;
	}

	for (idx = 0; idx < obj->als_level_num; idx++)
		len += snprintf(buf+len, PAGE_SIZE-len, "%d ", obj->hw->als_level[idx]);

	len += snprintf(buf+len, PAGE_SIZE-len, "\n");
	return len;
}

static ssize_t als_show_alslv_1st(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	struct ltrx130a_priv *obj = ltrx130a_dual.als[position_1st];

	if (!obj)
		return scnprintf(buf, PAGE_SIZE, ALS_NOT_EXSIT);

	len = ltrx130a_show_alslv(obj, buf);

	return len;
}

static ssize_t als_show_alslv_2nd(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	struct ltrx130a_priv *obj = ltrx130a_dual.als[position_2nd];

	if (!obj)
		return scnprintf(buf, PAGE_SIZE, ALS_NOT_EXSIT);

	len = ltrx130a_show_alslv(obj, buf);

	return len;
}

/*--------------------------------------------------------------------------*/
static ssize_t ltrx130a_store_alslv(struct ltrx130a_priv *obj, const char *buf, size_t count)
{
	if (!obj) {
		APS_ERR("obj is null!!\n");
		return 0;
	} else if (!strcmp(buf, "def")) {
		memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
	} else if (obj->als_level_num != read_int_from_buf(obj, buf, count,
			obj->hw->als_level, obj->als_level_num)) {
		APS_ERR("invalid format: '%s'\n", buf);
	}
	return count;
}

static ssize_t als_store_alslv_1st(struct device_driver *ddri, const char *buf, size_t count)
{
	struct ltrx130a_priv *obj = ltrx130a_dual.als[position_2nd];

	if (!obj) {
		APS_ERR(ALS_NOT_EXSIT);
		return -ENAVAIL;
	}

	return ltrx130a_store_alslv(obj, buf, count);
}

static ssize_t als_store_alslv_2nd(struct device_driver *ddri, const char *buf, size_t count)
{
	struct ltrx130a_priv *obj = ltrx130a_dual.als[position_2nd];

	if (!obj) {
		APS_ERR(ALS_NOT_EXSIT);
		return -ENAVAIL;
	}

	return ltrx130a_store_alslv(obj, buf, count);
}
/*--------------------------------------------------------------------------*/
static ssize_t ltrx130a_show_alsval(struct ltrx130a_priv *obj, char *buf)
{
	ssize_t len = 0;
	int idx;

	if (!obj) {
		APS_ERR("obj is null!!\n");
		return 0;
	}

	for (idx = 0; idx < obj->als_value_num; idx++)
		len += snprintf(buf+len, PAGE_SIZE-len, "%d ", obj->hw->als_value[idx]);

	len += snprintf(buf+len, PAGE_SIZE-len, "\n");
	return len;
}

static ssize_t als_show_alsval_1st(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	struct ltrx130a_priv *obj = ltrx130a_dual.als[position_1st];

	if (!obj)
		return scnprintf(buf, PAGE_SIZE, ALS_NOT_EXSIT);

	len = ltrx130a_show_alsval(obj, buf);

	return len;
}

static ssize_t als_show_alsval_2nd(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	struct ltrx130a_priv *obj = ltrx130a_dual.als[position_2nd];

	if (!obj)
		return scnprintf(buf, PAGE_SIZE, ALS_NOT_EXSIT);

	len = ltrx130a_show_alsval(obj, buf);

	return len;
}

/*--------------------------------------------------------------------------*/
static ssize_t ltrx130a_store_alsval(struct ltrx130a_priv *obj, const char *buf, size_t count)
{
	if (!obj) {
		APS_ERR("obj is null!!\n");
		return 0;
	} else if (!strcmp(buf, "def")) {
		memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
	} else if (obj->als_value_num != read_int_from_buf(obj, buf, count,
			obj->hw->als_value, obj->als_value_num)) {
		APS_ERR("invalid format: '%s'\n", buf);
	}
	return count;
}

static ssize_t als_store_alsval_1st(struct device_driver *ddri, const char *buf, size_t count)
{
	struct ltrx130a_priv *obj = ltrx130a_dual.als[position_1st];

	if (!obj) {
		APS_ERR(ALS_NOT_EXSIT);
		return -ENAVAIL;
	}

	return ltrx130a_store_alsval(obj, buf, count);

}

static ssize_t als_store_alsval_2nd(struct device_driver *ddri, const char *buf, size_t count)
{
	struct ltrx130a_priv *obj = ltrx130a_dual.als[position_2nd];

	if (!obj) {
		APS_ERR(ALS_NOT_EXSIT);
		return -ENAVAIL;
	}

	return ltrx130a_store_alsval(obj, buf, count);
}
/*--------------------------------------------------------------------------*/
static int ltrx130a_store_als_enable(struct ltrx130a_priv *obj, const char *buf)
{
	uint8_t en;
	int res;

	if (sysfs_streq(buf, "1"))
		en = 1;
	else if (sysfs_streq(buf, "0"))
		en = 0;
	else {
		APS_ERR("%s, invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}
	APS_LOG("%s: Enable ALS : %d\n", __func__, en);

	res = ltrx130a_als_enable(obj->client, en);

	return res;
}

static ssize_t store_als_enable_1st(struct device_driver *ddri, const char *buf, size_t size)
{
	struct ltrx130a_priv *obj = ltrx130a_dual.als[position_1st];
	int res = 0;

	if (!obj) {
		APS_ERR(ALS_NOT_EXSIT);
		return -ENAVAIL;
	}

	res = ltrx130a_store_als_enable(obj, buf);

	if (res < 0)
		APS_ERR("enable 1st als fail: %d\n", res);

	return size;
}

static ssize_t store_als_enable_2nd(struct device_driver *ddri, const char *buf, size_t size)
{
	struct ltrx130a_priv *obj = ltrx130a_dual.als[position_2nd];
	int res = 0;

	if (!obj) {
		APS_ERR(ALS_NOT_EXSIT);
		return -ENAVAIL;
	}

	res = ltrx130a_store_als_enable(obj, buf);
	if (res < 0)
		APS_ERR("enable 2nd als fail: %d\n", res);

	return size;
}
/*----------------------------------------------------------------------------*/
static int ltrx130a_show_als_enable(struct ltrx130a_priv *obj, int32_t *enable)
{
	u8 r_buf;
	int ret;

	ret = ltrx130a_register_read(obj->client,
					LTRX130A_MAIN_CTRL, &r_buf, 0x01);
	if (ret < 0) {
		APS_ERR("error: %d\n", ret);
		return -EFAULT;
	}

	*enable = (r_buf & LTR_STATE_EN_ALS_MASK) ? 1 : 0;

	return ret;
}

static ssize_t show_als_enable_1st(struct device_driver *ddri, char *buf)
{
	int32_t enable;
	int ret;
	struct ltrx130a_priv *obj = ltrx130a_dual.als[position_1st];

	if (!obj)
		return scnprintf(buf, PAGE_SIZE, ALS_NOT_EXSIT);

	ret = ltrx130a_show_als_enable(obj, &enable);

	if (ret == -EFAULT)
		return ret;

	return scnprintf(buf, PAGE_SIZE, "%d\n", enable);
}

static ssize_t show_als_enable_2nd(struct device_driver *ddri, char *buf)
{
	int32_t enable;
	int ret;
	struct ltrx130a_priv *obj = ltrx130a_dual.als[position_2nd];

	if (!obj)
		return scnprintf(buf, PAGE_SIZE, ALS_NOT_EXSIT);

	ret = ltrx130a_show_als_enable(obj, &enable);

	if (ret == -EFAULT)
		return ret;

	return scnprintf(buf, PAGE_SIZE, "%d\n", enable);
}
/*--------------------------------------------------------------------------*/
static ssize_t ltrx130a_show_als_cali_1st(struct device_driver *ddri, char *buf)
{
	struct ltrx130a_priv *obj = ltrx130a_dual.als[position_1st];
	int als_adc_cali = 0;

	if (!obj)
		return scnprintf(buf, PAGE_SIZE, ALS_NOT_EXSIT);

	als_adc_cali = ltrx130a_get_als_reading_avg(obj, 5);

	return scnprintf(buf, PAGE_SIZE,
		"adc_cali=%d\n",
		als_adc_cali);
}
/*--------------------------------------------------------------------------*/
static ssize_t ltrx130a_show_als_cali_2nd(struct device_driver *ddri, char *buf)
{
	struct ltrx130a_priv *obj = ltrx130a_dual.als[position_2nd];
	int als_adc_cali = 0;

	if (!obj)
		return scnprintf(buf, PAGE_SIZE, ALS_NOT_EXSIT);

	als_adc_cali = ltrx130a_get_als_reading_avg(obj, 5);

	return scnprintf(buf, PAGE_SIZE,
		"adc_cali=%d\n",
		als_adc_cali);
}
/*--------------------------------------------------------------------------*/
static ssize_t als_show_cali_value(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	struct ltrx130a_priv *obj = NULL;

	obj = ltrx130a_dual.als[position_1st];

	if (obj) {
		len += scnprintf(buf, PAGE_SIZE,
			"1st 20lux cali = %d, 1st 400lux cali = %d\n",
			obj->als_cal_low,
			obj->als_cal_high);
	}

	obj = ltrx130a_dual.als[position_2nd];

	if (obj) {
		len += scnprintf(buf + len, PAGE_SIZE - len,
			"2nd 20lux cali = %d, 2nd 400lux cali = %d\n",
			obj->als_cal_low,
			obj->als_cal_high);

	}
	return len;

}
/*--------------------------------------------------------------------------*/
static ssize_t als_store_cali_value(struct device_driver *ddri,
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
/*--------------------------------------------------------------------------*/
static DRIVER_ATTR(als_1,     S_IWUSR | S_IRUGO, als_show_1st,               NULL);
static DRIVER_ATTR(als_2,     S_IWUSR | S_IRUGO, als_show_2nd,               NULL);
static DRIVER_ATTR(alstest_1, S_IWUSR | S_IRUGO, als_show_test_1st,          NULL);
static DRIVER_ATTR(alstest_2, S_IWUSR | S_IRUGO, als_show_test_2nd,          NULL);
static DRIVER_ATTR(alslv_1,   S_IWUSR | S_IRUGO, als_show_alslv_1st,         als_store_alslv_1st);
static DRIVER_ATTR(alslv_2,   S_IWUSR | S_IRUGO, als_show_alslv_2nd,         als_store_alslv_2nd);
static DRIVER_ATTR(alsval_1,  S_IWUSR | S_IRUGO, als_show_alsval_1st,        als_store_alsval_1st);
static DRIVER_ATTR(alsval_2,  S_IWUSR | S_IRUGO, als_show_alsval_2nd,        als_store_alsval_2nd);

static DRIVER_ATTR(status_1,  S_IWUSR | S_IRUGO, als_show_status_1st,        NULL);
static DRIVER_ATTR(status_2,  S_IWUSR | S_IRUGO, als_show_status_2nd,        NULL);

static DRIVER_ATTR(trace,     S_IWUSR | S_IRUGO, ltrx130a_show_trace,        ltrx130a_store_trace);
static DRIVER_ATTR(config,    S_IWUSR | S_IRUGO, ltrx130a_show_config,       NULL);
static DRIVER_ATTR(calival,    S_IWUSR | S_IRUGO, als_show_cali_value,       als_store_cali_value);

static DRIVER_ATTR(reg_1,     S_IWUSR | S_IRUGO, als_show_reg_1st,           NULL);
static DRIVER_ATTR(reg_2,     S_IWUSR | S_IRUGO, als_show_reg_2nd,           NULL);
static DRIVER_ATTR(enable_1,  S_IWUSR | S_IRUGO, show_als_enable_1st,        store_als_enable_1st);
static DRIVER_ATTR(enable_2,  S_IWUSR | S_IRUGO, show_als_enable_2nd,        store_als_enable_2nd);
static DRIVER_ATTR(alscali_1, S_IWUSR | S_IRUGO, ltrx130a_show_als_cali_1st, NULL);
static DRIVER_ATTR(alscali_2, S_IWUSR | S_IRUGO, ltrx130a_show_als_cali_2nd, NULL);
/*--------------------------------------------------------------------------*/
static struct driver_attribute *ltrx130a_attr_list_common[] = {
	&driver_attr_trace,        /*trace log*/
	&driver_attr_config,
	&driver_attr_calival,

};

static struct driver_attribute *ltrx130a_attr_list_als_1[] = {
	&driver_attr_als_1,
	&driver_attr_alstest_1,
	&driver_attr_alslv_1,
	&driver_attr_alsval_1,
	&driver_attr_reg_1,
	&driver_attr_status_1,
	&driver_attr_enable_1,
	&driver_attr_alscali_1,
};

static struct driver_attribute *ltrx130a_attr_list_als_2[] = {
	&driver_attr_als_2,
	&driver_attr_alstest_2,
	&driver_attr_alslv_2,
	&driver_attr_alsval_2,
	&driver_attr_status_2,
	&driver_attr_reg_2,
	&driver_attr_enable_2,
	&driver_attr_alscali_2,
};
/*--------------------------------------------------------------------------*/
static int ltrx130a_create_attr(struct device_driver *driver,
				struct driver_attribute **attr_list, int num)
{
	int idx, err = 0;

	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++) {
		err = driver_create_file(driver, attr_list[idx]);
		if (err) {
			APS_ERR("driver_create_file (%s) = %d\n",
					attr_list[idx]->attr.name,
					err);
			break;
		}
	}
	return err;
}
/*--------------------------------------------------------------------------*/
static int ltrx130a_delete_attr(struct device_driver *driver,
				struct driver_attribute **attr_list, int num)
{
	int idx, err = 0;

	if (!driver)
		return -EINVAL;

	for (idx = 0; idx < num; idx++)
		driver_remove_file(driver, attr_list[idx]);

	return err;
}
static int dualals_create_node(int sensor_num)
{
	int err = 0;
	int num = 0;

	if (sensor_num == 1) {
		num = (int)(sizeof(ltrx130a_attr_list_als_1) /
			sizeof(ltrx130a_attr_list_als_1[0]));

		err =
		ltrx130a_create_attr(
			&(ltrx130a_init_info.platform_diver_addr->driver),
			ltrx130a_attr_list_als_1,
			num);
	} else {
		num = (int)(sizeof(ltrx130a_attr_list_als_2) /
			sizeof(ltrx130a_attr_list_als_2[0]));

		err =
		ltrx130a_create_attr(
			&(ltrx130a_init_info.platform_diver_addr->driver),
			ltrx130a_attr_list_als_2,
			7);
	}

	if (err) {
		APS_ERR("create attribute err = %d\n", err);
		return -ENAVAIL;
	}

	return 0;
}

static int dualals_delete_node(void)
{
	int err = 0;
	int num = 0;
	struct ltrx130a_priv *obj;

	obj = ltrx130a_dual.als[position_1st];
	if (obj) {
		num = (int)(sizeof(ltrx130a_attr_list_als_1) /
			sizeof(ltrx130a_attr_list_als_1[0]));

		err =
		ltrx130a_delete_attr(
			&(ltrx130a_init_info.platform_diver_addr->driver),
			ltrx130a_attr_list_als_1,
			num);
	}

	obj = ltrx130a_dual.als[position_2nd];
	if (obj) {
		num = (int)(sizeof(ltrx130a_attr_list_als_2) /
			sizeof(ltrx130a_attr_list_als_2[0]));

		err =
		ltrx130a_delete_attr(
			&(ltrx130a_init_info.platform_diver_addr->driver),
			ltrx130a_attr_list_als_2,
			num);
	}

	if (err) {
		APS_ERR("create attribute err = %d\n", err);
		return -ENAVAIL;
	}

	num = (int)(sizeof(ltrx130a_attr_list_common) /
			sizeof(ltrx130a_attr_list_common[0]));

	err =
	ltrx130a_delete_attr(
		&(ltrx130a_init_info.platform_diver_addr->driver),
		ltrx130a_attr_list_common,
		num);

	if (err) {
		APS_ERR("create attribute err = %d\n", err);
		return -ENAVAIL;
	}

	return 0;
}
/*----------------------------------------------------------------------------*/
static int common_delete_node(void)
{
	int num = 0;
	int err = 0;

	APS_LOG("delete common node\n");

	num = (int)(sizeof(ltrx130a_attr_list_common) /
			sizeof(ltrx130a_attr_list_common[0]));

	err =
	ltrx130a_delete_attr(
		&(ltrx130a_init_info.platform_diver_addr->driver),
		ltrx130a_attr_list_common,
		num);

	if (err) {
		APS_ERR("delete common attribute err = %d\n", err);
		return -EIO;
	}

	return 0;
}
/****************************************************************************/

/*--------------------------MISC device related-----------------------------*/
static int ltrx130a_open(struct inode *inode, struct file *file)
{
	file->private_data = ltrx130a_dual.als[MASTER]->client;

	if (!file->private_data) {
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}
/************************************************************/
static int ltrx130a_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}
/************************************************************/
static long ltrx130a_unlocked_ioctl(struct file *file, unsigned int cmd,
	   unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client *)file->private_data;
	struct ltrx130a_priv *obj = i2c_get_clientdata(client);
	int err = 0;
	void __user *ptr = (void __user *) arg;
	int dat;
	uint32_t enable;

	APS_DBG("cmd= %d\n", cmd);
	switch (cmd) {
	case ALSPS_SET_ALS_MODE:
		if (copy_from_user(&enable, ptr, sizeof(enable))) {
			err = -EFAULT;
			goto err_out;
		}
		err = ltrx130a_als_enable(obj->client, enable);
		if (err < 0) {
			APS_ERR("enable als fail: %d en: %d\n", err, enable);
			goto err_out;
		}
		if (enable)
			set_bit(CMC_BIT_ALS, &obj->enable);
		else
			clear_bit(CMC_BIT_ALS, &obj->enable);
		break;

	case ALSPS_GET_ALS_MODE:
		enable = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);
		if (copy_to_user(ptr, &enable, sizeof(enable))) {
			err = -EFAULT;
			goto err_out;
		}
		break;

	case ALSPS_GET_ALS_DATA:
		obj->als = ltrx130a_als_read(obj, &obj->als);
		if (obj->als < 0)
			goto err_out;

		dat = ltrx130a_get_als_value(obj, obj->als);
		if (copy_to_user(ptr, &dat, sizeof(dat))) {
			err = -EFAULT;
			goto err_out;
		}
		break;

	case ALSPS_GET_ALS_RAW_DATA:
		obj->als = ltrx130a_als_read(obj, &obj->als);
		if (obj->als < 0)
			goto err_out;

		dat = obj->als;
		if (copy_to_user(ptr, &dat, sizeof(dat))) {
			err = -EFAULT;
			goto err_out;
		}
		break;

	default:
		err = -ENOIOCTLCMD;
		break;
	}

err_out:
	return err;
}
/********************************************************************/
/*-----------misc device related operation functions------------------------*/
static const struct file_operations ltrx130a_fops = {
	.owner = THIS_MODULE,
	.open = ltrx130a_open,
	.release = ltrx130a_release,
	.unlocked_ioctl = ltrx130a_unlocked_ioctl,
};

static struct miscdevice ltrx130a_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als",
	.fops = &ltrx130a_fops,
};

/*--------------------------------------------------------------------------*/
static int ltrx130a_init_client(struct ltrx130a_priv *obj)
{
	int res;
	u8 buf;

	struct i2c_client *client = obj->client;

	msleep(PON_DELAY);

	/* ===============
	* ** IMPORTANT **
	* ===============
	* Other settings like timing and threshold to be set here, if required.
	* Not set and kept as device default for now.
	*/
#ifdef LTRX130A_DEBUG
	ltrx130a_dump_reg();
#endif

	/* Enable ALS to Full Range at startup */
	als_gainrange = ALS_RANGE_9;/*Set global variable*/
	APS_ERR("ALS sensor gainrange %d!\n", als_gainrange);

	switch (als_gainrange) {
	case ALS_RANGE_1:
		buf = MODE_ALS_Range1;
		res = ltrx130a_register_write(client,
							LTRX130A_ALS_GAIN,
							(char *)&buf,
							1);
		break;

	case ALS_RANGE_3:
		buf = MODE_ALS_Range3;
		res = ltrx130a_register_write(client,
							LTRX130A_ALS_GAIN,
							(char *)&buf,
							1);
		break;

	case ALS_RANGE_6:
		buf = MODE_ALS_Range6;
		res = ltrx130a_register_write(client,
							LTRX130A_ALS_GAIN,
							(char *)&buf,
							1);
		break;

	case ALS_RANGE_9:
		buf = MODE_ALS_Range9;
		res = ltrx130a_register_write(client,
							LTRX130A_ALS_GAIN,
							(char *)&buf,
							1);
		break;

	case ALS_RANGE_18:
		buf = MODE_ALS_Range18;
		res = ltrx130a_register_write(client,
							LTRX130A_ALS_GAIN,
							(char *)&buf,
							1);
		break;

	default:
		buf = MODE_ALS_Range3;
		res = ltrx130a_register_write(client,
							LTRX130A_ALS_GAIN,
							(char *)&buf,
							1);
		break;
	}

	if (res < 0) {
		APS_ERR("init dev: %d\n", res);
		return 1;
	}

	buf = ALS_RESO_MEAS;	/* 18 bit & 100ms measurement rate */
	res = ltrx130a_register_write(client,
							LTRX130A_ALS_MEAS_RATE,
							(char *)&buf,
							1);
	APS_ERR("ALS sensor resolution & measurement rate: %d!\n", ALS_RESO_MEAS);

	return 0;

}
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/* if use  this typ of enable , Gsensor should report
 * inputEvent(x, y, z ,stats, div) to HAL
 */
static int als_open_report_data(int open)
{
	/*should queuq work to report event if  is_report_input_direct=true*/
	return 0;
}

/* if use  this typ of enable , Gsensor only enabled but not report
 * inputEvent to HAL
 */
static int als_enable_nodata(int en)
{
	int res = 0;

	APS_LOG("als enable value = %d\n", en);


	for (int i = 0; i < ltrx130a_dual.als_count; i++) {
		res = ltrx130a_als_enable(ltrx130a_dual.als[i]->client, en);
		if (res) {
			APS_ERR("als_enable_nodata is failed!!\n");
			return -1;
		}

		mutex_lock(&ltrx130a_mutex);
		if (en)
			set_bit(CMC_BIT_ALS, &ltrx130a_dual.als[i]->enable);
		else
			clear_bit(CMC_BIT_ALS, &ltrx130a_dual.als[i]->enable);
		mutex_unlock(&ltrx130a_mutex);
	}

	return 0;
}

static int als_set_delay(u64 ns)
{
	/* Do nothing */
	return 0;
}

static int als_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
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
/*--------------------------------------------------------------------------*/
static void als_debug_log(int *lux_cali, int als_report)
{
	if (atomic_read(&ltrx130a_dual.trace) & LTR_TRC_DEBUG) {
		if (ltrx130a_dual.als[position_1st])
			APS_LOG("ALS_1_DATA=%d,%d,%d,%d\n",
				lux_cali[position_1st],
				ltrx130a_dual.als[position_1st]->als,
				ltrx130a_dual.als[position_1st]->als_code,
				ltrx130a_dual.als[position_1st]->mix_code);

		if (ltrx130a_dual.als[position_2nd])
			APS_LOG("ALS_2_DATA=%d,%d,%d,%d\n",
				lux_cali[position_2nd],
				ltrx130a_dual.als[position_2nd]->als,
				ltrx130a_dual.als[position_2nd]->als_code,
				ltrx130a_dual.als[position_2nd]->mix_code);
	}

	if (atomic_read(&ltrx130a_dual.trace) & LTR_TRC_ALS_DATA) {
		if (ltrx130a_dual.dual_als_enable) {
			APS_LOG(
				"ALS_DATA=%d,%s,%d,%d,%d,%d,%d,%d,%d,%s,%d,%d,%d,%d,%d,%d,%d\n",
				als_report,
				"1st",
				lux_cali[position_1st],
				ltrx130a_dual.als[position_1st]->als,
				ltrx130a_dual.als[position_1st]->als_code,
				ltrx130a_dual.als[position_1st]->mix_code,
				ltrx130a_dual.als_win_factor,
				ltrx130a_dual.als[position_1st]->als_cal_low,
				ltrx130a_dual.als[position_1st]->als_cal_high,
				"2nd",
				lux_cali[position_2nd],
				ltrx130a_dual.als[position_2nd]->als,
				ltrx130a_dual.als[position_2nd]->als_code,
				ltrx130a_dual.als[position_2nd]->mix_code,
				ltrx130a_dual.als_win_factor,
				ltrx130a_dual.als[position_2nd]->als_cal_low,
				ltrx130a_dual.als[position_2nd]->als_cal_high);
		} else {
			if (ltrx130a_dual.als[position_1st])
				APS_LOG("ALS_1_DATA=%s,%d,%d,%d,%d,%d,%d,%d\n",
					"1st",
					lux_cali[position_1st],
					ltrx130a_dual.als[position_1st]->als,
					ltrx130a_dual.als[position_1st]->als_code,
					ltrx130a_dual.als[position_1st]->mix_code,
					ltrx130a_dual.als_win_factor,
					ltrx130a_dual.als[position_1st]->als_cal_low,
					ltrx130a_dual.als[position_1st]->als_cal_high);

			if (ltrx130a_dual.als[position_2nd])
				APS_LOG("ALS_2_DATA=%s,%d,%d,%d,%d,%d,%d,%d\n",
					"2nd",
					lux_cali[position_2nd],
					ltrx130a_dual.als[position_2nd]->als,
					ltrx130a_dual.als[position_2nd]->als_code,
					ltrx130a_dual.als[position_2nd]->mix_code,
					ltrx130a_dual.als_win_factor,
					ltrx130a_dual.als[position_2nd]->als_cal_low,
					ltrx130a_dual.als[position_2nd]->als_cal_high);
		}
	}

}
/*--------------------------------------------------------------------------*/
static int als_get_lux(int *value)
{
	int alslux[2] = {0};

	for (int i = MASTER; i < ltrx130a_dual.als_count; i++) {
		struct ltrx130a_priv *obj = ltrx130a_dual.als[i];

		if (!obj) {
			APS_ERR("obj is null!!\n");
			return -1;
		}

		alslux[i] = ltrx130a_als_read(obj, &obj->als);

		if (alslux[i] < 0)
			return -1;
		else
			alslux[i] = als_calibration(obj, obj->als);

	}

	*value = als_select_data(alslux[MASTER], alslux[SLAVE]);

	als_debug_log(alslux, *value);

	return 0;
}

static int als_get_data(int *value, int *status)
{
	int err = 0;

	err = als_get_lux(value);

	if (!err)
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	return err;
}
/*--------------------------------------------------------------------------*/
static bool check_is_master(struct device_node *node)
{
	u32 is_master[] = {0};
	int ret = 0;

	ret = of_property_read_u32_array(node, "is_master", is_master,
	ARRAY_SIZE(is_master));

	if (ret)
		APS_ERR("get is_master failed!\n");

	if (is_master[0])
		return true;

	return false;
}
/*--------------------------------------------------------------------------*/
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
/*----------------------------------------------------------------------------*/
static void als_position_init(struct ltrx130a_priv *obj, uint8_t sensors)
{
	if (obj->sensor_num == 1)
		position_1st = sensors;
	else
		position_2nd = sensors;
}
/*--------------------------i2c operations----------------------------------*/
static int ltrx130a_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ltrx130a_priv *obj = NULL;
	struct alsps_hw *alsps_cust = NULL;
	struct als_control_path als_ctl = {0};
	struct als_data_path als_data = {0};
	int err = 0;

	ltrx130a_init_flag = -1; /* 0<==>OK -1 <==> fail */

	APS_FUN();

	alsps_cust = kzalloc(sizeof(*alsps_cust), GFP_KERNEL);
	if (!alsps_cust) {
		err = -ENOMEM;
		goto exit;
	}
	memset(alsps_cust, 0, sizeof(*alsps_cust));


	/* get customization and power on */
	err = get_alsps_dts_func(client->dev.of_node, alsps_cust);
	if (err < 0) {
		APS_ERR("get customization info from dts failed\n");
		return -EFAULT;
	}

	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (!obj) {
		err = -ENOMEM;
		goto exit;
	}
	memset(obj, 0, sizeof(*obj));

	obj->hw = alsps_cust;

	client->addr = *alsps_cust->i2c_addr;
	obj->client = client;
	i2c_set_clientdata(client, obj);
	/*----------value need to be confirmed----------*/

	obj->irq_node = client->dev.of_node;

	obj->enable = 0;

	obj->als_level_num = ARRAY_SIZE(obj->hw->als_level);
	obj->als_value_num = ARRAY_SIZE(obj->hw->als_value);
	obj->als_correct_factor = 0;
	/*----------value need to be confirmed----------*/

	BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
	memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
	BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
	memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
	atomic_set(&ltrx130a_dual.i2c_retry, 3);
	atomic_set(&obj->als_cmd_val, 0xDF);

	clear_bit(CMC_BIT_ALS, &obj->enable);


	APS_LOG("ltrx130a_init_client() start...!\n");

	err = ltrx130a_init_client(obj);
	if (err)
		goto exit_init_failed;

	APS_LOG("ltrx130a_init_client() OK!\n");

	err = read_sensor_num(client->dev.of_node, &obj->sensor_num);
	if (err) {
		APS_ERR("read sensor number fail! %d\n", err);
		goto exit_init_failed;
	}

	if (check_is_master(client->dev.of_node)) {
		ltrx130a_dual.als[MASTER] = obj;
		als_position_init(obj, MASTER);
	} else {
		ltrx130a_dual.als[SLAVE] = obj;
		als_position_init(obj, SLAVE);
	}

	ltrx130a_dual.als_count++;
	APS_LOG("als_count = %d\n", ltrx130a_dual.als_count);

	if (ltrx130a_dual.als_count > 1) {

		err = dualals_create_node(obj->sensor_num);
		if (err)
			goto exit_2nd_dualals_node_fail;

		ltrx130a_dual.dual_als_enable = 1;
		APS_LOG("dual sensor enable!\n");
		ltrx130a_init_flag = 0;
		return 0;
	}

	err = misc_register(&ltrx130a_device);
	if (err) {
		APS_ERR("ltrx130a_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	err =
	ltrx130a_create_attr(
		&(ltrx130a_init_info.platform_diver_addr->driver),
		ltrx130a_attr_list_common,
		3);

	if (err) {
		APS_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}

	err = dualals_create_node(obj->sensor_num);

	if (err)
		goto exit_dualals_node_fail;

	als_ctl.open_report_data = als_open_report_data;
	als_ctl.enable_nodata = als_enable_nodata;
	als_ctl.set_delay  = als_set_delay;
	als_ctl.batch = als_batch;
	als_ctl.flush = als_flush;
	als_ctl.is_report_input_direct = false;
	als_ctl.is_support_batch = false;

	err = als_register_control_path(&als_ctl);
	if (err) {
		APS_ERR("register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	als_data.get_data = als_get_data;
	als_data.vender_div = 100;
	err = als_register_data_path(&als_data);
	if (err) {
		APS_ERR("register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	ltrx130a_init_flag = 0;
	APS_LOG("%s: OK\n", __func__);
	return 0;

exit_2nd_dualals_node_fail:
exit_sensor_obj_attach_fail:
	dualals_delete_node();
exit_dualals_node_fail:
	common_delete_node();
exit_create_attr_failed:
	misc_deregister(&ltrx130a_device);
exit_misc_device_register_failed:
exit_init_failed:
	kfree(obj);
exit:
	kfree(alsps_cust);
	APS_ERR("%s: err = %d\n", __func__, err);
	ltrx130a_init_flag = -1;
	return err;
}

static int ltrx130a_i2c_remove(struct i2c_client *client)
{
	int err;

	if (ltrx130a_dual.als_count == 1) {
		err = dualals_delete_node();

		if (err)
			APS_ERR("stk3x1x_delete_attr fail: %d\n", err);

		misc_deregister(&ltrx130a_device);
	}

	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));

	ltrx130a_dual.als_count--;

	return 0;
}

static int ltrx130a_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, LTRX130A_DEV_NAME);
	return 0;
}

static int ltrx130a_i2c_suspend(struct device *dev)
{
	int err;

	APS_FUN();

	if (atomic_read(&ltrx130a_dual.als_suspend))
		return 0;

	alsps_driver_pause_polling(1);

	atomic_set(&ltrx130a_dual.als_suspend, 1);

	for (int i = 0; i < ltrx130a_dual.als_count; i++) {
		struct i2c_client *client = NULL;

		if (!ltrx130a_dual.als[i]) {
			APS_ERR("null pointer!!\n");

			atomic_set(&ltrx130a_dual.als_suspend, 0);
			alsps_driver_pause_polling(0);
			return -EINVAL;
		}

		client = ltrx130a_dual.als[i]->client;

		err &= ltrx130a_als_enable(client, 0);
	}

	if (err < 0) {
		APS_ERR("disable als: %d\n", err);
		atomic_set(&ltrx130a_dual.als_suspend, 0);
		alsps_driver_pause_polling(0);
		return err;
	}

	return 0;
}

static int ltrx130a_i2c_resume(struct device *dev)
{
	int err;

	APS_FUN();

	if (!atomic_read(&ltrx130a_dual.als_suspend))
		return 0;

	for (int i = 0; i < ltrx130a_dual.als_count; i++) {
		struct ltrx130a_priv *obj = ltrx130a_dual.als[i];

		if (!obj) {
			APS_ERR("null pointer!!\n");
			return -EINVAL;
		}

		if (alsps_driver_query_polling_state(ID_LIGHT) == 1) {
			if (test_bit(CMC_BIT_ALS, &obj->enable)) {
				err = ltrx130a_als_enable(obj->client, 1);
				if (err < 0)
					APS_ERR("enable als fail: %d\n", err);
				else
					atomic_set(&ltrx130a_dual.als_suspend, 0);
			}
		}
	}

	atomic_set(&ltrx130a_dual.als_suspend, 0);
	alsps_driver_pause_polling(0);

	return 0;
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static int ltrx130a_local_uninit(void)
{
	APS_FUN();

	i2c_del_driver(&ltrx130a_i2c_driver);
	ltrx130a_init_flag = -1;

	return 0;
}

/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static void set_chip_common_setting(void)
{
	ltrx130a_dual.dual_als_enable = 0;
	ltrx130a_dual.als[0] = NULL;
	ltrx130a_dual.als[1] = NULL;
	ltrx130a_dual.als_count = 0;

	atomic_set(&ltrx130a_dual.als_debounce, 300);
	atomic_set(&ltrx130a_dual.als_deb_on, 0);
	atomic_set(&ltrx130a_dual.als_deb_end, 0);
	atomic_set(&ltrx130a_dual.als_suspend, 0);

}
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
static int  ltrx130a_local_init(void)
{
	APS_FUN();

	set_chip_common_setting();

	if (i2c_add_driver(&ltrx130a_i2c_driver))	{
		APS_ERR("add driver error\n");
		return -1;
	}

	if (-1 == ltrx130a_init_flag)
		return -1;

	if (ltrx130a_get_dual_cal())
		APS_ERR("get idme for alscal failed!!\n ");

	ltrx130a_dual.tp_cg_color = idme_get_tp_cg_color();

	return 0;
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static int __init ltrx130a_init(void)
{
	alsps_driver_add(&ltrx130a_init_info);
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit ltrx130a_exit(void)
{
	APS_FUN();
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
module_init(ltrx130a_init);
module_exit(ltrx130a_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("Liteon");
MODULE_DESCRIPTION("LTRX130A Driver");
MODULE_LICENSE("GPL");

