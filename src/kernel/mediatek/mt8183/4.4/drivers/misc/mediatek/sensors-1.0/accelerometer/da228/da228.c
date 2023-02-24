/* For MTK android platform.
 *
 * da228.c - Linux kernel modules for 3-Axis Accelerometer
 *
 * Copyright (C) 2020 Compal Electronics, inc.
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

#include <linux/kernel.h>
#include <linux/unistd.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/firmware.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/atomic.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/uaccess.h>
#include <cust_acc.h>
#include <accel.h>
#include <sensors_io.h>
#include "da228.h"

#define DA228_DEV_NAME          "da228"

#define DA228_CAL_FILE          "/data/gsensor_cal_data.bin"
#define CALI_READ_DATA_DELAY_MS	20
#define CALI_READ_DATA_TIMES    20

#define DA228_AXIS_X            0
#define DA228_AXIS_Y            1
#define DA228_AXIS_Z            2
#define DA228_AXES_NUM          3

#define DA228_NORMAL_MODE      0x00
#define DA228_SUSPEND_MODE     0x80

#define GSE_TAG                  "[DA228] "
#define GSE_FUN(f)               pr_info(GSE_TAG"%s\n", __func__)
#define GSE_ERR(fmt, args...)    pr_err(GSE_TAG"%s %d : "fmt,\
								__func__, __LINE__, ##args)
#define GSE_DBG(fmt, args...)    pr_debug(GSE_TAG fmt, ##args)
#define GSE_LOG(fmt, args...)    pr_info(GSE_TAG fmt, ##args)

enum mir_trace {
	MIR_TRC_ACC_NONE = 0,
	MIR_TRC_ACC_DEBUG,
	MIR_TRC_ACC_DATA,
};

struct scale_factor {
	u16 whole;
	u16 fraction;
};
/*----------------------------------------------------------------------------*/
struct data_resolution {
	struct scale_factor scalefactor;
	int sensitivity;
};

struct da228_i2c_data {
	struct i2c_client *client;
	struct acc_hw *hw;
	struct hwmsen_convert cvt;

	struct data_resolution *reso;
	struct mutex lock;

	int pid;

	atomic_t trace;
	atomic_t suspend;
	atomic_t selftest;
	atomic_t power_mode;
	int sampling_mode;
	s16 cali_sw[DA228_AXES_NUM];

	s16 data[DA228_AXES_NUM];

};

static struct data_resolution da228_data_resolution[] = {
	{{ 1, 0}, 512},
};
static int da228_init_flag = -1;

static struct acc_hw accel_hw;
static struct acc_hw *hw = &accel_hw;
static struct da228_i2c_data *da228_obj;

static int da228_i2c_probe(struct i2c_client *client,
					const struct i2c_device_id *id);
static int da228_i2c_remove(struct i2c_client *client);
static int da228_local_init(void);
static int da228_remove(void);
static int da228_i2c_suspend(struct device *dev);
static int da228_i2c_resume(struct device *dev);

#ifdef CONFIG_OF
static const struct of_device_id gsensor_of_match[] = {
	{ .compatible = "mediatek,da228", },
	{},
};
#endif

/*----------------------------------------------------------------------------*/
static const struct dev_pm_ops da228_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(da228_i2c_suspend, da228_i2c_resume)
};
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id da228_i2c_id[] = {
	{DA228_DEV_NAME, 0},
	{}
};
static struct i2c_driver da228_i2c_driver = {
	.driver = {
		.name = DA228_DEV_NAME,
#ifdef CONFIG_OF
		.of_match_table = gsensor_of_match,
#endif
#ifdef CONFIG_PM_SLEEP
		.pm = &da228_pm_ops,
#endif
	},
	.probe	= da228_i2c_probe,
	.remove	= da228_i2c_remove,
	.id_table = da228_i2c_id,
};

static struct acc_init_info da228_init_info = {
	.name = "da228",
	.init = da228_local_init,
	.uninit = da228_remove,
};

/*----------------------------------------------------------------------------*/
int da228_register_read(struct i2c_client *client, u8 addr, u8 *data)
{
	int res = 0;

	mutex_lock(&da228_obj->lock);
	*data = i2c_smbus_read_byte_data(client, addr);
	mutex_unlock(&da228_obj->lock);

	return res;
}
/*----------------------------------------------------------------------------*/
int da228_register_read_block(struct i2c_client *client,
						u8 addr, u8 count, u8 *data)
{
	int res = 0;

	mutex_lock(&da228_obj->lock);
	res = i2c_smbus_read_i2c_block_data(client, addr, count, data);
	mutex_unlock(&da228_obj->lock);

	return res;
}
/*----------------------------------------------------------------------------*/
int da228_register_write(struct i2c_client *client, u8 addr, u8 data)
{
	int res = 0;

	mutex_lock(&da228_obj->lock);
	res = i2c_smbus_write_byte_data(client, addr, data);
	mutex_unlock(&da228_obj->lock);

	return res;
}

int da228_register_mask_write(struct i2c_client *client,
				u8 addr,
				u8 mask,
				u8 data)
{
	int res = 0;
	u8 tmp_data;

	mutex_lock(&da228_obj->lock);
	tmp_data = i2c_smbus_read_byte_data(client, addr);

	tmp_data &= ~mask;
	tmp_data |= data & mask;
	res = i2c_smbus_write_byte_data(client, addr, tmp_data);
	mutex_unlock(&da228_obj->lock);

	return res;
}
/*----------------------------------------------------------------------------*/
static int da228_get_pid(struct da228_i2c_data *obj)
{
	u8 val = 0;
	int res = 0;

	res = da228_register_read(obj->client, NSA_REG_WHO_AM_I, &val);

	if (res) {
		GSE_ERR("get pid error\n");
		return -1;
	}

	obj->pid = (int)val;

	if (obj->pid != 0x13) {
		GSE_ERR("DA228 pid is not match with 0x13 %x\n", val);
		return -1;
	}

	GSE_LOG("PID check pass\n");

	return 0;
}
/*----------------------------------------------------------------------------*/
static int da228_read_rawdata(struct i2c_client *client, s16 *acc)
{
	int err = 0;
	u8 tmp_data[6] = {0};

	if (atomic_read(&da228_obj->suspend)) {
		acc[DA228_AXIS_X] = 0;
		acc[DA228_AXIS_Y] = 0;
		acc[DA228_AXIS_Z] = 0;
		return 0;
	}

	err = da228_register_read_block(client, NSA_REG_ACC_X_LSB, 6, tmp_data);
	if (err < 0) {
		GSE_ERR("read raw data fail! err: %d\n", err);
		return err;
	}

	acc[DA228_AXIS_X] = tmp_data[1] << 8 | tmp_data[0];
	acc[DA228_AXIS_Y] = tmp_data[3] << 8 | tmp_data[2];
	acc[DA228_AXIS_Z] = tmp_data[5] << 8 | tmp_data[4];

	acc[DA228_AXIS_X] = acc[DA228_AXIS_X] >> 4;
	acc[DA228_AXIS_Y] = acc[DA228_AXIS_Y] >> 4;
	acc[DA228_AXIS_Z] = acc[DA228_AXIS_Z] >> 4;

	if (atomic_read(&da228_obj->trace) & MIR_TRC_ACC_DATA) {
		GSE_LOG("raw data x=%d, y=%d, z=%d\n",
			acc[DA228_AXIS_X],
			acc[DA228_AXIS_Y],
			acc[DA228_AXIS_Z]);
	}

	return 0;
}
/*----------------------------------------------------------------------------*/
static void da228_remap_axes(s16 *sdata, int *tdata)
{
	struct da228_i2c_data *obj = da228_obj;

	tdata[obj->cvt.map[DA228_AXIS_X]] =
		obj->cvt.sign[DA228_AXIS_X] * sdata[DA228_AXIS_X];
	tdata[obj->cvt.map[DA228_AXIS_Y]] =
		obj->cvt.sign[DA228_AXIS_Y] * sdata[DA228_AXIS_Y];
	tdata[obj->cvt.map[DA228_AXIS_Z]] =
		obj->cvt.sign[DA228_AXIS_Z] * sdata[DA228_AXIS_Z];
}
/*----------------------------------------------------------------------------*/
static int da228_read_acc_data(struct da228_i2c_data *obj,
						int *px, int *py, int *pz)
{
	int err = 0;
	int acc[DA228_AXES_NUM] = {0};
	s16 acc_buff[DA228_AXES_NUM] = {0};

	err = da228_read_rawdata(obj->client, acc_buff);
	if (err) {
		GSE_ERR("da228 read sensor data fail\n");
		return err;
	}

	/*remap coordinate*/
	da228_remap_axes(acc_buff, acc);

	acc[DA228_AXIS_X] += obj->cali_sw[DA228_AXIS_X];
	acc[DA228_AXIS_Y] += obj->cali_sw[DA228_AXIS_Y];
	acc[DA228_AXIS_Z] += obj->cali_sw[DA228_AXIS_Z];

	GSE_DBG("cvt x=%d, y=%d, z=%d\n",
		obj->cvt.sign[DA228_AXIS_X],
		obj->cvt.sign[DA228_AXIS_Y],
		obj->cvt.sign[DA228_AXIS_Z]);

	acc[DA228_AXIS_X] = acc[DA228_AXIS_X] * GRAVITY_EARTH_1000
					/ obj->reso->sensitivity;
	acc[DA228_AXIS_Y] = acc[DA228_AXIS_Y] * GRAVITY_EARTH_1000
					/ obj->reso->sensitivity;
	acc[DA228_AXIS_Z] = acc[DA228_AXIS_Z] * GRAVITY_EARTH_1000
					/ obj->reso->sensitivity;

	if (atomic_read(&da228_obj->trace) & MIR_TRC_ACC_DATA) {
		GSE_LOG("da228_read_data: x=%d, y=%d, z=%d",
				acc[DA228_AXIS_X],
				acc[DA228_AXIS_Y],
				acc[DA228_AXIS_Z]);
	}

	*px = acc[DA228_AXIS_X];
	*py = acc[DA228_AXIS_Y];
	*pz = acc[DA228_AXIS_Z];

	return 0;
}

/*----------------------------------------------------------------------------*/
static int idme_get_gsensorcal_calibration(void)
{
	struct da228_i2c_data *obj = da228_obj;
	int err;
	char *gsensor_cal = NULL;
	char *sepstr;
	char *sepdata;

	long data_x;
	long data_y;
	long data_z;
	char buf[64] = {0};

	gsensor_cal = idme_get_sensorcal();
	if (gsensor_cal == NULL) {
		GSE_ERR("idme get sensorcal fail!\n");
		return -1;
	}
	GSE_LOG("gsensor_cal %s\n", gsensor_cal);

	strcpy(buf, gsensor_cal);

	sepstr = buf;

	sepdata = strsep(&sepstr, ",");
	if (sepdata == NULL) {
		GSE_ERR("strsep calibration data fail\n");
		return -1;
	}

	GSE_LOG("gsensor_cal x %s\n", sepdata);
	err = kstrtol(sepdata, 10, &data_x);
	if (err) {
		GSE_ERR("calibration data x char to long fail\n");
		return -1;
	}

	sepdata = strsep(&sepstr, ",");
	if (sepdata == NULL) {
		GSE_ERR("strsep calibration data fail\n");
		return -1;
	}
	GSE_LOG("gsensor_cal y %s\n", sepdata);
	err = kstrtol(sepdata, 10, &data_y);
	if (err) {
		GSE_ERR("calibration data y char to long fail\n");
		return -1;
	}

	GSE_LOG("gsensor_cal z %s\n", sepstr);
	err = kstrtol(sepstr, 10, &data_z);

	obj->cali_sw[DA228_AXIS_X] = (int)data_x;
	obj->cali_sw[DA228_AXIS_Y] = (int)data_y;
	obj->cali_sw[DA228_AXIS_Z] = (int)data_z;

	return 0;
}
/*----------------------------------------------------------------------------*/
static bool gsensor_store_cali_in_file(const char *filename,
					int cali_x, int cali_y, int cali_z)
{
	struct file *cali_file;
	mm_segment_t fs;
	char data_buf[64] = {0};

	cali_file = filp_open(filename, O_CREAT | O_RDWR, 0777);
	if (IS_ERR(cali_file)) {
		GSE_ERR("%s open error! exit!\n", __func__);
		return false;
	}
	fs = get_fs();
	set_fs(get_ds());

	sprintf(data_buf, "%d,%d,%d", cali_x, cali_y, cali_z);
	GSE_LOG("%s sprintf(w_buf!\n", __func__);

	vfs_write(cali_file, data_buf, sizeof(data_buf), &cali_file->f_pos);

	set_fs(fs);

	filp_close(cali_file, NULL);
	GSE_LOG("pass\n");
	return 0;
}
/*----------------------------------------------------------------------------*/
static int da228_read_rawdata_avg(int period, int count,
			int *p_x, int *p_y, int *p_z)
{
	struct da228_i2c_data *obj = da228_obj;
	int num = 0;
	int res = 0;
	int remapbuf[DA228_AXES_NUM];
	s16 databuf[DA228_AXES_NUM];
	int avg_x = 0, avg_y = 0, avg_z = 0;

	while (num < count) {
		res = da228_read_rawdata(obj->client, databuf);
		if (res) {
			GSE_ERR("I2C error: ret value=%d", res);
			return -1;
		}

		da228_remap_axes(databuf, remapbuf);

		GSE_LOG("read gsensor data 20 times x: %d y: %d z: %d\n",
			remapbuf[DA228_AXIS_X],
			remapbuf[DA228_AXIS_Y],
			remapbuf[DA228_AXIS_Z]);

		avg_x += remapbuf[DA228_AXIS_X];
		avg_y += remapbuf[DA228_AXIS_Y];
		avg_z += remapbuf[DA228_AXIS_Z];

		GSE_LOG("%s tatal++: %d, %d, %d\n", __func__,
			avg_x, avg_y, avg_z);

		num++;
		msleep(period);
	}

	*p_x = avg_x / count;
	*p_y = avg_y / count;
	*p_z = avg_z / count;

	GSE_LOG("%s average: %d, %d, %d\n", __func__, *p_x, *p_y, *p_z);

	return 0;
}
/*----------------------------------------------------------------------------*/
static int da228_sw_calibration(int period, int count,
			int *cali_x, int *cali_y, int *cali_z)
{
	struct da228_i2c_data *obj = da228_obj;
	int golden_x = 0;
	int golden_y = 0;
	int golden_z = 512;

	int avg_x = 0;
	int avg_y = 0;
	int avg_z = 0;

	int res = 0;

	s16 databuf[DA228_AXES_NUM];

	if (!cali_x || !cali_y || !cali_z)
		return -2;

	if (obj == NULL)
		return -1;

	res = da228_read_rawdata(obj->client, databuf);
	if (res) {
		GSE_ERR("I2C error: ret value=%d", res);
		return -1;
	}

	GSE_LOG("da228 read rawdata gsensor data: %d, %d, %d!\n",
			databuf[DA228_AXIS_X],
			databuf[DA228_AXIS_Y],
			databuf[DA228_AXIS_Z]);

	res = da228_read_rawdata_avg(period, count,
						&avg_x, &avg_y, &avg_z);

	if (res) {
		GSE_ERR("read rawdata avg error %d\n", res);
		return res;
	}

	*cali_x = golden_x - avg_x;
	*cali_y = golden_y - avg_y;
	if (avg_z > 0)
		*cali_z = golden_z - avg_z;
	else
		*cali_z = -(avg_z + golden_z);

	GSE_LOG("%s %d, %d, %d\n", __func__, *cali_x, *cali_y, *cali_z);

	return res;
}

/*----------------------------------------------------------------------------*/
static int da228_soft_reset(struct da228_i2c_data *obj)
{
	int res = 0;

	res = da228_register_write(obj->client,
			NSA_REG_SPI_I2C, DA228_SW_RESET);
	mdelay(5);

	return res;
}

static int da228_chip_init(struct i2c_client *client)
{
	if (da228_register_write(client, NSA_REG_G_RANGE, DA228_RANGE_4G)) {
		GSE_ERR("G range register set failed!!\n");
		return -ENAVAIL;
	}

	if (da228_register_write(client,
				NSA_REG_POWERMODE_BW, DA228_BW_1_2)) {
		GSE_ERR("power mode bw register set failed!!\n");
		return -ENAVAIL;
	}

	if (da228_register_write(client,
			NSA_REG_ODR_AXIS_DISABLE, DA228_ODR_125HZ)) {
		GSE_ERR("ODR register set failed!!\n");
		return -ENAVAIL;
	}

	if (da228_register_write(client, NSA_REG_INTERRUPT_SETTINGS2, 0x00)) {
		GSE_ERR("Interrupt setting 2 register set failed!!\n");
		return -ENAVAIL;
	}

	if (da228_register_write(client, NSA_REG_INTERRUPT_MAPPING2, 0x00)) {
		GSE_ERR("Interrupt mapping 2 register set failed!!\n");
		return -ENAVAIL;
	}

	if (da228_register_write(client, NSA_REG_SWAP_POLARITY,
						DA228_X_Y_SWAP |
						DA228_Z_POLARITY_REVERSE |
						DA228_Y_POLARITY_REVERSE)) {
		GSE_ERR("swap polarity register set failed!!\n");
		return -ENAVAIL;
	}

	if (da228_register_mask_write(client,
							NSA_REG_INT_PIN_CONFIG,
							0x0F,
							0x05)) {
		GSE_ERR("Interrupt mapping 2 register set failed!!\n");
		return -ENAVAIL;
	}

	return 0;
}
/*----------------------------------------------------------------------------*/
static int da228_client_init(struct da228_i2c_data *obj)
{

	GSE_FUN();

	if (da228_soft_reset(obj)) {
		GSE_ERR("soft reset failed!!\n");
		return -ENAVAIL;
	}

	if (da228_chip_init(obj->client)) {
		GSE_ERR("chip init failed!\n");
		return -ENAVAIL;
	}

	GSE_LOG("i2c client init done!!\n");

	return 0;
}
/*----------------------------------------------------------------------------*/
static int da228_set_enable(struct da228_i2c_data *obj, char enable)
{
	int res = 0;


	if (!enable) {
		res = da228_register_mask_write(obj->client,
				NSA_REG_POWERMODE_BW, 0x0e, DA228_SUSPEND_MODE);
		atomic_set(&obj->power_mode, 0);
	} else {
		res = da228_register_mask_write(obj->client,
				NSA_REG_POWERMODE_BW, 0x0e, DA228_NORMAL_MODE);
		atomic_set(&obj->power_mode, 1);
	}

	if (res)
		GSE_ERR("da228_set_enable fail! expect eanble:%d error:%d\n", enable, res);


	return res;
}
/*----------------------------------------------------------------------------*/
static int da228_i2c_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct da228_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;

	GSE_FUN();

	acc_driver_pause_polling(1);
	atomic_set(&obj->suspend, 1);

	err = da228_set_enable(obj, false);

	if (err) {
		GSE_ERR("disable fail!\n");
		atomic_set(&obj->suspend, 0);
		acc_driver_pause_polling(0);
	}

	return err;
}
/*----------------------------------------------------------------------------*/
static int da228_i2c_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct da228_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;

	GSE_FUN();

	if (acc_driver_query_polling_state() == 1) {

		err = da228_set_enable(obj, true);
		if (err) {
			GSE_ERR("enable fail!!\n");
		}
	}

	atomic_set(&obj->suspend, 0);
	acc_driver_pause_polling(0);

	return err;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	struct da228_i2c_data *obj = da228_obj;

	if (!obj) {
		GSE_ERR("da228 obj is null!!\n");
		return scnprintf(buf, PAGE_SIZE, "Driver error\n");
	}

	return scnprintf(buf, PAGE_SIZE, "DA228 chip id %x\n", obj->pid);
}
/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	struct da228_i2c_data *obj = da228_obj;
	int err = 0;
	int data_x = 0;
	int data_y = 0;
	int data_z = 0;

	if (!obj) {
		GSE_ERR("da228 obj is null!!\n");
		return scnprintf(buf, PAGE_SIZE, "Driver error\n");
	}

	err = da228_read_acc_data(obj, &data_x, &data_y, &data_z);
	if (err) {
		GSE_ERR("show_sensordata_value() read sensor data error!\n");
		return 0;
	}

	return scnprintf(buf, PAGE_SIZE, "%d %d %d\n",
			data_x, data_y, data_z);
}
/*----------------------------------------------------------------------------*/
static ssize_t show_cali_value(struct device_driver *ddri, char *buf)
{
	struct da228_i2c_data *obj = da228_obj;

	if (!obj) {
		GSE_ERR("da228 obj is null!!\n");
		return scnprintf(buf, PAGE_SIZE, "Driver error\n");
	}

	return scnprintf(buf, PAGE_SIZE,
		"[SW ][%d] (%+3d, %+3d, %+3d)\n", 1,
		obj->cali_sw[DA228_AXIS_X],
		obj->cali_sw[DA228_AXIS_Y],
		obj->cali_sw[DA228_AXIS_Z]);
}
/*----------------------------------------------------------------------------*/
static ssize_t store_cali_value(struct device_driver *ddri,
				const char *buf, size_t count)
{
	struct da228_i2c_data *obj = da228_obj;
	int cali[DA228_AXES_NUM] = {0};
	int err = 0, x, y, z;

	if (!strncmp(buf, "rst", 3)) {
		memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));

	} else if (!strncmp(buf, "1", 1)) {

		err = da228_sw_calibration(CALI_READ_DATA_DELAY_MS,
						CALI_READ_DATA_TIMES,
						&cali[DA228_AXIS_X],
						&cali[DA228_AXIS_Y],
						&cali[DA228_AXIS_Z]);

		if (err) {
			GSE_ERR("calibration fail!\n");
			return count;
		}

		/*set cali to obj*/
		obj->cali_sw[DA228_AXIS_X] = cali[DA228_AXIS_X];
		obj->cali_sw[DA228_AXIS_Y] = cali[DA228_AXIS_Y];
		obj->cali_sw[DA228_AXIS_Z] = cali[DA228_AXIS_Z];

		/*write cali to file*/
		err = gsensor_store_cali_in_file(DA228_CAL_FILE,
						cali[DA228_AXIS_X],
						cali[DA228_AXIS_Y],
						cali[DA228_AXIS_Z]);

	} else if (sscanf(buf, "0x%02X 0x%02X 0x%02X", &x, &y, &z) == 3) {
		obj->cali_sw[DA228_AXIS_X] = x;
		obj->cali_sw[DA228_AXIS_Y] = y;
		obj->cali_sw[DA228_AXIS_Z] = z;

	}

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	struct da228_i2c_data *obj = da228_obj;

	if (!obj) {
		GSE_ERR("da228 obj is null!!\n");
		return scnprintf(buf, PAGE_SIZE, "Driver error\n");
	}

	return scnprintf(buf, PAGE_SIZE, "trace=%d", obj->trace);

}
/*----------------------------------------------------------------------------*/
static ssize_t store_trace_value(struct device_driver *ddri,
				const char *buf, size_t count)
{
	int trace;

	if (sysfs_streq(buf, "1")) {
		trace = MIR_TRC_ACC_DEBUG;
	} else if (sysfs_streq(buf, "2")) {
		trace = MIR_TRC_ACC_DATA;
	} else if (sysfs_streq(buf, "0")) {
		trace = MIR_TRC_ACC_NONE;
	} else {
		GSE_ERR("%s, invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}

	atomic_set(&da228_obj->trace, trace);
	GSE_LOG("set trace =%d, count = %d\n", trace, (int)count);

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	struct da228_i2c_data *obj = da228_obj;
	ssize_t len = 0;

	if (obj->hw) {
		len += scnprintf(buf + len, PAGE_SIZE - len,
				"CUST: %d %d (%d %d)\n",
				obj->hw->i2c_num,
				obj->hw->direction,
				obj->hw->power_id,
				obj->hw->power_vol);
	} else
		len += scnprintf(buf + len, PAGE_SIZE - len, "CUST: NULL\n");

	len += scnprintf(buf + len, PAGE_SIZE - len, "i2c addr:%#x\n",
			obj->client->addr);
	return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_registers(struct device_driver *ddri, char *buf)
{
	struct da228_i2c_data *obj = da228_obj;
	struct i2c_client *client = obj->client;
	unsigned char temp = 0;
	int count = 0;
	int len = 0;
	int ret = 0;

	if (!client) {
		GSE_ERR("i2c client is null!!\n");
		return scnprintf(buf, PAGE_SIZE, "Driver error\n");
	}

	while (count <= 0x12) {
		temp = 0;
		ret = da228_register_read(client, count, &temp);
		if (ret < 0) {
			len = scnprintf(buf, PAGE_SIZE, "Read byte error\n");
			return len;
		}
		len += scnprintf(buf + len, PAGE_SIZE, "0x%x\n", temp);
		count++;
	}

	return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_softreset(struct device_driver *ddri, char *buf)
{
	struct da228_i2c_data *obj = da228_obj;
	int err = 0;

	if (!obj) {
		GSE_ERR("da228 obj is null!!\n");
		return scnprintf(buf, PAGE_SIZE, "Driver error\n");
	}

	err = da228_soft_reset(obj);
	if (err) {
		GSE_ERR("soft reset fail!!\n");
		return scnprintf(buf, PAGE_SIZE, "softreset fail!\n");
	}

	err = da228_chip_init(obj->client);
	if (err) {
		GSE_ERR("chip init failed!\n");
		return scnprintf(buf, PAGE_SIZE, "chip init  fail!\n");
	}

	return scnprintf(buf, PAGE_SIZE, "softreset pass!\n");
}
/*----------------------------------------------------------------------------*/
static ssize_t show_selftest(struct device_driver *ddri, char *buf)
{
	int avg_x = 0, avg_y = 0, avg_z = 0;
	int res = 0;
	int x_failed = 0, y_failed = 0, z_failed = 0;
	int selftest_res = 1;
	int num = 0;

	res = da228_read_rawdata_avg(20, 20, &avg_x, &avg_y, &avg_z);

	if (avg_x >= 110) {
		GSE_ERR("X axis fail!! avg_x=%d\n", avg_x);
		selftest_res = 0;
		x_failed = 1;
	}

	if (avg_y >= 110) {
		GSE_ERR("Y axis fail!! avg_y=%d\n", avg_y);
		selftest_res = 0;
		y_failed = 1;
	}

	if (avg_z == 0 || avg_z > -400) {
		GSE_ERR("Z axis fail!! avg_z=%d\n", avg_z);
		selftest_res = 0;
		z_failed = 1;
	}

	if (selftest_res == 0) {
		num = scnprintf(buf, PAGE_SIZE,
			"selftest failed, X_failed=%d, Y_failed=%d, Z_failed=%d, avg_x=%d, avg_y=%d, avg_z=%d\n",
			x_failed, y_failed, z_failed,
			avg_x, avg_y, avg_z);
	} else {
		num = scnprintf(buf, PAGE_SIZE,
			"selftest success, avg_x=%d, avg_y=%d, avg_z=%d\n",
			avg_x, avg_y, avg_z);
	}

	return num;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_gsensor_test(struct device_driver *ddri, char *buf)
{
	struct da228_i2c_data *obj = da228_obj;
	int err = 0;
	int data_x = 0, data_y = 0, data_z = 0;

	if (atomic_read(&obj->power_mode) != 1) {
		err = da228_set_enable(obj, 1);
		if (err) {
			GSE_ERR("DA228 enable fail!\n");
			return scnprintf(buf, PAGE_SIZE, "DA228 enable fail!\n");
		}
	}

	msleep(150);

	err = da228_read_acc_data(obj, &data_x, &data_y, &data_z);
	if (err) {
		GSE_ERR("DA228 read sensor data fail!\n");
		return scnprintf(buf, PAGE_SIZE,
				"DA228 read sensor data fail!\n");
	}

	return scnprintf(buf, PAGE_SIZE, "%d %d %d\n",
		(int32_t)data_x, (int32_t)data_y, (int32_t)data_z);
}
/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(chipinfo, S_IWUSR | S_IRUGO, show_chipinfo_value, NULL);

static DRIVER_ATTR(sensordata, S_IWUSR | S_IRUGO, show_sensordata_value, NULL);
static DRIVER_ATTR(cali, S_IWUSR | S_IRUGO, show_cali_value, store_cali_value);

static DRIVER_ATTR(trace, S_IWUSR | S_IRUGO, show_trace_value,
				store_trace_value);
static DRIVER_ATTR(status, S_IRUGO, show_status_value, NULL);

static DRIVER_ATTR(dump_registers, S_IRUGO, show_registers, NULL);
static DRIVER_ATTR(softreset, S_IRUGO, show_softreset, NULL);
static DRIVER_ATTR(selftest, S_IRUGO, show_selftest, NULL);

static DRIVER_ATTR(gsensortest, S_IWUSR | S_IRUGO, show_gsensor_test, NULL);
/*----------------------------------------------------------------------------*/
static struct driver_attribute *da228_attr_list[] = {
	/*chip information*/
	&driver_attr_chipinfo,
	/*dump sensor data*/
	&driver_attr_sensordata,
	/*show calibration data*/
	&driver_attr_cali,
	/*trace log*/
	&driver_attr_trace,
	&driver_attr_status,

	&driver_attr_dump_registers,
	&driver_attr_softreset,
	&driver_attr_selftest,

	&driver_attr_gsensortest,
};
/*----------------------------------------------------------------------------*/
static int da228_create_attr(struct device_driver *driver)
{
	int num = (int)(ARRAY_SIZE(da228_attr_list));
	int idx, err = 0;

	if (driver == NULL)
		return -EINVAL;


	for (idx = 0; idx < num; idx++) {
		err = driver_create_file(driver, da228_attr_list[idx]);
		if (err) {
			GSE_ERR("driver_create_file (%s) = %d\n",
				da228_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}
/*----------------------------------------------------------------------------*/
static int da228_delete_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(ARRAY_SIZE(da228_attr_list));

	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++)
		driver_remove_file(driver, da228_attr_list[idx]);

	return err;
}
/*----------------------------------------------------------------------------*/
/******************************************************************************
* Function Configuration
******************************************************************************/
/*----------------------------------------------------------------------------*/
/* if use  this typ of enable ,*/
/*Gsensor should report inputEvent(x, y, z ,stats, div) to HAL*/
static int da228_open_report_data(int open)
{
	/*should queuq work to report event if  is_report_input_direct=true*/
	return 0;
}
/*----------------------------------------------------------------------------*/
/* if use  this typ of enable ,*/
/*Gsensor only enabled but not report inputEvent to HAL*/
static int da228_enable_nodata(int en)
{
	struct da228_i2c_data *obj = da228_obj;
	int err = 0;

	if (en)
		err = da228_set_enable(obj, 1);
	else
		err = da228_set_enable(obj, 0);

	if (err)
		GSE_ERR("da228_enable_nodata fail! err=%d\n", err);

	return err;
}
/*----------------------------------------------------------------------------*/
static int da228_set_delay(u64 ns)
{
	struct da228_i2c_data *obj = da228_obj;
	u8 data = DA228_ODR_125HZ;

	ns = ns / 1000 / 1000;

	/* keep ODR at 125Hz */

	if (atomic_read(&obj->trace) == MIR_TRC_ACC_DEBUG) {
		GSE_LOG("set ODR to 0x%x for delay %d\n", data, ns);
	}

	return 0;
}
/*----------------------------------------------------------------------------*/
static int gsensor_acc_batch(int flag, int64_t samplingPeriodNs,
					int64_t maxBatchReportLatencyNs)
{
	int res = 0;
	u64 ns = (int)samplingPeriodNs;

	res = da228_set_delay(ns);
	if (res) {
		GSE_ERR("set delay failed!! %d\n", res);
		return -ENAVAIL;
	}

	GSE_DBG("gsensor acc set delay = (%d) ok.\n", (ns / 1000 / 1000));

	return 0;
}
/*----------------------------------------------------------------------------*/
static int gsensor_acc_flush(void)
{
	return acc_flush_report();
}

/*----------------------------------------------------------------------------*/
static int gsensor_get_data(int *x, int *y, int *z, int *status)
{
	int err = 0;

	/* use acc raw data for gsensor */
	err = da228_read_acc_data(da228_obj, x, y, z);
	if (err)
		GSE_ERR("gsensor_get_data() read sensor data error! %d\n",
				err);
	else
		*status = SENSOR_STATUS_ACCURACY_HIGH;

	return err;
}
/*----------------------------------------------------------------------------*/
static int da228_i2c_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	int err = 0;
	static struct acc_init_info *init = &da228_init_info;
	struct da228_i2c_data *obj;
	struct acc_control_path mira_acc_control_path = {0};
	struct acc_data_path mira_acc_data_path = {0};

	GSE_FUN();
	err = get_accel_dts_func(client->dev.of_node, hw);
	if (err) {
		GSE_ERR("get dts info fail\n");
		err = -EFAULT;
		goto exit;
	}

	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (!obj) {
		err = -ENOMEM;
		goto exit;
	}

	obj->hw = hw;

	err = hwmsen_get_convert(obj->hw->direction, &obj->cvt);
	if (err) {
		GSE_ERR("invalid direction: %d\n", obj->hw->direction);
		goto exit_kfree;
	}

	da228_obj = obj;
	client->addr = *hw->i2c_addr;
	obj->client = client;
	i2c_set_clientdata(client, obj);
	mutex_init(&obj->lock);
	memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);
	obj->reso = &da228_data_resolution[0];

	err = da228_get_pid(obj);
	if (err) {
		GSE_ERR("check pid fail\n!");
		goto exit_kfree;
	}

	/* chip init */
	err = da228_client_init(obj);
	if (err) {
		GSE_ERR("init chip fail!\n");
		goto exit_init_err;
	}

	/* misc register */
	err = da228_create_attr((&init->platform_diver_addr->driver));
	if (err) {
		GSE_ERR("create attr fail!! %d\n", err);
		goto exit_create_attr_err;
	}

	/* control path init */
	mira_acc_control_path.open_report_data = da228_open_report_data;
	mira_acc_control_path.enable_nodata = da228_enable_nodata;
	mira_acc_control_path.set_delay = da228_set_delay;
	mira_acc_control_path.is_report_input_direct = false;
	mira_acc_control_path.is_support_batch = false;
	mira_acc_control_path.batch = gsensor_acc_batch;
	mira_acc_control_path.flush = gsensor_acc_flush;

	err = acc_register_control_path(&mira_acc_control_path);
	if (err) {
		GSE_ERR("register acc control path err\n");
		goto exit_register_control_path;
	}

	mira_acc_data_path.get_data = gsensor_get_data;
	mira_acc_data_path.vender_div = 1000;
	err = acc_register_data_path(&mira_acc_data_path);

	if (err) {
		GSE_ERR("acc_register_data_path fail");
		goto exit_register_control_path;
	}

	GSE_LOG("probe done!!\n");

	da228_init_flag = 0;
	return 0;

exit_register_control_path:
	da228_delete_attr(&(init->platform_diver_addr->driver));
exit_create_attr_err:
exit_kfree:
exit_init_err:
	kfree(obj);
exit:
	da228_init_flag = -1;
	return err;
}
/*----------------------------------------------------------------------------*/
static int da228_local_init(void)
{
	GSE_FUN();

	if (i2c_add_driver(&da228_i2c_driver)) {
		GSE_ERR("Add i2c driver fail\n");
		return -1;
	}

	if (-1 == da228_init_flag) {
		GSE_ERR("da228 init error!!\n");
		return -1;
	}

	if (idme_get_gsensorcal_calibration() != 0) {
		GSE_ERR("idme calibration values fail x:%d, y:%d, z:%d\n",
				da228_obj->cali_sw[DA228_AXIS_X],
				da228_obj->cali_sw[DA228_AXIS_Y],
				da228_obj->cali_sw[DA228_AXIS_Z]);
	}

	return 0;
}
/*----------------------------------------------------------------------------*/
static int da228_i2c_remove(struct i2c_client *client)
{
	static struct acc_init_info *init = &da228_init_info;
	int err = 0;

	GSE_FUN();

	err = da228_delete_attr(&(init->platform_diver_addr->driver));

	if (err)
		GSE_ERR("da228_delete_attr fail: %d\n", err);

	mutex_destroy(&da228_obj->lock);
	kfree(da228_obj);

	return 0;
}

static int da228_remove(void)
{

	GSE_FUN();
	i2c_del_driver(&da228_i2c_driver);
	return 0;
}
/*----------------------------------------------------------------------------*/
static int __init da228_init(void)
{

	GSE_FUN();

	acc_driver_add(&da228_init_info);
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit da228_exit(void)
{

	GSE_FUN();
}
/*----------------------------------------------------------------------------*/
module_init(da228_init);
module_exit(da228_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("EdwardCW Lin");
MODULE_DESCRIPTION("miramems DA228 accelerometer driver");
MODULE_LICENSE("GPLv2");
