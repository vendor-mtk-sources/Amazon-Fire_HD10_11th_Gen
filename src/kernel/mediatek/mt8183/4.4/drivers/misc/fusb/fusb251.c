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

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <linux/version.h>
#include <linux/err.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <misc/fusb251_notify.h>
#include "fusb251.h"
#include <linux/power_supply.h>
#include <tcpm.h>
#ifdef CONFIG_AMAZON_LD_SWITCH
#include <misc/amzn_ld_switch.h>
#endif

#define I2C_RETRIES 2
#define I2C_RETRY_DELAY 5 /* ms */

struct fusb251 *g_fusb251;

enum FUSB251_STATE_TYPE {
	TYPE_DRY = 0,
	TYPE_WET = 1,
	TYPE_WET_VBUS = 2,
};

enum FUSB251_SBU_TYPE {
	TYPE_SBU = 0,
	TYPE_SBUFT = 1,
};

static int fusb251_read_data(struct fusb251 *fusb251, unsigned char reg,
			int len, unsigned char value[])
{
	struct i2c_client *client;
	int err;
	int tries = 0;
	int error = 0;
	struct i2c_msg msgs[] = {
		{
			.flags = 0,
			.len = 1,
			.buf = &reg,
		}, {
			.flags = I2C_M_RD,
			.len = len,
			.buf = value,
		},
	};

	if (!fusb251) {
		pr_err("%s: read data: No device is available\n", __func__);
		return -1;
	}

	client = fusb251->i2c;
	msgs[0].addr = client->addr;
	msgs[1].addr = client->addr;

	do {
		err = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
		if (err != ARRAY_SIZE(msgs))
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != ARRAY_SIZE(msgs)) && (++tries < I2C_RETRIES));

	if (err != ARRAY_SIZE(msgs)) {
		pr_err("%s: i2c_read error, %d\n", __func__, err);
		error = -1;
	}

	return error;
}

static int fusb251_write_data(struct fusb251 *fusb251,
			char *writebuf, int writelen)
{
	int ret = 0;
	struct i2c_client *client;

	if (fusb251->i2c == NULL) {
		pr_err("%s: write data: No device is available\n", __func__);
		return -1;
	}

	client = fusb251->i2c;
	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
				.addr = client->addr,
				.flags = 0,
				.len = writelen,
				.buf = writebuf,
			},
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			pr_err("%s: i2c_write error, ret=%d\n", __func__, ret);
	}

	return ret;
}

static int fusb251_i2c_read_reg(struct fusb251 *fusb251,
			unsigned char regaddr, unsigned char *regvalue)
{
	int ret = 0;

	if (!fusb251) {
		pr_err("%s: No device is available\n", __func__);
		return -1;
	}

	mutex_lock(&fusb251->fusb251_i2c_mutex);

	ret = fusb251_read_data(fusb251, regaddr, 1, regvalue);

	mutex_unlock(&fusb251->fusb251_i2c_mutex);

	return ret;
}

static int fusb251_i2c_write_reg(struct fusb251 *fusb251,
			unsigned char regaddr, unsigned char regvalue)
{
	unsigned char buf[2] = {0};
	int ret = 0;

	if (!fusb251) {
		pr_err("%s: write reg: No device is available\n", __func__);
		return -1;
	}

	mutex_lock(&fusb251->fusb251_i2c_mutex);

	buf[0] = regaddr;
	buf[1] = regvalue;

	ret = fusb251_write_data(fusb251, buf, sizeof(buf));

	mutex_unlock(&fusb251->fusb251_i2c_mutex);

	return ret;
}

static void fusb251_read_interface(struct fusb251 *fusb251,
				unsigned char reg, unsigned char mask,
				unsigned char shift, unsigned char *value)
{
	unsigned char reg_val = 0;

	mutex_lock(&fusb251->fusb251_i2c_mutex);

	fusb251_read_data(fusb251, reg, 1, &reg_val);

	reg_val &= (mask << shift);
	*value = (reg_val >> shift);

	mutex_unlock(&fusb251->fusb251_i2c_mutex);
}

static void fusb251_config_interface(struct fusb251 *fusb251,
				unsigned char reg, unsigned char mask,
				unsigned char shift, int value)
{
	unsigned char reg_val = 0;
	unsigned char buf[2] = {0};

	mutex_lock(&fusb251->fusb251_i2c_mutex);

	fusb251_read_data(fusb251, reg, 1, &reg_val);

	reg_val &= ~(mask << shift);
	reg_val |= (value << shift);

	buf[0] = reg;
	buf[1] = reg_val;

	fusb251_write_data(fusb251, buf, sizeof(buf));

	mutex_unlock(&fusb251->fusb251_i2c_mutex);
}

static int fusb251_resistance_conver_reg(int value)
{
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(mos_det_R_table); i++) {
		if (mos_det_R_table[i] == value)
			return i;
	}

	return -1;
}

static int fusb251_sbuft_voltage_convert_reg(int value)
{
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(sbuft_det_MV_table); i++) {
		if (sbuft_det_MV_table[i] == value)
			return i;
	}

	return -1;
}

static int fusb251_init(struct fusb251 *fusb251);
static ssize_t reset_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t size)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	cancel_delayed_work(&fusb251->mos_det_work);

	fusb251_i2c_write_reg(fusb251, FUSB251_RESET, RESET_DEVICE);

	mdelay(10);

	fusb251_init(fusb251);

	switch_set_state(&fusb251->st_switch, TYPE_DRY);

	queue_delayed_work(system_freezable_wq, &fusb251->mos_det_work, 0);

	pr_info("%s: reset device and ld_switch\n", __func__);

	return size;
}

static ssize_t dry_det_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t size)
{
	int temp = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	if (kstrtouint(buff, 0, &temp))
		return -EINVAL;

	if (temp) {
		fusb251_config_interface(fusb251, FUSB251_CONTROL,
				EN_DRY_DET_MASK, EN_DRY_DET_SHIFT, true);
		pr_info("%s: enable dry detection\n", __func__);
	} else {
		fusb251_config_interface(fusb251, FUSB251_CONTROL,
				EN_DRY_DET_MASK, EN_DRY_DET_SHIFT, false);
		pr_info("%s: disable dry detection\n", __func__);
	}

	return size;
}

static ssize_t dry_det_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	unsigned char temp = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	fusb251_read_interface(fusb251, FUSB251_CONTROL,
			EN_DRY_DET_MASK, EN_DRY_DET_SHIFT, &temp);

	return scnprintf(buf, PAGE_SIZE, "%d\n", temp ? 1 : 0);
}

static ssize_t sbu_det_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t size)
{
	int temp = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	if (kstrtouint(buff, 0, &temp))
		return -EINVAL;

	if (temp) {
		fusb251_config_interface(fusb251, FUSB251_CONTROL,
				EN_SBU_DET_MASK, EN_SBU_DET_SHIFT, true);
		pr_info("%s: enable sbu detection\n", __func__);
	} else {
		fusb251_config_interface(fusb251, FUSB251_CONTROL,
				EN_SBU_DET_MASK, EN_SBU_DET_SHIFT, false);
		pr_info("%s: disable sbu detection\n", __func__);
	}

	return size;
}

static ssize_t sbu_det_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	unsigned char temp = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	fusb251_read_interface(fusb251, FUSB251_CONTROL,
			EN_SBU_DET_MASK, EN_SBU_DET_SHIFT, &temp);

	return scnprintf(buf, PAGE_SIZE, "%d\n", temp ? 1 : 0);
}

static ssize_t auto_sbu_det_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t size)
{
	int temp = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	if (kstrtouint(buff, 0, &temp))
		return -EINVAL;

	if (temp) {
		fusb251_config_interface(fusb251, FUSB251_CONTROL,
			EN_AUTO_SBU_DET_MASK, EN_AUTO_SBU_DET_SHIFT, true);
		pr_info("%s: enable auto sbu detection\n", __func__);
	} else {
		fusb251_config_interface(fusb251, FUSB251_CONTROL,
			EN_AUTO_SBU_DET_MASK, EN_AUTO_SBU_DET_SHIFT, false);
		pr_info("%s: disable auto sbu detection\n", __func__);
	}

	return size;
}

static ssize_t auto_sbu_det_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	unsigned char temp = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	fusb251_read_interface(fusb251, FUSB251_CONTROL,
			EN_AUTO_SBU_DET_MASK, EN_AUTO_SBU_DET_SHIFT, &temp);

	return scnprintf(buf, PAGE_SIZE, "%d\n", temp ? 1 : 0);
}

static ssize_t sbuft_det_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t size)
{
	int temp = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	if (kstrtouint(buff, 0, &temp))
		return -EINVAL;

	if (temp) {
		fusb251_config_interface(fusb251, FUSB251_CONTROL,
				EN_SBUFT_DET_MASK, EN_SBUFT_DET_SHIFT, true);
		pr_info("%s: enable sbu detection\n", __func__);
	} else {
		fusb251_config_interface(fusb251, FUSB251_CONTROL,
				EN_SBUFT_DET_MASK, EN_SBUFT_DET_SHIFT, false);
		pr_info("%s: disable sbu detection\n", __func__);
	}

	return size;
}

static ssize_t sbuft_det_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	unsigned char temp = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	fusb251_read_interface(fusb251, FUSB251_CONTROL,
			EN_SBUFT_DET_MASK, EN_SBUFT_DET_SHIFT, &temp);

	return scnprintf(buf, PAGE_SIZE, "%d\n", temp ? 1 : 0);
}

static ssize_t status_look4sbu_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	unsigned char temp = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	fusb251_read_interface(fusb251, FUSB251_STATUS,
			STATUS_LOOK4SBU_MASK, STATUS_LOOK4SBU_SHIFT, &temp);

	return scnprintf(buf, PAGE_SIZE, "%d\n", temp ? 1 : 0);
}

static ssize_t sbu1_mos_status_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			fusb251->sbu1_mos_status ? 1 : 0);
}

static ssize_t sbu2_mos_status_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
	(struct fusb251 *)i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			fusb251->sbu2_mos_status ? 1 : 0);
}

static ssize_t sbu1_ft_status_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
	(struct fusb251 *)i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			fusb251->sbu1_ft_status ? 1 : 0);
}

static ssize_t sbu2_ft_status_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
	(struct fusb251 *)i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			fusb251->sbu2_ft_status ? 1 : 0);
}

static ssize_t threshold1_sbu_dry2wet_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t size)
{
	int temp = 0;
	unsigned char ret = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	if (kstrtouint(buff, 0, &temp))
		return -EINVAL;

	if (temp >= 0 && temp <= MAX_RESISTANCE) {
		ret = fusb251_resistance_conver_reg(temp);
		if (ret >= 0) {
			fusb251->threshold_sbu_dry2wet = temp;

			pr_info("%s: set threshold1_sbu_dry2wet to %dk\n",
				__func__, temp);

			return size;
		}
	}

	return -EINVAL;
}

static ssize_t threshold1_sbu_dry2wet_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			fusb251->threshold_sbu_dry2wet);
}

static ssize_t threshold1_sbu_wet2dry_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t size)
{
	int temp = 0;
	unsigned char ret = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	if (kstrtouint(buff, 0, &temp))
		return -EINVAL;

	if (temp >= 0 && temp <= MAX_RESISTANCE) {
		ret = fusb251_resistance_conver_reg(temp);
		if (ret >= 0) {
			fusb251->threshold_sbu_wet2dry = temp;

			pr_info("%s: set threshold1_sbu wet2dry to %dk\n",
				__func__, temp);

			return size;
		}
	}

	return -EINVAL;
}

static ssize_t threshold1_sbu_wet2dry_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			fusb251->threshold_sbu_wet2dry);
}

static ssize_t threshold2_sbuft_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t size)
{
	int temp = 0;
	unsigned char ret = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	if (kstrtouint(buff, 0, &temp))
		return -EINVAL;

	if (temp >= 0 && temp <= MAX_VOLTAGE) {
		ret = fusb251_sbuft_voltage_convert_reg(temp);
		if (ret >= 0) {
			fusb251->threshold_sbuft = temp;
			fusb251_config_interface(fusb251, FUSB251_THRESHOLD2,
				SBU_FLOAT_DET_MASK, SBU_FLOAT_DET_SHIFT, ret);

			pr_info("%s: set threshold2_sbuft to %dmV\n",
				__func__, temp);

			return size;
		}
	}

	return -EINVAL;
}

static ssize_t threshold2_sbuft_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			fusb251->threshold_sbuft);
}

static ssize_t task_dry_timer_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t size)
{
	int temp = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	if (kstrtouint(buff, 0, &temp))
		return -EINVAL;

	if (temp >= 0) {
		fusb251->task_timer[TYPE_DRY] = temp;

		pr_info("%s: set timer for dry task to %d\n", __func__, temp);

		return size;
	}

	return -EINVAL;
}

static ssize_t task_dry_timer_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			fusb251->task_timer[TYPE_DRY]);
}

static ssize_t task_wet_timer_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t size)
{
	int temp = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	if (kstrtouint(buff, 0, &temp))
		return -EINVAL;

	if (temp >= 0) {
		fusb251->task_timer[TYPE_WET] = temp;

		pr_info("%s: set timer for wet task to %d\n", __func__, temp);

		return size;
	}

	return -EINVAL;
}

static ssize_t task_wet_timer_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			fusb251->task_timer[TYPE_WET]);
}

static ssize_t task_wet_vbus_timer_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t size)
{
	int temp = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	if (kstrtouint(buff, 0, &temp))
		return -EINVAL;

	if (temp >= 0) {
		fusb251->task_timer[TYPE_WET_VBUS] = temp;

		pr_info("%s: set timer for wet w/ vbus task to %d\n",
				__func__, temp);

		return size;
	}

	return -EINVAL;
}

static ssize_t task_wet_vbus_timer_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			fusb251->task_timer[TYPE_WET_VBUS]);
}

static ssize_t cc_det_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t size)
{
	int temp = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	if (kstrtouint(buff, 0, &temp))
		return -EINVAL;

	if (temp) {
		fusb251_config_interface(fusb251, FUSB251_CONTROL,
				EN_CC_DET_MASK, EN_CC_DET_SHIFT, true);
		pr_info("%s: enable cc detection\n", __func__);
	} else {
		fusb251_config_interface(fusb251, FUSB251_CONTROL,
				EN_CC_DET_MASK, EN_CC_DET_SHIFT, false);
		pr_info("%s: disable cc detection\n", __func__);
	}

	return size;
}

static ssize_t cc_det_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	unsigned char temp = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	fusb251_read_interface(fusb251, FUSB251_CONTROL,
			EN_CC_DET_MASK, EN_CC_DET_SHIFT, &temp);

	return scnprintf(buf, PAGE_SIZE, "%d\n", temp ? 1 : 0);
}

static ssize_t status_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	unsigned char temp = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	fusb251_i2c_read_reg(fusb251, FUSB251_STATUS, &temp);

	return scnprintf(buf, PAGE_SIZE, "0x%02x\n", temp);
}

static ssize_t moi_status_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	unsigned char temp = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	fusb251_i2c_read_reg(fusb251, FUSB251_MOISTATUS, &temp);

	return scnprintf(buf, PAGE_SIZE, "0x%02x\n", temp);
}

static ssize_t switch_cc_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t size)
{
	int temp = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	if (kstrtouint(buff, 0, &temp))
		return -EINVAL;

	if (temp) {
		fusb251_config_interface(fusb251, FUSB251_SWITCHCONTROL,
				SWITCH_CC_MASK, SWITCH_CC_SHIFT, true);
		pr_info("%s: close CC switch\n", __func__);
	} else {
		fusb251_config_interface(fusb251, FUSB251_SWITCHCONTROL,
				SWITCH_CC_MASK, SWITCH_CC_SHIFT, false);
		pr_info("%s: open CC switch\n", __func__);
	}

	return size;
}

static ssize_t switch_cc_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	unsigned char temp = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	fusb251_read_interface(fusb251, FUSB251_SWITCHCONTROL,
			SWITCH_CC_MASK, SWITCH_CC_SHIFT, &temp);

	return scnprintf(buf, PAGE_SIZE, "%s\n", temp ? "close" : "open");
}

static ssize_t switch_sbu_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t size)
{
	int temp = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	if (kstrtouint(buff, 0, &temp))
		return -EINVAL;

	if (temp == 0x01) {
		fusb251_config_interface(fusb251, FUSB251_SWITCHCONTROL,
				SWITCH_SBU_MASK, SWITCH_SBU_SHIFT, true);
		pr_info("%s: close sbu switch\n", __func__);
		return size;
	} else if (temp == 0x00) {
		fusb251_config_interface(fusb251, FUSB251_SWITCHCONTROL,
				SWITCH_SBU_MASK, SWITCH_SBU_SHIFT, false);
		pr_info("%s: open sbu switch\n", __func__);
		return size;
	}

	pr_info("%s: input error\n", __func__);
	return -EINVAL;
}

static ssize_t switch_sbu_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	unsigned char temp = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	fusb251_read_interface(fusb251, FUSB251_SWITCHCONTROL,
			SWITCH_SBU_MASK, SWITCH_SBU_SHIFT, &temp);

	return scnprintf(buf, PAGE_SIZE, "%s\n", temp ? "close" : "open");
}

static ssize_t threshold1_sbu_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t size)
{
	int temp = 0;
	unsigned char ret = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	if (kstrtouint(buff, 0, &temp))
		return -EINVAL;

	if (temp >= 0 && temp <= MAX_RESISTANCE) {
		ret = fusb251_resistance_conver_reg(temp);
		if (ret >= 0) {
			fusb251_config_interface(fusb251, FUSB251_THRESHOLD1,
				SBU_MOS_R_DET_MASK, SBU_MOS_R_DET_SHIFT, ret);

			pr_info("%s: set threshold1_sbu to %dk\n",
					__func__, temp);

			return size;
		}
	}

	return -EINVAL;
}

static ssize_t threshold1_sbu_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	unsigned char temp = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	fusb251_read_interface(fusb251, FUSB251_THRESHOLD1,
			SBU_MOS_R_DET_MASK, SBU_MOS_R_DET_SHIFT, &temp);

	return scnprintf(buf, PAGE_SIZE, "%d\n", mos_det_R_table[temp]);
}

static ssize_t threshold1_cc_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t size)
{
	int temp = 0;
	unsigned char ret = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	if (kstrtouint(buff, 0, &temp))
		return -EINVAL;

	if (temp >= 0 && temp <= MAX_RESISTANCE) {
		ret = fusb251_resistance_conver_reg(temp);
		if (ret >= 0) {
			fusb251_config_interface(fusb251, FUSB251_THRESHOLD1,
				CC_MOS_R_DET_MASK, CC_MOS_R_DET_SHIFT, ret);

			pr_info("%s: set threshold1_cc to %dk\n",
					__func__, temp);

			return size;
		}
	}

	return -EINVAL;
}

static ssize_t threshold1_cc_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	unsigned char temp = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	fusb251_read_interface(fusb251, FUSB251_THRESHOLD1,
			CC_MOS_R_DET_MASK, CC_MOS_R_DET_SHIFT, &temp);

	return scnprintf(buf, PAGE_SIZE, "%d\n", mos_det_R_table[temp]);
}

static ssize_t threshold2_vdry_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t size)
{
	int temp = 0;
	unsigned char ret = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	if (kstrtouint(buff, 0, &temp))
		return -EINVAL;

	if (temp >= 0 && temp <= MAX_RESISTANCE) {

		ret = fusb251_resistance_conver_reg(temp);
		if (ret >= 0) {
			fusb251_config_interface(fusb251, FUSB251_THRESHOLD2,
					VDRY_R_DET_MASK, VDRY_R_DET_SHIFT, ret);

			pr_info("%s: set threshold2_vdry to %dk\n",
					__func__, temp);

			return size;
		}
	}

	return -EINVAL;
}

static ssize_t threshold2_vdry_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	unsigned char temp = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	fusb251_read_interface(fusb251, FUSB251_THRESHOLD2,
			VDRY_R_DET_MASK, VDRY_R_DET_SHIFT, &temp);

	return scnprintf(buf, PAGE_SIZE, "%d\n", mos_det_R_table[temp]);
}

static ssize_t regdump_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int i = 0;
	unsigned char temp = 0;
	char temp_info[200] = "";
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	for (i = 1; i <= FUSB251_REG_NUM; i++) {
		fusb251_i2c_read_reg(fusb251, i, &temp);
		scnprintf(temp_info, PAGE_SIZE, "reg[0x%02x]=0x%02x\n",
				i, temp);
		strcat(buf, temp_info);
	}

	return strlen(buf);
}

DEVICE_ATTR(reset, 0220, NULL, reset_store);
DEVICE_ATTR(control_dry_det, 0660, dry_det_show, dry_det_store);
DEVICE_ATTR(control_sbu_det, 0660, sbu_det_show, sbu_det_store);
DEVICE_ATTR(control_auto_sbu_det, 0660, auto_sbu_det_show,
			auto_sbu_det_store);
DEVICE_ATTR(control_sbuft_det, 0660, sbuft_det_show,
			sbuft_det_store);
DEVICE_ATTR(status_look4sbu, 0440, status_look4sbu_show, NULL);
DEVICE_ATTR(sbu1_mos_status, 0440, sbu1_mos_status_show, NULL);
DEVICE_ATTR(sbu2_mos_status, 0440, sbu2_mos_status_show, NULL);
DEVICE_ATTR(sbu1_ft_status, 0440, sbu1_ft_status_show, NULL);
DEVICE_ATTR(sbu2_ft_status, 0440, sbu2_ft_status_show, NULL);
DEVICE_ATTR(threshold1_sbu_dry2wet, 0660, threshold1_sbu_dry2wet_show,
			threshold1_sbu_dry2wet_store);
DEVICE_ATTR(threshold1_sbu_wet2dry, 0660, threshold1_sbu_wet2dry_show,
			threshold1_sbu_wet2dry_store);
DEVICE_ATTR(threshold2_sbuft, 0660, threshold2_sbuft_show,
			threshold2_sbuft_store);
DEVICE_ATTR(task_dry_timer, 0660, task_dry_timer_show, task_dry_timer_store);
DEVICE_ATTR(task_wet_timer, 0660, task_wet_timer_show, task_wet_timer_store);
DEVICE_ATTR(task_wet_vbus_timer, 0660, task_wet_vbus_timer_show,
			task_wet_vbus_timer_store);
DEVICE_ATTR(control_cc_det, 0660, cc_det_show, cc_det_store);
DEVICE_ATTR(status, 0440, status_show, NULL);
DEVICE_ATTR(moi_status, 0440, moi_status_show, NULL);
DEVICE_ATTR(switch_cc, 0660, switch_cc_show, switch_cc_store);
DEVICE_ATTR(switch_sbu, 0660, switch_sbu_show, switch_sbu_store);
DEVICE_ATTR(threshold1_sbu, 0660, threshold1_sbu_show,
			threshold1_sbu_store);
DEVICE_ATTR(threshold1_cc, 0660, threshold1_cc_show,
			threshold1_cc_store);
DEVICE_ATTR(threshold2_vdry, 0660, threshold2_vdry_show,
			threshold2_vdry_store);
DEVICE_ATTR(regdump, 0440, regdump_show, NULL);

static int fusb251_create_attr(struct device *cdev)
{
	int ret = 0;

	ret = device_create_file(cdev, &dev_attr_reset);
	if (ret < 0) {
		pr_err("%s: failed to create dev_attr_reset\n", __func__);
		goto err_attr_reset;
	}

	ret = device_create_file(cdev, &dev_attr_control_dry_det);
	if (ret < 0) {
		pr_err("%s: failed to create dev_attr_control_dry_det\n",
				__func__);
		goto err_attr_control_dry_det;
	}

	ret = device_create_file(cdev, &dev_attr_control_sbu_det);
	if (ret < 0) {
		pr_err("%s: failed to create dev_attr_control_sbu_det\n",
				__func__);
		goto err_attr_control_sbu_det;
	}

	ret = device_create_file(cdev, &dev_attr_control_auto_sbu_det);
	if (ret < 0) {
		pr_err("%s; failed to create dev_attr_control_auto_sbu_det\n",
				__func__);
		goto err_attr_control_auto_sbu_det;
	}

	ret = device_create_file(cdev, &dev_attr_control_sbuft_det);
	if (ret < 0) {
		pr_err("%s: failed to create dev_attr_control_sbuft_det\n",
				__func__);
		goto err_attr_control_sbuft_det;
	}

	ret = device_create_file(cdev, &dev_attr_status_look4sbu);
	if (ret < 0) {
		pr_err("%s: failed to create dev_attr_status_look4sbu\n",
				__func__);
		goto err_attr_status_look4sbu;
	}

	ret = device_create_file(cdev, &dev_attr_sbu1_mos_status);
	if (ret < 0) {
		pr_err("%s: failed to create dev_attr_sbu1_mos_status\n",
				__func__);
		goto err_attr_sbu1_mos_status;
	}

	ret = device_create_file(cdev, &dev_attr_sbu2_mos_status);
	if (ret < 0) {
		pr_err("%s: failed to create dev_attr_sbu2_mos_status\n",
				__func__);
		goto err_attr_sbu2_mos_status;
	}

	ret = device_create_file(cdev, &dev_attr_sbu1_ft_status);
	if (ret < 0) {
		pr_err("%s: failed to create dev_attr_sbu1_ft_status\n",
				__func__);
		goto err_attr_sbu1_ft_status;
	}

	ret = device_create_file(cdev, &dev_attr_sbu2_ft_status);
	if (ret < 0) {
		pr_err("%s: failed to create dev_attr_sbu2_ft_status\n",
				__func__);
		goto err_attr_sbu2_ft_status;
	}

	ret = device_create_file(cdev, &dev_attr_threshold1_sbu_dry2wet);
	if (ret < 0) {
		pr_err("%s: failed to create dev_attr_threshold1_sbu_dry2wet\n",
				__func__);
		goto err_attr_threshold1_sbu_dry2wet;
	}

	ret = device_create_file(cdev, &dev_attr_threshold1_sbu_wet2dry);
	if (ret < 0) {
		pr_err("%s: failed to create dev_attr_threshold1_sbu_wet2dry\n",
				__func__);
		goto err_attr_threshold1_sbu_wet2dry;
	}

	ret = device_create_file(cdev, &dev_attr_threshold2_sbuft);
	if (ret < 0) {
		pr_err("%s: failed to create dev_attr_threshold2_sbuft\n",
				__func__);
		goto err_attr_threshold2_sbuft;
	}

	ret = device_create_file(cdev, &dev_attr_task_dry_timer);
	if (ret < 0) {
		pr_err("%s: failed to create dev_attr_task_dry_timer\n",
				__func__);
		goto err_attr_task_dry_timer;
	}

	ret = device_create_file(cdev, &dev_attr_task_wet_timer);
	if (ret < 0) {
		pr_err("%s: failed to create dev_attr_task_wet_timer\n",
				__func__);
		goto err_attr_task_wet_timer;
	}

	ret = device_create_file(cdev, &dev_attr_task_wet_vbus_timer);
	if (ret < 0) {
		pr_err("%s: failed to create dev_attr_task_wet_vbus_timer\n",
				__func__);
		goto err_attr_task_wet_vbus_timer;
	}

	ret = device_create_file(cdev, &dev_attr_control_cc_det);
	if (ret < 0) {
		pr_err("%s: failed to create dev_attr_control_cc_det\n",
				__func__);
		goto err_attr_control_cc_det;
	}

	ret = device_create_file(cdev, &dev_attr_status);
	if (ret < 0) {
		pr_err("%s: failed to create dev_attr_status\n", __func__);
		goto err_attr_status;
	}

	ret = device_create_file(cdev, &dev_attr_moi_status);
	if (ret < 0) {
		pr_err("%s: failed to create dev_attr_moi_status\n", __func__);
		goto err_attr_moi_status;
	}

	ret = device_create_file(cdev, &dev_attr_switch_cc);
	if (ret < 0) {
		pr_err("%s: failed to create dev_attr_switch_cc\n", __func__);
		goto err_attr_switch_cc;
	}

	ret = device_create_file(cdev, &dev_attr_switch_sbu);
	if (ret < 0) {
		pr_err("%s: failed to create dev_attr_switch_sbu\n", __func__);
		goto err_attr_switch_sbu;
	}

	ret = device_create_file(cdev, &dev_attr_threshold1_sbu);
	if (ret < 0) {
		pr_err("%s: failed to create dev_attr_threshold1_sbu\n",
				__func__);
		goto err_attr_threshold1_sbu;
	}

	ret = device_create_file(cdev, &dev_attr_threshold1_cc);
	if (ret < 0) {
		pr_err("%s: failed to create dev_attr_threshold1_cc\n",
				__func__);
		goto err_attr_threshold1_cc;
	}

	ret = device_create_file(cdev, &dev_attr_threshold2_vdry);
	if (ret < 0) {
		pr_err("%s: failed to create dev_attr_threshold2_vdry\n",
				__func__);
		goto err_attr_threshold2_vdry;
	}

	ret = device_create_file(cdev, &dev_attr_regdump);
	if (ret < 0) {
		pr_err("%s: failed to create dev_attr_regdump\n", __func__);
		goto err_attr_regdump;
	}

	return ret;

err_attr_regdump:
	device_remove_file(cdev, &dev_attr_threshold2_vdry);
err_attr_threshold2_vdry:
	device_remove_file(cdev, &dev_attr_threshold1_cc);
err_attr_threshold1_cc:
	device_remove_file(cdev, &dev_attr_threshold1_sbu);
err_attr_threshold1_sbu:
	device_remove_file(cdev, &dev_attr_switch_sbu);
err_attr_switch_sbu:
	device_remove_file(cdev, &dev_attr_switch_cc);
err_attr_switch_cc:
	device_remove_file(cdev, &dev_attr_moi_status);
err_attr_moi_status:
	device_remove_file(cdev, &dev_attr_status);
err_attr_status:
	device_remove_file(cdev, &dev_attr_control_cc_det);
err_attr_control_cc_det:
	device_remove_file(cdev, &dev_attr_task_wet_vbus_timer);
err_attr_task_wet_vbus_timer:
	device_remove_file(cdev, &dev_attr_task_wet_timer);
err_attr_task_wet_timer:
	device_remove_file(cdev, &dev_attr_task_dry_timer);
err_attr_task_dry_timer:
	device_remove_file(cdev, &dev_attr_threshold2_sbuft);
err_attr_threshold2_sbuft:
	device_remove_file(cdev, &dev_attr_threshold1_sbu_wet2dry);
err_attr_threshold1_sbu_wet2dry:
	device_remove_file(cdev, &dev_attr_threshold1_sbu_dry2wet);
err_attr_threshold1_sbu_dry2wet:
	device_remove_file(cdev, &dev_attr_sbu2_ft_status);
err_attr_sbu2_ft_status:
	device_remove_file(cdev, &dev_attr_sbu1_ft_status);
err_attr_sbu1_ft_status:
	device_remove_file(cdev, &dev_attr_sbu2_mos_status);
err_attr_sbu2_mos_status:
	device_remove_file(cdev, &dev_attr_sbu1_mos_status);
err_attr_sbu1_mos_status:
	device_remove_file(cdev, &dev_attr_status_look4sbu);
err_attr_status_look4sbu:
	device_remove_file(cdev, &dev_attr_control_sbuft_det);
err_attr_control_sbuft_det:
	device_remove_file(cdev, &dev_attr_control_auto_sbu_det);
err_attr_control_auto_sbu_det:
	device_remove_file(cdev, &dev_attr_control_sbu_det);
err_attr_control_sbu_det:
	device_remove_file(cdev, &dev_attr_control_dry_det);
err_attr_control_dry_det:
	device_remove_file(cdev, &dev_attr_reset);
err_attr_reset:
	return ret;
}

static void fusb251_delete_attr(struct device *cdev)
{
	device_remove_file(cdev, &dev_attr_regdump);
	device_remove_file(cdev, &dev_attr_threshold2_vdry);
	device_remove_file(cdev, &dev_attr_threshold1_cc);
	device_remove_file(cdev, &dev_attr_threshold1_sbu);
	device_remove_file(cdev, &dev_attr_switch_sbu);
	device_remove_file(cdev, &dev_attr_switch_cc);
	device_remove_file(cdev, &dev_attr_moi_status);
	device_remove_file(cdev, &dev_attr_status);
	device_remove_file(cdev, &dev_attr_control_cc_det);
	device_remove_file(cdev, &dev_attr_task_wet_vbus_timer);
	device_remove_file(cdev, &dev_attr_task_wet_timer);
	device_remove_file(cdev, &dev_attr_task_dry_timer);
	device_remove_file(cdev, &dev_attr_threshold2_sbuft);
	device_remove_file(cdev, &dev_attr_threshold1_sbu_wet2dry);
	device_remove_file(cdev, &dev_attr_threshold1_sbu_dry2wet);
	device_remove_file(cdev, &dev_attr_sbu2_ft_status);
	device_remove_file(cdev, &dev_attr_sbu1_ft_status);
	device_remove_file(cdev, &dev_attr_sbu2_mos_status);
	device_remove_file(cdev, &dev_attr_sbu1_mos_status);
	device_remove_file(cdev, &dev_attr_status_look4sbu);
	device_remove_file(cdev, &dev_attr_control_sbuft_det);
	device_remove_file(cdev, &dev_attr_control_auto_sbu_det);
	device_remove_file(cdev, &dev_attr_control_sbu_det);
	device_remove_file(cdev, &dev_attr_control_dry_det);
	device_remove_file(cdev, &dev_attr_reset);
}

static void fusb251_get_mos_status(struct fusb251 *fusb251,
				enum FUSB251_STATE_TYPE state,
				enum FUSB251_SBU_TYPE sbu)
{

	fusb251_i2c_write_reg(fusb251, FUSB251_CONTROL, CTL_DISABLE_ALL);

	if (sbu == TYPE_SBUFT) {
		if (state == TYPE_DRY) {
			fusb251_config_interface(fusb251, FUSB251_THRESHOLD1,
					SBU_MOS_R_DET_MASK, SBU_MOS_R_DET_SHIFT,
					fusb251_resistance_conver_reg(
					fusb251->threshold_sbu_dry2wet));
		}

		fusb251_i2c_write_reg(fusb251, FUSB251_CONTROL,
				CTL_EN_SBUFT_DET);

		msleep(SBUFT_DELAY);

		fusb251_read_interface(fusb251, FUSB251_MOISTATUS,
			MOISTATUS_SBU1FT_MASK, MOISTATUS_SBU1FT_SHIFT,
			&fusb251->sbu1_ft_status);

		fusb251_read_interface(fusb251, FUSB251_MOISTATUS,
			MOISTATUS_SBU2FT_MASK, MOISTATUS_SBU2FT_SHIFT,
			&fusb251->sbu2_ft_status);

		pr_info("%s: mos_status [SBU1_FT, SBU2_FT] = [%d %d]\n",
				__func__, fusb251->sbu1_ft_status,
				fusb251->sbu2_ft_status);
	} else if (sbu == TYPE_SBU) {
		fusb251_config_interface(fusb251, FUSB251_THRESHOLD1,
				SBU_MOS_R_DET_MASK, SBU_MOS_R_DET_SHIFT,
				fusb251_resistance_conver_reg(
				fusb251->threshold_sbu_wet2dry));

		fusb251_i2c_write_reg(fusb251, FUSB251_CONTROL, CTL_EN_SBU_DET);

		msleep(SBU_DELAY);

		fusb251_read_interface(fusb251, FUSB251_MOISTATUS,
				MOISTATUS_SBU1MOS_MASK, MOISTATUS_SBU1MOS_SHIFT,
				&fusb251->sbu1_mos_status);
		fusb251_read_interface(fusb251, FUSB251_MOISTATUS,
				MOISTATUS_SBU2MOS_MASK, MOISTATUS_SBU2MOS_SHIFT,
				&fusb251->sbu2_mos_status);

		pr_info("%s: mos_status [SBU1, SBU2] = [%d %d]\n", __func__,
				fusb251->sbu1_mos_status,
				fusb251->sbu2_mos_status);
	}

}

static int fusb251_check_usb_attached(struct fusb251 *fusb251)
{
	union power_supply_propval val;
	int ret = 0;
	bool is_usb_attached = false;

	if (!fusb251->usb_psy) {
		fusb251->usb_psy = power_supply_get_by_name("usb");
		if (!fusb251->usb_psy) {
			pr_err("%s: not find usb_psy\n", __func__);
			return false;
		}
	}

	ret = power_supply_get_property(fusb251->usb_psy,
				POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
	if (ret < 0) {
		pr_err("%s: get voltage_now failed, ret = %d\n", __func__, ret);
		return false;
	}

	pr_info("%s: vbus: %d\n", __func__, val.intval);

	if (val.intval > VBUS_INVALID_VOL)
		is_usb_attached = true;

	return is_usb_attached;
}

static void fusb251_report_event(struct fusb251 *fusb251, int event)
{
	struct timespec now_ts;
	int duration_sec = 0;
	int last_event = switch_get_state(&fusb251->st_switch);

	if (last_event != event) {
		switch_set_state(&fusb251->st_switch, event);
		pr_info("%s: event chang %d -> %d", __func__, last_event,
				event);

		get_monotonic_boottime(&now_ts);

		/*
		 * To avoid duration_sec number overflow on metrics service,
		 * "duration_sec" report 0 when event change from EVENT_DRY.
		 */

		if (last_event == TYPE_DRY)
			duration_sec = 0;
		else
			duration_sec = now_ts.tv_sec - fusb251->event_ts.tv_sec;
#if defined(CONFIG_AMAZON_METRICS_LOG) || defined(CONFIG_AMZN_METRICS_LOG)
		fusb251_metrics_log("LiquidDetection",
				"LiquidDetection:def:ld_current_state=%d;CT;1,ld_previous_state=%d;CT;1,ld_duration_sec=%d;CT;1:NR",
				event, last_event, duration_sec);
#endif
#if defined(CONFIG_AMZN_MINERVA_METRICS_LOG) || defined(CONFIG_AMAZON_MINERVA_METRICS_LOG)
		minerva_metrics_log(g_m_buf_fusb251, METRICS_BUFF_SIZE_FUSB251,
				"%s:%s:100:%s,%s,%s,ld_current_state=%d;IN,ld_previous_state=%d;IN,"
				"ld_duration_sec=%d;IN:us-east-1",
				METRICS_LD_GROUP_ID, METRICS_LD_SCHEMA_ID, PREDEFINED_ESSENTIAL_KEY,
				PREDEFINED_MODEL_KEY, PREDEFINED_TZ_KEY, event, last_event, duration_sec);
#endif
		memcpy(&fusb251->event_ts, &now_ts, sizeof(struct timespec));
	} else {
		if (event == TYPE_DRY)
			pr_info("%s: dry\n", __func__);
		else if (event == TYPE_WET)
			pr_info("%s: wet\n", __func__);
		else
			pr_info("%s: wet and vbus exists\n", __func__);
	}
}

void fusb251_vbus_changed_notify(void)
{
	int state;

	if (!g_fusb251)
		return;

	state = switch_get_state(&g_fusb251->st_switch);
	if (fusb251_check_usb_attached(g_fusb251)) {
		if (state == TYPE_WET) {
			fusb251_report_event(g_fusb251, TYPE_WET_VBUS);
			pr_info("%s: state changed: %d -> %d\n",
					__func__, TYPE_WET, TYPE_WET_VBUS);
		}
	} else {
		if (state == TYPE_WET_VBUS) {
			fusb251_report_event(g_fusb251, TYPE_WET);
			pr_info("%s: state changed: %d -> %d\n",
					__func__, TYPE_WET_VBUS, TYPE_WET);
		}
	}
}

static int fusb251_limit_charging_current(struct fusb251 *fusb251,
		bool enable)
{
	int ret = 0;
	union power_supply_propval propval;

	if (!fusb251->batt_psy) {
		fusb251->batt_psy = power_supply_get_by_name("battery");
		if (!fusb251->batt_psy) {
			pr_err("%s: not find batt_psy\n", __func__);
			return ret;
		}
	}

	propval.intval = enable ? IUSB_LIMITATION_UA : -1;
	ret = power_supply_set_property(fusb251->batt_psy,
			POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, &propval);
	if (ret < 0)
		pr_err("%s: psy limit input current failed, ret = %d\n",
				__func__, ret);

	return ret;
}

static inline int fusb251_select_period(struct fusb251 *fusb251, int state)
{
	if (state >= 0 && state < ARRAY_SIZE(fusb251->task_timer))
		return fusb251->task_timer[state];

	pr_err("State doesn't matched, use task_timer[TYPE_WET] as the default\n");
	return fusb251->task_timer[TYPE_WET];
}

static void fusb251_mos_det_work(struct work_struct *work)
{
	struct fusb251 *fusb251 =
		container_of(work, struct fusb251, mos_det_work.work);
	int state = 0;
	int sleep_interval = 0;
	int ovp_cc = 0;
	int ovp_sbu = 0;

	if (unlikely(system_state < SYSTEM_RUNNING)) {
		sleep_interval = TASK_DELAY_WAKEUP_SEC;
		goto rerun;
	}

	if (fusb251->fusb251_wake_lock.active == 0)
		__pm_stay_awake(&fusb251->fusb251_wake_lock);

	fusb251_read_interface(fusb251, FUSB251_STATUS, STATUS_OVP_CC_MASK,
			STATUS_OVP_CC_SHIFT, &fusb251->ovp_cc_status);
	fusb251_read_interface(fusb251, FUSB251_STATUS, STATUS_OVP_SBU_MASK,
			STATUS_OVP_SBU_SHIFT, &fusb251->ovp_sbu_status);

	ovp_cc = fusb251->ovp_cc_status;
	ovp_sbu = fusb251->ovp_sbu_status;

	state = switch_get_state(&fusb251->st_switch);

	if (state == TYPE_DRY) {
		if (ovp_cc || ovp_sbu) {
			pr_info("%s: OVP detected [%s]\n", __func__,
					ovp_cc ? "CC" : "SBU");
			fusb251_report_event(fusb251, TYPE_WET_VBUS);
			tcpm_typec_change_role(fusb251->tcpc, TYPEC_ROLE_SNK);
			fusb251_limit_charging_current(fusb251, true);
			goto skip;
		}

		fusb251_get_mos_status(fusb251, TYPE_DRY, TYPE_SBUFT);

		if (fusb251->sbu1_ft_status || fusb251->sbu2_ft_status) {
			if (fusb251_check_usb_attached(fusb251))
				fusb251_report_event(fusb251, TYPE_WET_VBUS);
			else
				fusb251_report_event(fusb251, TYPE_WET);

			tcpm_typec_change_role(fusb251->tcpc, TYPEC_ROLE_SNK);
			fusb251_limit_charging_current(fusb251, true);

		} else {
			fusb251_report_event(fusb251, TYPE_DRY);
		}
	} else {
		/* step1: check ovp */
		if (ovp_cc || ovp_sbu) {
			pr_info("%s: OVP detected [%s]\n", __func__,
					ovp_cc ? "CC" : "SBU");
			fusb251_report_event(fusb251, TYPE_WET_VBUS);
			goto skip;
		}

		/* step2: check sbu float */
		fusb251_get_mos_status(fusb251, TYPE_WET, TYPE_SBUFT);

		if (fusb251->sbu1_ft_status || fusb251->sbu2_ft_status) {
			if (fusb251_check_usb_attached(fusb251) &&
					(state != TYPE_WET_VBUS))
				fusb251_report_event(fusb251, TYPE_WET_VBUS);
			else
				fusb251_report_event(fusb251, state);
			goto skip;
		}

		/* step3: check sbu */
		fusb251_get_mos_status(fusb251, TYPE_WET, TYPE_SBU);

		if (!(fusb251->sbu1_mos_status || fusb251->sbu2_mos_status)) {
			fusb251_report_event(fusb251, TYPE_DRY);
			tcpm_typec_change_role(fusb251->tcpc,
					TYPEC_ROLE_TRY_SNK);
			fusb251_limit_charging_current(fusb251, false);
		} else {
			if (fusb251_check_usb_attached(fusb251) &&
					(state != TYPE_WET_VBUS))
				fusb251_report_event(fusb251, TYPE_WET_VBUS);
			else
				fusb251_report_event(fusb251, state);
		}
	}

skip:
	fusb251_i2c_write_reg(fusb251, FUSB251_CONTROL, CTL_DISABLE_ALL);
	state = switch_get_state(&fusb251->st_switch);
	sleep_interval = fusb251_select_period(fusb251, state);

rerun:
	queue_delayed_work(system_freezable_wq, &fusb251->mos_det_work,
				sleep_interval * HZ);

	if (fusb251->fusb251_wake_lock.active == 1)
		__pm_relax(&fusb251->fusb251_wake_lock);
}

static int fusb251_parse_dt(struct fusb251 *fusb251, struct device *dev)
{
	struct device_node *np = dev->of_node;
	int ret = 0;

	dev_info(fusb251->dev, "%s\n", __func__);

	if (!np) {
		dev_err(fusb251->dev, "%s: no device node\n", __func__);
		return -EINVAL;
	}

	ret = of_property_read_u32(np, "threshold1_sbu_det_dry2wet",
			&fusb251->threshold_sbu_dry2wet);
	if (ret < 0) {
		pr_err("%s: not define threshold1_sbu_det_dry2wet, use THRESHOLD_SBU_DRY2WET\n",
				__func__);
		fusb251->threshold_sbu_dry2wet = THRESHOLD_SBU_DRY2WET;
	}

	ret = of_property_read_u32(np, "threshold1_sbu_det_wet2dry",
			&fusb251->threshold_sbu_wet2dry);
	if (ret < 0) {
		pr_err("%s: not define threshold1_sbu_det_wet2dry, use THRESHOLD_SBU_WET2DRY\n",
				__func__);
		fusb251->threshold_sbu_wet2dry = THRESHOLD_SBU_WET2DRY;
	}

	ret = of_property_read_u32(np, "threshold2_sbu_float_det",
			&fusb251->threshold_sbuft);
	if (ret < 0) {
		pr_err("%s: not define threshold2_sbu_float_det, use THRESHOLD_SBUFT\n",
				__func__);
		fusb251->threshold_sbuft = THRESHOLD_SBUFT;
	}

	pr_info("%s: threshold_sbu_dry2wet[%d]\n",
			__func__, fusb251->threshold_sbu_dry2wet);
	pr_info("%s: threshold_sbu_wet2dry[%d]\n",
			__func__, fusb251->threshold_sbu_wet2dry);
	pr_info("%s: threshold_sbuft[%d]\n", __func__,
			fusb251->threshold_sbuft);

	return 0;
}

static int fusb251_init(struct fusb251 *fusb251)
{
	pr_info("%s enter\n", __func__);

	/* reset moisture */
	if (fusb251_i2c_write_reg(fusb251, FUSB251_RESET, RESET_MOISTURE) < 0) {
		pr_info("%s: reset moisture failed\n", __func__);
		return -1;
	}

	/* mask all the interrupt */
	if (fusb251_i2c_write_reg(fusb251,
				FUSB251_INT_MASK, MSK_I_MASK_ALL) < 0) {
		pr_info("%s: mask all interrupt failed\n", __func__);
		return -1;
	}
	/*
	 * CC moisture detection time
	 * 400us settle time with 3 time adc reading
	 */
	if (fusb251_i2c_write_reg(fusb251, FUSB251_TIMER2,
			ADC_READ_TIMES_3 | CC_SETTLE_400US) < 0) {
		pr_info("%s: failed to set up CC moisture detection time\n",
				__func__);
		return -1;
	}

	fusb251_config_interface(fusb251, FUSB251_THRESHOLD1,
		SBU_MOS_R_DET_MASK, SBU_MOS_R_DET_SHIFT,
		fusb251_resistance_conver_reg(
			fusb251->threshold_sbu_dry2wet));
	fusb251_config_interface(fusb251, FUSB251_THRESHOLD1,
		CC_MOS_R_DET_MASK, CC_MOS_R_DET_SHIFT, CC_MOS_R_DET_480K);

	/*
	 * set sbu float detection
	 * default set to 100mv
	 */
	fusb251_config_interface(fusb251, FUSB251_THRESHOLD2,
		SBU_FLOAT_DET_MASK, SBU_FLOAT_DET_SHIFT,
		fusb251_sbuft_voltage_convert_reg(fusb251->threshold_sbuft));

	return 0;
}

static int fusb251_i2c_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int ret = 0;
	struct fusb251 *fusb251 = NULL;
	unsigned char deviceID = 0;

	pr_info("%s enter\n", __func__);

#ifdef CONFIG_AMAZON_LD_SWITCH
	if (liquid_id_status == FUSB251_NO_MOUNT) {
		pr_err("%s: FUSB251 is not mounted, gpio53[%d]\n", __func__, liquid_id_status);
		return -ENODEV;
	}
#endif

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s: check functionality failed\n", __func__);
		return -ENODEV;
	}

	fusb251 = devm_kzalloc(&client->dev, sizeof(*fusb251), GFP_KERNEL);
	if (!fusb251)
		return -ENOMEM;

	memset(fusb251, 0, sizeof(*fusb251));

	fusb251->i2c = client;
	fusb251->dev = &client->dev;

	mutex_init(&fusb251->fusb251_i2c_mutex);

	ret = fusb251_parse_dt(fusb251, &client->dev);
	if (ret < 0) {
		dev_err(fusb251->dev, "%s: parse dt failed\n", __func__);
		goto err_parse_dt;
	}

	i2c_set_clientdata(client, fusb251);

	fusb251_i2c_read_reg(fusb251, FUSB251_PRODUCT_ID, &deviceID);
	if (deviceID != FUSB251_DEVICE_ID) {
		pr_err("%s: error device id [%02x]\n", __func__, deviceID);
		ret = -ENODEV;
		goto err_id;
	}

	fusb251->tcpc = tcpc_dev_get_by_name("type_c_port0");
	if (!fusb251->tcpc) {
		pr_err("%s: get type_c_port0 fail\n", __func__);
		ret = -EPROBE_DEFER;
		goto err_tcpc;
	}

	ret = fusb251_init(fusb251);
	if (ret < 0) {
		pr_err("%s: fusb251_init failed\n", __func__);
		goto err_init;
	}

	fusb251->st_switch.name = "ld";
	fusb251->st_switch.index = 0;
	fusb251->st_switch.state = TYPE_DRY;
	ret = switch_dev_register(&fusb251->st_switch);
	if (ret) {
		pr_err("%s: switch_dev_register failed\n", __func__);
		goto err_switch;
	}

	fusb251->task_timer[TYPE_DRY] = DRY_WORK_PERIOD;
	fusb251->task_timer[TYPE_WET] = WET_WORK_PERIOD;
	fusb251->task_timer[TYPE_WET_VBUS] = WET_AND_VBUS_WORK_PERIOD;

	ret = fusb251_create_attr(fusb251->dev);
	if (ret < 0)
		goto err_attr;

	g_fusb251 = fusb251;
	get_monotonic_boottime(&fusb251->event_ts);

	wakeup_source_init(&fusb251->fusb251_wake_lock, "fusb251");
	INIT_DELAYED_WORK(&fusb251->mos_det_work,
		fusb251_mos_det_work);
	queue_delayed_work(system_freezable_wq, &fusb251->mos_det_work, 0);
	pr_info("%s: probe successfully\n", __func__);

	return 0;

err_attr:
	switch_dev_unregister(&fusb251->st_switch);
err_switch:
err_init:
err_tcpc:
err_id:
err_parse_dt:
	devm_kfree(&client->dev, fusb251);
	fusb251 = NULL;

	return ret;
}

static int fusb251_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static void fusb251_i2c_shutdown(struct i2c_client *client)
{
	struct fusb251 *fusb251 =
			(struct fusb251 *)i2c_get_clientdata(client);

	g_fusb251 = NULL;

	cancel_delayed_work_sync(&fusb251->mos_det_work);

	fusb251_delete_attr(fusb251->dev);

	fusb251 = NULL;
}

static const struct i2c_device_id fusb251_i2c_id[] = {
	{"fusb251", 0},
	{},
};

#ifdef CONFIG_OF
static const struct of_device_id of_fusb251_match_table[] = {
	{.compatible = "on,fusb251",},
	{},
};
MODULE_DEVICE_TABLE(of, of_fusb251_match_table);
#endif

static struct i2c_driver fusb251_i2c_driver = {
	.driver = {
		.name = "fusb251",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(of_fusb251_match_table),
	},
	.probe =    fusb251_i2c_probe,
	.remove =   fusb251_i2c_remove,
	.shutdown =   fusb251_i2c_shutdown,
	.id_table = fusb251_i2c_id,
};

static int __init fusb251_i2c_init(void)
{
	int ret = 0;

	ret = i2c_add_driver(&fusb251_i2c_driver);
	if (ret != 0)
		pr_err("%s: fusb251 i2c init failed!\n", __func__);

	return ret;
}
late_initcall(fusb251_i2c_init);

static void __exit fusb251_i2c_exit(void)
{
	i2c_del_driver(&fusb251_i2c_driver);
}
module_exit(fusb251_i2c_exit);

MODULE_AUTHOR("jianli.gao@onsemi.com");
MODULE_DESCRIPTION("TYPEC FUSB251 driver");
MODULE_LICENSE("GPL v2");
