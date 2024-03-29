/*
 * STMicroelectronics st_lsm6dso sensor driver
 *
 * Copyright 2017 STMicroelectronics Inc.
 *
 * Lorenzo Bianconi <lorenzo.bianconi@st.com>
 *
 * Licensed under the GPL-2.
 */

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/interrupt.h>
#include <linux/pm.h>
#include <linux/regulator/consumer.h>

#include <linux/platform_data/st_sensors_pdata.h>

#include "st_lsm6dso.h"

#define ST_LSM6DSO_REG_INT1_CTRL_ADDR		0x0d
#define ST_LSM6DSO_REG_INT2_CTRL_ADDR		0x0e
#define ST_LSM6DSO_REG_FIFO_TH_MASK		BIT(3)
#define ST_LSM6DSO_REG_WHOAMI_ADDR		0x0f
#define ST_LSM6DSO_WHOAMI_VAL			0x6c
#define ST_LSM6DSO_CTRL1_XL_ADDR		0x10
#define ST_LSM6DSO_CTRL2_G_ADDR			0x11
#define ST_LSM6DSO_REG_CTRL3_C_ADDR		0x12
#define ST_LSM6DSO_REG_SW_RESET_MASK		BIT(0)
#define ST_LSM6DSO_REG_BOOT_MASK		BIT(7)
#define ST_LSM6DSO_REG_BDU_MASK			BIT(6)
#define ST_LSM6DSO_REG_CTRL5_C_ADDR		0x14
#define ST_LSM6DSO_REG_ROUNDING_MASK		GENMASK(6, 5)
#define ST_LSM6DSO_REG_CTRL6_C_ADDR		0x15
#define ST_LSM6DSO_REG_USR_OFF_W_MASK		BIT(3)
#define ST_LSM6DSO_REG_CTRL10_C_ADDR		0x19
#define ST_LSM6DSO_REG_TIMESTAMP_EN_MASK	BIT(5)

#define ST_LSM6DSO_REG_OUTX_L_A_ADDR		0x28
#define ST_LSM6DSO_REG_OUTY_L_A_ADDR		0x2a
#define ST_LSM6DSO_REG_OUTZ_L_A_ADDR		0x2c

#define ST_LSM6DSO_REG_OUTX_L_G_ADDR		0x22
#define ST_LSM6DSO_REG_OUTY_L_G_ADDR		0x24
#define ST_LSM6DSO_REG_OUTZ_L_G_ADDR		0x26

#define ST_LSM6DSO_REG_TAP_CFG0_ADDR		0x56
#define ST_LSM6DSO_REG_LIR_MASK			BIT(0)

#define ST_LSM6DSO_REG_FIFO_CTRL3_ADDR		0x09
#define ST_LSM6DSO_REG_BDR_XL_MASK		GENMASK(3, 0)
#define ST_LSM6DSO_REG_BDR_GY_MASK		GENMASK(7, 4)

#define ST_LSM6DSO_REG_MD1_CFG_ADDR		0x5e
#define ST_LSM6DSO_REG_MD2_CFG_ADDR		0x5f
#define ST_LSM6DSO_REG_INT2_TIMESTAMP_MASK	BIT(0)
#define ST_LSM6DSO_REG_INT_EMB_FUNC_MASK	BIT(1)
#define ST_LSM6DSO_INTERNAL_FREQ_FINE		0x63
#define ST_LSM6DSO_REG_X_OFS_USR		0x73
#define ST_LSM6DSO_REG_X_OFS_USR_MASK		GENMASK(7, 0)
#define ST_LSM6DSO_REG_Y_OFS_USR		0x74
#define ST_LSM6DSO_REG_Y_OFS_USR_MASK		GENMASK(7, 0)
#define ST_LSM6DSO_REG_Z_OFS_USR		0x75
#define ST_LSM6DSO_REG_Z_OFS_USR_MASK		GENMASK(7, 0)

/* Embedded function */
#define ST_LSM6DSO_REG_EMB_FUNC_INT1_ADDR	0x0a
#define ST_LSM6DSO_REG_EMB_FUNC_INT2_ADDR	0x0e

#define ST_LSM6DSO_ACCEL_OFFSET_WEIGHT_DEFAULT	0
#define ST_LSM6DSO_ACCEL_X_OFFSET_DEFAULT	0
#define ST_LSM6DSO_ACCEL_Y_OFFSET_DEFAULT	0
#define ST_LSM6DSO_ACCEL_Z_OFFSET_DEFAULT	0

struct st_lsm6dso_std_entry {
	u16 odr;
	u8 val;
};

struct st_lsm6dso_std_entry st_lsm6dso_std_table[] = {
	{  12, 7 },
	{  26, 7 },
	{  52, 8 },
	{ 104, 8 },
	{ 208, 8 },
	{ 416, 8 },
	{ 833, 8 },
};

static const struct st_lsm6dso_odr_table_entry st_lsm6dso_odr_table[] = {
	[ST_LSM6DSO_ID_ACC] = {
		.reg = {
			.addr = ST_LSM6DSO_CTRL1_XL_ADDR,
			.mask = GENMASK(7, 4),
		},
		.odr_avl[0] = {   0, 0x00 },
		.odr_avl[1] = {  12, 0x01 },
		.odr_avl[2] = {  26, 0x02 },
		.odr_avl[3] = {  52, 0x03 },
		.odr_avl[4] = { 104, 0x04 },
		.odr_avl[5] = { 208, 0x05 },
		.odr_avl[6] = { 416, 0x06 },
		.odr_avl[7] = { 833, 0x07 },
	},
	[ST_LSM6DSO_ID_GYRO] = {
		.reg = {
			.addr = ST_LSM6DSO_CTRL2_G_ADDR,
			.mask = GENMASK(7, 4),
		},
		.odr_avl[0] = {   0, 0x00 },
		.odr_avl[1] = {  12, 0x01 },
		.odr_avl[2] = {  26, 0x02 },
		.odr_avl[3] = {  52, 0x03 },
		.odr_avl[4] = { 104, 0x04 },
		.odr_avl[5] = { 208, 0x05 },
		.odr_avl[6] = { 416, 0x06 },
		.odr_avl[7] = { 833, 0x07 },
	}
};

static const struct st_lsm6dso_fs_table_entry st_lsm6dso_fs_table[] = {
	[ST_LSM6DSO_ID_ACC] = {
		.size = ST_LSM6DSO_FS_ACC_LIST_SIZE,
		.fs_avl[0] = {
			.reg = {
				.addr = ST_LSM6DSO_CTRL1_XL_ADDR,
				.mask = GENMASK(3, 2),
			},
			.gain = ST_LSM6DSO_ACC_FS_2G_GAIN,
			.val = 0x0,
		},
		.fs_avl[1] = {
			.reg = {
				.addr = ST_LSM6DSO_CTRL1_XL_ADDR,
				.mask = GENMASK(3, 2),
			},
			.gain = ST_LSM6DSO_ACC_FS_4G_GAIN,
			.val = 0x2,
		},
		.fs_avl[2] = {
			.reg = {
				.addr = ST_LSM6DSO_CTRL1_XL_ADDR,
				.mask = GENMASK(3, 2),
			},
			.gain = ST_LSM6DSO_ACC_FS_8G_GAIN,
			.val = 0x3,
		},
		.fs_avl[3] = {
			.reg = {
				.addr = ST_LSM6DSO_CTRL1_XL_ADDR,
				.mask = GENMASK(3, 2),
			},
			.gain = ST_LSM6DSO_ACC_FS_16G_GAIN,
			.val = 0x1,
		},
	},
	[ST_LSM6DSO_ID_GYRO] = {
		.size = ST_LSM6DSO_FS_GYRO_LIST_SIZE,
		.fs_avl[0] = {
			.reg = {
				.addr = ST_LSM6DSO_CTRL2_G_ADDR,
				.mask = GENMASK(3, 0),
			},
			.gain = ST_LSM6DSO_GYRO_FS_250_GAIN,
			.val = 0x0,
		},
		.fs_avl[1] = {
			.reg = {
				.addr = ST_LSM6DSO_CTRL2_G_ADDR,
				.mask = GENMASK(3, 0),
			},
			.gain = ST_LSM6DSO_GYRO_FS_500_GAIN,
			.val = 0x4,
		},
		.fs_avl[2] = {
			.reg = {
				.addr = ST_LSM6DSO_CTRL2_G_ADDR,
				.mask = GENMASK(3, 0),
			},
			.gain = ST_LSM6DSO_GYRO_FS_1000_GAIN,
			.val = 0x8,
		},
		.fs_avl[3] = {
			.reg = {
				.addr = ST_LSM6DSO_CTRL2_G_ADDR,
				.mask = GENMASK(3, 0),
			},
			.gain = ST_LSM6DSO_GYRO_FS_2000_GAIN,
			.val = 0x0C,
		},
	}
};

static const struct iio_chan_spec st_lsm6dso_acc_channels[] = {
	ST_LSM6DSO_DATA_CHANNEL(IIO_ACCEL, ST_LSM6DSO_REG_OUTX_L_A_ADDR,
				1, IIO_MOD_X, 0, 16, 16, 's'),
	ST_LSM6DSO_DATA_CHANNEL(IIO_ACCEL, ST_LSM6DSO_REG_OUTY_L_A_ADDR,
				1, IIO_MOD_Y, 1, 16, 16, 's'),
	ST_LSM6DSO_DATA_CHANNEL(IIO_ACCEL, ST_LSM6DSO_REG_OUTZ_L_A_ADDR,
				1, IIO_MOD_Z, 2, 16, 16, 's'),
	ST_LSM6DSO_EVENT_CHANNEL(IIO_ACCEL, flush),
	IIO_CHAN_SOFT_TIMESTAMP(3),
};

static const struct iio_chan_spec st_lsm6dso_gyro_channels[] = {
	ST_LSM6DSO_DATA_CHANNEL(IIO_ANGL_VEL, ST_LSM6DSO_REG_OUTX_L_G_ADDR,
				1, IIO_MOD_X, 0, 16, 16, 's'),
	ST_LSM6DSO_DATA_CHANNEL(IIO_ANGL_VEL, ST_LSM6DSO_REG_OUTY_L_G_ADDR,
				1, IIO_MOD_Y, 1, 16, 16, 's'),
	ST_LSM6DSO_DATA_CHANNEL(IIO_ANGL_VEL, ST_LSM6DSO_REG_OUTZ_L_G_ADDR,
				1, IIO_MOD_Z, 2, 16, 16, 's'),
	ST_LSM6DSO_EVENT_CHANNEL(IIO_ANGL_VEL, flush),
	IIO_CHAN_SOFT_TIMESTAMP(3),
};

static const struct iio_chan_spec st_lsm6dso_step_counter_channels[] = {
	{
		.type = IIO_STEP_COUNTER,
		.scan_index = 0,
		.scan_type = {
			.sign = 'u',
			.realbits = 16,
			.storagebits = 16,
			.endianness = IIO_LE,
		},
	},
	ST_LSM6DSO_EVENT_CHANNEL(IIO_STEP_COUNTER, flush),
	IIO_CHAN_SOFT_TIMESTAMP(1),
};

static const struct iio_chan_spec st_lsm6dso_step_detector_channels[] = {
	ST_LSM6DSO_EVENT_CHANNEL(IIO_STEP_DETECTOR, thr),
};

static const struct iio_chan_spec st_lsm6dso_sign_motion_channels[] = {
	ST_LSM6DSO_EVENT_CHANNEL(IIO_SIGN_MOTION, thr),
};

static const struct iio_chan_spec st_lsm6dso_tilt_channels[] = {
	ST_LSM6DSO_EVENT_CHANNEL(IIO_TILT, thr),
};

static const struct iio_chan_spec st_lsm6dso_glance_channels[] = {
	ST_LSM6DSO_EVENT_CHANNEL(IIO_GESTURE, thr),
};

static const struct iio_chan_spec st_lsm6dso_motion_channels[] = {
	ST_LSM6DSO_EVENT_CHANNEL(IIO_GESTURE, thr),
};

static const struct iio_chan_spec st_lsm6dso_no_motion_channels[] = {
	ST_LSM6DSO_EVENT_CHANNEL(IIO_GESTURE, thr),
};

static const struct iio_chan_spec st_lsm6dso_wakeup_channels[] = {
	ST_LSM6DSO_EVENT_CHANNEL(IIO_GESTURE, thr),
};

static const struct iio_chan_spec st_lsm6dso_pickup_channels[] = {
	ST_LSM6DSO_EVENT_CHANNEL(IIO_GESTURE, thr),
};

static const struct iio_chan_spec st_lsm6dso_orientation_channels[] = {
	{
		.type = IIO_GESTURE,
		.scan_index = 0,
		.scan_type = {
			.sign = 'u',
			.realbits = 8,
			.storagebits = 8,
		},
	},
	IIO_CHAN_SOFT_TIMESTAMP(1),
};

static const struct iio_chan_spec st_lsm6dso_wrist_channels[] = {
	ST_LSM6DSO_EVENT_CHANNEL(IIO_GESTURE, thr),
};

int __st_lsm6dso_write_with_mask(struct st_lsm6dso_hw *hw, u8 addr, u8 mask,
				 u8 val)
{
	u8 data;
	int err;

	mutex_lock(&hw->lock);

	err = hw->tf->read(hw->dev, addr, sizeof(data), &data);
	if (err < 0) {
		dev_err(hw->dev, "failed to read %02x register\n", addr);
		goto out;
	}

	data = (data & ~mask) | ((val << __ffs(mask)) & mask);

	err = hw->tf->write(hw->dev, addr, sizeof(data), &data);
	if (err < 0)
		dev_err(hw->dev, "failed to write %02x register\n", addr);

out:
	mutex_unlock(&hw->lock);

	return err;
}

static int st_lsm6dso_check_whoami(struct st_lsm6dso_hw *hw)
{
	int err;
	u8 data;

	err = hw->tf->read(hw->dev, ST_LSM6DSO_REG_WHOAMI_ADDR, sizeof(data),
			   &data);
	if (err < 0) {
		dev_err(hw->dev, "failed to read whoami register\n");
		return err;
	}

	if (data != ST_LSM6DSO_WHOAMI_VAL) {
		dev_err(hw->dev, "unsupported whoami [%02x]\n", data);
		return -ENODEV;
	}

	return 0;
}

static int st_lsm6dso_get_odr_calibration(struct st_lsm6dso_hw *hw)
{
	int err;
	s8 data;

	err = hw->tf->read(hw->dev, ST_LSM6DSO_INTERNAL_FREQ_FINE, sizeof(data),
			   (u8 *)&data);
	if (err < 0) {
		dev_err(hw->dev, "failed to read %d register\n",
				ST_LSM6DSO_INTERNAL_FREQ_FINE);
		return err;
	}

	hw->odr_calib = (data * 15) / 10000;
	dev_err(hw->dev, "ODR Calibration Factor %d\n", hw->odr_calib);

	return 0;
}

static int st_lsm6dso_set_full_scale(struct st_lsm6dso_sensor *sensor,
				     u32 gain)
{
	enum st_lsm6dso_sensor_id id = sensor->id;
	int i, err;
	u8 val;

	for (i = 0; i < st_lsm6dso_fs_table[id].size; i++)
		if (st_lsm6dso_fs_table[id].fs_avl[i].gain == gain)
			break;

	if (i == st_lsm6dso_fs_table[id].size)
		return -EINVAL;

	val = st_lsm6dso_fs_table[id].fs_avl[i].val;
	err = st_lsm6dso_write_with_mask(sensor->hw,
					 st_lsm6dso_fs_table[id].fs_avl[i].reg.addr,
					 st_lsm6dso_fs_table[id].fs_avl[i].reg.mask,
					 val);
	if (err < 0)
		return err;

	sensor->gain = gain;

	return 0;
}

int st_lsm6dso_get_odr_val(enum st_lsm6dso_sensor_id id, u16 odr, u8 *val)
{
	int i;

	for (i = 0; i < ST_LSM6DSO_ODR_LIST_SIZE; i++)
		if (st_lsm6dso_odr_table[id].odr_avl[i].hz >= odr)
			break;

	if (i == ST_LSM6DSO_ODR_LIST_SIZE)
		return -EINVAL;

	*val = st_lsm6dso_odr_table[id].odr_avl[i].val;

	return 0;
}

static int st_lsm6dso_set_std_level(struct st_lsm6dso_sensor *sensor, u16 odr)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(st_lsm6dso_std_table); i++)
		if (st_lsm6dso_std_table[i].odr == odr)
			break;

	if (i == ARRAY_SIZE(st_lsm6dso_std_table))
		return -EINVAL;

	sensor->std_level = st_lsm6dso_std_table[i].val;
	sensor->std_samples = 0;

	return 0;
}

static u16 st_lsm6dso_check_odr_dependency(struct st_lsm6dso_hw *hw, u16 odr,
					   enum st_lsm6dso_sensor_id ref_id)
{
	struct st_lsm6dso_sensor *ref = iio_priv(hw->iio_devs[ref_id]);
	bool enable = odr > 0;
	u16 ret;

	if (enable) {
		if (hw->enable_mask & BIT(ref_id))
			ret = max_t(u16, ref->odr, odr);
		else
			ret = odr;
	} else {
		ret = (hw->enable_mask & BIT(ref_id)) ? ref->odr : 0;
	}

	return ret;
}

static int st_lsm6dso_set_odr(struct st_lsm6dso_sensor *sensor, u16 req_odr)
{
	struct st_lsm6dso_hw *hw = sensor->hw;
	enum st_lsm6dso_sensor_id id = sensor->id;
	int err;
	u8 val;

	switch (sensor->id) {
	case ST_LSM6DSO_ID_STEP_COUNTER:
	case ST_LSM6DSO_ID_STEP_DETECTOR:
	case ST_LSM6DSO_ID_SIGN_MOTION:
	case ST_LSM6DSO_ID_NO_MOTION:
	case ST_LSM6DSO_ID_MOTION:
	case ST_LSM6DSO_ID_GLANCE:
	case ST_LSM6DSO_ID_WAKEUP:
	case ST_LSM6DSO_ID_PICKUP:
	case ST_LSM6DSO_ID_ORIENTATION:
	case ST_LSM6DSO_ID_WRIST_TILT:
	case ST_LSM6DSO_ID_TILT:
	case ST_LSM6DSO_ID_EXT0:
	case ST_LSM6DSO_ID_EXT1:
	case ST_LSM6DSO_ID_ACC: {
		u16 odr;
		int i;

		id = ST_LSM6DSO_ID_ACC;
		for (i = ST_LSM6DSO_ID_ACC; i <= ST_LSM6DSO_ID_TILT; i++) {
			if (!hw->iio_devs[i])
				continue;

			if (i == sensor->id)
				continue;

			odr = st_lsm6dso_check_odr_dependency(hw, req_odr, i);
			if (odr != req_odr)
				/* device already configured */
				return 0;
		}
		break;
	}
	default:
		break;
	}

	err = st_lsm6dso_get_odr_val(id, req_odr, &val);
	if (err < 0)
		return err;

	return st_lsm6dso_write_with_mask(hw, st_lsm6dso_odr_table[id].reg.addr,
					  st_lsm6dso_odr_table[id].reg.mask,
					  val);
}

int st_lsm6dso_sensor_set_enable(struct st_lsm6dso_sensor *sensor,
				 bool enable)
{
	u16 odr = enable ? sensor->odr : 0;
	int err;

	err = st_lsm6dso_set_odr(sensor, odr);
	if (err < 0)
		return err;

	if (enable)
		sensor->hw->enable_mask |= BIT(sensor->id);
	else
		sensor->hw->enable_mask &= ~BIT(sensor->id);

	return 0;
}

static int st_lsm6dso_read_oneshot(struct st_lsm6dso_sensor *sensor,
				   u8 addr, int *val)
{
	int err, delay;
	__le16 data;

	err = st_lsm6dso_sensor_set_enable(sensor, true);
	if (err < 0)
		return err;

	delay = 1000000 / sensor->odr;
	usleep_range(delay, 2 * delay);

	err = st_lsm6dso_read_atomic(sensor->hw, addr, sizeof(data),
				     (u8 *)&data);
	if (err < 0)
		return err;

	st_lsm6dso_sensor_set_enable(sensor, false);

	*val = (s16)le16_to_cpu(data);

	return IIO_VAL_INT;
}

static int st_lsm6dso_read_raw(struct iio_dev *iio_dev,
			       struct iio_chan_spec const *ch,
			       int *val, int *val2, long mask)
{
	struct st_lsm6dso_sensor *sensor = iio_priv(iio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&iio_dev->mlock);
		if (iio_buffer_enabled(iio_dev)) {
			ret = -EBUSY;
			mutex_unlock(&iio_dev->mlock);
			break;
		}
		ret = st_lsm6dso_read_oneshot(sensor, ch->address, val);
		mutex_unlock(&iio_dev->mlock);
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = sensor->odr;
		ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		*val2 = sensor->gain;
		ret = IIO_VAL_INT_PLUS_MICRO;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int st_lsm6dso_write_raw(struct iio_dev *iio_dev,
				struct iio_chan_spec const *chan,
				int val, int val2, long mask)
{
	struct st_lsm6dso_sensor *sensor = iio_priv(iio_dev);
	int err;

	mutex_lock(&iio_dev->mlock);

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		err = st_lsm6dso_set_full_scale(sensor, val2);
		break;
	case IIO_CHAN_INFO_SAMP_FREQ: {
		u8 data;

		err = st_lsm6dso_set_std_level(sensor, val);
		if (err < 0)
			break;

		err = st_lsm6dso_get_odr_val(sensor->id, val, &data);
		if (!err)
			sensor->odr = val;
		break;
	}
	default:
		err = -EINVAL;
		break;
	}

	mutex_unlock(&iio_dev->mlock);

	return err;
}

 static int st_lsm6dso_debugfs_reg_access(struct iio_dev *iio_dev,
 			      unsigned int reg, unsigned int writeval,
 			      unsigned int *readval)
 {
	struct st_lsm6dso_sensor *sensor = iio_priv(iio_dev);
	struct st_lsm6dso_hw *hw = sensor->hw;
	int err;
	u8 data;

	mutex_lock(&hw->lock);

	if (!readval) {
		data = (u8)writeval;
		err = hw->tf->write(hw->dev, (u8)reg, sizeof(data), &data);
		if (err < 0)
			dev_err(hw->dev, "failed to write %02x register\n", reg);
		goto out;
	}

	err = hw->tf->read(hw->dev, (u8)reg, sizeof(data), &data);
 	if (err < 0) {
		dev_err(hw->dev, "failed to read %02x register\n", reg);
		goto out;
	 }
	*readval = data;

out:
	mutex_unlock(&hw->lock);
	return (err < 0 ? err : 0);
 }

static int st_lsm6dso_read_event_config(struct iio_dev *iio_dev,
					const struct iio_chan_spec *chan,
					enum iio_event_type type,
					enum iio_event_direction dir)
{
	struct st_lsm6dso_sensor *sensor = iio_priv(iio_dev);
	struct st_lsm6dso_hw *hw = sensor->hw;

	return !!(hw->enable_mask & BIT(sensor->id));
}

static int st_lsm6dso_write_event_config(struct iio_dev *iio_dev,
					 const struct iio_chan_spec *chan,
					 enum iio_event_type type,
					 enum iio_event_direction dir,
					 int state)
{
	struct st_lsm6dso_sensor *sensor = iio_priv(iio_dev);
	int err;

	mutex_lock(&iio_dev->mlock);
	err = st_lsm6dso_embfunc_sensor_set_enable(sensor, state);
	mutex_unlock(&iio_dev->mlock);

	return err;
}

static ssize_t
st_lsm6dso_sysfs_sampling_frequency_avail(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct st_lsm6dso_sensor *sensor = iio_priv(dev_get_drvdata(dev));
	enum st_lsm6dso_sensor_id id = sensor->id;
	int i, len = 0;

	for (i = 0; i < ST_LSM6DSO_ODR_LIST_SIZE; i++) {
		if (!st_lsm6dso_odr_table[id].odr_avl[i].hz)
			continue;

		len += scnprintf(buf + len, PAGE_SIZE - len, "%d ",
				 st_lsm6dso_odr_table[id].odr_avl[i].hz);
	}

	buf[len - 1] = '\n';

	return len;
}

static ssize_t st_lsm6dso_sysfs_scale_avail(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct st_lsm6dso_sensor *sensor = iio_priv(dev_get_drvdata(dev));
	enum st_lsm6dso_sensor_id id = sensor->id;
	int i, len = 0;

	for (i = 0; i < st_lsm6dso_fs_table[id].size; i++)
		len += scnprintf(buf + len, PAGE_SIZE - len, "0.%06u ",
				 st_lsm6dso_fs_table[id].fs_avl[i].gain);
	buf[len - 1] = '\n';

	return len;
}

static ssize_t
st_lsm6dso_sysfs_reset_step_counter(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t size)
{
	struct iio_dev *iio_dev = dev_get_drvdata(dev);
	int err;

	err = st_lsm6dso_reset_step_counter(iio_dev);

	return err < 0 ? err : size;
}
static IIO_DEV_ATTR_SAMP_FREQ_AVAIL(st_lsm6dso_sysfs_sampling_frequency_avail);
static IIO_DEVICE_ATTR(in_accel_scale_available, 0444,
		       st_lsm6dso_sysfs_scale_avail, NULL, 0);
static IIO_DEVICE_ATTR(in_anglvel_scale_available, 0444,
		       st_lsm6dso_sysfs_scale_avail, NULL, 0);
static IIO_DEVICE_ATTR(hwfifo_watermark_max, 0444,
		       st_lsm6dso_get_max_watermark, NULL, 0);
static IIO_DEVICE_ATTR(hwfifo_flush, 0200, NULL, st_lsm6dso_flush_fifo, 0);
static IIO_DEVICE_ATTR(hwfifo_watermark, 0644, st_lsm6dso_get_watermark,
		       st_lsm6dso_set_watermark, 0);
static IIO_DEVICE_ATTR(reset_counter, 0200, NULL,
		       st_lsm6dso_sysfs_reset_step_counter, 0);

static struct attribute *st_lsm6dso_acc_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_in_accel_scale_available.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark_max.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark.dev_attr.attr,
	&iio_dev_attr_hwfifo_flush.dev_attr.attr,
	NULL,
};

static const struct attribute_group st_lsm6dso_acc_attribute_group = {
	.attrs = st_lsm6dso_acc_attributes,
};

static const struct iio_info st_lsm6dso_acc_info = {
	.driver_module = THIS_MODULE,
	.attrs = &st_lsm6dso_acc_attribute_group,
	.read_raw = st_lsm6dso_read_raw,
	.write_raw = st_lsm6dso_write_raw,
	.debugfs_reg_access = st_lsm6dso_debugfs_reg_access,
};

static struct attribute *st_lsm6dso_gyro_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_in_anglvel_scale_available.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark_max.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark.dev_attr.attr,
	&iio_dev_attr_hwfifo_flush.dev_attr.attr,
	NULL,
};

static const struct attribute_group st_lsm6dso_gyro_attribute_group = {
	.attrs = st_lsm6dso_gyro_attributes,
};

static const struct iio_info st_lsm6dso_gyro_info = {
	.driver_module = THIS_MODULE,
	.attrs = &st_lsm6dso_gyro_attribute_group,
	.read_raw = st_lsm6dso_read_raw,
	.write_raw = st_lsm6dso_write_raw,
	.debugfs_reg_access = st_lsm6dso_debugfs_reg_access,
};

static struct attribute *st_lsm6dso_step_counter_attributes[] = {
	&iio_dev_attr_hwfifo_watermark_max.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark.dev_attr.attr,
	&iio_dev_attr_reset_counter.dev_attr.attr,
	&iio_dev_attr_hwfifo_flush.dev_attr.attr,
	NULL,
};

static const struct attribute_group st_lsm6dso_step_counter_attribute_group = {
	.attrs = st_lsm6dso_step_counter_attributes,
};

static const struct iio_info st_lsm6dso_step_counter_info = {
	.driver_module = THIS_MODULE,
	.attrs = &st_lsm6dso_step_counter_attribute_group,
};

static struct attribute *st_lsm6dso_step_detector_attributes[] = {
	NULL,
};

static const struct attribute_group st_lsm6dso_step_detector_attribute_group = {
	.attrs = st_lsm6dso_step_detector_attributes,
};

static const struct iio_info st_lsm6dso_step_detector_info = {
	.driver_module = THIS_MODULE,
	.attrs = &st_lsm6dso_step_detector_attribute_group,
	.read_event_config = st_lsm6dso_read_event_config,
	.write_event_config = st_lsm6dso_write_event_config,
};

static struct attribute *st_lsm6dso_sign_motion_attributes[] = {
	NULL,
};

static const struct attribute_group st_lsm6dso_sign_motion_attribute_group = {
	.attrs = st_lsm6dso_sign_motion_attributes,
};

static const struct iio_info st_lsm6dso_sign_motion_info = {
	.driver_module = THIS_MODULE,
	.attrs = &st_lsm6dso_sign_motion_attribute_group,
	.read_event_config = st_lsm6dso_read_event_config,
	.write_event_config = st_lsm6dso_write_event_config,
};

static struct attribute *st_lsm6dso_tilt_attributes[] = {
	NULL,
};

static const struct attribute_group st_lsm6dso_tilt_attribute_group = {
	.attrs = st_lsm6dso_tilt_attributes,
};

static const struct iio_info st_lsm6dso_tilt_info = {
	.driver_module = THIS_MODULE,
	.attrs = &st_lsm6dso_tilt_attribute_group,
	.read_event_config = st_lsm6dso_read_event_config,
	.write_event_config = st_lsm6dso_write_event_config,
};

static struct attribute *st_lsm6dso_glance_attributes[] = {
	NULL,
};

static const struct attribute_group st_lsm6dso_glance_attribute_group = {
	.attrs = st_lsm6dso_glance_attributes,
};

static const struct iio_info st_lsm6dso_glance_info = {
	.driver_module = THIS_MODULE,
	.attrs = &st_lsm6dso_glance_attribute_group,
	.read_event_config = st_lsm6dso_read_event_config,
	.write_event_config = st_lsm6dso_write_event_config,
};

static struct attribute *st_lsm6dso_motion_attributes[] = {
	NULL,
};

static const struct attribute_group st_lsm6dso_motion_attribute_group = {
	.attrs = st_lsm6dso_motion_attributes,
};

static const struct iio_info st_lsm6dso_motion_info = {
	.driver_module = THIS_MODULE,
	.attrs = &st_lsm6dso_motion_attribute_group,
	.read_event_config = st_lsm6dso_read_event_config,
	.write_event_config = st_lsm6dso_write_event_config,
};

static struct attribute *st_lsm6dso_no_motion_attributes[] = {
	NULL,
};

static const struct attribute_group st_lsm6dso_no_motion_attribute_group = {
	.attrs = st_lsm6dso_no_motion_attributes,
};

static const struct iio_info st_lsm6dso_no_motion_info = {
	.driver_module = THIS_MODULE,
	.attrs = &st_lsm6dso_no_motion_attribute_group,
	.read_event_config = st_lsm6dso_read_event_config,
	.write_event_config = st_lsm6dso_write_event_config,
};

static struct attribute *st_lsm6dso_wakeup_attributes[] = {
	NULL,
};

static const struct attribute_group st_lsm6dso_wakeup_attribute_group = {
	.attrs = st_lsm6dso_wakeup_attributes,
};

static const struct iio_info st_lsm6dso_wakeup_info = {
	.driver_module = THIS_MODULE,
	.attrs = &st_lsm6dso_wakeup_attribute_group,
	.read_event_config = st_lsm6dso_read_event_config,
	.write_event_config = st_lsm6dso_write_event_config,
};

static struct attribute *st_lsm6dso_pickup_attributes[] = {
	NULL,
};

static const struct attribute_group st_lsm6dso_pickup_attribute_group = {
	.attrs = st_lsm6dso_pickup_attributes,
};

static const struct iio_info st_lsm6dso_pickup_info = {
	.driver_module = THIS_MODULE,
	.attrs = &st_lsm6dso_pickup_attribute_group,
	.read_event_config = st_lsm6dso_read_event_config,
	.write_event_config = st_lsm6dso_write_event_config,
};

static struct attribute *st_lsm6dso_orientation_attributes[] = {
	NULL,
};

static const struct attribute_group st_lsm6dso_orientation_attribute_group = {
	.attrs = st_lsm6dso_orientation_attributes,
};

static const struct iio_info st_lsm6dso_orientation_info = {
	.driver_module = THIS_MODULE,
	.attrs = &st_lsm6dso_orientation_attribute_group,
};

static struct attribute *st_lsm6dso_wrist_attributes[] = {
	NULL,
};

static const struct attribute_group st_lsm6dso_wrist_attribute_group = {
	.attrs = st_lsm6dso_wrist_attributes,
};

static const struct iio_info st_lsm6dso_wrist_info = {
	.driver_module = THIS_MODULE,
	.attrs = &st_lsm6dso_wrist_attribute_group,
	.read_event_config = st_lsm6dso_read_event_config,
	.write_event_config = st_lsm6dso_write_event_config,
};

static const unsigned long st_lsm6dso_available_scan_masks[] = { 0x7, 0x0 };
static const unsigned long st_lsm6dso_sc_available_scan_masks[] = { 0x1, 0x0 };

static int st_lsm6dso_of_get_pin(struct st_lsm6dso_hw *hw, int *pin)
{
	struct device_node *np = hw->dev->of_node;

	if (!np)
		return -EINVAL;

	return of_property_read_u32(np, "st,int-pin", pin);
}

static int st_lsm6dso_get_int_reg(struct st_lsm6dso_hw *hw, u8 *drdy_reg,
				  u8 *ef_irq_reg)
{
	int err = 0, int_pin;

	if (st_lsm6dso_of_get_pin(hw, &int_pin) < 0) {
		struct st_sensors_platform_data *pdata;
		struct device *dev = hw->dev;

		pdata = (struct st_sensors_platform_data *)dev->platform_data;
		int_pin = pdata ? pdata->drdy_int_pin : 1;
	}

	switch (int_pin) {
	case 1:
		hw->embfunc_irq_reg = ST_LSM6DSO_REG_EMB_FUNC_INT1_ADDR;
		*ef_irq_reg = ST_LSM6DSO_REG_MD1_CFG_ADDR;
		*drdy_reg = ST_LSM6DSO_REG_INT1_CTRL_ADDR;
		break;
	case 2:
		hw->embfunc_irq_reg = ST_LSM6DSO_REG_EMB_FUNC_INT2_ADDR;
		*ef_irq_reg = ST_LSM6DSO_REG_MD2_CFG_ADDR;
		*drdy_reg = ST_LSM6DSO_REG_INT2_CTRL_ADDR;
		break;
	default:
		dev_err(hw->dev, "unsupported interrupt pin\n");
		err = -EINVAL;
		break;
	}

	return err;
}

static int st_lsm6dso_reset_device(struct st_lsm6dso_hw *hw)
{
	int err;

	/* sw reset */
	err = st_lsm6dso_write_with_mask(hw, ST_LSM6DSO_REG_CTRL3_C_ADDR,
					 ST_LSM6DSO_REG_SW_RESET_MASK, 1);
	if (err < 0)
		return err;

	msleep(50);

	/* boot */
	err = st_lsm6dso_write_with_mask(hw, ST_LSM6DSO_REG_CTRL3_C_ADDR,
					 ST_LSM6DSO_REG_BOOT_MASK, 1);

	msleep(50);

	return err;
}

static int st_lsm6dso_init_timestamp_engine(struct st_lsm6dso_hw *hw,
					    bool enable)
{
	int err;

	/* Init timestamp engine. */
	err = st_lsm6dso_write_with_mask(hw, ST_LSM6DSO_REG_CTRL10_C_ADDR,
					 ST_LSM6DSO_REG_TIMESTAMP_EN_MASK, enable);
	if (err < 0)
		return err;

	/* Enable timestamp rollover interrupt on INT2. */
	err = st_lsm6dso_write_with_mask(hw, ST_LSM6DSO_REG_MD2_CFG_ADDR,
					 ST_LSM6DSO_REG_INT2_TIMESTAMP_MASK, enable);

	return err;
}

static int st_lsm6dso_parse_dt(struct st_lsm6dso_hw *hw)
{
	int err;
	const char *calib_data;
	int accel_offset_weight = 0;
	int accel_x_axis_offset = 0;
	int accel_y_axis_offset = 0;
	int accel_z_axis_offset = 0;
	struct device_node *node = hw->dev->of_node;

	hw->vdd = devm_regulator_get(hw->dev, "vdd");
	if (IS_ERR(hw->vdd)) {
		err = PTR_ERR(hw->vdd);
		dev_warn(hw->dev, "regulator get vdd failed (%d)\n", err);
	}

	/* Read calibration data */
	if (of_property_read_string(node, "acc_cal", &calib_data)) {
		dev_info(hw->dev, "calibaration data not found in DT\n");
		hw->accel_offset_weight = ST_LSM6DSO_ACCEL_OFFSET_WEIGHT_DEFAULT;
		hw->accel_x_axis_offset = ST_LSM6DSO_ACCEL_X_OFFSET_DEFAULT;
		hw->accel_y_axis_offset = ST_LSM6DSO_ACCEL_Y_OFFSET_DEFAULT;
		hw->accel_z_axis_offset = ST_LSM6DSO_ACCEL_Z_OFFSET_DEFAULT;
		return 0;
	}

	// 'acc_cal' is of the format ...
	// <accel_offset_weight> <accel_x_axis_offset> <accel_y_axis_offset> <accel_z_axis_offset>
	// example: 0 0xa 0x18 0xc9
	// accel_offset_weight: either 0 or 1
	//     Weight of XL user offset bits of registers X_OFS_USR (73h), Y_OFS_USR (74h),
	//     Z_OFS_USR (75h)
	//     0: 2-10 g/LSB
	//     1: 2-6 g/LSB
	// accel_x_axis_offset: Accelerometer X-axis user offset correction. The offset value
	//     set in the X_OFS_USR offset register is internally subtracted from the acceleration
	//     value measured on the X-axis.
	// accel_y_axis_offset: Accelerometer Y-axis user offset correction. The offset value
	//     set in the Y_OFS_USR offset register is internally subtracted from the acceleration
	//     value measured on the Y-axis.
	// accel_z_axis_offset: Accelerometer Z-axis user offset correction. The offset value
	//     set in the Z_OFS_USR offset register is internally subtracted from the acceleration
	//     value measured on the Z-axis.
	if (sscanf(calib_data, "%d %x %x %x",
			&accel_offset_weight,
			&accel_x_axis_offset,
			&accel_y_axis_offset,
			&accel_z_axis_offset) != 4) {
		dev_err(hw->dev, "acc_cal calibaration data has bad format in DT\n");
		return -EINVAL;
	}

	if (accel_offset_weight < 0 || accel_offset_weight > 1) {
		dev_err(hw->dev, "acc_cal accel_offset_weight %d is not 0 or 1\n", accel_offset_weight);
		return -EINVAL;
	}
	else if (accel_x_axis_offset < 0 || accel_x_axis_offset > 255) {
		dev_err(hw->dev, "acc_cal accel_x_axis_offset 0x%x is not between 0 and 0xff\n",
			accel_x_axis_offset);
		return -EINVAL;
	}
	else if (accel_y_axis_offset < 0 || accel_y_axis_offset > 255) {
		dev_err(hw->dev, "acc_cal accel_y_axis_offset 0x%x is not between 0 and 0xff\n",
			accel_y_axis_offset);
		return -EINVAL;
	}
	else if (accel_z_axis_offset < 0 || accel_z_axis_offset > 255) {
		dev_err(hw->dev, "acc_cal accel_z_axis_offset 0x%x is not between 0 and 0xff\n",
			accel_z_axis_offset);
		return -EINVAL;
	}

	hw->accel_offset_weight = accel_offset_weight;
	hw->accel_x_axis_offset = accel_x_axis_offset;
	hw->accel_y_axis_offset = accel_y_axis_offset;
	hw->accel_z_axis_offset = accel_z_axis_offset;

	return 0;
}

static int st_lsm6dso_set_calibration(struct st_lsm6dso_hw *hw)
{
	int err;
	if (hw->accel_offset_weight != ST_LSM6DSO_ACCEL_OFFSET_WEIGHT_DEFAULT) {
		err = st_lsm6dso_write_with_mask(hw, ST_LSM6DSO_REG_CTRL6_C_ADDR,
					ST_LSM6DSO_REG_USR_OFF_W_MASK, hw->accel_offset_weight);
		if (err < 0) {
			dev_err(hw->dev, "init accel_offset_weight failed with error %d\n", err);
			return err;
		}
		dev_info(hw->dev, "init accel_offset_weight with value %d\n",
			hw->accel_offset_weight);
	}

	if (hw->accel_x_axis_offset != ST_LSM6DSO_ACCEL_X_OFFSET_DEFAULT) {
		err = st_lsm6dso_write_with_mask(hw, ST_LSM6DSO_REG_X_OFS_USR,
					ST_LSM6DSO_REG_X_OFS_USR_MASK, hw->accel_x_axis_offset);
		if (err < 0) {
			dev_err(hw->dev, "init accel_x_axis_offset failed with error %d\n", err);
			return err;
		}
		dev_info(hw->dev, "init accel_x_axis_offset with value 0x%x\n",
			hw->accel_x_axis_offset & 0xff);
	}

	if (hw->accel_y_axis_offset != ST_LSM6DSO_ACCEL_Y_OFFSET_DEFAULT) {
		err = st_lsm6dso_write_with_mask(hw, ST_LSM6DSO_REG_Y_OFS_USR,
						ST_LSM6DSO_REG_Y_OFS_USR_MASK, hw->accel_y_axis_offset);
		if (err < 0) {
			dev_err(hw->dev, "init accel_y_axis_offset failed with error %d\n", err);
			return err;
		}
		dev_info(hw->dev, "init accel_y_axis_offset with value 0x%x\n",
			hw->accel_y_axis_offset & 0xff);
	}

	if (hw->accel_z_axis_offset != ST_LSM6DSO_ACCEL_Z_OFFSET_DEFAULT) {
		err = st_lsm6dso_write_with_mask(hw, ST_LSM6DSO_REG_Z_OFS_USR,
					ST_LSM6DSO_REG_Z_OFS_USR_MASK, hw->accel_z_axis_offset);
		if (err < 0) {
			dev_err(hw->dev, "init accel_z_axis_offset failed with error %d\n", err);
			return err;
		}
		dev_info(hw->dev, "init accel_z_axis_offset with value 0x%x\n",
			hw->accel_z_axis_offset & 0xff);
	}

	return 0;
}

static int st_lsm6dso_power_on(struct st_lsm6dso_hw *hw)
{
	int err;

	if (!IS_ERR(hw->vdd)) {
		err = regulator_enable(hw->vdd);
		if (err) {
			dev_err(hw->dev, "regulator enable vdd failed (%d)", err);
			return err;
		}
		mdelay(35); // 35ms turn-on-time according to datasheet
	}

	return 0;
}


static void st_lsm6dso_power_off(struct st_lsm6dso_hw *hw)
{
	if (!IS_ERR(hw->vdd))
		regulator_disable(hw->vdd);
}

static int st_lsm6dso_init_device(struct st_lsm6dso_hw *hw)
{
	u8 drdy_reg, ef_irq_reg;
	int err;

	err = st_lsm6dso_set_calibration(hw);
	if (err < 0)
		return err;

	/* latch interrupts */
	err = st_lsm6dso_write_with_mask(hw, ST_LSM6DSO_REG_TAP_CFG0_ADDR,
					 ST_LSM6DSO_REG_LIR_MASK, 1);
	if (err < 0)
		return err;

	/* enable Block Data Update */
	err = st_lsm6dso_write_with_mask(hw, ST_LSM6DSO_REG_CTRL3_C_ADDR,
					 ST_LSM6DSO_REG_BDU_MASK, 1);
	if (err < 0)
		return err;

	err = st_lsm6dso_write_with_mask(hw, ST_LSM6DSO_REG_CTRL5_C_ADDR,
					 ST_LSM6DSO_REG_ROUNDING_MASK, 3);
	if (err < 0)
		return err;

	err = st_lsm6dso_init_timestamp_engine(hw, true);
	if (err < 0)
		return err;

	err = st_lsm6dso_get_int_reg(hw, &drdy_reg, &ef_irq_reg);
	if (err < 0)
		return err;

	/* enable FIFO watermak interrupt */
	err = st_lsm6dso_write_with_mask(hw, drdy_reg,
					 ST_LSM6DSO_REG_FIFO_TH_MASK, 1);
	if (err < 0)
		return err;

	/* enable enbedded function interrupts */
	err = st_lsm6dso_write_with_mask(hw, ef_irq_reg,
					 ST_LSM6DSO_REG_INT_EMB_FUNC_MASK, 1);
	if (err < 0)
		return err;

	/* init finite state machine */
	return st_lsm6dso_fsm_init(hw);
}

static struct iio_dev *st_lsm6dso_alloc_iiodev(struct st_lsm6dso_hw *hw,
					       enum st_lsm6dso_sensor_id id)
{
	struct st_lsm6dso_sensor *sensor;
	struct iio_dev *iio_dev;

	iio_dev = devm_iio_device_alloc(hw->dev, sizeof(*sensor));
	if (!iio_dev)
		return NULL;

	iio_dev->modes = INDIO_DIRECT_MODE;
	iio_dev->dev.parent = hw->dev;

	sensor = iio_priv(iio_dev);
	sensor->id = id;
	sensor->hw = hw;
	sensor->watermark = 1;

	switch (id) {
	case ST_LSM6DSO_ID_ACC:
		iio_dev->channels = st_lsm6dso_acc_channels;
		iio_dev->num_channels = ARRAY_SIZE(st_lsm6dso_acc_channels);
		iio_dev->name = "lsm6dso_accel";
		iio_dev->info = &st_lsm6dso_acc_info;
		iio_dev->available_scan_masks = st_lsm6dso_available_scan_masks;

		sensor->batch_reg.addr = ST_LSM6DSO_REG_FIFO_CTRL3_ADDR;
		sensor->batch_reg.mask = ST_LSM6DSO_REG_BDR_XL_MASK;
		sensor->max_watermark = ST_LSM6DSO_MAX_FIFO_DEPTH;
		sensor->odr = st_lsm6dso_odr_table[id].odr_avl[1].hz;
		sensor->gain = st_lsm6dso_fs_table[id].fs_avl[0].gain;
		break;
	case ST_LSM6DSO_ID_GYRO:
		iio_dev->channels = st_lsm6dso_gyro_channels;
		iio_dev->num_channels = ARRAY_SIZE(st_lsm6dso_gyro_channels);
		iio_dev->name = "lsm6dso_gyro";
		iio_dev->info = &st_lsm6dso_gyro_info;
		iio_dev->available_scan_masks = st_lsm6dso_available_scan_masks;

		sensor->batch_reg.addr = ST_LSM6DSO_REG_FIFO_CTRL3_ADDR;
		sensor->batch_reg.mask = ST_LSM6DSO_REG_BDR_GY_MASK;
		sensor->max_watermark = ST_LSM6DSO_MAX_FIFO_DEPTH;
		sensor->odr = st_lsm6dso_odr_table[id].odr_avl[1].hz;
		sensor->gain = st_lsm6dso_fs_table[id].fs_avl[0].gain;
		break;
	case ST_LSM6DSO_ID_STEP_COUNTER:
		iio_dev->channels = st_lsm6dso_step_counter_channels;
		iio_dev->num_channels =
			ARRAY_SIZE(st_lsm6dso_step_counter_channels);
		iio_dev->name = "lsm6dso_step_c";
		iio_dev->info = &st_lsm6dso_step_counter_info;
		iio_dev->available_scan_masks = st_lsm6dso_sc_available_scan_masks;

		sensor->max_watermark = 1;
		sensor->odr =
			st_lsm6dso_odr_table[ST_LSM6DSO_ID_ACC].odr_avl[2].hz;
		break;
	case ST_LSM6DSO_ID_STEP_DETECTOR:
		iio_dev->channels = st_lsm6dso_step_detector_channels;
		iio_dev->num_channels =
			ARRAY_SIZE(st_lsm6dso_step_detector_channels);
		iio_dev->name = "lsm6dso_step_d";
		iio_dev->info = &st_lsm6dso_step_detector_info;
		iio_dev->available_scan_masks = st_lsm6dso_sc_available_scan_masks;

		sensor->odr =
			st_lsm6dso_odr_table[ST_LSM6DSO_ID_ACC].odr_avl[2].hz;
		break;
	case ST_LSM6DSO_ID_SIGN_MOTION:
		iio_dev->channels = st_lsm6dso_sign_motion_channels;
		iio_dev->num_channels =
			ARRAY_SIZE(st_lsm6dso_sign_motion_channels);
		iio_dev->name = "lsm6dso_sign_motion";
		iio_dev->info = &st_lsm6dso_sign_motion_info;
		iio_dev->available_scan_masks = st_lsm6dso_sc_available_scan_masks;

		sensor->odr =
			st_lsm6dso_odr_table[ST_LSM6DSO_ID_ACC].odr_avl[2].hz;
		break;
	case ST_LSM6DSO_ID_TILT:
		iio_dev->channels = st_lsm6dso_tilt_channels;
		iio_dev->num_channels = ARRAY_SIZE(st_lsm6dso_tilt_channels);
		iio_dev->name = "lsm6dso_tilt";
		iio_dev->info = &st_lsm6dso_tilt_info;
		iio_dev->available_scan_masks = st_lsm6dso_sc_available_scan_masks;

		sensor->odr =
			st_lsm6dso_odr_table[ST_LSM6DSO_ID_ACC].odr_avl[2].hz;
		break;
	case ST_LSM6DSO_ID_GLANCE:
		iio_dev->channels = st_lsm6dso_glance_channels;
		iio_dev->num_channels = ARRAY_SIZE(st_lsm6dso_glance_channels);
		iio_dev->name = "lsm6dso_glance";
		iio_dev->info = &st_lsm6dso_glance_info;
		iio_dev->available_scan_masks = st_lsm6dso_sc_available_scan_masks;

		sensor->odr =
			st_lsm6dso_odr_table[ST_LSM6DSO_ID_ACC].odr_avl[2].hz;
		break;
	case ST_LSM6DSO_ID_MOTION:
		iio_dev->channels = st_lsm6dso_motion_channels;
		iio_dev->num_channels = ARRAY_SIZE(st_lsm6dso_motion_channels);
		iio_dev->name = "lsm6dso_motion";
		iio_dev->info = &st_lsm6dso_motion_info;
		iio_dev->available_scan_masks = st_lsm6dso_sc_available_scan_masks;

		sensor->odr =
			st_lsm6dso_odr_table[ST_LSM6DSO_ID_ACC].odr_avl[2].hz;
		break;
	case ST_LSM6DSO_ID_NO_MOTION:
		iio_dev->channels = st_lsm6dso_no_motion_channels;
		iio_dev->num_channels = ARRAY_SIZE(st_lsm6dso_no_motion_channels);
		iio_dev->name = "lsm6dso_no_motion";
		iio_dev->info = &st_lsm6dso_no_motion_info;
		iio_dev->available_scan_masks = st_lsm6dso_sc_available_scan_masks;

		sensor->odr =
			st_lsm6dso_odr_table[ST_LSM6DSO_ID_ACC].odr_avl[2].hz;
		break;
	case ST_LSM6DSO_ID_WAKEUP:
		iio_dev->channels = st_lsm6dso_wakeup_channels;
		iio_dev->num_channels = ARRAY_SIZE(st_lsm6dso_wakeup_channels);
		iio_dev->name = "lsm6dso_wk";
		iio_dev->info = &st_lsm6dso_wakeup_info;
		iio_dev->available_scan_masks = st_lsm6dso_sc_available_scan_masks;

		sensor->odr =
			st_lsm6dso_odr_table[ST_LSM6DSO_ID_ACC].odr_avl[2].hz;
		break;
	case ST_LSM6DSO_ID_PICKUP:
		iio_dev->channels = st_lsm6dso_pickup_channels;
		iio_dev->num_channels = ARRAY_SIZE(st_lsm6dso_pickup_channels);
		iio_dev->name = "lsm6dso_pickup";
		iio_dev->info = &st_lsm6dso_pickup_info;
		iio_dev->available_scan_masks = st_lsm6dso_sc_available_scan_masks;

		sensor->odr =
			st_lsm6dso_odr_table[ST_LSM6DSO_ID_ACC].odr_avl[2].hz;
		break;
	case ST_LSM6DSO_ID_ORIENTATION:
		iio_dev->channels = st_lsm6dso_orientation_channels;
		iio_dev->num_channels = ARRAY_SIZE(st_lsm6dso_orientation_channels);
		iio_dev->name = "lsm6dso_dev_orientation";
		iio_dev->info = &st_lsm6dso_orientation_info;
		iio_dev->available_scan_masks = st_lsm6dso_sc_available_scan_masks;

		sensor->odr =
			st_lsm6dso_odr_table[ST_LSM6DSO_ID_ACC].odr_avl[2].hz;
		break;
	case ST_LSM6DSO_ID_WRIST_TILT:
		iio_dev->channels = st_lsm6dso_wrist_channels;
		iio_dev->num_channels = ARRAY_SIZE(st_lsm6dso_wrist_channels);
		iio_dev->name = "lsm6dso_wrist";
		iio_dev->info = &st_lsm6dso_wrist_info;
		iio_dev->available_scan_masks = st_lsm6dso_sc_available_scan_masks;

		sensor->odr =
			st_lsm6dso_odr_table[ST_LSM6DSO_ID_ACC].odr_avl[2].hz;
		break;
	default:
		return NULL;
	}

	return iio_dev;
}

int st_lsm6dso_probe(struct device *dev, int irq,
		     const struct st_lsm6dso_transfer_function *tf_ops)
{
	struct st_lsm6dso_hw *hw;
	int i, err;

	hw = devm_kzalloc(dev, sizeof(*hw), GFP_KERNEL);
	if (!hw)
		return -ENOMEM;

	dev_set_drvdata(dev, (void *)hw);

	mutex_init(&hw->lock);
	mutex_init(&hw->fifo_lock);
	mutex_init(&hw->page_lock);

	hw->dev = dev;
	hw->irq = irq;
	hw->tf = tf_ops;

	/* parse and init from device-tree */
	err = st_lsm6dso_parse_dt(hw);
	if (err < 0)
		return err;

	/* power on the device */
	err = st_lsm6dso_power_on(hw);
	if (err)
		return err;

	err = st_lsm6dso_check_whoami(hw);
	if (err < 0)
		return err;

	err = st_lsm6dso_get_odr_calibration(hw);
	if (err < 0)
		return err;

	err = st_lsm6dso_reset_device(hw);
	if (err < 0)
		return err;

	err = st_lsm6dso_init_device(hw);
	if (err < 0)
		return err;

	for (i = 0; i < ARRAY_SIZE(st_lsm6dso_main_sensor_list); i++) {
		enum st_lsm6dso_sensor_id id = st_lsm6dso_main_sensor_list[i];

		hw->iio_devs[id] = st_lsm6dso_alloc_iiodev(hw, id);
		if (!hw->iio_devs[id])
			return -ENOMEM;
	}

	err = st_lsm6dso_shub_probe(hw);
	if (err < 0)
		return err;

	if (hw->irq > 0) {
		err = st_lsm6dso_buffers_setup(hw);
		if (err < 0)
			return err;
	}

	for (i = 0; i < ST_LSM6DSO_ID_MAX; i++) {
		if (!hw->iio_devs[i])
			continue;

		err = devm_iio_device_register(hw->dev, hw->iio_devs[i]);
		if (err)
			return err;
	}

#if defined(CONFIG_PM) && defined(CONFIG_IIO_ST_LSM6DSO_MAY_WAKEUP)
	err = device_init_wakeup(dev, 1);
	if (err)
		return err;
#endif /* CONFIG_PM && CONFIG_IIO_ST_LSM6DSO_MAY_WAKEUP */

#ifdef TIMESTAMP_HW
	hw->delta_hw_ts = 0ull;
#endif /* TIMESTAMP_HW */

	dev_info(dev, "Device probed\n");

	return 0;
}
EXPORT_SYMBOL(st_lsm6dso_probe);

int st_lsm6dso_remove(struct device *dev)
{
	struct st_lsm6dso_hw *hw = dev_get_drvdata(dev);
	st_lsm6dso_power_off(hw);
	return st_lsm6dso_deallocate_buffers(hw);
 }
EXPORT_SYMBOL(st_lsm6dso_remove);

static int __maybe_unused st_lsm6dso_suspend(struct device *dev)
{
	struct st_lsm6dso_hw *hw = dev_get_drvdata(dev);
	struct st_lsm6dso_sensor *sensor;
	int i, err = 0;

	for (i = 0; i < ST_LSM6DSO_ID_MAX; i++) {
		sensor = iio_priv(hw->iio_devs[i]);
		if (!hw->iio_devs[i])
			continue;

		if (!(hw->enable_mask & BIT(sensor->id)))
			continue;

		err = st_lsm6dso_set_odr(sensor, 0);
		if (err < 0)
			return err;
	}

	if (st_lsm6dso_is_fifo_enabled(hw))
		err = st_lsm6dso_suspend_fifo(hw);
#ifdef CONFIG_IIO_ST_LSM6DSO_MAY_WAKEUP
	if (device_may_wakeup(dev))
		enable_irq_wake(hw->irq);
#endif /* CONFIG_IIO_ST_LSM6DSO_MAY_WAKEUP */
	dev_info(dev, "Suspending device\n");

	return err;
}

static int __maybe_unused st_lsm6dso_resume(struct device *dev)
{
	struct st_lsm6dso_hw *hw = dev_get_drvdata(dev);
	struct st_lsm6dso_sensor *sensor;
	int i, err = 0;

	dev_info(dev, "Resuming device\n");
#ifdef CONFIG_IIO_ST_LSM6DSO_MAY_WAKEUP
	if (device_may_wakeup(dev))
		disable_irq_wake(hw->irq);
#endif /* CONFIG_IIO_ST_LSM6DSO_MAY_WAKEUP */

	for (i = 0; i < ST_LSM6DSO_ID_MAX; i++) {
		sensor = iio_priv(hw->iio_devs[i]);
		if (!hw->iio_devs[i])
			continue;

		if (!(hw->enable_mask & BIT(sensor->id)))
			continue;

		err = st_lsm6dso_set_odr(sensor, sensor->odr);
		if (err < 0)
			return err;
	}

	if (st_lsm6dso_is_fifo_enabled(hw))
		err = st_lsm6dso_set_fifo_mode(hw, ST_LSM6DSO_FIFO_CONT);

	return err;
}

const struct dev_pm_ops st_lsm6dso_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(st_lsm6dso_suspend, st_lsm6dso_resume)
};
EXPORT_SYMBOL(st_lsm6dso_pm_ops);

MODULE_AUTHOR("Lorenzo Bianconi <lorenzo.bianconi@st.com>");
MODULE_DESCRIPTION("STMicroelectronics st_lsm6dso driver");
MODULE_LICENSE("GPL v2");
