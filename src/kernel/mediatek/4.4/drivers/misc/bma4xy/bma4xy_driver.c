/*
 * @section LICENSE
 * (C) Copyright 2019-2020 Bosch Sensortec GmbH All Rights Reserved
 *
 * This software program is licensed subject to the GNU General
 * Public License (GPL).Version 2,June 1991,
 * available at http://www.fsf.org/copyleft/gpl.html
 *
 * @filename bma4xy_driver.c
 * @date     2020/01/13
 * @version  0.2.8
 *
 * @brief    BMA4xy Linux Driver
 */

/*********************************************************************/
/* system header files */
/*********************************************************************/
#include <linux/types.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>

/*********************************************************************/
/* Own header files */
/*********************************************************************/
#include "bma4xy_driver.h"

/*********************************************************************/
/* Local macro definitions */
/*********************************************************************/
#define DRIVER_VERSION "0.0.2.7"
#define MILLIS_TO_MICROS(milli_sec) ((milli_sec) * (1000))

/*********************************************************************/
/* Constant definitions */
/*********************************************************************/
/*!
 * @brief Enumerations for the sensor power modes
 */
enum bma4xy_power_mode {
	BMA4XY_NORMAL_MODE = 0,
	BMA4XY_SUSPEND_MODE = 2,
};

/*!
 * @brief Enumerations to select the sensor features
 */
enum bma4xy_config_fun {
	BMA4XY_SINGLE_TAP_SENSOR = 1,
	BMA4XY_STEP_DETECTOR_SENSOR = 2,
	BMA4XY_STEP_COUNTER_SENSOR = 3,
	BMA4XY_WRIST_WEAR_WAKEUP = 4,
	BMA4XY_DOUBLE_TAP_SENSOR = 5,
	BMA4XY_ANY_MOTION_SENSOR = 6,
	BMA4XY_NO_MOTION_SENSOR = 7,
	BMA4XY_ACTIVITY_SENSOR = 8,
};

/*********************************************************************/
/* Static function declarations */
/*********************************************************************/
/*!
 * @brief Delay function for bma4xy I2C operations
 *
 * @param msec[in] : delay in milliseconds
 *
 * @return no return value
 */
static void bma4xy_i2c_delay(uint32_t msec);

/*!
 * @brief Function to check chip ID
 *
 * @param client_data[in,out] : instance of the client data
 * @param dev[in,out] : instance of the device
 *
 * @return the status
 * @retval 0 -> Success
 * @retval negative -> fail
 */
static int bma4xy_check_chip_id(struct bma4xy_client_data *client_data,
				struct device *dev);

/*!
 * @brief This function initializes BMA after config stream load
 *
 * @param client_data[in,out] : instance of the client data
 *
 * @return the status
 * @retval 0 -> Success
 * @retval negative -> fail
 */
static int bma4xy_init_after_config_stream_load(
				struct bma4xy_client_data *client_data);

/*!
 * @brief This function initializes the fifo configuration
 *
 * @param client_data[in,out] : instance of the client data
 *
 * @return the status
 * @retval 0 -> Success
 * @retval negative -> fail
 */
static int bma4xy_init_fifo_config(
				struct bma4xy_client_data *client_data);

/*!
 * @brief This function updates the configuration stream
 *
 * @param client_data[in,out] : instance of the client data
 * @param option[in] : The option to mention config stream.
 *                     For BMA456 this value is 3.
 * @return the status
 * @retval 0 -> Success
 * @retval negative -> fail
 */
static int bma4xy_update_config_stream(
				struct bma4xy_client_data *client_data,
				uint8_t option);

/*!
 * @brief This function configures the features to its previous state
 *        saved in client data. Typically used at reinit after error.
 *
 * @param client_data[in,out] : instance of the client data
 *
 * @return the status
 * @retval 0 -> Success
 * @retval negative -> fail
 */
static int bma456_config_feature(struct bma4xy_client_data *client_data);

/*!
 * @brief This function reinitializes accelerometer.
 *
 * @param client_data[in,out] : instance of the client data
 *
 * @return the status
 * @retval 0 -> Success
 * @retval negative -> fail
 */
static int bma4xy_reinit_acc(struct bma4xy_client_data *client_data);

/*!
 * @brief This function resets the chip to previous state up on error by
 *        configuring to the saved state.
 *
 * @param client_data[in,out] : instance of the client data
 *
 * @return the status
 * @retval 0 -> Success
 * @retval negative -> fail
 */
static int bma4xy_reinit_after_error_interrupt(
				struct bma4xy_client_data *client_data);

/*!
 * @brief This function initializes the accel input device.
 *
 * @param client_data[in,out] : instance of the client data
 *
 * @return the status
 * @retval 0 -> Success
 * @retval negative -> fail
 */
static int bma4xy_acc_input_init(struct bma4xy_client_data *client_data);

/*!
 * @brief This function cleans up the accel input device when done.
 *
 * @param client_data[in,out] : instance of the client data
 *
 * @return the status
 * @retval 0 -> Success
 * @retval negative -> fail
 */
static void bma4xy_acc_input_destroy(struct bma4xy_client_data *client_data);

/*!
 * @brief This function initializes the feature input device.
 *
 * @param client_data[in,out] : instance of the client data
 *
 * @return the status
 * @retval 0 -> Success
 * @retval negative -> fail
 */
static int bma4xy_feature_input_init(struct bma4xy_client_data *client_data);

/*!
 * @brief This function cleans up the feature input device when done.
 *
 * @param client_data[in,out] : instance of the client data
 *
 * @return the status
 * @retval 0 -> Success
 * @retval negative -> fail
 */
static void bma4xy_feature_input_destroy(
				struct bma4xy_client_data *client_data);

/**
 * @defgroup SYSFS_Functions The SYSFS functions.
 * @brief The show and store functions of SYSFS nodes.
 *
 * @param dev[in,out] : instance of the device
 * @param attr[in,out] : instance of the device attribute file
 * @param buf[in,out] : instance of the data buffer.
 *			This is input for store and output from show.
 * @param count[in] : The number of characters in the buffer `buf`.
 *		      Only used for store functions.
 *
 * @return In the case of store function this is the number of characters
 *         used from buffer `buf`, which equals count.
 *         In the case of show function this is the number of characters
 *         returned through the buffer `buf`.
 * @retval Positive -> Success. The number of characters in the buffer `buf`.
 * @retval negative -> Error
 * @{
 */

/*!
 * @brief Show function for `chip_id` node.
 */
static ssize_t chip_id_show(struct device *dev,
				struct device_attribute *attr, char *buf);

/*!
 * @brief Show function for `acc_op_mode` node.
 */
static ssize_t acc_op_mode_show(struct device *dev,
				struct device_attribute *attr,
				char *buf);

/*!
 * @brief Store function for `acc_op_mode` node.
 */
static ssize_t acc_op_mode_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count);

/*!
 * @brief Show function for `acc_value` node.
 */
static ssize_t acc_value_show(struct device *dev,
				struct device_attribute *attr, char *buf);

/*!
 * @brief Show function for `acc_range` node.
 */
static ssize_t acc_range_show(struct device *dev,
				struct device_attribute *attr, char *buf);

/*!
 * @brief Store function for accel range node.
 */
static ssize_t acc_range_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count);

/*!
 * @brief Show function for `acc_odr` node.
 */
static ssize_t acc_odr_show(struct device *dev,
				struct device_attribute *attr,
				char *buf);

/*!
 * @brief Store function for `acc_odr` node.
 */
static ssize_t acc_odr_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count);

/*!
 * @brief Show function for `selftest` node.
 */
static ssize_t selftest_show(struct device *dev,
				struct device_attribute *attr,
				char *buf);

/*!
 * @brief Store function for `selftest` node.
 */
static ssize_t selftest_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count);

/*!
 * @brief Show function for `foc` node.
 */
static ssize_t foc_show(struct device *dev,
			struct device_attribute *attr, char *buf);

/*!
 * @brief Store function for `foc` node.
 */
static ssize_t foc_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count);

/*!
 * @brief Show function for `config_function` node.
 */
static ssize_t config_function_show(struct device *dev,
					struct device_attribute *attr,
					char *buf);

/*!
 * @brief Store function for `config_function` node.
 */
static ssize_t config_function_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count);

/*!
 * @brief Store function for `axis_remapping` node.
 */
static ssize_t axis_remapping_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count);

/*!
 * @brief Show function for `fifo_length` node.
 */
static ssize_t fifo_length_show(struct device *dev,
				struct device_attribute *attr, char *buf);

/*!
 * @brief Store function for `fifo_flush` node.
 */
static ssize_t fifo_flush_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count);

/*!
 * @brief Show function for `acc_fifo_enable` node.
 */
static ssize_t acc_fifo_enable_show(struct device *dev,
					struct device_attribute *attr,
					char *buf);

/*!
 * @brief Store function for `acc_fifo_enable` node.
 */
static ssize_t acc_fifo_enable_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count);

/*!
 * @brief Show function for `load_config_stream` node.
 */
static ssize_t load_config_stream_show(struct device *dev,
					struct device_attribute *attr,
					char *buf);

/*!
 * @brief Store function for `load_config_stream` node.
 */
static ssize_t load_config_stream_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count);

/*!
 * @brief Show function for `fifo_watermark` node.
 */
static ssize_t fifo_watermark_show(struct device *dev,
					struct device_attribute *attr,
					char *buf);
/*!
 * @brief Store function for `fifo_watermark` node.
 */
static ssize_t fifo_watermark_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count);

/*!
 * @brief Show function for `fifo_data_out_frame` node.
 */
static ssize_t fifo_data_out_frame_show(struct device *dev,
					struct device_attribute *attr,
					char *buf);

/*!
 * @brief Show function for `reg_sel` node.
 */
static ssize_t reg_sel_show(struct device *dev,
				struct device_attribute *attr,
				char *buf);

/*!
 * @brief Store function for `reg_sel` node.
 */
static ssize_t reg_sel_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count);

/*!
 * @brief Show function for `reg_val` node.
 */
static ssize_t reg_val_show(struct device *dev,
				struct device_attribute *attr,
				char *buf);

/*!
 * @brief Store function for `reg_val` node.
 */
static ssize_t reg_val_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count);

/*!
 * @brief Show function for `driver_version` node.
 */
static ssize_t driver_version_show(struct device *dev,
					struct device_attribute *attr,
					char *buf);

/*!
 * @brief Show function for `config_file_version` node.
 */
static ssize_t config_file_version_show(struct device *dev,
					struct device_attribute *attr,
					char *buf);

/*!
 * @brief Show function for `avail_sensor` node.
 */
static ssize_t avail_sensor_show(struct device *dev,
					struct device_attribute *attr,
					char *buf);

/*!
 * @brief Show function for `step_counter_val` node.
 */
static ssize_t step_counter_val_show(struct device *dev,
					struct device_attribute *attr,
					char *buf);

/*!
 * @brief Show function for `step_counter_watermark` node.
 */
static ssize_t step_counter_watermark_show(struct device *dev,
						struct device_attribute *attr,
						char *buf);

/*!
 * @brief Store function for `step_counter_watermark` node.
 */
static ssize_t step_counter_watermark_store(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count);
/*!
 * @brief Show function for `step_counter_parameter` node.
 */
static ssize_t step_counter_parameter_show(struct device *dev,
						struct device_attribute *attr,
						char *buf);

/*!
 * @brief Store function for `step_counter_parameter` node.
 */
static ssize_t step_counter_parameter_store(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count);

/*!
 * @brief Store function for `step_counter_reset` node.
 */
static ssize_t step_counter_reset_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count);

/*!
 * @brief Show function for `anymotion_config` node.
 */
static ssize_t anymotion_config_show(struct device *dev,
					struct device_attribute *attr,
					char *buf);

/*!
 * @brief Store function for `anymotion_config` node.
 */
static ssize_t anymotion_config_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count);

/*!
 * @brief Store function for `nomotion_config` node.
 */
static ssize_t nomotion_config_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count);

/*!
 * @brief Show function for `nomotion_config` node.
 */
static ssize_t nomotion_config_show(struct device *dev,
					struct device_attribute *attr,
					char *buf);

/*!
 * @brief Show function for `err_int` node.
 */
static ssize_t err_int_show(struct device *dev,
				struct device_attribute *attr,
				char *buf);

/*!
 * @brief Store function for `err_int` node.
 */
static ssize_t err_int_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count);

/** @} */ /* @defgroup SYSFS_Functions */

/*********************************************************************/
/* functions */
/*********************************************************************/
/*!
 * @brief Delay function for bma4xy I2C operations.
 */
static void bma4xy_i2c_delay(uint32_t msec)
{
	unsigned long millisec = msec;
	unsigned long min_us = MILLIS_TO_MICROS(millisec);
	unsigned long max_us = MILLIS_TO_MICROS(millisec + 1);

	/* If the time is less than 20ms */
	if (msec <= 20)
		usleep_range(min_us, max_us);
	else
		msleep(msec);
}

/*!
 * @brief Function to check chip ID.
 */
static int bma4xy_check_chip_id(struct bma4xy_client_data *client_data,
				struct device *dev)
{
	uint16_t ret = 0;
	uint8_t chip_id = 0;

	ret = client_data->device.bus_read(client_data->device.bma4_i2c_client,
					BMA4_CHIP_ID_ADDR, &chip_id, 1);
	if (ret) {
		dev_err(dev, "chip id read failed\n");
		return -EIO;
	}

	dev_info(dev, "chip id result: %x\n", chip_id);

	return 0;
}

/*!
 * @brief This function initializes BMA after config stream load.
 */
static int bma4xy_init_after_config_stream_load(
				struct bma4xy_client_data *client_data)
{
	uint16_t ret = 0;
	uint8_t int_config;
	uint8_t latch_enable = BMA4_LATCH_MODE;
	/* value to map all interrupts in the interrupt register */
	uint8_t int1_map = 0xFF;
	struct bma4_dev *bma4_dev_ptr = &client_data->device;
	struct bma456_axes_remap axis_remap_data;

	ret = bma4_write_regs(BMA4_INT_MAP_1_ADDR, &int1_map, 1, bma4_dev_ptr);
	bma4xy_i2c_delay(10);

	int_config = ((BMA4_OUTPUT_ENABLE << BMA4_INT_OUTPUT_EN_POS)
			| (BMA4_ACTIVE_HIGH << BMA4_INT_LEVEL_POS));

	ret += bma4_write_regs(BMA4_INT1_IO_CTRL_ADDR,
			&int_config, 1, bma4_dev_ptr);
	bma4xy_i2c_delay(1);

	ret += bma4_write_regs(BMA4_INTR_LATCH_ADDR,
			&latch_enable, 1, bma4_dev_ptr);
	bma4xy_i2c_delay(1);
	if (ret)
		dev_err(client_data->dev, "mapping and enabling interrupt1 failed ret=%d\n", ret);

	memset(&axis_remap_data, 0, sizeof(axis_remap_data));
	axis_remap_data.x_axis = 0;
	axis_remap_data.x_axis_sign = 0;
	axis_remap_data.y_axis = 1;
	axis_remap_data.y_axis_sign = 1;
	axis_remap_data.z_axis = 2;
	axis_remap_data.z_axis_sign = 1;
	ret += bma456_set_remap_axes(&axis_remap_data, bma4_dev_ptr);

	if (ret) {
		dev_err(client_data->dev, "set bma456 step_count select platform error\n");
		dev_err(client_data->dev, "write axis_remap failed\n");
		return -EIO;
	}

	return 0;
}

/*!
 * @brief This function initializes the fifo configuration.
 */
static int bma4xy_init_fifo_config(
				struct bma4xy_client_data *client_data)
{
	int ret = 0;
	struct bma4_dev *bma4_dev_ptr = &client_data->device;

	ret = bma4_set_fifo_config(BMA4_FIFO_HEADER,
				BMA4_ENABLE, bma4_dev_ptr);
	if (ret) {
		dev_err(client_data->dev, "enable fifo header failed ret=%d\n", ret);
		return -EIO;
	}

	ret = bma4_set_fifo_config(BMA4_FIFO_TIME,
				BMA4_ENABLE, bma4_dev_ptr);
	if (ret) {
		dev_err(client_data->dev, "enable fifo timer failed ret=%d\n", ret);
		return -EIO;
	}

	return 0;
}

/*!
 * @brief This function updates the configuration stream.
 */
static int bma4xy_update_config_stream(
				struct bma4xy_client_data *client_data,
				uint8_t option)
{
	char *name;
	uint16_t ret = 0;
	uint8_t crc_check = 0;
	struct bma4_dev *bma4_dev_ptr = &client_data->device;

	if (option == 3) {
		name = "bma456_config_stream";
		ret = bma456_write_config_file(bma4_dev_ptr);
		if (ret) {
			dev_err(client_data->dev, "download config stream failed\n");
			goto exit_error;
		}
		bma4xy_i2c_delay(200);

		ret = bma4_read_regs(BMA4_INTERNAL_STAT,
					&crc_check, 1,
					bma4_dev_ptr);
		if (ret) {
			dev_err(client_data->dev, "reading CRC failed\n");
			goto exit_error;
		}

		if (crc_check != BMA4_ASIC_INITIALIZED) {
			dev_err(client_data->dev, "crc check error %x\n", crc_check);
			goto exit_error;
		}

		client_data->config_stream_name = name;

	} else {
		dev_err(client_data->dev, "Invalid argument for loading config stream\n");
		return -EINVAL;
	}

	return 0;

exit_error:
	return -EIO;
}

/*!
 * @brief This function configures the features to its previous state
 *        saved in client data. Typically used at reinit after error.
 */
static int bma456_config_feature(struct bma4xy_client_data *client_data)
{
	uint16_t ret = 0;
	uint8_t feature = 0;
	struct bma4_dev *bma4_dev_ptr = &client_data->device;
	/* Structure to define any-motion */
	struct bma456_any_no_mot_config any_mot = {0};

	if (client_data->stepdet_enable == BMA4_ENABLE) {
		if (bma456_step_detector_enable(BMA4_ENABLE,
						bma4_dev_ptr) < 0)
			dev_err(client_data->dev, "set BMA456_STEP_DETECTOR error\n");
	}

	bma4xy_i2c_delay(2);

	if (client_data->anymotion_enable == BMA4_ENABLE) {
		/* Get any-motion configuration to get the default values */
		ret = bma456_get_any_mot_config(&any_mot, bma4_dev_ptr);
		/* Enable any-motion for all axes */
		any_mot.axes_en = BMA456_EN_ALL_AXIS;

		/* Set the new configuration */
		ret += bma456_set_any_mot_config(&any_mot, bma4_dev_ptr);
	}
	if (client_data->nomotion_enable == BMA4_ENABLE) {
		/* Get any-motion configuration to get the default values */
		ret = bma456_get_no_mot_config(&any_mot, bma4_dev_ptr);
		/* Enable any-motion for all axes */
		any_mot.axes_en = BMA456_EN_ALL_AXIS;

		/* Set the new configuration */
		ret += bma456_set_any_mot_config(&any_mot, bma4_dev_ptr);
	}

	if (client_data->stepcounter_enable == BMA4_ENABLE)
		feature = feature | BMA456_STEP_CNTR;
	if (client_data->activity_enable == BMA4_ENABLE)
		feature = feature | BMA456_STEP_ACT;
	if (client_data->single_tap_enable == BMA4_ENABLE)
		feature = feature | BMA456_SINGLE_TAP;
	if (client_data->double_tap_enable == BMA4_ENABLE)
		feature = feature | BMA456_DOUBLE_TAP;
	if (client_data->wrist_wear_wakeup_enable == BMA4_ENABLE)
		feature = feature | BMA456_WRIST_WEAR;

	ret += bma456_feature_enable(feature, BMA4_ENABLE, bma4_dev_ptr);
	if (ret) {
		dev_err(client_data->dev, "set feature ret\n");
		return -EIO;
	}

	return 0;
}

/*!
 * @brief This function reinitializes accelerometer.
 */
static int bma4xy_reinit_acc(struct bma4xy_client_data *client_data)
{
	int err = 0;
	uint16_t ret = 0;
	uint8_t odr_data = 0;
	struct bma4_dev *bma4_dev_ptr = &client_data->device;

	odr_data = client_data->acc_odr;

	if (odr_data != 0) {
		/* odr 6.25HZ, acc_perf_mode = 0 res_avg128 */
		if (odr_data == BMA4_OUTPUT_DATA_RATE_6_25HZ)
			odr_data = BMA4XY_ODR_6_25_HZ;
		else
		/* acc_perf_mode =1 normal_avg4 */
			odr_data |= BMA4XY_PERF_MODE_AVG4;

		ret = client_data->device.bus_write(
					client_data->device.bma4_i2c_client,
					BMA4_ACCEL_CONFIG_ADDR,
					&odr_data, 1);
		if (ret)
			dev_err(client_data->dev, "set acc_odr failed\n");
		err = ret;
		bma4xy_i2c_delay(2);
	}

	if (client_data->pw.acc_pm == 0)
		ret = bma4_set_accel_enable(BMA4_ENABLE, bma4_dev_ptr);
	if (ret)
		dev_err(client_data->dev, "set acc_op_mode failed\n");
	err += ret;

	bma4xy_i2c_delay(2);

	ret = bma4_set_fifo_config(BMA4_FIFO_ACCEL,
				client_data->fifo_acc_enable, bma4_dev_ptr);
	if (ret)
		dev_err(client_data->dev, "set acc_fifo_enable failed\n");
	err += ret;

	bma4xy_i2c_delay(5);

	if (err)
		return -EIO;
	else
		return 0;
}

/*!
 * @brief This function resets the chip to previous state up on error by
 *        configuring to the saved state.
 */
static int bma4xy_reinit_after_error_interrupt(
				struct bma4xy_client_data *client_data)
{
	int ret = 0;
	uint32_t step_count;
	uint8_t crc_check = 0;
	struct bma4_dev *bma4_dev_ptr = &client_data->device;

	atomic_inc(&client_data->err_int_trigger_num);
	step_count = atomic_read(&client_data->step_count_total);
	atomic_set(&client_data->step_count_till_last_err, step_count);

	/* reset the bma4xy */
	ret = bma4_set_command_register(BMA4XY_SOFT_RESET_CMD, bma4_dev_ptr);
	if (!ret)
		dev_dbg(client_data->dev, "reset chip\n");
	else
		dev_err(client_data->dev, "reset chip failed\n");

	/* reinit the fifo config */
	ret = bma4xy_init_fifo_config(client_data);
	if (ret)
		dev_err(client_data->dev, "fifo init failed\n");
	else
		dev_info(client_data->dev, "fifo init successful\n");

	/* reload the config_stream */
	ret = bma4_write_config_file(bma4_dev_ptr);
	if (ret)
		dev_err(client_data->dev, "download config stream failed\n");
	else
		dev_info(client_data->dev, "download  successful\n");

	bma4xy_i2c_delay(200);

	ret = bma4_read_regs(BMA4_INTERNAL_STAT, &crc_check, 1, bma4_dev_ptr);
	if (ret)
		dev_err(client_data->dev, "reading crc failed\n");
	else
		dev_info(client_data->dev, "reading crc successful\n");

	if (crc_check != BMA4_ASIC_INITIALIZED)
		dev_err(client_data->dev, "crc check error %x\n", crc_check);

	/* reconfig interrupt and remap */
	ret = bma4xy_init_after_config_stream_load(client_data);
	if (ret)
		dev_err(client_data->dev, "reconfig interrupt and remap error\n");
	else
		dev_info(client_data->dev, "init successful\n");

	/* reinit the feature */
	ret = bma456_config_feature(client_data);
	if (ret)
		dev_err(client_data->dev, "reinitialization failed\n");

	/* reinit acc */
	bma4xy_reinit_acc(client_data);
	if (ret)
		dev_err(client_data->dev, "acc reinit failed\n");

	return 0;
}

/*!
 * @brief This function initializes the accel input device.
 */
static int bma4xy_acc_input_init(struct bma4xy_client_data *client_data)
{
	struct input_dev *dev;
	int ret = 0;

	dev = input_allocate_device();

	if (dev == NULL)
		return -ENOMEM;

	dev->name = client_data->sensor_name;
	dev->id.bustype = BUS_I2C;
	input_set_capability(dev, EV_ABS, ABS_MISC);
	input_set_capability(dev, EV_MSC, REL_FEATURE_STATUS);
	input_set_drvdata(dev, client_data);
	ret = input_register_device(dev);
	if (ret < 0) {
		input_free_device(dev);
		return ret;
	}

	client_data->acc_input = dev;

	return 0;
}

/*!
 * @brief This function cleans up the accel input device when done.
 */
static void bma4xy_acc_input_destroy(struct bma4xy_client_data *client_data)
{
	struct input_dev *dev = client_data->acc_input;

	input_unregister_device(dev);
	input_free_device(dev);
}

/*!
 * @brief This function initializes the feature input device.
 */
static int bma4xy_feature_input_init(struct bma4xy_client_data *client_data)
{
	struct input_dev *dev;
	int ret = 0;

	dev = input_allocate_device();
	if (dev == NULL)
		return -ENOMEM;

	dev->name = client_data->sensor_name_feature;
	dev->id.bustype = BUS_I2C;
	input_set_capability(dev, EV_MSC, REL_FEATURE_STATUS);
	input_set_drvdata(dev, client_data);
	ret = input_register_device(dev);
	if (ret < 0) {
		input_free_device(dev);
		return ret;
	}

	client_data->feature_input = dev;

	return 0;
}

/*!
 * @brief This function cleans up the feature input device when done.
 */
static void bma4xy_feature_input_destroy(
				struct bma4xy_client_data *client_data)
{
	struct input_dev *dev = client_data->feature_input;

	input_unregister_device(dev);
	input_free_device(dev);
}

/*********************************************************************/
/* sysfs functions */
/*********************************************************************/
/*!
 * @brief Show function for `chip_id` node.
 */
static ssize_t chip_id_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	uint8_t chip_id[2] = {0};
	uint16_t ret = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	ret = client_data->device.bus_read(client_data->device.bma4_i2c_client,
					BMA4_CHIP_ID_ADDR, chip_id, 2);
	if (ret) {
		dev_err(dev, "chip id read failed\n");
		return -EIO;
	}

	return scnprintf(buf, PAGE_SIZE, "chip_id=0x%x rev_id=0x%x\n",
			chip_id[0], chip_id[1]);
}

/*!
 * @brief Show function for `acc_op_mode` node.
 */
static ssize_t acc_op_mode_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	uint16_t ret;
	unsigned char acc_op_mode;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	ret = bma4_get_accel_enable(&acc_op_mode, &client_data->device);
	if (ret) {
		dev_err(dev, "acc opmode read failed\n");
		return -EIO;
	}

	return scnprintf(buf, PAGE_SIZE,
			"1 is enable, now it is %d\n", acc_op_mode);
}

/*!
 * @brief Store function for `acc_op_mode` node.
 */
static ssize_t acc_op_mode_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int ret = 0;
	uint8_t op_mode;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);
	struct bma4_dev *bma4_dev_ptr = &client_data->device;

	ret = kstrtou8(buf, BASE_DECIMAL, &op_mode);
	if (ret)
		return ret;

	if ((op_mode == BMA4XY_SUSPEND_MODE) &&
		(client_data->stepdet_enable == 0) &&
		(client_data->stepcounter_enable == 0) &&
		(client_data->anymotion_enable == 0) &&
		(client_data->nomotion_enable == 0) &&
		(client_data->activity_enable == 0) &&
		(client_data->single_tap_enable == 0) &&
		(client_data->double_tap_enable == 0) &&
		(client_data->wrist_wear_wakeup_enable == 0)) {
		ret = bma4_set_accel_enable(BMA4_DISABLE,
						bma4_dev_ptr);
		dev_dbg(dev, "acc_op_mode %u\n", op_mode);
	} else if (op_mode == BMA4XY_NORMAL_MODE) {
		ret = bma4_set_accel_enable(BMA4_ENABLE, bma4_dev_ptr);
		dev_dbg(dev, "acc_op_mode %u\n", op_mode);
	} else {
		return -EINVAL;
	}

	if (ret) {
		dev_err(dev, "acc enable failed\n");
		return -EIO;
	}

	client_data->pw.acc_pm = op_mode;

	return count;
}

/*!
 * @brief Show function for `acc_value` node.
 */
static ssize_t acc_value_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	uint16_t ret;
	struct bma4_accel data = {0};
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	ret = bma4_read_accel_xyz(&data, &client_data->device);
	if (ret < 0)
		return -EIO;

	return scnprintf(buf, PAGE_SIZE, "%hd %hd %hd\n",
			data.x, data.y, data.z);
}

/*!
 * @brief Show function for `acc_range` node.
 */
static ssize_t acc_range_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	uint16_t ret;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);
	struct bma4_accel_config acc_config;

	ret = bma4_get_accel_config(&acc_config, &client_data->device);
	if (ret) {
		dev_err(dev, "acc config read failed\n");
		return -EIO;
	}

	return scnprintf(buf, PAGE_SIZE, "%d\n", acc_config.range);
}

/*!
 * @brief Store function for accel range node.
 */
static ssize_t acc_range_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret;
	uint8_t acc_range;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);
	struct bma4_accel_config acc_config;
	struct bma4_dev *bma4_dev_ptr = &client_data->device;

	ret = kstrtou8(buf, BASE_DECIMAL, &acc_range);
	if (ret)
		return ret;
	else if (acc_range > BMA4_ACCEL_RANGE_16G)
		return -EINVAL;

	ret = bma4_get_accel_config(&acc_config, bma4_dev_ptr);
	if (ret) {
		dev_err(dev, "acc config read failed\n");
		return -EIO;
	}

	acc_config.range = acc_range;
	ret = bma4_set_accel_config(&acc_config, bma4_dev_ptr);
	if (ret) {
		dev_err(dev, "acc range write failed\n");
		return -EIO;
	}

	return count;
}

/*!
 * @brief Show function for `acc_odr` node.
 */
static ssize_t acc_odr_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	uint16_t ret;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);
	struct bma4_accel_config acc_config;

	ret = bma4_get_accel_config(&acc_config, &client_data->device);
	if (ret) {
		dev_err(dev, "acc config read failed\n");
		return -EIO;
	}

	client_data->acc_odr = acc_config.odr;

	return scnprintf(buf, PAGE_SIZE, "%d\n", acc_config.odr);
}

/*!
 * @brief Store function for `acc_odr` node.
 */
static ssize_t acc_odr_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret;
	uint8_t acc_odr;
	uint8_t data = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	ret = kstrtou8(buf, BASE_DECIMAL, &acc_odr);
	if (ret)
		return ret;

	data = acc_odr;
	/* odr 6.25HZ, acc_perf_mode = 0 res_avg128 */
	if (acc_odr == BMA4_OUTPUT_DATA_RATE_6_25HZ)
		data = BMA4XY_ODR_6_25_HZ;
	else
		/* acc_perf_mode =1 normal_avg4 */
		data |= BMA4XY_PERF_MODE_AVG4;
	ret = client_data->device.bus_write(client_data->device.bma4_i2c_client,
					BMA4_ACCEL_CONFIG_ADDR, &data, 1);
	if (ret) {
		dev_err(dev, "acc odr write failed\n");
		return -EIO;
	}

	dev_dbg(dev, "acc odr = %d\n", acc_odr);
	client_data->acc_odr = acc_odr;

	return count;
}

/*!
 * @brief Show function for `acc_bw` node.
 */
static ssize_t acc_bw_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	uint16_t ret;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);
	struct bma4_accel_config acc_config;

	ret = bma4_get_accel_config(&acc_config, &client_data->device);
	if (ret) {
		dev_err(dev, "acc config read failed\n");
		return -EIO;
	}

	return scnprintf(buf, PAGE_SIZE, "%d\n", acc_config.bandwidth);
}

/*!
 * @brief Store function for `acc_bw` node.
 */
static ssize_t acc_bw_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret;
	uint8_t acc_bw;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);
	struct bma4_accel_config acc_config;
	struct bma4_dev *bma4_dev_ptr = &client_data->device;

	ret = kstrtou8(buf, BASE_DECIMAL, &acc_bw);
	if (ret)
		return ret;

	ret = bma4_get_accel_config(&acc_config, bma4_dev_ptr);
	if (ret) {
		dev_err(dev, "acc config read failed\n");
		return -EIO;
	}

	acc_config.bandwidth = acc_bw;
	ret = bma4_set_accel_config(&acc_config, bma4_dev_ptr);
	if (ret) {
		dev_err(dev, "acc bw write failed\n");
		return -EIO;
	}

	return count;
}

/*!
 * @brief Show function for `selftest` node.
 */
static ssize_t selftest_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	return scnprintf(buf, PAGE_SIZE, "%d\n", client_data->selftest);
}

/*!
 * @brief Store function for `selftest` node.
 */
static ssize_t selftest_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	uint16_t ret;
	uint8_t result = 0;
	uint8_t selftest;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	ret = kstrtou8(buf, BASE_DECIMAL, &selftest);
	if (ret)
		return ret;

	ret = bma4_perform_accel_selftest(&result, &client_data->device);
	if (ret) {
		dev_err(dev, "self test failed\n");
		return -EIO;
	}

	if (result == 0) {
		dev_dbg(dev, "Selftest successful\n");
		client_data->selftest = 1;
	} else {
		client_data->selftest = 0;
	}

	return count;
}

/*!
 * @brief Show function for `foc` node.
 */
static ssize_t foc_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	if (client_data == NULL) {
		dev_err(dev, "Invalid client_data pointer\n");
		return -ENODEV;
	}

	return scnprintf(buf, PAGE_SIZE,
			"Use echo x_axis y_axis z_axis > foc to begin foc\n");
}

/*!
 * @brief Store function for `foc` node.
 */
static ssize_t foc_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int g_value[3] = {0};
	int32_t data[3] = {0};
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);
	int ret = 0;

	if (client_data == NULL) {
		dev_err(dev, "Invalid client_data pointer\n");
		return -ENODEV;
	}

	ret = sscanf(buf, "%11d %11d %11d",
		&g_value[0], &g_value[1], &g_value[2]);
	dev_dbg(dev, "g_value0=%d, g_value1=%d, g_value2=%d\n",
		g_value[0], g_value[1], g_value[2]);
	if (ret != 3) {
		dev_err(dev, "Invalid argument\n");
		return -EINVAL;
	}

	data[0] = g_value[0] * BMA4XY_MULTIPLIER;
	data[1] = g_value[1] * BMA4XY_MULTIPLIER;
	data[2] = g_value[2] * BMA4XY_MULTIPLIER;
	ret = bma4_perform_accel_foc(data, &client_data->device);
	if (ret) {
		dev_err(dev, "FOC failed\n");
		return -EIO;
	}

	dev_info(dev, "FOC successful\n");
	return count;
}

/*!
 * @brief Show function for `config_function` node.
 */
static ssize_t config_function_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	if (client_data == NULL) {
		dev_err(dev, "Invalid client_data pointer\n");
		return -ENODEV;
	}

	return scnprintf(buf, PAGE_SIZE,
		"single_tap1 = %d\n step_detector2 = %d\n"
		"step_counter3 = %d\n wrist_wear_wakeup4 = %d\n"
		"double_tap5 = %d\n any_motion6 = %d\n"
		"nomotion7 = %d\n activity8 = %d\n",
		client_data->single_tap_enable, client_data->stepdet_enable,
		client_data->stepcounter_enable,
		client_data->wrist_wear_wakeup_enable,
		client_data->double_tap_enable, client_data->anymotion_enable,
		client_data->nomotion_enable, client_data->activity_enable);
}

/*!
 * @brief Store function for `config_function` node.
 */
static ssize_t config_function_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int ret;
	int config_func = 0;
	int enable = 0;
	uint8_t feature = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);
	struct bma4_dev *bma4_dev_ptr = &client_data->device;
	/* Structure to define any-motion */
	struct bma456_any_no_mot_config any_mot = {0};


	if (client_data == NULL) {
		dev_err(dev, "Invalid client_data pointer\n");
		return -ENODEV;
	}

	ret = sscanf(buf, "%11d %11d", &config_func, &enable);
	dev_dbg(dev, "config_func = %d, enable=%d\n", config_func, enable);
	if (ret != 2) {
		dev_err(dev, "Invalid argument\n");
		return -EINVAL;
	}

	switch (config_func) {
	case BMA4XY_SINGLE_TAP_SENSOR:
		feature = BMA456_SINGLE_TAP;
		client_data->single_tap_enable = enable;
		break;
	case BMA4XY_DOUBLE_TAP_SENSOR:
		feature = BMA456_DOUBLE_TAP;
		client_data->double_tap_enable = enable;
		break;
	case BMA4XY_WRIST_WEAR_WAKEUP:
		feature = BMA456_WRIST_WEAR;
		client_data->wrist_wear_wakeup_enable = enable;
		break;
	case BMA4XY_STEP_DETECTOR_SENSOR:
		if (bma456_step_detector_enable(enable, bma4_dev_ptr) < 0) {
			dev_err(dev, "set BMA4XY_STEP_DETECTOR_SENSOR error\n");
			return -EINVAL;
		}
		client_data->stepdet_enable = enable;
		break;
	case BMA4XY_STEP_COUNTER_SENSOR:
		feature = BMA456_STEP_CNTR;
		client_data->stepcounter_enable = enable;
		break;
	case BMA4XY_ANY_MOTION_SENSOR:
		ret = bma456_get_any_mot_config(&any_mot, bma4_dev_ptr);
		if (enable)
			any_mot.axes_en = BMA456_EN_ALL_AXIS;
		else
			any_mot.axes_en = BMA456_DIS_ALL_AXIS;

		ret = bma456_set_any_mot_config(&any_mot, bma4_dev_ptr);
		client_data->anymotion_enable = enable;
		break;
	case BMA4XY_NO_MOTION_SENSOR:
		ret = bma456_get_no_mot_config(&any_mot, bma4_dev_ptr);
		if (enable)
			any_mot.axes_en = BMA456_EN_ALL_AXIS;
		else
			any_mot.axes_en = BMA456_DIS_ALL_AXIS;

		ret = bma456_set_no_mot_config(&any_mot, bma4_dev_ptr);
		client_data->nomotion_enable = enable;
		break;
	case BMA4XY_ACTIVITY_SENSOR:
		feature = BMA456_STEP_ACT;
		client_data->activity_enable = enable;
		break;
	default:
		dev_err(dev, "Invalid sensor handle: %d\n", config_func);
		return -EINVAL;
	}

	if (bma456_feature_enable(feature, enable, bma4_dev_ptr) < 0) {
		dev_err(dev, "bma456 feature enable/disable failed\n");
		return -EINVAL;
	}

	return count;
}

/*!
 * @brief Store function for `axis_remapping` node.
 */
static ssize_t axis_remapping_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int ret = 0;
	int data[BMA4XY_NUM_AXES] = {0};
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);
	struct bma456_axes_remap axis_remap_data;

	ret = sscanf(buf, "%11d %11d %11d %11d %11d %11d",
		&data[0], &data[1], &data[2], &data[3], &data[4], &data[5]);
	if (ret != BMA4XY_NUM_AXES) {
		dev_err(dev, "Invalid argument\n");
		return -EINVAL;
	}

	axis_remap_data.x_axis = (uint8_t)data[0];
	axis_remap_data.x_axis_sign = (uint8_t)data[1];
	axis_remap_data.y_axis = (uint8_t)data[2];
	axis_remap_data.y_axis_sign = (uint8_t)data[3];
	axis_remap_data.z_axis = (uint8_t)data[4];
	axis_remap_data.z_axis_sign = (uint8_t)data[5];
	dev_dbg(dev, "x_axis = %d x_axis_sign=%d\n",
		axis_remap_data.x_axis, axis_remap_data.x_axis_sign);
	dev_dbg(dev, "y_axis = %d y_axis_sign=%d\n",
		axis_remap_data.y_axis, axis_remap_data.y_axis_sign);
	dev_dbg(dev, "z_axis = %d z_axis_sign=%d\n",
		axis_remap_data.z_axis, axis_remap_data.z_axis_sign);

	ret = bma456_set_remap_axes(&axis_remap_data, &client_data->device);
	if (ret) {
		dev_err(dev, "axis remap failed\n");
		return -EIO;
	}

	return count;
}

/*!
 * @brief Show function for `fifo_length` node.
 */
static ssize_t fifo_length_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	uint16_t ret;
	uint16_t fifo_bytecount = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	ret = bma4_get_fifo_length(&fifo_bytecount, &client_data->device);
	if (ret) {
		dev_err(dev, "fifo length read failed\n");
		return -EIO;
	}

	return scnprintf(buf, PAGE_SIZE, "%d\n", fifo_bytecount);
}

/*!
 * @brief Store function for `fifo_flush` node.
 */
static ssize_t fifo_flush_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret;
	unsigned long enable;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	ret = kstrtoul(buf, BASE_DECIMAL, &enable);
	if (ret)
		return ret;

	if (enable) {
		ret = bma4_set_command_register(BMA4XY_CLEAR_FIFO_DATA_CMD,
						&client_data->device);
		if (ret) {
			dev_err(dev, "bus write failed\n");
			return -EIO;
		}
	} else {
		return -EINVAL;
	}

	return count;
}

/*!
 * @brief Show function for `acc_fifo_enable` node.
 */
static ssize_t acc_fifo_enable_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	uint16_t ret;
	uint8_t fifo_acc_enable;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	ret = bma4_get_fifo_config(&fifo_acc_enable, &client_data->device);
	fifo_acc_enable = (fifo_acc_enable & BMA4XY_FIFO_ACC_EN_MSK)
						>> BMA4XY_FIFO_ACC_EN_POS;
	if (ret) {
		dev_err(dev, "fifo config read failed\n");
		return -EIO;
	}

	return scnprintf(buf, PAGE_SIZE, "%x\n", fifo_acc_enable);
}

/*!
 * @brief Store function for `acc_fifo_enable` node.
 */
static ssize_t acc_fifo_enable_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int ret;
	uint8_t fifo_acc_enable;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	ret = kstrtou8(buf, BASE_DECIMAL, &fifo_acc_enable);
	if (ret)
		return ret;

	ret = bma4_set_fifo_config(BMA4_FIFO_ACCEL,
				fifo_acc_enable, &client_data->device);
	if (ret) {
		dev_err(dev, "fifo_acc_enable failed\n");
		return -EIO;
	}

	client_data->fifo_acc_enable = fifo_acc_enable;

	return count;
}

/*!
 * @brief Show function for `load_config_stream` node.
 */
static ssize_t load_config_stream_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	return scnprintf(buf, PAGE_SIZE, "config stream %s\n",
		client_data->config_stream_name);
}

/*!
 * @brief Store function for `load_config_stream` node.
 */
static ssize_t load_config_stream_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	uint8_t option = 0;
	int ret = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	ret = kstrtou8(buf, BASE_DECIMAL, &option);
	if (ret)
		return ret;

	dev_dbg(dev, "config stream option %u\n", option);
	ret = bma4xy_update_config_stream(client_data, option);
	if (ret) {
		dev_err(dev, "config stream load error\n");
		return count;
	}

	ret = bma4xy_init_after_config_stream_load(client_data);
	if (ret) {
		dev_err(dev, "bma4xy_init_after_config_stream_load error\n");
		return count;
	}

	return count;
}

/*!
 * @brief Show function for `fifo_watermark` node.
 */
static ssize_t fifo_watermark_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	uint16_t ret;
	uint16_t data;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	ret = bma4_get_fifo_wm(&data, &client_data->device);
	if (ret) {
		dev_err(dev, "read fifo watermark failed\n");
		return -EIO;
	}

	return scnprintf(buf, PAGE_SIZE, "%d\n", data);
}

/*!
 * @brief Store function for `fifo_watermark` node.
 */
static ssize_t fifo_watermark_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int ret;
	uint16_t fifo_watermark;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	ret = kstrtou16(buf, BASE_DECIMAL, &fifo_watermark);
	if (ret)
		return ret;

	ret = bma4_set_fifo_wm(fifo_watermark, &client_data->device);
	if (ret) {
		dev_err(dev, "set fifo watermark failed\n");
		return -EIO;
	}

	return count;
}

/*!
 * @brief Show function for `fifo_data_frame` node.
 */
static ssize_t fifo_data_out_frame_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);
	uint16_t ret = 0;
	uint16_t fifo_bytecount = 0;
	struct bma4_dev *bma4_dev_ptr = &client_data->device;

	if (!client_data->fifo_acc_enable) {
		dev_err(dev, "no sensor is selected for fifo data read\n");
		return -EINVAL;
	}

	ret = bma4_get_fifo_length(&fifo_bytecount, bma4_dev_ptr);
	if (ret) {
		dev_err(dev, "read fifo length ret=%d\n", ret);
		return -EIO;
	}

	if (fifo_bytecount) {
		ret =  bma4_read_regs(BMA4_FIFO_DATA_ADDR,
					buf, fifo_bytecount,
					bma4_dev_ptr);
		if (ret) {
			dev_err(dev, "read fifo failed\n");
			return -EIO;
		}
	}

	return fifo_bytecount;
}

/*!
 * @brief Show function for `reg_sel` node.
 */
static ssize_t reg_sel_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	if (client_data == NULL) {
		dev_err(dev, "Invalid client_data pointer\n");
		return -ENODEV;
	}

	return scnprintf(buf, PAGE_SIZE, "reg=0X%02X, len=%d\n",
				client_data->reg_sel, client_data->reg_len);
}

/*!
 * @brief Store function for `reg_sel` node.
 */
static ssize_t reg_sel_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);
	int ret;
	unsigned int reg_sel, reg_len;

	if (client_data == NULL) {
		dev_err(dev, "Invalid client_data pointer\n");
		return -ENODEV;
	}

	ret = sscanf(buf, "%11X %11d", &reg_sel, &reg_len);
	if ((ret != 2) || (reg_sel > BMA4XY_MAX_REG_ADDR)
			|| (reg_len > BMA4XY_MAX_REG_LEN)) {
		dev_err(dev, "Invalid argument\n");
		return -EINVAL;
	}

	client_data->reg_sel = reg_sel;
	client_data->reg_len = reg_len;

	return count;
}

/*!
 * @brief Show function for `reg_val` node.
 */
static ssize_t reg_val_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);
	uint16_t ret = 0;
	uint8_t reg_data[128], i;
	int16_t pos;

	if (client_data == NULL) {
		dev_err(dev, "Invalid client_data pointer\n");
		return -ENODEV;
	}

	ret = client_data->device.bus_read(client_data->device.bma4_i2c_client,
					client_data->reg_sel,
					reg_data,
					client_data->reg_len);
	if (ret) {
		dev_err(dev, "Reg op failed\n");
		return -EIO;
	}

	pos = 0;
	for (i = 0; i < client_data->reg_len; ++i) {
		pos += scnprintf(buf + pos, 16, "%02X", reg_data[i]);
		/* insert a space after a value and
		 * a new line after 16 values
		 */
		buf[pos++] = (((i + 1) % 16) == 0 ? '\n' : ' ');
	}
	if (buf[pos - 1] == ' ')
		buf[pos - 1] = '\n';
	return pos;
}

/*!
 * @brief Store function for `reg_val` node.
 */
static ssize_t reg_val_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);
	uint16_t err;
	uint8_t reg_data[128] = {0};
	unsigned int reg_val;
	int i, buf_offset, length, ret = 0;

	if (client_data == NULL) {
		dev_err(dev, "Invalid client_data pointer\n");
		return -ENODEV;
	}

	buf_offset = 0;
	for (i = 0; i < client_data->reg_len; ++i) {
		if (buf_offset >= count)
			return -EINVAL;
		ret =  sscanf(buf + buf_offset, "%x %n", &reg_val, &length);
		if ((ret != 1) || (reg_val > 255))
			return -EINVAL;
		reg_data[i] = reg_val;
		buf_offset += length;
	}

	dev_dbg(dev, "Reg data read as\n");
	for (i = 0; i < client_data->reg_len; ++i)
		dev_dbg(dev, "%d", reg_data[i]);
	err = client_data->device.bus_write(client_data->device.bma4_i2c_client,
					client_data->reg_sel,
					reg_data,
					client_data->reg_len);
	if (err) {
		dev_err(dev, "Reg operation failed\n");
		return -EIO;
	}

	return count;
}

/*!
 * @brief Show function for `driver_version` node.
 */
static ssize_t driver_version_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	return scnprintf(buf, PAGE_SIZE,
		"Driver version: %s\n", DRIVER_VERSION);
}

/*!
 * @brief Show function for `config_file_version` node.
 */
static ssize_t config_file_version_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	uint16_t ret = 0;
	uint16_t version = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	ret = bma456_get_config_id(&version, &client_data->device);
	if (ret) {
		dev_err(dev, "config read failed\n");
		return -EIO;
	}

	return scnprintf(buf, PAGE_SIZE,
		"Driver version: %s Config_stream version :0x%x\n",
		DRIVER_VERSION, version);
}

/*!
 * @brief Show function for `avail_sensor` node.
 */
static ssize_t avail_sensor_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	uint16_t avail_sensor = 0;

	avail_sensor = 456;

	return scnprintf(buf, PAGE_SIZE, "%d\n", avail_sensor);
}

/*!
 * @brief Show function for `step_counter_val` node.
 */
static ssize_t step_counter_val_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int ret = 0;
	uint32_t step_count_val = 0, current_step_count;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	ret = bma456_step_counter_output(&current_step_count,
					&client_data->device);

	if (ret) {
		dev_err(dev, "step count read failed\n");
		return -EIO;
	}

	dev_dbg(dev, "val %u\n", current_step_count);

	if (atomic_read(&client_data->err_int_trigger_num) == 0) {
		step_count_val = current_step_count;
		dev_dbg(dev, "report val %u", step_count_val);
		atomic_set(&client_data->step_count_till_last_err,
			step_count_val);
	} else {
		step_count_val = atomic_read(
				&client_data->step_count_till_last_err);
		step_count_val += current_step_count;
		dev_dbg(dev, "after ret report val %u\n", step_count_val);
	}
	atomic_set(&client_data->step_count_total, step_count_val);

	ret = scnprintf(buf, PAGE_SIZE, "%u\n", step_count_val);

	return ret;
}

/*!
 * @brief Show function for `step_counter_watermark` node.
 */
static ssize_t step_counter_watermark_show(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	uint16_t ret = 0;
	uint16_t watermark;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	ret = bma456_step_counter_get_watermark(&watermark,
					&client_data->device);
	if (ret) {
		dev_err(dev, "get step count watermark failed\n");
		return -EIO;
	}

	return scnprintf(buf, PAGE_SIZE, "%d\n", watermark);
}

/*!
 * @brief Store function for `step_counter_watermark` node.
 */
static ssize_t step_counter_watermark_store(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	int ret = 0;
	uint16_t step_watermark;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	ret = kstrtou16(buf, BASE_DECIMAL, &step_watermark);
	if (ret)
		return ret;

	dev_dbg(dev, "step_counter watermark %u\n", step_watermark);

	ret = bma456_step_counter_set_watermark(step_watermark,
						&client_data->device);

	if (ret) {
		dev_err(dev, "set step count watermark failed\n");
		return -EIO;
	}

	return count;
}

/*!
 * @brief Show function for `step_counter_parameter` node.
 */
static ssize_t step_counter_parameter_show(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	uint16_t ret = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);
	struct bma456_stepcounter_settings setting;

	ret = bma456_stepcounter_get_parameter(&setting, &client_data->device);
	if (ret) {
		dev_err(dev, "stepcounter parameter read failed\n");
		return -EIO;
	}

	return scnprintf(buf, PAGE_SIZE,
	"parameter1 =0x%x parameter2= 0x%x\n"
	"parameter3 = 0x%x parameter4 = 0x%x\n"
	"parameter5 = 0x%x parameter6 = 0x%x\n"
	"parameter7 = 0x%x parameter8 = 0x%x\n"
	"parameter9 = 0x%x parameter10 = 0x%x\n"
	"parameter11 = 0x%x parameter12 = 0x%x\n"
	"parameter13 = 0x%x parameter14 = 0x%x\n"
	"parameter15 = 0x%x parameter16 = 0x%x\n"
	"parameter17 = 0x%x parameter18 = 0x%x\n"
	"parameter19 = 0x%x parameter20 = 0x%x\n"
	"parameter21 = 0x%x parameter22 = 0x%x\n"
	"parameter23 = 0x%x parameter24 = 0x%x\n"
	"parameter25 = 0x%x\n",
	setting.param1, setting.param2, setting.param3, setting.param4,
	setting.param5, setting.param6, setting.param7, setting.param8,
	setting.param9, setting.param10, setting.param11, setting.param12,
	setting.param13, setting.param14, setting.param15, setting.param16,
	setting.param17, setting.param18, setting.param19, setting.param20,
	setting.param21, setting.param22, setting.param23, setting.param24,
	setting.param25);
}

/*!
 * @brief Store function for `step_counter_parameter` node.
 */
static ssize_t step_counter_parameter_store(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	int ret = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);
	unsigned int data[25] = {0};
	struct bma456_stepcounter_settings setting;

	ret = sscanf(buf,
			"%11x %11x %11x %11x %11x %11x %11x %11x\n"
			"%11x %11x %11x %11x %11x %11x %11x %11x\n"
			"%11x %11x %11x %11x %11x %11x %11x %11x\n"
			"%11x\n",
			&data[0], &data[1], &data[2], &data[3], &data[4],
			&data[5], &data[6], &data[7], &data[8], &data[9],
			&data[10], &data[11], &data[12], &data[13], &data[14],
			&data[15], &data[16], &data[17], &data[18], &data[19],
			&data[20], &data[21], &data[22], &data[23], &data[24]);
	if (ret != 25) {
		dev_err(dev, "Invalid argument\n");
		return -EINVAL;
	}

	setting.param1 = (uint16_t)data[0];
	setting.param2 = (uint16_t)data[1];
	setting.param3 = (uint16_t)data[2];
	setting.param4 = (uint16_t)data[3];
	setting.param5 = (uint16_t)data[4];
	setting.param6 = (uint16_t)data[5];
	setting.param7 = (uint16_t)data[6];
	setting.param8 = (uint16_t)data[7];
	setting.param9 = (uint16_t)data[8];
	setting.param10 = (uint16_t)data[9];
	setting.param11 = (uint16_t)data[10];
	setting.param12 = (uint16_t)data[11];
	setting.param13 = (uint16_t)data[12];
	setting.param14 = (uint16_t)data[13];
	setting.param15 = (uint16_t)data[14];
	setting.param16 = (uint16_t)data[15];
	setting.param17 = (uint16_t)data[16];
	setting.param18 = (uint16_t)data[17];
	setting.param19 = (uint16_t)data[18];
	setting.param20 = (uint16_t)data[19];
	setting.param21 = (uint16_t)data[20];
	setting.param22 = (uint16_t)data[21];
	setting.param23 = (uint16_t)data[22];
	setting.param24 = (uint16_t)data[23];
	setting.param25 = (uint16_t)data[24];
	ret = bma456_stepcounter_set_parameter(&setting, &client_data->device);
	if (ret) {
		dev_err(dev, "set stepcounter parameter failed\n");
		return -EIO;
	}

	return count;
}

/*!
 * @brief Store function for `step_counter_reset` node.
 */
static ssize_t step_counter_reset_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int ret = 0;
	unsigned long reset_counter;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	ret = kstrtoul(buf, BASE_DECIMAL, &reset_counter);
	if (ret)
		return ret;

	dev_dbg(dev, "reset_counter %ld\n", reset_counter);
	ret = bma456_reset_step_counter(&client_data->device);
	if (ret) {
		dev_err(dev, "reset step counter failed\n");
		return -EIO;
	}

	atomic_set(&client_data->step_count_till_last_err, 0);
	atomic_set(&client_data->step_count_total, 0);

	return count;
}

/*!
 * @brief Show function for `anymotion_config` node.
 */
static ssize_t anymotion_config_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	uint16_t ret;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);
	struct bma456_any_no_mot_config any_motion;

	ret = bma456_get_any_mot_config(&any_motion, &client_data->device);

	if (ret) {
		dev_err(dev, "anymotion config read failed\n");
		return -EIO;
	}

	return scnprintf(buf, PAGE_SIZE,
			"duration =0x%x threshold= 0x%x\n",
			any_motion.duration, any_motion.threshold);

}

/*!
 * @brief Store function for `anymotion_config` node.
 */
static ssize_t anymotion_config_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int ret = 0;
	unsigned int data[2] = {0};
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);
	struct bma456_any_no_mot_config any_motion;

	ret = sscanf(buf, "%11x %11x", &data[0], &data[1]);
	if (ret != 2) {
		dev_err(dev, "Invalid argument\n");
		return -EINVAL;
	}

	memset(&any_motion, 0, sizeof(any_motion));
	any_motion.duration = (uint16_t)data[0];
	any_motion.threshold = (uint16_t)data[1];
	ret = bma456_set_any_mot_config(&any_motion, &client_data->device);

	if (ret) {
		dev_err(dev, "anymotion config write failed\n");
		return -EIO;
	}

	return count;
}

/*!
 * @brief Store function for `nomotion_config` node.
 */
static ssize_t nomotion_config_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int ret = 0;
	unsigned int data[2] = {0};
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);
	struct bma456_any_no_mot_config no_motion;

	ret = sscanf(buf, "%11x %11x", &data[0], &data[1]);
	if (ret != 2) {
		dev_err(dev, "Invalid argument\n");
		return -EINVAL;
	}

	memset(&no_motion, 0, sizeof(no_motion));
	no_motion.duration = (uint16_t)data[0];
	no_motion.threshold = (uint16_t)data[1];
	ret = bma456_set_no_mot_config(&no_motion, &client_data->device);

	if (ret) {
		dev_err(dev, "nomotion config write failed\n");
		return -EIO;
	}

	return count;
}

/*!
 * @brief Show function for `nomotion_config` node.
 */
static ssize_t nomotion_config_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	uint16_t ret;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);
	struct bma456_any_no_mot_config no_motion;

	ret = bma456_get_no_mot_config(&no_motion, &client_data->device);

	if (ret) {
		dev_err(dev, "anymotion config read failed\n");
		return -EIO;
	}

	return scnprintf(buf, PAGE_SIZE,
			"duration =0x%x threshold= 0x%x\n",
			no_motion.duration, no_motion.threshold);

}
/*!
 * @brief Show function for `err_int` node.
 */
static ssize_t err_int_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return scnprintf(buf, PAGE_SIZE,
			"check whether the sensor is working fine");
}

/*!
 * @brief Store function for `err_int` node.
 */
static ssize_t err_int_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret = 0;
	unsigned long op_mode;
	struct input_dev *input = to_input_dev(dev);
	struct bma4xy_client_data *client_data = input_get_drvdata(input);

	ret = kstrtoul(buf, BASE_DECIMAL, &op_mode);
	if (ret)
		return ret;

	ret = bma4xy_reinit_after_error_interrupt(client_data);
	if (ret)
		return ret;

	return count;
}

static DEVICE_ATTR_RO(chip_id);
static DEVICE_ATTR_RW(acc_op_mode);
static DEVICE_ATTR_RO(acc_value);
static DEVICE_ATTR_RW(acc_range);
static DEVICE_ATTR_RW(acc_odr);
static DEVICE_ATTR_RW(acc_bw);
static DEVICE_ATTR_RW(acc_fifo_enable);
static DEVICE_ATTR_WO(fifo_flush);
static DEVICE_ATTR_RW(selftest);
static DEVICE_ATTR_RO(avail_sensor);
static DEVICE_ATTR_RO(fifo_length);
static DEVICE_ATTR_RW(fifo_watermark);
static DEVICE_ATTR_RW(load_config_stream);
static DEVICE_ATTR_RW(reg_sel);
static DEVICE_ATTR_RW(reg_val);
static DEVICE_ATTR_RO(driver_version);
static DEVICE_ATTR_RO(config_file_version);
static DEVICE_ATTR_RO(fifo_data_out_frame);
static DEVICE_ATTR_RW(foc);
static DEVICE_ATTR_RW(config_function);
static DEVICE_ATTR_WO(axis_remapping);
static DEVICE_ATTR_RO(step_counter_val);
static DEVICE_ATTR_RW(step_counter_watermark);
static DEVICE_ATTR_RW(step_counter_parameter);
static DEVICE_ATTR_WO(step_counter_reset);
static DEVICE_ATTR_RW(anymotion_config);
static DEVICE_ATTR_RW(nomotion_config);
static DEVICE_ATTR_RW(err_int);

static struct attribute *bma4xy_attributes[] = {
	&dev_attr_chip_id.attr,
	&dev_attr_acc_op_mode.attr,
	&dev_attr_acc_value.attr,
	&dev_attr_acc_range.attr,
	&dev_attr_acc_odr.attr,
	&dev_attr_acc_bw.attr,
	&dev_attr_acc_fifo_enable.attr,
	&dev_attr_selftest.attr,
	&dev_attr_avail_sensor.attr,
	&dev_attr_foc.attr,
	&dev_attr_fifo_length.attr,
	&dev_attr_fifo_watermark.attr,
	&dev_attr_fifo_flush.attr,
	&dev_attr_driver_version.attr,
	&dev_attr_load_config_stream.attr,
	&dev_attr_fifo_data_out_frame.attr,
	&dev_attr_config_file_version.attr,
	&dev_attr_reg_sel.attr,
	&dev_attr_reg_val.attr,
	&dev_attr_config_function.attr,
	&dev_attr_axis_remapping.attr,
	&dev_attr_step_counter_val.attr,
	&dev_attr_step_counter_watermark.attr,
	&dev_attr_step_counter_parameter.attr,
	&dev_attr_step_counter_reset.attr,
	&dev_attr_anymotion_config.attr,
	&dev_attr_nomotion_config.attr,
	&dev_attr_err_int.attr,
	NULL
};

static struct attribute_group bma4xy_attribute_group = {
	.attrs = bma4xy_attributes
};

/*!
 *  @brief This is the probe function for BMA sensor.
 *  Called from the driver (I2C) probe function to
 *  setup the sensor.
 */
int bma4xy_probe(struct bma4xy_client_data *client_data, struct device *dev)
{
	int ret = 0;

	dev_info(dev, "%s: function entrance\n", __func__);

	/* check chip id */
	ret = bma4xy_check_chip_id(client_data, dev);
	if (!ret) {
		dev_info(dev, "Bosch Sensortec Device detected\n");
	} else {
		dev_err(dev, "Bosch Sensortec Device not found, chip id mismatch\n");
		ret = -ENODEV;
		goto exit_err_clean;
	}

	dev_set_drvdata(dev, client_data);

	client_data->dev = dev;
	client_data->device.delay = bma4xy_i2c_delay;

	/* acc input device init */
	ret = bma4xy_acc_input_init(client_data);

	if (ret < 0)
		goto exit_err_clean;

	/* sysfs node creation */
	ret = sysfs_create_group(&client_data->acc_input->dev.kobj,
			&bma4xy_attribute_group);
	if (ret < 0)
		goto exit_err_clean;

	ret = bma4xy_feature_input_init(client_data);
	if (ret < 0)
		goto exit_err_clean;

	ret = bma456_init(&client_data->device);
	if (ret < 0) {
		dev_err(dev, "init failed\n");
		goto exit_err_clean;
	}

	ret = bma4_set_command_register(BMA4XY_SOFT_RESET_VAL,
					&client_data->device);
	if (!ret)
		dev_dbg(dev, "reset chip success\n");
	else
		dev_err(dev, "reset chip failed\n");
	bma4xy_i2c_delay(10);

	if (ret)
		dev_err(dev, "fifo init failed\n");

	dev_info(dev, "Bosch Sensortec Device probed successfully\n");
	return 0;

exit_err_clean:
	if (ret) {
		if (client_data->device.temp_buff)
			kfree(client_data->device.temp_buff);
		if (client_data != NULL)
			kfree(client_data);
		return ret;
	}

	return ret;
}

/*!
 *  @brief This function is used for cleanup while removing driver.
 *  Called from the driver (I2C/SPI) remove function.
 */
int bma4xy_remove(struct device *dev)
{
	int ret = 0;
	struct bma4xy_client_data *client_data = dev_get_drvdata(dev);

	if (client_data != NULL) {
		bma4xy_i2c_delay(BMA4XY_I2C_WRITE_DELAY_TIME);
		sysfs_remove_group(&client_data->acc_input->dev.kobj,
				&bma4xy_attribute_group);
		bma4xy_acc_input_destroy(client_data);
		bma4xy_feature_input_destroy(client_data);
		client_data->device.bma4_i2c_client = NULL;
		if (client_data->device.temp_buff)
			kfree(client_data->device.temp_buff);
		kfree(client_data);
	}

	return ret;
}
EXPORT_SYMBOL(bma4xy_remove);
/** @}*/
