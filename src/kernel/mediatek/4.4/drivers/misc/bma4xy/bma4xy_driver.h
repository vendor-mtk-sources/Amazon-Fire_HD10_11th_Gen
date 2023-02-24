/*!
 * @section LICENSE
 * (C) Copyright 2019~2020 Bosch Sensortec GmbH All Rights Reserved
 *
 * This software program is licensed subject to the GNU General
 * Public License (GPL).Version 2,June 1991,
 * available at http://www.fsf.org/copyleft/gpl.html
 *
 * @filename bma4xy_driver.h
 * @date     2020/01/13
 * @version  0.2.8
 *
 * @brief   The head file of BMA4XY device driver core code
 */

#ifndef BMA4XY_DRIVER_H_
#define BMA4XY_DRIVER_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************/
/* header files */
/*********************************************************************/
#include <linux/types.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/firmware.h>
#include "bma4_defs.h"
#include "bma4.h"
#include "bma456.h"

/*********************************************************************/
/* macro definitions */
/*********************************************************************/
/*! Name of the device driver and accel input device*/
#define SENSOR_NAME                     "bma4xy_acc"
/*! Name of the feature input device*/
#define SENSOR_NAME_FEATURE             "bma4xy_feature"

#define SENSOR_NAME_SIZE                (24)

#define SENSOR_NAME_FEATURE_SIZE        (24)

#define BMA4XY_MAX_REG_ADDR             UINT8_C(0x7E)
#define BMA4XY_MAX_REG_LEN              UINT8_C(0x7E)
#define BMA4XY_I2C_WRITE_DELAY_TIME     UINT8_C(1)
#define BMA4XY_MAX_RETRY_I2C_XFER       UINT8_C(10)
#define BMA4XY_MAX_RETRY_WAIT_DRDY      UINT8_C(100)
#define BMA4XY_I2C_WRITE_DELAY_TIME     UINT8_C(1)
#define BMA4XY_MAX_RETRY_WAKEUP         UINT8_C(5)
#define BMA4XY_DELAY_MIN                UINT8_C(1)
#define BMA4XY_DELAY_DEFAULT            UINT8_C(200)
#define BMA4XY_SOFT_RESET_VAL           UINT8_C(0XB6)
#define BMA4XY_ODR_6_25_HZ              UINT8_C(0x74)
#define BMA4XY_PERF_MODE_AVG4           UINT8_C(0xA0)
#define REL_FEATURE_STATUS              UINT8_C(1)
#define BMA4XY_NUM_AXES                 UINT8_C(6)
#define BMA4XY_SOFT_RESET_CMD           UINT8_C(0xB6)
#define BMA4XY_CLEAR_FIFO_DATA_CMD      UINT8_C(0xB0)
#define BASE_DECIMAL                    UINT8_C(10)
#define BMA4XY_FIFO_ACC_EN_MSK          UINT8_C(0x40)
#define BMA4XY_FIFO_ACC_EN_POS          UINT8_C(0x06)

/*********************************************************************/
/* global variables */
/*********************************************************************/

/*!
 * @name power mode variable
 *
 * @brief This variable is used to track the power modes of acc and mag
 *
 */
struct pw_mode {
	uint8_t acc_pm;
	uint8_t mag_pm;
};

/*!
 *
 * @name client data variable
 *
 * @brief This variable contains the state of the driver
 *
 */
struct bma4xy_client_data {
	struct bma4_dev device;
	struct device *dev;
	struct input_dev *acc_input;
	struct input_dev *feature_input;
	uint8_t fifo_acc_enable;
	struct pw_mode pw;
	uint8_t acc_odr;
	char *config_stream_name;
	uint8_t reg_sel;
	uint8_t reg_len;
	uint8_t selftest;
	uint8_t single_tap_enable;
	uint8_t double_tap_enable;
	uint8_t stepdet_enable;
	uint8_t stepcounter_enable;
	uint8_t wrist_wear_wakeup_enable;
	uint8_t anymotion_enable;
	uint8_t nomotion_enable;
	uint8_t activity_enable;
	atomic_t err_int_trigger_num;
	atomic_t step_count_till_last_err;
	atomic_t step_count_total;
	char sensor_name[SENSOR_NAME_SIZE];
	char sensor_name_feature[SENSOR_NAME_FEATURE_SIZE];
};

/*********************************************************************/
/* function prototype declarations */
/*********************************************************************/
/*!
 *  @brief This is the probe function for bma sensor. Called from the
 *  driver (I2C) probe function to setup the sensor.
 *
 *  @param[in,out] client_data  : Structure instance of client data variable.
 *  @param[in,out] dev  : Structure instance of device.
 *
 *  @return Result of execution status
 *  @retval zero -> Success / negative value -> Error
 */
int bma4xy_probe(struct bma4xy_client_data *client_data, struct device *dev);

/*!
 *  @brief This function is used for cleanup while removing driver. Called from
 *  the driver (I2C) remove function.
 *
 *  @param[in,out] dev  : Structure instance of device.
 *
 *  @return Result of execution status
 *  @retval zero -> Success / negative value -> Error
 */
int bma4xy_remove(struct device *dev);

#ifdef __cplusplus
}
#endif

#endif /* BMA4XY_DRIVER_H_  */
/** @}*/
