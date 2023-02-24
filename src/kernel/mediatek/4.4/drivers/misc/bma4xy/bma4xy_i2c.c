/*!
 * @section LICENSE
 * (C) Copyright 2019~202 Bosch Sensortec GmbH All Rights Reserved
 *
 * This software program is licensed subject to the GNU General
 * Public License (GPL).Version 2,June 1991,
 * available at http://www.fsf.org/copyleft/gpl.html
 *
 * @filename bma4xy_i2c.c
 * @date     2020/01/13
 * @version  0.2.8
 *
 * @brief    bma4xy I2C bus Driver
 */

/*********************************************************************/
/* system header files */
/*********************************************************************/
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/input.h>

/*********************************************************************/
/* own header files */
/*********************************************************************/
#include "bma4xy_driver.h"

/*********************************************************************/
/* static function declarations */
/*********************************************************************/
/*!
 * @brief The I2C read function.
 *
 * @param client[in,out] : Instance of the I2C client
 * @param reg_addr[in] : The register address from where the data is read.
 * @param data[out] : The pointer to buffer to return data.
 * @param len[in] : The number of bytes to be read
 *
 * @return the status
 * @retval 0 -> Success
 * @retval negative -> fail
 */
static int bma4xy_i2c_read(struct i2c_client *client,
			uint8_t reg_addr, uint8_t *data, uint16_t len);

/*!
 * @brief The I2C write function.
 *
 * @param client[in,out] : Instance of the I2C client
 * @param reg_addr[in]   : The register adress of register to strat write
 * @param data[in]       : The pointer to buffer holding data to be written.
 * @param len[in]        : The number of bytes to wtrite
 *
 * @return the status
 * @retval 0 -> Success
 * @retval negative -> fail
 */
static int bma4xy_i2c_write(struct i2c_client *client,
			uint8_t reg_addr, uint8_t *data, uint16_t len);

/*!
 * @brief The I2C probe function.
 *
 * @param client[in, out]   : The I2C client instance
 * @param id[in, out]       : The I2C device ID instance
 *
 * @return the status
 * @retval 0 -> Success
 * @retval negative -> fail
 */
static int bma4xy_i2c_probe(struct i2c_client *client,
			const struct i2c_device_id *id);

/*********************************************************************/
/* functions */
/*********************************************************************/
/*!
 * @brief The I2C read function.
 */
static int bma4xy_i2c_read(struct i2c_client *client,
			uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	int32_t retry;

	struct i2c_msg msg[] = {
		{
		.addr = client->addr,
		.flags = 0,
		.len = 1,
		.buf = &reg_addr,
		},

		{
		.addr = client->addr,
		.flags = I2C_M_RD,
		.len = len,
		.buf = data,
		},
	};
	for (retry = 0; retry < BMA4XY_MAX_RETRY_I2C_XFER; retry++) {
		if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) > 0)
			break;

		usleep_range(BMA4XY_I2C_WRITE_DELAY_TIME * 1000,
				BMA4XY_I2C_WRITE_DELAY_TIME * 1000);
	}

	if (retry >= BMA4XY_MAX_RETRY_I2C_XFER) {
		dev_err(&client->dev, "I2C xfer error\n");
		return -EIO;
	}

	return 0;
}

/**
 * @brief I2C write function.
 *
 * @param client[in,out] : Instance of the I2C client
 * @param reg_addr[in] : The register address to start writing the data.
 * @param data[in] : The pointer to buffer holding data to be written.
 * @param len[in] : The number of bytes to write.
 *
 * @return the status
 * @retval 0 -> Success
 * @retval negative -> fail
 */
static int bma4xy_i2c_write(struct i2c_client *client,
	uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	int32_t retry;

	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = 0,
		.len = len + 1,
		.buf = NULL,
	};
	msg.buf = kmalloc(len + 1, GFP_KERNEL);
	if (!msg.buf) {
		dev_err(&client->dev, "Allocate mem failed\n");
		return -ENOMEM;
	}
	msg.buf[0] = reg_addr;
	memcpy(&msg.buf[1], data, len);

	for (retry = 0; retry < BMA4XY_MAX_RETRY_I2C_XFER; retry++) {
		if (i2c_transfer(client->adapter, &msg, 1) > 0)
			break;

		usleep_range(BMA4XY_I2C_WRITE_DELAY_TIME * 1000,
				BMA4XY_I2C_WRITE_DELAY_TIME * 1000);
	}
	kfree(msg.buf);
	if (retry >= BMA4XY_MAX_RETRY_I2C_XFER) {
		dev_err(&client->dev, "I2C xfer error\n");
		return -EIO;
	}

	return 0;
}

/**
 * @brief I2c Client initialization function.
 *
 * @param client_data[in,out] : BMA4xy driver instance
 * @param client[in,out] : I2C client instance
 *
 * @return the status
 * @retval 0 -> Success
 * @retval negative -> fail
 */
static int bma4_init_i2c_client(struct bma4xy_client_data *client_data,
					struct i2c_client *client)
{
	client_data = kzalloc(sizeof(struct bma4xy_client_data),
						GFP_KERNEL);
	if (client_data == NULL) {
		dev_err(&client->dev, "no memory available\n");
		return -ENOMEM;
	}

	/* h/w init */
	client_data->device.interface = BMA4_I2C_INTERFACE;
	client_data->device.bus_read = (void *)bma4xy_i2c_read;
	client_data->device.bus_write = (void *)bma4xy_i2c_write;
	client_data->device.read_write_len = 8;
	client_data->device.bma4_i2c_client = client;

	snprintf(client_data->sensor_name, SENSOR_NAME_SIZE, "%s_%x_%x",
		SENSOR_NAME, client->adapter->nr, client->addr);

	snprintf(client_data->sensor_name_feature, SENSOR_NAME_FEATURE_SIZE, "%s_%x_%x",
		SENSOR_NAME_FEATURE, client->adapter->nr, client->addr);

	return bma4xy_probe(client_data, &client->dev);
}

/**
 * @brief I2C probe function called by I2C bus driver.
 *
 * @param client[in,out] : The I2C client instance
 * @param id[in] : The I2C device ID instance
 *
 * @return the status
 * @retval 0 -> Success
 * @retval negative -> fail
 */
static int bma4xy_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret = 0;
	struct bma4xy_client_data *client_data = NULL;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "i2c_check_functionality error!");
		ret = -EIO;
	}

	ret = bma4_init_i2c_client(client_data, client);

	return ret;
}

/**
 * @brief Callback called when device is unbinded.
 * @param client[in,out] : Instance of I2C client device.
 *
 * @return the status
 * @retval 0 -> Success
 * @retval negative -> fail
 */
static int bma4xy_i2c_remove(struct i2c_client *client)
{
	int ret = 0;

	ret = bma4xy_remove(&client->dev);

	return ret;
}

static const struct i2c_device_id bma4xy_id[] = {
	{ SENSOR_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, bma4xy_id);
static const struct of_device_id bma4xy_of_match[] = {
	{ .compatible = "bma4xy", },
	{ }
};
MODULE_DEVICE_TABLE(i2c, bma4xy_of_match);

static struct i2c_driver bma4xy_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = SENSOR_NAME,
		.of_match_table = bma4xy_of_match,
	},
	.class = I2C_CLASS_HWMON,
	.id_table = bma4xy_id,
	.probe = bma4xy_i2c_probe,
	.remove = bma4xy_i2c_remove,

};

/**
 * @brief  I2C driver init function.
 *
 * @return the status
 * @retval 0 -> Success
 * @retval negative -> fail
 */
static int __init bma4xy_init(void)
{
	return i2c_add_driver(&bma4xy_driver);
}

/**
 * @brief  I2C driver exit function.
 */
static void __exit bma4xy_exit(void)
{
	i2c_del_driver(&bma4xy_driver);
}

MODULE_AUTHOR("contact@bosch-sensortec.com>");
MODULE_DESCRIPTION("BMA4XY SENSOR DRIVER");
MODULE_LICENSE("GPL v2");

module_init(bma4xy_init);
module_exit(bma4xy_exit);
/** @}*/
