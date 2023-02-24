/*
 * Thermal Framework Driver
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 * Author: Dan Murphy <DMurphy@ti.com>
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

#ifndef __LINUX_THERMAL_FRAMEWORK_H__
#define __LINUX_THERMAL_FRAMEWORK_H__

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/thermal.h>
#include <linux/platform_device.h>

#define NUM_COOLERS 10
#define NODE_NAME_MAX 40
#include <linux/seq_file.h>

struct thermal_dev;
struct cooling_device;

struct trip_t {
	unsigned long temp;
	enum thermal_trip_type type;
	unsigned long hyst;
#ifdef CONFIG_abe123
	struct cdev_t cdev[THERMAL_MAX_TRIPS];
#endif
};

struct mtk_thermal_platform_data {
	int num_trips;
	enum thermal_device_mode mode;
	int polling_delay;
	struct list_head ts_list;
	struct thermal_zone_params tzp;
	struct trip_t trips[THERMAL_MAX_TRIPS];
	int num_cdevs;
	char cdevs[THERMAL_MAX_TRIPS][THERMAL_NAME_LENGTH];
};

struct mtk_cooler_platform_data {
	char type[THERMAL_NAME_LENGTH];
	unsigned long state;
	unsigned long max_state;
	struct thermal_cooling_device *cdev;
	int level;
	int levels[THERMAL_MAX_TRIPS];
};

/**
 * struct virtual_sensor_params  - Structure for each virtual sensor params.
 * @alpha:  Moving average coefficient
 * @offset: Temperature offset
 * @weight: Weight
 * @select_device: Decide to register which thermalzone device
 * @aux_channel_num: the thermistor index
 */
struct thermal_dev_params {
	int offset;
	int alpha;
	int weight;
	int select_device;
	int aux_channel_num;
};

/**
 * struct virtual_sensor_params  - Structure for each virtual sensor params.
 * @offset_name: The offset-node name in DTS
 * @offset_invert_name: The offset-invert-name in DTS
 * @alpha: The alpha-name in DTS
 * @weight: The alpha-name in DTS
 * @select_device: The weight-name in DTS
 * @aux_channel_num: The aux-channel-num-name in DTS
 */
struct thermal_dev_node_names {
	char offset_name[NODE_NAME_MAX];
	char offset_invert_name[NODE_NAME_MAX];
	char alpha_name[NODE_NAME_MAX];
	char weight_name[NODE_NAME_MAX];
	char weight_invert_name[NODE_NAME_MAX];
	char select_device_name[NODE_NAME_MAX];
	char aux_channel_num_name[NODE_NAME_MAX];
};

/**
 * struct thermal_dev_ops  - Structure for device operation call backs
 * @get_temp: A temp sensor call back to get the current temperature.
 *		temp is reported in milli degrees.
 */
struct thermal_dev_ops {
	int (*get_temp) (struct thermal_dev *);
};

/**
 * struct thermal_dev  - Structure for each thermal device.
 * @name: The name of the device that is registering to the framework
 * @dev: Device node
 * @dev_ops: The device specific operations for the sensor, governor and cooling
 *           agents.
 * @node: The list node of the
 * @current_temp: The current temperature reported for the specific domain
 *
 */
struct thermal_dev {
	const char *name;
	struct device *dev;
	struct thermal_dev_ops *dev_ops;
	struct list_head node;
	struct thermal_dev_params *tdp;
	int current_temp;
	long off_temp;
};

/**
 * virtual_sensor_temp_sensor structure
 * @iclient - I2c client pointer
 * @dev - device pointer
 * @sensor_mutex - Mutex for sysfs, irq and PM
 * @therm_fw - thermal device
 */
struct virtual_sensor_temp_sensor {
	struct i2c_client *iclient;
	struct device *dev;
	struct mutex sensor_mutex;
	struct thermal_dev *therm_fw;
	u16 config_orig;
	u16 config_current;
	unsigned long last_update;
	int temp[3];
	int debug_temp;
};

struct virtual_sensor_thermal_zone {
	struct thermal_zone_device *tz;
	struct work_struct therm_work;
	struct mtk_thermal_platform_data *pdata;
#ifdef CONFIG_AMAZON_METRICS_LOG
	atomic_t query_count;
	unsigned int mask;
#endif
};

struct cooler_sort_list{
	struct mtk_cooler_platform_data *pdata;
	struct list_head list;
};

extern int thermal_level_compare(struct mtk_cooler_platform_data *cooler_data, struct cooler_sort_list *head, bool positive_seq);

void last_kmsg_thermal_shutdown(void);

/**
 * API to register a temperature sensor with a thermal zone
 */
int thermal_dev_register(struct thermal_dev *tdev);
void thermal_parse_node_int(const struct device_node *np,
			const char *node_name, int *cust_val);
struct thermal_dev_params *thermal_sensor_dt_to_params(struct device *dev,
			struct thermal_dev_params *params, struct thermal_dev_node_names *name_params);
void cooler_init_cust_data_from_dt(struct platform_device *dev,
				struct mtk_cooler_platform_data *pcdata);
#endif
