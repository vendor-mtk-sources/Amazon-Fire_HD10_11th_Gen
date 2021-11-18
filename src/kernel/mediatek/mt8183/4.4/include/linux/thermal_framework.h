/*
 * Thermal Framework Driver
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 * Author: Dan Murphy <DMurphy@ti.com>
 *
 * Copyright (C) 2019-2020 Amazon Technologies Inc. All rights reserved.
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

#ifdef CONFIG_THERMAL_FOD
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/power_supply.h>
extern struct pinctrl *fod_vs_pctl;
#define fod_printk(fmt, args...) pr_debug("[FOD_INFO]" fmt, ##args)

#define FOD_VS_DEVICEID		3
#define PINCTRL_WPC_NTC		"switch_wpc_ntc"
#define PINCTRL_EMMC_NTC	"switch_emmc_ntc"
#define NTC_WPC			1
#define NTC_EMMC		0
#define TIME_TO_WORK		0
#define NUM_OF_COLLECT		5
#define TASK_DURATION		3000

struct vs_temp_data {
	int temperature;
	struct list_head list;
};

struct fod_algo {
	unsigned int work_delay;
	unsigned int window_size;
	unsigned int polling_interval;
	unsigned int wpc_online_status;
	unsigned int fod_temp_cnt;
	struct delayed_work wpc_event_wq;
	struct vs_temp_data fod_temp;
	struct list_head *cur_list;
	struct notifier_block psy_nb;
	struct power_supply *psy;
	struct mutex fod_lock;
};
#endif

#ifdef CONFIG_AMZN_THERMAL_VIRTUAL_SENSOR
struct vs_thermal_platform_data {
	int num_trips;
	enum thermal_device_mode mode;
	int polling_delay;
	struct list_head ts_list;
	struct thermal_zone_params tzp;
	struct trip_t trips[THERMAL_MAX_TRIPS];
	int num_cdevs;
	char cdevs[THERMAL_MAX_TRIPS][THERMAL_NAME_LENGTH];
};

struct vs_cooler_platform_data {
	char type[THERMAL_NAME_LENGTH];
	unsigned long state;
	unsigned long max_state;
	struct thermal_cooling_device *cdev;
	int level;
	int max_level;
	int thermal_cooler_id;
	int levels[THERMAL_MAX_TRIPS];
};
#endif

/**
 * struct virtual_sensor_params  - Structure for each virtual sensor params.
 * @alpha:  Moving average coefficient
 * @offset: Temperature offset
 * @weight: Weight
 * @select_device: Decide to register which thermalzone device
 * if defined(CONFIG_AMZN_THERMAL_VIRTUAL_SENSOR)
 * @thermal_sensor_id: Decide to get which thermalzone device temperature.
 * @aux_channel_num: the thermistor index
 */
struct thermal_dev_params {
	int offset;
	int alpha;
	int weight;
	int select_device;
#ifdef CONFIG_AMZN_THERMAL_VIRTUAL_SENSOR
	int thermal_sensor_id;
#endif
	int aux_channel_num;
};

/**
 * struct virtual_sensor_params  - Structure for each virtual sensor params.
 * @offset_name: The offset-node name in DTS
 * @offset_invert_name: The offset-invert-name in DTS
 * @alpha: The alpha-name in DTS
 * @weight: The alpha-name in DTS
 * @select_device: The weight-name in DTS
 * if defined(CONFIG_AMZN_THERMAL_VIRTUAL_SENSOR)
 * @thermal_sensor_id: The thermal_sensor_id name in DTS
 * @aux_channel_num: The aux-channel-num-name in DTS
 */
struct thermal_dev_node_names {
	char offset_name[NODE_NAME_MAX];
	char offset_invert_name[NODE_NAME_MAX];
	char alpha_name[NODE_NAME_MAX];
	char weight_name[NODE_NAME_MAX];
	char weight_invert_name[NODE_NAME_MAX];
	char select_device_name[NODE_NAME_MAX];
#ifdef CONFIG_AMZN_THERMAL_VIRTUAL_SENSOR
	char thermal_sensor_id[NODE_NAME_MAX];
#endif
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
	int first_data_get;
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
#ifdef CONFIG_AMZN_THERMAL_VIRTUAL_SENSOR
	struct vs_thermal_platform_data *pdata;
#else
	struct mtk_thermal_platform_data *pdata;
#endif

#if defined(CONFIG_AMAZON_METRICS_LOG) || defined(CONFIG_AMZN_METRICS_LOG)
	atomic_t query_count;
	unsigned int mask;
#endif
};

#ifdef CONFIG_AMZN_THERMAL_VIRTUAL_SENSOR
struct cooler_sort_list {
	struct vs_cooler_platform_data *pdata;
	struct list_head list;
	struct mutex update_mutex;
};

extern int thermal_level_compare(struct vs_cooler_platform_data *cooler_data,
			struct cooler_sort_list *head, bool positive_seq);

enum vs_thermal_sensor_id {
	VS_THERMAL_SENSOR_WMT = 0,
	VS_THERMAL_SENSOR_BATTERY,
	VS_THERMAL_SENSOR_GPU,
	VS_THERMAL_SENSOR_PMIC,
	VS_THERMAL_SENSOR_CPU,
	VS_THERMAL_SENSOR_THERMISTOR,
	VS_THERMAL_SENSOR_WIRELESS_CHG,
	VS_THERMAL_SENSOR_CHARGER,
	VS_THERMAL_SENSOR_COUNT
};

enum vs_thermal_cooler_id {
	VS_THERMAL_COOLER_BCCT = 0,
	VS_THERMAL_COOLER_BACKLIGHT,
	VS_THERMAL_COOLER_BUDGET,
	VS_THERMAL_COOLER_WIRELESS_CHG,
#ifdef CONFIG_THERMAL_FOD
	VS_THERMAL_COOLER_FOD_WIRELESS_CHG,
#endif
	VS_THERMAL_COOLER_COUNT
};

/* Get the current temperature of the thermal sensor. */
#ifdef CONFIG_THERMAL_FOD
int vs_thermal_sensor_get_temp(enum vs_thermal_sensor_id id, int index, int device);
#else
int vs_thermal_sensor_get_temp(enum vs_thermal_sensor_id id, int index);
#endif
/* Set a level limit via the thermal cooler. */
int vs_set_cooling_level(struct thermal_cooling_device *cdev,
	enum vs_thermal_cooler_id id, int level_limit);
/* Returns the temperature value of wpc in milli degrees Celsius. */
extern int vs_wpc_read_temp(void);
extern int init_vs_thermal_platform_data(void);
/* Set cpu power limit */
extern void set_cpu_power_limit(int power_limit);
extern int set_shutdown_enable_dcap(struct device *dev);

#else
struct cooler_sort_list{
	struct mtk_cooler_platform_data *pdata;
	struct list_head list;
};

extern int thermal_level_compare(struct mtk_cooler_platform_data *cooler_data,
		struct cooler_sort_list *head, bool positive_seq);
#endif

void last_kmsg_thermal_shutdown(void);

/**
 * API to register a temperature sensor with a thermal zone
 */
int thermal_dev_register(struct thermal_dev *tdev);
void thermal_parse_node_int(const struct device_node *np,
			const char *node_name, int *cust_val);
struct thermal_dev_params *thermal_sensor_dt_to_params(struct device *dev,
		struct thermal_dev_params *params,
		struct thermal_dev_node_names *name_params);
#ifdef CONFIG_AMZN_THERMAL_VIRTUAL_SENSOR
void cooler_init_cust_data_from_dt(struct platform_device *dev,
				struct vs_cooler_platform_data *pcdata);
#else
void cooler_init_cust_data_from_dt(struct platform_device *dev,
				struct mtk_cooler_platform_data *pcdata);
#endif
#endif
