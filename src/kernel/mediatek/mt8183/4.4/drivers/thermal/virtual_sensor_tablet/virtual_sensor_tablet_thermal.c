/*
 * Copyright (C) 2020 Amazon.com, Inc.  All rights reserved.
 * Author: Akwasi Boateng <boatenga@lab126.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/atomic.h>
#include <linux/uaccess.h>
#include <linux/thermal.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/atomic.h>
#include <linux/list_sort.h>
#include <linux/thermal_framework.h>

#ifdef CONFIG_AMZN_SIGN_OF_LIFE
#include <linux/amzn_sign_of_life.h>
#endif

#ifdef CONFIG_AMZN_METRICS_LOG
#include <linux/amzn_metricslog.h>

#define VIRTUAL_SENSOR_METRICS_STR_LEN 128
/*
 * Define the metrics log printing interval.
 * If virtual sensor throttles, the interval
 * is 0x3F*3 seconds(3 means polling interval of virtual_sensor).
 * If doesn't throttle, the interval is 0x3FF*3 seconds.
 */
#define VIRTUAL_SENSOR_THROTTLE_TIME_MASK 0x3F
#define VIRTUAL_SENSOR_UNTHROTTLE_TIME_MASK 0x3FF
static unsigned long virtual_sensor_temp = 25000;
#endif

#include <thermal_core.h>

#define DRIVER_NAME "virtual_sensor-thermal"
#define THERMAL_NAME "virtual_sensor"
#define COMPATIBLE_NAME "amazon,virtual_sensor-thermal"
#define BUF_SIZE 128
#define DMF 1000
#define MASK (0x0FFF)
#define VIRTUAL_SENSOR_NUM_MAX 3

static DEFINE_MUTEX(therm_lock);
static unsigned int virtual_sensor_nums;

static struct vs_thermal_platform_data *virtual_sensor_thermal_data;

#ifdef CONFIG_THERMAL_FOD
#include "mt-plat/charger_type.h"
#include "mt-plat/charger_class.h"
#include <linux/of_platform.h>

#define FOD_THERMAL_NAME "virtual_sensor4"
struct pinctrl *fod_vs_pctl;
struct fod_algo *g_fod;

static struct timeval timer_start;
static struct timeval timer_now;
static long calculate_raising_rate(int data, int delay)
{
	int slope = 0;
	struct vs_temp_data *temp_add;
	struct vs_temp_data *temp_next;
	int temp_first, temp_last;
	int period = 0;

	fod_printk("%s\n", __func__);
	if (!delay)
		return slope;

	if (!g_fod->fod_temp_cnt) {
		if (!timer_start.tv_sec || !timer_start.tv_usec) {
			do_gettimeofday(&timer_start);
			return slope;
		} else {
			do_gettimeofday(&timer_now);
			period = timer_now.tv_sec - timer_start.tv_sec;
			fod_printk("wait %d sec to start\n", period);
			if (period < g_fod->work_delay)
				return slope;
		}
	}

	fod_printk("start to store temperature\n");

	temp_add = list_entry(g_fod->cur_list, struct vs_temp_data, list);
	temp_add->temperature = data;

	if (g_fod->fod_temp_cnt <= (g_fod->window_size-1)) {
		g_fod->cur_list = g_fod->cur_list->next;
		g_fod->fod_temp_cnt++;
		fod_printk("TEMP = %d, g_fod->fod_temp_cnt = %d\n", data, g_fod->fod_temp_cnt);
		return slope;
	}

	temp_last = data;
	g_fod->cur_list = g_fod->cur_list->next;
	temp_next = container_of(g_fod->cur_list, struct vs_temp_data, list);
	temp_first = temp_next->temperature;
	fod_printk("Count: %d, TEMP[fisrt, last] = [%d, %d]\n", (g_fod->fod_temp_cnt+1), temp_first, temp_last);
	return (temp_last - temp_first)*60/(int)(delay*(g_fod->window_size-1)/DMF);
}

static void wpc_online_work(struct work_struct *work)
{
	int i;
	struct thermal_zone_device *thermal;
	struct virtual_sensor_thermal_zone *tzone;
	struct vs_thermal_platform_data *pdata;
	struct vs_temp_data *temp_next;
	struct thermal_dev *tdev;

	thermal = thermal_zone_get_zone_by_name(FOD_THERMAL_NAME);
	if (IS_ERR(thermal) || !(thermal->devdata)) {
		pr_err("[FOD_INFO]: thermal:%p or thermal->devdata is NULL!\n", thermal);
		return ;
	}
	tzone = thermal->devdata;
	pdata = tzone->pdata;
	if (!pdata) {
		pr_err("[FOD_INFO]: tzone->pdata is NULL!\n");
		return ;
	}

	mutex_lock(&therm_lock);
	if (g_fod->wpc_online_status) {
		pdata->mode = THERMAL_DEVICE_ENABLED;
		mutex_lock(&tzone->tz->lock);
		if (!tzone->tz->polling_delay)
			tzone->tz->polling_delay = g_fod->polling_interval;
		for (i = 0; i < thermal->tzp->num_tbps; i++) {
			if (thermal->tzp->tbp[i].cdev) {
				if (!strcmp(thermal->tzp->tbp[i].cdev->type, "wpc_bcct1")) {
					struct vs_cooler_platform_data *pdata;
					pdata = thermal->tzp->tbp[i].cdev->devdata;
					pdata->state = 0;
					break;
				}
			}
		}
		mutex_unlock(&tzone->tz->lock);
		fod_printk("wpc is online, polling_delay = %d, vs mode = %d\n",
				thermal->polling_delay, pdata->mode);
		schedule_work(&tzone->therm_work);
	} else {
		pdata->mode = THERMAL_DEVICE_DISABLED;
		if (!g_fod->polling_interval)
			g_fod->polling_interval = tzone->tz->polling_delay;
		fod_printk("wpc is not online, polling_delay = %d, vs mode = %d\n",
				thermal->polling_delay, pdata->mode);
		mutex_lock(&tzone->tz->lock);
		cancel_delayed_work(&tzone->tz->poll_queue);
		tzone->tz->polling_delay = 0;
		mutex_unlock(&tzone->tz->lock);

		list_for_each_entry(tdev, &pdata->ts_list, node) {
			tdev->first_data_get = 0;
		}

		mutex_lock(&g_fod->fod_lock);
		temp_next = list_first_entry(&g_fod->fod_temp.list, struct vs_temp_data, list);
		g_fod->cur_list = &temp_next->list;
		memset(&timer_start, 0, sizeof(struct timeval));
		memset(&timer_now, 0, sizeof(struct timeval));
		if (g_fod->fod_temp_cnt)
			g_fod->fod_temp_cnt = 0;
		mutex_unlock(&g_fod->fod_lock);
	}
	mutex_unlock(&therm_lock);
}

static int wpc_online_event(struct notifier_block *nb, unsigned long event, void *v)
{
	struct power_supply *psy = v;
	union power_supply_propval propval;
	int ret;

	if (unlikely(system_state < SYSTEM_RUNNING))
		return NOTIFY_OK;

	if (event == PSY_EVENT_PROP_CHANGED && strcmp(psy->desc->name, "Wireless") == 0) {
		g_fod->psy = psy;
		ret = power_supply_get_property(g_fod->psy, POWER_SUPPLY_PROP_ONLINE, &propval);
		if (ret < 0) {
			pr_err("[FOD_INFO]: get psy online failed, ret = %d\n", __func__, ret);
			return NOTIFY_OK;
		} else {
			if (g_fod->wpc_online_status != propval.intval) {
				g_fod->wpc_online_status = propval.intval;
				queue_delayed_work(system_freezable_wq, &g_fod->wpc_event_wq, 0);
				pr_info("[FOD_INFO] %s, online statue = %d\n", __func__, g_fod->wpc_online_status);
			}
		}
	}

	return NOTIFY_OK;
}

#endif /* CONFIG_THERMAL_FOD is defined */

int init_vs_thermal_platform_data(void)
{
	int i, ret = 0;
	struct device_node *node;

	for_each_compatible_node(node, NULL, COMPATIBLE_NAME) {
		if (!of_device_is_available(node))
			continue;
		virtual_sensor_nums++;
	}
	pr_info("%s virtual_sensor_nums (%d)\n", __func__, virtual_sensor_nums);

	virtual_sensor_thermal_data = kcalloc(virtual_sensor_nums,
			sizeof(struct vs_thermal_platform_data), GFP_KERNEL);
	if (!virtual_sensor_thermal_data) {
		pr_err("%s: virtual_sensor_thermal_data failed to allocate memory!\n",
			__func__);
		ret = -ENOMEM;
		goto out;
	}

	for (i = 0; i < virtual_sensor_nums; i++)
		INIT_LIST_HEAD(&virtual_sensor_thermal_data[i].ts_list);
out:
	return ret;
}

#ifdef CONFIG_AMZN_METRICS_LOG
unsigned long get_virtualsensor_temp(void)
{
	return virtual_sensor_temp / 1000;
}
EXPORT_SYMBOL(get_virtualsensor_temp);
#endif

static int level_cmp(void *priv, struct list_head *a, struct list_head *b)
{
	struct cooler_sort_list *cooler_a =
	    container_of(a, struct cooler_sort_list, list);
	struct cooler_sort_list *cooler_b =
	    container_of(b, struct cooler_sort_list, list);
	int level;

	if (!cooler_a || !cooler_b || !(cooler_a->pdata)
		|| !(cooler_b->pdata)) {
		pr_err("%s cooler_a:%p, cooler_b:%p,"
			" cooler_a->pdata or cooler_b->pdata is NULL!\n",
			__func__, cooler_a, cooler_b);
		return -EINVAL;
	}

	level = cooler_a->pdata->level - cooler_b->pdata->level;

	return level;
}

int thermal_level_compare(struct vs_cooler_platform_data *cooler_data,
			  struct cooler_sort_list *head, bool positive_seq)
{
	struct cooler_sort_list *level_list;

	list_sort(NULL, &head->list, level_cmp);

	if (positive_seq)
		level_list =
		    list_entry(head->list.next, struct cooler_sort_list, list);
	else
		level_list =
		    list_entry(head->list.prev, struct cooler_sort_list, list);

	if (!level_list || !(level_list->pdata)) {
		pr_err("%s level_list:%p or level_list->pdata is NULL!\n", __func__, level_list);
		return -EINVAL;
	}
	return level_list->pdata->level;
}

#define PREFIX "thermalsensor:def"

static int match(struct thermal_zone_device *tz,
		 struct thermal_cooling_device *cdev)
{
	int i;
	struct virtual_sensor_thermal_zone *tzone;
	struct vs_thermal_platform_data *pdata;
	const struct thermal_zone_params *tzp;

	if (!tz || !(tz->devdata) || !(tz->tzp)) {
		pr_err("%s tz:%p maybe devdata or tzp is NULL!\n", __func__, tz);
		return -EINVAL;
	}
	tzp = tz->tzp;
	tzone = tz->devdata;
	pdata = tzone->pdata;
	if (!pdata) {
		pr_err("%s pdata is NULL!\n", __func__);
		return -EINVAL;
	}
	if (strncmp(tz->type, "virtual_sensor", strlen("virtual_sensor")))
		return -EINVAL;

	for (i = 0; i < tzp->num_tbps; i++) {
		if (tzp->tbp[i].cdev) {
			if (!strcmp(tzp->tbp[i].cdev->type, cdev->type))
				return -EEXIST;
		}
	}

	for (i = 0; i < pdata->num_cdevs; i++) {
		pr_debug("pdata->cdevs[%d] %s cdev->type %s\n", i,
			 pdata->cdevs[i], cdev->type);
		if (!strncmp(pdata->cdevs[i], cdev->type, strlen(cdev->type)))
			return 0;
	}

	return -EINVAL;
}

static int virtual_sensor_thermal_get_temp(struct thermal_zone_device *thermal,
					   int *t)
{
	struct thermal_dev *tdev;
	struct virtual_sensor_thermal_zone *tzone;
	struct vs_thermal_platform_data *pdata;
	long temp = 0;
	long tempv = 0;
	int alpha, offset, weight;
#ifdef CONFIG_THERMAL_FOD
	int wpc_ntc_temp = 0;
#endif
#ifdef CONFIG_AMZN_METRICS_LOG
	char buf[VIRTUAL_SENSOR_METRICS_STR_LEN];
#endif

	if (!thermal || !(thermal->devdata)) {
		pr_err("%s thermal:%p or thermal->devdata is NULL!\n", __func__, thermal);
		return -EINVAL;
	}
	tzone = thermal->devdata;
	pdata = tzone->pdata;
	if (!pdata) {
		pr_err("%s pdata is NULL!\n", __func__);
		return -EINVAL;
	}

#ifdef CONFIG_THERMAL_FOD
	if (!strncmp(thermal->type, FOD_THERMAL_NAME, strlen(FOD_THERMAL_NAME))) {
		if (strncmp(current->comm, "kworker", strlen("kworker")))
			return -EINVAL;
	}
#endif

#ifdef CONFIG_AMZN_METRICS_LOG
	atomic_inc(&tzone->query_count);
#endif

	list_for_each_entry(tdev, &pdata->ts_list, node) {
		temp = tdev->dev_ops->get_temp(tdev);
#ifdef CONFIG_AMZN_METRICS_LOG
		/*
		 * Log in metrics around every 1 hour normally
		 * and 3 mins wheny throttling
		 */
		if (!(atomic_read(&tzone->query_count) & tzone->mask)) {
			snprintf(buf, VIRTUAL_SENSOR_METRICS_STR_LEN,
				 "%s:%s_%s_temp=%ld;CT;1:NR",
				 PREFIX, thermal->type, tdev->name, temp);
			log_to_metrics(ANDROID_LOG_INFO, "ThermalEvent", buf);
		}
#endif
#ifdef CONFIG_THERMAL_FOD
		if (!strncmp(thermal->type, FOD_THERMAL_NAME, strlen(FOD_THERMAL_NAME))
				&& tdev->tdp->weight) {
			fod_printk("[%s, %d]\n", tdev->name, temp);
			wpc_ntc_temp = temp;
			mutex_lock(&g_fod->fod_lock);
			temp = calculate_raising_rate(temp, thermal->polling_delay);
			mutex_unlock(&g_fod->fod_lock);
			fod_printk("%s_slope = %d\n", tdev->name, temp);
		}
#endif
		alpha = tdev->tdp->alpha;
		offset = tdev->tdp->offset;
		weight = tdev->tdp->weight;
		if (!tdev->first_data_get) {
			tdev->off_temp = temp - offset;
			tdev->first_data_get = 1;
		}
		else {
			tdev->off_temp = alpha * (temp - offset) +
			    (DMF - alpha) * tdev->off_temp;
			tdev->off_temp /= DMF;
		}

		tempv += (weight * tdev->off_temp) / DMF;
#ifdef CONFIG_THERMAL_FOD
		if (!strncmp(thermal->type, FOD_THERMAL_NAME, strlen(FOD_THERMAL_NAME))
				&& tdev->tdp->weight) {
			fod_printk("[%s, %s_slope, VS4_temp] = [%d, %d, %d]\n",
					tdev->name, tdev->name, wpc_ntc_temp, temp, tempv);
		}
#endif
	}

#ifdef CONFIG_AMZN_METRICS_LOG
	/*
	 * Log in metrics around every 1 hour normally
	 * and 3 mins wheny throttling
	 */
	if (!(atomic_read(&tzone->query_count) & tzone->mask)) {
		snprintf(buf, VIRTUAL_SENSOR_METRICS_STR_LEN,
			 "%s:%s_temp=%ld;CT;1:NR",
			 PREFIX, thermal->type, tempv);
		log_to_metrics(ANDROID_LOG_INFO, "ThermalEvent", buf);
	}

	if (tempv > pdata->trips[0].temp)
		tzone->mask = VIRTUAL_SENSOR_THROTTLE_TIME_MASK;
	else
		tzone->mask = VIRTUAL_SENSOR_UNTHROTTLE_TIME_MASK;
#endif

	*t = (unsigned long)tempv;
#ifdef CONFIG_AMZN_METRICS_LOG
	virtual_sensor_temp = (unsigned long)tempv;
#endif

	return 0;
}

static int virtual_sensor_thermal_get_mode(struct thermal_zone_device *thermal,
					   enum thermal_device_mode *mode)
{
	struct virtual_sensor_thermal_zone *tzone;
	struct vs_thermal_platform_data *pdata;

	if (!thermal || !(thermal->devdata)) {
		pr_err("%s thermal:%p or thermal->devdata is NULL!\n", __func__, thermal);
		return -EINVAL;
	}
	tzone = thermal->devdata;
	pdata = tzone->pdata;
	if (!pdata) {
		pr_err("%s pdata is NULL!\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&therm_lock);
	*mode = pdata->mode;
	mutex_unlock(&therm_lock);
	return 0;
}

static int virtual_sensor_thermal_set_mode(struct thermal_zone_device *thermal,
					   enum thermal_device_mode mode)
{
	struct virtual_sensor_thermal_zone *tzone;
	struct vs_thermal_platform_data *pdata;
	struct thermal_dev *tdev;

	if (!thermal || !(thermal->devdata)) {
		pr_err("%s thermal:%p or thermal->devdata is NULL!\n", __func__, thermal);
		return -EINVAL;
	}
	tzone = thermal->devdata;
	pdata = tzone->pdata;
	if (!pdata) {
		pr_err("%s pdata is NULL!\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&therm_lock);
#ifdef CONFIG_THERMAL_FOD
	if (!strncmp(thermal->type, FOD_THERMAL_NAME, strlen(FOD_THERMAL_NAME))) {
		if (!g_fod->wpc_online_status)
			mode = THERMAL_DEVICE_DISABLED;
	}
#endif
	pdata->mode = mode;
	if (mode == THERMAL_DEVICE_DISABLED) {
		mutex_lock(&tzone->tz->lock);
		tzone->tz->polling_delay = 0;
		mutex_unlock(&tzone->tz->lock);
		thermal_zone_device_update(tzone->tz);
		list_for_each_entry(tdev, &pdata->ts_list, node) {
			tdev->first_data_get = 0;
		}
		mutex_unlock(&therm_lock);
		return 0;
	}
	schedule_work(&tzone->therm_work);
	mutex_unlock(&therm_lock);
	return 0;
}

static int virtual_sensor_thermal_get_trip_type(struct thermal_zone_device
						*thermal, int trip,
						enum thermal_trip_type *type)
{
	struct virtual_sensor_thermal_zone *tzone;
	struct vs_thermal_platform_data *pdata;

	if (!thermal || !(thermal->devdata)) {
		pr_err("%s thermal:%p or thermal->devdata is NULL!\n", __func__, thermal);
		return -EINVAL;
	}
	tzone = thermal->devdata;
	pdata = tzone->pdata;
	if (!pdata) {
		pr_err("%s pdata is NULL!\n", __func__);
		return -EINVAL;
	}

	*type = pdata->trips[trip].type;
	return 0;
}

static int virtual_sensor_thermal_get_trip_temp(struct thermal_zone_device
						*thermal, int trip, int *temp)
{
	struct virtual_sensor_thermal_zone *tzone;
	struct vs_thermal_platform_data *pdata;

	if (!thermal || !(thermal->devdata)) {
		pr_err("%s thermal:%p or thermal->devdata is NULL!\n", __func__, thermal);
		return -EINVAL;
	}
	tzone = thermal->devdata;
	pdata = tzone->pdata;
	if (!pdata) {
		pr_err("%s pdata is NULL!\n", __func__);
		return -EINVAL;
	}

	*temp = pdata->trips[trip].temp;
	return 0;
}

static int virtual_sensor_thermal_set_trip_temp(struct thermal_zone_device
						*thermal, int trip, int temp)
{
	struct virtual_sensor_thermal_zone *tzone;
	struct vs_thermal_platform_data *pdata;

	if (!thermal || !(thermal->devdata)) {
		pr_err("%s thermal:%p or thermal->devdata is NULL!\n", __func__, thermal);
		return -EINVAL;
	}
	tzone = thermal->devdata;
	pdata = tzone->pdata;
	if (!pdata) {
		pr_err("%s pdata is NULL!\n", __func__);
		return -EINVAL;
	}

	pdata->trips[trip].temp = temp;
	return 0;
}

static int virtual_sensor_thermal_get_crit_temp(struct thermal_zone_device
						*thermal, int *temp)
{
	int i;
	struct virtual_sensor_thermal_zone *tzone;
	struct vs_thermal_platform_data *pdata;

	if (!thermal || !(thermal->devdata)) {
		pr_err("%s thermal:%p or thermal->devdata is NULL!\n", __func__, thermal);
		return -EINVAL;
	}
	tzone = thermal->devdata;
	pdata = tzone->pdata;
	if (!pdata) {
		pr_err("%s pdata is NULL!\n", __func__);
		return -EINVAL;
	}

	for (i = 0; i < THERMAL_MAX_TRIPS; i++) {
		if (pdata->trips[i].type == THERMAL_TRIP_CRITICAL) {
			*temp = pdata->trips[i].temp;
			return 0;
		}
	}
	return -EINVAL;
}

static int virtual_sensor_thermal_get_trip_hyst(struct thermal_zone_device
						*thermal, int trip, int *hyst)
{
	struct virtual_sensor_thermal_zone *tzone;
	struct vs_thermal_platform_data *pdata;

	if (!thermal || !(thermal->devdata)) {
		pr_err("%s thermal:%p or thermal->devdata is NULL!\n", __func__, thermal);
		return -EINVAL;
	}
	tzone = thermal->devdata;
	pdata = tzone->pdata;
	if (!pdata) {
		pr_err("%s pdata is NULL!\n", __func__);
		return -EINVAL;
	}

	*hyst = pdata->trips[trip].hyst;
	return 0;
}

static int virtual_sensor_thermal_set_trip_hyst(struct thermal_zone_device
						*thermal, int trip, int hyst)
{
	struct virtual_sensor_thermal_zone *tzone;
	struct vs_thermal_platform_data *pdata;

	if (!thermal || !(thermal->devdata)) {
		pr_err("%s thermal:%p or thermal->devdata is NULL!\n", __func__, thermal);
		return -EINVAL;
	}
	tzone = thermal->devdata;
	pdata = tzone->pdata;
	if (!pdata) {
		pr_err("%s pdata is NULL!\n", __func__);
		return -EINVAL;
	}

	pdata->trips[trip].hyst = hyst;
	return 0;
}

#ifdef CONFIG_THERMAL_SHUTDOWN_LAST_KMESG
void last_kmsg_thermal_shutdown(void)
{
	int rc;
	char *argv[] = {
		"/sbin/crashreport",
		"thermal_shutdown",
		NULL
	};

	pr_err("%s: start to save last kmsg\n", __func__);
	/* UMH_WAIT_PROC UMH_WAIT_EXEC */
	rc = call_usermodehelper(argv[0], argv, NULL, UMH_WAIT_EXEC);
	pr_err("%s: save last kmsg finish\n", __func__);

	if (rc < 0)
		pr_err("call /sbin/crashreport failed, rc = %d\n", rc);
	else
		msleep(6000);	/* 6000ms */
}
EXPORT_SYMBOL_GPL(last_kmsg_thermal_shutdown);
#endif

static int virtual_sensor_thermal_notify(struct thermal_zone_device *thermal,
					 int trip, enum thermal_trip_type type)
{
	char data[20];
	char *envp[] = { data, NULL };
	int trip_temp;

	if (!thermal) {
		pr_err("%s thermal:%p is NULL!\n", __func__, thermal);
		return -EINVAL;
	}

	snprintf(data, sizeof(data), "%s", "SHUTDOWN_WARNING");
	kobject_uevent_env(&thermal->device.kobj, KOBJ_CHANGE, envp);

#if defined(CONFIG_AMAZON_SIGN_OF_LIFE) || defined(CONFIG_AMZN_SIGN_OF_LIFE)
	if (type == THERMAL_TRIP_CRITICAL) {
		virtual_sensor_thermal_get_trip_temp(thermal, trip, &trip_temp);
		pr_err("[%s][%s]type:[%s] Thermal shutdown virtual sensor, current temp=%d, trip=%d, trip_temp=%d\n",
			__func__, dev_name(&thermal->device), thermal->type,
			thermal->temperature, trip, trip_temp);
		life_cycle_set_thermal_shutdown_reason
			(THERMAL_SHUTDOWN_REASON_PCB);
	}
#endif

#ifdef CONFIG_THERMAL_SHUTDOWN_LAST_KMESG
	if (type == THERMAL_TRIP_CRITICAL) {
		pr_err("%s: thermal_shutdown notify\n", __func__);
		last_kmsg_thermal_shutdown();
		pr_err("%s: thermal_shutdown notify end\n", __func__);
	}
#endif

	if (type == THERMAL_TRIP_CRITICAL)
		set_shutdown_enable_dcap(&thermal->device);

	return 0;
}

static struct thermal_zone_device_ops virtual_sensor_tz_dev_ops = {
	.get_temp = virtual_sensor_thermal_get_temp,
	.get_mode = virtual_sensor_thermal_get_mode,
	.set_mode = virtual_sensor_thermal_set_mode,
	.get_trip_type = virtual_sensor_thermal_get_trip_type,
	.get_trip_temp = virtual_sensor_thermal_get_trip_temp,
	.set_trip_temp = virtual_sensor_thermal_set_trip_temp,
	.get_crit_temp = virtual_sensor_thermal_get_crit_temp,
	.get_trip_hyst = virtual_sensor_thermal_get_trip_hyst,
	.set_trip_hyst = virtual_sensor_thermal_set_trip_hyst,
	.notify = virtual_sensor_thermal_notify,
};

static ssize_t params_show(struct device *dev,
			   struct device_attribute *devattr, char *buf)
{
	int o = 0;
	int a = 0;
	int w = 0;
	char pbufo[BUF_SIZE];
	char pbufa[BUF_SIZE];
	char pbufw[BUF_SIZE];
	int alpha, offset, weight;
	struct thermal_dev *tdev;
	struct thermal_zone_device *thermal =
	    container_of(dev, struct thermal_zone_device, device);
	struct virtual_sensor_thermal_zone *tzone;
	struct vs_thermal_platform_data *pdata;

	if (!thermal || !(thermal->devdata)) {
		pr_err("%s thermal:%p or thermal->devdata is NULL!\n", __func__, thermal);
		return -EINVAL;
	}
	tzone = thermal->devdata;
	pdata = tzone->pdata;
	if (!pdata) {
		pr_err("%s pdata is NULL!\n", __func__);
		return -EINVAL;
	}

	o += scnprintf(pbufo + o, BUF_SIZE - o, "offsets ");
	a += scnprintf(pbufa + a, BUF_SIZE - a, "alphas ");
	w += scnprintf(pbufw + w, BUF_SIZE - w, "weights ");

	list_for_each_entry(tdev, &pdata->ts_list, node) {
		alpha = tdev->tdp->alpha;
		offset = tdev->tdp->offset;
		weight = tdev->tdp->weight;

		o += scnprintf(pbufo + o, BUF_SIZE - o, "%d ", offset);
		a += scnprintf(pbufa + a, BUF_SIZE - a, "%d ", alpha);
		w += scnprintf(pbufw + w, BUF_SIZE - w, "%d ", weight);
	}
	return scnprintf(buf, 3 * BUF_SIZE + 6, "%s\n%s\n%s\n", pbufo, pbufa, pbufw);
}

static ssize_t trips_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct thermal_zone_device *thermal =
	    container_of(dev, struct thermal_zone_device, device);

	if (!thermal) {
		pr_err("%s thermal:%p is NULL!\n", __func__, thermal);
		return -EINVAL;
	}

	return scnprintf(buf, BUF_SIZE, "%d\n", thermal->trips);
}

static ssize_t trips_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	int trips = 0;
	struct thermal_zone_device *thermal =
	    container_of(dev, struct thermal_zone_device, device);
	struct virtual_sensor_thermal_zone *tzone;
	struct vs_thermal_platform_data *pdata;

	if (!thermal || !(thermal->devdata)) {
		pr_err("%s thermal:%p or thermal->devdata is NULL!\n", __func__, thermal);
		return -EINVAL;
	}
	tzone = thermal->devdata;
	pdata = tzone->pdata;
	if (!pdata) {
		pr_err("%s pdata is NULL!\n", __func__);
		return -EINVAL;
	}

	if (sscanf(buf, "%d\n", &trips) != 1)
		return -EINVAL;
	if (trips < 0)
		return -EINVAL;

	pdata->num_trips = trips;
	thermal->trips = pdata->num_trips;
	return count;
}

static ssize_t polling_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct thermal_zone_device *thermal =
	    container_of(dev, struct thermal_zone_device, device);

	if (!thermal) {
		pr_err("%s thermal:%p is NULL!\n", __func__, thermal);
		return -EINVAL;
	}

	return scnprintf(buf, BUF_SIZE, "%d\n", thermal->polling_delay);
}

static ssize_t polling_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	int polling_delay = 0;
	struct thermal_zone_device *thermal =
	    container_of(dev, struct thermal_zone_device, device);
	struct virtual_sensor_thermal_zone *tzone;
	struct vs_thermal_platform_data *pdata;
#ifdef CONFIG_THERMAL_FOD
	bool wpc_online = false;
#endif

	if (!thermal || !(thermal->devdata)) {
		pr_err("%s thermal:%p or thermal->devdata is NULL!\n", __func__, thermal);
		return -EINVAL;
	}
	tzone = thermal->devdata;
	pdata = tzone->pdata;
	if (!pdata) {
		pr_err("%s pdata is NULL!\n", __func__);
		return -EINVAL;
	}

	if (sscanf(buf, "%d\n", &polling_delay) != 1)
		return -EINVAL;
	if (polling_delay < 0)
		return -EINVAL;

	mutex_lock(&therm_lock);
	pdata->polling_delay = polling_delay;
	thermal->polling_delay = pdata->polling_delay;
#ifdef CONFIG_THERMAL_FOD
	if (!strncmp(thermal->type, FOD_THERMAL_NAME, strlen(FOD_THERMAL_NAME))) {
		g_fod->polling_interval = polling_delay;
		wireless_charger_dev_get_online(get_charger_by_name("wireless_chg"), &wpc_online);
		if (wpc_online)
			thermal_zone_device_update(thermal);
	} else
#endif
		thermal_zone_device_update(thermal);
	mutex_unlock(&therm_lock);
	return count;
}

#ifdef CONFIG_THERMAL_FOD
static ssize_t fod_delay_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	return scnprintf(buf, BUF_SIZE, "%d\n", g_fod->work_delay);
}

static ssize_t fod_delay_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned int delay = 0;

	if (kstrtouint(buf, 0, &delay))
		return -EINVAL;

	g_fod->work_delay = delay;

	return count;
}

static ssize_t window_size_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	return scnprintf(buf, BUF_SIZE, "%d\n", g_fod->window_size);
}

static ssize_t window_size_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned int size = 0;
	struct vs_temp_data *temp_data, *ptr;
	int i;

	if (kstrtouint(buf, 0, &size))
		return -EINVAL;

	if (size < 2) {
		pr_err("%s Error setting value is less than 2\n");
		return -EINVAL;
	}

	mutex_lock(&g_fod->fod_lock);
	list_for_each_entry_safe(temp_data, ptr, &g_fod->fod_temp.list, list) {
		list_del(&temp_data->list);
		kfree(temp_data);
	}
	g_fod->window_size = size;

	g_fod->cur_list = &g_fod->fod_temp.list;
	for (i = 1; i < g_fod->window_size; i++) {
		ptr = kzalloc(sizeof(struct vs_temp_data), GFP_KERNEL);
		if (!ptr) {
			pr_err("%s Failed to allocate vs_temp_data memory\n",
				__func__);
			return -ENOMEM;
		}
		list_add_tail(&ptr->list, &g_fod->fod_temp.list);
	}
	g_fod->fod_temp_cnt = 0;
	mutex_unlock(&g_fod->fod_lock);
	return count;
}

static DEVICE_ATTR(fod_delay, 0644, fod_delay_show, fod_delay_store);
static DEVICE_ATTR(fod_window_size, 0644, window_size_show, window_size_store);
#endif
static DEVICE_ATTR(trips, 0644, trips_show, trips_store);
static DEVICE_ATTR(polling, 0644, polling_show, polling_store);
static DEVICE_ATTR(params, 0444, params_show, NULL);

static int virtual_sensor_create_sysfs(struct virtual_sensor_thermal_zone
				       *tzone)
{
	int ret = 0;
#ifdef CONFIG_THERMAL_FOD
	struct thermal_zone_device *tz;
#endif

	ret = device_create_file(&tzone->tz->device, &dev_attr_params);
	if (ret)
		pr_err("%s Failed to create polling attr\n", __func__);
	ret = device_create_file(&tzone->tz->device, &dev_attr_polling);
	if (ret)
		pr_err("%s Failed to create polling attr\n", __func__);
	ret = device_create_file(&tzone->tz->device, &dev_attr_trips);
	if (ret)
		pr_err("%s Failed to create trips attr\n", __func__);
#ifdef CONFIG_THERMAL_FOD
	if (get_charger_by_name("wireless_chg") != NULL) {
		tz = thermal_zone_get_zone_by_name(FOD_THERMAL_NAME);
		if (IS_ERR(tz)) {
			pr_err("[FOD_INFO]: thermal:%p  is NULL!\n", tz);
			tz = NULL;
		} else {
			ret = device_create_file(&tzone->tz->device, &dev_attr_fod_delay);
			if (ret)
				pr_err("%s Failed to create fod_delay attr\n", __func__);
			ret = device_create_file(&tzone->tz->device, &dev_attr_fod_window_size);
			if (ret)
				pr_err("%s Failed to create fod_window_size attr\n", __func__);
		}
	}
#endif
	return ret;
}

static void virtual_sensor_thermal_work(struct work_struct *work)
{
	struct virtual_sensor_thermal_zone *tzone;
	struct vs_thermal_platform_data *pdata;

	mutex_lock(&therm_lock);
	tzone =
	    container_of(work, struct virtual_sensor_thermal_zone, therm_work);
	if (!tzone)
		return;
	pdata = tzone->pdata;
	if (!pdata)
		return;
	if (pdata->mode == THERMAL_DEVICE_ENABLED)
		thermal_zone_device_update(tzone->tz);
	mutex_unlock(&therm_lock);
}

void thermal_parse_node_int(const struct device_node *np,
			    const char *node_name, int *cust_val)
{
	u32 val = 0;

	if (of_property_read_u32(np, node_name, &val) == 0) {
		(*cust_val) = (int)val;
		pr_debug("Get %s %d\n", node_name, *cust_val);
	} else
		pr_notice("Get %s failed\n", node_name);
}
EXPORT_SYMBOL(thermal_parse_node_int);

struct thermal_dev_params *thermal_sensor_dt_to_params(struct device *dev,
						       struct thermal_dev_params
						       *params,
						       struct
						       thermal_dev_node_names
						       *name_params)
{
	struct device_node *np = dev->of_node;
	int offset_invert = 0;
	int weight_invert = 0;

	if (!params || !name_params) {
		dev_err(dev, "the params or name_params is NULL\n");
		return NULL;
	}

	thermal_parse_node_int(np, name_params->offset_name, &params->offset);
	thermal_parse_node_int(np, name_params->alpha_name, &params->alpha);
	thermal_parse_node_int(np, name_params->weight_name, &params->weight);
	thermal_parse_node_int(np, name_params->select_device_name,
			       &params->select_device);
	thermal_parse_node_int(np, name_params->thermal_sensor_id,
				&params->thermal_sensor_id);

	if (*name_params->offset_invert_name)
		thermal_parse_node_int(np, name_params->offset_invert_name,
				       &offset_invert);

	if (offset_invert)
		params->offset = 0 - params->offset;

	if (*name_params->weight_invert_name)
		thermal_parse_node_int(np, name_params->weight_invert_name,
				       &weight_invert);

	if (weight_invert)
		params->weight = 0 - params->weight;

	if (*name_params->aux_channel_num_name)
		thermal_parse_node_int(np, name_params->aux_channel_num_name,
				       &params->aux_channel_num);

	return params;
}
EXPORT_SYMBOL(thermal_sensor_dt_to_params);

static void virtual_sensor_thermal_parse_node_int_array(const struct device_node
							*np,
							const char *node_name,
							unsigned long
							*tripsdata)
{
	u32 array[THERMAL_MAX_TRIPS] = { 0 };
	int i = 0;

	if (of_property_read_u32_array(np, node_name, array, ARRAY_SIZE(array))
	    == 0) {
		for (i = 0; i < ARRAY_SIZE(array); i++) {
			tripsdata[i] = (unsigned long)array[i];
			pr_debug("Get %s %ld\n", node_name, tripsdata[i]);
		}
	} else
		pr_notice("Get %s failed\n", node_name);
}

static void cooler_parse_node_int_array(const struct device_node *np,
			const char *node_name, int *tripsdata)
{
	u32 array[THERMAL_MAX_TRIPS] = {0};
	int i = 0;

	if (of_property_read_u32_array(np, node_name, array, ARRAY_SIZE(array)) == 0) {
		for (i = 0; i < ARRAY_SIZE(array); i++) {
			tripsdata[i] = array[i];
			pr_debug("Get %s %d\n", node_name, tripsdata[i]);
		}
	} else
		pr_notice("Get %s failed\n", node_name);
}

static void virtual_sensor_thermal_tripsdata_convert(char *type,
						     unsigned long *tripsdata,
						     struct trip_t *trips)
{
	int i = 0;

	if (strncmp(type, "type", 4) == 0) {
		for (i = 0; i < THERMAL_MAX_TRIPS; i++) {
			switch (tripsdata[i]) {
			case 0:
				trips[i].type = THERMAL_TRIP_ACTIVE;
				break;
			case 1:
				trips[i].type = THERMAL_TRIP_PASSIVE;
				break;
			case 2:
				trips[i].type = THERMAL_TRIP_HOT;
				break;
			case 3:
				trips[i].type = THERMAL_TRIP_CRITICAL;
				break;
			default:
				pr_notice("unknown type!\n");
			}
			pr_debug("tripsdata[%d]=%ld\n", i, tripsdata[i]);
		}
	} else if (strncmp(type, "temp", 4) == 0) {
		for (i = 0; i < THERMAL_MAX_TRIPS; i++) {
			trips[i].temp = tripsdata[i];
			pr_debug("tripsdata[%d]=%ld\n", i, tripsdata[i]);
		}
	} else {
		for (i = 0; i < THERMAL_MAX_TRIPS; i++) {
			trips[i].hyst = tripsdata[i];
			pr_debug("tripsdata[%d]=%ld\n", i, tripsdata[i]);
		}
	}
}

static void virtual_sensor_thermal_init_trips(const struct device_node *np,
					      struct trip_t *trips)
{
	unsigned long tripsdata[THERMAL_MAX_TRIPS] = { 0 };

	virtual_sensor_thermal_parse_node_int_array(np, "temp", tripsdata);
	virtual_sensor_thermal_tripsdata_convert("temp", tripsdata, trips);
	virtual_sensor_thermal_parse_node_int_array(np, "type", tripsdata);
	virtual_sensor_thermal_tripsdata_convert("type", tripsdata, trips);
	virtual_sensor_thermal_parse_node_int_array(np, "hyst", tripsdata);
	virtual_sensor_thermal_tripsdata_convert("hyst", tripsdata, trips);
}

static void virtual_sensor_thermal_parse_node_string_index(struct device_node *np,
							   const char *node_name,
							   int index,
							   char *cust_string)
{
	const char *string;

	if (of_property_read_string_index(np, node_name, index, &string) == 0) {
		strncpy(cust_string, string, strlen(string));
		pr_debug("Get %s %s\n", node_name, cust_string);
	} else
		pr_notice("Get %s failed\n", node_name);
}

static int virtual_sensor_thermal_init_tbp(struct thermal_zone_params *tzp,
					   struct platform_device *pdev)
{
	int i;
	struct thermal_bind_params *tbp;

	tbp =
	    devm_kzalloc(&pdev->dev,
			 sizeof(struct thermal_bind_params) * tzp->num_tbps,
			 GFP_KERNEL);
	if (!tbp)
		return -ENOMEM;

	for (i = 0; i < tzp->num_tbps; i++) {
		tbp[i].cdev = NULL;
		tbp[i].weight = 0;
		tbp[i].trip_mask = MASK;
		tbp[i].match = match;
	}
	tzp->tbp = tbp;

	return 0;
}

void cooler_init_cust_data_from_dt(struct platform_device *dev,
				struct vs_cooler_platform_data *pcdata)
{
	struct device_node *np = dev->dev.of_node;

	virtual_sensor_thermal_parse_node_string_index(np, "type",
						0, (char *)&pcdata->type);
	thermal_parse_node_int(np, "state", (int *)&pcdata->state);
	thermal_parse_node_int(np, "max_state",
					(int *)&pcdata->max_state);
	thermal_parse_node_int(np, "level", &pcdata->level);
	thermal_parse_node_int(np, "max_level", &pcdata->max_level);
	thermal_parse_node_int(np, "thermal_cooler_id",
					&pcdata->thermal_cooler_id);
	cooler_parse_node_int_array(np, "levels", pcdata->levels);
}

static int virtual_sensor_thermal_init_cust_data_from_dt(struct platform_device
							 *dev)
{
	int ret = 0;
	struct device_node *np = dev->dev.of_node;
	struct vs_thermal_platform_data *p_virtual_sensor_thermal_data;
	int i = 0;

	dev->id = 0;
	thermal_parse_node_int(np, "dev_id", &dev->id);
	if (dev->id < virtual_sensor_nums) {
		p_virtual_sensor_thermal_data =
		    &virtual_sensor_thermal_data[dev->id];
#ifdef CONFIG_THERMAL_FOD
		if (dev->id == FOD_VS_DEVICEID) {
			fod_vs_pctl = devm_pinctrl_get(&dev->dev);
			if (IS_ERR(fod_vs_pctl)) {
				pr_err("%s: Cannot find pinctrl for FOD VS\n", __func__);
				return -ENODEV;
			}
		}
#endif
	} else {
		ret = -1;
		pr_err("dev->id invalid!\n");
		return ret;
	}

	thermal_parse_node_int(np, "num_trips",
			       &p_virtual_sensor_thermal_data->num_trips);
	thermal_parse_node_int(np, "mode",
			       (int *)&p_virtual_sensor_thermal_data->mode);
	thermal_parse_node_int(np, "polling_delay",
			       &p_virtual_sensor_thermal_data->polling_delay);
	virtual_sensor_thermal_parse_node_string_index(np, "governor_name", 0,
						       p_virtual_sensor_thermal_data->tzp.governor_name);
	thermal_parse_node_int(np, "num_tbps",
			       &p_virtual_sensor_thermal_data->tzp.num_tbps);
	virtual_sensor_thermal_init_trips(np,
					  p_virtual_sensor_thermal_data->trips);

	thermal_parse_node_int(np, "num_cdevs",
			       &p_virtual_sensor_thermal_data->num_cdevs);
	if (p_virtual_sensor_thermal_data->num_cdevs > THERMAL_MAX_TRIPS)
		p_virtual_sensor_thermal_data->num_cdevs = THERMAL_MAX_TRIPS;

	while (i < p_virtual_sensor_thermal_data->num_cdevs) {
		virtual_sensor_thermal_parse_node_string_index(np, "cdev_names",
							       i,
							       p_virtual_sensor_thermal_data->cdevs[i]);
		i++;
	}

	ret =
	    virtual_sensor_thermal_init_tbp(&p_virtual_sensor_thermal_data->tzp,
					    dev);

	return ret;
}

#ifdef DEBUG
static void test_data(void)
{
	int i = 0;
	int j = 0;
	int h = 0;

	for (i = 0; i < virtual_sensor_nums; i++) {
		pr_debug("num_trips %d\n",
			 virtual_sensor_thermal_data[i].num_trips);
		pr_debug("mode %d\n", virtual_sensor_thermal_data[i].mode);
		pr_debug("polling_delay %d\n",
			 virtual_sensor_thermal_data[i].polling_delay);
		pr_debug("governor_name %s\n",
			 virtual_sensor_thermal_data[i].tzp.governor_name);
		pr_debug("num_tbps %d\n",
			 virtual_sensor_thermal_data[i].tzp.num_tbps);
		while (j < THERMAL_MAX_TRIPS) {
			pr_debug("trips[%d] %d\n", j,
				 virtual_sensor_thermal_data[i].trips[j].temp);
			pr_debug("type[%d] %d\n", j,
				 virtual_sensor_thermal_data[i].trips[j].type);
			pr_debug("hyst[%d] %d\n", j,
				 virtual_sensor_thermal_data[i].trips[j].hyst);
			j++;
		}
		j = 0;
		pr_debug("num_cdevs %d\n",
			 virtual_sensor_thermal_data[i].num_cdevs);

		while (h < virtual_sensor_thermal_data[i].num_cdevs) {
			pr_debug("cdevs[%d] %s\n", h,
				 virtual_sensor_thermal_data[i].cdevs[h]);
			h++;
		}
		h = 0;
	}
}
#endif

static int virtual_sensor_thermal_probe(struct platform_device *pdev)
{
	int ret;
	struct virtual_sensor_thermal_zone *tzone;
	struct vs_thermal_platform_data *pdata = NULL;
	char thermal_name[THERMAL_NAME_LENGTH];

#ifdef CONFIG_OF
	pr_notice("virtual_sensor thermal custom init by DTS!\n");
	ret = virtual_sensor_thermal_init_cust_data_from_dt(pdev);
	if (ret) {
		pr_err("%s: init data error\n", __func__);
		return ret;
	}
#endif
	if (pdev->id < virtual_sensor_nums)
		pdata = &virtual_sensor_thermal_data[pdev->id];
	else
		pr_err("pdev->id is invalid!\n");

	if (!pdata)
		return -EINVAL;
#ifdef DEBUG
	pr_notice("%s %d test data\n", __func__, __LINE__);
	test_data();
#endif
	tzone = devm_kzalloc(&pdev->dev, sizeof(*tzone), GFP_KERNEL);
	if (!tzone)
		return -ENOMEM;
	memset(tzone, 0, sizeof(*tzone));

	tzone->pdata = pdata;
	snprintf(thermal_name, sizeof(thermal_name), "%s%d", THERMAL_NAME,
		 pdev->id + 1);

	tzone->tz = thermal_zone_device_register(thermal_name,
						 THERMAL_MAX_TRIPS,
						 MASK,
						 tzone,
						 &virtual_sensor_tz_dev_ops,
						 &pdata->tzp,
						 0, pdata->polling_delay);
	if (IS_ERR(tzone->tz)) {
		pr_err("%s Failed to register thermal zone device\n", __func__);
		kfree(tzone);
		return -EINVAL;
	}
	tzone->tz->trips = pdata->num_trips;

#ifdef CONFIG_AMZN_METRICS_LOG
	tzone->mask = VIRTUAL_SENSOR_UNTHROTTLE_TIME_MASK;
#endif
	ret = virtual_sensor_create_sysfs(tzone);
	INIT_WORK(&tzone->therm_work, virtual_sensor_thermal_work);
	platform_set_drvdata(pdev, tzone);
	if (!tzone->tz->polling_delay)
		pdata->mode = THERMAL_DEVICE_DISABLED;
	else
		pdata->mode = THERMAL_DEVICE_ENABLED;
	return ret;
}

static int virtual_sensor_thermal_remove(struct platform_device *pdev)
{
	struct virtual_sensor_thermal_zone *tzone = platform_get_drvdata(pdev);

	if (tzone) {
		cancel_work_sync(&tzone->therm_work);
		if (tzone->tz)
			thermal_zone_device_unregister(tzone->tz);
		kfree(tzone);
	}
	return 0;
}

int thermal_dev_register(struct thermal_dev *tdev)
{
	struct vs_thermal_platform_data *pdata = NULL;

	if (unlikely(IS_ERR_OR_NULL(tdev))) {
		pr_err("%s: NULL sensor thermal device\n", __func__);
		return -ENODEV;
	}
	if (!tdev->dev_ops->get_temp) {
		pr_err("%s: Error getting get_temp()\n", __func__);
		return -EINVAL;
	}
	pr_info("%s %s select_device: %d aux_channel_num %d\n",
		__func__, tdev->name, tdev->tdp->select_device,
		tdev->tdp->aux_channel_num);

	if (tdev->tdp->select_device < virtual_sensor_nums) {
		pdata = &virtual_sensor_thermal_data[tdev->tdp->select_device];
	} else {
		pdata = &virtual_sensor_thermal_data[0];
		pr_err("%s select_device invalid!\n", tdev->name);
	}
	mutex_lock(&therm_lock);
	list_add_tail(&tdev->node, &pdata->ts_list);
	mutex_unlock(&therm_lock);
	return 0;
}
EXPORT_SYMBOL(thermal_dev_register);

#ifdef CONFIG_OF
static const struct of_device_id virtual_sensor_thermal_of_match[] = {
	{.compatible = COMPATIBLE_NAME,},
	{},
};

MODULE_DEVICE_TABLE(of, virtual_sensor_thermal_of_match);
#endif

static struct platform_driver virtual_sensor_thermal_zone_driver = {
	.probe = virtual_sensor_thermal_probe,
	.remove = virtual_sensor_thermal_remove,
	.suspend = NULL,
	.resume = NULL,
	.shutdown = NULL,
	.driver = {
		   .name = DRIVER_NAME,
		   .owner = THIS_MODULE,
#ifdef CONFIG_OF
		   .of_match_table = virtual_sensor_thermal_of_match,
#endif
		   },
};

#ifdef CONFIG_THERMAL_FOD
static void virtual_sensor_fod_remove(void)
{
	struct vs_temp_data *temp_data, *ptr;

	list_for_each_entry_safe(temp_data, ptr, &g_fod->fod_temp.list, list) {
		list_del(&temp_data->list);
		kfree(temp_data);
	}
}

static int virtual_sensor_fod_init(void)
{
	int i, retval = 0;
	struct vs_temp_data *ptr;
	struct fod_algo *fod_algo = NULL;

	fod_algo = kzalloc(sizeof(struct fod_algo), GFP_KERNEL);
	if (!fod_algo) {
		pr_err("%s Failed to allocate fod_algo memory\n",
			__func__);
		return -ENOMEM;
	}

	fod_algo->work_delay = TIME_TO_WORK;
	fod_algo->window_size = NUM_OF_COLLECT;
	fod_algo->polling_interval = TASK_DURATION;

	fod_algo->wpc_online_status = 0;
	fod_algo->fod_temp_cnt = 0;

	mutex_init(&fod_algo->fod_lock);

	fod_algo->psy_nb.notifier_call = wpc_online_event;
	retval = power_supply_reg_notifier(&fod_algo->psy_nb);
	if (retval < 0) {
		pr_err("%s register power supply notifier fail\n", __func__);
		goto err_reg_notify;
	}

	INIT_LIST_HEAD(&fod_algo->fod_temp.list);
	fod_algo->cur_list = &fod_algo->fod_temp.list;

	for (i = 1; i < fod_algo->window_size; i++) {
		ptr = kzalloc(sizeof(struct vs_temp_data), GFP_KERNEL);
		if (!ptr) {
			retval = -ENOMEM;
			goto err_list_add;
		}
		list_add_tail(&ptr->list, &fod_algo->fod_temp.list);
	}

	g_fod = fod_algo;
	INIT_DELAYED_WORK(&fod_algo->wpc_event_wq, wpc_online_work);

	return retval;
err_list_add:
	virtual_sensor_fod_remove();
err_reg_notify:
	kfree(fod_algo);
	fod_algo = NULL;
	return retval;
}
#endif

static int __init virtual_sensor_thermal_init(void)
{
	int err = 0;
#ifdef CONFIG_THERMAL_FOD
	struct thermal_zone_device *tz;
	bool wpc_online = false;
#endif

	err = platform_driver_register(&virtual_sensor_thermal_zone_driver);
	if (err) {
		pr_err("%s: Failed to register driver %s\n", __func__,
		       virtual_sensor_thermal_zone_driver.driver.name);
		goto error;
	}
#ifdef CONFIG_THERMAL_FOD
	tz = thermal_zone_get_zone_by_name(FOD_THERMAL_NAME);
	if (IS_ERR(tz)) {
		pr_err("[FOD_INFO]: thermal:%p  is NULL!\n", tz);
		tz = NULL;
	} else {
		err = virtual_sensor_fod_init();
		if (err)
			pr_err("%s Failed to allocate vs_temp_data memory\n",
				__func__);
		wireless_charger_dev_get_online(get_charger_by_name("wireless_chg"), &wpc_online);
		if (wpc_online) {
			g_fod->wpc_online_status = wpc_online;
			queue_delayed_work(system_freezable_wq, &g_fod->wpc_event_wq, 0);
		}
	}
#endif
error:
	return err;
}

static void __exit virtual_sensor_thermal_exit(void)
{
	platform_driver_unregister(&virtual_sensor_thermal_zone_driver);

#ifdef CONFIG_THERMAL_FOD
	virtual_sensor_fod_remove();
#endif
}

late_initcall(virtual_sensor_thermal_init);
module_exit(virtual_sensor_thermal_exit);

MODULE_DESCRIPTION("VIRTUAL_SENSOR pcb virtual sensor thermal zone driver");
MODULE_AUTHOR("Akwasi Boateng <boatenga@amazon.com>");
MODULE_LICENSE("GPL");
