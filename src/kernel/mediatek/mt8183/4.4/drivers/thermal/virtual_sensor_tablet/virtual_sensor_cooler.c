/*
 *  virtual_sensor_cooler.c
 *
 *  Copyright (C) 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved
 *  Author: Akwasi Boateng <boatenga@amazon.com>
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/printk.h>
#include <linux/types.h>
#include <linux/kobject.h>
#include <linux/slab.h>
#include <linux/thermal_framework.h>
#ifdef CONFIG_THERMAL_FOD
#include "mt-plat/charger_type.h"
#define POLICY_BPP_IDX 1
#define POLICY_EPP_IDX 2
#define POLICY_MFA_IDX 3
#endif

#define DRIVER_NAME "virtual_sensor_cooler"

static struct cooler_sort_list *cooler_list_head_map[VS_THERMAL_COOLER_COUNT] = {NULL};

static int virtual_sensor_get_max_state(struct thermal_cooling_device *cdev,
					unsigned long *state)
{
	struct vs_cooler_platform_data *pdata;

	if (!cdev || !(cdev->devdata)) {
		pr_err("%s cdev: %p or cdev->devdata is NULL!\n", __func__, cdev);
		return -EINVAL;
	}
	pdata = cdev->devdata;
	*state = pdata->max_state;
	return 0;
}

static int virtual_sensor_get_cur_state(struct thermal_cooling_device *cdev,
					unsigned long *state)
{
	struct vs_cooler_platform_data *pdata;

	if (!cdev || !(cdev->devdata)) {
		pr_err("%s cdev: %p or cdev->devdata is NULL!\n", __func__, cdev);
		return -EINVAL;
	}
	pdata = cdev->devdata;
	*state = pdata->state;
	return 0;
}

static int virtual_sensor_set_cur_state(struct thermal_cooling_device *cdev,
					unsigned long state)
{
	int level;
	struct vs_cooler_platform_data *pdata;
	struct cooler_sort_list *cooler_list = NULL;
#ifdef CONFIG_THERMAL_FOD
	int ret = 0;
	union power_supply_propval propval;
	struct power_supply *chg_psy;
	int dock_type;
#endif

	if (!cdev || !(cdev->devdata)) {
		pr_err("%s cdev: %p or cdev->devdata is NULL!\n", __func__, cdev);
		return -EINVAL;
	}
	pdata = cdev->devdata;
	if (pdata->state == state)
		return 0;

#ifdef CONFIG_THERMAL_FOD
	if (!strcmp(cdev->type, "wpc_bcct1")) {
		chg_psy = power_supply_get_by_name("charger");
		if (!chg_psy) {
			pr_err("Cannot find power supply \"charger\"\n");
			return -ENODEV;
		}

		ret = power_supply_get_property(chg_psy, POWER_SUPPLY_PROP_CHARGER_TYPE, &propval);
		if (ret < 0) {
			pr_err("[FOD_INFO] : get psy type failed, ret = %d\n",
				__func__, ret);
			return ret;
		}

		/* -------------------------------------------------------------
		 * Type    | enum charger_type            | Idx of policy      |
		 * -------------------------------------------------------------
		 * BPP     | WIRELESS_CHARGER_5W (8)      | POLICY_BPP_IDX (1) |
		 * -------------------------------------------------------------
		 * EPP     | WIRELESS_CHARGER_10W(9)      | POLICY_EPP_IDX (2) |
		 * -------------------------------------------------------------
		 * MFA     | WIRELESS_CHARGER_15W(10)     | POLICY_MFA_IDX (3) |
		 * -------------------------------------------------------------
		 * Default | WIRELESS_CHARGER_DEFAULT(11) | POLICY_BPP_IDX (1) |
		 * -------------------------------------------------------------
		 *  Set up charger type as trip point into policy.
		 *  The action of each trip point corresponds to the specified charger type.
		 */
		switch (propval.intval) {
		case WIRELESS_CHARGER_5W:
		case WIRELESS_CHARGER_DEFAULT:
			dock_type = POLICY_BPP_IDX;
			break;
		case WIRELESS_CHARGER_10W:
			dock_type = POLICY_EPP_IDX;
			break;
		case WIRELESS_CHARGER_15W:
			dock_type = POLICY_MFA_IDX;
			break;
		default:
			dock_type = POLICY_BPP_IDX;
			pr_err("[FOD_INFO] : charger_type[%d] is not supported\n", propval.intval);
			break;
		}

		if (state < dock_type)
			return 0;
		pr_info("[FOD_INFO]:%s state = %d, dock_type = %d\n", __func__, state, dock_type);
	}
#endif
	pdata->state = (state > pdata->max_state) ? pdata->max_state : state;
	if (!pdata->state)
		level = pdata->max_level;
	else
		level = pdata->levels[pdata->state - 1];

	pdata->level = level;

	if (pdata->thermal_cooler_id < VS_THERMAL_COOLER_COUNT)
		cooler_list = cooler_list_head_map[pdata->thermal_cooler_id];
	else {
		pr_err("%s incorrect thermal_cooler_id:[%d]\n",
				__func__, pdata->thermal_cooler_id);
		return -EINVAL;
	}

	mutex_lock(&cooler_list->update_mutex);
	level = thermal_level_compare(pdata, cooler_list, true);
	vs_set_cooling_level(cdev, pdata->thermal_cooler_id, level);
	mutex_unlock(&cooler_list->update_mutex);
	return 0;
}

static ssize_t levels_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct thermal_cooling_device *cdev =
	    container_of(dev, struct thermal_cooling_device, device);
	struct vs_cooler_platform_data *pdata;
	int i;
	int offset = 0;
	size_t bufsz = 0;

	if (!cdev || !(cdev->devdata)) {
		pr_err("%s cdev: %p or cdev->devdata is NULL!\n", __func__, cdev);
		return -EINVAL;
	}
	pdata = cdev->devdata;
	bufsz = pdata->max_state * 20;
	for (i = 0; i < pdata->max_state; i++)
		offset +=
		    scnprintf(buf + offset, bufsz - offset, "%d %d\n", i + 1, pdata->levels[i]);
	return offset;
}

static ssize_t levels_store(struct device *dev,
			    struct device_attribute *attr,
			    const char *buf, size_t count)
{
	int level, state;
	struct thermal_cooling_device *cdev =
	    container_of(dev, struct thermal_cooling_device, device);
	struct vs_cooler_platform_data *pdata;

	if (!cdev || !(cdev->devdata)) {
		pr_err("%s cdev: %p or cdev->devdata is NULL!\n", __func__, cdev);
		return -EINVAL;
	}
	pdata = cdev->devdata;
	if (sscanf(buf, "%d %d\n", &state, &level) != 2)
		return -EINVAL;
	if (state >= pdata->max_state)
		return -EINVAL;
	pdata->levels[state] = level;
	return count;
}

static struct thermal_cooling_device_ops cooling_ops = {
	.get_max_state = virtual_sensor_get_max_state,
	.get_cur_state = virtual_sensor_get_cur_state,
	.set_cur_state = virtual_sensor_set_cur_state,
};

static DEVICE_ATTR(levels, 0644, levels_show, levels_store);

static int vs_cooler_init_add(struct cooler_sort_list *node)
{
	struct cooler_sort_list *tz_list = NULL;
	int ret = 0;

	if (cooler_list_head_map[node->pdata->thermal_cooler_id] == NULL) {
		tz_list = kzalloc(sizeof(struct cooler_sort_list), GFP_KERNEL);
		if (!tz_list) {
			pr_err("%s Failed to allocate cooler_sort_list memory\n",
			__func__);
			ret = -ENOMEM;
		}
		INIT_LIST_HEAD(&tz_list->list);
		mutex_init(&tz_list->update_mutex);
		cooler_list_head_map[node->pdata->thermal_cooler_id] = tz_list;
	} else {
		tz_list = cooler_list_head_map[node->pdata->thermal_cooler_id];
	}

	mutex_lock(&tz_list->update_mutex);
	list_add(&node->list, &tz_list->list);
	mutex_unlock(&tz_list->update_mutex);

	return ret;
}

static int vs_cooler_probe(struct platform_device *pdev)
{
	struct vs_cooler_platform_data *pdata = NULL;
	struct cooler_sort_list *tz_list = NULL;
	int ret = 0;

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

#ifdef CONFIG_OF
	dev_info(&pdev->dev, "cooler custom init by DTS!\n");
	cooler_init_cust_data_from_dt(pdev, pdata);
#endif

	pdata->cdev = thermal_cooling_device_register(pdata->type,
							pdata,
							&cooling_ops);
	if (!pdata->cdev) {
		dev_err(&pdev->dev,
		"%s Failed to create virtual_sensor cooling device\n",
		__func__);
		ret = -EINVAL;
		goto register_err;
	}

	tz_list = kzalloc(sizeof(struct cooler_sort_list), GFP_KERNEL);
	if (!tz_list) {
		dev_err(&pdev->dev,
		"%s Failed to allocate cooler_sort_list memory\n",
		__func__);
		ret = -ENOMEM;
		goto list_mem_err;
	}
	tz_list->pdata = pdata;

	ret = vs_cooler_init_add(tz_list);
	if (ret)
		goto list_add_mem_err;

	ret = device_create_file(&pdata->cdev->device, &dev_attr_levels);
	if (ret)
		dev_err(&pdev->dev,
		"%s Failed to create vs cooler levels attr\n",
		__func__);

	platform_set_drvdata(pdev, pdata);
	return 0;

list_add_mem_err:
	kfree(tz_list);
list_mem_err:
	thermal_cooling_device_unregister(pdata->cdev);
register_err:
	devm_kfree(&pdev->dev, pdata);
	return ret;
}

static int vs_cooler_remove(struct platform_device *pdev)
{
	struct vs_cooler_platform_data *pdata = platform_get_drvdata(pdev);
	struct cooler_sort_list *cooler_list = NULL;
	struct cooler_sort_list *vscooler_list, *tmp_list;

	if (pdata) {
		cooler_list = cooler_list_head_map[pdata->thermal_cooler_id];
		mutex_lock(&cooler_list->update_mutex);
		list_for_each_entry_safe(vscooler_list, tmp_list, &cooler_list->list, list) {
			if (vscooler_list->pdata == pdata) {
				list_del(&vscooler_list->list);
				kfree(vscooler_list);
				vscooler_list = NULL;
				break;
			}
		}
		mutex_unlock(&cooler_list->update_mutex);
		if (pdata->cdev) {
			device_remove_file(&pdata->cdev->device, &dev_attr_levels);
			thermal_cooling_device_unregister(pdata->cdev);
		}
		devm_kfree(&pdev->dev, pdata);

		if (list_empty(&cooler_list->list)) {
			mutex_destroy(&cooler_list->update_mutex);
			kfree(cooler_list);
			cooler_list_head_map[pdata->thermal_cooler_id] = NULL;
		}
	}
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id virtual_sensor_of_match_table[] = {
	{.compatible = "amazon,virtual_sensor_cooler", },
	{},
};
MODULE_DEVICE_TABLE(of, virtual_sensor_of_match_table);
#endif

static struct platform_driver vs_cooler_driver = {
	.probe = vs_cooler_probe,
	.remove = vs_cooler_remove,
	.driver     = {
		.name  = DRIVER_NAME,
#ifdef CONFIG_OF
		.of_match_table = virtual_sensor_of_match_table,
#endif
		.owner = THIS_MODULE,
	},
};

static int __init virtual_sensor_init(void)
{
	int ret;

	ret = platform_driver_register(&vs_cooler_driver);
	if (ret) {
		pr_err("%s: Failed to register driver %s\n", __func__,
			vs_cooler_driver.driver.name);
		return ret;
	}

	return 0;

}

static void __exit virtual_sensor_exit(void)
{
	platform_driver_unregister(&vs_cooler_driver);
}

module_init(virtual_sensor_init);
module_exit(virtual_sensor_exit);
