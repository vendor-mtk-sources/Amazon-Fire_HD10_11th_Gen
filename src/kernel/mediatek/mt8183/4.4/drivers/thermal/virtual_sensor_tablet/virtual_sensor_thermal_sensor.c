/*
 * Copyright (C) 2020 Amazon.com, Inc.  All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify it under the terms of the
 * GNU General Public License version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/stddef.h>
#include <linux/sysfs.h>
#include <linux/err.h>
#include <linux/reboot.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/thermal_framework.h>

#define SENSOR_NAME "virtual_sensor_thermal_sensor"
#define BUF_SIZE 128

struct thermal_dev_node_names thermal_sensor_node_names = {
	.offset_name = "thermal_sensor,offset",
	.offset_invert_name = "thermal_sensor,offset_invert",
	.alpha_name = "thermal_sensor,alpha",
	.weight_name = "thermal_sensor,weight",
	.weight_invert_name = "thermal_sensor,weight_invert",
	.select_device_name = "select_device",
	.thermal_sensor_id = "thermal_sensor_id",
	.aux_channel_num_name = "aux_channel_num",
};

static int virtual_sensor_thermal_sensor_read_temp(struct thermal_dev *tdev)
{
	if (!tdev || !(tdev->tdp)) {
		pr_err("%s tdev: %p or tdev->tdp is NULL!\n", __func__, tdev);
		return -EINVAL;
	}

#ifdef CONFIG_THERMAL_FOD
	return vs_thermal_sensor_get_temp(tdev->tdp->thermal_sensor_id,
				tdev->tdp->aux_channel_num, tdev->tdp->select_device);
#else
	return vs_thermal_sensor_get_temp(tdev->tdp->thermal_sensor_id,
				tdev->tdp->aux_channel_num);
#endif
}

static struct thermal_dev_ops virtual_sensor_thermal_sensor_fops = {
	.get_temp = virtual_sensor_thermal_sensor_read_temp,
};

static ssize_t virtual_sensor_thermal_sensor_show_temp(struct device *dev,
					     struct device_attribute *devattr,
					     char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct virtual_sensor_temp_sensor *virtual_sensor;
	int temp;

	if (!pdev) {
		pr_err("%s pdev is NULL!\n", __func__);
		return -EINVAL;
	}
	virtual_sensor = platform_get_drvdata(pdev);
	if (!virtual_sensor || !(virtual_sensor->therm_fw)
		|| !(virtual_sensor->therm_fw->tdp)) {
		pr_err("%s virtual_sensor: %p, virtual_sensor->therm_fw or"
			" virtual_sensor->therm_fw->tdp is NULL!\n",
			__func__, virtual_sensor);
		return -EINVAL;
	}

#ifdef CONFIG_THERMAL_FOD
	temp = vs_thermal_sensor_get_temp(
				virtual_sensor->therm_fw->tdp->thermal_sensor_id,
				virtual_sensor->therm_fw->tdp->aux_channel_num,
				virtual_sensor->therm_fw->tdp->select_device);
#else
	temp = vs_thermal_sensor_get_temp(
				virtual_sensor->therm_fw->tdp->thermal_sensor_id,
				virtual_sensor->therm_fw->tdp->aux_channel_num);
#endif

	return scnprintf(buf, BUF_SIZE, "%d\n", temp);
}

static ssize_t virtual_sensor_thermal_sensor_show_params(struct device *dev,
					       struct device_attribute *devattr,
					       char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct virtual_sensor_temp_sensor *virtual_sensor;

	if (!pdev) {
		pr_err("%s pdev is NULL!\n", __func__);
		return -EINVAL;
	}
	virtual_sensor = platform_get_drvdata(pdev);
	if (!virtual_sensor || !(virtual_sensor->therm_fw)
		|| !(virtual_sensor->therm_fw->tdp)) {
		pr_err("%s virtual_sensor: %p, virtual_sensor->therm_fw or"
			" virtual_sensor->therm_fw->tdp is NULL!\n",
			__func__, virtual_sensor);
		return -EINVAL;
	}

	return scnprintf(buf, BUF_SIZE, "offset=%d alpha=%d weight=%d\n",
		       virtual_sensor->therm_fw->tdp->offset,
		       virtual_sensor->therm_fw->tdp->alpha,
		       virtual_sensor->therm_fw->tdp->weight);
}

static ssize_t virtual_sensor_thermal_sensor_store_params(struct device *dev, struct device_attribute
						*devattr, const char *buf,
						size_t count)
{
	char param[20];
	int value = 0;
	struct platform_device *pdev = to_platform_device(dev);
	struct virtual_sensor_temp_sensor *virtual_sensor;

	if (!pdev) {
		pr_err("%s pdev is NULL!\n", __func__);
		return -EINVAL;
	}
	virtual_sensor = platform_get_drvdata(pdev);
	if (!virtual_sensor || !(virtual_sensor->therm_fw)
		|| !(virtual_sensor->therm_fw->tdp)) {
		pr_err("%s virtual_sensor: %p, virtual_sensor->therm_fw or"
			" virtual_sensor->therm_fw->tdp is NULL!\n",
			__func__, virtual_sensor);
		return -EINVAL;
	}

	if (sscanf(buf, "%19s %d", param, &value) == 2) {
		if (!strcmp(param, "offset"))
			virtual_sensor->therm_fw->tdp->offset = value;
		if (!strcmp(param, "alpha"))
			virtual_sensor->therm_fw->tdp->alpha = value;
		if (!strcmp(param, "weight"))
			virtual_sensor->therm_fw->tdp->weight = value;
		return count;
	}
	return -EINVAL;
}

static DEVICE_ATTR(params, 0644, virtual_sensor_thermal_sensor_show_params,
		   virtual_sensor_thermal_sensor_store_params);
static DEVICE_ATTR(temp, 0444, virtual_sensor_thermal_sensor_show_temp, NULL);

static int virtual_sensor_thermal_sensor_probe(struct platform_device *pdev)
{
	struct virtual_sensor_temp_sensor *virtual_sensor;
	struct thermal_dev_params *thermal_sensor_params;
	int ret = 0;

	virtual_sensor =
	    kzalloc(sizeof(struct virtual_sensor_temp_sensor), GFP_KERNEL);
	if (!virtual_sensor)
		return -ENOMEM;

	thermal_sensor_params =
	    devm_kzalloc(&pdev->dev, sizeof(*thermal_sensor_params), GFP_KERNEL);
	if (!thermal_sensor_params) {
		ret = -ENOMEM;
		goto thermal_sensor_params_err;
	}

	virtual_sensor->dev = &pdev->dev;
	platform_set_drvdata(pdev, virtual_sensor);

	virtual_sensor->last_update = jiffies - HZ;

	virtual_sensor->therm_fw =
	    kzalloc(sizeof(struct thermal_dev), GFP_KERNEL);
	if (virtual_sensor->therm_fw) {
		virtual_sensor->therm_fw->name = pdev->dev.of_node->name;
		virtual_sensor->therm_fw->dev = virtual_sensor->dev;
		virtual_sensor->therm_fw->dev_ops = &virtual_sensor_thermal_sensor_fops;
		virtual_sensor->therm_fw->tdp =
		    thermal_sensor_dt_to_params(&pdev->dev, thermal_sensor_params,
						&thermal_sensor_node_names);
		ret = thermal_dev_register(virtual_sensor->therm_fw);
		if (ret) {
			dev_err(&pdev->dev,
				"error registering therml device\n");
			ret = -EINVAL;
			goto register_err;
		}
	} else {
		ret = -ENOMEM;
		goto therm_fw_alloc_err;
	}

	ret = device_create_file(&pdev->dev, &dev_attr_params);
	if (ret)
		pr_err("%s Failed to create params attr\n", __func__);
	ret = device_create_file(&pdev->dev, &dev_attr_temp);
	if (ret)
		pr_err("%s Failed to create temp attr\n", __func__);

	return 0;

register_err:
	kfree(virtual_sensor->therm_fw);

therm_fw_alloc_err:
	devm_kfree(&pdev->dev, (void *)thermal_sensor_params);

thermal_sensor_params_err:
	kfree(virtual_sensor);
	return ret;
}

static int virtual_sensor_thermal_sensor_remove(struct platform_device *pdev)
{
	struct virtual_sensor_temp_sensor *virtual_sensor =
	    platform_get_drvdata(pdev);

	if (virtual_sensor && virtual_sensor->therm_fw) {
		if (virtual_sensor->therm_fw->tdp)
			devm_kfree(&pdev->dev, (void *)virtual_sensor->therm_fw->tdp);
		kfree(virtual_sensor->therm_fw);
	}
	if (virtual_sensor)
		kfree(virtual_sensor);
	device_remove_file(&pdev->dev, &dev_attr_params);
	device_remove_file(&pdev->dev, &dev_attr_temp);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id thermal_sensor_thermal_sensor[] = {
	{.compatible = "amazon,virtual_sensor_thermal_sensor",},
	{},
};

MODULE_DEVICE_TABLE(of, thermal_sensor_thermal_sensor);
#endif

static struct platform_driver virtual_sensor_thermal_sensor_driver = {
	.probe = virtual_sensor_thermal_sensor_probe,
	.remove = virtual_sensor_thermal_sensor_remove,
	.driver = {
		   .name = SENSOR_NAME,
#ifdef CONFIG_OF
		   .of_match_table = thermal_sensor_thermal_sensor,
#endif
		   .owner = THIS_MODULE,
		   },
};

static int __init virtual_sensor_thermal_sensor_init(void)
{
	int ret;

	ret = init_vs_thermal_platform_data();
	if (ret) {
		pr_err("%s: init_vs_thermal_platform_data error\n", __func__);
		return ret;
	}

	ret = platform_driver_register(&virtual_sensor_thermal_sensor_driver);
	if (ret) {
		pr_err("Unable to register virtual sensor battery driver (%d)\n", ret);
		goto err_unreg;
	}
	return 0;

err_unreg:
	return ret;
}

static void __exit virtual_sensor_thermal_sensor_exit(void)
{
	platform_driver_unregister(&virtual_sensor_thermal_sensor_driver);
}

module_init(virtual_sensor_thermal_sensor_init);
module_exit(virtual_sensor_thermal_sensor_exit);
