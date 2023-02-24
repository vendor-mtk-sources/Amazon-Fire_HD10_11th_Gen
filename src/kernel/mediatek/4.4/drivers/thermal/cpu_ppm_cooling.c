/*
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/thermal.h>
#include <linux/of_platform.h>
#include "mach/mtk_ppm_api.h"

/*
 * This PPM_FREQS array corresponds to the available frequencies allowed by the
 * architecture.
 */
static const int PPM_FREQS[] = {1989000, 1924000, 1846000, 1781000, 1716000, 1677000,
				1625000, 1586000, 1508000, 1417000, 1326000, 1248000,
				1131000, 1014000, 910000, 793000};

/*
 * struct cooler_platform_data
 * state : current state of the cooler
 * max_state : max state of the cooler
 * cdev : cooling device pointer
 */
struct cooler_platform_data {
	unsigned int state;
	unsigned int max_state;
	struct thermal_cooling_device *cdev;
};

static int cpu_ppm_cooling_get_max_state(struct thermal_cooling_device *cdev,
	unsigned long *state)
{
	struct cooler_platform_data *pdata = cdev->devdata;

	*state = pdata->max_state;

	return 0;
}

static int cpu_ppm_cooling_get_cur_state(struct thermal_cooling_device *cdev,
	unsigned long *state)
{
	struct cooler_platform_data *pdata = cdev->devdata;

	*state = pdata->state;

	return 0;
}

static int cpu_ppm_cooling_set_cur_state(struct thermal_cooling_device *cdev,
	unsigned long state)
{
	struct cooler_platform_data *pdata = cdev->devdata;

	pdata->state = state;

	// Set PPM for small cores
	mt_ppm_sysboost_set_freq_limit(BOOST_BY_UT, 0, -1, PPM_FREQS[state]);

	// Set PPM for big cores
	mt_ppm_sysboost_set_freq_limit(BOOST_BY_UT, 1, -1, PPM_FREQS[state]);

	return 0;
}

static struct thermal_cooling_device_ops cpu_ppm_cooling_ops = {
	.get_max_state = cpu_ppm_cooling_get_max_state,
	.get_cur_state = cpu_ppm_cooling_get_cur_state,
	.set_cur_state = cpu_ppm_cooling_set_cur_state,
};

static int cpu_ppm_cooling_probe(struct platform_device *pdev)
{
	struct cooler_platform_data *pdata;
	struct device_node *np;
	int ret;

	np = pdev->dev.of_node;
	if (!np) {
		dev_err(&pdev->dev, "%s: Error no of_node\n", __func__);
		return -EINVAL;
	}

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(&pdev->dev, "%s: No Memory \n", __func__);
		return -ENOMEM;
	}

	ret = of_property_read_u32(np, "max_state", &pdata->max_state);
	if (ret) {
		dev_info(&pdev->dev, "%s: max state not set %d\n", __func__, ret);
		return -EINVAL;
	}

	if (pdata->max_state >= ARRAY_SIZE(PPM_FREQS))
		dev_err(&pdev->dev, "%s: max state in dts greater than size of levels array\n",
			__func__);

	pdata->cdev = thermal_of_cooling_device_register(np, (char *)np->name,
					pdata, &cpu_ppm_cooling_ops);

	if (IS_ERR(pdata->cdev)) {
		dev_err(&pdev->dev,
				"Failed to register cpu ppm cooling device\n");
		return PTR_ERR(pdata->cdev);
	}

	platform_set_drvdata(pdev, pdata->cdev);
	dev_info(&pdev->dev, "CPU ppm cooling device registered: %s\n",
		pdata->cdev->type);

	return 0;
}

static int cpu_ppm_cooling_remove(struct platform_device *pdev)
{
	struct thermal_cooling_device *cdev = platform_get_drvdata(pdev);

	thermal_cooling_device_unregister(cdev);

	return 0;
}

static const struct of_device_id cpu_ppm_cooling_of_match[] = {
	{.compatible = "mt8183,cpu_ppm_cooler", },
	{ },
};

static struct platform_driver cpu_ppm_cooling_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "cpu_ppm_cooling",
		.of_match_table = cpu_ppm_cooling_of_match,
	},
	.probe = cpu_ppm_cooling_probe,
	.remove = cpu_ppm_cooling_remove,
};

static int cpu_ppm_cooling_init(void)
{
	int ret;

	ret = platform_driver_register(&cpu_ppm_cooling_driver);

	if (ret)
		pr_err("Unable to register cpu ppm cooling driver (%d)\n", ret);

	return ret;
}

static void cpu_ppm_cooling_exit(void)
{
	platform_driver_unregister(&cpu_ppm_cooling_driver);
}

module_init(cpu_ppm_cooling_init);
module_exit(cpu_ppm_cooling_exit);
MODULE_DESCRIPTION("Amazon CPU PPM cooling driver");
MODULE_LICENSE("GPL");
