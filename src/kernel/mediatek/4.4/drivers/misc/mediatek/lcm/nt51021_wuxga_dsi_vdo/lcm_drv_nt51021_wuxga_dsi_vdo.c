/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/gpio.h>
#include <linux/device.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#endif

#include "nt51021_wuxga_dsi_vdo.h"

#define LOG_TAG "LCM-NT51021-WUXGA-DSI-VDO"

#define LCM_DBG(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)

static int lcm_request_gpio_control(struct device *dev)
{
	int ret = 0;

	GPIO_LCD_RST_EN = of_get_named_gpio(dev->of_node, "gpio_lcm_rst_en", 0);

	ret = gpio_request(GPIO_LCD_RST_EN, "GPIO_LCD_RST_EN");
	if (ret)
		LCM_DBG("gpio request reset pin = 0x%x fail", GPIO_LCD_RST_EN);

	return ret;
}

static int lcm_driver_probe(struct device *dev, void const *data)
{
	int ret;

	ret = lcm_request_gpio_control(dev);
	if (ret)
		return ret;

	return 0;
}

static const struct of_device_id lcm_platform_of_match[] = {
	{
		.compatible = "novatek,nt51021",
		.data = 0,
	}, {
		/* sentinel */
	}
};


MODULE_DEVICE_TABLE(of, platform_of_match);

static int lcm_platform_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;

	id = of_match_node(lcm_platform_of_match, pdev->dev.of_node);
	if (!id)
		return -ENODEV;

	return lcm_driver_probe(&pdev->dev, id->data);
}

static struct platform_driver lcm_driver = {
	.probe = lcm_platform_probe,
	.driver = {
		.name = "nt51021_wuxga_dsi_vdo",
		.owner = THIS_MODULE,
		.of_match_table = lcm_platform_of_match,
	},
};

static int __init lcm_init(void)
{
	LCM_DBG("Register panel driver for nt51021_wuxga_dsi_vdo");
	if (platform_driver_register(&lcm_driver)) {
		LCM_DBG("Failed to register this driver!");
		return -ENODEV;
	}

	return 0;
}

static void __exit lcm_exit(void)
{
	platform_driver_unregister(&lcm_driver);
	LCM_DBG("Unregister this driver done");
}

late_initcall(lcm_init);
module_exit(lcm_exit);
MODULE_AUTHOR("Compal");
MODULE_DESCRIPTION("LCM display subsystem driver");
MODULE_LICENSE("GPL");
