/*
 * Copyright (C) 2021 Amazon.com, Inc.  All rights reserved.
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <misc/amzn_ld_switch.h>
static struct pinctrl *adcsw3_pctl;
int liquid_id_status;
int hardware_id_status;
bool board_id_status;
int amzn_ld_switch_is_support;
static DEFINE_MUTEX(adcsw3_switch_lock);
static int adcsw3_switch_status; /* Default to read battery id */
#define PINCTRL_BATTERY_ID	"switch_battery_id"
#define PINCTRL_BATTERY_NTC	"switch_battery_ntc"
#define PVT_1B			7
#define BOARD_ID		"/idme/board_id"

static int get_liquid_id_num(void)
{
	struct device_node *node;
	int gpionum = -1;
	int err = 0;

	node = of_find_compatible_node(NULL, NULL, "amzn,liquid_id_pin");
	if (!node || !of_device_is_available(node))
		goto exit;
	else {
		gpionum = of_get_named_gpio(node, "liquid_id_pin", 0);
		err = gpio_request(gpionum, "liquid_id_pin");
		if (err) {
			pr_err("%s, failed to request liquid_id_pin %d\n",
				__func__, gpionum);
			gpionum = -1;
		}
	}
exit:
	of_node_put(node);
	return gpionum;
}

static void get_liquid_id_status(int num)
{
	liquid_id_status = gpio_get_value(num);
	pr_info("%s, liquid_id_pin[%d]\n", __func__, liquid_id_status);
	gpio_free(num);
}

static int get_hardware_id(void)
{
	struct device_node *node;
	int hwid = -1;
	int id1, id2, id3;
	int err = 0;

	node = of_find_compatible_node(NULL, NULL, "amzn,board_id_pin");
	if (!node || !of_device_is_available(node))
		goto exit;
	else {
		id3 = of_get_named_gpio(node, "boardid_3", 0);
		err = gpio_request(id3, "board_id_3");
		if (err) {
			pr_err("%s, failed to request boardid_3 %d\n",
				__func__, id3);
			goto exit;
		} else {
			hwid = (gpio_get_value(id3) << 2);
			gpio_free(id3);
		}

		id2 = of_get_named_gpio(node, "boardid_2", 0);
		err = gpio_request(id2, "board_id_2");
		if (err) {
			pr_err("%s, failed to request boardid_2 %d\n",
				__func__, id2);
			goto exit;
		} else {
			hwid += (gpio_get_value(id2) << 1);
			gpio_free(id2);
		}

		id1 = of_get_named_gpio(node, "boardid_1", 0);
		err = gpio_request(id1, "board_id_1");
		if (err) {
			pr_err("%s, failed to request boardid_1 %d\n",
				__func__, id1);
			goto exit;
		} else {
			hwid += gpio_get_value(id1);
			gpio_free(id1);
		}

		of_node_put(node);
		return hwid;
	}
exit:
	of_node_put(node);
	hwid = -1;
	return hwid;
}

static bool adcsw3_switch_init(void)
{
	struct device_node *np;
	struct platform_device *pdev;
	struct pinctrl *pinctrl = NULL;

	np = of_find_compatible_node(NULL, NULL, "amzn,adcsw3_switch_ctrl");
	if (!np || !of_device_is_available(np)) {
		pr_err("%s, can't find compatible node for adcsw3_switch_ctrl\n", __func__);
		goto err_node;
	} else {
		pdev = of_find_device_by_node(np);
		pinctrl = devm_pinctrl_get(&pdev->dev);
		if (IS_ERR(pinctrl)) {
			pr_err("%s, can't find pinctrl\n", __func__);
			goto err_node;
		} else {
			adcsw3_pctl = pinctrl;
			return true;
		}
	}
err_node:
	of_node_put(np);
	return false;
}

void amzn_ld_switch_adcsw3_lock(void)
{
	mutex_lock(&adcsw3_switch_lock);
}

void amzn_ld_switch_adcsw3_unlock(void)
{
	mutex_unlock(&adcsw3_switch_lock);
}

int amzn_ld_switch_adcsw3(int level)
{
	struct pinctrl_state *adcsw3_gpio;
	int ret = 0;

	if (!IS_ERR_OR_NULL(adcsw3_pctl)) {
		if (level != adcsw3_switch_status) {
			if (level == NTC_BATTERY)
				adcsw3_gpio = pinctrl_lookup_state(adcsw3_pctl, PINCTRL_BATTERY_NTC);
			else
				adcsw3_gpio = pinctrl_lookup_state(adcsw3_pctl, PINCTRL_BATTERY_ID);

			if (IS_ERR(adcsw3_gpio)) {
				pr_err("failed to find pinctrl [%s]\n",
					((level > 0) ? PINCTRL_BATTERY_NTC : PINCTRL_BATTERY_ID));
				return -ENODEV;
			} else {
				pinctrl_select_state(adcsw3_pctl, adcsw3_gpio);
				adcsw3_switch_status = level;
				/* it will need > 200ms for the delay time for NTC reading */
				msleep(200);
				return ret;
			}
		}
	} else {
		pr_err("%s: failed to find pinctrl [adcsw3_pctl]\n", __func__);
		return -ENODEV;
	}

	return ret;
}

static bool get_boardinfo(void)
{
	struct device_node *ap;
	int len;

	ap = of_find_node_by_path(BOARD_ID);
	if (ap) {
		const char *boardid = of_get_property(ap, "value", &len);
		if (len >= 2) {
			if (boardid[2] == '5' && boardid[3] == 'E')
				return true;
		}
	}

	return false;
}

static int __init ld_switch_init(void)
{
	int number;

	number = get_liquid_id_num();
	if (number != -1)
		get_liquid_id_status(number);
	else {
		pr_err("%s: failed to get liquid id status\n", __func__);
		goto out;
	}

	if (adcsw3_switch_init() != true) {
		pr_err("%s, failed to init adcsw3 gpio\n", __func__);
		goto out;
	}

	hardware_id_status = get_hardware_id();
	if (hardware_id_status != -1)
		pr_info("%s, hardwareID = %d\n", __func__, hardware_id_status);
	else
		goto out;

	board_id_status = get_boardinfo();
	pr_info("%s, board id = %d\n", __func__, board_id_status);

	/*
	 * Based on product/hardware_phase/gpio_value to use switch ic solution or not.
	 *
	 * @liquid_id_status : level of gpio to identify ld solution used.
	 * 1 -> use analog switch ic; 0 -> use fusb251 ic.
	 * @hardware_id_status : to get defined hardware phase.
	 * @board_id_status : using correct product or not. 0 -> No; 1-> Yes.
	 *
	 */
	if (liquid_id_status != FUSB251_NO_MOUNT && hardware_id_status != PVT_1B && board_id_status)
		amzn_ld_switch_is_support = 0;
	else
		amzn_ld_switch_is_support = 1;
	return 0;
out:
	amzn_ld_switch_is_support = 0;
	return -ENODEV;
}

fs_initcall(ld_switch_init);
