/* drivers/input/amzn_keycombo.c
 *
 * Copyright (C) 2018 Amazon.com, Inc.
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

#include <linux/input.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/keycombo.h>
#include <linux/of.h>
#include <linux/reboot.h>
#include <mt-plat/mtk_rtc.h>
#ifdef CONFIG_AMAZON_POWEROFF_LOG
#include <linux/delay.h>
#endif

#ifdef CONFIG_AMAZON_SIGN_OF_LIFE
#include <linux/sign_of_life.h>
#endif

extern bool check_pwrkey_status(void);

/*
 * Weak functions
 */
bool __attribute__ ((weak)) check_pwrkey_status(void)
{
	pr_err("Weak Function: %s doesn't exist\n", __func__);
	return true;
}

#define AMZN_KEYCOMBO_NAME "amzn_keycombo"
#define PROP_KEYS_DOWN "keys_down"
#define PROP_KEYS_DOWN_DELAY "keys_down_delay"
#define PROP_KEYS_DOWN_FUNC "keys_down_func"
#define PROP_KEYS_UP "keys_up"

enum COMBO_FUNC {
	COMBO_FUNC_PANIC = 0,
	COMBO_FUNC_POWER_OFF,
	COMBO_FUNC_NUM
};

#define COMBO_MSG_SIZE 100
struct function {
	enum COMBO_FUNC fn;
	void (*handler)(void *data);
	char msg[COMBO_MSG_SIZE];
};

struct keys_config {
	struct keycombo_platform_data *pcombo_data;
	int combo_data_size;
	struct platform_device *pdev_keycombo;
	struct function *func;
};

struct label {
    const char *name;
    int value;
};

#define LABEL(constant) { #constant, constant }
#define LABEL_END { NULL, -1 }

static struct label key_labels[] = {
	LABEL(KEY_VOLUMEDOWN),
	LABEL(KEY_VOLUMEUP),
	LABEL(KEY_POWER),
	LABEL_END,
};

#ifdef CONFIG_AMAZON_POWEROFF_LOG
static void log_long_press_power_key(void)
{
	int rc;
	char *argv[] = {
		"/sbin/crashreport",
		"long_press_power_key",
		NULL
	};

	rc = call_usermodehelper(argv[0], argv, NULL, UMH_WAIT_EXEC);
	if (rc < 0)
		pr_err("call /sbin/crashreport failed, rc = %d\n", rc);

	/*Sleep 6 seconds for saving logs*/
	msleep(6000);
}
#endif /* CONFIG_AMAZON_POWEROFF_LOG */

static int get_key_from_label(const char *name)
{
	int i;
	for (i = 0; key_labels[i].name; i++) {
		if (!strcmp(name, key_labels[i].name))
			return key_labels[i].value;
	}
	return -EINVAL;
}

static void do_panic(void *data)
{
	struct keys_config *keys_cfg = data;
	panic(keys_cfg->func->msg);
}

static void do_power_off(void *data)
{
	struct keys_config *keys_cfg = data;
	pr_info("%s above %d seconds\n", keys_cfg->func->msg, keys_cfg->pcombo_data->key_down_delay/1000);

	if (!check_pwrkey_status()) {
		pr_err("power key is released after 6 seconds\n");
		return;
	}

#ifdef CONFIG_AMAZON_POWEROFF_LOG
	log_long_press_power_key();
#endif
	if (!check_pwrkey_status()) {
		pr_err("power key is released after 12 seconds\n");
		return;
	}

#ifdef CONFIG_AMAZON_SIGN_OF_LIFE
	life_cycle_set_shutdown_reason(SHUTDOWN_BY_LONG_PWR_KEY_PRESS);
#endif
	rtc_mark_sw_lprst();
	orderly_reboot();
}

static struct function funcs[] = {
	{
		.fn = COMBO_FUNC_PANIC,
		.handler = do_panic,
		.msg = "Trigger panic by press ",
	},
	{
		.fn = COMBO_FUNC_POWER_OFF,
		.handler = do_power_off,
		.msg  = "Trigger power off by long press ",
	},
};

static struct function* get_func(enum COMBO_FUNC fn)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(funcs); i++) {
		if (funcs[i].fn == fn)
			return &funcs[i];
	}
	return NULL;
}

static void free_keycombo_platdata(struct keys_config *keys_cfg)
{
	struct keycombo_platform_data *pcombo_data = keys_cfg->pcombo_data;

	if (pcombo_data) {
		if (pcombo_data->keys_up) {
			devm_kfree(&keys_cfg->pdev_keycombo->dev, pcombo_data->keys_up);
			pcombo_data->keys_up = NULL;
		}
		devm_kfree(&keys_cfg->pdev_keycombo->dev, pcombo_data);
		keys_cfg->pcombo_data = NULL;
	}
}

static int dt_to_keycombo_data(struct platform_device *pdev, struct keys_config *keys_cfg)
{
	struct device_node *node = pdev->dev.of_node;
	int down_size, up_size, size, ret, idx;
	struct keycombo_platform_data *pcombo_data;
	struct device *dev = &pdev->dev;
	unsigned int func;

	/* get keys_down mapping */
	down_size = of_property_count_strings(dev->of_node, PROP_KEYS_DOWN);
	if (IS_ERR_VALUE(down_size)) {
		dev_err(dev, "%s: Failed to get keys_down mapping %d\n", __func__, down_size);
		return 0;
	}

	size = sizeof(struct keycombo_platform_data)
			+ sizeof(int) * (down_size + 1);
	pcombo_data = devm_kzalloc(&pdev->dev, size, GFP_KERNEL);
	if (!pcombo_data) {
		dev_err(dev, "%s: Failed to alloc memory %d for struct keycombo_platform_data\n", __func__, size);
		return 0;
	}

	/* get keys_down_func prop and initial string */
	ret = of_property_read_u32(node, PROP_KEYS_DOWN_FUNC,
				&func);
	if (ret) {
			dev_err(dev, "%s: %s not found\n",
						__func__, PROP_KEYS_DOWN_FUNC);
			goto err;
	} else {
		keys_cfg->func = get_func(func);
		if (keys_cfg->func) {
			pcombo_data->key_down_fn = keys_cfg->func->handler;
		} else {
			dev_err(dev, "%s: %s not found\n",
						__func__, PROP_KEYS_DOWN_FUNC);
			goto err;
		}
	}

	for (idx  = 0; idx < down_size; idx++) {
		const char *name;
		int key;
		ret = of_property_read_string_index(dev->of_node,
						    PROP_KEYS_DOWN, idx,
						    &name);
		if (ret) {
			dev_err(dev, "%s: of read string %s idx %d error %d\n",
				__func__, PROP_KEYS_DOWN, idx, ret);
			goto err;
		}
		key = get_key_from_label(name);
		if (key < 0) {
			dev_err(dev, "%s: invalid down key name %s\n",
				__func__, name);
			goto err;
		}

		pcombo_data->keys_down[idx] = key;
		strncat(keys_cfg->func->msg + strlen(keys_cfg->func->msg), name, COMBO_MSG_SIZE - strlen(keys_cfg->func->msg));
		strncat(keys_cfg->func->msg + strlen(keys_cfg->func->msg), " ", COMBO_MSG_SIZE - strlen(keys_cfg->func->msg));
	}
	pcombo_data->keys_down[idx] = 0;

	/* get keys_down_delay prop */
	ret = of_property_read_u32(node, PROP_KEYS_DOWN_DELAY,
				&pcombo_data->key_down_delay);
	if (ret) {
			dev_err(dev, "%s: %s not found\n",
						__func__, PROP_KEYS_DOWN_DELAY);
			goto err;
	}

	/*get keys_up mapping */
	up_size = of_property_count_strings(dev->of_node, PROP_KEYS_UP);
	if (!IS_ERR_VALUE(up_size)) {

		if (up_size > 0) {
			pcombo_data->keys_up = devm_kzalloc(&pdev->dev, up_size + 1,
									GFP_KERNEL);
			if (!pcombo_data->keys_up) {
				dev_err(dev, "%s: Failed to alloc memory %d for keys_up\n", __func__, up_size + 1);
				goto err;
			}

			for (idx  = 0; idx < up_size; idx++) {
				const char *name;
				int key;
				ret = of_property_read_string_index(dev->of_node,
								    PROP_KEYS_UP, idx,
								    &name);
				if (ret) {
					dev_err(dev, "%s: of read string %s idx %d error %d\n",
						__func__, PROP_KEYS_UP, idx, ret);
					break;
				}
				key = get_key_from_label(name);
				if (key < 0) {
					dev_err(dev, "%s: invalid up key name %s\n",
							__func__, name);
					break;
				}

				pcombo_data->keys_up[idx] = key;
			}
			pcombo_data->keys_up[idx] = 0;
		}
	}

	keys_cfg->pcombo_data = pcombo_data;
	return size;

err:
	free_keycombo_platdata(keys_cfg);
	return 0;
}

static int amzn_keycombo_probe(struct platform_device *pdev)
{
	int ret = -ENOMEM;
	struct keys_config *keys_cfg;

	keys_cfg = devm_kzalloc(&pdev->dev, sizeof(struct keys_config), GFP_KERNEL);
	if (!keys_cfg)
		return -ENOMEM;

	keys_cfg->combo_data_size = dt_to_keycombo_data(pdev, keys_cfg);

	if (!keys_cfg->combo_data_size)
		goto error;

	keys_cfg->pcombo_data->priv = keys_cfg;
	keys_cfg->pdev_keycombo = platform_device_alloc(KEYCOMBO_NAME,
							PLATFORM_DEVID_AUTO);
	if (!keys_cfg->pdev_keycombo)
		goto error;

	keys_cfg->pdev_keycombo->dev.parent = &pdev->dev;

	ret = platform_device_add_data(keys_cfg->pdev_keycombo, keys_cfg->pcombo_data, keys_cfg->combo_data_size);
	if (ret)
		goto error;

	platform_set_drvdata(pdev, keys_cfg);
	return platform_device_add(keys_cfg->pdev_keycombo);

error:
	free_keycombo_platdata(keys_cfg);
	if (keys_cfg->pdev_keycombo)
		platform_device_put(keys_cfg->pdev_keycombo);
	devm_kfree(&pdev->dev, keys_cfg);

	return ret;
}

int amzn_keycombo_remove(struct platform_device *pdev)
{
	struct keys_config *keys_cfg = platform_get_drvdata(pdev);

	free_keycombo_platdata(keys_cfg);
	if (keys_cfg->pdev_keycombo)
		platform_device_put(keys_cfg->pdev_keycombo);

	devm_kfree(&pdev->dev, keys_cfg);
	return 0;
}

static struct of_device_id amzn_keycombo_dt_match[] = {
	{	.compatible = "amzn,keycombo",
	},
	{}
};

struct platform_driver amzn_keycombo_driver = {
	.remove = amzn_keycombo_remove,
	.driver = {
		.name = AMZN_KEYCOMBO_NAME,
		.owner = THIS_MODULE,
		.of_match_table = amzn_keycombo_dt_match,
	},
};

module_platform_driver_probe(amzn_keycombo_driver, amzn_keycombo_probe);

MODULE_LICENSE("GPL");
