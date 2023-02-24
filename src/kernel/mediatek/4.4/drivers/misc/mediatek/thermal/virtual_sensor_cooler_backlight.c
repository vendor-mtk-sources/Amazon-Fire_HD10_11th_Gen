#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt


#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/printk.h>
#include <linux/types.h>
#include <linux/kobject.h>
#include <linux/slab.h>

#include "mt-plat/mtk_thermal_monitor.h"
#include <linux/thermal_framework.h>

#define virtual_sensor_cooler_backlight_dprintk(fmt, args...) pr_debug("thermal/cooler/backlight " fmt, ##args)

#define MAX_BRIGHTNESS		255
#define DRIVER_NAME "backlight_cooler"

static struct mutex bl_update_lock;
struct cooler_sort_list backlight_list_head;

/*
 * Weak functions
 */
int __attribute__ ((weak)) setMaxbrightness(int max_level, int enable)
{
	pr_err("E_WF: %s doesn't exist\n", __func__);
	return 0;
}

static void thermal_parse_of_int(const struct device_node *np,
			const char *node_name, int *cust_val)
{
	u32 val = 0;

	if (!of_property_read_u32(np, node_name, &val)) {
		(*cust_val) = (int)val;
		pr_debug("Get %s %d\n", node_name, *cust_val);
	} else {
		pr_notice("Get %s failed\n", node_name);
	}
}

static void thermal_parse_of_int_array(const struct device_node *np,
			const char *node_name, int *trips_data, size_t num_levels)
{
	u32 *level_array = kzalloc(sizeof(u32) * num_levels, GFP_KERNEL);
	int i = 0;

	if (!of_property_read_u32_array(np, node_name, level_array, num_levels)) {
		for (i = 0; i < num_levels; i++) {
			trips_data[i] = level_array[i];
			pr_debug("Get %s %d\n", node_name, trips_data[i]);
		}
	} else {
		pr_notice("Get %s failed\n", node_name);
	}
	kfree(level_array);
}

static void thermal_parse_of_string_index(struct device_node *np,
			const char *node_name, int index, char *cust_string)
{
	const char *prop_string;

	if (!of_property_read_string_index(np, node_name, index, &prop_string)) {
		strcpy(cust_string, prop_string);
		pr_debug("Get %s %s\n", node_name, cust_string);
	} else {
		pr_notice("Get %s failed\n", node_name);
	}
}

static void cooler_init_from_dt(struct platform_device *dev,
				struct mtk_cooler_platform_data *pcdata)
{
	struct device_node *np = dev->dev.of_node;

	thermal_parse_of_string_index(np, "type", 0, (char *)&pcdata->type);
	thermal_parse_of_int(np,"state", (int *)&pcdata->state);
	thermal_parse_of_int(np,"max_state", (int *)&pcdata->max_state);
	thermal_parse_of_int(np, "level", &pcdata->level);
	thermal_parse_of_int_array(np, "levels", pcdata->levels, pcdata->max_state);
}

static int virtual_sensor_get_max_state(struct thermal_cooling_device *cdev,
			unsigned long *state)
{
	struct mtk_cooler_platform_data *pdata = cdev->devdata;
	*state = pdata->max_state;
	return 0;
}

static int virtual_sensor_get_cur_state(struct thermal_cooling_device *cdev,
			unsigned long *state)
{
	struct mtk_cooler_platform_data *pdata = cdev->devdata;
	*state = pdata->state;
	return 0;
}

static int virtual_sensor_set_cur_state(struct thermal_cooling_device *cdev,
			unsigned long state)
{
	int level;
	unsigned long max_state;
	struct mtk_cooler_platform_data *pdata = cdev->devdata;

	if (!cdev)
		return 0;

	mutex_lock(&(bl_update_lock));

	if (pdata->state == state)
		goto out;

	max_state = pdata->max_state;
	pdata->state = (state > max_state) ? max_state : state;

	if (!pdata->state)
		level = MAX_BRIGHTNESS;
	else
		level = pdata->levels[pdata->state - 1];

	if (level == pdata->level)
		goto out;

	pdata->level = level;

	level = thermal_level_compare(pdata, &backlight_list_head, true);

	setMaxbrightness(level, 1);

out:
	mutex_unlock(&(bl_update_lock));

	return 0;
}

static ssize_t levels_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct thermal_cooling_device *cdev =
		container_of(dev, struct thermal_cooling_device, device);
	struct mtk_cooler_platform_data *pdata = cdev->devdata;
	int i;
	int offset = 0;

	if (!pdata->cdev)
		return -EINVAL;
	for (i = 0; i < THERMAL_MAX_TRIPS; i++)
		offset += sprintf(buf + offset, "%d %d\n", i+1, pdata->levels[i]);
	return offset;
}

static ssize_t levels_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int level, state;
	struct thermal_cooling_device *cdev =
		container_of(dev, struct thermal_cooling_device, device);
	struct mtk_cooler_platform_data *pdata = cdev->devdata;

	if (!pdata->cdev)
		return -EINVAL;
	if (sscanf(buf, "%d %d\n", &state, &level) != 2)
		return -EINVAL;
	if (state >= THERMAL_MAX_TRIPS)
		return -EINVAL;
	pdata->levels[state] = level;
	return count;
}

static struct thermal_cooling_device_ops cooling_ops = {
	.get_max_state = virtual_sensor_get_max_state,
	.get_cur_state = virtual_sensor_get_cur_state,
	.set_cur_state = virtual_sensor_set_cur_state,
};

static DEVICE_ATTR(levels, S_IRUGO | S_IWUSR, levels_show, levels_store);

static int backlight_probe(struct platform_device *pdev)
{
	struct mtk_cooler_platform_data *pdata = NULL;
	struct cooler_sort_list *tz_list = NULL;

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

#ifdef CONFIG_OF
	pr_notice("cooler custom init by DTS!\n");
	cooler_init_from_dt(pdev, pdata);
#endif

	pdata->cdev = thermal_cooling_device_register(pdata->type,
							pdata,
							&cooling_ops);
	if (!pdata->cdev) {
		pr_err("%s Failed to create backlight cooling device\n",
			__func__);
		return -EINVAL;
	}

	device_create_file(&pdata->cdev->device, &dev_attr_levels);

	tz_list = kzalloc(sizeof(struct cooler_sort_list), GFP_KERNEL);
	tz_list->pdata = pdata;
	list_add(&tz_list->list, &backlight_list_head.list);

	platform_set_drvdata(pdev, pdata);
	return 0;
}

static int backlight_remove(struct platform_device *pdev)
{
	struct mtk_cooler_platform_data *pdata = platform_get_drvdata(pdev);
	struct cooler_sort_list *cooler_list, *tmp;

	if (pdata) {
		thermal_cooling_device_unregister(pdata->cdev);
		devm_kfree(&pdev->dev, pdata);
	}

	list_for_each_entry_safe(cooler_list, tmp, &backlight_list_head.list, list) {
		if (cooler_list->pdata == pdata) {
			list_del(&cooler_list->list);
			kfree(cooler_list);
			cooler_list = NULL;
			break;
		}
	}

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id backlight_of_match_table[] = {
	{.compatible = "amazon,backlight_cooler", },
	{},
};
MODULE_DEVICE_TABLE(of, backlight_of_match_table);
#endif

static struct platform_driver backlight_driver = {
	.probe = backlight_probe,
	.remove = backlight_remove,
	.driver     = {
		.name  = DRIVER_NAME,
#ifdef CONFIG_OF
		.of_match_table = backlight_of_match_table,
#endif
		.owner = THIS_MODULE,
	},
};


static int __init virtual_sensor_cooler_backlight_init(void)
{
	int err = 0;

	virtual_sensor_cooler_backlight_dprintk("init\n");

	INIT_LIST_HEAD(&backlight_list_head.list);

	mutex_init(&(bl_update_lock));

	err = platform_driver_register(&backlight_driver);
	if (err) {
		pr_err("%s: Failed to register driver %s\n", __func__,
			backlight_driver.driver.name);
		return err;
	}

	return 0;
}

static void __exit virtual_sensor_cooler_backlight_exit(void)
{
	virtual_sensor_cooler_backlight_dprintk("exit\n");

	platform_driver_unregister(&backlight_driver);
	mutex_destroy(&(bl_update_lock));
}
module_init(virtual_sensor_cooler_backlight_init);
module_exit(virtual_sensor_cooler_backlight_exit);
