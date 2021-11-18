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
#include <linux/platform_device.h>
#include <linux/thermal_framework.h>

#include "mt-plat/mtk_thermal_monitor.h"
#include <mt-plat/battery_common.h>

#define DRIVER_NAME "battery_current_cooler"

#define virtual_sensor_cooler_bccl_dprintk(fmt, args...) pr_debug("thermal/cooler/bcct " fmt, ##args)

static struct mutex bcct_updata_lock;
struct cooler_sort_list bcct_list_head;

/*
 * Weak functions
 */
unsigned int __attribute__ ((weak)) set_bat_charging_current_limit(int current_limit)
{
	pr_err("E_WF: %s doesn't exist\n", __func__);
	return 0;
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

	mutex_lock(&(bcct_updata_lock));

	if (pdata->state == state)
		goto out;

	max_state = pdata->max_state;
	pdata->state = (state > max_state) ? max_state : state;

	if (!pdata->state)
		level = CHARGE_CURRENT_MAX;
	else
		level = pdata->levels[pdata->state - 1];

	if (level == pdata->level)
		goto out;

	pdata->level = level;

	level = thermal_level_compare(pdata, &bcct_list_head, true);

	if (level >= CHARGE_CURRENT_MAX)
		set_bat_charging_current_limit(-1);
	else
		set_bat_charging_current_limit(level/100);

out:
	mutex_unlock(&(bcct_updata_lock));

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

static int bcct_probe(struct platform_device *pdev)
{
	struct mtk_cooler_platform_data *pdata = NULL;
	struct cooler_sort_list *tz_list = NULL;

	virtual_sensor_cooler_bccl_dprintk("probe\n");

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

#ifdef CONFIG_OF
	pr_notice("cooler custom init by DTS!\n");
	cooler_init_cust_data_from_dt(pdev, pdata);
#endif

	pdata->cdev = thermal_cooling_device_register(pdata->type,
							pdata,
							&cooling_ops);
	if (!pdata->cdev) {
		pr_err("%s Failed to create bcct cooling device\n",
			__func__);
		return -EINVAL;
	}

	device_create_file(&pdata->cdev->device, &dev_attr_levels);

	tz_list = kzalloc(sizeof(struct cooler_sort_list), GFP_KERNEL);
	tz_list->pdata = pdata;
	list_add(&tz_list->list, &bcct_list_head.list);

	platform_set_drvdata(pdev, pdata);
	return 0;
}

static int bcct_remove(struct platform_device *pdev)
{
	struct mtk_cooler_platform_data *pdata = platform_get_drvdata(pdev);
	struct cooler_sort_list *cooler_list, *tmp;

	virtual_sensor_cooler_bccl_dprintk("remove\n");
	if (pdata) {
		thermal_cooling_device_unregister(pdata->cdev);
		devm_kfree(&pdev->dev, pdata);
	}


	list_for_each_entry_safe(cooler_list, tmp, &bcct_list_head.list, list) {
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
static const struct of_device_id bcct_of_match_table[] = {
	{.compatible = "amazon,battery_current_cooler", },
	{},
};
MODULE_DEVICE_TABLE(of, bcct_of_match_table);
#endif

static struct platform_driver bcct_driver = {
	.probe = bcct_probe,
	.remove = bcct_remove,
	.driver     = {
		.name  = DRIVER_NAME,
#ifdef CONFIG_OF
		.of_match_table = bcct_of_match_table,
#endif
		.owner = THIS_MODULE,
	},
};

static int __init virtual_sensor_cooler_bcct_init(void)
{
	int err = 0;

	virtual_sensor_cooler_bccl_dprintk("init\n");

	INIT_LIST_HEAD(&bcct_list_head.list);

	mutex_init(&(bcct_updata_lock));

	err = platform_driver_register(&bcct_driver);
	if (err) {
		pr_err("%s: Failed to register driver %s\n", __func__,
			bcct_driver.driver.name);
		return err;
	}

	return 0;
}

static void __exit virtual_sensor_cooler_bcct_exit(void)
{
	virtual_sensor_cooler_bccl_dprintk("exit\n");

	platform_driver_unregister(&bcct_driver);

	mutex_destroy(&(bcct_updata_lock));
}

module_init(virtual_sensor_cooler_bcct_init);
module_exit(virtual_sensor_cooler_bcct_exit);
