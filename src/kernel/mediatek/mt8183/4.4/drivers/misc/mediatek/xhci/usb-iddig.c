/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/switch.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/jiffies.h>
#include <linux/list.h>
#include <linux/atomic.h>

#include <xhci-mtk-driver.h>

#ifdef CONFIG_USB_AMAZON_DOCK
#include <linux/switch.h>
#include <mt-plat/mtk_charger.h>
#include <mt-plat/mtk_battery.h>
#endif

#define RET_SUCCESS 0
#define RET_FAIL 1

static struct pinctrl *pinctrl;
static struct pinctrl_state *pinctrl_iddig_init;
static struct pinctrl_state *pinctrl_iddig_enable;
static struct pinctrl_state *pinctrl_iddig_disable;

#ifdef CONFIG_USB_MTK_OTG_SWITCH
static bool otg_switch_state;

static struct mutex otg_switch_mutex;
static ssize_t otg_mode_show(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t otg_mode_store(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count);

DEVICE_ATTR(otg_mode, 0664, otg_mode_show, otg_mode_store);

static struct attribute *otg_attributes[] = {
	&dev_attr_otg_mode.attr,
	NULL
};

static const struct attribute_group otg_attr_group = {
	.attrs = otg_attributes,
};

static const struct of_device_id otg_switch_of_match[] = {
	{.compatible = "mediatek,otg_switch"},
	{},
};

#endif

#ifdef CONFIG_USB_VBUS_GPIO
struct platform_device *g_pdev;
#endif
static const struct of_device_id otg_iddig_of_match[] = {
	{.compatible = "mediatek,usb_iddig_bi_eint"},
	{},
};

static bool otg_iddig_isr_enable;
static int mtk_xhci_eint_iddig_irq_en(void);

enum idpin_state {
	IDPIN_OUT,
	IDPIN_IN_HOST,
	IDPIN_IN_DEVICE,
#ifdef CONFIG_USB_AMAZON_DOCK
	IDPIN_IN_DOCK,
#endif
};

static int mtk_idpin_irqnum;
static enum idpin_state mtk_idpin_cur_stat = IDPIN_OUT;

static struct delayed_work mtk_xhci_delaywork;

#ifdef CONFIG_USB_AMAZON_DOCK
enum dock_mode_state {
	DOCK_NOT_PRESENT = 0,
	DOCK_PRESENT = 1,
	DOCK_UNPOWERED = 2,
};

enum dock_switch_state {
	TYPE_DOCKED = 5,
	TYPE_UNDOCKED = 6,
};

struct dock_data {
	int state;
	int iddig_state;
	struct switch_dev dock_state;
};

static struct dock_data *g_dock;
static DEFINE_MUTEX(dock_detection_mutex);
#endif


int mtk_iddig_debounce = 50;
module_param(mtk_iddig_debounce, int, 0400);

void switch_int_to_host_and_mask(void)
{
	irq_set_irq_type(mtk_idpin_irqnum, IRQF_TRIGGER_LOW);
	disable_irq(mtk_idpin_irqnum);
}

void switch_int_to_host(void)
{
	irq_set_irq_type(mtk_idpin_irqnum, IRQF_TRIGGER_LOW);
	enable_irq(mtk_idpin_irqnum);
}

static void mtk_set_iddig_out_detect(void)
{
	irq_set_irq_type(mtk_idpin_irqnum, IRQF_TRIGGER_HIGH);
	enable_irq(mtk_idpin_irqnum);
}

static void mtk_set_iddig_in_detect(void)
{
	irq_set_irq_type(mtk_idpin_irqnum, IRQF_TRIGGER_LOW);
	enable_irq(mtk_idpin_irqnum);
}

bool mtk_is_usb_id_pin_short_gnd(void)
{
	return (mtk_idpin_cur_stat != IDPIN_OUT) ? true : false;
}

void mtk_set_host_mode_in_host(void)
{
	mtk_idpin_cur_stat = IDPIN_IN_HOST;
}

void mtk_set_host_mode_out(void)
{
	mtk_idpin_cur_stat = IDPIN_OUT;
}


void mtk_enable_host(void)
{
	switch_int_to_host();	/* resotre ID pin interrupt */
}

void mtk_disable_host(void)
{
	switch_int_to_host_and_mask();
	if (mtk_is_host_mode() == true)
		mtk_xhci_driver_unload(true);
	mtk_idpin_cur_stat = IDPIN_OUT;
}

#ifdef CONFIG_USB_AMAZON_DOCK
#define VALID_VBUS_MV 4300
extern int mt_usb_phychk_extconn(void);
static int mtk_xhci_dock_detection(void)
{
	int type = 0, vbus_mv = 0;

	type = mt_usb_phychk_extconn();
	pr_info( "%s: charger_type = %d\n", __func__, type);
	if (type != STANDARD_CHARGER)
		return DOCK_NOT_PRESENT;

	vbus_mv = battery_get_vbus();
	pr_info("%s: vbus_mv = %d\n", __func__, vbus_mv);
	if (vbus_mv < VALID_VBUS_MV)
		return DOCK_UNPOWERED;

	return DOCK_PRESENT;
}

static bool mtk_xhci_dock_handler(int iddig_state)
{
	int is_dock_mode = 0;

	pr_info("%s: iddig_state = %d\n", __func__, iddig_state);
	mutex_lock(&dock_detection_mutex);
	if (iddig_state == 0) {
		g_dock->state = mtk_xhci_dock_detection();
		switch (g_dock->state) {
		case DOCK_PRESENT:
			pr_info("%s: Dock detected\n", __func__);
			switch_set_state(&g_dock->dock_state, TYPE_DOCKED);
			mtk_idpin_cur_stat = IDPIN_IN_DOCK;
			mtk_set_iddig_out_detect();
			is_dock_mode = 1;
			break;
		case DOCK_UNPOWERED:
			pr_info("%s: Unpowered Dock detected\n", __func__);
			mtk_idpin_cur_stat = IDPIN_IN_DOCK;
			mtk_set_iddig_out_detect();
			is_dock_mode = 1;
			break;
		default:
			break;
		}
	} else {
		switch (g_dock->state) {
		case DOCK_PRESENT:
			pr_info("%s: Dock removed\n", __func__);
			switch_set_state(&g_dock->dock_state, TYPE_UNDOCKED);
			mtk_idpin_cur_stat = IDPIN_OUT;
			mtk_set_iddig_in_detect();
			g_dock->state = DOCK_NOT_PRESENT;
			is_dock_mode = 1;
			break;
		case DOCK_UNPOWERED:
			pr_info("%s: Unpowered Dock removed\n", __func__);
			mtk_idpin_cur_stat = IDPIN_OUT;
			mtk_set_iddig_in_detect();
			g_dock->state = DOCK_NOT_PRESENT;
			is_dock_mode = 1;
			break;
		default:
			break;
		}
	}
	mutex_unlock(&dock_detection_mutex);

	pr_debug("%s: is_dock_mode = %d\n", __func__, is_dock_mode);
	return is_dock_mode;
}

void mtk_xhci_rerun_dock_detection(void)
{
	int vbus_mv = 0;

	if (!g_dock)
		return;

	mutex_lock(&dock_detection_mutex);
	switch (g_dock->state) {
	case DOCK_UNPOWERED:
		vbus_mv = battery_get_vbus();
		pr_info("%s: DOCK_UNPOWERED: vbus_mv: %d\n", __func__, vbus_mv);
		if (vbus_mv > VALID_VBUS_MV) {
			pr_info("%s: Dock detected: Unpowered -> Powered\n",
					__func__);
			g_dock->state = DOCK_PRESENT;
			switch_set_state(&g_dock->dock_state, TYPE_DOCKED);
		}
		break;
	case DOCK_PRESENT:
		vbus_mv = battery_get_vbus();
		pr_info("%s: DOCK_PRESENT: vbus_mv: %d\n", __func__, vbus_mv);
		if (vbus_mv < VALID_VBUS_MV) {
			pr_info("%s: Dock removed: Powered -> Unpowered\n",
					__func__);
			g_dock->state = DOCK_UNPOWERED;
			switch_set_state(&g_dock->dock_state, TYPE_UNDOCKED);
		}
		break;
	default:
		pr_debug("%s: not in dock mode\n", __func__);
		break;
	}
	mutex_unlock(&dock_detection_mutex);
}
#endif

void mtk_xhci_mode_switch(struct work_struct *work)
{
	static bool is_load;
	int ret = 0;

	mtk_xhci_mtk_printk(K_DEBUG, "mtk_xhci_mode_switch\n");

#ifdef CONFIG_USB_AMAZON_DOCK
	/* Dock detection */
	if (mtk_xhci_dock_handler(mtk_idpin_cur_stat))
		return;
#endif

	if (mtk_idpin_cur_stat == 0) {
		is_load = false;

		/* expect next isr is for id-pin out action */
		mtk_idpin_cur_stat = (mtk_is_charger_4_vol()) ? IDPIN_IN_DEVICE : IDPIN_IN_HOST;
		/* make id pin to detect the plug-out */
		mtk_set_iddig_out_detect();

		if (mtk_idpin_cur_stat == IDPIN_IN_DEVICE)
			goto done;

		ret = mtk_xhci_driver_load(true);
		if (!ret)
			is_load = true;
	} else {
		if (is_load) {
			mtk_xhci_driver_unload(true);
			is_load = false;
		}

		/* expect next isr is for id-pin in action */
		mtk_idpin_cur_stat = IDPIN_OUT;
		/* make id pin to detect the plug-in */
		mtk_set_iddig_in_detect();
	}

done:
	mtk_xhci_mtk_printk(K_ALET, "current mode is %s, ret(%d)\n",
			 (mtk_idpin_cur_stat == IDPIN_IN_HOST) ? "host" :
			 (mtk_idpin_cur_stat == IDPIN_IN_DEVICE) ? "id_device" : "device",
			 ret);
}

static irqreturn_t xhci_eint_iddig_isr(int irqnum, void *data)
{
	disable_irq_nosync(irqnum);
	schedule_delayed_work(&mtk_xhci_delaywork, msecs_to_jiffies(mtk_iddig_debounce));
	mtk_xhci_mtk_printk(K_DEBUG, "xhci_eint_iddig_isr\n");

	return IRQ_HANDLED;
}

static int mtk_xhci_eint_iddig_irq_en(void)
{
	int retval = 0;

	if (!otg_iddig_isr_enable) {
		retval =
			request_irq(mtk_idpin_irqnum, xhci_eint_iddig_isr, IRQF_TRIGGER_LOW, "iddig_eint",
			NULL);

		if (retval != 0) {
			mtk_xhci_mtk_printk(K_ERR, "request_irq fail, ret %d, irqnum %d!!!\n", retval,
					 mtk_idpin_irqnum);
		} else {
			otg_iddig_isr_enable = true;
		}
	} else {
#ifdef CONFIG_USB_MTK_OTG_SWITCH
		switch_int_to_host();	/* restore ID pin interrupt */
#endif
	}
	return retval;
}

static int otg_iddig_probe(struct platform_device *pdev)
{
	int retval = 0;
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	int iddig_gpio, iddig_debounce;
	u32 ints[2] = {0, 0};

#ifdef CONFIG_USB_VBUS_GPIO
	g_pdev = pdev;
#endif

	mtk_idpin_irqnum = irq_of_parse_and_map(node, 0);
	if (mtk_idpin_irqnum < 0)
		return -ENODEV;

	pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(pinctrl)) {
		dev_err(&pdev->dev, "Cannot find usb pinctrl!\n");
		return -1;
	}

	pinctrl_iddig_init = pinctrl_lookup_state(pinctrl, "iddig_init");
	if (IS_ERR(pinctrl_iddig_init))
		dev_err(&pdev->dev, "Cannot find usb pinctrl iddig_init\n");
	else
		pinctrl_select_state(pinctrl, pinctrl_iddig_init);


	pinctrl_iddig_enable = pinctrl_lookup_state(pinctrl, "iddig_enable");
	pinctrl_iddig_disable = pinctrl_lookup_state(pinctrl, "iddig_disable");
	if (IS_ERR(pinctrl_iddig_enable))
		dev_err(&pdev->dev, "Cannot find usb pinctrl iddig_enable\n");

	if (IS_ERR(pinctrl_iddig_disable))
		dev_err(&pdev->dev, "Cannot find usb pinctrl iddig_disable\n");

	retval = of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
	if (!retval) {
		iddig_gpio = ints[0];
		iddig_debounce = ints[1];
		mtk_xhci_mtk_printk(K_DEBUG, "iddig gpio num = %d\n", mtk_idpin_irqnum);
		/*mt_gpio_set_debounce(iddig_gpio, iddig_debounce);*/
	}

	INIT_DELAYED_WORK(&mtk_xhci_delaywork, mtk_xhci_mode_switch);

#ifdef CONFIG_USB_MTK_OTG_SWITCH
#ifdef CONFIG_SYSFS
	retval = sysfs_create_group(&pdev->dev.kobj, &otg_attr_group);
	if (retval < 0) {
		dev_err(&pdev->dev, "Cannot register USB bus sysfs attributes: %d\n",
			retval);
		return retval;
	}
	mutex_init(&otg_switch_mutex);
#endif
#else
	retval = mtk_xhci_eint_iddig_irq_en();
#endif

#ifdef CONFIG_USB_AMAZON_DOCK
	g_dock = devm_kzalloc(&pdev->dev, sizeof(struct dock_data), GFP_KERNEL);
	if (!g_dock)
		return -ENOMEM;

	g_dock->dock_state.name = "dock";
	g_dock->dock_state.index = 0;
	g_dock->dock_state.state = TYPE_UNDOCKED;
	retval = switch_dev_register(&g_dock->dock_state);
	if (retval) {
		dev_err(&pdev->dev, "switch_dev_register dock_state Fail\n");
		return retval;
	}
#endif
	return 0;
}

static int otg_iddig_remove(struct platform_device *pdev)
{
	if (otg_iddig_isr_enable) {
		disable_irq_nosync(mtk_idpin_irqnum);
		free_irq(mtk_idpin_irqnum, NULL);
		otg_iddig_isr_enable = false;
	}

	cancel_delayed_work(&mtk_xhci_delaywork);
	mtk_idpin_cur_stat = IDPIN_OUT;

	mtk_xhci_mtk_printk(K_DEBUG, "external iddig unregister done.\n");

#ifdef CONFIG_USB_MTK_OTG_SWITCH
#ifdef CONFIG_SYSFS
	sysfs_remove_group(&pdev->dev.kobj, &otg_attr_group);
#endif
#endif

#ifdef CONFIG_USB_AMAZON_DOCK
	switch_dev_unregister(&g_dock->dock_state);
#endif
	return 0;
}

static void otg_iddig_shutdown(struct platform_device *pdev)
{
	if (mtk_is_host_mode() == true) {
		mtk_xhci_disable_vbus();
		mtk_xhci_mtk_printk(K_ALET, "otg_disable_vbus\n");
	}
}

#ifdef CONFIG_PM
static int otg_iddig_suspend(struct platform_device *dev, pm_message_t state)
{
	disable_irq(mtk_idpin_irqnum);

	cancel_delayed_work_sync(&mtk_xhci_delaywork);

	if (!IS_ERR(pinctrl_iddig_disable))
		pinctrl_select_state(pinctrl, pinctrl_iddig_disable);
	return 0;
}
static int otg_iddig_resume(struct platform_device *dev)
{
	if (!IS_ERR(pinctrl_iddig_disable))
		pinctrl_select_state(pinctrl, pinctrl_iddig_enable);

	enable_irq(mtk_idpin_irqnum);

	return 0;
}
#endif

static struct platform_driver otg_iddig_driver = {
	.probe = otg_iddig_probe,
	.remove = otg_iddig_remove,
	.shutdown = otg_iddig_shutdown,
#ifdef CONFIG_PM
	.suspend = otg_iddig_suspend,
	.resume = otg_iddig_resume,
#endif
	.driver = {
		.name = "otg_iddig",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(otg_iddig_of_match),
	},
};

static int __init otg_iddig_init(void)
{
	return platform_driver_register(&otg_iddig_driver);
}
late_initcall(otg_iddig_init);

static void __exit otg_iddig_cleanup(void)
{
	platform_driver_unregister(&otg_iddig_driver);
}

module_exit(otg_iddig_cleanup);


#ifdef CONFIG_USB_MTK_OTG_SWITCH
static ssize_t otg_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	if (!dev) {
		dev_err(dev, "otg_mode_store no dev\n");
		return 0;
	}

	return sprintf(buf, "%d\n", otg_switch_state);
}

static ssize_t otg_mode_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	unsigned int mode;

	if (!dev) {
		dev_err(dev, "otg_mode_store no dev\n");
		return count;
	}

	mutex_lock(&otg_switch_mutex);
	if (sscanf(buf, "%ud", &mode) == 1) {
		if (mode == 0) {
			mtk_xhci_mtk_printk(K_DEBUG, "otg_mode_enable start\n");
			if (otg_switch_state == true) {
				if (otg_iddig_isr_enable) {
					disable_irq(mtk_idpin_irqnum);
					irq_set_irq_type(mtk_idpin_irqnum, IRQF_TRIGGER_LOW);
				}
				cancel_delayed_work_sync(&mtk_xhci_delaywork);
				if (mtk_is_host_mode() == true)
					mtk_xhci_driver_unload(true);
				mtk_idpin_cur_stat = IDPIN_OUT;


				if (!IS_ERR(pinctrl_iddig_disable))
					pinctrl_select_state(pinctrl, pinctrl_iddig_disable);

				otg_switch_state = false;
			}
			mtk_xhci_mtk_printk(K_DEBUG, "otg_mode_enable end\n");
		} else {
			mtk_xhci_mtk_printk(K_DEBUG, "otg_mode_disable start\n");
			if (otg_switch_state == false) {
				otg_switch_state = true;
				if (!IS_ERR(pinctrl_iddig_enable))
					pinctrl_select_state(pinctrl, pinctrl_iddig_enable);

				msleep(20);
				mtk_xhci_eint_iddig_irq_en();
			}
			mtk_xhci_mtk_printk(K_DEBUG, "otg_mode_disable end\n");
		}
	}
	mutex_unlock(&otg_switch_mutex);
	return count;
}
#endif




