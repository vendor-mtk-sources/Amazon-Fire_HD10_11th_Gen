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
#include <linux/of_gpio.h>
#include <linux/jiffies.h>
#include <linux/list.h>
#include <linux/atomic.h>
#include <linux/gpio.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <xhci-mtk-driver.h>
#include <mtk-phy-ext.h>
#include <musb_core.h>

struct usb_vbusd {
	struct device	*dev;	/* the "inherit" device */

	/* parameters */
	unsigned int	gpio_dts;
	unsigned int	gpio_rise_debounce;
	unsigned int	gpio_fall_debounce;
	unsigned int	gpio_polarity;
	unsigned int	gpio_iddts;

	/* internel parameters */
	unsigned long	gpio_irq_count;
	unsigned int	gpio_pin;
	unsigned int	gpio_irq;
	unsigned int	gpio_val;
	unsigned int	gpio_idpin;

	unsigned int	gpio_state;

	unsigned int	gpio_trig;
	unsigned int	gpio_irqf[2];

	unsigned int	gpio_debounce;
	struct delayed_work gpio_workqueue;

#ifdef CONFIG_USB_PHYCHK_EXTCONN
	int		gpio_cust_linetype;
#endif

	spinlock_t	gpio_lock;
	struct mutex	gpio_mutex;

#ifdef CONFIG_DEBUG_FS
	struct dentry *debugfs_root;
#endif
};

static struct usb_vbusd *mt6771_usb_vbusd;
unsigned int vbusd_debounce_time = 500;
module_param(vbusd_debounce_time, int, 0644);
static struct usb_vbusd *mtk_vbusd_get_handle(void)
{
	struct usb_vbusd *vbusd = ERR_PTR(-ENODEV);

	if (mt6771_usb_vbusd)
		vbusd = mt6771_usb_vbusd;

	return vbusd;
}

int mtk_vbusd_get_linetype(void)
{
	int ret;
	struct usb_vbusd *vbusd = NULL;

	vbusd = mtk_vbusd_get_handle();
	if (IS_ERR(vbusd))
		ret = PTR_ERR(vbusd);
	else {
		mutex_lock(&vbusd->gpio_mutex);
		ret = vbusd->gpio_cust_linetype;
		mutex_unlock(&vbusd->gpio_mutex);
	}

	return ret;
}

int mtk_vbusd_enable(bool on)
{
	int ret = 0;
	unsigned long flags, irq_flags;
	struct usb_vbusd *vbusd = NULL;

	vbusd = mtk_vbusd_get_handle();
	if (IS_ERR(vbusd))
		ret = PTR_ERR(vbusd);
	else {
		dev_info(vbusd->dev,
			"[vbusd]%s intr:  %i @LN%i\n",
			__func__, on, __LINE__);

		flush_delayed_work(&vbusd->gpio_workqueue);
		mutex_lock(&vbusd->gpio_mutex);
		spin_lock_irqsave(&vbusd->gpio_lock, flags);

		if (!on) {
			disable_irq_nosync(vbusd->gpio_irq);
		} else {
			vbusd->gpio_trig = vbusd->gpio_polarity;
			irq_flags = vbusd->gpio_irqf[vbusd->gpio_trig];
			irq_set_irq_type(vbusd->gpio_irq, irq_flags);
			enable_irq(vbusd->gpio_irq);
		}

		spin_unlock_irqrestore(&vbusd->gpio_lock, flags);
		mutex_unlock(&vbusd->gpio_mutex);
	}

	return ret;
}

int mtk_vbusd_vbus_status(void)
{
	int ret;
	int vbus_present = 0;
	struct usb_vbusd *vbusd = NULL;

	vbusd = mtk_vbusd_get_handle();
	if (IS_ERR(vbusd)) {
		ret = PTR_ERR(vbusd);
	} else {
		mutex_lock(&vbusd->gpio_mutex);
		ret = gpio_get_value(vbusd->gpio_pin);
		mutex_unlock(&vbusd->gpio_mutex);
		vbus_present = (ret == vbusd->gpio_polarity) ? 1 : 0;
	}

	return vbus_present;
}

#ifdef CONFIG_USB_PHYCHK_EXTCONN
static void mtk_vbusd_workqueue(struct work_struct *data)
{
	int id = 0;
	int val = 0;
	struct usb_vbusd *vbusd = mtk_vbusd_get_handle();
	unsigned long flags, irq_flags;
	char const *sztrigger[2] = {
		"IRQF_TRIGGER_LOW",
		"IRQF_TRIGGER_HIGH"
	};
	int linetype = MT_USB_EXTCONN_UNKNOWN;


	mutex_lock(&vbusd->gpio_mutex);
	id = gpio_get_value(vbusd->gpio_idpin);
	mutex_unlock(&vbusd->gpio_mutex);

	dev_info(vbusd->dev,
		"[vbusd]%s event: id(pin: %u) = %i @LN%i\n",
		__func__, id, vbusd->gpio_iddts, __LINE__);


	mutex_lock(&vbusd->gpio_mutex);
	val = gpio_get_value(vbusd->gpio_pin);
	if (val == vbusd->gpio_polarity) {
		dev_info(vbusd->dev,
			"[vbusd]%s (%s) event: gpio%u: %i, %s@LN%i\n",
			__func__,
			vbusd->gpio_state ? "connect" : "disconnect",
			vbusd->gpio_dts, val,
			sztrigger[val], __LINE__);
		if (id) {
			linetype = mt_usb_phychk_extconn();
			if (linetype == MT_USB_EXTCONN_STANDARDHOST) {
				spin_lock_irqsave(&vbusd->gpio_lock, flags);
				vbusd->gpio_cust_linetype = linetype;
				spin_unlock_irqrestore(&vbusd->gpio_lock,
						       flags);

				mt_usb_connect();
			}
			vbusd->gpio_state = 1;
		} else
			dev_info(vbusd->dev,
			"[vbusd]gpio%u is not wired with vBus in HOST Mode!\n",
			vbusd->gpio_dts);

	} else {
		dev_info(vbusd->dev,
			"[vbusd]%s (%s) event: gpio%u: %i, %s@LN%i\n",
			__func__,
			vbusd->gpio_state ? "connect" : "disconnect",
			vbusd->gpio_dts, val,
			sztrigger[val], __LINE__);
		if (id || vbusd->gpio_state) {
			mt_usb_disconnect();
			spin_lock_irqsave(&vbusd->gpio_lock, flags);
			vbusd->gpio_cust_linetype = MT_USB_EXTCONN_UNKNOWN;
			spin_unlock_irqrestore(&vbusd->gpio_lock, flags);
			vbusd->gpio_state = 0;
		} else
		   dev_info(vbusd->dev,
		   "[vbusd]gpio%u is not wired with vBus in HOST Mode!\n",
		   vbusd->gpio_dts);
	}

	spin_lock_irqsave(&vbusd->gpio_lock, flags);

	if (val)
		vbusd->gpio_debounce = vbusd->gpio_fall_debounce;
	else
		vbusd->gpio_debounce = vbusd->gpio_rise_debounce;

	vbusd->gpio_trig = val ? 0 : 1;
	irq_flags = vbusd->gpio_irqf[vbusd->gpio_trig];
	irq_set_irq_type(vbusd->gpio_irq, irq_flags);
	enable_irq(vbusd->gpio_irq);

	spin_unlock_irqrestore(&vbusd->gpio_lock, flags);

	mutex_unlock(&vbusd->gpio_mutex);

}
#else
static void mtk_vbusd_workqueue(struct work_struct *data)
{
	int id = 0;
	int val = 0;
	struct usb_vbusd *vbusd = mtk_vbusd_get_handle();
	unsigned long flags, irq_flags;
	char const *sztrigger[2] = {
		"IRQF_TRIGGER_LOW",
		"IRQF_TRIGGER_HIGH"
	};

	mutex_lock(&vbusd->gpio_mutex);
	id = gpio_get_value(vbusd->gpio_idpin);
	mutex_unlock(&vbusd->gpio_mutex);

	dev_info(vbusd->dev,
		"[vbusd]%s event: id(pin: %u) = %i, @LN%i\n",
		__func__, id, vbusd->gpio_iddts, __LINE__);


	mutex_lock(&vbusd->gpio_mutex);
	val = gpio_get_value(vbusd->gpio_pin);
	if (val == vbusd->gpio_polarity) {
		dev_info(vbusd->dev,
			"[vbusd]%s (%s) event: gpio%u: %i, %s@LN%i\n",
			__func__,
			vbusd->gpio_state ? "connect" : "disconnect",
			vbusd->gpio_dts, val,
			sztrigger[val], __LINE__);
		if (id) {
			mt_usb_connect();
			vbusd->gpio_state = 1;
		} else
			dev_info(vbusd->dev,
			"[vbusd]gpio%u is not wired with vBus in HOST Mode!\n",
			vbusd->gpio_dts);

	} else {
		dev_info(vbusd->dev,
			"[vbusd]%s (%s) event: gpio%u: %i, %s@LN%i\n",
			__func__,
			vbusd->gpio_state ? "connect" : "disconnect",
			vbusd->gpio_dts, val,
			sztrigger[val], __LINE__);
		if (id || vbusd->gpio_state) {
			mt_usb_disconnect();
			vbusd->gpio_state = 0;
		} else
		   dev_info(vbusd->dev,
		   "[vbusd]gpio%u is not wired with vBus in HOST Mode!\n",
		   vbusd->gpio_dts);
	}

	spin_lock_irqsave(&vbusd->gpio_lock, flags);

	if (val)
		vbusd->gpio_debounce = vbusd->gpio_fall_debounce;
	else
		vbusd->gpio_debounce = vbusd->gpio_rise_debounce;

	vbusd->gpio_trig = val ? 0 : 1;
	irq_flags = vbusd->gpio_irqf[vbusd->gpio_trig];
	irq_set_irq_type(vbusd->gpio_irq, irq_flags);
	enable_irq(vbusd->gpio_irq);

	spin_unlock_irqrestore(&vbusd->gpio_lock, flags);

	mutex_unlock(&vbusd->gpio_mutex);

}
#endif

static irqreturn_t mtk_vbusd_intr(int irq, void *dev_id)
{
	int ret;
	struct usb_vbusd *vbusd = NULL;

	vbusd = mtk_vbusd_get_handle();
	if (IS_ERR(vbusd))
		ret = PTR_ERR(vbusd);
	else {
		disable_irq_nosync(vbusd->gpio_irq);
		spin_lock(&vbusd->gpio_lock);
		vbusd->gpio_irq_count++;
		spin_unlock(&vbusd->gpio_lock);

		schedule_delayed_work(&vbusd->gpio_workqueue,
				msecs_to_jiffies(vbusd->gpio_debounce));

		dev_info(vbusd->dev, "[vbusd] vbus pin interrupt assert\n");
	}

	return IRQ_HANDLED;
}

#ifdef CONFIG_DEBUG_FS
#ifdef CONFIG_USB_PHYCHK_EXTCONN
static int mtk_vbusd_show(struct seq_file *s, void *v)
{
	int id = 0;
	struct usb_vbusd *vbusd = s->private;
	char * const szlinetype[] = {
		"UNKNOWN LINE TYPE",
		"STANDARD_HOST",
		"CHARGING_HOST",
		"NONSTANDARD_CHARGER",
		"STANDARD_CHARGER",
		"INVALID PARAMETER"
	};

	seq_printf(s,
		   "\n===mtk usb vbusd gpio%u debug information===\n",
		   vbusd->gpio_dts);
	seq_printf(s,
		   "[HW]gpio%u = %u!\n",
		   vbusd->gpio_dts, gpio_get_value(vbusd->gpio_pin));

	/* parameters */
	seq_printf(s, "[CMD]vbusd_debounce_time : %u ms\n",
		   vbusd_debounce_time);

	seq_printf(s, "[SW]gpio_iddts	: %u\n", vbusd->gpio_iddts);
	seq_printf(s, "[SW]gpio_dts		: %u\n", vbusd->gpio_dts);
	seq_printf(s, "[SW]gpio_rise_debounce	: %u ms\n",
		   vbusd->gpio_rise_debounce);
	seq_printf(s, "[SW]gpio_fall_debounce	: %u ms\n",
		   vbusd->gpio_fall_debounce);
	seq_printf(s, "[SW]gpio_polarity	: %u\n",
		   vbusd->gpio_polarity);

	/* internel parameters */
	seq_printf(s, "[ST]gpio_val		: %u\n", vbusd->gpio_val);
	seq_printf(s, "[ST]gpio_irq_count	: %lu\n",
		   vbusd->gpio_irq_count);
	seq_printf(s, "[ST]gpio_trig		: %u\n", vbusd->gpio_trig);

	/* current result */
	id = gpio_get_value(vbusd->gpio_idpin);
	seq_printf(s, "[FINAL][%s] gpio_cust_linetype: %s(%u)\n",
			id ? "Device" : "Host",
			szlinetype[vbusd->gpio_cust_linetype],
			vbusd->gpio_cust_linetype);

	return 0;
}
#else
static int mtk_vbusd_show(struct seq_file *s, void *v)
{
	int id = 0;
	struct usb_vbusd *vbusd = s->private;
	int vbus = 0;

	seq_printf(s,
		   "\n===mtk usb vbusd gpio%u debug information===\n",
		   vbusd->gpio_dts);
	seq_printf(s,
		   "[HW]gpio%u = %u!\n",
		   vbusd->gpio_dts, gpio_get_value(vbusd->gpio_pin));

	/* parameters */
	seq_printf(s, "[CMD]vbusd_debounce_time : %u ms\n",
		   vbusd_debounce_time);

	seq_printf(s, "[SW]gpio_iddts	: %u\n", vbusd->gpio_iddts);
	seq_printf(s, "[SW]gpio_dts		: %u\n", vbusd->gpio_dts);
	seq_printf(s, "[SW]gpio_rise_debounce	: %u ms\n",
		   vbusd->gpio_rise_debounce);
	seq_printf(s, "[SW]gpio_fall_debounce	: %u ms\n",
		   vbusd->gpio_fall_debounce);
	seq_printf(s, "[SW]gpio_polarity	: %u\n",
		   vbusd->gpio_polarity);

	/* internel parameters */
	seq_printf(s, "[ST]gpio_val		: %u\n", vbusd->gpio_val);
	seq_printf(s, "[ST]gpio_irq_count	: %lu\n",
		   vbusd->gpio_irq_count);
	seq_printf(s, "[ST]gpio_trig		: %u\n", vbusd->gpio_trig);

	/* current result */
	id = gpio_get_value(vbusd->gpio_idpin);
	vbus = mtk_vbusd_vbus_status();
	seq_printf(s, "[FINAL][%s] vBus is [%ssent]\n",
		id ? "Device" : "Host",
		vbus ? "pre" : "ab");

	return 0;
}
#endif

static int mtk_vbusd_open(struct inode *inode, struct file *file)
{
	return single_open(file, mtk_vbusd_show, inode->i_private);
}

static const struct file_operations mtk_vbusd_proc_fops = {
	.open    = mtk_vbusd_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release,
};

int mtk_vbusd_init_debugfs(struct usb_vbusd *vbusd)
{
	int	ret;
	struct dentry *file;
	struct dentry *root;

	root = debugfs_create_dir(dev_driver_string(vbusd->dev), NULL);
	if (!root) {
		ret = -ENOMEM;
		goto err0;
	}

	file = debugfs_create_file("vbusd", S_IRUGO, root,
					vbusd, &mtk_vbusd_proc_fops);
	if (!file) {
		ret = -ENOMEM;
		goto err1;
	}

	vbusd->debugfs_root = root;

	return 0;

err1:
	debugfs_remove_recursive(root);

err0:
	return ret;
}

void mtk_vbusd_exit_debugfs(struct usb_vbusd *vbusd)
{
	debugfs_remove_recursive(vbusd->debugfs_root);
}
#else
static inline int mtk_vbusd_init_debugfs(struct usb_vbusd *vbusd)
{
	return 0;
}
static inline void mtk_vbusd_exit_debugfs(struct usb_vbusd *vbusd)
{
	;
}
#endif

int mtk_vbusd_init(struct platform_device *pdev)
{
	int ret = 0;
	unsigned int val;
	unsigned int edges[2];
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct pinctrl *vbusd_pinctrl;
	struct pinctrl_state *pinctrl_vbus_detect;
	struct usb_vbusd *vbusd = NULL;
	unsigned long flags = 0;

	vbusd = kzalloc(sizeof(*vbusd), GFP_KERNEL);
	if (!vbusd) {
		ret = -ENOMEM;
		goto exit;
	}

	vbusd->dev		  = dev;
	vbusd->gpio_rise_debounce = 500;
	vbusd->gpio_fall_debounce = 1000;
	vbusd->gpio_polarity	  = 1;
	vbusd->gpio_debounce	  = vbusd_debounce_time;

	vbusd->gpio_irqf[0] = IRQF_TRIGGER_LOW;
	vbusd->gpio_irqf[1] = IRQF_TRIGGER_HIGH;

	spin_lock_init(&vbusd->gpio_lock);
	mutex_init(&vbusd->gpio_mutex);
	INIT_DELAYED_WORK(&vbusd->gpio_workqueue, mtk_vbusd_workqueue);
	mt6771_usb_vbusd = vbusd;

	ret = of_property_read_u32(node, "vbusd,gpio_pin", &val);
	if (!ret) {
		vbusd->gpio_dts = val;
		dev_info(dev, "[vbusd] gpio pin: %u!\n", val);
	}

	ret = of_property_read_u32_array(node, "vbusd,gpio_edges", edges, 2);
	if (!ret) {
		vbusd->gpio_rise_debounce = edges[0];
		vbusd->gpio_fall_debounce = edges[1];
		dev_info(dev,
			"[vbusd] rise_debounce: %u, fall_debounce: %u!\n",
			edges[0], edges[1]);
	}

	ret = of_property_read_u32(node, "vbusd,gpio_polarity", &val);
	if (!ret) {
		vbusd->gpio_polarity = val ? 1 : 0;
		dev_info(dev, "[vbusd] gpio pin active: %u!\n", val);
	}

	vbusd->gpio_pin = of_get_named_gpio(node, "vbusd,gpio_vbusdetect", 0);
	if (!gpio_is_valid(vbusd->gpio_pin)) {
		ret = vbusd->gpio_pin;
		dev_dbg(dev,
			"get dtsi vbusd,gpio_vbusdetect failed since %i\n",
			ret);
		goto error;
	}

	dev_info(dev, "[vbusd]gpio_pin %u!\n", vbusd->gpio_pin);

	ret = of_property_read_u32(node, "vbusd,gpio_idpin", &val);
	if (!ret) {
		vbusd->gpio_iddts = val;
		dev_info(dev, "[vbusd] gpio id pin: %u!\n", val);
	}

	vbusd->gpio_idpin = of_get_named_gpio(node, "vbusd,gpio_iddig", 0);
	if (!gpio_is_valid(vbusd->gpio_idpin)) {
		ret = vbusd->gpio_idpin;
		dev_dbg(dev,
			"get dtsi vbusd,gpio_iddig failed since %i\n",
			ret);
		goto error;
	}

	dev_info(dev, "[vbusd]gpio_iddig pin:%u:%u!\n",
		 vbusd->gpio_iddts, vbusd->gpio_idpin);

	vbusd_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(vbusd_pinctrl)) {
		ret = PTR_ERR(vbusd_pinctrl);
		dev_dbg(dev, "Cannot find usb pinctrl since %i!\n", ret);
		goto error;
	}

	pinctrl_vbus_detect =
		pinctrl_lookup_state(vbusd_pinctrl, "vbusd_init");
	if (IS_ERR(pinctrl_vbus_detect)) {
		ret = PTR_ERR(pinctrl_vbus_detect);
		dev_dbg(dev, "Can't find pinctrl vbusd_init since %i\n", ret);
		goto error;
	}

	ret = pinctrl_select_state(vbusd_pinctrl, pinctrl_vbus_detect);
	if (ret < 0) {
		dev_dbg(dev, "pinctrl_select_state failed since %i\n", ret);
		goto error;
	}

	ret = gpio_to_irq(vbusd->gpio_pin);
	if (ret < 0) {
		dev_dbg(dev, "gpio_to_irq failed since %i\n", ret);
		goto error;
	}
	vbusd->gpio_irq = ret;
	dev_info(dev, "[vbusd]gpio_irq %u\n", vbusd->gpio_irq);

	vbusd->gpio_trig = vbusd->gpio_polarity ? 1 : 0;
	flags = vbusd->gpio_irqf[vbusd->gpio_trig];
	ret = request_irq(vbusd->gpio_irq, mtk_vbusd_intr,
				  flags, "USB_VBUSD", NULL);
	if (ret)
		dev_dbg(dev,
				"USB_VBUSD request_irq failed since %i!\n",
				ret);

	dev_info(dev, "[vbusd]/sys/kernel/debug/%s/vbusd\n",
			 dev_driver_string(vbusd->dev));
	mtk_vbusd_init_debugfs(vbusd);

	goto exit;

error:
	if (vbusd) {
		mt6771_usb_vbusd = NULL;
		kfree(vbusd);
	}

exit:
	return ret;
}

void mtk_vbusd_denit(void)
{
	struct usb_vbusd *vbusd = NULL;

	vbusd = mtk_vbusd_get_handle();
	if (!IS_ERR(vbusd)) {
		mtk_vbusd_exit_debugfs(vbusd);
		cancel_delayed_work(&vbusd->gpio_workqueue);
		destroy_delayed_work_on_stack(&vbusd->gpio_workqueue);
		free_irq(vbusd->gpio_irq, NULL);
		mt6771_usb_vbusd = NULL;
		kfree(vbusd);
	}
}
