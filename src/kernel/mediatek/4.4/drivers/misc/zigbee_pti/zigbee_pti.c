/*******************************************************************************
* Copyright 2018 Amazon Technologies, Inc. All Rights Reserved.*
*
*@file	drivers/misc/zigbee_pti.c
*
* Author: ken dai <kendai@amazon.com>
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* version 2 as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, you may obtain a copy of the GNU
* General Public License Version 2 or later at the following locations:
* http://www.opensource.org/licenses/gpl-license.html
* http://www.gnu.org/copyleft/gpl.html
*******************************************************************************/
#include <linux/version.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/bitops.h>
#include <linux/uaccess.h>
#include <linux/circ_buf.h>
#include <asm/irq.h>
#include <linux/tty.h>
#include <linux/delay.h>
#include <linux/of_platform.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/unistd.h>

#include "zigbee_pti.h"
#define zigbee_dbg_verbose(fmt, ...)

static int pti_read_dts(struct platform_device *pdev);

static struct zigbee_pti_entry_struct *priv_g;

#ifdef PTI_HAS_SHARED_UART_PINS
/* Some products share the PTI UART */
static void change_pin_mode(struct zigbee_pti_entry_struct *priv, enum pti_uart_pin_cfg cfg)
{
	if (cfg == PTI_UART_PIN_CFG_TO_PTI){
		pinctrl_select_state(priv->pinctrl, priv->toPTI);
	} else if (cfg == PTI_UART_PIN_CFG_TO_OUTSIDE_FUNCTION){
		pinctrl_select_state(priv->pinctrl, priv->toOutsideFunction);
	} else {
		pr_err("Not a recognized pin mode!\n");
		return;
	}
}
#endif

static void pti_buf_init(void)
{
	priv_g->pti_data_buf.count = -1;
	priv_g->pti_data_buf.w_idx = -1;
	priv_g->pti_data_buf.r_idx = 0;
	priv_g->pti_data_buf.w_pos = 1;
}

static void pti_tty_cleanup(void)
{
	int err;

	err = tty_unregister_ldisc(N_ZIGBEE_PTI);
	if (err)
		pr_err("can't unregister N_ZIGBEE_PTI line discipline\n");
	else
		pr_debug("N_ZIGBEE_PTI line discipline removed\n");
	pr_debug("N_ZIGBEE_PTI:Exiting.\n");
}

static void pti_clean(struct zigbee_pti_entry_struct *priv,
		bool b_tty)
{
	gpio_free((unsigned)priv->platform_data.pti_sync_gpio);

	if (b_tty)
		pti_tty_cleanup();
}

/* pti_tty_open()
 *
 *	Called when line discipline changed to N_ZIGBEE_PTI.
 *
 * Arguments:
 *	tty	pointer to tty info structure
 *
 * Return Value:
 *	0 if success, otherwise error code
*/
static int pti_tty_open(struct tty_struct *tty)
{
	struct uart_state *state;

	if (!priv_g) {
		pr_err("pti_tty_open(): driver data already freed?");
		return -ENOMEM;
	}

#ifdef PTI_HAS_SHARED_UART_PINS
	change_pin_mode(priv_g, PTI_UART_PIN_CFG_TO_PTI);
	pr_debug("pti_tty_open(): PTI took control of UART");
#endif

	state = tty->driver_data;
	if (priv_g->pti_ldisc_saved.open)
		priv_g->pti_ldisc_saved.open(tty);

	pti_buf_init();
	pr_debug("enable irq %d.\n", priv_g->platform_data.pti_sync_irq);
	enable_irq(priv_g->platform_data.pti_sync_irq);

	return 0;
}

/* pti_tty_close()
 *
 *	Called when the line discipline is changed to something
 *	else, the tty is closed, or the tty detects a hangup.
 */
static void pti_tty_close(struct tty_struct *tty)
{
	struct uart_state *state;

	if (!priv_g) {
		pr_err("pti_tty_close(): driver data already freed?");
		return;
	}

	pr_debug("disable irq.\n");
	disable_irq(priv_g->platform_data.pti_sync_irq);
	state = tty->driver_data;

	if (priv_g->pti_ldisc_saved.close)
		priv_g->pti_ldisc_saved.close(tty);

#ifdef PTI_HAS_SHARED_UART_PINS
	change_pin_mode(priv_g, PTI_UART_PIN_CFG_TO_OUTSIDE_FUNCTION);

	pr_debug("pti_tty_close(): UART returned to MC logging");
#endif
}

/* pti_tty_read()
 *	Perform reads for the line discipline.
 *	Always called in user context, may sleep.
 *
 * Arguments:
 *	tty	tty device
 *	file	file object
 *	buf	userspace buffer pointer
 *	nr	size of I/O
 *
 * Return Value:
 *	number of byte readen
 */
static ssize_t pti_tty_read(struct tty_struct *tty, struct file *file,
		unsigned char __user *buf, size_t nr)
{
	unsigned long intr_flags;
	unsigned long left;
	ssize_t  retval = 0;

	if (priv_g->pti_data_buf.count <= 0) {
		/* no new packet received, return 0 */
		return 0;
	}

	if (nr < priv_g->pti_data_buf.data_buf[priv_g->pti_data_buf.r_idx][0]) {
		/*
		* the incoming buf is too small, return nothign.
		* we return either a full PTI data frame, or nothing at all.
		*/
		return 0;
	}
	zigbee_dbg_verbose("read nr=%d, pck count %d\n",
		(int)nr, priv_g->pti_data_buf.count);

	spin_lock_irqsave(&priv_g->pti_data_buf.lock, intr_flags);
	left = copy_to_user(buf,
		&priv_g->pti_data_buf.data_buf[priv_g->pti_data_buf.r_idx][1],
		priv_g->pti_data_buf.data_buf[priv_g->pti_data_buf.r_idx][0]);

	if (left != 0) {
		pr_err("read failed copy to user %d\n", (int) left);
		retval = 0;
	} else {
		retval = priv_g->pti_data_buf.data_buf
				[priv_g->pti_data_buf.r_idx][0];
	}

	priv_g->pti_data_buf.r_idx = (priv_g->pti_data_buf.r_idx+1)
		& (MAX_PACKET_BUF - 1);
	priv_g->pti_data_buf.count--;
	spin_unlock_irqrestore(&priv_g->pti_data_buf.lock, intr_flags);

	zigbee_dbg_verbose("read nr=%d.\n",
		priv_g->pti_data_buf.data_buf[priv_g->pti_data_buf.r_idx][0]);
	return retval;
}

/* pti_tty_receive()
 *
 *     Called by tty low level driver when receive data is
 *     available.
 *
 * Arguments:
 *	tty          pointer to tty isntance data
 *	data         pointer to received data
 *	flags        pointer to flags for data
 *	count        count of received data in bytes
 *
 * Return Value:
 *	Number of byte accepted.
 */
static int pti_tty_receive(struct tty_struct *tty,
	const unsigned char *data, char *flags, int count)
{
	unsigned long intr_flags;

	if (priv_g->pti_data_buf.w_idx == -1) {
	/*
	* Data may arrive earlier than the PTI_SYNC rising-edge interrupt,
	* this happens when pti_tty_open at the middle of a pti data frame.
	* Ignore the data frame until next PTI_SYNC rising edge
	*/
		return count;
	}

	zigbee_dbg_verbose("rx: count %d, w_idx %d, w_pos %d\n",
		count,
		priv_g->pti_data_buf.w_idx,
		priv_g->pti_data_buf.w_pos);

	spin_lock_irqsave(&priv_g->pti_data_buf.lock, intr_flags);

	if (priv_g->pti_data_buf.w_pos + count >= MAX_PACKET_LEN) {

	/*
	* Get a packet longer than MAX_PACKET_LEN from low layer,
	* This should not happen, it may indicate pti_sync isr
	* is not serviced on time
	*/

		pr_warn("rx: failed to copy data %d\n", count);
		spin_unlock_irqrestore(&priv_g->pti_data_buf.lock, intr_flags);
		return count;
	}

	memcpy(&priv_g->pti_data_buf.data_buf[priv_g->pti_data_buf.w_idx]
				[priv_g->pti_data_buf.w_pos],
			data,
			count);

	priv_g->pti_data_buf.w_pos += count;

	/* Update the length for receiving packet */
	priv_g->pti_data_buf.data_buf[priv_g->pti_data_buf.w_idx][0] =
		priv_g->pti_data_buf.w_pos - 1;

	spin_unlock_irqrestore(&priv_g->pti_data_buf.lock, intr_flags);
	zigbee_dbg_verbose("rx: exit w_pos %d\n", priv_g->pti_data_buf.w_pos);
	tty_unthrottle(tty);

	return count;
}


/* pti_tty_init()
 *
 *     Register N_ZIGBEE_PTI with TTY core
 *
 */
static int pti_tty_init(void)
{
	int err;

	/* Inherit the N_TTY's ops */
	n_tty_inherit_ops(&priv_g->pti_ldisc_ops);

	priv_g->pti_ldisc_ops.owner = THIS_MODULE;
	priv_g->pti_ldisc_ops.name = "pti_tty";
	priv_g->pti_ldisc_saved.open = priv_g->pti_ldisc_ops.open;
	priv_g->pti_ldisc_saved.close = priv_g->pti_ldisc_ops.close;
	priv_g->pti_ldisc_ops.open = pti_tty_open;
	priv_g->pti_ldisc_ops.close = pti_tty_close;
	priv_g->pti_ldisc_ops.read = pti_tty_read;
	priv_g->pti_ldisc_ops.receive_buf2	= pti_tty_receive;

	err = tty_register_ldisc(N_ZIGBEE_PTI, &priv_g->pti_ldisc_ops);
	if (err)
		pr_err("can't register N_ZIGBEE_PTI line discipline\n");
	else
		pr_debug("N_ZIGBEE_PTI line discipline registered\n");

	spin_lock_init(&priv_g->pti_data_buf.lock);

	pti_buf_init();
	return err;
}

/* pti_sync_isr()
 *
 *     Interrupt service routine for PTI_SYNC, switch to a new buffer
 *
 * Arguments:
 *	irq          irq number
 *	dev_id         device id
 *
 * Return Value:
 *	None.
 */
static irqreturn_t pti_sync_isr(int irq, void *dev_id)
{
	unsigned long flags;

	zigbee_dbg_verbose("%s: pck count %d, idx %d\n", __func__,
		priv_g->pti_data_buf.count, priv_g->pti_data_buf.w_idx);
	spin_lock_irqsave(&priv_g->pti_data_buf.lock, flags);

	/*Switch to a new frame buffer*/
	priv_g->pti_data_buf.w_idx = (priv_g->pti_data_buf.w_idx + 1)
			& (MAX_PACKET_BUF - 1);
	priv_g->pti_data_buf.data_buf[priv_g->pti_data_buf.w_idx][0] = 0;
	priv_g->pti_data_buf.w_pos = 1;

	/*Increase received packe counter*/
	priv_g->pti_data_buf.count = (priv_g->pti_data_buf.count + 1)
			& (MAX_PACKET_BUF - 1);

	spin_unlock_irqrestore(&priv_g->pti_data_buf.lock, flags);

	return IRQ_HANDLED;
}

static int pti_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct zigbee_pti_platform_data *pdata = NULL;

	/*
	* TTY ldisc driver register to TTY core, with unique assigned number
	* Each ldisc driver can register only once. the probe function should
	* only be called once.
	* we check priv_g here to ensure it's only execute one time
	*/
	if (priv_g != NULL)
		return -EPERM;

	priv_g = devm_kzalloc(&pdev->dev,
		sizeof(struct zigbee_pti_entry_struct),
		GFP_KERNEL);
	if (!priv_g)
		return -ENOMEM;

	if (pdev->dev.platform_data) {
		pdata = pdev->dev.platform_data;
		priv_g->platform_data.pti_sync_gpio = pdata->pti_sync_gpio;
	} else if (pdev->dev.of_node) {
		pr_debug("read dts.\n");

		if (pti_read_dts(pdev) != 0) {
			pr_err("%s: failed to read device tree info\n",
				__func__);
			goto clean_data;
		}

#ifdef PTI_HAS_SHARED_UART_PINS
		/* Some products share the PTI UART, so the pinumx
		 * must be switched at runtime.
		 */
		priv_g->pinctrl = devm_pinctrl_get(&pdev->dev);
		rc = IS_ERR(priv_g->pinctrl);
		if (rc) {
			dev_err(&pdev->dev, "Cannot find pinctrl!\n");
			goto clean_data;
		}

		priv_g->toPTI = pinctrl_lookup_state(priv_g->pinctrl, "toPTI");
		rc = IS_ERR(priv_g->toPTI);
		if (rc) {
			dev_err(&pdev->dev, "Cannot find pinctrl toPTI!\n");
			goto clean_data;
		}

		priv_g->toOutsideFunction = pinctrl_lookup_state(priv_g->pinctrl, "default");
		rc = IS_ERR(priv_g->toOutsideFunction);
		if (rc) {
			dev_err(&pdev->dev, "Cannot find pinctrl toOutsideFunction!\n");
			goto clean_data;
		}
#endif

	} else {
		pr_err("%s: **ERROR** NO platform data available\n", __func__);
		goto clean_data;
	}

	pr_debug("%s: pti_sync_gpio=%d\n",
		__func__, priv_g->platform_data.pti_sync_gpio);

	/* register the tty line discipline driver */
	rc = pti_tty_init();

	if (rc) {
		pr_err("%s: pti_tty_init failed\n", __func__);
		priv_g = 0;
		return rc;
	}

	return 0;

clean_data:
	priv_g = 0;
	return rc;
}

static int pti_remove(struct platform_device *pdev)
{
	if (priv_g == NULL)
		return -EPERM;

	pti_clean(priv_g, true);
	priv_g = 0;
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id pti_match[] = {
	{ .compatible = "amazon,zigbee_pti",},
	{ /* Sentinel */ },
};
MODULE_DEVICE_TABLE(of, pti_match);
#endif

static int pti_read_dts(struct platform_device *pdev)
{
	int ret = 0;
	u32 interrupts[2] = { 0, 0 };
	struct device_node *node = NULL;

	node = of_find_matching_node(node, pti_match);
	if (node) {
		of_property_read_u32_array(node, "interrupts", interrupts,
			ARRAY_SIZE(interrupts));

		pr_debug("interrupts(%d, %d)\n",
			interrupts[0], interrupts[1]);

		priv_g->platform_data.pti_sync_gpio = of_get_named_gpio(node,
			"frame-gpio", 0);

		if (priv_g->platform_data.pti_sync_gpio < 0) {
			pr_err("Couldn't find pti_sync info from device tree\n");
			return -EINVAL;
		}
		pr_debug("pti_sync_gpio(%d)\n",
			priv_g->platform_data.pti_sync_gpio);


		ret = gpio_request(priv_g->platform_data.pti_sync_gpio,
			"pti_sync_gpio");
		if (ret) {
			pr_err("pti_sync request fail, ret(%d)\n", ret);
			return ret;
		}

		gpio_direction_input(priv_g->platform_data.pti_sync_gpio);

		priv_g->platform_data.pti_sync_irq =
			irq_of_parse_and_map(node, 0);

		pr_debug("%s request irq %d\n",  __func__,
			priv_g->platform_data.pti_sync_irq);

		ret = request_irq(priv_g->platform_data.pti_sync_irq,
			pti_sync_isr, interrupts[1],
			"pti_sync_isr", NULL);
		if (ret != 0) {
			pr_err("amz_pti: frame_gpio_irq line not available %d\n",
			ret);
		} else {
			disable_irq(priv_g->platform_data.pti_sync_irq);
			pr_debug("amz_pti: frame_gpio_irq line=%d type=%d\n",
				priv_g->platform_data.pti_sync_irq,
				interrupts[1]);
		}

	} else {
		pr_debug("can't find compatible node\n");
	}

	return ret;
}


static struct platform_driver pti_platform_driver = {
	.probe = pti_probe,
	.remove = pti_remove,
	.driver = {
		.name = "zigbee-pti",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = pti_match,
#endif
	},
};

static int __init pti_init(void)
{
	int rc = platform_driver_register(&pti_platform_driver);

	if (rc)
		pr_err("pti_platform_driver_register failed err=%d\n", rc);
	else
		pr_debug("pti_init success\n");

	return rc;
}

static void __exit pti_exit(void)
{
	platform_driver_unregister(&pti_platform_driver);
}

module_init(pti_init);
module_exit(pti_exit);

MODULE_DESCRIPTION("Amazon Zigbee PTI driver");
MODULE_AUTHOR("kendai@amazon.com");
MODULE_LICENSE("GPL");
MODULE_ALIAS_LDISC(N_ZIGBEE_PTI);

