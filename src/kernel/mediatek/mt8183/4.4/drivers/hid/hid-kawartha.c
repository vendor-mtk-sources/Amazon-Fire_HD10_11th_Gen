/*
 * HID driver for Kawartha Bluetooth Keyboard
 * adapted from hid_aspen.c
 *
 * Copyright (C) 2020 - 2021 Amazon.com, Inc. and its affiliates. All Rights Reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/device.h>
#include <linux/hid.h>
#include <linux/input/mt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/version.h>
#include "hid-ids.h"

/* Debug message */
#undef KAWARTHA_DEBUG
#ifdef KAWARTHA_DEBUG
	#define DEBUG_MSG(fmt, args...)	printk(fmt, ## args)
#else
	#define DEBUG_MSG(fmt, args...)
#endif

#undef VERY_VERBOSE_LOGGING
#ifdef VERY_VERBOSE_LOGGING
	#define KAWARTHA_DEBUG_VERBOSE DEBUG_MSG
#else
	#define KAWARTHA_DEBUG_VERBOSE(fmt, args...)
#endif

/* Report id */
#define REPORT_ID_KEYBOARD	0x01
#define REPORT_ID_MOUSE		0x06
#define REPORT_ID_CONSUMER	0x0c
/* Handle Function key */
#define REPORT_ID_KAWARTHA	0x04

#define S_KEY_RAW		0x16
#define FN_KEY_PRESSED		0x07
#define FN_KEY_RELEASED		0x02

#define FN_STATUS_UP		0
#define FN_STATUS_DOWN		1

static int fn_status = FN_STATUS_UP;

/*
 * Device structure for matched devices
 * @quirks: Currently unused.
 * @input: Input device through which we report events.
 * @init_done: To indicate evdev has been created successfully
 */
struct kawartha_device {
	unsigned long quirks;
	struct input_dev *input;
	unsigned char init_done;
};

/*
 *  Deal with input raw event
 *  REPORT_ID:		DESCRIPTION:
 *	0x01			Keyboard
 *	0x04			Fn Key
 *	0x07			Mouse
 *	0x08			Touchpad
 *
 */
static int kawartha_raw_event(struct hid_device *hdev, struct hid_report *report,
	 u8 *data, int size)
{
	struct kawartha_device *kawartha_dev = hid_get_drvdata(hdev);

	char key_code;
	int i = 0;

	KAWARTHA_DEBUG_VERBOSE("%s: report id: %d, size: %d report data:\n", __func__, report->id, size);

	for (i = 0; i < size; i++) {
		KAWARTHA_DEBUG_VERBOSE("0x%02x ", data[i]);
	}
	KAWARTHA_DEBUG_VERBOSE("\n");

	if (!(hdev->claimed & HID_CLAIMED_INPUT))
		return 0;

	KAWARTHA_DEBUG_VERBOSE("%s: hdev->claimed = 0x%x, ready to send input event\n", __func__, hdev->claimed);

	switch (report->id) {
	case REPORT_ID_KEYBOARD:	/* Keyboard event: 0x01 */
		DEBUG_MSG("%s: Enter:: received REPORT_ID_KEYBOARD event\n", __func__);
		if (size <= 3)
			break;
		key_code = data[3];
		switch (key_code) {
		case S_KEY_RAW:
			if (fn_status) {
				DEBUG_MSG("%s: Handle Fn + S key combination\n", __func__);
				/*
				 * Wrap reporting of S key down with reports for Fn key down and Fn
				 * key up when Fn status is FN_STATUS_DOWN. Report for S key up will
				 * be sent separately upon key release.
				 */
				input_report_key(kawartha_dev->input, KEY_FN, 1);
				input_sync(kawartha_dev->input);

				input_report_key(kawartha_dev->input, KEY_S, 1);
				input_sync(kawartha_dev->input);

				input_report_key(kawartha_dev->input, KEY_FN, 0);
				input_sync(kawartha_dev->input);
			} else {
				input_report_key(kawartha_dev->input, KEY_S, 1);
				input_sync(kawartha_dev->input);
			}
			DEBUG_MSG("%s: returning 1 - handled report id\n", __func__);
			return 1;
		}
		break;

	case REPORT_ID_CONSUMER:		/* Consumer event: 0x0c */
		DEBUG_MSG("%s: Enter:: received REPORT_ID_CONSUMER event\n", __func__);
		break;

	case REPORT_ID_MOUSE:		/* Mouse event: 0x06 */
		DEBUG_MSG("%s: Enter:: received REPORT_ID_MOUSE event\n", __func__);
		break;

	case REPORT_ID_KAWARTHA:		/* Fn key event: 0x04 */
		if (size <= 1)
			break;
		key_code = data[1];
		DEBUG_MSG("%s: Enter:: received REPORT_ID_KAWARTHA event keycode = %02x\n",
			__func__, key_code);
		switch (key_code) {
		case FN_KEY_PRESSED:
			DEBUG_MSG("%s: Fn key status: Key pressed\n", __func__);
			fn_status = FN_STATUS_DOWN;
			break;
		case FN_KEY_RELEASED:
			DEBUG_MSG("%s: Fn key status: Key released\n", __func__);
			fn_status = FN_STATUS_UP;
			break;
		}
		break;
	default:	/* Unknown report id */
		DEBUG_MSG("%s: unhandled report id %d\n", __func__, report->id);
		break;
	}

	KAWARTHA_DEBUG_VERBOSE("%s: returning 0 to pass event to linux input subsystem\n", __func__);
	return 0;	/* Pass event to linux input subsystem */

}

/*
 * Probe function for matched devices
 */
static int kawartha_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	int ret;
	struct kawartha_device *kawartha;

	DEBUG_MSG("%s\n", __func__);

	kawartha = kmalloc(sizeof(*kawartha), GFP_KERNEL | __GFP_ZERO);
	if (kawartha == NULL) {
		hid_err(hdev, "%s: can't alloc kawartha descriptor\n", __func__);
		return -ENOMEM;
	}

	kawartha->quirks = id->driver_data;
	hid_set_drvdata(hdev, kawartha);

	ret = hid_parse(hdev);
	if (ret) {
		hid_err(hdev, "%s: parse failed\n", __func__);
		goto fail;
	}

	ret = hid_hw_start(hdev, HID_CONNECT_DEFAULT);
	if (ret) {
		hid_err(hdev, "%s: hw start failed\n", __func__);
		goto fail;
	}

	fn_status = FN_STATUS_UP;

	if (!kawartha->input) {
		hid_err(hdev, "kawartha input not registered\n");
		ret = -ENOMEM;
		goto err_stop_hw;
	}

	kawartha->init_done = 1;
	pr_info("%s: Init ok, hdev->claimed = 0x%x\n", __func__, hdev->claimed);

	return 0;

err_stop_hw:
	hid_hw_stop(hdev);

fail:
	kfree(kawartha);
	return ret;
}

/*
 * Input settings and key mapping for matched devices
 *  USAGE_PAGE_ID:      DESCRIPTION:
 *		0x0001			Generic Desktop
 *		0x0006			Generic Device Control
 *		0x0007			Keyboard/Keypad
 *		0x0008			LED
 *		0x0009			BUTTON
 *		0x000c			Consumer
 *		0x000d			Digitizer
 */
static int kawartha_input_mapping(struct hid_device *hdev,
		struct hid_input *hi, struct hid_field *field,
		struct hid_usage *usage, unsigned long **bit, int *max)
{
	struct input_dev *input = hi->input;
	struct kawartha_device *kawartha = hid_get_drvdata(hdev);

	if (!kawartha->input)
		kawartha->input = hi->input;

	DEBUG_MSG("%s: Usage page = 0x%x, Usage id = 0x%x\n", __func__,
				(usage->hid & HID_USAGE_PAGE) >> 4, usage->hid & HID_USAGE);

	switch (usage->hid & HID_USAGE_PAGE) {
	case HID_UP_GENDESK:	/* 0x0001 */
		/* Define accepted event */
		set_bit(EV_REL, input->evbit);
		set_bit(EV_ABS, input->evbit);
		set_bit(EV_KEY, input->evbit);
		set_bit(EV_SYN, input->evbit);

		/* Handle Function key */
		set_bit(KEY_FN, input->keybit);

		break;

	case HID_UP_KEYBOARD:	/* 0x0007 */
		break;

	case HID_UP_LED:		/* 0x0008 */
		break;

	case HID_UP_BUTTON:		/* 0x0009 */
		break;

	case HID_UP_CONSUMER:	/* 0x000c */
		break;

	case HID_UP_DIGITIZER:	/* 0x000d */
		break;

	default:
		break;
	}

	/*
	 * Return a 1 means a matching mapping is found, otherwise need
	 * HID driver to search mapping in hid-input.c
	*/
	return 0;
}

/*
 * Remove function
 */
static void kawartha_remove(struct hid_device *hdev)
{
	struct kawartha_device *kawartha = hid_get_drvdata(hdev);

	fn_status = FN_STATUS_UP;

	DEBUG_MSG("%s\n", __func__);
	hid_hw_stop(hdev);

	if (NULL != kawartha)
		kfree(kawartha);
}

/*
 * Device list that matches this driver
 */
static const struct hid_device_id kawartha_devices[] = {
	{ HID_BLUETOOTH_DEVICE(BT_VENDOR_ID_LAB126, USB_DEVICE_ID_LAB126_KAWARTHA_KB) },
	{ }	/* Terminating entry */
};
MODULE_DEVICE_TABLE(hid, kawartha_devices);

/*
 * Special driver function structure for matched devices
 */
static struct hid_driver kawartha_driver = {
	.name		= "Kawartha Keyboard",
	.id_table	= kawartha_devices,
	.raw_event	= kawartha_raw_event,
	.probe		= kawartha_probe,
	.remove		= kawartha_remove,
	.input_mapping	= kawartha_input_mapping,
};

/*
 * Init function
 */
static int __init kawartha_init(void)
{
	int ret = hid_register_driver(&kawartha_driver);

	DEBUG_MSG("%s: hid_register_driver returned %d\n", __func__, ret);
	return ret;
}

/*
 * Exit function
 */
static void __exit kawartha_exit(void)
{
	DEBUG_MSG("%s\n", __func__);
	hid_unregister_driver(&kawartha_driver);
}

module_init(kawartha_init);
module_exit(kawartha_exit);

MODULE_AUTHOR("arnkup@amazon.com");
MODULE_LICENSE("GPL");
