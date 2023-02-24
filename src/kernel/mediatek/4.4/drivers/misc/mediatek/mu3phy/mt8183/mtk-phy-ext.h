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

#ifndef __MTK_PHY_EXT_H
#define __MTK_PHY_EXT_H

#include <linux/err.h>
#include <linux/of.h>
#include <linux/device.h>

#ifdef CONFIG_USB_PHYCHK_EXTCONN
enum mtk_usb_extconn_type {
	MT_USB_EXTCONN_UNKNOWN = 0,
	MT_USB_EXTCONN_STANDARDHOST = 1,
	MT_USB_EXTCONN_CHARGINGHOST = 2,
	MT_USB_EXTCONN_NONSTANDARDCHARGER = 3,
	MT_USB_EXTCONN_STANDARDCHARGER = 4,
	MT_USB_EXTCONN_MAXIMUM = 5,
};
extern int mt_usb_phychk_extconn(void);
#else
static inline int mt_usb_phychk_extconn(void)
{
	return 0;
}
#endif

#endif /* __MTK_PHY_EXT_H */
