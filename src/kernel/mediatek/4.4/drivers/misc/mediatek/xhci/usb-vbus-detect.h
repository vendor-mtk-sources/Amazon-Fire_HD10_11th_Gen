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

#ifndef __USB_VBUS_DETECT_H
#define __USB_VBUS_DETECT_H
#include <linux/err.h>
#include <linux/of.h>
#include <linux/device.h>
#include <linux/platform_device.h>

#ifdef CONFIG_USB_VBUSDET_IN_GPIO

#ifdef CONFIG_USB_PHYCHK_EXTCONN
extern int mtk_vbusd_get_linetype(void);
#endif	/* CONFIG_USB_PHYCHK_EXTCONN */

extern int mtk_vbusd_enable(bool on);
extern int mtk_vbusd_vbus_status(void);
extern int mtk_vbusd_init(struct platform_device *pdev);
extern void mtk_vbusd_denit(void);

#else	/* CONFIG_USB_VBUSDET_IN_GPIO */

#ifdef CONFIG_USB_PHYCHK_EXTCONN
static inline void mtk_vbusd_get_linetype(void)
{
	return 0;
}
#endif	/* CONFIG_USB_PHYCHK_EXTCONN */
static inline int mtk_vbusd_enable(bool on)
{
	return 0;
}

static inline int mtk_vbusd_vbus_status(void)
{
	return 0;
}

static inline int mtk_vbusd_init(struct platform_device *pdev)
{
	return 0;
}

static inline int mtk_vbusd_denit(void)
{
	return 0;
}

#endif	/* CONFIG_USB_VBUSDET_IN_GPIO */
#endif	/* __USB_VBUS_DETECT_H */
