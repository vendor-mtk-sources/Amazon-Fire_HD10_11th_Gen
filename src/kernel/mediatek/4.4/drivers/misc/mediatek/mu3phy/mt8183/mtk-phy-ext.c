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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include "musb_core.h"
#include "mtk-phy-ext.h"
#include "mtk-phy.h"
#ifdef CONFIG_PROJECT_PHY
#include "mtk-phy-asic.h"
#endif

#define USBPHYACR6                     0x018
#define U2PHYACR4                      0x020
#define U2PHYDTM0                      0x68
#define U2PHYDMON1                     0x74
#define B_USB20_LINE_STATE             22
#define USB20_LINE_STATE_MASK          (3ul << 22)

#define MOD "[EXTCON]"
#if 0
#define LS_PRINT(fmt, ...)             pr_err(fmt, ##__VA_ARGS__)
#else
#define LS_PRINT(fmt, ...)
#endif

#define USBPHY_READ32(offset)          \
	U3PhyReadReg32((uintptr_t)((SSUSB_SIFSLV_U2PHY_COM_BASE) + offset))

#define USBPHY_WRITE32(offset, value)  \
	U3PhyWriteReg32((uintptr_t)((SSUSB_SIFSLV_U2PHY_COM_BASE) + offset), \
			value)

int mt_usb_phychk_extconn(void)
{
	u32 val = 0;
	u32 line_state = 0;
	int usb_extconn = MT_USB_EXTCONN_UNKNOWN;

	static const char *const string_usb_extconn_type[] = {
	    "UNKNOWN LINE TYPE",
	    "STANDARD_HOST",
	    "CHARGING_HOST",
	    "NONSTANDARD_CHARGER",
	    "STANDARD_CHARGER",
	    "INVALID PARAMETER"
	};

	/* enable USB MAC clock. */
	usb_enable_clock(true);

	/* set PHY 0x18[23] = 1'b0 */
	val = USBPHY_READ32(USBPHYACR6);
	val &= ~(RG_USB20_BC11_SW_EN);
	USBPHY_WRITE32(USBPHYACR6, val);
	LS_PRINT("usbphy addr 0x%p = 0x%x but 0x%x\n",
		 SSUSB_SIFSLV_U2PHY_COM_BASE + USBPHYACR6,
		 val, USBPHY_READ32(USBPHYACR6));

	/* Device side does NOT apply 15K pull-down on DP,
	 * and apply 100K pull upon DM.
	 * 1. For USB Host, 15K pull-down will cause linestate as 2'b00.
	 * 2. For Charger since no pull-down exist on D+/-, the linesate
	 *    will be 2'b1x.
	 *
	 * stage 1
	 * set PHY 0x68[21:20] = 2'b11
	 * set PHY 0x68[ 7: 6] = 2'b00
	 * set PHY 0x20[   17] = 1'b1
	 */

	val = USBPHY_READ32(U2PHYDTM0);
	val |= (FORCE_DP_PULLDOWN | FORCE_DM_PULLDOWN);
	val &= ~(RG_DPPULLDOWN | RG_DMPULLDOWN);
	USBPHY_WRITE32(U2PHYDTM0, val);
	LS_PRINT("usbphy addr 0x%p = 0x%x but 0x%x\n",
		 SSUSB_SIFSLV_U2PHY_COM_BASE + U2PHYDTM0,
		 val, USBPHY_READ32(U2PHYDTM0));

	val = USBPHY_READ32(U2PHYACR4);
	val |= RG_USB20_DM_100K_EN;
	USBPHY_WRITE32(U2PHYACR4, val);
	LS_PRINT("usbphy addr 0x%p = 0x%x but 0x%x\n",
		 SSUSB_SIFSLV_U2PHY_COM_BASE + U2PHYACR4,
		 val, USBPHY_READ32(U2PHYACR4));

	mdelay(10);

	/*
	 * Read linestate
	 * Read PHY 0x74[23:22] 2'bxx
	 */
	line_state = USBPHY_READ32(U2PHYDMON1);
	LS_PRINT("usbphy addr 0x%p = 0x%x\n",
		 SSUSB_SIFSLV_U2PHY_COM_BASE + U2PHYDMON1,
		 USBPHY_READ32(U2PHYDMON1));
	line_state &= USB20_LINE_STATE_MASK;
	line_state >>= B_USB20_LINE_STATE;

	if ((line_state & 0x02) == 0)
		usb_extconn = MT_USB_EXTCONN_STANDARDHOST;
	else if ((line_state & 0x02) != 0) {
	/*
	 * Device side does apply 15K pul-down on DP, and apply 100K pull up
	 * on DM.
	 * 1. For standard charger, D+/- are shorted, so the D+ pulling down
	 *     will drive both D+/- to low. therefore  linestate as 2'b00.
	 * 2. For non-standard charger, since no pull-down exist on D-,
	 *     the linesate will be 2'b1x.
	 *
	 * stage 2
	 * set PHY 0x68[21:20] = 2'b11
	 * set PHY 0x68[ 7: 6] = 2'b01
	 * set PHY 0x20[   17] = 1'b1
	 */

		val = USBPHY_READ32(U2PHYDTM0);
		val |= (FORCE_DP_PULLDOWN | FORCE_DM_PULLDOWN);
		val &= ~(RG_DPPULLDOWN | RG_DMPULLDOWN);
		val |= RG_DPPULLDOWN;
		USBPHY_WRITE32(U2PHYDTM0, val);
		LS_PRINT("usbphy addr 0x%p = 0x%x but 0x%x\n",
			 SSUSB_SIFSLV_U2PHY_COM_BASE + U2PHYDTM0,
			 val, USBPHY_READ32(U2PHYDTM0));

		val = USBPHY_READ32(U2PHYACR4);
		val |= RG_USB20_DM_100K_EN;
		USBPHY_WRITE32(U2PHYACR4, val);
		LS_PRINT("usbphy addr 0x%p = 0x%x but 0x%x\n",
			 SSUSB_SIFSLV_U2PHY_COM_BASE + U2PHYACR4,
			 val, USBPHY_READ32(U2PHYACR4));

		mdelay(10);

		/* Read linestate
		 * Read PHY 0x74[23:22] 2'bxx
		 */
		line_state = USBPHY_READ32(U2PHYDMON1);
		LS_PRINT("usbphy addr 0x%p = 0x%x\n",
			 SSUSB_SIFSLV_U2PHY_COM_BASE + U2PHYDMON1,
			 USBPHY_READ32(U2PHYDMON1));
		line_state &= USB20_LINE_STATE_MASK;
		line_state >>= B_USB20_LINE_STATE;

		switch (line_state) {
		case 0x00:
			usb_extconn = MT_USB_EXTCONN_STANDARDCHARGER;
			break;
		case 0x02:
		case 0x03:
			usb_extconn = MT_USB_EXTCONN_NONSTANDARDCHARGER;
			break;
		default:
			usb_extconn = MT_USB_EXTCONN_UNKNOWN;
			break;
		}
	}

	val = USBPHY_READ32(U2PHYDTM0);
	val &= ~(FORCE_DP_PULLDOWN | FORCE_DM_PULLDOWN);
	USBPHY_WRITE32(U2PHYDTM0, val);
	LS_PRINT("usbphy addr 0x%p = 0x%x but 0x%x\n",
		 SSUSB_SIFSLV_U2PHY_COM_BASE + U2PHYDTM0,
		 val, USBPHY_READ32(U2PHYDTM0));

	val = USBPHY_READ32(U2PHYACR4);
	val &= ~RG_USB20_DM_100K_EN;
	USBPHY_WRITE32(U2PHYACR4, val);
	LS_PRINT("usbphy addr 0x%p = 0x%x but 0x%x\n",
		 SSUSB_SIFSLV_U2PHY_COM_BASE + U2PHYACR4,
		 val, USBPHY_READ32(U2PHYACR4));

	usb_extconn = (usb_extconn > MT_USB_EXTCONN_STANDARDCHARGER)
	    ? MT_USB_EXTCONN_MAXIMUM : usb_extconn;

	usb_enable_clock(false);

	pr_err("\n%s Final USB line type: %s since line state: 0x%x\n",
	       MOD, string_usb_extconn_type[usb_extconn], usb_extconn);

	return usb_extconn;
}
