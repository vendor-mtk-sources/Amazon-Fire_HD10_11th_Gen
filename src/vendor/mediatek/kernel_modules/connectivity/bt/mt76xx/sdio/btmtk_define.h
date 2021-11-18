/*
 *  Copyright (c) 2016,2017 MediaTek Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#ifndef __BTMTK_DEFINE_H__
#define __BTMTK_DEFINE_H__

#include "btmtk_config.h"

/**
 * Type definition
 */
#ifndef TRUE
	#define TRUE 1
#endif
#ifndef FALSE
	#define FALSE 0
#endif

#ifndef UNUSED
	#define UNUSED(x) (void)(x)
#endif

#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))

/**
 * Log level definition
 */
#define BTMTK_LOG_LEVEL_ERROR		1
#define BTMTK_LOG_LEVEL_WARNING		2
#define BTMTK_LOG_LEVEL_INFO		3
#define BTMTK_LOG_LEVEL_DEBUG		4
#define BTMTK_LOG_LEVEL_DEFAULT		BTMTK_LOG_LEVEL_INFO	/* default setting */

#define BTSDIO_INFO_RAW(p, l, fmt, ...)					\
	do {								\
		int raw_count = 0;					\
		const unsigned char *ptr = p;				\
		pr_info("[btmtk_info] "fmt, ##__VA_ARGS__);		\
		for (raw_count = 0; raw_count <= l; ++raw_count)	\
			pr_info(" %02X", ptr[raw_count]);		\
		pr_info("\n");						\
	} while (0)

#define BTSDIO_DEBUG_RAW(p, l, fmt, ...)				\
	do {								\
		int raw_count = 0;					\
		const unsigned char *ptr = p;				\
		pr_debug("[btmtk_info] "fmt, ##__VA_ARGS__);		\
		for (raw_count = 0; raw_count <= l; ++raw_count)	\
			pr_debug(" %02X", ptr[raw_count]);		\
		pr_debug("\n");						\
	} while (0)

/**
 * HCI packet type
 */
#define MTK_HCI_COMMAND_PKT	 	0x01
#define MTK_HCI_ACLDATA_PKT		0x02
#define MTK_HCI_SCODATA_PKT		0x03
#define MTK_HCI_EVENT_PKT		0x04

#define MTK_HCI_WRITE_CR_PKT		0x07
#define MTK_HCI_READ_CR_PKT		0x08

#define MTK_HCI_READ_CR_PKT_LENGTH	0x05
#define MTK_HCI_WRITE_CR_PKT_LENGTH	0x09

/**
 * Log file path & name, the default path is /sdcard
 */
#define PRINT_DUMP_COUNT		20
#define SYSLOG_FNAME			"bt_sys_log"
#define FWDUMP_FNAME			"bt_fw_dump"

char *COREDUMP_FILE_NAME[] = {
	"/data/misc/bluedroid/"FWDUMP_FNAME, "/sdcard/"FWDUMP_FNAME, "/tmp/"FWDUMP_FNAME
};

/**
 * SYS control
 */
#define SSYSCTL	0x400000

/**
 * WLAN
 */
#define WLAN	0x410000

/**
 * MCUCTL
 */
#define CLOCK_CTL	0x0708
#define INT_LEVEL	0x0718
#define COM_REG0	0x0730
#define SEMAPHORE_00	0x07B0
#define SEMAPHORE_01	0x07B4
#define SEMAPHORE_02	0x07B8
#define SEMAPHORE_03	0x07BC

/**
 * Timeout setting, mescs
 */
#define USB_CTRL_IO_TIMO	100
#define USB_INTR_MSG_TIMO	2000


/**
 * USB request type definition
 */
#define DEVICE_VENDOR_REQUEST_OUT	0x40
#define DEVICE_VENDOR_REQUEST_IN	0xc0
#define DEVICE_CLASS_REQUEST_OUT	0x20
#define DEVICE_CLASS_REQUEST_IN		0xa0

#define BTUSB_MAX_ISOC_FRAMES	10
#define BTUSB_INTR_RUNNING	0
#define BTUSB_BULK_RUNNING	1
#define BTUSB_ISOC_RUNNING	2
#define BTUSB_SUSPENDING	3
#define BTUSB_DID_ISO_RESUME	4

/**
 * ROM patch related
 */
#define PATCH_HCI_HEADER_SIZE	4
#define PATCH_WMT_HEADER_SIZE	5
#define PATCH_HEADER_SIZE	(PATCH_HCI_HEADER_SIZE + PATCH_WMT_HEADER_SIZE)
#define UPLOAD_PATCH_UNIT	2048
#define PATCH_INFO_SIZE		30
#define PATCH_PHASE1		1
#define PATCH_PHASE2		2
#define PATCH_PHASE3		3

#define USB_IO_BUF_SIZE		(HCI_MAX_EVENT_SIZE > 256 ? HCI_MAX_EVENT_SIZE : 256)
#define HCI_SNOOP_ENTRY_NUM	30
#define HCI_SNOOP_BUF_SIZE	32

/**
 * stpbt device node
 */
#define BUFFER_SIZE	(1024 * 4)	/* Size of RX Queue */
#define IOC_MAGIC	0xb0
#define IOCTL_FW_ASSERT _IOWR(IOC_MAGIC, 0, void *)

/**
 * fw log queue count
 */
#define FWLOG_QUEUE_COUNT 200
#define FWLOG_ASSERT_QUEUE_COUNT 6000

/**
 * Maximum rom patch file name length
 */
#define MAX_BIN_FILE_NAME_LEN 32

/**
 *  Firmware version size
 */
#define FW_VERSION_BUF_SIZE 32	/* 14 bytes for firmware version + 1 bytes for '0' */
#define FW_VERSION_SIZE 15	/* 14 bytes for firmware version + 1 bytes for '0' */

#endif /* __BTMTK_DEFINE_H__ */
