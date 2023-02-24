/*
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/*
 * Definitions for LTRX130A als sensor chip.
 */
#ifndef _LTRX130A_H_
#define _LTRX130A_H_

#include <linux/ioctl.h>
/* LTRX130A Registers */
#define LTRX130A_MAIN_CTRL          0x00
#define LTRX130A_ALS_MEAS_RATE      0x04
#define LTRX130A_ALS_GAIN           0x05
#define LTRX130A_INT_CFG            0x19
#define LTRX130A_INT_PST            0x1A
#define LTRX130A_ALS_THRES_UP_0     0x21
#define LTRX130A_ALS_THRES_UP_1     0x22
#define LTRX130A_ALS_THRES_UP_2     0x23
#define LTRX130A_ALS_THRES_LOW_0    0x24
#define LTRX130A_ALS_THRES_LOW_1    0x25
#define LTRX130A_ALS_THRES_LOW_2    0x26

/* Read Only Registers */
#define LTRX130A_PART_ID            0x06
#define LTRX130A_MAIN_STATUS        0x07
#define LTRX130A_CLEAR_DATA_0       0x0A
#define LTRX130A_CLEAR_DATA_1       0x0B
#define LTRX130A_CLEAR_DATA_2       0x0C
#define LTRX130A_ALS_DATA_0         0x0D
#define LTRX130A_ALS_DATA_1         0x0E
#define LTRX130A_ALS_DATA_2         0x0F

/* Basic Operating Modes */
#define MODE_ALS_Range1             0x00  /* for als gain x1 */
#define MODE_ALS_Range3             0x01  /* for als gain x3 */
#define MODE_ALS_Range6             0x02  /* for als gain x6 */
#define MODE_ALS_Range9             0x03  /* for als gain x9 */
#define MODE_ALS_Range18            0x04  /* for als gain x18 */

#define ALS_RANGE_1                 1
#define ALS_RANGE_3                 3
#define ALS_RANGE_6                 6
#define ALS_RANGE_9                 9
#define ALS_RANGE_18                18

#define ALS_RESO_MEAS               0x22

#define ALS_WIN_FACTOR              1
#define ALS_WIN_FACTOR_BLACK        39
#define ALS_WIN_FACTOR_BLACK_IR     44
#define ALS_WIN_FACTOR_WHITE        104
#define ALS_WIN_FACTOR_WHITE_IR     117
#define ALS_WIN_FACTOR2             5
#define ALS_MIX_DATA_FORMULA        1
/*
 * Magic Number
 * ============
 * Refer to file ioctl-number.txt for allocation
 */
#define LTRX130A_IOCTL_MAGIC        'c'

/* IOCTLs for LTRX130A device */
#define LTRX130A_IOCTL_ALS_ENABLE       _IOW(LTRX130A_IOCTL_MAGIC, 1, char *)
#define LTRX130A_IOCTL_READ_ALS_DATA    _IOR(LTRX130A_IOCTL_MAGIC, 2, char *)
#define LTRX130A_IOCTL_READ_ALS_INT     _IOR(LTRX130A_IOCTL_MAGIC, 3, char *)

/* Power On response time in ms */
#define PON_DELAY                   600
#define WAKEUP_DELAY                10

#define LTRX130A_SUCCESS             0
#define LTRX130A_ERR_I2C            -1
#define LTRX130A_ERR_STATUS         -3
#define LTRX130A_ERR_SETUP_FAILURE  -4
#define LTRX130A_ERR_GETGSENSORDATA	-5
#define LTRX130A_ERR_IDENTIFICATION	-6

/* Interrupt vector number to use when probing IRQ number.
 * User changeable depending on sys interrupt.
 * For IRQ numbers used, see /proc/interrupts.
 */
#define GPIO_INT_NO	32

extern struct platform_device *get_alsps_platformdev(void);

#endif
