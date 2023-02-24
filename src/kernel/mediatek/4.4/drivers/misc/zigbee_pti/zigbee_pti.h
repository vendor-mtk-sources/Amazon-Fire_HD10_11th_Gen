/*******************************************************************************
* Copyright 2018 Amazon Technologies, Inc. All Rights Reserved.*
*
*@file	drivers/misc/zigbee_pti.h
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
#ifndef __ZIGBEE_PTI_H

#define __ZIGBEE_PTI_H

#include <linux/serial_core.h>
#include <linux/tty.h>

#define ZIGBEE_PTI_MAGIC	0x80
#define TIO_TEST	_IO(ZIGBEE_PTI_MAGIC, 3)

#define MAX_PACKET_BUF 8
#define MAX_PACKET_LEN 256

struct pti_ldisc_data {
	int	(*open)(struct tty_struct *);
	void	(*close)(struct tty_struct *);
};

struct pti_buf_t {
	/* index of buffer to write */
	int w_idx;
	/* index of buffer to read */
	int r_idx;
	/* number of packet available to upper layer read */
	int count;
	/* spin lock for data_buf */
	spinlock_t            lock;
	/* write position in currect buffer */
	int w_pos;
	/* packet buffers */
	char data_buf[MAX_PACKET_BUF][MAX_PACKET_LEN];
};

struct zigbee_pti_platform_data {
	/* GPIO number of PTI_SYNC */
	int pti_sync_gpio;
	/* IRQ assigned for PTI_SYNC */
	int pti_sync_irq;
};

struct zigbee_pti_entry_struct {
	/* Platform Data */
	struct zigbee_pti_platform_data platform_data;
	/* Struct to store ldisc ops */
	struct tty_ldisc_ops pti_ldisc_ops;
	/* Struct to store the open/close ops for NTTY */
	struct pti_ldisc_data pti_ldisc_saved;
	/* PTI packet buffer */
	struct pti_buf_t pti_data_buf;

#ifdef PTI_HAS_SHARED_UART_PINS
	/* Some products share PTI UART so must
	 * implement pinmux change at runtime
	 */
	struct pinctrl *pinctrl;
	struct pinctrl_state *toPTI;
	struct pinctrl_state *toOutsideFunction;
#endif
};

#ifdef PTI_HAS_SHARED_UART_PINS
enum pti_uart_pin_cfg{
	PTI_UART_PIN_CFG_TO_PTI=0,
	PTI_UART_PIN_CFG_TO_OUTSIDE_FUNCTION
};
#endif

#endif
