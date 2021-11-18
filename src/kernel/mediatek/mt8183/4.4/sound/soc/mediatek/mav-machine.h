/*
 * mav-machine.h  --  MAV machine driver definition
 *
 * Copyright (c) 2016 MediaTek Inc.
 * Author: Garlic Tseng <garlic.tseng@mediatek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _MAV_MACHINE_H_
#define _MAV_MACHINE_H_

#define BOARD_CHANNEL_TYPE_PROPERTY "channel-type"
#ifdef CONFIG_KPD_VOLUME_KEY_SWAP
extern void set_kpd_swap_vol_key(bool flag);
extern bool get_kpd_swap_vol_key(void);
#endif

#endif

