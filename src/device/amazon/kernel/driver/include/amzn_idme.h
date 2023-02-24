/*
 * amzn_idme.h
 *
 * Copyright 2020 Amazon Technologies, Inc. All Rights Reserved.
 *
 * The code contained herein is licensed under the GNU General Public
 * License Version 2.
 * You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#ifndef _AMZN_IDME_H_
#define _AMZN_IDME_H_

#define ERR_BOARD_TYPE		0x0
#define ERR_BOARD_REV		0xFFFF
#define ERR_BATTERY_INFO	0xFFFF
#define ERR_ALS_VALUE		0x0
#define ERR_DEV_FLAGS		0xFFFFFFFF

unsigned int idme_get_battery_info(int index, size_t length);
unsigned int idme_get_board_type(void);
unsigned int idme_get_board_rev(void);
unsigned int idme_get_bootmode(void);
u64 idme_get_dev_flags_value(void);
const char *idme_get_item(char *idme_item);
bool board_has_wan(void);
#endif
