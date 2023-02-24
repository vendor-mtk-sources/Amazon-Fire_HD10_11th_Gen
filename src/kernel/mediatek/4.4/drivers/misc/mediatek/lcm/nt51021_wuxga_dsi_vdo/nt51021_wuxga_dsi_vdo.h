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

#ifndef _NT51021_WUXGA_DSI_VDO_
#define _NT51021_WUXGA_DSI_VDO_

extern unsigned int GPIO_LCD_RST_EN;
extern unsigned int LCM_ID1_GPIO;
extern unsigned int LCM_ID0_GPIO;

extern unsigned int idme_get_board_type(void);
extern unsigned int idme_get_board_rev(void);
#ifdef WITH_ENABLE_IDME
extern char *board_version(void);
#endif
#endif
