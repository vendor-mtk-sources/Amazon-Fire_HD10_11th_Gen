/*
 * Copyright (C) 2021 Amazon.com, Inc.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

#ifndef _MISC_AMAZON_LD_SWITCH_H
#define _MISC_AMAZON_LD_SWITCH_H

extern int liquid_id_status;
extern int amzn_ld_switch_is_support;

extern int amzn_ld_switch_adcsw3(int level);
extern void amzn_ld_switch_adcsw3_lock(void);
extern void amzn_ld_switch_adcsw3_unlock(void);

#define FUSB251_NO_MOUNT	1
#define NTC_BATTERY		1
#define ID_BATTERY		0
#define ADC_CHANNEL_0		0

#endif
