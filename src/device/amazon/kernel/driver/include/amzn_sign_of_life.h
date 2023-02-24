/*
* amzn_sign_of_life.h
*
* Device sign of life header file
*
* Copyright (C) 2015-2020 Amazon Technologies Inc. All rights reserved.
* Yang Liu (yangliu@lab126.com)
* Jiangli Yuan (jly@amazon.com)
* TODO: Add additional contributor's names.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*/

#ifndef __AMZN_SIGN_OF_LIFE_H
#define __AMZN_SIGN_OF_LIFE_H

typedef enum {
	/* Device Boot Reason */
	LIFE_CYCLE_NOT_AVAILABLE     = -1,
	WARMBOOT_BY_KERNEL_PANIC     = 0x100,
	WARMBOOT_BY_KERNEL_WATCHDOG  = 0x101,
	WARMBOOT_BY_HW_WATCHDOG      = 0x102,
	WARMBOOT_BY_SW               = 0x103,
	COLDBOOT_BY_USB              = 0x104,
	COLDBOOT_BY_POWER_KEY        = 0x105,
	COLDBOOT_BY_POWER_SUPPLY     = 0x106,
	/* Device Shutdown Reason */
	SHUTDOWN_BY_LONG_PWR_KEY_PRESS = 0x201,
	SHUTDOWN_BY_SW                 = 0x202,
	SHUTDOWN_BY_PWR_KEY            = 0x203,
	SHUTDOWN_BY_SUDDEN_POWER_LOSS  = 0x204,
	SHUTDOWN_BY_UNKNOWN_REASONS    = 0x205,
	THERMAL_SHUTDOWN_REASON_BATTERY = 0x301,
	THERMAL_SHUTDOWN_REASON_PMIC    = 0x302,
	THERMAL_SHUTDOWN_REASON_SOC     = 0x303,
	THERMAL_SHUTDOWN_REASON_MODEM   = 0x304,
	THERMAL_SHUTDOWN_REASON_WIFI    = 0x305,
	THERMAL_SHUTDOWN_REASON_PCB     = 0x306,
	THERMAL_SHUTDOWN_REASON_BTS     = 0x307,
	/* LIFE CYCLE Special Mode */
	LIFE_CYCLE_SMODE_NONE		= 0x400,
	LIFE_CYCLE_SMODE_LOW_BATTERY    = 0x401,
	LIFE_CYCLE_SMODE_WARM_BOOT_USB_CONNECTED  = 0x402,
	LIFE_CYCLE_SMODE_OTA            = 0x403,
	LIFE_CYCLE_SMODE_FACTORY_RESET  = 0x404,
	LIFE_CYCLE_SMODE_RTC_CHECK_FAIL  = 0x405,
} life_cycle_reason_t;


/*
 * Below interfaces used by product to record life cycle reason.
 */


/*
 *  * life_cycle_set_boot_reason
 *   * Description: this function will set the boot reason which causing device booting
 *    */
int life_cycle_set_boot_reason(life_cycle_reason_t boot_reason);

/*
 *  * life_cycle_set_shutdown_reason
 *   * Description: this function will set the Shutdown reason which causing device shutdown
 *    */
int life_cycle_set_shutdown_reason(life_cycle_reason_t shutdown_reason);

/*
 *  * life_cycle_set_thermal_shutdown_reason
 *   * Description: this function will set the Thermal Shutdown reason which causing device shutdown
 *    */
int life_cycle_set_thermal_shutdown_reason(life_cycle_reason_t thermal_shutdown_reason);

/*
 *  * life_cycle_set_special_mode
 *   * Description: this function will set the special mode which causing device life cycle change
 *    */
int life_cycle_set_special_mode(life_cycle_reason_t life_cycle_special_mode);

#endif
