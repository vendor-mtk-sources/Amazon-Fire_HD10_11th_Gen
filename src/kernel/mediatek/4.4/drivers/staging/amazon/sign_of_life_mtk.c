/*
 * sign_of_life_mtk.c
 *
 * MTK platform implementation
 *
 * Copyright (C) Amazon Technologies Inc. All rights reserved.
 * Yang Liu (yangliu@lab126.com)
 * TODO: Add additional contributor's names.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/vmalloc.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/sign_of_life.h>
#include "../../misc/mediatek/include/mt-plat/mtk_rtc.h"
#include "../../misc/mediatek/include/mt-plat/mt8183/include/mach/mtk_rtc_hal.h"
#include "../../misc/mediatek/include/mt-plat/mtk_rtc_hal_common.h"
#include "../../misc/mediatek/include/mt-plat/mtk_pmic_wrap.h"
#include "../../misc/mediatek/rtc/mt6358/mtk_rtc_hw.h"

/* RTC Spare Register Definition */

/*
 * RTC_SPAR0:
 *	bit 0 - 5 : SEC in power-on time
 *	bit 6 : 32K less bit. True:with 32K, False:Without 32K
 *	bit 7 : LP_DET
 *	bit 8 : Enter KPOC
 *	bit 9 : Enter SW LPRST
 *	bit 10 - 15: reserved bits
 */
#define RTC_SPAR0_SHUTDOWN_LONG_PWR_KEY_PRESS	(1U << 10)
#define RTC_SPAR0_SHUTDOWN_SW			(1U << 11)
#define RTC_SPAR0_SHUTDOWN_MASK			0x0c00

#define RTC_SPAR0_WARM_BOOT_KERNEL_PANIC	(1U << 12)
#define RTC_SPAR0_WARM_BOOT_KERNEL_WDOG		(1U << 13)
#define RTC_SPAR0_WARM_BOOT_HW_WDOG		(1U << 14)
#define RTC_SPAR0_WARM_BOOT_SW			(1U << 15)
#define RTC_SPAR0_WARM_BOOT_MASK		0xf000

/*
 * RTC_NEW_SPARE2:
 *	bit 8 - 15 : RTC_AL_DOW reserved bits
 */
#define RTC_NEW_SPARE2_THERMAL_SHUTDOWN_PMIC_VALUE		0x0100
#define RTC_NEW_SPARE2_THERMAL_SHUTDOWN_SOC_VALUE		0x0200
#define RTC_NEW_SPARE2_THERMAL_SHUTDOWN_MOTOR_VALUE		0x0300
#define RTC_NEW_SPARE2_THERMAL_SHUTDOWN_MOTOR_PCB_VALUE		0x0400
#define RTC_NEW_SPARE2_THERMAL_SHUTDOWN_MLB_VALUE		0x0500
#define RTC_NEW_SPARE2_THERMAL_SHUTDOWN_DISP_VALUE		0x0600
#define RTC_NEW_SPARE2_THERMAL_SHUTDOWN_UPPER_DRUM_VALUE	0x0700
#define RTC_NEW_SPARE2_THERMAL_SHUTDOWN_DRUM_BASE_VALUE		0x0800
#define RTC_NEW_SPARE2_THERMAL_SHUTDOWN_CPVS_VALUE		0x0900
#define RTC_NEW_SPARE2_THERMAL_SHUTDOWN_DUAL_SOC_VALUE		0x0A00
#define RTC_NEW_SPARE2_THERMAL_SHUTDOWN_SVS_VALUE		0x0B00
#define RTC_NEW_SPARE2_THERMAL_SHUTDOWN_MASK			0x3f00

#define RTC_NEW_SPARE2_SPECIAL_MODE_LOW_BATTERY	(1U << 14)
#define RTC_NEW_SPARE2_SPECIAL_MODE_MASK       0x4000

/*
* RTC_SPAR1:
*     bit 0      : QUIESCENT mode
*     bit 1 - 15 : reserved.
*/
#define PMIC_RTC_SPAR1_QUIESCENT        (1U << 0)
#define PMIC_RTC_SPAR1_QUIESCENT_MASK   (1U << 0)


/*drivers/watchdog/mediatek/wdt/common/mtk_wdt.c*/
extern void mtk_wdt_mode_config(bool dual_mode_en, bool irq, bool ext_en, bool ext_pol, bool wdt_en);

static life_cycle_reason_t lc_boot = LIFE_CYCLE_NOT_AVAILABLE;

static u16 set_val(u16 old, u16 val, u16 mask)
{
	return (old & ~mask) | (val & mask);
}

static void rtc_acquire_lock(void)
{
	/* FIXME
	 * MTK RTC code doesn't use any lock in their code drop.
	 * We will rtc_acquire_lock once MTK add lock protection in
	 * mtk_rtc_hal_common.c */
	return;
}

static void rtc_release_lock(void)

{
	return;
}

static int (mtk_read_boot_reason)(life_cycle_reason_t *boot_reason)
{
	u16 rtc_boot_reason;
	*boot_reason = lc_boot;

	rtc_acquire_lock();
	rtc_boot_reason = rtc_read(RTC_SPAR0);
	rtc_release_lock();

	printk(KERN_ERR"%s: boot reason is 0x%x\n", __func__, rtc_boot_reason & RTC_SPAR0_WARM_BOOT_MASK);

	if (lc_boot == LIFE_CYCLE_NOT_AVAILABLE) {
		if (rtc_boot_reason & RTC_SPAR0_WARM_BOOT_KERNEL_PANIC)
			*boot_reason = WARMBOOT_BY_KERNEL_PANIC;
		else if (rtc_boot_reason & RTC_SPAR0_WARM_BOOT_SW)
			*boot_reason = WARMBOOT_BY_SW;
		else if (rtc_boot_reason & RTC_SPAR0_WARM_BOOT_KERNEL_WDOG)
			*boot_reason = WARMBOOT_BY_KERNEL_WATCHDOG;
		else if (rtc_boot_reason & RTC_SPAR0_WARM_BOOT_HW_WDOG)
			*boot_reason = WARMBOOT_BY_HW_WATCHDOG;
		else {
			printk(KERN_ERR"Failed to read rtc boot reason\n");
			return -1;
		}
	}
	return 0;
}

static int (mtk_write_boot_reason)(life_cycle_reason_t boot_reason)
{
	u16 rtc_boot_reason;

	rtc_acquire_lock();
	rtc_boot_reason = rtc_read(RTC_SPAR0);

	printk(KERN_ERR"%s:boot_reason 0x%x\n", __func__, boot_reason);

	if (boot_reason == WARMBOOT_BY_KERNEL_PANIC)
		rtc_boot_reason = rtc_boot_reason | RTC_SPAR0_WARM_BOOT_KERNEL_PANIC;
	else if (boot_reason == WARMBOOT_BY_KERNEL_WATCHDOG)
		rtc_boot_reason = rtc_boot_reason | RTC_SPAR0_WARM_BOOT_KERNEL_WDOG;
	else if (boot_reason == WARMBOOT_BY_HW_WATCHDOG)
		rtc_boot_reason = rtc_boot_reason | RTC_SPAR0_WARM_BOOT_HW_WDOG;
	else if (boot_reason == WARMBOOT_BY_SW)
		rtc_boot_reason = rtc_boot_reason | RTC_SPAR0_WARM_BOOT_SW;

	rtc_write(RTC_SPAR0, rtc_boot_reason);
	rtc_write_trigger();
	rtc_release_lock();

	return 0;
}

static int __init lcr_mtk_bootreason(char *options)
{
	if (!strcmp("=power_key", options))
		lc_boot = COLDBOOT_BY_POWER_KEY;
	else if (!strcmp("=usb", options))
		lc_boot = COLDBOOT_BY_USB;
	else
		lc_boot = LIFE_CYCLE_NOT_AVAILABLE;

	return 0;
}
__setup("androidboot.bootreason", lcr_mtk_bootreason);

static int (mtk_read_shutdown_reason)(life_cycle_reason_t *shutdown_reason)
{
	u16 rtc_shutdown_reason;

	rtc_acquire_lock();
	rtc_shutdown_reason = rtc_read(RTC_SPAR0);
	rtc_release_lock();

	printk(KERN_ERR"%s: shutdown reason is 0x%x\n", __func__, rtc_shutdown_reason & RTC_SPAR0_SHUTDOWN_MASK);
	if (rtc_shutdown_reason & RTC_SPAR0_SHUTDOWN_LONG_PWR_KEY_PRESS)
		*shutdown_reason = SHUTDOWN_BY_LONG_PWR_KEY_PRESS;
	else if (rtc_shutdown_reason & RTC_SPAR0_SHUTDOWN_SW)
		*shutdown_reason = SHUTDOWN_BY_SW;
	else {
		printk(KERN_ERR"Failed to read rtc shutdown reason\n");
		return -1;
	}

	return 0;
}

static int (mtk_write_shutdown_reason)(life_cycle_reason_t shutdown_reason)
{
	u16 rtc_shutdown_reason;

	rtc_acquire_lock();
	rtc_shutdown_reason = rtc_read(RTC_SPAR0);

	printk(KERN_ERR"%s:shutdown_reason 0x%x\n", __func__, shutdown_reason);

	if (shutdown_reason == SHUTDOWN_BY_LONG_PWR_KEY_PRESS)
		rtc_shutdown_reason = rtc_shutdown_reason | RTC_SPAR0_SHUTDOWN_LONG_PWR_KEY_PRESS;
	else if (shutdown_reason == SHUTDOWN_BY_SW)
		rtc_shutdown_reason = rtc_shutdown_reason | RTC_SPAR0_SHUTDOWN_SW;

	rtc_write(RTC_SPAR0, rtc_shutdown_reason);
	rtc_write_trigger();
	rtc_release_lock();
	return 0;
}

static int (mtk_read_thermal_shutdown_reason)(life_cycle_reason_t *thermal_shutdown_reason)
{
	u16 rtc_thermal_shutdown_reason;

	rtc_acquire_lock();
	rtc_thermal_shutdown_reason = rtc_read(RTC_AL_DOW);
	rtc_release_lock();

	rtc_thermal_shutdown_reason &= RTC_NEW_SPARE2_THERMAL_SHUTDOWN_MASK;
	printk(KERN_INFO"%s: thermal shutdown reason 0x%x\n", __func__, rtc_thermal_shutdown_reason);
	switch (rtc_thermal_shutdown_reason) {
	case RTC_NEW_SPARE2_THERMAL_SHUTDOWN_PMIC_VALUE:
		*thermal_shutdown_reason = THERMAL_SHUTDOWN_REASON_PMIC;
		break;
	case RTC_NEW_SPARE2_THERMAL_SHUTDOWN_SOC_VALUE:
		*thermal_shutdown_reason = THERMAL_SHUTDOWN_REASON_SOC;
		break;
	case RTC_NEW_SPARE2_THERMAL_SHUTDOWN_MOTOR_VALUE:
		*thermal_shutdown_reason = THERMAL_SHUTDOWN_REASON_MOTOR;
		break;
	case RTC_NEW_SPARE2_THERMAL_SHUTDOWN_MOTOR_PCB_VALUE:
		*thermal_shutdown_reason = THERMAL_SHUTDOWN_REASON_MOTOR_PCB;
		break;
	case RTC_NEW_SPARE2_THERMAL_SHUTDOWN_MLB_VALUE:
		*thermal_shutdown_reason = THERMAL_SHUTDOWN_REASON_MLB;
		break;
	case RTC_NEW_SPARE2_THERMAL_SHUTDOWN_DISP_VALUE:
		*thermal_shutdown_reason = THERMAL_SHUTDOWN_REASON_DISP;
		break;
	case RTC_NEW_SPARE2_THERMAL_SHUTDOWN_UPPER_DRUM_VALUE:
		*thermal_shutdown_reason = THERMAL_SHUTDOWN_REASON_UPPER_DRUM;
		break;
	case RTC_NEW_SPARE2_THERMAL_SHUTDOWN_DRUM_BASE_VALUE:
		*thermal_shutdown_reason = THERMAL_SHUTDOWN_REASON_DRUM_BASE;
		break;
	case RTC_NEW_SPARE2_THERMAL_SHUTDOWN_CPVS_VALUE:
		*thermal_shutdown_reason = THERMAL_SHUTDOWN_REASON_CPVS;
		break;
	case RTC_NEW_SPARE2_THERMAL_SHUTDOWN_DUAL_SOC_VALUE:
		*thermal_shutdown_reason = THERMAL_SHUTDOWN_REASON_DUAL_SOC;
		break;
	case RTC_NEW_SPARE2_THERMAL_SHUTDOWN_SVS_VALUE:
		*thermal_shutdown_reason = THERMAL_SHUTDOWN_REASON_SVS;
		break;
	default:
		printk(KERN_ERR"Failed to read rtc thermal shutdown reason\n");
		return -EOPNOTSUPP;
	}

	return 0;
}

static int (mtk_write_thermal_shutdown_reason)(life_cycle_reason_t thermal_shutdown_reason)
{
	u16 rtc_thermal_shutdown_reason;
	u16 thermal_shutdown_reason_val = 0;
	int ret = 0;

	rtc_acquire_lock();
	rtc_thermal_shutdown_reason = rtc_read(RTC_AL_DOW);

	printk(KERN_ERR"%s:shutdown_reason 0x%0x\n", __func__, thermal_shutdown_reason);

	switch (thermal_shutdown_reason) {
	case THERMAL_SHUTDOWN_REASON_PMIC:
		thermal_shutdown_reason_val = RTC_NEW_SPARE2_THERMAL_SHUTDOWN_PMIC_VALUE;
		break;
	case THERMAL_SHUTDOWN_REASON_SOC:
		thermal_shutdown_reason_val = RTC_NEW_SPARE2_THERMAL_SHUTDOWN_SOC_VALUE;
		break;
	case THERMAL_SHUTDOWN_REASON_MOTOR:
		thermal_shutdown_reason_val = RTC_NEW_SPARE2_THERMAL_SHUTDOWN_MOTOR_VALUE;
		break;
	case THERMAL_SHUTDOWN_REASON_MOTOR_PCB:
		thermal_shutdown_reason_val = RTC_NEW_SPARE2_THERMAL_SHUTDOWN_MOTOR_PCB_VALUE;
		break;
	case THERMAL_SHUTDOWN_REASON_MLB:
		thermal_shutdown_reason_val = RTC_NEW_SPARE2_THERMAL_SHUTDOWN_MLB_VALUE;
		break;
	case THERMAL_SHUTDOWN_REASON_DISP:
		thermal_shutdown_reason_val = RTC_NEW_SPARE2_THERMAL_SHUTDOWN_DISP_VALUE;
		break;
	case THERMAL_SHUTDOWN_REASON_UPPER_DRUM:
		thermal_shutdown_reason_val = RTC_NEW_SPARE2_THERMAL_SHUTDOWN_UPPER_DRUM_VALUE;
		break;
	case THERMAL_SHUTDOWN_REASON_DRUM_BASE:
		thermal_shutdown_reason_val = RTC_NEW_SPARE2_THERMAL_SHUTDOWN_DRUM_BASE_VALUE;
		break;
	case THERMAL_SHUTDOWN_REASON_CPVS:
		thermal_shutdown_reason_val = RTC_NEW_SPARE2_THERMAL_SHUTDOWN_CPVS_VALUE;
		break;
	case THERMAL_SHUTDOWN_REASON_DUAL_SOC:
		thermal_shutdown_reason_val = RTC_NEW_SPARE2_THERMAL_SHUTDOWN_DUAL_SOC_VALUE;
		break;
	case THERMAL_SHUTDOWN_REASON_SVS:
		thermal_shutdown_reason_val = RTC_NEW_SPARE2_THERMAL_SHUTDOWN_SVS_VALUE;
		break;
	default:
		printk(KERN_ERR"%s:unsupported thermal life cycle reason 0x%0x\n", __func__,
				thermal_shutdown_reason);
		ret = -EOPNOTSUPP;
	}

	if (!ret) {
		rtc_thermal_shutdown_reason = set_val(rtc_thermal_shutdown_reason,
				thermal_shutdown_reason_val, RTC_NEW_SPARE2_THERMAL_SHUTDOWN_MASK);
		rtc_write(RTC_AL_DOW, rtc_thermal_shutdown_reason);
		rtc_write_trigger();
	}

	rtc_release_lock();
	return ret;
}

static int (mtk_read_special_mode)(life_cycle_reason_t *special_mode)
{
	u16 rtc_special_mode;

	rtc_acquire_lock();
	rtc_special_mode = rtc_read(RTC_AL_DOW);
	rtc_release_lock();

	printk(KERN_ERR"%s: special mode is 0x%x\n", __func__, rtc_special_mode & RTC_NEW_SPARE2_SPECIAL_MODE_MASK);
	if (rtc_special_mode & RTC_NEW_SPARE2_SPECIAL_MODE_LOW_BATTERY)
		*special_mode = LIFE_CYCLE_SMODE_LOW_BATTERY;
	else {
		printk(KERN_ERR"Failed to read rtc special mode\n");
		return -1;
	}

	return 0;
}

static int (mtk_write_special_mode)(life_cycle_reason_t special_mode)
{
	u16 rtc_special_mode;

	rtc_acquire_lock();
	rtc_special_mode = rtc_read(RTC_AL_DOW);

	printk(KERN_ERR"%s:special_mode 0x%x\n", __func__, special_mode);

	if (special_mode == LIFE_CYCLE_SMODE_LOW_BATTERY)
		rtc_special_mode = rtc_special_mode | RTC_NEW_SPARE2_SPECIAL_MODE_LOW_BATTERY;

	rtc_write(RTC_AL_DOW, rtc_special_mode);
	rtc_write_trigger();
	rtc_release_lock();
	return 0;
}

int rtc_mark_quiescent(int value)
{
	u32 rtc_special_mode;

	rtc_acquire_lock();
	pwrap_read(RTC_SPAR1, &rtc_special_mode);
	if (value == 0)
		rtc_special_mode &= ~PMIC_RTC_SPAR1_QUIESCENT;
	else
		rtc_special_mode |= PMIC_RTC_SPAR1_QUIESCENT;
	rtc_write(RTC_SPAR1, rtc_special_mode);
	rtc_write_trigger();
	rtc_release_lock();
	return 0;
}
EXPORT_SYMBOL(rtc_mark_quiescent);

int mtk_lcr_reset(void)
{
	u16 data;

	rtc_acquire_lock();
	/* clean up the shutdown and boot reason */
	data = rtc_read(RTC_SPAR0);
	data = data & ~(RTC_SPAR0_SHUTDOWN_MASK | RTC_SPAR0_WARM_BOOT_MASK);
	rtc_write(RTC_SPAR0, data);

	/* clean up the thermal shutdown reason and special mode */
	data = rtc_read(RTC_AL_DOW);
	data = data & ~(RTC_NEW_SPARE2_THERMAL_SHUTDOWN_MASK | RTC_NEW_SPARE2_SPECIAL_MODE_MASK);
	rtc_write(RTC_AL_DOW, data);

	rtc_write_trigger();
	rtc_release_lock();

	return 0;
}

int life_cycle_platform_init(sign_of_life_ops *sol_ops)
{
	printk(KERN_ERR "%s: Support MTK platform\n", __func__);
	sol_ops->read_boot_reason = mtk_read_boot_reason;
	sol_ops->write_boot_reason = mtk_write_boot_reason;
	sol_ops->read_shutdown_reason = mtk_read_shutdown_reason;
	sol_ops->write_shutdown_reason = mtk_write_shutdown_reason;
	sol_ops->read_thermal_shutdown_reason = mtk_read_thermal_shutdown_reason;
	sol_ops->write_thermal_shutdown_reason = mtk_write_thermal_shutdown_reason;
	sol_ops->read_special_mode = mtk_read_special_mode;
	sol_ops->write_special_mode = mtk_write_special_mode;
	sol_ops->lcr_reset = mtk_lcr_reset;

	return 0;
}


void kree_disable_fiq(int);
void sysrq_trigger_lcr_test(int act)
{
	unsigned long flags;
	DEFINE_SPINLOCK(wdt_test_lock);

	switch (act) {
	case 3:
		spin_lock_irqsave(&wdt_test_lock, flags);
		/* mtk_wdt_mode_config(0, 0, 1, 0, 1); */
		while (1) {
			;
		};
		break;
	case 2:
		spin_lock(&wdt_test_lock);
		while (1)
			;
		break;
	default:
		/* *(int *)NULL = 0; */
		break;
	}
}
EXPORT_SYMBOL(sysrq_trigger_lcr_test);
