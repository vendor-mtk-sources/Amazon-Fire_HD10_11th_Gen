/*
 * amzn_sign_of_life_rtc_impl.c
 *
 * [SOC] platform implementation
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
#include <linux/uaccess.h>
#include <linux/amzn_sign_of_life_rtc.h>
#include <amzn_sign_of_life_platform.h>

#define BOOT_REASON_NUM			4
#define SHUTDOWN_REASON_NUM		6
#define THERMAL_SHUTDOWN_REASON_NUM	8
#define SPECIAL_REASON_NUM		5

/* Use to record cold boot reason*/
life_cycle_reason_t lcr_boot = LIFE_CYCLE_NOT_AVAILABLE;

/* structure used for mapping rtc to lcr */
struct rtc_lcr_map {
	int rtc;
	life_cycle_reason_t lcr;
};

/* sign_of_life_private_data */
struct sign_of_life_private_data {
	unsigned int mask;
	unsigned int shift;
	unsigned int reg;
	struct rtc_lcr_map *rtc_to_lcr;
	unsigned int table_num;
};

static struct rtc_lcr_map rtc_to_boot_reason[BOOT_REASON_NUM] = {
	{RTC_BOOT_NONE, LIFE_CYCLE_NOT_AVAILABLE},
	/* Device Boot Reason */
	{RTC_WARMBOOT_BY_KERNEL_PANIC, WARMBOOT_BY_KERNEL_PANIC},
	{RTC_WARMBOOT_BY_KERNEL_WATCHDOG, WARMBOOT_BY_KERNEL_WATCHDOG},
	{RTC_WARMBOOT_BY_SW, WARMBOOT_BY_SW},
};

static struct rtc_lcr_map rtc_to_shutdown_reason[SHUTDOWN_REASON_NUM] = {
	{RTC_SHUTDOWN_NONE, LIFE_CYCLE_NOT_AVAILABLE},
	/* Device Shutdown Reason */
	{RTC_SHUTDOWN_BY_LONG_PWR_KEY_PRESS, SHUTDOWN_BY_LONG_PWR_KEY_PRESS},
	{RTC_SHUTDOWN_BY_SW, SHUTDOWN_BY_SW},
	{RTC_SHUTDOWN_BY_PWR_KEY, SHUTDOWN_BY_PWR_KEY},
	{RTC_SHUTDOWN_BY_SUDDEN_POWER_LOSS, SHUTDOWN_BY_SUDDEN_POWER_LOSS},
	{RTC_SHUTDOWN_BY_UNKNOWN_REASONS, SHUTDOWN_BY_UNKNOWN_REASONS},
};

static struct rtc_lcr_map rtc_to_thermal_shutdown_reason[THERMAL_SHUTDOWN_REASON_NUM] = {
	{RTC_THERMAL_SHUTDOWN_NONE, LIFE_CYCLE_NOT_AVAILABLE},
	/* Device Thermal Shutdown Reason */
	{RTC_THERMAL_SHUTDOWN_REASON_BATTERY, THERMAL_SHUTDOWN_REASON_BATTERY},
	{RTC_THERMAL_SHUTDOWN_REASON_PMIC, THERMAL_SHUTDOWN_REASON_PMIC},
	{RTC_THERMAL_SHUTDOWN_REASON_SOC, THERMAL_SHUTDOWN_REASON_SOC},
	{RTC_THERMAL_SHUTDOWN_REASON_MODEM, THERMAL_SHUTDOWN_REASON_MODEM},
	{RTC_THERMAL_SHUTDOWN_REASON_WIFI, THERMAL_SHUTDOWN_REASON_WIFI},
	{RTC_THERMAL_SHUTDOWN_REASON_PCB, THERMAL_SHUTDOWN_REASON_PCB},
	{RTC_THERMAL_SHUTDOWN_REASON_BTS, THERMAL_SHUTDOWN_REASON_BTS},
};

static struct rtc_lcr_map rtc_to_special_mode_reason[SPECIAL_REASON_NUM] = {
	{RTC_LIFE_CYCLE_SMODE_NONE, LIFE_CYCLE_NOT_AVAILABLE},
	/* LIFE CYCLE Special Mode */
	{RTC_LIFE_CYCLE_SMODE_LOW_BATTERY, LIFE_CYCLE_SMODE_LOW_BATTERY},
	{RTC_LIFE_CYCLE_SMODE_WARM_BOOT_USB_CONNECTED, LIFE_CYCLE_SMODE_WARM_BOOT_USB_CONNECTED},
	{RTC_LIFE_CYCLE_SMODE_OTA, LIFE_CYCLE_SMODE_OTA},
	{RTC_LIFE_CYCLE_SMODE_FACTORY_RESET, LIFE_CYCLE_SMODE_FACTORY_RESET}
};

static struct sign_of_life_private_data life_cycle_reason[LIFE_REASON_NUM] = {
	{
		.mask = RTC_BOOT_MASK,
		.shift = RTC_BOOT_SHIFT,
		.reg = BOOT_REASON_REG,
		.rtc_to_lcr = rtc_to_boot_reason,
		.table_num = ARRAY_SIZE(rtc_to_boot_reason),
	},
	{
		.mask = RTC_SHUTDOWN_MASK,
		.shift = RTC_SHUTDOWN_SHIFT,
		.reg = SHUTDOWN_REASON_REG,
		.rtc_to_lcr = rtc_to_shutdown_reason,
		.table_num = ARRAY_SIZE(rtc_to_shutdown_reason),
	},
	{
		.mask = RTC_THERMAL_SHUTDOWN_MASK,
		.shift = RTC_THERMAL_SHUTDOWN_SHIFT,
		.reg = THERMAL_SHUTDOWN_REASON_REG,
		.rtc_to_lcr = rtc_to_thermal_shutdown_reason,
		.table_num = ARRAY_SIZE(rtc_to_thermal_shutdown_reason),
	},
	{
		.mask = RTC_SPECIAL_MODE_MASK,
		.shift = RTC_SPECIAL_MODE_SHIFT,
		.reg = SPECIAL_MODE_REASON_REG,
		.rtc_to_lcr = rtc_to_special_mode_reason,
		.table_num = ARRAY_SIZE(rtc_to_special_mode_reason),
	},
};

int rtc_mark_quiescent(int value)
{
	u16 rtc_special_mode;

	lcr_rtc_lock();
	rtc_special_mode = lcr_rtc_read(QUIE_REG);
	if (value == 0)
		rtc_special_mode &= ~PMIC_RTC_QUIESCENT;
	else
		rtc_special_mode |= PMIC_RTC_QUIESCENT;
	lcr_rtc_write(QUIE_REG, rtc_special_mode);
	lcr_rtc_write_trigger();
	lcr_rtc_unlock();
	return 0;
}
EXPORT_SYMBOL(rtc_mark_quiescent);

static int search_rtc_by_lcr(const struct rtc_lcr_map *table,
			      int num, life_cycle_reason_t reason)
{
	int i;

	for (i = 0; i < num; i++) {
		if (table[i].lcr == reason)
			return table[i].rtc;
	}
	/* fails to match with any predefined reason, print a warnning msg */
	pr_warn("the lcr %d is not in the pre-defined table\n", reason);

	/* fails to find a good reason, return the default index= 0 */
	return table[0].rtc;
}

static life_cycle_reason_t search_lcr_by_rtc(const struct rtc_lcr_map *table,
			      int num, u16 reason)
{
	int i;

	for (i = 0; i < num; i++) {
		if (table[i].rtc == reason)
			return table[i].lcr;
	}
	/* fails to match with any predefined reason, print a warnning msg */
	pr_warn("the rtc %d is not in the pre-defined table\n", reason);

	/* fails to find a good reason, return the default index= 0 */
	return table[0].lcr;
}

static int get_sign_of_life_parity(void)
{
	int x = 0;
	int i;
	unsigned int mask = 0;
	unsigned int bit_count = 0;
	u32 parity = 0;
	u16 rtc_value = 0;
	u16 rtc_breason = 0;

	for (i = 0; i < ARRAY_SIZE(life_cycle_reason); i++) {
		lcr_rtc_lock();
		rtc_value = lcr_rtc_read(life_cycle_reason[i].reg);
		lcr_rtc_unlock();

		rtc_breason = ((rtc_value & life_cycle_reason[i].mask) >> life_cycle_reason[i].shift);

		parity |= (rtc_breason << bit_count);

		mask = (life_cycle_reason[i].mask >> life_cycle_reason[i].shift);

		do {
			bit_count++;
		} while(0 != (mask >>= 1));

		pr_debug("%s: rtc_value:0x%x,rtc_breason:0x%x,party:0x%x, bit_count=%d\n",
				__func__, rtc_value, rtc_breason, parity, bit_count);
	}

	for (i = 0; i < bit_count; i++) {
		x ^= (parity & 1);
		parity >>= 1;
	}

	return x;
}

static inline bool check_sign_of_life_parity(void)
{
	int x = get_sign_of_life_parity();
	u16 reg_parity = 0;
	u16 rtc_value = 0;

	lcr_rtc_lock();
	rtc_value = lcr_rtc_read(PARITY_REG);
	lcr_rtc_unlock();

	reg_parity = ((rtc_value & RTC_LIFE_CYCLE_REASON_PARITY_MASK) >> RTC_LIFE_CYCLE_REASON_PARITY_SHIFT);
	if (reg_parity != x) {
		pr_err("%s: Check Parity Failed, reg_parity is 0x%x,compute parity is 0x%x\n", __func__, reg_parity, x);
		return false;
	}

	return true;
}

static void rtc_clear_life_cycle_reason(void)
{
	int i;
	u16 rtc_value = 0;

	for (i = 0; i < ARRAY_SIZE(life_cycle_reason); i++) {
		lcr_rtc_lock();
		rtc_value = lcr_rtc_read(life_cycle_reason[i].reg);

		rtc_value &= ~life_cycle_reason[i].mask;

		lcr_rtc_write(life_cycle_reason[i].reg, rtc_value);
		lcr_rtc_write_trigger();

		lcr_rtc_unlock();

		pr_debug("%s: rtc_value:0x%x\n", __func__, rtc_value);
	}

	lcr_rtc_lock();
	rtc_value = lcr_rtc_read(PARITY_REG);

	rtc_value &= ~RTC_LIFE_CYCLE_REASON_PARITY_MASK;

	lcr_rtc_write(PARITY_REG, rtc_value);
	lcr_rtc_write_trigger();
	lcr_rtc_unlock();

	return;
}

static int rtc_read_life_cycle_reason(life_cycle_reason_t *reason, struct sign_of_life_private_data *pdata)
{
	u16 rtc_value = 0;
	u16 rtc_breason = 0;

	if (!pdata || !pdata->reg) {
		pr_err("%s: Pdata is NULL\n", __func__);
		return -EINVAL;
	}

	/*Read RTC to rtc_value*/
	lcr_rtc_lock();
	rtc_value = lcr_rtc_read(pdata->reg);
	lcr_rtc_unlock();

	rtc_breason = ((rtc_value & pdata->mask) >> pdata->shift);

	pr_info("%s: rtc_value:0x%x,rtc_breason:0x%x\n", __func__, rtc_value, rtc_breason);

	if (!check_sign_of_life_parity()) {
		pr_err("life cycle reason parity fail!\n");
		*reason = LIFE_CYCLE_SMODE_RTC_CHECK_FAIL;
	} else {
		*reason = search_lcr_by_rtc(pdata->rtc_to_lcr, pdata->table_num, rtc_breason);
	}

	return 0;
}

static int rtc_write_life_cycle_reason(life_cycle_reason_t reason, struct sign_of_life_private_data *pdata)
{
	u16 rtc_breason = 0;
	u16 life_cycle_breason = 0;

	if (!pdata) {
		pr_err("%s: Pdata is NULL\n", __func__);
		return -EINVAL;
	}

	life_cycle_breason = search_rtc_by_lcr(pdata->rtc_to_lcr, pdata->table_num, reason);

	/*Read rtc to rtc_breason*/
	lcr_rtc_lock();
	rtc_breason = lcr_rtc_read(pdata->reg);
	lcr_rtc_unlock();

	pr_debug("%s:current rtc 0x%x life cycle reason 0x%x\n", __func__, rtc_breason, reason);

	rtc_breason &= ~pdata->mask;
	rtc_breason |= ((life_cycle_breason << pdata->shift) & pdata->mask);

	pr_debug("%s:rtc_breason write 0x%x\n", __func__, rtc_breason);

	lcr_rtc_lock();
	lcr_rtc_write(pdata->reg, rtc_breason);
	lcr_rtc_write_trigger();
	lcr_rtc_unlock();

	/*write parity*/
	rtc_breason = 0;
	lcr_rtc_lock();
	rtc_breason = lcr_rtc_read(PARITY_REG);
	lcr_rtc_unlock();

	rtc_breason &= ~RTC_LIFE_CYCLE_REASON_PARITY_MASK;
	rtc_breason |= ((get_sign_of_life_parity() << RTC_LIFE_CYCLE_REASON_PARITY_SHIFT)
		& RTC_LIFE_CYCLE_REASON_PARITY_MASK);

	lcr_rtc_lock();
	lcr_rtc_write(PARITY_REG, rtc_breason);
	lcr_rtc_write_trigger();
	lcr_rtc_unlock();

	pr_debug("%s: Write parity is 0x%x\n", __func__, rtc_breason);

	return 0;
}

static int __init android_bootreason(char *options)
{
	if (!strcmp(POWER_KEY_BOOT, options))
		lcr_boot = COLDBOOT_BY_POWER_KEY;
	else if (!strcmp(USB_BOOT, options))
		lcr_boot = COLDBOOT_BY_USB;
	else if (!strcmp(WATCHDOG_SW, options))
		lcr_boot = WARMBOOT_BY_KERNEL_WATCHDOG;
	else if (!strcmp(WATCHDOG_HW, options))
		lcr_boot = WARMBOOT_BY_HW_WATCHDOG;
	else
		lcr_boot = LIFE_CYCLE_NOT_AVAILABLE;

	return 0;
}

__setup("androidboot.bootreason", android_bootreason);

static int (rtc_read_boot_reason)(life_cycle_reason_t *boot_reason)
{
	int ret = 0;

	ret = rtc_read_life_cycle_reason(boot_reason, &life_cycle_reason[BOOT]);

	if ((LIFE_CYCLE_NOT_AVAILABLE == *boot_reason) && (0 == ret))
		*boot_reason = lcr_boot;

	return ret;
}

static int (rtc_write_boot_reason)(life_cycle_reason_t boot_reason)
{
	return rtc_write_life_cycle_reason(boot_reason, &life_cycle_reason[BOOT]);
}

static int (rtc_read_shutdown_reason)(life_cycle_reason_t *shutdown_reason)
{
	return rtc_read_life_cycle_reason(shutdown_reason, &life_cycle_reason[SHUTDOWN]);
}

static int (rtc_write_shutdown_reason)(life_cycle_reason_t shutdown_reason)
{
	return rtc_write_life_cycle_reason(shutdown_reason, &life_cycle_reason[SHUTDOWN]);
}

static int (rtc_read_thermal_shutdown_reason)(life_cycle_reason_t *thermal_shutdown_reason)
{
	return rtc_read_life_cycle_reason(thermal_shutdown_reason, &life_cycle_reason[THERMAL_SHUTDOWN]);
}

static int (rtc_write_thermal_shutdown_reason)(life_cycle_reason_t thermal_shutdown_reason)
{
	return rtc_write_life_cycle_reason(thermal_shutdown_reason, &life_cycle_reason[THERMAL_SHUTDOWN]);
}

static int (rtc_read_special_mode)(life_cycle_reason_t *special_mode)
{
	return rtc_read_life_cycle_reason(special_mode, &life_cycle_reason[SPECIAL]);
}

static int (rtc_write_special_mode)(life_cycle_reason_t special_mode)
{
	return rtc_write_life_cycle_reason(special_mode, &life_cycle_reason[SPECIAL]);
}

static int rtc_lcr_reset(void)
{
	rtc_clear_life_cycle_reason();
	return 0;
}

int life_cycle_platform_init(sign_of_life_ops *sol_ops)
{
	pr_info("%s: Support soc platform\n", __func__);
	sol_ops->read_boot_reason = rtc_read_boot_reason;
	sol_ops->write_boot_reason = rtc_write_boot_reason;
	sol_ops->read_shutdown_reason = rtc_read_shutdown_reason;
	sol_ops->write_shutdown_reason = rtc_write_shutdown_reason;
	sol_ops->read_thermal_shutdown_reason = rtc_read_thermal_shutdown_reason;
	sol_ops->write_thermal_shutdown_reason = rtc_write_thermal_shutdown_reason;
	sol_ops->read_special_mode = rtc_read_special_mode;
	sol_ops->write_special_mode = rtc_write_special_mode;
	sol_ops->lcr_reset = rtc_lcr_reset;

	return 0;
}
