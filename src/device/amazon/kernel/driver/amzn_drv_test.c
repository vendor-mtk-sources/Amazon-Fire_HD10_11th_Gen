/* Amazon Common Driver Tester
 *
 * Copyright 2016-2020 Amazon Technologies, Inc. All Rights Reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/cpu.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/uaccess.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <include/amzn_sign_of_life.h>
#include <include/amzn_metricslog.h>
#include <include/amzn_idme.h>
#include <linux/amzn_sign_of_life_rtc.h>
#include <amzn_sign_of_life_platform.h>

#define AMZN_DRIVERS	        "amzn_drvs"
#define AMZN_SIGN_OF_LIFE	"sign_of_life"
#define AMZN_IDME               "idme"
#define AMZN_LOGGER             "logger"

#define RESULT_BUF_SIZE         2048
#define TEST_ITEM_SIZE          20
#define METRICS_STR_LEN         128

/* Test timeout threshold, unit: ms*/
#define TEST_TIMEOUT            2000

static int logger_loop = 1;
module_param(logger_loop, uint, 0644);
MODULE_PARM_DESC(logger_loop, "# of loop");

struct bootup_life_cycle_reason {
	int boot_index;
	int shutdown_index;
	int thermal_shutdown_index;
	int special_mode_index;
};

struct drv_test_data {
	char test_item[TEST_ITEM_SIZE];
	int test_index;
	struct rw_semaphore rwlock;
	char result_buf[RESULT_BUF_SIZE];
};

static struct bootup_life_cycle_reason boot_index;
static struct drv_test_data *life_data;
static struct drv_test_data *idme_data;
static struct drv_test_data *logger_data;

void read_bootup_life_reason(void)
{
	int index = -1;

	index = read_life_cycle_reason(BOOT);
	if (index < 0 ) {
		pr_err("read boot reason Failed \n");
		boot_index.boot_index = -1;
	} else {
		pr_info("boot reason is %s\n",
			lcr_data[index].life_cycle_reasons);

		boot_index.boot_index = index;
	}

	index = read_life_cycle_reason(SHUTDOWN);
	if (index < 0 ) {
		pr_err("read shutdown reason Failed \n");
		boot_index.shutdown_index = -1;
	} else {
		pr_info("shutdown reason is %s\n",
			lcr_data[index].life_cycle_reasons);

		boot_index.shutdown_index = index;
	}

	index = read_life_cycle_reason(THERMAL_SHUTDOWN);
	if (index < 0 ) {
		pr_err("read thermal shutdown reason Failed \n");
		boot_index.thermal_shutdown_index = -1;
	} else {
		pr_info("thermal shutdown reason is %s\n",
			lcr_data[index].life_cycle_reasons);

		boot_index.thermal_shutdown_index = index;
	}

	index = read_life_cycle_reason(SPECIAL);
	if (index < 0 ) {
		pr_err("read speial mode reason Failed \n");
		boot_index.special_mode_index = -1;
	} else {
		pr_info("special mode reason is %s\n",
			lcr_data[index].life_cycle_reasons);

		boot_index.special_mode_index = index;
	}
}

static void bootup_life_reason(struct drv_test_data *pdata)
{
	int n = 0;

	if (boot_index.boot_index < 0) {
		n += scnprintf(pdata->result_buf+n, sizeof(pdata->result_buf)-n,
				"Boot up boot reason: Read Failed\n");
	} else {
		n += scnprintf(pdata->result_buf+n, sizeof(pdata->result_buf)-n,
				"Boot up boot reason: %s\n",
				lcr_data[boot_index.boot_index].life_cycle_reasons);
	}

	if (boot_index.shutdown_index < 0) {
		n += scnprintf(pdata->result_buf+n, sizeof(pdata->result_buf)-n,
				"Boot up shutdown reason: Read Failed\n");
	} else {
		n += scnprintf(pdata->result_buf+n, sizeof(pdata->result_buf)-n,
				"Boot up shutdown reason: %s\n",
				lcr_data[boot_index.shutdown_index].life_cycle_reasons);
	}

	if (boot_index.thermal_shutdown_index < 0) {
		n += scnprintf(pdata->result_buf+n, sizeof(pdata->result_buf)-n,
				"Boot up thermal_shutdown reason: Read Failed\n");
	} else {
		n += scnprintf(pdata->result_buf+n, sizeof(pdata->result_buf)-n,
				"Boot up thermal_shutdown reason: %s\n",
				lcr_data[boot_index.thermal_shutdown_index].life_cycle_reasons);
	}

	if (boot_index.special_mode_index < 0) {
		n += scnprintf(pdata->result_buf+n, sizeof(pdata->result_buf)-n,
				"Boot up special_mode reason: Read Failed\n");
	} else {
		n += scnprintf(pdata->result_buf+n, sizeof(pdata->result_buf)-n,
				"Boot up special mode reason: %s\n",
				lcr_data[boot_index.special_mode_index].life_cycle_reasons);
	}
}

static void read_all_life_reason(struct drv_test_data *pdata)
{
	int index = -1;
	int n = 0;

	index = read_life_cycle_reason(BOOT);
	if (index < 0 ) {
		pr_err("read boot reason Failed \n");
		n += scnprintf(pdata->result_buf+n, sizeof(pdata->result_buf)-n,
				"boot reason: Read Failed\n");
	} else {
		pr_info("boot reason is %s\n",
			lcr_data[index].life_cycle_reasons);

		n += scnprintf(pdata->result_buf+n, sizeof(pdata->result_buf)-n,"boot reason: %s\n",
			lcr_data[index].life_cycle_reasons);
	}

	index = read_life_cycle_reason(SHUTDOWN);
	if (index < 0 ) {
		pr_err("read shutdown reason Failed \n");
		n += scnprintf(pdata->result_buf+n, sizeof(pdata->result_buf)-n,
			"shutdown reason: Read Failed\n");
	} else {
		pr_info("shutdown reason is %s\n",
			lcr_data[index].life_cycle_reasons);

		n += scnprintf(pdata->result_buf+n, sizeof(pdata->result_buf)-n,"shutdown reason: %s\n",
			lcr_data[index].life_cycle_reasons);
	}

	index = read_life_cycle_reason(THERMAL_SHUTDOWN);
	if (index < 0 ) {
		pr_err("read thermal shutdown reason Failed \n");
		n += scnprintf(pdata->result_buf+n, sizeof(pdata->result_buf)-n,
			"thermal shutdown reason: Read Failed\n");
	} else {
		pr_info("thermal shutdown reason is %s\n",
			lcr_data[index].life_cycle_reasons);

		n += scnprintf(pdata->result_buf+n, sizeof(pdata->result_buf)-n,"thermal shutdown reason:%s\n",
			lcr_data[index].life_cycle_reasons);
	}

	index = read_life_cycle_reason(SPECIAL);
	if (index < 0 ) {
		pr_err("read speial mode reason Failed \n");
		n += scnprintf(pdata->result_buf+n, sizeof(pdata->result_buf)-n,
			"special mode reason: Read Failed\n");
	} else {
		pr_info("special mode reason is %s\n",
			lcr_data[index].life_cycle_reasons);

		n += scnprintf(pdata->result_buf+n, sizeof(pdata->result_buf)-n,"special mode reason: %s\n",
			lcr_data[index].life_cycle_reasons);
	}
}

static void life_interface_test(struct drv_test_data *pdata)
{
	life_cycle_set_boot_reason(WARMBOOT_BY_SW);
	life_cycle_set_shutdown_reason(SHUTDOWN_BY_SW);
	life_cycle_set_thermal_shutdown_reason(THERMAL_SHUTDOWN_REASON_SOC);
	life_cycle_set_special_mode(LIFE_CYCLE_SMODE_LOW_BATTERY);

	read_all_life_reason(pdata);
}

static void sign_of_life_test(struct drv_test_data *pdata)
{
	int index = 0;
	int n = 0;
	u16 rtc_value = 0;

	if (!pdata) {
		pr_err("Test data is NULL. Error! \n");
		return;
	}

	switch (pdata->test_index) {
		case 1:
			/* Test: Test boot, shutdown, thermal shutdown, special mode interfaces */
			life_interface_test(pdata);
			return;
		case 2:
			/* Test: Warm Boot By Software */
			life_cycle_set_boot_reason(WARMBOOT_BY_SW);
			index = read_life_cycle_reason(BOOT);
			break;
		case 3:
			/* Test: Warm Boot By Kernel Panic */
			life_cycle_set_boot_reason(WARMBOOT_BY_KERNEL_PANIC);
			index = read_life_cycle_reason(BOOT);
			break;
		case 4:
			/* Test: Warm Boot By Kernel Watchdog */
			life_cycle_set_boot_reason(WARMBOOT_BY_KERNEL_WATCHDOG);
			index = read_life_cycle_reason(BOOT);
			break;
		case 5:
			/* Test: Warm Boot By HW Watchdog */
			life_cycle_set_boot_reason(WARMBOOT_BY_HW_WATCHDOG);
			index = read_life_cycle_reason(BOOT);
			break;
		case 6:
			/* Test: Cold Boot By Power Key */
			if (0 != clear_life_cycle_reason()) {
				n += scnprintf(pdata->result_buf+n, sizeof(pdata->result_buf)-n,
					"Clear life cycle reasons failed. \n");
			}

			lcr_boot = COLDBOOT_BY_POWER_KEY;

			index = read_life_cycle_reason(BOOT);
			break;
		case 7:
			/* Test: Cold Boot By USB */
			if (0 != clear_life_cycle_reason()) {
				n += scnprintf(pdata->result_buf+n, sizeof(pdata->result_buf)-n,
					"Clear life cycle reasons failed. \n");
			}

			lcr_boot = COLDBOOT_BY_USB;
			index = read_life_cycle_reason(BOOT);
			break;
		case 8:
			/* Test: Cold Boot By Power Supply */
			if (0 != clear_life_cycle_reason()) {
				n += scnprintf(pdata->result_buf+n, sizeof(pdata->result_buf)-n,
					"Clear life cycle reasons failed. \n");
			}

			lcr_boot = COLDBOOT_BY_POWER_SUPPLY;
			index = read_life_cycle_reason(BOOT);
			break;
		case 9:
			/* Test: Software Shutdown */
			life_cycle_set_shutdown_reason(SHUTDOWN_BY_SW);
			index = read_life_cycle_reason(SHUTDOWN);
			break;
		case 10:
			/* Test: Long Pressed Power Key Shutdown */
			life_cycle_set_shutdown_reason(SHUTDOWN_BY_LONG_PWR_KEY_PRESS);
			index = read_life_cycle_reason(SHUTDOWN);
			break;
		case 11:
			/* Test: Sudden Power Loss Shutdown */
			life_cycle_set_shutdown_reason(SHUTDOWN_BY_SUDDEN_POWER_LOSS);
			index = read_life_cycle_reason(SHUTDOWN);
			break;
		case 12:
			/* Test: Unknown Shutdown */
			life_cycle_set_shutdown_reason(SHUTDOWN_BY_UNKNOWN_REASONS);
			index = read_life_cycle_reason(SHUTDOWN);
			break;
		case 13:
			/* Test: PMIC Overheated Thermal Shutdown */
			life_cycle_set_thermal_shutdown_reason(THERMAL_SHUTDOWN_REASON_PMIC);
			index = read_life_cycle_reason(THERMAL_SHUTDOWN);
			break;
		case 14:
			/* Test: Battery Overheated Thermal Shutdown */
			life_cycle_set_thermal_shutdown_reason(THERMAL_SHUTDOWN_REASON_BATTERY);
			index = read_life_cycle_reason(THERMAL_SHUTDOWN);
			break;
		case 15:
			/* Test: SOC Overheated Thermal Shutdown */
			life_cycle_set_thermal_shutdown_reason(THERMAL_SHUTDOWN_REASON_SOC);
			index = read_life_cycle_reason(THERMAL_SHUTDOWN);
			break;
		case 16:
			/* Test: PCB Overheated Thermal Shutdown */
			life_cycle_set_thermal_shutdown_reason(THERMAL_SHUTDOWN_REASON_PCB);
			index = read_life_cycle_reason(THERMAL_SHUTDOWN);
			break;
		case 17:
			/* Test: WIFI Overheated Thermal Shutdown */
			life_cycle_set_thermal_shutdown_reason(THERMAL_SHUTDOWN_REASON_WIFI);
			index = read_life_cycle_reason(THERMAL_SHUTDOWN);
			break;
		case 18:
			/* Test: Modem Overheated Thermal Shutdown */
			life_cycle_set_thermal_shutdown_reason(THERMAL_SHUTDOWN_REASON_MODEM);
			index = read_life_cycle_reason(THERMAL_SHUTDOWN);
			break;
		case 19:
			/* Test: Low Battery Shutdown */
			life_cycle_set_special_mode(LIFE_CYCLE_SMODE_LOW_BATTERY);
			index = read_life_cycle_reason(SPECIAL);
			break;
		case 20:
			/* Test: Power Off Charging Mode */
			life_cycle_set_special_mode(LIFE_CYCLE_SMODE_WARM_BOOT_USB_CONNECTED);
			index = read_life_cycle_reason(SPECIAL);
			break;
		case 21:
			/* Test: Factory Reset Reboot */
			life_cycle_set_special_mode(LIFE_CYCLE_SMODE_FACTORY_RESET);
			index = read_life_cycle_reason(SPECIAL);
			break;
		case 22:
			/* Test: OTA Reboot */
			life_cycle_set_special_mode(LIFE_CYCLE_SMODE_OTA);
			index = read_life_cycle_reason(SPECIAL);
			break;
		case 23:
			/* Test: RTC Check Failed */
			life_cycle_set_special_mode(LIFE_CYCLE_SMODE_OTA);

			/*Write RTC and skip computing parity*/
			lcr_rtc_lock();
			rtc_value = lcr_rtc_read(SPECIAL_MODE_REASON_REG);

			rtc_value &= ~RTC_SPECIAL_MODE_MASK;
			rtc_value |= ((LIFE_CYCLE_SMODE_FACTORY_RESET << RTC_SPECIAL_MODE_SHIFT) & RTC_SPECIAL_MODE_MASK);

			lcr_rtc_write(SPECIAL_MODE_REASON_REG, rtc_value);
			lcr_rtc_write_trigger();
			lcr_rtc_unlock();

			index = read_life_cycle_reason(SPECIAL);
			break;
		case 24:
			/* Test: Print all Boot/Shutdown/Thermal/Special life cycle reason */
			read_all_life_reason(pdata);
			return;
		case 25:
			/* Test: Print Boot up life cycle reasons */
			bootup_life_reason(pdata);
			return;
		case 26:
			/* Test: Clear all life cycle reasons */
			if (0 == clear_life_cycle_reason()) {
				n += scnprintf(pdata->result_buf+n, sizeof(pdata->result_buf)-n,
					"Clear all life cycle reasons. \n");
			} else {
				n += scnprintf(pdata->result_buf+n, sizeof(pdata->result_buf)-n,
					"Clear life cycle reasons failed, please check the interface. \n");
			}
			return;
		default:
			index = -1;
			pr_err("There is no this test item, please check index. \n");

			n += scnprintf(pdata->result_buf+n, sizeof(pdata->result_buf)-n,
				"There is no this test item, please check index. \n");
			break;
	}

	if (index < 0 ) {
		pr_err("read life cycle reason Failed \n");
		n += scnprintf(pdata->result_buf+n, sizeof(pdata->result_buf)-n,
				"life cycle reason: Read Failed\n");
	} else {
		pr_info("life cycle reason is %s\n",
			lcr_data[index].life_cycle_reasons);

		n += scnprintf(pdata->result_buf+n, sizeof(pdata->result_buf)-n,"life cycle reason: %s\n",
					lcr_data[index].life_cycle_reasons);
	}
}

static unsigned int idme_interface_test(struct drv_test_data *pdata)
{
	const char *board_id = NULL;
	const char *bootmode = NULL;
	const char *dev_flags = NULL;
	int n = 0;

	board_id = idme_get_item("board_id");
	if (board_id == NULL) {
		pr_err("get idme data failed !\v");
		return -1;
	}

	bootmode = idme_get_item("bootmode");
	if (bootmode == NULL) {
		pr_err("get idme data failed !\v");
		return -1;
	}

	bootmode = idme_get_item("dev_flags");
	if (bootmode == NULL) {
		pr_err("get idme data failed !\v");
		return -1;
	}

	n += scnprintf(pdata->result_buf+n, sizeof(pdata->result_buf)-n,
			"board_id: %s\n", board_id);

	n += scnprintf(pdata->result_buf+n, sizeof(pdata->result_buf)-n,
			"bootmode: %s\n", bootmode);

	n += scnprintf(pdata->result_buf+n, sizeof(pdata->result_buf)-n,
			"dev_flags: %s\n", dev_flags);

	return 0;
}

static void idme_test(struct drv_test_data *pdata)
{
	int ret = 0, n = 0;

	if (!pdata) {
		pr_err("Test data is NULL. Error! \n");
		return;
	}

	switch (pdata->test_index) {
		case 1:
			/* Test: idme_get_item interface */
			ret = idme_interface_test(pdata);
			if (ret < 0) {
				pr_err("idme interface test failed! error = %d\n", ret);
				n += scnprintf(pdata->result_buf+n, sizeof(pdata->result_buf)-n,
					"idme interface test failed! error = %d\n", ret);
			}

			break;
		case 2:
			/* Test: idme_get_item interface */
			scnprintf(pdata->result_buf, sizeof(pdata->result_buf),
					"board_type: %d\n", idme_get_board_type());
			break;
		case 3:
			/* Test: idme_get_board_rev interface */
			scnprintf(pdata->result_buf, sizeof(pdata->result_buf),
					"board_ver: %d\n", idme_get_board_rev());
			break;
		case 4:
			/* Test: idme_get_bootmode interface */
			scnprintf(pdata->result_buf, sizeof(pdata->result_buf),
					"boot_mode: %d\n", idme_get_bootmode());
			break;
		case 5:
			/* Test: idme_get_dev_flags_value interface */
			scnprintf(pdata->result_buf, sizeof(pdata->result_buf),
					"dev_flags: %lld\n", idme_get_dev_flags_value());
			break;
		case 6:
			/* Test: board_has_wan interface */
			scnprintf(pdata->result_buf, sizeof(pdata->result_buf),
					"board_has_wan: %d\n", board_has_wan());
			break;
		default:
			pr_err("There is no this test item, please check index. \n");
			scnprintf(pdata->result_buf, sizeof(pdata->result_buf),
				"There is no this test item, please check index. \n");
			break;
	}
}

static int metrics_test(void)
{
	int i;
	char metrics_log[METRICS_STR_LEN];
	int ret = -1;

	memset(metrics_log, 0, METRICS_STR_LEN);
	snprintf(metrics_log, METRICS_STR_LEN, "TEST:METRIC:VALUE_A=%d;CT;1,VALUE_B=%d;CT;1:NR",
		 1, 0);

	for (i = 0; i < logger_loop; i++) {
		/* normal case */
		ret = log_to_metrics(ANDROID_LOG_INFO, "metrics_test", metrics_log);
		if (ret)
			pr_err("log_to_metrics: fails in normal case %d\n", ret);
		/* domain=NULL case */
		ret = log_to_metrics(ANDROID_LOG_INFO, NULL, metrics_log);
		if (ret)
			pr_err("log_to_metrics: fails in domain=NULL case %d\n", ret);
		/* msg=NULL case */
		ret = log_to_metrics(ANDROID_LOG_INFO, NULL, NULL);
		if (!ret)
			pr_err("log_to_metrics: fails msg=NULL case %d\n", ret);
	}
	memset(metrics_log, 0, METRICS_STR_LEN);

	/* special msg case, need log_to_metrics() to replace the space into "_" */
	snprintf(metrics_log, METRICS_STR_LEN, "T e s t :M E T R I C:V A L U E A=%d;CT;1,V A L U E B=%d;CT;1:NR",
		 1, 0);
	for (i = 0; i < logger_loop; i++) {
		/* normal case */
		ret = log_to_metrics(ANDROID_LOG_INFO, "metrics_test", metrics_log);
		if (ret)
			pr_err("log_to_metrics with space: fails in normal case %d\n", ret);
		/* domain=NULL case */
		ret = log_to_metrics(ANDROID_LOG_INFO, NULL, metrics_log);
		if (ret)
			pr_err("log_to_metrics with space: fails in domain=NULL case %d\n", ret);
		/* msg=NULL case */
		ret = log_to_metrics(ANDROID_LOG_INFO, NULL, NULL);
		if (!ret)
			pr_err("log_to_metrics with space: fails msg=NULL case%d\n", ret);
	}
	return 0;
}

static int vital_test(void)
{
	int i;
	struct timespec now;
	char metrics_log[METRICS_STR_LEN];
	int ret = -1;

	now = current_kernel_time();
	memset(metrics_log, 0, METRICS_STR_LEN);
	snprintf(metrics_log, METRICS_STR_LEN, "Kernel:TEST:VALUE_A=%d;CT;1,VALUE_B=%d;CT;1:NR",
		 1, 0);

	for (i = 0; i < logger_loop; i++) {
		/* key=NULL case */
		ret = log_timer_to_vitals(ANDROID_LOG_INFO, "Kernel timer", "TEST",
					  "time_in_test", NULL, now.tv_sec, NULL, VITALS_TIME_BUCKET);
		if (ret)
			pr_err("log_timer_to_vitals: fails in key=NULL case %d\n", ret);
		/* normal case */
		ret = log_timer_to_vitals(ANDROID_LOG_INFO, "Kernel timer", "TEST",
					  "time_in_test", "Test", now.tv_sec, "s", VITALS_TIME_BUCKET);
		if (ret)
			pr_err("log_timer_to_vitals: fails in normal case %d\n", ret);
	}

	for (i = 0; i < logger_loop; i++) {
		/* nomral case */
		ret = log_counter_to_vitals(ANDROID_LOG_INFO, "Kernel vitals", "TEST",
					    "VITALS", "test_value", (u32)1, "count", "s", VITALS_NORMAL);
		if (ret)
			pr_err("log_counter_to_vitals fails in nomral case %d\n", ret);
		/* metadata=NULL case */
		ret = log_counter_to_vitals(ANDROID_LOG_INFO, "Kernel vitals", "TEST",
					    "VITALS", "test_value", (u32)1, "count", NULL, VITALS_NORMAL);
		if (ret)
			pr_err("log_counter_to_vitals: fails in metadata=NULL %d\n", ret);
		/* key=NULL case */
		ret = log_counter_to_vitals(ANDROID_LOG_INFO, "Kernel vitals", "TEST", "VITALS",
					    NULL, (u32)1, "count", "s", VITALS_NORMAL);
		if (ret)
			pr_err("log_counter_to_vitals: fails in key=NULL case %d\n", ret);
		/* key=NULL, metadata=NULL case */
		ret = log_counter_to_vitals(ANDROID_LOG_INFO, "Kernel vitals", "TEST", "VITALS",
					    NULL, (u32)1, "count", NULL, VITALS_NORMAL);
		if (ret)
			pr_err("log_counter_to_vitals: fails in key=NULL,metadata=NULL case %d\n", ret);
	}
	return 0;
}

static void logger_test(struct drv_test_data *pdata)
{
	if (!pdata) {
		pr_err("Test data is NULL. Error! \n");
		return;
	}

	switch (pdata->test_index) {
		case 1:
			/* Test: Metrics and Vitals interfaces */
			metrics_test();
			vital_test();

			scnprintf(pdata->result_buf, sizeof(pdata->result_buf),
					"Run 'adb logcat -b metrics/vitals' to check.\n");
			break;
		case 2:
			/* Test: log_to_metrics interface */
			metrics_test();

			scnprintf(pdata->result_buf, sizeof(pdata->result_buf),
					"Run 'adb logcat -b metrics' to check.\n");
			break;
		case 3:
			/* Test: log_to_vitals interface */
			vital_test();

			scnprintf(pdata->result_buf, sizeof(pdata->result_buf),
					"Run 'adb logcat -b vitals' to check.\n");
			break;
		default:
			pr_err("There is no this test item, please check index. \n");
			scnprintf(pdata->result_buf, sizeof(pdata->result_buf),
				"There is no this test item, please check index. \n");
			break;
	}
}

static void show_help(struct seq_file *seq, struct drv_test_data *pdata)
{
	if (!strcmp(pdata->test_item, AMZN_SIGN_OF_LIFE)) {
		seq_printf(seq, "\nHelp Info:\n");
		seq_printf(seq, "echo [index] > sign_of_life to test below items,"
				"then cat sign_of_life to check result:\n\n");
		seq_printf(seq,	"0 ---  Show Help Info\n");
		seq_printf(seq,	"1 ---  Test Boot/Shutdown/Thermal/Special Interfaces\n");
		seq_printf(seq,	"2 ---  Warm Boot By Software\n");
		seq_printf(seq,	"3 ---  Warm Boot By Kernel Panic\n");
		seq_printf(seq, "4 ---  Warm Boot By Kernel Watchdog\n");
		seq_printf(seq,	"5 ---  Warm Boot By HW Watchdog\n");
		seq_printf(seq,	"6 ---  Cold Boot By Power Key\n");
		seq_printf(seq,	"7 ---  Cold Boot By USB\n");
		seq_printf(seq, "8 ---  Cold Boot By Power Supply\n");
		seq_printf(seq, "9 ---  Software Shutdown\n");
		seq_printf(seq, "10 --  Long Pressed Power Key Shutdown\n");
		seq_printf(seq, "11 --  Sudden Power Loss Shutdown\n");
		seq_printf(seq, "12 --  Unknown Shutdown\n");
		seq_printf(seq, "13 --  PMIC Overheated Thermal Shutdown\n");
		seq_printf(seq, "14 --  Battery Overheated Thermal Shutdown\n");
		seq_printf(seq, "15 --  SOC Overheated Thermal Shutdown\n");
		seq_printf(seq, "16 --  PCB Overheated Thermal Shutdown\n");
		seq_printf(seq, "17 --  WIFI Overheated Thermal Shutdown\n");
		seq_printf(seq, "18 --  Modem Overheated Thermal Shutdown\n");
		seq_printf(seq, "19 --  Special Mode: Low Battery Shutdown\n");
		seq_printf(seq, "20 --  Special Mode: Power Off Charging Mode\n");
		seq_printf(seq, "21 --  Special Mode: Factory Reset Reboot\n");
		seq_printf(seq, "22 --  Special Mode: OTA Reboot\n");
		seq_printf(seq, "23 --  Special Mode: RTC Check Failed\n");
		seq_printf(seq, "24 --  Print all Boot/Shutdown/Thermal/Special "
				"life cycle reason\n");
		seq_printf(seq, "25 --  Print Boot up life cycle reasons\n");
		seq_printf(seq, "26 --  Clear all life cycle reasons\n");
	}

	if (!strcmp(pdata->test_item, AMZN_IDME)) {
		seq_printf(seq, "\nHelp Info:\n");
		seq_printf(seq, "echo [index] > idme to test below items,"
				"then cat idme to check result:\n\n");
		seq_printf(seq,	"0 ---  Show Help Info\n");
		seq_printf(seq,	"1 ---  Test idme_get_item Interface\n");
		seq_printf(seq,	"2 ---  idme_get_board_type\n");
		seq_printf(seq,	"3 ---  idme_get_board_rev\n");
		seq_printf(seq, "4 ---  idme_get_bootmode\n");
		seq_printf(seq,	"5 ---  idme_get_dev_flags_value\n");
		seq_printf(seq,	"6 ---  board_has_wan\n");
	}

	if (!strcmp(pdata->test_item, AMZN_LOGGER)) {
		seq_printf(seq, "\nHelp Info:\n");
		seq_printf(seq, "echo [index] > logger to test below items,"
				"then cat logger to check result\n\n");
		seq_printf(seq,	"0 ---  Show Help Info\n");
		seq_printf(seq,	"1 ---  Test Metrics and Vitals interfaces\n");
		seq_printf(seq,	"2 ---  Test log_to_metrics interface\n");
		seq_printf(seq,	"3 ---  Test log_to_vitals interface\n");
	}
}

static void amzn_drv_test(struct drv_test_data *pdata)
{
	memset(pdata->result_buf, 0, sizeof(pdata->result_buf));

	if (!strcmp(pdata->test_item, AMZN_SIGN_OF_LIFE)) {
		sign_of_life_test(pdata);
	}

	if (!strcmp(pdata->test_item, AMZN_IDME)) {
		idme_test(pdata);
	}

	if (!strcmp(pdata->test_item, AMZN_LOGGER)) {
		logger_test(pdata);
	}
}

static int proc_show(struct seq_file *seq, void *v)
{
	struct drv_test_data *pdata = seq->private;

	if (!pdata) {
		pr_err("Test data is NULL. Error! \n");
		return -EINVAL;
	}

	down_read(&pdata->rwlock);
	seq_printf(seq, "%s=%d \n", pdata->test_item, pdata->test_index);

	if (0 == pdata->test_index)
		show_help(seq, pdata);
	else {
		seq_printf(seq, "Test Result: \n%s\n", pdata->result_buf);
	}
	up_read(&pdata->rwlock);

	return 0;
}

static int proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_show, PDE_DATA(inode));
}

static ssize_t proc_write(struct file *file, const char __user *buf,
				   size_t count, loff_t *ppos)
{
	struct drv_test_data *pdata = PDE_DATA(file_inode(file));
	int argc;
	char input[64];

	if (!pdata) {
		pr_err("Test data is NULL. Error! \n");
		return -EINVAL;
	}

	if (count >= sizeof(input))
		return -EINVAL;

	if (copy_from_user(input, buf, count))
		return -EFAULT;

	input[count] = '\0';

	down_write(&pdata->rwlock);

	argc = sscanf(input, "%d", &pdata->test_index);
	if (argc != 1) {
		up_write(&pdata->rwlock);
		return -EINVAL;
	}

	pr_info("test_index is %d\n", pdata->test_index);

	amzn_drv_test(pdata);

	up_write(&pdata->rwlock);

	return count;
}

static const struct file_operations test_fops = {
	.owner = THIS_MODULE,
	.open = proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.write = proc_write,
};

static int __init amzn_drv_test_init(void)
{
	struct proc_dir_entry *proc_amzn_drv = NULL;

	/* Create the root Amazon drivers procfs node */
	proc_amzn_drv = proc_mkdir(AMZN_DRIVERS, NULL);
	if (!proc_amzn_drv)
		return -ENOMEM;

	life_data = kzalloc(sizeof(struct drv_test_data), GFP_KERNEL);
	if (!life_data) {
		remove_proc_entry(AMZN_DRIVERS, NULL);
		return -ENOMEM;
	}

	strcpy(life_data->test_item, AMZN_SIGN_OF_LIFE);

	init_rwsem(&life_data->rwlock);

	proc_create_data(life_data->test_item, S_IRUGO|S_IWUSR, proc_amzn_drv,
		&test_fops, life_data);

	idme_data = kzalloc(sizeof(struct drv_test_data), GFP_KERNEL);
	if (!idme_data) {
		kfree(life_data);
		remove_proc_entry(AMZN_DRIVERS, NULL);
		return -ENOMEM;
	}

	strcpy(idme_data->test_item, AMZN_IDME);

	init_rwsem(&idme_data->rwlock);

	proc_create_data(idme_data->test_item, S_IRUGO|S_IWUSR, proc_amzn_drv,
		&test_fops, idme_data);

	logger_data = kzalloc(sizeof(struct drv_test_data), GFP_KERNEL);
	if (!logger_data) {
		kfree(idme_data);
		kfree(life_data);
		remove_proc_entry(AMZN_DRIVERS, NULL);
		return -ENOMEM;
	}

	strcpy(logger_data->test_item, AMZN_LOGGER);

	init_rwsem(&logger_data->rwlock);

	proc_create_data(logger_data->test_item, S_IRUGO|S_IWUSR, proc_amzn_drv,
		&test_fops, logger_data);

	return 0;
}

static void __exit amzn_drv_test_exit(void)
{
	remove_proc_entry(AMZN_DRIVERS, NULL);

	if (life_data) {
		kfree(life_data);
		life_data = NULL;
	}

	if (idme_data) {
		kfree(idme_data);
		idme_data = NULL;
	}

	if (logger_data) {
		kfree(logger_data);
		logger_data = NULL;
	}
}

module_init(amzn_drv_test_init);
module_exit(amzn_drv_test_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Amazon Common BSP Drvier Test");
