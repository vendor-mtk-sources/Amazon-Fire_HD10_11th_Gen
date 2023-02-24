#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/metricslog.h>
#include <linux/power_supply.h>
#include <mt-plat/charger_type.h>
#include <mt-plat/mtk_battery.h>
#include "mtk_battery_internal.h"
#ifdef CONFIG_AMAZON_SIGN_OF_LIFE
#include <linux/sign_of_life.h>
#endif
#ifdef CONFIG_FB
#include <linux/notifier.h>
#include <linux/fb.h>
#endif


#define BATTERY_METRICS_BUFF_SIZE 512
char g_metrics_buf[BATTERY_METRICS_BUFF_SIZE];

#define bat_metrics_log(domain, fmt, ...)				\
do {									\
	memset(g_metrics_buf, 0, BATTERY_METRICS_BUFF_SIZE);		\
	snprintf(g_metrics_buf, sizeof(g_metrics_buf),	fmt, ##__VA_ARGS__);\
	log_to_metrics(ANDROID_LOG_INFO, domain, g_metrics_buf);	\
} while (0)

struct screen_state {
	struct timespec screen_on_time;
	struct timespec screen_off_time;
	int screen_on_soc;
	int screen_off_soc;
};

struct bat_metrics_data {
	bool is_top_off_mode;
	u8 fault_type_old;
	u32 chg_sts_old;

	struct screen_state screen;
#if defined(CONFIG_FB)
	struct notifier_block pm_notifier;
#endif
};
static struct bat_metrics_data metrics_data;

#define ADAPTER_POWER_CATEGORY_SIZE	5
int bat_metrics_adapter_power(u32 type, u32 aicl_ma)
{
	static const char * const category_text[ADAPTER_POWER_CATEGORY_SIZE] = {
		"5W", "7.5W", "9W", "12W", "15W"
	};

	if (type >= ADAPTER_POWER_CATEGORY_SIZE)
		return 0;

	bat_metrics_log("battery",
			"adapter_power:def:adapter_%s=1;CT;1,aicl=%d;CT;1:NR",
			category_text[type], (aicl_ma < 0) ? 0 : aicl_ma);

	return 0;
}

int bat_metrics_chg_fault(u8 fault_type)
{
	static const char * const charger_fault_text[] = {
		"NONE", "VBUS_OVP", "VBAT_OVP", "SAFETY_TIMEOUT"
	};

	if (metrics_data.fault_type_old == fault_type)
		return 0;

	metrics_data.fault_type_old = fault_type;
	if (fault_type != 0)
		bat_metrics_log("battery",
			"charger:def:charger_fault_type_%s=1;CT;1:NA",
			charger_fault_text[fault_type]);

	return 0;
}

int bat_metrics_chrdet(u32 chr_type)
{
	static const char * const charger_type_text[] = {
		"UNKNOWN", "STANDARD_HOST", "CHARGING_HOST",
		"NONSTANDARD_CHARGER", "STANDARD_CHARGER", "APPLE_2_1A_CHARGER",
		"APPLE_1_0A_CHARGER", "APPLE_0_5A_CHARGER", "WIRELESS_CHARGER"
	};

	if (chr_type > CHARGER_UNKNOWN && chr_type <= WIRELESS_CHARGER) {
		bat_metrics_log("USBCableEvent",
			"%s:charger:chg_type_%s=1;CT;1:NR",
			__func__, charger_type_text[chr_type]);
	}

	return 0;
}

int bat_metrics_chg_state(u32 chg_sts)
{
	int soc, vbat, ibat_avg;

	if (metrics_data.chg_sts_old == chg_sts)
		return 0;

	soc = battery_get_soc();
	vbat = battery_get_bat_voltage();
	ibat_avg = battery_get_bat_avg_current() / 10;
	metrics_data.chg_sts_old = chg_sts;
	bat_metrics_log("battery",
		"charger:def:POWER_STATUS_%s=1;CT;1,cap=%u;CT;1,mv=%d;CT;1,current_avg=%d;CT;1:NR",
		(chg_sts == POWER_SUPPLY_STATUS_CHARGING) ?
		"CHARGING" : "DISCHARGING", soc, vbat, ibat_avg);

	return 0;
}

int bat_metrics_critical_shutdown(void)
{
	bat_metrics_log("battery", "battery:def:critical_shutdown=1;CT;1:HI");
#ifdef CONFIG_AMAZON_SIGN_OF_LIFE
	life_cycle_set_special_mode(LIFE_CYCLE_SMODE_LOW_BATTERY);
#endif

	return 0;
}

int bat_metrics_top_off_mode(bool is_on, long plugin_time)
{
	if (metrics_data.is_top_off_mode == is_on)
		return 0;

	metrics_data.is_top_off_mode = is_on;
	if (is_on) {
		bat_metrics_log("battery",
		"battery:def:Charging_Over_7days=1;CT;1,Total_Plug_Time=%ld;CT;1,Bat_Vol=%d;CT;1,UI_SOC=%d;CT;1,SOC=%d;CT;1,Bat_Temp=%d;CT;1:NA",
		plugin_time, battery_get_bat_voltage(), battery_get_uisoc(),
		battery_get_soc(), battery_get_bat_temperature());
	}

	return 0;
}

#if defined(CONFIG_FB)
static int bat_metrics_screen_on(void)
{
	struct timespec screen_on_time;
	struct timespec diff;
	struct screen_state *screen = &metrics_data.screen;
	long elaps_msec;
	int soc, diff_soc;

	soc = battery_get_soc();
	get_monotonic_boottime(&screen_on_time);
	if (screen->screen_on_soc == -1 || screen->screen_off_soc == -1)
		goto exit;

	diff_soc = screen->screen_off_soc - soc;
	diff = timespec_sub(screen_on_time, screen->screen_off_time);
	elaps_msec = diff.tv_sec * 1000 + diff.tv_nsec / NSEC_PER_MSEC;
	pr_info("%s: diff_soc: %d[%d -> %d] elapsed=%ld\n", __func__,
		diff_soc, screen->screen_off_soc, soc, elaps_msec);
	bat_metrics_log("drain_metrics",
		"screen_off_drain:def:value=%d;CT;1,elapsed=%ld;TI;1:NR",
		diff_soc, elaps_msec);

exit:
	screen->screen_on_time = screen_on_time;
	screen->screen_on_soc = soc;
	return 0;
}

static int bat_metrics_screen_off(void)
{
	struct timespec screen_off_time;
	struct timespec diff;
	struct screen_state *screen = &metrics_data.screen;
	long elaps_msec;
	int soc, diff_soc;

	soc = battery_get_soc();
	get_monotonic_boottime(&screen_off_time);
	if (screen->screen_on_soc == -1 || screen->screen_off_soc == -1)
		goto exit;

	diff_soc = screen->screen_on_soc - soc;
	diff = timespec_sub(screen_off_time, screen->screen_on_time);
	elaps_msec = diff.tv_sec * 1000 + diff.tv_nsec / NSEC_PER_MSEC;
	pr_info("%s: diff_soc: %d[%d -> %d] elapsed=%ld\n", __func__,
		diff_soc, screen->screen_on_soc, soc, elaps_msec);
	bat_metrics_log("drain_metrics",
		"screen_on_drain:def:value=%d;CT;1,elapsed=%ld;TI;1:NR",
		diff_soc, elaps_msec);

exit:
	screen->screen_off_time = screen_off_time;
	screen->screen_off_soc = soc;
	return 0;
}

static int pm_notifier_callback(struct notifier_block *notify,
			unsigned long event, void *data)
{
	struct fb_event *ev_data = data;
	int *blank;

	if (ev_data && ev_data->data && event == FB_EVENT_BLANK) {
		blank = ev_data->data;
		if (*blank == FB_BLANK_UNBLANK)
			bat_metrics_screen_on();

		else if (*blank == FB_BLANK_POWERDOWN)
			bat_metrics_screen_off();
	}

	return 0;
}
#endif

int bat_metrics_init(void)
{
	int ret = 0;

	metrics_data.screen.screen_off_soc = -1;
	metrics_data.screen.screen_off_soc = -1;
#if defined(CONFIG_FB)
	metrics_data.pm_notifier.notifier_call = pm_notifier_callback;
	ret = fb_register_client(&metrics_data.pm_notifier);
	if (ret)
		pr_err("%s: fail to register pm notifier\n", __func__);
#endif

	return 0;
}

void bat_metrics_uninit(void)
{
#if defined(CONFIG_FB)
	fb_unregister_client(&metrics_data.pm_notifier);
#endif
}
