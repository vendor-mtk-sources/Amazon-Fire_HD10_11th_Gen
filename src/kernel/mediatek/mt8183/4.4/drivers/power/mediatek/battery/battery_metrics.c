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
#include <linux/power_supply.h>
#include <mt-plat/charger_type.h>
#include <mt-plat/mtk_battery.h>
#include "mtk_battery_internal.h"
#ifdef CONFIG_AMAZON_SIGN_OF_LIFE
#include <linux/sign_of_life.h>
#endif

#ifdef CONFIG_AMZN_SIGN_OF_LIFE
#include <linux/amzn_sign_of_life.h>
#endif

#ifdef CONFIG_FB
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

#ifdef CONFIG_AMAZON_METRICS_LOG
#include <linux/metricslog.h>
#endif

#ifdef CONFIG_AMZN_METRICS_LOG
#include <linux/amzn_metricslog.h>
#endif

#define BATTERY_METRICS_BUFF_SIZE 512
char g_metrics_buf[BATTERY_METRICS_BUFF_SIZE];

enum {
	SCREEN_OFF,
	SCREEN_ON,
};

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
	int screen_state;
};

struct pm_state {
	struct timespec suspend_ts;
	struct timespec resume_ts;
	int suspend_soc;
	int resume_soc;
	int suspend_bat_car;
	int resume_bat_car;
};

struct iavg_data {
	struct timespec last_ts;
	int pre_bat_car;
	int pre_screen_state;
};

struct bat_metrics_data {
	bool is_top_off_mode;
	u8 fault_type_old;
	u32 chg_sts_old;

	struct iavg_data iavg;
	struct screen_state screen;
	struct pm_state pm;
	struct delayed_work dwork;
#if defined(CONFIG_FB)
	struct notifier_block pm_notifier;
#endif
	struct notifier_block psy_nb;
	struct power_supply *bat_psy;
	struct delayed_work bat_psy_work;
};
static struct bat_metrics_data metrics_data;

int __attribute__ ((weak))
battery_report_uevent(void)
{
	pr_err("%s doesn't exist\n", __func__);
	return 0;
}

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
		"APPLE_1_0A_CHARGER", "APPLE_0_5A_CHARGER", "WIRELESS_CHARGER_5W",
		"WIRELESS_CHARGER_10W", "WIRELESS_CHARGER_15W",
	};

	if (chr_type > CHARGER_UNKNOWN && chr_type <= WIRELESS_CHARGER_15W) {
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
#if defined(CONFIG_AMAZON_SIGN_OF_LIFE) || defined(CONFIG_AMZN_SIGN_OF_LIFE)
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

int bat_metrics_aging(int aging_factor, int bat_cycle, int qmax)
{
	static struct timespec last_log_time;
	struct timespec now_time, diff;

	get_monotonic_boottime(&now_time);
	diff = timespec_sub(now_time, last_log_time);

	if (last_log_time.tv_sec != 0 && diff.tv_sec < 3600)
		return 0;

	pr_info("[%s]diff time:%ld\n", __func__, diff.tv_sec);
	bat_metrics_log("battery",
		"battery:def:aging_factor=%d;CT;1,bat_cycle=%d;CT;1,qmax=%d;CT;1:NA",
		aging_factor / 100, bat_cycle, qmax / 10);
	get_monotonic_boottime(&last_log_time);

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

	screen->screen_state = SCREEN_ON;
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

	screen->screen_state = SCREEN_OFF;
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

#define SUSPEND_RESUME_INTEVAL_MIN 30
int bat_metrics_suspend(void)
{
	struct pm_state *pm = &metrics_data.pm;

	get_monotonic_boottime(&pm->suspend_ts);
	pm->suspend_bat_car = gauge_get_coulomb();

	return 0;
}

int bat_metrics_resume(void)
{
	struct pm_state *pm = &metrics_data.pm;
	struct timespec resume_ts;
	struct timespec sleep_ts;
	int resume_bat_car, diff_bat_car, ibat_avg;

	get_monotonic_boottime(&resume_ts);
	sleep_ts = timespec_sub(resume_ts, pm->suspend_ts);
	if (sleep_ts.tv_sec < SUSPEND_RESUME_INTEVAL_MIN)
		goto exit;

	resume_bat_car = gauge_get_coulomb();
	diff_bat_car = resume_bat_car - pm->suspend_bat_car;
	ibat_avg = diff_bat_car * 3600 / sleep_ts.tv_sec / 10;
	pr_info("IBAT_AVG: sleep: diff_car[%d %d]=%d,time=%ld,ibat_avg=%d\n",
			pm->suspend_bat_car, resume_bat_car,
			diff_bat_car, sleep_ts.tv_sec, ibat_avg);

exit:
	pm->resume_bat_car = resume_bat_car;
	pm->resume_ts = resume_ts;
	pm->resume_bat_car = resume_bat_car;
	return 0;
}

static void bat_metrics_work(struct work_struct *work)
{
	struct bat_metrics_data *data =
		container_of(work, struct bat_metrics_data, dwork.work);
	struct iavg_data *iavg = &data->iavg;
	struct timespec ts_now, diff_ts;
	int bat_car, diff_bat_car, ibat_now, ibat_avg, sign;
	int screen_state = metrics_data.screen.screen_state;

	get_monotonic_boottime(&ts_now);
	bat_car = gauge_get_coulomb(); /* 0.1 mAh */
	if (iavg->pre_bat_car == -1)
		goto again;

	if (screen_state != iavg->pre_screen_state) {
		pr_info("%s: screen state change, drop data\n", __func__);
		goto again;
	}

	diff_ts = timespec_sub(ts_now, iavg->last_ts);
	ibat_now = battery_get_bat_current() / 10; /* mA */
	if (bat_car > iavg->pre_bat_car)
		sign = 1;
	else
		sign = -1;
	diff_bat_car = abs(iavg->pre_bat_car - bat_car); /* 0.1mAh */
	ibat_avg = sign * diff_bat_car * 3600 / diff_ts.tv_sec / 10; /* mA */

	pr_info("IBAT_AVG: %s: diff_car[%d %d]=%d,time=%ld,ibat=%d,ibat_avg=%d\n",
			screen_state == SCREEN_ON ? "screen_on" : "screen_off",
			iavg->pre_bat_car, bat_car, diff_bat_car,
			diff_ts.tv_sec, ibat_now, ibat_avg);

again:
	iavg->pre_bat_car = bat_car;
	iavg->last_ts = ts_now;
	iavg->pre_screen_state = screen_state;
	queue_delayed_work(system_freezable_wq, &data->dwork,
			msecs_to_jiffies(10000));
}

static void battery_psy_work(struct work_struct *work)
{
	struct power_supply *psy = metrics_data.bat_psy;
	union power_supply_propval val;
	static int last_capacity, last_status;
	int ret, capacity, status;

	if (!psy)
		goto exit;

	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_STATUS, &val);
	if (ret) {
		pr_err("%s: fail to get PROP_STATUS: %d \n", __func__, ret);
		goto exit;
	} else {
		status = val.intval;
	}

	ret = power_supply_get_property(psy,
			POWER_SUPPLY_PROP_CAPACITY, &val);
	if (ret) {
		pr_err("%s: fail to get PROP_CAPACITY: %d \n", __func__, ret);
		goto exit;
	} else {
		capacity = val.intval;
	}

	if (status != last_status || capacity != last_capacity)
		battery_report_uevent();

	last_capacity = capacity;
	last_status = status;
exit:
	return;
}

int battery_psy_event(struct notifier_block *nb, unsigned long event, void *v)
{
	struct power_supply *psy = v;

	if (unlikely(system_state < SYSTEM_RUNNING))
		return NOTIFY_OK;

	if (event == PSY_EVENT_PROP_CHANGED &&
		strcmp(psy->desc->name, "battery") == 0) {
		metrics_data.bat_psy = psy;
		schedule_delayed_work(&metrics_data.bat_psy_work, 0);
	}

	return NOTIFY_OK;
}


int bat_metrics_init(void)
{
	int ret = 0;

	metrics_data.screen.screen_on_soc = -1;
	metrics_data.screen.screen_off_soc = -1;
	metrics_data.pm.suspend_soc = -1;
	metrics_data.pm.resume_soc = -1;
	metrics_data.pm.suspend_bat_car = -1;
	metrics_data.pm.resume_bat_car = -1;
	metrics_data.iavg.pre_bat_car = -1;
	INIT_DELAYED_WORK(&metrics_data.dwork, bat_metrics_work);
	queue_delayed_work(system_freezable_wq, &metrics_data.dwork, 0);
#if defined(CONFIG_FB)
	metrics_data.pm_notifier.notifier_call = pm_notifier_callback;
	ret = fb_register_client(&metrics_data.pm_notifier);
	if (ret)
		pr_err("%s: fail to register pm notifier\n", __func__);
#endif
	INIT_DELAYED_WORK(&metrics_data.bat_psy_work, battery_psy_work);
	metrics_data.psy_nb.notifier_call = battery_psy_event;
	power_supply_reg_notifier(&metrics_data.psy_nb);
	return 0;
}

void bat_metrics_uninit(void)
{
#if defined(CONFIG_FB)
	fb_unregister_client(&metrics_data.pm_notifier);
#endif
	power_supply_unreg_notifier(&metrics_data.psy_nb);
}
