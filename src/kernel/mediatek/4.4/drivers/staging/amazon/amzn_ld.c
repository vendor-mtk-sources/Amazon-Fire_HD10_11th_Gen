#include <linux/init.h>
#include <linux/module.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/platform_device.h>
#include <linux/time.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/vmalloc.h>
#include <linux/power_supply.h>
#include <linux/notifier.h>
#include <linux/suspend.h>
#include <linux/switch.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/amzn_ld.h>
#include <tcpm.h>

static struct ld_data *g_ld;
static int g_adc1_mv, g_adc2_mv;
module_param(g_adc1_mv, int, 0664);
module_param(g_adc2_mv, int, 0664);

static inline bool is_in_range(int mv, int range_l, int range_h)
{
	return (mv >= range_l) && (mv <= range_h);
}

static bool ld_check_vbus_valid(struct ld_data *ld)
{
	union power_supply_propval val;
	int ret = 0;
	bool is_valid = false;

	if (!ld->usb_psy) {
		ld->usb_psy = power_supply_get_by_name("usb");
		if (!ld->usb_psy) {
			pr_err("%s: not find usb_psy\n", __func__);
			return false;
		}
	}

	ret = power_supply_get_property(ld->usb_psy,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
	if (ret < 0) {
		pr_err("%s: get voltage_now failed, ret = %d\n", __func__, ret);
		return false;
	}

	pr_info("%s: vbus: %d\n", __func__, val.intval);
	if (val.intval > INVALID_VBUS_UV)
		is_valid = true;

	return is_valid;
}


static void ld_report_event(struct ld_data *ld, int event)
{
	struct timespec now_ts;
	int duration_sec, adc1, adc2;
	int pre_event = switch_get_state(&ld->ld_switch);

	pr_info("%s: event change %d -> %d", __func__, pre_event, event);
	switch_set_state(&ld->ld_switch, event);

	if (pre_event != event) {
		get_monotonic_boottime(&now_ts);

		/*
		 * To avoid duration_sec number overflow on metrics service,
		 * "duration_sec" report 0 when event change from EVENT_DRY.
		 *
		 * adc1 and adc2 is only valid when event change from EVENT_DRY.
		 */
		if (pre_event == EVENT_DRY) {
			duration_sec = 0;
			adc1 = ld->ld_adc1;
			adc2 = ld->ld_adc2;
		} else {
			duration_sec = now_ts.tv_sec - ld->event_ts.tv_sec;
			adc1 = adc2 = 0;
		}

		ld_metrics_log("LiquidDetection",
		"LiquidDetection:def:ld_current_state=%d;CT;1,ld_previous_state=%d;CT;1,ld_duration_sec=%d;CT;1,ld_adc1=%d;CT;1,ld_adc2=%d;CT;1:NR",
				event, pre_event, duration_sec, adc1, adc2);
		memcpy(&ld->event_ts, &now_ts, sizeof(struct timespec));
	}
}

void ld_vbus_changed(void)
{
	int event;

	if (!g_ld)
		return;

	event = switch_get_state(&g_ld->ld_switch);
	if (ld_check_vbus_valid(g_ld)) {
		if (event == EVENT_WET) {
			ld_report_event(g_ld, EVENT_WET_VBUS);
			pr_info("%s: event changed: %d -> %d\n",
					__func__, EVENT_WET, EVENT_WET_VBUS);
		}
	} else {
		if (event == EVENT_WET_VBUS) {
			ld_report_event(g_ld, EVENT_WET);
			pr_info("%s: event changed: %d -> %d\n",
					__func__, EVENT_WET_VBUS, EVENT_WET);
		}
	}

	return;
}

static int ld_limit_charging_current(struct ld_data *ld, bool en)
{
	int ret = 0;
	union power_supply_propval propval;

	if (!ld->bat_psy) {
		ld->bat_psy = power_supply_get_by_name("battery");
		if (!ld->bat_psy) {
			pr_err("%s: not find bat_psy\n", __func__);
			return ret;
		}
	}

	propval.intval = en ? IUSB_LIMITATION_UA : -1;
	ret = power_supply_set_property(ld->bat_psy,
			POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, &propval);
	if (ret < 0)
		pr_err("%s: get psy online failed, ret = %d\n", __func__, ret);

	return ret;
}

static int ld_protection(struct ld_data *ld, bool en)
{
	if (ld->ctrl_mode == MODE_NONE)
		return 0;

	switch (ld->ctrl_mode) {
	case MODE_DISABLE_USB:
		if (en) {
			pr_info("%s: disable usb function\n", __func__);
			tcpm_typec_disable_function(ld->tcpc, true);
		} else {
			pr_info("%s: restore usb function\n", __func__);
			tcpm_typec_disable_function(ld->tcpc, false);
		}
		break;
	case MODE_SINK_ONLY:
		if (en) {
			pr_info("%s: force to Sink-Only\n", __func__);
			ld_limit_charging_current(ld, true);
			tcpm_typec_change_role(ld->tcpc, TYPEC_ROLE_SNK);
		} else {
			pr_info("%s: restore to DRP\n", __func__);
			ld_limit_charging_current(ld, false);
			tcpm_typec_change_role(ld->tcpc, TYPEC_ROLE_TRY_SNK);
		}
		break;
	default:
		pr_info("%s: invalid mode: %d\n", __func__, ld->ctrl_mode);
		break;
	}

	return 0;
}

extern int IMM_IsAdcInitReady(void);
extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int *rawdata);
static int ld_get_auxadc_data(int ch)
{
	int ret = 0, ret_temp = 0, data[4] = {0};
	int auxadc_cali_mv = 0;

	if (IMM_IsAdcInitReady() == 0) {
		pr_err("%s: AUXADC is not ready\n", __func__);
		return -1;
	}

	ret = IMM_GetOneChannelValue(ch, data, &ret_temp);
	if (ret) {
		pr_err("%s: AUXADC is busy\n", __func__);
		return -1;
	}

	/*
	 * by reference mtk_auxadc.c
	 *
	 * convert to volt:
	 *	data[0] = (rawdata * 1500 / (4096 + cali_ge)) / 1000;
	 *
	 * convert to mv, need multiply 10:
	 *	data[1] = (rawdata * 150 / (4096 + cali_ge)) % 100;
	 *
	 * provide high precision mv:
	 *	data[2] = (rawdata * 1500 / (4096 + cali_ge)) % 1000;
	 */
	pr_debug("%s: %d %d %d\n", __func__, data[0], data[1], data[2]);

	auxadc_cali_mv = data[0] * 1000 + data[2];
	pr_debug("%s: channel[%d] = %d mV\n", __func__, ch, auxadc_cali_mv);

	return auxadc_cali_mv;
}

static int ld_detection(struct ld_data *ld)
{
	int threshold_l, threshold_h;
	int adc1_mv, adc2_mv;
	int last_state = ld->state;
	int state;

	/* STEP1: get adc when GPIOs is LOW */
	memset(ld->adc_mv, 0, sizeof(ld->adc_mv));
	pinctrl_select_state(ld->ld_pinctrl, ld->ld_pin1_low);
	pinctrl_select_state(ld->ld_pinctrl, ld->ld_pin2_low);
	pinctrl_select_state(ld->ld_pinctrl, ld->buffer_ctrl_high);
	msleep(GPIO_VOLT_SETUP_DELAY_MS);
	adc1_mv = ld_get_auxadc_data(ld->adc_channel[0]);
	adc2_mv = ld_get_auxadc_data(ld->adc_channel[1]);
	g_adc1_mv = ld->adc_mv[0].step1_mv = adc1_mv;
	g_adc2_mv = ld->adc_mv[1].step1_mv = adc2_mv;
	pinctrl_select_state(ld->ld_pinctrl, ld->buffer_ctrl_low);
	pinctrl_select_state(ld->ld_pinctrl, ld->ld_pin1_low);
	pinctrl_select_state(ld->ld_pinctrl, ld->ld_pin2_low);

	threshold_l = ld->threshold[THRESHOLD_L];
	threshold_h = ld->threshold[THRESHOLD_H];

	if (last_state == WET) {
		/* Wet -> Dry */
		state = WET;
		if (is_in_range(adc1_mv, threshold_l, ADC_MAX_MV)
			|| is_in_range(adc2_mv, threshold_l, ADC_MAX_MV)) {
			pr_info("%s: Liquid still on SUB1 or SUB2\n", __func__);
			state = WET;
			goto out;
		}

		/* STEP2: get adc when GPIOs is HIGH */
		pinctrl_select_state(ld->ld_pinctrl, ld->ld_pin1_high);
		pinctrl_select_state(ld->ld_pinctrl, ld->ld_pin2_high);
		pinctrl_select_state(ld->ld_pinctrl, ld->buffer_ctrl_high);
		msleep(GPIO_VOLT_SETUP_DELAY_MS);
		adc1_mv = ld_get_auxadc_data(ld->adc_channel[0]);
		adc2_mv = ld_get_auxadc_data(ld->adc_channel[1]);
		ld->adc_mv[0].step2_mv = adc1_mv;
		ld->adc_mv[1].step2_mv = adc2_mv;
		g_adc1_mv = adc1_mv;
		g_adc2_mv = adc2_mv;
		pinctrl_select_state(ld->ld_pinctrl, ld->buffer_ctrl_low);
		pinctrl_select_state(ld->ld_pinctrl, ld->ld_pin1_low);
		pinctrl_select_state(ld->ld_pinctrl, ld->ld_pin2_low);

		if (is_in_range(adc1_mv, threshold_h, ADC_MAX_MV)
			&& is_in_range(adc2_mv, threshold_h, ADC_MAX_MV)) {
			pr_info("%s: liquid disappear\n", __func__);
			state = DRY;
			goto out;
		}
	} else {
		/* Dry -> Wet */
		state = DRY;
		if (is_in_range(adc1_mv, threshold_l, ADC_MAX_MV)) {
			pr_info("%s: ld on SUB1: %d\n", __func__, adc1_mv);
			state = WET;
			goto out;
		}

		if (is_in_range(adc2_mv, threshold_l, ADC_MAX_MV)) {
			pr_info("%s: ld on SUB2: %d\n", __func__, adc2_mv);
			state = WET;
			goto out;
		}
	}

out:
	return state;
}

static void ld_state_change(struct ld_data *ld, int state)
{
	int event;
	int pre_event = switch_get_state(&ld->ld_switch);

	if (state == WET) {
		if (ld_check_vbus_valid(ld))
			event = EVENT_WET_VBUS;
		else
			event = EVENT_WET;
	} else {
		event = EVENT_DRY;
	}

	pr_info("%s: event changed: %d -> %d\n", __func__, pre_event, event);
	ld_report_event(ld, event);
}

static void ld_routine_work(struct work_struct *work)
{
	struct ld_data *ld = container_of(work, struct ld_data, dwork.work);
	static const char * const state_text[] = {"DRY", "WET"};
	int sleep_interval = ld->sleep_interval;
	int state = 0;

	if (unlikely(system_state < SYSTEM_RUNNING)) {
		sleep_interval = RECHECK_DELAY_MSEC;
		goto skip;
	}

	if (ld->stop_detection)
		goto skip;

	state = ld_detection(ld);
	pr_info("%s: pre[%s] now[%s] STEP1[%d %d] STEP2[%d %d]\n",
			__func__, state_text[ld->state], state_text[state],
			ld->adc_mv[0].step1_mv, ld->adc_mv[1].step1_mv,
			ld->adc_mv[0].step2_mv, ld->adc_mv[1].step2_mv);
	if (state == ld->state)
		goto skip;

	pr_info("%s: state changed: %s -> %s\n",
		__func__, state_text[ld->state], state_text[state]);
	ld->state = state;
	if (state == WET) {
		ld->ld_adc1 = ld->adc_mv[0].step1_mv;
		ld->ld_adc2 = ld->adc_mv[1].step1_mv;
		ld_protection(ld, true);
		ld_state_change(ld, WET);
	} else {
		ld->ld_adc1 = ld->ld_adc2 = 0;
		ld_protection(ld, false);
		ld_state_change(ld, DRY);
	}

skip:
	if (atomic_read(&ld->is_suspend))
		pr_err("%s: skip next schedule\n", __func__);
	else
		schedule_delayed_work(&ld->dwork,
				msecs_to_jiffies(sleep_interval));
}

static int ld_pm_event(struct notifier_block *nb, unsigned long pm_event,
			void *unused)
{
	struct ld_data *ld = container_of(nb, struct ld_data, pm_notifier);

	switch (pm_event) {
	case PM_SUSPEND_PREPARE:
		pr_debug("[%s] cancel work\n", __func__);
		atomic_set(&ld->is_suspend, 1);
		cancel_delayed_work_sync(&ld->dwork);
		break;

	case PM_POST_SUSPEND:
		pr_debug("[%s] restore work\n", __func__);
		atomic_set(&ld->is_suspend, 0);
		schedule_delayed_work(&ld->dwork,
			msecs_to_jiffies(TASK_DELAY_WAKEUP_MSEC));
		break;
	}

	return NOTIFY_OK;
}

static inline int ld_parse_dt(struct platform_device *pdev)
{
	struct ld_data *ld = platform_get_drvdata(pdev);
	struct device_node *np = pdev->dev.of_node;
	int ret = 0, threshold[2] = {0}, adc_channel[2] = {0};

	ret = of_property_read_u32_array(np, "adc_channel", adc_channel,
				ARRAY_SIZE(adc_channel));
	if (ret < 0) {
		pr_err("%s: can't find adc_channel\n", __func__);
		goto out;
	}

	ret = of_property_read_u32_array(np, "threshold", threshold,
				ARRAY_SIZE(threshold));
	if (ret < 0) {
		pr_err("%s: can't find ld_threshold\n", __func__);
		goto out;
	}

	ret = of_property_read_u32(np, "ctrl_mode", &ld->ctrl_mode);
	if (ret < 0) {
		pr_err("%s: not define ctrl_mode, use MODE_DISABLE_USB\n",
			__func__);
		ld->ctrl_mode = MODE_DISABLE_USB;
	}

	ret = of_property_read_u32(np, "sleep_interval", &ld->sleep_interval);
	if (ret < 0) {
		pr_err("%s: not define sleep_interval, default 15s\n",
			__func__);
		ld->sleep_interval = TASK_DELAY_MSEC;
	}

	memcpy(&ld->threshold, threshold, sizeof(threshold));
	memcpy(&ld->adc_channel, adc_channel, sizeof(adc_channel));

	pr_info("%s: ADC[%d %d], threshold[%d %d] sleep_interval[%d] ctrl_mode[%d]\n",
			__func__,
			ld->adc_channel[0], ld->adc_channel[1],
			ld->threshold[0], ld->threshold[1],
			ld->sleep_interval, ld->ctrl_mode);
	return 0;
out:
	return ret;
}

static int ld_gpio_init(struct platform_device *pdev)
{
	struct ld_data *ld = platform_get_drvdata(pdev);
	struct pinctrl *pinctrl = NULL;

	pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(pinctrl)) {
		pr_err("%s: can't find pinctrl\n", __func__);
		goto out;
	}

	ld->ld_pinctrl = pinctrl;
	ld->ld_pin1_init = pinctrl_lookup_state(pinctrl, "ld1_init");
	if (IS_ERR(ld->ld_pin1_init)) {
		pr_err("%s: can't find ld1_init\n", __func__);
		goto out;
	}

	ld->ld_pin1_low = pinctrl_lookup_state(pinctrl, "ld1_low");
	if (IS_ERR(ld->ld_pin1_low)) {
		pr_err("%s: can't find ld1_low\n", __func__);
		goto out;
	}

	ld->ld_pin1_high = pinctrl_lookup_state(pinctrl, "ld1_high");
	if (IS_ERR(ld->ld_pin1_high)) {
		pr_err("%s: can't find ld1_high\n", __func__);
		goto out;
	}

	ld->ld_pin2_init = pinctrl_lookup_state(pinctrl, "ld2_init");
	if (IS_ERR(ld->ld_pin2_init)) {
		pr_err("%s: can't find ld2_init\n", __func__);
		goto out;
	}

	ld->ld_pin2_low = pinctrl_lookup_state(pinctrl, "ld2_low");
	if (IS_ERR(ld->ld_pin2_low)) {
		pr_err("%s: can't find ld2_low\n", __func__);
		goto out;
	}

	ld->ld_pin2_high = pinctrl_lookup_state(pinctrl, "ld2_high");
	if (IS_ERR(ld->ld_pin2_high)) {
		pr_err("%s: can't find ld2_high\n", __func__);
		goto out;
	}

	ld->buffer_ctrl_init = pinctrl_lookup_state(pinctrl, "bc_init");
	if (IS_ERR(ld->buffer_ctrl_init)) {
		pr_err("%s: can't find bc_init\n", __func__);
		goto out;
	}

	ld->buffer_ctrl_low = pinctrl_lookup_state(pinctrl, "bc_low");
	if (IS_ERR(ld->buffer_ctrl_low)) {
		pr_err("%s: can't find bc_low\n", __func__);
		goto out;
	}

	ld->buffer_ctrl_high = pinctrl_lookup_state(pinctrl, "bc_high");
	if (IS_ERR(ld->buffer_ctrl_high)) {
		pr_err("%s: can't find bc_high\n", __func__);
		goto out;
	}

	pinctrl_select_state(ld->ld_pinctrl, ld->ld_pin1_init);
	pinctrl_select_state(ld->ld_pinctrl, ld->ld_pin2_init);
	pinctrl_select_state(ld->ld_pinctrl, ld->buffer_ctrl_init);

	return 0;
out:
	return -1;
}

static ssize_t show_stop_detection(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ld_data *ld = dev->driver_data;

	return scnprintf(buf, PAGE_SIZE, "%d\n", ld->stop_detection);
}

static ssize_t store_stop_detection(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct ld_data *ld = dev->driver_data;
	int ret, en = 0;

	ret = kstrtouint(buf, 0, &en);
	if (en) {
		ld->stop_detection = 1;
		ld->state = DRY;
		switch_set_state(&ld->ld_switch, 0);
		tcpm_typec_disable_function(ld->tcpc, false);
		pr_info("%s: Stop detection, restore USB function\n", __func__);
	} else {
		ld->stop_detection = 0;
		pr_info("%s: start detection\n", __func__);
	}

	return size;
}
static DEVICE_ATTR(stop_detection, 0664,
			show_stop_detection, store_stop_detection);

static ssize_t show_ctrl_mode(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ld_data *ld = dev->driver_data;

	return scnprintf(buf, PAGE_SIZE, "%d\n", ld->ctrl_mode);
}

static ssize_t store_ctrl_mode(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct ld_data *ld = dev->driver_data;
	int ret, mode = 0;

	ret = kstrtouint(buf, 0, &mode);
	if (mode >= 0 && mode <= MODE_SINK_ONLY) {
		ld->ctrl_mode = mode;
		pr_info("%s: set ctrl_mode: %d\n", __func__, ld->ctrl_mode);
	}

	return size;
}
static DEVICE_ATTR(ctrl_mode, 0664, show_ctrl_mode, store_ctrl_mode);

static ssize_t show_threshold_l(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ld_data *ld = dev->driver_data;

	return scnprintf(buf, PAGE_SIZE, "%d\n", ld->threshold[THRESHOLD_L]);
}

static ssize_t store_threshold_l(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct ld_data *ld = dev->driver_data;
	int ret, threshold = 0;

	ret = kstrtouint(buf, 0, &threshold);
	if (threshold >= 0 && threshold <= ADC_MAX_MV) {
		ld->threshold[THRESHOLD_L] = threshold;
		pr_info("%s: set THRESHOLD_L: %d\n", __func__, threshold);
	}

	return size;
}
static DEVICE_ATTR(threshold_l, 0664, show_threshold_l, store_threshold_l);

static ssize_t show_threshold_h(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ld_data *ld = dev->driver_data;

	return scnprintf(buf, PAGE_SIZE, "%d\n", ld->threshold[THRESHOLD_H]);
}

static ssize_t store_threshold_h(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct ld_data *ld = dev->driver_data;
	int ret, threshold = 0;

	ret = kstrtouint(buf, 0, &threshold);
	if (threshold >= 0 && threshold <= ADC_MAX_MV) {
		ld->threshold[THRESHOLD_H] = threshold;
		pr_info("%s: set THRESHOLD_H: %d\n", __func__, threshold);
	}

	return size;
}
static DEVICE_ATTR(threshold_h, 0664, show_threshold_h, store_threshold_h);

static ssize_t show_sleep_interval(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ld_data *ld = dev->driver_data;

	return scnprintf(buf, PAGE_SIZE, "%d\n", ld->sleep_interval);
}

static ssize_t store_sleep_interval(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct ld_data *ld = dev->driver_data;
	int ret, sleep_ms = 0;

	ret = kstrtouint(buf, 0, &sleep_ms);
	if (sleep_ms >= 1000) {
		ld->sleep_interval = sleep_ms;
		pr_info("%s: set sleep_interval: %d\n", __func__, sleep_ms);
	} else {
		pr_info("%s: set sleep_interval > 1000\n", __func__);
	}

	return size;
}
static DEVICE_ATTR(sleep_interval, 0664,
		show_sleep_interval, store_sleep_interval);

static ssize_t adc1_1_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ld_data *ld = dev->driver_data;

	return scnprintf(buf, PAGE_SIZE, "%d\n", ld->adc_mv[0].step1_mv);
}
static const DEVICE_ATTR_RO(adc1_1);

static ssize_t adc2_1_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ld_data *ld = dev->driver_data;

	return scnprintf(buf, PAGE_SIZE, "%d\n", ld->adc_mv[1].step1_mv);
}
static const DEVICE_ATTR_RO(adc2_1);

static ssize_t adc1_2_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ld_data *ld = dev->driver_data;

	return scnprintf(buf, PAGE_SIZE, "%d\n", ld->adc_mv[0].step2_mv);
}
static const DEVICE_ATTR_RO(adc1_2);

static ssize_t adc2_2_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ld_data *ld = dev->driver_data;

	return scnprintf(buf, PAGE_SIZE, "%d\n", ld->adc_mv[1].step2_mv);
}
static const DEVICE_ATTR_RO(adc2_2);

static int ld_setup_files(struct platform_device *pdev)
{
	device_create_file(&(pdev->dev), &dev_attr_stop_detection);
	device_create_file(&(pdev->dev), &dev_attr_ctrl_mode);
	device_create_file(&(pdev->dev), &dev_attr_threshold_l);
	device_create_file(&(pdev->dev), &dev_attr_threshold_h);
	device_create_file(&(pdev->dev), &dev_attr_sleep_interval);
	device_create_file(&(pdev->dev), &dev_attr_adc1_1);
	device_create_file(&(pdev->dev), &dev_attr_adc2_1);
	device_create_file(&(pdev->dev), &dev_attr_adc1_2);
	device_create_file(&(pdev->dev), &dev_attr_adc2_2);

	return 0;
}

static int ld_probe(struct platform_device *pdev)
{
	struct ld_data *ld = NULL;
	int ret = -ENOMEM;

	ld = devm_kzalloc(&pdev->dev, sizeof(struct ld_data),
			    GFP_KERNEL);
	if (!ld) {
		ret = -ENOMEM;
		goto out;
	}

	platform_set_drvdata(pdev, ld);
	ld->pdev = pdev;

	ret = ld_parse_dt(pdev);
	if (ret)
		goto out_mem;

	ret = ld_gpio_init(pdev);
	if (ret)
		goto out_mem;

	ld->tcpc = tcpc_dev_get_by_name("type_c_port0");
	if (!ld->tcpc) {
		pr_err("%s: type_c_port0 is not ready yet\n", __func__);
		ret = -EPROBE_DEFER;
		goto out_mem;
	}

	ld->ld_switch.name = "ld";
	ld->ld_switch.index = 0;
	ld->ld_switch.state = 0;
	ret = switch_dev_register(&ld->ld_switch);
	if (ret) {
		pr_err("%s: switch_dev_register fail: %d\n", __func__, ret);
		goto out_mem;
	}

	atomic_set(&ld->is_suspend, 0);
	get_monotonic_boottime(&ld->event_ts);
	INIT_DELAYED_WORK(&ld->dwork, ld_routine_work);
	ld->pm_notifier.notifier_call = ld_pm_event;
	ret = register_pm_notifier(&ld->pm_notifier);
	if (ret) {
		pr_err("%s: failed to register PM notifier\n", __func__);
		goto out_switch;
	}

	ld_setup_files(pdev);
	schedule_delayed_work(&ld->dwork, 0);

	g_ld = ld;
	pr_info("%s: success\n", __func__);

	return 0;

out_switch:
	switch_dev_unregister(&ld->ld_switch);
out_mem:
	if (ld)
		devm_kfree(&pdev->dev, ld);
out:
	return ret;
}

static int ld_remove(struct platform_device *pdev)
{
	return 0;
}

static void ld_shutdown(struct platform_device *pdev)
{
	struct ld_data *ld = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&ld->dwork);
	unregister_pm_notifier(&ld->pm_notifier);
	switch_dev_unregister(&ld->ld_switch);
}

static const struct of_device_id ld_dt_match[] = {
	{.compatible = "amzn,ld",},
	{},
};
MODULE_DEVICE_TABLE(of, ld_dt_match);

struct platform_driver ld_driver = {
	.probe	= ld_probe,
	.remove = ld_remove,
	.shutdown = ld_shutdown,
	.driver = {
		.name = "ld",
		.owner = THIS_MODULE,
		.of_match_table = ld_dt_match,
	},
};

static int __init ld_init(void)
{
	return platform_driver_register(&ld_driver);
}
late_initcall(ld_init);

static void __exit ld_exit(void)
{
	platform_driver_unregister(&ld_driver);
}
module_exit(ld_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("liquid Detection Module Driver");
MODULE_AUTHOR("Amazon");
