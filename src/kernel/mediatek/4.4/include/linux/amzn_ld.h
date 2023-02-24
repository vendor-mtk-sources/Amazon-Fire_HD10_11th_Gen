#ifndef _AMZN_LD_H
#define _AMZN_LD_H

#ifdef CONFIG_AMAZON_METRICS_LOG
#include <linux/metricslog.h>
#endif


enum ld_state {
	DRY = 0,
	WET = 1,
};

enum ld_ctrl_mode {
	MODE_NONE = 0,
	MODE_DISABLE_USB = 1,
	MODE_SINK_ONLY = 2,
};

enum ld_threshold {
	THRESHOLD_L = 0,
	THRESHOLD_H = 1,
};

enum report_event {
	EVENT_DRY = 0,
	EVENT_WET = 1,
	EVENT_WET_VBUS = 2,
};

struct adc_data {
	int step1_mv;
	int step2_mv;
};

struct ld_data {
	struct device *dev;
	struct platform_device *pdev;
	struct notifier_block pm_notifier;
	struct delayed_work dwork;
	struct pinctrl *ld_pinctrl;
	struct power_supply *bat_psy;
	struct power_supply *usb_psy;
	struct pinctrl_state *ld_pin1_init;
	struct pinctrl_state *ld_pin1_low;
	struct pinctrl_state *ld_pin1_high;
	struct pinctrl_state *ld_pin2_init;
	struct pinctrl_state *ld_pin2_low;
	struct pinctrl_state *ld_pin2_high;
	struct pinctrl_state *buffer_ctrl_init;
	struct pinctrl_state *buffer_ctrl_low;
	struct pinctrl_state *buffer_ctrl_high;
	struct switch_dev ld_switch;
	struct tcpc_device *tcpc;
	struct adc_data adc_mv[2];

	/* For metrics */
	struct timespec event_ts;
	int ld_adc1;
	int ld_adc2;

	int adc_channel[2];
	int threshold[2];
	int ctrl_mode;
	int sleep_interval;
	int stop_detection;
	atomic_t is_suspend;
	int state;
};

#ifdef CONFIG_AMAZON_METRICS_LOG
#define BATTERY_METRICS_BUFF_SIZE 512
char g_m_buf[BATTERY_METRICS_BUFF_SIZE];

#define ld_metrics_log(domain, fmt, ...)			\
do {								\
	memset(g_m_buf, 0, BATTERY_METRICS_BUFF_SIZE);		\
	snprintf(g_m_buf, sizeof(g_m_buf), fmt, ##__VA_ARGS__);\
	log_to_metrics(ANDROID_LOG_INFO, domain, g_m_buf);	\
} while (0)
#else
static inline void ld_metrics_log(void) {}
#endif

#define TASK_DELAY_MSEC 15000
#define RECHECK_DELAY_MSEC 5000
#define GPIO_VOLT_SETUP_DELAY_MS 5
#define ADC_MAX_MV 1500
#define IUSB_LIMITATION_UA 0
#define INVALID_VBUS_UV 1000000
#define TASK_DELAY_WAKEUP_MSEC 5000

#endif
