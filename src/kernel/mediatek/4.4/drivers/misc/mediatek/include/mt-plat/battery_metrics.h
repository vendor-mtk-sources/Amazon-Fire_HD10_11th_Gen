#ifndef _BATTERY_METRICS_H
#define _BATTERY_METRICS_H

enum {
	METRICS_FAULT_NONE = 0,
	METRICS_FAULT_VBUS_OVP = 1,
	METRICS_FAULT_VBAT_OVP = 2,
	METRICS_FAULT_SAFETY_TIMEOUT = 3,
};

#if defined(CONFIG_AMAZON_METRICS_LOG)
int bat_metrics_chrdet(u32 chr_type);
int bat_metrics_chg_fault(u8 fault_type);
int bat_metrics_chg_state(u32 chg_sts);
int bat_metrics_critical_shutdown(void);
int bat_metrics_top_off_mode(bool is_on, long plugin_time);
int bat_metrics_init(void);
int bat_metrics_adapter_power(u32 type, u32 aicl_ma);
void bat_metrics_uninit(void);
#else
static inline void bat_metrics_chrdet(u32 chr_type) {}
static inline void bat_metrics_chg_fault(u8 fault_type) {}
static inline void bat_metrics_chg_state(u32 chg_sts) {}
static inline void bat_metrics_critical_shutdown(void) {}
static inline void bat_metrics_top_off_mode(bool is_on, long plugin_time) {}
static inline void bat_metrics_init(void) {}
static inline void bat_metrics_adapter_power(void) {}
static inline void bat_metrics_uninit(void) {}
#endif

#endif /* _BATTERY_METRICS_H */
