/*
 * Copyright 2011-2020 Amazon Technologies, Inc. All Rights Reserved.
 * Portions Copyright (C) 2007-2008 Google, Inc.
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

#ifndef _LINUX_AMZN_METRICSLOG_H
#define _LINUX_AMZN_METRICSLOG_H

#ifdef CONFIG_AMZN_MINERVA_METRICS_LOG

/* Group ID */

/* group name: TabletBspVitalsPower */
#define VITALS_POWER_GROUP_ID "p4qtuscn"

/* group name: TabletBspMetricThermal */
#define METRICS_THERMAL_GROUP_ID "mpl090qb"

/* group name: TabletBspMetricInput */
#define METRICS_INPUT_GROUP_ID "mz6oxwtu"

/* group name: TabletBspMetricDSP */
#define METRICS_DSP_GROUP_ID "4fhznjtt"

/* group name: TabletBspMetricLCD */
#define METRICS_LCD_GROUP_ID "fjf2h0b5"

/* group name: TabletBspMetricLiquidDetection */
#define METRICS_LD_GROUP_ID "1q5jxqoz"

/* group name: TabletBspMetricBattery */
#define METRICS_BATTERY_GROUP_ID "qidponuq"

/* group name: TabletBspMetricHeadset */
#define METRICS_HEADSET_GROUP_ID "hnstckl9"

/* group name: TabletBspMetricPowerKey */
#define METRICS_PWRKEY_GROUP_ID "be682ppk"

/* group name: TabletBspVitalsDSP */
#define VITALS_DSP_GROUP_ID "bfhpkhn3"

/* group name: TabletBspVitalseMMc */
#define VITALS_EMMC_GROUP_ID "jwsz7rbz"

/* group name: TabletBSPVitalsThermal */
#define VITALS_THERMAL_GROUP_ID "jnxyeirr"

/* group name: TabletBspVitalsBattery */
#define VITALS_BATTERY_GROUP_ID "l1zgbpss"

/* Schema ID */

/* schema name: TabletBspVitalsPowerStatus */
#define VITALS_POWER_STATUS_SCHEMA_ID "ubch/2/03330400"

/* Schema name: TabletBspInputEvent */
#define METRICS_INPUT_SCHEMA_ID "03cp/2/02330400"

/* schema name: TabletBspMetricThermalCooler */
#define METRICS_THERMAL_COOLER_SCHEMA_ID "7h19/2/04330400"

/* schema name: TabletBspMetricVoiceDSP */
#define METRICS_DSP_VOICE_SCHEMA_ID "gyjx/2/07330400"

/* schema name: TabletBspMetricVoiceDSPCatchUp */
#define METRICS_DSP_CATCH_SCHEMA_ID "tnt6/2/06330400"

/* schema name: TabletBspMetricVoiceDSPFreqStat */
#define METRICS_DSP_FREQ_SCHEMA_ID "a67r/2/05330400"

/* schema name: TabletBspMetricLCDEvent */
#define METRICS_LCD_SCHEMA_ID "of7a/2/03330400"

/* schema name: TabletBspMetricLDEevent */
#define METRICS_LD_SCHEMA_ID "x9cc/2/06330400"

/* schema name: TabletBspMetricBatteryEvents */
#define METRICS_BATTERY_SCHEMA_ID "scot/2/0a330400"

/* schema name: TabletBspMetricBatteryAdapter*/
#define METRICS_BATTERY_ADAPTER_SCHEMA_ID "2ag5/2/02330400"

/* schema name: TabletBspMetricJackEvent */
#define METRICS_HEADSET_JACK_SCHEMA_ID "lc8i/2/04330400"

/* schema name: TabletBspMetricHeadsetKeyEvent */
#define METRICS_HEADSET_KEY_SCHEMA_ID "0e8o/2/05330400"

/* schema name: TabletBspMetricPwrkeyEvent */
#define METRICS_PWRKEY_SCHEMA_ID "ub2a/2/04330400"

/* schema name: TabletBspVitalsDSPCounter */
#define VITALS_DSP_COUNTER_SCHEMA_ID "l1o8/2/02330400"

/* schema name: TabletBspVitalsDSPTimer */
#define VITALS_DSP_TIMER_SCHEMA_ID "jd4a/2/03330400"

/* schema name: TabletBSPVitalsThermalThrottle */
#define VITALS_THERMAL_THROTTLE_SCHEMA_ID "pj83/2/03330400"

/* schema name: TabletBSPVitalsThermalSensor */
#define VITALS_THERMAL_SENSOR_SCHEMA_ID "hkqz/2/02330400"

/* schema name: TabletBspVitalseMMcCounter */
#define VITALS_EMMC_SCHEMA_ID "ubwk/2/02330400"

/* schema name: TabletBspVitalsBatteryDrain */
#define VITALS_BATTERY_DRAIN_SCHEMA_ID "5tyg/2/04330400"

/* schema name: TabletBspVitalsBatteryAging */
#define VITALS_BATTERY_AGING_SCHEMA_ID "f5fc/2/03330400"

/* predefined key */

/* required keys */
#define PREDEFINED_ESSENTIAL_KEY "_deviceType=;SY,_platform=;SY,_marketPlaceId=;SY,_countryOfResidence=;SY,_otaGroupName=;SY,_softwareVersion=;SY,_buildType=;SY,_hardware=;SY"

#define REQUIRED_VITALS_KEY "_osFileTag=;SY,_deviceId=;SY,_countryOfResidence=;SY,_deviceType=;SY,_marketPlaceId=;SY,_otaGroupName=;SY,_platform=;SY"

/* OS Key */
#define PREDEFINED_OS_KEY "_osFileTag=;SY"

/* Time Zone Key */
#define PREDEFINED_TZ_KEY "_timeZone=;SY"

/* Model Key */
#define PREDEFINED_MODEL_KEY "_model=;SY"

/* Device Id Key */
#define PREDEFINED_DEVICE_ID_KEY "_deviceId=;SY"

/* Customer Id Key */
#define PREDEFINED_CUSTOMER_ID_KEY "_customerId=;SY"

/* Device Language Key */
#define PREDEFINED_DEVICE_LANGUAGE_KEY "_deviceLanguage=;SY"

#endif /* CONFIG_AMZN_MINERVA_METRICS_LOG */

typedef enum {
	VITALS_NORMAL = 0,
	VITALS_FGTRACKING,
	VITALS_TIME_BUCKET,
} vitals_type;

/*
 * Android log priority values, in ascending priority order.
 */
typedef enum android_log_priority {
	ANDROID_LOG_UNKNOWN = 0,
	ANDROID_LOG_DEFAULT,	/* only for SetMinPriority() */
	ANDROID_LOG_VERBOSE,
	ANDROID_LOG_DEBUG,
	ANDROID_LOG_INFO,
	ANDROID_LOG_WARN,
	ANDROID_LOG_ERROR,
	ANDROID_LOG_FATAL,
	ANDROID_LOG_SILENT,	/* only for SetMinPriority(); must be last */
} android_LogPriority;

#ifdef CONFIG_AMZN_MINERVA_METRICS_LOG
int minerva_metrics_log(char *buf, int max_size, char *fmt, ...);

int minerva_counter_to_vitals(enum android_log_priority priority,
			const char *group_id, const char *schema_id,
			const char *domain, const char *program,
			const char *source, const char *key,
			long counter_value, const char *unit,
			const char *metadata, vitals_type type,
			const char *dimensions, const char *annotations);

int minerva_timer_to_vitals(enum android_log_priority priority,
			const char *group_id, const char *schema_id,
			const char *domain, const char *program,
			const char *source, const char *key,
			long timer_value, const char *unit, vitals_type type,
			const char *dimensions, const char *annotations);
#endif /* CONFIG_AMZN_MINERVA_METRICS_LOG */

#ifdef CONFIG_AMZN_METRICS_LOG
int log_to_metrics(enum android_log_priority priority,
		const char *domain, char *logmsg);

int log_counter_to_vitals(enum android_log_priority priority,
		const char *domain, const char *program,
		const char *source, const char *key,
		long counter_value, const char *unit,
		const char *metadata, vitals_type type);
int log_timer_to_vitals(enum android_log_priority priority,
		const char *domain, const char *program,
		const char *source, const char *key,
		long timer_value, const char *unit, vitals_type type);
#endif /* CONFIG_AMZN_METRICS_LOG */

#endif /* _LINUX_AMZN_METRICSLOG_H */
