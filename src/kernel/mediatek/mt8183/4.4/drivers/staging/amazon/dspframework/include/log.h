/*
 * log.h
 *
 * the LOG module for all DSP platforms, common code
 *
 * Copyright 2020-2021 Amazon Technologies, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#ifndef _ADF_LOG_H_
#define _ADF_LOG_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CFG_LOG_CONFIG
#define ADF_LOG_CONFIG          (0xFF)  /* default enable all logs */
#else
#define ADF_LOG_CONFIG          CFG_LOG_CONFIG
#endif

#ifndef CFG_LOG_COLOR_EN
#define ADF_ENABLE_LOG_COLOR    0
#else
#define ADF_ENABLE_LOG_COLOR    CFG_LOG_COLOR_EN
#endif

typedef int32_t (*adfLog_t)(const char *fmt, ...);

#define ADF_LOG_ERROR           (1<<0)
#define ADF_LOG_WARNING         (1<<1)
#define ADF_LOG_DEBUG           (1<<2)
#define ADF_LOG_INFO            (1<<3)
#define ADF_LOG_VERBOSE         (1<<4)

#define ADF_ENABLE_LOGE         (0 != (ADF_LOG_CONFIG & ADF_LOG_ERROR))
#define ADF_ENABLE_LOGW         (0 != (ADF_LOG_CONFIG & ADF_LOG_WARNING))
#define ADF_ENABLE_LOGD         (0 != (ADF_LOG_CONFIG & ADF_LOG_DEBUG))
#define ADF_ENABLE_LOGI         (0 != (ADF_LOG_CONFIG & ADF_LOG_INFO))
#define ADF_ENABLE_LOGV         (0 != (ADF_LOG_CONFIG & ADF_LOG_VERBOSE))
/* @} */

/**
 * @name ace audio log color
 * @brief ace audio log color pre define.
 * @{
 */
#define ADF_LOG_COLOR_RED       "31"
#define ADF_LOG_COLOR_GREEN     "32"
#define ADF_LOG_COLOR_YELLOW    "33"
#define ADF_LOG_COLOR_BLUE      "34"
#define ADF_LOG_COLOR_PURPLE    "35"
#define ADF_LOG_COLOR_CYAN      "36"
#define ADF_LOG_COLOR_WHITE     "37"

#if ADF_ENABLE_LOG_COLOR
#define ADF_LOG_RESET_COLOR     "\033[0m"
#define ADF_LOG_COLOR(COLOR)    "\033[0;" COLOR "m"
#else
#define ADF_LOG_RESET_COLOR
#define ADF_LOG_COLOR(COLOR)
#endif

#define ADF_LOG_E_COLOR         ADF_LOG_COLOR(ADF_LOG_COLOR_RED)
#define ADF_LOG_W_COLOR         ADF_LOG_COLOR(ADF_LOG_COLOR_YELLOW)
#define ADF_LOG_D_COLOR         ADF_LOG_COLOR(ADF_LOG_COLOR_CYAN)
#define ADF_LOG_I_COLOR         ADF_LOG_COLOR(ADF_LOG_COLOR_GREEN)
#define ADF_LOG_V_COLOR         ADF_LOG_COLOR(ADF_LOG_COLOR_PURPLE)
/* @} */

#if ADF_ENABLE_LOGE
#define ADF_LOG_E(tag, fmt, ...)  pr_err("<E>" ADF_LOG_E_COLOR"[" tag "] " \
										fmt ADF_LOG_RESET_COLOR, ##__VA_ARGS__)
#else
#define ADF_LOG_E(tag, fmt, ...)
#endif

#if ADF_ENABLE_LOGW
#define ADF_LOG_W(tag, fmt, ...)  pr_warning("<W>" ADF_LOG_W_COLOR "[" tag "] " \
										fmt ADF_LOG_RESET_COLOR, ##__VA_ARGS__)
#else
#define ADF_LOG_W(tag, fmt, ...)
#endif

#if ADF_ENABLE_LOGD
#define ADF_LOG_D(tag, fmt, ...)  pr_debug("<D>" ADF_LOG_D_COLOR "[" tag "] " \
										fmt ADF_LOG_RESET_COLOR, ##__VA_ARGS__)
#else
#define ADF_LOG_D(tag, fmt, ...)
#endif

#if ADF_ENABLE_LOGI
#define ADF_LOG_I(tag, fmt, ...)  pr_info("<I>" ADF_LOG_I_COLOR "[" tag "] " \
										fmt ADF_LOG_RESET_COLOR, ##__VA_ARGS__)
#else
#define ADF_LOG_I(tag, fmt, ...)
#endif

#if ADF_ENABLE_LOGV
#define ADF_LOG_V(tag, fmt, ...)  pr_debug("<V>" ADF_LOG_V_COLOR "[" tag "] " \
										fmt ADF_LOG_RESET_COLOR, ##__VA_ARGS__)
#else
#define ADF_LOG_V(tag, fmt, ...)
#endif

typedef enum {
	ADF_LOG_DUMP_RECENT = 0,
	ADF_LOG_DUMP_ALL  = 1,
} adfLogDumpMode_t;

int adfLog_dump(void *logAddr, struct seq_file *file);
int adfLog_print(void *logAddr);
void *adfLog_query(void *logAddr, void *logFliter);

#ifdef __cplusplus
}
#endif

#endif  /* _ADF_LOG_H_ */
