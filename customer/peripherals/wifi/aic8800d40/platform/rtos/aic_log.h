/*
 * Copyright (c) 2022, Artinchip Technology Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __AIC_LOG_H__
#define __AIC_LOG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>

#define AIC_LOG_ERR     3
#define AIC_LOG_WARN    4
#define AIC_LOG_INFO    6
#define AIC_LOG_DEBUG   7

/* Use the ULOG level as a global configuration */
#ifdef ULOG_OUTPUT_LVL
#define AIC_LOG_LEVEL   ULOG_OUTPUT_LVL
#endif

/* Must inlude aic_common.h before aic_log.h to avoid redefine */
#ifndef AIC_LOG_LEVEL
#define AIC_LOG_LEVEL   AIC_LOG_WARN
#endif

#if defined(KERNEL_RTTHREAD)
#include <rtdbg.h>
#define PRINTF_API      rt_kprintf
#elif defined(KERNEL_FREERTOS)
#define PRINTF_API      printf
#else
#define PRINTF_API      printf
#endif

#if defined(ULOG_USING_FILTER)
/* Support to change the log level in runtime */
#define AIC_LOG_RUN_LVL ulog_global_filter_lvl_get()
#else
/* Whether to show the log is decided in compile time */
#define AIC_LOG_RUN_LVL AIC_LOG_LEVEL
#endif

#define aic_log(level, tag, fmt, ...)                               \
    do {                                                            \
        if (level <= AIC_LOG_RUN_LVL)                               \
            PRINTF_API("[%s] %s()%d " fmt, tag, __func__, __LINE__, \
                       ##__VA_ARGS__);                              \
    } while (0)

#define hal_log_err(fmt, ...)   aic_log(AIC_LOG_ERR, "E", fmt, ##__VA_ARGS__)
#define hal_log_warn(fmt, ...)  aic_log(AIC_LOG_WARN, "W", fmt, ##__VA_ARGS__)
#define hal_log_info(fmt, ...)  aic_log(AIC_LOG_INFO, "I", fmt, ##__VA_ARGS__)

/* The 'debug' level log should not be compiled in release version */
#if (AIC_LOG_LEVEL < AIC_LOG_DEBUG)
#define hal_log_debug(fmt, ...) \
    do {                        \
    } while (0)
#else
#define hal_log_debug(fmt, ...) aic_log(AIC_LOG_DEBUG, "D", fmt, ##__VA_ARGS__)
#endif

#ifndef pr_fmt
#define pr_fmt(fmt) fmt
#endif

#define pr_err(fmt, ...)        hal_log_err(pr_fmt(fmt), ##__VA_ARGS__)
#define pr_warn(fmt, ...)       hal_log_warn(pr_fmt(fmt), ##__VA_ARGS__)
#define pr_info(fmt, ...)       hal_log_info(pr_fmt(fmt), ##__VA_ARGS__)
#define pr_debug(fmt, ...)      hal_log_debug(pr_fmt(fmt), ##__VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif
