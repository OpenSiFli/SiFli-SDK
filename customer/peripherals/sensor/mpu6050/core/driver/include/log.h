/**
  ******************************************************************************
  * @file   log.h
  * @author Sifli software development team
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2019 - 2022,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef _LIBS_CUTILS_MPL_LOG_H
#define _LIBS_CUTILS_MPL_LOG_H

#include <stdlib.h>
#include <stdarg.h>

#ifdef ANDROID
    #ifdef NDK_BUILD
        #include "log_macros.h"
    #else
        #include <utils/Log.h>      /* For the LOG macro */
    #endif
#endif

#ifdef __KERNEL__
    #include <linux/kernel.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if defined ANDROID_JELLYBEAN
#define LOG ALOG
#define LOG_ERRROR ANDROID_LOG_ERROR
#endif

/* --------------------------------------------------------------------- */

/*
 * Normally we strip MPL_LOGV (VERBOSE messages) from release builds.
 * You can modify this (for example with "#define MPL_LOG_NDEBUG 0"
 * at the top of your source file) to change that behavior.
 */
#ifndef MPL_LOG_NDEBUG
#ifdef NDEBUG
#define MPL_LOG_NDEBUG 1
#else
#define MPL_LOG_NDEBUG 0
#endif
#endif

#ifdef __KERNEL__
#define MPL_LOG_UNKNOWN MPL_LOG_VERBOSE
#define MPL_LOG_DEFAULT KERN_DEFAULT
#define MPL_LOG_VERBOSE KERN_CONT
#define MPL_LOG_DEBUG   KERN_NOTICE
#define MPL_LOG_INFO    KERN_INFO
#define MPL_LOG_WARN    KERN_WARNING
#define MPL_LOG_ERROR   KERN_ERR
#define MPL_LOG_SILENT  MPL_LOG_VERBOSE

#else
/* Based off the log priorities in android
   /system/core/include/android/log.h */
#define MPL_LOG_UNKNOWN     (0)
#define MPL_LOG_DEFAULT     (1)
#define MPL_LOG_VERBOSE     (2)
#define MPL_LOG_DEBUG       (3)
#define MPL_LOG_INFO        (4)
#define MPL_LOG_WARN        (5)
#define MPL_LOG_ERROR       (6)
#define MPL_LOG_SILENT      (8)
#endif


/*
 * This is the local tag used for the following simplified
 * logging macros.  You can change this preprocessor definition
 * before using the other macros to change the tag.
 */
#ifndef MPL_LOG_TAG
#ifdef __KERNEL__
#define MPL_LOG_TAG
#else
#define MPL_LOG_TAG NULL
#endif
#endif

/* --------------------------------------------------------------------- */

/*
 * Simplified macro to send a verbose log message using the current MPL_LOG_TAG.
 */
#ifndef MPL_LOGV
#if MPL_LOG_NDEBUG
#ifdef _WIN32
#define MPL_LOGV(fmt, ...)                      \
    do {                                \
        __pragma (warning(suppress : 4127 )) \
        if (0)                          \
            MPL_LOG(LOG_VERBOSE, MPL_LOG_TAG, fmt, ##__VA_ARGS__);\
            __pragma (warning(suppress : 4127 )) \
    } while (0)
#else
#define MPL_LOGV(fmt, ...)                      \
    do {                                \
        if (0)                          \
            MPL_LOG(LOG_VERBOSE, MPL_LOG_TAG, fmt, ##__VA_ARGS__);\
    } while (0)
#endif

#else
#define MPL_LOGV(fmt, ...) MPL_LOG(LOG_VERBOSE, MPL_LOG_TAG, fmt, ##__VA_ARGS__)
#endif
#endif

#ifndef CONDITION
#define CONDITION(cond)     ((cond) != 0)
#endif

#ifndef MPL_LOGV_IF
#if MPL_LOG_NDEBUG
#define MPL_LOGV_IF(cond, fmt, ...)  \
    do { if (0) MPL_LOG(fmt, ##__VA_ARGS__); } while (0)
#else
#define MPL_LOGV_IF(cond, fmt, ...) \
    ((CONDITION(cond))                      \
        ? MPL_LOG(LOG_VERBOSE, MPL_LOG_TAG, fmt, ##__VA_ARGS__) \
        : (void)0)
#endif
#endif

/*
 * Simplified macro to send a debug log message using the current MPL_LOG_TAG.
 */
#ifndef MPL_LOGD
#define MPL_LOGD(fmt, ...) MPL_LOG(LOG_DEBUG, MPL_LOG_TAG, fmt, ##__VA_ARGS__)
#endif

#ifndef MPL_LOGD_IF
#define MPL_LOGD_IF(cond, fmt, ...) \
    ((CONDITION(cond))                         \
        ? MPL_LOG(LOG_DEBUG, MPL_LOG_TAG, fmt, ##__VA_ARGS__)  \
        : (void)0)
#endif

/*
 * Simplified macro to send an info log message using the current MPL_LOG_TAG.
 */
#ifndef MPL_LOGI
#ifdef __KERNEL__
#define MPL_LOGI(fmt, ...) pr_info(KERN_INFO MPL_LOG_TAG fmt, ##__VA_ARGS__)
#else
#define MPL_LOGI(fmt, ...) MPL_LOG(LOG_INFO, MPL_LOG_TAG, fmt, ##__VA_ARGS__)
#endif
#endif

#ifndef MPL_LOGI_IF
#define MPL_LOGI_IF(cond, fmt, ...) \
    ((CONDITION(cond))                                              \
        ? MPL_LOG(LOG_INFO, MPL_LOG_TAG, fmt, ##__VA_ARGS__)   \
        : (void)0)
#endif

/*
 * Simplified macro to send a warning log message using the current MPL_LOG_TAG.
 */
#ifndef MPL_LOGW
#ifdef __KERNEL__
#define MPL_LOGW(fmt, ...) printk(KERN_WARNING MPL_LOG_TAG fmt, ##__VA_ARGS__)
#else
#define MPL_LOGW(fmt, ...) MPL_LOG(LOG_WARN, MPL_LOG_TAG, fmt, ##__VA_ARGS__)
#endif
#endif

#ifndef MPL_LOGW_IF
#define MPL_LOGW_IF(cond, fmt, ...) \
    ((CONDITION(cond))                         \
        ? MPL_LOG(LOG_WARN, MPL_LOG_TAG, fmt, ##__VA_ARGS__)   \
        : (void)0)
#endif

/*
 * Simplified macro to send an error log message using the current MPL_LOG_TAG.
 */
#ifndef MPL_LOGE
#ifdef __KERNEL__
#define MPL_LOGE(fmt, ...) printk(KERN_ERR MPL_LOG_TAG fmt, ##__VA_ARGS__)
#else
#define MPL_LOGE(fmt, ...) MPL_LOG(LOG_ERROR, MPL_LOG_TAG, fmt, ##__VA_ARGS__)
#endif
#endif

#ifndef MPL_LOGE_IF
#define MPL_LOGE_IF(cond, fmt, ...) \
    ((CONDITION(cond))                         \
        ? MPL_LOG(LOG_ERROR, MPL_LOG_TAG, fmt, ##__VA_ARGS__)  \
        : (void)0)
#endif

/* --------------------------------------------------------------------- */

/*
 * Log a fatal error.  If the given condition fails, this stops program
 * execution like a normal assertion, but also generating the given message.
 * It is NOT stripped from release builds.  Note that the condition test
 * is -inverted- from the normal assert() semantics.
 */
#define MPL_LOG_ALWAYS_FATAL_IF(cond, fmt, ...) \
    ((CONDITION(cond))                     \
        ? ((void)android_printAssert(#cond, MPL_LOG_TAG,   \
                        fmt, ##__VA_ARGS__))    \
        : (void)0)

#define MPL_LOG_ALWAYS_FATAL(fmt, ...) \
    (((void)android_printAssert(NULL, MPL_LOG_TAG, fmt, ##__VA_ARGS__)))

/*
 * Versions of MPL_LOG_ALWAYS_FATAL_IF and MPL_LOG_ALWAYS_FATAL that
 * are stripped out of release builds.
 */
#if MPL_LOG_NDEBUG
#define MPL_LOG_FATAL_IF(cond, fmt, ...)                \
    do {                                \
        if (0)                          \
            MPL_LOG_ALWAYS_FATAL_IF(cond, fmt, ##__VA_ARGS__); \
    } while (0)
#define MPL_LOG_FATAL(fmt, ...)                     \
    do {                                \
        if (0)                          \
            MPL_LOG_ALWAYS_FATAL(fmt, ##__VA_ARGS__)    \
    } while (0)
#else
#define MPL_LOG_FATAL_IF(cond, fmt, ...) \
    MPL_LOG_ALWAYS_FATAL_IF(cond, fmt, ##__VA_ARGS__)
#define MPL_LOG_FATAL(fmt, ...) \
    MPL_LOG_ALWAYS_FATAL(fmt, ##__VA_ARGS__)
#endif

/*
 * Assertion that generates a log message when the assertion fails.
 * Stripped out of release builds.  Uses the current MPL_LOG_TAG.
 */
#define MPL_LOG_ASSERT(cond, fmt, ...)          \
    MPL_LOG_FATAL_IF(!(cond), fmt, ##__VA_ARGS__)

/* --------------------------------------------------------------------- */

/*
 * Basic log message macro.
 *
 * Example:
 *  MPL_LOG(MPL_LOG_WARN, NULL, "Failed with error %d", errno);
 *
 * The second argument may be NULL or "" to indicate the "global" tag.
 */
#ifndef MPL_LOG
#ifdef REMOVE_LOGGING
#define MPL_LOG(priority, tag, fmt, ...) do {} while (0)
#else
#define MPL_LOG(priority, tag, fmt, ...)        \
    MPL_LOG_PRI(priority, tag, fmt, ##__VA_ARGS__)
#endif
#endif

/*
 * Log macro that allows you to specify a number for the priority.
 */
#ifndef MPL_LOG_PRI
#ifdef ANDROID
#define MPL_LOG_PRI(priority, tag, fmt, ...) \
    LOG(priority, tag, fmt, ##__VA_ARGS__)
#elif defined __KERNEL__
#define MPL_LOG_PRI(priority, tag, fmt, ...) \
    pr_debug(MPL_##priority tag fmt, ##__VA_ARGS__)
#else
#define MPL_LOG_PRI(priority, tag, fmt, ...) \
    _MLPrintLog(MPL_##priority, tag, fmt, ##__VA_ARGS__)
#endif
#endif

/*
 * Log macro that allows you to pass in a varargs ("args" is a va_list).
 */
#ifndef MPL_LOG_PRI_VA
#ifdef ANDROID
#define MPL_LOG_PRI_VA(priority, tag, fmt, args) \
    android_vprintLog(priority, NULL, tag, fmt, args)
#elif defined __KERNEL__
/* not allowed in the Kernel because there is no dev_dbg that takes a va_list */
#else
#define MPL_LOG_PRI_VA(priority, tag, fmt, args) \
    _MLPrintVaLog(priority, NULL, tag, fmt, args)
#endif
#endif

/* --------------------------------------------------------------------- */

/*
 * ===========================================================================
 *
 * The stuff in the rest of this file should not be used directly.
 */

#ifndef ANDROID
int _MLPrintLog(int priority, const char *tag, const char *fmt, ...);
int _MLPrintVaLog(int priority, const char *tag, const char *fmt, va_list args);
/* Final implementation of actual writing to a character device */
int _MLWriteLog(const char *buf, int buflen);
#endif

static inline void __print_result_location(int result,
        const char *file,
        const char *func, int line)
{
    MPL_LOGE("%s|%s|%d returning %d\n", file, func, line, result);
}

#ifdef _WIN32
/* The pragma removes warning about expression being constant */
#define LOG_RESULT_LOCATION(condition) \
    do {                                \
        __print_result_location((int)(condition), __FILE__, \
                    __func__, __LINE__);        \
        __pragma (warning(suppress : 4127 )) \
    } while (0)
#else
#define LOG_RESULT_LOCATION(condition) \
    do {                                \
        __print_result_location((int)(condition), __FILE__, \
                    __func__, __LINE__);        \
    } while (0)
#endif


#define INV_ERROR_CHECK(r_1329) \
    if (r_1329) { \
        LOG_RESULT_LOCATION(r_1329); \
        return r_1329; \
    }

#ifdef __cplusplus
}
#endif
#endif              /* _LIBS_CUTILS_MPL_LOG_H *//************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
