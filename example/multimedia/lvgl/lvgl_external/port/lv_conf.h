/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * LVGL configuration for the external checkout.
 *
 * LVGL's own Kconfig symbols (LV_COLOR_FORMAT_*, LV_OS_*, ...) reuse the names
 * of LVGL C identifiers. The SiFli Kconfig tool writes symbols into rtconfig.h
 * without a CONFIG_ prefix, so sourcing the LVGL Kconfig tree would turn those
 * identifiers into macros. LVGL is therefore configured here, the upstream way.
 *
 * Project switches (EXAMPLE_LVGL_BENCHMARK, buffer sizes) still come from the
 * SiFli Kconfig tree.
 */

#ifndef LV_CONF_H
#define LV_CONF_H

#include "rtconfig.h"

/*====================
   COLOR SETTINGS
 *====================*/

#define LV_COLOR_FORMAT_DEFAULT LV_COLOR_FORMAT_RGB565

/*====================
   MEMORY SETTINGS
 *====================*/

#define LV_USE_STDLIB_MALLOC LV_STDLIB_RTTHREAD
/* This RT-Thread port does not provide rt_strcpy()/rt_vsnprintf() wrappers. */
#define LV_USE_STDLIB_STRING LV_STDLIB_CLIB
#define LV_USE_STDLIB_SPRINTF LV_STDLIB_CLIB

/* Draw buffers are handed to the LCD engine, which wants 64 byte alignment. */
#define LV_DRAW_BUF_ALIGN 64
#define LV_ATTRIBUTE_MEM_ALIGN __attribute__((aligned(64)))

/*====================
   HAL SETTINGS
 *====================*/

#define LV_USE_OS LV_OS_NONE
#define LV_RTTHREAD_INCLUDE <rtthread.h>
#define LV_DEF_REFR_PERIOD 16

/*====================
   LOGGING
 *====================*/

#define LV_USE_LOG 1
#define LV_LOG_LEVEL LV_LOG_LEVEL_WARN
#define LV_LOG_PRINTF 0

/*====================
   DRAW BACKENDS
 *====================*/

#define LV_USE_SIFLI_EPIC 1

/*====================
   FONTS
 *====================*/

#define LV_FONT_MONTSERRAT_14 1
#define LV_FONT_DEFAULT &lv_font_montserrat_14

#ifdef EXAMPLE_LVGL_BENCHMARK
#define LV_FONT_MONTSERRAT_16 1
#define LV_FONT_MONTSERRAT_20 1
#define LV_FONT_MONTSERRAT_24 1
#define LV_FONT_MONTSERRAT_26 1

#define LV_USE_SYSMON 1
#define LV_USE_PERF_MONITOR 1

#define LV_USE_DEMO_WIDGETS 1
#define LV_USE_DEMO_BENCHMARK 1
#endif

#endif /* LV_CONF_H */
