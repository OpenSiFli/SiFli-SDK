/*
 * SPDX-FileCopyrightText: 2026-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef fmna_util_platform_h
#define fmna_util_platform_h

#define APP_TIMER_MODE_REPEATED              OS_TIMER_FLAG_PERIODIC
#define APP_TIMER_MODE_SINGLE_SHOT           OS_TIMER_FLAG_ONE_SHOT
#define APP_TIMER_MIN_TIMEOUT_TICKS          5

#ifndef MIN
/**
 * @brief Obtain the minimum of two values.
 *
 * @note Arguments are evaluated twice. Use Z_MIN for a GCC-only, single
 * evaluation version
 *
 * @param a First value.
 * @param b Second value.
 *
 * @returns Minimum value of @p a and @p b.
 */
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

uint8_t app_timer_create(ot_timer_id_t timer_id, uint8_t flag, os_timer_func_t func);

uint8_t app_timer_stop(ot_timer_id_t timer_id);

uint8_t app_timer_start(ot_timer_id_t timer_id, uint32_t duration, uint8_t reserved);

void app_sched_event_put(void *event_data, uint16_t event_data_size, void *handler);
#endif /* fmna_util_platform_h */
