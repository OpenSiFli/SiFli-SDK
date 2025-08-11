/*
 *  Copyright 2023 NXP
 *
 *  SPDX-License-Identifier: BSD-3-Clause
 *
 */

#ifndef _OSA_THREADX_H_
#define _OSA_THREADX_H_

#include "tx_port.h"
#include "tx_user.h"
#include "tx_api.h"
#include "tx_thread.h"
#include "tx_semaphore.h"
#include "tx_event_flags.h"
#include "tx_mutex.h"
#include "tx_initialize.h"
#include "tx_uart.h"

#define CONFIG_MEM_POOLS 1

#include <mem_pool_config.h>

/*** Timer Management ***/

struct timer_data;

typedef struct timer_data *os_timer_t;
typedef os_timer_t osa_timer_arg_t;
typedef uint32_t osa_timer_tick;

/** OS Timer data structure
 */
struct timer_data
{
    void (*callback)(osa_timer_arg_t);
    void *user_arg;
    TX_TIMER timer;
};

/**
 * OS Timer Activate Options
 */
typedef enum osa_timer_activation
{
    /** Start the timer on creation. */
    OSA_TIMER_AUTO_ACTIVATE,
    /** Do not start the timer on creation. */
    OSA_TIMER_NO_ACTIVATE,
} osa_timer_activate_t;

/**
 * \def os_get_runtime_stats(__buff__)
 *
 * Get ASCII formatted run time statistics
 *
 * Please ensure that your buffer is big enough for the formatted data to
 * fit. Failing to do this may cause memory data corruption.
 */
#define OSA_GetRuntimeStats(__buff__)

/**
 * \def os_get_task_list(__buff__)
 *
 * Get ASCII formatted task list
 *
 * Please ensure that your buffer is big enough for the formatted data to
 * fit. Failing to do this may cause memory data corruption.
 */

#define OSA_GetTaskList(__buff__)

#endif /* ! _OSA_THREADX_H_ */
