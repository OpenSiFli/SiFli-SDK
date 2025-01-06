/**
 ****************************************************************************************
 *
 * @file ke_timer.c
 *
 * @brief This file contains the scheduler primitives called to create or delete
 * a task. It contains also the scheduler itself.
 *
 * Copyright (C) RivieraWaves 2009-2019
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup KE_TIMER
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stddef.h>              // standard definition
#include <stdint.h>              // standard integer
#include <stdbool.h>             // standard boolean
#include "arch.h"                // architecture

#include "ke_queue.h"            // kernel queue
#include "ke_mem.h"              // kernel memory
#include "ke_int.h"              // kernel environment
#include "ke_event.h"            // kernel event
#include "ke_timer.h"            // kernel timer
#include "ke_task.h"             // kernel task

#include "dbg_swdiag.h"          // Software diag

#include "co_time.h"             // common time module
#include "co_utils.h"            // stack common utility definitions

#include "dbg.h"
#include "btdm_patch.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/// Timer Object
typedef struct ke_timer
{
    /// List header
    co_list_hdr_t    hdr;
    /// time value
    co_timer_t  timer;
    /// message identifier
    ke_msg_id_t      id;
    /// task identifier
    ke_task_id_t     task;
} ke_timer_t;

typedef ke_timer_t *(*ke_timer_get_handler)(ke_msg_id_t const timer_id, ke_task_id_t const task_id);
typedef void (*ke_timer_expired_handler)(co_timer_t *p_timer);

extern ke_timer_t *_ke_timer_get(ke_msg_id_t const timer_id, ke_task_id_t const task_id);
__STATIC void _ke_timer_expired(co_timer_t *p_timer);

ke_timer_get_handler _ke_timer_get_handler = _ke_timer_get;
ke_timer_expired_handler _ke_timer_expired_handler = _ke_timer_expired;
ke_timer_flush_handler _ke_timer_flush_handler = _ke_timer_flush;
ke_timer_set_handler _ke_timer_set_handler = _ke_timer_set;
ke_timer_clear_handler _ke_timer_clear_handler = _ke_timer_clear;
ke_timer_active_handler _ke_timer_active_handler = _ke_timer_active;

#define ke_timer_get(timer_id, task_id)  _ke_timer_get_handler(timer_id, task_id)
#define ke_timer_expired(p_timer)  _ke_timer_expired_handler(p_timer)
//#define ke_timer_expired  _ke_timer_expired_handler

/**
 ****************************************************************************************
 * @brief retrieve a timer in the list of programmed timer
 *
 * @param[in] timer_id      Timer identifier (message identifier type).
 * @param[in] task_id       Task identifier which will be notified
 ****************************************************************************************
 */
ke_timer_t *_ke_timer_get(ke_msg_id_t const timer_id, ke_task_id_t const task_id)
{
    ke_timer_t *p_timer = (struct ke_timer *) co_list_pick(&(ke_env.queue_timer));

    // Browse list of timer to find expected one
    while (p_timer != NULL)
    {
        // if message ID and targeted task matches
        if ((p_timer->id == timer_id) && (p_timer->task == task_id))
        {
            // timer is found
            break;
        }

        // else Check next timer
        p_timer = (ke_timer_t *) p_timer->hdr.next;
    }

    return (p_timer);
}


/**
 ****************************************************************************************
 * @brief Function executed when a kernel timer has elapsed
 *
 * @param[in] p_timer Pointer to timer which has elapsed
 ****************************************************************************************
 */
__STATIC void _ke_timer_expired(co_timer_t *p_timer)
{
    ke_timer_t *p_ke_timer = CONTAINER_OF(p_timer, ke_timer_t, timer);
    // notify the task
    ke_msg_send_basic(p_ke_timer->id, p_ke_timer->task, TASK_NONE);

    // remove timer from kernel timer list
    co_list_extract(&(ke_env.queue_timer), &(p_ke_timer->hdr));

    // free the memory allocated for the timer
    ke_free(p_ke_timer);
}

/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

void _ke_timer_flush(void)
{
    // free all timers
    while (1)
    {
        struct ke_timer *timer = (struct ke_timer *) ke_queue_pop(&ke_env.queue_timer);
        if (timer == NULL)
            break;
        ke_free(timer);
    }
}

void _ke_timer_set(ke_msg_id_t const timer_id, ke_task_id_t const task_id, uint32_t delay_ms)
{
    // Get kernel timer
    ke_timer_t *p_timer = ke_timer_get(timer_id, task_id);

    if (p_timer == NULL)
    {
        // Create new one
        p_timer        = (ke_timer_t *) ke_malloc_system(sizeof(ke_timer_t), KE_MEM_KE_MSG);
        ASSERT_ERR_KE(p_timer);
        p_timer->id    = timer_id;
        p_timer->task  = task_id;
        // initialize timer
        co_timer_init(&(p_timer->timer), (co_timer_cb)_ke_timer_expired_handler);

        // put timer in list
        co_list_push_back(&(ke_env.queue_timer), &(p_timer->hdr));
    }

    // program timer
    co_timer_set(&(p_timer->timer), delay_ms * 10);
}

void _ke_timer_clear(ke_msg_id_t const timer_id, ke_task_id_t const task_id)
{
    // Get kernel timer
    ke_timer_t *p_timer = ke_timer_get(timer_id, task_id);

    if (p_timer != NULL)
    {
        // stop timer
        co_timer_stop(&(p_timer->timer));

        // remove timer from kernel timer list
        co_list_extract(&(ke_env.queue_timer), &(p_timer->hdr));

        // free the memory allocated for the timer
        ke_free(p_timer);
    }
}

bool _ke_timer_active(ke_msg_id_t const timer_id, ke_task_id_t const task_id)
{
    return (ke_timer_get(timer_id, task_id) != NULL);
}

///@} KE_TIMER
