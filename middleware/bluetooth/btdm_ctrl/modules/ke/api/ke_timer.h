/**
 ****************************************************************************************
 *
 * @file ke_timer.h
 *
 * @brief This file contains the definitions used for timer management
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

#ifndef _KE_TIMER_H_
#define _KE_TIMER_H_

/**
 ****************************************************************************************
 * @defgroup TIMER BT Time
 * @ingroup KERNEL
 * @brief Timer management module.
 *
 * This module implements the functions used for managing kernel timers.
 *
 ****************************************************************************************
 */

#include "rwip.h"                 // RW definitions
#include "rwip_config.h"          // stack configuration
#include "ke_msg.h"               // messaging definition


/*
 * DEFINITIONS
 ****************************************************************************************
 */


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */


/*
 * FUNCTION PROTOTYPES
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief This function flushes all timers pending in the kernel.
 *
 ****************************************************************************************
 */
void _ke_timer_flush(void);

/**
 ****************************************************************************************
 * @brief Set a timer.
 *
 * The function first cancel the timer if it is already existing, then
 * it creates a new one. The timer can be one-shot or periodic, i.e. it
 * will be automatically set again after each trigger.
 *
 * When the timer expires, a message is sent to the task provided as
 * argument, with the timer id as message id.
 *
 *
 * @param[in] timer_id      Timer identifier (message identifier type).
 * @param[in] task_id       Task identifier which will be notified
 * @param[in] delay         Delay in time milliseconds.
 ****************************************************************************************
 */
void _ke_timer_set(ke_msg_id_t const timer_id, ke_task_id_t const task, uint32_t delay_ms);

/**
 ****************************************************************************************
 * @brief Remove an registered timer.
 *
 * This function search for the timer identified by its id and its task id.
 * If found it is stopped and freed, otherwise an error message is returned.
 *
 * @param[in] timer_id  Timer identifier.
 * @param[in] task      Task identifier.
 ****************************************************************************************
 */
void _ke_timer_clear(ke_msg_id_t const timerid, ke_task_id_t const task);

/**
 ****************************************************************************************
 * @brief Checks if a requested timer is active.
 *
 * This function pops the first timer from the timer queue and notifies the appropriate
 * task by sending a kernel message. If the timer is periodic, it is set again;
 * if it is one-shot, the timer is freed. The function checks also the next timers
 * and process them if they have expired or are about to expire.
 ****************************************************************************************
 */
bool _ke_timer_active(ke_msg_id_t const timer_id, ke_task_id_t const task_id);

typedef void (*ke_timer_flush_handler)(void);
typedef void (*ke_timer_set_handler)(ke_msg_id_t const timer_id, ke_task_id_t const task_id, uint32_t delay_ms);
typedef void (*ke_timer_clear_handler)(ke_msg_id_t const timer_id, ke_task_id_t const task_id);
typedef bool (*ke_timer_active_handler)(ke_msg_id_t const timer_id, ke_task_id_t const task_id);

extern ke_timer_flush_handler _ke_timer_flush_handler;
extern ke_timer_set_handler _ke_timer_set_handler;
extern ke_timer_clear_handler _ke_timer_clear_handler ;
extern ke_timer_active_handler _ke_timer_active_handler ;

#define ke_timer_flush()  _ke_timer_flush_handler()
#define ke_timer_set(timer_id, task, delay_ms)    _ke_timer_set_handler(timer_id, task, delay_ms)
#define ke_timer_clear(timer_id, task)  _ke_timer_clear_handler(timer_id, task)
#define ke_timer_active(timer_id, task_id) _ke_timer_active_handler(timer_id, task_id)

/// @} TIMER

#endif // _KE_TIMER_H_
