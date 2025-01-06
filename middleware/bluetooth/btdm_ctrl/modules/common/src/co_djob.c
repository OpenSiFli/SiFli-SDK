/**
 ****************************************************************************************
 * @file co_djob.c
 *
 * @brief Common Delayed Job Module
 *
 * Copyright (C) RivieraWaves 2017-2019
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup CO_DJOB
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "co_djob.h"      // Delayed job definition
#include "arch.h"         // for defines
#include "ke_event.h"     // Kernel Event Definitions
#include "arch.h"         // architecture
#include "co_list.h"      // list usage
#include "rwip.h"         // reset state


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Delayed job environment structure
typedef struct co_djob_env_
{
    /// Job queue to execute - Low / High Priority
    co_list_t job_queue[CO_DJOB_PRIO_NB];
    /// Job queue to execute - ISR Priority
    co_list_t isr_job_queue;
} co_djob_env_t;


/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */
/// Delayed job environment
__STATIC co_djob_env_t co_djob_env;

/// Event code
__STATIC const uint8_t co_djob_evt[CO_DJOB_PRIO_NB] =
{
    [CO_DJOB_LOW]  = KE_EVENT_DJOB_LP,
    [CO_DJOB_HIGH] = KE_EVENT_DJOB_HP,
};

/*
 * LOCAL FUNCTIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handler of common event to immediately start a delayed job - Low Priority
 ****************************************************************************************
 */
__STATIC void co_djob_lp_evt_handler(void)
{
    DBG_FUNC_ENTER(co_djob_evt_handler);
    co_djob_t *p_djob;
    co_list_t *p_job_queue = &(co_djob_env.job_queue[CO_DJOB_LOW]);

    p_djob = (co_djob_t *)co_list_pop_front(p_job_queue);
    // Check if another job must be executed
    if (co_list_is_empty(p_job_queue))
    {
        // Clear event
        ke_event_clear(KE_EVENT_DJOB_LP);
    }

    if (p_djob != NULL)
    {
        p_djob->hdr.next = NULL;
        p_djob->cb(p_djob);
    }

    DBG_FUNC_EXIT(co_djob_evt_handler);
}

/**
 ****************************************************************************************
 * @brief Handler of common event to immediately start a delayed job - High Priority
 ****************************************************************************************
 */
__STATIC void co_djob_hp_evt_handler(void)
{
    DBG_FUNC_ENTER(co_djob_hp_evt_handler);
    co_djob_t *p_djob;
    co_list_t *p_job_queue = &(co_djob_env.job_queue[CO_DJOB_HIGH]);

    p_djob = (co_djob_t *)co_list_pop_front(p_job_queue);
    // Check if another job must be executed
    if (co_list_is_empty(p_job_queue))
    {
        // Clear event
        ke_event_clear(KE_EVENT_DJOB_HP);
    }

    if (p_djob != NULL)
    {
        p_djob->hdr.next = NULL;
        p_djob->cb(p_djob);
    }
    DBG_FUNC_EXIT(co_djob_hp_evt_handler);
}

/**
 ****************************************************************************************
 * @brief Handler of common event to immediately start a delayed job - ISR priority
 ****************************************************************************************
 */
__STATIC void co_djob_isr_evt_handler(void)
{
    DBG_FUNC_ENTER(co_djob_isr_evt_handler);
    co_djob_t *p_djob;
    co_list_t *p_job_queue = &(co_djob_env.isr_job_queue);

    // Add a critical section as list can be accessed under interrupt context
    GLOBAL_INT_DISABLE();
    p_djob = (co_djob_t *)co_list_pop_front(p_job_queue);
    // Check if another job must be executed
    if (co_list_is_empty(p_job_queue))
    {
        // Clear event
        ke_event_clear(KE_EVENT_DJOB_ISR);
    }
    GLOBAL_INT_RESTORE();

    if (p_djob != NULL)
    {
        p_djob->hdr.next = NULL;
        p_djob->cb(p_djob);
    }

    DBG_FUNC_EXIT(co_djob_isr_evt_handler);
}

/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */

void co_djob_initialize(bool is_reset)
{
    if (!is_reset)
    {
        // Register BLE delayed job kernel event
        ke_event_callback_set(KE_EVENT_DJOB_LP,  &co_djob_lp_evt_handler);
        ke_event_callback_set(KE_EVENT_DJOB_HP,  &co_djob_hp_evt_handler);
        ke_event_callback_set(KE_EVENT_DJOB_ISR, &co_djob_isr_evt_handler);
        co_list_init(&(co_djob_env.isr_job_queue));
    }

    // remove all delayed job pending
    co_list_init(&(co_djob_env.job_queue[CO_DJOB_LOW]));
    co_list_init(&(co_djob_env.job_queue[CO_DJOB_HIGH]));
}


void co_djob_init(co_djob_t *p_djob, co_djob_cb cb)
{
    p_djob->hdr.next = NULL;
    p_djob->cb       = cb;
}

void co_djob_reg(uint8_t prio_lvl, co_djob_t *p_djob)
{
    co_list_t *p_job_queue = &(co_djob_env.job_queue[prio_lvl]);
    ASSERT_ERR(prio_lvl < CO_DJOB_PRIO_NB);

    // check if job is not already present in the queue
    if (p_djob->hdr.next == NULL)
    {
        // Set event only if no job present in queue
        if (co_list_is_empty(p_job_queue))
        {
            ke_event_set(co_djob_evt[prio_lvl]);
        }

        // Put element at end of the list
        co_list_push_back(p_job_queue, &(p_djob->hdr));
    }
}

void co_djob_unreg(uint8_t prio_lvl, co_djob_t *p_djob)
{
    co_list_t *p_job_queue = &(co_djob_env.job_queue[prio_lvl]);
    ASSERT_ERR(prio_lvl < CO_DJOB_PRIO_NB);

    // extract element
    co_list_extract(p_job_queue, &(p_djob->hdr));
    p_djob->hdr.next = NULL;

    // Check if another job must be executed
    if (co_list_is_empty(p_job_queue))
    {
        ke_event_clear(co_djob_evt[prio_lvl]);
    }
}

void co_djob_isr_reg(co_djob_t *p_djob)
{
    uint8_t is_evt_set = 0;
    // check if job is not already present in the queue
    co_list_t *p_job_queue = &(co_djob_env.isr_job_queue);
    //if (p_djob->hdr.next == NULL)
    if (!co_list_find(p_job_queue, &(p_djob->hdr)))
    {
        // Set event only if no job present in queue
        if (co_list_is_empty(p_job_queue))
            is_evt_set = 1;

        // Add a critical section as function can be called under interrupt context
        GLOBAL_INT_DISABLE();
        // Put element at end of the list
        co_list_push_back(p_job_queue, &(p_djob->hdr));
        GLOBAL_INT_RESTORE();

        if (is_evt_set)
            ke_event_set(KE_EVENT_DJOB_ISR);
    }
}

/// @} CO_DJOB

