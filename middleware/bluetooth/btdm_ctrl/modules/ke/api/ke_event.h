/**
 ****************************************************************************************
 *
 * @file ke_event.h
 *
 * @brief This file contains the definition related to kernel events.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

#ifndef _KE_EVENT_H_
#define _KE_EVENT_H_

/**
 ****************************************************************************************
 * @addtogroup EVT Events and Schedule
 * @ingroup KERNEL
 * @brief Event scheduling module.
 *
 * The KE_EVT module implements event scheduling functions. It can be used to
 * implement deferred actions.
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"          // stack configuration

#include <stdint.h>       // standard integer definition


/*
 * CONSTANTS
 ****************************************************************************************
 */

#define KE_EVT_TASK_PRIOIRTY      (RT_THREAD_PRIORITY_HIGH)



/// Status of ke_task API functions
enum KE_EVENT_STATUS
{
    KE_EVENT_OK = 0,
    KE_EVENT_FAIL,
    KE_EVENT_UNKNOWN,
    KE_EVENT_CAPA_EXCEEDED,
    KE_EVENT_ALREADY_EXISTS,
};

enum
KE_EVENT_PRIORITY_TYPE
{
    KE_EVENT_PRI_LOW,
    KE_EVENT_PRI_MEDIUM,
    KE_EVNET_PRI_HIGH,
    KE_EVNET_PRI_HIGHEST,
    KE_EVNET_PRI_MAX
};



/*
 * TYPE DEFINITION
 ****************************************************************************************
 */




/*
 * FUNCTION PROTOTYPES
 ****************************************************************************************
 */



/**
 ****************************************************************************************
 * @brief Initialize Kernel event module.
 ****************************************************************************************
 */
void _ke_event_init(void);

/**
 ****************************************************************************************
 * @brief Register an event callback.
 *
 * @param[in]  event_type       Event type.
 * @param[in]  p_callback       Pointer to callback function.
 *
 * @return                      Status
 ****************************************************************************************
 */
uint8_t _ke_event_callback_set(uint8_t event_type, void (*p_callback)(void));

/**
 ****************************************************************************************
 * @brief Set an event
 *
 * This primitive sets one event. It will trigger the call to the corresponding event
 * handler in the next scheduling call.
 *
 * @param[in]  event_type      Event to be set.
 ****************************************************************************************
 */
void _ke_event_set(uint8_t event_type);

/**
 ****************************************************************************************
 * @brief Clear an event
 *
 * @param[in]  event_type      Event to be cleared.
 ****************************************************************************************
 */
void _ke_event_clear(uint8_t event_type);

/**
 ****************************************************************************************
 * @brief Get the status of an event
 *
 * @param[in]  event_type      Event to get.
 *
 * @return                     Event status (0: not set / 1: set)
 ****************************************************************************************
 */
uint8_t _ke_event_get(uint8_t event_type);

/**
 ****************************************************************************************
 * @brief Get all event status
 *
 * @return                     Events bit field
 ****************************************************************************************
 */
uint32_t _ke_event_get_all(void);

/**
 ****************************************************************************************
 * @brief Flush all pending events.
 ****************************************************************************************
 */
void _ke_event_flush(void);

/**
 ****************************************************************************************
 * @brief Event scheduler entry point.
 *
 * This primitive is the entry point of Kernel event scheduling.
 ****************************************************************************************
 */
void _ke_event_schedule(void);

void _ke_event_schedule_via_evt_mask(uint32_t evt_mask);

typedef uint8_t (*ke_evt_get_priority_handler)(uint8_t event_type);
typedef void (*ke_evt_task_entry_handler)(void *param);
typedef void (*ke_event_init_handler)(void);
typedef uint8_t (*ke_event_callback_set_handler)(uint8_t event_type, void (*p_callback)(void));
typedef void (*ke_event_set_handler)(uint8_t event_type);
typedef void (*ke_event_clear_handler)(uint8_t event_type);
typedef uint8_t (*ke_event_get_handler)(uint8_t event_type);
typedef uint32_t (*ke_event_get_all_handler)(void);
typedef void (*ke_event_flush_handler)(void);
typedef void (*ke_event_schedule_handler)(void);
typedef void (*ke_event_schedule_via_evt_mask_handler)(uint32_t evt_mask);

extern ke_evt_get_priority_handler _ke_evt_get_priority_handler;
extern ke_evt_task_entry_handler _ke_evt_task_entry_handler;
extern ke_event_init_handler _ke_event_init_handler;
extern ke_event_callback_set_handler _ke_event_callback_set_handler;
extern ke_event_set_handler _ke_event_set_handler;
extern ke_event_clear_handler _ke_event_clear_handler;
extern ke_event_get_handler _ke_event_get_handler;
extern ke_event_get_all_handler _ke_event_get_all_handler;
extern ke_event_flush_handler _ke_event_flush_handler;
extern ke_event_schedule_handler _ke_event_schedule_handler;
extern ke_event_schedule_via_evt_mask_handler _ke_event_schedule_via_evt_mask_handler;

#define ke_event_init()  _ke_event_init_handler()
#define ke_event_callback_set(event_type, p_callback)  _ke_event_callback_set_handler(event_type, p_callback)
#define ke_event_set(event_type)   _ke_event_set_handler(event_type)
#define ke_event_clear(event_type)  _ke_event_clear_handler(event_type)
#define ke_event_get(event_type)  _ke_event_get_handler(event_type)
#define ke_event_get_all()       _ke_event_get_all_handler()
#define ke_event_flush()    _ke_event_flush_handler()
#define ke_event_schedule()  _ke_event_schedule_handler()
#define ke_event_schedule_via_evt_mask(evt_mask)  _ke_event_schedule_via_evt_mask_handler(evt_mask)

/// @} EVT

#endif //_KE_EVENT_H_
