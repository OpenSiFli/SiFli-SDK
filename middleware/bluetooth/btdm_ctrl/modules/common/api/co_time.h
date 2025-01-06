/**
 ****************************************************************************************
 *
 * @file co_time.h
 *
 * @brief The Common Time module provides information about the device time and a
 *        scheduler for timers used by the different modules. It maintains a list of
 *        pending timers sorted by ascending expiration time. One timer is programmed
 *        at a given time. A callback is associated with each timer and is called upon
 *        expiration of the timer.
 *
 *        Timers shall be used for non real time software.
 *
 * Copyright (C) RivieraWaves 2009-2019
 *
 ****************************************************************************************
 */
#ifndef _CO_TIME_H_
#define _CO_TIME_H_

/**
 ****************************************************************************************
 * @defgroup CO_TIME_API Non-Real time API for time and timer
 * @ingroup COMMON_API
 * @brief  Time utilities
 *
 * This module contains the Common time utilities functions and macros.
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup CO_TIME_TIME_API Time
 * @ingroup CO_TIME_API
 *
 * Retrieve time in millisecond
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup CO_TIMER_API Timer
 * @ingroup CO_TIME_API
 *
 * Start/Stop millisecond timers
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup CO_TIMER_PERIODIC_API Periodic timer
 * @ingroup CO_TIME_API
 *
 * Start/Stop millisecond periodic timers
 *
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <stdbool.h>
#include <stdint.h>       // standard definitions
#include <stddef.h>       // standard definitions

/*
 * MACRO DEFINITIONS
 ****************************************************************************************
 */

/// @addtogroup CO_TIME_TIME_API
/// @{

/// convert seconds to milliseconds
#define CO_TIME_SEC_TO_MS(sec) ((sec) * 1000)
/// @}

/// @addtogroup CO_TIME_TIME_API
/// @{

/// Maximum timer duration that can be programed  (in milliseconds)
#define CO_TIMER_MAX_DURATION (0x7FFFFFFF)
/// @}

/// @addtogroup CO_TIMER_PERIODIC_API
/// @{

/// Maximum period duration that can be programed (in milliseconds)
#define CO_TIMER_MAX_PERIOD (0x7FFFFFFF)
/// @}

/*
 * ENUMERATIONS DEFINITIONS
 ****************************************************************************************
 */

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */


/// @addtogroup CO_TIME_TIME_API
/// @{

/// Time Structure
typedef struct co_time
{
    /// Current time [0-31] part (in milliseconds)
    uint32_t ms_lsb;
    /// Current time [32-39] part (in milliseconds)
    uint8_t  ms_msb;
} co_time_t;
/// @}



typedef struct co_timer co_timer_t;

/**
 ****************************************************************************************
 * @brief Function to called once timer expires
 *
 * @param[in] p_timer   Pointer to timer which expired.
 ****************************************************************************************
 */
typedef void (*co_timer_cb)(co_timer_t *p_timer);

/// Timer structure
struct co_timer
{
    /// Pointer to next timer in timer list
    co_timer_t *p_next;
    /// Callback to execute in background context upon timer expiration
    co_timer_cb cb;
    /// Expiration time instant in millisecond
    uint32_t    expire_instant_ms;
};



typedef struct co_timer_periodic co_timer_periodic_t;

/**
 ****************************************************************************************
 * @brief Function called each timer period
 *
 * @param[in] p_timer   Pointer to periodic timer which expired.
 ****************************************************************************************
 */
typedef void (*co_timer_periodic_cb)(co_timer_periodic_t *p_timer);

/// Periodic timer structure
struct co_timer_periodic
{
    /// Inherited from timer
    co_timer_t hdr;
    /// Callback to execute in background context upon periodic timer expiration
    co_timer_periodic_cb cb;
    /// Period time in milliseconds (cannot be greater than #CO_TIMER_MAX_PERIOD)
    uint32_t period_ms;
};


/*
 * CONSTANT DECLARATIONS
 ****************************************************************************************
 */

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialize Common time module.
 *
 * @param[in] is_reset  True if reset is requested, false for an initialization
 ****************************************************************************************
 */
void co_time_init(bool is_reset);

/*
 ****************************************************************************************
 * Time and timer functions
 ****************************************************************************************
 */

/// @addtogroup CO_TIME_TIME_API
/// @{

/**
 ****************************************************************************************
 * @brief Retrieve current time in milliseconds.
 *
 * Time value can be either Up-time of the device or Time since the device has been
 * started for the first time.
 *
 * This depends if system is able to compensate power-off time to update internal time value
 *
 * @return current time in milliseconds.
 *
 ****************************************************************************************
 */
co_time_t co_time_get(void);


/**
 ****************************************************************************************
 * @brief Retrieve current time in milliseconds.
 *
 * Compensate device time. This compensation should be done after power-up of the device.
 * The time compensation is automatically performed by device when wake-up from sleep mode.
 * This function shall be called when no timer are programmed at device start-up.
 *
 * @param[in] delta_time_ms_lsb Delta time [0-31] part (in milliseconds)
 * @param[in] delta_time_ms_msb Delta time [32-39] part (in milliseconds)
 ****************************************************************************************
 */
void co_time_compensate(uint32_t delta_time_ms_lsb, uint8_t delta_time_ms_msb);

/// @}


/// @addtogroup CO_TIMER_API
/// @{

/**
 ****************************************************************************************
 * @brief Initialize timer structure.
 *
 * @param[in] p_timer    Pointer to the timer structure.
 * @param[in] cb         Function to be called upon timer expiration.
 ****************************************************************************************
 */
void co_timer_init(co_timer_t *p_timer, co_timer_cb cb);

/**
 ****************************************************************************************
 * @brief Program a timer to be scheduled in the future.
 *        If timer is already programmed, it is restarted.
 *        If delay is less than 10ms, delay is set to 10ms.
 *
 * @param[in] p_timer    Pointer to the timer structure.
 * @param[in] delay_ms   Duration of the timer in milliseconds (cannot be greater than #CO_TIMER_MAX_DURATION).
 ****************************************************************************************
 */
void co_timer_set(co_timer_t *p_timer, uint32_t delay_ms);

/**
 ****************************************************************************************
 * @brief Stop a programmed timer.
 *
 * @param[in] p_timer    Pointer to the timer structure.
 ****************************************************************************************
 */
void co_timer_stop(co_timer_t *p_timer);

/// @}


/// @addtogroup CO_TIMER_PERIODIC_API
/// @{

/**
 ****************************************************************************************
 * @brief Initialize periodic timer structure.
 *
 * @param[in] p_timer    Pointer to the periodic timer structure.
 * @param[in] cb         Function to be called each timer period.
 ****************************************************************************************
 */
void co_timer_periodic_init(co_timer_periodic_t *p_timer, co_timer_periodic_cb cb);

/**
 ****************************************************************************************
 * @brief Program a timer to be scheduled periodically. If timer is already programmed, it is restarted.
 *        If period exceed maximum value, timer is programmed using maximum period.
 *        If period less than 10ms, period is set to 10ms.
 *
 *
 * @param[in] p_timer    Pointer to the periodic timer structure.
 * @param[in] period_ms  Period of the timer in milliseconds (cannot be greater than #CO_TIMER_MAX_PERIOD).
 ****************************************************************************************
 */
void co_timer_periodic_set(co_timer_periodic_t *p_timer, uint32_t period_ms);

/**
 ****************************************************************************************
 * @brief Stop a programmed periodic timer.
 *
 * @param[in] p_timer    Pointer to the periodic timer structure.
 ****************************************************************************************
 */
void co_timer_periodic_stop(co_timer_periodic_t *p_timer);

/// @}

#endif // _CO_TIME_H_
