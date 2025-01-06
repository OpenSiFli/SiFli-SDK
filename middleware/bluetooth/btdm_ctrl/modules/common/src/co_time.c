/**
 ****************************************************************************************
 * @file co_time.c
 *
 * @brief Common Time Module
 *
 * Copyright (C) RivieraWaves 2017-2019
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup CO_TIME
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "co_time.h"      // Common time module
#include "arch.h"         // for defines
#include "rwip.h"         // To get system time
#include "co_utils.h"     // to get clock macros
#include "ke_event.h"     // kernel event used to handle time in a background task


/*
 * MACROS
 ****************************************************************************************
 */

/// Convert time in milliseconds to microseconds
#define CO_TIME_MS_TO_US(time) ((time) * (CO_TIME_1_MS_TO_US))

/*
 * DEFINES
 ****************************************************************************************
 */


/// 1 Milliseconds in half microseconds
#define CO_TIME_1_MS_TO_US              (1000)

/// Minimum delay 1 ms
#define CO_TIME_DELAY_MIN                (1)
/// Minimum period 8388608 ms ~= 2,3 hours
#define CO_TIME_PERIOD_MAX               (0x7FFFFF)
/// Maximum delay to reprogram system timer (15 min in microseconds)
#define CO_TIME_SYS_TIMER_DELAY_US_MAX   (900000000)
/// Maximum delay to reprogram system timer (15 min in milliseconds)
#define CO_TIME_SYS_TIMER_DELAY_MS_MAX   (900000)

/// Use to know if a timer is programmed or not
#define CO_TIMER_NOT_PROG                ((co_timer_t*) (0xFFFFFFFF))


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Timer environment structure
typedef struct co_time_env_
{
    /// First timer in the list sorted according to elapse time.
    co_timer_t    *p_first;

    /// Last sampled reference time (microseconds counter)
    uint32_t       last_samp_ref_clock_us;
    /// Microsecond rest for the conversion of microsecond reference clock to millisecond clock
    uint16_t       clock_conversion_us_rest;
    /// Last sampled time in milliseconds
    co_time_t           last_samp_time;
} co_time_env_t;

/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */

/// Time environment structure
__STATIC co_time_env_t co_time_env;

/*
 * LOCAL FUNCTIONS
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @brief Update of System timer
 *
 * Here it is considered that device time has been already sampled
 ****************************************************************************************
 */
__STATIC void co_time_sys_timer_update(void)
{
    DBG_FUNC_ENTER(co_time_sys_timer_update);
    // get current time - consider it has been already sampled
    co_time_t time       = co_time_env.last_samp_time;
    uint32_t  delay_us   = CO_TIME_SYS_TIMER_DELAY_US_MAX;
    co_timer_t *p_timer  = co_time_env.p_first;

    // Check if a timer is present
    if (p_timer != NULL)
    {
        int32_t delay_ms = p_timer->expire_instant_ms - time.ms_lsb;

        // check if timer is in the past looking at MSB
        if (delay_ms <= 0)
        {
            delay_us = 0;
        }
        // check if next timer delay is less that max system timer delay
        else if (delay_ms < CO_TIME_SYS_TIMER_DELAY_MS_MAX)
        {
            delay_us = CO_TIME_MS_TO_US(delay_ms);
        }
    }

    // if delay is one milliseconds or less, consider to trigger it now
    if (delay_us <= CO_TIME_1_MS_TO_US)
    {
        ke_event_set(KE_EVENT_TIMER);
    }
    // compute when next timer should be programmed
    else
    {

#if !EMB_PRESENT
        //rt timer only need duration but not target time
        uint32_t target_ref_clock_us = delay_us;
#else
        uint32_t target_ref_clock_us = co_time_env.last_samp_ref_clock_us + delay_us;
#endif
        rwip_timer_co_set(target_ref_clock_us);
    }
    DBG_FUNC_EXIT(co_time_sys_timer_update);
}

/**
 ****************************************************************************************
 * @brief Extract timer form timer queue
 *
 * @param[in] p_timer       Pointer to the timer structure.
 *
 * @return True if system timer must be updated, false otherwise
 ****************************************************************************************
 */
__STATIC bool co_timer_extract(co_timer_t *p_timer)
{
    bool update_sys_timer = false;
    co_timer_t *p_prev_timer = NULL;
    co_timer_t *p_cur_timer  = co_time_env.p_first;

    // find timer into timer list
    while (p_cur_timer != NULL)
    {
        // timer found
        if (p_cur_timer == p_timer)
        {
            // timer at begin of the list
            if (p_prev_timer == NULL)
            {
                co_time_env.p_first = p_cur_timer->p_next;
                update_sys_timer   = true;
            }
            // or into the list
            else
            {
                p_prev_timer->p_next = p_cur_timer->p_next;
            }

            break;
        }

        p_prev_timer = p_cur_timer;
        p_cur_timer  = p_cur_timer->p_next;
    }

    p_timer->p_next = CO_TIMER_NOT_PROG;

    return (update_sys_timer);
}


/**
 ****************************************************************************************
 * @brief Insert timer into timer queue
 *
 * @param[in] p_timer  Pointer to the timer structure.
 * @param[in] reprog   True: if it's a timer re-program, False otherwise
 *
 * @return True if system timer must be updated, false otherwise
 ****************************************************************************************
 */
__STATIC bool co_timer_insert(co_timer_t *p_timer)
{
    bool update_sys_timer = false;
    co_timer_t *p_prev_timer = NULL;
    co_timer_t *p_cur_timer;

    // check if timer must be first extract from timer list.
    if (p_timer->p_next != CO_TIMER_NOT_PROG)
    {
        update_sys_timer = co_timer_extract(p_timer);
    }

    p_cur_timer  = co_time_env.p_first;

    // find timer into timer list
    while (p_cur_timer != NULL)
    {
        int32_t time_diff_ms = (int32_t)(p_timer->expire_instant_ms - p_cur_timer->expire_instant_ms);

        // check if timer to insert elapse before current timer
        if (time_diff_ms < 0)
        {
            // insertion point found
            break;
        }

        p_prev_timer = p_cur_timer;
        p_cur_timer  = p_cur_timer->p_next;
    }


    // timer to insert at beginning of the list
    if (p_prev_timer == NULL)
    {
        co_time_env.p_first = p_timer;
        update_sys_timer   = true;
    }
    // or into the list (before current)
    else
    {
        p_prev_timer->p_next = p_timer;
    }

    p_timer->p_next = p_cur_timer;

    return (update_sys_timer);
}

/**
 ****************************************************************************************
 * @brief Program a timer
 *
 * Here it is considered that device time has been already sampled
 *
 * @param[in] p_timer       Pointer to the timer structure.
 * @param[in] delay_ms_lsb  Duration before expiration of the timer (in milliseconds)
 ****************************************************************************************
 */
__STATIC void co_timer_prog(co_timer_t *p_timer, uint32_t delay_ms)
{
    // get current time
    co_time_t cur_time = co_time_get();
    p_timer->expire_instant_ms = cur_time.ms_lsb + delay_ms;

    // program timer
    if (co_timer_insert(p_timer))
    {
        // update system timer
        co_time_sys_timer_update();
    }
}

/**
 ****************************************************************************************
 * @brief Handle RWIP timer trigger.
 ****************************************************************************************
 */
__STATIC void co_timer_handler(void)
{
    // Clear event
    ke_event_clear(KE_EVENT_TIMER);

    // sample time
    co_time_t cur_time = co_time_get();

    while (co_time_env.p_first != NULL)
    {
        // pick first timer
        co_timer_t *p_timer  = co_time_env.p_first;
        // Compute time difference between current and next
        int32_t time_diff_ms = (int32_t)(p_timer->expire_instant_ms - cur_time.ms_lsb);

        // time expire instant in the past --> timer expires
        if (time_diff_ms <= 0)
        {
            // remove timer from the list
            co_time_env.p_first = p_timer->p_next;

            // execute timer callback
            p_timer->p_next = NULL; // mark it out of progammed list
            p_timer->cb(p_timer);
        }
        else
        {
            // stop loop
            break;
        }

        // refresh current time
        cur_time = co_time_get();
    }


    // Update system timer for clock update and possibly new timer trigger
    co_time_sys_timer_update();
}

/// Handle expiration of periodic timer
__STATIC void co_timer_periodic_expires(co_timer_periodic_t *p_timer)
{
    // update expiration time
    p_timer->hdr.expire_instant_ms += p_timer->period_ms;

    // Insert again timer
    if (co_timer_insert(&(p_timer->hdr)))
    {
        // update system timer
        co_time_sys_timer_update();
    }

    // Call handler of periodic expiration
    p_timer->cb(p_timer);
}
/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */

void co_time_init(bool is_reset)
{
    if (!is_reset)
    {
        co_time_env.last_samp_time.ms_lsb = 0;
        co_time_env.last_samp_time.ms_msb = 0;
        co_time_env.clock_conversion_us_rest = 0;
        co_time_env.last_samp_ref_clock_us   = rwip_time_get().bts;

        // Register System timer handler
        ke_event_callback_set(KE_EVENT_TIMER, &co_timer_handler);
    }

    // Clear all timers
    co_time_env.p_first = NULL;
    // update HW timer
    co_time_get();
    // program next timer
    co_time_sys_timer_update();
}

/*
 ****************************************************************************************
 * Time and timer functions
 ****************************************************************************************
 */

#if !EMB_PRESENT

#if 0
co_time_t co_time_get(void)
{
    co_time_t r;
    rt_tick_t ticks = rt_tick_get_millisecond();

    r.ms_lsb = ticks & 0xffff;
    //r.ms_msb = (ticks >> 16) & 0xff;
    r.ms_msb = 0;
    return r;
}
#endif
void timeout_isr(void *param)
{
    co_timer_handler();
}

#endif
co_time_t co_time_get(void)
{
    co_time_t curr_time;

#if !EMB_PRESENT
    // In host subsystem, co_time_get may call in different thread
    GLOBAL_INT_DISABLE();
#endif
// In host, it get the ms
    uint32_t cur_ref_clock_us = rwip_time_get().bts;
    uint32_t clk_diff_ms;

    // compute time in microseconds between last sampled and new sampled reference clock in microsecond
    uint32_t clk_diff_us = cur_ref_clock_us - co_time_env.last_samp_ref_clock_us;

#if !EMB_PRESENT
    clk_diff_us *= 1000;
#endif

    // Reference clock sampling must be done at least every 15 minutes to ensure that
    // difference between last sampling and new sampling is always positive
    ASSERT_ERR(clk_diff_us < 0x80000000);

    clk_diff_ms          = clk_diff_us / CO_TIME_1_MS_TO_US;
    clk_diff_us         -= clk_diff_ms * CO_TIME_1_MS_TO_US;

    // update us counter
    co_time_env.clock_conversion_us_rest += clk_diff_us;

    // check if us counter must wrap
    if (co_time_env.clock_conversion_us_rest >= CO_TIME_1_MS_TO_US)
    {
        co_time_env.clock_conversion_us_rest -= CO_TIME_1_MS_TO_US;
        clk_diff_ms                    += 1;
    }

    // detect milliseconds counter LSB wrap
    if ((co_time_env.last_samp_time.ms_lsb + clk_diff_ms) < co_time_env.last_samp_time.ms_lsb)
    {
        // to increase milliseconds counter MSB part
        co_time_env.last_samp_time.ms_msb += 1;
    }

    // update milliseconds counter LSB part
    co_time_env.last_samp_time.ms_lsb += clk_diff_ms;

    // update last sampled value
    co_time_env.last_samp_ref_clock_us = cur_ref_clock_us;

    curr_time = co_time_env.last_samp_time;

#if !EMB_PRESENT
    GLOBAL_INT_RESTORE();
#endif
    // return current time
    return curr_time;
}
//#endif

void co_time_compensate(uint32_t delta_time_ms_lsb, uint8_t delta_time_ms_msb)
{
    // detect milliseconds counter LSB wrap
    if ((co_time_env.last_samp_time.ms_lsb + delta_time_ms_lsb) < co_time_env.last_samp_time.ms_lsb)
    {
        // to increase milliseconds counter MSB part
        co_time_env.last_samp_time.ms_msb += 1;
    }

    // update milliseconds counter LSB part
    co_time_env.last_samp_time.ms_lsb += delta_time_ms_lsb;

    // update milliseconds counter MSB part
    co_time_env.last_samp_time.ms_msb += delta_time_ms_msb;

    // check if some timer must be updated.
    ke_event_set(KE_EVENT_TIMER);
}

void co_timer_init(co_timer_t *p_timer, co_timer_cb cb)
{
    ASSERT_ERR(cb != NULL);
    // initialize callback and environment
    p_timer->p_next   = CO_TIMER_NOT_PROG;
    p_timer->cb       = cb;
}

void co_timer_set(co_timer_t *p_timer, uint32_t delay_ms)
{
    DBG_FUNC_ENTER(co_timer_set);

    // Force timer to be set in valid range
    CO_VAL_FORCE_RANGE(delay_ms, CO_TIME_DELAY_MIN, CO_TIMER_MAX_DURATION);

    // program timer
    co_timer_prog(p_timer, delay_ms);
    DBG_FUNC_EXIT(co_timer_set);
}


void co_timer_stop(co_timer_t *p_timer)
{
    // check if timer is present in timer queue
    if (p_timer->p_next != CO_TIMER_NOT_PROG)
    {
        // extract timer
        if (co_timer_extract(p_timer))
        {
            // if system timer must be updated, sample time and update system timer
            co_time_get();
            co_time_sys_timer_update();
        }
    }
}


void co_timer_periodic_init(co_timer_periodic_t *p_timer, co_timer_periodic_cb cb)
{
    ASSERT_ERR(cb != NULL);
    co_timer_init(&(p_timer->hdr), (co_timer_cb) co_timer_periodic_expires);
    p_timer->cb = cb;
}

void co_timer_periodic_set(co_timer_periodic_t *p_timer, uint32_t period_ms)
{
    // Force timer to be set in valid range
    CO_VAL_FORCE_RANGE(period_ms, CO_TIME_DELAY_MIN, CO_TIMER_MAX_DURATION);
    p_timer->period_ms = period_ms;

    // program timer
    co_timer_prog(&(p_timer->hdr), period_ms);
}

void co_timer_periodic_stop(co_timer_periodic_t *p_timer)
{
    co_timer_stop(&(p_timer->hdr));
}


/// @} CO_TIME

