/**
****************************************************************************************
*
* @file rwip_int.h
*
* @brief RW IP internal SW main module
*
* Copyright (C) RivieraWaves 2009-2015
*
*
****************************************************************************************
*/
#ifndef _RWIP_INT_H_
#define _RWIP_INT_H_

/**
 ****************************************************************************************
 * @addtogroup ROOT
 * @brief Entry points of the RW IP stacks/modules
 *
 * This module contains the primitives that allow an application accessing and running the
 * RW IP protocol stacks / modules.
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"          // stack configuration

#include <stdint.h>               // standard integer definitions
#include <stdbool.h>              // standard boolean definitions


/*
 * DEFINES
 ****************************************************************************************
 */


/*
 * STRUCTURE DEFINITIONS
 ****************************************************************************************
 */

/// RWIP Environment structure
struct rwip_env_tag
{
    /// Prevent sleep bit field
    uint32_t          prevent_sleep;

#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
    /// Arbiter target timer  (integer part, in half slots)
    rwip_time_t       timer_arb_target;
    /// Alarm target timer (integer part, in half slots)
    rwip_time_t       timer_alarm_target;
    /// For IP sleep isr mask
    uint32_t          irq_mask;
#if BLE_EMB_PRESENT
    uint32_t          irq_mask_ble;
#endif // BT_EMB_PRESENT
#if BT_EMB_PRESENT
    uint32_t          irq_mask_bt;
#endif // BT_EMB_PRESENT
#endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)
    /// Common target timer (in half slots)
    rwip_time_t       timer_co_target;
    /// Last Sampled time (used for time conversion)
    rwip_time_t       last_samp_time;

#if (EMB_PRESENT)
    /// channel assessment data
    struct rwip_ch_assess_data ch_assess;
#endif //EMB_PRESENT
#if (EAVESDROPPING_SUPPORT)
    /// Counter incremented before every deep sleep entrance
    uint32_t deep_sleep_counter;
#endif // EAVESDROPPING_SUPPORT

#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
    /// Maximum sleep duration (in LP cycles, depends on Low power clock frequency)
    uint32_t          sleep_dur_max;
    /// Contains sleep duration accumulated timing error (32kHz: 1/2 half us | 32.768kHz: 1/256 half-us)
    uint32_t          sleep_acc_error;
    /// Power_up delay (in LP clock cycle unit, depends on Low power clock frequency)
    uint32_t          lp_cycle_wakeup_delay;
    /// Used RC as lp clock if xTAL disabled. It indicates how many ref cycles contained in n lp cycles.
    uint32_t lp_ref_cycle;
#endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)
#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
    /// Maximum value of the sleep clock drift (in ppm)
    uint16_t          sleep_clock_drift;
    /// Maximum value of the sleep clock accuracy (@see enum SCA)
    uint8_t           sleep_clock_accuracy;
    /// Maximum value of the active clock drift (in ppm)
    uint8_t           active_clock_drift;
    /// External wake-up support
    bool              ext_wakeup_enable;
#if (!BLE_ISO_HW_PRESENT)
    /// BTS sampling clock half microseconds residual (0 or 1)
    uint8_t           samp_hus_residual;
#endif // (!BLE_ISO_HW_PRESENT)

    /// Channel assessment scheme enabled/disabled
    bool ch_ass_en;
    /// interferer threshold ('real' signed value in dBm)
    int8_t ch_ass_rssi_interf_thr;
#endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)
};


/*
 * GLOBAL DEFINITIONS
 ****************************************************************************************
 */

/// RW SW environment
extern struct rwip_env_tag rwip_env;


/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 * Initialization of the RW IP Common core driver
 *
 * @param[in] is_reset  True if reset is requested, false for an initialization
 */
void rwip_driver_init(bool is_reset);


///@} ROOT

#endif // _RWIP_INT_H_
