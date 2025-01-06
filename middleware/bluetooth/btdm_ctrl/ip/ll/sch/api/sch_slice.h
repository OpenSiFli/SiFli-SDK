/**
****************************************************************************************
*
* @file sch_slice.h
*
* @brief LD Scheduling Parameters API
*
* Copyright (C) RivieraWaves 2009-2015
*
****************************************************************************************
*/

#ifndef SCH_SLICE_H_
#define SCH_SLICE_H_

/**
 ****************************************************************************************
 * @defgroup SCHSLICE Scheduling parameters
 * @ingroup SCH
 * @brief Responsible for selecting the optimal scheduling parameters.
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"    // stack configuration

#include <stdint.h>         // integer
#include <stdbool.h>        // boolean

/*
 * DEFINES
 ****************************************************************************************
 */

/// Undefined BT clock value
#define SCH_CLOCK_UNDEF       0xFFFFFFFF


/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/// Activity type
enum activity_type
{
#if BLE_EMB_PRESENT
    BLE_SCAN,
    BLE_INIT,
    BLE_HDC_ADV,
    BLE_PER_ADV,
    BLE_CON,
#endif //BLE_EMB_PRESENT

#if BT_EMB_PRESENT
    BT_INQ,
    BT_PAGE,
    BT_ISCAN,
    BT_PSCAN,
    BT_SSCAN,
    BT_SNIFF,
    BT_SCO,
#endif //BT_EMB_PRESENT
};


/*
 * STRUCTURE DEFINITIONS
 ****************************************************************************************
 */

/// SCH SLICE parameters structure
struct sch_slice_params_tag
{
    /// Scan / Inq / Page event duration (in half-us)
    uint16_t scan_evt_dur;

#if BT_EMB_PRESENT
    /// ACL event duration size (in half-us)
    uint16_t acl_evt_dur;

    /// SCO retx are allowed to reserve
    bool sco_retx_allowed;
#endif //BT_EMB_PRESENT
};


/*
 * VARIABLE DECLARATION
 ****************************************************************************************
 */

/// SCH SLICE parameters variable
extern struct sch_slice_params_tag sch_slice_params;


/*
 * FUNCTIONS DECLARATION
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialize the SCH_SLICE module
 *
 * @param[in] is_reset  True if reset is requested, false for an initialization
 ****************************************************************************************
 */
void sch_slice_init(bool is_reset);

/**
 ****************************************************************************************
 * @brief Add a background activity
 *
 * @param[in]  type        Activity type (@see @enum activity_type)
 ****************************************************************************************
 */
void sch_slice_bg_add(uint8_t type);

/**
 ****************************************************************************************
 * @brief Remove a background activity
 *
 * @param[in]  type        Activity type (@see @enum activity_type)
 ****************************************************************************************
 */
void sch_slice_bg_remove(uint8_t type);

/**
 ****************************************************************************************
 * @brief Add a periodic activity
 *
 * @param[in]  type        Activity type (@see @enum activity_type)
 * @param[in]  id          Identifier of the activity (depending on the activity type)
 * @param[in]  intv        Interval (in half-slot)
 * @param[in]  duration    Events duration (in half-us)
 * @param[in]  retx        Potential retransmissions (applicable to SCO only)
 ****************************************************************************************
 */
void sch_slice_per_add(uint8_t type, uint8_t id, uint32_t intv, uint16_t duration, bool retx);

/**
 ****************************************************************************************
 * @brief Remove a periodic activity
 *
 * @param[in]  type        Activity type (@see @enum activity_type)
 * @param[in]  id          Identifier of the activity (depending on the activity type)
 ****************************************************************************************
 */
void sch_slice_per_remove(uint8_t type, uint8_t id);

/// @} SCHSLICE
#endif // SCH_SLICE_H_
