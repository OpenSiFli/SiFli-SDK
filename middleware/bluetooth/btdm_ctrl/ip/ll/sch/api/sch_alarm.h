/**
****************************************************************************************
*
* @file sch_alarm.h
*
* @brief sch_alarm main module
*
* Copyright (C) RivieraWaves 2009-2017
*
*
****************************************************************************************
*/

#ifndef _SCH_ALARM_H_
#define _SCH_ALARM_H_

/**
 ****************************************************************************************
 * @addtogroup SCH
 * @brief Entry points of the Scheduling alaram module
 *
 * @{
 ****************************************************************************************
 */

#include "rwip_config.h"

#include <stdint.h>               // Standard integer definitions
#include <stdbool.h>              // Standard boolean definitions
#include "co_list.h"              // List management functions definitions


/*
 * MACROS
 ****************************************************************************************
 */



/*
 * DEFINES
 ****************************************************************************************
 */


/*
 * DEFINITIONS
 ****************************************************************************************
 */

/// SCH_ALARM error codes
enum sch_alarm_error
{
    /// SCH_ALARM request succeed
    SCH_ALARM_ERROR_OK                 = 0,
    /// SCH_ALARM request rejected error
    SCH_ALARM_ERROR_REJECTED,
    /// SCH_ALARM element not found error
    SCH_ALARM_ERROR_NOT_FOUND,
    /// SCH_ALARM request rejected due to bandwidth full error
    SCH_ALARM_ERROR_BW_FULL
};


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Alarm element structure
struct sch_alarm_tag
{
    /// List element for chaining in the Even Arbiter lists
    struct co_list_hdr hdr;

    /// Timestamp of alarm expiry (in BT half-slots and half-us)
    rwip_time_t        time;

    /// Call back function invoked upon alarm expiry
    void (*cb_alarm)(struct sch_alarm_tag *);
};

/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */


/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialize the Scheduling alarm.
 *
 * @param[in] is_reset  True if reset is requested, false for an initialization
 ****************************************************************************************
 */
void sch_alarm_init(bool is_reset);

/**
 ****************************************************************************************
 * @brief Alarm interrupt handler
 *
 * This function is called under interrupt when an alarm is triggered.
 *
 ****************************************************************************************
 */
void sch_alarm_timer_isr(void);

/**
 ****************************************************************************************
 * @brief Set an alarm
 *
 * If the alarm specify an instant in the past, the alarm is scheduled for earliest
 * possible time.
 *
 * @param[in] elt      Pointer to the alarm element to be programmed
 ****************************************************************************************
 */
void sch_alarm_set(struct sch_alarm_tag *elt);

/**
 ****************************************************************************************
 * @brief Clear an alarm
 *
 * @param[in] elt      Pointer to the alarm element to be cleared
 *
 * @return 0 - no error / 1:255 - error
 ****************************************************************************************
 */
uint8_t sch_alarm_clear(struct sch_alarm_tag *elt);

///@} SCH_ALARM

#endif // _SCH_ALARM_H_
