/**
****************************************************************************************
*
* @file SCH_PLAN.h
*
* @brief SCH_PLAN main module
*
* Copyright (C) RivieraWaves 2009-2018
*
*
****************************************************************************************
*/

#ifndef _SCH_PLAN_H_
#define _SCH_PLAN_H_

/**
 ****************************************************************************************
 * @addtogroup SCH
 * @brief Entry points of the Scheduling Planner module
 *
 * This module contains the primitives that allow stacks to plan periodic activities
 *
 * @{
 ****************************************************************************************
 */

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

/// SCH_PLAN error codes
enum sch_plan_error
{
    /// SCH_PLAN request succeed
    SCH_PLAN_ERROR_OK                 = 0,
    /// SCH_PLAN request rejected error
    SCH_PLAN_ERROR_REJECTED,
    /// SCH_PLAN element not found error
    SCH_PLAN_ERROR_NOT_FOUND,
    /// SCH_PLAN request rejected due to bandwidth full error
    SCH_PLAN_ERROR_BW_FULL
};

/// SCH_PLAN mobility levels
enum sch_plan_mb_lvl
{
    /// SCH_PLAN mobility level 0 (not movable)
    SCH_PLAN_MB_LVL_0                 = 0,
    /// SCH_PLAN mobility level 1
    SCH_PLAN_MB_LVL_1,
    /// SCH_PLAN mobility level 2
    SCH_PLAN_MB_LVL_2,
    /// SCH_PLAN mobility level 3
    SCH_PLAN_MB_LVL_3,
    /// SCH_PLAN mobility level 4
    SCH_PLAN_MB_LVL_4
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Planner element structure
struct sch_plan_elt_tag
{
    /// List element for chaining in the Interval list
    struct co_list_hdr hdr;
    /// Move notification call back function
    void (*cb_move)(uint16_t conhdl);
    /// Interval (in half-slots)
    uint16_t interval;
    /// Offset (in half-slots)
    uint16_t offset;
    /// Minimum duration (in half-slots)
    uint16_t duration_min;
    /// Maximum duration (in half-slots)
    uint16_t duration_max;
    /// Connection handle
    uint16_t conhdl;
    /// Reference connection handle, the primary activity on which the new activity refers to (e.g. ACL connection for a synchronous connection)
    uint16_t conhdl_ref;
    /// Margin to consider around the activity events, when not related to the same activity (in half-slots)
    uint16_t margin;
    /// Mobility level
    uint8_t mobility;
};

/// API request parameters
struct sch_plan_req_param
{
    /// Interval minimum requested (in half-slots)
    uint16_t interval_min;
    /// Interval maximum requested (in half-slots)
    uint16_t interval_max;
    /// Interval returned (in half-slots)
    uint16_t interval;
    /// Duration minimum requested (in half-slots)
    uint16_t duration_min;
    /// Duration maximum requested (in half-slots)
    uint16_t duration_max;
    /// Minimum offset requested and returned (in half-slots)
    uint16_t offset_min;
    /// Maximum offset requested and returned (in half-slots)
    uint16_t offset_max;
    /// Connection handle
    uint16_t conhdl;
    /// Reference connection handle, the primary activity on which the new activity refers to (e.g. ACL connection for a synchronous connection)
    uint16_t conhdl_ref;
    /// Margin to consider around the activity events, when not related to the same activity (in half-slots)
    uint16_t margin;
    /// Preferred periodicity (in half-slots)
    uint8_t pref_period;
};

/// API check parameters
struct sch_plan_chk_param
{
    /// Interval (in half-slots)
    uint16_t interval;
    /// Duration minimum requested (in half-slots)
    uint16_t duration_min;
    /// Offset requested (in half-slots)
    uint16_t offset;
    /// Connection handle
    uint16_t conhdl;
    /// Reference connection handle, the primary activity on which the new activity refers to (e.g. ACL connection for a synchronous connection)
    uint16_t conhdl_ref;
    /// Margin to consider around the activity events, when not related to the same activity (in half-slots)
    uint16_t margin;
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
 * @brief Initialize the Scheduling Planner.
 *
 * @param[in] is_reset  True if reset is requested, false for an initialization
 ****************************************************************************************
 */
void sch_plan_init(bool is_reset);

/**
 ****************************************************************************************
 * @brief Insert the element in the planning
 ****************************************************************************************
 */
void sch_plan_set(struct sch_plan_elt_tag *plan_elt);

/**
 ****************************************************************************************
 * @brief Remove the element from the planning
 ****************************************************************************************
 */
void sch_plan_rem(struct sch_plan_elt_tag *plan_elt);

/**
 ****************************************************************************************
 * @brief Choose appropriate parameters for the activity
 *
 * @param[in|out] req_param  Input parameters, modified according to the planning
 *
 * @return See sch_plan_error error codes
 ****************************************************************************************
 */
uint8_t sch_plan_req(struct sch_plan_req_param *req_param);

/**
 ****************************************************************************************
 * @brief Check if the provided activity parameters are fitting in the planning
 *
 * @param[in] chk_param      Input parameters
 *
 * @return See sch_plan_error error codes
 ****************************************************************************************
 */
uint8_t sch_plan_chk(struct sch_plan_chk_param *chk_param);

/**
 ****************************************************************************************
 * @brief Shift activities related to a reference connection handle
 *
 * The planner updates all activities related to a reference connection handle. If a conflict results to
 * the shift, it might notify one activity to move.
 *
 * @param[in]     conhdl_ref  Reference connection handle
 * @param[in]     shift       Shift in (half-slots)
 ****************************************************************************************
 */
void sch_plan_shift(uint16_t conhdl_ref, int16_t shift);


///@} SCH_PLAN

#endif // _SCH_PLAN_H_
