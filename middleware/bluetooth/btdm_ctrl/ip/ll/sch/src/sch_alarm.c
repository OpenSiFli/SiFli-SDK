/**
 ****************************************************************************************
 *
 * @file sch_alarm.c
 *
 * @brief Scheduling alarm module
 *
 * Copyright (C) RivieraWaves 2009-2017
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup SCH_ALARM main module
 * @ingroup SCH
 * @brief The SCH_ALARM main module.
 *
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#include <string.h>          // For mem* functions
#include "arch.h"            // For asserts
#include "co_bt.h"
#include "co_math.h"
#include "co_list.h"
#include "ke_mem.h"
#include "ll.h"
#include "rwip.h"
#include "co_utils.h"

#include "dbg.h"
#include "sch_alarm.h"
#include "btdm_patch.h"


/*
 * DEFINES
 ****************************************************************************************
 */

/// Undefined timestamp value
#define SCH_ALARM_UNDEF_TIME           (0xFFFFFFFF)

/// Margin for programming the HW timer in (half-us)
#define SCH_ALARM_TIMER_PROG_MARGIN    (100)


/*
 * ENUMERATIONS DEFINITIONS
 ****************************************************************************************
 */


/*
 * STRUCT DEFINITIONS
 ****************************************************************************************
 */
/// Scheduling alarm Environment
struct sch_alarm_env_tag
{
    /// List of alarms
    struct co_list alarm_list;
};

/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */

/// Scheduling Alarm environment variable
__STATIC struct sch_alarm_env_tag sch_alarm_env;

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @brief API to try to check if a timer should be started
 ****************************************************************************************
 */
__STATIC void sch_alarm_prog(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(SCH_ALARM_PATCH_TYPE, SCH_ALARM_PROG_FUNC_BIT);

    // Check the 1st alarm
    struct sch_alarm_tag *alarm = (struct sch_alarm_tag *) co_list_pick(&sch_alarm_env.alarm_list);

    if (alarm != NULL)
    {
        // Get current time
        rwip_time_t target_min = rwip_time_get();
        rwip_time_t target     = alarm->time;

        // Add a margin to secure timer programming
        target_min.hus += SCH_ALARM_TIMER_PROG_MARGIN;

        while (target_min.hus >= HALF_SLOT_SIZE)
        {
            target_min.hus -= HALF_SLOT_SIZE;
            target_min.hs = CLK_ADD_2(target_min.hs, 1);
        }

        if (CLK_GREATER_THAN_HUS(target_min.hs, target_min.hus, target.hs, target.hus))
        {
            target.hs  = target_min.hs;
            target.hus = target_min.hus;
        }

        // Program timer
        rwip_timer_alarm_set(target.hs, target.hus);
    }
    else
    {
        // If no fine target timer to be set
        rwip_timer_alarm_set(RWIP_INVALID_TARGET_TIME, 0);
    }
}

/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void sch_alarm_init(bool is_reset)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(SCH_ALARM_PATCH_TYPE, SCH_ALARM_INIT_FUNC_BIT, is_reset);
    co_list_init(&sch_alarm_env.alarm_list);
}



void sch_alarm_timer_isr(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(SCH_ALARM_PATCH_TYPE, SCH_ALARM_TIMER_ISR_FUNC_BIT);

    struct sch_alarm_tag *alarm = NULL;

    do
    {
        // Get current time
        rwip_time_t current_time = rwip_time_get();

        // retrieve first alarm to trigger
        alarm = (struct sch_alarm_tag *) co_list_pick(&sch_alarm_env.alarm_list);

        // Check the alarm expiry timestamp
        if ((alarm != NULL) && CLK_LOWER_EQ(alarm->time.hs, current_time.hs))
        {
            // Pop the alarm
            co_list_pop_front(&sch_alarm_env.alarm_list);

            // Invoke call back function
            if (alarm->cb_alarm != NULL)
            {
                alarm->cb_alarm(alarm);
            }
            else
            {
                ASSERT_ERR_FORCE(0);
            }
        }
        // nothing more to do
        else
        {
            break;
        }
    }
    while (true);

    // Update/clear timer as needed
    sch_alarm_prog();
}

void sch_alarm_set(struct sch_alarm_tag *elt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(SCH_ALARM_PATCH_TYPE, SCH_ALARM_SET_FUNC_BIT, elt);

    struct sch_alarm_tag *prev = NULL;
    struct sch_alarm_tag *next;

    GLOBAL_INT_DISABLE();

    // Point to 1st alarm
    next = (struct sch_alarm_tag *) co_list_pick(&sch_alarm_env.alarm_list);

    // Find the place to insert
    while (next != NULL)
    {
        if (CLK_LOWER_EQ_HUS(elt->time.hs, elt->time.hus, next->time.hs, next->time.hus))
        {
            break;
        }

        // Jump to next element
        prev = next;
        next = (struct sch_alarm_tag *) co_list_next((struct co_list_hdr *) next);
    }

    // Check the position of the new alarm
    if (prev)
    {
        // Second or more
        co_list_insert_after(&sch_alarm_env.alarm_list, &prev->hdr, &elt->hdr);
    }
    else
    {
        // First
        co_list_push_front(&sch_alarm_env.alarm_list, &elt->hdr);

        // Update timer if needed
        sch_alarm_prog();
    }

    GLOBAL_INT_RESTORE();
}

uint8_t sch_alarm_clear(struct sch_alarm_tag *elt)
{
    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(SCH_ALARM_PATCH_TYPE, SCH_ALARM_CLEAR_FUNC_BIT, uint8_t, elt);

    uint8_t status = SCH_ALARM_ERROR_OK;

    GLOBAL_INT_DISABLE();

    do
    {
        if (((struct co_list_hdr *)elt) == co_list_pick(&sch_alarm_env.alarm_list))
        {
            // Pop the first element of the list
            co_list_pop_front(&sch_alarm_env.alarm_list);

            // Update/clear timer as needed
            sch_alarm_prog();
            break;
        }

        if (co_list_extract(&sch_alarm_env.alarm_list, (struct co_list_hdr *) elt))
            break;

        status = SCH_ALARM_ERROR_NOT_FOUND;
    }
    while (0);

    GLOBAL_INT_RESTORE();

    return (status);
}

///@} SCH_ALARM
