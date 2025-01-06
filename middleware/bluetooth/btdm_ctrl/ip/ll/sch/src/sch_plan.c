/**
 ****************************************************************************************
 *
 * @file sch_plan.c
 *
 * @brief Scheduling Planner module
 *
 * Copyright (C) RivieraWaves 2009-2018
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup SCH_PLAN main module
 * @ingroup SCH
 * @brief The SCH_PLAN main module.
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "arch.h"            // For asserts
#include "co_math.h"
#include "sch_plan.h"
#include "co_utils.h"
#include "btdm_patch.h"


/*
 * ENUMERATIONS DEFINITIONS
 ****************************************************************************************
 */

/// Action for the parameters request API
enum sch_plan_param_req_action
{
    SCH_PLAN_PARAM_REQ_GET,
    SCH_PLAN_PARAM_REQ_CHECK,
};


/*
 * STRUCT DEFINITIONS
 ****************************************************************************************
 */

/// Scheduling Planner Environment
struct sch_plan_env_tag
{
    struct co_list elt_list;
};


/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */

/// Scheduling Planner environment variable
__STATIC struct sch_plan_env_tag sch_plan_env;

/// Last value of the most significant clock bit (to detect clock wrap)
__STATIC bool time_msb;


/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/// Update all periodic activity offsets after a clock wrap
__STATIC void sch_plan_clock_wrap_offset_update(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(SCH_PLAN_PATCH_TYPE, SCH_PLAN_CLOCK_WRAP_OFFSET_UPDATE_FUNC_BIT);

    struct sch_plan_elt_tag *elt = (struct sch_plan_elt_tag *)co_list_pick(&sch_plan_env.elt_list);
    while (elt != NULL)
    {
        elt->offset = CO_MOD((elt->interval - CO_MOD((RWIP_MAX_CLOCK_TIME + 1), elt->interval) + elt->offset), elt->interval);
        elt = (struct sch_plan_elt_tag *)elt->hdr.next;
    }
}

/// Choose an appropriate offset for the event
__STATIC uint8_t sch_plan_offset_chk(struct sch_plan_chk_param *chk_param, struct sch_plan_elt_tag **conflict_elt)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(SCH_PLAN_PATCH_TYPE, SCH_PLAN_OFFSET_CHK_FUNC_BIT, uint8_t, chk_param, conflict_elt);

    uint8_t status = SCH_PLAN_ERROR_OK;
    struct sch_plan_elt_tag *scan = NULL;
    uint32_t offset = CO_MOD((chk_param->interval + chk_param->offset - chk_param->margin), chk_param->interval);
    uint32_t interval = chk_param->interval;
    uint32_t duration = chk_param->duration_min + 2 * chk_param->margin;
    uint16_t conhdl_ref = chk_param->conhdl_ref;
    bool current_time_msb = (rwip_time_get().hs & BT_CLOCK_MSB);

    // If a clock wrap has occurred, update all periodic activity offsets
    if (time_msb && !current_time_msb)
    {
        sch_plan_clock_wrap_offset_update();
    }
    time_msb = current_time_msb;

    // Parse all planning entries
    scan = (struct sch_plan_elt_tag *) co_list_pick(&sch_plan_env.elt_list);
    while (scan != NULL)
    {
        if (scan->conhdl_ref != conhdl_ref)
        {
            uint32_t event_duration = scan->duration_min + 2 * scan->margin;
            uint32_t newstart, newend, event_start, event_end;

            uint32_t min_interval = co_min(interval, scan->interval);
            uint32_t max_interval = co_max(interval, scan->interval);

            // Checks that the sum of the activity durations fits the minimum interval
            if ((event_duration + duration) > min_interval)
            {
                status = SCH_PLAN_ERROR_BW_FULL;
                break;
            }

            // If the intervals are not multiple or half-multiple
            if (CO_MOD(max_interval, min_interval) && CO_MOD(2 * max_interval, min_interval))
            {
                // Skip this activity and move on to the next one
                scan = (struct sch_plan_elt_tag *)scan->hdr.next;
                continue;
            }

            // Compute activity boundaries
            event_start = CO_MOD((scan->offset - scan->margin + min_interval), min_interval);
            event_end = event_start + event_duration - 1;
            newstart = CO_MOD(offset, min_interval);
            newend = newstart + duration - 1;

            // Checks whether the events overlap, taking into account wrap-around effects
            if (((newstart >= event_start) && (newstart <= event_end)) ||
                    // Check if end of event is in another event
                    ((newend >= event_start) && (newend <= event_end)) ||
                    // Check if event envelops another event
                    ((newstart <= event_start) && (newend >= event_end)) ||
                    //Check if the other event wraps around
                    ((event_end >= min_interval) && (newstart <= CO_MOD(event_end, min_interval))) ||
                    // Check if this event wraps around
                    ((newend >= min_interval) && (event_start <= CO_MOD(newend, min_interval))))
            {
                status = SCH_PLAN_ERROR_BW_FULL;
                break;
            }
        }

        // Proceed to the next element
        scan = (struct sch_plan_elt_tag *)scan->hdr.next;
    }

    // If conflict has been detected
    if ((status == SCH_PLAN_ERROR_BW_FULL) && (conflict_elt != NULL))
    {
        // Indicate the first element in the list that conflicted with the new one
        *conflict_elt = scan;
    }

    return (status);
}

__STATIC uint8_t sch_plan_offset_range_compute(struct sch_plan_req_param *req_param, bool duration_max)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(SCH_PLAN_PATCH_TYPE, SCH_PLAN_OFFSET_RANGE_COMPUTE_FUNC_BIT, uint8_t, req_param, duration_max);

    uint8_t status = SCH_PLAN_ERROR_OK;

    uint16_t interval = req_param->interval;
    uint16_t duration = duration_max ? req_param->duration_max : req_param->duration_min;

    // check that parameters are duration
    if ((interval == 0) || (duration == 0) || (req_param->offset_min > req_param->offset_max))
    {
        status = SCH_PLAN_ERROR_REJECTED;
    }
    else
    {
        // Iterate all registered interval item
        struct sch_plan_elt_tag *iter_item = (struct sch_plan_elt_tag *) co_list_pick(&sch_plan_env.elt_list);

        // Initial offset window from input parameters
        uint16_t offset = req_param->offset_min;
        uint16_t offset_window = req_param->offset_max - offset;

        // Compute end of event if new interval to insert is the reference offset time
        uint16_t new_end = duration - 1;

        // Main loop, parse all planning entries to find a possible offset range
        while ((iter_item != NULL) && (offset <= req_param->offset_max))
        {
            if (iter_item->conhdl != req_param->conhdl)
            {
                uint16_t item_start, item_end;
                uint8_t margin = (iter_item->conhdl_ref != req_param->conhdl_ref) * (iter_item->margin + req_param->margin);
                uint16_t max_interval = co_max(interval, iter_item->interval);
                uint16_t min_interval = co_min(interval, iter_item->interval);
                uint16_t iter_duration = duration_max ? iter_item->duration_max : iter_item->duration_min;

                // ********************************************************************************************************
                // Perform a folding of bandwidth according to minimum interval with the new activity to insert as timing reference
                // ********************************************************************************************************

                // Check that bandwidth to insert is possible
                if ((iter_duration + duration + margin) > min_interval)
                {
                    status = SCH_PLAN_ERROR_BW_FULL;
                    break;
                }

                // If the intervals are not multiple or half-multiple
                if (CO_MOD(max_interval, min_interval) && CO_MOD(2 * max_interval, min_interval))
                {
                    // Skip this activity and move on to the next one
                    iter_item = (struct sch_plan_elt_tag *)iter_item->hdr.next;
                    continue;
                }

                // Compute start and end time according to minimum interval and new offset as reference time
                item_start = CO_MOD((min_interval + iter_item->offset - margin - CO_MOD(offset, min_interval)), min_interval);
                item_end   = CO_MOD((item_start + iter_duration + 2 * margin - 1), min_interval);

                // Check if new event end is before event start
                if ((new_end >= item_start)
                        // Check if end of event is wrapping in
                        || (item_end < item_start))
                {
                    // Consider next possible offset after scan item required bandwidth
                    offset += item_end + 1;

                    // Check if there is enough room to place the requested duration
                    if ((offset + duration) < interval)
                    {
                        // Compute remaining room
                        offset_window = interval - duration - offset;

                        // Go back to the beginning of the list and repeat the checks using the new offset
                        iter_item  = (struct sch_plan_elt_tag *) co_list_pick(&sch_plan_env.elt_list);
                        continue;
                    }
                    else
                    {
                        // Not possible to insert the new activity
                        offset = interval;
                        break;
                    }
                }

                // ********************************************************************************************************
                // There is no collision with previous element
                // Idea is computing max bandwidth available for new activity according to other activities
                // min value between new_start and event_end
                // ********************************************************************************************************
                if ((offset_window + duration) > item_start)
                {
                    offset_window = item_start - duration;
                }
            }

            // Proceed to the next element
            iter_item = (struct sch_plan_elt_tag *)iter_item->hdr.next;
        }

        // If selected offset has exceed the window
        if (offset > req_param->offset_max)
        {
            status = SCH_PLAN_ERROR_BW_FULL;
        }
        else
        {
            req_param->offset_min = offset;
            req_param->offset_max = req_param->offset_min + offset_window;
        }
    }

    return (status);
}

__STATIC void sch_plan_interval_req(struct sch_plan_req_param *req_param)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(SCH_PLAN_PATCH_TYPE, SCH_PLAN_INTERVAL_REQ_FUNC_BIT, req_param);

    req_param->interval = req_param->interval_max;

    // Check if a range of possible intervals is provided, else do nothing
    if (req_param->interval_min < req_param->interval_max)
    {
        // Computed interval
        uint32_t comp_interval = 0xFFFFFFFF;
        uint32_t interval_min  = 0xFFFFFFFF;
        uint8_t nb_links = 0;
        struct sch_plan_elt_tag *scan = (struct sch_plan_elt_tag *) co_list_pick(&sch_plan_env.elt_list);

        // Find the minimal interval and the total number of links
        while (scan != NULL)
        {
            if (scan->conhdl != req_param->conhdl)
            {
                nb_links++;

                if (scan->interval < interval_min)
                {
                    interval_min = scan->interval;
                }
            }

            // Proceed to the next element
            scan = (struct sch_plan_elt_tag *)scan->hdr.next;
        }

        // If this is the only link, take preferred periodicity into account
        if (nb_links == 0)
        {
            if (req_param->pref_period != 0)
            {
                // Calculate a multiple of preferred periodicity
                comp_interval = /*floor*/(req_param->interval_max / req_param->pref_period);
                comp_interval *= req_param->pref_period;
            }
        }
        else if (req_param->interval_max >= interval_min)
        {
            // Calculate a multiple between existing and requested interval
            comp_interval  = /*floor*/(req_param->interval_max / interval_min);
            comp_interval *= interval_min;
        }
        else // interval_min > input_param->interval_max
        {
            uint32_t candidate = req_param->interval_max;
            bool found = false;

            while ((candidate >= req_param->interval_min) && (!found))
            {
                if (CO_MOD(interval_min, candidate) == 0)
                {
                    comp_interval = candidate;
                    found = true;
                }

                candidate -= 2;
            }
        }

        // Check if the computed interval matches the requirements
        if ((req_param->interval_min <= comp_interval) && (comp_interval <= req_param->interval_max))
        {
            req_param->interval = comp_interval;
        }
    }
}


/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void sch_plan_init(bool is_reset)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(SCH_PLAN_PATCH_TYPE, SCH_PLAN_INIT_FUNC_BIT, is_reset);
    time_msb = false;

    co_list_init(&sch_plan_env.elt_list);
}

void sch_plan_set(struct sch_plan_elt_tag *plan_elt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(SCH_PLAN_PATCH_TYPE, SCH_PLAN_SET_FUNC_BIT, plan_elt);

    struct sch_plan_elt_tag *conflict_elt;
    struct sch_plan_chk_param chk_param;

    chk_param.interval = plan_elt->interval;
    chk_param.duration_min = plan_elt->duration_min;
    chk_param.conhdl = plan_elt->conhdl;
    chk_param.conhdl_ref = plan_elt->conhdl_ref;
    chk_param.offset = plan_elt->offset;
    chk_param.margin = plan_elt->margin;

    // Before inserting the activity, check if it overlaps with another one
    if (sch_plan_offset_chk(&chk_param, &conflict_elt) == SCH_PLAN_ERROR_BW_FULL)
    {
        ASSERT_ERR(conflict_elt != NULL);

        // Check if an activity can be moved
        if ((plan_elt->cb_move != NULL) || (conflict_elt->cb_move != NULL))
        {
            struct sch_plan_elt_tag *elt_to_move = (plan_elt->cb_move != NULL) ? plan_elt : conflict_elt;

            // If both activities are movable choose the one with the higher mobility
            if ((plan_elt->cb_move != NULL) && (conflict_elt->cb_move != NULL)
                    && (plan_elt->mobility < conflict_elt->mobility))
            {
                elt_to_move = conflict_elt;
            }

            // Run the callback
            elt_to_move->cb_move(elt_to_move->conhdl);
        }
    }

    // Inserting the activity in the planning
    co_list_push_back(&sch_plan_env.elt_list, &plan_elt->hdr);
}

void sch_plan_rem(struct sch_plan_elt_tag *plan_elt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(SCH_PLAN_PATCH_TYPE, SCH_PLAN_REM_FUNC_BIT, plan_elt);

    // Extract the interval linked to this element
    co_list_extract(&sch_plan_env.elt_list, &plan_elt->hdr);
}

uint8_t sch_plan_req(struct sch_plan_req_param *req_param)
{
    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(SCH_PLAN_PATCH_TYPE, SCH_PLAN_REQ_FUNC_BIT, uint8_t, req_param);

    uint8_t status;

    // Select an interval
    sch_plan_interval_req(req_param);

    // Ensure the initial offset range is lower than the selected interval
    req_param->offset_min = co_min(req_param->offset_min, req_param->interval - 1);
    req_param->offset_max = co_min(req_param->offset_max, req_param->interval - 1);

    // Find the possible offset range (first consider the maximum duration)
    status = sch_plan_offset_range_compute(req_param, true);

    // If no suitable offset range has been found
    if (status != SCH_PLAN_ERROR_OK)
    {
        // Find the possible offset range (consider the minimum duration)
        status = sch_plan_offset_range_compute(req_param, false);
    }

    return (status);
}

uint8_t sch_plan_chk(struct sch_plan_chk_param *chk_param)
{
    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(SCH_PLAN_PATCH_TYPE, SCH_PLAN_CHK_FUNC_BIT, uint8_t, chk_param);

    uint8_t status;

    // Check activity
    status = sch_plan_offset_chk(chk_param, NULL);

    return (status);
}

void sch_plan_shift(uint16_t conhdl_ref, int16_t shift)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(SCH_PLAN_PATCH_TYPE, SCH_PLAN_SHIFT_FUNC_BIT, conhdl_ref, shift);

    // Update planning
    struct sch_plan_elt_tag *iter_item = NULL;
    struct sch_plan_elt_tag *conflict_elt = NULL;

    // Parse all planning entries
    iter_item = (struct sch_plan_elt_tag *) co_list_pick(&sch_plan_env.elt_list);
    while (iter_item != NULL)
    {
        // If the element is related to the reference connection handle
        if (iter_item->conhdl_ref == conhdl_ref)
        {
            // Apply the shift
            iter_item->offset = CO_MOD((iter_item->offset + shift), iter_item->interval);

            // If no conflict already been detected
            if (conflict_elt == NULL)
            {
                struct sch_plan_chk_param chk_param;
                chk_param.interval = iter_item->interval;
                chk_param.duration_min = iter_item->duration_min;
                chk_param.conhdl = iter_item->conhdl;
                chk_param.conhdl_ref = iter_item->conhdl_ref;
                chk_param.offset = iter_item->offset;
                chk_param.margin = iter_item->margin;

                // Check if the shifted element overlaps with another one
                if (sch_plan_offset_chk(&chk_param, &conflict_elt) == SCH_PLAN_ERROR_BW_FULL)
                {
                    ASSERT_ERR(conflict_elt != NULL);

                    // Check if an activity can be moved
                    if ((iter_item->cb_move != NULL) || (conflict_elt->cb_move != NULL))
                    {
                        struct sch_plan_elt_tag *elt_to_move = (iter_item->cb_move != NULL) ? iter_item : conflict_elt;

                        // If both activities are movable choose the one with the higher mobility
                        if ((iter_item->cb_move != NULL) && (conflict_elt->cb_move != NULL)
                                && (iter_item->mobility < conflict_elt->mobility))
                        {
                            elt_to_move = conflict_elt;
                        }

                        // Run the callback
                        elt_to_move->cb_move(elt_to_move->conhdl);
                    }
                }
            }
        }

        // Load next activity
        iter_item = (struct sch_plan_elt_tag *)iter_item->hdr.next;
    }
}

///@} SCH_PLAN
