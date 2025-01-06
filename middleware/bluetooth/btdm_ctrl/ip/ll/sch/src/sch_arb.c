/**
 ****************************************************************************************
 *
 * @file sch_arb.c
 *
 * @brief Scheduling Arbiter module
 *
 * Copyright (C) RivieraWaves 2009-2017
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup SCH_ARB main module
 * @ingroup SCH
 * @brief The SCH_ARB main module.
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

#include "rwip.h"
#include <string.h>          // For mem* functions
#include "arch.h"            // For asserts
#include "co_bt.h"
#include "co_math.h"
#include "co_list.h"
#include "ke_mem.h"
#include "sch_arb.h"
#include "ll.h"
#include "rwip.h"
#include "co_utils.h"

#include "dbg.h"
#include "btdm_patch.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/// Undefined timestamp value
#define SCH_ARB_UNDEF_TIME           (0xFFFFFFFF)

/// Maximum difference between 2 timestamps (half of the range)
#define SCH_ARB_MAX_INTERVAL_TIME    ((RWIP_MAX_CLOCK_TIME >> 1) - 1)

/// Margin needed for inserting an activity to the schedule (in half-us)
#define SCH_ARB_SCHED_ACT_MARGIN     (200)

/// Threshold for programming the HW timer (in half-us)
#define SCH_ARB_TIMER_PROG_THR    (500)


/*
 * ENUMERATIONS DEFINITIONS
 ****************************************************************************************
 */

/// Definitions of conflicts between time reservations
enum sch_arb_conflict
{
    START_BEFORE_END_BEFORE,
    START_BEFORE_END_DURING,
    START_BEFORE_END_AFTER,
    START_DURING_END_DURING,
    START_DURING_END_AFTER,
    START_AFTER_END_AFTER,
};


/*
 * STRUCT DEFINITIONS
 ****************************************************************************************
 */
/// Scheduling Arbiter Environment
struct sch_arb_env_tag
{
    /// List of pending element to program
    struct co_list elt_wait;
    /// element programmed
    struct sch_arb_elt_tag *elt_prog;
    /// List of element canceled
    struct co_list elt_canceled;
    /// Immediate scheduling
    bool immediate_sched;
};

/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */

/// Scheduling Arbiter environment variable
__STATIC struct sch_arb_env_tag sch_arb_env;

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

#if BT_EMB_PRESENT
/**
 ****************************************************************************************
 * @brief Return the next timestamp that is aligned with the required phase
 *
 * @param[in] clock  Clock expressed in half-slots
 * @param[in] phase  Phase (allowed range: 0 - 3)
 *
 * @return Next phase-aligned timestamp
 ****************************************************************************************
 */
__STATIC uint32_t sch_arb_phase_align_hi(uint32_t clock, uint8_t phase)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(SCH_ARB_PATCH_TYPE, SCH_ARB_PHASE_ALIGN_HI_FUNC_BIT, uint32_t, clock, phase);

    ASSERT_ERR(phase < 4);
    clock = ((clock & 0x03) <= phase) ? CO_ALIGN4_LO(clock) + phase : CO_ALIGN4_HI(clock) + phase;
    return (clock & RWIP_MAX_CLOCK_TIME);
}
#endif //BT_EMB_PRESENT


/**
 ****************************************************************************************
 * @brief Check conflicts between 2 events
 *
 * The function select the appropriate conflict type among the followings:
 *    - A finishes before B starts
 *    - A starts before B and finishes inside B (partial overlap)
 *    - A starts before B and finishes after B (B is included in A)
 *    - A starts and finishes during B (A is included in B)
 *    - A starts during B and finishes after B (partial overlap)
 *    - A starts after B finishes
 *
 * @param[in] evt_a  Event A
 * @param[in] evt_b  Event B
 *
 * @return conflict
 ****************************************************************************************
 */
__STATIC uint8_t sch_arb_conflict_check(struct sch_arb_elt_tag *evt_a, struct sch_arb_elt_tag *evt_b)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(SCH_ARB_PATCH_TYPE, SCH_ARB_CONFLICT_CHECK_FUNC_BIT, uint8_t, evt_a, evt_b);

    uint8_t conflict;

    /*
     * Terminology:
     *  sa => start A
     *  ea => end A
     *  sb => start B
     *  eb => end B
     */

    do
    {
        // Distance from start A to start B
        int32_t diff_sa_sb_hslot = CLK_DIFF(evt_a->time.hs, evt_b->time.hs);

        // A starts before B starts
        if ((diff_sa_sb_hslot > 0) || ((diff_sa_sb_hslot == 0) && (evt_a->time.hus < evt_b->time.hus)))
        {
            // Distance from end A to start B
            int32_t diff_ea_sb_hslot;
            int32_t diff_ea_sb_hus = evt_a->time.hus + evt_a->duration_min - evt_b->time.hus;
            if (diff_ea_sb_hus > 0)
            {
                diff_ea_sb_hslot = diff_sa_sb_hslot - ((diff_ea_sb_hus - 1) / HALF_SLOT_SIZE);
            }
            else
            {
                diff_ea_sb_hslot = diff_sa_sb_hslot - (diff_ea_sb_hus / HALF_SLOT_SIZE) + 1;
            }

            // A ends before B starts
            if (diff_ea_sb_hslot > 0)
            {
                conflict = START_BEFORE_END_BEFORE;
                break;
            }
            else
            {
                // Distance from end A to end B
                int32_t diff_ea_eb_hslot;
                int32_t diff_ea_eb_hus = evt_a->time.hus + evt_a->duration_min - evt_b->time.hus - evt_b->duration_min;
                if (diff_ea_eb_hus > 0)
                {
                    diff_ea_eb_hslot = diff_sa_sb_hslot - ((diff_ea_eb_hus - 1) / HALF_SLOT_SIZE);
                }
                else
                {
                    diff_ea_eb_hslot = diff_sa_sb_hslot - (diff_ea_eb_hus / HALF_SLOT_SIZE) + 1;
                }

                // A ends during B
                if (diff_ea_eb_hslot > 0)
                {
                    conflict = START_BEFORE_END_DURING;
                }
                else
                {
                    conflict = START_BEFORE_END_AFTER;
                }
                break;
            }
        }
        // B starts before A starts
        else
        {
            // Distance from start A to end B
            int32_t diff_sa_eb_hslot;
            int32_t diff_sa_eb_hus = evt_b->time.hus + evt_b->duration_min - evt_a->time.hus;
            if (diff_sa_eb_hus > 0)
            {
                diff_sa_eb_hslot = diff_sa_sb_hslot + ((diff_sa_eb_hus - 1) / HALF_SLOT_SIZE) + 1;
            }
            else
            {
                diff_sa_eb_hslot = diff_sa_sb_hslot + (diff_sa_eb_hus / HALF_SLOT_SIZE);
            }

            // A starts after B ends
            if (diff_sa_eb_hslot <= 0)
            {
                conflict = START_AFTER_END_AFTER;
                break;
            }
            else
            {
                // Distance from end A to end B
                int32_t diff_ea_eb_hslot;
                int32_t diff_ea_eb_hus = evt_a->time.hus + evt_a->duration_min - evt_b->time.hus - evt_b->duration_min;
                if (diff_ea_eb_hus > 0)
                {
                    diff_ea_eb_hslot = diff_sa_sb_hslot - ((diff_ea_eb_hus - 1) / HALF_SLOT_SIZE);
                }
                else
                {
                    diff_ea_eb_hslot = diff_sa_sb_hslot - (diff_ea_eb_hus / HALF_SLOT_SIZE) + 1;
                }

                // A ends during B
                if (diff_ea_eb_hslot > 0)
                {
                    conflict = START_DURING_END_DURING;
                }
                else
                {
                    conflict = START_DURING_END_AFTER;
                }
                break;
            }
        }
    }
    while (0);


    DBG_SWDIAG(SCH_ARB, CONFLICT, (((conflict != START_BEFORE_END_BEFORE) && (conflict != START_AFTER_END_AFTER)) ? 1 : 0));

    return conflict;
}

/**
 ****************************************************************************************
 * @brief Cancel elements
 ****************************************************************************************
 */
__STATIC void sch_arb_elt_cancel(struct sch_arb_elt_tag *new_elt, struct sch_arb_elt_tag *cancel_first, struct sch_arb_elt_tag *cancel_last)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(SCH_ARB_PATCH_TYPE, SCH_ARB_ELT_CANCEL_FUNC_BIT, new_elt, cancel_first, cancel_last);

    struct sch_arb_elt_tag *cancel_prev = NULL;
    struct sch_arb_elt_tag *cancel_curr = cancel_first;

    DBG_SWDIAG(SCH_ARB, ELT_CANCEL, 1);

    // Try to reschedule each element one-by-one
    while (cancel_curr)
    {
        uint8_t resched_att = SCH_ARB_ASAP_STG_RESCHED_ATT_GET(cancel_curr);

        struct sch_arb_elt_tag *cancel_next = (struct sch_arb_elt_tag *) co_list_next(&cancel_curr->hdr);

        // Check ASAP flag
        if ((SCH_ARB_ASAP_STG_TYPE_GET(cancel_curr) != SCH_ARB_FLAG_NO_ASAP) && (resched_att > 0))
        {
            uint8_t status = SCH_ARB_ERROR_OK;
            struct sch_arb_elt_tag *wait_curr = new_elt;
            struct sch_arb_elt_tag *wait_prev = NULL;

            // Increment priority
            cancel_curr->current_prio = RWIP_PRIO_ADD_2(cancel_curr->current_prio, SCH_ARB_ASAP_STG_PRIO_INC_GET(cancel_curr));

            // Decrement number of rescheduling attempts
            SCH_ARB_ASAP_STG_RESCHED_ATT_SET(cancel_curr, resched_att - 1);

            // Scan the wait list
            while (wait_curr)
            {
                // Compare element boundaries
                uint8_t conflict = sch_arb_conflict_check(cancel_curr, wait_curr);

                if (conflict == START_BEFORE_END_BEFORE)
                {
                    break;
                }
                else if (conflict == START_AFTER_END_AFTER)
                {
                    // do nothing
                }
                else
                {
#if BT_EMB_PRESENT
                    uint8_t phase;
#endif //BT_EMB_PRESENT

                    // Move the current element after the scan element
                    uint32_t shift_hus = (wait_curr->time.hus + wait_curr->duration_min);
                    uint32_t shift_hslot = shift_hus / HALF_SLOT_SIZE;
                    uint32_t new_timestamp = wait_curr->time.hs + shift_hslot;
                    if (cancel_curr->time.hus < (shift_hus - (shift_hslot * HALF_SLOT_SIZE)))
                    {
                        new_timestamp += 1;
                    }

#if BT_EMB_PRESENT
                    // Check phase
                    phase = SCH_ARB_ASAP_STG_PHASE_GET(cancel_curr);
                    if (phase != SCH_ARB_NO_PHASE)
                    {
                        new_timestamp = sch_arb_phase_align_hi(new_timestamp, phase);
                    }
#endif //BT_EMB_PRESENT

                    cancel_curr->time.hs = new_timestamp & RWIP_MAX_CLOCK_TIME;

                    // Check in ASAP case if the limit has been reached
                    if ((SCH_ARB_ASAP_STG_TYPE_GET(cancel_curr) >= SCH_ARB_FLAG_ASAP_LIMIT)
                            && (CLK_DIFF(cancel_curr->time.hs, cancel_curr->asap_limit) < ((int32_t)(cancel_curr->duration_min / HALF_SLOT_SIZE))))
                    {
                        status = SCH_ARB_ERROR_REJECTED;
                        break;
                    }
                }

                // Get next element from wait list
                wait_prev = wait_curr;
                wait_curr = (struct sch_arb_elt_tag *) co_list_next(&wait_curr->hdr);
            }

            if (status == SCH_ARB_ERROR_OK)
            {
                // If the element is the first
                if (cancel_prev == NULL)
                {
                    // Next element becomes the first one
                    cancel_first = cancel_next;
                }
                else
                {
                    // Re-link previous element to the next one
                    cancel_prev->hdr.next = (struct co_list_hdr *) cancel_next;
                }

                // If the element is the last
                if (cancel_curr == cancel_last)
                {
                    // Previous element becomes the last one
                    cancel_last = cancel_prev;
                }

                // Re-insert into the wait list
                co_list_insert_after(&sch_arb_env.elt_wait, &wait_prev->hdr, &cancel_curr->hdr);

                TRC_REQ_SCH_ARB_SHIFT(cancel_curr);
            }
            else
            {
                cancel_prev = cancel_curr;
            }
        }
        else
        {
            cancel_prev = cancel_curr;
        }

        // Move to next element from cancel list
        cancel_curr = cancel_next;
    }

    // Check if cancel list is still not empty
    if (cancel_first != NULL)
    {
        // Append the canceled elements to cancel list
        co_list_push_back_sublist(&sch_arb_env.elt_canceled, (struct co_list_hdr *) cancel_first, (struct co_list_hdr *) cancel_last);

        // request for a SW interrupt
        rwip_sw_int_req();

        TRC_REQ_SCH_ARB_CANC((struct sch_arb_elt_tag *) sch_arb_env.elt_canceled.first);
    }

    DBG_SWDIAG(SCH_ARB, ELT_CANCEL, 0);
}

/**
 ****************************************************************************************
 * @brief API to program a timer if needed
 ****************************************************************************************
 */
__STATIC void sch_arb_prog_timer(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(SCH_ARB_PATCH_TYPE, SCH_ARB_PROG_TIMER_FUNC_BIT);

    // Pick the element in the pending list
    struct sch_arb_elt_tag *first_elt = (struct sch_arb_elt_tag *)co_list_pick(&sch_arb_env.elt_wait);

    // Initialize default target as an invalid value
    rwip_time_t target = {SCH_ARB_UNDEF_TIME, 0, 0};

    // Get current time
    rwip_time_t immediate_sched_thr = rwip_time_get();

    DBG_SWDIAG(SCH_ARB, PROG_TIMER, 1);

    // Compute time after which the start is delayed with a timer
    immediate_sched_thr.hus += SCH_ARB_TIMER_PROG_THR;
    if (immediate_sched_thr.hus >= HALF_SLOT_SIZE)
    {
        immediate_sched_thr.hus -= HALF_SLOT_SIZE;
        immediate_sched_thr.hs = CLK_ADD_2(immediate_sched_thr.hs, 1);
    }

    // Check if list is empty
    if (first_elt != NULL)
    {
        // Get the start notification time of the 1st element in the pending list
        target.hs =  CLK_SUB(first_elt->time.hs, rwip_prog_delay);
        target.hus = first_elt->time.hus;

        // Check is prevent stop timer should be started
        if (sch_arb_env.elt_prog != NULL)
        {
            // Set stop notification for current activity in advance of the start time of the new activity (if necessary)
            target.hs = CLK_SUB(target.hs, sch_arb_env.elt_prog->stop_latency);
        }
    }

    // If no target
    if (target.hs == SCH_ARB_UNDEF_TIME)
    {
        // Disable timer
        rwip_timer_arb_set(RWIP_INVALID_TARGET_TIME, 0);
    }
    else
    {
        // Check if the target is possible
        if (CLK_LOWER_EQ_HUS(immediate_sched_thr.hs, immediate_sched_thr.hus, target.hs, target.hus))
        {
            // Program timer
            rwip_timer_arb_set(target.hs, target.hus);
        }
        else
        {
            // Disable timer
            rwip_timer_arb_set(RWIP_INVALID_TARGET_TIME, 0);

            // Use the SW interrupt to schedule immediately
            rwip_sw_int_req();
            sch_arb_env.immediate_sched = true;
        }
    }

    DBG_SWDIAG(SCH_ARB, PROG_TIMER, 0);
}

/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void sch_arb_init(bool is_reset)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(SCH_ARB_PATCH_TYPE, SCH_ARB_INIT_FUNC_BIT, is_reset);
    co_list_init(&sch_arb_env.elt_wait);
    co_list_init(&sch_arb_env.elt_canceled);

    sch_arb_env.elt_prog = NULL;
}

uint8_t sch_arb_insert(struct sch_arb_elt_tag *elt)
{
    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(SCH_ARB_PATCH_TYPE, SCH_ARB_INSERT_FUNC_BIT, uint8_t, elt);

    // List of element waiting for programming
    struct co_list *list = &sch_arb_env.elt_wait;
    struct sch_arb_elt_tag *prev = NULL;
    struct sch_arb_elt_tag *cancel_first = NULL;
    struct sch_arb_elt_tag *cancel_last = NULL;
    struct sch_arb_elt_tag *scan = NULL;
    // Returned status
    uint8_t status = SCH_ARB_ERROR_OK;
    // Number of canceled elements
    uint8_t cancel_cpt = 0;

    ASSERT_INFO((elt->duration_min > 0) && (elt->duration_min <= SCH_ARB_MAX_DURATION), elt->duration_min, 0);
    ASSERT_ERR(elt->time.hus < HALF_SLOT_SIZE);

    DBG_SWDIAG(SCH_ARB, INSERT, 1);

#if (TRACER_PRESENT && TRC_ARB)
    uint32_t timestamp = elt->time.hs;
#endif /*(TRACER_PRESENT && TRC_ARB)*/

    // Get current time
    rwip_time_t current_time = rwip_time_get();

    for (;;)
    {
        // Compare the TS requested and the 1st possible TS from now
        rwip_time_t first_allowed_ts;

        first_allowed_ts.hs = CLK_ADD_2(current_time.hs, rwip_prog_delay);
        first_allowed_ts.hus = current_time.hus + SCH_ARB_SCHED_ACT_MARGIN;

        if (first_allowed_ts.hus >= HALF_SLOT_SIZE)
        {
            uint8_t num_hs = first_allowed_ts.hus / HALF_SLOT_SIZE;
            first_allowed_ts.hus -= num_hs * HALF_SLOT_SIZE;
            first_allowed_ts.hs = CLK_ADD_2(first_allowed_ts.hs, num_hs);
        }

        if (sch_arb_env.elt_prog && (sch_arb_env.elt_prog->stop_latency != 0))
        {
            // Take the stop notification delay into account (delay to stop the ongoing activity)
            if (CLK_DIFF(current_time.hs, first_allowed_ts.hs) < (rwip_prog_delay + sch_arb_env.elt_prog->stop_latency))
            {
                first_allowed_ts.hs = CLK_ADD_2(current_time.hs, (rwip_prog_delay + sch_arb_env.elt_prog->stop_latency));
            }
        }

#if BT_EMB_PRESENT
        // Check parity
        if (SCH_ARB_ASAP_STG_TYPE_GET(elt) != SCH_ARB_FLAG_NO_ASAP)
        {
            uint8_t phase = SCH_ARB_ASAP_STG_PHASE_GET(elt);
            if (phase != SCH_ARB_NO_PHASE)
            {
                // Align first allowed time to the next half-slot boundary
                first_allowed_ts.hs = CLK_ADD_2(first_allowed_ts.hs, 1);
                first_allowed_ts.hus = 0;

                // Align first allowed time as per the phase
                first_allowed_ts.hs = sch_arb_phase_align_hi(first_allowed_ts.hs, phase);
            }
        }
#endif //BT_EMB_PRESENT

        if (CLK_GREATER_THAN_HUS(first_allowed_ts.hs, first_allowed_ts.hus, elt->time.hs, elt->time.hus))
        {
            // If the element should not be programmed ASAP
            if (SCH_ARB_ASAP_STG_TYPE_GET(elt) == SCH_ARB_FLAG_NO_ASAP)
            {
                status = SCH_ARB_ERROR_REJECTED;
                break;
            }
            else
            {
                // Set the element timestamp after the first allowed timestamp
                elt->time.hs  = first_allowed_ts.hs;
                if (elt->time.hus < first_allowed_ts.hus)
                {
                    elt->time.hs = CLK_ADD_2(first_allowed_ts.hs, 1);
                }
            }
        }
        // Check in ASAP case if the limit has been reached
        if ((SCH_ARB_ASAP_STG_TYPE_GET(elt) >= SCH_ARB_FLAG_ASAP_LIMIT)
                && (CLK_DIFF(elt->time.hs, elt->asap_limit) < ((int32_t)(elt->duration_min / HALF_SLOT_SIZE))))
        {
            status = SCH_ARB_ERROR_REJECTED;
            break;
        }
        scan = (sch_arb_env.elt_prog) ? sch_arb_env.elt_prog : (struct sch_arb_elt_tag *)list->first;
        // scan the list until the end or cmp() returns true
        while (scan)
        {
            // check that element to insert is not already present into wait list
            ASSERT_ERR(scan != elt);

            uint8_t conflict = sch_arb_conflict_check(elt, scan);

            if (conflict == START_BEFORE_END_BEFORE)
            {
                break;
            }
            else if (conflict == START_AFTER_END_AFTER)
            {
                // do nothing
            }
            else
            {
                // Check priority only if the scan element is not the programmed one
                if ((elt->current_prio > scan->current_prio) && (scan != sch_arb_env.elt_prog))
                {
                    // Save the first of the element(s) to remove
                    if (cancel_first == NULL)
                    {
                        cancel_first = scan;
                    }
                    // Save the last of the element(s) to remove
                    cancel_last = scan;
                    cancel_cpt++;

                    //If new element finishes before scanned element exit the loop
                    if ((conflict == START_BEFORE_END_DURING) || (conflict == START_DURING_END_DURING))
                        break;
                }
                else
                {
                    // Check ASAP flag
                    if (SCH_ARB_ASAP_STG_TYPE_GET(elt) != SCH_ARB_FLAG_NO_ASAP)
                    {
                        uint32_t shift_hus = (scan->time.hus + scan->duration_min);
                        uint32_t shift_hslot = shift_hus / HALF_SLOT_SIZE;
                        uint32_t new_timestamp = scan->time.hs + shift_hslot;
                        if (elt->time.hus < (shift_hus - (shift_hslot * HALF_SLOT_SIZE)))
                        {
                            new_timestamp += 1;
                        }

#if BT_EMB_PRESENT
                        // Check parity
                        uint8_t phase = SCH_ARB_ASAP_STG_PHASE_GET(elt);
                        if (phase != SCH_ARB_NO_PHASE)
                        {
                            new_timestamp = sch_arb_phase_align_hi(new_timestamp, phase);
                        }
#endif //BT_EMB_PRESENT

                        elt->time.hs = new_timestamp & RWIP_MAX_CLOCK_TIME;

                        //If the element can be reprogrammed ASAP and we have already taken the decision to cancel some elements
                        //Do not cancel those elements
                        cancel_first = NULL;
                        cancel_last  = NULL;
                        cancel_cpt   = 0;
                    }
                    // element in conflict already started, check priorities
                    else if ((elt->current_prio > scan->current_prio) && (scan == sch_arb_env.elt_prog)
                             && ((conflict == START_DURING_END_DURING) || (conflict == START_DURING_END_AFTER)))
                    {
                        // nothing to do, let HW automatically abort previous event
                    }
                    else
                    {
                        status = SCH_ARB_ERROR_REJECTED;
                        break;
                    }
                }
            }

            // Check in ASAP case if the limit has been reached
            if ((SCH_ARB_ASAP_STG_TYPE_GET(elt) >= SCH_ARB_FLAG_ASAP_LIMIT)
                    && (CLK_DIFF(elt->time.hs, elt->asap_limit) < ((int32_t)(elt->duration_min / HALF_SLOT_SIZE))))
            {
                status = SCH_ARB_ERROR_REJECTED;
                break;
            }

            // Is the scanned element is in the wait list
            if (scan != sch_arb_env.elt_prog)
            {
                //If elements should be canceled do not update the previous to avoid issue during insertion
                if (cancel_cpt == 0)
                {
                    prev = scan;
                }

                // Jump to next element in wait list
                scan = (struct sch_arb_elt_tag *) co_list_next(&scan->hdr);
            }
            else // If the scanned element was the programmed one, check the wait list
            {
                scan = (struct sch_arb_elt_tag *) co_list_pick(list);
            }
        }
        break;
    }

    if (status != SCH_ARB_ERROR_REJECTED)
    {
        if (cancel_first)
        {
            // Remove the canceled elements from wait list
            co_list_extract_sublist(&sch_arb_env.elt_wait, (struct co_list_hdr *) prev, (struct co_list_hdr *) cancel_last);
        }

        // Check the position of the new event
        if (prev)
        {
            // Second or more
            co_list_insert_after(&sch_arb_env.elt_wait, &prev->hdr, &elt->hdr);
        }
        else
        {
            // First
            co_list_push_front(&sch_arb_env.elt_wait, &elt->hdr);

            // Reprogram timer if needed
            sch_arb_prog_timer();
        }

        // Check if some events have been removed from schedule
        if (cancel_first)
        {
            // Reschedule or cancel the events
            sch_arb_elt_cancel(elt, cancel_first, cancel_last);
        }
    }

    TRC_REQ_SCH_ARB_INSERT(timestamp, status, elt);

    DBG_SWDIAG(SCH_ARB, INSERT_ERR, ((status != SCH_ARB_ERROR_OK) ? 1 : 0));
    DBG_SWDIAG(SCH_ARB, INSERT, 0);

    return (status);
}


uint8_t sch_arb_remove(struct sch_arb_elt_tag *elt, bool not_waiting)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(SCH_ARB_PATCH_TYPE, SCH_ARB_REMOVE_FUNC_BIT, uint8_t, elt, not_waiting);

    // Returned status
    uint8_t status = SCH_ARB_ERROR_OK;

    DBG_SWDIAG(SCH_ARB, REMOVE, 1);

    // If the element is not NULL
    if (elt)
    {
        TRC_REQ_SCH_ARB_REM(elt);

        // Check the wait queue first
        do
        {
            // check if the element is the current in the ea
            if (elt == sch_arb_env.elt_prog)
            {
                sch_arb_env.elt_prog = NULL;
                break;
            }

            // If activity is not waiting, no need to search in the schedule
            if (not_waiting)
                break;

            // If the element is the first and the fine target timer is started, stop it
            if (&elt->hdr == co_list_pick(&sch_arb_env.elt_wait))
            {
                // Pop the first element of the list
                co_list_pop_front(&sch_arb_env.elt_wait);

                // Update timer if needed
                sch_arb_prog_timer();

                break;
            }

            // Try extracting the element from the wait queue
            if (co_list_extract(&sch_arb_env.elt_wait, &elt->hdr))
                break;

            // Try extracting the element from the cancel queue
            if (co_list_extract(&sch_arb_env.elt_canceled, &elt->hdr))
                break;

            // Element has not been found
            status = SCH_ARB_ERROR_NOT_FOUND;
        }
        while (0);
    }

    DBG_SWDIAG(SCH_ARB, REMOVE, 0);

    return (status);
}

void sch_arb_event_start_isr(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(SCH_ARB_PATCH_TYPE, SCH_ARB_EVENT_START_ISR_FUNC_BIT);

    // Get next element
    struct sch_arb_elt_tag *next_elt = (struct sch_arb_elt_tag *)co_list_pick(&sch_arb_env.elt_wait);
    rwip_time_t current_time = rwip_time_get();

    DBG_SWDIAG(SCH_ARB, EVT_START, 1);

    // Potentially stop the current event
    if ((sch_arb_env.elt_prog) && (next_elt))
    {
        struct sch_arb_elt_tag *current_elt = sch_arb_env.elt_prog;

        // Check if it is time to notify the current event to stop
        if (CLK_DIFF(current_time.hs, next_elt->time.hs) <= (rwip_prog_delay + current_elt->stop_latency))
        {
            // Remove the current element
            sch_arb_env.elt_prog = NULL;

            if (current_elt->cb_stop != NULL)
            {
                // Call back the end of schedule function
                current_elt->cb_stop(current_elt);
            }
        }
    }

    /*
     * ____________________________________________________________________________________
     *                    Tn-D-X       Tn-D        Tn-Y                    Tn
     *      prog timer       |         start         |       cancel
     * ______________________|_______________________|_____________________________________
     *                                   '--------------------------------->|\\\\activity\\
     * Tn is the effective start of the activity
     * Tn-D is the expected notification of activity start (programming delay)
     * Tn-Y is the last possible time for notifying the activity to start
     * Tn-D+X is the earliest possible time for notifying the activity to start
     *
     * For each event in the schedule, the action depends on current time:
     *
     *   - Before Tn-D-X:
     *      - No action from this ISR, a new timer is programmed closer to activity start
     *
     *   - Between Tn-D-X and Tn-Y:
     *      - The activity is notified to start
     *
     *   - After Tn-Y:
     *      - The activity is canceled
     */

    // Check if too late to program the activity
    if (next_elt != NULL)
    {
        // The limit is a fixed margin before the element start time
        uint32_t thr_hs = next_elt->time.hs;
        int32_t thr_hus = next_elt->time.hus - ((IP_PREFETCHABORT_TIME_US << 1) + SLEEP_PROG_MARGIN);
        if (thr_hus < 0)
        {
            uint8_t num_hs = CO_DIVIDE_CEIL((-thr_hus), HALF_SLOT_SIZE);
            thr_hs = CLK_SUB(thr_hs, num_hs);
            thr_hus += num_hs * HALF_SLOT_SIZE;
        }

        // Check that the current time has not over-stepped the limit
        if (CLK_GREATER_THAN_HUS(current_time.hs,
                                 current_time.hus,
                                 thr_hs,
                                 thr_hus))
        {
            // Uncomment for debugging
            //ASSERT_INFO(0, (current_time.hs - thr_hs), (current_time.hus - thr_hus));

            // Pop the element
            next_elt = (struct sch_arb_elt_tag *)co_list_pop_front(&sch_arb_env.elt_wait);

            // Push element in the canceled list
            co_list_push_back(&sch_arb_env.elt_canceled, &next_elt->hdr);

            //Pick the next element
            next_elt = (struct sch_arb_elt_tag *)co_list_pick(&sch_arb_env.elt_wait);

            if (next_elt != NULL)
            {
                // Check that the current time is no more than one half slot later than the targeted start notification time
                if (CLK_GREATER_THAN_HUS(current_time.hs,
                                         current_time.hus,
                                         CLK_SUB(next_elt->time.hs, rwip_prog_delay - 1),
                                         next_elt->time.hus))
                {
                    // Should not happen that the start instants of 2 events are passed
                    ASSERT_ERR_FORCE(0);
                }
            }
        }
    }

    // Check if the activity should be programmed soon
    if (next_elt != NULL)
    {
        // The earliest time to notify a start is 2 half-slots before the targeted activity start
        if (CLK_LOWER_EQ_HUS(CLK_SUB(next_elt->time.hs, rwip_prog_delay + 1),
                             next_elt->time.hus,
                             current_time.hs,
                             current_time.hus))
        {
            // Pop the element
            next_elt = (struct sch_arb_elt_tag *)co_list_pop_front(&sch_arb_env.elt_wait);

            // If a current event is still present, it is notified to stop (before the start notification of the waiting one)
            if (sch_arb_env.elt_prog != NULL)
            {
                if (sch_arb_env.elt_prog->cb_stop != NULL)
                {
                    // Call back the end of schedule function
                    sch_arb_env.elt_prog->cb_stop(sch_arb_env.elt_prog);
                }
            }

            //Set the current element programmed
            sch_arb_env.elt_prog = next_elt;

            if (next_elt->cb_start != NULL)
            {
                TRC_REQ_SCH_ARB_START(next_elt);

                // Call back the end of schedule function
                next_elt->cb_start(next_elt);
            }

            // Check next waiting activity, must not be earlier than the programmed one
            next_elt = (struct sch_arb_elt_tag *)co_list_pick(&sch_arb_env.elt_wait);
            ASSERT_ERR((next_elt == NULL) || CLK_LOWER_EQ(sch_arb_env.elt_prog->time.hs, next_elt->time.hs));
        }
    }

    // Check if element(s) has(have) been canceled
    if (!co_list_is_empty(&sch_arb_env.elt_canceled))
    {
        // request for a SW interrupt
        rwip_sw_int_req();
    }

    // Check if the base time timer should be started
    sch_arb_prog_timer();

    DBG_SWDIAG(SCH_ARB, EVT_START, 0);
}


void sch_arb_sw_isr(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(SCH_ARB_PATCH_TYPE, SCH_ARB_SW_ISR_FUNC_BIT);

    DBG_SWDIAG(SCH_ARB, SW_ISR, 1);

    // If an activity is scheduled immediately
    if (sch_arb_env.immediate_sched)
    {
        // Clear immediate scheduling indicator
        sch_arb_env.immediate_sched = false;

        // Start activity
        sch_arb_event_start_isr();
    }

    // Notify each element about cancellation
    while (!co_list_is_empty(&sch_arb_env.elt_canceled))
    {
        // Pop the element in the pending list
        struct sch_arb_elt_tag *elt = (struct sch_arb_elt_tag *)co_list_pop_front(&sch_arb_env.elt_canceled);

        ASSERT_ERR(elt != NULL);

        if (elt->cb_cancel != NULL)
        {
            TRC_REQ_SCH_ARB_CANC(elt);

            // Call back the cancel function
            elt->cb_cancel(elt);
        }
    }

    DBG_SWDIAG(SCH_ARB, SW_ISR, 0);
}

///@} SCH_ARB
