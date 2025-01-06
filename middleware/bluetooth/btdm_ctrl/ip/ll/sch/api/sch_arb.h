/**
****************************************************************************************
*
* @file SCH_ARB.h
*
* @brief SCH_ARB main module
*
* Copyright (C) RivieraWaves 2009-2017
*
*
****************************************************************************************
*/

#ifndef _SCH_ARB_H_
#define _SCH_ARB_H_

/**
 ****************************************************************************************
 * @addtogroup SCH
 * @brief Entry points of the Scheduling Arbiter module
 *
 * This module contains the primitives that allow stacks to schedule an event or frame.
 *
 * @{
 ****************************************************************************************
 */

#include "rwip_config.h"

#include <stdint.h>               // Standard integer definitions
#include <stdbool.h>              // Standard boolean definitions
#include "co_list.h"              // List management functions definitions
#include "rwip.h"                 // For rwip_time_t


/*
 * MACROS
 ****************************************************************************************
 */



/*
 * DEFINES
 ****************************************************************************************
 */

/// Default BW 2 slots
#define SCH_ARB_BW_USED_DFT_SLOT         (2)
#define SCH_ARB_BW_USED_DFT_US           (SCH_ARB_BW_USED_DFT_SLOT*SLOT_SIZE)


/// Set ASAP settings
#define SCH_ARB_ASAP_STG_SET(evt, type, phase, resched_att, prio_inc)  evt->asap_settings = ( (((type) << 14) & 0xC000) | (((phase) << 11) & 0x3800) | (((resched_att) << 6) & 0x03C0) | (((prio_inc) << 0) & 0x003F) );
/// Get/Set type from ASAP settings
#define SCH_ARB_ASAP_STG_TYPE_GET(evt)                      ((uint8_t)((evt->asap_settings & 0xC000) >> 14))
#define SCH_ARB_ASAP_STG_TYPE_SET(evt, type)                ( evt->asap_settings = ((evt->asap_settings & ~(0xC000)) | (((type) << 14) & 0xC000)) )
/// Get phase from ASAP settings
#define SCH_ARB_ASAP_STG_PHASE_GET(evt)                    ((uint8_t)((evt->asap_settings & 0x3800) >> 11))
/// Get/Set rescheduling attempts from ASAP settings
#define SCH_ARB_ASAP_STG_RESCHED_ATT_GET(evt)               ((uint8_t)((evt->asap_settings & 0x03C0) >> 6))
#define SCH_ARB_ASAP_STG_RESCHED_ATT_SET(evt, resched_att)  ( evt->asap_settings = ((evt->asap_settings & ~(0x03C0)) | (((resched_att) << 6) & 0x03C0)) )
/// Get priority increment from ASAP settings
#define SCH_ARB_ASAP_STG_PRIO_INC_GET(evt)                  ((uint8_t)((evt->asap_settings & 0x003F) >> 0))

/// Maximum SCH_ARB element duration in half-us
#define SCH_ARB_MAX_DURATION               (400000)


/*
 * DEFINITIONS
 ****************************************************************************************
 */

/// SCH_ARB error codes
enum sch_arb_error
{
    /// SCH_ARB request succeed
    SCH_ARB_ERROR_OK                 = 0,
    /// SCH_ARB request rejected error
    SCH_ARB_ERROR_REJECTED,
    /// SCH_ARB element not found error
    SCH_ARB_ERROR_NOT_FOUND,
    /// SCH_ARB request rejected due to bandwidth full error
    SCH_ARB_ERROR_BW_FULL
};

/// ASAP type definition
/*@TRACE*/
enum sch_arb_elt_asap_type
{
    /// 00: No ASAP
    SCH_ARB_FLAG_NO_ASAP                = 0,
    /// 01: ASAP no limit
    SCH_ARB_FLAG_ASAP_NO_LIMIT,
    /// 10: ASAP with limit
    SCH_ARB_FLAG_ASAP_LIMIT,
    SCH_ARB_FLAG_MAX
};

/// ASAP slot parity definition
enum sch_arb_elt_asap_phase
{
    SCH_ARB_PHASE_0,
    SCH_ARB_PHASE_1,
    SCH_ARB_PHASE_2,
    SCH_ARB_PHASE_3,
    SCH_ARB_NO_PHASE,
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */



/// Scheduling Arbiter Element
/*@TRACE*/
struct sch_arb_elt_tag
{
    /// List element for chaining in the Even Arbiter lists
    struct co_list_hdr hdr;

    /// Programming time expressed in half-slots and half-us
    rwip_time_t time;

    /// Scheduling time limit in base time (half-slots) (only for ASAP LIMIT requests)
    uint32_t asap_limit;

    /// Minimum duration of the event or frame (in half-us)
    uint32_t duration_min;

    /**
     * ASAP settings field
     * bit |15  14|13  12  11|     10      |    9..6     |   5..0   |
     * def | TYPE |   Phase  | To protect  | Resched att | Prio inc |
     *
     * Type:
     *  - 00: No ASAP
     *  - 01: ASAP no limit
     *  - 10: ASAP with limit
     *  - 11: ASAP with limit, no parity check
     *
     * Phase: (only for ASAP requests)
     *  - 0: phase 0
     *  - 1: phase 1
     *  - 2: phase 2
     *  - 3: phase 3
     *  - 4: don't care
     *
     * Number of rescheduling attempts:
     *  - The remaining number of rescheduling attempts.
     *  - Rescheduling happens when the event is overlapped by a higher priority event
     *  - Only used for ASAP requests
     *
     * Priority increment:
     *  - The current priority value is incremented each time the event is overlapped by a new insertion and postponed
     *  - Only used for ASAP requests
     */
    /*@trc_desc
     *bit |15..14|  13..11  |  10  |    9..6     |   5..0   |
     *def | Type |   Phase  | Rsvd | Resched_att | Prio_inc |
     *
     *Phase: only if Type = 01..11
     *Resched_att: only if Type = 01..11
     *Prio_inc: only if Type = 01..11
     *
     *@trc_ref Type:
     *  - 00: No ASAP
     *  - 01: ASAP no limit
     *  - 10: ASAP with limit
     *  - 11: ASAP with limit and no parity check
     *
     *@trc_ref Phase:
     *  - 000: phase 0
     *  - 001: phase 1
     *  - 010: phase 2
     *  - 011: phase 3
     *  - 100: don-t care
     */
    uint16_t asap_settings;

    /// Current priority
    uint8_t current_prio;
    /// Latency to notify to stop the activity before next activity is notified to start (in half-slots, 0 if no stop required)
    uint8_t stop_latency;

    /************************************************************************************
     * ISR CALLBACKS
     ************************************************************************************/

    /// Start notification call back function
    void (*cb_start)(struct sch_arb_elt_tag *);
    /// Stop notification call back function
    void (*cb_stop)(struct sch_arb_elt_tag *);
    /// Cancel notification call back function
    void (*cb_cancel)(struct sch_arb_elt_tag *);
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
 * @brief Initialize the Scheduling Arbiter.
 *
 * @param[in] is_reset  True if reset is requested, false for an initialization
 ****************************************************************************************
 */
void sch_arb_init(bool is_reset);

/**
 ****************************************************************************************
 * @brief Insert a new reservation in the schedule
 *
 * @param[in] elt           Pointer to the element to be inserted
 *
 * @return No error if element is inserted.
 ****************************************************************************************
 */
uint8_t sch_arb_insert(struct sch_arb_elt_tag *elt);

/**
 ****************************************************************************************
 * @brief Remove a reservation
 *
 * @param[in] elt           Pointer to the element to be removed from current ptr
 * @param[in] not_waiting   True: activity is already started | False: may be waiting or not
 *
 ****************************************************************************************
 */
uint8_t sch_arb_remove(struct sch_arb_elt_tag *elt, bool not_waiting);

/**
 ****************************************************************************************
 * @brief Start of event/frame interrupt handler
 *
 * This function is called under interrupt when a start of event/frame interrupt is
 * generated by the BLE/BT core.
 *
 ****************************************************************************************
 */
void sch_arb_event_start_isr(void);


/**
 ****************************************************************************************
 * @brief Software interrupt handler
 *
 * This function is called under interrupt when a SW interrupt is generated by the BLE/BT
 * core.
 *
 ****************************************************************************************
 */
void sch_arb_sw_isr(void);


///@} SCH_ARB

#endif // _SCH_ARB_H_
