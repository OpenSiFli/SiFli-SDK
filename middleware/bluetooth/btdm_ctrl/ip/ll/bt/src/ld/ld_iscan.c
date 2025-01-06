/**
****************************************************************************************
*
* @file ld_iscan.c
*
* @brief LD Inquiry Scan source code
*
* Copyright (C) RivieraWaves 2009-2015
*
****************************************************************************************
*/

/**
 ****************************************************************************************
 * @addtogroup LDISCAN
 * @ingroup LD
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"    // stack configuration

#include <string.h>

#include "co_utils.h"
#include "co_math.h"

#include "arch.h"
#include "ke_mem.h"
#include "rwip.h"

#include "ld.h"             // link driver API
#include "ld_util.h"        // link driver utilities
#include "ld_int.h"         // link driver internal

#include "dbg.h"

#include "sch_arb.h"            // Scheduling Arbiter
#include "sch_prog.h"           // Scheduling Programmer
#include "sch_slice.h"          // Scheduling Slicer

#include "reg_btcore.h"         // BT core registers
#include "reg_em_bt_cs.h"       // BT EM Control Structure
#include "reg_em_bt_txdesc.h"   // BT EM TX descriptors
#if (EAVESDROPPING_SUPPORT)
    #include "lm.h"
#endif // EAVESDROPPING_SUPPORT
#include "btdm_patch.h"


/*
 * DEFINES
 *****************************************************************************************
 */

/// Minimum scheduling delay threshold where SW tries to advance the schedule (in BT slots)
#define LD_ISCAN_EVT_DELAY_RESCHED_MIN        30

/// Inquiry scan event automatic rescheduling attempts
#define LD_ISCAN_EVT_AUTO_RESCHED_ATT   4

/// MAX_RAND value used for random back-off
#define LD_ISCAN_MAX_RAND            1023

/// Inquiry Scan event states
enum ISCAN_EVT_STATE
{
    ISCAN_EVT_WAIT,
    ISCAN_EVT_ACTIVE,
    ISCAN_EVT_END,
};

/// Interlace Scan state
enum iscan_inter_state
{
    ISCAN_INTER_WIN0,
    ISCAN_INTER_WIN1,
    ISCAN_INTER_OFF,
};

#if (EAVESDROPPING_SUPPORT)
/// Segmented Scan states
enum SEG_SCAN_STATE
{
    SEG_SCAN_OFF,
    SEG_SCAN_ENABLING,
    SEG_SCAN_ON,
    SEG_SCAN_DISABLING,
};
#endif //EAVESDROPPING_SUPPORT

/*
 * STRUCTURE DEFINITION
 *****************************************************************************************
 */

/// LD ISCAN EIR data information
struct ld_iscan_eir_info
{
    /**
     * FEC_Required
     *
     * 0x00 FEC is not required.
     * 0x01 FEC is required.
     */
    uint8_t fec_req;

    ///Extended_Inquiry_Response length in bytes
    uint8_t eir_len;

    /// Extended_Inquiry_Response packet type
    uint8_t eir_pkt_type;
};

#if (EAVESDROPPING_SUPPORT)
/// LD ISCAN Segmented Scan environment structure
struct ld_iscan_seg_scan_env_tag
{
    /// Segmented scan state (off, enabling, on, disabling)
    uint8_t seg_scan_state;

    /// Segments counter
    uint8_t seg_cnt;

    /// Timestamp of the first scan segment (in 625us slots)
    uint32_t first_seg_ts;

    /// Segmented scan parameters
    struct ld_seg_scan_params seg_scan;
};
#endif // EAVESDROPPING_SUPPORT

/// LD ISCAN environment structure
struct ld_iscan_env_tag
{
    /// Inquiry scan event
    struct sch_arb_elt_tag evt;

    /// Remaining slots in current scan window (in 625us BT slots)
    uint32_t win_rem_slots;
    /// LAP to be used for the access code construction (GIAC or DIAC)
    struct lap lap;
    /// State
    uint8_t state;
    /// Amount of time between consecutive inquiry scans in slots (625 us)
    uint16_t iscan_intv;
    /// Amount of time for the duration of the inquiry scan in slots (625 us)
    uint16_t iscan_win;
    /// Interlace scan state (1st window, 2nd window, or no interlace scan)
    uint8_t interlace_state;
    /// Page scan repetition mode
    uint8_t pscan_rep_mode;
    /// Phase in the inquiry scan hop sequence (BB:8.4.3)
    uint8_t phase_offset;
};

/*
 * VARIABLE DEFINITION
 *****************************************************************************************
 */

/// LD ISCAN EIR data information
__STATIC struct ld_iscan_eir_info ld_iscan_eir;

/// LD ISCAN environment variable
__STATIC struct ld_iscan_env_tag *ld_iscan_env;

#if (EAVESDROPPING_SUPPORT)
    /// LD ISCAN Segmented Scan environment variable
    __STATIC struct ld_iscan_seg_scan_env_tag ld_iscan_seg_scan_env;
#endif // EAVESDROPPING_SUPPORT

#if RW_BT_MWS_COEX // Generalized Interlaced Scan Support
    /// bit mask for X input to inquiry scan of MWS occupied channels
    __STATIC uint32_t ld_iscan_mwscoex_xi_mask;
#endif // RW_BT_MWS_COEX

/*
 * LOCAL FUNCTIONS DECLARATION
 *****************************************************************************************
 */

__STATIC void ld_iscan_frm_cbk(uint32_t timestamp, uint32_t dummy, uint8_t irq_type);
#if RW_BT_MWS_COEX
    __STATIC uint8_t ld_iscan_mwscoex_xi_get(uint8_t xi_nom);
#endif //RW_BT_MWS_COEX


/*
 * LOCAL FUNCTIONS DEFINITION
 *****************************************************************************************
 */

#if (EAVESDROPPING_SUPPORT)
/**
 ****************************************************************************************
 * @brief Set timestamp and offset of the next scan segment
 ****************************************************************************************
 */
__STATIC void ld_iscan_set_seg_scan_timings(struct sch_arb_elt_tag *evt)
{
    uint32_t clk_offset = 0;
    uint16_t bit_offset = 0;
    int16_t diff = 0;
    uint32_t timestamp_piconet = evt->time.hs;

    // Check if the BD address corresponds to an active connection
    uint8_t link_id = lm_find_link_id(ld_iscan_seg_scan_env.seg_scan.bd_addr);

    if (link_id < MAX_NB_ACTIVE_ACL)
    {
        // Get the timings of the link
        ld_acl_timing_info_get(link_id, &clk_offset, &bit_offset);
        timestamp_piconet = CLK_ADD_2(timestamp_piconet, clk_offset);

        // Increase priority
        evt->current_prio = rwip_priority[RWIP_PRIO_ISCAN_DFT_IDX].value;
    }

    diff = ld_iscan_seg_scan_env.seg_scan.time_offset - (timestamp_piconet % ld_iscan_seg_scan_env.seg_scan.periodicity);

    // Apply the microsecond alignment
    diff += ld_iscan_seg_scan_env.seg_scan.hus_offset / HALF_SLOT_SIZE;
    int16_t delay = bit_offset + (ld_iscan_seg_scan_env.seg_scan.hus_offset % HALF_SLOT_SIZE);

    // Adjust delay and timestamp if needed
    if (delay < 0)
    {
        delay += HALF_SLOT_SIZE;
        diff--;
    }
    else if (delay >= HALF_SLOT_SIZE)
    {
        delay -= HALF_SLOT_SIZE;
        diff++;
    }

    // Adjust delay forward if negative
    if (diff < 0)
    {
        diff = ld_iscan_seg_scan_env.seg_scan.periodicity - CO_MOD((-diff), ld_iscan_seg_scan_env.seg_scan.periodicity);
    }

    evt->time.hs = CLK_ADD_2(evt->time.hs, diff);
    evt->time.hus = (uint16_t)delay;
}

/**
 ****************************************************************************************
 * @brief Schedule segmented inquiry scan
 ****************************************************************************************
 */
__STATIC void ld_iscan_seg_scan_sched(void)
{
    // Point to parameters
    struct sch_arb_elt_tag *evt = &(ld_iscan_env->evt);
    struct ld_iscan_env_tag *iscan_par = ld_iscan_env;

    switch (ld_iscan_seg_scan_env.seg_scan_state)
    {
    case SEG_SCAN_ENABLING:
    {
        evt->time.hs = rwip_time_get().hs;

        // Set the first segment timings
        ld_iscan_set_seg_scan_timings(evt);

        if (ld_iscan_seg_scan_env.seg_scan.segments > 1)
        {
            // Time from the first segment until the next toggling of CLKN[12]
            uint16_t time_to_toggling = BIT11 - (CLK_ADD_2(evt->time.hs, ld_iscan_seg_scan_env.seg_scan.distance) % BIT11);
            if (time_to_toggling < ld_iscan_seg_scan_env.seg_scan.segments * ld_iscan_seg_scan_env.seg_scan.distance)
            {
                // Not enough space to fit all segments before the next toggling:
                // Schedule the first segment to occur shortly after the next toggling
                uint16_t align = time_to_toggling % ld_iscan_seg_scan_env.seg_scan.periodicity;
                evt->time.hs = CLK_ADD_3(evt->time.hs, time_to_toggling, align ? (ld_iscan_seg_scan_env.seg_scan.periodicity - align) : 0);
            }
        }

        ld_iscan_seg_scan_env.first_seg_ts = evt->time.hs;

        // Set the event as strictly periodic
        SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_NO_ASAP, SCH_ARB_NO_PHASE, 0, 0);

        // Set the event duration including scan window and time for receiving inquiry report
        evt->duration_min = ld_iscan_seg_scan_env.seg_scan.duration + 2 * HALF_SLOT_SIZE;

        // Initialize segment counter
        ld_iscan_seg_scan_env.seg_cnt = 0;

        // Set as active
        ld_iscan_seg_scan_env.seg_scan_state = SEG_SCAN_ON;

    } // No break

    case SEG_SCAN_ON:
    {
        // Schedule the next window
        do
        {
            // Increment segment counter
            if (ld_iscan_seg_scan_env.seg_cnt++ == (ld_iscan_seg_scan_env.seg_scan.segments - 1))
            {
                // Reset segment counter
                ld_iscan_seg_scan_env.seg_cnt = 0;

                // Go to next scan interval
                ld_iscan_seg_scan_env.first_seg_ts = CLK_ADD_2(ld_iscan_seg_scan_env.first_seg_ts, iscan_par->iscan_intv);
                evt->time.hs = ld_iscan_seg_scan_env.first_seg_ts;
            }
            else
            {
                // Go to next segment
                evt->time.hs = CLK_ADD_2(evt->time.hs, ld_iscan_seg_scan_env.seg_scan.distance);
            }

            // Set the segment timings
            ld_iscan_set_seg_scan_timings(evt);

        }
        while (sch_arb_insert(evt) != SCH_ARB_ERROR_OK);
    }
    break;

    case SEG_SCAN_DISABLING:
    {
        // Restore initial scan parameters
        iscan_par->win_rem_slots     = iscan_par->iscan_win;
        iscan_par->interlace_state   = ISCAN_INTER_OFF;
        evt->duration_min            = sch_slice_params.scan_evt_dur;
        evt->time.hus                = 0;
        evt->current_prio            = rwip_priority[RWIP_PRIO_ISCAN_DFT_IDX].value;
        SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_ASAP_NO_LIMIT, SCH_ARB_NO_PHASE, LD_ISCAN_EVT_AUTO_RESCHED_ATT, RWIP_PRIO_INC(RWIP_PRIO_ISCAN_DFT_IDX));

        // Schedule event ASAP
        evt->time.hs = rwip_time_get().hs;
        if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
        {
            iscan_par->state = ISCAN_EVT_WAIT;
        }
        else
        {
            ASSERT_ERR_FORCE(0);
        }

        // Set as inactive
        ld_iscan_seg_scan_env.seg_scan_state = SEG_SCAN_OFF;
    }
    break;

    default:
    {
        ASSERT_INFO_FORCE(0, ld_iscan_seg_scan_env.seg_scan_state, 0);
    }
    break;
    }

    // Set state
    iscan_par->state = ISCAN_EVT_WAIT;

}
#endif // EAVESDROPPING_SUPPORT

/**
 ****************************************************************************************
 * @brief Cleanup iscan environment
 ****************************************************************************************
 */
__STATIC void ld_iscan_cleanup(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LD_ISCAN_PATCH_TYPE, LD_ISCAN_CLEANUP_BIT);

    // Remove permission/status of CS as now unused
    DBG_MEM_PERM_SET((const void *)(REG_EM_BT_CS_BASE_ADDR + REG_EM_BT_CS_ADDR_GET(EM_BT_CS_ISCAN_INDEX)), REG_EM_BT_CS_SIZE, false, false, false);

    // Unregister the scan window from scheduling parameters
    sch_slice_bg_remove(BT_ISCAN);

    // Free event memory
    ke_free(ld_iscan_env);
    ld_iscan_env = NULL;
}

/**
 ****************************************************************************************
 * @brief Program the activity
 ****************************************************************************************
 */
__STATIC void ld_iscan_prog(uint32_t clock)
{
    // Point to parameters
    struct ld_iscan_env_tag *iscan_par = ld_iscan_env;
    struct sch_arb_elt_tag *evt = &iscan_par->evt;
    uint8_t cs_idx = EM_BT_CS_ISCAN_INDEX;

    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_ISCAN_PATCH_TYPE, LD_ISCAN_PROG_BIT, clock);
#if (EAVESDROPPING_SUPPORT)
    if ((ld_iscan_seg_scan_env.seg_scan_state == SEG_SCAN_ON) || (ld_iscan_seg_scan_env.seg_scan_state == SEG_SCAN_DISABLING))
    {
        uint8_t phase_offset = ld_iscan_seg_scan_env.seg_scan.phase_offsets[ld_iscan_seg_scan_env.seg_cnt];

        // Prepare CS
        em_bt_wincntl_pack(EM_BT_CS_ISCAN_INDEX, 0, ld_iscan_seg_scan_env.seg_scan.duration >> 2);

        // Set phase offset for the segment
        em_bt_clkoff0_setf(EM_BT_CS_ISCAN_INDEX, (phase_offset << 11) & EM_BT_CLKOFF0_MASK);
        em_bt_clkoff1_setf(EM_BT_CS_ISCAN_INDEX, (phase_offset << 11) >> 16);
    }
    else
#endif // EAVESDROPPING_SUPPORT
    {
        // Check to not exceed the maximum value supported by HW
        uint32_t max_win_len_hs = co_min(2 * iscan_par->win_rem_slots, (EM_BT_RXWINSZ_MASK >> EM_BT_RXWINSZ_LSB));

        // Prepare CS
        em_bt_wincntl_pack(cs_idx, 1, (max_win_len_hs + 1) >> 1);
    }

    {
        // Push the programming to SCH PROG
        struct sch_prog_params prog_par;
        prog_par.frm_cbk         = &ld_iscan_frm_cbk;
        prog_par.time.hs         = evt->time.hs;
#if (EAVESDROPPING_SUPPORT)
        prog_par.time.hus        = evt->time.hus;
#else // EAVESDROPPING_SUPPORT
        prog_par.time.hus        = 0;
#endif // EAVESDROPPING_SUPPORT
        prog_par.cs_idx          = cs_idx;
        prog_par.dummy           = cs_idx;
        prog_par.bandwidth       = evt->duration_min;
        prog_par.prio_1          = evt->current_prio;
        prog_par.prio_2          = 0;
        prog_par.prio_3          = rwip_priority[RWIP_PRIO_PSCAN_1ST_PKT_IDX].value;
        prog_par.pti_prio        = BT_PTI_INQRES_IDX;
        prog_par.add.bt.frm_type = SCH_BT_FRAME_TYPE_NORMAL;
        prog_par.mode            = SCH_PROG_BT;
        sch_prog_push(&prog_par);
    }
}

/**
 ****************************************************************************************
 * @brief Handle event start notification
 ****************************************************************************************
 */
__STATIC void ld_iscan_evt_start_cbk(struct sch_arb_elt_tag *evt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_ISCAN_PATCH_TYPE, LD_ISCAN_EVT_START_CBK_BIT, evt);

    DBG_SWDIAG(INQ, ISCAN_EVT_START, 1);

    ASSERT_ERR((&(ld_iscan_env->evt)) == evt);

    if (evt != NULL)
    {
        // Point to parameters
        struct ld_iscan_env_tag *iscan_par = ld_iscan_env;

        // Program inquiry scan
        ld_iscan_prog(evt->time.hs);

        // Move state
        iscan_par->state = ISCAN_EVT_ACTIVE;
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(INQ, ISCAN_EVT_START, 0);
}

/**
 ****************************************************************************************
 * @brief Handle event canceled notification
 ****************************************************************************************
 */
__STATIC void ld_iscan_evt_canceled_cbk(struct sch_arb_elt_tag *evt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_ISCAN_PATCH_TYPE, LD_ISCAN_EVT_CANCELED_CBK_BIT, evt);

    DBG_SWDIAG(INQ, ISCAN_EVT_CANCELED, 1);

    ASSERT_ERR((&(ld_iscan_env->evt)) == evt);

    if (evt != NULL)
    {
        // Check inquiry scan end
        if (ld_iscan_env->state == ISCAN_EVT_END)
        {
#if (EAVESDROPPING_SUPPORT)
            if (ld_iscan_seg_scan_env.seg_scan_state == SEG_SCAN_ON)
            {
                // Clear segmented scan
                ld_iscan_seg_scan_env.seg_scan_state = SEG_SCAN_ENABLING;
            }
            else if (ld_iscan_seg_scan_env.seg_scan_state == SEG_SCAN_DISABLING)
            {
                // Clear segmented scan
                ld_iscan_seg_scan_env.seg_scan_state = SEG_SCAN_OFF;
            }
#endif // EAVESDROPPING_SUPPORT

            // Free event memory
            ld_iscan_cleanup();
        }
        else
        {
            // Increment priority
            evt->current_prio = RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(RWIP_PRIO_ISCAN_DFT_IDX));

#if (EAVESDROPPING_SUPPORT)
            if (ld_iscan_seg_scan_env.seg_scan_state != SEG_SCAN_OFF)
            {
                ld_iscan_seg_scan_sched();
            }
            else
#endif // EAVESDROPPING_SUPPORT
            {
                // Reload the automatic rescheduling attempts
                SCH_ARB_ASAP_STG_RESCHED_ATT_SET(evt, LD_ISCAN_EVT_AUTO_RESCHED_ATT);

                // Reschedule ASAP
                if (sch_arb_insert(evt) != SCH_ARB_ERROR_OK)
                {
                    ASSERT_ERR_FORCE(0);
                }
                else
                {
                    ld_iscan_env->state = ISCAN_EVT_WAIT;
                }
            }
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(INQ, ISCAN_EVT_CANCELED, 0);
}

/**
 ****************************************************************************************
 * @brief Handle frame interrupt
 ****************************************************************************************
 */
__STATIC void ld_iscan_frm_isr(uint32_t timestamp)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_ISCAN_PATCH_TYPE, LD_ISCAN_FRM_ISR_BIT, timestamp);

    DBG_SWDIAG(INQ, ISCAN_FRM_ISR, 1);

    if (ld_iscan_env != NULL)
    {
        // Point to parameters
        struct sch_arb_elt_tag *evt = &(ld_iscan_env->evt);
        struct ld_iscan_env_tag *iscan_par = ld_iscan_env;

        // Remove event
        sch_arb_remove(evt, true);

        // Check inquiry scan end
        if (iscan_par->state == ISCAN_EVT_END)
        {
#if (EAVESDROPPING_SUPPORT)
            if (ld_iscan_seg_scan_env.seg_scan_state == SEG_SCAN_ON)
            {
                // Clear segmented scan
                ld_iscan_seg_scan_env.seg_scan_state = SEG_SCAN_ENABLING;
            }
            else if (ld_iscan_seg_scan_env.seg_scan_state == SEG_SCAN_DISABLING)
            {
                // Clear segmented scan
                ld_iscan_seg_scan_env.seg_scan_state = SEG_SCAN_OFF;
            }
#endif // EAVESDROPPING_SUPPORT

            // Free event memory
            ld_iscan_cleanup();
        }
#if (EAVESDROPPING_SUPPORT)
        else if (ld_iscan_seg_scan_env.seg_scan_state != SEG_SCAN_OFF)
        {
            ld_iscan_seg_scan_sched();
        }
#endif // EAVESDROPPING_SUPPORT
        else
        {
            uint32_t clock = ld_read_clock();
            uint8_t cs_idx = EM_BT_CS_ISCAN_INDEX;

            // Check if ID has been received
            if (em_bt_rxbit_frrxok_getf(EM_BT_CS_ISCAN_INDEX) && em_bt_aclrxstat_rxid_getf(EM_BT_CS_ISCAN_INDEX))
            {
                // Reload a new window
                iscan_par->win_rem_slots = iscan_par->iscan_win;

                // Apply a random back-off period
                uint16_t rand = CO_MOD(co_rand_word(), LD_ISCAN_MAX_RAND);
                evt->time.hs = CO_ALIGN4_HI(CLK_ADD_2(clock, 2 * rand));

                // Increment the phase in the inquiry hop sequence
                iscan_par->interlace_state = ISCAN_INTER_WIN0;
                iscan_par->phase_offset++;

                // Register the scan window as active to scheduling parameters
                sch_slice_bg_add(BT_ISCAN);
            }
            else
            {
                uint32_t effective_slots = CLK_SUB(clock, timestamp) >> 1;

                if (effective_slots <= iscan_par->win_rem_slots)
                {
                    // Subtract the number of effective scanning slots
                    iscan_par->win_rem_slots -= effective_slots;
                }
                else
                {
                    iscan_par->win_rem_slots = 0;
                }

                // Check if current inquiry scan window is finished
                if (iscan_par->win_rem_slots == 0)
                {
                    uint32_t iscan_win = iscan_par->iscan_win;

                    // The delay and frequency of the next window depends on the interlace scan state
                    switch (iscan_par->interlace_state)
                    {
                    case ISCAN_INTER_WIN0:
                    {
                        // Increment the phase in the inquiry hop sequence by the interlace offset
                        iscan_par->interlace_state = ISCAN_INTER_WIN1;

#if RW_BT_MWS_COEX // Generalized Interlaced Scan Support - enabled for ISCAN_INTER_WIN1
                        if (ld_iscan_mwscoex_xi_mask && (ld_mws_inactivity_duration_get() << 3) < iscan_par->iscan_win) // 5ms units * 8 < iscan_win slots
                        {
                            // Control for hop system in generalized interlaced scan: (CLKN16:12 + interlace offset) mod 32
                            uint32_t clk_16to12 = (ld_read_clock() >> 12) & 0x1F;
                            uint8_t Xi = ld_iscan_mwscoex_xi_get((clk_16to12 + iscan_par->phase_offset + INTERLACE_OFFSET_DFT) & 0x1F);
                            iscan_par->phase_offset = (32 + Xi - clk_16to12) & 0x1F;
                            // Coex does not prevent Tx/Rx
                            em_bt_frcntl_dnabort_setf(cs_idx, 1);
                        }
                        // Adjust for unavailable slots
                        else if (ld_env.mws_ext_fr_scan_dur < ld_env.mws_ext_fr_duration)
                        {
                            // Coex prevents Tx/Rx, redimension scan window
                            iscan_win = (iscan_win * ld_env.mws_ext_fr_duration) / ld_env.mws_ext_fr_scan_dur;
                            em_bt_frcntl_dnabort_setf(cs_idx, 0);
                        }
                        else
#endif // RW_BT_MWS_COEX
                        {
                            iscan_par->phase_offset += INTERLACE_OFFSET_DFT;
                        }

                        // Register the scan window as active to scheduling parameters
                        sch_slice_bg_add(BT_ISCAN);
                    }
                    break;
                    case ISCAN_INTER_WIN1:
                    {
                        // Clear the phase in the inquiry hop sequence
                        iscan_par->interlace_state = ISCAN_INTER_WIN0;
                        iscan_par->phase_offset = 0;
                    } // No break
                    case ISCAN_INTER_OFF:
                    {
                        // Add scan window period
                        int32_t delay = iscan_par->iscan_intv - iscan_win;
                        evt->time.hs = CO_ALIGN4_HI(CLK_ADD_2(clock, 2 * delay));

                        // Unregister the scan window from scheduling parameters
                        sch_slice_bg_remove(BT_ISCAN);
                    }
                    break;
                    default:
                    {
                        ASSERT_ERR_FORCE(0);
                    }
                    break;
                    }

                    // Reload window length
                    iscan_par->win_rem_slots = iscan_win;
                }
                else
                {
                    // Register the scan window as active to scheduling parameters
                    sch_slice_bg_add(BT_ISCAN);
                }
            }

            // Update clock offset (used for the inquiry hopping sequence)
            em_bt_clkoff0_setf(cs_idx, (iscan_par->phase_offset << 12) & EM_BT_CLKOFF0_MASK);
            em_bt_clkoff1_setf(cs_idx, (iscan_par->phase_offset << 12) >> 16);

            // Restore original priority
            evt->current_prio = rwip_priority[RWIP_PRIO_ISCAN_DFT_IDX].value;
            evt->duration_min = co_min(2 * iscan_par->win_rem_slots * SLOT_SIZE, sch_slice_params.scan_evt_dur);

            // Try reschedule
            if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
            {
                iscan_par->state = ISCAN_EVT_WAIT;
            }
            else
            {
                ASSERT_ERR_FORCE(0);
            }
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(INQ, ISCAN_FRM_ISR, 0);
}

/**
 ****************************************************************************************
 * @brief Handle frame interrupt notification
 ****************************************************************************************
 */
__STATIC void ld_iscan_frm_cbk(uint32_t timestamp, uint32_t dummy, uint8_t irq_type)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LD_ISCAN_PATCH_TYPE, LD_ISCAN_FRM_CBK_BIT, timestamp, dummy, irq_type);

    ASSERT_INFO(dummy == EM_BT_CS_ISCAN_INDEX, dummy, irq_type);

    switch (irq_type)
    {
    case SCH_FRAME_IRQ_EOF:
    case SCH_FRAME_IRQ_EOF_ABORT_UNDER_PRIO:
    case SCH_FRAME_IRQ_EOF_ABORT_AFTER_PRIO:
    {
        ld_iscan_frm_isr(timestamp);
    }
    break;
    case SCH_FRAME_IRQ_SKIP:
    {
        // Point to parameters
        struct sch_arb_elt_tag *evt = &(ld_iscan_env->evt);

        // Remove event
        sch_arb_remove(evt, true);

        // Invoke event cancel callback
        ld_iscan_evt_canceled_cbk(evt);
    }
    break;
    default:
    {
        ASSERT_INFO_FORCE(0, dummy, irq_type);
    }
    break;
    }
}

#if RW_BT_MWS_COEX // Generalized Interlaced Scan Support
/**
 ****************************************************************************************
 * @brief Get a suitable X-input for inquiry scan based on MWS mask
 ****************************************************************************************
 */
__STATIC uint8_t ld_iscan_mwscoex_xi_get(uint8_t xi_nom)
{
    uint32_t iscan_mask;
    uint8_t xi;

    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LD_ISCAN_PATCH_TYPE, LD_ISCAN_MWSCOEX_XI_GET_BIT, uint8_t, xi_nom);

    // search higher bits for suitable X input
    iscan_mask = ld_iscan_mwscoex_xi_mask >> xi_nom;
    for (xi = xi_nom; (iscan_mask & BIT0) && (xi < 32); xi++)
    {
        iscan_mask >>= 1;
    }

    if (xi == 32) // searched all higher bits
    {
        //search lower bits for suitable X input
        iscan_mask = ld_iscan_mwscoex_xi_mask;
        for (xi = 0; (iscan_mask & BIT0) && (xi <= xi_nom); xi++)
        {
            iscan_mask >>= 1;
        }
    }

    return xi;
}
#endif // RW_BT_MWS_COEX

/*
 * EXPORTED FUNCTIONS DEFINITION
 *****************************************************************************************
 */

void ld_iscan_init(void)
{
    struct eir eir_zeros;

    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LD_ISCAN_PATCH_TYPE, LD_ISCAN_INIT_BIT);

    ld_iscan_env = NULL;

#if (EAVESDROPPING_SUPPORT)
    memset(&ld_iscan_seg_scan_env, 0, sizeof(ld_iscan_seg_scan_env));
#endif // EAVESDROPPING_SUPPORT

    // The initial value of the inquiry response data is all zero octets
    memset(&eir_zeros, 0, sizeof(struct eir));
    em_wr(&eir_zeros, EM_BT_EIRTXBUF_OFFSET, EIR_DATA_SIZE);

#if RW_BT_MWS_COEX
    ld_iscan_mwscoex_xi_mask = 0;
#endif // RW_BT_MWS_COEX
}

void ld_iscan_reset(void)
{
    struct eir eir_zeros;

    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LD_ISCAN_PATCH_TYPE, LD_ISCAN_RESET_BIT);

    // Check if inquiry scan is active
    if (ld_iscan_env != NULL)
    {
        // Free event memory
        ke_free(ld_iscan_env);
        ld_iscan_env = NULL;
    }

#if (EAVESDROPPING_SUPPORT)
    memset(&ld_iscan_seg_scan_env, 0, sizeof(ld_iscan_seg_scan_env));
#endif // EAVESDROPPING_SUPPORT

    // The initial value of the inquiry response data is all zero octets
    // The extended inquiry response data is not preserved over a reset
    memset(&eir_zeros, 0, sizeof(struct eir));
    em_wr(&eir_zeros, EM_BT_EIRTXBUF_OFFSET, EIR_DATA_SIZE);

#if RW_BT_MWS_COEX
    ld_iscan_mwscoex_xi_mask = 0;
#endif // RW_BT_MWS_COEX
}

uint8_t ld_iscan_start(struct ld_inquiry_scan_params *params)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    struct ld_iscan_env_tag *iscan_par;

    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LD_ISCAN_PATCH_TYPE, LD_ISCAN_START_BIT, uint8_t, params);

    // Check if inquiry scan is inactive
    if (ld_iscan_env == NULL)
    {
        // Allocate event
        iscan_par = LD_ALLOC_EVT(ld_iscan_env_tag);

        if (iscan_par != NULL)
        {
            uint32_t clock = ld_read_clock();
            bool eir = (ld_iscan_eir.eir_len > 0);
            uint8_t bch[LD_BCH_SIZE];
            uint8_t cs_idx = EM_BT_CS_ISCAN_INDEX;

            // Point to parameters
            struct sch_arb_elt_tag *evt = &(iscan_par->evt);

            LD_INIT_EVT(evt, ld_iscan_env_tag);

            // Set permission/status of CS as R/W but uninitialized
            DBG_MEM_PERM_SET((const void *)(REG_EM_BT_CS_BASE_ADDR + REG_EM_BT_CS_ADDR_GET(cs_idx)), REG_EM_BT_CS_SIZE, true, true, true);

            // Initialize event parameters (common part)
            evt->cb_cancel        = &ld_iscan_evt_canceled_cbk;
            evt->cb_start         = &ld_iscan_evt_start_cbk;
            evt->cb_stop          = NULL;
            evt->current_prio        = rwip_priority[RWIP_PRIO_ISCAN_DFT_IDX].value;
            SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_ASAP_NO_LIMIT, SCH_ARB_NO_PHASE, LD_ISCAN_EVT_AUTO_RESCHED_ATT, RWIP_PRIO_INC(RWIP_PRIO_ISCAN_DFT_IDX));

            // Initialize event parameters (inquiry scan part)
            iscan_par->lap               = params->lap;
            iscan_par->iscan_intv        = params->iscan_intv;
            iscan_par->iscan_win         = params->iscan_win;
            iscan_par->pscan_rep_mode    = params->page_scan_rep_mode;
            iscan_par->win_rem_slots     = iscan_par->iscan_win;
            iscan_par->interlace_state   = ISCAN_INTER_OFF;
            if ((params->iscan_intv >= (2 * params->iscan_win)) && (params->iscan_type == INTERLACED_SCAN))
            {
                iscan_par->interlace_state = ISCAN_INTER_WIN0;
            }

            // Compute the BCH with the BD Address (used for FHS for example)
            ld_util_bch_create(&iscan_par->lap.A[0], &bch[0]);

            // Pack FHS
            ld_util_fhs_pk(EM_BT_ISCANFHSTXBUF_OFFSET, &ld_env.local_bch[0], eir, iscan_par->pscan_rep_mode, &ld_env.local_bd_addr, &ld_env.class_of_dev, 0, MANDATORY_PAGE_SCAN_MODE);

            // Set FHS Tx descriptor fields
            em_bt_txheader_pack(EM_BT_TXDESC_ISCANFHS_INDEX, 0, 0, 1, 1, 1, FHS_TYPE, 0);
            em_bt_txpheader_pack(EM_BT_TXDESC_ISCANFHS_INDEX, 0, FHS_PACKET_SIZE, 0, LLID_START);
            em_bt_txaclbufptr_setf(EM_BT_TXDESC_ISCANFHS_INDEX, 0);
            em_bt_txlmbufptr_setf(EM_BT_TXDESC_ISCANFHS_INDEX, EM_BT_ISCANFHSTXBUF_OFFSET);
            em_bt_txptr_pack(EM_BT_TXDESC_ISCANFHS_INDEX, 0, 0);

            if (eir)
            {
                // Set FHS descriptor next pointer
                em_bt_txptr_nextptr_setf(EM_BT_TXDESC_ISCANFHS_INDEX, REG_EM_ADDR_GET(BT_TXDESC, EM_BT_TXDESC_EIR_INDEX));

                // Set EIR Tx descriptor fields
                em_bt_txheader_pack(EM_BT_TXDESC_EIR_INDEX, 0, 0, 0, 0, 0, ld_iscan_eir.eir_pkt_type, 0);
                em_bt_txpheader_pack(EM_BT_TXDESC_EIR_INDEX, 0, ld_iscan_eir.eir_len, 0, LLID_START);
                em_bt_txaclbufptr_setf(EM_BT_TXDESC_EIR_INDEX, EM_BT_EIRTXBUF_OFFSET);
                em_bt_txlmbufptr_setf(EM_BT_TXDESC_EIR_INDEX, 0);
                em_bt_txptr_pack(EM_BT_TXDESC_EIR_INDEX, 0, REG_EM_ADDR_GET(BT_TXDESC, EM_BT_TXDESC_ISCANFHS_INDEX));
            }

            // Set control structure fields
            em_bt_frcntl_pack(cs_idx, 1 /*fpoll*/, 0 /*fmwspatt*/, RWIP_COEX_GET(INQRES, TXBSY), RWIP_COEX_GET(INQRES, RXBSY), RWIP_COEX_GET(INQRES, DNABORT), 0 /*samen*/, 1 /*lsam_dsb*/, EM_BT_CS_FMT_INQ_RSP);
            em_bt_bdaddr_setf(cs_idx, 0, (iscan_par->lap.A[1] << 8) | iscan_par->lap.A[0]);
            em_bt_bdaddr_setf(cs_idx, 1, (0 << 8) | iscan_par->lap.A[2]);
            em_bt_bdaddr_setf(cs_idx, 2, (0 << 8) | 0);
            em_bt_bch0_setf(cs_idx, (bch[1] << 8) | bch[0]);
            em_bt_bch1_setf(cs_idx, (bch[3] << 8) | bch[2]);
            em_bt_rxmaxbuf_bch2_pack(cs_idx, 0, 0, bch[4] & EM_BT_BCH2_MASK);
            em_bt_pwrcntl_pack(cs_idx, /*fhen*/ 1, /*freq*/ 0, /*txpwr*/ INQ_RSP_TX_PWR);
            em_bt_linkcntl_pack(cs_idx,
                                /*aknena*/ 0,
                                /*afhena*/ 0,
                                /*laap*/ 0,
                                /*whdsb*/ 0,
                                /*acledr*/ 0,
                                /*aclltaddr*/ 0,
#if (EAVESDROPPING_SUPPORT)
                                /*aclbinackena*/ 0, /*escobinackena*/ 0, /*duplfltena*/ 0,
#endif // EAVESDROPPING_SUPPORT
                                /*linklbl*/ cs_idx);
            em_bt_txrxcntl_pack(cs_idx, /*rxthr*/ 0,
#if (EAVESDROPPING_SUPPORT)
                                /*stopmod*/ 0,
#endif // EAVESDROPPING_SUPPORT
                                /*rxeir*/ 0, /*rxbcry*/ 0, /*rxcrypt*/ 0, /*stopmod/escofnak*/ 0, /*nullflt*/ 0, /*extpaen*/ 0, /*txeir*/ eir, /*txbcry*/ 0, /*txcrypt*/ 0);
            em_bt_txdescptr_setf(cs_idx, REG_EM_ADDR_GET(BT_TXDESC, EM_BT_TXDESC_ISCANFHS_INDEX));
            em_bt_acltxstat_pack(cs_idx, /*txtog*/ 0, /*txflush*/ 0, /*fcrc*/ 0, /*fnak*/ 0, /*rswitch*/ 0, /*waitack*/ 0, /*lastnull*/ 0, /*lasttxack*/ 0, /*lasttxseqn*/ 0);
            em_bt_clkoff0_setf(cs_idx, 0);
            em_bt_clkoff1_setf(cs_idx, 0);
            em_bt_loopcntl_pack(cs_idx, /*tonb*/ 1, /*attnb*/ 1);

            // Default clear other control
            em_bt_chmap0_set(cs_idx, 0);
            em_bt_chmap1_set(cs_idx, 0);
            em_bt_chmap2_set(cs_idx, 0);
            em_bt_chmap3_set(cs_idx, 0);
            em_bt_chmap4_set(cs_idx, 0);
            em_bt_maxfrmtime_set(cs_idx, 0);
#if (EAVESDROPPING_SUPPORT)
            em_bt_cidcntl_set(cs_idx, 0);
            em_bt_rxcrc_set(cs_idx, 0);
#endif // EAVESDROPPING_SUPPORT

            rwip_rf.txpwr_cs_set(TYPE_BT, cs_idx, rwip_rf.txpwr_max_mod, rwip_rf.txpwr_max);

            GLOBAL_INT_DISABLE();

            // Assign configured parameters
            ld_iscan_env = iscan_par;

#if (EAVESDROPPING_SUPPORT)
            if (ld_iscan_seg_scan_env.seg_scan_state != SEG_SCAN_OFF)
            {
                ld_iscan_seg_scan_sched();
            }
            else
#endif // EAVESDROPPING_SUPPORT
            {
                // Schedule event ASAP
                evt->time.hs = clock;
                evt->duration_min = co_min(2 * iscan_par->win_rem_slots * SLOT_SIZE, sch_slice_params.scan_evt_dur);
                if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
                {
                    iscan_par->state = ISCAN_EVT_WAIT;
                }
                else
                {
                    ASSERT_ERR_FORCE(0);
                }
            }

            GLOBAL_INT_RESTORE();

            status = CO_ERROR_NO_ERROR;
        }
        else
        {
            status = CO_ERROR_MEMORY_CAPA_EXCEED;
            ASSERT_ERR_FORCE(0);
        }
    }

    return (status);
}

uint8_t ld_iscan_stop(void)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    FUNC_PATCH_ENTRY_NO_PARAM_HAVE_RETURN(LD_ISCAN_PATCH_TYPE, LD_ISCAN_STOP_BIT, uint8_t);

    GLOBAL_INT_DISABLE();

    if (ld_iscan_env != NULL)
    {
        // Point to parameters
        struct sch_arb_elt_tag *evt = &(ld_iscan_env->evt);
        struct ld_iscan_env_tag *iscan_par = ld_iscan_env;

        switch (iscan_par->state)
        {
        case ISCAN_EVT_WAIT:
        {
            // Remove event
            sch_arb_remove(evt, false);

#if (EAVESDROPPING_SUPPORT)
            if (ld_iscan_seg_scan_env.seg_scan_state == SEG_SCAN_ON)
            {
                // Clear segmented scan
                ld_iscan_seg_scan_env.seg_scan_state = SEG_SCAN_ENABLING;
            }
            else if (ld_iscan_seg_scan_env.seg_scan_state == SEG_SCAN_DISABLING)
            {
                // Clear segmented scan
                ld_iscan_seg_scan_env.seg_scan_state = SEG_SCAN_OFF;
            }
#endif // EAVESDROPPING_SUPPORT

            // Free event memory
            ld_iscan_cleanup();
        }
        break;

        case ISCAN_EVT_ACTIVE:
        {
            // Clear frame duration (in case the CS has not been prefetched yet)
            em_bt_wincntl_pack(EM_BT_CS_ISCAN_INDEX, 0, NORMAL_WIN_SIZE);

            // Abort inquiry scan
            bt_rwbtcntl_scan_abort_setf(1);

            // Move state
            iscan_par->state = ISCAN_EVT_END;
        }
        break;

        default:
        {
            // Nothing to do
        }
        break;
        }

        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return (status);
}

uint8_t ld_iscan_eir_set(uint8_t fec_req, const struct eir *eir_ptr)
{
    // Packet size table. Zeros are added for reserved packet and are needed to get the
    // packet size from the packet type flags
    const uint16_t pkt_size[16] =
    {
        0,
        0,
        0,
        DM1_PACKET_SIZE,
        DH1_PACKET_SIZE,
        0,
        0,
        0,
        DV_ACL_PACKET_SIZE,
        AUX1_PACKET_SIZE,
        DM3_PACKET_SIZE,
        DH3_PACKET_SIZE,
        0,
        0,
        DM5_PACKET_SIZE,
        DH5_PACKET_SIZE
    };

    uint8_t status = CO_ERROR_NO_ERROR;
    uint8_t eir_len = 0;
    uint8_t eir_pkt_type = 0;
    uint8_t i = 0;

    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LD_ISCAN_PATCH_TYPE, LD_ISCAN_EIR_SET_BIT, uint8_t, fec_req, eir_ptr);

    if (fec_req > 1)
    {
        status = CO_ERROR_INVALID_HCI_PARAM;
    }
    else
    {
        // Algorithm has been changed for the qualification: look from the end of the data
        // until non-zeros data are found (instead of looking at the data content)
        i = EIR_DATA_SIZE - 1;
        while ((eir_ptr->data[i] == 0) && (i > 0))
        {
            i--;
        }
        eir_len = i + 1;

        eir_pkt_type = 0;
        while (eir_len > pkt_size[eir_pkt_type])
        {
            eir_pkt_type++;
        }

        // skip AUX1 packet, not allowed in EIR
        if (eir_pkt_type == AUX1_TYPE)
        {
            eir_pkt_type = DM3_TYPE;
        }

        // If FEC packet has been required, get the following packet type with FEC
        if (fec_req == 0x01)
        {
            switch (eir_pkt_type)
            {
            case DH1_TYPE:
                eir_pkt_type = DM3_TYPE;
                break;
            case DH3_TYPE:
                eir_pkt_type = DM5_TYPE;
                break;
            case DH5_TYPE:
                status = CO_ERROR_EIR_TOO_LARGE;
                break;
            default:
                break;
            }
        }

        if (status == CO_ERROR_NO_ERROR)
        {
            // Copy EIR to EM EIR buffer
            em_wr(eir_ptr, EM_BT_EIRTXBUF_OFFSET, EIR_DATA_SIZE);

            ld_iscan_eir.fec_req = fec_req;
            ld_iscan_eir.eir_len = eir_len;
            ld_iscan_eir.eir_pkt_type = eir_pkt_type;

            // Qualification requires "on the fly" update because iscan may be enabled before EIR is set
            if (ld_iscan_env != NULL)
            {
                bool eir = (ld_iscan_eir.eir_len > 0);

                if (eir)
                {
                    // Set FHS descriptor next pointer
                    em_bt_txptr_nextptr_setf(EM_BT_TXDESC_ISCANFHS_INDEX, REG_EM_ADDR_GET(BT_TXDESC, EM_BT_TXDESC_EIR_INDEX));

                    // Set EIR Tx descriptor fields
                    em_bt_txheader_pack(EM_BT_TXDESC_EIR_INDEX, 0, 0, 0, 0, 0, ld_iscan_eir.eir_pkt_type, 0);
                    em_bt_txpheader_pack(EM_BT_TXDESC_EIR_INDEX, 0, ld_iscan_eir.eir_len, 0, LLID_START);
                    em_bt_txaclbufptr_setf(EM_BT_TXDESC_EIR_INDEX, EM_BT_EIRTXBUF_OFFSET);
                    em_bt_txlmbufptr_setf(EM_BT_TXDESC_EIR_INDEX, 0);
                    em_bt_txptr_pack(EM_BT_TXDESC_EIR_INDEX, 0, REG_EM_ADDR_GET(BT_TXDESC, EM_BT_TXDESC_ISCANFHS_INDEX));
                }

                // Update only if current EIR setting is different from the previous one
                if (em_bt_txrxcntl_txeir_getf(EM_BT_CS_ISCAN_INDEX) != eir)
                {
                    // Point to parameters
                    struct ld_iscan_env_tag *iscan_par = ld_iscan_env;

                    // Pack FHS
                    ld_util_fhs_pk(EM_BT_ISCANFHSTXBUF_OFFSET, &ld_env.local_bch[0], eir, iscan_par->pscan_rep_mode, &ld_env.local_bd_addr, &ld_env.class_of_dev, 0, MANDATORY_PAGE_SCAN_MODE);

                    em_bt_txrxcntl_txeir_setf(EM_BT_CS_ISCAN_INDEX, eir);
                }
            }
        }
    }

    return (status);
}

uint8_t ld_iscan_eir_get(uint8_t *fec_req, struct eir *eir_ptr)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LD_ISCAN_PATCH_TYPE, LD_ISCAN_EIR_GET_BIT, uint8_t, fec_req, eir_ptr);

    // Read EIR from EM EIR buffer
    em_rd(eir_ptr, EM_BT_EIRTXBUF_OFFSET, EIR_DATA_SIZE);

    *fec_req = ld_iscan_eir.fec_req;

    return (CO_ERROR_NO_ERROR);
}

#if RW_BT_MWS_COEX // Generalized Interlaced Scan Support
/**
 ****************************************************************************************
 * @brief Build an X-input mask for inquiry scan based on MWS
 ****************************************************************************************
 */
void ld_iscan_mwscoex_xi_mask_build(struct lap *iac_lap)
{
    uint32_t uap_lap = ((uint32_t)iac_lap->A[2] << 16) + ((uint32_t)iac_lap->A[1] << 8) + iac_lap->A[0];

    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_ISCAN_PATCH_TYPE, LD_ISCAN_MWSCOEX_XI_MASK_BUILD_BIT, iac_lap);

    // Build Xi mask for inquiry scan in ld_iscan_mwscoex_xi_mask based on IAC lap
    ld_mwscoex_xi_scan_mask_build(&ld_iscan_mwscoex_xi_mask, uap_lap);
}
#endif // RW_BT_MWS_COEX

#if (EAVESDROPPING_SUPPORT)
uint8_t ld_iscan_seg_scan_en(struct ld_seg_scan_params *params)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    GLOBAL_INT_DISABLE();

    if ((ld_iscan_seg_scan_env.seg_scan_state == SEG_SCAN_OFF) || (ld_iscan_seg_scan_env.seg_scan_state == SEG_SCAN_DISABLING))
    {
        // Enable segmented scan
        ld_iscan_seg_scan_env.seg_scan_state = SEG_SCAN_ENABLING;

        // Store parameters
        memcpy(&ld_iscan_seg_scan_env.seg_scan, params, sizeof(struct ld_seg_scan_params));

        // Check if inquiry scan is active
        if (ld_iscan_env != NULL)
        {
            // Point to parameters
            struct ld_iscan_env_tag *iscan_par = ld_iscan_env;
            struct sch_arb_elt_tag *evt = &iscan_par->evt;

            // Check if event is waiting
            if (iscan_par->state == ISCAN_EVT_WAIT)
            {
                // Remove from schedule
                sch_arb_remove(evt, false);

                // Reschedule as segmented scan
                ld_iscan_seg_scan_sched();
            }
        }

        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return status;
}

uint8_t ld_iscan_seg_scan_dis(void)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    GLOBAL_INT_DISABLE();

    if ((ld_iscan_env != NULL) && (ld_iscan_seg_scan_env.seg_scan_state == SEG_SCAN_ON))
    {
        // Disable segmented scan
        ld_iscan_seg_scan_env.seg_scan_state = SEG_SCAN_DISABLING;

        status = CO_ERROR_NO_ERROR;
    }
    else if (ld_iscan_seg_scan_env.seg_scan_state == SEG_SCAN_ENABLING)
    {
        // Indicate segmented scan as inactive
        ld_iscan_seg_scan_env.seg_scan_state = SEG_SCAN_OFF;

        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return status;
}
#endif // EAVESDROPPING_SUPPORT
///@} LDISCAN
