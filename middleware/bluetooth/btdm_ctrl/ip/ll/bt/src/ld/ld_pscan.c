/**
****************************************************************************************
*
* @file ld_pscan.c
*
* @brief LD Page Scan source code
*
* Copyright (C) RivieraWaves 2009-2015
*
****************************************************************************************
*/

/**
 ****************************************************************************************
 * @addtogroup LDPSCAN
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
#include <stdbool.h>

#include "co_utils.h"
#include "co_math.h"
#include "co_bt.h"

#include "arch.h"
#include "ke_mem.h"
#include "ke_event.h"
#include "rwip.h"

#include "ld.h"              // link driver API
#include "ld_util.h"         // link driver utilities
#include "ld_int.h"          // link driver internal

#include "lm.h"

#include "dbg.h"

#include "sch_arb.h"            // Scheduling Arbiter
#include "sch_prog.h"           // Scheduling Programmer
#include "sch_slice.h"          // Scheduling Slicer

#include "reg_btcore.h"         // BT core registers
#include "reg_em_bt_cs.h"       // BT EM Control Structure
#include "reg_em_bt_txdesc.h"   // BT EM RX descriptors
#include "reg_em_bt_rxdesc.h"   // BT EM RX descriptors
#include "btdm_patch.h"

/*
 * DEFINES
 *****************************************************************************************
 */

/// Minimum scheduling delay threshold where SW tries to advance the schedule (in BT slots)
#define LD_PSCAN_EVT_DELAY_RESCHED_MIN        30

/// Page scan event automatic rescheduling attempts
#define LD_PSCAN_EVT_AUTO_RESCHED_ATT   4

/// Page scan first packet event duration min (in us)
#define LD_PSCAN_1ST_PKT_EVT_DUR_MIN         (6*SLOT_SIZE - LD_ACL_INTERFRAME_MARGIN)
/// Page scan event automatic rescheduling attempts
#define LD_PSCAN_1ST_PKT_EVT_AUTO_RESCHED_ATT   4

/*
 * ENUMERATIONS DEFINITIONS
 ****************************************************************************************
 */

/// Page scan event states
enum PSCAN_EVT_STATE
{
    PSCAN_EVT_WAIT,
    PSCAN_EVT_ACTIVE,
    PSCAN_EVT_END,
};

/// Page scan response steps
enum PSCAN_STEP
{
    PSCAN_STEP_PSCAN,
    PSCAN_STEP_1ST_PKT,
};

/// Interlace Scan state
enum pscan_inter_state
{
    PSCAN_INTER_WIN0,
    PSCAN_INTER_WIN1,
    PSCAN_INTER_OFF,
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

#if (EAVESDROPPING_SUPPORT)
/// LD PSCAN Segmented Scan environment structure
struct ld_pscan_seg_scan_env_tag
{
    /// Segmented scan state (off, enabling, on, disabling)
    uint8_t seg_scan_state;

    /// Segments counter
    uint8_t seg_cnt;

    /// Timestamp of the first scan segment (in 625us slots)
    uint32_t first_seg_ts;

    /// Timestamp of the last segment (in 625us slots)
    uint32_t last_seg_ts;

    /// Segmented scan parameters
    struct ld_seg_scan_params seg_scan;
};
#endif //EAVESDROPPING_SUPPORT

/// LD PSCAN environment structure
struct ld_pscan_env_tag
{
    /// Page scan event
    struct sch_arb_elt_tag evt;

    /// Remaining slots in current scan window (in 625us BT slots)
    uint32_t win_rem_slots;
    /// Amount of time between consecutive page scans in slots (625 us)
    uint16_t pscan_intv;
    /// Amount of time for the duration of the page scan in slots (625 us)
    uint16_t pscan_win;
    /// Peer BD_ADDR
    struct bd_addr peer_bd_addr;
    // Peer Class of Device
    struct devclass class_of_dev;
    /// Link identifier
    uint8_t link_id;
    /// State
    uint8_t state;
    /// Page Scan Response step
    uint8_t step;
    /// Page scan step BCH
    uint8_t bch_pscan[LD_BCH_SIZE];
    /// 1st packet step BCH
    uint8_t bch_1st_pkt[LD_BCH_SIZE];
    /// First packet phase (0, 1, 2 or 3)
    uint8_t phase;
    /// Interlace scan state (1st window, 2nd window, or no interlace scan)
    uint8_t interlace_state;
};

/*
 * VARIABLE DEFINITION
 *****************************************************************************************
 */

/// LD PSCAN environment variable
__STATIC struct ld_pscan_env_tag *ld_pscan_env;

#if (EAVESDROPPING_SUPPORT)
    /// LD PSCAN Segmented Scan environment variable
    __STATIC struct ld_pscan_seg_scan_env_tag ld_pscan_seg_scan_env;
#endif // EAVESDROPPING_SUPPORT

#if RW_BT_MWS_COEX // Generalized Interlaced Scan Support
    /// bit mask for X input to page scan of MWS occupied channels
    __STATIC uint32_t ld_pscan_mwscoex_xi_mask;
#endif // RW_BT_MWS_COEX

/*
 * LOCAL FUNCTIONS DECLARATION
 *****************************************************************************************
 */

__STATIC void ld_pscan_frm_cbk(uint32_t timestamp, uint32_t dummy, uint8_t irq_type);
#if RW_BT_MWS_COEX
    __STATIC uint8_t ld_pscan_mwscoex_xi_get(uint8_t xi_nom);
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
__STATIC void ld_pscan_set_seg_scan_timings(struct sch_arb_elt_tag *evt)
{
    uint32_t clk_offset = 0;
    uint16_t bit_offset = 0;
    int16_t diff = 0;
    uint32_t timestamp_piconet = evt->time.hs;

    // Check if the BD address corresponds to an active connection
    uint8_t link_id = lm_find_link_id(ld_pscan_seg_scan_env.seg_scan.bd_addr);

    if (link_id < MAX_NB_ACTIVE_ACL)
    {
        // Get the timings of the link
        ld_acl_timing_info_get(link_id, &clk_offset, &bit_offset);
        timestamp_piconet = CLK_ADD_2(timestamp_piconet, clk_offset);

        // Increase priority
        evt->current_prio = rwip_priority[RWIP_PRIO_PSCAN_DFT_IDX].value;
    }

    diff = ld_pscan_seg_scan_env.seg_scan.time_offset - (timestamp_piconet % ld_pscan_seg_scan_env.seg_scan.periodicity);

    // Apply the half-microsecond alignment
    diff += ld_pscan_seg_scan_env.seg_scan.hus_offset / HALF_SLOT_SIZE;
    int16_t delay = bit_offset + (ld_pscan_seg_scan_env.seg_scan.hus_offset % HALF_SLOT_SIZE);

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
        diff = ld_pscan_seg_scan_env.seg_scan.periodicity - CO_MOD((-diff), ld_pscan_seg_scan_env.seg_scan.periodicity);
    }

    evt->time.hs = CLK_ADD_2(evt->time.hs, diff);
    evt->time.hus = (uint16_t)delay;
}
#endif // (EAVESDROPPING_SUPPORT)

/**
 ****************************************************************************************
 * @brief Initialize exchange memory
 ****************************************************************************************
 */
__STATIC void ld_pscan_em_init(struct ld_pscan_env_tag *pscan_par)
{
    uint8_t cs_idx;

    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_PSCAN_PATCH_TYPE, LD_PSCAN_EM_INIT_BIT, pscan_par);

    // Set control structure fields
    cs_idx = EM_BT_CS_ACL_INDEX(pscan_par->link_id);

    // Set permission/status of CS as R/W but uninitialized
    DBG_MEM_PERM_SET((const void *)(REG_EM_BT_CS_BASE_ADDR + REG_EM_BT_CS_ADDR_GET(cs_idx)), REG_EM_BT_CS_SIZE, true, true, true);

    em_bt_frcntl_pack(cs_idx, 1 /*fpoll*/, 0 /*fmwspatt*/, RWIP_COEX_GET(PSCAN, TXBSY), RWIP_COEX_GET(PSCAN, RXBSY), RWIP_COEX_GET(PSCAN, DNABORT), 0 /*samen*/, 1 /*lsam_dsb*/, EM_BT_CS_FMT_PAGE_SCAN);
    em_bt_bdaddr_setf(cs_idx, 0, (ld_env.local_bd_addr.addr[1] << 8) | ld_env.local_bd_addr.addr[0]);
    em_bt_bdaddr_setf(cs_idx, 1, (ld_env.local_bd_addr.addr[3] << 8) | ld_env.local_bd_addr.addr[2]);
    em_bt_bdaddr_setf(cs_idx, 2, (ld_env.local_bd_addr.addr[5] << 8) | ld_env.local_bd_addr.addr[4]);
    em_bt_bch0_setf(cs_idx, (ld_env.local_bch[1] << 8) | ld_env.local_bch[0]);
    em_bt_bch1_setf(cs_idx, (ld_env.local_bch[3] << 8) | ld_env.local_bch[2]);
    em_bt_rxmaxbuf_bch2_pack(cs_idx, FHS_PACKET_SIZE, 0, ld_env.local_bch[4] & EM_BT_BCH2_MASK);
    em_bt_pwrcntl_pack(cs_idx, /*fhen*/ 1, /*freq*/ 0, /*txpwr*/ rwip_rf.txpwr_max);
    em_bt_txrxcntl_pack(cs_idx, 0,
#if (EAVESDROPPING_SUPPORT)
                        0,
#endif // EAVESDROPPING_SUPPORT
                        0, 0, 0, 0, 0, 0, 0, 0, 0);
    em_bt_txdescptr_setf(cs_idx, 0);
    em_bt_acltxstat_pack(cs_idx, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    em_bt_clkoff0_setf(cs_idx, 0);
    em_bt_clkoff1_setf(cs_idx, 0);
    em_bt_linkcntl_pack(cs_idx,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
#if (EAVESDROPPING_SUPPORT)
                        /*aclbinackena*/ 0, /*escobinackena*/ 0, /*duplfltena*/ 0,
#endif // EAVESDROPPING_SUPPORT
                        cs_idx);

    // Default clear other control
    em_bt_wincntl_set(cs_idx, 0);
    em_bt_chmap0_set(cs_idx, 0);
    em_bt_chmap1_set(cs_idx, 0);
    em_bt_chmap2_set(cs_idx, 0);
    em_bt_chmap3_set(cs_idx, 0);
    em_bt_chmap4_set(cs_idx, 0);
    em_bt_loopcntl_set(cs_idx, 0);
    em_bt_maxfrmtime_set(cs_idx, 0);
    rwip_rf.txpwr_cs_set(TYPE_BT, cs_idx, rwip_rf.txpwr_max_mod, rwip_rf.txpwr_max);
#if (EAVESDROPPING_SUPPORT)
    em_bt_cidcntl_set(cs_idx, 0);
    em_bt_rxcrc_set(cs_idx, 0);
#endif // EAVESDROPPING_SUPPORT
}

/**
 ****************************************************************************************
 * @brief Check the reception during activity
 ****************************************************************************************
 */
__STATIC bool ld_pscan_fhs_rx(struct bd_addr *bd_addr_ptr, struct devclass *class_of_dev_ptr, uint8_t *lt_addr_ptr, uint32_t *clk_off_ptr)
{
    bool fhs_received = false;

    // Point to parameters
    struct ld_pscan_env_tag *pscan_par = ld_pscan_env;

    FUNC_PATCH_ENTRY_4_PARAM_HAVE_RETURN(LD_PSCAN_PATCH_TYPE, LD_PSCAN_FHS_RX_BIT, bool, bd_addr_ptr, class_of_dev_ptr, lt_addr_ptr, clk_off_ptr);

    // Check if FHS has been received
    if (ld_rxdesc_check(EM_BT_CS_ACL_INDEX(pscan_par->link_id)))
    {
        uint8_t cs_idx = EM_BT_CS_ACL_INDEX(pscan_par->link_id);
        // Get current RX descriptor index
        uint8_t index_fhs = ld_env.curr_rxdesc_index;
        // Retrieve RX status and type
        uint16_t rxstat_fhs = em_bt_rxstat_get(index_fhs);

        // Check if FHS reception is correct
        if ((rxstat_fhs & (EM_BT_RXSYNCERR_BIT | EM_BT_RXHECERR_BIT | EM_BT_RXCRCERR_BIT | EM_BT_RXBADLT_BIT)) == 0)
        {
            uint32_t fhs_clk_frm;
            uint32_t rx_clk_hslot;

            // Point to FHS payload buffer
            uint16_t fhs_buf_ptr = em_bt_rxlmbufptr_getf(index_fhs);

            ASSERT_INFO(em_bt_rxheader_rxtype_getf(index_fhs) == FHS_TYPE, em_bt_rxheader_rxtype_getf(index_fhs), 0);

            // Extract fields from packet
            ld_util_fhs_unpk(fhs_buf_ptr, &pscan_par->bch_1st_pkt[0], bd_addr_ptr, class_of_dev_ptr, lt_addr_ptr, &fhs_clk_frm, NULL);

            // Read RX clock (in half-slots)
            rx_clk_hslot = (em_bt_rxclkn1_getf(cs_idx) << 16) | em_bt_rxclkn0_getf(cs_idx);

            // Compute clock offset (in half-slots)
            *clk_off_ptr = CLK_SUB((fhs_clk_frm << 2), rx_clk_hslot);

            // Indicate successful reception
            fhs_received = true;
        }

        // Free RX descriptor
        ld_rxdesc_free();
    }

    return (fhs_received);
}

/**
 ****************************************************************************************
 * @brief Check the reception during activity
 ****************************************************************************************
 */
__STATIC bool ld_pscan_1st_pkt_rx(void)
{
    bool poll_received = false;

    // Point to parameters
    struct ld_pscan_env_tag *pscan_par = ld_pscan_env;

    FUNC_PATCH_ENTRY_NO_PARAM_HAVE_RETURN(LD_PSCAN_PATCH_TYPE, LD_PSCAN_1ST_PKT_RX_BIT, bool);

    // TODO [DAD]: Workaround for the reception loop that is not expected to consume several descriptors - to remove when TLM fixed
    ASSERT_INFO(em_bt_rxdesccnt_aclrxdesccnt_getf(pscan_par->link_id) <= 1, em_bt_rxdesccnt_aclrxdesccnt_getf(pscan_par->link_id), pscan_par->link_id);

    // Check if 1st master packet has been received
    if (ld_rxdesc_check(EM_BT_CS_ACL_INDEX(pscan_par->link_id)))
    {
        // Get current RX descriptor index
        uint8_t index_1st_pkt = ld_env.curr_rxdesc_index;
        // Retrieve RX status and type
        uint16_t rxstat = em_bt_rxstat_get(index_1st_pkt);
        uint8_t rxtype = em_bt_rxheader_rxtype_getf(index_1st_pkt);

        // Check if Poll reception is correct
        if (((rxstat & (EM_BT_RXSYNCERR_BIT | EM_BT_RXHECERR_BIT)) == 0) && (rxtype == POLL_TYPE))
        {
            // Indicate successful reception
            poll_received = true;
        }

        // Free RX descriptor
        ld_rxdesc_free();
    }

    return (poll_received);
}

/**
 ****************************************************************************************
 * @brief Report the end of the activity
 ****************************************************************************************
 */
__STATIC void ld_pscan_end(bool connected)
{
    // Point to parameters
    struct ld_pscan_env_tag *pscan_par = ld_pscan_env;
    uint8_t cs_idx = EM_BT_CS_ACL_INDEX(pscan_par->link_id);

    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_PSCAN_PATCH_TYPE, LD_PSCAN_END_BIT, connected);

    // Report page scan end to LM
    struct lm_page_scan_end_ind *ind = KE_MSG_ALLOC(LM_PAGE_SCAN_END_IND, TASK_LM, TASK_NONE, lm_page_scan_end_ind);
    ind->link_id = pscan_par->link_id;

    if (connected)
    {
        ind->peer_bd_addr = pscan_par->peer_bd_addr;
        ind->class_of_dev = pscan_par->class_of_dev;
        ind->slave_timing_info.last_sync_ts = (em_bt_rxclkn1_getf(cs_idx) << 16) |  em_bt_rxclkn0_getf(cs_idx);;
        ind->slave_timing_info.last_sync_clk_off = (em_bt_clkoff1_getf(cs_idx) << 16) | em_bt_clkoff0_getf(cs_idx);
        ind->slave_timing_info.last_sync_bit_off = pscan_par->evt.time.hus;
    }

    ke_msg_send(ind);

#if (EAVESDROPPING_SUPPORT)
    if (ld_pscan_seg_scan_env.seg_scan_state == SEG_SCAN_ON)
    {
        // Clear segmented scan
        ld_pscan_seg_scan_env.seg_scan_state = SEG_SCAN_ENABLING;
    }
    else if (ld_pscan_seg_scan_env.seg_scan_state == SEG_SCAN_DISABLING)
    {
        // Clear segmented scan
        ld_pscan_seg_scan_env.seg_scan_state = SEG_SCAN_OFF;
    }
#endif // EAVESDROPPING_SUPPORT

    // Remove permission/status of CS as now unused
    DBG_MEM_PERM_SET((const void *)(REG_EM_BT_CS_BASE_ADDR + REG_EM_BT_CS_ADDR_GET(EM_BT_CS_ACL_INDEX(pscan_par->link_id))), REG_EM_BT_CS_SIZE, false, false, false);

    // Unregister the scan window from scheduling parameters
    sch_slice_bg_remove(BT_PSCAN);

    // Free event memory
    ke_free(ld_pscan_env);
    ld_pscan_env = NULL;
}

/**
 ****************************************************************************************
 * @brief Program the activity
 ****************************************************************************************
 */
__STATIC void ld_pscan_prog(uint32_t clock)
{
    // Point to parameters
    struct ld_pscan_env_tag *pscan_par = ld_pscan_env;
    struct sch_arb_elt_tag *evt = &pscan_par->evt;
    uint8_t cs_idx = EM_BT_CS_ACL_INDEX(pscan_par->link_id);

    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_PSCAN_PATCH_TYPE, LD_PSCAN_PROG_BIT, clock);

    switch (pscan_par->step)
    {
    case PSCAN_STEP_PSCAN:
    {
#if (EAVESDROPPING_SUPPORT)
        if ((ld_pscan_seg_scan_env.seg_scan_state == SEG_SCAN_ON) || (ld_pscan_seg_scan_env.seg_scan_state == SEG_SCAN_DISABLING))
        {
            uint8_t phase_offset = ld_pscan_seg_scan_env.seg_scan.phase_offsets[ld_pscan_seg_scan_env.seg_cnt];

            // Set window duration
            em_bt_wincntl_pack(cs_idx, 0, ld_pscan_seg_scan_env.seg_scan.duration >> 2);

            // Set phase offset for the segment
            em_bt_clkoff0_setf(cs_idx, (phase_offset << 11) & EM_BT_CLKOFF0_MASK);
            em_bt_clkoff1_setf(cs_idx, (phase_offset << 11) >> 16);
        }
        else
#endif // EAVESDROPPING_SUPPORT
        {
            // Check to not exceed the maximum value supported by HW
            uint32_t max_win_len_hs = co_min(2 * pscan_par->win_rem_slots, (EM_BT_RXWINSZ_MASK >> EM_BT_RXWINSZ_LSB));

            // Set window duration
            em_bt_wincntl_pack(cs_idx, 1, (max_win_len_hs + 1) >> 1);
        }

        // Prepare CS
        em_bt_frcntl_format_setf(cs_idx, EM_BT_CS_FMT_PAGE_SCAN);
        em_bt_loopcntl_att_nb_setf(cs_idx, PAGE_RESP_TO_DEF / 2);
    }
    break;
    case PSCAN_STEP_1ST_PKT:
    {
        uint8_t att_nb = CLK_SUB(ld_pscan_env->evt.asap_limit, ld_pscan_env->evt.time.hs) >> 1;

        // Prepare CS
        em_bt_loopcntl_att_nb_setf(cs_idx, att_nb / 2);
    }
    break;
    default:
    {
        // Nothing to do
        ASSERT_INFO_FORCE(0, pscan_par->step, 0);
    }
    break;
    }

    {
        // Push the programming to SCH PROG
        struct sch_prog_params prog_par;
        prog_par.frm_cbk         = &ld_pscan_frm_cbk;
        prog_par.time.hs         = clock;
        prog_par.time.hus        = evt->time.hus;
        prog_par.cs_idx          = cs_idx;
        prog_par.dummy           = cs_idx;
        prog_par.bandwidth       = evt->duration_min;
        prog_par.prio_1          = evt->current_prio;
        prog_par.prio_2          = 0;
        prog_par.prio_3          = rwip_priority[RWIP_PRIO_PSCAN_1ST_PKT_IDX].value;
        prog_par.pti_prio        = BT_PTI_PSCAN_IDX;
        prog_par.add.bt.frm_type = SCH_BT_FRAME_TYPE_NORMAL;
        prog_par.mode            = SCH_PROG_BT;
        sch_prog_push(&prog_par);
    }
}

/**
 ****************************************************************************************
 * @brief Reschedule page scan
 ****************************************************************************************
 */
__STATIC void ld_pscan_resched(void)
{
    // Point to parameters
    struct sch_arb_elt_tag *evt = &(ld_pscan_env->evt);
    struct ld_pscan_env_tag *pscan_par = ld_pscan_env;
    uint8_t phase_offset = 0;

    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LD_PSCAN_PATCH_TYPE, LD_PSCAN_RESCHED_BIT);

#if (EAVESDROPPING_SUPPORT)
    if (ld_pscan_seg_scan_env.seg_scan_state != SEG_SCAN_OFF)
    {
        switch (ld_pscan_seg_scan_env.seg_scan_state)
        {
        case SEG_SCAN_ENABLING:
        {
            evt->time.hs = ld_read_clock();

            // Set the first segment timings
            ld_pscan_set_seg_scan_timings(evt);

            if (ld_pscan_seg_scan_env.seg_scan.segments > 1)
            {
                // Time from the first segment until the next toggling of CLKN[12]
                uint16_t time_to_toggling = BIT11 - (CLK_ADD_2(evt->time.hs, ld_pscan_seg_scan_env.seg_scan.distance) % BIT11);
                if (time_to_toggling < ld_pscan_seg_scan_env.seg_scan.segments * ld_pscan_seg_scan_env.seg_scan.distance)
                {
                    // Not enough space to fit all segments before the next toggling:
                    // Schedule the first segment to occur shortly after the next toggling
                    uint16_t align = time_to_toggling % ld_pscan_seg_scan_env.seg_scan.periodicity;
                    evt->time.hs = CLK_ADD_3(evt->time.hs, time_to_toggling, align ? (ld_pscan_seg_scan_env.seg_scan.periodicity - align) : 0);
                }
            }

            ld_pscan_seg_scan_env.first_seg_ts = evt->time.hs;
            ld_pscan_seg_scan_env.last_seg_ts = evt->time.hs;

            // Set the event as strictly periodic
            SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_NO_ASAP, SCH_ARB_NO_PHASE, 0, 0);

            // Set the event duration including scan window and extra time for a potential page response procedure
            evt->duration_min = ld_pscan_seg_scan_env.seg_scan.duration + 2 * HALF_SLOT_SIZE;

            // Initialize segment counter
            ld_pscan_seg_scan_env.seg_cnt = 0;

            // Set as active
            ld_pscan_seg_scan_env.seg_scan_state = SEG_SCAN_ON;

        } // No break

        case SEG_SCAN_ON:
        {
            // Restore the timestamp of last scheduled segment
            evt->time.hs = ld_pscan_seg_scan_env.last_seg_ts;

            // Schedule the next window
            do
            {
                // Increment segment counter
                if (ld_pscan_seg_scan_env.seg_cnt++ == (ld_pscan_seg_scan_env.seg_scan.segments - 1))
                {
                    // Reset segment counter
                    ld_pscan_seg_scan_env.seg_cnt = 0;

                    // Go to next scan interval
                    ld_pscan_seg_scan_env.first_seg_ts = CLK_ADD_2(ld_pscan_seg_scan_env.first_seg_ts, pscan_par->pscan_intv);
                    evt->time.hs = ld_pscan_seg_scan_env.first_seg_ts;
                }
                else
                {
                    // Go to next segment
                    evt->time.hs = CLK_ADD_2(evt->time.hs, ld_pscan_seg_scan_env.seg_scan.distance);
                }

                // Set the segment timings
                ld_pscan_set_seg_scan_timings(evt);

            }
            while (sch_arb_insert(evt) != SCH_ARB_ERROR_OK);

            // Save the timestamp, to serve as a base for placing next segment
            ld_pscan_seg_scan_env.last_seg_ts = evt->time.hs;
        }
        break;

        case SEG_SCAN_DISABLING:
        {
            // Restore initial scan parameters
            pscan_par->win_rem_slots     = pscan_par->pscan_win;
            pscan_par->interlace_state   = PSCAN_INTER_OFF;
            evt->duration_min            = sch_slice_params.scan_evt_dur;
            evt->time.hus                = 0;
            evt->current_prio            = rwip_priority[RWIP_PRIO_PSCAN_DFT_IDX].value;
            SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_ASAP_NO_LIMIT, SCH_ARB_NO_PHASE, LD_PSCAN_EVT_AUTO_RESCHED_ATT, RWIP_PRIO_INC(RWIP_PRIO_PSCAN_DFT_IDX));

            // Schedule event ASAP
            evt->time.hs = rwip_time_get().hs;
            if (sch_arb_insert(evt) != SCH_ARB_ERROR_OK)
            {
                ASSERT_ERR_FORCE(0);
            }

            // Set as inactive
            ld_pscan_seg_scan_env.seg_scan_state = SEG_SCAN_OFF;
        }
        break;

        default:
        {
            ASSERT_INFO_FORCE(0, ld_pscan_seg_scan_env.seg_scan_state, 0);
        }
        break;
        }
    }
    else
#endif // EAVESDROPPING_SUPPORT
    {
        // Check if current page scan window is finished
        if (pscan_par->win_rem_slots == 0)
        {
            uint8_t cs_idx = EM_BT_CS_ACL_INDEX(pscan_par->link_id);

            uint32_t pscan_win = pscan_par->pscan_win;

            // The delay and frequency of the next window depends on the interlace scan state
            switch (pscan_par->interlace_state)
            {
            case PSCAN_INTER_WIN0:
            {
                // Increment the phase in the page hop sequence by the interlace offset
                pscan_par->interlace_state = PSCAN_INTER_WIN1;

#if RW_BT_MWS_COEX // Generalized Interlaced Scan Support - enabled for PSCAN_INTER_WIN1
                if (ld_pscan_mwscoex_xi_mask && (ld_mws_inactivity_duration_get() << 3) < pscan_par->pscan_win) // 5ms units * 8 < pscan_win slots
                {
                    // Control for hop system in generalized interlaced scan: X = (CLKN16:12 + interlace offset) mod 32
                    uint32_t clk_16to12 = (ld_read_clock() >> 12) & 0x1F;
                    uint8_t Xi = ld_pscan_mwscoex_xi_get((clk_16to12 + INTERLACE_OFFSET_DFT) & 0x1F);
                    phase_offset = (32 + Xi - clk_16to12) & 0x1F;
                    // Coex does not prevent Tx/Rx
                    em_bt_frcntl_dnabort_setf(cs_idx, 1);
                }
                // Adjust for unavailable slots
                else if (ld_env.mws_ext_fr_scan_dur < ld_env.mws_ext_fr_duration)
                {
                    // Coex prevents Tx/Rx, redimension scan window
                    pscan_win = (pscan_win * ld_env.mws_ext_fr_duration) / ld_env.mws_ext_fr_scan_dur;
                    em_bt_frcntl_dnabort_setf(cs_idx, 0);
                }
                else
#endif // RW_BT_MWS_COEX
                {
                    phase_offset = INTERLACE_OFFSET_DFT;
                }

                // Register the scan window as active to scheduling parameters
                sch_slice_bg_add(BT_PSCAN);
            }
            break;
            case PSCAN_INTER_WIN1:
            {
                // Clear the phase in the page hop sequence
                pscan_par->interlace_state = PSCAN_INTER_WIN0;

            } // No break
            case PSCAN_INTER_OFF:
            {
                uint32_t clock = ld_read_clock();

                // Add scan window period
                int32_t delay = pscan_par->pscan_intv - pscan_win;
                evt->time.hs = CO_ALIGN4_HI(CLK_ADD_2(clock, 2 * delay));

                // Unregister the scan window from scheduling parameters
                sch_slice_bg_remove(BT_PSCAN);
            }
            break;
            default:
            {
                ASSERT_ERR_FORCE(0);
            }
            break;
            }

            // Reload window length
            pscan_par->win_rem_slots = pscan_win;

            // Apply phase in the page scan hop sequence (BB:8.4.3)
            // Update clock offset (used for the page hopping sequence)
            em_bt_clkoff0_setf(cs_idx, (phase_offset << 12) & EM_BT_CLKOFF0_MASK);
            em_bt_clkoff1_setf(cs_idx, (phase_offset >> 4) & EM_BT_CLKOFF1_MASK);
        }
        else
        {
            // Register the scan window as active to scheduling parameters
            sch_slice_bg_add(BT_PSCAN);
        }

        // Refresh event duration
        evt->duration_min = co_min(2 * pscan_par->win_rem_slots * SLOT_SIZE, sch_slice_params.scan_evt_dur);

        // Reschedule
        if (sch_arb_insert(&(ld_pscan_env->evt)) != SCH_ARB_ERROR_OK)
        {
            ASSERT_ERR_FORCE(0);
        }
    }

    pscan_par->state = PSCAN_EVT_WAIT;
}

/**
 ****************************************************************************************
 * @brief Handle event start notification
 ****************************************************************************************
 */
__STATIC void ld_pscan_evt_start_cbk(struct sch_arb_elt_tag *evt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_PSCAN_PATCH_TYPE, LD_PSCAN_EVT_START_CBK_BIT, evt);

    DBG_SWDIAG(PAGE, PSCAN_EVT_START, 1);

    ASSERT_ERR((&(ld_pscan_env->evt)) == evt);

    if (ld_pscan_env != NULL)
    {
        // Point to parameters
        struct ld_pscan_env_tag *pscan_par = ld_pscan_env;

        // Program page scan
        ld_pscan_prog(evt->time.hs);

        // Move state
        pscan_par->state = PSCAN_EVT_ACTIVE;
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(PAGE, PSCAN_EVT_START, 0);
}

/**
 ****************************************************************************************
 * @brief Handle event canceled notification
 ****************************************************************************************
 */
__STATIC void ld_pscan_evt_canceled_cbk(struct sch_arb_elt_tag *evt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_PSCAN_PATCH_TYPE, LD_PSCAN_EVT_CANCELED_CBK_BIT, evt);

    DBG_SWDIAG(PAGE, PSCAN_EVT_CANCELED, 1);

    ASSERT_ERR((&(ld_pscan_env->evt)) == evt);

    if (evt != NULL)
    {
        // Point to parameters
        struct ld_pscan_env_tag *pscan_par = ld_pscan_env;

        // Check page scan end
        if (pscan_par->state == PSCAN_EVT_END)
        {
#if (EAVESDROPPING_SUPPORT)
            if (ld_pscan_seg_scan_env.seg_scan_state == SEG_SCAN_ON)
            {
                // Clear segmented scan
                ld_pscan_seg_scan_env.seg_scan_state = SEG_SCAN_ENABLING;
            }
            else if (ld_pscan_seg_scan_env.seg_scan_state == SEG_SCAN_DISABLING)
            {
                // Clear segmented scan
                ld_pscan_seg_scan_env.seg_scan_state = SEG_SCAN_OFF;
            }
#endif // EAVESDROPPING_SUPPORT

            // End page scan
            ld_pscan_end(false);
        }
        else
        {
            // Check Page Scan step
            switch (pscan_par->step)
            {
            case PSCAN_STEP_PSCAN:
            {
                // Increment priority
                evt->current_prio = RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(RWIP_PRIO_PSCAN_DFT_IDX));

                // Reload the automatic rescheduling attempts
                SCH_ARB_ASAP_STG_RESCHED_ATT_SET((&(ld_pscan_env->evt)), LD_PSCAN_EVT_AUTO_RESCHED_ATT);

                // Reschedule page scan
                ld_pscan_resched();
            }
            break;
            case PSCAN_STEP_1ST_PKT:
            {
                // Increment priority
                evt->current_prio = RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(RWIP_PRIO_PSCAN_1ST_PKT_IDX));

                // Reload the automatic rescheduling attempts
                SCH_ARB_ASAP_STG_RESCHED_ATT_SET((&(ld_pscan_env->evt)), LD_PSCAN_1ST_PKT_EVT_AUTO_RESCHED_ATT);

                // Try to schedule 1st packet
                if (sch_arb_insert(evt) != SCH_ARB_ERROR_OK)
                {
                    // Failed to schedule before the new connection end timestamp, return to page scan step
                    pscan_par->step = PSCAN_STEP_PSCAN;

                    // Reset event settings
                    evt->current_prio = rwip_priority[RWIP_PRIO_PSCAN_DFT_IDX].value;
                    SCH_ARB_ASAP_STG_SET((&(ld_pscan_env->evt)), SCH_ARB_FLAG_ASAP_NO_LIMIT, SCH_ARB_NO_PHASE, LD_PSCAN_EVT_AUTO_RESCHED_ATT, RWIP_PRIO_INC(RWIP_PRIO_PSCAN_DFT_IDX));

                    // Initialize EM
                    ld_pscan_em_init(ld_pscan_env);

                    // Reschedule page scan
                    ld_pscan_resched();
                }
            }
            break;
            default:
            {
                // Nothing to do
                ASSERT_INFO_FORCE(0, pscan_par->step, 0);
            }
            break;
            }
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(PAGE, PSCAN_EVT_CANCELED, 0);
}

/**
 ****************************************************************************************
 * @brief Handle frame interrupt
 ****************************************************************************************
 */
__STATIC void ld_pscan_frm_isr(uint32_t timestamp)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_PSCAN_PATCH_TYPE, LD_PSCAN_FRM_ISR_BIT, timestamp);

    DBG_SWDIAG(PAGE, PSCAN_FRM_ISR, 1);

    if (ld_pscan_env != NULL)
    {
        // Point to parameters
        struct ld_pscan_env_tag *pscan_par = ld_pscan_env;

        // Remove event
        sch_arb_remove(&(ld_pscan_env->evt), true);

        // Check page scan end
        if (pscan_par->state == PSCAN_EVT_END)
        {
#if (EAVESDROPPING_SUPPORT)
            if (ld_pscan_seg_scan_env.seg_scan_state == SEG_SCAN_ON)
            {
                // Clear segmented scan
                ld_pscan_seg_scan_env.seg_scan_state = SEG_SCAN_ENABLING;
            }
            else if (ld_pscan_seg_scan_env.seg_scan_state == SEG_SCAN_DISABLING)
            {
                // Clear segmented scan
                ld_pscan_seg_scan_env.seg_scan_state = SEG_SCAN_OFF;
            }
#endif // EAVESDROPPING_SUPPORT

            // Flush a potentially consumed descriptor
            if (ld_rxdesc_check(EM_BT_CS_ACL_INDEX(pscan_par->link_id)))
            {
                // Free RX descriptor
                ld_rxdesc_free();
            }

            // End page scan
            ld_pscan_end(false);
        }
        else
        {
            uint32_t clock = ld_read_clock();

            // Update timestamp to current clock
            ld_pscan_env->evt.time.hs = clock;

            // Move to wait state
            pscan_par->state = PSCAN_EVT_WAIT;

            // Check Page Scan step
            switch (pscan_par->step)
            {
            case PSCAN_STEP_PSCAN:
            {
                uint8_t cs_idx = EM_BT_CS_ACL_INDEX(pscan_par->link_id);
                uint32_t clk_off = 0;
                uint8_t lt_addr = 0;
                uint32_t effective_slots = CLK_SUB(clock, timestamp) >> 1;

                if (effective_slots <= pscan_par->win_rem_slots)
                {
                    // Subtract the number of effective scanning slots
                    pscan_par->win_rem_slots -= effective_slots;
                }
                else
                {
                    pscan_par->win_rem_slots = 0;
                }

                // Check if FHS has been received
                if (ld_pscan_fhs_rx(&pscan_par->peer_bd_addr, &pscan_par->class_of_dev, &lt_addr, &clk_off))
                {
                    int16_t bit_off = 0;

                    // Calculate the bit offset and adjust the clock offset accordingly
                    bit_off = HALF_SLOT_TIME_MAX - em_bt_rxbit_rxbit_getf(cs_idx) - 2 * ld_env.exp_sync_pos;
                    if (bit_off < 0)
                    {
                        bit_off += HALF_SLOT_SIZE;
                        clk_off = CLK_ADD_2(clk_off, 1);
                    }

                    em_bt_linkcntl_aclltaddr_setf(cs_idx, lt_addr);
                    em_bt_clkoff0_setf(cs_idx, clk_off & EM_BT_CLKOFF0_MASK);
                    em_bt_clkoff1_setf(cs_idx, (clk_off >> 16) & EM_BT_CLKOFF1_MASK);

                    // Store the bit offset
                    pscan_par->evt.time.hus = bit_off;

                    // Store the phase
                    pscan_par->phase = (4 - (clk_off & 0x03)) & 0x03;

                    // Change event settings
                    ld_pscan_env->evt.current_prio = rwip_priority[RWIP_PRIO_PSCAN_1ST_PKT_IDX].value;
                    ld_pscan_env->evt.duration_min = 2 * LD_PSCAN_1ST_PKT_EVT_DUR_MIN;
                    SCH_ARB_ASAP_STG_SET((&ld_pscan_env->evt), SCH_ARB_FLAG_ASAP_LIMIT, pscan_par->phase, LD_PSCAN_1ST_PKT_EVT_AUTO_RESCHED_ATT, RWIP_PRIO_INC(RWIP_PRIO_PSCAN_1ST_PKT_IDX));
                    // Set the new connection timeout (32 slots)
                    ld_pscan_env->evt.asap_limit = CLK_ADD_2(clock, 2 * (NEW_CONNECTION_TO + 1));

                    // Try to schedule before the new connection end timestamp
                    if (sch_arb_insert(&(ld_pscan_env->evt)) == SCH_ARB_ERROR_OK)
                    {
                        // Set control structure fields
                        em_bt_frcntl_format_setf(cs_idx, EM_BT_CS_FMT_SLV_CONNECT);
                        em_bt_bdaddr_setf(cs_idx, 0, (pscan_par->peer_bd_addr.addr[1] << 8) | pscan_par->peer_bd_addr.addr[0]);
                        em_bt_bdaddr_setf(cs_idx, 1, (pscan_par->peer_bd_addr.addr[3] << 8) | pscan_par->peer_bd_addr.addr[2]);
                        em_bt_bdaddr_setf(cs_idx, 2, (pscan_par->peer_bd_addr.addr[5] << 8) | pscan_par->peer_bd_addr.addr[4]);
                        em_bt_bch0_setf(cs_idx, (pscan_par->bch_1st_pkt[1] << 8) | pscan_par->bch_1st_pkt[0]);
                        em_bt_bch1_setf(cs_idx, (pscan_par->bch_1st_pkt[3] << 8) | pscan_par->bch_1st_pkt[2]);
                        em_bt_rxmaxbuf_bch2_pack(cs_idx, 0, 0, pscan_par->bch_1st_pkt[4]  & EM_BT_BCH2_MASK);
                        em_bt_linkcntl_afhena_setf(cs_idx, 0);
                        em_bt_pwrcntl_fh_en_setf(cs_idx, 1);
                        em_bt_txrxcntl_txeir_setf(cs_idx, 0);
                        em_bt_txdescptr_setf(cs_idx, 0);
                        em_bt_wincntl_pack(cs_idx, 0, NORMAL_WIN_SIZE);
                        em_bt_acltxstat_fnak_setf(cs_idx, 1);
                        em_bt_acltxstat_fcrc_setf(cs_idx, 1);

                        // Move step to 1st Packet
                        pscan_par->step = PSCAN_STEP_1ST_PKT;
                    }
                    else
                    {
                        // Failed to schedule before the new connection end timestamp, reset event settings
                        ld_pscan_env->evt.current_prio = rwip_priority[RWIP_PRIO_PSCAN_DFT_IDX].value;
                        SCH_ARB_ASAP_STG_SET((&(ld_pscan_env->evt)), SCH_ARB_FLAG_ASAP_NO_LIMIT, SCH_ARB_NO_PHASE, LD_PSCAN_EVT_AUTO_RESCHED_ATT, RWIP_PRIO_INC(RWIP_PRIO_PSCAN_DFT_IDX));

                        // Restore clock offset and bit offset
                        em_bt_clkoff0_setf(cs_idx, 0);
                        em_bt_clkoff1_setf(cs_idx, 0);
                        pscan_par->evt.time.hus = 0;

                        // Reschedule page scan
                        ld_pscan_resched();
                    }

                    break;
                }

#if CSB_SUPPORT
                // Check if page response timeout
                if (em_bt_frcntl_format_getf(cs_idx) == EM_BT_CS_FMT_SLV_PAGE_RSP)
                {
                    // Report page response timeout to LM
                    ke_msg_send_basic(LM_PAGE_RESP_TO_IND, TASK_LM, TASK_NONE);
                }
#endif //CSB_SUPPORT

                // Restore original page scan priority
                ld_pscan_env->evt.current_prio = rwip_priority[RWIP_PRIO_PSCAN_DFT_IDX].value;

                // Reschedule page scan
                ld_pscan_resched();
            }
            break;
            case PSCAN_STEP_1ST_PKT:
            {
                // Check if 1st Master packet has been received
                if (ld_pscan_1st_pkt_rx())
                {
                    // Report page scan end
                    ld_pscan_end(true);
                    break;
                }

                // Try to schedule before the new connection end timestamp
                if (sch_arb_insert(&(ld_pscan_env->evt)) == SCH_ARB_ERROR_OK)
                {
                    break;
                }

                // Failed to schedule before the new connection end timestamp, return to page scan step
                pscan_par->step = PSCAN_STEP_PSCAN;

                // Reset event settings
                ld_pscan_env->evt.current_prio = rwip_priority[RWIP_PRIO_PSCAN_DFT_IDX].value;
                SCH_ARB_ASAP_STG_SET((&ld_pscan_env->evt), SCH_ARB_FLAG_ASAP_NO_LIMIT, SCH_ARB_NO_PHASE, LD_PSCAN_EVT_AUTO_RESCHED_ATT, RWIP_PRIO_INC(RWIP_PRIO_PSCAN_DFT_IDX));

                // Initialize EM
                ld_pscan_em_init(ld_pscan_env);

                // Reschedule page scan
                ld_pscan_resched();
            }
            break;
            default:
            {
                // Nothing to do
                ASSERT_INFO_FORCE(0, pscan_par->step, 0);
            }
            break;
            }
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(PAGE, PSCAN_FRM_ISR, 0);
}

/**
 ****************************************************************************************
 * @brief Handle frame interrupt notification
 ****************************************************************************************
 */
__STATIC void ld_pscan_frm_cbk(uint32_t timestamp, uint32_t dummy, uint8_t irq_type)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LD_PSCAN_PATCH_TYPE, LD_PSCAN_FRM_CBK_BIT, timestamp, dummy, irq_type);

    switch (irq_type)
    {
    case SCH_FRAME_IRQ_EOF:
    case SCH_FRAME_IRQ_EOF_ABORT_UNDER_PRIO:
    case SCH_FRAME_IRQ_EOF_ABORT_AFTER_PRIO:
    {
        ld_pscan_frm_isr(timestamp);
    }
    break;
    case SCH_FRAME_IRQ_SKIP:
    {
        // Point to parameters
        struct sch_arb_elt_tag *evt = &(ld_pscan_env->evt);

        // Remove event
        sch_arb_remove(evt, true);

        // Invoke event cancel callback
        ld_pscan_evt_canceled_cbk(&(ld_pscan_env->evt));
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
 * @brief Get a suitable X-input for page scan based on MWS mask
 ****************************************************************************************
 */
__STATIC uint8_t ld_pscan_mwscoex_xi_get(uint8_t xi_nom)
{
    uint32_t pscan_mask;
    uint8_t xi;

    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LD_PSCAN_PATCH_TYPE, LD_PSCAN_MWSCOEX_XI_GET_BIT, uint8_t, xi_nom);

    // search higher bits for suitable X input
    pscan_mask = ld_pscan_mwscoex_xi_mask >> xi_nom;
    for (xi = xi_nom; (pscan_mask & BIT0) && (xi < 32); xi++)
    {
        pscan_mask >>= 1;
    }

    if (xi == 32) // searched all higher bits
    {
        //search lower bits for suitable X input
        pscan_mask = ld_pscan_mwscoex_xi_mask;
        for (xi = 0; (pscan_mask & BIT0) && (xi <= xi_nom); xi++)
        {
            pscan_mask >>= 1;
        }
    }

    return xi;
}
#endif // RW_BT_MWS_COEX


/*
 * EXPORTED FUNCTIONS DEFINITION
 *****************************************************************************************
 */

void ld_pscan_init(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LD_PSCAN_PATCH_TYPE, LD_PSCAN_INIT_BIT);

    ld_pscan_env = NULL;

#if (EAVESDROPPING_SUPPORT)
    memset(&ld_pscan_seg_scan_env, 0, sizeof(ld_pscan_seg_scan_env));
#endif // EAVESDROPPING_SUPPORT

#if RW_BT_MWS_COEX
    ld_pscan_mwscoex_xi_mask = 0;
#endif // RW_BT_MWS_COEX
}

void ld_pscan_reset(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LD_PSCAN_PATCH_TYPE, LD_PSCAN_RESET_BIT);

    // Check if page scan is active
    if (ld_pscan_env != NULL)
    {
        // Free event memory
        ke_free(ld_pscan_env);
        ld_pscan_env = NULL;
    }

#if (EAVESDROPPING_SUPPORT)
    memset(&ld_pscan_seg_scan_env, 0, sizeof(ld_pscan_seg_scan_env));
#endif // EAVESDROPPING_SUPPORT

#if RW_BT_MWS_COEX
    ld_pscan_mwscoex_xi_mask = 0;
#endif // RW_BT_MWS_COEX
}

uint8_t ld_pscan_start(struct ld_page_scan_params *params)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    struct ld_pscan_env_tag *pscan_par;

    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LD_PSCAN_PATCH_TYPE, LD_PSCAN_START_BIT, uint8_t, params);

    // Check if page scan is inactive
    if (ld_pscan_env == NULL)
    {
        // Allocate event
        pscan_par = LD_ALLOC_EVT(ld_pscan_env_tag);

        if (pscan_par != NULL)
        {
            uint32_t clock = ld_read_clock();

            // Point to parameters
            struct sch_arb_elt_tag *evt = &(pscan_par->evt);

            LD_INIT_EVT(evt, ld_pscan_env_tag);

            // Initialize event parameters (common part)
            evt->cb_cancel        = &ld_pscan_evt_canceled_cbk;
            evt->cb_start         = &ld_pscan_evt_start_cbk;
            evt->cb_stop          = NULL;
            evt->current_prio        = rwip_priority[RWIP_PRIO_PSCAN_DFT_IDX].value;
            SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_ASAP_NO_LIMIT, SCH_ARB_NO_PHASE, LD_PSCAN_EVT_AUTO_RESCHED_ATT, RWIP_PRIO_INC(RWIP_PRIO_PSCAN_DFT_IDX));

            // Initialize event parameters (page scan part)
            pscan_par->peer_bd_addr.addr[0] = 0;
            pscan_par->peer_bd_addr.addr[1] = 0;
            pscan_par->peer_bd_addr.addr[2] = 0;
            pscan_par->peer_bd_addr.addr[3] = 0;
            pscan_par->peer_bd_addr.addr[4] = 0;
            pscan_par->peer_bd_addr.addr[5] = 0;
            pscan_par->link_id        = params->link_id;
            pscan_par->pscan_intv = params->pscan_intv;
            pscan_par->pscan_win = params->pscan_win;
            pscan_par->step             = PSCAN_STEP_PSCAN;
            pscan_par->win_rem_slots     = pscan_par->pscan_win;
            pscan_par->interlace_state = PSCAN_INTER_OFF;
            if ((params->pscan_intv >= (2 * params->pscan_win)) && (params->pscan_type == INTERLACED_SCAN))
            {
                pscan_par->interlace_state = PSCAN_INTER_WIN0;
            }

            // Initialize EM
            ld_pscan_em_init(pscan_par);

            GLOBAL_INT_DISABLE();

            // Assign configured parameters
            ld_pscan_env = pscan_par;

#if (EAVESDROPPING_SUPPORT)
            if (ld_pscan_seg_scan_env.seg_scan_state != SEG_SCAN_OFF)
            {
                ld_pscan_resched();
                status = CO_ERROR_NO_ERROR;
            }
            else
#endif // EAVESDROPPING_SUPPORT
            {
                evt->time.hs    = clock + 2 * params->pscan_intv; // Delay the first window to reduce conflict with slave ACL establishment
                evt->duration_min = co_min(2 * pscan_par->win_rem_slots * SLOT_SIZE, sch_slice_params.scan_evt_dur);
                evt->time.hus = 0;
                // Try to schedule page scan with no time limit
                if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
                {
                    pscan_par->state = PSCAN_EVT_WAIT;
                    status = CO_ERROR_NO_ERROR;
                }
                else
                {
                    ASSERT_ERR_FORCE(0);
                }
            }

            GLOBAL_INT_RESTORE();
        }
        else
        {
            status = CO_ERROR_MEMORY_CAPA_EXCEED;
            ASSERT_ERR_FORCE(0);
        }
    }

    return (status);
}

uint8_t ld_pscan_stop(void)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    FUNC_PATCH_ENTRY_NO_PARAM_HAVE_RETURN(LD_PSCAN_PATCH_TYPE, LD_PSCAN_STOP_BIT, uint8_t);

    GLOBAL_INT_DISABLE();

    if (ld_pscan_env != NULL)
    {
        // Point to parameters
        struct ld_pscan_env_tag *pscan_par = ld_pscan_env;

        switch (pscan_par->state)
        {
        case PSCAN_EVT_WAIT:
        {
            // Remove event
            sch_arb_remove(&(ld_pscan_env->evt), false);

#if (EAVESDROPPING_SUPPORT)
            if (ld_pscan_seg_scan_env.seg_scan_state == SEG_SCAN_ON)
            {
                // Clear segmented scan
                ld_pscan_seg_scan_env.seg_scan_state = SEG_SCAN_ENABLING;
            }
            else if (ld_pscan_seg_scan_env.seg_scan_state == SEG_SCAN_DISABLING)
            {
                // Clear segmented scan
                ld_pscan_seg_scan_env.seg_scan_state = SEG_SCAN_OFF;
            }
#endif // EAVESDROPPING_SUPPORT

            // End page scan
            ld_pscan_end(false);
        }
        break;

        case PSCAN_EVT_ACTIVE:
        {
            uint8_t cs_idx = EM_BT_CS_ACL_INDEX(pscan_par->link_id);

            // Clear frame duration (in case the CS has not been prefetched yet)
            em_bt_wincntl_pack(cs_idx, 0, NORMAL_WIN_SIZE);
            em_bt_maxfrmtime_setf(cs_idx, 1);
            em_bt_loopcntl_att_nb_setf(cs_idx, 1);

            // Abort page scan
            bt_rwbtcntl_scan_abort_setf(1);

            // Move state
            pscan_par->state = PSCAN_EVT_END;
        }
        break;

        default:
        {
            // Nothing to do
            ASSERT_INFO_FORCE(0, pscan_par->state, 0);
        }
        break;
        }

        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return (status);
}

#if RW_BT_MWS_COEX // Generalized Interlaced Scan Support
void ld_pscan_mwscoex_xi_mask_build(void)
{
    struct bd_addr *bd_addr = &ld_env.local_bd_addr;

    uint32_t uap_lap = ((uint32_t)bd_addr->addr[3] << 24) + ((uint32_t)bd_addr->addr[2] << 16)
                       + ((uint32_t)bd_addr->addr[1] << 8) + bd_addr->addr[0];

    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LD_PSCAN_PATCH_TYPE, LD_PSCAN_MWSCOEX_XI_MASK_BUILD_BIT);

    // Build Xi mask for page scan in  ld_pscan_mwscoex_xi_mask based on uap_lap of local bd_addr
    ld_mwscoex_xi_scan_mask_build(&ld_pscan_mwscoex_xi_mask, uap_lap);
}
#endif // RW_BT_MWS_COEX

#if (EAVESDROPPING_SUPPORT)
uint8_t ld_pscan_seg_scan_en(struct ld_seg_scan_params *params)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    GLOBAL_INT_DISABLE();

    if ((ld_pscan_seg_scan_env.seg_scan_state == SEG_SCAN_OFF) || (ld_pscan_seg_scan_env.seg_scan_state == SEG_SCAN_DISABLING))
    {
        // Enable segmented scan
        ld_pscan_seg_scan_env.seg_scan_state = SEG_SCAN_ENABLING;

        // Store parameters
        memcpy(&ld_pscan_seg_scan_env.seg_scan, params, sizeof(struct ld_seg_scan_params));

        // Check if inquiry scan is active
        if (ld_pscan_env != NULL)
        {
            // Point to parameters
            struct sch_arb_elt_tag *evt = &ld_pscan_env->evt;
            struct ld_pscan_env_tag *pscan_par = ld_pscan_env;

            // Check if event is waiting
            if (pscan_par->state == PSCAN_EVT_WAIT)
            {
                // Remove from schedule
                sch_arb_remove(evt, false);

                // Reschedule as segmented scan
                ld_pscan_resched();
            }
        }

        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return status;
}

uint8_t ld_pscan_seg_scan_dis(void)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    GLOBAL_INT_DISABLE();

    if ((ld_pscan_env != NULL) && (ld_pscan_seg_scan_env.seg_scan_state == SEG_SCAN_ON))
    {
        // Disable segmented scan
        ld_pscan_seg_scan_env.seg_scan_state = SEG_SCAN_DISABLING;

        status = CO_ERROR_NO_ERROR;
    }
    else if (ld_pscan_seg_scan_env.seg_scan_state == SEG_SCAN_ENABLING)
    {
        // Indicate segmented scan as inactive
        ld_pscan_seg_scan_env.seg_scan_state = SEG_SCAN_OFF;

        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return status;
}
#endif // EAVESDROPPING_SUPPORT

///@} LDPSCAN
