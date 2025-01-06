/**
****************************************************************************************
*
* @file ld_page.c
*
* @brief LD Page source code
*
* Copyright (C) RivieraWaves 2009-2015
*
****************************************************************************************
*/

/**
 ****************************************************************************************
 * @addtogroup LDPAGE
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
#include "rwip.h"

#include "ld.h"              // link driver API
#include "ld_util.h"        // link driver utilities
#include "ld_int.h"         // link driver internal

#include "lm.h"

#include "dbg.h"

#include "sch_arb.h"            // Scheduling Arbiter
#include "sch_prog.h"           // Scheduling Programmer
#include "sch_slice.h"          // Scheduling Slicer

#include "reg_btcore.h"         // BT core registers
#include "reg_em_bt_cs.h"       // BT EM Control Structure
#include "reg_em_bt_txdesc.h"   // BT EM TX descriptors
#include "reg_em_bt_rxdesc.h"   // BT EM RX descriptors
#include "btdm_patch.h"

/*
 * DEFINES
 *****************************************************************************************
 */

/// Page event automatic rescheduling attempts
#define LD_PAGE_EVT_AUTO_RESCHED_ATT  4

/// Page first packet event duration min (in us)
#define LD_PAGE_1ST_PKT_EVT_DUR_MIN         (6*SLOT_SIZE - LD_ACL_INTERFRAME_MARGIN)
/// Page scan event automatic rescheduling attempts
#define LD_PAGE_1ST_PKT_EVT_AUTO_RESCHED_ATT  4

/*
 * ENUMERATIONS DEFINITIONS
 ****************************************************************************************
 */

/// Page event states
enum PAGE_EVT_STATE
{
    PAGE_EVT_WAIT,
    PAGE_EVT_ACTIVE,
    PAGE_EVT_END,
};

/// Page response steps
enum PAGE_STEP
{
    PAGE_STEP_PAGE,
    PAGE_STEP_1ST_PKT,
};


/*
 * STRUCTURE DEFINITION
 *****************************************************************************************
 */

/// LD PAGE environment structure
struct ld_page_env_tag
{
    /// Page event
    struct sch_arb_elt_tag evt;

    /// BD Address of the paged device
    struct bd_addr bd_addr;
    /// Page Timeout (in slots)
    uint32_t page_to;
    /// Page start TS (in 312.5us BT half-slots)
    uint32_t page_start_ts;
    /// Page end TS (in 312.5us BT half-slots)
    uint32_t page_end_ts;
    /// New Connection end TS (in 312.5us BT half-slots)
    uint32_t new_con_end_ts;
    /// Clock Offset (in 312.5us BT half-slots)
    uint32_t clk_off;
    /// Number of A/B train repetitions (16 slots per train)
    uint16_t n_page;
    /// LT address
    uint8_t lt_addr;
    /// Link identifier
    uint8_t link_id;
    /// State
    uint8_t state;
    /// Page Response step
    uint8_t step;
    /// Page step BCH
    uint8_t bch_page[LD_BCH_SIZE];
    /// 1st packet step BCH
    uint8_t bch_1st_pkt[LD_BCH_SIZE];
    /// Truncated page
    bool truncated;
    /// Knudge enable
    bool knudge_en;
};

/*
 * VARIABLE DEFINITION
 *****************************************************************************************
 */

/// LD PAGE environment variable
__STATIC struct ld_page_env_tag *ld_page_env;

#if (EAVESDROPPING_SUPPORT)
    /// Custom paging variables
    __STATIC bool cust_enabled = 0;
    __STATIC uint16_t cust_min_page_timeout;
    __STATIC uint8_t cust_train_nudging_offset;
#endif // EAVESDROPPING_SUPPORT


/*
 * LOCAL FUNCTIONS DECLARATION
 *****************************************************************************************
 */

__STATIC void ld_page_frm_cbk(uint32_t timestamp, uint32_t dummy, uint8_t irq_type);


/*
 * LOCAL FUNCTIONS DEFINITION
 *****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialize exchange memory
 ****************************************************************************************
 */
__STATIC void ld_page_em_init(void)
{
    uint8_t cs_idx, txdesc_idx;

    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LD_PAGE_PATCH_TYPE, LD_PAGE_EM_INIT_BIT);

    // Point to parameters
    struct ld_page_env_tag *page_par = ld_page_env;

    // Set control structure fields
    cs_idx = EM_BT_CS_ACL_INDEX(page_par->link_id);

    // Set permission/status of CS as R/W but uninitialized
    DBG_MEM_PERM_SET((const void *)(REG_EM_BT_CS_BASE_ADDR + REG_EM_BT_CS_ADDR_GET(cs_idx)), REG_EM_BT_CS_SIZE, true, true, true);

    em_bt_frcntl_pack(cs_idx, 1 /*fpoll*/, 0 /*fmwspatt*/, RWIP_COEX_GET(PAGE, TXBSY), RWIP_COEX_GET(PAGE, RXBSY), RWIP_COEX_GET(PAGE, DNABORT), 0 /*samen*/, 1 /*lsam_dsb*/, EM_BT_CS_FMT_PAGE);
    em_bt_bdaddr_setf(cs_idx, 0, (page_par->bd_addr.addr[1] << 8) | page_par->bd_addr.addr[0]);
    em_bt_bdaddr_setf(cs_idx, 1, (page_par->bd_addr.addr[3] << 8) | page_par->bd_addr.addr[2]);
    em_bt_bdaddr_setf(cs_idx, 2, (page_par->bd_addr.addr[5] << 8) | page_par->bd_addr.addr[4]);
    em_bt_bch0_setf(cs_idx, (page_par->bch_page[1] << 8) | page_par->bch_page[0]);
    em_bt_bch1_setf(cs_idx, (page_par->bch_page[3] << 8) | page_par->bch_page[2]);
    em_bt_rxmaxbuf_bch2_pack(cs_idx, 0, 0, page_par->bch_page[4] & EM_BT_BCH2_MASK);
    em_bt_pwrcntl_pack(cs_idx, /*fhen*/ 1, /*freq*/ 0, /*txpwr*/ rwip_rf.txpwr_max);
    em_bt_txrxcntl_pack(cs_idx, /*rxthr*/ 0,
#if (EAVESDROPPING_SUPPORT)
                        /*stopmod*/ 0,
#endif // EAVESDROPPING_SUPPORT
                        /*rxeir*/ 0, /*rxbcry*/ 0, /*rxcrypt*/ 0, /*stopmod/escofnak*/ 0, /*nullflt*/ 0, /*extpaen*/ 0, /*txeir*/ 0, /*txbcry*/ 0, /*txcrypt*/ 0);
    em_bt_acltxstat_pack(cs_idx, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    em_bt_wincntl_pack(cs_idx, 0, NORMAL_WIN_SIZE);
    em_bt_clkoff0_setf(cs_idx, page_par->clk_off);
    em_bt_clkoff1_setf(cs_idx, (page_par->clk_off >> 16));
#if (EAVESDROPPING_SUPPORT)
    if (cust_enabled)
    {
        // Custom paging enabled: enforce the given train nudging parameters
        bt_coexifcntl0_pageknudgeinc_setf(cust_train_nudging_offset >> 1);
        em_bt_linkcntl_pack(cs_idx,
                            /*aknena (Train Nudging Support)*/ (cust_train_nudging_offset != 0),
                            /*afhena*/ 0,
                            /*laap*/ 0,
                            /*whdsb*/ 0,
                            /*acledr*/ 0,
                            /*aclltaddr*/ 0,
#if (EAVESDROPPING_SUPPORT)
                            /*aclbinackena*/ 0, /*escobinackena*/ 0, /*duplfltena*/ 0,
#endif // EAVESDROPPING_SUPPORT
                            /*linklbl*/ cs_idx);
    }
    else
#endif // EAVESDROPPING_SUPPORT
    {
        // Custom paging disabled: use the default settings
        em_bt_linkcntl_pack(cs_idx,
                            /*aknena*/ page_par->knudge_en,
                            /*afhena*/ 0,
                            /*laap*/ 0,
                            /*whdsb*/ 0,
                            /*acledr*/ 0,
                            /*aclltaddr*/ 0,
#if (EAVESDROPPING_SUPPORT)
                            /*aclbinackena*/ 0, /*escobinackena*/ 0, /*duplfltena*/ 0,
#endif // EAVESDROPPING_SUPPORT
                            /*linklbl*/ cs_idx);
    }

    if (!page_par->truncated)
    {
        // Set FHS Tx descriptor fields
        txdesc_idx = EM_BT_TXDESC_ACL_INDEX(page_par->link_id, 0);
        em_bt_txheader_pack(txdesc_idx, 0, 0, 1, 1, 1, FHS_TYPE, 0);
        em_bt_txpheader_pack(txdesc_idx, 0, FHS_PACKET_SIZE, 0, LLID_START);
        em_bt_txaclbufptr_setf(txdesc_idx, 0);
        em_bt_txlmbufptr_setf(txdesc_idx, EM_BT_PAGEFHSTXBUF_OFFSET);
        em_bt_txptr_pack(txdesc_idx, 0, 0);

        // Point CS to TX descriptor
        em_bt_txdescptr_setf(cs_idx, REG_EM_ADDR_GET(BT_TXDESC, txdesc_idx));
    }
    else
    {
        em_bt_txdescptr_setf(cs_idx, 0);
    }

    // Default clear other control
    em_bt_chmap0_set(cs_idx, 0);
    em_bt_chmap1_set(cs_idx, 0);
    em_bt_chmap2_set(cs_idx, 0);
    em_bt_chmap3_set(cs_idx, 0);
    em_bt_chmap4_set(cs_idx, 0);
    em_bt_loopcntl_set(cs_idx, 0);
    em_bt_maxfrmtime_set(cs_idx, 0);
    em_bt_aclrxstat_set(cs_idx, 0);

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
__STATIC bool ld_page_1st_pkt_rx(void)
{
    bool packet_received = false;

    FUNC_PATCH_ENTRY_NO_PARAM_HAVE_RETURN(LD_PAGE_PATCH_TYPE, LD_PAGE_1ST_PKT_RX_BIT, bool);

    // Check if 1st master packet has been received
    if (ld_rxdesc_check(EM_BT_CS_ACL_INDEX(ld_page_env->link_id)))
    {
        // Get current RX descriptor index
        uint8_t index_1st_pkt = ld_env.curr_rxdesc_index;
        // Retrieve RX status and type
        uint16_t rxstat = em_bt_rxstat_get(index_1st_pkt);

        // Check if Poll reception is correct
        if ((rxstat & (EM_BT_RXSYNCERR_BIT | EM_BT_RXHECERR_BIT)) == 0)
        {
            // Indicate successful reception
            packet_received = true;
        }

        // Free RX descriptor
        ld_rxdesc_free();
    }

    return (packet_received);
}

/**
 ****************************************************************************************
 * @brief Report the end of the activity
 ****************************************************************************************
 */
__STATIC void ld_page_end(uint8_t status)
{
    // Point to parameters
    struct ld_page_env_tag *page_par = ld_page_env;

    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_PAGE_PATCH_TYPE, LD_PAGE_END_BIT, status);
    // Report page end to LM
    struct lm_page_end_ind *ind = KE_MSG_ALLOC(LM_PAGE_END_IND, TASK_LM, TASK_NONE, lm_page_end_ind);
    ind->link_id = page_par->link_id;
    ind->status = status;
    ke_msg_send(ind);

    // Remove permission/status of CS as now unused
    DBG_MEM_PERM_SET((const void *)(REG_EM_BT_CS_BASE_ADDR + REG_EM_BT_CS_ADDR_GET(EM_BT_CS_ACL_INDEX(page_par->link_id))), REG_EM_BT_CS_SIZE, false, false, false);

    // Free event memory
    ke_free(ld_page_env);
    ld_page_env = NULL;

    // Disable Page train counter
    bt_abtraincntl_abtpageen_setf(0);

    // Unregister the page process from scheduling parameters
    sch_slice_bg_remove(BT_PAGE);

    DBG_SWDIAG(PAGE, PAGE_STEP, 0);
}

/**
 ****************************************************************************************
 * @brief Program the activity
 ****************************************************************************************
 */
__STATIC void ld_page_prog(uint32_t clock)
{
    // Point to parameters
    struct ld_page_env_tag *page_par = ld_page_env;
    struct sch_arb_elt_tag *evt = &page_par->evt;
    uint8_t cs_idx = EM_BT_CS_ACL_INDEX(page_par->link_id);
    uint8_t frm_trype = (page_par->truncated) ? SCH_BT_FRAME_TYPE_CSB : SCH_BT_FRAME_TYPE_NORMAL;

    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_PAGE_PATCH_TYPE, LD_PAGE_PROG_BIT, clock);

    switch (page_par->step)
    {
    case PAGE_STEP_PAGE:
    {
        uint32_t max_win_len = CLK_SUB(page_par->page_end_ts, evt->time.hs) >> 1;
        uint32_t curr_page_dur_16slots = CLK_SUB(evt->time.hs, page_par->page_start_ts) >> 5;
        uint16_t curr_abtrain_period = curr_page_dur_16slots / page_par->n_page;

        // Determine the number of train repetitions, remaining in the current period
        uint16_t n_page_rem = page_par->n_page - (curr_page_dur_16slots - (curr_abtrain_period * page_par->n_page));

        // Check to not exceed the maximum value supported by HW
        max_win_len = co_min(max_win_len, 2 * EM_BT_CS_MAXFRMTIME_MAX);

        // Max to end of train if configuring sub n_page
        if (n_page_rem < page_par->n_page)
        {
            max_win_len = co_min(max_win_len, (n_page_rem << 5));
        }

        // Prepare CS
        em_bt_frcntl_format_setf(cs_idx, EM_BT_CS_FMT_PAGE);
        em_bt_maxfrmtime_setf(cs_idx, max_win_len / 2);
        em_bt_loopcntl_att_nb_setf(cs_idx, PAGE_RESP_TO_DEF / 2);

        // Configure A/B train selection

        bt_abtraincntl_abtpagetime_setf(n_page_rem);

        // Train used in the current period
        bt_abtraincntl_abtpagestartvalue_setf(curr_abtrain_period & 0x1);

        // Load the A/B train counter
        bt_abtraincntl_abtpageload_setf(1);
    }
    break;
    case PAGE_STEP_1ST_PKT:
    {
        uint8_t att_nb = CLK_SUB(page_par->new_con_end_ts, evt->time.hs) >> 1;

        // Prepare CS
        em_bt_loopcntl_att_nb_setf(cs_idx, att_nb / 2);
    }
    break;
    default:
    {
        // Nothing to do
        ASSERT_INFO_FORCE(0, page_par->step, 0);
    }
    break;
    }

    {
        // Push the programming to SCH PROG
        struct sch_prog_params prog_par;
        prog_par.frm_cbk         = &ld_page_frm_cbk;
        prog_par.time.hs         = evt->time.hs;
        prog_par.time.hus        = 0;
        prog_par.cs_idx          = cs_idx;
        prog_par.dummy           = cs_idx;
        prog_par.bandwidth       = evt->duration_min;
        prog_par.prio_1          = evt->current_prio;
        prog_par.prio_2          = 0;
        prog_par.prio_3          = rwip_priority[RWIP_PRIO_PAGE_1ST_PKT_IDX].value;
        prog_par.pti_prio        = BT_PTI_PAGE_IDX;
        prog_par.add.bt.frm_type = frm_trype;
        prog_par.mode            = SCH_PROG_BT;
        sch_prog_push(&prog_par);
    }
}

/**
 ****************************************************************************************
 * @brief Handle event start notification
 ****************************************************************************************
 */
__STATIC void ld_page_evt_start_cbk(struct sch_arb_elt_tag *evt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_PAGE_PATCH_TYPE, LD_PAGE_EVT_START_CBK_BIT, evt);

    DBG_SWDIAG(PAGE, PAGE_EVT_START, 1);

    ASSERT_ERR((&(ld_page_env->evt)) == evt);

    if (evt != NULL)
    {
        // Point to parameters
        struct ld_page_env_tag *page_par = ld_page_env;

        // Check if a page window is ongoing
        if (page_par->page_end_ts > RWIP_MAX_CLOCK_TIME)
        {
            // Store window's start/end TS
            page_par->page_start_ts = evt->time.hs;
            page_par->page_end_ts = CLK_ADD_2(evt->time.hs, 2 * page_par->page_to);

            // Register the page process into scheduling parameters
            sch_slice_bg_add(BT_PAGE);
        }

        // SW Workaround: CS not updated due to abort, CS-RXID is cleared by SW
        em_bt_aclrxstat_rxid_setf(EM_BT_CS_ACL_INDEX(page_par->link_id), 0);

        // Program page
        ld_page_prog(evt->time.hs);

        // Move state
        page_par->state = PAGE_EVT_ACTIVE;
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(PAGE, PAGE_EVT_START, 0);
}

/**
 ****************************************************************************************
 * @brief Handle event canceled notification
 ****************************************************************************************
 */
__STATIC void ld_page_evt_canceled_cbk(struct sch_arb_elt_tag *evt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_PAGE_PATCH_TYPE, LD_PAGE_EVT_CANCELED_CBK_BIT, evt);

    DBG_SWDIAG(PAGE, PAGE_EVT_CANCELED, 1);

    ASSERT_ERR((&(ld_page_env->evt)) == evt);

    if (evt != NULL)
    {
        // Point to parameters
        struct ld_page_env_tag *page_par = ld_page_env;

        // Check page end
        if (page_par->state == PAGE_EVT_END)
        {
            // Report page end
            ld_page_end(CO_ERROR_CON_TERM_BY_LOCAL_HOST);
        }
        else
        {
            uint32_t clock = ld_read_clock();

            // Update timestamp to current clock
            evt->time.hs = clock;

            // Check Page Scan step
            switch (page_par->step)
            {
            case PAGE_STEP_PAGE:
            {
                // Increment priority
                evt->current_prio = RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(RWIP_PRIO_PAGE_DFT_IDX));
                evt->duration_min = sch_slice_params.scan_evt_dur;

                // Try to reschedule as it was
                if (sch_arb_insert(evt) != SCH_ARB_ERROR_OK)
                {
                    // Report page end
                    ld_page_end(CO_ERROR_PAGE_TIMEOUT);
                }
                else
                {
                    // Move to wait state
                    page_par->state = PAGE_EVT_WAIT;
                }
            }
            break;
            case PAGE_STEP_1ST_PKT:
            {
                // Increment priority
                evt->current_prio = RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(RWIP_PRIO_PAGE_1ST_PKT_IDX));

                // Try to schedule as it was
                if (sch_arb_insert(evt) != SCH_ARB_ERROR_OK)
                {
                    // Failed to schedule before the new connection end timestamp, return to page step
                    page_par->step = PAGE_STEP_PAGE;
                    DBG_SWDIAG(PAGE, PAGE_STEP, 0);

                    // Reset priority
                    evt->current_prio = rwip_priority[RWIP_PRIO_PAGE_DFT_IDX].value;
                    evt->duration_min = sch_slice_params.scan_evt_dur;
                    SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_ASAP_LIMIT, SCH_ARB_PHASE_0, LD_PAGE_EVT_AUTO_RESCHED_ATT, RWIP_PRIO_INC(RWIP_PRIO_PAGE_DFT_IDX));
                    evt->asap_limit = page_par->page_end_ts;

                    // Initialize EM
                    ld_page_em_init();

                    // Try to schedule page before the end timestamp
                    if (sch_arb_insert(evt) != SCH_ARB_ERROR_OK)
                    {
                        // Report page end
                        ld_page_end(CO_ERROR_PAGE_TIMEOUT);
                    }
                    else
                    {
                        // Move to wait state
                        page_par->state = PAGE_EVT_WAIT;
                    }
                }
                else
                {
                    // Move to wait state
                    page_par->state = PAGE_EVT_WAIT;
                }
            }
            break;
            default:
            {
                // Nothing to do
                ASSERT_INFO_FORCE(0, page_par->step, 0);
            }
            break;
            }
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(PAGE, PAGE_EVT_CANCELED, 0);
}

/**
 ****************************************************************************************
 * @brief Handle frame interrupt
 ****************************************************************************************
 */
__STATIC void ld_page_frm_isr(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LD_PAGE_PATCH_TYPE, LD_PAGE_FRM_ISR_BIT);

    DBG_SWDIAG(PAGE, PAGE_FRM_ISR, 1);

    if (ld_page_env != NULL)
    {
        // Point to parameters
        struct sch_arb_elt_tag *evt = &(ld_page_env->evt);
        struct ld_page_env_tag *page_par = ld_page_env;

        // Remove event
        sch_arb_remove(evt, true);

        // Check page end
        if (page_par->state == PAGE_EVT_END)
        {
            // Flush a potentially consumed descriptor
            if (ld_rxdesc_check(EM_BT_CS_ACL_INDEX(page_par->link_id)))
            {
                // Free RX descriptor
                ld_rxdesc_free();
            }

            // Report page end
            ld_page_end(CO_ERROR_CON_TERM_BY_LOCAL_HOST);
        }
        else
        {
            uint32_t clock = ld_read_clock();
            uint8_t cs_idx = EM_BT_CS_ACL_INDEX(page_par->link_id);

            // Update timestamp to current clock
            evt->time.hs = clock;

            // Move to wait state
            page_par->state = PAGE_EVT_WAIT;

            // Check Page step
            switch (page_par->step)
            {
            case PAGE_STEP_PAGE:
            {
                // Check if ID has been received in truncated page mode
                if (page_par->truncated && em_bt_aclrxstat_rxid_getf(cs_idx))
                {
                    // Report page end
                    ld_page_end(CO_ERROR_NO_ERROR);

                    break;
                }

                // Check if ID has been received in the master page response state
                if (em_bt_rxbit_frrxok_getf(cs_idx) && em_bt_aclrxstat_rxid_getf(cs_idx) && (em_bt_frcntl_format_getf(cs_idx) == EM_BT_CS_FMT_MST_PAGE_RSP))
                {
                    // Set the new connection timeout (32 slots)
                    page_par->new_con_end_ts = CLK_ADD_2(clock, 2 * (NEW_CONNECTION_TO + 1));

                    // Change event settings
                    evt->current_prio = rwip_priority[RWIP_PRIO_PAGE_1ST_PKT_IDX].value;
                    evt->duration_min = 2 * LD_PAGE_1ST_PKT_EVT_DUR_MIN;
                    SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_ASAP_LIMIT, SCH_ARB_PHASE_0, LD_PAGE_1ST_PKT_EVT_AUTO_RESCHED_ATT, RWIP_PRIO_INC(RWIP_PRIO_PAGE_1ST_PKT_IDX));
                    evt->asap_limit = page_par->new_con_end_ts;

                    // Try to schedule before the new connection end timestamp
                    if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
                    {
                        // Set control structure fields
                        em_bt_frcntl_format_setf(cs_idx, EM_BT_CS_FMT_MST_CONNECT);
                        em_bt_bdaddr_setf(cs_idx, 0, (ld_env.local_bd_addr.addr[1] << 8) | ld_env.local_bd_addr.addr[0]);
                        em_bt_bdaddr_setf(cs_idx, 1, (ld_env.local_bd_addr.addr[3] << 8) | ld_env.local_bd_addr.addr[2]);
                        em_bt_bdaddr_setf(cs_idx, 2, (ld_env.local_bd_addr.addr[5] << 8) | ld_env.local_bd_addr.addr[4]);
                        em_bt_bch0_setf(cs_idx, (ld_env.local_bch[1] << 8) | ld_env.local_bch[0]);
                        em_bt_bch1_setf(cs_idx, (ld_env.local_bch[3] << 8) | ld_env.local_bch[2]);
                        em_bt_rxmaxbuf_bch2_pack(cs_idx, 0, 0, ld_env.local_bch[4]  & EM_BT_BCH2_MASK);
                        em_bt_linkcntl_afhena_setf(cs_idx, 0);
                        em_bt_linkcntl_aclltaddr_setf(cs_idx, page_par->lt_addr);
                        em_bt_pwrcntl_fh_en_setf(cs_idx, 1);
                        em_bt_txrxcntl_txeir_setf(cs_idx, 0);
                        em_bt_txdescptr_setf(cs_idx, 0);
                        em_bt_acltxstat_fnak_setf(cs_idx, 1);
                        em_bt_acltxstat_fcrc_setf(cs_idx, 1);
                        em_bt_frcntl_fpoll_setf(cs_idx, 1);
                        em_bt_aclrxstat_lastrxseqn_setf(cs_idx, 0);
                        em_bt_clkoff0_setf(cs_idx, 0);
                        em_bt_clkoff1_setf(cs_idx, 0);

                        // Move step to 1st Packet
                        page_par->step = PAGE_STEP_1ST_PKT;
                        DBG_SWDIAG(PAGE, PAGE_STEP, 1);

                        break;
                    }
                }

                // Restore original Page priority
                evt->current_prio = rwip_priority[RWIP_PRIO_PAGE_DFT_IDX].value;
                evt->duration_min = sch_slice_params.scan_evt_dur;
                SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_ASAP_LIMIT, SCH_ARB_PHASE_0, LD_PAGE_EVT_AUTO_RESCHED_ATT, RWIP_PRIO_INC(RWIP_PRIO_PAGE_DFT_IDX));
                evt->asap_limit = page_par->page_end_ts;

                // Try to schedule page before the end timestamp
                if (sch_arb_insert(evt) != SCH_ARB_ERROR_OK)
                {
                    // Report page end
                    ld_page_end(CO_ERROR_PAGE_TIMEOUT);
                }
            }
            break;
            case PAGE_STEP_1ST_PKT:
            {
                // Check if 1st Slave packet has been received
                if (ld_page_1st_pkt_rx())
                {
                    // Report page end
                    ld_page_end(CO_ERROR_NO_ERROR);
                    break;
                }

                // Try to schedule before the new connection end timestamp
                if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
                {
                    break;
                }

                // Failed to schedule before the new connection end timestamp, return to page step
                page_par->step = PAGE_STEP_PAGE;
                DBG_SWDIAG(PAGE, PAGE_STEP, 0);

                // Reset event settings
                evt->current_prio = rwip_priority[RWIP_PRIO_PAGE_DFT_IDX].value;
                evt->duration_min = sch_slice_params.scan_evt_dur;
                SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_ASAP_LIMIT, SCH_ARB_PHASE_0, LD_PAGE_EVT_AUTO_RESCHED_ATT, RWIP_PRIO_INC(RWIP_PRIO_PAGE_DFT_IDX));
                evt->asap_limit = page_par->page_end_ts;

                // Try to schedule page before the end timestamp
                if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
                {
                    // Initialize EM
                    ld_page_em_init();
                }
                else
                {
                    // Report page end
                    ld_page_end(CO_ERROR_PAGE_TIMEOUT);
                }
            }
            break;
            default:
            {
                // Nothing to do
                ASSERT_INFO_FORCE(0, page_par->step, 0);
            }
            break;
            }
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(PAGE, PAGE_FRM_ISR, 0);
}

/**
 ****************************************************************************************
 * @brief Handle frame interrupt notification
 ****************************************************************************************
 */
__STATIC void ld_page_frm_cbk(uint32_t timestamp, uint32_t dummy, uint8_t irq_type)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LD_PAGE_PATCH_TYPE, LD_PAGE_FRM_CBK_BIT, timestamp, dummy, irq_type);

    switch (irq_type)
    {
    case SCH_FRAME_IRQ_EOF:
    case SCH_FRAME_IRQ_EOF_ABORT_UNDER_PRIO:
    case SCH_FRAME_IRQ_EOF_ABORT_AFTER_PRIO:
    {
        ld_page_frm_isr();
    }
    break;
    case SCH_FRAME_IRQ_SKIP:
    {
        // Point to parameters
        struct sch_arb_elt_tag *evt = &(ld_page_env->evt);

        // Remove event
        sch_arb_remove(evt, true);

        // Invoke event cancel callback
        ld_page_evt_canceled_cbk(evt);
    }
    break;
    default:
    {
        ASSERT_INFO_FORCE(0, dummy, irq_type);
    }
    break;
    }
}


/*
 * EXPORTED FUNCTIONS DEFINITION
 *****************************************************************************************
 */

void ld_page_init(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LD_PAGE_PATCH_TYPE, LD_PAGE_INIT_BIT);

    ld_page_env = NULL;
}

void ld_page_reset(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LD_PAGE_PATCH_TYPE, LD_PAGE_RESET_BIT);

    // Check if page is active
    if (ld_page_env != NULL)
    {
        // Free event memory
        ke_free(ld_page_env);
        ld_page_env = NULL;
    }

#if (EAVESDROPPING_SUPPORT)
    cust_enabled = false;
#endif // EAVESDROPPING_SUPPORT
}

uint8_t ld_page_start(struct ld_page_params *params)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LD_PAGE_PATCH_TYPE, LD_PAGE_START_BIT, uint8_t, params);
    // Check if page is inactive
    if (ld_page_env == NULL)
    {
        // Allocate event
        ld_page_env = LD_ALLOC_EVT(ld_page_env_tag);

        if (ld_page_env != NULL)
        {
            uint32_t clock         = ld_read_clock();

            // Point to parameters
            struct sch_arb_elt_tag *evt = &(ld_page_env->evt);
            struct ld_page_env_tag *page_par = ld_page_env;

            LD_INIT_EVT(evt, ld_page_env_tag);

            // Initialize event parameters (common part)
            evt->cb_cancel        = &ld_page_evt_canceled_cbk;
            evt->cb_start         = &ld_page_evt_start_cbk;
            evt->cb_stop          = NULL;
            evt->current_prio        = rwip_priority[RWIP_PRIO_PAGE_DFT_IDX].value;
            evt->duration_min        = sch_slice_params.scan_evt_dur;
            evt->time.hs           = clock;
            SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_ASAP_NO_LIMIT, SCH_ARB_PHASE_0, LD_PAGE_EVT_AUTO_RESCHED_ATT, RWIP_PRIO_INC(RWIP_PRIO_PAGE_DFT_IDX));

            // Initialize event parameters (page part)
            memcpy(&page_par->bd_addr.addr[0], &params->bd_addr.addr[0], BD_ADDR_LEN);
            page_par->link_id          = params->link_id;
            page_par->page_to          = params->page_to;
#if (EAVESDROPPING_SUPPORT)
            if (cust_enabled && cust_min_page_timeout > params->page_to)
            {
                // Custom paging enabled: enforce the given minimum page timeout
                page_par->page_to      = cust_min_page_timeout;
            }
#endif // EAVESDROPPING_SUPPORT
            page_par->clk_off          = 2 * params->clk_off;
            page_par->lt_addr           = params->lt_addr;
            page_par->n_page            = params->n_page;
            page_par->truncated         = params->truncated;
            page_par->knudge_en         = params->knudge_en;
            page_par->step              = PAGE_STEP_PAGE;
            DBG_SWDIAG(PAGE, PAGE_STEP, 0);
            page_par->page_end_ts       = LD_CLOCK_UNDEF;

            // Compute the BCH for page step
            ld_util_bch_create(&page_par->bd_addr.addr[0], &page_par->bch_page[0]);

            if (!page_par->truncated)
            {
                // Pack FHS
                ld_util_fhs_pk(EM_BT_PAGEFHSTXBUF_OFFSET, &ld_env.local_bch[0], 0, params->page_scan_rep_mode, &ld_env.local_bd_addr, &ld_env.class_of_dev, page_par->lt_addr, MANDATORY_PAGE_SCAN_MODE);
            }

            // Initialize EM
            ld_page_em_init();

#if (EAVESDROPPING_SUPPORT)
            if (!cust_enabled && page_par->knudge_en)
#else //!(EAVESDROPPING_SUPPORT)
            if (page_par->knudge_en)
#endif //!(EAVESDROPPING_SUPPORT)
            {
                // Configure a default knudge increment to occur after 1st 2 x Npage repetitions
                bt_coexifcntl0_pageknudgeinc_setf(BT_KNUDGE_INC);
            }

            // Disable Page train counter
            bt_abtraincntl_abtpageen_setf(0);

            // Configure the number of train repetition
            bt_abtraincntl_abtpagetime_setf(page_par->n_page);

            // Always start using the A-train
            bt_abtraincntl_abtpagestartvalue_setf(0);

            // Set Page train enable
            bt_abtraincntl_abtpageen_setf(1);

            // Load the Value
            bt_abtraincntl_abtpageload_setf(1);

            GLOBAL_INT_DISABLE();

            // Try to schedule page with no time limit
            if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
            {
                page_par->state = PAGE_EVT_WAIT;
                status = CO_ERROR_NO_ERROR;
            }
            else
            {
                ASSERT_ERR_FORCE(0);
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

uint8_t ld_page_stop(void)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    FUNC_PATCH_ENTRY_NO_PARAM_HAVE_RETURN(LD_PAGE_PATCH_TYPE, LD_PAGE_STOP_BIT, uint8_t);

    GLOBAL_INT_DISABLE();

    if (ld_page_env != NULL)
    {
        // Point to parameters
        struct sch_arb_elt_tag *evt = &(ld_page_env->evt);
        struct ld_page_env_tag *page_par = ld_page_env;

        switch (page_par->state)
        {
        case PAGE_EVT_WAIT:
        {
            // Remove event
            sch_arb_remove(evt, false);

            // Report page end
            ld_page_end(CO_ERROR_CON_TERM_BY_LOCAL_HOST);
        }
        break;

        case PAGE_EVT_ACTIVE:
        {
            uint8_t cs_idx = EM_BT_CS_ACL_INDEX(page_par->link_id);

            // Clear frame duration (in case the CS has not been prefetched yet)
            em_bt_maxfrmtime_setf(cs_idx, 1);
            em_bt_loopcntl_att_nb_setf(cs_idx, 1);

            // Abort page
            bt_rwbtcntl_pageinq_abort_setf(1);

            // Move state
            page_par->state = PAGE_EVT_END;
        }
        break;

        default:
        {
            // Nothing to do
            ASSERT_INFO_FORCE(0, page_par->state, 0);
        }
        break;
        }

        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return (status);
}

#if (EAVESDROPPING_SUPPORT)
uint8_t ld_page_set_cust_params(bool enable, uint16_t min_page_timeout, uint8_t train_nudging_offset)
{
    uint8_t status = CO_ERROR_NO_ERROR;

    GLOBAL_INT_DISABLE();

    cust_enabled = enable;
    cust_min_page_timeout = min_page_timeout;
    cust_train_nudging_offset = train_nudging_offset;

    GLOBAL_INT_RESTORE();

    return (status);
}
#endif // EAVESDROPPING_SUPPORT
///@} LDPAGE
