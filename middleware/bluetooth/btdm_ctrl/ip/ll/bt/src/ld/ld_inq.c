/**
****************************************************************************************
*
* @file ld_inq.c
*
* @brief LD Inquiry source code
*
* Copyright (C) RivieraWaves 2009-2015
*
****************************************************************************************
*/

/**
 ****************************************************************************************
 * @addtogroup LDINQ
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

#include "co_math.h"
#include "co_utils.h"
#include "co_bt.h"

#include "arch.h"
#include "ke_mem.h"
#include "rwip.h"

#include "ld.h"             // link driver API
#include "ld_util.h"        // link driver utilities
#include "ld_int.h"         // link driver internal

#include "lm.h"

#include "dbg.h"

#include "sch_arb.h"            // Scheduling Arbiter
#include "sch_prog.h"           // Scheduling Programmer
#include "sch_slice.h"          // Scheduling Slicer

#include "reg_btcore.h"         // BT core registers
#include "reg_em_bt_cs.h"       // BT EM Control Structure
#include "reg_em_bt_rxdesc.h"   // BT EM RX descriptors
#include "btdm_patch.h"

/*
 * DEFINES
 *****************************************************************************************
 */

/// Inquiry event automatic rescheduling attempts
#define LD_INQ_EVT_AUTO_RESCHED_ATT  4

/*
 * ENUMERATIONS DEFINITIONS
 ****************************************************************************************
 */

/// Inquiry event states
enum INQ_EVT_STATE
{
    INQ_EVT_WAIT,
    INQ_EVT_ACTIVE,
    INQ_EVT_END,
};


/*
 * STRUCTURE DEFINITION
 *****************************************************************************************
 */

/// LD INQ environment structure
struct ld_inq_env_tag
{
    /// Inquiry event
    struct sch_arb_elt_tag evt;

    /// Inquiry window's start TS (in 312.5us BT half-slots)
    uint32_t win_start_ts;
    /// Inquiry window's end TS (in 312.5us BT half-slots)
    uint32_t win_end_ts;
    /// LAP to be used for the access code construction (GIAC or DIAC)
    struct lap lap;
    /// Minimum duration between consecutive inquiries in number of 1.28 seconds (0 for non-periodic inquiry)
    uint16_t per_min;
    /// Maximum duration between consecutive inquiries in number of 1.28 seconds (0 for non-periodic inquiry)
    uint16_t per_max;
    /// Length of the inquiry in number of 1.28 seconds (2048 slots)
    uint8_t inq_len;
    /// Enable/Disable Extended Inquiry Response reception
    bool eir_en;
    /// State
    uint8_t state;
    /// Max number of response before halting the inquiry
    uint8_t nb_rsp_max;
    /// Current number of response received
    uint8_t nb_rsp_curr;
    /// Inquiry TX power level (in dBm)
    int8_t tx_pwr_lvl;
    /// Ninquiry
    uint16_t n_inq;
};

/*
 * VARIABLE DEFINITION
 *****************************************************************************************
 */

/// LD INQ environment variable
__STATIC struct ld_inq_env_tag *ld_inq_env;


/*
 * LOCAL FUNCTIONS DECLARATION
 *****************************************************************************************
 */

__STATIC void ld_inq_frm_cbk(uint32_t timestamp, uint32_t dummy, uint8_t irq_type);


/*
 * LOCAL FUNCTIONS DEFINITION
 *****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Report the end of the activity
 ****************************************************************************************
 */
__STATIC void ld_inq_end(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LD_INQ_PATCH_TYPE, LD_INQ_END_BIT);

    // Report inquiry end to LM
    ke_msg_send_basic(LM_INQ_END_IND, TASK_LM, TASK_NONE);

    // Remove permission/status of CS as now unused
    DBG_MEM_PERM_SET((const void *)(REG_EM_BT_CS_BASE_ADDR + REG_EM_BT_CS_ADDR_GET(EM_BT_CS_INQ_INDEX)), REG_EM_BT_CS_SIZE, false, false, false);

    // Free event memory
    ke_free(ld_inq_env);
    ld_inq_env = NULL;

    // Disable Inquiry train counter
    bt_abtraincntl_abtinqen_setf(0);

    // Unregister the inquiry process from scheduling parameters
    sch_slice_bg_remove(BT_INQ);
}

/**
 ****************************************************************************************
 * @brief Check the reception during activity
 ****************************************************************************************
 */
__STATIC void ld_inq_rx(void)
{
    // Point to parameters
    struct ld_inq_env_tag *inq_par = ld_inq_env;

    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LD_INQ_PATCH_TYPE, LD_INQ_RX_BIT);

    // Check if FHS has been received during inquiry frames
    if (ld_rxdesc_check(EM_BT_CS_INQ_INDEX))
    {
        uint8_t cs_idx = EM_BT_CS_INQ_INDEX;
        // Get current RX descriptor index
        uint8_t index_fhs = ld_env.curr_rxdesc_index;
        // Retrieve RX status and type
        uint16_t rxstat_fhs = em_bt_rxstat_get(index_fhs);

        // Free RX descriptor
        ld_rxdesc_free();

        // Check if FHS reception is correct
        if ((inq_par->state != INQ_EVT_END) && ((rxstat_fhs & (EM_BT_RXSYNCERR_BIT | EM_BT_RXHECERR_BIT | EM_BT_RXCRCERR_BIT)) == 0))
        {
            struct lm_inq_res_ind *ind = NULL;
            uint32_t fhs_clk_frm;
            uint32_t rx_clk_hslot;
            int32_t clk_off;

            // Point to FHS payload buffer
            uint16_t fhs_buf_ptr = em_bt_rxlmbufptr_getf(index_fhs);

            ASSERT_INFO(em_bt_rxheader_rxtype_getf(index_fhs) == FHS_TYPE, em_bt_rxheader_rxtype_getf(index_fhs), 0);

            // Check if EIR has been received during inquiry frames
            if (inq_par->eir_en && (rxstat_fhs & EM_BT_RXEIRSTAT_BIT) && ld_rxdesc_check(EM_BT_CS_INQ_INDEX))
            {
                // Get current RX descriptor index
                uint8_t index_eir = ld_env.curr_rxdesc_index;
                // Retrieve RX status
                uint16_t rxstat_eir = em_bt_rxstat_get(index_eir);

                // Check if EIR reception is correct
                if ((rxstat_eir & (EM_BT_RXSYNCERR_BIT | EM_BT_RXHECERR_BIT | EM_BT_RXCRCERR_BIT)) == 0)
                {
                    // Point to EIR payload buffer
                    uint16_t eir_buf_ptr = em_bt_rxaclbufptr_getf(index_eir);
                    uint8_t length = em_bt_rxpheader_rxlength_getf(index_eir);

                    // Allocate message
                    ind = KE_MSG_ALLOC_DYN(LM_INQ_RES_IND, TASK_LM, TASK_NONE, lm_inq_res_ind, length);

                    // Copy EIR data
                    ind->eir_len = length;
                    em_rd(ind->eir_data, eir_buf_ptr, length);
                }

                // Free RX descriptor
                ld_rxdesc_free();
            }

            // If no EIR to report
            if (ind == NULL)
            {
                ind = KE_MSG_ALLOC_DYN(LM_INQ_RES_IND, TASK_LM, TASK_NONE, lm_inq_res_ind, 0);
                ind->eir_len = 0;
            }

            // Extract fields from packet
            ld_util_fhs_unpk(fhs_buf_ptr, NULL, &ind->bd_addr, &ind->class_of_dev, NULL, &fhs_clk_frm, &ind->page_scan_rep_mode);

            // Read RX clock (in half-slots)
            rx_clk_hslot = (em_bt_rxclkn1_getf(cs_idx) << 16) | em_bt_rxclkn0_getf(cs_idx);

            // Compute clock offset (in half-slots)
            clk_off = (fhs_clk_frm << 2) - rx_clk_hslot;
            if (clk_off < 0)
            {
                clk_off += (RWIP_MAX_CLOCK_TIME + 1);
            }
            ind->clk_off = (clk_off >> 2) & 0x7FFF;

            // Read RSSI
            ind->rssi = em_bt_rxchass_rxrssi_getf(index_fhs);

            // Check whether the EIR bit was set in the FHS packet
            ind->fhs_eir_bit = (rxstat_fhs & EM_BT_RXEIRSTAT_BIT) ? true : false;

            // Send message
            ke_msg_send(ind);

            // Increment response counter
            if (inq_par->nb_rsp_curr < inq_par->nb_rsp_max)
            {
                inq_par->nb_rsp_curr++;
            }
        }

        /*// Increment response counter
        if(inq_par->nb_rsp_curr < inq_par->nb_rsp_max)
        {
            inq_par->nb_rsp_curr++;
        }*/
    }
}

/**
 ****************************************************************************************
 * @brief Program the activity
 ****************************************************************************************
 */
__STATIC void ld_inq_prog(uint32_t clock)
{
    uint32_t max_win_len;
    uint8_t cs_idx = EM_BT_CS_INQ_INDEX;

    // Point to parameters
    struct ld_inq_env_tag *inq_par = ld_inq_env;
    struct sch_arb_elt_tag *evt = &inq_par->evt;

    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_INQ_PATCH_TYPE, LD_INQ_PROG_BIT, clock);

    max_win_len = CLK_SUB(inq_par->win_end_ts, clock) >> 1;

    // Check to not exceed the maximum value supported by HW
    max_win_len = co_min(max_win_len, 2 * EM_BT_CS_MAXFRMTIME_MAX);

    // Prepare CS
    em_bt_maxfrmtime_setf(cs_idx, max_win_len / 2);

    {
        // Push the programming to SCH PROG
        struct sch_prog_params prog_par;
        prog_par.frm_cbk         = &ld_inq_frm_cbk;
        prog_par.time.hs         = evt->time.hs;
        prog_par.time.hus        = 0;
        prog_par.cs_idx          = cs_idx;
        prog_par.dummy           = cs_idx;
        prog_par.bandwidth       = evt->duration_min;
        prog_par.prio_1          = evt->current_prio;
        prog_par.prio_2          = 0;
        prog_par.prio_3          = rwip_priority[RWIP_PRIO_PAGE_1ST_PKT_IDX].value;
        prog_par.pti_prio        = BT_PTI_INQ_IDX;
        prog_par.add.bt.frm_type = SCH_BT_FRAME_TYPE_NORMAL;
        prog_par.mode            = SCH_PROG_BT;
        sch_prog_push(&prog_par);
    }
}

/**
 ****************************************************************************************
 * @brief Schedule the activity
 ****************************************************************************************
 */
__STATIC void ld_inq_sched(void)
{
    // Point to parameters
    struct sch_arb_elt_tag *evt = &(ld_inq_env->evt);
    struct ld_inq_env_tag *inq_par = ld_inq_env;

    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LD_INQ_PATCH_TYPE, LD_INQ_SCHED_BIT);

    // Schedule the next inquiry window
    do
    {
        // Set state to WAIT, assuming that inquiry will be scheduled
        inq_par->state = INQ_EVT_WAIT;

        // Check if a current window is opened
        if (inq_par->win_end_ts != LD_CLOCK_UNDEF)
        {
            uint32_t next_win_ts;

            SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_ASAP_LIMIT, SCH_ARB_PHASE_0, LD_INQ_EVT_AUTO_RESCHED_ATT, RWIP_PRIO_INC(RWIP_PRIO_INQ_DFT_IDX));
            evt->asap_limit = inq_par->win_end_ts;

            // Check if the number of responses has been reached
            if ((inq_par->nb_rsp_max == 0) || (inq_par->nb_rsp_curr < inq_par->nb_rsp_max))
            {
                // Try to reschedule the current window
                if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
                    break;
            }

            // Check if inquiry is one-shot
            if (inq_par->per_min == 0)
            {
                // Report inquiry end
                ld_inq_end();
                break;
            }

            // Unregister the inquiry process from scheduling parameters
            sch_slice_bg_remove(BT_INQ);

            // Disable Inquiry train counter
            bt_abtraincntl_abtinqen_setf(0);

            // For periodic inquiry, each single inquiry process has to be reported to the Host
            ke_msg_send_basic(LM_INQ_END_IND, TASK_LM, TASK_NONE);

            // Compute timestamp for the next inquiry process
            next_win_ts = CO_MOD(co_rand_word(), (inq_par->per_max - inq_par->per_min));
            next_win_ts += inq_par->per_min;
            next_win_ts *= 2 * 2048;
            next_win_ts = CLK_ADD_2(inq_par->win_start_ts, next_win_ts);
            evt->time.hs = next_win_ts;

            // Clear window's start/end TS
            inq_par->win_end_ts = LD_CLOCK_UNDEF;
            inq_par->win_start_ts = LD_CLOCK_UNDEF;

            // Clear number of responses
            inq_par->nb_rsp_curr = 0;
        }

        // Reset the event settings
        SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_ASAP_NO_LIMIT, SCH_ARB_PHASE_0, LD_INQ_EVT_AUTO_RESCHED_ATT, RWIP_PRIO_INC(RWIP_PRIO_INQ_DFT_IDX));

        // Try to schedule the next window
        if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
            break;

        ASSERT_ERR_FORCE(0);

    }
    while (0);
}

/**
 ****************************************************************************************
 * @brief Handle event start notification
 ****************************************************************************************
 */
__STATIC void ld_inq_evt_start_cbk(struct sch_arb_elt_tag *evt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_INQ_PATCH_TYPE, LD_INQ_EVT_START_CBK_BIT, evt);

    ASSERT_ERR((&(ld_inq_env->evt)) == evt);

    DBG_SWDIAG(INQ, INQ_EVT_START, 1);

    if (evt != NULL)
    {
        // Point to parameters
        struct ld_inq_env_tag *inq_par = ld_inq_env;

        // Check if an inquiry window is ongoing
        if (inq_par->win_end_ts > RWIP_MAX_CLOCK_TIME)
        {
            // Store window's start TS
            inq_par->win_start_ts = evt->time.hs;
            // Store window's end TS
            inq_par->win_end_ts = CLK_ADD_2(inq_par->win_start_ts, inq_par->inq_len * 2 * 2048);

            // Register the inquiry process into scheduling parameters
            sch_slice_bg_add(BT_INQ);

            // Disable Inquiry train counter
            bt_abtraincntl_abtinqen_setf(0);

            // Repetition time for each train
            bt_abtraincntl_abtinqtime_setf(inq_par->n_inq);

            // Starting train according to current CLKN
            bt_abtraincntl_abtinqstartvalue_setf(0);

            // Enable the Inquiry train
            bt_abtraincntl_abtinqen_setf(1);

            // Load the time value
            bt_abtraincntl_abtinqload_setf(1);
        }

        // Program inquiry
        ld_inq_prog(evt->time.hs);

        // Move state
        inq_par->state = INQ_EVT_ACTIVE;
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(INQ, INQ_EVT_START, 0);
}

/**
 ****************************************************************************************
 * @brief Handle event canceled notification
 ****************************************************************************************
 */
__STATIC void ld_inq_evt_canceled_cbk(struct sch_arb_elt_tag *evt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_INQ_PATCH_TYPE, LD_INQ_EVT_CANCELED_CBK_BIT, evt);

    ASSERT_ERR((&(ld_inq_env->evt)) == evt);

    DBG_SWDIAG(INQ, INQ_EVT_CANCELED, 1);

    if (evt != NULL)
    {
        // Check inquiry end
        if (ld_inq_env->state == INQ_EVT_END)
        {
            // Report inquiry end
            ld_inq_end();
        }
        else
        {
            // Increment priority
            evt->current_prio = RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(RWIP_PRIO_INQ_DFT_IDX));

            // Try reschedule
            ld_inq_sched();
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(INQ, INQ_EVT_CANCELED, 0);
}

/**
 ****************************************************************************************
 * @brief Handle frame interrupt
 ****************************************************************************************
 */
__STATIC void ld_inq_frm_isr(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LD_INQ_PATCH_TYPE, LD_INQ_FRM_ISR_BIT);

    DBG_SWDIAG(INQ, INQ_FRM_ISR, 1);

    uint32_t clock = ld_read_clock();

    if (ld_inq_env != NULL)
    {
        // Point to parameters
        struct sch_arb_elt_tag *evt = &(ld_inq_env->evt);
        struct ld_inq_env_tag *inq_par = ld_inq_env;

        // Check inquiry response reception
        ld_inq_rx();

        // Remove event
        sch_arb_remove(&(ld_inq_env->evt), true);

        // Check inquiry end
        if (inq_par->state == INQ_EVT_END)
        {
            // Report inquiry end
            ld_inq_end();
        }
        else
        {
            // Restore original priority
            evt->current_prio = rwip_priority[RWIP_PRIO_INQ_DFT_IDX].value;

            // Try reschedule
            evt->time.hs = clock;
            ld_inq_sched();
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(INQ, INQ_FRM_ISR, 0);
}

/**
 ****************************************************************************************
 * @brief Handle frame interrupt notification
 ****************************************************************************************
 */
__STATIC void ld_inq_frm_cbk(uint32_t timestamp, uint32_t dummy, uint8_t irq_type)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LD_INQ_PATCH_TYPE, LD_INQ_FRM_CBK_BIT, timestamp, dummy, irq_type);

    ASSERT_INFO(dummy == EM_BT_CS_INQ_INDEX, dummy, irq_type);

    switch (irq_type)
    {
    case SCH_FRAME_IRQ_EOF:
    case SCH_FRAME_IRQ_EOF_ABORT_UNDER_PRIO:
    case SCH_FRAME_IRQ_EOF_ABORT_AFTER_PRIO:
    {
        ld_inq_frm_isr();
    }
    break;
    case SCH_FRAME_IRQ_SKIP:
    {
        // Point to parameters
        struct sch_arb_elt_tag *evt = &(ld_inq_env->evt);

        // Remove event
        sch_arb_remove(evt, true);

        // Invoke event cancel callback
        ld_inq_evt_canceled_cbk(evt);
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

void ld_inq_init(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LD_INQ_PATCH_TYPE, LD_INQ_INIT_BIT);

    ld_inq_env = NULL;
}

void ld_inq_reset(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LD_INQ_PATCH_TYPE, LD_INQ_RESET_BIT);

    // Check if inquiry is active
    if (ld_inq_env != NULL)
    {
        // Free event memory
        ke_free(ld_inq_env);
        ld_inq_env = NULL;
    }
}

uint8_t ld_inq_start(struct ld_inquiry_params *params)
{
    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LD_INQ_PATCH_TYPE, LD_INQ_START_BIT, uint8_t, params);
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;
    uint8_t rf_mod;

    // Check if inquiry is inactive
    if (ld_inq_env == NULL)
    {
        // Allocate event
        ld_inq_env = LD_ALLOC_EVT(ld_inq_env_tag);

        if (ld_inq_env != NULL)
        {
            uint32_t clock = ld_read_clock();
            uint8_t bch[LD_BCH_SIZE];
            uint8_t tx_pwr = 0;
            uint8_t cs_idx = EM_BT_CS_INQ_INDEX;

            // Point to parameters
            struct sch_arb_elt_tag *evt = &(ld_inq_env->evt);
            struct ld_inq_env_tag *inq_par = ld_inq_env;

            LD_INIT_EVT(evt, ld_inq_env_tag);

            // Set permission/status of CS as R/W but uninitialized
            DBG_MEM_PERM_SET((const void *)(REG_EM_BT_CS_BASE_ADDR + REG_EM_BT_CS_ADDR_GET(cs_idx)), REG_EM_BT_CS_SIZE, true, true, true);

            // Initialize event parameters (common part)
            evt->cb_cancel        = &ld_inq_evt_canceled_cbk;
            evt->cb_start         = &ld_inq_evt_start_cbk;
            evt->cb_stop          = NULL;
            evt->current_prio        = rwip_priority[RWIP_PRIO_INQ_DFT_IDX].value;
            evt->duration_min        = sch_slice_params.scan_evt_dur;

            // Initialize event parameters (inquiry part)
            inq_par->lap               = params->lap;
            inq_par->inq_len           = params->inq_len;
            inq_par->per_min           = params->per_min;
            inq_par->per_max           = params->per_max;
            inq_par->nb_rsp_max        = params->nb_rsp_max;
            inq_par->nb_rsp_curr       = 0;
            inq_par->tx_pwr_lvl        = params->tx_pwr_lvl;
            inq_par->eir_en            = params->eir_en;
            inq_par->n_inq             = params->n_inq;
            inq_par->win_start_ts      = LD_CLOCK_UNDEF;
            inq_par->win_end_ts        = LD_CLOCK_UNDEF;

            // Compute the BCH with the BD Address (used for FHS for example)
            ld_util_bch_create(&inq_par->lap.A[0], &bch[0]);

            // Convert dBm into HW TX power settings depending on the radio
            rf_mod = rwip_rf.txpwr_mdlt_get(inq_par->tx_pwr_lvl - BT_DFT_TX_PATH_RF_COMPENSATION / 10, 0);
            tx_pwr = rwip_rf.txpwr_cs_get(inq_par->tx_pwr_lvl - BT_DFT_TX_PATH_RF_COMPENSATION / 10, TXPWR_CS_HIGHER, rf_mod);

            // Set control structure fields
            em_bt_frcntl_pack(cs_idx, 1 /*fpoll*/, 0 /*fmwspatt*/, RWIP_COEX_GET(INQ, TXBSY), RWIP_COEX_GET(INQ, RXBSY), RWIP_COEX_GET(INQ, DNABORT), 0 /*samen*/, 1 /*lsam_dsb*/, EM_BT_CS_FMT_INQUIRY);
            em_bt_bdaddr_setf(cs_idx, 0, (inq_par->lap.A[1] << 8) | inq_par->lap.A[0]);
            em_bt_bdaddr_setf(cs_idx, 1, (0 << 8) | inq_par->lap.A[2]);
            em_bt_bdaddr_setf(cs_idx, 2, (0 << 8) | 0);
            em_bt_bch0_setf(cs_idx, (bch[1] << 8) | bch[0]);
            em_bt_bch1_setf(cs_idx, (bch[3] << 8) | bch[2]);
            em_bt_rxmaxbuf_bch2_pack(cs_idx, EIR_DATA_SIZE, 0, bch[4] & EM_BT_BCH2_MASK);

            em_bt_linkcntl_pack(cs_idx,
                                /*aknena*/ params->knudge_en,
                                /*afhena*/ 0,
                                /*laap*/ 0,
                                /*whdsb*/ 0,
                                /*acledr*/ 0,
                                /*aclltaddr*/ 0,
#if (EAVESDROPPING_SUPPORT)
                                /*aclbinackena*/ 0, /*escobinackena*/ 0, /*duplfltena*/ 0,
#endif // EAVESDROPPING_SUPPORT
                                /*linklbl*/ cs_idx);

            em_bt_pwrcntl_pack(cs_idx, /*fhen*/ 1, /*freq*/ 0, /*txpwr*/ tx_pwr);
            em_bt_txrxcntl_pack(cs_idx, 0,
#if (EAVESDROPPING_SUPPORT)
                                0,
#endif // EAVESDROPPING_SUPPORT
                                inq_par->eir_en, 0, 0, 0, 0, 0, 0, 0, 0);
            em_bt_acltxstat_pack(cs_idx, /*txtog*/ 0, /*txflush*/ 0, /*fcrc*/ 0, /*fnak*/ 0, /*rswitch*/ 0, /*waitack*/ 0, /*lastnull*/ 0, /*lasttxack*/ 0, /*lasttxseqn*/ 0);
            em_bt_wincntl_pack(cs_idx, 0, NORMAL_WIN_SIZE);
            em_bt_clkoff0_setf(cs_idx, 0);
            em_bt_clkoff1_setf(cs_idx, 0);
            em_bt_loopcntl_pack(cs_idx, 1, 1);

            if (params->knudge_en)
            {
                // Configure a default knudge increment to occur after 1st 2 x Npage repetitions
                bt_coexifcntl0_inqknudgeinc_setf(BT_KNUDGE_INC);
            }

            // Default clear other control
            em_bt_txdescptr_setf(cs_idx, 0);
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

            rwip_rf.txpwr_cs_set(TYPE_BT, cs_idx, rf_mod, tx_pwr);

            // Schedule event ASAP
            evt->time.hs = clock;

            GLOBAL_INT_DISABLE();

            ld_inq_sched();

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

uint8_t ld_inq_stop(void)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    FUNC_PATCH_ENTRY_NO_PARAM_HAVE_RETURN(LD_INQ_PATCH_TYPE, LD_INQ_STOP_BIT, uint8_t);

    GLOBAL_INT_DISABLE();

    if (ld_inq_env != NULL)
    {
        // Point to parameters
        struct sch_arb_elt_tag *evt = &(ld_inq_env->evt);
        struct ld_inq_env_tag *inq_par = ld_inq_env;

        switch (inq_par->state)
        {
        case INQ_EVT_WAIT:
        {
            // Remove event
            sch_arb_remove(evt, false);

            // Report inquiry end
            ld_inq_end();
        }
        break;

        case INQ_EVT_ACTIVE:
        {
            // Clear frame duration (in case the CS has not been prefetched yet)
            em_bt_maxfrmtime_setf(EM_BT_CS_INQ_INDEX, 1);

            // Abort inquiry (in CS has already been prefetched)
            bt_rwbtcntl_pageinq_abort_setf(1);

            // Move state
            inq_par->state = INQ_EVT_END;
        }
        break;

        default:
        {
            // Nothing to do
            ASSERT_INFO_FORCE(0, inq_par->state, 0);
        }
        break;
        }

        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return (status);
}


///@} LDINQ
