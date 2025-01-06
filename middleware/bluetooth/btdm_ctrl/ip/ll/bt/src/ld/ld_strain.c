/**
****************************************************************************************
*
* @file ld_strain.c
*
* @brief LD Synchronization Train source code
*
* Copyright (C) RivieraWaves 2009-2015
*
****************************************************************************************
*/

/**
 ****************************************************************************************
 * @addtogroup LDSTRAIN
 * @ingroup LD
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"    // stack configuration

#if CSB_SUPPORT || PCA_SUPPORT

#include <string.h>

#include "co_utils.h"
#include "co_math.h"

#include "arch.h"
#include "rwip.h"
#include "ke_mem.h"

#include "ld.h"             // link driver API
#include "ld_util.h"        // link driver utilities
#include "ld_int.h"         // link driver internal

#include "lb.h"             // link broadcast API

#include "sch_arb.h"            // Scheduling Arbiter
#include "sch_prog.h"           // Scheduling Programmer

#include "reg_ipcore.h"         // IP core registers
#include "reg_btcore.h"         // BT core registers
#include "reg_em_bt_cs.h"       // BT EM Control Structure
#include "reg_em_bt_txdesc.h"   // BT EM TX descriptors
#include "btdm_patch.h"

/*
 * DEFINES
 *****************************************************************************************
 */

/// Synchronization Train event duration min (in us)
#define LD_STRAIN_EVT_DUR_MIN         (SLOT_SIZE)

/// Synchronization Train event states
enum STRAIN_EVT_STATE
{
    STRAIN_EVT_WAIT,
    STRAIN_EVT_ACTIVE,
    STRAIN_EVT_END,
};

/*
 * STRUCTURE DEFINITION
 *****************************************************************************************
 */

/// LD STRAIN environment structure
struct ld_strain_env_tag
{
    /// Synchronization Train event
    struct sch_arb_elt_tag evt;
    /// Synchronization Train event timestamp (in half-slots)
    uint32_t event_ts;
    /// Synchronization Train Timeout (in slots)
    uint32_t strain_to;
    /// Synchronization Trian End Time (in half-slots)
    uint32_t strain_end;
    /// Amount of time between consecutive synchronization trains in slots (625 us)
    uint16_t strain_intv;
    /// Connectionless slave broadcast offset (in half-slots)
    uint16_t csb_offset;
    /// Connectionless slave broadcast interval (in half-slots)
    uint16_t csb_intv;
    /// State
    uint8_t state;
    /// Channel index
    uint8_t ch_ind;
    /// Started for coarse clock adjustment
    bool coarse_clk_adj;
};

/*
 * VARIABLE DEFINITION
 *****************************************************************************************
 */

/// LD STRAIN environment variable
__STATIC struct ld_strain_env_tag *ld_strain_env;


/*
 * LOCAL FUNCTIONS DECLARATION
 *****************************************************************************************
 */

__STATIC void ld_strain_frm_cbk(uint32_t timestamp, uint32_t dummy, uint8_t irq_type);


/*
 * LOCAL FUNCTIONS DEFINITION
 *****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Report the end of the activity
 ****************************************************************************************
 */
__STATIC void ld_strain_end(uint8_t status)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_STRAIN_PATCH_TYPE, LD_STRAIN_END_BIT, status);

    if (ld_strain_env)
    {
        // Report synchronization end to LB
        struct lb_strain_end_ind *ind = KE_MSG_ALLOC(LB_STRAIN_END_IND, TASK_LB, TASK_NONE, lb_strain_end_ind);
        struct ld_strain_env_tag *strain_par = ld_strain_env;
        ind->coarse_clk_adj = strain_par->coarse_clk_adj;
        ind->status = status; //CO_ERROR_NO_ERROR;
        ke_msg_send(ind);

        // Remove permission/status of CS as now unused
        DBG_MEM_PERM_SET((const void *)(REG_EM_BT_CS_BASE_ADDR + REG_EM_BT_CS_ADDR_GET(EM_BT_CS_STRAIN_INDEX)), REG_EM_BT_CS_SIZE, false, false, false);

        // Free event memory
        ke_free(ld_strain_env);
        ld_strain_env = NULL;
    }
}

/**
 ****************************************************************************************
 * @brief Handle event start notification
 ****************************************************************************************
 */
__STATIC void ld_strain_evt_start_cbk(struct sch_arb_elt_tag *evt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_STRAIN_PATCH_TYPE, LD_STRAIN_EVT_START_CBK_BIT, evt);

    ASSERT_ERR((&(ld_strain_env->evt)) == evt);

    if (evt != NULL)
    {
        // Make sure that the master clock considered is even (corresponds to a BT slot)
        ASSERT_ERR(!(evt->time.hs & 0x1));

        // Point to parameters
        struct ld_strain_env_tag *strain_par = ld_strain_env;
        uint8_t cs_idx = EM_BT_CS_STRAIN_INDEX;
        uint8_t txdesc_idx = EM_BT_TXDESC_STP_INDEX;
        uint32_t future_csb_inst;
        int32_t diff;

        // Reload TX descriptor
        em_bt_txptr_pack(txdesc_idx, 0, 0);
        em_bt_txdescptr_setf(cs_idx, REG_EM_ADDR_GET(BT_TXDESC, txdesc_idx));

        // Set the current frequency
        em_bt_pwrcntl_freq_setf(cs_idx, ld_sync_train_channels[strain_par->ch_ind] / 2);

        // Update local clock value in STP
        em_wr32p(EM_BT_STPTXBUF_OFFSET + STP_CLK_POS, evt->time.hs);

        // Find the next CSB instant
        if (strain_par->coarse_clk_adj)
        {
            diff = SYNC_TRAIN_CSB_INSTANT_OFFSET_CLK_ADJ;
        }
        else
        {
            diff = strain_par->csb_offset - CO_MOD(evt->time.hs, strain_par->csb_intv);
            if (diff <= 0)
            {
                diff += strain_par->csb_intv;
            }
        }
        future_csb_inst = CLK_ADD_2(evt->time.hs, diff) >> 1;

        // Update future CSB instant (expressed in slots) in STP
        em_wr32p(EM_BT_STPTXBUF_OFFSET + STP_FUT_CSB_INST_POS, future_csb_inst);

        {
            // Push the programming to SCH PROG
            struct sch_prog_params prog_par;
            prog_par.frm_cbk         = &ld_strain_frm_cbk;
            prog_par.time.hs         = evt->time.hs;
            prog_par.time.hus        = 0;
            prog_par.cs_idx          = cs_idx;
            prog_par.dummy           = cs_idx;
            prog_par.bandwidth       = evt->duration_min;
            prog_par.prio_1          = evt->current_prio;
            prog_par.prio_2          = 0;
            prog_par.prio_3          = 0;
            prog_par.pti_prio        = BT_PTI_BCAST_IDX;
            prog_par.add.bt.frm_type = SCH_BT_FRAME_TYPE_CSB;
            prog_par.mode            = SCH_PROG_BT;
            sch_prog_push(&prog_par);
        }

        // Move state
        strain_par->state = STRAIN_EVT_ACTIVE;
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }
}

/**
 ****************************************************************************************
 * @brief Handle event canceled notification
 ****************************************************************************************
 */
__STATIC void ld_strain_evt_canceled_cbk(struct sch_arb_elt_tag *evt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_STRAIN_PATCH_TYPE, LD_STRAIN_EVT_CANCELED_CBK_BIT, evt);

    ASSERT_ERR((&(ld_strain_env->evt)) == evt);

    if (evt != NULL)
    {
        ASSERT_ERR(ld_strain_env->state == STRAIN_EVT_WAIT);

        // Increment priority
        evt->current_prio = RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(RWIP_PRIO_STRAIN_DFT_IDX));

        // Reschedule ASAP
        if (sch_arb_insert(evt) != SCH_ARB_ERROR_OK)
        {
            ASSERT_ERR_FORCE(0);
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }
}

/**
 ****************************************************************************************
 * @brief Handle frame interrupt
 ****************************************************************************************
 */
__STATIC void ld_strain_frm_isr(uint32_t timestamp)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_STRAIN_PATCH_TYPE, LD_STRAIN_FRM_ISR_BIT, timestamp);

    if (ld_strain_env != NULL)
    {
        // Point to parameters
        struct sch_arb_elt_tag *evt = &(ld_strain_env->evt);
        struct ld_strain_env_tag *strain_par = ld_strain_env;

        uint32_t clock = ld_read_clock();

        // Remove event
        sch_arb_remove(evt, true);

        // Check end request
        if (strain_par->state == STRAIN_EVT_END)
        {
            // End the activity
            ld_strain_end(CO_ERROR_NO_ERROR);
        }
        else
        {
            // Check if the sync train packet has been sent over the 3 frequencies
            if (++strain_par->ch_ind >= SYNC_TRAIN_CHANNEL_NB)
            {
                int8_t delay;
                uint8_t max_delay = (strain_par->coarse_clk_adj) ? SYNC_TRAIN_DELAY_MAX_CLK_ADJ : SYNC_TRAIN_DELAY_MAX_DFT;

                // Reset the frequency index counter
                strain_par->ch_ind = 0;

                // Find the next sync train event timestamp
                while (CLK_DIFF(clock, strain_par->event_ts) <= rwip_prog_delay)
                {
                    strain_par->event_ts += 2 * strain_par->strain_intv;
                }

                // Add sync train interval with a little random back off period
                delay = (co_rand_byte() + ip_finetimtgt_get() + (clock >> 1)) & (max_delay - 1);
                evt->time.hs = CO_ALIGN4_HI(CLK_ADD_2(strain_par->event_ts, 2 * delay));
            }

            // Restore original priority
            evt->current_prio = rwip_priority[RWIP_PRIO_STRAIN_DFT_IDX].value;

            if (CLK_LOWER_EQ(strain_par->strain_end, evt->time.hs))
            {
                // End the activity
                ld_strain_end(CO_ERROR_NO_ERROR);
            }
            // Try reschedule
            else if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
            {
                strain_par->state = STRAIN_EVT_WAIT;
            }
            else
            {
                // End the activity
                ld_strain_end(CO_ERROR_UNSPECIFIED_ERROR);
            }
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }
}

/**
 ****************************************************************************************
 * @brief Handle frame interrupt notification
 ****************************************************************************************
 */
__STATIC void ld_strain_frm_cbk(uint32_t timestamp, uint32_t dummy, uint8_t irq_type)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LD_STRAIN_PATCH_TYPE, LD_STRAIN_FRM_CBK_BIT, timestamp, dummy, irq_type);

    switch (irq_type)
    {
    case SCH_FRAME_IRQ_EOF:
    case SCH_FRAME_IRQ_EOF_ABORT_UNDER_PRIO:
    case SCH_FRAME_IRQ_EOF_ABORT_AFTER_PRIO:
    {
        ld_strain_frm_isr(timestamp);
    }
    break;
    case SCH_FRAME_IRQ_SKIP:
    {
        ld_strain_frm_isr(timestamp);
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

void ld_strain_init(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LD_STRAIN_PATCH_TYPE, LD_STRAIN_INIT_BIT);

    ld_strain_env = NULL;
}

void ld_strain_reset(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LD_STRAIN_PATCH_TYPE, LD_STRAIN_RESET_BIT);

    // Check if synchronization sTrainis active
    if (ld_strain_env != NULL)
    {
        // Free event memory
        ke_free(ld_strain_env);
        ld_strain_env = NULL;
    }
}

uint8_t ld_strain_start(uint32_t strain_to, uint32_t strain_intv, bool coarse_clk_adj, struct bt_ch_map *ch_map, uint16_t csb_offset, uint16_t csb_intv, uint8_t csb_lt_addr, uint8_t svc_data)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    FUNC_PATCH_ENTRY_VARI_PARAM_HAVE_RETURN(LD_STRAIN_PATCH_TYPE, LD_STRAIN_START_BIT, uint8_t, 8, strain_to, strain_intv, coarse_clk_adj, ch_map, csb_offset, csb_intv, csb_lt_addr, svc_data);

    // Check if synchronization Train is inactive
    if (ld_strain_env == NULL)
    {
        // Allocate event
        ld_strain_env = LD_ALLOC_EVT(ld_strain_env_tag);

        if (ld_strain_env != NULL)
        {
            uint32_t clock = ld_read_clock();
            uint8_t cs_idx = EM_BT_CS_STRAIN_INDEX;
            uint8_t txdesc_idx = EM_BT_TXDESC_STP_INDEX;

            // Point to parameters
            struct sch_arb_elt_tag *evt = &(ld_strain_env->evt);
            struct ld_strain_env_tag *strain_par = ld_strain_env;

            LD_INIT_EVT(evt, ld_strain_env_tag);

            // Set permission/status of CS as R/W but uninitialized
            DBG_MEM_PERM_SET((const void *)(REG_EM_BT_CS_BASE_ADDR + REG_EM_BT_CS_ADDR_GET(cs_idx)), REG_EM_BT_CS_SIZE, true, true, true);

            // Initialize event parameters (common part)
            evt->cb_cancel        = &ld_strain_evt_canceled_cbk;
            evt->cb_start         = &ld_strain_evt_start_cbk;
            evt->cb_stop          = NULL;
            evt->current_prio        = rwip_priority[RWIP_PRIO_STRAIN_DFT_IDX].value;
            evt->duration_min        = 2 * LD_STRAIN_EVT_DUR_MIN;
            evt->asap_limit          = CLK_ADD_2(clock, 2 * strain_to);
            SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_ASAP_LIMIT, SCH_ARB_PHASE_0, 0, 0);

            // Initialize event parameters (synchronization Train part)
            strain_par->strain_intv        = strain_intv;
            strain_par->strain_to          = strain_to;
            strain_par->strain_end         = CLK_ADD_2(clock, 2 * strain_to);
            strain_par->csb_offset         = csb_offset;
            strain_par->csb_intv           = csb_intv;
            strain_par->coarse_clk_adj     = coarse_clk_adj;

            // Build sync train packet
            ld_util_stp_pk(EM_BT_STPTXBUF_OFFSET, 0, 0, ch_map, &ld_env.local_bd_addr, csb_intv / 2, csb_lt_addr, svc_data);

            // Prepare TX descriptor
            em_bt_txheader_pack(txdesc_idx, 0, 0, 0, 0, 0, DM3_TYPE, 0);
            em_bt_txpheader_pack(txdesc_idx, 0, STP_PACKET_SIZE, 0, LLID_START);
            em_bt_txaclbufptr_setf(txdesc_idx, EM_BT_STPTXBUF_OFFSET);
            em_bt_txlmbufptr_setf(txdesc_idx, 0);
            em_bt_txptr_pack(txdesc_idx, 0, 0);

            // Set control structure fields
            em_bt_frcntl_pack(cs_idx, 1 /*fpoll*/, 0 /*fmwspatt*/, RWIP_COEX_GET(BCAST, TXBSY), RWIP_COEX_GET(BCAST, RXBSY), RWIP_COEX_GET(BCAST, DNABORT), 0 /*samen*/, 1 /*lsam_dsb*/, EM_BT_CS_FMT_MST_BCST);
            em_bt_bdaddr_setf(cs_idx, 0, (ld_env.local_bd_addr.addr[1] << 8) | ld_env.local_bd_addr.addr[0]);
            em_bt_bdaddr_setf(cs_idx, 1, (ld_env.local_bd_addr.addr[3] << 8) | ld_env.local_bd_addr.addr[2]);
            em_bt_bdaddr_setf(cs_idx, 2, (ld_env.local_bd_addr.addr[5] << 8) | ld_env.local_bd_addr.addr[4]);
            em_bt_bch0_setf(cs_idx, (ld_env.local_bch[1] << 8) | ld_env.local_bch[0]);
            em_bt_bch1_setf(cs_idx, (ld_env.local_bch[3] << 8) | ld_env.local_bch[2]);
            em_bt_rxmaxbuf_bch2_pack(cs_idx, 0, 0, ld_env.local_bch[4] & EM_BT_BCH2_MASK);
            em_bt_txdescptr_setf(cs_idx, REG_EM_ADDR_GET(BT_TXDESC, txdesc_idx));
            em_bt_acltxstat_set(cs_idx, 0);
            em_bt_aclrxstat_set(cs_idx, 0);
            em_bt_linkcntl_pack(cs_idx,
                                /*aknena */ 0,
                                /*afhena*/ 0,
                                /*laap*/ 0,
                                /*whdsb*/ 1,
                                /*acledr*/ 0,
                                /*aclltaddr*/ LT_ADDR_BCST,
#if (EAVESDROPPING_SUPPORT)
                                /*aclbinackena*/ 0, /*escobinackena*/ 0, /*duplfltena*/ 0,
#endif // EAVESDROPPING_SUPPORT
                                /*linklbl*/ cs_idx);
            em_bt_pwrcntl_pack(cs_idx, /*fhen*/ 0, /*freq*/ 0, /*txpwr*/ rwip_rf.txpwr_max);
            em_bt_clkoff0_setf(cs_idx, 0);
            em_bt_clkoff1_setf(cs_idx, 0);
            em_bt_txrxcntl_pack(cs_idx, /*rxthr*/ 0,
#if (EAVESDROPPING_SUPPORT)
                                /*stopmod*/ 0,
#endif // EAVESDROPPING_SUPPORT
                                /*rxeir*/ 0, /*rxbcry*/ 0, /*rxcrypt*/ 0, /*stopmod/escofnak*/ 0, /*nullflt*/ 0, /*extpaen*/ 0, /*txeir*/ 0, /*txbcry*/ 0, /*txcrypt*/ 0);
            em_bt_loopcntl_pack(cs_idx, /*tonb*/ 0, /*attnb*/ 1);

            // Default clear other control
            em_bt_wincntl_set(cs_idx, 0);
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

            // Schedule event ASAP
            evt->time.hs = clock;
            rwip_rf.txpwr_cs_set(TYPE_BT, cs_idx, rwip_rf.txpwr_max_mod, rwip_rf.txpwr_max);


            GLOBAL_INT_DISABLE();

            if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
            {
                // Initialize sync train event timestamp (fixing the anchor point for subsequent events)
                strain_par->event_ts = evt->time.hs;

                // Initialize event state
                strain_par->state = STRAIN_EVT_WAIT;
            }
            else
            {
                ASSERT_ERR_FORCE(0);
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

uint8_t ld_strain_stop(void)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    FUNC_PATCH_ENTRY_NO_PARAM_HAVE_RETURN(LD_STRAIN_PATCH_TYPE, LD_STRAIN_STOP_BIT, uint8_t);

    GLOBAL_INT_DISABLE();

    if (ld_strain_env != NULL)
    {
        // Point to parameters
        struct sch_arb_elt_tag *evt = &(ld_strain_env->evt);
        struct ld_strain_env_tag *strain_par = ld_strain_env;

        switch (strain_par->state)
        {
        case STRAIN_EVT_WAIT:
        {
            // Remove event
            sch_arb_remove(evt, false);

            // Report synchronization train end
            ld_strain_end(CO_ERROR_NO_ERROR);
        }
        break;

        case STRAIN_EVT_ACTIVE:
        {
            // Move state
            strain_par->state = STRAIN_EVT_END;
        }
        break;

        default:
        {
            // Nothing to do
            ASSERT_INFO_FORCE(0, strain_par->state, 0);
        }
        break;
        }

        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return (status);
}

#endif // CSB_SUPPORT || PCA_SUPPORT

///@} LDSTRAIN
