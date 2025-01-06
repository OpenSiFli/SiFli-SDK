/**
****************************************************************************************
*
* @file ld_bcst.c
*
* @brief LD BCST source code
*
* Copyright (C) RivieraWaves 2015
*
****************************************************************************************
*/

/**
 ****************************************************************************************
 * @addtogroup LDBCST
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
#include <stddef.h>

#include "co_math.h"
#include "co_utils.h"
#include "co_bt.h"

#include "arch.h"
#include "rwip.h"
#include "ke_mem.h"

#include "ld.h"             // link driver API
#include "ld_util.h"        // link driver utilities
#include "ld_int.h"         // link driver internal
#include "ld_util.h"        // link driver utilities

#include "lb.h"

#include "dbg.h"

#include "sch_arb.h"            // Scheduling Arbiter
#include "sch_prog.h"           // Scheduling Programmer

#include "reg_ipcore.h"         // IP core registers
#include "reg_btcore.h"         // BT core registers
#include "reg_btcore_esco.h"    // eSCO registers
#include "reg_btcore_audio.h"   // Audio registers
#include "reg_em_bt_cs.h"       // BT EM Control Structure
#include "reg_em_bt_rxdesc.h"   // BT EM RX descriptors
#include "reg_em_bt_txdesc.h"   // BT EM TX descriptors
#include "btdm_patch.h"

/*
 * DEFINES
 *****************************************************************************************
 */

/*
 * ENUMERATIONS DEFINITIONS
 ****************************************************************************************
 */

/// ACL event hop mode
enum BCST_HOP_MODE
{
    BCST_STD_HOP,
    BCST_AFH_HOP,
};

/// ACL event types - LMP transmission, or ACL transmission
enum BCST_EVT_TYPE
{
    BCST_ACL_EVT_TYPE,
    BCST_LMP_EVT_TYPE
};

/// BCST event states
enum BCST_EVT_STATE
{
    BCST_EVT_WAIT,
    BCST_EVT_ACTIVE,
    BCST_EVT_END,
};


/*
 * STRUCTURE DEFINITION
 *****************************************************************************************
 */
/// LD broadcast ACL environment structure
struct ld_bcst_env_tag
{
    /// Broadcast ACL event
    struct sch_arb_elt_tag evt;

    /// Active hop schemes
    /**
    * Hop schemes - bit=1 => used | bit=0 => not used
    *  bit 0: afh_hop
    *  bit 1: std_hop
    */
    uint8_t active_hop_types;

    /// ACL TX queue
    struct co_list queue_acl_tx;

    /// Buffer element of the packet currently under transmission
    struct bt_em_acl_buf_elt *curr_acl_buf_elt;
    /// EM position of the next data to transmit
    uint16_t data_ptr;
    /// Length of data remaining to transmit from the current or next buffer (bytes)
    uint16_t data_len;

    /// number of acl broadcast retransmissions
    uint8_t acl_nbc;
    /// number of lmp broadcast retransmissions
    uint8_t lmp_nbc;

    /// reduced number of nbc attempts (to workaround bandwidth issues)
    uint8_t nbc_att;

    /// Buffer element of the packet currently under transmission
    struct bt_em_lmp_buf_elt *curr_lmp_buf_elt;

    /// LMP specific clock instant (in half-slots, clock based value)
    uint32_t lmp_clk;

    /// NO_ASAP priority control (priority value)
    uint8_t no_asap_prio;

    /// Evt Type (@see enum  BCST_EVT_TYPE)
    uint8_t evt_type;

    /// Hop mode (@see enum  BCST_HOP_MODE)
    uint8_t hop_mode;

    /// State (@see enum  BCST_EVT_STATE)
    uint8_t state;
};


/*
 * VARIABLES DEFINITIONS
 *****************************************************************************************
 */

/// LD broadcast ACL environment variable
__STATIC struct ld_bcst_env_tag *ld_bcst_env;

/*
 * LOCAL FUNCTIONS DECLARATION
 *****************************************************************************************
 */
__STATIC void ld_bcst_frm_cbk(uint32_t timestamp, uint32_t dummy, uint8_t irq_type);
__STATIC void ld_bcst_start(struct bt_ch_map *afh_ch_map, uint8_t nbc);

/*
 * LOCAL FUNCTIONS DEFINITION
 *****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Process broadcast packet transmits
 ****************************************************************************************
 */
__STATIC void ld_bcst_process_pkt_tx(void)
{
    // Point to parameters
    struct ld_bcst_env_tag *bcst_par = ld_bcst_env;

    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LD_BCST_PATCH_TYPE, LD_BCST_PROCESS_PKT_TX_BIT);

    DBG_SWDIAG(BCST, PDU_TX_CFM, 1);

    if (BCST_LMP_EVT_TYPE == bcst_par->evt_type)
    {
        // Free the buffer
        bt_util_buf_lmp_tx_free(bcst_par->curr_lmp_buf_elt->buf_ptr);
        bcst_par->curr_lmp_buf_elt = NULL;
    }
    else if (bcst_par->data_len == 0) // BCST_ACL_EVT_TYPE
    {
        // Report ACL TX confirmation
        struct lb_acl_tx_cfm *msg = KE_MSG_ALLOC(LB_ACL_TX_CFM, TASK_LB, TASK_NONE, lb_acl_tx_cfm);
        msg->em_buf = bcst_par->curr_acl_buf_elt->buf_ptr;
        ke_msg_send(msg);

        // Free the buffer
        bt_util_buf_acl_tx_free(bcst_par->curr_acl_buf_elt->buf_ptr);
        bcst_par->curr_acl_buf_elt = NULL;
    }

    DBG_SWDIAG(BCST, PDU_TX_CFM, 0);
}

/**
 ****************************************************************************************
 * @brief Schedule broadcast transmissions
 ****************************************************************************************
 */
__STATIC void ld_bcst_sched(void)
{
    // Point to parameters
    struct sch_arb_elt_tag *evt = &(ld_bcst_env->evt);
    struct ld_bcst_env_tag *bcst_par = ld_bcst_env;

    uint8_t txdesc_idx = EM_BT_TXDESC_ACT_BCST_INDEX;

    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LD_BCST_PATCH_TYPE, LD_BCST_SCHED_BIT);

    // Set hop mode
    bcst_par->hop_mode = (bcst_par->active_hop_types & LD_AFH_HOP_USED) ? BCST_AFH_HOP : BCST_STD_HOP;

    // Set default priority & arbitration type
    SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_ASAP_NO_LIMIT, SCH_ARB_PHASE_0, 0, 0);
    evt->current_prio = rwip_priority[RWIP_PRIO_BCST_DFT_IDX].value;

    if (bcst_par->curr_lmp_buf_elt)
    {
        struct bt_em_lmp_buf_elt *lmp_buf_elt = bcst_par->curr_lmp_buf_elt;

        // Fill descriptor
        em_bt_txpheader_pack(txdesc_idx, 0, lmp_buf_elt->length, 1, LLID_CNTL);
        em_bt_txlmbufptr_setf(txdesc_idx, lmp_buf_elt->buf_ptr);
        // Disable waitack for SEQN toggle
        em_bt_acltxstat_waitack_setf(EM_BT_CS_ACT_BCST_INDEX, 0);

        // Release descriptor
        em_bt_txptr_txdone_setf(txdesc_idx, 0);

        bcst_par->nbc_att = bcst_par->lmp_nbc;

        // Some LMP require a specific clock instant, If clock specified override NO_ASAP
        if (bcst_par->lmp_clk != LD_BCST_LMP_CLK_UNUSED)
        {
            // Try to schedule broadcast LMP
            evt->time.hs = bcst_par->lmp_clk;
            SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_NO_ASAP, SCH_ARB_PHASE_0, 0, 0);
            evt->current_prio = bcst_par->no_asap_prio;
        }

        if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
        {
            bcst_par->evt_type = BCST_LMP_EVT_TYPE;
            bcst_par->state = BCST_EVT_WAIT;
        }
        else
        {
            if (bcst_par->lmp_clk != LD_BCST_LMP_CLK_UNUSED)
            {
                // Free the buffer
                bt_util_buf_lmp_tx_free(bcst_par->curr_lmp_buf_elt->buf_ptr);
                bcst_par->curr_lmp_buf_elt = NULL;

                bcst_par->no_asap_prio = RWIP_PRIO_ADD_2(bcst_par->no_asap_prio, RWIP_PRIO_INC(RWIP_PRIO_BCST_ACT_IDX));
            }
            else
            {
                ASSERT_ERR(0);
            }
        }
    }
    else if (bcst_par->curr_acl_buf_elt)
    {
        if (bcst_par->data_len > 0)
        {
            // Find packet length
            uint16_t new_txlength = co_min(bcst_par->data_len, DM1_PACKET_SIZE);

            em_bt_txpheader_pack(txdesc_idx, 0, new_txlength, 0, LLID_CONTINUE);
            em_bt_txaclbufptr_setf(txdesc_idx, bcst_par->data_ptr);
            // Disable waitack for SEQN toggle
            em_bt_acltxstat_waitack_setf(EM_BT_CS_ACT_BCST_INDEX, 0);

            // Release descriptor
            em_bt_txptr_txdone_setf(txdesc_idx, 0);

            // Update remaining data length
            bcst_par->data_len -= new_txlength;
            // Update data pointer
            bcst_par->data_ptr += new_txlength;

            // Reset the Nbc attempts
            bcst_par->nbc_att = bcst_par->acl_nbc;

            // Try scheduling broadcast ACL
            if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
            {
                bcst_par->evt_type = BCST_ACL_EVT_TYPE;
                bcst_par->state = BCST_EVT_WAIT;
            }
            else
            {
                ASSERT_ERR(0);
            }
        }
    }

    if (!bcst_par->curr_lmp_buf_elt && !bcst_par->curr_acl_buf_elt) // If no event inserted, then check for further broadcast
    {
        // Get new buffer (ACL)
        struct bt_em_acl_buf_elt *buf_elt = (struct bt_em_acl_buf_elt *) co_list_pop_front(&bcst_par->queue_acl_tx);

        if (buf_elt != NULL)
        {
            uint16_t data_len = GETF(buf_elt->data_len_flags, BT_EM_ACL_DATA_LEN);

            if (data_len > 0)
            {
                uint8_t new_txllid;

                // Find packet length
                uint16_t new_txlength = co_min(data_len, DM1_PACKET_SIZE);

                // Save the buffer as the current one
                bcst_par->curr_acl_buf_elt = buf_elt;

                // Set data length and pointer
                bcst_par->data_len = data_len;
                bcst_par->data_ptr = buf_elt->buf_ptr;

                // Check packet boundary flag
                new_txllid = LLID_CONTINUE;
                if (GETF(buf_elt->data_len_flags, BT_EM_ACL_PBF) != PBF_CONT_HL_FRAG)
                {
                    new_txllid = LLID_START;
                }

                // Fill descriptor
                em_bt_txpheader_pack(txdesc_idx, 0, new_txlength, 0, new_txllid);
                em_bt_txaclbufptr_setf(txdesc_idx, bcst_par->data_ptr);
                // Disable waitack for SEQN toggle
                em_bt_acltxstat_waitack_setf(EM_BT_CS_ACT_BCST_INDEX, 0);

                // Release descriptor
                em_bt_txptr_txdone_setf(txdesc_idx, 0);

                // Update remaining data length
                bcst_par->data_len -= new_txlength;
                // Update data pointer
                bcst_par->data_ptr += new_txlength;

                // Reset the Nbc attempts
                bcst_par->nbc_att = bcst_par->acl_nbc;

                // Try scheduling broadcast ACL
                if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
                {
                    bcst_par->evt_type = BCST_ACL_EVT_TYPE;
                    bcst_par->state = BCST_EVT_WAIT;
                }
                else
                {
                    ASSERT_ERR(0);
                }
            }
            else
            {
                ASSERT_ERR(0);
            }
        }
        else
        {
            // Remove permission/status of CS as now unused
            DBG_MEM_PERM_SET((const void *)(REG_EM_BT_CS_BASE_ADDR + REG_EM_BT_CS_ADDR_GET(EM_BT_CS_ACT_BCST_INDEX)), REG_EM_BT_CS_SIZE, false, false, false);

            // Free common event memory
            ke_free(ld_bcst_env);
            ld_bcst_env = NULL;
        }
    }
}

/**
 ****************************************************************************************
 * @brief Reschedule for broadcast retransmissions
 ****************************************************************************************
 */
__STATIC void ld_bcst_resched(void)
{
    struct sch_arb_elt_tag *evt = &(ld_bcst_env->evt);
    struct ld_bcst_env_tag *bcst_par = ld_bcst_env;

    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LD_BCST_PATCH_TYPE, LD_BCST_RESCHED_BIT);

    // Transmit same SEQN on retransmissions
    em_bt_acltxstat_waitack_setf(EM_BT_CS_ACT_BCST_INDEX, 1);

    // Clear txdone
    em_bt_txptr_txdone_setf(EM_BT_TXDESC_ACT_BCST_INDEX, 0);

    // Try scheduling broadcast ACL
    if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
    {
        bcst_par->state = BCST_EVT_WAIT;
    }
    else
    {
        ASSERT_ERR(0);
    }
}

/**
 ****************************************************************************************
 * @brief Handle event start notification
 ****************************************************************************************
 */
__STATIC void ld_bcst_evt_start_cbk(struct sch_arb_elt_tag *evt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_BCST_PATCH_TYPE, LD_BCST_EVT_START_CBK_BIT, evt);

    DBG_SWDIAG(BCST, EVT_START, 1);

    if (evt != NULL)
    {
        // Point to parameters
        struct ld_bcst_env_tag *bcst_par = ld_bcst_env;
        uint8_t cs_idx = EM_BT_CS_ACT_BCST_INDEX;

        {
            // Push the programming to SCH PROG
            struct sch_prog_params prog_par;
            prog_par.frm_cbk         = &ld_bcst_frm_cbk;
            prog_par.time.hs         = evt->time.hs;
            prog_par.time.hus        = 0;
            prog_par.cs_idx          = cs_idx;
            prog_par.dummy           = cs_idx;
            prog_par.bandwidth       = evt->duration_min;
            prog_par.prio_1          = evt->current_prio;
            prog_par.prio_2          = 0;
            prog_par.prio_3          = 0;
            prog_par.pti_prio        = BT_PTI_BCAST_IDX;
            prog_par.add.bt.frm_type = SCH_BT_FRAME_TYPE_NORMAL;
            prog_par.mode            = SCH_PROG_BT;
            sch_prog_push(&prog_par);
        }

        // Program control structure
        em_bt_loopcntl_att_nb_setf(cs_idx, 1);

        if (bcst_par->hop_mode == BCST_AFH_HOP)
        {
            em_bt_linkcntl_afhena_setf(cs_idx, 1);
        }
        else // bcst_par->hop_mode == BCST_STD_HOP
        {
            em_bt_linkcntl_afhena_setf(cs_idx, 0);
        }

        // Move state
        bcst_par->state = BCST_EVT_ACTIVE;
    }
    else
    {
        ASSERT_ERR(0);
    }

    DBG_SWDIAG(BCST, EVT_START, 0);
}

/**
 ****************************************************************************************
 * @brief Handle event canceled notification
 ****************************************************************************************
 */
__STATIC void ld_bcst_evt_canceled_cbk(struct sch_arb_elt_tag *evt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_BCST_PATCH_TYPE, LD_BCST_EVT_CANCELED_CBK_BIT, evt);

    DBG_SWDIAG(BCST, EVT_CANCELED, 1);

    if (evt != NULL)
    {
        if (SCH_ARB_FLAG_ASAP_NO_LIMIT == SCH_ARB_ASAP_STG_TYPE_GET(evt))
        {
            // Delay with a random number between 2 and 9 slots
            evt->time.hs += 2 * ((2 + ((co_rand_byte() + ip_finetimtgt_get() + (evt->time.hs >> 1)) & 0x07)) & 0x0E);
            evt->time.hs &= RWIP_MAX_CLOCK_TIME;

            // Reschedule according to the current mode
            ld_bcst_resched();
        }
        else
        {
            struct ld_bcst_env_tag *bcst_par = ld_bcst_env;

            // Increment priority
            bcst_par->no_asap_prio = RWIP_PRIO_ADD_2(bcst_par->no_asap_prio, RWIP_PRIO_INC(RWIP_PRIO_BCST_ACT_IDX));

            // Process packet tx
            ld_bcst_process_pkt_tx();

            // Check next transmission
            ld_bcst_sched();
        }
    }
    else
    {
        ASSERT_ERR(0);
    }

    DBG_SWDIAG(BCST, EVT_CANCELED, 0);
}

/**
 ****************************************************************************************
 * @brief Handle frame interrupt
 ****************************************************************************************
 */
__STATIC void ld_bcst_frm_isr(uint32_t timestamp)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_BCST_PATCH_TYPE, LD_BCST_FRM_ISR_BIT, timestamp);

    DBG_SWDIAG(BCST, FRM_ISR, 1);

    if (ld_bcst_env != NULL)
    {
        // Point to parameters
        struct sch_arb_elt_tag *evt = &(ld_bcst_env->evt);
        struct ld_bcst_env_tag *bcst_par = ld_bcst_env;

        // Remove event
        sch_arb_remove(evt, true);

        // Check for broadcast retransmissions
        if (bcst_par->nbc_att > 1)
        {
            bcst_par->nbc_att--;

            // Handle retransmissions
            ld_bcst_resched();
        }
        // Check if also need to broadcast on standard hopping
        else if ((bcst_par->hop_mode == BCST_AFH_HOP) && (bcst_par->active_hop_types & LD_STD_HOP_USED)
                 && (SCH_ARB_FLAG_ASAP_NO_LIMIT == SCH_ARB_ASAP_STG_TYPE_GET(evt)))
        {
            bcst_par->nbc_att = ((BCST_LMP_EVT_TYPE == bcst_par->evt_type) ? (bcst_par->lmp_nbc) : (bcst_par->acl_nbc));

            bcst_par->hop_mode = BCST_STD_HOP;

            // Handle retransmissions
            ld_bcst_resched();
        }
        else
        {
            if (BCST_LMP_EVT_TYPE == bcst_par->evt_type)
            {
                // Reset priority
                bcst_par->no_asap_prio = rwip_priority[RWIP_PRIO_BCST_ACT_IDX].value;
            }

            // Process packet tx
            ld_bcst_process_pkt_tx();

            // Check next transmission
            ld_bcst_sched();
        }
    }
    else
    {
        ASSERT_ERR(0);
    }

    DBG_SWDIAG(BCST, FRM_ISR, 0);
}

/**
 ****************************************************************************************
 * @brief Handle SkipET interrupt
 ****************************************************************************************
 */
__STATIC void ld_bcst_sket_isr(uint32_t timestamp)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_BCST_PATCH_TYPE, LD_BCST_SKET_ISR_BIT, timestamp);

    DBG_SWDIAG(BCST, SKET_ISR, 1);

    if (ld_bcst_env != NULL)
    {
        // Point to parameters
        struct sch_arb_elt_tag *evt = &(ld_bcst_env->evt);

        // Remove event
        sch_arb_remove(evt, true);

        if (SCH_ARB_FLAG_ASAP_NO_LIMIT == SCH_ARB_ASAP_STG_TYPE_GET(evt))
        {
            // Delay with a random number between 2 and 9 slots
            evt->time.hs += 2 * ((2 + ((co_rand_byte() + ip_finetimtgt_get() + (evt->time.hs >> 1)) & 0x07)) & 0x0E);
            evt->time.hs &= RWIP_MAX_CLOCK_TIME;

            // Reschedule according to the current mode
            ld_bcst_resched();
        }
        else
        {
            struct ld_bcst_env_tag *bcst_par = ld_bcst_env;

            // Increment priority
            bcst_par->no_asap_prio = RWIP_PRIO_ADD_2(bcst_par->no_asap_prio, RWIP_PRIO_INC(RWIP_PRIO_BCST_ACT_IDX));

            // Process packet tx
            ld_bcst_process_pkt_tx();

            // Check next transmission
            ld_bcst_sched();
        }
    }
    else
    {
        ASSERT_ERR(0);
    }

    DBG_SWDIAG(BCST, SKET_ISR, 0);
}

/**
 ****************************************************************************************
 * @brief Handle frame interrupt notification
 ****************************************************************************************
 */
__STATIC void ld_bcst_frm_cbk(uint32_t timestamp, uint32_t dummy, uint8_t irq_type)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LD_BCST_PATCH_TYPE, LD_BCST_FRM_CBK_BIT, timestamp, dummy, irq_type);

    ASSERT_INFO(dummy == EM_BT_CS_ACT_BCST_INDEX, dummy, irq_type);

    switch (irq_type)
    {
    case SCH_FRAME_IRQ_EOF:
    case SCH_FRAME_IRQ_EOF_ABORT_UNDER_PRIO:
    case SCH_FRAME_IRQ_EOF_ABORT_AFTER_PRIO:
    {
        ld_bcst_frm_isr(timestamp);
    }
    break;
    case SCH_FRAME_IRQ_SKIP:
    {
        ld_bcst_sket_isr(timestamp);
    }
    break;
    default:
    {
        ASSERT_INFO_FORCE(0, dummy, irq_type);
    }
    break;
    }
}

__STATIC void ld_bcst_start(struct bt_ch_map *afh_ch_map, uint8_t nbc)
{
    uint32_t clock = ld_read_clock();

    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LD_BCST_PATCH_TYPE, LD_BCST_START_BIT, afh_ch_map, nbc);

    // Allocate broadcast event
    ld_bcst_env = LD_ALLOC_EVT(ld_bcst_env_tag);

    if (ld_bcst_env != NULL)
    {
        // Point to parameters
        struct sch_arb_elt_tag *evt = &(ld_bcst_env->evt);
        struct ld_bcst_env_tag *bcst_par = ld_bcst_env;

        struct bd_addr *bd_addr = &ld_env.local_bd_addr;
        uint8_t *bch = &ld_env.local_bch[0];

        uint8_t cs_idx = EM_BT_CS_ACT_BCST_INDEX;

        LD_INIT_EVT(evt, ld_bcst_env_tag);

        // Set permission/status of CS as R/W, retain initialize status
        DBG_MEM_PERM_SET((const void *)(REG_EM_BT_CS_BASE_ADDR + REG_EM_BT_CS_ADDR_GET(cs_idx)), REG_EM_BT_CS_SIZE, true, true, false);

        // Initialize event parameters (common part)
        evt->cb_cancel       = &ld_bcst_evt_canceled_cbk;
        evt->cb_start        = &ld_bcst_evt_start_cbk;
        evt->cb_stop         = NULL; // no need stop cbk as activity cannot be stopped by HW
        evt->current_prio       = rwip_priority[RWIP_PRIO_BCST_DFT_IDX].value;
        evt->duration_min       = 2 * HALF_SLOT_SIZE;

        co_list_init(&bcst_par->queue_acl_tx);

        // Get the hopping type(s) to use for Broadcast
        bcst_par->active_hop_types = ld_acl_active_hop_types_get();

        bcst_par->no_asap_prio = rwip_priority[RWIP_PRIO_BCST_ACT_IDX].value;

        // Prepare CS
        em_bt_txrxcntl_txeir_setf(cs_idx, 0);
        em_bt_txrxcntl_rxthr_setf(cs_idx, 0);
        em_bt_txrxcntl_stopmod_setf(cs_idx, 0);
        em_bt_pwrcntl_pack(cs_idx, /*fhen*/ 1, /*freq*/ 0, /*txpwr*/ rwip_rf.txpwr_max);
        em_bt_wincntl_pack(cs_idx, 0, NORMAL_WIN_SIZE);
        em_bt_clkoff0_setf(cs_idx, 0);
        em_bt_clkoff1_setf(cs_idx, 0);
        em_bt_linkcntl_aclltaddr_setf(cs_idx, 0);
        em_bt_aclrxstat_pack(cs_idx,
                             /*rxbufffull*/ 0,
                             /*rxbfmicerr*/ 0,
                             /*rxid*/ 0,
                             /*rxacl*/ 0,
                             /*rxaudio*/ 0,
                             /*rxlmp*/ 0,
#if (EAVESDROPPING_SUPPORT)
                             /*lastrxa2dp*/ 0,
#endif // EAVESDROPPING_SUPPORT
                             /*lastrxflow*/ 1,
                             /*lastrxack*/ 0,
                             /*lastrxseqn*/ 0);
        em_bt_txdescptr_setf(cs_idx, REG_EM_ADDR_GET(BT_TXDESC, EM_BT_TXDESC_ACT_BCST_INDEX));
        em_bt_frcntl_pack(cs_idx, 1 /*fpoll*/, 0 /*fmwspatt*/, RWIP_COEX_GET(BCAST, TXBSY), RWIP_COEX_GET(BCAST, RXBSY), RWIP_COEX_GET(BCAST, DNABORT), 0 /*samen*/, 1 /*lsam_dsb*/, EM_BT_CS_FMT_MST_CONNECT);

        em_bt_bdaddr_setf(cs_idx, 0, (bd_addr->addr[1] << 8) | bd_addr->addr[0]);
        em_bt_bdaddr_setf(cs_idx, 1, (bd_addr->addr[3] << 8) | bd_addr->addr[2]);
        em_bt_bdaddr_setf(cs_idx, 2, (bd_addr->addr[5] << 8) | bd_addr->addr[4]);
        em_bt_bch0_setf(cs_idx, (bch[1] << 8) | bch[0]);
        em_bt_bch1_setf(cs_idx, (bch[3] << 8) | bch[2]);
        em_bt_rxmaxbuf_bch2_pack(cs_idx, 0, 0, bch[4] & EM_BT_BCH2_MASK);
        em_bt_linkcntl_whdsb_setf(cs_idx, 0);

        // Write channel map to CS
        em_bt_chmap0_setf(cs_idx, co_read16p(&afh_ch_map->map[0]));
        em_bt_chmap1_setf(cs_idx, co_read16p(&afh_ch_map->map[2]));
        em_bt_chmap2_setf(cs_idx, co_read16p(&afh_ch_map->map[4]));
        em_bt_chmap3_setf(cs_idx, co_read16p(&afh_ch_map->map[6]));
        em_bt_chmap4_setf(cs_idx, co_read16p(&afh_ch_map->map[8]) & EM_BT_CHMAP4_MASK);

        // Prepare Tx descriptor
        em_bt_txheader_pack(EM_BT_TXDESC_ACT_BCST_INDEX, /*txdv*/ 0, /*txrsvd*/ 0, /*txseqn*/ 0, /*txarqn*/ 0, /*txflow*/ 1, /*txtype*/ DM1_TYPE, /*txltaddr*/ 0);
        em_bt_txpheader_txlength_setf(EM_BT_TXDESC_ACT_BCST_INDEX, 0);
        em_bt_txaclbufptr_setf(EM_BT_TXDESC_ACT_BCST_INDEX, 0);
        em_bt_txlmbufptr_setf(EM_BT_TXDESC_ACT_BCST_INDEX, 0);
        em_bt_txptr_pack(EM_BT_TXDESC_ACT_BCST_INDEX, 0, 0);
        em_bt_txptr_nextptr_setf(EM_BT_TXDESC_ACT_BCST_INDEX, REG_EM_ADDR_GET(BT_TXDESC, EM_BT_TXDESC_ACT_BCST_INDEX));

        rwip_rf.txpwr_cs_set(TYPE_BT, cs_idx, rwip_rf.txpwr_max_mod, rwip_rf.txpwr_max);

        // Try to schedule broadcast ACL/LMP
        evt->time.hs = CO_ALIGN4_HI(clock);
        SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_ASAP_NO_LIMIT, SCH_ARB_PHASE_0, 0, 0);
    }
    else
    {
        ASSERT_ERR(0);
    }
}


/**
 ****************************************************************************************
 * @brief Initialize exchange memory
 ****************************************************************************************
 */
__STATIC void ld_bcst_em_init(void)
{
    uint8_t cs_idx = EM_BT_CS_ACT_BCST_INDEX;

    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LD_BCST_PATCH_TYPE, LD_BCST_EM_INIT_BIT);

    // Set permission/status of CS as R/W but uninitialized
    DBG_MEM_PERM_SET((const void *)(REG_EM_BT_CS_BASE_ADDR + REG_EM_BT_CS_ADDR_GET(cs_idx)), REG_EM_BT_CS_SIZE, true, true, true);

    // Init acl tx status - let persist ld_bcst_start for correct tx seqn control
    em_bt_acltxstat_pack(cs_idx, 0, 0, 0, 0, 0, 0, 0, 0, 0);

    // Default clear other control
    em_bt_txrxcntl_set(cs_idx, 0);
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
                        /*linklbl*/ cs_idx);
    em_bt_loopcntl_set(cs_idx, 0);
    em_bt_maxfrmtime_set(cs_idx, 0);

    for (uint16_t i = 0 ; i < (sizeof(struct initialization_vector) / sizeof(uint16_t)) ; i++)
    {
        em_bt_iv_setf(cs_idx, i, 0);
    }
    em_bt_txccmpldcnt0_setf(cs_idx, 0);
    em_bt_txccmpldcnt1_setf(cs_idx, 0);
    em_bt_txccmpldcnt2_setf(cs_idx, 0);
    em_bt_rxccmpldcnt0_setf(cs_idx, 0);
    em_bt_rxccmpldcnt1_setf(cs_idx, 0);
    em_bt_rxccmpldcnt2_setf(cs_idx, 0);
#if (EAVESDROPPING_SUPPORT)
    em_bt_cidcntl_set(cs_idx, 0);
    em_bt_rxcrc_set(cs_idx, 0);
#endif // EAVESDROPPING_SUPPORT

    // Remove permission/status of CS until used
    DBG_MEM_PERM_SET((const void *)(REG_EM_BT_CS_BASE_ADDR + REG_EM_BT_CS_ADDR_GET(cs_idx)), REG_EM_BT_CS_SIZE, false, false, false);
}

/*
 * EXPORTED FUNCTIONS DEFINITION
 *****************************************************************************************
 */

void ld_bcst_acl_init(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LD_BCST_PATCH_TYPE, LD_BCST_ACL_INIT_BIT);

    ld_bcst_env = NULL;

    // Initialize EM
    ld_bcst_em_init();
}

void ld_bcst_acl_reset(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LD_BCST_PATCH_TYPE, LD_BCST_ACL_RESET_BIT);

    if (ld_bcst_env != NULL)
    {
        // Delete event
        ke_free(ld_bcst_env);
        ld_bcst_env = NULL;
    }

    // Initialize EM
    ld_bcst_em_init();
}

uint8_t ld_bcst_lmp_tx(struct bt_em_lmp_buf_elt *buf_elt, struct bt_ch_map *afh_ch_map, uint32_t clock, uint8_t nbc)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    FUNC_PATCH_ENTRY_4_PARAM_HAVE_RETURN(LD_BCST_PATCH_TYPE, LD_BCST_LMP_TX_BIT, uint8_t, buf_elt, afh_ch_map, clock, nbc);

    GLOBAL_INT_DISABLE();

    DBG_SWDIAG(BCST, LMP_TX, 1);

    struct ld_bcst_env_tag *bcst_par;

    // Start broadcast event and insert if first use scenario
    if (ld_bcst_env == NULL)
    {
        ld_bcst_start(afh_ch_map, nbc);
    }

    bcst_par = ld_bcst_env;

    if (NULL == bcst_par->curr_lmp_buf_elt)
    {
        bcst_par->curr_lmp_buf_elt = buf_elt;
        bcst_par->lmp_nbc = nbc;
        bcst_par->lmp_clk = clock;

        // Check if the new packet can be prepared
        if (NULL == bcst_par->curr_acl_buf_elt)
        {
            ld_bcst_sched();
        }

        status = CO_ERROR_NO_ERROR;
    }

    DBG_SWDIAG(BCST, LMP_TX, 0);

    GLOBAL_INT_RESTORE();

    return (status);
}

uint8_t ld_bcst_acl_data_tx(uint8_t link_id, struct bt_em_acl_buf_elt *buf_elt, struct bt_ch_map *afh_ch_map, uint8_t nbc)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    uint16_t data_len = GETF(buf_elt->data_len_flags, BT_EM_ACL_DATA_LEN);

    FUNC_PATCH_ENTRY_4_PARAM_HAVE_RETURN(LD_BCST_PATCH_TYPE, LD_BCST_ACL_DATA_TX_BIT, uint8_t, link_id, buf_elt, afh_ch_map, nbc);

    GLOBAL_INT_DISABLE();

    DBG_SWDIAG(BCST, ACL_TX, 1);

    // Check Broadcast flag & data Length
    if ((BCF_ACTIVE_SLV_BCST == GETF(buf_elt->data_len_flags, BT_EM_ACL_BF)) && (data_len > 0))
    {
        struct sch_arb_elt_tag *evt;
        struct ld_bcst_env_tag *bcst_par;

        // Start broadcast event and insert if first use scenario
        if (ld_bcst_env == NULL)
        {
            ld_bcst_start(afh_ch_map, nbc);
        }

        evt = &(ld_bcst_env->evt);
        bcst_par =  ld_bcst_env;

        // Update Nbc - even if curr_acl_buf_elt active
        bcst_par->acl_nbc = nbc;

        // Check if a new packet can be prepared
        if ((NULL == bcst_par->curr_acl_buf_elt) && (NULL == bcst_par->curr_lmp_buf_elt))
        {
            uint8_t new_txllid = LLID_CONTINUE;
            uint8_t txdesc_idx = EM_BT_TXDESC_ACT_BCST_INDEX;

            // Find packet length
            uint16_t new_txlength = co_min(data_len, DM1_PACKET_SIZE);

            // Check packet boundary flag
            if (GETF(buf_elt->data_len_flags, BT_EM_ACL_PBF) != PBF_CONT_HL_FRAG)
            {
                new_txllid = LLID_START;
            }

            bcst_par->curr_acl_buf_elt = buf_elt;

            // Set data length and pointer
            bcst_par->data_len = data_len;
            bcst_par->data_ptr = buf_elt->buf_ptr;

            // Set hop mode
            bcst_par->hop_mode = (bcst_par->active_hop_types & LD_AFH_HOP_USED) ? BCST_AFH_HOP : BCST_STD_HOP;

            // Fill descriptor
            em_bt_txpheader_pack(txdesc_idx, 0, new_txlength, 0, new_txllid);
            em_bt_txaclbufptr_setf(txdesc_idx, bcst_par->data_ptr);
            // Disable waitack for SEQN toggle
            em_bt_acltxstat_waitack_setf(EM_BT_CS_ACT_BCST_INDEX, 0);

            // Release descriptor
            em_bt_txptr_txdone_setf(txdesc_idx, 0);

            // Update remaining data length
            bcst_par->data_len -= new_txlength;
            // Update data pointer
            bcst_par->data_ptr += new_txlength;

            bcst_par->nbc_att = bcst_par->acl_nbc;

            if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
            {
                bcst_par->state = BCST_EVT_WAIT;
                bcst_par->evt_type = BCST_ACL_EVT_TYPE;
                status = CO_ERROR_NO_ERROR;
            }
            else
            {
                ASSERT_ERR(0);
            }
        }
        else
        {
            // Push at the end of bcst ACL TX queue
            co_list_push_back(&bcst_par->queue_acl_tx, &buf_elt->hdr);
            status = CO_ERROR_NO_ERROR;
        }
    }
    else
    {
        ASSERT_ERR(0);
    }

    DBG_SWDIAG(BCST, ACL_TX, 0);

    GLOBAL_INT_RESTORE();

    return (status);
}

void ld_bcst_lmp_cancel(void)
{

    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LD_BCST_PATCH_TYPE, LD_BCST_LMP_CANCEL_BIT);

    GLOBAL_INT_DISABLE();

    // Point to parameters
    struct ld_bcst_env_tag *bcst_par = ld_bcst_env;

    if ((bcst_par != NULL) && (bcst_par->curr_lmp_buf_elt))
    {
        // If lmp_buf_elt is not yet scheduled
        if (BCST_LMP_EVT_TYPE != bcst_par->evt_type)
        {
            // Free the buffer
            bt_util_buf_lmp_tx_free(bcst_par->curr_lmp_buf_elt->buf_ptr);
            bcst_par->curr_lmp_buf_elt = NULL;
        }
        else if (BCST_EVT_WAIT == bcst_par->state)
        {
            // Remove event
            sch_arb_remove(&(bcst_par->evt), false);

            // Free the buffer
            bt_util_buf_lmp_tx_free(bcst_par->curr_lmp_buf_elt->buf_ptr);
            bcst_par->curr_lmp_buf_elt = NULL;

            // Schedule other broadcast
            ld_bcst_sched();
        }
        else
        {
            // LMP broadcast active, block potential for further retransmissions
            bcst_par->hop_mode = BCST_STD_HOP;
            bcst_par->nbc_att = 1;
        }
    }

    GLOBAL_INT_RESTORE();
}

/**
 ****************************************************************************************
 * @brief Updates AFH channel map for active broadcast TX.
 ****************************************************************************************
 */
uint8_t ld_bcst_afh_update(struct bt_ch_map *afh_ch_map)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LD_BCST_PATCH_TYPE, LD_BCST_AFH_UPDATE_BIT, uint8_t, afh_ch_map);

    if (ld_bcst_env != NULL)
    {
        uint8_t cs_idx = EM_BT_CS_ACT_BCST_INDEX;

        // Write channel map to CS
        em_bt_chmap0_setf(cs_idx, co_read16p(&afh_ch_map->map[0]));
        em_bt_chmap1_setf(cs_idx, co_read16p(&afh_ch_map->map[2]));
        em_bt_chmap2_setf(cs_idx, co_read16p(&afh_ch_map->map[4]));
        em_bt_chmap3_setf(cs_idx, co_read16p(&afh_ch_map->map[6]));
        em_bt_chmap4_setf(cs_idx, co_read16p(&afh_ch_map->map[8]) & EM_BT_CHMAP4_MASK);

        status = CO_ERROR_NO_ERROR;
    }

    return (status);
}

#if BCAST_ENC_SUPPORT
void ld_bcst_tx_enc(uint8_t mode)
{
    uint8_t cs_idx = EM_BT_CS_ACT_BCST_INDEX;

    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_BCST_PATCH_TYPE, LD_BCST_TX_ENC_BIT, mode);

    // Set permission/status of CS as R/W, retain initialize status
    DBG_MEM_PERM_SET((const void *)(REG_EM_BT_CS_BASE_ADDR + REG_EM_BT_CS_ADDR_GET(cs_idx)), REG_EM_BT_CS_SIZE, true, true, false);

    // Set encryption mode for TX packets
    em_bt_txrxcntl_txcrypt_setf(cs_idx, mode);
    em_bt_txrxcntl_txbcry_setf(cs_idx, mode);

    // Remove permission/status of CS until used
    DBG_MEM_PERM_SET((const void *)(REG_EM_BT_CS_BASE_ADDR + REG_EM_BT_CS_ADDR_GET(cs_idx)), REG_EM_BT_CS_SIZE, false, false, false);
}

void ld_bcst_enc_key_load(struct ltk *key)
{
    uint8_t cs_idx = EM_BT_CS_ACT_BCST_INDEX;

    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_BCST_PATCH_TYPE, LD_BCST_ENC_KEY_LOAD_BIT, key);

    // Set permission/status of CS as R/W, retain initialize status
    DBG_MEM_PERM_SET((const void *)(REG_EM_BT_CS_BASE_ADDR + REG_EM_BT_CS_ADDR_GET(cs_idx)), REG_EM_BT_CS_SIZE, true, true, false);

    // Set the SK
    for (uint16_t i = 0 ; i < (sizeof(struct ltk) / sizeof(uint16_t)) ; i++)
    {
        em_bt_sk_setf(cs_idx, i, co_read16p(&key->ltk[i * 2]));
    }

    // Remove permission/status of CS until used
    DBG_MEM_PERM_SET((const void *)(REG_EM_BT_CS_BASE_ADDR + REG_EM_BT_CS_ADDR_GET(cs_idx)), REG_EM_BT_CS_SIZE, false, false, false);
}
#endif // BCAST_ENC_SUPPORT

///@} LDBCST
