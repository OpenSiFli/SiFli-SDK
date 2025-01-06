/**
****************************************************************************************
*
* @file ld_csb_tx.c
*
* @brief LD BCST source code
*
* Copyright (C) RivieraWaves 2015
*
****************************************************************************************
*/

/**
 ****************************************************************************************
 * @addtogroup LDCSBTX
 * @ingroup LD
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"    // stack configuration

#if CSB_SUPPORT

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

#include "reg_btcore.h"         // BT core registers
#include "reg_btcore_esco.h"    // eSCO registers
#include "reg_btcore_audio.h"   // Audio registers
#include "reg_em_bt_cs.h"       // BT EM Control Structure
#include "reg_em_bt_rxdesc.h"   // BT EM RX descriptors
#include "reg_em_bt_txdesc.h"   // BT EM TX descriptors

/*
 * DEFINES
 *****************************************************************************************
 */

/*
 * ENUMERATIONS DEFINITIONS
 ****************************************************************************************
 */

/// ACL event states
enum CSB_TX_EVT_STATE
{
    CSB_TX_EVT_NONE = 0,
    CSB_TX_EVT_WAIT,
    CSB_TX_EVT_ACTIVE,
    CSB_TX_EVT_END,
};


/*
 * STRUCTURE DEFINITION
 *****************************************************************************************
 */

/// Data set for CSB - stored regardless of CSB enable/disable
struct bt_em_acl_buf_elt *ld_csb_tx_buf_elt;

/// LD broadcast ACL environment structure
struct ld_csb_tx_env_tag
{
    /// Broadcast ACL event
    struct sch_arb_elt_tag evt;

    /// active Tx type
    uint8_t tx_type;
    /// length of CSB data
    uint16_t data_len;
    /// rserved lt_addr for CSB
    uint8_t lt_addr;
    /// packet types supported
    uint16_t packet_types;
    /// CSB interval (in half-slots)
    uint16_t csb_interval;
    /// CSB supervision timeout
    uint16_t csb_supv_to;
    /// latest successful tx timestamp (in BT half-slots)
    uint32_t latest_tx;

    /// flag to disable CSB tx
    bool disable_pending;
    /// flag for pending new data for CSB
    struct bt_em_acl_buf_elt *buf_elt_pending;
    /// State
    uint8_t state;
};

/*
 * VARIABLES DEFINITIONS
 *****************************************************************************************
 */

/// LD connection-less slave broadcast ACL environment variable
__STATIC struct ld_csb_tx_env_tag *ld_csb_tx_env;

/*
 * LOCAL FUNCTIONS DECLARATION
 *****************************************************************************************
 */
__STATIC void ld_csb_tx_frm_cbk(uint32_t timestamp, uint32_t dummy, uint8_t irq_type);

/*
 * LOCAL FUNCTIONS DEFINITION
 *****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Cleanup csb_tx environment
 ****************************************************************************************
 */
__STATIC void ld_csb_tx_cleanup(void)
{
    DBG_SWDIAG(CSB_TX, END, 1);

    // Remove permission/status of CS as now unused
    DBG_MEM_PERM_SET((const void *)(REG_EM_BT_CS_BASE_ADDR + REG_EM_BT_CS_ADDR_GET(EM_BT_CS_CSB_TX_INDEX)), REG_EM_BT_CS_SIZE, false, false, false);

    // Free event memory
    ke_free(ld_csb_tx_env);
    ld_csb_tx_env = NULL;

    DBG_SWDIAG(CSB_TX, END, 0);
}

/**
 ****************************************************************************************
 * @brief Select a packet type for CSB broadcast TX data
 ****************************************************************************************
 */
__STATIC uint8_t ld_csb_tx_packet_type_select(uint16_t packet_types, uint16_t tx_data_len)
{
    uint8_t select = DM1_IDX;
    uint8_t idx = 0;
    uint8_t tx_type;

    packet_types ^= (PACKET_TYPE_NO_2_DH1_FLAG |
                     PACKET_TYPE_NO_3_DH1_FLAG |
                     PACKET_TYPE_NO_2_DH3_FLAG |
                     PACKET_TYPE_NO_3_DH3_FLAG |
                     PACKET_TYPE_NO_2_DH5_FLAG |
                     PACKET_TYPE_NO_3_DH5_FLAG);

    if (packet_types & (PACKET_TYPE_NO_2_DH1_FLAG |
                        PACKET_TYPE_NO_3_DH1_FLAG |
                        PACKET_TYPE_NO_2_DH3_FLAG |
                        PACKET_TYPE_NO_3_DH3_FLAG |
                        PACKET_TYPE_NO_2_DH5_FLAG |
                        PACKET_TYPE_NO_3_DH5_FLAG))
    {
        const uint16_t edr_packet_types_flag[] = {PACKET_TYPE_DM1_FLAG, PACKET_TYPE_NO_2_DH1_FLAG, PACKET_TYPE_NO_3_DH1_FLAG,
                                                  PACKET_TYPE_NO_2_DH3_FLAG, PACKET_TYPE_NO_3_DH3_FLAG, PACKET_TYPE_NO_2_DH5_FLAG, PACKET_TYPE_NO_3_DH5_FLAG
                                                 };

        // Find the smallest allowed EDR packet type that is large enough for the data to send
        for (idx = 0; idx < ARRAY_LEN(ld_acl_edr_sizes); idx++)
        {
            // Check if packet type is enabled
            if (packet_types & edr_packet_types_flag[idx])
            {
                // Replace the selected index by the new one
                select = idx;

                // Check if the packet size is enough for the data to send
                if (ld_acl_edr_sizes[idx] >= tx_data_len)
                {
                    break;
                }
            }
        }

        // Get packet type
        tx_type = ld_acl_edr_types[select];
    }
    else
    {
        const uint16_t br_packet_types_flag[] = {PACKET_TYPE_DM1_FLAG, PACKET_TYPE_DH1_FLAG, PACKET_TYPE_DM3_FLAG, PACKET_TYPE_DH3_FLAG,
                                                 PACKET_TYPE_DM5_FLAG, PACKET_TYPE_DH5_FLAG
                                                };

        // Find the smallest allowed BR packet type that is large enough for the data to send
        for (idx = 0; idx < ARRAY_LEN(ld_acl_br_sizes); idx++)
        {
            // Check if packet type is enabled
            if (packet_types & br_packet_types_flag[idx])
            {
                // Replace the selected index by the new one
                select = idx;

                // Check if the packet size is enough for the data to send
                if (ld_acl_br_sizes[idx] >= tx_data_len)
                {
                    break;
                }
            }
        }

        // Get packet type
        tx_type = ld_acl_br_types[select];
    }

    return tx_type;
}

/**
 ****************************************************************************************
 * @brief Schedule next CSB ACL activity
 ****************************************************************************************
 */
__STATIC void ld_csb_tx_sched()
{
    struct sch_arb_elt_tag *evt = &(ld_csb_tx_env->evt);
    struct ld_csb_tx_env_tag *csb_tx_par = ld_csb_tx_env;

    DBG_SWDIAG(CSB_TX, SCHED, 1);

    // Check for new pending CSB data to replace existing CSB data
    if (csb_tx_par->buf_elt_pending)
    {
        if (ld_csb_tx_buf_elt)
        {
            bt_util_buf_acl_tx_free(ld_csb_tx_buf_elt->buf_ptr);
        }

        ld_csb_tx_buf_elt = csb_tx_par->buf_elt_pending;

        csb_tx_par->buf_elt_pending = NULL;
    }

    if (!csb_tx_par->disable_pending)
    {
        // Attempt to insert next event, iteratively, unless supervision timeout from latest_tx
        while (((evt->time.hs - csb_tx_par->latest_tx) & RWIP_MAX_CLOCK_TIME) <= (2 * csb_tx_par->csb_supv_to))
        {
            // Progress to next CSB instant
            evt->time.hs += csb_tx_par->csb_interval;

            // Try scheduling CSB ACL
            if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
            {
                csb_tx_par->state = CSB_TX_EVT_WAIT;
                break;
            }
            else
            {
                // Increment priority
                evt->current_prio = RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(RWIP_PRIO_CSB_TX_DFT_IDX));
            }
        }

        // If event not inserted within supervision timeout, report supervisionTO, delete event
        if (CSB_TX_EVT_WAIT != csb_tx_par->state)
        {
            // Report CSB supervisionTO
            struct lb_csb_tx_end_ind *msg = KE_MSG_ALLOC(LB_CSB_TX_END_IND, TASK_LB, TASK_NONE, lb_csb_tx_end_ind);
            msg->bd_addr = ld_env.local_bd_addr;
            msg->lt_addr = csb_tx_par->lt_addr;
            ke_msg_send(msg);

            // Delete event
            ld_csb_tx_cleanup();
        }
    }
    else
    {
        // Delete event
        ld_csb_tx_cleanup();
    }

    DBG_SWDIAG(CSB_TX, SCHED, 0);
}


/**
 ****************************************************************************************
 * @brief Handle event start notification
 ****************************************************************************************
 */
__STATIC void ld_csb_tx_evt_start_cbk(struct sch_arb_elt_tag *evt)
{
    DBG_SWDIAG(CSB_TX, EVT_START, 1);

    if (evt != NULL)
    {
        // Point to parameters
        struct ld_csb_tx_env_tag *csb_tx_par = ld_csb_tx_env;
        uint8_t cs_idx = EM_BT_CS_CSB_TX_INDEX;
        uint8_t txdesc_idx = EM_BT_TXDESC_CSB_INDEX;

        {
            // Push the programming to SCH PROG
            struct sch_prog_params prog_par;
            prog_par.frm_cbk         = &ld_csb_tx_frm_cbk;
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

        // Update CSB data related settings
        if (ld_csb_tx_buf_elt)
        {
            csb_tx_par->data_len = GETF(ld_csb_tx_buf_elt->data_len_flags, BT_EM_ACL_DATA_LEN);
            csb_tx_par->tx_type  = ld_csb_tx_packet_type_select(csb_tx_par->packet_types, csb_tx_par->data_len);

            em_bt_txheader_txtype_setf(txdesc_idx, csb_tx_par->tx_type);
            em_bt_txpheader_txlength_setf(txdesc_idx, csb_tx_par->data_len);
            em_bt_txpheader_txllid_setf(txdesc_idx, LLID_START);
            em_bt_txaclbufptr_setf(txdesc_idx, ld_csb_tx_buf_elt->buf_ptr);

            em_bt_txptr_pack(txdesc_idx, 0, 0);
            em_bt_txdescptr_setf(cs_idx, REG_EM_ADDR_GET(BT_TXDESC, txdesc_idx));
        }

        csb_tx_par->state = CSB_TX_EVT_ACTIVE;
    }
    else
    {
        ASSERT_ERR(0);
    }

    DBG_SWDIAG(CSB_TX, EVT_START, 0);
}

/**
 ****************************************************************************************
 * @brief Handle event canceled notification
 ****************************************************************************************
 */
__STATIC void ld_csb_tx_evt_canceled_cbk(struct sch_arb_elt_tag *evt)
{
    DBG_SWDIAG(CSB_TX, EVT_CANCELED, 1);

    if (evt != NULL)
    {
        // Increment priority
        evt->current_prio = RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(RWIP_PRIO_CSB_TX_DFT_IDX));

        // Schedule next transmit
        ld_csb_tx_sched();
    }
    else
    {
        ASSERT_ERR(0);
    }

    DBG_SWDIAG(CSB_TX, EVT_CANCELED, 0);
}


/**
 ****************************************************************************************
 * @brief Handle frame interrupt
 ****************************************************************************************
 */
__STATIC void ld_csb_tx_frm_isr(uint32_t timestamp)
{
    DBG_SWDIAG(CSB_TX, FRM_ISR, 1);

    if (ld_csb_tx_env != NULL)
    {
        struct sch_arb_elt_tag *evt = &(ld_csb_tx_env->evt);
        struct ld_csb_tx_env_tag *csb_tx_par = ld_csb_tx_env;

        // Store latest_tx time, and Restore default event priority
        evt->current_prio = rwip_priority[RWIP_PRIO_CSB_TX_DFT_IDX].value;
        csb_tx_par->latest_tx = evt->time.hs;

        // Remove event
        sch_arb_remove(evt, true);

        // Schedule next transmit
        ld_csb_tx_sched();
    }
    else
    {
        ASSERT_ERR(0);
    }

    DBG_SWDIAG(CSB_TX, FRM_ISR, 0);
}

/**
 ****************************************************************************************
 * @brief Handle SkipET interrupt
 ****************************************************************************************
 */
__STATIC void ld_csb_tx_sket_isr(uint32_t timestamp)
{
    DBG_SWDIAG(CSB_TX, EVT_CANCELED, 1);

    if (ld_csb_tx_env != NULL)
    {
        // Remove event
        sch_arb_remove(&(ld_csb_tx_env->evt), true);

        // Increment priority
        ld_csb_tx_env->evt.current_prio = RWIP_PRIO_ADD_2(ld_csb_tx_env->evt.current_prio, RWIP_PRIO_INC(RWIP_PRIO_CSB_TX_DFT_IDX));

        // Schedule next transmit
        ld_csb_tx_sched();
    }
    else
    {
        ASSERT_ERR(0);
    }

    DBG_SWDIAG(CSB_TX, EVT_CANCELED, 0);
}

/**
 ****************************************************************************************
 * @brief Handle frame interrupt notification
 ****************************************************************************************
 */
__STATIC void ld_csb_tx_frm_cbk(uint32_t timestamp, uint32_t dummy, uint8_t irq_type)
{
    ASSERT_INFO(dummy == EM_BT_CS_CSB_TX_INDEX, dummy, irq_type);

    switch (irq_type)
    {
    case SCH_FRAME_IRQ_EOF:
    case SCH_FRAME_IRQ_EOF_ABORT_UNDER_PRIO:
    case SCH_FRAME_IRQ_EOF_ABORT_AFTER_PRIO:
    {
        ld_csb_tx_frm_isr(timestamp);
    }
    break;
    case SCH_FRAME_IRQ_SKIP:
    {
        ld_csb_tx_sket_isr(timestamp);
    }
    break;
    default:
    {
        ASSERT_INFO(0, dummy, irq_type);
    }
    break;
    }
}

/*
 * EXPORTED FUNCTIONS DEFINITION
 *****************************************************************************************
 */

void ld_csb_tx_init(void)
{
    ld_csb_tx_env = NULL;
    ld_csb_tx_buf_elt = 0;
}

void ld_csb_tx_reset(void)
{
    if (ld_csb_tx_env != NULL)
    {
        // Point to parameters
        struct ld_csb_tx_env_tag *csb_tx_par = ld_csb_tx_env;

        if (ld_csb_tx_buf_elt)
        {
            // Free the buffer
            bt_util_buf_acl_tx_free(ld_csb_tx_buf_elt->buf_ptr);
            ld_csb_tx_buf_elt = NULL;
        }

        if (csb_tx_par->buf_elt_pending)
        {
            // Free the buffer
            bt_util_buf_acl_tx_free(csb_tx_par->buf_elt_pending->buf_ptr);
        }

        // Free event memory
        ke_free(ld_csb_tx_env);
        ld_csb_tx_env = NULL;
    }
}

/**
 ****************************************************************************************
 * @brief Enables connectionless slave broadcast.
 * param[in] lt_addr;
 * param[in] packet types supported
 * param[in] CSB interval
 * param[in] CSB supervision timeout
 *
****************************************************************************************
 */
uint8_t ld_csb_tx_en(struct ld_csb_en_params *params)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    // Check if CSB is inactive
    if (ld_csb_tx_env == NULL)
    {
        // Allocate event
        ld_csb_tx_env = LD_ALLOC_EVT(ld_csb_tx_env_tag);

        if (ld_csb_tx_env != NULL)
        {
            uint32_t clock = ld_read_clock();

            uint8_t cs_idx = EM_BT_CS_CSB_TX_INDEX;
            uint8_t txdesc_idx = EM_BT_TXDESC_CSB_INDEX;
            struct bd_addr *bd_addr = &ld_env.local_bd_addr;
            uint8_t *bch = &ld_env.local_bch[0];
            uint16_t csb_intv = params->csb_interval;
            bool edr = ((params->packet_types ^ PACKET_TYPE_EDR_MSK) & PACKET_TYPE_EDR_MSK & ~PACKET_TYPE_DM1_FLAG);

            // Point to parameters
            struct sch_arb_elt_tag *evt = &(ld_csb_tx_env->evt);
            struct ld_csb_tx_env_tag *csb_tx_par = ld_csb_tx_env;

            LD_INIT_EVT(evt, ld_csb_tx_env_tag);

            // Set permission/status of CS as R/W but uninitialized
            DBG_MEM_PERM_SET((const void *)(REG_EM_BT_CS_BASE_ADDR + REG_EM_BT_CS_ADDR_GET(cs_idx)), REG_EM_BT_CS_SIZE, true, true, true);

            // Initialize event parameters (common part)
            evt->cb_cancel       = &ld_csb_tx_evt_canceled_cbk;
            evt->cb_start        = &ld_csb_tx_evt_start_cbk;
            evt->cb_stop         = NULL;
            evt->current_prio       = rwip_priority[RWIP_PRIO_CSB_TX_DFT_IDX].value;
            evt->duration_min       = 2 * (5 * params->max_slots * SLOT_SIZE - LD_ACL_INTERFRAME_MARGIN);
            SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_NO_ASAP, 0, 0, 0);

            // Initialize event parameters (CSB part)
            csb_tx_par->lt_addr = params->lt_addr;
            csb_tx_par->packet_types = params->packet_types;
            csb_tx_par->csb_interval = params->csb_interval;
            csb_tx_par->csb_supv_to = params->csb_supv_to;

            // Find the latest CSB instant, will be incremented csb_interval prior to next insert
            evt->time.hs = CLK_ADD_2(((clock / csb_intv) * csb_intv), params->csb_offset);
            csb_tx_par->latest_tx = evt->time.hs;

            // Prepare Tx descriptor
            em_bt_txheader_pack(txdesc_idx, 0, 0, 0, 0, 0, ID_NUL_TYPE, params->lt_addr);
            em_bt_txpheader_pack(txdesc_idx, 0, 0, 0, LLID_START);
            em_bt_txaclbufptr_setf(txdesc_idx, 0); // later set to buf_elt->buf_ptr
            em_bt_txlmbufptr_setf(txdesc_idx, 0);
            em_bt_txptr_pack(txdesc_idx, 0, 0);

            // Set CS fields
            em_bt_frcntl_pack(cs_idx, 1 /*fpoll*/, 0 /*fmwspatt*/, RWIP_COEX_GET(BCAST, TXBSY), RWIP_COEX_GET(BCAST, RXBSY), RWIP_COEX_GET(BCAST, DNABORT), 0 /*samen*/, 1 /*lsam_dsb*/, EM_BT_CS_FMT_MST_BCST);
            em_bt_bdaddr_setf(cs_idx, 0, (bd_addr->addr[1] << 8) | bd_addr->addr[0]);
            em_bt_bdaddr_setf(cs_idx, 1, (bd_addr->addr[3] << 8) | bd_addr->addr[2]);
            em_bt_bdaddr_setf(cs_idx, 2, (bd_addr->addr[5] << 8) | bd_addr->addr[4]);
            em_bt_bch0_setf(cs_idx, (bch[1] << 8) | bch[0]);
            em_bt_bch1_setf(cs_idx, (bch[3] << 8) | bch[2]);
            em_bt_rxmaxbuf_bch2_pack(cs_idx, 0, 0, bch[4] & EM_BT_BCH2_MASK);
            em_bt_clkoff0_setf(cs_idx, 0);
            em_bt_clkoff1_setf(cs_idx, 0);
            em_bt_txrxcntl_pack(cs_idx, 0,
#if (EAVESDROPPING_SUPPORT)
                                0,
#endif // EAVESDROPPING_SUPPORT
                                0, 0, 0, 0, 0, 0, 0, 0, 0);
            em_bt_acltxstat_pack(cs_idx, 0, 0, 0, 0, 0, 0, 0, 0, 0);
            em_bt_linkcntl_pack(cs_idx,
                                /*aknena*/ 0,
                                /*afhena*/ 1,
                                /*laap*/ 0,
                                /*whdsb*/ 0,
                                /*acledr*/ edr,
                                /*aclltaddr*/ params->lt_addr,
#if (EAVESDROPPING_SUPPORT)
                                /*aclbinackena*/ 0, /*escobinackena*/ 0, /*duplfltena*/ 0,
#endif // EAVESDROPPING_SUPPORT
                                /*linklbl*/ cs_idx);
            em_bt_pwrcntl_pack(cs_idx, /*fhen*/ 1, /*freq*/ 0, /*txpwr*/ rwip_rf.txpwr_max);
            em_bt_loopcntl_pack(cs_idx, /*tonb*/ 0, /*attnb*/ 1);
            em_bt_txdescptr_setf(cs_idx, 0);

            // Write channel map to CS
            em_bt_chmap0_setf(cs_idx, co_read16p(&params->afh_ch_map->map[0]));
            em_bt_chmap1_setf(cs_idx, co_read16p(&params->afh_ch_map->map[2]));
            em_bt_chmap2_setf(cs_idx, co_read16p(&params->afh_ch_map->map[4]));
            em_bt_chmap3_setf(cs_idx, co_read16p(&params->afh_ch_map->map[6]));
            em_bt_chmap4_setf(cs_idx, co_read16p(&params->afh_ch_map->map[8]) & EM_BT_CHMAP4_MASK);

            // Default clear other control
            em_bt_wincntl_set(cs_idx, 0);
            em_bt_maxfrmtime_set(cs_idx, 0);
#if (EAVESDROPPING_SUPPORT)
            em_bt_cidcntl_set(cs_idx, 0);
            em_bt_rxcrc_set(cs_idx, 0);
#endif // EAVESDROPPING_SUPPORT

            rwip_rf.txpwr_cs_set(TYPE_BT, cs_idx, rwip_rf.txpwr_max_mod, rwip_rf.txpwr_max);

            GLOBAL_INT_DISABLE();

            ld_csb_tx_sched();

            GLOBAL_INT_RESTORE();

            status = CO_ERROR_NO_ERROR;
        }
        else
        {
            status = CO_ERROR_MEMORY_CAPA_EXCEED;
            ASSERT_ERR(0);
        }
    }

    return (status);
}

/**
 ****************************************************************************************
 * @brief Disables connectionless slave broadcast.
 ****************************************************************************************
 */
void ld_csb_tx_dis(void)
{
    if (ld_csb_tx_env != NULL)
    {
        // Point to parameters
        struct ld_csb_tx_env_tag *csb_tx_par = ld_csb_tx_env;

        csb_tx_par->disable_pending = 1;
    }
}

/**
 ****************************************************************************************
 * @brief Updates AFH channel map for connectionless slave broadcast TX.
 ****************************************************************************************
 */
uint8_t ld_csb_tx_afh_update(struct bt_ch_map *afh_ch_map)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    DBG_SWDIAG(CSB_TX, AFH_UPDATE, 1);

    if (ld_csb_tx_env != NULL)
    {
        uint8_t cs_idx = EM_BT_CS_CSB_TX_INDEX;

        // Write channel map to CS
        em_bt_chmap0_setf(cs_idx, co_read16p(&afh_ch_map->map[0]));
        em_bt_chmap1_setf(cs_idx, co_read16p(&afh_ch_map->map[2]));
        em_bt_chmap2_setf(cs_idx, co_read16p(&afh_ch_map->map[4]));
        em_bt_chmap3_setf(cs_idx, co_read16p(&afh_ch_map->map[6]));
        em_bt_chmap4_setf(cs_idx, co_read16p(&afh_ch_map->map[8]) & EM_BT_CHMAP4_MASK);

        status = CO_ERROR_NO_ERROR;
    }
    else
    {
        ASSERT_ERR(0);
    }

    DBG_SWDIAG(CSB_TX, AFH_UPDATE, 0);

    return (status);
}

/**
 ****************************************************************************************
 * @brief Sets transmit data for connectionless slave broadcast.
 * @param[in]  buf_elt      Data buffer, length of data buffer
 ****************************************************************************************
 */
void ld_csb_tx_set_data(struct bt_em_acl_buf_elt *buf_elt)
{
    DBG_SWDIAG(CSB_TX, SET_DATA, 1);
    GLOBAL_INT_DISABLE();

    struct bt_em_acl_buf_elt *buf_elt_pending = buf_elt;

    if (ld_csb_tx_env)
    {
        // Point to parameters
        struct ld_csb_tx_env_tag *csb_tx_par = ld_csb_tx_env;

        if (csb_tx_par->buf_elt_pending)
        {
            bt_util_buf_acl_tx_free(csb_tx_par->buf_elt_pending->buf_ptr);
            csb_tx_par->buf_elt_pending = NULL;
        }

        // Event active, so set buf_elt_pending for later processing
        if (csb_tx_par->state == CSB_TX_EVT_ACTIVE)
        {
            csb_tx_par->buf_elt_pending = buf_elt_pending;
            buf_elt_pending = NULL;
        }
    }

    if (buf_elt_pending)
    {
        if (ld_csb_tx_buf_elt)
        {
            bt_util_buf_acl_tx_free(ld_csb_tx_buf_elt->buf_ptr);
        }

        // Event not active, so set buf_elt with immediate effect
        ld_csb_tx_buf_elt = buf_elt;
    }

    GLOBAL_INT_RESTORE();
    DBG_SWDIAG(CSB_TX, SET_DATA, 0);
}

/**
 ****************************************************************************************
 * @brief Clears transmit data for connectionless slave broadcast.
 * @param[in]  buf_elt      Data buffer, length of data buffer
 ****************************************************************************************
 */
void ld_csb_tx_clr_data(void)
{
    DBG_SWDIAG(CSB_TX, CLR_DATA, 1);
    if (ld_csb_tx_env)
    {
        // Should not be called if CSB active
        ASSERT_ERR(0);
    }
    else
    {
        // Free data resource
        if (ld_csb_tx_buf_elt)
        {
            bt_util_buf_acl_tx_free(ld_csb_tx_buf_elt->buf_ptr);
        }

        ld_csb_tx_buf_elt = NULL;
    }
    DBG_SWDIAG(CSB_TX, CLR_DATA, 0);
}

#endif

///@} LDCSBTX
