/**
****************************************************************************************
*
* @file lld_ch_scan.c
*
* @brief LLD Channel Scanning source code
*
* Copyright (C) RivieraWaves 2009-2021
*
****************************************************************************************
*/

/**
 ****************************************************************************************
 * @addtogroup LLDCHSCAN
 * @ingroup LLD
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"         // stack configuration
#if (BLE_CH_SCAN_SUPPORT)

#include "ke_mem.h"
#include "ke_msg.h"              // kernel messages
#include "rwip.h"

#include "lld.h"                 // link driver API
#include "lld_int.h"             // link layer driver internal

#include "sch_arb.h"             // Scheduling Arbiter
#include "sch_prog.h"            // Scheduling Programmer

#include "reg_blecore.h"         // BLE core registers
#include "reg_em_ble_cs.h"       // BLE EM Control Structure
#include "reg_em_ble_tx_desc.h"  // BLE EM TX descriptors
#include "reg_em_ble_rx_desc.h"  // BLE EM RX descriptors

#include "dbg_swdiag.h"
#include "btdm_patch.h"
/*
 * DEFINES
 *****************************************************************************************
 */

/// Channel Scanning event states
enum CH_SCAN_EVT_STATE
{
    CH_SCAN_EVT_WAIT,
    CH_SCAN_EVT_ACTIVE,
    CH_SCAN_EVT_END,
};


/*
 * STRUCTURE DEFINITION
 *****************************************************************************************
 */

/// LLD Channel Scanning environment structure
struct lld_ch_scan_env_tag
{
    /// Pointer to inquiry event
    struct sch_arb_elt_tag evt;
    /// Channel map used for the channel scan
    struct le_ch_map ch_map;
    /// Time to process one channel in us
    uint32_t time_process_ch;
    /// current state of the channel scan
    uint8_t state;
    /// Activity index
    uint8_t act_id;
    /// Scan interval in half-slots (312.5 us)
    uint16_t intv;
    /// Minimum Channel Scanning event in us
    uint32_t scan_duration;
    /// Current Channel Scan event time in us
    uint32_t scan_current_duration;
    /// Channel selection on which channel scan is performed (see: enum ch_scan_ch_sel)
    uint8_t ch_sel;
};


/*
 * VARIABLE DEFINITION
 *****************************************************************************************
 */

/// LLD Channel Scanning environment variable
__STATIC struct lld_ch_scan_env_tag *lld_ch_scan_env;


/*
 * LOCAL FUNCTIONS DECLARATION
 *****************************************************************************************
 */

__STATIC void lld_ch_scan_frm_cbk(uint32_t timestamp, uint32_t dummy, uint8_t irq_type);


/*
 * LOCAL FUNCTIONS DEFINITION
 *****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Cleanup Channel Scanning environment variable
 ****************************************************************************************
 */
__STATIC void lld_ch_scan_cleanup(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LLD_CH_SCAN_PATCH_TYPE, LLD_CH_SCAN_CLEANUP_FUN_BIT);
    if (lld_ch_scan_env != NULL)
    {
        // Free event memory
        ke_free(lld_ch_scan_env);
        lld_ch_scan_env = NULL;
    }
}

__STATIC void lld_ch_scan_set_ch_map(int elt_idx)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_CH_SCAN_PATCH_TYPE, LLD_CH_SCAN_ENABLE_ALL_CH_MAP_FUN_BIT, elt_idx);
    em_ble_chmap0_set(elt_idx, co_read16p(&(lld_ch_scan_env->ch_map.map[0])));
    em_ble_chmap1_set(elt_idx, co_read16p(&(lld_ch_scan_env->ch_map.map[2])));
    em_ble_chmap2_pack(elt_idx, 0, 0, lld_ch_scan_env->ch_map.map[4]);
}

/**
 ****************************************************************************************
 * @brief Handle event start notification
 ****************************************************************************************
 */
__STATIC void lld_ch_scan_evt_start_cbk(struct sch_arb_elt_tag *evt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_CH_SCAN_PATCH_TYPE, LLD_CH_SCAN_EVT_START_CBK_FUN_BIT, evt);
    ASSERT_ERR(&(lld_ch_scan_env->evt) == evt);

    DBG_SWDIAG(LECHSCAN, EVT_START, 1);

    if (evt != NULL)
    {
        // Point to parameters
        struct lld_ch_scan_env_tag *ch_scan_evt = (struct lld_ch_scan_env_tag *) evt;
        struct sch_prog_params prog_par;

        // Set max event duration
        em_ble_maxevtime_set(EM_BLE_CS_ACT_ID_TO_INDEX(ch_scan_evt->act_id), (ch_scan_evt->scan_current_duration + (SLOT_SIZE - 1)) / SLOT_SIZE);

        // Push the programming to SCH PROG
        prog_par.frm_cbk        = &lld_ch_scan_frm_cbk;
        prog_par.time.hs        = evt->time.hs;
        prog_par.time.hus       = 0;
        prog_par.cs_idx         = EM_BLE_CS_ACT_ID_TO_INDEX(ch_scan_evt->act_id);
        prog_par.dummy          = 0;
        prog_par.bandwidth      = ch_scan_evt->scan_current_duration << 1;
        prog_par.prio_1         = evt->current_prio;
        prog_par.prio_2         = 0;
        prog_par.prio_3         = evt->current_prio;
        prog_par.pti_prio       = RW_BLE_PTI_PRIO_AUTO;
        prog_par.add.ble.ae_nps = 0;
        prog_par.add.ble.iso    = 0;
        prog_par.mode           = SCH_PROG_BLE;
        sch_prog_push(&prog_par);

        // Move state
        ch_scan_evt->state = CH_SCAN_EVT_ACTIVE;
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(LECHSCAN, EVT_START, 0);
}

/**
 ****************************************************************************************
 * @brief Handle event canceled notification
 ****************************************************************************************
 */
__STATIC void lld_ch_scan_evt_canceled_cbk(struct sch_arb_elt_tag *evt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_CH_SCAN_PATCH_TYPE, LLD_CH_SCAN_EVT_CANCELED_CBK_FUN_BIT, evt);
    ASSERT_ERR(&(lld_ch_scan_env->evt) == evt);

    DBG_SWDIAG(LECHSCAN, EVT_CANCELED, 1);

    if (evt != NULL)
    {
        ASSERT_ERR(((struct lld_ch_scan_env_tag *) evt)->state == CH_SCAN_EVT_WAIT);

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

    DBG_SWDIAG(LECHSCAN, EVT_CANCELED, 0);
}

/**
 ****************************************************************************************
 * @brief Handle Rx interrupt
 ****************************************************************************************
 */
__STATIC void lld_ch_scan_rx_isr(uint32_t timestamp)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_CH_SCAN_PATCH_TYPE, LLD_CH_SCAN_RX_ISR_FUN_BIT, timestamp);
    DBG_SWDIAG(LECHSCAN, RX_ISR, 1);

    if (lld_ch_scan_env != NULL)
    {
        // Check if a descriptor has been used
        while (lld_rxdesc_check(EM_BLE_CS_ACT_ID_TO_INDEX(lld_ch_scan_env->act_id)))
        {
            // Get channel information from the exchange memory
            uint16_t rxchass   = em_ble_rxchass_get(lld_env.curr_rxdesc_index);
            uint8_t  rxrssi    = (rxchass & EM_BLE_RSSI_MASK) >> EM_BLE_RSSI_LSB;
            uint8_t  usedchidx = (rxchass & EM_BLE_USED_CH_IDX_MASK) >> EM_BLE_USED_CH_IDX_LSB;

            rwip_channel_assess_ble(usedchidx, RWIP_RX_NO_SYNC, rxrssi, timestamp, false);

            // Free RX descriptor
            lld_rxdesc_free();
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(LECHSCAN, RX_ISR, 0);
}

/**
 ****************************************************************************************
 * @brief Handle frame interrupt
 ****************************************************************************************
 */
__STATIC void lld_ch_scan_isr(uint32_t timestamp, uint8_t isr_type)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LLD_CH_SCAN_PATCH_TYPE, LLD_CH_SCAN_EOF_ISR_FUN_BIT, timestamp, isr_type);
    DBG_SWDIAG(LECHSCAN, EOF_ISR, 1);

    if (lld_ch_scan_env != NULL)
    {
        struct sch_arb_elt_tag *evt = &(lld_ch_scan_env->evt);

        // Remove event
        sch_arb_remove(evt, true);

        if (lld_ch_scan_env->state == CH_SCAN_EVT_END)
        {
            // Report channel scan end to LLM
            struct lld_ch_scan_end_ind *ind = KE_MSG_ALLOC(LLD_CH_SCAN_END_IND, TASK_LLM, TASK_NONE, lld_ch_scan_end_ind);
            ind->status = CO_ERROR_NO_ERROR;
            ind->act_id = lld_ch_scan_env->act_id;
            ke_msg_send(ind);

            // Free event memory
            lld_ch_scan_cleanup();
        }
        else
        {
            switch (isr_type)
            {
            /// Normal End of Event
            case SCH_FRAME_IRQ_EOF:
            {
                // Set again scan_duration
                lld_ch_scan_env->scan_current_duration = lld_ch_scan_env->scan_duration;
                // Reschedule after scan interval
                evt->time.hs      = CLK_ADD_2(evt->time.hs, lld_ch_scan_env->intv);
                evt->time.hus     = 0;
            }
            break;

            /// End of event due to an abort under/after the primary priority duration
            case SCH_FRAME_IRQ_EOF_ABORT_UNDER_PRIO:
            case SCH_FRAME_IRQ_EOF_ABORT_AFTER_PRIO:
            {
                uint32_t processing_time_us = 0;
                uint8_t rxdesc_cnt;
                uint8_t ch_idx = em_ble_hopcntl_ch_idx_getf(lld_ch_scan_env->act_id);

                if (lld_ch_scan_env->ch_sel == CH_SCAN_INACTIVE_CH)
                {
                    uint8_t byte_idx, bit_pos, ch_stat = 0;

                    // Search the first active channel by decreasing the index
                    while (ch_stat == 0)
                    {
                        ch_idx = (ch_idx - 1) % BLE_DATA_CHANNEL_MAX;
                        byte_idx = ch_idx >> 3;
                        bit_pos = ch_idx & 0x7;
                        ch_stat = lld_ch_scan_env->ch_map.map[byte_idx] & (1 << bit_pos);
                    }
                }
                else
                {
                    ch_idx = (ch_idx - 1) % BLE_DATA_CHANNEL_MAX;
                }

                em_ble_hopcntl_ch_idx_setf(lld_ch_scan_env->act_id, ch_idx);

                // Compute the processing time of the activity interrupted
                rxdesc_cnt = em_ble_txrxdesccnt_aclrxdesccnt_getf(lld_ch_scan_env->act_id);
                if (rxdesc_cnt > 0)
                {
                    processing_time_us = co_min(lld_ch_scan_env->scan_current_duration, ((rxdesc_cnt - 1) * lld_ch_scan_env->time_process_ch));
                }

                lld_ch_scan_env->scan_current_duration = co_max(SLOT_SIZE, (lld_ch_scan_env->scan_current_duration - processing_time_us));
            }
            break;

            /// SKIP Event
            case SCH_FRAME_IRQ_SKIP:
            {
                // Nothing to do, as the activity is rescheduled immediately
            }
            break;

            default:
            {
                ASSERT_INFO_FORCE(0, isr_type, 0);
            }
            break;
            }

            // Set channel map for channel scan
            lld_ch_scan_set_ch_map(EM_BLE_CS_ACT_ID_TO_INDEX(lld_ch_scan_env->act_id));

            // Move state
            lld_ch_scan_env->state = CH_SCAN_EVT_WAIT;

            if (sch_arb_insert(evt) != SCH_ARB_ERROR_OK)
            {
                ASSERT_ERR_FORCE(0);
            }
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(LECHSCAN, EOF_ISR, 0);
}

/**
 ****************************************************************************************
 * @brief Handle frame interrupt notification
 ****************************************************************************************
 */
__STATIC void lld_ch_scan_frm_cbk(uint32_t timestamp, uint32_t dummy, uint8_t irq_type)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LLD_CH_SCAN_PATCH_TYPE, LLD_CH_SCAN_FRM_CBK_FUN_BIT, timestamp, dummy, irq_type);
    switch (irq_type)
    {
    case SCH_FRAME_IRQ_EOF_ABORT_UNDER_PRIO:
    case SCH_FRAME_IRQ_EOF_ABORT_AFTER_PRIO:
    case SCH_FRAME_IRQ_EOF:
    case SCH_FRAME_IRQ_SKIP:
    {
        lld_ch_scan_isr(timestamp, irq_type);
    }
    break;
    case SCH_FRAME_IRQ_RX:
    {
        lld_ch_scan_rx_isr(timestamp);
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

uint8_t lld_ch_scan_start(uint8_t act_id, struct lld_ch_scan_params *params)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLD_CH_SCAN_PATCH_TYPE, LLD_CH_SCAN_START_FUN_BIT, uint8_t, act_id, params);
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    DBG_SWDIAG(LECHSCAN, START, 1);

    // Check if Channel Scanning is inactive
    if (lld_ch_scan_env == NULL)
    {
        // Allocate event
        lld_ch_scan_env = LLD_ALLOC_ENV(lld_ch_scan_env_tag);

        if (lld_ch_scan_env != NULL)
        {
            // Point to parameters
            struct lld_ch_scan_env_tag *ch_scan_evt = lld_ch_scan_env;
            struct sch_arb_elt_tag *evt = &(ch_scan_evt->evt);
            uint8_t cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(act_id);

            LLD_INIT_ENV(lld_ch_scan_env, lld_ch_scan_env_tag);

            // Set channel selection
            lld_ch_scan_env->ch_sel = params->ch_sel;

            // Init state
            lld_ch_scan_env->state = CH_SCAN_EVT_WAIT;

            // Initialize event parameters
            lld_ch_scan_env->act_id = act_id;
            ch_scan_evt->intv = params->intv << 2;

            // Set permission/status of CS as R/W but uninitialized
            DBG_MEM_PERM_SET((const void *)(REG_EM_BLE_CS_BASE_ADDR + REG_EM_BLE_CS_ADDR_GET(cs_idx)), REG_EM_BLE_CS_SIZE, true, true, true);

            // Initialize event parameters (common part)
            evt->cb_cancel        = &lld_ch_scan_evt_canceled_cbk;
            evt->cb_start         = &lld_ch_scan_evt_start_cbk;
            evt->current_prio     = rwip_priority[RWIP_PRIO_CH_SCAN_IDX].value;
            evt->duration_min     = params->scan_duration << 1;
            evt->time.hs          = lld_read_clock();
            SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_ASAP_NO_LIMIT, SCH_ARB_NO_PHASE, 0, RWIP_PRIO_INC(RWIP_PRIO_CH_SCAN_IDX));

            // Initialize event parameters (channel scan part)
            // Set the scan type for this PHY
            em_ble_cntl_pack(cs_idx, RWIP_COEX_GET(CH_SCAN, TXBSY),
                             RWIP_COEX_GET(CH_SCAN, RXBSY),
                             RWIP_COEX_GET(CH_SCAN, DNABORT),
#if (EAVESDROPPING_SUPPORT)
                             0,
#endif // EAVESDROPPING_SUPPORT
                             EM_BLE_CS_FMT_CHAN_SCAN);

            // update channel map value
            memcpy(&lld_ch_scan_env->ch_map.map[0], &params->ch_map.map[0], LE_CH_MAP_LEN);

            // Set channel map for channel scan
            lld_ch_scan_set_ch_map(cs_idx);

            // Thresholds and Rates
            em_ble_thrcntl_ratecntl_pack(cs_idx, /*rxthr*/   1,
                                         /*txthr*/   0,
                                         /*auxrate*/ 0,
                                         /*rxrate*/  CO_RATE_1MBPS,
                                         /*txrate*/  CO_RATE_1MBPS);

            // Hopping control
            em_ble_hopcntl_pack(cs_idx, /*fhen*/     true,
                                /*hop_mode*/ LLD_HOP_MODE_CHAN_SEL_1,
                                /*hopint*/   1,
                                /*chidx*/    BLE_DATA_CHANNEL_MAX);

            // Link control
            em_ble_linkcntl_pack(cs_idx, /*hplpmode*/      0,
                                 /*linklbl*/       cs_idx,
                                 /*sas*/           false,
                                 /*nullrxllidflt*/ false,
                                 /*micmode*/       ENC_MIC_PRESENT,
                                 /*cryptmode*/     ENC_MODE_PKT_PLD_CNT,
                                 /*txcrypten*/     false,
                                 /*rxcrypten*/     false,
                                 /*privnpub*/      false);

            // Set event duration
            ch_scan_evt->scan_duration = params->scan_duration;
            ch_scan_evt->scan_current_duration = ch_scan_evt->scan_duration;

            // Set RX HALF window size in us
            em_ble_rxwincntl_pack(cs_idx, /*rxwide*/ 0, /*rxwinsz*/ params->win_duration >> 1);

            // Set sync word
            em_ble_syncwl_set(cs_idx, (uint16_t)(RWIP_CH_SCAN_ACCESS_ADDR & 0xFFFF));
            em_ble_syncwh_set(cs_idx, (uint16_t)((RWIP_CH_SCAN_ACCESS_ADDR & 0xFFFF0000) >> 16));

            // Set min event duration
            em_ble_minevtime_set(EM_BLE_CS_ACT_ID_TO_INDEX(ch_scan_evt->act_id), 0);

            // Compute the time to process one channel
            // Radio Rx Power Up + Rx path delay + RX window size + Radio Rx Power down (same as Tx) + update/fetch RX descriptor (3us)
            ch_scan_evt->time_process_ch = ble_radiopwrupdn0_rxpwrup0_getf() + ble_radiotxrxtim0_rxpathdly0_getf() +
                                           params->win_duration + ble_radiopwrupdn0_txpwrdn0_getf() + 3;
            GLOBAL_INT_DISABLE();
            if (sch_arb_insert(evt) != SCH_ARB_ERROR_OK)
            {
                // Free event memory
                lld_ch_scan_cleanup();
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

    DBG_SWDIAG(LECHSCAN, START, 0);

    return (status);
}

void lld_ch_scan_stop(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LLD_CH_SCAN_PATCH_TYPE, LLD_CH_SCAN_STOP_FUN_BIT);
    DBG_SWDIAG(LECHSCAN, STOP, 1);

    GLOBAL_INT_DISABLE();

    if (lld_ch_scan_env != NULL)
    {
        // Point to parameters
        struct lld_ch_scan_env_tag *ch_scan_param = lld_ch_scan_env;
        struct sch_arb_elt_tag *evt = &(lld_ch_scan_env->evt);

        switch (ch_scan_param->state)
        {
        case CH_SCAN_EVT_WAIT:
        {
            // Remove event
            sch_arb_remove(evt, false);

            // Report channel Scanning end to LLM
            struct lld_ch_scan_end_ind *ind = KE_MSG_ALLOC(LLD_CH_SCAN_END_IND, TASK_LLM, TASK_NONE, lld_ch_scan_end_ind);
            ind->status = CO_ERROR_NO_ERROR;
            ind->act_id = lld_ch_scan_env->act_id;
            ke_msg_send(ind);

            // Free event memory
            lld_ch_scan_cleanup();
        }
        break;

        case CH_SCAN_EVT_ACTIVE:
        {
            // Move state
            lld_ch_scan_env->state = CH_SCAN_EVT_END;
        }
        break;

        default:
        {
            // Nothing to do
        }
        break;
        }
    }

    GLOBAL_INT_RESTORE();

    DBG_SWDIAG(LECHSCAN, STOP, 0);
}


void lld_ch_scan_init(bool is_reset)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_CH_SCAN_PATCH_TYPE, LLD_CH_SCAN_INIT_FUN_BIT, is_reset);
    if (!is_reset)
    {
        lld_ch_scan_env = NULL;
    }
    else
    {
        // clean-up allocated memory
        lld_ch_scan_cleanup();
    }
}

void lld_ch_scan_ch_map_update(struct le_ch_class *ch_class)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_CH_SCAN_PATCH_TYPE, LLD_CH_SCAN_CH_MAP_UPDATE_FUN_BIT, ch_class);

    GLOBAL_INT_DISABLE();

    if ((lld_ch_scan_env != NULL) && (lld_ch_scan_env->ch_sel == CH_SCAN_INACTIVE_CH))
    {
        // Start with all channels marked as good (no scan on them)
        memset(&lld_ch_scan_env->ch_map.map[0], 0x00, LE_CH_MAP_LEN);

        // Update channel map of the channel scanner
        // If channel is bad or unknown the channel scan should compute on it.
        for (int i = 0 ; i < BLE_DATA_CHANNEL_NB ; i++)
        {
            uint8_t byte_idx_class = i >> 2;
            uint8_t bit_pos_class = (i & 0x3) << 1;

            uint8_t byte_idx_map = i >> 3;
            uint8_t bit_pos_map = i & 0x7;

            // Clear previous state
            if (!(((ch_class->map[byte_idx_class] & (3 << bit_pos_class)) >> bit_pos_class) ==  LE_CH_CLASS_STATUS_GOOD))
            {
                lld_ch_scan_env->ch_map.map[byte_idx_map] |= (1 << bit_pos_map);
            }
        }
    }

    GLOBAL_INT_RESTORE();
}

#endif // (BLE_CH_SCAN_SUPPORT)
///@} LLDCHSCAN
