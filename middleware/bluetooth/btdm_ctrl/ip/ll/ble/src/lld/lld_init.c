/**
****************************************************************************************
*
* @file lld_init.c
*
* @brief LLD Initiating source code
*
* Copyright (C) RivieraWaves 2009-2016
*
****************************************************************************************
*/

/**
 ****************************************************************************************
 * @addtogroup LLDINIT
 * @ingroup LLD
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"         // stack configuration
#if (BLE_CENTRAL)

#include <string.h>

#include "co_math.h"
#include "co_endian.h"
#include "co_utils.h"
#include "ble_util.h"            // BLE utility functions

#include "ke_mem.h"
#include "ke_task.h"             // kernel task management
#include "rwip.h"

#include "lld.h"                 // link driver API
#include "lld_int.h"             // link layer driver internal

#include "dbg.h"

#include "sch_arb.h"             // Scheduling Arbiter
#include "sch_prog.h"            // Scheduling Programmer
#include "sch_slice.h"           // Scheduling Slicer

#include "em_map.h"

#include "reg_blecore.h"         // BLE core registers
#include "reg_em_ble_cs.h"       // BLE EM Control Structure
#include "reg_em_ble_tx_desc.h"  // BLE EM TX descriptors
#include "reg_em_ble_rx_desc.h"  // BLE EM RX descriptors
#include "reg_em_ble_ral.h"      // BLE RAL structures
#include "reg_em_et.h"           // EM Exchange Table
#include "btdm_patch.h"

/*
 * DEFINES
 *****************************************************************************************
 */

/// TX buffer area offset definitions
#define LLD_INIT_CONNECTREQ_1MBPS_OFFSET 0x00
#define LLD_INIT_CONNECTREQ_2MBPS_OFFSET 0x22
#define LLD_INIT_CONNECTREQ_CODED_OFFSET 0x44

/// RX Threshold for initiating
#define LLD_INIT_RX_THRESHOLD                 2

/// Bit field definition scanning PHYs
#define LLD_PHY_1M_POS            0
#define LLD_PHY_1M_BIT            0x01
#define LLD_PHY_CODED_POS         1
#define LLD_PHY_CODED_BIT         0x02


/*
 * ENUMERATION DEFINITION
 *****************************************************************************************
 */

/// Initiating event states
enum INIT_EVT_STATE
{
    INIT_EVT_WAIT,
    INIT_EVT_ACTIVE,
    INIT_EVT_END,
};

/// Init AUX offload modes
enum INIT_OFFLOAD_MODE
{
    NO_AUX_OFFLOAD = 0,
    SW_CNTL_AUX_OFFLOAD,
    HW_CNTL_AUX_OFFLOAD,
};

/// Parameter indexes
enum INIT_IDX
{
    IDX_1MBPS = 0,
    IDX_2MBPS = 1,
    IDX_CODED = 2,
};


/*
 * CONSTANT DEFINITION
 ****************************************************************************************
 */

/// Table indicating the max duration of an AUX_ADV_IND->AUX_CONNECT_REQ->AUX_CONNECT_RSP event (in us) depending on the rate used
const uint16_t lld_init_max_aux_dur_tab[CO_RATE_MAX] =
{
    [CO_RATE_1MBPS  ]  = (PDU_1MBPS_LEN_US(PDU_ADV_PAYLOAD_LEN_MAX)   + BLE_IFS_DUR + PDU_1MBPS_LEN_US(PDU_CON_REQ_LEN)   + BLE_IFS_DUR + PDU_1MBPS_LEN_US(PDU_CON_RSP_LEN)),
    [CO_RATE_2MBPS  ]  = (PDU_2MBPS_LEN_US(PDU_ADV_PAYLOAD_LEN_MAX)   + BLE_IFS_DUR + PDU_2MBPS_LEN_US(PDU_CON_REQ_LEN)   + BLE_IFS_DUR + PDU_2MBPS_LEN_US(PDU_CON_RSP_LEN)),
    [CO_RATE_500KBPS]  = (PDU_500KBPS_LEN_US(PDU_ADV_PAYLOAD_LEN_MAX) + BLE_IFS_DUR + PDU_500KBPS_LEN_US(PDU_CON_REQ_LEN) + BLE_IFS_DUR + PDU_500KBPS_LEN_US(PDU_CON_RSP_LEN)),
    [CO_RATE_125KBPS]  = (PDU_125KBPS_LEN_US(PDU_ADV_PAYLOAD_LEN_MAX) + BLE_IFS_DUR + PDU_125KBPS_LEN_US(PDU_CON_REQ_LEN) + BLE_IFS_DUR + PDU_125KBPS_LEN_US(PDU_CON_RSP_LEN)),
};

/*
 * STRUCTURE DEFINITIONS
 *****************************************************************************************
 */

/// LLD INIT event parameters structure
struct lld_init_env_params
{
    /// Initiator Scheduling Arbiter data
    struct sch_arb_elt_tag evt;

    /// Remaining slots in current scan window (in 625us BT slots)
    uint32_t win_rem_slots;

    /// Event timestamp of the last time the event priority was updated in half-slots (312.5 us)
    uint32_t last_prio_upd_ts;

    /// Scan interval in slots (625 us)
    uint16_t intv;

    /// Scan window in slots (625 us)
    uint16_t win;

    /// PHY Rate information (@see enum lld_rate)
    uint8_t rate;

    /// State
    uint8_t state;

    /// Advertiser identity address
    struct bd_addr adva;

    /// ADI field1: adv_ext_ind - used for collision detection
    uint16_t adi1;

    /// ADI field2: aux_adv_ind - used for collision detection
    uint16_t adi2;

    /// TX power of peer device
    uint8_t tx_power;

    /// Indicates offload to secondary PHY under software/hardware control (@see INIT_OFFLOAD_MODE)
    uint8_t aux_offload;

    /// Transmit Window Delay (slot pairs)
    uint8_t txwin_delay;

    /// Scheduling information for next AUX channel
    struct lld_calc_aux_rx_out aux_rx_out;

    /// Indicates a packet rx in this event
    bool pkt_rx;

    /// Index of the scan event environment
    uint8_t env_idx;

    /// Control Structure index
    uint8_t cs_idx;
};


/// LLD INIT connection parameters structure
struct lld_init_con_params
{
    /// Connection interval in slots pairs (1.25 ms)
    uint16_t intv;

    /// Connection offset in half-slots (312.5 us)
    uint16_t offset;

    /// Connection latency
    uint16_t latency;

    /// Connection supervision timeout in 10ms (N * 10ms)
    uint16_t superv_to;

    /// Index of this parameter set
    uint8_t phy_idx;
};

/// LLD INIT environment structure
struct lld_init_env_tag
{
    /// initiating event parameters
    struct lld_init_env_params *params[MAX_SCAN_PHYS];

    /// Indication to send when environment closed
    struct lld_init_end_ind *ind;

    /// initiating connection parameters
    struct lld_init_con_params con_params[MAX_INIT_PHYS];

    /// Extended initiating
    bool ext_init;

    /// Bit field of active scanning PHYs (bit 0: 1M | bit 1: coded)
    uint8_t scan_phys;

    /// Activity identifier
    uint8_t act_id;

    /// Channel map to use for the connection
    struct le_ch_map ch_map;

    /// Local address type
    uint8_t own_addr_type;

    /// Peer address type (only for directed advertising)
    uint8_t peer_addr_type;

    /// Generated access address
    struct access_addr  aa;

    /// CRC init
    struct crc_init     crcinit;

    /// hopping increment
    uint8_t             hop_inc;

    /// sleep clock accuracy
    uint8_t             m_sca;

    /// Local address
    struct bd_addr own_addr;

    /// Peer address
    struct bd_addr peer_addr;

    /// RAL mode - false=Disabled, true=Enabled.
    bool ral_mode;

    /// RAL structure pointer
    uint16_t ral_ptr;

    /// PHY to be used for the connection
    uint8_t con_rate;

    /// Initiator filter policy - 0x00=not used, 0x01=used.
    uint8_t filter_policy;

    /// Connect status - True = connecting | False = not connecting
    bool con_status;

    /// When connection request packet has been sent (half slots)
    uint32_t base_cnt;

    /// Indicates whether channel selection algorithm #2 will be used or not
    bool ch_sel_2;
};


/*
 * VARIABLE DEFINITION
 *****************************************************************************************
 */

/// LLD INIT environment variable
__STATIC struct lld_init_env_tag *lld_init_env;

/*
 * LOCAL FUNCTIONS DEFINITION
 *****************************************************************************************
 */
__STATIC void lld_init_frm_cbk(uint32_t timestamp, uint32_t dummy, uint8_t irq_type);

/**
 ****************************************************************************************
 * @brief End Scanning
 ****************************************************************************************
 */
__STATIC void lld_init_end(void)
{
    struct lld_init_env_params *init_par;
    struct sch_arb_elt_tag *evt;

    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LLD_INIT_PATCH_TYPE, LLD_INIT_END_FUNC_BIT);

    for (uint8_t scan_idx = 0; scan_idx < MAX_SCAN_PHYS; scan_idx++)
    {
        if (lld_init_env->params[scan_idx] == NULL)
            continue;

        // Point to parameters
        init_par = lld_init_env->params[scan_idx];
        evt = &(init_par->evt);

        switch (init_par->state)
        {
        case INIT_EVT_WAIT:
        {
            // Remove event
            sch_arb_remove(evt, false);
            // Unregister the init window from scheduling parameters
            sch_slice_bg_remove(BLE_INIT);

            // Remove permission/status of CS as now unused
            DBG_MEM_PERM_SET((const void *)(REG_EM_BLE_CS_BASE_ADDR + REG_EM_BLE_CS_ADDR_GET(init_par->cs_idx)), REG_EM_BLE_CS_SIZE, false, false, false);

            // Free this scanning entity
            ke_free(lld_init_env->params[scan_idx]);
            lld_init_env->params[scan_idx] = NULL;
            lld_init_env->scan_phys &= ~(1 << scan_idx);
        }
        break;

        case INIT_EVT_ACTIVE:
        {
            // Abort scan, and minimize maxevtime in case pre-fetch
            em_ble_maxevtime_set(init_par->cs_idx, 1);
            ble_rwblecntl_scan_abort_setf(1);

            // Indicate the scan entity needs to be closed
            init_par->state = INIT_EVT_END;
        }
        break;

        default:
        {
            // Nothing to do
        }
        break;
        }
    }

    // Check if no more scanning entity
    if (!lld_init_env->scan_phys)
    {
        // Check if an indication was prepared due to connection complete
        if (!lld_init_env->ind)
        {
            // Indicate connection was not completed
            struct lld_init_end_ind *ind = KE_MSG_ALLOC(LLD_INIT_END_IND, TASK_LLM, TASK_NONE, lld_init_end_ind);
            ind->act_id = lld_init_env->act_id;
            ind->connected = false;
            ke_msg_send(ind);
        }
        else
        {
            // Send prepared indication
            ke_msg_send(lld_init_env->ind);

            // Start connection
            {
                struct lld_con_params params;

                params.aa               = lld_init_env->aa;
                params.crcinit          = lld_init_env->crcinit;
                params.winsize          = 0;
                params.winoffset        = 0;
                params.interval         = lld_init_env->ind->interval;
                params.latency          = lld_init_env->ind->latency;
                params.timeout          = lld_init_env->ind->timeout;
                params.ch_map           = lld_init_env->ch_map;
                params.hop_inc          = lld_init_env->hop_inc;
                params.master_sca       = lld_init_env->m_sca;
                params.first_anchor_hs  = lld_init_env->base_cnt;
                params.first_anchor_hus = 0;
                params.role             = ROLE_MASTER;
                params.rate             = lld_init_env->con_rate;
                params.ch_sel_2         = lld_init_env->ch_sel_2;

                lld_con_start(lld_init_env->act_id, &params);
            }
        }

        // Free common event memory
        ke_free(lld_init_env);
        lld_init_env = NULL;
    }
}

/**
****************************************************************************************
* @brief Compute the window offset for initiating mode
*
* @param[in] interval           Connection interval (slot pairs)
* @param[in] offset             Connection offset (half slots)
* "param[in] txwin_delay        Transmit Window Delay (slot pairs)
* @param[in] timestamp          Window start Timestamp (half slots)
*
* @return Offset to set in the CS-WINOFFSET. (slot pairs)
****************************************************************************************
*/
__STATIC uint16_t lld_init_compute_winoffset(uint16_t con_intv, uint16_t con_offset, uint8_t txwin_delay, uint32_t timestamp)
{
    uint32_t output_offset;

    FUNC_PATCH_ENTRY_4_PARAM_HAVE_RETURN(LLD_INIT_PATCH_TYPE, LLD_INIT_COMPUTE_WINOFFSET_FUNC_BIT, uint16_t, con_intv, con_offset, txwin_delay, timestamp);

    // Calculate offset based on con_intv, ref timestamp, and timestamp. As N*intv + offset % intv  == offset,
    // it is equivalent to use con_offset here instead of a reference timestamp.
    // output value in half slots
    output_offset = (con_intv << 2) - CO_MOD(timestamp, (con_intv << 2)) + con_offset;

    // If the offset is too close (4 slots + txwin_delay)
    if ((int32_t)output_offset < (8 + (txwin_delay << 2)))
    {
        // Else set the offset to the next anchor point
        output_offset += (con_intv << 2);
    }

    // return the offset in frame (2 slots) steps, removing N frame transmit window delay
    return ((output_offset) >> 2) - txwin_delay;
}

/**
 ****************************************************************************************
 * @brief Schedule next LE initiating activity
 ****************************************************************************************
 */
__STATIC void lld_init_sched(uint8_t env_idx, uint32_t timestamp, bool resched)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LLD_INIT_PATCH_TYPE, LLD_INIT_SCHED_FUNC_BIT, env_idx, timestamp, resched);

    // Point to parameters
    struct lld_init_env_params *init_par = lld_init_env->params[env_idx];
    struct sch_arb_elt_tag *evt = &(init_par->evt);
    bool elt_inserted = false;

    uint32_t clock = lld_read_clock();
    uint32_t effective_slots = (CLK_SUB(clock, timestamp) + 1) / 2;

    uint8_t cs_idx = init_par->cs_idx;


    DBG_SWDIAG(LEINIT, SCHED, 1);

    if (SW_CNTL_AUX_OFFLOAD == init_par->aux_offload) // SW controlled offload to secondary channel
    {
        // Unregister the scan window from scheduling parameters
        sch_slice_bg_remove(BLE_INIT);

        evt->time.hs = init_par->aux_rx_out.time.hs;
        evt->time.hus = init_par->aux_rx_out.time.hus;
        evt->duration_min = ((lld_init_max_aux_dur_tab[(init_par->aux_rx_out.rate)] + init_par->aux_rx_out.sync_win_size_us) << 1) + BLE_RESERVATION_TIME_MARGIN_HUS;

        // Schedule offloaded channels NO_ASAP
        evt->current_prio = co_max(rwip_priority[RWIP_PRIO_AUX_RX_IDX].value, evt->current_prio);
        SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_NO_ASAP, SCH_ARB_NO_PHASE, 0, RWIP_PRIO_INC(RWIP_PRIO_AUX_RX_IDX));

        if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
        {
            /* vol.6. part B LLS 4.5.3.
             * The value of transmitWindowDelay shall be 2.5 ms when an AUX_CONNECT_REQ PDU is used on an LE
                  Uncoded PHY, and 3.75 ms when an AUX_CONNECT_REQ PDU is used on the LE Coded PHY.
             */
            uint8_t aux_rate = init_par->aux_rx_out.rate;
            uint8_t max_evt_slots;
            uint32_t aux_sync_win_size_us = init_par->aux_rx_out.sync_win_size_us;

            init_par->txwin_delay = (aux_rate < CO_RATE_125KBPS) ? 2 : 3;

            // Set aux rate
            em_ble_thrcntl_ratecntl_aux_rate_setf(cs_idx, aux_rate);

            // Set max event time limitation for offload: sync_win + aux_adv_ind + t_ifs + connect_req + t_ifs + connect_rsp
            max_evt_slots = (lld_init_max_aux_dur_tab[aux_rate] + aux_sync_win_size_us + (SLOT_SIZE - 1)) / SLOT_SIZE;

            em_ble_maxevtime_set(cs_idx, max_evt_slots);

            // Set Wide-open mode, Size of the Rx window in slots
            if (aux_sync_win_size_us > EM_BLE_CS_RXWINSZ_MAX)
            {
                // Size of the Rx half window in half-slots (wide-open mode)
                uint8_t sync_win_slots = (aux_sync_win_size_us + (SLOT_SIZE - 1)) / SLOT_SIZE;

                em_ble_rxwincntl_pack(cs_idx, 1, sync_win_slots);
            }
            else
            {
                // Size of the Rx half window in us (normal mode)
                em_ble_rxwincntl_pack(cs_idx, 0, (aux_sync_win_size_us + 1) >> 1);
            }

            // set Channel Index for Aux.
            em_ble_chmap2_ch_aux_setf(cs_idx, init_par->aux_rx_out.ch_idx);

            init_par->pkt_rx = false;
            elt_inserted = true;
        }
        else
        {
            // Erroneous receive - Reset aux offload mode for next ADV_EXT_IND
            init_par->aux_offload = NO_AUX_OFFLOAD;
        }
    }

    if (!elt_inserted)
    {
        if (effective_slots < init_par->win_rem_slots)
        {
            // Subtract the number of effective scanning slots
            init_par->win_rem_slots -= effective_slots;
        }
        else
        {
            // Unregister the initiating window from scheduling parameters
            sch_slice_bg_remove(BLE_INIT);

            // Add scan window period delay adjusted
            evt->time.hs = CO_ALIGN4_LO(CLK_ADD_2(clock, 2 * (init_par->intv - init_par->win)));

            // Reset delay to slot boundary
            evt->time.hus = 0;

            // Reload window length
            init_par->win_rem_slots = init_par->win;
        }

        evt->duration_min = co_min((2 * init_par->win_rem_slots * SLOT_SIZE), sch_slice_params.scan_evt_dur);

        if (!resched)
        {
            // Reset the current priority
            evt->current_prio = rwip_priority[RWIP_PRIO_INIT_IDX].value;
            // Save the timestamp
            init_par->last_prio_upd_ts = evt->time.hs;
        }

        // Schedule primary scanning ASAP
        SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_ASAP_NO_LIMIT, SCH_ARB_NO_PHASE, 0, RWIP_PRIO_INC(RWIP_PRIO_INIT_IDX));

        /* vol.6. part B LLS 4.5.3.
         * The value of transmitWindowDelay shall be 1.25 ms when a CONNECT_IND PDU is used.
         */
        init_par->txwin_delay = 1;

        em_ble_maxevtime_set(cs_idx, init_par->win_rem_slots);

        // set Wide-open mode, Size of the Rx half-window in half-slots
        em_ble_rxwincntl_pack(cs_idx, 1, init_par->win_rem_slots);

        // initialize all extadv status as start of new sequence
        em_ble_extadvstat_pack(cs_idx, 0, 0, 0, 0, 0);

#if (BLE_HOST_PRESENT)  // BLE HOST workaround allows random address update on active scans
        // Set the Device identity (BD Address)
        em_ble_lebdaddr_set(cs_idx, 0, co_read16p(&lld_init_env->own_addr.addr[0]));
        em_ble_lebdaddr_set(cs_idx, 1, co_read16p(&lld_init_env->own_addr.addr[2]));
        em_ble_lebdaddr_set(cs_idx, 2, co_read16p(&lld_init_env->own_addr.addr[4]));
#endif // (BLE_HOST_PRESENT)

        if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
        {
            init_par->pkt_rx = false;
        }
        else
        {
            ASSERT_ERR_FORCE(0);
        }
    }

    DBG_SWDIAG(LEINIT, SCHED, 0);
}

/**
 ****************************************************************************************
 * @brief Process LE initiating Pkt Rx
 ****************************************************************************************
 */
__STATIC void lld_init_process_pkt_rx(uint8_t env_idx)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_INIT_PATCH_TYPE, LLD_INIT_PROCESS_PKT_RX_FUNC_BIT, env_idx);

    if ((lld_init_env != NULL) && (lld_init_env->params[env_idx] != NULL))
    {
        // Point to parameters
        struct lld_init_env_params *init_par = lld_init_env->params[env_idx];
        struct lld_init_env_tag *init_env = lld_init_env;

        // Process rx packet(s) if done. Ensure robust to multiple rx (as observed on target).
        while (lld_rxdesc_check(init_par->cs_idx))
        {
            // Get current RX descriptor index
            uint8_t rxdesc_idx = lld_env.curr_rxdesc_index;

            // Retrieve RX status and type
            uint16_t rxstat_pkt = em_ble_rxstatadv_get(rxdesc_idx);

            DBG_SWDIAG(LEINIT, PKT_RX, 1);

            // Trace the current RX descriptor
            TRC_REQ_RX_DESC(LLD_INIT, lld_init_env->act_id, REG_EM_BLE_RX_DESC_ADDR_GET(rxdesc_idx));

            // Check if packet reception is correct
            if (((rxstat_pkt & LLD_ADV_ERR_MASK) == 0) && !(init_env->con_status))
            {
                uint8_t rxtype = em_ble_rxphadv_rxtype_getf(rxdesc_idx);
                // The received packet indicates which device has been connected, the driver
                // should retrieve this info and report back to LLM
                uint16_t rxdataptr = em_ble_rxdataptr_get(rxdesc_idx);

                // Flag that a packet has been received
                init_par->pkt_rx = true;
                TRC_REQ_INIT_RX_PDU(rxdesc_idx, rxdataptr);

                if ((BLE_ADV_IND == rxtype) || (BLE_ADV_DIRECT_IND == rxtype))
                {
                    // retrieve rxadvlen, rate
                    uint16_t rxadvlen = em_ble_rxphadv_rxadvlen_getf(rxdesc_idx);
                    uint8_t rate = em_ble_rxchass_rate_getf(rxdesc_idx);

                    // Read clock value where sync has been found
                    uint32_t base_cnt = (em_ble_rxclknsync1_clknrxsync1_getf(rxdesc_idx) << 16)
                                        | (em_ble_rxclknsync0_clknrxsync0_getf(rxdesc_idx) << 0);

                    // Read bit position where sync has been found (in half us)
                    uint32_t fine_cnt = LLD_FINECNT_MAX - em_ble_rxfcntsync_fcntrxsync_getf(lld_env.curr_rxdesc_index);

                    // add duration of packet and size in half us to know when connection request packet has been sent
                    fine_cnt += ((ble_util_pkt_dur_in_us(rxadvlen, rate) - lld_exp_sync_pos_tab[rate] + BLE_IFS_DUR + ble_util_pkt_dur_in_us(PDU_CON_REQ_LEN, rate)) << 1);

                    // compute when the connection request packet is sent in half slots.
                    init_env->base_cnt = CLK_ADD_2(base_cnt, (fine_cnt + (HALF_SLOT_SIZE - 1)) / HALF_SLOT_SIZE) ;

                    // retrieve RAL pointer
                    init_env->ral_ptr = em_ble_rxralptr_getf(rxdesc_idx);

                    init_env->ch_sel_2 = em_ble_rxphadv_rxchsel2_getf(rxdesc_idx);
                    init_env->con_rate = rate;

                    em_rd((void *)&init_par->adva.addr[0], rxdataptr, BD_ADDR_LEN);

                    // retrieve peer address type
                    init_env->peer_addr_type = em_ble_rxphadv_rxtxadd_getf(rxdesc_idx);

                    init_env->con_status = true;
                }
                else
                {
                    uint8_t rxaemode = em_ble_rxaeheader_rxaemode_getf(rxdesc_idx);
                    uint16_t rxaeheader = em_ble_rxaeheader_get(rxdesc_idx);

                    int i = 0; // index into rxdataptr.

                    if ((BLE_ADV_EXT_IND == rxtype) && (rxaemode & CON_ADV_EVT_MSK))
                    {
                        // Fields applicable to ADV_EXT_IND: ADI:M, AuxPtr:M, TxPower:O.
                        // Fields applicable to AUX_ADV_IND: AdvA:M, TargetA:O, ADI:M, TxPower:O, ACAD:O, AdvData:O.

                        init_par->aux_offload = NO_AUX_OFFLOAD;

                        if (rxaeheader & EM_BLE_RXADVA_BIT) // AdvA (AUX_ADV_IND)
                        {
                            em_rd((void *)&init_par->adva.addr[0], rxdataptr + i, BD_ADDR_LEN);
                            i += BD_ADDR_LEN;
                        }

                        if (rxaeheader & EM_BLE_RXTGTA_BIT) // TargetA (AUX_ADV_IND): No action required
                        {
                            i += BD_ADDR_LEN;
                        }

                        if (rxaeheader & EM_BLE_RXCTE_BIT) // SuppInfo (N/A): No action required
                        {
                            i += BLE_EXT_CTE_INFO_LEN;
                        }

                        if ((rxaeheader & EM_BLE_RXADI_BIT) && (rxaeheader & EM_BLE_RXAUXPTR_BIT)) // ADI & AuxPtr (ADV_EXT_IND)
                        {
                            init_par->adi1 = em_rd16p(rxdataptr + i);
                            i += BLE_EXT_ADI_LEN;

                            /* Check for ADI match, and if HW RW-BLE Core will manage AuxPtr automatically:
                             * We can use FOLLOW_ADI for indication of the HW follow auxptr. If this bit is not set, HW will stop
                             * the event and let the RW-BLE Software reschedule chained packet later on a new event.
                             */
                            uint32_t aux_data;
                            em_rd((void *)&aux_data, rxdataptr + i, BLE_EXT_AUX_PTR_LEN);

                            if (0 == em_ble_rxstatadv_followauxptr_getf(rxdesc_idx))
                            {
                                uint8_t aux_phy = GETF(aux_data, BLE_AUX_PHY);

                                /* Check if PHY supported. Perform software offload only if AUX PHY supported, otherwise
                                   initiator shall remain scanning on the primary PHY. */
                                bool phy_support = ((BLE_PHY_1MBPS_SUPPORT) && (AUX_PHY_1MBPS == aux_phy))
                                                   || ((BLE_PHY_2MBPS_SUPPORT) && (AUX_PHY_2MBPS == aux_phy))
                                                   || ((BLE_PHY_CODED_SUPPORT) && (AUX_PHY_CODED == aux_phy));

                                if (phy_support && lld_calc_aux_rx(&init_par->aux_rx_out, rxdesc_idx, aux_data))
                                {
                                    // Calculated AUX RX activity for Software reschedule
                                    init_par->aux_offload = SW_CNTL_AUX_OFFLOAD;
                                }
                            }
                            else
                            {
                                // Store in case automatic procedure later aborted by other activity
                                init_par->aux_offload = HW_CNTL_AUX_OFFLOAD;
                            }

                            i += BLE_EXT_AUX_PTR_LEN;
                        }
                        else if (rxaeheader & EM_BLE_RXADI_BIT) // ADI (AUX_ADV_IND): Save
                        {
                            init_par->adi2 = em_rd16p(rxdataptr + i);
                            i += BLE_EXT_ADI_LEN;
                        }

                        if (rxaeheader & EM_BLE_RXSYNC_BIT) // SyncInfo (N/A): No action required
                        {
                            i += BLE_EXT_SYNC_LEN;
                        }

                        if (rxaeheader & EM_BLE_RXPOW_BIT) // TxPower: Save/overwrite as most recent value
                        {
                            init_par->tx_power = em_rd8p(rxdataptr + i);
                            // i+= BLE_EXT_TX_PWR_LEN;
                        }

                        /* ACAD Field present if i < rxaelength. However, ACAD in our standard IP
                           Ignore field when received.*/
                    }
                    else if (BLE_AUX_CONNECT_RSP == rxtype)
                    {
                        // Since the HW reports all Rx RSP packet (no filtering), the SW Init decision to connect on secondary,
                        // needs to check the AUX CONNECT RSP with tgta present and bdaddr match bit in the rxdesc.
                        if ((init_par->adi1 == init_par->adi2) && (rxaeheader & EM_BLE_RXADVA_BIT) && (rxaeheader & EM_BLE_RXTGTA_BIT)
                                && em_ble_rxstatadv_dev_filtering_ok_getf(rxdesc_idx))
                        {
                            // retrieve rate
                            uint8_t rate = em_ble_rxchass_rate_getf(rxdesc_idx);

                            // Read clock value where sync has been found
                            uint32_t base_cnt = (em_ble_rxclknsync1_clknrxsync1_getf(rxdesc_idx) << 16)
                                                | (em_ble_rxclknsync0_clknrxsync0_getf(rxdesc_idx) << 0);

                            // Read bit position where sync has been found (in half us)
                            uint32_t fine_cnt = LLD_FINECNT_MAX - em_ble_rxfcntsync_fcntrxsync_getf(lld_env.curr_rxdesc_index);

                            // subtract duration of half us to when connection request packet had been sent, avoid below-zero values
                            fine_cnt += HALF_SLOT_SIZE - ((lld_exp_sync_pos_tab[em_ble_rxchass_rate_getf(rxdesc_idx)] + BLE_IFS_DUR) << 1);

                            // compute when the connection request packet has been sent in half slots.
                            init_env->base_cnt = CLK_ADD_2(base_cnt, ((fine_cnt + (HALF_SLOT_SIZE - 1)) / HALF_SLOT_SIZE) - 1) ;

                            // retrieve RAL pointer
                            init_env->ral_ptr = em_ble_rxralptr_getf(rxdesc_idx);

                            if (init_env->ral_ptr)
                            {
                                // Store latest received address, may have changed since AUX_ADV_IND due to RPA refresh
                                em_rd((void *)&init_par->adva.addr[0], rxdataptr + i, BD_ADDR_LEN);
                            }

                            init_env->ch_sel_2 = true;
                            init_env->con_rate = rate;

                            // retrieve peer address type
                            init_env->peer_addr_type = em_ble_rxphadv_rxtxadd_getf(rxdesc_idx);

                            init_env->con_status = true;
                        }

                        init_par->aux_offload = NO_AUX_OFFLOAD;
                    }
                }
            }

            // Free RX descriptor
            lld_rxdesc_free();

            DBG_SWDIAG(LEINIT, PKT_RX, 0);
        }

        if ((HW_CNTL_AUX_OFFLOAD == init_par->aux_offload) || (!init_par->pkt_rx && (SW_CNTL_AUX_OFFLOAD == init_par->aux_offload)))
        {
            // Erroneous receive - Reset aux offload mode for next ADV_EXT_IND
            init_par->aux_offload = NO_AUX_OFFLOAD;
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }
}

/**
 ****************************************************************************************
 * @brief Process LE initiating Pkt Tx
 ****************************************************************************************
 */
__STATIC void lld_init_process_pkt_tx(uint8_t env_idx)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_INIT_PATCH_TYPE, LLD_INIT_PROCESS_PKT_TX_FUNC_BIT, env_idx);

    // Point to parameters
    struct lld_init_env_params *init_par = lld_init_env->params[env_idx];
    struct lld_init_env_tag *init_env = lld_init_env;

    uint8_t txdesc_idx = EM_BLE_TXDESC_INDEX(init_env->act_id, 0);

    if (em_ble_txcntl_txdone_getf(txdesc_idx))
    {
        // Release CONNECT_IND/AUX_CONNECT_REQ descriptor
        em_ble_txcntl_txdone_setf(txdesc_idx, 0);

        if (init_env->con_status)
        {
            struct lld_init_end_ind *ind = KE_MSG_ALLOC(LLD_INIT_END_IND, TASK_LLM, TASK_NONE, lld_init_end_ind);

            uint8_t cs_idx = init_par->cs_idx;

            /*The transmit window starts at transmitWindowDelay + transmitWindowOffset after the end of
             *the packet containing the CONNECT_IND/AUX_CONNECT_REQ PDU */
            struct lld_init_con_params *con_par = &init_env->con_params[co_rate_to_phy[init_env->con_rate] - 1];

            // retrieve TX win offset --> translate it to half slot (tx win offset is in frame)
            uint32_t tx_win_offset = (em_ble_txwinoffset_getf(cs_idx) << 2);

            // retrieve base count of when connection request packet has been sent (half slots)
            uint32_t base_cnt = init_env->base_cnt;

            // compute the connection timestamp according to connection req/ind TX time
            base_cnt = CLK_ADD_2(base_cnt, ((con_par->intv << 2) - CO_MOD(CLK_SUB(base_cnt, con_par->offset), (con_par->intv << 2))));

            // If timestamp is before TX window offset
            if (CLK_SUB(base_cnt, init_env->base_cnt) < tx_win_offset)
            {
                // Use next connection interval
                base_cnt = CLK_ADD_2(base_cnt, con_par->intv << 2);
            }

            init_env->base_cnt = base_cnt;

            // Prepare connect indication to LLM
            memset(ind, 0, sizeof(struct lld_init_end_ind));

            // get Peer address type
            ind->peer_addr_type = init_env->peer_addr_type & ADDR_MASK;

            if (init_env->ral_ptr != 0)
            {
                uint8_t ral_idx = (init_env->ral_ptr - REG_EM_BLE_RAL_ADDR_GET(0)) / REG_EM_BLE_RAL_SIZE;

                // get the peer ID -  Do a burst read in the exchange memory
                em_rd(&ind->peer_id_addr.addr[0], init_env->ral_ptr + EM_BLE_RAL_PEER_ID_INDEX * 2, BD_ADDR_LEN);

                if ((init_env->own_addr_type & ADDR_RPA_MASK) && em_ble_ral_info_local_rpa_valid_getf(ral_idx))
                {
                    // get the local RPA -  Do a burst read in the exchange memory
                    em_rd(&ind->local_rpa.addr[0], init_env->ral_ptr + EM_BLE_RAL_LOCAL_RPA_INDEX * 2, BD_ADDR_LEN);
                }

                if (em_ble_ral_info_peer_rpa_valid_getf(ral_idx))
                {
                    // get the peer RPA -  Do a burst read in the exchange memory
                    em_rd(&ind->peer_rpa.addr[0], init_env->ral_ptr + EM_BLE_RAL_PEER_RPA_INDEX * 2, BD_ADDR_LEN);

                    ind->peer_addr_type = em_ble_ral_info_peer_id_type_getf(ral_idx) | ADDR_RPA_MASK;
                }
            }
            else
            {
                // Otherwise  peer ID would be the address presented by peer in advertising packet.
                memcpy(&ind->peer_id_addr.addr[0], &init_par->adva.addr[0], BD_ADDR_LEN);
            }

            // get Connection interval in slots pairs (1.25 ms)
            ind->interval = con_par->intv;

            // get Connection offset in half-slots
            ind->con_offset = con_par->offset;

            // get the channel map
            memcpy(&ind->ch_map.map[0], &init_env->ch_map.map[0], LE_CH_MAP_LEN);

            // get the initiating PHY index used
            ind->phy_idx = con_par->phy_idx;

            // get the PHY rate to be used for the connection
            ind->rate = init_env->con_rate;

            // get the connection latency
            ind->latency = con_par->latency;

            // get the connection supervision timeout
            ind->timeout = con_par->superv_to;

            // get the master sleep clock accuracy
            ind->master_sca = init_env->m_sca;

            // Indicates whether channel selection algorithm #2 will be used or not
            ind->ch_sel_2 = init_env->ch_sel_2;

            ind->connected = true;
            ind->act_id = init_env->act_id;

            // Assign indication to be sent later, at environment closure
            init_env->ind = ind;

            TRC_REQ_INIT_TX_PDU(txdesc_idx, init_par->adva);

            // The initiating is over
            lld_init_end();
        }
    }
    else
    {
        init_env->con_status = false;
    }
}

/**
 ****************************************************************************************
 * @brief Handle frame interrupt
 ****************************************************************************************
 */
__STATIC void lld_init_frm_eof_isr(uint8_t env_idx, uint32_t timestamp, bool abort)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LLD_INIT_PATCH_TYPE, LLD_INIT_FRM_EOF_ISR_FUNC_BIT, env_idx, timestamp, abort);

    DBG_SWDIAG(LEINIT, FRM_EOF_ISR, 1);

    if ((lld_init_env != NULL) && (lld_init_env->params[env_idx] != NULL))
    {
        // Point to parameters
        struct lld_init_env_params *init_par = lld_init_env->params[env_idx];
        struct sch_arb_elt_tag *evt = &(init_par->evt);
        uint8_t cs_idx = init_par->cs_idx;

        // Check if scan is under termination
        bool end = (init_par->state == INIT_EVT_END);

        // Set the state back to Wait, as the frame is completed
        init_par->state = INIT_EVT_WAIT;

        // If scan is under termination
        if (end)
        {
            // Flush potentially consumed descriptor(s)
            while (lld_rxdesc_check(cs_idx))
            {
                // Free RX descriptor
                lld_rxdesc_free();
            }

            // Close the driver
            lld_init_end();
        }
        else
        {
            // Remove event
            sch_arb_remove(evt, true);

            // Check rx packet discards (if not respecting T_MAFS, HW discards the chained packet)
            if (em_ble_txrxcntl_rxmafserr_getf(cs_idx))
            {
                em_ble_txrxcntl_rxmafserr_setf(cs_idx, 0);
                // Chained packet discarded  - Reset aux offload for next ADV_EXT_IND
                init_par->aux_offload = NO_AUX_OFFLOAD;
            }

            // Process packet rx
            lld_init_process_pkt_rx(env_idx);

            // Process packet tx
            lld_init_process_pkt_tx(env_idx);

            // Check if the initiating is still active
            if ((lld_init_env != NULL) && (lld_init_env->params[env_idx] != NULL))
            {
                // Reschedule according to the current mode
                lld_init_sched(env_idx, timestamp, abort);
            }
        }
    }
    else
    {
        ASSERT_INFO_FORCE(0, env_idx, lld_init_env);
    }

    DBG_SWDIAG(LEINIT, FRM_EOF_ISR, 0);
}

/**
 ****************************************************************************************
 * @brief Handle skip interrupt
 ****************************************************************************************
 */
__STATIC void lld_init_frm_skip_isr(uint8_t env_idx)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_INIT_PATCH_TYPE, LLD_INIT_FRM_SKIP_ISR_FUNC_BIT, env_idx);

    DBG_SWDIAG(LEINIT, EVT_CANCELED, 1);

    if ((lld_init_env != NULL) && (lld_init_env->params[env_idx] != NULL))
    {
        // Point to parameters
        struct lld_init_env_params *init_par = lld_init_env->params[env_idx];
        struct sch_arb_elt_tag *evt = &(init_par->evt);
        uint32_t clock = lld_read_clock();

        // Check if scan is under termination
        bool end = (init_par->state == INIT_EVT_END);

        // Set the state back to Wait, as the frame is completed
        init_par->state = INIT_EVT_WAIT;

        // If scan is under termination
        if (end)
        {
            // Close the driver
            lld_init_end();
        }
        else
        {
            // Remove event
            sch_arb_remove(&init_par->evt, true);

            if (CLK_SUB(clock, init_par->last_prio_upd_ts) >= 2 * init_par->intv)
            {
                // Increment priority
                evt->current_prio = RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(RWIP_PRIO_INIT_IDX));

                // Save the timestamp
                init_par->last_prio_upd_ts = clock;
            }

            if (SW_CNTL_AUX_OFFLOAD != init_par->aux_offload)
            {
                // Reschedule ASAP
                if (sch_arb_insert(evt) != SCH_ARB_ERROR_OK)
                {
                    ASSERT_ERR_FORCE(0);
                }
            }
            else
            {
                init_par->aux_offload = NO_AUX_OFFLOAD;

                // Reschedule according to the current mode
                lld_init_sched(env_idx, clock, true);
            }
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(LEINIT, EVT_CANCELED, 0);
}

/**
 ****************************************************************************************
 * @brief Handle frame interrupt notification
 ****************************************************************************************
 */
__STATIC void lld_init_frm_cbk(uint32_t timestamp, uint32_t dummy, uint8_t irq_type)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LLD_INIT_PATCH_TYPE, LLD_INIT_FRM_CBK_FUNC_BIT, timestamp, dummy, irq_type);

    DBG_SWDIAG(LEINIT, FRM_CBK, 1);

    ASSERT_INFO(dummy < MAX_SCAN_PHYS, dummy, irq_type);

    switch (irq_type)
    {
    case SCH_FRAME_IRQ_EOF:
    case SCH_FRAME_IRQ_EOF_ABORT_AFTER_PRIO: // Post-bandwidth contested as low priority (prio_2 = 0)
    {
        lld_init_frm_eof_isr(dummy, timestamp, false);
    }
    break;
    case SCH_FRAME_IRQ_EOF_ABORT_UNDER_PRIO:
    {
        lld_init_frm_eof_isr(dummy, timestamp, true);
    }
    break;
    case SCH_FRAME_IRQ_RX:
    {
        // Process packet rx
        lld_init_process_pkt_rx(dummy);
    }
    break;
    case SCH_FRAME_IRQ_SKIP:
    {
        lld_init_frm_skip_isr(dummy);
    }
    break;
    default:
    {
        ASSERT_INFO_FORCE(0, dummy, irq_type);
    }
    break;
    }

    DBG_SWDIAG(LEINIT, FRM_CBK, 0);
}

/**
 ****************************************************************************************
 * @brief Handle event start notification
 ****************************************************************************************
 */
__STATIC void lld_init_evt_start_cbk(struct sch_arb_elt_tag *evt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_INIT_PATCH_TYPE, LLD_INIT_EVT_START_CBK_FUNC_BIT, evt);

    DBG_SWDIAG(LEINIT, EVT_START, 1);

    if (evt != NULL)
    {
        // Point to parameters
        struct lld_init_env_params *init_par = (struct lld_init_env_params *) evt;
        struct lld_init_env_tag *init_env = lld_init_env;

        uint8_t cs_idx = init_par->cs_idx;
        uint8_t txwin_delay = init_par->txwin_delay;

        do
        {
            // set Transmit Window Offset Initialization value for device as initiator for each PHY (multiple of 1.25ms)
            em_ble_winoffset_set(cs_idx, lld_init_compute_winoffset(init_env->con_params[IDX_1MBPS].intv,
                                 init_env->con_params[IDX_1MBPS].offset, txwin_delay, evt->time.hs));
            em_ble_winoffset_2m_set(cs_idx, lld_init_compute_winoffset(init_env->con_params[IDX_2MBPS].intv,
                                    init_env->con_params[IDX_2MBPS].offset, txwin_delay, evt->time.hs));
            em_ble_winoffset_lr_set(cs_idx, lld_init_compute_winoffset(init_env->con_params[IDX_CODED].intv,
                                    init_env->con_params[IDX_CODED].offset, txwin_delay, evt->time.hs));

            {
                // Push the programming to SCH PROG
                struct sch_prog_params prog_par;
                prog_par.frm_cbk        = &lld_init_frm_cbk;
                prog_par.time.hs        = evt->time.hs;
                prog_par.time.hus       = evt->time.hus;
                prog_par.cs_idx         = cs_idx;
                prog_par.dummy          = init_par->env_idx;
                prog_par.bandwidth      = evt->duration_min;
                prog_par.prio_1         = evt->current_prio;
                prog_par.prio_2         = 0;
                prog_par.prio_3         = rwip_priority[RWIP_PRIO_AUX_RX_IDX].value;
                prog_par.pti_prio       = RW_BLE_PTI_PRIO_AUTO;
                prog_par.add.ble.ae_nps = (SW_CNTL_AUX_OFFLOAD == init_par->aux_offload);
                prog_par.add.ble.iso    = 0;
                prog_par.mode           = SCH_PROG_BLE;
                sch_prog_push(&prog_par);
            }

            // Register the initiating window as active to scheduling parameters
            sch_slice_bg_add(BLE_INIT);

            // Move state
            init_par->state = INIT_EVT_ACTIVE;

        }
        while (0);
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(LEINIT, EVT_START, 0);
}

/**
 ****************************************************************************************
 * @brief Handle event canceled notification
 ****************************************************************************************
 */
__STATIC void lld_init_evt_canceled_cbk(struct sch_arb_elt_tag *evt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_INIT_PATCH_TYPE, LLD_INIT_EVT_CANCELED_CBK_FUNC_BIT, evt);

    DBG_SWDIAG(LEINIT, EVT_CANCELED, 1);

    if (evt != NULL)
    {
        // Point to parameters
        struct lld_init_env_params *init_par = (struct lld_init_env_params *) evt;

        if (init_par->state == INIT_EVT_WAIT)
        {
            uint32_t clock = lld_read_clock();

            if (CLK_SUB(clock, init_par->last_prio_upd_ts) >= 2 * init_par->intv)
            {
                // Increment priority
                evt->current_prio = RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(RWIP_PRIO_INIT_IDX));

                // Save the timestamp
                init_par->last_prio_upd_ts = clock;
            }

            if (SW_CNTL_AUX_OFFLOAD != init_par->aux_offload)
            {
                // Reschedule ASAP
                if (sch_arb_insert(evt) != SCH_ARB_ERROR_OK)
                {
                    ASSERT_ERR_FORCE(0);
                }
            }
            else
            {
                init_par->aux_offload = NO_AUX_OFFLOAD;

                // Reschedule according to the current mode
                lld_init_sched(init_par->env_idx, clock, true);
            }
        }
        else
        {
            ASSERT_INFO_FORCE(0, init_par->state, init_par->env_idx);
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(LEINIT, EVT_CANCELED, 0);
}

/**
 ****************************************************************************************
 * @brief Perform packing of the CONNECT REQ structure
 ****************************************************************************************
 */
void lld_init_connect_req_pack(uint8_t *p_data_pack, struct pdu_con_req_lldata *p_data)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LLD_INIT_PATCH_TYPE, LLD_INIT_CONNECT_REQ_PACK_FUNC_BIT, p_data_pack, p_data);

    *p_data_pack++ = p_data->aa.addr[0];
    *p_data_pack++ = p_data->aa.addr[1];
    *p_data_pack++ = p_data->aa.addr[2];
    *p_data_pack++ = p_data->aa.addr[3];
    *p_data_pack++ = p_data->crcinit.crc[0];
    *p_data_pack++ = p_data->crcinit.crc[1];
    *p_data_pack++ = p_data->crcinit.crc[2];
    *p_data_pack++ = p_data->winsize;

    co_write16p(p_data_pack, co_htobs(p_data->winoffset));
    p_data_pack += 2;
    co_write16p(p_data_pack, co_htobs(p_data->interval));
    p_data_pack += 2;
    co_write16p(p_data_pack, co_htobs(p_data->latency));
    p_data_pack += 2;
    co_write16p(p_data_pack, co_htobs(p_data->timeout));
    p_data_pack += 2;

    *p_data_pack++ = p_data->ch_map.map[0];
    *p_data_pack++ = p_data->ch_map.map[1];
    *p_data_pack++ = p_data->ch_map.map[2];
    *p_data_pack++ = p_data->ch_map.map[3];
    *p_data_pack++ = p_data->ch_map.map[4];
    *p_data_pack = p_data->hop_sca;
}

/*
 * EXPORTED FUNCTIONS DEFINITION
 *****************************************************************************************
 */

void lld_init_init(bool is_reset)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_INIT_PATCH_TYPE, LLD_INIT_INIT_FUNC_BIT, is_reset);

    if (is_reset)
    {
        if (lld_init_env != NULL)
        {
            if (lld_init_env->ind)
            {
                // Free allocated message not sent
                ke_msg_free(ke_param2msg(lld_init_env->ind));
            }

            // Free activities for each scanning PHY
            for (int i = 0; i < MAX_SCAN_PHYS; i++)
            {
                if (lld_init_env->params[i])
                {
                    ke_free(lld_init_env->params[i]);
                }
            }

            // Free common event memory
            ke_free(lld_init_env);
            lld_init_env = NULL;
        }
    }
    else
    {
        lld_init_env = NULL;
    }
}


uint8_t lld_init_start(struct lld_init_params *params)
{
    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LLD_INIT_PATCH_TYPE, LLD_INIT_START_FUNC_BIT, uint8_t, params);

    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    struct lld_init_env_tag *init_env;
    struct lld_init_env_params *init_par;
    struct sch_arb_elt_tag *evt;

    uint8_t *data_packed[PDU_CON_REQ_LEN - (2 * BD_ADDR_LEN)];
    struct pdu_con_req_lldata data;

    uint8_t txdesc_idx = EM_BLE_TXDESC_INDEX(params->act_id, 0);
    uint16_t txdataptr = EM_BLE_AUXCONNECTREQTXBUF_OFFSET;

    uint32_t clock = lld_read_clock();
    uint32_t random_nb = 0xFFFFFF & co_rand_word();


    DBG_SWDIAG(LEINIT, START, 1);

    // Check if initiating is inactive
    if (lld_init_env == NULL)
    {
        // Allocate common environent
        lld_init_env = LLD_ALLOC_ENV(lld_init_env_tag);

        if (lld_init_env != NULL)
        {
            uint8_t init_idx = 0;
            uint8_t hop;
            uint16_t ral_ptr = 0;
            bool local_rpa_sel;
            bool ral_en = false;
            struct lld_init_con_params *con_par;

            init_env = lld_init_env;

            LLD_INIT_ENV(lld_init_env, lld_init_env_tag);

            // Initialize common initiating parameters

            init_env->act_id = params->act_id;
            init_env->own_addr_type = params->own_addr_type;
            init_env->peer_addr_type = params->peer_addr_type;
            init_env->filter_policy = params->filter_policy;

            memcpy(&init_env->own_addr.addr[0], &params->own_addr.addr[0], BD_ADDR_LEN);
            memcpy(&init_env->peer_addr.addr[0], &params->peer_addr.addr[0], BD_ADDR_LEN);
            memcpy(&init_env->ch_map.map[0], &params->ch_map.map[0], LE_CH_MAP_LEN);


            // Generate the access address
            lld_aa_gen(&init_env->aa.addr[0], params->act_id);

            // save the CRC init
            co_write16(&init_env->crcinit.crc[0], co_htobs(0x0000FFFF & random_nb));
            co_write8(&init_env->crcinit.crc[2], (0x00FF0000 & random_nb) >> 16);

            // compute & save the hopping interval (random number between 5 and 16)
            hop = 5 + CO_MOD(random_nb, 12);
            ASSERT_ERR((hop > 4) && (hop < 17));

            init_env->hop_inc = hop;

            init_env->m_sca = rwip_sca_get();

            /*
             *  Prepare the common parameters of CONNECT_IND PDU
             */

            // gets the access address
            memcpy(&data.aa.addr[0], &init_env->aa.addr[0], ACCESS_ADDR_LEN);

            // save the CRC init, winsize, hop_sca
            memcpy(&data.crcinit.crc[0], &init_env->crcinit.crc[0], CRC_INIT_LEN);

            // Init window size
            data.winsize = 2; // transmitWindowSize

            // WinOffset
            data.winoffset = 0; //left blank, filled by HW based on CS-WINOFFSET, interval,..

            // get the channel map
            memcpy(&data.ch_map.map[0], &params->ch_map.map[0], LE_CH_MAP_LEN);

            // gets the hop and sca
            data.hop_sca = hop | (init_env->m_sca << 5);

            /* Where the connection is made on a PHY whose bit is not set in the Initiating_PHYs parameter, the Controller shall use parameters
               for an implementation-chosen PHY whose bit is set in the Initiating_PHYs parameter.  */
            for (int i = 0; i < MAX_INIT_PHYS; i++)
            {
                con_par = &init_env->con_params[i];

                //Set default connection parameter values to those of the first set.
                con_par->intv = params->phy[0].con_intv;
                con_par->offset = params->phy[0].con_offset;
                con_par->latency = params->phy[0].con_latency;
                con_par->superv_to = params->phy[0].superv_to;
                con_par->phy_idx = 0;
            }

            // Initialize PHY-specific information for 1MBPS PHY

            con_par = &init_env->con_params[IDX_1MBPS];

            if (params->init_phys & PHY_1MBPS_BIT)
            {
                // Allocate 1M PHY environment
                init_env->params[LLD_PHY_1M_POS] = LLD_ALLOC_ENV(lld_init_env_params);

                if (init_env->params[LLD_PHY_1M_POS] != NULL)
                {
                    // Point to parameters
                    init_par = init_env->params[LLD_PHY_1M_POS];
                    evt = &(init_par->evt);

                    LLD_INIT_ENV(init_par, lld_init_env_params);

                    // Initialize event parameters (common part)
                    evt->cb_cancel           = &lld_init_evt_canceled_cbk;
                    evt->cb_start            = &lld_init_evt_start_cbk;
                    evt->current_prio        = rwip_priority[RWIP_PRIO_INIT_IDX].value;
                    evt->duration_min = co_min((2 * params->phy[init_idx].win * SLOT_SIZE), sch_slice_params.scan_evt_dur);

                    SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_ASAP_NO_LIMIT, SCH_ARB_NO_PHASE, 0, RWIP_PRIO_INC(RWIP_PRIO_INIT_IDX));

                    // Initialize event parameters (initiating part)
                    init_par->intv = params->phy[init_idx].intv;
                    init_par->win = params->phy[init_idx].win;
                    init_par->rate = CO_RATE_1MBPS; //00: 1Mbps - uncoded PHY
                    init_par->win_rem_slots = init_par->win;
                    init_par->env_idx = LLD_PHY_1M_POS;
                    init_par->tx_power = REP_ADV_DBM_UNKNOWN;
                    init_par->aux_offload = NO_AUX_OFFLOAD;

                    /* vol.6. part B LLS 4.5.3.
                     * The value of transmitWindowDelay shall be 1.25 ms when a CONNECT_IND PDU is used.
                     */
                    init_par->txwin_delay = 1;

                    init_par->cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(params->act_id);

                    // Connection parameters for the LE 1M PHY are provided.
                    con_par->intv = params->phy[init_idx].con_intv;
                    con_par->offset = params->phy[init_idx].con_offset;
                    con_par->latency = params->phy[init_idx].con_latency;
                    con_par->superv_to = params->phy[init_idx].superv_to;
                    con_par->phy_idx = init_idx;

                    // Indicate scanning PHY 1M is present
                    SETB(init_env->scan_phys, LLD_PHY_1M, 1);

                    init_idx++;
                }
                else
                {
                    status = CO_ERROR_MEMORY_CAPA_EXCEED;
                    ASSERT_ERR_FORCE(0);
                }
            }

            data.interval = co_htobs(con_par->intv); // Init interval
            data.latency = co_htobs(con_par->latency); // gets the latency
            data.timeout = co_htobs(con_par->superv_to); // gets the TO

            // Prepare the PDU to be copied in the EM
            lld_init_connect_req_pack((uint8_t *)&data_packed, &data);

            // Copy in the EM - CONNECT_REQ for 1MBPS at offset 0x00
            em_wr((void *)&data_packed, txdataptr + LLD_INIT_CONNECTREQ_1MBPS_OFFSET, PDU_CON_REQ_LEN - (2 * BD_ADDR_LEN));

            // Initialize PHY-specific information for 2MBPS PHY

            con_par = &init_env->con_params[IDX_2MBPS];

            if (params->init_phys & PHY_2MBPS_BIT)
            {
                // Connection parameters for the LE 2M PHY are provided.
                con_par->intv = params->phy[init_idx].con_intv;
                con_par->offset = params->phy[init_idx].con_offset;
                con_par->latency = params->phy[init_idx].con_latency;
                con_par->superv_to = params->phy[init_idx].superv_to;
                con_par->phy_idx = init_idx;

                init_idx++;
            }

            data.interval = co_htobs(con_par->intv); // Init interval
            data.latency = co_htobs(con_par->latency); // gets the latency
            data.timeout = co_htobs(con_par->superv_to); // gets the TO

            // Prepare the PDU to be copied in the EM
            lld_init_connect_req_pack((uint8_t *)&data_packed, &data);

            // Copy in the EM - CONNECT_REQ for 2MBPS at offset 0x22
            em_wr((void *)&data_packed, txdataptr + LLD_INIT_CONNECTREQ_2MBPS_OFFSET, PDU_CON_REQ_LEN - (2 * BD_ADDR_LEN));

            // Initialize PHY-specific information for CODED PHY

            con_par = &init_env->con_params[IDX_CODED];

            if (params->init_phys & PHY_CODED_BIT)
            {
                // Allocate coded PHY environment
                init_env->params[LLD_PHY_CODED_POS] = LLD_ALLOC_ENV(lld_init_env_params);

                if (init_env->params[LLD_PHY_CODED_POS] != NULL)
                {
                    // Point to parameters
                    init_par = init_env->params[LLD_PHY_CODED_POS];
                    evt = &(init_par->evt);

                    LLD_INIT_ENV(init_par, lld_init_env_params);

                    // Initialize event parameters (common part)
                    evt->cb_cancel           = &lld_init_evt_canceled_cbk;
                    evt->cb_start            = &lld_init_evt_start_cbk;
                    evt->current_prio        = rwip_priority[RWIP_PRIO_INIT_IDX].value;
                    evt->duration_min = co_min((2 * params->phy[init_idx].win * SLOT_SIZE), sch_slice_params.scan_evt_dur);

                    SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_ASAP_NO_LIMIT, SCH_ARB_NO_PHASE, 0, RWIP_PRIO_INC(RWIP_PRIO_INIT_IDX));

                    // Initialize event parameters (initiating part)
                    init_par->intv = params->phy[init_idx].intv;
                    init_par->win = params->phy[init_idx].win;
                    init_par->rate = CO_RATE_500KBPS; //1x: 125kbps or 500kbps - coded PHY
                    init_par->win_rem_slots = init_par->win;
                    init_par->env_idx = LLD_PHY_CODED_POS;
                    init_par->tx_power = REP_ADV_DBM_UNKNOWN;
                    init_par->aux_offload = NO_AUX_OFFLOAD;
                    init_par->state = INIT_EVT_WAIT;

                    /* vol.6. part B LLS 4.5.3.
                     * The value of transmitWindowDelay shall be 1.25 ms when a CONNECT_IND PDU is used.
                     */
                    init_par->txwin_delay = 1;

                    init_par->cs_idx = EM_BLE_CS_EXT_INIT_IDX2;

                    // Connection parameters for the LE Coded PHY are provided.
                    con_par->intv = params->phy[init_idx].con_intv;
                    con_par->offset = params->phy[init_idx].con_offset;
                    con_par->latency = params->phy[init_idx].con_latency;
                    con_par->superv_to = params->phy[init_idx].superv_to;
                    con_par->phy_idx = init_idx;

                    // Indicate scanning PHY 1M is present
                    SETB(init_env->scan_phys, LLD_PHY_CODED, 1);

                    init_idx++;
                }
                else
                {
                    status = CO_ERROR_MEMORY_CAPA_EXCEED;
                    ASSERT_ERR_FORCE(0);
                }
            }

            data.interval = co_htobs(con_par->intv); // Init interval
            data.latency = co_htobs(con_par->latency); // gets the latency
            data.timeout = co_htobs(con_par->superv_to); // gets the TO

            // Prepare the PDU to be copied in the EM
            lld_init_connect_req_pack((uint8_t *)&data_packed, &data);

            // Copy in the EM - CONNECT_REQ for 1MBPS at offset 0x44
            em_wr((void *)&data_packed, txdataptr + LLD_INIT_CONNECTREQ_CODED_OFFSET, PDU_CON_REQ_LEN - (2 * BD_ADDR_LEN));

            init_env->ext_init = params->ext_init;

            // Filter Policy - retrieve if peer device is present in resolving list
            if ((params->own_addr_type & ADDR_RPA_MASK) || (params->addr_resolution_en))
            {
                if (params->filter_policy == INIT_FILT_IGNORE_WLST)
                {
                    uint8_t ral_idx = lld_ral_search(&params->peer_addr, params->peer_addr_type);
                    if (ral_idx < BLE_RAL_MAX)
                    {
                        ral_ptr = REG_EM_ADDR_GET(BLE_RAL, ral_idx);
                        ral_en = true;
                    }
                }
                else
                {
                    ral_en = true;
                }
            }

            local_rpa_sel = ((params->own_addr_type & ADDR_RPA_MASK) != 0);
            init_env->ral_mode = (ral_ptr != 0);

            // Configure CS -

            for (uint8_t env_idx = 0; env_idx < MAX_SCAN_PHYS; env_idx++)
            {
                uint8_t cs_idx;

                if (init_env->params[env_idx] == NULL)
                    continue;

                // Point to parameters
                init_par = init_env->params[env_idx];
                cs_idx =  init_par->cs_idx;

                // Set permission/status of CS as R/W but uninitialized
                DBG_MEM_PERM_SET((const void *)(REG_EM_BLE_CS_BASE_ADDR + REG_EM_BLE_CS_ADDR_GET(cs_idx)), REG_EM_BLE_CS_SIZE, true, true, true);

                // set the TX power level for CONNECT_REQ
                em_ble_txrxcntl_set(cs_idx, rwip_rf.txpwr_max);

                // set connection interval of the BLE Link (multiple of 1.25ms)
                em_ble_conninterval_set(cs_idx, init_env->con_params[IDX_1MBPS].intv);

                // set connection interval of the BLE Link (multiple of 1.25ms)
                em_ble_conninterval_2m_set(cs_idx, init_env->con_params[IDX_2MBPS].intv);

                // set connection interval of the BLE Link (multiple of 1.25ms)
                em_ble_conninterval_lr_set(cs_idx, init_env->con_params[IDX_CODED].intv);

                // Pointer to the Tx Descriptor
                em_ble_acltxdescptr_set(cs_idx, REG_EM_ADDR_GET(BLE_TX_DESC, txdesc_idx));

                // Pointer to the Aux Tx Descriptor
                em_ble_auxtxdescptr_set(cs_idx, REG_EM_ADDR_GET(BLE_TX_DESC, txdesc_idx));

                // Configure the Control Structure
                em_ble_cntl_pack(cs_idx,
                                 RWIP_COEX_GET(INIT, TXBSY),
                                 RWIP_COEX_GET(INIT, RXBSY),
                                 RWIP_COEX_GET(INIT, DNABORT),
#if (EAVESDROPPING_SUPPORT)
                                 0,
#endif // EAVESDROPPING_SUPPORT
                                 ((params->ext_init) ? EM_BLE_CS_FMT_EXT_INITIATOR : EM_BLE_CS_FMT_INITIATOR));

                // set Synchronization Word
                em_ble_syncwl_set(cs_idx, LE_ADV_CH_ACC_ADDR_L);
                em_ble_syncwh_set(cs_idx, LE_ADV_CH_ACC_ADDR_H);
                // set CRC Initialization value
                em_ble_crcinit0_set(cs_idx, 0x5555);
                em_ble_crcinit1_pack(cs_idx, /*rxmaxctebuf*/ 0, /*crcinit1*/ 0x55);

                // initialize all extadv status
                em_ble_extadvstat_pack(cs_idx, 0, 0, 0, 0, 0);

                em_ble_filtpol_ralcntl_pack(cs_idx, /*filterpolicy*/params->filter_policy, /*ralresolen*/ params->addr_resolution_en,
                                            /*peradvfilten*/false,
                                            /*localrpasel*/local_rpa_sel, /*ralmode*/init_env->ral_mode, /*ralen*/ral_en);

                if (init_env->ral_mode)
                {
                    em_ble_peer_ralptr_setf(cs_idx, ral_ptr);
                }

                // check if a dedicated address is targeted
                if ((params->filter_policy == INIT_FILT_IGNORE_WLST) && (init_env->ral_mode == 0))
                {
                    // When the device is set as Initiator with Device filtering Policy set to 0x0, this field must be used in place of the White List
                    // Copy the advertiser address into the control structure
                    for (uint8_t i = 0; i < (sizeof(struct bd_addr) / 2); i++)
                    {
                        em_ble_adv_bd_addr_set(cs_idx, i, co_read16p(&params->peer_addr.addr[i * 2]));
                    }

                    // Copy the advertiser address type
                    em_ble_adv_bd_addr_type_set(cs_idx, params->peer_addr_type & ADDR_MASK);
                }

                // set Frequency Hopping, Channel Index
                em_ble_hopcntl_pack(cs_idx, /*fhen*/ 1, /*hopsel*/ 0, /*hopint*/ 0, /*chidx*/ 39);

                // Set Rx Max buf and Rx Max Time @0x0 -> v4.0 behavior
                em_ble_rxmaxbuf_set(cs_idx, 0x0);
                em_ble_rxmaxtime_set(cs_idx, 0x0);

                // Set link field
                em_ble_linkcntl_pack(cs_idx, /*hplpmode*/ 0, /* linklbl */ cs_idx, /*sas*/ false, /*nullrxllidflt*/true,
                                     /*micmode*/ENC_MIC_PRESENT, /*cryptmode*/ENC_MODE_PKT_PLD_CNT, /*txcrypten*/ false,
                                     /*rxcrypten*/ false, /*privnpub*/ ((params->own_addr_type & ADDR_MASK) != ADDR_PUBLIC));

                // Set the Device identity (BD Address)
                em_ble_lebdaddr_set(cs_idx, 0, co_read16p(&params->own_addr.addr[0]));
                em_ble_lebdaddr_set(cs_idx, 1, co_read16p(&params->own_addr.addr[2]));
                em_ble_lebdaddr_set(cs_idx, 2, co_read16p(&params->own_addr.addr[4]));

                // Set Rx/Tx threshold + rate
                em_ble_thrcntl_ratecntl_pack(cs_idx, /*rxthr*/LLD_INIT_RX_THRESHOLD, /*txthr*/0, /*auxrate*/0,
                                             /*rxrate*/ init_par->rate, /*txrate*/init_par->rate);

                em_ble_maxevtime_set(cs_idx, init_par->win_rem_slots);

                // set Wide-open mode, Size of the Rx half-window in half-slots
                em_ble_rxwincntl_pack(cs_idx, 1, init_par->win_rem_slots);

                // Disable unused control
                em_ble_chmap1_set(cs_idx, 0);
                em_ble_chmap2_set(cs_idx, 0);
                em_ble_evtcnt_setf(cs_idx, 0);
                em_ble_txheadercntl_set(cs_idx, 0);
            }

            // Configure TXDESC -

            // set Tx for BLE_CONNECT_IND
            em_ble_txphadv_txtype_setf(txdesc_idx, BLE_CONNECT_IND);
            // Set the TX data ptr
            em_ble_txdataptr_setf(txdesc_idx, txdataptr);
            // Set the length
            em_ble_txphadv_txadvlen_setf(txdesc_idx, PDU_CON_REQ_LEN);
            // Set nextptr, indicate end of chained list
            em_ble_txcntl_nextptr_setf(txdesc_idx, 0);
            // Fill the tx descriptor header
            em_ble_txphadv_pack(txdesc_idx,                            // index for the CONNECTION REQ descriptor
                                PDU_CON_REQ_LEN,                       // Length of the connect req
                                params->peer_addr_type & ADDR_MASK,    // txrxadd - peer address type
                                0,                                     // txtxadd - updated by HW
                                1,                                     // txchsel2 - channel selection algorithm #2 is supported
                                0,                                     // txadvrfu - Not set for the moment
                                BLE_AUX_CONNECT_REQ);                  // type of PDU
            // Release descriptor
            em_ble_txcntl_txdone_setf(txdesc_idx, 0);

            init_env->ind = NULL;

            GLOBAL_INT_DISABLE();

            // Schedule events ASAP
            for (uint8_t env_idx = 0; env_idx < MAX_SCAN_PHYS; env_idx++)
            {
                if (init_env->params[env_idx] == NULL)
                    continue;

                init_par = init_env->params[env_idx];
                evt = &(init_par->evt);

                // Schedule event ASAP
                evt->time.hs = clock;

                // Save the timestamp
                init_par->last_prio_upd_ts = evt->time.hs;

                if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
                {
                    init_par->pkt_rx = false;

                    // push next phy to be program just after current one
                    clock = CLK_ADD_2(clock, (init_par->win) << 1);
                }
                else
                {
                    ASSERT_ERR_FORCE(0);
                }

                status = CO_ERROR_NO_ERROR;
            }

            GLOBAL_INT_RESTORE();
        }
        else
        {
            status = CO_ERROR_MEMORY_CAPA_EXCEED;
            ASSERT_ERR_FORCE(0);
        }
    }

    DBG_SWDIAG(LEINIT, START, 0);

    return (status);
}

uint8_t lld_init_stop(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_HAVE_RETURN(LLD_INIT_PATCH_TYPE, LLD_INIT_STOP_FUNC_BIT, uint8_t);

    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    GLOBAL_INT_DISABLE();

    DBG_SWDIAG(LEINIT, STOP, 1);

    if (lld_init_env != NULL)
    {
        // Stop initiating
        lld_init_end();

        status = CO_ERROR_NO_ERROR;
    }

    DBG_SWDIAG(LEINIT, STOP, 0);

    GLOBAL_INT_RESTORE();

    return (status);
}

#if(BLE_HOST_PRESENT)
void lld_init_rand_addr_update(const struct bd_addr *p_addr)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_INIT_PATCH_TYPE, LLD_INIT_RAND_ADDR_UPDATE_FUNC_BIT, p_addr);

    GLOBAL_INT_DISABLE();

    if (lld_init_env != NULL)
    {
        lld_init_env->own_addr = *p_addr;
    }

    GLOBAL_INT_RESTORE();
}
#endif // (BLE_HOST_PRESENT)

#endif // (BLE_CENTRAL)

///@} LLDINIT
