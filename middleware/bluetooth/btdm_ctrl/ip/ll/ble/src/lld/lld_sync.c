/**
****************************************************************************************
*
* @file lld_sync.c
*
* @brief LLD Periodic Advertising RX source code
*
* Copyright (C) RivieraWaves 2009-2016
*
****************************************************************************************
*/

/**
 ****************************************************************************************
 * @addtogroup LLDSYNC
 * @ingroup LLD
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"         // stack configuration
#if (BLE_OBSERVER)

#include <string.h>

#include "co_math.h"
#include "co_utils.h"
#include "ble_util.h"            // BLE utility functions

#include "ke_mem.h"
#include "ke_task.h"             // kernel task management
#include "rwip.h"

#include "lld.h"                 // link driver API
#include "lld_int.h"             // link layer driver internal
#if (BLE_BIS)
    #include "lld_int_iso.h"         // Specific definition for BIS
#endif // (BLE_BIS)

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

#if BLE_CONLESS_CTE_RX
    #include "reg_em_ble_rx_cte_desc.h"
#endif // BLE_CONLESS_CTE_RX
#include "btdm_patch.h"


/*
 * DEFINES
 *****************************************************************************************
 */

/// SYNC OFFSET Offset Units in usecs
#define SYNC_OFFSET_UNIT_USECS(offset_units) ((offset_units)?300:30)

/// Calculate SYNC OFFSET in usecs from SyncInfo SYNC Offset and Offset Units fields
#define SYNC_OFFSET_USECS(sync_offset, offset_units) ((sync_offset) * ((offset_units)?300:30))

/// Scan Duration Units in half-slots from standard N * 10 ms units
#define SYNC_TIMEOUT_HS(sync_to)       ((uint32_t)(sync_to) * 32)

/// Minimum number of reception attempt to ensure for computation of maximum number of event that can be skipped
#define LLD_SYNC_MIN_RX_ATTEMPT      6


/*
 * ENUMERATION DEFINITION
 *****************************************************************************************
 */

/// Sync event states
enum SYNC_EVT_STATE
{
    SYNC_EVT_WAIT = 0,
    SYNC_EVT_ACTIVE,
    SYNC_EVT_END,
};

/// Scan AUX offload modes
enum SYNC_OFFLOAD_MODE
{
    NO_AUX_OFFLOAD = 0,
    SW_CNTL_AUX_OFFLOAD,
    HW_CNTL_AUX_OFFLOAD,
};


/*
 * CONSTANT DEFINITION
 ****************************************************************************************
 */

/// Table indicating the max duration of an AUX_SYNC_IND event (in us) depending on the PHY used
const uint16_t lld_sync_max_aux_dur_tab[CO_RATE_MAX] =
{
    [CO_RATE_1MBPS]   = PDU_1MBPS_LEN_US(PDU_ADV_PAYLOAD_LEN_MAX),
    [CO_RATE_2MBPS]   = PDU_2MBPS_LEN_US(PDU_ADV_PAYLOAD_LEN_MAX),
    [CO_RATE_500KBPS] = PDU_500KBPS_LEN_US(PDU_ADV_PAYLOAD_LEN_MAX), // 4542 us
    [CO_RATE_125KBPS] = PDU_125KBPS_LEN_US(PDU_ADV_PAYLOAD_LEN_MAX), // 17040 us
};


/*
 * STRUCTURE DEFINITION
 *****************************************************************************************
 */

/// LLD SYNC environment structure
struct lld_sync_env_tag
{
    /// Pointer to periodic scan event
    struct sch_arb_elt_tag evt;

    /// Scheduling information for next AUX channel
    struct lld_calc_aux_rx_out aux_rx_out;

#if BLE_CONLESS_CTE_RX
    // Periodic advertising CTE Rx parameters
    struct lld_sync_cte_params cte_params;
#endif // BLE_CONLESS_CTE_RX

    /// Sync timeout (half slots)
    uint32_t sync_to;

    /// Timestamp of most recent sync
    uint32_t last_sync_ts;

    /// Timestamp of most recent successful receive packet
    uint32_t last_pkt_ok_ts;

    /// Scan interval in half slots
    uint32_t intv;

#if (BLE_BIS)
    /// Drift accumulated between expected sync time and effective sync time (in half us)
    int32_t  sync_drift_acc;
#endif // (BLE_BIS)

    /// Reference timestamp for the periodic sync (can be updated when an anchor packet is received or new anchor reception is scheduled)
    uint32_t anchor_ts;
    /// Reference bit offset of the anchor
    int16_t  anchor_bit_off;
    /// Reference Event counter linked to the anchor timestamp
    uint16_t evt_counter;


    /// Number of periodic advertising event that can be skipped
    uint16_t skip;
    /// Used to know when skip event is over
    uint16_t skip_end_evt_cnt;

    /// Additional drift to consider on given time reference, for the 1st reception attempt (in half-us)
    uint16_t add_drift;

    /// Channel map update instant
    uint16_t ch_map_instant;

    /// Lengths of the previous pkt chain payloads (max size if unknown) (in bytes)
    uint16_t rx_aux_pld_len[1 + BLE_ADV_FRAG_NB_RX_MAX];

    /// Aggregate length of reception
    uint16_t rx_chain_cnt_bytes;

    /// Chain packet count
    uint8_t rx_chain_cnt_pkt;

#if BLE_CONLESS_CTE_RX
    /// Indicates whether the CTE Rx parameters need to be updated
    bool cte_update;
#endif // BLE_CONLESS_CTE_RX

    ///  Activity Identifier
    uint8_t act_id;

    /// Offset units
    uint8_t offset_units;

    /// Indicates if sync is established
    bool sync_established;

    /// Rate information  (@see enum lld_rate)
    uint8_t rate;

    /// Advertiser clock accuracy ((@see enum SCA))
    uint8_t adv_ca;

    /// Sync activity state (@see enum  SYNC_EVT_STATE)
    uint8_t state;

    /// Channel map to use for the periodic scan
    struct le_ch_map ch_map;

    /// Advertiser address
    struct bd_addr adv_addr;

    /// Channel map update pending
    bool ch_map_update_pending;

    /// Data Status
    uint8_t data_status;

    /// Indicates offload to secondary PHY under software/hardware control (@see SYNC_OFFLOAD_MODE)
    uint8_t aux_offload;

    /// Indicates a packet rx count in this event
    uint8_t pkt_rx;

    /// Specifies whether to only synchronize to periodic advertising with certain types of CTE (@see enum sync_cte_type)
    uint8_t sync_cte_type;

#if (BT_53)
    /// ADI of latest received data
    uint16_t adi_data;

    /// ADI filtering enable
    bool adi_filt_en;
#endif // (BT_53)

    /// Filter data mode active
    bool filt_data;

    /// Periodic advertising reporting enable
    bool rep_en;

    /// Indicates that the periodic advertising has the wrong type of CTE
    bool wrong_cte_type;
};

/*
 * VARIABLE DEFINITION
 *****************************************************************************************
 */


/// Periodic scanning event parameters
__STATIC struct lld_sync_env_tag *lld_sync_env[BLE_ACTIVITY_MAX];


/*
 * LOCAL FUNCTIONS DEFINITION
 *****************************************************************************************
 */


/**
 ****************************************************************************************
 * @brief Cleanup sync environment variable
 *
 * @param[in] act_id      Activity identifier
 * @param[in] reason      Used to know disconnection reason
 ****************************************************************************************
 */
__STATIC void lld_sync_cleanup(uint8_t act_id, uint8_t status)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LLD_SYNC_PATCH_TYPE, LLD_SYNC_CLEANUP_FUNC_BIT, act_id, status);

    if (lld_sync_env[act_id] != NULL)
    {
        struct lld_per_adv_rx_end_ind *ind = KE_MSG_ALLOC(LLD_PER_ADV_RX_END_IND, TASK_LLM, TASK_NONE, lld_per_adv_rx_end_ind);
        ind->act_id = act_id;
        ind->status = status;
        ke_msg_send(ind);

        // Remove permission/status of CS as now unused
        DBG_MEM_PERM_SET((const void *)(REG_EM_BLE_CS_BASE_ADDR + REG_EM_BLE_CS_ADDR_GET(EM_BLE_CS_ACT_ID_TO_INDEX(act_id))), REG_EM_BLE_CS_SIZE, false, false, false);

        // Free event memory
        ke_free(lld_sync_env[act_id]);
        lld_sync_env[act_id] = NULL;
    }
}

/**
 ****************************************************************************************
 * @brief Generate truncated report
 ****************************************************************************************
 */
__STATIC void lld_sync_trunc_ind(uint8_t act_id)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_SYNC_PATCH_TYPE, LLD_SYNC_TRUNC_IND_FUNC_BIT, act_id);

    // Incomplete advertisement, generate a truncated report
    struct lld_per_adv_rep_ind *ind = KE_MSG_ALLOC(LLD_PER_ADV_REP_IND, TASK_LLM, TASK_NONE, lld_per_adv_rep_ind);

    ind->act_id       = act_id;
    ind->data_len     = 0;
    ind->rssi         = REP_ADV_DBM_UNKNOWN;
    ind->tx_power     = REP_ADV_DBM_UNKNOWN;
    ind->data_status  = PER_ADV_EVT_DATA_STATUS_TRUNCATED;
    ind->cte_type     = CTE_TYPE_NO_CTE;
    ind->acad_len     = 0;
    ind->filt_data    = 0;
    ind->act_offset   = lld_sync_env[act_id]->intv;

    ke_msg_send(ind);
}

/**
 ****************************************************************************************
 * @brief Process LE scan activity
 ****************************************************************************************
 */
__STATIC void lld_sync_process_pkt_rx(uint8_t act_id, uint32_t timestamp)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LLD_SYNC_PATCH_TYPE, LLD_SYNC_PROCESS_PKT_RX_FUNC_BIT, act_id, timestamp);

    DBG_SWDIAG(LESYNC, PKT_RX, 1);

    struct lld_sync_env_tag *sync_env = lld_sync_env[act_id];

    // Process rx packet if done
    while (lld_rxdesc_check(EM_BLE_CS_ACT_ID_TO_INDEX(act_id)))
    {
        uint16_t rxstat_pkt;

        // Flag whether this is an AUX_SYNC_IND
        bool aux_sync = (NO_AUX_OFFLOAD == sync_env->aux_offload);

        // Get current RX descriptor index
        uint8_t rxdesc_idx = lld_env.curr_rxdesc_index;

        // Get RSSI related data
        uint8_t rx_rssi = em_ble_rxchass_rssi_getf(rxdesc_idx);
        uint8_t rx_ch_idx = em_ble_rxchass_used_ch_idx_getf(rxdesc_idx);
        uint8_t rx_status = RWIP_RX_OTHER_ERROR;

#if BLE_CONLESS_CTE_RX
        // No CTE Rx indication has been sent yet
        bool cte_rx_ind_sent = false;
#endif // BLE_CONLESS_CTE_RX

        // Trace the current RX descriptor
        TRC_REQ_RX_DESC(LLD_SCAN, act_id, REG_EM_BLE_RX_DESC_ADDR_GET(rxdesc_idx));

        // Retrieve RX status and type
        rxstat_pkt = em_ble_rxstatadv_get(rxdesc_idx);

        // Check if chain should be flushed
        if ((sync_env->rx_chain_cnt_bytes > HOST_ADV_DATA_LEN_MAX) || sync_env->filt_data)
        {
            // Determine RX status
            rx_status = ((rxstat_pkt & LLD_ADV_ERR_MASK) == 0) ? RWIP_RX_OK : (rxstat_pkt & EM_BLE_SYNC_ERR_BIT) ? RWIP_RX_NO_SYNC :
                        (rxstat_pkt & EM_BLE_CRC_ERR_BIT) ? RWIP_RX_CRC_ERROR : RWIP_RX_OTHER_ERROR;

            // Provide reception information for channel assessment
            rwip_channel_assess_ble(rx_ch_idx, rx_status, rx_rssi, timestamp, true);

            // Free RX descriptor
            lld_rxdesc_free();
            continue;
        }

        // *** Synchronize slave after correct reception on an AUX_SYNC_IND instant
        if (aux_sync && ((rxstat_pkt & EM_BLE_SYNC_ERR_BIT) == 0))
        {
            // Read clock value where sync has been found
            uint32_t base_cnt = (em_ble_rxclknsync1_clknrxsync1_getf(rxdesc_idx) << 16)
                                | em_ble_rxclknsync0_clknrxsync0_getf(rxdesc_idx);

            // Read bit position where sync has been found
            uint16_t fine_cnt = LLD_FINECNT_MAX - em_ble_rxfcntsync_fcntrxsync_getf(rxdesc_idx);

            // Compute new sync timestamp
            uint32_t new_sync_ts = base_cnt;

            int16_t new_bit_off = fine_cnt - 2 * lld_exp_sync_pos_tab[sync_env->rate];
            ASSERT_ERR((new_bit_off > (-2 * HALF_SLOT_SIZE)) && (new_bit_off < HALF_SLOT_SIZE));

            // Adjust bit offset and sync timestamp if needed
            if (new_bit_off < 0)
            {
                uint8_t num_hs = CO_DIVIDE_CEIL((-new_bit_off), HALF_SLOT_SIZE);
                new_bit_off += num_hs * HALF_SLOT_SIZE;
                new_sync_ts = CLK_SUB(new_sync_ts, num_hs);
            }

#if (BLE_BIS)
            // Timing adjustment between BIG and Sync is performed only if Sync already established
            if (sync_env->sync_established)
            {
                // Add the delta time between expected packet start and effective synchronization time
                sync_env->sync_drift_acc += (CLK_DIFF(sync_env->anchor_ts, new_sync_ts) * HALF_SLOT_SIZE) + (new_bit_off - sync_env->anchor_bit_off);
                // Update BIG if present
                lld_bi_sync_time_update(act_id, base_cnt, sync_env->sync_drift_acc);
            }
#endif // (BLE_BIS)

            sync_env->anchor_ts      = new_sync_ts;
            sync_env->anchor_bit_off = new_bit_off;
            sync_env->last_sync_ts   = base_cnt;
            sync_env->add_drift = 0;
            sync_env->rx_chain_cnt_bytes = 0;
            sync_env->rx_chain_cnt_pkt = 0;
            sync_env->filt_data = 0;
        }

        if (!aux_sync)
        {
            // increment chain packet count (note - this can span multiple SW offload events)
            sync_env->rx_chain_cnt_pkt++;
        }

        // Check if packet reception is correct
        if ((rxstat_pkt & LLD_ADV_ERR_MASK) == 0)
        {
            uint8_t rxaelength, rxadvlen;
            uint16_t rxaeheader;
            uint16_t rxdataptr;
            uint8_t acad_len = 0;
            uint8_t acad_offset = 0;
            uint8_t tx_power = 0;
            uint8_t cte_type = CTE_TYPE_NO_CTE;
            uint8_t data_len;

            // The packet should be either AUX_SYNC_IND or AUX_CHAIN_IND.
            ASSERT_ERR(BLE_ADV_EXT_IND == em_ble_rxphadv_rxtype_getf(rxdesc_idx));

            // Consider channel as good
            rx_status = RWIP_RX_OK;

            // Store anchor when last valid packet has been received
            sync_env->last_pkt_ok_ts = sync_env->anchor_ts;

            // Flag that a packet has been received in this event
            sync_env->pkt_rx++;

            // Check if reporting is enabled at AUX_SYNC_IND receptions
            if (aux_sync && !sync_env->rep_en)
            {
                // Do not process data, and filter the chain. Proceed only to check for potential ACAD data.
                sync_env->filt_data = true;

                // Abort HW Follow
                ble_rwblecntl_scan_abort_setf(1);
            }

            // Fetch rx dataptr
            rxdataptr = em_ble_rxdataptr_get(rxdesc_idx);

            // Process AE header length, determine data offset & length
            rxaelength = em_ble_rxaeheader_rxaelength_getf(rxdesc_idx);
            rxadvlen = em_ble_rxphadv_rxadvlen_getf(rxdesc_idx);
            ASSERT_ERR(rxaelength < rxadvlen);

            data_len = rxadvlen - rxaelength - BLE_EXT_ADV_PRE_HEADER_LEN;

            if (aux_sync || (sync_env->rx_chain_cnt_pkt <= BLE_ADV_FRAG_NB_RX_MAX))
            {
                // Store the number of bytes received in payload
                sync_env->rx_aux_pld_len[sync_env->rx_chain_cnt_pkt] = rxadvlen;
            }

            rxaeheader = em_ble_rxaeheader_get(rxdesc_idx);

            if (rxaelength > 0)
            {
                int i = 0;

                if (rxaeheader & EM_BLE_RXADVA_BIT) // AdvA
                {
                    // This field is reserved for future use for SYNC/CHAIN - Ignore field when received.
                    i += BD_ADDR_LEN;
                }

                if (rxaeheader & EM_BLE_RXTGTA_BIT) // TargetA
                {
                    // This field is reserved for future use for SYNC/CHAIN - Ignore field when received.
                    i += BD_ADDR_LEN;
                }

                if (rxaeheader & EM_BLE_RXCTE_BIT) // CTEInfo
                {
                    // Save CTE Information
                    uint8_t cte_info = em_rd8p(rxdataptr + i);
                    cte_type = GETF(cte_info, BLE_CTE_INFO_CTE_TYPE);

                    i += BLE_EXT_CTE_INFO_LEN;
                }

                // If the periodic advertiser changes the type of Constant Tone Extension after the
                // scanner has synchronized with the periodic advertising, the scanner's Link
                // Layer shall remain synchronized
                if (!sync_env->sync_established)
                {
                    // Check packet has correct CTE type
                    if (((sync_env->sync_cte_type & NO_SYNC_AOA_BIT) && (cte_type == CTE_TYPE_AOA))
                            || ((sync_env->sync_cte_type & NO_SYNC_AOD_1US_BIT) && (cte_type == CTE_TYPE_AOD_1US))
                            || ((sync_env->sync_cte_type & NO_SYNC_AOD_2US_BIT) && (cte_type == CTE_TYPE_AOD_2US))
                            || ((sync_env->sync_cte_type & NO_SYNC_NO_CTE_BIT) && (cte_type == CTE_TYPE_NO_CTE)))
                    {
                        sync_env->wrong_cte_type = true;
                    }
                }

                if (rxaeheader & EM_BLE_RXADI_BIT)// AdvDataInfo
                {
#if (BT_53)
                    if (!sync_env->filt_data)
                    {
                        // Periodic Advertising ADI filtering
                        uint16_t adi_data = em_rd16p(rxdataptr + i);

                        if (aux_sync)
                        {
                            // The SID subfield shall have the same value as that in the superior PDU of this PDU
                            // The ADI field shall not have same value if duplicate filtering enabled
                            if ((GETF(adi_data, BLE_ADI_SID) != GETF(sync_env->adi_data, BLE_ADI_SID))
                                    || (sync_env->adi_filt_en && sync_env->sync_established && (sync_env->adi_data == adi_data)))
                            {
                                // Do not process data, and filter the chain. Proceed only to check for potential ACAD data.
                                sync_env->filt_data = true;

                                // Abort HW Follow
                                ble_rwblecntl_scan_abort_setf(1);
                            }
                            else
                            {
                                // Save latest ADI
                                sync_env->adi_data = adi_data;
                            }
                        }
                        // The ADI field shall have same value as the field in the superior PDU of this PDU
                        else if (sync_env->adi_data != adi_data)
                        {
                            // ADI mismatch. Do not process data, and filter the chain. Proceed to report as Truncated Data.
                            sync_env->filt_data = true;
                            sync_env->aux_offload = NO_AUX_OFFLOAD;

                            // Abort HW Follow
                            ble_rwblecntl_scan_abort_setf(1);
                        }
                    }
#endif // (BT_53)

                    // This field is unused prior to BT5.3 on SYNC/CHAIN - Ignore field when received.
                    i += BLE_EXT_ADI_LEN;
                }

                if (rxaeheader & EM_BLE_RXAUXPTR_BIT) // AuxPtr
                {
                    if (!sync_env->filt_data)
                    {
                        uint32_t aux_data;
                        uint8_t aux_phy;
                        bool phy_support;

                        em_rd((void *)&aux_data, rxdataptr + i, BLE_EXT_AUX_PTR_LEN);

                        aux_phy = (aux_data & BLE_AUX_PHY_MASK) >> BLE_AUX_PHY_LSB;

                        /* Check if PHY supported. AUX PHY will be same on all chained PDUs, so sufficient to just
                           check on PHY support in the initial ADV_EXT_IND aux_data. */
                        phy_support = ((BLE_PHY_1MBPS_SUPPORT) && (AUX_PHY_1MBPS == aux_phy))
                                      || ((BLE_PHY_2MBPS_SUPPORT) && (AUX_PHY_2MBPS == aux_phy))
                                      || ((BLE_PHY_CODED_SUPPORT) && (AUX_PHY_CODED == aux_phy));

                        /* we can use FOLLOW_ADI for indication of the HW follow auxptr (regardless of ADI is present
                         * or not). If this bit is not set, HW will stop the event and let the RW-BLE Software reschedule
                         *   chained packet later on a new event.
                         */
                        if (0 == em_ble_rxstatadv_followauxptr_getf(rxdesc_idx))
                        {
                            if (phy_support &&
#if (BLE_ADV_FRAG_NB_RX_MAX)
                                    (sync_env->rx_chain_cnt_pkt < BLE_ADV_FRAG_NB_RX_MAX) &&
#endif //(BLE_ADV_FRAG_NB_RX_MAX)
                                    lld_calc_aux_rx(&sync_env->aux_rx_out, rxdesc_idx, aux_data))
                            {
                                // Calculated AUX RX activity for Software reschedule
                                sync_env->aux_offload = SW_CNTL_AUX_OFFLOAD;
                                sync_env->data_status = PER_ADV_EVT_DATA_STATUS_INCOMPLETE;
                            }
                            else
                            {
                                // No chained packet will follow, indicate as truncated.
                                sync_env->aux_offload = NO_AUX_OFFLOAD;
                                sync_env->data_status = PER_ADV_EVT_DATA_STATUS_TRUNCATED;
                            }
                        }
                        else
                        {
                            if (phy_support)
                            {
                                // Indicate mode is HW follow
                                sync_env->aux_offload = HW_CNTL_AUX_OFFLOAD;
                                sync_env->data_status = PER_ADV_EVT_DATA_STATUS_INCOMPLETE;
                            }
                            else
                            {
                                // No chained packet will follow, indicate as truncated.
                                sync_env->aux_offload = NO_AUX_OFFLOAD;
                                sync_env->data_status = PER_ADV_EVT_DATA_STATUS_TRUNCATED;
                            }
                        }
                    }

                    i += BLE_EXT_AUX_PTR_LEN;
                }
                else
                {
                    sync_env->aux_offload = NO_AUX_OFFLOAD;

                    // No following Data - so advertising report will complete with this PDU
                    sync_env->data_status = PER_ADV_EVT_DATA_STATUS_COMPLETE;
                }

                if (rxaeheader & EM_BLE_RXSYNC_BIT) // SyncInfo
                {
                    // This field is reserved for future use for SYNC/CHAIN - Ignore field when received.
                    i += BLE_EXT_SYNC_LEN;
                }

                if (rxaeheader & EM_BLE_RXPOW_BIT) // TxPower
                {
                    // Tx Power Level - save to advertising report.
                    tx_power = em_rd8p(rxdataptr + i);
                    i += BLE_EXT_TX_PWR_LEN;
                }

                if (aux_sync) // AUX_SYNC_IND - Check ACAD Field - This field is reserved for future use on AUX_CHAIN_IND
                {
                    // The remainder of the extended header forms the ACAD Field - save to advertising report.
                    acad_len = rxaelength - i - BLE_EXT_ADV_HEADER_FLAGS_LEN;
                    acad_offset = i; // if acad_len non-zero, this will provide the offset.
                }
            }
            else
            {
                sync_env->aux_offload = NO_AUX_OFFLOAD;

                // No following Data - so advertising report will complete with this PDU
                sync_env->data_status = PER_ADV_EVT_DATA_STATUS_COMPLETE;
            }

            // LLM not notified if: Wrong CTE type, or data filtered and no other reason to notify LLM.
            if (!sync_env->wrong_cte_type && !(sync_env->filt_data && aux_sync && sync_env->sync_established && !acad_len))
            {
                // Report periodic advertising reception
                struct lld_per_adv_rep_ind *ind = KE_MSG_ALLOC(LLD_PER_ADV_REP_IND, TASK_LLM, TASK_NONE, lld_per_adv_rep_ind);

                ind->act_id      = act_id;
                ind->ref_evt_cnt = sync_env->evt_counter;
                ind->tx_power    = tx_power;
                ind->rssi        = rwip_rf.rssi_convert(rx_rssi) - lld_env.rx_path_comp / 10;
                ind->cte_type    = cte_type;
                ind->acad_len    = acad_len;
                ind->acad_offset = acad_offset;

                // If this is first sync, report RX sync established to LLM
                if (!sync_env->sync_established)
                {
                    sync_env->sync_established = true;
                    ind->rate        = sync_env->rate;
                    ind->interval    = sync_env->intv >> 2;
                    ind->adv_ca      = sync_env->adv_ca;
                }

                /*
                 * Report the activity offset of the latest successfully received AUX_SYNC_IND
                 * If not AUX_SYNC_IND, report the interval instead (to be discarded by LLM)
                 */
                ind->act_offset  = (aux_sync) ? CO_MOD(sync_env->anchor_ts, sync_env->intv) : sync_env->intv;

                // Indicate if data filtered. Exception is chained data, where a truncated event is reported.
                ind->filt_data = sync_env->filt_data && aux_sync;
                if (sync_env->filt_data)
                {
                    // Report filtered continuation chains as truncated data
                    ind->data_len = 0;
                    ind->data_status = PER_ADV_EVT_DATA_STATUS_TRUNCATED;
                }
                else
                {
                    // Shall not exceed Host Advertising Data maximum - Truncate as required for Host.
                    sync_env->rx_chain_cnt_bytes += data_len;
                    if (sync_env->rx_chain_cnt_bytes > HOST_ADV_DATA_LEN_MAX)
                    {
                        sync_env->data_status = PER_ADV_EVT_DATA_STATUS_TRUNCATED;
                        data_len -= (sync_env->rx_chain_cnt_bytes - HOST_ADV_DATA_LEN_MAX);

                        sync_env->aux_offload = NO_AUX_OFFLOAD;
                    }

                    // Set data_len, status, with adv_data_len to copy
                    ind->data_len = data_len;
                    ind->data_status = sync_env->data_status;
                }

                if (ind->data_len || ind->acad_len)
                {
                    // Set dataptr to copy data from
                    ind->em_buf = rxdataptr;
                    ind->data_offset = (rxaelength) ? (rxaelength - BLE_EXT_ADV_HEADER_FLAGS_LEN) : (0);

                    // Clear RX data pointer
                    em_ble_rxdataptr_setf(rxdesc_idx, 0);
                }

                ke_msg_send(ind);
            }
            else
            {
                // Do not follow AUX_CHAIN_IND by SW
                if (sync_env->aux_offload == SW_CNTL_AUX_OFFLOAD)
                {
                    sync_env->aux_offload = NO_AUX_OFFLOAD;
                }
            }

#if BLE_CONLESS_CTE_RX
            if (sync_env->cte_params.sampl_en)
            {
                uint8_t nbrxiqsamp = em_ble_rxphcte_nbrxiqsamp_getf(rxdesc_idx);
                uint16_t rxcteptr = em_ble_rxcteptr_getf(rxdesc_idx);

                if ((rxcteptr != 0) && (nbrxiqsamp != 0))
                {
                    ASSERT_INFO(rxcteptr >= EM_BLE_RX_CTE_DESC_OFFSET, rxcteptr, EM_BLE_RX_CTE_DESC_OFFSET);
                    ASSERT_INFO(rxcteptr <  EM_BLE_RX_CTE_DESC_END,    rxcteptr, EM_BLE_RX_CTE_DESC_OFFSET);

                    // Get RX CTE descriptor index
                    uint8_t em_rx_cte_desc_idx = (rxcteptr - EM_BLE_RX_CTE_DESC_OFFSET) / REG_EM_BLE_RX_CTE_DESC_SIZE;

                    // Report CTE reception
                    struct lld_conless_cte_rx_ind *msg = KE_MSG_ALLOC(LLD_CONLESS_CTE_RX_IND, TASK_LLM, TASK_NONE, lld_conless_cte_rx_ind);

                    msg->act_id = act_id;
                    msg->em_rx_cte_desc_idx = em_rx_cte_desc_idx;
                    msg->channel_idx = rx_ch_idx;
                    msg->rssi = 10 * rwip_rf.rssi_convert(rx_rssi) - lld_env.rx_path_comp; // Convert to 0.1 dBm
                    msg->rssi_antenna_id = 0;
                    msg->cte_type = cte_type;
                    msg->slot_dur = sync_env->cte_params.slot_dur;
                    msg->sample_cnt = nbrxiqsamp;
                    msg->pa_evt_cnt = sync_env->evt_counter;

                    ke_msg_send(msg);

                    cte_rx_ind_sent = true;
                }
            }
#endif // BLE_CONLESS_CTE_RX
        }
        else // bad reception
        {
            // remainder of chain from the transmitter to unknown
            for (int i = sync_env->rx_chain_cnt_pkt; i <= BLE_ADV_FRAG_NB_RX_MAX; i++)
            {
                sync_env->rx_aux_pld_len[i] = PDU_ADV_PAYLOAD_LEN_MAX;
            }

            // Determine reason for bad reception
            rx_status = (rxstat_pkt & EM_BLE_SYNC_ERR_BIT) ? RWIP_RX_NO_SYNC : (rxstat_pkt & EM_BLE_CRC_ERR_BIT) ? RWIP_RX_CRC_ERROR : RWIP_RX_OTHER_ERROR;
        }

#if BLE_CONLESS_CTE_RX
        if (!cte_rx_ind_sent)
        {
            uint16_t rxcteptr = em_ble_rxcteptr_getf(rxdesc_idx);
            if (rxcteptr != 0)
            {
                ASSERT_INFO(rxcteptr >= EM_BLE_RX_CTE_DESC_OFFSET, rxcteptr, EM_BLE_RX_CTE_DESC_OFFSET);
                ASSERT_INFO(rxcteptr <  EM_BLE_RX_CTE_DESC_END,    rxcteptr, EM_BLE_RX_CTE_DESC_OFFSET);

                // Clear RX done bit
                uint16_t em_rx_cte_desc_idx = (rxcteptr - EM_BLE_RX_CTE_DESC_OFFSET) / REG_EM_BLE_RX_CTE_DESC_SIZE;
                em_ble_rxctecntl_rxdone_setf(em_rx_cte_desc_idx, 0);
            }
        }
#endif // BLE_CONLESS_CTE_RX

        // Provide reception information for channel assessment
        rwip_channel_assess_ble(rx_ch_idx, rx_status, rx_rssi, timestamp, true);

        // Free RX descriptor
        lld_rxdesc_free();
    }

    DBG_SWDIAG(LESYNC, PKT_RX, 0);
}

#if BLE_CONLESS_CTE_RX
/**
 ****************************************************************************************
 * @brief Configure antenna switching and enable/disable CTE reception
 ****************************************************************************************
 */
__STATIC void lld_sync_ant_switch_config(uint8_t act_id)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_SYNC_PATCH_TYPE, LLD_SYNC_ANT_SWITCH_CONFIG_FUNC_BIT, act_id);

    // Point to parameters
    struct lld_sync_env_tag *sync_env = lld_sync_env[act_id];

    uint8_t cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(act_id);

    // Check if CTE is enabled
    if (sync_env->cte_params.sampl_en)
    {
        // Set the Antenna ID pointer
        em_ble_rxdfantswptr_set(cs_idx, EM_BLE_RX_ANTENNA_ID_OFFSET >> 2);

        // Write antenna IDs to EM
        em_wr(&sync_env->cte_params.antenna_id[0], EM_BLE_RX_ANTENNA_ID_OFFSET, sync_env->cte_params.switching_pattern_len);

        // Set the length of the switching pattern
        em_ble_rxdfantpattcntl_rx_ant_patt_length_setf(cs_idx, sync_env->cte_params.switching_pattern_len);

        // max I&Q samples per CTE
        em_ble_crcinit1_rxmaxctebuf_setf(cs_idx, LLD_MAX_CTE_IQ_SAMPLES);

        // max sampled CTE per event
        if (sync_env->cte_params.max_sampl_cte != 0)
            em_ble_rxdfantpattcntl_max_samp_cte_setf(cs_idx, sync_env->cte_params.max_sampl_cte);
        else
            em_ble_rxdfantpattcntl_max_samp_cte_setf(cs_idx, 0x1F); // 0x1F - no limit

        // Set RXDFCNTL indicating that both AoA and AoD are supported
        em_ble_rxdfcntl_pack(cs_idx, /*DFRSPEN*/ 0, /*DFSWCNTL*/ sync_env->cte_params.slot_dur, /*DFSAMPCNTL*/ sync_env->cte_params.slot_dur, /*DFTYPE*/ 0x03, /*DFFILTEREN*/ 0, /*DFEN*/ 1);
    }
    else
    {
        // Disable antenna switching
        em_ble_rxdfantpattcntl_rx_ant_patt_length_setf(cs_idx, 0);
        // Disable CTE reception
        em_ble_rxdfcntl_dfen_setf(cs_idx, 0);
    }
}
#endif // BLE_CONLESS_CTE_RX

/**
 ****************************************************************************************
 * @brief Schedule next LE scan activity
 ****************************************************************************************
 */
__STATIC void lld_sync_sched(uint8_t act_id, bool resched)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LLD_SYNC_PATCH_TYPE, LLD_SYNC_SCHED_FUNC_BIT, act_id, resched);

    // Point to parameters
    struct lld_sync_env_tag *sync_env = lld_sync_env[act_id];
    struct sch_arb_elt_tag *evt = &(sync_env->evt);
    bool elt_inserted = false;

    uint8_t cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(act_id);

    DBG_SWDIAG(LESYNC, SCHED, 1);

    while (!elt_inserted)
    {
        uint32_t sched_ts;
        int32_t  new_bit_off;
        uint32_t sync_win_size_us;
        uint8_t  rate;
        uint8_t prio_idx = (sync_env->sync_established) ? RWIP_PRIO_PER_ADV_RX_DFT_IDX : RWIP_PRIO_PER_ADV_RX_ESTAB_IDX;
        uint32_t pkt_duration_us;

        // update activity priority according to success or not of the scheduling
        if (resched)
        {
            evt->current_prio = RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(prio_idx));
        }
        else
        {
            evt->current_prio = rwip_priority[prio_idx].value;
        }

        if (SW_CNTL_AUX_OFFLOAD == sync_env->aux_offload) // SW controlled offload to secondary channel
        {
            // Configure window size as calculated for AUX offload
            sync_win_size_us = sync_env->aux_rx_out.sync_win_size_us;
            rate             = sync_env->aux_rx_out.rate;

            // Offset and timestamp for AUX offload
            new_bit_off = sync_env->aux_rx_out.time.hus;
            sched_ts    = sync_env->aux_rx_out.time.hs;

            // Get duration of last reception of this pdu
#if BLE_ADV_FRAG_NB_RX_MAX
            ASSERT_ERR(sync_env->rx_chain_cnt_pkt < BLE_ADV_FRAG_NB_RX_MAX);
            pkt_duration_us = ble_util_pkt_dur_in_us(sync_env->rx_aux_pld_len[1 + sync_env->rx_chain_cnt_pkt], rate);
#else // !BLE_ADV_FRAG_NB_RX_MAX
            pkt_duration_us = lld_sync_max_aux_dur_tab[rate];
#endif // !BLE_ADV_FRAG_NB_RX_MAX
        }
        // normal periodic sync
        else
        {
            int16_t evt_inc;
            uint32_t timeout;
            int32_t  clk_diff;

            // check if it is possible to skip some events
            if ((sync_env->pkt_rx > 0) && (PER_ADV_EVT_DATA_STATUS_COMPLETE == sync_env->data_status) && (sync_env->skip != 0))
            {
                // First scheduling
                if (!resched)
                {
                    evt_inc                    = sync_env->skip + 1;
                    sync_env->skip_end_evt_cnt = sync_env->evt_counter + evt_inc;
                }
                // max skip slot insertion rejected
                else
                {
                    // try to schedule before max skip
                    evt_inc = -1;
                }
            }
            else
            {
                // schedule next event
                evt_inc = 1;
            }

            // compute next reference timestamp - and reference counter
            sync_env->anchor_ts = CLK_ADD_2(sync_env->anchor_ts, evt_inc * sync_env->intv);
            sync_env->evt_counter += evt_inc;

            // check that next timestamp is not in the past
            clk_diff = CLK_DIFF(sync_env->anchor_ts, lld_read_clock() + rwip_prog_delay * 2);
            if (clk_diff > 0)
            {
                // check that we are  trying to go backward in skipping and we cannot go further in the past
                if (!BLE_UTIL_INSTANT_PASSED(sync_env->skip_end_evt_cnt, sync_env->evt_counter))
                {
                    // ensure that we increment event counter just after the event skip
                    evt_inc = sync_env->skip_end_evt_cnt - sync_env->evt_counter + 1;
                    // ensure that scheduler will not go backward
                    sync_env->pkt_rx = 0;
                }
                else
                {
                    // compute how many increment in the future must be performed
                    evt_inc = (clk_diff + sync_env->intv - 1) / sync_env->intv;
                }

                // compute next reference timestamp - and reference counter
                sync_env->anchor_ts = CLK_ADD_2(sync_env->anchor_ts, evt_inc * sync_env->intv);
                // update reference event counter
                sync_env->evt_counter += evt_inc;
            }

            // Check synchronization timeout
            timeout = sync_env->sync_established ? sync_env->sync_to : 6 * sync_env->intv;
            if (CLK_SUB(sync_env->anchor_ts, sync_env->last_pkt_ok_ts) > timeout)
            {
                // Cleanup and report error
                lld_sync_cleanup(act_id, CO_ERROR_CON_TIMEOUT);
                break;
            }

            sched_ts    = sync_env->anchor_ts;
            new_bit_off = sync_env->anchor_bit_off;

            // Compute RX timings
            sync_win_size_us = lld_rx_timing_compute(sync_env->last_sync_ts, &sched_ts, &new_bit_off, co_sca2ppm[sync_env->adv_ca], sync_env->rate, sync_env->add_drift) >> 1;

            // If sync not yet established, offset unit is appended to the sync window
            if (!sync_env->sync_established)
            {
                sync_win_size_us += SYNC_OFFSET_UNIT_USECS(sync_env->offset_units);
            }

            // Check if the windowWidening reaches ((periodicInterval/2) - T_IFS us) in magnitude while synchronizing
            if (!sync_env->sync_established && (sync_win_size_us / 2 > ((sync_env->intv * HALF_SLOT_SIZE / 2) - BLE_IFS_DUR)))
            {
                // Cleanup and report error
                lld_sync_cleanup(act_id, CO_ERROR_CON_TIMEOUT);
                break;
            }

            // Reset aggregate data length
            sync_env->rx_chain_cnt_bytes = 0;
            sync_env->rx_chain_cnt_pkt = 0;
            sync_env->filt_data = 0;

            rate = sync_env->rate;

            // Get duration of last reception of this PDU
            pkt_duration_us = ble_util_pkt_dur_in_us(sync_env->rx_aux_pld_len[0], rate);
        }

        // Set the duration for arbiter + half window size + margin
        evt->duration_min = co_min(SCH_ARB_MAX_DURATION, ((pkt_duration_us << 1) + sync_win_size_us + BLE_RESERVATION_TIME_MARGIN_HUS));

        // Prepare the scheduling with computed start timestamp
        evt->time.hs = sched_ts;
        evt->time.hus     = new_bit_off;

        if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
        {
            uint16_t max_evt_slots;

            // Set Rx/Tx threshold + rate
            em_ble_thrcntl_ratecntl_pack(cs_idx, /*rxthr*/1, /*txthr*/0, /*auxrate*/rate,
                                         /*rxrate*/rate, /*txrate*/0);

            if (SW_CNTL_AUX_OFFLOAD == sync_env->aux_offload) // SW controlled offload to secondary channel
            {
                // set Channel Index, no Hopping.
                em_ble_hopcntl_pack(cs_idx, /*fhen*/false,/*hopmode*/ LLD_HOP_MODE_CHAN_SEL_2, /*hopint*/ 0, /* chidx */sync_env->aux_rx_out.ch_idx);

                // Set the max AUX CHAIN Rx Byte
                em_ble_rxmaxauxchain_maxrxchbyte_setf(cs_idx, /*maxrxchbyte*/ HOST_ADV_DATA_LEN_MAX - sync_env->rx_chain_cnt_bytes);

                // set Channel Index for Aux.
                em_ble_chmap2_ch_aux_setf(cs_idx, sync_env->aux_rx_out.ch_idx);
            }
            else
            {
                // Check if a channel map update instant is reached or over-stepped
                if (sync_env->ch_map_update_pending && BLE_UTIL_INSTANT_PASSED(sync_env->ch_map_instant, sync_env->evt_counter))
                {
                    // Configure the new channel map
                    em_ble_chmap0_llchmap0_setf(cs_idx, co_read16p(&sync_env->ch_map.map[0]));
                    em_ble_chmap1_llchmap1_setf(cs_idx, co_read16p(&sync_env->ch_map.map[2]));
                    em_ble_chmap2_llchmap2_setf(cs_idx, sync_env->ch_map.map[4] & 0x1F);
                    sync_env->ch_map_update_pending = false;
                }

                // Set hop control
                em_ble_hopcntl_pack(cs_idx, /*fhen*/ true, /*hopmode*/ LLD_HOP_MODE_CHAN_SEL_2, /*hopint*/ 0, /*chidx*/ 0);

                // Set the max AUX CHAIN Rx Byte and Rx Desc
                em_ble_rxmaxauxchain_pack(cs_idx, /*maxrxchdesc*/ BLE_ADV_FRAG_NB_RX_MAX, /*maxrxchbyte*/ HOST_ADV_DATA_LEN_MAX);

                // initialize all extadv status as start of new sequence
                em_ble_extadvstat_pack(cs_idx, 0, 0, 0, 0, 0);
            }

            // Set max event slots based on rate and window size
            max_evt_slots = CO_DIVIDE_CEIL((lld_sync_max_aux_dur_tab[rate] + sync_win_size_us), SLOT_SIZE);
            em_ble_maxevtime_set(cs_idx, max_evt_slots);

            // Set Wide-open mode, Size of the Rx window in slots
            if (sync_win_size_us > EM_BLE_CS_RXWINSZ_MAX)
            {
                uint16_t sync_win_slots = (sync_win_size_us + (SLOT_SIZE - 1)) / SLOT_SIZE;
                em_ble_rxwincntl_pack(cs_idx, 1, sync_win_slots);
            }
            else
            {
                // Size of the Rx half window in us (normal mode)
                em_ble_rxwincntl_pack(cs_idx, 0, (sync_win_size_us + 1) >> 1);
            }

            // Update event counter
            em_ble_evtcnt_setf(cs_idx, sync_env->evt_counter);

            sync_env->state = SYNC_EVT_WAIT;
            elt_inserted = true;
        }
        else
        {
            resched = true;

            if (SW_CNTL_AUX_OFFLOAD == sync_env->aux_offload)
            {
                // Unable to schedule chained data, generate a truncated report
                lld_sync_trunc_ind(act_id);
                sync_env->aux_offload = NO_AUX_OFFLOAD;
            }
        }
    }

    DBG_SWDIAG(LESYNC, SCHED, 0);
}

/**
 ****************************************************************************************
 * @brief Handle frame interrupt
 ****************************************************************************************
 */
__STATIC void lld_sync_frm_eof_isr(uint8_t act_id, uint32_t timestamp, bool abort)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LLD_SYNC_PATCH_TYPE, LLD_SYNC_FRM_EOF_ISR_FUNC_BIT, act_id, timestamp, abort);

    DBG_SWDIAG(LESYNC, FRM_EOF_ISR, 1);


    if (lld_sync_env[act_id] != NULL)
    {
        // Point to parameters
        struct lld_sync_env_tag *sync_env = lld_sync_env[act_id];
        struct sch_arb_elt_tag *evt = &(sync_env->evt);
        uint8_t cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(sync_env->act_id);

        // Remove event
        sch_arb_remove(evt, true);

#if BLE_CONLESS_CTE_RX
        // Reset the value of max_sampl_cte in the CS because this is overwritten by the HW
        if (sync_env->cte_params.sampl_en && (sync_env->cte_params.max_sampl_cte != 0))
        {
            em_ble_rxdfantpattcntl_max_samp_cte_setf(cs_idx, sync_env->cte_params.max_sampl_cte);
        }
#endif // BLE_CONLESS_CTE_RX

        // Check for T_MAFS error prior to Process packet rx
        if (em_ble_txrxcntl_rxmafserr_getf(cs_idx))
        {
            em_ble_txrxcntl_rxmafserr_setf(cs_idx, 0);

            if (PER_ADV_EVT_DATA_STATUS_INCOMPLETE == sync_env->data_status)
            {
                // Incomplete advertisement, generate a truncated report
                lld_sync_trunc_ind(act_id);
            }

            sync_env->aux_offload = NO_AUX_OFFLOAD;
        }

        // Process packet rx
        lld_sync_process_pkt_rx(act_id, timestamp);

        // Check for processing status after Process packet rx
        //1) We need to send a truncated report if a HW follow was still pending at EOF.
        //2) We need to send a truncated report if a SW offload has failed. This is identified by pkt_rx = 0 and still in the SW offload state.
        //3) In both cases, we need to check for data_status, as we should not send truncated reports if we have only received an ADV_EXT_IND, for example.
        if (((HW_CNTL_AUX_OFFLOAD == sync_env->aux_offload) || (!sync_env->pkt_rx && (SW_CNTL_AUX_OFFLOAD == sync_env->aux_offload)))
                && (PER_ADV_EVT_DATA_STATUS_INCOMPLETE == sync_env->data_status))
        {
            // Incomplete advertisement, generate a truncated report
            lld_sync_trunc_ind(act_id);

            sync_env->aux_offload = NO_AUX_OFFLOAD;
        }

        if (SYNC_EVT_END == sync_env->state)
        {
            lld_sync_cleanup(act_id, CO_ERROR_NO_ERROR);
        }
        else if (sync_env->wrong_cte_type)
        {
            lld_sync_cleanup(act_id, CO_ERROR_UNSUPPORTED_REMOTE_FEATURE);
        }
        else
        {
#if BLE_CONLESS_CTE_RX
            // Check if the host has updated the CTE Rx parameters
            if (sync_env->cte_update)
            {
                // Set the CTE Rx parameters
                lld_sync_ant_switch_config(act_id);
                sync_env->cte_update = false;
            }
#endif // BLE_CONLESS_CTE_RX

            // Reschedule according to the current mode
            lld_sync_sched(act_id, abort);
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(LESYNC, FRM_EOF_ISR, 0);
}

/**
 ****************************************************************************************
 * @brief Handle RX interrupt
 ****************************************************************************************
 */
__STATIC void lld_sync_frm_rx_isr(uint8_t act_id, uint32_t timestamp)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LLD_SYNC_PATCH_TYPE, LLD_SYNC_FRM_RX_ISR_FUNC_BIT, act_id, timestamp);

    DBG_SWDIAG(LESYNC, FRM_RX_ISR, 1);

    if ((lld_sync_env[act_id] != NULL))
    {
        // Process packet rx - handle received descriptors (threshold reached)
        lld_sync_process_pkt_rx(act_id, timestamp);
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(LESYNC, FRM_RX_ISR, 0);
}

/**
 ****************************************************************************************
 * @brief Handle skip interrupt
 ****************************************************************************************
 */
__STATIC void lld_sync_frm_skip_isr(uint8_t act_id)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_SYNC_PATCH_TYPE, LLD_SYNC_FRM_SKIP_ISR_FUNC_BIT, act_id);

    DBG_SWDIAG(LESYNC, EVT_CANCELED, 1);

    if ((lld_sync_env[act_id] != NULL))
    {
        // Point to parameters
        struct lld_sync_env_tag *sync_env = lld_sync_env[act_id];

        // Remove event
        sch_arb_remove(&sync_env->evt, true);

        if (SW_CNTL_AUX_OFFLOAD == sync_env->aux_offload)
        {
            // Unable to schedule chained data, generate a truncated report
            lld_sync_trunc_ind(act_id);
            sync_env->aux_offload = NO_AUX_OFFLOAD;
        }

        if (SYNC_EVT_END == sync_env->state)
        {
            lld_sync_cleanup(sync_env->act_id, CO_ERROR_NO_ERROR);
        }
        else
        {
            // Reschedule according to the current mode
            lld_sync_sched(sync_env->act_id, true);
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(LESYNC, EVT_CANCELED, 0);
}

/**
 ****************************************************************************************
 * @brief Handle frame interrupt notification
 ****************************************************************************************
 */
__STATIC void lld_sync_frm_cbk(uint32_t timestamp, uint32_t dummy, uint8_t irq_type)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LLD_SYNC_PATCH_TYPE, LLD_SYNC_FRM_CBK_FUNC_BIT, timestamp, dummy, irq_type);

    ASSERT_INFO(dummy < BLE_ACTIVITY_MAX, dummy, irq_type);

    switch (irq_type)
    {
    case SCH_FRAME_IRQ_EOF:
    {
        lld_sync_frm_eof_isr(dummy, timestamp, false);
    }
    break;
    case SCH_FRAME_IRQ_EOF_ABORT_UNDER_PRIO:
    case SCH_FRAME_IRQ_EOF_ABORT_AFTER_PRIO:
    {
        lld_sync_frm_eof_isr(dummy, timestamp, true);
    }
    break;
    case SCH_FRAME_IRQ_RX:
    {
        lld_sync_frm_rx_isr(dummy, timestamp);
    }
    break;
    case SCH_FRAME_IRQ_SKIP:
    {
        lld_sync_frm_skip_isr(dummy);
    }
    break;
    default:
    {
        ASSERT_INFO_FORCE(0, dummy, irq_type);
    }
    break;
    }
}

/**
****************************************************************************************
* @brief Handle sync event start notification
****************************************************************************************
*/
__STATIC void lld_sync_evt_start_cbk(struct sch_arb_elt_tag *evt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_SYNC_PATCH_TYPE, LLD_SYNC_EVT_START_CBK_FUNC_BIT, evt);

    DBG_SWDIAG(LESYNC, EVT_START, 1);

    if (evt != NULL)
    {
        // Point to parameters
        struct lld_sync_env_tag *sync_env = (struct lld_sync_env_tag *) evt;
        uint8_t cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(sync_env->act_id);

        // Push the programming to SCH PROG
        struct sch_prog_params prog_par;
        prog_par.frm_cbk        = &lld_sync_frm_cbk;
        prog_par.time.hs        = evt->time.hs;
        prog_par.time.hus       = evt->time.hus;
        prog_par.cs_idx         = cs_idx;
        prog_par.dummy          = sync_env->act_id;
        prog_par.bandwidth      = evt->duration_min;
        prog_par.prio_1         = evt->current_prio;
        prog_par.prio_2         = evt->current_prio;
        prog_par.prio_3         = evt->current_prio;
        prog_par.pti_prio       = RW_BLE_PTI_PRIO_AUTO;
        prog_par.add.ble.ae_nps = 1;
        prog_par.add.ble.iso    = 0;
        prog_par.mode           = SCH_PROG_BLE;
        sch_prog_push(&prog_par);

        // Move state
        sync_env->state  = SYNC_EVT_ACTIVE;
        sync_env->pkt_rx = 0;
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(LESYNC, EVT_START, 0);
}

/**
****************************************************************************************
* @brief Handle sync event canceled notification
****************************************************************************************
*/
__STATIC void lld_sync_evt_canceled_cbk(struct sch_arb_elt_tag *evt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_SYNC_PATCH_TYPE, LLD_SYNC_EVT_CANCELED_CBK_FUNC_BIT, evt);

    DBG_SWDIAG(LESYNC, EVT_CANCELED, 1);

    if (evt != NULL)
    {
        // Point to parameters
        struct lld_sync_env_tag *sync_env = (struct lld_sync_env_tag *) evt;

        ASSERT_INFO(((struct lld_sync_env_tag *) evt)->state == SYNC_EVT_WAIT, ((struct lld_sync_env_tag *) evt)->state, SYNC_EVT_WAIT);

        if (SW_CNTL_AUX_OFFLOAD == sync_env->aux_offload)
        {
            // Unable to schedule chained data, generate a truncated report
            lld_sync_trunc_ind(sync_env->act_id);
            sync_env->aux_offload = NO_AUX_OFFLOAD;
        }

        if (SYNC_EVT_END == sync_env->state)
        {
            lld_sync_cleanup(sync_env->act_id, CO_ERROR_NO_ERROR);
        }
        else
        {
            // Reschedule according to the current mode
            lld_sync_sched(sync_env->act_id, true);
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(LESYNC, EVT_CANCELED, 0);
}


/*
 * EXPORTED FUNCTIONS DEFINITION
 *****************************************************************************************
 */

void lld_sync_init(bool is_reset)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_SYNC_PATCH_TYPE, LLD_SYNC_INIT_FUNC_BIT, is_reset);

    if (is_reset)
    {
        for (int8_t act_id = (BLE_ACTIVITY_MAX - 1); act_id >= 0 ; act_id--)
        {
            // Check if periodic synchronization is ongoing
            if (lld_sync_env[act_id] != NULL)
            {
                // Free event memory
                ke_free(lld_sync_env[act_id]);
            }
        }
    }

    // Initialize environment
    memset(&lld_sync_env, 0, sizeof(lld_sync_env));
}

uint8_t lld_sync_start(uint8_t act_id, struct lld_sync_params *params)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLD_SYNC_PATCH_TYPE, LLD_SYNC_START_FUNC_BIT, uint8_t, act_id, params);
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;
    DBG_SWDIAG(LESYNC, START, 1);

    if ((act_id < BLE_ACTIVITY_MAX) && (NULL == lld_sync_env[act_id]))
    {
        // Allocate coded PHY environment
        lld_sync_env[act_id] = LLD_ALLOC_ENV(lld_sync_env_tag);

        if (lld_sync_env[act_id] != NULL)
        {
            // Point to parameters
            struct lld_sync_env_tag *sync_env = lld_sync_env[act_id];
            struct sch_arb_elt_tag *evt = &(sync_env->evt);
            const struct sync_info *p_syncinfo = params->p_syncinfo;
            int16_t max_skip;

            uint8_t cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(act_id);

            // The SYNC packet does not start any earlier than the Sync Packet Offset and starts no later than the Offset plus one Offset Unit.
            uint32_t sync_offset_us = SYNC_OFFSET_USECS(p_syncinfo->sync_offset, p_syncinfo->offset_units); // calculate offset (us)
            // Calculate total bit offset in half us
            uint32_t bit_offset = params->fine_cnt + (sync_offset_us * 2);


            // The AUX_SYNC_IND PDUs and the PDUs that   point to them shall always be sent on the same PHY.
            uint8_t rx_rate = params->rate;


            LLD_INIT_ENV(sync_env, lld_sync_env_tag);

            // Initialize event parameters (common part)
            evt->cb_cancel     = &lld_sync_evt_canceled_cbk;
            evt->cb_start      = &lld_sync_evt_start_cbk;

            SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_NO_ASAP, SCH_ARB_NO_PHASE, 0, RWIP_PRIO_INC(RWIP_PRIO_PER_ADV_RX_DFT_IDX));

            // Store parameters required for periodic sync
            sync_env->act_id           = act_id;
            sync_env->sync_to          = SYNC_TIMEOUT_HS(params->sync_to);
            sync_env->intv             = p_syncinfo->interval << 2;
            memcpy(&sync_env->ch_map.map[0], &p_syncinfo->ch_map.map[0], LE_CH_MAP_LEN);
            sync_env->rate             = rx_rate;
            sync_env->adv_ca           = p_syncinfo->sca;
            sync_env->last_sync_ts     = params->base_cnt;
            sync_env->offset_units     = p_syncinfo->offset_units;
            sync_env->sync_cte_type    = params->sync_cte_type;
            sync_env->adv_addr         = params->adv_addr;

            // Set anchor one interval (and counter value) in the past to be sure that first event scheduled will be expected one
            sync_env->anchor_ts        = CLK_SUB(CLK_ADD_2(params->base_cnt, bit_offset / HALF_SLOT_SIZE), sync_env->intv);
            sync_env->last_pkt_ok_ts   = sync_env->anchor_ts;
            sync_env->anchor_bit_off   = CO_MOD(bit_offset, HALF_SLOT_SIZE);
            sync_env->evt_counter      = (p_syncinfo->evt_counter - 1);
            sync_env->skip_end_evt_cnt = sync_env->evt_counter;
            sync_env->add_drift        = params->add_drift;

            sync_env->rep_en           = params->rep_en;
#if (BT_53)
            sync_env->adi_filt_en      = params->adi_filt_en;
            sync_env->adi_data         = params->adv_sid << BLE_ADI_SID_LSB;
#endif // (BT_53)

            sync_env->aux_offload      = NO_AUX_OFFLOAD;
            for (int i = 0; i <= BLE_ADV_FRAG_NB_RX_MAX; i++)
            {
                sync_env->rx_aux_pld_len[i] = PDU_ADV_PAYLOAD_LEN_MAX;
            }

            max_skip = (sync_env->sync_to / sync_env->intv) - LLD_SYNC_MIN_RX_ATTEMPT;
            if (max_skip < 0)
            {
                max_skip = 0;
            }
            sync_env->skip         = co_min(params->skip, max_skip);

            // Configure EM

            // Set permission/status of CS as R/W but uninitialized
            DBG_MEM_PERM_SET((const void *)(REG_EM_BLE_CS_BASE_ADDR + REG_EM_BLE_CS_ADDR_GET(cs_idx)), REG_EM_BLE_CS_SIZE, true, true, true);

            // set Synchronization Word
            em_ble_syncwl_set(cs_idx, (p_syncinfo->aa.addr[1] << 8) | p_syncinfo->aa.addr[0]);
            em_ble_syncwh_set(cs_idx, (p_syncinfo->aa.addr[3] << 8) | p_syncinfo->aa.addr[2]);
            // set CRC Initialization value
            em_ble_crcinit0_set(cs_idx, (p_syncinfo->crcinit.crc[1] << 8) | p_syncinfo->crcinit.crc[0]);
            em_ble_crcinit1_pack(cs_idx, /*rxmaxctebuf*/ 0, /*crcinit1*/ p_syncinfo->crcinit.crc[2]);

            // clear Filter policy configurations
            em_ble_filtpol_ralcntl_pack(cs_idx, 0, 0, 0, 0, 0, 0);

            // Set the max AUX CHAIN Rx Byte and Rx Desc
            em_ble_rxmaxauxchain_pack(cs_idx, /*maxrxchdesc*/ BLE_ADV_FRAG_NB_RX_MAX, /*maxrxchbyte*/ HOST_ADV_DATA_LEN_MAX);

            em_ble_cntl_pack(cs_idx,
                             RWIP_COEX_GET(SCAN, TXBSY),
                             RWIP_COEX_GET(SCAN, RXBSY),
                             RWIP_COEX_GET(SCAN, DNABORT),
#if (EAVESDROPPING_SUPPORT)
                             1,
#endif // EAVESDROPPING_SUPPORT
                             EM_BLE_CS_FMT_EXT_PASSIVE_SCAN);

            // Set channel map, hop control & event counter
            em_ble_chmap0_llchmap0_setf(cs_idx, co_read16p(&sync_env->ch_map.map[0]));
            em_ble_chmap1_llchmap1_setf(cs_idx, co_read16p(&sync_env->ch_map.map[2]));
            em_ble_chmap2_pack(cs_idx, /*chaux*/ 0, /*advchmap*/ 0, /*llchmap2*/ sync_env->ch_map.map[4]);

            // Clear txrxcntl
            em_ble_txrxcntl_set(cs_idx, 0);

            // Set link field
            em_ble_linkcntl_pack(cs_idx, /*hplpmode*/ 0, /*linklbl*/ cs_idx, /*sas*/ false, /*nullrxllidflt*/true,
                                 /*micmode*/ENC_MIC_PRESENT, /*cryptmode*/ENC_MODE_PKT_PLD_CNT,
                                 /*txcrypten*/ false, /*rxcrypten*/ false, /*privnpub*/ false);

            // Set the event counter offset fields to 0
            em_ble_evtcnt_offset0_set(cs_idx, 0x0000);
            em_ble_evtcnt_offset1_set(cs_idx, 0x0000);
            em_ble_evtcnt_offset2_set(cs_idx, 0x0000);

            // Disable antenna switching
            em_ble_rxdfantpattcntl_set(cs_idx, 0);
            // Disable CTE reception
            em_ble_rxdfcntl_set(cs_idx, 0);

            // Disable unused control
            em_ble_lebdaddr_set(cs_idx, 0, 0);
            em_ble_lebdaddr_set(cs_idx, 1, 0);
            em_ble_lebdaddr_set(cs_idx, 2, 0);
            em_ble_acltxdescptr_set(cs_idx, 0);
            em_ble_minevtime_set(cs_idx, 0);
            em_ble_txheadercntl_set(cs_idx, 0);

            GLOBAL_INT_DISABLE();
            lld_sync_sched(act_id, false);
            GLOBAL_INT_RESTORE();

            status = CO_ERROR_NO_ERROR;
        }
        else
        {
            status = CO_ERROR_MEMORY_CAPA_EXCEED;
            ASSERT_ERR_FORCE(0);
        }
    }

    DBG_SWDIAG(LESYNC, START, 0);

    return status;
}

uint8_t lld_sync_ch_map_update(uint8_t act_id, struct le_ch_map *p_ch_map, uint16_t instant)
{
    FUNC_PATCH_ENTRY_3_PARAM_HAVE_RETURN(LLD_SYNC_PATCH_TYPE, LLD_SYNC_CH_MAP_UPDATE_FUNC_BIT, uint8_t, act_id, p_ch_map, instant);

    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    GLOBAL_INT_DISABLE();

    if (lld_sync_env[act_id] != NULL)
    {
        // Point to parameters
        struct lld_sync_env_tag *sync_env = lld_sync_env[act_id];

        // Store new parameters
        sync_env->ch_map_instant = instant;
        memcpy(&sync_env->ch_map.map[0], p_ch_map, LE_CH_MAP_LEN);

        sync_env->ch_map_update_pending = true;

        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return (status);
}

bool lld_sync_rep_filt_en(uint8_t act_id, uint8_t en)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLD_SYNC_PATCH_TYPE, LLD_SYNC_REP_FILT_EN_FUNC_BIT, bool, act_id, en);
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    GLOBAL_INT_DISABLE();

    if (lld_sync_env[act_id] != NULL)
    {
        // Point to parameters
        struct lld_sync_env_tag *sync_env = lld_sync_env[act_id];
        // Set reporting mode
        sync_env->rep_en = GETB(en, PER_ADV_REC_EN);
#if (BT_53)
        // Set ADI filtering mode
        sync_env->adi_filt_en = GETB(en, PER_ADV_DUPLIC_FILT_EN);
#endif // (BT_53)
        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return (status);
}

uint8_t lld_sync_stop(uint8_t act_id)
{
    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LLD_SYNC_PATCH_TYPE, LLD_SYNC_STOP_FUNC_BIT, uint8_t, act_id);

    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    DBG_SWDIAG(LESYNC, STOP, 1);

    GLOBAL_INT_DISABLE();

    if (lld_sync_env[act_id] != NULL)
    {
        // Point to parameters
        struct lld_sync_env_tag *sync_env = lld_sync_env[act_id];
        struct sch_arb_elt_tag *evt = &(sync_env->evt);

        switch (sync_env->state)
        {
        case SYNC_EVT_WAIT:
        {
            // Remove event
            sch_arb_remove(evt, false);

            // Cleanup
            lld_sync_cleanup(act_id, CO_ERROR_NO_ERROR);
        }
        break;

        case SYNC_EVT_ACTIVE:
        {
            // Move state
            sync_env->state = SYNC_EVT_END;
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

    DBG_SWDIAG(LESYNC, STOP, 0);

    return (status);
}

#if (BLE_BIS)
uint8_t lld_sync_info_for_big_get(uint8_t act_id, uint16_t evt_cnt, struct lld_sync_info_for_big *info)
{
    FUNC_PATCH_ENTRY_3_PARAM_HAVE_RETURN(LLD_SYNC_PATCH_TYPE, LLD_SYNC_INFO_FOR_BIG_GET_FUNC_BIT, uint8_t, act_id, evt_cnt, info);

    uint8_t status = CO_ERROR_UNKNOWN_CONNECTION_ID;

    // Point to parameters
    struct lld_sync_env_tag *sync_env = lld_sync_env[act_id];

    if (sync_env != NULL)
    {
        uint32_t sync_interval  = 0;
        uint32_t sync_timestamp = 0;
        uint16_t sync_evt_cnt   = 0;
        int16_t  evt_cnt_diff;

        sync_evt_cnt          = sync_env->evt_counter;
        sync_interval         = sync_env->intv;
        sync_timestamp        = sync_env->anchor_ts;

        // Compute connection timestamp according to provided connection event counter
        evt_cnt_diff         = BLE_UTIL_EVT_CNT_DIFF(evt_cnt, sync_evt_cnt);
        info->ref_timestamp  = CLK_ADD_2(sync_timestamp, evt_cnt_diff * sync_interval);

        info->last_sync_ts   = sync_env->last_sync_ts;
        info->ref_bit_off    = sync_env->anchor_bit_off;
        info->sync_drift_acc = sync_env->sync_drift_acc;
        info->ca             = sync_env->adv_ca;

        status = CO_ERROR_NO_ERROR;
    }

    return (status);
}

void lld_sync_sync_time_update(uint8_t act_id, uint32_t last_sync_ts, int32_t sync_drift_acc)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LLD_SYNC_PATCH_TYPE, LLD_SYNC_SYNC_TIME_UPDATE_FUNC_BIT, act_id, last_sync_ts, sync_drift_acc);

    // Point to parameters
    struct lld_sync_env_tag *sync_env = lld_sync_env[act_id];

    // Check that connection exists
    if (sync_env != NULL)
    {
        // Timing adjustment between BIG and Sync is performed only if Sync already established
        if (sync_env->sync_established)
        {
            // Get drift since last drift value
            int32_t drift_val = sync_drift_acc - sync_env->sync_drift_acc;
            // Anchor timestamp - half microseconds part
            int32_t new_bit_off = sync_env->anchor_bit_off + drift_val;

            // Update event position
            if (new_bit_off < 0)
            {
                new_bit_off = -new_bit_off - 1;
                sync_env->anchor_ts = CLK_SUB(sync_env->anchor_ts, 1 + (new_bit_off / HALF_SLOT_SIZE));
                sync_env->anchor_bit_off = HALF_SLOT_SIZE - 1 - CO_MOD(new_bit_off, HALF_SLOT_SIZE);
            }
            else
            {
                sync_env->anchor_ts = CLK_ADD_2(sync_env->anchor_ts, new_bit_off / HALF_SLOT_SIZE);
                sync_env->anchor_bit_off = CO_MOD(new_bit_off, HALF_SLOT_SIZE);
            }

            sync_env->last_sync_ts = last_sync_ts;

            // If event is scheduled, update timings, assuming the sync window reduces thanks to new synchronization infos
            if (sync_env->state == SYNC_EVT_WAIT)
            {
                // Packet duration in usecs
                uint32_t pkt_duration_us;
                // Get event parameters
                struct sch_arb_elt_tag *p_evt = &sync_env->evt;

                // Expected next anchor point (half microsecond part)
                int32_t bit_off = sync_env->anchor_bit_off;
                // Expected next anchor point (half slot part)
                uint32_t target_clock = sync_env->anchor_ts;

                // Compute RX timings
                uint32_t rx_win_size = lld_rx_timing_compute(sync_env->last_sync_ts, &target_clock, &bit_off, co_sca2ppm[sync_env->adv_ca], sync_env->rate, sync_env->add_drift);

                // Update the activity arbitration element
                p_evt->time.hs      = target_clock;
                p_evt->time.hus     = bit_off;

                // Check if next reception is an anchor point or a chain
                if (SW_CNTL_AUX_OFFLOAD == sync_env->aux_offload)
                {
#if BLE_ADV_FRAG_NB_RX_MAX
                    ASSERT_ERR(sync_env->rx_chain_cnt_pkt < BLE_ADV_FRAG_NB_RX_MAX);
                    pkt_duration_us = ble_util_pkt_dur_in_us(sync_env->rx_aux_pld_len[1 + sync_env->rx_chain_cnt_pkt], sync_env->aux_rx_out.rate);
#else // !BLE_ADV_FRAG_NB_RX_MAX
                    pkt_duration_us = lld_sync_max_aux_dur_tab[sync_env->aux_rx_out.rate];
#endif // !BLE_ADV_FRAG_NB_RX_MAX
                }
                else
                {
                    pkt_duration_us = ble_util_pkt_dur_in_us(sync_env->rx_aux_pld_len[0], sync_env->rate);
                }

                // Set the duration for arbiter  + half window size + margin.
                p_evt->duration_min = co_min(SCH_ARB_MAX_DURATION, (pkt_duration_us << 1) + (rx_win_size / 2) + BLE_RESERVATION_TIME_MARGIN_HUS);
            }
        }

        // Save the last accumulative drift aligned between BIG and Sync
        sync_env->sync_drift_acc = sync_drift_acc;
    }
}
#endif // (BLE_BIS)

#if BLE_CONLESS_CTE_RX
uint8_t lld_sync_cte_start(uint8_t act_id, struct lld_sync_cte_params *params)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLD_SYNC_PATCH_TYPE, LLD_SYNC_CTE_START_FUNC_BIT, uint8_t, act_id, params);

    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    GLOBAL_INT_DISABLE();

    if (lld_sync_env[act_id] != NULL)
    {
        // Copy the CTE parameters
        lld_sync_env[act_id]->cte_params = *params;

        // If the event hasn't started yet, configure antenna switching now
        // Otherwise, do this upon EOF
        if (lld_sync_env[act_id]->state == SYNC_EVT_WAIT)
        {
            lld_sync_ant_switch_config(act_id);
        }
        else
        {
            lld_sync_env[act_id]->cte_update = true;
        }

        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return (status);
}

uint8_t lld_sync_cte_stop(uint8_t act_id)
{
    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LLD_SYNC_PATCH_TYPE, LLD_SYNC_CTE_STOP_FUNC_BIT, uint8_t, act_id);

    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    GLOBAL_INT_DISABLE();

    if (lld_sync_env[act_id] != NULL)
    {
        // Set IQ sampling flag to disabled
        lld_sync_env[act_id]->cte_params.sampl_en = IQ_SAMPL_DIS;

        // If the event hasn't started yet, configure antenna switching now
        // Otherwise, do this upon EOF
        if (lld_sync_env[act_id]->state == SYNC_EVT_WAIT)
        {
            lld_sync_ant_switch_config(act_id);
        }
        else
        {
            lld_sync_env[act_id]->cte_update = true;
        }

        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return (status);
}
#endif // BLE_CONLESS_CTE_RX

#if (BLE_PAST)
uint8_t lld_sync_info_get(uint8_t act_id, uint8_t *phy, uint16_t *intv, struct access_addr *aa, struct crc_init *crcinit, uint32_t *sync_ind_ts, uint16_t *sync_ind_bit_off, uint16_t *pa_evt_cnt, struct le_ch_map *p_ch_map, uint8_t *sca)
{
    FUNC_PATCH_ENTRY_VARI_PARAM_HAVE_RETURN(LLD_SYNC_PATCH_TYPE, LLD_SYNC_INFO_GET_FUNC_BIT, uint8_t, 10, act_id, phy, intv, aa, crcinit, sync_ind_ts, sync_ind_bit_off, pa_evt_cnt, p_ch_map, sca);

    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    GLOBAL_INT_DISABLE();

    if (lld_sync_env[act_id] != NULL)
    {
        // Point to parameters
        struct lld_sync_env_tag *sync_env = lld_sync_env[act_id];

        uint8_t cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(act_id);

        *phy = sync_env->rate;
        *intv = (sync_env->intv >> 2);

        // get Synchronization Word
        co_write16p(&aa->addr[0], em_ble_syncwl_get(cs_idx));
        co_write16p(&aa->addr[2], em_ble_syncwh_get(cs_idx));

        // get CRC Initialization value
        co_write16p(&crcinit->crc[0], em_ble_crcinit0_get(cs_idx));
        crcinit->crc[2] = em_ble_crcinit1_crcinit1_getf(cs_idx);

        *sync_ind_ts = sync_env->anchor_ts;
        *sync_ind_bit_off = sync_env->anchor_bit_off;
        *pa_evt_cnt = sync_env->evt_counter;
        *p_ch_map = sync_env->ch_map;
        *sca = sync_env->adv_ca;

        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return (status);
}
#endif // (BLE_PAST)
#endif // (BLE_OBSERVER)

///@} LLDSYNC
