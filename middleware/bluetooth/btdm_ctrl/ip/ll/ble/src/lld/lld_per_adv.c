/**
****************************************************************************************
*
* @file lld_per_adv.c
*
* @brief LLD Periodic advertising source code
*
* Copyright (C) RivieraWaves 2009-2017
*
****************************************************************************************
*/

/**
 ****************************************************************************************
 * @addtogroup LLDPERADV
 * @ingroup LLD
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"         // stack configuration

#if (BLE_BROADCASTER)

#include <string.h>

#include "co_math.h"
#include "co_endian.h"
#include "ble_util.h"            // BLE utility functions

#include "ke_mem.h"
#include "ke_task.h"             // kernel task management
#include "rwip.h"

#include "lld.h"                 // link driver API
#include "lld_int.h"             // link layer driver internal

#include "llm.h"                 // link layer manager definitions

#if (BLE_BIS)
    #include "lld_int_iso.h"         // Specific definition for BIG
#endif // (BLE_BIS)

#include "dbg.h"

#include "sch_arb.h"             // Scheduling Arbiter
#include "sch_prog.h"            // Scheduling Programmer
#include "sch_slice.h"           // Scheduling Slicer

#include "reg_blecore.h"         // BLE core registers
#include "reg_em_ble_cs.h"       // BLE EM Control Structure
#include "reg_em_ble_tx_desc.h"  // BLE EM TX descriptors
#include "reg_em_ble_rx_desc.h"  // BLE EM RX descriptors
#include "reg_em_et.h"           // EM Exchange Table
#include "btdm_patch.h"

/*
 * DEFINES
 *****************************************************************************************
 */

/// Maximum number of attempts to schedule event
#define MAX_SCHED_ATT               15

/// Step for the channel map update instant (in number of events)
#define CH_MAP_UPD_INSTANT_STEP     10

/*
 * ENUMERATION DEFINITION
 *****************************************************************************************
 */

/// Periodic advertising event states
enum PER_ADV_EVT_STATE
{
    PER_ADV_EVT_WAIT,
    PER_ADV_EVT_ACTIVE,
    PER_ADV_EVT_END,
};

/*
 * CONSTANT DEFINITION
 ****************************************************************************************
 */

/*
 * STRUCTURE DEFINITION
 *****************************************************************************************
 */

/// Channel map update procedure information
struct lld_ch_map_upd_proc_info
{
    /// New channel map
    struct le_ch_map ch_map;

    /// Instant when the new parameters apply (compared to connection event counter)
    uint16_t instant;

    /// Boolean indicating whether a new channel map has been provided
    bool new_ch_map;

    /// Boolean indicating whether the new channel map update is pending
    bool new_ch_map_pending;
};

/// LLD Periodic ADV data structure
struct lld_per_adv_data_tag
{
    /// Advertising data buffer
    uint16_t adv_data_buf;

    /// Advertising data length in bytes
    uint16_t adv_data_len;
};

/// LLD Periodic ADV environment structure
struct lld_per_adv_env_tag
{
    /// Advertising Scheduling Arbiter data
    struct sch_arb_elt_tag evt;

    /// Advertising data, used to update the advertising data in a safe manner
    struct lld_per_adv_data_tag data;

    /// Access address
    struct access_addr aa;

    /// CRC init
    struct crc_init crcinit;

    /// Channel map
    struct le_ch_map ch_map;

    /// Channel map update procedure
    struct lld_ch_map_upd_proc_info ch_map_upd;

#if BLE_CONLESS_CTE_TX
    // Periodic advertising CTE Tx parameters
    struct lld_per_adv_cte_params cte_params;
#endif // BLE_CONLESS_CTE_TX

    /// Periodic advertising interval expressed in units of half slots
    uint32_t intv;

    /// Total advertising duration in us
    uint32_t total_adv_dur_us;

    /// Periodic advertising properties
    uint16_t properties;

    /// AUX Offset value in AuxPtr field
    uint16_t aux_offset;

    /// paEventCounter
    uint16_t pa_evt_cnt;

    /// Current advertising data buffer
    uint16_t curr_adv_data_buf;

    /// Current advertising data length
    uint16_t curr_adv_data_len;

#if (BT_53)
    /// AdvDataInfo (ADI) field
    uint16_t adi;
#endif // (BT_53)

    /// Activity identifier
    uint8_t act_id;

    /// state of the advertising mode (@see enum ADV_EVT_STATE)
    uint8_t state;

    /**
     * Local address type
     * 0x00 Public Device Address (default)
     * 0x01 Random Device Address
     * 0x02 Controller generates Resolvable Private Address based on the local IRK from resolving list. If resolving
     *  list contains no matching entry, use public address.
     * 0x03 Controller generates Resolvable Private Address based on the local IRK from resolving list. If resolving
     *  list contains no matching entry, use random address from LE_Set_Random_Address.
     */
    uint8_t own_addr_type;

    /// Secondary advertising rate (@see enum lld_rate)
    uint8_t lld_sec_adv_rate;

    /// Max advertising Tx power (in dBm)
    int8_t adv_tx_pwr;

    /// Current auxiliary channel index
    uint8_t aux_ch_idx;

    /// Total number of Tx descriptors used
    uint8_t total_txdesc_count;

    /// Offset of ACAD data
    uint8_t acad_offset;

    /// Size of AUX_SYNC_IND extended header (without acad data)
    uint8_t sync_ind_ext_hdr_len;

    /// Length of the AUX_SYNC_IND advertising data
    uint8_t sync_ind_adv_data_len;

#if BLE_CONLESS_CTE_TX
    /// Indicates whether the CTE Tx parameters need to be updated
    bool cte_update;
#endif // BLE_CONLESS_CTE_TX

#if (BLE_BIS)
    /// *************************************************
    /// *** Broadcast Isochronous  specific data      ***
    /// *************************************************
    /// BIG attached to periodic advertiser - 0xFF means no BIG attached
    uint8_t  big_act_id;
    /// Update ACAD data
    bool     big_data_update;
#endif // (BLE_BIS)

#if (BT_53)
    /// Indicates whether periodic advertising packets include the ADI field
    bool include_adi;

    /// Advertising SID
    uint8_t adv_sid;

    /// Indicates whether the advertising DID needs to be updated
    bool update_did;
#endif // (BT_53)
};


/*
 * VARIABLE DEFINITION
 *****************************************************************************************
 */

/// LLD Periodic ADV environment variable
__STATIC struct lld_per_adv_env_tag *lld_per_adv_env[BLE_ACTIVITY_MAX];


/*
 * LOCAL FUNCTIONS DECLARATION
 *****************************************************************************************
 */

__STATIC void lld_per_adv_frm_cbk(uint32_t timestamp, uint32_t dummy, uint8_t irq_type);

/**
 ****************************************************************************************
 * @brief Cleanup periodic adv environment variable
 ****************************************************************************************
 */
__STATIC void lld_per_adv_cleanup(uint8_t act_id)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_PER_ADV_PATCH_TYPE, LLD_PER_ADV_CLEANUP_FUNC_BIT, act_id);

    if (lld_per_adv_env[act_id] != NULL)
    {
        // Remove permission/status of CS as now unused
        DBG_MEM_PERM_SET((const void *)(REG_EM_BLE_CS_BASE_ADDR + REG_EM_BLE_CS_ADDR_GET(EM_BLE_CS_ACT_ID_TO_INDEX(act_id))), REG_EM_BLE_CS_SIZE, false, false, false);

        // Free event memory
        ke_free(lld_per_adv_env[act_id]);
        lld_per_adv_env[act_id] = NULL;
    }
}

/**
 ****************************************************************************************
 * @brief Schedule next periodic advertising event
 ****************************************************************************************
 */
__STATIC void lld_per_adv_sched(uint8_t act_id)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_PER_ADV_PATCH_TYPE, LLD_PER_ADV_SCHED_FUNC_BIT, act_id);

    // Point to parameters
    struct sch_arb_elt_tag *evt = &(lld_per_adv_env[act_id]->evt);
    struct lld_per_adv_env_tag *adv_par = lld_per_adv_env[act_id];

    bool found = false;
    uint8_t att = 0;

    // Move timestamp
    evt->time.hs = CLK_ADD_2(evt->time.hs, adv_par->intv);

    // Increment paEventCounter
    adv_par->pa_evt_cnt++;

    while ((!found) && (att < MAX_SCHED_ATT))
    {
        if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
        {
            found = true;
        }
        else
        {
            // Increment priority
            evt->current_prio = RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(RWIP_PRIO_PER_ADV_IDX));

            // Move timestamp
            evt->time.hs = CLK_ADD_2(evt->time.hs, adv_par->intv);
            // Increment paEventCounter
            adv_par->pa_evt_cnt++;

            att++;
        }
    }

    if (found)
    {
        adv_par->state = PER_ADV_EVT_WAIT;
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }
}

/**
 ****************************************************************************************
 * @brief Prepares an extended advertising packet
 ****************************************************************************************
 */

__STATIC uint32_t lld_per_adv_ext_pkt_prepare(uint8_t act_id, uint8_t txdesc_idx,
        uint8_t type, uint8_t mode, bool tx_power, bool big_info,
        uint16_t *ext_header_txbuf_offset, uint16_t *data_txbuf_offset,
        uint16_t *data_len, bool force_aux_ptr)
{
    FUNC_PATCH_ENTRY_VARI_PARAM_HAVE_RETURN(LLD_PER_ADV_PATCH_TYPE, LLD_PER_ADV_EXT_PKT_PREPARE_FUNC_BIT, uint32_t, 10, act_id, txdesc_idx,
                                            type, mode, tx_power, big_info, ext_header_txbuf_offset, data_txbuf_offset, data_len, force_aux_ptr);

    // Point to parameters
    struct lld_per_adv_env_tag *adv_par = lld_per_adv_env[act_id];

    uint32_t dur_us = 0;
    uint32_t aux_offset = 0;
    bool offset_units = 0;
    uint8_t offset;
    uint8_t adv_data_len = 0;
    bool aux_ptr = force_aux_ptr;  // presence of the AUX Pointer
    bool sync_ind = (txdesc_idx == EM_BLE_TXDESC_INDEX(adv_par->act_id, 0));
    bool cte_info;
    uint8_t ext_header_len;
    uint8_t res_ext_header_len; // reserved extended header length

#if BLE_CONLESS_CTE_TX
    cte_info = (adv_par->cte_params.cte_len != NO_CTE) && ((txdesc_idx - EM_BLE_TXDESC_INDEX(adv_par->act_id, 0)) < adv_par->cte_params.cte_count);
#else // BLE_CONLESS_CTE_TX
    cte_info = false;
#endif // BLE_CONLESS_CTE_TX

    // Set advertising data descriptor next pointer to 0
    em_ble_txcntl_nextptr_setf(txdesc_idx, 0);

    ext_header_len = 0;
    offset = 0;

    // First calculate the extended header length
#if BLE_CONLESS_CTE_TX
    if (cte_info)
    {
        ext_header_len += BLE_EXT_CTE_INFO_LEN;
    }
#endif // BLE_CONLESS_CTE_TX

#if (BT_53)
    if (adv_par->include_adi)
    {
        ext_header_len += BLE_EXT_ADI_LEN;
    }
#endif // (BT_53)

    if (aux_ptr)
    {
        ext_header_len += BLE_EXT_AUX_PTR_LEN;
    }

    if (tx_power)
    {
        ext_header_len += BLE_EXT_TX_PWR_LEN;
    }

    // If the extended header length is not 0, add another byte for the extended header flags
    if (ext_header_len != 0)
    {
        ext_header_len += BLE_EXT_ADV_HEADER_FLAGS_LEN; // Extended header flags field (1 octet)
    }

    res_ext_header_len = ext_header_len;

    // If the packet is an AUX_SYNC_IND, add the Channel Map Update Indication length
    if (sync_ind)
    {
        if (res_ext_header_len == 0)
        {
            res_ext_header_len = BLE_EXT_ADV_HEADER_FLAGS_LEN;
        }
#if (BLE_BIS)
        if (big_info)
        {
            // Length to update according to advertising header + 2 for ad_type + length
            res_ext_header_len += BLE_EXT_ACAD_BIG_INFO_ENC_LEN + 2;
        }
        else
#endif // (BLE_BIS)
        {
            res_ext_header_len += BLE_EXT_CH_MAP_UPD_IND_LEN + 2;
        }
    }


    // The aux_ptr field needs to be set if the data does not fit in a single packet
    if ((res_ext_header_len + (*data_len)) > BLE_ADV_FRAG_SIZE_TX)
    {
        aux_ptr = true;
        if (ext_header_len == 0)
        {
            ext_header_len    = BLE_EXT_ADV_HEADER_FLAGS_LEN; // Extended header flags field (1 octet)
            if (res_ext_header_len == 0)
            {
                res_ext_header_len = BLE_EXT_ADV_HEADER_FLAGS_LEN;
            }
        }

        ext_header_len    += BLE_EXT_AUX_PTR_LEN;
        res_ext_header_len += BLE_EXT_AUX_PTR_LEN;
    }

#if BLE_CONLESS_CTE_TX
    if (cte_info)
    {
        uint8_t cte_info_val = 0;

        SETF(cte_info_val, BLE_CTE_INFO_CTE_TIME, adv_par->cte_params.cte_len);
        SETF(cte_info_val, BLE_CTE_INFO_CTE_TYPE, adv_par->cte_params.cte_type);

        // Copy CTEInfo value
        em_wr((void *)&cte_info_val, *ext_header_txbuf_offset + offset, BLE_EXT_CTE_INFO_LEN);

        offset += BLE_EXT_CTE_INFO_LEN;
    }
#endif // BLE_CONLESS_CTE_TX

#if (BT_53)
    if (adv_par->include_adi)
    {
        // Copy adv_data_info
        em_wr((void *)&adv_par->adi, *ext_header_txbuf_offset + offset, BLE_EXT_ADI_LEN);

        offset += BLE_EXT_ADI_LEN;
    }
#endif // (BT_53)

    if (aux_ptr)
    {
        offset += BLE_EXT_AUX_PTR_LEN;
    }

    if (tx_power)
    {
        uint8_t txpwr_idx;
        int8_t rad_tx_pwr;
        uint8_t rf_mod;

        if (adv_par->adv_tx_pwr != ADV_TX_PWR_NO_PREF)
        {
            // Update the transceiver Tx power
            rf_mod = rwip_rf.txpwr_mdlt_get((adv_par->adv_tx_pwr - lld_env.tx_path_comp / 10), 0);
            txpwr_idx = rwip_rf.txpwr_cs_get((adv_par->adv_tx_pwr - lld_env.tx_path_comp / 10), TXPWR_CS_LOWER, rf_mod);
            em_ble_txrxcntl_set(EM_BLE_CS_ACT_ID_TO_INDEX(act_id), txpwr_idx);
        }
        else
        {
            // Using max transceiver Tx power
            txpwr_idx = rwip_rf.txpwr_max;
            rf_mod = rwip_rf.txpwr_max_mod;
        }

        // Determine the radiative Tx power
        rad_tx_pwr = lld_tx_power_dbm_get(txpwr_idx, rf_mod);

        // Copy Tx power
        em_wr((void *)&rad_tx_pwr, *ext_header_txbuf_offset + offset, BLE_EXT_TX_PWR_LEN);

        offset += BLE_EXT_TX_PWR_LEN;
    }

    if (sync_ind)
    {
        // Save the ACAD data offset
        adv_par->acad_offset = offset;

        offset += BLE_EXT_CH_MAP_UPD_IND_LEN + 2;
    }

    if (*data_len > 0)
    {
        uint8_t max_adv_data_len = BLE_ADV_FRAG_SIZE_TX - res_ext_header_len;

        adv_data_len = co_min((*data_len), max_adv_data_len);

        (*data_len) -= adv_data_len;
    }

    em_ble_txaeheader_pack(txdesc_idx, /*txrsvd*/ 0, /*txpow*/ tx_power,
#if (BT_53)
                           /*txsync*/ false, /*txauxptr*/ aux_ptr, /*txadi*/ adv_par->include_adi,
#else // (BT_53)
                           /*txsync*/ false, /*txauxptr*/ aux_ptr, /*txadi*/ false,
#endif // (BT_53)
                           /*txsupp*/ cte_info, /*txtgta*/ false, /*txadva*/ false,
                           /*txaemode*/ mode, /*txaelength*/ ext_header_len);

#if BLE_CONLESS_CTE_TX
    if (cte_info)
    {
        em_ble_txphcte_pack(txdesc_idx, /*txctetype*/ adv_par->cte_params.cte_type, /*txcterfu*/ 0, /*txctetime*/ adv_par->cte_params.cte_len);
    }
#endif // BLE_CONLESS_CTE_TX

    dur_us = ble_util_pkt_dur_in_us((res_ext_header_len + adv_data_len + 1), adv_par->lld_sec_adv_rate);

#if BLE_CONLESS_CTE_TX
    // Update the duration of the packet if CTE present
    if (cte_info)
    {
        dur_us += adv_par->cte_params.cte_len * 8;
    }
#endif // BLE_CONLESS_CTE_TX

    if (aux_ptr)
    {
        dur_us += BLE_AFS_DUR;

        if (dur_us <= OFFSET_LIMIT)
        {
            aux_offset = (dur_us < OFFSET_THRESHOLD) ? (dur_us / 30) : (dur_us / 300);
            offset_units = (dur_us < OFFSET_THRESHOLD) ? 0 : 1;

            aux_offset++;
            adv_par->aux_offset = aux_offset;

            dur_us = (offset_units) ? aux_offset * 300 : aux_offset * 30;
        }
    }

    em_ble_txauxptr0_pack(txdesc_idx, /*txauxoffsetlsb*/ aux_offset, /*txauxoffsetunit*/ offset_units, /*txauxca*/ (rwip_max_drift_get() <= 50), /*txllch*/ adv_par->aux_ch_idx);
    em_ble_txauxptr1_pack(txdesc_idx, /*txauxphy*/ adv_par->lld_sec_adv_rate, /*txauxoffsetmsb*/(adv_par->aux_offset >> 8));
    em_ble_txaedataptr_setf(txdesc_idx, *ext_header_txbuf_offset);

    (*ext_header_txbuf_offset) += offset;

    // Set advertising data Tx descriptor length to (extended header length + advertising data length + 1)
    em_ble_txphadv_pack(txdesc_idx, (ext_header_len + adv_data_len + 1) /*txadvlen*/,
                        0 /*txrxadd*/,
                        0 /*txtxadd*/,
                        0 /*txchsel2*/,
                        0 /*txadvrfu*/,
                        type /*txtype*/);

    // Set data pointer
    em_ble_txdataptr_setf(txdesc_idx, *data_txbuf_offset);

    (*data_txbuf_offset) += adv_data_len;

    // Release descriptor
    em_ble_txcntl_txdone_setf(txdesc_idx, 0);


    if (sync_ind)
    {
        adv_par->sync_ind_ext_hdr_len  = ext_header_len;
        adv_par->sync_ind_adv_data_len = adv_data_len;
    }

    return (dur_us);
}

/**
 ****************************************************************************************
 * @brief Constructs the periodic advertising packet chain
 ****************************************************************************************
 */
__STATIC void lld_per_adv_chain_construct(uint8_t act_id)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_PER_ADV_PATCH_TYPE, LLD_PER_ADV_CHAIN_CONSTRUCT_FUNC_BIT, act_id);

    // Point to parameters
    struct lld_per_adv_env_tag *adv_par = lld_per_adv_env[act_id];

    uint8_t cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(act_id);

    uint8_t type, mode;
    bool tx_power, big_info;

    uint8_t txdesc_index = 0;
    uint8_t sync_txdesc_idx = EM_BLE_TXDESC_INDEX(adv_par->act_id, txdesc_index);
    uint8_t prev_txdesc_idx;

    uint16_t ext_header_txbuf_offset = EM_BLE_ADVEXTHDRTXBUF_OFF(adv_par->act_id);
    uint16_t data_txbuf_offset = 0;
    uint16_t rem_data_len = 0;

    bool force_aux_ptr;

    // Select an auxiliary channel index
    adv_par->aux_ch_idx  = lld_ch_idx_get();

    adv_par->aux_offset = 0;
    adv_par->total_adv_dur_us = 0;

    prev_txdesc_idx = sync_txdesc_idx;

    data_txbuf_offset = adv_par->curr_adv_data_buf;
    rem_data_len = adv_par->curr_adv_data_len;

    // Prepare the AUX_SYNC_IND packet, aux_ptr will be automatically set if necessary
    type = BLE_AUX_SYNC_IND;
    mode = 0;
    tx_power = (adv_par->properties & ADV_TX_PWR);

#if BLE_CONLESS_CTE_TX
    force_aux_ptr = ((adv_par->cte_params.cte_len != NO_CTE) && (adv_par->cte_params.cte_count > 1));
#else // BLE_CONLESS_CTE_TX
    force_aux_ptr = false;
#endif // BLE_CONLESS_CTE_TX

#if (BLE_BIS)
    big_info = (adv_par->big_act_id != BLE_INVALID_LINK_ID);
#else // !(BLE_BIS)
    big_info = false;
#endif // (BLE_BIS)

    adv_par->total_adv_dur_us += lld_per_adv_ext_pkt_prepare(act_id, sync_txdesc_idx, type, mode, tx_power, big_info, &ext_header_txbuf_offset, &data_txbuf_offset, &rem_data_len, force_aux_ptr);
    // Some of the info in the packet have to be duplicated in the CS
    em_ble_chmap2_ch_aux_setf(cs_idx, adv_par->aux_ch_idx);
    em_ble_thrcntl_ratecntl_aux_rate_setf(cs_idx, adv_par->lld_sec_adv_rate);
    em_ble_auxtxdescptr_setf(cs_idx, REG_EM_ADDR_GET(BLE_TX_DESC, sync_txdesc_idx));


    // Construct a chain of AUX_CHAIN_IND packets if needed
    while (rem_data_len)
    {
        txdesc_index++;
        uint8_t chain_txdesc_idx = EM_BLE_TXDESC_INDEX(adv_par->act_id, txdesc_index);

        // Prepare an AUX_CHAIN_IND packet, aux_ptr will be automatically set if necessary
        type = BLE_AUX_CHAIN_IND;
        mode = 0;
        tx_power = false;

#if BLE_CONLESS_CTE_TX
        force_aux_ptr = ((adv_par->cte_params.cte_len != NO_CTE) && ((txdesc_index + 1) < adv_par->cte_params.cte_count));
#endif // BLE_CONLESS_CTE_TX

        adv_par->total_adv_dur_us += lld_per_adv_ext_pkt_prepare(act_id, chain_txdesc_idx, type, mode, tx_power, big_info, &ext_header_txbuf_offset, &data_txbuf_offset, &rem_data_len, force_aux_ptr);

        // Link previous Tx descriptor to this one
        em_ble_txcntl_nextptr_setf(prev_txdesc_idx, REG_EM_ADDR_GET(BLE_TX_DESC, chain_txdesc_idx));
        prev_txdesc_idx = chain_txdesc_idx;
    }

#if BLE_CONLESS_CTE_TX
    // If CTE is enabled
    if (adv_par->cte_params.cte_len != NO_CTE)
    {
        // Continue the chain using AUX_CHAIN_IND PDUs with no AdvData to reach CTE_Count
        while ((txdesc_index + 1) < adv_par->cte_params.cte_count)
        {
            txdesc_index++;
            uint8_t chain_txdesc_idx = EM_BLE_TXDESC_INDEX(adv_par->act_id, txdesc_index);
            force_aux_ptr = ((txdesc_index + 1) < adv_par->cte_params.cte_count);

            // Prepare an AUX_CHAIN_IND packet, force aux_ptr if CTE_Count hasn't been reached
            type = BLE_AUX_CHAIN_IND;
            mode = 0;
            tx_power = false;
            adv_par->total_adv_dur_us += lld_per_adv_ext_pkt_prepare(act_id, chain_txdesc_idx, type, mode, tx_power,
                                         false, &ext_header_txbuf_offset, &data_txbuf_offset,
                                         &rem_data_len, force_aux_ptr);

            // Link previous Tx descriptor to this one
            em_ble_txcntl_nextptr_setf(prev_txdesc_idx, REG_EM_ADDR_GET(BLE_TX_DESC, chain_txdesc_idx));
            prev_txdesc_idx = chain_txdesc_idx;
        }
    }
#endif // BLE_CONLESS_CTE_TX

    adv_par->total_txdesc_count = txdesc_index + 1;

    // Set minimum duration according to size
    lld_per_adv_env[act_id]->evt.duration_min = co_max(2 * adv_par->total_adv_dur_us + BLE_RESERVATION_TIME_MARGIN_HUS, HALF_SLOT_SIZE + 1);

    // Set max event time in slots (unit of 625 us)
    em_ble_maxevtime_set(cs_idx, (adv_par->total_adv_dur_us / SLOT_SIZE) + 1);
}

/**
 ****************************************************************************************
 * @brief Set periodic advertising data
 ****************************************************************************************
 */
__STATIC void lld_per_adv_data_set(uint8_t act_id, uint8_t len, uint16_t data, bool release_old_buf)
{
    FUNC_PATCH_ENTRY_4_PARAM_NO_RETURN(LLD_PER_ADV_PATCH_TYPE, LLD_PER_ADV_DATA_SET_FUNC_BIT, act_id, len, data, release_old_buf);

    // Point to parameters
    struct lld_per_adv_env_tag *adv_par = lld_per_adv_env[act_id];

    uint16_t old_buf_offset = 0;

    uint8_t sync_txdesc_idx = EM_BLE_TXDESC_INDEX(act_id, 0);

    if (release_old_buf)
    {
        old_buf_offset = em_ble_txdataptr_getf(sync_txdesc_idx);
    }

    adv_par->curr_adv_data_len = len;
    adv_par->curr_adv_data_buf = data;
    lld_per_adv_chain_construct(adv_par->act_id);

    if ((release_old_buf) && (old_buf_offset != 0))
    {
        ble_util_buf_adv_tx_free(old_buf_offset);
    }
}

#if (BT_53)
/**
 ****************************************************************************************
 * @brief Set a new advertising DID
 ****************************************************************************************
 */
__STATIC void lld_per_adv_did_set(uint8_t act_id)
{
    // Point to parameters
    struct lld_per_adv_env_tag *adv_par = lld_per_adv_env[act_id];
    // Generate a new ADI DID
    uint16_t adi_did = co_rand_hword() & BLE_ADI_DID_MASK;

    // DID shall not be the same as the previously used one, LL 4.4.2.11
    while (adi_did == GETF(adv_par->adi, BLE_ADI_DID))
    {
        adi_did = co_rand_hword() & BLE_ADI_DID_MASK;
    }

    // Set the new DID
    SETF(adv_par->adi, BLE_ADI_DID, adi_did);

    // Update the chain
    lld_per_adv_chain_construct(act_id);
}
#endif // (BT_53)
#if BLE_CONLESS_CTE_TX
/**
 ****************************************************************************************
 * @brief Configure antenna switching
 ****************************************************************************************
 */
__STATIC void lld_per_adv_ant_switch_config(uint8_t act_id)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_PER_ADV_PATCH_TYPE, LLD_PER_ADV_ANT_SWITCH_CONFIG_FUNC_BIT, act_id);

    // Point to parameters
    struct lld_per_adv_env_tag *adv_par = lld_per_adv_env[act_id];

    uint8_t cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(act_id);

    // Check if CTE is enabled and multiple antennae are used
    if ((adv_par->cte_params.cte_len != NO_CTE) && (adv_par->cte_params.cte_type != CTE_TYPE_AOA))
    {
        // Set the Antenna ID pointer
        em_ble_txdfantswptr_set(cs_idx, EM_BLE_TX_ANTENNA_ID_OFFSET >> 2);

        // Write antenna IDs to EM
        em_wr(&adv_par->cte_params.antenna_id[0], EM_BLE_TX_ANTENNA_ID_OFFSET, adv_par->cte_params.switching_pattern_len);

        // Set the length of the switching pattern
        em_ble_txdfantpattcntl_tx_ant_patt_length_setf(cs_idx, adv_par->cte_params.switching_pattern_len);
    }
    else
    {
        // Disable antenna switching
        em_ble_txdfantpattcntl_tx_ant_patt_length_setf(cs_idx, 0);
    }
}
#endif // BLE_CONLESS_CTE_TX

/**
 ****************************************************************************************
 * @brief Handle event start notification
 ****************************************************************************************
 */
__STATIC void lld_per_adv_evt_start_cbk(struct sch_arb_elt_tag *evt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_PER_ADV_PATCH_TYPE, LLD_PER_ADV_EVT_START_CBK_FUNC_BIT, evt);

    DBG_SWDIAG(LEPERADV, EVT_START, 1);

    if (evt != NULL)
    {
        // Point to parameters
        struct lld_per_adv_env_tag *adv_par = (struct lld_per_adv_env_tag *) evt;
        uint8_t cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(adv_par->act_id);
        uint8_t sync_txdesc_idx = EM_BLE_TXDESC_INDEX(adv_par->act_id, 0);
        uint8_t ext_header_len = adv_par->sync_ind_ext_hdr_len;
        uint16_t ext_header_txbuf_offset = EM_BLE_ADVEXTHDRTXBUF_OFF(adv_par->act_id);

        // Set the paEventCounter in the CS
        em_ble_evtcnt_setf(cs_idx, adv_par->pa_evt_cnt);

        // Check if there is a new channel map
        if (adv_par->ch_map_upd.new_ch_map)
        {
            uint8_t ch_map_upd_ind_ad_type_len[2];

            ch_map_upd_ind_ad_type_len[0] = BLE_EXT_CH_MAP_UPD_IND_LEN + 1; // Length of the data plus 1 byte for ad_type
            ch_map_upd_ind_ad_type_len[1] = BLE_EXT_ACAD_CHANNEL_MAP_UPDATE_INDICATION_AD_TYPE;

            // Set the channel Map Update Indication AD Type and length
            em_wr((void *)&ch_map_upd_ind_ad_type_len[0], ext_header_txbuf_offset + adv_par->acad_offset, 2);

            // Set the channel map in the channel Map Update Indication
            em_wr((void *)&adv_par->ch_map_upd.ch_map.map[0], ext_header_txbuf_offset + adv_par->acad_offset + 2, LE_CH_MAP_LEN);

            // Update the instant
            adv_par->ch_map_upd.instant = adv_par->pa_evt_cnt + CH_MAP_UPD_INSTANT_STEP;

            // Set the instant in the Channel Map Update Indication
            {
                uint8_t byte7 = adv_par->ch_map_upd.instant;
                uint8_t byte8 = adv_par->ch_map_upd.instant >> 8;

                // Copy 2 bytes
                em_wr((void *)&byte7, ext_header_txbuf_offset + adv_par->acad_offset + 7, 1);
                em_wr((void *)&byte8, ext_header_txbuf_offset + adv_par->acad_offset + 8, 1);
            }

            // Clear the new channel map flag
            adv_par->ch_map_upd.new_ch_map = false;

            // Set the pending flag
            adv_par->ch_map_upd.new_ch_map_pending = true;
        }
        else if ((adv_par->ch_map_upd.new_ch_map_pending) && BLE_UTIL_INSTANT_PASSED(adv_par->ch_map_upd.instant, adv_par->pa_evt_cnt))
        {
            // Apply new the channel map
            em_ble_chmap0_set(cs_idx, (adv_par->ch_map_upd.ch_map.map[1] << 8) | adv_par->ch_map_upd.ch_map.map[0]);
            em_ble_chmap1_set(cs_idx, (adv_par->ch_map_upd.ch_map.map[3] << 8) | adv_par->ch_map_upd.ch_map.map[2]);
            em_ble_chmap2_set(cs_idx, adv_par->ch_map_upd.ch_map.map[4]);

            // The new channel map becomes the current one
            adv_par->ch_map = adv_par->ch_map_upd.ch_map;

            // Clear the pending flag
            adv_par->ch_map_upd.new_ch_map_pending = false;
        }

        // Update extended header length
        if (adv_par->ch_map_upd.new_ch_map_pending)
        {
            uint8_t  acad_len = BLE_EXT_CH_MAP_UPD_IND_LEN + 2;
            ext_header_len += (ext_header_len == 0) * BLE_EXT_ADV_HEADER_FLAGS_LEN + acad_len;
        }

#if (BLE_BIS)
        // BIGInfo can be added if the BIG is unencrypted or a channel map update is not pending
        if (adv_par->big_act_id != BLE_INVALID_LINK_ID)
        {
            uint8_t  biginfo_ind_offset = (adv_par->ch_map_upd.new_ch_map_pending) ? (adv_par->acad_offset + BLE_EXT_CH_MAP_UPD_IND_LEN + 2) : adv_par->acad_offset;
            uint8_t max_acad_size = BLE_EXT_MAX_HEADER_LEN - ext_header_len - (ext_header_len == 0) * BLE_EXT_ADV_HEADER_FLAGS_LEN;
            uint8_t  acad_len;

            // request BIG activity to update ACAD data.
            acad_len = lld_big_update_info(adv_par->big_act_id, evt->time.hs, (ext_header_txbuf_offset + biginfo_ind_offset), max_acad_size);

            // Update extended header length
            if (acad_len > 0)
            {
                ext_header_len += (ext_header_len == 0) * BLE_EXT_ADV_HEADER_FLAGS_LEN + acad_len;
            }
        }
#endif // (BLE_BIS)

        // Update extended header length
        em_ble_txaeheader_txaelength_setf(sync_txdesc_idx, ext_header_len);

        // Set advertising data Tx descriptor length to (extended header length + advertising data length + 1)
        em_ble_txphadv_txadvlen_setf(sync_txdesc_idx, (ext_header_len + adv_par->sync_ind_adv_data_len + 1));

        {
            // Push the programming to SCH PROG
            struct sch_prog_params prog_par;
            prog_par.frm_cbk        = &lld_per_adv_frm_cbk;
            prog_par.time.hs        = evt->time.hs;
            prog_par.time.hus       = 0;
            prog_par.cs_idx         = cs_idx;
            prog_par.dummy          = cs_idx;
            prog_par.bandwidth      = evt->duration_min;
            prog_par.prio_1         = evt->current_prio;
            prog_par.prio_2         = 0;
            prog_par.prio_3         = 0;
            prog_par.pti_prio       = RW_BLE_PTI_PRIO_AUTO;
            prog_par.add.ble.ae_nps = 1;
            prog_par.add.ble.iso    = 0;
            prog_par.mode           = SCH_PROG_BLE;
            sch_prog_push(&prog_par);
        }

        // Move state
        adv_par->state = PER_ADV_EVT_ACTIVE;
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(LEPERADV, EVT_START, 0);
}

/**
 ****************************************************************************************
 * @brief Handle event canceled notification
 ****************************************************************************************
 */
__STATIC void lld_per_adv_evt_canceled_cbk(struct sch_arb_elt_tag *evt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_PER_ADV_PATCH_TYPE, LLD_PER_ADV_EVT_CANCELED_CBK_FUNC_BIT, evt);

    DBG_SWDIAG(LEPERADV, EVT_CANCELED, 1);

    if (evt != NULL)
    {
        // Point to parameters
        struct lld_per_adv_env_tag *adv_par = (struct lld_per_adv_env_tag *) evt;

        ASSERT_ERR(adv_par->state == PER_ADV_EVT_WAIT);

        // Increment priority
        evt->current_prio = RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(RWIP_PRIO_PER_ADV_IDX));

        // Try to reschedule
        lld_per_adv_sched(adv_par->act_id);
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(LEPERADV, EVT_CANCELED, 0);
}

/**
 ****************************************************************************************
 * @brief Handle frame interrupt
 ****************************************************************************************
 */
__STATIC void lld_per_adv_frm_isr(uint8_t act_id, uint32_t timestamp, bool abort)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LLD_PER_ADV_PATCH_TYPE, LLD_PER_ADV_FRM_ISR_FUNC_BIT, act_id, timestamp, abort);

    DBG_SWDIAG(LEPERADV, FRM_ISR, 1);

    if (lld_per_adv_env[act_id] != NULL)
    {
        // Point to parameters
        struct sch_arb_elt_tag *evt = &(lld_per_adv_env[act_id]->evt);
        struct lld_per_adv_env_tag *adv_par = lld_per_adv_env[act_id];

        // Remove event
        sch_arb_remove(evt, true);

        // Check advertising end
        if (adv_par->state == PER_ADV_EVT_END)
        {
            // Report advertising end to LM
            struct lld_per_adv_end_ind *ind = KE_MSG_ALLOC(LLD_PER_ADV_END_IND, TASK_LLM, TASK_NONE, lld_per_adv_end_ind);
            ind->act_id = adv_par->act_id;
            ind->status = CO_ERROR_NO_ERROR;
            ke_msg_send(ind);

            // Free event memory
            lld_per_adv_cleanup(act_id);
        }
        else
        {
            uint8_t txdesc_count = 0;
            uint8_t txdesc_idx = EM_BLE_TXDESC_INDEX(adv_par->act_id, txdesc_count);

            // Check if the host has updated the advertising data
            if (adv_par->data.adv_data_buf != 0)
            {
                // Set the advertising data
                lld_per_adv_data_set(act_id, adv_par->data.adv_data_len, adv_par->data.adv_data_buf, true);
                adv_par->data.adv_data_buf = 0;
            }

#if (BT_53)
            // Check if the advertising DID has to be updated
            if (adv_par->update_did)
            {
                // Set the new DID
                lld_per_adv_did_set(act_id);
                adv_par->update_did = false;
            }
#endif // (BT_53)

#if BLE_CONLESS_CTE_TX
            // Check if the host has updated the CTE Tx parameters
            if (adv_par->cte_update)
            {
                // Reconstruct the periodic advertising chain
                lld_per_adv_chain_construct(act_id);

                // Configure antenna switching
                lld_per_adv_ant_switch_config(act_id);

                adv_par->cte_update = false;
            }
#endif // BLE_CONLESS_CTE_TX

#if (BLE_BIS)
            // BIG Info ACAD data must be updated
            if (adv_par->big_data_update)
            {
                lld_per_adv_chain_construct(act_id);
                adv_par->big_data_update = false;
            }
#endif // (BLE_BIS)

            // Release descriptor
            em_ble_txcntl_txdone_setf(txdesc_idx, 0);

            // Update auxiliary channel index in packets and CS if needed
            if (em_ble_txaeheader_txauxptr_getf(txdesc_idx))
            {
                uint8_t cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(adv_par->act_id);

                // Select an auxiliary channel index
                adv_par->aux_ch_idx  = lld_ch_idx_get();

                // Update the channel index
                em_ble_txauxptr0_tx_ll_ch_setf(txdesc_idx, adv_par->aux_ch_idx);

                // Update CS
                em_ble_chmap2_ch_aux_setf(cs_idx, adv_par->aux_ch_idx);

                // Update all remaining packets
                while (1)
                {
                    // Point to next packet
                    txdesc_idx = EM_BLE_TXDESC_INDEX(adv_par->act_id, ++txdesc_count);

                    // Release descriptor
                    em_ble_txcntl_txdone_setf(txdesc_idx, 0);

                    // If last packet, exit the loop
                    if (txdesc_count == (adv_par->total_txdesc_count - 1))
                        break;

                    // Update the channel index
                    em_ble_txauxptr0_tx_ll_ch_setf(txdesc_idx, adv_par->aux_ch_idx);
                }
            }

            // update priority
            evt->current_prio = abort
                                ? RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(RWIP_PRIO_PER_ADV_IDX))
                                : rwip_priority[RWIP_PRIO_PER_ADV_IDX].value;

            SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_NO_ASAP, SCH_ARB_NO_PHASE, 0, RWIP_PRIO_INC(RWIP_PRIO_PER_ADV_IDX));

            // Try to reschedule
            lld_per_adv_sched(adv_par->act_id);
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(LEPERADV, FRM_ISR, 0);
}

/**
 ****************************************************************************************
 * @brief Handle skip interrupt
 ****************************************************************************************
 */
__STATIC void lld_per_adv_frm_skip_isr(uint8_t act_id)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_PER_ADV_PATCH_TYPE, LLD_PER_ADV_FRM_SKIP_ISR_FUNC_BIT, act_id);

    DBG_SWDIAG(LEPERADV, EVT_CANCELED, 1);

    if (lld_per_adv_env[act_id] != NULL)
    {
        // Point to parameters
        struct sch_arb_elt_tag *evt = &(lld_per_adv_env[act_id]->evt);
        struct lld_per_adv_env_tag *adv_par = lld_per_adv_env[act_id];

        ASSERT_ERR((adv_par->state == PER_ADV_EVT_ACTIVE) || (adv_par->state == PER_ADV_EVT_END));

        // Remove event
        sch_arb_remove(evt, true);

        // Check advertising end
        if (adv_par->state == PER_ADV_EVT_END)
        {
            // Report advertising end to LM
            struct lld_per_adv_end_ind *ind = KE_MSG_ALLOC(LLD_PER_ADV_END_IND, TASK_LLM, TASK_NONE, lld_per_adv_end_ind);
            ind->act_id = adv_par->act_id;
            ind->status = CO_ERROR_NO_ERROR;
            ke_msg_send(ind);

            // Free event memory
            lld_per_adv_cleanup(act_id);
        }
        else
        {
            // Increment priority
            evt->current_prio = RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(RWIP_PRIO_PER_ADV_IDX));

            // Try to reschedule
            lld_per_adv_sched(adv_par->act_id);
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(LEPERADV, EVT_CANCELED, 0);
}

/**
 ****************************************************************************************
 * @brief Handle frame interrupt notification
 ****************************************************************************************
 */
__STATIC void lld_per_adv_frm_cbk(uint32_t timestamp, uint32_t dummy, uint8_t irq_type)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LLD_PER_ADV_PATCH_TYPE, LLD_PER_ADV_FRM_CBK_FUNC_BIT, timestamp, dummy, irq_type);

    switch (irq_type)
    {
    case SCH_FRAME_IRQ_EOF:
    case SCH_FRAME_IRQ_EOF_ABORT_AFTER_PRIO: // Post-bandwidth contested as low priority (prio_2 = 0)
    {
        lld_per_adv_frm_isr(EM_BLE_CS_IDX_TO_ACT_ID(dummy), timestamp, false);
    }
    break;
    case SCH_FRAME_IRQ_EOF_ABORT_UNDER_PRIO:
    {
        lld_per_adv_frm_isr(EM_BLE_CS_IDX_TO_ACT_ID(dummy), timestamp, true);
    }
    break;
    case SCH_FRAME_IRQ_RX:
    {
        // No need to act upon the Rx interrupt for advertising
    } break;
    case SCH_FRAME_IRQ_SKIP:
    {
        lld_per_adv_frm_skip_isr(EM_BLE_CS_IDX_TO_ACT_ID(dummy));
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

void lld_per_adv_init(bool is_reset)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLD_PER_ADV_PATCH_TYPE, LLD_PER_ADV_INIT_FUNC_BIT, is_reset);

    if (is_reset)
    {
        for (int8_t act_id = (BLE_ACTIVITY_MAX - 1); act_id >= 0 ; act_id--)
        {
            // Check if periodic advertising is ongoing
            if (lld_per_adv_env[act_id] != NULL)
            {
                // Free event memory
                ke_free(lld_per_adv_env[act_id]);
            }
        }
    }

    // Initialize environment
    memset(&lld_per_adv_env, 0, sizeof(lld_per_adv_env));
}

uint8_t lld_per_adv_start(uint8_t act_id, struct lld_per_adv_params *params)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLD_PER_ADV_PATCH_TYPE, LLD_PER_ADV_START_FUNC_BIT, uint8_t, act_id, params);

    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;
    uint8_t rf_mod;
    uint8_t txpwr;

    // Check if advertising is inactive
    if (lld_per_adv_env[act_id] == NULL)
    {
        // Allocate environment
        lld_per_adv_env[act_id] = LLD_ALLOC_ENV(lld_per_adv_env_tag);

        if (lld_per_adv_env[act_id] != NULL)
        {
            // Point to parameters
            struct sch_arb_elt_tag *evt = &(lld_per_adv_env[act_id]->evt);
            struct lld_per_adv_env_tag *adv_par = lld_per_adv_env[act_id];

            uint8_t cs_idx = EM_BLE_CS_ACT_ID_TO_INDEX(act_id);

            LLD_INIT_ENV(adv_par, lld_per_adv_env_tag);

            // Set permission/status of CS as R/W but uninitialized
            DBG_MEM_PERM_SET((const void *)(REG_EM_BLE_CS_BASE_ADDR + REG_EM_BLE_CS_ADDR_GET(cs_idx)), REG_EM_BLE_CS_SIZE, true, true, true);

            // Initialize event parameters (common part)
            evt->cb_cancel           = &lld_per_adv_evt_canceled_cbk;
            evt->cb_start            = &lld_per_adv_evt_start_cbk;
            evt->current_prio        = rwip_priority[RWIP_PRIO_PER_ADV_IDX].value;

            // Initialize event parameters (advertising part)
            adv_par->act_id     = act_id;
            adv_par->properties = params->per_adv_properties;
            adv_par->intv       = (params->per_adv_intv << 2);
            adv_par->ch_map_upd.instant = adv_par->pa_evt_cnt + CH_MAP_UPD_INSTANT_STEP;
            adv_par->own_addr_type = params->own_addr_type;
            adv_par->adv_tx_pwr = params->adv_tx_pwr;
            adv_par->lld_sec_adv_rate = ((params->sec_adv_phy > 2) && (lld_env.le_coded_phy_500)) ? params->sec_adv_phy : params->sec_adv_phy - 1;

            adv_par->curr_adv_data_len = params->init_adv_data_len;
            adv_par->curr_adv_data_buf = params->init_adv_data_buf;
#if (BLE_BIS)
            adv_par->big_act_id        = params->big_act_id;
#endif // (BLE_BIS)

            // Initialise the channel map
            memset(&adv_par->ch_map.map[0], 0x00, LE_CH_MAP_LEN);
            for (uint8_t idx = 0 ; idx < lld_env.nb_used_ch; idx++)
            {
                uint8_t byte_idx = lld_env.sec_adv_ch_list[idx] >> 3;
                uint8_t bit_pos = lld_env.sec_adv_ch_list[idx] & 0x7;

                adv_par->ch_map.map[byte_idx] |= (1 << bit_pos);
            }

#if BLE_CONLESS_CTE_TX
            // Copy the CTE parameters
            memcpy(&lld_per_adv_env[act_id]->cte_params, &params->cte_params, sizeof(struct lld_per_adv_cte_params));
            adv_par->cte_update = false;
#endif // BLE_CONLESS_CTE_TX

#if (BT_53)
            adv_par->include_adi = params->include_adi;

            // Set the ADI field (SID and DID)
            if (adv_par->include_adi)
            {
                uint16_t adi_did = co_rand_hword() & BLE_ADI_DID_MASK;

                SETF(adv_par->adi, BLE_ADI_DID, adi_did);
                SETF(adv_par->adi, BLE_ADI_SID, params->adv_sid);
            }
#endif // (BT_53)

            // Initialize addresses of fields set conditionally / before pack initialize - TODO cleaning
            em_ble_chmap2_set(cs_idx, 0);
            em_ble_thrcntl_ratecntl_set(cs_idx, 0);

            // Construct the periodic advertising packet chain
            lld_per_adv_chain_construct(act_id);

#if BLE_CONLESS_CTE_TX
            // Configure antenna switching
            lld_per_adv_ant_switch_config(act_id);
#endif // BLE_CONLESS_CTE_TX

            // Set control structure fields
            // Extended advertiser
            em_ble_cntl_pack(cs_idx,
                             RWIP_COEX_GET(ADV, TXBSY),
                             RWIP_COEX_GET(ADV, RXBSY),
                             RWIP_COEX_GET(ADV, DNABORT),
#if (EAVESDROPPING_SUPPORT)
                             1,
#endif // EAVESDROPPING_SUPPORT
                             EM_BLE_CS_FMT_EXT_ADV);

            // FH_EN and HOP_SEL must be set in the CS
            em_ble_hopcntl_pack(cs_idx, /*fhen*/true,/*hopmode*/LLD_HOP_MODE_CHAN_SEL_2, /*hopint*/ 0, /* chidx */39);

            if (adv_par->adv_tx_pwr != ADV_TX_PWR_NO_PREF)
            {
                // Apply transceiver Tx power level equivalent to radiative Tx power parameter value
                rf_mod = rwip_rf.txpwr_mdlt_get((adv_par->adv_tx_pwr - lld_env.tx_path_comp / 10), 0);
                txpwr = rwip_rf.txpwr_cs_get((adv_par->adv_tx_pwr - lld_env.tx_path_comp / 10), TXPWR_CS_LOWER, rf_mod);
                //em_ble_txrxcntl_set(cs_idx, rwip_rf.txpwr_cs_get((adv_par->adv_tx_pwr - lld_env.tx_path_comp / 10), TXPWR_CS_LOWER));
            }
            else
            {
                // Maximum Tx power
                //em_ble_txrxcntl_set(cs_idx, rwip_rf.txpwr_max);
                rf_mod = rwip_rf.txpwr_max_mod;
                txpwr = rwip_rf.txpwr_max;
            }
            rwip_rf.txpwr_cs_set(TYPE_BLE, cs_idx, rf_mod, txpwr);

            // Set channel map
            em_ble_chmap0_set(cs_idx, (adv_par->ch_map.map[1] << 8) | adv_par->ch_map.map[0]);
            em_ble_chmap1_set(cs_idx, (adv_par->ch_map.map[3] << 8) | adv_par->ch_map.map[2]);
            em_ble_chmap2_set(cs_idx, adv_par->ch_map.map[4]);

            // Generate the access address
            lld_aa_gen(&adv_par->aa.addr[0], act_id);

            // save the CRC init
            {
                uint32_t random_nb = co_rand_word();
                co_write16(&adv_par->crcinit.crc[0], co_htobs(0x0000FFFF & random_nb));
                co_write8(&adv_par->crcinit.crc[2], (0x00FF0000 & random_nb) >> 16);
            }

            // set Synchronization Word
            em_ble_syncwl_set(cs_idx, (adv_par->aa.addr[1] << 8) | adv_par->aa.addr[0]);
            em_ble_syncwh_set(cs_idx, (adv_par->aa.addr[3] << 8) | adv_par->aa.addr[2]);
            // set CRC Initialization value
            em_ble_crcinit0_set(cs_idx, (adv_par->crcinit.crc[1] << 8) | adv_par->crcinit.crc[0]);
            em_ble_crcinit1_pack(cs_idx,  /*rxmaxctebuf*/ 0, /*crcinit1*/ adv_par->crcinit.crc[2]);
            // Set Rx Max buf and Rx Max Time @0x0 -> v4.0 behavior
            em_ble_rxmaxbuf_set(cs_idx, 0x0);
            em_ble_rxmaxtime_set(cs_idx, 0x0);


            // Set Rx/Tx threshold + rate
            em_ble_thrcntl_ratecntl_pack(cs_idx, /*rxthr*/0, /*txthr*/0, /*auxrate*/ adv_par->lld_sec_adv_rate,
                                         /*rxrate*/ adv_par->lld_sec_adv_rate, /*txrate*/ adv_par->lld_sec_adv_rate);

            // Set link field
            em_ble_linkcntl_pack(cs_idx, /*hplpmode*/ 0, /*linklbl*/ cs_idx, /*sas*/ false, /*nullrxllidflt*/true,
                                 /*micmode*/ ENC_MIC_PRESENT, /*cryptmode*/ENC_MODE_PKT_PLD_CNT, /*txcrypten*/ false,
                                 /*rxcrypten*/ false, /*privnpub*/ ((params->own_addr_type & ADDR_MASK) != ADDR_PUBLIC));

            // Set the event counter offset fields to 0
            em_ble_evtcnt_offset0_set(cs_idx, 0x0000);
            em_ble_evtcnt_offset1_set(cs_idx, 0x0000);
            em_ble_evtcnt_offset2_set(cs_idx, 0x0000);

            // Disable unused control
            em_ble_rxwincntl_set(cs_idx, 0);
            em_ble_acltxdescptr_set(cs_idx, 0);
            em_ble_minevtime_set(cs_idx, 0);
            em_ble_filtpol_ralcntl_set(cs_idx, 0);
            em_ble_adv_bd_addr_set(cs_idx, 0, 0x0000);
            em_ble_adv_bd_addr_set(cs_idx, 1, 0x0000);
            em_ble_adv_bd_addr_set(cs_idx, 2, 0x0000);
            em_ble_adv_bd_addr_type_setf(cs_idx, 0);
            em_ble_lebdaddr_set(cs_idx, 0, 0);
            em_ble_lebdaddr_set(cs_idx, 1, 0);
            em_ble_lebdaddr_set(cs_idx, 2, 0);
            em_ble_txheadercntl_set(cs_idx, 0);

            // Schedule event
            {
                bool found = false;
                uint8_t att = 0;
                uint32_t clock = CLK_ADD_2(lld_read_clock(), rwip_prog_delay + 1); // Ensure that first event can be programmed

                evt->time.hs = CLK_ADD_2(clock, adv_par->intv - CO_MOD(clock, adv_par->intv) + params->per_adv_offset);
                evt->time.hus = 0;
                SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_NO_ASAP, SCH_ARB_NO_PHASE, 0, RWIP_PRIO_INC(RWIP_PRIO_PER_ADV_IDX));

                GLOBAL_INT_DISABLE();

                while ((!found) && (att < MAX_SCHED_ATT))
                {
                    if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
                    {
                        found = true;
                    }
                    else
                    {
                        // Increment priority
                        evt->current_prio = RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(RWIP_PRIO_PER_ADV_IDX));

                        // Move timestamp
                        evt->time.hs = CLK_ADD_2(evt->time.hs, adv_par->intv);

                        att++;
                    }
                }

                if (found)
                {
                    adv_par->state = PER_ADV_EVT_WAIT;
                }
                else
                {
                    ASSERT_ERR_FORCE(0);
                }

                GLOBAL_INT_RESTORE();
            }

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

uint8_t lld_per_adv_stop(uint8_t act_id)
{
    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LLD_PER_ADV_PATCH_TYPE, LLD_PER_ADV_STOP_FUNC_BIT, uint8_t, act_id);

    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    GLOBAL_INT_DISABLE();

    if (lld_per_adv_env[act_id] != NULL)
    {
        // Point to parameters
        struct sch_arb_elt_tag *evt = &(lld_per_adv_env[act_id]->evt);
        struct lld_per_adv_env_tag *adv_par = lld_per_adv_env[act_id];

        switch (adv_par->state)
        {
        case PER_ADV_EVT_WAIT:
        {
            // Remove event
            sch_arb_remove(evt, false);

            // Report advertising end to LLM
            struct lld_per_adv_end_ind *ind = KE_MSG_ALLOC(LLD_PER_ADV_END_IND, TASK_LLM, TASK_NONE, lld_per_adv_end_ind);
            ind->act_id = act_id;
            ind->status = CO_ERROR_NO_ERROR;
            ke_msg_send(ind);

            // Free event memory
            lld_per_adv_cleanup(act_id);
        }
        break;

        case PER_ADV_EVT_ACTIVE:
        {
            // Abort advertising only if applicable, and minimize maxevtime in case pre-fetch
            em_ble_maxevtime_set(EM_BLE_CS_ACT_ID_TO_INDEX(adv_par->act_id), 1);
            ble_rwblecntl_advert_abort_setf(1);
            // Move state
            adv_par->state = PER_ADV_EVT_END;
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

void lld_per_adv_data_update(uint8_t act_id, uint8_t len, uint16_t buffer)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LLD_PER_ADV_PATCH_TYPE, LLD_PER_ADV_DATA_UPDATE_FUNC_BIT, act_id, len, buffer);

    GLOBAL_INT_DISABLE();

    if (lld_per_adv_env[act_id] != NULL)
    {
        if (lld_per_adv_env[act_id]->state == PER_ADV_EVT_WAIT)
        {
            lld_per_adv_data_set(act_id, len, buffer, true);
        }
        else
        {
            lld_per_adv_env[act_id]->data.adv_data_len = len;
            lld_per_adv_env[act_id]->data.adv_data_buf = buffer;
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    GLOBAL_INT_RESTORE();
}

#if (BT_53)
void lld_per_adv_did_update(uint8_t act_id)
{
    GLOBAL_INT_DISABLE();

    if (lld_per_adv_env[act_id] != NULL)
    {
        // If ADI in included
        if (lld_per_adv_env[act_id]->include_adi)
        {
            if (lld_per_adv_env[act_id]->state == PER_ADV_EVT_WAIT)
            {
                lld_per_adv_did_set(act_id);
            }
            else
            {
                lld_per_adv_env[act_id]->update_did = true;
            }
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    GLOBAL_INT_RESTORE();
}
#endif // (BT_53)

uint8_t lld_per_adv_sync_info_get(uint8_t act_id, uint32_t *sync_ind_ts, uint16_t *pa_evt_cnt, struct le_ch_map *p_ch_map)
{
    FUNC_PATCH_ENTRY_4_PARAM_HAVE_RETURN(LLD_PER_ADV_PATCH_TYPE, LLD_PER_ADV_SYNC_INFO_GET_FUNC_BIT, uint8_t, act_id, sync_ind_ts, pa_evt_cnt, p_ch_map);

    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    GLOBAL_INT_DISABLE();

    if (lld_per_adv_env[act_id] != NULL)
    {
        // Point to parameters
        struct sch_arb_elt_tag *evt = &(lld_per_adv_env[act_id]->evt);
        struct lld_per_adv_env_tag *adv_par = lld_per_adv_env[act_id];

        *sync_ind_ts = evt->time.hs;
        *pa_evt_cnt = adv_par->pa_evt_cnt;
        *p_ch_map = ((adv_par->ch_map_upd.new_ch_map_pending) && (adv_par->pa_evt_cnt == adv_par->ch_map_upd.instant)) ? adv_par->ch_map_upd.ch_map : adv_par->ch_map;
        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return (status);
}

uint8_t lld_per_adv_init_info_get(uint8_t act_id, struct access_addr *aa, struct crc_init *crcinit, struct le_ch_map *p_ch_map)
{
    FUNC_PATCH_ENTRY_4_PARAM_HAVE_RETURN(LLD_PER_ADV_PATCH_TYPE, LLD_PER_ADV_INIT_INFO_GET_FUNC_BIT, uint8_t, act_id, aa, crcinit, p_ch_map);

    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    GLOBAL_INT_DISABLE();

    if (lld_per_adv_env[act_id] != NULL)
    {
        // Point to parameters
        struct lld_per_adv_env_tag *adv_par = lld_per_adv_env[act_id];

        memcpy(&aa->addr[0], &adv_par->aa.addr[0], ACCESS_ADDR_LEN);
        memcpy(&crcinit->crc[0], &adv_par->crcinit.crc[0], CRC_INIT_LEN);
        memcpy(&p_ch_map->map[0], &adv_par->ch_map.map[0], LE_CH_MAP_LEN);
        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return (status);
}

uint8_t lld_per_adv_ch_map_update(uint8_t act_id, struct le_ch_map *p_ch_map)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLD_PER_ADV_PATCH_TYPE, LLD_PER_ADV_CH_MAP_UPDATE_FUNC_BIT, uint8_t, act_id, p_ch_map);

    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    GLOBAL_INT_DISABLE();

    if (lld_per_adv_env[act_id] != NULL)
    {
        // Point to parameters
        struct lld_per_adv_env_tag *adv_par = lld_per_adv_env[act_id];

        // Reject if an update is already pending
        if (!adv_par->ch_map_upd.new_ch_map_pending && !memcmp(p_ch_map, &adv_par->ch_map_upd.ch_map, sizeof(struct le_ch_map)))
        {
            // Store new channel map
            adv_par->ch_map_upd.ch_map = *p_ch_map;
            adv_par->ch_map_upd.new_ch_map = true;

            status = CO_ERROR_NO_ERROR;
        }
    }

    GLOBAL_INT_RESTORE();

    return (status);
}

#if (BLE_BIS)
uint8_t lld_per_adv_big_update(uint8_t act_id, uint8_t big_id)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLD_PER_ADV_PATCH_TYPE, LLD_PER_ADV_BIG_UPDATE_FUNC_BIT, uint8_t, act_id, big_id);

    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    GLOBAL_INT_DISABLE();

    if (lld_per_adv_env[act_id] != NULL)
    {
        // Point to parameters
        struct lld_per_adv_env_tag *adv_par = lld_per_adv_env[act_id];

        // Update ACAD data only if BIG activity id changes
        if (adv_par->big_act_id != big_id)
        {
            adv_par->big_act_id = big_id;
            // reconstruct Periodic Advertising data
            if (adv_par->state == PER_ADV_EVT_WAIT)
            {
                lld_per_adv_chain_construct(act_id);
            }
            else
            {
                // mark that ACAD data must be updated
                adv_par->big_data_update = true;
            }
        }
        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return (status);
}
#endif // (BLE_BIS)

#if (BLE_PAST)
uint8_t lld_per_adv_info_get(uint8_t act_id, uint8_t *phy, uint16_t *intv, struct access_addr *aa, struct crc_init *crcinit, uint32_t *sync_ind_ts, uint16_t *pa_evt_cnt, struct le_ch_map *p_ch_map)
{
    FUNC_PATCH_ENTRY_VARI_PARAM_HAVE_RETURN(LLD_PER_ADV_PATCH_TYPE, LLD_PER_ADV_INFO_GET_FUNC_BIT, uint8_t, 8, act_id, phy, intv, aa, crcinit, sync_ind_ts, pa_evt_cnt, p_ch_map);

    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    GLOBAL_INT_DISABLE();

    if (lld_per_adv_env[act_id] != NULL)
    {
        // Point to parameters
        struct sch_arb_elt_tag *evt = &(lld_per_adv_env[act_id]->evt);
        struct lld_per_adv_env_tag *adv_par = lld_per_adv_env[act_id];

        *phy = adv_par->lld_sec_adv_rate;
        *intv = (adv_par->intv >> 2);

        memcpy(&aa->addr[0], &adv_par->aa.addr[0], ACCESS_ADDR_LEN);
        memcpy(&crcinit->crc[0], &adv_par->crcinit.crc[0], CRC_INIT_LEN);

        *sync_ind_ts = evt->time.hs;
        *pa_evt_cnt = adv_par->pa_evt_cnt;
        *p_ch_map = ((adv_par->ch_map_upd.new_ch_map_pending) && (adv_par->pa_evt_cnt == adv_par->ch_map_upd.instant)) ? adv_par->ch_map_upd.ch_map : adv_par->ch_map;
        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return (status);
}
#endif // (BLE_PAST)

#if BLE_CONLESS_CTE_TX
uint8_t lld_per_adv_cte_start(uint8_t act_id, struct lld_per_adv_cte_params *params)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLD_PER_ADV_PATCH_TYPE, LLD_PER_ADV_CTE_START_FUNC_BIT, uint8_t, act_id, params);

    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    GLOBAL_INT_DISABLE();

    if (lld_per_adv_env[act_id] != NULL)
    {
        // Copy the CTE parameters
        memcpy(&lld_per_adv_env[act_id]->cte_params, params, sizeof(struct lld_per_adv_cte_params));

        // If the event hasn't started yet, reconstruct the chain and configure antenna switching now
        // Otherwise, do this upon EOF
        if (lld_per_adv_env[act_id]->state == PER_ADV_EVT_WAIT)
        {
            // Reconstruct the periodic advertising chain
            lld_per_adv_chain_construct(act_id);

            // Configure antenna switching
            lld_per_adv_ant_switch_config(act_id);
        }
        else
        {
            lld_per_adv_env[act_id]->cte_update = true;
        }

        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return (status);
}

uint8_t lld_per_adv_cte_stop(uint8_t act_id)
{
    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LLD_PER_ADV_PATCH_TYPE, LLD_PER_ADV_CTE_STOP_FUNC_BIT, uint8_t, act_id);

    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    GLOBAL_INT_DISABLE();

    // Set the length to 0
    lld_per_adv_env[act_id]->cte_params.cte_len = NO_CTE;

    if (lld_per_adv_env[act_id] != NULL)
    {
        // If the event hasn't started yet, reconstruct the chain and configure antenna switching now
        // Otherwise, do this upon EOF
        if (lld_per_adv_env[act_id]->state == PER_ADV_EVT_WAIT)
        {
            // Reconstruct the periodic advertising chain
            lld_per_adv_chain_construct(act_id);

            // Configure antenna switching
            lld_per_adv_ant_switch_config(act_id);
        }
        else
        {
            lld_per_adv_env[act_id]->cte_update = true;
        }

        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return (status);
}
#endif // BLE_CONLESS_CTE_TX
#endif // (BLE_BROADCASTER)
///@} LLDPERADV
