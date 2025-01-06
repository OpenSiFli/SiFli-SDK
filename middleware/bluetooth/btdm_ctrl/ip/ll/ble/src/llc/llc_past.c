/**
 ****************************************************************************************
 *
 * @file llc_past.c
 *
 * @brief Handles the periodic advertising sync transfer procedure.
 *
 * Copyright (C) RivieraWaves 2009-2018
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup LLC_OP
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"
#include "em_map_ble.h"
#if ((BLE_PERIPHERAL || BLE_CENTRAL) && BLE_PAST)
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "compiler.h"    // __ARRAY_EMPTY and __STATIC define
#include "co_version.h"  // For device version
#include "co_bt.h"       // BT Standard defines (HCI, LLCP, Error codes)
#include "co_utils.h"    // For device version pdu preparation
#include "co_math.h"     // Math operations

#include "ke_msg.h"      // Kernel message
#include "ke_timer.h"    // Kernel timers

#include "aes.h"         // AES API

#include "llm.h"         // LLM API

#include "llc_int.h"     // Internal LLC API
#include "llc_llcp.h"    // Internal LLCP API

#include "lld.h"         // LLD API
#include "llm.h"         // LLM API

#include "hci.h"         // For HCI handler



/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// PAST operation indication structure definition
/*@TRACE*/
struct llc_op_past_ind
{
    /// Service data (value provided by the Host)
    uint16_t serv_data;

    /// Indicate if the periodic advertising is local or from another device
    bool local;

    /// Periodic advertising activity (for local transfer) or sync activity (for external transfer)
    uint8_t act_id;

    /// Extended advertising activity (for local transfer only)
    uint8_t ext_act_id;
} ;


/*
 * DEFINES
 ****************************************************************************************
 */


/*
 * MACROS
 ****************************************************************************************
 */


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

#if (BLE_OBSERVER)
/**
 ****************************************************************************************
 * @brief Extract/unpack Syncinfo
 ****************************************************************************************
 */
__STATIC bool llc_past_sync_info_unpack(struct sync_info *p_syncinfo_struct, uint8_t *p_syncinfo_tab)
{
    bool par_ok;
    DBG_MEM_INIT(p_syncinfo_struct, sizeof(struct sync_info));

    // Extract sync_packet_offset (13 bits), offset_units (1 bit), offset_adjust (1 bit), rfu (1 bit)
    p_syncinfo_struct->sync_offset = co_read16p(p_syncinfo_tab);
    p_syncinfo_struct->offset_units = GETB(p_syncinfo_struct->sync_offset, BLE_SYNC_OFFSET_UNITS);
    p_syncinfo_struct->offset_adjust = GETB(p_syncinfo_struct->sync_offset, BLE_SYNC_OFFSET_ADJUST);
    p_syncinfo_struct->sync_offset &= BLE_SYNC_OFFSET_MASK;
    p_syncinfo_tab += sizeof(uint16_t);

    // Extract interval (2 octets)
    p_syncinfo_struct->interval = co_read16p(p_syncinfo_tab);
    p_syncinfo_tab += sizeof(uint16_t);

    // Extract channel map (37 bits), SCA (3 bits)
    memcpy((void *)&p_syncinfo_struct->ch_map, p_syncinfo_tab, LE_CH_MAP_LEN);
    p_syncinfo_struct->sca = (p_syncinfo_struct->ch_map.map[LE_CH_MAP_LEN - 1] & BLE_SYNC_SCA_MASK) >> BLE_SYNC_SCA_LSB;
    p_syncinfo_struct->ch_map.map[LE_CH_MAP_LEN - 1] &= BLE_SYNC_CH_MAP_END_MASK;
    p_syncinfo_tab += LE_CH_MAP_LEN;

    // Extr act AA (4 octets)
    memcpy((void *)&p_syncinfo_struct->aa, p_syncinfo_tab, ACCESS_ADDR_LEN);
    p_syncinfo_tab += sizeof(uint32_t);

    // Extract CRCinit (3 octets)
    memcpy((void *)&p_syncinfo_struct->crcinit, p_syncinfo_tab, CRC_INIT_LEN);
    p_syncinfo_tab += CRC_INIT_LEN;

    // Extract event counter (2 octets)
    p_syncinfo_struct->evt_counter = co_read16p(p_syncinfo_tab);
    p_syncinfo_tab += sizeof(uint16_t);

    // Check paramaters of SyncInfo. A value of 0 for the Sync Packet Offset indicates that the time to the next AUX_SYNC_IND packet
    // is greater than can be represented. The interval shall not be less than the minimum (7.5ms)
    par_ok = ((p_syncinfo_struct->sync_offset != PER_SYNC_OFFSET_UNSPECIFIED) && (p_syncinfo_struct->interval >= PER_SYNC_INTERVAL_MIN));

    return par_ok;
}


/**
 ****************************************************************************************
 * @brief Sync on LL_PER_SYNC_IND reception
 ****************************************************************************************
 */
__STATIC void llc_past_rx(uint8_t link_id)
{
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];
    uint32_t add_drift;
    uint16_t curr_con_evt_cnt, target_pa_evt_cnt;
    uint32_t curr_time_hs, curr_con_time_hs, ref_con_time_hs, ref_pa_time_hs, target_pa_time_hs;
    uint16_t curr_con_time_hus, ref_con_time_hus, ref_pa_time_hus, target_pa_time_hus;
    uint32_t sync_offset_hus;
    struct llm_per_adv_sync_params params;
    uint16_t sca_a, sca_b, sca_c;
    uint8_t status;

    // Prepare parameters for LLM to create a sync activity
    params.syncinfo = llc_env_ptr->past.rx_info.syncinfo;
    params.act_id = link_id;
    params.adv_addr = llc_env_ptr->past.rx_info.adva;
    params.adv_rpa = llc_env_ptr->past.rx_info.adv_rpa;
    params.adv_addr_type = GETB(llc_env_ptr->past.rx_info.sid_atype_sca, LLCP_PER_SYNC_IND_ATYPE);
    params.adv_sid = GETF(llc_env_ptr->past.rx_info.sid_atype_sca, LLCP_PER_SYNC_IND_SID);
    params.rate = co_phy_mask_to_rate[llc_env_ptr->past.rx_info.phy];
    params.skip = llc_env_ptr->past.skip;
    params.sync_to = llc_env_ptr->past.sync_to;
    params.id = llc_env_ptr->past.rx_info.id;
#if (BT_53)
    params.adv_rep_en = (llc_env_ptr->past.mode == SYNC_REP_EN) || (llc_env_ptr->past.mode == SYNC_REP_EN_FILT_EN);
    params.adi_filt_en = (llc_env_ptr->past.mode == SYNC_REP_EN_FILT_EN);
#else // !(BT_53)
    params.adv_rep_en = (llc_env_ptr->past.mode == SYNC_REP_EN);
#endif // !(BT_53)
    params.sync_cte_type = llc_env_ptr->past.cte_type;

    /*
     * Timing computations
     */

    // Get current connection time
    lld_con_time_get(link_id, &curr_con_evt_cnt, &curr_con_time_hs, &curr_con_time_hus);

    // Compute reference connection event time
    // currEvent - 16384 < connEventCount < currEvent + 16384 (modulo 65536)
    if (((llc_env_ptr->past.rx_info.con_evt_cnt - curr_con_evt_cnt) & 0xFFFF) < 16384) // connEventCount is in the future
    {
        uint16_t diff_con_evt_cnt = llc_env_ptr->past.rx_info.con_evt_cnt - curr_con_evt_cnt;
        ref_con_time_hs = CLK_ADD_2(curr_con_time_hs, diff_con_evt_cnt * llc_env_ptr->con_params.interval * 4);
    }
    else // connEventCount is in the past
    {
        uint16_t diff_con_evt_cnt = (curr_con_evt_cnt - llc_env_ptr->past.rx_info.con_evt_cnt) & 0x3FFF;
        ref_con_time_hs = CLK_SUB(curr_con_time_hs, diff_con_evt_cnt * llc_env_ptr->con_params.interval * 4);
    }
    ref_con_time_hus = curr_con_time_hus;

    // Compute reference periodic advertising event time
    sync_offset_hus = (params.syncinfo.sync_offset * (params.syncinfo.offset_units ? 300 : 30) + (params.syncinfo.offset_adjust ? OFFSET_ADJUST : 0)) * 2; // calculate offset (half-us)
    sync_offset_hus += ref_con_time_hus;
    ref_pa_time_hs = CLK_ADD_2(ref_con_time_hs, sync_offset_hus / HALF_SLOT_SIZE);
    ref_pa_time_hus = CO_MOD(sync_offset_hus, HALF_SLOT_SIZE);

    // Get latest periodic advertising event
    curr_time_hs = lld_read_clock();
    if (CLK_GREATER_THAN(curr_time_hs, ref_pa_time_hs))
    {
        uint32_t diff_intv = CLK_SUB(curr_time_hs, ref_pa_time_hs) / (4 * params.syncinfo.interval) + 1;
        target_pa_time_hs = CLK_ADD_2(ref_pa_time_hs, diff_intv * (4 * params.syncinfo.interval));
        target_pa_evt_cnt = params.syncinfo.evt_counter + diff_intv;
    }
    else
    {
        uint32_t diff_intv = (CLK_SUB(ref_pa_time_hs, curr_time_hs) / (4 * params.syncinfo.interval)) + 1;
        target_pa_time_hs = CLK_SUB(ref_pa_time_hs, diff_intv * (4 * params.syncinfo.interval));
        target_pa_evt_cnt = params.syncinfo.evt_counter - diff_intv;
    }
    target_pa_time_hus = ref_pa_time_hus;

    /*
     * Additional drift considered on the time reference, depends on last periodic advertising event observed, and periodic
     * advertiser clock accuracy
     */
    sca_a = co_sca2ppm[params.syncinfo.sca];
    sca_b = co_sca2ppm[GETF(llc_env_ptr->past.rx_info.sid_atype_sca, LLCP_PER_SYNC_IND_SCA)];
    sca_c = rwip_max_drift_get();

    // Drift from last observed periodic advertising event or last observed connection event
    add_drift =  2 * 2 * (co_abs(target_pa_evt_cnt                        - llc_env_ptr->past.rx_info.last_pa_evt_cnt)) * (2 * params.syncinfo.interval) * (sca_a + sca_c) / 1600; // half-slots * ppm * 625 half-us / 1000000;
    add_drift += 2 * 2 * (co_abs(llc_env_ptr->past.rx_info.con_evt_cnt_rx - llc_env_ptr->past.rx_info.sync_con_evt_cnt)) * (2 * params.syncinfo.interval) * (sca_b + sca_c) / 1600; // half-slots * ppm * 625 half-us / 1000000;
    add_drift += add_drift + ((add_drift * (sca_a + sca_b + sca_c)) / 1000000);

    // Set timing information for LLM
    params.add_drift = add_drift;
    params.base_cnt = target_pa_time_hs;
    params.fine_cnt = target_pa_time_hus;
    params.syncinfo.sync_offset = 0;
    params.syncinfo.evt_counter = target_pa_evt_cnt;

    // Request LLM to create a sync activity
    status = llm_per_adv_sync(&params);

    if ((status != CO_ERROR_NO_ERROR) && (status != CO_ERROR_CON_ALREADY_EXISTS))
    {
        struct hci_le_per_adv_sync_transf_rec_evt *evt = KE_MSG_ALLOC(HCI_LE_EVENT, BLE_LINKID_TO_CONHDL(link_id), HCI_LE_META_EVT_CODE, hci_le_per_adv_sync_transf_rec_evt);
        evt->subcode        = HCI_LE_PER_ADV_SYNC_TRANSF_REC_EVT_SUBCODE;
        evt->status         = status;
        evt->conhdl         = BLE_LINKID_TO_CONHDL(link_id);
        evt->adv_addr       = llc_env_ptr->past.rx_info.adva;
        evt->adv_addr_type  = GETB(llc_env_ptr->past.rx_info.sid_atype_sca, LLCP_PER_SYNC_IND_ATYPE);
        evt->adv_sid        = GETF(llc_env_ptr->past.rx_info.sid_atype_sca, LLCP_PER_SYNC_IND_SID);
        evt->adv_ca         = GETF(llc_env_ptr->past.rx_info.sid_atype_sca, LLCP_PER_SYNC_IND_SCA);
        evt->serv_data      = llc_env_ptr->past.rx_info.id;
        evt->phy            = co_phy_mask_to_value[llc_env_ptr->past.rx_info.phy];
        evt->sync_hdl       = 0x0000;
        evt->interval       = 0;
        hci_send_2_host(evt);
    }
}

__STATIC void llc_past_rpa_res_cb(uint8_t status, uint8_t index, uint32_t metainfo, const struct bd_addr *p_addr,
                                  const struct irk *p_irk)
{
    uint8_t link_id = metainfo;
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];

    // Check if a command is currently handled, drop the message if not the case.
    if (llc_env_ptr != NULL)
    {
        // Check if address has been resolved
        if (index < BLE_RESOL_ADDR_LIST_MAX)
        {
            struct ral_entry *ral = llm_ral_get();

            // Move received BD address as RPA
            llc_env_ptr->past.rx_info.adv_rpa = llc_env_ptr->past.rx_info.adva;

            // Retrieve identity address from resolving address list (RAL)
            llc_env_ptr->past.rx_info.adva = ral[index].bd_addr;
            SETB(llc_env_ptr->past.rx_info.sid_atype_sca, LLCP_PER_SYNC_IND_ATYPE, ral[index].addr_type);
        }

        // Start synchronization
        llc_past_rx(link_id);
    }
}


/**
 ****************************************************************************************
 * @brief Handles the PAST procedure indication message.
 *
 * PlantUML procedure description
 *
 * @startuml llc_past_op_start.png
 * title : Periodic Advertising Sync Transfer procedure start
 * participant LLC
 *  --> LLC : LLC_OP_PAST_IND
 * LLC -> LLC: llc_op_past_ind_handler()
 * activate LLC
 * hnote over LLC : LOC_PROC = Busy
 * LLC -> LLC: Send LL_PERIODIC_SYNC_IND
 * deactivate LLC
 * @enduml
 *
 ****************************************************************************************
 */

int llc_op_past_ind_handler(ke_msg_id_t const msgid, struct llc_op_past_ind *param,
                            ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Current message status
    int msg_status = KE_MSG_CONSUMED;
    uint8_t link_id = KE_IDX_GET(dest_id);
    struct bd_addr adva;
    uint8_t sid, atype;

    do
    {
        // Check if state in disconnected state
        if (llc_is_disconnecting(link_id))
            break;

        // check if another local procedure is on-going
        if (llc_proc_id_get(link_id, LLC_PROC_LOCAL) != LLC_PROC_NONE)
        {
            // process this message later
            msg_status = KE_MSG_SAVED;
            break;
        }

        if (param->local)
        {
#if (BLE_BROADCASTER)
            // Get periodic advertising train identifiers from LLM ADV
            if (llm_adv_set_id_get(param->ext_act_id, &sid, &atype, &adva) != CO_ERROR_NO_ERROR)
#endif // (BLE_BROADCASTER)
                break;
        }
        else
        {
            // Get periodic advertising train identifiers from LLM SCAN
            if (llm_scan_sync_info_get(param->act_id, &sid, &atype, &adva) != CO_ERROR_NO_ERROR)
                break;
        }

        {
            // Send LL_PERIODIC_SYNC_IND
            struct ll_per_sync_ind pdu;

            uint32_t sync_ind_ts, ref_con_evt_ts;
            uint16_t ref_pa_evt_cnt, sync_pkt_offset, ref_con_evt_cnt, sync_ind_bit_off = 0, per_adv_intv, ref_con_evt_bit_off;
            uint8_t offset_units, offset_adjust, local_sca, per_adv_sca, per_adv_phy;
            int32_t diff_hs, diff_us;
            uint16_t con_intv;

            local_sca = rwip_sca_get();

            pdu.op_code = LL_PER_SYNC_IND_OPCODE;
            pdu.id = param->serv_data;

            memset(&pdu.sync_info[0], 0x00, BLE_EXT_SYNC_LEN);

            if (param->local)
            {
#if (BLE_BROADCASTER)
                // Get information from the periodic advertiser
                lld_per_adv_info_get(param->act_id, &per_adv_phy, &per_adv_intv, (struct access_addr *)&pdu.sync_info[9], (struct crc_init *)&pdu.sync_info[13], &sync_ind_ts, &ref_pa_evt_cnt, (struct le_ch_map *)&pdu.sync_info[4]);
                per_adv_sca = local_sca;
#endif // (BLE_BROADCASTER)
            }
            else
            {
                // Get information from the periodic sync driver
                lld_sync_info_get(param->act_id, &per_adv_phy, &per_adv_intv, (struct access_addr *)&pdu.sync_info[9], (struct crc_init *)&pdu.sync_info[13], &sync_ind_ts, &sync_ind_bit_off, &ref_pa_evt_cnt, (struct le_ch_map *)&pdu.sync_info[4], &per_adv_sca);
            }

            // Get the latest synchronization information from the connection driver
            lld_con_time_get(link_id, &ref_con_evt_cnt, &ref_con_evt_ts, &ref_con_evt_bit_off);

            // lastPaEventCount is the last PA event transmitted or received
            pdu.last_pa_evt_cnt = ref_pa_evt_cnt;
            // SyncConnEventCount is the last connection event transmitted or received
            pdu.sync_con_evt_cnt = ref_con_evt_cnt;

            // Compute timings
            offset_adjust = 0;

            con_intv = llc_env[link_id]->con_params.interval;

            // Make sure that the AUX_SYNC_IND comes after the reference connection event
            diff_hs  = CLK_DIFF(ref_con_evt_ts, sync_ind_ts);
            diff_us = ((diff_hs * HALF_SLOT_SIZE) >> 1) + ((sync_ind_bit_off - ref_con_evt_bit_off + 1) >> 1);
            while (diff_us <= 0)
            {
                sync_ind_ts = CLK_ADD_2(sync_ind_ts, per_adv_intv * 4);
                ref_pa_evt_cnt++;
                diff_hs = CLK_DIFF(ref_con_evt_ts, sync_ind_ts);
                diff_us = ((diff_hs * HALF_SLOT_SIZE) >> 1) + ((sync_ind_bit_off - ref_con_evt_bit_off + 1) >> 1);
            }

            // Make sure that the time difference is less than 5 seconds
            while (diff_us > OFFSET_DIFF_MAX)
            {
                ref_con_evt_ts = CLK_ADD_2(ref_con_evt_ts, con_intv * 4);
                ref_con_evt_cnt++;
                diff_hs = CLK_DIFF(ref_con_evt_ts, sync_ind_ts);
                diff_us = ((diff_hs * HALF_SLOT_SIZE) >> 1) - ((ref_con_evt_bit_off + 1) >> 1);
            }

            if (diff_us < OFFSET_ADJUST)
            {
                sync_pkt_offset = (diff_us < OFFSET_THRESHOLD) ? (diff_us / 30) : (diff_us / 300);
                offset_units = (diff_us < OFFSET_THRESHOLD) ? 0 : 1;
            }
            else
            {
                offset_adjust = 1;
                offset_units = 1;
                diff_us -= OFFSET_ADJUST;
                sync_pkt_offset = diff_us / 300;
            }

            // Sync Packet Offset, Offset Units, Offset Adjust
            pdu.sync_info[0] = sync_pkt_offset & 0xFF;
            pdu.sync_info[1] = ((sync_pkt_offset >> 8) & 0x1F) | (offset_units << 5) | (offset_adjust << 6);

            // Interval
            pdu.sync_info[2] = per_adv_intv & 0xFF;
            pdu.sync_info[3] = (per_adv_intv >> 8) & 0xFF;

            // Copy SCA
            pdu.sync_info[8] = (pdu.sync_info[8] & 0x1F) | (per_adv_sca << 5);

            // Event Counter
            pdu.sync_info[16] = ref_pa_evt_cnt & 0xFF;
            pdu.sync_info[17] = (ref_pa_evt_cnt >> 8) & 0xFF;

            pdu.con_evt_cnt = ref_con_evt_cnt;

            pdu.sid_atype_sca = (sid & 0x0F) | ((atype & 0x01) << 4) | ((local_sca & 0x07) << 5);
            pdu.phy = co_rate_to_phy_mask[per_adv_phy];
            pdu.adva = adva;

            llc_llcp_send(link_id, (union llcp_pdu *) &pdu, NULL);
        }
    }
    while (0);

    return (msg_status);
}
#endif //(BLE_OBSERVER)


/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handles the reception of a LLCP periodic sync indication
 *
 * @param[in] link_id        Link identifier on which the pdu will be sent.
 * @param[in] pdu            LLCP PDU information received
 * @param[in] event_cnt      Event counter value when PDU has been received
 *
 * @return status code of handler:
 *    - CO_ERROR_NO_ERROR:               Nothing more to do
 *    - CO_ERROR_TERMINATED_MIC_FAILURE: Immediately disconnect the link
 *    - others:                          Send an LLCP_REJECT_IND or LLCP_REJECT_IND_EXT
 ****************************************************************************************
 */
int ll_per_sync_ind_handler(uint8_t link_id, struct ll_per_sync_ind *pdu, uint16_t event_cnt)
{
    int status = CO_ERROR_NO_ERROR;

#if (BLE_OBSERVER)
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];

    do
    {
        // Check if a synchronization should be attempted
        if (llc_env_ptr->past.mode == NO_SYNC)
            break;

        // Extract sync info
        if (!llc_past_sync_info_unpack(&llc_env_ptr->past.rx_info.syncinfo, &pdu->sync_info[0]))
            break;

        /* If the PHY field of the LL_PERIODIC_SYNC_IND PDU has no bits or more than one bit set, or the bit
           set corresponds to a PHY that the recipient does not support or is reserved for future use, the
           recipient shall ignore the PDU.*/
        if (!pdu->phy || (CO_BIT_CNT(pdu->phy) > 1) || (pdu->phy & ~PHY_ALL)
#if !(BLE_PHY_2MBPS_SUPPORT)
                || (pdu->phy == PHY_2MBPS_BIT)
#endif //!(BLE_PHY_2MBPS_SUPPORT)
#if !(BLE_PHY_CODED_SUPPORT)
                || (pdu->phy == PHY_CODED_BIT)
#endif //!(BLE_PHY_CODED_SUPPORT)
           )
            break;

        // Copy received parameters
        llc_env_ptr->past.rx_info.id               = pdu->id;
        llc_env_ptr->past.rx_info.adva             = pdu->adva;
        llc_env_ptr->past.rx_info.con_evt_cnt      = pdu->con_evt_cnt;
        llc_env_ptr->past.rx_info.sync_con_evt_cnt = pdu->sync_con_evt_cnt;
        llc_env_ptr->past.rx_info.last_pa_evt_cnt  = pdu->last_pa_evt_cnt;
        llc_env_ptr->past.rx_info.phy              = pdu->phy;
        llc_env_ptr->past.rx_info.sid_atype_sca    = pdu->sid_atype_sca;
        llc_env_ptr->past.rx_info.con_evt_cnt_rx   = event_cnt;
        memset(&llc_env_ptr->past.rx_info.adv_rpa, 0, BD_ADDR_LEN);

        // Check if RPA needs to be resolved
        if ((GETB(pdu->sid_atype_sca, LLCP_PER_SYNC_IND_ATYPE) == ADDR_RAND)
                && ((pdu->adva.addr[BD_ADDR_LEN - 1] & BD_ADDR_RND_ADDR_TYPE_MSK) == BD_ADDR_RSLV))
        {
            struct irk irks[BLE_RAL_MAX];
            struct ral_entry *ral = llm_ral_get();

            // Initialize table of IRKs
            memset(irks, 0, sizeof(irks));

            // If at least one peer IRK is valid
            for (uint8_t position = 0; position < BLE_RAL_MAX; position++)
            {
                if ((ral[position].addr_type != 0xFF) && memcmp(ral[position].peer_irk.key, co_null_key, KEY_LEN))
                {
                    // Copy peer IRK
                    memcpy(&irks[position].key[0], &ral[position].peer_irk.key[0], KEY_LEN);
                }
            }

            // Start address resolution
            aes_rpa_resolve(BLE_RAL_MAX, (const aes_key_t *)irks, (uint8_t *)&llc_env_ptr->past.rx_info.adva,
                            (aes_rpa_func_result_cb) &llc_past_rpa_res_cb, link_id);
        }
        else
        {
            // Start synchronization
            llc_past_rx(link_id);
        }

    }
    while (0);
#endif //(BLE_OBSERVER)

    return status;
}

#if (BLE_OBSERVER)
/**
 ****************************************************************************************
 * @brief Handles the command HCI LE Periodic Advertising Sync Transfer.
 *
 * @param[in] link_id Link Identifier
 * @param[in] param   Pointer to the parameters of the message.
 * @param[in] opcode  HCI Operation code
 ****************************************************************************************
 */
int hci_le_per_adv_sync_transf_cmd_handler(uint8_t link_id, struct hci_le_per_adv_sync_transf_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    do
    {
#if (BLE_ADV_LEGACY_ITF)
        // Check current interface version
        if (llm_is_adv_itf_legacy())
            break;

        // Use extended interface
        llm_adv_itf_extended_set();
#endif //(BLE_ADV_LEGACY_ITF)
        // check if state is Free or in disconnection state
        if (llc_is_disconnecting(link_id))
            break;

        // If the remote device has not indicated support for the Periodic Advertising Sync Transfer - Recipient feature,
        // the Controller shall return the error code Unsupported Remote Feature / Unsupported LMP Feature (0x1A)
        if (!llc_le_feature_check(link_id, BLE_FEAT_PER_ADV_SYNC_TRANSF_RX))
        {
            status = CO_ERROR_UNSUPPORTED_REMOTE_FEATURE;
            break;
        }

        // Get periodic advertising train identifiers from LLM
        status = llm_scan_sync_info_get(BLE_SYNCHDL_TO_ACTID(param->sync_hdl), NULL, NULL, NULL);

        if (status != CO_ERROR_NO_ERROR)
            break;

        // Send LLC_OP_PAST_IND
        {
            ke_task_id_t llc_id = KE_BUILD_ID(TASK_LLC, link_id);
            struct llc_op_past_ind *past_req = KE_MSG_ALLOC(LLC_OP_PAST_IND, llc_id, llc_id, llc_op_past_ind);
            past_req->serv_data       = param->serv_data;
            past_req->local           = false;
            past_req->act_id          = BLE_SYNCHDL_TO_ACTID(param->sync_hdl);
            ke_msg_send(past_req);
        }
    }
    while (0);

    // Send the command complete event
    llc_cmd_cmp_send(link_id, opcode, status);

    return (KE_MSG_CONSUMED);
}
#endif //(BLE_OBSERVER)

#if (BLE_BROADCASTER)
/**
 ****************************************************************************************
 * @brief Handles the command HCI LE Periodic Advertising Set Info Transfer.
 *
 * @param[in] link_id Link Identifier
 * @param[in] param   Pointer to the parameters of the message.
 * @param[in] opcode  HCI Operation code
 ****************************************************************************************
 */
int hci_le_per_adv_set_info_transf_cmd_handler(uint8_t link_id, struct hci_le_per_adv_set_info_transf_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    do
    {
        uint8_t ext_adv_id, per_adv_id;

#if (BLE_ADV_LEGACY_ITF)
        // Check current interface version
        if (llm_is_adv_itf_legacy())
            break;

        // Use extended interface
        llm_adv_itf_extended_set();
#endif //(BLE_ADV_LEGACY_ITF)

        // check if state is Free or in disconnection state
        if (llc_is_disconnecting(link_id))
            break;

        // If the remote device has not indicated support for the Periodic Advertising Sync Transfer - Recipient feature,
        // the Controller shall return the error code Unsupported Remote Feature / Unsupported LMP Feature (0x1A)
        if (!llc_le_feature_check(link_id, BLE_FEAT_PER_ADV_SYNC_TRANSF_RX))
        {
            status = CO_ERROR_UNSUPPORTED_REMOTE_FEATURE;
            break;
        }

        // Get activity IDs of the extended and periodic advertising activities linked to adv_hdl
        status = llm_adv_act_id_get(param->adv_hdl, &ext_adv_id, &per_adv_id);

        if (status != CO_ERROR_NO_ERROR)
            break;

        // Get advertising set identifiers
        status = llm_adv_set_id_get(ext_adv_id, NULL, NULL, NULL);

        if (status != CO_ERROR_NO_ERROR)
            break;

        // Send LLC_OP_PAST_IND
        {
            ke_task_id_t llc_id = KE_BUILD_ID(TASK_LLC, link_id);
            struct llc_op_past_ind *past_req = KE_MSG_ALLOC(LLC_OP_PAST_IND, llc_id, llc_id, llc_op_past_ind);
            past_req->serv_data       = param->serv_data;
            past_req->local           = true;
            past_req->act_id          = per_adv_id;
            past_req->ext_act_id      = ext_adv_id;
            ke_msg_send(past_req);
        }
    }
    while (0);

    // Send the command complete event
    llc_cmd_cmp_send(link_id, opcode, status);

    return (KE_MSG_CONSUMED);
}
#endif // (BLE_BROADCASTER)

/**
 ****************************************************************************************
 * @brief Handles the command HCI LE set Periodic Advertising Sync Transfer parameters.
 *
 * @param[in] link_id Link Identifier
 * @param[in] param   Pointer to the parameters of the message.
 * @param[in] opcode  HCI Operation code
 ****************************************************************************************
 */
int hci_le_set_per_adv_sync_transf_param_cmd_handler(uint8_t link_id, struct hci_le_set_per_adv_sync_transf_param_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];

    do
    {
#if (BLE_ADV_LEGACY_ITF)
        // Check current interface version
        if (llm_is_adv_itf_legacy())
            break;

        // Use extended interface
        llm_adv_itf_extended_set();
#endif //(BLE_ADV_LEGACY_ITF)

        // check if state is Free or in disconnection state
        if (llc_is_disconnecting(link_id))
            break;

        status = CO_ERROR_INVALID_HCI_PARAM;

#if (BT_53)
        if (param->mode > SYNC_REP_EN_FILT_EN)
#else // !(BT_53)
        if (param->mode > SYNC_REP_EN)
#endif // !(BT_53)
            break;

        if (param->skip > SYNC_SKIP_MAX)
            break;

        if ((param->sync_to < SYNC_TIMEOUT_MIN) || (param->sync_to > SYNC_TIMEOUT_MAX))
            break;

        if ((param->cte_type & ~(NO_SYNC_AOA_BIT | NO_SYNC_AOD_1US_BIT | NO_SYNC_AOD_2US_BIT | NO_SYNC_NO_CTE_BIT)))
            break;

        // Save parameters
        llc_env_ptr->past.mode = param->mode;
        llc_env_ptr->past.cte_type = param->cte_type;
        llc_env_ptr->past.skip = param->skip;
        llc_env_ptr->past.sync_to = param->sync_to;

        status = CO_ERROR_NO_ERROR;

    }
    while (0);

    // Send the command complete event
    llc_cmd_cmp_send(link_id, opcode, status);

    return (KE_MSG_CONSUMED);
}

#endif // (BLE_PERIPHERAL || BLE_CENTRAL)
/// @} LLC_OP
