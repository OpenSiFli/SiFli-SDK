/**
 ****************************************************************************************
 *
 * @file llm_adv.c
 *
 * @brief Definition of the functions used by the link layer manager
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup LLM
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"       // stack configuration

#if (BLE_BROADCASTER)

#include <string.h>
#include "co_bt.h"             // BLE standard definitions
#include "co_utils.h"

#include "ke_mem.h"            // kernel memory
#include "ke_msg.h"            // kernel messages
#include "ke_task.h"           // kernel task

#include "sch_plan.h"          // Scheduling Planner
#include "aes.h"               // AES definitions

#include "ble_util_buf.h"      // buffer allocation
#include "reg_access.h"        // access to EM buffers

#include "ble_util.h"            // BLE utility functions

#include "llm.h"               // link layer manager definitions
#include "llc.h"               // link layer controller definitions
#include "lld.h"               // link layer driver definitions

#include "llm_int.h"           // link layer manager internal definitions

#include "rwip.h"

#if HCI_PRESENT
    #include "hci.h"               // host controller interface
#endif //HCI_PRESENT

/*
 * DEFINES
 ****************************************************************************************
 */

#if (BLE_CFG_MAX_ADV_DATA_LEN > HOST_ADV_DATA_LEN_MAX)
    #error "The total amount of Host Advertising Data before fragmentation shall not exceed 1650 octets"
#endif //(BLE_CFG_MAX_ADV_DATA_LEN > HOST_ADV_DATA_LEN_MAX)

/*
 * CONSTANTS DEFINITION
 *****************************************************************************************
 */

#if (BLE_ADV_LEGACY_ITF)
/// Conversion table from advertising type to advertising event properties values for legacy PDUs
__STATIC const uint8_t adv_evt_type2prop[] =
{
    [ADV_CONN_UNDIR   ] = ADV_IND,
    [ADV_CONN_DIR     ] = ADV_DIRECT_HI_IND,
    [ADV_DISC_UNDIR   ] = ADV_SCAN_IND,
    [ADV_NONCONN_UNDIR] = ADV_NONCONN_IND,
    [ADV_CONN_DIR_LDC ] = ADV_DIRECT_LO_IND
};
#endif //(BLE_ADV_LEGACY_ITF)


/*
 * LOCAL FUNCTIONS DEFINITION
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @brief Call back definition of the function that can handle result of an AES RPA generation algorithm
 *
 * @param[in] status       Execution status
 * @param[in] aes_res      16 bytes block result
 * @param[in] metainfo     Metadata information provided by API user (see \glos{METAINFO}) (used for storing activity ID)
 ****************************************************************************************
 */
__STATIC void llm_adv_rpa_gen_cb(uint8_t status, const uint8_t *aes_res, uint32_t metainfo)
{
    uint8_t act_id = metainfo;

    ASSERT_INFO(act_id < BLE_ACTIVITY_MAX_CFG, act_id, metainfo);

    // Check if activity is in advertising state
    if ((status == CO_ERROR_NO_ERROR) && (llm_env.act_info[act_id].state == LLM_ADV_EN))
    {
        // Store local RPA
        memcpy(&llm_env.act_info[act_id].info.adv.local_rpa.addr[0], aes_res, BD_ADDR_LEN);
    }
}

/// Release an advertising set
__STATIC void llm_adv_set_release(uint8_t act_id)
{
    // Release complete advertising data, if present
    if (llm_env.act_info[act_id].info.adv.adv_data.curr_buf)
    {
        ble_util_buf_adv_tx_free(llm_env.act_info[act_id].info.adv.adv_data.curr_buf);
        llm_env.act_info[act_id].info.adv.adv_data.curr_buf = 0;
        llm_env.act_info[act_id].info.adv.adv_data.curr_len = 0;
    }
    // Release advertising data in construction, if present
    if (llm_env.act_info[act_id].info.adv.adv_data.new_buf)
    {
        ble_util_buf_adv_tx_free(llm_env.act_info[act_id].info.adv.adv_data.new_buf);
        llm_env.act_info[act_id].info.adv.adv_data.new_buf = 0;
        llm_env.act_info[act_id].info.adv.adv_data.new_len = 0;
    }

    // Release complete scan response data, if present
    if (llm_env.act_info[act_id].info.adv.scan_rsp_data.curr_buf)
    {
        ble_util_buf_adv_tx_free(llm_env.act_info[act_id].info.adv.scan_rsp_data.curr_buf);
        llm_env.act_info[act_id].info.adv.scan_rsp_data.curr_buf = 0;
        llm_env.act_info[act_id].info.adv.scan_rsp_data.curr_len = 0;
    }
    // Release scan response data in construction, if present
    if (llm_env.act_info[act_id].info.adv.scan_rsp_data.new_buf)
    {
        ble_util_buf_adv_tx_free(llm_env.act_info[act_id].info.adv.scan_rsp_data.new_buf);
        llm_env.act_info[act_id].info.adv.scan_rsp_data.new_buf = 0;
        llm_env.act_info[act_id].info.adv.scan_rsp_data.new_len = 0;
    }

    // Release Host parameters
    if (llm_env.act_info[act_id].host_params != NULL)
    {
        ke_msg_free(ke_param2msg(llm_env.act_info[act_id].host_params));
        llm_env.act_info[act_id].host_params = NULL;
    }

#if BLE_CONLESS_CTE_TX
    // Release CTE Tx parameters if they exist
    if (llm_env.act_info[act_id].info.adv.cte_tx_params != NULL)
    {
        ke_msg_free(ke_param2msg(llm_env.act_info[act_id].info.adv.cte_tx_params));
        llm_env.act_info[act_id].info.adv.cte_tx_params = NULL;
    }
#endif // BLE_CONLESS_CTE_TX

#if (EAVESDROPPING_SUPPORT)
    // Release custom advertising parameters if they exist
    if (llm_env.act_info[act_id].info.adv.cust_adv_params != NULL)
    {
        ke_msg_free(ke_param2msg(llm_env.act_info[act_id].info.adv.cust_adv_params));
        llm_env.act_info[act_id].info.adv.cust_adv_params = NULL;
    }

    // Release manual advertising parameters if they exist
    if (llm_env.act_info[act_id].info.adv.manual_adv_params != NULL)
    {
        ke_msg_free(ke_param2msg(llm_env.act_info[act_id].info.adv.manual_adv_params));
        llm_env.act_info[act_id].info.adv.manual_adv_params = NULL;
    }
#endif //(EAVESDROPPING_SUPPORT)

    // Check if a periodic advertising is linked to this advertising set
    if (llm_env.act_info[act_id].info.adv.per_act_id < BLE_ACTIVITY_MAX_CFG)
    {
        uint8_t per_act_id = llm_env.act_info[act_id].info.adv.per_act_id;

        // Release complete advertising data, if present
        if (llm_env.act_info[per_act_id].info.per_adv.adv_data.curr_buf)
        {
            ble_util_buf_adv_tx_free(llm_env.act_info[per_act_id].info.per_adv.adv_data.curr_buf);
            llm_env.act_info[per_act_id].info.per_adv.adv_data.curr_buf = 0;
            llm_env.act_info[per_act_id].info.per_adv.adv_data.curr_len = 0;
        }
        // Release advertising data in construction, if present
        if (llm_env.act_info[per_act_id].info.per_adv.adv_data.new_buf)
        {
            ble_util_buf_adv_tx_free(llm_env.act_info[per_act_id].info.per_adv.adv_data.new_buf);
            llm_env.act_info[per_act_id].info.per_adv.adv_data.new_buf = 0;
            llm_env.act_info[per_act_id].info.per_adv.adv_data.new_len = 0;
        }

        // Release Host parameters
        if (llm_env.act_info[per_act_id].host_params != NULL)
        {
            ke_msg_free(ke_param2msg(llm_env.act_info[per_act_id].host_params));
            llm_env.act_info[per_act_id].host_params = NULL;
        }

        // Release activity identifier
        llm_env.act_info[per_act_id].state = LLM_FREE;
    }

    // Release activity identifier
    llm_env.act_info[act_id].state = LLM_FREE;
}

// Checks if the connectable advertising data can fit in a single packet and returns a status
__STATIC uint8_t llm_adv_con_len_check(struct hci_le_set_ext_adv_param_cmd *ext_param, uint16_t adv_data_len)
{
    ASSERT_ERR(!(ext_param->adv_evt_properties & ADV_LEGACY));
    ASSERT_ERR(ext_param->adv_evt_properties & ADV_CON);

    uint8_t status = CO_ERROR_NO_ERROR;
    uint8_t ext_header_len = 0;

    // First calculate the extended header length
    ext_header_len += BD_ADDR_LEN; // AdvA
    if (ext_param->adv_evt_properties & ADV_DIRECT)
    {
        ext_header_len += BD_ADDR_LEN; // TargetA
    }
    ext_header_len += BLE_EXT_ADI_LEN; // ADI
    if (ext_param->adv_evt_properties & ADV_TX_PWR)
    {
        ext_header_len += BLE_EXT_TX_PWR_LEN; // TxPower
    }
    ext_header_len += 1; // Extended header flags field (1 octet)

    // // Check if the data can fit in a single packet
    if ((ext_header_len + adv_data_len) > BLE_ADV_FRAG_SIZE_TX)
    {
        status = CO_ERROR_INVALID_HCI_PARAM;
    }

    return (status);
}

// Checks if the adv/scan_rsp data has changed
__STATIC bool llm_adv_is_data_changed(uint16_t new_len, uint16_t old_len)
{
    bool data_chg = true;

    if ((new_len == old_len) && (new_len == 0))
    {
        data_chg = false;
    }

    return (data_chg);
}

#if (BLE_ADV_LEGACY_ITF)
// Set the default legacy advertising parameters (using the extended format)
__STATIC void llm_adv_set_dft_params(uint8_t act_id)
{
    ASSERT_ERR(llm_env.act_info[act_id].host_params == NULL);

    struct hci_le_set_ext_adv_param_cmd *ext_param;

    // Allocate extended HCI command for storing default parameters
    llm_env.act_info[act_id].host_params = KE_MSG_ALLOC(0, 0, 0, hci_le_set_ext_adv_param_cmd);

    // Fill with default parameters
    ext_param = (struct hci_le_set_ext_adv_param_cmd *) llm_env.act_info[act_id].host_params;
    ext_param->adv_hdl              = LLM_ADV_HDL_INVL;
    ext_param->adv_evt_properties   = ADV_IND;
    ext_param->prim_adv_intv_max[2] = 0x00;
    ext_param->prim_adv_intv_max[1] = (uint8_t)(ADV_INTERVAL_DFT >> 8);
    ext_param->prim_adv_intv_max[0] = (uint8_t)(ADV_INTERVAL_DFT >> 0);
    ext_param->prim_adv_intv_min[2] = 0x00;
    ext_param->prim_adv_intv_min[1] = (uint8_t)(ADV_INTERVAL_DFT >> 8);
    ext_param->prim_adv_intv_min[0] = (uint8_t)(ADV_INTERVAL_DFT >> 0);
    ext_param->prim_adv_ch_map      = ADV_ALL_CHNLS_EN;
    ext_param->own_addr_type        = ADDR_PUBLIC;
    ext_param->peer_addr_type       = ADDR_PUBLIC;
    memset(&ext_param->peer_addr.addr[0], 0x00, BD_ADDR_LEN);
    ext_param->adv_filt_policy      = ADV_ALLOW_SCAN_ANY_CON_ANY;
    ext_param->adv_tx_pwr           = ADV_TX_PWR_NO_PREF;
    ext_param->prim_adv_phy         = PHY_1MBPS_VALUE;
    ext_param->sec_adv_max_skip     = 0;
    ext_param->sec_adv_phy          = PHY_1MBPS_VALUE;
    ext_param->adv_sid              = 0;
    ext_param->scan_req_notif_en    = 0;

    // Get local public address
    llm_env.act_info[act_id].info.adv.bd_addr = llm_env.local_pub_addr;

    // Set advertising state for advertising activity identifier
    llm_env.act_info[act_id].state = LLM_ADV_RSVD;
}
#endif //(BLE_ADV_LEGACY_ITF)

uint8_t llm_adv_hdl_to_id(uint16_t adv_hdl, struct hci_le_set_ext_adv_param_cmd **adv_param)
{
    uint8_t act_id;
    struct hci_le_set_ext_adv_param_cmd *param;

    // Look for the advertising activity with specified handle
    for (act_id = 0; act_id < BLE_ACTIVITY_MAX_CFG; act_id++)
    {
        // Check if activity is allocated to advertising
        if ((llm_env.act_info[act_id].state == LLM_ADV_RSVD) || (llm_env.act_info[act_id].state == LLM_ADV_EN) || (llm_env.act_info[act_id].state == LLM_ADV_STOPPING))
        {
            // Point Host parameters
            param = (struct hci_le_set_ext_adv_param_cmd *) llm_env.act_info[act_id].host_params;

            ASSERT_INFO(param != NULL, act_id, llm_env.act_info[act_id].state);

            // Check advertising handle
            if (param->adv_hdl == adv_hdl)
                break;
        }
    }

    // If advertising set exists, return parameters
    if (act_id < BLE_ACTIVITY_MAX_CFG)
    {
        // provide parameter if requested
        if (adv_param != NULL)
        {
            *adv_param = param;
        }
    }
    // else mark activity invalid
    else
    {
        act_id = LLM_ACT_IDX_INVL;
    }

    return act_id;
}

/// Start periodic advertising
__STATIC uint8_t llm_adv_per_adv_start(uint8_t ext_act_id, uint8_t per_act_id)
{
    uint8_t status = CO_ERROR_CONN_REJ_LIMITED_RESOURCES;
    struct lld_per_adv_params per_adv_par;
    struct hci_le_set_per_adv_param_cmd *per_param = (struct hci_le_set_per_adv_param_cmd *) llm_env.act_info[per_act_id].host_params;
    struct hci_le_set_ext_adv_param_cmd *ext_param = (struct hci_le_set_ext_adv_param_cmd *) llm_env.act_info[ext_act_id].host_params;

    struct sch_plan_req_param req_param;

    uint16_t data_length = llm_env.act_info[per_act_id].info.per_adv.adv_data.curr_len;

    // Check if BIG is present
    if (llm_env.act_info[per_act_id].info.per_adv.ass_act_id != LLM_ACT_IDX_INVL)
    {
        data_length += BLE_EXT_ACAD_BIG_INFO_ENC_LEN;
    }

    // Input parameters
    req_param.interval_min    = per_param->adv_intv_min << 2;
    req_param.interval_max    = per_param->adv_intv_max << 2;
    req_param.duration_min    = llm_per_adv_chain_dur(data_length, ext_param->sec_adv_phy);
    req_param.duration_max    = req_param.duration_min;
    req_param.offset_min      = 0;
    req_param.offset_max      = req_param.interval_max - 1;
    req_param.pref_period     = 0;
    req_param.conhdl          = per_act_id;
    req_param.conhdl_ref      = req_param.conhdl;
    req_param.margin          = 1;

    if (sch_plan_req(&req_param) == SCH_PLAN_ERROR_OK)
    {
        // Fill advertising parameters
        per_adv_par.per_adv_properties = per_param->adv_prop;
        per_adv_par.per_adv_intv       = req_param.interval >> 2;
        per_adv_par.per_adv_offset     = req_param.offset_min;
        per_adv_par.init_adv_data_buf  = llm_env.act_info[per_act_id].info.per_adv.adv_data.curr_buf;
        per_adv_par.init_adv_data_len  = llm_env.act_info[per_act_id].info.per_adv.adv_data.curr_len;

        per_adv_par.own_addr_type      = ext_param->own_addr_type;
        per_adv_par.adv_tx_pwr         = ext_param->adv_tx_pwr;
        per_adv_par.sec_adv_phy        = ext_param->sec_adv_phy;

#if (BLE_BIS)
        per_adv_par.big_act_id         = llm_env.act_info[per_act_id].info.per_adv.ass_act_id;
#endif // (BLE_BIS)

#if BLE_CONLESS_CTE_TX
        // If CTE has already been enabled, set the CTE parameters before starting the periodic advertising driver
        // Else, set the CTE length to NO_CTE
        if (llm_env.act_info[ext_act_id].info.adv.cte_en)
        {
            struct hci_le_set_conless_cte_tx_param_cmd const *cte_param = llm_env.act_info[ext_act_id].info.adv.cte_tx_params;

            per_adv_par.cte_params.cte_len = cte_param->cte_len;
            per_adv_par.cte_params.cte_type = cte_param->cte_type;
            per_adv_par.cte_params.cte_count = cte_param->cte_count;
            per_adv_par.cte_params.switching_pattern_len = cte_param->switching_pattern_len;
            memcpy(&per_adv_par.cte_params.antenna_id[0], &cte_param->antenna_id[0], cte_param->switching_pattern_len);
        }
        else
        {
            per_adv_par.cte_params.cte_len = NO_CTE;
        }
#endif // BLE_CONLESS_CTE_TX

#if (BT_53)
        // Indicate whether the ADI field is included or not
        per_adv_par.include_adi = llm_env.act_info[per_act_id].info.per_adv.include_adi;

        if (per_adv_par.include_adi)
        {
            // Provide the advertising SID to the driver
            per_adv_par.adv_sid = ext_param->adv_sid;
        }
#endif // (BT_53)

        // Start periodic advertising
        status = lld_per_adv_start(per_act_id, &per_adv_par);

        if (status == CO_ERROR_NO_ERROR)
        {
            // Register activity into planner
            struct sch_plan_elt_tag *new_plan_elt = &llm_env.act_info[per_act_id].plan_elt;

            llm_env.act_info[per_act_id].state = LLM_PER_ADV_EN;

            lld_adv_sync_info_update(ext_act_id, per_act_id, per_adv_par.per_adv_intv);

            new_plan_elt->offset = req_param.offset_min;
            new_plan_elt->interval = req_param.interval;
            new_plan_elt->duration_min = req_param.duration_min;
            new_plan_elt->duration_max = req_param.duration_max;
            new_plan_elt->conhdl = per_act_id;
            new_plan_elt->conhdl_ref = per_act_id;
            new_plan_elt->margin = 1;
            new_plan_elt->cb_move = NULL;
            new_plan_elt->mobility = SCH_PLAN_MB_LVL_0;
            sch_plan_set(new_plan_elt);
        }
    }

    return status;
}

/// Approximates the duration of an extended advertising event
__STATIC uint32_t llm_adv_dur(struct hci_le_set_ext_adv_param_cmd const *ext_param, uint16_t adv_data_len, uint16_t scan_rsp_data_len)
{
    uint8_t prim_adv_rate = ext_param->prim_adv_phy - 1;
    uint8_t sec_adv_rate  = ext_param->sec_adv_phy - 1;
    uint8_t prim_adv_ch_nb = NB_ONE_BITS(ext_param->prim_adv_ch_map);
    uint8_t eff_data_size = 240;
    uint8_t overhead_size = 15;

    // Total duration of ADV_EXT_IND packets
    uint32_t dur_us = prim_adv_ch_nb * (ble_util_pkt_dur_in_us(overhead_size, prim_adv_rate) + BLE_IFS_DUR);

    ASSERT_ERR(adv_data_len <= BLE_CFG_MAX_ADV_DATA_LEN);
    ASSERT_ERR(scan_rsp_data_len <= BLE_CFG_MAX_ADV_DATA_LEN);
    ASSERT_ERR(ext_param != NULL);
    ASSERT_ERR(!(ext_param->adv_evt_properties & ADV_LEGACY));

    if (!(ext_param->adv_evt_properties & (ADV_CON | ADV_SCAN)))
    {
        uint8_t pkt_nb = adv_data_len / eff_data_size + (CO_MOD(adv_data_len, eff_data_size) != 0);
        uint8_t frag_size = (pkt_nb > 1) ? BLE_ADV_FRAG_SIZE_TX + 1 : co_min(adv_data_len + overhead_size, BLE_ADV_FRAG_SIZE_TX + 1);

        // Total duration of AUX_ADV_IND and AUX_CHAIN_IND packets
        if (pkt_nb == 1)
        {
            dur_us += ble_util_pkt_dur_in_us(frag_size, sec_adv_rate) + BLE_AFS_DUR;
        }
        else // (pkt_nb > 1)
        {
            uint8_t last_frag_size = co_min(adv_data_len - (pkt_nb - 1) * eff_data_size + overhead_size, BLE_ADV_FRAG_SIZE_TX + 1);

            dur_us += (pkt_nb - 1) * (ble_util_pkt_dur_in_us(frag_size, sec_adv_rate) + BLE_AFS_DUR);
            dur_us += ble_util_pkt_dur_in_us(last_frag_size, sec_adv_rate) + BLE_AFS_DUR;
        }
    }
    else if (ext_param->adv_evt_properties & ADV_CON)
    {
        uint8_t len = co_min(adv_data_len + overhead_size, BLE_ADV_FRAG_SIZE_TX + 1);

        // Duration of AUX_ADV_IND
        dur_us += ble_util_pkt_dur_in_us(len, sec_adv_rate) + BLE_AFS_DUR;
        // Duration of AUX_CONNECT_REQ
        dur_us += BLE_IFS_DUR + ble_util_pkt_dur_in_us(PDU_CON_REQ_LEN, sec_adv_rate);
        // Duration of AUX_CONNECT_RSP
        dur_us += BLE_IFS_DUR + ble_util_pkt_dur_in_us(overhead_size, sec_adv_rate);
    }
    else if (ext_param->adv_evt_properties & ADV_SCAN)
    {
        uint8_t len = co_min(adv_data_len + overhead_size, BLE_ADV_FRAG_SIZE_TX + 1);
        uint8_t pkt_nb = scan_rsp_data_len / eff_data_size + (CO_MOD(scan_rsp_data_len, eff_data_size) != 0);
        uint8_t frag_size = (pkt_nb > 1) ? BLE_ADV_FRAG_SIZE_TX + 1 : co_min(scan_rsp_data_len + overhead_size, BLE_ADV_FRAG_SIZE_TX + 1);

        // // Duration of AUX_ADV_IND
        dur_us += ble_util_pkt_dur_in_us(len, sec_adv_rate) + BLE_AFS_DUR;
        // Duration of AUX_SCAN_REQ
        dur_us += BLE_IFS_DUR + ble_util_pkt_dur_in_us(PDU_SCAN_REQ_LEN, sec_adv_rate);
        // Total duration of AUX_SCAN_RSP and AUX_CHAIN_IND packets
        if (pkt_nb == 1)
        {
            dur_us += BLE_IFS_DUR + ble_util_pkt_dur_in_us(frag_size, sec_adv_rate) + BLE_AFS_DUR;
        }
        else // (pkt_nb > 1)
        {
            uint8_t last_frag_size = co_min(scan_rsp_data_len - (pkt_nb - 1) * eff_data_size + overhead_size, BLE_ADV_FRAG_SIZE_TX + 1);

            dur_us += BLE_IFS_DUR + (pkt_nb - 1) * (ble_util_pkt_dur_in_us(frag_size, sec_adv_rate) + BLE_AFS_DUR);
            dur_us += ble_util_pkt_dur_in_us(last_frag_size, sec_adv_rate) + BLE_AFS_DUR;
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    return (dur_us);
}


/*
 * HCI COMMAND HANDLERS
 ****************************************************************************************
 */

int hci_le_rd_adv_ch_tx_pw_cmd_handler(void const *param, uint16_t opcode)
{
    struct hci_rd_adv_chnl_tx_pw_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_rd_adv_chnl_tx_pw_cmd_cmp_evt);

    // Read TX power
    event->adv_tx_pw_lvl = lld_tx_power_dbm_get(rwip_rf.txpwr_max, rwip_rf.txpwr_max_mod);

#if (BLE_ADV_LEGACY_ITF)
    // Check current interface version
    if (llm_env.adv_itf_version == LLM_ADV_ITF_EXTENDED)
    {
        event->status = CO_ERROR_COMMAND_DISALLOWED;
    }
    else
#endif //(BLE_ADV_LEGACY_ITF)
    {
#if (BLE_ADV_LEGACY_ITF)
        // Use legacy interface
        llm_env.adv_itf_version = LLM_ADV_ITF_LEGACY;
#endif //(BLE_ADV_LEGACY_ITF)

        event->status = CO_ERROR_NO_ERROR;
    }

    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}

#if (BLE_ADV_LEGACY_ITF)
int hci_le_set_adv_param_cmd_handler(struct hci_le_set_adv_param_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    do
    {
        uint8_t act_id;
        bool directed_adv = false;

        // Check current interface version
        if (llm_env.adv_itf_version == LLM_ADV_ITF_EXTENDED)
            break;

        // Use legacy interface
        llm_env.adv_itf_version = LLM_ADV_ITF_LEGACY;

        // Look for the legacy advertising activity
        act_id = llm_adv_hdl_to_id(LLM_ADV_HDL_INVL, NULL);

        // Check if legacy advertising has been found
        if (act_id != LLM_ACT_IDX_INVL)
        {
            // If advertising is already enabled, reject the command
            if (llm_env.act_info[act_id].state != LLM_ADV_RSVD)
                break;
        }
        else
        {
            // Allocate a new activity
            status = llm_activity_free_get(&act_id);

            if (status == CO_ERROR_NO_ERROR)
            {
                // Create and initialize advertising parameters
                llm_adv_set_dft_params(act_id);
            }
            else
            {
                // If not possible to start a new advertising activity, reject the command
                break;
            }
        }

        status = CO_ERROR_INVALID_HCI_PARAM;

        // Check parameters
        if (param->adv_type > ADV_CONN_DIR_LDC)
            break;

        // For high duty cycle directed advertising, i.e. when Advertising_Type is 0x01
        // (ADV_DIRECT_IND, high duty cycle), the Advertising_Interval_Min and
        // Advertising_Interval_Max parameters are not used and shall be ignored.
        if ((param->adv_type != ADV_CONN_DIR)
                && ((param->adv_intv_min > param->adv_intv_max) || (param->adv_intv_min < ADV_INTERVAL_MIN) || (param->adv_intv_max > ADV_INTERVAL_MAX)))
            break;

        if ((param->adv_ch_map > ADV_ALL_CHNLS_EN) || (param->adv_ch_map == 0) || (param->adv_filt_policy > ADV_ALLOW_SCAN_WLST_CON_WLST))
            break;

        if ((param->own_addr_type > ADDR_RPA_OR_RAND) || (param->peer_addr_type > ADDR_RAND))
            break;

        // Check peer address is valid (directed ADV only)
        if ((param->adv_type == ADV_CONN_DIR) || (param->adv_type == ADV_CONN_DIR_LDC))
        {
            if (co_bdaddr_compare(&param->peer_addr, &co_null_bdaddr))
            {
                break;
            }

            directed_adv = true;
        }

        // Convert legacy HCI command to extended format
        {
            struct hci_le_set_ext_adv_param_cmd *ext_param = (struct hci_le_set_ext_adv_param_cmd *) llm_env.act_info[act_id].host_params;
            ext_param->adv_evt_properties   = adv_evt_type2prop[param->adv_type];
            ext_param->prim_adv_intv_max[2] = (param->adv_intv_max >> 16);
            ext_param->prim_adv_intv_max[1] = (param->adv_intv_max >> 8);
            ext_param->prim_adv_intv_max[0] = (param->adv_intv_max >> 0);
            ext_param->prim_adv_intv_min[2] = (param->adv_intv_min >> 16);
            ext_param->prim_adv_intv_min[1] = (param->adv_intv_min >> 8);
            ext_param->prim_adv_intv_min[0] = (param->adv_intv_min >> 0);
            ext_param->prim_adv_ch_map      = param->adv_ch_map;
            ext_param->own_addr_type        = param->own_addr_type;
            ext_param->peer_addr_type       = param->peer_addr_type;
            ext_param->peer_addr            = param->peer_addr;
            ext_param->adv_filt_policy      = directed_adv ? ADV_ALLOW_SCAN_ANY_CON_ANY : param->adv_filt_policy;

            // Assign the local BD address, depending on address type
            switch (param->own_addr_type)
            {
            case  ADDR_PUBLIC:
            case  ADDR_RPA_OR_PUBLIC:
            {
                // Get local public address
                llm_env.act_info[act_id].info.adv.bd_addr = llm_env.local_pub_addr;
            }
            break;
            case  ADDR_RAND:
            case  ADDR_RPA_OR_RAND:
            {
                // Get local random address
                llm_env.act_info[act_id].info.adv.bd_addr = llm_env.local_rand_addr;
            }
            break;
            default:
            {
                ASSERT_INFO_FORCE(0, param->own_addr_type, 0);
            }
            break;
            }
        }

        // Assign the allocated activity identifier to legacy advertising
        llm_env.act_info[act_id].state = LLM_ADV_RSVD;

        status = CO_ERROR_NO_ERROR;

    }
    while (0);

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

int hci_le_set_adv_data_cmd_handler(struct hci_le_set_adv_data_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;

    do
    {
        uint8_t act_id;

        // Check current interface version
        if (llm_env.adv_itf_version == LLM_ADV_ITF_EXTENDED)
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        // Use legacy interface
        llm_env.adv_itf_version = LLM_ADV_ITF_LEGACY;

        // Check if ADV data length is valid
        if (param->adv_data_len > ADV_DATA_LEN)
            break;

        // Look for the legacy advertising activity
        act_id = llm_adv_hdl_to_id(LLM_ADV_HDL_INVL, NULL);

        // Check if legacy advertising has been found
        if (act_id == LLM_ACT_IDX_INVL)
        {
            // Allocate a new activity
            status = llm_activity_free_get(&act_id);

            if (status == CO_ERROR_NO_ERROR)
            {
                // Create and initialize advertising parameters
                llm_adv_set_dft_params(act_id);
            }
            else
            {
                // If not possible to start a new advertising activity, reject the command
                break;
            }
        }

        // If data changed
        if (llm_adv_is_data_changed(param->adv_data_len, llm_env.act_info[act_id].info.adv.adv_data.curr_len))
        {
            uint16_t old_buf, new_buf;

            // Retrieve existing advertising data buffer if present
            old_buf = llm_env.act_info[act_id].info.adv.adv_data.curr_buf;

            // Try to allocate a new buffer for the new advertising data
            new_buf = ble_util_buf_adv_tx_alloc();

            // Check if a new buffer is available
            if (new_buf == 0)
            {
                status = CO_ERROR_MEMORY_CAPA_EXCEED;
                break;
            }

            // Copy data from EM
            em_wr(&param->data, new_buf, param->adv_data_len);

            // Store information
            llm_env.act_info[act_id].info.adv.adv_data.curr_buf = new_buf;
            llm_env.act_info[act_id].info.adv.adv_data.curr_len = param->adv_data_len;

            // Check if advertising is enabled
            if (llm_env.act_info[act_id].state == LLM_ADV_EN)
            {
                // Indicate new data to driver (the driver is responsible to free the old data buffer when no more used)
                lld_adv_adv_data_update(act_id, param->adv_data_len, new_buf);
            }
            else if (old_buf)
            {
                // Release the buffer
                ble_util_buf_adv_tx_free(old_buf);

#if (BT_53)
                // If there is a reason to update the RPAs
                if (llm_env.act_info[act_id].info.adv.chg_reasons & RPA_ADV_DATA_CHG)
                {
                    // Set RPA flag
                    llm_env.act_info[act_id].info.adv.adv_rpa_upd = true;
                }
#endif // (BT_53)
            }
        }

        status = CO_ERROR_NO_ERROR;

    }
    while (0);

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

int hci_le_set_scan_rsp_data_cmd_handler(struct hci_le_set_scan_rsp_data_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;

    do
    {
        uint8_t act_id;

        // Check current interface version
        if (llm_env.adv_itf_version == LLM_ADV_ITF_EXTENDED)
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        // Use legacy interface
        llm_env.adv_itf_version = LLM_ADV_ITF_LEGACY;

        // Check if scan response data length is valid
        if (param->scan_rsp_data_len > SCAN_RSP_DATA_LEN)
            break;

        // Look for the legacy advertising activity
        act_id = llm_adv_hdl_to_id(LLM_ADV_HDL_INVL, NULL);

        // Check if legacy advertising has been found
        if (act_id == LLM_ACT_IDX_INVL)
        {
            // Allocate a new activity
            status = llm_activity_free_get(&act_id);

            if (status == CO_ERROR_NO_ERROR)
            {
                // Create and initialize advertising parameters
                llm_adv_set_dft_params(act_id);
            }
            else
            {
                // If not possible to start a new advertising activity, reject the command
                break;
            }
        }

        // If data changed
        if (llm_adv_is_data_changed(param->scan_rsp_data_len, llm_env.act_info[act_id].info.adv.scan_rsp_data.curr_len))
        {
            uint16_t old_buf, new_buf;

            // Retrieve existing advertising data buffer if present
            old_buf = llm_env.act_info[act_id].info.adv.scan_rsp_data.curr_buf;

            // Try to allocate a new buffer for the new advertising data
            new_buf = ble_util_buf_adv_tx_alloc();

            // Check if a new buffer is available
            if (new_buf == 0)
            {
                status = CO_ERROR_MEMORY_CAPA_EXCEED;
                break;
            }

            // Copy data from EM
            em_wr(&param->data, new_buf, param->scan_rsp_data_len);

            // Store information
            llm_env.act_info[act_id].info.adv.scan_rsp_data.curr_buf = new_buf;
            llm_env.act_info[act_id].info.adv.scan_rsp_data.curr_len = param->scan_rsp_data_len;

            // Check if advertising is enabled
            if (llm_env.act_info[act_id].state == LLM_ADV_EN)
            {
                // Indicate new data to driver (the driver is responsible to free the old data buffer when no more used)
                lld_adv_scan_rsp_data_update(act_id, param->scan_rsp_data_len, new_buf);
            }
            else if (old_buf)
            {
                // Release the buffer
                ble_util_buf_adv_tx_free(old_buf);

#if (BT_53)
                // If there is a reason to update the RPAs
                if (llm_env.act_info[act_id].info.adv.chg_reasons & RPA_SCAN_RSP_DATA_CHG)
                {
                    // Set RPA flag
                    llm_env.act_info[act_id].info.adv.adv_rpa_upd = true;
                }
#endif // (BT_53)
            }
        }

        status = CO_ERROR_NO_ERROR;

    }
    while (0);

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

int hci_le_set_adv_en_cmd_handler(struct hci_le_set_adv_en_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;
    bool cmd_cmp   = false;

    // Check current interface version
    if (llm_env.adv_itf_version != LLM_ADV_ITF_EXTENDED)
    {
        uint8_t act_id;

        // Use legacy interface
        llm_env.adv_itf_version = LLM_ADV_ITF_LEGACY;

        // Look for the legacy advertising activity
        act_id = llm_adv_hdl_to_id(LLM_ADV_HDL_INVL, NULL);

        // Check parameter
        switch (param->adv_en)
        {
        case ADV_DIS:
        {
            // Verify that advertising is present and enabled
            if ((act_id >= BLE_ACTIVITY_MAX_CFG) || (llm_env.act_info[act_id].state != LLM_ADV_EN))
            {
                status = CO_ERROR_NO_ERROR;
                cmd_cmp = true;
                break;
            }

            // Stop advertising
            status = lld_adv_stop(act_id);

            if (status == CO_ERROR_NO_ERROR)
            {
                llm_env.act_info[act_id].state = LLM_ADV_STOPPING;
            }
        }
        break;

        case ADV_EN:
        {
            // Check if advertising is present
            if (act_id < BLE_ACTIVITY_MAX_CFG)
            {
                // Check if already enabled
                if (llm_env.act_info[act_id].state != LLM_ADV_RSVD)
                    break;
            }
            else
            {
                // Allocate a new activity
                status = llm_activity_free_get(&act_id);

                if (status == CO_ERROR_NO_ERROR)
                {
                    // Create and initialize advertising parameters
                    llm_adv_set_dft_params(act_id);
                }
                else
                {
                    // If not possible to start a new advertising activity, reject the command
                    break;
                }
            }

            {
                uint8_t con_act_id = LLM_ACT_IDX_INVL;
                struct lld_adv_params adv_par;
                struct hci_le_set_ext_adv_param_cmd *ext_param = (struct hci_le_set_ext_adv_param_cmd *) llm_env.act_info[act_id].host_params;

                /*
                 * If the advertising parameters' Own_Address_Type parameter is set to 0x01
                 * and the random address for the device has not been initialized, the Controller
                 * shall return the error code Invalid HCI Command Parameters (0x12).
                 */
                if (((ext_param->own_addr_type & ADDR_MASK) == ADDR_RAND)
                        && (co_bdaddr_compare(&llm_env.local_rand_addr, &co_null_bdaddr) == true))
                {
                    status = CO_ERROR_INVALID_HCI_PARAM;
                    break;
                }

                // If connectable advertising
                if (ext_param->adv_evt_properties & ADV_CON)
                {
#if (BLE_PERIPHERAL)
                    // Try to find a free activity identifier for the potential connection
                    status = llm_activity_free_get(&con_act_id);

                    // Reject if cannot allocate one new activity identifier
                    if (status != CO_ERROR_NO_ERROR)
                    {
                        break;
                    }

                    // If directed advertising
                    if ((ext_param->adv_evt_properties & (ADV_DIRECT | ADV_DIRECT_HI)) != 0)
                    {
                        // Check if device is already connected
                        if (llm_is_dev_connected(&ext_param->peer_addr, ext_param->peer_addr_type))
                        {
                            status = CO_ERROR_CON_ALREADY_EXISTS;
                            break;
                        }
                    }
#else // (BLE_PERIPHERAL)
                    status = CO_ERROR_INVALID_HCI_PARAM;
                    break;
#endif // (BLE_PERIPHERAL)
                }

                // Fill advertising parameters
                adv_par.adv_evt_properties = ext_param->adv_evt_properties;
                adv_par.prim_adv_intv      = (ext_param->prim_adv_intv_max[2] << 16) | (ext_param->prim_adv_intv_max[1] << 8) | ext_param->prim_adv_intv_max[0];
                adv_par.prim_adv_ch_map    = ext_param->prim_adv_ch_map;
                adv_par.own_addr_type      = ext_param->own_addr_type;
                adv_par.peer_addr_type     = ext_param->peer_addr_type;
                adv_par.peer_addr          = ext_param->peer_addr;
                adv_par.adv_filter_policy  = ext_param->adv_filt_policy;
                adv_par.adv_tx_pwr         = ext_param->adv_tx_pwr;
                adv_par.prim_adv_phy       = ext_param->prim_adv_phy;
                adv_par.sec_adv_max_skip   = ext_param->sec_adv_max_skip;
                adv_par.sec_adv_phy        = ext_param->prim_adv_phy;
                adv_par.adv_sid            = ext_param->adv_sid;
                adv_par.scan_req_notif_en  = ext_param->scan_req_notif_en;
                adv_par.own_addr           = (ext_param->own_addr_type & ADDR_RAND) ? llm_env.local_rand_addr : llm_env.local_pub_addr;
                adv_par.init_adv_data_buf  = llm_env.act_info[act_id].info.adv.adv_data.curr_buf;
                adv_par.init_adv_data_len  = llm_env.act_info[act_id].info.adv.adv_data.curr_len;
                adv_par.init_scan_rsp_data_buf = llm_env.act_info[act_id].info.adv.scan_rsp_data.curr_buf;
                adv_par.init_scan_rsp_data_len = llm_env.act_info[act_id].info.adv.scan_rsp_data.curr_len;
                adv_par.duration           = 0;
                adv_par.max_ext_adv_evt    = 0;
                adv_par.addr_resolution_en = llm_env.addr_resolution_en;
                adv_par.ral_idx            = BLE_RAL_MAX;
                adv_par.con_act_id         = con_act_id;
#if (BT_53)
                adv_par.chg_reasons = llm_env.act_info[act_id].info.adv.chg_reasons;
                adv_par.adv_rpa_upd = llm_env.act_info[act_id].info.adv.adv_rpa_upd;
#endif // (BT_53)

#if (EAVESDROPPING_SUPPORT)
                if (llm_env.act_info[act_id].info.adv.manual_adv_params != NULL)
                {
                    // Copy the manual adv parameters
                    adv_par.manual_adv_params.en = llm_env.act_info[act_id].info.adv.manual_adv_params->enable;
                    adv_par.manual_adv_params.time_offset = llm_env.act_info[act_id].info.adv.manual_adv_params->time_offset;
                    adv_par.manual_adv_params.max_rand_offset = llm_env.act_info[act_id].info.adv.manual_adv_params->max_rand_offset;
                }
                else
                {
                    adv_par.manual_adv_params.en = false;
                }

                if (llm_env.act_info[act_id].info.adv.cust_adv_params != NULL)
                {
                    // Copy the custom adv parameters
                    adv_par.cust_adv_params.periodicity = llm_env.act_info[act_id].info.adv.cust_adv_params->periodicity;
                    adv_par.cust_adv_params.en = llm_env.act_info[act_id].info.adv.cust_adv_params->enable;
                    adv_par.cust_adv_params.rand_en = llm_env.act_info[act_id].info.adv.cust_adv_params->rand_en;
                    adv_par.cust_adv_params.time_offset = llm_env.act_info[act_id].info.adv.cust_adv_params->time_offset;
                    adv_par.cust_adv_params.us_offset = llm_env.act_info[act_id].info.adv.cust_adv_params->us_offset;
                    memcpy(&adv_par.cust_adv_params.bd_addr.addr[0], &llm_env.act_info[act_id].info.adv.cust_adv_params->bd_addr.addr[0], BD_ADDR_LEN);
                    adv_par.cust_adv_params.group_ext_adv_ind = false;
                }
                else
                {
                    adv_par.cust_adv_params.en = false;
                }
#endif // EAVESDROPPING_SUPPORT

                // Check if peer device has a corresponding resolving list entry
                if ((adv_par.addr_resolution_en && (ext_param->adv_evt_properties & ADV_DIRECT))
                        || (ext_param->own_addr_type & ADDR_RPA_MASK))
                {
                    // Find peer device in RAL
                    adv_par.ral_idx = llm_ral_search(&ext_param->peer_addr, ext_param->peer_addr_type);

                    // If peer has been found, check if local RPA needs/can to be generated
                    if ((adv_par.ral_idx < BLE_RAL_MAX)
                            && ((ext_param->own_addr_type & ADDR_RPA_MASK) != 0)
                            && (memcmp(&llm_env.ral[adv_par.ral_idx].local_irk.key[0], co_null_key, KEY_LEN)))
                    {
                        // Generate an RPA
                        aes_rpa_gen((aes_key_t *)&llm_env.ral[adv_par.ral_idx].local_irk, llm_adv_rpa_gen_cb, act_id);
                    }
                }

                // Start advertising
                status = lld_adv_start(act_id, &adv_par);

                if (status == CO_ERROR_NO_ERROR)
                {
                    // Set advertising state for advertising activity identifier
                    llm_env.act_info[act_id].state = LLM_ADV_EN;

#if (BT_53)
                    // Clear RPA flag
                    llm_env.act_info[act_id].info.adv.adv_rpa_upd = false;
#endif // (BT_53)

                    // If connectable advertising
                    if ((ext_param->adv_evt_properties & ADV_CON))
                    {
                        // Save connection activity identifier to the advertising activity parameters
                        llm_env.act_info[act_id].info.adv.con_act_id = con_act_id;

                        // Reserve the connection activity identifier
                        llm_env.act_info[con_act_id].state = LLM_CON_RSVD;
                    }
                }
            }
        }
        break;

        default:
        {
            status = CO_ERROR_INVALID_HCI_PARAM;
        }
        break;
        }
    }

    // If advertising is stopping, wait the indication from the driver before reporting the command complete event
    if ((status != CO_ERROR_NO_ERROR) || (param->adv_en != ADV_DIS) || cmd_cmp)
    {
        // Send the command complete event
        llm_cmd_cmp_send(opcode, status);
    }

    return (KE_MSG_CONSUMED);
}
#endif //(BLE_ADV_LEGACY_ITF)

int hci_le_set_ext_adv_param_cmd_handler(struct hci_le_set_ext_adv_param_cmd const *param, uint16_t opcode)
{
    struct hci_le_set_ext_adv_param_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_le_set_ext_adv_param_cmd_cmp_evt);
    uint8_t ret_status = KE_MSG_CONSUMED;
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    // Initialize the selected Tx power in case the command is rejected
    event->sel_tx_pwr = 0;

    do
    {
        uint32_t prim_adv_intv_min, prim_adv_intv_max;
        uint8_t act_id;

#if (BLE_ADV_LEGACY_ITF)
        // Check current interface version
        if (llm_env.adv_itf_version == LLM_ADV_ITF_LEGACY)
            break;

        // Use extended interface
        llm_env.adv_itf_version = LLM_ADV_ITF_EXTENDED;
#endif //(BLE_ADV_LEGACY_ITF)

        // Look for the advertising activity with specified handle
        act_id = llm_adv_hdl_to_id(param->adv_hdl, NULL);

        // Check if advertising handle has been found
        if (act_id != LLM_ACT_IDX_INVL)
        {
            // If advertising is already enabled, reject the command
            if (llm_env.act_info[act_id].state != LLM_ADV_RSVD)
                break;
        }
        else
        {
            // If not present, try to allocate an activity identifier
            status = llm_activity_free_get(&act_id);

            // If not possible to start a new advertising activity, reject the command
            if (status != CO_ERROR_NO_ERROR)
            {
                status = CO_ERROR_MEMORY_CAPA_EXCEED;
                break;
            }

            // Initialize the periodic advertising ID to invalid
            llm_env.act_info[act_id].info.adv.per_act_id = LLM_ACT_IDX_INVL;
        }

#if !(BLE_PHY_CODED_SUPPORT)
        if ((PHY_CODED_VALUE == param->prim_adv_phy) || (PHY_CODED_VALUE == param->sec_adv_phy))
        {
            status = CO_ERROR_UNSUPPORTED;
            break;
        }
#endif

#if !(BLE_PHY_2MBPS_SUPPORT)
        if (PHY_2MBPS_VALUE == param->sec_adv_phy)
        {
            status = CO_ERROR_UNSUPPORTED;
            break;
        }
#endif

        status = CO_ERROR_INVALID_HCI_PARAM;

        // Check parameters
        if (param->adv_hdl > ADV_HDL_MAX)
            break;

        if (llm_env.act_info[act_id].info.adv.per_act_id != LLM_ACT_IDX_INVL
                && llm_env.act_info[llm_env.act_info[act_id].info.adv.per_act_id].state != LLM_PER_ADV_RSVD)
            break;


        // If legacy advertising PDU types are being used, then the advertising event properties value shall be one of those specified in Table 7.2
        if ((param->adv_evt_properties & ADV_LEGACY) && (param->adv_evt_properties != ADV_IND)
                && (param->adv_evt_properties != ADV_DIRECT_LO_IND) && (param->adv_evt_properties != ADV_DIRECT_HI_IND)
                && (param->adv_evt_properties != ADV_SCAN_IND) && (param->adv_evt_properties != ADV_NONCONN_IND))
            break;

        // If extended advertising PDU types are being used then:
        if (!(param->adv_evt_properties & ADV_LEGACY))
        {
            // The advertisement shall not be both connectable and scannable
            if ((param->adv_evt_properties & ADV_CON) && (param->adv_evt_properties & ADV_SCAN))
                break;

            // High duty cycle directed connectable advertising shall not be used
            if (param->adv_evt_properties & ADV_DIRECT_HI)
                break;

            // The "anonymous advertising" bit cannot be set for connectable or scannable advertising
            if ((param->adv_evt_properties & (ADV_CON | ADV_SCAN)) && (param->adv_evt_properties & ADV_ANONYMOUS))
                break;
        }

        // If legacy advertising PDUs are being used, the Primary_Advertising_PHY shall be set to 0x01 (LE 1M)
        if ((param->adv_evt_properties & ADV_LEGACY) && (param->prim_adv_phy != PHY_1MBPS_VALUE))
            break;

        // Extract primary interval fields
        if (!(param->adv_evt_properties & ADV_DIRECT_HI))
        {
            prim_adv_intv_min  = (param->prim_adv_intv_min[2] << 16) | (param->prim_adv_intv_min[1] << 8) | param->prim_adv_intv_min[0];
            prim_adv_intv_max  = (param->prim_adv_intv_max[2] << 16) | (param->prim_adv_intv_max[1] << 8) | param->prim_adv_intv_max[0];
        }
        else // The interval is ignored for high duty cycle connectable directed advertising
        {
            prim_adv_intv_min = ADV_INTERVAL_MIN;
            prim_adv_intv_max = ADV_INTERVAL_MIN;
        }

        if ((prim_adv_intv_min > prim_adv_intv_max) || (param->prim_adv_ch_map > ADV_ALL_CHNLS_EN)
                || (param->prim_adv_ch_map == 0) || (param->adv_filt_policy > ADV_ALLOW_SCAN_WLST_CON_WLST))
            break;

        if ((prim_adv_intv_min < ADV_INTERVAL_MIN) || (prim_adv_intv_max < ADV_INTERVAL_MIN))
            break;

        if ((param->prim_adv_phy != PHY_1MBPS_VALUE) && (param->prim_adv_phy != PHY_CODED_VALUE))
            break;

        if ((!(param->adv_evt_properties & ADV_LEGACY)) && ((param->sec_adv_phy < PHY_1MBPS_VALUE) || (param->sec_adv_phy > PHY_CODED_VALUE)))
            break;

        // Check peer address is valid (directed ADV only)
        if (param->adv_evt_properties & ADV_DIRECT)
        {
            if (co_bdaddr_compare(&param->peer_addr, &co_null_bdaddr))
                break;
        }

        if ((param->adv_sid > ADV_SID_MAX) || (param->scan_req_notif_en > 0x01))
            break;

        // Check if there is an associated periodic advertising handle
        if (llm_env.act_info[act_id].info.adv.per_act_id < BLE_ACTIVITY_MAX_CFG)
        {
            uint8_t per_act_id = llm_env.act_info[act_id].info.adv.per_act_id;

            // If periodic advertising is already enabled
            if (llm_env.act_info[per_act_id].state != LLM_PER_ADV_RSVD)
            {
                struct hci_le_set_ext_adv_param_cmd *ext_param = (struct hci_le_set_ext_adv_param_cmd *) llm_env.act_info[act_id].host_params;

                // The advertising set must have already been configured
                ASSERT_ERR(ext_param != NULL);

                // If connectable, scannable, legacy, or anonymous advertising is specified
                if (param->adv_evt_properties & (ADV_CON | ADV_SCAN | ADV_LEGACY | ADV_ANONYMOUS))
                    break;

                // If the Secondary_Advertising_PHY parameter does not specify the PHY currently being used for the periodic advertising
                if (param->sec_adv_phy != ext_param->sec_adv_phy)
                {
                    status = CO_ERROR_COMMAND_DISALLOWED;
                    break;
                }
            }
        }

        /*
         * If the advertising set already contains advertising data or scan response data,
         * extended advertising is being used, and the length of the data is greater than
         * the maximum that the Controller can transmit within the longest possible
         * auxiliary advertising segment consistent with the parameters, the Controller
         * shall return the error code Packet Too Long (0x45).
         */
        if (!(param->adv_evt_properties & ADV_LEGACY))
        {
            if (llm_env.act_info[act_id].info.adv.adv_data.curr_len || llm_env.act_info[act_id].info.adv.scan_rsp_data.curr_len)
            {
                uint16_t adv_data_len = llm_env.act_info[act_id].info.adv.adv_data.curr_len;
                uint16_t scan_rsp_data_len = llm_env.act_info[act_id].info.adv.scan_rsp_data.curr_len;
                uint32_t intv = prim_adv_intv_max * SLOT_SIZE;

                if (llm_adv_dur(param, adv_data_len, scan_rsp_data_len) >= intv)
                {
                    status = CO_ERROR_PKT_TOO_LONG;
                    break;
                }
            }
        }

        // Free potentially stored parameters
        if (llm_env.act_info[act_id].host_params != NULL)
        {
            ke_msg_free(ke_param2msg(llm_env.act_info[act_id].host_params));
            llm_env.act_info[act_id].host_params = NULL;
        }

        // Keep pointer to parameters
        llm_env.act_info[act_id].host_params = param;

        // Initialize the connection ID to invalid
        llm_env.act_info[act_id].info.adv.con_act_id = LLM_ACT_IDX_INVL;

        // Assign the local BD address, depending on address type
        switch (param->own_addr_type)
        {
        case  ADDR_PUBLIC:
        case  ADDR_RPA_OR_PUBLIC:
        {
            // Get local public address
            llm_env.act_info[act_id].info.adv.bd_addr = llm_env.local_pub_addr;
        }
        break;
        case  ADDR_RAND:
        case  ADDR_RPA_OR_RAND:
        {
            // The local random address should be assigned by HCI_LE_set_adv_set_rand_addr command
        }
        break;
        default:
        {
            ASSERT_INFO_FORCE(0, param->own_addr_type, 0);
        }
        break;
        }

        // Assign the allocated activity identifier to advertising
        llm_env.act_info[act_id].state = LLM_ADV_RSVD;

        // Choose an appropriate Tx power level
        {
            int16_t trans_tx_pwr_lvl_db;
            uint8_t rf_mod;

            if (param->adv_tx_pwr != ADV_TX_PWR_NO_PREF)
            {
                trans_tx_pwr_lvl_db = param->adv_tx_pwr - lld_tx_path_comp_get() / 10;
                rf_mod = rwip_rf.txpwr_mdlt_get(trans_tx_pwr_lvl_db, 0);
            }
            else
            {
                trans_tx_pwr_lvl_db = rwip_rf.txpwr_dbm_get(rwip_rf.txpwr_max, MOD_GFSK, rwip_rf.txpwr_max_mod);
                rf_mod = rwip_rf.txpwr_max_mod;
            }

            llm_env.act_info[act_id].info.adv.tx_pwr = lld_tx_power_dbm_get(rwip_rf.txpwr_cs_get(trans_tx_pwr_lvl_db, TXPWR_CS_LOWER, rf_mod), rf_mod);
        }
        // Return the selected Tx power
        event->sel_tx_pwr = llm_env.act_info[act_id].info.adv.tx_pwr;

        // Message needs to be maintained
        ret_status = KE_MSG_NO_FREE;

        status = CO_ERROR_NO_ERROR;

    }
    while (0);

    // Send the command complete event
    event->status   = status;
    hci_send_2_host(event);

    return (ret_status);
}

#if (EAVESDROPPING_SUPPORT)
int hci_vs_set_cust_adv_cmd_handler(struct hci_vs_set_cust_adv_cmd const *param, uint16_t opcode)
{
    uint8_t ret_status = KE_MSG_CONSUMED;
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

#if (BLE_ADV_LEGACY_ITF)
    // Check current interface version
    if (llm_env.adv_itf_version != LLM_ADV_ITF_LEGACY)
#endif //(BLE_ADV_LEGACY_ITF)
    {
        uint8_t act_id;

#if (BLE_ADV_LEGACY_ITF)
        // Use extended interface
        llm_env.adv_itf_version = LLM_ADV_ITF_EXTENDED;
#endif //(BLE_ADV_LEGACY_ITF)

        status = CO_ERROR_UNKNOWN_ADVERTISING_ID;

        // Look for the advertising activity with specified handle
        act_id = llm_adv_hdl_to_id(param->adv_hdl, NULL);

        // Check if advertising handle has been found
        if (act_id < BLE_ACTIVITY_MAX_CFG)
        {
            // Free potentially stored parameters
            if (llm_env.act_info[act_id].info.adv.cust_adv_params != NULL)
            {
                ke_msg_free(ke_param2msg(llm_env.act_info[act_id].info.adv.cust_adv_params));
                llm_env.act_info[act_id].info.adv.cust_adv_params = NULL;
            }

            // Keep pointer to parameters
            llm_env.act_info[act_id].info.adv.cust_adv_params = param;

            // Message needs to be maintained
            ret_status = KE_MSG_NO_FREE;

            status = CO_ERROR_NO_ERROR;
        }
    }

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (ret_status);
}

int hci_vs_le_set_manual_adv_cmd_handler(struct hci_vs_le_set_manual_adv_cmd const *param, uint16_t opcode)
{
    uint8_t ret_status = KE_MSG_CONSUMED;
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

#if (BLE_ADV_LEGACY_ITF)
    // Check current interface version
    if (llm_env.adv_itf_version != LLM_ADV_ITF_LEGACY)
#endif //(BLE_ADV_LEGACY_ITF)
    {
        uint8_t act_id;

#if (BLE_ADV_LEGACY_ITF)
        // Use extended interface
        llm_env.adv_itf_version = LLM_ADV_ITF_EXTENDED;
#endif //(BLE_ADV_LEGACY_ITF)

        status = CO_ERROR_UNKNOWN_ADVERTISING_ID;

        // Look for the advertising activity with specified handle
        act_id = llm_adv_hdl_to_id(param->adv_hdl, NULL);

        // Check if advertising handle has been found
        if (act_id < BLE_ACTIVITY_MAX_CFG)
        {
            // Free potentially stored parameters
            if (llm_env.act_info[act_id].info.adv.manual_adv_params != NULL)
            {
                ke_msg_free(ke_param2msg(llm_env.act_info[act_id].info.adv.manual_adv_params));
                llm_env.act_info[act_id].info.adv.manual_adv_params = NULL;
            }

            // Keep pointer to parameters
            llm_env.act_info[act_id].info.adv.manual_adv_params = param;

            // Message needs to be maintained
            ret_status = KE_MSG_NO_FREE;

            status = CO_ERROR_NO_ERROR;
        }
    }

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (ret_status);
}
#endif //(EAVESDROPPING_SUPPORT)

int hci_le_set_adv_set_rand_addr_cmd_handler(struct hci_le_set_adv_set_rand_addr_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

#if (BLE_ADV_LEGACY_ITF)
    // Check current interface version
    if (llm_env.adv_itf_version != LLM_ADV_ITF_LEGACY)
#endif //(BLE_ADV_LEGACY_ITF)
    {
        struct hci_le_set_ext_adv_param_cmd *ext_param;
        uint8_t act_id;

#if (BLE_ADV_LEGACY_ITF)
        // Use extended interface
        llm_env.adv_itf_version = LLM_ADV_ITF_EXTENDED;
#endif //(BLE_ADV_LEGACY_ITF)

        status = CO_ERROR_UNKNOWN_ADVERTISING_ID;

        // Look for the advertising activity with specified handle
        act_id = llm_adv_hdl_to_id(param->adv_hdl, &ext_param);

        // Check if advertising handle has been found
        if (act_id < BLE_ACTIVITY_MAX_CFG)
        {
#if (!BLE_HOST_PRESENT)
            // If connectable advertising is enabled
            if ((llm_env.act_info[act_id].state == LLM_ADV_EN) && (ext_param->adv_evt_properties & ADV_CON))
            {
                status = CO_ERROR_COMMAND_DISALLOWED;
            }
            else
#endif // (!BLE_HOST_PRESENT)
            {
                status = CO_ERROR_NO_ERROR;

                // Store random address
                llm_env.act_info[act_id].info.adv.bd_addr = param->rand_addr;

                // Check if advertising is enabled
                if (llm_env.act_info[act_id].state == LLM_ADV_EN)
                {
                    // Update driver
                    lld_adv_rand_addr_update(act_id, param->rand_addr);
                }
            }
        }
    }

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

int hci_le_set_ext_adv_data_cmd_handler(struct hci_le_set_ext_adv_data_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_UNKNOWN_ADVERTISING_ID;

    do
    {
        struct hci_le_set_ext_adv_param_cmd *ext_param;
        uint8_t act_id;

#if (BLE_ADV_LEGACY_ITF)
        // Check current interface version
        if (llm_env.adv_itf_version == LLM_ADV_ITF_LEGACY)
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        // Use extended interface
        llm_env.adv_itf_version = LLM_ADV_ITF_EXTENDED;
#endif //(BLE_ADV_LEGACY_ITF)

        // Look for the advertising activity with specified handle
        act_id = llm_adv_hdl_to_id(param->adv_hdl, &ext_param);

        // If advertising set does not exists, reject the command
        if (act_id == LLM_ACT_IDX_INVL)
            break;

        // Check parameters
        status = CO_ERROR_INVALID_HCI_PARAM;

        // Operation
        if (param->operation > ADV_DATA_OP_UNCHANGED_DATA)
            break;

        // Fragment preference
        if (param->frag_pref > ADV_DATA_SHOULD_NOT_FRAG)
            break;

        // Data length
        if (param->data_len > HCI_ADV_DATA_FRAG_MAX_LEN)
            break;

        /* If Operation is 0x04 and:
         *  - advertising is currently disabled for the advertising set;
         *  - the advertising set contains no data;
         *  - the advertising set uses legacy PDUs; or
         *  - Advertising_Data_Length is not zero; */
        if ((param->operation == ADV_DATA_OP_UNCHANGED_DATA)
                && ((llm_env.act_info[act_id].state == LLM_ADV_RSVD)
                    || (!llm_env.act_info[act_id].info.adv.adv_data.curr_buf)
                    || (param->data_len != 0)))
            break;

        // If Operation is not 0x03 or 0x04 and Advertising_Data_Length is zero
        if (((param->operation != ADV_DATA_OP_COMPLETE) && (param->operation != ADV_DATA_OP_UNCHANGED_DATA))
                && (param->data_len == 0))
            break;

        // Check that the message is valid as per current context
        status = CO_ERROR_COMMAND_DISALLOWED;

        // If advertising is enabled
        if (llm_env.act_info[act_id].state != LLM_ADV_RSVD)
        {
            // If Operation does not have the value 0x03 or 0x04
            if ((param->operation != ADV_DATA_OP_COMPLETE) && (param->operation != ADV_DATA_OP_UNCHANGED_DATA))
                break;

            // If advertising is extended connectable and the data does not fit in a single packet
            if (!(ext_param->adv_evt_properties & ADV_LEGACY) && (ext_param->adv_evt_properties & ADV_CON))
            {
                if (llm_adv_con_len_check(ext_param, param->data_len) != CO_ERROR_NO_ERROR)
                {
                    status = CO_ERROR_INVALID_HCI_PARAM;
                    break;
                }
            }

            // If the advertising set uses legacy advertising PDUs and the set supports advertising data and the data exceeds 31 octets
            if ((ext_param->adv_evt_properties & ADV_LEGACY) && !(ext_param->adv_evt_properties & (ADV_DIRECT | ADV_DIRECT_HI)) && (param->data_len > ADV_DATA_LEN))
            {
                status = CO_ERROR_INVALID_HCI_PARAM;
                break;
            }
        }

        status = CO_ERROR_MEMORY_CAPA_EXCEED;

        /*
         * If the combined length of the data exceeds the capacity of the advertising set identified by the
         * Advertising_Handle parameter (see Section 7.8.57 LE Read Maximum Advertising Data Length Command) or the
         * amount of memory currently available, all the data shall be discarded and the Controller shall return the
         * error code Memory Capacity Exceeded (0x07).
         */
        if ((llm_env.act_info[act_id].info.adv.adv_data.new_len + param->data_len) > EM_BLE_ADVDATATXBUF_SIZE)
        {
            // Free exchange memory buffer of the data in construction
            if (llm_env.act_info[act_id].info.adv.adv_data.new_buf)
            {
                ble_util_buf_adv_tx_free(llm_env.act_info[act_id].info.adv.adv_data.new_buf);
                llm_env.act_info[act_id].info.adv.adv_data.new_buf = 0;
                llm_env.act_info[act_id].info.adv.adv_data.new_len = 0;
            }

            break;
        }

        /*
         * If the advertising set uses extended advertising and the combined length of the
         * data is greater than the maximum that the Controller can transmit within the
         * longest possible auxiliary advertising segment consistent with the current
         * parameters of the advertising set (using the current advertising interval if
         * advertising is enabled), all the data shall be discarded and the Controller shall
         * return the error code Packet Too Long (0x45).
         */
        if (!(ext_param->adv_evt_properties & ADV_LEGACY))
        {
            uint16_t adv_data_len = llm_env.act_info[act_id].info.adv.adv_data.new_len + param->data_len;
            uint16_t scan_rsp_data_len = llm_env.act_info[act_id].info.adv.scan_rsp_data.curr_len;
            uint32_t prim_adv_intv_max  = (ext_param->prim_adv_intv_max[2] << 16) | (ext_param->prim_adv_intv_max[1] << 8) | ext_param->prim_adv_intv_max[0];
            uint32_t intv = prim_adv_intv_max * SLOT_SIZE;

            if (llm_adv_dur(ext_param, adv_data_len, scan_rsp_data_len) >= intv)
            {
                // Free exchange memory buffer of the data in construction
                if (llm_env.act_info[act_id].info.adv.adv_data.new_buf)
                {
                    ble_util_buf_adv_tx_free(llm_env.act_info[act_id].info.adv.adv_data.new_buf);
                    llm_env.act_info[act_id].info.adv.adv_data.new_buf = 0;
                    llm_env.act_info[act_id].info.adv.adv_data.new_len = 0;
                }

                status = CO_ERROR_PKT_TOO_LONG;
                break;
            }
        }

        // If new data is provided, allocate a new buffer
        if ((param->operation == ADV_DATA_OP_COMPLETE) || (param->operation == ADV_DATA_OP_FIRST_FRAG))
        {
            // Free exchange memory buffer of the data in construction
            if (llm_env.act_info[act_id].info.adv.adv_data.new_buf)
            {
                ble_util_buf_adv_tx_free(llm_env.act_info[act_id].info.adv.adv_data.new_buf);
                llm_env.act_info[act_id].info.adv.adv_data.new_len = 0;
            }

            // Try to allocate a new buffer
            llm_env.act_info[act_id].info.adv.adv_data.new_buf = ble_util_buf_adv_tx_alloc();

            if (!llm_env.act_info[act_id].info.adv.adv_data.new_buf)
                break;
        }

        // Copy the data to exchange memory
        em_wr(&param->data, llm_env.act_info[act_id].info.adv.adv_data.new_buf + llm_env.act_info[act_id].info.adv.adv_data.new_len, param->data_len);

        // Increment total data length
        llm_env.act_info[act_id].info.adv.adv_data.new_len += param->data_len;

        // If the data is complete
        if ((param->operation == ADV_DATA_OP_COMPLETE) || (param->operation == ADV_DATA_OP_LAST_FRAG))
        {
            // If data changed
            if (llm_adv_is_data_changed(llm_env.act_info[act_id].info.adv.adv_data.new_len, llm_env.act_info[act_id].info.adv.adv_data.curr_len))
            {
                // If advertising is enabled
                if (llm_env.act_info[act_id].state == LLM_ADV_EN)
                {
                    // Indicate new data to driver (the driver is responsible to free the old data buffer when no more used)
                    lld_adv_adv_data_update(act_id, llm_env.act_info[act_id].info.adv.adv_data.new_len, llm_env.act_info[act_id].info.adv.adv_data.new_buf);
                }
                else if (llm_env.act_info[act_id].info.adv.adv_data.curr_buf)
                {
                    // Free the current data
                    ble_util_buf_adv_tx_free(llm_env.act_info[act_id].info.adv.adv_data.curr_buf);

#if (BT_53)
                    // If there is a reason to update the RPAs
                    if (llm_env.act_info[act_id].info.adv.chg_reasons & RPA_ADV_DATA_CHG)
                    {
                        // Set RPA flag
                        llm_env.act_info[act_id].info.adv.adv_rpa_upd = true;
                    }
#endif // (BT_53)
                }

                // Move the construction buffer to the current buffer
                llm_env.act_info[act_id].info.adv.adv_data.curr_buf = llm_env.act_info[act_id].info.adv.adv_data.new_buf;
                llm_env.act_info[act_id].info.adv.adv_data.curr_len = llm_env.act_info[act_id].info.adv.adv_data.new_len;
                llm_env.act_info[act_id].info.adv.adv_data.new_buf = 0;
                llm_env.act_info[act_id].info.adv.adv_data.new_len = 0;
            }
            else // Data not changed
            {
                // Free the new data
                ble_util_buf_adv_tx_free(llm_env.act_info[act_id].info.adv.adv_data.new_buf);
                llm_env.act_info[act_id].info.adv.adv_data.new_buf = 0;
                llm_env.act_info[act_id].info.adv.adv_data.new_len = 0;
            }
        }
        // Or unchanged
        else if (param->operation == ADV_DATA_OP_UNCHANGED_DATA)
        {
            // Indicate to driver to update data ID
            lld_adv_adv_data_update(act_id, 0, 0);
        }

        status = CO_ERROR_NO_ERROR;

    }
    while (0);

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

int hci_le_set_ext_scan_rsp_data_cmd_handler(struct hci_le_set_ext_scan_rsp_data_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_UNKNOWN_ADVERTISING_ID;

    do
    {
        struct hci_le_set_ext_adv_param_cmd *ext_param;
        uint8_t act_id;

#if (BLE_ADV_LEGACY_ITF)
        // Check current interface version
        if (llm_env.adv_itf_version == LLM_ADV_ITF_LEGACY)
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        // Use extended interface
        llm_env.adv_itf_version = LLM_ADV_ITF_EXTENDED;
#endif //(BLE_ADV_LEGACY_ITF)

        // Look for the advertising activity with specified handle
        act_id = llm_adv_hdl_to_id(param->adv_hdl, &ext_param);

        // If advertising set does not exists, reject the command
        if (act_id == LLM_ACT_IDX_INVL)
            break;

        // Check parameters
        status = CO_ERROR_INVALID_HCI_PARAM;

        // Operation
        if (param->operation > ADV_DATA_OP_COMPLETE)
            break;

        // Fragment preference
        if (param->frag_pref > ADV_DATA_SHOULD_NOT_FRAG)
            break;

        // Data length
        if (param->data_len > HCI_ADV_DATA_FRAG_MAX_LEN)
            break;

        // If Operation is not 0x03 and Scan_Response_Data_Length is zero
        if ((param->operation != ADV_DATA_OP_COMPLETE) && (param->data_len == 0))
            break;

        // Check that the message is valid as per current context
        status = CO_ERROR_COMMAND_DISALLOWED;

        // If advertising is enabled
        if (llm_env.act_info[act_id].state != LLM_ADV_RSVD)
        {
            // If Operation does not have the value 0x03
            if (param->operation != ADV_DATA_OP_COMPLETE)
                break;

            // If advertising is scannable and there is no scan response data (except if legacy advertising PDU types are being used)
            if ((ext_param->adv_evt_properties & ADV_SCAN) && (param->data_len == 0) && (!(ext_param->adv_evt_properties & ADV_LEGACY)))
                break;

            // If the advertising set uses legacy advertising PDUs and the set is scannable and the scan response data exceeds 31 octets
            if ((ext_param->adv_evt_properties & ADV_LEGACY) && (ext_param->adv_evt_properties & ADV_SCAN) && (param->data_len > ADV_DATA_LEN))
            {
                status = CO_ERROR_INVALID_HCI_PARAM;
                break;
            }
        }

        status = CO_ERROR_MEMORY_CAPA_EXCEED;

        /*
         * If the combined length of the data exceeds the capacity of the advertising set identified by the
         * Advertising_Handle parameter (see Section 7.8.57 LE Read Maximum Advertising Data Length Command) or the
         * amount of memory currently available, all the data shall be discarded and the Controller shall return the
         * error code Memory Capacity Exceeded (0x07).
         */
        if ((llm_env.act_info[act_id].info.adv.scan_rsp_data.new_len + param->data_len) > EM_BLE_ADVDATATXBUF_SIZE)
        {
            // Free exchange memory buffer of the data in construction
            if (llm_env.act_info[act_id].info.adv.scan_rsp_data.new_buf)
            {
                ble_util_buf_adv_tx_free(llm_env.act_info[act_id].info.adv.scan_rsp_data.new_buf);
                llm_env.act_info[act_id].info.adv.scan_rsp_data.new_buf = 0;
                llm_env.act_info[act_id].info.adv.scan_rsp_data.new_len = 0;
            }

            break;
        }

        /*
         * If the advertising set uses extended advertising and the combined length of the
         * data is greater than the maximum that the Controller can transmit within the
         * longest possible auxiliary advertising segment consistent with the current
         * parameters of the advertising set (using the current advertising interval if
         * advertising is enabled), all the data shall be discarded and the Controller shall
         * return the error code Packet Too Long (0x45).
         */
        if (!(ext_param->adv_evt_properties & ADV_LEGACY))
        {
            uint16_t adv_data_len = llm_env.act_info[act_id].info.adv.adv_data.curr_len;
            uint16_t scan_rsp_data_len = llm_env.act_info[act_id].info.adv.scan_rsp_data.new_len + param->data_len;
            uint32_t prim_adv_intv_max  = (ext_param->prim_adv_intv_max[2] << 16) | (ext_param->prim_adv_intv_max[1] << 8) | ext_param->prim_adv_intv_max[0];
            uint32_t intv = prim_adv_intv_max * SLOT_SIZE;

            if (llm_adv_dur(ext_param, adv_data_len, scan_rsp_data_len) >= intv)
            {
                // Free exchange memory buffer of the data in construction
                if (llm_env.act_info[act_id].info.adv.scan_rsp_data.new_buf)
                {
                    ble_util_buf_adv_tx_free(llm_env.act_info[act_id].info.adv.scan_rsp_data.new_buf);
                    llm_env.act_info[act_id].info.adv.scan_rsp_data.new_buf = 0;
                    llm_env.act_info[act_id].info.adv.scan_rsp_data.new_len = 0;
                }

                status = CO_ERROR_PKT_TOO_LONG;
                break;
            }
        }

        // If new data is provided, allocate a new buffer
        if ((param->operation == ADV_DATA_OP_COMPLETE) || (param->operation == ADV_DATA_OP_FIRST_FRAG))
        {
            // Free exchange memory buffer of the data in construction
            if (llm_env.act_info[act_id].info.adv.scan_rsp_data.new_buf)
            {
                ble_util_buf_adv_tx_free(llm_env.act_info[act_id].info.adv.scan_rsp_data.new_buf);
                llm_env.act_info[act_id].info.adv.scan_rsp_data.new_len = 0;
            }

            // Try to allocate a new buffer
            llm_env.act_info[act_id].info.adv.scan_rsp_data.new_buf = ble_util_buf_adv_tx_alloc();

            if (!llm_env.act_info[act_id].info.adv.scan_rsp_data.new_buf)
                break;
        }

        // Copy the data to exchange memory
        em_wr(&param->data, llm_env.act_info[act_id].info.adv.scan_rsp_data.new_buf + llm_env.act_info[act_id].info.adv.scan_rsp_data.new_len, param->data_len);

        // Increment total data length
        llm_env.act_info[act_id].info.adv.scan_rsp_data.new_len += param->data_len;

        // If the data is complete
        if ((param->operation == ADV_DATA_OP_COMPLETE) || (param->operation == ADV_DATA_OP_LAST_FRAG))
        {
            // If data changed
            if (llm_adv_is_data_changed(llm_env.act_info[act_id].info.adv.scan_rsp_data.new_len, llm_env.act_info[act_id].info.adv.scan_rsp_data.curr_len))
            {
                // If advertising is enabled
                if (llm_env.act_info[act_id].state == LLM_ADV_EN)
                {
                    // Indicate new data to driver (the driver is responsible to free the old data buffer when no more used)
                    lld_adv_scan_rsp_data_update(act_id, llm_env.act_info[act_id].info.adv.scan_rsp_data.new_len, llm_env.act_info[act_id].info.adv.scan_rsp_data.new_buf);
                }
                else if (llm_env.act_info[act_id].info.adv.scan_rsp_data.curr_buf)
                {
                    // Free the current data
                    ble_util_buf_adv_tx_free(llm_env.act_info[act_id].info.adv.scan_rsp_data.curr_buf);

#if (BT_53)
                    // If there is a reason to update the RPAs
                    if (llm_env.act_info[act_id].info.adv.chg_reasons & RPA_SCAN_RSP_DATA_CHG)
                    {
                        // Set RPA flag
                        llm_env.act_info[act_id].info.adv.adv_rpa_upd = true;
                    }
#endif // (BT_53)
                }

                // Move the construction buffer to the current buffer
                llm_env.act_info[act_id].info.adv.scan_rsp_data.curr_buf = llm_env.act_info[act_id].info.adv.scan_rsp_data.new_buf;
                llm_env.act_info[act_id].info.adv.scan_rsp_data.curr_len = llm_env.act_info[act_id].info.adv.scan_rsp_data.new_len;
                llm_env.act_info[act_id].info.adv.scan_rsp_data.new_buf = 0;
                llm_env.act_info[act_id].info.adv.scan_rsp_data.new_len = 0;
            }
            else // Data not changed
            {
                // Free the new data
                ble_util_buf_adv_tx_free(llm_env.act_info[act_id].info.adv.scan_rsp_data.new_buf);
                llm_env.act_info[act_id].info.adv.scan_rsp_data.new_buf = 0;
                llm_env.act_info[act_id].info.adv.scan_rsp_data.new_len = 0;
            }
        }
        // Or unchanged
        else if (param->operation == ADV_DATA_OP_UNCHANGED_DATA)
        {
            // Indicate to driver to update data ID
            lld_adv_scan_rsp_data_update(act_id, 0, 0);
        }

        status = CO_ERROR_NO_ERROR;

    }
    while (0);

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

int hci_le_set_ext_adv_en_cmd_handler(struct hci_le_set_ext_adv_en_cmd *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;
    bool cmd_cmp   = false;

    do
    {
        uint8_t act_id;

#if (BLE_ADV_LEGACY_ITF)
        // Check current interface version
        if (llm_env.adv_itf_version == LLM_ADV_ITF_LEGACY)
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        // Use extended interface
        llm_env.adv_itf_version = LLM_ADV_ITF_EXTENDED;
#endif //(BLE_ADV_LEGACY_ITF)

        // If this command disables and there are still some sets stopping
        if ((param->enable == ADV_DIS) && (llm_env.nb_ext_adv_sets_stopping > 0))
        {
            // process this message later
            return (KE_MSG_SAVED);
        }

        // Check parameters
        if (param->enable > ADV_EN)
            break;

        // If the Enable parameter is set to 0x01 (Advertising is enabled) and Number_of_Sets is set to 0x00
        if ((param->enable == ADV_EN) && (param->nb_sets == 0))
            break;

        // Check if advertising set are targeted
        if (param->nb_sets > 0)
        {
            uint8_t nb_connectable = 0;
            bool reject = false;

            // Verify that all advertising handles exist and data is complete
            for (uint8_t i = 0; i < param->nb_sets; i++)
            {
                struct hci_ext_adv_set_en_param *p_set = &(param->sets[i]);

                // If advertising handle is not in the valid range
                if (p_set->adv_hdl > ADV_HDL_MAX)
                {
                    reject = true;
                    break;
                }

                // Look for the advertising activity with specified handle
                act_id = llm_adv_hdl_to_id(p_set->adv_hdl, NULL);

                // If advertising set does not exist
                if (act_id == LLM_ACT_IDX_INVL)
                {
                    status = CO_ERROR_UNKNOWN_ADVERTISING_ID;
                    reject = true;
                    break;
                }

                // If advertising set is already disabled
                if ((param->enable == ADV_DIS) && (llm_env.act_info[act_id].state == LLM_ADV_RSVD))
                {
                    continue;
                }

                // If data is incomplete
                if (llm_env.act_info[act_id].info.adv.adv_data.new_buf || llm_env.act_info[act_id].info.adv.scan_rsp_data.new_buf)
                {
                    reject = true;
                    break;
                }

                if (param->enable == ADV_EN)
                {
                    // Point Host parameters
                    struct hci_le_set_ext_adv_param_cmd *ext_param = (struct hci_le_set_ext_adv_param_cmd *) llm_env.act_info[act_id].host_params;

                    // If advertising is scannable and there is no scan response data (except if legacy advertising PDU types are being used)
                    if ((ext_param->adv_evt_properties & ADV_SCAN) && (llm_env.act_info[act_id].info.adv.scan_rsp_data.curr_len == 0) && (!(ext_param->adv_evt_properties & ADV_LEGACY)))
                    {
                        status = CO_ERROR_COMMAND_DISALLOWED;
                        reject = true;
                        break;
                    }

                    // If advertising is extended connectable and the data does not fit in a single packet
                    if (!(ext_param->adv_evt_properties & ADV_LEGACY) && (ext_param->adv_evt_properties & ADV_CON))
                    {
                        if (llm_adv_con_len_check(ext_param, llm_env.act_info[act_id].info.adv.adv_data.curr_len) != CO_ERROR_NO_ERROR)
                        {
                            status = CO_ERROR_INVALID_HCI_PARAM;
                            reject = true;
                            break;
                        }
                    }

                    // If the advertising set uses legacy advertising PDUs
                    if (ext_param->adv_evt_properties & ADV_LEGACY)
                    {
                        // If the set supports advertising data and the data exceeds 31 octets
                        // or if the set is scannable and the scan response data exceeds 31 octets
                        if ((!(ext_param->adv_evt_properties & (ADV_DIRECT | ADV_DIRECT_HI)) && (llm_env.act_info[act_id].info.adv.adv_data.curr_len > ADV_DATA_LEN))
                                || ((ext_param->adv_evt_properties & ADV_SCAN) && (llm_env.act_info[act_id].info.adv.scan_rsp_data.curr_len > ADV_DATA_LEN)))
                        {
                            status = CO_ERROR_COMMAND_DISALLOWED;
                            reject = true;
                            break;
                        }
                    }

                    // If the advertising set's Own_Address_Type parameter is set to 0x01 and the random address for the advertising set has not been initialized
                    if (((ext_param->own_addr_type & ADDR_MASK) == ADDR_RAND)
                            && (co_bdaddr_compare(&llm_env.act_info[act_id].info.adv.bd_addr, &co_null_bdaddr) == true))
                    {
                        status = CO_ERROR_INVALID_HCI_PARAM;
                        reject = true;
                        break;
                    }

                    // If connectable advertising
                    if (ext_param->adv_evt_properties & ADV_CON)
                    {
#if (BLE_PERIPHERAL)
                        // If directed advertising
                        if ((ext_param->adv_evt_properties & (ADV_DIRECT | ADV_DIRECT_HI)) != 0)
                        {
                            // Check if device is already connected
                            if (llm_is_dev_connected(&ext_param->peer_addr, ext_param->peer_addr_type))
                            {
                                status = CO_ERROR_CON_ALREADY_EXISTS;
                                reject = true;
                                break;
                            }
                        }

                        // Count number of new connectable advertising sets
                        if (llm_env.act_info[act_id].state != LLM_ADV_EN)
                        {
                            nb_connectable += 1;
                        }
#else // (BLE_PERIPHERAL)
                        status = CO_ERROR_INVALID_HCI_PARAM;
                        break;
#endif // (BLE_PERIPHERAL)
                    }

                    /*
                     * If extended advertising is being used and the length of any advertising data or
                     * of any scan response data is greater than the maximum that the Controller can
                     * transmit within the longest possible auxiliary advertising segment consistent
                     * with the chosen advertising interval, the Controller shall return the error code
                     * Packet Too Long (0x45).
                     */
                    if (!(ext_param->adv_evt_properties & ADV_LEGACY))
                    {
                        uint16_t adv_data_len = llm_env.act_info[act_id].info.adv.adv_data.curr_len;
                        uint16_t scan_rsp_data_len = llm_env.act_info[act_id].info.adv.scan_rsp_data.curr_len;
                        uint32_t prim_adv_intv_max  = (ext_param->prim_adv_intv_max[2] << 16) | (ext_param->prim_adv_intv_max[1] << 8) | ext_param->prim_adv_intv_max[0];
                        uint32_t intv = prim_adv_intv_max * SLOT_SIZE;

                        if (llm_adv_dur(ext_param, adv_data_len, scan_rsp_data_len) >= intv)
                        {
                            status = CO_ERROR_PKT_TOO_LONG;
                            reject = true;
                            break;
                        }
                    }

                }

                // Replace advertising handle by its associated activity identifier
                p_set->adv_hdl = act_id;
            }

            if (reject)
                break;

            // Check that there is sufficient number of additional free activity IDs for connectable advertising sets
            if (param->enable && nb_connectable)
            {
                uint8_t nb_free = 0;

                // Count number of free activity IDs
                for (uint8_t act_id = 0; act_id < BLE_ACTIVITY_MAX_CFG; act_id++)
                {
                    nb_free += (llm_env.act_info[act_id].state == LLM_FREE);

                    if (nb_free >= nb_connectable)
                        break;
                }

                // Reject if cannot allocate one new activity ID per connectable advertising
                if (nb_free < nb_connectable)
                {
                    status = CO_ERROR_MEMORY_CAPA_EXCEED;
                    break;
                }
            }

            // Apply the parameters to each advertising set specified
            for (uint8_t i = 0; i < param->nb_sets; i++)
            {
                struct hci_ext_adv_set_en_param *p_set = &(param->sets[i]);
                // activity handle has been replaced by activity identifier
                uint8_t act_id = p_set->adv_hdl;

                // If the advertising set is already enabled
                if ((llm_env.act_info[act_id].state == LLM_ADV_EN) && param->enable)
                {
                    // Reset the event count, update the duration and the maximum number of extended advertising events
                    lld_adv_restart(act_id, p_set->duration, p_set->max_ext_adv_evt);
                    continue;
                }

                // If the advertising set is already disabled
                if ((llm_env.act_info[act_id].state != LLM_ADV_EN) && !param->enable)
                {
                    continue;
                }

                if (param->enable)
                {
                    struct hci_le_set_ext_adv_param_cmd *ext_param = (struct hci_le_set_ext_adv_param_cmd *) llm_env.act_info[act_id].host_params;
                    struct lld_adv_params adv_par;
                    uint8_t con_act_id = LLM_ACT_IDX_INVL;

                    // If advertising is connectable
                    if (ext_param->adv_evt_properties & ADV_CON)
                    {
                        // Allocate a new activity identifier for the potential connection
                        status = llm_activity_free_get(&con_act_id);

                        // Check a new activity identifier has been found
                        if (status == CO_ERROR_NO_ERROR)
                        {
                            // Save it to the advertising activity parameters
                            llm_env.act_info[act_id].info.adv.con_act_id = con_act_id;

                            // Reserve the activity identifier for the potential connection
                            llm_env.act_info[con_act_id].state = LLM_CON_RSVD;
                        }
                        else
                        {
                            // Should not happen as the number of free activity identifiers was previously checked
                            ASSERT_ERR_FORCE(0);
                        }
                    }

                    // Fill advertising parameters
                    adv_par.adv_evt_properties = ext_param->adv_evt_properties;
                    adv_par.prim_adv_intv      = (ext_param->prim_adv_intv_max[2] << 16) | (ext_param->prim_adv_intv_max[1] << 8) | ext_param->prim_adv_intv_max[0];
                    adv_par.prim_adv_ch_map    = ext_param->prim_adv_ch_map;
                    adv_par.own_addr_type      = ext_param->own_addr_type;
                    adv_par.peer_addr_type     = ext_param->peer_addr_type;
                    adv_par.peer_addr          = ext_param->peer_addr;
                    adv_par.adv_filter_policy  = (ext_param->adv_evt_properties & ADV_DIRECT) ? ADV_ALLOW_SCAN_ANY_CON_ANY : ext_param->adv_filt_policy;
                    adv_par.adv_tx_pwr         = llm_env.act_info[act_id].info.adv.tx_pwr;
                    adv_par.prim_adv_phy       = ext_param->prim_adv_phy;
                    adv_par.sec_adv_max_skip   = !(ext_param->adv_evt_properties & ADV_LEGACY) ? ext_param->sec_adv_max_skip : 0;
                    adv_par.sec_adv_phy        = !(ext_param->adv_evt_properties & ADV_LEGACY) ? ext_param->sec_adv_phy : PHY_1MBPS_VALUE;
                    adv_par.adv_sid            = ext_param->adv_sid;
                    adv_par.scan_req_notif_en  = ext_param->scan_req_notif_en;
                    adv_par.own_addr           = llm_env.act_info[act_id].info.adv.bd_addr;
                    adv_par.init_adv_data_buf  = llm_env.act_info[act_id].info.adv.adv_data.curr_buf;
                    adv_par.init_adv_data_len  = llm_env.act_info[act_id].info.adv.adv_data.curr_len;
                    adv_par.init_scan_rsp_data_buf = llm_env.act_info[act_id].info.adv.scan_rsp_data.curr_buf;
                    adv_par.init_scan_rsp_data_len = llm_env.act_info[act_id].info.adv.scan_rsp_data.curr_len;
                    adv_par.duration           = p_set->duration;
                    adv_par.max_ext_adv_evt    = p_set->max_ext_adv_evt;
                    adv_par.addr_resolution_en = llm_env.addr_resolution_en;
                    adv_par.ral_idx            = BLE_RAL_MAX;
                    adv_par.con_act_id         = con_act_id;
#if (BT_53)
                    adv_par.chg_reasons = llm_env.act_info[act_id].info.adv.chg_reasons;
                    adv_par.adv_rpa_upd = llm_env.act_info[act_id].info.adv.adv_rpa_upd;
#endif // (BT_53)

#if (EAVESDROPPING_SUPPORT)
                    if (llm_env.act_info[act_id].info.adv.manual_adv_params != NULL)
                    {
                        // Copy the manual adv parameters
                        adv_par.manual_adv_params.en = llm_env.act_info[act_id].info.adv.manual_adv_params->enable;
                        adv_par.manual_adv_params.time_offset = llm_env.act_info[act_id].info.adv.manual_adv_params->time_offset;
                        adv_par.manual_adv_params.max_rand_offset = llm_env.act_info[act_id].info.adv.manual_adv_params->max_rand_offset;
                    }
                    else
                    {
                        adv_par.manual_adv_params.en = false;
                    }

                    if (llm_env.act_info[act_id].info.adv.cust_adv_params != NULL)
                    {
                        // Copy the custom adv parameters
                        adv_par.cust_adv_params.periodicity = llm_env.act_info[act_id].info.adv.cust_adv_params->periodicity;
                        adv_par.cust_adv_params.en = llm_env.act_info[act_id].info.adv.cust_adv_params->enable;
                        adv_par.cust_adv_params.rand_en = llm_env.act_info[act_id].info.adv.cust_adv_params->rand_en;
                        adv_par.cust_adv_params.time_offset = llm_env.act_info[act_id].info.adv.cust_adv_params->time_offset;
                        adv_par.cust_adv_params.us_offset = llm_env.act_info[act_id].info.adv.cust_adv_params->us_offset;
                        memcpy(&adv_par.cust_adv_params.bd_addr.addr[0], &llm_env.act_info[act_id].info.adv.cust_adv_params->bd_addr.addr[0], BD_ADDR_LEN);
                        adv_par.cust_adv_params.group_ext_adv_ind = (!(ext_param->adv_evt_properties & ADV_LEGACY)) ? llm_env.act_info[act_id].info.adv.cust_adv_params->group_ext_adv_ind : false;
                    }
                    else
                    {
                        adv_par.cust_adv_params.en = false;
                    }
#endif // EAVESDROPPING_SUPPORT

                    // Check if peer device has a corresponding resolving list entry
                    if ((adv_par.addr_resolution_en && (ext_param->adv_evt_properties & ADV_DIRECT))
                            || (ext_param->own_addr_type & ADDR_RPA_MASK))
                    {
                        // Find peer device in RAL
                        adv_par.ral_idx = llm_ral_search(&ext_param->peer_addr, ext_param->peer_addr_type);

                        // If peer has been found, check if local RPA needs/can to be generated
                        if ((adv_par.ral_idx < BLE_RAL_MAX)
                                && ((ext_param->own_addr_type & ADDR_RPA_MASK) != 0)
                                && (memcmp(&llm_env.ral[adv_par.ral_idx].local_irk.key[0], co_null_key, KEY_LEN)))
                        {
                            // Generate an RPA
                            aes_rpa_gen((const aes_key_t *) &llm_env.ral[adv_par.ral_idx].local_irk, llm_adv_rpa_gen_cb, act_id);
                        }
                    }

                    // Start advertising
                    status = lld_adv_start(act_id, &adv_par);

                    if (status == CO_ERROR_NO_ERROR)
                    {
                        uint8_t per_act_id;

                        llm_env.act_info[act_id].state = LLM_ADV_EN;

#if (BT_53)
                        // Clear RPA flag
                        llm_env.act_info[act_id].info.adv.adv_rpa_upd = false;
#endif // (BT_53)

                        // Clear local RPA
                        memset(&llm_env.act_info[act_id].info.adv.local_rpa, 0, BD_ADDR_LEN);

                        // Check if there is an associated periodic advertising handle that is pending
                        per_act_id = llm_env.act_info[act_id].info.adv.per_act_id;
                        if (!(ext_param->adv_evt_properties & (ADV_CON | ADV_SCAN)) && !(ext_param->adv_evt_properties & ADV_LEGACY) && (per_act_id < BLE_ACTIVITY_MAX_CFG) && (llm_env.act_info[per_act_id].state == LLM_PER_ADV_PENDING))
                        {
                            // Start periodic advertising
                            status = llm_adv_per_adv_start(act_id, per_act_id);

                            // Check a new activity identifier has been found
                            if (status != CO_ERROR_NO_ERROR)
                            {
                                reject = true;
                                break;
                            }
                        }
                    }
                    else
                    {
                        // Not supposed to fail starting the driver
                        ASSERT_INFO_FORCE(0, act_id, 0);
                        reject = true;
                        break;
                    }
                }
                else
                {
                    // Stop advertising
                    status = lld_adv_stop(act_id);

                    if (status == CO_ERROR_NO_ERROR)
                    {
                        llm_env.act_info[act_id].state = LLM_ADV_STOPPING;
                        llm_env.nb_ext_adv_sets_stopping++;
                    }
                }
            }

            if (reject)
                break;

        }
        else
        {
            // Disable all ongoing advertising sets
            for (act_id = 0; act_id < BLE_ACTIVITY_MAX_CFG; act_id++)
            {
                // Check if activity is allocated to advertising that is enabled
                if (llm_env.act_info[act_id].state == LLM_ADV_EN)
                {
                    // Stop advertising
                    status = lld_adv_stop(act_id);

                    if (status == CO_ERROR_NO_ERROR)
                    {
                        llm_env.act_info[act_id].state = LLM_ADV_STOPPING;
                        llm_env.nb_ext_adv_sets_stopping++;
                    }
                }
            }
        }

        status = CO_ERROR_NO_ERROR;

    }
    while (0);

    // If advertising is stopping, wait the indication from the driver before reporting the command complete event
    if ((status != CO_ERROR_NO_ERROR) || (param->enable != ADV_DIS) || (llm_env.nb_ext_adv_sets_stopping == 0) || cmd_cmp)
    {
        // Send the command complete event
        llm_cmd_cmp_send(opcode, status);
    }

    return (KE_MSG_CONSUMED);
}

int hci_le_set_per_adv_param_cmd_handler(struct hci_le_set_per_adv_param_cmd const *param, uint16_t opcode)
{
    uint8_t ret_status = KE_MSG_CONSUMED;
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    do
    {
        uint8_t per_act_id, ext_act_id;

#if (BLE_ADV_LEGACY_ITF)
        // Check current interface version
        if (llm_env.adv_itf_version == LLM_ADV_ITF_LEGACY)
            break;

        // Use extended interface
        llm_env.adv_itf_version = LLM_ADV_ITF_EXTENDED;
#endif //(BLE_ADV_LEGACY_ITF)

        // If advertising handle is not in the valid range
        if (param->adv_hdl > ADV_HDL_MAX)
        {
            status = CO_ERROR_INVALID_HCI_PARAM;
            break;
        }

        // Look for the advertising activity with specified handle
        ext_act_id = llm_adv_hdl_to_id(param->adv_hdl, NULL);

        // If the corresponding advertising set does not already exist, then the Controller shall return the error code Unknown Advertising Identifier (0x42)
        if (ext_act_id == LLM_ACT_IDX_INVL)
        {
            status = CO_ERROR_UNKNOWN_ADVERTISING_ID;
            break;
        }

        // Check if there is an associated periodic advertising handle
        per_act_id = llm_env.act_info[ext_act_id].info.adv.per_act_id;
        if (per_act_id != LLM_ACT_IDX_INVL)
        {
            // If periodic advertising is already enabled, reject the command
            if (llm_env.act_info[per_act_id].state != LLM_PER_ADV_RSVD)
                break;
        }
        else
        {
            // If not present, try to allocate an activity identifier
            status = llm_activity_free_get(&per_act_id);

            // If not possible to start a new periodic advertising activity, reject the command
            if (status != CO_ERROR_NO_ERROR)
            {
                status = CO_ERROR_MEMORY_CAPA_EXCEED;
                break;
            }
            else
            {
                // These activities are now associated
                llm_env.act_info[ext_act_id].info.adv.per_act_id = per_act_id;
                llm_env.act_info[per_act_id].info.per_adv.ass_act_id = LLM_ACT_IDX_INVL;
            }
        }

        status = CO_ERROR_INVALID_HCI_PARAM;

        // Check parameters
        if (param->adv_hdl > ADV_HDL_MAX)
            break;

        // The Periodic_Advertising_Interval_Min parameter shall be less than or equal to the Periodic_Advertising_Interval_Max parameter
        // Must be at least 6
        if ((param->adv_intv_min < PER_ADV_INTERVAL_MIN) || (param->adv_intv_min > param->adv_intv_max))
            break;

        // Check advertising set parameters
        {
            struct hci_le_set_ext_adv_param_cmd *ext_param = (struct hci_le_set_ext_adv_param_cmd *) llm_env.act_info[ext_act_id].host_params;

            // If the advertising set identified by the Advertising_Handle specified scannable, connectable, legacy, or anonymous advertising
            if (ext_param->adv_evt_properties & (ADV_SCAN | ADV_CON | ADV_LEGACY | ADV_ANONYMOUS))
                break;
        }

        /*
         * If the advertising set already contains periodic advertising data and the length
         * of the data is greater than the maximum that the Controller can transmit within
         * a periodic advertising interval of Periodic_Advertising_Interval_Max, the
         * Controller shall return the error code Packet Too Long (0x45).
         */
        if (llm_env.act_info[per_act_id].info.per_adv.adv_data.curr_len)
        {
            struct hci_le_set_ext_adv_param_cmd *ext_param = (struct hci_le_set_ext_adv_param_cmd *) llm_env.act_info[ext_act_id].host_params;
            uint16_t len = llm_env.act_info[per_act_id].info.per_adv.adv_data.curr_len;
            uint32_t intv = param->adv_intv_max << 2;

            if (llm_per_adv_chain_dur(len, ext_param->sec_adv_phy) >= intv)
            {
                status = CO_ERROR_PKT_TOO_LONG;
                break;
            }
        }

        // Free potentially stored parameters
        if (llm_env.act_info[per_act_id].host_params != NULL)
        {
            ke_msg_free(ke_param2msg(llm_env.act_info[per_act_id].host_params));
            llm_env.act_info[per_act_id].host_params = NULL;
        }

        // Keep pointer to parameters
        llm_env.act_info[per_act_id].host_params = param;

        // Assign the allocated activity identifier to periodic advertising
        llm_env.act_info[per_act_id].state = LLM_PER_ADV_RSVD;

        // Message needs to be maintained
        ret_status = KE_MSG_NO_FREE;

        status = CO_ERROR_NO_ERROR;
    }
    while (0);

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (ret_status);
}

int hci_le_set_per_adv_data_cmd_handler(struct hci_le_set_per_adv_data_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;

    do
    {
        uint8_t per_act_id, ext_act_id;

#if (BLE_ADV_LEGACY_ITF)
        // Check current interface version
        if (llm_env.adv_itf_version == LLM_ADV_ITF_LEGACY)
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        // Use extended interface
        llm_env.adv_itf_version = LLM_ADV_ITF_EXTENDED;
#endif //(BLE_ADV_LEGACY_ITF)

        // If advertising handle is not in the valid range
        if (param->adv_hdl > ADV_HDL_MAX)
        {
            break;
        }

        // Look for the advertising activity with specified handle
        ext_act_id = llm_adv_hdl_to_id(param->adv_hdl, NULL);

        // If the corresponding advertising set does not already exist, then the Controller shall return the error code Unknown Advertising Identifier (0x42)
        if (ext_act_id == LLM_ACT_IDX_INVL)
        {
            status = CO_ERROR_UNKNOWN_ADVERTISING_ID;
            break;
        }

        // If the advertising set has not been configured for periodic advertising, then the Controller shall return the error code Command Disallowed (0x0C)
        per_act_id = llm_env.act_info[ext_act_id].info.adv.per_act_id;
        if (per_act_id == LLM_ACT_IDX_INVL)
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        // Check parameters
        status = CO_ERROR_INVALID_HCI_PARAM;

        // Operation
#if (BT_53)
        if (param->operation > ADV_DATA_OP_UNCHANGED_DATA)
#else // (BT_53)
        if (param->operation > ADV_DATA_OP_COMPLETE)
#endif // (BT_53)
            break;

        // Data length
        if (param->data_len > HCI_PER_ADV_DATA_FRAG_MAX_LEN)
            break;

#if (BT_53)
        /* If Operation is 0x04 and:
         *  - periodic advertising is currently disabled for the advertising set;
         *  - the periodic advertising set contains no data; or
         *  - Advertising_Data_Length is not zero; */
        if ((param->operation == ADV_DATA_OP_UNCHANGED_DATA)
                && ((llm_env.act_info[per_act_id].state != LLM_PER_ADV_EN)
                    || (llm_env.act_info[per_act_id].info.per_adv.adv_data.curr_len == 0) || (param->data_len != 0)))
        {
            break;
        }
#endif // (BT_53)

#if (BT_53)
        // If Operation is not 0x03 or 0x04 and Advertising_Data_Length is zero
        if (((param->operation != ADV_DATA_OP_COMPLETE) && (param->operation != ADV_DATA_OP_UNCHANGED_DATA))
                && (param->data_len == 0))
#else // (BT_53)
        // If Operation is not 0x03 and Advertising_Data_Length is zero
        if ((param->operation != ADV_DATA_OP_COMPLETE) && (param->data_len == 0))
#endif // (BT_53)
            break;

        // Check that the message is valid as per current context
        status = CO_ERROR_COMMAND_DISALLOWED;

#if (BT_53)
        // If advertising is enabled and Operation does not have the value 0x03 or 0x04
        if ((llm_env.act_info[per_act_id].state == LLM_PER_ADV_EN)
                && ((param->operation != ADV_DATA_OP_COMPLETE) && (param->operation != ADV_DATA_OP_UNCHANGED_DATA)))
#else // (BT_53)
        // If advertising is enabled and Operation does not have the value 0x03
        if ((param->operation != ADV_DATA_OP_COMPLETE) && (llm_env.act_info[per_act_id].state == LLM_PER_ADV_EN))
#endif // (BT_53)
            break;

#if (BT_53)
        // If operation is 0x04
        if (param->operation == ADV_DATA_OP_UNCHANGED_DATA)
        {
            // If the ADI is included
            if (llm_env.act_info[per_act_id].info.per_adv.include_adi)
            {
                // Update the Advertising DID
                lld_per_adv_did_update(per_act_id);
            }
            status = CO_ERROR_NO_ERROR;
            break;
        }
#endif // (BT_53)

        status = CO_ERROR_MEMORY_CAPA_EXCEED;

        /*
         * If the combined length of the data exceeds the capacity of the advertising set identified by the
         * Advertising_Handle parameter (see Section 7.8.57 LE Read Maximum Advertising Data Length Command) or the
         * amount of memory currently available, all the data shall be discarded and the Controller shall return the
         * error code Memory Capacity Exceeded (0x07).
         */
        if ((llm_env.act_info[per_act_id].info.per_adv.adv_data.new_len + param->data_len) > EM_BLE_ADVDATATXBUF_SIZE)
        {
            // Free exchange memory buffer of the data in construction
            if (llm_env.act_info[per_act_id].info.per_adv.adv_data.new_buf)
            {
                ble_util_buf_adv_tx_free(llm_env.act_info[per_act_id].info.per_adv.adv_data.new_buf);
                llm_env.act_info[per_act_id].info.per_adv.adv_data.new_buf = 0;
                llm_env.act_info[per_act_id].info.per_adv.adv_data.new_len = 0;
            }

            break;
        }

        /*
         * If the combined length of the data is greater than the maximum that the
         * Controller can transmit within the current periodic advertising interval (if
         * periodic advertising is currently enabled) or the
         * Periodic_Advertising_Interval_Max for the advertising set (if currently
         * disabled), all the data shall be discarded and the Controller shall return the
         * error code Packet Too Long (0x45).
         */
        {
            struct hci_le_set_ext_adv_param_cmd *ext_param = (struct hci_le_set_ext_adv_param_cmd *) llm_env.act_info[ext_act_id].host_params;
            uint16_t len = llm_env.act_info[per_act_id].info.per_adv.adv_data.new_len + param->data_len;
            uint32_t intv = 0;
            if (llm_env.act_info[per_act_id].state == LLM_PER_ADV_EN)
            {
                intv = llm_env.act_info[per_act_id].plan_elt.interval;
            }
            else
            {
                struct hci_le_set_per_adv_param_cmd *per_param = (struct hci_le_set_per_adv_param_cmd *) llm_env.act_info[per_act_id].host_params;
                intv = per_param->adv_intv_max << 2;
            }

            if (llm_per_adv_chain_dur(len, ext_param->sec_adv_phy) >= intv)
            {
                // Free exchange memory buffer of the data in construction
                if (llm_env.act_info[per_act_id].info.per_adv.adv_data.new_buf)
                {
                    ble_util_buf_adv_tx_free(llm_env.act_info[per_act_id].info.per_adv.adv_data.new_buf);
                    llm_env.act_info[per_act_id].info.per_adv.adv_data.new_buf = 0;
                    llm_env.act_info[per_act_id].info.per_adv.adv_data.new_len = 0;
                }

                status = CO_ERROR_PKT_TOO_LONG;
                break;
            }
        }

        // If new data is provided, allocate a new buffer
        if ((param->operation == ADV_DATA_OP_COMPLETE) || (param->operation == ADV_DATA_OP_FIRST_FRAG))
        {
            // Free exchange memory buffer of the data in construction
            if (llm_env.act_info[per_act_id].info.per_adv.adv_data.new_buf)
            {
                ble_util_buf_adv_tx_free(llm_env.act_info[per_act_id].info.per_adv.adv_data.new_buf);
                llm_env.act_info[per_act_id].info.per_adv.adv_data.new_len = 0;
            }

            // Try to allocate a new buffer
            llm_env.act_info[per_act_id].info.per_adv.adv_data.new_buf = ble_util_buf_adv_tx_alloc();

            if (!llm_env.act_info[per_act_id].info.per_adv.adv_data.new_buf)
                break;
        }

        // Copy the data to exchange memory
        em_wr(&param->data, llm_env.act_info[per_act_id].info.per_adv.adv_data.new_buf + llm_env.act_info[per_act_id].info.per_adv.adv_data.new_len, param->data_len);

        // Increment total data length
        llm_env.act_info[per_act_id].info.per_adv.adv_data.new_len += param->data_len;

        // If the data is complete
        if ((param->operation == ADV_DATA_OP_COMPLETE) || (param->operation == ADV_DATA_OP_LAST_FRAG))
        {
            // If periodic advertising is enabled
            if (llm_env.act_info[per_act_id].state == LLM_PER_ADV_EN)
            {
                // Indicate new data to driver (the driver is responsible to free the old data buffer when no more used)
                lld_per_adv_data_update(per_act_id, llm_env.act_info[per_act_id].info.per_adv.adv_data.new_len, llm_env.act_info[per_act_id].info.per_adv.adv_data.new_buf);
            }
            else if (llm_env.act_info[per_act_id].info.per_adv.adv_data.curr_buf)
            {
                // Free the current data
                ble_util_buf_adv_tx_free(llm_env.act_info[per_act_id].info.per_adv.adv_data.curr_buf);
            }

            // Move the construction buffer to the current buffer
            llm_env.act_info[per_act_id].info.per_adv.adv_data.curr_buf = llm_env.act_info[per_act_id].info.per_adv.adv_data.new_buf;
            llm_env.act_info[per_act_id].info.per_adv.adv_data.curr_len = llm_env.act_info[per_act_id].info.per_adv.adv_data.new_len;
            llm_env.act_info[per_act_id].info.per_adv.adv_data.new_buf = 0;
            llm_env.act_info[per_act_id].info.per_adv.adv_data.new_len = 0;
        }

        status = CO_ERROR_NO_ERROR;
    }
    while (0);

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

int hci_le_set_per_adv_en_cmd_handler(struct hci_le_set_per_adv_en_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;
    bool cmd_cmp   = false;

    do
    {
        uint8_t per_act_id, ext_act_id;

#if (BLE_ADV_LEGACY_ITF)
        // Check current interface version
        if (llm_env.adv_itf_version == LLM_ADV_ITF_LEGACY)
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        // Use extended interface
        llm_env.adv_itf_version = LLM_ADV_ITF_EXTENDED;
#endif //(BLE_ADV_LEGACY_ITF)

        // Check parameters
#if (BT_53)
        if (param->enable > (PER_ADV_EN_BIT | PER_ADV_EN_ADI_BIT))
#else // (BT_53)
        if (param->enable > ADV_EN)
#endif // (BT_53)
            break;

        // If advertising handle is not in the valid range
        if (param->adv_hdl > ADV_HDL_MAX)
            break;

        // Look for the advertising activity with specified handle
        ext_act_id = llm_adv_hdl_to_id(param->adv_hdl, NULL);

        // If the advertising set corresponding to the Advertising_Handle parameter does not exist, the Controller shall return the error code Unknown Advertising Identifier (0x42)
        if (ext_act_id == LLM_ACT_IDX_INVL)
        {
            status = CO_ERROR_UNKNOWN_ADVERTISING_ID;
            break;
        }

        per_act_id = llm_env.act_info[ext_act_id].info.adv.per_act_id;
        if (per_act_id == LLM_ACT_IDX_INVL)
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        // If the periodic advertising data in the advertising set is not complete, the Controller shall return the error code Command Disallowed (0x0C)
        if (llm_env.act_info[per_act_id].info.per_adv.adv_data.new_buf)
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        status = CO_ERROR_NO_ERROR;

#if (BT_53)
        // Remember whether the ADI field needs to be included or not
        // Bit 1 is ignored if bit 0 is not set
        llm_env.act_info[per_act_id].info.per_adv.include_adi = (param->enable == (PER_ADV_EN_BIT | PER_ADV_EN_ADI_BIT));
#endif // (BT_53)

#if (BT_53)
        if (param->enable & PER_ADV_EN_BIT)
#else // (BT_53)
        if (param->enable)
#endif // (BT_53)
        {
            struct hci_le_set_ext_adv_param_cmd *ext_param = (struct hci_le_set_ext_adv_param_cmd *) llm_env.act_info[ext_act_id].host_params;

            // If the advertising set identified by the Advertising_Handle specified scannable, connectable, legacy, or anonymous advertising
            if (ext_param->adv_evt_properties & (ADV_SCAN | ADV_CON | ADV_LEGACY | ADV_ANONYMOUS))
            {
                status = CO_ERROR_COMMAND_DISALLOWED;
                break;
            }

            /*
             * If Enable is set to 0x01 and the length of the periodic advertising data is greater
             * than the maximum that the Controller can transmit within the chosen periodic
             * advertising interval, the Controller shall return the error code Packet Too Long (0x45).
             */
            {
                struct hci_le_set_per_adv_param_cmd *per_param = (struct hci_le_set_per_adv_param_cmd *) llm_env.act_info[per_act_id].host_params;
                uint16_t len = llm_env.act_info[per_act_id].info.per_adv.adv_data.curr_len;
                uint32_t intv = intv = per_param->adv_intv_max << 2;

                if (llm_per_adv_chain_dur(len, ext_param->sec_adv_phy) >= intv)
                {
                    status = CO_ERROR_PKT_TOO_LONG;
                    break;
                }
            }

            if (llm_env.act_info[per_act_id].state == LLM_PER_ADV_STOPPING)
            {
                status = CO_ERROR_COMMAND_DISALLOWED;
                break;
            }
            else if (llm_env.act_info[per_act_id].state == LLM_PER_ADV_RSVD)
            {
                llm_env.act_info[per_act_id].state = LLM_PER_ADV_PENDING;
            }

            // If the advertising set is not currently enabled, the periodic advertising is not started until the advertising set is enabled
            if (!(ext_param->adv_evt_properties & (ADV_CON | ADV_SCAN)) && !(ext_param->adv_evt_properties & ADV_LEGACY) && (llm_env.act_info[ext_act_id].state == LLM_ADV_EN) && (llm_env.act_info[per_act_id].state == LLM_PER_ADV_PENDING))
            {
                // Start periodic advertising
                status = llm_adv_per_adv_start(ext_act_id, per_act_id);
            }
        }
        else
        {
            if (llm_env.act_info[per_act_id].state == LLM_PER_ADV_PENDING)
            {
                cmd_cmp = true;
                llm_env.act_info[per_act_id].state = LLM_PER_ADV_RSVD;
                break;
            }

            // Check if periodic advertising activity is enabled
            if (llm_env.act_info[per_act_id].state == LLM_PER_ADV_EN)
            {
                struct hci_le_set_ext_adv_param_cmd *ext_param = (struct hci_le_set_ext_adv_param_cmd *) llm_env.act_info[ext_act_id].host_params;

                // If the associated extended advertiser points to the periodic advertising activity, stop
                if (!(ext_param->adv_evt_properties & (ADV_CON | ADV_SCAN)) && (llm_env.act_info[ext_act_id].state == LLM_ADV_EN))
                {
                    lld_adv_sync_info_update(ext_act_id, per_act_id, 0);
                }

                // Stop periodic advertising
                status = lld_per_adv_stop(per_act_id);

                if (status == CO_ERROR_NO_ERROR)
                {
                    llm_env.act_info[per_act_id].state = LLM_PER_ADV_STOPPING;
                }

                // Unregister the activity from bandwidth allocation system
                sch_plan_rem(&llm_env.act_info[per_act_id].plan_elt);
            }
            else
            {
                cmd_cmp = true;
            }
        }
    }
    while (0);

    // If periodic advertising is stopping, wait the indication from the driver before reporting the command complete event
#if (BT_53)
    if ((status != CO_ERROR_NO_ERROR) || (param->enable & PER_ADV_EN_BIT) || cmd_cmp)
#else // (BT_53)
    if ((status != CO_ERROR_NO_ERROR) || (param->enable != ADV_DIS) || cmd_cmp)
#endif // (BT_53)
    {
        // Send the command complete event
        llm_cmd_cmp_send(opcode, status);
    }

    return (KE_MSG_CONSUMED);
}

#if (HCI_TL_SUPPORT)
int hci_le_rd_max_adv_data_len_cmd_handler(void const *param, uint16_t opcode)
{
    struct hci_le_rd_max_adv_data_len_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_le_rd_max_adv_data_len_cmd_cmp_evt);

#if (BLE_ADV_LEGACY_ITF)
    // Check current interface version
    if (llm_env.adv_itf_version == LLM_ADV_ITF_LEGACY)
    {
        event->status = CO_ERROR_COMMAND_DISALLOWED;
    }
    else
#endif //(BLE_ADV_LEGACY_ITF)
    {
#if (BLE_ADV_LEGACY_ITF)
        // Use extended interface
        llm_env.adv_itf_version = LLM_ADV_ITF_EXTENDED;
#endif //(BLE_ADV_LEGACY_ITF)

        event->status = CO_ERROR_NO_ERROR;
    }

    event->max_adv_data_len = EM_BLE_ADVDATATXBUF_SIZE;

    // Send the command complete event
    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}

int hci_le_rd_nb_supp_adv_sets_cmd_handler(void const *param, uint16_t opcode)
{
    struct hci_le_rd_nb_supp_adv_sets_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_le_rd_nb_supp_adv_sets_cmd_cmp_evt);
    event->nb_supp_adv_sets = 0;

#if (BLE_ADV_LEGACY_ITF)
    // Check current interface version
    if (llm_env.adv_itf_version == LLM_ADV_ITF_LEGACY)
    {
        event->status = CO_ERROR_COMMAND_DISALLOWED;
    }
    else
#endif //(BLE_ADV_LEGACY_ITF)
    {
#if (BLE_ADV_LEGACY_ITF)
        // Use extended interface
        llm_env.adv_itf_version = LLM_ADV_ITF_EXTENDED;
#endif //(BLE_ADV_LEGACY_ITF)

        event->status = CO_ERROR_NO_ERROR;

        // Count number of remaining free activities
        for (uint8_t act_id = 0; act_id < BLE_ACTIVITY_MAX_CFG; act_id++)
        {
            switch (llm_env.act_info[act_id].state)
            {
            case LLM_FREE:
            case LLM_ADV_RSVD:
            case LLM_ADV_EN:
            case LLM_ADV_STOPPING:
            {
                event->nb_supp_adv_sets += 1;
            }
            break;
            default:
                break;
            }
        }

        // Divide the number by 2 because an advertising set may be connectable or support periodic advertising
        event->nb_supp_adv_sets /= 2;
    }

    // Send the command complete event
    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}
#endif // (HCI_TL_SUPPORT)

int hci_le_rmv_adv_set_cmd_handler(struct hci_le_rem_adv_set_cmd const *param, uint16_t opcode)
{
    uint8_t status;

    do
    {
        uint8_t act_id;
        struct hci_le_set_ext_adv_param_cmd *ext_param;

#if (BLE_ADV_LEGACY_ITF)
        // Check current interface version
        if (llm_env.adv_itf_version == LLM_ADV_ITF_LEGACY)
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        // Use extended interface
        llm_env.adv_itf_version = LLM_ADV_ITF_EXTENDED;
#endif //(BLE_ADV_LEGACY_ITF)

        // Look for the advertising activity with specified handle
        act_id = llm_adv_hdl_to_id(param->adv_hdl, &ext_param);

        // If advertising set does not exists, reject the command
        if (act_id == LLM_ACT_IDX_INVL)
        {
            status = CO_ERROR_UNKNOWN_ADVERTISING_ID;
            break;
        }

        // Check that advertising is not enabled
        if (llm_env.act_info[act_id].state != LLM_ADV_RSVD)
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        // Check if a periodic advertising is linked to this advertising set
        if (llm_env.act_info[act_id].info.adv.per_act_id < BLE_ACTIVITY_MAX_CFG)
        {
            // Check that periodic advertising is not enabled
            if (llm_env.act_info[llm_env.act_info[act_id].info.adv.per_act_id].state != LLM_PER_ADV_RSVD)
            {
                status = CO_ERROR_COMMAND_DISALLOWED;
                break;
            }
        }

        // Release advertising set
        llm_adv_set_release(act_id);

        status = CO_ERROR_NO_ERROR;

    }
    while (0);

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

int hci_le_clear_adv_sets_cmd_handler(void const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    do
    {
        uint8_t act_id;

#if (BLE_ADV_LEGACY_ITF)
        // Check current interface version
        if (llm_env.adv_itf_version == LLM_ADV_ITF_LEGACY)
            break;

        // Use extended interface
        llm_env.adv_itf_version = LLM_ADV_ITF_EXTENDED;
#endif //(BLE_ADV_LEGACY_ITF)

        // Verify that there is no ongoing advertising set
        for (act_id = 0; act_id < BLE_ACTIVITY_MAX_CFG; act_id++)
        {
            // Check if activity is allocated to advertising that is ongoing
            if ((llm_env.act_info[act_id].state == LLM_ADV_EN) || (llm_env.act_info[act_id].state == LLM_ADV_STOPPING))
                break;

            // Check if activity is allocated to periodic advertising that is ongoing
            if ((llm_env.act_info[act_id].state == LLM_PER_ADV_EN) || (llm_env.act_info[act_id].state == LLM_PER_ADV_STOPPING))
                break;
        }

        // If advertising set does not exists, reject the command
        if (act_id < BLE_ACTIVITY_MAX_CFG)
            break;

        // Remove all advertising sets
        for (act_id = 0; act_id < BLE_ACTIVITY_MAX_CFG; act_id++)
        {
            // Check if activity is allocated to advertising
            if (llm_env.act_info[act_id].state == LLM_ADV_RSVD)
            {
                // Release advertising set
                llm_adv_set_release(act_id);
            }
        }

        status = CO_ERROR_NO_ERROR;

    }
    while (0);

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

#if BLE_CONLESS_CTE_TX
int hci_le_set_conless_cte_tx_param_cmd_handler(struct hci_le_set_conless_cte_tx_param_cmd const *param, uint16_t opcode)
{
    uint8_t ret_status = KE_MSG_CONSUMED;
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    do
    {
        uint8_t ext_act_id;

#if (BLE_ADV_LEGACY_ITF)
        // Check current interface version
        if (llm_env.adv_itf_version == LLM_ADV_ITF_LEGACY)
            break;
#endif //(BLE_ADV_LEGACY_ITF)

        // If advertising handle is not in the valid range
        if (param->adv_hdl > ADV_HDL_MAX)
        {
            status = CO_ERROR_INVALID_HCI_PARAM;
            break;
        }

        // Look for the advertising activity with specified handle
        ext_act_id = llm_adv_hdl_to_id(param->adv_hdl, NULL);

        // If the corresponding advertising set does not already exist, then the Controller shall return the error code Unknown Advertising Identifier (0x42)
        if (ext_act_id == LLM_ACT_IDX_INVL)
        {
            status = CO_ERROR_UNKNOWN_ADVERTISING_ID;
            break;
        }

        // Check if the parameters are valid
        if (((param->cte_len != NO_CTE) && (param->cte_len < CTE_LEN_MIN))
                || (param->cte_len > CTE_LEN_MAX)
                || (param->cte_type > CTE_TYPE_AOD_2US)
                || (param->cte_count < MIN_CTE_CNT)
                || (param->cte_count > MAX_CTE_CNT))
        {
            status = CO_ERROR_INVALID_HCI_PARAM;
            break;
        }

        // Only check the length of the switching pattern when the CTE type is AoD
        if ((param->cte_type == CTE_TYPE_AOD_1US) || (param->cte_type == CTE_TYPE_AOD_2US))
        {
            if ((param->switching_pattern_len < MIN_SWITCHING_PATTERN_LEN) || (param->switching_pattern_len > MAX_SWITCHING_PATTERN_LEN))
            {
                status = CO_ERROR_INVALID_HCI_PARAM;
                break;
            }

            if (param->switching_pattern_len > BLE_MAX_SW_PAT_LEN)
            {
                status = CO_ERROR_UNSUPPORTED;
                break;
            }
        }

        // If CTE_Length is not zero and PHY specifies a PHY that does not allow Constant Tone Extensions
        {
            // Point Host parameters
            struct hci_le_set_ext_adv_param_cmd *ext_param = (struct hci_le_set_ext_adv_param_cmd *) llm_env.act_info[ext_act_id].host_params;

            if ((param->cte_len != NO_CTE) && (ext_param->sec_adv_phy != PHY_1MBPS_VALUE) && (ext_param->sec_adv_phy != PHY_2MBPS_VALUE))
            {
                status = CO_ERROR_COMMAND_DISALLOWED;
                break;
            }
        }

        if (param->cte_count > BLE_ADV_FRAG_NB_TX)
        {
            status = CO_ERROR_UNSUPPORTED;
            break;
        }

        // If CTE has already been enabled
        if (llm_env.act_info[ext_act_id].info.adv.cte_en)
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        // Free potentially stored parameters
        if (llm_env.act_info[ext_act_id].info.adv.cte_tx_params != NULL)
        {
            ke_msg_free(ke_param2msg(llm_env.act_info[ext_act_id].info.adv.cte_tx_params));
            llm_env.act_info[ext_act_id].info.adv.cte_tx_params = NULL;
        }

        // Keep pointer to parameters
        llm_env.act_info[ext_act_id].info.adv.cte_tx_params = param;

        // Message needs to be maintained
        ret_status = KE_MSG_NO_FREE;

        status = CO_ERROR_NO_ERROR;
    }
    while (0);

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (ret_status);
}

int hci_le_set_conless_cte_tx_en_cmd_handler(struct hci_le_set_conless_cte_tx_en_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;

    do
    {
        uint8_t per_act_id, ext_act_id;

#if (BLE_ADV_LEGACY_ITF)
        // Check current interface version
        if (llm_env.adv_itf_version == LLM_ADV_ITF_LEGACY)
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        // Use extended interface
        llm_env.adv_itf_version = LLM_ADV_ITF_EXTENDED;
#endif //(BLE_ADV_LEGACY_ITF)

        // Check parameters
        if (param->cte_en > CTE_EN)
            break;

        // If advertising handle is not in the valid range
        if (param->adv_hdl > ADV_HDL_MAX)
            break;

        // Look for the advertising activity with specified handle
        ext_act_id = llm_adv_hdl_to_id(param->adv_hdl, NULL);

        // If the advertising set corresponding to the Advertising_Handle parameter does not exist, the Controller shall return the error code Unknown Advertising Identifier (0x42)
        if (ext_act_id == LLM_ACT_IDX_INVL)
        {
            status = CO_ERROR_UNKNOWN_ADVERTISING_ID;
            break;
        }

        // If CTE Tx is enabled and no parameters have been set
        if (param->cte_en && (llm_env.act_info[ext_act_id].info.adv.cte_tx_params == NULL))
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        // Find the associated periodic advertising activity
        per_act_id = llm_env.act_info[ext_act_id].info.adv.per_act_id;

        // If periodic advertising parameters have not been set
        if (per_act_id == LLM_ACT_IDX_INVL)
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        // Check if periodic advertising activity is enabled
        if (llm_env.act_info[per_act_id].state == LLM_PER_ADV_EN)
        {
            // If CTE has not yet been enabled
            if (!llm_env.act_info[ext_act_id].info.adv.cte_en && param->cte_en)
            {
                struct lld_per_adv_cte_params per_adv_cte_par;
                struct hci_le_set_conless_cte_tx_param_cmd const *cte_param = llm_env.act_info[ext_act_id].info.adv.cte_tx_params;

                per_adv_cte_par.cte_len = cte_param->cte_len;
                per_adv_cte_par.cte_type = cte_param->cte_type;
                per_adv_cte_par.cte_count = cte_param->cte_count;
                per_adv_cte_par.switching_pattern_len = cte_param->switching_pattern_len;

                // Only copy the antenna pattern when the CTE type is AoD
                if ((per_adv_cte_par.cte_type == CTE_TYPE_AOD_1US) || (per_adv_cte_par.cte_type == CTE_TYPE_AOD_2US))
                {
                    memcpy(&per_adv_cte_par.antenna_id[0], &cte_param->antenna_id[0], cte_param->switching_pattern_len);
                }

                // Start CTE on periodic advertising
                lld_per_adv_cte_start(per_act_id, &per_adv_cte_par);
            }
            // If CTE has already been enabled
            else if (llm_env.act_info[ext_act_id].info.adv.cte_en && !param->cte_en)
            {
                // Stop CTE on periodic advertising
                lld_per_adv_cte_stop(per_act_id);
            }
        }

        // Store CTE enable/disable
        llm_env.act_info[ext_act_id].info.adv.cte_en = param->cte_en;

        status = CO_ERROR_NO_ERROR;

    }
    while (0);

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}
#endif // BLE_CONLESS_CTE_TX

#if (BT_53)
int hci_le_set_data_related_addr_chg_cmd_handler(struct hci_le_set_data_related_addr_chg_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;

    do
    {
        uint8_t act_id = LLM_ACT_IDX_INVL;

        // Check parameters
        if (param->chg_reasons > (RPA_ADV_DATA_CHG | RPA_SCAN_RSP_DATA_CHG))
            break;

#if (BLE_ADV_LEGACY_ITF)
        // Check current interface version
        if (llm_env.adv_itf_version == LLM_ADV_ITF_LEGACY)
        {
            // Look for the legacy advertising activity
            act_id = llm_adv_hdl_to_id(LLM_ADV_HDL_INVL, NULL);
        }
        else if (llm_env.adv_itf_version == LLM_ADV_ITF_EXTENDED)
#endif //(BLE_ADV_LEGACY_ITF)
        {
            // If advertising handle is not in the valid range
            if (param->adv_hdl > ADV_HDL_MAX)
                break;

            // Look for the advertising activity with specified handle
            act_id = llm_adv_hdl_to_id(param->adv_hdl, NULL);
        }

        // If the advertising set corresponding to the Advertising_Handle parameter does not exist, the Controller shall return the error code Unknown Advertising Identifier (0x42)
        if (act_id == LLM_ACT_IDX_INVL)
        {
            status = CO_ERROR_UNKNOWN_ADVERTISING_ID;
            break;
        }

        // Store change reasons
        llm_env.act_info[act_id].info.adv.chg_reasons = param->chg_reasons;

        // If advertising is enabled
        if (llm_env.act_info[act_id].state != LLM_ADV_RSVD)
        {
            lld_adv_chg_reasons_update(act_id, param->chg_reasons);
        }

        status = CO_ERROR_NO_ERROR;

    }
    while (0);

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}
#endif // BLE_CONLESS_CTE_TX



/*
 * LLD INDICATION HANDLERS
 ****************************************************************************************
 */

KE_MSG_HANDLER_NO_STATIC(lld_scan_req_ind, struct lld_scan_req_ind)
{
    uint8_t act_id = param->act_id;
    struct hci_le_set_ext_adv_param_cmd *ext_param = (struct hci_le_set_ext_adv_param_cmd *) llm_env.act_info[act_id].host_params;

    ASSERT_INFO((llm_env.act_info[act_id].state == LLM_ADV_STOPPING) || (llm_env.act_info[act_id].state == LLM_ADV_EN), act_id, llm_env.act_info[act_id].state);

    if (ext_param->scan_req_notif_en)
    {
        struct hci_le_scan_req_rcvd_evt *event = KE_MSG_ALLOC(HCI_LE_EVENT, 0, HCI_LE_META_EVT_CODE, hci_le_scan_req_rcvd_evt);
        event->subcode = HCI_LE_SCAN_REQ_RCVD_EVT_SUBCODE;
        event->adv_hdl = ext_param->adv_hdl;
        event->scan_addr_type = param->addr_type;
        event->scan_addr = param->addr;
        hci_send_2_host(event);
    }

    return (KE_MSG_CONSUMED);
}

KE_MSG_HANDLER_NO_STATIC(lld_adv_end_ind, struct lld_adv_end_ind)
{
    uint8_t act_id = param->act_id;
#if (BLE_PERIPHERAL)
    bool connected = false;
    uint8_t con_act_id = llm_env.act_info[act_id].info.adv.con_act_id;
#endif //(BLE_PERIPHERAL)
    struct hci_le_set_ext_adv_param_cmd *ext_param = (struct hci_le_set_ext_adv_param_cmd *) llm_env.act_info[act_id].host_params;

    // Check if Host requested to stop
    if (llm_env.act_info[act_id].state == LLM_ADV_STOPPING)
    {
#if (BLE_ADV_LEGACY_ITF)
        if (llm_env.adv_itf_version == LLM_ADV_ITF_LEGACY) // Legacy advertising commands
        {
            // Report the completion of the command
            llm_cmd_cmp_send(HCI_LE_SET_ADV_EN_CMD_OPCODE, CO_ERROR_NO_ERROR);
        }
        else // Extended advertising commands
#endif //(BLE_ADV_LEGACY_ITF)
        {
            ASSERT_ERR(llm_env.nb_ext_adv_sets_stopping > 0);

            llm_env.nb_ext_adv_sets_stopping--;

            if (llm_env.nb_ext_adv_sets_stopping == 0)
            {
                // Report the completion of the command
                llm_cmd_cmp_send(HCI_LE_SET_EXT_ADV_EN_CMD_OPCODE, CO_ERROR_NO_ERROR);
            }
        }

        // Indicate advertising is disabled
        llm_env.act_info[act_id].state = LLM_ADV_RSVD;
#if (BLE_PERIPHERAL)
        // If an activity was allocated for a connection
        if ((ext_param->adv_evt_properties & ADV_CON))
        {
            // Release the associated activity identifier
            llm_env.act_info[con_act_id].state = LLM_FREE;

            // Clear the connection id of the advertising set
            llm_env.act_info[act_id].info.adv.con_act_id = LLM_ACT_IDX_INVL;
        }
#endif //(BLE_PERIPHERAL)
    }
    else if ((param->status == CO_ERROR_ADV_TO) || (param->status == CO_ERROR_LIMIT_REACHED))
    {
        // Indicate advertising is removed
        llm_env.act_info[act_id].state = LLM_ADV_RSVD;
#if (BLE_PERIPHERAL)
        // If an activity was allocated for a connection
        if ((ext_param->adv_evt_properties & ADV_CON))
        {
            // Release the associated activity identifier
            llm_env.act_info[con_act_id].state = LLM_FREE;

            // Clear the connection id of the advertising set
            llm_env.act_info[act_id].info.adv.con_act_id = LLM_ACT_IDX_INVL;
        }
#endif //(BLE_PERIPHERAL)
    }
#if (BLE_PERIPHERAL)
    // Or if a connection has been created
    else if (param->connected)
    {
        // Check if device is already connected
        if (llm_is_dev_connected(&param->peer_id_addr, param->peer_addr_type))
        {
            struct lld_adv_params adv_par;
            uint8_t status;

            // Should not happen for directed advertising
            ASSERT_ERR((ext_param->adv_evt_properties & (ADV_DIRECT | ADV_DIRECT_HI)) == 0);

            // Stop connection
            lld_con_stop(llm_env.act_info[act_id].info.adv.con_act_id, true);

            /*
             * Ignore the incoming connection and restart advertising
             */

            // Fill advertising parameters
            adv_par.adv_evt_properties  = ext_param->adv_evt_properties;
            adv_par.prim_adv_intv       = (ext_param->prim_adv_intv_max[2] << 16) | (ext_param->prim_adv_intv_max[1] << 8) | ext_param->prim_adv_intv_max[0];
            adv_par.prim_adv_ch_map     = ext_param->prim_adv_ch_map;
            adv_par.own_addr_type       = ext_param->own_addr_type;
            adv_par.peer_addr_type      = ext_param->peer_addr_type;
            adv_par.peer_addr           = ext_param->peer_addr;
            adv_par.adv_filter_policy   = (ext_param->adv_evt_properties & ADV_DIRECT) ? ADV_ALLOW_SCAN_ANY_CON_ANY : ext_param->adv_filt_policy;
            adv_par.adv_tx_pwr          = ext_param->adv_tx_pwr;
            adv_par.prim_adv_phy        = ext_param->prim_adv_phy;
            adv_par.sec_adv_max_skip    = ext_param->sec_adv_max_skip;
            adv_par.sec_adv_phy         = ext_param->prim_adv_phy;
            adv_par.adv_sid             = ext_param->adv_sid;
            adv_par.scan_req_notif_en   = ext_param->scan_req_notif_en;
            adv_par.own_addr            = llm_env.act_info[act_id].info.adv.bd_addr;
            adv_par.init_adv_data_buf   = llm_env.act_info[act_id].info.adv.adv_data.curr_buf;
            adv_par.init_adv_data_len   = llm_env.act_info[act_id].info.adv.adv_data.curr_len;
            adv_par.init_scan_rsp_data_buf = llm_env.act_info[act_id].info.adv.scan_rsp_data.curr_buf;
            adv_par.init_scan_rsp_data_len = llm_env.act_info[act_id].info.adv.scan_rsp_data.curr_len;
            adv_par.duration            = 0;
            adv_par.max_ext_adv_evt     = 0;
            adv_par.addr_resolution_en  = llm_env.addr_resolution_en;
            adv_par.ral_idx             = BLE_RAL_MAX;
            adv_par.con_act_id          = llm_env.act_info[act_id].info.adv.con_act_id;

#if (EAVESDROPPING_SUPPORT)
            if (llm_env.act_info[act_id].info.adv.manual_adv_params != NULL)
            {
                // Copy the manual adv parameters
                adv_par.manual_adv_params.en = llm_env.act_info[act_id].info.adv.manual_adv_params->enable;
                adv_par.manual_adv_params.time_offset = llm_env.act_info[act_id].info.adv.manual_adv_params->time_offset;
                adv_par.manual_adv_params.max_rand_offset = llm_env.act_info[act_id].info.adv.manual_adv_params->max_rand_offset;
            }
            else
            {
                adv_par.manual_adv_params.en = false;
            }

            if (llm_env.act_info[act_id].info.adv.cust_adv_params != NULL)
            {
                // Copy the custom adv parameters
                adv_par.cust_adv_params.periodicity = llm_env.act_info[act_id].info.adv.cust_adv_params->periodicity;
                adv_par.cust_adv_params.en = llm_env.act_info[act_id].info.adv.cust_adv_params->enable;
                adv_par.cust_adv_params.rand_en = llm_env.act_info[act_id].info.adv.cust_adv_params->rand_en;
                adv_par.cust_adv_params.time_offset = llm_env.act_info[act_id].info.adv.cust_adv_params->time_offset;
                adv_par.cust_adv_params.us_offset = llm_env.act_info[act_id].info.adv.cust_adv_params->us_offset;
                memcpy(&adv_par.cust_adv_params.bd_addr.addr[0], &llm_env.act_info[act_id].info.adv.cust_adv_params->bd_addr.addr[0], BD_ADDR_LEN);
                adv_par.cust_adv_params.group_ext_adv_ind = (!(ext_param->adv_evt_properties & ADV_LEGACY)) ? llm_env.act_info[act_id].info.adv.cust_adv_params->group_ext_adv_ind : false;
            }
            else
            {
                adv_par.cust_adv_params.en = false;
            }
#endif // EAVESDROPPING_SUPPORT

            // Check if peer device has a corresponding resolving list entry
            if ((adv_par.addr_resolution_en && (ext_param->adv_evt_properties & ADV_DIRECT))
                    || (ext_param->own_addr_type & ADDR_RPA_MASK))
            {
                // Find peer device in RAL
                adv_par.ral_idx = llm_ral_search(&ext_param->peer_addr, ext_param->peer_addr_type);
            }

#if (BT_53)
            adv_par.chg_reasons = llm_env.act_info[act_id].info.adv.chg_reasons;
            adv_par.adv_rpa_upd = llm_env.act_info[act_id].info.adv.adv_rpa_upd;
#endif // (BT_53)

            // Start advertising
            status = lld_adv_start(act_id, &adv_par);

            if (status == CO_ERROR_NO_ERROR)
            {
                llm_env.act_info[act_id].state = LLM_ADV_EN;
            }
            else
            {
                ASSERT_ERR_FORCE(0);
            }
        }
        else
        {
            uint8_t position;

            // Start LLC for connection establishment
            struct llc_init_parameters llc_par;

            llc_par.interval = param->interval;
            llc_par.latency = param->latency;
            llc_par.timeout = param->timeout;
            llc_par.ch_map = param->ch_map;
            llc_par.master_sca = param->master_sca;
            llc_par.role = SLAVE_ROLE;
            llc_par.rate = param->rate;
            llc_par.suggested_max_tx_octets = llm_env.suggested_max_tx_octets;
            llc_par.suggested_max_tx_time   = llm_env.suggested_max_tx_time;
#if (EAVESDROPPING_SUPPORT)
            llc_par.suggested_max_rx_octets = llm_env.suggested_max_rx_octets;
            llc_par.suggested_max_rx_time   = llm_env.suggested_max_rx_time;
#endif //(EAVESDROPPING_SUPPORT)
#if (BT_53 && BLE_PERIPHERAL)
            llc_par.ch_class_rep_intv       = llm_env.ch_class_rep_intv;
#endif // (BT_53 && BLE_PERIPHERAL)
            llc_par.phy_opt                 = llm_env.phy_opt;
            llc_par.tx_phys                 = llm_env.tx_phys;
            llc_par.rx_phys                 = llm_env.rx_phys;
#if (BLE_PAST)
            llc_par.past_mode               = llm_env.past_mode;
            llc_par.past_skip               = llm_env.past_skip;
            llc_par.past_sync_to            = llm_env.past_sync_to;
            llc_par.past_cte_type           = llm_env.past_cte_type;
#endif // (BLE_PAST)

            llc_start(con_act_id, &llc_par);

            // Update link identifier
            ASSERT_INFO(llm_env.act_info[con_act_id].state == LLM_CON_RSVD, con_act_id, llm_env.act_info[con_act_id].state);
            llm_env.act_info[con_act_id].info.con.bd_addr   = param->peer_id_addr;
            llm_env.act_info[con_act_id].info.con.addr_type = param->peer_addr_type;
            llm_env.act_info[con_act_id].state              = LLM_CONNECTED;
            llm_env.act_info[con_act_id].info.con.role      = ROLE_SLAVE;

            {
                struct sch_plan_elt_tag *new_plan_elt = &llm_env.act_info[con_act_id].plan_elt;

                // Calculate sync window in half-us
                uint32_t rx_win_size = 2 * ((1 + param->latency) * 4 * param->interval * (rwip_max_drift_get() + co_sca2ppm[param->master_sca]) / 1600); // half-slots * ppm * 625 half-us / 1000000;

                new_plan_elt->conhdl = con_act_id;
                new_plan_elt->conhdl_ref = new_plan_elt->conhdl;
                new_plan_elt->interval = 4 * param->interval;
                new_plan_elt->duration_min = 4;
                new_plan_elt->duration_max = new_plan_elt->duration_min;
                new_plan_elt->margin = 1 + ((rx_win_size + HALF_SLOT_SIZE / 2) / HALF_SLOT_SIZE);

                // Initialize with an invalid offset, the initial offset is detected at connection establishment
                new_plan_elt->offset = 4 * param->interval;

                // If connection can autonomously move
                if (llm_env.con_move_en)
                {
                    new_plan_elt->cb_move = &llc_con_move_cbk;
                    new_plan_elt->mobility = SCH_PLAN_MB_LVL_1;
                }
                else
                {
                    new_plan_elt->cb_move = NULL;
                    new_plan_elt->mobility = SCH_PLAN_MB_LVL_0;
                }
            }

            // Check if the device is in the device list
            position = llm_dev_list_search(&param->peer_id_addr, param->peer_addr_type);

            // Remove device from EM white list, if it is in white list
            if ((position < BLE_WHITELIST_MAX) && GETB(llm_env.dev_list[position].status, LLM_DEV_IN_WL))
            {
                lld_white_list_rem(position, (struct bd_addr *) &param->peer_id_addr, param->peer_addr_type);
            }

            // Register linkID / BD address at HCI level
            hci_ble_conhdl_register(con_act_id);

            // Indicate advertising activity ID as reserved
            llm_env.act_info[act_id].state = LLM_ADV_RSVD;

            // Clear the connection id of the advertising set
            llm_env.act_info[act_id].info.adv.con_act_id = LLM_ACT_IDX_INVL;

            connected = true;
        }
    }
#endif //(BLE_PERIPHERAL)
    else
    {
        ASSERT_INFO_FORCE(0, llm_env.act_info[act_id].state, param->status);
    }

#if (BLE_PERIPHERAL)
    // Report the connection creation or failure
    if (connected || ((ext_param->adv_evt_properties & ADV_DIRECT_HI) && (param->status == CO_ERROR_ADV_TO)))
    {
        // Report HCI_LE_Connection_Complete or HCI_LE_Enhanced_Connection_Complete event
        if (llm_le_evt_mask_check(LE_EVT_MASK_ENH_CON_CMP_EVT_BIT))
        {
            struct hci_le_enh_con_cmp_evt *evt = KE_MSG_ALLOC(HCI_LE_EVENT, 0, HCI_LE_META_EVT_CODE, hci_le_enh_con_cmp_evt);
            memset(evt, 0, sizeof(struct hci_le_enh_con_cmp_evt));
            evt->subcode = HCI_LE_ENH_CON_CMP_EVT_SUBCODE;
            evt->status = (connected) ? CO_ERROR_NO_ERROR : CO_ERROR_ADV_TO;
            evt->role = SLAVE_ROLE;
            evt->conhdl = BLE_LINKID_TO_CONHDL(con_act_id);
            if (connected)
            {
                memcpy(&evt->peer_addr.addr[0], &param->peer_id_addr.addr[0], BD_ADDR_LEN);
                evt->peer_addr_type = param->peer_addr_type;

                if (ext_param->own_addr_type & ADDR_RPA_MASK)
                {
                    memcpy(&evt->loc_rslv_priv_addr.addr[0], &param->local_rpa.addr[0], BD_ADDR_LEN);
                }
                else
                {
                    memset(&evt->loc_rslv_priv_addr.addr[0], 0, BD_ADDR_LEN);
                }
                if (param->peer_addr_type & ADDR_RPA_MASK)
                {
                    memcpy(&evt->peer_rslv_priv_addr.addr[0], &param->peer_rpa.addr[0], BD_ADDR_LEN);
                }
                else
                {
                    memset(&evt->peer_rslv_priv_addr.addr[0], 0, BD_ADDR_LEN);
                }
                evt->con_interval   = param->interval;
                evt->con_latency    = param->latency;
                evt->sup_to         = param->timeout;
                evt->clk_accuracy   = param->master_sca;
            }
            else // High duty cycle directed advertising timeout
            {
                evt->peer_addr_type = ext_param->peer_addr_type;
                evt->peer_addr = ext_param->peer_addr;
            }
            hci_send_2_host(evt);
        }
        else
        {
            struct hci_le_con_cmp_evt *evt = KE_MSG_ALLOC(HCI_LE_EVENT, 0, HCI_LE_META_EVT_CODE, hci_le_con_cmp_evt);
            evt->subcode = HCI_LE_CON_CMP_EVT_SUBCODE;
            evt->status = (connected) ? CO_ERROR_NO_ERROR : CO_ERROR_ADV_TO;
            if (connected)
            {
                evt->role   = SLAVE_ROLE;
                evt->conhdl = BLE_LINKID_TO_CONHDL(con_act_id);
                if (param->peer_addr_type & ADDR_RPA_MASK)
                {
                    evt->peer_addr_type = ADDR_RAND;
                    memcpy(&evt->peer_addr.addr[0], &param->peer_rpa.addr[0], BD_ADDR_LEN);
                }
                else
                {
                    evt->peer_addr_type = param->peer_addr_type & ADDR_MASK;
                    memcpy(&evt->peer_addr.addr[0], &param->peer_id_addr.addr[0], BD_ADDR_LEN);
                }
                evt->con_interval   = param->interval;
                evt->con_latency    = param->latency;
                evt->sup_to         = param->timeout;
                evt->clk_accuracy   = param->master_sca;
            }
            hci_send_2_host(evt);
        }

        // Send the LE Channel Selection Algorithm event
        if (connected)
        {
            struct hci_le_ch_sel_algo_evt *event = KE_MSG_ALLOC(HCI_LE_EVENT, 0, HCI_LE_META_EVT_CODE, hci_le_ch_sel_algo_evt);
            event->subcode = HCI_LE_CH_SEL_ALGO_EVT_SUBCODE;
            event->conhdl = BLE_LINKID_TO_CONHDL(con_act_id);
            event->ch_sel_algo = param->ch_sel_2;
            hci_send_2_host(event);
        }
    }
#endif // (BLE_PERIPHERAL)

    // Send the LE Advertising Set Terminated Event if necessary
#if (BLE_ADV_LEGACY_ITF)
    if (llm_env.adv_itf_version == LLM_ADV_ITF_EXTENDED)
#endif //(BLE_ADV_LEGACY_ITF)
    {
        if ((param->status != CO_ERROR_NO_ERROR)
#if (BLE_PERIPHERAL)
                || (connected)
#endif // (BLE_PERIPHERAL)
           )
        {
            struct hci_le_adv_set_term_evt *event = KE_MSG_ALLOC(HCI_LE_EVENT, 0, HCI_LE_META_EVT_CODE, hci_le_adv_set_term_evt);
            event->subcode = HCI_LE_ADV_SET_TERMINATED_EVT_SUBCODE;
            event->status = param->status;
            event->adv_hdl = ext_param->adv_hdl;
#if (BLE_PERIPHERAL)
            event->conhdl = BLE_LINKID_TO_CONHDL(con_act_id);
#else // (BLE_PERIPHERAL)
            event->conhdl = 0;
#endif // (BLE_PERIPHERAL)
            event->nb_cmp_ext_adv_evt = param->nb_ext_adv_evts;
            hci_send_2_host(event);
        }
    }

    return (KE_MSG_CONSUMED);
}

KE_MSG_HANDLER_NO_STATIC(lld_per_adv_end_ind, struct lld_per_adv_end_ind)
{
    if (llm_env.act_info[param->act_id].state == LLM_PER_ADV_STOPPING)
    {
        // Report the completion of the command
        llm_cmd_cmp_send(HCI_LE_SET_PER_ADV_EN_CMD_OPCODE, CO_ERROR_NO_ERROR);

        // Indicate periodic advertising is disabled
        llm_env.act_info[param->act_id].state = LLM_PER_ADV_RSVD;
    }

    return (KE_MSG_CONSUMED);
}

/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

uint8_t llm_adv_act_id_get(uint8_t adv_hdl, uint8_t *ext_adv_id, uint8_t *per_adv_id)
{
    uint8_t status = CO_ERROR_UNKNOWN_ADVERTISING_ID;

    do
    {
        uint8_t per_act_id, ext_act_id;

#if (BLE_ADV_LEGACY_ITF)
        // Check current interface version
        if (llm_env.adv_itf_version == LLM_ADV_ITF_LEGACY)
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        // Use extended interface
        llm_env.adv_itf_version = LLM_ADV_ITF_EXTENDED;
#endif //(BLE_ADV_LEGACY_ITF)

        // If advertising handle is not in the valid range
        if (adv_hdl > ADV_HDL_MAX)
            break;

        // Look for the advertising activity with specified handle
        ext_act_id = llm_adv_hdl_to_id(adv_hdl, NULL);

        // If the advertising set corresponding to the Advertising_Handle parameter does not exist, the Controller shall return the error code Unknown Advertising Identifier (0x42)
        if (ext_act_id == LLM_ACT_IDX_INVL)
            break;

        per_act_id = llm_env.act_info[ext_act_id].info.adv.per_act_id;
        // If the periodic advertising ID is invalid or the periodic advertiser is not active
        if ((per_act_id == LLM_ACT_IDX_INVL) || (llm_env.act_info[per_act_id].state != LLM_PER_ADV_EN))
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        status = CO_ERROR_NO_ERROR;
        *ext_adv_id = ext_act_id;
        *per_adv_id = per_act_id;
    }
    while (0);

    return (status);
}

uint8_t llm_adv_set_id_get(uint8_t act_id, uint8_t *sid, uint8_t *atype, struct bd_addr *adva)
{
    uint8_t status = CO_ERROR_UNKNOWN_ADVERTISING_ID;

    if ((llm_env.act_info[act_id].state == LLM_ADV_RSVD)
            || (llm_env.act_info[act_id].state == LLM_ADV_EN)
            || (llm_env.act_info[act_id].state == LLM_ADV_STOPPING))
    {
        struct hci_le_set_ext_adv_param_cmd *ext_param = (struct hci_le_set_ext_adv_param_cmd *) llm_env.act_info[act_id].host_params;

        // Check if own address is RPA
        bool rpa = memcmp(&llm_env.act_info[act_id].info.adv.local_rpa, &co_null_bdaddr, BD_ADDR_LEN);

        if (sid != NULL)
            *sid = ext_param->adv_sid;

        if (atype != NULL)
            *atype = rpa ? ADDR_RAND : ext_param->own_addr_type & ADDR_MASK;

        if (adva != NULL)
            *adva = rpa ? llm_env.act_info[act_id].info.adv.local_rpa : llm_env.act_info[act_id].info.adv.bd_addr;

        status = CO_ERROR_NO_ERROR;
    }

    return (status);
}

#if (BLE_BIS)
uint8_t llm_adv_per_id_get(uint8_t adv_hdl, uint8_t *ext_act_id, uint8_t *per_act_id)
{
    uint8_t status;

    do
    {
        struct hci_le_set_ext_adv_param_cmd *ext_param = NULL;

        // If advertising handle is not in the valid range
        if (adv_hdl > ADV_HDL_MAX)
        {
            status = CO_ERROR_INVALID_HCI_PARAM;
            break;
        }

        // Look for the advertising activity with specified handle
        *ext_act_id = llm_adv_hdl_to_id(adv_hdl, &ext_param);

        // If the advertising set corresponding to the Advertising_Handle parameter does not exist,
        // the Controller shall return the error code Unknown Advertising Identifier (0x42)
        if (*ext_act_id == LLM_ACT_IDX_INVL)
        {
            status = CO_ERROR_UNKNOWN_ADVERTISING_ID;
            break;
        }

        // Check that periodic advertising is configured
        *per_act_id = llm_env.act_info[*ext_act_id].info.adv.per_act_id;
        if ((*per_act_id == LLM_ACT_IDX_INVL)
                || ((llm_env.act_info[*per_act_id].state != LLM_PER_ADV_RSVD)
                    && (llm_env.act_info[*per_act_id].state != LLM_PER_ADV_EN)
                    && (llm_env.act_info[*per_act_id].state != LLM_PER_ADV_PENDING)
                    && (llm_env.act_info[*per_act_id].state != LLM_PER_ADV_STOPPING)))
        {
            status = CO_ERROR_UNKNOWN_ADVERTISING_ID;
            break;
        }

        // just a sanity check
        ASSERT_INFO(ext_param != NULL, adv_hdl, *ext_act_id);

        status = CO_ERROR_NO_ERROR;
    }
    while (0);

    return status;
}

uint8_t llm_adv_big_attach(uint8_t ext_act_id, uint8_t per_act_id, uint8_t big_act_id)
{
    uint8_t status;

    do
    {
        // Check that periodic advertising is configured
        if ((per_act_id == LLM_ACT_IDX_INVL)
                || ((llm_env.act_info[per_act_id].state != LLM_PER_ADV_RSVD)
                    && (llm_env.act_info[per_act_id].state != LLM_PER_ADV_EN)
                    && (llm_env.act_info[per_act_id].state != LLM_PER_ADV_PENDING)
                    && (llm_env.act_info[per_act_id].state != LLM_PER_ADV_STOPPING))
                || (llm_env.act_info[per_act_id].info.per_adv.ass_act_id != LLM_ACT_IDX_INVL))
        {
            status = CO_ERROR_UNKNOWN_ADVERTISING_ID;
            break;
        }

        // mark BIG attached to periodic advertiser
        llm_env.act_info[per_act_id].info.per_adv.ass_act_id = big_act_id;

        // Inform the periodic ADV driver that it should convey BIG info in ACAD data.
        if (llm_env.act_info[per_act_id].state == LLM_PER_ADV_EN)
        {
            // Update the planner interval element
            struct sch_plan_elt_tag *plan_elt = &llm_env.act_info[per_act_id].plan_elt;
            struct hci_le_set_ext_adv_param_cmd *ext_param = (struct hci_le_set_ext_adv_param_cmd *) llm_env.act_info[ext_act_id].host_params;
            uint16_t data_length = llm_env.act_info[per_act_id].info.per_adv.adv_data.curr_len + BLE_EXT_ACAD_BIG_INFO_ENC_LEN;

            ASSERT_ERR(plan_elt != NULL);

            // Unregister the old per adv from planner
            sch_plan_rem(plan_elt);

            // Update duration considering BIG info
            plan_elt->duration_min = llm_per_adv_chain_dur(data_length, ext_param->sec_adv_phy);
            plan_elt->duration_max = plan_elt->duration_min;

            // Register the new per adv into planner
            sch_plan_set(plan_elt);

            // Update driver
            lld_per_adv_big_update(per_act_id, big_act_id);
        }

        status = CO_ERROR_NO_ERROR;
    }
    while (0);

    return status;
}

uint8_t llm_adv_big_detach(uint8_t ext_act_id, uint8_t per_act_id, uint8_t big_act_id)
{
    uint8_t status = CO_ERROR_NO_ERROR;

    do
    {
        // Check that periodic advertising is configured
        if ((per_act_id == LLM_ACT_IDX_INVL)
                || ((llm_env.act_info[per_act_id].state != LLM_PER_ADV_RSVD)
                    && (llm_env.act_info[per_act_id].state != LLM_PER_ADV_EN)
                    && (llm_env.act_info[per_act_id].state != LLM_PER_ADV_STOPPING))
                || (llm_env.act_info[per_act_id].info.per_adv.ass_act_id != big_act_id))
        {
            status = CO_ERROR_UNKNOWN_ADVERTISING_ID;
            break;
        }

        // Inform the periodic ADV driver that it shall stop convey BIG Info in ACAD data.
        if (llm_env.act_info[per_act_id].state == LLM_PER_ADV_EN)
        {
            // Update the planner interval element
            struct sch_plan_elt_tag *plan_elt = &llm_env.act_info[per_act_id].plan_elt;
            struct hci_le_set_ext_adv_param_cmd *ext_param = (struct hci_le_set_ext_adv_param_cmd *) llm_env.act_info[ext_act_id].host_params;
            uint16_t data_length = llm_env.act_info[per_act_id].info.per_adv.adv_data.curr_len;

            ASSERT_ERR(plan_elt != NULL);

            // Unregister the old per adv from planner
            sch_plan_rem(plan_elt);

            // Update duration considering BIG info
            plan_elt->duration_min = llm_per_adv_chain_dur(data_length, ext_param->sec_adv_phy);
            plan_elt->duration_max = plan_elt->duration_min;

            // Register the new per adv into planner
            sch_plan_set(plan_elt);

            // Update driver
            lld_per_adv_big_update(per_act_id, LLM_ACT_IDX_INVL);
        }

        // mark BIG detached from periodic advertiser
        llm_env.act_info[per_act_id].info.per_adv.ass_act_id = LLM_ACT_IDX_INVL;

        status = CO_ERROR_NO_ERROR;
    }
    while (0);

    return status;
}
#endif // (BLE_BIS)

#endif // (BLE_BROADCASTER)

/// @} LLM
