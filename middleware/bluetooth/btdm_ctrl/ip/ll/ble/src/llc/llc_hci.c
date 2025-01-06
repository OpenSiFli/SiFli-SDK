/**
 ****************************************************************************************
 *
 * @file llc_hci.c
 *
 * @brief LLC host controller interface source file
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup LLCTASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"    // stack configuration
#if (BLE_PERIPHERAL || BLE_CENTRAL)
#include <string.h>
#include "co_utils.h"
#include "co_bt.h"          // BLE standard definitions

#include "lld.h"            // link layer driver definitions
#include "llc.h"            // link layer controller definitions
#include "llc_int.h"        // link layer controller internal definitions
#include "llm.h"            // link layer manager definitions

#if HCI_PRESENT
    #include "hci.h"            // host controller interface
#endif //HCI_PRESENT

#include "rwip.h"

/*
 * TYPES DEFINITION
 ****************************************************************************************
 */

/// Format of a HCI command handler function
typedef int (*llc_hci_cmd_hdl_func_t)(uint8_t link_id, void const *param, uint16_t opcode);


/*
 * STRUCT DEFINITION
 ****************************************************************************************
 */

/// Element of a HCI command handler table.
struct llc_hci_cmd_handler
{
    /// Command opcode
    uint16_t opcode;
    /// Pointer to the handler function for HCI command.
    llc_hci_cmd_hdl_func_t func;
};

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * LOCAL FUNCTIONS DEFINITION
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Send HCI_NB_CMP_PKTS_EVT to host to inform that TX buffer are available
 *
 * @param[in] link_id    Link identifier
 * @param[in] nb_of_pkt  Number of buffer available
 ****************************************************************************************
 */
__STATIC void llc_hci_nb_cmp_pkts_evt_send(uint8_t link_id, uint8_t nb_of_pkt)
{
    uint16_t conhdl = BLE_LINKID_TO_CONHDL(link_id);

    // allocates the message to send
    struct hci_nb_cmp_pkts_evt *event = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_NB_CMP_PKTS_EVT_CODE, hci_nb_cmp_pkts_evt);
    // gets the connection handle used
    event->con[0].hdl         = conhdl;
    // gets the number of packet sent
    event->con[0].nb_comp_pkt = nb_of_pkt;
    // processed handle by handle
    event->nb_of_hdl          = 1;
    // send the message
    hci_send_2_host(event);
}


/*
 * EXTERNAL FUNCTIONS DEFINITION
 ****************************************************************************************
 */
void llc_cmd_cmp_send(uint8_t link_id, uint16_t opcode, uint8_t status)
{
    uint16_t conhdl = BLE_LINKID_TO_CONHDL(link_id);

    // allocate the complete event message
    struct hci_basic_conhdl_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, conhdl, opcode, hci_basic_conhdl_cmd_cmp_evt);
    // update the status
    event->status = status;
    event->conhdl = conhdl;
    // send the message
    hci_send_2_host(event);
}

void llc_cmd_stat_send(uint8_t link_id, uint16_t opcode, uint8_t status)
{
    uint16_t conhdl = BLE_LINKID_TO_CONHDL(link_id);

    // allocate the status event message
    struct hci_cmd_stat_event *event = KE_MSG_ALLOC(HCI_CMD_STAT_EVENT, conhdl, opcode, hci_cmd_stat_event);
    // update the status
    event->status = status;
    // send the message
    hci_send_2_host(event);
}
/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */

#if (BLE_PERIPHERAL)
/// Handles the command HCI Vendor Specific Set Preferred Slave Latency
int hci_vs_set_pref_slave_latency_cmd_handler(uint8_t link_id, struct hci_vs_set_pref_slave_latency_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    // check if state is Free or in disconnected state
    if (!llc_is_disconnecting(link_id))
    {
        status = CO_ERROR_NO_ERROR;
        lld_con_pref_slave_latency_set(link_id, param->latency);
    }

    llc_cmd_cmp_send(link_id, opcode, status);

    return (KE_MSG_CONSUMED);
}

/// Handles the command HCI Vendor Specific Set Preferred Slave event duration
int hci_vs_set_pref_slave_evt_dur_cmd_handler(uint8_t link_id, struct hci_vs_set_pref_slave_evt_dur_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    // check if state is Free or in disconnected state
    if (!llc_is_disconnecting(link_id))
    {
        status = CO_ERROR_NO_ERROR;
        lld_con_pref_slave_evt_dur_set(link_id, param->duration, param->single_tx);
    }

    llc_cmd_cmp_send(link_id, opcode, status);

    return (KE_MSG_CONSUMED);
}
#endif // (BLE_PERIPHERAL)

/// Handles the command HCI Vendor Specific Set Priority Elevation
int hci_vs_set_prio_elevation_cmd_handler(uint8_t link_id, struct hci_vs_set_prio_elevation_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    // check if state is Free or in disconnected state
    if (!llc_is_disconnecting(link_id))
    {
        // Set the elevation for link
        status = lld_con_prio_elevation_set(link_id, param->elevation);
    }

    llc_cmd_cmp_send(link_id, opcode, status);

    return (KE_MSG_CONSUMED);
}

// Handler in llc_ver_exch.c
extern int hci_rd_rem_ver_info_cmd_handler(uint8_t link_id, struct hci_rd_rem_ver_info_cmd const *param, uint16_t opcode);

// Handler in llc_feats_exch.c
extern int hci_le_rd_rem_feats_cmd_handler(uint8_t link_id, struct hci_le_rd_rem_feats_cmd const *param, uint16_t opcode);

// Handler in llc_disconnect.c
extern int hci_disconnect_cmd_handler(uint8_t link_id, struct hci_disconnect_cmd const *param, uint16_t opcode);

// Handlers in llc_encrypt.c
extern int hci_le_en_enc_cmd_handler(uint8_t link_id, struct hci_le_en_enc_cmd const *param, uint16_t opcode);
extern int hci_le_ltk_req_reply_cmd_handler(uint8_t link_id, struct hci_le_ltk_req_reply_cmd const *param, uint16_t opcode);
extern int hci_le_ltk_req_neg_reply_cmd_handler(uint8_t link_id, struct hci_le_ltk_req_neg_reply_cmd const *param, uint16_t opcode);

// Handlers in llc_le_ping.c
extern int hci_rd_auth_payl_to_cmd_handler(uint8_t link_id, struct hci_rd_auth_payl_to_cmd const *param, uint16_t opcode);
extern int hci_wr_auth_payl_to_cmd_handler(uint8_t link_id, struct hci_wr_auth_payl_to_cmd const *param, uint16_t opcode);

// Handlers in llc_con_upd.c
extern int hci_le_con_upd_cmd_handler(uint8_t link_id, struct hci_le_con_update_cmd const *param, uint16_t opcode);
extern int hci_le_rem_con_param_req_reply_cmd_handler(uint8_t link_id, struct hci_le_rem_con_param_req_reply_cmd const *param, uint16_t opcode);
extern int hci_le_rem_con_param_req_neg_reply_cmd_handler(uint8_t link_id, struct hci_le_rem_con_param_req_neg_reply_cmd const *param, uint16_t opcode);

// Handlers in llc_ch_map_upd.c
extern int hci_le_rd_ch_map_cmd_handler(uint8_t link_id, struct hci_basic_conhdl_cmd const *param, uint16_t opcode);

// Handlers in llc_phy_upd.c
extern int hci_le_set_phy_cmd_handler(uint8_t link_id, struct hci_le_set_phy_cmd const *param, uint16_t opcode);
extern int hci_le_rd_phy_cmd_handler(uint8_t link_id, struct hci_le_rd_phy_cmd const *param, uint16_t opcode);

// Handlers in llc_dl_upd.c
extern int hci_le_set_data_len_cmd_handler(uint8_t link_id, struct hci_le_set_data_len_cmd const *param, uint16_t opcode);
#if (EAVESDROPPING_SUPPORT)
    extern int hci_vs_le_enh_set_data_len_cmd_handler(uint8_t link_id, struct hci_vs_le_enh_set_data_len_cmd const *param, uint16_t opcode);
#endif //(EAVESDROPPING_SUPPORT)
extern int hci_vs_set_max_rx_size_and_time_cmd_handler(uint8_t link_id, struct hci_vs_set_max_rx_size_and_time_cmd const *param, uint16_t opcode);

// Handlers in llc_cte.c
#if BLE_CON_CTE_REQ
    extern int hci_le_set_con_cte_rx_param_cmd_handler(uint8_t link_id, struct hci_le_set_con_cte_rx_param_cmd const *param, uint16_t opcode);
    extern int hci_le_con_cte_req_en_cmd_handler(uint8_t link_id, struct hci_le_con_cte_req_en_cmd const *param, uint16_t opcode);
#endif // BLE_CON_CTE_REQ
#if BLE_CON_CTE_RSP
    extern int hci_le_set_con_cte_tx_param_cmd_handler(uint8_t link_id, struct hci_le_set_con_cte_tx_param_cmd const *param, uint16_t opcode);
    extern int hci_le_con_cte_rsp_en_cmd_handler(uint8_t link_id, struct hci_le_con_cte_rsp_en_cmd const *param, uint16_t opcode);
#endif // BLE_CON_CTE_RSP

#if BLE_PAST
    // Handlers in llc_past.c
    extern int hci_le_per_adv_sync_transf_cmd_handler(uint8_t link_id, struct hci_le_per_adv_sync_transf_cmd const *param, uint16_t opcode);
    extern int hci_le_per_adv_set_info_transf_cmd_handler(uint8_t link_id, struct hci_le_per_adv_set_info_transf_cmd const *param, uint16_t opcode);
    extern int hci_le_set_per_adv_sync_transf_param_cmd_handler(uint8_t link_id, struct hci_le_set_per_adv_sync_transf_param_cmd const *param, uint16_t opcode);
#endif // BLE_PAST

#if (HCI_TL_SUPPORT)
    extern int hci_le_req_peer_sca_cmd_handler(uint8_t link_id, struct hci_le_req_peer_sca_cmd const *param, uint16_t opcode);
#endif // (HCI_TL_SUPPORT)

// Handlers in llc_pwr.c
extern int hci_rd_tx_pwr_lvl_cmd_handler(uint8_t link_id, struct hci_rd_tx_pwr_lvl_cmd const *param, uint16_t opcode);
extern int hci_rd_rssi_cmd_handler(uint8_t link_id, struct hci_basic_conhdl_cmd const *param, uint16_t opcode);
#if BLE_PWR_CTRL
    extern int hci_le_enh_rd_tx_pwr_lvl_cmd_handler(uint8_t link_id, struct hci_le_enh_rd_tx_pwr_lvl_cmd const *param, uint16_t opcode);
    extern int hci_le_rd_remote_tx_pwr_lvl_cmd_handler(uint8_t link_id, struct hci_le_rd_remote_tx_pwr_lvl_cmd const *param, uint16_t opcode);
    extern int hci_le_set_path_loss_rep_param_cmd_handler(uint8_t link_id, struct hci_le_set_path_loss_rep_param_cmd const *param, uint16_t opcode);
    extern int hci_le_set_path_loss_rep_en_cmd_handler(uint8_t link_id, struct hci_le_set_path_loss_rep_en_cmd const *param, uint16_t opcode);
    extern int hci_le_set_tx_power_rep_en_cmd_handler(uint8_t link_id, struct hci_le_set_tx_power_rep_en_cmd const *param, uint16_t opcode);
#endif // BLE_PWR_CTRL

#if (BT_53)
    extern int hci_le_subrate_req_cmd_handler(uint8_t link_id, struct hci_le_subrate_req_cmd const *param, uint16_t opcode);
#endif //(BT_53)

#if (RW_DEBUG)
    // Handler in llc_llcp.c
    extern int hci_dbg_send_llcp_cmd_handler(uint8_t link_id, struct hci_dbg_send_llcp_cmd const *param, uint16_t opcode);
    // Handler in llc_dbg.c
    extern int hci_dbg_llcp_discard_cmd_handler(uint8_t link_id, struct hci_dbg_llcp_discard_cmd const *param, uint16_t opcode);
#endif // (RW_DEBUG)

/**
 ***************************************************************************************
 * BLE 4.2 handlers
 ***************************************************************************************
*/


/**
 ***************************************************************************************
 * BLE 5.0 handlers
 ***************************************************************************************
*/

/*
 * HCI COMMAND HANDLING
 ****************************************************************************************
 */

/// The message handlers for HCI commands
HCI_CMD_HANDLER_TAB(llc)
{
    {HCI_DISCONNECT_CMD_OPCODE, (llc_hci_cmd_hdl_func_t)hci_disconnect_cmd_handler},
    {HCI_RD_REM_VER_INFO_CMD_OPCODE, (llc_hci_cmd_hdl_func_t)hci_rd_rem_ver_info_cmd_handler},
    {HCI_RD_AUTH_PAYL_TO_CMD_OPCODE, (llc_hci_cmd_hdl_func_t)hci_rd_auth_payl_to_cmd_handler},
    {HCI_WR_AUTH_PAYL_TO_CMD_OPCODE, (llc_hci_cmd_hdl_func_t)hci_wr_auth_payl_to_cmd_handler},
    {HCI_LE_CON_UPDATE_CMD_OPCODE, (llc_hci_cmd_hdl_func_t)hci_le_con_upd_cmd_handler},
    {HCI_LE_RD_CH_MAP_CMD_OPCODE, (llc_hci_cmd_hdl_func_t)hci_le_rd_ch_map_cmd_handler},
    {HCI_LE_RD_REM_FEATS_CMD_OPCODE, (llc_hci_cmd_hdl_func_t)hci_le_rd_rem_feats_cmd_handler},
    {HCI_LE_EN_ENC_CMD_OPCODE, (llc_hci_cmd_hdl_func_t)hci_le_en_enc_cmd_handler},
    {HCI_LE_LTK_REQ_REPLY_CMD_OPCODE, (llc_hci_cmd_hdl_func_t)hci_le_ltk_req_reply_cmd_handler},
    {HCI_LE_LTK_REQ_NEG_REPLY_CMD_OPCODE, (llc_hci_cmd_hdl_func_t)hci_le_ltk_req_neg_reply_cmd_handler},
    {HCI_LE_REM_CON_PARAM_REQ_REPLY_CMD_OPCODE, (llc_hci_cmd_hdl_func_t)hci_le_rem_con_param_req_reply_cmd_handler},
    {HCI_LE_REM_CON_PARAM_REQ_NEG_REPLY_CMD_OPCODE, (llc_hci_cmd_hdl_func_t)hci_le_rem_con_param_req_neg_reply_cmd_handler},
    {HCI_LE_SET_DATA_LEN_CMD_OPCODE, (llc_hci_cmd_hdl_func_t)hci_le_set_data_len_cmd_handler},
#if (EAVESDROPPING_SUPPORT)
    {HCI_VS_LE_ENH_SET_DATA_LEN_CMD_OPCODE, (llc_hci_cmd_hdl_func_t)hci_vs_le_enh_set_data_len_cmd_handler},
#endif // EAVESDROPPING_SUPPORT
    {HCI_LE_RD_PHY_CMD_OPCODE, (llc_hci_cmd_hdl_func_t)hci_le_rd_phy_cmd_handler},
    {HCI_LE_SET_PHY_CMD_OPCODE, (llc_hci_cmd_hdl_func_t)hci_le_set_phy_cmd_handler},
#if (BLE_PAST)
#if (BLE_OBSERVER)
    {HCI_LE_PER_ADV_SYNC_TRANSF_CMD_OPCODE, (llc_hci_cmd_hdl_func_t)hci_le_per_adv_sync_transf_cmd_handler},
#endif //(BLE_OBSERVER)
#if (BLE_BROADCASTER)
    {HCI_LE_PER_ADV_SET_INFO_TRANSF_CMD_OPCODE, (llc_hci_cmd_hdl_func_t)hci_le_per_adv_set_info_transf_cmd_handler},
#endif // (BLE_BROADCASTER)
    {HCI_LE_SET_PER_ADV_SYNC_TRANSF_PARAM_CMD_OPCODE, (llc_hci_cmd_hdl_func_t)hci_le_set_per_adv_sync_transf_param_cmd_handler},
#endif // (BLE_PAST)
#if (BLE_PERIPHERAL)
    {HCI_VS_SET_PREF_SLAVE_LATENCY_CMD_OPCODE, (llc_hci_cmd_hdl_func_t)hci_vs_set_pref_slave_latency_cmd_handler},
    {HCI_VS_SET_PREF_SLAVE_EVT_DUR_CMD_OPCODE, (llc_hci_cmd_hdl_func_t)hci_vs_set_pref_slave_evt_dur_cmd_handler},
#endif // (BLE_PERIPHERAL)
    {HCI_VS_SET_MAX_RX_SIZE_AND_TIME_CMD_OPCODE, (llc_hci_cmd_hdl_func_t)hci_vs_set_max_rx_size_and_time_cmd_handler},
    {HCI_VS_SET_PRIO_ELEVATION_CMD_OPCODE, (llc_hci_cmd_hdl_func_t)hci_vs_set_prio_elevation_cmd_handler},
#if (RW_DEBUG)
    {HCI_DBG_SEND_LLCP_CMD_OPCODE, (llc_hci_cmd_hdl_func_t)hci_dbg_send_llcp_cmd_handler},
    {HCI_DBG_LLCP_DISCARD_CMD_OPCODE, (llc_hci_cmd_hdl_func_t)hci_dbg_llcp_discard_cmd_handler},
#endif // (RW_DEBUG)

    {HCI_RD_TX_PWR_LVL_CMD_OPCODE, (llc_hci_cmd_hdl_func_t)hci_rd_tx_pwr_lvl_cmd_handler},
    {HCI_RD_RSSI_CMD_OPCODE, (llc_hci_cmd_hdl_func_t)hci_rd_rssi_cmd_handler},
#if BLE_CON_CTE_REQ
    {HCI_LE_SET_CON_CTE_RX_PARAM_CMD_OPCODE, (llc_hci_cmd_hdl_func_t)hci_le_set_con_cte_rx_param_cmd_handler},
    {HCI_LE_CON_CTE_REQ_EN_CMD_OPCODE, (llc_hci_cmd_hdl_func_t)hci_le_con_cte_req_en_cmd_handler},
#endif // BLE_CON_CTE_REQ
#if BLE_CON_CTE_RSP
    {HCI_LE_SET_CON_CTE_TX_PARAM_CMD_OPCODE, (llc_hci_cmd_hdl_func_t)hci_le_set_con_cte_tx_param_cmd_handler},
    {HCI_LE_CON_CTE_RSP_EN_CMD_OPCODE, (llc_hci_cmd_hdl_func_t)hci_le_con_cte_rsp_en_cmd_handler},
#endif // BLE_CON_CTE_RSP

#if (HCI_TL_SUPPORT)
    {HCI_LE_REQ_PEER_SCA_CMD_OPCODE, (llc_hci_cmd_hdl_func_t)hci_le_req_peer_sca_cmd_handler},
#endif // (HCI_TL_SUPPORT)

#if BLE_PWR_CTRL
    {HCI_LE_ENH_RD_TX_PWR_LVL_CMD_OPCODE, (llc_hci_cmd_hdl_func_t)hci_le_enh_rd_tx_pwr_lvl_cmd_handler},
    {HCI_LE_RD_REMOTE_TX_PWR_LVL_CMD_OPCODE, (llc_hci_cmd_hdl_func_t)hci_le_rd_remote_tx_pwr_lvl_cmd_handler},
    {HCI_LE_SET_PATH_LOSS_REP_PARAM_CMD_OPCODE, (llc_hci_cmd_hdl_func_t)hci_le_set_path_loss_rep_param_cmd_handler},
    {HCI_LE_SET_PATH_LOSS_REP_EN_CMD_OPCODE, (llc_hci_cmd_hdl_func_t)hci_le_set_path_loss_rep_en_cmd_handler},
    {HCI_LE_SET_TX_POWER_REP_EN_CMD_OPCODE, (llc_hci_cmd_hdl_func_t)hci_le_set_tx_power_rep_en_cmd_handler},
#endif // BLE_PWR_CTRL

#if (BT_53)
    {HCI_LE_SUBRATE_REQ_CMD_OPCODE, (llc_hci_cmd_hdl_func_t)hci_le_subrate_req_cmd_handler},
#endif // (BT_53)
};

/// Handles any HCI command
KE_MSG_HANDLER_NO_STATIC(hci_command_llc, uint16_t)
{
    int return_status = KE_MSG_CONSUMED;

    // Check if there is a handler corresponding to the original command opcode
    for (uint16_t i = 0; i < ARRAY_LEN(llc_hci_command_handler_tab); i++)
    {
        // Check if opcode matches
        if (llc_hci_command_handler_tab[i].opcode == src_id)
        {
            // Check if there is a handler function
            if (llc_hci_command_handler_tab[i].func != NULL)
            {
                uint8_t link_id = BLE_CONHDL_TO_LINKID(*param);
                // Call handler
                return_status = llc_hci_command_handler_tab[i].func(link_id, param, src_id);
            }
            break;
        }
    }

    return return_status;
}

///Handles the packet data to be sent.
int hci_acl_data_handler(ke_msg_id_t const msgid, struct hci_acl_data *param,
                         ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    DBG_FUNC_ENTER(hci_acl_data_handler);
    uint8_t link_id = KE_IDX_GET(dest_id);
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];

    // Check if state in disconnected state
    if (llc_is_disconnecting(link_id))
    {
        // nothing to do
    }
    // send packet only if data length greater than zero, else ignore it.
    else if (param->length > 0)
    {
        struct ble_em_acl_buf_elt *tx_elt = ble_util_buf_acl_tx_elt_get((uint16_t) param->buf_ptr);

        // Fill data length and flag fields
        tx_elt->data_len_flags = 0;
        SETF(tx_elt->data_len_flags, BLE_EM_ACL_DATA_LEN, param->length);
        SETF(tx_elt->data_len_flags, BLE_EM_ACL_BF, GETF(param->conhdl_pb_bc_flag, HCI_ACL_HDR_BC_FLAG));

        // If an L2CAP start fragment is pending, force packet boundary flag to "start"
        if (llc_env_ptr->l2cap_start)
        {
            SETF(tx_elt->data_len_flags, BLE_EM_ACL_PBF, PBF_1ST_NF_HL_FRAG);
            llc_env_ptr->l2cap_start = false;
        }
        else
        {
            SETF(tx_elt->data_len_flags, BLE_EM_ACL_PBF, GETF(param->conhdl_pb_bc_flag, HCI_ACL_HDR_PB_FLAG));
        }

        status = lld_con_data_tx(link_id, tx_elt);
    }
    else if (GETF(param->conhdl_pb_bc_flag, HCI_ACL_HDR_PB_FLAG) == PBF_1ST_NF_HL_FRAG)
    {
        // Indicate L2CAP start fragment is pending, and ignore HCI packet
        llc_env_ptr->l2cap_start = true;
    }

    // if an error occurs, drop the buffer
    if (status != CO_ERROR_NO_ERROR)
    {
        if (param->length > 0)
        {
            // Free buffer
            ble_util_buf_acl_tx_free(param->buf_ptr);
        }

        // inform host that buffer is available
        llc_hci_nb_cmp_pkts_evt_send(link_id, 1);
    }

    DBG_FUNC_EXIT(hci_acl_data_handler);

    return (KE_MSG_CONSUMED);
}

///Handles reception of data from peer device
int lld_acl_rx_ind_handler(ke_msg_id_t const msgid, struct lld_acl_rx_ind *param,
                           ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    DBG_FUNC_ENTER(lld_acl_rx_ind_handler);
    bool msg_sent = false;
    uint8_t link_id = KE_IDX_GET(dest_id);
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];

    // Check if state in disconnected state
    if (llc_is_disconnecting(link_id))
    {
        // message must be dropped
    }
    // check if RX Flow is Off
    else if (GETF(llc_env_ptr->llcp_state, LLC_LLCP_RX) != LLC_LLCP_IDLE)
    {
        // MIC Failure
        llc_disconnect(link_id, CO_ERROR_TERMINATED_MIC_FAILURE, true);
    }
    else
    {
        uint16_t conhdl = BLE_LINKID_TO_CONHDL(link_id);
        // allocate the status event message
        struct hci_acl_data *rx_data = KE_MSG_ALLOC(HCI_ACL_DATA, conhdl, 0, hci_acl_data);
        // set connection handle and flags
        rx_data->conhdl_pb_bc_flag = 0;
        SETF(rx_data->conhdl_pb_bc_flag, HCI_ACL_HDR_HDL,     conhdl);
        SETF(rx_data->conhdl_pb_bc_flag, HCI_ACL_HDR_BC_FLAG, BCF_P2P);
        SETF(rx_data->conhdl_pb_bc_flag, HCI_ACL_HDR_PB_FLAG, (param->llid == LLID_START) ? PBF_1ST_HL_FRAG : PBF_CONT_HL_FRAG);
        // fill data length
        rx_data->length  = param->data_len;
        // fill the data buffer pointer
        rx_data->buf_ptr = param->em_buf;
        // send the message
        hci_send_2_host(rx_data);

        // restart LE Ping timer
        llc_le_ping_restart(link_id);
        msg_sent = true;
    }

    // message dropped
    if (!msg_sent)
    {
        // Free buffer, no more needed
        ble_util_buf_rx_free(param->em_buf);
    }
    DBG_FUNC_EXIT(lld_acl_rx_ind_handler);

    return (KE_MSG_CONSUMED);
}

///Handles baseband acknowledgment of packet sent to peer device
int lld_acl_tx_cfm_handler(ke_msg_id_t const msgid, void *param,
                           ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    DBG_FUNC_ENTER(lld_acl_tx_cfm_handler);
    uint8_t link_id = KE_IDX_GET(dest_id);

    // Inform host that buffer is available
    llc_hci_nb_cmp_pkts_evt_send(link_id, 1);

    DBG_FUNC_EXIT(lld_acl_tx_cfm_handler);
    return (KE_MSG_CONSUMED);
}




#endif // (BLE_PERIPHERAL || BLE_CENTRAL)

/// @} LLCTASK
