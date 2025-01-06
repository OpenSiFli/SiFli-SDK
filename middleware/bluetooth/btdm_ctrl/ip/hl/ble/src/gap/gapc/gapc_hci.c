/**
 ****************************************************************************************
 *
 * @file gapc_hci.c
 *
 * @brief Generic Access Profile Controller HCI handler implementation.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup GAPC_HCI Generic Access Profile Controller HCI Hander
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"

#if (BLE_CENTRAL || BLE_PERIPHERAL)
#include <string.h>

#include "co_error.h"
#include "co_bt.h"


#include "hci.h"
#include "gap.h"
#include "gapc_int.h"

#include "gapm.h"

#include "../smpc/smpc_int.h" // Internal API required

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */



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

/**
 ****************************************************************************************
 * @brief Handles Disconnect command completed event
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPM).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_disc_cmp_evt_handler(ke_msg_id_t const msgid,
                                    struct hci_disc_cmp_evt const *cmp_evt,
                                    ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Current process state
    uint8_t state = ke_state_get(dest_id);
    // Current connection index
    uint8_t conidx = KE_IDX_GET(dest_id);

    if (state != GAPC_FREE)
    {
        // send a disconnection message
        gapc_send_disconect_ind(conidx, cmp_evt->reason, cmp_evt->conhdl,
                                (gapc_env[conidx]->disc_requester != TASK_NONE) ? gapc_env[conidx]->disc_requester
                                : gapc_get_dest_task(conidx));

        // send completed event only if it's a disconnect request
        if (gapc_env[conidx]->disc_requester != TASK_NONE)
        {
            /* send disconnect command complete event */
            gapc_send_error_evt(conidx, GAPC_DISCONNECT, gapc_env[conidx]->disc_requester,
                                RW_ERR_HCI_TO_HL(cmp_evt->status));
        }

        // cleanup Allocated connection resources
        gapm_con_cleanup(conidx, cmp_evt->conhdl, cmp_evt->reason);
    }
    /* message is consumed */
    return (KE_MSG_CONSUMED);
}



/**
 ****************************************************************************************
 * @brief Handles Read version command completed event
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPM).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_rd_rem_ver_info_cmp_evt_handler(ke_msg_id_t const msgid,
        struct hci_rd_rem_ver_info_cmp_evt const *cmp_evt,
        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Current process state
    uint8_t state = ke_state_get(dest_id);
    // Current connection index
    uint8_t conidx = KE_IDX_GET(dest_id);

    if (state != GAPC_FREE)
    {
        uint8_t status = GAP_ERR_PROTOCOL_PROBLEM;

        if (gapc_get_operation(conidx, GAPC_OP_LINK_INFO) == GAPC_GET_PEER_VERSION)
        {
            if ((cmp_evt->status == CO_ERROR_NO_ERROR))
            {
                status = GAP_ERR_NO_ERROR;
                // create version indication message.
                struct gapc_peer_version_ind *version_ind =
                    KE_MSG_ALLOC(GAPC_PEER_VERSION_IND, gapc_get_requester(conidx, GAPC_OP_LINK_INFO),
                                 KE_BUILD_ID(TASK_GAPC, conidx), gapc_peer_version_ind);

                // LMP version
                version_ind->lmp_vers    = cmp_evt->vers;
                // LMP subversion
                version_ind->lmp_subvers = cmp_evt->subvers;
                // Manufacturer name
                version_ind->compid      = cmp_evt->compid;
                // send indication
                ke_msg_send(version_ind);

                // Slave Connection parameters request feature only supported by device version >= 4.1
                if (version_ind->lmp_vers <= BT40_VERSION)
                {
                    // Clear parameter request feature in the environment because not supported by peer
                    gapc_env[conidx]->features &= ~(GAPC_CONN_PARAM_REQ_FEAT_MASK);
                }
            }
            else
            {
                status = RW_ERR_HCI_TO_HL(cmp_evt->status);
            }
        }
        /* send command complete event */
        gapc_send_complete_evt(conidx, GAPC_OP_LINK_INFO, status);

    }
    /* message is consumed */
    return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles Read version command completed event
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPM).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_le_rd_rem_feats_cmp_evt_handler(ke_msg_id_t const msgid,
        struct hci_le_rd_rem_feats_cmd_cmp_evt const *cmp_evt,
        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Current process state
    uint8_t state = ke_state_get(dest_id);
    // Current connection index
    uint8_t conidx = KE_IDX_GET(dest_id);

    if (state != GAPC_FREE)
    {
        uint8_t status = GAP_ERR_PROTOCOL_PROBLEM;

        if (gapc_get_operation(conidx, GAPC_OP_LINK_INFO) == GAPC_GET_PEER_FEATURES)
        {
            if ((cmp_evt->status == CO_ERROR_NO_ERROR))
            {
                status = GAP_ERR_NO_ERROR;
                // create feature indication message.
                struct gapc_peer_features_ind *features_ind =
                    KE_MSG_ALLOC(GAPC_PEER_FEATURES_IND, gapc_get_requester(conidx, GAPC_OP_LINK_INFO),
                                 KE_BUILD_ID(TASK_GAPC, conidx), gapc_peer_features_ind);

                //save 5 LSB bits relevant of peer feature in the environment
                gapc_env[conidx]->features = cmp_evt->le_feats.feats[0];

                // fill parameters
                memcpy(features_ind->features, cmp_evt->le_feats.feats, LE_FEATS_LEN);
                // send indication
                ke_msg_send(features_ind);
            }
            else
            {
                status = RW_ERR_HCI_TO_HL(cmp_evt->status);

#if (BLE_PERIPHERAL)
                if (GAPC_GET_FIELD(conidx, ROLE) == ROLE_SLAVE)
                {
                    switch (status)
                    {
                    case LL_ERR_UNKNOWN_HCI_COMMAND:
                    case LL_ERR_COMMAND_DISALLOWED:
                    case LL_ERR_UNSUPPORTED:
                    case LL_ERR_UNKNOWN_LMP_PDU:
                    case LL_ERR_UNSUPPORTED_REMOTE_FEATURE:
                    {
                        // Clear parameter request feature in the environment because not supported by peer
                        gapc_env[conidx]->features &= ~(GAPC_CONN_PARAM_REQ_FEAT_MASK);
                    }
                    break;
                    default: /* Nothing to do */
                        break;
                    }
                }
#endif // (BLE_PERIPHERAL)
            }
        }
        /* send command complete event */
        gapc_send_complete_evt(conidx, GAPC_OP_LINK_INFO, status);

    }
    /* message is consumed */
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles read RSSI command completed event
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPM).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_rd_rssi_cmd_cmp_evt_handler(ke_msg_id_t const msgid,
        struct hci_rd_rssi_cmd_cmp_evt const *cmp_evt,
        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Current process state
    uint8_t state = ke_state_get(dest_id);
    // Current connection index
    uint8_t conidx = KE_IDX_GET(dest_id);

    if (state != GAPC_FREE)
    {
        uint8_t status = GAP_ERR_PROTOCOL_PROBLEM;
        if (gapc_get_operation(conidx, GAPC_OP_LINK_INFO) == GAPC_GET_CON_RSSI)
        {
            if ((cmp_evt->status == CO_ERROR_NO_ERROR))
            {
                status = GAP_ERR_NO_ERROR;
                // create rssi indication message.
                struct gapc_con_rssi_ind *rssi_ind =
                    KE_MSG_ALLOC(GAPC_CON_RSSI_IND, gapc_get_requester(conidx, GAPC_OP_LINK_INFO),
                                 KE_BUILD_ID(TASK_GAPC, conidx), gapc_con_rssi_ind);

                // fill parameters
                rssi_ind->rssi = cmp_evt->rssi;

                // send indication
                ke_msg_send(rssi_ind);
            }
            else
            {
                status = RW_ERR_HCI_TO_HL(cmp_evt->status);
            }
        }
        /* send command complete event */
        gapc_send_complete_evt(conidx, GAPC_OP_LINK_INFO, status);

    }
    /* message is consumed */
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles read Channel Map command completed event
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPM).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_rd_chnl_map_cmd_cmp_evt_handler(ke_msg_id_t const msgid,
        struct hci_le_rd_ch_map_cmd_cmp_evt const *cmp_evt,
        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Current process state
    uint8_t state = ke_state_get(dest_id);
    // Current connection index
    uint8_t conidx = KE_IDX_GET(dest_id);

    if (state != GAPC_FREE)
    {
        uint8_t status = GAP_ERR_PROTOCOL_PROBLEM;
        if (gapc_get_operation(conidx, GAPC_OP_LINK_INFO) == GAPC_GET_CON_CHANNEL_MAP)
        {
            if ((cmp_evt->status == CO_ERROR_NO_ERROR))
            {
                status = GAP_ERR_NO_ERROR;
                // create rssi indication message.
                struct gapc_con_channel_map_ind *ch_map_ind =
                    KE_MSG_ALLOC(GAPC_CON_CHANNEL_MAP_IND, gapc_get_requester(conidx, GAPC_OP_LINK_INFO),
                                 KE_BUILD_ID(TASK_GAPC, conidx), gapc_con_channel_map_ind);

                // fill parameters
                memcpy(&(ch_map_ind->ch_map), &(cmp_evt->ch_map), sizeof(struct le_ch_map));

                // send indication
                ke_msg_send(ch_map_ind);
            }
            else
            {
                status = RW_ERR_HCI_TO_HL(cmp_evt->status);
            }
        }
        /* send command complete event */
        gapc_send_complete_evt(conidx, GAPC_OP_LINK_INFO, status);

    }
    /* message is consumed */
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Connection update parameter is done. Master role.
 * Slave application should handle the connection update parameter complete event!
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_le_con_update_cmp_evt_handler(ke_msg_id_t const msgid, struct hci_le_con_update_cmp_evt const *event,
        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Current process state
    uint8_t state = ke_state_get(dest_id);
    // Current connection index
    uint8_t conidx = KE_IDX_GET(dest_id);


    if (state != GAPC_FREE)
    {
        switch (event->status)
        {
#if (BLE_PERIPHERAL)
        case RW_ERR_HL_TO_HCI(LL_ERR_UNKNOWN_HCI_COMMAND) :
        case RW_ERR_HL_TO_HCI(LL_ERR_COMMAND_DISALLOWED) :
        case RW_ERR_HL_TO_HCI(LL_ERR_UNSUPPORTED) :
        case RW_ERR_HL_TO_HCI(LL_ERR_UNKNOWN_LMP_PDU) :
        case RW_ERR_HL_TO_HCI(LL_ERR_UNSUPPORTED_REMOTE_FEATURE) :
        {
            if (GAPC_GET_FIELD(conidx, ROLE) == ROLE_SLAVE)
            {
                // Clear parameter request feature in the environment because not supported by peer
                gapc_env[conidx]->features &= ~(GAPC_CONN_PARAM_REQ_FEAT_MASK);

                // Reschedule the operation to perform connection update by L2CAP
                gapc_reschedule_operation(conidx, GAPC_OP_LINK_UPD);
            }
            else
            {
                gapc_send_complete_evt(conidx, GAPC_OP_LINK_UPD, RW_ERR_HCI_TO_HL(event->status));
            }
        }
        break;
#endif // (BLE_PERIPHERAL)
        case GAP_ERR_NO_ERROR :
        {
            struct gapc_param_updated_ind *ind = KE_MSG_ALLOC(GAPC_PARAM_UPDATED_IND,
                                                 gapc_get_dest_task(conidx), dest_id,
                                                 gapc_param_updated_ind);
            /* fill up parameters */
            ind->con_interval = event->con_interval;
            ind->con_latency  = event->con_latency;
            ind->sup_to       = event->sup_to;

            /* send message */
            ke_msg_send(ind);
        }
        // no break
        default :
        {
            gapc_send_complete_evt(conidx, GAPC_OP_LINK_UPD, RW_ERR_HCI_TO_HL(event->status));
        }
        break;
        }
    }
    /* message is consumed */
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Connection update parameter request.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_le_rem_con_param_req_evt_handler(ke_msg_id_t const msgid, struct hci_le_rem_con_param_req_evt const *event,
        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Current process state
    uint8_t state = ke_state_get(dest_id);

    if (state != GAPC_FREE)
    {
        // send a command to request update of connection parameters
        struct gapc_param_update_cmd *cmd = KE_MSG_ALLOC(GAPC_PARAM_UPDATE_CMD,
                                            dest_id, dest_id, gapc_param_update_cmd);

        // Set parameter request feature in the environment because looks to be supported by peer
        gapc_env[KE_IDX_GET(dest_id)]->features |= (GAPC_CONN_PARAM_REQ_FEAT_MASK);

        /* fill up the parameters */
        cmd->operation = GAPC_UPDATE_PARAMS;
        cmd->intv_max = event->interval_max;
        cmd->intv_min = event->interval_min;
        cmd->latency  = event->latency;
        cmd->time_out = event->timeout;
        /* send the message */
        ke_msg_send(cmd);
    }

    /* message is consumed */
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Set Data Length extension event.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_le_data_len_chg_evt_handler(ke_msg_id_t const msgid, struct hci_le_data_len_chg_evt const *event,
        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Current process state
    uint8_t state = ke_state_get(dest_id);

    if (state != GAPC_FREE)
    {
        struct gapc_le_pkt_size_ind *ind = KE_MSG_ALLOC(GAPC_LE_PKT_SIZE_IND, gapc_get_dest_task(KE_IDX_GET(dest_id)),
                                           dest_id, gapc_le_pkt_size_ind);

        /* fill up parameters */
        ind->max_rx_octets  = event->max_rx_octets;
        ind->max_rx_time    = event->max_rx_time;
        ind->max_tx_octets  = event->max_tx_octets;
        ind->max_tx_time    = event->max_tx_time;

        /* send message */
        ke_msg_send(ind);
    }

    /* message is consumed */
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief LE PHY updated complete event
  *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_le_phy_upd_cmp_evt_handler(ke_msg_id_t const msgid, struct hci_le_phy_upd_cmp_evt const *event,
        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Current process state
    uint8_t state = ke_state_get(dest_id);
    // Current connection index
    uint8_t conidx = gapc_get_conidx(event->conhdl);

    if (state != GAPC_FREE)
    {
        if (event->status == CO_ERROR_NO_ERROR)
        {
            struct gapc_le_phy_ind *phy_ind = KE_MSG_ALLOC(GAPC_LE_PHY_IND, gapc_get_dest_task(conidx), dest_id, gapc_le_phy_ind);

            phy_ind->rx_phy = event->rx_phy;
            phy_ind->tx_phy = event->tx_phy;

            ke_msg_send(phy_ind);
        }

        if (gapc_get_operation(conidx, GAPC_OP_LINK_UPD) == GAPC_SET_PHY)
        {
            gapc_send_complete_evt(conidx, GAPC_OP_LINK_UPD, RW_ERR_HCI_TO_HL(event->status));
        }
    }

    /* message is consumed */
    return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief LE ping timeout expired
  *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_auth_payl_to_exp_evt_handler(ke_msg_id_t const msgid, struct hci_auth_payl_to_exp_evt const *event,
        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Current process state
    uint8_t state = ke_state_get(dest_id);

    if (state != GAPC_FREE)
    {
        ke_msg_send_basic(GAPC_LE_PING_TO_IND, gapc_get_dest_task(KE_IDX_GET(dest_id)), dest_id);
    }

    /* message is consumed */
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief LE ping read timeout value
  *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_rd_auth_payl_to_cmd_cmp_evt_handler(ke_msg_id_t const msgid, struct hci_rd_auth_payl_to_cmd_cmp_evt const *event,
        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Current process state
    uint8_t state = ke_state_get(dest_id);
    // Current connection index
    uint8_t conidx = KE_IDX_GET(dest_id);

    if (state != GAPC_FREE)
    {
        if (event->status == GAP_ERR_NO_ERROR)
        {
            struct gapc_le_ping_to_val_ind *ind = KE_MSG_ALLOC(GAPC_LE_PING_TO_VAL_IND, gapc_get_requester(conidx, GAPC_OP_LINK_INFO),
                                                  dest_id, gapc_le_ping_to_val_ind);

            /* fill up parameters */

            ind->timeout       = event->auth_payl_to;

            /* send message */
            ke_msg_send(ind);
        }

        gapc_send_complete_evt(conidx, GAPC_OP_LINK_INFO, RW_ERR_HCI_TO_HL(event->status));
    }

    /* message is consumed */
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief LE ping set timeout value
  *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_wr_auth_payl_to_cmd_cmp_evt_handler(ke_msg_id_t const msgid, struct hci_wr_auth_payl_to_cmd_cmp_evt const *event,
        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Current process state
    uint8_t state = ke_state_get(dest_id);
    // Current connection index
    uint8_t conidx = KE_IDX_GET(dest_id);

    if (state != GAPC_FREE)
    {
        gapc_send_complete_evt(conidx, GAPC_OP_LINK_INFO, RW_ERR_HCI_TO_HL(event->status));
    }

    /* message is consumed */
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Basic Command complete event
  *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_le_set_data_len_cmd_cmp_evt_handler(ke_msg_id_t const msgid, struct hci_basic_conhdl_cmd_cmp_evt const *event,
        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Current process state
    uint8_t state = ke_state_get(dest_id);
    // Current connection index
    uint8_t conidx = gapc_get_conidx(event->conhdl);

    if (state != GAPC_FREE)
    {
        /* send command complete event with error code*/
        gapc_send_complete_evt(conidx, GAPC_OP_LINK_INFO, RW_ERR_HCI_TO_HL(event->status));
    }

    /* message is consumed */
    return (KE_MSG_CONSUMED);
}

#if (BLE_PERIPHERAL)
/**
 ****************************************************************************************
 * @brief Command complete event for HCI VS set preferred slave latency
  *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_vs_set_pref_slave_latency_cmd_cmp_evt_handler(ke_msg_id_t const msgid, struct hci_basic_conhdl_cmd_cmp_evt const *event,
        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Current process state
    uint8_t state = ke_state_get(dest_id);
    // Current connection index
    uint8_t conidx = gapc_get_conidx(event->conhdl);

    if (state != GAPC_FREE)
    {
        /* send command complete event with error code*/
        gapc_send_complete_evt(conidx, GAPC_OP_LINK_INFO, RW_ERR_HCI_TO_HL(event->status));
    }

    /* message is consumed */
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Command complete event for HCI VS set preferred slave event duration
  *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_vs_set_pref_slave_evt_dur_cmd_cmp_evt_handler(ke_msg_id_t const msgid, struct hci_basic_conhdl_cmd_cmp_evt const *event,
        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Current process state
    uint8_t state = ke_state_get(dest_id);
    // Current connection index
    uint8_t conidx = gapc_get_conidx(event->conhdl);

    if (state != GAPC_FREE)
    {
        /* send command complete event with error code*/
        gapc_send_complete_evt(conidx, GAPC_OP_LINK_INFO, RW_ERR_HCI_TO_HL(event->status));
    }

    /* message is consumed */
    return (KE_MSG_CONSUMED);
}
#endif // (BLE_PERIPHERAL)

/**
 ****************************************************************************************
 * @brief Read LE PHY Command complete event
  *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_le_rd_phy_cmd_cmp_evt_handler(ke_msg_id_t const msgid, struct hci_le_rd_phy_cmd_cmp_evt const *event,
        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Current process state
    uint8_t state = ke_state_get(dest_id);
    // Current connection index
    uint8_t conidx = gapc_get_conidx(event->conhdl);

    if (state != GAPC_FREE)
    {
        if (event->status == CO_ERROR_NO_ERROR)
        {
            struct gapc_le_phy_ind *phy_ind = KE_MSG_ALLOC(GAPC_LE_PHY_IND, gapc_get_dest_task(conidx), dest_id, gapc_le_phy_ind);

            phy_ind->rx_phy = event->rx_phy;
            phy_ind->tx_phy = event->tx_phy;

            ke_msg_send(phy_ind);
        }

        /* send command complete event with error code*/
        gapc_send_complete_evt(conidx, GAPC_OP_LINK_INFO, RW_ERR_HCI_TO_HL(event->status));
    }

    /* message is consumed */
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles common status event for connection purpose.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPM).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_cmd_stat_event_handler(ke_msg_id_t const msgid,
                                      struct hci_cmd_stat_event const *event,
                                      ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Current process state
    uint8_t state = ke_state_get(dest_id);
    // Current connection index
    uint8_t conidx = KE_IDX_GET(dest_id);

    if (state != GAPC_FREE)
    {
        switch (msgid)
        {
        // Disconnect status event.
        case HCI_DISCONNECT_CMD_OPCODE:
        {
            if (event->status != CO_ERROR_NO_ERROR)
            {
                // send error event only if it's a requested disconnect
                if (gapc_env[conidx]->disc_requester != TASK_NONE)
                {
                    /* send disconnect command complete event */
                    gapc_send_error_evt(conidx, GAPC_DISCONNECT, gapc_env[conidx]->disc_requester,
                                        RW_ERR_HCI_TO_HL(event->status));

                    //Yier, doesn't find the meaing of this condition, mark it first.
                    //if (gapc_env[conidx]->operation == NULL)
                    {
                        // set state to ready
                        ke_state_set(KE_BUILD_ID(TASK_GAPC, conidx), GAPC_READY);
                    }
                }
            }
        }
        break;
        // Read Remote version status event
        case HCI_RD_REM_VER_INFO_CMD_OPCODE:
        // Read Remote feature status event
        case HCI_LE_RD_REM_FEATS_CMD_OPCODE:
        {
            if (event->status != CO_ERROR_NO_ERROR)
            {
                /* send command complete event with error */
                gapc_send_complete_evt(conidx, GAPC_OP_LINK_INFO, RW_ERR_HCI_TO_HL(event->status));
            }
        }
        break;
        // Update connection parameter status event
        case HCI_LE_CON_UPDATE_CMD_OPCODE:
        {
            // finish operation
            if ((event->status != CO_ERROR_NO_ERROR) && (gapc_get_operation(conidx, GAPC_OP_LINK_UPD) == GAPC_UPDATE_PARAMS))
            {
                switch (event->status)
                {
#if (BLE_PERIPHERAL)
                case RW_ERR_HL_TO_HCI(LL_ERR_UNKNOWN_HCI_COMMAND) :
                case RW_ERR_HL_TO_HCI(LL_ERR_COMMAND_DISALLOWED) :
                case RW_ERR_HL_TO_HCI(LL_ERR_UNSUPPORTED) :
                case RW_ERR_HL_TO_HCI(LL_ERR_UNKNOWN_LMP_PDU) :
                case RW_ERR_HL_TO_HCI(LL_ERR_UNSUPPORTED_REMOTE_FEATURE) :
                {
                    if (GAPC_GET_FIELD(conidx, ROLE) == ROLE_SLAVE)
                    {
                        // Clear parameter request feature in the environment because not supported by peer
                        gapc_env[conidx]->features &= ~(GAPC_CONN_PARAM_REQ_FEAT_MASK);

                        // Reschedule the operation to perform connection update by L2CAP
                        gapc_reschedule_operation(conidx, GAPC_OP_LINK_UPD);
                        break;
                    }
                }
                    // no break
#endif // (BLE_PERIPHERAL)

                default:
                {
                    gapc_send_complete_evt(conidx, GAPC_OP_LINK_UPD, RW_ERR_HCI_TO_HL(event->status));
                }
                break;
                }
            }
        }
        break;
        // SET PHY Command status event
        case HCI_LE_SET_PHY_CMD_OPCODE:
        {
            // finish operation
            if ((event->status != CO_ERROR_NO_ERROR) && (gapc_get_operation(conidx, GAPC_OP_LINK_UPD) == GAPC_SET_PHY))
            {
                gapc_send_complete_evt(conidx, GAPC_OP_LINK_UPD, RW_ERR_HCI_TO_HL(event->status));
            }
        }
        break;
        default:
        {
            /* send command complete event with error */
            gapc_send_complete_evt(conidx, GAPC_OP_LINK_INFO, GAP_ERR_PROTOCOL_PROBLEM);
        }
        break;
        } /* end of switch */
    }
    /* message is consumed */
    return (KE_MSG_CONSUMED);
}


#if (BLE_CENTRAL)
/**
 ****************************************************************************************
 * @brief Handles common command status events from HCI.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] event     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_SMP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_le_start_enc_stat_evt_handler(ke_msg_id_t const msgid,
        struct hci_cmd_stat_event const *event,
        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Connection Index
    uint8_t idx = KE_IDX_GET(dest_id);

    if (ke_state_get(dest_id) != GAPC_FREE)
    {
        /*
         * The HCI command status event is sent by the LL upon reception of the HCI_LE_START_ENC_CMD
         */
        if (event->status != CO_ERROR_NO_ERROR)
        {
            // Encryption has not be started due to an error in LL
            ASSERT_ERR(gapc_get_operation(idx, GAPC_OP_SMP) == GAPC_ENCRYPT);
            gapc_send_complete_evt(idx, GAPC_OP_SMP, RW_ERR_HCI_TO_HL(event->status));
        }
    }

    return (KE_MSG_CONSUMED);
}
#endif //(BLE_CENTRAL)

#if (BLE_PERIPHERAL)
/**
 ****************************************************************************************
 * @brief Handles long term key request from link layer.
 * Link layer checks if host has long term key corresponding to the given EDIV and Rand.
 * Only a slave can receive this event,
 * It is due to session encryption procedure started by Master.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] event     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_SMP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_le_ltk_request_evt_handler(ke_msg_id_t const msgid,
        struct hci_le_ltk_request_evt const *param,
        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Recover connection index
    uint8_t idx = gapc_get_conidx(param->conhdl);

    if (idx != GAP_INVALID_CONIDX)
    {
        // Master started encryption with LTK
        if (gapc_get_operation(idx, GAPC_OP_SMP) != GAPC_BOND)
        {
            // generate a local encrypt command to update correctly internal state machine
            struct gapc_encrypt_cmd *cmd = KE_MSG_ALLOC(GAPC_ENCRYPT_CMD,
                                           KE_BUILD_ID(TASK_GAPC, idx), dest_id,
                                           gapc_encrypt_cmd);
            cmd->operation = GAPC_ENCRYPT;
            cmd->ltk.ediv = param->ediv;
            memcpy(&cmd->ltk.randnb, &param->rand, sizeof(struct rand_nb));

            ke_msg_send(cmd);
        }
        // Master started encryption with STK
        else
        {
#if (SECURE_CONNECTIONS)
            if (smpc_secure_connections_enabled(idx))
            {
                ASSERT_INFO(gapc_env[idx]->smpc.state == SMPC_PAIRING_SC_W4_ENCRYPTION_START,
                            idx, gapc_env[idx]->smpc.state);

                gapc_env[idx]->smpc.state = SMPC_PAIRING_SC_W4_ENCRYPTION_CHANGE;
                smpc_send_ltk_req_rsp(idx, true, &gapc_env[idx]->smpc.info.pair->key.key[0]);
            }
            else
#endif // (SECURE_CONNECTIONS)
            {
                // Calculate STK
                smpc_generate_stk(idx, ROLE_SLAVE);
            }
        }
    }

    //message is consumed
    return (KE_MSG_CONSUMED);
}
#endif //(BLE_PERIPHERAL)

/**
 ****************************************************************************************
 * @brief Handles the HCI encryption change event.
 *        This event with Ok status will symbolize the end of the link encryption session
 *        for a link that was not encrypted to start with.
 *        It can arrive in two cases
 *         - the un-encrypted link is encrypted with STK after pairing
 *         - the un-encrypted link is encrypted with LTK using existing LTK, wo pairing
 * Both slave and master receive this event, after which, if status OK and no TKDP,
 * they send the host an ENC_STARTED_IND.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] event     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_enc_chg_evt_handler(ke_msg_id_t const msgid,
                                   struct hci_enc_change_evt const *param,
                                   ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t conidx = gapc_get_conidx(param->conhdl);

    if (conidx != GAP_INVALID_CONIDX)
    {
        smpc_handle_enc_change_evt(conidx, gapc_get_role(conidx), param->status);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the HCI encryption key refresh event.
 *        This event with Ok status will symbolize the end of the link encryption session
 *        with a new key . This can arrive in two cases:
 *         - the pairing and encryption using STK was done on an already encrypted link
 *         - the link encrypted under STK is re-encrypted using LTK.
 *        Any different status will be signaled to the Host as error in encrypting the link.
 * Both slave and master receive this event, after which, if encryption successful,
 * they send the host an ENC_STARTED_IND.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] event     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_enc_key_refr_evt_handler(ke_msg_id_t const msgid,
                                        struct hci_enc_key_ref_cmp_evt const *param,
                                        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t conidx = gapc_get_conidx(param->conhdl);

    if (conidx != GAP_INVALID_CONIDX)
    {
        smpc_handle_enc_change_evt(conidx, gapc_get_role(conidx), param->status);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handle reception of HCI LE Channel Selection Algorithm event.
 *
 * @param[in] opcode      Operation code of received message
 * @param[in] p_event     Pointer to the parameters of the message.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int hci_le_ch_sel_algo_evt_handler(uint16_t opcode, struct hci_le_ch_sel_algo_evt const *p_evt)
{
    uint8_t conidx = gapc_get_conidx(p_evt->conhdl);

    if (conidx != GAP_INVALID_CONIDX)
    {
        gapc_env[conidx]->chan_sel_algo = p_evt->ch_sel_algo;
    }

    return (KE_MSG_CONSUMED);
}

/*
 * HCI HANDLER DEFINITIONS
 ****************************************************************************************
 */


/// The message handlers for HCI command complete events
static const struct ke_msg_handler hci_cmd_cmp_event_handler_tab[] =
{
#ifndef GAPC_SLIM
    { HCI_RD_RSSI_CMD_OPCODE, (ke_msg_func_t) hci_rd_rssi_cmd_cmp_evt_handler },
    { HCI_LE_RD_CH_MAP_CMD_OPCODE, (ke_msg_func_t) hci_rd_chnl_map_cmd_cmp_evt_handler },
    { HCI_RD_AUTH_PAYL_TO_CMD_OPCODE, (ke_msg_func_t) hci_rd_auth_payl_to_cmd_cmp_evt_handler },
    { HCI_WR_AUTH_PAYL_TO_CMD_OPCODE, (ke_msg_func_t) hci_wr_auth_payl_to_cmd_cmp_evt_handler },
#endif //GAPC_SLIM
    { HCI_LE_SET_DATA_LEN_CMD_OPCODE, (ke_msg_func_t) hci_le_set_data_len_cmd_cmp_evt_handler },
    { HCI_LE_RD_PHY_CMD_OPCODE, (ke_msg_func_t) hci_le_rd_phy_cmd_cmp_evt_handler },

#if (BLE_PERIPHERAL)
    { HCI_LE_LTK_REQ_REPLY_CMD_OPCODE, (ke_msg_func_t) ke_msg_discard},
    { HCI_LE_LTK_REQ_NEG_REPLY_CMD_OPCODE, (ke_msg_func_t) ke_msg_discard},
#ifndef GAPC_SLIM
    { HCI_VS_SET_PREF_SLAVE_LATENCY_CMD_OPCODE, (ke_msg_func_t) hci_vs_set_pref_slave_latency_cmd_cmp_evt_handler},
    { HCI_VS_SET_PREF_SLAVE_EVT_DUR_CMD_OPCODE, (ke_msg_func_t) hci_vs_set_pref_slave_evt_dur_cmd_cmp_evt_handler},
#endif // !GAPC_SLIM
#endif //(BLE_PERIPHERAL)
};


/// The message handlers for HCI LE events
static const struct ke_msg_handler hci_cmd_stat_event_handler_tab[] =
{
    { HCI_DISCONNECT_CMD_OPCODE, (ke_msg_func_t) hci_cmd_stat_event_handler },
    { HCI_RD_REM_VER_INFO_CMD_OPCODE, (ke_msg_func_t) hci_cmd_stat_event_handler },
    { HCI_LE_RD_REM_FEATS_CMD_OPCODE, (ke_msg_func_t) hci_cmd_stat_event_handler },
    { HCI_LE_CON_UPDATE_CMD_OPCODE, (ke_msg_func_t) hci_cmd_stat_event_handler },
    { HCI_LE_SET_PHY_CMD_OPCODE, (ke_msg_func_t) hci_cmd_stat_event_handler },

#if (BLE_CENTRAL)
    { HCI_LE_EN_ENC_CMD_OPCODE, (ke_msg_func_t) hci_le_start_enc_stat_evt_handler},
#endif //(BLE_CENTRAL)
};


/// The message handlers for HCI LE events
static const struct ke_msg_handler hci_le_event_handler_tab[] =
{
    { HCI_LE_RD_REM_FEATS_CMP_EVT_SUBCODE, (ke_msg_func_t) hci_le_rd_rem_feats_cmp_evt_handler },
    { HCI_LE_CON_UPDATE_CMP_EVT_SUBCODE, (ke_msg_func_t) hci_le_con_update_cmp_evt_handler },
    { HCI_LE_REM_CON_PARAM_REQ_EVT_SUBCODE, (ke_msg_func_t) hci_le_rem_con_param_req_evt_handler },
    { HCI_LE_DATA_LEN_CHG_EVT_SUBCODE, (ke_msg_func_t) hci_le_data_len_chg_evt_handler },
    { HCI_LE_PHY_UPD_CMP_EVT_SUBCODE, (ke_msg_func_t) hci_le_phy_upd_cmp_evt_handler },
    { HCI_LE_CH_SEL_ALGO_EVT_SUBCODE, (ke_msg_func_t) hci_le_ch_sel_algo_evt_handler },

#if (BLE_PERIPHERAL)
    { HCI_LE_LTK_REQUEST_EVT_SUBCODE, (ke_msg_func_t) hci_le_ltk_request_evt_handler},
#endif //(BLE_PERIPHERAL)
};
/// The message handlers for HCI events
static const struct ke_msg_handler hci_event_handler_tab[] =
{
    { HCI_DISC_CMP_EVT_CODE, (ke_msg_func_t) hci_disc_cmp_evt_handler },
    { HCI_RD_REM_VER_INFO_CMP_EVT_CODE, (ke_msg_func_t) hci_rd_rem_ver_info_cmp_evt_handler },
    { HCI_AUTH_PAYL_TO_EXP_EVT_CODE, (ke_msg_func_t) hci_auth_payl_to_exp_evt_handler },
    { HCI_ENC_CHG_EVT_CODE, (ke_msg_func_t) hci_enc_chg_evt_handler},
    { HCI_ENC_KEY_REFRESH_CMP_EVT_CODE, (ke_msg_func_t) hci_enc_key_refr_evt_handler},
};


/**
 ****************************************************************************************
 * @brief Handles any HCI event
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPC).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int gapc_hci_handler(ke_msg_id_t const msgid, void const *event, ke_task_id_t dest_id, ke_task_id_t src_id)
{
    int return_status = KE_MSG_CONSUMED;

    const struct ke_msg_handler *handler_tab = NULL;
    uint32_t tab_size = 0;

    switch (msgid)
    {
    case HCI_CMD_CMP_EVENT:
    {
        handler_tab = hci_cmd_cmp_event_handler_tab;
        tab_size    = sizeof(hci_cmd_cmp_event_handler_tab) / sizeof(hci_cmd_cmp_event_handler_tab[0]);
    }
    break;

    case HCI_CMD_STAT_EVENT:
    {
        handler_tab = hci_cmd_stat_event_handler_tab;
        tab_size    = sizeof(hci_cmd_stat_event_handler_tab) / sizeof(hci_cmd_stat_event_handler_tab[0]);
    }
    break;

    case HCI_LE_EVENT:
    {
        handler_tab = hci_le_event_handler_tab;
        tab_size    = sizeof(hci_le_event_handler_tab) / sizeof(hci_le_event_handler_tab[0]);
        // Get subcode at from message parameters (1st byte position)
        src_id      = *((uint8_t *)event);
    }
    break;
    case HCI_EVENT:
    {
        handler_tab = hci_event_handler_tab;
        tab_size    = sizeof(hci_event_handler_tab) / sizeof(hci_event_handler_tab[0]);
    }
    break;
    default:   /* should not occurs, nothing to do */
        break;
    }

    // Check if there is a handler corresponding to the original command opcode
    for (uint32_t i = 0; i < tab_size; i++)
    {
        // Check if opcode matches
        if (handler_tab[i].id == src_id)
        {
            // Check if there is a handler function
            if (handler_tab[i].func != NULL)
            {
                // Call handler
                return_status = handler_tab[i].func(src_id, event, dest_id, src_id);
            }
            break;
        }
    }

    return return_status;
}
#endif // (BLE_CENTRAL || BLE_PERIPHERAL)

/// @} GAPC_HCI
