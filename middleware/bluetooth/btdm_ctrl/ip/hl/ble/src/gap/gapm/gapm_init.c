/**
 ****************************************************************************************
 *
 * @file gapm_init.c
 *
 * @brief Generic Access Profile Manager - Initiating manager module.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup GAPM_INIT Generic Access Profile Manager - Initiating manager module.
 * @ingroup GAPM
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (BLE_CENTRAL)

#include <string.h>

#include "gap.h"
#include "gapc.h"
#include "gapm_task.h"
#include "gapc_task.h"
#include "gapm_int.h"
#include "ke_mem.h"
#include "ke_timer.h"
#include "hci.h"
#include "co_math.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/*
 * MACROS
 ****************************************************************************************
 */

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Send a LE Extended Create Connection command over HCI.
 *
 * @param[in] p_actv_init    Pointer to the initiating activity structure
 * @param[in] p_init_param   Pointer to the initiating parameters provided by the application
 ****************************************************************************************
 */
__STATIC void gapm_init_send_hci_le_ext_create_con_cmd(struct gapm_actv_init_tag *p_actv_init,
        struct gapm_init_param *p_init_param)
{
    // Counter
    int i = 0;
    // Allocate HCI command message
    struct hci_le_ext_create_con_cmd *p_cmd = KE_MSG_ALLOC(HCI_COMMAND, 0, HCI_LE_EXT_CREATE_CON_CMD_OPCODE,
            hci_le_ext_create_con_cmd);

    // Fill the command parameters
    p_cmd->own_addr_type = gapm_actv_get_hci_own_addr_type(p_actv_init->common.own_addr_type);

    if (p_actv_init->common.subtype != GAPM_INIT_TYPE_AUTO_CONN_EST)
    {
        // Do not use white list
        p_cmd->init_filter_policy = INIT_FILT_IGNORE_WLST;

        // Peer address
        p_cmd->peer_addr_type = p_init_param->peer_addr.addr_type;
        memcpy(&p_cmd->peer_addr.addr[0], &p_init_param->peer_addr.addr.addr[0], GAP_BD_ADDR_LEN);
    }
    else
    {
        // Use white list
        p_cmd->init_filter_policy = INIT_FILT_USE_WLST;
    }

    p_cmd->init_phys = p_init_param->prop & (GAPM_INIT_PROP_1M_BIT | GAPM_INIT_PROP_2M_BIT |
                       GAPM_INIT_PROP_CODED_BIT);

    if (p_init_param->prop & GAPM_INIT_PROP_1M_BIT)
    {
        p_cmd->phy[i].scan_interval = p_init_param->scan_param_1m.scan_intv;
        p_cmd->phy[i].scan_window   = p_init_param->scan_param_1m.scan_wd;
        p_cmd->phy[i].con_intv_min  = p_init_param->conn_param_1m.conn_intv_min;
        p_cmd->phy[i].con_intv_max  = p_init_param->conn_param_1m.conn_intv_max;
        p_cmd->phy[i].ce_len_min    = p_init_param->conn_param_1m.ce_len_min;
        p_cmd->phy[i].ce_len_max    = p_init_param->conn_param_1m.ce_len_max;
        p_cmd->phy[i].con_latency   = p_init_param->conn_param_1m.conn_latency;
        p_cmd->phy[i].superv_to     = p_init_param->conn_param_1m.supervision_to;

        // Increase counter
        i++;
    }

    if (p_init_param->prop & GAPM_INIT_PROP_2M_BIT)
    {
        p_cmd->phy[i].con_intv_min  = p_init_param->conn_param_2m.conn_intv_min;
        p_cmd->phy[i].con_intv_max  = p_init_param->conn_param_2m.conn_intv_max;
        p_cmd->phy[i].ce_len_min    = p_init_param->conn_param_2m.ce_len_min;
        p_cmd->phy[i].ce_len_max    = p_init_param->conn_param_2m.ce_len_max;
        p_cmd->phy[i].con_latency   = p_init_param->conn_param_2m.conn_latency;
        p_cmd->phy[i].superv_to     = p_init_param->conn_param_2m.supervision_to;

        // Increase counter
        i++;
    }

    if (p_init_param->prop & GAPM_INIT_PROP_CODED_BIT)
    {
        p_cmd->phy[i].scan_interval = p_init_param->scan_param_coded.scan_intv;
        p_cmd->phy[i].scan_window   = p_init_param->scan_param_coded.scan_wd;
        p_cmd->phy[i].con_intv_min  = p_init_param->conn_param_coded.conn_intv_min;
        p_cmd->phy[i].con_intv_max  = p_init_param->conn_param_coded.conn_intv_max;
        p_cmd->phy[i].ce_len_min    = p_init_param->conn_param_coded.ce_len_min;
        p_cmd->phy[i].ce_len_max    = p_init_param->conn_param_coded.ce_len_max;
        p_cmd->phy[i].con_latency   = p_init_param->conn_param_coded.conn_latency;
        p_cmd->phy[i].superv_to     = p_init_param->conn_param_coded.supervision_to;
    }

    // Send the command
    hci_send_2_controller(p_cmd);

    // Keep in mind which command status event has to be received
    p_actv_init->common.next_exp_opcode = HCI_LE_EXT_CREATE_CON_CMD_OPCODE;
}

/**
 ****************************************************************************************
 * @brief Send a LE Create Connection Cancel command over HCI. The command
 * complete event is handled in hci_le_cmd_cmp_evt_init_handler function.
 ****************************************************************************************
 */
__STATIC void gapm_init_send_hci_le_create_con_cancel_cmd(struct gapm_actv_init_tag *p_actv_init)
{
    hci_basic_cmd_send_2_controller(HCI_LE_CREATE_CON_CANCEL_CMD_OPCODE);

    // Keep in mind which command complete event has to be received
    p_actv_init->common.next_exp_opcode = HCI_LE_CREATE_CON_CANCEL_CMD_OPCODE;
}

/**
 ****************************************************************************************
 * @brief Following HCI command complete event message can be received from controller
 * while activity state is STARTING:
 *    - LE Extended Create Connection Command (HCI_LE_EXT_CREATE_CON_CMD_OPCODE)
 *
 * @param[in|out] p_actv_init    Pointer to the activity structure for which the event has
 * been received.
 * @param[in] opcode             Operation code of received message.
 * @param[in] p_event            Pointer to the HCI command complete event message content.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC void hci_le_cmd_cmp_evt_starting_handler(struct gapm_actv_init_tag *p_actv_init,
        uint16_t opcode,
        struct hci_basic_cmd_cmp_evt const *p_event)
{
    // Map the received status on HL error status
    uint8_t status = RW_ERR_HCI_TO_HL(p_event->status);

    if (status != GAP_ERR_NO_ERROR)
    {
        gapm_env.init_actv_idx = GAPM_ACTV_INVALID_IDX;

        if (p_actv_init->common.subtype == GAPM_INIT_TYPE_AUTO_CONN_EST)
        {
            // Clear timeout timer for automatic connection
            ke_timer_clear(GAPM_AUTO_CONN_TO_IND, TASK_GAPM);
        }
    }

    // Operation is over
    gapm_actv_started((struct gapm_actv_tag *)p_actv_init, status);
}

/**
 ****************************************************************************************
 * @brief Following HCI command complete event message can be received from controller
 * while activity state is STARTER:
 *    - LE Extended Create Connection Command (HCI_LE_EXT_CREATE_CON_CMD_OPCODE)
 *          -> In case of Automatic Connection Establishment
 *
 * @param[in|out] p_actv_init    Pointer to the activity structure for which the event has
 * been received.
 * @param[in] opcode             Operation code of received message.
 * @param[in] p_event            Pointer to the HCI command complete event message content.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC void hci_le_cmd_cmp_evt_started_handler(struct gapm_actv_init_tag *p_actv_init,
        uint16_t opcode,
        struct hci_basic_cmd_cmp_evt const *p_event)
{
    // Map the received status on HL error status
    uint8_t status = RW_ERR_HCI_TO_HL(p_event->status);

    if (status != GAP_ERR_NO_ERROR)
    {
        // Clear timeout timer for automatic connection
        ke_timer_clear(GAPM_AUTO_CONN_TO_IND, TASK_GAPM);

        gapm_env.init_actv_idx = GAPM_ACTV_INVALID_IDX;

        // Operation is over
        gapm_actv_stopped((struct gapm_actv_tag *)p_actv_init, status);
    }
}

/**
 ****************************************************************************************
 * @brief Following HCI command complete event message can be received from controller
 * while activity state is STOPPING:
 *    - LE Create Connection Cancel Command (HCI_LE_CREATE_CON_CANCEL_CMD_OPCODE)
 *
 * @param[in|out] p_actv_init    Pointer to the activity structure for which the event has
 * been received.
 * @param[in] opcode             Operation code of received message.
 * @param[in] p_event            Pointer to the HCI command complete event message content.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC void hci_le_cmd_cmp_evt_stopping_handler(struct gapm_actv_init_tag *p_actv_init,
        uint16_t opcode,
        struct hci_basic_cmd_cmp_evt const *p_event)
{
    gapm_env.init_actv_idx = GAPM_ACTV_INVALID_IDX;

    // Operation is over
    gapm_actv_stopped((struct gapm_actv_tag *)p_actv_init, p_actv_init->kept_status);
}

/**
 ****************************************************************************************
 * @brief Check initiating parameters provided by the application.
 *
 * @param[in] p_init_param      Pointer to the initiating parameters
 *
 * @return GAP_ERR_NO_ERROR if parameters are valid, else GAP_ERR_INVALID_PARAM
 ****************************************************************************************
 */
__STATIC uint8_t gapm_init_check_param(struct gapm_init_param *p_init_param)
{
    // Error code, invalid parameter by default
    uint8_t error = GAP_ERR_INVALID_PARAM;

    do
    {
        // Check initiating type
        if (p_init_param->type > GAPM_INIT_TYPE_NAME_DISC)
        {
            break;
        }

        // Check peer address type
        if (p_init_param->peer_addr.addr_type > ADDR_RAND)
        {
            break;
        }

        error = GAP_ERR_NO_ERROR;
    }
    while (0);

    return (error);
}

/**
 ****************************************************************************************
 * @brief Start an initiating activity.
 *
 * @param[in] p_actv        Pointer to the activity structure
 * @param[in] p_param       GAPM_ACTIVITY_START_CMD message parameters
 ****************************************************************************************
 */
__STATIC uint8_t gapm_init_start(struct gapm_actv_tag *p_actv, struct gapm_activity_start_cmd *p_param)
{
    // Cast the activity structure into an initiating activity structure
    struct gapm_actv_init_tag *p_actv_init = (struct gapm_actv_init_tag *)p_actv;
    // Initiating Parameters
    struct gapm_init_param *p_init_param = (struct gapm_init_param *)&p_param->u_param.init_param;
    // Check new provided initiating parameters
    uint8_t error;

    do
    {
        // Cannot have two initiating procedures in parallel
        if (gapm_env.init_actv_idx != GAPM_ACTV_INVALID_IDX)
        {
            error = GAP_ERR_COMMAND_DISALLOWED;
            break;
        }

        // Check provided initiating parameters
        error = gapm_init_check_param(p_init_param);

        if (error != GAP_ERR_NO_ERROR)
        {
            break;
        }

        // Configure the activity
        p_actv_init->common.subtype = p_init_param->type;
        p_actv_init->kept_status = GAP_ERR_NO_ERROR;

        if (p_actv_init->common.subtype == GAPM_INIT_TYPE_AUTO_CONN_EST)
        {
            // Keep initiating parameters
            memcpy(&p_actv_init->init_param, &p_param->u_param.init_param, sizeof(struct gapm_init_param));

            // Keep number of connection to be established in mind
            p_actv_init->nb_auto_conn = gapm_env.nb_dev_wl;

            // Start timeout timer for automatic connection
            if (p_init_param->conn_to)
            {
                ke_timer_set(GAPM_AUTO_CONN_TO_IND, TASK_GAPM, p_init_param->conn_to);
            }
        }

        // Keep activity identifier in mind
        gapm_env.actv_idx      = p_actv_init->common.idx;
        gapm_env.init_actv_idx = p_actv_init->common.idx;

        // Send a LE Extended Create Connection command to the controller
        gapm_init_send_hci_le_ext_create_con_cmd(p_actv_init, p_init_param);
    }
    while (0);

    return (error);
}

/**
 ****************************************************************************************
 * @brief Stop one initiating activity. Activity state must have been set to STOPPING state
 * before calling this function
 *
 * @param[in] p_actv    Pointer to the activity structure to be stopped
 ****************************************************************************************
 */
__STATIC void gapm_init_stop(struct gapm_actv_tag *p_actv)
{
    // Cast the activity structure in order to retrieve initiating parameters
    struct gapm_actv_init_tag *p_actv_init = (struct gapm_actv_init_tag *)p_actv;

    // Clear timeout timer if automatic connection establishment
    if (p_actv_init->common.subtype == GAPM_INIT_TYPE_AUTO_CONN_EST)
    {
        ke_timer_clear(GAPM_AUTO_CONN_TO_IND, TASK_GAPM);
    }

    // Keep activity identifier in mind
    gapm_env.actv_idx = p_actv_init->common.idx;

    // Send a LE Set Extended Scan Enable command (Disable) to the controller
    gapm_init_send_hci_le_create_con_cancel_cmd(p_actv_init);
}

/**
 ****************************************************************************************
 * @brief Delete an initiating activity. It is considered that activity state is DELETING.
 *
 * @param[in] p_actv    Pointer to the activity structure for the activity to delete
 ****************************************************************************************
 */
__STATIC void gapm_init_delete(struct gapm_actv_tag *p_actv)
{
    // Nothing special to be done
    gapm_actv_deleted(p_actv);
}

/*
 * EXTERNAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

uint8_t gapm_init_create(uint8_t actv_idx, struct gapm_activity_create_cmd *p_param)
{
    // Check provided scanning parameters
    uint8_t error = GAP_ERR_NO_ERROR;
    // Allocate an activity structure
    struct gapm_actv_init_tag *p_actv_init;

    do
    {
        // Central role must be supported by the device
        if (!GAPM_IS_ROLE_SUPPORTED(GAP_ROLE_CENTRAL))
        {
            error = GAP_ERR_COMMAND_DISALLOWED;
            break;
        }

        // Check that provided address type is valid
        if (!gapm_is_addr_type_valid(p_param->own_addr_type))
        {
            error = GAP_ERR_INVALID_PARAM;
            break;
        }

        // If controller privacy is enabled, non resolvable private address can not be used.
        if ((gapm_get_address_type() & GAPM_PRIV_CFG_PRIV_EN_BIT)
                && (p_param->own_addr_type == GAPM_GEN_NON_RSLV_ADDR))
        {
            error = GAP_ERR_PRIVACY_CFG_PB;
            break;
        }

        p_actv_init = (struct gapm_actv_init_tag *)gapm_actv_alloc(actv_idx, sizeof(struct gapm_actv_init_tag));

        // Check if enough memory has been found
        if (!p_actv_init)
        {
            error = GAP_ERR_INSUFF_RESOURCES;
            break;
        }

        // Fill the activity structure
        p_actv_init->common.type = GAPM_ACTV_TYPE_INIT;
        p_actv_init->common.own_addr_type = p_param->own_addr_type;
        p_actv_init->common.requester = gapm_get_requester(GAPM_OP_AIR);

        // Set callback functions
        p_actv_init->common.cb_actv_start  = gapm_init_start;
        p_actv_init->common.cb_actv_stop   = gapm_init_stop;
        p_actv_init->common.cb_actv_delete = gapm_init_delete;

        gapm_actv_created((struct gapm_actv_tag *)p_actv_init, GAP_ERR_NO_ERROR);
    }
    while (0);

    return (error);
}

bool gapm_init_connection_ind(uint8_t conidx)
{
    // Retrieve initiating activity currently in use
    struct gapm_actv_init_tag *p_actv_init = (struct gapm_actv_init_tag *)gapm_env.actvs[gapm_env.init_actv_idx];
    // Indicate if activity has been stopped
    bool stop = false;

    // Sanity check
    ASSERT_ERR(p_actv_init);

    // Check if connection has been established
    if (conidx != GAP_INVALID_CONIDX)
    {
        // If resolvable private address or non-resolvable private address has been used for advertising,
        // and controller privacy was not enabled, we need to keep the address generated by the host as
        // a local address for pairing.
        if ((p_actv_init->common.own_addr_type != GAPM_STATIC_ADDR)
                && (!(gapm_get_address_type() & GAPM_PRIV_CFG_PRIV_EN_BIT)))
        {
            gapc_set_local_addr(conidx, &p_actv_init->common.addr.addr[0]);
        }

        // Next action depends on initiating activity subtype
        switch (p_actv_init->common.subtype)
        {
        case GAPM_INIT_TYPE_NAME_DISC:
        {
            // Send a GAPC_GET_INFO_CMD in order to read the device name characteristic value
            struct gapc_get_info_cmd *p_cmd = KE_MSG_ALLOC(GAPC_GET_INFO_CMD,
                                              KE_BUILD_ID(TASK_GAPC, conidx), TASK_GAPM,
                                              gapc_get_info_cmd);

            // request peer device name.
            p_cmd->operation = GAPC_GET_PEER_NAME;

            // send command
            ke_msg_send(p_cmd);
        }
        break;

        case GAPM_INIT_TYPE_AUTO_CONN_EST:
        {
            // Sanity check
            ASSERT_ERR(p_actv_init->nb_auto_conn);

            // Decrease the number of connection to be established
            p_actv_init->nb_auto_conn--;

            // Check if there is more connections to be established
            if (p_actv_init->nb_auto_conn)
            {
                // Send a LE Extended Create Connection command to the controller
                gapm_init_send_hci_le_ext_create_con_cmd(p_actv_init, &p_actv_init->init_param);
            }
            else
            {
                // Clear automatic connection timeout timer
                ke_timer_clear(GAPM_AUTO_CONN_TO_IND, TASK_GAPM);

                // Activity can be stopped
                stop = true;
            }
        }
        break;

        case GAPM_INIT_TYPE_DIRECT_CONN_EST:
        {
            // Activity can be stopped
            stop = true;
        }
        break;

        default:
        {
            // Cannot happen
        }
        break;
        }
    }
    else
    {
        stop = true;
    }

    if (stop)
    {
        gapm_env.init_actv_idx = GAPM_ACTV_INVALID_IDX;

        // Operation is over
        gapm_actv_stopped((struct gapm_actv_tag *)p_actv_init, GAP_ERR_NO_ERROR);
    }

    return (stop);
}

/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handle automatic connection establishment timeout
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int gapm_auto_conn_to_ind_handler(ke_msg_id_t const msgid,
                                  void const *param,
                                  ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Check if an initiating activity has been started
    uint8_t init_actv_idx = gapm_env.init_actv_idx;

    if (init_actv_idx != GAPM_ACTV_INVALID_IDX)
    {
        // Retrieve initiating activity currently in use
        struct gapm_actv_init_tag *p_actv_init = (struct gapm_actv_init_tag *)gapm_env.actvs[init_actv_idx];
        // Send a GAPM_ACTIVITY_STOP_CMD message
        struct gapm_activity_stop_cmd *p_cmd = KE_MSG_ALLOC(GAPM_ACTIVITY_STOP_CMD,
                                               TASK_GAPM, TASK_GAPM,
                                               gapm_activity_stop_cmd);

        p_cmd->operation = GAPM_STOP_ACTIVITY;
        p_cmd->actv_idx  = init_actv_idx;

        // Send command
        ke_msg_send(p_cmd);

        p_actv_init->kept_status = GAP_ERR_TIMEOUT;
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of GAP controller command completed event.
 *  - For Name Request
 *  - Disconnection part of name request operation
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPM).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int gapc_cmp_evt_handler(ke_msg_id_t const msgid,
                         struct gapc_cmp_evt const *p_event,
                         ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Get index of started initiating activity
    uint8_t init_actv_idx = gapm_env.init_actv_idx;

    if (init_actv_idx != GAPM_ACTV_INVALID_IDX)
    {
        struct gapm_actv_init_tag *p_actv_init = (struct gapm_actv_init_tag *)gapm_env.actvs[init_actv_idx];

        // Keep operation status
        if (p_actv_init->kept_status == GAP_ERR_NO_ERROR)
        {
            p_actv_init->kept_status = RW_ERR_HCI_TO_HL(p_event->status);
        }

        // Check reason of command complete.
        switch (p_event->operation)
        {
        // Disconnection
        case GAPC_DISCONNECT:
        {
            gapm_env.init_actv_idx = GAPM_ACTV_INVALID_IDX;

            // Activity can be stopped
            gapm_actv_stopped((struct gapm_actv_tag *)p_actv_init,
                              p_actv_init->kept_status);
        }
        break;

        // Name request, device name received in GAPC_PEER_ATT_INFO_IND message
        case GAPC_GET_PEER_NAME:
        {
            // Disconnect the link
            struct gapc_disconnect_cmd *p_cmd = KE_MSG_ALLOC(GAPC_DISCONNECT_CMD,
                                                src_id, TASK_GAPM,
                                                gapc_disconnect_cmd);

            p_cmd->operation = GAPC_DISCONNECT;
            p_cmd->reason    = CO_ERROR_REMOTE_USER_TERM_CON;

            // Send command
            ke_msg_send(p_cmd);
        }
        break;

        default:
        {
        }
        break;
        }
    }

    // Message is consumed
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of name indication. Convey message to name requester.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPM).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int gapc_peer_att_info_ind_handler(ke_msg_id_t const msgid,
                                   struct gapc_peer_att_info_ind const *p_event,
                                   ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Get index of started initiating activity
    uint8_t init_actv_idx = gapm_env.init_actv_idx;

    if (gapm_env.init_actv_idx != GAPM_ACTV_INVALID_IDX)
    {
        // Get initiating activity
        struct gapm_actv_init_tag *p_actv_init = (struct gapm_actv_init_tag *)gapm_env.actvs[init_actv_idx];
        // Extract connection index from source id
        uint8_t conidx = KE_IDX_GET(src_id);

        // Send the read name to the application
        struct gapm_peer_name_ind *p_ind = KE_MSG_ALLOC_DYN(GAPM_PEER_NAME_IND,
                                           p_actv_init->common.requester, TASK_GAPM,
                                           gapm_peer_name_ind,
                                           p_event->info.name.length);

        // Fill parameters
        memcpy(&(p_ind->addr), &(gapc_get_bdaddr(conidx, SMPC_INFO_PEER)->addr), sizeof(bd_addr_t));
        p_ind->addr_type = gapc_get_bdaddr(conidx, SMPC_INFO_PEER)->addr_type;
        p_ind->name_len = p_event->info.name.length;
        memcpy(p_ind->name, p_event->info.name.value, p_ind->name_len);

        // Send the message
        ke_msg_send(p_ind);
    }

    // Message is consumed
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Common handler for following HCI command complete events:
 *  - LE Extended Create Connection Command (HCI_LE_EXT_CREATE_CON_CMD_OPCODE)
 *  - LE Create Connection Cancel Command (HCI_LE_CREATE_CON_CANCEL_CMD_OPCODE)
 *
 * @param[in] opcode    Operation code of received message
 * @param[in] param     Pointer to the parameters of the message.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int hci_le_cmd_cmp_evt_init_handler(uint16_t opcode, struct hci_basic_cmd_cmp_evt const *p_event)
{
    // Initiating activity structure
    struct gapm_actv_init_tag *p_actv_init;

    do
    {
        // Find activity for which the command complete event has been received
        if (!gapm_actv_retrieve_cmd_cmp_evt((struct gapm_actv_tag **)&p_actv_init, opcode))
        {
            break;
        }

        // Call the proper handler based on current activity state
        switch (p_actv_init->common.state)
        {
        case GAPM_ACTV_STARTING:
        {
            hci_le_cmd_cmp_evt_starting_handler(p_actv_init, opcode, p_event);
        }
        break;

        case GAPM_ACTV_STARTED:
        {
            hci_le_cmd_cmp_evt_started_handler(p_actv_init, opcode, p_event);
        }
        break;

        case GAPM_ACTV_STOPPING:
        {
            hci_le_cmd_cmp_evt_stopping_handler(p_actv_init, opcode, p_event);
        }
        break;

        default:
        {
            // Nothing to do
        }
        break;
        }
    }
    while (0);

    return (KE_MSG_CONSUMED);
}

#endif //(BLE_CENTRAL)

/// @} GAPM_INIT
