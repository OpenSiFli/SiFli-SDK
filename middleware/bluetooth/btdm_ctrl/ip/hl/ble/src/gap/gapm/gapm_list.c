/**
 ****************************************************************************************
 *
 * @file gapm_list.c
 *
 * @brief Generic Access Profile Manager list manager module.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup GAPM_LIST Generic Access Profile Manager list manager
 * @ingroup GAPM
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */


#include "rwip_config.h"
#include "rwble_hl.h"
#include "rwip.h"            // Common API to retrieve device parameters

#include <string.h>

#include "co_error.h"
#include "co_bt.h"
#include "co_math.h"
#include "co_version.h"
#include "co_utils.h"        // core utility functions

#include "gap.h"
#include "gapm_task.h"
#include "gapm_int.h"

#include "ke_mem.h"

#include "hci.h"

#include "ke_timer.h"

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
 * @brief Send a LE Add Device To White List command over HCI. Command complete event
 * handled in hci_le_cmd_cmp_evt_list_handler.
 *
 * @param[in] p_bd_addr     Pointer to the device address and device address type
 ****************************************************************************************
 */
__STATIC void gapm_list_send_hci_le_add_dev_to_wlst_cmd(struct gap_bdaddr *p_bd_addr)
{
    // Allocate HCI command message
    struct hci_le_add_dev_to_wlst_cmd *p_cmd = KE_MSG_ALLOC(HCI_COMMAND,
            0, HCI_LE_ADD_DEV_TO_WLST_CMD_OPCODE,
            hci_le_add_dev_to_wlst_cmd);

    // Fill command parameters
    p_cmd->dev_addr_type = p_bd_addr->addr_type;
    memcpy(&p_cmd->dev_addr.addr[0], &p_bd_addr->addr.addr[0], BD_ADDR_LEN);

    // Send the command
    hci_send_2_controller(p_cmd);
}

/**
 ****************************************************************************************
 * @brief Send a LE Add Device To Resolving List command over HCI. Command complete event
 * handled in hci_le_cmd_cmp_evt_list_handler.
 *
 * @param[in] p_ral_info    Pointer to resolving list information
 ****************************************************************************************
 */
__STATIC void gapm_list_send_hci_le_add_dev_to_rslv_list_cmd(struct gap_ral_dev_info *p_ral_info)
{
    // Allocate HCI command message
    struct hci_le_add_dev_to_rslv_list_cmd *p_cmd = KE_MSG_ALLOC(HCI_COMMAND, 0, HCI_LE_ADD_DEV_TO_RSLV_LIST_CMD_OPCODE,
            hci_le_add_dev_to_rslv_list_cmd);

    // Fill command parameters
    p_cmd->peer_id_addr_type = p_ral_info->addr.addr_type;
    memcpy(&p_cmd->peer_id_addr.addr[0], &p_ral_info->addr.addr.addr[0], BD_ADDR_LEN);
    memcpy(&p_cmd->peer_irk.key[0],      &p_ral_info->peer_irk[0],       GAP_KEY_LEN);
    memcpy(&p_cmd->local_irk.key[0],     &p_ral_info->local_irk[0],      GAP_KEY_LEN);

    // Send the command
    hci_send_2_controller(p_cmd);
}

/**
 ****************************************************************************************
 * @brief
 ****************************************************************************************
 */
__STATIC void gapm_list_send_hci_le_set_priv_mode_cmd(struct gap_ral_dev_info *p_ral_info)
{
    struct hci_le_set_priv_mode_cmd *p_cmd = KE_MSG_ALLOC(HCI_COMMAND, 0, HCI_LE_SET_PRIV_MODE_CMD_OPCODE,
            hci_le_set_priv_mode_cmd);

    // Fill up the parameters */
    p_cmd->peer_addr_type = p_ral_info->addr.addr_type;
    memcpy(&p_cmd->peer_addr.addr[0], &p_ral_info->addr.addr.addr[0], BD_ADDR_LEN);
    p_cmd->priv_mode      = p_ral_info->priv_mode;

    // Send the message
    hci_send_2_controller(p_cmd);
}

/**
 ****************************************************************************************
 * @brief Send a LE Add Device To Periodic Advertiser List command over HCI. Command complete event
 * handled in hci_le_cmd_cmp_evt_list_handler.
 *
 * @param[in] p_bd_addr     Pointer to the advertiser address
 * @param[in] addr_type     Advertiser address type
 * @param[in] adv_sid       Advertising SID
 ****************************************************************************************
 */
__STATIC void gapm_list_send_hci_le_add_dev_to_per_adv_list_cmd(struct gapm_period_adv_addr_cfg *p_pal_info)
{
    // Allocate HCI command message
    struct hci_le_add_dev_to_per_adv_list_cmd *p_cmd = KE_MSG_ALLOC(HCI_COMMAND,
            0, HCI_LE_ADD_DEV_TO_PER_ADV_LIST_CMD_OPCODE,
            hci_le_add_dev_to_per_adv_list_cmd);

    // Fill command parameters
    p_cmd->adv_addr_type = p_pal_info->addr.addr_type;
    p_cmd->adv_sid       = p_pal_info->adv_sid;
    memcpy(&p_cmd->adv_addr.addr[0], &p_pal_info[0], BD_ADDR_LEN);

    // Send the command
    hci_send_2_controller(p_cmd);
}

/**
 ****************************************************************************************
 * @brief Send a LE Clear White List command over HCI. Command complete event
 * handled in hci_le_cmd_cmp_evt_list_handler.
 ****************************************************************************************
 */
__STATIC void gapm_list_send_hci_le_clear_wlst_cmd(void)
{
    hci_basic_cmd_send_2_controller(HCI_LE_CLEAR_WLST_CMD_OPCODE);
}

/**
 ****************************************************************************************
 * @brief Send a LE Clear Resolving List command over HCI. Command complete event
 * handled in hci_le_cmd_cmp_evt_list_handler.
 ****************************************************************************************
 */
__STATIC void gapm_list_send_hci_le_clear_rslv_list_cmd(void)
{
    hci_basic_cmd_send_2_controller(HCI_LE_CLEAR_RSLV_LIST_CMD_OPCODE);
}

/**
 ****************************************************************************************
 * @brief Send a LE Clear Periodic Advertiser List command over HCI. Command complete event
 * handled in hci_le_cmd_cmp_evt_list_handler.
 ****************************************************************************************
 */
__STATIC void gapm_list_send_hci_le_clear_per_adv_list_cmd(void)
{
    hci_basic_cmd_send_2_controller(HCI_LE_CLEAR_PER_ADV_LIST_CMD_OPCODE);
}

/**
 ****************************************************************************************
 * @brief Send either HCI LE Read Local Resolvable Address or HCI LE Read Peer Resolvable
 * Address command to the controller
 *
 * @param[in] opcode    Command opcode (HCI_LE_RD_LOC_RSLV_ADDR_CMD_OPCODE or HCI_LE_RD_PEER_RSLV_ADDR_CMD_OPCODE)
 * @param[in] p_bdaddr  Pointer to either the local identity or the peer identity
 ****************************************************************************************
 */
__STATIC void gapm_list_send_hci_le_rd_rslv_addr_cmd(uint16_t opcode, struct gap_bdaddr *p_bdaddr)
{
    struct hci_le_rd_loc_rslv_addr_cmd *p_cmd = KE_MSG_ALLOC(HCI_COMMAND,
            0, opcode,
            hci_le_rd_loc_rslv_addr_cmd);

    // Fill up the parameters */
    p_cmd->peer_id_addr_type = p_bdaddr->addr_type;
    memcpy(&p_cmd->peer_id_addr.addr[0], &p_bdaddr->addr.addr[0], BD_ADDR_LEN);

    // Send the message
    hci_send_2_controller(p_cmd);
}

/**
 ****************************************************************************************
 * @brief Add next entry in either the white list or the resolving list or the periodic
 * advertiser list.
 *
 * @param[in, out] p_param  Pointer to the received GAPM_LIST_SET_CMD message that contains
 *                          the list of entries to be added.
 *                          'size' parameter is decremented. The operation will be considered
 *                          as over if its value is 0 upon reception of the command complete
 *                          event from the controller.
 ****************************************************************************************
 */
__STATIC void gapm_list_add_next_entry(struct gapm_list_set_cmd *p_param)
{
    uint8_t index = p_param->size - 1;

    switch (p_param->operation)
    {
    case GAPM_SET_WL:
    {
        struct gapm_list_set_wl_cmd *p_param_wl = (struct gapm_list_set_wl_cmd *)p_param;

        // Send LE Add Device To White List HCI command
        gapm_list_send_hci_le_add_dev_to_wlst_cmd(&p_param_wl->wl_info[index]);
    }
    break;

    case GAPM_SET_RAL:
    {
        struct gapm_list_set_ral_cmd *p_param_ral = (struct gapm_list_set_ral_cmd *)p_param;

        // Send LE Add Device To Resolving List HCI command
        gapm_list_send_hci_le_add_dev_to_rslv_list_cmd(&p_param_ral->ral_info[index]);
    }
    break;

    case GAPM_SET_PAL:
    {
        struct gapm_list_set_pal_cmd *p_param_pal = (struct gapm_list_set_pal_cmd *)p_param;

        // Send LE Add Device To Periodic Advertising List HCI command
        gapm_list_send_hci_le_add_dev_to_per_adv_list_cmd(&p_param_pal->pal_info[index]);
    }
    break;

    default:
    {
        // Cannot happen
    } break;
    }

    // Decrease number of entries
    p_param->size--;
}

/**
 ****************************************************************************************
 * @brief Sent the request list size to the application
 *
 * @param[in] operation     Operation code provided by application, indicate list for which
 *                          size is returned
 * @param[in] size          List size
 ****************************************************************************************
 */
__STATIC void gapm_list_send_list_size_ind(uint8_t operation, uint8_t size)
{
    // Allocate a GAPM_LIST_SIZE_IND message
    struct gapm_list_size_ind *p_ind = KE_MSG_ALLOC(GAPM_LIST_SIZE_IND,
                                       gapm_get_requester(GAPM_OP_CFG), TASK_GAPM,
                                       gapm_list_size_ind);

    // Fill up the parameters
    p_ind->operation = operation;
    p_ind->size      = size;

    // Send the message
    ke_msg_send(p_ind);
}

/**
 ****************************************************************************************
 * @brief Send a GAPM_LIST_SEND_RAL_ADDR_IND message to the application. It contains either
 * the requested local resolvable address or the peer resolvable address.
 *
 * @param[in] operation     Operation set in GAPM_GET_RAL_ADDR_CMD
                            (GAPM_GET_RAL_LOC_ADDR or GAPM_GET_RAL_PEER_ADDR)
 * @param[in] p_bd_addr     Local or peer resolvable address
 ****************************************************************************************
 */
__STATIC void gapm_list_send_ral_addr_ind(uint8_t operation, struct bd_addr const *p_bd_addr)
{
    // Allocate a GAPM_RAL_ADDR_IND message
    struct gapm_ral_addr_ind *p_ind = KE_MSG_ALLOC(GAPM_RAL_ADDR_IND,
                                      gapm_get_requester(GAPM_OP_CFG), TASK_GAPM,
                                      gapm_ral_addr_ind);

    // Fill up the parameters */
    p_ind->operation = operation;
    memcpy(&p_ind->addr, &p_bd_addr->addr[0], BD_ADDR_LEN);

    /* send the message indication */
    ke_msg_send(p_ind);
}

/**
 ****************************************************************************************
 * @brief Handles request of setting content of lists:
 *  - GAPM_SET_WL: Set content of white list
 *  - GAPM_SET_RAL: Set content of resolving list
 *  - GAPM_SET_PAL: Set content of periodic advertiser list
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
extern int gapm_activity_suspend_all_handler_internal();
int gapm_list_set_cmd_handler(ke_msg_id_t const msgid,
                              struct gapm_list_set_cmd *p_param,
                              ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // List of handler supported operations
    enum gapm_operation supp_ops[] = {GAPM_SET_WL,
                                      GAPM_SET_RAL,
                                      GAPM_SET_PAL,
                                      GAPM_NO_OP
                                     };
    int msg_status, stop_status = 0;

    if (gapm_env.pause_act_flg == 0)
    {
        stop_status = gapm_activity_suspend_all_handler_internal();
    }

    // Check if operation can be executed
    if (1 != stop_status)
    {
        msg_status = gapm_process_op(GAPM_OP_CFG, p_param, supp_ops);
    }
    else
    {
        msg_status = KE_MSG_SAVED;
    }

    rt_kprintf("set cmd 0x%x, stop_status:%d, msg_status:%d\n", p_param->operation, stop_status, msg_status);


    // Operation can be handled
    if (msg_status == KE_MSG_NO_FREE)
    {
        // Start by clearing content of the indicated list
        // Next steps of the operation will be started in hci_le_cmd_cmp_evt_list_handler function
        switch (p_param->operation)
        {
        case GAPM_SET_WL:
        {
#if (BLE_OBSERVER)
            // Consider that White List is empty
            gapm_env.nb_dev_wl = 0;
#endif //(BLE_OBSERVER)

            gapm_list_send_hci_le_clear_wlst_cmd();
        }
        break;

        case GAPM_SET_RAL:
        {
            gapm_list_send_hci_le_clear_rslv_list_cmd();
        }
        break;

        case GAPM_SET_PAL:
        {
            gapm_list_send_hci_le_clear_per_adv_list_cmd();
        }
        break;

        default:
        {
            // Cannot happen
        } break;
        }
    }

    return (msg_status);
}

/**
 ****************************************************************************************
 * @brief Handles request of setting content of lists:
 *  - GAPM_GET_RAL_LOC_ADDR: Set content of white list
 *  - GAPM_GET_RAL_PEER_ADDR: Set content of resolving list
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int gapm_get_ral_addr_cmd_handler(ke_msg_id_t const msgid,
                                  struct gapm_get_ral_addr_cmd *p_param,
                                  ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // List of handler supported operations
    enum gapm_operation supp_ops[] = {GAPM_GET_RAL_LOC_ADDR,
                                      GAPM_GET_RAL_PEER_ADDR
                                     };
    // Check if operation can be executed
    int msg_status = gapm_process_op(GAPM_OP_CFG, p_param, supp_ops);

    // Operation can be handled
    if (msg_status == KE_MSG_NO_FREE)
    {
        uint16_t opcode = (p_param->operation == GAPM_GET_RAL_LOC_ADDR) ?
                          HCI_LE_RD_LOC_RSLV_ADDR_CMD_OPCODE : HCI_LE_RD_PEER_RSLV_ADDR_CMD_OPCODE;

        // Send the read command
        gapm_list_send_hci_le_rd_rslv_addr_cmd(opcode, &p_param->peer_identity);
    }

    return (msg_status);
}

/*
 * HCI EVENT HANDLERS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief  Common handler for following HCI command complete events:
 *  - LE Clear White List Command (HCI_LE_CLEAR_WLST_CMD_OPCODE)
 *  - LE Clear Resolving List Command (HCI_LE_CLEAR_RSLV_LIST_CMD_OPCODE)
 *  - LE Clear Periodic Advertiser Command (HCI_LE_CLEAR_PER_ADV_LIST_CMD_OPCODE)
 *  - LE Add Device To White List Command (HCI_LE_ADD_DEV_TO_WLST_CMD_OPCODE)
 *  - LE Add Device To Resolving List Command (HCI_LE_ADD_DEV_TO_RSLV_LIST_CMD_OPCODE)
 *  - LE Add Device To Periodic Advertiser Command (HCI_LE_ADD_DEV_TO_PER_ADV_LIST_CMD_OPCODE)
 *
 * @param[in] opcode    Operation code of received message
 * @param[in] param     Pointer to the parameters of the message.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
extern int gapm_activity_resume_all_handler_internal();
int hci_le_cmd_cmp_evt_list_handler(uint16_t opcode, struct hci_basic_cmd_cmp_evt const *p_event)
{
    struct gapm_list_set_cmd *p_param;
    uint8_t error;
    do
    {
        // Get current air operation
        uint8_t operation = gapm_get_operation(GAPM_OP_CFG);

        // Check if current air operation is correct
        if ((operation != GAPM_SET_WL)
                && (operation != GAPM_SET_PAL)
                && (operation != GAPM_SET_RAL))
        {
            // Finish operation execution with an error: unexpected response received
            gapm_send_complete_evt(GAPM_OP_CFG, GAP_ERR_PROTOCOL_PROBLEM);
            break;
        }

        // Map the controller error status on a GAP error status
        error = RW_ERR_HCI_TO_HL(p_event->status);

        if (error != GAP_ERR_NO_ERROR)
        {
            // Send the GAPM_CMP_EVT message containing the status
            gapm_send_complete_evt(GAPM_OP_CFG, error);
            break;
        }

#if (BLE_OBSERVER)
        if (opcode == HCI_LE_ADD_DEV_TO_WLST_CMD_OPCODE)
        {
            gapm_env.nb_dev_wl++;
        }
#endif //(BLE_OBSERVER)

        // Retrieve content of GAPM_LIST_SET_CMD message
        p_param = (struct gapm_list_set_cmd *)gapm_get_operation_ptr(GAPM_OP_CFG);

        if ((operation == GAPM_SET_RAL) && (opcode == HCI_LE_ADD_DEV_TO_RSLV_LIST_CMD_OPCODE))
        {
            // Send HCI LE Set Privacy Mode command
            gapm_list_send_hci_le_set_priv_mode_cmd(&((struct gapm_list_set_ral_cmd *)p_param)->ral_info[p_param->size]);
            break;
        }

        // If no entry has to be added in the list, clean operation
        if (p_param->size == 0)
        {
            // Send the GAPM_CMP_EVT message containing the status
            gapm_send_complete_evt(GAPM_OP_CFG, GAP_ERR_NO_ERROR);
#ifndef GAPM_SLIM
            gapm_activity_resume_all_handler_internal();
#endif // GAPM_SLIM
            break;
        }

        // Add next device
        gapm_list_add_next_entry(p_param);
    }
    while (0);

    // Message is consumed
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handle HCI Read White List Size command complete event
 *
 * @param[in] opcode    Operation code of received message
 * @param[in] param     Pointer to the parameters of the message.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int hci_le_rd_wlst_size_cmd_cmp_evt_handler(uint16_t opcode, struct hci_le_rd_wlst_size_cmd_cmp_evt const *p_param)
{
    // Current process state
    uint8_t state = ke_state_get(TASK_GAPM);

    if (state != GAPM_DEVICE_SETUP)
    {
        // Check if there is no protocol issues.
        if (gapm_get_operation(GAPM_OP_CFG) != GAPM_GET_WLIST_SIZE)
        {
            // Finish operation execution with an error: unexpected response received
            gapm_send_complete_evt(GAPM_OP_CFG, GAP_ERR_PROTOCOL_PROBLEM);
        }
        else
        {
            // Send white list size indication
            if (p_param->status == CO_ERROR_NO_ERROR)
            {
                gapm_list_send_list_size_ind(GAPM_GET_WLIST_SIZE, p_param->wlst_size);
            }

            // Finish operation execution
            gapm_send_complete_evt(GAPM_OP_CFG, RW_ERR_HCI_TO_HL(p_param->status));
        }
    }

    // Message is consumed
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handle HCI Read Resolving List Size command complete event
 *
 * @param[in] opcode    Operation code of received message
 * @param[in] param     Pointer to the parameters of the message.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int hci_le_rd_ral_size_cmd_cmp_evt_handler(uint16_t opcode, struct hci_le_rd_rslv_list_size_cmd_cmp_evt const *p_param)
{
    // Current process state
    uint8_t state = ke_state_get(TASK_GAPM);

    if (state != GAPM_DEVICE_SETUP)
    {
        // Check if there are no protocol issues.
        if (gapm_get_operation(GAPM_OP_CFG) != GAPM_GET_RAL_SIZE)
        {
            // Finish operation execution with an error: unexpected response received
            gapm_send_complete_evt(GAPM_OP_CFG, GAP_ERR_PROTOCOL_PROBLEM);
        }
        else
        {
            // Send size indication
            if (p_param->status == CO_ERROR_NO_ERROR)
            {
                gapm_list_send_list_size_ind(GAPM_GET_RAL_SIZE, p_param->size);
            }

            // Finish operation execution
            gapm_send_complete_evt(GAPM_OP_CFG, RW_ERR_HCI_TO_HL(p_param->status));
        }
    }

    // Message is consumed
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handle HCI Read Periodic Advertiser List Size command complete event
 *
 * @param[in] opcode    Operation code of received message
 * @param[in] param     Pointer to the parameters of the message.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int hci_le_rd_pal_size_cmd_cmp_evt_handler(uint16_t opcode, struct hci_le_rd_per_adv_list_size_cmd_cmp_evt const *p_param)
{
    // Current process state
    uint8_t state = ke_state_get(TASK_GAPM);

    if (state != GAPM_DEVICE_SETUP)
    {
        // Check if there are no protocol issues.
        if (gapm_get_operation(GAPM_OP_CFG) != GAPM_GET_PAL_SIZE)
        {
            // Finish operation execution with an error: unexpected response received
            gapm_send_complete_evt(GAPM_OP_CFG, GAP_ERR_PROTOCOL_PROBLEM);
        }
        else
        {
            // Send size indication
            if (p_param->status == CO_ERROR_NO_ERROR)
            {
                gapm_list_send_list_size_ind(GAPM_GET_PAL_SIZE, p_param->size);
            }

            // Finish operation execution
            gapm_send_complete_evt(GAPM_OP_CFG, RW_ERR_HCI_TO_HL(p_param->status));
        }
    }

    // Message is consumed
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Read resolving list address complete event handler.
 *
 * @param[in] opcode    Operation code of received message
 * @param[in] param     Pointer to the parameters of the message.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int hci_le_rd_ral_addr_cmd_cmp_evt_handler(ke_msg_id_t const msgid,
        struct hci_le_rd_peer_rslv_addr_cmd_cmp_evt const *p_param)
{
    // Current process state
    uint8_t state = ke_state_get(TASK_GAPM);

    if (state != GAPM_DEVICE_SETUP)
    {
        uint8_t operation = gapm_get_operation(GAPM_OP_CFG);

        // Check if there are no protocol issues.
        if ((operation != GAPM_GET_RAL_PEER_ADDR) &&
                (operation != GAPM_GET_RAL_LOC_ADDR))
        {
            // Finish operation execution with an error: unexpected response received
            gapm_send_complete_evt(GAPM_OP_CFG, GAP_ERR_PROTOCOL_PROBLEM);
        }
        else
        {
            if (p_param->status == CO_ERROR_NO_ERROR)
            {
                gapm_list_send_ral_addr_ind(operation, &p_param->peer_rslv_addr);
            }

            // Finish operation execution
            gapm_send_complete_evt(GAPM_OP_CFG, RW_ERR_HCI_TO_HL(p_param->status));
        }
    }

    // Message in consumed
    return (KE_MSG_CONSUMED);
}

/// @} GAPM_LIST
