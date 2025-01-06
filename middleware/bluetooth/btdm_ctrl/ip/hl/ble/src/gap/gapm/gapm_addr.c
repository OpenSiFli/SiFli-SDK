/**
 ****************************************************************************************
 *
 * @file gapm_addr.c
 *
 * @brief Generic Access Profile Manager - Address manager module.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup GAPM_ADDR Generic Access Profile Manager - Address manager module.
 * @ingroup GAPM
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#include <string.h>
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
 * ENUMERATIONS
 ****************************************************************************************
 */

/// State machine for address generation procedure
enum gapm_addr_state
{
    /// Idle, ready to generate and set a new random address
    GAPM_ADDR_STATE_IDLE = 0,
    /// Waiting for reception of GAPM_CMP_EVT for GAPM_GEN_RAND_ADDR operation
    GAPM_ADDR_STATE_GEN_RAND_ADDR,
    /// Waiting for reception of command complete event for HCI LE Set Random Address Command
    GAPM_ADDR_STATE_SET_RAND_ADDR,
    /// Waiting for reception of command complete event for HCI LE Set Advertising Set Random Address Command
    GAPM_ADDR_STATE_SET_ADV_SET_RAND_ADDR,
};

/// Event for address generation procedure
enum gapm_addr_evt
{
    /// Reception of address generation request
    GAPM_ADDR_EVT_GEN_RAND_ADDR_REQ = 0,
    /// Reception of GAPM_CMP_EVT for GAPM_GEN_RAND_ADDR operation
    GAPM_ADDR_EVT_GEN_RAND_ADDR_CMP,
    /// Reception of command complete event for HCI LE Set Random Address Command
    GAPM_ADDR_EVT_SET_RAND_ADDR_CMP,
    /// Reception of command complete event for HCI LE Set Advertising Set Random Address Command
    GAPM_ADDR_EVT_SET_ADV_SET_RAND_ADDR_CMP,
};

/*
 * MACROS
 ****************************************************************************************
 */

/*
 * LOCAL FUNCTIONS DECLARATION
 ****************************************************************************************
 */

__STATIC void gapm_addr_fsm(uint8_t event);

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

#if (BLE_OBSERVER)
/**
 ****************************************************************************************
 * @brief Send HCI LE Set Random Address command to the controller.
 *
 * @param[in] p_actv    Activity for which random address has to be set.
 ****************************************************************************************
 */
__STATIC void gapm_addr_send_hci_le_set_rand_addr_cmd(uint8_t *p_bd_addr)
{
    // Allocate HCI command message
    struct hci_le_set_rand_addr_cmd *p_cmd = KE_MSG_ALLOC(HCI_COMMAND,
            0, HCI_LE_SET_RAND_ADDR_CMD_OPCODE,
            hci_le_set_rand_addr_cmd);

    // Copy the address
    memcpy(&p_cmd->rand_addr.addr[0], p_bd_addr, BD_ADDR_LEN);

    // Send the command
    hci_send_2_controller(p_cmd);
}
#endif //(BLE_OBSERVER)

#if (BLE_BROADCASTER)
/**
 ****************************************************************************************
 * @brief Send a LE Set Advertising Set Random Address command over HCI. The command
 * complete event is handled in hci_le_cmd_cmp_evt_adv_handler function.
 *
 * @param[in] p_actv_adv    Pointer to the activity structure
 ****************************************************************************************
 */
__STATIC void gapm_addr_send_hci_le_set_adv_set_rand_addr_cmd(struct gapm_actv_tag *p_actv)
{
    // Allocate HCI command message
    struct hci_le_set_adv_set_rand_addr_cmd *p_cmd = KE_MSG_ALLOC(HCI_COMMAND,
            0, HCI_LE_SET_ADV_SET_RAND_ADDR_CMD_OPCODE,
            hci_le_set_adv_set_rand_addr_cmd);

    // Use local advertising identifier as advertising handle
    p_cmd->adv_hdl = p_actv->idx;
    // Copy the address
    memcpy(&p_cmd->rand_addr.addr[0], &p_actv->addr.addr[0], BD_ADDR_LEN);

    // Send the command
    hci_send_2_controller(p_cmd);
}
#endif //(BLE_BROADCASTER)

#if (BLE_CENTRAL) || (BLE_OBSERVER)
/**
 ****************************************************************************************
 * @brief Send GAPM_DEV_BDADDR_IND message to the application. Provide the random address used
 * for one activity.
 *
 * @param[in] p_actv    Pointer to the structure activity for which address has been generated
 ****************************************************************************************
 */
__STATIC void gapm_addr_send_dev_bdaddr_ind(struct gapm_actv_tag *p_actv)
{
    // Indication sent to the requester
    struct gapm_dev_bdaddr_ind *p_ind = KE_MSG_ALLOC(GAPM_DEV_BDADDR_IND,
                                        p_actv->requester, TASK_GAPM,
                                        gapm_dev_bdaddr_ind);

    // Fill up the parameters
    memcpy(&p_ind->addr.addr.addr[0], &p_actv->addr.addr[0], GAP_BD_ADDR_LEN);
    p_ind->addr.addr_type = ADDR_RAND;
    p_ind->actv_idx = p_actv->idx;

    // Send the message
    ke_msg_send(p_ind);
}
#endif //(BLE_CENTRAL) || (BLE_OBSERVER)

/**
 ****************************************************************************************
 * @brief Send GAPM_GEN_RAND_ADDR_CMD message so that it can be handled as an operation.
 *
 * @param[in] p_actv    Pointer to the structure activity for which address has been generated
 ****************************************************************************************
 */
__STATIC void gapm_addr_send_gen_rand_addr(struct gapm_actv_tag *p_actv)
{
    // Allocate a GAPM_GEN_RAND_ADDR_CMD message
    struct gapm_gen_rand_addr_cmd *p_cmd = KE_MSG_ALLOC(GAPM_GEN_RAND_ADDR_CMD,
                                           TASK_GAPM, TASK_GAPM,
                                           gapm_gen_rand_addr_cmd);

    p_cmd->operation = GAPM_GEN_RAND_ADDR;
    p_cmd->rnd_type = (p_actv->own_addr_type == GAPM_GEN_NON_RSLV_ADDR) ? GAP_NON_RSLV_ADDR : GAP_RSLV_ADDR;

    // Send request
    ke_msg_send(p_cmd);
}

/**
 ****************************************************************************************
 * @brief Called during address renewal procedure in order to get the next started
 * activity using a non-static random address for which address has to be renewed.
 *
 * @param[in, out] p_param      Pointer to the internal GAPM_ADDR_RENEW_CMD parameters.
 ****************************************************************************************
 */
__STATIC void gapm_addr_renew_next_addr(struct gapm_addr_renew_cmd *p_param)
{
    // Counter
    uint8_t i;

    // Loop over the remaining activities and detect those for which address must be renewed
    for (i = p_param->actv_idx; i < GAPM_ACTV_NB; i++)
    {
        // Get activity structure
        struct gapm_actv_tag *p_actv = gapm_env.actvs[i];

        // Check if activity exists, is started and is using a non-static random address
        if (p_actv
                && (p_actv->type != GAPM_ACTV_TYPE_PER_SYNC)
                && (p_actv->own_addr_type != GAPM_STATIC_ADDR)
                && (p_actv->state >= GAPM_ACTV_CREATED)
                && (p_actv->state <= GAPM_ACTV_STOPPING))
        {
            // If address is renewed for at least one activity, restart RENEW_TO timer
            // (p_param->actv_idx == 0 indicates that we are looking for the first
            // activity with a non-static random address)
            if (p_param->actv_idx == 0)
            {
                // Restart timer
                ke_timer_set(GAPM_ADDR_RENEW_TO_IND, TASK_GAPM, ((uint32_t)gapm_env.renew_dur) * 100);
            }

#if (BLE_OBSERVER)
            // Initiating or scanning activities shall use the same random BD addresses
            if ((p_actv->type == GAPM_ACTV_TYPE_INIT) || (p_actv->type == GAPM_ACTV_TYPE_SCAN))
            {
                // If first initiating or scanning activity met, keep its index as other activities
                // met after will use the same address
                if (p_param->init_scan_actv_idx == GAPM_ACTV_INVALID_IDX)
                {
                    p_param->init_scan_actv_idx = i;
                }
                else
                {
                    struct gapm_actv_tag *p_created_actv = gapm_env.actvs[p_param->init_scan_actv_idx];

                    // Use the same address for both activities
                    memcpy(&p_actv->addr.addr[0], &p_created_actv->addr.addr[0], GAP_BD_ADDR_LEN);

                    // Inform the host about the BD address used for the activity
                    gapm_addr_send_dev_bdaddr_ind(p_actv);

                    // Look for next activity
                    continue;
                }
            }
#endif //(BLE_OBSERVER)

            // Leave the loop
            break;
        }
    }

    // Check if address generation has been requested for one activity
    if (i != GAPM_ACTV_NB)
    {
        // Set activity index of next activity index for which address could be renewed
        p_param->actv_idx = i + 1;

        // Keep index of activity for which address will be generated and set
        gapm_env.actv_idx = i;

        // Update FSM
        gapm_addr_fsm(GAPM_ADDR_EVT_GEN_RAND_ADDR_REQ);
    }
    else
    {
        gapm_env.actv_idx = GAPM_ACTV_INVALID_IDX;

        // End air operation
        gapm_send_complete_evt(GAPM_OP_AIR, GAP_ERR_NO_ERROR);
    }
}

/**
 ****************************************************************************************
 * @brief Finite state machine for GAPM address manager. Perform next operation based on
 * indicated event and current state.
 *
 * @param[in] event     Event that triggers update of the state machine.
 ****************************************************************************************
 */
__STATIC void gapm_addr_fsm(uint8_t event)
{
    // Get activity structure
    struct gapm_actv_tag *p_actv = gapm_env.actvs[gapm_env.actv_idx];

    switch (event)
    {
    case (GAPM_ADDR_EVT_GEN_RAND_ADDR_REQ):
    {
        // Send an address generation command
        gapm_addr_send_gen_rand_addr(p_actv);

        // Update address manager state machine
        gapm_env.gapm_addr_state = GAPM_ADDR_STATE_GEN_RAND_ADDR;
    }
    break;

    case (GAPM_ADDR_EVT_GEN_RAND_ADDR_CMP):
    {
#if (BLE_BROADCASTER)
        // We have to set the address, the command to be used depends on kind of activity
        if (p_actv->type == GAPM_ACTV_TYPE_ADV)
        {
            // Send a HCI LE Set Advertising Set Random Address command to the controller
            gapm_addr_send_hci_le_set_adv_set_rand_addr_cmd(p_actv);

            // Update address manager state machine
            gapm_env.gapm_addr_state = GAPM_ADDR_STATE_SET_ADV_SET_RAND_ADDR;
        }
        else
#endif //(BLE_BROADCASTER)
        {
#if (BLE_OBSERVER)
            // Send a HCI LE Set Random Address command to the controller
            gapm_addr_send_hci_le_set_rand_addr_cmd(&p_actv->addr.addr[0]);

            // Update address manager state machine
            gapm_env.gapm_addr_state = GAPM_ADDR_STATE_SET_RAND_ADDR;
#endif //(BLE_OBSERVER)
        }
    }
    break;

#if (BLE_OBSERVER)
    case (GAPM_ADDR_EVT_SET_RAND_ADDR_CMP):
#endif //(BLE_OBSERVER)
#if (BLE_BROADCASTER)
    case (GAPM_ADDR_EVT_SET_ADV_SET_RAND_ADDR_CMP):
#endif //(BLE_BROADCASTER)
    {
        // A new address can be set/generated
        gapm_env.gapm_addr_state = GAPM_ADDR_STATE_IDLE;

        // Verify current air operation
        if (gapm_get_operation(GAPM_OP_AIR) == GAPM_RENEW_ADDR)
        {
            // Retrieve GAPM_RENEW_ADDR_CMD message
            struct gapm_addr_renew_cmd *p_cmd =
                (struct gapm_addr_renew_cmd *)gapm_get_operation_ptr(GAPM_OP_AIR);

            // Generate next address
            gapm_addr_renew_next_addr(p_cmd);
        }
        else
        {
            gapm_env.actv_idx = GAPM_ACTV_INVALID_IDX;

            // Call the callback for indication that address has been generated and set
            // in the controller
            gapm_actv_rand_addr_set_ind(p_actv);
        }
    }
    break;

    default:
    {
        // Not possible
    } break;
    }
}

/*
 * EXTERNAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

void gapm_addr_init(void)
{
    // Clean module state
    gapm_env.gapm_addr_state = GAPM_ADDR_STATE_IDLE;

    // Stop timer for address renewal
    ke_timer_clear(GAPM_ADDR_RENEW_TO_IND, TASK_GAPM);
}

#if (BLE_OBSERVER)
bool gapm_is_addr_type_valid(uint8_t addr_type)
{
    // Counter
    uint8_t i;
    // Indicate if address type is valid
    bool addr_type_valid = true;

    // Loop over the activities
    for (i = 0; i < GAPM_ACTV_NB; i++)
    {
        // Get activity structure
        struct gapm_actv_tag *p_actv = gapm_env.actvs[i];

        // Check if activity exists
        if (p_actv
                && ((p_actv->type == GAPM_ACTV_TYPE_INIT) ||
                    (p_actv->type == GAPM_ACTV_TYPE_SCAN)))
        {
            // All created initiating and scanning activities shall share the same address type
            if (p_actv->own_addr_type != addr_type)
            {
                addr_type_valid = false;
                break;
            }
        }
    }

    // Return if address is valid or not
    return (addr_type_valid);
}
#endif //(BLE_OBSERVER)

void gapm_addr_set_rand_addr(struct gapm_actv_tag *p_actv)
{
    // Event for FSM, consider by default that a new address has to be generated
    uint8_t event = GAPM_ADDR_EVT_GEN_RAND_ADDR_REQ;

    // !!!!! It is assumed here that the provided activity is not a periodic sync activity !!!!!
    // !!!!! It is assumed here that the activity uses a least a random static address !!!!!
    // !!!!! It is assumed here that provided kind of address in valid (always the case for advertising
    //       activities, checked with gapm_is_addr_type_valid function for initiating and scanning
    //       activities)
    // !!!!! It is assumed here that the function is called as part of execution of an air operation
    //       so that it does not enter in conflict with address renewal procedure !!!!!
    // !!!!! It is assumed that the activity is in CREATING state !!!!!

    // Check kind of activity
    //     - If activity is an advertising activity it can have its own random address so a
    // new address can be generated if it is not required to use the random static address
    //     - Initiating activity and scanning activity must share the same random address.
    // If a random address has already been generated for one of these activities, we can reuse
    // the same.

    switch (p_actv->type)
    {
#if (BLE_BROADCASTER)
    case (GAPM_ACTV_TYPE_ADV):
    {
        if (p_actv->own_addr_type == GAPM_STATIC_ADDR)
        {
            // Use static random address, generation not needed
            memcpy(&p_actv->addr.addr[0], &gapm_env.addr.addr[0], GAP_BD_ADDR_LEN);
            event = GAPM_ADDR_EVT_GEN_RAND_ADDR_CMP;
        }
    }
    break;
#endif //(BLE_BROADCASTER)

    default:
    {
#if (BLE_OBSERVER)
        if (p_actv->own_addr_type != GAPM_STATIC_ADDR)
        {
            // Already created initiating or scanning activity
            struct gapm_actv_tag *p_created_actv = NULL;
            // Counter
            uint8_t cnt;

            // Loop over the activities
            for (cnt = 0; cnt < GAPM_ACTV_NB; cnt++)
            {
                // Get activity structure
                struct gapm_actv_tag *p_loop_actv = gapm_env.actvs[cnt];

                // Check if activity exists
                if (p_loop_actv
                        && ((p_loop_actv->type == GAPM_ACTV_TYPE_INIT) ||
                            (p_loop_actv->type == GAPM_ACTV_TYPE_SCAN))
                        && ((p_loop_actv->state == GAPM_ACTV_CREATED) ||
                            (p_loop_actv->state == GAPM_ACTV_STARTED)))
                {
                    p_created_actv = p_loop_actv;
                    break;
                }
            }

            // Reuse the address already in use
            if (p_created_actv)
            {
                // Use the same address for both activities
                memcpy(&p_actv->addr.addr[0], &p_created_actv->addr.addr[0], GAP_BD_ADDR_LEN);
                // Even if it is not needed, set the random address in order to always inform
                // the caller module in the same way
                event = GAPM_ADDR_EVT_GEN_RAND_ADDR_CMP;

                // Inform the host about the BD address used for the activity
                gapm_addr_send_dev_bdaddr_ind(p_actv);
            }
        }
        else
        {
            // Use static random address, generation not needed
            memcpy(&p_actv->addr.addr[0], &gapm_env.addr.addr[0], GAP_BD_ADDR_LEN);
            event = GAPM_ADDR_EVT_GEN_RAND_ADDR_CMP;
        }
#endif //(BLE_OBSERVER)
    }
    break;
    }

    if (p_actv->own_addr_type != GAPM_STATIC_ADDR)
    {
        // If timer for address renewal not yet started, start it
        if (!ke_timer_active(GAPM_ADDR_RENEW_TO_IND, TASK_GAPM))
        {
            // Start timer
            ke_timer_set(GAPM_ADDR_RENEW_TO_IND, TASK_GAPM, ((uint32_t)gapm_env.renew_dur) * 100);
        }
    }

    gapm_env.actv_idx = p_actv->idx;

    // Update FSM
    gapm_addr_fsm(event);
}

/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handler for internally sent GAPM_ADDR_RENEW_CMD message. Initiates
 * generation of a new resolvable random address that will be pushed to the controller
 * once generated.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int gapm_addr_renew_cmd_handler(ke_msg_id_t const msgid,
                                struct gapm_addr_renew_cmd *p_param,
                                ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // List of handler supported operations
    enum gapm_operation supp_ops[] = {GAPM_RENEW_ADDR,
                                      GAPM_NO_OP
                                     };
    // Check if operation can be executed
    int msg_status = gapm_process_op(GAPM_OP_AIR, p_param, supp_ops);

    // Operation can be handled
    if (msg_status == KE_MSG_NO_FREE)
    {
        // Look for the first activity for which an address has to be generated
        gapm_addr_renew_next_addr(p_param);
    }

    return (msg_status);
}

/**
 ****************************************************************************************
 * @brief Handle expiration of timer indicating that resolvable random address
 * generated by the host has to be updated.
 * Generates and sends a GAPM_ADDR_RENEW_CMD to GAPM task so that the address
 * generation can be handled as an air operation.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int gapm_addr_renew_to_ind_handler(ke_msg_id_t const msgid,
                                   void const *p_param,
                                   ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Start a new air operation, needed so that renewal of address does not enter in
    // conflict with create/start/stop/delete activity operation.
    struct gapm_addr_renew_cmd *p_cmd = KE_MSG_ALLOC(GAPM_ADDR_RENEW_CMD,
                                        TASK_GAPM, TASK_GAPM,
                                        gapm_addr_renew_cmd);

    // Set operation
    p_cmd->operation = GAPM_RENEW_ADDR;
    // Set index of first activity for which address could be renewed
    p_cmd->actv_idx = 0;
    // No created init or scan activity found yet
    p_cmd->init_scan_actv_idx = GAPM_ACTV_INVALID_IDX;

    ke_msg_send(p_cmd);

    // Message is consumed
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handle reception of GAPM_DEV_BDADDR_IND message.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int gapm_dev_bdaddr_ind_handler(ke_msg_id_t const msgid,
                                struct gapm_dev_bdaddr_ind *p_ind,
                                ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Message status
    int msg_status = KE_MSG_CONSUMED;

    // Check if address is expected
    if (gapm_env.gapm_addr_state == GAPM_ADDR_STATE_GEN_RAND_ADDR)
    {
        // Activity for which address has been generated
        struct gapm_actv_tag *p_actv = gapm_env.actvs[gapm_env.actv_idx];

        if (p_actv)
        {
            // Keep the address in the activity structure
            memcpy(&p_actv->addr.addr[0], &p_ind->addr.addr.addr[0], GAP_BD_ADDR_LEN);

            // Set the activity identifier in the indication
            p_ind->actv_idx = p_actv->idx;

            // And forward it to the task responsible for the activity
            ke_msg_forward(p_ind, p_actv->requester, src_id);
            msg_status = KE_MSG_NO_FREE;
        }
    }

    // Return message status
    return (msg_status);
}

/**
 ****************************************************************************************
 * @brief Handle reception of GAPM_CMP_EVT message for GAPM_GEN_RAND_ADDR operation.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int gapm_cmp_evt_handler(ke_msg_id_t const msgid,
                         struct gapm_cmp_evt const *p_cmp_evt,
                         ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // A new random address has been generated
    if (p_cmp_evt->operation == GAPM_GEN_RAND_ADDR)
    {
        // Check that we were waiting for this event
        if (gapm_env.gapm_addr_state == GAPM_ADDR_STATE_GEN_RAND_ADDR)
        {
            // Update FSM
            gapm_addr_fsm(GAPM_ADDR_EVT_GEN_RAND_ADDR_CMP);
        }
    }

    // Message is consumed
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Common handler for following HCI command complete events:
 *  - LE Set Random Address (HCI_LE_SET_RAND_ADDR_CMD_OPCODE)
 *  - LE Set Advertising Set Random Address (HCI_LE_SET_ADV_SET_RAND_ADDR_CMD_OPCODE)
 *
 * @param[in] opcode    Operation code of received message
 * @param[in] param     Pointer to the parameters of the message.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int hci_le_cmd_cmp_evt_addr_handler(uint16_t opcode, struct hci_basic_cmd_cmp_evt const *p_event)
{
    switch (opcode)
    {
#if (BLE_OBSERVER)
    case (HCI_LE_SET_RAND_ADDR_CMD_OPCODE):
    {
        if (gapm_env.gapm_addr_state == GAPM_ADDR_STATE_SET_RAND_ADDR)
        {
            gapm_addr_fsm(GAPM_ADDR_EVT_SET_RAND_ADDR_CMP);
        }
    }
    break;
#endif //(BLE_OBSERVER)

#if (BLE_BROADCASTER)
    case (HCI_LE_SET_ADV_SET_RAND_ADDR_CMD_OPCODE):
    {
        if (gapm_env.gapm_addr_state == GAPM_ADDR_STATE_SET_ADV_SET_RAND_ADDR)
        {
            gapm_addr_fsm(GAPM_ADDR_EVT_SET_ADV_SET_RAND_ADDR_CMP);
        }
    }
    break;
#endif //(BLE_BROADCASTER)

    default:
    {

    } break;
    }

    return (KE_MSG_CONSUMED);
}

/// @} GAPM_ADDR
