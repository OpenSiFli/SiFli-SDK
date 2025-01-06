/**
 ****************************************************************************************
 *
 * @file llc_le_ping.c
 *
 * @brief Handles the LE Ping feature.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup LLC_LE_PING
 * @{
 ****************************************************************************************
 */
#include "rwip_config.h"

#if (BLE_CENTRAL || BLE_PERIPHERAL)
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "co_utils.h"    // For bit field manipulation
#include "co_math.h"     // for co_min

#include "ke_msg.h"      // Kernel message
#include "ke_timer.h"    // Kernel timer to trigger ping request PDU

#include "llc_int.h"     // Internal LLC API
#include "llc_llcp.h"    // Internal LLCP API

#include "hci.h"         // For HCI handler
#include "btdm_patch.h"

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// LE Ping Operation structure definition
/*@TRACE*/
struct llc_op_le_ping_ind
{
    /// Procedure information
    llc_procedure_t                 proc;
};



/*
 * DEFINES
 ****************************************************************************************
 */
// make conversion between slot time and 10ms timers
#define CONVERT_10MS_TO_SLOT(value) (((uint32_t) (value)) << 4)
#define CONVERT_SLOT_TO_10MS(value) (((uint32_t) (value)) >> 4)

// make conversion between double slot time and 10ms timers
#define CONVERT_10MS_TO_2SLOT(value) (((uint32_t) (value)) << 3)
#define CONVERT_2SLOT_TO_10MS(value) (((uint32_t) (value)) >> 3)


/// Local Encryption Procedure state machine (Master only)
/*@TRACE*/
enum llc_le_ping_state
{
    /// Start LE Ping Procedure
    LLC_LOC_PING_PROC_START,
    /// Wait for LLCP_PING_RSP PDU
    LLC_LOC_WAIT_PING_RSP,
};
/*
 * MACROS
 ****************************************************************************************
 */


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/*
 * LOCAL FUNCTIONS DECLARATIONS
 ****************************************************************************************
 */

__STATIC void llc_le_ping_proc_continue(uint8_t link_id, uint8_t state, uint8_t status);
__STATIC void llc_le_ping_proc_err_cb(uint8_t link_id, uint8_t error_type, void *param);
__STATIC void llc_ll_ping_req_pdu_send(uint8_t link_id);
__STATIC void llc_ll_ping_rsp_pdu_send(uint8_t link_id);

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * Continue execution of local procedure
 *
 * @param[in] link_id Link identifier
 * @param[in] state   Expected state of the procedure
 * @param[in] status  Status of the operation
 ****************************************************************************************
 */
__STATIC void llc_le_ping_proc_continue(uint8_t link_id, uint8_t state, uint8_t status)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(PROC_CONTINUE_PATCH_TYPE, LLC_LE_PING_PROC_CONTINUE_C_FUN_BIT, link_id, state, status);
    /// retrieve procedure parameters
    struct llc_op_le_ping_ind *param = (struct llc_op_le_ping_ind *) llc_proc_get(link_id, LLC_PROC_LOCAL);
    bool finished = false;

    // check that current procedure state equals to expected state given in parameter
    if (llc_proc_state_get(&param->proc) != state)
    {
        finished = true;
    }
    else
    {
        switch (llc_proc_state_get(&param->proc))
        {
        case LLC_LOC_PING_PROC_START:
        {
            // Send LLCP_PING_REQ pdu
            llc_ll_ping_req_pdu_send(link_id);
            // Start local LLCP exchange Timer
            llc_proc_timer_set(link_id, LLC_PROC_LOCAL, true);
        }
        break;

        case LLC_LOC_WAIT_PING_RSP:
        {
            finished = true;
        }
        break;

        default:
        {
            ASSERT_INFO_FORCE(0, link_id, llc_proc_state_get(&param->proc));
        }
        break;
        }
    }

    if (finished)
    {
        // Clear local LLCP exchange Timer
        llc_proc_timer_set(link_id, LLC_PROC_LOCAL, false);
        // unregister procedure
        llc_proc_unreg(link_id, LLC_PROC_LOCAL);
    }
}


/**
 * Local procedure callback used to inform if an unexpected error is raised during procedure execution
 *
 * @param[in] link_id     Link Identifier
 * @param[in] error_type  Error type (@see enum llc_error_type)
 * @param[in] param       Parameter according to error type:
 *   - LLC_ERR_DISCONNECT:          reason
 *   - LLC_ERR_LLCP_UNKNOWN_RSP:    struct ll_unknown_rsp*
 *   - LLC_ERR_LLCP_REJECT_IND:     struct ll_reject_ind*
 *   - LLC_ERR_LLCP_REJECT_IND_EXT: struct ll_reject_ext_ind*
 */
__STATIC void llc_le_ping_proc_err_cb(uint8_t link_id, uint8_t error_type, void *param)
{
    //Get environment pointer
    switch (error_type)
    {
    // link disconnection occurs
    case LLC_ERR_DISCONNECT:
    {
        uint8_t reason = *((uint8_t *) param);
        llc_le_ping_proc_continue(link_id, LLC_LOC_WAIT_PING_RSP, reason);
    }
    break;
    case LLC_ERR_LLCP_UNKNOWN_RSP:
    {
        struct ll_unknown_rsp *rsp = (struct ll_unknown_rsp *) param;
        if (rsp->unk_type == LL_PING_REQ_OPCODE)
        {
            // expected message, accept it as a response
            llc_le_ping_proc_continue(link_id, LLC_LOC_WAIT_PING_RSP, CO_ERROR_NO_ERROR);
            break;
        }
    }
    // no break;
    case LLC_ERR_LLCP_REJECT_EXT_IND:
    case LLC_ERR_LLCP_REJECT_IND:
    {
        // do nothing, ignore
    } break;
    default:
    {
        // not expected at all
        ASSERT_INFO_FORCE(0, link_id, error_type);
    }
    break;
    }
}


/**
 ****************************************************************************************
 * @brief Sends the LE Ping request PDU.
 *
 * @param[in] link_id Link Identifier
 ****************************************************************************************
 */
__STATIC void llc_ll_ping_req_pdu_send(uint8_t link_id)
{
    struct ll_ping_req pdu;

    pdu.op_code = LL_PING_REQ_OPCODE;

    llc_llcp_send(link_id, (union llcp_pdu *) &pdu, NULL);
}

/**
 ****************************************************************************************
 * @brief Sends the LE Ping response PDU.
 *
 * @param[in] link_id Link Identifier
 ****************************************************************************************
 */
__STATIC void llc_ll_ping_rsp_pdu_send(uint8_t link_id)
{
    struct ll_ping_rsp pdu;

    pdu.op_code = LL_PING_RSP_OPCODE;

    llc_llcp_send(link_id, (union llcp_pdu *) &pdu, NULL);
}

/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */


/*
 ****************************************************************************************
 * LLCP Handlers
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handles the reception of a LLCP ping request
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
int ll_ping_req_handler(uint8_t link_id, struct ll_enc_rsp *pdu, uint16_t event_cnt)
{
    // just send back ping response, nothing more to do
    llc_ll_ping_rsp_pdu_send(link_id);

    return (CO_ERROR_NO_ERROR);
}

/**
 ****************************************************************************************
 * @brief Handles the reception of a LLCP ping response
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
int ll_ping_rsp_handler(uint8_t link_id, struct ll_enc_rsp *pdu, uint16_t event_cnt)
{
    if (llc_proc_id_get(link_id, LLC_PROC_LOCAL) == LLC_PROC_LE_PING)
    {
        // expected message, accept it as a response
        llc_le_ping_proc_continue(link_id, LLC_LOC_WAIT_PING_RSP, CO_ERROR_NO_ERROR);
    }
    return (CO_ERROR_NO_ERROR);
}


/*
 ****************************************************************************************
 * HCI Handlers
 ****************************************************************************************
 */



/**
 ****************************************************************************************
 * @brief Handles the LE Ping Operation Start request
 ****************************************************************************************
 */
int hci_rd_auth_payl_to_cmd_handler(uint8_t link_id, struct hci_rd_auth_payl_to_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;
    // allocate the status event message
    struct hci_rd_auth_payl_to_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, param->conhdl, HCI_RD_AUTH_PAYL_TO_CMD_OPCODE, hci_rd_auth_payl_to_cmd_cmp_evt);

    // check if state is Free or in disconnected state
    if (llc_is_disconnecting(link_id))
    {
        // Nothing to do
    }
    else
    {
        // The authenticated payload timeout is expressed in units of 10 ms
        event->auth_payl_to = llc_env[link_id]->le_ping.auth_payl_to;
        status = CO_ERROR_NO_ERROR;
    }

    // fill event parameters
    event->status = status;
    event->conhdl = param->conhdl;
    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the command HCI write authentication payload timeout
 ****************************************************************************************
 */
int hci_wr_auth_payl_to_cmd_handler(uint8_t link_id, struct hci_wr_auth_payl_to_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];

    // allocate the status event message
    struct hci_wr_auth_payl_to_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, param->conhdl, HCI_WR_AUTH_PAYL_TO_CMD_OPCODE, hci_wr_auth_payl_to_cmd_cmp_evt);

    do
    {

        // check if state is Free or in disconnected state
        if (llc_is_disconnecting(link_id))
            break;

#if (BT_53)
        // Authenticated_Payload_Timeout shall be equal to or greater than connInterval x connSubrateFactor x (1 + connPeripheralLatency). Units in double slots
        if (CONVERT_10MS_TO_2SLOT(param->auth_payl_to) < (llc_env_ptr->con_params.interval * (llc_env_ptr->con_params.latency + 1U) * llc_env_ptr->con_params.subrate))
            break;
#else // !(BT_53)
        // Authenticated_Payload_Timeout shall be equal to or greater than connInterval x (1 + connSlaveLatency). Units in double slots
        if (CONVERT_10MS_TO_2SLOT(param->auth_payl_to) < (llc_env_ptr->con_params.interval * (llc_env_ptr->con_params.latency + 1U)))
            break;
#endif // !(BT_53)

        // set the authenticated payload timeout
        status = llc_le_ping_set(link_id, param->auth_payl_to);

    }
    while (0);

    // fill event parameters
    event->status = status;
    event->conhdl = param->conhdl;
    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}

/*
 ****************************************************************************************
 * Local Messages Handlers
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @brief Handles the LE Ping Operation Start request
 ****************************************************************************************
 */
int llc_op_le_ping_ind_handler(ke_msg_id_t const msgid, struct llc_op_le_ping_ind *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Current message status
    int msg_status = KE_MSG_CONSUMED;
    uint8_t link_id = KE_IDX_GET(dest_id);

    // Check if state in disconnected state
    if (llc_is_disconnecting(link_id))
    {
        // LLC is IDLE, discard the message
    }
    // check if another local procedure is on-going
    else if (llc_proc_id_get(link_id, LLC_PROC_LOCAL) != LLC_PROC_NONE)
    {
        // process this message later
        msg_status = KE_MSG_SAVED;
    }
    else
    {
        msg_status = KE_MSG_NO_FREE;

        // store local procedure
        llc_proc_reg(link_id, LLC_PROC_LOCAL, &(param->proc));

        // execute local Encryption procedure
        llc_le_ping_proc_continue(link_id, LLC_LOC_PING_PROC_START, CO_ERROR_NO_ERROR);
    }

    return (msg_status);
}


/**
 ****************************************************************************************
 * @brief Handles the authenticated payload near timeout to start LE_PING REQ/RSP procedure
 ****************************************************************************************
 */
int llc_auth_payl_nearly_to_handler(ke_msg_id_t const msgid, void *param,
                                    ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t link_id = KE_IDX_GET(dest_id);

    struct llc_env_tag *llc_env_ptr = llc_env[link_id];

    // Check if state in disconnected state
    if (llc_is_disconnecting(link_id))
    {
        // LLC is IDLE, discard the message
    }
    // check if link encrypted or not, else nothing to do.
    else if (GETB(llc_env_ptr->link_info, LLC_INFO_LINK_ENCRYPTED))
    {
        // prepare ping procedure
        struct llc_op_le_ping_ind *ping = KE_MSG_ALLOC(LLC_OP_LE_PING_IND, dest_id, dest_id, llc_op_le_ping_ind);
        llc_proc_init(&ping->proc, LLC_PROC_LE_PING, llc_le_ping_proc_err_cb);
        llc_proc_state_set(&ping->proc, link_id, LLC_LOC_PING_PROC_START);
        ke_msg_send(ping);
    }

    return (KE_MSG_CONSUMED);
}



/**
 ****************************************************************************************
 * @brief Handles the authenticated payload timeout to inform host that no encrypted payload received
 ****************************************************************************************
 */
int llc_auth_payl_real_to_handler(ke_msg_id_t const msgid, void *param,
                                  ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t link_id = KE_IDX_GET(dest_id);
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];

    // Check if state in disconnected state
    if (llc_is_disconnecting(link_id))
    {
        // LLC is IDLE, discard the message
    }
    // check if link encrypted or not, else nothing to do.
    else if (GETB(llc_env_ptr->link_info, LLC_INFO_LINK_ENCRYPTED))
    {
        uint16_t conhdl = BLE_LINKID_TO_CONHDL(link_id);
        // Inform host that no authenticated payload has been received
        struct hci_auth_payl_to_exp_evt *event =
            KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_AUTH_PAYL_TO_EXP_EVT_CODE, hci_auth_payl_to_exp_evt);
        event->conhdl = conhdl;
        hci_send_2_host(event);

        // restart the timer
        llc_le_ping_restart(link_id);
    }

    return (KE_MSG_CONSUMED);
}


/*
 ****************************************************************************************
 * Exported functions
 ****************************************************************************************
 */


/*
 *
 * @startuml  llc_le_ping.png
 * participant HCI
 * participant ke_timer
 * participant LLC
 * participant LLC_LLCP
 * participant LLD
 * LLD --> LLC: Data encrypted
 * LLC -> LLC: llc_le_ping_restart()
 * ... No encrypted data ...
 * ke_timer --> LLC : LLC_AUTH_PAYL_NEARLY_TO
 * LLC -> LLC : llc_auth_payl_nearly_to_handler()
 * activate LLC
 *     note over LLC : Create LE Ping Procedure
 *     LLC --> LLC: LLC_OP_LE_PING_IND
 * deactivate LLC
 * LLC -> LLC : llc_op_le_ping_ind_handler()
 * activate LLC
 *     hnote over LLC : LOC_PROC = busy
 *     LLC -> LLC : llc_le_ping_proc_continue(START)
 *     activate LLC
 *         note over LLC #aqua: state = WAIT_PING_RSP
 *         LLC --> LLD: LLCP_PING_REQ
 *     deactivate LLC
 * deactivate LLC
 * alt LLCP_PING_RSP received from peer
 * LLD --> LLC: LLCP_PING_RSP
 * LLC -> LLC : ll_ping_rsp_handler()
 * activate LLC
 *     LLC -> LLC : llc_le_ping_proc_continue(PING_RSP)
 *     activate LLC
 *         hnote over LLC : LOC_PROC = idle
 *         LLC -> LLC: llc_le_ping_restart()
 *     deactivate LLC
 * deactivate LLC
 * else peer is 4.0 device, receive LLCP_UNKNOWN_RSP packet
 * LLD --> LLC_LLCP: LLCP_UNKNOWN_RSP
 * LLC_LLCP -> LLC_LLCP : ll_unknown_rsp_handler()
 * activate LLC_LLCP
 *     LLC_LLCP -> LLC : llc_le_ping_proc_err_cb()
 *     activate LLC
 *         LLC -> LLC : llc_le_ping_proc_continue(PING_RSP)
 *         activate LLC
 *             hnote over LLC : LOC_PROC = idle
 *             LLC -> LLC: llc_le_ping_restart()
 *             LLC_LLCP \- LLC:
 *         deactivate LLC
 *     deactivate LLC
 * deactivate LLC_LLCP
 * else peer never answer with encrypted data
 * ... No encrypted data ...
 * ke_timer --> LLC : LLC_AUTH_PAYL_REAL_TO
 * LLC -> LLC : llc_auth_payl_real_to_handler()
 * activate LLC
 *     note over LLC : Inform host that authenticated payload timer expires.\nthen restart procedure
 *     LLC --> HCI: HCI_AUTH_PAYL_TO_EXP_EVT
 *     LLC -> LLC : llc_le_ping_proc_continue(PING_RSP)
 *     activate LLC
 *         hnote over LLC : LOC_PROC = idle
 *         LLC -> LLC: llc_le_ping_restart()
 *     deactivate LLC
 * deactivate LLC
 * end
 * @enduml
 *
 */

void llc_le_ping_restart(uint8_t link_id)
{
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];

    // check if link encrypted or not, else nothing to do.
    if (GETB(llc_env_ptr->link_info, LLC_INFO_LINK_ENCRYPTED))
    {
        ke_task_id_t llc_id =  KE_BUILD_ID(TASK_LLC, link_id);

        // set authenticated payload TO timers
        ke_timer_set(LLC_AUTH_PAYL_REAL_TO,   llc_id, 10 * llc_env_ptr->le_ping.auth_payl_to);
        ke_timer_set(LLC_AUTH_PAYL_NEARLY_TO, llc_id, 10 * llc_env_ptr->le_ping.auth_near_payl_to);
    }
}

uint8_t llc_le_ping_set(uint8_t link_id, uint16_t auth_payl_to)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];

    ASSERT_INFO(llc_env_ptr != NULL, link_id, auth_payl_to);

    do
    {
        llc_con_params_t *con_params;
        uint32_t margin;

        if (llc_env_ptr == NULL)
            break;

        con_params = &(llc_env_ptr->con_params);

#if (BT_53)
        /*
         * Determine a margin for auth_near_payl_to. Reduced for BT5.3 due to requirements on HCI/CCO/BI-39,
         * in which an unsupported range above minimum valid timeout is not tolerated.
         *
         * Apply a nominal margin which is less than minimum valid timeout, based on 4 x connection interval, or connection latency.
         */
        margin = CONVERT_2SLOT_TO_10MS((uint32_t) con_params->interval * co_max(4, con_params->latency));

#else // !(BT_53)
        {
            uint32_t auth_payl_to_in_slot, min_auth_payl_to_in_slot;

            auth_payl_to_in_slot     = CONVERT_10MS_TO_SLOT(auth_payl_to);
            min_auth_payl_to_in_slot = ((uint32_t) con_params->interval) * ((uint32_t)(con_params->latency + 1)) * 2;

            // First consider that minimal margin is at least 4 x min TO
            margin = (min_auth_payl_to_in_slot << 2);

            // Then check that margin is not greater than provided "authenticated payload timeout"
            if (margin > auth_payl_to_in_slot)
            {
                margin = auth_payl_to_in_slot / min_auth_payl_to_in_slot;
                margin *= min_auth_payl_to_in_slot;
            }

            // Convert margin in 10ms timer
            margin = CONVERT_SLOT_TO_10MS(margin);
        }
#endif // !(BT_53)

        // Ensure that it's at least 10ms.
        if (margin == 0)
        {
            margin = 1;
        }

        // Check if auth
        if (margin == auth_payl_to)
        {
            status = CO_ERROR_UNSUPPORTED;
            break;
        }

        // store computed values
        llc_env_ptr->le_ping.auth_payl_to      = auth_payl_to;
        llc_env_ptr->le_ping.auth_near_payl_to = auth_payl_to - ((uint16_t)margin);

        // restart the timer
        llc_le_ping_restart(link_id);
        status = CO_ERROR_NO_ERROR;

    }
    while (0);

    return status;
}


#endif // (BLE_CENTRAL || BLE_PERIPHERAL)

/// @} LLC_LE_PING
