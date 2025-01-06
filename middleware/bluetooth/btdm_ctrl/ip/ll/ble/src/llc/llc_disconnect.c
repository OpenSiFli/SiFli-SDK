/**
 ****************************************************************************************
 *
 * @file llc_disconnect.c
 *
 * @brief Handles the Disconnection procedure.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup LLC_DISC
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (BLE_CENTRAL || BLE_PERIPHERAL)
#include <stdint.h>
#include <stdbool.h>

#include "co_utils.h"    // For bit field manipulation

#include "ke_msg.h"      // Kernel message
#include "ke_timer.h"    // Kernel timers

#include "llc.h"        // LLC API
#include "llc_int.h"    // Internal LLC API
#include "llc_llcp.h"   // Internal LLCP API

#include "hci.h"        // For HCI handler
#if (BLE_ISO_PRESENT)
    #include "lli.h"        // LLI API
#endif // (BLE_ISO_PRESENT)
#include "lld.h"        // For stopping activity at Link driver level
#include "llm.h"        // To inform about disconnection
#include "btdm_patch.h"

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Disconnection operation indication structure definition
/*@TRACE*/
struct llc_op_disconnect_ind
{
    /// procedure information
    llc_procedure_t proc;
    /// Disconnection reason at LLCP level
    uint8_t reason;
    /// Local Procedure termination reason
    uint8_t loc_term_reason;
};


/*
 * DEFINES
 ****************************************************************************************
 */

/*@TRACE*/
enum llc_disconnect_state
{
    /// Start Disconnection procedure
    LLC_DISC_START,
    /// Wait for acknowledgment of disconnection PDU
    LLC_DISC_WAIT_TERM_ACK,
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
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */
__STATIC void llc_ll_terminate_ind_ack(uint8_t link_id, uint8_t op_code);

/**
 ****************************************************************************************
 * Send HCI CC Event message to inform host about link disconnection
 *
 * @param[in] link_id Link identifier
 * @param[in] status  Status of the operation
 * @param[in] reason  Disconnection reason
 ****************************************************************************************
 */
__STATIC void llc_disconnect_end(uint8_t link_id, uint8_t status, uint8_t reason)
{
    uint16_t conhdl = BLE_LINKID_TO_CONHDL(link_id);

    struct hci_disc_cmp_evt *disc_cc = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_DISC_CMP_EVT_CODE, hci_disc_cmp_evt);

    disc_cc->status  = status;
    disc_cc->conhdl  = conhdl;
    disc_cc->reason  = reason;

    // send the message
    hci_send_2_host(disc_cc);
}

/**
 ****************************************************************************************
 * @brief Sends the terminate indication pdu.
 *
 ****************************************************************************************
 */
__STATIC void llc_ll_terminate_ind_pdu_send(uint16_t link_id, uint8_t reason)
{
    //Get environment pointer
    struct ll_terminate_ind pdu;

    pdu.op_code = LL_TERMINATE_IND_OPCODE;
    pdu.err_code = reason;

    llc_llcp_send(link_id, (union llcp_pdu *) &pdu, llc_ll_terminate_ind_ack);
}

/**
 ****************************************************************************************
 * Continue execution of local procedure
 *
 * @param[in] link_id Link identifier
 * @param[in] status  Status of the operation
 ****************************************************************************************
 */
__STATIC void llc_disconnect_proc_continue(uint8_t link_id, uint8_t status)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(PROC_CONTINUE_PATCH_TYPE, LLC_DISCONNECT_PROC_CONTINUE_C_FUN_BIT, link_id, status);
    /// retrieve procedure parameters
    struct llc_op_disconnect_ind *param = (struct llc_op_disconnect_ind *) llc_proc_get(link_id, LLC_PROC_LOCAL);
    // Get environment pointer
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];
    bool finished = true;

    if (status == CO_ERROR_NO_ERROR)
    {
        switch (llc_proc_state_get(&param->proc))
        {
        /// Procedure started
        case LLC_DISC_START:
        {
            // mark that link is terminated
            llc_llcp_state_set(link_id, LLC_LLCP_DIR_BOTH, LLC_LLCP_TERMINATE);
            llc_proc_timer_set(link_id, LLC_PROC_REMOTE, false);

            // Store the disconnection reason
            llc_env_ptr->disc_reason = param->loc_term_reason;

            // send llcp message
            llc_ll_terminate_ind_pdu_send(link_id, param->reason);

            llc_proc_state_set(&param->proc, link_id, LLC_DISC_WAIT_TERM_ACK);

            // Ensure that local procedure is not paused
            llc_proc_timer_pause_set(link_id, LLC_PROC_LOCAL, false);
            // Enable LLCP Response timeout timer
            llc_proc_timer_set(link_id, LLC_PROC_LOCAL, true);


            finished = false;
        }
        break;

        /// Procedure started
        case LLC_DISC_WAIT_TERM_ACK:
        {
            // Immediately stop connection
            llc_disconnect(link_id, param->loc_term_reason, true);
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
        // unregister procedure
        llc_proc_unreg(link_id, LLC_PROC_LOCAL);
    }
}

/**
 ****************************************************************************************
 * Handles reception of LLCP_TERMINATE_IND baseband acknowledgment
 *
 * @param[in] link_id Link identifier
 * @param[in] op_code Operation code acknowledged
 ****************************************************************************************
 */
__STATIC void llc_ll_terminate_ind_ack(uint8_t link_id, uint8_t op_code)
{
    llc_disconnect_proc_continue(link_id, CO_ERROR_NO_ERROR);
}


/**
 * Local/Remote procedure callback used to inform if an unexpected error is raised during procedure execution
 *
 * @param[in] link_id     Link Identifier
 * @param[in] error_type  Error type (@see enum llc_error_type)
 * @param[in] param       Parameter according to error type:
 *   - LLC_ERR_DISCONNECT:          reason
 *   - LLC_ERR_LLCP_UNKNOWN_RSP:    struct ll_unknown_rsp*
 *   - LLC_ERR_LLCP_REJECT_IND:     struct ll_reject_ind*
 *   - LLC_ERR_LLCP_REJECT_IND_EXT: struct ll_reject_ext_ind*
 */
__STATIC void llc_disconnect_proc_err_cb(uint8_t link_id, uint8_t error_type, void *param)
{
    switch (error_type)
    {
    // link disconnection occurs
    case LLC_ERR_DISCONNECT:
    {
        uint8_t reason = *((uint8_t *) param);
        llc_disconnect_proc_continue(link_id, reason);
    }
    break;
    case LLC_ERR_LLCP_UNKNOWN_RSP:
    case LLC_ERR_LLCP_REJECT_EXT_IND:
    case LLC_ERR_LLCP_REJECT_IND:
    {
        // nothing to do, ignore.
    } break;
    default:
    {
        // not expected at all
        ASSERT_INFO_FORCE(0, link_id, error_type);
    }
    break;
    }
}


/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handles the reception of a LLCP terminate indication
 *
 * PlantUML procedure description
 *
 * @startuml
 * title : Link termination initiated by peer
 * participant LLC
 * participant LLD
 * LLD --> LLC:  LLCP_TERMINATE_IND
 * LLC -> LLC:  ll_terminate_ind_handler()
 * activate LLC
 * note over LLC: Clear procedure timers
 * LLC -> LLC:  llc_disconnect(reason)
 * activate LLC
 * note over LLC #aqua: Mark disconnection on-going\nStore Reason
 * LLC -> LLD: lld_con_stop()
 * note right LLD #lightgreen: See __lld_disc_ind_handler__
 * deactivate LLC
 * deactivate LLC
 * @enduml
 *
*  @param[in] link_id        Link identifier on which the pdu will be sent.
 * @param[in] pdu            LLCP PDU information received
 * @param[in] event_cnt      Event counter value when PDU has been received
 *
 * @return status code of handler:
 *    - CO_ERROR_NO_ERROR:               Nothing more to do
 *    - CO_ERROR_TERMINATED_MIC_FAILURE: Immediately disconnect the link
 *    - others:                          Send an LLCP_REJECT_IND or LLCP_REJECT_IND_EXT
 ****************************************************************************************
 */
uint8_t ll_terminate_ind_handler(uint8_t link_id, struct ll_terminate_ind *pdu, uint16_t event_cnt)
{
    //Get environment pointer
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];

    // Clears the LLCP response timeouts in case procedures are ongoing
    llc_proc_timer_set(link_id, LLC_PROC_LOCAL, false);
    llc_proc_timer_set(link_id, LLC_PROC_REMOTE, false);

    // Immediately stop connection (force next event to be executed if master receive the PDU
    llc_disconnect(link_id, pdu->err_code, !GETB(llc_env_ptr->link_info, LLC_INFO_MASTER_ROLE));

    return (CO_ERROR_NO_ERROR);
}



/**
 ****************************************************************************************
 * @brief Handles that link has been disconnected at LL driver level.
 *
 * PlantUML procedure description
 *
 * @startuml
 * title : Disconnect received from LLD
 * participant HCI
 * participant LLC
 * participant LLD
 * participant LLM
 * LLD --> LLC:  LLD_DISC_IND
 * LLC -> LLC:  lld_disc_ind_handler()
 * activate LLC
 *     note over LLC: Retrieve Error code
 *     note over LLC: Abort on-going local/remote operation
 *     LLC -> LLC:  llc_stop()
 *     activate LLC
 *     note over LLC: Set task state to FREE \n to restore pending operation\n in message queue
 *     LLC --> LLC: LLC_STOPPED
 *     deactivate LLC
 * deactivate LLC
 * LLC -> LLC:llc_stopped_handler
 * activate LLC
 *     LLC -> LLC:  llc_cleanup()
 *     note right : Clean environment variable
 *     LLC -> LLM: llm_link_disc()
 *     note right: Inform that conhdl is free
 *     LLC --> HCI: HCI_DISC_CMP_EVT
 * deactivate LLC
 * @enduml
 ****************************************************************************************
 */
int lld_disc_ind_handler(ke_msg_id_t const msgid, struct lld_disc_ind *param,
                         ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Current message status
    uint8_t link_id = KE_IDX_GET(dest_id);

    struct llc_env_tag *llc_env_ptr = llc_env[link_id];
    ASSERT_INFO(llc_env_ptr != NULL, link_id,  0);

    // Unexpected disconnection reason
    if (param->reason != CO_ERROR_CON_TERM_BY_LOCAL_HOST)
    {
        llc_env_ptr->disc_reason = param->reason;
    }

    // Indicate connection driver has stopped
    llc_env_ptr->con_stop = true;

    // inform local and remote procedure that disconnection occurs
    llc_proc_err_ind(link_id, LLC_PROC_LOCAL, LLC_ERR_DISCONNECT,  &(llc_env_ptr->disc_reason));
    ASSERT_INFO(llc_proc_id_get(link_id, LLC_PROC_LOCAL) == LLC_PROC_NONE,  link_id, llc_proc_id_get(link_id, LLC_PROC_LOCAL));

    llc_proc_err_ind(link_id, LLC_PROC_REMOTE, LLC_ERR_DISCONNECT, &(llc_env_ptr->disc_reason));
    ASSERT_INFO(llc_proc_id_get(link_id, LLC_PROC_REMOTE) == LLC_PROC_NONE, link_id, llc_proc_id_get(link_id, LLC_PROC_REMOTE));

    // stop the link
    llc_stop(link_id);

    return (KE_MSG_CONSUMED);
}


int llc_stopped_ind_handler(ke_msg_id_t const msgid, void *param, ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t link_id = KE_IDX_GET(dest_id);
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];
    ASSERT_INFO(llc_env_ptr != NULL, link_id,  0);

    // send the HCI message about disconnection
    llc_disconnect_end(link_id, CO_ERROR_NO_ERROR, llc_env_ptr->disc_reason);
#if (BLE_ISO_PRESENT)
    // Stop all ISO channels started for this connection
    lli_link_stop_ind(link_id, llc_env_ptr->disc_reason);
#endif // (BLE_ISO_PRESENT)
    // perform a LLC clean-up
    llc_cleanup(link_id, false);
    // inform LLM that link identifier is valid
    llm_link_disc(link_id);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the disconnection procedure indication message.
 *
 * PlantUML procedure description
 *
 * @startuml
 * title : Disconnect Operation
 * participant LLC
 * participant LLD
 * LLC --> LLC:  LLC_OP_DISCONNECT_IND
 * LLC -> LLC:  llc_op_disconnect_ind_handler()
 * activate LLC
 * LLC -> LLC:  llc_disconnect_proc_continue()
 * activate LLC
 * hnote over LLC: LOC PROC Busy
 * note over LLC #aqua: Mark disconnection on-going
 * LLC --> LLD: LLCP_TERMINATE_IND
 * note right LLD: wait for BB ACK
 * deactivate LLC
 * deactivate LLC
 * LLD --> LLC: LLD_LLCP_TX_CFM
 * LLC -> LLC:  llc_ll_terminate_ind_ack()
 * activate LLC
 * LLC -> LLC:  llc_disconnect_proc_continue()
 * activate LLC
 * hnote over LLC: LOC PROC Idle
 * LLC-> LLC:  llc_disconnect(reason)
 * activate LLC
 * note over LLC #aqua: Store Reason
 * LLC -> LLD: lld_con_stop()
 * note right LLD #lightgreen: See __lld_disc_ind_handler__
 * deactivate LLC
 * deactivate LLC
 * deactivate LLC
 * @enduml
 ****************************************************************************************
 */
int llc_op_disconnect_ind_handler(ke_msg_id_t const msgid, struct llc_op_disconnect_ind *param,
                                  ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Current message status
    int msg_status = KE_MSG_CONSUMED;
    uint8_t link_id = KE_IDX_GET(dest_id);
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];

    // Check if state in disconnected state
    if (llc_is_disconnecting(link_id))
    {
        // LLC is IDLE, discard the message
    }
    else
    {
        // Check if encryption start procedure ongoing
        if (!GETB(llc_env_ptr->link_info, LLC_INFO_ENC_START))
        {
            // local procedure on-going abort it to force disconnection
            if (llc_proc_id_get(link_id, LLC_PROC_LOCAL) != LLC_PROC_NONE)
            {
                // inform local and remote procedure that disconnection occurs
                llc_proc_err_ind(link_id, LLC_PROC_LOCAL, LLC_ERR_DISCONNECT, &(param->loc_term_reason));
                ASSERT_INFO(llc_proc_id_get(link_id, LLC_PROC_LOCAL) == LLC_PROC_NONE,  link_id, llc_proc_id_get(link_id, LLC_PROC_LOCAL));
            }

            msg_status = KE_MSG_NO_FREE;

            // store local procedure
            llc_proc_reg(link_id, LLC_PROC_LOCAL, &(param->proc));
            // execute disconnection procedure
            llc_disconnect_proc_continue(link_id, CO_ERROR_NO_ERROR);
        }
        else
        {
            msg_status = KE_MSG_SAVED;
        }
    }
    return (msg_status);
}


/**
 ****************************************************************************************
 * @brief Handles the command HCI disconnect.
 *
 * PlantUML procedure description
 *
 * @startuml
 * title : Disconnect requested from host
 * participant HCI
 * participant LLC
 * HCI --> LLC: HCI_DISCONNECT_CMD
 * LLC -> LLC:  hci_disconnect_cmd_handler
 * activate LLC
 * note over LLC: Check if disconnection on-going?
 * alt no
 *     LLC --> LLC: LLC_OP_DISCONNECT_IND
 *     note right LLC #lightgreen: see __llc_op_disconnect_ind_handler__
 * else yes
 *     note over LLC #aqua: Nothing to do
 * end
 * LLC --> HCI: HCI_CMD_STAT_EVENT
 * deactivate LLC
 * @enduml
 *
 * @param[in] link_id Link Identifier
 * @param[in] param   Pointer to the parameters of the message.
 * @param[in] opcode  HCI Operation code
 ****************************************************************************************
 */
int hci_disconnect_cmd_handler(uint8_t link_id, struct hci_disconnect_cmd const *param, uint16_t opcode)
{
    // Command status
    uint8_t status = CO_ERROR_NO_ERROR;
    //Get environment pointer
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];

    // check if state is Free or in disconnected state
    if (llc_is_disconnecting(link_id))
    {
        status = CO_ERROR_COMMAND_DISALLOWED;
    }
    // ensure that host not currently processing request
    else if (GETB(llc_env_ptr->flow_ctrl, LLC_HCI_DISCONNECT_REQ))
    {
        status = CO_ERROR_CONTROLLER_BUSY;
    }
    else
    {
        ke_task_id_t llc_id = KE_BUILD_ID(TASK_LLC, link_id);
        // start disconnection procedure
        struct llc_op_disconnect_ind *disc = KE_MSG_ALLOC(LLC_OP_DISCONNECT_IND, llc_id, llc_id, llc_op_disconnect_ind);
        llc_proc_init(&disc->proc, LLC_PROC_DISCONNECT, llc_disconnect_proc_err_cb);
        llc_proc_state_set(&disc->proc, link_id, LLC_DISC_START);
        disc->reason        = param->reason;


        // Set the local procedure termination reason
        if (param->reason == CO_ERROR_REMOTE_USER_TERM_CON)
        {
            disc->loc_term_reason = CO_ERROR_CON_TERM_BY_LOCAL_HOST;
        }
        else
        {
            disc->loc_term_reason = param->reason;
        }

        ke_msg_send(disc);

        SETB(llc_env_ptr->flow_ctrl, LLC_HCI_DISCONNECT_REQ, true);
    }

    // Send the command status event
    llc_cmd_stat_send(link_id, opcode, status);

    return (KE_MSG_CONSUMED);
}

/*
 * PlantUML procedure description
 *
 * @startuml
 * title : Disconnect requested from controller
 * participant LLC
 * participant LLD
 * -> LLC:  llc_disconnect(reason)
 * activate LLC
 * note over LLC #aqua: Mark disconnection on-going\nStore Reason
 * LLC -> LLD: lld_con_stop()
 * note right LLD #lightgreen: See __lld_disc_ind_handler__
 * deactivate LLC
 * @enduml
 */
void llc_disconnect(uint8_t link_id, uint8_t reason, bool immediate)
{
    //Get environment pointer
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];

    // Ensure that driver is not stopped twice
    if (!llc_env_ptr->con_stop)
    {
        // Store the disconnection reason
        llc_env_ptr->disc_reason = reason;

        // Immediately stop connection
        lld_con_stop(link_id, immediate);

        // Indicate connection driver has stopped
        llc_env_ptr->con_stop = true;

        // mark that link is terminated
        llc_llcp_state_set(link_id, LLC_LLCP_DIR_BOTH, LLC_LLCP_TERMINATE);
    }
}

// Initiate the termination procedure
void llc_init_term_proc(uint8_t link_id, uint8_t reason)
{
    ke_task_id_t llc_id = KE_BUILD_ID(TASK_LLC, link_id);
    // start disconnection procedure
    struct llc_op_disconnect_ind *disc = KE_MSG_ALLOC(LLC_OP_DISCONNECT_IND, llc_id, llc_id, llc_op_disconnect_ind);
    llc_proc_init(&disc->proc, LLC_PROC_DISCONNECT, llc_disconnect_proc_err_cb);
    llc_proc_state_set(&disc->proc, link_id, LLC_DISC_START);
    disc->reason        = reason;
    disc->loc_term_reason = reason;
    ke_msg_send(disc);
}


#endif // (BLE_CENTRAL || BLE_PERIPHERAL)

/// @} LLC_DISC
