/**
 ****************************************************************************************
 *
 * @file llc_clk_acc.c
 *
 * @brief Handles the sleep clock accuracy update procedure.
 *
 * Copyright (C) RivieraWaves 2009-2019
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

#if (BLE_PERIPHERAL || BLE_CENTRAL)
#include <stdint.h>
#include <stdbool.h>

#include "compiler.h"    // __ARRAY_EMPTY and __STATIC define
#include "co_version.h"  // For device version
#include "co_bt.h"       // BT Standard defines (HCI, LLCP, Error codes)
#include "co_utils.h"    // For device version pdu preparation

#include "ke_msg.h"      // Kernel message
#include "ke_timer.h"    // Kernel timers

#include "llc_int.h"     // Internal LLC API
#include "llc_llcp.h"    // Internal LLCP API

#include "lld.h"         // LLD API
#include "llm.h"         // LLM API

#include "hci.h"         // For HCI handler
#include "btdm_patch.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/*@TRACE*/
enum llc_clk_acc_state
{
    /// Start clock accuracy exchange procedure
    LLC_CLK_ACC_PROC_START,
    /// Wait for peer's clock accuracy response
    LLC_CLK_ACC_WAIT_PEER_RSP,
};

/*@TRACE*/
enum llc_op_clk_acc
{
    /// Procedure initiated because of an HCI_LE_Modify_Sleep_Clock_Accuracy command
    LLC_MOD_SCA,
    /// Procedure initiated because of an HCI_LE_Request_Peer_SCA command
    LLC_REQ_PEER_SCA,
};


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Clock accuracy exchange operation indication structure definition
/*@TRACE*/
struct llc_op_clk_acc_ind
{
    /// procedure information
    llc_procedure_t proc;

    /// Switch to more or less accurate clock (@see enum clk_acc_action)
    uint8_t action;

    /// Peer's SCA (@see enum SCA)
    uint8_t peer_sca;

    /// Indicates the purpose of the procedure (@see enum llc_op_clk_acc)
    uint8_t purpose;
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

/**
 ****************************************************************************************
 * @brief Sends the clock accuracy request pdu.
 *
 ****************************************************************************************
 */
__STATIC void llc_ll_clk_acc_req_pdu_send(uint8_t link_id, uint8_t sca)
{
    struct ll_clk_acc_req pdu;

    pdu.op_code = LL_CLK_ACC_REQ_OPCODE;
    pdu.sca = sca;

    llc_llcp_send(link_id, (union llcp_pdu *) &pdu, NULL);
}

/**
 ****************************************************************************************
 * @brief Sends the clock accuracy response pdu.
 *
 ****************************************************************************************
 */
__STATIC void llc_ll_clk_acc_rsp_pdu_send(uint8_t link_id, uint8_t sca)
{
    struct ll_clk_acc_rsp pdu;

    pdu.op_code = LL_CLK_ACC_RSP_OPCODE;
    pdu.sca = sca;

    llc_llcp_send(link_id, (union llcp_pdu *) &pdu, NULL);
}

/**
 ****************************************************************************************
 * Continue execution of local procedure
 *
 * @param[in] link_id Link identifier
 * @param[in] status  Status of the operation
 ****************************************************************************************
 */

__STATIC void llc_loc_clk_acc_proc_continue(uint8_t link_id, uint8_t status)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(PROC_CONTINUE_PATCH_TYPE, LLC_LOC_CLK_ACC_PROC_CONTINUE_C_FUN_BIT, link_id, status);
    /// retrieve procedure parameters
    struct llc_op_clk_acc_ind *param = (struct llc_op_clk_acc_ind *) llc_proc_get(link_id, LLC_PROC_LOCAL);
    bool finished = true;
    //Get environment pointer
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];

    switch (llc_proc_state_get(&param->proc))
    {
    /// Procedure started
    case LLC_CLK_ACC_PROC_START:
        /*
         * @startuml llc_clk_acc_start.png
         * title : Clock accuracy Exchange procedure start (CLK_ACC_PROC_START)
         * participant LLC
         * participant LLD
         * LLC -> LLC: llc_loc_clk_acc_proc_continue(START)
         * hnote over LLC #aqua: state = CLK_ACC_WAIT_PEER_RSP
         * activate LLC
         *     LLC --> LLD : LL_CLK_ACC_REQ
         * deactivate LLC
         * note over LLC: Start LLCP timeout
         * @enduml
         */
    {
        uint8_t sca;

        // Check purpose
        if (param->purpose == LLC_MOD_SCA)
        {
            // Check action
            if (param->action == SWITCH_TO_MORE_ACC_CLK)
            {
                // Indicate LLM that the link requires active clock
                llm_clk_acc_set(link_id, true);

                // Active clock is indicated to peer
                SETB(llc_env_ptr->link_info, LLC_INFO_CLK_ACC, 1);
            }
            else // SWITCH_TO_LESS_ACC_CLK
            {
                // Sleep clock is indicated to peer
                SETB(llc_env_ptr->link_info, LLC_INFO_CLK_ACC, 0);
            }
        }

        // Check configuration
        if (GETB(llc_env[link_id]->link_info, LLC_INFO_CLK_ACC))
        {
            uint16_t clock_drift;
            // Get the clock drift
            clock_drift = rwip_current_drift_get();

            // Convert to SCA
            if (clock_drift <= 20)
            {
                sca = SCA_20PPM;
            }
            else if (clock_drift <= 50)
            {
                sca = SCA_50PPM;
            }
            else
            {
                sca = SCA_50PPM;
                ASSERT_INFO_FORCE(0, clock_drift, 0);
            }
        }
        else
        {
            // Sleep clock is indicated to peer
            sca = rwip_sca_get();
        }

        llc_ll_clk_acc_req_pdu_send(link_id, sca);
        llc_proc_state_set(&param->proc, link_id, LLC_CLK_ACC_WAIT_PEER_RSP);
        finished = false;

        // Start the LLCP Response TO
        llc_proc_timer_set(link_id, LLC_PROC_LOCAL, true);
    }
    break;

    /// Peer answer
    case LLC_CLK_ACC_WAIT_PEER_RSP:
    {
        /*
         * @startuml llc_clk_acc_wait_peer_rsp.png
         * title : Clock accuracy peer's response (CLK_ACC_PROC_START)
         * participant HCI
         * participant LLC
         *  LLC <-: ll_clk_acc_rsp_handler
         * activate LLC
         *     LLC -> LLC:llc_loc_clk_acc_proc_continue(CLK_ACC_WAIT_PEER_RSP)
         *     note over LLC : Stop LLCP timeout
         *     hnote over LLC : LOC_PROC = Busy
         * deactivate LLC
         * @enduml
         */
        // Got the response for clock accuracy exchange procedure locally initiated
        llc_proc_timer_set(link_id, LLC_PROC_LOCAL, false);

        if ((param->action == SWITCH_TO_LESS_ACC_CLK) && (status == CO_ERROR_NO_ERROR))
        {
            // Indicate LLM that link does not require active clock anymore
            llm_clk_acc_set(link_id, false);
        }
    }
    break;

    default:
    {
        ASSERT_INFO_FORCE(0, link_id, llc_proc_state_get(&param->proc));
    }
    break;
    }

    if (finished)
    {
        if (param->purpose == LLC_REQ_PEER_SCA)
        {
            uint16_t conhdl = BLE_LINKID_TO_CONHDL(link_id);

            // Send the HCI_LE_Request_Peer_SCA_Complete event
            struct hci_le_req_peer_sca_cmp_evt *event = KE_MSG_ALLOC(HCI_LE_EVENT, conhdl, HCI_LE_META_EVT_CODE, hci_le_req_peer_sca_cmp_evt);
            event->subcode  = HCI_LE_REQ_PEER_SCA_CMP_EVT_SUBCODE;
            event->status   = status;
            event->conhdl   = conhdl;
            event->peer_clock_accuracy = param->peer_sca;
            hci_send_2_host(event);
        }

        // unregister procedure
        llc_proc_unreg(link_id, LLC_PROC_LOCAL);
    }
}

/**
 ****************************************************************************************
 * @brief Handles error during clock accuracy exchange (UNKNOWN/REJECT).
 *
 * PlantUML Clock accuracy exchange error description
 *
 * @startuml llc_clk_acc_error_cb.png
 * title : Clock accuracy exchange procedure error callback description
 * participant LLC
 * participant LLD
 * alt unknown received from peer
 *     LLD --> LLC : LLCP_UNKNOWN_RSP
 *     activate LLC
 *     LLC -> LLC: llc_clk_acc_proc_err_cb(LLCP_UMKNOWN_RSP)
 *     note over LLC: Disable the clock accuracy exchange procedure
 * end
 * @enduml
 *
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC void llc_clk_acc_proc_err_cb(uint8_t link_id, uint8_t error_type, void *param)
{
    uint8_t status = CO_ERROR_NO_ERROR;
    switch (error_type)
    {
    case LLC_ERR_DISCONNECT:
    {
        status = *((uint8_t *) param);
    }
    break;
    case LLC_ERR_LLCP_UNKNOWN_RSP:
    {
        struct ll_unknown_rsp *rsp = (struct ll_unknown_rsp *) param;
        if (rsp->unk_type == LL_CLK_ACC_REQ_OPCODE)
        {
            status = CO_ERROR_UNSUPPORTED;
        }
    }
    break;
    case LLC_ERR_LLCP_REJECT_IND:
    {
        struct ll_reject_ind *reject = (struct ll_reject_ind *) param;
        status =  reject->err_code;
    }
    break;
    case LLC_ERR_LLCP_REJECT_EXT_IND:
    {
        struct ll_reject_ext_ind *reject_ext = (struct ll_reject_ext_ind *) param;
        if (reject_ext->rej_op_code == LL_CLK_ACC_REQ_OPCODE)
        {
            status =  reject_ext->err_code;
        }
    }
    break;
    default: /* Nothing to do, ignore */
        break;
    }

    if (status != CO_ERROR_NO_ERROR)
    {
        if (status == CO_ERROR_UNSUPPORTED)
        {
            llc_le_feature_set(link_id, BLE_FEAT_SLEEP_CLK_ACC_UPD, false);
        }

        llc_loc_clk_acc_proc_continue(link_id, status);
    }
}


/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 *  @brief Handles the reception of an LLCP clock accuracy request
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
uint8_t ll_clk_acc_req_handler(uint8_t link_id, struct ll_clk_acc_req *pdu, uint16_t event_cnt)
{
    //Get environment pointer
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];
    uint8_t status = CO_ERROR_NO_ERROR;
    uint8_t sca;

    // If in the slave role and the SCA received is valid
    if (!GETB(llc_env[link_id]->link_info, LLC_INFO_MASTER_ROLE) && (pdu->sca <= SCA_20PPM))
    {
        // Set the peer SCA
        lld_con_peer_sca_set(link_id, pdu->sca);
    }

    if (GETB(llc_env_ptr->link_info, LLC_INFO_CLK_ACC))
    {
        // Get the clock drift
        uint16_t clock_drift = rwip_current_drift_get();

        // Convert to SCA
        if (clock_drift <= 20)
        {
            sca = SCA_20PPM;
        }
        else if (clock_drift <= 50)
        {
            sca = SCA_50PPM;
        }
        else
        {
            sca = SCA_50PPM;
            ASSERT_INFO_FORCE(0, clock_drift, 0);
        }
    }
    else
    {
        // Get the worst SCA of the device
        sca = rwip_sca_get();
    }

    // Send the clock accuracy response
    llc_ll_clk_acc_rsp_pdu_send(link_id, sca);

    return (status);
}

/**
 ****************************************************************************************
 *  @brief Handles the reception of an LLCP clock accuracy response
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
uint8_t ll_clk_acc_rsp_handler(uint8_t link_id, struct ll_clk_acc_rsp *pdu, uint16_t event_cnt)
{
    uint8_t status = CO_ERROR_NO_ERROR;

    // Retrieve procedure parameters
    struct llc_op_clk_acc_ind *param = (struct llc_op_clk_acc_ind *) llc_proc_get(link_id, LLC_PROC_LOCAL);
    // Save the peer's SCA
    param->peer_sca = pdu->sca;

    // If in the slave role and the SCA received is valid
    if (!GETB(llc_env[link_id]->link_info, LLC_INFO_MASTER_ROLE) && (pdu->sca <= SCA_20PPM))
    {
        // Set the peer SCA
        lld_con_peer_sca_set(link_id, pdu->sca);
    }

    if (llc_proc_id_get(link_id, LLC_PROC_LOCAL) == LLC_PROC_CLK_ACC)
    {
        // finish execution of clock accuracy exchange
        llc_loc_clk_acc_proc_continue(link_id, CO_ERROR_NO_ERROR);
    }

    return (status);
}

/**
 ****************************************************************************************
 * @brief Handles the clock accuracy exchange procedure indication message.
 *
 * PlantUML procedure description
 *
 * @startuml llc_clk_acc_op_start.png
 * title : Clock accuracy exchange procedure start
 * participant LLC
 *  --> LLC : LLC_OP_CLK_ACC_IND
 * LLC -> LLC: llc_op_clk_acc_ind_handler()
 * activate LLC
 * hnote over LLC : LOC_PROC = Busy
 * LLC -> LLC: llc_loc_clk_acc_proc_continue(START)
 * activate LLC
 * note right LLC #lightgreen: See __llc_loc_clk_acc_proc_continue()__
 * deactivate LLC
 * deactivate LLC
 * @enduml
 *
 ****************************************************************************************
 */
int llc_op_clk_acc_ind_handler(ke_msg_id_t const msgid, struct llc_op_clk_acc_ind *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Current message status
    int msg_status = KE_MSG_CONSUMED;
    uint8_t link_id = KE_IDX_GET(dest_id);

    // Check if state in disconnected state
    if (llc_is_disconnecting(link_id))
    {
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
        // execute feature exchange procedure
        llc_loc_clk_acc_proc_continue(link_id, CO_ERROR_NO_ERROR);

    }

    return (msg_status);
}

void llc_clk_acc_modify(uint8_t link_id, uint8_t action)
{
    // check if state is Free or in disconnected state
    if (llc_is_disconnecting(link_id))
    {
    }
    // check if peer doesn't support clock accuracy request
    else if (!llc_le_feature_check(link_id, BLE_FEAT_SLEEP_CLK_ACC_UPD))
    {
    }
    else
    {
        ke_task_id_t llc_id = KE_BUILD_ID(TASK_LLC, link_id);
        struct llc_op_clk_acc_ind *clk_acc = KE_MSG_ALLOC(LLC_OP_CLK_ACC_IND, llc_id, llc_id, llc_op_clk_acc_ind);
        llc_proc_init(&clk_acc->proc, LLC_PROC_CLK_ACC, llc_clk_acc_proc_err_cb);
        llc_proc_state_set(&clk_acc->proc, link_id, LLC_CLK_ACC_PROC_START);
        clk_acc->action = action;
        clk_acc->purpose = LLC_MOD_SCA;
        ke_msg_send(clk_acc);
    }
}

/*
 ****************************************************************************************
 * HCI Handlers
 ****************************************************************************************
 */

#if HCI_TL_SUPPORT
/**
 ****************************************************************************************
 * @brief Handles the HCI LE Request Peer SCA command.
 *
 ****************************************************************************************
 */
int hci_le_req_peer_sca_cmd_handler(uint8_t link_id, struct hci_le_req_peer_sca_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_NO_ERROR;

    // check if state is Free or in disconnected state
    if (llc_is_disconnecting(link_id))
    {
        status = CO_ERROR_COMMAND_DISALLOWED;
    }
    // check if peer doesn't support clock accuracy request
    else if (!llc_le_feature_check(link_id, BLE_FEAT_SLEEP_CLK_ACC_UPD))
    {
        status = CO_ERROR_UNSUPPORTED_REMOTE_FEATURE;
    }

    // Send the command status event
    llc_cmd_stat_send(link_id, opcode, status);

    if (status == CO_ERROR_NO_ERROR)
    {
        ke_task_id_t llc_id = KE_BUILD_ID(TASK_LLC, link_id);
        struct llc_op_clk_acc_ind *clk_acc = KE_MSG_ALLOC(LLC_OP_CLK_ACC_IND, llc_id, llc_id, llc_op_clk_acc_ind);
        llc_proc_init(&clk_acc->proc, LLC_PROC_CLK_ACC, llc_clk_acc_proc_err_cb);
        llc_proc_state_set(&clk_acc->proc, link_id, LLC_CLK_ACC_PROC_START);
        clk_acc->purpose = LLC_REQ_PEER_SCA;
        clk_acc->action = GETB(llc_env[link_id]->link_info, LLC_INFO_CLK_ACC) ? SWITCH_TO_MORE_ACC_CLK : SWITCH_TO_LESS_ACC_CLK;
        clk_acc->peer_sca = SCA_500PPM;
        ke_msg_send(clk_acc);
    }

    return (KE_MSG_CONSUMED);
}
#endif // (HCI_TL_SUPPORT)

#endif // (BLE_PERIPHERAL || BLE_CENTRAL)
/// @} LLC_OP
