/**
 ****************************************************************************************
 *
 * @file llc_feat_exch.c
 *
 * @brief Handles the Feature exchange procedure.
 *
 * Copyright (C) RivieraWaves 2009-2016
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

#if (BLE_CENTRAL || BLE_PERIPHERAL)
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "compiler.h"    // __ARRAY_EMPTY and __STATIC define
#include "co_bt.h"       // BT Standard defines (HCI, LLCP, Error codes)
#include "co_utils.h"    // For device pdu preparation

#include "ke_msg.h"      // Kernel message
#include "ke_timer.h"    // Kernel timers

#include "llc_int.h"     // Internal LLC API
#include "llc_llcp.h"    // Internal LLCP API

#include "hci.h"         // For HCI handler
#include "llm.h"         // To Get Local supported features
#include "btdm_patch.h"


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Version exchange operation indication structure definition
/*@TRACE*/
struct llc_op_feats_exch_ind
{
    /// procedure information
    llc_procedure_t proc;
    /// Use to know id operation requested by host
    bool req_by_host;
} ;


/*
 * DEFINES
 ****************************************************************************************
 */
/*@TRACE*/
enum llc_feature_exchange_state
{
    /// Start Feature exchange procedure
    LLC_FEATS_PROC_START,
    /// Wait for feature supported response
    LLC_FEATS_WAIT_PEER_FEAT_RSP,
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

__STATIC void llc_ll_feature_req_pdu_send(uint8_t link_id);
__STATIC void llc_ll_feature_rsp_pdu_send(uint8_t link_id);
__STATIC void llc_hci_feats_info_send(uint8_t link_id, uint8_t status, struct le_features *feats);
__STATIC void llc_loc_feats_exch_proc_continue(uint8_t link_id, uint8_t status);
__STATIC void llc_feats_exch_proc_err_cb(uint8_t link_id, uint8_t error_type, void *param);

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Sends the features request pdu.
 *
 ****************************************************************************************
 */
__STATIC void llc_ll_feature_req_pdu_send(uint8_t link_id)
{
    struct ll_feature_req pdu;

    if (GETB(llc_env[link_id]->link_info, LLC_INFO_MASTER_ROLE))
    {
        pdu.op_code = LL_FEATURE_REQ_OPCODE;
    }
    else
    {
        pdu.op_code = LL_SLAVE_FEATURE_REQ_OPCODE;
    }

    // Get the local supported features
    llm_le_features_get(&pdu.feats);
    // Mask the Remote Public Key Validation feature bit
    pdu.feats.feats[BLE_FEAT_PUB_KEY_VALID >> 3] &= ~(1 << (BLE_FEAT_PUB_KEY_VALID & 0x7));

    llc_llcp_send(link_id, (union llcp_pdu *) &pdu, NULL);
}

/**
 ****************************************************************************************
 * @brief Sends the features response pdu.
 *
 ****************************************************************************************
 */
__STATIC void llc_ll_feature_rsp_pdu_send(uint8_t link_id)
{
    struct ll_feature_rsp pdu;

    pdu.op_code = LL_FEATURE_RSP_OPCODE;

    // Get the local supported features
    llm_le_features_get(&pdu.feats);

    // Mask the Remote Public Key Validation feature bit
    pdu.feats.feats[BLE_FEAT_PUB_KEY_VALID >> 3] &= ~(1 << (BLE_FEAT_PUB_KEY_VALID & 0x7));

    // Mask byte 0 with peer's features (see BT standard Vol 6 Part B.2.4.2.10)
    pdu.feats.feats[0] &= llc_env[link_id]->rem_feats.feats[0];

    llc_llcp_send(link_id, (union llcp_pdu *) &pdu, NULL);
}

/**
 ****************************************************************************************
 * @brief Sends the remote information version to the host.
 *
 * @param[in] link_id        Link identifier on which the pdu will be sent
 * @param[in] status         Status of the host request
 * @param[in] ver            Peer device informations
 ****************************************************************************************
 */
__STATIC void llc_hci_feats_info_send(uint8_t link_id, uint8_t status, struct le_features *feats)
{
    uint16_t conhdl = BLE_LINKID_TO_CONHDL(link_id);

    // sends the complete meta event
    // allocate the status event message
    struct hci_le_rd_rem_feats_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_LE_EVENT, conhdl, 0, hci_le_rd_rem_feats_cmd_cmp_evt);
    // gets event subcode
    event->subcode  = HCI_LE_RD_REM_FEATS_CMP_EVT_SUBCODE;
    // gets the status
    event->status   = status;
    // gets connection handle
    event->conhdl   = conhdl;
    // set the features to transmit
    memcpy(&event->le_feats.feats[0], &feats->feats[0], LE_FEATS_LEN);
    // send the message
    hci_send_2_host(event);
}

/**
 ****************************************************************************************
 * Continue execution of local procedure
 *
 * @param[in] link_id Link identifier
 * @param[in] status  Status of the operation
 ****************************************************************************************
 */

__STATIC void llc_loc_feats_exch_proc_continue(uint8_t link_id, uint8_t status)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(PROC_CONTINUE_PATCH_TYPE, LLC_LOC_FEATS_EXCH_PROC_CONTINUE_C_FUN_BIT, link_id, status);
    /// Gets the LLC environment dedicated to this link
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];
    /// retrieve procedure parameters
    struct llc_op_feats_exch_ind *param = (struct llc_op_feats_exch_ind *) llc_proc_get(link_id, LLC_PROC_LOCAL);
    bool finished = true;

    switch (llc_proc_state_get(&param->proc))
    {
    /// Procedure started
    case LLC_FEATS_PROC_START:
        /*
         * @startuml llc_feat_exch_start.png
         * title : Feature Exchange procedure start (FEATS_PROC_START)
         * participant LLC
         * participant LLD
         * LLC -> LLC: llc_loc_feat_exch_proc_continue(START)
         * hnote over LLC #aqua: state = WAIT_PEER_FEAT_RSP
         * activate LLC
         *     LLC --> LLD : LLCP_FEATURE_REQ
         * deactivate LLC
         * note over LLC: Start LLCP timeout
         * @enduml
         */
    {
        // Check if remote version already known
        // yes -> no need to continue operation
        if (!GETB(llc_env_ptr->link_info, LLC_INFO_PEER_FEATURE_RECV))
        {
            // no --> send local version to retrieve peer version information
            llc_ll_feature_req_pdu_send(link_id);
            llc_proc_state_set(&param->proc, link_id, LLC_FEATS_WAIT_PEER_FEAT_RSP);
            finished = false;

            // Start the LLCP Response TO
            llc_proc_timer_set(link_id, LLC_PROC_LOCAL, true);
        }
    }
    break;

    /// Peer answer
    case LLC_FEATS_WAIT_PEER_FEAT_RSP:
    {
        /*
         * @startuml llc_feat_exch_wait_peer_rsp.png
         * title : Feature Exchange procedure start (FEATS_PROC_START)
         * participant HCI
         * participant LLC
         *  LLC <-: ll_feature_rsp_handler
         * activate LLC
         *     LLC -> LLC:llc_loc_feat_exch_proc_continue(PEER_FEAT_RSP)
         *     note over LLC : Stop LLCP timeout
         *     alt host request or at least on value has changed
         *         LLC --> HCI : HCI_LE_RD_REM_FEATS_CMP_EVT
         *     end
         *     hnote over LLC : LOC_PROC = Busy
         * deactivate LLC
         * @enduml
         */
        // We got the response for the version indication procedure we initiated
        llc_proc_timer_set(link_id, LLC_PROC_LOCAL, false);
    }
    break;

    default:
    {
        status = CO_ERROR_UNSPECIFIED_ERROR;
        ASSERT_INFO_FORCE(0, link_id, llc_proc_state_get(&param->proc));
    }
    break;
    }

    if (finished)
    {
        // check if version info has to be sent to host
        if (param->req_by_host)
        {
            llc_hci_feats_info_send(link_id, status, &(llc_env_ptr->rem_feats));
            SETB(llc_env_ptr->flow_ctrl, LLC_HCI_RD_REM_FEAT_REQ, false);
        }
        // unregister procedure
        llc_proc_unreg(link_id, LLC_PROC_LOCAL);
    }
}

/**
 ****************************************************************************************
 * @brief Handles the feature exchange error (UNKNOWN/REJECT).
 *
 * PlantUML Feature exchange error description
 *
 * @startuml llc_feat_exch_error_cb.png
 * title : Features exchange procedure error callback description
 * participant LLC
 * participant LLD
 * alt unknown received from peer
 *     LLD --> LLC : LLCP_UNKNOWN_RSP
 *     activate LLC
 *     LLC -> LLC: llc_feats_exch_proc_err_cb(LLCP_UMKNOWN_RSP)
 *     note over LLC: Disable the feature exchange procedure
 * end
 * @enduml
 *
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC void llc_feats_exch_proc_err_cb(uint8_t link_id, uint8_t error_type, void *param)
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
        if (rsp->unk_type == LL_SLAVE_FEATURE_REQ_OPCODE)
        {
            status = CO_ERROR_UNSUPPORTED_REMOTE_FEATURE;
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
        if (reject_ext->rej_op_code == LL_SLAVE_FEATURE_REQ_OPCODE)
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
        if (status == CO_ERROR_UNSUPPORTED_REMOTE_FEATURE)
        {
            llc_le_feature_set(link_id, BLE_FEAT_SLAVE_INIT_FEAT_EXCHG, false);
        }

        llc_loc_feats_exch_proc_continue(link_id, status);
    }
}

/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handles the reception of a LLCP feature or LLCP slave feature requests

 * @param[in] link_id        Link identifier on which the pdu will be sent.
 * @param[in] pdu            LLCP PDU information received
 * @param[in] event_cnt      Event counter value when PDU has been received
 *
 * @return status code of handler:
 *    - CO_ERROR_NO_ERROR:               Nothing more to do
 *    - CO_ERROR_TERMINATED_MIC_FAILURE: Immediately disconnect the link
 *    - others:                          Send an LLCP_REJECT_IND or LLCP_REJECT_IND_EXT
 *
 *
 * PlantUML procedure description
 *
 * @startuml llc_feat_exch_init_peer.png
 * title : Feature exchange initiated by peer device
 * participant HCI
 * participant LLC
 * participant LLD
 * alt Peer is Slave
 *      LLD --> LLC : LLCP_SLAVE_FEATURE_REQ
 * else Peer is master
 *      LLD --> LLC : LLCP_FEATURE_REQ
 * end
 * activate LLC
 * note over LLC: store remote feature
 * alt Local feature already sent
 *      alt Host request
 *          LLC --> HCI : HCI_LE_RD_REM_FEATS_CMP_EVT
 *      else Controller request
 *          note over LLC : nothing to do
 *      end
 * else provide local feature
 *      LLC --> LLD : LLCP_FEATURE_RSP
 * deactivate LLC
 * end
 * @enduml
 ****************************************************************************************
 */
uint8_t ll_feature_req_handler(uint8_t link_id, struct ll_feature_req *pdu, uint16_t event_cnt)
{
    // Command status
    uint8_t status = CO_ERROR_NO_ERROR;
    //Get environment pointer
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];

    // Device must be slave of the link
    if (GETB(llc_env_ptr->link_info, LLC_INFO_MASTER_ROLE))
    {
        status = CO_ERROR_LMP_PDU_NOT_ALLOWED;
    }
    else
    {
        // Save the peer version in our environment
        // Merge local features and peer features
        llc_env_ptr->rem_feats.feats[0] = pdu->feats.feats[0];
        llc_env_ptr->rem_feats.feats[1] = pdu->feats.feats[1];
        llc_env_ptr->rem_feats.feats[2] = pdu->feats.feats[2];
        llc_env_ptr->rem_feats.feats[3] = pdu->feats.feats[3];
        llc_env_ptr->rem_feats.feats[4] = pdu->feats.feats[4];
        llc_env_ptr->rem_feats.feats[5] = pdu->feats.feats[5];
        llc_env_ptr->rem_feats.feats[6] = pdu->feats.feats[6];
        llc_env_ptr->rem_feats.feats[7] = pdu->feats.feats[7];

        SETB(llc_env_ptr->link_info, LLC_INFO_PEER_FEATURE_RECV, true);

        // The procedure is initiated by the peer, reply and simply wait for Ack
        llc_ll_feature_rsp_pdu_send(link_id);
    }

    return (status);
}

/**
 ****************************************************************************************
 * @brief Handles the reception of a LLCP Slave feature request
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
uint8_t ll_slave_feature_req_handler(uint8_t link_id, struct ll_slave_feature_req *pdu, uint16_t event_cnt)
{
    // Command status
    uint8_t status = CO_ERROR_NO_ERROR;
    //Get environment pointer
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];

    // Device must be master of the link
    if (!GETB(llc_env_ptr->link_info, LLC_INFO_MASTER_ROLE))
    {
        status = CO_ERROR_LMP_PDU_NOT_ALLOWED;
    }
    else
    {
        // Save the peer version in our environment
        // Merge local features and peer features
        llc_env_ptr->rem_feats.feats[0] = pdu->feats.feats[0];
        llc_env_ptr->rem_feats.feats[1] = pdu->feats.feats[1];
        llc_env_ptr->rem_feats.feats[2] = pdu->feats.feats[2];
        llc_env_ptr->rem_feats.feats[3] = pdu->feats.feats[3];
        llc_env_ptr->rem_feats.feats[4] = pdu->feats.feats[4];
        llc_env_ptr->rem_feats.feats[5] = pdu->feats.feats[5];
        llc_env_ptr->rem_feats.feats[6] = pdu->feats.feats[6];
        llc_env_ptr->rem_feats.feats[7] = pdu->feats.feats[7];

        SETB(llc_env_ptr->link_info, LLC_INFO_PEER_FEATURE_RECV, true);

        // The procedure is initiated by the peer, reply and simply wait for Ack
        llc_ll_feature_rsp_pdu_send(link_id);
    }

    return (status);
}
/**
 ****************************************************************************************
 * @brief Handles the reception of a LLCP feature response
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
uint8_t ll_feature_rsp_handler(uint8_t link_id, struct ll_feature_rsp *pdu, uint16_t event_cnt)
{
    //Get environment pointer
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];

    // Save the peer version in our environment
    // Merge local features and peer features
    // Merge local features and peer features
    llc_env_ptr->rem_feats.feats[0] = pdu->feats.feats[0];
    llc_env_ptr->rem_feats.feats[1] = pdu->feats.feats[1];
    llc_env_ptr->rem_feats.feats[2] = pdu->feats.feats[2];
    llc_env_ptr->rem_feats.feats[3] = pdu->feats.feats[3];
    llc_env_ptr->rem_feats.feats[4] = pdu->feats.feats[4];
    llc_env_ptr->rem_feats.feats[5] = pdu->feats.feats[5];
    llc_env_ptr->rem_feats.feats[6] = pdu->feats.feats[6];
    llc_env_ptr->rem_feats.feats[7] = pdu->feats.feats[7];

    SETB(llc_env_ptr->link_info, LLC_INFO_PEER_FEATURE_RECV, true);

    if (llc_proc_id_get(link_id, LLC_PROC_LOCAL) == LLC_PROC_FEATURE_EXCHANGE)
    {
        // finish execution of version exchange
        llc_loc_feats_exch_proc_continue(link_id, CO_ERROR_NO_ERROR);
    }

    return (CO_ERROR_NO_ERROR); // TODO check if return is valid
}

/**
 ****************************************************************************************
 * @brief Handles the command HCI read remote used feature.
 *
 * PlantUML procedure description
 *
 * @startuml llc_feat_exch_init_host.png
 * title : Features exchange initiated by host
 * participant HCI
 * participant LLC
 * HCI --> LLC : HCI_LE_RD_REM_FEAT_CMD
 * LLC -> LLC: hci_le_rd_rem_feats_cmd_handler()
 * activate LLC
 * alt link disconnected \nor slave \nor peer not support encryption
 *     LLC --> HCI : HCI_CMD_STAT_EVENT(DISALLOWED)
 * else  Parameters out of range
 *     LLC --> HCI : HCI_CMD_STAT_EVENT(INVALID_HCI_PARAMS)
 * else  Procedure already started
 *     LLC --> HCI : HCI_CMD_STAT_EVENT(BUSY)
 * else  Procedure can be started
 *     LLC --> LLC : LLC_OP_FEATS_EXCH_IND
 *     note right LLC #lightgreen: See __llc_op_feats_exch_ind_handler()__
 *     LLC --> HCI : HCI_CMD_STAT_EVENT(OK)
 * end
 * deactivate LLC
 * @enduml
 *
 * @param[in] link_id Link Identifier
 * @param[in] param   Pointer to the parameters of the message.
 * @param[in] opcode  HCI Operation code
 ****************************************************************************************
 */

int hci_le_rd_rem_feats_cmd_handler(uint8_t link_id, struct hci_le_rd_rem_feats_cmd const *param, uint16_t opcode)
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
    else if (GETB(llc_env_ptr->flow_ctrl, LLC_HCI_RD_REM_FEAT_REQ))
    {
        status = CO_ERROR_CONTROLLER_BUSY;
    }
    // check if peer doesn't support Slave initiated request and peer feature not received
    else if (!llc_le_feature_check(link_id, BLE_FEAT_SLAVE_INIT_FEAT_EXCHG)
             && !GETB(llc_env_ptr->link_info, LLC_INFO_MASTER_ROLE)
             && !GETB(llc_env_ptr->link_info, LLC_INFO_PEER_FEATURE_RECV))
    {
        status = CO_ERROR_UNSUPPORTED_REMOTE_FEATURE;
    }
    else
    {
        ke_task_id_t llc_id = KE_BUILD_ID(TASK_LLC, link_id);
        struct llc_op_feats_exch_ind *feats_exch = KE_MSG_ALLOC(LLC_OP_FEATS_EXCH_IND, llc_id, llc_id, llc_op_feats_exch_ind);
        llc_proc_init(&feats_exch->proc, LLC_PROC_FEATURE_EXCHANGE, llc_feats_exch_proc_err_cb);
        llc_proc_state_set(&feats_exch->proc, link_id, LLC_FEATS_PROC_START);
        feats_exch->req_by_host   = true;
        ke_msg_send(feats_exch);

        SETB(llc_env_ptr->flow_ctrl, LLC_HCI_RD_REM_FEAT_REQ, true);
    }

    // Send the command status event
    llc_cmd_stat_send(link_id, opcode, status);

    return (KE_MSG_CONSUMED);
}

#if (BT_53)
void llc_feat_exch_proc_start(uint8_t link_id)
{
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];

    // Check if master role, and an auto feature exchange required
    if (GETB(llc_env_ptr->link_info, LLC_INFO_MASTER_ROLE) && llm_le_feature_check(BLE_FEAT_CON_SUBRATING_HOST_SUPPORT))
    {
        ke_task_id_t llc_id = KE_BUILD_ID(TASK_LLC, link_id);
        struct llc_op_feats_exch_ind *feats_exch = KE_MSG_ALLOC(LLC_OP_FEATS_EXCH_IND, llc_id, llc_id, llc_op_feats_exch_ind);
        llc_proc_init(&feats_exch->proc, LLC_PROC_FEATURE_EXCHANGE, llc_feats_exch_proc_err_cb);
        llc_proc_state_set(&feats_exch->proc, link_id, LLC_FEATS_PROC_START);
        feats_exch->req_by_host = false;
        ke_msg_send(feats_exch);
    }
}
#endif // (BT_53)


/**
 ****************************************************************************************
 * @brief Handles the Feature exchange procedure indication message.
 *
 * PlantUML procedure description
 *
 * @startuml llc_feat_exch_op_start.png
 * title : Data length procedure start
 * participant LLC
 *  --> LLC : LLC_OP_FEATS_EXCH_IND
 * LLC -> LLC: llc_op_feats_exch_ind_handler()
 * activate LLC
 * hnote over LLC : LOC_PROC = Busy
 * LLC -> LLC: llc_feats_exch_loc_continue(START)
 * activate LLC
 * note right LLC #lightgreen: See __llc_feats_exch_loc_continue()__
 * deactivate LLC
 * deactivate LLC
 * @enduml
 *
 ****************************************************************************************
 */

int llc_op_feats_exch_ind_handler(ke_msg_id_t const msgid, struct llc_op_feats_exch_ind *param,
                                  ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Current message status
    int msg_status = KE_MSG_CONSUMED;
    uint8_t link_id = KE_IDX_GET(dest_id);

    // Check if state in disconnected state
    if (llc_is_disconnecting(link_id))
    {
        // disconnection on-going inform that command is aborted
        if (param->req_by_host)
        {
            //Get environment pointer
            struct llc_env_tag *llc_env_ptr = llc_env[link_id];
            llc_hci_feats_info_send(link_id, llc_env_ptr->disc_reason, &(llc_env_ptr->rem_feats));
        }
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
        llc_loc_feats_exch_proc_continue(link_id, CO_ERROR_NO_ERROR);

    }
    return (msg_status);
}

#endif //(BLE_CENTRAL || BLE_PERIPHERAL)

/// @} llc_feats_exch
