/**
 ****************************************************************************************
 *
 * @file llc_ver_exch.c
 *
 * @brief Handles the Version exchange procedure.
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

#include "compiler.h"    // __ARRAY_EMPTY and __STATIC define
#include "co_version.h"  // For device version
#include "co_bt.h"       // BT Standard defines (HCI, LLCP, Error codes)
#include "co_utils.h"    // For device version pdu preparation

#include "ke_msg.h"      // Kernel message
#include "ke_timer.h"    // Kernel timers

#include "llc_int.h"     // Internal LLC API
#include "llc_llcp.h"    // Internal LLCP API

#include "hci.h"         // For HCI handler
#include "btdm_patch.h"


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Version exchange operation indication structure definition
/*@TRACE*/
struct llc_op_ver_exch_ind
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
enum llc_version_exchange_state
{
    /// Start Version exchange procedure
    LLC_VER_PROC_START,
    /// Wait for peer version indication
    LLC_VER_WAIT_PEER_VER_IND,
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
 * @brief Sends the read remote information version pdu.
 *
 * This function allocates an sets header and parameters of the pdu before pushing it in
 * the tx queue.
 *
 * @param[in] link_id        Link identifier on which the pdu will be sent
 ****************************************************************************************
 */
__STATIC void llc_llcp_version_ind_pdu_send(uint8_t link_id)
{
    struct ll_version_ind pdu;
    pdu.op_code  = LL_VERSION_IND_OPCODE;
    pdu.vers    = RWBT_SW_VERSION_MAJOR;
    pdu.compid  = RW_COMP_ID;
    pdu.subvers = CO_SUBVERSION_BUILD(RWBT_SW_VERSION_MINOR, RWBT_SW_VERSION_BUILD);

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
__STATIC void llc_hci_version_info_send(uint8_t link_id, uint8_t status, llc_version_t *ver)
{
    uint16_t conhdl = BLE_LINKID_TO_CONHDL(link_id);
    struct hci_rd_rem_ver_info_cmp_evt *event = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_RD_REM_VER_INFO_CMP_EVT_CODE, hci_rd_rem_ver_info_cmp_evt);

    event->conhdl   = conhdl;
    event->status   = status;
    if (status == CO_ERROR_NO_ERROR)
    {
        event->compid   = ver->compid;
        event->subvers  = ver->subvers;
        event->vers     = ver->lmp_vers;
    }
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
__STATIC void llc_ver_exch_loc_proc_continue(uint8_t link_id, uint8_t status)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(PROC_CONTINUE_PATCH_TYPE, LLC_VER_EXCH_LOC_PROC_CONTINUE_C_FUN_BIT, link_id, status);
    /// Gets the LLC environment dedicated to this link
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];
    /// retrieve procedure parameters
    struct llc_op_ver_exch_ind *param = (struct llc_op_ver_exch_ind *) llc_proc_get(link_id, LLC_PROC_LOCAL);
    bool finished = true;

    switch (llc_proc_state_get(&param->proc))
    {
    /// Procedure started
    case LLC_VER_PROC_START:
    {
        // Check if remote version already known
        // yes -> no need to continue operation
        if (!GETB(llc_env_ptr->link_info, LLC_INFO_PEER_VERSION_RECV))
        {
            // no --> send local version to retrieve peer version information
            llc_llcp_version_ind_pdu_send(link_id);
            SETB(llc_env_ptr->link_info, LLC_INFO_LOCAL_VERSION_SENT, true);
            llc_proc_state_set(&param->proc, link_id, LLC_VER_WAIT_PEER_VER_IND);
            finished = false;

            // Start the LLCP Response TO
            llc_proc_timer_set(link_id, LLC_PROC_LOCAL, true);
        }
    }
    break;

    /// Wait for peer version
    case LLC_VER_WAIT_PEER_VER_IND:
    {
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
            llc_hci_version_info_send(link_id, status, &(llc_env_ptr->rem_ver));

            SETB(llc_env_ptr->flow_ctrl, LLC_HCI_RD_REM_VER_REQ, false);
        }
        // unregister procedure
        llc_proc_unreg(link_id, LLC_PROC_LOCAL);
    }
}


__STATIC void llc_ver_proc_err_cb(uint8_t link_id, uint8_t error_type, void *param)
{
    switch (error_type)
    {
    case LLC_ERR_DISCONNECT:
    {
        uint8_t reason = *((uint8_t *) param);
        llc_ver_exch_loc_proc_continue(link_id, reason);
    }
    break;
    case LLC_ERR_LLCP_UNKNOWN_RSP:
    case LLC_ERR_LLCP_REJECT_IND:
    case LLC_ERR_LLCP_REJECT_EXT_IND:
    {
        // ignore
    } break;
    default:
    {
        // not expected at all
        ASSERT_INFO_FORCE(0, link_id, error_type);
    }
    break;
    }
}

__STATIC void llc_rem_supp_feat_set(uint8_t link_id, uint8_t peer_ver)
{
    if (peer_ver < BT41_VERSION)
    {
        llc_le_feature_set(link_id, BLE_FEAT_CON_PARAM_REQ_PROC,        false);
        llc_le_feature_set(link_id, BLE_FEAT_EXT_REJ_IND,               false);
        llc_le_feature_set(link_id, BLE_FEAT_SLAVE_INIT_FEAT_EXCHG,     false);
        llc_le_feature_set(link_id, BLE_FEAT_PING,                      false);
    }

    if (peer_ver < BT42_VERSION)
    {
        llc_le_feature_set(link_id, BLE_FEAT_DATA_PKT_LEN_EXT,          false);
        llc_le_feature_set(link_id, BLE_FEAT_LL_PRIVACY,                false);
        llc_le_feature_set(link_id, BLE_FEAT_EXT_SCAN_FILT_POLICY,      false);
    }

    if (peer_ver < BT50_VERSION)
    {
        llc_le_feature_set(link_id, BLE_FEAT_2M_PHY,                    false);
        llc_le_feature_set(link_id, BLE_FEAT_STABLE_MOD_IDX_TX,         false);
        llc_le_feature_set(link_id, BLE_FEAT_STABLE_MOD_IDX_RX,         false);
        llc_le_feature_set(link_id, BLE_FEAT_CODED_PHY,                 false);
        llc_le_feature_set(link_id, BLE_FEAT_EXT_ADV,                   false);
        llc_le_feature_set(link_id, BLE_FEAT_PER_ADV,                   false);
        llc_le_feature_set(link_id, BLE_FEAT_CHAN_SEL_ALGO_2,           false);
        llc_le_feature_set(link_id, BLE_FEAT_PWR_CLASS_1,               false);
        llc_le_feature_set(link_id, BLE_FEAT_MIN_NUM_USED_CHAN_PROC,    false);
    }

    if (peer_ver < BT51_VERSION)
    {
        llc_le_feature_set(link_id, BLE_FEAT_CON_CTE_REQ,               false);
        llc_le_feature_set(link_id, BLE_FEAT_CON_CTE_RSP,               false);
        llc_le_feature_set(link_id, BLE_FEAT_CONLESS_CTE_TX,            false);
        llc_le_feature_set(link_id, BLE_FEAT_CONLESS_CTE_RX,            false);
        llc_le_feature_set(link_id, BLE_FEAT_AOD,                       false);
        llc_le_feature_set(link_id, BLE_FEAT_AOA,                       false);
        llc_le_feature_set(link_id, BLE_FEAT_CTE_RX,                    false);
        llc_le_feature_set(link_id, BLE_FEAT_PER_ADV_SYNC_TRANSF_TX,    false);
        llc_le_feature_set(link_id, BLE_FEAT_PER_ADV_SYNC_TRANSF_RX,    false);
        llc_le_feature_set(link_id, BLE_FEAT_SLEEP_CLK_ACC_UPD,         false);
        llc_le_feature_set(link_id, BLE_FEAT_PUB_KEY_VALID,             false);
    }

    if (peer_ver < BT52_VERSION)
    {
        llc_le_feature_set(link_id, BLE_FEAT_CIS_MASTER,                false);
        llc_le_feature_set(link_id, BLE_FEAT_CIS_SLAVE,                 false);
        llc_le_feature_set(link_id, BLE_FEAT_ISO_BROADCASTER,           false);
        llc_le_feature_set(link_id, BLE_FEAT_SYNCED_RECEIVER,           false);
        llc_le_feature_set(link_id, BLE_FEAT_CIS_HOST_SUPPORT,          false);
        llc_le_feature_set(link_id, BLE_FEAT_POWER_CONTROL_REQ,         false);
        llc_le_feature_set(link_id, BLE_FEAT_POWER_CHANGE_IND,          false);
        llc_le_feature_set(link_id, BLE_FEAT_PATH_LOSS_MONITORING,      false);
    }

#if (BT_53)
    if (peer_ver < BT53_VERSION)
    {
        llc_le_feature_set(link_id, BLE_FEAT_PER_ADV_ADI,               false);
        llc_le_feature_set(link_id, BLE_FEAT_CON_SUBRATING,             false);
        llc_le_feature_set(link_id, BLE_FEAT_CON_SUBRATING_HOST_SUPPORT, false);
        llc_le_feature_set(link_id, BLE_FEAT_CHAN_CLASS,                false);
    }
#endif // (BT_53)
}

/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @brief Handles the Version exchange procedure indication message.
 *
 ****************************************************************************************
 */
int llc_op_ver_exch_ind_handler(ke_msg_id_t const msgid, struct llc_op_ver_exch_ind *param,
                                ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Current message status
    int msg_status = KE_MSG_CONSUMED;
    uint8_t link_id = KE_IDX_GET(dest_id);

    // Check if state in disconnected state
    if (llc_is_disconnecting(link_id))
    {
        // Abort request immediately with disconnection reason
        if (param->req_by_host)
        {
            //Get environment pointer
            struct llc_env_tag *llc_env_ptr = llc_env[link_id];
            llc_hci_version_info_send(link_id, llc_env_ptr->disc_reason, &(llc_env_ptr->rem_ver));
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
        // execute version exchange procedure
        llc_ver_exch_loc_proc_continue(link_id, CO_ERROR_NO_ERROR);
    }
    return (msg_status);
}

/**
 ****************************************************************************************
 * @brief Handles the reception of a LLCP Version Indication message
 *
 * PlantUML procedure description
 *
 * @startuml llc_ver_exch_peer_init.png
 * title : Version exchange initiated by peer device
 * participant LLC
 * participant LLD
 * LLD --> LLC : LLCP_VERSION_IND
 * LLC -> LLC : ll_version_ind_handler
 * activate LLC
 * note over LLC: store remote version
 * alt Local version already sent
 *      note over LLC : Nothing to do
 * else provide local version
 *      LLC --> LLD : LLCP_VERSION_IND
 * deactivate LLC
 * end
 * @enduml
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
uint8_t ll_version_ind_handler(uint8_t link_id, struct ll_version_ind *pdu, uint16_t event_cnt)
{
    // note: when receiving message all sanity check have been performed by default receive handler

    //Get environment pointer
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];

    // Save the peer version in our environment
    llc_env_ptr->rem_ver.lmp_vers   = pdu->vers;
    llc_env_ptr->rem_ver.compid     = pdu->compid;
    llc_env_ptr->rem_ver.subvers    = pdu->subvers;
    SETB(llc_env_ptr->link_info, LLC_INFO_PEER_VERSION_RECV, true);

    if (llc_proc_id_get(link_id, LLC_PROC_LOCAL) == LLC_PROC_VERSION_EXCHANGE)
    {
        // finish execution of version exchange
        llc_ver_exch_loc_proc_continue(link_id, CO_ERROR_NO_ERROR);
    }
    else if (!GETB(llc_env_ptr->link_info, LLC_INFO_LOCAL_VERSION_SENT))
    {
        // The procedure is initiated by the peer, reply and simply wait for Ack
        llc_llcp_version_ind_pdu_send(link_id);
        SETB(llc_env_ptr->link_info, LLC_INFO_LOCAL_VERSION_SENT, true);
    }
    else
    {
        // nothing more to do
    }

    // Restrict remote supported features based on version number
    llc_rem_supp_feat_set(link_id, llc_env_ptr->rem_ver.lmp_vers);

    return (CO_ERROR_NO_ERROR);
}


/**
 ****************************************************************************************
 * @brief Handles the command HCI read version information.
 *
 * PlantUML procedure description
 *
 * @startuml llc_ver_exch_host_init.png
 * title : Version exchange initiated by host
 * participant HCI
 * participant LLC
 * participant LLD
 * activate LLC
 * HCI --> LLC : HCI_RD_REM_VERSION_CMD
 * LLC --> HCI : HCI_CMD_STAT_EVENT
 * LLC --> LLC : LLC_OP_VER_EXCH_IND
 * deactivate LLC
 * LLC -> LLC: llc_op_ver_exch_ind_handler
 * activate LLC
 * alt Remote version already exchanged
 *      note over LLC : load known peer version
 * else start LLCP version exchange
 *      hnote over LLC : local busy
 *      LLC --> LLD : LLCP_VERSION_IND
 *      deactivate LLC
 *      LLD --> LLC : LLCP_VERSION_IND
 *      LLC -> LLC : ll_version_ind_handler
 *      activate LLC
 *      note over LLC: store remote version
 *      hnote over LLC : local ready
 * end
 * LLC --> HCI : HCI_RD_REM_VER_INFO_CMP_EVT_CODE
 * deactivate LLC
 * @enduml
 *
 * @param[in] link_id Link Identifier
 * @param[in] param   Pointer to the parameters of the message.
 * @param[in] opcode  HCI Operation code
 ****************************************************************************************
 */
int hci_rd_rem_ver_info_cmd_handler(uint8_t link_id, struct hci_rd_rem_ver_info_cmd const *param, uint16_t opcode)
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
    else if (GETB(llc_env_ptr->flow_ctrl, LLC_HCI_RD_REM_VER_REQ))
    {
        status = CO_ERROR_CONTROLLER_BUSY;
    }
    else
    {
        ke_task_id_t llc_id = KE_BUILD_ID(TASK_LLC, link_id);
        struct llc_op_ver_exch_ind *ver_exch = KE_MSG_ALLOC(LLC_OP_VER_EXCH_IND, llc_id, llc_id, llc_op_ver_exch_ind);
        llc_proc_init(&ver_exch->proc, LLC_PROC_VERSION_EXCHANGE, llc_ver_proc_err_cb);
        llc_proc_state_set(&ver_exch->proc, link_id, LLC_VER_PROC_START);
        ver_exch->req_by_host   = true;
        ke_msg_send(ver_exch);

        SETB(llc_env_ptr->flow_ctrl, LLC_HCI_RD_REM_VER_REQ, true);
    }

    // Send the command status event
    llc_cmd_stat_send(link_id, opcode, status);

    return (KE_MSG_CONSUMED);
}


#endif // (BLE_CENTRAL || BLE_PERIPHERAL)
/// @} LLC_OP
