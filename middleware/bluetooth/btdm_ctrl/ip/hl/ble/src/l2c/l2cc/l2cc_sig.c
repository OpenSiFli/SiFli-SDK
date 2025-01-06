/**
 ****************************************************************************************
 *
 * @file l2cc_sig.c
 *
 * @brief Generic Access Profile Controller Signaling PDU Handler.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup L2CC_SIG Generic Access Profile Controller Signaling PDU Handler.
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"

#if (BLE_L2CC)

#include "gapc.h"
#include "gapm.h"
#include "../l2cm/l2cm_int.h" // Internal API required
#include "l2cc_int.h"
#include "l2cc_lecb.h"

#include "ke_mem.h"
#include "ke_task.h"
#include "ke_timer.h"

#include "co_utils.h"

#if (BLE_ISO_MODE_0_PROTOCOL)
    #include "am0_api.h"
#endif // (BLE_ISO_MODE_0_PROTOCOL)



/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Format of a pdu handler function
typedef int (*l2cc_sig_func_t)(uint8_t conidx, void *pdu);

/// Element of a pdu handler table.
typedef struct
{
    /// PDU identifier of the message
    uint8_t pdu_id;
    /// Pointer to the handler function for the pdu above.
    l2cc_sig_func_t handler;

} l2cc_sig_pdu_handler_t;




/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * MACROS
 ****************************************************************************************
 */

#define L2CC_LECB_AUTH(slvl)              (slvl & 0x0003)
#define L2CC_LECB_EKS(slvl)               ((slvl >> 2) & 0x0001)

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

#if (BLE_LECB)
/**
 ****************************************************************************************
 * @brief Retrieve Host error code from LECB error Code
 *
 * @param[in] l_err      LECB Error code
 *
 * @return Host Error code
 ****************************************************************************************
 */
static uint8_t l2cc_sig_lecb_l2h_err(uint16_t l_err)
{
    uint8_t status = L2C_ERR_NOT_UNDERSTOOD;
    switch (l_err)
    {
    case L2C_CB_CON_SUCCESS:
        // No error
        status = GAP_ERR_NO_ERROR;
        break;

    case L2C_CB_CON_LEPSM_NOT_SUPP:
        // LEPSM is not supported by peer device
        status = L2C_ERR_LEPSM_NOT_SUPP;
        break;

    case L2C_CB_CON_NO_RES_AVAIL:
        // No resources available
        status = L2C_ERR_NO_RES_AVAIL;
        break;

    case L2C_CB_CON_INS_AUTH:
        // Insufficient authentication
        status = L2C_ERR_INSUFF_AUTHEN;
        break;

    case L2C_CB_CON_INS_AUTHOR:
        // Insufficient authorization
        status = L2C_ERR_INSUFF_AUTHOR;
        break;

    case L2C_CB_CON_INS_EKS:
        // Insufficient encription key size
        status = L2C_ERR_INSUFF_ENC_KEY_SIZE;
        break;

    case L2C_CB_CON_INS_ENCRYPTION:
        // Insufficient encription
        status = L2C_ERR_INSUFF_ENC;
        break;

    case L2C_CB_CON_INVALID_SRC_CID:
        // Invalid Source CID
        status = L2C_ERR_INVALID_CID;
        break;

    case L2C_CB_CON_SRC_CID_ALREADY_ALLOC:
        // Source CID Already allocated
        status =  L2C_ERR_CID_ALREADY_ALLOC;
        break;

    case L2C_CB_CON_UNACCEPTABLE_PARAM :
        // Unacceptable parameters
        status = GAP_ERR_INVALID_PARAM;
        break;

    default:
        // status is L2C_ERR_NOT_UNDERSTOOD
        break;
    }

    return (status);
}
#endif // (BLE_LECB)


/*
 * GLOBAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

void l2cc_sig_send_cmd_reject(uint8_t conidx, uint8_t pkt_id, uint16_t reason,
                              uint16_t opt1, uint16_t opt2)
{
    struct l2cc_reject *reject = L2CC_SIG_PDU_ALLOC(conidx, L2C_CODE_REJECT, KE_BUILD_ID(TASK_L2CC, conidx), l2cc_reject);
    reject->reason = reason;

    switch (reason)
    {
    // The 2-octet Data field represents the maximum signaling MTU
    case L2C_MTU_SIG_EXCEEDED:
    {
        reject->opt_len = 1;
        co_write16p(reject->opt, opt1);
    }
    break;

    case L2C_INVALID_CID:
    {
        // The 4 octets contain the local (first) and remote (second) channel endpoints
        reject->opt_len = 2;
        co_write16p(reject->opt, opt1);
        co_write16p(reject->opt + 2, opt2);
    }
    break;

    case L2C_CMD_NOT_UNDERSTOOD:
    default:
    {
        // No data is used
        reject->opt_len = 0;
    }
    break;
    }

    // Save packet id
    reject->pkt_id = pkt_id;
    // Reason size plus options size
    reject->length = (reject->opt_len * 2) + 2;

    l2cc_pdu_send(reject);
}




/*
 * L2CAP SIGNALING MESSAGE HANDLERS FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @brief Handles reception of Signaling PDU Reject
 *
 * @param[in] conidx    Connection Index
 * @param[in] pdu       Pointer to the PDU Information
 *
 ****************************************************************************************
 */
static int l2c_code_reject_handler(uint8_t conidx, struct l2cc_reject *pdu)
{
    int msg_status =  KE_MSG_CONSUMED;

    uint8_t status = GAP_ERR_UNEXPECTED;

#if (BLE_LECB)
    uint8_t exp_pkt_id = 0;
    uint8_t operation = L2CC_NO_OP;
#endif // (BLE_LECB)
    struct l2cc_pdu_recv_ind *ind = L2CC_PDU_TO_IND(pdu);


    // remap the Error code
    switch (pdu->reason)
    {
    case L2C_CMD_NOT_UNDERSTOOD:
        status = L2C_ERR_NOT_UNDERSTOOD;
        break;
    case L2C_MTU_SIG_EXCEEDED:
        status = L2C_ERR_INVALID_MTU_EXCEED;
        break;
    case L2C_INVALID_CID:
        status = L2C_ERR_INVALID_CID;
        break;
    default: /* Nothing to do */
        break;
    }

#if (BLE_LECB)
    operation = l2cc_get_operation(conidx, L2CC_OP_SIG);
    switch (operation)
    {
    case L2CC_LECB_CONNECT:
    case L2CC_LECB_DISCONNECT:
    {
        // packet id is just after the operation code in command
        exp_pkt_id = *(((uint8_t *) l2cc_get_operation_ptr(conidx, L2CC_OP_SIG)) + 1);
    }
    break;

    default: /* Nothing to do */
        break;
    }

    if (exp_pkt_id == pdu->pkt_id)
    {
        // Clear timer
        ke_timer_clear(L2CC_SIGNALING_TRANS_TO_IND, KE_BUILD_ID(TASK_L2CC, conidx));

        if (operation == L2CC_LECB_CONNECT)
        {

            // Free the environment variable
            struct l2cc_lecb_connect_cmd *cmd =
                (struct l2cc_lecb_connect_cmd *) l2cc_get_operation_ptr(conidx, L2CC_OP_SIG);

            if (cmd != NULL)
            {
                struct l2cc_lecb_info *lecb = l2cc_lecb_find(conidx, L2CC_LECB_LOCAL_CID, cmd->local_cid);

                if (lecb != NULL)
                {
                    l2cc_lecb_free(conidx, lecb, false);
                }
            }

        }
        else if (operation == L2CC_LECB_DISCONNECT)
        {
            // Free the environment variable
            struct l2cc_lecb_disconnect_cmd *cmd =
                (struct l2cc_lecb_disconnect_cmd *) l2cc_get_operation_ptr(conidx, L2CC_OP_SIG);

            if (cmd != NULL)
            {
                struct l2cc_lecb_info *lecb = l2cc_lecb_find(conidx, L2CC_LECB_LOCAL_CID, cmd->local_cid);

                if (lecb != NULL)
                {
                    l2cc_lecb_free(conidx, lecb, true);
                }
            }
        }

        // completed operation
        l2cc_op_complete(conidx, L2CC_OP_SIG, status);
    }
    else
#endif // (BLE_LECB)
#if (BLE_ISO_MODE_0_PROTOCOL)
        // If peer device did not support audio mode 0 protocol, inform audio mode 0 module
        if ((status == L2C_ERR_INVALID_CID)
                && (pdu->opt[0] == AM0_L2C_CID_AUDIO_MODE_0))
        {
            ke_msg_forward(ind, KE_BUILD_ID(TASK_GAPC, conidx), KE_BUILD_ID(TASK_L2CC, conidx));
            msg_status = KE_MSG_NO_FREE;
        }
        else
#endif // (BLE_ISO_MODE_0_PROTOCOL)
        {
            // check if GAPC can handle it.
            ke_msg_forward(ind, KE_BUILD_ID(TASK_GAPC, conidx), KE_BUILD_ID(TASK_L2CC, conidx));
            msg_status = KE_MSG_NO_FREE;
        }

    return (msg_status);
}


/**
 ****************************************************************************************
 * @brief Handles reception of Signaling Connection Parameter Update request
 *
 * @param[in] conidx    Connection Index
 * @param[in] pdu       Pointer to the PDU Information
 *
 ****************************************************************************************
 */
static int l2c_code_conn_param_upd_req_handler(uint8_t conidx, struct l2cc_conn_param_upd_req *pdu)
{
    int msg_status =  KE_MSG_CONSUMED;
#if (BLE_PERIPHERAL)
    /* process the parameter request
     * decode the parameters
     * */
    if (gapc_get_role(conidx) == ROLE_SLAVE)
    {
        /* Param update indication only for MASTER*/
        l2cc_sig_send_cmd_reject(conidx,  pdu->pkt_id, L2C_CMD_NOT_UNDERSTOOD, 0, 0);
    }
    else
#endif // (BLE_PERIPHERAL)
    {


        // message handled in GAPC
        struct l2cc_pdu_recv_ind *ind = L2CC_PDU_TO_IND(pdu);
        ke_msg_forward(ind, KE_BUILD_ID(TASK_GAPC, conidx), KE_BUILD_ID(TASK_L2CC, conidx));
        msg_status = KE_MSG_NO_FREE;
    }
    return (msg_status);
}

/**
 ****************************************************************************************
 * @brief Handles reception of Signaling Connection Parameter Update response
 *
 * @param[in] conidx    Connection Index
 * @param[in] pdu       Pointer to the PDU Information
 *
 ****************************************************************************************
 */
static int l2c_code_conn_param_upd_resp_handler(uint8_t conidx, struct l2cc_conn_param_upd_resp *pdu)
{
    int msg_status =  KE_MSG_CONSUMED;
#if (BLE_CENTRAL)
    if (gapc_get_role(conidx) == ROLE_MASTER)
    {
        /* Parameter update response only for SLAVE  */
        l2cc_sig_send_cmd_reject(conidx,  pdu->pkt_id, L2C_CMD_NOT_UNDERSTOOD, 0, 0);
    }
    else
#endif // (BLE_CENTRAL)
    {
        // message handled in GAPC
        struct l2cc_pdu_recv_ind *ind = L2CC_PDU_TO_IND(pdu);
        ke_msg_forward(ind, KE_BUILD_ID(TASK_GAPC, conidx), KE_BUILD_ID(TASK_L2CC, conidx));
        msg_status = KE_MSG_NO_FREE;
    }

    return (msg_status);
}

#if (BLE_LECB)
/**
 ****************************************************************************************
 * @brief Handles reception of Signaling LECB Connection Creation request
 *
 * @param[in] conidx    Connection Index
 * @param[in] pdu       Pointer to the PDU Information
 *
 ****************************************************************************************
 */
static int l2c_code_le_cb_conn_req_handler(uint8_t conidx, struct l2cc_le_cb_conn_req *pdu)
{
    uint16_t status = GAP_ERR_NO_ERROR;
    ke_task_id_t app_task;
    uint8_t sec_lvl;

    // check if LE_PSM is registered
    status = gapm_le_psm_get_info(pdu->le_psm, conidx, &app_task, &sec_lvl);

    if (status == GAP_ERR_NOT_FOUND)
    {
        status = L2C_ERR_LEPSM_NOT_SUPP;
    }
    else
    {
        // Check encryption is not enabled
        if (!gapc_is_sec_set(conidx, GAPC_LK_ENCRYPTED))
        {
            if (L2CC_LECB_AUTH(sec_lvl) > GAP_LK_NO_AUTH)
            {
                // Check LTK availability
                if (gapc_is_sec_set(conidx, GAPC_LK_BONDED) && gapc_is_sec_set(conidx, GAPC_LK_LTK_PRESENT))
                {
                    status = L2C_ERR_INSUFF_ENC;
                }
                else
                {
                    status = L2C_ERR_INSUFF_AUTHEN;
                }
            }
        }
        // Link encrypted, LTK available
        else
        {
            // Authentication required
            if ((L2CC_LECB_AUTH(sec_lvl) > GAP_LK_UNAUTH) && (L2CC_LECB_AUTH(sec_lvl) > gapc_lk_sec_lvl_get(conidx)))
            {
                status = L2C_ERR_INSUFF_AUTHEN;
            }
            // Bigger encryption size key required
            else if (L2CC_LECB_EKS(sec_lvl) && (gapc_enc_keysize_get(conidx) < ATT_SEC_ENC_KEY_SIZE))
            {
                status = L2C_ERR_INSUFF_ENC_KEY_SIZE;
            }
        }
    }

    // convert status from HL error code to L2CAP LE Credit Error code
    status = l2cc_lecb_h2l_err(status);

    if (status == L2C_CB_CON_SUCCESS)
    {
        // perform a sanity check of connection parameters
        // check CID
        if (!L2C_IS_DYNAMIC_CID(pdu->scid))
        {
            status = L2C_CB_CON_INVALID_SRC_CID;
        }
        // Check max MTU / MPS
        if ((pdu->mtu < L2C_MIN_LE_MTUSIG) || (pdu->mps < L2C_MIN_LE_MTUSIG))
        {
            status = L2C_CB_CON_UNACCEPTABLE_PARAM;
        }
        // Check that the Channel Id is not used by another LECB connection
        if ((l2cc_lecb_find(conidx, L2CC_LECB_PEER_CID, pdu->scid) != NULL))
        {
            status = L2C_CB_CON_SRC_CID_ALREADY_ALLOC;
        }
        else
        {
            // register the connection
            if (gapm_lecb_register(pdu->le_psm, true) != GAP_ERR_NO_ERROR)
            {
                status = L2C_CB_CON_NO_RES_AVAIL;
            }
        }
    }

#ifndef TP_LE_CID_BI_01_C
    if (status == L2C_CB_CON_SUCCESS)
    {
        // allocate a request indication with credit based connection information
        struct l2cc_lecb_connect_req_ind *conn_ind = KE_MSG_ALLOC(
                    L2CC_LECB_CONNECT_REQ_IND, app_task, KE_BUILD_ID(TASK_L2CC, conidx),
                    l2cc_lecb_connect_req_ind);

        // allocate environment variable for new LECB connection
        struct l2cc_lecb_info *lecb = (struct l2cc_lecb_info *) ke_malloc(sizeof(struct l2cc_lecb_info), KE_MEM_ATT_DB);

        lecb->le_psm       = pdu->le_psm;
        lecb->local_cid    = 0;
        lecb->peer_cid     = pdu->scid;
        lecb->peer_mtu     = pdu->mtu;
        lecb->peer_mps     = pdu->mps;
        lecb->peer_credit  = pdu->initial_credits;
        lecb->task_id      = app_task;
        lecb->state        = L2CC_LECB_PEER_INIT_BIT;
        lecb->disc_reason  = GAP_ERR_NO_ERROR;
        lecb->pkt_id       = pdu->pkt_id;
        lecb->tx_sdu       = NULL;
        lecb->rx_sdu       = NULL;

        co_list_push_front(&(l2cc_env[conidx]->lecb_list), &(lecb->hdr));

        // Fill up paremeters
        conn_ind->le_psm       = lecb->le_psm;
        conn_ind->peer_cid     = lecb->peer_cid;
        conn_ind->peer_mtu     = lecb->peer_mtu;
        conn_ind->peer_mps     = lecb->peer_mps;

        // send the message
        ke_msg_send(conn_ind);
    }
#else
    if (status == L2C_CB_CON_SUCCESS)
    {
        l2cc_lecb_send_con_rsp(conidx, status, pdu->pkt_id, 0x41, 512, pdu->mps, pdu->mtu);
    }
#endif
    else
    {
        // Send connection response (L2C_CB_CON_INS_AUTH or L2C_CB_CON_INS_EKS)
        l2cc_lecb_send_con_rsp(conidx, status, pdu->pkt_id, 0, 0, 0, 0);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of Signaling LECB Connection Creation response
 *
 * @param[in] conidx    Connection Index
 * @param[in] pdu       Pointer to the PDU Information
 *
 ****************************************************************************************
 */
static int l2c_code_le_cb_conn_resp_handler(uint8_t conidx, struct l2cc_le_cb_conn_resp *pdu)
{
    struct l2cc_lecb_connect_cmd *cmd =
        (struct l2cc_lecb_connect_cmd *) l2cc_get_operation_ptr(conidx, L2CC_OP_SIG);

    if (cmd != NULL)
    {
        if ((pdu->pkt_id == cmd->pkt_id) && (cmd->operation == L2CC_LECB_CONNECT))
        {
            uint8_t status = L2C_ERR_INVALID_CID;
            struct l2cc_lecb_info *lecb = l2cc_lecb_find(conidx, L2CC_LECB_LOCAL_CID, cmd->local_cid);
            if (lecb != NULL)
            {
                // Clear timer
                ke_timer_clear(L2CC_SIGNALING_TRANS_TO_IND, KE_BUILD_ID(TASK_L2CC, conidx));
                // Align error code
                status = l2cc_sig_lecb_l2h_err(pdu->result);

                if (status == GAP_ERR_NO_ERROR)
                {
                    // perform a sanity check of connection parameters
                    // check CID
                    if ((!L2C_IS_DYNAMIC_CID(pdu->dcid))
                            // Check max MTU
                            || (pdu->mtu < L2C_MIN_LE_MTUSIG)
                            // Check MPS
                            || (pdu->mps < L2C_MIN_LE_MTUSIG)
                            // Check that the Channel Id is not used by another LECB connection
                            || (l2cc_lecb_find(conidx, L2CC_LECB_PEER_CID, pdu->dcid) != NULL))
                    {
                        status = GAP_ERR_UNEXPECTED;
                    }
                }

                if (status != GAP_ERR_NO_ERROR)
                {
                    l2cc_lecb_free(conidx, lecb, false);
                }
                else
                {
                    // allocate a request indication with credit based connection information
                    struct l2cc_lecb_connect_ind *conn_ind = KE_MSG_ALLOC(L2CC_LECB_CONNECT_IND,
                            lecb->task_id, KE_BUILD_ID(TASK_L2CC, conidx), l2cc_lecb_connect_ind);

                    // Register peer connection information
                    lecb->peer_cid    = pdu->dcid;
                    lecb->peer_mtu    = pdu->mtu;
                    lecb->peer_mps    = pdu->mps;
                    lecb->peer_credit = pdu->initial_credits;

                    SETB(lecb->state, L2CC_LECB_CONNECTED, true);

                    // Fill up paremeters
                    conn_ind->le_psm      = lecb->le_psm;
                    conn_ind->local_cid   = lecb->local_cid;
                    conn_ind->peer_credit = lecb->peer_credit;
                    conn_ind->peer_mtu    = lecb->peer_mtu;
                    conn_ind->peer_mps    = lecb->peer_mps;
                    conn_ind->status      = GAP_ERR_NO_ERROR;

                    // send the message
                    ke_msg_send(conn_ind);
                }
            }

            // complete operation
            l2cc_op_complete(conidx, L2CC_OP_SIG, status);
        }
    }

    return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles reception of Signaling LECB disconnect request
 *
 * @param[in] conidx    Connection Index
 * @param[in] pdu       Pointer to the PDU Information
 *
 ****************************************************************************************
 */
static int l2c_code_disconnection_req_handler(uint8_t conidx, struct l2cc_disconnection_req *pdu)
{
    // Search element
    struct l2cc_lecb_info *lecb = l2cc_lecb_find(conidx, L2CC_LECB_PEER_CID, pdu->scid);

    if (lecb != NULL)
    {
//        Ignore, not relevant to continue connection anyway
//        if (lecb->local_cid != pdu->dcid)
//        {
//           // SCID does not match
//            status = GAP_ERR_INVALID_PARAM;
//        }

        SETB(lecb->state, L2CC_LECB_CONNECTED, false);
        lecb->disc_reason = LL_ERR_REMOTE_USER_TERM_CON;

        // Send response
        l2cc_lecb_send_disc_rsp(conidx, pdu->pkt_id, lecb->peer_cid, lecb->local_cid);

        // free the connection
        l2cc_lecb_free(conidx, lecb, true);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of Signaling LECB disconnect response
 *
 * @param[in] conidx    Connection Index
 * @param[in] pdu       Pointer to the PDU Information
 *
 ****************************************************************************************
 */
static int l2c_code_disconnection_resp_handler(uint8_t conidx, struct l2cc_disconnection_resp *pdu)
{
    struct l2cc_lecb_disconnect_cmd *cmd =
        (struct l2cc_lecb_disconnect_cmd *) l2cc_get_operation_ptr(conidx, L2CC_OP_SIG);

    if (cmd != NULL)
    {
        // Check packet ID
        if ((pdu->pkt_id == cmd->pkt_id) && (cmd->operation == L2CC_LECB_DISCONNECT))
        {
            uint8_t status = GAP_ERR_NOT_FOUND;

//        Ignore, not relevant to continue connection anyway
//        if ((lecb->local_cid != pdu->dcid) || (lecb->peer_cid != pdu->scid))
//        {
//           // SCID does not match
//            status = GAP_ERR_INVALID_PARAM;
//        }

            struct l2cc_lecb_info *lecb = l2cc_lecb_find(conidx, L2CC_LECB_LOCAL_CID, cmd->local_cid);

            if (lecb != NULL)
            {
                // free the connection
                l2cc_lecb_free(conidx, lecb, true);

                status = GAP_ERR_NO_ERROR;
            }

            // Clear transaction timer
            ke_timer_clear(L2CC_SIGNALING_TRANS_TO_IND, KE_BUILD_ID(TASK_L2CC, conidx));

            // The  status should be OK if disconnection response was sent by the peer device
            // send complete event
            l2cc_op_complete(conidx, L2CC_OP_SIG, status);
        }
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of Signaling LECB Channel Credit number update indication
 *
 * @param[in] conidx    Connection Index
 * @param[in] pdu       Pointer to the PDU Information
 *
 ****************************************************************************************
 */
static int l2c_code_le_flow_control_credit_handler(uint8_t conidx, struct l2cc_le_flow_control_credit *pdu)
{
    // update number of credits
    struct l2cc_lecb_info *lecb = l2cc_lecb_find(conidx, L2CC_LECB_PEER_CID, pdu->cid);
    if (lecb != NULL)
    {
        if (GETB(lecb->state, L2CC_LECB_CONNECTED))
        {
            // Check if exceeds credit count
            if (pdu->credits > (L2C_LECB_MAX_CREDIT - lecb->peer_credit))
            {
                l2cc_lecb_init_disconnect(conidx, lecb, L2C_ERR_CREDIT_ERROR);
            }
            else if (pdu->credits > 0)
            {
                // allocate a request indication
                struct l2cc_lecb_add_ind *add_ind = KE_MSG_ALLOC(L2CC_LECB_ADD_IND,
                                                    lecb->task_id, KE_BUILD_ID(TASK_L2CC, conidx), l2cc_lecb_add_ind);

                // Save new credit count
                lecb->peer_credit         += pdu->credits;

                // Fill up indication
                add_ind->local_cid         = lecb->local_cid;
                add_ind->peer_added_credit = pdu->credits;
                // send the message
                ke_msg_send(add_ind);

                if (GETB(lecb->state, L2CC_LECB_TX_WAIT))
                {
                    ASSERT_INFO((lecb->tx_sdu != NULL), conidx, lecb->state);
                    SETB(lecb->state, L2CC_LECB_TX_WAIT, false);

                    // push SDU TX message at end of TX queue;
                    co_list_push_back(&(l2cc_env[conidx]->tx_queue), &(ke_param2msg(lecb->tx_sdu)->hdr));

                    // inform l2cm that buffer tx should be continued;
                    l2cm_tx_status(conidx, true);
                }
            }
        }
    }

    return (KE_MSG_CONSUMED);
}
#endif // (BLE_LECB)

/// The default PDU handlers
const l2cc_sig_pdu_handler_t l2cc_sig_handlers[] =
{
    { L2C_CODE_REJECT, (l2cc_sig_func_t) l2c_code_reject_handler                 },
    { L2C_CODE_CONN_PARAM_UPD_REQ, (l2cc_sig_func_t) l2c_code_conn_param_upd_req_handler     },
    { L2C_CODE_CONN_PARAM_UPD_RESP, (l2cc_sig_func_t) l2c_code_conn_param_upd_resp_handler    },
#if (BLE_LECB)
    { L2C_CODE_DISCONNECTION_REQ, (l2cc_sig_func_t) l2c_code_disconnection_req_handler      },
    { L2C_CODE_DISCONNECTION_RESP, (l2cc_sig_func_t) l2c_code_disconnection_resp_handler     },
    { L2C_CODE_LE_CB_CONN_REQ, (l2cc_sig_func_t) l2c_code_le_cb_conn_req_handler         },
    { L2C_CODE_LE_CB_CONN_RESP, (l2cc_sig_func_t) l2c_code_le_cb_conn_resp_handler        },
    { L2C_CODE_LE_FLOW_CONTROL_CREDIT, (l2cc_sig_func_t) l2c_code_le_flow_control_credit_handler },
#endif // (BLE_LECB)
};


int l2cc_sig_pdu_recv_handler(uint8_t conidx, struct l2cc_pdu *pdu)
{
    // means that message cannot be handled by ATTS module
    int msg_status = KE_MSG_CONSUMED;

    uint8_t cursor;
    att_func_t fhandler = NULL;

    // search PDU Handler
    for (cursor = 0 ; cursor < (sizeof(l2cc_sig_handlers) / sizeof(l2cc_sig_pdu_handler_t)) ; cursor++)
    {
        if (l2cc_sig_handlers[cursor].pdu_id == pdu->data.code)
        {
            fhandler = l2cc_sig_handlers[cursor].handler;
        }
    }

    // execute function handler
    if (fhandler != NULL)
    {
        // execute PDU Handler
        msg_status = fhandler(conidx, &(pdu->data.code));
    }
    else
    {
        l2cc_sig_send_cmd_reject(conidx, pdu->data.reject.pkt_id, L2C_CMD_NOT_UNDERSTOOD, 0, 0);
    }

    return (msg_status);
}



#endif // (BLE_L2CC)
/// @} L2CC_SIG
