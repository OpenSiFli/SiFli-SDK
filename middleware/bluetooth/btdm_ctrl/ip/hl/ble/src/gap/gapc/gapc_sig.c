/**
 ****************************************************************************************
 *
 * @file gapc_sig.c
 *
 * @brief Generic Access Profile Controller Signaling PDU Handler.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup GAPC_SIG Generic Access Profile Controller Signaling PDU Handler.
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"

#if (BLE_CENTRAL || BLE_PERIPHERAL)

#include "gapc_int.h"
#include "gapm.h"
#include "l2cc.h"

#include "ke_mem.h"
#include "ke_task.h"
#include "ke_timer.h"

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Format of a pdu handler function
typedef int (*gapc_sig_func_t)(uint8_t conidx, void *pdu);

/// Element of a pdu handler table.
typedef struct
{
    /// PDU identifier of the message
    uint8_t pdu_id;
    /// Pointer to the handler function for the pdu above.
    gapc_sig_func_t handler;

} gapc_sig_pdu_handler_t;




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


/*
 * GLOBAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */


#if (BLE_CENTRAL)
void gapc_sig_send_param_resp(uint8_t conidx, uint16_t result, uint8_t pkt_id)
{
    struct l2cc_conn_param_upd_resp *rsp = L2CC_SIG_PDU_ALLOC(conidx, L2C_CODE_CONN_PARAM_UPD_RESP,
                                           KE_BUILD_ID(TASK_GAPC, conidx), l2cc_conn_param_upd_resp);

    rsp->response    = result;
    rsp->pkt_id      = pkt_id;

    l2cc_pdu_send(rsp);
}
#endif // (BLE_CENTRAL)


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
    uint8_t exp_pkt_id = 0;
    uint8_t operation = GAPC_NO_OP;

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

    operation = gapc_get_operation(conidx, GAPC_OP_LINK_INFO);

    if (operation == GAPC_UPDATE_PARAMS)
    {
        // packet id is just after the operation code in command
        exp_pkt_id = *(((uint8_t *) gapc_get_operation_ptr(conidx, GAPC_OP_LINK_INFO)) + 1);


        if (exp_pkt_id == pdu->pkt_id)
        {
            // send complete event
            gapc_send_complete_evt(conidx, GAPC_OP_LINK_INFO, status);
        }
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
#if (BLE_CENTRAL)
    /* perform a sanity check of connection parameters */
    if (gapc_param_update_sanity(pdu->intv_max,
                                 pdu->intv_min,
                                 pdu->latency,
                                 pdu->timeout))
    {
        // send a command to request update of connection parameters
        struct gapc_param_update_cmd *cmd = KE_MSG_ALLOC(GAPC_PARAM_UPDATE_CMD,
                                            KE_BUILD_ID(TASK_GAPC, conidx), KE_BUILD_ID(TASK_GAPC, conidx), gapc_param_update_cmd);

        /* fill up the parameters */
        cmd->operation = GAPC_UPDATE_PARAMS;
        cmd->intv_max = pdu->intv_max;
        cmd->intv_min = pdu->intv_min;
        cmd->latency  = pdu->latency;
        cmd->time_out = pdu->timeout;
        cmd->pkt_id = pdu->pkt_id;
        /* send the message */
        ke_msg_send(cmd);

        // Clear parameter request feature in the environment because not supported by peer
        gapc_env[conidx]->features &= ~(GAPC_CONN_PARAM_REQ_FEAT_MASK);
    }
    else
    {
        /* construct and send immediately the parameter update response */
        gapc_sig_send_param_resp(conidx, L2C_CONN_PARAM_REJECT, pdu->pkt_id);
    }
#endif // (BLE_CENTRAL)

    return (KE_MSG_CONSUMED);
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
#if (BLE_PERIPHERAL)
    if (gapc_get_operation(conidx, GAPC_OP_LINK_UPD) == GAPC_UPDATE_PARAMS)
    {
        struct gapc_param_update_cmd *cmd =
            (struct gapc_param_update_cmd *) gapc_get_operation_ptr(conidx, GAPC_OP_LINK_UPD);
        if (pdu->pkt_id == cmd->pkt_id)
        {
            uint8_t status = ((pdu->response != L2C_CONN_PARAM_ACCEPT) ? GAP_ERR_REJECTED : GAP_ERR_NO_ERROR);
            // inform about request status
            gapc_send_complete_evt(conidx, GAPC_OP_LINK_UPD, status);

            /* Stop timer */
            ke_timer_clear(GAPC_PARAM_UPDATE_TO_IND, KE_BUILD_ID(TASK_GAPC, conidx));
        }
    }
#endif // (BLE_PERIPHERAL)

    return (KE_MSG_CONSUMED);
}


/// The default PDU handlers
const gapc_sig_pdu_handler_t gapc_sig_handlers[] =
{
    { L2C_CODE_REJECT, (gapc_sig_func_t) l2c_code_reject_handler                 },
    { L2C_CODE_CONN_PARAM_UPD_REQ, (gapc_sig_func_t) l2c_code_conn_param_upd_req_handler     },
    { L2C_CODE_CONN_PARAM_UPD_RESP, (gapc_sig_func_t) l2c_code_conn_param_upd_resp_handler    },
};


int gapc_sig_pdu_recv_handler(uint8_t conidx, struct l2cc_pdu *pdu)
{
    // means that message cannot be handled by ATTS module
    int msg_status = KE_MSG_CONSUMED;
    uint8_t cursor;
    att_func_t fhandler = NULL;

    // search PDU Handler
    for (cursor = 0 ; cursor < (sizeof(gapc_sig_handlers) / sizeof(gapc_sig_pdu_handler_t)) ; cursor++)
    {
        if (gapc_sig_handlers[cursor].pdu_id == pdu->data.code)
        {
            fhandler = gapc_sig_handlers[cursor].handler;
        }
    }

    // execute function handler
    if (fhandler != NULL)
    {
        // execute PDU Handler
        msg_status = fhandler(conidx, &(pdu->data.code));
    }

    return (msg_status);
}


#endif // (BLE_CENTRAL || BLE_PERIPHERAL)
/// @} GAPC_SIG
