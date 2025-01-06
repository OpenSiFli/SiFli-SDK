/**
 ****************************************************************************************
 *
 * @file llc_cte.c
 *
 * @brief Handles the constant tone extension procedure.
 *
 * Copyright (C) RivieraWaves 2009-2018
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
#if (BLE_CON_CTE_REQ | BLE_CON_CTE_RSP)
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

#include "hci.h"         // For HCI handler

#include "em_map.h"
#include "reg_access.h"
#include "reg_em_ble_rx_cte_desc.h"



/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// CTE operation indication structure definition
/*@TRACE*/
struct llc_op_cte_ind
{
    /// procedure information
    llc_procedure_t proc;
} ;


/*
 * DEFINES
 ****************************************************************************************
 */

/*@TRACE*/
enum llc_cte_state
{
    /// Start CTE procedure
    LLC_CTE_PROC_START,
    /// Wait for CTE response
    LLC_CTE_WAIT_PEER_CTE_RSP,
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

#if BLE_CON_CTE_REQ
/**
 ****************************************************************************************
 * Continue execution of local procedure
 *
 * @param[in] link_id Link identifier
 * @param[in] status  Status of the operation
 ****************************************************************************************
 */

__STATIC void llc_loc_cte_proc_continue(uint8_t link_id, uint8_t status)
{
    /// Gets the LLC environment dedicated to this link
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];
    /// retrieve procedure parameters
    struct llc_op_cte_ind *param = (struct llc_op_cte_ind *) llc_proc_get(link_id, LLC_PROC_LOCAL);
    bool finished = true;

    switch (llc_proc_state_get(&param->proc))
    {
    /// Procedure started
    case LLC_CTE_PROC_START:
        /*
         * @startuml llc_cte_start.png
         * title : CTE procedure start (CTE_PROC_START)
         * participant LLC
         * participant LLD
         * LLC -> LLC: llc_loc_cte_proc_continue(START)
         * hnote over LLC #aqua: state = WAIT_PEER_CTE_RSP
         * activate LLC
         *     LLC --> LLD : LLCP_CTE_REQ
         * deactivate LLC
         * note over LLC: Start LLCP timeout
         * @enduml
         */
        if (llc_env_ptr->cte_rx.req_active)
        {
            {
                // Send LL_CTE_REQ
                struct ll_cte_req pdu;
                pdu.op_code = LL_CTE_REQ_OPCODE;
                pdu.cte_len_type = 0;
                SETF(pdu.cte_len_type, MIN_CTE_LEN_REQ, llc_env_ptr->cte_rx.req_cte_len);
                SETF(pdu.cte_len_type, CTE_TYPE_REQ, llc_env_ptr->cte_rx.req_cte_type);
                llc_llcp_send(link_id, (union llcp_pdu *) &pdu, NULL);
            }

            llc_proc_state_set(&param->proc, link_id, LLC_CTE_WAIT_PEER_CTE_RSP);
            finished = false;

            // Start the LLCP Response TO
            llc_proc_timer_set(link_id, LLC_PROC_LOCAL, true);
        }
        break;

    /// Peer answer
    case LLC_CTE_WAIT_PEER_CTE_RSP:
    {
        /*
         * @startuml llc_cte_wait_peer_rsp.png
         * title : CTE procedure response (CTE_PROC_RSP)
         * participant HCI
         * participant LLC
         *  LLC <-: ll_cte_rsp_handler
         * activate LLC
         *     LLC -> LLC:llc_loc_cte_proc_continue(PEER_CTE_RSP)
         *     note over LLC : Stop LLCP timeout
         *     LLC --> HCI : HCI_LE_CON_IQ_REP_EVT
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
        ASSERT_INFO_FORCE(0, link_id, llc_proc_state_get(&param->proc));
    }
    break;
    }

    if (finished)
    {
        // unregister procedure
        llc_proc_unreg(link_id, LLC_PROC_LOCAL);

        // Indicate CTE request inactive if the request is only sent once
        if (llc_env_ptr->cte_rx.cte_req_intv == 0)
            llc_env_ptr->cte_rx.req_active = false;
    }
}

/**
 ****************************************************************************************
 * @brief Handles the CTE error (UNKNOWN/REJECT).
 *
 * PlantUML CTE error description
 *
 * @startuml llc_cte_error_cb.png
 * title : CTE procedure error callback description
 * participant LLC
 * participant LLD
 * alt unknown received from peer
 *     LLD --> LLC : LLCP_UNKNOWN_RSP
 *     activate LLC
 *     LLC -> LLC: llc_cte_proc_err_cb(LLCP_UMKNOWN_RSP)
 *     note over LLC: Disable the CTE procedure
 * end
 * @enduml
 *
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC void llc_cte_proc_err_cb(uint8_t link_id, uint8_t error_type, void *param)
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
        if (rsp->unk_type == LL_CTE_REQ_OPCODE)
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
        if (reject_ext->rej_op_code == LL_CTE_REQ_OPCODE)
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
        uint16_t conhdl = BLE_LINKID_TO_CONHDL(link_id);

        // Send HCI LE CTE request failed event
        struct hci_le_cte_req_failed_evt *evt = KE_MSG_ALLOC(HCI_LE_EVENT, conhdl, HCI_LE_META_EVT_CODE, hci_le_cte_req_failed_evt);
        evt->subcode = HCI_LE_CTE_REQ_FAILED_EVT_SUBCODE;
        evt->status  = status;
        evt->conhdl  = conhdl;
        hci_send_2_host(evt);

        if (status == CO_ERROR_UNSUPPORTED_REMOTE_FEATURE)
        {
            llc_le_feature_set(link_id, BLE_FEAT_CON_CTE_RSP, false);
        }

        llc_loc_cte_proc_continue(link_id, status);
    }
}
#endif // BLE_CON_CTE_REQ


/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

#if BLE_CON_CTE_REQ
/**
 ****************************************************************************************
 * @brief Handles the reception of a LLCP CTE request
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
int ll_cte_req_handler(uint8_t link_id, struct ll_cte_req *pdu, uint16_t event_cnt)
{
    int status = CO_ERROR_NO_ERROR;
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];
    uint8_t min_cte_len = GETF(pdu->cte_len_type, MIN_CTE_LEN_REQ);
    uint8_t cte_type = GETF(pdu->cte_len_type, CTE_TYPE_REQ);

    // If values not in range
    if ((min_cte_len < CTE_LEN_MIN) || (min_cte_len > CTE_LEN_MAX) || (cte_type > CTE_TYPE_AOD_2US))
    {
        llc_ll_reject_ind_pdu_send(link_id, LL_CTE_REQ_OPCODE, CO_ERROR_INVALID_LMP_PARAM, NULL);
    }
    // If CTE responses are disabled or not currently configured to respond with the type and length of the requested CTE
    else if ((llc_env_ptr->cte_tx.en_status != CO_ERROR_NO_ERROR) || !(llc_env_ptr->cte_tx.cte_types & (1 << cte_type)))
    {
        uint8_t reason = (llc_env_ptr->cte_tx.en_status != CO_ERROR_NO_ERROR) ? llc_env_ptr->cte_tx.en_status : CO_ERROR_UNSUPPORTED_LMP_PARAM_VALUE;
        llc_ll_reject_ind_pdu_send(link_id, LL_CTE_REQ_OPCODE, reason, NULL);
    }
    // If Constant Tone Extensions are not allowed on the current transmitter PHY
    else if ((llc_env_ptr->con_params.tx_phy != PHY_1MBPS_VALUE) && (llc_env_ptr->con_params.tx_phy != PHY_2MBPS_VALUE))
    {
        llc_ll_reject_ind_pdu_send(link_id, LL_CTE_REQ_OPCODE, CO_ERROR_INVALID_LMP_PARAM, NULL);
    }
    else
    {
        // Configure the CTE Tx parameters
        lld_con_cte_tx_param_set(link_id, min_cte_len, cte_type);

        // Reply with LL_CTE_RSP
        struct ll_default pdu;
        pdu.op_code = LL_CTE_RSP_OPCODE;
        llc_llcp_send(link_id, (union llcp_pdu *) &pdu, NULL);
    }

    return (status);
}
#endif // BLE_CON_CTE_REQ

#if BLE_CON_CTE_RSP
/**
 ****************************************************************************************
 * @brief Handles the reception of a LLCP CTE response
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
int ll_cte_rsp_handler(uint8_t link_id, struct ll_default *pdu, uint16_t event_cnt)
{
    int status = CO_ERROR_NO_ERROR;

    // check that local procedure is the correct one
    if (llc_proc_id_get(link_id, LLC_PROC_LOCAL) != LLC_PROC_CTE)
    {
        status = CO_ERROR_LMP_PDU_NOT_ALLOWED;
    }

    if (status != CO_ERROR_LMP_PDU_NOT_ALLOWED)
    {
        llc_loc_cte_proc_continue(link_id, status);
    }

    return status;
}
#endif // BLE_CON_CTE_RSP

#if BLE_CON_CTE_REQ
/**
 ****************************************************************************************
 * @brief Handles the command HCI LE set connection CTE receive parameters.
 *
 * @param[in] link_id Link Identifier
 * @param[in] param   Pointer to the parameters of the message.
 * @param[in] opcode  HCI Operation code
 ****************************************************************************************
 */
int hci_le_set_con_cte_rx_param_cmd_handler(uint8_t link_id, struct hci_le_set_con_cte_rx_param_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];

    do
    {
        // check if state is Free or in disconnected state
        if (llc_is_disconnecting(link_id))
            break;

        status = CO_ERROR_INVALID_HCI_PARAM;

        if (param->sampl_en > 1)
            break;

        if ((param->sampl_en == 1)
                && ((param->slot_dur < SLOT_DUR_1US) || (param->slot_dur > SLOT_DUR_2US)
                    || (param->switching_pattern_len < MIN_SWITCHING_PATTERN_LEN)
                    || (param->switching_pattern_len > MAX_SWITCHING_PATTERN_LEN)))
            break;

#if BLE_AOA
        if ((param->switching_pattern_len > BLE_MAX_SW_PAT_LEN))
        {
            status = CO_ERROR_UNSUPPORTED;
            break;
        }
#endif // BLE_AOA

        status = CO_ERROR_NO_ERROR;

        // Indicate parameters have been received
        llc_env_ptr->cte_rx.param_received = true;

        // Sampling enable
        llc_env_ptr->cte_rx.sampl_en = param->sampl_en;

        // Configure driver
        if (param->sampl_en)
        {
            // Write antenna IDs to EM
            em_wr(&param->antenna_id[0], EM_BLE_RX_ANTENNA_ID_OFFSET, param->switching_pattern_len);

            // Store parameters
            llc_env_ptr->cte_rx.slot_dur = param->slot_dur;
            llc_env_ptr->cte_rx.switching_pattern_len = param->switching_pattern_len;

            // Enable CTE reception
            lld_con_cte_rx_en(link_id, llc_env_ptr->cte_rx.slot_dur, llc_env_ptr->cte_rx.switching_pattern_len);
        }
        else
        {
            // Disable CTE reception
            lld_con_cte_rx_dis(link_id);
        }

    }
    while (0);

    // Send the command complete event
    llc_cmd_cmp_send(link_id, opcode, status);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the command HCI LE connection CTE request enable parameters.
 *
 * @param[in] link_id Link Identifier
 * @param[in] param   Pointer to the parameters of the message.
 * @param[in] opcode  HCI Operation code
 ****************************************************************************************
 */
int hci_le_con_cte_req_en_cmd_handler(uint8_t link_id, struct hci_le_con_cte_req_en_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];

    do
    {
        ke_task_id_t llc_id = KE_BUILD_ID(TASK_LLC, link_id);

        // check if state is Free or in disconnected state
        if (llc_is_disconnecting(link_id))
            break;

        // If the Host issues this command before issuing the HCI_LE_Set_Connection_CTE_Receive_Parameters command at least once on the connection
        if (!llc_env_ptr->cte_rx.param_received)
            break;

        // If the Host issues this command and IQ sampling is disabled
        if (!llc_env_ptr->cte_rx.sampl_en)
            break;

        // If the Host issues this command when the receiver PHY for the connection is not a PHY that allows Constant Tone Extensions
        if (llc_env_ptr->con_params.rx_phy == PHY_CODED_VALUE)
            break;

        // If the Host issues this command with Enable set to 0x01 while a request is active
        if ((param->en == 1) && (llc_env_ptr->cte_rx.req_active))
            break;

#if (BT_53)
        // If the Host sets CTE_Request_Interval to a non-zero value less than connSubrateFactor x (connPeripheralLatency +1)
        if ((param->en == 1) && (param->cte_req_intv != 0) && (param->cte_req_intv < ((llc_env_ptr->con_params.latency + 1) * llc_env_ptr->con_params.subrate)))
            break;
#else // !(BT_53)
        // If the Host sets CTE_Request_Interval to a non-zero value less than or equal to connSlaveLatency
        if ((param->en == 1) && (param->cte_req_intv != 0) && (param->cte_req_intv <= llc_env_ptr->con_params.latency))
            break;
#endif // !(BT_53)

        status = CO_ERROR_UNSUPPORTED_REMOTE_FEATURE;

        // check if peer support feature
        if (!llc_le_feature_check(link_id, BLE_FEAT_CON_CTE_RSP))
            break;

        status = CO_ERROR_INVALID_HCI_PARAM;

        if (param->en > 1)
            break;

        if ((param->en == 1)
                && ((param->req_cte_len < CTE_LEN_MIN) || (param->req_cte_len > CTE_LEN_MAX)
                    || (param->req_cte_type > CTE_TYPE_AOD_2US)))
            break;

        status = CO_ERROR_NO_ERROR;

        // If a timer is ongoing
        if (llc_env_ptr->cte_rx.req_active && (llc_env_ptr->cte_rx.cte_req_intv > 0))
        {
            // Clear ongoing timer
            ke_timer_clear(LLC_CTE_REQ_TO, llc_id);
        }

        // Handle command
        if (param->en == 1)
        {
            struct llc_op_cte_ind *cte_req = KE_MSG_ALLOC(LLC_OP_CTE_IND, llc_id, llc_id, llc_op_cte_ind);
            llc_proc_init(&cte_req->proc, LLC_PROC_CTE, llc_cte_proc_err_cb);
            llc_proc_state_set(&cte_req->proc, link_id, LLC_CTE_PROC_START);
            ke_msg_send(cte_req);

            // Indicate CTE request active
            llc_env_ptr->cte_rx.req_active = true;

            // If Host request periodic request
            if (param->cte_req_intv)
            {
                // If periodic request, start a timer
                ke_timer_set(LLC_CTE_REQ_TO, llc_id, param->cte_req_intv * llc_env_ptr->con_params.interval * 10 / 8);
            }

            // Store parameters
            llc_env_ptr->cte_rx.req_cte_len = param->req_cte_len;
            llc_env_ptr->cte_rx.cte_req_intv = param->cte_req_intv;
            llc_env_ptr->cte_rx.req_cte_type = param->req_cte_type;
        }
        else
        {
            // Indicate CTE request inactive
            llc_env_ptr->cte_rx.req_active = false;

            // Clear parameters
            llc_env_ptr->cte_rx.req_cte_len = 0;
            llc_env_ptr->cte_rx.cte_req_intv = 0;
            llc_env_ptr->cte_rx.req_cte_type = 0;
        }

    }
    while (0);

    // Send the command complete event
    llc_cmd_cmp_send(link_id, opcode, status);

    return (KE_MSG_CONSUMED);
}
#endif // BLE_CON_CTE_REQ

#if BLE_CON_CTE_RSP
/**
 ****************************************************************************************
 * @brief Handles the command HCI LE set connection CTE transmit parameters.
 *
 * @param[in] link_id Link Identifier
 * @param[in] param   Pointer to the parameters of the message.
 * @param[in] opcode  HCI Operation code
 ****************************************************************************************
 */
int hci_le_set_con_cte_tx_param_cmd_handler(uint8_t link_id, struct hci_le_set_con_cte_tx_param_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];

    do
    {
        // check if state is Free or in disconnected state
        if (llc_is_disconnecting(link_id))
            break;

        status = CO_ERROR_INVALID_HCI_PARAM;

        if (param->cte_types & ~(CTE_TYPES_AOA_BIT | CTE_TYPES_AOD_1US_BIT | CTE_TYPES_AOD_2US_BIT))
            break;

        // Only check the length of the switching pattern when the CTE types include AoD
        if (param->cte_types & (CTE_TYPES_AOD_1US_BIT | CTE_TYPES_AOD_2US_BIT))
        {
            if ((param->switching_pattern_len < MIN_SWITCHING_PATTERN_LEN) || (param->switching_pattern_len > MAX_SWITCHING_PATTERN_LEN))
                break;

            if ((param->switching_pattern_len > BLE_MAX_SW_PAT_LEN))
            {
                status = CO_ERROR_UNSUPPORTED;
                break;
            }
        }

        // If Constant Tone Extension responses have already been enabled on the connection
        if (llc_env_ptr->cte_tx.en_status == CO_ERROR_NO_ERROR)
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        status = CO_ERROR_NO_ERROR;

        // Indicate parameters have been received
        llc_env_ptr->cte_tx.param_received = true;

        // Write antenna IDs to EM
        em_wr(&param->antenna_id[0], EM_BLE_TX_ANTENNA_ID_OFFSET, param->switching_pattern_len);

        // Store parameters
        llc_env_ptr->cte_tx.cte_types             = param->cte_types;
        llc_env_ptr->cte_tx.switching_pattern_len = param->switching_pattern_len;

    }
    while (0);

    // Send the command complete event
    llc_cmd_cmp_send(link_id, opcode, status);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the command HCI LE connection CTE response enable parameters.
 *
 * @param[in] link_id Link Identifier
 * @param[in] param   Pointer to the parameters of the message.
 * @param[in] opcode  HCI Operation code
 ****************************************************************************************
 */
int hci_le_con_cte_rsp_en_cmd_handler(uint8_t link_id, struct hci_le_con_cte_rsp_en_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];

    do
    {
        // check if state is Free or in disconnected state
        if (llc_is_disconnecting(link_id))
            break;

        // If the Host issues this command before issuing the HCI_LE_Set_Connection_CTE_Receive_Parameters command at least once on the connection
        if (!llc_env_ptr->cte_tx.param_received)
            break;

        // If the Host issues this command when the transmitter PHY for the connection is not a PHY that allows Constant Tone Extensions
        if (llc_env_ptr->con_params.tx_phy == PHY_CODED_VALUE)
            break;

        status = CO_ERROR_INVALID_HCI_PARAM;

        if (param->en > 1)
            break;

        status = CO_ERROR_NO_ERROR;

        // Handle command
        if (param->en == 1)
        {
            // Enable CTE responses
            lld_con_cte_tx_ant_switch_config(link_id, llc_env_ptr->cte_tx.switching_pattern_len, llc_env_ptr->cte_tx.cte_types);
            llc_env_ptr->cte_tx.en_status = CO_ERROR_NO_ERROR;
        }
        else
        {
            // Disable CTE responses
            llc_env_ptr->cte_tx.en_status = CO_ERROR_UNSUPPORTED_LMP_PARAM_VALUE;
        }

    }
    while (0);

    // Send the command complete event
    llc_cmd_cmp_send(link_id, opcode, status);

    return (KE_MSG_CONSUMED);
}
#endif // BLE_CON_CTE_RSP


#if BLE_CON_CTE_REQ
/**
 ****************************************************************************************
 * @brief Handles the CTE procedure indication message.
 *
 * PlantUML procedure description
 *
 * @startuml llc_cte_op_start.png
 * title : Data length procedure start
 * participant LLC
 *  --> LLC : LLC_OP_CTE_IND
 * LLC -> LLC: llc_op_cte_ind_handler()
 * activate LLC
 * hnote over LLC : LOC_PROC = Busy
 * LLC -> LLC: llc_cte_loc_continue(START)
 * activate LLC
 * note right LLC #lightgreen: See __llc_cte_loc_continue()__
 * deactivate LLC
 * deactivate LLC
 * @enduml
 *
 ****************************************************************************************
 */

int llc_op_cte_ind_handler(ke_msg_id_t const msgid, struct llc_op_cte_ind *param,
                           ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Current message status
    int msg_status = KE_MSG_CONSUMED;
    uint8_t link_id = KE_IDX_GET(dest_id);

    // Check if state in disconnected state
    if (llc_is_disconnecting(link_id))
    {
        // disconnection on-going no procedure started
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
        // execute CTE procedure
        llc_loc_cte_proc_continue(link_id, CO_ERROR_NO_ERROR);

    }
    return (msg_status);
}

/**
 ****************************************************************************************
 * @brief Handles the CTE request timeout to start CTE REQ/RSP procedure
 ****************************************************************************************
 */
int llc_cte_req_to_handler(ke_msg_id_t const msgid, void *param,
                           ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t link_id = KE_IDX_GET(dest_id);

    struct llc_env_tag *llc_env_ptr = llc_env[link_id];

    // Check if state in disconnected state
    if (llc_is_disconnecting(link_id))
    {
        // LLC is IDLE, discard the message
    }
    else
    {
        // Start procedure unless a CTE request still pending
        if (llc_proc_id_get(link_id, LLC_PROC_LOCAL) != LLC_PROC_CTE)
        {
            ke_task_id_t llc_id = KE_BUILD_ID(TASK_LLC, link_id);
            struct llc_op_cte_ind *cte_req = KE_MSG_ALLOC(LLC_OP_CTE_IND, llc_id, llc_id, llc_op_cte_ind);
            llc_proc_init(&cte_req->proc, LLC_PROC_CTE, llc_cte_proc_err_cb);
            llc_proc_state_set(&cte_req->proc, link_id, LLC_CTE_PROC_START);
            ke_msg_send(cte_req);
        }

        // If Host request periodic request
        if (llc_env_ptr->cte_rx.cte_req_intv)
        {
            // If periodic request, start a timer
            ke_timer_set(LLC_CTE_REQ_TO, KE_BUILD_ID(TASK_LLC, link_id), llc_env_ptr->cte_rx.cte_req_intv * llc_env_ptr->con_params.interval * 10 / 8);
        }
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the connection CTE RX indication message.
 ****************************************************************************************
 */
int lld_con_cte_rx_ind_handler(ke_msg_id_t const msgid, struct lld_con_cte_rx_ind *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t link_id = KE_IDX_GET(dest_id);

    struct llc_env_tag *llc_env_ptr = llc_env[link_id];

    if ((param->cte_time == 0) && (param->nb_iq_samp == 0))
    {
        uint16_t conhdl = BLE_LINKID_TO_CONHDL(link_id);

        // Send HCI LE CTE request failed event
        struct hci_le_cte_req_failed_evt *evt = KE_MSG_ALLOC(HCI_LE_EVENT, conhdl, HCI_LE_META_EVT_CODE, hci_le_cte_req_failed_evt);
        evt->subcode = HCI_LE_CTE_REQ_FAILED_EVT_SUBCODE;
        evt->status  = 0; // LL_CTE_RSP PDU received successfully but without a Constant Tone Extension field
        evt->conhdl  = conhdl;
        hci_send_2_host(evt);
    }
    else
    {
        // Send HCI LE connection IQ report event
        struct hci_le_con_iq_report_evt *evt = KE_MSG_ALLOC(HCI_LE_EVENT, BLE_LINKID_TO_CONHDL(link_id), HCI_LE_META_EVT_CODE, hci_le_con_iq_report_evt);
        evt->subcode            = HCI_LE_CON_IQ_REPORT_EVT_SUBCODE;
        evt->conhdl             = BLE_LINKID_TO_CONHDL(link_id);
        evt->rx_phy             = param->phy;
        evt->data_channel_idx   = param->channel_idx;
        evt->rssi               = param->rssi;
        evt->rssi_antenna_id    = param->rssi_antenna_id;
        evt->cte_type           = param->cte_type;
        evt->slot_dur           = llc_env_ptr->cte_rx.slot_dur;
        evt->pkt_status         = CTE_PKT_STAT_CRC_OK;
        evt->con_evt_cnt        = param->con_evt_cnt;
        evt->sample_cnt         = param->nb_iq_samp;
        em_rd(&evt->iq_sample[0], REG_EM_BLE_RX_CTE_DESC_ADDR_GET(param->em_rx_cte_desc_idx) + (EM_BLE_RXCTESAMPBUF_INDEX * 2), param->nb_iq_samp * 2);
        hci_send_2_host(evt);
    }

    // Release the Rx CTE descriptor
    em_ble_rxctecntl_rxdone_setf(param->em_rx_cte_desc_idx, 0);

    return (KE_MSG_CONSUMED);
}
#endif // BLE_CON_CTE_REQ

#if BLE_CON_CTE_REQ
void llc_cte_req_dis(uint8_t link_id)
{
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];

    // If periodic request, clear the timer
    ke_timer_clear(LLC_CTE_REQ_TO, KE_BUILD_ID(TASK_LLC, link_id));

    // Clear CTE request interval
    llc_env_ptr->cte_rx.cte_req_intv = 0;
}
#endif // BLE_CON_CTE_REQ

#if BLE_CON_CTE_RSP
void llc_cte_rsp_dis(uint8_t link_id)
{
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];

    // Disable CTE responses
    llc_env_ptr->cte_tx.en_status = CO_ERROR_DIFF_TRANSACTION_COLLISION;
}
#endif // BLE_CON_CTE_RSP


#endif // (BLE_CON_CTE_REQ | BLE_CON_CTE_RSP)
#endif // (BLE_PERIPHERAL || BLE_CENTRAL)
/// @} LLC_OP
