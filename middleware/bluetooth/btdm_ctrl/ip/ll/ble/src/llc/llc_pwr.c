/**
 ****************************************************************************************
 *
 * @file llc_pwr.c
 *
 * @brief LE Power Control Procedures
 *
 * Copyright (C) RivieraWaves 2019
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup LLC_PWR Power Control Procedures
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

#include "co_utils.h"    // For bit field manipulation

#include "ke_msg.h"      // Kernel message

#include "llc_int.h"    // Internal LLC API
#include "llc_llcp.h"   // Internal LLCP API

#include "lld.h"        // link layer driver definitions
#include "llm.h"        // link layer manager definitions

#include "hci.h"        // For HCI handler
#include "btdm_patch.h"

#if BLE_PWR_CTRL

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Power control exchange operation indication structure definition
/*@TRACE*/
struct llc_op_pwr_ctrl_ind
{
    /// procedure information
    llc_procedure_t proc;
    /// PHY on which power control is requested (see @enum le_phy_pwr)
    uint8_t phy;
    /// Reason for procedure (see @enum pwr_report_reason)
    uint8_t reason;
    /// Use to know if event should be reported to host
    bool evt_report;
    /// The responder's transmit power (in dBm)
    int8_t tx_pwr;
    /// The responder's transmit power flags (see @enum pwr_ctrl_flags)
    uint8_t flags;
    /// The requested or actual change in the recipient's power level (in dBm)
    int8_t delta;
    /// Acceptable power reduction
    uint8_t apr;
};


/*
 * DEFINES
 ****************************************************************************************
 */

/*@TRACE*/
enum llc_pwr_ctrl_state
{
    /// Start power control request procedure
    LLC_PWR_CTRL_PROC_START,
    /// Wait for peer's power control response
    LLC_PWR_CTRL_WAIT_PEER_RSP,
};


/*
 * MACROS
 ****************************************************************************************
 */

/// Fetch power control flags
#define LLC_PWR_FLAGS_GET(tx_pwr) (((int8_t)lld_tx_power_dbm_get(rwip_rf.txpwr_max, rwip_rf.txpwr_max_mod) == (int8_t)(tx_pwr))?BLE_PWR_CTRL_MAX_BIT:0) \
        | (((int8_t)lld_tx_power_dbm_get(rwip_rf.txpwr_min, rwip_rf.txpwr_min_mod) == (int8_t)(tx_pwr))?BLE_PWR_CTRL_MIN_BIT:0)

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/*
 * LOCAL FUNCTIONS DECLARATIONS
 ****************************************************************************************
 */


/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Sends the power control request pdu.
 *
 ****************************************************************************************
 */
__STATIC void llc_ll_pwr_ctrl_req_pdu_send(uint8_t link_id, uint8_t phy_mask, int8_t delta, int8_t tx_pwr)
{
    struct ll_pwr_ctrl_req pdu;

    pdu.op_code = LL_PWR_CTRL_REQ_OPCODE;
    pdu.phy_mask = phy_mask;
    pdu.delta = delta;
    pdu.tx_pwr = tx_pwr;

    llc_llcp_send(link_id, (union llcp_pdu *) &pdu, NULL);
}

/**
 ****************************************************************************************
 * @brief Sends the power control response pdu.
 *
 ****************************************************************************************
 */
__STATIC void llc_ll_pwr_ctrl_rsp_pdu_send(uint8_t link_id, uint8_t flags, int8_t delta, int8_t tx_pwr, uint8_t apr)
{
    struct ll_pwr_ctrl_rsp pdu;

    pdu.op_code = LL_PWR_CTRL_RSP_OPCODE;
    pdu.flags = flags;
    pdu.delta = delta;
    pdu.tx_pwr = tx_pwr;
    pdu.apr = apr;

    llc_llcp_send(link_id, (union llcp_pdu *) &pdu, NULL);
}

/**
 ****************************************************************************************
 * @brief Sends the power control response pdu.
 *
 ****************************************************************************************
 */
__STATIC void llc_ll_pwr_change_ind_pdu_send(uint8_t link_id, uint8_t phy_mask, uint8_t flags, int8_t delta, int8_t tx_pwr)
{
    struct ll_pwr_change_ind pdu;

    pdu.op_code = LL_PWR_CHANGE_IND_OPCODE;
    pdu.phy_mask = phy_mask;
    pdu.flags = flags;
    pdu.delta = delta;
    pdu.tx_pwr = tx_pwr;

    llc_llcp_send(link_id, (union llcp_pdu *) &pdu, NULL);
}

/**
 ****************************************************************************************
 * @brief Send HCI_LE_TX_POWER_REPORTING_EVT to host
 *
 * @param[in] link_id    Link identifier
 * @param[in] status     Status
 ****************************************************************************************
 */
__STATIC void llc_hci_le_tx_power_reporting_evt_send(uint8_t link_id, uint8_t status, uint8_t reason, uint8_t phy, int8_t tx_pwr, uint8_t flags, int8_t delta)
{
    uint16_t conhdl = BLE_LINKID_TO_CONHDL(link_id);

    // Send the HCI LE Transmit Power Reporting event
    struct hci_le_tx_power_rep_evt *event = KE_MSG_ALLOC(HCI_LE_EVENT, conhdl, 0, hci_le_tx_power_rep_evt);
    event->subcode  = HCI_LE_TX_POWER_REPORTING_EVT_SUBCODE;
    event->status   = status;
    event->conhdl   = conhdl;
    event->reason   = reason;
    event->phy      = phy;
    event->tx_pwr   = tx_pwr;
    event->flags    = flags;
    event->delta    = delta;

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
__STATIC void llc_pwr_ctrl_proc_continue(uint8_t link_id, uint8_t status)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(PROC_CONTINUE_PATCH_TYPE, LLC_PWR_CTRL_PROC_CONTINUE_C_FUN_BIT, link_id, status);
    /// retrieve procedure parameters
    struct llc_op_pwr_ctrl_ind *param = (struct llc_op_pwr_ctrl_ind *) llc_proc_get(link_id, LLC_PROC_LOCAL);
    bool finished = true;
    //Get environment pointer
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];

    switch (llc_proc_state_get(&param->proc))
    {
    /// Procedure started
    case LLC_PWR_CTRL_PROC_START:
        /*
         * @startuml llc_pwr_ctrl_start.png
         * title : Power control procedure start (PWR_CTRL_PROC_START)
         * participant LLC
         * participant LLD
         * LLC -> LLC: llc_pwr_ctrl_proc_continue(START)
         * hnote over LLC #aqua: state = PWR_CTRL_WAIT_PEER_RSP
         * activate LLC
         *     LLC --> LLD : LL_PWR_CTRL_REQ
         * deactivate LLC
         * note over LLC: Start LLCP timeout
         * @enduml
         */
    {
        int8_t tx_pwr = lld_con_tx_power_get(link_id, param->phy);
        uint8_t phy_mask = co_phypwr_value_to_mask[param->phy];

        llc_ll_pwr_ctrl_req_pdu_send(link_id, phy_mask, param->delta, tx_pwr);

        // On procedure start, the latest APR is unknown
        param->apr = PWR_CTRL_APR_UNKNOWN;

        llc_proc_state_set(&param->proc, link_id, LLC_PWR_CTRL_WAIT_PEER_RSP);
        finished = false;

        // Start the LLCP Response TO
        llc_proc_timer_set(link_id, LLC_PROC_LOCAL, true);
    }
    break;

    /// Peer answer
    case LLC_PWR_CTRL_WAIT_PEER_RSP:
    {
        /*
         * @startuml llc_pwr_ctrl_wait_peer_rsp.png
         * title : Power control response (PWR_CTRL_PROC_START)
         * participant HCI
         * participant LLC
         *  LLC <-: ll_pwr_ctrl_rsp_handler
         * activate LLC
         *     LLC -> LLC:llc_pwr_ctrl_proc_continue(PWR_CTRL_WAIT_PEER_RSP)
         *     note over LLC : Stop LLCP timeout
         *     hnote over LLC : LOC_PROC = Busy
         * deactivate LLC
         * @enduml
         */
        // Got the response for power control procedure locally initiated
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
        // process the power control information if no error status
        if (CO_ERROR_NO_ERROR == status)
        {
            uint8_t rate_idx = co_phypwr_to_rate[param->phy];

            // Save the senders transmit power level for the PHY indicated
            lld_con_remote_tx_pwr_set(link_id, rate_idx, param->tx_pwr, param->flags);

            // Check for acceptable power reduction, use an APR target to avoid power control hysteresis at rssi_low_thr.
            if ((PWR_CTRL_APR_UNKNOWN != param->apr) && (param->apr > RW_RSSI_APR_TARGET))
            {
                // Get current Tx power and determine new Tx power
                int8_t old_tx_pwr = llc_env_ptr->pwr.loc_tx_pwr[rate_idx];
                // Adjust local power based on APR.
                int8_t new_tx_pwr = lld_con_tx_power_adj(link_id, param->phy, RW_RSSI_APR_TARGET - param->apr);

                // Send change indication to peer if indications enabled
                if (new_tx_pwr != old_tx_pwr)
                {
                    // Get delta in transmit power and min/max flags
                    uint8_t flags = LLC_PWR_FLAGS_GET(new_tx_pwr);

                    // Set delta if applicable
                    int8_t delta = (BLE_PWR_CTRL_UNUSED != old_tx_pwr) ? (new_tx_pwr - old_tx_pwr) : 0;

                    // Send HCI event if local power has changed and reporting enabled.
                    if (llc_env_ptr->pwr.loc_tx_pwr_rep_en)
                    {
                        llc_hci_le_tx_power_reporting_evt_send(link_id, status, BLE_PWR_LOC_TX_CHG, param->phy, new_tx_pwr, flags, delta);
                    }

                    // Send change indication to peer if indications enabled
                    if (llc_env_ptr->pwr.loc_tx_pwr_chg_ind_en)
                    {
                        uint8_t phy_mask = co_phypwr_value_to_mask[param->phy];

                        llc_ll_pwr_change_ind_pdu_send(link_id, phy_mask, flags, delta, new_tx_pwr);
                    }

                    llc_env_ptr->pwr.loc_tx_pwr[rate_idx] = new_tx_pwr;
                }
            }
        }

        // check if power reporting event has to be sent to host
        if (param->evt_report)
        {
            llc_hci_le_tx_power_reporting_evt_send(link_id, status, param->reason, param->phy, param->tx_pwr, param->flags, param->delta);
        }

        // check if peer supports change indication feature
        if (llc_le_feature_check(link_id, BLE_FEAT_POWER_CHANGE_IND))
        {
            // Reception of a power control request enables autonomous power change indications from the peer
            llc_env_ptr->pwr.rem_tx_pwr_chg_ind_en = true;
        }

        SETB(llc_env_ptr->flow_ctrl, LLC_HCI_PWR_CTRL_REQ, false);

        // unregister procedure
        llc_proc_unreg(link_id, LLC_PROC_LOCAL);
    }
}

/**
 ****************************************************************************************
 * @brief Handles error during power control procedure (UNKNOWN/REJECT).
 *
 * PlantUML Power control procedure error description
 *
 * @startuml llc_pwr_ctrl_error_cb.png
 * title : Power control procedure error callback description
 * participant LLC
 * participant LLD
 * alt unknown received from peer
 *     LLD --> LLC : LLCP_UNKNOWN_RSP
 *     activate LLC
 *     LLC -> LLC: llc_pwr_ctrl_proc_err_cb(LLCP_UMKNOWN_RSP)
 *     note over LLC: Disable the power control request procedure
 * end
 * @enduml
 *
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC void llc_pwr_ctrl_proc_err_cb(uint8_t link_id, uint8_t error_type, void *param)
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
        if (rsp->unk_type == LL_PWR_CTRL_REQ_OPCODE)
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
        if (reject_ext->rej_op_code == LL_PWR_CTRL_REQ_OPCODE)
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
            llc_le_feature_set(link_id, BLE_FEAT_POWER_CONTROL_REQ, false);
        }

        llc_pwr_ctrl_proc_continue(link_id, status);
    }
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
 *  @brief Handles the reception of an LLCP power control request
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
int ll_pwr_ctrl_req_handler(uint8_t link_id, struct ll_pwr_ctrl_req *pdu, uint16_t event_cnt)
{
    uint8_t status = CO_ERROR_NO_ERROR;

    if ((NB_ONE_BITS(pdu->phy_mask) != 1) || (BLE_PWR_CTRL_UNUSED == pdu->tx_pwr))
    {
        status = CO_ERROR_INVALID_LMP_PARAM;
    }
    else if (pdu->phy_mask & ~PHY_PWR_ALL)
    {
        status = CO_ERROR_UNSUPPORTED_LMP_PARAM_VALUE;
    }
#if !(BLE_PHY_2MBPS_SUPPORT)
    else if (PHY_PWR_2MBPS_BIT & pdu->phy_mask)
    {
        status = CO_ERROR_UNSUPPORTED_LMP_PARAM_VALUE;
    }
#endif //(BLE_PHY_2MBPS_SUPPORT)
#if !(BLE_PHY_CODED_SUPPORT)
    else if ((PHY_PWR_S8_CODED_BIT & pdu->phy_mask) || (PHY_PWR_S2_CODED_BIT & pdu->phy_mask))
    {
        status = CO_ERROR_UNSUPPORTED_LMP_PARAM_VALUE;
    }
#endif //(BLE_PHY_CODED_SUPPORT)
    else
    {
        //Get environment pointer
        struct llc_env_tag *llc_env_ptr = llc_env[link_id];
        // Determine PHY from mask
        uint8_t phy = co_phypwr_mask_to_value[pdu->phy_mask];
        uint8_t rate_idx = co_phypwr_to_rate[phy];

        uint8_t flags = 0;
        int8_t delta = 0;
        int8_t tx_pwr = 0;
        uint8_t apr = 0;

        if (CO_ERROR_LMP_COLLISION == llc_proc_collision_check(link_id, LLC_PROC_PWR_CTRL))
        {
            // On collision scenarios, the APR shall be set unknown
            apr = PWR_CTRL_APR_UNKNOWN;
        }
        else
        {
            // Fetch APR on the corresponding rate
            apr = lld_con_apr_get(link_id, co_phypwr_to_rate[phy]);
        }

        if (0 != pdu->delta)
        {
            // Get current Tx power and determine new Tx power
            int8_t old_tx_pwr = llc_env_ptr->pwr.loc_tx_pwr[rate_idx];
            int8_t new_tx_pwr = lld_con_tx_power_adj(link_id, phy, pdu->delta);

            // Set delta if applicable
            delta = (BLE_PWR_CTRL_UNUSED != old_tx_pwr) ? (new_tx_pwr - old_tx_pwr) : 0;

            // Send HCI event if local power has changed and reporting enabled.
            if ((new_tx_pwr != old_tx_pwr) && llc_env_ptr->pwr.loc_tx_pwr_rep_en)
            {
                llc_hci_le_tx_power_reporting_evt_send(link_id, status, BLE_PWR_LOC_TX_CHG, phy, new_tx_pwr, LLC_PWR_FLAGS_GET(new_tx_pwr), delta);
            }

            llc_env_ptr->pwr.loc_tx_pwr[rate_idx] = new_tx_pwr;
        }

        // Update record of the peer's transmit power, min/max flags unknown
        lld_con_remote_tx_pwr_set(link_id, rate_idx, pdu->tx_pwr, LLD_PWR_FLGS_UNKNOWN);

        // check if peer supports change indication feature
        if (llc_le_feature_check(link_id, BLE_FEAT_POWER_CHANGE_IND))
        {
            // Reception of a power control request enables autonomous power change indications to the peer
            llc_env_ptr->pwr.loc_tx_pwr_chg_ind_en = true;
        }

        // Local transmit power for response
        tx_pwr = llc_env_ptr->pwr.loc_tx_pwr[rate_idx];
        // Local transmit flags
        flags = LLC_PWR_FLAGS_GET(tx_pwr);

        // The procedure is initiated by the peer, reply and simply wait for Ack
        llc_ll_pwr_ctrl_rsp_pdu_send(link_id, flags, delta, tx_pwr, apr);
    }

    return status;
}

/**
 ****************************************************************************************
 *  @brief Handles the reception of an LLCP power control response
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
int ll_pwr_ctrl_rsp_handler(uint8_t link_id, struct ll_pwr_ctrl_rsp *pdu, uint16_t event_cnt)
{
    uint8_t status = CO_ERROR_NO_ERROR;

    // Check that local procedure is in progress for Power control request else reject
    if (llc_proc_id_get(link_id, LLC_PROC_LOCAL) != LLC_PROC_PWR_CTRL)
    {
        status = CO_ERROR_LMP_PDU_NOT_ALLOWED;
    }
    else
    {
        //Get environment pointer
        struct llc_env_tag *llc_env_ptr = llc_env[link_id];
        // retrieve procedure parameters
        struct llc_op_pwr_ctrl_ind *param = (struct llc_op_pwr_ctrl_ind *) llc_proc_get(link_id, LLC_PROC_LOCAL);

        // Ignore RFU bits
        pdu->flags &= (BLE_PWR_CTRL_MIN_BIT | BLE_PWR_CTRL_MAX_BIT);

        // Save procedure parameters received from the peer device
        param->flags = pdu->flags;
        param->tx_pwr = pdu->tx_pwr;
        param->delta = pdu->delta;
        param->apr = pdu->apr;

        // Report the event if there is a change
        if ((param->reason != BLE_PWR_HCI_REQ) && (llc_env_ptr->pwr.rem_tx_pwr_rep_en))
        {
            param->evt_report = (0 != pdu->delta);
        }

        // execute power control procedure
        llc_pwr_ctrl_proc_continue(link_id, CO_ERROR_NO_ERROR);
    }

    return status;
}

/**
 ****************************************************************************************
 *  @brief Handles the reception of an LLCP power change indication
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
int ll_pwr_change_ind_handler(uint8_t link_id, struct ll_pwr_change_ind *pdu, uint16_t event_cnt)
{
    //Get environment pointer
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];
    uint8_t status = CO_ERROR_NO_ERROR;

    uint8_t updated_phys[CO_RATE_MAX];
    int n_phy = 0;

    // Ignore RFU bits
    pdu->flags &= (BLE_PWR_CTRL_MIN_BIT | BLE_PWR_CTRL_MAX_BIT);

    /* The LE Power Change Ind PDU can have multiple bits set in the PHY mask. Update the
       specified peer transmit power level for each PHY indicated */
    if (GETB(pdu->phy_mask, PHY_PWR_1MBPS))
    {
        lld_con_remote_tx_pwr_set(link_id, CO_RATE_1MBPS, pdu->tx_pwr, pdu->flags);
        updated_phys[n_phy++] = PHY_PWR_1MBPS_VALUE;
    }
    if (GETB(pdu->phy_mask, PHY_PWR_2MBPS))
    {
        lld_con_remote_tx_pwr_set(link_id, CO_RATE_2MBPS, pdu->tx_pwr, pdu->flags);
        updated_phys[n_phy++] = PHY_PWR_2MBPS_VALUE;
    }
    if (GETB(pdu->phy_mask, PHY_PWR_S8_CODED))
    {
        lld_con_remote_tx_pwr_set(link_id, CO_RATE_125KBPS, pdu->tx_pwr, pdu->flags);
        updated_phys[n_phy++] = PHY_PWR_S8_CODED_VALUE;
    }
    if (GETB(pdu->phy_mask, PHY_PWR_S2_CODED))
    {
        lld_con_remote_tx_pwr_set(link_id, CO_RATE_500KBPS, pdu->tx_pwr, pdu->flags);
        updated_phys[n_phy++] = PHY_PWR_S2_CODED_VALUE;
    }

    if (llc_env_ptr->pwr.rem_tx_pwr_rep_en)
    {
        for (int i = 0; i < n_phy; i++) // for each PHY updated
        {
            // Send the HCI LE Transmit Power Reporting event
            llc_hci_le_tx_power_reporting_evt_send(link_id, status, BLE_PWR_REM_TX_CHG, updated_phys[i], pdu->tx_pwr, pdu->flags, pdu->delta);
        }
    }

    return status;
}

/**
 ****************************************************************************************
 * @brief Handles the LLD power control procedure request indication message
 ****************************************************************************************
 */
int lld_con_pwr_ctrl_ind_handler(ke_msg_id_t const msgid, struct lld_con_pwr_ctrl_ind *param,
                                 ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t link_id = KE_IDX_GET(dest_id);

    struct llc_env_tag *llc_env_ptr = llc_env[link_id];

    // Current message status
    int msg_status = KE_MSG_CONSUMED;

    // Check if state in disconnected state
    if (llc_is_disconnecting(link_id) || !llc_le_feature_check(link_id, BLE_FEAT_POWER_CONTROL_REQ))
    {
        // LLC is IDLE, or peer doesn't support feature, discard the message
    }
    else
    {
        // If procedure not already active
        if (!GETB(llc_env_ptr->flow_ctrl, LLC_HCI_PWR_CTRL_REQ))
        {
            if (llc_proc_id_get(link_id, LLC_PROC_LOCAL) == LLC_PROC_NONE)
            {
                ke_task_id_t llc_id = KE_BUILD_ID(TASK_LLC, link_id);
                // start power control procedure
                struct llc_op_pwr_ctrl_ind *pwr_ctrl = KE_MSG_ALLOC(LLC_OP_PWR_CTRL_IND, llc_id, llc_id, llc_op_pwr_ctrl_ind);
                llc_proc_init(&pwr_ctrl->proc, LLC_PROC_PWR_CTRL, llc_pwr_ctrl_proc_err_cb);
                pwr_ctrl->phy = co_rate_to_phypwr[param->rx_rate];
                pwr_ctrl->delta = param->delta;
                pwr_ctrl->evt_report = false; // event report decision based on delta response
                pwr_ctrl->reason = BLE_PWR_REM_TX_CHG;
                llc_proc_state_set(&pwr_ctrl->proc, link_id, LLC_PWR_CTRL_PROC_START);

                SETB(llc_env_ptr->flow_ctrl, LLC_HCI_PWR_CTRL_REQ, true);

                // store local procedure
                llc_proc_reg(link_id, LLC_PROC_LOCAL, &(pwr_ctrl->proc));
                // execute power control procedure
                llc_pwr_ctrl_proc_continue(link_id, CO_ERROR_NO_ERROR);
            }
            else
            {
                // process this message later
                msg_status = KE_MSG_SAVED;
            }
        }
    }

    return (msg_status);
}

/**
****************************************************************************************
* @brief Handles the LLD path loss change indication message
****************************************************************************************
*/
int lld_con_path_loss_change_ind_handler(ke_msg_id_t const msgid, struct lld_con_path_loss_change_ind *param,
        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint16_t conhdl = BLE_LINKID_TO_CONHDL(KE_IDX_GET(dest_id));

    // Send the HCI LE Path Threshold event
    struct hci_le_path_loss_threshold_evt *event = KE_MSG_ALLOC(HCI_LE_EVENT, conhdl, 0, hci_le_path_loss_threshold_evt);
    event->subcode  = HCI_LE_PATH_LOSS_THRESHOLD_EVT_SUBCODE;
    event->conhdl   = conhdl;
    event->curr_path_loss   = param->path_loss;
    event->zone_entered = param->zone;

    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the LLD power change indication message
 ****************************************************************************************
 */
int lld_con_pwr_change_ind_handler(ke_msg_id_t const msgid, struct lld_con_pwr_change_ind *param,
                                   ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t link_id = KE_IDX_GET(dest_id);

    struct llc_env_tag *llc_env_ptr = llc_env[link_id];

    // Send change indication to peer if indications enabled
    if (llc_env_ptr->pwr.loc_tx_pwr_chg_ind_en)
    {
        if (CO_RATE_UNDEF != param->tx_rate)
        {
            ASSERT_ERR(param->tx_rate < CO_RATE_UNDEF);

            int8_t old_tx_pwr = llc_env_ptr->pwr.loc_tx_pwr[param->tx_rate];
            int8_t new_tx_pwr = param->pwr;

            // Get delta in transmit power and min/max flags
            uint8_t flags = LLC_PWR_FLAGS_GET(new_tx_pwr);
            // Set delta if applicable
            int8_t delta = ((BLE_PWR_UNKNOWN != old_tx_pwr) && (BLE_PWR_CTRL_UNUSED != old_tx_pwr)) ? (new_tx_pwr - old_tx_pwr) : 0;

            // Notify of power level on newly enabled rate. If coded phy, set both bits in the mask.
            uint8_t phy_msk = co_rate_to_phypwr_mask[param->tx_rate];
            if (phy_msk & (PHY_PWR_S2_CODED_BIT | PHY_PWR_S8_CODED_BIT))
                phy_msk |= (PHY_PWR_S2_CODED_BIT | PHY_PWR_S8_CODED_BIT);
            llc_ll_pwr_change_ind_pdu_send(link_id, phy_msk, flags, delta, new_tx_pwr);

            llc_env_ptr->pwr.loc_tx_pwr[param->tx_rate] = param->pwr;
        }

        if (CO_RATE_UNDEF != param->d_rate)
        {
            ASSERT_ERR(param->d_rate < CO_RATE_UNDEF);

            // Notify of stopped managing power on the disabled rate. If coded phy, set both bits in the mask.
            uint8_t phy_msk = co_rate_to_phypwr_mask[param->d_rate];
            if (phy_msk & (PHY_PWR_S2_CODED_BIT | PHY_PWR_S8_CODED_BIT))
                phy_msk |= (PHY_PWR_S2_CODED_BIT | PHY_PWR_S8_CODED_BIT);

            // Phy mask should not be disabled if CIS use
            phy_msk &= llc_env_ptr->pwr.cis_phy_msk;

            if (phy_msk)
            {
                llc_ll_pwr_change_ind_pdu_send(link_id, phy_msk, 0, 0, BLE_PWR_CTRL_UNUSED);

                llc_env_ptr->pwr.loc_tx_pwr[param->d_rate] = BLE_PWR_CTRL_UNUSED;
            }
        }
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the LLD power change indication message
 ****************************************************************************************
 */
int lld_cis_pwr_change_ind_handler(ke_msg_id_t const msgid, struct lld_cis_pwr_change_ind *param,
                                   ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t link_id = KE_IDX_GET(dest_id);

    struct llc_env_tag *llc_env_ptr = llc_env[link_id];

    ASSERT_ERR(param->tx_rate < CO_RATE_UNDEF);

    // Send change indication to peer if indications enabled
    if (llc_env_ptr->pwr.loc_tx_pwr_chg_ind_en)
    {
        if (param->en)
        {
            // Start on max tranmsit power. Set max/min flags accordingly and delta to 0.
            int8_t tx_pwr = lld_tx_power_dbm_get(rwip_rf.txpwr_max, rwip_rf.txpwr_max_mod);
            uint8_t flags = BLE_PWR_CTRL_MAX_BIT | ((rwip_rf.txpwr_min == rwip_rf.txpwr_max) ? BLE_PWR_CTRL_MIN_BIT : 0);

            // Notify of power level on newly enabled rate. If coded phy, set both bits in the mask.
            uint8_t phy_msk = co_rate_to_phypwr_mask[param->tx_rate];
            if (phy_msk & (PHY_PWR_S2_CODED_BIT | PHY_PWR_S8_CODED_BIT))
                phy_msk |= (PHY_PWR_S2_CODED_BIT | PHY_PWR_S8_CODED_BIT);

            llc_ll_pwr_change_ind_pdu_send(link_id, phy_msk, flags, 0, tx_pwr);

            llc_env_ptr->pwr.cis_phy_msk = phy_msk;

            llc_env_ptr->pwr.loc_tx_pwr[param->tx_rate] = tx_pwr;
        }
        else
        {
            // Notify of stopped managing power on the disabled rate. If coded phy, set both bits in the mask.
            uint8_t phy_msk = co_rate_to_phypwr_mask[param->tx_rate];
            if (phy_msk & (PHY_PWR_S2_CODED_BIT | PHY_PWR_S8_CODED_BIT))
                phy_msk |= (PHY_PWR_S2_CODED_BIT | PHY_PWR_S8_CODED_BIT);

            llc_ll_pwr_change_ind_pdu_send(link_id, phy_msk, 0, 0, BLE_PWR_CTRL_UNUSED);

            llc_env_ptr->pwr.cis_phy_msk = 0;

            llc_env_ptr->pwr.loc_tx_pwr[param->tx_rate] = BLE_PWR_CTRL_UNUSED;
        }
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the power control procedure indication message.
 *
 * PlantUML procedure description
 *
 * @startuml llc_pwr_ctrl_op_start.png
 * title : Power control procedure start
 * participant LLC
 *  --> LLC : LLC_OP_PWR_CTRL_IND
 * LLC -> LLC: llc_op_pwr_ctrl_ind_handler()
 * activate LLC
 * hnote over LLC : LOC_PROC = Busy
 * LLC -> LLC: llc_loc_pwr_ctrl_proc_continue(START)
 * activate LLC
 * note right LLC #lightgreen: See __llc_pwr_ctrl_proc_continue()__
 * deactivate LLC
 * deactivate LLC
 * @enduml
 *
 ****************************************************************************************
 */
int llc_op_pwr_ctrl_ind_handler(ke_msg_id_t const msgid, struct llc_op_pwr_ctrl_ind *param,
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
        // execute power control procedure
        llc_pwr_ctrl_proc_continue(link_id, CO_ERROR_NO_ERROR);

    }

    return (msg_status);
}

#endif // BLE_PWR_CTRL

/*
 ****************************************************************************************
 * HCI Handlers
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handles the command HCI read transmit power.
 *
 * @param[in] link_id    Link Identifier
 * @param[in] param      Pointer to the parameters of the message.
 * @param[in] opcode     HCI message operation code.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int hci_rd_tx_pwr_lvl_cmd_handler(uint8_t link_id, struct hci_rd_tx_pwr_lvl_cmd const *param, uint16_t opcode)
{
    // allocate the status event message
    struct hci_rd_tx_pwr_lvl_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, param->conhdl, HCI_RD_TX_PWR_LVL_CMD_OPCODE, hci_rd_tx_pwr_lvl_cmd_cmp_evt);

#if (BT_PWR_CTRL || BLE_PWR_CTRL)
    // check if state is Free or in disconnected state
    if (llc_is_disconnecting(link_id))
    {
        event->status = CO_ERROR_COMMAND_DISALLOWED;
        event->tx_pow_lvl = 0;
    }
    else
    {
        event->status = CO_ERROR_NO_ERROR;
        switch (param->type)
        {
        case TX_PW_LVL_CURRENT:
        {
            // gets the current level
            event->tx_pow_lvl = lld_con_current_tx_power_get(link_id);
            // sets status
        }
        break;
        case TX_PW_LVL_MAX:
        {
            // gets the level max
            event->tx_pow_lvl = lld_tx_power_dbm_get(rwip_rf.txpwr_max, rwip_rf.txpwr_max_mod);
        }
        break;
        default:
        {
            // sets status
            event->status = CO_ERROR_INVALID_HCI_PARAM;
            event->tx_pow_lvl = 0;
        }
        break;
        }
    }
#else // !(BT_PWR_CTRL || BLE_PWR_CTRL)
    {
        event->status = CO_ERROR_UNSUPPORTED;
        event->tx_pow_lvl = 0;
    }
#endif // !(BT_PWR_CTRL || BLE_PWR_CTRL)

    // gets connection handle
    event->conhdl = param->conhdl;

    // send the message
    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the command HCI read RSSI.
 *
 * @param[in] link_id    Link Identifier
 * @param[in] param      Pointer to the parameters of the message.
 * @param[in] opcode     HCI message operation code.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int hci_rd_rssi_cmd_handler(uint8_t link_id, struct hci_basic_conhdl_cmd const *param, uint16_t opcode)
{
    // allocate the status event message
    struct hci_rd_rssi_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, param->conhdl, HCI_RD_RSSI_CMD_OPCODE, hci_rd_rssi_cmd_cmp_evt);

    // check if state is Free or in disconnected state
    if (llc_is_disconnecting(link_id))
    {
        event->status = CO_ERROR_COMMAND_DISALLOWED;
        // gets the rssi
        event->rssi = 0;
    }
    else
    {
        event->status = CO_ERROR_NO_ERROR;
        // gets the rssi
        event->rssi = lld_con_rssi_get(link_id);
    }

    // gets connection handle
    event->conhdl = param->conhdl;

    // send the message
    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}

#if BLE_PWR_CTRL

/**
 ****************************************************************************************
 * @brief Handles the command HCI enhanced read transmit power level command.
 *
 * @param[in] link_id    Link Identifier
 * @param[in] param      Pointer to the parameters of the message.
 * @param[in] opcode     HCI message operation code.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int hci_le_enh_rd_tx_pwr_lvl_cmd_handler(uint8_t link_id, struct hci_le_enh_rd_tx_pwr_lvl_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_UNKNOWN_CONNECTION_ID;
    // allocate the status event message
    struct hci_le_enh_rd_tx_pwr_lvl_cmd_cmp_evt *event;

    // allocate the Command Complete event message
    event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, param->conhdl, opcode, hci_le_enh_rd_tx_pwr_lvl_cmd_cmp_evt);

    // check if state is Free or in disconnected state
    if (!llc_is_disconnecting(link_id))
    {
        if (param->phy == 0)
        {
            status = CO_ERROR_INVALID_HCI_PARAM;
        }
        else if (param->phy > PHY_PWR_S2_CODED_VALUE)
        {
            status = CO_ERROR_UNSUPPORTED;
        }
#if !(BLE_PHY_2MBPS_SUPPORT)
        else if (PHY_PWR_2MBPS_VALUE == param->phy)
        {
            status = CO_ERROR_UNSUPPORTED;
        }
#endif //(BLE_PHY_2MBPS_SUPPORT)
#if !(BLE_PHY_CODED_SUPPORT)
        else if ((PHY_PWR_S8_CODED_VALUE == param->phy) || (PHY_PWR_S2_CODED_VALUE == param->phy))
        {
            status = CO_ERROR_UNSUPPORTED;
        }
#endif //(BLE_PHY_CODED_SUPPORT)
        else
        {
            // Fill the PHY information
            event->curr_tx_pwr_lvl = lld_con_tx_power_get(link_id, param->phy);
            event->max_tx_pwr_lvl =  lld_tx_power_dbm_get(rwip_rf.txpwr_max, rwip_rf.txpwr_max_mod);
            status = CO_ERROR_NO_ERROR;
        }
    } // else nothing to do

    event->phy = param->phy;
    event->conhdl = param->conhdl;
    event->status = status;

    if (status != CO_ERROR_NO_ERROR)
    {
        event->curr_tx_pwr_lvl = 0;
        event->max_tx_pwr_lvl = 0;
    }

    // sends the message
    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the command HCI read remote transmit power level command.
 *
 * @startuml llc_pwr_ctrl_req_init_host.png
 * title : Power control request initiated by host
 * participant HCI
 * participant LLC
 * participant LLD
 *
 * HCI --> LLC : HCI_LE_RD_REMOTE_TX_PWR_LVL_CMD
 * LLC -> LLC: hci_le_rd_remote_tx_pwr_lvl_cmd_handler()
 * activate LLC
 * alt link disconnected \nor slave \nor peer not support encryption
 *     LLC --> HCI : HCI_CMD_STAT_EVENT(DISALLOWED)
 * else  Peer does not support feature
 *     LLC --> HCI : HCI_CMD_STAT_EVENT(UNSUPPORTED_REMOTE_FEATURE)
 * else  Invalid parameters
 *     LLC --> HCI : HCI_CMD_STAT_EVENT(INVALID_HCI_PARAM)
 * else  Local device does not support configured parameters
 *     LLC --> HCI : HCI_CMD_STAT_EVENT(UNSUPPORTED)
 * else  Procedure already started
 *     LLC --> HCI : HCI_CMD_STAT_EVENT(BUSY)
 * else  Procedure can be started
 *     note over LLC: Mark procedure started
 *     LLC --> LLC : LLC_OP_RD_REMOTE_TX_PWR_LVL_RSP
 *     note right LLC #lightgreen: See __llc_op_rd_rem_tx_pwer_lvl_ind_handler()__
 *     LLC --> HCI : HCI_CMD_STAT_EVENT(OK)
 * end
 * deactivate LLC
 * @enduml
 *
 * @param[in] link_id    Link Identifier
 * @param[in] param      Pointer to the parameters of the message.
 * @param[in] opcode     HCI message operation code.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int hci_le_rd_remote_tx_pwr_lvl_cmd_handler(uint8_t link_id, struct hci_le_rd_remote_tx_pwr_lvl_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_UNKNOWN_CONNECTION_ID;
    bool send_event = false;

    int8_t tx_pwr = 0;
    uint8_t flags = 0;

    // Current message status
    int msg_status = KE_MSG_CONSUMED;

    // check if state is Free or in disconnected state
    if (!llc_is_disconnecting(link_id))
    {
        // check if peer doesn't support power control request
        if (!llc_le_feature_check(link_id, BLE_FEAT_POWER_CONTROL_REQ))
        {
            status = CO_ERROR_UNSUPPORTED_REMOTE_FEATURE;
        }
        else
        {
            if (param->phy == 0)
            {
                status = CO_ERROR_INVALID_HCI_PARAM;
            }
            else if (param->phy > PHY_PWR_S2_CODED_VALUE)
            {
                status = CO_ERROR_UNSUPPORTED;
            }
#if !(BLE_PHY_2MBPS_SUPPORT)
            else if (PHY_PWR_2MBPS_VALUE == param->phy)
            {
                status = CO_ERROR_UNSUPPORTED;
            }
#endif //(BLE_PHY_2MBPS_SUPPORT)
#if !(BLE_PHY_CODED_SUPPORT)
            else if ((PHY_PWR_S8_CODED_VALUE == param->phy) || (PHY_PWR_S2_CODED_VALUE == param->phy))
            {
                status = CO_ERROR_UNSUPPORTED;
            }
#endif //(BLE_PHY_CODED_SUPPORT)
            else
            {
                //Get environment pointer
                struct llc_env_tag *llc_env_ptr = llc_env[link_id];

                tx_pwr = lld_con_remote_tx_pwr_get(link_id, co_phypwr_to_rate[param->phy], &flags);

                // If tx power and flags known, and peer obliged to send power change indications, then the locally stored value can be reported.
                if ((BLE_PWR_UNKNOWN != tx_pwr) && (llc_env_ptr->pwr.rem_tx_pwr_chg_ind_en) && (flags != LLD_PWR_FLGS_UNKNOWN))
                {
                    send_event = true;
                    status = CO_ERROR_NO_ERROR;
                }
                else
                {
                    // ensure not currently processing request
                    if (GETB(llc_env_ptr->flow_ctrl, LLC_HCI_PWR_CTRL_REQ))
                    {
                        // the procedure is active
                        if (LLC_PROC_PWR_CTRL == llc_proc_id_get(link_id, LLC_PROC_LOCAL))
                        {
                            /// retrieve procedure parameters
                            struct llc_op_pwr_ctrl_ind *pwr_ctrl = (struct llc_op_pwr_ctrl_ind *) llc_proc_get(link_id, LLC_PROC_LOCAL);

                            // The active automatic procedure can return the requested information
                            if ((pwr_ctrl->phy == param->phy) && !pwr_ctrl->evt_report)
                            {
                                // Report the power level when the procedure completes
                                pwr_ctrl->evt_report = true;
                                pwr_ctrl->reason = BLE_PWR_HCI_REQ;

                                status = CO_ERROR_NO_ERROR;
                            }
                            else
                            {
                                // process this message later
                                msg_status = KE_MSG_SAVED;
                            }
                        }
                        else
                        {
                            // process this message later
                            msg_status = KE_MSG_SAVED;
                        }
                    }
                    else
                    {
                        ke_task_id_t llc_id = KE_BUILD_ID(TASK_LLC, link_id);
                        // start power control procedure
                        struct llc_op_pwr_ctrl_ind *pwr_ctrl = KE_MSG_ALLOC(LLC_OP_PWR_CTRL_IND, llc_id, llc_id, llc_op_pwr_ctrl_ind);
                        llc_proc_init(&pwr_ctrl->proc, LLC_PROC_PWR_CTRL, llc_pwr_ctrl_proc_err_cb);
                        pwr_ctrl->phy = param->phy;
                        pwr_ctrl->delta = 0; // no change requested

                        pwr_ctrl->evt_report = true;
                        pwr_ctrl->reason = BLE_PWR_HCI_REQ;
                        llc_proc_state_set(&pwr_ctrl->proc, link_id, LLC_PWR_CTRL_PROC_START);
                        ke_msg_send(pwr_ctrl);

                        SETB(llc_env_ptr->flow_ctrl, LLC_HCI_PWR_CTRL_REQ, true);

                        status = CO_ERROR_NO_ERROR;
                    }
                }
            }
        }
    } // else nothing to do

    if (msg_status != KE_MSG_SAVED)
    {
        // Send the command status event
        llc_cmd_stat_send(link_id, opcode, status);

        // Send the prepared HCI LE Transmit Power Reporting event
        if (send_event)
        {
            llc_hci_le_tx_power_reporting_evt_send(link_id, status, BLE_PWR_HCI_REQ, param->phy, tx_pwr, flags, 0);
        }
    }

    return (msg_status);
}

/**
 ****************************************************************************************
 * @brief Handles the command HCI set path loss reporting parameters command.
 *
 * @param[in] link_id    Link Identifier
 * @param[in] param      Pointer to the parameters of the message.
 * @param[in] opcode     HCI message operation code.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int hci_le_set_path_loss_rep_param_cmd_handler(uint8_t link_id, struct hci_le_set_path_loss_rep_param_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_UNKNOWN_CONNECTION_ID;
    // allocate the status event message
    struct hci_le_set_path_loss_rep_param_cmd_cmp_evt *event;

    // allocate the Command Complete event message
    event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, param->conhdl, opcode, hci_le_set_path_loss_rep_param_cmd_cmp_evt);

    // check if state is Free or in disconnected state
    if (!llc_is_disconnecting(link_id))
    {
        if (((param->hi_thr + param->hi_hyst) > PHY_PATH_LOSS_MAX) || (param->lo_thr < param->lo_hyst)
                || (param->lo_thr > param->hi_thr) || ((param->lo_thr + param->lo_hyst) > (param->hi_thr - param->hi_hyst))
           )
        {
            status = CO_ERROR_INVALID_HCI_PARAM;
        }
        else
        {
            //Get environment pointer
            struct llc_env_tag *llc_env_ptr = llc_env[link_id];

            // Configure path loss monitor
            lld_con_path_loss_monitor_config(link_id, param->hi_thr, param->hi_hyst, param->lo_thr, param->lo_hyst, param->min_time);
            llc_env_ptr->pwr.path_loss_cfg = true;

            status = CO_ERROR_NO_ERROR;
        }

    } // else nothing to do

    event->conhdl = param->conhdl;
    event->status = status;

    // sends the message
    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the command HCI set path loss reporting enable command.
 *
 * @param[in] link_id    Link Identifier
 * @param[in] param      Pointer to the parameters of the message.
 * @param[in] opcode     HCI message operation code.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int hci_le_set_path_loss_rep_en_cmd_handler(uint8_t link_id, struct hci_le_set_path_loss_rep_en_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_UNKNOWN_CONNECTION_ID;
    // allocate the status event message
    struct hci_le_set_path_loss_rep_en_cmd_cmp_evt *event;

    // allocate the Command Complete event message
    event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, param->conhdl, opcode, hci_le_set_path_loss_rep_en_cmd_cmp_evt);

    // check if state is Free or in disconnected state
    if (!llc_is_disconnecting(link_id))
    {
        //Get environment pointer
        struct llc_env_tag *llc_env_ptr = llc_env[link_id];

        if (!llc_env_ptr->pwr.path_loss_cfg)
        {
            // Command disallowed if host issues command before parameters configured
            status = CO_ERROR_COMMAND_DISALLOWED;
        }
        else
        {
            // Enable path loss monitoring
            lld_con_path_loss_monitor_en(link_id, param->en);

            // If path loss monitoring enabled, and power control request procedure not already activated
            if (param->en && !GETB(llc_env_ptr->flow_ctrl, LLC_HCI_PWR_CTRL_REQ))
            {
                // Controller has to query the remote Controller for its transmit power level
                ke_task_id_t llc_id = KE_BUILD_ID(TASK_LLC, link_id);
                // start power control procedure
                struct llc_op_pwr_ctrl_ind *pwr_ctrl = KE_MSG_ALLOC(LLC_OP_PWR_CTRL_IND, llc_id, llc_id, llc_op_pwr_ctrl_ind);
                llc_proc_init(&pwr_ctrl->proc, LLC_PROC_PWR_CTRL, llc_pwr_ctrl_proc_err_cb);
                pwr_ctrl->phy = co_rate_to_phypwr[lld_con_rx_rate_get(link_id)];
                pwr_ctrl->delta = 0; // no change requested

                pwr_ctrl->evt_report = false;
                pwr_ctrl->reason = BLE_PWR_REM_TX_CHG;
                llc_proc_state_set(&pwr_ctrl->proc, link_id, LLC_PWR_CTRL_PROC_START);
                ke_msg_send(pwr_ctrl);

                SETB(llc_env_ptr->flow_ctrl, LLC_HCI_PWR_CTRL_REQ, true);
            }

            status = CO_ERROR_NO_ERROR;
        }

    } // else nothing to do

    event->conhdl = param->conhdl;
    event->status = status;

    // sends the message
    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the command HCI set transmit power reporting enable command.
 *
 * @param[in] link_id    Link Identifier
 * @param[in] param      Pointer to the parameters of the message.
 * @param[in] opcode     HCI message operation code.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int hci_le_set_tx_power_rep_en_cmd_handler(uint8_t link_id, struct hci_le_set_tx_power_rep_en_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_UNKNOWN_CONNECTION_ID;

    // allocate the status event message
    struct hci_le_set_tx_power_rep_en_cmd_cmp_evt *event;

    // allocate the Command Complete event message
    event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, param->conhdl, opcode, hci_le_set_tx_power_rep_en_cmd_cmp_evt);

    // check if state is Free or in disconnected state
    if (!llc_is_disconnecting(link_id))
    {
        // check if peer doesn't support power control request
        if (!llc_le_feature_check(link_id, BLE_FEAT_POWER_CONTROL_REQ))
        {
            status = CO_ERROR_UNSUPPORTED_REMOTE_FEATURE;
        }
        else if ((param->local_en > BLE_TX_PWR_REP_EN) || (param->remote_en > BLE_TX_PWR_REP_EN))
        {
            status = CO_ERROR_INVALID_HCI_PARAM;
        }
        else
        {
            //Get environment pointer
            struct llc_env_tag *llc_env_ptr = llc_env[link_id];

            llc_env_ptr->pwr.loc_tx_pwr_rep_en =  param->local_en;
            llc_env_ptr->pwr.rem_tx_pwr_rep_en =  param->remote_en;

            /*
             * Start a power control request procedure if enabling either local/remote transmit power reporting
             *  a) to attain an APR value for local power adjustment, and/or
             *  b) to enable remote transmit power reporting
             */
            if ((param->local_en || param->remote_en) && !GETB(llc_env_ptr->flow_ctrl, LLC_HCI_PWR_CTRL_REQ))
            {
                ke_task_id_t llc_id = KE_BUILD_ID(TASK_LLC, link_id);
                // start power control procedure

                struct llc_op_pwr_ctrl_ind *pwr_ctrl = KE_MSG_ALLOC(LLC_OP_PWR_CTRL_IND, llc_id, llc_id, llc_op_pwr_ctrl_ind);
                llc_proc_init(&pwr_ctrl->proc, LLC_PROC_PWR_CTRL, llc_pwr_ctrl_proc_err_cb);
                pwr_ctrl->phy = co_rate_to_phypwr[lld_con_rx_rate_get(link_id)];
                pwr_ctrl->delta = 0; // no change requested

                pwr_ctrl->evt_report = false;
                pwr_ctrl->reason = (param->local_en) ? BLE_PWR_LOC_TX_CHG : BLE_PWR_REM_TX_CHG;
                llc_proc_state_set(&pwr_ctrl->proc, link_id, LLC_PWR_CTRL_PROC_START);
                ke_msg_send(pwr_ctrl);

                SETB(llc_env_ptr->flow_ctrl, LLC_HCI_PWR_CTRL_REQ, true);

                status = CO_ERROR_NO_ERROR;
            }

            status = CO_ERROR_NO_ERROR;
        }
    } // else nothing to do

    event->conhdl = param->conhdl;
    event->status = status;

    // sends the message
    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}


/*
 ****************************************************************************************
 * Local Messages Handlers
 ****************************************************************************************
 */

#endif // BLE_PWR_CTRL

#endif // (BLE_CENTRAL || BLE_PERIPHERAL)

/// @} LLC_PWR
