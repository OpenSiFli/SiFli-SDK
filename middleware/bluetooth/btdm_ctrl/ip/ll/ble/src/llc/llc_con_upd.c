/**
 ****************************************************************************************
 *
 * @file llc_con_upd.c
 *
 * @brief Connection parameter Update Procedure
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup LLC_CON_UPD
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
#include "co_math.h"     // For co_min

#include "ke_msg.h"      // Kernel message
#include "ke_timer.h"    // Kernel timers

#include "sch_plan.h"    // Scheduling Planner

#include "llc_int.h"     // Internal LLC API
#include "llc_llcp.h"    // Internal LLCP API

#include "llm.h"         // LLM API

#include "lld.h"         // LLD API

#include "hci.h"         // For HCI handler

#include "ble_util.h"    // BLE utility functions

#if (BLE_ISO_MODE_0)
    #include "lli.h"         // LLI API
#endif // (BLE_ISO_MODE_0)
#include "btdm_patch.h"

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */


/// Connection update operation indication structure definition
/*@TRACE*/
struct llc_op_con_upd_ind
{
    /// procedure information
    llc_procedure_t proc;

    // ****  Negotiated parameters ****
    /// minimum value of connInterval (units of 1.25 ms)
    uint16_t       con_intv_min;
    /// maximum value of connInterval (units of 1.25 ms)
    uint16_t       con_intv_max;
    /// Connection latency (unit of connection event)
    uint16_t       con_latency;
    /// Link supervision timeout (unit of 10 ms)
    uint16_t       superv_to;
    /// Minimum of CE length (units of 0.625 ms)
    uint16_t       ce_len_min;
    /// Maximum of CE length  (units of 0.625 ms)
    uint16_t       ce_len_max;
    /// Reference ConEventCount (unit of connection event)
    uint16_t       ref_con_event_count;
    /// Offset0 (units of 1.25 ms)
    uint16_t       offset0;
    /// Offset1 (units of 1.25 ms)
    uint16_t       offset1;
    /// Offset2 (units of 1.25 ms)
    uint16_t       offset2;
    /// Offset3 (units of 1.25 ms)
    uint16_t       offset3;
    /// Offset4 (units of 1.25 ms)
    uint16_t       offset4;
    /// Offset5 (units of 1.25 ms)
    uint16_t       offset5;
    /// window offset (units of 1.25 ms)
    uint16_t       win_off;
    /// instant (unit of connection event)
    uint16_t       instant;
    /// window size (units of 1.25 ms)
    uint8_t        win_size;
    /// preferred periodicity (units of 1.25 ms)
    uint8_t        pref_period;



    // **** computed preferred values ****
    /// Computed connection Interval (units of 1.25 ms)
    uint16_t       comp_interval;
    /// Computed offset in 1.25 ms
    uint16_t       comp_offset;

//    /// Status for the reject pdu sent
//    uint8_t        reject_reason;
    /// Forced parameter update (without negotiation)
    bool           forced;
    /// Use to know operation requested by host
    bool           req_by_host;
    /// Use to know if host part of negotiation
    bool           host_neg;
};






/*
 * DEFINES
 ****************************************************************************************
 */
// Default window size is set to 1.25ms
#define CON_UPD_WIN_SZ_DFT  (1)
/*@TRACE*/
enum llc_con_update_state
{
    //Local states
    /// Local start connection parameter request procedure
    LLC_LOC_CON_UPD_START,
    /// Force sending the LLCP_CONNECTION_UPDATE_IND (Master only)
    /// because procedure initiated by slave but instant must be set by Master
    LLC_LOC_CON_UPD_FORCED,
    /// Wait for connection parameters response (Master only)
    LLC_LOC_WAIT_CON_PARAM_RSP,
    /// Wait for connection update indication (Slave only)
    LLC_LOC_WAIT_CON_UPD_IND,
    /// Wait connection update instant
    LLC_LOC_WAIT_CON_UPD_INSTANT,
    /// Wait Reject indication acknowledgment
    LLC_LOC_CON_UPD_WAIT_REJECT_ACK,
    /// Error occurs during connection parameter update
    LLC_LOC_CON_UPD_ERROR,

    //Remote states
    /// Remote start connection parameter request procedure
    LLC_REM_CON_UPD_START,
    /// Wait host reply
    LLC_REM_WAIT_HOST_RPLY,
    /// Wait for connection update indication (Slave only)
    LLC_REM_WAIT_CON_UPD_IND,
    /// Wait connection update instant (Slave only)
    LLC_REM_WAIT_CON_UPD_INSTANT,
    /// Error occurs during connection parameter update
    LLC_REM_CON_UPD_ERROR,
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
 * LOCAL FUNCTIONS DECLARATION
 ****************************************************************************************
 */
__STATIC void llc_loc_con_upd_proc_continue(uint8_t link_id, uint8_t state, uint8_t status);
__STATIC void llc_rem_con_upd_proc_continue(uint8_t link_id, uint8_t state, uint8_t status);

__STATIC bool llc_con_upd_param_in_range(uint8_t link_id, uint16_t interval_max, uint16_t interval_min, uint16_t latency, uint16_t timeout);
__STATIC void llc_pref_param_compute(uint8_t link_id, bool local_proc, struct llc_op_con_upd_ind *param);
__STATIC void llc_ll_connection_update_ind_pdu_send(uint8_t link_id, struct llc_op_con_upd_ind *param);
__STATIC void llc_ll_connection_param_req_pdu_send(uint8_t link_id, struct llc_op_con_upd_ind *param);
__STATIC void llc_ll_connection_param_rsp_pdu_send(uint8_t link_id, struct llc_op_con_upd_ind *param);
__STATIC void llc_hci_con_param_req_evt_send(uint8_t link_id, uint16_t con_intv_min, uint16_t con_intv_max, uint16_t con_latency, uint16_t superv_to);
__STATIC void llc_hci_con_upd_info_send(uint8_t link_id, uint8_t status, struct llc_op_con_upd_ind *param);
__STATIC void llc_loc_con_upd_proc_err_cb(uint8_t link_id, uint8_t error_type, void *param);
__STATIC void llc_rem_con_upd_proc_err_cb(uint8_t link_id, uint8_t error_type, void *param);

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * Continue execution of local procedure
 *
 *
 * @startuml llc_loc_con_upd_proc_continue_peer_init.png
 * participant HCI
 * participant LLC
 * participant LLD
 * == Peer initiated Connection Update Negotiation (Master Role) ==
 * LLC --> LLC : LLC_OP_CON_UPD_IND
 * LLC -> LLC: llc_op_con_upd_ind_handler()
 * activate LLC
 *     hnote over LLC : LOC_PROC = Busy
 *     LLC -> LLC: llc_loc_con_upd_proc_continue(CON_UPD_FORCED)
 *     activate LLC
 *         note over LLC #aqua: Compute instant + Parameters
 *         note over LLC: state = WAIT_CON_UPD_INSTANT
 *         LLC -> LLD: lld_con_param_update()
 *         LLC-->LLD: LLCP_CON_UPD_IND
 *     deactivate LLC
 * deactivate LLC
 * == Wait For Connection Update Instant ==
 * LLD --> LLC : LLD_CON_UPD_CFM
 * LLC -> LLC: lld_con_upd_cfm_handler()
 * activate LLC
 *     LLC -> LLC: llc_loc_con_upd_proc_continue(CON_UPD_INSTANT)
 *     activate LLC
 *         alt Connection parameter Updated or host involved in negotiation
 *             note over LLC #aqua: Store new connection parameters
 *             hnote over LLC: LOC_PROC=Idle
 *             LLC-->HCI: HCI_LE_CON_UPD_CMP_EVT
 *         end
 *     deactivate LLC
 * deactivate LLC
 * @enduml
 *
 * @startuml llc_loc_con_upd_proc_continue_local_init.png
 * participant HCI
 * participant LLC
 * participant LLD
 * == Local initiated Connection parameter Negotiation ==
 * LLC --> LLC : LLC_OP_CON_UPD_IND
 * LLC -> LLC: llc_op_con_upd_ind_handler()
 * activate LLC
 *     hnote over LLC : LOC_PROC = Busy
 *     LLC -> LLC: llc_loc_con_upd_proc_continue(START)
 *     activate LLC
 *         note over LLC #aqua: store preferred host parameters
 *         alt Slave Role
 *             note over LLC: state = WAIT_CON_UPD_IND
 *             LLC-->LLD: LLCP_CON_PARAM_REQ
 *         else Master Role
 *             note over LLC: state = WAIT_CON_PARAM_RSP
 *             LLC-->LLD: LLCP_CON_PARAM_REQ
 *         end
 *     deactivate LLC
 * deactivate LLC
 * == Peer does not support feature ==
 *     LLD --> LLC : LLCP_UNKNOWN_RSP
 *     LLC -> LLC: llc_loc_con_upd_proc_err_cb(LLCP_UNKNOWN_RSP)
 *     activate LLC
 *         note over LLC: Mark feature not supported by peer device.
 *         LLC -> LLC: llc_loc_con_upd_proc_continue(PARAM_RSP, UNSUPPORTED)
 *         activate LLC
 *             note over LLC #aqua: Compute instant + Parameters
 *             note over LLC: state = WAIT_CON_UPD_INSTANT
 *             LLC -> LLD: lld_con_param_update()
 *             LLC-->LLD: LLCP_CON_UPD_IND
 *         deactivate LLC
 *     deactivate LLC
 * == Wait For Connection parameter Response (Master Only) ==
 *     LLD --> LLC : LLCP_CON_PARAM_RSP
 *     LLC -> LLC: ll_connection_param_rsp_handler()
 *     activate LLC
 *         note over LLC: Store peer parameters
 *         LLC -> LLC: llc_loc_con_upd_proc_continue(CON_PARAM_RSP)
 *         activate LLC
 *             note over LLC #aqua: Compute instant + Parameters
 *             note over LLC: state = WAIT_CON_UPD_INSTANT
 *             LLC -> LLD: lld_con_param_update()
 *             LLC-->LLD: LLCP_CON_UPD_IND
 *         deactivate LLC
 *     deactivate LLC
 * == Wait For Connection Update Indication (SlaveOnly) ==
 *     LLD --> LLC : LLCP_CON_UPD_IND
 *     LLC -> LLC: llcp_con_upd_ind_handler()
 *     activate LLC
 *         note over LLC: Store peer configured params
 *         LLC -> LLC: llc_loc_con_upd_proc_continue(CON_UPD_IND)
 *         activate LLC
 *             note over LLC: state = WAIT_CON_UPD_INSTANT
 *             LLC -> LLD: lld_con_param_update()
 *         deactivate LLC
 *     deactivate LLC
 * == Wait For Connection param Update Instant ==
 * LLD --> LLC : LLD_CON_UPD_CFM
 * LLC -> LLC: lld_con_upd_cfm_handler()
 * activate LLC
 *     alt Connection parameter Updated or host initiated negotiation
 *         note over LLC #aqua: Store new connection parameters
 *         hnote over LLC: LOC_PROC=Idle
 *         LLC-->HCI: HCI_LE_CON_UPD_CMP_EVT
 *     end
 * deactivate LLC
 * @enduml
 *
 * @param[in] link_id Link identifier
 * @param[in] state   Expected state of the procedure
 * @param[in] status  Status of the operation
 ****************************************************************************************
 */
__STATIC void llc_loc_con_upd_proc_continue(uint8_t link_id, uint8_t state, uint8_t status)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(PROC_CONTINUE_PATCH_TYPE, LLC_LOC_CON_UPD_PROC_CONTINUE_C_FUN_BIT, link_id, state, status);
    bool finished = false;
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];

    // Gets the values requested by the peer
    struct llc_op_con_upd_ind *param = (struct llc_op_con_upd_ind *)llc_proc_get(link_id, LLC_PROC_LOCAL);

    // an unexpected error occurs, close the procedure
    if (state == LLC_LOC_CON_UPD_ERROR)
    {
        llc_proc_state_set(&param->proc, link_id, LLC_LOC_CON_UPD_ERROR);

        // Restore the TX length as per the current interval
        lld_con_tx_len_update_for_intv(link_id, llc_env_ptr->con_params.interval);
    }

    // check that current procedure state equals to expected state given in parameter
    if ((llc_proc_state_get(&param->proc) != state))
    {
        ASSERT_WARN(0, llc_proc_state_get(&param->proc), state);
        // unexpected state, ignore it
    }
    else
    {
        switch (llc_proc_state_get(&param->proc))
        {
        case LLC_LOC_WAIT_CON_PARAM_RSP:
        {
            // Stop the LLCP Response TO
            llc_proc_timer_set(link_id, LLC_PROC_LOCAL, false);

            // if rejected only because peer device does not support peer feature, force the update
            if ((status != CO_ERROR_NO_ERROR) && llc_le_feature_check(link_id, BLE_FEAT_CON_PARAM_REQ_PROC))
            {
                if (!GETB(llc_env_ptr->link_info, LLC_INFO_MASTER_ROLE))
                {
                    struct sch_plan_elt_tag *plan_elt = llm_plan_elt_get(link_id);

                    ASSERT_ERR(plan_elt != NULL);
                    ASSERT_ERR(plan_elt->interval != 0);

                    // Mark the interval element as not movable
                    plan_elt->cb_move = NULL;
                    plan_elt->mobility = SCH_PLAN_MB_LVL_0;
                }

                finished = true;
                break;
            }
        }
        // no break
        case LLC_LOC_CON_UPD_FORCED:
        {
            ASSERT_INFO(GETB(llc_env_ptr->link_info, LLC_INFO_MASTER_ROLE) == 1, link_id, status);
            param->forced = true;
        }
        // no break
        case LLC_LOC_CON_UPD_START:
        {
            // procedure with instant started
            SETB(llc_env_ptr->link_info, LLC_INFO_INSTANT_PROC, true);

            // connection param request can be issued
            if (!param->forced)
            {
                // fill preferred offset
                param->ref_con_event_count    = lld_con_event_counter_get(link_id);

                // compute preferred parameters
                llc_pref_param_compute(link_id, true, param);

                // If auto-initiated connection move and offset does not change
                if (!param->req_by_host && (param->comp_offset == 0))
                {
                    finished = true;
                    break;
                }

                param->offset0                = param->comp_offset;
                param->offset1                = PARAM_REQ_INVALID_OFFSET;
                param->offset2                = PARAM_REQ_INVALID_OFFSET;
                param->offset3                = PARAM_REQ_INVALID_OFFSET;
                param->offset4                = PARAM_REQ_INVALID_OFFSET;
                param->offset5                = PARAM_REQ_INVALID_OFFSET;

                llc_ll_connection_param_req_pdu_send(link_id, param);
                // Start the LLCP Response TO
                llc_proc_timer_set(link_id, LLC_PROC_LOCAL, true);

                // if master, wait for LLCP_CON_PARAM_RSP
                if (GETB(llc_env_ptr->link_info, LLC_INFO_MASTER_ROLE))
                {
                    llc_proc_state_set(&param->proc, link_id, LLC_LOC_WAIT_CON_PARAM_RSP);
                }
                // if slave, wait for LLCP_CON_UPDATE_IND
                else
                {
                    // Update the TX length as per the minimum proposed interval
                    lld_con_tx_len_update_for_intv(link_id, param->con_intv_min);

                    llc_proc_state_set(&param->proc, link_id, LLC_LOC_WAIT_CON_UPD_IND);
                }
            }
            // parameter update indication must start
            else
            {
                // Compute procedure instant
                param->instant = llc_inst_select(link_id);
                param->win_size = CON_UPD_WIN_SZ_DFT;

                param->ref_con_event_count = param->instant;
                // compute preferred parameters
                llc_pref_param_compute(link_id, true, param);

                // If auto-initiated connection move and offset does not change
                if ((state == LLC_LOC_CON_UPD_START) && !param->req_by_host && (param->comp_offset == 0))
                {
                    finished = true;
                    break;
                }

                param->win_off      = param->comp_offset;

                // Set the connection parameters in the LLD environment
                status = lld_con_param_update(link_id, param->win_size, param->win_off,
                                              param->comp_interval, param->con_latency,
                                              param->superv_to, param->instant);

                llc_proc_state_set(&param->proc, link_id, LLC_LOC_WAIT_CON_UPD_INSTANT);

                if (status != CO_ERROR_NO_ERROR)
                {
                    finished = true;
                }
                else // at least one value has been changed
                {
                    // Send connection update to the peer
                    llc_ll_connection_update_ind_pdu_send(link_id, param);
                }
            }
        }
        break;

        // Slave Only
        case LLC_LOC_WAIT_CON_UPD_IND:
        {
            // Stop the LLCP Response TO
            llc_proc_timer_set(link_id, LLC_PROC_LOCAL, false);

            // check that Master does not reject parameter update request
            if (status == CO_ERROR_NO_ERROR)
            {
                // Set the connection parameters in the LLD environment
                status = lld_con_param_update(link_id, param->win_size, param->win_off, param->comp_interval, param->con_latency,
                                              param->superv_to, param->instant);

                if (status != CO_ERROR_NO_ERROR)
                {
                    finished = true;
                }
                else
                {
                    llc_proc_state_set(&param->proc, link_id, LLC_LOC_WAIT_CON_UPD_INSTANT);
                }
            }
            else // LLCP reject received
            {
                finished = true;
            }
        }
        break;
        case LLC_LOC_WAIT_CON_UPD_INSTANT:
        {

            finished = true;
        }
        break;


        case LLC_LOC_CON_UPD_ERROR:
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
#if (BT_53)
        // check if connection interval changed
        bool intv_changed = llc_env_ptr->con_params.interval != param->comp_interval;
#endif // (BT_53)

        // try to send complete event
        llc_hci_con_upd_info_send(link_id, status, param);

        // procedure with instant finished
        SETB(llc_env_ptr->link_info, LLC_INFO_INSTANT_PROC, false);

        if (!param->req_by_host)
        {
            // mark local procedure not under process
            SETB(llc_env_ptr->flow_ctrl, LLC_LOC_CON_UPDATE_REQ, false);
        }

        // unregister procedure
        llc_proc_unreg(link_id, LLC_PROC_LOCAL);

#if (BT_53)
        // Shall not restore subrating if connection interval changed, otherwise partial restore
        llc_subrate_restore(link_id, intv_changed ? LLC_SUBRATE_NO_RESTORE : LLC_SUBRATE_PARTIAL_RESTORE);
#endif // (BT_53)
    }
}

/**
 ****************************************************************************************
 * Continue execution of remote procedure
 *
 *
 * @startuml llc_rem_con_upd_proc_continue.png
 * participant HCI
 * participant LLC
 * participant LLD
 * == Reception of Connection Update Request from peer device ==
 * LLD --> LLC : LLCP_CON_UPD_REQ
 * LLC -> LLC: llcp_con_upd_req_handler()
 * activate LLC
 *     note over LLC #aqua: Allocate Remote procedure structure\nstore peer parameter\nregister procedure
 *     hnote over LLC: REM_PROC=busy
 *     LLC -> LLC: llc_rem_con_upd_proc_continue(START)
 *     activate LLC
 *         alt Master Role - no need to involve host
 *             note over LLC #lightgreen: Create a new Local procedure to manage instant\n see __llc_loc_con_upd_proc_continue__
 *             LLC --> LLC: LLC_OP_CON_UPD_IND
 *             hnote over LLC: REM_PROC=Idle
 *         else Slave Role - no need to involve host
 *             note over LLC #aqua: Compute a new connection offset\n and propose it
 *             note over LLC: state = WAIT_CON_UPD_IND
 *             LLC --> LLD: LLCP_CON_UPD_RSP
 *         else Host must be involved in parameter update
 *             note over LLC: state = WAIT_HOST_RPLY
 *             LLC --> HCI: HCI_LE_REM_CON_PARAM_REQ_EVT
 *         end
 *     deactivate LLC
 * deactivate LLC
 * == Wait For Host Response - Accepted ==
 * alt Slave Role
 *     HCI --> LLC : HCI_LE_REM_CON_PARAM_REQ_REPLY_CMD
 *     LLC -> LLC: hci_le_rem_con_param_req_reply_cmd_handler()
 *     activate LLC
 *         LLC --> HCI: HCI_CMD_STATUS_EVT
 *         LLC -> LLC: llc_rem_con_upd_proc_continue(HOST_RPLY)
 *         activate LLC
 *             alt Master Role
 *                 note over LLC #lightgreen: Create a new Local procedure to manage instant\n see __llc_loc_con_upd_proc_continue__
 *                 LLC --> LLC: LLC_OP_CON_UPD_IND
 *                 hnote over LLC: REM_PROC=Idle
 *             else Slave Role
 *                 note over LLC #aqua: Compute a new connection offset\n and propose it
 *                 note over LLC: state = WAIT_CON_UPD_IND
 *                 LLC --> LLD: LLCP_CON_UPD_RSP
 *             end
 *         deactivate LLC
 *     deactivate LLC
 * end
 * == Wait For Host Response - Rejected ==
 * alt Slave Role
 *     HCI --> LLC : HCI_LE_REM_CON_PARAM_REQ_NEG_REPLY_CMD
 *     LLC -> LLC: hci_le_rem_con_param_req_reply_neg_cmd_handler()
 *     activate LLC
 *         LLC --> HCI: HCI_CMD_STATUS_EVT
 *         LLC -> LLC: llc_rem_con_upd_proc_continue(HOST_RPLY, Reject)
 *         activate LLC
 *             hnote over LLC: REM_PROC=Idle
 *             LLC-->LLD: LLCP_REJECT_EXT_IND
 *         deactivate LLC
 *     deactivate LLC
 * end
 * == Wait For Connection Update Indication (Slave Only) ==
 * alt Slave Role
 *     LLD --> LLC : LLCP_CON_UPD_IND
 *     LLC -> LLC: llcp_con_upd_ind_handler()
 *     activate LLC
 *         note over LLC #aqua: Store Connection parameters programmed by peer
 *         LLC -> LLC: llc_rem_con_upd_proc_continue(CON_UPD_IND)
 *         activate LLC
 *             note over LLC: state = WAIT_CON_UPD_INSTANT
 *             note over LLC #aqua: Configure new connection parameter at expected instant
 *             LLC -> LLD: lld_con_param_update()
 *         deactivate LLC
 *     deactivate LLC
 * end
 * == Wait For Connection Update Instant (Slave Only) ==
 * alt Slave Role, PHY Updated
 *     LLD --> LLC : LLD_CON_UPD_CFM
 *     LLC -> LLC: lld_con_upd_cfm_handler()
 *     activate LLC
 *         LLC -> LLC: llc_rem_con_upd_proc_continue(CON_UPD_INSTANT)
 *         activate LLC
 *             note over LLC #aqua: Store new connection parameters
 *             hnote over LLC: REM_PROC=Idle
 *             alt If parameter changed or host involved
 *                 LLC-->HCI: HCI_LE_CON_UPD_CMP_EVT
 *             end
 *         deactivate LLC
 *     deactivate LLC
 * end
 * @enduml
 *
 * @param[in] link_id Link identifier
 * @param[in] state   Expected state of the procedure
 * @param[in] status  Status of the operation
 ****************************************************************************************
 */
__STATIC void llc_rem_con_upd_proc_continue(uint8_t link_id, uint8_t state, uint8_t status)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(PROC_CONTINUE_PATCH_TYPE, LLC_REM_CON_UPD_PROC_CONTINUE_C_FUN_BIT, link_id, state, status);
    bool finished = false;
    bool filter_hci_evt = false;
    // Gets the values requested by the peer
    struct llc_op_con_upd_ind *param = (struct llc_op_con_upd_ind *)llc_proc_get(link_id, LLC_PROC_REMOTE);
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];

    if (state == LLC_REM_CON_UPD_ERROR)
    {
//        ASSERT_ERR(param != NULL);
        llc_proc_state_set(&param->proc, link_id, LLC_REM_CON_UPD_ERROR);

        // Restore the TX length as per the current interval
        lld_con_tx_len_update_for_intv(link_id, llc_env_ptr->con_params.interval);
    }

    // check that current procedure state equals to expected state given in parameter
    if ((llc_proc_state_get(&param->proc) != state))
    {
        ASSERT_WARN(0, llc_proc_state_get(&param->proc), state);
        // unexpected state, ignore it
    }
    else
    {
        switch (llc_proc_state_get(&param->proc))
        {
        // Here we are slave
        case LLC_REM_WAIT_CON_UPD_IND:
        {
            if (status != CO_ERROR_NO_ERROR)
            {
                finished = true;
                break;
            }

            // Stop the LLCP Response TO
            llc_proc_timer_set(link_id, LLC_PROC_REMOTE, false);
            param->forced = true;
        }
        // no break
        case LLC_REM_CON_UPD_START:
        {
            // procedure with instant started
            SETB(llc_env_ptr->link_info, LLC_INFO_INSTANT_PROC, true);

            if (param->forced)
            {
                // Set the connection parameters in the LLD environment
                status = lld_con_param_update(link_id, param->win_size, param->win_off, param->comp_interval, param->con_latency,
                                              param->superv_to, param->instant);

                if (status != CO_ERROR_NO_ERROR)
                {
                    finished = true;
                }
                else
                {
                    llc_proc_state_set(&param->proc, link_id, LLC_REM_WAIT_CON_UPD_INSTANT);
                }
                break;
            }
            else
            {
                // If the parameters are the same (or very similar, i.e. no need for update)
                // do not send the request to the host
                if ((param->con_intv_min <= llc_env_ptr->con_params.interval)
                        && (param->con_intv_max >= llc_env_ptr->con_params.interval)
                        && (param->con_latency  == llc_env_ptr->con_params.latency)
                        && (param->superv_to    == llc_env_ptr->con_params.timeout))
                {
                    // keep same connection interval and do like if host accept request to send response
                    param->con_intv_min = llc_env_ptr->con_params.interval;
                    param->con_intv_max = llc_env_ptr->con_params.interval;
                }
                else
                {
                    //Send indication to the host
                    llc_hci_con_param_req_evt_send(link_id, param->con_intv_min, param->con_intv_max, param->con_latency,
                                                   param->superv_to);

                    llc_proc_state_set(&param->proc, link_id, LLC_REM_WAIT_HOST_RPLY);
                    // host is informed about parameter update request so an event has to be triggered
                    param->host_neg = true;

                    break;
                }
            }
        }
        // no break

        case LLC_REM_WAIT_HOST_RPLY: // Slave or Master
        {
            if (status == CO_ERROR_NO_ERROR)
            {
                // SLAVE device
                if (!GETB(llc_env_ptr->link_info, LLC_INFO_MASTER_ROLE))
                {
                    // compute prefered parameters
                    llc_pref_param_compute(link_id, false, param);
                    // send the response
                    llc_ll_connection_param_rsp_pdu_send(link_id, param);
                    llc_proc_state_set(&param->proc, link_id, LLC_REM_WAIT_CON_UPD_IND);
                    // Start the LLCP Response TO
                    llc_proc_timer_set(link_id, LLC_PROC_REMOTE, true);

                    // Update the TX length as per the minimum proposed interval
                    lld_con_tx_len_update_for_intv(link_id, param->con_intv_min);
                }
                else // MASTER
                {
                    // request a local operation
                    ke_task_id_t llc_id = KE_BUILD_ID(TASK_LLC, link_id);
                    struct llc_op_con_upd_ind *con_upd = KE_MSG_ALLOC(LLC_OP_CON_UPD_IND, llc_id, llc_id, llc_op_con_upd_ind);

                    memcpy(con_upd, param, sizeof(struct llc_op_con_upd_ind));
                    con_upd->proc.error_cb = llc_loc_con_upd_proc_err_cb;
                    llc_proc_state_set(&con_upd->proc, link_id, LLC_LOC_CON_UPD_FORCED);
                    ke_msg_send(con_upd);
                    filter_hci_evt = true;
                    finished = true;
                    break;
                }
            }
            else
            {
                llc_ll_reject_ind_pdu_send(link_id, LL_CONNECTION_PARAM_REQ_OPCODE, status, NULL);
                finished = true;
                filter_hci_evt = true;
            }
        }
        break;
        case LLC_REM_WAIT_CON_UPD_INSTANT:
        case LLC_REM_CON_UPD_ERROR:
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
#if (BT_53) && (BLE_PERIPHERAL)
        // check if connection interval changed
        bool intv_changed = llc_env_ptr->con_params.interval != param->comp_interval;
#endif // (BT_53) && (BLE_PERIPHERAL)

        // filter HCI event if Remote procedure becomes a Local procedure
        if (!filter_hci_evt)
        {
            // try to send connection update complete event
            llc_hci_con_upd_info_send(link_id, status, param);
        }

        // unregister procedure
        llc_proc_unreg(link_id, LLC_PROC_REMOTE);

        // procedure with instant finished
        SETB(llc_env_ptr->link_info, LLC_INFO_INSTANT_PROC, false);

#if (BT_53) && (BLE_PERIPHERAL)
        // Shall not restore subrating if connection interval changed, SLAVE device context
        if (intv_changed && !GETB(llc_env_ptr->link_info, LLC_INFO_MASTER_ROLE))
        {
            // set subrate parameters back to default
            llc_subrate_restore(link_id, LLC_SUBRATE_NO_RESTORE);
        }
#endif // (BT_53) && (BLE_PERIPHERAL)
    }
}

/**
 ****************************************************************************************
 *  @brief Check if the parameters are in the range
 *
 * @param[in] interval     Connection interval.
 * @param[in] latency      Connection latency
 * @param[in] timeout      Connection Timeout
 *
 * @return status code of handler:
 *    - True:   all parameters are in range
 *    - False:  at least one parameter is not in range
 ****************************************************************************************
 */
__STATIC bool llc_con_upd_param_in_range(uint8_t link_id, uint16_t interval_max, uint16_t interval_min, uint16_t latency, uint16_t timeout)
{
    uint16_t con_interval_min = CON_INTERVAL_MIN;

#if (BLE_ISO_MODE_0)
    // Check if link is configured for audio mode 0
    if (lli_am0_check(link_id))
    {
        con_interval_min = CON_INTERVAL_MIN_AUDIO;
    }
#endif // (BLE_ISO_MODE_0)

    if ((interval_min > interval_max)      || (latency > CON_LATENCY_MAX)
            || (interval_max > CON_INTERVAL_MAX)  || (interval_min < con_interval_min)
            || (timeout > CON_SUP_TO_MAX)         || (timeout < CON_SUP_TO_MIN)
            // CSA/ESR6 : supervision timeout minimum value
            // The Supervision_Timeout parameter defines the link supervision timeout for the connection. The
            // Supervision_Timeout in milliseconds shall be larger than (1 + Conn_Latency) * Conn_Interval_Max * 2,
            // where Conn_Interval_Max is given in milliseconds. (See [Vol 6] Part B, Section 4.5.2).
            // supervision timeout (mult of 10 ms); conn interval (mult of 1.25 ms)
            // to avoid floating point we do the following conversion (due to the shall we add 1 to ceil the result)
            // ((1+data->latency)*data->interval*2*1.25) => ((data->timeout*10) < (((1+data->latency)*data->interval*5 + 1)>>1)
            || ((timeout * 10) < ((1 + latency) * ((interval_max * 5) >> 1))))
    {
        return false;
    }
    else
    {
        return true;
    }
}

/**
 * Compute local device parameters to update connection
 *
 * @param[in]     link_id       Link Identifier
 * @param[in]     local_proc    Indicates whether the procedure is local (true) or not (false)
 * @param[in|out] param         Negotiated connection parameters
 */
__STATIC void llc_pref_param_compute(uint8_t link_id, bool local_proc, struct llc_op_con_upd_ind *param)
{
    do
    {
        struct llc_env_tag *llc_env_ptr = llc_env[link_id];
        struct sch_plan_req_param req_param;
        struct sch_plan_elt_tag *plan_elt = llm_plan_elt_get(link_id);

        // to handle back compatibility with 4.x
        if (local_proc)
        {
            param->pref_period   = 1;
        }

        // If the peer has an offset preference
        if ((!param->req_by_host) && (!param->host_neg) && (param->offset0 != PARAM_REQ_INVALID_OFFSET) && (param->offset0 != 0))
        {
            // Check if it can be accommodated
            struct sch_plan_chk_param chk_param;
            chk_param.interval     = (param->con_intv_min << 2);
            chk_param.duration_min = co_max(param->ce_len_min * 2, 4); // Reserve at least two slots
            chk_param.conhdl       = BLE_LINKID_TO_CONHDL(link_id);
            chk_param.conhdl_ref   = BLE_LINKID_TO_CONHDL(link_id);
            chk_param.offset       = CO_MOD((plan_elt->offset + (param->offset0 << 2)), (param->con_intv_min << 2));
            chk_param.margin       = 1 * !GETB(llc_env_ptr->link_info, LLC_INFO_MASTER_ROLE);
            if (sch_plan_chk(&chk_param) == SCH_PLAN_ERROR_OK)
            {
                param->comp_offset   = param->offset0;
                param->comp_interval = (chk_param.interval >> 2);
                break;
            }
        }

        // Consult the planner for possible offset
        req_param.interval_min = (param->con_intv_min << 2);
        req_param.interval_max = (param->con_intv_max << 2);
        req_param.duration_min = co_max(param->ce_len_min * 2, 4); // Reserve at least two slots
        req_param.duration_max = co_max(param->ce_len_max * 2, req_param.duration_min);
        req_param.offset_min = 0;
        req_param.offset_max = req_param.interval_max - 1;
        req_param.pref_period  = (param->pref_period << 2);
        req_param.conhdl       = BLE_LINKID_TO_CONHDL(link_id);
        req_param.conhdl_ref   = BLE_LINKID_TO_CONHDL(link_id);
        req_param.margin       = 1 * !GETB(llc_env_ptr->link_info, LLC_INFO_MASTER_ROLE);

        if (sch_plan_req(&req_param) == SCH_PLAN_ERROR_OK)
        {
            uint32_t con_evt_time_hs;
            uint16_t con_evt_cnt;

            // Select the minimum offset
            uint32_t offset = req_param.offset_min;

            // If update for a connection move
            if ((!param->req_by_host) && (!param->host_neg))
            {
                // Select an offset in the middle of the available range
                offset += (req_param.offset_max - req_param.offset_min) / 2;
            }

            // Get current connection event time
            lld_con_time_get(link_id, &con_evt_cnt, &con_evt_time_hs, NULL);

            // Compute reference connection event time
            con_evt_time_hs = CLK_ADD_2(con_evt_time_hs, BLE_UTIL_EVT_CNT_DIFF(param->ref_con_event_count, con_evt_cnt) * req_param.interval);

            // Compute offset from reference connection event to new connection event (in half-slot)
            offset = CO_MOD(CLK_SUB(con_evt_time_hs, offset), req_param.interval);
            if (offset > 0)
            {
                offset = req_param.interval - offset;
            }

            param->comp_offset = offset >> 2; // in 1.25ms frames
            param->comp_interval = (req_param.interval >> 2); // in 1.25ms frames
        }
        else
        {
            param->comp_offset   = 0;
            param->comp_interval = param->con_intv_max;
        }
    }
    while (0);
}

/**
 ****************************************************************************************
 * @brief Sends the connection update request pdu.
 *
 ****************************************************************************************
 */
__STATIC void llc_ll_connection_update_ind_pdu_send(uint8_t link_id, struct llc_op_con_upd_ind *param)
{
    struct ll_connection_update_ind pdu;

    pdu.op_code  = LL_CONNECTION_UPDATE_IND_OPCODE;
    pdu.win_size = param->win_size;
    pdu.win_off  = param->win_off;
    pdu.interv   = param->comp_interval;
    pdu.latency  = param->con_latency;
    pdu.timeout  = param->superv_to;
    pdu.instant  = param->instant;

    llc_llcp_send(link_id, (union llcp_pdu *) &pdu, NULL);
}

/**
 ****************************************************************************************
 * @brief Sends the connection parameters request pdu.
 *
 ****************************************************************************************
 */
__STATIC void llc_ll_connection_param_req_pdu_send(uint8_t link_id, struct llc_op_con_upd_ind *param)
{
    struct ll_connection_param_req pdu;

    pdu.op_code             = LL_CONNECTION_PARAM_REQ_OPCODE;
    pdu.interval_min        = param->con_intv_min;
    pdu.interval_max        = param->con_intv_max;
    pdu.latency             = param->con_latency;
    pdu.timeout             = param->superv_to;
    pdu.pref_period         = param->pref_period;
    pdu.ref_con_event_count = param->ref_con_event_count;
    pdu.offset0             = param->offset0;
    pdu.offset1             = param->offset1;
    pdu.offset2             = param->offset2;
    pdu.offset3             = param->offset3;
    pdu.offset4             = param->offset4;
    pdu.offset5             = param->offset5;

    llc_llcp_send(link_id, (union llcp_pdu *) &pdu, NULL);
}

/**
 ****************************************************************************************
 * @brief Sends the connection parameters response pdu.
 *
 ****************************************************************************************
 */
__STATIC void llc_ll_connection_param_rsp_pdu_send(uint8_t link_id, struct llc_op_con_upd_ind *param)
{
    struct ll_connection_param_rsp pdu;

    pdu.op_code             = LL_CONNECTION_PARAM_RSP_OPCODE;
    pdu.interval_min        = param->con_intv_min;
    pdu.interval_max        = param->con_intv_max;
    pdu.latency             = param->con_latency;
    pdu.timeout             = param->superv_to;
    pdu.pref_period         = param->pref_period;
    pdu.ref_con_event_count = param->ref_con_event_count;
    pdu.offset0             = param->offset0;
    pdu.offset1             = param->offset1;
    pdu.offset2             = param->offset2;
    pdu.offset3             = param->offset3;
    pdu.offset4             = param->offset4;
    pdu.offset5             = param->offset5;

    llc_llcp_send(link_id, (union llcp_pdu *) &pdu, NULL);
}

/**
 ****************************************************************************************
 * @brief Indicate the request to the host
 *
 * @param[in] link_id        Link identifier on which the pdu will be sent
 * @param[in] status         Status of the host request
 ****************************************************************************************
 */
__STATIC void llc_hci_con_param_req_evt_send(uint8_t link_id, uint16_t con_intv_min, uint16_t con_intv_max, uint16_t con_latency, uint16_t superv_to)
{
    uint16_t conhdl = BLE_LINKID_TO_CONHDL(link_id);
    // Indicate to the host  the request
    struct hci_le_rem_con_param_req_evt *event = KE_MSG_ALLOC(HCI_LE_EVENT, conhdl, HCI_LE_META_EVT_CODE, hci_le_rem_con_param_req_evt);

    // fill event parameters
    event->subcode      = HCI_LE_REM_CON_PARAM_REQ_EVT_SUBCODE;
    event->conhdl       = conhdl;
    event->interval_min = con_intv_min;
    event->interval_max = con_intv_max;
    event->latency      = con_latency;
    event->timeout      = superv_to;

    // send the message
    hci_send_2_host(event);
}

/**
 ****************************************************************************************
 * @brief Indicate the request to the host
 *
 * @param[in] link_id        Link identifier on which the pdu will be sent
 * @param[in] status         Status of the host request
 * @param[in] param          Connection update operation parameters
 ****************************************************************************************
 */
__STATIC void llc_hci_con_upd_info_send(uint8_t link_id, uint8_t status, struct llc_op_con_upd_ind *param)
{
    uint16_t conhdl = BLE_LINKID_TO_CONHDL(link_id);
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];
    bool trigger_event = (param->req_by_host | param->host_neg);

    if (status == CO_ERROR_NO_ERROR)
    {
        // check if parameters has been updated
        if ((param->comp_interval != llc_env_ptr->con_params.interval)
                || (param->con_latency   != llc_env_ptr->con_params.latency)
                || (param->superv_to     != llc_env_ptr->con_params.timeout))
        {
            // store new connection parameters
            llc_env_ptr->con_params.interval = param->comp_interval;
            llc_env_ptr->con_params.latency  = param->con_latency;
            llc_env_ptr->con_params.timeout  = param->superv_to;
            trigger_event = true;
        }

        // Update the planner interval element
        {
            struct sch_plan_elt_tag *plan_elt = llm_plan_elt_get(link_id);

            // Calculate sync window in half-us
            uint32_t rx_win_size = 2 * ((1 + param->con_latency) * 4 * param->comp_interval * (rwip_max_drift_get() + co_sca2ppm[llc_env_ptr->con_params.sca]) / 1600); // half-slots * ppm * 625 half-us / 1000000;

            // Unregister the old connection from bandwidth allocation system
            sch_plan_rem(plan_elt);

            // Store new parameters
            plan_elt->interval = llc_env_ptr->con_params.interval << 2;
            plan_elt->offset = lld_con_offset_get(link_id);
            plan_elt->duration_min = co_max(param->ce_len_min * 2, 4);
            plan_elt->duration_max = co_max(param->ce_len_max * 2, 4);
            plan_elt->margin = 1 + !GETB(llc_env_ptr->link_info, LLC_INFO_MASTER_ROLE) * ((rx_win_size + HALF_SLOT_SIZE / 2) / HALF_SLOT_SIZE);

            // Register the connection in bandwidth allocation system
            sch_plan_set(plan_elt);
        }
    }

    // check if HCI completed event triggered
    if (trigger_event)
    {
        // Allocate the status event message
        struct hci_le_con_update_cmp_evt *event = KE_MSG_ALLOC(HCI_LE_EVENT, conhdl, 0, hci_le_con_update_cmp_evt);

        // fill parameters
        event->subcode      = HCI_LE_CON_UPDATE_CMP_EVT_SUBCODE;
        event->status       = status;
        event->conhdl       = conhdl;
        event->con_interval = llc_env_ptr->con_params.interval;
        event->con_latency  = llc_env_ptr->con_params.latency;
        event->sup_to       = llc_env_ptr->con_params.timeout;

        // send the message
        hci_send_2_host(event);
    }

    // allow new request by host, mark HCI procedure not under process
    if (param->req_by_host)
    {
        SETB(llc_env_ptr->flow_ctrl, LLC_HCI_CON_UPDATE_REQ, false);
    }
}

/**
 ****************************************************************************************
 * @brief Handles the connection update error (UNKNOWN/REJECT).
 *
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC void llc_loc_con_upd_proc_err_cb(uint8_t link_id, uint8_t error_type, void *param)
{
    uint8_t status = CO_ERROR_NO_ERROR;
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];
    uint8_t state  = GETB(llc_env_ptr->link_info, LLC_INFO_MASTER_ROLE)
                     ? LLC_LOC_WAIT_CON_PARAM_RSP : LLC_LOC_WAIT_CON_UPD_IND;

    switch (error_type)
    {
    case LLC_ERR_DISCONNECT:
    {
        state  = LLC_LOC_CON_UPD_ERROR;
        status = *((uint8_t *) param);
    }
    break;
    case LLC_ERR_LLCP_UNKNOWN_RSP:
    {
        struct ll_unknown_rsp *rsp = (struct ll_unknown_rsp *) param;
        if (rsp->unk_type == LL_CONNECTION_PARAM_REQ_OPCODE)
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
        if (reject_ext->rej_op_code == LL_CONNECTION_PARAM_REQ_OPCODE)
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
        if ((status == CO_ERROR_UNKNOWN_LMP_PDU) || (status == CO_ERROR_UNSUPPORTED_REMOTE_FEATURE))
        {
            llc_le_feature_set(link_id, BLE_FEAT_CON_PARAM_REQ_PROC, false);

            // Mark the corresponding planner element as not movable when in the slave role
            if (!GETB(llc_env_ptr->link_info, LLC_INFO_MASTER_ROLE))
            {
                struct sch_plan_elt_tag *plan_elt = llm_plan_elt_get(link_id);

                ASSERT_ERR(plan_elt != NULL);
                ASSERT_ERR(plan_elt->interval != 0);

                // Mark the interval element as not movable
                plan_elt->cb_move = NULL;
                plan_elt->mobility = SCH_PLAN_MB_LVL_0;
            }
        }

        llc_loc_con_upd_proc_continue(link_id, state, status);
    }
}
/**
 ****************************************************************************************
 * @brief Handles the connection update error (UNKNOWN/REJECT).
 *
 * PlantUML remote connection update error description
 *
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC void llc_rem_con_upd_proc_err_cb(uint8_t link_id, uint8_t error_type, void *param)
{
    uint8_t status  = CO_ERROR_NO_ERROR;
    uint8_t state   = LLC_REM_WAIT_CON_UPD_IND;

    switch (error_type)
    {
    // This case is out of specification scope but already arrived during UPF
    case LLC_ERR_LLCP_REJECT_IND:
    {
        struct ll_reject_ind *reject = (struct ll_reject_ind *) param;
        status =  reject->err_code;
    }
    break;

    case LLC_ERR_LLCP_REJECT_EXT_IND:
    {
        struct ll_reject_ext_ind *reject_ext = (struct ll_reject_ext_ind *)param;
        if (reject_ext->rej_op_code == LL_CONNECTION_PARAM_RSP_OPCODE)
        {
            status =  reject_ext->err_code;
        }
    }
    break;

    case LLC_ERR_DISCONNECT:
    {
        status = *((uint8_t *) param);
        state  = LLC_REM_CON_UPD_ERROR;
    }
    break;

    default: /* Ignore*/
        break;
    }

    if (status != CO_ERROR_NO_ERROR)
    {
        llc_rem_con_upd_proc_continue(link_id, state, status);
    }
}

/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 *  @brief Handles the reception of a LLCP connection parameter update request
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
uint8_t ll_connection_update_ind_handler(uint8_t link_id, struct ll_connection_update_ind *pdu, uint16_t event_cnt)
{
    //Get environment pointer
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];
    uint8_t status = CO_ERROR_NO_ERROR;
    //Check the parameters range
    if (!llc_con_upd_param_in_range(link_id, pdu->interv, pdu->interv, pdu->latency, pdu->timeout))
    {
        status = CO_ERROR_UNSUPPORTED_LMP_PARAM_VALUE;
    }
    // Device must be slave of the link
    else if (GETB(llc_env_ptr->link_info, LLC_INFO_MASTER_ROLE))
    {
        status = CO_ERROR_LMP_PDU_NOT_ALLOWED;
    }
    // Check if instant has passed
    else if (BLE_UTIL_INSTANT_PASSED(pdu->instant, event_cnt))
    {
        status = CO_ERROR_INSTANT_PASSED;
    }
    else
    {
        struct llc_op_con_upd_ind *param;

        // check that local procedure is the correct one
        if (llc_proc_id_get(link_id, LLC_PROC_LOCAL) == LLC_PROC_CON_UPDATE)
        {
            param = (struct llc_op_con_upd_ind *) llc_proc_get(link_id, LLC_PROC_LOCAL);

            param->comp_interval  = pdu->interv;
            param->con_latency    = pdu->latency;
            param->superv_to      = pdu->timeout;
            param->instant        = pdu->instant;
            param->win_off        = pdu->win_off;
            param->win_size       = pdu->win_size;

            // continue procedure execution
            llc_loc_con_upd_proc_continue(link_id, LLC_LOC_WAIT_CON_UPD_IND, status);
        }
        else
        {
            uint8_t state = LLC_REM_WAIT_CON_UPD_IND;

            // Parameter update to be started
            if (llc_proc_id_get(link_id, LLC_PROC_REMOTE) == LLC_PROC_NONE)
            {
                param = KE_MSG_ALLOC(LLC_OP_CON_UPD_IND, TASK_LLC, TASK_LLC, llc_op_con_upd_ind);
                llc_proc_init(&param->proc, LLC_PROC_CON_UPDATE, llc_rem_con_upd_proc_err_cb);
                param->ce_len_max = 0;
                param->ce_len_min = 0;
                param->req_by_host   = false;
                param->host_neg    = false;
                param->offset0     = PARAM_REQ_INVALID_OFFSET;

                // register procedure
                llc_proc_reg(link_id, LLC_PROC_REMOTE, &(param->proc));
                // update the local state
                state = LLC_REM_CON_UPD_START;
            }

            // Continue parameter update Procedure
            if (llc_proc_id_get(link_id, LLC_PROC_REMOTE) == LLC_PROC_CON_UPDATE)
            {
                param = (struct llc_op_con_upd_ind *)llc_proc_get(link_id, LLC_PROC_REMOTE);
                llc_proc_state_set(&param->proc, link_id, state);
                param->comp_interval  = pdu->interv;
                param->con_latency    = pdu->latency;
                param->superv_to      = pdu->timeout;
                param->instant        = pdu->instant;
                param->win_off        = pdu->win_off;
                param->win_size       = pdu->win_size;
                param->forced         = true;

                llc_rem_con_upd_proc_continue(link_id, state, CO_ERROR_NO_ERROR);
            }
            else
            {
                status = CO_ERROR_LMP_PDU_NOT_ALLOWED;
            }
        }
    }

    return (status);
}

/**
 ****************************************************************************************
 *  @brief Handles the reception of a LLCP connection parameters request
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
uint8_t ll_connection_param_req_handler(uint8_t link_id, struct ll_connection_param_req *pdu, uint16_t event_cnt)
{
    //Get environment pointer
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];
    uint8_t status = CO_ERROR_NO_ERROR;


    // check that no remote procedure is on-going
    if (llc_proc_id_get(link_id, LLC_PROC_REMOTE) != LLC_PROC_NONE)
    {
        // unexpected message, reject it.
        status = CO_ERROR_LMP_PDU_NOT_ALLOWED;
    }
    // If values are valid or not
    else if (!llc_con_upd_param_in_range(link_id, pdu->interval_max, pdu->interval_min, pdu->latency, pdu->timeout))
    {
        //Return an error to send a ll_reject_ind
        status = CO_ERROR_INVALID_LMP_PARAM;
    }
    // check if HCI event is mask or not
    else if (!llm_le_evt_mask_check(LE_EVT_MASK_REM_CON_PARA_REQ_EVT_BIT))
    {
        // reject to inform that remote feature is not supported
        status = CO_ERROR_UNSUPPORTED_REMOTE_FEATURE;
    }
    else
    {
        // Check if there is a possible procedure collision
        if (GETB(llc_env_ptr->link_info, LLC_INFO_MASTER_ROLE))
        {
            status = llc_proc_collision_check(link_id, LLC_PROC_CON_UPDATE);
        }

        // remote procedure can be started
        if (status == CO_ERROR_NO_ERROR)
        {
            ke_task_id_t llc_id = KE_BUILD_ID(TASK_LLC, link_id);
            struct llc_op_con_upd_ind *con_upd = KE_MSG_ALLOC(LLC_OP_CON_UPD_IND, llc_id, llc_id, llc_op_con_upd_ind);
            memset(con_upd, 0, sizeof(struct llc_op_con_upd_ind));
            llc_proc_init(&con_upd->proc, LLC_PROC_CON_UPDATE, llc_rem_con_upd_proc_err_cb);
            llc_proc_state_set(&con_upd->proc, link_id, LLC_REM_CON_UPD_START);
            con_upd->con_intv_min         = pdu->interval_min;
            con_upd->con_intv_max         = pdu->interval_max;
            con_upd->con_latency          = pdu->latency;
            con_upd->superv_to            = pdu->timeout;
            con_upd->pref_period          = pdu->pref_period;
            con_upd->ref_con_event_count  = pdu->ref_con_event_count;
            con_upd->offset0              = pdu->offset0;
            con_upd->offset1              = pdu->offset1;
            con_upd->offset2              = pdu->offset2;
            con_upd->offset3              = pdu->offset3;
            con_upd->offset4              = pdu->offset4;
            con_upd->offset5              = pdu->offset5;
            con_upd->req_by_host          = false;
            con_upd->host_neg             = false;
            con_upd->ce_len_max           = 0;
            con_upd->ce_len_min           = 0;
            con_upd->forced               = false;

            // store local procedure
            llc_proc_reg(link_id, LLC_PROC_REMOTE, &(con_upd->proc));
            // execute feature exchange procedure
            llc_rem_con_upd_proc_continue(link_id, LLC_REM_CON_UPD_START, CO_ERROR_NO_ERROR);
        }
    }

    return (status);
}

/**
 ****************************************************************************************
 *  @brief Handles the reception of a LLCP connection parameters response
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
uint8_t ll_connection_param_rsp_handler(uint8_t link_id, struct ll_connection_param_rsp *param, uint16_t event_cnt)
{
    uint8_t status = CO_ERROR_NO_ERROR;

    // check that local procedure is on-going, else reject
    if (llc_proc_id_get(link_id, LLC_PROC_LOCAL) == LLC_PROC_CON_UPDATE)
    {
        // Gets the values requested by the peer
        struct llc_op_con_upd_ind *con_upd = (struct llc_op_con_upd_ind *)llc_proc_get(link_id, LLC_PROC_LOCAL);

        if (!llc_con_upd_param_in_range(link_id, param->interval_max, param->interval_min, param->latency, param->timeout))
        {
            //Return an error to send a ll_reject_ind
            status = CO_ERROR_INVALID_LMP_PARAM;
        }
        // Check if the peer parameters are compatible with the host connection update request, else reject
        else if ((param->interval_min >= con_upd->con_intv_min)
                 && (param->interval_min <= con_upd->con_intv_max)
                 && (param->interval_max <= con_upd->con_intv_max)
                 && (param->interval_max >= con_upd->con_intv_min)
                 && (param->latency      == con_upd->con_latency))
        {

            // Merge information
            con_upd->con_intv_min   = co_max(con_upd->con_intv_min, param->interval_min);
            con_upd->con_intv_min   = co_max(con_upd->con_intv_max, param->interval_max);
            con_upd->superv_to      = co_max(con_upd->superv_to, param->timeout);
            con_upd->win_off        = 0;
            con_upd->win_size       = 0;
        }
        else
        {
            status = CO_ERROR_UNSUPPORTED_LMP_PARAM_VALUE;
        }

        // continue local procedure execution
        llc_loc_con_upd_proc_continue(link_id, LLC_LOC_WAIT_CON_PARAM_RSP, status);
    }
    else
    {
        // unexpected message, reject it.
        status = CO_ERROR_LMP_PDU_NOT_ALLOWED;
    }

    return (status);
}


/**
 ****************************************************************************************
 * @brief Handles the command HCI connection update.
 *
 * PlantUML procedure description
 *
 * @startuml
 * title : Connection update procedure initiated by host
 * participant HCI
 * participant LLC
 * HCI --> LLC : HCI_LE_CON_UPD_REQ_CMD
 * LLC -> LLC: hci_le_con_upd_cmd_handler()
 * activate LLC
 * alt link disconnected
 *     LLC --> HCI : HCI_CMD_STAT_EVENT(DISALLOWED)
 * else  Con oaram upd procedure rejected by peer or feature not supported  and slave
 *     LLC --> HCI : HCI_CMD_STAT_EVENT(UNSUPPORTED_REMOTE_FEATURE)
 * else  Parameters out of range
 *     LLC --> HCI : HCI_CMD_STAT_EVENT(INVALID_HCI_PARAMS)
 * else  Procedure already started
 *     LLC --> HCI : HCI_CMD_STAT_EVENT(BUSY)
 * else  Procedure can be started
 *     LLC --> LLC : LLC_OP_CON_UPD_IND
 *     note right LLC #lightgreen: See __llc_op_con_upd_ind_handler(CON_PARAM_REQ_START)__
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
int hci_le_con_upd_cmd_handler(uint8_t link_id, struct hci_le_con_update_cmd const *param, uint16_t opcode)
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
    // check if peer support this feature in case of slave mode
    else if (!llc_le_feature_check(link_id, BLE_FEAT_CON_PARAM_REQ_PROC) && !GETB(llc_env_ptr->link_info, LLC_INFO_MASTER_ROLE))
    {
        status = CO_ERROR_UNSUPPORTED_REMOTE_FEATURE;
    }
    // ensure that host not currently processing request
    else if (GETB(llc_env_ptr->flow_ctrl, LLC_HCI_CON_UPDATE_REQ))
    {
        status = CO_ERROR_CONTROLLER_BUSY;
    }
    // Conn_Interval_Min is greater than the Conn_Interval_Max.
    else if (!llc_con_upd_param_in_range(link_id, param->con_intv_max, param->con_intv_min, param->con_latency, param->superv_to))
    {
        status = CO_ERROR_INVALID_HCI_PARAM;
    }
    else
    {
        ke_task_id_t llc_id = KE_BUILD_ID(TASK_LLC, link_id);
        struct llc_op_con_upd_ind *con_upd = KE_MSG_ALLOC(LLC_OP_CON_UPD_IND, llc_id, llc_id, llc_op_con_upd_ind);

        //If Master and connection parameter request has been rejected start immediately the connection update procedure
        llc_proc_init(&con_upd->proc, LLC_PROC_CON_UPDATE, llc_loc_con_upd_proc_err_cb);
        llc_proc_state_set(&con_upd->proc, link_id, LLC_LOC_CON_UPD_START);
        con_upd->ce_len_max     = co_min(param->ce_len_max, 2 * param->con_intv_max);
        con_upd->ce_len_min     = co_min(param->ce_len_min, 2 * param->con_intv_min);
        con_upd->con_intv_max   = param->con_intv_max;
        con_upd->con_intv_min   = param->con_intv_min;
        con_upd->con_latency    = param->con_latency;
        con_upd->superv_to      = param->superv_to;
        con_upd->forced         = !llc_le_feature_check(link_id, BLE_FEAT_CON_PARAM_REQ_PROC);
        con_upd->req_by_host    = true;
        con_upd->host_neg       = true;
        ke_msg_send(con_upd);

        // Mark HCI procedure started and waiting to be granted.
        SETB(llc_env_ptr->flow_ctrl, LLC_HCI_CON_UPDATE_REQ, true);
    }

    // Send the command status event
    llc_cmd_stat_send(link_id, opcode, status);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the command HCI remote connection parameter request reply.
 *
 * @param[in] link_id Link Identifier
 * @param[in] param   Pointer to the parameters of the message.
 * @param[in] opcode  HCI Operation code
 ****************************************************************************************
 */
int hci_le_rem_con_param_req_reply_cmd_handler(uint8_t link_id, struct hci_le_rem_con_param_req_reply_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    // check if state is Free or in disconnected state
    if (!llc_is_disconnecting(link_id))
    {

        // Check if the peer parameters are in range
        if (!llc_con_upd_param_in_range(link_id, param->interval_max, param->interval_min, param->latency, param->timeout))
        {
            status = CO_ERROR_UNSUPPORTED_LMP_PARAM_VALUE;
        }
        else
        {
            // Gets the values requested by the peer
            struct llc_op_con_upd_ind *con_upd = (struct llc_op_con_upd_ind *)llc_proc_get(link_id, LLC_PROC_REMOTE);

            if (con_upd != NULL)
            {
                // Merge information
                con_upd->con_intv_max   = co_max(con_upd->con_intv_max, param->interval_max);
                con_upd->con_intv_min   = co_max(con_upd->con_intv_min, param->interval_min);
                con_upd->superv_to      = co_max(con_upd->superv_to, param->timeout);

                llc_rem_con_upd_proc_continue(link_id, LLC_REM_WAIT_HOST_RPLY, CO_ERROR_NO_ERROR);

                status = CO_ERROR_NO_ERROR;
            }
        }
    }
    // Send the command complete event
    llc_cmd_cmp_send(link_id, opcode, status);
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the command HCI remote connection parameter request negative reply
 *
 * @param[in] link_id Link Identifier
 * @param[in] param   Pointer to the parameters of the message.
 * @param[in] opcode  HCI Operation code
 ****************************************************************************************
 */
int hci_le_rem_con_param_req_neg_reply_cmd_handler(uint8_t link_id, struct hci_le_rem_con_param_req_neg_reply_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    // check if state is Free or in disconnected state
    if (!llc_is_disconnecting(link_id))
    {
        // LL5.1.7.2 The Host shall only use the error code Unacceptable Connection Parameters (0x3B) in order to reject the request.
        if (param->reason != CO_ERROR_UNACCEPTABLE_CONN_PARAM)
        {
            status = CO_ERROR_INVALID_HCI_PARAM;
        }
        else if (llc_proc_get(link_id, LLC_PROC_REMOTE))
        {
            llc_rem_con_upd_proc_continue(link_id, LLC_REM_WAIT_HOST_RPLY, param->reason);
            status = CO_ERROR_NO_ERROR;
        }
    }
    // Send the command complete event
    llc_cmd_cmp_send(link_id, opcode, status);
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief  Handles the connection update procedure indication message.
 *
 * @startuml llc_op_con_upd_ind_handler.png
 * participant LLC
 *  --> LLC : LLC_OP_CON_UPD_IND
 * LLC -> LLC: llc_op_con_upd_ind_handler()
 * activate LLC
 * hnote over LLC : LOC_PROC = Busy
 * LLC -> LLC: llc_loc_con_upd_proc_continue(START)
 * activate LLC
 * note right LLC #lightgreen: See __llc_loc_con_upd_proc_continue()__
 * deactivate LLC
 * deactivate LLC
 * @enduml
 ****************************************************************************************
 */
int llc_op_con_upd_ind_handler(ke_msg_id_t const msgid, struct llc_op_con_upd_ind *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Current message status
    int msg_status = KE_MSG_CONSUMED;
    uint8_t link_id = KE_IDX_GET(dest_id);
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];

    // Check if state in disconnected state
    if (llc_is_disconnecting(link_id))
    {
        // disconnection on-going inform that command is aborted
        if (param->req_by_host)
        {
            llc_hci_con_upd_info_send(link_id, llc_env_ptr->disc_reason, param);
        }
        else
        {
            // mark local procedure not under process
            SETB(llc_env_ptr->flow_ctrl, LLC_LOC_CON_UPDATE_REQ, false);
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
        // check if procedure with instant on-going
        if (GETB(llc_env_ptr->link_info, LLC_INFO_INSTANT_PROC) && (llc_proc_state_get(&param->proc) == LLC_LOC_CON_UPD_START))
        {
            // process this message later
            msg_status = KE_MSG_SAVED;
        }
#if (BT_53) && (BLE_CENTRAL)
        // check to suspend subrating for this procedure if needed
        else if (llc_subrate_suspend(link_id))
        {
            // process this message later
            msg_status = KE_MSG_SAVED;
        }
#endif // (BT_53) && (BLE_CENTRAL)
        else
        {
            msg_status = KE_MSG_NO_FREE;

            ASSERT_INFO((llc_proc_state_get(&param->proc) == LLC_LOC_CON_UPD_START) || (llc_proc_state_get(&param->proc) == LLC_LOC_CON_UPD_FORCED),
                        link_id, llc_proc_state_get(&param->proc));

            // store local procedure
            llc_proc_reg(link_id, LLC_PROC_LOCAL, &(param->proc));

            // execute connection update procedure
            llc_loc_con_upd_proc_continue(link_id, llc_proc_state_get(&param->proc), CO_ERROR_NO_ERROR);
        }
    }
    return (msg_status);
}

/**
 ****************************************************************************************
 * @brief  Handles the connection update procedure indication message.
 *
 ****************************************************************************************
 */
int lld_con_param_upd_cfm_handler(ke_msg_id_t const msgid, void *param,
                                  ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Current message status
    int msg_status = KE_MSG_CONSUMED;
    uint8_t link_id = KE_IDX_GET(dest_id);
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];

    // Check if state in disconnected state
    //if (!llc_is_disconnecting(link_id))
    // Yier, During disconnecting, conn instant still could be updated, and will affect lld_con_offset_upd_ind_handler() when conn inv changed.
    if (llc_env_ptr &&
            ((llc_proc_id_get(link_id, LLC_PROC_LOCAL) == LLC_PROC_CON_UPDATE) || (llc_proc_id_get(link_id, LLC_PROC_REMOTE) == LLC_PROC_CON_UPDATE)))
    {

        //Get environment pointer
        if (llc_proc_id_get(link_id, LLC_PROC_LOCAL) == LLC_PROC_CON_UPDATE)
        {
            llc_loc_con_upd_proc_continue(link_id, LLC_LOC_WAIT_CON_UPD_INSTANT, CO_ERROR_NO_ERROR);
        }

        else if (llc_proc_id_get(link_id, LLC_PROC_REMOTE) == LLC_PROC_CON_UPDATE)
        {
            llc_rem_con_upd_proc_continue(link_id, LLC_REM_WAIT_CON_UPD_INSTANT, CO_ERROR_NO_ERROR);
        }

        if (llc_env_ptr != NULL)
        {
            // update LE_Ping timers
            llc_le_ping_set(link_id, llc_env_ptr->le_ping.auth_payl_to);
        }
    }

    return (msg_status);
}

/**
 ****************************************************************************************
 * @brief  Handles the connection offset update indication message.
 *
 ****************************************************************************************
 */
int lld_con_offset_upd_ind_handler(ke_msg_id_t const msgid, struct lld_con_offset_upd_ind *param, ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Current message status
    int msg_status = KE_MSG_CONSUMED;
    uint8_t link_id = KE_IDX_GET(dest_id);
    struct sch_plan_elt_tag *plan_elt = llm_plan_elt_get(link_id);
    uint32_t new_offset = param->con_offset;

    ASSERT_ERR(llc_env[link_id] != NULL);
    ASSERT_ERR(plan_elt->interval != 0);
    //ASSERT_INFO(param->con_offset < plan_elt->interval, param->con_offset, plan_elt->interval);
    // Yier, don't need update it in disconnecting stage
    if (param->con_offset >= plan_elt->interval)
    {
        if (llc_is_disconnecting(link_id) != 0)
        {
            ASSERT_INFO_FORCE(0, param->con_offset, plan_elt->interval);
        }
        else
            return msg_status;
    }

    // Check if offset has changed
    if (plan_elt->offset >= plan_elt->interval)
    {
        // Update offset
        plan_elt->offset = new_offset;

        // Register the connection in bandwidth allocation system
        sch_plan_set(plan_elt);
    }
    if (new_offset != plan_elt->offset)
    {
        // Update offset in the planner (for all activities related to this connection)
        sch_plan_shift(BLE_LINKID_TO_CONHDL(link_id), ((int16_t) new_offset - plan_elt->offset));
    }

    return (msg_status);
}

/**
 ****************************************************************************************
 * @brief Handle move notification
 ****************************************************************************************
 */
void llc_con_move_cbk(uint16_t id)
{
    uint8_t link_id = id;
    ASSERT_ERR(llc_env[link_id] != NULL);

    // If not disconnecting and there is no HCI or local connection update request
    if (!llc_is_disconnecting(link_id) && !GETB(llc_env[link_id]->flow_ctrl, LLC_HCI_CON_UPDATE_REQ) && !GETB(llc_env[link_id]->flow_ctrl, LLC_LOC_CON_UPDATE_REQ))
    {
        ke_task_id_t llc_id = KE_BUILD_ID(TASK_LLC, link_id);
        struct llc_op_con_upd_ind *con_upd = KE_MSG_ALLOC(LLC_OP_CON_UPD_IND, llc_id, llc_id, llc_op_con_upd_ind);
        struct sch_plan_elt_tag *plan_elt = llm_plan_elt_get(link_id);

        //If Master and connection parameter request has been rejected start immediately the connection update procedure
        llc_proc_init(&con_upd->proc, LLC_PROC_CON_UPDATE, llc_loc_con_upd_proc_err_cb);
        llc_proc_state_set(&con_upd->proc, link_id, LLC_LOC_CON_UPD_START);
        con_upd->ce_len_min     = plan_elt->duration_min / 2;
        con_upd->ce_len_max     = plan_elt->duration_max / 2;
        con_upd->con_intv_max   = llc_env[link_id]->con_params.interval;
        con_upd->con_intv_min   = llc_env[link_id]->con_params.interval;
        con_upd->con_latency    = llc_env[link_id]->con_params.latency;
        con_upd->superv_to      = llc_env[link_id]->con_params.timeout;
        con_upd->forced         = (GETB(llc_env[link_id]->link_info, LLC_INFO_MASTER_ROLE)) ? !llc_le_feature_check(link_id, BLE_FEAT_CON_PARAM_REQ_PROC) : false;
        con_upd->req_by_host    = false;
        con_upd->host_neg       = false;
        con_upd->offset0        = PARAM_REQ_INVALID_OFFSET;
        ke_msg_send(con_upd);

        // Mark local procedure started and waiting to be granted.
        SETB(llc_env[link_id]->flow_ctrl, LLC_LOC_CON_UPDATE_REQ, true);
    }
}

#endif // (BLE_CENTRAL || BLE_PERIPHERAL)
/// @} LLC_CON_UPD
