/**
 ****************************************************************************************
 *
 * @file llc_ch_map_upd.c
 *
 * @brief Handles the channel map update for both master and slave roles
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup LLC_ch_map_UPD
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
#include "ble_util.h"    // BLE utility functions

#include "ke_msg.h"      // Kernel message

#include "llc_int.h"    // Internal LLC API
#include "llc_llcp.h"   // Internal LLCP API

#include "lld.h"        // To update the channel map
#include "llm.h"        // To retrieve channel map to apply

#include "hci.h"         // For HCI handler

#include "rwip.h"         // For HCI handler
#include "btdm_patch.h"

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */


/// Channel map update operation indication structure definition
/*@TRACE*/
struct llc_op_ch_map_upd_ind
{
    /// Procedure information
    llc_procedure_t                 proc;
    /// Channel map to apply
    struct le_ch_map                ch_map;
    /// Instant when the channel map must be applied
    uint16_t                        instant;
};


/*
 * DEFINES
 ****************************************************************************************
 */

/*@TRACE*/
enum llc_ch_map_update_state
{
    // State machine of the local initiated channel map update
    /// Start channel map update procedure
    LLC_LOC_CH_MAP_UP_START,
    /// Wait for channel map update instant
    LLC_LOC_CH_MAP_WAIT_INSTANT,
    /// Error occurs during channel map update
    LLC_LOC_CH_MAP_ERROR,


    // State machine of the peer initiated channel map update
    /// Start channel map update procedure
    LLC_REM_CH_MAP_UP_START,
    /// Wait for channel map update instant
    LLC_REM_CH_MAP_WAIT_INSTANT,
    /// Error occurs during channel map update
    LLC_REM_CH_MAP_ERROR,
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
__STATIC void llc_llcp_ch_map_update_ind_pdu_send(uint8_t link_id, uint16_t instant, struct le_ch_map *ch_map);
__STATIC void llc_ch_map_up_proc_err_cb(uint8_t link_id, uint8_t error_type, void *param);

__STATIC void llc_rem_ch_map_proc_continue(uint8_t link_id, uint8_t state, uint8_t status);
__STATIC void llc_loc_ch_map_proc_continue(uint8_t link_id, uint8_t state, uint8_t status);
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
__STATIC void llc_loc_ch_map_proc_continue(uint8_t link_id, uint8_t state, uint8_t status)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(PROC_CONTINUE_PATCH_TYPE, LLC_LOC_CH_MAP_PROC_CONTINUE_C_FUN_BIT, link_id, state, status);
    /// Gets the LLC environment dedicated to this link
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];
    /// retrieve procedure parameters
    struct llc_op_ch_map_upd_ind *param = (struct llc_op_ch_map_upd_ind *) llc_proc_get(link_id, LLC_PROC_LOCAL);
    bool finished = false;

    // check that current procedure state equals to expected state given in parameter
    if (llc_proc_state_get(&param->proc) != state)
    {
        // unexpected state, terminate procedure
        finished = true;
    }
    else
    {
        switch (llc_proc_state_get(&param->proc))
        {
        /// Procedure started
        case LLC_LOC_CH_MAP_UP_START:
        {
            // Set local channel map procedure ongoing
            SETB(llc_env_ptr->link_info, LLC_INFO_LOC_CH_MAP_UPD, true);

            // procedure with instant started
            SETB(llc_env_ptr->link_info, LLC_INFO_INSTANT_PROC, true);

            // Compute procedure instant
            param->instant = llc_inst_select(link_id);

            // request driver to perform channel map update
            if (lld_con_ch_map_update(link_id, &(param->ch_map), param->instant) == CO_ERROR_NO_ERROR)
            {
                // inform peer device about new channel map to apply
                llc_llcp_ch_map_update_ind_pdu_send(link_id, param->instant, &(param->ch_map));
                llc_proc_state_set(&param->proc, link_id, LLC_LOC_CH_MAP_WAIT_INSTANT);
            }
            else // update rejected.
            {
                finished = true;
            }
        }
        break;

        /// Procedure started
        case LLC_LOC_CH_MAP_WAIT_INSTANT:
        {
            finished = true;
            // copy information about the new applied channel map.
            memcpy(&(llc_env_ptr->con_params.ch_map), &(param->ch_map), sizeof(struct le_ch_map));
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
        // procedure with instant finished
        SETB(llc_env_ptr->link_info, LLC_INFO_INSTANT_PROC, false);

        // unregister procedure
        llc_proc_unreg(link_id, LLC_PROC_LOCAL);

        // Clear local channel map procedure ongoing
        SETB(llc_env_ptr->link_info, LLC_INFO_LOC_CH_MAP_UPD, false);

#if (BT_53) && (BLE_CENTRAL)
        // start procedure to resume subrating if needed
        llc_subrate_restore(link_id, LLC_SUBRATE_FULL_RESTORE);
#endif // (BT_53) && (BLE_CENTRAL)
    }
}


/**
 ****************************************************************************************
 * Continue execution of remote procedure
 *
 * @param[in] link_id Link identifier
 * @param[in] state   Expected state of the procedure
 * @param[in] status  Status of the operation
 ****************************************************************************************
 */
__STATIC void llc_rem_ch_map_proc_continue(uint8_t link_id, uint8_t state, uint8_t status)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(PROC_CONTINUE_PATCH_TYPE, LLC_REM_CH_MAP_PROC_CONTINUE_C_FUN_BIT, link_id, state, status);
    /// Gets the LLC environment dedicated to this link
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];
    /// retrieve procedure parameters
    struct llc_op_ch_map_upd_ind *param = (struct llc_op_ch_map_upd_ind *) llc_proc_get(link_id, LLC_PROC_REMOTE);
    bool finished = false;

    // check that current procedure state equals to expected state given in parameter
    if (llc_proc_state_get(&param->proc) != state)
    {
        // unexpected state, terminate procedure
        finished = true;
    }
    else
    {
        switch (llc_proc_state_get(&param->proc))
        {
        /// Procedure started
        case LLC_REM_CH_MAP_UP_START:
        {
            // procedure with instant started
            SETB(llc_env_ptr->link_info, LLC_INFO_INSTANT_PROC, true);
            // request driver to perform channel map update
            if (lld_con_ch_map_update(link_id, &(param->ch_map), param->instant) == CO_ERROR_NO_ERROR)
            {
                llc_proc_state_set(&param->proc, link_id, LLC_REM_CH_MAP_WAIT_INSTANT);
            }
            else // update rejected.
            {
                finished = true; // it means that link is in bad state and will be at least disconnected soon
            }
        }
        break;

        /// Procedure started
        case LLC_REM_CH_MAP_WAIT_INSTANT:
        {
            finished = true;
            // copy information about the new applied channel map.
            memcpy(&(llc_env_ptr->con_params.ch_map), &(param->ch_map), sizeof(struct le_ch_map));
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
        // procedure with instant finished
        SETB(llc_env_ptr->link_info, LLC_INFO_INSTANT_PROC, false);
        // unregister procedure
        llc_proc_unreg(link_id, LLC_PROC_REMOTE);

#if (BT_53) && (BLE_CENTRAL)
        // start procedure to resume subrating if needed
        llc_subrate_restore(link_id, LLC_SUBRATE_FULL_RESTORE);
#endif // (BT_53) && (BLE_CENTRAL)
    }
}

/**
 ****************************************************************************************
 * @brief Sends the channel map request pdu
 *
 * @param[in] link_id Link Identifier
 * @param[in] instant Instant when channel map have to be applied
 * @param[in] ch_map  Channel Map to apply
 ****************************************************************************************
 */
__STATIC void llc_llcp_ch_map_update_ind_pdu_send(uint8_t link_id, uint16_t instant, struct le_ch_map *ch_map)
{
    struct ll_channel_map_ind pdu;

    pdu.op_code = LL_CHANNEL_MAP_IND_OPCODE;
    pdu.instant = instant;
    memcpy(&(pdu.ch_map), ch_map, sizeof(struct le_ch_map));

    llc_llcp_send(link_id, (union llcp_pdu *) &pdu, NULL);
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
__STATIC void llc_ch_map_up_proc_err_cb(uint8_t link_id, uint8_t error_type, void *param)
{
    switch (error_type)
    {
    // link disconnection occurs
    case LLC_ERR_DISCONNECT:
    {
        uint8_t reason = *((uint8_t *) param);
        if (llc_proc_id_get(link_id, LLC_PROC_LOCAL) == LLC_PROC_CH_MAP_UPDATE)
        {
            llc_loc_ch_map_proc_continue(link_id, LLC_LOC_CH_MAP_ERROR, reason);
        }
        else if (llc_proc_id_get(link_id, LLC_PROC_REMOTE) == LLC_PROC_CH_MAP_UPDATE)
        {
            llc_rem_ch_map_proc_continue(link_id, LLC_REM_CH_MAP_ERROR, reason);
        }
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

void llc_ch_map_create(uint8_t link_id, struct le_ch_map *ch_map)
{
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];
    uint8_t nbchgood;

#if (BT_53)
    // Take peer's channel classification into account
    for (int i = 0 ; i < BLE_DATA_CHANNEL_NB ; i++)
    {
        uint8_t byte_idx = i >> 3;
        uint8_t bit_pos = i & 0x7;
        uint8_t ch_class_byte_idx = i >> 2;
        uint8_t ch_class_bit_pos = (i & 0x3) << 1;

        // If the channel has been reported as bad
        if (((llc_env_ptr->ch_class.ch_class_map.map[ch_class_byte_idx] >> ch_class_bit_pos) & 0x3) == LE_CH_CLASS_STATUS_BAD)
        {
            // Remove it from the channel map
            ch_map->map[byte_idx] &= ~(1 << bit_pos);
        }
    }
#endif // (BT_53)

    // Count number of good channels
    nbchgood = ble_util_nb_good_channels(ch_map);

    // Check if the map has a sufficient number of used channels
    if (nbchgood < llc_env_ptr->con_params.min_used_ch)
    {
        rwip_ch_map_ble_fill((llc_env_ptr->con_params.min_used_ch - nbchgood), llm_host_ch_class_get(), ch_map);
    }
}

void llc_ch_map_update(uint8_t link_id, struct le_ch_map *llm_ch_map)
{
    // Check if state in disconnected state
    if (!llc_is_disconnecting(link_id))
    {
        struct llc_env_tag *llc_env_ptr = llc_env[link_id];

        // check that a local channel map update is not ongoing
        if (!GETB(llc_env_ptr->link_info, LLC_INFO_LOC_CH_MAP_UPD))
        {
            struct le_ch_map new_ch_map;

            // Copy LLM channel map to local channel map
            memcpy(&new_ch_map, llm_ch_map, sizeof(struct le_ch_map));

            // Create the new channel map
            llc_ch_map_create(link_id, &new_ch_map);

            // Check if the channel map has changed
            if (memcmp(&(llc_env_ptr->con_params.ch_map), &new_ch_map, sizeof(struct le_ch_map)) != 0)
            {
                ke_task_id_t llc_id = KE_BUILD_ID(TASK_LLC, link_id);
                // prepare local channel map update procedure
                struct llc_op_ch_map_upd_ind *ch_map_upd = KE_MSG_ALLOC(LLC_OP_CH_MAP_UPD_IND, llc_id, llc_id, llc_op_ch_map_upd_ind);

                llc_proc_init(&ch_map_upd->proc, LLC_PROC_CH_MAP_UPDATE, llc_ch_map_up_proc_err_cb);
                llc_proc_state_set(&ch_map_upd->proc, link_id, LLC_LOC_CH_MAP_UP_START);

                // Store the new channel map into local procedure operation data
                memcpy(&(ch_map_upd->ch_map), &new_ch_map, sizeof(struct le_ch_map));

                ke_msg_send(ch_map_upd);
            }
        }
    } // else nothing to do
}

/*
 ****************************************************************************************
 * LLCP Handlers
 ****************************************************************************************
 */
/**
 ****************************************************************************************
 *  @brief Handles the reception of a LLCP channel map update request
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
uint8_t ll_channel_map_ind_handler(uint8_t link_id, struct ll_channel_map_ind *pdu, uint16_t event_cnt)
{
    //Get environment pointer
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];
    uint8_t status = CO_ERROR_NO_ERROR;

    // This LLCP request shall be issued by the master only
    if (GETB(llc_env_ptr->link_info, LLC_INFO_MASTER_ROLE))
    {
        // unexpected message, reject it.
        status = CO_ERROR_LMP_PDU_NOT_ALLOWED;
    }
    // check that no remote procedure is on-going
    else if (llc_proc_id_get(link_id, LLC_PROC_REMOTE) != LLC_PROC_NONE)
    {
        // unexpected message, reject it.
        status = CO_ERROR_LMP_PDU_NOT_ALLOWED;
    }
    else if ((ble_util_nb_good_channels(&pdu->ch_map) < DATA_CHANNEL_USED_NB_MIN))
    {
        status = CO_ERROR_INVALID_LMP_PARAM;
    }
    else
    {
        ke_task_id_t llc_id = KE_BUILD_ID(TASK_LLC, link_id);
        // prepare local channel map update procedure
        struct llc_op_ch_map_upd_ind *ch_map_upd = KE_MSG_ALLOC(LLC_OP_CH_MAP_UPD_IND, llc_id, llc_id, llc_op_ch_map_upd_ind);
        llc_proc_init(&ch_map_upd->proc, LLC_PROC_CH_MAP_UPDATE, llc_ch_map_up_proc_err_cb);
        llc_proc_state_set(&ch_map_upd->proc, link_id, LLC_REM_CH_MAP_UP_START);
        memcpy(&(ch_map_upd->ch_map), &(pdu->ch_map), sizeof(struct le_ch_map));
        ch_map_upd->instant = pdu->instant;

        // store local procedure
        llc_proc_reg(link_id, LLC_PROC_REMOTE, &(ch_map_upd->proc));

        // start procedure execution
        llc_rem_ch_map_proc_continue(link_id, LLC_REM_CH_MAP_UP_START, CO_ERROR_NO_ERROR);
    }

    return (status);
}

/**
 ****************************************************************************************
 *  @brief Handles the reception of an LLCP min used channels indication
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
uint8_t ll_min_used_channels_ind_handler(uint8_t link_id, struct ll_min_used_channels_ind *pdu, uint16_t event_cnt)
{
    //Get environment pointer
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];
    uint8_t status = CO_ERROR_NO_ERROR;

    // This LLCP shall be issued by the slave only
    if (!GETB(llc_env_ptr->link_info, LLC_INFO_MASTER_ROLE))
    {
        // unexpected message, reject it.
        status = CO_ERROR_LMP_PDU_NOT_ALLOWED;
    }
    // Check parameters
    else if (!(pdu->phys & PHY_ALL) || (pdu->min_used_ch < DATA_CHANNEL_USED_NB_MIN) || (pdu->min_used_ch > BLE_DATA_CHANNEL_NB))
    {
        status = CO_ERROR_PARAM_OUT_OF_MAND_RANGE;
    }
    else
    {
        uint8_t nbchgood = ble_util_nb_good_channels(&llc_env_ptr->con_params.ch_map);

        llc_env_ptr->con_params.min_used_ch_phys = pdu->phys & PHY_ALL;
        llc_env_ptr->con_params.min_used_ch = pdu->min_used_ch;

        if (nbchgood < llc_env_ptr->con_params.min_used_ch)
        {
            ke_task_id_t llc_id = KE_BUILD_ID(TASK_LLC, link_id);
            // prepare local channel map update procedure
            struct llc_op_ch_map_upd_ind *ch_map_upd = KE_MSG_ALLOC(LLC_OP_CH_MAP_UPD_IND, llc_id, llc_id, llc_op_ch_map_upd_ind);
            llc_proc_init(&ch_map_upd->proc, LLC_PROC_CH_MAP_UPDATE, llc_ch_map_up_proc_err_cb);
            llc_proc_state_set(&ch_map_upd->proc, link_id, LLC_LOC_CH_MAP_UP_START);
            // Channel map is not filled in the message, it is read from LLM when starting the procedure
            // Compute channel map
            llm_ch_map_compute(&(ch_map_upd->ch_map));

            // Create the new channel map
            llc_ch_map_create(link_id, &(ch_map_upd->ch_map));

            ke_msg_send(ch_map_upd);
        }
    }

    return (status);
}

/*
 ****************************************************************************************
 * HCI Handlers
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @brief Handles request from host to get current channel map.
 ****************************************************************************************
 */
int hci_le_rd_ch_map_cmd_handler(uint8_t link_id, struct hci_basic_conhdl_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;
    struct hci_le_rd_ch_map_cmd_cmp_evt *event;

    // allocate the Command Complete event message
    event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, param->conhdl, HCI_LE_RD_CH_MAP_CMD_OPCODE, hci_le_rd_ch_map_cmd_cmp_evt);

    // check if state is Free or in disconnected state
    if (!llc_is_disconnecting(link_id))
    {
        // Fill the device channel map
        struct llc_env_tag *llc_env_ptr = llc_env[link_id];
        memcpy(&(event->ch_map), &(llc_env_ptr->con_params.ch_map), sizeof(struct le_ch_map));
        status = CO_ERROR_NO_ERROR;
    } // else nothing to do

    event->status = status;
    event->conhdl = param->conhdl;

    // sends the message
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
 * @brief Handles the Channel Map Update Local procedure indication message.
 ****************************************************************************************
 */
int llc_op_ch_map_upd_ind_handler(ke_msg_id_t const msgid, struct llc_op_ch_map_upd_ind *param,
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
    // check if another local procedure is on-going
    else if ((llc_proc_id_get(link_id, LLC_PROC_LOCAL) != LLC_PROC_NONE)
             // or procedure with instant on-going
             || (GETB(llc_env_ptr->link_info, LLC_INFO_INSTANT_PROC)))
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

        // store local procedure
        llc_proc_reg(link_id, LLC_PROC_LOCAL, &(param->proc));
        // execute local procedure
        llc_loc_ch_map_proc_continue(link_id, LLC_LOC_CH_MAP_UP_START, CO_ERROR_NO_ERROR);
    }
    return (msg_status);
}

/**
 ****************************************************************************************
 * @brief Handles the channel map update local procedure indication message.
 ****************************************************************************************
 */
int lld_ch_map_upd_cfm_handler(ke_msg_id_t const msgid, void *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t link_id = KE_IDX_GET(dest_id);

    // channel map has been applied
    if (llc_proc_id_get(link_id, LLC_PROC_LOCAL) == LLC_PROC_CH_MAP_UPDATE)
    {
        llc_loc_ch_map_proc_continue(link_id, LLC_LOC_CH_MAP_WAIT_INSTANT, CO_ERROR_NO_ERROR);
    }
    else if (llc_proc_id_get(link_id, LLC_PROC_REMOTE) == LLC_PROC_CH_MAP_UPDATE)
    {
        llc_rem_ch_map_proc_continue(link_id, LLC_REM_CH_MAP_WAIT_INSTANT, CO_ERROR_NO_ERROR);
    }

    return (KE_MSG_CONSUMED);
}



#endif // (BLE_CENTRAL || BLE_PERIPHERAL)
/// @} LLC_CH_MAP_UPD
