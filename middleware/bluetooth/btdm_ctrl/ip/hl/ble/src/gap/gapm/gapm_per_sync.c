/**
 ****************************************************************************************
 *
 * @file gapm_per_sync.c
 *
 * @brief Generic Access Profile Manager - Periodic synchronization manager module.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup GAPM_PER_SYNC Generic Access Profile Manager - Periodic synchronization manager module.
 * @ingroup GAPM
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (BLE_OBSERVER)

#include <string.h>

#include "gap.h"
#include "gapm_task.h"
#include "gapm_int.h"
#include "ke_mem.h"
#include "hci.h"
#include "co_utils.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/// First invalid sync handle value
#define GAPM_PER_SYNC_INVALID_SYNC_HANDLE   (0x0EFF + 1)

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/// Bit field bit positions for periodic synchronization activity info parameter
enum gapm_per_sync_info_pos
{
    /// Synchronization has been established for this activity
    GAPM_PER_SYNC_INFO_SYNC_POS               = 0,
    /// Indicate that application accepts uploading of truncated reports
    GAPM_PER_SYNC_INFO_ACCEPT_TRUNCATED_POS,
};

/// Bit field bit values for periodic synchronization activity info parameter
enum gapm_per_sync_info_bit
{
    /// Synchronization has been established for this activity
    GAPM_PER_SYNC_INFO_SYNC_BIT               = CO_BIT(GAPM_PER_SYNC_INFO_SYNC_POS),
    /// Indicate that application accepts uploading of truncated reports
    GAPM_PER_SYNC_INFO_ACCEPT_TRUNCATED_BIT   = CO_BIT(GAPM_PER_SYNC_INFO_ACCEPT_TRUNCATED_POS),
};

/*
 * MACROS
 ****************************************************************************************
 */

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Send a LE Periodic Advertising Create Sync command over HCI.
 *
 * @param[in] p_actv_per_sync    Pointer to the activity structure
 * @param[in] p_per_sync_param   Pointer to the synchronization parameters
 ****************************************************************************************
 */
__STATIC void gapm_per_sync_send_hci_le_per_adv_create_sync_cmd(struct gapm_actv_per_sync_tag *p_actv_per_sync,
        struct gapm_per_sync_param *p_per_sync_param)
{
    // Allocate HCI command message
    struct hci_le_per_adv_create_sync_cmd *p_cmd = KE_MSG_ALLOC(HCI_COMMAND,
            0, HCI_LE_PER_ADV_CREATE_SYNC_CMD_OPCODE,
            hci_le_per_adv_create_sync_cmd);

    // Fill the command
    if (p_actv_per_sync->common.subtype == GAPM_PER_SYNC_TYPE_GENERAL)
    {
        p_cmd->options |= 1 << PER_SYNC_FILT_USE_PAL_POS;

        // Fill advertiser identity
        memcpy(&p_cmd->adv_addr.addr[0], &p_per_sync_param->adv_addr.addr.addr.addr[0], GAP_BD_ADDR_LEN);
        p_cmd->adv_addr_type = p_per_sync_param->adv_addr.addr.addr_type;
        p_cmd->adv_sid       = p_per_sync_param->adv_addr.adv_sid;
    }
    else
    {
        p_cmd->options &= ~PER_SYNC_FILT_USE_PAL_BIT;
    }

    //TODO: Not implement options for initialize enabled.
    p_cmd->skip    = p_per_sync_param->skip;
    p_cmd->sync_to = p_per_sync_param->sync_to;

    // Send the command
    hci_send_2_controller(p_cmd);

    // Keep in mind which command status event has to be received
    p_actv_per_sync->common.next_exp_opcode = HCI_LE_PER_ADV_CREATE_SYNC_CMD_OPCODE;
}

/**
 ****************************************************************************************
 * @brief Send a LE Periodic Advertising Create Sync Cancel command over HCI.
 *
 * @param[in] p_actv_per_sync    Pointer to the activity structure
 ****************************************************************************************
 */
__STATIC void gapm_per_sync_send_hci_le_per_adv_create_sync_cancel_cmd(struct gapm_actv_per_sync_tag *p_actv_per_sync)
{
    hci_basic_cmd_send_2_controller(HCI_LE_PER_ADV_CREATE_SYNC_CANCEL_CMD_OPCODE);

    // Keep in mind which command complete event has to be received
    p_actv_per_sync->common.next_exp_opcode = HCI_LE_PER_ADV_CREATE_SYNC_CANCEL_CMD_OPCODE;
}

/**
 ****************************************************************************************
 * @brief Send a LE Periodic Advertising Terminate Sync command over HCI.
 *
 * @param[in] p_actv_per_sync    Pointer to the activity structure
 ****************************************************************************************
 */
__STATIC void gapm_per_sync_send_hci_le_per_adv_term_sync_cmd(struct gapm_actv_per_sync_tag *p_actv_per_sync)
{
    // Allocate HCI command message
    struct hci_le_per_adv_term_sync_cmd *p_cmd = KE_MSG_ALLOC(HCI_COMMAND,
            0, HCI_LE_PER_ADV_TERM_SYNC_CMD_OPCODE,
            hci_le_per_adv_term_sync_cmd);

    // Fill the command
    p_cmd->sync_handle = p_actv_per_sync->sync_hdl;

    // Send the command
    hci_send_2_controller(p_cmd);

    // Keep in mind which command complete event has to be received
    p_actv_per_sync->common.next_exp_opcode = HCI_LE_PER_ADV_TERM_SYNC_CMD_OPCODE;
}

/**
 ****************************************************************************************
 * @brief Concatenate all received fragment of a periodic advertising report.
 *
 * @param[in] p_actv_per_sync   Pointer to the periodic synchronization report
 * @param[in] p_data            Pointer to the data buffer in which the fragments have
 *                              to be copied.
 ****************************************************************************************
 */
__STATIC void gapm_per_sync_copy_fragments(struct gapm_actv_per_sync_tag *p_actv_per_sync,
        uint8_t *p_data)
{
    // Loop over all stored stored report fragments
    while (co_list_pick(&p_actv_per_sync->report_list))
    {
        struct ke_msg *p_msg = (struct ke_msg *)co_list_pop_front(&p_actv_per_sync->report_list);

        if (p_data)
        {
            // Get message parameters
            struct hci_le_per_adv_report_evt *p_event = (struct hci_le_per_adv_report_evt *)ke_msg2param(p_msg);

            // Copy the data
            memcpy(p_data, &p_event->data[0], p_event->data_len);

            // Increase cursor
            p_data += p_event->data_len;
        }

        // Free the message
        ke_msg_free(p_msg);
    }

    // Reset stored data length
    p_actv_per_sync->length = 0;
}

/**
 ****************************************************************************************
 * @brief Send a GAPM_SYNC_ESTABLISHED_IND message to the application
 *
 * @param[in] p_actv_per_sync    Pointer to the activity structure
 * @param[in] p_event            Content of the LE Periodic Advertising Sync Established event
 ****************************************************************************************
 */
__STATIC void gapm_per_sync_send_sync_established_ind(struct gapm_actv_per_sync_tag *p_actv_per_sync,
        struct hci_le_per_adv_sync_est_evt const *p_event)
{
    // Allocate a GAPM_SYNC_ESTABLISHED_IND message
    struct gapm_sync_established_ind *p_ind = KE_MSG_ALLOC(GAPM_SYNC_ESTABLISHED_IND,
            p_actv_per_sync->common.requester, TASK_GAPM,
            gapm_sync_established_ind);

    // Fill the message
    p_ind->actv_idx       = p_actv_per_sync->common.idx;
    p_ind->phy            = p_event->phy;
    p_ind->intv           = p_event->interval;
    p_ind->adv_sid        = p_event->adv_sid;
    p_ind->clk_acc        = p_event->adv_ca;
    p_ind->addr.addr_type = p_event->adv_addr_type;
    memcpy(&p_ind->addr.addr.addr[0], &p_event->adv_addr.addr[0], GAP_BD_ADDR_LEN);

    // Send the message
    ke_msg_send(p_ind);
}

/**
 ****************************************************************************************
 * @brief Send a GAPM_EXT_ADV_REPORT_IND message containing a received periodic advertising
 * report to the application.
 *
 * @param[in] p_actv_per_sync   Pointer to the periodic synchronization activity
 * @param[in] p_event           Pointer to the last received advertising report fragment
 * @param[in] complete          Indicate if periodic advertising report is complete
 ****************************************************************************************
 */
__STATIC void gapm_per_sync_send_adv_report_ind(struct gapm_actv_per_sync_tag *p_actv_per_sync,
        struct hci_le_per_adv_report_evt const *p_event,
        bool complete)
{
    // Compute report length
    uint16_t report_length = p_actv_per_sync->length + p_event->data_len;
    // Allocate a GAPM_EXT_ADV_REPORT_IND message
    struct gapm_ext_adv_report_ind *p_ind = KE_MSG_ALLOC_DYN(GAPM_EXT_ADV_REPORT_IND,
                                            p_actv_per_sync->common.requester, TASK_GAPM,
                                            gapm_ext_adv_report_ind,
                                            report_length);

    // Fill the message
    p_ind->actv_idx = p_actv_per_sync->common.idx;
    p_ind->length   = report_length;
    p_ind->rssi     = p_event->rssi;
    p_ind->tx_pwr   = p_event->tx_power;
    p_ind->info     = GAPM_REPORT_TYPE_PER_ADV;

    if (complete)
    {
        p_ind->info |= GAPM_REPORT_INFO_COMPLETE_BIT;
    }

    // Copy the last received fragment
    memcpy(&p_ind->data[p_actv_per_sync->length], &p_event->data[0], p_event->data_len);

    // Copy the stored fragment
    gapm_per_sync_copy_fragments(p_actv_per_sync, &p_ind->data[0]);

    // Send the message
    ke_msg_send(p_ind);
}

/**
 ****************************************************************************************
 * @brief Following HCI command complete event message can be received from controller
 * while activity state is STARTING:
 *    - LE Periodic Advertising Create Sync command (HCI_LE_PER_ADV_CREATE_SYNC_CMD_OPCODE)
 *
 * @param[in] p_actv_per_sync      Pointer to the activity structure for which the event has
 * been received.
 * @param[in] opcode               Operation code of received message.
 * @param[in] p_event              Pointer to the HCI command complete event message content.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC void hci_le_cmd_cmp_evt_starting_handler(struct gapm_actv_per_sync_tag *p_actv_per_sync,
        uint16_t opcode,
        struct hci_basic_cmd_cmp_evt const *p_event)
{
    // Operation is over
    gapm_actv_started((struct gapm_actv_tag *)p_actv_per_sync, RW_ERR_HCI_TO_HL(p_event->status));
}

/**
 ****************************************************************************************
 * @brief Following HCI command complete event message can be received from controller
 * while activity state is STOPPING:
 *    - LE Periodic Advertising Create Sync Cancel command (HCI_LE_PER_ADV_CREATE_SYNC_CANCEL_CMD_OPCODE)
 *    - LE Periodic Advertising Terminate Sync command (HCI_LE_PER_ADV_TERM_SYNC_CMD_OPCODE)
 *
 * @param[in] p_actv_per_sync    Pointer to the activity structure for which the event has
 * been received.
 * @param[in] opcode             Operation code of received message.
 * @param[in] p_event            Pointer to the HCI command complete event message content.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC void hci_le_cmd_cmp_evt_stopping_handler(struct gapm_actv_per_sync_tag *p_actv_per_sync,
        uint16_t opcode,
        struct hci_basic_cmd_cmp_evt const *p_event)
{
    // Operation is over
    gapm_actv_stopped((struct gapm_actv_tag *)p_actv_per_sync, GAP_ERR_NO_ERROR);
}

/**
 ****************************************************************************************
 * @brief Store a received periodic advertising report fragment
 *
 * @param p_actv_per_sync       Pointer to a periodic synchronization activity
 * @param p_event               Pointer to the received advertising report event.
 ****************************************************************************************
 */
__STATIC void gapm_per_sync_store_fragment(struct gapm_actv_per_sync_tag *p_actv_per_sync,
        struct hci_le_per_adv_report_evt const *p_event)
{
    // Increase stored data length
    p_actv_per_sync->length += p_event->data_len;

    // Store the complete message
    co_list_push_back(&p_actv_per_sync->report_list, &(ke_param2msg(p_event)->hdr));
}

/**
 ****************************************************************************************
 * @brief Look for a periodic synchronization activity.
 *
 * @param[in] sync_handle   Synchronization handle if looking for an activity on which
 *                          synchronization has been established,
 *                          GAPM_PER_SYNC_INVALID_SYNC_HANDLE if looking for an activity
 *                          not yet synchronized.
 ****************************************************************************************
 */
__STATIC struct gapm_actv_per_sync_tag *gapm_per_sync_get_activity(uint16_t sync_handle)
{
    // Found activity
    struct gapm_actv_per_sync_tag *p_actv_per_sync = NULL;
    // Counter
    uint8_t i;

    // Loop over the activities
    for (i = 0; i < GAPM_ACTV_NB; i++)
    {
        // Get activity
        struct gapm_actv_per_sync_tag *p_actv_loop = (struct gapm_actv_per_sync_tag *)gapm_env.actvs[i];

        // Check that activity exists, is started and is a periodic synchronization activity
        if (p_actv_loop
                && (p_actv_loop->common.type == GAPM_ACTV_TYPE_PER_SYNC)
                && (p_actv_loop->common.state == GAPM_ACTV_STARTED))
        {
            if (sync_handle < GAPM_PER_SYNC_INVALID_SYNC_HANDLE)
            {
                // Look for a synchronized activity with the same sync handle
                if (!GETB(p_actv_loop->common.info, GAPM_PER_SYNC_INFO_SYNC)
                        || (p_actv_loop->sync_hdl != sync_handle))
                {
                    // Check next activity
                    continue;
                }
            }
            else
            {
                // Look for a not yet synchronized activity
                if (GETB(p_actv_loop->common.info, GAPM_PER_SYNC_INFO_SYNC))
                {
                    // Check next activity
                    continue;
                }
            }

            // If we are here, activity is the one we are looking for
            p_actv_per_sync = p_actv_loop;
            break;
        }
    }

    return (p_actv_per_sync);
}

/**
 ****************************************************************************************
 * @brief Start a periodic synchronization activity.
 *
 * @param[in] p_actv        Pointer to the activity structure
 * @param[in] p_param       GAPM_ACTIVITY_START_CMD message parameters
 ****************************************************************************************
 */
__STATIC uint8_t gapm_per_sync_start(struct gapm_actv_tag *p_actv, struct gapm_activity_start_cmd *p_param)
{
    // Cast the activity structure into a periodic synchronization activity structure
    struct gapm_actv_per_sync_tag *p_actv_per_sync = (struct gapm_actv_per_sync_tag *)p_actv;
    // Periodic synchronization Parameters
    struct gapm_per_sync_param *p_per_sync_param = (struct gapm_per_sync_param *)&p_param->u_param.per_sync_param;
    // Check new provided initiating parameters
    uint8_t error = GAP_ERR_NO_ERROR;

    do
    {
        // Check periodic synchronization type
        if (p_per_sync_param->type > GAPM_PER_SYNC_TYPE_SELECTIVE)
        {
            error = GAP_ERR_INVALID_PARAM;
            break;
        }

        // Only one synchronization establishment can be started
        if (gapm_per_sync_get_activity(GAPM_PER_SYNC_INVALID_SYNC_HANDLE))
        {
            // A periodic synchronization activity in synchronizing state has been found
            error = GAP_ERR_COMMAND_DISALLOWED;
            break;
        }

        // Configure the activity
        p_actv_per_sync->common.subtype = p_per_sync_param->type;

        // Keep activity identifier in mind
        gapm_env.actv_idx = p_actv_per_sync->common.idx;

        // Send a LE Periodic Advertising Create Sync command to the controller
        gapm_per_sync_send_hci_le_per_adv_create_sync_cmd(p_actv_per_sync, p_per_sync_param);
    }
    while (0);

    return (error);
}

/**
 ****************************************************************************************
 * @brief Stop one periodic synchronization activity. Activity state must have been set
 * to STOPPING state before calling this function
 *
 * @param[in] p_actv    Pointer to the activity structure to be stopped
 ****************************************************************************************
 */
__STATIC void gapm_per_sync_stop(struct gapm_actv_tag *p_actv)
{
    // Cast the activity structure in order to retrieve periodic synchronization parameters
    struct gapm_actv_per_sync_tag *p_actv_per_sync = (struct gapm_actv_per_sync_tag *)p_actv;

    // Keep activity identifier in mind
    gapm_env.actv_idx = p_actv_per_sync->common.idx;

    // Command to be sent to the controller depends on current synchronization state
    if (GETB(p_actv_per_sync->common.info, GAPM_PER_SYNC_INFO_SYNC))
    {
        // Send a LE Periodic Advertising Terminate Sync command to the controller
        gapm_per_sync_send_hci_le_per_adv_term_sync_cmd(p_actv_per_sync);
    }
    else
    {
        // Send a LE Periodic Advertising Create Sync Cancel command to the controller
        gapm_per_sync_send_hci_le_per_adv_create_sync_cancel_cmd(p_actv_per_sync);
    }
}

/**
 ****************************************************************************************
 * @brief Delete provided periodic synchronization activity. It is considered here that
 * activity is in DELETING state.
 *
 * @param[in] p_actv    Pointer to the activity structure to be deleted.
 ****************************************************************************************
 */
__STATIC void gapm_per_sync_delete(struct gapm_actv_tag *p_actv)
{
    // Nothing special to be done
    gapm_actv_deleted(p_actv);
}

/*
 * EXTERNAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

uint8_t gapm_per_sync_create(uint8_t actv_idx, struct gapm_activity_create_cmd *p_param)
{
    // Check provided scanning parameters
    uint8_t error = GAP_ERR_NO_ERROR;
    // Allocate an activity structure
    struct gapm_actv_per_sync_tag *p_actv_per_sync
        = (struct gapm_actv_per_sync_tag *)gapm_actv_alloc(actv_idx, sizeof(struct gapm_actv_per_sync_tag));

    // Check if enough memory has been found
    if (!p_actv_per_sync)
    {
        error = GAP_ERR_INSUFF_RESOURCES;
    }
    else
    {
        // Fill the activity structure
        p_actv_per_sync->common.type = GAPM_ACTV_TYPE_PER_SYNC;
        p_actv_per_sync->common.requester = gapm_get_requester(GAPM_OP_AIR);

        // Set callback functions
        p_actv_per_sync->common.cb_actv_start  = gapm_per_sync_start;
        p_actv_per_sync->common.cb_actv_stop   = gapm_per_sync_stop;
        p_actv_per_sync->common.cb_actv_delete = gapm_per_sync_delete;

        gapm_actv_created((struct gapm_actv_tag *)p_actv_per_sync, GAP_ERR_NO_ERROR);
    }

    return (error);
}

void gapm_per_sync_clear_fragments(struct gapm_actv_per_sync_tag *p_actv_per_sync)
{
    // Loop over all stored stored report fragments
    while (co_list_pick(&p_actv_per_sync->report_list))
    {
        struct ke_msg *p_msg = (struct ke_msg *)co_list_pop_front(&p_actv_per_sync->report_list);

        // Free the message
        ke_msg_free(p_msg);
    }

    // Reset stored data length
    p_actv_per_sync->length = 0;
}

/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Common handler for following HCI command complete events or command status events:
 *    - LE Periodic Advertising Create Sync command (HCI_LE_PER_ADV_CREATE_SYNC_CMD_OPCODE)
 *    - LE Periodic Advertising Create Sync Cancel command (HCI_LE_PER_ADV_CREATE_SYNC_CANCEL_CMD_OPCODE)
 *    - LE Periodic Advertising Terminate Sync command (HCI_LE_PER_ADV_TERM_SYNC_CMD_OPCODE)
 *
 * @param[in] opcode    Operation code of received message
 * @param[in] param     Pointer to the parameters of the message.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int hci_le_cmd_cmp_evt_per_sync_handler(uint16_t opcode, struct hci_basic_cmd_cmp_evt const *p_event)
{
    // Periodic synchronization activity structure
    struct gapm_actv_per_sync_tag *p_actv_per_sync;

    do
    {
        // Find activity for which the command complete event has been received
        if (!gapm_actv_retrieve_cmd_cmp_evt((struct gapm_actv_tag **)&p_actv_per_sync, opcode))
        {
            break;
        }

        // Call the proper handler based on current activity state
        switch (p_actv_per_sync->common.state)
        {
        case GAPM_ACTV_STARTING:
        {
            hci_le_cmd_cmp_evt_starting_handler(p_actv_per_sync, opcode, p_event);
        }
        break;

        case GAPM_ACTV_STOPPING:
        {
            hci_le_cmd_cmp_evt_stopping_handler(p_actv_per_sync, opcode, p_event);
        }
        break;

        default:
        {
            // Nothing to do
        }
        break;
        }
    }
    while (0);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handle reception of HCI LE Periodic Advertising Sync Established event.
 *
 * @param[in] opcode      Operation code of received message
 * @param[in] p_event     Pointer to the parameters of the message.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int hci_le_per_adv_sync_est_evt_handler(uint16_t opcode, struct hci_le_per_adv_sync_est_evt const *p_event)
{
    // Get periodic synchronization activity for which synchronization establishment was in progress,
    // only one should exist
    struct gapm_actv_per_sync_tag *p_actv_per_sync = gapm_per_sync_get_activity(GAPM_PER_SYNC_INVALID_SYNC_HANDLE);

    if (p_actv_per_sync)
    {
        // Next step depends on the provided status
        if (p_event->status == CO_ERROR_NO_ERROR)
        {
            // Keep in mind the provided sync handle
            p_actv_per_sync->sync_hdl = p_event->sync_handle;

            // Keep in mind that synchronization has been established
            SETB(p_actv_per_sync->common.info, GAPM_PER_SYNC_INFO_SYNC, true);

            // Inform the application about the established synchronization
            gapm_per_sync_send_sync_established_ind(p_actv_per_sync, p_event);
        }
        else
        {
            // Consider that the activity can be stopped
            gapm_actv_stopped((struct gapm_actv_tag *)p_actv_per_sync, RW_ERR_HCI_TO_HL(p_event->status));
        }
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handle reception of HCI LE Periodic Advertising Report event.
 *
 * @param[in] opcode      Operation code of received message
 * @param[in] p_event     Pointer to the parameters of the message.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int hci_le_per_adv_report_evt_handler(uint16_t opcode, struct hci_le_per_adv_report_evt const *p_event)
{
    // Look for period synchronization activity with provided sync handle
    struct gapm_actv_per_sync_tag *p_actv_per_sync = gapm_per_sync_get_activity(p_event->sync_handle);
    // Message status
    int msg_status = KE_MSG_CONSUMED;

    if (p_actv_per_sync)
    {
        switch (p_event->status)
        {
        case PER_ADV_EVT_DATA_STATUS_COMPLETE:
        {
            // Send the full report to the application
            gapm_per_sync_send_adv_report_ind(p_actv_per_sync, p_event, true);
        }
        break;

        case ADV_EVT_DATA_STATUS_INCOMPLETE:
        {
            // Store the receive report fragment
            gapm_per_sync_store_fragment(p_actv_per_sync, p_event);

            // Do not free the message
            msg_status = KE_MSG_NO_FREE;
        }
        break;

        case ADV_EVT_DATA_STATUS_TRUNCATED:
        {
            // Check if application has accepted forwarding of truncated data.
            if (GETB(p_actv_per_sync->common.info, GAPM_PER_SYNC_INFO_ACCEPT_TRUNCATED))
            {
                // Send the truncated report to the application
                gapm_per_sync_send_adv_report_ind(p_actv_per_sync, p_event, false);
            }
            else
            {
                // Clear stored fragments
                gapm_per_sync_clear_fragments(p_actv_per_sync);
            }
        }
        break;

        default:
        {
            // Drop the message
        } break;
        }
    }

    return (msg_status);
}

/**
 ****************************************************************************************
 * @brief Handle reception of HCI LE Periodic Advertising Sync Lost event.
 *
 * @param[in] opcode      Operation code of received message
 * @param[in] p_event     Pointer to the parameters of the message.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int hci_le_per_adv_sync_lost_evt_handler(uint16_t opcode, struct hci_le_per_adv_sync_lost_evt const *p_event)
{
    // Get periodic synchronization activity with the provided sync handle
    struct gapm_actv_per_sync_tag *p_actv_per_sync = gapm_per_sync_get_activity(p_event->sync_handle);

    if (p_actv_per_sync)
    {
        // Keep in mind that the synchronization has been lost
        SETB(p_actv_per_sync->common.info, GAPM_PER_SYNC_INFO_SYNC, false);

        // Stop the activity and inform the application
        gapm_actv_stopped((struct gapm_actv_tag *)p_actv_per_sync, GAP_ERR_DISCONNECTED);
    }

    return (KE_MSG_CONSUMED);
}

#endif //(BLE_OBSERVER)

/// @} GAPM_PER_SYNC
