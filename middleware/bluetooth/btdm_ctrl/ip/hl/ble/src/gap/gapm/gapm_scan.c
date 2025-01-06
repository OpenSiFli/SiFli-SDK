/**
 ****************************************************************************************
 *
 * @file gapm_scan.c
 *
 * @brief Generic Access Profile Manager - Scanning manager module.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup GAPM_SCAN Generic Access Profile Manager - Scanning manager module.
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

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/// Bit field bit positions for scanning activity info parameter
enum gapm_scan_info_pos
{
    /// Indicate that application accepts uploading of truncated reports
    GAPM_SCAN_INFO_ACCEPT_TRUNCATED_POS = 0,
};


/// Bit field bit values for scanning activity info parameter
enum gapm_scan_info_bit
{
    /// Indicate that application accepts uploading of truncated reports
    GAPM_SCAN_INFO_ACCEPT_TRUNCATED_BIT   = CO_BIT(GAPM_SCAN_INFO_ACCEPT_TRUNCATED_POS),
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
 * @brief Send a LE Set Extended Scan Parameters command over HCI. The command
 * complete event is handled in hci_le_cmd_cmp_evt_scan_handler function.
 *
 * @param[in] p_actv_scan    Pointer to the activity structure
 ****************************************************************************************
 */
__STATIC void gapm_scan_send_hci_le_set_ext_scan_param_cmd(struct gapm_actv_scan_tag *p_actv_scan,
        struct gapm_scan_param *p_scan_param)
{
    // Index
    uint8_t index = 0;
    // Allocate HCI command message
    struct hci_le_set_ext_scan_param_cmd *p_cmd = KE_MSG_ALLOC(HCI_COMMAND,
            0, HCI_LE_SET_EXT_SCAN_PARAM_CMD_OPCODE,
            hci_le_set_ext_scan_param_cmd);

    // Fill the command parameters
    p_cmd->own_addr_type = gapm_actv_get_hci_own_addr_type(p_actv_scan->common.own_addr_type);
    p_cmd->scan_filt_policy = 0;

    // Check if white list must be used
    if ((p_actv_scan->common.subtype == GAPM_SCAN_TYPE_SEL_OBSERVER)
            || (p_actv_scan->common.subtype == GAPM_SCAN_TYPE_SEL_CONN_DISC))
    {
        p_cmd->scan_filt_policy |= SCAN_ALLOW_ADV_WLST;
    }

    if (p_actv_scan->common.own_addr_type == GAPM_GEN_RSLV_ADDR)
    {
        // Check if received directed advertising packets with a non resolved target address must be
        // sent to the host
        if (p_scan_param->prop & GAPM_SCAN_PROP_ACCEPT_RPA_BIT)
        {
            p_cmd->scan_filt_policy |= SCAN_ALLOW_ADV_ALL_AND_INIT_RPA;
        }
    }

    p_cmd->scan_phys = 0;

    // Check if scan has to be done on LE 1M PHY
    if (p_scan_param->prop & GAPM_SCAN_PROP_PHY_1M_BIT)
    {
        p_cmd->scan_phys |= PHY_1MBPS_BIT;

        p_cmd->phy[index].scan_intv   = p_scan_param->scan_param_1m.scan_intv;
        p_cmd->phy[index].scan_window = p_scan_param->scan_param_1m.scan_wd;
        p_cmd->phy[index].scan_type   = (p_scan_param->prop & GAPM_SCAN_PROP_ACTIVE_1M_BIT) ? SCAN_ACTIVE : SCAN_PASSIVE;

        // Increase index
        index++;
    }

    // Check if scan has to be done on LE 1M Coded
    if (p_scan_param->prop & GAPM_SCAN_PROP_PHY_CODED_BIT)
    {
        p_cmd->scan_phys |= PHY_CODED_BIT;

        p_cmd->phy[index].scan_intv   = p_scan_param->scan_param_coded.scan_intv;
        p_cmd->phy[index].scan_window = p_scan_param->scan_param_coded.scan_wd;
        p_cmd->phy[index].scan_type   = (p_scan_param->prop & GAPM_SCAN_PROP_ACTIVE_CODED_BIT) ? SCAN_ACTIVE : SCAN_PASSIVE;
    }

    // Send the command
    hci_send_2_controller(p_cmd);

    // Keep in mind which command complete event has to be received
    p_actv_scan->common.next_exp_opcode = HCI_LE_SET_EXT_SCAN_PARAM_CMD_OPCODE;
}

/**
 ****************************************************************************************
 * @brief Send a LE Set Extended Scan Enable command over HCI. The command
 * complete event is handled in hci_le_cmd_cmp_evt_scan_handler function.
 *
 * @param[in] p_actv_scan    Pointer to the activity structure
 ****************************************************************************************
 */
__STATIC void gapm_scan_send_hci_le_set_ext_scan_en_cmd(struct gapm_actv_scan_tag *p_actv_scan,
        bool enable)
{
    // Retrieve GAPM_ACTIVITY_START_CMD message
    struct gapm_activity_start_cmd *p_param = (struct gapm_activity_start_cmd *)gapm_get_operation_ptr(GAPM_OP_AIR);
    // Get scan parameters
    struct gapm_scan_param *p_scan_param = &p_param->u_param.scan_param;
    // Allocate HCI command message
    struct hci_le_set_ext_scan_en_cmd *p_cmd = KE_MSG_ALLOC(HCI_COMMAND,
            0, HCI_LE_SET_EXT_SCAN_EN_CMD_OPCODE,
            hci_le_set_ext_scan_en_cmd);

    // Fill command parameters
    p_cmd->scan_en       = (uint8_t)enable;
    p_cmd->filter_duplic = p_scan_param->dup_filt_pol;

    if (p_actv_scan->common.subtype == GAPM_SCAN_TYPE_LIM_DISC)
    {
        p_cmd->duration = GAP_TMR_LIM_DISC_SCAN;
        p_cmd->period   = 0;
    }
    else if (p_actv_scan->common.subtype == GAPM_SCAN_TYPE_GEN_DISC)
    {
        p_cmd->duration = GAP_TMR_GEN_DISC_SCAN;
        p_cmd->period   = 0;
    }
    else
    {
        p_cmd->duration = p_scan_param->duration;
        p_cmd->period   = p_scan_param->period;
    }

    // Send the command
    hci_send_2_controller(p_cmd);

    // Keep in mind which command complete event has to be received
    p_actv_scan->common.next_exp_opcode = HCI_LE_SET_EXT_SCAN_EN_CMD_OPCODE;
}

/**
 ****************************************************************************************
 * @brief Following HCI command complete event message can be received from controller
 * while activity state is STARTING:
 *    - LE Set Extended Scan Parameters command (HCI_LE_SET_EXT_SCAN_PARAM_CMD_OPCODE)
 *    - LE Set Extended Scan Enable command (HCI_LE_SET_EXT_SCAN_EN_CMD_OPCODE)
 *
 * @param[in|out] p_actv_scan    Pointer to the activity structure for which the event has
 * been received.
 * @param[in] opcode             Operation code of received message.
 * @param[in] p_event            Pointer to the HCI command complete event message content.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC void hci_le_cmd_cmp_evt_starting_handler(struct gapm_actv_scan_tag *p_actv_scan,
        uint16_t opcode,
        struct hci_basic_cmd_cmp_evt const *p_event)
{
    // Map the controller error status on a GAP error status
    uint8_t status = RW_ERR_HCI_TO_HL(p_event->status);

    if (status == GAP_ERR_NO_ERROR)
    {
        if (opcode == HCI_LE_SET_EXT_SCAN_PARAM_CMD_OPCODE)
        {
            // Send a LE Set Extended Scan Enable command to the controller
            gapm_scan_send_hci_le_set_ext_scan_en_cmd(p_actv_scan, true);
        }
        else if (opcode == HCI_LE_SET_EXT_SCAN_EN_CMD_OPCODE)
        {
            // Operation is over
            gapm_actv_started((struct gapm_actv_tag *)p_actv_scan, GAP_ERR_NO_ERROR);
        }
    }
    else
    {
        // Scan has not been started properly
        gapm_env.scan_actv_idx = GAPM_ACTV_INVALID_IDX;

        // Operation is over
        gapm_actv_started((struct gapm_actv_tag *)p_actv_scan, status);
    }
}

/**
 ****************************************************************************************
 * @brief Following HCI command complete event message can be received from controller
 * while activity state is STOPPING:
 *    - LE Set Extended Advertising Enable command (HCI_LE_SET_EXT_ADV_EN_CMD_OPCODE)
 *        -> For all kind of advertising
 *    - LE Set Periodic Advertising Enable command (HCI_LE_SET_PER_ADV_EN_CMD_OPCODE)
 *        -> For periodic advertising only
 *
 * @param[in|out] p_actv_scan    Pointer to the activity structure for which the event has
 * been received.
 * @param[in] opcode             Operation code of received message.
 * @param[in] p_event            Pointer to the HCI command complete event message content.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC void hci_le_cmd_cmp_evt_stopping_handler(struct gapm_actv_scan_tag *p_actv_scan,
        uint16_t opcode,
        struct hci_basic_cmd_cmp_evt const *p_event)
{
    gapm_env.scan_actv_idx = GAPM_ACTV_INVALID_IDX;

    // Operation is over
    gapm_actv_stopped((struct gapm_actv_tag *)p_actv_scan, GAP_ERR_NO_ERROR);
}

/**
 ****************************************************************************************
 * @brief Look for AD Type flag value in a received advertising report.
 *
 * @param[in] p_data    Pointer to the received advertising report
 * @param[in] length    Length of the received advertising report
 *
 * @return 0 if AD Type flag has not been found else AD Type flag value
 ****************************************************************************************
 */
__STATIC uint8_t gapm_scan_get_ad_type_flag(uint8_t *p_data, uint8_t length)
{
    uint8_t cursor = 0;
    uint8_t ad_flag = 0;

    // Parse advertising data and look for ad_type
    while (cursor < length)
    {
        // Check if it's AD Type flag data
        if (p_data[cursor + 1] == GAP_AD_TYPE_FLAGS)
        {
            // Keep flag value
            ad_flag = p_data[cursor + 2];
            break;
        }

        // Go to next advertising info
        cursor += (p_data[cursor] + 1);
    }

    return (ad_flag);
}

/**
 ****************************************************************************************
 * @brief Add device to list of filtered devices
 *
 * @param[in] p_addr     Device address
 * @param[in] addr_type  Device address type
 ****************************************************************************************
 */
__STATIC void gapm_scan_add_to_filter(struct gapm_actv_scan_tag *p_scan_actv,
                                      bd_addr_t *p_addr, uint8_t addr_type)
{
    bd_addr_t default_addr = {{0, 0, 0, 0, 0, 0}};
    uint8_t cursor = 0;

    // Allocate scan filtered device list if needed.
    if (p_scan_actv->p_scan_filter == NULL)
    {
        p_scan_actv->p_scan_filter =
            (struct gap_bdaddr *)ke_malloc(sizeof(struct gap_bdaddr) * GAPM_SCAN_FILTER_SIZE, KE_MEM_KE_MSG);

        memset(p_scan_actv->p_scan_filter, 0, sizeof(struct gap_bdaddr) * GAPM_SCAN_FILTER_SIZE);
    }

    // Find first available space in array
    while ((memcmp(&(p_scan_actv->p_scan_filter[cursor].addr), &default_addr, sizeof(bd_addr_t)) != 0)
            && (cursor < GAPM_SCAN_FILTER_SIZE))
    {
        cursor++;
    }

    // Copy provided device address in array
    if (cursor < GAPM_SCAN_FILTER_SIZE)
    {
        memcpy(&(p_scan_actv->p_scan_filter[cursor].addr), p_addr, GAP_BD_ADDR_LEN);
        p_scan_actv->p_scan_filter[cursor].addr_type = addr_type;
    }
    // else don't put device into filter.
}

/**
 ****************************************************************************************
 * @brief Check if device is filtered or not when a scan response data is received
 *
 * If device is not filtered (present in filter list), in that case function returns false
 * and device is removed from filtered devices.
 *
 * @param[in] p_addr     Device address
 * @param[in] addr_type  Device address type
 *
 * @return true if device filtered, else false.
 ****************************************************************************************
 */
__STATIC bool gapm_scan_accept_scan_rsp(struct gapm_actv_scan_tag *p_scan_actv,
                                        bd_addr_t *p_addr, uint8_t addr_type)
{
    bool ret = false;
    uint8_t cursor = 0;

    // Check that filter array exists
    if (p_scan_actv->p_scan_filter != NULL)
    {
        // Find provided address in filter
        while (((memcmp(&(p_scan_actv->p_scan_filter[cursor].addr), p_addr, sizeof(bd_addr_t)) != 0)
                || (p_scan_actv->p_scan_filter[cursor].addr_type != addr_type))
                && (cursor <  GAPM_SCAN_FILTER_SIZE))
        {
            cursor++;
        }

        // Copy provided device address in array
        if (cursor < GAPM_SCAN_FILTER_SIZE)
        {
            ret = true;

            // Remove device from filter.
            memset(&(p_scan_actv->p_scan_filter[cursor].addr), 0, GAP_BD_ADDR_LEN);
        }
    }
    return ret;
}

/**
 ****************************************************************************************
 * @brief Check if received advertising and scan response report must be filtered or
 * not.
 *
 * @param[in] p_scan_actv
 * @param[in] p_ind
 * @param[in] p_last_adv_report
 ****************************************************************************************
 */
__STATIC bool gapm_scan_filter_packet(struct gapm_actv_scan_tag *p_scan_actv,
                                      struct gapm_ext_adv_report_ind *p_ind,
                                      struct ext_adv_report *p_last_adv_report)
{
    // Return status, indicate if packet has to be filtered
    bool status = false;

    do
    {
        // Received AD type flag
        uint8_t ad_type;

        if (p_scan_actv->common.subtype >= GAPM_SCAN_TYPE_OBSERVER)
        {
            break;
        }

        // Check if reception of advertising data triggered filtering of scan response data
        if (p_last_adv_report->evt_type & SCAN_RSP_EVT_MSK)
        {
            if ((p_last_adv_report->evt_type & LGCY_ADV_EVT_MSK) ||
                    (!(p_last_adv_report->evt_type & LGCY_ADV_EVT_MSK) && !(p_last_adv_report->evt_type & SCAN_ADV_EVT_MSK)))
            {
                // Check if reception of scan response is filtered
                status = !gapm_scan_accept_scan_rsp(p_scan_actv,
                                                    (bd_addr_t *)(&p_last_adv_report->adv_addr),
                                                    p_last_adv_report->adv_addr_type);
                break;
            }
        }

        // Retrieve AD Type flag
        ad_type = gapm_scan_get_ad_type_flag(&p_ind->data[0], p_ind->length);

        // Check for limited discovery that we have received limited discoverable data
        if ((GAPM_ISBITSET(ad_type, GAP_LE_LIM_DISCOVERABLE_FLG) && (p_scan_actv->common.subtype == GAPM_SCAN_TYPE_LIM_DISC))
                // or For general discovery that one of limited or general discoverable, flag is present in ad_type flag
                || (((GAPM_ISBITSET(ad_type, GAP_LE_GEN_DISCOVERABLE_FLG)) || (GAPM_ISBITSET(ad_type, GAP_LE_LIM_DISCOVERABLE_FLG)))
                    && (p_scan_actv->common.subtype == GAPM_SCAN_TYPE_GEN_DISC)))
        {
            if (!(p_last_adv_report->evt_type & SCAN_RSP_EVT_MSK))
            {
                // Add device to filtering list, scan response will be accepted
                gapm_scan_add_to_filter(p_scan_actv,
                                        (bd_addr_t *) & (p_last_adv_report->adv_addr),
                                        p_last_adv_report->adv_addr_type);
            }
        }
        else
        {
            status = true;
        }
    }
    while (0);

    return (status);
}

/**
 ****************************************************************************************
 * @brief Clear fragments stored in a report list.
 *
 * @param[in] p_report_list     Pointer to the list containing the report fragments
 ****************************************************************************************
 */
__STATIC void gapm_scan_clear_fragments(struct gapm_report_list *p_report_list)
{
    if (p_report_list)
    {
        while (!co_list_is_empty(&p_report_list->report_list))
        {
            // Get first fragment is the list
            struct gapm_report_elem *p_elem = (struct gapm_report_elem *)co_list_pop_front(&p_report_list->report_list);

            // Free the report element
            ke_free(p_elem);
        }
    }
}

/**
 ****************************************************************************************
 * @brief Send a GAPM_EXT_ADV_REPORT_IND message to the application. Used either once
 * all fragments of an advertising or a scan response report have been received
 * or once latest fragment of a truncated report has been received (if forwarding
 * or truncated reports has been enabled by the application when scan activity has been
 * started).
 *
 * @param[in] p_actv_scan           Pointer to the scan activity structure
 * @param[in] p_report_list         List of received fragment for the report to be sent
 * @param[in] p_last_adv_report     Pointer to the last received fragment
 * @param[in] complete              Indicate if report is complete (true) or truncated.
 ****************************************************************************************
 */
__STATIC void gapm_scan_send_adv_report_ind(struct gapm_actv_scan_tag *p_actv_scan,
        struct gapm_report_list *p_report_list,
        struct ext_adv_report *p_last_adv_report,
        bool complete)
{
    // GAPM_EXT_ADV_REPORT_IND message
    struct gapm_ext_adv_report_ind *p_ind;
    // Compute length of report to upload
    uint16_t length = (p_report_list != NULL) ? p_report_list->length : 0;
    // Offset for data writing
    uint16_t offset = 0;
    // Indicate if report is filtered or not
    bool filter;

    //  Add length of last received fragment
    length += p_last_adv_report->data_len;

    // Allocate message
    p_ind = KE_MSG_ALLOC_DYN(GAPM_EXT_ADV_REPORT_IND,
                             p_actv_scan->common.requester, TASK_GAPM,
                             gapm_ext_adv_report_ind,
                             length);

    if (p_report_list)
    {
        // Copy the stored received data
        while (!co_list_is_empty(&p_report_list->report_list))
        {
            // Get first fragment is the list
            struct gapm_report_elem *p_elem = (struct gapm_report_elem *)co_list_pop_front(&p_report_list->report_list);

            // Copy the received data
            memcpy(&p_ind->data[offset], &p_elem->data[0], p_elem->data_len);

            // Increase the offset
            offset += p_elem->data_len;

            // Free the report element
            ke_free(p_elem);
        }
    }

    // Copy the last received fragment
    memcpy(&p_ind->data[offset], &p_last_adv_report->data[0], p_last_adv_report->data_len);

    // Set the length
    p_ind->length = length;

    // Check if packet must be filtered
    filter = gapm_scan_filter_packet(p_actv_scan, p_ind, p_last_adv_report);

    if (filter)
    {
        // Free the allocated message
        ke_msg_free(ke_param2msg(p_ind));
    }
    else
    {
        // Fill the message
        if (p_last_adv_report->evt_type & SCAN_RSP_EVT_MSK)
        {
            p_ind->info = (p_last_adv_report->evt_type & LGCY_ADV_EVT_MSK)
                          ? GAPM_REPORT_TYPE_SCAN_RSP_LEG : GAPM_REPORT_TYPE_SCAN_RSP_EXT;
        }
        else
        {
            p_ind->info = (p_last_adv_report->evt_type & LGCY_ADV_EVT_MSK)
                          ? GAPM_REPORT_TYPE_ADV_LEG : GAPM_REPORT_TYPE_ADV_EXT;
        }

        if (p_last_adv_report->evt_type & CON_ADV_EVT_MSK)
        {
            p_ind->info |= GAPM_REPORT_INFO_CONN_ADV_BIT;
        }

        if (p_last_adv_report->evt_type & SCAN_ADV_EVT_MSK)
        {
            p_ind->info |= GAPM_REPORT_INFO_SCAN_ADV_BIT;
        }

        if (p_last_adv_report->evt_type & DIR_ADV_EVT_MSK)
        {
            p_ind->info |= GAPM_REPORT_INFO_DIR_ADV_BIT;
        }

        if (complete)
        {
            p_ind->info |= GAPM_REPORT_INFO_COMPLETE_BIT;
        }

        p_ind->actv_idx             = p_actv_scan->common.idx;
        p_ind->rssi                 = p_last_adv_report->rssi;
        p_ind->tx_pwr               = p_last_adv_report->tx_power;
        p_ind->phy_prim             = p_last_adv_report->phy;
        p_ind->phy_second           = p_last_adv_report->phy2;
        p_ind->adv_sid              = p_last_adv_report->adv_sid;
        p_ind->period_adv_intv      = p_last_adv_report->interval;

        // Transmitter address
        if (p_last_adv_report->adv_addr_type == ADDR_NONE)
        {
            p_ind->trans_addr.addr_type = ADDR_NONE;
        }
        else
        {
            p_ind->trans_addr.addr_type = (p_last_adv_report->adv_addr_type & ADDR_MASK);
            memcpy(&p_ind->trans_addr.addr.addr[0], &p_last_adv_report->adv_addr.addr[0], GAP_BD_ADDR_LEN);
        }

        if (p_last_adv_report->evt_type & DIR_ADV_EVT_MSK)
        {
            // Target address
            p_ind->target_addr.addr_type = (p_last_adv_report->dir_addr_type & ADDR_MASK);
            memcpy(&p_ind->target_addr.addr.addr[0], &p_last_adv_report->dir_addr.addr[0], GAP_BD_ADDR_LEN);
        }

        // Send the message
        ke_msg_send(p_ind);
    }
}

/**
 ****************************************************************************************
 * @brief Check if we can start reception of a new fragmented report by looking for an available
 * report list in which we can push the received advertising report fragment.
 *
 * @param[in] p_actv_scan       Pointer to the scan activity structure
 * @param[in] p_adv_report      Pointer to the advertising report
 *
 * @return Pointer to the report list that can contain the received fragment and the next
 * to be received.
 ****************************************************************************************
 */
__STATIC struct gapm_report_list *gapm_scan_get_new_list(struct gapm_actv_scan_tag *p_actv_scan,
        struct ext_adv_report *p_adv_report)
{
    struct gapm_report_list *p_found_list = NULL;
    // Counter
    uint8_t i;

    // Look for an available list
    for (i = 0; i < GAPM_REPORT_NB_MAX; i++)
    {
        struct gapm_report_list *p_report_list = &p_actv_scan->report_lists[i];

        if (co_list_is_empty(&p_report_list->report_list))
        {
            p_found_list = p_report_list;

            // Store the address
            p_found_list->adv_addr.addr_type = p_adv_report->adv_addr_type;
            memcpy(&p_found_list->adv_addr.addr.addr[0], &p_adv_report->adv_addr.addr[0], GAP_BD_ADDR_LEN);

            p_found_list->length = 0;

            break;
        }
    }

    return (p_found_list);
}

/**
 ****************************************************************************************
 * @brief Store received fragment of advertising or scan response report.
 *
 * @param[in] p_actv_scan       Pointer to the scan activity for which fragment has been
 *                              received.
 * @param[in] p_report_list     List in which report has to be inserted. Can be NULL if
 *                              fragment is the first one.
 * @param[in] p_adv_report      Pointer to the received fragment.
 ****************************************************************************************
 */
__STATIC void gapm_scan_store_fragment(struct gapm_actv_scan_tag *p_actv_scan,
                                       struct gapm_report_list *p_report_list,
                                       struct ext_adv_report *p_adv_report)
{
    // Report element
    struct gapm_report_elem *p_elem;
    // Report list in which report element will be inserted
    struct gapm_report_list *p_list = p_report_list;

    if (!p_list)
    {
        // Get a list that can be used
        p_list = gapm_scan_get_new_list(p_actv_scan, p_adv_report);
    }

    if (p_list)
    {
        // Allocate report element
        p_elem = (struct gapm_report_elem *)ke_malloc(sizeof(struct gapm_report_elem) + p_adv_report->data_len,
                 KE_MEM_KE_MSG);

        // Check if enough memory has been found
        if (!p_elem)
        {
            // Clear the fragment in the list
            gapm_scan_clear_fragments(p_list);
        }
        else
        {
            // Keep the data, other parameters will be taken from the last fragment
            p_elem->data_len = p_adv_report->data_len;
            memcpy(&p_elem->data[0], &p_adv_report->data[0], p_adv_report->data_len);

            // Insert the element in the list
            co_list_push_back(&p_list->report_list, &p_elem->list_hdr);

            // Increase the length
            p_list->length += p_adv_report->data_len;
        }
    }
}

/**
 ****************************************************************************************
 * @brief Check if fragment of reports have already been received for a given advertiser
 * address.
 * Up to GAPM_REPORT_NB_MAX fragmented reports from different advertiser devices can be
 * handled in parallel
 *
 * @param[in] p_actv_scan   Pointer to the scan activity structure
 * @param[in] p_adv_report  Pointer to the received advertising report fragment. Contains
 * the advertiser's address.
 *
 * @return Pointer to the list containing previously received fragment of the report
 ****************************************************************************************
 */
__STATIC struct gapm_report_list *gapm_scan_check_adv_addr(struct gapm_actv_scan_tag *p_actv_scan,
        struct ext_adv_report *p_adv_report)
{
    // Report list for the provided advertiser address
    struct gapm_report_list *p_report_list = NULL;
    // Counter
    uint8_t idx;

    for (idx = 0; idx < GAPM_REPORT_NB_MAX; idx++)
    {
        // Get report structure
        struct gapm_report_list *p_loop_rep_list = &p_actv_scan->report_lists[idx];

        // Get if list already contains report fragments
        if (!co_list_is_empty(&p_loop_rep_list->report_list))
        {
            // Compare advertiser address
            if ((p_loop_rep_list->adv_addr.addr_type == p_adv_report->adv_addr_type)
                    && !memcmp(&p_loop_rep_list->adv_addr.addr.addr[0], &p_adv_report->adv_addr.addr[0], GAP_BD_ADDR_LEN))
            {
                // Stored reports have been found
                p_report_list = p_loop_rep_list;
                break;
            }
        }
    }

    return (p_report_list);
}

/**
 ****************************************************************************************
 * @brief Check scanning parameters provided by the application.
 *
 * @param[in] p_scan_param      Pointer to the scanning parameters
 *
 * @return GAP_ERR_NO_ERROR if parameters are valid, else GAP_ERR_INVALID_PARAM
 ****************************************************************************************
 */
__STATIC uint8_t gapm_scan_check_param(struct gapm_scan_param *p_scan_param)
{
    // Error code, invalid parameter by default
    uint8_t error = GAP_ERR_INVALID_PARAM;

    do
    {
        // Check scan type
        if (p_scan_param->type > GAPM_SCAN_TYPE_SEL_CONN_DISC)
        {
            break;
        }

        // Check filter policy for duplicated packets
        if (p_scan_param->dup_filt_pol > GAPM_DUP_FILT_EN_PERIOD)
        {
            break;
        }

        error = GAP_ERR_NO_ERROR;
    }
    while (0);

    return (error);
}

/**
 ****************************************************************************************
 * @brief Start a scanning activity.
 *
 * @param[in] p_actv        Pointer to the activity structure
 * @param[in] p_param       GAPM_ACTIVITY_START_CMD message parameters
 *
 * @return GAP_ERR_NO_ERR if activity can be started
 *         GAP_ERR_INVALID_PARAMETER if scanning parameters are not valid
 *         GAP_ERR_COMMAND_DISALLOWED if scanning activity cannot be started
 ****************************************************************************************
 */
__STATIC uint8_t gapm_scan_start(struct gapm_actv_tag *p_actv, struct gapm_activity_start_cmd *p_param)
{
    // Cast the activity structure into a scanning activity structure
    struct gapm_actv_scan_tag *p_actv_scan = (struct gapm_actv_scan_tag *)p_actv;
    // Scan Parameters
    struct gapm_scan_param *p_scan_param = &p_param->u_param.scan_param;
    // Check new provided scanning parameters
    uint8_t error;

    do
    {
        // Cannot have two scanning procedures in parallel
        if (gapm_env.scan_actv_idx != GAPM_ACTV_INVALID_IDX)
        {
            error = GAP_ERR_COMMAND_DISALLOWED;
            break;
        }

        // Check provided scan parameters
        error = gapm_scan_check_param(p_scan_param);

        if (error != GAP_ERR_NO_ERROR)
        {
            break;
        }

        // Configure the activity
        p_actv_scan->common.subtype = p_scan_param->type;

        // Check if truncated advertising report are accepted
        if (!(p_scan_param->prop & GAPM_SCAN_PROP_FILT_TRUNC_BIT))
        {
            SETB(p_actv_scan->common.info, GAPM_SCAN_INFO_ACCEPT_TRUNCATED, true);
        }

        // Keep activity identifier in mind
        gapm_env.actv_idx      = p_actv_scan->common.idx;
        gapm_env.scan_actv_idx = p_actv_scan->common.idx;

        // Send a LE Set Extended Scan Parameters command to the controller
        gapm_scan_send_hci_le_set_ext_scan_param_cmd(p_actv_scan, p_scan_param);
    }
    while (0);

    return (error);
}

/**
 ****************************************************************************************
 * @brief Stop one scanning activity. Activity state must have been set to STOPPING state
 * before calling this function
 *
 * @param[in] p_actv    Pointer to the activity structure to be stopped
 ****************************************************************************************
 */
__STATIC void gapm_scan_stop(struct gapm_actv_tag *p_actv)
{
    // Cast the activity structure in order to retrieve scanning parameters
    struct gapm_actv_scan_tag *p_actv_scan = (struct gapm_actv_scan_tag *)p_actv;

    // Keep activity identifier in mind
    gapm_env.actv_idx = p_actv_scan->common.idx;

    // Send a LE Set Extended Scan Enable command (Disable) to the controller
    gapm_scan_send_hci_le_set_ext_scan_en_cmd(p_actv_scan, false);
}

/**
 ****************************************************************************************
 * @brief Remove one scanning activity. It is considered here that scanning activity
 * is in DELETING state.
 *
 * @param[in] p_actv    Pointer to the activity structure to be deleted.
 ****************************************************************************************
 */
__STATIC void gapm_scan_delete(struct gapm_actv_tag *p_actv)
{
    // Nothing special to be done
    gapm_actv_deleted(p_actv);
}

/*
 * EXTERNAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

void gapm_scan_actv_clean(struct gapm_actv_tag *p_actv)
{
    struct gapm_actv_scan_tag *p_actv_scan = (struct gapm_actv_scan_tag *)p_actv;
    // Counter
    uint8_t i;

    if (p_actv_scan->p_scan_filter)
    {
        ke_free(p_actv_scan->p_scan_filter);
    }

    for (i = 0; i < GAPM_REPORT_NB_MAX; i++)
    {
        gapm_scan_clear_fragments(&p_actv_scan->report_lists[i]);
    }
}

uint8_t gapm_scan_create(uint8_t actv_idx, struct gapm_activity_create_cmd *p_param)
{
    // Check provided scanning parameters
    uint8_t error;
    // Allocated activity structure
    struct gapm_actv_scan_tag *p_actv_scan;

    do
    {
        // Check if supported roles allow to start scanning
        if (!GAPM_IS_ROLE_SUPPORTED(GAP_ROLE_OBSERVER) && !GAPM_IS_ROLE_SUPPORTED(GAP_ROLE_CENTRAL))
        {
            error = GAP_ERR_COMMAND_DISALLOWED;
            break;
        }

        // Check that provided address type is valid
        if (!gapm_is_addr_type_valid(p_param->own_addr_type))
        {
            error = GAP_ERR_INVALID_PARAM;
            break;
        }

        p_actv_scan = (struct gapm_actv_scan_tag *)gapm_actv_alloc(actv_idx, sizeof(struct gapm_actv_scan_tag));

        // Check if enough memory has been found
        if (!p_actv_scan)
        {
            error = GAP_ERR_INSUFF_RESOURCES;
            break;
        }

        error = GAP_ERR_NO_ERROR;

        // Fill the activity structure
        p_actv_scan->common.type = GAPM_ACTV_TYPE_SCAN;
        p_actv_scan->common.own_addr_type = p_param->own_addr_type;
        p_actv_scan->common.requester = gapm_get_requester(GAPM_OP_AIR);

        p_actv_scan->common.cb_actv_start  = gapm_scan_start;
        p_actv_scan->common.cb_actv_stop   = gapm_scan_stop;
        p_actv_scan->common.cb_actv_delete = gapm_scan_delete;

        gapm_actv_created((struct gapm_actv_tag *)p_actv_scan, GAP_ERR_NO_ERROR);
    }
    while (0);

    return (error);
}

/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Common handler for following HCI command complete events:
 *  - LE Set Extended Scan Parameters Command (HCI_LE_SET_EXT_SCAN_PARAM_CMD_OPCODE)
 *  - LE Set Extended Scan Enable Command (HCI_LE_SET_EXT_SCAN_EN_CMD_OPCODE)
 *
 * @param[in] opcode    Operation code of received message
 * @param[in] param     Pointer to the parameters of the message.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int hci_le_cmd_cmp_evt_scan_handler(uint16_t opcode, struct hci_basic_cmd_cmp_evt const *p_event)
{
    // Scanning activity structure
    struct gapm_actv_scan_tag *p_actv_scan;

    do
    {
        // Find activity for which the command complete event has been received
        if (!gapm_actv_retrieve_cmd_cmp_evt((struct gapm_actv_tag **)&p_actv_scan, opcode))
        {
            break;
        }

        // Call the proper handler based on current activity state
        switch (p_actv_scan->common.state)
        {
        case GAPM_ACTV_STARTING:
        {
            hci_le_cmd_cmp_evt_starting_handler(p_actv_scan, opcode, p_event);
        }
        break;

        case GAPM_ACTV_STOPPING:
        {
            hci_le_cmd_cmp_evt_stopping_handler(p_actv_scan, opcode, p_event);
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
 * @brief Handle reception of HCI LE Extended Advertising Report Event
 *
 * @param[in] opcode      Operation code of received message
 * @param[in] p_event     Pointer to the parameters of the message.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int hci_le_ext_adv_report_evt_handler(uint16_t opcode, struct hci_le_ext_adv_report_evt *p_event)
{
    // Get index of started scanning activity
    uint8_t scan_actv_idx = gapm_env.scan_actv_idx;

    if (scan_actv_idx != GAPM_ACTV_INVALID_IDX)
    {
        // Get activity structure
        struct gapm_actv_scan_tag *p_actv_scan
            = (struct gapm_actv_scan_tag *)gapm_env.actvs[scan_actv_idx];

        for (int i = 0; i < p_event->nb_reports; i++)
        {
            // Get report information
            struct ext_adv_report *p_adv_report = &p_event->adv_rep[i];
            // Report list
            struct gapm_report_list *p_report_list;
            // Data status
            uint8_t data_status;

            // If only connectable discovery, only accept connectable advertising report
            if (p_actv_scan->common.subtype >= GAPM_SCAN_TYPE_CONN_DISC)
            {
                if (!(p_adv_report->evt_type & CON_ADV_EVT_MSK))
                {
                    continue;
                }
            }

            // Check if fragments are currently stored for the advertiser address - can be NULL
            p_report_list = gapm_scan_check_adv_addr(p_actv_scan, p_adv_report);

            // Extract data status
            data_status = (uint8_t) GETF(p_adv_report->evt_type, ADV_EVT_DATA_STATUS);

            // If no more fragment will come because report is complete or truncated, upload the report
            if (data_status == ADV_EVT_DATA_STATUS_COMPLETE)
            {
                // Send the full report to the application
                gapm_scan_send_adv_report_ind(p_actv_scan, p_report_list, p_adv_report, true);
            }
            else if (data_status == ADV_EVT_DATA_STATUS_TRUNCATED)
            {
                // Check if application has accepted forwarding of truncated data.
                if (GETB(p_actv_scan->common.info, GAPM_SCAN_INFO_ACCEPT_TRUNCATED))
                {
                    // Send the truncated report to the application
                    gapm_scan_send_adv_report_ind(p_actv_scan, p_report_list, p_adv_report, false);
                }
                else
                {
                    // Clear stored fragment
                    gapm_scan_clear_fragments(p_report_list);
                }
            }
            else if (data_status == ADV_EVT_DATA_STATUS_INCOMPLETE)
            {
                // Store the receive report fragment
                gapm_scan_store_fragment(p_actv_scan, p_report_list, p_adv_report);
            }
            // else drop the fragment
        }
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handle reception of HCI LE Scan Timeout Event
 *
 * @param[in] opcode      Operation code of received message
 * @param[in] p_event     Pointer to the parameters of the message.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int hci_le_scan_timeout_evt_handler(uint16_t opcode, struct hci_le_scan_timeout_evt const *p_event)
{
    // Get index of started scanning activity
    uint8_t scan_actv_idx = gapm_env.scan_actv_idx;

    if (scan_actv_idx != GAPM_ACTV_INVALID_IDX)
    {
        // Get activity structure
        struct gapm_actv_scan_tag *p_actv_scan
            = (struct gapm_actv_scan_tag *)gapm_env.actvs[scan_actv_idx];

        gapm_env.scan_actv_idx = GAPM_ACTV_INVALID_IDX;

        // Operation is over
        gapm_actv_stopped((struct gapm_actv_tag *)p_actv_scan, GAP_ERR_TIMEOUT);
    }

    return (KE_MSG_CONSUMED);
}


#endif //(BLE_OBSERVER)

/// @} GAPM_SCAN
