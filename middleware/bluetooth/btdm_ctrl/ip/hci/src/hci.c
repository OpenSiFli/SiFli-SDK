/**
 ****************************************************************************************
 *
 * @file hci.c
 *
 * @brief HCI module source file.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup HCI
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"       // SW configuration

#if (HCI_PRESENT)
#include "rwip.h"            // IP definitions
#include <string.h>          // string manipulation
#include "co_error.h"        // error definition
#include "co_utils.h"        // common utility definition
#include "co_list.h"         // list definition

#include "hci.h"             // hci definition
#include "hci_int.h"         // hci internal definition

#include "ke_msg.h"          // kernel message declaration
#include "ke_task.h"         // kernel task definition
#include "ke_event.h"        // kernel event definition
#include "ke_mem.h"          // kernel memory definition
#include "ke_timer.h"        // kernel timer definition

#if (HOST_PRESENT)
    #include "hl_hci.h"         // use to push message to host
#endif //(HOST_PRESENT)

#if (EMB_PRESENT)
    #include "reg_access.h"     // us em_wr
#endif // (EMB_PRESENT)
#if (BLE_EMB_PRESENT && BLE_ISOOHCI)
    #include "isoohci.h"
#endif //(BLE_EMB_PRESENT && BLE_ISOOHCI)

#include "dbg.h"

/*
 * DEFINES
 ****************************************************************************************
 */
/// Default event mask
#if (!BT_HOST_PRESENT)
    #define EVT_MASK_DEFAULT    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x9F, 0x00, 0x20}
#else
    #define EVT_MASK_DEFAULT    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x3F}
#endif // (!BT_HOST_PRESENT)

/*
 * ENUMERATIONS DEFINITIONS
 ****************************************************************************************
 */

/*
 * STRUCTURES DEFINITIONS
 ****************************************************************************************
 */

/*
 * CONSTANTS DEFINITIONS
 ****************************************************************************************
 */

/// Default event mask
__STATIC const struct evt_mask hci_def_evt_msk = {EVT_MASK_DEFAULT};

#if (BLE_EMB_PRESENT)
/// Default LE event mask
__STATIC const struct evt_mask hci_le_def_evt_msk = {BLE_EVT_MASK};
#endif // (BLE_EMB_PRESENT)

/// Reserved event mask
__STATIC const struct evt_mask hci_rsvd_evt_msk = {{0x00, 0x60, 0x04, 0x00, 0xF8, 0x07, 0x40, 0x02}};


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

#if (EMB_PRESENT && HOST_PRESENT && HCI_TL_SUPPORT)
    /**
    * Indicate if HCI is used by external Host, or internal Host.
    * Used in Full mode only. By default HCI is used by internal Host.
    * HCI switches to external host as soon as an HCI command is received.
    * This variable should not be reset.
    */
    bool hci_ext_host;
#endif // (EMB_PRESENT && HOST_PRESENT && HCI_TL_SUPPORT)

///HCI environment context
struct hci_env_tag hci_env;

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
/**
 ****************************************************************************************
 * @brief Check if the event to be sent to the host is masked or not
 *
 * @param[in] msg  Pointer to the message containing the event
 *
 * @return true id the message has to be filtered out, false otherwise
 *****************************************************************************************
 */
__STATIC bool hci_evt_mask_check(struct ke_msg *msg)
{
    bool masked = false;
    uint8_t evt_code;

    switch (msg->id)
    {
#if BLE_EMB_PRESENT
    case HCI_LE_EVENT:
    {
        // LE meta event
        evt_code = HCI_LE_META_EVT_CODE;
    }
    break;
#endif // BLE_EMB_PRESENT
    case HCI_EVENT:
    {
        // Get event code
        evt_code = msg->src_id;
    }
    break;

    default:
    {
        // Cannot be masked
        return false;
    }
    }

    // Check if this event is maskable
    if (evt_code < HCI_MAX_EVT_MSK_PAGE_1_CODE)
    {
        uint8_t index = evt_code - 1;

        //Checking if the event is masked or not
        masked = ((hci_env.evt_msk.mask[index >> 3] & (1 << (index & 0x07))) == 0x00);

#if (BLE_EMB_PRESENT)
        if (!masked && (evt_code == HCI_LE_META_EVT_CODE))
        {
            // Get Sub-code of the mevent
            uint8_t *subcode = (uint8_t *)ke_msg2param(msg);
            //Translate the subcode in index to parse the mask
            uint8_t index = *subcode - 1;
            //Checking if the event is masked or not
            masked = ((hci_env.le_evt_msk.mask[index >> 3] & (1 << (index & 0x07))) == 0x00);
        }
#endif // (BLE_EMB_PRESENT)
    }
    else if (evt_code < HCI_MAX_EVT_MSK_PAGE_2_CODE)
    {
        // In this area the evt code is in the range [EVT_MASK_CODE_MAX<evt_code<HCI_MAX_EVT_MSK_CODE]
        // The index should be < EVT_MASK_CODE_MAX to avoid evt_msk_page_2.mask array overflow
        uint8_t index = evt_code - EVT_MASK_CODE_MAX;

        //Checking if the event is masked or not
        masked = ((hci_env.evt_msk_page_2.mask[index >> 3] & (1 << (index & 0x07))) == 0x00);
    }
    return masked;
}
#endif //(BLE_EMB_PRESENT || BT_EMB_PRESENT)

#if BT_EMB_PRESENT
/**
 ****************************************************************************************
 * @brief Check if a connection request event has to be forwarded to the host or not
 * This function also takes the required decisions according to the auto_accept parameter
 *
 * @param[in] bdaddr    BDADDR contained in the inquiry result event
 * @param[in] class     Class of device contained in the inquiry result event
 * @param[in] link_type Type of link that is requested (asynchronous or synchronous)
 *
 * @return true if the event has to be filtered out, false otherwise
 *****************************************************************************************
 */
__STATIC uint8_t hci_evt_filter_con_check(struct bd_addr *bdaddr, struct devclass   *classofdev, uint8_t link_type, uint8_t link_id)
{
    uint8_t filtered = false;
    uint8_t auto_accept = DO_NOT_AUTO_ACCEPT_CONNECTION;

    /* Check if a Connection type is present (start from last to first)           */
    for (int i = HCI_FILTER_NB - 1 ; i >= 0 ; i--)
    {
        hci_evt_filter_t *filter = &hci_env.evt_filter[i];

        // If this filter is a ConnectionSetupFilter
        if ((filter->in_use == true) && (filter->type == CONNECTION_FILTER_TYPE))
        {
            // There is at least one connection filter set, so now we should reject by default unless
            // the connection request matches one of the filters
            auto_accept = 0xFF;

            // If the condition is a All Device type
            if (filter->condition == ALL_FILTER_CONDITION_TYPE)
            {
                // This connection will not be rejected
                auto_accept = filter->auto_accept;
                break;
            }
            // If the condition is a ClassOfDevice type
            else if (filter->condition == CLASS_FILTER_CONDITION_TYPE)
            {
                struct devclass class_masked;
                struct classofdevcondition *cond = &filter->param.device_class;

                // Remove don't care bit of Class Of Device
                class_masked.A[0] = classofdev->A[0] & cond->class_mask.A[0];
                class_masked.A[1] = classofdev->A[1] & cond->class_mask.A[1];
                class_masked.A[2] = classofdev->A[2] & cond->class_mask.A[2];

                // Check if any device class is allowed or the Class of the filter matches the Class of the Inquiry result
                if ((cond->classofdev.A[0] == 0 && cond->classofdev.A[1] == 0 && cond->classofdev.A[2] == 0)
                        || !memcmp(&class_masked, &cond->classofdev, sizeof(struct devclass)))
                {
                    // This connection will not be rejected
                    auto_accept = filter->auto_accept;
                    break;
                }
            }
            /* If this filter is a ConnectionFilter                                     */
            else if (filter->condition == BD_ADDR_FILTER_CONDITION_TYPE)
            {
                // Check if the BdAddr of the filter matches the BdAddr of the Connect req
                if (!memcmp(bdaddr, &filter->param.bdaddr, sizeof(struct bd_addr)))
                {
                    // This connection will not be rejected
                    auto_accept = filter->auto_accept;
                    break;
                }
            }
        }
    }

    if ((auto_accept == ACCEPT_CONNECTION_SLAVE) || (auto_accept == ACCEPT_CONNECTION_MASTER))
    {
        // Auto accept the connection request
        filtered = true;

        // If this is an ACL link
        if ((link_type == ACL_TYPE)
                // or a SCO link
#if (MAX_NB_SYNC > 0)
                || (link_type == SCO_TYPE)
#endif // (MAX_NB_SYNC > 0)
           )
        {
            struct hci_accept_con_req_cmd *cmd = KE_MSG_ALLOC(HCI_COMMAND, KE_BUILD_ID(TASK_LC, link_id), HCI_ACCEPT_CON_REQ_CMD_OPCODE, hci_accept_con_req_cmd);

            // Fill-in the parameter structure
            if (auto_accept == ACCEPT_CONNECTION_SLAVE)
            {
                cmd->role = ACCEPT_REMAIN_SLAVE;
            }
            else
            {
                cmd->role = ACCEPT_SWITCH_TO_MASTER;
            }
            cmd->bd_addr = *bdaddr;

            // Send the message
            ke_msg_send(cmd);

            // Save opcode, used to filter the returning CS event
            hci_env.auto_accept_opcode = HCI_ACCEPT_CON_REQ_CMD_OPCODE;
        }
#if (MAX_NB_SYNC > 0)
        // If this is an eSCO Synchronous Link
        else
        {
            struct hci_accept_sync_con_req_cmd *cmd = KE_MSG_ALLOC(HCI_COMMAND, KE_BUILD_ID(TASK_LC, link_id), HCI_ACCEPT_SYNC_CON_REQ_CMD_OPCODE, hci_accept_sync_con_req_cmd);

            // Fill in parameter structure
            cmd->bd_addr = *bdaddr;
            cmd->tx_bw = SYNC_BANDWIDTH_DONT_CARE;
            cmd->rx_bw = SYNC_BANDWIDTH_DONT_CARE;
            cmd->max_lat = SYNC_DONT_CARE_LATENCY;
            cmd->vx_set = hci_env.voice_settings;
            cmd->retx_eff = SYNC_RE_TX_DONT_CARE;
            cmd->pkt_type = 0xFFFF;   /// All packet type

            // Send the message
            ke_msg_send(cmd);

            // Save opcode, used to filter the returning CS event
            hci_env.auto_accept_opcode = HCI_ACCEPT_SYNC_CON_REQ_CMD_OPCODE;
        }
#endif // (MAX_NB_SYNC > 0)
    }
    else if (auto_accept != DO_NOT_AUTO_ACCEPT_CONNECTION)
    {
        // This Device does not match a filter => filtered and rejected
        filtered = true;

        // If this is an ACL link
        if (link_type == ACL_TYPE)
        {
            struct hci_reject_con_req_cmd *cmd = KE_MSG_ALLOC(HCI_COMMAND, KE_BUILD_ID(TASK_LC, link_id), HCI_REJECT_CON_REQ_CMD_OPCODE, hci_reject_con_req_cmd);
            cmd->bd_addr = *bdaddr;
            cmd->reason = CO_ERROR_CONN_REJ_UNACCEPTABLE_BDADDR;
            ke_msg_send(cmd);

            // Save opcode, used to filter the returning CS event
            hci_env.auto_accept_opcode = HCI_REJECT_CON_REQ_CMD_OPCODE;
        }
#if (MAX_NB_SYNC > 0)
        // If this is a Synchronous Link (SCO or eSCO)
        else
        {
            struct hci_reject_sync_con_req_cmd *cmd = KE_MSG_ALLOC(HCI_COMMAND, KE_BUILD_ID(TASK_LC, link_id), HCI_REJECT_SYNC_CON_REQ_CMD_OPCODE, hci_reject_sync_con_req_cmd);
            cmd->bd_addr = *bdaddr;
            cmd->reason = CO_ERROR_CONN_REJ_UNACCEPTABLE_BDADDR;
            ke_msg_send(cmd);

            // Save opcode, used to filter the returning CS event
            hci_env.auto_accept_opcode = HCI_REJECT_SYNC_CON_REQ_CMD_OPCODE;
        }
#endif // (MAX_NB_SYNC > 0)
    }

    return (filtered);
}

/**
 ****************************************************************************************
 * @brief Check if an inquiry result event has to be forwarded to the host or not
 *
 * @param[in] bdaddr  BDADDR contained in the inquiry result event
 * @param[in] class   Class of device contained in the inquiry result event
 *
 * @return true if the event has to be filtered out, false otherwise
 *****************************************************************************************
 */
__STATIC uint8_t hci_evt_filter_inq_check(struct bd_addr *bdaddr, struct devclass *classofdev)
{
    int     i;
    uint8_t filtered = false;  // By default the event is not filtered
    struct devclass empty_classofdev = {{0, 0, 0}};

    // Check if an inquiry filter type is present
    for (i = 0; i < HCI_FILTER_NB; i++)
    {
        hci_evt_filter_t *filter = &hci_env.evt_filter[i];

        // If this filter is an InquiryFilter
        if ((filter->in_use == true) && (filter->type == INQUIRY_FILTER_TYPE))
        {
            // There is at least one inquiry filter set, so now we should filter unless
            // the inquiry result matches one of the filters
            filtered = true;

            // If the condition is a ClassOfDevice type
            if (filter->condition == CLASS_FILTER_CONDITION_TYPE)
            {
                struct devclass class_masked;
                struct classofdevcondition *cond = &filter->param.device_class;

                // Remove don't care bit of Class Of Device
                class_masked.A[0] = classofdev->A[0] & cond->class_mask.A[0];
                class_masked.A[1] = classofdev->A[1] & cond->class_mask.A[1];
                class_masked.A[2] = classofdev->A[2] & cond->class_mask.A[2];

                // Check if the Class of the filter match the Class of the Inquiry result
                if (!memcmp(&empty_classofdev.A[0], &cond->classofdev.A[0], sizeof(struct devclass))
                        || !memcmp(&class_masked.A[0], &cond->classofdev.A[0], sizeof(struct devclass)))
                {
                    // This InquiryResult must NOT be filtered
                    filtered = false;
                    break;
                }
            }
            // If this filter is a BDADDR type
            else if (filter->condition == BD_ADDR_FILTER_CONDITION_TYPE)
            {
                // Check if the BdAddr of the filter match the BdAddr of the Inquiry res
                if (!memcmp(bdaddr, &filter->param.bdaddr, sizeof(struct bd_addr)))
                {
                    // This InquiryResult must NOT be filtered
                    filtered = false;
                    break;
                }
            }
        }
    }

    return (filtered);
}

/**
 ****************************************************************************************
 * @brief Check if an event has to be forwarded to the host or not
 *
 * @param[in] msg  Pointer to the event message to be transmitted to the host
 *
 * @return true if the event has to be filtered out, false otherwise
 *****************************************************************************************
 */
__STATIC uint8_t hci_evt_filter_check(struct ke_msg *msg)
{
    uint8_t filtered = false;

    switch (msg->id)
    {
    case HCI_EVENT:
    {
        // Get event code
        uint8_t evt_code = msg->src_id;
        uint8_t *param = (uint8_t *)msg->param;

        switch (evt_code)
        {
        // InquiryResult Event
        case HCI_INQ_RES_EVT_CODE:
        case HCI_INQ_RES_WITH_RSSI_EVT_CODE:
        case HCI_EXT_INQ_RES_EVT_CODE:
        {
            struct devclass *classofdev;
            // Retrieve the information required for the filtering from the PDU
            struct bd_addr *bdaddr = (struct bd_addr *)&param[1];
            if (evt_code == HCI_INQ_RES_EVT_CODE)
            {
                struct hci_inq_res_evt *evt = (struct hci_inq_res_evt *) param;
                classofdev = (struct devclass *)&evt->class_of_dev;
            }
            else
            {
                struct hci_ext_inq_res_evt *evt = (struct hci_ext_inq_res_evt *) param;
                classofdev = (struct devclass *)&evt->class_of_dev;
            }

            // Check if the event has to be filtered or not
            filtered = hci_evt_filter_inq_check(bdaddr, classofdev);
        }
        break;
        // Connection Request Event
        case HCI_CON_REQ_EVT_CODE:
        {
            // Retrieve the information required for the filtering from the PDU
            struct bd_addr *bdaddr = (struct bd_addr *)&param[0];
            struct devclass *classofdev = (struct devclass *)&param[6];
            uint8_t link_type = param[9];

            // Check if the event has to be filtered or not
            filtered = hci_evt_filter_con_check(bdaddr, classofdev, link_type, msg->dest_id);
        }
        break;
        // Connection Complete Event
        case HCI_CON_CMP_EVT_CODE:
        {
            // Check if a connection was auto-rejected
            if (hci_env.auto_reject)
            {
                filtered = true;
                hci_env.auto_reject = false;
            }
        }
        break;
        default:
        {
            // Nothing to do
        }
        }
    }
    break;

    case HCI_CMD_STAT_EVENT:
    {
        // Filter CS event associated to the current auto-accept command
        if (msg->src_id == hci_env.auto_accept_opcode)
        {
            filtered = true;
            hci_env.auto_accept_opcode = 0x0000;

            if (msg->src_id == HCI_REJECT_CON_REQ_CMD_OPCODE)
            {
                hci_env.auto_reject = true;
            }
        }
    }
    break;

    default:
    {
        // Not an event
    }
    break;
    }

    return filtered;
}

/**
 ****************************************************************************************
 * @brief Reset the list of event filters
 *
 * @return Status
 *****************************************************************************************
 */
__STATIC uint8_t hci_evt_filter_reset(void)
{
    for (int i = 0 ; i < HCI_FILTER_NB; i++)
    {
        hci_env.evt_filter[i].in_use = false;
    }
    return (CO_ERROR_NO_ERROR);
}

/**
 ****************************************************************************************
 * @brief Allocate an event filter structure
 *
 * @return A pointer to the allocated filter, if any, NULL otherwise
 *****************************************************************************************
 */
__STATIC hci_evt_filter_t *hci_evt_filter_alloc(void)
{
    int i;
    hci_evt_filter_t *evt_filter;

    for (i = 0; i < HCI_FILTER_NB; i++)
    {
        if (hci_env.evt_filter[i].in_use == false)
        {
            break;
        }
    }
    if (i < HCI_FILTER_NB)
    {
        evt_filter = &hci_env.evt_filter[i];
        evt_filter->in_use = true;
    }
    else
    {
        evt_filter = NULL;
    }

    return (evt_filter);
}

/**
 ****************************************************************************************
 * @brief Add an inquiry event filter
 *
 * @param[in] condition  Filter condition type
 * @param[in] param      Pointer to the condition parameters
 *
 * @return The status of the filter addition
 *****************************************************************************************
 */
__STATIC uint8_t hci_evt_filter_inq_add(struct inq_res_filter const *filter)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;
    hci_evt_filter_t *evt_filter;

    switch (filter->cond_type)
    {
    case ALL_FILTER_CONDITION_TYPE:
    {
        // Remove all Inquiry type
        for (int i = 0 ; i < HCI_FILTER_NB ; i++)
        {
            if (hci_env.evt_filter[i].type == INQUIRY_FILTER_TYPE)
            {
                hci_env.evt_filter[i].in_use = false;
            }
        }

        status = CO_ERROR_NO_ERROR;
    }
    break;

    case CLASS_FILTER_CONDITION_TYPE:
    case BD_ADDR_FILTER_CONDITION_TYPE:
    {
        // Add the new filter
        evt_filter = hci_evt_filter_alloc();
        if (evt_filter != NULL)
        {
            evt_filter->type = INQUIRY_FILTER_TYPE;
            evt_filter->condition = filter->cond_type;
            if (filter->cond_type == CLASS_FILTER_CONDITION_TYPE)
            {
                struct classofdevcondition *dev_class = &evt_filter->param.device_class;
                // Store the mask
                dev_class->class_mask = filter->cond.cond_1.class_of_dev_msk;

                // Store the class, masked with the class mask
                dev_class->classofdev.A[0] = filter->cond.cond_1.class_of_dev.A[0] & filter->cond.cond_1.class_of_dev_msk.A[0];
                dev_class->classofdev.A[1] = filter->cond.cond_1.class_of_dev.A[1] & filter->cond.cond_1.class_of_dev_msk.A[1];
                dev_class->classofdev.A[2] = filter->cond.cond_1.class_of_dev.A[2] & filter->cond.cond_1.class_of_dev_msk.A[2];
            }
            else
            {
                evt_filter->param.bdaddr = filter->cond.cond_2.bd_addr;
            }
            status = CO_ERROR_NO_ERROR;
        }
        else
        {
            status = CO_ERROR_MEMORY_CAPA_EXCEED;
        }
    }
    break;

    default:
    {
        // Nothing to do
    }
    break;
    }
    return (status);
}


/**
 ****************************************************************************************
 * @brief Add a connection event filter
 *
 * @param[in] condition  Filter condition type
 * @param[in] param      Pointer to the condition parameters
 *
 * @return The status of the filter addition
 *****************************************************************************************
 */
__STATIC uint8_t hci_evt_filter_con_add(struct con_set_filter const *filter)
{
    uint8_t status = CO_ERROR_NO_ERROR;
    hci_evt_filter_t *evt_filter;

    switch (filter->cond_type)
    {
    case ALL_FILTER_CONDITION_TYPE:
    {
        uint8_t auto_accept = filter->cond.cond_0.auto_accept;
        // Check auto_accept parameter
        if ((auto_accept >= DO_NOT_AUTO_ACCEPT_CONNECTION) && (auto_accept <= ACCEPT_CONNECTION_MASTER))
        {
            // Remove all Connection type
            for (int i = 0 ; i < HCI_FILTER_NB ; i++)
            {
                if (hci_env.evt_filter[i].type == CONNECTION_FILTER_TYPE)
                {
                    hci_env.evt_filter[i].in_use = false;
                }
            }
            // Add the new filter
            evt_filter = hci_evt_filter_alloc();
            if (evt_filter != NULL)
            {
                evt_filter->type = CONNECTION_FILTER_TYPE;
                evt_filter->condition = ALL_FILTER_CONDITION_TYPE;
                evt_filter->auto_accept = auto_accept;
            }
            else
            {
                status = CO_ERROR_MEMORY_CAPA_EXCEED;
            }
        }
    }
    break;

    case CLASS_FILTER_CONDITION_TYPE:
    case BD_ADDR_FILTER_CONDITION_TYPE:
    {
        uint8_t auto_accept = filter->cond.cond_1.auto_accept;
        // Check auto_accept parameter
        if ((auto_accept >= DO_NOT_AUTO_ACCEPT_CONNECTION) && (auto_accept <= ACCEPT_CONNECTION_MASTER))
        {
            // Remove all Connection type with ALL_FILTER_CONDITION_TYPE set
            for (int i = 0; i < HCI_FILTER_NB ; i++)
            {
                if ((hci_env.evt_filter[i].in_use == true) &&
                        (hci_env.evt_filter[i].type == CONNECTION_FILTER_TYPE) &&
                        (hci_env.evt_filter[i].condition == ALL_FILTER_CONDITION_TYPE))
                {
                    hci_env.evt_filter[i].in_use = false;
                }
            }
            // Add the new filter
            evt_filter = hci_evt_filter_alloc();
            if (evt_filter != NULL)
            {
                evt_filter->type = CONNECTION_FILTER_TYPE;
                evt_filter->condition = filter->cond_type;
                evt_filter->auto_accept = auto_accept;
                if (filter->cond_type == CLASS_FILTER_CONDITION_TYPE)
                {
                    struct classofdevcondition *dev_class = &evt_filter->param.device_class;
                    // Store the mask
                    dev_class->class_mask = filter->cond.cond_1.class_of_dev_msk;

                    // Store the class, masked with the class mask
                    dev_class->classofdev.A[0] = filter->cond.cond_1.class_of_dev.A[0] & filter->cond.cond_1.class_of_dev_msk.A[0];
                    dev_class->classofdev.A[1] = filter->cond.cond_1.class_of_dev.A[1] & filter->cond.cond_1.class_of_dev_msk.A[1];
                    dev_class->classofdev.A[2] = filter->cond.cond_1.class_of_dev.A[2] & filter->cond.cond_1.class_of_dev_msk.A[2];
                }
                else
                {
                    evt_filter->param.bdaddr = filter->cond.cond_2.bd_addr;
                }
            }
            else
            {
                status = CO_ERROR_MEMORY_CAPA_EXCEED;
            }
        }
        else
        {
            status = CO_ERROR_INVALID_HCI_PARAM;
        }
    }
    break;

    default:
    {
        // Nothing to do
    }
    break;
    }

    return (status);
}
#endif //BT_EMB_PRESENT

/*
 * MODULES INTERNAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */


/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void hci_initialize(bool is_reset)
{
    if (!is_reset)
    {
#if (RW_DEBUG)
#if (BLE_EMB_PRESENT || BLE_HOST_PRESENT)
        // Check compiler padding for co_util_unpack expected S2/s2 alignment.
        struct hci_le_set_ext_adv_en_cmd s1;
        struct hci_le_set_ext_scan_param_cmd s2;
#if (BLE_CIS)
        struct hci_le_set_cig_params_cmd s3;
#endif // (BLE_CIS)
        ASSERT_ERR(((((uint32_t) & (s1.sets[0])) & 0x1) == 0) && ((((uint32_t) & (s1.sets[1])) & 0x1) == 0));
        ASSERT_ERR(((((uint32_t) & (s2.phy[0])) & 0x1) == 0) && ((((uint32_t) & (s2.phy[1])) & 0x1) == 0));
#if (BLE_CIS)
        ASSERT_ERR(((((uint32_t) & (s3.params[0])) & 0x1) == 0) && ((((uint32_t) & (s3.params[1])) & 0x1) == 0));
#endif // (BLE_CIS)
#endif // (BLE_EMB_PRESENT || BLE_HOST_PRESENT)
        // check that HCI tab are properly ordered
        hci_check_desc_tabs_order();
#endif // (RW_DEBUG)

#if (EMB_PRESENT && HOST_PRESENT && HCI_TL_SUPPORT)
        // HCI used by internal Host by default
        hci_ext_host = false;
#endif // (EMB_PRESENT && HOST_PRESENT && HCI_TL_SUPPORT)
    }

    memset(&hci_env, 0, sizeof(hci_env));

    // Initialize event mask
    hci_evt_mask_set(&hci_def_evt_msk, HCI_PAGE_DFT);

    // Initialize connection accept timeout
    hci_con_accept_to_set(CON_ACCEPT_TO_DFT);

#if (BLE_EMB_PRESENT)
    // Initialize LE event mask
    hci_evt_mask_set(&hci_le_def_evt_msk, HCI_PAGE_LE);
#endif // (BLE_EMB_PRESENT)

#if (BT_EMB_PRESENT || BLE_EMB_PRESENT)
    hci_fc_init();
#endif //(BT_EMB_PRESENT || BLE_EMB_PRESENT)

#if (HCI_TL_SUPPORT)
    // Reset the HCI
    hci_tl_init(is_reset);
#endif //(HCI_TL_SUPPORT)
}

#if (EMB_PRESENT && HOST_PRESENT && HCI_TL_SUPPORT)
bool hci_is_ext_host(void)
{
    return hci_ext_host;
}
#endif // (EMB_PRESENT && HOST_PRESENT && HCI_TL_SUPPORT)

#if (EMB_PRESENT)
#if (BT_53)
bool hci_evtcode_mask_check(uint8_t evt_code)
{
    bool mask = 0;

    // check if bit mask in page 1
    if (evt_code < HCI_MAX_EVT_MSK_PAGE_1_CODE)
    {
        uint8_t index = evt_code - 1;
        mask = ((hci_env.evt_msk.mask[index >> 3] & (1 << (index & 0x07))) != 0);
    }
    else if (evt_code < HCI_MAX_EVT_MSK_PAGE_2_CODE)
    {
        // check bit mask in page 2
        uint8_t index = evt_code - EVT_MASK_CODE_MAX;
        mask = ((hci_env.evt_msk_page_2.mask[index >> 3] & (1 << (index & 0x07))) != 0);
    }

    return mask;
}
#endif // (BT_53)

void hci_send_2_host(void *p_param)
{
    struct ke_msg *p_msg = ke_param2msg(p_param);

    //Trace sent kernel message
    TRC_REQ_KE_MSG_SEND(p_msg);

    // Initialize Descriptor pointers
    hci_env.p_cmd_evt_desc = NULL;
    hci_env.p_evt_desc = NULL;

    if (hci_evt_mask_check(p_msg)
#if (BT_EMB_PRESENT)
            || hci_evt_filter_check(p_msg)
#endif //(BT_EMB_PRESENT)
       )
    {
        // Free the kernel message space
        ke_msg_free(p_msg);
        return;
    }

    /// Compute event or command descriptor
    switch (p_msg->id)
    {
    case HCI_CMD_CMP_EVENT:
    {
        uint16_t opcode = p_msg->src_id;
        TRC_REQ_HCI_CMD_CMP_EVT(opcode, p_msg->param_len, p_msg->param);
        hci_env.p_cmd_evt_desc = hci_msg_cmd_desc_get(opcode);
    }
    break;

    case HCI_CMD_STAT_EVENT:
    {
        uint16_t opcode = p_msg->src_id;
        TRC_REQ_HCI_CMD_STAT_EVT(opcode, p_msg->param_len, p_msg->param);
        hci_env.p_cmd_evt_desc = hci_msg_cmd_desc_get(opcode);
    }
    break;

    case HCI_EVENT:
    {
        uint16_t opcode = p_msg->src_id;
        TRC_REQ_HCI_EVT(opcode, p_msg->param_len, p_msg->param);
        ASSERT_INFO(p_msg->src_id <= UINT8_MAX, p_msg->src_id, 0);
        // Look for the event descriptor
        hci_env.p_evt_desc = hci_msg_evt_desc_get(opcode);
        ASSERT_INFO(hci_env.p_evt_desc != NULL, opcode, 0);
    }
    break;

#if (BLE_EMB_PRESENT)
    case HCI_LE_EVENT:
    {
        uint8_t subcode = *((uint8_t *)p_param);
        TRC_REQ_HCI_LE_EVT(subcode, p_msg->param_len, p_msg->param);
        // Look for the event descriptor
        hci_env.p_evt_desc = hci_msg_le_evt_desc_get(subcode);
        ASSERT_INFO(hci_env.p_evt_desc != NULL, subcode, 0);
    }
    break;
#endif // (BLE_EMB_PRESENT)

#if (RW_DEBUG || AUDIO_SYNC_SUPPORT || EAVESDROPPING_SUPPORT)
    case HCI_DBG_EVT:
    {
        uint8_t subcode = *((uint8_t *)p_param);
        hci_env.p_evt_desc = hci_msg_dbg_evt_desc_get(subcode);
        ASSERT_INFO(hci_env.p_evt_desc != NULL, subcode, 0);
    }
    break;
#endif // (RW_DEBUG || AUDIO_SYNC_SUPPORT || 0)

    default: { /* Nothing to do */  } break;
    }

#if (HOST_PRESENT)
#if(EMB_PRESENT && HCI_TL_SUPPORT)
    // check if communication is performed over embedded host
    if (!hci_ext_host)
#endif // (EMB_PRESENT && HCI_TL_SUPPORT)
    {
        uint8_t  host_tl_dest = HOST_UNDEF;
        uint8_t  evt_type = HL_HCI_UNDEF;
        uint16_t opcode = p_msg->src_id;

        // The internal destination first depends on the message type (command, event, data)
        switch (p_msg->id)
        {
        case HCI_CMD_STAT_EVENT:
        {
            evt_type = HL_HCI_CMD_STAT_EVT;
        }
        break;

        case HCI_CMD_CMP_EVENT:
        {
            evt_type = HL_HCI_CMD_CMP_EVT;
        }
        break;

        case HCI_EVENT:
        {
            evt_type = HL_HCI_EVT;
        }
        break;
#if(BLE_HOST_PRESENT)
        case HCI_LE_EVENT:
        {
            opcode = *((uint8_t *)p_param);
            evt_type = HL_HCI_LE_EVT;
        }
        break;
#endif // (BLE_HOST_PRESENT)

#if (RW_DEBUG)
        case HCI_DBG_EVT:
        {
            opcode = *((uint8_t *)p_param);
            evt_type = HL_HCI_DBG_EVT;
        }
        break;
#endif //  (RW_DEBUG)

#if (BT_HOST_PRESENT || HCI_BLE_CON_SUPPORT)
        case HCI_ACL_DATA:
        {
            struct hci_acl_data *p_acl_data = (struct hci_acl_data *) p_param;
            uint16_t conhdl = GETF(p_acl_data->conhdl_pb_bc_flag, HCI_ACL_HDR_HDL);

            evt_type  = HL_HCI_ACL_DATA;
            host_tl_dest = hl_hci_get_host_tl_dest_from_conhdl(conhdl);
        }
        break;
#endif // (BT_HOST_PRESENT || HCI_BLE_CON_SUPPORT)

#if BLE_EMB_PRESENT
#if BLE_ISO_PRESENT
        case HCI_ISO_DATA:
        {
            evt_type = HL_HCI_ISO_DATA;
            host_tl_dest = HOST_TL_UPK;
        }
        break;
#endif //BLE_ISO_PRESENT
#endif //BLE_EMB_PRESENT

#if (BT_EMB_PRESENT)
#if VOICE_OVER_HCI
        case HCI_SYNC_DATA:
        {
            evt_type = HL_HCI_VOHCI_DATA;
            host_tl_dest = HOST_TL_PK;
        }
        break;
#endif //VOICE_OVER_HCI
#endif //(BT_EMB_PRESENT)

        default: {  /* Nothing to do */ } break;
        }

        // Retrieve host local identifier
        if (hci_env.p_cmd_evt_desc != NULL)
        {
            ASSERT_ERR((evt_type == HL_HCI_CMD_STAT_EVT) || (evt_type == HL_HCI_CMD_CMP_EVT));
            host_tl_dest = hl_hci_cmd_evt_get_host_tl_dest(opcode);

#if (!BT_HOST_PRESENT)
            ASSERT_ERR(host_tl_dest == HOST_TL_UPK);
#endif // (!BT_HOST_PRESENT)

            if (host_tl_dest == HOST_TL_UPK)
            {

                uint8_t dest = hci_msg_cmd_ll_dest_get(hci_env.p_cmd_evt_desc);
                hl_hci_send_cmd_evt_to_host(dest, evt_type, opcode, HL_HCI_NOT_TL, p_param);
            }
#if (BT_HOST_PRESENT && HCI_TL_SUPPORT)
            else
            {
                ASSERT_ERR(host_tl_dest == HOST_TL_PK);
                hci_tl_send(p_msg);
            }
#endif // (BT_HOST_PRESENT && HCI_TL_SUPPORT)
        }
        else if (hci_env.p_evt_desc != NULL)
        {
            ASSERT_INFO((evt_type >= HL_HCI_EVT) && (evt_type <= HL_HCI_DBG_EVT), evt_type, opcode);
            host_tl_dest = hci_msg_evt_get_hl_tl_dest(hci_env.p_evt_desc);

            if (host_tl_dest == HOST_UNDEF)
            {
                uint16_t conhdl = p_msg->dest_id;
                host_tl_dest = hl_hci_get_host_tl_dest_from_conhdl(conhdl);
            }

            if (host_tl_dest == HOST_TL_UPK)
            {
                uint8_t host_lid = HL_HCI_INVALID_LID;
                host_lid = hci_msg_evt_host_lid_get(hci_env.p_evt_desc);
                hl_hci_send_evt_to_host(evt_type, (uint8_t)opcode, p_msg->dest_id, HL_HCI_NOT_TL, p_param);
            }
#if (BT_HOST_PRESENT && HCI_TL_SUPPORT)
            else if (host_tl_dest == HOST_TL_PK)
            {
                // Send message through virtual TL
                hci_tl_send(p_msg);
            }
#endif // (BT_HOST_PRESENT && HCI_TL_SUPPORT)
            else
            {
                ASSERT_WARN(0, evt_type, opcode);
                ke_msg_free(p_msg);
            }
        }
        else // Data
        {
            switch (host_tl_dest)
            {
            case HOST_TL_UPK:
            {
#if (BLE_ISOOHCI)
                if (evt_type == HL_HCI_ISO_DATA)
                {
                    const struct hci_iso_data *p_evt_iso_data = (const struct hci_iso_data *)p_param;

                    hl_hci_send_iso_data_to_host(p_param);

#if (BLE_EMB_PRESENT)
                    // Free the indicated RX buffer
                    isoohci_out_buf_release(p_evt_iso_data->iso_data_load.iso_sdu_ptr);
#endif //(BLE_EMB_PRESENT)
                }
                else
#endif //(BLE_ISOOHCI)
                {
                    hl_hci_send_data_to_host(evt_type, p_param, 0);
                }
                break;
#if (BT_HOST_PRESENT && HCI_TL_SUPPORT)
                case HOST_TL_PK:
                {
                    hci_tl_send(p_msg);
                }
                break;
#endif // (BT_HOST_PRESENT && HCI_TL_SUPPORT)
            // ignore message
                default:
                {
                    ASSERT_WARN(0, evt_type, opcode);
                    ke_msg_free(p_msg);
                }
                break;
            }
            }
        }
#if(EMB_PRESENT && HCI_TL_SUPPORT)
        else
#endif // (EMB_PRESENT && HCI_TL_SUPPORT)
#endif // (HOST_PRESENT)
#if (HCI_TL_SUPPORT)
        {
            // Send the HCI message over TL
            hci_tl_send(p_msg);
        }
#endif //(HCI_TL_SUPPORT)
    }
#endif //(EMB_PRESENT)

#if (HOST_PRESENT)
    void hci_send_2_controller(void *param)
    {
        DBG_FUNC_ENTER(hci_send_2_controller);
        struct ke_msg *msg = ke_param2msg(param);

#if (HCI_TL_SUPPORT && !EMB_PRESENT)
        if (msg->id == HCI_ACL_DATA)
        {
            struct hci_acl_data *param1 = (struct hci_acl_data *) param;

            TRC_REQ_HCI_ACL_DATA(1, param1->conhdl_pb_bc_flag, param1->length, (uint32_t *)(param1->buf_ptr));
        }
        else if (msg->id == HCI_PACKED_DATA)
        {
            struct hci_packed_data *pkt = (struct hci_packed_data *)param;

            if (pkt->type == HCI_CMD_MSG_TYPE)
            {
                struct hci_cmd_hdr *hdr = (struct hci_cmd_hdr *)(pkt + 1);
                TRC_REQ_HCI_CMD(hdr->opcode, hdr->parlen, (uint32_t const *)((uint8_t *)hdr + HCI_CMD_HDR_LEN));
            }
            else if (pkt->type == HCI_ACL_MSG_TYPE)
            {
                struct hci_acl_hdr *hdr = (struct hci_acl_hdr *)(pkt + 1);
                TRC_REQ_HCI_ACL_DATA(1, hdr->hdl_flags, hdr->datalen, (uint32_t const *)(hdr + 1));
            }

        }
        // Send the HCI message over TL
        hci_tl_send(msg);
#else //(HCI_TL_SUPPORT && !EMB_PRESENT)

#if(EMB_PRESENT && HCI_TL_SUPPORT)
        // check if communication is performed over embedded host
        if (!hci_ext_host)
#endif // (EMB_PRESENT && HCI_TL_SUPPORT)
        {
            ke_task_id_t dest = TASK_NONE;
            uint8_t ll_dest = LL_UNDEF;

            // The internal destination first depends on the message type (command, event, data)
            switch (msg->id)
            {
            case HCI_COMMAND:
            {
                TRC_REQ_HCI_CMD(msg->src_id, msg->param_len, msg->param);

                // Find a command descriptor associated to the command opcode
                const hci_cmd_desc_t *p_cmd_desc = hci_msg_cmd_desc_get(msg->src_id);

                // Check if the command is supported
                if (p_cmd_desc != NULL)
                {
                    ll_dest =  hci_msg_cmd_ll_dest_get(p_cmd_desc);
                }
            }
            break;
#if (HCI_BLE_CON_SUPPORT || BT_EMB_PRESENT)
            case HCI_ACL_DATA:
            {
                ll_dest = CTRL;
            }
            break;
#endif // (HCI_BLE_CON_SUPPORT || BT_EMB_PRESENT)

            default: { /* Nothing to do */ } break;
            }

            // Compute LL task destination
            dest = hci_msg_task_dest_compute(ll_dest, msg->param_len, (const uint8_t *) param);

            // Check it the destination has been found
            if (dest != TASK_NONE)
            {
                // Send the command to the internal destination task associated to this command
                msg->dest_id = dest;
                ke_msg_send(param);
            }
            else
            {
                ASSERT_WARN(0, msg->id, msg->src_id);

                // Free message to avoid memory leak
                ke_msg_free(msg);
            }
        }
#if(BLE_EMB_PRESENT && HCI_TL_SUPPORT)
        else
        {
            // receiving a message from internal host is not expected at all
            ASSERT_ERR(0);
            // Free message to avoid memory leak
            ke_msg_free(msg);
        }
#endif // (BLE_EMB_PRESENT && HCI_TL_SUPPORT)
#endif //(HCI_TL_SUPPORT)

        DBG_FUNC_EXIT(hci_send_2_controller);
    }


    void hci_basic_cmd_send_2_controller(uint16_t opcode)
    {
        //TODO: Check the details.
        void *no_param = ke_msg_alloc(HCI_COMMAND, 0, opcode, 0);
        hci_send_2_controller(no_param);
    }

#if (BT_HOST_PRESENT)
    void hci_send_pk_2_controller(uint8_t type, uint16_t length, uint8_t *p_buf, hci_done_cb cb_done)
    {
#if(EMB_PRESENT)
        // send packet
        switch (type)
        {
        case HCI_CMD_MSG_TYPE :
        {
            uint16_t opcode;
            uint8_t param_len;

            // Get the opcode, length and adjust payload
            opcode    = co_read16p(&p_buf[0]);
            param_len = p_buf[HCI_CMD_OPCODE_LEN];

            ASSERT_ERR(param_len == (length - HCI_CMD_HDR_LEN));

            // Send message through virtual transport layer
            if (param_len <= hci_tl_cmd_get_max_param_size(opcode))
            {
                hci_tl_cmd_received(HCI_TL_VIRTUAL, opcode, param_len, p_buf + HCI_CMD_HDR_LEN);
            }
            else
            {
                ASSERT_ERR(0);
            }
        }
        break;

        case HCI_ACL_MSG_TYPE :
        {
            uint8_t  *p_em_buf = NULL;
            uint16_t  conhdl_pb_bc_flag;
            uint16_t  datalen;

            conhdl_pb_bc_flag = co_read16p(&p_buf[0]);
            datalen = co_read16p(&p_buf[2]);

            // allocate buffer memory & copy HCI ACL Packet Data into buffer
            if (datalen > 0)
            {
                p_em_buf = hci_tl_acl_tx_data_alloc(HCI_TL_VIRTUAL, conhdl_pb_bc_flag, datalen);
                ASSERT_ERR(p_em_buf != NULL);
                if (p_em_buf != NULL)
                {
                    em_wr(&p_buf[4], em_addr_get(p_em_buf), datalen);
                } // else out of sync
            }

            hci_tl_acl_tx_data_received(HCI_TL_VIRTUAL, conhdl_pb_bc_flag, datalen, p_em_buf);
        }
        break;
#if (VOICE_OVER_HCI)
        case HCI_SYNC_MSG_TYPE :
        {
            uint8_t  *p_em_buf = NULL;
            uint16_t  conhdl_flag;
            uint8_t  datalen;

            conhdl_flag = co_read16p(&p_buf[0]);
            datalen = p_buf[2];

            // allocate buffer memory & copy HCI ACL Packet Data into buffer
            if (datalen > 0)
            {
                p_em_buf = hci_tl_sync_tx_data_alloc(HCI_TL_VIRTUAL, conhdl_flag, datalen);
                ASSERT_ERR(p_em_buf != NULL);
                if (p_em_buf != NULL)
                {
                    em_wr(&p_buf[3], em_addr_get(p_em_buf), datalen);
                } // else out of sync
            }

            hci_tl_sync_tx_data_received(HCI_TL_VIRTUAL, conhdl_flag, datalen, p_em_buf);
        }
        break;
#endif // (VOICE_OVER_HCI)

        default :
        {
            ASSERT_ERR(0);
        }
        break;
        }

        // Inform that packet has been processed
        if (cb_done)
            cb_done();
#else // !(EMB_PRESENT)
#if 0
        // Directly ask HCI_TL module to send the HCI message (already packed)
        hci_tl_pk_send(type, length, p_buf, cb_done);
#else
        struct hci_packed_data *pkt = ke_msg_alloc(HCI_PACKED_DATA, TASK_COMMON, 0, length + sizeof(struct hci_packed_data));
        uint8_t *p;
        pkt->type = type;
        pkt->length = length;
        pkt->reserved = 0;
        p = (uint8_t *)(pkt + 1);
        memcpy(p, p_buf, length);
        if (type == HCI_CMD_MSG_TYPE)
        {
            uint16_t opcode;

            opcode = (uint16_t) * (p + 1);
            opcode <<= 8;
            opcode += *p;
            hl_hci_mark_host_bt(opcode);
        }
        //hci_send_2_controller(pkt);
        ke_msg_send((void const *)pkt);
#endif


#endif // (EMB_PRESENT)
    }
#endif // (BT_HOST_PRESENT)
#endif // (HOST_PRESENT)

#if (BLE_EMB_PRESENT && HCI_BLE_CON_SUPPORT)
    void hci_ble_conhdl_register(uint8_t link_id)
    {
        ASSERT_INFO(link_id < BLE_ACTIVITY_MAX, link_id, BLE_ACTIVITY_MAX);
        ASSERT_ERR(!hci_env.ble_con_state[link_id]);

        // Link ID associated with BD address AND connection handle
        hci_env.ble_con_state[link_id] = true;
    }

    void hci_ble_conhdl_unregister(uint8_t link_id)
    {
        ASSERT_INFO(link_id < BLE_ACTIVITY_MAX, link_id, BLE_ACTIVITY_MAX);
        ASSERT_ERR(hci_env.ble_con_state[link_id]);

        // Link ID associated with BD address
        hci_env.ble_con_state[link_id] = false;
    }
#endif //(BLE_EMB_PRESENT && HCI_BLE_CON_SUPPORT)

#if (BT_EMB_PRESENT)
    void hci_bt_acl_bdaddr_register(uint8_t link_id, struct bd_addr * bd_addr)
    {
        ASSERT_INFO(link_id < MAX_NB_ACTIVE_ACL, link_id, 0);
        ASSERT_INFO(hci_env.bt_acl_con_tab[link_id].state == HCI_BT_ACL_STATUS_NOT_ACTIVE, hci_env.bt_acl_con_tab[link_id].state, 0);

        // Store BD address
        memcpy(&hci_env.bt_acl_con_tab[link_id].bd_addr.addr[0], &bd_addr->addr[0], sizeof(struct bd_addr));

        // Link ID associated with BD address
        hci_env.bt_acl_con_tab[link_id].state = HCI_BT_ACL_STATUS_BD_ADDR;
    }

    void hci_bt_acl_conhdl_register(uint8_t link_id)
    {
        ASSERT_INFO(link_id < MAX_NB_ACTIVE_ACL, link_id, 0);
        ASSERT_INFO(hci_env.bt_acl_con_tab[link_id].state == HCI_BT_ACL_STATUS_BD_ADDR, hci_env.bt_acl_con_tab[link_id].state, 0);

        // Link ID associated with BD address AND connection handle
        hci_env.bt_acl_con_tab[link_id].state = HCI_BT_ACL_STATUS_BD_ADDR_CONHDL;
    }

    void hci_bt_acl_bdaddr_unregister(uint8_t link_id)
    {
        ASSERT_INFO(link_id < MAX_NB_ACTIVE_ACL, link_id, 0);

        // Link ID associated with BD address
        hci_env.bt_acl_con_tab[link_id].state = HCI_BT_ACL_STATUS_NOT_ACTIVE;
    }
#endif //(BT_EMB_PRESENT)

    uint8_t hci_evt_mask_set(struct evt_mask const * evt_msk, uint8_t page)
    {
        switch (page)
        {
        case HCI_PAGE_0:
        case HCI_PAGE_1:
        {
            ASSERT_INFO(page == HCI_PAGE_DFT, page, page);
        }
        break;
        case HCI_PAGE_2:
        {
            // Store event mask
            memcpy(&hci_env.evt_msk_page_2.mask[0], &evt_msk->mask[0], EVT_MASK_LEN);
        }
        break;
        case HCI_PAGE_DFT:
        {
            // Store event mask
            memcpy(&hci_env.evt_msk.mask[0], &evt_msk->mask[0], EVT_MASK_LEN);

            // ensure that reserved bit are set
            for (uint8_t i = 0; i < EVT_MASK_LEN; i++)
            {
                hci_env.evt_msk.mask[i] |= hci_rsvd_evt_msk.mask[i];
            }
        }
        break;
#if (BLE_EMB_PRESENT)
        case HCI_PAGE_LE:
        {
            // Store event mask
            memcpy(&hci_env.le_evt_msk.mask[0], &evt_msk->mask[0], EVT_MASK_LEN);
        }
        break;
#endif // (BLE_EMB_PRESENT)
        default:
        {
            ASSERT_ERR_FORCE(0);
        }
        break;
        }
        return CO_ERROR_NO_ERROR;
    }

    void hci_con_accept_to_set(uint16_t con_accept_to)
    {
        hci_env.con_accept_to = con_accept_to;
    }

    uint16_t hci_con_accept_to_get(void)
    {
        return hci_env.con_accept_to;
    }

#if (BT_EMB_PRESENT)
    uint8_t hci_evt_filter_add(struct hci_set_evt_filter_cmd const * param)
    {
        uint8_t status = CO_ERROR_INVALID_HCI_PARAM;

        // Perform the requested action according to the filter type
        switch (param->filter_type)
        {
        case CLEAR_ALL_FILTER_TYPE:
        {
            // Reset all Filters
            status = hci_evt_filter_reset();
        }
        break;

        case INQUIRY_FILTER_TYPE:
        {
            // Add inquiry event filter
            status = hci_evt_filter_inq_add(&param->filter.inq_res);
        }
        break;

        case CONNECTION_FILTER_TYPE:
        {
            // Add connection event filter
            status = hci_evt_filter_con_add(&param->filter.con_set);
        }
        break;

        default:
        {
            // Nothing to do
        }
        break;
        }

        return (status);
    }

#if (MAX_NB_SYNC > 0)
    uint16_t hci_voice_settings_get(void)
    {
        return (hci_env.voice_settings);
    }

    uint8_t hci_voice_settings_set(uint16_t voice_settings)
    {
        uint8_t status = CO_ERROR_UNSUPPORTED;

        // Check if the requested voice settings are supported
        switch (voice_settings & AIR_COD_MSK)
        {
        case AIR_COD_CVSD:
        case AIR_COD_MULAW:
        case AIR_COD_ALAW:
        case AIR_COD_TRANS:
        {
            hci_env.voice_settings = voice_settings;
            status = CO_ERROR_NO_ERROR;
        }
        break;
        default:
            break;
        }

        return (status);
    }
#endif // (MAX_NB_SYNC > 0)
#endif //(BT_EMB_PRESENT)

#endif //(HCI_PRESENT)

/// @} HCI
