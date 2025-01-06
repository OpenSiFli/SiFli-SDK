/**
 ****************************************************************************************
 *
 * @file llm_scan.c
 *
 * @brief Definition of the functions used by the link layer manager
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup LLM
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"        // stack configuration

#if (BLE_OBSERVER)

#include <string.h>

#if (EAVESDROPPING_SUPPORT)
    #include <limits.h>            // integer limits
#endif //(EAVESDROPPING_SUPPORT)

#include "co_bt.h"              // BLE standard definitions
#include "co_utils.h"
#include "co_math.h"
#include "ble_util.h"           // BLE utility functions

#include "ke_mem.h"             // kernel memory
#include "ke_task.h"            // kernel task management
#include "ke_timer.h"           // kernel timer

#include "llm.h"                // link layer manager definitions
#include "lld.h"                // link layer driver definitions

#include "llm_int.h"            // link layer manager internal definitions

#include "rwip.h"

#include "em_map.h"             // EM Map
#include "reg_em_ble_ral.h"     // BLE RAL structures

#if HCI_PRESENT
    #include "hci.h"                // host controller interface
#endif //HCI_PRESENT

#include "dbg.h"
#if BLE_CONLESS_CTE_RX
    #include "reg_em_ble_rx_cte_desc.h"
#endif // BLE_CONLESS_CTE_RX


/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * CONSTANTS DEFINITION
 *****************************************************************************************
 */

#if (BLE_ADV_LEGACY_ITF)
/// Conversion table from advertising event properties values for legacy PDUs to legacy event types
__STATIC const uint8_t adv_evt_prop2type[] =
{
    [ADV_IND] = ADV_IND_EVT,
    [ADV_DIRECT_IND] = ADV_DIRECT_IND_EVT,
    [ADV_NONCONN_IND] = ADV_NONCONN_IND_EVT,
    [SCAN_RSP_TO_ADV_IND] = SCAN_RSP_EVT,
    [SCAN_RSP_TO_ADV_SCAN_IND] = SCAN_RSP_EVT,
    [ADV_SCAN_IND] = ADV_SCAN_IND_EVT
};
#endif //(BLE_ADV_LEGACY_ITF)


/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */

/// Finds the activity that is under periodic sync establishment (invalid index if not found)
__STATIC uint8_t llm_activity_syncing_get(void)
{
    uint8_t act_id = 0;

    // If not present, try to allocate an activity identifier
    for (act_id = 0; act_id < BLE_ACTIVITY_MAX_CFG; act_id++)
    {
        if (llm_env.act_info[act_id].state == LLM_PER_SCAN_SYNCING)
            break;
    }

    return (act_id);
}

/// Start scanning
__STATIC uint8_t llm_scan_start(uint8_t act_id)
{
    struct hci_le_set_ext_scan_param_cmd *host_params = (struct hci_le_set_ext_scan_param_cmd *) llm_env.act_info[act_id].host_params;
    struct lld_scan_params scan_par;
    int i = 0; // phys index

    scan_par.scan_phys = host_params->scan_phys;
    scan_par.win_1m = 0;
    scan_par.win_c = 0;

    if (scan_par.scan_phys & PHY_1MBPS_BIT)
    {
        scan_par.type_1m = host_params->phy[0].scan_type;
        scan_par.intv_1m = host_params->phy[0].scan_intv;
        scan_par.win_1m = host_params->phy[0].scan_window;

        i++;
    }

    if (scan_par.scan_phys & PHY_CODED_BIT)
    {
        scan_par.type_c = host_params->phy[i].scan_type;
        scan_par.intv_c = host_params->phy[i].scan_intv;
        scan_par.win_c = host_params->phy[i].scan_window;
    }

    /*
     * Adjustment for multiple scans. The Scan_Interval[i] and Scan_Window[i] parameters are recommendations.
     * The actual values used are implementation specific. HCI:7.8.64.
     */
    if (scan_par.scan_phys == (PHY_1MBPS_BIT | PHY_CODED_BIT))
    {
        uint32_t agg_win = scan_par.win_1m + scan_par.win_c;

        if (scan_par.intv_1m < agg_win)
        {
            scan_par.intv_1m = agg_win;
        }

        if (scan_par.intv_c < agg_win)
        {
            scan_par.intv_c = agg_win;
        }
    }

    scan_par.own_addr_type = host_params->own_addr_type;
    scan_par.filter_policy = host_params->scan_filt_policy;
    scan_par.ext_scan = llm_env.ext_scan;
    scan_par.duration = llm_env.act_info[act_id].info.scan.duration;
    scan_par.addr_resolution_en = llm_env.addr_resolution_en;

    // Get own address of addr type
    switch (scan_par.own_addr_type)
    {
    case  ADDR_PUBLIC:
    case  ADDR_RPA_OR_PUBLIC:
    {
        // Get local public address
        scan_par.own_addr = llm_env.local_pub_addr;
    }
    break;
    case  ADDR_RAND:
    case  ADDR_RPA_OR_RAND:
    {
        // Get local random address
        scan_par.own_addr = llm_env.local_rand_addr;
    }
    break;
    default:
    {
        ASSERT_INFO_FORCE(0, scan_par.own_addr_type, 0);
    }
    break;
    }

    // Start scanning
    return lld_scan_start(act_id, &scan_par);
}

#if (BLE_BIS)
/**
 ****************************************************************************************
 * @brief Indicates that the Controller has received an Advertising PDU that contained a BIGInfo field
 *
 * @param[in] sync_hdl        Sync_Handle identifying the periodic advertising train
 * @param[in] p_big_info      Pointer to the BIGInfo
 ****************************************************************************************
 */
__STATIC void llm_big_info_adv_report_evt_send(uint16_t sync_hdl, struct big_info *p_big_info)
{
    // Prepare the event
    struct hci_le_big_info_adv_report_evt *p_evt;
    // Allocate message to return
    p_evt                  = KE_MSG_ALLOC(HCI_LE_EVENT, 0, HCI_LE_META_EVT_CODE, hci_le_big_info_adv_report_evt);
    p_evt->subcode         = HCI_LE_BIG_INFO_ADV_REPORT_EVT_SUBCODE;
    p_evt->sync_hdl        = sync_hdl;
    p_evt->num_bis         = p_big_info->num_bis;
    p_evt->nse             = p_big_info->nse;
    p_evt->iso_interval    = p_big_info->iso_interval;
    p_evt->bn              = p_big_info->bn;
    p_evt->pto             = p_big_info->pto;
    p_evt->irc             = p_big_info->irc;
    p_evt->max_pdu         = p_big_info->max_pdu;
    p_evt->sdu_interval[0] = p_big_info->sdu_interval;
    p_evt->sdu_interval[1] = p_big_info->sdu_interval >> 8;
    p_evt->sdu_interval[2] = p_big_info->sdu_interval >> 16;
    p_evt->max_sdu         = p_big_info->max_sdu;
    p_evt->phy             = co_phy_mask_to_value[p_big_info->phy];
    p_evt->framing         = p_big_info->framing;
    p_evt->encryption      = p_big_info->encrypted;

    // Send the event
    hci_send_2_host(p_evt);
}
#endif // (BLE_BIS)


/*
 * HCI COMMAND HANDLERS
 ****************************************************************************************
 */

#if (BLE_ADV_LEGACY_ITF)
int hci_le_set_scan_param_cmd_handler(struct hci_le_set_scan_param_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    do
    {
        uint8_t act_id;

        // Check current interface version
        if (llm_env.adv_itf_version == LLM_ADV_ITF_EXTENDED)
            break;

        // Use legacy interface
        llm_env.adv_itf_version = LLM_ADV_ITF_LEGACY;

        // Look for the scanning activity
        for (act_id = 0; act_id < BLE_ACTIVITY_MAX_CFG; act_id++)
        {
            if ((llm_env.act_info[act_id].state == LLM_SCAN_RSVD) || (llm_env.act_info[act_id].state == LLM_SCAN_EN) || (llm_env.act_info[act_id].state == LLM_SCAN_STOPPING))
                break;
        }

        // Check if scanning has been found
        if (act_id < BLE_ACTIVITY_MAX_CFG)
        {
            // If scanning is already enabled, reject the command
            if (llm_env.act_info[act_id].state != LLM_SCAN_RSVD)
                break;
        }
        else
        {
            // If not present, try to allocate an activity identifier
            status = llm_activity_free_get(&act_id);

            // If not possible to start a new advertising activity, reject the command
            if (status != CO_ERROR_NO_ERROR)
            {
                break;
            }

            // Assign the allocated activity identifier to legacy advertising
            llm_env.act_info[act_id].state = LLM_SCAN_RSVD;
        }

        status = CO_ERROR_INVALID_HCI_PARAM;

        // Check provided parameters: scan_window, scan_intv
        if ((param->scan_window > param->scan_intv) || (param->scan_type > SCAN_ACTIVE) || (param->scan_window > SCAN_WINDOW_MAX) ||
                (param->scan_window < SCAN_WINDOW_MIN) || (param->scan_intv > SCAN_INTERVAL_MAX) || (param->scan_intv < SCAN_INTERVAL_MIN))
        {
            break;
        }

        // Check provided parameters: address type, filter policy
        if ((param->own_addr_type > ADDR_RPA_OR_RAND) || (param->scan_filt_policy > SCAN_ALLOW_ADV_WLST_AND_INIT_RPA))
        {
            break;
        }

        // Check privacy -> scan with filter policy 2 & 3 RAL must be enabled
        if (!llm_env.addr_resolution_en && (param->scan_filt_policy > SCAN_ALLOW_ADV_WLST))
        {
            break;
        }

        // Allocate an extended scan params HCI command if needed
        if (llm_env.act_info[act_id].host_params == NULL)
        {
            llm_env.act_info[act_id].host_params = KE_MSG_ALLOC(0, 0, 0, hci_le_set_ext_scan_param_cmd);
        }

        // Convert legacy HCI command to extended format
        {
            struct hci_le_set_ext_scan_param_cmd *ext_param = (struct hci_le_set_ext_scan_param_cmd *) llm_env.act_info[act_id].host_params;
            ext_param->scan_phys            = PHY_1MBPS_BIT;
            ext_param->phy[0].scan_type     = param->scan_type;
            ext_param->phy[0].scan_intv     = param->scan_intv;
            ext_param->phy[0].scan_window   = param->scan_window;
            ext_param->own_addr_type        = param->own_addr_type;
            ext_param->scan_filt_policy     = param->scan_filt_policy;
        }

        // Assign the allocated activity identifier to scanning
        llm_env.act_info[act_id].state = LLM_SCAN_RSVD;

        status = CO_ERROR_NO_ERROR;

    }
    while (0);

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

int hci_le_set_scan_en_cmd_handler(struct hci_le_set_scan_en_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;
    bool stopping = false;

    // Check current interface version
    if (llm_env.adv_itf_version != LLM_ADV_ITF_EXTENDED)
    {
        uint8_t act_id;

        // Use legacy interface
        llm_env.adv_itf_version = LLM_ADV_ITF_LEGACY;
        // Extended scanning shall be disabled
        llm_env.ext_scan = false;

        // Look for the scanning activity
        for (act_id = 0; act_id < BLE_ACTIVITY_MAX_CFG; act_id++)
        {
            if ((llm_env.act_info[act_id].state == LLM_SCAN_RSVD) || (llm_env.act_info[act_id].state == LLM_SCAN_EN) || (llm_env.act_info[act_id].state == LLM_SCAN_STOPPING))
                break;
        }

        // Check parameter
        switch (param->scan_en)
        {
        case SCAN_DIS:
        {
            // Verify that the activity ID is valid
            if (act_id >= BLE_ACTIVITY_MAX_CFG)
                break;

            // If scanning is not enabled
            if (llm_env.act_info[act_id].state != LLM_SCAN_EN)
            {
                status = CO_ERROR_NO_ERROR;
                break;
            }

            // Stop scanning
            status = lld_scan_stop();

            if (status == CO_ERROR_NO_ERROR)
            {
                llm_env.act_info[act_id].state = LLM_SCAN_STOPPING;
                stopping = true;
            }
        }
        break;

        case SCAN_ENABLE:
        {
            struct hci_le_set_ext_scan_param_cmd *host_params;

            status = CO_ERROR_INVALID_HCI_PARAM;

            // Check filter duplicates
            if (param->filter_duplic > SCAN_FILT_DUPLIC_EN)
                break;

            // Check if scanning is present
            if (act_id < BLE_ACTIVITY_MAX_CFG)
            {
                // Cannot be enabled if Host has requested to stop
                if (llm_env.act_info[act_id].state == LLM_SCAN_STOPPING)
                {
                    status = CO_ERROR_COMMAND_DISALLOWED;
                    break;
                }

                // Check if already enabled
                else if (llm_env.act_info[act_id].state == LLM_SCAN_EN)
                {
                    // Any change to the Filter_Duplicates setting shall take effect.
                    SETB(llm_env.adv_dup_filt_info, LLM_ADV_DUP_FILT_EN, (param->filter_duplic != SCAN_FILT_DUPLIC_DIS));
                    status = CO_ERROR_NO_ERROR;
                    break;
                }
            }
            else
            {
                // Allocate activity identifier
                status = llm_activity_free_get(&act_id);

                // If not possible to start a new advertising activity, reject the command
                if (status != CO_ERROR_NO_ERROR)
                {
                    break;
                }
            }

            // Check if parameters have been provided from the Host
            if (llm_env.act_info[act_id].host_params == NULL)
            {
                struct hci_le_set_ext_scan_param_cmd *ext_param;

                // Allocate extended HCI command for storing default parameters
                llm_env.act_info[act_id].host_params = KE_MSG_ALLOC(0, 0, 0, hci_le_set_ext_scan_param_cmd);

                // Fill with default parameters
                ext_param = (struct hci_le_set_ext_scan_param_cmd *) llm_env.act_info[act_id].host_params;
                ext_param->scan_phys            = PHY_1MBPS_BIT;
                ext_param->phy[0].scan_type     = SCAN_PASSIVE;
                ext_param->phy[0].scan_intv     = SCAN_INTERVAL_DFT;
                ext_param->phy[0].scan_window   = SCAN_WINDOW_DFT;
                ext_param->own_addr_type        = ADDR_PUBLIC;
                ext_param->scan_filt_policy     = SCAN_ALLOW_ADV_ALL;

                // Indicate the activity as reserved
                llm_env.act_info[act_id].state = LLM_SCAN_RSVD;
            }

            // Fetch parameters
            host_params = (struct hci_le_set_ext_scan_param_cmd *) llm_env.act_info[act_id].host_params;

            /*
             * If the scanning parameters' Own_Address_Type parameter is set to 0x1 or 0x03 and the random address for the
             * device has not been initialized, the Controller shall return the error code Invalid HCI Command
             * Parameters (0x12).
             */
            if (((host_params->own_addr_type == ADDR_RAND) || (host_params->own_addr_type == ADDR_RPA_OR_RAND))
                    && co_bdaddr_compare(&llm_env.local_rand_addr, &co_null_bdaddr))
                break;

            /*
             * If the scanning parameters' Own_Address_Type parameter is set to 0x3, the controller's resolving list
             * did not contain a matching entry, and the random address for the device has not been initialized, the
             * Controller shall return the error code Invalid HCI Command Parameters (0x12).
             */
            if ((host_params->own_addr_type == ADDR_RPA_OR_RAND)
                    && llm_ral_is_empty()
                    && co_bdaddr_compare(&llm_env.local_rand_addr, &co_null_bdaddr))
                break;

            // Save parameters
            SETB(llm_env.adv_dup_filt_info, LLM_ADV_DUP_FILT_EN, (param->filter_duplic != SCAN_FILT_DUPLIC_DIS));
            llm_env.act_info[act_id].info.scan.duration = 0;
            llm_env.act_info[act_id].info.scan.period   = 0;

            // Start scanning
            status = llm_scan_start(act_id);
            llm_env.act_info[act_id].state = LLM_SCAN_EN;
        }
        break;

        default:
        {
            status = CO_ERROR_INVALID_HCI_PARAM;
        }
        break;
        }
    }

    // If scanning is stopping, wait the indication from the driver before reporting the command complete event
    if (!stopping)
    {
        // Send the command complete event
        llm_cmd_cmp_send(opcode, status);
    }

    return (KE_MSG_CONSUMED);
}
#endif //(BLE_ADV_LEGACY_ITF)

int hci_le_set_ext_scan_param_cmd_handler(struct hci_le_set_ext_scan_param_cmd const *param, uint16_t opcode)
{
    uint8_t ret_status = KE_MSG_CONSUMED;
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    do
    {
        uint8_t act_id;
        uint8_t num_scan_phys;
        int i;

#if (BLE_ADV_LEGACY_ITF)
        // Check current interface version
        if (llm_env.adv_itf_version == LLM_ADV_ITF_LEGACY)
            break;

        // Use extended interface
        llm_env.adv_itf_version = LLM_ADV_ITF_EXTENDED;
#endif //(BLE_ADV_LEGACY_ITF)

        // Look for the scanning activity
        for (act_id = 0; act_id < BLE_ACTIVITY_MAX_CFG; act_id++)
        {
            if ((llm_env.act_info[act_id].state == LLM_SCAN_RSVD) || (llm_env.act_info[act_id].state == LLM_SCAN_EN) || (llm_env.act_info[act_id].state == LLM_SCAN_STOPPING))
                break;
        }

        // Check if scanning has been found
        if (act_id < BLE_ACTIVITY_MAX_CFG)
        {
            // If scanning is already enabled, reject the command
            if (llm_env.act_info[act_id].state != LLM_SCAN_RSVD)
                break;
        }
        else
        {
            // If not present, try to allocate an activity identifier
            status = llm_activity_free_get(&act_id);

            // If not possible to start a new advertising activity, reject the command
            if (status != CO_ERROR_NO_ERROR)
            {
                break;
            }
        }

#if !(BLE_PHY_CODED_SUPPORT)
        if (PHY_CODED_BIT & param->scan_phys)
        {
            status = CO_ERROR_UNSUPPORTED;
            break;
        }
#endif

        // If at least one of the RFU bits is set
        if (param->scan_phys > (PHY_1MBPS_BIT | PHY_2MBPS_BIT | PHY_CODED_BIT))
        {
            status = CO_ERROR_UNSUPPORTED;
            break;
        }

        status = CO_ERROR_INVALID_HCI_PARAM;

        // Check provided parameters: scan_phys
        if ((PHY_1MBPS_BIT == param->scan_phys) || (PHY_CODED_BIT == param->scan_phys))
        {
            num_scan_phys = 1;
        }
        else if ((PHY_1MBPS_BIT | PHY_CODED_BIT) == param->scan_phys)
        {
            num_scan_phys = 2;
        }
        else
        {
            break;
        }

        // Check provided parameters: scan_window, scan_intv, scan_type
        for (i = 0; i < num_scan_phys; i++)
        {
            if ((param->phy[i].scan_window > param->phy[i].scan_intv) || (param->phy[i].scan_type > SCAN_ACTIVE) ||
                    (param->phy[i].scan_window < EXT_SCAN_WINDOW_MIN) || (param->phy[i].scan_intv < EXT_SCAN_INTERVAL_MIN))
            {
                break;
            }
        }

        if (i < num_scan_phys)
        {
            break;
        }

        // Check provided parameters: address type, filter policy
        if ((param->own_addr_type > ADDR_RPA_OR_RAND) || (param->scan_filt_policy > SCAN_ALLOW_ADV_WLST_AND_INIT_RPA))
        {
            break;
        }

        // Free potentially stored parameters
        if (llm_env.act_info[act_id].host_params != NULL)
        {
            ke_msg_free(ke_param2msg(llm_env.act_info[act_id].host_params));
            llm_env.act_info[act_id].host_params = NULL;
        }

        // Keep pointer to parameters
        llm_env.act_info[act_id].host_params = param;

        // Assign the allocated activity identifier to scannning
        llm_env.act_info[act_id].state = LLM_SCAN_RSVD;

        // Message needs to be maintained
        ret_status = KE_MSG_NO_FREE;

        status = CO_ERROR_NO_ERROR;

    }
    while (0);

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (ret_status);
}

int hci_le_set_ext_scan_en_cmd_handler(struct hci_le_set_ext_scan_en_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;
    bool stopping = false;

#if (BLE_ADV_LEGACY_ITF)
    // Check current interface version
    if (llm_env.adv_itf_version != LLM_ADV_ITF_LEGACY)
#endif //(BLE_ADV_LEGACY_ITF)
    {
        uint8_t act_id;

#if (BLE_ADV_LEGACY_ITF)
        // Use extended interface
        llm_env.adv_itf_version = LLM_ADV_ITF_EXTENDED;
#endif //(BLE_ADV_LEGACY_ITF)

        // Look for the scanning activity
        for (act_id = 0; act_id < BLE_ACTIVITY_MAX_CFG; act_id++)
        {
            if ((llm_env.act_info[act_id].state == LLM_SCAN_RSVD) || (llm_env.act_info[act_id].state == LLM_SCAN_EN) || (llm_env.act_info[act_id].state == LLM_SCAN_STOPPING))
                break;
        }

        // Check parameter
        switch (param->scan_en)
        {
        case SCAN_DIS:
        {
            // Verify that the activity ID is valid
            if (act_id >= BLE_ACTIVITY_MAX_CFG)
                break;

            // If scanning is not enabled
            if (llm_env.act_info[act_id].state != LLM_SCAN_EN)
            {
                status = CO_ERROR_NO_ERROR;
                break;
            }

            // Stop scanning
            status = lld_scan_stop();

            if (status == CO_ERROR_NO_ERROR)
            {
                // Wait scan driver to close scan
                llm_env.act_info[act_id].state = LLM_SCAN_STOPPING;
                stopping = true;
            }
            else
            {
                // Scan driver already closed
                llm_env.act_info[act_id].state = LLM_SCAN_RSVD;

                // Clear timer of next scan period
                ke_timer_clear(LLM_SCAN_PERIOD_TO, TASK_LLM);

                status = CO_ERROR_NO_ERROR;
            }
        }
        break;

        case SCAN_ENABLE:
        {
            struct hci_le_set_ext_scan_param_cmd *host_params;

            status = CO_ERROR_INVALID_HCI_PARAM;

            // Check filter duplicates
            if (param->filter_duplic > SCAN_FILT_DUPLIC_EN_PER_PERIOD)
                break;

            // If Filter_Duplicates is set to 0x02 and either Period or Duration to zero
            if ((SCAN_FILT_DUPLIC_EN_PER_PERIOD == param->filter_duplic) && ((param->period == 0) || (param->duration == 0)))
                break;

            if ((param->period != 0) && (param->duration >= (param->period * 128)))
                break;

            // Check if scanning is present
            if (act_id < BLE_ACTIVITY_MAX_CFG)
            {
                // Cannot be enabled if Host has requested to stop
                if (llm_env.act_info[act_id].state == LLM_SCAN_STOPPING)
                {
                    status = CO_ERROR_COMMAND_DISALLOWED;
                    break;
                }

                // Check if already enabled
                else if (llm_env.act_info[act_id].state == LLM_SCAN_EN)
                {
                    ASSERT_ERR(NULL != llm_env.act_info[act_id].host_params);

                    // Any change to the random address shall take effect.
                    host_params = (struct hci_le_set_ext_scan_param_cmd *) llm_env.act_info[act_id].host_params;
                    struct bd_addr *bd_addr = (host_params->own_addr_type & ADDR_RPA_MASK) ?
                                              &llm_env.local_rand_addr : NULL;

                    // Timers used for duration and period are reset to the new parameter values and a new scan period is started.
                    lld_scan_params_update(param->duration, bd_addr);

                    // Any change to the Filter_Duplicates setting shall take effect
                    SETB(llm_env.adv_dup_filt_info, LLM_ADV_DUP_FILT_EN, (param->filter_duplic != SCAN_FILT_DUPLIC_DIS));
                    SETB(llm_env.adv_dup_filt_info, LLM_ADV_DUP_FILT_RST, (param->filter_duplic == SCAN_FILT_DUPLIC_EN_PER_PERIOD));
                    llm_env.act_info[act_id].info.scan.duration = param->duration;
                    llm_env.act_info[act_id].info.scan.period   = param->period;
                    status = CO_ERROR_NO_ERROR;
                    break;
                }
            }
            else
            {
                // Allocate activity identifier
                status = llm_activity_free_get(&act_id);

                // If not possible to start a new advertising activity, reject the command
                if (status != CO_ERROR_NO_ERROR)
                {
                    break;
                }
            }

            // Check if parameters have been provided from the Host
            if (llm_env.act_info[act_id].host_params == NULL)
            {
                struct hci_le_set_ext_scan_param_cmd *ext_param;

                // Allocate extended HCI command for storing default parameters
                llm_env.act_info[act_id].host_params = KE_MSG_ALLOC(0, 0, 0, hci_le_set_ext_scan_param_cmd);

                // Fill with default parameters
                ext_param = (struct hci_le_set_ext_scan_param_cmd *) llm_env.act_info[act_id].host_params;
                ext_param->scan_phys = PHY_1MBPS_BIT;
                ext_param->phy[0].scan_type = SCAN_PASSIVE;
                ext_param->phy[0].scan_intv = SCAN_INTERVAL_DFT;
                ext_param->phy[0].scan_window = SCAN_WINDOW_DFT;
                ext_param->own_addr_type = ADDR_PUBLIC;
                ext_param->scan_filt_policy = SCAN_ALLOW_ADV_ALL;

                // Indicate the activity as reserved
                llm_env.act_info[act_id].state = LLM_SCAN_RSVD;
            }

            // Get parameters
            host_params = (struct hci_le_set_ext_scan_param_cmd *) llm_env.act_info[act_id].host_params;

            /*
             * If the scanning parameters' Own_Address_Type parameter is set to 0x1 and the random address for the
             * device has not been initialized, the Controller shall return the error code Invalid HCI Command
             * Parameters (0x12).
             */
            if ((host_params->own_addr_type == ADDR_RAND) && co_bdaddr_compare(&llm_env.local_rand_addr, &co_null_bdaddr))
                break;

            /*
             * If the scanning parameters' Own_Address_Type parameter is set to 0x3, the controller's resolving list
             * did not contain a matching entry, and the random address for the device has not been initialized, the
             * Controller shall return the error code Invalid HCI Command Parameters (0x12).
             */
            if ((host_params->own_addr_type == ADDR_RPA_OR_RAND)
                    && llm_ral_is_empty()
                    && co_bdaddr_compare(&llm_env.local_rand_addr, &co_null_bdaddr))
                break;

            // Save parameters
            SETB(llm_env.adv_dup_filt_info, LLM_ADV_DUP_FILT_EN, (param->filter_duplic != SCAN_FILT_DUPLIC_DIS));
            SETB(llm_env.adv_dup_filt_info, LLM_ADV_DUP_FILT_RST, (param->filter_duplic == SCAN_FILT_DUPLIC_EN_PER_PERIOD));
            llm_env.act_info[act_id].info.scan.duration = param->duration;
            llm_env.act_info[act_id].info.scan.period   = param->period;

            // Start scanning
            status = llm_scan_start(act_id);
            llm_env.act_info[act_id].state = LLM_SCAN_EN;
        }
        break;

        default:
        {
            status = CO_ERROR_INVALID_HCI_PARAM;
        }
        break;
        }
    }

    // If scanning is stopping, wait the indication from the driver before reporting the command complete event
    if (!stopping)
    {
        // Send the command complete event
        llm_cmd_cmp_send(opcode, status);
    }

    return (KE_MSG_CONSUMED);
}

#if (EAVESDROPPING_SUPPORT)
int hci_vs_le_set_seg_scan_cmd_handler(struct hci_vs_le_set_seg_scan_cmd const *param, uint16_t opcode)
{
    uint8_t ret_status = KE_MSG_CONSUMED;
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    // Check parameter
    if (SCAN_DIS == param->enable)
    {
        // Stop segmented scanning
        status = lld_scan_seg_stop();
    }
    else // SCAN_EN
    {
        do
        {
            struct lld_scan_seg_params scan_seg;
            int i = 0;

            // Parameter sanity checks
            if ((param->scan_phys > (PHY_1MBPS_BIT | PHY_CODED_BIT)) || (0 == (param->scan_phys & (PHY_1MBPS_BIT | PHY_CODED_BIT))))
            {
                status = CO_ERROR_INVALID_HCI_PARAM;
                break;
            }

            // Scan phys to configure
            scan_seg.scan_phys = param->scan_phys;

            // BDADDR for clock alignment if present
            scan_seg.bd_addr = param->bd_addr;

            // Extract 1M phy params if specified
            if (param->scan_phys & PHY_1MBPS_BIT)
            {
                // Parameter sanity checks
                if ((param->phy[0].segments == 0) || (param->phy[0].duration >= (SLOT_SIZE * param->phy[0].periodicity)) || (co_abs(param->phy[0].us_offset) > SLOT_SIZE)
                        || (param->phy[0].time_offset >= param->phy[0].periodicity)
                        || (param->phy[0].periodicity > USHRT_MAX / 2) || (param->phy[0].duration > USHRT_MAX / 2))
                {
                    status = CO_ERROR_INVALID_HCI_PARAM;
                    break;
                }

                scan_seg.time_offset_1m = param->phy[0].time_offset;
                scan_seg.periodicity_1m = param->phy[0].periodicity;
                scan_seg.duration_1m = param->phy[0].duration;
                scan_seg.segments_1m = param->phy[0].segments;
                scan_seg.us_offset_1m = param->phy[0].us_offset;
                i++;
            }

            // Extract coded phy params if specified
            if (param->scan_phys & PHY_CODED_BIT)
            {
                // Parameter sanity checks
                if ((param->phy[i].segments == 0) || (param->phy[i].duration >= (SLOT_SIZE * param->phy[i].periodicity)) || (co_abs(param->phy[i].us_offset) > SLOT_SIZE)
                        || (param->phy[i].time_offset >= param->phy[i].periodicity)
                        || (param->phy[i].periodicity > USHRT_MAX / 2) || (param->phy[i].duration > USHRT_MAX / 2))
                {
                    status = CO_ERROR_INVALID_HCI_PARAM;
                    break;
                }

                scan_seg.time_offset_c = param->phy[i].time_offset;
                scan_seg.periodicity_c = param->phy[i].periodicity;
                scan_seg.duration_c = param->phy[i].duration;
                scan_seg.segments_c = param->phy[i].segments;
                scan_seg.us_offset_c = param->phy[i].us_offset;
            }

            // Configure/start segmented scanning.
            status = lld_scan_seg_start(&scan_seg);

        }
        while (0);
    }

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (ret_status);
}
#endif // (EAVESDROPPING_SUPPORT)

int hci_le_per_adv_create_sync_cmd_handler(struct hci_le_per_adv_create_sync_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;
    uint8_t ret_status = KE_MSG_CONSUMED;
    uint8_t act_id;

    do
    {
        if ((param->sync_to < SYNC_TIMEOUT_MIN) || (param->sync_to > SYNC_TIMEOUT_MAX) || (param->options & ~(PER_SYNC_OPTIONS_MASK))
                || (param->adv_addr_type > ADDR_RAND) || (param->adv_sid > SYNC_SID_MAX) || (param->skip > SYNC_SKIP_MAX))
            break;

        status = CO_ERROR_COMMAND_DISALLOWED;

#if (BLE_ADV_LEGACY_ITF)
        // Check current interface version
        if (llm_env.adv_itf_version == LLM_ADV_ITF_LEGACY)
            break;

        // Use extended interface
        llm_env.adv_itf_version = LLM_ADV_ITF_EXTENDED;
#endif //(BLE_ADV_LEGACY_ITF)

        // Look for the periodic sync activity
        act_id = llm_activity_syncing_get();

        if (act_id < BLE_ACTIVITY_MAX_CFG)
        {
            // If the Host issues this command when another Create_Sync is pending,  return Command Disallowed
            break;
        }

        // Check if not already synced to this device
        else if ((!GETB(param->options, PER_SYNC_FILT_USE_PAL)) && llm_is_dev_synced(&param->adv_addr, param->adv_addr_type, param->adv_sid))
        {
            status = CO_ERROR_CON_ALREADY_EXISTS;
            break;
        }

        // Allocate link identifier
        status = llm_activity_free_get(&act_id);

        // Check if link identifier found
        if (status != CO_ERROR_NO_ERROR)
        {
            break;
        }

        llm_env.act_info[act_id].state = LLM_PER_SCAN_SYNCING;

        ASSERT_ERR(llm_env.act_info[act_id].host_params == NULL);

        // Keep pointer to parameters
        llm_env.act_info[act_id].host_params = param;

        // Message needs to be maintained
        ret_status = KE_MSG_NO_FREE;

        // create sync - This command may be issued whether or not scanning is enabled
        status = lld_scan_create_sync(act_id, GETB(param->options, PER_SYNC_FILT_USE_PAL), param->adv_sid, param->adv_addr_type, (struct bd_addr *) &param->adv_addr);

    }
    while (0);

    // Send CS event
    llm_cmd_stat_send(opcode, status);

    return (ret_status);
}

int hci_le_per_adv_create_sync_cancel_cmd_handler(void const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;
    bool cc_evt = true;

    uint8_t act_id = 0;

#if (BLE_ADV_LEGACY_ITF)
    // Check current interface version
    if (llm_env.adv_itf_version != LLM_ADV_ITF_LEGACY)
#endif //(BLE_ADV_LEGACY_ITF)
    {
#if (BLE_ADV_LEGACY_ITF)
        // Use extended interface
        llm_env.adv_itf_version = LLM_ADV_ITF_EXTENDED;
#endif //(BLE_ADV_LEGACY_ITF)

        // Look for the periodic sync activity
        act_id = llm_activity_syncing_get();

        if (act_id < BLE_ACTIVITY_MAX_CFG)
        {
            // cancel sync
            status = lld_scan_create_sync_cancel(act_id);

            if (status == CO_ERROR_NO_ERROR)
            {
                // Clear state
                llm_env.act_info[act_id].state = LLM_FREE;

                // Free HCI command
                ke_msg_free(ke_param2msg(llm_env.act_info[act_id].host_params));
                llm_env.act_info[act_id].host_params = NULL;
            }
            else
            {
                // CC event not sent here
                cc_evt = false;

                // Syncinfo already processed, but may be waiting on first sync
                status = lld_sync_stop(act_id);

                if (status != CO_ERROR_NO_ERROR)
                {
                    /*
                     * LLM is waiting for sync establishment request, whereas LLD has already sent the request...
                     * => Re-post message in order to re-process after LLM has started the driver.
                     */
                    ke_msg_forward(param, TASK_LLM, opcode);

                    return KE_MSG_NO_FREE;
                }
            }
        }
    }

    if (cc_evt)
    {
        // Send the command complete event
        llm_cmd_cmp_send(opcode, status);

        if (status == CO_ERROR_NO_ERROR)
        {
            // Send LE Periodic Advertising Sync Established event with error code
            struct hci_le_per_adv_sync_est_evt *event;

            // allocate the event message
            event = KE_MSG_ALLOC(HCI_LE_EVENT, 0, HCI_LE_META_EVT_CODE, hci_le_per_adv_sync_est_evt);

            // fill event parameters
            event->subcode = HCI_LE_PER_ADV_SYNC_EST_EVT_SUBCODE;
            event->status = CO_ERROR_OPERATION_CANCELED_BY_HOST;
            event->sync_handle = BLE_ACTID_TO_SYNCHDL(act_id);
            event->adv_sid = llm_env.act_info[act_id].info.sync.adv_sid;
            event->adv_addr_type = llm_env.act_info[act_id].info.sync.addr_type;
            memcpy(&event->adv_addr.addr[0], &llm_env.act_info[act_id].info.sync.bd_addr, BD_ADDR_LEN);
            event->phy = PHY_1MBPS_VALUE;
            event->interval = 0;
            event->adv_ca = 0;

            // send the message
            hci_send_2_host(event);
        }
    }

    return (KE_MSG_CONSUMED);
}

int hci_le_per_adv_term_sync_cmd_handler(struct hci_le_per_adv_term_sync_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    do
    {
        uint8_t act_id;

#if (BLE_ADV_LEGACY_ITF)
        // Check current interface version
        if (llm_env.adv_itf_version == LLM_ADV_ITF_LEGACY)
            break;

        // Use extended interface
        llm_env.adv_itf_version = LLM_ADV_ITF_EXTENDED;
#endif //(BLE_ADV_LEGACY_ITF)

        act_id = BLE_SYNCHDL_TO_ACTID(param->sync_handle);

        if ((act_id >= BLE_ACTIVITY_MAX_CFG) || (llm_env.act_info[act_id].state != LLM_PER_SCAN_SYNCED))
        {
            break;
        }

        // terminate sync
        status = lld_sync_stop(act_id);

        if (status == CO_ERROR_NO_ERROR)
        {
            llm_env.act_info[act_id].state = LLM_PER_SCAN_STOPPING;
        }
    }
    while (0);

    // If periodic scan is stopping, wait the indication from the driver before reporting the command complete event
    if (status != CO_ERROR_NO_ERROR)
    {
        // Send the command complete event
        llm_cmd_cmp_send(opcode, status);
    }

    return (KE_MSG_CONSUMED);
}

int hci_le_add_dev_to_per_adv_list_cmd_handler(struct hci_le_add_dev_to_per_adv_list_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;

    do
    {
        uint8_t act_id;
        uint8_t position;

        // Check ADV Set ID
        if (param->adv_sid > ADV_SID_MAX)
            break;

        // Check advertiser address type
        if (param->adv_addr_type > ADDR_RAND)
            break;

        status = CO_ERROR_COMMAND_DISALLOWED;

#if (BLE_ADV_LEGACY_ITF)
        // Check current interface version
        if (llm_env.adv_itf_version == LLM_ADV_ITF_LEGACY)
            break;

        // Use extended interface
        llm_env.adv_itf_version = LLM_ADV_ITF_EXTENDED;
#endif //(BLE_ADV_LEGACY_ITF)

        // Look for the periodic sync activity
        act_id = llm_activity_syncing_get();

        if (act_id < BLE_ACTIVITY_MAX_CFG)
        {
            // If the Host issues this command when a Create_Sync is pending,  return Command Disallowed
            break;
        }

        // Check if the device is already in the list
        position = llm_dev_list_search(&param->adv_addr, param->adv_addr_type);

        if (position < BLE_WHITELIST_MAX)
        {
            // If the device is already on the list, the Controller shall return the error code Invalid HCI Command Parameters (0x12)
            if (GETB(llm_env.dev_list[position].status, LLM_DEV_IN_PL) && (llm_env.dev_list[position].adv_sids & (1 << param->adv_sid)))
            {
                status = CO_ERROR_INVALID_HCI_PARAM;
                break;
            }
        }
        else
        {
            // Try to find an empty entry to write the new address
            position = llm_dev_list_empty_entry();

            if (position == BLE_WHITELIST_MAX)
            {
                status = CO_ERROR_MEMORY_CAPA_EXCEED;
                break;
            }
        }

        // Insert the BD address in the list
        memcpy(&llm_env.dev_list[position].addr, &param->adv_addr, BD_ADDR_LEN);
        llm_env.dev_list[position].addr_type = param->adv_addr_type;
        llm_env.dev_list[position].adv_sids |= (1 << param->adv_sid);
        SETB(llm_env.dev_list[position].status, LLM_DEV_LIST_ENTRY_USED, 1);
        SETB(llm_env.dev_list[position].status, LLM_DEV_IN_PL, 1);

        // Write the EM list entry (if not synced)
        if (!llm_is_dev_synced((struct bd_addr *) &param->adv_addr, param->adv_addr_type, param->adv_sid))
        {
            lld_per_adv_list_add(position, (struct bd_addr *) &param->adv_addr, param->adv_addr_type, param->adv_sid);
        }

        status = CO_ERROR_NO_ERROR;

    }
    while (0);


    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

int hci_le_rmv_dev_from_per_adv_list_cmd_handler(struct hci_le_rmv_dev_from_per_adv_list_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;

    do
    {
        uint8_t act_id;
        uint8_t position;

        // Check ADV Set ID
        if (param->adv_sid > ADV_SID_MAX)
            break;

        // Check advertiser address type
        if (param->adv_addr_type > ADDR_RAND)
            break;

        status = CO_ERROR_COMMAND_DISALLOWED;

#if (BLE_ADV_LEGACY_ITF)
        // Check current interface version
        if (llm_env.adv_itf_version == LLM_ADV_ITF_LEGACY)
            break;

        // Use extended interface
        llm_env.adv_itf_version = LLM_ADV_ITF_EXTENDED;
#endif //(BLE_ADV_LEGACY_ITF)

        // Look for the periodic sync activity
        act_id = llm_activity_syncing_get();

        if (act_id < BLE_ACTIVITY_MAX_CFG)
        {
            // If the Host issues this command when a Create_Sync is pending,  return Command Disallowed
            break;
        }

        // Find the device in the list
        position = llm_dev_list_search(&param->adv_addr, param->adv_addr_type);

        if ((position >= BLE_WHITELIST_MAX) || !GETB(llm_env.dev_list[position].status, LLM_DEV_IN_PL))
        {
            status = CO_ERROR_UNKNOWN_ADVERTISING_ID;
            break;
        }

        // Remove ADV_SID from list entry
        llm_env.dev_list[position].adv_sids &= ~(1 << param->adv_sid);

        // Remove the periodic advertiser list entry from driver
        lld_per_adv_list_rem(position, (struct bd_addr *) &param->adv_addr, param->adv_addr_type, (1 << param->adv_sid));

        // If no more ADV_SIDs to follow, remove the entry from PAL
        if (llm_env.dev_list[position].adv_sids == 0x0000)
        {
            // Indicate the device as no more present in periodic advertiser list
            SETB(llm_env.dev_list[position].status, LLM_DEV_IN_PL, 0);

            // Check if device is in white list
            if (!GETB(llm_env.dev_list[position].status, LLM_DEV_IN_WL))
            {
                // Remove the device list entry
                SETB(llm_env.dev_list[position].status, LLM_DEV_LIST_ENTRY_USED, 0);
            }
        }

        status = CO_ERROR_NO_ERROR;

    }
    while (0);

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

int hci_le_clear_per_adv_list_cmd_handler(void const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

#if (BLE_ADV_LEGACY_ITF)
    // Check current interface version
    if (llm_env.adv_itf_version != LLM_ADV_ITF_LEGACY)
#endif //(BLE_ADV_LEGACY_ITF)
    {
        uint8_t act_id;

#if (BLE_ADV_LEGACY_ITF)
        // Use extended interface
        llm_env.adv_itf_version = LLM_ADV_ITF_EXTENDED;
#endif //(BLE_ADV_LEGACY_ITF)

        // Look for the periodic sync activity
        act_id = llm_activity_syncing_get();

        // If the Host issues this command when a Create_Sync is pending,  return Command Disallowed
        if (act_id >= BLE_ACTIVITY_MAX_CFG)
        {
            uint8_t position;

            // Parse the list linearly
            for (position = 0; position < BLE_WHITELIST_MAX ; position++)
            {
                // Check if list entry is used
                if (!GETB(llm_env.dev_list[position].status, LLM_DEV_LIST_ENTRY_USED))
                    continue;

                // Indicate the device as no more present in periodic advertiser list
                SETB(llm_env.dev_list[position].status, LLM_DEV_IN_PL, 0);

                // Check if device is in white list
                if (!GETB(llm_env.dev_list[position].status, LLM_DEV_IN_WL))
                {
                    // Remove the device list entry
                    SETB(llm_env.dev_list[position].status, LLM_DEV_LIST_ENTRY_USED, 0);
                }

                // Remove the periodic advertiser list entry from driver
                lld_per_adv_list_rem(position, &llm_env.dev_list[position].addr, llm_env.dev_list[position].addr_type, 0xFFFF);
            }

            status = CO_ERROR_NO_ERROR;
        }
    }

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

int hci_le_rd_per_adv_list_size_cmd_handler(void const *param, uint16_t opcode)
{
    struct hci_le_rd_per_adv_list_size_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_le_rd_per_adv_list_size_cmd_cmp_evt);

#if (BLE_ADV_LEGACY_ITF)
    // Check current interface version
    if (llm_env.adv_itf_version == LLM_ADV_ITF_LEGACY)
    {
        event->status  = CO_ERROR_COMMAND_DISALLOWED;
    }
    else
#endif //(BLE_ADV_LEGACY_ITF)
    {
#if (BLE_ADV_LEGACY_ITF)
        // Use extended interface
        llm_env.adv_itf_version = LLM_ADV_ITF_EXTENDED;
#endif //(BLE_ADV_LEGACY_ITF)

        event->status  = CO_ERROR_NO_ERROR;
    }

    event->size    = BLE_WHITELIST_MAX;
    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}

#if BLE_CONLESS_CTE_RX
int hci_le_set_conless_iq_sampl_en_cmd_handler(struct hci_le_set_conless_iq_sampl_en_cmd const *param, uint16_t opcode)
{
    struct hci_le_set_conless_iq_sampl_en_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_le_set_conless_iq_sampl_en_cmd_cmp_evt);
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;
    uint8_t ret_status = KE_MSG_CONSUMED;

    do
    {
        uint8_t act_id;

        if (param->sampl_en > IQ_SAMPL_EN)
        {
            status = CO_ERROR_INVALID_HCI_PARAM;
            break;
        }

        if (param->sampl_en == IQ_SAMPL_EN)
        {
            if ((param->slot_dur < SLOT_DUR_1US)
                    || (param->slot_dur > SLOT_DUR_2US)
                    || (param->max_sampl_cte > 0x10)
                    || (param->switching_pattern_len < MIN_SWITCHING_PATTERN_LEN)
                    || (param->switching_pattern_len > MAX_SWITCHING_PATTERN_LEN))
            {
                status = CO_ERROR_INVALID_HCI_PARAM;
                break;
            }

            if ((param->switching_pattern_len > BLE_MAX_SW_PAT_LEN))
            {
                status = CO_ERROR_UNSUPPORTED;
                break;
            }
        }

        status = CO_ERROR_COMMAND_DISALLOWED;

#if (BLE_ADV_LEGACY_ITF)
        // Check current interface version
        if (llm_env.adv_itf_version == LLM_ADV_ITF_LEGACY)
            break;
#endif //(BLE_ADV_LEGACY_ITF)

        // Look for the periodic advertising Rx activity
        act_id = BLE_SYNCHDL_TO_ACTID(param->sync_hdl);

        if (act_id >= BLE_ACTIVITY_MAX_CFG)
        {
            status = CO_ERROR_UNKNOWN_ADVERTISING_ID;
            break;
        }

        // Make sure that the activity is synced
        if (llm_env.act_info[act_id].state != LLM_PER_SCAN_SYNCED)
        {
            status = CO_ERROR_UNKNOWN_ADVERTISING_ID;
            break;
        }

        // If Sampling_Enable is set to 0x01 and the periodic advertising is on a PHY that does not allow Constant Tone Extensions
        if (param->sampl_en && (llm_env.act_info[act_id].info.sync.per_adv_phy != PHY_1MBPS_VALUE) && (llm_env.act_info[act_id].info.sync.per_adv_phy != PHY_2MBPS_VALUE))
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        // Free potentially stored parameters
        if (llm_env.act_info[act_id].info.sync.cte_rx_params != NULL)
        {
            ke_msg_free(ke_param2msg(llm_env.act_info[act_id].info.sync.cte_rx_params));
            llm_env.act_info[act_id].info.sync.cte_rx_params = NULL;
        }

        // Keep pointer to parameters
        llm_env.act_info[act_id].info.sync.cte_rx_params = param;

        // Message needs to be maintained
        ret_status = KE_MSG_NO_FREE;

        //If CTE is enabled
        if (param->sampl_en)
        {
            struct lld_sync_cte_params sync_cte_par;

            sync_cte_par.sampl_en = param->sampl_en;
            sync_cte_par.slot_dur = param->slot_dur;
            sync_cte_par.max_sampl_cte = param->max_sampl_cte;
            sync_cte_par.switching_pattern_len = param->switching_pattern_len;
            memcpy(&sync_cte_par.antenna_id[0], &param->antenna_id[0], param->switching_pattern_len);

            // Start IQ sampling
            lld_sync_cte_start(act_id, &sync_cte_par);
        }
        else //If CTE is disabled
        {
            // Stop IQ sampling
            lld_sync_cte_stop(act_id);
        }

        status = CO_ERROR_NO_ERROR;

    }
    while (0);

    // Send Command Complete event
    event->status  = status;
    event->sync_hdl = param->sync_hdl;
    hci_send_2_host(event);

    return (ret_status);
}
#endif // BLE_CONLESS_CTE_RX

/**
 ****************************************************************************************
 * @brief Handles the command HCI LE set periodic advertising receive enable.
 *
 * @param[in] link_id Link Identifier
 * @param[in] param   Pointer to the parameters of the message.
 * @param[in] opcode  HCI Operation code
 ****************************************************************************************
 */
int hci_le_set_per_adv_rec_en_cmd_handler(struct hci_le_set_per_adv_rec_en_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;

    do
    {
        uint8_t act_id;

        if (param->en > PER_ADV_REC_EN_MASK)
            break;

        // Look for the periodic advertising Rx activity
        act_id = BLE_SYNCHDL_TO_ACTID(param->sync_hdl);

        if (act_id >= BLE_ACTIVITY_MAX_CFG)
        {
            status = CO_ERROR_UNKNOWN_ADVERTISING_ID;
            break;
        }

        // Make sure that the activity is synced
        if (llm_env.act_info[act_id].state != LLM_PER_SCAN_SYNCED)
        {
            status = CO_ERROR_UNKNOWN_ADVERTISING_ID;
            break;
        }

        // Update filter enable flags to driver
        status = lld_sync_rep_filt_en(act_id, param->en);

    }
    while (0);

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Check if the advertiser address has been already received.
 *
 * This function checks in case where the filtering is enabled if the address is already listed.
 * If not already listed, the function lists it.
 *
 * @param[in] adv_bd_addr         Pointer on the device address to be checked.
 * @param[in] addr_type           Address type (Public, Random, Identity, Static)
 * @param[in] adv_evt_type        Type of the advertising report (ADV_IND, ADV., SCANRSP)
 * @param[in] sid                 Advertising Set ID (0xFF if not present)
 * @param[in] did                 Advertising Data ID
 * @param[in] per_adv_intv        Periodic Advertising Interval
 *
 * @return If the device address has been found or not.
 ****************************************************************************************
 */
__INLINE bool llm_adv_reports_list_check(struct bd_addr const *adv_bd_addr, uint8_t addr_type, uint8_t adv_evt_type, uint8_t sid, uint16_t did, uint16_t per_adv_intv)
{
    bool found = false;
    bool filtered = false;

    for (int i = 0; i < llm_env.dup_filt.nb_adv; i++)
    {
        // Check BD Address, type, and set ID
        if (co_bdaddr_compare(adv_bd_addr, &llm_env.dup_filt.tab[i].adv_addr)
                && (addr_type == llm_env.dup_filt.tab[i].addr_type)
                && (adv_evt_type == llm_env.dup_filt.tab[i].adv_evt_type)
                && (sid == llm_env.dup_filt.tab[i].sid))
        {
            bool param_updated = false;
            found = true;

            // Check the presence of ADI
            if (sid != REP_ADV_NO_ADI)
            {
                // Check if data is new
                if (did != llm_env.dup_filt.tab[i].did)
                {
                    // Store the latest DID
                    llm_env.dup_filt.tab[i].did = did;
                    param_updated = true;
                }
            }

            // Check the periodic advertising interval
            // Minimum value is 0x0006
            if (per_adv_intv >= CON_INTERVAL_MIN)
            {
                // Check if the periodic advertising interval is new
                if (per_adv_intv != llm_env.dup_filt.tab[i].per_adv_intv)
                {
                    // Store the latest periodic advertising interval
                    llm_env.dup_filt.tab[i].per_adv_intv = per_adv_intv;
                    param_updated = true;
                }
            }

            // Do not filter if a parameter has been updated
            if (param_updated)
                break;

            filtered = true;

            break;
        }
    }

    // If the device has not been found, add it to the duplicate filter list
    if (!found)
    {
        // Fill the current table entry
        struct adv_dup_filt_entry *dev =  llm_env.dup_filt.tab + llm_env.dup_filt.curr_pos;
        memcpy(&dev->adv_addr.addr[0], &adv_bd_addr->addr[0], BD_ADDR_LEN);
        dev->adv_evt_type = adv_evt_type;
        dev->addr_type = addr_type;
        dev->sid = sid;
        dev->did = did;
        dev->per_adv_intv = per_adv_intv;

        // Move table cursor
        llm_env.dup_filt.curr_pos++;
        if (llm_env.dup_filt.curr_pos == BLE_DUPLICATE_FILTER_MAX)
        {
            llm_env.dup_filt.curr_pos = 0;
        }

        // Increment number of discovered advertisers
        if (llm_env.dup_filt.nb_adv < BLE_DUPLICATE_FILTER_MAX)
        {
            llm_env.dup_filt.nb_adv++;
        }
    }

    return (filtered);
}


/*
 * LLM MESSAGE HANDLERS
 ****************************************************************************************
 */
/**
 ****************************************************************************************
 * @brief Handles the period time out message.
 ****************************************************************************************
 */
KE_MSG_HANDLER_NO_STATIC(llm_scan_period_to, void)
{
    uint8_t act_id;

    // Look for the scanning activity
    for (act_id = 0; act_id < BLE_ACTIVITY_MAX_CFG; act_id++)
    {
        if (llm_env.act_info[act_id].state == LLM_SCAN_EN)
            break;
    }

    if (act_id < BLE_ACTIVITY_MAX_CFG)
    {
        // Restart scanning for a new period
        llm_scan_start(act_id);
    }

    return (KE_MSG_CONSUMED);
}


/*
 * LLD INDICATION HANDLERS
 ****************************************************************************************
 */
KE_MSG_HANDLER_NO_STATIC(lld_adv_rep_ind, struct lld_adv_rep_ind)
{
    uint8_t prim_phy = co_rate_to_phy[param->rate1];
    struct hci_le_set_ext_scan_param_cmd *host_params = (struct hci_le_set_ext_scan_param_cmd *) llm_env.act_info[param->act_id].host_params;

    struct bd_addr peer_id_addr; // Peer device identity
    struct bd_addr local_rpa; // Local RPA (adv direct and InitA not resolved)
    struct bd_addr peer_rpa; // Peer RPA (if privacy enabled)
    struct bd_addr dir_addr; //Direct address value

    memset(&peer_id_addr.addr[0], 0, BD_ADDR_LEN);
    memset(&local_rpa.addr[0], 0, BD_ADDR_LEN);
    memset(&peer_rpa.addr[0], 0, BD_ADDR_LEN);
    memset(&dir_addr.addr[0], 0, BD_ADDR_LEN);

    uint8_t dir_addr_type = 0; // Direct address type
    uint8_t addr_type = param->addr_type; // Peer address type

    bool filtered = false;

    // Check resolving list
    if (addr_type != ADDR_NONE)
    {
        uint16_t rxralptr = param->rxralptr;

        uint8_t ral_idx = (rxralptr) ? ((rxralptr - EM_BLE_RAL_OFFSET) / REG_EM_BLE_RAL_SIZE) : 0;

        if (rxralptr != 0)
        {
            // get the peer ID -  Do a burst read in the exchange memory
            em_rd(&peer_id_addr.addr[0], rxralptr + EM_BLE_RAL_PEER_ID_INDEX * 2, BD_ADDR_LEN);

            // get the local RPA -  Do a burst read in the exchange memory
            if (em_ble_ral_info_local_rpa_valid_getf(ral_idx))
            {
                em_rd(&local_rpa.addr[0], rxralptr + EM_BLE_RAL_LOCAL_RPA_INDEX * 2, BD_ADDR_LEN);
            }

            // Only load RPA if peer_add_match, or where peer_add_match is not applicable (undirected, SCAN RSP events).
            if ((param->peer_add_match) || (0 == (param->evt_type & DIR_ADV_EVT_MSK)) || (param->evt_type & SCAN_ADV_EVT_MSK))
            {
                // get the peer RPA -  Do a burst read in the exchange memory
                if (em_ble_ral_info_peer_rpa_valid_getf(ral_idx))
                {
                    em_rd(&peer_rpa.addr[0], rxralptr + EM_BLE_RAL_PEER_RPA_INDEX * 2, BD_ADDR_LEN);
                }

                // Determine Peer address type
                if (em_ble_ral_info_peer_rpa_valid_getf(ral_idx) || em_ble_ral_info_local_rpa_valid_getf(ral_idx))
                {
                    addr_type = em_ble_ral_info_peer_id_type_getf(ral_idx);

                    //Inspect received peer address
                    if ((param->peer_id_addr.addr[BD_ADDR_LEN - 1] & 0xC0) == RND_RSLV_ADDR)
                    {
                        addr_type |= ADDR_RPA_MASK;
                    }
                }
            }
        }
        else
        {
            memcpy(&peer_id_addr.addr[0], &param->peer_id_addr.addr[0], BD_ADDR_LEN);
        }

        // Indicate whether a directed advertising type/addr is resolved
        if (param->evt_type & DIR_ADV_EVT_MSK)
        {
            /*
             * Direct_Address_Type and Direct_Address specify the address the directed advertisements are being directed to.
             */
            if (param->local_add_match)
            {
                dir_addr_type = host_params->own_addr_type & ADDR_MASK;

                //Inspect received target address
                if ((param->target_id_addr.addr[BD_ADDR_LEN - 1] & 0xC0) == RND_RSLV_ADDR)
                {
                    dir_addr_type |= ADDR_RPA_MASK;
                }

                if (dir_addr_type & ADDR_RAND) // local random address
                {
                    memcpy(&dir_addr.addr[0], &llm_env.local_rand_addr.addr[0], BD_ADDR_LEN);
                }
                else // local public address
                {
                    memcpy(&dir_addr.addr[0], &llm_env.local_pub_addr.addr[0], BD_ADDR_LEN);
                }
            }
            else
            {
                memcpy(&dir_addr.addr[0], &param->target_id_addr.addr[0], BD_ADDR_LEN);

                dir_addr_type = param->rx_rxadd;

                // Check if the Directed_Address is a Private Address
                if (((dir_addr_type != ADDR_PUBLIC) && ((param->target_id_addr.addr[BD_ADDR_LEN - 1] & 0xC0) != RND_STATIC_ADDR))
                        && (host_params->scan_filt_policy >= SCAN_ALLOW_ADV_ALL_AND_INIT_RPA))
                {
                    dir_addr_type = ADDR_RAND_UNRESOLVED;
                }
            }
        }
    }

    ASSERT_ERR((prim_phy == PHY_1MBPS_VALUE) || (prim_phy == PHY_CODED_VALUE));

    if ((param->evt_type & ADV_DIRECT)
            && ((dir_addr.addr[BD_ADDR_LEN - 1] & 0xC0) == RND_RSLV_ADDR)
            && (dir_addr_type == ADDR_RAND_UNRESOLVED)
            && (host_params->scan_filt_policy < SCAN_ALLOW_ADV_ALL_AND_INIT_RPA)
       )
    {
        filtered = true; //  Advertiser with an incorrect target IRK, should not report it if filter policy excludes it.
    }
    // Check if filter duplicate is enabled
    else if (GETB(llm_env.adv_dup_filt_info, LLM_ADV_DUP_FILT_EN))
    {
        // Get chain info for the scan PHY
        uint8_t chain_info;

        ASSERT_ERR((param->chain == 1) || (param->chain == 2));

        if (param->chain == 1)
        {
            chain_info = (prim_phy == PHY_1MBPS_VALUE) ? GETF(llm_env.adv_dup_filt_info, LLM_ADV_DUP_FILT_CHAIN1_1M) :
                         GETF(llm_env.adv_dup_filt_info, LLM_ADV_DUP_FILT_CHAIN1_CODED);
        }
        else // chain 2
        {
            chain_info = (prim_phy == PHY_1MBPS_VALUE) ? GETF(llm_env.adv_dup_filt_info, LLM_ADV_DUP_FILT_CHAIN2_1M) :
                         GETF(llm_env.adv_dup_filt_info, LLM_ADV_DUP_FILT_CHAIN2_CODED);
        }

        switch (chain_info)
        {
        case LLM_ADV_CHAIN_FILTERED:
        {
            // The report is part of a filtered chain
            filtered = true;
        }
        break;
        case LLM_ADV_CHAIN_NO:
        {
            uint8_t sid = REP_ADV_NO_ADI;
            uint16_t did = 0;

            if (param->adi_present)
            {
                sid = GETF(param->adi, BLE_ADI_SID);
                did = GETF(param->adi, BLE_ADI_DID);
            }

            // Check if this new report must be filtered
            filtered = llm_adv_reports_list_check(&peer_id_addr, addr_type, param->evt_type, sid, did, param->interval);
        }
        break;
        default:
            break; // Nothing to do
        }

        // Evaluate new chain info
        if (param->data_status != ADV_EVT_DATA_STATUS_INCOMPLETE)
        {
            chain_info = LLM_ADV_CHAIN_NO;
        }
        else if (filtered)
        {
            chain_info = LLM_ADV_CHAIN_FILTERED;
        }
        else
        {
            chain_info = LLM_ADV_CHAIN_VALID;
        }

        // Update chain info for the scan PHY
        if (prim_phy == PHY_1MBPS_VALUE)
        {
            if (param->chain == 1)
            {
                SETF(llm_env.adv_dup_filt_info, LLM_ADV_DUP_FILT_CHAIN1_1M, chain_info);
            }
            else // chain 2
            {
                SETF(llm_env.adv_dup_filt_info, LLM_ADV_DUP_FILT_CHAIN2_1M, chain_info);
            }
        }
        else
        {
            if (param->chain == 1)
            {
                SETF(llm_env.adv_dup_filt_info, LLM_ADV_DUP_FILT_CHAIN1_CODED, chain_info);
            }
            else // chain 2
            {
                SETF(llm_env.adv_dup_filt_info, LLM_ADV_DUP_FILT_CHAIN2_CODED, chain_info);
            }
        }
    }

    if (!filtered)
    {
#if (BLE_ADV_LEGACY_ITF)
        if (llm_env.adv_itf_version == LLM_ADV_ITF_LEGACY) // legacy HCI advertising reports
        {
            // Choose advertising report event type (hci_le_adv_report or hci_le_dir_adv_rep)
            /*
             * The LE Direct Advertising Report event indicates that directed advertisements have been received where the
             *  advertiser is using a resolvable private address for the TargetA field of the advertising PDU which the
             *  Controller is unable to resolve (and not filtered, as the Scanning_Filter_Policy is equal to 0x02 or 0x03).
             */
            do
            {

                if ((param->evt_type & ADV_LEGACY) == 0)
                    break;

                if ((param->evt_type & ADV_DIRECT)
                        && ((dir_addr.addr[BD_ADDR_LEN - 1] & 0xC0) == RND_RSLV_ADDR)
                        && (dir_addr_type == ADDR_RAND_UNRESOLVED)
                   )
                {
                    struct hci_le_dir_adv_rep_evt *event = KE_MSG_ALLOC(HCI_LE_EVENT, 0, HCI_LE_META_EVT_CODE, hci_le_dir_adv_rep_evt);

                    event->subcode = HCI_LE_DIR_ADV_REP_EVT_SUBCODE;
                    event->nb_reports = 1;
                    event->adv_rep[0].rssi = param->rssi;
                    event->adv_rep[0].evt_type = ADV_DIRECT_IND_EVT;
                    event->adv_rep[0].addr_type = addr_type;
                    memcpy(&event->adv_rep[0].addr.addr[0], &peer_id_addr.addr[0], BD_ADDR_LEN);
                    event->adv_rep[0].dir_addr_type = ADDR_RAND;
                    memcpy(&event->adv_rep[0].dir_addr.addr[0], &dir_addr.addr[0], BD_ADDR_LEN);
                    hci_send_2_host(event);
                }
                else
                {
                    // Report the advertising reception to Host
                    struct hci_le_adv_report_evt *event = KE_MSG_ALLOC(HCI_LE_EVENT, 0, HCI_LE_META_EVT_CODE, hci_le_adv_report_evt);

                    event->subcode = HCI_LE_ADV_REPORT_EVT_SUBCODE;
                    event->nb_reports = 1;
                    event->adv_rep[0].evt_type = adv_evt_prop2type[param->evt_type];
                    event->adv_rep[0].adv_addr_type = addr_type;
                    memcpy(&event->adv_rep[0].adv_addr.addr[0], &peer_id_addr.addr[0], BD_ADDR_LEN);
                    event->adv_rep[0].data_len = param->data_len;
                    em_rd(&event->adv_rep[0].data[0], param->em_buf + param->data_offset, param->data_len);
                    memset(&event->adv_rep[0].data[param->data_len], 0, ADV_DATA_LEN - param->data_len);
                    event->adv_rep[0].rssi = param->rssi;

                    hci_send_2_host(event);
                }
            }
            while (0);
        }
        else
#endif //(BLE_ADV_LEGACY_ITF)
        {
            uint8_t rem_data_len = param->data_len;
            uint8_t event_data_offset = param->data_offset;

            // do-while ensures at least one adv report is sent, i.e. if data_len == 0, one report is sent.
            do
            {
                bool incomplete = (rem_data_len > EXT_ADV_DATA_MAX_LEN);
                uint8_t report_data_len = (rem_data_len > EXT_ADV_DATA_MAX_LEN) ? EXT_ADV_DATA_MAX_LEN : rem_data_len;

                struct hci_le_ext_adv_report_evt *event = KE_MSG_ALLOC(HCI_LE_EVENT, 0, HCI_LE_META_EVT_CODE, hci_le_ext_adv_report_evt);

                event->subcode = HCI_LE_EXT_ADV_REPORT_EVT_SUBCODE;
                event->nb_reports = 1;
                event->adv_rep[0].evt_type = param->evt_type;
                SETF(event->adv_rep[0].evt_type, ADV_EVT_DATA_STATUS, (incomplete ? ADV_EVT_DATA_STATUS_INCOMPLETE : (param->data_status)));
                event->adv_rep[0].adv_addr_type = addr_type;
                memcpy(&event->adv_rep[0].adv_addr.addr[0], &peer_id_addr.addr[0], BD_ADDR_LEN);
                event->adv_rep[0].phy = co_rate_to_phy[param->rate1];
                event->adv_rep[0].phy2 = co_rate_to_phy[param->rate2];
                event->adv_rep[0].adv_sid = (param->adi_present) ? GETF(param->adi, BLE_ADI_SID) : REP_ADV_NO_ADI;
                event->adv_rep[0].tx_power = param->tx_power;
                event->adv_rep[0].rssi = param->rssi;
                event->adv_rep[0].interval = param->interval;
                event->adv_rep[0].dir_addr_type = dir_addr_type;
                memcpy(&event->adv_rep[0].dir_addr.addr[0], &dir_addr.addr[0], BD_ADDR_LEN);
                event->adv_rep[0].data_len = report_data_len;
                em_rd(&event->adv_rep[0].data[0], param->em_buf + event_data_offset, report_data_len);
                memset(&event->adv_rep[0].data[report_data_len], 0, EXT_ADV_DATA_MAX_LEN - report_data_len);

                hci_send_2_host(event);

                event_data_offset += report_data_len;
                rem_data_len -= report_data_len;

            }
            while (rem_data_len);
        }
    }

    if (param->data_len)
    {
        // Free buffer, no more needed
        ble_util_buf_rx_free(param->em_buf);
    }

    return (KE_MSG_CONSUMED);
}

KE_MSG_HANDLER_NO_STATIC(lld_sync_start_req, struct lld_sync_start_req)
{
    struct lld_sync_params sync_params;
    uint8_t act_id = param->act_id;

    // Check that the state did not become LLM_FREE due to Create Sync Cancel
    if (llm_env.act_info[act_id].state == LLM_PER_SCAN_SYNCING)
    {
        // Check resolving list
        if (param->rxralptr != 0)
        {
            uint8_t position = (param->rxralptr - EM_BLE_RAL_OFFSET) / REG_EM_BLE_RAL_SIZE;

            ASSERT_ERR(position < BLE_RAL_MAX);

            // Get the peer ID from RAL
            llm_env.act_info[act_id].info.sync.bd_addr = llm_env.ral[position].bd_addr;
            llm_env.act_info[act_id].info.sync.addr_type = llm_env.ral[position].addr_type;

            // Check whether peer sent public address or resolvable address (RAL pointer may be set even when no address resolution is performed
            // - if the peer's public address is in the resolving list).
            if (((param->adv_addr_type == ADDR_RAND) && ((param->adv_addr.addr[BD_ADDR_LEN - 1] & 0xC0) == RND_RSLV_ADDR)))
            {
                llm_env.act_info[act_id].info.sync.addr_type |= ADDR_RPA_MASK;
            }

            // Also store peer RPA (used by PAST feature)
            llm_env.act_info[act_id].info.sync.peer_rpa = param->adv_addr;
        }
        else
        {
            // Peer ID is the received address
            llm_env.act_info[act_id].info.sync.bd_addr = param->adv_addr;
            llm_env.act_info[act_id].info.sync.addr_type = param->adv_addr_type;

            // Clear peer RPA
            llm_env.act_info[act_id].info.sync.peer_rpa = co_null_bdaddr;
        }

        llm_env.act_info[act_id].info.sync.adv_sid = param->adv_sid;
        llm_env.act_info[act_id].info.sync.skip = ((struct hci_le_per_adv_create_sync_cmd const *)llm_env.act_info[act_id].host_params)->skip;

        // Assign validated SyncInfo & timing, Periodic Sync configuration/parameters to start Sync driver.
        sync_params.p_syncinfo = &param->syncinfo;
        sync_params.base_cnt = param->base_cnt;
        sync_params.fine_cnt = param->fine_cnt;
        sync_params.rate = param->rate;
        sync_params.skip = ((struct hci_le_per_adv_create_sync_cmd const *)llm_env.act_info[act_id].host_params)->skip;
        sync_params.sync_to = ((struct hci_le_per_adv_create_sync_cmd const *)llm_env.act_info[act_id].host_params)->sync_to;
        sync_params.sync_cte_type = ((struct hci_le_per_adv_create_sync_cmd const *)llm_env.act_info[act_id].host_params)->sync_cte_type;
        sync_params.add_drift = 0;
        sync_params.adv_addr = llm_env.act_info[act_id].info.sync.bd_addr;
        sync_params.rep_en = !GETB(((struct hci_le_per_adv_create_sync_cmd const *)llm_env.act_info[act_id].host_params)->options, PER_SYNC_REP_INIT_DIS);
#if (BT_53)
        sync_params.adv_sid = param->adv_sid;
        sync_params.adi_filt_en = GETB(((struct hci_le_per_adv_create_sync_cmd const *)llm_env.act_info[act_id].host_params)->options, PER_SYNC_DUPLIC_FILT_EN);
#endif // (BT_53)

        // Start receive of Periodic Advertisements based on presented SyncInfo
        lld_sync_start(param->act_id, &sync_params);
    }
    else
    {
        ASSERT_ERR(llm_env.act_info[act_id].state == LLM_FREE);
    }

    return (KE_MSG_CONSUMED);
}

KE_MSG_HANDLER_NO_STATIC(lld_per_adv_rep_ind, struct lld_per_adv_rep_ind)
{
    uint8_t act_id = param->act_id;


    if ((llm_env.act_info[act_id].state == LLM_PER_SCAN_SYNCING)
#if (BLE_PAST)
            || (llm_env.act_info[act_id].state == LLM_PER_SCAN_SYNCING_FROM_SYNC_TRANSF)
#endif // (BLE_PAST)
       )
    {
        uint8_t addr_type = llm_env.act_info[act_id].info.sync.addr_type;
        struct bd_addr *p_bd_addr = &llm_env.act_info[act_id].info.sync.bd_addr;
        uint8_t position;

#if (BLE_PAST)
        bool sync_from_scan = (llm_env.act_info[act_id].state == LLM_PER_SCAN_SYNCING);

        if (sync_from_scan)
#endif // (BLE_PAST)
        {
            // Cancel sync
            lld_scan_create_sync_cancel(act_id);
        }

        // Set state to periodic scanning
        llm_env.act_info[act_id].state                  = LLM_PER_SCAN_SYNCED;
        llm_env.act_info[act_id].info.sync.acad_dest_id = TASK_NONE;

        {
            // Deduce max drift to take into account for next event
            uint32_t max_drift = (1 + llm_env.act_info[act_id].info.sync.skip) * (param->interval << 2) * (rwip_max_drift_get() + co_sca2ppm[param->adv_ca]) / 1600; // half-slots * ppm * 625 half-us / 1000000;
            // Window widening in half-us
            uint32_t win_widening = max_drift + 2 * BLE_MAX_JITTER;
            // Computed RX window size in half-us
            uint32_t rx_win_size = 2 * win_widening;
            // Duration in half-slots
            uint16_t duration_min_hs = (2 * ble_util_pkt_dur_in_us(param->data_len, param->rate) + (HALF_SLOT_SIZE - 1)) / HALF_SLOT_SIZE;

            // Register activity into planner
            struct sch_plan_elt_tag *new_plan_elt = &llm_env.act_info[act_id].plan_elt;
            new_plan_elt->interval = param->interval << 2;
            new_plan_elt->conhdl = act_id;
            new_plan_elt->conhdl_ref = new_plan_elt->conhdl;
            new_plan_elt->cb_move = NULL;
            new_plan_elt->mobility = SCH_PLAN_MB_LVL_0;
            new_plan_elt->offset = param->act_offset;
            new_plan_elt->duration_min = duration_min_hs;
            new_plan_elt->duration_max = new_plan_elt->duration_min;;
            new_plan_elt->margin = 1 + ((rx_win_size + HALF_SLOT_SIZE / 2)) / HALF_SLOT_SIZE;
            sch_plan_set(new_plan_elt);
        }

        // Check if the device is in the device list
        position = llm_dev_list_search(p_bd_addr, addr_type & ADDR_MASK);

        // Remove device from EM per_adv list, if it is in per_adv list
        if ((position < BLE_WHITELIST_MAX) && GETB(llm_env.dev_list[position].status, LLM_DEV_IN_PL))
        {
            lld_per_adv_list_rem(position, p_bd_addr, addr_type & ADDR_MASK, (1 << llm_env.act_info[act_id].info.sync.adv_sid));
        }

#if (BLE_PAST)
        if (sync_from_scan)
#endif // (BLE_PAST)
        {
            // Report sync establishment
            struct hci_le_per_adv_sync_est_evt *event = KE_MSG_ALLOC(HCI_LE_EVENT, 0, HCI_LE_META_EVT_CODE, hci_le_per_adv_sync_est_evt);
            event->subcode = HCI_LE_PER_ADV_SYNC_EST_EVT_SUBCODE;
            event->status = CO_ERROR_NO_ERROR;
            event->sync_handle = BLE_ACTID_TO_SYNCHDL(act_id);
            event->adv_sid = llm_env.act_info[act_id].info.sync.adv_sid;
            event->adv_addr_type = addr_type;
            memcpy(&event->adv_addr.addr[0], p_bd_addr, BD_ADDR_LEN);
            event->phy = co_rate_to_phy[param->rate];
            event->interval = param->interval;
            event->adv_ca = param->adv_ca;
            hci_send_2_host(event);

            // Free HCI command
            ke_msg_free(ke_param2msg(llm_env.act_info[act_id].host_params));
            llm_env.act_info[act_id].host_params = NULL;
        }
#if (BLE_PAST)
        else
        {
            // Report sync establishment
            struct hci_le_per_adv_sync_transf_rec_evt *event = KE_MSG_ALLOC(HCI_LE_EVENT, BLE_LINKID_TO_CONHDL(llm_env.act_info[act_id].info.sync.ass_act_id), HCI_LE_META_EVT_CODE, hci_le_per_adv_sync_transf_rec_evt);
            event->subcode = HCI_LE_PER_ADV_SYNC_TRANSF_REC_EVT_SUBCODE;
            event->status = CO_ERROR_NO_ERROR;
            event->conhdl = BLE_LINKID_TO_CONHDL(llm_env.act_info[act_id].info.sync.ass_act_id);
            event->sync_hdl = BLE_ACTID_TO_SYNCHDL(act_id);
            event->adv_sid = llm_env.act_info[act_id].info.sync.adv_sid;
            event->phy = co_rate_to_phy[param->rate];
            event->interval = param->interval;
            event->adv_ca = param->adv_ca;
            event->serv_data = llm_env.act_info[act_id].info.sync.serv_data;
            event->adv_addr = llm_env.act_info[act_id].info.sync.bd_addr;
            event->adv_addr_type = addr_type;
            hci_send_2_host(event);
        }
#endif // (BLE_PAST)

        // Save the periodic advertising PHY value
        llm_env.act_info[act_id].info.sync.per_adv_phy = co_rate_to_phy[param->rate];
    }


    if (LLM_PER_SCAN_SYNCED == llm_env.act_info[act_id].state)
    {
        uint8_t rem_data_len = param->data_len;
        uint8_t event_data_offset = param->data_len ? param->data_offset : 0;

        // If the event includes data to report, i.e. not filtered.
        if (!param->filt_data)
        {
            //Per Adv Report: 0 - 248; AdvData: 0 - 254; => Must support segmentation here.
            // do-while ensures at least one adv report is sent, i.e. if data_len == 0, one report is sent.
            do
            {
                bool incomplete = (rem_data_len > PER_ADV_DATA_MAX_LEN);
                uint8_t report_data_len = (rem_data_len > PER_ADV_DATA_MAX_LEN) ? PER_ADV_DATA_MAX_LEN : rem_data_len;
                // allocate the event message
                struct hci_le_per_adv_report_evt *event = KE_MSG_ALLOC(HCI_LE_EVENT, 0, HCI_LE_META_EVT_CODE, hci_le_per_adv_report_evt);

                // fill event parameters
                event->subcode = HCI_LE_PER_ADV_REPORT_EVT_SUBCODE;
                event->sync_handle = BLE_ACTID_TO_SYNCHDL(act_id);
                event->tx_power = param->tx_power;
                event->rssi = param->rssi;
                event->cte_type = param->cte_type;
                event->status = incomplete ? PER_ADV_EVT_DATA_STATUS_INCOMPLETE : param->data_status;

                event->data_len = report_data_len;
                if (report_data_len)
                {
                    em_rd(&event->data[0], param->em_buf + event_data_offset, report_data_len);
                }

                // send the message
                hci_send_2_host(event);

                event_data_offset += report_data_len;
                rem_data_len -= report_data_len;

            }
            while (rem_data_len);
        }

        {
            struct sch_plan_elt_tag *plan_elt = &llm_env.act_info[act_id].plan_elt;

            // If offset has changed
            if ((param->act_offset < plan_elt->interval) && (plan_elt->offset != param->act_offset))
            {
                // Update planner element
                sch_plan_shift(act_id, param->act_offset - plan_elt->offset);
            }
        }

        // ACAD data is sent by periodic advertiser
        if (param->acad_len > 0)
        {
            uint8_t cursor = 0;

            // Channel Map Update Indication; shall not appear more than once in a block.
            bool ch_map_updated = false;

            // loop in ACAD data
            while (cursor < param->acad_len)
            {
                uint8_t ad_type_len = em_rd8p(param->em_buf + param->acad_offset + cursor);
                uint8_t ad_type     = em_rd8p(param->em_buf + param->acad_offset + cursor + 1);
                uint8_t unpack_status = CO_ERROR_UNSPECIFIED_ERROR;
                struct big_info big_info;

                switch (ad_type)
                {
                case BLE_EXT_ACAD_CHANNEL_MAP_UPDATE_INDICATION_AD_TYPE:
                {
                    if (!ch_map_updated)
                    {
                        struct le_ch_map ch_map;
                        uint16_t instant;

                        // Extract channel map and instant
                        uint8_t param_offset = param->acad_offset + cursor + 2;
                        em_rd((void *)&ch_map, param->em_buf + param_offset, LE_CH_MAP_LEN);
                        param_offset += LE_CH_MAP_LEN;
                        instant = em_rd16p(param->em_buf + param_offset);

                        // Instruct the driver about the channel map update
                        lld_sync_ch_map_update(act_id, &ch_map, instant);

                        ch_map_updated = true;
                    }
                }
                break;

#if (BLE_BIS)
                case BLE_EXT_ACAD_BIG_INFO_AD_TYPE:
                {
                    uint8_t big_info_packed[BLE_EXT_ACAD_BIG_INFO_ENC_LEN];

                    // Read the advertising data
                    em_rd(&big_info_packed, param->em_buf + param->acad_offset + cursor + 2, ad_type_len - 1);

                    // Unpack BIG info
                    unpack_status = ble_util_big_info_unpack(&big_info, (const uint8_t *) &big_info_packed, ad_type_len - 1);
                    if ((unpack_status == CO_ERROR_NO_ERROR) && !param->filt_data)
                    {
                        // Send the BIGInfo Advertising Report event
                        if (big_info.phy <= 3)
                            llm_big_info_adv_report_evt_send(BLE_ACTID_TO_SYNCHDL(act_id), &big_info);
                    }
                }
                break;
#endif // (BLE_BIS)

                default:
                {
                    // Do nothing
                }
                break;
                }


                // If an activity is following ACAD, for this specific ad type, and data length is valid
                if ((llm_env.act_info[act_id].info.sync.acad_dest_id != TASK_NONE) && (ad_type == llm_env.act_info[act_id].info.sync.ad_type)
                        && (ad_type_len < (param->acad_len - cursor)) && (unpack_status == CO_ERROR_NO_ERROR))
                {
                    struct llm_acad_data_ind *acad_data = KE_MSG_ALLOC(LLM_ACAD_DATA_IND,
                                                          llm_env.act_info[act_id].info.sync.acad_dest_id,
                                                          TASK_LLM, llm_acad_data_ind);
                    // fill information
                    acad_data->sync_handle = BLE_ACTID_TO_SYNCHDL(act_id);
                    acad_data->status      = unpack_status;
                    acad_data->ad_type     = ad_type;
                    acad_data->ref_evt_cnt = param->ref_evt_cnt;
                    memcpy(&acad_data->data, &big_info, sizeof(struct big_info));

                    ke_msg_send(acad_data);
                }

                // update cursor to next adv data type
                cursor += ad_type_len + 1;
            }

        }
    }

    if (param->data_len || param->acad_len)
    {
        // Free buffer, no more needed
        ble_util_buf_rx_free(param->em_buf);
    }

    return (KE_MSG_CONSUMED);
}

KE_MSG_HANDLER_NO_STATIC(lld_per_adv_rx_end_ind, struct lld_per_adv_rx_end_ind)
{
    uint8_t act_id = param->act_id;
    bool continue_syncing = false;

    uint8_t addr_type = llm_env.act_info[act_id].info.sync.addr_type;
    struct bd_addr *p_bd_addr = &llm_env.act_info[act_id].info.sync.bd_addr;

    switch (llm_env.act_info[act_id].state)
    {
    case LLM_PER_SCAN_SYNCED:
    case LLM_PER_SCAN_STOPPING:
    {
        uint8_t position;

        // Unregister the activity from bandwidth allocation system
        sch_plan_rem(&llm_env.act_info[act_id].plan_elt);

        // Check if the device is in the device list
        position = llm_dev_list_search(p_bd_addr, addr_type);

        // Restore presence of device in EM periodic advertiser list, if it is periodic advertiser list
        if ((position < BLE_WHITELIST_MAX) && GETB(llm_env.dev_list[position].status, LLM_DEV_IN_PL)
                && (llm_env.dev_list[position].adv_sids & (1 << llm_env.act_info[act_id].info.sync.adv_sid)))
        {
            lld_per_adv_list_add(position, p_bd_addr, addr_type, llm_env.act_info[act_id].info.sync.adv_sid);
        }

        if (llm_env.act_info[act_id].state == LLM_PER_SCAN_STOPPING)
        {
            // Send the command complete event
            llm_cmd_cmp_send(HCI_LE_PER_ADV_TERM_SYNC_CMD_OPCODE, CO_ERROR_NO_ERROR);
        }
        else if (llm_env.act_info[act_id].state == LLM_PER_SCAN_SYNCED)
        {
            // Send an HCI_LE_PER_ADV_SYNC_LOST event
            struct hci_le_per_adv_sync_lost_evt *event = KE_MSG_ALLOC(HCI_LE_EVENT, 0, HCI_LE_META_EVT_CODE, hci_le_per_adv_sync_lost_evt);
            event->subcode = HCI_LE_PER_ADV_SYNC_LOST_EVT_SUBCODE;
            event->sync_handle = BLE_ACTID_TO_SYNCHDL(act_id);
            hci_send_2_host(event);
        }

#if BLE_CONLESS_CTE_RX
        // The activity does not exist any more, so free any CTE Rx parameters
        if (llm_env.act_info[act_id].info.sync.cte_rx_params != NULL)
        {
            ke_msg_free(ke_param2msg(llm_env.act_info[act_id].info.sync.cte_rx_params));
            llm_env.act_info[act_id].info.sync.cte_rx_params = NULL;
        }
#endif // BLE_CONLESS_CTE_RX

        // Inform activity waiting for ACAD data that periodic sync is over
        if (llm_env.act_info[act_id].info.sync.acad_dest_id != TASK_NONE)
        {
            struct llm_acad_data_ind *acad_data = KE_MSG_ALLOC(LLM_ACAD_DATA_IND,
                                                  llm_env.act_info[act_id].info.sync.acad_dest_id,
                                                  TASK_LLM, llm_acad_data_ind);
            // fill information
            acad_data->sync_handle = BLE_ACTID_TO_SYNCHDL(act_id);
            acad_data->status      = (llm_env.act_info[act_id].state == LLM_PER_SCAN_STOPPING)
                                     ? CO_ERROR_CON_TERM_BY_LOCAL_HOST
                                     : CO_ERROR_CON_TIMEOUT;
            acad_data->ad_type     = llm_env.act_info[act_id].info.sync.ad_type;
            acad_data->ref_evt_cnt = 0;
            ke_msg_send(acad_data);
        }

    }
    break;

    case LLM_PER_SCAN_SYNCING:
    {
        bool free_command = true;

        // Check status
        switch (param->status)
        {
        case CO_ERROR_UNSUPPORTED_REMOTE_FEATURE:
        {
            struct hci_le_per_adv_create_sync_cmd const *host_params = (struct hci_le_per_adv_create_sync_cmd const *)llm_env.act_info[act_id].host_params;

            ASSERT_ERR(llm_env.act_info[act_id].host_params != NULL);

            if (GETB(host_params->options, PER_SYNC_FILT_USE_PAL))
            {
                lld_scan_create_sync(act_id, GETB(host_params->options, PER_SYNC_FILT_USE_PAL), 0, 0, NULL);

                free_command = false;
                continue_syncing = true;
                break;
            }
        }
        // No break
        case CO_ERROR_CON_TIMEOUT:
        {
            // Report synchronization failure to the Host
            struct hci_le_per_adv_sync_est_evt *event = KE_MSG_ALLOC(HCI_LE_EVENT, 0, HCI_LE_META_EVT_CODE, hci_le_per_adv_sync_est_evt);
            event->subcode = HCI_LE_PER_ADV_SYNC_EST_EVT_SUBCODE;
            event->status = (param->status == CO_ERROR_UNSUPPORTED_REMOTE_FEATURE) ? CO_ERROR_UNSUPPORTED_REMOTE_FEATURE : CO_ERROR_CONN_FAILED_TO_BE_EST;
            event->sync_handle = BLE_ACTID_TO_SYNCHDL(act_id);
            event->adv_sid = llm_env.act_info[act_id].info.sync.adv_sid;
            event->adv_addr_type = llm_env.act_info[act_id].info.sync.addr_type;
            memcpy(&event->adv_addr.addr[0], &llm_env.act_info[act_id].info.sync.bd_addr, BD_ADDR_LEN);
            event->phy = PHY_1MBPS_VALUE;
            event->interval = 0;
            event->adv_ca = 0;
            hci_send_2_host(event);
        }
        break;
        case CO_ERROR_NO_ERROR:
        {
            // Send the command complete event
            llm_cmd_cmp_send(HCI_LE_PER_ADV_CREATE_SYNC_CANCEL_CMD_OPCODE, CO_ERROR_NO_ERROR);
        }
        break;
        default:
        {
            ASSERT_INFO_FORCE(0, act_id, param->status);
        }
        break;
        }

        if (free_command)
        {
            // Free HCI command
            ke_msg_free(ke_param2msg(llm_env.act_info[act_id].host_params));
            llm_env.act_info[act_id].host_params = NULL;
        }
    }
    break;

#if (BLE_PAST)
    case LLM_PER_SCAN_SYNCING_FROM_SYNC_TRANSF:
    {
        // Check status
        switch (param->status)
        {
        case CO_ERROR_CON_TIMEOUT:
        {
            // Report sync establishment
            struct hci_le_per_adv_sync_transf_rec_evt *event = KE_MSG_ALLOC(HCI_LE_EVENT, BLE_LINKID_TO_CONHDL(llm_env.act_info[act_id].info.sync.ass_act_id), HCI_LE_META_EVT_CODE, hci_le_per_adv_sync_transf_rec_evt);
            event->subcode = HCI_LE_PER_ADV_SYNC_TRANSF_REC_EVT_SUBCODE;
            event->status = CO_ERROR_CONN_FAILED_TO_BE_EST;
            event->conhdl = BLE_LINKID_TO_CONHDL(llm_env.act_info[act_id].info.sync.ass_act_id);
            event->sync_hdl = BLE_ACTID_TO_SYNCHDL(act_id);
            event->adv_sid = llm_env.act_info[act_id].info.sync.adv_sid;
            event->phy = PHY_1MBPS_VALUE;
            event->interval = 0;
            event->adv_ca = 0;
            event->serv_data = llm_env.act_info[act_id].info.sync.serv_data;
            event->adv_addr = llm_env.act_info[act_id].info.sync.bd_addr;
            event->adv_addr_type = llm_env.act_info[act_id].info.sync.addr_type;
            hci_send_2_host(event);
        }
        break;
        default:
        {
            ASSERT_INFO_FORCE(0, act_id, param->status);
        }
        break;
        }
    }
    break;
#endif // (BLE_PAST)

    default:
        ASSERT_ERR_FORCE(0);
        break;
    }

    if (!continue_syncing)
    {
        // Free activity ID
        llm_env.act_info[act_id].state = LLM_FREE;
    }

    return (KE_MSG_CONSUMED);
}

KE_MSG_HANDLER_NO_STATIC(lld_scan_end_ind, struct lld_scan_end_ind)
{
    uint8_t act_id = param->act_id;

#if (BLE_ADV_LEGACY_ITF)
    if (llm_env.adv_itf_version == LLM_ADV_ITF_LEGACY)
    {
        // Check if Host requested to stop
        if (llm_env.act_info[act_id].state == LLM_SCAN_STOPPING)
        {
            // Report the completion of the command
            llm_cmd_cmp_send(HCI_LE_SET_SCAN_EN_CMD_OPCODE, CO_ERROR_NO_ERROR);

            // Set scanning disabled
            llm_env.act_info[act_id].state = LLM_SCAN_RSVD;
        }
        else
        {
            ASSERT_ERR_FORCE(0);
        }
    }
    else
#endif //(BLE_ADV_LEGACY_ITF)
    {
        // Check if Host requested to stop
        if (llm_env.act_info[act_id].state == LLM_SCAN_STOPPING)
        {
            // Report the completion of the command
            llm_cmd_cmp_send(HCI_LE_SET_EXT_SCAN_EN_CMD_OPCODE, CO_ERROR_NO_ERROR);

            // Set scanning disabled
            llm_env.act_info[act_id].state = LLM_SCAN_RSVD;
        }
        else if (llm_env.act_info[act_id].info.scan.period)
        {
            // Set a timer for starting the next period
            ke_timer_set(LLM_SCAN_PERIOD_TO, TASK_LLM, ((llm_env.act_info[act_id].info.scan.period * 1280) - (llm_env.act_info[act_id].info.scan.duration * 10)));
        }
        else
        {
            // Indicate scan timeout to the Host
            struct hci_le_scan_timeout_evt *event = KE_MSG_ALLOC(HCI_LE_EVENT, 0, HCI_LE_META_EVT_CODE, hci_le_scan_timeout_evt);
            event->subcode = HCI_LE_SCAN_TIMEOUT_EVT_SUBCODE;
            hci_send_2_host(event);

            // Set scanning disabled
            llm_env.act_info[act_id].state = LLM_SCAN_RSVD;
        }
    }

    // Check if duplicate filter needs to be reset
    if ((llm_env.act_info[act_id].state == LLM_SCAN_RSVD) || GETB(llm_env.adv_dup_filt_info, LLM_ADV_DUP_FILT_RST))
    {
        // Flush advertising reports list
        llm_env.dup_filt.nb_adv = 0;
        llm_env.dup_filt.curr_pos = 0;
    }

    return (KE_MSG_CONSUMED);
}

#if BLE_CONLESS_CTE_RX
/**
 ****************************************************************************************
 * @brief Handles the connectionless CTE RX indication message.
 ****************************************************************************************
 */
KE_MSG_HANDLER_NO_STATIC(lld_conless_cte_rx_ind, struct lld_conless_cte_rx_ind)
{
    bool send_event = false;
    uint8_t act_id = param->act_id;

    if (((llm_env.act_info[act_id].state == LLM_PER_SCAN_SYNCED) || (llm_env.act_info[act_id].state == LLM_PER_SCAN_STOPPING)) && (llm_env.act_info[act_id].info.sync.cte_rx_params != NULL))
    {
        if (llm_env.act_info[act_id].info.sync.cte_rx_params->sampl_en)
        {
            send_event = true;
        }
    }
    else if (llm_env.act_info[act_id].state == LLM_FREE) // Test mode
    {
        send_event = true;
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    if (send_event)
    {
        // Send HCI LE connectionless IQ report event
        struct hci_le_conless_iq_report_evt *evt = KE_MSG_ALLOC(HCI_LE_EVENT, 0, HCI_LE_META_EVT_CODE, hci_le_conless_iq_report_evt);
        evt->subcode            = HCI_LE_CONLESS_IQ_REPORT_EVT_SUBCODE;
        // The sync handle for a receiver test is 0x0FFF
        evt->sync_hdl           = (llm_env.act_info[act_id].state != LLM_FREE) ? BLE_ACTID_TO_SYNCHDL(param->act_id) : RX_TEST_SYNC_HDL;
        evt->channel_idx        = param->channel_idx;
        evt->rssi               = param->rssi;
        evt->rssi_antenna_id    = param->rssi_antenna_id;
        evt->cte_type           = param->cte_type;
        evt->slot_dur           = param->slot_dur;
        evt->pkt_status         = CTE_PKT_STAT_CRC_OK;
        evt->pa_evt_cnt         = param->pa_evt_cnt;
        evt->sample_cnt         = param->sample_cnt;
        em_rd(&evt->iq_sample[0], REG_EM_BLE_RX_CTE_DESC_ADDR_GET(param->em_rx_cte_desc_idx) + (EM_BLE_RXCTESAMPBUF_INDEX * 2), param->sample_cnt * 2);
        hci_send_2_host(evt);
    }

    // Release the Rx CTE descriptor
    em_ble_rxctecntl_rxdone_setf(param->em_rx_cte_desc_idx, 0);

    return (KE_MSG_CONSUMED);
}
#endif // BLE_CONLESS_CTE_RX

/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

uint8_t llm_scan_sync_acad_attach(uint8_t sync_act_id, uint8_t ad_type, uint16_t task)
{
    uint8_t status;

    do
    {
        // search activity of the periodic sync
        if ((sync_act_id >= BLE_ACTIVITY_MAX_CFG) || (llm_env.act_info[sync_act_id].state != LLM_PER_SCAN_SYNCED))
        {
            status = CO_ERROR_UNKNOWN_ADVERTISING_ID;
            break;
        }

        // Attach to the ACAD data in periodic advertising report
        llm_env.act_info[sync_act_id].info.sync.ad_type      = ad_type;
        llm_env.act_info[sync_act_id].info.sync.acad_dest_id = task;

        status = CO_ERROR_NO_ERROR;
    }
    while (0);

    return status;
}

#if (BLE_PAST)
uint8_t llm_scan_sync_info_get(uint8_t sync_act_id, uint8_t *sid, uint8_t *atype, struct bd_addr *adva)
{
    uint8_t status = CO_ERROR_UNKNOWN_ADVERTISING_ID;

    do
    {
        // search activity of the periodic sync
        if ((sync_act_id >= BLE_ACTIVITY_MAX_CFG) || (llm_env.act_info[sync_act_id].state != LLM_PER_SCAN_SYNCED))
            break;

        if (sid != NULL)
            *sid = llm_env.act_info[sync_act_id].info.sync.adv_sid;

        if (atype != NULL)
        {
            if (llm_env.act_info[sync_act_id].info.sync.addr_type & ADDR_RPA_MASK)
            {
                *atype = ADDR_RAND;
            }
            else
            {
                *atype = llm_env.act_info[sync_act_id].info.sync.addr_type;
            }
        }

        if (adva != NULL)
        {
            if (llm_env.act_info[sync_act_id].info.sync.addr_type & ADDR_RPA_MASK)
            {
                *adva = llm_env.act_info[sync_act_id].info.sync.peer_rpa;
            }
            else
            {
                *adva = llm_env.act_info[sync_act_id].info.sync.bd_addr;
            }
        }

        status = CO_ERROR_NO_ERROR;
    }
    while (0);

    return (status);
}

uint8_t llm_per_adv_sync(struct llm_per_adv_sync_params *params)
{
    uint8_t status;

    do
    {
        uint8_t act_id;
        struct lld_sync_params sync_params;

        // Check if device is not already synced
        if (llm_is_dev_synced(&params->adv_addr, params->adv_addr_type, params->adv_sid))
        {
            status = CO_ERROR_CON_ALREADY_EXISTS;
            break;
        }

        // Allocate link identifier
        status = llm_activity_free_get(&act_id);

        // Check if link identifier found
        if (status != CO_ERROR_NO_ERROR)
            break;

        llm_env.act_info[act_id].state = LLM_PER_SCAN_SYNCING_FROM_SYNC_TRANSF;

        // Keep reference to the connection
        llm_env.act_info[act_id].info.sync.ass_act_id = params->act_id;
        llm_env.act_info[act_id].info.sync.serv_data = params->id;
        llm_env.act_info[act_id].info.sync.bd_addr = params->adv_addr;
        llm_env.act_info[act_id].info.sync.addr_type = params->adv_addr_type;
        llm_env.act_info[act_id].info.sync.peer_rpa = params->adv_rpa;
        llm_env.act_info[act_id].info.sync.adv_sid = params->adv_sid;
        llm_env.act_info[act_id].info.sync.skip = params->skip;

        if (memcmp(&params->adv_rpa, &co_null_bdaddr, BD_ADDR_LEN))
        {
            // Indicate address as RPA
            llm_env.act_info[act_id].info.sync.addr_type |= ADDR_RPA_MASK;
        }

        // Start Sync driver.
        sync_params.p_syncinfo = &params->syncinfo;
        sync_params.base_cnt = params->base_cnt;
        sync_params.fine_cnt = params->fine_cnt;
        sync_params.rate = params->rate;
        sync_params.skip = params->skip;
        sync_params.sync_to = params->sync_to;
        sync_params.add_drift = params->add_drift;
        sync_params.sync_cte_type = params->sync_cte_type;
        sync_params.adv_addr = llm_env.act_info[act_id].info.sync.bd_addr;
        sync_params.rep_en = params->adv_rep_en;

#if (BT_53)
        sync_params.adv_sid = params->adv_sid;
        sync_params.adi_filt_en = params->adi_filt_en;
#endif // (BT_53)

        status = lld_sync_start(act_id, &sync_params);

    }
    while (0);

    return status;
}
#endif // (BLE_PAST)

uint8_t llm_scan_sync_acad_detach(uint8_t sync_act_id)
{
    uint8_t status;

    do
    {
        // search activity of the periodic sync
        if ((sync_act_id >= BLE_ACTIVITY_MAX_CFG) || (llm_env.act_info[sync_act_id].state != LLM_PER_SCAN_SYNCED))
        {
            status = CO_ERROR_UNKNOWN_CONNECTION_ID;
            break;
        }

        // remove ACAD data watcher
        llm_env.act_info[sync_act_id].info.sync.acad_dest_id = TASK_NONE;

        status = CO_ERROR_NO_ERROR;
    }
    while (0);

    return status;
}

#endif //(BLE_OBSERVER)

/// @} LLM
