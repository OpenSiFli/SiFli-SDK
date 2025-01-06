/**
 ****************************************************************************************
 *
 * @file lli.c
 *
 * @brief Definition of the functions used by the Link Layer ISO
 *
 * Copyright (C) RivieraWaves 2009-2017
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup LLI
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"
#if (BLE_ISO_PRESENT)
#include "rwip.h"         // IP definitions
#include <string.h>       // For memset definition
#include "lli_int.h"      // Internal LLI API
#include "lli.h"          // LLI API
#include "ke_task.h"      // For LLI task creation
#include "ke_mem.h"       // For memory allocation
#include "llm.h"
#include "hci.h"          // For HCI definitions

#include "data_path.h"    // Isochronous data path manager
#include "co_utils.h"     // Utils functions
#include "btdm_patch.h"
/*
 * DEFINES
 ****************************************************************************************
 */
/// Margin added by default to the distance between sub-events (in us)
#define LLI_DFLT_SUB_EVT_MARGIN 0

/*
 * TYPES DEFINITION
 *****************************************************************************************
 */


/*
 * GLOBAL VARIABLE DEFINITION
 *****************************************************************************************
 */

/// Task definition
extern const struct ke_task_desc TASK_DESC_LLI;

/// Isochronous environment
struct lli_env_tag lli_env;

/*
 * MODULE INTERNAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */


/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void lli_init(bool is_reset)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLI_PATCH_TYPE, LLI_INIT_FUN_BIT, is_reset);
    // Channel handle
    uint16_t hdl;

#if (BLE_ISO_MODE_0)
    // Initialize AM0 module
    lli_am0_init(is_reset);
#endif // (BLE_ISO_MODE_0)

#if (BLE_BIS)
    // Initialize BI module
    lli_bi_init(is_reset);
#endif // (BLE_BIS)

#if (BLE_CIS)
    // Initialize CI module
    lli_ci_init(is_reset);
#endif // (BLE_CIS)

    if (!is_reset)
    {
        // Create LLI task
        ke_task_create(TASK_LLI, &TASK_DESC_LLI);

#if (BLE_BIS || BLE_CIS)
        // Set all environment pointers to NULL
        memset(&lli_env.group[0], 0, sizeof(lli_env.group));
#endif // (BLE_BIS || BLE_CIS)
    }
    else
    {
#if (BLE_BIS || BLE_CIS)
        // Clean all allocated ISO environments
        for (hdl = 0; hdl < BLE_ISO_GROUP_MAX ; hdl++)
        {
            // Free the Group
            lli_group_cleanup(hdl);
        }
#endif // (BLE_BIS || BLE_CIS)
    }

    // default value to used that can be updated over a vendor specific command
#if (BLE_BIS || BLE_CIS)
    lli_env.sub_evt_margin = LLI_DFLT_SUB_EVT_MARGIN;
#endif // (BLE_BIS || BLE_CIS)

    // Initialize data path ISO driver
    data_path_init(is_reset);
}

#if (BLE_BIS || BLE_CIS)
uint8_t lli_group_create(uint8_t iso_type, uint16_t size, struct lli_group_env **pp_env)
{
    FUNC_PATCH_ENTRY_3_PARAM_HAVE_RETURN(LLI_PATCH_TYPE, LLI_GROUP_CREATE_FUN_BIT, uint8_t, iso_type, size, pp_env);
    uint8_t  grp_hdl = BLE_INVALID_GROUP_HDL;
    uint8_t  status = CO_ERROR_NO_ERROR;
    uint16_t cur;

    // search for available stream handle
    for (cur = 0;  cur < BLE_ISO_GROUP_MAX;  cur++)
    {
        // Check if channel handle is available and if activity is also available
        if (!lli_env.group[cur])
        {
            grp_hdl = cur;
            break;
        }
    }

    // Check if channel allocation succeed
    if (grp_hdl == BLE_INVALID_GROUP_HDL)
    {
        status = CO_ERROR_CONN_REJ_LIMITED_RESOURCES;
    }
    else
    {
        // Allocated environment
        struct lli_group_env *p_env = (struct lli_group_env *)ke_malloc_system(size, KE_MEM_ENV);

        // Clean the content of the structure
        memset(p_env, 0, size);

        // Initialize ISO parameters
        p_env->type      = iso_type;
        p_env->hdl       = grp_hdl;

        // Set pointer to the allocated environment
        lli_env.group[grp_hdl] = p_env;
        *pp_env = p_env;
    }

    // Return Status of channel allocation
    return (status);
}

void lli_group_cleanup(uint16_t grp_hdl)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLI_PATCH_TYPE, LLI_GROUP_CLEANUP_FUN_BIT, grp_hdl);
    ASSERT_INFO((grp_hdl < BLE_ISO_GROUP_MAX), grp_hdl, 0);

    // Get indicated environment for indicated channel
    if (grp_hdl < BLE_ISO_GROUP_MAX)
    {
        if (lli_env.group[grp_hdl] != NULL)
        {
            // Free environment
            ke_free(lli_env.group[grp_hdl]);

            // Clean environment pointer
            lli_env.group[grp_hdl] = NULL;
        }
    }
}

uint8_t lli_group_env_get(uint16_t grp_hdl, uint8_t iso_type, struct lli_group_env **pp_env)
{
    FUNC_PATCH_ENTRY_3_PARAM_HAVE_RETURN(LLI_PATCH_TYPE, LLI_GROUP_ENV_GET_FUN_BIT, uint8_t, grp_hdl, iso_type, pp_env);
    // Status
    uint8_t status;

    do
    {
        struct lli_group_env *p_env;

        // Check provided channel handle
        if (grp_hdl >= BLE_ISO_GROUP_MAX)
        {
            status = CO_ERROR_INVALID_HCI_PARAM;
            break;
        }

        // Get environment for indicated CIS Channel
        p_env = lli_env.group[grp_hdl];

        // Check if ISO Channel well exists and is an expected chanel type
        if ((!p_env) || ((iso_type != LLI_ISO_GROUP_UNDEF) && (p_env->type != iso_type)))
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        // Can return the environment
        *pp_env = p_env;

        status = CO_ERROR_NO_ERROR;
    }
    while (0);

    // Return status
    return (status);
}
#endif // (BLE_BIS || BLE_CIS)

void lli_link_stop_ind(uint8_t link_id, uint8_t reason)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LLI_PATCH_TYPE, LLI_LINK_STOP_IND_FUN_BIT, link_id, reason);
#if (BLE_CIS)
    // Inform that connection has been stopped
    lli_cis_link_stop_ind(link_id, reason);
#endif // (BLE_CIS)

#if (BLE_ISO_MODE_0)
    // Inform that connection has been stopped
    lli_am0_link_stop_ind(link_id, reason);
#endif // (BLE_ISO_MODE_0)
}

#if ((RW_DEBUG) && (BLE_BIS || BLE_CIS))
int hci_dbg_iso_set_param_cmd_handler(struct hci_dbg_iso_set_param_cmd const *p_param, uint16_t opcode)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLI_PATCH_TYPE, HCI_DBG_ISO_SET_PARAM_CMD_HANDLER_FUN_BIT, int, p_param, opcode);
    // Allocate the status event to be sent to the host
    struct hci_basic_cmd_cmp_evt *p_evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_basic_cmd_cmp_evt);
    p_evt->status = CO_ERROR_NO_ERROR;

    // Store the given parameter
    lli_env.sub_evt_margin = p_param->sub_evt_margin;

    // Send the command complete event
    hci_send_2_host(p_evt);

    // Message can be consumed
    return (KE_MSG_CONSUMED);
}
#endif //((RW_DEBUG) && (BLE_BIS || BLE_CIS))

#if (BLE_BIS || BLE_CIS)
int hci_le_rd_iso_link_quality_cmd_handler(struct hci_le_rd_iso_link_quality_cmd const *param, uint16_t opcode)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLI_PATCH_TYPE, HCI_LE_RD_ISO_LINK_QUALITY_CMD_HANDLER_FUN_BIT, int, param, opcode);
    // Allocate HCI message
    struct hci_le_rd_iso_link_quality_cmd_cmp_evt *p_evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_le_rd_iso_link_quality_cmd_cmp_evt);

    // Command status
    uint8_t status = CO_ERROR_UNKNOWN_CONNECTION_ID;

    // Initialize all counters
    memset(p_evt, 0, sizeof(struct hci_le_rd_iso_link_quality_cmd_cmp_evt));

    do
    {
#if (BLE_BIS)
        // If stream is a BIS
        if (param->conhdl >= BLE_BISHDL_MIN)
        {
            uint8_t act_id = BLE_BISHDL_TO_ACTID(param->conhdl);

            if (act_id < BLE_ACTIVITY_MAX)
            {
                // Read BIS statistics
                status = lli_bis_stats_get(act_id, &p_evt->crc_error_packets, &p_evt->rx_unreceived_packets);
            }
            break;
        }
#endif // (BLE_BIS)

#if (BLE_CIS)
        // If stream is a CIS
        if (param->conhdl >= BLE_CISHDL_MIN)
        {
            uint8_t act_id = BLE_CISHDL_TO_ACTID(param->conhdl);

            if (act_id < BLE_ACTIVITY_MAX)
            {
                // Read CIS statistics
                status = lli_cis_stats_get(act_id, &p_evt->tx_unacked_packets, &p_evt->tx_flushed_packets, &p_evt->tx_last_subevent_packets,
                                           &p_evt->retransmitted_packets, &p_evt->crc_error_packets, &p_evt->rx_unreceived_packets,
                                           &p_evt->duplicate_packets);
            }
            break;
        }
#endif // (BLE_CIS)

    }
    while (0);

    p_evt->status = status;
    p_evt->conhdl = param->conhdl;

    // Send the message
    hci_send_2_host(p_evt);

    return (KE_MSG_CONSUMED);
}
#endif // (BLE_BIS || BLE_CIS)

#endif // (BLE_ISO_PRESENT)
/// @} LLI
