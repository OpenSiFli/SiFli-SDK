/**
 ****************************************************************************************
 *
 * @file llm_ch_scan.c
 *
 * @brief LLM Channel Scanning source file
 *
 * Copyright (C) RivieraWaves 2009-2021
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup LLMCHSCAN
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"    // stack configuration
#if (BLE_CH_SCAN_SUPPORT)
#include <string.h>

#include "ke_task.h"        // kernel task definitions

#include "llm.h"            // link layer manager definitions
#include "lld.h"            // link layer driver definitions

#include "llm_int.h"        // link layer manager internal definitions

#if HCI_PRESENT
    #include "hci.h"            // host controller interface
#endif //HCI_PRESENT


/*
 * LOCAL FUNCTIONS DEFINITION
 ****************************************************************************************
 */

/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */
KE_MSG_HANDLER_NO_STATIC(lld_ch_scan_end_ind, struct lld_ch_scan_end_ind)
{
    // Send the complete event
    struct hci_vs_le_ch_scan_end_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, HCI_VS_LE_CH_SCAN_END_CMD_OPCODE, hci_vs_le_ch_scan_end_cmd_cmp_evt);
    event->status = param->status;
    hci_send_2_host(event);

    // Free activity ID
    llm_env.act_info[param->act_id].state = LLM_FREE;

    return (KE_MSG_CONSUMED);
}


/*
 * HCI COMMAND HANDLERS
 ****************************************************************************************
 */

int hci_vs_le_ch_scan_cmd_handler(struct hci_vs_le_ch_scan_cmd const *param, uint16_t opcode)
{
    // Status returned in the command complete event
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;
    uint8_t act_id;

    do
    {
        struct lld_ch_scan_params ch_scan_params;

        // Ensure Channel Scanning is not already active
        for (act_id = 0; act_id < BLE_ACTIVITY_MAX_CFG; act_id++)
        {
            if (llm_env.act_info[act_id].state == LLM_CH_SCAN_EN)
                break;
        }
        if (act_id != BLE_ACTIVITY_MAX_CFG)
            break;

        status = llm_activity_free_get(&act_id);

        // If not possible to start a new channel scanning activity, reject the command
        if (status != CO_ERROR_NO_ERROR)
            break;

        // Assign the allocated activity identifier to initiating
        llm_env.act_info[act_id].state = LLM_CH_SCAN_EN;

        ch_scan_params.win_duration = param->win_duration;
        ch_scan_params.scan_duration = param->scan_duration;
        ch_scan_params.intv = param->intv;
        ch_scan_params.ch_sel = param->ch_sel;

        if (param->ch_sel == CH_SCAN_INACTIVE_CH)
        {
            // Compute channel map
            llm_ch_map_compute(&ch_scan_params.ch_map);

            // Update channel map of the channel scanner (only inactive channels are selected)
            for (int i = 0; i < LE_CH_MAP_LEN ; i++)
            {
                ch_scan_params.ch_map.map[i] = ~(ch_scan_params.ch_map.map[i]);
            }
            ch_scan_params.ch_map.map[LE_CH_MAP_LEN - 1] &= 0x1F;

            // Enable channel map monitoring
            llm_ch_map_timer_enable();
        }
        else
        {
            // Start with all channels marked as enabled
            memset(&ch_scan_params.ch_map.map[0], 0xFF, LE_CH_MAP_LEN);
            ch_scan_params.ch_map.map[LE_CH_MAP_LEN - 1] &= 0x1F;
        }
        status = lld_ch_scan_start(act_id, &ch_scan_params);
        if (status != CO_ERROR_NO_ERROR)
        {
            llm_env.act_info[act_id].state = LLM_FREE;
        }
    }
    while (0);

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

int hci_vs_le_ch_scan_end_cmd_handler(void const *param, uint16_t opcode)
{
    int act_id;

    // Ensure Channel Scanning is active
    for (act_id = 0; act_id < BLE_ACTIVITY_MAX_CFG; act_id++)
    {
        if (llm_env.act_info[act_id].state == LLM_CH_SCAN_EN)
            break;
    }

    if (act_id < BLE_ACTIVITY_MAX_CFG)
    {
        lld_ch_scan_stop();
    }
    else
    {
        // allocate the complete event message
        struct hci_basic_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_basic_cmd_cmp_evt);
        // update the status
        evt->status = CO_ERROR_COMMAND_DISALLOWED;
        // send the message
        hci_send_2_host(evt);
    }

    return (KE_MSG_CONSUMED);
}

#endif //(BLE_CH_SCAN_SUPPORT)
/// @} LLMCHSCAN
