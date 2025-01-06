/**
 ****************************************************************************************
 *
 * @file llm_hci.c
 *
 * @brief LLM HCI source file
 *
 * Copyright (C) RivieraWaves 2009-2017
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup LLMHCI
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"    // stack configuration

#include <string.h>
#include "em_map.h"
#include "co_math.h"
#include "co_bt.h"          // BLE standard definitions
#include "co_version.h"
#include "co_utils.h"
#include "ble_util.h"       // BLE utility functions

#include "ke_mem.h"         // kernel memory definitions
#include "ke_timer.h"       // kernel timer definitions
#include "ke_msg.h"         // kernel message definitions
#include "ke_task.h"        // kernel task definitions

#include "llm.h"            // link layer manager definitions
#include "llc.h"            // link layer controller
#include "lld.h"            // link layer driver
#include "llm_int.h"        // link layer manager internal definitions

#include "rwip.h"           // stack main module

#include "sch_plan.h"           // Scheduling Planner

#if HCI_PRESENT
    #include "hci.h"            // host controller interface
#endif //HCI_PRESENT

#include "ecc_p256.h"

#include "aes.h"            // AES encryption function

#include "data_path.h"      // Data Path API

#include "dbg.h"

#if (NVDS_SUPPORT)
    #include "sifli_nvds.h"
#endif


/*
 * DEFINES
 ****************************************************************************************
 */


/*
 * TYPES DEFINITION
 ****************************************************************************************
 */

/// Format of a HCI command handler function
typedef int (*llm_hci_cmd_hdl_func_t)(void const *param, uint16_t opcode);


/*
 * STRUCT DEFINITION
 ****************************************************************************************
 */

/// Element of a HCI command handler table.
struct llm_hci_cmd_handler
{
    /// Command opcode
    uint16_t opcode;
    /// Pointer to the handler function for HCI command.
    llm_hci_cmd_hdl_func_t func;
};


/*
 * CONSTANT DEFINITION
 ****************************************************************************************
 */

//#if (!BT_DUAL_MODE && HCI_TL_SUPPORT)
#if (HCI_TL_SUPPORT)

/// Local supported commands
__STATIC uint8_t const llm_local_supp_cmds[] =
{
    BLE_CMDS_BYTE0,
    0x00,
    BLE_CMDS_BYTE2,
    0x00,
    0x00,
    BLE_CMDS_BYTE5,
    0x00,
    0x00,
    0x00,
    0x00,
    BLE_CMDS_BYTE10,
    0x00,
    0x00,
    0x00,
    BLE_CMDS_BYTE14,
    BLE_CMDS_BYTE15,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    BLE_CMDS_BYTE22,
    0x00,
    0x00,
    BLE_CMDS_BYTE25,
    BLE_CMDS_BYTE26,
    BLE_CMDS_BYTE27,
    BLE_CMDS_BYTE28,
    0x00,
    0x00,
    0x00,
    BLE_CMDS_BYTE32,
    BLE_CMDS_BYTE33,
    BLE_CMDS_BYTE34,
    BLE_CMDS_BYTE35,
    BLE_CMDS_BYTE36,
    BLE_CMDS_BYTE37,
    BLE_CMDS_BYTE38,
    BLE_CMDS_BYTE39,
    BLE_CMDS_BYTE40,
    BLE_CMDS_BYTE41,
    BLE_CMDS_BYTE42,
    BLE_CMDS_BYTE43,
    BLE_CMDS_BYTE44,
    BLE_CMDS_BYTE45,
#if (BT_53)
    BLE_CMDS_BYTE46,
#endif // (BT_53)
};
#endif // (!BT_DUAL_MODE && HCI_TL_SUPPORT)

/// Local LE supported states
__STATIC const struct le_states llm_local_le_states =
{
    {
        BLE_STATES_BYTE0, BLE_STATES_BYTE1, BLE_STATES_BYTE2, BLE_STATES_BYTE3,
        BLE_STATES_BYTE4, BLE_STATES_BYTE5, BLE_STATES_BYTE6, BLE_STATES_BYTE7
    }
};


/*
 * ENUMERATIONS DEFINITION
 ****************************************************************************************
 */


/*
 * MESSAGE HANDLERS DECLARATION
 ****************************************************************************************
 */
#if (BLE_BROADCASTER)
    extern int hci_le_set_adv_param_cmd_handler(struct hci_le_set_adv_param_cmd const *param, uint16_t opcode);
    extern int hci_le_rd_adv_ch_tx_pw_cmd_handler(void const *param, uint16_t opcode);
    extern int hci_le_set_adv_data_cmd_handler(struct hci_le_set_adv_data_cmd const *param, uint16_t opcode);
    extern int hci_le_set_adv_en_cmd_handler(struct hci_le_set_adv_en_cmd const *param, uint16_t opcode);
    extern int hci_le_set_scan_rsp_data_cmd_handler(struct hci_le_set_scan_rsp_data_cmd const *param, uint16_t opcode);
    extern int hci_le_set_adv_set_rand_addr_cmd_handler(struct hci_le_set_adv_set_rand_addr_cmd const *param, uint16_t opcode);
    extern int hci_le_set_ext_adv_param_cmd_handler(struct hci_le_set_ext_adv_param_cmd const *param, uint16_t opcode);
    extern int hci_le_set_ext_adv_data_cmd_handler(struct hci_le_set_ext_adv_data_cmd const *param, uint16_t opcode);
    extern int hci_le_set_ext_scan_rsp_data_cmd_handler(struct hci_le_set_ext_scan_rsp_data_cmd const *param, uint16_t opcode);
    extern int hci_le_set_ext_adv_en_cmd_handler(struct hci_le_set_ext_adv_en_cmd const *param, uint16_t opcode);
    #if (BT_53)
        extern int hci_le_set_data_related_addr_chg_cmd_handler(struct hci_le_set_data_related_addr_chg_cmd const *param, uint16_t opcode);
    #endif // (BT_53)
    #if (HCI_TL_SUPPORT)
        extern int hci_le_rd_max_adv_data_len_cmd_handler(void const *param, uint16_t opcode);
        extern int hci_le_rd_nb_supp_adv_sets_cmd_handler(void const *param, uint16_t opcode);
    #endif // (HCI_TL_SUPPORT)
    extern int hci_le_rmv_adv_set_cmd_handler(void const *param, uint16_t opcode);
    extern int hci_le_clear_adv_sets_cmd_handler(void const *param, uint16_t opcode);
    extern int hci_le_set_per_adv_param_cmd_handler(struct hci_le_set_per_adv_param_cmd const *param, uint16_t opcode);
    extern int hci_le_set_per_adv_data_cmd_handler(struct hci_le_set_per_adv_data_cmd const *param, uint16_t opcode);
    extern int hci_le_set_per_adv_en_cmd_handler(struct hci_le_set_per_adv_en_cmd const *param, uint16_t opcode);
    #if BLE_CONLESS_CTE_TX
        extern int hci_le_set_conless_cte_tx_param_cmd_handler(struct hci_le_set_conless_cte_tx_param_cmd const *param, uint16_t opcode);
        extern int hci_le_set_conless_cte_tx_en_cmd_handler(struct hci_le_set_conless_cte_tx_en_cmd const *param, uint16_t opcode);
    #endif // BLE_CONLESS_CTE_TX
#endif //(BLE_BROADCASTER)

#if (BLE_OBSERVER)
    extern int hci_le_set_scan_param_cmd_handler(struct hci_le_set_scan_param_cmd const *param, uint16_t opcode);
    extern int hci_le_set_scan_en_cmd_handler(struct hci_le_set_scan_en_cmd const *param, uint16_t opcode);
    extern int hci_le_set_ext_scan_param_cmd_handler(struct hci_le_set_ext_scan_param_cmd const *param, uint16_t opcode);
    extern int hci_le_set_ext_scan_en_cmd_handler(struct hci_le_set_ext_scan_en_cmd const *param, uint16_t opcode);
    extern int hci_le_per_adv_create_sync_cmd_handler(struct hci_le_per_adv_create_sync_cmd const *param, uint16_t opcode);
    extern int hci_le_per_adv_create_sync_cancel_cmd_handler(void const *param, uint16_t opcode);
    extern int hci_le_per_adv_term_sync_cmd_handler(struct hci_le_per_adv_term_sync_cmd const *param, uint16_t opcode);
    extern int hci_le_add_dev_to_per_adv_list_cmd_handler(struct hci_le_add_dev_to_per_adv_list_cmd const *param, uint16_t opcode);
    extern int hci_le_rmv_dev_from_per_adv_list_cmd_handler(struct hci_le_rmv_dev_from_per_adv_list_cmd const *param, uint16_t opcode);
    extern int hci_le_clear_per_adv_list_cmd_handler(void const *param, uint16_t opcode);
    extern int hci_le_rd_per_adv_list_size_cmd_handler(void const *param, uint16_t opcode);
    extern int hci_le_set_per_adv_rec_en_cmd_handler(struct hci_le_set_per_adv_rec_en_cmd const *param, uint16_t opcode);
    #if BLE_CONLESS_CTE_RX
        extern int hci_le_set_conless_iq_sampl_en_cmd_handler(struct hci_le_set_conless_iq_sampl_en_cmd const *param, uint16_t opcode);
    #endif // BLE_CONLESS_CTE_RX
#endif //(BLE_OBSERVER)

#if (BLE_CENTRAL)
    extern int hci_le_create_con_cmd_handler(struct hci_le_create_con_cmd const *param, uint16_t opcode);
    extern int hci_le_create_con_cancel_cmd_handler(void const *param, uint16_t opcode);
    extern int hci_le_ext_create_con_cmd_handler(struct hci_le_ext_create_con_cmd const *param, uint16_t opcode);
#endif //(BLE_CENTRAL)

#if (BLE_TEST_MODE_SUPPORT)
    extern int hci_le_rx_test_v1_cmd_handler(struct hci_le_rx_test_v1_cmd const *param, uint16_t opcode);
    extern int hci_le_tx_test_v1_cmd_handler(struct hci_le_tx_test_v1_cmd const *param, uint16_t opcode);
    extern int hci_le_rx_test_v2_cmd_handler(struct hci_le_rx_test_v2_cmd const *param, uint16_t opcode);
    extern int hci_le_tx_test_v2_cmd_handler(struct hci_le_tx_test_v2_cmd const *param, uint16_t opcode);
    extern int hci_le_rx_test_v3_cmd_handler(struct hci_le_rx_test_v3_cmd const *param, uint16_t opcode);
    extern int hci_le_tx_test_v3_cmd_handler(struct hci_le_tx_test_v3_cmd const *param, uint16_t opcode);
    extern int hci_le_tx_test_v4_cmd_handler(struct hci_le_tx_test_v4_cmd const *param, uint16_t opcode);
    extern int hci_le_test_end_cmd_handler(void const *param, uint16_t opcode);
    #if (EAVESDROPPING_SUPPORT)
        extern int hci_vs_le_set_cust_transmitter_test_cmd_handler(struct hci_vs_le_set_cust_transmitter_test_cmd const *param, uint16_t opcode);
    #endif //(EAVESDROPPING_SUPPORT)
#endif //(BLE_TEST_MODE_SUPPORT)

#if (EAVESDROPPING_SUPPORT)
    #if (BLE_BROADCASTER || BLE_PERIPHERAL)
        extern int hci_vs_set_cust_adv_cmd_handler(struct hci_vs_set_cust_adv_cmd const *param, uint16_t opcode);
        extern int hci_vs_le_set_manual_adv_cmd_handler(struct hci_vs_le_set_manual_adv_cmd const *param, uint16_t opcode);
        extern int hci_vs_le_set_seg_scan_cmd_handler(struct hci_vs_le_set_seg_scan_cmd const *param, uint16_t opcode);
    #endif //(BLE_BROADCASTER || BLE_PERIPHERAL)
#endif //(EAVESDROPPING_SUPPORT)

#if (BLE_CH_SCAN_SUPPORT)
    extern int hci_vs_le_ch_scan_cmd_handler(struct hci_vs_le_ch_scan_cmd const *param, uint16_t opcode);
    extern int hci_vs_le_ch_scan_end_cmd_handler(void const *param, uint16_t opcode);
#endif //(BLE_CH_SCAN_SUPPORT)
/*
 * LOCAL FUNCTIONS DEFINITION
 ****************************************************************************************
 */

/// Check if the white list is accessible (if not in conflict with a current operation)
__STATIC bool llm_is_wl_accessible(void)
{
    bool accessible = true;
    uint8_t act_id;

    // Check all activities
    for (act_id = 0; act_id < BLE_ACTIVITY_MAX_CFG; act_id++)
    {
#if (BLE_BROADCASTER)
        if ((llm_env.act_info[act_id].state == LLM_ADV_EN) || (llm_env.act_info[act_id].state == LLM_ADV_STOPPING))
        {
            // Point Host parameters
            struct hci_le_set_ext_adv_param_cmd *ext_param = (struct hci_le_set_ext_adv_param_cmd *) llm_env.act_info[act_id].host_params;

            // Check if filter policy implies to use white list
            if (ext_param->adv_filt_policy != ADV_ALLOW_SCAN_ANY_CON_ANY)
            {
                accessible = false;
                break;
            }
        }
#endif //(BLE_BROADCASTER)

#if (BLE_OBSERVER)
        if (llm_env.act_info[act_id].state == LLM_SCAN_EN)
        {
            // Point Host parameters
            struct hci_le_set_ext_scan_param_cmd *ext_param = (struct hci_le_set_ext_scan_param_cmd *) llm_env.act_info[act_id].host_params;

            // Check if filter policy implies to use white list
            if (ext_param->scan_filt_policy != SCAN_ALLOW_ADV_ALL)
            {
                accessible = false;
                break;
            }
        }
#endif //(BLE_OBSERVER)

#if (BLE_CENTRAL)
        if (llm_env.act_info[act_id].state == LLM_INITIATING)
        {
            // Point Host parameters
            struct hci_le_ext_create_con_cmd *ext_param = (struct hci_le_ext_create_con_cmd *) llm_env.act_info[act_id].host_params;

            if (ext_param->init_filter_policy != SCAN_ALLOW_ADV_ALL)
            {
                accessible = false;
                break;
            }
        }
#endif //(BLE_CENTRAL)
    }

    return accessible;
}

/// Check if a non-connected activity is ongoing
__STATIC bool llm_is_non_con_act_ongoing_check(void)
{
    bool ongoing = false;
    uint8_t act_id;

    // Check all activities
    for (act_id = 0; act_id < BLE_ACTIVITY_MAX_CFG; act_id++)
    {
#if (BLE_BROADCASTER)
        if ((llm_env.act_info[act_id].state == LLM_ADV_EN) || (llm_env.act_info[act_id].state == LLM_ADV_STOPPING))
        {
            ongoing = true;
            break;
        }
#endif //(BLE_BROADCASTER)

#if (BLE_OBSERVER)
        if ((llm_env.act_info[act_id].state == LLM_SCAN_EN) || (llm_env.act_info[act_id].state == LLM_PER_SCAN_SYNCING)
#if (BLE_PAST)
                || (llm_env.act_info[act_id].state == LLM_PER_SCAN_SYNCING_FROM_SYNC_TRANSF)
#endif // (BLE_PAST)
           )
        {
            ongoing = true;
            break;
        }
#endif //(BLE_OBSERVER)

#if (BLE_CENTRAL)
        if (llm_env.act_info[act_id].state == LLM_INITIATING)
        {
            ongoing = true;
            break;
        }
#endif //(BLE_CENTRAL)
    }

    return ongoing;
}

#if ((HCI_TL_SUPPORT) && (BLE_CENTRAL || BLE_PERIPHERAL))
/// Check if a connected activity is ongoing
__STATIC bool llm_is_con_act_ongoing_check(void)
{
    bool ongoing = false;
    uint8_t act_id;

#if (BLE_CENTRAL || BLE_PERIPHERAL)
    // Check all activities
    for (act_id = 0; act_id < BLE_ACTIVITY_MAX_CFG; act_id++)
    {
        if (llm_env.act_info[act_id].state == LLM_CONNECTED)
        {
            ongoing = true;
            break;
        }
    }
#endif //(BLE_CENTRAL || BLE_PERIPHERAL)

    return ongoing;
}
#endif //((HCI_TL_SUPPORT) && (BLE_CENTRAL || BLE_PERIPHERAL))

#if (!BLE_HOST_PRESENT) || (HCI_TL_SUPPORT)
/// Check if the random address set by the LE Set Random Address command could be in use
__STATIC bool llm_is_rand_addr_in_use_check(void)
{
    uint8_t act_id;

    // Check all activities
    for (act_id = 0; act_id < BLE_ACTIVITY_MAX_CFG; act_id++)
    {
        // The random address set by the LE Set Random Address command is used by legacy advertising only
        // Otherwise, the random address is specified by the LE Set Advertising Set Random Address command
#if (BLE_BROADCASTER && BLE_ADV_LEGACY_ITF)
        // Check current interface version
        if (((llm_env.act_info[act_id].state == LLM_ADV_EN) || (llm_env.act_info[act_id].state == LLM_ADV_STOPPING))
                && (llm_env.adv_itf_version == LLM_ADV_ITF_LEGACY))
        {
            break;
        }
#endif // (BLE_BROADCASTER && BLE_ADV_LEGACY_ITF)

#if (BLE_OBSERVER)
        if (llm_env.act_info[act_id].state == LLM_SCAN_EN)
        {
            break;
        }
#endif //(BLE_OBSERVER)

#if (BLE_CENTRAL)
        if (llm_env.act_info[act_id].state == LLM_INITIATING)
        {
            break;
        }
#endif //(BLE_CENTRAL)
    }

    return (act_id < BLE_ACTIVITY_MAX_CFG);
}
#endif // (!BLE_HOST_PRESENT) || (HCI_TL_SUPPORT)


/*
 * HCI COMMAND HANDLERS
 ****************************************************************************************
 */

//#if !BT_DUAL_MODE
#if (HCI_TL_SUPPORT)
/// Handles the command HCI reset
__STATIC int hci_reset_cmd_handler(void const *param, uint16_t opcode)
{
    // Reset BLE
    rwip_reset();
    // Send the command complete event
    llm_cmd_cmp_send(opcode, CO_ERROR_NO_ERROR);

    return (KE_MSG_CONSUMED);
}

/// Handles the command HCI read local version informations
__STATIC int hci_rd_local_ver_info_cmd_handler(void const *param, uint16_t opcode)
{
    // allocate the status event message
    struct hci_rd_local_ver_info_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_rd_local_ver_info_cmd_cmp_evt);

    // gets the hci version
    evt->hci_ver    = RWBT_SW_VERSION_MAJOR;
    // gets the hci revision
    evt->hci_rev    = CO_SUBVERSION_BUILD(RWBT_SW_VERSION_MINOR, RWBT_SW_VERSION_BUILD);
    // gets the lmp version
    evt->lmp_ver    = RWBT_SW_VERSION_MAJOR;
    // gets the lmp subversion
    evt->lmp_subver = CO_SUBVERSION_BUILD(RWBT_SW_VERSION_MINOR, RWBT_SW_VERSION_BUILD);
    // gets the manufacturer name
    evt->manuf_name = RW_COMP_ID;
    // sets the status
    evt->status     = CO_ERROR_NO_ERROR;
    // send the message
    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handles the command HCI read bd address
__STATIC int hci_rd_bd_addr_cmd_handler(void const *param, uint16_t opcode)
{
    struct hci_rd_bd_addr_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_rd_bd_addr_cmd_cmp_evt);
    memcpy(&evt->local_addr, &llm_env.local_pub_addr, sizeof(co_default_bdaddr));
    evt->status = CO_ERROR_NO_ERROR;
    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handles the command HCI read local supported commands
__STATIC int hci_rd_local_supp_cmds_cmd_handler(void const *param, uint16_t opcode)
{
    // allocate the status event message
    struct hci_rd_local_supp_cmds_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_rd_local_supp_cmds_cmd_cmp_evt);

    // Copy the first part from ROM stored table
    memcpy(&evt->local_cmds.cmds[0], llm_local_supp_cmds, sizeof(llm_local_supp_cmds));

    // Fill remaining bytes with 0
    memset(&evt->local_cmds.cmds[sizeof(llm_local_supp_cmds)], 0x00, SUPP_CMDS_LEN - sizeof(llm_local_supp_cmds));

    // sets the status
    evt->status = CO_ERROR_NO_ERROR;

    // send the message
    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI read local supported features
__STATIC int hci_rd_local_supp_feats_cmd_handler(void const *param, uint16_t opcode)
{
    // allocate the complete event message
    struct hci_rd_local_supp_feats_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_rd_local_supp_feats_cmd_cmp_evt);

    // get the local features
    memset(&evt->feats.feats[0], 0, FEATS_LEN);

    // Set BR_EDR_Not_Supported and LE_Supported
    evt->feats.feats[4] |= (B4_BR_EDR_NOT_SUPP_MSK | B4_LE_SUPP_MSK);

    // update the status
    evt->status = CO_ERROR_NO_ERROR;

    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handles the command HCI set event mask.
__STATIC int hci_set_evt_mask_cmd_handler(struct hci_set_evt_mask_cmd const *param, uint16_t opcode)
{
    // Set the event mask in the HCI
    uint8_t status = hci_evt_mask_set(&param->event_mask, HCI_PAGE_DFT);

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI read connection accept timeout
__STATIC int hci_rd_con_accept_to_cmd_handler(void const *param, uint16_t opcode)
{
    // allocate the complete event message
    struct hci_rd_con_accept_to_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_rd_con_accept_to_cmd_cmp_evt);
    evt->status = CO_ERROR_NO_ERROR;
    evt->con_acc_to = hci_con_accept_to_get();
    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI write connection accept timeout
__STATIC int hci_wr_con_accept_to_cmd_handler(struct hci_wr_con_accept_to_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;

    if ((param->con_acc_to >= CON_ACCEPT_TO_MIN) && (param->con_acc_to <= CON_ACCEPT_TO_MAX))
    {
        hci_con_accept_to_set(param->con_acc_to);
        status = CO_ERROR_NO_ERROR;
    }

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

/// Handles the command HCI set event mask page 2.
__STATIC int hci_set_evt_mask_page_2_cmd_handler(struct hci_set_evt_mask_cmd const *param, uint16_t opcode)
{
    // Set the event mask in the HCI
    uint8_t status = hci_evt_mask_set(&param->event_mask, HCI_PAGE_2);

    // send the command complete event message
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

/// Handles the command HCI set controller to host flow control.
__STATIC int hci_set_ctrl_to_host_flow_ctrl_cmd_handler(struct hci_set_ctrl_to_host_flow_ctrl_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;

    if (param->flow_cntl <= FLOW_CONTROL_ACL_SCO)
    {
        bool acl_flow_cntl_en  = false;

        switch (param->flow_cntl)
        {
        case FLOW_CONTROL_OFF:
        case FLOW_CONTROL_SCO:
        {
            // disabled by default
        } break;
        case FLOW_CONTROL_ACL:
        case FLOW_CONTROL_ACL_SCO:
        {
            acl_flow_cntl_en = true;
        }
        break;
        default:
            break;
        }

        // Enable HCI Host flow control for ACL data
        status = hci_fc_acl_en(acl_flow_cntl_en);
    }

    // send the command complete event message
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

/// Handles the command HCI host buffer size.
__STATIC int hci_host_buf_size_cmd_handler(struct hci_host_buf_size_cmd const *param, uint16_t opcode)
{
    uint8_t status = hci_fc_acl_buf_size_set(param->acl_pkt_len, param->nb_acl_pkts);

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

/// Handles the command HCI host number of completed packets.
__STATIC int hci_host_nb_cmp_pkts_cmd_handler(struct hci_host_nb_cmp_pkts_cmd const *param, uint16_t opcode)
{
#if (BLE_CENTRAL || BLE_PERIPHERAL)
    uint8_t status = CO_ERROR_NO_ERROR;
    uint8_t idx;
    uint16_t acl_pkt_cnt = 0;

    for (idx = 0; idx < param->nb_of_hdl; idx++)
    {
        // BLE ACL link
        if (param->con[idx].hdl < BLE_ACTIVITY_MAX_CFG)
        {
            acl_pkt_cnt += param->con[idx].nb_comp_pkt;
        }

        // Not a valid connection handle
        else
        {
            ASSERT_ERR_FORCE(0);
            status = CO_ERROR_INVALID_HCI_PARAM;
            break;
        }
    }

    // A command complete event must be sent to host only if there is an error
    if (status != CO_ERROR_NO_ERROR)
    {
        // Send the command complete event
        llm_cmd_cmp_send(opcode, status);
    }
    else
    {
        // update the Flow Control module with counted packets
        hci_fc_host_nb_acl_pkts_complete(acl_pkt_cnt);
    }
#endif // (BLE_CENTRAL || BLE_PERIPHERAL)

    return (KE_MSG_CONSUMED);
}
#endif // (HCI_TL_SUPPORT)

/// Handles the command HCI Set Ecosystem Base Interval.
__STATIC int hci_set_eco_base_intv_cmd_handler(struct hci_set_eco_base_intv_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_NO_ERROR;

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

/// Handles the command HCI Configure Data Path.
__STATIC int hci_config_data_path_cmd_handler(struct hci_config_data_path_cmd const *param, uint16_t opcode)
{
    uint8_t status;

    // Configure the data path
#if (BLE_ISO_PRESENT)
    status = data_path_config(param->data_path_id, param->vendor_specific_cfg_len, &param->vendor_specific_cfg[0]);
#else //!(BLE_ISO_PRESENT)
    status = CO_ERROR_INVALID_HCI_PARAM;
#endif //!(BLE_ISO_PRESENT)

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

/// Handles the command HCI Read Local Supported Codecs v2.
__STATIC int hci_rd_local_supp_codecs_v2_cmd_handler(void const *param, uint16_t opcode)
{
    // allocate the complete event message
    struct hci_rd_local_supp_codecs_v2_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_rd_local_supp_codecs_v2_cmd_cmp_evt);

    evt->status = CO_ERROR_NO_ERROR;
    evt->num_supp_standard_codecs = 0;
    evt->num_supp_vendor_specific_codecs = 0;

    // send the message
    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handles the command HCI Read Local Supported Codec Capabilities.
__STATIC int hci_rd_local_supp_codec_cap_cmd_handler(struct hci_rd_local_supp_codec_cap_cmd const *param, uint16_t opcode)
{
    // allocate the complete event message
    struct hci_rd_local_supp_codec_cap_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_rd_local_supp_codec_cap_cmd_cmp_evt);

    evt->status = CO_ERROR_NO_ERROR;
    evt->num_codec_cap = 1;
    evt->codec_cap_len = 4;
    memset(&evt->codec_cap[0], 0, evt->codec_cap_len);

    // send the message
    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handles the command HCI Read Local Supported Controller Delay.
__STATIC int hci_rd_local_supp_ctrl_delay_cmd_handler(struct hci_rd_local_supp_ctrl_delay_cmd const *param, uint16_t opcode)
{
    // allocate the complete event message
    struct hci_rd_local_supp_ctrl_delay_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_rd_local_supp_ctrl_delay_cmd_cmp_evt);

    evt->status = CO_ERROR_NO_ERROR;
    memset(&evt->min_ctrl_delay[0], 0, 3);
    memset(&evt->max_ctrl_delay[0], 0, 3);

    // send the message
    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handles the command HCI read AFH channel assessment mode
__STATIC int hci_rd_afh_ch_assess_mode_cmd_handler(void const *param, uint16_t opcode)
{
    // allocate the complete event message
    struct hci_rd_afh_ch_assess_mode_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_rd_afh_ch_assess_mode_cmd_cmp_evt);

    evt->status = CO_ERROR_NO_ERROR;
    evt->afh_ch_ass_mode = rwip_ch_ass_en_get();

    hci_send_2_host(evt);
    return (KE_MSG_CONSUMED);
}

/// Handles the command HCI write AFH channel assessment mode
__STATIC int hci_wr_afh_ch_assess_mode_cmd_handler(struct hci_wr_afh_ch_assess_mode_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;

    if (param->afh_ch_ass_mode <= AFH_CH_ASS_ENABLED)
    {
        /* If the AFH_Channel_Assessment_Mode parameter is enabled and the Controller does not support
         * a channel assessment scheme, other than via the HCI_LE_Set_Host_Channel_Classification command,
         * then a Status parameter of Channel Assessment Not Supported should be returned */
        if ((param->afh_ch_ass_mode == AFH_CH_ASS_DISABLED) || rwip_ch_ass_en_get())
        {
            rwip_ch_ass_en_set(param->afh_ch_ass_mode);
            status = CO_ERROR_NO_ERROR;
        }
        else
        {
            status = CO_ERROR_CHANNEL_CLASS_NOT_SUP;
        }
    }

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

//#endif // !BT_DUAL_MODE

#if (HCI_TL_SUPPORT)
__STATIC int hci_le_set_evt_mask_cmd_handler(struct hci_le_set_evt_mask_cmd const *param, uint16_t opcode)
{
    // Set the event mask in the environment variable (for masking impact to the controller behavior)
    memcpy(&llm_env.le_event_mask.mask[0], &param->le_mask.mask[0], EVT_MASK_LEN);

    // Set the event mask to HCI (for basic masking)
    hci_evt_mask_set(&llm_env.le_event_mask, HCI_PAGE_LE);

    // Send the command complete event
    llm_cmd_cmp_send(opcode, CO_ERROR_NO_ERROR);

    return (KE_MSG_CONSUMED);
}

#if (BLE_PERIPHERAL || BLE_CENTRAL)
__STATIC int hci_le_rd_buf_size_cmd_handler(void const *param, uint16_t opcode)
{
    struct hci_le_rd_buf_size_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, HCI_LE_RD_BUF_SIZE_CMD_OPCODE, hci_le_rd_buf_size_cmd_cmp_evt);
    event->hc_tot_nb_data_pkts =  EM_NUM_BLE_ACLTXBUF;//BLE_ACL_BUF_NB_TX;
    event->hc_data_pk_len       = BLE_MAX_OCTETS;
    event->status               = CO_ERROR_NO_ERROR;
    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}
#endif // (BLE_PERIPHERAL || BLE_CENTRAL)

__STATIC int hci_le_rd_local_supp_feats_cmd_handler(void const *param, uint16_t opcode)
{
    struct hci_le_rd_local_supp_feats_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, HCI_LE_RD_LOCAL_SUPP_FEATS_CMD_OPCODE, hci_le_rd_local_supp_feats_cmd_cmp_evt);
    llm_le_features_get(&event->feats);
    event->status = CO_ERROR_NO_ERROR;
    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}
#endif // (HCI_TL_SUPPORT)

__STATIC int hci_le_set_rand_addr_cmd_handler(struct hci_le_set_rand_addr_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

#if (!BLE_HOST_PRESENT)
    // Check if the random address set by the LE Set Random Address command could be in use
    if (!llm_is_rand_addr_in_use_check())
#elif (HCI_TL_SUPPORT)
    // Check if the random address set by the LE Set Random Address command could be in use (work-around: this check is not performed if the Host is local)
    if (!hci_is_ext_host() || !llm_is_rand_addr_in_use_check())
#endif // (!BLE_HOST_PRESENT)
    {
        if (co_bdaddr_compare((struct bd_addr *)&param->rand_addr, &co_null_bdaddr) != true)
        {
            // save the private address
            memcpy(&llm_env.local_rand_addr.addr[0], &param->rand_addr.addr[0], BD_ADDR_LEN);

#if (BLE_HOST_PRESENT)
#if (BLE_OBSERVER)
            // Update Scan random address to use
            lld_scan_rand_addr_update(&(param->rand_addr));
#endif // (BLE_OBSERVER)

#if (BLE_CENTRAL)
            // Update Init random address to use
            lld_init_rand_addr_update(&(param->rand_addr));
#endif // (BLE_CENTRAL)
#endif // (BLE_HOST_PRESENT)

            status = CO_ERROR_NO_ERROR;
        }
        else
        {
            status = CO_ERROR_INVALID_HCI_PARAM;
        }
    }

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

#if (BLE_CENTRAL || BLE_BROADCASTER)
__STATIC int hci_le_set_host_ch_class_cmd_handler(struct hci_le_set_host_ch_class_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    if (!llm_env.ch_map_info.new_host_class)
    {
        uint8_t nbchgood = 0;

        // Make sure that the three most significant bits are 0
        if ((param->ch_map.map[LE_CH_MAP_LEN - 1] & 0xE0) == 0)
        {
            // Count number of good channels
            nbchgood = ble_util_nb_good_channels(&param->ch_map);
        }

        // Check if there are enough channels in the map
        if ((nbchgood != 0) && (nbchgood <= BLE_DATA_CHANNEL_NB))
        {
            // Save Host channel classification
            memcpy(&llm_env.ch_map_info.host_ch_class.map[0], &param->ch_map.map[0], LE_CH_MAP_LEN);

            // As per standard, the interval between two successive commands sent shall be at least one second (1 sec --> 100*10ms)
            ke_timer_set(LLM_NEW_HOST_CLASS_TO, TASK_LLM, 1000);
            llm_env.ch_map_info.new_host_class = true;

            // Initiate channel map update immediately
            ke_timer_clear(LLM_CH_MAP_TO, TASK_LLM);
            llm_ch_map_update();

            status = CO_ERROR_NO_ERROR;
        }
        else
        {
            status = CO_ERROR_INVALID_HCI_PARAM;
        }
    }

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}
#endif //(BLE_CENTRAL || BLE_BROADCASTER)

__STATIC int hci_le_rd_wl_size_cmd_handler(void const *param, uint16_t opcode)
{
    struct hci_le_rd_wlst_size_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_le_rd_wlst_size_cmd_cmp_evt);
    event->status       = CO_ERROR_NO_ERROR;
    event->wlst_size    = BLE_WHITELIST_MAX;
    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}

__STATIC int hci_le_clear_wlst_cmd_handler(void const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    // Check if white list is accessible
    if (llm_is_wl_accessible())
    {
        uint8_t position;

        // Parse the list linearly
        for (position = 0; position < BLE_WHITELIST_MAX ; position++)
        {
            // Check if list entry is used
            if (!GETB(llm_env.dev_list[position].status, LLM_DEV_LIST_ENTRY_USED))
                continue;

            // Indicate the device as no more present in white list
            SETB(llm_env.dev_list[position].status, LLM_DEV_IN_WL, 0);

            // Check if device is in periodic advertiser list
            if (!GETB(llm_env.dev_list[position].status, LLM_DEV_IN_PL))
            {
                // Remove the device list entry
                SETB(llm_env.dev_list[position].status, LLM_DEV_LIST_ENTRY_USED, 0);
            }

            // Remove the white list entry from driver
            lld_white_list_rem(position, &llm_env.dev_list[position].addr, llm_env.dev_list[position].addr_type);
        }

        status = CO_ERROR_NO_ERROR;
    }

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

__STATIC int hci_le_add_dev_to_wlst_cmd_handler(struct hci_le_add_dev_to_wlst_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    do
    {
        uint8_t position;

        // Check if white list is accessible
        if (!llm_is_wl_accessible())
            break;

        // Check if addr type is valid
        if ((param->dev_addr_type > ADDR_RAND) && (param->dev_addr_type != ANONYMOUS_ADV_ADDR_TYPE))
        {
            status = CO_ERROR_INVALID_HCI_PARAM;
            break;
        }

        // Check if the device is already in the list
        position = llm_dev_list_search(&param->dev_addr, param->dev_addr_type);

        if (position < BLE_WHITELIST_MAX)
        {
            // Check if it is in white list
            if (GETB(llm_env.dev_list[position].status, LLM_DEV_IN_WL))
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
        memcpy(&llm_env.dev_list[position].addr, &param->dev_addr, BD_ADDR_LEN);
        llm_env.dev_list[position].addr_type = param->dev_addr_type;
        SETB(llm_env.dev_list[position].status, LLM_DEV_LIST_ENTRY_USED, 1);
        SETB(llm_env.dev_list[position].status, LLM_DEV_IN_WL, 1);

        // Write the EM list entry (if not connected)
#if (BLE_CENTRAL || BLE_PERIPHERAL || BLE_OBSERVER)
        if (!llm_is_dev_connected((struct bd_addr *) &param->dev_addr, param->dev_addr_type))
#endif //(BLE_CENTRAL || BLE_PERIPHERAL || BLE_OBSERVER)
        {
            lld_white_list_add(position, (struct bd_addr *) &param->dev_addr, param->dev_addr_type);
        }

        status = CO_ERROR_NO_ERROR;

    }
    while (0);

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

__STATIC int hci_le_rmv_dev_from_wlst_cmd_handler(struct hci_le_rmv_dev_from_wlst_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    do
    {
        uint8_t position;

        // Check if white list is accessible
        if (!llm_is_wl_accessible())
            break;

        // Check if addr type is valid
        if ((param->dev_addr_type > ADDR_RAND) && (param->dev_addr_type != ANONYMOUS_ADV_ADDR_TYPE))
        {
            status = CO_ERROR_INVALID_HCI_PARAM;
            break;
        }

        // Find the device in the list
        position = llm_dev_list_search(&param->dev_addr, param->dev_addr_type);

        if ((position >= BLE_WHITELIST_MAX) || !GETB(llm_env.dev_list[position].status, LLM_DEV_IN_WL))
        {
            status = CO_ERROR_INVALID_HCI_PARAM;
            break;
        }

        // Indicate the device as no more present in white list
        SETB(llm_env.dev_list[position].status, LLM_DEV_IN_WL, 0);

        // Check if device is in periodic advertiser list
        if (!GETB(llm_env.dev_list[position].status, LLM_DEV_IN_PL))
        {
            // Remove the device list entry
            SETB(llm_env.dev_list[position].status, LLM_DEV_LIST_ENTRY_USED, 0);
        }

        // Remove the white list entry from driver
        lld_white_list_rem(position, (struct bd_addr *) &param->dev_addr, param->dev_addr_type);

        status = CO_ERROR_NO_ERROR;

    }
    while (0);

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

__STATIC int hci_le_rd_rslv_list_size_cmd_handler(void const *param, uint16_t opcode)
{
    struct hci_le_rd_rslv_list_size_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_le_rd_rslv_list_size_cmd_cmp_evt);
    event->status   = CO_ERROR_NO_ERROR;
    event->size     = BLE_RESOL_ADDR_LIST_MAX;
    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}

__STATIC int hci_le_clear_rslv_list_cmd_handler(void const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    // Check if resolving address list is accessible
    if (!llm_env.addr_resolution_en || !llm_is_non_con_act_ongoing_check())
    {
        for (uint8_t position = 0 ; position < BLE_RAL_MAX ; position++)
        {
            // Invalidate entry
            llm_env.ral[position].addr_type = 0xFF;
        }

        // Clear HW resolving address list
        lld_res_list_clear();

        status = CO_ERROR_NO_ERROR;
    }

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

__STATIC int hci_le_add_dev_to_rslv_list_cmd_handler(struct hci_le_add_dev_to_rslv_list_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;

    do
    {
        uint8_t position;
        bool in_wl = false;

        // Check if addr type is valid
        if ((param->peer_id_addr_type > ADDR_RAND) || ((param->peer_id_addr_type == ADDR_RAND) &&
                ((param->peer_id_addr.addr[BD_ADDR_LEN - 1] & 0xC0) != RND_STATIC_ADDR)))
            break;

        // Check if resolving address list is accessible
        if (llm_env.addr_resolution_en && llm_is_non_con_act_ongoing_check())
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        // Check if device is in device list
        position = llm_dev_list_search(&param->peer_id_addr, param->peer_id_addr_type);

        // Check if device is in white list
        if (position < BLE_WHITELIST_MAX)
        {
            in_wl = GETB(llm_env.dev_list[position].status, LLM_DEV_IN_WL);
        }

        // Check if the device is already in resolving list
        position = llm_ral_search(&param->peer_id_addr, param->peer_id_addr_type);

        if (position < BLE_RAL_MAX)
        {
            status = CO_ERROR_INVALID_HCI_PARAM;
            break;
        }

        // If Peer_IRK valid
        if (memcmp(&param->peer_irk.key[0], co_null_key, KEY_LEN) != 0)
        {
            // Check if the Peer_IRK is already in resolving list
            for (position = 0 ; position < BLE_RAL_MAX ; position++)
            {
                // If entry is valid and matching Peer_IRK
                if ((llm_env.ral[position].addr_type != 0xFF) && (memcmp(&param->peer_irk.key[0], &llm_env.ral[position].peer_irk.key[0], KEY_LEN) == 0))
                    break;
            }

            if (position < BLE_RAL_MAX)
            {
                status = CO_ERROR_INVALID_HCI_PARAM;
                break;
            }
        }

        // Find an empty slot to write the new address
        for (position = 0 ; position < BLE_RAL_MAX ; position++)
        {
            if (llm_env.ral[position].addr_type == 0xFF)
                break;
        }

        if (position == BLE_RAL_MAX)
        {
            status = CO_ERROR_MEMORY_CAPA_EXCEED;
            break;
        }

        // Add device to HW resolving address list
        lld_res_list_add(position, &param->peer_id_addr, &param->peer_irk, &param->local_irk, param->peer_id_addr_type, in_wl);

        // Store entry in RAM table
        llm_env.ral[position].addr_type = param->peer_id_addr_type;
        llm_env.ral[position].bd_addr      = param->peer_id_addr;
        llm_env.ral[position].local_irk = param->local_irk;
        llm_env.ral[position].peer_irk  = param->peer_irk;

        status = CO_ERROR_NO_ERROR;

    }
    while (0);

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

__STATIC int hci_le_rmv_dev_from_rslv_list_cmd_handler(struct hci_le_rmv_dev_from_rslv_list_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    // Check if addr type is valid
    if (param->peer_id_addr_type > ADDR_RAND)
    {
        status = CO_ERROR_INVALID_HCI_PARAM;
    }
    // Check if resolving address list is accessible
    else if (!llm_env.addr_resolution_en || !llm_is_non_con_act_ongoing_check())
    {
        // Find device in resolving list
        uint8_t position = llm_ral_search((struct bd_addr *) &param->peer_id_addr, param->peer_id_addr_type);

        if (position < BLE_RAL_MAX)
        {
            // Remove device from HW resolving address list
            lld_res_list_rem(position);

            // Invalidate entry
            llm_env.ral[position].addr_type = 0xFF;

            status = CO_ERROR_NO_ERROR;
        }
        else
        {
            status = CO_ERROR_UNKNOWN_CONNECTION_ID;
        }
    }

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

__STATIC int hci_le_set_rslv_priv_addr_to_cmd_handler(struct hci_le_set_rslv_priv_addr_to_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;

    // Check parameter range
    if ((param->rpa_timeout >= RPA_TO_MIN) && (param->rpa_timeout <= RPA_TO_MAX))
    {
        // Set the resolvable private addresses timeout value
        llm_env.rpa_renew_to = param->rpa_timeout;

        // Restart the timer that triggers renewal of resolvable private address
        co_timer_periodic_set(&(llm_env.rpa_renew_timer), CO_TIME_SEC_TO_MS(llm_env.rpa_renew_to));
        status = CO_ERROR_NO_ERROR;
    }

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

__STATIC int hci_le_set_addr_resol_en_cmd_handler(struct hci_le_set_addr_resol_en_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    // Check if resolving address list is accessible
    if (!llm_is_non_con_act_ongoing_check())
    {
        if (param->enable <= 1)
        {
            // Set the address resolution enable flag
            llm_env.addr_resolution_en = param->enable;
            status = CO_ERROR_NO_ERROR;
        }
        else
        {
            status = CO_ERROR_INVALID_HCI_PARAM;
        }
    }

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

__STATIC int hci_le_rd_peer_rslv_addr_cmd_handler(struct hci_le_rd_peer_rslv_addr_cmd const *param, uint16_t opcode)
{
    struct hci_le_rd_peer_rslv_addr_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_le_rd_peer_rslv_addr_cmd_cmp_evt);
    memset(&evt->peer_rslv_addr, 0, BD_ADDR_LEN);
    evt->status = CO_ERROR_INVALID_HCI_PARAM;

    if (param->peer_id_addr_type <= ADDR_RAND)
    {
        // Find device in resolving list
        uint8_t position = llm_ral_search(&param->peer_id_addr, param->peer_id_addr_type);

        if (position < BLE_RAL_MAX)
        {
            // Search in HW resolving list
            evt->status = lld_res_list_peer_rpa_get(position, &evt->peer_rslv_addr);
        }
        else
        {
            evt->status = CO_ERROR_UNKNOWN_CONNECTION_ID;
        }
    }

    if (CO_ERROR_NO_ERROR != evt->status)
    {
        memset(&evt->peer_rslv_addr.addr[0], 0, BD_ADDR_LEN);
    }

    // send the event
    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

__STATIC int hci_le_rd_loc_rslv_addr_cmd_handler(struct hci_le_rd_loc_rslv_addr_cmd const *param, uint16_t opcode)
{
    struct hci_le_rd_loc_rslv_addr_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_le_rd_loc_rslv_addr_cmd_cmp_evt);
    evt->status = CO_ERROR_INVALID_HCI_PARAM;

    if (param->peer_id_addr_type <= ADDR_RAND)
    {
        // Find device in resolving list
        uint8_t position = llm_ral_search(&param->peer_id_addr, param->peer_id_addr_type);

        if (position < BLE_RAL_MAX)
        {
            // Search in HW resolving list
            evt->status = lld_res_list_local_rpa_get(position, &evt->loc_rslv_addr);
        }
        else
        {
            evt->status = CO_ERROR_UNKNOWN_CONNECTION_ID;
        }
    }

    if (CO_ERROR_NO_ERROR != evt->status)
    {
        memset(&evt->loc_rslv_addr.addr[0], 0, BD_ADDR_LEN);
    }

    // send the event
    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Call back definition of the function that can handle result of an AES based algorithm
 *
 * @param[in] status       Execution status
 * @param[in] aes_res      16 bytes block result
 * @param[in] metainfo     Metadata information provided by API user (see \glos{METAINFO})
 ****************************************************************************************
 */
__STATIC void llm_aes_res_cb(uint8_t status, const uint8_t *aes_res, uint32_t dummy)
{
    // send message with result data
    struct llm_encrypt_ind *msg = KE_MSG_ALLOC(LLM_ENCRYPT_IND, TASK_LLM, TASK_LLM, llm_encrypt_ind);
    msg->status = status;
    memcpy(msg->result, aes_res, KEY_LEN);
    ke_msg_send(msg);
}

__STATIC int hci_le_enc_cmd_handler(struct hci_le_enc_cmd const *param, uint16_t opcode)
{
    // perform AES encryption
    aes_encrypt((uint8_t *) & (param->key), (uint8_t *) & (param->plain_data[0]), true, llm_aes_res_cb, 0);

    return (KE_MSG_CONSUMED);
}


#if (!HOST_PRESENT)
/// AES-128 Decrypt result
__STATIC void llm_aes_decrypt_res_cb(uint8_t status, const uint8_t *aes_res, uint32_t dummy)
{
    // Report the encrypted data to Host via HCI
    struct hci_vs_le_decrypt_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, HCI_VS_LE_DECRYPT_CMD_OPCODE, hci_vs_le_decrypt_cmd_cmp_evt);
    memcpy(&event->plain_data[0], aes_res, ENC_DATA_LEN);
    event->status = CO_ERROR_NO_ERROR;
    hci_send_2_host(event);
}

__STATIC int hci_vs_le_decrypt_cmd_handler(struct hci_vs_le_decrypt_cmd const *param, uint16_t opcode)
{
    // perform AES encryption
    aes_decrypt((uint8_t *) & (param->key), (uint8_t *) & (param->encrypt_data[0]), true, llm_aes_decrypt_res_cb, 0);

    return (KE_MSG_CONSUMED);
}
#endif // (!HOST_PRESENT)


__STATIC int hci_le_rand_cmd_handler(void const *param, uint16_t opcode)
{
    struct hci_le_rand_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, HCI_LE_RAND_CMD_OPCODE, hci_le_rand_cmd_cmp_evt);
    co_write32p(&event->nb.nb[0], co_rand_word());
    co_write32p(&event->nb.nb[4], co_rand_word());
    event->status = CO_ERROR_NO_ERROR;
    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}

__STATIC int hci_le_rd_supp_states_cmd_handler(void const *param, uint16_t opcode)
{
    struct hci_le_rd_supp_states_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, HCI_LE_RD_SUPP_STATES_CMD_OPCODE, hci_le_rd_supp_states_cmd_cmp_evt);
    memcpy(&event->states.supp_states[0], &llm_local_le_states.supp_states[0], LE_STATES_LEN);
    event->status = CO_ERROR_NO_ERROR;
    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}

__STATIC int hci_le_rd_tx_pwr_cmd_handler(void const *param, uint16_t opcode)
{
    struct hci_le_rd_tx_pwr_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, HCI_LE_RD_TX_PWR_CMD_OPCODE, hci_le_rd_tx_pwr_cmd_cmp_evt);

    event->status = CO_ERROR_NO_ERROR;
    event->min_tx_pwr = lld_tx_power_dbm_get(rwip_rf.txpwr_min, rwip_rf.txpwr_min_mod);
    event->max_tx_pwr = lld_tx_power_dbm_get(rwip_rf.txpwr_max, rwip_rf.txpwr_max_mod);
    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}

__STATIC int hci_le_rd_rf_path_comp_cmd_handler(void const *param, uint16_t opcode)
{
    struct hci_le_rd_rf_path_comp_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, HCI_LE_RD_RF_PATH_COMP_CMD_OPCODE, hci_le_rd_rf_path_comp_cmd_cmp_evt);

    // Get TX/RX path compensation delays
    event->tx_path_comp = lld_tx_path_comp_get();
    event->rx_path_comp = lld_rx_path_comp_get();
    event->status = CO_ERROR_NO_ERROR;
    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}

__STATIC int hci_le_wr_rf_path_comp_cmd_handler(struct hci_le_wr_rf_path_comp_cmd const *param, uint16_t opcode)
{
    struct hci_basic_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, HCI_LE_WR_RF_PATH_COMP_CMD_OPCODE, hci_basic_cmd_cmp_evt);

    if ((param->rx_path_comp >= (-128 * 10)) && (param->rx_path_comp <= (128 * 10))
            && (param->tx_path_comp >= (-128 * 10)) && (param->tx_path_comp <= (128 * 10)))
    {
        // Set TX/RX path compensation delays
        lld_path_comp_set(param->tx_path_comp, param->rx_path_comp);
        event->status = CO_ERROR_NO_ERROR;
    }
    else
    {
        event->status = CO_ERROR_INVALID_HCI_PARAM;
    }
    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}

__STATIC int hci_le_set_priv_mode_cmd_handler(struct hci_le_set_priv_mode_cmd const *param, uint16_t opcode)
{
    // Status returned in the command complete event
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    // Check if resolving address list is accessible
    if (!llm_env.addr_resolution_en || !llm_is_non_con_act_ongoing_check())
    {
        if ((param->priv_mode <= PRIV_TYPE_DEVICE) && (param->peer_addr_type <= ADDR_RAND))
        {
            // Find device in resolving list
            uint8_t position = llm_ral_search((struct bd_addr *) &param->peer_addr, param->peer_addr_type);

            if (position < BLE_RAL_MAX)
            {
                // Allow the Host to specify the privacy mode to be used for a given entry on the resolving list
                lld_res_list_priv_mode_update(position, param->priv_mode);
                status = CO_ERROR_NO_ERROR;
            }
            else
            {
                status = CO_ERROR_UNKNOWN_CONNECTION_ID;
            }
        }
        else
        {
            status = CO_ERROR_INVALID_HCI_PARAM;
        }
    }

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

__STATIC int hci_le_mod_sleep_clk_acc_cmd_handler(struct hci_le_mod_sleep_clk_acc_cmd const *param, uint16_t opcode)
{
    // Status returned in the command complete event
    uint8_t status = CO_ERROR_LIMIT_REACHED;
    // Indicates if the active clock is in use
    bool active_clk = llm_env.act_clk_acc;

    if ((param->action == SWITCH_TO_MORE_ACC_CLK) ^ active_clk)
    {
#if (BLE_PERIPHERAL || BLE_CENTRAL)
        // Inform connections
        for (uint8_t act_id = 0; act_id < BLE_ACTIVITY_MAX_CFG; act_id++)
        {
            if (llm_env.act_info[act_id].state == LLM_CONNECTED)
            {
                llc_clk_acc_modify(act_id, param->action);
            }
        }
#endif // (BLE_PERIPHERAL || BLE_CENTRAL)

        status = CO_ERROR_NO_ERROR;
    }

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

#if (BLE_AOD | BLE_AOA)
__STATIC int hci_le_rd_antenna_inf_cmd_handler(void const *param, uint16_t opcode)
{
    // Report the command complete event
    struct hci_le_rd_antenna_inf_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, HCI_LE_RD_ANTENNA_INF_CMD_OPCODE, hci_le_rd_antenna_inf_cmd_cmp_evt);

    event->supp_switching_sampl_rates = AOD_TX_1_US | AOD_RX_1_US | AOA_RX_1_US;
    event->antennae_num               = BLE_ANTENNA_NB;
    event->max_switching_pattern_len  = BLE_MAX_SW_PAT_LEN;
    event->max_cte_len                = CTE_LEN_MAX;
    event->status                     = CO_ERROR_NO_ERROR;

    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}
#endif // (BLE_AOD | BLE_AOA)


#if (BLE_PERIPHERAL || BLE_CENTRAL)
__STATIC int hci_le_rd_suggted_dft_data_len_cmd_handler(void const *param, uint16_t opcode)
{
    // Report the command complete event
    struct hci_le_rd_suggted_dft_data_len_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, HCI_LE_RD_SUGGTED_DFT_DATA_LEN_CMD_OPCODE, hci_le_rd_suggted_dft_data_len_cmd_cmp_evt);

    event->suggted_max_tx_octets    = llm_env.suggested_max_tx_octets;
    event->suggted_max_tx_time      = llm_env.suggested_max_tx_time;
    event->status                   = CO_ERROR_NO_ERROR;

    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}

__STATIC int hci_le_wr_suggted_dft_data_len_cmd_handler(struct hci_le_wr_suggted_dft_data_len_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;

    // Check parameters
    if ((param->suggted_max_tx_octets >= LE_MIN_OCTETS) && (param->suggted_max_tx_octets <= LE_MAX_OCTETS)
            && (param->suggted_max_tx_time >= LE_MIN_TIME) && (param->suggted_max_tx_time <= LE_MAX_TIME_CODED))
    {
        // Store the parameters
        llm_env.suggested_max_tx_octets = param->suggted_max_tx_octets;
        llm_env.suggested_max_tx_time   = param->suggted_max_tx_time;
        status                               = CO_ERROR_NO_ERROR;
    }

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

#if (EAVESDROPPING_SUPPORT)
__STATIC int hci_vs_le_enable_enh_set_data_len_cmd_handler(struct hci_vs_le_enable_enh_set_data_len_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    // Allow to enable/disable the enhanced version only if there is no active link
    if (llm_nb_active_link_get() == 0)
    {
        // Store the received value
        llm_env.ehn_set_data_length_enabled = param->enable;
        // Update the status
        status = CO_ERROR_NO_ERROR;
    }

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

__STATIC int hci_vs_le_enh_wr_suggted_dft_data_len_cmd_handler(struct hci_vs_le_enh_wr_suggted_dft_data_len_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_NO_ERROR;

    // Try to set the parameters
    if ((param->suggted_max_tx_octets < BLE_MIN_OCTETS) || (param->suggted_max_tx_octets > BLE_MAX_OCTETS)
            || (param->suggted_max_tx_time < BLE_MIN_TIME)      || (param->suggted_max_tx_time > BLE_MAX_TIME)
            || (param->suggted_max_rx_octets < BLE_MIN_OCTETS) || (param->suggted_max_rx_octets > BLE_MAX_OCTETS)
            || (param->suggted_max_rx_time < BLE_MIN_TIME)      || (param->suggted_max_rx_time > BLE_MAX_TIME))
    {
        status = CO_ERROR_INVALID_HCI_PARAM;
    }
    else
    {
        llm_env.suggested_max_tx_octets = param->suggted_max_tx_octets;
        llm_env.suggested_max_tx_time = param->suggted_max_tx_time;
        llm_env.suggested_max_rx_octets = param->suggted_max_rx_octets;
        llm_env.suggested_max_rx_time = param->suggted_max_rx_time;
    }

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}
#endif //(EAVESDROPPING_SUPPORT)

__STATIC int hci_le_rd_max_data_len_cmd_handler(void const *param, uint16_t opcode)
{
    // Report the command complete event
    struct hci_le_rd_max_data_len_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_le_rd_max_data_len_cmd_cmp_evt);

    event->suppted_max_rx_octets    = BLE_MAX_OCTETS;
    event->suppted_max_rx_time      = BLE_MAX_TIME;
    event->suppted_max_tx_octets    = BLE_MAX_OCTETS;
    event->suppted_max_tx_time      = BLE_MAX_TIME;
    event->status                   = CO_ERROR_NO_ERROR;

    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}
#endif //(BLE_PERIPHERAL || BLE_CENTRAL)

/// Callback executed when Elliptic Curve algorithm completes
__STATIC void llm_pub_key_gen_result_handler(uint32_t task_id, const ecc_p256_result_t *p_res)
{
    // Report the public key to Host
    struct hci_le_rd_loc_p256_pub_key_cmp_evt *event = KE_MSG_ALLOC(HCI_LE_EVENT, 0, HCI_LE_META_EVT_CODE, hci_le_rd_loc_p256_pub_key_cmp_evt);
    event->subcode = HCI_LE_RD_LOC_P256_PUB_KEY_CMP_EVT_SUBCODE;
    event->status  = CO_ERROR_NO_ERROR;
    memcpy(&(event->public_key.x[0]), p_res->key_res_x, 32);
    memcpy(&(event->public_key.y[0]), p_res->key_res_y, 32);
    hci_send_2_host(event);

    if (rwip_param.set(PARAM_ID_LE_PRIVATE_KEY_P256, PARAM_LEN_PRIVATE_KEY_P256, &llm_env.secret_key256[0]) != PARAM_OK)
    {
        // Not Able to store private key
    }
}

__STATIC int hci_le_rd_local_p256_public_key_cmd_handler(void const *param, uint16_t opcode)
{
    // Send the command status event
    llm_cmd_stat_send(opcode, CO_ERROR_NO_ERROR);

    // Need to get a 32 Byte random number - which we will use as the secret key.
    // We then ECC multiply this by the Base Points, to get a new Public Key.
    bool forced_key;
    uint8_t len = PARAM_LEN_DBG_FIXED_P256_KEY;

    uint8_t *secret_key256 = &llm_env.secret_key256[0];

    // Check if private key is forced by host
    if (rwip_param.get(PARAM_ID_LE_DBG_FIXED_P256_KEY, &len, (uint8_t *)&forced_key) != PARAM_OK)
    {
        forced_key = false;
    }

    // Load the forced private key
    len = PARAM_LEN_PRIVATE_KEY_P256;
    if (forced_key && rwip_param.get(PARAM_ID_LE_PRIVATE_KEY_P256, &len, secret_key256) != PARAM_OK)
    {
        forced_key = false;
    }

    if (!forced_key)
    {
        // Generate a new secret key
        ecc_p256_gen_new_secret_key(secret_key256);
    }

    // Generate the associated public key
    ecc_p256_gen_new_public_key(secret_key256, TASK_LLM, llm_pub_key_gen_result_handler);

    return (KE_MSG_CONSUMED);
}

/// Callback executed when Elliptic Curve algorithm completes
__STATIC void llm_dh_key_gen_result_handler(uint32_t task_id, const ecc_p256_result_t *p_res)
{
    // Report the DH key to Host
    struct hci_le_gen_dhkey_cmp_evt *event = KE_MSG_ALLOC(HCI_LE_EVENT, 0, HCI_LE_META_EVT_CODE, hci_le_gen_dhkey_cmp_evt);
    event->subcode = HCI_LE_GEN_DHKEY_CMP_EVT_SUBCODE;
    event->status  = CO_ERROR_NO_ERROR;
    memcpy(&(event->dh_key[0]), p_res->key_res_x, 32);
    hci_send_2_host(event);
}

__STATIC void hci_le_gen_dhkey_cmd_handler(uint16_t opcode, const uint8_t *p_public_key, uint8_t key_type)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;
    const uint8_t *p_secret_key = NULL;

    if (key_type == USE_GEN_PRIV_KEY)
    {
        // Check if private key exist
        for (int8_t i = PRIV_KEY_256_LEN - 1 ; i > 0 ; i--)
        {
            if (llm_env.secret_key256[i] != 0)
            {
                p_secret_key = &llm_env.secret_key256[0];
                break;
            }
        }
    }
    else
    {
        p_secret_key = ecc_p256_get_debug_priv_key();
    }

    if (p_secret_key != NULL)
    {
        // Generate DH key
        status = ecc_p256_gen_dh_key(p_secret_key, &(p_public_key[0]),  &(p_public_key[32]),
                                     TASK_LLM, llm_dh_key_gen_result_handler);
    }

    // Send the command status event
    llm_cmd_stat_send(opcode, status);
}

__STATIC int hci_le_gen_dhkey_v2_cmd_handler(struct hci_le_gen_dhkey_v2_cmd *param, uint16_t opcode)
{
    hci_le_gen_dhkey_cmd_handler(opcode, param->public_key, param->key_type);
    return (KE_MSG_CONSUMED);
}

__STATIC int hci_le_gen_dhkey_v1_cmd_handler(struct hci_le_gen_dhkey_v1_cmd *param, uint16_t opcode)
{
    hci_le_gen_dhkey_cmd_handler(opcode, param->public_key, USE_GEN_PRIV_KEY);
    return (KE_MSG_CONSUMED);
}

#if (BLE_PERIPHERAL || BLE_CENTRAL)
__STATIC int hci_le_set_dft_phy_cmd_handler(struct hci_le_set_dft_phy_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_NO_ERROR;
    uint8_t supp_phy_msk = PHY_1MBPS_BIT;
    SETB(supp_phy_msk, PHY_2MBPS, BLE_PHY_2MBPS_SUPPORT);
    SETB(supp_phy_msk, PHY_CODED, BLE_PHY_CODED_SUPPORT);

    // Phy preference and value in the range
    if ((!(param->all_phys & ALL_PHYS_RX_NO_PREF) && (param->rx_phys == 0))
            || (!(param->all_phys & ALL_PHYS_TX_NO_PREF) && (param->tx_phys == 0)))
    {
        status = CO_ERROR_INVALID_HCI_PARAM;
    }
    else
    {
        // Phy preference and value supported
        if ((!(param->all_phys & ALL_PHYS_RX_NO_PREF) && ((param->rx_phys & supp_phy_msk) != param->rx_phys))
                || (!(param->all_phys & ALL_PHYS_TX_NO_PREF) && ((param->tx_phys & supp_phy_msk) != param->tx_phys)))
        {
            status = CO_ERROR_UNSUPPORTED;
        }
        else
        {
            // Save preferred PHY parameters
            llm_env.rx_phys = (param->all_phys & ALL_PHYS_RX_NO_PREF) ? (supp_phy_msk) : (param->rx_phys);
            llm_env.tx_phys = (param->all_phys & ALL_PHYS_TX_NO_PREF) ? (supp_phy_msk) : (param->tx_phys);
        }
    }

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

#if (BLE_PAST)
__STATIC int hci_le_set_dft_per_adv_sync_transf_param_cmd_handler(struct hci_le_set_dft_per_adv_sync_transf_param_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;

    do
    {
#if (BLE_ADV_LEGACY_ITF)
        // Check current interface version
        if (llm_env.adv_itf_version == LLM_ADV_ITF_LEGACY)
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        // Use extended interface
        llm_env.adv_itf_version = LLM_ADV_ITF_EXTENDED;
#endif //(BLE_ADV_LEGACY_ITF)

#if (BT_53)
        if (param->mode > SYNC_REP_EN_FILT_EN)
#else // !(BT_53)
        if (param->mode > SYNC_REP_EN)
#endif // !(BT_53)
            break;

        if (param->skip > SYNC_SKIP_MAX)
            break;

        if ((param->sync_to < SYNC_TIMEOUT_MIN) || (param->sync_to > SYNC_TIMEOUT_MAX))
            break;

        if ((param->cte_type & ~(NO_SYNC_AOA_BIT | NO_SYNC_AOD_1US_BIT | NO_SYNC_AOD_2US_BIT | NO_SYNC_NO_CTE_BIT)))
            break;

        // Save parameters
        llm_env.past_mode = param->mode;
        llm_env.past_cte_type = param->cte_type;
        llm_env.past_skip = param->skip;
        llm_env.past_sync_to = param->sync_to;

        status = CO_ERROR_NO_ERROR;

    }
    while (0);

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}
#endif //(BLE_PAST)
#endif //(BLE_CENTRAL || BLE_PERIPHERAL)

#if (BT_53) && (BLE_CENTRAL)
__STATIC int hci_le_set_dft_subrate_cmd_handler(struct hci_le_set_dft_subrate_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_NO_ERROR;

    // Check for valid parameters
    if ((param->subrate_min < CON_SUBRATE_MIN) || (param->subrate_min > param->subrate_max) || (param->subrate_max > CON_SUBRATE_MAX)
            || (param->subrate_max * (param->max_latency + 1) > CON_SUBRATE_MAX)
            || (param->cont_num >= param->subrate_max)
            || (param->supv_to < CON_SUP_TO_MIN) || (param->supv_to > CON_SUP_TO_MAX))
    {
        status = CO_ERROR_INVALID_HCI_PARAM;
    }
    else
    {
        // Store the new default values (does not affect existing connections)
        llm_env.subrate_info.subrate_min = param->subrate_min;
        llm_env.subrate_info.subrate_max = param->subrate_max;
        llm_env.subrate_info.max_latency = param->max_latency;
        llm_env.subrate_info.cont_num = param->cont_num;
        llm_env.subrate_info.timeout =  param->supv_to;
    }

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

#endif //(BLE_CENTRAL || BLE_PERIPHERAL)


#if (HCI_TL_SUPPORT)
__STATIC int hci_le_set_host_feature_cmd_handler(struct hci_le_set_host_feature_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;

    do
    {
        // If Bit_Value is not 0 or 1
        if (param->bit_value > 1)
            break;

#if (BLE_CENTRAL || BLE_PERIPHERAL)
        // If the Host issues this command while the Controller has an established ACL
        if (llm_is_con_act_ongoing_check())
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }
#endif //(BLE_CENTRAL || BLE_PERIPHERAL)

        // Check if feature bit is controlled by the Host and supported
        if (!(
                    ((param->bit_number == BLE_FEAT_CIS_HOST_SUPPORT)
#if (!BLE_ISO_PRESENT)
                     && (param->bit_value == 0)
#endif // (BLE_ISO_PRESENT)
                    )
#if (BT_53)
                    || (param->bit_number == BLE_FEAT_CON_SUBRATING_HOST_SUPPORT)
#endif // (BT_53)
                ))
        {
            status = CO_ERROR_UNSUPPORTED;
            break;
        }

        // Set the feature bit
        llm_env.le_feats.feats[param->bit_number >> 3] = (llm_env.le_feats.feats[param->bit_number >> 3] & ~(1 << (param->bit_number & 0x7))) | (param->bit_value << (param->bit_number & 0x7));

        status = CO_ERROR_NO_ERROR;
    }
    while (0);

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}
#endif // (HCI_TL_SUPPORT)

#if (RW_DEBUG)
__STATIC int hci_dbg_ble_reg_rd_cmd_handler(struct hci_dbg_ble_reg_rd_cmd const *param, uint16_t opcode)
{
    // Report the command complete event
    struct hci_dbg_ble_reg_rd_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_dbg_ble_reg_rd_cmd_cmp_evt);

    // Read register
    event->reg_value = lld_reg_rd(param->reg_addr);

    event->reg_addr  = param->reg_addr;
    event->status    = CO_ERROR_NO_ERROR;

    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}

__STATIC int hci_dbg_ble_reg_wr_cmd_handler(struct hci_dbg_ble_reg_wr_cmd const *param, uint16_t opcode)
{
    // Report the command complete event
    struct hci_dbg_ble_reg_wr_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_dbg_ble_reg_wr_cmd_cmp_evt);

    // Write register
    lld_reg_wr(param->reg_addr, param->reg_value);

    event->reg_addr  = param->reg_addr;
    event->status    = CO_ERROR_NO_ERROR;

    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}
#endif // (RW_DEBUG)





/*
 * HCI COMMAND HANDLING
 ****************************************************************************************
 */

/// The message handlers for HCI commands
HCI_CMD_HANDLER_TAB(llm)
{
#if (BLE_BROADCASTER)
    {HCI_LE_SET_EXT_ADV_EN_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_set_ext_adv_en_cmd_handler},
    {HCI_LE_SET_EXT_ADV_PARAM_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_set_ext_adv_param_cmd_handler},
    {HCI_LE_SET_EXT_ADV_DATA_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_set_ext_adv_data_cmd_handler},
#endif // (BLE_BROADCASTER)

//#if !BT_DUAL_MODE
#if (HCI_TL_SUPPORT)
    {HCI_RESET_CMD_OPCODE, (llm_hci_cmd_hdl_func_t) hci_reset_cmd_handler                     },
    {HCI_RD_LOCAL_VER_INFO_CMD_OPCODE, (llm_hci_cmd_hdl_func_t) hci_rd_local_ver_info_cmd_handler         },
    {HCI_RD_BD_ADDR_CMD_OPCODE, (llm_hci_cmd_hdl_func_t) hci_rd_bd_addr_cmd_handler                },
    {HCI_RD_LOCAL_SUPP_CMDS_CMD_OPCODE, (llm_hci_cmd_hdl_func_t) hci_rd_local_supp_cmds_cmd_handler        },
    {HCI_RD_LOCAL_SUPP_FEATS_CMD_OPCODE, (llm_hci_cmd_hdl_func_t) hci_rd_local_supp_feats_cmd_handler       },
    {HCI_SET_EVT_MASK_CMD_OPCODE, (llm_hci_cmd_hdl_func_t) hci_set_evt_mask_cmd_handler              },
    {HCI_RD_CON_ACCEPT_TO_CMD_OPCODE, (llm_hci_cmd_hdl_func_t) hci_rd_con_accept_to_cmd_handler          },
    {HCI_WR_CON_ACCEPT_TO_CMD_OPCODE, (llm_hci_cmd_hdl_func_t) hci_wr_con_accept_to_cmd_handler          },
    {HCI_SET_EVT_MASK_PAGE_2_CMD_OPCODE, (llm_hci_cmd_hdl_func_t) hci_set_evt_mask_page_2_cmd_handler       },
    {HCI_SET_CTRL_TO_HOST_FLOW_CTRL_CMD_OPCODE, (llm_hci_cmd_hdl_func_t) hci_set_ctrl_to_host_flow_ctrl_cmd_handler},
    {HCI_HOST_BUF_SIZE_CMD_OPCODE, (llm_hci_cmd_hdl_func_t) hci_host_buf_size_cmd_handler             },
    {HCI_HOST_NB_CMP_PKTS_CMD_OPCODE, (llm_hci_cmd_hdl_func_t) hci_host_nb_cmp_pkts_cmd_handler          },
#endif // (HCI_TL_SUPPORT)
    {HCI_RD_AFH_CH_ASSESS_MODE_CMD_OPCODE, (llm_hci_cmd_hdl_func_t) hci_rd_afh_ch_assess_mode_cmd_handler     },
    {HCI_WR_AFH_CH_ASSESS_MODE_CMD_OPCODE, (llm_hci_cmd_hdl_func_t) hci_wr_afh_ch_assess_mode_cmd_handler     },
    {HCI_SET_ECO_BASE_INTV_CMD_OPCODE, (llm_hci_cmd_hdl_func_t) hci_set_eco_base_intv_cmd_handler         },
    {HCI_CONFIG_DATA_PATH_CMD_OPCODE, (llm_hci_cmd_hdl_func_t) hci_config_data_path_cmd_handler          },
    {HCI_RD_LOCAL_SUPP_CODECS_V2_CMD_OPCODE, (llm_hci_cmd_hdl_func_t) hci_rd_local_supp_codecs_v2_cmd_handler   },
    {HCI_RD_LOCAL_SUPP_CODEC_CAP_CMD_OPCODE, (llm_hci_cmd_hdl_func_t) hci_rd_local_supp_codec_cap_cmd_handler   },
    {HCI_RD_LOCAL_SUPP_CTRL_DELAY_CMD_OPCODE, (llm_hci_cmd_hdl_func_t) hci_rd_local_supp_ctrl_delay_cmd_handler  },
//#endif // !BT_DUAL_MODE

    /// low energy commands
#if (HCI_TL_SUPPORT)
    {HCI_LE_SET_EVT_MASK_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_set_evt_mask_cmd_handler},
#if (BLE_PERIPHERAL || BLE_CENTRAL)
    {HCI_LE_RD_BUF_SIZE_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_rd_buf_size_cmd_handler},
#endif // (BLE_PERIPHERAL || BLE_CENTRAL)
    {HCI_LE_RD_LOCAL_SUPP_FEATS_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_rd_local_supp_feats_cmd_handler},
#endif // (HCI_TL_SUPPORT)
    {HCI_LE_SET_RAND_ADDR_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_set_rand_addr_cmd_handler},

#if (BLE_BROADCASTER)
#if (BLE_ADV_LEGACY_ITF)
    {HCI_LE_SET_ADV_PARAM_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_set_adv_param_cmd_handler},
    {HCI_LE_SET_ADV_DATA_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_set_adv_data_cmd_handler},
    {HCI_LE_SET_ADV_EN_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_set_adv_en_cmd_handler},
    {HCI_LE_SET_SCAN_RSP_DATA_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_set_scan_rsp_data_cmd_handler},
#endif //(BLE_ADV_LEGACY_ITF)
    {HCI_LE_RD_ADV_CHNL_TX_PW_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_rd_adv_ch_tx_pw_cmd_handler},
    {HCI_LE_SET_ADV_SET_RAND_ADDR_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_set_adv_set_rand_addr_cmd_handler},
    {HCI_LE_SET_EXT_SCAN_RSP_DATA_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_set_ext_scan_rsp_data_cmd_handler},
#if (HCI_TL_SUPPORT)
    {HCI_LE_RD_MAX_ADV_DATA_LEN_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_rd_max_adv_data_len_cmd_handler},
    {HCI_LE_RD_NB_SUPP_ADV_SETS_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_rd_nb_supp_adv_sets_cmd_handler},
#endif // (HCI_TL_SUPPORT)
    {HCI_LE_RMV_ADV_SET_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_rmv_adv_set_cmd_handler},
    {HCI_LE_CLEAR_ADV_SETS_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_clear_adv_sets_cmd_handler},
    {HCI_LE_SET_PER_ADV_PARAM_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_set_per_adv_param_cmd_handler},
    {HCI_LE_SET_PER_ADV_DATA_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_set_per_adv_data_cmd_handler},
    {HCI_LE_SET_PER_ADV_EN_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_set_per_adv_en_cmd_handler},
#if BLE_CONLESS_CTE_TX
    {HCI_LE_SET_CONLESS_CTE_TX_PARAM_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_set_conless_cte_tx_param_cmd_handler},
    {HCI_LE_SET_CONLESS_CTE_TX_EN_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_set_conless_cte_tx_en_cmd_handler},
#endif // BLE_CONLESS_CTE_TX
#if (BT_53)
    {HCI_LE_SET_DATA_RELATED_ADDR_CHG_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_set_data_related_addr_chg_cmd_handler},
#endif // BLE_CONLESS_CTE_TX
#endif //(BLE_BROADCASTER)

#if (BLE_OBSERVER)
#if (BLE_ADV_LEGACY_ITF)
    {HCI_LE_SET_SCAN_PARAM_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_set_scan_param_cmd_handler},
    {HCI_LE_SET_SCAN_EN_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_set_scan_en_cmd_handler},
#endif //(BLE_ADV_LEGACY_ITF)
    {HCI_LE_SET_EXT_SCAN_PARAM_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_set_ext_scan_param_cmd_handler},
    {HCI_LE_SET_EXT_SCAN_EN_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_set_ext_scan_en_cmd_handler},
    {HCI_LE_PER_ADV_CREATE_SYNC_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_per_adv_create_sync_cmd_handler},
    {HCI_LE_PER_ADV_CREATE_SYNC_CANCEL_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_per_adv_create_sync_cancel_cmd_handler},
    {HCI_LE_PER_ADV_TERM_SYNC_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_per_adv_term_sync_cmd_handler},
    {HCI_LE_ADD_DEV_TO_PER_ADV_LIST_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_add_dev_to_per_adv_list_cmd_handler},
    {HCI_LE_RMV_DEV_FROM_PER_ADV_LIST_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_rmv_dev_from_per_adv_list_cmd_handler},
    {HCI_LE_CLEAR_PER_ADV_LIST_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_clear_per_adv_list_cmd_handler},
    {HCI_LE_RD_PER_ADV_LIST_SIZE_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_rd_per_adv_list_size_cmd_handler},
    {HCI_LE_SET_PER_ADV_REC_EN_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_set_per_adv_rec_en_cmd_handler},
#if BLE_CONLESS_CTE_RX
    {HCI_LE_SET_CONLESS_IQ_SAMPL_EN_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_set_conless_iq_sampl_en_cmd_handler},
#endif // BLE_CONLESS_CTE_RX
#endif //(BLE_OBSERVER)

#if (BLE_CENTRAL)
#if (BLE_ADV_LEGACY_ITF)
    {HCI_LE_CREATE_CON_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_create_con_cmd_handler},
#endif //(BLE_ADV_LEGACY_ITF)
    {HCI_LE_CREATE_CON_CANCEL_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_create_con_cancel_cmd_handler},
    {HCI_LE_EXT_CREATE_CON_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_ext_create_con_cmd_handler},
#endif //(BLE_CENTRAL)

#if (BLE_CENTRAL || BLE_BROADCASTER)
    {HCI_LE_SET_HOST_CH_CLASS_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_set_host_ch_class_cmd_handler},
#endif //(BLE_CENTRAL || BLE_BROADCASTER)

    {HCI_LE_RD_WLST_SIZE_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_rd_wl_size_cmd_handler},
    {HCI_LE_CLEAR_WLST_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_clear_wlst_cmd_handler},
    {HCI_LE_ADD_DEV_TO_WLST_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_add_dev_to_wlst_cmd_handler},
    {HCI_LE_RMV_DEV_FROM_WLST_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_rmv_dev_from_wlst_cmd_handler},
    {HCI_LE_RD_RSLV_LIST_SIZE_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_rd_rslv_list_size_cmd_handler},
    {HCI_LE_ADD_DEV_TO_RSLV_LIST_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_add_dev_to_rslv_list_cmd_handler},
    {HCI_LE_RMV_DEV_FROM_RSLV_LIST_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_rmv_dev_from_rslv_list_cmd_handler},
    {HCI_LE_CLEAR_RSLV_LIST_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_clear_rslv_list_cmd_handler},
    {HCI_LE_SET_RSLV_PRIV_ADDR_TO_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_set_rslv_priv_addr_to_cmd_handler},
    {HCI_LE_SET_ADDR_RESOL_EN_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_set_addr_resol_en_cmd_handler},
    {HCI_LE_RD_PEER_RSLV_ADDR_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_rd_peer_rslv_addr_cmd_handler},
    {HCI_LE_RD_LOC_RSLV_ADDR_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_rd_loc_rslv_addr_cmd_handler},
    {HCI_LE_ENC_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_enc_cmd_handler},
    {HCI_LE_RAND_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_rand_cmd_handler},
    {HCI_LE_RD_SUPP_STATES_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_rd_supp_states_cmd_handler},
    {HCI_LE_RD_TX_PWR_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_rd_tx_pwr_cmd_handler},
    {HCI_LE_RD_RF_PATH_COMP_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_rd_rf_path_comp_cmd_handler},
    {HCI_LE_WR_RF_PATH_COMP_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_wr_rf_path_comp_cmd_handler},
    {HCI_LE_SET_PRIV_MODE_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_set_priv_mode_cmd_handler},
    {HCI_LE_MOD_SLEEP_CLK_ACC_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_mod_sleep_clk_acc_cmd_handler},
#if (BLE_AOD | BLE_AOA)
    {HCI_LE_RD_ANTENNA_INF_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_rd_antenna_inf_cmd_handler},
#endif // (BLE_AOD | BLE_AOA)

#if (BLE_PERIPHERAL || BLE_CENTRAL)
    {HCI_LE_RD_SUGGTED_DFT_DATA_LEN_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_rd_suggted_dft_data_len_cmd_handler},
    {HCI_LE_WR_SUGGTED_DFT_DATA_LEN_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_wr_suggted_dft_data_len_cmd_handler},
    {HCI_LE_RD_MAX_DATA_LEN_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_rd_max_data_len_cmd_handler},
#endif // (BLE_PERIPHERAL || BLE_CENTRAL)

#if (BLE_TEST_MODE_SUPPORT)
    {HCI_LE_RX_TEST_V1_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_rx_test_v1_cmd_handler},
    {HCI_LE_TX_TEST_V1_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_tx_test_v1_cmd_handler},
    {HCI_LE_RX_TEST_V2_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_rx_test_v2_cmd_handler},
    {HCI_LE_TX_TEST_V2_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_tx_test_v2_cmd_handler},
    {HCI_LE_RX_TEST_V3_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_rx_test_v3_cmd_handler},
    {HCI_LE_TX_TEST_V3_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_tx_test_v3_cmd_handler},
    {HCI_LE_TX_TEST_V4_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_tx_test_v4_cmd_handler},
    {HCI_LE_TEST_END_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_test_end_cmd_handler},
#endif //(BLE_TEST_MODE_SUPPORT)

    {HCI_LE_RD_LOC_P256_PUB_KEY_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_rd_local_p256_public_key_cmd_handler},
    {HCI_LE_GEN_DHKEY_V1_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_gen_dhkey_v1_cmd_handler},
    {HCI_LE_GEN_DHKEY_V2_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_gen_dhkey_v2_cmd_handler},

#if (BLE_CENTRAL || BLE_PERIPHERAL)
    {HCI_LE_SET_DFT_PHY_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_set_dft_phy_cmd_handler},
#if (BLE_PAST)
    {HCI_LE_SET_DFT_PER_ADV_SYNC_TRANSF_PARAM_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_set_dft_per_adv_sync_transf_param_cmd_handler},
#endif // (BLE_PAST)
#endif // (BLE_CENTRAL || BLE_PERIPHERAL)

#if (BT_53) && (BLE_CENTRAL)
    {HCI_LE_SET_DFT_SUBRATE_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_set_dft_subrate_cmd_handler},
#endif // (BT_53) && (BLE_CENTRAL)

#if (HCI_TL_SUPPORT)
    {HCI_LE_SET_HOST_FEATURE_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_le_set_host_feature_cmd_handler},
#endif // (HCI_TL_SUPPORT)

#if (RW_DEBUG)
    {HCI_DBG_BLE_REG_RD_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_dbg_ble_reg_rd_cmd_handler},
    {HCI_DBG_BLE_REG_WR_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_dbg_ble_reg_wr_cmd_handler},
#endif // (RW_DEBUG)

#if (!HOST_PRESENT)
    {HCI_VS_LE_DECRYPT_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_vs_le_decrypt_cmd_handler},
#endif // (!HOST_PRESENT)

#if (EAVESDROPPING_SUPPORT)
#if (BLE_BROADCASTER || BLE_PERIPHERAL)
    {HCI_VS_SET_CUST_ADV_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_vs_set_cust_adv_cmd_handler},
#endif //(BLE_BROADCASTER || BLE_PERIPHERAL)
#endif //(EAVESDROPPING_SUPPORT)

#if (EAVESDROPPING_SUPPORT)
    {HCI_VS_LE_ENABLE_ENH_SET_DATA_LEN_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_vs_le_enable_enh_set_data_len_cmd_handler},
    {HCI_VS_LE_ENH_WR_SUGG_DFT_DATA_LEN_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_vs_le_enh_wr_suggted_dft_data_len_cmd_handler},
    {HCI_VS_LE_SET_CUST_TRANSMITTER_TEST_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_vs_le_set_cust_transmitter_test_cmd_handler},
#endif // EAVESDROPPING_SUPPORT

#if (EAVESDROPPING_SUPPORT)
#if (BLE_BROADCASTER || BLE_PERIPHERAL)
    {HCI_VS_LE_SET_MANUAL_ADV_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_vs_le_set_manual_adv_cmd_handler},
#endif //(BLE_BROADCASTER || BLE_PERIPHERAL)
#if (BLE_OBSERVER)
    {HCI_VS_LE_SET_SEG_SCAN_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_vs_le_set_seg_scan_cmd_handler},
#endif //(BLE_OBSERVER)
#endif //(EAVESDROPPING_SUPPORT)

#if (BLE_CH_SCAN_SUPPORT)
    {HCI_VS_LE_CH_SCAN_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_vs_le_ch_scan_cmd_handler},
    {HCI_VS_LE_CH_SCAN_END_CMD_OPCODE, (llm_hci_cmd_hdl_func_t)hci_vs_le_ch_scan_end_cmd_handler},
#endif //(BLE_CH_SCAN_SUPPORT)

};

/**
 ****************************************************************************************
 * @brief Handles any HCI command
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER_NO_STATIC(hci_command_llm, void)
{
    int return_status = KE_MSG_CONSUMED;

    // Check if there is a handler corresponding to the original command opcode
    for (uint16_t i = 0; i < ARRAY_LEN(llm_hci_command_handler_tab); i++)
    {
        // Check if opcode matches
        if (llm_hci_command_handler_tab[i].opcode == src_id)
        {
            // Check if there is a handler function
            if (llm_hci_command_handler_tab[i].func != NULL)
            {
                // Call handler
                return_status = llm_hci_command_handler_tab[i].func(param, src_id);
            }
            break;
        }
    }

    return return_status;
}

/// @} LLMHCI
