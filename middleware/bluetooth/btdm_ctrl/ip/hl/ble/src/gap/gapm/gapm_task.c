/**
 ****************************************************************************************
 *
 * @file gapm_task.c
 *
 * @brief Generic Access Profile Manager Task implementation.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup GAPM_TASK Generic Access Profile Manager Task
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"
#include "rwble_hl.h"
#include "rwip.h"            // Common API to retrieve device paramters

#include <string.h>

#include "co_error.h"
#include "co_bt.h"
#include "co_math.h"
#include "co_version.h"
#include "co_utils.h"        // core utility functions

#include "gap.h"

#include "gattm_task.h"
#include "gapm_task.h"
#include "gapm_int.h"

#include "gapc.h"
#include "gapc_task.h"

#include "attm.h"


#include "ke_mem.h"

#include "hci.h"


#include "ke_timer.h"

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Format of a HCI event handler function
typedef int (*gapm_hci_evt_hdl_func_t)(uint16_t opcode, void const *param);

/// Element of a HCI command handler table.
struct gapm_hci_evt_handler
{
    /// Command opcode
    uint16_t opcode;
    /// Pointer to the handler function for HCI command.
    gapm_hci_evt_hdl_func_t func;
};


/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * MACROS
 ****************************************************************************************
 */

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */


/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

#if 0
//#if (BLE_ISOGEN)
/**
 ****************************************************************************************
 * @brief Handle reception of HCI LE Scan Request Received event.
 * This event indicates that a SCAN_REQ PDU or an AUX_SCAN_REQ PDU has been received.
 *
 * @param[in] opcode      Operation code of received message
 * @param[in] p_event     Pointer to the parameters of the message.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_vs_isogen_stat_evt_handler(uint16_t opcode, struct hci_vs_isogen_stat_evt const *p_event)
{
    // trigger statistic information of the ISO exchange
    struct gapm_iso_stat_ind *ind = KE_MSG_ALLOC(GAPM_ISO_STAT_IND, APP_MAIN_TASK, TASK_GAPM, gapm_iso_stat_ind);

    ind->iso_hdl           = p_event->iso_hdl;
    ind->nb_tx             = p_event->nb_tx;
    ind->nb_tx_ok          = p_event->nb_tx_ok;
    ind->nb_tx_not_granted = p_event->nb_tx_not_granted;
    ind->nb_rx             = p_event->nb_rx;
    ind->nb_rx_ok          = p_event->nb_rx_ok;
    ind->nb_rx_not_granted = p_event->nb_rx_not_granted;
    ind->nb_rx_data_err    = p_event->nb_rx_data_err;
    ind->nb_rx_crc_err     = p_event->nb_rx_crc_err;
    ind->nb_rx_sync_err    = p_event->nb_rx_sync_err;
    ind->nb_rx_empty       = p_event->nb_rx_empty;

    ke_msg_send(ind);

    return (KE_MSG_CONSUMED);
}
#endif // (BLE_ISOGEN)


#if (!BLE_EMB_PRESENT)
#if (BLE_CENTRAL || BLE_PERIPHERAL)
/**
 ****************************************************************************************
 * @brief Handle reception of HCI LE Scan Request Received event.
 * This event indicates that a SCAN_REQ PDU or an AUX_SCAN_REQ PDU has been received.
 *
 * @param[in] opcode      Operation code of received message
 * @param[in] p_event     Pointer to the parameters of the message.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_le_con_evt_handler(uint16_t opcode, struct hci_le_con_evt const *p_event)
{
    int message_status = KE_MSG_CONSUMED;
    // Get connection index
    uint8_t idx = gapc_get_conidx(p_event->conhdl);

    if ((p_event->conhdl != GAP_INVALID_CONHDL) && (idx != GAP_INVALID_CONIDX))
    {
        // forward message to proper connection
        ke_msg_forward(p_event, KE_BUILD_ID(TASK_GAPC, idx), 0);
        message_status = KE_MSG_NO_FREE;
    }
    else
    {
        // message cannot be forwarded
        ASSERT_WARN(0, opcode, p_event->conhdl);
    }

    return (message_status);
}
#endif //(BLE_CENTRAL || BLE_PERIPHERAL)
#endif // (!BLE_EMB_PRESENT)


#if (RW_DEBUG)

extern ke_task_id_t gapm_get_default_requester(void);

__STATIC int gapm_dbg_rssi_notify_task_msg_handler(uint16_t opcode, struct hci_dbg_bt_rssi_notify_evt_t const *p_event)
{
    // inform main application that a message to an unknown task has been requested
    uint8_t num = 0;
    uint32_t max = MAX_BT_ACL_LINK; // For BLE in future
    for (uint32_t i = 0; i < max; i++)
    {
        if (p_event->ave_rssi[i] != 0)
        {
            num++;
        }
    }
    struct gapm_dbg_rssi_notify_ind *ind = KE_MSG_ALLOC_DYN(GAPM_DBG_RSSI_NOTIFY_IND, gapm_get_default_requester(), TASK_GAPM, gapm_dbg_rssi_notify_ind, num);
    ind->rssi_num = num;
    num = 0;
    memcpy(ind->channel_asset, p_event->chan_lvl, AFH_NB_CHANNEL_MAX);
    for (uint32_t i = 0; i < max; i++)
    {
        if (p_event->ave_rssi[i] != 0)
        {
            *((int8_t *)&ind->rssi_ave + num++) = p_event->ave_rssi[i];
        }
    }

    ke_msg_send(ind);

    return (KE_MSG_CONSUMED);
}


__STATIC int gapm_dbg_assert_task_msg_handler(uint16_t opcode, struct hci_dbg_assert_evt const *p_event)
{
    rt_kprintf("Controller asserted %s, type %d, line %d, para0 %d, para1 %d\r\n", p_event->file, p_event->type, p_event->line, p_event->param0, p_event->param1);
    struct gapm_dbg_assert_ind *ind = KE_MSG_ALLOC(GAPM_ASSERT_IND, gapm_get_default_requester(), TASK_GAPM, gapm_dbg_assert_ind);
    ind->type = p_event->type;
    ke_msg_send(ind);

    return (KE_MSG_CONSUMED);
}
#endif
/*
 * HCI HANDLERS DEFINITIONS
 ****************************************************************************************
 */

// Handler in gapm_cfg.c
extern int hci_basic_cmd_cmp_evt_cfg_handler(uint16_t opcode, struct hci_basic_cmd_cmp_evt const *event);
extern int hci_le_rd_buff_size_cmd_cmp_evt_handler(uint16_t opcode, struct hci_le_rd_buf_size_cmd_cmp_evt const *event);
extern int hci_rd_buff_size_cmd_cmp_evt_handler(uint16_t opcode, struct hci_rd_buf_size_cmd_cmp_evt const *event);
extern int hci_rd_local_ver_info_cmd_cmp_evt_handler(uint16_t opcode, struct hci_rd_local_ver_info_cmd_cmp_evt const *event);
extern int hci_rd_bd_addr_cmd_cmp_evt_handler(uint16_t opcode, struct hci_rd_bd_addr_cmd_cmp_evt const *event);
extern int hci_le_rd_adv_chnl_tx_pw_cmd_cmp_evt_handler(uint16_t opcode, struct hci_rd_adv_chnl_tx_pw_cmd_cmp_evt const *event);
extern int hci_le_rd_suggted_dft_data_len_cmd_cmp_evt_handler(uint16_t opcode, struct hci_le_rd_suggted_dft_data_len_cmd_cmp_evt const *event);
extern int hci_le_rd_max_data_len_cmd_cmp_evt_handler(uint16_t opcode, struct hci_le_rd_max_data_len_cmd_cmp_evt const *event);
extern int hci_basic_cmd_cmp_evt_rl_cfg_handler(uint16_t opcode, struct hci_basic_cmd_cmp_evt const *event);
extern int hci_le_rd_tx_pwr_cmd_cmp_evt_handler(uint16_t opcode, struct hci_le_rd_tx_pwr_cmd_cmp_evt const *p_event);
extern int hci_le_rd_rf_path_comp_cmd_cmp_evt_handler(uint16_t opcode, struct hci_le_rd_rf_path_comp_cmd_cmp_evt const *p_event);
extern int hci_test_end_cmd_cmp_evt_handler(uint16_t opcode, struct hci_test_end_cmd_cmp_evt const *p_event);

// Handler in gapm_list.c
extern int hci_le_cmd_cmp_evt_list_handler(uint16_t opcode, struct hci_basic_cmd_cmp_evt const *p_event);
extern int hci_le_rd_wlst_size_cmd_cmp_evt_handler(uint16_t opcode, struct hci_le_rd_wlst_size_cmd_cmp_evt const *param);
extern int hci_le_rd_ral_size_cmd_cmp_evt_handler(uint16_t opcode, struct hci_le_rd_rslv_list_size_cmd_cmp_evt const *param);
extern int hci_le_rd_pal_size_cmd_cmp_evt_handler(uint16_t opcode, struct hci_le_rd_per_adv_list_size_cmd_cmp_evt const *p_param);
extern int hci_le_rd_ral_addr_cmd_cmp_evt_handler(ke_msg_id_t const msgid, struct hci_le_rd_peer_rslv_addr_cmd_cmp_evt const *param);

// Handler in gapm_actv.c
extern int hci_le_enh_con_cmp_evt_handler(uint16_t opcode, struct hci_le_enh_con_cmp_evt const *event);

// Handler in gapm_addr.c
extern int hci_le_cmd_cmp_evt_addr_handler(uint16_t opcode, struct hci_basic_cmd_cmp_evt const *p_event);

// Handler in gapm_adv.c
extern int hci_le_cmd_cmp_evt_adv_handler(uint16_t opcode, struct hci_basic_cmd_cmp_evt const *p_event);
extern int hci_le_read_max_adv_data_len_cmd_cmp_evt_handler(uint16_t opcode, struct hci_le_rd_max_adv_data_len_cmd_cmp_evt const *p_event);
extern int hci_le_rd_nb_supp_adv_sets_cmd_cmp_evt_handler(uint16_t opcode, struct hci_le_rd_nb_supp_adv_sets_cmd_cmp_evt const *event);
extern int hci_le_adv_set_term_evt_handler(uint16_t opcode, struct hci_le_adv_set_term_evt const *event);
extern int hci_le_scan_req_rcvd_evt_handler(uint16_t opcode, struct hci_le_scan_req_rcvd_evt const *event);

// Handler in gapm_scan.c
extern int hci_le_cmd_cmp_evt_scan_handler(uint16_t opcode, struct hci_basic_cmd_cmp_evt const *event);
extern int hci_le_ext_adv_report_evt_handler(uint16_t opcode, struct hci_le_ext_adv_report_evt const *event);
extern int hci_le_scan_timeout_evt_handler(uint16_t opcode, struct hci_le_scan_timeout_evt const *event);

// Handler in gapm_init.c
extern int hci_le_cmd_cmp_evt_init_handler(uint16_t opcode, struct hci_basic_cmd_cmp_evt const *event);

// Handler in gapm_period_sync.c
extern int hci_le_cmd_cmp_evt_per_sync_handler(uint16_t opcode, struct hci_basic_cmd_cmp_evt const *event);
int hci_le_per_adv_sync_est_evt_handler(uint16_t opcode, struct hci_le_per_adv_sync_est_evt const *p_event);
int hci_le_per_adv_report_evt_handler(uint16_t opcode, struct hci_le_per_adv_report_evt const *p_event);
int hci_le_per_adv_sync_lost_evt_handler(uint16_t opcode, struct hci_le_per_adv_sync_lost_evt const *p_event);

// Handler in gapm_smp.c
extern int hci_le_enc_cmd_cmp_evt_handler(uint16_t opcode, struct hci_le_enc_cmd_cmp_evt *param);
extern int hci_le_rd_local_p256_public_key_cmp_evt_handler(uint16_t opcode, struct hci_rd_local_p256_public_key_cmp_evt const *cmp_evt);
extern int hci_le_generate_dhkey_cmp_evt_handler(uint16_t opcode, struct hci_le_gen_dhkey_cmp_evt const *param);
extern int hci_le_gen_dhkey_stat_evt_handler(uint16_t opcode, struct hci_basic_cmd_cmp_evt const *p_event);

/// The message handlers for HCI command complete events
static const struct gapm_hci_evt_handler gapm_hci_cmd_cmp_event_handler_tab[] =
{
    { HCI_RESET_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_basic_cmd_cmp_evt_cfg_handler },
    { HCI_SET_EVT_MASK_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_basic_cmd_cmp_evt_cfg_handler },
    { HCI_LE_SET_EVT_MASK_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_basic_cmd_cmp_evt_cfg_handler },
    { HCI_LE_RX_TEST_V2_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_basic_cmd_cmp_evt_cfg_handler },
    { HCI_LE_TX_TEST_V2_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_basic_cmd_cmp_evt_cfg_handler },
    { HCI_LE_TEST_END_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_test_end_cmd_cmp_evt_handler  },

#if (BLE_CENTRAL || BLE_PERIPHERAL)
    { HCI_LE_RD_BUF_SIZE_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_le_rd_buff_size_cmd_cmp_evt_handler },
    { HCI_RD_BUF_SIZE_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_rd_buff_size_cmd_cmp_evt_handler },
#endif // (BLE_CENTRAL || BLE_PERIPHERAL)

#ifndef GAPM_SLIM
    { HCI_LE_SET_HOST_CH_CLASS_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_basic_cmd_cmp_evt_cfg_handler },
#endif // !GAPM_SLIM
    { HCI_RD_LOCAL_VER_INFO_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_rd_local_ver_info_cmd_cmp_evt_handler },
    { HCI_RD_BD_ADDR_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_rd_bd_addr_cmd_cmp_evt_handler },

    { HCI_LE_WR_SUGGTED_DFT_DATA_LEN_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_basic_cmd_cmp_evt_cfg_handler },
#ifndef GAPM_SLIM
    { HCI_LE_RD_SUGGTED_DFT_DATA_LEN_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_le_rd_suggted_dft_data_len_cmd_cmp_evt_handler },
#endif // !GAPM_SLIM
    { HCI_LE_RD_MAX_DATA_LEN_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_le_rd_max_data_len_cmd_cmp_evt_handler },
#ifndef GAPM_SLIM
    { HCI_LE_RD_TX_PWR_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_le_rd_tx_pwr_cmd_cmp_evt_handler },
#endif //GAPM_SLIM
#ifndef GAPM_SLIM
    { HCI_LE_RD_RF_PATH_COMP_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_le_rd_rf_path_comp_cmd_cmp_evt_handler },
#endif // GAPM_SLIM
    { HCI_LE_WR_RF_PATH_COMP_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_basic_cmd_cmp_evt_cfg_handler },

    { HCI_LE_SET_DFT_PHY_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_basic_cmd_cmp_evt_cfg_handler },

    /* White list management */
    { HCI_LE_RD_WLST_SIZE_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_le_rd_wlst_size_cmd_cmp_evt_handler },
    { HCI_LE_ADD_DEV_TO_WLST_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_le_cmd_cmp_evt_list_handler },
    { HCI_LE_CLEAR_WLST_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_le_cmd_cmp_evt_list_handler },

    /* Advertising procedure */
#if (BLE_BROADCASTER)
#ifdef RWIP_BLE_4
    { HCI_LE_SET_ADV_PARAM_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_le_cmd_cmp_evt_adv_handler },
    { HCI_LE_SET_ADV_DATA_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_le_cmd_cmp_evt_adv_handler },
    { HCI_LE_SET_SCAN_RSP_DATA_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_le_cmd_cmp_evt_adv_handler },
    { HCI_LE_SET_ADV_EN_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_le_cmd_cmp_evt_adv_handler },
#else
    { HCI_LE_SET_EXT_ADV_PARAM_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_le_cmd_cmp_evt_adv_handler },
    { HCI_LE_SET_EXT_ADV_DATA_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_le_cmd_cmp_evt_adv_handler },
    { HCI_LE_SET_EXT_SCAN_RSP_DATA_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_le_cmd_cmp_evt_adv_handler },
    { HCI_LE_SET_EXT_ADV_EN_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_le_cmd_cmp_evt_adv_handler },
#ifndef GAPM_SLIM
    { HCI_LE_SET_PER_ADV_PARAM_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_le_cmd_cmp_evt_adv_handler },
    { HCI_LE_SET_PER_ADV_DATA_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_le_cmd_cmp_evt_adv_handler },
    { HCI_LE_SET_PER_ADV_EN_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_le_cmd_cmp_evt_adv_handler },
#endif // GAPM_SLIM
    { HCI_LE_RD_MAX_ADV_DATA_LEN_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_le_read_max_adv_data_len_cmd_cmp_evt_handler },
    { HCI_LE_RD_NB_SUPP_ADV_SETS_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_le_rd_nb_supp_adv_sets_cmd_cmp_evt_handler },
    { HCI_LE_RMV_ADV_SET_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_le_cmd_cmp_evt_adv_handler },
    { HCI_LE_CLEAR_ADV_SETS_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_le_cmd_cmp_evt_adv_handler },
#endif
#endif // (BLE_BROADCASTER)

    /* Scan procedure */
#if (BLE_OBSERVER)
    { HCI_LE_SET_EXT_SCAN_PARAM_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_le_cmd_cmp_evt_scan_handler },
    { HCI_LE_SET_EXT_SCAN_EN_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_le_cmd_cmp_evt_scan_handler },
#endif //(BLE_OBSERVER)

#if (BLE_OBSERVER)
    /* Periodic synchronization procedure */
    { HCI_LE_PER_ADV_CREATE_SYNC_CANCEL_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_le_cmd_cmp_evt_per_sync_handler },
    { HCI_LE_PER_ADV_TERM_SYNC_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_le_cmd_cmp_evt_per_sync_handler },
#endif // (BLE_OBSERVER)

#if (BLE_OBSERVER)
    /* Periodic advertiser list management */
    { HCI_LE_ADD_DEV_TO_PER_ADV_LIST_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_le_cmd_cmp_evt_list_handler },
    { HCI_LE_CLEAR_PER_ADV_LIST_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_le_cmd_cmp_evt_list_handler },
    { HCI_LE_RD_PER_ADV_LIST_SIZE_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_le_rd_pal_size_cmd_cmp_evt_handler },
#endif // (BLE_OBSERVER)

    /* Connection procedure */
#if (BLE_CENTRAL)
    { HCI_LE_CREATE_CON_CANCEL_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_le_cmd_cmp_evt_init_handler },
#endif // (BLE_CENTRAL)

    /* Address Management */
    { HCI_LE_SET_RAND_ADDR_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_le_cmd_cmp_evt_addr_handler },
    { HCI_LE_SET_ADV_SET_RAND_ADDR_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_le_cmd_cmp_evt_addr_handler },

#if ((BLE_OBSERVER) || (BLE_PERIPHERAL))
    /* retrieve adv tx power level value */
    { HCI_LE_RD_ADV_CHNL_TX_PW_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_le_rd_adv_chnl_tx_pw_cmd_cmp_evt_handler },
#endif // ((BLE_OBSERVER) || (BLE_PERIPHERAL))

#if (!BLE_EMB_PRESENT)
    {HCI_LE_ENC_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_le_enc_cmd_cmp_evt_handler},
#endif // (!BLE_EMB_PRESENT)

    /* Resolving List Management */
    { HCI_LE_RD_RSLV_LIST_SIZE_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_le_rd_ral_size_cmd_cmp_evt_handler },
    { HCI_LE_RD_PEER_RSLV_ADDR_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_le_rd_ral_addr_cmd_cmp_evt_handler },
    { HCI_LE_RD_LOC_RSLV_ADDR_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_le_rd_ral_addr_cmd_cmp_evt_handler },
    { HCI_LE_SET_ADDR_RESOL_EN_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_basic_cmd_cmp_evt_rl_cfg_handler },
    { HCI_LE_SET_RSLV_PRIV_ADDR_TO_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_basic_cmd_cmp_evt_rl_cfg_handler },
    { HCI_LE_ADD_DEV_TO_RSLV_LIST_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_le_cmd_cmp_evt_list_handler },
    { HCI_LE_CLEAR_RSLV_LIST_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_le_cmd_cmp_evt_list_handler },
    { HCI_LE_SET_PRIV_MODE_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_le_cmd_cmp_evt_list_handler },
};


#if (BLE_MESH || BLE_OBSERVER || BLE_PERIPHERAL)
/// The message handlers for HCI status event
static const struct gapm_hci_evt_handler  gapm_hci_cmd_stat_event_handler_tab[] =
{
#if (BLE_CENTRAL)
    { HCI_LE_EXT_CREATE_CON_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_le_cmd_cmp_evt_init_handler     },
#endif //(BLE_CENTRAL)

#if (BLE_OBSERVER)
    { HCI_LE_PER_ADV_CREATE_SYNC_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_le_cmd_cmp_evt_per_sync_handler },
#endif //(BLE_OBSERVER)

#if (BLE_MESH || BLE_CENTRAL || BLE_PERIPHERAL)
#if (SECURE_CONNECTIONS)
    { HCI_LE_GEN_DHKEY_V1_CMD_OPCODE, (gapm_hci_evt_hdl_func_t) hci_le_gen_dhkey_stat_evt_handler   },
#endif //(SECURE_CONNECTIONS)
#endif //(BLE_MESH || BLE_CENTRAL || BLE_PERIPHERAL)
};
#endif //(BLE_MESH || BLE_OBSERVER || BLE_PERIPHERAL)

#if (BLE_OBSERVER || BLE_BROADCASTER)
/// The message handlers for HCI LE events
static const struct gapm_hci_evt_handler gapm_hci_le_event_handler_tab[] =
{
#if (BLE_OBSERVER)
    /* Scan procedure */
    { HCI_LE_EXT_ADV_REPORT_EVT_SUBCODE, (gapm_hci_evt_hdl_func_t) hci_le_ext_adv_report_evt_handler },
    { HCI_LE_SCAN_TIMEOUT_EVT_SUBCODE, (gapm_hci_evt_hdl_func_t) hci_le_scan_timeout_evt_handler },

    /* Periodic synchronization procedure */
    { HCI_LE_PER_ADV_SYNC_EST_EVT_SUBCODE, (gapm_hci_evt_hdl_func_t) hci_le_per_adv_sync_est_evt_handler },
    { HCI_LE_PER_ADV_REPORT_EVT_SUBCODE, (gapm_hci_evt_hdl_func_t) hci_le_per_adv_report_evt_handler },
    { HCI_LE_PER_ADV_SYNC_LOST_EVT_SUBCODE, (gapm_hci_evt_hdl_func_t) hci_le_per_adv_sync_lost_evt_handler },
#endif //(BLE_OBSERVER)

#if (BLE_CENTRAL || BLE_PERIPHERAL)
    { HCI_LE_ENH_CON_CMP_EVT_SUBCODE, (gapm_hci_evt_hdl_func_t) hci_le_enh_con_cmp_evt_handler },
    { HCI_LE_CON_CMP_EVT_SUBCODE, (gapm_hci_evt_hdl_func_t) hci_le_enh_con_cmp_evt_handler },

#endif //(BLE_CENTRAL || BLE_PERIPHERAL)

#if (BLE_MESH || BLE_CENTRAL || BLE_PERIPHERAL)
#if (SECURE_CONNECTIONS)
    { HCI_LE_RD_LOC_P256_PUB_KEY_CMP_EVT_SUBCODE, (gapm_hci_evt_hdl_func_t) hci_le_rd_local_p256_public_key_cmp_evt_handler },
    { HCI_LE_GEN_DHKEY_CMP_EVT_SUBCODE, (gapm_hci_evt_hdl_func_t) hci_le_generate_dhkey_cmp_evt_handler },
#endif //(SECURE_CONNECTIONS)
#endif //(BLE_MESH || BLE_CENTRAL || BLE_PERIPHERAL)

#if (BLE_BROADCASTER)
    /* Advertising procedure */
    { HCI_LE_ADV_SET_TERMINATED_EVT_SUBCODE, (gapm_hci_evt_hdl_func_t) hci_le_adv_set_term_evt_handler },
    { HCI_LE_SCAN_REQ_RCVD_EVT_SUBCODE, (gapm_hci_evt_hdl_func_t) hci_le_scan_req_rcvd_evt_handler },
#endif //(BLE_BROADCASTER)

#if (!BLE_EMB_PRESENT)
#if (BLE_CENTRAL || BLE_PERIPHERAL)
    { HCI_LE_LTK_REQUEST_EVT_SUBCODE, (gapm_hci_evt_hdl_func_t) hci_le_con_evt_handler },
    { HCI_LE_REM_CON_PARAM_REQ_EVT_SUBCODE, (gapm_hci_evt_hdl_func_t) hci_le_con_evt_handler },
    { HCI_LE_DATA_LEN_CHG_EVT_SUBCODE, (gapm_hci_evt_hdl_func_t) hci_le_con_evt_handler },
    { HCI_LE_PHY_UPD_CMP_EVT_SUBCODE, (gapm_hci_evt_hdl_func_t) hci_le_con_evt_handler },
    { HCI_LE_CH_SEL_ALGO_EVT_SUBCODE, (gapm_hci_evt_hdl_func_t) hci_le_con_evt_handler },
#endif //(BLE_CENTRAL || BLE_PERIPHERAL)
#endif //(!BLE_EMB_PRESENT)
};
#endif //(BLE_OBSERVER || BLE_BROADCASTER)

#if (RW_DEBUG)
/// The message handlers for HCI VS events
static const struct gapm_hci_evt_handler gapm_hci_vs_event_handler_tab[] =
{
    { HCI_DBG_ASSERT_EVT_SUBCODE, (gapm_hci_evt_hdl_func_t) gapm_dbg_assert_task_msg_handler },
    { HCI_DBG_BT_RSSI_NOTIFY_EVT_SUBCODE, (gapm_hci_evt_hdl_func_t) gapm_dbg_rssi_notify_task_msg_handler },
};
#endif

/*
 * MESSAGES HANDLERS DEFINITIONS
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @brief Handles any HCI event
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPM).
 * @param[in] opcode    HCI Operation code is filled in source message ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int gapm_hci_handler(ke_msg_id_t const msgid, void const *event, ke_task_id_t dest_id, ke_task_id_t opcode)
{
    int return_status = KE_MSG_CONSUMED;

    const struct gapm_hci_evt_handler *handler_tab = NULL;
    uint32_t tab_size = 0;

    switch (msgid)
    {
    case HCI_CMD_CMP_EVENT:
    {
        handler_tab = gapm_hci_cmd_cmp_event_handler_tab;
        tab_size    = sizeof(gapm_hci_cmd_cmp_event_handler_tab) / sizeof(gapm_hci_cmd_cmp_event_handler_tab[0]);
    }
    break;

#if (BLE_MESH || BLE_OBSERVER || BLE_PERIPHERAL)
    case HCI_CMD_STAT_EVENT:
    {
        handler_tab = gapm_hci_cmd_stat_event_handler_tab;
        tab_size    = sizeof(gapm_hci_cmd_stat_event_handler_tab) / sizeof(gapm_hci_cmd_stat_event_handler_tab[0]);
    }
    break;
#endif  //(BLE_MESH || BLE_OBSERVER || BLE_PERIPHERAL)

#if (BLE_OBSERVER || BLE_PERIPHERAL)
    case HCI_LE_EVENT:
    {
        handler_tab = gapm_hci_le_event_handler_tab;
        tab_size    = sizeof(gapm_hci_le_event_handler_tab) / sizeof(gapm_hci_le_event_handler_tab[0]);
        // Get subcode at from message parameters (1st byte position)
        opcode      = *((uint8_t *)event);
    }
    break;
#endif // (BLE_OBSERVER || BLE_PERIPHERAL)
#if (RW_DEBUG)
    case HCI_DBG_EVT:
    {
        handler_tab = gapm_hci_vs_event_handler_tab;
        tab_size    = sizeof(gapm_hci_vs_event_handler_tab) / sizeof(gapm_hci_vs_event_handler_tab[0]);
        // Get subcode at from message parameters (1st byte position)
        opcode      = *((uint8_t *)event);
    }
    break;
#endif
    default:   /* should not occurs, nothing to do */
        break;
    }

    // Check if there is a handler corresponding to the original command opcode
    for (uint32_t i = 0; i < tab_size; i++)
    {
        // Check if opcode matches
        if (handler_tab[i].opcode == opcode)
        {
            // Check if there is a handler function
            if (handler_tab[i].func != NULL)
            {
                // Call handler
                return_status = handler_tab[i].func(opcode, event);
            }
            break;
        }
    }

    return return_status;
}



/**
 ****************************************************************************************
 * @brief Default message handler
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPM).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int gapm_default_msg_handler(ke_msg_id_t const msgid, void *event,
                                      ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Check if it's a GAPM message
    if (MSG_T(msgid) == TASK_ID_GAPM)
    {
        // prepare unknown message indication
        struct gapm_unknown_msg_ind *p_ind = KE_MSG_ALLOC(GAPM_UNKNOWN_MSG_IND,
                                             src_id, dest_id, gapm_unknown_msg_ind);

        p_ind->unknown_msg_id = msgid;

        // send event
        ke_msg_send(p_ind);
    }
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Message to an Unknown task received
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int gapm_unknown_task_msg_handler(ke_msg_id_t const msgid,  void *param,
        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    struct ke_msg *msg = ke_param2msg(param);

    // inform main application that a message to an unknown task has been requested
    struct gapm_unknown_task_ind *ind = KE_MSG_ALLOC(GAPM_UNKNOWN_TASK_IND, APP_MAIN_TASK, dest_id, gapm_unknown_task_ind);
    ind->msg_id = msg->param_len;
    ind->task_id = src_id;
    ke_msg_send(ind);

    return (KE_MSG_CONSUMED);
}





// Handler in gapm_cfg.c
extern int gapm_reset_cmd_handler(ke_msg_id_t const msgid, struct gapm_reset_cmd *param, ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int gapm_set_dev_config_cmd_handler(ke_msg_id_t const msgid, struct gapm_set_dev_config_cmd *param, ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int gapm_set_channel_map_cmd_handler(ke_msg_id_t const msgid, struct gapm_set_channel_map_cmd *param, ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int gapm_get_dev_info_cmd_handler(ke_msg_id_t const msgid, struct gapm_get_dev_info_cmd *param, ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int gapm_profile_task_add_cmd_handler(ke_msg_id_t const msgid, struct gapm_profile_task_add_cmd *param, ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int gapm_set_irk_cmd_handler(ke_msg_id_t const msgid, struct gapm_set_irk_cmd *param, ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int gapm_lepsm_register_cmd_handler(ke_msg_id_t const msgid, struct gapm_lepsm_register_cmd *param, ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int gapm_lepsm_unregister_cmd_handler(ke_msg_id_t const msgid, struct gapm_lepsm_unregister_cmd *param, ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int gapm_le_test_mode_ctrl_cmd_handler(ke_msg_id_t const msgid, struct gapm_le_test_mode_ctrl_cmd *param, ke_task_id_t const dest_id, ke_task_id_t const src_id);

// Handler in gapm_list.c
extern int gapm_list_set_cmd_handler(ke_msg_id_t const msgid, struct gapm_list_set_cmd const *param, ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int gapm_get_ral_addr_cmd_handler(ke_msg_id_t const msgid, struct gapm_get_ral_addr_cmd *p_param, ke_task_id_t const dest_id, ke_task_id_t const src_id);

// Handler in gapm_actv.c
extern int gapm_activity_create_cmd_handler(ke_msg_id_t const msgid, struct gapm_activity_create_cmd const *param, ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int gapm_activity_start_cmd_handler(ke_msg_id_t const msgid, struct gapm_activity_start_cmd const *param, ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int gapm_activity_stop_cmd_handler(ke_msg_id_t const msgid, struct gapm_activity_stop_cmd const *param, ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int gapm_activity_delete_cmd_handler(ke_msg_id_t const msgid, struct gapm_activity_delete_cmd const *param, ke_task_id_t const dest_id, ke_task_id_t const src_id);

// Handler in gapm_addr.c
extern int gapm_cmp_evt_handler(ke_msg_id_t const msgid, struct gapm_cmp_evt const *cmp_evt, ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int gapm_addr_renew_cmd_handler(ke_msg_id_t const msgid, struct gapm_addr_renew_cmd *p_param, ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int gapm_addr_renew_to_ind_handler(ke_msg_id_t const msgid, void const *param, ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int gapm_dev_bdaddr_ind_handler(ke_msg_id_t const msgid, void const *param, ke_task_id_t const dest_id, ke_task_id_t const src_id);

// Handler in gapm_adv.c
extern int gapm_set_adv_data_cmd_handler(ke_msg_id_t const msgid, struct gapm_set_adv_data_cmd const *param, ke_task_id_t const dest_id, ke_task_id_t const src_id);

// Handler in gapm_init.c
extern int gapm_auto_conn_to_ind_handler(ke_msg_id_t const msgid, void const *param, ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int gapc_cmp_evt_handler(ke_msg_id_t const msgid, struct gapc_cmp_evt const *cmp_evt, ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int gapc_peer_att_info_ind_handler(ke_msg_id_t const msgid, struct gapc_peer_att_info_ind const *param, ke_task_id_t const dest_id, ke_task_id_t const src_id);

// Handler in gapm_smp.c
extern int gapm_resolv_addr_cmd_handler(ke_msg_id_t const msgid, struct gapm_resolv_addr_cmd *param, ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int gapm_gen_rand_addr_cmd_handler(ke_msg_id_t const msgid, struct gapm_gen_rand_addr_cmd *param, ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int gapm_use_enc_block_cmd_handler(ke_msg_id_t const msgid, struct gapm_use_enc_block_cmd *param, ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int gapm_gen_dh_key_cmd_handler(ke_msg_id_t const msgid, struct gapm_gen_dh_key_cmd *param, ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int gapm_get_pub_key_cmd_handler(ke_msg_id_t const msgid, struct gapm_get_pub_key_cmd *param, ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int gapm_aes_h6_cmd_handler(ke_msg_id_t const msgid, struct gapm_aes_h6_cmd const *param, ke_task_id_t const dest_id, ke_task_id_t const src_id);

// To distingush APP and AHI when both eixsted.
extern int gapm_set_default_requester(ke_msg_id_t const msgid, void const *param, ke_task_id_t const dest_id, ke_task_id_t const src_id);


/*
 * TASK VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// The default message handlers
KE_MSG_HANDLER_TAB(gapm)
{
    // note: first message is latest message checked by kernel so default is put on top.
    { KE_MSG_DEFAULT_HANDLER, (ke_msg_func_t) gapm_default_msg_handler },

    /* Reset command */
    { GAPM_RESET_CMD, (ke_msg_func_t) gapm_reset_cmd_handler },

    /* Device set configuration */
    { GAPM_SET_DEV_CONFIG_CMD, (ke_msg_func_t) gapm_set_dev_config_cmd_handler },
#ifndef GAPM_SLIM
    { GAPM_SET_CHANNEL_MAP_CMD, (ke_msg_func_t) gapm_set_channel_map_cmd_handler },
#endif

    /* Device get configuration */
    { GAPM_GET_DEV_INFO_CMD, (ke_msg_func_t) gapm_get_dev_info_cmd_handler },

    /* Address resolution */
    { GAPM_RESOLV_ADDR_CMD, (ke_msg_func_t) gapm_resolv_addr_cmd_handler },
    { GAPM_GEN_RAND_ADDR_CMD, (ke_msg_func_t) gapm_gen_rand_addr_cmd_handler },
    { GAPM_USE_ENC_BLOCK_CMD, (ke_msg_func_t) gapm_use_enc_block_cmd_handler },
#if (SECURE_CONNECTIONS)
    { GAPM_GEN_DH_KEY_CMD, (ke_msg_func_t) gapm_gen_dh_key_cmd_handler   },
    { GAPM_GET_PUB_KEY_CMD, (ke_msg_func_t) gapm_get_pub_key_cmd_handler   },
#endif // (SECURE_CONNECTIONS)
    { GAPM_GEN_RAND_NB_CMD, (ke_msg_func_t) gapm_use_enc_block_cmd_handler },

#if (BLE_CENTRAL)
    { GAPC_CMP_EVT, (ke_msg_func_t) gapc_cmp_evt_handler },
    { GAPC_PEER_ATT_INFO_IND, (ke_msg_func_t) gapc_peer_att_info_ind_handler },
#endif // (BLE_CENTRAL)

    /* Extended Air Operations */
    { GAPM_ACTIVITY_CREATE_CMD, (ke_msg_func_t) gapm_activity_create_cmd_handler },
    { GAPM_ACTIVITY_START_CMD, (ke_msg_func_t) gapm_activity_start_cmd_handler },
    { GAPM_ACTIVITY_STOP_CMD, (ke_msg_func_t) gapm_activity_stop_cmd_handler },
    { GAPM_ACTIVITY_DELETE_CMD, (ke_msg_func_t) gapm_activity_delete_cmd_handler },
#if (BLE_BROADCASTER)
    { GAPM_SET_ADV_DATA_CMD, (ke_msg_func_t) gapm_set_adv_data_cmd_handler },
#endif //(BLE_BROADCASTER)
#if (BLE_CENTRAL)
    { GAPM_AUTO_CONN_TO_IND, (ke_msg_func_t) gapm_auto_conn_to_ind_handler },
#endif //(BLE_CENTRAL)

    /* Address Management (Internal) */
    { GAPM_ADDR_RENEW_CMD, (ke_msg_func_t) gapm_addr_renew_cmd_handler },
    { GAPM_ADDR_RENEW_TO_IND, (ke_msg_func_t) gapm_addr_renew_to_ind_handler },
    { GAPM_DEV_BDADDR_IND, (ke_msg_func_t) gapm_dev_bdaddr_ind_handler },
    { GAPM_CMP_EVT, (ke_msg_func_t) gapm_cmp_evt_handler },

    /* List Management */
    { GAPM_LIST_SET_CMD, (ke_msg_func_t) gapm_list_set_cmd_handler },
    { GAPM_GET_RAL_ADDR_CMD, (ke_msg_func_t) gapm_get_ral_addr_cmd_handler },

    /* Profile Management */
#if BLE_PROFILES
    { GAPM_PROFILE_TASK_ADD_CMD, (ke_msg_func_t) gapm_profile_task_add_cmd_handler },
#endif // (BLE_PROFILES)

    { GAPM_UNKNOWN_TASK_MSG, (ke_msg_func_t) gapm_unknown_task_msg_handler },

    /* HCI management */
    { HCI_DBG_EVT, (ke_msg_func_t) gapm_hci_handler },
    { HCI_CMD_CMP_EVENT, (ke_msg_func_t) gapm_hci_handler },
    { HCI_CMD_STAT_EVENT, (ke_msg_func_t) gapm_hci_handler },
    { HCI_LE_EVENT, (ke_msg_func_t) gapm_hci_handler },

    /* Set IRK */
    { GAPM_SET_IRK_CMD, (ke_msg_func_t) gapm_set_irk_cmd_handler },

#ifndef GAPM_SLIM
    /* LE Credit Based Channel Management */
    { GAPM_LEPSM_REGISTER_CMD, (ke_msg_func_t) gapm_lepsm_register_cmd_handler },
    { GAPM_LEPSM_UNREGISTER_CMD, (ke_msg_func_t) gapm_lepsm_unregister_cmd_handler },

    /* Test Mode control */
    { GAPM_LE_TEST_MODE_CTRL_CMD, (ke_msg_func_t) gapm_le_test_mode_ctrl_cmd_handler },
#endif  // !GAPM_SLIM
    {GAPM_AES_H6_CMD, (ke_msg_func_t)gapm_aes_h6_cmd_handler},
    { GAPM_SET_DEFAULT_REQUESTER, (ke_msg_func_t) gapm_set_default_requester },
};

/// GATT task instance.
ke_state_t gapm_state[GAPM_IDX_MAX];

/// GAP Manager task descriptor
const struct ke_task_desc TASK_DESC_GAPM = {gapm_msg_handler_tab, gapm_state, GAPM_IDX_MAX, ARRAY_LEN(gapm_msg_handler_tab)};

/// @} GAPM_TASK
