/**
 ****************************************************************************************
 *
 * @file llc_task.c
 *
 * @brief LLC task source file
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup LLCTASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"    // stack configuration

#include <string.h>
#include "co_utils.h"
#include "co_math.h"
#include "co_bt.h"          // BLE standard definitions
#include "ke_task.h"        // Kernel task
#include "ke_timer.h"       // Kernel timer

#include "llc_int.h"        // link layer controller internal definitions
#include "llc_llcp.h"       // link layer controller protocol PDU definitions
#include "llc.h"            // link layer controller definitions

#include "lld.h"            // link layer driver definitions
#include "llm.h"            // Link layer manager API

#include "sch_plan.h"       // SCH_PLAN API

#if HCI_PRESENT
    #include "hci.h"            // host controller interface
#endif //HCI_PRESENT

#include "aes.h"            // For encryption result

#if (BLE_PERIPHERAL || BLE_CENTRAL)

/*
 * LOCAL DEFINE
 ****************************************************************************************
 */
//Minimum instant needed by the procedures
#define MIN_INSTANT     (6)
/*
 * LOCAL FUNCTIONS DEFINITION
 ****************************************************************************************
 */

/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */


// handlers in llc_hci.c
extern KE_MSG_HANDLER_NO_STATIC(hci_command_llc, void);
extern KE_MSG_HANDLER_NO_STATIC(hci_acl_data, struct hci_acl_data);
extern KE_MSG_HANDLER_NO_STATIC(lld_acl_rx_ind, struct lld_acl_rx_ind);
extern KE_MSG_HANDLER_NO_STATIC(lld_acl_tx_cfm, void);

// handlers in llc_llcp.c
extern KE_MSG_HANDLER_NO_STATIC(lld_llcp_rx_ind, struct lld_llcp_rx_ind);
extern KE_MSG_HANDLER_NO_STATIC(lld_llcp_tx_cfm, void);
// handlers in llc_disconnect.c
extern KE_MSG_HANDLER_NO_STATIC(lld_disc_ind, struct lld_disc_ind);

// handlers in llc_ver_exch.c
struct llc_op_ver_exch_ind;
extern KE_MSG_HANDLER_NO_STATIC(llc_op_ver_exch_ind, struct llc_op_ver_exch_ind);

// handlers in llc_feat_exch.c
struct llc_op_feats_exch_ind;
extern KE_MSG_HANDLER_NO_STATIC(llc_op_feats_exch_ind, struct llc_op_feats_exch_ind);

// handlers in llc_disconnect.c
struct llc_op_disconnect_ind;
extern KE_MSG_HANDLER_NO_STATIC(llc_op_disconnect_ind, struct llc_op_disconnect_ind);
extern KE_MSG_HANDLER_NO_STATIC(llc_stopped_ind, void);

// handlers in llc_encypt.c
struct llc_op_encrypt_ind;
extern KE_MSG_HANDLER_NO_STATIC(llc_op_encrypt_ind, struct llc_op_encrypt_ind);
extern KE_MSG_HANDLER_NO_STATIC(llc_encrypt_ind, struct llc_encrypt_ind);

// handlers in llc_dl_upd.c
struct llc_op_dl_upd_ind;
extern KE_MSG_HANDLER_NO_STATIC(llc_op_dl_upd_ind, struct llc_op_dl_upd_ind);

// handlers in llc_con_upd.c
struct llc_op_con_upd_ind;
extern KE_MSG_HANDLER_NO_STATIC(llc_op_con_upd_ind, struct llc_op_con_upd_ind);
extern KE_MSG_HANDLER_NO_STATIC(lld_con_param_upd_cfm, void);
extern KE_MSG_HANDLER_NO_STATIC(lld_con_offset_upd_ind, struct lld_con_offset_upd_ind);

#if (BLE_CIS)
    // handlers in llc_cis.c
    struct llc_op_cis_create_ind;
    struct llc_op_cis_stop_ind;
    extern KE_MSG_HANDLER_NO_STATIC(llc_op_cis_create_ind, struct llc_op_cis_create_ind);
    extern KE_MSG_HANDLER_NO_STATIC(llc_op_cis_stop_ind, struct llc_op_cis_stop_ind);
    #if (BLE_PERIPHERAL)
        extern KE_MSG_HANDLER_NO_STATIC(llc_cis_accept_to, void);
    #endif // (BLE_PERIPHERAL)
#endif // (BLE_CIS)

// handlers in llc_le_ping.c
struct llc_op_le_ping_ind;
extern KE_MSG_HANDLER_NO_STATIC(llc_op_le_ping_ind, struct llc_op_le_ping_ind);
extern KE_MSG_HANDLER_NO_STATIC(llc_auth_payl_nearly_to, void);
extern KE_MSG_HANDLER_NO_STATIC(llc_auth_payl_real_to, void);

// handlers in llc_ch_map_upd.c
struct llc_op_ch_map_upd_ind;
extern KE_MSG_HANDLER_NO_STATIC(llc_op_ch_map_upd_ind, struct llc_op_ch_map_upd_ind);
extern KE_MSG_HANDLER_NO_STATIC(lld_ch_map_upd_cfm, void);

// handlers in llc_phy_upd.c
struct llc_op_phy_upd_ind;
extern KE_MSG_HANDLER_NO_STATIC(llc_op_phy_upd_ind, struct llc_op_phy_upd_ind);
extern KE_MSG_HANDLER_NO_STATIC(lld_phy_upd_cfm, void);

// handlers in llc_cte.c
#if BLE_CON_CTE_REQ
    extern KE_MSG_HANDLER_NO_STATIC(lld_con_cte_rx_ind, struct lld_con_cte_rx_ind);
    struct llc_op_cte_ind;
    extern KE_MSG_HANDLER_NO_STATIC(llc_op_cte_ind, struct llc_op_cte_ind);
    extern KE_MSG_HANDLER_NO_STATIC(llc_cte_req_to, void);
#endif // BLE_CON_CTE_REQ

// handlers in llc_clk_acc.c
struct llc_op_clk_acc_ind;
extern KE_MSG_HANDLER_NO_STATIC(llc_op_clk_acc_ind, struct llc_op_clk_acc_ind);

#if BLE_PAST
    // handlers in llc_past.c
    struct llc_op_past_ind;
    extern KE_MSG_HANDLER_NO_STATIC(llc_op_past_ind, struct llc_op_past_ind);
#endif // BLE_PAST

#if BLE_PWR_CTRL
    // handlers in llc_pwr.c
    struct llc_op_pwr_ctrl_ind;
    extern KE_MSG_HANDLER_NO_STATIC(llc_op_pwr_ctrl_ind, struct llc_op_pwr_ctrl_ind);
    extern KE_MSG_HANDLER_NO_STATIC(lld_con_pwr_ctrl_ind, struct lld_con_pwr_ctrl_ind);
    extern KE_MSG_HANDLER_NO_STATIC(lld_con_path_loss_change_ind, struct lld_con_path_loss_change_ind);
    extern KE_MSG_HANDLER_NO_STATIC(lld_con_pwr_change_ind, struct lld_con_pwr_change_ind);
    extern KE_MSG_HANDLER_NO_STATIC(lld_cis_pwr_change_ind, struct lld_cis_pwr_change_ind);
#endif // BLE_PWR_CTRL

#if (BT_53)
    // handlers in llc_subrate.c
    struct llc_op_subrate_ind;
    extern KE_MSG_HANDLER_NO_STATIC(llc_op_subrate_ind, struct llc_op_subrate_ind);

    // handlers in llc_ch_class.c
    #if (BLE_CENTRAL)
        struct llc_op_ch_class_en_ind;
        extern KE_MSG_HANDLER_NO_STATIC(llc_op_ch_class_en_ind, struct llc_op_ch_class_en_ind);
    #endif // (BLE_CENTRAL)
    #if (BLE_PERIPHERAL)
        struct llc_op_ch_class_rep_ind;
        extern KE_MSG_HANDLER_NO_STATIC(llc_op_ch_class_rep_ind, struct llc_op_ch_class_rep_ind);
        extern KE_MSG_HANDLER_NO_STATIC(llc_ch_class_to, void);
    #endif // (BLE_PERIPHERAL)
#endif // (BT_53)

// handler in llc_dbg.c
#if (RW_DEBUG)
    extern KE_MSG_HANDLER_NO_STATIC(lld_con_evt_trace_ind, struct lld_con_evt_trace_ind);
#endif // (RW_DEBUG)
/*
 * HCI MESSAGE HANDLING
 ****************************************************************************************
 */
/**
 ****************************************************************************************
 * @brief Handles the Local LLCP transaction timeout indication message.
 ****************************************************************************************
 */
KE_MSG_HANDLER(llc_loc_llcp_rsp_to, void)
{
    uint8_t link_id = KE_IDX_GET(dest_id);

    // Disconnect link due to transaction timeout
    llc_disconnect(link_id, CO_ERROR_LMP_RSP_TIMEOUT, true);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the Remote LLCP transaction timeout indication message.
 ****************************************************************************************
 */
KE_MSG_HANDLER(llc_rem_llcp_rsp_to, void)
{
    uint8_t link_id = KE_IDX_GET(dest_id);

    // Disconnect link due to transaction timeout
    llc_disconnect(link_id, CO_ERROR_LMP_RSP_TIMEOUT, true);

    return (KE_MSG_CONSUMED);
}

/*
 * TASK DESCRIPTOR DEFINITIONS
 ****************************************************************************************
 */

/// Specifies the default message handlers
KE_MSG_HANDLER_TAB(llc)
{
    // Note: all messages must be sorted in ID ascending order

    // Timer handlers
    {LLC_LOC_LLCP_RSP_TO, (ke_msg_func_t)llc_loc_llcp_rsp_to_handler},
    {LLC_REM_LLCP_RSP_TO, (ke_msg_func_t)llc_rem_llcp_rsp_to_handler},

    {LLC_AUTH_PAYL_REAL_TO, (ke_msg_func_t)llc_auth_payl_real_to_handler},
    {LLC_AUTH_PAYL_NEARLY_TO, (ke_msg_func_t)llc_auth_payl_nearly_to_handler},

    // Link stopped, clean-up environment
    {LLC_STOPPED_IND, (ke_msg_func_t)llc_stopped_ind_handler},
    {LLC_ENCRYPT_IND, (ke_msg_func_t)llc_encrypt_ind_handler},

#if (BLE_CON_CTE_REQ)
    // Connection CTE request timeout
    {LLC_CTE_REQ_TO, (ke_msg_func_t)llc_cte_req_to_handler},
#endif // (BLE_CON_CTE_REQ)
#if (BLE_CIS) && (BLE_PERIPHERAL)
    // CIS connection accept timeout
    {LLC_CIS_ACCEPT_TO, (ke_msg_func_t)llc_cis_accept_to_handler},
#endif // (BLE_CIS) && (BLE_PERIPHERAL)
#if (BT_53)
#if (BLE_PERIPHERAL)
    // Channel classification timer
    {LLC_CH_CLASS_TO, (ke_msg_func_t)llc_ch_class_to_handler},
#endif // (BLE_PERIPHERAL)
#endif // (BT_53)

    /*
     * ************** Msg LLC->LLC ****************
     */
    // Version exchange operation
    {LLC_OP_VER_EXCH_IND, (ke_msg_func_t)llc_op_ver_exch_ind_handler},
    // Features exchange operation
    {LLC_OP_FEATS_EXCH_IND, (ke_msg_func_t)llc_op_feats_exch_ind_handler},
    // Disconnect operation
    {LLC_OP_DISCONNECT_IND, (ke_msg_func_t)llc_op_disconnect_ind_handler},
    // Encryption operation
    {LLC_OP_ENCRYPT_IND, (ke_msg_func_t)llc_op_encrypt_ind_handler},
    // Data length updatate operation
    {LLC_OP_DL_UPD_IND, (ke_msg_func_t)llc_op_dl_upd_ind_handler},
    // Connection and connection parameters request operation
    {LLC_OP_CON_UPD_IND, (ke_msg_func_t)llc_op_con_upd_ind_handler},
    // LE Ping operation
    {LLC_OP_LE_PING_IND, (ke_msg_func_t)llc_op_le_ping_ind_handler},
    // Channel map update operation
    {LLC_OP_CH_MAP_UPD_IND, (ke_msg_func_t)llc_op_ch_map_upd_ind_handler},
    // PHY Update operation
    {LLC_OP_PHY_UPD_IND, (ke_msg_func_t)llc_op_phy_upd_ind_handler},
#if BLE_CON_CTE_REQ
    // Connection CTE operation
    {LLC_OP_CTE_IND, (ke_msg_func_t)llc_op_cte_ind_handler},
#endif // BLE_CON_CTE_REQ
    // Clock accuracy operation
    {LLC_OP_CLK_ACC_IND, (ke_msg_func_t)llc_op_clk_acc_ind_handler},
#if (BLE_OBSERVER && BLE_PAST)
    // Periodic Advertising Sync Transfer operation
    {LLC_OP_PAST_IND, (ke_msg_func_t)llc_op_past_ind_handler},
#endif //(BLE_OBSERVER)
#if (BLE_CIS)
#if (BLE_CENTRAL)
    // CIS Channel creation operation
    {LLC_OP_CIS_CREATE_IND, (ke_msg_func_t)llc_op_cis_create_ind_handler},
#endif // (BLE_CENTRAL)
    // CIS Channel stop operation
    {LLC_OP_CIS_STOP_IND, (ke_msg_func_t)llc_op_cis_stop_ind_handler},
#endif // (BLE_CIS)
#if BLE_PWR_CTRL
    // Power control operation
    {LLC_OP_PWR_CTRL_IND, (ke_msg_func_t)llc_op_pwr_ctrl_ind_handler},
#endif // BLE_PWR_CTRL
#if (BT_53)
    // Power control operation
    {LLC_OP_SUBRATE_IND, (ke_msg_func_t)llc_op_subrate_ind_handler},
#if (BLE_CENTRAL)
    // Channel classification enable operation
    {LLC_OP_LE_CH_CLASS_EN_IND, (ke_msg_func_t)llc_op_ch_class_en_ind_handler},
#endif // (BLE_CENTRAL)
#if (BLE_PERIPHERAL)
    // Channel classification reporting operation
    {LLC_OP_CH_CLASS_REP_IND, (ke_msg_func_t)llc_op_ch_class_rep_ind_handler},
#endif // (BLE_PERIPHERAL)
#endif // (BT_53)

    /*
     * ************** Msg LLD->LLC ****************
     */
    // LLCP Reception from peer device
    {LLD_LLCP_RX_IND, (ke_msg_func_t)lld_llcp_rx_ind_handler},
    // LLCP Acknowledgment reception
    {LLD_LLCP_TX_CFM, (ke_msg_func_t)lld_llcp_tx_cfm_handler},
    // ACL Reception from peer device
    {LLD_ACL_RX_IND, (ke_msg_func_t)lld_acl_rx_ind_handler},
    // ACL Data Acknowledgment reception
    {LLD_ACL_TX_CFM, (ke_msg_func_t)lld_acl_tx_cfm_handler},
    {LLD_CON_PARAM_UPD_CFM, (ke_msg_func_t)lld_con_param_upd_cfm_handler},
    {LLD_CH_MAP_UPD_CFM, (ke_msg_func_t)lld_ch_map_upd_cfm_handler},
    {LLD_PHY_UPD_CFM, (ke_msg_func_t)lld_phy_upd_cfm_handler},
    // Link disconnection handler
    {LLD_DISC_IND, (ke_msg_func_t)lld_disc_ind_handler},
    {LLD_CON_OFFSET_UPD_IND, (ke_msg_func_t)lld_con_offset_upd_ind_handler},

#if BLE_PWR_CTRL
    {LLD_CON_PWR_CTRL_IND, (ke_msg_func_t)lld_con_pwr_ctrl_ind_handler},
    {LLD_CON_PATH_LOSS_CHANGE_IND, (ke_msg_func_t)lld_con_path_loss_change_ind_handler},
    {LLD_CON_PWR_CHANGE_IND, (ke_msg_func_t)lld_con_pwr_change_ind_handler},
    {LLD_CIS_PWR_CHANGE_IND, (ke_msg_func_t)lld_cis_pwr_change_ind_handler},
#endif // BLE_PWR_CTRL

#if BLE_CON_CTE_REQ
    // Connection CTE Rx indication
    {LLD_CON_CTE_RX_IND, (ke_msg_func_t)lld_con_cte_rx_ind_handler},
#endif // BLE_CON_CTE_REQ

#if (RW_DEBUG)
    {LLD_CON_EVT_TRACE_IND, (ke_msg_func_t)lld_con_evt_trace_ind_handler},
#endif // (RW_DEBUG)

    /*
     * ************** Msg HCI->LLC ****************
     */
    // HCI command management
    {HCI_COMMAND, (ke_msg_func_t)hci_command_llc_handler},
    // Data sent to the peer
    {HCI_ACL_DATA, (ke_msg_func_t)hci_acl_data_handler},
};

/// Defines the place holder for the states of all the task instances.
__STATIC ke_state_t llc_state[LLC_IDX_MAX];

/// LLC task descriptor
const struct ke_task_desc TASK_DESC_LLC = {llc_msg_handler_tab, llc_state, LLC_IDX_MAX, ARRAY_LEN(llc_msg_handler_tab)};


#endif //(BLE_PERIPHERAL || BLE_CENTRAL)

/// @} LLCTASK
