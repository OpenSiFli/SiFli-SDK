/**
 ****************************************************************************************
 *
 * @file llc_llcp.c
 *
 * @brief Implementation of the functions for control pdu transmission/reception
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup LLCLLCP
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"    // stack configuration
#if (BLE_PERIPHERAL || BLE_CENTRAL)

#include <string.h>
#include "co_utils.h"
#include "co_math.h"
#include "co_bt.h"          // BLE standard definitions
#include "co_version.h"     // BLE version definitions

#include "ke_task.h"        // Kernel task
#include "ke_mem.h"         // Kernel memory

#include "llc.h"            // link layer controller definitions
#include "llc_int.h"        // link layer controller internal definitions
#include "llc_llcp.h"       // link layer controller protocol PDU definitions

#include "lld.h"            // link layer driver

#include "reg_access.h"     // Read/Write exchange memory

#include "dbg.h"

/*
 * MACROS
 ****************************************************************************************
 */
/// retrieve message from pdu in handler
#define LLC_GET_BASE_MSG(pdu) \
    (struct llc_llcp_recv_ind*) (((int32_t) pdu) - (sizeof(struct llc_llcp_recv_ind) - sizeof(union llcp_pdu)))

/// Handler definition of LLCP packet
#define HANDLER(name, hdl, pack, perm) \
    [name##_OPCODE]   = { (llc_llcp_pdu_handler_func_t) hdl##_handler, pack, name##_LEN, perm }


/// llcp permission
#define PERM(val)  (LLC_LLCP_##val##_AUTHZED)
/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */



/// Status to check if a llcp can be sent or not during start en pause encryption
enum llc_llcp_authorize
{
    /// LLCP PDU authorized only in normal mode
    LLC_LLCP_IDLE_AUTHZED       = (1 << LLC_LLCP_IDLE),
    /// LLCP PDU authorized during start encryption procedure
    LLC_LLCP_START_ENC_AUTHZED  = (1 << LLC_LLCP_PAUSE_ENC),
    /// LLCP PDU authorized during Pause encryption procedure
    LLC_LLCP_PAUSE_ENC_AUTHZED  = (1 << LLC_LLCP_START_ENC),
    /// LLCP PDU authorized in link termination procedure
    LLC_LLCP_TERMINATE_AUTHZED  = (1 << LLC_LLCP_TERMINATE),
};


/// LLCP PDU list element
typedef struct
{
    /// list element header
    struct co_list_hdr hdr;
    /// LLCP TX Confirm callback
    llc_llcp_tx_cfm_cb tx_cfm_cb;
    /// PDU length
    uint8_t length;
    /// PDU data
    uint8_t  pdu[__ARRAY_EMPTY];
} llcp_pdu_t;


/// LLCP PDU handler information for packing/unpacking and function handler
struct llcp_pdu_handler_info
{
    /// Message handler
    llc_llcp_pdu_handler_func_t handler;
    /// Pack/Unpack format string
    const char                 *pack_format;
    /// Length of LLCP PDU
    uint16_t                    length;
    /// Process authorization
    uint8_t                     authz;
};



/*
 * LLCP HANDLER FUNCTION DECLARATION
 ****************************************************************************************
 */

// handlers in llc_disconnect.c
extern uint8_t ll_terminate_ind_handler(uint8_t link_id, struct ll_terminate_ind *pdu, uint16_t event_cnt);

// handlers in llc_ver_exch.c
extern uint8_t ll_version_ind_handler(uint8_t link_id, struct ll_version_ind *pdu, uint16_t event_cnt);

// handlers in llc_encrypt.c
extern uint8_t ll_pause_enc_req_handler(uint8_t link_id, struct ll_pause_enc_req *pdu, uint16_t event_cnt);
extern uint8_t ll_pause_enc_rsp_handler(uint8_t link_id, struct ll_pause_enc_rsp *pdu, uint16_t event_cnt);
extern uint8_t ll_enc_req_handler(uint8_t link_id, struct ll_enc_req *pdu, uint16_t event_cnt);
extern uint8_t ll_enc_rsp_handler(uint8_t link_id, struct ll_enc_rsp *pdu, uint16_t event_cnt);
extern uint8_t ll_start_enc_req_handler(uint8_t link_id, struct ll_start_enc_req *pdu, uint16_t event_cnt);
extern uint8_t ll_start_enc_rsp_handler(uint8_t link_id, struct ll_start_enc_rsp *pdu, uint16_t event_cnt);

// handlers in llc_feat_exch.c
extern uint8_t ll_feature_req_handler(uint8_t link_id, struct ll_feature_req *pdu, uint16_t event_cnt);
extern uint8_t ll_slave_feature_req_handler(uint8_t link_id, struct ll_slave_feature_req *pdu, uint16_t event_cnt);
extern uint8_t ll_feature_rsp_handler(uint8_t link_id, struct ll_feature_rsp *pdu, uint16_t event_cnt);

// handlers in llc_con_upd.c
extern uint8_t ll_connection_param_req_handler(uint8_t link_id, struct ll_connection_param_req *pdu, uint16_t event_cnt);
extern uint8_t ll_connection_param_rsp_handler(uint8_t link_id, struct ll_connection_param_rsp *pdu, uint16_t event_cnt);
extern uint8_t ll_connection_update_ind_handler(uint8_t link_id, struct ll_connection_update_ind *pdu, uint16_t event_cnt);

// handlers in llc_le_ping.c
extern int ll_ping_req_handler(uint8_t link_id, struct ll_enc_rsp *pdu, uint16_t event_cnt);
extern int ll_ping_rsp_handler(uint8_t link_id, struct ll_enc_rsp *pdu, uint16_t event_cnt);

// handlers in llc_ch_map_upd.c
extern int ll_channel_map_ind_handler(uint8_t link_id, struct ll_channel_map_ind *pdu, uint16_t event_cnt);
extern int ll_min_used_channels_ind_handler(uint8_t link_id, struct ll_min_used_channels_ind *pdu, uint16_t event_cnt);

// handlers in llc_phy_upd.c
extern int ll_phy_req_handler(uint8_t link_id, struct ll_phy_req *pdu, uint16_t event_cnt);
extern int ll_phy_rsp_handler(uint8_t link_id, struct ll_phy_rsp *pdu, uint16_t event_cnt);
extern int ll_phy_update_ind_handler(uint8_t link_id, struct ll_phy_update_ind *pdu, uint16_t event_cnt);

// handlers in llc_dl_upd.c
extern int ll_length_req_handler(uint8_t link_id, struct ll_length_req *pdu, uint16_t event_cnt);
extern int ll_length_rsp_handler(uint8_t link_id, struct ll_length_rsp *pdu, uint16_t event_cnt);

// handlers in llc_cte.c
#if BLE_CON_CTE_REQ
    extern int ll_cte_req_handler(uint8_t link_id, struct ll_cte_req *pdu, uint16_t event_cnt);
#endif // BLE_CON_CTE_REQ
#if BLE_CON_CTE_RSP
    extern int ll_cte_rsp_handler(uint8_t link_id, struct ll_default *pdu, uint16_t event_cnt);
#endif // BLE_CON_CTE_RSP

#if BLE_PAST
    // handlers in llc_past.c
    extern int ll_per_sync_ind_handler(uint8_t link_id, struct ll_per_sync_ind *pdu, uint16_t event_cnt);
#endif // BLE_PAST

// handlers in llc_clk_acc.c
extern int ll_clk_acc_req_handler(uint8_t link_id, struct ll_clk_acc_req *pdu, uint16_t event_cnt);
extern int ll_clk_acc_rsp_handler(uint8_t link_id, struct ll_clk_acc_rsp *pdu, uint16_t event_cnt);


#if (BLE_CIS)
    // handlers in llc_cis.c
    #if (BLE_PERIPHERAL)
        extern int ll_cis_req_handler(uint8_t link_id, struct ll_cis_req *pdu, uint16_t event_cnt);
        extern int ll_cis_ind_handler(uint8_t link_id, struct ll_cis_ind *pdu, uint16_t event_cnt);
    #endif // (BLE_PERIPHERAL)
    #if (BLE_CENTRAL)
        extern int ll_cis_rsp_handler(uint8_t link_id, struct ll_cis_rsp *pdu, uint16_t event_cnt);
    #endif // (BLE_CENTRAL)
    extern int ll_cis_terminate_ind_handler(uint8_t link_id, struct ll_cis_terminate_ind *pdu, uint16_t event_cnt);
#endif // (BLE_CIS)

#if BLE_PWR_CTRL
    // handlers in llc_pwr.c
    extern int ll_pwr_ctrl_req_handler(uint8_t link_id, struct ll_phy_req *pdu, uint16_t event_cnt);
    extern int ll_pwr_ctrl_rsp_handler(uint8_t link_id, struct ll_phy_rsp *pdu, uint16_t event_cnt);
    extern int ll_pwr_change_ind_handler(uint8_t link_id, struct ll_phy_update_ind *pdu, uint16_t event_cnt);
#endif // BLE_PWR_CTRL

#if (BT_53)
    // hanlers in llc_subrate.c
    #if (BLE_CENTRAL)
        extern int ll_subrate_req_handler(uint8_t link_id, struct ll_subrate_req *pdu, uint16_t event_cnt);
    #endif // (BLE_CENTRAL)
    #if (BLE_PERIPHERAL)
        extern int ll_subrate_ind_handler(uint8_t link_id, struct ll_subrate_ind *pdu, uint16_t event_cnt);
    #endif // (BLE_PERIPHERAL)

    // handlers in llc_ch_class.c
    #if (BLE_PERIPHERAL)
        extern int ll_ch_rep_ind_handler(uint8_t link_id, struct ll_ch_rep_ind *pdu, uint16_t event_cnt);
    #endif // (BLE_PERIPHERAL)
    #if (BLE_CENTRAL)
        extern int ll_ch_status_ind_handler(uint8_t link_id, struct ll_ch_status_ind *pdu, uint16_t event_cnt);
    #endif // (BLE_CENTRAL)
#endif // (BT_53)

__STATIC uint8_t ll_reject_ind_handler(uint8_t link_id, struct ll_reject_ind *pdu, uint16_t event_cnt);
__STATIC uint8_t ll_reject_ext_ind_handler(uint8_t link_id, struct ll_reject_ext_ind *pdu, uint16_t event_cnt);
__STATIC uint8_t ll_unknown_rsp_handler(uint8_t link_id, struct ll_unknown_rsp *pdu, uint16_t event_cnt);

__STATIC void llc_ll_unknown_rsp_pdu_send(uint8_t link_id, uint8_t op_code);
/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

///Table for normal LMP PDUs handler utilities
__STATIC const struct llcp_pdu_handler_info llcp_pdu_handler[LL_OPCODE_MAX_OPCODE] =
{
    HANDLER(LL_CONNECTION_UPDATE_IND, ll_connection_update_ind, "BBHHHHH",                PERM(IDLE)),
    HANDLER(LL_CHANNEL_MAP_IND,       ll_channel_map_ind,       "B5BH",                   PERM(IDLE)),
    HANDLER(LL_TERMINATE_IND,         ll_terminate_ind,         "BB",                     PERM(IDLE) | PERM(START_ENC) | PERM(PAUSE_ENC) | PERM(TERMINATE)),
    HANDLER(LL_ENC_REQ,               ll_enc_req,               "B8BH8B4B",               PERM(IDLE) | PERM(PAUSE_ENC) | PERM(START_ENC)),
    HANDLER(LL_ENC_RSP,               ll_enc_rsp,               "B8B4B",                  PERM(IDLE) | PERM(START_ENC)),
    HANDLER(LL_START_ENC_REQ,         ll_start_enc_req,         "B",                      PERM(IDLE) | PERM(START_ENC)),
    HANDLER(LL_START_ENC_RSP,         ll_start_enc_rsp,         "B",                      PERM(IDLE) | PERM(START_ENC) | PERM(TERMINATE)),
    HANDLER(LL_UNKNOWN_RSP,           ll_unknown_rsp,           "BB",                     PERM(IDLE)),
    HANDLER(LL_FEATURE_REQ,           ll_feature_req,           "B8B",                    PERM(IDLE)),
    HANDLER(LL_FEATURE_RSP,           ll_feature_rsp,           "B8B",                    PERM(IDLE)),
    HANDLER(LL_PAUSE_ENC_REQ,         ll_pause_enc_req,         "B",                      PERM(IDLE)),
    HANDLER(LL_PAUSE_ENC_RSP,         ll_pause_enc_rsp,         "B",                      PERM(IDLE) | PERM(PAUSE_ENC)),
    HANDLER(LL_VERSION_IND,           ll_version_ind,           "BBHH",                   PERM(IDLE)),
    HANDLER(LL_REJECT_IND,            ll_reject_ind,            "BB",                     PERM(IDLE) | PERM(START_ENC)),
    HANDLER(LL_SLAVE_FEATURE_REQ,     ll_slave_feature_req,     "B8B",                    PERM(IDLE)),
    HANDLER(LL_CONNECTION_PARAM_REQ,  ll_connection_param_req,  "BHHHHBHHHHHHH",          PERM(IDLE)),
    HANDLER(LL_CONNECTION_PARAM_RSP,  ll_connection_param_rsp,  "BHHHHBHHHHHHH",          PERM(IDLE)),
    HANDLER(LL_REJECT_EXT_IND,        ll_reject_ext_ind,        "BBB",                    PERM(IDLE) | PERM(START_ENC)),
    HANDLER(LL_PING_REQ,              ll_ping_req,              "B",                      PERM(IDLE)),
    HANDLER(LL_PING_RSP,              ll_ping_rsp,              "B",                      PERM(IDLE)),
    HANDLER(LL_LENGTH_REQ,            ll_length_req,            "BHHHH",                  PERM(IDLE)),
    HANDLER(LL_LENGTH_RSP,            ll_length_rsp,            "BHHHH",                  PERM(IDLE)),
    HANDLER(LL_PHY_REQ,               ll_phy_req,               "BBB",                    PERM(IDLE)),
    HANDLER(LL_PHY_RSP,               ll_phy_rsp,               "BBB",                    PERM(IDLE)),
    HANDLER(LL_PHY_UPDATE_IND,        ll_phy_update_ind,        "BBBH",                   PERM(IDLE)),
    HANDLER(LL_MIN_USED_CHANNELS_IND, ll_min_used_channels_ind, "BBB",                    PERM(IDLE)),
#if BLE_CON_CTE_REQ
    HANDLER(LL_CTE_REQ,               ll_cte_req,               "BB",                     PERM(IDLE)),
#endif // BLE_CON_CTE_REQ
#if BLE_CON_CTE_RSP
    HANDLER(LL_CTE_RSP,               ll_cte_rsp,               "B",                      PERM(IDLE)),
#endif // BLE_CON_CTE_RSP
#if BLE_PAST
    HANDLER(LL_PER_SYNC_IND,          ll_per_sync_ind,          "BH18BHHBB6BH",           PERM(IDLE)),
#endif // BLE_PAST
    HANDLER(LL_CLK_ACC_REQ,           ll_clk_acc_req,           "BB",                     PERM(IDLE)),
    HANDLER(LL_CLK_ACC_RSP,           ll_clk_acc_rsp,           "BB",                     PERM(IDLE)),
#if (BLE_CIS)
#if (BLE_PERIPHERAL)
    HANDLER(LL_CIS_REQ,               ll_cis_req,               "BBBBBHHDDHHBDBBBHDDH",   PERM(IDLE)),
    HANDLER(LL_CIS_IND,               ll_cis_ind,               "BLDDDH",                 PERM(IDLE)),
#endif // (BLE_PERIPHERAL)
#if (BLE_CENTRAL)
    HANDLER(LL_CIS_RSP,               ll_cis_rsp,               "BDDH",                   PERM(IDLE)),
#endif // (BLE_CENTRAL)
    HANDLER(LL_CIS_TERMINATE_IND,     ll_cis_terminate_ind,     "BBBB",                   PERM(IDLE)),
#endif // (BLE_CIS)
#if BLE_PWR_CTRL
    HANDLER(LL_PWR_CTRL_REQ,          ll_pwr_ctrl_req,          "BBBB",                   PERM(IDLE)),
    HANDLER(LL_PWR_CTRL_RSP,          ll_pwr_ctrl_rsp,          "BBBBB",                  PERM(IDLE)),
    HANDLER(LL_PWR_CHANGE_IND,        ll_pwr_change_ind,        "BBBBB",                  PERM(IDLE)),
#endif // BLE_PWR_CTRL
#if (BT_53)
#if (BLE_CENTRAL)
    HANDLER(LL_SUBRATE_REQ,          ll_subrate_req,            "BHHHHH",                 PERM(IDLE)),
#endif // (BLE_CENTRAL)
#if (BLE_PERIPHERAL)
    HANDLER(LL_SUBRATE_IND,          ll_subrate_ind,            "BHHHHH",                 PERM(IDLE)),
    HANDLER(LL_CH_REP_IND,            ll_ch_rep_ind,            "BBBB",                   PERM(IDLE)),
#endif // (BLE_PERIPHERAL)
#if (BLE_CENTRAL)
    HANDLER(LL_CH_STATUS_IND,         ll_ch_status_ind,         "B10B",                   PERM(IDLE)),
#endif // (BLE_CENTRAL)
#endif // (BT_53)
};



/**
 ****************************************************************************************
 * @brief Handle a badly formated or an unknown LLCP message
 *
 * @param[in] link_id        Link identifier on which the pdu will be sent.
 * @param[in] op_code        LLCP PDU Operation code received
 * @param[in] event_cnt      Event counter value when PDU has been received
 *
 * @return status code of handler:
 *    - CO_ERROR_NO_ERROR:               Nothing more to do
 *    - CO_ERROR_TERMINATED_MIC_FAILURE: Immediately disconnect the link
 *    - others:                          Send an LLCP_REJECT_IND or LLCP_REJECT_IND_EXT
 ****************************************************************************************
 */
__STATIC uint8_t llc_ll_unknown_ind_handler(uint8_t link_id, uint8_t opcode, uint16_t event_cnt)
{
    // Reject the receive request
    llc_ll_unknown_rsp_pdu_send(link_id, opcode);

    return (CO_ERROR_NO_ERROR);
}


/**
 ****************************************************************************************
 * @brief Handles the reception of a LLCP unknown response
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
__STATIC uint8_t ll_reject_ind_handler(uint8_t link_id, struct ll_reject_ind *pdu, uint16_t event_cnt)
{
    // request local operation to handle command rejection
    llc_proc_err_ind(link_id, LLC_PROC_LOCAL,  LLC_ERR_LLCP_REJECT_IND, pdu);
    llc_proc_err_ind(link_id, LLC_PROC_REMOTE, LLC_ERR_LLCP_REJECT_IND, pdu);

    return (CO_ERROR_NO_ERROR);
}

/**
 ****************************************************************************************
 * @brief Handles the reception of a LLCP unknown response
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
__STATIC uint8_t ll_reject_ext_ind_handler(uint8_t link_id, struct ll_reject_ext_ind *pdu, uint16_t event_cnt)
{
    // request local and remote operation to handle command rejection
    llc_proc_err_ind(link_id, LLC_PROC_LOCAL,  LLC_ERR_LLCP_REJECT_EXT_IND, pdu);
    llc_proc_err_ind(link_id, LLC_PROC_REMOTE, LLC_ERR_LLCP_REJECT_EXT_IND, pdu);

    return (CO_ERROR_NO_ERROR);
}

/**
 ****************************************************************************************
 * @brief Handles the reception of a LLCP unknown response
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
__STATIC uint8_t ll_unknown_rsp_handler(uint8_t link_id, struct ll_unknown_rsp *pdu, uint16_t event_cnt)
{
    // request local operation to handle command rejection
    llc_proc_err_ind(link_id, LLC_PROC_LOCAL, LLC_ERR_LLCP_UNKNOWN_RSP, pdu);

    return (CO_ERROR_NO_ERROR);
}


/**
 ****************************************************************************************
 * @brief Sends the unknown response PDU
 *
 * @param[in] link_id       Link identifier on which the PDU will be sent.
 * @param[in] opcode        Unknown Operation code.
 ****************************************************************************************
 */
__STATIC void llc_ll_unknown_rsp_pdu_send(uint8_t link_id, uint8_t op_code)
{
    struct ll_unknown_rsp pdu;

    pdu.op_code = LL_UNKNOWN_RSP_OPCODE;
    pdu.unk_type = op_code;

    llc_llcp_send(link_id, (union llcp_pdu *) &pdu, NULL);
}
/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void llc_ll_reject_ind_pdu_send(uint8_t link_id, uint8_t rej_op_code, uint8_t reason, llc_llcp_tx_cfm_cb cfm_cb)
{
    // If the features have been exchanged and the peer supports it, use extended reject indication
    // Always use extended reject indication for param_req and param_rsp
    if (llc_le_feature_check(link_id, BLE_FEAT_EXT_REJ_IND) || (rej_op_code > LL_REJECT_IND_OPCODE))
    {
        struct ll_reject_ext_ind pdu;

        pdu.op_code     = LL_REJECT_EXT_IND_OPCODE;
        pdu.err_code    = reason;
        pdu.rej_op_code = rej_op_code;

        llc_llcp_send(link_id, (union llcp_pdu *) &pdu, cfm_cb);
    }
    else
    {
        struct ll_reject_ind pdu;

        pdu.op_code     = LL_REJECT_IND_OPCODE;
        pdu.err_code    = reason;

        llc_llcp_send(link_id, (union llcp_pdu *) &pdu, cfm_cb);
    }
}



/*
 * LOCAL FUNCTIONS
 ****************************************************************************************
 */



/*
 * LLCP Message handler
 ****************************************************************************************
 */




void llc_llcp_send(uint8_t link_id, union llcp_pdu *pdu, llc_llcp_tx_cfm_cb tx_cfm_cb)
{
    /*
     ****************************************************************************************
     * PlantUML procedure description
     *
     * @startuml  llc_llcp_send.png
     * title : LLCP TX Path
     * participant LLC
     * participant LLC_LLCP
     * participant LLD
     * activate LLC_LLCP
     * LLC -> LLC_LLCP : llc_llcp_send(pdu, tx_end_cb)
     * note over LLC_LLCP #aqua : 1. Pack LLCP PDU and \n2. Register tx_end callback
     * note over LLC_LLCP : 3.PUSH in end of \n   LLCP TX queue
     * LLC_LLCP -> LLC_LLCP : llc_llcp_tx_check
     * note right LLC_LLCP: see llc_llcp_tx_check()
     * activate LLC_LLCP
     * hnote over LLC_LLCP : LLCP TX Busy
     * LLC_LLCP -> LLD: lld_con_llcp_tx(pdu)
     * deactivate LLC_LLCP
     * deactivate LLC_LLCP
     * LLD --> LLC_LLCP: LLD_LLCP_TX_CFM
     * LLC_LLCP -> LLC_LLCP: lld_llcp_tx_cfm_handler
     * activate LLC_LLCP
     * note over LLC_LLCP: 1. Retrieve PDU in queue
     * alt tx_end callback set
     *     LLC_LLCP -> LLC: tx_end_cb(op_code)
     * end
     * note over LLC_LLCP: 2. Free PDU Memory
     * hnote over LLC_LLCP : LLCP TX Idle
     * LLC_LLCP -> LLC_LLCP : llc_llcp_tx_check
     * activate LLC_LLCP
     * alt More LLCP PDU to send
     *     hnote over LLC_LLCP : LLCP TX Busy
     *     LLC_LLCP -> LLD: lld_con_llcp_tx(pdu)
     * end
     * deactivate LLC_LLCP
     * deactivate LLC_LLCP
     * @enduml
     *
     ****************************************************************************************
     */

    struct llc_env_tag *llc_env_ptr = llc_env[link_id];
    if (pdu  && llc_env_ptr && (pdu->op_code < LL_OPCODE_MAX_OPCODE))
    {
        uint8_t status;
        uint16_t tx_len = llcp_pdu_handler[pdu->op_code].length;
        llcp_pdu_t *elt = (llcp_pdu_t *) ke_malloc_system(sizeof(llcp_pdu_t) + tx_len, KE_MEM_KE_MSG);
        // set TX Confirm callback
        elt->tx_cfm_cb = tx_cfm_cb;

        // Perform PDU packing
        status = co_util_pack(elt->pdu, (uint8_t *) pdu, &tx_len, sizeof(union llcp_pdu),
                              llcp_pdu_handler[pdu->op_code].pack_format);

        ASSERT_INFO(status == CO_UTIL_PACK_OK, status, pdu->op_code);
        if (status == CO_UTIL_PACK_OK)
        {
            elt->length = (uint8_t) tx_len;

            // Push the LLCP to send at end of the queue list
            co_list_push_back(&(llc_env_ptr->llcp_tx_queue), &elt->hdr);

            // Try to send the packet immediately
            llc_llcp_tx_check(link_id);
        }
    }
    else
    {
        ASSERT_INFO_FORCE(0, link_id, pdu ? pdu->op_code : 0xFFFF);
    }
}


/*
 ****************************************************************************************
 * Try to push LLCP packet to send in LL Driver.
 * (only one buffer can be sent until it's acknowledgment)
 *
 *
 * @startuml  llc_llcp_tx_check.png
 *  title LLCP PDU Push algorithm
 *  start
 *  if (LLCP_TX_State == Busy?) then (yes)
 *  else (no)
 *      :PDU = LLCP_TX_QUEUE.first();
 *      while (PDU != NULL?) is (yes)
 *        if (LLCP TX Allowed?) then (yes)
 *        : call lld_con_llcp_tx(pdu);
 *        stop
 *        else (no)
 *        : PDU = PDU->next;
 *        endif
 *      endwhile (no)
 *  endif
 *  stop
 *  @enduml
 ****************************************************************************************
 */
void llc_llcp_tx_check(uint8_t link_id)
{
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];
    // check if a pdu can be pushed into the controller
    if (!GETB(llc_env_ptr->link_info, LLC_LLCP_TX_ONGOING))
    {
        bool first = true;
        llcp_pdu_t *elt = (llcp_pdu_t *) co_list_pick(&(llc_env_ptr->llcp_tx_queue));

        while (elt)
        {
            bool pdu_process = false;
            uint8_t op_code = elt->pdu[0];

            // apply a PDU filtering according to the current link state
            switch (GETF(llc_env_ptr->llcp_state, LLC_LLCP_TX))
            {
            case LLC_LLCP_IDLE:
            {
                pdu_process = ((llcp_pdu_handler[op_code].authz & LLC_LLCP_IDLE_AUTHZED) != 0);
            }
            break;
            case LLC_LLCP_PAUSE_ENC:
            {
                pdu_process = ((llcp_pdu_handler[op_code].authz & LLC_LLCP_PAUSE_ENC_AUTHZED) != 0);
            }
            break;
            case LLC_LLCP_START_ENC:
            {
                pdu_process = ((llcp_pdu_handler[op_code].authz & LLC_LLCP_START_ENC_AUTHZED) != 0);
            }
            break;
            case LLC_LLCP_TERMINATE:
            {
                pdu_process = ((llcp_pdu_handler[op_code].authz & LLC_LLCP_TERMINATE_AUTHZED) != 0);
            }
            break;
            default:
            {
                ASSERT_INFO_FORCE(0, link_id, GETF(llc_env_ptr->llcp_state, LLC_LLCP_TX));
            }
            break;
            }

            // Check if buffer can be sent in current state
            if (pdu_process)
            {
                struct ble_em_llcp_buf_elt *tx_elt = ble_util_buf_llcp_tx_alloc();
                ASSERT_ERR(tx_elt != NULL);

                // copy buffer into exchange memory
                em_wr(elt->pdu, tx_elt->buf_ptr, elt->length);
                tx_elt->length = elt->length;


                // Mark that LLCP TX is on-going
                SETB(llc_env_ptr->link_info, LLC_LLCP_TX_ONGOING, 1);


                // request LL Driver to send the LLCP pdu
                if (lld_con_llcp_tx(link_id, tx_elt) != CO_ERROR_NO_ERROR)
                {
                    // expect a disconnection soon, so just free the buffer
                    ble_util_buf_llcp_tx_free(tx_elt->buf_ptr);
                }
                else
                {
                    //Trace the LLCP packet
                    TRC_REQ_LLCP_TX(BLE_LINKID_TO_CONHDL(link_id), tx_elt->length, elt->pdu);

                    if (!first)
                    {
                        // Put buffer on top of the queue to handle it immediately when it receives baseband acknowledgment.
                        co_list_extract(&(llc_env_ptr->llcp_tx_queue), &(elt->hdr));
                        co_list_push_front(&(llc_env_ptr->llcp_tx_queue), &(elt->hdr));
                    }
                }
                break;
            }

            // check if next element can be sent
            elt   = (llcp_pdu_t *) elt->hdr.next;
            first = false;
        }
    }
}

typedef llc_llcp_pdu_handler_func_t (*lld_get_pdu_pre_handler)(uint8_t opcode);

static llc_llcp_pdu_handler_func_t lld_get_pdu_handler(uint8_t opcode)
{
    llc_llcp_pdu_handler_func_t handler = NULL;
    return handler;
}
lld_get_pdu_pre_handler lld_get_pdu_handler_pre = lld_get_pdu_handler;


/**
 ****************************************************************************************
 * @brief Handles reception of an LLCP data packet
 ****************************************************************************************
 */
int lld_llcp_rx_ind_handler(ke_msg_id_t const msgid, struct lld_llcp_rx_ind *param,
                            ke_task_id_t const dest_id, ke_task_id_t const src_id)
{

    /*
     ****************************************************************************************
     * PlantUML procedure description
     *
     * @startuml llc_llcp_rx_msc.png
     * title LLCP PDU Reception MSC
     * participant LLC
     * participant LLC_LLCP
     * participant LLD
     * LLD -->LLC_LLCP: LLD_LLCP_RX_IND
     * LLC_LLCP -> LLC_LLCP: lld_llcp_rx_ind_handler
     * activate LLC_LLCP
     * note over LLC_LLCP: Unpack PDU
     * alt PDU Allowed
     *      alt PDU Valid
     *          LLC_LLCP->LLC: llcp_llcp_XXX_pdu_handler(pdu, evt_cnt)
     *      else PDU Invalid
     *          LLC_LLCP->LLC: llc_llcp_unknown_ind_handler(op_code, evt_cnt)
     *      end
     * else Disconnect State
     *      note over LLC_LLCP: Ignore message
     * else Pause Encrypt or Start Encrypt State
     *      LLC_LLCP->LLC: llc_util_dicon_procedure(MIC Failure)
     * end
     * deactivate LLC_LLCP
     * @enduml
     *
     * @startuml llc_llcp_rx_state.png
     * title RX State Machine algorithm
     * start
     * if (Link active?) then (no)
     *     : ignore message;
     * else (yes)
     *     if (PDU Valid?) then (yes)
     *         :load and unpack pdu;
     *         :pdu_handler = llcp_XXX_pdu_handler();
     *     else (no)
     *         :pdu_handler = llc_llcp_unknown_ind_handler();
     *     endif
     *     if (PDU Allowed?) then (yes)
     *         :call state = pdu_handler();
     *         if(state?) then (NO_ERROR)
     *         else if (MIC_ERROR)
     *             :disconnect the link;
     *         else if (OTHER_ERROR)
     *             :Send llcp_reject_ind(_ext);
     *         endif
     *     else (no)
     *         if(LINK State?) then (disconnected)
     *             : ignore message;
     *         else (Pause Encrypt or Start Encrypt)
     *             : Stop connection (MIC Error);
     *         endif
     *     endif
     * endif
     * stop
     * @enduml
     *
     ****************************************************************************************
     */

    // Current message status
    int msg_status = KE_MSG_CONSUMED;
    uint8_t link_id = KE_IDX_GET(dest_id);

    struct llc_env_tag *llc_env_ptr = llc_env[link_id];
    ASSERT_INFO(llc_env_ptr != NULL, link_id,  0);

    // check connection exist
    if (llc_env_ptr != NULL)
    {

        // Check if LLC messages have to be discarded
        if (GETB(llc_env_ptr->link_info, LLC_INFO_DBG_LLCP_DISCARD))
        {
            // discard the message
        }
        else
        {
            bool pdu_process = false;
            uint8_t status;
            uint8_t rx_data[LL_PDU_LENGTH_MAX];
            union llcp_pdu pdu;

            // load pdu data
            em_rd(rx_data, param->em_buf, co_min(param->length, LL_PDU_LENGTH_MAX));
            pdu.op_code = rx_data[0];

            //Trace the received LLCP packet
            TRC_REQ_LLCP_RX(BLE_LINKID_TO_CONHDL(link_id), param->length, rx_data);

            // Check op_code
            if (pdu.op_code >= LL_OPCODE_MAX_OPCODE)
            {
                status = CO_ERROR_UNKNOWN_LMP_PDU;

                // check if PDU should be processed or not
                switch (GETF(llc_env_ptr->llcp_state, LLC_LLCP_RX))
                {
                case LLC_LLCP_IDLE:
                {
                    pdu_process = true;
                }
                break;
                case LLC_LLCP_PAUSE_ENC:
                case LLC_LLCP_START_ENC:
                {
                    // This unexpected packet has been received during an Encryption Pause Procedure,
                    // so we need to exit the connection state immediately with reason "MIC Failure".
                    llc_disconnect(link_id, CO_ERROR_TERMINATED_MIC_FAILURE, true);
                }
                break;
                case LLC_LLCP_TERMINATE:
                {
                    // else discard message
                } break;
                default:
                {
                    ASSERT_INFO_FORCE(0, link_id, GETF(llc_env_ptr->llcp_state, LLC_LLCP_RX));
                }
                break;
                }
            }
            else if (param->length != llcp_pdu_handler[pdu.op_code].length)
            {
                status = CO_ERROR_UNKNOWN_LMP_PDU; // incorrect length
                pdu_process = true;
            }
            else
            {
                uint16_t pdu_len = sizeof(union llcp_pdu);

                // Perform PDU unpacking
                status = co_util_unpack((uint8_t *) & (pdu), rx_data, &pdu_len, param->length,
                                        llcp_pdu_handler[pdu.op_code].pack_format);

                // check if unpack succeed or not
                if (status == CO_UTIL_PACK_OK)
                {
                    status = CO_ERROR_NO_ERROR;
                }
                else
                {
                    status = CO_ERROR_INVALID_LMP_PARAM;
                }

                // apply a pdu filtering according to the current link state
                switch (GETF(llc_env_ptr->llcp_state, LLC_LLCP_RX))
                {
                case LLC_LLCP_IDLE:
                {
                    if (llcp_pdu_handler[pdu.op_code].authz & LLC_LLCP_IDLE_AUTHZED)
                    {
                        pdu_process = true;
                    }
                    // else discard message
                }
                break;
                case LLC_LLCP_PAUSE_ENC:
                {
                    if (llcp_pdu_handler[pdu.op_code].authz & LLC_LLCP_PAUSE_ENC_AUTHZED)
                    {
                        pdu_process = true;
                    }
                    else
                    {
                        // This unexpected packet has been received during an Encryption Pause Procedure,
                        // so we need to exit the connection state immediately with reason "MIC Failure".
                        llc_disconnect(link_id, CO_ERROR_TERMINATED_MIC_FAILURE, true);
                    }
                }
                break;
                case LLC_LLCP_START_ENC:
                {
                    if (llcp_pdu_handler[pdu.op_code].authz & LLC_LLCP_START_ENC_AUTHZED)
                    {
                        pdu_process = true;
                    }
                    else
                    {
                        // This unexpected packet has been received during an Encryption Start,
                        // so we need to exit the connection state immediately with reason "MIC Failure".
                        llc_disconnect(link_id, CO_ERROR_TERMINATED_MIC_FAILURE, true);
                    }
                }
                break;
                case LLC_LLCP_TERMINATE:
                {
                    if (llcp_pdu_handler[pdu.op_code].authz & LLC_LLCP_TERMINATE_AUTHZED)
                    {
                        pdu_process = true;
                    }
                    // else discard message
                }
                break;
                default:
                {
                    ASSERT_INFO_FORCE(0, link_id, llc_env_ptr->llcp_state);
                }
                break;
                }
            }

            // PDU can be processed
            if (pdu_process)
            {
                // valid llcp pdu
                if (status == CO_ERROR_NO_ERROR)
                {
                    ASSERT_INFO(pdu.op_code < LL_OPCODE_MAX_OPCODE, dest_id, pdu.op_code);
                    llc_llcp_pdu_handler_func_t pre_handler;
                    // execute message handler
                    if ((pre_handler = lld_get_pdu_handler_pre(pdu.op_code)) != NULL)
                        status = pre_handler(link_id, &pdu, param->event_cnt);
                    else
                        status = llcp_pdu_handler[pdu.op_code].handler(link_id, &pdu, param->event_cnt);
                }
                // unknown pdu
                else if (status == CO_ERROR_UNKNOWN_LMP_PDU)
                {
                    status = llc_ll_unknown_ind_handler(link_id, pdu.op_code, param->event_cnt);
                }

                // after execution of handler, check
                switch (status)
                {
                // no error
                case CO_ERROR_NO_ERROR:
                {
                    // nothing to do
                } break;

                // mic failure or instant passed, disconnect
                case CO_ERROR_TERMINATED_MIC_FAILURE:
                case CO_ERROR_INSTANT_PASSED:
                {
                    llc_disconnect(link_id, status, true);
                }
                break;

                // other error codes - send a reject pdu
                default:
                {
                    llc_ll_reject_ind_pdu_send(link_id, pdu.op_code, status, NULL);
                }
                break;
                }
            }
        }
    }

    // Free RX buffer
    ble_util_buf_rx_free(param->em_buf);

    // restart LE_PING timer
    llc_le_ping_restart(link_id);

    return (msg_status);
}



/**
 ****************************************************************************************
 * @brief Handles reception of an LLCP TX buffer baseband acknowledgmentS
 ****************************************************************************************
 */
int lld_llcp_tx_cfm_handler(ke_msg_id_t const msgid, void *param,
                            ke_task_id_t const dest_id, ke_task_id_t const src_id)
{

    /*
     ****************************************************************************************
     * PlantUML procedure description
     *
     * @startuml llc_llcp_tx_cfm.png
     * title : LLCP TX Confirmation Algorithm
     * participant LLC
     * participant LLC_LLCP
     * participant LLD
     * LLD --> LLC_LLCP: LLD_LLCP_TX_CFM
     * LLC_LLCP -> LLC_LLCP: lld_llcp_tx_cfm_handler
     * activate LLC_LLCP
     * note over LLC_LLCP: 1. Retrieve PDU in queue
     * alt tx_end callback set
     *     LLC_LLCP -> LLC: tx_end_cb(op_code)
     * end
     * note over LLC_LLCP: 2. Free PDU Memory
     * hnote over LLC_LLCP : LLCP TX Idle
     * LLC_LLCP -> LLC_LLCP : llc_llcp_tx_check
     * activate LLC_LLCP
     * alt More LLCP PDU to send
     *     hnote over LLC_LLCP : LLCP TX Busy
     *     LLC_LLCP -> LLD: lld_con_llcp_tx(pdu)
     * end
     * deactivate LLC_LLCP
     * deactivate LLC_LLCP
     * @enduml
     *
     ****************************************************************************************
     */

    // Current message status
    int msg_status = KE_MSG_CONSUMED;
    uint8_t link_id = KE_IDX_GET(dest_id);

    struct llc_env_tag *llc_env_ptr = llc_env[link_id];
    ASSERT_INFO(llc_env_ptr != NULL, link_id,  0);

    // check connection exist
    if (llc_env_ptr != NULL)
    {
        // Handles TX packet confirmation and inform corresponding task handler
        llcp_pdu_t *elt = (llcp_pdu_t *) co_list_pop_front(&(llc_env_ptr->llcp_tx_queue));

        //Trace the acknowledge LLCP packet
        TRC_REQ_LLCP_ACK(BLE_LINKID_TO_CONHDL(link_id), elt->length, elt->pdu);

        // Check if procedure is waiting for LLCP baseband ACK
        if (elt->tx_cfm_cb != NULL)
        {
            elt->tx_cfm_cb(link_id, elt->pdu[0]);
        }

        // remove element
        ke_free(elt);

        // Mark that No LLCP TX is on-going
        SETB(llc_env_ptr->link_info, LLC_LLCP_TX_ONGOING, 0);
    }

    // Check if another packet has to be transmitted
    llc_llcp_tx_check(link_id);

    return (msg_status);
}

#if (RW_DEBUG)
/// @brief Handle the reception of the vendor specific command send llcp.
int hci_dbg_send_llcp_cmd_handler(uint8_t link_id, struct hci_dbg_send_llcp_cmd const *param, uint16_t opcode)
{
    // Get environment pointer
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];

    llcp_pdu_t *elt = (llcp_pdu_t *) ke_malloc_system(sizeof(llcp_pdu_t) + param->buf.length, KE_MEM_KE_MSG);
    elt->tx_cfm_cb = NULL;
    elt->length = param->buf.length;
    memcpy(&elt->pdu[0], &param->buf.data[0], param->buf.length);

    // Push the LLCP to send at end of the queue list
    co_list_push_back(&(llc_env_ptr->llcp_tx_queue), &elt->hdr);

    // Try to send the packet immediately
    llc_llcp_tx_check(link_id);

    llc_cmd_cmp_send(link_id, opcode, CO_ERROR_NO_ERROR);

    return (KE_MSG_CONSUMED);
}
#endif // (RW_DEBUG)

#endif // #if (BLE_PERIPHERAL || BLE_CENTRAL)
/// @} LLCLLCP
