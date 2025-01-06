/**
****************************************************************************************
*
* @file lc_sco.c
*
* @brief Definitions for LC SCO procedures
*
* Copyright (C) RivieraWaves 2009-2015
*
*
****************************************************************************************
*/

/**
 ****************************************************************************************
 * @addtogroup LCSCO
 * @ingroup LC
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"    // stack configuration


#if (MAX_NB_SYNC > 0)

#include <string.h>         // standard mem definitions
#include "co_math.h"
#include "co_bt.h"             // common bluetooth
#include "co_lmp.h"
#include "co_utils.h"

#include "ke_mem.h"         // Kernel Memory
#include "ke_timer.h"         // Kernel Timer

#include "lc_sco.h"         // link controller sco
#include "lm_sco.h"
#include "lc.h"             // link controller
#include "lc_int.h"         // link controller internal
#include "lc_util.h"             // link controller
#include "lc_lmppdu.h"
#if (EAVESDROPPING_SUPPORT)
    #include "../lm/lm_int.h"
#endif // EAVESDROPPING_SUPPORT

#include "sch_plan.h"

#include "lm.h"
#include "ld.h"
#include "hci.h"
#include "btdm_patch.h"

/*
 * DEFINES
 *****************************************************************************************
 */

/// Default sync handle
#define DEFAUT_SYNC_HANDLE     0x00


/// Definitions for Synchronous data path configuration
#define LINK1_CFG_MSK       0x000F
#define LINK1_CFG_SFT       0
#define LINK2_CFG_MSK       0x00F0
#define LINK2_CFG_SFT       4
#define LINK3_CFG_MSK       0x0F00
#define LINK3_CFG_SFT       8


/// SCO Negotiation States
enum LC_SCO_STATES
{
    LC_SCO_IDLE,
    LC_SCO_NEGO_WAIT_HOST_RSP,
    LC_SCO_NEGO_WAIT_PEER_RSP,
    LC_SCO_NEGO_WAIT_PEER_CFM,
    LC_SCO_CONNECTED,
    LC_SCO_NEGO_WAIT_PEER_MOD_RSP,
    LC_SCO_NEGO_WAIT_PEER_MOD_CFM,
    LC_SCO_NEGO_WAIT_PEER_DISC_RSP,
    LC_SCO_NEGO_WAIT_PEER_DISC_CFM,
};

/// SCO Negotiation Action
enum LC_SCO_ACTION
{
    LC_SCO_ACCEPT,
    LC_SCO_REJECT,
    LC_SCO_REQUEST,
};


/*
 * STRUCTURE DEFINITION
 *****************************************************************************************
 */

/// LC SCO link parameters structure
struct lc_sco_link_params_tag
{
    /// Planner element
    struct sch_plan_elt_tag plan_elt;

    /// Synchronous packet type bit field given by Host
    uint16_t sync_pkt_type;

    /// ACL link ID
    uint16_t link_id;

    /// Synchronous handle used on ESCO link
    uint8_t sync_hdl;

    /// Synchronous LT address used on ESCO link
    uint8_t sync_lt_addr;

    /// Synchronous connection type (SCO / ESCO)
    uint8_t sync_type;

    /// Current state of the synchronous connection
    uint8_t state;

    /// Disconnection reason
    uint8_t disc_reason;

    /// Disconnection type (True: disconnect all sync links on a ACL / False: disconnect 1 sync link)
    bool disc_all;

    /// Flag indicating if HV1 is used or not
    bool hv1_flag;

    /*
     * Store parameters that can be renegotiated
     */
    /// Interval (in slots)
    uint8_t t_esco;
    /// Retransmission window (in slots)
    uint8_t w_esco;
    /// Packet length in receive direction (in bytes)
    uint16_t rx_pkt_len;
    /// Packet length in transmit direction (in bytes)
    uint16_t tx_pkt_len;
};

struct lc_sco_nego_infos_tag
{
    /// Flag indicating if local is initiator of the negotiation
    bool initiator;

    /// Flag indicating if the old HCI command is used by Host (HCI_setup_sync_con / HCI_acc_sync_con)
    bool old_style_hci;

    /// Flag indicating if the very old HCI command is used by Host when accepting the sync link (HCI_acc_con / HCI_rej_con)
    bool very_old_style_hci;

    /// Flag indicating if the procedure was initiated by the host (true) or not (false)
    bool host_initiated;
};


/// LC SCO environment structure
struct lc_sco_nego_params_tag
{
    /// General information of the negotiation
    struct lc_sco_nego_infos_tag nego_infos;

    /// Parameters received from Host
    struct lc_sco_host_params_tag host_params;

    /// Parameters received from peer
    struct lc_sco_air_params_tag air_params_received;

    /// Parameters sent to Host
    struct lc_sco_air_params_tag air_params_sent;
};

/// LC SCO environment structure
struct lc_sco_env_tag
{
    /// Pointer to the structure for allocated the sync link
    struct lc_sco_link_params_tag *p_link_params;

    /// Pointer to the structure for allocated the negotiation
    struct lc_sco_nego_params_tag *p_nego_params;
};


/*
 * VARIABLE DEFINITION
 *****************************************************************************************
 */

/// LC SCO environment variable
__STATIC struct lc_sco_env_tag lc_sco_env[MAX_NB_SYNC];

/// Sync data path configuration
__STATIC uint16_t lc_sco_data_path_config;


/*
 * LOCAL FUNCTIONS DECLARATION
 *****************************************************************************************
 */
__STATIC void lc_sco_send_host_cmp_evt(ke_task_id_t pid, uint8_t sync_linkid, uint8_t status);
__STATIC void lc_sco_send_host_chg_evt(ke_task_id_t pid, uint8_t sync_linkid, uint8_t status);

__STATIC void lc_sco_wrapper_lm_remove_sync(ke_task_id_t pid, uint8_t sync_linkid);
__STATIC uint8_t lc_sco_wrapper_lm_get_sync_param(ke_task_id_t pid, uint8_t sync_linkid, uint8_t request);
extern void lc_save_sco_para(struct hci_sync_con_cmp_evt *evt);



/*
 * LOCAL FUNCTIONS DEFINITION
 *****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handle move notification
 ****************************************************************************************
 */
__STATIC void lc_sco_move_cbk(uint16_t id)
{
    uint8_t link_id = id & 0x7F;
    ke_task_id_t lc_id = KE_BUILD_ID(TASK_LC, link_id);

    // Trigger setup synchronous connection procedure
    struct lc_op_loc_sync_con_req *req = KE_MSG_ALLOC(LC_OP_LOC_SYNC_CON_REQ, lc_id, lc_id, lc_op_loc_sync_con_req);
    req->conhdl = id;
    req->old_style = true;
    req->host_initiated = false;
    ke_msg_send(req);
}

/**
 ****************************************************************************************
 * @brief Indicate the synchronous connection update to all active links.
 *
 * @param[in] pid        process identifier.
 ****************************************************************************************
 */
__STATIC void lc_sco_ind(ke_task_id_t pid)
{
    // Send the request to all other active link(s)
    for (int idx = 0; idx < MAX_NB_ACTIVE_ACL; idx++)
    {
        // Check if connection is active
        if (ke_state_get(KE_BUILD_ID(TASK_LC, idx)) != LC_FREE)
        {
            ke_msg_send_basic(LC_SYNC_IND, KE_BUILD_ID(TASK_LC, idx), pid);
        }
    }
}

//**************************************************************************************************************
//**************************************************************************************************************
//**************************************************************************************************************
//***********************                 RESOURCE MANAGEMENT                ***********************************
//**************************************************************************************************************
//**************************************************************************************************************
//**************************************************************************************************************

__STATIC uint8_t lc_sco_alloc_link(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    uint8_t sync_linkid = 0;

    // Try to allocate a new SCO link
    for (sync_linkid = 0; sync_linkid < MAX_NB_SYNC ; sync_linkid++)
    {
        // Check if sco link ID is free
        if (lc_sco_env[sync_linkid].p_link_params == NULL)
        {
            break;
        }
    }

    if (sync_linkid < MAX_NB_SYNC)
    {
        // Allocate new link buffer
        lc_sco_env[sync_linkid].p_link_params = (struct lc_sco_link_params_tag *) ke_malloc_system(sizeof(struct lc_sco_link_params_tag), KE_MEM_NON_RETENTION);

        // Save ACL connection handle
        lc_sco_env[sync_linkid].p_link_params->link_id = idx;

        // Initialize SCO negotiation state machine
        lc_sco_env[sync_linkid].p_link_params->state = LC_SCO_IDLE;
    }

    return sync_linkid;
}

__STATIC void lc_sco_free_link(ke_task_id_t pid, uint8_t sync_linkid)
{

    lc_sco_wrapper_lm_remove_sync(pid, sync_linkid);

    // Remove planner element
    sch_plan_rem(&lc_sco_env[sync_linkid].p_link_params->plan_elt);

    // Free the link buffer
    if (lc_sco_env[sync_linkid].p_link_params != NULL)
    {
        ke_free(lc_sco_env[sync_linkid].p_link_params);
        lc_sco_env[sync_linkid].p_link_params = NULL;
    }
    else
    {
        ASSERT_INFO_FORCE(0, sync_linkid, 0);
    }

    ASSERT_INFO(lc_sco_env[sync_linkid].p_nego_params == NULL, sync_linkid, 0);
}

__STATIC void lc_sco_alloc_nego(ke_task_id_t pid, uint8_t sync_linkid)
{
    // Allocate new negotiation buffer
    lc_sco_env[sync_linkid].p_nego_params = (struct lc_sco_nego_params_tag *) ke_malloc_system(sizeof(struct lc_sco_nego_params_tag), KE_MEM_NON_RETENTION);

    // Initialize all nego flags to false
    memset((uint8_t *) lc_sco_env[sync_linkid].p_nego_params, 0, sizeof(struct lc_sco_nego_params_tag));

    // Set LC task state machine
    ke_state_set(pid, LC_SCO_NEGO_ONGOING);
}

__STATIC void lc_sco_free_nego(ke_task_id_t pid, uint8_t sync_linkid)
{
    // Free the nego buffer
    if (lc_sco_env[sync_linkid].p_nego_params != NULL)
    {
        ke_free(lc_sco_env[sync_linkid].p_nego_params);
        lc_sco_env[sync_linkid].p_nego_params = NULL;
        lm_sco_nego_end();
    }
    else
    {
        ASSERT_INFO_FORCE(0, sync_linkid, 0);
    }

    // Clear LC task state machine
    ke_state_set(pid, LC_CONNECTED);
}

__STATIC void lc_sco_start_link(ke_task_id_t pid, uint8_t sync_linkid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    uint8_t status = CO_ERROR_NO_ERROR;
    struct ld_sco_params params;

    lc_sco_env[sync_linkid].p_link_params->hv1_flag = false;

    // Save packet type table
    lc_sco_env[sync_linkid].p_link_params->sync_pkt_type = lc_sco_env[sync_linkid].p_nego_params->host_params.packet_type;

    // Save link identifiers
    if (lc_env_ptr->link.Role == MASTER_ROLE)
    {
        lc_sco_env[sync_linkid].p_link_params->sync_hdl = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.esco_hdl;
        lc_sco_env[sync_linkid].p_link_params->sync_lt_addr = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.esco_lt_addr;
    }
    else
    {
        lc_sco_env[sync_linkid].p_link_params->sync_hdl = lc_sco_env[sync_linkid].p_nego_params->air_params_received.esco_hdl;
        lc_sco_env[sync_linkid].p_link_params->sync_lt_addr = lc_sco_env[sync_linkid].p_nego_params->air_params_received.esco_lt_addr;
    }

    // Start SCO link
    params.lt_addr   = lc_sco_env[sync_linkid].p_link_params->sync_lt_addr;
    params.sync_type = lc_sco_env[sync_linkid].p_link_params->sync_type;
    params.d_esco = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.d_esco;
    params.t_esco = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.t_esco;
    params.w_esco = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.w_esco;
    params.init   = ((lc_sco_env[sync_linkid].p_nego_params->air_params_sent.flags & INIT2_FLAG) != 0);
    if (lc_env_ptr->link.Role == MASTER_ROLE)
    {
        params.rx_pkt_type = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.s2m_pkt_type;
        params.tx_pkt_type = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.m2s_pkt_type;
        params.rx_pkt_len  = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.s2m_pkt_len;
        params.tx_pkt_len  = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.m2s_pkt_len;
    }
    else
    {
        params.rx_pkt_type = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.m2s_pkt_type;
        params.tx_pkt_type = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.s2m_pkt_type;
        params.rx_pkt_len  = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.m2s_pkt_len;
        params.tx_pkt_len  = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.s2m_pkt_len;
    }
    if (lc_sco_env[sync_linkid].p_link_params->sync_type == SCO_TYPE)
    {
        // Only HV3/HV1 supported in SCO
        if (params.t_esco == 2)
        {
            params.rx_pkt_len  = HV1_PACKET_SIZE;
            params.tx_pkt_len  = params.rx_pkt_len;
            lc_sco_env[sync_linkid].p_link_params->hv1_flag = true;
        }
        else
        {
            params.rx_pkt_len  = HV3_PACKET_SIZE;
            params.tx_pkt_len  = params.rx_pkt_len;
        }
    }
    params.audio.air_coding = lc_sco_env[sync_linkid].p_nego_params->host_params.tx_cod_fmt[0];
    params.audio.in_coding = lc_sco_env[sync_linkid].p_nego_params->host_params.in_cod_fmt[0];
    params.audio.data_path = lc_sco_env[sync_linkid].p_nego_params->host_params.in_data_path;
    params.audio.in_data_format = lc_sco_env[sync_linkid].p_nego_params->host_params.in_data_fmt;
    params.audio.in_sample_size = lc_sco_env[sync_linkid].p_nego_params->host_params.in_cod_data_sz;
    params.audio.msb_position = lc_sco_env[sync_linkid].p_nego_params->host_params.in_msb_pos;
    status = ld_sco_start(idx, sync_linkid, &params, ((lm_get_loopback_mode() == REMOTE_LOOPBACK) || lc_env_ptr->link.esco_loopback_mode));

    ASSERT_INFO(status == CO_ERROR_NO_ERROR, sync_linkid, status);

    lc_util_set_loc_trans_coll(idx, LC_UTIL_NOT_USED, LC_UTIL_NOT_USED, LC_UTIL_NOT_USED);

    // Set SCO negotiation state machine
    lc_sco_env[sync_linkid].p_link_params->state = LC_SCO_CONNECTED;

    // Inform Host that SCO setup is completed
    lc_sco_send_host_cmp_evt(pid, sync_linkid, status);

    // Store link parameters
    lc_sco_env[sync_linkid].p_link_params->t_esco     = params.t_esco;
    lc_sco_env[sync_linkid].p_link_params->w_esco     = params.w_esco;
    lc_sco_env[sync_linkid].p_link_params->rx_pkt_len = params.rx_pkt_len;
    lc_sco_env[sync_linkid].p_link_params->tx_pkt_len = params.tx_pkt_len;

    // Register activity in the planner
    lc_sco_env[sync_linkid].p_link_params->plan_elt.offset       = lc_offset_lmp_to_local(2 * params.d_esco, 2 * params.t_esco, ld_read_clock(), ld_acl_clock_offset_get(idx).hs, params.init);
    lc_sco_env[sync_linkid].p_link_params->plan_elt.interval     = params.t_esco * 2;
    lc_sco_env[sync_linkid].p_link_params->plan_elt.duration_min = 4;
    lc_sco_env[sync_linkid].p_link_params->plan_elt.duration_max = 4 + params.w_esco * 2;
    lc_sco_env[sync_linkid].p_link_params->plan_elt.conhdl       = BT_SYNC_CONHDL(idx, sync_linkid);
    lc_sco_env[sync_linkid].p_link_params->plan_elt.conhdl_ref   = BT_ACL_CONHDL_MIN + idx;
    lc_sco_env[sync_linkid].p_link_params->plan_elt.margin       = 1 * (lc_env_ptr->link.Role == SLAVE_ROLE);
    if (lm_sco_move_en())
    {
        lc_sco_env[sync_linkid].p_link_params->plan_elt.cb_move = &lc_sco_move_cbk;
        lc_sco_env[sync_linkid].p_link_params->plan_elt.mobility = (lc_env_ptr->link.Role == MASTER_ROLE) ? SCH_PLAN_MB_LVL_3 : SCH_PLAN_MB_LVL_2;
    }
    else
    {
        lc_sco_env[sync_linkid].p_link_params->plan_elt.cb_move = NULL;
        lc_sco_env[sync_linkid].p_link_params->plan_elt.mobility = SCH_PLAN_MB_LVL_0;
    }

    sch_plan_set(&lc_sco_env[sync_linkid].p_link_params->plan_elt);

    // End of link negotiation
    lc_sco_free_nego(pid, sync_linkid);

    // Informs LC that a SCO link has been created
    lc_sco_ind(pid);
}

__STATIC void lc_sco_modify_link(ke_task_id_t pid, uint8_t sync_linkid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    uint8_t status = CO_ERROR_NO_ERROR;
    struct ld_sco_params params;

    if (lc_sco_env[sync_linkid].p_nego_params->nego_infos.initiator)
    {
        // Save packet type table
        lc_sco_env[sync_linkid].p_link_params->sync_pkt_type = lc_sco_env[sync_linkid].p_nego_params->host_params.packet_type;
    }

    // Update SCO link
    params.lt_addr   = lc_sco_env[sync_linkid].p_link_params->sync_lt_addr;
    params.sync_type = lc_sco_env[sync_linkid].p_link_params->sync_type;
    params.d_esco = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.d_esco;
    params.t_esco = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.t_esco;
    params.w_esco = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.w_esco;
    params.init   = ((lc_sco_env[sync_linkid].p_nego_params->air_params_sent.flags & INIT2_FLAG) != 0);
    if (lc_env_ptr->link.Role == MASTER_ROLE)
    {
        params.rx_pkt_type = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.s2m_pkt_type;
        params.tx_pkt_type = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.m2s_pkt_type;
        params.rx_pkt_len  = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.s2m_pkt_len;
        params.tx_pkt_len  = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.m2s_pkt_len;
    }
    else
    {
        params.rx_pkt_type = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.m2s_pkt_type;
        params.tx_pkt_type = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.s2m_pkt_type;
        params.rx_pkt_len  = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.m2s_pkt_len;
        params.tx_pkt_len  = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.s2m_pkt_len;
    }
    if (lc_sco_env[sync_linkid].p_link_params->sync_type == SCO_TYPE)
    {
        // Only HV3/HV1 supported in SCO
        if (params.t_esco == 2)
        {
            params.rx_pkt_len  = HV1_PACKET_SIZE;
            params.tx_pkt_len  = params.rx_pkt_len;
        }
        else
        {
            params.rx_pkt_len  = HV3_PACKET_SIZE;
            params.tx_pkt_len  = params.rx_pkt_len;
        }
    }
#if (EAVESDROPPING_SUPPORT)
    params.audio.air_coding = lc_sco_env[sync_linkid].p_nego_params->host_params.tx_cod_fmt[0];
#endif // EAVESDROPPING_SUPPORT
    status = ld_sco_update(sync_linkid, &params);

    ASSERT_INFO(status == CO_ERROR_NO_ERROR, sync_linkid, status);

    lc_util_set_loc_trans_coll(idx, LC_UTIL_NOT_USED, LC_UTIL_NOT_USED, LC_UTIL_NOT_USED);

    // Set SCO negotiation state machine
    lc_sco_env[sync_linkid].p_link_params->state = LC_SCO_CONNECTED;

    // Unregister activity from planner
    sch_plan_rem(&lc_sco_env[sync_linkid].p_link_params->plan_elt);

    // Register activity in the planner with new parameters
    lc_sco_env[sync_linkid].p_link_params->plan_elt.offset       = lc_offset_lmp_to_local(2 * params.d_esco, 2 * params.t_esco, ld_read_clock(), ld_acl_clock_offset_get(idx).hs, params.init);
    lc_sco_env[sync_linkid].p_link_params->plan_elt.interval     = params.t_esco * 2;
    lc_sco_env[sync_linkid].p_link_params->plan_elt.duration_min = 4;
    lc_sco_env[sync_linkid].p_link_params->plan_elt.duration_max = 4 + params.w_esco * 2;
    sch_plan_set(&lc_sco_env[sync_linkid].p_link_params->plan_elt);

    // Check if new parameters are meaningful for the Host
    if (lc_sco_env[sync_linkid].p_nego_params->nego_infos.host_initiated
            || lc_sco_env[sync_linkid].p_link_params->t_esco     != params.t_esco
            || lc_sco_env[sync_linkid].p_link_params->w_esco     != params.w_esco
            || lc_sco_env[sync_linkid].p_link_params->rx_pkt_len != params.rx_pkt_len
            || lc_sco_env[sync_linkid].p_link_params->tx_pkt_len != params.tx_pkt_len)
    {
        // Inform Host that SCO setup is completed
        lc_sco_send_host_chg_evt(pid, sync_linkid, status);
    }

    // Store link parameters
    lc_sco_env[sync_linkid].p_link_params->t_esco     = params.t_esco;
    lc_sco_env[sync_linkid].p_link_params->w_esco     = params.w_esco;
    lc_sco_env[sync_linkid].p_link_params->rx_pkt_len = params.rx_pkt_len;
    lc_sco_env[sync_linkid].p_link_params->tx_pkt_len = params.tx_pkt_len;

    // End of link negotiation
    lc_sco_free_nego(pid, sync_linkid);

    // Informs LC that a SCO link has changed
    lc_sco_ind(pid);
}

__STATIC void lc_sco_stop_link(ke_task_id_t pid, uint8_t sync_linkid)
{
    // Stop SCO link
    ld_sco_stop(sync_linkid);
}


__STATIC void lc_sco_send_host_cmp_evt(ke_task_id_t pid, uint8_t sync_linkid, uint8_t status)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LC_TASK_PATCH_TYPE, LC_SCO_SEND_HOST_CMP_EVT_FUN_BIT, pid, sync_linkid, status);
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    if (!lc_sco_env[sync_linkid].p_nego_params->nego_infos.very_old_style_hci)
    {
        uint16_t conhdl = BT_SYNC_CONHDL(idx, sync_linkid);
        // allocate the status event message
        struct hci_sync_con_cmp_evt *event = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_SYNC_CON_CMP_EVT_CODE, hci_sync_con_cmp_evt);
        event->status = status;
        event->conhdl = conhdl;
        memcpy(&event->bd_addr.addr[0], &lc_env_ptr->info.BdAddr.addr[0], BD_ADDR_LEN);
        event->lk_type = lc_sco_env[sync_linkid].p_link_params->sync_type;
        if (lc_sco_env[sync_linkid].p_link_params->sync_type == SCO_TYPE)
        {
            event->tx_int  = 0x00;
        }
        else
        {
            event->tx_int  = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.t_esco;
        }
        event->ret_win = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.w_esco;
        if (lc_env_ptr->link.Role == MASTER_ROLE)
        {
            event->rx_pkt_len = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.s2m_pkt_len;
            event->tx_pkt_len = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.m2s_pkt_len;
        }
        else
        {
            event->tx_pkt_len = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.s2m_pkt_len;
            event->rx_pkt_len = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.m2s_pkt_len;
        }
        event->air_mode = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.air_mode;
        lc_save_sco_para(event);
        hci_send_2_host(event);
    }
    else
    {
        uint16_t conhdl = BT_SYNC_CONHDL(idx, sync_linkid);
        // allocate the status event message
        struct hci_con_cmp_evt *event = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_CON_CMP_EVT_CODE, hci_con_cmp_evt);
        event->status = status;
        event->conhdl = conhdl;
        event->link_type = lc_sco_env[sync_linkid].p_link_params->sync_type;
        event->enc_en = lc_env_ptr->enc.EncMode;
        memcpy(&event->bd_addr.addr[0], &lc_env_ptr->info.BdAddr.addr[0], BD_ADDR_LEN);
        hci_send_2_host(event);
    }
    if (bt_sco_data_handle_fun)  // host callback function
    {
        bt_sco_callback_para_t sco_callback_para;
        sco_callback_para.handle_type = AUDIO_PATH_SCO_CMP_EVT;
        sco_callback_para.un_para.u32_value = lc_sco_env[sync_linkid].p_nego_params->nego_infos.very_old_style_hci;
        bt_sco_data_handle_fun(&sco_callback_para);

    }
}


__STATIC void lc_sco_send_host_chg_evt(ke_task_id_t pid, uint8_t sync_linkid, uint8_t status)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LC_TASK_PATCH_TYPE, LC_SCO_SEND_HOST_CHG_EVT_FUN_BIT, pid, sync_linkid, status);
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    if (lc_sco_env[sync_linkid].p_link_params->sync_type == SCO_TYPE)
    {
        if ((lc_sco_env[sync_linkid].p_nego_params->nego_infos.initiator) && (lc_sco_env[sync_linkid].p_nego_params->nego_infos.host_initiated))
        {
            // allocate the status event message
            uint16_t conhdl = BT_SYNC_CONHDL(idx, sync_linkid);
            struct hci_con_pkt_type_chg_evt *event = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_CON_PKT_TYPE_CHG_EVT_CODE, hci_con_pkt_type_chg_evt);
            event->status = status;
            event->sync_conhdl = conhdl;

            // Inform Host about the final packet type used on the link
            switch (lc_sco_env[sync_linkid].p_nego_params->air_params_sent.m2s_pkt_type)
            {
            case SCO_PACKET_HV1:
            {
                event->pkt_type = PACKET_TYPE_HV1_FLAG;
            }
            break;
            case SCO_PACKET_HV3:
            {
                event->pkt_type = PACKET_TYPE_HV3_FLAG;
            }
            break;
            default:
            {
                ASSERT_INFO_FORCE(0, sync_linkid, lc_sco_env[sync_linkid].p_nego_params->air_params_sent.m2s_pkt_type);
            }
            break;
            }
            hci_send_2_host(event);
        }
    }
    else
    {
        // allocate the status event message
        uint16_t conhdl = BT_SYNC_CONHDL(idx, sync_linkid);
        struct hci_sync_con_chg_evt *event = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_SYNC_CON_CHG_EVT_CODE, hci_sync_con_chg_evt);
        event->status = status;
        event->sync_conhdl = conhdl;
        event->tx_int = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.t_esco;
        event->ret_win = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.w_esco;
        if (lc_env_ptr->link.Role == MASTER_ROLE)
        {
            event->rx_pkt_len = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.s2m_pkt_len;
            event->tx_pkt_len = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.m2s_pkt_len;
        }
        else
        {
            event->tx_pkt_len = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.s2m_pkt_len;
            event->rx_pkt_len = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.m2s_pkt_len;
        }
        lc_save_sco_chg_para(event);
        hci_send_2_host(event);
    }
    if (bt_sco_data_handle_fun)  // host callback function
    {
        bt_sco_callback_para_t sco_callback_para;
        sco_callback_para.handle_type = AUDIO_PATH_SCO_CHG_EVT;
        sco_callback_para.un_para.u32_value = lc_sco_env[sync_linkid].p_link_params->sync_type;
        bt_sco_data_handle_fun(&sco_callback_para);
    }
}


__STATIC void lc_sco_send_host_req(ke_task_id_t pid, uint8_t sync_linkid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

#if (EAVESDROPPING_SUPPORT)
    if (lm_env.hci.sco_link_request_delay != 0)
    {
        // send the custom HCI event
        struct hci_vs_sco_link_request_evt *event =
            KE_MSG_ALLOC(HCI_DBG_EVT, 0, 0, hci_vs_sco_link_request_evt);

        event->subcode = HCI_VS_SCO_LINK_REQUEST_EVT_SUBCODE;
        memcpy(&event->bd_addr.addr[0], &lc_env_ptr->info.BdAddr.addr[0], BD_ADDR_LEN);
        hci_send_2_host(event);
        // schedule the standard HCI event for later and translate ms to multiple of 10 ms
        ke_timer_set(LC_SCO_DELAY_HOST_REQ, pid, lm_env.hci.sco_link_request_delay / 10);
    }
    else
#endif // EAVESDROPPING_SUPPORT
    {
        // allocate the status event message
        struct hci_con_req_evt *event = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_CON_REQ_EVT_CODE, hci_con_req_evt);
        event->lk_type = lc_sco_env[sync_linkid].p_link_params->sync_type;
        memcpy(&event->bd_addr.addr[0], &lc_env_ptr->info.BdAddr.addr[0], BD_ADDR_LEN);
        memcpy(&event->classofdev.A[0], &lc_env_ptr->link.Class.A[0], DEV_CLASS_LEN);
        hci_send_2_host(event);

        // set the Connection Accept Timeout timer
        ke_timer_set(LC_CON_ACCEPT_TO, pid, 10 * co_slot_to_duration(hci_con_accept_to_get()));
    }
}


__STATIC void lc_sco_send_host_disc_evt(ke_task_id_t pid, uint8_t sync_linkid, uint8_t status, uint8_t reason)
{
    uint16_t conhdl = BT_SYNC_CONHDL(KE_IDX_GET(pid), sync_linkid);
    // Send HCI Disconnection Complete event
    struct hci_disc_cmp_evt *event = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_DISC_CMP_EVT_CODE, hci_disc_cmp_evt);
    event->status = status;
    event->reason = reason;
    event->conhdl = conhdl;
    hci_send_2_host(event);

    // Informs LC that a SCO link has been stopped
    lc_sco_ind(pid);
}


__STATIC void lc_sco_send_peer_req(ke_task_id_t pid, uint8_t sync_linkid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    uint8_t tr_id;

    if (lc_sco_env[sync_linkid].p_link_params->sync_type == ESCO_TYPE)
    {
        // send LMP_eScoLinkReq
        struct lmp_esco_link_req pdu;

        tr_id             = (lc_sco_env[sync_linkid].p_nego_params->nego_infos.initiator) ?
                            lc_env_ptr->link.Role : !lc_env_ptr->link.Role;
        pdu.opcode        = LMP_OPCODE(LMP_ESC4_OPCODE, tr_id);
        pdu.ext_opcode    = LMP_ESCO_LINK_REQ_EXTOPCODE;
        pdu.esco_lt_addr = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.esco_lt_addr;
        pdu.esco_hdl     = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.esco_hdl;
        pdu.flags        = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.flags;
        pdu.d_esco       = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.d_esco;
        pdu.t_esco       = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.t_esco;
        pdu.w_esco       = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.w_esco;
        pdu.m2s_pkt_type = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.m2s_pkt_type;
        pdu.s2m_pkt_type = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.s2m_pkt_type;
        pdu.m2s_pkt_len  = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.m2s_pkt_len;
        pdu.s2m_pkt_len  = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.s2m_pkt_len;
        pdu.nego_state   = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.nego_state;
        pdu.air_mode     = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.air_mode;
        lc_send_lmp(idx, &pdu);

        if (lc_env_ptr->link.Role == MASTER_ROLE)
        {
            lc_util_set_loc_trans_coll(idx, LMP_ESC4_OPCODE, LMP_ESCO_LINK_REQ_EXTOPCODE, LC_UTIL_INUSED);
        }
    }
    else
    {
        // send LMP_ScoLinkReq
        struct lmp_sco_link_req pdu;

        tr_id          = (lc_sco_env[sync_linkid].p_nego_params->nego_infos.initiator) ?
                         lc_env_ptr->link.Role : !lc_env_ptr->link.Role;
        pdu.opcode     = LMP_OPCODE(LMP_SCO_LINK_REQ_OPCODE, tr_id);
        pdu.sco_hdl    = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.esco_hdl;
        pdu.air_mode   = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.air_mode;
        pdu.d_sco      = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.d_esco;
        pdu.t_sco      = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.t_esco;
        pdu.flags      = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.flags;
        pdu.sco_pkt    = lc_sco_env[sync_linkid].p_nego_params->air_params_sent.m2s_pkt_type;
        lc_send_lmp(idx, &pdu);

        if (lc_env_ptr->link.Role == MASTER_ROLE)
        {
            lc_util_set_loc_trans_coll(idx, LMP_SCO_LINK_REQ_OPCODE, LC_UTIL_NOT_USED, LC_UTIL_INUSED);
        }
    }
}

__STATIC void lc_sco_send_peer_accept(ke_task_id_t pid, uint8_t sync_linkid)
{
    int idx = KE_IDX_GET(pid);


    if (lc_sco_env[sync_linkid].p_link_params->sync_type == ESCO_TYPE)
    {
        // LMP_Accepted_Ext(LMP_ESCO_LINK_REQ_EXTOPCODE)
        lc_send_pdu_acc_ext4(idx, LMP_ESCO_LINK_REQ_EXTOPCODE, lc_sco_env[sync_linkid].p_nego_params->air_params_received.tr_id);
    }
    else
    {
        // LMP_Accepted(LMP_SCO_LINK_REQ_OPCODE)
        lc_send_pdu_acc(idx, LMP_SCO_LINK_REQ_OPCODE, lc_sco_env[sync_linkid].p_nego_params->air_params_received.tr_id);
    }
}

__STATIC void lc_sco_send_peer_reject(ke_task_id_t pid, uint8_t reason, uint8_t sync_type, uint8_t tr_id)
{
    int idx = KE_IDX_GET(pid);


    if (sync_type == ESCO_TYPE)
    {
        // LMP_Not_Accepted_Ext(LMP_ESCO_LINK_REQ_EXTOPCODE)
        lc_send_pdu_not_acc_ext4(idx, LMP_ESCO_LINK_REQ_EXTOPCODE, reason, tr_id);
    }
    else
    {
        // LMP_Not_Accepted(LMP_SCO_LINK_REQ_OPCODE)
        lc_send_pdu_not_acc(idx, LMP_SCO_LINK_REQ_OPCODE, reason, tr_id);
    }
}

__STATIC void lc_sco_send_peer_disc(ke_task_id_t pid, uint8_t sync_linkid, uint8_t reason)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    if (lc_sco_env[sync_linkid].p_link_params->sync_type == SCO_TYPE)
    {
        //  send LMP_RemoveScoLink
        lc_send_pdu_sco_lk_rem_req(idx, lc_sco_env[sync_linkid].p_link_params->sync_hdl, reason, lc_env_ptr->link.Role);

        if (lc_env_ptr->link.Role == MASTER_ROLE)
        {
            lc_util_set_loc_trans_coll(idx, LMP_RMV_SCO_LINK_REQ_OPCODE, LC_UTIL_NOT_USED, LC_UTIL_INUSED);
        }
    }
    else
    {
        // send LMP_RemoveeScoLink
        lc_send_pdu_esco_lk_rem_req(idx, lc_sco_env[sync_linkid].p_link_params->sync_hdl, reason, lc_env_ptr->link.Role);

        if (lc_env_ptr->link.Role == MASTER_ROLE)
        {
            lc_util_set_loc_trans_coll(idx, LMP_RMV_ESCO_LINK_REQ_EXTOPCODE, LC_UTIL_NOT_USED, LC_UTIL_INUSED);
        }
    }
}

__STATIC void lc_sco_send_peer_accept_disc(ke_task_id_t pid, uint8_t sync_linkid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    if (lc_sco_env[sync_linkid].p_link_params->sync_type == ESCO_TYPE)
    {
        // LMP_Accepted_Ext(LMP_RMV_ESCO_LINK_REQ_EXTOPCODE)
        lc_send_pdu_acc_ext4(idx, LMP_RMV_ESCO_LINK_REQ_EXTOPCODE, !lc_env_ptr->link.Role);
    }
    else
    {
        // LMP_Accepted(LMP_RMV_SCO_LINK_REQ_OPCODE)
        lc_send_pdu_acc(idx, LMP_RMV_SCO_LINK_REQ_OPCODE,  !lc_env_ptr->link.Role);
    }
}

__STATIC void lc_sco_send_peer_reject_disc(ke_task_id_t pid, uint8_t reason, uint8_t sync_type)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    if (sync_type == ESCO_TYPE)
    {
        // LMP_Not_Accepted_Ext(LMP_RMV_ESCO_LINK_REQ_EXTOPCODE)
        lc_send_pdu_not_acc_ext4(idx, LMP_RMV_ESCO_LINK_REQ_EXTOPCODE, reason, !lc_env_ptr->link.Role);
    }
    else
    {
        // LMP_Not_Accepted(LMP_RMV_SCO_LINK_REQ_OPCODE)
        lc_send_pdu_not_acc(idx, LMP_RMV_SCO_LINK_REQ_OPCODE, reason, !lc_env_ptr->link.Role);
    }
}



__STATIC uint8_t lc_sco_wrapper_lm_add_sync(ke_task_id_t pid, uint8_t sync_linkid, uint8_t request)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    uint8_t status = CO_ERROR_NO_ERROR;

    // Get all pointers
    struct lc_sco_link_params_tag  *p_link    = lc_sco_env[sync_linkid].p_link_params;
    struct lc_sco_host_params_tag  *p_host    = &lc_sco_env[sync_linkid].p_nego_params->host_params;
    struct lc_sco_air_params_tag   *p_air_rx  = &lc_sco_env[sync_linkid].p_nego_params->air_params_received;
    struct lc_sco_air_params_tag   *p_air_tx  = &lc_sco_env[sync_linkid].p_nego_params->air_params_sent;

    uint16_t vx_set = 0x0040;
    switch (p_host->tx_cod_fmt[0])
    {
    case CODING_FORMAT_ULAW:
        vx_set += AIR_COD_MULAW;
        break;
    case CODING_FORMAT_ALAW:
        vx_set += AIR_COD_MULAW;
        break;
    case CODING_FORMAT_TRANSP:
        vx_set += AIR_COD_TRANS;
        break;
    case CODING_FORMAT_MSBC:
        vx_set += AIR_COD_TRANS;
        break;
    default:
        vx_set += AIR_COD_CVSD;
        break;
    }

    status = lm_add_sync(sync_linkid,
                         &lc_env_ptr->info.RemoteFeatures[FEATURE_PAGE_0],
                         idx,
                         lc_env_ptr->link.Role,
                         &p_link->sync_type,
                         p_host->tx_bw,
                         p_host->rx_bw,
                         p_host->max_lat,
                         vx_set,
                         p_host->retx_eff,
                         p_host->packet_type,
                         p_air_rx,
                         request);

    memcpy(p_air_tx, p_air_rx, sizeof(struct lc_sco_air_params_tag));

    return (status);
}
__STATIC uint8_t lc_sco_wrapper_lm_modif_sync(ke_task_id_t pid, uint8_t sync_linkid, uint8_t request)
{
    uint8_t status = CO_ERROR_NO_ERROR;

    // Get all pointers
    struct lc_sco_link_params_tag  *p_link    = lc_sco_env[sync_linkid].p_link_params;
    struct lc_sco_host_params_tag  *p_host    = &lc_sco_env[sync_linkid].p_nego_params->host_params;
    struct lc_sco_air_params_tag   *p_air_rx  = &lc_sco_env[sync_linkid].p_nego_params->air_params_received;
    struct lc_sco_air_params_tag   *p_air_tx  = &lc_sco_env[sync_linkid].p_nego_params->air_params_sent;

    uint16_t vx_set = 0x0040;
    switch (p_host->tx_cod_fmt[0])
    {
    case CODING_FORMAT_ULAW:
        vx_set += AIR_COD_MULAW;
        break;
    case CODING_FORMAT_ALAW:
        vx_set += AIR_COD_MULAW;
        break;
    case CODING_FORMAT_TRANSP:
        vx_set += AIR_COD_TRANS;
        break;
    case CODING_FORMAT_MSBC:
        vx_set += AIR_COD_TRANS;
        break;
    default:
        vx_set += AIR_COD_CVSD;
        break;
    }

    status = lm_modif_sync(sync_linkid,
                           p_link->sync_type,
                           p_host->tx_bw,
                           p_host->rx_bw,
                           p_host->max_lat,
                           vx_set,
                           p_host->retx_eff,
                           p_host->packet_type,
                           p_air_rx,
                           request);

    memcpy(p_air_tx, p_air_rx, sizeof(struct lc_sco_air_params_tag));

    return (status);
}
__STATIC uint8_t lc_sco_wrapper_lm_check_sync_hl_rsp(ke_task_id_t pid, uint8_t sync_linkid)
{
    uint8_t status = CO_ERROR_NO_ERROR;

    // Get all pointers
    struct lc_sco_host_params_tag  *p_host    = &lc_sco_env[sync_linkid].p_nego_params->host_params;
    struct lc_sco_air_params_tag   *p_air_rx  = &lc_sco_env[sync_linkid].p_nego_params->air_params_received;
    struct lc_sco_air_params_tag   *p_air_tx  = &lc_sco_env[sync_linkid].p_nego_params->air_params_sent;

    uint16_t vx_set = 0x0040;
    switch (p_host->tx_cod_fmt[0])
    {
    case CODING_FORMAT_ULAW:
        vx_set += AIR_COD_MULAW;
        break;
    case CODING_FORMAT_ALAW:
        vx_set += AIR_COD_MULAW;
        break;
    case CODING_FORMAT_TRANSP:
        vx_set += AIR_COD_TRANS;
        break;
    case CODING_FORMAT_MSBC:
        vx_set += AIR_COD_TRANS;
        break;
    default:
        vx_set += AIR_COD_CVSD;
        break;
    }

    status = lm_check_sync_hl_rsp(sync_linkid,
                                  p_host->tx_bw,
                                  p_host->rx_bw,
                                  p_host->max_lat,
                                  vx_set,
                                  p_host->retx_eff,
                                  p_host->packet_type,
                                  p_air_rx);

    memcpy(p_air_tx, p_air_rx, sizeof(struct lc_sco_air_params_tag));

    return (status);
}
__STATIC void lc_sco_wrapper_lm_remove_sync(ke_task_id_t pid, uint8_t sync_linkid)
{
    lm_remove_sync(sync_linkid);
}
__STATIC uint8_t lc_sco_wrapper_lm_get_sync_param(ke_task_id_t pid, uint8_t sync_linkid, uint8_t request)
{
    uint8_t status = CO_ERROR_NO_ERROR;

    // Get all pointers
    struct lc_sco_air_params_tag   *p_air_rx  = &lc_sco_env[sync_linkid].p_nego_params->air_params_received;
    struct lc_sco_air_params_tag   *p_air_tx  = &lc_sco_env[sync_linkid].p_nego_params->air_params_sent;

    status = lm_get_sync_param(sync_linkid, p_air_rx, request);

    memcpy(p_air_tx, p_air_rx, sizeof(struct lc_sco_air_params_tag));

    return (status);
}


/*
 * EXPORTED FUNCTIONS DEFINITION
 *****************************************************************************************
 */

void lc_sco_init(void)
{
    uint8_t length;

    // Clear LC SCO environment structure
    memset(&lc_sco_env, 0, sizeof(lc_sco_env));

    // Try to get the voice mode from NVDS
    length = sizeof(lc_sco_data_path_config);
    if (rwip_param.get(PARAM_ID_SYNC_CONFIG, &length, (uint8_t *)&lc_sco_data_path_config) != PARAM_OK)
    {
        lc_sco_data_path_config = (AUDIO_DATA_PATH_HCI    << LINK1_CFG_SFT) |
                                  (AUDIO_DATA_PATH_HCI    << LINK2_CFG_SFT) ;
    }
#if BT_SCO_CALLBACK_TEST
    bt_sco_data_handle_register(bt_sco_data_handle_callback);
#endif
}

void lc_sco_reset(void)
{
    uint8_t sync_linkid = 0;

    // Parse all SCO links
    for (sync_linkid = 0; sync_linkid < MAX_NB_SYNC ; sync_linkid++)
    {
        // Release memory buffers if needed
        if (lc_sco_env[sync_linkid].p_link_params != NULL)
        {
            ke_free(lc_sco_env[sync_linkid].p_link_params);
        }
        if (lc_sco_env[sync_linkid].p_nego_params != NULL)
        {
            ke_free(lc_sco_env[sync_linkid].p_nego_params);
        }
    }

    // Initialize module
    lc_sco_init();
}


uint8_t lc_sco_detach(ke_task_id_t pid, uint8_t reason)
{
    int idx = KE_IDX_GET(pid);
    uint8_t sync_linkid = 0;
    uint8_t status = CO_ERROR_UNKNOWN_CONNECTION_ID;

    // Find if there is a SCO link to disconnect
    for (sync_linkid = 0; sync_linkid < MAX_NB_SYNC ; sync_linkid++)
    {
        // Check if sco link ID corresponds to current ACL link
        if (lc_sco_env[sync_linkid].p_link_params != NULL)
        {
            if (lc_sco_env[sync_linkid].p_link_params->link_id == idx)
                break;
        }

    }

    if (sync_linkid < MAX_NB_SYNC)
    {
        ASSERT_ERR(lc_sco_env[sync_linkid].p_nego_params == NULL);

        switch (lc_sco_env[sync_linkid].p_link_params->state)
        {
        case LC_SCO_CONNECTED:
        {
            if (!lc_sco_env[sync_linkid].p_link_params->hv1_flag)
            {
                // Disconnect existing link
                lc_sco_stop_link(pid, sync_linkid);
            }

            // Save disconnection reason
            lc_sco_env[sync_linkid].p_link_params->disc_reason = reason;

            // Send disc to peer
            lc_sco_send_peer_disc(pid, sync_linkid, reason);

            // Start timeout
            lc_start_lmp_to(pid);

            // Set SCO negotiation state machine
            lc_sco_env[sync_linkid].p_link_params->state = LC_SCO_NEGO_WAIT_PEER_DISC_RSP;

            // Set LC task state machine
            ke_state_set(pid, LC_SCO_DISC_ONGOING);

            status = CO_ERROR_NO_ERROR;
        }
        break;
        default :
        {
            ASSERT_INFO_FORCE(0, sync_linkid, lc_sco_env[sync_linkid].p_link_params->state);
        }
        break;
        }
    }

    return status;
}


void lc_sco_release(ke_task_id_t pid, uint8_t reason)
{
    int idx = KE_IDX_GET(pid);
    uint8_t sync_linkid = 0;

    // Find if there is a SCO link to disconnect
    for (sync_linkid = 0; sync_linkid < MAX_NB_SYNC ; sync_linkid++)
    {
        // Check if sco link ID corresponds to current ACL link
        if (lc_sco_env[sync_linkid].p_link_params != NULL)
        {
            if (lc_sco_env[sync_linkid].p_link_params->link_id == idx)
            {
                if (lc_sco_env[sync_linkid].p_nego_params == NULL)
                {
                    if ((lc_sco_env[sync_linkid].p_link_params->state != LC_SCO_NEGO_WAIT_PEER_DISC_RSP)
                            && (lc_sco_env[sync_linkid].p_link_params->state != LC_SCO_NEGO_WAIT_PEER_DISC_CFM))
                    {
                        // Disconnect existing link
                        lc_sco_stop_link(pid, sync_linkid);
                    }

                    // Inform Host that SCO is disconnected
                    lc_sco_send_host_disc_evt(pid, sync_linkid, CO_ERROR_NO_ERROR, reason);

                    // Release the link buffer
                    lc_sco_free_link(pid, sync_linkid);
                }
                else
                {
                    // Inform Host that SCO setup is completed
                    lc_sco_send_host_cmp_evt(pid, sync_linkid, reason);

                    // Free negotiation parameters
                    lc_sco_free_nego(pid, sync_linkid);

                    // Free link parameters
                    lc_sco_free_link(pid, sync_linkid);
                }
            }
        }

    }
}


uint8_t lc_sco_host_request(ke_task_id_t pid, bool old_style, bool host_initiated, uint16_t conhdl, struct lc_sco_host_params_tag *p_host_params)
{
    int idx = KE_IDX_GET(pid);
    uint8_t status = CO_ERROR_NO_ERROR;
    uint8_t sync_linkid = 0;

    // Check if new SCO link is requested
    if (BT_ACL_CONHDL_MIN + idx == conhdl)
    {
        // New link request from Host

        // Try to create a new link structure
        sync_linkid = lc_sco_alloc_link(pid);

        if (sync_linkid < MAX_NB_SYNC)
        {
            // Allocate buffer for negotiation parameters
            lc_sco_alloc_nego(pid, sync_linkid);

            // Set initiator flag to true
            lc_sco_env[sync_linkid].p_nego_params->nego_infos.initiator = true;
            // Set old style flag
            lc_sco_env[sync_linkid].p_nego_params->nego_infos.old_style_hci = old_style;
            // Set host_initiated flag
            lc_sco_env[sync_linkid].p_nego_params->nego_infos.host_initiated = host_initiated;

            // Store Host parameters
            memcpy((uint8_t *) &lc_sco_env[sync_linkid].p_nego_params->host_params, (uint8_t *)p_host_params, sizeof(struct lc_sco_host_params_tag));

            // Input Data Path / Output Data Path (old HCI commands)
            if (old_style)
            {
                // Get the data path from static configuration
                lc_sco_env[sync_linkid].p_nego_params->host_params.in_data_path = ((lc_sco_data_path_config >> (4 * sync_linkid)) & 0xF);
                lc_sco_env[sync_linkid].p_nego_params->host_params.out_data_path = ((lc_sco_data_path_config >> (4 * sync_linkid)) & 0xF);
            }

            status = lc_sco_wrapper_lm_add_sync(pid, sync_linkid, SYNC_HL_REQ);

            if (status == CO_ERROR_NO_ERROR)
            {
                // Send request to peer
                lc_sco_send_peer_req(pid, sync_linkid);

                // Start timeout
                lc_start_lmp_to(pid);

                // Set SCO negotiation state machine
                lc_sco_env[sync_linkid].p_link_params->state = LC_SCO_NEGO_WAIT_PEER_RSP;
            }
            else
            {
                // Free negotiation parameters
                lc_sco_free_nego(pid, sync_linkid);

                // Free link parameters
                lc_sco_free_link(pid, sync_linkid);
            }
        }
        else
        {
            status = CO_ERROR_SYNC_CON_LIMIT_DEV_EXCEED;
        }
    }
    else
    {
        // Find the SCO link to re-negotiate
        for (sync_linkid = 0; sync_linkid < MAX_NB_SYNC ; sync_linkid++)
        {
            // Check if sco link ID is free
            if (lc_sco_env[sync_linkid].p_link_params != NULL)
            {
                if (BT_SYNC_CONHDL(idx, sync_linkid) == conhdl)
                    break;
            }

        }


        if (sync_linkid < MAX_NB_SYNC)
        {
            struct lc_sco_air_params_tag *params;
            ASSERT_ERR(lc_sco_env[sync_linkid].p_nego_params == NULL);

            // Modification request from Host

            // Allocate buffer for negotiation parameters
            lc_sco_alloc_nego(pid, sync_linkid);

            params = &lc_sco_env[sync_linkid].p_nego_params->air_params_sent;

            // Set initiator flag to false
            lc_sco_env[sync_linkid].p_nego_params->nego_infos.initiator = true;
            // Set old style flag to false
            lc_sco_env[sync_linkid].p_nego_params->nego_infos.old_style_hci = old_style;
            // Set host_initiated flag
            lc_sco_env[sync_linkid].p_nego_params->nego_infos.host_initiated = host_initiated;

            // Store Host parameters
            memcpy((uint8_t *) &lc_sco_env[sync_linkid].p_nego_params->host_params, (uint8_t *)p_host_params, sizeof(struct lc_sco_host_params_tag));


            status =  lc_sco_wrapper_lm_modif_sync(pid, sync_linkid, SYNC_HL_MODIF);

            // If the Host initiates the SCO update or device auto-initiates a SCO move with a different offset
            if ((status == CO_ERROR_NO_ERROR)
                    && (host_initiated
                        || (lc_sco_env[sync_linkid].p_link_params->plan_elt.offset != lc_offset_lmp_to_local(2 * params->d_esco, 2 * params->t_esco, ld_read_clock(), ld_acl_clock_offset_get(idx).hs, ((params->flags & INIT2_FLAG) != 0)))))
            {
                // Send request to peer
                lc_sco_send_peer_req(pid, sync_linkid);

                // Start timeout
                lc_start_lmp_to(pid);

                // Set SCO negotiation state machine
                lc_sco_env[sync_linkid].p_link_params->state = LC_SCO_NEGO_WAIT_PEER_MOD_RSP;
            }
            else
            {
                // Free negotiation parameters
                lc_sco_free_nego(pid, sync_linkid);
            }
        }
        else
        {
            status = CO_ERROR_UNKNOWN_CONNECTION_ID;
        }
    }

    return (status);
}


uint8_t lc_sco_host_accept(ke_task_id_t pid, uint16_t opcode, struct lc_sco_host_params_tag *p_host_params)
{
    uint8_t action = LC_SCO_ACCEPT;
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    uint8_t status = CO_ERROR_NO_ERROR;
    uint8_t sync_linkid = 0;

    // Find the SCO link under negotiation
    for (sync_linkid = 0; sync_linkid < MAX_NB_SYNC ; sync_linkid++)
    {
        // Check if sco link ID is free
        if (lc_sco_env[sync_linkid].p_link_params != NULL && lc_sco_env[sync_linkid].p_nego_params != NULL)
        {
            if (lc_sco_env[sync_linkid].p_link_params->link_id == idx)
                break;
        }

    }


    if (sync_linkid < MAX_NB_SYNC)
    {
        switch (lc_sco_env[sync_linkid].p_link_params->state)
        {
        case LC_SCO_NEGO_WAIT_HOST_RSP:
        {
            bool old_style = (opcode == HCI_ACCEPT_CON_REQ_CMD_OPCODE) || (opcode == HCI_ACCEPT_SYNC_CON_REQ_CMD_OPCODE);
            bool very_old_style = (opcode == HCI_ACCEPT_CON_REQ_CMD_OPCODE);

            // Clear the Connection Accept Timeout timer
            ke_timer_clear(LC_CON_ACCEPT_TO, pid);

            // Save old style flag
            lc_sco_env[sync_linkid].p_nego_params->nego_infos.old_style_hci = old_style;
            // Save very old style flag
            lc_sco_env[sync_linkid].p_nego_params->nego_infos.very_old_style_hci = very_old_style;

            // Store Host parameters
            memcpy((uint8_t *) &lc_sco_env[sync_linkid].p_nego_params->host_params, (uint8_t *)p_host_params, sizeof(struct lc_sco_host_params_tag));

            // Input Data Path / Output Data Path (old HCI commands)
            if (old_style)
            {
                // Get the data path from static configuration
                lc_sco_env[sync_linkid].p_nego_params->host_params.in_data_path = ((lc_sco_data_path_config >> (4 * sync_linkid)) & 0xF);
                lc_sco_env[sync_linkid].p_nego_params->host_params.out_data_path = ((lc_sco_data_path_config >> (4 * sync_linkid)) & 0xF);
            }

            // Parameters checking
            do
            {
                // Check HCI command parameters
                status =  lc_sco_wrapper_lm_check_sync_hl_rsp(pid, sync_linkid);

                // Send command status event
                lc_cmd_stat_send(opcode, status);

                if (status != CO_ERROR_NO_ERROR)
                {
                    // Parameters rejected
                    action = LC_SCO_REJECT;
                    break;
                }

                if (lc_env_ptr->link.Role == MASTER_ROLE || lc_sco_env[sync_linkid].p_nego_params->air_params_received.nego_state != ESCO_NEGO_INIT)
                {
                    // Continue negotiation
                    action = LC_SCO_REQUEST;
                    break;
                }

            }
            while (0);

            // Perform result action
            switch (action)
            {
            case LC_SCO_ACCEPT:
            {
                // Send accept to peer
                lc_sco_send_peer_accept(pid, sync_linkid);

                // Start timeout
                lc_start_lmp_to(pid);

                // Set SCO negotiation state machine
                lc_sco_env[sync_linkid].p_link_params->state = LC_SCO_NEGO_WAIT_PEER_CFM;
            }
            break;
            case LC_SCO_REJECT:
            {
                // Send reject to peer
                lc_sco_send_peer_reject(pid, status, lc_sco_env[sync_linkid].p_link_params->sync_type, lc_sco_env[sync_linkid].p_nego_params->air_params_received.tr_id);

                // End of link negotiation
                lc_sco_free_nego(pid, sync_linkid);

                // Release link buffer
                lc_sco_free_link(pid, sync_linkid);
            }
            break;
            case LC_SCO_REQUEST:
            {
                // Send request to peer
                lc_sco_send_peer_req(pid, sync_linkid);

                // Start timeout
                lc_start_lmp_to(pid);

                // Set SCO negotiation state machine
                lc_sco_env[sync_linkid].p_link_params->state = LC_SCO_NEGO_WAIT_PEER_RSP;
            }
            break;
            default:
            {
                // Nothing
            }
            break;
            }
        }
        break;
        default:
        {
            ASSERT_INFO_FORCE(0, sync_linkid, lc_sco_env[sync_linkid].p_link_params->state);
        }
        break;
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    return (status);
}


void lc_sco_host_reject(ke_task_id_t pid, bool very_old_style, uint8_t status)
{
    int idx = KE_IDX_GET(pid);
    uint8_t sync_linkid = 0;

    // Find the SCO link under negotiation
    for (sync_linkid = 0; sync_linkid < MAX_NB_SYNC ; sync_linkid++)
    {
        // Check if sco link ID is free
        if (lc_sco_env[sync_linkid].p_link_params != NULL && lc_sco_env[sync_linkid].p_nego_params != NULL)
        {
            if (lc_sco_env[sync_linkid].p_link_params->link_id == idx)
                break;
        }

    }


    if (sync_linkid < MAX_NB_SYNC)
    {
        switch (lc_sco_env[sync_linkid].p_link_params->state)
        {
        case LC_SCO_NEGO_WAIT_HOST_RSP:
        {
            // Clear the Connection Accept Timeout timer
            ke_timer_clear(LC_CON_ACCEPT_TO, pid);

            // Save old style flag
            lc_sco_env[sync_linkid].p_nego_params->nego_infos.very_old_style_hci = very_old_style;

            // Send reject to peer
            lc_sco_send_peer_reject(pid, status, lc_sco_env[sync_linkid].p_link_params->sync_type, lc_sco_env[sync_linkid].p_nego_params->air_params_received.tr_id);

            // Inform Host that SCO setup is completed
            lc_sco_send_host_cmp_evt(pid, sync_linkid, status);

            // End of link negotiation
            lc_sco_free_nego(pid, sync_linkid);

            // Release link buffer
            lc_sco_free_link(pid, sync_linkid);
        }
        break;
        default:
        {
            ASSERT_INFO_FORCE(0, sync_linkid, lc_sco_env[sync_linkid].p_link_params->state);
        }
        break;
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }
}


uint8_t lc_sco_host_request_disc(ke_task_id_t pid, uint16_t conhdl, uint8_t reason)
{
    int idx = KE_IDX_GET(pid);
    uint8_t status = CO_ERROR_NO_ERROR;
    uint8_t sync_linkid = 0;

    // Find the SCO link under to disconnect
    for (sync_linkid = 0; sync_linkid < MAX_NB_SYNC ; sync_linkid++)
    {
        // Check if sco link ID is free
        if (lc_sco_env[sync_linkid].p_link_params != NULL)
        {
            if (BT_SYNC_CONHDL(idx, sync_linkid) == conhdl)
                break;
        }
    }

    if (sync_linkid < MAX_NB_SYNC)
    {
        ASSERT_ERR(lc_sco_env[sync_linkid].p_nego_params == NULL);

        switch (lc_sco_env[sync_linkid].p_link_params->state)
        {
        case LC_SCO_CONNECTED:
        {
            if (!lc_sco_env[sync_linkid].p_link_params->hv1_flag)
            {
                // Disconnect existing link
                lc_sco_stop_link(pid, sync_linkid);
            }

            // Send disc to peer
            lc_sco_send_peer_disc(pid, sync_linkid, reason);

            // Start timeout
            lc_start_lmp_to(pid);

            // Set SCO negotiation state machine
            lc_sco_env[sync_linkid].p_link_params->state = LC_SCO_NEGO_WAIT_PEER_DISC_RSP;

            // Set LC task state machine
            ke_state_set(pid, LC_SCO_DISC_ONGOING);
        }
        break;
        default :
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
        }
        break;
        }
    }
    else
    {
        status = CO_ERROR_UNKNOWN_CONNECTION_ID;
    }

    return (status);
}



void lc_sco_peer_request(ke_task_id_t pid, uint8_t req_type, struct lc_sco_air_params_tag *p_peer_params)
{
    uint8_t action = LC_SCO_ACCEPT;
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    uint8_t sync_linkid = 0;
    uint8_t status = CO_ERROR_NO_ERROR;

    // Find the SCO link corresponding to SCO handle
    for (sync_linkid = 0; sync_linkid < MAX_NB_SYNC ; sync_linkid++)
    {
        // Check if sco link ID is free
        if (lc_sco_env[sync_linkid].p_link_params != NULL)
        {
            if ((lc_sco_env[sync_linkid].p_link_params->link_id == idx) &&
                    ((lc_sco_env[sync_linkid].p_nego_params != NULL) || (lc_sco_env[sync_linkid].p_link_params->sync_hdl == p_peer_params->esco_hdl)))
                break;
        }

    }

    if (sync_linkid < MAX_NB_SYNC)
    {
        switch (lc_sco_env[sync_linkid].p_link_params->state)
        {
        case LC_SCO_CONNECTED:
        {
            // Modification request from peer

            // Allocate buffer for negotiation parameters
            lc_sco_alloc_nego(pid, sync_linkid);

            // Store Peer parameters
            memcpy((uint8_t *) &lc_sco_env[sync_linkid].p_nego_params->air_params_received,
                   (uint8_t *) p_peer_params,
                   sizeof(struct lc_sco_air_params_tag));


            status =  lc_sco_wrapper_lm_modif_sync(pid, sync_linkid, SYNC_PEER_MODIF);

            if (status == CO_ERROR_NO_ERROR)
            {
                if (((lc_env_ptr->link.Role == SLAVE_ROLE) && (lc_sco_env[sync_linkid].p_link_params->sync_type == SCO_TYPE)) ||
                        ((lc_sco_env[sync_linkid].p_nego_params->air_params_received.nego_state == ESCO_NEGO_INIT)   && (lc_sco_env[sync_linkid].p_link_params->sync_type == ESCO_TYPE)))
                {
                    // Send accept to peer
                    lc_sco_send_peer_accept(pid, sync_linkid);

                    // Start timeout
                    lc_start_lmp_to(pid);

                    // Set SCO negotiation state machine
                    lc_sco_env[sync_linkid].p_link_params->state = LC_SCO_NEGO_WAIT_PEER_MOD_CFM;
                }
                else
                {
                    // Request other params
                    lc_sco_send_peer_req(pid, sync_linkid);

                    // Start timeout
                    lc_start_lmp_to(pid);

                    // Set SCO negotiation state machine
                    lc_sco_env[sync_linkid].p_link_params->state = LC_SCO_NEGO_WAIT_PEER_MOD_RSP;
                }
            }
            else
            {
                // Send reject to peer
                lc_sco_send_peer_reject(pid, status, req_type, p_peer_params->tr_id);

                // End of link negotiation
                lc_sco_free_nego(pid, sync_linkid);
            }
        }
        break;
        case LC_SCO_NEGO_WAIT_PEER_RSP:
        {
            // Clear timeout
            ke_timer_clear(LC_LMP_RSP_TO, pid);

            // Store Peer parameters
            memcpy((uint8_t *) &lc_sco_env[sync_linkid].p_nego_params->air_params_received,
                   (uint8_t *) p_peer_params,
                   sizeof(struct lc_sco_air_params_tag));


            // Parameters checking
            do
            {
                // Compute the ESCO parameters
                status = lc_sco_wrapper_lm_get_sync_param(pid, sync_linkid, SYNC_PEER_RSP);

                if (status != CO_ERROR_NO_ERROR)
                {
                    // Parameters rejected
                    action = LC_SCO_REJECT;
                    break;
                }

                if (lc_sco_env[sync_linkid].p_nego_params->air_params_received.nego_state != ESCO_NEGO_INIT)
                {
                    // Continue negotiation
                    action = LC_SCO_REQUEST;
                    break;
                }

            }
            while (0);

            // Perform result action
            switch (action)
            {
            case LC_SCO_ACCEPT:
            {
                // Send accept to peer
                lc_sco_send_peer_accept(pid, sync_linkid);

                // Start timeout
                lc_start_lmp_to(pid);

                // Set SCO negotiation state machine
                lc_sco_env[sync_linkid].p_link_params->state = LC_SCO_NEGO_WAIT_PEER_CFM;
            }
            break;
            case LC_SCO_REJECT:
            {
                // Send reject to peer
                lc_sco_send_peer_reject(pid, status, req_type, p_peer_params->tr_id);

                // Inform Host that SCO setup is completed
                lc_sco_send_host_cmp_evt(pid, sync_linkid, status);

                // End of link negotiation
                lc_sco_free_nego(pid, sync_linkid);

                // Release link buffer
                lc_sco_free_link(pid, sync_linkid);
            }
            break;
            case LC_SCO_REQUEST:
            {
                // Request other params
                lc_sco_send_peer_req(pid, sync_linkid);

                // Start timeout
                lc_start_lmp_to(pid);

                // Set SCO negotiation state machine
                lc_sco_env[sync_linkid].p_link_params->state = LC_SCO_NEGO_WAIT_PEER_RSP;
            }
            break;
            default:
            {
                // Nothing
            }
            break;
            }

        }
        break;
        case LC_SCO_NEGO_WAIT_PEER_MOD_RSP:
        {
            // Clear timeout
            ke_timer_clear(LC_LMP_RSP_TO, pid);

            // Store Peer parameters
            memcpy((uint8_t *) &lc_sco_env[sync_linkid].p_nego_params->air_params_received,
                   (uint8_t *) p_peer_params,
                   sizeof(struct lc_sco_air_params_tag));


            // Compute the ESCO parameters
            status = lc_sco_wrapper_lm_get_sync_param(pid, sync_linkid, SYNC_PEER_MODIF);


            if (lc_sco_env[sync_linkid].p_nego_params->air_params_received.nego_state == ESCO_NEGO_INIT)
            {

                // Send accept to peer
                lc_sco_send_peer_accept(pid, sync_linkid);

                // Start timeout
                lc_start_lmp_to(pid);

                // Set SCO negotiation state machine
                lc_sco_env[sync_linkid].p_link_params->state = LC_SCO_NEGO_WAIT_PEER_MOD_CFM;
            }
            else
            {
                // Request other params
                lc_sco_send_peer_req(pid, sync_linkid);

                // Start timeout
                lc_start_lmp_to(pid);

                // Set SCO negotiation state machine
                lc_sco_env[sync_linkid].p_link_params->state = LC_SCO_NEGO_WAIT_PEER_MOD_RSP;
            }
        }
        break;
        default:
        {
            ASSERT_INFO_FORCE(0, sync_linkid, lc_sco_env[sync_linkid].p_link_params->state);
        }
        break;
        }
    }
    else
    {
        // New link request from peer

        // Try to create a new link structure
        sync_linkid = lc_sco_alloc_link(pid);

        if (sync_linkid < MAX_NB_SYNC)
        {
            // Allocate buffer for negotiation parameters
            lc_sco_alloc_nego(pid, sync_linkid);

            // Save the request type (SCO / ESCO)
            lc_sco_env[sync_linkid].p_link_params->sync_type = req_type;

            // Store Peer parameters
            memcpy((uint8_t *) &lc_sco_env[sync_linkid].p_nego_params->air_params_received,
                   (uint8_t *) p_peer_params,
                   sizeof(struct lc_sco_air_params_tag));


            status = lc_sco_wrapper_lm_add_sync(pid, sync_linkid, SYNC_PEER_REQ);

            // If the sync type is eSCO and the offset or interval has been rejected, continue the negotiation
            if ((status == CO_ERROR_NO_ERROR) || ((lc_sco_env[sync_linkid].p_link_params->sync_type == ESCO_TYPE) && ((status == CO_ERROR_SCO_OFFSET_REJECTED) || (status == CO_ERROR_SCO_INTERVAL_REJECTED))))
            {
                // Send request to host
                lc_sco_send_host_req(pid, sync_linkid);

                // Set SCO negotiation state machine
                lc_sco_env[sync_linkid].p_link_params->state = LC_SCO_NEGO_WAIT_HOST_RSP;
            }
            else
            {
                // Send reject to peer
                lc_sco_send_peer_reject(pid, status, req_type, p_peer_params->tr_id);

                // End of link negotiation
                lc_sco_free_nego(pid, sync_linkid);

                // Release link buffer
                lc_sco_free_link(pid, sync_linkid);
            }
        }
        else
        {
            // Send reject to peer
            lc_sco_send_peer_reject(pid, CO_ERROR_SYNC_CON_LIMIT_DEV_EXCEED, req_type, p_peer_params->tr_id);
        }
    }
}


void lc_sco_peer_accept(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    uint8_t sync_linkid = 0;

    // Find the SCO link under negotiation
    for (sync_linkid = 0; sync_linkid < MAX_NB_SYNC ; sync_linkid++)
    {
        // Check if sco link ID is free
        if (lc_sco_env[sync_linkid].p_link_params != NULL && lc_sco_env[sync_linkid].p_nego_params != NULL)
        {
            if (lc_sco_env[sync_linkid].p_link_params->link_id == idx)
                break;
        }

    }


    if (sync_linkid < MAX_NB_SYNC)
    {
        switch (lc_sco_env[sync_linkid].p_link_params->state)
        {
        case LC_SCO_NEGO_WAIT_PEER_RSP:
        {
            // Clear timeout
            ke_timer_clear(LC_LMP_RSP_TO, pid);

            lc_sco_wrapper_lm_get_sync_param(pid, sync_linkid, SYNC_UPDATE);

            // Start the SCO link
            lc_sco_start_link(pid, sync_linkid);
        }
        break;
        case LC_SCO_NEGO_WAIT_PEER_MOD_RSP:
        {
            // Clear timeout
            ke_timer_clear(LC_LMP_RSP_TO, pid);

            lc_sco_wrapper_lm_get_sync_param(pid, sync_linkid, SYNC_UPDATE);

            // Modify the SCO link
            lc_sco_modify_link(pid, sync_linkid);
        }
        break;
        default:
        {
            ASSERT_INFO_FORCE(0, sync_linkid, lc_sco_env[sync_linkid].p_link_params->state);
        }
        break;
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }
}


void lc_sco_peer_reject(ke_task_id_t pid, uint8_t reason)
{
    int idx = KE_IDX_GET(pid);
    uint8_t sync_linkid = 0;

    // Find the SCO link under negotiation
    for (sync_linkid = 0; sync_linkid < MAX_NB_SYNC ; sync_linkid++)
    {
        // Check if sco link ID is free
        if (lc_sco_env[sync_linkid].p_link_params != NULL && lc_sco_env[sync_linkid].p_nego_params != NULL)
        {
            if (lc_sco_env[sync_linkid].p_link_params->link_id == idx)
                break;
        }

    }


    if (sync_linkid < MAX_NB_SYNC)
    {
        switch (lc_sco_env[sync_linkid].p_link_params->state)
        {
        case LC_SCO_NEGO_WAIT_PEER_RSP:
        {
            // Clear timeout
            ke_timer_clear(LC_LMP_RSP_TO, pid);

            // Inform Host that SCO setup is completed
            lc_sco_send_host_cmp_evt(pid, sync_linkid, reason);

            // End of link negotiation
            lc_sco_free_nego(pid, sync_linkid);

            // Release link buffer
            lc_sco_free_link(pid, sync_linkid);
        }
        break;
        case LC_SCO_NEGO_WAIT_PEER_MOD_RSP:
        {
            // Clear timeout
            ke_timer_clear(LC_LMP_RSP_TO, pid);

            if (lc_sco_env[sync_linkid].p_nego_params->nego_infos.initiator)
            {
                // Inform Host that SCO change has failed
                lc_sco_send_host_chg_evt(pid, sync_linkid, reason);
            }

            // End of link negotiation
            lc_sco_free_nego(pid, sync_linkid);

            // Set SCO negotiation state machine
            lc_sco_env[sync_linkid].p_link_params->state = LC_SCO_CONNECTED;
        }
        break;
        default:
        {
            ASSERT_INFO_FORCE(0, sync_linkid, lc_sco_env[sync_linkid].p_link_params->state);
        }
        break;
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }
}


void lc_sco_peer_request_disc(ke_task_id_t pid, uint8_t req_type, uint8_t scohdl, uint8_t reason)
{
    uint8_t sync_linkid = 0;

    // Find the SCO link to disconnect
    for (sync_linkid = 0; sync_linkid < MAX_NB_SYNC; sync_linkid++)
    {
        // Check if sco link ID is free
        if (lc_sco_env[sync_linkid].p_link_params != NULL)
        {
            if (lc_sco_env[sync_linkid].p_link_params->sync_hdl == scohdl)
                break;
        }
    }


    if (sync_linkid < MAX_NB_SYNC)
    {
        ASSERT_ERR(lc_sco_env[sync_linkid].p_nego_params == NULL);

        switch (lc_sco_env[sync_linkid].p_link_params->state)
        {
        case LC_SCO_CONNECTED:
        {
            if (!lc_sco_env[sync_linkid].p_link_params->hv1_flag)
            {
                // Disconnect existing link
                lc_sco_stop_link(pid, sync_linkid);
            }

            // Save disconnection reason
            lc_sco_env[sync_linkid].p_link_params->disc_reason = reason;

            // Send accept to peer
            lc_sco_send_peer_accept_disc(pid, sync_linkid);

            // Start timeout
            lc_start_lmp_to(pid);

            // Set SCO negotiation state machine
            lc_sco_env[sync_linkid].p_link_params->state = LC_SCO_NEGO_WAIT_PEER_DISC_CFM;

            // Set LC task state machine
            ke_state_set(pid, LC_SCO_DISC_ONGOING);
        }
        break;
        default :
        {
            ASSERT_INFO_FORCE(0, sync_linkid, lc_sco_env[sync_linkid].p_link_params->state);
        }
        break;
        }
    }
    else
    {
        lc_sco_send_peer_reject_disc(pid, CO_ERROR_UNKNOWN_CONNECTION_ID, req_type);
    }
}


void lc_sco_peer_accept_disc(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    uint8_t sync_linkid = 0;

    // Find the SCO link under disconnection
    for (sync_linkid = 0; sync_linkid < MAX_NB_SYNC ; sync_linkid++)
    {
        // Check if sco link ID is free
        if (lc_sco_env[sync_linkid].p_link_params != NULL)
        {
            if ((lc_sco_env[sync_linkid].p_link_params->link_id == idx) &&
                    (lc_sco_env[sync_linkid].p_link_params->state == LC_SCO_NEGO_WAIT_PEER_DISC_RSP))
                break;
        }

    }


    if (sync_linkid < MAX_NB_SYNC)
    {
        switch (lc_sco_env[sync_linkid].p_link_params->state)
        {
        case LC_SCO_NEGO_WAIT_PEER_DISC_RSP:
        {
            // Clear timeout
            ke_timer_clear(LC_LMP_RSP_TO, pid);

            if (lc_sco_env[sync_linkid].p_link_params->hv1_flag)
            {
                // Disconnect existing link
                lc_sco_stop_link(pid, sync_linkid);
            }

            // Inform Host that SCO is disconnected
            lc_sco_send_host_disc_evt(pid, sync_linkid, CO_ERROR_NO_ERROR, CO_ERROR_CON_TERM_BY_LOCAL_HOST);

            // Clear LC task state machine
            ke_state_set(pid, LC_CONNECTED);

            // Release the link buffer
            lc_sco_free_link(pid, sync_linkid);
        }
        break;
        default:
        {
            ASSERT_INFO_FORCE(0, sync_linkid, lc_sco_env[sync_linkid].p_link_params->state);
        }
        break;
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }
}


void lc_sco_peer_reject_disc(ke_task_id_t pid, uint8_t reason)
{
    int idx = KE_IDX_GET(pid);
    uint8_t sync_linkid = 0;

    // Find the SCO link under negotiation
    for (sync_linkid = 0; sync_linkid < MAX_NB_SYNC ; sync_linkid++)
    {
        // Check if sco link ID is free
        if (lc_sco_env[sync_linkid].p_link_params != NULL)
        {
            if ((lc_sco_env[sync_linkid].p_link_params->link_id == idx) &&
                    (lc_sco_env[sync_linkid].p_link_params->state == LC_SCO_NEGO_WAIT_PEER_DISC_RSP))
                break;
        }

    }


    if (sync_linkid < MAX_NB_SYNC)
    {
        switch (lc_sco_env[sync_linkid].p_link_params->state)
        {
        case LC_SCO_NEGO_WAIT_PEER_DISC_RSP:
        {
            // Clear timeout
            ke_timer_clear(LC_LMP_RSP_TO, pid);

            // Inform Host that SCO is disconnected
            lc_sco_send_host_disc_evt(pid, sync_linkid, reason, CO_ERROR_CON_TERM_BY_LOCAL_HOST);

            // Clear LC task state machine
            ke_state_set(pid, LC_CONNECTED);

            // Release the link buffer
            lc_sco_free_link(pid, sync_linkid);
        }
        break;
        default:
        {
            ASSERT_INFO_FORCE(0, sync_linkid, lc_sco_env[sync_linkid].p_link_params->state);
        }
        break;
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

}





void lc_sco_baseband_ack(ke_task_id_t pid, uint8_t lmp_opcode)
{
    int idx = KE_IDX_GET(pid);
    uint8_t sync_linkid = 0;

    // Find the SCO link with an ongoing procedure (nego or disc)
    for (sync_linkid = 0; sync_linkid < MAX_NB_SYNC ; sync_linkid++)
    {
        // Check if sco link ID is free
        if (lc_sco_env[sync_linkid].p_link_params != NULL)
        {
            if ((lc_sco_env[sync_linkid].p_link_params->link_id == idx) &&
                    (lc_sco_env[sync_linkid].p_link_params->state != LC_SCO_CONNECTED))
                break;
        }

    }

    if (sync_linkid < MAX_NB_SYNC)
    {
        switch (lc_sco_env[sync_linkid].p_link_params->state)
        {
        case LC_SCO_NEGO_WAIT_PEER_CFM:
        {
            // Clear timeout
            ke_timer_clear(LC_LMP_RSP_TO, pid);

            // Start the SCO link
            lc_sco_start_link(pid, sync_linkid);
        }
        break;
        case LC_SCO_NEGO_WAIT_PEER_MOD_CFM:
        {
            // Clear timeout
            ke_timer_clear(LC_LMP_RSP_TO, pid);

            // Modify the SCO link
            lc_sco_modify_link(pid, sync_linkid);
        }
        break;
        case LC_SCO_NEGO_WAIT_PEER_DISC_CFM:
        {
            // Clear timeout
            ke_timer_clear(LC_LMP_RSP_TO, pid);

            if (lc_sco_env[sync_linkid].p_link_params->hv1_flag)
            {
                // Disconnect existing link
                lc_sco_stop_link(pid, sync_linkid);
            }

            // Inform Host that SCO is disconnected
            lc_sco_send_host_disc_evt(pid, sync_linkid, CO_ERROR_NO_ERROR, lc_sco_env[sync_linkid].p_link_params->disc_reason);

            // Clear LC task state machine
            ke_state_set(pid, LC_CONNECTED);

            // Release the link buffer
            lc_sco_free_link(pid, sync_linkid);
        }
        break;
        default:
        {
            // Several packets are not predictable
        }
        break;
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }
}

void lc_sco_timeout(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    uint8_t sync_linkid = 0;

    // Find the SCO link with an ongoing procedure (nego or disc)
    for (sync_linkid = 0; sync_linkid < MAX_NB_SYNC ; sync_linkid++)
    {
        // Check if sco link ID is free
        if (lc_sco_env[sync_linkid].p_link_params != NULL)
        {
            if ((lc_sco_env[sync_linkid].p_link_params->link_id == idx) &&
                    (lc_sco_env[sync_linkid].p_link_params->state != LC_SCO_CONNECTED))
                break;
        }

    }

    if (sync_linkid < MAX_NB_SYNC)
    {
        switch (lc_sco_env[sync_linkid].p_link_params->state)
        {
        case LC_SCO_NEGO_WAIT_PEER_RSP:
        {
            // Inform Host that SCO setup is completed
            lc_sco_send_host_cmp_evt(pid, sync_linkid, CO_ERROR_LMP_RSP_TIMEOUT);

            // End of link negotiation
            lc_sco_free_nego(pid, sync_linkid);

            // Release the link buffer
            lc_sco_free_link(pid, sync_linkid);
        }
        break;
        case LC_SCO_NEGO_WAIT_PEER_MOD_RSP:
        case LC_SCO_NEGO_WAIT_PEER_MOD_CFM:
        {
            // Disconnect existing link
            lc_sco_stop_link(pid, sync_linkid);

            // Inform Host that SCO is disconnected
            lc_sco_send_host_disc_evt(pid, sync_linkid, CO_ERROR_LMP_RSP_TIMEOUT, CO_ERROR_UNSPECIFIED_ERROR);

            // End of link negotiation
            lc_sco_free_nego(pid, sync_linkid);

            // Release the link buffer
            lc_sco_free_link(pid, sync_linkid);
        }
        break;
        case LC_SCO_NEGO_WAIT_PEER_DISC_CFM:
        case LC_SCO_NEGO_WAIT_PEER_DISC_RSP:
        {
            // Inform Host that SCO is disconnected
            lc_sco_send_host_disc_evt(pid, sync_linkid, CO_ERROR_LMP_RSP_TIMEOUT, CO_ERROR_UNSPECIFIED_ERROR);

            // Release the link buffer
            lc_sco_free_link(pid, sync_linkid);

            // Clear LC task state machine
            ke_state_set(pid, LC_CONNECTED);
        }
        break;
        default:
        {
            ASSERT_INFO_FORCE(0, sync_linkid, lc_sco_env[sync_linkid].p_link_params->state);
        }
        break;
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }
}

void lc_sco_offset_update(uint8_t sync_linkid, uint16_t sco_offset)
{
    // Check if SCO exists and its offset has changed
    if ((lc_sco_env[sync_linkid].p_link_params != NULL) && (sco_offset != lc_sco_env[sync_linkid].p_link_params->plan_elt.offset))
    {
        ASSERT_ERR(sco_offset < lc_sco_env[sync_linkid].p_link_params->plan_elt.interval);

        // Update planner element
        sch_plan_shift(BT_ACL_CONHDL_MIN + lc_sco_env[sync_linkid].p_link_params->link_id, sco_offset - lc_sco_env[sync_linkid].p_link_params->plan_elt.offset);
    }
}

#if (EAVESDROPPING_SUPPORT)
void lc_sco_send_delayed_host_req(ke_task_id_t pid)
{
    int idx = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    if (lc_env_ptr != NULL)
    {
        uint8_t sync_linkid = 0;

        // Find the SCO link being delayed
        for (sync_linkid = 0; sync_linkid < MAX_NB_SYNC; sync_linkid++)
        {
            // Check if sco link ID is free
            if (lc_sco_env[sync_linkid].p_link_params != NULL && lc_sco_env[sync_linkid].p_nego_params != NULL)
            {
                if (lc_sco_env[sync_linkid].p_link_params->link_id == idx)
                    break;
            }
        }

        if (sync_linkid < MAX_NB_SYNC)
        {
            // allocate the status event message
            struct hci_con_req_evt *event = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_CON_REQ_EVT_CODE, hci_con_req_evt);
            event->lk_type = lc_sco_env[sync_linkid].p_link_params->sync_type;
            memcpy(&event->bd_addr.addr[0], &lc_env_ptr->info.BdAddr.addr[0], BD_ADDR_LEN);
            memcpy(&event->classofdev.A[0], &lc_env_ptr->link.Class.A[0], DEV_CLASS_LEN);
            hci_send_2_host(event);

            // set the Connection Accept Timeout timer
            ke_timer_set(LC_CON_ACCEPT_TO, pid, 10 * co_slot_to_duration(hci_con_accept_to_get()));
        }
    }
}
#endif // EAVESDROPPING_SUPPORT

#endif // (MAX_NB_SYNC > 0)


///@} LCSCO
