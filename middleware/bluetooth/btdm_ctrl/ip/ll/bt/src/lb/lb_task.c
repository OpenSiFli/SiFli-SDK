/**
 ****************************************************************************************
 *
 * @file lb_task.c
 *
 * @brief LB task source file
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup LBTASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"    // stack configuration

#include <string.h>
#include "co_endian.h"
#include "co_utils.h"
#include "co_math.h"
#include "co_bt.h"          // BT standard definitions

#include "em_map.h"
#include "sch_plan.h"

#include "lb.h"             // link broadcaster definitions
#include "lb_int.h"         // link broadcaster internal definitions
//#include "lb_util.h"        // link broadcaster utilities definitions

#include "lc.h"
#include "lm.h"

#include "ke_mem.h"         // kernel memory definitions
#include "ke_timer.h"
#include "lm.h"
#include "ld.h"             // link driver

#include "rwip.h"           // stack main module

#if HCI_PRESENT
    #include "hci.h"            // host controller interface
#endif //HCI_PRESENT


/*
 * TYPES DEFINITION
 ****************************************************************************************
 */

/// Format of a HCI command handler function
typedef int (*lb_hci_cmd_hdl_func_t)(void const *param, uint16_t opcode);


/*
 * STRUCT DEFINITION
 ****************************************************************************************
 */

/// Element of a HCI command handler table.
struct lb_hci_cmd_handler
{
    /// Command opcode
    uint16_t opcode;
    /// Pointer to the handler function for HCI command.
    lb_hci_cmd_hdl_func_t func;
};

/*
 * LOCAL FUNCTIONS DEFINITION
 ****************************************************************************************
 */

#if BCAST_ENC_SUPPORT
__STATIC void lb_cmd_stat_send(uint16_t opcode, uint8_t status)
{
    // allocate the status event message
    struct hci_cmd_stat_event *evt = KE_MSG_ALLOC(HCI_CMD_STAT_EVENT, 0, opcode, hci_cmd_stat_event);
    // update the status
    evt->status = status;
    // send the message
    hci_send_2_host(evt);
}
#endif // BCAST_ENC_SUPPORT

/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */

#if BCAST_ENC_SUPPORT
/**
 ****************************************************************************************
 * @brief Handles the master key confirmation.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lb_mst_key_cfm, struct lb_mst_key_cfm)
{
    uint8_t link_id = KE_IDX_GET(src_id);

    switch (ke_state_get(dest_id))
    {
    case LB_WAIT_MASTER_KEY_CFM:

        // Find the corresponding entry and update it
        if (lb_env.mstkey[link_id].in_use)
        {
            lb_env.mstkey[link_id].enc_mode = param->enc_mode;
            lb_env.mstkey[link_id].enc_key_size_msk = param->key_sz_msk;
            lb_env.mstkey[link_id].status = param->status;
            lb_env.mstkey[link_id].in_use = (param->status != CO_ERROR_UNSPECIFIED_ERROR);
            lb_env.pid_lc_cnt -= 1;
        }

        // If no remaining entries to update, complete the procedure
        if (!lb_env.pid_lc_cnt)
        {
            uint8_t status = CO_ERROR_NO_ERROR;
            uint16_t conhdl = 0;
            for (link_id = 0; link_id < MAX_NB_ACTIVE_ACL; link_id++)
            {
                if (lb_env.mstkey[link_id].in_use == true)
                {
                    conhdl = BT_ACL_CONHDL_MIN + link_id;

                    if (lb_env.mstkey[link_id].status != CO_ERROR_NO_ERROR)
                    {
                        status = lb_env.mstkey[link_id].status;
                        break;
                    }
                }
            }

            if (status == CO_ERROR_NO_ERROR)
            {
                lb_mst_key_restart_enc(conhdl);
            }
            else
            {
                lb_mst_key_cmp(status, conhdl, lb_env.new_key_flag);
            }
        }
        break;
    default:
        ASSERT_ERR(0);
        break;
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the restart encryption confirmation.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lb_enc_restart_cfm, struct lb_enc_restart_cfm)
{
    uint8_t link_id = KE_IDX_GET(src_id);

    switch (ke_state_get(dest_id))
    {
    case LB_WAIT_RESTART_ENC_CFM:

        // Find the corresponding entry and update it
        if (lb_env.mstkey[link_id].in_use == true)
        {
            lb_env.mstkey[link_id].status = param->status;
            lb_env.pid_lc_cnt -= 1;
        }

        lb_env.conhdl = BT_ACL_CONHDL_MIN + link_id;

        // If no remaining entries to update, complete the procedure
        if (!lb_env.pid_lc_cnt)
        {
            uint8_t status;
            if (lb_env.new_key_flag == TEMPORARY_KEY)
            {
                status = CO_ERROR_NO_ERROR;
                for (link_id = 0; link_id < MAX_NB_ACTIVE_ACL; link_id++)
                {
                    if ((lb_env.mstkey[link_id].in_use == true) && (lb_env.mstkey[link_id].enc_mode != ENC_DISABLED))
                    {
                        if (lb_env.mstkey[link_id].status != CO_ERROR_NO_ERROR)
                        {
                            status = lb_env.mstkey[link_id].status;
                            break;
                        }
                    }
                }
                if (!status)
                {
                    lb_mst_start_act_bcst_enc();
                    lb_env.enc_brcst = true;
                }
            }
            else
            {
                lb_mst_stop_act_bcst_enc();
                lb_env.enc_brcst = false;
                status = param->status;
            }
            lb_mst_key_cmp(status, lb_env.conhdl, lb_env.new_key_flag);

        }
        break;
    case LB_WAIT_MASTER_KEY_CFM:
        break;
    default:
        ASSERT_ERR(0);
        break;
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the start encryption indication.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lb_enc_start_ind, struct lb_enc_start_ind)
{
    uint8_t status;
    switch (ke_state_get(dest_id))
    {
    case LB_WAIT_RESTART_ENC_CFM:
        break;
    default:
        status = CO_ERROR_COMMAND_DISALLOWED;
        for (uint8_t link_id = 0; link_id < MAX_NB_ACTIVE_ACL; link_id++)
        {
            if (lb_env.mstkey[link_id].in_use == true)
            {
                lb_env.mstkey[link_id].enc_mode = ENC_PP_ENABLED;
                status = CO_ERROR_NO_ERROR;
                break;
            }
        }
        if (!status)
        {
            if (param->key_flag == TEMPORARY_KEY)
            {
                lb_mst_start_act_bcst_enc();
                lb_env.enc_brcst = true;
            }
            else
            {
                lb_mst_stop_act_bcst_enc();
                lb_env.enc_brcst = false;
            }
        }
        break;
    }
    return (KE_MSG_CONSUMED);
}
/**
 ****************************************************************************************
 * @brief Handles the stop encryption indication.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lb_enc_stop_ind, void)
{
    uint8_t status;
    switch (ke_state_get(dest_id))
    {
    case LB_WAIT_RESTART_ENC_CFM:
        break;
    default:
        status = CO_ERROR_COMMAND_DISALLOWED;
        for (uint8_t link_id = 0; link_id < MAX_NB_ACTIVE_ACL; link_id++)
        {
            if (lb_env.mstkey[link_id].in_use == true)
            {
                lb_env.mstkey[link_id].enc_mode = ENC_DISABLED;
                status = CO_ERROR_NO_ERROR;
                break;
            }
        }
        if (!status)
        {
            lb_mst_stop_act_bcst_enc();
            lb_env.enc_brcst = false;
        }
        break;
    }
    return (KE_MSG_CONSUMED);
}
#endif // BCAST_ENC_SUPPORT

/*
 * HCI LINK CONTROL COMMANDS HANDLERS
 ****************************************************************************************
 */

#if BCAST_ENC_SUPPORT
/// Handle the command HCI master link key
HCI_CMD_HANDLER(master_lk, struct hci_master_lk_cmd)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;

    do
    {
        if ((param->key_flag != SEMI_PERMANENT_KEY) && (param->key_flag != TEMPORARY_KEY))
            break;

        if ((lc_check_all_slaves_support_sec_con() && (param->key_flag == TEMPORARY_KEY))
                || (lb_env.new_key_flag == param->key_flag))
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        status = CO_ERROR_NO_ERROR;

        lb_env.new_key_flag = param->key_flag;
        lb_env.mst_key_req = true;

        if (lb_env.mst_key_req == true)
        {
            lb_mst_key();
        }
        else
        {
            ke_state_set(TASK_LB, LB_IDLE);
        }

    }
    while (0);

    lb_cmd_stat_send(HCI_MASTER_LK_CMD_OPCODE, status);

    return (KE_MSG_CONSUMED);
}
#endif // BCAST_ENC_SUPPORT

#if CSB_SUPPORT

/**
 ****************************************************************************************
 * @brief Handles CSB tx supervision timeout event
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lb_csb_tx_end_ind, struct lb_csb_tx_end_ind)
{
    struct hci_con_slv_bcst_to_evt *evt = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_CON_SLV_BCST_TO_EVT_CODE, hci_con_slv_bcst_to_evt);

    SETB(lb_env.csb_mode_enabled, LB_CSB_TX_EN, 0);

    // invalidate csb not lpo_allowed flag
    rwip_prevent_sleep_clear(RW_CSB_NOT_LPO_ALLOWED);

    evt->bd_addr = param->bd_addr; //ld_env.local_bd_addr;
    evt->lt_addr = param->lt_addr; // lb_env.res_lt_addr;

    // send the message
    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles CSB Rx indication event
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lb_csb_rx_ind, struct lb_csb_rx_ind)
{
    struct bd_addr bd_addr = param->bd_addr;
    uint8_t  lt_addr = param->lt_addr;
    uint32_t clk  = param->clk;
    uint32_t offset = param->offset;

    uint8_t rx_status = param->rx_status;

    struct hci_con_slv_bcst_rec_evt *evt;

    // Shift rx statistics, for a new stat saved to LSB
    lb_env.csb_rx_stats <<= 1;

    if (rx_status == CSB_RX_OK)
    {
        uint16_t length = param->length;

        if (length <= CSB_FRAGMENT_SIZE_MAX)
        {
            evt = KE_MSG_ALLOC_DYN(HCI_EVENT, 0, HCI_CON_SLV_BCST_REC_EVT_CODE, hci_con_slv_bcst_rec_evt, length);
            evt->bd_addr = bd_addr;
            evt->lt_addr  = lt_addr;
            evt->clk = clk;
            evt->offset = offset;
            evt->receive_status = rx_status;
            evt->fragment = CSB_NO_FRAGMENTATION;
            evt->data_length = length;

            memcpy(&evt->data[0], &param->pdu[0], length);

            // send the message
            hci_send_2_host(evt);
        }
        else
        {
            // Send STARTING_FRAGMENT

            evt = KE_MSG_ALLOC_DYN(HCI_EVENT, 0, HCI_CON_SLV_BCST_REC_EVT_CODE, hci_con_slv_bcst_rec_evt, CSB_FRAGMENT_SIZE_MAX);
            evt->bd_addr = bd_addr;
            evt->lt_addr  = lt_addr;
            evt->clk = clk;
            evt->offset = offset;
            evt->receive_status = rx_status;
            evt->fragment = CSB_STARTING_FRAGMENT;
            evt->data_length = CSB_FRAGMENT_SIZE_MAX;

            memcpy(&evt->data[0], &param->pdu[0], CSB_FRAGMENT_SIZE_MAX);

            // send the message
            hci_send_2_host(evt);

            // Send CONTINUATION FRAGMENTS

            length -= CSB_FRAGMENT_SIZE_MAX;

            while (length > CSB_FRAGMENT_SIZE_MAX)
            {
                evt = KE_MSG_ALLOC_DYN(HCI_EVENT, 0, HCI_CON_SLV_BCST_REC_EVT_CODE, hci_con_slv_bcst_rec_evt, CSB_FRAGMENT_SIZE_MAX);
                evt->bd_addr = bd_addr;
                evt->lt_addr  = lt_addr;
                evt->clk = clk;
                evt->offset = offset;
                evt->receive_status = rx_status;
                evt->fragment = CSB_CONTINUATION_FRAGMENT;
                evt->data_length = CSB_FRAGMENT_SIZE_MAX;

                memcpy(&evt->data[param->length - length], &param->pdu[0], CSB_FRAGMENT_SIZE_MAX);

                // send the message
                hci_send_2_host(evt);

                length -= CSB_FRAGMENT_SIZE_MAX;
            }

            // Send ENDING FRAGMENT

            evt = KE_MSG_ALLOC_DYN(HCI_EVENT, 0, HCI_CON_SLV_BCST_REC_EVT_CODE, hci_con_slv_bcst_rec_evt, length);
            evt->bd_addr = bd_addr;
            evt->lt_addr  = lt_addr;
            evt->clk = clk;
            evt->offset = offset;
            evt->receive_status = rx_status;
            evt->fragment = CSB_ENDING_FRAGMENT;
            evt->data_length = length;

            memcpy(&evt->data[param->length - length], &param->pdu[0], length);

            // send the message
            hci_send_2_host(evt);
        }
    }
    else
    {
        // Receivers should monitor the CPB reception rate and obtain the current  AFH channel map of the Transmitter
        // via the synchronization train if degradation exceeds desired thresholds. BB:Appx C.5.
        lb_env.csb_rx_stats |= 1;

        // Check number of recent bad status relative to a threshold, and synchronization scan inactive
        if ((NB_ONE_BITS(lb_env.csb_rx_stats) > LB_CSB_RX_STATS_CH_MAP_THRESHOLD) && (lb_env.sscan_req == LB_SSCAN_IDLE))
        {
            // Start synchronization train to obtain the current AFH channel map
            if (CO_ERROR_NO_ERROR == ld_sscan_start(&bd_addr, lb_env.sync_scan_to, lb_env.sync_scan_win, lb_env.sync_scan_int))
            {
                lb_env.sscan_req = LB_SSCAN_CHMAP_REQ;
            }
        }

        // A single event shall be generated for a CPB instant on which a CPB packet was scheduled for reception but
        // the BR/EDR Controller failed to successfully receive it. HCI:7.7.69.
        evt = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_CON_SLV_BCST_REC_EVT_CODE, hci_con_slv_bcst_rec_evt);
        evt->bd_addr = bd_addr;
        evt->lt_addr  = lt_addr;
        evt->clk = clk;
        evt->offset = offset;
        evt->receive_status = rx_status;
        evt->fragment = CSB_NO_FRAGMENTATION;
        evt->data_length = 0;

        // send the message
        hci_send_2_host(evt);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles CSB Rx timeout indication event
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lb_csb_rx_end_ind, struct lb_csb_rx_end_ind)
{
    struct hci_con_slv_bcst_to_evt *evt = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_CON_SLV_BCST_TO_EVT_CODE, hci_con_slv_bcst_to_evt);

    evt->bd_addr = param->bd_addr; //ld_env.local_bd_addr;
    evt->lt_addr = param->lt_addr; // lb_env.res_lt_addr;

    SETB(lb_env.csb_mode_enabled, LB_CSB_RX_EN, 0);

    // send the message
    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}
#endif // CSB_SUPPORT

#if CSB_SUPPORT || PCA_SUPPORT


/**
 ****************************************************************************************
 * @brief Handles Sync Train End  indication event
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lb_sscan_end_ind, struct lb_sscan_end_ind)
{
#if CSB_SUPPORT
    if (lb_env.sscan_req == LB_SSCAN_HOST_REQ)
    {
        // Generate the sync train received  event
        struct hci_sync_train_rec_evt *evt = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_SYNC_TRAIN_REC_EVT_CODE, hci_sync_train_rec_evt);

        evt->bd_addr = param->bd_addr;

        evt->clock_offset = param->clock_offset;
        evt->afh_ch_map = param->afh_ch_map;
        evt->lt_addr = param->lt_addr;
        evt->next_bcst_instant = param->next_bcst_instant;
        evt->csb_int = param->csb_int;
        evt->service_data = param->service_data;

        evt->status = param->status;

        hci_send_2_host(evt);
    }
    else if (GETB(lb_env.csb_mode_enabled, LB_CSB_RX_EN) && (param->status == CO_ERROR_NO_ERROR))
    {
        // CSB receiver already active  - Update channel map only
        ld_csb_rx_afh_update((struct bt_ch_map *)&param->afh_ch_map);
        // Reset CSB reception statistics
        lb_env.csb_rx_stats = 0;
    }
#endif // CSB_SUPPORT

#if PCA_SUPPORT
    if ((lb_env.sscan_req == LB_SSCAN_PCA_SYNC) && (param->status == CO_ERROR_NO_ERROR))
    {
        struct lc_pca_sscan_clk_ind *msg = KE_MSG_ALLOC(LC_PCA_SSCAN_CLK_IND, KE_BUILD_ID(TASK_LC, lb_env.pca.sscan_link_id), TASK_LB, lc_pca_sscan_clk_ind);
        msg->clock_offset = param->clock_offset;
        msg->bit_offset = param->bit_offset;
        msg->bd_addr = param->bd_addr;
        ke_msg_send(msg);
    }
#endif // PCA_SUPPORT

    // Clear the request
    lb_env.sscan_req = LB_SSCAN_IDLE;

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles Sync Train End  indication event
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lb_strain_end_ind, struct lb_strain_end_ind)
{

#if CSB_SUPPORT
    if (!param->coarse_clk_adj)
    {
        struct hci_sync_train_cmp_evt *evt = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_SYNC_TRAIN_CMP_EVT_CODE, hci_sync_train_cmp_evt);
        evt->status = param->status;
        hci_send_2_host(evt);
    }
#endif // CSB_SUPPORT

    return (KE_MSG_CONSUMED);
}
#endif // CSB_SUPPORT || PCA_SUPPORT

#if PCA_SUPPORT

#if RW_BT_MWS_COEX
/**
 ****************************************************************************************
 * @brief Handles a local PCA request
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lb_local_pca_req, struct lb_local_pca_req)
{
    if (!lb_env.pca.instant_pending)
    {
        if (lm_get_nb_acl(MASTER_FLAG))
        {
            /* process clock adjust requirement as master */
            uint8_t status = lb_master_clk_adj_req_handler(param->clk_adj_us, param->clk_adj_slots, param->clk_adj_period, MASTER_ROLE);
            if (CO_ERROR_DIFF_TRANSACTION_COLLISION == status)
            {
                // Discard the message and wait for fresh PCA sample on next frame
                ld_pca_reporting_enable(true);
            }
        }
        else if (lm_get_nb_acl(SLAVE_FLAG))
        {
            /* process requirement as a slave */
            for (int idx = 0; idx < MAX_NB_ACTIVE_ACL; idx++)
            {
                ke_task_id_t _dest_id = KE_BUILD_ID(TASK_LC, idx);

                if (lm_is_acl_con_role(idx, SLAVE_ROLE))
                {
                    struct lc_local_pca_req *msg = KE_MSG_ALLOC(LC_LOCAL_PCA_REQ, _dest_id, TASK_LB, lc_local_pca_req);
                    msg->clk_adj_us = param->clk_adj_us;
                    msg->clk_adj_slots = param->clk_adj_slots;
                    msg->clk_adj_period = param->clk_adj_period;

                    ke_msg_send(msg);
                }
            }
        }
        else
        {
            /* no active connections yet - so can adjust local clock with immediate effect */
            uint32_t clk_adj_instant = (ld_read_clock() >> 1) + PCA_INSTANT_MIN;
            ld_pca_coarse_clock_adjust(param->clk_adj_us, param->clk_adj_slots, clk_adj_instant);
        }
    }

    return (KE_MSG_CONSUMED);
}
#endif // RW_BT_MWS_COEX

/**
 ****************************************************************************************
 * @brief Handles PCA broadcast retransmission interval
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lb_pca_tx_intv_to, void)
{
    if (lb_env.pca.pending_clk_adj_acks && (lm_get_nb_acl(MASTER_FLAG)))
    {
        uint32_t clk_adj_tx_clock = CO_ALIGN4_LO(ld_read_clock() + 2 * LB_CLK_ADJ_CLK_TX_INSTANT_OFFSET);

        // Unless in recovery mode, Ensure not setting a timeout after the instant
        if ((lb_env.pca.clk_adj_mode == CLK_ADJ_AFTER_INSTANT) || CLK_GREATER_THAN(2 * lb_env.pca.clk_adj_instant, clk_adj_tx_clock))
        {
            // clk_adj_clk is expressed in slot pairs
            lb_env.pca.clk_adj_clk = clk_adj_tx_clock >> 2;

            // Rebroadcast the clk_adj directive
            lb_send_pdu_clk_adj(lb_env.pca.clk_adj_instant, lb_env.pca.clk_adj_id, lb_env.pca.clk_adj_us, lb_env.pca.clk_adj_slots,
                                lb_env.pca.clk_adj_mode, lb_env.pca.clk_adj_clk, lb_env.pca.tr_id);

            // Restart CCA retransmit interval timer
            ke_timer_set(LB_PCA_TX_INTV_TO, TASK_LB, 10 /*10ms*/);
        }
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles a PCA complete indication
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lb_pca_end_ind, struct lb_pca_end_ind)
{
#if RW_BT_MWS_COEX
    /* Immediate re-enable frame sync interrupt for monitoring external frame alignment */
    if (lm_local_ext_fr_configured())
    {
        ld_pca_reporting_enable(true);
    }
#endif //RW_BT_MWS_COEX

    if (lb_env.pca.pending_clk_adj_acks || (param->status != CO_ERROR_NO_ERROR))
    {
        // Coarse Clock Adjustment Recovery Mode

        // Indicate is after instant
        lb_env.pca.clk_adj_mode = CLK_ADJ_AFTER_INSTANT;

        // clk_adj_clk is expressed in slot pairs
        lb_env.pca.clk_adj_clk = CO_ALIGN4_LO(ld_read_clock() + 2 * LB_CLK_ADJ_CLK_TX_INSTANT_OFFSET) >> 2;

        // Schedule the next broadcast clk_adj message
        lb_send_pdu_clk_adj(lb_env.pca.clk_adj_instant, lb_env.pca.clk_adj_id, lb_env.pca.clk_adj_us, lb_env.pca.clk_adj_slots,
                            lb_env.pca.clk_adj_mode, lb_env.pca.clk_adj_clk, lb_env.pca.tr_id);

        // Restart CCA retransmit interval timer
        ke_timer_set(LB_PCA_TX_INTV_TO, TASK_LB, 10 /*10ms*/);

        /* synchronisation train should be started on master if coarse clock adjust fails, or if
           slaves have yet to acknowledge the coarse clock adjustment prior to the adjustment instant */
        ld_strain_start(SYNC_TRAIN_TO_CCA_RM_DEFAULT, SYNC_TRAIN_INTV_CLK_ADJ, 1, NULL, 0, 0, 0, 0);
    }

    lc_notify_all_slaves_clk_adj_complete();

    lb_env.pca.instant_pending = false;

    return (KE_MSG_CONSUMED);
}

#if RW_BT_MWS_COEX
/**
 ****************************************************************************************
 * @brief Handles re-enable of MWS monitoring for PCA after a successful PCA adjustment
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lb_pca_monitor_intv_to, void)
{
    ld_pca_reporting_enable(true);

    return (KE_MSG_CONSUMED);
}
#endif //RW_BT_MWS_COEX
#endif // PCA_SUPPORT

/*
 * HCI DATA HANDLERS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handles the HCI ACL data packet
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(hci_acl_data_lb, struct hci_acl_data)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    // If payload is null, ignore the message
    if (param->length > 0)
    {
        struct bt_em_acl_buf_elt *tx_elt = bt_util_buf_acl_tx_elt_get((uint16_t) param->buf_ptr);

        // Fill data length and flag fields
        SETF(tx_elt->data_len_flags, BT_EM_ACL_DATA_LEN, param->length);
        SETF(tx_elt->data_len_flags, BT_EM_ACL_PBF, GETF(param->conhdl_pb_bc_flag, HCI_ACL_HDR_PB_FLAG));
        SETF(tx_elt->data_len_flags, BT_EM_ACL_BF, GETF(param->conhdl_pb_bc_flag, HCI_ACL_HDR_BC_FLAG));

        lb_env.conhdl = src_id;

        status = ld_bcst_acl_data_tx(0, tx_elt, &lb_env.ch_map, lb_env.num_bcst_ret);

        if (status != CO_ERROR_NO_ERROR)
        {
            // Free the buffer
            bt_util_buf_acl_tx_free(tx_elt->buf_ptr);
        }
        else
        {
            // Activate AFH timer if needed
            lm_afh_activate_timer();
        }
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles ACL TX confirmation
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
KE_MSG_HANDLER(lb_acl_tx_cfm, struct lb_acl_tx_cfm)
{
    uint16_t conhdl = lb_env.conhdl;
    uint8_t nb_of_pkt = 1;

    // allocates the message to send
    struct hci_nb_cmp_pkts_evt *event = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_NB_CMP_PKTS_EVT_CODE, hci_nb_cmp_pkts_evt);

    // gets the connection handle used
    event->con[0].hdl         = co_htobs(conhdl);
    // gets the number of packet sent
    event->con[0].nb_comp_pkt = co_htobs(nb_of_pkt);
    // processed handle by handle
    event->nb_of_hdl          = 1;

    // send the message
    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}

/*
 * HCI COMMAND HANDLING
 ****************************************************************************************
 */

/// Handle the command HCI read number of broadcast retransmissions
HCI_CMD_HANDLER(rd_nb_bdcst_retx, void)
{
    // allocate the complete event message
    struct hci_rd_nb_bdcst_retx_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_rd_nb_bdcst_retx_cmd_cmp_evt);

    evt->num_bcst_ret = lb_util_get_nb_broadcast() - 1;
    evt->status = CO_ERROR_NO_ERROR;
    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI write number of broadcast retransmission
HCI_CMD_HANDLER(wr_nb_bdcst_retx, struct hci_wr_nb_bdcst_retx_cmd)
{
    // allocate the complete event message
    struct hci_basic_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_basic_cmd_cmp_evt);

    lb_util_set_nb_broadcast(param->num_bcst_ret + 1);

    evt->status = CO_ERROR_NO_ERROR;
    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

#if CSB_SUPPORT

/// Handle the command HCI set reserved LT_ADDR
HCI_CMD_HANDLER(set_res_lt_addr, struct hci_set_res_lt_addr_cmd)
{
    // allocate the complete event message
    struct hci_set_res_lt_addr_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_set_res_lt_addr_cmd_cmp_evt);

    uint8_t lt_addr = param->lt_addr;

    if (lt_addr == 0 || lt_addr > LT_ADDR_MAX)
    {
        evt->status = CO_ERROR_INVALID_HCI_PARAM;
    }
    else
    {
        if (lm_lt_addr_reserve(lt_addr))
        {
            lb_env.res_lt_addr = lt_addr;
            evt->status = CO_ERROR_NO_ERROR;
        }
        else
        {
            evt->status = CO_ERROR_CON_ALREADY_EXISTS;
        }
    }

    evt->lt_addr = lt_addr;

    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI delete reserved LT_ADDR
HCI_CMD_HANDLER(del_res_lt_addr, struct hci_del_res_lt_addr_cmd)
{
    struct hci_del_res_lt_addr_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_del_res_lt_addr_cmd_cmp_evt);

    uint8_t lt_addr = param->lt_addr;

    if (lt_addr == 0 || lt_addr > LT_ADDR_MAX)
    {
        evt->status = CO_ERROR_INVALID_HCI_PARAM;
    }
    else
    {
        if (lt_addr != lb_env.res_lt_addr)
        {
            evt->status = CO_ERROR_UNKNOWN_CONNECTION_ID;
        }
        else if (GETB(lb_env.csb_mode_enabled, LB_CSB_TX_EN))
        {
            evt->status = CO_ERROR_COMMAND_DISALLOWED;
        }
        else
        {
            lm_lt_addr_free(lt_addr);
            ld_csb_tx_clr_data();
            lb_env.res_lt_addr = LB_CSB_LT_ADDR_NOT_RESERVED;
            evt->status = CO_ERROR_NO_ERROR;
        }
    }

    evt->lt_addr = lt_addr;

    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI set connectionless slave broadcast
HCI_CMD_HANDLER(set_con_slv_bcst, struct hci_set_con_slv_bcst_cmd)
{
    struct hci_set_con_slv_bcst_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_set_con_slv_bcst_cmd_cmp_evt);

    evt->lt_addr = param->lt_addr;
    evt->interval = 0;

    if (param->lt_addr != lb_env.res_lt_addr)
    {
        evt->status = CO_ERROR_UNKNOWN_CONNECTION_ID;
    }
    else if ((param->enable > 1) || ((param->enable == 1) && ((param->lpo_allowed > 1) || (param->interval_min & 1) || (param->interval_min == 0) ||
                                     (param->interval_max & 1) || (param->interval_max == 0) || (param->csb_supv_to & 1) || (param->csb_supv_to == 0)
                                     || (((param->packet_type & ~PACKET_TYPE_DM1_FLAG) & PACKET_TYPE_GFSK_MSK) && ((~param->packet_type & ~PACKET_TYPE_DM1_FLAG) & PACKET_TYPE_EDR_MSK))
                                     || (param->packet_type == (PACKET_TYPE_EDR_MSK & ~PACKET_TYPE_DM1_FLAG)))))
    {
        evt->status = CO_ERROR_INVALID_HCI_PARAM;
    }
    else if (param->enable == 1)
    {
        // Input parameters
        struct sch_plan_req_param req_param;

        uint8_t max_slots = LM_MaxSlot(param->packet_type);

        // Compute the scheduling parameters
        req_param.interval_min = param->interval_min * 2;
        req_param.interval_max = param->interval_max * 2;
        req_param.duration_min = 2;
        req_param.duration_max = max_slots * 2;
        req_param.offset_min   = 0;
        req_param.offset_max   = req_param.interval_max - 1;
        req_param.pref_period  = 0;
        req_param.conhdl       = 0xFFFF;
        req_param.conhdl_ref   = 0xFFFF;
        req_param.margin       = 0;

        if (sch_plan_req(&req_param) == SCH_PLAN_ERROR_OK)
        {
            struct ld_csb_en_params csb_en_par;
            uint8_t status;

            csb_en_par.lt_addr = param->lt_addr;
            csb_en_par.packet_types = param->packet_type;
            csb_en_par.max_slots = max_slots;
            csb_en_par.csb_interval = CO_ALIGN4_LO(req_param.interval);
            csb_en_par.csb_offset = CO_ALIGN4_LO(req_param.offset_min);
            csb_en_par.csb_supv_to = param->csb_supv_to;
            csb_en_par.afh_ch_map = &lb_env.ch_map;

            status = ld_csb_tx_en(&csb_en_par);

            if (CO_ERROR_NO_ERROR == status)
            {
                lb_env.csb_interval = csb_en_par.csb_interval;
                lb_env.csb_offset = csb_en_par.csb_offset;

                lb_env.csb_max_pkt_size = lc_util_get_max_packet_size(param->packet_type);

                if (param->lpo_allowed)
                {
                    rwip_prevent_sleep_clear(RW_CSB_NOT_LPO_ALLOWED);
                }
                else
                {
                    rwip_prevent_sleep_set(RW_CSB_NOT_LPO_ALLOWED);
                }

                SETB(lb_env.csb_mode_enabled, LB_CSB_TX_EN, 1);

                // Activate timer if needed
                lm_afh_activate_timer();
            }

            evt->interval = lb_env.csb_interval / 2;
            evt->status = status;
        }
        else
        {
            evt->status = CO_ERROR_CONN_REJ_LIMITED_RESOURCES;
        }
    }
    else
    {
        // disable CSB and set to inactive
        ld_csb_tx_dis();
        SETB(lb_env.csb_mode_enabled, LB_CSB_TX_EN, 0);

        // also exit the sync train substate if active
        ld_strain_stop();

        // invalidate csb not lpo_allowed flag
        rwip_prevent_sleep_clear(RW_CSB_NOT_LPO_ALLOWED);

        evt->status = CO_ERROR_NO_ERROR;
    }

    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI set connectionless slave broadcast receive
HCI_CMD_HANDLER(set_con_slv_bcst_rec, struct hci_set_con_slv_bcst_rec_cmd)
{
    struct hci_set_con_slv_bcst_rec_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_set_con_slv_bcst_rec_cmd_cmp_evt);

    uint8_t status;

    if (param->enable == CSB_RX_MODE_EN)
    {
        uint8_t lt_addr = param->lt_addr;
        uint16_t csb_supv_to = param->csb_supv_to;
        uint16_t interval = param->interval;

        if ((lt_addr == 0) || (lt_addr > LT_ADDR_MAX) || (interval == 0) || (interval & 1) || (csb_supv_to == 0) || (csb_supv_to & 1)
                || (((param->packet_type & ~PACKET_TYPE_DM1_FLAG) & PACKET_TYPE_GFSK_MSK) && ((~param->packet_type & ~PACKET_TYPE_DM1_FLAG) & PACKET_TYPE_EDR_MSK))
                || (param->packet_type == (PACKET_TYPE_EDR_MSK & ~PACKET_TYPE_DM1_FLAG)))
        {
            status = CO_ERROR_INVALID_HCI_PARAM;
        }
        else
        {
            uint32_t clock_offset = param->clock_offset & 0x0FFFFFFF; // 28 bits meaningful
            uint32_t next_csb_clock = param->next_csb_clock & 0x0FFFFFFF; // 28 bits meaningful

            struct ld_csb_rx_params csb_rx;

            csb_rx.bd_addr = param->bd_addr;
            csb_rx.lt_addr = lt_addr;
            csb_rx.interval = interval;
            csb_rx.clock_offset = clock_offset;
            csb_rx.next_csb_clock = next_csb_clock;
            csb_rx.csb_supv_to = csb_supv_to;
            csb_rx.remote_timing_accuracy = param->remote_timing_accuracy;
            csb_rx.skip = param->skip;

            // The Host shall either enable BR packet types or shall enable EDR and DM1 packet types only  HCI:7.1.50
            // Check if EDR flags set - if so, indicate EDR to the LD
            csb_rx.edr = ((param->packet_type ^ PACKET_TYPE_EDR_MSK) & PACKET_TYPE_EDR_MSK & ~PACKET_TYPE_DM1_FLAG);

            csb_rx.afh_ch_map = param->afh_ch_map;

            status = ld_csb_rx_start(&csb_rx);

            if (status == CO_ERROR_NO_ERROR)
            {
                SETB(lb_env.csb_mode_enabled, LB_CSB_RX_EN, 1);
                lb_env.csb_rx_stats = 0;
            }
        }
    }
    else if (param->enable == CSB_RX_MODE_DIS)
    {
        // Stop CSB receive mode
        status = ld_csb_rx_stop();
    }
    else
    {
        status = CO_ERROR_INVALID_HCI_PARAM;
    }

    evt->status = status;
    evt->lt_addr = param->lt_addr;
    memcpy(&evt->bd_addr.addr[0], &param->bd_addr.addr[0], BD_ADDR_LEN);

    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI set connectionless slave broadcast data
HCI_CMD_HANDLER(set_con_slv_bcst_data, struct hci_set_con_slv_bcst_data_cmd)
{
    struct hci_set_con_slv_bcst_data_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_set_con_slv_bcst_data_cmd_cmp_evt);

    uint8_t fragment = param->fragment;

    if (param->lt_addr != lb_env.res_lt_addr)
    {
        evt->status = CO_ERROR_UNKNOWN_CONNECTION_ID;
    }
    else
    {
        if (lb_env.buf_ptr == 0)
        {
            lb_env.buf_ptr = bt_util_buf_acl_tx_alloc();
        }

        if (lb_env.buf_ptr != 0)
        {
            // Give the pointer to the data space
            uint8_t *buf = (uint8_t *)(EM_BASE_ADDR + lb_env.buf_ptr);

            if ((fragment == CSB_STARTING_FRAGMENT) || (fragment == CSB_NO_FRAGMENTATION))
            {
                if (param->data_length <= lb_env.csb_max_pkt_size)
                {
                    memcpy(buf, &param->data[0], param->data_length);
                    lb_env.csb_buf_data_len = param->data_length;
                    evt->status = CO_ERROR_NO_ERROR;
                }
                else
                {
                    // if length exceeds capacity, discard and report error
                    bt_util_buf_acl_tx_free(lb_env.buf_ptr);
                    lb_env.buf_ptr = 0;
                    lb_env.csb_buf_data_len = 0;
                    evt->status = CO_ERROR_INVALID_HCI_PARAM;
                }
            }
            else if ((fragment == CSB_CONTINUATION_FRAGMENT) || (fragment == CSB_ENDING_FRAGMENT))
            {
                if (lb_env.csb_buf_data_len + param->data_length <= lb_env.csb_max_pkt_size)
                {
                    memcpy(&buf[lb_env.csb_buf_data_len], &param->data[0], param->data_length);
                    lb_env.csb_buf_data_len += param->data_length;
                    evt->status = CO_ERROR_NO_ERROR;
                }
                else
                {
                    // if combined length exceeds capacity, discard and report error
                    bt_util_buf_acl_tx_free(lb_env.buf_ptr);
                    lb_env.buf_ptr = 0;
                    lb_env.csb_buf_data_len = 0;
                    evt->status = CO_ERROR_INVALID_HCI_PARAM;
                }
            }
            else
            {
                evt->status = CO_ERROR_INVALID_HCI_PARAM;
            }
        }
        else
        {
            ASSERT_ERR(0);
        }
    }

    if (((fragment == CSB_NO_FRAGMENTATION) || (fragment == CSB_ENDING_FRAGMENT))
            && (CO_ERROR_NO_ERROR == evt->status))
    {
        struct bt_em_acl_buf_elt *buf_elt = bt_util_buf_acl_tx_elt_get(lb_env.buf_ptr);
        SETF(buf_elt->data_len_flags, BT_EM_ACL_DATA_LEN, lb_env.csb_buf_data_len);

        ld_csb_tx_set_data(buf_elt);

        lb_env.buf_ptr = 0;
        lb_env.csb_buf_data_len = 0;
    }

    evt->lt_addr = param->lt_addr;

    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI read synchronization train parameters
HCI_CMD_HANDLER(rd_sync_train_param, void)
{
    struct hci_rd_sync_train_param_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_rd_sync_train_param_cmd_cmp_evt);

    evt->sync_train_int = lb_env.sync_train_intv;
    evt->sync_train_to = lb_env.sync_train_to;
    evt->service_data = lb_env.service_data;

    evt->status = CO_ERROR_NO_ERROR;

    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI write synchronization train parameters
HCI_CMD_HANDLER(wr_sync_train_param, struct hci_wr_sync_train_param_cmd)
{
    struct hci_wr_sync_train_param_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_wr_sync_train_param_cmd_cmp_evt);

    uint8_t status;

    uint16_t int_min = param->int_min;
    uint16_t int_max = param->int_max;
    uint32_t sync_train_to = param->sync_train_to;

    if ((int_min & 1) || (int_max & 1) || (int_min < SYNC_TRAIN_INTV_MIN) || (int_max < SYNC_TRAIN_INTV_MIN)
            || (sync_train_to & 1) || (sync_train_to == 0))
    {
        status = CO_ERROR_INVALID_HCI_PARAM;
    }
    else
    {
        // Determine a sync_train_intv between Interval_Max, Interval_Min. For now set to Interval_Min.
        lb_env.sync_train_intv = int_min;

        lb_env.sync_train_to = sync_train_to;
        lb_env.service_data = param->service_data;

        status = CO_ERROR_NO_ERROR;
    }

    evt->sync_train_int = lb_env.sync_train_intv;
    evt->status = status;

    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI start synchronization train
HCI_CMD_HANDLER(start_sync_train, void)
{
    struct hci_cmd_stat_event *evt = KE_MSG_ALLOC(HCI_CMD_STAT_EVENT, 0, opcode, hci_cmd_stat_event);

    if (GETB(lb_env.csb_mode_enabled, LB_CSB_TX_EN))
    {
        struct bt_ch_map *ch_map = &lb_env.ch_map;

        evt->status = ld_strain_start(lb_env.sync_train_to, lb_env.sync_train_intv, 0, ch_map, lb_env.csb_offset,
                                      lb_env.csb_interval, lb_env.res_lt_addr, lb_env.service_data);
    }
    else
    {
        evt->status = CO_ERROR_COMMAND_DISALLOWED;
    }

    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

/// Handle the command HCI receive synchronization train
HCI_CMD_HANDLER(rec_sync_train, struct hci_rec_sync_train_cmd)
{
    struct hci_cmd_stat_event *evt = KE_MSG_ALLOC(HCI_CMD_STAT_EVENT, 0, opcode, hci_cmd_stat_event);

    struct bd_addr *bd_addr = (struct bd_addr *)&param->bd_addr;

    uint16_t sync_scan_to = param->sync_scan_to;
    uint16_t sync_scan_win = param->sync_scan_win;
    uint16_t sync_scan_int = param->sync_scan_int;

    if ((sync_scan_to & 1) || (sync_scan_to < SYNC_SCAN_TO_MIN) || (sync_scan_win & 1) || (sync_scan_win < SYNC_SCAN_WIN_MIN)
            || (sync_scan_int & 1) || (sync_scan_int < SYNC_SCAN_INTV_MIN) || ((sync_scan_win + 2) > sync_scan_int) || (sync_scan_to < sync_scan_win))
    {
        evt->status = CO_ERROR_INVALID_HCI_PARAM;
    }
    // Check if resource is idle
    else if (LB_SSCAN_IDLE != lb_env.sscan_req)
    {
        evt->status = CO_ERROR_COMMAND_DISALLOWED;
    }
    else
    {
        // Start sync train
        evt->status = ld_sscan_start(bd_addr, sync_scan_to, sync_scan_int, sync_scan_win);

        if (CO_ERROR_NO_ERROR == evt->status)
        {
            // Save LB environment
            lb_env.sync_scan_to = sync_scan_to;
            lb_env.sync_scan_win = sync_scan_win;
            lb_env.sync_scan_int = sync_scan_int;

            lb_env.sscan_req = LB_SSCAN_HOST_REQ;
        }
    }

    hci_send_2_host(evt);

    return (KE_MSG_CONSUMED);
}

#endif //CSB_SUPPORT

/// The message handlers for HCI commands
HCI_CMD_HANDLER_TAB(lb)
{
#if BCAST_ENC_SUPPORT
    {HCI_MASTER_LK_CMD_OPCODE, (lb_hci_cmd_hdl_func_t) hci_master_lk_cmd_handler                 },
#endif // BCAST_ENC_SUPPORT
    {HCI_RD_NB_BDCST_RETX_CMD_OPCODE, (lb_hci_cmd_hdl_func_t) hci_rd_nb_bdcst_retx_cmd_handler          },
    {HCI_WR_NB_BDCST_RETX_CMD_OPCODE, (lb_hci_cmd_hdl_func_t) hci_wr_nb_bdcst_retx_cmd_handler          },
#if CSB_SUPPORT
    {HCI_SET_RES_LT_ADDR_CMD_OPCODE, (lb_hci_cmd_hdl_func_t) hci_set_res_lt_addr_cmd_handler           },
    {HCI_DEL_RES_LT_ADDR_CMD_OPCODE, (lb_hci_cmd_hdl_func_t) hci_del_res_lt_addr_cmd_handler           },
    {HCI_SET_CON_SLV_BCST_CMD_OPCODE, (lb_hci_cmd_hdl_func_t) hci_set_con_slv_bcst_cmd_handler          },
    {HCI_SET_CON_SLV_BCST_REC_CMD_OPCODE, (lb_hci_cmd_hdl_func_t) hci_set_con_slv_bcst_rec_cmd_handler      },
    {HCI_SET_CON_SLV_BCST_DATA_CMD_OPCODE, (lb_hci_cmd_hdl_func_t) hci_set_con_slv_bcst_data_cmd_handler     },
    {HCI_RD_SYNC_TRAIN_PARAM_CMD_OPCODE, (lb_hci_cmd_hdl_func_t) hci_rd_sync_train_param_cmd_handler       },
    {HCI_WR_SYNC_TRAIN_PARAM_CMD_OPCODE, (lb_hci_cmd_hdl_func_t) hci_wr_sync_train_param_cmd_handler       },
    {HCI_START_SYNC_TRAIN_CMD_OPCODE, (lb_hci_cmd_hdl_func_t) hci_start_sync_train_cmd_handler          },
    {HCI_REC_SYNC_TRAIN_CMD_OPCODE, (lb_hci_cmd_hdl_func_t) hci_rec_sync_train_cmd_handler            },
#endif //CSB_SUPPORT
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
KE_MSG_HANDLER(hci_command_lb, void)
{
    int return_status = KE_MSG_CONSUMED;

    // Check if there is a handler corresponding to the original command opcode
    for (uint16_t i = 0; i < ARRAY_LEN(lb_hci_command_handler_tab); i++)
    {
        // Check if opcode matches
        if (lb_hci_command_handler_tab[i].opcode == src_id)
        {
            // Check if there is a handler function
            if (lb_hci_command_handler_tab[i].func != NULL)
            {
                // Call handler
                return_status = lb_hci_command_handler_tab[i].func(param, src_id);
            }
            break;
        }
    }

    return return_status;
}

/*
 * TASK DESCRIPTOR DEFINITIONS
 ****************************************************************************************
 */

/// Specifies the default message handlers
KE_MSG_HANDLER_TAB(lb)
{
    // Note: all messages must be sorted in ID ascending order

    /*
     * ************** Msg LC -> LB****************
     */
#if BCAST_ENC_SUPPORT
    {LB_MST_KEY_CFM, (ke_msg_func_t)lb_mst_key_cfm_handler},
    {LB_ENC_RESTART_CFM, (ke_msg_func_t)lb_enc_restart_cfm_handler},
    {LB_ENC_START_IND, (ke_msg_func_t)lb_enc_start_ind_handler},
    {LB_ENC_STOP_IND, (ke_msg_func_t)lb_enc_stop_ind_handler},
#endif // BCAST_ENC_SUPPORT

    /*
     * ************** Msg LD -> LB****************
     */
    {LB_ACL_TX_CFM, (ke_msg_func_t) lb_acl_tx_cfm_handler  },

#if (CSB_SUPPORT)
    {LB_CSB_TX_END_IND, (ke_msg_func_t)lb_csb_tx_end_ind_handler },
    {LB_CSB_RX_IND, (ke_msg_func_t)lb_csb_rx_ind_handler },
    {LB_CSB_RX_END_IND, (ke_msg_func_t)lb_csb_rx_end_ind_handler },
#endif // (CSB_SUPPORT)

#if ((CSB_SUPPORT) || (PCA_SUPPORT))
    {LB_SSCAN_END_IND, (ke_msg_func_t)lb_sscan_end_ind_handler },
    {LB_STRAIN_END_IND, (ke_msg_func_t)lb_strain_end_ind_handler },
#endif // ((CSB_SUPPORT) || (PCA_SUPPORT))

#if (PCA_SUPPORT)
    {LB_PCA_END_IND, (ke_msg_func_t)lb_pca_end_ind_handler},
#if (RW_BT_MWS_COEX)
    {LB_LOCAL_PCA_REQ, (ke_msg_func_t) lb_local_pca_req_handler },
    {LB_PCA_MONITOR_INTV_TO, (ke_msg_func_t)lb_pca_monitor_intv_to_handler },
#endif // (RW_BT_MWS_COEX)

    /*
     * ************** Msg LB->LB****************
     */
    {LB_PCA_TX_INTV_TO, (ke_msg_func_t)lb_pca_tx_intv_to_handler},
#endif // (PCA_SUPPORT)

    {HCI_COMMAND, (ke_msg_func_t) hci_command_lb_handler       },
    {HCI_ACL_DATA, (ke_msg_func_t) hci_acl_data_lb_handler},
};

/// Specifies the message handlers that are common to all states.
//KE_MSG_STATE(lb)

/// Defines the place holder for the states of all the task instances.
ke_state_t lb_state[LB_IDX_MAX];

/// LLC task descriptor
const struct ke_task_desc TASK_DESC_LB = {lb_msg_handler_tab, lb_state, LB_IDX_MAX, ARRAY_LEN(lb_msg_handler_tab)};

/// @} LBTASK
