/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 ****************************************************************************************
 *
 * @file rwnx_msg_rx.c
 *
 * @brief RX function definitions
 *
 * Copyright (C) RivieraWaves 2012-2019
 *
 ****************************************************************************************
 */
#include "co_utils.h"
#include "lmac_msg.h"
#include "cfgrwnx.h"
#include "fhost.h"
#include "fhost_tx.h"
#include "fhost_cntrl.h"
#include "fhost_config.h"
#include "rwnx_defs.h"
#include "rwnx_utils.h"
#include "aic_plat_log.h"
#include "sys_al.h"

extern uint8_t mac_vif_index;

static int rwnx_freq_to_idx(struct rwnx_hw *rwnx_hw, int freq)
{
    int i, nb_chan, idx = 0;
    struct mac_chan_def *chans;

    if (freq < PHY_FREQ_5G) {
        chans = fhost_chan.chan2G4;
        nb_chan = fhost_chan.chan2G4_cnt;
    } else {
        chans = fhost_chan.chan5G;
        nb_chan = fhost_chan.chan5G_cnt;
        idx = fhost_chan.chan2G4_cnt;
    }

    for (i = 0; i < nb_chan; i++, chans++) {
        if (freq == chans->freq) {
            idx += i;
            break;
        }
    }

    return idx;
}

/***************************************************************************
 * Messages from MM task
 **************************************************************************/
static inline int rwnx_rx_chan_pre_switch_ind(struct rwnx_hw *rwnx_hw,
                                              struct rwnx_cmd *cmd,
                                              struct e2a_msg *msg)
{
#if 1
    RWNX_DBG(RWNX_FN_ENTRY_STR);
    //struct mm_channel_pre_switch_ind *ind = (struct mm_channel_pre_switch_ind *)msg->param;
    //int chan_idx = ind->chan_index;

    fhost_txq_vif_stop(0, TXQ_STOP_CHAN); // TODO:
#endif
    return 0;
}

static inline int rwnx_rx_chan_switch_ind(struct rwnx_hw *rwnx_hw,
                                          struct rwnx_cmd *cmd,
                                          struct e2a_msg *msg)
{
    #if 1
    RWNX_DBG(RWNX_FN_ENTRY_STR);
    struct mm_channel_switch_ind *ind = (struct mm_channel_switch_ind *)msg->param;
    int chan_idx = ind->chan_index;
    bool roc     = ind->roc;
    bool roc_tdls = ind->roc_tdls;
    uint8_t vif_index = ind->vif_index;
    struct fhost_vif_tag *vif = NULL;

    vif = &fhost_env.vif[vif_index];

    if (roc_tdls) {

    } else if (!roc) {
        fhost_txq_vif_start(0, TXQ_STOP_CHAN); // TODO:
    } else {

    }
    vif->chan_index = chan_idx;
    fhost_tx_env.to_times = 0;
    #endif
    return 0;
}

static inline int rwnx_rx_tdls_chan_switch_cfm(struct rwnx_hw *rwnx_hw,
                                                struct rwnx_cmd *cmd,
                                                struct e2a_msg *msg)
{
 //   RWNX_DBG(RWNX_FN_ENTRY_STR);
    return 0;
}

static inline int rwnx_rx_tdls_chan_switch_ind(struct rwnx_hw *rwnx_hw,
                                               struct rwnx_cmd *cmd,
                                               struct e2a_msg *msg)
{
   // RWNX_DBG(RWNX_FN_ENTRY_STR);

    // Enable traffic on OFF channel queue
    //rwnx_txq_offchan_start(rwnx_hw);

    return 0;
}

static inline int rwnx_rx_tdls_chan_switch_base_ind(struct rwnx_hw *rwnx_hw,
                                                    struct rwnx_cmd *cmd,
                                                    struct e2a_msg *msg)
{
    #if 0
    RWNX_DBG(RWNX_FN_ENTRY_STR);


    struct rwnx_vif *rwnx_vif;
    u8 vif_index = ((struct tdls_chan_switch_base_ind *)msg->param)->vif_index;


    list_for_each_entry(rwnx_vif, &rwnx_hw->vifs, list) {
        if (rwnx_vif->vif_index == vif_index) {
            rwnx_vif->roc_tdls = false;
            rwnx_txq_tdls_sta_stop(rwnx_vif, RWNX_TXQ_STOP_CHAN, rwnx_hw);
        }
    }
    #endif
    return 0;
}

static inline int rwnx_rx_tdls_peer_ps_ind(struct rwnx_hw *rwnx_hw,
                                           struct rwnx_cmd *cmd,
                                           struct e2a_msg *msg)
{
    #if 0
    RWNX_DBG(RWNX_FN_ENTRY_STR);


    struct rwnx_vif *rwnx_vif;
    u8 vif_index = ((struct tdls_peer_ps_ind *)msg->param)->vif_index;
    bool ps_on = ((struct tdls_peer_ps_ind *)msg->param)->ps_on;

    list_for_each_entry(rwnx_vif, &rwnx_hw->vifs, list) {
        if (rwnx_vif->vif_index == vif_index) {
            rwnx_vif->sta.tdls_sta->tdls.ps_on = ps_on;
            // Update PS status for the TDLS station
            rwnx_ps_bh_enable(rwnx_hw, rwnx_vif->sta.tdls_sta, ps_on);
        }
    }

    #endif
    return 0;
}

static inline int rwnx_rx_remain_on_channel_exp_ind(struct rwnx_hw *rwnx_hw,
                                                    struct rwnx_cmd *cmd,
                                                    struct e2a_msg *msg)
{
    #if 0
    aic_dbg(RWNX_FN_ENTRY_STR);

    /* Retrieve the allocated RoC element */
    struct rwnx_roc_elem *roc_elem = rwnx_hw->roc_elem;
    /* Get VIF on which RoC has been started */
    struct rwnx_vif *rwnx_vif = netdev_priv(roc_elem->wdev->netdev);


    /* For debug purpose (use ftrace kernel option) */
    trace_roc_exp(rwnx_vif->vif_index);

    /* If mgmt_roc is true, remain on channel has been started by ourself */
    /* If RoC has been cancelled before we switched on channel, do not call cfg80211 */
    if (!roc_elem->mgmt_roc && roc_elem->on_chan) {
        /* Inform the host that off-channel period has expired */
        cfg80211_remain_on_channel_expired(roc_elem->wdev, (u64)(rwnx_hw->roc_cookie_cnt),
                                           roc_elem->chan, GFP_ATOMIC);
    }

    /* De-init offchannel TX queue */
    rwnx_txq_offchan_deinit(rwnx_vif);

    /* Increase the cookie counter cannot be zero */
    rwnx_hw->roc_cookie_cnt++;

    if (rwnx_hw->roc_cookie_cnt == 0) {
        rwnx_hw->roc_cookie_cnt = 1;
    }

    /* Free the allocated RoC element */
    kfree(roc_elem);
    rwnx_hw->roc_elem = NULL;

    #endif
    return 0;
}

static inline int rwnx_rx_p2p_vif_ps_change_ind(struct rwnx_hw *rwnx_hw,
                                                struct rwnx_cmd *cmd,
                                                struct e2a_msg *msg)
{
    #if 0
    RWNX_DBG(RWNX_FN_ENTRY_STR);

    int vif_idx  = ((struct mm_p2p_vif_ps_change_ind *)msg->param)->vif_index;
    int ps_state = ((struct mm_p2p_vif_ps_change_ind *)msg->param)->ps_state;
    struct rwnx_vif *vif_entry;


    vif_entry = rwnx_hw->vif_table[vif_idx];

    if (vif_entry) {
        goto found_vif;
    }

    goto exit;

found_vif:

    if (ps_state == MM_PS_MODE_OFF) {
        // Start TX queues for provided VIF
        rwnx_txq_vif_start(vif_entry, RWNX_TXQ_STOP_VIF_PS, rwnx_hw);
    }
    else {
        // Stop TX queues for provided VIF
        rwnx_txq_vif_stop(vif_entry, RWNX_TXQ_STOP_VIF_PS, rwnx_hw);
    }

exit:
    #endif
    return 0;
}

static inline int rwnx_rx_channel_survey_ind(struct rwnx_hw *rwnx_hw,
                                             struct rwnx_cmd *cmd,
                                             struct e2a_msg *msg)
{
    RWNX_DBG(RWNX_FN_ENTRY_STR);

    struct mm_channel_survey_ind *ind = (struct mm_channel_survey_ind *)msg->param;
    // Get the channel index
    int idx = rwnx_freq_to_idx(rwnx_hw, ind->freq);
    // Get the survey
    struct rwnx_survey_info *rwnx_survey = &rwnx_hw->survey[idx];

    // Store the received parameters
    rwnx_survey->chan_time_ms = ind->chan_time_ms;
    rwnx_survey->chan_time_busy_ms = ind->chan_time_busy_ms;
    rwnx_survey->noise_dbm = ind->noise_dbm;
    rwnx_survey->filled = (SURVEY_INFO_TIME |
                           SURVEY_INFO_TIME_BUSY);

    if (ind->noise_dbm != 0) {
        rwnx_survey->filled |= SURVEY_INFO_NOISE_DBM;
    }

    return 0;
}

static inline int rwnx_rx_p2p_noa_upd_ind(struct rwnx_hw *rwnx_hw,
                                          struct rwnx_cmd *cmd,
                                          struct e2a_msg *msg)
{
    //RWNX_DBG(RWNX_FN_ENTRY_STR);
    return 0;
}

static inline int rwnx_rx_rssi_status_ind(struct rwnx_hw *rwnx_hw,
                                          struct rwnx_cmd *cmd,
                                          struct e2a_msg *msg)
{
    #if 0
    RWNX_DBG(RWNX_FN_ENTRY_STR);

    struct mm_rssi_status_ind *ind = (struct mm_rssi_status_ind *)msg->param;
    int vif_idx  = ind->vif_index;
    bool rssi_status = ind->rssi_status;

    struct rwnx_vif *vif_entry;


    vif_entry = rwnx_hw->vif_table[vif_idx];
    if (vif_entry) {
        cfg80211_cqm_rssi_notify(vif_entry->ndev,
                                 rssi_status ? NL80211_CQM_RSSI_THRESHOLD_EVENT_LOW :
                                               NL80211_CQM_RSSI_THRESHOLD_EVENT_HIGH,
                                 ind->rssi, GFP_ATOMIC);
    }
    #endif
    return 0;
}

static inline int rwnx_rx_pktloss_notify_ind(struct rwnx_hw *rwnx_hw,
                                             struct rwnx_cmd *cmd,
                                             struct e2a_msg *msg)
{
#if 0
    aic_dbg(RWNX_FN_ENTRY_STR);
    struct mm_pktloss_ind *ind = (struct mm_pktloss_ind *)msg->param;
    struct rwnx_vif *vif_entry;
    int vif_idx  = ind->vif_index;


    vif_entry = rwnx_hw->vif_table[vif_idx];
    if (vif_entry) {
        cfg80211_cqm_pktloss_notify(vif_entry->ndev, (const u8 *)ind->mac_addr.array,
                                    ind->num_packets, GFP_ATOMIC);
    }
#endif /* CONFIG_RWNX_FULLMAC */

    return 0;
}

extern int rwnx_apm_staloss_notify(struct mm_apm_staloss_ind *ind);

static inline int rwnx_apm_staloss_ind(struct rwnx_hw *rwnx_hw,
                                       struct rwnx_cmd *cmd,
                                       struct e2a_msg *msg)
{
    struct mm_apm_staloss_ind *ind = (struct mm_apm_staloss_ind *)msg->param;
    int ret;

    RWNX_DBG(RWNX_FN_ENTRY_STR);

    //queue_work(rwnx_hw->apmStaloss_wq, &rwnx_hw->apmStalossWork);
    rwnx_apm_staloss_notify(ind);

    return 0;
}

static inline int rwnx_rx_csa_counter_ind(struct rwnx_hw *rwnx_hw,
                                          struct rwnx_cmd *cmd,
                                          struct e2a_msg *msg)
{
#if 0
    aic_dbg(RWNX_FN_ENTRY_STR);
    struct mm_csa_counter_ind *ind = (struct mm_csa_counter_ind *)msg->param;
    if(mac_vif_index != ind->vif_index) {
        aic_dbg("%s %d %d\r\n", __func__, mac_vif_index, ind->vif_index);
        return -1;
    }
    struct fhost_vif_tag *fhost_vif = fhost_from_mac_vif(ind->vif_index);

    if(ind->vif_index >= NX_VIRT_DEV_MAX) {
        return -1;
    }
    fhost_vif->mac_vif->u.ap.csa_count = ind->csa_count;
#endif
    return 0;
}

static inline int rwnx_rx_csa_finish_ind(struct rwnx_hw *rwnx_hw,
                                         struct rwnx_cmd *cmd,
                                         struct e2a_msg *msg)
{
    #if 0
    aic_dbg(RWNX_FN_ENTRY_STR);

    struct fhost_vif_tag *vif = NULL;
    struct mm_csa_finish_ind *ind = (struct mm_csa_finish_ind *)msg->param;

    if(ind->vif_index >= NX_VIRT_DEV_MAX) {
        return -1;
    }
    vif = &fhost_env.vif[ind->vif_index];
    if(VIF_AP == vif->mac_vif->type) {
        if(!ind->status) {
            vif->chan_index = ind->chan_idx;
            rtos_semaphore_signal(vif->mac_vif->u.ap.csa_semaphore, false);
        } else {
            aic_dbg("rwnx_rx_csa_finish_ind fail\r\n");
        }
    } else {
        if (ind->status == 0) {
            if (vif->chan_index == ind->chan_idx) {
                fhost_txq_vif_start(ind->vif_index, TXQ_STOP_CHAN);
            } else {
                fhost_txq_vif_stop(ind->vif_index, TXQ_STOP_CHAN);
            }
        }
    }
#endif
    return 0;
}

static inline int rwnx_rx_csa_traffic_ind(struct rwnx_hw *rwnx_hw,
                                          struct rwnx_cmd *cmd,
                                          struct e2a_msg *msg)
{
#if 0
    aic_dbg(RWNX_FN_ENTRY_STR);
    struct mm_csa_traffic_ind *ind = (struct mm_csa_traffic_ind *)msg->param;

    if(ind->vif_index >= NX_VIRT_DEV_MAX) {
        return -1;
    }

    if (ind->enable) {
        fhost_txq_vif_start(ind->vif_index, TXQ_STOP_CSA);
    } else {
        fhost_txq_vif_stop(ind->vif_index, TXQ_STOP_CSA);
    }
#endif
    return 0;
}
#ifdef CFG_SOFTAP
static inline int rwnx_rx_ps_change_ind(struct rwnx_hw *rwnx_hw,
                                        struct rwnx_cmd *cmd,
                                        struct e2a_msg *msg)
{
    //RWNX_DBG(RWNX_FN_ENTRY_STR);
    if ((rwnx_hw->chipid == PRODUCT_ID_AIC8800DC) ||
        (rwnx_hw->chipid == PRODUCT_ID_AIC8800DW) ||
        (rwnx_hw->chipid == PRODUCT_ID_AIC8800D80)) {
        //aic_dbg("%s, wifi driver do not handle ps_change_ind\r\n", __func__);
        return 0;
    }

    struct mm_ps_change_ind *ind = (struct mm_ps_change_ind *)msg->param;

    fhost_tx_sta_ps_enable(ind->sta_idx, ind->ps_state);

    return 0;
}

static inline int rwnx_rx_traffic_req_ind(struct rwnx_hw *rwnx_hw,
                                          struct rwnx_cmd *cmd,
                                          struct e2a_msg *msg)
{
    //RWNX_DBG(RWNX_FN_ENTRY_STR);
    struct mm_traffic_req_ind *ind = (struct mm_traffic_req_ind *)msg->param;

    fhost_tx_do_ps_traffic_req(ind->sta_idx, ind->pkt_cnt, (enum fhost_tx_ps_type)ind->uapsd);

    return 0;
}

#endif /* CFG_SOFTAP */

/***************************************************************************
 * Messages from SCANU task
 **************************************************************************/
static inline int rwnx_rx_scanu_start_cfm(struct rwnx_hw *rwnx_hw,
                                          struct rwnx_cmd *cmd,
                                          struct e2a_msg *msg)
{
    #if 1
    RWNX_DBG(RWNX_FN_ENTRY_STR);

    struct cfgrwnx_scan_completed resp;
    struct scanu_start_cfm *cfm = (struct scanu_start_cfm *)msg->param;
    if(mac_vif_index != cfm->vif_idx) {
        aic_dbg("%s %d %d\r\n", __func__, mac_vif_index, cfm->vif_idx);
        return -1;
    }
    struct fhost_vif_tag *fhost_vif = fhost_from_mac_vif(cfm->vif_idx);

    resp.hdr.id = CFGRWNX_SCAN_DONE_EVENT;
    resp.hdr.len = sizeof(resp);
    resp.result_cnt = cfm->result_cnt;

    if (cfm->status == CO_OK)
        resp.status = CFGRWNX_SUCCESS;
    else
        resp.status = CFGRWNX_ERROR;

    fhost_cntrl_cfgrwnx_event_send(&resp.hdr, fhost_vif->scan_sock);
    #endif
    return 0;
}

static inline int rwnx_rx_scanu_result_ind(struct rwnx_hw *rwnx_hw,
                                           struct rwnx_cmd *cmd,
                                           struct e2a_msg *msg)
{
    #if 1
    RWNX_DBG(RWNX_FN_ENTRY_STR);
    struct cfgrwnx_scan_result result;
    struct scanu_result_ind *ind = (struct scanu_result_ind *)msg->param;
    if(mac_vif_index != ind->inst_nbr) {
        aic_dbg("%s %d %d\r\n", __func__, mac_vif_index, ind->inst_nbr);
        return 0;
    }

    struct fhost_vif_tag *fhost_vif = fhost_from_mac_vif(ind->inst_nbr);

    result.hdr.id = CFGRWNX_SCAN_RESULT_EVENT;
    result.hdr.len = sizeof(struct cfgrwnx_scan_result);

    result.fhost_vif_idx = CO_GET_INDEX(fhost_vif, fhost_env.vif);
    result.freq = ind->center_freq;
    result.rssi = ind->rssi;
    result.length = ind->length;
    result.payload = rtos_malloc(ind->length);
    if (result.payload == NULL)
        return 0;
    fhost_scan_frame_handler(ind);
    memcpy(result.payload, ind->payload, ind->length);

    if (fhost_cntrl_cfgrwnx_event_send(&result.hdr, fhost_vif->scan_sock)) {
        rtos_free(result.payload);
    }
    #endif
    return 0;
}

/***************************************************************************
 * Messages from ME task
 **************************************************************************/
static inline int rwnx_rx_me_tkip_mic_failure_ind(struct rwnx_hw *rwnx_hw,
                                                  struct rwnx_cmd *cmd,
                                                  struct e2a_msg *msg)
{
    #if 0
    RWNX_DBG(RWNX_FN_ENTRY_STR);

    struct cfgrwnx_mic_failure_event event;
    struct me_tkip_mic_failure_ind *ind = (struct me_tkip_mic_failure_ind *)msg->param;
    if(mac_vif_index != ind->vif_idx) {
        aic_dbg("%s %d %d\r\n", __func__, mac_vif_index, ind->vif_idx);
        return -1;
    }
    struct fhost_vif_tag *fhost_vif = fhost_from_mac_vif(ind->vif_idx);

    event.hdr.id = CFGRWNX_MIC_FAILURE_EVENT;
    event.hdr.len = sizeof(event);

    MAC_ADDR_CPY(&event.addr, &ind->addr);
    event.ga = ind->ga;
    event.fhost_vif_idx = CO_GET_INDEX(fhost_vif, fhost_env.vif);

    fhost_cntrl_cfgrwnx_event_send(&event.hdr, fhost_vif->conn_sock);
    #endif
    return 0;
}

static inline int rwnx_rx_me_tx_credits_update_ind(struct rwnx_hw *rwnx_hw,
                                                   struct rwnx_cmd *cmd,
                                                   struct e2a_msg *msg)
{
    //RWNX_DBG(RWNX_FN_ENTRY_STR);

    //struct me_tx_credits_update_ind *ind = (struct me_tx_credits_update_ind *)msg->param;

    //fhost_tx_do_credits_update(ind->sta_idx, ind->tid, ind->credits);

    return 0;
}

uint8_t sta_index = 0;
/***************************************************************************
 * Messages from SM task
 **************************************************************************/
static inline int rwnx_rx_sm_connect_ind(struct rwnx_hw *rwnx_hw,
                                         struct rwnx_cmd *cmd,
                                         struct e2a_msg *msg)
{
    #if 1
    RWNX_DBG(RWNX_FN_ENTRY_STR);

    struct cfgrwnx_connect_event event;
    struct sm_connect_ind *param = (struct sm_connect_ind *)msg->param;
    if(mac_vif_index != param->vif_idx) {
        aic_dbg("%s %d %d\r\n", __func__, mac_vif_index, param->vif_idx);
        return -1;
    }
    struct fhost_vif_tag *fhost_vif = fhost_from_mac_vif(param->vif_idx);
    int req_resp_ies_len = param->assoc_req_ie_len + param->assoc_rsp_ie_len;

    event.hdr.id = CFGRWNX_CONNECT_EVENT;
    event.hdr.len = sizeof(event);

    MAC_ADDR_CPY(&event.bssid, &param->bssid);
    event.status_code = param->status_code;
    event.freq = param->center_freq;
    event.assoc_req_ie_len = param->assoc_req_ie_len;
    event.assoc_resp_ie_len = param->assoc_rsp_ie_len;
    event.sta_idx = param->ap_idx;

    if (req_resp_ies_len)
    {
        event.req_resp_ies = rtos_malloc(req_resp_ies_len);
        if (event.req_resp_ies == NULL)
        {
            TRACE_FHOST("Failed to allocate assoc IE");
            return 0;
        }

        memcpy(event.req_resp_ies, param->assoc_ie_buf, req_resp_ies_len);
    }
    else
    {
        event.req_resp_ies = NULL;
    }

    if (fhost_cntrl_cfgrwnx_event_send(&event.hdr, fhost_vif->conn_sock) &&
        event.req_resp_ies)
    {
        rtos_free(event.req_resp_ies);
        return 0;
    }

    if (param->status_code == MAC_ST_SUCCESSFUL)
    {
        fhost_vif->ap_id = param->ap_idx;
        fhost_vif->acm = param->acm;

        //fhost_tx_sta_add(fhost_vif->ap_id);
        fhost_vif->mac_vif->u.sta.ap_id = param->ap_idx;
        struct sta_info_tag *sta = (struct sta_info_tag*)co_list_pop_front(&free_sta_list);
        sta->inst_nbr = fhost_vif->mac_vif->index;
        sta->staid = param->ap_idx;
        sta->valid =  true;
        MAC_ADDR_CPY(&sta->mac_addr, &param->bssid);
        MAC_ADDR_CPY(&(vif_info_tab[fhost_vif->mac_vif->index].bss_info.bssid), &param->bssid);
        sta_index = CO_GET_INDEX(fhost_vif->mac_vif, vif_info_tab);
        aic_dbg("Connect %d %x:%x:%x, sta_index %d\n", fhost_vif->ap_id, sta->mac_addr.array[0], sta->mac_addr.array[1],sta->mac_addr.array[2], sta_index);

        fhost_vif->mac_vif->active = true;

        fhost_tx_do_sta_add(fhost_vif->ap_id);

    }
    #endif
    return 0;
}

static inline int rwnx_rx_sm_disconnect_ind(struct rwnx_hw *rwnx_hw,
                                            struct rwnx_cmd *cmd,
                                            struct e2a_msg *msg)
{
    RWNX_DBG(RWNX_FN_ENTRY_STR);

    struct cfgrwnx_disconnect_event event;
    struct sm_disconnect_ind *param = (struct sm_disconnect_ind *)msg->param;
    if(mac_vif_index != param->vif_idx) {
        aic_dbg("%s %d %d\r\n", __func__, mac_vif_index, param->vif_idx);
        return -1;
    }
    struct fhost_vif_tag *fhost_vif = fhost_from_mac_vif(param->vif_idx);

    event.hdr.id = CFGRWNX_DISCONNECT_EVENT;
    event.hdr.len = sizeof(event);

    event.fhost_vif_idx = CO_GET_INDEX(fhost_vif, fhost_env.vif);
    event.reason_code = param->reason_code;

    fhost_tx_do_sta_del(fhost_vif->ap_id);

    fhost_cntrl_cfgrwnx_event_send(&event.hdr, fhost_vif->conn_sock);
    fhost_vif->conn_sock = -1;

    fhost_vif->mac_vif->active = false;

    struct sta_info_tag *sta = vif_mgmt_get_sta_by_staid(fhost_vif->ap_id);

    if (sta) {
#if (AICWF_RX_REORDER)
		reord_deinit_sta_by_mac((uint8_t *)(sta->mac_addr.array));
#endif
        aic_dbg("Disconnect %d %x:%x:%x\n", fhost_vif->ap_id, sta->mac_addr.array[0], sta->mac_addr.array[1],sta->mac_addr.array[2]);
        sta->valid = false;
        memset(sta, 0, sizeof *sta);
        sta->staid = 0xFF;
        co_list_push_back(&free_sta_list, (struct co_list_hdr*)sta);
    }
    fhost_vif->ap_id = INVALID_STA_IDX;

    wlan_connected = 0;

    if (fhost_mac_status_get_callback)
        fhost_mac_status_get_callback(WIFI_MAC_STATUS_DISCONNECTED);

    return 0;
}

static inline int rwnx_rx_sm_external_auth_required_ind(struct rwnx_hw *rwnx_hw,
                                                        struct rwnx_cmd *cmd,
                                                        struct e2a_msg *msg)
{
    #if 1
    RWNX_DBG(RWNX_FN_ENTRY_STR);

    struct cfgrwnx_external_auth_event event;
    struct sm_external_auth_required_ind *param = (struct sm_external_auth_required_ind *)msg->param;
    if(mac_vif_index != param->vif_idx) {
        aic_dbg("%s %d %d\r\n", __func__, mac_vif_index, param->vif_idx);
        return -1;
    }
    struct fhost_vif_tag *fhost_vif = fhost_from_mac_vif(param->vif_idx);

    event.hdr.id = CFGRWNX_EXTERNAL_AUTH_EVENT;
    event.hdr.len = sizeof(event);

    event.fhost_vif_idx = CO_GET_INDEX(fhost_vif, fhost_env.vif);
    event.bssid = param->bssid;
    event.ssid = param->ssid;
    event.akm = param->akm;

    fhost_cntrl_cfgrwnx_event_send(&event.hdr, fhost_vif->conn_sock);
    #endif
    return 0;
}


static inline int rwnx_rx_mesh_path_create_cfm(struct rwnx_hw *rwnx_hw,
                                               struct rwnx_cmd *cmd,
                                               struct e2a_msg *msg)
{
    //RWNX_DBG(RWNX_FN_ENTRY_STR);
    #if 0
    struct mesh_path_create_cfm *cfm = (struct mesh_path_create_cfm *)msg->param;
    struct rwnx_vif *rwnx_vif = rwnx_hw->vif_table[cfm->vif_idx];


    /* Check we well have a Mesh Point Interface */
    if (rwnx_vif && (RWNX_VIF_TYPE(rwnx_vif) == NL80211_IFTYPE_MESH_POINT)) {
        rwnx_vif->ap.create_path = false;
    }

    #endif
    return 0;
}

static inline int rwnx_rx_mesh_peer_update_ind(struct rwnx_hw *rwnx_hw,
                                               struct rwnx_cmd *cmd,
                                               struct e2a_msg *msg)
{
    //RWNX_DBG(RWNX_FN_ENTRY_STR);
    #if 0
    struct mesh_peer_update_ind *ind = (struct mesh_peer_update_ind *)msg->param;
    struct rwnx_vif *rwnx_vif = rwnx_hw->vif_table[ind->vif_idx];
    struct rwnx_sta *rwnx_sta = &rwnx_hw->sta_table[ind->sta_idx];


    if ((ind->vif_idx >= (NX_VIRT_DEV_MAX + NX_REMOTE_STA_MAX)) ||
        (rwnx_vif && (RWNX_VIF_TYPE(rwnx_vif) != NL80211_IFTYPE_MESH_POINT)) ||
        (ind->sta_idx >= NX_REMOTE_STA_MAX))
        return 1;

    /* Check we well have a Mesh Point Interface */
    if (!rwnx_vif->user_mpm)
    {
        /* Check if peer link has been established or lost */
        if (ind->estab) {
            if (!rwnx_sta->valid) {
                u8 txq_status;

                rwnx_sta->valid = true;
                rwnx_sta->sta_idx = ind->sta_idx;
                rwnx_sta->ch_idx = rwnx_vif->ch_index;
                rwnx_sta->vif_idx = ind->vif_idx;
                rwnx_sta->vlan_idx = rwnx_sta->vif_idx;
                rwnx_sta->ps.active = false;
                rwnx_sta->qos = true;
                rwnx_sta->aid = ind->sta_idx + 1;
                //rwnx_sta->acm = ind->acm;
                memcpy(rwnx_sta->mac_addr, ind->peer_addr.array, ETH_ALEN);

                rwnx_chanctx_link(rwnx_vif, rwnx_sta->ch_idx, NULL);

                /* Add the station in the list of VIF's stations */
                INIT_LIST_HEAD(&rwnx_sta->list);
                list_add_tail(&rwnx_sta->list, &rwnx_vif->ap.sta_list);

                /* Initialize the TX queues */
                if (rwnx_sta->ch_idx == rwnx_hw->cur_chanctx) {
                    txq_status = 0;
                } else {
                    txq_status = RWNX_TXQ_STOP_CHAN;
                }

                rwnx_txq_sta_init(rwnx_hw, rwnx_sta, txq_status);
                rwnx_dbgfs_register_rc_stat(rwnx_hw, rwnx_sta);

#ifdef CONFIG_RWNX_BFMER
                // TODO: update indication to contains vht capabilties
                if (rwnx_hw->mod_params->bfmer)
                    rwnx_send_bfmer_enable(rwnx_hw, rwnx_sta, NULL);

                rwnx_mu_group_sta_init(rwnx_sta, NULL);
#endif /* CONFIG_RWNX_BFMER */

            } else {
                WARN_ON(0);
            }
        } else {
            if (rwnx_sta->valid) {
                rwnx_sta->ps.active = false;
                rwnx_sta->valid = false;

                /* Remove the station from the list of VIF's station */
                list_del_init(&rwnx_sta->list);

                rwnx_txq_sta_deinit(rwnx_hw, rwnx_sta);
                rwnx_dbgfs_unregister_rc_stat(rwnx_hw, rwnx_sta);
            } else {
                WARN_ON(0);
            }
        }
    } else {
        if (!ind->estab && rwnx_sta->valid) {
            /* There is no way to inform upper layer for lost of peer, still
               clean everything in the driver */
            rwnx_sta->ps.active = false;
            rwnx_sta->valid = false;

            /* Remove the station from the list of VIF's station */
            list_del_init(&rwnx_sta->list);

            rwnx_txq_sta_deinit(rwnx_hw, rwnx_sta);
            rwnx_dbgfs_unregister_rc_stat(rwnx_hw, rwnx_sta);
        } else {
            WARN_ON(0);
        }
    }

    #endif
    return 0;
}

static inline int rwnx_rx_mesh_path_update_ind(struct rwnx_hw *rwnx_hw,
                                               struct rwnx_cmd *cmd,
                                               struct e2a_msg *msg)
{
    //RWNX_DBG(RWNX_FN_ENTRY_STR);
    #if 0
    struct mesh_path_update_ind *ind = (struct mesh_path_update_ind *)msg->param;
    struct rwnx_vif *rwnx_vif = rwnx_hw->vif_table[ind->vif_idx];
    struct rwnx_mesh_path *mesh_path;
    bool found = false;


    if (ind->vif_idx >= (NX_VIRT_DEV_MAX + NX_REMOTE_STA_MAX))
        return 1;

    if (!rwnx_vif || (RWNX_VIF_TYPE(rwnx_vif) != NL80211_IFTYPE_MESH_POINT))
        return 0;

    /* Look for path with provided target address */
    list_for_each_entry(mesh_path, &rwnx_vif->ap.mpath_list, list) {
        if (mesh_path->path_idx == ind->path_idx) {
            found = true;
            break;
        }
    }

    /* Check if element has been deleted */
    if (ind->delete) {
        if (found) {
            trace_mesh_delete_path(mesh_path);
            /* Remove element from list */
            list_del_init(&mesh_path->list);
            /* Free the element */
            kfree(mesh_path);
        }
    }
    else {
        if (found) {
            // Update the Next Hop STA
            mesh_path->p_nhop_sta = &rwnx_hw->sta_table[ind->nhop_sta_idx];
            trace_mesh_update_path(mesh_path);
        } else {
            // Allocate a Mesh Path structure
            mesh_path = (struct rwnx_mesh_path *)kmalloc(sizeof(struct rwnx_mesh_path), GFP_ATOMIC);

            if (mesh_path) {
                INIT_LIST_HEAD(&mesh_path->list);

                mesh_path->path_idx = ind->path_idx;
                mesh_path->p_nhop_sta = &rwnx_hw->sta_table[ind->nhop_sta_idx];
                memcpy(&mesh_path->tgt_mac_addr, &ind->tgt_mac_addr, MAC_ADDR_LEN);

                // Insert the path in the list of path
                list_add_tail(&mesh_path->list, &rwnx_vif->ap.mpath_list);

                trace_mesh_create_path(mesh_path);
            }
        }
    }

    #endif
    return 0;
}

static inline int rwnx_rx_mesh_proxy_update_ind(struct rwnx_hw *rwnx_hw,
                                               struct rwnx_cmd *cmd,
                                               struct e2a_msg *msg)
{
    //RWNX_DBG(RWNX_FN_ENTRY_STR);

    #if 0
    struct mesh_proxy_update_ind *ind = (struct mesh_proxy_update_ind *)msg->param;
    struct rwnx_vif *rwnx_vif = rwnx_hw->vif_table[ind->vif_idx];
    struct rwnx_mesh_proxy *mesh_proxy;
    bool found = false;


    if (ind->vif_idx >= (NX_VIRT_DEV_MAX + NX_REMOTE_STA_MAX))
        return 1;

    if (!rwnx_vif || (RWNX_VIF_TYPE(rwnx_vif) != NL80211_IFTYPE_MESH_POINT))
        return 0;

    /* Look for path with provided external STA address */
    list_for_each_entry(mesh_proxy, &rwnx_vif->ap.proxy_list, list) {
        if (!memcmp(&ind->ext_sta_addr, &mesh_proxy->ext_sta_addr, ETH_ALEN)) {
            found = true;
            break;
        }
    }

    if (ind->delete && found) {
        /* Delete mesh path */
        list_del_init(&mesh_proxy->list);
        kfree(mesh_proxy);
    } else if (!ind->delete && !found) {
        /* Allocate a Mesh Path structure */
        mesh_proxy = (struct rwnx_mesh_proxy *)kmalloc(sizeof(*mesh_proxy),
                                                       GFP_ATOMIC);

        if (mesh_proxy) {
            INIT_LIST_HEAD(&mesh_proxy->list);

            memcpy(&mesh_proxy->ext_sta_addr, &ind->ext_sta_addr, MAC_ADDR_LEN);
            mesh_proxy->local = ind->local;

            if (!ind->local) {
                memcpy(&mesh_proxy->proxy_addr, &ind->proxy_mac_addr, MAC_ADDR_LEN);
            }

            /* Insert the path in the list of path */
            list_add_tail(&mesh_proxy->list, &rwnx_vif->ap.proxy_list);
        }
    }

    #endif
    return 0;
}

/***************************************************************************
 * Messages from APM task
 **************************************************************************/


/***************************************************************************
 * Messages from DEBUG task
 **************************************************************************/
static inline int rwnx_rx_dbg_error_ind(struct rwnx_hw *rwnx_hw,
                                        struct rwnx_cmd *cmd,
                                        struct e2a_msg *msg)
{
 //   RWNX_DBG(RWNX_FN_ENTRY_STR);

    //rwnx_error_ind(rwnx_hw);

    return 0;
}

static msg_cb_fct mm_hdlrs[MSG_I(MM_MAX)] = {
    [MSG_I(MM_CHANNEL_SWITCH_IND)]     = rwnx_rx_chan_switch_ind,
    [MSG_I(MM_CHANNEL_PRE_SWITCH_IND)] = rwnx_rx_chan_pre_switch_ind,
    [MSG_I(MM_REMAIN_ON_CHANNEL_EXP_IND)] = rwnx_rx_remain_on_channel_exp_ind,
    #ifdef CFG_SOFTAP
    [MSG_I(MM_PS_CHANGE_IND)]          = rwnx_rx_ps_change_ind,
    [MSG_I(MM_TRAFFIC_REQ_IND)]        = rwnx_rx_traffic_req_ind,
    [MSG_I(MM_CSA_COUNTER_IND)]        = rwnx_rx_csa_counter_ind,
    [MSG_I(MM_CSA_FINISH_IND)]         = rwnx_rx_csa_finish_ind,
    [MSG_I(MM_CSA_TRAFFIC_IND)]        = rwnx_rx_csa_traffic_ind,
    #endif /* CFG_SOFTAP */
    [MSG_I(MM_CHANNEL_SURVEY_IND)]     = rwnx_rx_channel_survey_ind,
    #if NX_P2P
    [MSG_I(MM_P2P_VIF_PS_CHANGE_IND)]  = rwnx_rx_p2p_vif_ps_change_ind,
    [MSG_I(MM_P2P_NOA_UPD_IND)]        = rwnx_rx_p2p_noa_upd_ind,
    #endif
    [MSG_I(MM_RSSI_STATUS_IND)]        = rwnx_rx_rssi_status_ind,
    [MSG_I(MM_PKTLOSS_IND)]            = rwnx_rx_pktloss_notify_ind,
    [MSG_I(MM_APM_STALOSS_IND)]        = rwnx_apm_staloss_ind,
};

static msg_cb_fct scan_hdlrs[MSG_I(SCANU_MAX)] = {
    [MSG_I(SCANU_START_CFM)]           = rwnx_rx_scanu_start_cfm,
    [MSG_I(SCANU_RESULT_IND)]          = rwnx_rx_scanu_result_ind,
};

static msg_cb_fct me_hdlrs[MSG_I(ME_MAX)] = {
    [MSG_I(ME_TKIP_MIC_FAILURE_IND)] = rwnx_rx_me_tkip_mic_failure_ind,
    //[MSG_I(ME_TX_CREDITS_UPDATE_IND)] = rwnx_rx_me_tx_credits_update_ind,
};

static msg_cb_fct sm_hdlrs[MSG_I(SM_MAX)] = {
    [MSG_I(SM_CONNECT_IND)]    = rwnx_rx_sm_connect_ind,
    [MSG_I(SM_DISCONNECT_IND)] = rwnx_rx_sm_disconnect_ind,
    [MSG_I(SM_EXTERNAL_AUTH_REQUIRED_IND)] = rwnx_rx_sm_external_auth_required_ind,
};
#if 1

static msg_cb_fct apm_hdlrs[MSG_I(APM_MAX)] = {
};
static msg_cb_fct mesh_hdlrs[MSG_I(MESH_MAX)] = {
    [MSG_I(MESH_PATH_CREATE_CFM)]  = rwnx_rx_mesh_path_create_cfm,
    [MSG_I(MESH_PEER_UPDATE_IND)]  = rwnx_rx_mesh_peer_update_ind,
    [MSG_I(MESH_PATH_UPDATE_IND)]  = rwnx_rx_mesh_path_update_ind,
    [MSG_I(MESH_PROXY_UPDATE_IND)] = rwnx_rx_mesh_proxy_update_ind,
};
#endif

static msg_cb_fct dbg_hdlrs[MSG_I(DBG_MAX)] = {
    [MSG_I(DBG_ERROR_IND)]                = rwnx_rx_dbg_error_ind,
};

#if 0
static msg_cb_fct tdls_hdlrs[MSG_I(TDLS_MAX)] = {
    [MSG_I(TDLS_CHAN_SWITCH_CFM)] = rwnx_rx_tdls_chan_switch_cfm,
    [MSG_I(TDLS_CHAN_SWITCH_IND)] = rwnx_rx_tdls_chan_switch_ind,
    [MSG_I(TDLS_CHAN_SWITCH_BASE_IND)] = rwnx_rx_tdls_chan_switch_base_ind,
    [MSG_I(TDLS_PEER_PS_IND)] = rwnx_rx_tdls_peer_ps_ind,
};
#endif
static msg_cb_fct *msg_hdlrs[] = {
    [TASK_MM]    = mm_hdlrs,
    [TASK_DBG]   = dbg_hdlrs,
    //[TASK_TDLS]  = tdls_hdlrs,
    [TASK_SCANU] = scan_hdlrs,
    [TASK_ME]    = me_hdlrs,
    [TASK_SM]    = sm_hdlrs,
    [TASK_APM]   = apm_hdlrs,
    //[TASK_MESH]  = mesh_hdlrs,
};

/**
 *
 */
void rwnx_rx_handle_msg(struct rwnx_hw *rwnx_hw, struct e2a_msg *msg)
{
//    RWNX_DBG(RWNX_FN_ENTRY_STR);
    //aic_dbg("T %d, I %d\r\n", MSG_T(msg->id), MSG_I(msg->id));
    DBG_MACIF_VRB("RxM,%x\n", msg->id);
    rwnx_hw->cmd_mgr.msgind(&rwnx_hw->cmd_mgr, msg,
                            msg_hdlrs[MSG_T(msg->id)][MSG_I(msg->id)]);
}
