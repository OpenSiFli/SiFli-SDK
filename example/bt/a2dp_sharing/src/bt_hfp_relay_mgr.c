/*
 * SPDX-FileCopyrightText: 2026-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "bt_hfp_relay_mgr.h"
#include "bts2_app_inc.h"
#include "bt_connection_manager.h"
#include "bf0_sibles.h"

#ifdef CFG_BT_VOICE_RELAY
    #include "hfp_audio_api.h"
#endif

static rt_timer_t g_hfp_relay_clcc_timer = NULL;
static uint8_t g_hfp_relay_clcc_process_status = BT_HFP_RELAY_CLCC_STATUS_COMPLETE;

static bt_hfp_relay_context_t g_hfp_relay_ctx =
{
    .hf_channel = BT_HFP_RELAY_INVALID_CHANNEL,
    .ag_channel = BT_HFP_RELAY_INVALID_CHANNEL,
    .cind_status =
    {
        .service_status = 1,
        .call = 0,
        .callsetup = 0,
        .batt_level = 5,
        .signal = 5,
        .roam_status = 0,
        .callheld = 0,
    },
};

uint8_t bt_hfp_ag_set_auto_answer_call_default()
{
    return 0;
}

void bt_hfp_relay_mgr_init(void)
{
    memset(&g_hfp_relay_ctx, 0, sizeof(g_hfp_relay_ctx));
    g_hfp_relay_ctx.hf_channel = BT_HFP_RELAY_INVALID_CHANNEL;
    g_hfp_relay_ctx.ag_channel = BT_HFP_RELAY_INVALID_CHANNEL;
    g_hfp_relay_ctx.cind_status.service_status = 1;
    g_hfp_relay_ctx.cind_status.batt_level = 5;
    g_hfp_relay_ctx.cind_status.signal = 5;
    g_hfp_relay_ctx.remote_calls.calls = &g_hfp_relay_ctx.remote_call;
    g_hfp_relay_clcc_process_status = BT_HFP_RELAY_CLCC_STATUS_COMPLETE;
}

void bt_hfp_relay_handle_sco_event(uint16_t event_id, bt_notify_device_sco_info_t *sco_info)
{
    bt_hfp_relay_context_t *ctx = bt_hfp_relay_get_context();

    if ((NULL == sco_info) || (NULL == ctx))
    {
        return;
    }

    LOG_I("HFP relay sco event:%d type:%d channel:%d res:%d", event_id, sco_info->sco_type,
          sco_info->profile_channel, sco_info->sco_res);

    switch (event_id)
    {
    case BT_NOTIFY_COMMON_SCO_CONNECTED:
    {
#ifdef CFG_BT_VOICE_RELAY
        hfp_audio_relay_option(&sco_info->para, 1);
        if (sco_info->sco_type == BT_NOTIFY_HFP_HF)
        {
            bt_hfp_relay_context_t *ctx = bt_hfp_relay_get_context();
            if (!ctx->hfp_ag_sco_state)
            {
                BTS2S_BD_ADDR dest_addr;
                bt_addr_convert_to_bts((bd_addr_t *)&ctx->ag_mac, &dest_addr);
                bt_hfp_connect_audio(&dest_addr);
            }
        }
        else if (sco_info->sco_type == BT_NOTIFY_HFP_AG)
        {
            bt_hfp_relay_context_t *ctx = bt_hfp_relay_get_context();
            ctx->hfp_ag_sco_state = 1;
        }
#endif
        break;
    }

    case BT_NOTIFY_COMMON_SCO_DISCONNECTED:
    {
#ifdef CFG_BT_VOICE_RELAY
        hfp_audio_relay_option(&sco_info->para, 0);
        if (sco_info->sco_type == BT_NOTIFY_HFP_HF)
        {
            bt_hfp_relay_context_t *ctx = bt_hfp_relay_get_context();
            if (ctx->hfp_ag_sco_state)
            {
                BTS2S_BD_ADDR dest_addr;
                bt_addr_convert_to_bts((bd_addr_t *)&ctx->ag_mac, &dest_addr);
                bt_hfp_disconnect_audio(&dest_addr);
            }
        }
        else if (sco_info->sco_type == BT_NOTIFY_HFP_AG)
        {
            bt_hfp_relay_context_t *ctx = bt_hfp_relay_get_context();
            ctx->hfp_ag_sco_state = 0;
        }
#endif
        break;
    }
    default:
        break;
    }

}

bt_hfp_relay_context_t *bt_hfp_relay_get_context(void)
{
    return &g_hfp_relay_ctx;
}

uint16_t bt_hfp_relay_get_hf_channel(void)
{
    return g_hfp_relay_ctx.hf_channel;
}

uint16_t bt_hfp_relay_get_ag_channel(void)
{
    return g_hfp_relay_ctx.ag_channel;
}

void bt_hfp_relay_set_hf_channel(uint16_t channel, bt_notify_device_mac_t *mac)
{
    g_hfp_relay_ctx.hf_channel = channel;
    bmemcpy(&g_hfp_relay_ctx.hf_mac, mac, sizeof(bt_notify_device_mac_t));
}

void bt_hfp_relay_set_ag_channel(uint16_t channel, bt_notify_device_mac_t *mac)
{
    g_hfp_relay_ctx.ag_channel = channel;
    bmemcpy(&g_hfp_relay_ctx.ag_mac, mac, sizeof(bt_notify_device_mac_t));
}

void bt_hfp_relay_clear_hf_channel(uint16_t channel)
{
    if (g_hfp_relay_ctx.hf_channel == channel)
    {
        g_hfp_relay_ctx.hf_channel = BT_HFP_RELAY_INVALID_CHANNEL;
        bt_hfp_relay_reset_cached_call();
        memset(&g_hfp_relay_ctx.local_phone_num, 0, sizeof(g_hfp_relay_ctx.local_phone_num));
        g_hfp_relay_ctx.has_local_phone_num = 0;
        g_hfp_relay_ctx.pending_ag_make_call = 0;
    }
}

void bt_hfp_relay_clear_ag_channel(uint16_t channel)
{
    if (g_hfp_relay_ctx.ag_channel == channel)
    {
        g_hfp_relay_ctx.ag_channel = BT_HFP_RELAY_INVALID_CHANNEL;
        g_hfp_relay_ctx.pending_ag_make_call = 0;
    }
}

void bt_hfp_relay_reset_cached_call(void)
{
    memset(&g_hfp_relay_ctx.remote_call, 0, sizeof(g_hfp_relay_ctx.remote_call));
    g_hfp_relay_ctx.remote_calls.num_call = 0;
    g_hfp_relay_ctx.remote_calls.calls = &g_hfp_relay_ctx.remote_call;
    g_hfp_relay_ctx.has_remote_call = 0;
}

void bt_hfp_relay_cache_local_phone_num(uint8_t *number, uint16_t number_len)
{
    uint16_t number_size = number_len;

    memset(&g_hfp_relay_ctx.local_phone_num, 0, sizeof(g_hfp_relay_ctx.local_phone_num));
    if (number_size >= PHONE_NUM_LEN)
    {
        number_size = PHONE_NUM_LEN - 1;
    }
    memcpy(g_hfp_relay_ctx.local_phone_num.phone_number, number, number_size);
    g_hfp_relay_ctx.local_phone_num.phone_number[number_size] = '\0';
    g_hfp_relay_ctx.local_phone_num.type = PHONE_NUMBER_TYPE_UNKNOWN;
    g_hfp_relay_ctx.has_local_phone_num = 1;
}

void bt_hfp_relay_update_cind_from_all_status(bt_notify_all_call_status *call_status)
{
    g_hfp_relay_ctx.cind_status.call = call_status->call_status;
    g_hfp_relay_ctx.cind_status.callsetup = call_status->callsetup_status;
    g_hfp_relay_ctx.cind_status.callheld = call_status->callheld_status;
    g_hfp_relay_ctx.cind_status.service_status = call_status->service;
    g_hfp_relay_ctx.cind_status.signal = call_status->signal;
    g_hfp_relay_ctx.cind_status.batt_level = call_status->batt_level;
    g_hfp_relay_ctx.cind_status.roam_status = call_status->roam;
}

void bt_hfp_relay_update_cind_by_indicator(uint8_t type, uint8_t val)
{
    switch (type)
    {
    case HFP_AG_CIND_SERVICE_TYPE:
        g_hfp_relay_ctx.cind_status.service_status = val;
        break;
    case HFP_AG_CIND_CALL_TYPE:
        g_hfp_relay_ctx.cind_status.call = val;
        break;
    case HFP_AG_CIND_CALLSETUP_TYPE:
        g_hfp_relay_ctx.cind_status.callsetup = val;
        break;
    case HFP_AG_CIND_BATT_TYPE:
        g_hfp_relay_ctx.cind_status.batt_level = val;
        break;
    case HFP_AG_CIND_SIGNAL_TYPE:
        g_hfp_relay_ctx.cind_status.signal = val;
        break;
    case HFP_AG_CIND_ROAM_TYPE:
        g_hfp_relay_ctx.cind_status.roam_status = val;
        break;
    case HFP_AG_CIND_CALLHELD_TYPE:
        g_hfp_relay_ctx.cind_status.callheld = val;
        break;
    default:
        break;
    }
}

void bt_hfp_relay_notify_ag_call_state(void)
{
    uint8_t phone_len = 0;
    hfp_phone_call_info_t *call = &g_hfp_relay_ctx.remote_call;

    if (BT_HFP_RELAY_INVALID_CHANNEL == g_hfp_relay_ctx.ag_channel)
    {
        return;
    }

    if (g_hfp_relay_ctx.has_remote_call)
    {
        phone_len = strlen(call->phone_info.phone_number);
        if (phone_len >= PHONE_NUM_LEN)
        {
            phone_len = PHONE_NUM_LEN - 1;
        }
    }

    HFP_CALL_INFO_T *call_info = rt_malloc(sizeof(HFP_CALL_INFO_T) + phone_len);
    RT_ASSERT(call_info);
    memset(call_info, 0, sizeof(HFP_CALL_INFO_T) + phone_len);

    call_info->mux_id = (uint8_t)g_hfp_relay_ctx.ag_channel;
    call_info->num_active = g_hfp_relay_ctx.cind_status.call;
    call_info->num_held = g_hfp_relay_ctx.cind_status.callheld;
    call_info->callsetup_state = g_hfp_relay_ctx.cind_status.callsetup;
    call_info->phone_type = call->phone_info.type ? call->phone_info.type : PHONE_NUMBER_TYPE_UNKNOWN;
    call_info->phone_len = phone_len;
    if (phone_len)
    {
        memcpy(call_info->phone_number, call->phone_info.phone_number, phone_len);
    }

    bt_interface_phone_state_changed(call_info);
    rt_free(call_info);
}

void bt_hfp_relay_cache_remote_call(bt_notify_clcc_ind_t *clcc_info)
{
    hfp_phone_call_info_t *call = &g_hfp_relay_ctx.remote_call;

    memset(call, 0, sizeof(*call));
    call->call_idx = clcc_info->idx;
    call->call_dir = clcc_info->dir;
    call->call_status = clcc_info->st;
    call->call_mode = clcc_info->mode;
    call->call_mtpty = clcc_info->mpty;
    call->phone_info.type = clcc_info->phone_number_type ? clcc_info->phone_number_type : PHONE_NUMBER_TYPE_UNKNOWN;

    if (clcc_info->number_size)
    {
        uint8_t number_size = clcc_info->number_size;
        if (number_size >= PHONE_NUM_LEN)
        {
            number_size = PHONE_NUM_LEN - 1;
        }
        memcpy(call->phone_info.phone_number, clcc_info->number, number_size);
        call->phone_info.phone_number[number_size] = '\0';
    }

    g_hfp_relay_ctx.remote_calls.num_call = 1;
    g_hfp_relay_ctx.remote_calls.calls = call;
    g_hfp_relay_ctx.has_remote_call = 1;
}

static void bt_hfp_relay_clcc_timeout(void *parameter)
{
    if (BT_HFP_RELAY_CLCC_STATUS_START == g_hfp_relay_clcc_process_status)
    {
        bt_interface_get_ph_num_by_id(g_hfp_relay_ctx.hf_channel);
    }
    else
    {
        LOG_I("%s:start clcc get again!", __func__);
        bt_hfp_relay_start_get_clcc();
    }
}

void bt_hfp_relay_start_get_clcc(void)
{
    if (NULL == g_hfp_relay_clcc_timer)
    {
        g_hfp_relay_clcc_timer = rt_timer_create("bt_clcc", bt_hfp_relay_clcc_timeout, &g_hfp_relay_clcc_process_status,
                                 rt_tick_from_millisecond(100), RT_TIMER_FLAG_ONE_SHOT | RT_TIMER_FLAG_SOFT_TIMER);
    }

    RT_ASSERT(g_hfp_relay_clcc_timer);
    if (BT_HFP_RELAY_CLCC_STATUS_COMPLETE == g_hfp_relay_clcc_process_status)
    {
        g_hfp_relay_clcc_process_status = BT_HFP_RELAY_CLCC_STATUS_START;
    }
    rt_timer_stop(g_hfp_relay_clcc_timer);
    rt_timer_start(g_hfp_relay_clcc_timer);
}

void bt_hfp_relay_handle_at_cmd_cfm(uint8_t cmd_id, uint8_t res)
{
    switch (cmd_id)
    {
    case HFP_HF_AT_CIND_STATUS:
        LOG_I("get remote all call status complete %d", res);
        break;
    case HFP_HF_AT_CHLD_CMD:
        LOG_I("control remote three_waiting call complete %d", res);
        break;
    case HFP_HF_AT_CLIP:
        LOG_I("open remote incoming call notify complete %d", res);
        break;
    case HFP_HF_AT_CCWA:
        LOG_I("open remote the second incoming call notify complete %d", res);
        break;
    case HFP_HF_AT_BVRA:
        LOG_I("start / stop phone voice recognition complete %d", res);
        break;
    case HFP_HF_AT_CLCC:
        LOG_I("get remote all call information complete %d", res);
        g_hfp_relay_clcc_process_status = BT_HFP_RELAY_CLCC_STATUS_COMPLETE;
        if (BT_HFP_RELAY_INVALID_CHANNEL != g_hfp_relay_ctx.ag_channel)
        {
            bt_interface_remote_call_info_res(g_hfp_relay_ctx.ag_channel, &g_hfp_relay_ctx.remote_calls);
        }
        break;
    case HFP_HF_AT_ATA:
        LOG_I("answer a call complete %d", res);
        break;
    case HFP_HF_AT_CHUP:
        LOG_I("hangup a call complete %d", res);
        break;
    case HFP_HF_AT_ATD:
        LOG_I("make a call complete %d", res);
        if (g_hfp_relay_ctx.pending_ag_make_call && (BT_HFP_RELAY_INVALID_CHANNEL != g_hfp_relay_ctx.ag_channel))
        {
            g_hfp_relay_ctx.pending_ag_make_call = 0;
            bt_interface_make_call_res(g_hfp_relay_ctx.ag_channel, res);
        }
        break;
    case HFP_HF_AT_BLDN:
        LOG_I("make a callback complete %d", res);
        break;
    case HFP_HF_AT_VTS:
        LOG_I("send a DTMF key complete %d", res);
        break;
    case HFP_HF_AT_VGS:
        LOG_I("change volume value complete %d", res);
        break;
    case HFP_HF_AT_BCC:
        break;
    case HFP_HF_AT_CNUM:
        LOG_I("get remote local phone number complete %d", res);
        break;
    case HFP_HF_AT_BATT_UPDATE:
        LOG_I("update local battery level complete %d", res);
        break;
    default:
        break;
    }
}

int bt_hfp_relay_ag_event_handle(bt_hfp_relay_notify_data_t *msg)
{
    bt_hfp_relay_context_t *ctx = bt_hfp_relay_get_context();

    if ((NULL == msg) || (NULL == ctx))
    {
        return -RT_EINVAL;
    }

    switch (msg->event_id)
    {
    case BT_NOTIFY_AG_PROFILE_CONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)msg->data;
        bt_hfp_relay_set_ag_channel(profile_info->profile_channel, &profile_info->mac);

        bt_interface_conn_to_source_ext((unsigned char *)&profile_info->mac, BT_PROFILE_AVRCP);

        LOG_I("HFP AG connected");
        break;
    }
    case BT_NOTIFY_AG_PROFILE_DISCONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)msg->data;
        bt_hfp_relay_clear_ag_channel(profile_info->profile_channel);
        LOG_I("HFP AG disconnected");
        break;
    }
    case BT_NOTIFY_AG_MAKE_CALL_REQ:
    {
        bt_notify_ag_at_arg_t *at_arg = (bt_notify_ag_at_arg_t *)msg->data;
        if ((BT_HFP_RELAY_INVALID_CHANNEL != ctx->hf_channel) && at_arg->payload_len)
        {
            ctx->pending_ag_make_call = 1;
            bt_interface_hf_out_going_call_by_id(g_hfp_relay_ctx.hf_channel, at_arg->payload_len, at_arg->payload);
        }
        else
        {
            bt_interface_make_call_res(ctx->ag_channel, BTS2_FAILED);
        }
        break;
    }
    case BT_NOTIFY_AG_ANSWER_CALL_REQ:
    {
        if (BT_HFP_RELAY_INVALID_CHANNEL != ctx->hf_channel)
        {
            bt_interface_start_hf_answer_req_send_by_id(ctx->hf_channel);
        }
        break;
    }
    case BT_NOTIFY_AG_HANGUP_CALL_REQ:
    {
        if (BT_HFP_RELAY_INVALID_CHANNEL != ctx->hf_channel)
        {
            bt_interface_handup_call_by_id(ctx->hf_channel);
        }
        break;
    }
    case BT_NOTIFY_AG_RECV_DTMF_KEY:
    {
        bt_notify_ag_at_arg_t *at_arg = (bt_notify_ag_at_arg_t *)msg->data;
        char key = at_arg->payload_len ? at_arg->payload[0] : 0;
        if ((BT_HFP_RELAY_INVALID_CHANNEL != ctx->hf_channel) && key)
        {
            bt_interface_start_dtmf_req_send_by_id(ctx->hf_channel, key);
        }
        break;
    }
    case BT_NOTIFY_AG_VOLUME_CHANGE:
    {
        bt_notify_ag_at_arg_t *at_arg = (bt_notify_ag_at_arg_t *)msg->data;
        if ((BT_HFP_RELAY_INVALID_CHANNEL != ctx->hf_channel) && at_arg->payload_len)
        {
            bt_interface_set_speaker_volume_by_id(ctx->hf_channel, at_arg->payload[0]);
#if defined(AUDIO_USING_MANAGER)
            audio_server_set_private_volume(AUDIO_TYPE_BT_VOICE, at_arg->payload[0]);
#endif
        }
        break;
    }
    case BT_NOTIFY_AG_GET_INDICATOR_STATUS_REQ:
    {
        bt_notify_ag_at_arg_t *at_arg = (bt_notify_ag_at_arg_t *)msg->data;
        if ((BT_HFP_RELAY_INVALID_CHANNEL != ctx->hf_channel) &&
                (BT_HFP_RELAY_INVALID_CHANNEL != ctx->ag_channel))
        {
            bt_interface_get_remote_call_status_by_id(ctx->hf_channel);
        }
        else
        {
            bt_interface_get_all_indicator_info_res(at_arg->profile_channel, &ctx->cind_status);
        }
        break;
    }
    case BT_NOTIFY_AG_GET_ALL_REMT_CALLS_INFO_REQ:
    {
        bt_notify_ag_at_arg_t *at_arg = (bt_notify_ag_at_arg_t *)msg->data;
        ctx->remote_calls.calls = &ctx->remote_call;
        ctx->remote_calls.num_call = ctx->has_remote_call ? 1 : 0;
        if (BT_HFP_RELAY_INVALID_CHANNEL != ctx->hf_channel)
        {
            bt_hfp_relay_start_get_clcc();
        }
        else
        {
            bt_interface_remote_call_info_res(at_arg->profile_channel, &ctx->remote_calls);
        }
        break;
    }
    case BT_NOTIFY_AG_GET_LOCAL_PHONE_INFO_REQ:
    {
        bt_notify_ag_at_arg_t *at_arg = (bt_notify_ag_at_arg_t *)msg->data;
        if (BT_HFP_RELAY_INVALID_CHANNEL != ctx->hf_channel)
        {
            bt_interface_get_ph_num_by_id(ctx->hf_channel);
        }
        else
        {
            bt_interface_local_phone_info_res(at_arg->profile_channel, &ctx->local_phone_num);
        }
        break;
    }
    case BT_NOTIFY_AG_EXTERN_AT_CMD_KEY_REQ:
    {
        bt_notify_ag_at_arg_t *at_arg = (bt_notify_ag_at_arg_t *)msg->data;
        if ((BT_HFP_RELAY_INVALID_CHANNEL != ctx->hf_channel) && at_arg->payload_len)
        {
            bt_interface_hfp_set_extern_cmd_by_id(ctx->hf_channel, at_arg->payload, at_arg->payload_len);
            bt_interface_make_call_res(ctx->ag_channel, BTS2_SUCC);
        }
        else
        {
            bt_interface_make_call_res(ctx->ag_channel, BTS2_FAILED);
        }
        break;
    }
    case BT_NOTIFY_AG_BATTERY_UPDATE:
    {
        bt_notify_ag_at_arg_t *at_arg = (bt_notify_ag_at_arg_t *)msg->data;
        if ((BT_HFP_RELAY_INVALID_CHANNEL != ctx->hf_channel) && at_arg->payload_len)
        {
            bt_interface_hf_update_battery_by_id(ctx->hf_channel, at_arg->payload[0]);
        }
        break;
    }
    default:
        break;
    }

    return 0;
}

int bt_hfp_relay_hf_event_handle(bt_hfp_relay_notify_data_t *msg)
{
    switch (msg->event_id)
    {
    case BT_NOTIFY_HF_PROFILE_CONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)msg->data;
        bt_hfp_relay_set_hf_channel(profile_info->profile_channel, &profile_info->mac);
        LOG_I("HFP HF connected");
        break;
    }
    case BT_NOTIFY_HF_PROFILE_DISCONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)msg->data;
        bt_hfp_relay_clear_hf_channel(profile_info->profile_channel);
        LOG_I("HFP HF disconnected");
        break;
    }
    case BT_NOTIFY_HF_VOICE_RECOG_CAP_UPDATE:
    {
        bt_notify_ag_at_arg_t *data = (bt_notify_ag_at_arg_t *)msg->data;
        g_hfp_relay_ctx.phone_bvra = data->payload[0];
        LOG_I("remote device %s voice recognition", data->payload[0] ? "support" : "dont support");
        break;
    }
    case BT_NOTIFY_HF_VOICE_RECOG_STATUS_CHANGE:
    {
        bt_notify_ag_at_arg_t *data = (bt_notify_ag_at_arg_t *)msg->data;
        LOG_I("remote device voice recognition status is %s", data->payload[0] ? "on" : "off");
        break;
    }
    case BT_NOTIFY_HF_LOCAL_PHONE_NUMBER:
    {
        bt_notify_ag_at_arg_t *data = (bt_notify_ag_at_arg_t *)msg->data;
        if (data->payload_len)
        {
            bt_hfp_relay_cache_local_phone_num(data->payload, data->payload_len);
            LOG_I("the remote phone local number:%s", g_hfp_relay_ctx.local_phone_num.phone_number);
        }
        if (BT_HFP_RELAY_INVALID_CHANNEL != g_hfp_relay_ctx.ag_channel)
        {
            bt_interface_local_phone_info_res(g_hfp_relay_ctx.ag_channel, &g_hfp_relay_ctx.local_phone_num);
        }
        break;
    }
    case BT_NOTIFY_HF_REMOTE_CALL_INFO_IND:
    {
        bt_notify_clcc_ind_t *clcc_info = (bt_notify_clcc_ind_t *)msg->data;
        if (msg->data_len)
        {
            g_hfp_relay_clcc_process_status = BT_HFP_RELAY_CLCC_STATUS_IN_PROGRESS;
            LOG_I("the remote phone call info phone_number_type:%d", clcc_info->phone_number_type);
            LOG_I("the remote phone call info call_idx:%d", clcc_info->idx);
            LOG_I("the remote phone call info call_direction:%d", clcc_info->dir);
            LOG_I("the remote phone call info call_status:%d", clcc_info->st);
            LOG_I("the remote phone call info call_mode:%d", clcc_info->mode);
            LOG_I("the remote phone call info call_mpty:%d", clcc_info->mpty);
            LOG_I("the remote phone call info call_number_size:%d", clcc_info->number_size);
            LOG_I("the remote phone call info call_number:%s", clcc_info->number);
            bt_hfp_relay_cache_remote_call(clcc_info);
        }
        break;
    }
    case BT_NOTIFY_HF_VOLUME_CHANGE:
    {
        bt_notify_ag_at_arg_t *data = (bt_notify_ag_at_arg_t *)msg->data;
        g_hfp_relay_ctx.phone_vgs = data->payload[0];
        LOG_I("the remote phone want to change volume be: %d", data->payload[0]);
        if (BT_HFP_RELAY_INVALID_CHANNEL != g_hfp_relay_ctx.ag_channel)
        {
            bt_interface_spk_vol_change_req(g_hfp_relay_ctx.ag_channel, g_hfp_relay_ctx.phone_vgs);
        }
        break;
    }
    case BT_NOTIFY_HF_CALL_STATUS_UPDATE:
    {
        bt_notify_all_call_status *call_status = (bt_notify_all_call_status *)msg->data;
        bt_hfp_relay_update_cind_from_all_status(call_status);
        if ((0 == call_status->call_status) && (0 == call_status->callsetup_status) && (0 == call_status->callheld_status))
        {
            bt_hfp_relay_reset_cached_call();
        }
        bt_hfp_relay_notify_ag_call_state();
        LOG_I("the remote phone call_status: %d", call_status->call_status);
        LOG_I("the remote phone callsetup_status: %d", call_status->callsetup_status);
        LOG_I("the remote phone callheld_status: %d", call_status->callheld_status);
        break;
    }
    case BT_NOTIFY_HF_INDICATOR_UPDATE:
    {
        bt_notify_cind_ind_t *cind_status = (bt_notify_cind_ind_t *)msg->data;
        HFP_IND_INFO_T ind_info;

        bt_hfp_relay_update_cind_by_indicator(cind_status->type, cind_status->val);
        ind_info.ind_type = cind_status->type;
        ind_info.ind_val = cind_status->val;
        if (BT_HFP_RELAY_INVALID_CHANNEL != g_hfp_relay_ctx.ag_channel)
        {
            bt_interface_indicator_status_changed(g_hfp_relay_ctx.ag_channel, &ind_info);
        }
        LOG_I("the remote phone call status type:%d, status: %d", cind_status->type, cind_status->val);
        break;
    }
    case BT_NOTIFY_HF_INBAND_RING_STATUS_CHANGE:
    {
        bt_notify_ag_at_arg_t *data = (bt_notify_ag_at_arg_t *)msg->data;
        g_hfp_relay_ctx.phone_bsir = data->payload[0];
        LOG_I("remote device in-band ring status is %s", data->payload[0] ? "on" : "off");
        break;
    }
    case BT_NOTIFY_HF_AT_CMD_CFM:
    {
        bt_notify_at_cmd_cfm_t *at_cmd_cfm = (bt_notify_at_cmd_cfm_t *)msg->data;
        bt_hfp_relay_handle_at_cmd_cfm(at_cmd_cfm->at_cmd_id, at_cmd_cfm->res);
        break;
    }
    default:
        break;
    }

    return 0;
}

static void help(void)
{
}

// __ROM_USED void hfp_cmd(int argc, char **argv)
// {
//     if (argc < 2)
//         help();
//     else
//     {
//         const char *cmd = argv[1];
//         if (strcmp(cmd, "c") == 0)
//         {
//             bt_cm_delete_bonded_devs();
//         }
//         else if (strcmp(cmd, "start_inquiry") == 0)
//         {
//             bt_start_inquiry_ex_t para;
//             para.max_rsp = MAX_DISCOV_RESS;
//             para.max_timeout = 60;
//             para.dev_cls_mask = BT_DEVCLS_PHONE;
//             bt_interface_start_inquiry_ex(&para);
//         }
//         else if (strcmp(cmd, "stop_inquiry") == 0)
//         {
//             bt_interface_stop_inquiry();
//         }
//         else if (strcmp(cmd, "hfp_connect") == 0)
//         {
//             bd_addr_t mac;
//             bt_addr_convert_from_string_to_general(argv[2], &mac);
//             bt_interface_conn_ext((unsigned char *)&mac, BT_PROFILE_HFP);
//         }
//         else if (strcmp(cmd, "hfp_disconnect") == 0)
//         {
//             bd_addr_t mac;
//             bt_addr_convert_from_string_to_general(argv[2], &mac);
//             bt_interface_disc_ext((unsigned char *)&mac, BT_PROFILE_HFP);
//         }
//         else if (strcmp(cmd, "local_phone_number") == 0)
//         {
//             bt_interface_get_ph_num();
//         }
//         else if (strcmp(cmd, "remote_calls_info") == 0)
//         {
//             bt_interface_get_remote_ph_num();
//         }
//         else if (strcmp(cmd, "remote_calls_status") == 0)
//         {
//             bt_interface_get_remote_call_status();
//         }
//         else if (strcmp(cmd, "make_call") == 0)
//         {
//             bt_interface_hf_out_going_call(rt_strlen(argv[2]), argv[2]);
//         }
//         else if (strcmp(cmd, "call_back") == 0)
//         {
//             bt_interface_start_last_num_dial_req_send();
//         }
//         else if (strcmp(cmd, "answer_call") == 0)
//         {
//             bt_interface_start_hf_answer_req_send();
//         }
//         else if (strcmp(cmd, "handup_call") == 0)
//         {
//             bt_interface_handup_call();
//         }
//         else if (strcmp(cmd, "dtmf_key") == 0)
//         {
//             char key = argv[2][0];
//             bt_interface_start_dtmf_req_send(key);
//         }
//         else if (strcmp(cmd, "volume_control") == 0)
//         {
//             bt_interface_set_speaker_volume(atoi(argv[2]));
// #if defined(AUDIO_USING_MANAGER)
//             audio_server_set_private_volume(AUDIO_TYPE_BT_VOICE, atoi(argv[2]));
// #endif
//         }
//         else if (strcmp(cmd, "voice_recognition") == 0)
//         {
//             bt_interface_voice_recog(atoi(argv[2]));
//         }
//         else if (strcmp(cmd, "audio_connect") == 0)
//         {
//             bt_interface_audio_switch(0);
//         }
//         else if (strcmp(cmd, "audio_disconnect") == 0)
//         {
//             bt_interface_audio_switch(1);
//         }
//         else if (strcmp(cmd, "battery_update") == 0)
//         {
//             bt_interface_hf_update_battery(atoi(argv[2]));
//         }
//     }
// }
// MSH_CMD_EXPORT(hfp_cmd, hfp_cmd command)