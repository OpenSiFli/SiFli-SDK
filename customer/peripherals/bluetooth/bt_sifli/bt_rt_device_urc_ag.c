/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <stdio.h>
#include <string.h>
#include "drv_bt.h"
//#include "utf8_unicode.h"
#include "bt_rt_device.h"
#include "bts2_global.h"
#include "bts2_app_inc.h"

#define DBG_TAG               "bt_rt_device.urc_ag"
//#define DBG_LVL               DBG_INFO
#include <log.h>


static void urc_func_ag_answer_call_req(uint8_t *mux_id)
{
    bt_notify_t args;
    args.event = BT_EVENT_AG_ANSWER_CALL_REQ;
    args.args = mux_id;
    rt_bt_event_notify(&args);
    LOG_I("URC AG BT_EVENT_AG_ANSWER_CALL_REQ %d", *mux_id);
}

static void urc_func_ag_hangup_call_req(uint8_t *mux_id)
{
    bt_notify_t args;
    args.event = BT_EVENT_AG_HUNGUP_CALL_REQ;
    args.args = mux_id;
    rt_bt_event_notify(&args);
    LOG_I("URC AG BT_EVENT_AG_HUNGUP_CALL_REQ %d", *mux_id);
}

static void urc_func_ag_make_call_req(bt_notify_ag_at_arg_t *data)
{
    bt_notify_t args;
    args.event = BT_EVENT_MAKE_CALL_REQ;
    args.args = data->payload;
    rt_bt_event_notify(&args);
    LOG_I("URC AG phone num:%s", data->payload);
}

static void urc_func_ag_dmtf_key_req(bt_notify_ag_at_arg_t *data)
{
    bt_notify_t args;
    args.event = BT_EVENT_DTMF_KEY_REQ;
    args.args = data->payload;
    rt_bt_event_notify(&args);
    LOG_I("URC AG dmtf key:%c", data->payload[0]);
}

static void urc_func_ag_get_local_phone_info_req(uint8_t *mux_id)
{
    bt_notify_t args;
    args.event = BT_EVENT_GET_LOCAL_PHONE_INFO_REQ;
    args.args = mux_id;
    rt_bt_event_notify(&args);
    LOG_I("URC AG BT_EVENT_GET_LOCAL_PHONE_INFO_REQ %d", *mux_id);
}

static void urc_func_ag_get_indicator_status_req(uint8_t *mux_id)
{
    bt_notify_t args;
    args.event = BT_EVENT_GET_INDICATOR_STATUS_REQ;
    args.args = mux_id;
    rt_bt_event_notify(&args);
    LOG_I("URC AG BT_EVENT_GET_INDICATOR_STATUS_REQ %d", *mux_id);
}

static void urc_func_ag_get_all_call_status_req(uint8_t *mux_id)
{
    bt_notify_t args;
    args.event = BT_EVENT_GET_ALL_REMOTE_CALL_INFO_REQ;
    args.args = mux_id;
    rt_bt_event_notify(&args);
    LOG_I("URC AG BT_EVENT_GET_ALL_REMOTE_CALL_INFO_REQ %d", *mux_id);
}

static void urc_func_bt_voice_volume_sifli(bt_notify_ag_at_arg_t *volume)
{
    bt_notify_t args;
    bt_volume_set_t vol = {0};
    vol.conn_idx = urc_func_bt_profile_channel_by_conn_idx(BT_PROFILE_HFP, volume->profile_channel);
    vol.mac = rt_bt_get_connect_dev_by_idx(urc_func_bt_get_device(), vol.conn_idx)->mac;
    vol.mode = BT_VOLUME_CALL;
    vol.volume.call_volume = volume->payload[0];
    args.event = BT_EVENT_VOL_CHANGED;
    args.args = &vol;
    rt_bt_event_notify(&args);
    LOG_I("URC BT ag-volume ind:%d vol:%d", vol.conn_idx, vol.volume.call_volume);
}

static void urc_func_bt_buttery_update_sifli(uint8_t *data)
{
    bt_notify_t args;
    args.event = BT_EVENT_AG_BATTERY_UPDATE;
    args.args = data;
    rt_bt_event_notify(&args);
    LOG_I("URC AG battery status:%d val:%d", ((hfp_battery_vaule_t *)data)->batt_status, ((hfp_battery_vaule_t *)data)->batt_val);
}

int bt_sifli_notify_hfp_ag_event_hdl(uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    switch (event_id)
    {
    case BT_NOTIFY_AG_PROFILE_CONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        urc_func_profile_conn_sifli(profile_info, BT_PROFILE_HFP);
        break;
    }
    case BT_NOTIFY_AG_PROFILE_DISCONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        urc_func_profile_disc_sifli(profile_info->mac.addr, BT_PROFILE_HFP, profile_info->res);
        break;
    }
    case BT_NOTIFY_AG_MAKE_CALL_REQ:
    {
        urc_func_ag_make_call_req((bt_notify_ag_at_arg_t *)data);
        break;
    }
    case BT_NOTIFY_AG_ANSWER_CALL_REQ:
    {
        urc_func_ag_answer_call_req(data);
        break;
    }
    case BT_NOTIFY_AG_HANGUP_CALL_REQ:
    {
        urc_func_ag_hangup_call_req(data);
        break;
    }
    case BT_NOTIFY_AG_RECV_DTMF_KEY:
    {
        urc_func_ag_dmtf_key_req((bt_notify_ag_at_arg_t *)data);
        break;
    }
    case BT_NOTIFY_AG_VOLUME_CHANGE:
    {
        urc_func_bt_voice_volume_sifli((bt_notify_ag_at_arg_t *)data);
        break;
    }
    case BT_NOTIFY_AG_GET_INDICATOR_STATUS_REQ:
    {
        urc_func_ag_get_indicator_status_req(data);
        break;
    }
    case BT_NOTIFY_AG_GET_ALL_REMT_CALLS_INFO_REQ:
    {
        urc_func_ag_get_all_call_status_req(data);
        break;
    }
    case BT_NOTIFY_AG_GET_LOCAL_PHONE_INFO_REQ:
    {
        urc_func_ag_get_local_phone_info_req(data);
        break;
    }
    case BT_NOTIFY_AG_EXTERN_AT_CMD_KEY_REQ:
    {
        break;
    }
    case BT_NOTIFY_AG_BATTERY_UPDATE:
    {
        urc_func_bt_buttery_update_sifli(data);
        break;
    }
    default:
        return -1;
    }
    return 0;
}

