/*
 * SPDX-FileCopyrightText: 2022-2025 SiFli Technologies(Nanjing) Co., Ltd
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

#define DBG_TAG               "bt_rt_device.urc_hf"
//#define DBG_LVL               DBG_INFO
#include <log.h>

//#define              BT_HFP_CLCC_TIMEOUT        (3000)

#ifdef BT_USING_SIRI
extern rt_timer_t    siri_timer_hdl;

static void urc_func_bt_voice_recog_sifli(bt_notify_ag_at_arg_t *data)
{
    bt_notify_t args;

    args.event = BT_EVENT_SIRI_STATE_NOTIFY;

    args.args = data->payload; // 1:AG active complete;0:AG deactive complete;

    rt_bt_event_notify(&args);
    LOG_I("URC BT voice recog ind %d", data->payload[0]);
    return;
}

void urc_func_bt_hf_voice_recog_sifli(uint8_t res)
{
    bt_notify_t args;
    LOG_I("bt_set_event %x", bt_sifli_get_bt_event());
    args.args = &res;//res 0:BTS2_SUCC; other value:error code
    if (bt_sifli_check_bt_event(BT_SET_SIRI_ON_EVENT))
    {
        bt_sifli_reset_bt_event(BT_SET_SIRI_ON_EVENT);
        args.event = BT_EVENT_SIRI_ON_COMPLETE;
        rt_bt_event_notify(&args);
    }
    else if (bt_sifli_check_bt_event(BT_SET_SIRI_OFF_EVENT))
    {
        bt_sifli_reset_bt_event(BT_SET_SIRI_OFF_EVENT);
        args.event = BT_EVENT_SIRI_OFF_COMPLETE;
        rt_bt_event_notify(&args);
    }

    if (NULL != siri_timer_hdl)
    {
        rt_timer_stop(siri_timer_hdl);
    }

    LOG_I("URC BT hf voice recog cfm res %d set_event %d hfp timer stop", res, bt_sifli_get_bt_event());
    return;
}

static void urc_func_bt_voice_recog_cap_sifli(bt_notify_ag_at_arg_t *data)
{
    bt_notify_t args;
    args.event = BT_EVENT_SIRI_CAPABILITY_NOTIFY;
    args.args = data->payload;//1:AG active complete;0:AG deactive complete;

    rt_bt_event_notify(&args);
    LOG_I("URC BT voice recog cap ind %d", data->payload[0]);
    return;
}
#endif

static void urc_func_local_phone_number_sifli(bt_notify_ag_at_arg_t *data)
{
    bt_notify_t args;
    phone_number_t number = {0};
    int  numSize;
    int  i;

    args.event = BT_EVENT_LOCAL_CALL_NUMBER;
    LOG_I("input data len:%d", data->payload_len);
    if ((data->payload_len < 3) || (data->payload_len > 20))
    {
        return;
    }

    numSize = 0;
    for (i = 0; i < data->payload_len; i++)
    {
        if (data->payload[i] != '"')
        {
            number.number[numSize] = data->payload[i];
            numSize++;
        }
    }
    number.size = numSize;
    args.args = &number;
    rt_bt_event_notify(&args);

    LOG_I("URC local phone len:%d buf:%s", number.size, number.number);
    return;
}

static void urc_func_bt_voice_volume_sifli(bt_notify_ag_at_arg_t *data)
{
    bt_notify_t args;
    bt_volume_set_t vol = {0};
    vol.conn_idx = urc_func_bt_profile_channel_by_conn_idx(BT_PROFILE_HFP, data->profile_channel);
    vol.mac = rt_bt_get_connect_dev_by_idx(urc_func_bt_get_device(), vol.conn_idx)->mac;
    vol.mode = BT_VOLUME_CALL;
    vol.volume.call_volume = data->payload[0];
    args.event = BT_EVENT_VOL_CHANGED;
    args.args = &vol;
    rt_bt_event_notify(&args);
    LOG_I("URC BT hfp-volume ind:%d vol:%d", vol.conn_idx, vol.volume.call_volume);
}

static void urc_func_bt_dial_complete_sifli(uint8_t res)
{
    bt_notify_t args;
    args.event = BT_EVENT_DIAL_COMPLETE;
    args.args = &res;
    if (bt_sifli_check_bt_event(BT_SET_DIAL_EVENT))
    {
        LOG_I("URC BT dial complete ind %x, res:%d", bt_sifli_get_bt_event(), res);
        bt_sifli_reset_bt_event(BT_SET_DIAL_EVENT);
        rt_bt_event_notify(&args);
        LOG_I("hfp timer stop");
    }
    return;
}

static void urc_func_bt_cind_sifli(bt_cind_ind_t *ind)
{
    bt_notify_t args;
    args.event = BT_EVENT_CIND_IND;
    args.args = ind;
    rt_bt_event_notify(&args);
    return;
}

static void urc_func_profile_cind_sifli(bts2_hfp_hf_cind *cind)
{
    //callStatus is not 0,means exista active call at least ,when hfp profile is connected
    //callSetupStatus is not 0,means exist a setup call,when hfp profile is connected
    //callHeldStatus is not 0,means exist a 3way call,when hfp pforile is connected
    bt_notify_t args;
    bt_cind_data_t data;
    data.service = cind->service;
    data.signal = cind->signal;
    data.batt_level = cind->batt_level;
    data.roam = cind->roam;
    args.event = BT_EVENT_CINDS_IND;
    args.args = &data;
    rt_bt_event_notify(&args);
    LOG_I("URC cind signal:%d batt_level:%d service:%d roam:%d", cind->signal, cind->batt_level, cind->service, cind->roam);
}

static void urc_func_clcc_sifli(bt_clcc_ind_t *ind)
{
    bt_notify_t args;
    args.event = BT_EVENT_CLCC_IND;
    args.args = ind;

    if (bt_sifli_check_bt_event(BT_SET_CLCC_EVENT))
    {
        rt_bt_event_notify(&args);
    }
    else
    {
        LOG_E("URC clcc ind error");
    }
    return;
}

static void urc_func_bt_clcc_complete_sifli(uint8_t res)
{
    bt_notify_t args;
    args.event = BT_EVENT_CLCC_COMPLETE;
    args.args = &res;
    rt_bt_event_notify(&args);
    bt_sifli_reset_bt_event(BT_SET_CLCC_EVENT);
    LOG_I("URC clcc res:%d", res);
    return;
}


static void urc_func_bt_call_vol_ind_sifli(uint8_t res)
{
    bt_notify_t args;
    args.event = BT_EVENT_VGS_IND;
    args.args = &res;
    if (bt_sifli_check_bt_event(BT_SET_VGS_EVENT))
    {
        LOG_I("URC BT call vol res:%d", res);
        bt_sifli_reset_bt_event(BT_SET_VGS_EVENT);
        rt_bt_event_notify(&args);
    }
    return;
}

static void urc_func_bt_call_dtmf_ind_sifli(uint8_t res)
{
    bt_notify_t args;
    args.event = BT_EVENT_DTMF_IND;
    args.args = &res;
    if (bt_sifli_check_bt_event(BT_SET_DTMF_EVENT))
    {
        LOG_I("URC BT dtmf_ind res:%d", res);
        bt_sifli_reset_bt_event(BT_SET_DTMF_EVENT);
        rt_bt_event_notify(&args);
    }
    return;
}

static void urc_func_hfp_at_cfm_status_notify(uint8_t cmd_id, uint8_t res)
{
    bt_notify_t args;
    bt_at_cmd_cfm_t cmd;
    cmd.cmd_id = cmd_id;
    cmd.res = res;
    args.event = BT_EVENT_AT_CMD_CFM_STATUS;
    args.args = &cmd;
    rt_bt_event_notify(&args);
}

static void bt_sifli_notify_hdl_at_cmd(uint8_t cmd_id, uint8_t res)
{
    switch (cmd_id)
    {
#ifdef BT_USING_SIRI
    case HFP_HF_AT_BVRA:
    {
        urc_func_bt_hf_voice_recog_sifli(res);
        break;
    }
#endif
    case HFP_HF_AT_CLCC:
    {
        urc_func_bt_clcc_complete_sifli(res);
        break;
    }
    case HFP_HF_AT_ATD:
    case HFP_HF_AT_BLDN:
    {
        urc_func_bt_dial_complete_sifli(res);
        break;
    }
    case HFP_HF_AT_VTS:
    {
        urc_func_bt_call_dtmf_ind_sifli(res);
        break;
    }
    case HFP_HF_AT_VGS:
    {
        urc_func_bt_call_vol_ind_sifli(res);
        break;
    }
    case HFP_HF_AT_BCC:
    {
        if (res)
        {
            bt_notify_device_sco_info_t sco_info = {0};
            rt_bt_device_t *dev = urc_func_bt_get_device();
            sco_info.sco_res = res;
            sco_info.sco_type = BT_NOTIFY_HFP_HF;
            sco_info.conn_idx = dev->active_idx;
            urc_func_call_link_ested_sifli(&sco_info);
        }
        break;
    }
    default:
        break;
    }
    urc_func_hfp_at_cfm_status_notify(cmd_id, res);
}

int bt_sifli_notify_hfp_hf_event_hdl(uint16_t event_id, uint8_t *data, uint16_t data_len)
{
    switch (event_id)
    {
    case BT_NOTIFY_HF_PROFILE_CONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        urc_func_profile_conn_sifli(profile_info, BT_PROFILE_HFP);
        break;
    }
    case BT_NOTIFY_HF_PROFILE_DISCONNECTED:
    {
        bt_notify_profile_state_info_t *profile_info = (bt_notify_profile_state_info_t *)data;
        urc_func_profile_disc_sifli(profile_info->mac.addr, BT_PROFILE_HFP, profile_info->res);
        urc_func_bt_call_vol_ind_sifli(BTS2_FAILED);
        urc_func_bt_call_dtmf_ind_sifli(BTS2_FAILED);
        break;
    }
#ifdef BT_USING_SIRI
    case BT_NOTIFY_HF_VOICE_RECOG_CAP_UPDATE:
    {
        urc_func_bt_voice_recog_cap_sifli((bt_notify_ag_at_arg_t *)data);
        break;
    }
    case BT_NOTIFY_HF_VOICE_RECOG_STATUS_CHANGE:
    {
        urc_func_bt_voice_recog_sifli((bt_notify_ag_at_arg_t *)data);
        break;
    }
#endif
    case BT_NOTIFY_HF_LOCAL_PHONE_NUMBER:
    {
        urc_func_local_phone_number_sifli((bt_notify_ag_at_arg_t *)data);
        break;
    }
    case BT_NOTIFY_HF_REMOTE_CALL_INFO_IND:
    {
        bt_notify_clcc_ind_t *clcc_info = (bt_notify_clcc_ind_t *)data;
        bt_clcc_ind_t ind;
        ind.dir = clcc_info->dir;
        ind.mode = clcc_info->mode;
        ind.mpty = clcc_info->mpty;
        ind.st = clcc_info->st;
        ind.phone_number_type = clcc_info->phone_number_type;
        ind.number_size = clcc_info->number_size;
        ind.idx = clcc_info->idx;
        ind.number = clcc_info->number;
        urc_func_clcc_sifli(&ind);
        break;
    }
    case BT_NOTIFY_HF_VOLUME_CHANGE:
    {
        urc_func_bt_voice_volume_sifli((bt_notify_ag_at_arg_t *)data);
        break;
    }
    case BT_NOTIFY_HF_CALL_STATUS_UPDATE:
    {
        bt_notify_all_call_status *call_status = (bt_notify_all_call_status *)data;
        bts2_hfp_hf_cind cind_status;
        cind_status.callHeldStatus = call_status->callheld_status;
        cind_status.callStatus = call_status->call_status;
        cind_status.callSetupStatus = call_status->callsetup_status;
        cind_status.roam = call_status->roam;
        cind_status.service = call_status->service;
        cind_status.signal = call_status->signal;
        cind_status.batt_level = call_status->batt_level;
        urc_func_profile_cind_sifli(&cind_status);
        break;
    }
    case BT_NOTIFY_HF_INDICATOR_UPDATE:
    {
        bt_notify_cind_ind_t   *cind_status = (bt_notify_cind_ind_t *)data;
        bt_cind_ind_t cind_info;
        cind_info.type = cind_status->type;
        cind_info.val = cind_status->val;
        urc_func_bt_cind_sifli(&cind_info);
        break;
    }
    case BT_NOTIFY_HF_AT_CMD_CFM:
    {
        bt_notify_at_cmd_cfm_t *at_cmd_cfm = (bt_notify_at_cmd_cfm_t *) data;
        bt_sifli_notify_hdl_at_cmd(at_cmd_cfm->at_cmd_id, at_cmd_cfm->res);
        break;
    }
    default:
        return -1;
    }
    return 0;
}


