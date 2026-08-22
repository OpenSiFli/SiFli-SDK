/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <string.h>

#include "rt_bt_app.h"
#include "bf0_ble_common.h"          /* bd_addr_t, bt_addr_convert_from_string_to_general() */
#include "bt_connection_manager.h"   /* bt_cm_delete_bonded_devs() */

#define DBG_TAG "bt.srv.hfp"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

/* CLCC polling period, matching the reference example. */
#define HFP_CLCC_INTERVAL_MS 100

/* ---------------------------------------------------------------------------
 * CLCC one-shot polling timer
 *
 * On each call-indicator change the timer is (re)started. When it fires it
 * either issues a remote-call-info query (CLCC) or reschedules itself, matching
 * the polling logic of the reference HFP example.
 * ------------------------------------------------------------------------- */

static rt_uint8_t s_clcc_status = CALL_CLCC_COMPLETE;
static rt_timer_t s_clcc_timer  = RT_NULL;

static bt_err_t hfp_query_remote_calls(void)
{
    return rt_bt_app_control(BT_CONTROL_GET_REMOTE_PHONE_NUMER, RT_NULL);
}

static void hfp_clcc_start(void);

static void hfp_clcc_timeout(void *parameter)
{
    (void)parameter;

    if (s_clcc_status == CALL_CLCC_START)
    {
        hfp_query_remote_calls();
    }
    else
    {
        hfp_clcc_start();
    }
}

static void hfp_clcc_start(void)
{
    if (s_clcc_timer == RT_NULL)
    {
        s_clcc_timer = rt_timer_create("hfp_clcc", hfp_clcc_timeout, RT_NULL,
                                       rt_tick_from_millisecond(HFP_CLCC_INTERVAL_MS),
                                       RT_TIMER_FLAG_ONE_SHOT | RT_TIMER_FLAG_SOFT_TIMER);
        if (s_clcc_timer == RT_NULL)
        {
            LOG_E("create CLCC timer failed");
            return;
        }
    }

    if (s_clcc_status == CALL_CLCC_COMPLETE)
    {
        s_clcc_status = CALL_CLCC_START;
    }
    rt_timer_stop(s_clcc_timer);
    rt_timer_start(s_clcc_timer);
}

/* ---------------------------------------------------------------------------
 * Device-command wrappers (each maps a readable action to a BT_CONTROL_* code)
 * ------------------------------------------------------------------------- */

static bt_err_t hfp_start_inquiry(int argc, char **argv)
{
    bt_start_inquiry_ex_t para = {0};

    (void)argc;
    (void)argv;
    para.dev_cls_mask = 0;   /* 0 = no class filter, discover all devices */
    para.max_timeout  = 30;  /* seconds; 0 = unlimited */
    para.max_rsp      = 0;   /* 0 = unlimited number of responses */
    return rt_bt_app_control(BT_CONTROL_SEARCH_EQUIPMENT_EX, &para);
}

static bt_err_t hfp_stop_inquiry(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    return rt_bt_app_control(BT_CONTROL_CANCEL_SEARCH, RT_NULL);
}

static bt_err_t hfp_connect(int argc, char **argv)
{
    bt_connect_info_t info = {0};
    bd_addr_t         addr = {0};

    if (argc < 2)
    {
        LOG_E("connect requires an address xx:xx:xx:xx:xx:xx");
        return BT_ERROR_INPARAM;
    }
    if (bt_addr_convert_from_string_to_general(argv[1], &addr) != BT_MAX_MAC_LEN)
    {
        LOG_E("invalid BT address: %s", argv[1]);
        return BT_ERROR_INPARAM;
    }
    rt_memcpy(info.mac.addr, addr.addr, BT_MAX_MAC_LEN);
    info.profile  = BT_PROFILE_HFP;
    info.conn_idx = BT_INVALID_CONN_INDEX;
    return rt_bt_app_control(BT_CONTROL_CONNECT_DEVICE_EX, &info);
}

static bt_err_t hfp_disconnect(int argc, char **argv)
{
    bt_profile_t profile = BT_PROFILE_HFP;

    (void)argc;
    (void)argv;
    return rt_bt_app_control(BT_CONTROL_DISCONNECT_EX, &profile);
}

static bt_err_t hfp_make_call(int argc, char **argv)
{
    phone_number_t num = {0};
    rt_size_t      len;

    if (argc < 2)
    {
        LOG_E("make_call requires a phone number");
        return BT_ERROR_INPARAM;
    }
    len = rt_strlen(argv[1]);
    if (len == 0 || len > BT_MAX_PHONE_NUMBER_LEN)
    {
        LOG_E("invalid phone number length: %d", (int)len);
        return BT_ERROR_INPARAM;
    }
    rt_memcpy(num.number, argv[1], len);
    num.size = (int)len;
    return rt_bt_app_control(BT_CONTROL_MAKE_CALL, &num);
}

static bt_err_t hfp_dial_back(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    return rt_bt_app_control(BT_CONTROL_DIAL_BACK, RT_NULL);
}

static bt_err_t hfp_answer(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    return rt_bt_app_control(BT_CONTROL_PHONE_CONNECT, RT_NULL);
}

static bt_err_t hfp_hangup(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    return rt_bt_app_control(BT_CONTROL_PHONE_HANDUP, RT_NULL);
}

static bt_err_t hfp_local_number(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    return rt_bt_app_control(BT_CONTROL_GET_PHONE_NUMBER, RT_NULL);
}

static bt_err_t hfp_remote_calls_info(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    return hfp_query_remote_calls();
}

static bt_err_t hfp_remote_calls_status(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    return rt_bt_app_control(BT_CONTROL_GET_REMOTE_CALL_STATUS, RT_NULL);
}

static bt_err_t hfp_volume_control(int argc, char **argv)
{
    bt_volume_set_t vol = {0};
    int             v;

    if (argc < 2)
    {
        LOG_E("volume_control requires a value 0-15");
        return BT_ERROR_INPARAM;
    }
    v = atoi(argv[1]);
    if (v < 0)  v = 0;
    if (v > 15) v = 15;
    vol.mode               = BT_VOLUME_CALL;
    vol.volume.call_volume = (rt_uint8_t)v;
    vol.save               = BT_VOLUME_NO_SAVE;
    return rt_bt_app_control(BT_CONTROL_SET_VOLUME, &vol);
}

static bt_err_t hfp_audio_connect(int argc, char **argv)
{
    int state = 0; /* 0: connect SCO audio */

    (void)argc;
    (void)argv;
    return rt_bt_app_control(BT_CONTROL_AUDIO_TRANSFER, &state);
}

static bt_err_t hfp_audio_disconnect(int argc, char **argv)
{
    int state = 1; /* 1: disconnect SCO audio */

    (void)argc;
    (void)argv;
    return rt_bt_app_control(BT_CONTROL_AUDIO_TRANSFER, &state);
}

static bt_err_t hfp_battery_update(int argc, char **argv)
{
    rt_uint8_t val;

    if (argc < 2)
    {
        LOG_E("battery_update requires a value 0-9");
        return BT_ERROR_INPARAM;
    }
    val = (rt_uint8_t)atoi(argv[1]);
    if (val > 9)   /* the driver rejects values greater than 9 */
    {
        LOG_E("battery level %d out of range (0..9)", val);
        return BT_ERROR_INPARAM;
    }
    return rt_bt_app_control(BT_CONTROL_UPDATE_BATT_BY_HFP, &val);
}

static bt_err_t hfp_delete_bonded(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    /* Deleting bonded devices uses the connection-manager API, not a
     * BT_CONTROL_* command -- the device framework exposes no equivalent. */
    bt_cm_delete_bonded_devs();
    return BT_EOK;
}

#ifdef BT_USING_DTMF
static bt_err_t hfp_dtmf_key(int argc, char **argv)
{
    bt_dtmf_key_t dtmf;
    char          key;

    if (argc < 2)
    {
        LOG_E("dtmf_key requires a key 0-9, * or #");
        return BT_ERROR_INPARAM;
    }
    key = argv[1][0];
    if (key >= '0' && key <= '9')
    {
        dtmf = (bt_dtmf_key_t)(BT_DTMF_KEY_0 + (key - '0'));
    }
    else if (key == '*')
    {
        dtmf = BT_DTMF_KEY_STAR;
    }
    else if (key == '#')
    {
        dtmf = BT_DTMF_KEY_HASH;
    }
    else
    {
        LOG_E("invalid DTMF key: %c", key);
        return BT_ERROR_INPARAM;
    }
    return rt_bt_app_control(BT_CONTROL_DTMF_DIAL, &dtmf);
}
#endif /* BT_USING_DTMF */

#ifdef BT_USING_SIRI
static bt_err_t hfp_voice_recognition(int argc, char **argv)
{
    rt_bool_t on;

    if (argc < 2)
    {
        LOG_E("voice_recognition requires on or off");
        return BT_ERROR_INPARAM;
    }
    on = (rt_strcmp(argv[1], "on") == 0);
    return rt_bt_app_control(on ? BT_CONTROL_SIRI_ON : BT_CONTROL_SIRI_OFF, RT_NULL);
}
#endif /* BT_USING_SIRI */

/* ---------------------------------------------------------------------------
 * Command table
 * ------------------------------------------------------------------------- */

static const rt_bt_cmd_entry_t hfp_cmds[] =
{
    /* Delete all paired (bonded) devices and clear pairing information. */
    { "c",                   hfp_delete_bonded,       "delete all bonded devices",        RT_FALSE },
    /* Search for nearby discoverable BT devices (inquiry). */
    { "start_inquiry",       hfp_start_inquiry,       "search nearby BT devices",         RT_TRUE  },
    /* Stop the ongoing search. */
    { "stop_inquiry",        hfp_stop_inquiry,        "stop searching",                   RT_TRUE  },
    /* Establish an HFP connection to the phone at the given address: hfp_connect <addr>. */
    { "hfp_connect",         hfp_connect,             "<addr> connect HFP to a phone",    RT_TRUE  },
    /* Disconnect the current HFP connection. */
    { "hfp_disconnect",      hfp_disconnect,          "disconnect HFP",                   RT_TRUE  },
    /* Query the local (subscriber) phone number. */
    { "local_phone_number",  hfp_local_number,        "query local subscriber number",    RT_TRUE  },
    /* Query the remote current call list (CLCC). */
    { "remote_calls_info",   hfp_remote_calls_info,   "query remote call list (CLCC)",    RT_TRUE  },
    /* Query the remote current call status. */
    { "remote_calls_status", hfp_remote_calls_status, "query remote call status",         RT_TRUE  },
    /* Place an outgoing call to the given number: make_call <number>. */
    { "make_call",           hfp_make_call,           "<number> place an outgoing call",  RT_TRUE  },
    /* Redial the last dialed number. */
    { "call_back",           hfp_dial_back,           "redial the last number",           RT_TRUE  },
    /* Answer an incoming call. */
    { "answer_call",         hfp_answer,              "answer an incoming call",          RT_TRUE  },
    /* Hang up the current call. */
    { "hangup_call",         hfp_hangup,              "hang up the current call",         RT_TRUE  },
    /* Set the call speaker volume: volume_control <0-15>. */
    { "volume_control",      hfp_volume_control,      "<0-15> set call speaker volume",   RT_TRUE  },
    /* Establish the call audio (SCO) link. */
    { "audio_connect",       hfp_audio_connect,       "connect call (SCO) audio",         RT_TRUE  },
    /* Disconnect the call audio (SCO) link. */
    { "audio_disconnect",    hfp_audio_disconnect,    "disconnect call (SCO) audio",      RT_TRUE  },
    /* Report the local battery level to the phone via HFP: battery_update <0-9>. */
    { "battery_update",      hfp_battery_update,      "<0-9> report local battery",       RT_TRUE  },
#ifdef BT_USING_DTMF
    /* Send a DTMF key during a call: dtmf_key <0-9,*,#>. */
    { "dtmf_key",            hfp_dtmf_key,            "<0-9,*,#> send a DTMF key",        RT_TRUE  },
#endif
#ifdef BT_USING_SIRI
    /* Toggle the phone's voice assistant (Siri): voice_recognition <on|off>. */
    { "voice_recognition",   hfp_voice_recognition,   "<on|off> toggle voice assistant",  RT_TRUE  },
#endif
};

/* ---------------------------------------------------------------------------
 * Event handler (runs on the core service thread)
 * ------------------------------------------------------------------------- */

static void hfp_on_event(rt_uint16_t event, void *args)
{
    switch (event)
    {
    /* Local phone number reported. */
    case BT_EVENT_LOCAL_CALL_NUMBER:
        LOG_I("local phone number: %s", ((phone_number_t *)args)->number);
        break;

    /* Dial complete: result of an outgoing call (make_call/call_back). */
    case BT_EVENT_DIAL_COMPLETE:
        LOG_I("make a call complete, res %d", *(rt_uint8_t *)args);
        break;

    /* A single call indicator changed (e.g. call/callsetup/callheld).
     * Restart CLCC polling on every change to fetch the latest call details. */
    case BT_EVENT_CIND_IND:
    {
        bt_cind_ind_t *cind = (bt_cind_ind_t *)args;
        LOG_I("call indicator type %d, value %d", cind->type, cind->val);
        hfp_clcc_start();
        break;
    }

    /* Full indicator status: service/signal/battery/roaming reported at once. */
    case BT_EVENT_CINDS_IND:
    {
        bt_cind_data_t *c = (bt_cind_data_t *)args;
        LOG_I("indicators service %d signal %d batt %d roam %d",
              c->service, c->signal, c->batt_level, c->roam);
        break;
    }

    /* Current call list entry (one CLCC record): direction/status/number of a call. */
    case BT_EVENT_CLCC_IND:
    {
        bt_clcc_ind_t *clcc = (bt_clcc_ind_t *)args;
        s_clcc_status = CALL_CLCC_IN_PROGRESS;
        LOG_I("call info idx %d dir %d status %d mode %d mpty %d number %s",
              clcc->idx, clcc->dir, clcc->st, clcc->mode, clcc->mpty,
              clcc->number ? (char *)clcc->number : "");
        break;
    }

    /* Call list query complete: all CLCC records for this round have been reported. */
    case BT_EVENT_CLCC_COMPLETE:
        s_clcc_status = CALL_CLCC_COMPLETE;
        LOG_I("get remote all call information complete, res %d", *(rt_uint8_t *)args);
        break;

    /* Volume set confirmation: result of setting the call speaker volume (VGS). */
    case BT_EVENT_VGS_IND:
        LOG_I("change volume value complete, res %d", *(rt_uint8_t *)args);
        break;

    /* DTMF send confirmation: result of sending a DTMF key. */
    case BT_EVENT_DTMF_IND:
        LOG_I("send DTMF key complete, res %d", *(rt_uint8_t *)args);
        break;

    /* AT command confirmation: result of an AT command (cmd_id + res). */
    case BT_EVENT_AT_CMD_CFM_STATUS:
    {
        bt_at_cmd_cfm_t *cfm = (bt_at_cmd_cfm_t *)args;
        LOG_I("AT cmd 0x%x confirm, res %d", cfm->cmd_id, cfm->res);
        break;
    }

#ifdef BT_USING_SIRI
    /* Notification of whether the peer supports voice recognition (Siri). */
    case BT_EVENT_SIRI_CAPABILITY_NOTIFY:
        LOG_I("remote %s voice recognition", (*(rt_uint8_t *)args) ? "supports" : "does not support");
        break;

    /* Notification of the peer's voice recognition (Siri) on/off state. */
    case BT_EVENT_SIRI_STATE_NOTIFY:
        LOG_I("remote voice recognition state %s", (*(rt_uint8_t *)args) ? "on" : "off");
        break;
#endif

    /* HFP events not handled individually: just print the event code. */
    default:
        LOG_I("unhandled hf event 0x%x", event);
        break;
    }
}

/* ---------------------------------------------------------------------------
 * Clone hook (runs in the BT notification context)
 *
 * Deep-copies the event argument into the core-owned buffer. Most HFP events
 * carry a fixed-size struct (or a single status byte); only CLCC carries a
 * pointer (the phone number) that must be flattened into the buffer.
 * ------------------------------------------------------------------------- */

static rt_size_t hfp_clone(rt_uint16_t event, void *args, void *buf, rt_size_t cap)
{
    switch (event)
    {
    case BT_EVENT_CLCC_IND:
    {
        bt_clcc_ind_t *src = (bt_clcc_ind_t *)args;
        bt_clcc_ind_t *dst = (bt_clcc_ind_t *)buf;
        rt_size_t hdr = sizeof(*dst);
        rt_size_t nlen;

        if (cap < hdr + 1)
        {
            return 0;
        }
        *dst = *src;
        nlen = src->number_size;
        if (nlen > cap - hdr - 1)
        {
            nlen = cap - hdr - 1;
        }
        if (src->number != RT_NULL && nlen > 0)
        {
            rt_memcpy((rt_uint8_t *)buf + hdr, src->number, nlen);
        }
        ((char *)buf)[hdr + nlen] = '\0';
        dst->number      = (rt_uint8_t *)buf + hdr;
        dst->number_size = nlen;
        return hdr + nlen + 1;
    }

    case BT_EVENT_LOCAL_CALL_NUMBER:
        rt_memcpy(buf, args, sizeof(phone_number_t));
        return sizeof(phone_number_t);

    case BT_EVENT_CIND_IND:
        rt_memcpy(buf, args, sizeof(bt_cind_ind_t));
        return sizeof(bt_cind_ind_t);

    case BT_EVENT_CINDS_IND:
        rt_memcpy(buf, args, sizeof(bt_cind_data_t));
        return sizeof(bt_cind_data_t);

    case BT_EVENT_AT_CMD_CFM_STATUS:
        rt_memcpy(buf, args, sizeof(bt_at_cmd_cfm_t));
        return sizeof(bt_at_cmd_cfm_t);

    default:
        /* Single status byte (DIAL_COMPLETE / CLCC_COMPLETE / VGS_IND /
         * DTMF_IND / SIRI_*). */
        *(rt_uint8_t *)buf = *(rt_uint8_t *)args;
        return sizeof(rt_uint8_t);
    }
}

/* ---------------------------------------------------------------------------
 * Service descriptor + self-registration
 * ------------------------------------------------------------------------- */

static rt_bt_service_t hfp_service =
{
    .name        = "hfp",
    .event_group = BT_HF_TYPE_ID,
    .on_event    = hfp_on_event,
    .clone       = hfp_clone,
    .cmds        = hfp_cmds,
    .cmd_num     = sizeof(hfp_cmds) / sizeof(hfp_cmds[0]),
};

RT_BT_SERVICE_REGISTER(hfp_service);
