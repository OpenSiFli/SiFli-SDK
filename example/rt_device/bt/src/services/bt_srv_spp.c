/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rt_bt_app.h"

#include <string.h>
#include "bf0_ble_common.h"   /* bt_addr_convert_from_string_to_general() */

#define DBG_TAG "bt.srv.spp"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

/* The SPP service channel this demo uses. Real applications may manage several. */
#define BT_SPP_DEMO_SRV_CHL   1

/* ---------------------------------------------------------------------------
 * 1. Device-command wrappers
 *
 * Each wrapper builds the argument struct expected by a BT_CONTROL_* command
 * and hands it to the core via rt_bt_app_control(). The core forwards it to the
 * BT device through rt_device_control().
 * ------------------------------------------------------------------------- */

/** @brief  Send a text payload over SPP to a peer address on the demo channel. */
static bt_err_t spp_dev_send(const char *addr_str, const char *text)
{
    spp_data_t data = {0};
    bd_addr_t  addr = {0};

    if (bt_addr_convert_from_string_to_general((char *)addr_str, &addr) != BT_MAX_MAC_LEN)
    {
        LOG_E("invalid BT address: %s", addr_str);
        return BT_ERROR_INPARAM;
    }
    rt_memcpy(data.mac_addr.addr, addr.addr, BT_MAX_MAC_LEN);
    data.srv_chl = BT_SPP_DEMO_SRV_CHL;
    data.data    = (uint8_t *)text;
    data.len     = (uint16_t)rt_strlen(text);
    return rt_bt_app_control(BT_CONTROL_SEND_SPP_DATA, &data);
}

/** @brief  Request disconnection of the SPP channel with a peer. */
static bt_err_t spp_dev_disconnect(const char *addr_str)
{
    spp_common_t info = {0};
    bd_addr_t    addr = {0};

    if (bt_addr_convert_from_string_to_general((char *)addr_str, &addr) != BT_MAX_MAC_LEN)
    {
        LOG_E("invalid BT address: %s", addr_str);
        return BT_ERROR_INPARAM;
    }
    rt_memcpy(info.mac_addr.addr, addr.addr, BT_MAX_MAC_LEN);
    info.srv_chl = BT_SPP_DEMO_SRV_CHL;
    return rt_bt_app_control(BT_CONTROL_SEND_SPP_DISC_REQ, &info);
}

/* ---------------------------------------------------------------------------
 * 2. Command table  ("bt spp <cmd> ...")
 *
 * argv[0] is the sub-command word; arguments start at argv[1].
 * ------------------------------------------------------------------------- */

static bt_err_t cmd_send(int argc, char **argv)
{
    if (argc < 3)
    {
        LOG_E("usage: bt spp send <addr> <text>");
        return BT_ERROR_INPARAM;
    }
    return spp_dev_send(argv[1], argv[2]);
}

static bt_err_t cmd_disconnect(int argc, char **argv)
{
    if (argc < 2)
    {
        LOG_E("usage: bt spp disconnect <addr>");
        return BT_ERROR_INPARAM;
    }
    return spp_dev_disconnect(argv[1]);
}

static const rt_bt_cmd_entry_t spp_cmds[] =
{
    { "send",       cmd_send,       "send <addr> <text>  send a string over SPP", RT_TRUE },
    { "disconnect", cmd_disconnect, "disconnect <addr>   close the SPP channel",  RT_TRUE },
};

/* ---------------------------------------------------------------------------
 * 3. Event handler  (runs on the core service thread)
 *
 * Reacts to events in the SPP group (high byte == BT_SPP_TYPE_ID). The core has
 * already deep-copied the argument (see the clone hook below), so @p args is
 * safe to read here.
 * ------------------------------------------------------------------------- */

static void spp_on_event(rt_uint16_t event, void *args)
{
    switch (event)
    {
    case BT_EVENT_SPP_CONN_IND:
        LOG_I("SPP connected");
        break;

    case BT_EVENT_SPP_DATA_IND:
    {
        /* The public payload descriptor. Note: data.data points into the
         * cloned buffer (see spp_clone). */
        spp_data_t *d = (spp_data_t *)args;
        LOG_I("SPP data received on chl %d, len %d", d->srv_chl, d->len);
        break;
    }

    case BT_EVENT_SPP_DATA_CFM:
        LOG_I("SPP data sent");
        break;

    case BT_EVENT_SPP_DISCONN_IND:
        LOG_I("SPP disconnected");
        break;

    default:
        LOG_I("unhandled SPP event 0x%x", event);
        break;
    }
}

/* ---------------------------------------------------------------------------
 * 4. Clone hook  (runs in the BT notification context)
 *
 * Copies the event argument into the core-owned buffer. For data-carrying
 * events the payload pointer must be flattened into the buffer so it stays
 * valid on the service thread. Fixed-size events are a plain shallow copy.
 * ------------------------------------------------------------------------- */

static rt_size_t spp_clone(rt_uint16_t event, void *args, void *buf, rt_size_t cap)
{
    if (event == BT_EVENT_SPP_DATA_IND)
    {
        spp_data_t *src = (spp_data_t *)args;
        spp_data_t *dst = (spp_data_t *)buf;
        rt_size_t   hdr = sizeof(*dst);
        rt_size_t   len = src->len;

        if (cap < hdr)
        {
            return 0;
        }
        *dst = *src;
        if (len > cap - hdr)
        {
            len = cap - hdr;
        }
        if (src->data != RT_NULL && len > 0)
        {
            rt_memcpy((rt_uint8_t *)buf + hdr, src->data, len);
        }
        dst->data = (uint8_t *)buf + hdr;
        dst->len  = (uint16_t)len;
        return hdr + len;
    }

    /* Fixed-size SPP events: shallow copy of the common descriptor. */
    {
        rt_size_t n = sizeof(spp_common_t);
        if (n > cap)
        {
            n = cap;
        }
        rt_memcpy(buf, args, n);
        return n;
    }
}

/* ---------------------------------------------------------------------------
 * Descriptor + self-registration
 * ------------------------------------------------------------------------- */

static rt_bt_service_t spp_service =
{
    .name        = "spp",
    .event_group = BT_SPP_TYPE_ID,
    .on_event    = spp_on_event,
    .clone       = spp_clone,
    .cmds        = spp_cmds,
    .cmd_num     = sizeof(spp_cmds) / sizeof(spp_cmds[0]),
};

RT_BT_SERVICE_REGISTER(spp_service);
