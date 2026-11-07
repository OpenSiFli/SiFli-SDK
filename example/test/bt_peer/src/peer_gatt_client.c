/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * GATT client for the bt_peer instrument: discovery, read, write, subscribe
 * and MTU exchange on a connection the instrument holds as central.
 *
 * Every operation is synchronous from the caller's point of view. The BLE
 * callback thread fills in the result and wakes the shell thread, so the
 * terminal ``@ok`` always carries the outcome of the work it names.
 */

#include <rtthread.h>
#include <string.h>

#include "peer.h"
#include "bf0_sibles.h"

/** Discovery and read may span a connection interval or two. */
#define PEER_GATT_TIMEOUT_MS        10000
/** A write with response is acknowledged by the peer, not by the air. */
#define PEER_GATT_WRITE_TIMEOUT_MS  3000
/** MTU exchange is best effort: a peer that never answers keeps 23. */
#define PEER_GATT_MTU_TIMEOUT_MS    3000

/*
 * The stack routes read responses and notifications by looking the handle up
 * in the remote service range registered for a connection, and passes that
 * range to the controller. Register the range the search actually reported,
 * not the whole attribute database, exactly as the SDK examples do.
 */

/** Descriptors reported per characteristic; the protocol line stays bounded. */
#define PEER_GATT_MAX_DESCS         16

/** Empty slot in the per-connection registration table. */
#define PEER_GATT_NO_CONN           0xFF

static uint8_t g_reg_conn[PEER_MAX_LINKS];
static uint16_t g_reg_handle[PEER_MAX_LINKS];
static uint16_t g_reg_start[PEER_MAX_LINKS];
static uint16_t g_reg_end[PEER_MAX_LINKS];
static uint8_t g_reg_count;

static uint8_t g_read_buf[PEER_MAX_PAYLOAD];
static uint16_t g_read_len;
static uint16_t g_read_handle;
static uint8_t g_read_conn = PEER_GATT_NO_CONN;
static int g_read_rc;

static uint8_t g_write_conn = PEER_GATT_NO_CONN;
static int g_write_rc;

static uint8_t g_notify_buf[PEER_MAX_PAYLOAD];

static uint16_t g_char_count;
/** Outcome of the last search: 0 found, -1 not found or failed. */
static int g_search_ok;

/*
 * The stack also runs searches of its own (the service-changed module after
 * bonding). `SIBLES_SEARCH_SVC_RSP` therefore is not necessarily ours; only a
 * response matching this outstanding request may update the link.
 */
static uint8_t g_search_pending;
static uint8_t g_search_conn = PEER_GATT_NO_CONN;
static uint8_t g_search_uuid_len;
static uint8_t g_search_uuid[ATT_UUID_128_LEN];

static void peer_gatt_remote_release(uint8_t conn_idx)
{
    uint8_t i;

    for (i = 0; i < PEER_MAX_LINKS; i++)
    {
        if (g_reg_conn[i] != conn_idx)
            continue;

        sibles_unregister_remote_svc(conn_idx, g_reg_start[i], g_reg_end[i],
                                     peer_gatt_client_remote_cb);
        g_reg_conn[i] = PEER_GATT_NO_CONN;
        g_reg_handle[i] = 0;
        g_reg_start[i] = 0;
        g_reg_end[i] = 0;
        if (g_reg_count > 0)
            g_reg_count--;
    }
}

/** Register the discovered service range for a connection. */
static uint16_t peer_gatt_remote_register(uint8_t conn_idx, uint16_t start,
        uint16_t end)
{
    uint8_t i;
    uint16_t hdl;

    if (start == 0 || end < start)
        return SIBLES_ERROR_REMOTE_HANDLE;

    /* A re-discovery replaces the previous range instead of leaking a slot. */
    peer_gatt_remote_release(conn_idx);

    for (i = 0; i < PEER_MAX_LINKS; i++)
        if (g_reg_conn[i] == PEER_GATT_NO_CONN)
            break;

    if (i == PEER_MAX_LINKS)
        return SIBLES_ERROR_REMOTE_HANDLE;

    hdl = sibles_register_remote_svc(conn_idx, start, end,
                                     peer_gatt_client_remote_cb);
    if (hdl == SIBLES_ERROR_REMOTE_HANDLE)
        return SIBLES_ERROR_REMOTE_HANDLE;

    g_reg_conn[i] = conn_idx;
    g_reg_handle[i] = hdl;
    g_reg_start[i] = start;
    g_reg_end[i] = end;
    g_reg_count++;

    return hdl;
}

/** Take the link's single GATT slot, or report it busy. */
static int peer_gatt_enter(peer_link_t *link, peer_reply_t *reply)
{
    if (link->gatt_busy)
    {
        peer_reply_err(reply, PEER_ERR_BUSY, "gatt");
        return -1;
    }

    link->gatt_busy = 1;
    return 0;
}

/** Same, for operations that need a handle range from a prior discovery. */
static int peer_gatt_enter_remote(peer_link_t *link, peer_reply_t *reply)
{
    if (link->rmt_svc_hdl == SIBLES_ERROR_REMOTE_HANDLE)
    {
        peer_reply_err(reply, PEER_ERR_STATE, "discover");
        return -1;
    }

    return peer_gatt_enter(link, reply);
}

static void peer_gatt_leave(peer_link_t *link)
{
    link->gatt_busy = 0;
}

static void peer_gatt_emit_characteristics(peer_link_t *link,
        const sibles_svc_remote_svc_t *svc)
{
    const sibles_svc_search_char_t *chara =
        (const sibles_svc_search_char_t *)svc->att_db;
    uint8_t i;
    uint8_t d;

    g_char_count = 0;

    if (chara == RT_NULL)
        return;

    for (i = 0; i < svc->char_count; i++)
    {
        char descs[PEER_GATT_MAX_DESCS * 6 + 1];
        size_t used = 0;
        uint8_t reported = chara->desc_count;

        descs[0] = '\0';
        if (reported > PEER_GATT_MAX_DESCS)
            reported = PEER_GATT_MAX_DESCS;

        for (d = 0; d < reported; d++)
        {
            int n = rt_snprintf(descs + used, sizeof(descs) - used, "%s%u",
                                d == 0 ? "" : ",", chara->desc[d].attr_hdl);

            if (n < 0 || used + (size_t)n >= sizeof(descs))
                break;
            used += (size_t)n;
        }

        /* One line per characteristic, all of them before the terminal @ok. */
        peer_emit_evt("char", "link=%u index=%u handle=%u value_handle=%u descs=%s",
                      link->link_id, i, chara->attr_hdl, chara->pointer_hdl, descs);

        g_char_count++;
        chara = (const sibles_svc_search_char_t *)((const uint8_t *)chara +
                sizeof(sibles_svc_search_char_t) +
                chara->desc_count * sizeof(struct sibles_disc_char_desc_ind));
    }
}

void peer_gatt_client_init(void)
{
    uint8_t i;

    for (i = 0; i < PEER_MAX_LINKS; i++)
    {
        g_reg_conn[i] = PEER_GATT_NO_CONN;
        g_reg_handle[i] = 0;
        g_reg_start[i] = 0;
        g_reg_end[i] = 0;
    }
    g_reg_count = 0;
    g_read_conn = PEER_GATT_NO_CONN;
    g_write_conn = PEER_GATT_NO_CONN;
    g_search_ok = -1;
}

void peer_gatt_client_reset(void)
{
    uint8_t i;

    for (i = 0; i < PEER_MAX_LINKS; i++)
    {
        if (g_reg_conn[i] == PEER_GATT_NO_CONN)
            continue;

        sibles_unregister_remote_svc(g_reg_conn[i], g_reg_start[i],
                                     g_reg_end[i],
                                     peer_gatt_client_remote_cb);
        g_reg_conn[i] = PEER_GATT_NO_CONN;
        g_reg_handle[i] = 0;
        g_reg_start[i] = 0;
        g_reg_end[i] = 0;
    }

    g_reg_count = 0;
    g_read_conn = PEER_GATT_NO_CONN;
    g_write_conn = PEER_GATT_NO_CONN;
    g_read_len = 0;
    g_read_rc = -1;
    g_write_rc = -1;
    g_char_count = 0;
    g_search_ok = -1;
}

void peer_gatt_client_on_disconnect(uint8_t conn_idx)
{
    peer_link_t *link = peer_link_find_conn(conn_idx);

    peer_gatt_remote_release(conn_idx);
    if (link != RT_NULL)
        link->rmt_svc_hdl = SIBLES_ERROR_REMOTE_HANDLE;

    if (g_read_conn == conn_idx)
        g_read_conn = PEER_GATT_NO_CONN;
    if (g_write_conn == conn_idx)
        g_write_conn = PEER_GATT_NO_CONN;
}

void peer_gatt_client_on_search(void *rsp_)
{
    sibles_svc_search_rsp_t *rsp = (sibles_svc_search_rsp_t *)rsp_;
    peer_link_t *link;

    /* Ignore the stack's own searches: only the outstanding request may
     * update the link, or a service-changed search after bonding would
     * replace the DUT's registration with a GATT-service range. */
    if (!g_search_pending || rsp->conn_idx != g_search_conn)
        return;

    if (g_search_uuid_len > 0 &&
            (rsp->search_svc_len != g_search_uuid_len ||
             memcmp(rsp->search_uuid, g_search_uuid, g_search_uuid_len) != 0))
        return;

    g_search_pending = 0;
    link = peer_link_find_conn(rsp->conn_idx);
    g_char_count = 0;
    g_search_ok = -1;

    if (link == RT_NULL)
    {
        peer_ble_signal(PEER_BIT_SEARCH_DONE);
        return;
    }

    if (rsp->result != HL_ERR_NO_ERROR || rsp->svc == RT_NULL)
    {
        peer_ble_signal(PEER_BIT_SEARCH_DONE);
        return;
    }

    link->svc_start = rsp->svc->hdl_start;
    link->svc_end = rsp->svc->hdl_end;
    peer_gatt_emit_characteristics(link, rsp->svc);

    /*
     * Register the range the search reported. Read responses and notifications
     * for this service are routed through it; nothing else is.
     */
    link->rmt_svc_hdl = peer_gatt_remote_register(rsp->conn_idx,
                        link->svc_start, link->svc_end);
    if (link->rmt_svc_hdl == SIBLES_ERROR_REMOTE_HANDLE)
    {
        /* Keep the sentinel: slot 0 is a valid registration. */
        g_search_ok = -2;
    }
    else
    {
        g_search_ok = 0;
    }

    peer_ble_signal(PEER_BIT_SEARCH_DONE);
}

void peer_gatt_client_on_write_rsp(void *rsp_)
{
    sibles_write_remote_value_rsp_t *rsp = (sibles_write_remote_value_rsp_t *)rsp_;

    if (g_write_conn == PEER_GATT_NO_CONN || rsp->conn_idx != g_write_conn)
        return;

    g_write_rc = rsp->result;
    peer_ble_signal(PEER_BIT_WRITE_DONE);
}

void peer_gatt_client_on_read_error(void *ind_)
{
    sibles_error_cmp_event_ind_t *ind = (sibles_error_cmp_event_ind_t *)ind_;

    if (g_read_conn == PEER_GATT_NO_CONN || ind->conn_idx != g_read_conn)
        return;

    g_read_rc = -1;
    peer_ble_signal(PEER_BIT_READ_DONE);
}

int peer_gatt_client_remote_cb(uint16_t event_id, uint8_t *data, uint16_t len)
{
    (void)len;

    switch (event_id)
    {
    case SIBLES_REMOTE_EVENT_IND:
    {
        sibles_remote_event_ind_t *ind = (sibles_remote_event_ind_t *)data;
        peer_link_t *link = peer_link_find_conn(ind->conn_idx);
        /* Static: this runs on the 4 KB mbox thread, next to peer_emit_evt's
         * own 1 KB line buffer. The callback is serialized by the stack. */
        static char payload[PEER_MAX_PAYLOAD * 2 + 32];
        uint16_t copy = ind->length;

        if (link == RT_NULL || ind->value == RT_NULL)
            break;

        if (copy > sizeof(g_notify_buf))
            copy = sizeof(g_notify_buf);
        memcpy(g_notify_buf, ind->value, copy);

        peer_format_payload(payload, sizeof(payload), g_notify_buf, copy);
        peer_emit_evt("notify", "link=%u handle=%u %s",
                      link->link_id, ind->handle, payload);
        break;
    }

    case SIBLES_READ_REMOTE_VALUE_RSP:
    {
        sibles_read_remote_value_rsp_t *rsp =
            (sibles_read_remote_value_rsp_t *)data;
        uint16_t copy = rsp->length;

        if (rsp->conn_idx != g_read_conn)
            break;

        if (copy > sizeof(g_read_buf))
            copy = sizeof(g_read_buf);
        if (copy > 0 && rsp->value != RT_NULL)
            memcpy(g_read_buf, rsp->value, copy);

        g_read_len = copy;
        g_read_handle = rsp->handle;
        g_read_rc = 0;
        peer_ble_signal(PEER_BIT_READ_DONE);
        break;
    }

    case SIBLES_REGISTER_REMOTE_SVC_RSP:
    {
        sibles_register_remote_svc_rsp_t *rsp =
            (sibles_register_remote_svc_rsp_t *)data;

        if (rsp->status != HL_ERR_NO_ERROR)
            rt_kprintf("bt_peer: remote svc register failed %u on conn %u\n",
                       rsp->status, rsp->conn_idx);
        break;
    }

    default:
        break;
    }

    return 0;
}

int peer_gatt_client_discover(peer_link_t *link, const uint8_t *uuid,
                              uint8_t uuid_len, peer_reply_t *reply)
{
    int rc = -1;

    if (peer_gatt_enter(link, reply) < 0)
        return -1;

    g_char_count = 0;
    g_search_ok = -1;
    /* A previous discovery leaves this set; without clearing it the wait
     * below returns immediately with that result instead of this one's. */
    peer_ble_clear_bits(PEER_BIT_SEARCH_DONE);

    g_search_pending = 1;
    g_search_conn = link->conn_idx;
    g_search_uuid_len = uuid_len;
    memset(g_search_uuid, 0, sizeof(g_search_uuid));
    if (uuid_len > 0)
        memcpy(g_search_uuid, uuid, uuid_len);

    if (sibles_search_service(link->conn_idx, uuid_len, (uint8_t *)uuid) != 0)
    {
        peer_reply_err(reply, PEER_ERR_BUSY, "search");
        goto out;
    }

    if (peer_ble_wait(PEER_BIT_SEARCH_DONE, PEER_GATT_TIMEOUT_MS) != RT_EOK)
    {
        peer_reply_err(reply, PEER_ERR_TIMEOUT, "gatt.discover");
        goto out;
    }

    if (g_search_ok == -2)
    {
        peer_reply_err(reply, PEER_ERR_NO_MEM, "remote_svc");
        goto out;
    }

    if (g_search_ok != 0)
    {
        peer_reply_err(reply, PEER_ERR_NO_SERVICE, "gatt.discover");
        goto out;
    }

    peer_reply_ok(reply, "count=%u", g_char_count);
    rc = 0;

out:
    g_search_pending = 0;
    peer_gatt_leave(link);
    return rc;
}

int peer_gatt_client_read(peer_link_t *link, uint16_t handle,
                          peer_reply_t *reply)
{
    sibles_read_remote_value_req_t req;
    static char payload[PEER_MAX_PAYLOAD * 2 + 32];
    int rc = -1;

    if (peer_gatt_enter_remote(link, reply) < 0)
        return -1;

    memset(&req, 0, sizeof(req));
    req.read_type = SIBLES_READ;
    req.handle = handle;
    req.offset = 0;
    req.length = 0;

    g_read_len = 0;
    g_read_rc = -1;
    g_read_handle = 0;
    g_read_conn = link->conn_idx;
    peer_ble_clear_bits(PEER_BIT_READ_DONE);

    if (sibles_read_remote_value(link->rmt_svc_hdl, link->conn_idx, &req) != 0)
    {
        peer_reply_err(reply, PEER_ERR_READ_FAILED, "gatt.read");
        goto out;
    }

    if (peer_ble_wait(PEER_BIT_READ_DONE, PEER_GATT_TIMEOUT_MS) != RT_EOK)
    {
        peer_reply_err(reply, PEER_ERR_TIMEOUT, "gatt.read");
        goto out;
    }

    if (g_read_rc != 0)
    {
        peer_reply_err(reply, PEER_ERR_READ_FAILED, "gatt.read");
        goto out;
    }

    if (peer_format_payload(payload, sizeof(payload), g_read_buf, g_read_len) < 0)
    {
        peer_reply_err(reply, PEER_ERR_NO_MEM, "gatt.read");
        goto out;
    }

    peer_reply_ok(reply, "%s", payload);
    rc = 0;

out:
    g_read_conn = PEER_GATT_NO_CONN;
    peer_gatt_leave(link);
    return rc;
}

static int peer_gatt_write_raw(peer_link_t *link, uint16_t handle,
                               const uint8_t *data, uint16_t len,
                               uint8_t response, peer_reply_t *reply)
{
    sibles_write_remote_value_t value;
    int8_t ret;

    memset(&value, 0, sizeof(value));
    value.handle = handle;
    value.write_type = response ? SIBLES_WRITE : SIBLES_WRITE_WITHOUT_RSP;
    value.len = len;
    value.value = (uint8_t *)data;

    g_write_rc = -1;
    /* Set before the request: the response can arrive before the call returns,
     * and a response that finds g_write_conn unset would be dropped. */
    g_write_conn = response ? link->conn_idx : PEER_GATT_NO_CONN;
    peer_ble_clear_bits(PEER_BIT_WRITE_DONE);

    ret = sibles_write_remote_value(link->rmt_svc_hdl, link->conn_idx, &value);
    if (ret == SIBLES_WIRTE_TX_FLOWCTRL_ERR)
    {
        g_write_conn = PEER_GATT_NO_CONN;
        peer_reply_err(reply, PEER_ERR_BUSY, "tx");
        return -1;
    }
    if (ret != SIBLES_WRITE_NO_ERR)
    {
        g_write_conn = PEER_GATT_NO_CONN;
        peer_reply_err(reply, PEER_ERR_WRITE_FAILED, "gatt.write");
        return -1;
    }

    if (response)
    {
        if (peer_ble_wait(PEER_BIT_WRITE_DONE, PEER_GATT_WRITE_TIMEOUT_MS) != RT_EOK)
        {
            g_write_conn = PEER_GATT_NO_CONN;
            peer_reply_err(reply, PEER_ERR_TIMEOUT, "gatt.write");
            return -1;
        }
        g_write_conn = PEER_GATT_NO_CONN;

        if (g_write_rc != 0)
        {
            peer_reply_err(reply, PEER_ERR_WRITE_FAILED, "gatt.write");
            return -1;
        }
    }
    else
    {
        /* The stack still acks the command; do not let it wake a later op. */
        g_write_conn = PEER_GATT_NO_CONN;
    }

    return 0;
}

int peer_gatt_client_write(peer_link_t *link, uint16_t handle,
                           const uint8_t *data, uint16_t len,
                           uint8_t response, peer_reply_t *reply)
{
    int rc;

    if (peer_gatt_enter_remote(link, reply) < 0)
        return -1;

    rc = peer_gatt_write_raw(link, handle, data, len, response, reply);
    if (rc == 0)
        peer_reply_ok(reply, "handle=%u", handle);

    peer_gatt_leave(link);
    return rc;
}

int peer_gatt_client_subscribe(peer_link_t *link, uint16_t cccd,
                               uint8_t enable, peer_reply_t *reply)
{
    uint8_t value[2];
    int rc;

    if (peer_gatt_enter_remote(link, reply) < 0)
        return -1;

    value[0] = enable ? 0x01 : 0x00;
    value[1] = 0x00;

    rc = peer_gatt_write_raw(link, cccd, value, sizeof(value), 1, reply);
    if (rc == 0)
        peer_reply_ok(reply, "cccd=%u enable=%u", cccd, enable);

    peer_gatt_leave(link);
    return rc;
}

int peer_gatt_client_mtu(peer_link_t *link, peer_reply_t *reply)
{
    /* MTU exchange needs no discovered service, only the link itself. */
    if (peer_gatt_enter(link, reply) < 0)
        return -1;

    /*
     * The connection already asked for an MTU when it came up, so a link that
     * negotiated more than the default has nothing left to do. Only a link
     * still at 23 asks again; a silent peer keeps 23 and the caller decides.
     */
    if (link->mtu <= 23)
    {
        peer_ble_clear_bits(PEER_BIT_MTU_DONE);
        sibles_exchange_mtu(link->conn_idx);
        peer_ble_wait(PEER_BIT_MTU_DONE, PEER_GATT_MTU_TIMEOUT_MS);
    }

    peer_reply_ok(reply, "mtu=%u", link->mtu);
    peer_gatt_leave(link);
    return 0;
}
