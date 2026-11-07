/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * GATT server for the bt_peer instrument.
 *
 * The instrument exposes one generic 128-bit service with a readable,
 * writable and notifiable characteristic. The host uses it as the loopback
 * end point of a DUT-initiated transfer: whatever the DUT writes is kept and
 * can be read back, and the host can push notifications that the DUT
 * subscribes to.
 */

#include <rtthread.h>
#include <string.h>

#include "peer.h"
#include "bf0_sibles.h"

#define PEER_SVC_LOOPBACK_MAX   PEER_MAX_PAYLOAD

/* Attribute indices, in ATT database order; handles are start + index. */
enum
{
    PEER_SVC_IDX_SVC,
    PEER_SVC_IDX_CHAR,
    PEER_SVC_IDX_VALUE,
    PEER_SVC_IDX_CCCD,
    PEER_SVC_IDX_NB
};

/* attm_desc_128.uuid is a plain array, so the initializer needs a macro. */
#define SERIAL_UUID_16(x) {((uint8_t)((x) & 0xff)), ((uint8_t)(((x) >> 8) & 0xff))}

/*
 * "bt_peer-loop" and "bt_peer-val" in little-endian UUID byte order. The last
 * four bytes are zero so the value cannot be confused with the service UUID.
 */
#define PEER_SVC_UUID   { 'b', 't', '_', 'p', 'e', 'e', 'r', '-', 'l', 'o', 'o', 'p', 0, 0, 0, 0 }
#define PEER_CHAR_UUID  { 'b', 't', '_', 'p', 'e', 'e', 'r', '-', 'v', 'a', 'l', 0, 0, 0, 0, 0 }

static uint8_t s_svc_uuid[ATT_UUID_128_LEN] = PEER_SVC_UUID;

static struct attm_desc_128 s_att_db[PEER_SVC_IDX_NB] =
{
    /* Service declaration. */
    [PEER_SVC_IDX_SVC] = {SERIAL_UUID_16(ATT_DECL_PRIMARY_SERVICE), PERM(RD, ENABLE), 0, 0},
    /* Characteristic declaration. */
    [PEER_SVC_IDX_CHAR] = {SERIAL_UUID_16(ATT_DECL_CHARACTERISTIC), PERM(RD, ENABLE), 0, 0},
    /* Characteristic value: read, write request, write command and notify. */
    [PEER_SVC_IDX_VALUE] =
    {
        PEER_CHAR_UUID,
        PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE) | PERM(WRITE_COMMAND, ENABLE) |
        PERM(NTF, ENABLE) | PERM(WP, NO_AUTH),
        PERM(UUID_LEN, UUID_128) | PERM(RI, ENABLE),
        PEER_SVC_LOOPBACK_MAX
    },
    /* Client characteristic configuration descriptor. */
    [PEER_SVC_IDX_CCCD] =
    {
        SERIAL_UUID_16(ATT_DESC_CLIENT_CHAR_CFG),
        PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE) | PERM(WP, NO_AUTH),
        PERM(RI, ENABLE), 2
    },
};

static sibles_hdl g_srv_handle;
static uint16_t g_value_handle;
static uint16_t g_cccd_handle;

static uint8_t g_value[PEER_SVC_LOOPBACK_MAX];
static uint16_t g_value_len;
static uint8_t g_cccd_on[PEER_MAX_LINKS];

/** The local ATT database, sized for the stack's default service slots. */
static sibles_local_svc_t s_local_svcs[8];

static uint16_t peer_svc_value_handle(void)
{
    uint8_t i;

    memset(s_local_svcs, 0, sizeof(s_local_svcs));
    sibles_get_all_gatt_handle(s_local_svcs);

    for (i = 0; i < (uint8_t)(sizeof(s_local_svcs) / sizeof(s_local_svcs[0])); i++)
    {
        if (!s_local_svcs[i].state)
            continue;
        if (s_local_svcs[i].uuid_len != ATT_UUID_128_LEN)
            continue;
        if (memcmp(s_local_svcs[i].uuid, s_svc_uuid, ATT_UUID_128_LEN) != 0)
            continue;

        return (uint16_t)(s_local_svcs[i].start_handle + PEER_SVC_IDX_VALUE);
    }

    return 0;
}

static uint8_t *peer_svc_get_cbk(uint8_t conn_idx, uint8_t idx, uint16_t *len)
{
    peer_link_t *link;

    *len = 0;

    if (idx != PEER_SVC_IDX_VALUE)
        return RT_NULL;

    *len = g_value_len;

    /*
     * The DUT read the value we serve. Report it before handing the pointer
     * back so a host waiting for `read` sees the same bytes the peer gets.
     */
    link = peer_link_find_conn(conn_idx);
    if (link != RT_NULL)
    {
        /* Static: GATT callbacks run on the 4 KB mbox thread, next to
         * peer_emit_evt's own line buffer. The stack serializes callbacks. */
        static char payload[PEER_MAX_PAYLOAD * 2 + 32];

        peer_format_payload(payload, sizeof(payload), g_value, g_value_len);
        peer_emit_evt("read", "link=%u handle=%u %s",
                      link->link_id, g_value_handle, payload);
    }

    return g_value;
}

static uint8_t peer_svc_set_cbk(uint8_t conn_idx, sibles_set_cbk_t *para)
{
    peer_link_t *link = peer_link_find_conn(conn_idx);

    if (para->idx == PEER_SVC_IDX_VALUE)
    {
        uint16_t offset = para->offset;
        uint16_t len = para->len;

        /* Prepare-write fragments carry a non-zero offset; writing each at 0
         * would leave only the last fragment. */
        if (offset >= sizeof(g_value))
            return 0;
        if (len > sizeof(g_value) - offset)
            len = sizeof(g_value) - offset;
        if (len > 0 && para->value != RT_NULL)
            memcpy(g_value + offset, para->value, len);
        if (offset + len > g_value_len)
            g_value_len = offset + len;

        if (link != RT_NULL)
        {
            static char payload[PEER_MAX_PAYLOAD * 2 + 32];

            peer_format_payload(payload, sizeof(payload), g_value, g_value_len);
            peer_emit_evt("write", "link=%u handle=%u %s",
                          link->link_id, g_value_handle, payload);
        }
    }
    else if (para->idx == PEER_SVC_IDX_CCCD)
    {
        uint8_t enable = (para->len > 0 && para->value != RT_NULL) ? (para->value[0] & 0x01) : 0;

        if (conn_idx < PEER_MAX_LINKS)
            g_cccd_on[conn_idx] = enable;

        if (link != RT_NULL)
        {
            static char payload[PEER_MAX_PAYLOAD * 2 + 32];

            peer_format_payload(payload, sizeof(payload), para->value,
                                para->len > 2 ? 2 : para->len);
            peer_emit_evt("write", "link=%u handle=%u %s",
                          link->link_id, g_cccd_handle, payload);
        }
    }

    return 0;
}

void peer_gatt_server_init(void)
{
    g_srv_handle = RT_NULL;
    g_value_handle = 0;
    g_cccd_handle = 0;
    g_value_len = 0;
    memset(g_value, 0, sizeof(g_value));
    memset(g_cccd_on, 0, sizeof(g_cccd_on));
    memset(s_local_svcs, 0, sizeof(s_local_svcs));
}

void peer_gatt_server_on_disconnect(uint8_t conn_idx)
{
    if (conn_idx < PEER_MAX_LINKS)
        g_cccd_on[conn_idx] = 0;
}

void peer_gatt_server_reset(void)
{
    /*
     * The stack has no API to remove a registered service, so the database
     * survives state.reset(); only the per-link subscription state is dropped.
     */
    memset(g_cccd_on, 0, sizeof(g_cccd_on));
}

int peer_gatt_server_register(peer_reply_t *reply)
{
    sibles_register_svc_128_t svc;

    if (g_srv_handle != RT_NULL)
    {
        peer_reply_ok(reply, "handle=%u", g_value_handle);
        return 0;
    }

    if (!peer_ble_ready())
    {
        peer_reply_err(reply, PEER_ERR_NOT_READY, "ble");
        return -1;
    }

    memset(&svc, 0, sizeof(svc));
    svc.uuid = s_svc_uuid;
    svc.att_db = s_att_db;
    svc.num_entry = PEER_SVC_IDX_NB;
    svc.sec_lvl = PERM(SVC_AUTH, NO_AUTH) | PERM(SVC_UUID_LEN, UUID_128) |
                  PERM(SVC_MI, ENABLE);

    g_srv_handle = sibles_register_svc_128(&svc);
    if (g_srv_handle == RT_NULL)
    {
        peer_reply_err(reply, PEER_ERR_NO_MEM, "svc.register");
        return -1;
    }

    sibles_register_cbk(g_srv_handle, peer_svc_get_cbk, peer_svc_set_cbk);

    g_value_handle = peer_svc_value_handle();
    if (g_value_handle == 0)
    {
        /* Do not leave a half-registered service behind: a retry would take
         * the idempotent branch and answer @ok handle=0. */
        g_srv_handle = RT_NULL;
        peer_reply_err(reply, PEER_ERR_STATE, "handle");
        return -1;
    }
    g_cccd_handle = (uint16_t)(g_value_handle + 1);

    peer_reply_ok(reply, "handle=%u", g_value_handle);
    return 0;
}

int peer_gatt_server_unregister(peer_reply_t *reply)
{
    /* sibles_stop_svc() is a no-op, so the ATT database stays until reboot. */
    peer_reply_err(reply, PEER_ERR_UNSUPPORTED, "svc.unregister");
    return -1;
}

int peer_gatt_server_notify(peer_link_t *link, uint16_t handle,
                            const uint8_t *data, uint16_t len,
                            peer_reply_t *reply)
{
    sibles_value_t value;
    int ret;

    if (g_srv_handle == RT_NULL)
    {
        peer_reply_err(reply, PEER_ERR_STATE, "svc.register");
        return -1;
    }

    if (handle != 0 && handle != g_value_handle)
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "handle");
        return -1;
    }

    if (link->conn_idx >= PEER_MAX_LINKS || !g_cccd_on[link->conn_idx])
    {
        peer_reply_err(reply, PEER_ERR_STATE, "not_subscribed");
        return -1;
    }

    if (len > PEER_SVC_LOOPBACK_MAX)
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "len");
        return -1;
    }

    memset(&value, 0, sizeof(value));
    value.hdl = g_srv_handle;
    value.idx = PEER_SVC_IDX_VALUE;
    value.len = len;
    value.value = (uint8_t *)data;

    ret = sibles_write_value(link->conn_idx, &value);

    /* A zero-length notification also returns 0, which otherwise looks like
     * TX flow control; it is a successful send, not a busy stack. */
    if (ret == (int)len || (len == 0 && ret == 0))
    {
        peer_reply_ok(reply, "handle=%u", g_value_handle);
        return 0;
    }

    if (ret == 0)
    {
        peer_reply_err(reply, PEER_ERR_BUSY, "tx");
        return -1;
    }

    peer_reply_err(reply, PEER_ERR_WRITE_FAILED, "svc.notify");
    return -1;
}
