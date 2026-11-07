/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Line protocol used by the bt_peer instrument.
 *
 * Host -> device:
 *   peer info
 *   peer req <id> <op> [key=value ...]
 *   peer data <id> <hex>
 *
 * Device -> host:
 *   @info proto=1 ...
 *   @ok  <id> [key=value ...]
 *   @err <id> <code> <text>
 *   @evt <seq> <name> [key=value ...]
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>

#include "peer.h"

static rt_mutex_t g_out_lock;
static uint32_t g_evt_seq;

void peer_proto_init(void)
{
    if (g_out_lock == RT_NULL)
        g_out_lock = rt_mutex_create("peer_out", RT_IPC_FLAG_FIFO);
}

static void peer_line_out(const char *line)
{
    rt_device_t console;
    rt_uint16_t old_flag;

    if (g_out_lock != RT_NULL)
        rt_mutex_take(g_out_lock, RT_WAITING_FOREVER);

    /*
     * Write to the console device directly instead of rt_kprintf(). Once ulog
     * is initialised rt_kprintf() goes through ulog_vraw(), whose line buffer
     * is ULOG_LINE_BUF_SIZE (128 bytes): a 143-byte advertising report is a
     * ~350-byte protocol line and would be truncated mid-token, losing the
     * newline too. The serial TX ring buffer is 4096 bytes, so one
     * rt_device_write() carries the whole line.
     *
     * One call per line, under the mutex, so BLE callbacks and the shell
     * thread cannot interleave a protocol line.
     */
    console = rt_console_get_device();
    if (console != RT_NULL)
    {
        old_flag = console->open_flag;
        /* STREAM expands "\n" to "\r\n", matching the shell's expectations. */
        console->open_flag |= RT_DEVICE_FLAG_STREAM;
        console->open_flag &= ~RT_DEVICE_FLAG_DMA_TX;
        /* A leading newline terminates the shell prompt, which is printed
         * without one; otherwise an event can arrive as "msh />@evt ...". */
        rt_device_write(console, 0, "\n", 1);
        rt_device_write(console, 0, line, rt_strlen(line));
        rt_device_write(console, 0, "\n", 1);
        console->open_flag = old_flag;
    }

    if (g_out_lock != RT_NULL)
        rt_mutex_release(g_out_lock);
}

static uint32_t peer_evt_seq_next(void)
{
    uint32_t seq;

    rt_enter_critical();
    seq = ++g_evt_seq;
    rt_exit_critical();

    return seq;
}

void peer_emit_evt(const char *name, const char *fmt, ...)
{
    char line[PEER_LINE_MAX];
    int n;
    uint32_t seq = peer_evt_seq_next();

    n = rt_snprintf(line, sizeof(line), "@evt %u %s", seq, name);
    if (n < 0 || n >= (int)sizeof(line))
        return;

    if (fmt != RT_NULL && fmt[0] != '\0')
    {
        va_list ap;
        line[n++] = ' ';
        va_start(ap, fmt);
        rt_vsnprintf(line + n, sizeof(line) - n, fmt, ap);
        va_end(ap);
    }

    peer_line_out(line);
}

void peer_reply_ok(peer_reply_t *reply, const char *fmt, ...)
{
    reply->code = RT_NULL;
    reply->text[0] = '\0';
    reply->extra[0] = '\0';

    if (fmt != RT_NULL && fmt[0] != '\0')
    {
        va_list ap;
        va_start(ap, fmt);
        rt_vsnprintf(reply->extra, sizeof(reply->extra), fmt, ap);
        va_end(ap);
    }
}

void peer_reply_err(peer_reply_t *reply, const char *code, const char *fmt, ...)
{
    reply->code = code;
    reply->extra[0] = '\0';
    reply->text[0] = '\0';

    if (fmt != RT_NULL && fmt[0] != '\0')
    {
        va_list ap;
        va_start(ap, fmt);
        rt_vsnprintf(reply->text, sizeof(reply->text), fmt, ap);
        va_end(ap);
    }
}

const char *peer_arg_get(const peer_args_t *args, const char *key)
{
    uint8_t i;

    for (i = 0; i < args->count; i++)
        if (strcmp(args->items[i].key, key) == 0)
            return args->items[i].value;

    return RT_NULL;
}

int peer_arg_int(const peer_args_t *args, const char *key, int def, int *out)
{
    const char *value = peer_arg_get(args, key);
    char *end = RT_NULL;
    long parsed;

    if (value == RT_NULL)
    {
        *out = def;
        return 0;
    }

    if (value[0] == '\0')
        return -1;

    parsed = strtol(value, &end, 0);
    if (end == value || *end != '\0')
        return -1;

    *out = (int)parsed;
    return 0;
}

static int peer_hex_nibble(char c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    return -1;
}

int peer_hex_decode(const char *hex, uint8_t *out, size_t max)
{
    size_t len, i;

    if (hex == RT_NULL)
        return -1;

    len = strlen(hex);
    if ((len & 1) != 0)
        return -1;
    if (len / 2 > max)
        return -1;

    for (i = 0; i < len / 2; i++)
    {
        int hi = peer_hex_nibble(hex[i * 2]);
        int lo = peer_hex_nibble(hex[i * 2 + 1]);

        if (hi < 0 || lo < 0)
            return -1;
        out[i] = (uint8_t)((hi << 4) | lo);
    }

    return (int)(len / 2);
}

int peer_arg_payload(const peer_args_t *args, uint8_t *out, size_t max, uint16_t *len)
{
    const char *data = peer_arg_get(args, "data");
    int want = -1;
    int decoded;

    if (peer_arg_int(args, "len", -1, &want) < 0)
        return -1;

    if (data == RT_NULL)
    {
        if (want == 0)
        {
            *len = 0;
            return 0;
        }
        return -1;
    }

    decoded = peer_hex_decode(data, out, max);
    if (decoded < 0)
        return -1;

    if (want >= 0 && want != decoded)
        return -1;

    *len = (uint16_t)decoded;
    return 0;
}

int peer_format_payload(char *out, size_t out_size, const uint8_t *data, uint16_t len)
{
    static const char hex[] = "0123456789abcdef";
    size_t used;
    uint16_t i;

    used = (size_t)rt_snprintf(out, out_size, "len=%u data=", len);
    if (used >= out_size)
        return -1;

    for (i = 0; i < len; i++)
    {
        if (used + 2 >= out_size)
            return -1;
        out[used++] = hex[data[i] >> 4];
        out[used++] = hex[data[i] & 0x0F];
    }

    out[used] = '\0';
    return (int)used;
}

static void peer_emit_terminal(const char *id, const peer_reply_t *reply, int rc)
{
    char line[PEER_LINE_MAX];

    if (rc == 0)
    {
        if (reply->extra[0] != '\0')
            rt_snprintf(line, sizeof(line), "@ok %s %s", id, reply->extra);
        else
            rt_snprintf(line, sizeof(line), "@ok %s", id);
    }
    else
    {
        rt_snprintf(line, sizeof(line), "@err %s %s %s", id,
                    reply->code != RT_NULL ? reply->code : "error",
                    reply->text);
    }

    peer_line_out(line);
}

static void peer_emit_err_raw(const char *id, const char *code, const char *text)
{
    char line[PEER_LINE_MAX];

    rt_snprintf(line, sizeof(line), "@err %s %s %s", id, code, text);
    peer_line_out(line);
}

static void peer_emit_info(void)
{
    char line[PEER_LINE_MAX];

    rt_snprintf(line, sizeof(line),
                "@info proto=%d fw=%s caps=%s roles=central,peripheral max_conn=%d ble=5.2 max_line=%d",
                PEER_PROTO_VERSION, PEER_FW_ID, PEER_CAPS, PEER_MAX_LINKS,
                PEER_LINE_MAX);
    peer_line_out(line);
}

static peer_link_t *peer_require_link(const peer_args_t *args, peer_reply_t *reply)
{
    int link_id;
    peer_link_t *link;

    if (peer_arg_int(args, "link", -1, &link_id) < 0 || link_id < 0)
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "link");
        return RT_NULL;
    }

    link = peer_link_find((uint8_t)link_id);
    if (link == RT_NULL)
    {
        peer_reply_err(reply, PEER_ERR_STATE, "link");
        return RT_NULL;
    }

    return link;
}

static int peer_op_state_reset(const peer_args_t *args, peer_reply_t *reply)
{
    (void)args;

    peer_periodic_reset();
    peer_ble_reset();
    peer_gatt_client_reset();
    peer_gatt_server_reset();
    peer_reply_ok(reply, "");
    return 0;
}

static int peer_op_gap_scan(const peer_args_t *args, peer_reply_t *reply)
{
    int duration_ms, interval_ms, window_ms, dup, rssi_min;

    if (peer_arg_int(args, "duration_ms", 3000, &duration_ms) < 0 ||
            peer_arg_int(args, "interval_ms", 100, &interval_ms) < 0 ||
            peer_arg_int(args, "window_ms", 100, &window_ms) < 0 ||
            peer_arg_int(args, "dup", 0, &dup) < 0 ||
            peer_arg_int(args, "rssi_min", -127, &rssi_min) < 0)
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "gap.scan");
        return -1;
    }

    /* duration_ms/10 is what the controller takes; 0 means "no timeout". */
    if (duration_ms < 10 || interval_ms <= 0 || window_ms <= 0 ||
            window_ms > interval_ms || (dup != 0 && dup != 1) ||
            rssi_min < -127 || rssi_min > 20)
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "gap.scan");
        return -1;
    }

    return peer_ble_scan_start((uint32_t)duration_ms, (uint32_t)interval_ms,
                               (uint32_t)window_ms, (uint8_t)dup,
                               (int8_t)rssi_min, reply);
}

static int peer_op_gap_scan_stop(const peer_args_t *args, peer_reply_t *reply)
{
    (void)args;
    return peer_ble_scan_stop(reply);
}

static int peer_op_gap_connect(const peer_args_t *args, peer_reply_t *reply)
{
    const peer_adv_report_t *report = RT_NULL;
    const char *addr_hex = peer_arg_get(args, "addr");
    uint8_t addr[BD_ADDR_LEN];
    uint8_t addr_type = 0;
    uint8_t link_id = 0;
    int index = 0;
    int timeout_ms, interval_ms, type;

    if (peer_arg_int(args, "index", 0, &index) < 0 ||
            peer_arg_int(args, "addr_type", 0, &type) < 0 ||
            peer_arg_int(args, "timeout_ms", 10000, &timeout_ms) < 0 ||
            peer_arg_int(args, "interval_ms", 30, &interval_ms) < 0)
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "gap.connect");
        return -1;
    }
    addr_type = (uint8_t)type;

    if (index > 0)
    {
        /* Check the range before the cast: index=257 would wrap to 1 and
         * silently connect to the wrong report. */
        if (index > (int)peer_ble_adv_report_count())
        {
            peer_reply_err(reply, PEER_ERR_INVALID_ARG, "index");
            return -1;
        }
        report = peer_ble_adv_report((uint8_t)index);
        if (report == RT_NULL)
        {
            peer_reply_err(reply, PEER_ERR_INVALID_ARG, "index");
            return -1;
        }
    }
    else if (addr_hex != RT_NULL)
    {
        if (peer_hex_decode(addr_hex, addr, sizeof(addr)) != BD_ADDR_LEN)
        {
            peer_reply_err(reply, PEER_ERR_INVALID_ARG, "addr");
            return -1;
        }
    }
    else
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "index_or_addr");
        return -1;
    }

    if (timeout_ms <= 0 || interval_ms < 8)
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "timeout_ms_or_interval_ms");
        return -1;
    }

    if (peer_ble_connect(report, addr, addr_type, (uint32_t)timeout_ms,
                         (uint32_t)interval_ms, &link_id, reply) < 0)
        return -1;

    peer_reply_ok(reply, "link=%u", link_id);
    return 0;
}

static int peer_op_gap_disconnect(const peer_args_t *args, peer_reply_t *reply)
{
    peer_link_t *link = peer_require_link(args, reply);

    if (link == RT_NULL)
        return -1;

    if (peer_ble_disconnect(link->link_id, reply) < 0)
        return -1;

    peer_reply_ok(reply, "link=%u", link->link_id);
    return 0;
}

static int peer_op_dev_list(const peer_args_t *args, peer_reply_t *reply)
{
    uint8_t slot;
    uint8_t count = 0;
    char addr[BD_ADDR_LEN * 2 + 1];

    (void)args;

    for (slot = 0; slot < PEER_MAX_LINKS; slot++)
    {
        peer_link_t *link = peer_link_at(slot);

        if (link == RT_NULL)
            continue;

        peer_ble_addr_str(&link->addr, addr, sizeof(addr));
        peer_emit_evt("link", "link=%u addr=%s role=%u interval_ms=%u",
                      link->link_id, addr, link->role,
                      (unsigned)(link->interval * 5 / 4));
        count++;
    }

    peer_reply_ok(reply, "links=%u", count);
    return 0;
}

static int peer_op_gatt_discover(const peer_args_t *args, peer_reply_t *reply)
{
    peer_link_t *link = peer_require_link(args, reply);
    const char *uuid_hex = peer_arg_get(args, "uuid");
    uint8_t uuid[ATT_UUID_128_LEN];
    uint8_t uuid_len = 0;

    if (link == RT_NULL)
        return -1;

    memset(uuid, 0, sizeof(uuid));

    if (uuid_hex != RT_NULL)
    {
        int decoded = peer_hex_decode(uuid_hex, uuid, sizeof(uuid));

        if (decoded != ATT_UUID_16_LEN && decoded != ATT_UUID_128_LEN)
        {
            peer_reply_err(reply, PEER_ERR_INVALID_ARG, "uuid");
            return -1;
        }
        uuid_len = (uint8_t)decoded;
    }

    return peer_gatt_client_discover(link, uuid, uuid_len, reply);
}

static int peer_op_gatt_read(const peer_args_t *args, peer_reply_t *reply)
{
    peer_link_t *link = peer_require_link(args, reply);
    int handle;

    if (link == RT_NULL)
        return -1;

    if (peer_arg_int(args, "handle", -1, &handle) < 0 || handle <= 0 || handle > 0xFFFF)
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "handle");
        return -1;
    }

    return peer_gatt_client_read(link, (uint16_t)handle, reply);
}

static int peer_op_gatt_write(const peer_args_t *args, peer_reply_t *reply)
{
    static uint8_t payload[PEER_MAX_PAYLOAD];
    peer_link_t *link = peer_require_link(args, reply);
    uint16_t len = 0;
    int handle, response;

    if (link == RT_NULL)
        return -1;

    if (peer_arg_int(args, "handle", -1, &handle) < 0 || handle <= 0 || handle > 0xFFFF)
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "handle");
        return -1;
    }

    if (peer_arg_int(args, "response", 1, &response) < 0 || (response != 0 && response != 1))
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "response");
        return -1;
    }

    if (peer_arg_payload(args, payload, sizeof(payload), &len) < 0)
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "payload");
        return -1;
    }

    return peer_gatt_client_write(link, (uint16_t)handle, payload, len,
                                  (uint8_t)response, reply);
}

static int peer_op_gatt_subscribe(const peer_args_t *args, peer_reply_t *reply)
{
    peer_link_t *link = peer_require_link(args, reply);
    int cccd, enable;

    if (link == RT_NULL)
        return -1;

    if (peer_arg_int(args, "cccd", -1, &cccd) < 0 || cccd <= 0 || cccd > 0xFFFF)
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "cccd");
        return -1;
    }

    if (peer_arg_int(args, "enable", 1, &enable) < 0 || (enable != 0 && enable != 1))
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "enable");
        return -1;
    }

    return peer_gatt_client_subscribe(link, (uint16_t)cccd, (uint8_t)enable, reply);
}

static int peer_op_gatt_mtu(const peer_args_t *args, peer_reply_t *reply)
{
    peer_link_t *link = peer_require_link(args, reply);

    if (link == RT_NULL)
        return -1;

    return peer_gatt_client_mtu(link, reply);
}

static int peer_op_adv_start(const peer_args_t *args, peer_reply_t *reply)
{
    static uint8_t payload[PEER_MAX_PAYLOAD];
    const char *mode = peer_arg_get(args, "mode");
    uint16_t len = 0;
    int interval_ms, connectable;

    if (mode == RT_NULL)
        mode = "extended";

    if (strcmp(mode, "legacy") != 0 && strcmp(mode, "extended") != 0)
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "mode");
        return -1;
    }

    if (peer_arg_int(args, "interval_ms", 30, &interval_ms) < 0 ||
            peer_arg_int(args, "connectable", 1, &connectable) < 0)
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "adv.start");
        return -1;
    }

    if (interval_ms < 20 || (connectable != 0 && connectable != 1))
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "adv.start");
        return -1;
    }

    if (peer_arg_payload(args, payload, sizeof(payload), &len) < 0)
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "payload");
        return -1;
    }

    if (strcmp(mode, "legacy") == 0 && len > 31)
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "len");
        return -1;
    }

    return peer_ble_adv_start(strcmp(mode, "extended") == 0, (uint32_t)interval_ms,
                              (uint8_t)connectable, payload, len, reply);
}

static int peer_op_adv_stop(const peer_args_t *args, peer_reply_t *reply)
{
    (void)args;
    return peer_ble_adv_stop(reply);
}

static int peer_op_svc_register(const peer_args_t *args, peer_reply_t *reply)
{
    const char *name = peer_arg_get(args, "name");

    if (name == RT_NULL)
        name = "loopback";

    if (strcmp(name, "loopback") != 0)
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "name");
        return -1;
    }

    return peer_gatt_server_register(reply);
}

static int peer_op_svc_unregister(const peer_args_t *args, peer_reply_t *reply)
{
    const char *name = peer_arg_get(args, "name");

    if (name != RT_NULL && strcmp(name, "loopback") != 0)
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "name");
        return -1;
    }

    return peer_gatt_server_unregister(reply);
}

static int peer_op_svc_notify(const peer_args_t *args, peer_reply_t *reply)
{
    static uint8_t payload[PEER_MAX_PAYLOAD];
    peer_link_t *link = peer_require_link(args, reply);
    uint16_t len = 0;
    int handle;

    if (link == RT_NULL)
        return -1;

    if (peer_arg_int(args, "handle", 0, &handle) < 0 || handle < 0 || handle > 0xFFFF)
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "handle");
        return -1;
    }

    if (peer_arg_payload(args, payload, sizeof(payload), &len) < 0)
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "payload");
        return -1;
    }

    return peer_gatt_server_notify(link, (uint16_t)handle, payload, len, reply);
}

static int peer_op_sec_bond(const peer_args_t *args, peer_reply_t *reply)
{
    const char *io = peer_arg_get(args, "io");
    const char *passkey_arg = peer_arg_get(args, "passkey");
    uint8_t has_passkey = passkey_arg != RT_NULL && passkey_arg[0] != '\0';
    int passkey = 0;
    peer_link_t *link = peer_require_link(args, reply);

    if (link == RT_NULL)
        return -1;

    if (has_passkey &&
            (peer_arg_int(args, "passkey", 0, &passkey) < 0 || passkey < 0))
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "passkey");
        return -1;
    }

    return peer_sec_bond(link, io, has_passkey, (uint32_t)passkey, reply);
}

static int peer_op_sec_clear(const peer_args_t *args, peer_reply_t *reply)
{
    (void)args;
    return peer_sec_clear(reply);
}

static int peer_op_periodic_adv_start(const peer_args_t *args, peer_reply_t *reply)
{
    static uint8_t payload[PEER_MAX_PAYLOAD];
    uint16_t len = 0;
    int interval_ms;

    if (peer_arg_int(args, "interval_ms", 100, &interval_ms) < 0)
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "periodic.adv.start");
        return -1;
    }

    if (peer_arg_payload(args, payload, sizeof(payload), &len) < 0)
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "payload");
        return -1;
    }

    return peer_periodic_adv_start((uint32_t)interval_ms, payload, len, reply);
}

static int peer_op_periodic_adv_stop(const peer_args_t *args, peer_reply_t *reply)
{
    (void)args;
    return peer_periodic_adv_stop(reply);
}

static int peer_op_periodic_sync(const peer_args_t *args, peer_reply_t *reply)
{
    const peer_adv_report_t *report = RT_NULL;
    const char *addr_hex = peer_arg_get(args, "addr");
    uint8_t addr[BD_ADDR_LEN];
    int index, addr_type;

    if (peer_arg_int(args, "index", 0, &index) < 0 ||
            peer_arg_int(args, "addr_type", 0, &addr_type) < 0)
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "periodic.sync");
        return -1;
    }

    if (index > 0)
    {
        /* Check the range before the cast: index=257 would wrap to 1 and
         * silently connect to the wrong report. */
        if (index > (int)peer_ble_adv_report_count())
        {
            peer_reply_err(reply, PEER_ERR_INVALID_ARG, "index");
            return -1;
        }
        report = peer_ble_adv_report((uint8_t)index);
        if (report == RT_NULL)
        {
            peer_reply_err(reply, PEER_ERR_INVALID_ARG, "index");
            return -1;
        }
    }
    else if (addr_hex != RT_NULL)
    {
        if (peer_hex_decode(addr_hex, addr, sizeof(addr)) != BD_ADDR_LEN)
        {
            peer_reply_err(reply, PEER_ERR_INVALID_ARG, "addr");
            return -1;
        }
    }
    else
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "index_or_addr");
        return -1;
    }

    return peer_periodic_sync(report, addr, (uint8_t)addr_type, reply);
}

static int peer_op_periodic_sync_stop(const peer_args_t *args, peer_reply_t *reply)
{
    (void)args;
    return peer_periodic_sync_stop(reply);
}

static int peer_op_gap_conn_params(const peer_args_t *args, peer_reply_t *reply)
{
    int link_id, intv_min, intv_max, latency, timeout_ms;

    if (peer_arg_int(args, "link", -1, &link_id) < 0 ||
            peer_arg_int(args, "interval_min_ms", 0, &intv_min) < 0 ||
            peer_arg_int(args, "interval_max_ms", 0, &intv_max) < 0 ||
            peer_arg_int(args, "latency", 0, &latency) < 0 ||
            peer_arg_int(args, "timeout_ms", 0, &timeout_ms) < 0)
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "gap.conn_params");
        return -1;
    }

    if (link_id <= 0 ||
            intv_min < 8 || intv_min > 4000 ||
            intv_max < intv_min || intv_max > 4000 ||
            latency < 0 || latency > 499 ||
            timeout_ms < 100 || timeout_ms > 32000)
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "gap.conn_params");
        return -1;
    }

    return peer_ble_update_conn_param((uint8_t)link_id, (uint16_t)intv_min,
                                      (uint16_t)intv_max, (uint16_t)latency,
                                      (uint16_t)timeout_ms, reply);
}

static int peer_op_gap_phy(const peer_args_t *args, peer_reply_t *reply)
{
    int link_id, phy;

    if (peer_arg_int(args, "link", -1, &link_id) < 0 ||
            peer_arg_int(args, "phy", 0, &phy) < 0)
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "gap.phy");
        return -1;
    }

    if (link_id <= 0 || phy < 1 || phy > 3)
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "gap.phy");
        return -1;
    }

    return peer_ble_update_phy((uint8_t)link_id, (uint8_t)phy, reply);
}

static int peer_op_gap_data_len(const peer_args_t *args, peer_reply_t *reply)
{
    int link_id, tx_octets, tx_time;

    if (peer_arg_int(args, "link", -1, &link_id) < 0 ||
            peer_arg_int(args, "tx_octets", 0, &tx_octets) < 0 ||
            peer_arg_int(args, "tx_time", 0, &tx_time) < 0)
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "gap.data_len");
        return -1;
    }

    if (link_id <= 0 || tx_octets < 27 || tx_octets > 251 ||
            tx_time < 0 || tx_time > 2120)
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "gap.data_len");
        return -1;
    }

    return peer_ble_update_data_len((uint8_t)link_id, (uint16_t)tx_octets,
                                    (uint16_t)tx_time, reply);
}

static int peer_op_adv_data(const peer_args_t *args, peer_reply_t *reply)
{
    uint8_t payload[PEER_ADV_DATA_MAX];
    uint16_t len = 0;

    if (peer_arg_payload(args, payload, sizeof(payload), &len) < 0)
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "payload");
        return -1;
    }

    return peer_ble_adv_update(payload, len, reply);
}

static int peer_dispatch(const char *id, const char *op, peer_args_t *args)
{
    peer_reply_t reply;
    int rc;

    memset(&reply, 0, sizeof(reply));

    if (strcmp(op, "state.reset") == 0)
        rc = peer_op_state_reset(args, &reply);
    else if (strcmp(op, "gap.scan") == 0)
        rc = peer_op_gap_scan(args, &reply);
    else if (strcmp(op, "gap.scan.stop") == 0)
        rc = peer_op_gap_scan_stop(args, &reply);
    else if (strcmp(op, "gap.connect") == 0)
        rc = peer_op_gap_connect(args, &reply);
    else if (strcmp(op, "gap.disconnect") == 0)
        rc = peer_op_gap_disconnect(args, &reply);
    else if (strcmp(op, "dev.list") == 0)
        rc = peer_op_dev_list(args, &reply);
    else if (strcmp(op, "gatt.discover") == 0)
        rc = peer_op_gatt_discover(args, &reply);
    else if (strcmp(op, "gatt.read") == 0)
        rc = peer_op_gatt_read(args, &reply);
    else if (strcmp(op, "gatt.write") == 0)
        rc = peer_op_gatt_write(args, &reply);
    else if (strcmp(op, "gatt.subscribe") == 0)
        rc = peer_op_gatt_subscribe(args, &reply);
    else if (strcmp(op, "adv.start") == 0)
        rc = peer_op_adv_start(args, &reply);
    else if (strcmp(op, "adv.stop") == 0)
        rc = peer_op_adv_stop(args, &reply);
    else if (strcmp(op, "svc.register") == 0)
        rc = peer_op_svc_register(args, &reply);
    else if (strcmp(op, "svc.unregister") == 0)
        rc = peer_op_svc_unregister(args, &reply);
    else if (strcmp(op, "svc.notify") == 0)
        rc = peer_op_svc_notify(args, &reply);
    else if (strcmp(op, "sec.bond") == 0)
        rc = peer_op_sec_bond(args, &reply);
    else if (strcmp(op, "sec.clear") == 0)
        rc = peer_op_sec_clear(args, &reply);
    else if (strcmp(op, "periodic.adv.start") == 0)
        rc = peer_op_periodic_adv_start(args, &reply);
    else if (strcmp(op, "periodic.adv.stop") == 0)
        rc = peer_op_periodic_adv_stop(args, &reply);
    else if (strcmp(op, "periodic.sync") == 0)
        rc = peer_op_periodic_sync(args, &reply);
    else if (strcmp(op, "periodic.sync.stop") == 0)
        rc = peer_op_periodic_sync_stop(args, &reply);
    else if (strcmp(op, "gap.conn_params") == 0)
        rc = peer_op_gap_conn_params(args, &reply);
    else if (strcmp(op, "gap.phy") == 0)
        rc = peer_op_gap_phy(args, &reply);
    else if (strcmp(op, "gap.data_len") == 0)
        rc = peer_op_gap_data_len(args, &reply);
    else if (strcmp(op, "gatt.mtu") == 0)
        rc = peer_op_gatt_mtu(args, &reply);
    else if (strcmp(op, "adv.data") == 0)
        rc = peer_op_adv_data(args, &reply);
    else
    {
        peer_reply_err(&reply, PEER_ERR_UNSUPPORTED, "%s", op);
        rc = -1;
    }

    peer_emit_terminal(id, &reply, rc);
    return rc;
}

int peer_cmd(int argc, char **argv)
{
    peer_args_t args;
    const char *id;
    int i;

    if (argc < 2)
    {
        rt_kprintf("usage: peer info | peer req <id> <op> [key=value ...]\n");
        return 0;
    }

    if (strcmp(argv[1], "info") == 0)
    {
        peer_emit_info();
        return 0;
    }

    if (strcmp(argv[1], "data") == 0)
    {
        id = argc >= 3 ? argv[2] : "0";
        peer_emit_err_raw(id, PEER_ERR_UNSUPPORTED, "peer.data");
        return 0;
    }

    if (strcmp(argv[1], "req") != 0)
    {
        rt_kprintf("usage: peer info | peer req <id> <op> [key=value ...]\n");
        return 0;
    }

    if (argc < 4)
    {
        peer_emit_err_raw(argc >= 3 ? argv[2] : "0", PEER_ERR_PROTO, "bad_request");
        return 0;
    }

    id = argv[2];
    memset(&args, 0, sizeof(args));

    for (i = 4; i < argc; i++)
    {
        char *eq = strchr(argv[i], '=');

        if (eq == RT_NULL || eq == argv[i])
        {
            peer_emit_err_raw(id, PEER_ERR_INVALID_ARG, "arg");
            return 0;
        }

        if (args.count >= PEER_MAX_ARGS)
        {
            peer_emit_err_raw(id, PEER_ERR_PROTO, "too_many_args");
            return 0;
        }

        *eq = '\0';
        args.items[args.count].key = argv[i];
        args.items[args.count].value = eq + 1;
        args.count++;
    }

    peer_dispatch(id, argv[3], &args);
    return 0;
}

MSH_CMD_EXPORT_ALIAS(peer_cmd, peer, bt_peer instrument protocol);
