/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * wifi_peer instrument firmware: the line protocol.
 *
 * One line in, at most one terminal line out, plus whatever events the
 * operation causes. Everything is text so a transcript can be read by a human
 * and replayed to the host parser unchanged.
 *
 * Why the console device is written directly instead of via rt_kprintf():
 * once ulog is initialised rt_kprintf() goes through ulog_vraw(), whose line
 * buffer is ULOG_LINE_BUF_SIZE (128 bytes). A longer line is split without a
 * newline, so the host would see half a protocol line glued to the next log
 * record. One rt_device_write() per line, under a mutex, keeps a line whole.
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <string.h>
#include <stdarg.h>

#include <wlan_mgnt.h>

#include "peer.h"

static rt_mutex_t g_out_lock;
static rt_uint32_t g_evt_seq;

void peer_proto_init(void)
{
    if (g_out_lock == RT_NULL)
        g_out_lock = rt_mutex_create("peer_out", RT_IPC_FLAG_FIFO);
}

static void peer_line_out(const char *line)
{
    rt_device_t console;
    rt_uint16_t old_flag;
    char buffer[PEER_LINE_MAX + 4];
    rt_size_t length;

    if (line == RT_NULL)
        return;

    length = rt_strlen(line);
    if (length + 3 > sizeof(buffer))
        length = sizeof(buffer) - 3;

    /* One write for the whole record: the leading newline that ends the shell
     * prompt, the line, and the terminating newline. Three separate writes
     * would leave two windows in which another thread's output can land
     * between them, which is all it takes to turn one protocol line into two
     * unparseable halves. */
    buffer[0] = '\n';
    rt_memcpy(buffer + 1, line, length);
    buffer[length + 1] = '\n';

    if (g_out_lock != RT_NULL)
        rt_mutex_take(g_out_lock, RT_WAITING_FOREVER);

    console = rt_console_get_device();
    if (console != RT_NULL)
    {
        old_flag = console->open_flag;
        /* STREAM expands "\n" to "\r\n", matching the shell's expectations. */
        console->open_flag |= RT_DEVICE_FLAG_STREAM;
        console->open_flag &= ~RT_DEVICE_FLAG_DMA_TX;
        rt_device_write(console, 0, buffer, length + 2);
        console->open_flag = old_flag;
    }

    if (g_out_lock != RT_NULL)
        rt_mutex_release(g_out_lock);
}

void peer_log(const char *fmt, ...)
{
    char line[PEER_LINE_MAX];
    va_list ap;

    if (fmt == RT_NULL)
        return;

    va_start(ap, fmt);
    rt_vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);

    peer_line_out(line);
}

static int peer_hex_digit(char c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    return -1;
}

int peer_hex_encode(const char *in, char *out, rt_size_t out_size)
{
    static const char digits[] = "0123456789abcdef";
    rt_size_t i;
    rt_size_t len;

    if (in == RT_NULL || out == RT_NULL)
        return -1;

    len = rt_strlen(in);
    if (out_size < len * 2 + 1)
        return -1;

    for (i = 0; i < len; i++)
    {
        rt_uint8_t value = (rt_uint8_t)in[i];

        out[i * 2] = digits[value >> 4];
        out[i * 2 + 1] = digits[value & 0x0f];
    }
    out[len * 2] = '\0';

    return (int)(len * 2);
}

int peer_hex_decode(const char *in, char *out, rt_size_t out_size)
{
    rt_size_t i;
    rt_size_t len;
    rt_size_t produced = 0;

    if (in == RT_NULL || out == RT_NULL || out_size == 0)
        return -1;

    len = rt_strlen(in);
    if ((len % 2) != 0 || (len / 2) + 1 > out_size)
        return -1;

    for (i = 0; i < len; i += 2)
    {
        int high = peer_hex_digit(in[i]);
        int low = peer_hex_digit(in[i + 1]);

        if (high < 0 || low < 0)
            return -1;

        /* Everything downstream treats the result as a C string; a NUL byte in
         * the middle would silently truncate an SSID. */
        if (((high << 4) | low) == 0)
            return -1;

        out[produced++] = (char)((high << 4) | low);
    }
    out[produced] = '\0';

    return 0;
}

int peer_arg_str(const peer_args_t *args, const char *hex_key,
                 const char *plain_key, char *out, rt_size_t out_size)
{
    const char *encoded;
    const char *plain;
    rt_size_t length;

    if (args == RT_NULL || out == RT_NULL || out_size == 0)
        return -1;

    out[0] = '\0';

    encoded = peer_arg_get(args, hex_key);
    if (encoded != RT_NULL)
        return peer_hex_decode(encoded, out, out_size);

    plain = peer_arg_get(args, plain_key);
    if (plain == RT_NULL)
        return 0;

    length = rt_strlen(plain);
    if (length + 1 > out_size)
        return -1;

    rt_memcpy(out, plain, length + 1);
    return 0;
}

static rt_uint32_t peer_evt_seq_next(void)
{
    rt_uint32_t seq;

    rt_enter_critical();
    seq = ++g_evt_seq;
    rt_exit_critical();

    return seq;
}

rt_uint32_t peer_evt_seq_current(void)
{
    rt_uint32_t seq;

    rt_enter_critical();
    seq = g_evt_seq;
    rt_exit_critical();

    return seq;
}

void peer_emit_evt(const char *name, const char *fmt, ...)
{
    char line[PEER_LINE_MAX];
    int n;
    rt_uint32_t seq = peer_evt_seq_next();

    n = rt_snprintf(line, sizeof(line), "@evt %u %s", (unsigned)seq, name);
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

    if (fmt != RT_NULL && fmt[0] != '\0')
    {
        va_list ap;
        va_start(ap, fmt);
        rt_vsnprintf(reply->text, sizeof(reply->text), fmt, ap);
        va_end(ap);
    }
    else
    {
        reply->text[0] = '\0';
    }
}

const char *peer_arg_get(const peer_args_t *args, const char *key)
{
    rt_uint8_t i;

    for (i = 0; i < args->count; i++)
    {
        if (rt_strcmp(args->items[i].key, key) == 0)
            return args->items[i].value;
    }

    return RT_NULL;
}

int peer_arg_int(const peer_args_t *args, const char *key, int def, int *out)
{
    const char *value = peer_arg_get(args, key);
    char *end = RT_NULL;
    long parsed;

    if (out == RT_NULL)
        return -1;

    if (value == RT_NULL)
    {
        *out = def;
        return 0;
    }

    parsed = strtol(value, &end, 0);
    if (end == value || *end != '\0')
        return -1;

    *out = (int)parsed;
    return 0;
}

/* Append "key=value" to an already-built @ok line. */
static void peer_reply_appendf(peer_reply_t *reply, const char *fmt, ...)
{
    size_t used = rt_strlen(reply->extra);
    va_list ap;

    if (used + 2 >= sizeof(reply->extra))
        return;

    reply->extra[used++] = ' ';
    va_start(ap, fmt);
    rt_vsnprintf(reply->extra + used, sizeof(reply->extra) - used, fmt, ap);
    va_end(ap);
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
                "@info proto=%d fw=%s caps=%s iface=%s max_line=%d",
                PEER_PROTO_VERSION, PEER_FW_ID, PEER_CAPS,
                RT_WLAN_DEVICE_AP_NAME, PEER_CMD_MAX);
    peer_line_out(line);
}

static int peer_op_state_reset(peer_reply_t *reply)
{
    peer_reply_t stop_reply;

    /* A reset means "as if this run had not touched the radio", so it takes
     * the AP down and forgets its credentials: the next case must not find the
     * previous case's SSID still on the air, or its key still in force.
     *
     * Stopping an AP that still has a station associated can block for the
     * framework's full RT_WLAN_START_AP_WAIT_MS, which is why the host gives
     * this op a much longer timeout than the others (see
     * ``ApInstrument.reset()``). */
    rt_memset(&stop_reply, 0, sizeof(stop_reply));

    if (rt_wlan_ap_is_active())
    {
        if (peer_ap_stop(&stop_reply) != 0)
        {
            *reply = stop_reply;
            return -1;
        }

        peer_ap_forget();
        peer_reply_ok(reply, "reset=1 active=0 count=1");
        return 0;
    }

    peer_ap_forget();
    peer_reply_ok(reply, "reset=1 active=0 count=0");
    return 0;
}

static int peer_dispatch(const char *id, const char *op, peer_args_t *args)
{
    peer_reply_t reply;
    rt_uint32_t first_seq;
    int rc;

    memset(&reply, 0, sizeof(reply));

    /* Taken before the op can emit anything: this is what makes its events
     * separable from an earlier operation's still in flight. */
    first_seq = peer_evt_seq_current();

    if (strcmp(op, "state.reset") == 0)
        rc = peer_op_state_reset(&reply);
    else if (strcmp(op, "ap.start") == 0)
        rc = peer_ap_start(args, &reply);
    else if (strcmp(op, "ap.stop") == 0)
        rc = peer_ap_stop(&reply);
    else if (strcmp(op, "ap.status") == 0)
        rc = peer_ap_status(&reply);
    else
    {
        peer_reply_err(&reply, PEER_ERR_UNSUPPORTED, "%s", op);
        rc = -1;
    }

    if (rc == 0)
        peer_reply_appendf(&reply, "first_seq=%u", (unsigned)first_seq);

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
        peer_log("usage: peer info | peer req <id> <op> [key=value ...]");
        return 0;
    }

    if (strcmp(argv[1], "info") == 0)
    {
        peer_emit_info();
        return 0;
    }

    if (strcmp(argv[1], "req") != 0)
    {
        peer_log("usage: peer info | peer req <id> <op> [key=value ...]");
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

MSH_CMD_EXPORT_ALIAS(peer_cmd, peer, wifi_peer instrument protocol);
