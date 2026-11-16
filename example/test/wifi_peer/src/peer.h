/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * wifi_peer instrument firmware: protocol constants and shared types.
 *
 * The wire format is the one bt_peer uses, so the same host layer
 * (sdk_test.instruments.console) drives both instruments:
 *
 *   peer info                          -> @info proto=1 fw=... caps=... max_line=...
 *   peer req <id> <op> [key=value ...] -> @ok <id> [key=value ...] | @err <id> <code> <text>
 *   asynchronous events                -> @evt <seq> <name> [key=value ...]
 *
 * Values are separated by spaces, so a string that may contain one (an SSID
 * with a space in it, say) cannot travel as `ssid=Lab AP`: the shell splits it
 * into two arguments and the host's parser reads only the first half. Those
 * fields travel as `ssid_hex=`/`key_hex=`, two hex digits per byte, which is
 * unambiguous and reversible. A reply carries `ssid_hex=` always and a plain
 * `ssid=` when the value is a single token, so a transcript stays readable
 * without making the parse depend on it.
 *
 * A line this image prints for a human must never start with '@': the host
 * parser treats that character as the start of a protocol line.
 */

#ifndef WIFI_PEER_H
#define WIFI_PEER_H

#include <rtthread.h>

/* The AP interface the wlan framework exposes. `rtconfig.h` carries the
 * configured name; the fallback keeps this header usable outside a build that
 * selects RT_USING_WIFI (a syntax-only check, for instance). */
#ifndef RT_WLAN_DEVICE_AP_NAME
#define RT_WLAN_DEVICE_AP_NAME  "wlan1"
#endif

#define PEER_PROTO_VERSION      1
#define PEER_FW_ID              "wifi_peer-1"

/*!< Capabilities the host may ask for with `fixture: wifi_peer:ap`. */
#define PEER_CAPS               "ap"

/*!< Longest protocol line, including the terminating NUL. */
#define PEER_LINE_MAX           512
/*!< The `@info` line reports this as max_line: the host refuses to build a
 *   request longer than it, so the firmware never has to truncate one.
 *
 *   Sized for the worst `ap.start` request: SSID and key travel hex-encoded
 *   (two characters per byte), so a 32-byte SSID and a 32-byte key already
 *   account for 128 characters before the op name and the remaining
 *   arguments. */
#define PEER_CMD_MAX            320
#define PEER_MAX_ARGS           8

#define PEER_ERR_UNSUPPORTED    "unsupported"
#define PEER_ERR_INVALID_ARG    "invalid_arg"
#define PEER_ERR_NOT_READY      "not_ready"
#define PEER_ERR_STATE          "state"
#define PEER_ERR_BUSY           "busy"
#define PEER_ERR_PROTO          "proto"
#define PEER_ERR_NO_MEM         "no_mem"
#define PEER_ERR_RADIO          "radio"

/*!< The SoftAP this image brings up by itself, the way the shipped example in
 *   example/rt_device/wifi/ap does. */
#define PEER_AP_SSID            "SiFli_AP"
#define PEER_AP_PASSWORD        "12345678"
/*!< TCP echo port, same as the shipped example. */
#define PEER_AP_ECHO_PORT       8888

typedef struct
{
    const char *key;
    const char *value;
} peer_arg_t;

typedef struct
{
    rt_uint8_t count;
    peer_arg_t items[PEER_MAX_ARGS];
} peer_args_t;

/** Filled by an op handler; the dispatcher emits the terminal line. */
typedef struct
{
    const char *code;           /*!< NULL on success, else the `@err` code. */
    char text[64];              /*!< `@err` detail, ignored on success. */
    char extra[192];            /*!< `@ok` key=value list. */
} peer_reply_t;

/* ---- peer_proto.c ------------------------------------------------------ */

void peer_proto_init(void);
void peer_emit_evt(const char *name, const char *fmt, ...);

/**
 * Print one log line for a human.
 *
 * Goes through the same mutex and the same single rt_device_write() as a
 * protocol line, so a log printed by this image can never be spliced into the
 * middle of an `@ok` or `@evt`. Logs printed by other modules (ulog, the Wi-Fi
 * driver) do not take this mutex; they can still appear between two lines, but
 * not inside one of ours.
 *
 * The text must not start with '@'.
 */
void peer_log(const char *fmt, ...);

/**
 * Hex-encode @p in into @p out (lower case, 2 chars per byte).
 *
 * @retval >= 0 the number of characters written, excluding the NUL.
 * @retval -1   the output buffer is too small.
 */
int peer_hex_encode(const char *in, char *out, rt_size_t out_size);

/**
 * Decode @p in into @p out.
 *
 * @retval 0  decoded; a byte value of 0 is rejected, because everything above
 *            treats the result as a C string.
 * @retval -1 odd length, a non-hex character, a NUL byte, or a short buffer.
 */
int peer_hex_decode(const char *in, char *out, rt_size_t out_size);

/**
 * Read a string argument that may have travelled hex-encoded.
 *
 * Prefers @p hex_key, falls back to @p plain_key, so a hand typed
 * `peer req 1 ap.start ssid=Lab` keeps working while the host sends
 * `ssid_hex=...`.
 *
 * @retval 0  read (or absent: @p out is empty); use peer_arg_get() on both
 *            keys to tell "absent" from "empty".
 * @retval -1 a malformed encoding or a value longer than @p out_size.
 */
int peer_arg_str(const peer_args_t *args, const char *hex_key,
                 const char *plain_key, char *out, rt_size_t out_size);

/**
 * The sequence number events were at a moment ago.
 *
 * An operation reports this in its `@ok` as `first_seq`, so the host can tell
 * the events it caused from ones still in flight from an earlier operation.
 */
rt_uint32_t peer_evt_seq_current(void);

void peer_reply_ok(peer_reply_t *reply, const char *fmt, ...);
void peer_reply_err(peer_reply_t *reply, const char *code, const char *fmt, ...);
const char *peer_arg_get(const peer_args_t *args, const char *key);
int peer_arg_int(const peer_args_t *args, const char *key, int def, int *out);

/* ---- peer_ap.c --------------------------------------------------------- */

void peer_ap_init(void);

/**
 * Bring the AP interface up with @p ssid and start a WPA2/PSK AP.
 *
 * Args: ssid / ssid_hex (required), key / key_hex (empty or absent means an
 * open network), ch (1..13, default 6), hidden (0/1), mac (default
 * 02:00:5e:00:00:01).
 *
 * `mac` is not cosmetic: the modules in the lab share one burned-in address,
 * so an AP that keeps it advertises the DUT's own address as its BSSID and the
 * DUT never reports the SSID. See PEER_AP_DEFAULT_MAC in peer_ap.c.
 *
 * Reports `count` (1 when an `ap_start` event follows) and `first_seq`. An AP
 * that already serves exactly this SSID, key, channel, hidden flag and BSSID
 * is reported as `already=1` with `count=0` and is left running.
 */
int peer_ap_start(const peer_args_t *args, peer_reply_t *reply);

/**
 * Stop the AP. Idempotent: stopping an inactive AP is a success with count=0.
 */
int peer_ap_stop(peer_reply_t *reply);

/**
 * Forget the parameters of the last successful start, without touching the
 * radio. The next `ap.start` then starts afresh instead of answering
 * `already=1`; `state.reset` calls this after it has stopped the AP.
 */
void peer_ap_forget(void);

/** active, ssid, ch, mac, sta_num, auto_start. */
int peer_ap_status(peer_reply_t *reply);

/**
 * Same as the `ap.start` op, for callers inside the image (autostart and the
 * `ap_start` MSH command) that have no protocol request to answer.
 *
 * @p key empty or NULL means an open network; @p mac NULL (or empty) keeps
 * PEER_AP_DEFAULT_MAC.
 * @retval 0 on success.
 */
int peer_ap_start_simple(const char *ssid, const char *key, const char *mac);

/**
 * Bring up PEER_AP_SSID / PEER_AP_PASSWORD at boot, so the board is a usable
 * SoftAP without any host driving it: a human can join it, ping it or open the
 * echo port, and the suites still own the AP through `ap.start` / `ap.stop`.
 */
int peer_ap_autostart(void);

/* ---- main.c ------------------------------------------------------------ */

/**
 * Make sure the TCP echo server is running.
 *
 * Idempotent, and deliberately independent of whether the AP ever came up: the
 * listening thread is created once and retries bind() until the AP interface
 * has an address. A boot where the AP failed is therefore recoverable -- a
 * later `ap_start` or `peer req ... ap.start` succeeds and the echo port is
 * there -- instead of leaving the promised service off for the whole run.
 */
void peer_echo_ensure(void);

#endif /* WIFI_PEER_H */
