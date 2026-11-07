/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __PEER_H__
#define __PEER_H__

#include <rtthread.h>
#include <stdint.h>

#include "bf0_ble_gap.h"

/** Protocol version reported by @info. */
#define PEER_PROTO_VERSION      1
/** Firmware identity reported by @info. */
#define PEER_FW_ID              "bt_peer-m2"

/**
 * Capabilities advertised by @info.
 *
 * One image can expose more than one capability on the same console stream;
 * the host declares the set it needs (`fixture: bt_peer:ble,classic`) and
 * requires every entry here. A build that adds the Classic stack overrides
 * this with "ble,classic" rather than changing the protocol.
 */
#ifndef PEER_CAPS
#define PEER_CAPS               "ble"
#endif

#define PEER_MAX_LINKS          8
#define PEER_MAX_ADV_REPORTS    64
/*
 * Sized so a full payload still fits one protocol line: "len=460 data=<920>"
 * plus the command/reply prefix is under PEER_LINE_MAX (1024) and under the
 * 1024-byte FINSH_CMD_SIZE. Larger values would be silently truncated by the
 * shell on the way in and by rt_snprintf on the way out.
 */
#define PEER_MAX_PAYLOAD        460
#define PEER_ADV_DATA_MAX       255
#define PEER_LINE_MAX           1024
#define PEER_MAX_ARGS           10

/** Result codes used in @err replies. */
#define PEER_ERR_UNSUPPORTED    "unsupported"
#define PEER_ERR_INVALID_ARG    "invalid_arg"
#define PEER_ERR_NOT_READY      "not_ready"
#define PEER_ERR_STATE          "state"
#define PEER_ERR_BUSY           "busy"
#define PEER_ERR_TIMEOUT        "timeout"
#define PEER_ERR_NO_SERVICE     "no_service"
#define PEER_ERR_WRITE_FAILED   "write_failed"
#define PEER_ERR_READ_FAILED    "read_failed"
#define PEER_ERR_NO_MEM         "no_mem"
#define PEER_ERR_BOND_FAILED    "bond_failed"
#define PEER_ERR_PROTO          "proto"

/** Event bits signalled from the BLE callback thread. */
#define PEER_BIT_CONNECTED      (1UL << 0)
#define PEER_BIT_CONN_FAIL      (1UL << 1)
#define PEER_BIT_SCAN_DONE      (1UL << 2)
#define PEER_BIT_SEARCH_DONE    (1UL << 3)
#define PEER_BIT_READ_DONE      (1UL << 4)
#define PEER_BIT_WRITE_DONE     (1UL << 5)
#define PEER_BIT_MTU_DONE       (1UL << 6)
#define PEER_BIT_NOTIFY_DONE    (1UL << 7)
#define PEER_BIT_PERIODIC_CREATED   (1UL << 8)
#define PEER_BIT_PERIODIC_SYNC      (1UL << 9)
#define PEER_BIT_PERIODIC_FAIL      (1UL << 10)
#define PEER_BIT_PERIODIC_STOPPED   (1UL << 11)
#define PEER_BIT_CONN_PARAM_DONE    (1UL << 12)
#define PEER_BIT_PHY_DONE           (1UL << 13)
#define PEER_BIT_DATA_LEN_DONE      (1UL << 14)
#define PEER_BIT_ADV_STARTED        (1UL << 15)
#define PEER_BIT_BOND_DONE          (1UL << 16)
#define PEER_BIT_PERIODIC_STARTED   (1UL << 17)
#define PEER_BIT_PERIODIC_STOPPED   (1UL << 18)

/** "No connection": the stack's conn_idx is uint8_t, so 0xFF is free. */
#define PEER_NO_CONN                0xFF

/** One BLE link as seen by the wire protocol. */
typedef struct
{
    uint8_t used;
    uint8_t link_id;            /*!< Stable 1-based id while connected. */
    uint8_t conn_idx;           /*!< Stack connection index. */
    uint8_t role;               /*!< 0 = central, 1 = peripheral. */
    uint8_t addr_type;
    bd_addr_t addr;
    uint16_t interval;          /*!< Connection interval in 1.25 ms units. */
    uint16_t mtu;
    uint16_t rmt_svc_hdl;       /*!< Handle from sibles_register_remote_svc(). */
    uint16_t svc_start;
    uint16_t svc_end;
    uint8_t gatt_busy;
} peer_link_t;

/** One advertising report captured by gap.scan. */
typedef struct
{
    uint8_t addr_type;
    uint8_t report_type;
    uint8_t conn_adv;           /*!< 1 when the report is connectable. */
    uint8_t adv_sid;            /*!< Advertising SID, needed to sync periodic adv. */
    int8_t rssi;
    bd_addr_t addr;
    uint16_t len;
    uint8_t data[PEER_ADV_DATA_MAX];
} peer_adv_report_t;

typedef struct
{
    const char *key;
    const char *value;
} peer_arg_t;

typedef struct
{
    uint8_t count;
    peer_arg_t items[PEER_MAX_ARGS];
} peer_args_t;

/** Filled by an op handler; the dispatcher emits the terminal line. */
typedef struct
{
    const char *code;           /*!< NULL on success, else @err code. */
    char text[96];
    char extra[PEER_MAX_PAYLOAD * 2 + 64];
} peer_reply_t;

/* ---- peer_proto.c ------------------------------------------------------ */
void peer_proto_init(void);
void peer_emit_evt(const char *name, const char *fmt, ...);
/**
 * The sequence number events were at a moment ago.
 *
 * An operation reports this in its `@ok` as `first_seq`, so the host can tell
 * the events it causes from ones still in flight from an earlier operation.
 */
uint32_t peer_evt_seq_current(void);
void peer_reply_ok(peer_reply_t *reply, const char *fmt, ...);
void peer_reply_err(peer_reply_t *reply, const char *code, const char *fmt, ...);
const char *peer_arg_get(const peer_args_t *args, const char *key);
int peer_arg_int(const peer_args_t *args, const char *key, int def, int *out);
int peer_arg_payload(const peer_args_t *args, uint8_t *out, size_t max, uint16_t *len);
int peer_hex_decode(const char *hex, uint8_t *out, size_t max);
int peer_format_payload(char *out, size_t out_size, const uint8_t *data, uint16_t len);

/* ---- peer_ble.c -------------------------------------------------------- */
void peer_ble_init(void);
uint8_t peer_ble_ready(void);
rt_err_t peer_ble_wait(uint32_t bits, int32_t timeout_ms);
void peer_ble_signal(uint32_t bits);
void peer_ble_clear_bits(uint32_t bits);
void peer_ble_addr_str(const bd_addr_t *addr, char *out, size_t out_size);
void peer_ble_local_addr_str(char *out, size_t out_size);
const char *peer_ble_local_name(void);
peer_link_t *peer_link_find(uint8_t link_id);
peer_link_t *peer_link_find_conn(uint8_t conn_idx);
peer_link_t *peer_link_at(uint8_t slot);
uint8_t peer_link_count(void);
const peer_adv_report_t *peer_ble_adv_report(uint8_t index);
uint8_t peer_ble_adv_report_count(void);

int peer_ble_scan_start(uint32_t duration_ms, uint32_t interval_ms,
                        uint32_t window_ms, uint8_t dup, int8_t rssi_min,
                        peer_reply_t *reply);
int peer_ble_scan_stop(peer_reply_t *reply);
int peer_ble_connect(const peer_adv_report_t *report, const uint8_t *addr,
                     uint8_t addr_type, uint32_t timeout_ms,
                     uint32_t interval_ms, uint8_t *link_id,
                     peer_reply_t *reply);
int peer_ble_disconnect(uint8_t link_id, peer_reply_t *reply);
void peer_ble_reset(void);
int peer_ble_adv_start(uint8_t extended, uint32_t interval_ms,
                       uint8_t connectable, const uint8_t *data,
                       uint16_t len, peer_reply_t *reply);
int peer_ble_adv_stop(peer_reply_t *reply);
int peer_ble_adv_update(const uint8_t *data, uint16_t len, peer_reply_t *reply);

int peer_ble_update_conn_param(uint8_t link_id, uint16_t intv_min_ms,
                               uint16_t intv_max_ms, uint16_t latency,
                               uint16_t timeout_ms, peer_reply_t *reply);
int peer_ble_update_phy(uint8_t link_id, uint8_t phy, peer_reply_t *reply);
int peer_ble_update_data_len(uint8_t link_id, uint16_t tx_octets,
                             uint16_t tx_time, peer_reply_t *reply);

/* ---- peer_gatt_client.c ------------------------------------------------ */
void peer_gatt_client_init(void);
void peer_gatt_client_reset(void);
int peer_gatt_client_discover(peer_link_t *link, const uint8_t *uuid,
                              uint8_t uuid_len, peer_reply_t *reply);
int peer_gatt_client_read(peer_link_t *link, uint16_t handle,
                          peer_reply_t *reply);
int peer_gatt_client_write(peer_link_t *link, uint16_t handle,
                           const uint8_t *data, uint16_t len,
                           uint8_t response, peer_reply_t *reply);
int peer_gatt_client_subscribe(peer_link_t *link, uint16_t cccd,
                               uint8_t enable, peer_reply_t *reply);
int peer_gatt_client_mtu(peer_link_t *link, peer_reply_t *reply);
void peer_gatt_client_on_search(void *rsp);
void peer_gatt_client_on_write_rsp(void *rsp);
void peer_gatt_client_on_read_error(void *ind);
void peer_gatt_client_on_disconnect(uint8_t conn_idx);
int peer_gatt_client_remote_cb(uint16_t event_id, uint8_t *data, uint16_t len);

/* ---- peer_gatt_server.c ------------------------------------------------ */
void peer_gatt_server_init(void);
void peer_gatt_server_reset(void);
void peer_gatt_server_on_disconnect(uint8_t conn_idx);
int peer_gatt_server_register(peer_reply_t *reply);
int peer_gatt_server_unregister(peer_reply_t *reply);
int peer_gatt_server_notify(peer_link_t *link, uint16_t handle,
                            const uint8_t *data, uint16_t len,
                            peer_reply_t *reply);

/* ---- peer_sec.c -------------------------------------------------------- */
void peer_sec_init(void);
int peer_sec_bond(peer_link_t *link, const char *io, const char *auth,
                  uint8_t has_passkey, uint32_t passkey, peer_reply_t *reply);
int peer_sec_clear(peer_reply_t *reply);

/* ---- peer_periodic.c --------------------------------------------------- */
void peer_periodic_init(void);
void peer_periodic_reset(void);
int peer_periodic_adv_start(uint32_t interval_ms, const uint8_t *data,
                            uint16_t len, peer_reply_t *reply);
int peer_periodic_adv_stop(peer_reply_t *reply);
int peer_periodic_sync(const peer_adv_report_t *report, const uint8_t *addr,
                       uint8_t addr_type, peer_reply_t *reply);
int peer_periodic_sync_stop(peer_reply_t *reply);

#endif /* __PEER_H__ */
