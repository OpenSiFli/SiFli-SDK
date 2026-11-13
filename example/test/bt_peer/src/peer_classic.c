/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Classic (BR/EDR) half of the bt_peer instrument: inquiry, SPP links and the
 * `btc.*` operations.
 *
 * Everything here is compiled only into a build that carries the Classic
 * controller; elsewhere this file is an empty translation unit and no Classic
 * symbol reaches the image.
 *
 * Threading: the stack delivers events on the Bluetooth task, the commands run
 * on the shell thread. They share the link table and the operation flags, and
 * every access to them goes through the small mutex below. Nothing that can
 * block (a stack call, a console write) is done while holding it.
 */

#include <rtthread.h>
#include <string.h>

#include "peer.h"
#include "peer_classic.h"

#ifdef PEER_CLASSIC

/* Pulls in the stack's protocol headers (sc_api.h and friends) that
 * bts2_app_interface.h's prototypes need. */
#include "bts2_app_inc.h"
#include "bts2_app_interface.h"
#include "bts2_bt.h"
#include "bt_connection_manager.h"
#include "bts2_mem.h"
/* bt_addr_convert_to_bts() is declared here under BT_FINSH. */
#include "bf0_sibles.h"
/*
 * After the stack headers, not before: bts2_dbg.h defines DBG_WARNING and
 * DBG_ERROR as no-op trace macros, the same names ulog_def.h maps to log
 * levels. ulog_def.h #undefs them before defining, so it has to come second.
 */
#include "ulog.h"

/** Classic links are tracked separately from BLE ones; four is plenty. */
#define PEER_CLC_MAX_LINKS          4
/** Event bits signalled from the Bluetooth task. */
#define PEER_CLC_BIT_READY          (1UL << 0)
#define PEER_CLC_BIT_LOCAL_ADDR     (1UL << 1)
#define PEER_CLC_BIT_LOCAL_NAME     (1UL << 2)
#define PEER_CLC_BIT_CONN           (1UL << 3)
#define PEER_CLC_BIT_CONN_FAIL      (1UL << 4)
#define PEER_CLC_BIT_DISC           (1UL << 5)
#define PEER_CLC_BIT_INQUIRY_CMP    (1UL << 6)
#define PEER_CLC_BIT_PAIRED         (1UL << 7)
#define PEER_CLC_BIT_SDP            (1UL << 8)
#define PEER_CLC_BIT_PAIRED_DB      (1UL << 9)
/** Allowance on top of the caller's budget for the stack to answer. */
#define PEER_CLC_WAIT_SLACK_MS      2000
/** Bound on waiting for the stack to tear links down in state.reset(). */
#define PEER_CLC_RESET_WAIT_MS      1000
#define PEER_CLC_RESET_POLL_MS      10
/** Bound on absorbing the completion of an inquiry that timed out. */
#define PEER_CLC_CMP_DRAIN_MS       1000
/** Bound on waiting for the security manager's paired-device list. */
#define PEER_CLC_PAIRED_DB_WAIT_MS  1000
/** Reported in btc.disc when the stack gave no disconnect reason. */
#define PEER_CLC_REASON_UNKNOWN     0

typedef struct
{
    uint8_t used;
    uint8_t link_id;            /*!< 1-based, unique among the live links. */
    uint8_t srv_chl;
    uint8_t role;               /*!< 0 = instrument paged out, 1 = peer paged in. */
    uint16_t mtu;
    uint8_t mac[6];             /*!< Same order as bt_notify_device_mac_t.addr. */
} peer_clc_link_t;

static rt_event_t g_clc_event;
static rt_mutex_t g_clc_lock;
/* Set once peer_classic_init() has brought the IPC objects up; see below. */
static rt_bool_t g_clc_init_ok;

static peer_clc_link_t g_clc_links[PEER_CLC_MAX_LINKS];
static uint8_t g_clc_next_link;

static uint8_t g_clc_ready;
static uint8_t g_clc_addr[6];
static uint8_t g_clc_addr_valid;
static char g_clc_name[BT_NOTIFY_MAX_FRIENDLY_NAME_LEN + 1];
static uint8_t g_clc_name_valid;

/*
 * One outstanding multi-step operation at a time. `connect_result` is 0 while
 * waiting, 1 when the matching SPP link came up and 2 when the stack refused
 * the ACL. The callback fills it, the command reads it.
 */
static uint8_t g_clc_connect_pending;
static uint8_t g_clc_connect_addr[6];
static uint8_t g_clc_connect_result;
static uint8_t g_clc_connect_link;
/*
 * Address of an attempt that already timed out. A connection completing for it
 * afterwards is dropped instead of being reported as an incoming link, so it
 * can never be mistaken for the result of the next btc.connect.
 */
static uint8_t g_clc_fail_valid;
static uint8_t g_clc_fail_addr[6];
/*
 * Set while this attempt's SPP connection request is in the stack. Until then
 * an ACL failure for the same address belongs to some other link (an incoming
 * one, or an earlier attempt) and must not decide this command's result.
 */
static uint8_t g_clc_acl_submitted;
/* Set when the ACL of this attempt came up, so a timeout knows whether the
 * stack is still paging and can be cancelled. */
static uint8_t g_clc_acl_up;

static uint8_t g_clc_inquiry_pending;
static uint16_t g_clc_inquiry_count;

/*
 * Pairing in progress. `pair_result` is 0 while waiting, 1 when the stack
 * reported success and 2 when it reported a failure.
 */
static uint8_t g_clc_pair_pending;
static uint8_t g_clc_pair_addr[6];
static uint8_t g_clc_pair_result;
/* Set while the pairing fallback waits for the security manager's device list. */
static uint8_t g_clc_pair_record_pending;

/*
 * SDP search that has to precede the SPP connection request: the client needs
 * the remote service channel, and asking for the connection without it fails
 * inside the stack. `sdp_result` is 0 while waiting, 1 on success, 2 on a
 * reported failure.
 */
static uint8_t g_clc_sdp_pending;
static uint8_t g_clc_sdp_addr[6];
static uint8_t g_clc_sdp_result;

/* Set while state.reset() tears the old state down; suppresses btc.* events. */
static uint8_t g_clc_quiet;

static const char s_clc_hex[] = "0123456789abcdef";

static void peer_clc_lock(void)
{
    if (g_clc_lock != RT_NULL)
        rt_mutex_take(g_clc_lock, RT_WAITING_FOREVER);
}

static void peer_clc_unlock(void)
{
    if (g_clc_lock != RT_NULL)
        rt_mutex_release(g_clc_lock);
}

/** Length of a non-terminated name field, capped at the field size. */
static size_t peer_clc_name_len(const char *name, size_t max)
{
    size_t len = 0;

    while (len < max && name[len] != '\0')
        len++;

    return len;
}

static void peer_clc_hex_encode(const uint8_t *data, size_t len, char *out)
{
    size_t i;

    for (i = 0; i < len; i++)
    {
        out[i * 2] = s_clc_hex[data[i] >> 4];
        out[i * 2 + 1] = s_clc_hex[data[i] & 0x0F];
    }
    out[len * 2] = '\0';
}

static void peer_clc_addr_str(const uint8_t *mac, char *out)
{
    peer_clc_hex_encode(mac, 6, out);
}

/**
 * The caller's budget plus the stack's slack, clamped to what a tick timeout
 * can represent. A negative value would mean "wait forever" there.
 */
static rt_int32_t peer_clc_wait_ms(uint32_t ms)
{
    const uint32_t max_ms = 0x7FFFFFFF - PEER_CLC_WAIT_SLACK_MS;

    return (rt_int32_t)(ms > max_ms ? max_ms + PEER_CLC_WAIT_SLACK_MS
                                    : ms + PEER_CLC_WAIT_SLACK_MS);
}

/* ---- link table (all *_locked helpers expect the mutex) ----------------- */

static int peer_clc_slot_by_mac_locked(const uint8_t *mac, uint8_t chl)
{
    int i;

    for (i = 0; i < PEER_CLC_MAX_LINKS; i++)
    {
        if (g_clc_links[i].used && g_clc_links[i].srv_chl == chl &&
                memcmp(g_clc_links[i].mac, mac, 6) == 0)
            return i;
    }

    return -1;
}

static int peer_clc_slot_by_id_locked(uint8_t link_id)
{
    int i;

    if (link_id == 0)
        return -1;

    for (i = 0; i < PEER_CLC_MAX_LINKS; i++)
    {
        if (g_clc_links[i].used && g_clc_links[i].link_id == link_id)
            return i;
    }

    return -1;
}

static uint8_t peer_clc_next_id_locked(void)
{
    uint8_t id;
    uint8_t taken;
    int i;

    do
    {
        g_clc_next_link = g_clc_next_link >= 0xFF ? 1 : g_clc_next_link + 1;
        id = g_clc_next_link;
        taken = 0;
        for (i = 0; i < PEER_CLC_MAX_LINKS; i++)
        {
            if (g_clc_links[i].used && g_clc_links[i].link_id == id)
            {
                taken = 1;
                break;
            }
        }
    } while (taken);

    return id;
}

/**
 * Track a link, or refresh the entry of a channel that was already up.
 *
 * @return 0 on success, -1 when the table is full
 */
static int peer_clc_link_new_locked(const uint8_t *mac, uint8_t chl,
                                    uint8_t role, uint16_t mtu, uint8_t *link_id)
{
    peer_clc_link_t *link;
    int slot;

    slot = peer_clc_slot_by_mac_locked(mac, chl);
    if (slot < 0)
    {
        for (slot = 0; slot < PEER_CLC_MAX_LINKS; slot++)
            if (!g_clc_links[slot].used)
                break;
        if (slot == PEER_CLC_MAX_LINKS)
            return -1;
    }

    link = &g_clc_links[slot];
    if (!link->used)
    {
        memset(link, 0, sizeof(*link));
        link->used = 1;
        link->link_id = peer_clc_next_id_locked();
    }

    memcpy(link->mac, mac, 6);
    link->srv_chl = chl;
    link->role = role;
    link->mtu = mtu;
    *link_id = link->link_id;

    return 0;
}

static uint8_t peer_clc_link_count_locked(void)
{
    uint8_t count = 0;
    int i;

    for (i = 0; i < PEER_CLC_MAX_LINKS; i++)
        if (g_clc_links[i].used)
            count++;

    return count;
}

/**
 * Forget one link and report it, unless events are suppressed.
 *
 * The slot is re-validated under the lock, so a caller that searched the table
 * earlier cannot drop an unrelated link that took the slot since.
 */
static void peer_clc_drop_slot(int slot, const uint8_t *mac,
                               uint8_t use_chl, uint8_t chl,
                               uint8_t reason)
{
    peer_clc_link_t link;
    uint8_t match;
    uint8_t quiet;

    peer_clc_lock();
    match = slot >= 0 && slot < PEER_CLC_MAX_LINKS && g_clc_links[slot].used &&
            memcmp(g_clc_links[slot].mac, mac, 6) == 0 &&
            (!use_chl || g_clc_links[slot].srv_chl == chl);
    if (match)
    {
        link = g_clc_links[slot];
        memset(&g_clc_links[slot], 0, sizeof(g_clc_links[slot]));
    }
    quiet = g_clc_quiet;
    peer_clc_unlock();

    if (!match)
        return;

    if (!quiet)
        peer_emit_evt("btc.disc", "link=%u reason=%u", link.link_id, reason);

    if (g_clc_event != RT_NULL)
        rt_event_send(g_clc_event, PEER_CLC_BIT_DISC);
}

/** Drop every link matching @p mac, or only its @p chl when @p use_chl is set. */
static void peer_clc_drop_matching(const uint8_t *mac, uint8_t use_chl,
                                   uint8_t chl, uint8_t reason)
{
    int slot;

    for (slot = 0; slot < PEER_CLC_MAX_LINKS; slot++)
        peer_clc_drop_slot(slot, mac, use_chl, chl, reason);
}

/* ---- event handlers (Bluetooth task) ------------------------------------ */

/*
 * The Bluetooth task runs on a 3 KB stack (BTS_TASK_SIZE), so the buffers that
 * format an event live in .bss rather than on it. Only this task runs the
 * handlers below; the ones shared with the shell thread are covered by
 * g_clc_lock, and the two below are not touched from anywhere else.
 */
static char s_clc_dev_addr[13];
static char s_clc_dev_name[BT_NOTIFY_MAX_FRIENDLY_NAME_LEN * 2 + 1];

static void peer_clc_on_discover(const bt_notify_remote_device_info_t *info)
{
    size_t name_len;
    uint8_t pending;

    peer_clc_lock();
    pending = g_clc_inquiry_pending;
    if (pending)
        g_clc_inquiry_count++;
    peer_clc_unlock();

    if (!pending)
        return;

    /* The stack fills the whole 61-byte field, so trust the terminator only
     * after bounding it; an over-long name is truncated, never over-read. */
    name_len = peer_clc_name_len(info->bt_name, BT_NOTIFY_MAX_FRIENDLY_NAME_LEN);
    peer_clc_hex_encode((const uint8_t *)info->bt_name, name_len, s_clc_dev_name);
    peer_clc_addr_str(info->mac.addr, s_clc_dev_addr);

    peer_emit_evt("btc.dev", "addr=%s name=%s rssi=%d", s_clc_dev_addr,
                  s_clc_dev_name, info->rssi);
}

static void peer_clc_on_acl_connected(const bt_notify_device_acl_conn_info_t *info)
{
    uint8_t expected;
    uint8_t failed;

    /*
     * Only an ACL this attempt actually asked for can decide its result. A
     * failure reported while we are still searching SDP belongs to an
     * incoming link or an earlier attempt, and charging it to this command
     * would turn a working request into `@err state connect`.
     */
    peer_clc_lock();
    expected = g_clc_acl_submitted && g_clc_connect_pending &&
               memcmp(g_clc_connect_addr, info->mac.addr, 6) == 0;
    failed = expected && info->res != 0;
    if (expected)
    {
        if (info->res == 0)
            g_clc_acl_up = 1;
        else
            g_clc_connect_result = 2;
    }
    peer_clc_unlock();

    if (failed && g_clc_event != RT_NULL)
        rt_event_send(g_clc_event, PEER_CLC_BIT_CONN_FAIL);
}

static void peer_clc_on_acl_disconnected(const bt_notify_device_base_info_t *info)
{
    peer_clc_drop_matching(info->mac.addr, 0, 0, info->res);
}

static void peer_clc_on_pair(const bt_notify_device_base_info_t *info)
{
    uint8_t expected;

    peer_clc_lock();
    expected = g_clc_pair_pending &&
               memcmp(g_clc_pair_addr, info->mac.addr, 6) == 0;
    if (expected)
        g_clc_pair_result = info->res == 0 ? 1 : 2;
    peer_clc_unlock();

    if (expected && g_clc_event != RT_NULL)
        rt_event_send(g_clc_event, PEER_CLC_BIT_PAIRED);
}

static void peer_clc_on_encryption(const uint8_t *addr)
{
    uint8_t expected;

    /* The link is encrypted, which for our pairing means the exchange
     * succeeded even though the stack only reports a CFM for it. */
    peer_clc_lock();
    expected = g_clc_pair_pending &&
               memcmp(g_clc_pair_addr, addr, 6) == 0;
    if (expected)
        g_clc_pair_result = 1;
    peer_clc_unlock();

    if (expected && g_clc_event != RT_NULL)
        rt_event_send(g_clc_event, PEER_CLC_BIT_PAIRED);
}

static void peer_clc_on_paired_record(const bt_notify_paired_dev_record_t *rec)
{
    uint8_t expected;
    uint32_t i;

    peer_clc_lock();
    expected = g_clc_pair_record_pending && g_clc_pair_pending;
    if (expected)
    {
        for (i = 0; i < rec->count && i < BT_NOTIFY_MAX_PAIRED_DEV; i++)
        {
            if (memcmp(rec->mac[i].addr, g_clc_pair_addr, 6) == 0)
            {
                /*
                 * The list is persistent, so a match may be an older bond; a
                 * failure this attempt already reported outranks it.
                 */
                if (g_clc_pair_result != 2)
                    g_clc_pair_result = 1;
                break;
            }
        }
    }
    peer_clc_unlock();

    /* Wake the caller on any matching record, found or not: an empty or
     * unrelated list is still the answer it is waiting for. */
    if (expected && g_clc_event != RT_NULL)
        rt_event_send(g_clc_event, PEER_CLC_BIT_PAIRED_DB);
}

static void peer_clc_on_sdp(const bt_notify_spp_sdp_cfm_t *cfm)
{
    uint8_t expected;

    peer_clc_lock();
    expected = g_clc_sdp_pending &&
               memcmp(g_clc_sdp_addr, cfm->mac.addr, 6) == 0;
    if (expected)
        g_clc_sdp_result = cfm->res == 0 ? 1 : 2;
    peer_clc_unlock();

    if (expected && g_clc_event != RT_NULL)
        rt_event_send(g_clc_event, PEER_CLC_BIT_SDP);
}

static void peer_clc_on_spp_conn(const bt_notify_spp_conn_ind_t *ind)
{
    char addr[13];
    uint8_t pending;
    uint8_t late;
    uint8_t quiet;
    uint8_t link_id = 0;
    int rc;

    /*
     * Decide and record in one locked step: the command may be timing out at
     * this very moment, and whether this connection answers it or belongs to
     * an abandoned attempt must not change after the decision.
     */
    peer_clc_lock();
    pending = g_clc_connect_pending &&
              memcmp(g_clc_connect_addr, ind->mac.addr, 6) == 0;
    late = !pending && g_clc_fail_valid &&
           memcmp(g_clc_fail_addr, ind->mac.addr, 6) == 0;
    if (late)
        g_clc_fail_valid = 0;

    rc = -1;
    if (!late)
    {
        rc = peer_clc_link_new_locked(ind->mac.addr, ind->srv_chl,
                                      pending ? 0 : 1, ind->mfs, &link_id);
        if (rc == 0 && pending)
        {
            g_clc_connect_pending = 0;
            g_clc_connect_result = 1;
            g_clc_connect_link = link_id;
            /* This attempt succeeded, so the address is no longer one whose
             * late completion has to be isolated. */
            g_clc_fail_valid = 0;
        }
    }
    quiet = g_clc_quiet;
    peer_clc_unlock();

    if (late || rc < 0)
    {
        /* An answer to a request that already timed out, or a link that would
         * not fit in the table: close it instead of tracking it invisibly. */
        bt_notify_device_mac_t mac;

        memcpy(mac.addr, ind->mac.addr, 6);
        bt_interface_dis_spp_by_addr_and_chl_ext(&mac, ind->srv_chl);
        return;
    }

    if (!quiet)
    {
        peer_clc_addr_str(ind->mac.addr, addr);
        peer_emit_evt("btc.conn", "link=%u addr=%s channel=%u role=%u",
                      link_id, addr, ind->srv_chl, pending ? 0 : 1);
    }

    if (pending && g_clc_event != RT_NULL)
        rt_event_send(g_clc_event, PEER_CLC_BIT_CONN);
}

static void peer_clc_on_spp_disc(const bt_notify_spp_disc_ind_t *ind)
{
    peer_clc_drop_matching(ind->mac.addr, 1, ind->srv_chl,
                           PEER_CLC_REASON_UNKNOWN);
}

/*
 * Formatting a 460-byte payload takes a 984-byte buffer, and peer_emit_evt()
 * adds a 1 KB line: neither fits comfortably on the Bluetooth task's 3 KB
 * stack. This buffer is therefore static. It needs no lock of its own: only
 * peer_clc_on_spp_data() uses it, and event callbacks run one at a time on the
 * Bluetooth task. The lock is still taken for the table lookup, and released
 * before the console write, matching the rule stated at the top of the file.
 */
static char s_clc_data_line[PEER_MAX_PAYLOAD * 2 + 64];

static void peer_clc_on_spp_data(const bt_notify_spp_data_ind_t *ind)
{
    uint8_t quiet;
    uint8_t link_id = 0;
    uint16_t off;
    int slot;

    /* Flow control is not used: answer "no" while the payload is still the
     * stack's, which is exactly the lifetime of this callback. */
    if (ind->is_flow_ctrl != RT_NULL)
        *ind->is_flow_ctrl = 0;

    peer_clc_lock();
    slot = peer_clc_slot_by_mac_locked(ind->mac.addr, ind->srv_chl);
    if (slot >= 0)
        link_id = g_clc_links[slot].link_id;
    quiet = g_clc_quiet;
    peer_clc_unlock();

    if (slot < 0 || quiet)
        return;

    /* One protocol line holds PEER_MAX_PAYLOAD bytes; a longer SPP packet is
     * split instead of truncated. */
    for (off = 0; off < ind->payload_len;)
    {
        uint16_t chunk = ind->payload_len - off;

        if (chunk > PEER_MAX_PAYLOAD)
            chunk = PEER_MAX_PAYLOAD;

        if (peer_format_payload(s_clc_data_line, sizeof(s_clc_data_line),
                                ind->payload + off, chunk) >= 0)
            peer_emit_evt("btc.data", "link=%u %s off=%u", link_id,
                          s_clc_data_line, off);

        off += chunk;
    }
}

static void peer_clc_on_common(uint16_t event_id, uint8_t *data, uint16_t len)
{
    switch (event_id)
    {
    case BT_NOTIFY_COMMON_BT_STACK_READY:
        peer_clc_lock();
        g_clc_ready = 1;
        peer_clc_unlock();
        if (g_clc_event != RT_NULL)
            rt_event_send(g_clc_event, PEER_CLC_BIT_READY);
        break;

    case BT_NOTIFY_COMMON_LOCAL_ADDR_RSP:
        if (data == RT_NULL || len < 6)
            break;
        peer_clc_lock();
        memcpy(g_clc_addr, data, 6);
        g_clc_addr_valid = 1;
        peer_clc_unlock();
        if (g_clc_event != RT_NULL)
            rt_event_send(g_clc_event, PEER_CLC_BIT_LOCAL_ADDR);
        break;

    case BT_NOTIFY_COMMON_LOCAL_NAME_RSP:
        if (data != RT_NULL)
        {
            size_t max = len < sizeof(g_clc_name) ? len : sizeof(g_clc_name) - 1;
            size_t n = peer_clc_name_len((const char *)data, max);

            peer_clc_lock();
            memcpy(g_clc_name, data, n);
            g_clc_name[n] = '\0';
            g_clc_name_valid = 1;
            peer_clc_unlock();
            if (g_clc_event != RT_NULL)
                rt_event_send(g_clc_event, PEER_CLC_BIT_LOCAL_NAME);
        }
        break;

    case BT_NOTIFY_COMMON_DISCOVER_IND:
        if (data != RT_NULL && len >= sizeof(bt_notify_remote_device_info_t))
            peer_clc_on_discover((const bt_notify_remote_device_info_t *)data);
        break;

    case BT_NOTIFY_COMMON_INQUIRY_CMP:
        peer_clc_lock();
        g_clc_inquiry_pending = 0;
        peer_clc_unlock();
        if (g_clc_event != RT_NULL)
            rt_event_send(g_clc_event, PEER_CLC_BIT_INQUIRY_CMP);
        break;

    case BT_NOTIFY_COMMON_ACL_CONNECTED:
        if (data != RT_NULL && len >= sizeof(bt_notify_device_acl_conn_info_t))
            peer_clc_on_acl_connected((const bt_notify_device_acl_conn_info_t *)data);
        break;

    case BT_NOTIFY_COMMON_ACL_DISCONNECTED:
        if (data != RT_NULL && len >= sizeof(bt_notify_device_base_info_t))
            peer_clc_on_acl_disconnected((const bt_notify_device_base_info_t *)data);
        break;

    case BT_NOTIFY_COMMON_PAIR_IND:
    case BT_NOTIFY_COMMON_PAIR_CFM:
        /* IND is the passive side's result, CFM the locally initiated one;
         * both carry the same mac/res structure. */
        if (data != RT_NULL && len >= sizeof(bt_notify_device_base_info_t))
            peer_clc_on_pair((const bt_notify_device_base_info_t *)data);
        break;

    case BT_NOTIFY_COMMON_ENCRYPTION:
        /* This event carries the bare 6-byte address, not a structure. */
        if (data != RT_NULL && len >= 6)
            peer_clc_on_encryption(data);
        break;

    case BT_NOTIFY_COMMON_PAIRED_DEV_RECORD:
        if (data != RT_NULL && len >= sizeof(bt_notify_paired_dev_record_t))
            peer_clc_on_paired_record((const bt_notify_paired_dev_record_t *)data);
        break;

    default:
        break;
    }
}

static void peer_clc_on_spp(uint16_t event_id, uint8_t *data, uint16_t len)
{
    switch (event_id)
    {
    case BT_NOTIFY_SPP_CONN_IND:
        if (data != RT_NULL && len >= sizeof(bt_notify_spp_conn_ind_t))
            peer_clc_on_spp_conn((const bt_notify_spp_conn_ind_t *)data);
        break;

    case BT_NOTIFY_SPP_DISC_IND:
        if (data != RT_NULL && len >= sizeof(bt_notify_spp_disc_ind_t))
            peer_clc_on_spp_disc((const bt_notify_spp_disc_ind_t *)data);
        break;

    case BT_NOTIFY_SPP_DATA_IND:
        /* The stack reports sizeof(pointer) here, not the payload size, so the
         * length neither validates the struct nor bounds the payload. */
        if (data != RT_NULL)
            peer_clc_on_spp_data((const bt_notify_spp_data_ind_t *)data);
        break;

    case BT_NOTIFY_SPP_SDP_CFM:
        if (data != RT_NULL && len >= sizeof(bt_notify_spp_sdp_cfm_t))
            peer_clc_on_sdp((const bt_notify_spp_sdp_cfm_t *)data);
        break;

    default:
        break;
    }
}

static int peer_clc_event_cb(uint16_t type, uint16_t event_id, uint8_t *data,
                             uint16_t data_len)
{
    if (type == BT_NOTIFY_COMMON)
        peer_clc_on_common(event_id, data, data_len);
    else if (type == BT_NOTIFY_SPP)
        peer_clc_on_spp(event_id, data, data_len);

    return 0;
}

/* ---- commands (shell thread) -------------------------------------------- */

void peer_classic_init(void)
{
    bt_interface_status_t status;

    g_clc_init_ok = RT_FALSE;

    if (g_clc_lock == RT_NULL)
        g_clc_lock = rt_mutex_create("peer_clc", RT_IPC_FLAG_FIFO);
    if (g_clc_event == RT_NULL)
        g_clc_event = rt_event_create("peer_clc", RT_IPC_FLAG_FIFO);

    /*
     * Without both objects the callbacks cannot record state and the commands
     * cannot wait for it. Registering anyway would advertise Classic support
     * (`btc=1`) and then answer every operation with a timeout, so stop here
     * and let peer_classic_available() keep the capability undeclared.
     */
    if (g_clc_lock == RT_NULL || g_clc_event == RT_NULL)
    {
        LOG_E("bt_peer: Classic IPC objects not created (lock=%d event=%d)",
              g_clc_lock != RT_NULL, g_clc_event != RT_NULL);
        return;
    }

    status = bt_interface_register_bt_event_notify_callback(peer_clc_event_cb);
    if (status != BT_INTERFACE_STATUS_OK &&
            status != BT_INTERFACE_STATUS_ALREADY_EXIST)
    {
        /* Without the callback no state is ever recorded, so the operations
         * would only time out; treat it like a failed IPC creation. */
        LOG_E("bt_peer: Classic event callback not registered (%d)", status);
        return;
    }

#ifdef BSP_BT_CONNECTION_MANAGER
    /* Take the pageable (server) role: the peer may be the one connecting. */
    bt_cm_set_profile_target(0, BT_SLAVE_ROLE, 0);
#endif

    g_clc_init_ok = RT_TRUE;
}

rt_bool_t peer_classic_available(void)
{
    return g_clc_init_ok;
}

int peer_classic_local(uint32_t timeout_ms, peer_reply_t *reply)
{
    rt_tick_t deadline;
    uint8_t ready = 0;
    uint8_t addr_valid = 0;
    uint8_t name_valid = 0;
    uint8_t addr[6];
    char name[BT_NOTIFY_MAX_FRIENDLY_NAME_LEN + 1];
    char addr_str[13];
    char name_hex[BT_NOTIFY_MAX_FRIENDLY_NAME_LEN * 2 + 1];

    if (g_clc_event == RT_NULL)
    {
        peer_reply_err(reply, PEER_ERR_NOT_READY, "classic");
        return -1;
    }

    deadline = rt_tick_get() + rt_tick_from_millisecond((rt_int32_t)timeout_ms);

    for (;;)
    {
        rt_int32_t remain;

        peer_clc_lock();
        ready = g_clc_ready;
        addr_valid = g_clc_addr_valid;
        name_valid = g_clc_name_valid;
        memcpy(addr, g_clc_addr, sizeof(addr));
        memcpy(name, g_clc_name, sizeof(name));
        peer_clc_unlock();

        if (ready && addr_valid && name_valid)
            break;

        remain = (rt_int32_t)(deadline - rt_tick_get());
        if (remain <= 0)
            break;

        rt_event_recv(g_clc_event,
                      PEER_CLC_BIT_READY | PEER_CLC_BIT_LOCAL_ADDR |
                      PEER_CLC_BIT_LOCAL_NAME,
                      RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                      (rt_tick_t)remain, RT_NULL);
    }

    if (!ready)
    {
        peer_reply_err(reply, PEER_ERR_NOT_READY, "stack");
        return -1;
    }

    if (!addr_valid)
    {
        peer_reply_err(reply, PEER_ERR_TIMEOUT, "addr");
        return -1;
    }

    /* An empty name is a name, but a missing event is not: report it instead
     * of sending `name=` and letting the caller read it as an empty name. */
    if (!name_valid)
    {
        peer_reply_err(reply, PEER_ERR_TIMEOUT, "name");
        return -1;
    }

    peer_clc_addr_str(addr, addr_str);
    peer_clc_hex_encode((const uint8_t *)name,
                        peer_clc_name_len(name, sizeof(name) - 1), name_hex);
    peer_reply_ok(reply, "addr=%s name=%s role=server", addr_str, name_hex);
    return 0;
}

int peer_classic_inquiry(uint32_t duration_ms, uint8_t max_rsp,
                         peer_reply_t *reply)
{
    BTS2S_CPL_FILTER filter;
    uint32_t first_seq = peer_evt_seq_current();
    /* The stack takes seconds; round up so a sub-second request still runs. */
    uint32_t seconds = duration_ms / 1000 + (duration_ms % 1000 != 0 ? 1 : 0);
    uint32_t count;
    rt_err_t rc;

    if (g_clc_event == RT_NULL)
    {
        peer_reply_err(reply, PEER_ERR_NOT_READY, "classic");
        return -1;
    }

    /* max_timeout is in seconds and 0 means "no timeout", which is not what a
     * bounded request asked for. */
    if (seconds == 0)
        seconds = 1;
    if (seconds > 0xFFFF)
        seconds = 0xFFFF;

    peer_clc_lock();
    if (!g_clc_ready)
    {
        peer_clc_unlock();
        peer_reply_err(reply, PEER_ERR_NOT_READY, "classic");
        return -1;
    }
    if (g_clc_inquiry_pending || g_clc_connect_pending ||
            g_clc_pair_pending)
    {
        peer_clc_unlock();
        peer_reply_err(reply, PEER_ERR_BUSY, "inquiry");
        return -1;
    }
    g_clc_inquiry_pending = 1;
    g_clc_inquiry_count = 0;
    peer_clc_unlock();

    filter.filter = BTS2_INQ_FILTER_CLEAR;
    filter.dev_mask_cls = 0;    /* 0 selects every device class */

    /* Do not let a completion from an earlier inquiry satisfy this one. */
    rt_event_recv(g_clc_event, PEER_CLC_BIT_INQUIRY_CMP,
                  RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 0, RT_NULL);

    /*
     * gap_discov_req() directly, not bt_interface_start_inquiry_ex(): the
     * latter goes through bt_start_inquiry(), which now ignores dev_mask_cls
     * and always filters for audio devices. The instrument has to hear any
     * peer, including SPP-only ones. The result and completion events are
     * raised by the finsh layer for every caller, so nothing else depends on
     * bt_start_inquiry()'s own bookkeeping.
     */
    gap_discov_req(bts2_task_get_app_task_id(), max_rsp, (U16)seconds,
                   &filter, TRUE);

    rc = rt_event_recv(g_clc_event, PEER_CLC_BIT_INQUIRY_CMP,
                       RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                       rt_tick_from_millisecond(peer_clc_wait_ms(duration_ms)),
                       RT_NULL);
    if (rc != RT_EOK)
    {
        /*
         * Stop the inquiry. bt_interface_stop_inquiry() goes through
         * ESC_DISCOV, which the stack does not report as INQUIRY_CMP -- only a
         * natural completion does -- so nothing is owed here. The drain below
         * covers just the narrow case where the inquiry completed naturally
         * right before the timeout and its CMP was still in flight.
         *
         * A natural CMP that arrives more than PEER_CLC_CMP_DRAIN_MS after the
         * timeout is a residual window: the stack interface carries no
         * generation number, so it cannot be told from the next inquiry's
         * completion. The next btc.inquiry pre-clears the event bit, which
         * covers the part of that window before it starts; a CMP later than
         * that is accepted as a limitation of the interface.
         */
        rt_err_t drained;

        bt_interface_stop_inquiry();
        drained = rt_event_recv(g_clc_event, PEER_CLC_BIT_INQUIRY_CMP,
                                RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                                rt_tick_from_millisecond(PEER_CLC_CMP_DRAIN_MS),
                                RT_NULL);

        peer_clc_lock();
        g_clc_inquiry_pending = 0;
        peer_clc_unlock();

        /* A completion consumed just as the drain expired left its event bit
         * behind; take it so it cannot satisfy a later inquiry. */
        if (drained != RT_EOK)
            rt_event_recv(g_clc_event, PEER_CLC_BIT_INQUIRY_CMP,
                          RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 0, RT_NULL);

        peer_reply_err(reply, PEER_ERR_TIMEOUT, "inquiry");
        return -1;
    }

    peer_clc_lock();
    count = g_clc_inquiry_count;
    peer_clc_unlock();

    peer_reply_ok(reply, "count=%u first_seq=%u", count, (unsigned)first_seq);
    return 0;
}

/**
 * Remember @p addr as an attempt that ended without a link, so a connection
 * completing for it later is dropped instead of being reported as incoming.
 * Caller holds the mutex.
 */
static void peer_clc_fail_arm_locked(const uint8_t *addr)
{
    memcpy(g_clc_fail_addr, addr, 6);
    g_clc_fail_valid = 1;
}

int peer_classic_connect(const uint8_t *addr, const uint8_t *uuid,
                         uint8_t uuid_len, uint32_t timeout_ms,
                         peer_reply_t *reply)
{
    bt_notify_device_mac_t mac;
    char addr_str[13];
    uint8_t link_id = 0;
    uint8_t result = 0;
    uint8_t channel = 0;
    uint8_t sdp_result;
    uint8_t cancel;
    uint8_t link_up;
    int8_t cancel_rc;
    rt_tick_t deadline;
    rt_int32_t remain;
    rt_err_t rc;

    if (g_clc_event == RT_NULL)
    {
        peer_reply_err(reply, PEER_ERR_NOT_READY, "classic");
        return -1;
    }

    memset(&mac, 0, sizeof(mac));
    memcpy(mac.addr, addr, 6);
    peer_clc_addr_str(addr, addr_str);

    peer_clc_lock();
    if (!g_clc_ready)
    {
        peer_clc_unlock();
        peer_reply_err(reply, PEER_ERR_NOT_READY, "classic");
        return -1;
    }
    if (g_clc_connect_pending || g_clc_inquiry_pending ||
            g_clc_pair_pending)
    {
        peer_clc_unlock();
        peer_reply_err(reply, PEER_ERR_BUSY, "connect");
        return -1;
    }
    g_clc_connect_pending = 1;
    g_clc_connect_result = 0;
    g_clc_connect_link = 0;
    g_clc_acl_submitted = 0;
    g_clc_acl_up = 0;
    memcpy(g_clc_connect_addr, addr, 6);
    /*
     * g_clc_fail_valid is deliberately not cleared here: a connection
     * completing for an attempt that timed out before this one started is
     * still not this one's answer. It is cleared only when that completion is
     * dropped, by state.reset(), and when a link for the same address does
     * come up.
     */
    g_clc_sdp_pending = 1;
    g_clc_sdp_result = 0;
    memcpy(g_clc_sdp_addr, addr, 6);
    peer_clc_unlock();

    rt_event_recv(g_clc_event, PEER_CLC_BIT_CONN | PEER_CLC_BIT_CONN_FAIL,
                  RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 0, RT_NULL);
    rt_event_recv(g_clc_event, PEER_CLC_BIT_SDP,
                  RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 0, RT_NULL);

    /* SDP and the SPP request share one budget: the caller's timeout plus the
     * stack allowance, so a slow search leaves less for the connection. */
    deadline = rt_tick_get() +
               rt_tick_from_millisecond(peer_clc_wait_ms(timeout_ms));

    /*
     * The client needs the remote service channel before it can ask for the
     * connection; without the search the stack answers "can't get an inst".
     */
    if (bt_interface_spp_client_sdp_search_req(&mac, (uint8_t *)uuid,
            uuid_len) != BT_EOK)
    {
        peer_clc_lock();
        g_clc_sdp_pending = 0;
        g_clc_connect_pending = 0;
        peer_clc_unlock();
        peer_reply_err(reply, PEER_ERR_STATE, "connect");
        return -1;
    }

    remain = (rt_int32_t)(deadline - rt_tick_get());
    if (remain > 0)
        rc = rt_event_recv(g_clc_event, PEER_CLC_BIT_SDP,
                           RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                           (rt_tick_t)remain, RT_NULL);
    else
        rc = -RT_ETIMEOUT;

    peer_clc_lock();
    /* The search may have finished just as the wait expired. */
    if (rc != RT_EOK && g_clc_sdp_result != 0)
        rc = RT_EOK;
    sdp_result = g_clc_sdp_result;
    g_clc_sdp_pending = 0;
    peer_clc_unlock();

    if (rc != RT_EOK || sdp_result == 0)
    {
        peer_clc_lock();
        g_clc_connect_pending = 0;
        peer_clc_unlock();
        peer_reply_err(reply, PEER_ERR_TIMEOUT, "connect");
        return -1;
    }

    if (sdp_result != 1)
    {
        peer_clc_lock();
        g_clc_connect_pending = 0;
        peer_clc_unlock();
        peer_reply_err(reply, PEER_ERR_STATE, "connect");
        return -1;
    }

    /*
     * Drain first, then clear the recorded result and arm ACL failure
     * detection in one critical section. Doing the clear first would leave a
     * window in which a failure event for this address is recorded and then
     * has only its event bit cleared, so the wait below would read a stale 2.
     *
     * The drain also tells us whether a link came up during the search. The
     * instrument is pageable, so the peer can connect to it while it searches
     * for the service; that link answers this command. The check and the
     * clear are in the same lock for the same reason as above.
     */
    rt_event_recv(g_clc_event, PEER_CLC_BIT_CONN | PEER_CLC_BIT_CONN_FAIL,
                  RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 0, RT_NULL);

    peer_clc_lock();
    link_up = g_clc_connect_result == 1;
    if (!link_up)
    {
        /* From here an ACL failure for this address is this attempt's own. */
        g_clc_connect_result = 0;
        g_clc_acl_submitted = 1;
    }
    peer_clc_unlock();

    if (link_up)
    {
        /* The link is already tracked; asking for it again would open a
         * second channel or be refused, and the result is already recorded. */
        rc = RT_EOK;
    }
    else
    {
        if (bt_interface_spp_client_conn_req(&mac, (uint8_t *)uuid, uuid_len) != BT_EOK)
        {
            /* Nothing was issued, so there is nothing that can complete late
             * and no reason to arm the late-connection mark. */
            peer_clc_lock();
            g_clc_acl_submitted = 0;
            g_clc_connect_pending = 0;
            peer_clc_unlock();
            peer_reply_err(reply, PEER_ERR_STATE, "connect");
            return -1;
        }

        remain = (rt_int32_t)(deadline - rt_tick_get());
        if (remain > 0)
            rc = rt_event_recv(g_clc_event, PEER_CLC_BIT_CONN | PEER_CLC_BIT_CONN_FAIL,
                               RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                               (rt_tick_t)remain, RT_NULL);
        else
            rc = -RT_ETIMEOUT;
    }

    peer_clc_lock();
    /* The link may have come up, or the stack may have refused the ACL, just
     * as the wait expired; both are recorded before the event is signalled,
     * so either is authoritative. */
    if (rc != RT_EOK && g_clc_connect_result != 0)
        rc = RT_EOK;

    g_clc_connect_pending = 0;
    g_clc_acl_submitted = 0;
    cancel = 0;

    if (rc == RT_EOK && g_clc_connect_result == 1)
    {
        int slot;

        result = 1;
        link_id = g_clc_connect_link;
        slot = peer_clc_slot_by_id_locked(link_id);
        if (slot >= 0)
            channel = g_clc_links[slot].srv_chl;
    }
    else
    {
        /*
         * No link came up: the stack refused the ACL (2) or the wait expired
         * without a result (0). Either way, a connection completing for this
         * address later is not an incoming link and has to be dropped.
         */
        result = g_clc_connect_result;
        peer_clc_fail_arm_locked(addr);
        /* Only a page that has not produced an ACL can still be stopped. */
        if (result == 0)
            cancel = g_clc_acl_up ? 0 : 1;
    }
    peer_clc_unlock();

    /*
     * Without this the page would complete after the command already answered
     * `timeout`. The failure mark also covers a cancel that races with the
     * completion.
     */
    if (cancel)
    {
        cancel_rc = bt_interface_cancel_connect_req(mac.addr);
        if (cancel_rc != 0)
            LOG_W("bt_peer: cancel page to %s returned %d", addr_str, cancel_rc);
    }

    if (result != 1)
    {
        peer_reply_err(reply, result == 2 ? PEER_ERR_STATE : PEER_ERR_TIMEOUT,
                       "connect");
        return -1;
    }

    peer_reply_ok(reply, "link=%u addr=%s channel=%u role=0",
                  link_id, addr_str, channel);
    return 0;
}

int peer_classic_pair(const uint8_t *addr, uint32_t timeout_ms,
                      peer_reply_t *reply)
{
    bt_notify_device_mac_t mac;
    BTS2S_BD_ADDR bd;
    /*
     * The stack's pairing entry point wants connection information from an
     * inquiry result. We have none for this address, and info_vld == 0 is the
     * documented way to say so (the demo in bts2_app_generic.c:143-152 leaves
     * it unfilled for the same reason).
     */
    BTS2S_CONN_INFO conn_info;
    char addr_str[13];
    uint8_t result;
    rt_err_t rc;
    rt_err_t rc_db;

    if (g_clc_event == RT_NULL)
    {
        peer_reply_err(reply, PEER_ERR_NOT_READY, "classic");
        return -1;
    }

    memset(&mac, 0, sizeof(mac));
    memcpy(mac.addr, addr, 6);
    memset(&bd, 0, sizeof(bd));
    memset(&conn_info, 0, sizeof(conn_info));

    peer_clc_lock();
    if (!g_clc_ready)
    {
        peer_clc_unlock();
        peer_reply_err(reply, PEER_ERR_NOT_READY, "classic");
        return -1;
    }
    if (g_clc_pair_pending || g_clc_inquiry_pending ||
            g_clc_connect_pending)
    {
        peer_clc_unlock();
        peer_reply_err(reply, PEER_ERR_BUSY, "pair");
        return -1;
    }
    g_clc_pair_pending = 1;
    g_clc_pair_result = 0;
    memcpy(g_clc_pair_addr, addr, 6);
    peer_clc_unlock();

    /* Do not let a completion from an earlier attempt satisfy this one. */
    rt_event_recv(g_clc_event, PEER_CLC_BIT_PAIRED,
                  RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 0, RT_NULL);

    bt_addr_convert_to_bts((bd_addr_t *)&mac, &bd);
    /*
     * The app task id is what the finsh app stores as its protocol handle
     * (bts2_app_demo.c:54) and what the demo's own pairing request passes
     * (bts2_app_generic.c:147). sc_pair_req() starts the ACL if the peer is
     * not connected yet, so btc.pair works before btc.connect.
     */
    sc_pair_req(bts2_task_get_app_task_id(), &bd, &conn_info);

    rc = rt_event_recv(g_clc_event, PEER_CLC_BIT_PAIRED,
                       RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                       rt_tick_from_millisecond(peer_clc_wait_ms(timeout_ms)),
                       RT_NULL);

    peer_clc_lock();
    /* The stack may have recorded the result just as the wait expired. */
    if (rc != RT_EOK && g_clc_pair_result != 0)
        rc = RT_EOK;
    result = g_clc_pair_result;
    peer_clc_unlock();

    if (rc != RT_EOK || result == 0)
    {
        /*
         * Neither the pairing CFM nor an encryption event named the outcome.
         * Ask the security manager for its paired-device list: a successful
         * pairing is recorded there even when no event reaches the
         * application. An empty or unrelated list leaves the result at 0.
         */
        rt_event_recv(g_clc_event, PEER_CLC_BIT_PAIRED_DB,
                      RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 0, RT_NULL);

        peer_clc_lock();
        g_clc_pair_record_pending = 1;
        peer_clc_unlock();

        sc_rd_paired_dev_record_req(bts2_task_get_app_task_id());

        rc_db = rt_event_recv(g_clc_event, PEER_CLC_BIT_PAIRED_DB,
                              RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                              rt_tick_from_millisecond(PEER_CLC_PAIRED_DB_WAIT_MS),
                              RT_NULL);

        peer_clc_lock();
        g_clc_pair_record_pending = 0;
        /*
         * The list itself is only evidence when the wait delivered it (rc_db),
         * but a pairing event that named the outcome while the wait was
         * expiring is this attempt's evidence too and is recorded in
         * g_clc_pair_result. An expired wait that saw neither leaves the
         * result at 0. A recorded failure is never turned into a success by
         * the list: peer_clc_on_paired_record() refuses to overwrite it.
         */
        if (rc_db == RT_EOK || g_clc_pair_result != 0)
            result = g_clc_pair_result;
        peer_clc_unlock();
    }

    peer_clc_lock();
    g_clc_pair_pending = 0;
    peer_clc_unlock();

    peer_clc_addr_str(addr, addr_str);

    if (result != 1)
    {
#ifdef BSP_BT_CONNECTION_MANAGER
        /*
         * Last resort: the connection manager's own bond table, which is
         * updated when encryption is enabled. Only consulted when nothing
         * reported a failure.
         */
        if (result == 0 &&
                bt_cm_get_bonded_dev_by_addr((uint8_t *)addr) != RT_NULL)
        {
            peer_reply_ok(reply, "addr=%s", addr_str);
            return 0;
        }
#endif
        peer_reply_err(reply, result == 2 ? PEER_ERR_STATE : PEER_ERR_TIMEOUT,
                       "pair");
        return -1;
    }

    peer_reply_ok(reply, "addr=%s", addr_str);
    return 0;
}

int peer_classic_disconnect(uint8_t link_id, peer_reply_t *reply)
{
    bt_notify_device_mac_t mac;
    uint8_t channel;
    int slot;

    memset(&mac, 0, sizeof(mac));

    peer_clc_lock();
    slot = peer_clc_slot_by_id_locked(link_id);
    if (slot < 0)
    {
        peer_clc_unlock();
        peer_reply_err(reply, PEER_ERR_STATE, "link");
        return -1;
    }
    memcpy(mac.addr, g_clc_links[slot].mac, 6);
    channel = g_clc_links[slot].srv_chl;
    peer_clc_unlock();

    if (bt_interface_dis_spp_by_addr_and_chl_ext(&mac, channel) != BT_EOK)
    {
        peer_reply_err(reply, PEER_ERR_WRITE_FAILED, "disconnect");
        return -1;
    }

    /* The link stays tracked until btc.disc names it in the teardown. */
    peer_reply_ok(reply, "");
    return 0;
}

int peer_classic_send(uint8_t link_id, const uint8_t *data, uint16_t len,
                      peer_reply_t *reply)
{
    bt_notify_device_mac_t mac;
    uint16_t mtu;
    uint8_t channel;
    uint8_t *buf;
    bt_err_t rc;
    int slot;

    memset(&mac, 0, sizeof(mac));

    peer_clc_lock();
    slot = peer_clc_slot_by_id_locked(link_id);
    if (slot < 0)
    {
        peer_clc_unlock();
        peer_reply_err(reply, PEER_ERR_STATE, "link");
        return -1;
    }
    memcpy(mac.addr, g_clc_links[slot].mac, 6);
    channel = g_clc_links[slot].srv_chl;
    mtu = g_clc_links[slot].mtu;
    peer_clc_unlock();

    if (len > mtu)
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "len");
        return -1;
    }

    /* The stack frees the buffer itself on every error path and sends it on
     * success, so it is never freed here. */
    buf = (uint8_t *)bt_mem_alloc(len > 0 ? len : 1);
    if (buf == RT_NULL)
    {
        peer_reply_err(reply, PEER_ERR_NO_MEM, "spp");
        return -1;
    }

    if (len > 0)
        memcpy(buf, data, len);

    rc = bt_interface_spp_send_data_ext(buf, len, &mac, channel);
    if (rc != BT_EOK)
    {
        peer_reply_err(reply, PEER_ERR_WRITE_FAILED, "spp");
        return -1;
    }

    peer_reply_ok(reply, "bytes=%u", len);
    return 0;
}

void peer_classic_reset(void)
{
    uint8_t ready;
    uint8_t inquiry_running;
    uint16_t used;
    int i;

    if (g_clc_lock == RT_NULL)
        return;

    peer_clc_lock();
    g_clc_quiet = 1;
    ready = g_clc_ready;
    inquiry_running = g_clc_inquiry_pending;
    g_clc_inquiry_pending = 0;
    g_clc_connect_pending = 0;
    g_clc_connect_result = 0;
    g_clc_connect_link = 0;
    g_clc_acl_submitted = 0;
    g_clc_acl_up = 0;
    g_clc_fail_valid = 0;
    g_clc_pair_pending = 0;
    g_clc_pair_result = 0;
    g_clc_pair_record_pending = 0;
    g_clc_sdp_pending = 0;
    g_clc_sdp_result = 0;
    peer_clc_unlock();

    if (ready && inquiry_running)
        bt_interface_stop_inquiry();

    if (ready)
    {
        bt_interface_dis_spp_all();

        /* Bound the wait: a link the peer refuses to drop must not stall the
         * reset. Whatever is still tracked afterwards is forgotten below, so
         * its late btc.disc finds nothing to name. */
        for (i = 0; i < PEER_CLC_RESET_WAIT_MS / PEER_CLC_RESET_POLL_MS; i++)
        {
            peer_clc_lock();
            used = peer_clc_link_count_locked();
            peer_clc_unlock();

            if (used == 0)
                break;
            rt_thread_mdelay(PEER_CLC_RESET_POLL_MS);
        }
    }

    peer_clc_lock();
    memset(g_clc_links, 0, sizeof(g_clc_links));
    g_clc_quiet = 0;
    peer_clc_unlock();

#ifdef BSP_BT_CONNECTION_MANAGER
    if (ready)
        bt_cm_delete_bonded_devs();
#endif
}

#else /* !PEER_CLASSIC */

/* Keep the translation unit non-empty for toolchains that warn about it. */
typedef int peer_classic_unused_t;

#endif /* PEER_CLASSIC */
