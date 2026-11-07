/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * BLE GAP central/peripheral management for the bt_peer instrument.
 */

#include <rtthread.h>
#include <string.h>

#include "peer.h"
#include "bf0_ble_common.h"
#include "bf0_sibles.h"
#include "bf0_sibles_advertising.h"

static rt_event_t g_ble_event;
static volatile uint8_t g_powered_on;

static peer_link_t g_links[PEER_MAX_LINKS];
static peer_adv_report_t g_adv[PEER_MAX_ADV_REPORTS];
static uint16_t g_adv_count;
static int8_t g_scan_rssi_min = -127;
static uint8_t g_scanning;
static uint8_t g_scan_failed;

static uint8_t g_connect_pending;
static int g_connect_result;
static uint8_t g_connect_link;

static uint8_t g_adv_on;
/** Status the last ADV_STARTED callback carried; 0 means started. */
static int g_adv_start_status;

/* Results of the last update request, written by the BLE callback thread. */
static uint8_t g_conn_param_status;
static uint8_t g_data_len_status;
static uint8_t g_phy_rx;
static uint8_t g_phy_tx;
/* Which connection each outstanding request belongs to, so a completion for
 * another link (or a late one) cannot satisfy it. */
static uint8_t g_conn_param_conn = PEER_NO_CONN;
static uint8_t g_data_len_conn = PEER_NO_CONN;
static uint8_t g_phy_conn = PEER_NO_CONN;
/* Whether the running advertisement was started in extended mode. */
static uint8_t g_adv_extended;

/* UID-derived public address and device name. Without the override below the
 * stack's weak default is the fixed AB:05:03:02:05:08, which every instrument
 * in a lab would share. */
static bd_addr_t g_local_addr;
static uint8_t g_local_addr_valid;
static char g_local_name[32];

SIBLES_ADVERTISING_CONTEXT_DECLAR(g_adv_context);

static const char s_hex[] = "0123456789abcdef";

void peer_ble_addr_str(const bd_addr_t *addr, char *out, size_t out_size)
{
    uint8_t i;

    if (out_size < BD_ADDR_LEN * 2 + 1)
    {
        if (out_size > 0)
            out[0] = '\0';
        return;
    }

    for (i = 0; i < BD_ADDR_LEN; i++)
    {
        out[i * 2] = s_hex[addr->addr[i] >> 4];
        out[i * 2 + 1] = s_hex[addr->addr[i] & 0x0F];
    }
    out[BD_ADDR_LEN * 2] = '\0';
}

/*
 * The stack calls this weak hook while bringing the controller up. The
 * examples derive the public address from the chip UID; without the override
 * the weak fallback is one fixed address for every board in the lab.
 */
ble_common_update_type_t ble_request_public_address(bd_addr_t *addr)
{
    if (bt_mac_addr_generate_via_uid_v2(addr) != 0)
        return BLE_UPDATE_NO_UPDATE;

    g_local_addr = *addr;
    g_local_addr_valid = 1;
    return BLE_UPDATE_ONCE;
}

void peer_ble_local_addr_str(char *out, size_t out_size)
{
    if (g_local_addr_valid)
        peer_ble_addr_str(&g_local_addr, out, out_size);
    else if (out_size > 0)
        out[0] = '\0';
}

const char *peer_ble_local_name(void)
{
    return g_local_name;
}

static void peer_ble_build_name(void)
{
    if (!g_local_addr_valid)
    {
        g_local_name[0] = '\0';
        return;
    }

    /* Four bytes: 16 characters, under GAP_MAX_LOCAL_NAME_LEN even on 55X (18). */
    rt_snprintf(g_local_name, sizeof(g_local_name),
                "bt_peer-%02x%02x%02x%02x",
                g_local_addr.addr[0], g_local_addr.addr[1],
                g_local_addr.addr[2], g_local_addr.addr[3]);
}

/** Install the name on the LE stack once it is powered up. */
static void peer_ble_apply_name(void)
{
    ble_gap_dev_name_t *dev;
    bd_addr_t effective;
    size_t len;

    /* The hook requests the UID address, but NVDS may already hold a
     * provisioned (factory/product) one; read back what the stack uses. */
    if (ble_get_public_address(&effective) == HL_ERR_NO_ERROR)
    {
        g_local_addr = effective;
        g_local_addr_valid = 1;
    }
    peer_ble_build_name();

    if (!g_local_addr_valid)
        return;

    len = (size_t)rt_strlen(g_local_name);
    dev = rt_malloc(sizeof(*dev) + len);
    if (dev == RT_NULL)
        return;
    dev->len = (uint16_t)len;
    memcpy(dev->name, g_local_name, len);
    ble_gap_set_dev_name(dev);
    rt_free(dev);
}

void peer_ble_init(void)
{
    bd_addr_t addr;

    /* Seed before main() prints the banner, so an early `peer info` already
     * carries the UID-derived values; the power-on path refreshes them from
     * NVDS, which is authoritative. */
    if (bt_mac_addr_generate_via_uid_v2(&addr) == 0)
    {
        g_local_addr = addr;
        g_local_addr_valid = 1;
        peer_ble_build_name();
    }

    if (g_ble_event == RT_NULL)
        g_ble_event = rt_event_create("peer_ble", RT_IPC_FLAG_FIFO);
}

uint8_t peer_ble_ready(void)
{
    return g_powered_on;
}

rt_err_t peer_ble_wait(uint32_t bits, int32_t timeout_ms)
{
    rt_tick_t timeout;

    if (g_ble_event == RT_NULL)
        return -RT_ERROR;

    /*
     * Do not clear here: the caller clears before issuing the async request.
     * Clearing at this point would also drop a completion that arrived between
     * the request and this call (the mbox thread can preempt the shell).
     */
    if (timeout_ms < 0)
        timeout = RT_WAITING_FOREVER;
    else
        timeout = rt_tick_from_millisecond((rt_int32_t)timeout_ms);

    return rt_event_recv(g_ble_event, bits, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                         timeout, RT_NULL);
}

void peer_ble_signal(uint32_t bits)
{
    if (g_ble_event != RT_NULL)
        rt_event_send(g_ble_event, bits);
}

void peer_ble_clear_bits(uint32_t bits)
{
    if (g_ble_event != RT_NULL)
        rt_event_recv(g_ble_event, bits, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                      0, RT_NULL);
}

peer_link_t *peer_link_find(uint8_t link_id)
{
    uint8_t i;

    if (link_id == 0)
        return RT_NULL;

    for (i = 0; i < PEER_MAX_LINKS; i++)
        if (g_links[i].used && g_links[i].link_id == link_id)
            return &g_links[i];

    return RT_NULL;
}

peer_link_t *peer_link_find_conn(uint8_t conn_idx)
{
    uint8_t i;

    for (i = 0; i < PEER_MAX_LINKS; i++)
        if (g_links[i].used && g_links[i].conn_idx == conn_idx)
            return &g_links[i];

    return RT_NULL;
}

peer_link_t *peer_link_at(uint8_t slot)
{
    if (slot >= PEER_MAX_LINKS || !g_links[slot].used)
        return RT_NULL;

    return &g_links[slot];
}

uint8_t peer_link_count(void)
{
    uint8_t i;
    uint8_t count = 0;

    for (i = 0; i < PEER_MAX_LINKS; i++)
        if (g_links[i].used)
            count++;

    return count;
}

const peer_adv_report_t *peer_ble_adv_report(uint8_t index)
{
    if (index == 0 || index > g_adv_count)
        return RT_NULL;

    return &g_adv[index - 1];
}

uint8_t peer_ble_adv_report_count(void)
{
    return (uint8_t)g_adv_count;
}

static peer_link_t *peer_link_add(uint8_t conn_idx, uint8_t role,
                                  uint16_t interval, uint8_t addr_type,
                                  const bd_addr_t *addr)
{
    uint8_t i;

    for (i = 0; i < PEER_MAX_LINKS; i++)
    {
        peer_link_t *link = &g_links[i];

        if (link->used)
            continue;

        memset(link, 0, sizeof(*link));
        link->used = 1;
        link->link_id = i + 1;
        link->conn_idx = conn_idx;
        link->role = role;
        link->interval = interval;
        link->addr_type = addr_type;
        link->addr = *addr;
        link->mtu = 23;
        /* sibles_register_remote_svc() returns a 0-based slot index, so 0 is a
         * valid registration and must not double as "not discovered". */
        link->rmt_svc_hdl = SIBLES_ERROR_REMOTE_HANDLE;
        return link;
    }

    return RT_NULL;
}

static void peer_ble_handle_adv(ble_gap_ext_adv_report_ind_t *ind)
{
    peer_adv_report_t *report;
    char addr[BD_ADDR_LEN * 2 + 1];
    char payload[PEER_ADV_DATA_MAX * 2 + 32];
    uint8_t type;

    if (ind->rssi < g_scan_rssi_min)
        return;

    type = ind->info & GAPM_REPORT_INFO_REPORT_TYPE_MASK;
    if (type == GAPM_REPORT_TYPE_PER_ADV)
        return;

    if (g_adv_count >= PEER_MAX_ADV_REPORTS)
        return;

    report = &g_adv[g_adv_count];
    report->addr_type = ind->addr.addr_type;
    report->report_type = type;
    report->conn_adv = (ind->info & GAPM_REPORT_INFO_CONN_ADV_BIT) ? 1 : 0;
    report->adv_sid = ind->adv_sid;
    report->rssi = ind->rssi;
    report->addr = ind->addr.addr;
    report->len = ind->length > PEER_ADV_DATA_MAX ? PEER_ADV_DATA_MAX : ind->length;
    memcpy(report->data, ind->data, report->len);
    g_adv_count++;

    peer_ble_addr_str(&report->addr, addr, sizeof(addr));
    peer_format_payload(payload, sizeof(payload), report->data, report->len);

    peer_emit_evt("adv", "index=%u addr=%s type=%u conn=%u rssi=%d %s",
                  g_adv_count, addr, report->addr_type, report->conn_adv,
                  report->rssi, payload);
}

static void peer_ble_handle_connected(ble_gap_connect_ind_t *ind)
{
    peer_link_t *link;
    char addr[BD_ADDR_LEN * 2 + 1];

    link = peer_link_add(ind->conn_idx, ind->role, ind->con_interval,
                         ind->peer_addr_type, &ind->peer_addr);
    if (link == RT_NULL)
    {
        ble_gap_disconnect_t dis;

        dis.conn_idx = ind->conn_idx;
        dis.reason = CO_ERROR_REMOTE_USER_TERM_CON;
        ble_gap_disconnect(&dis);
        return;
    }

    peer_ble_addr_str(&link->addr, addr, sizeof(addr));
    peer_emit_evt("conn", "link=%u addr=%s role=%u interval_ms=%u",
                  link->link_id, addr, link->role,
                  (unsigned)(link->interval * 5 / 4));

    /* Ask for the largest usable MTU; the result arrives as SIBLES_MTU_EXCHANGE_IND. */
    sibles_exchange_mtu(link->conn_idx);

    if (g_connect_pending)
    {
        g_connect_result = 0;
        g_connect_link = link->link_id;
        peer_ble_signal(PEER_BIT_CONNECTED);
    }
}

static void peer_ble_handle_disconnected(ble_gap_disconnected_ind_t *ind)
{
    peer_link_t *link = peer_link_find_conn(ind->conn_idx);

    if (link == RT_NULL)
        return;

    peer_emit_evt("disconn", "link=%u reason=%u", link->link_id, ind->reason);

    /* Release the remote service registration before the slot is reused. */
    peer_gatt_client_on_disconnect(ind->conn_idx);
    /* A reused conn_idx must not inherit the previous client's CCCD state. */
    peer_gatt_server_on_disconnect(ind->conn_idx);

    link->used = 0;
    link->gatt_busy = 0;
}

static int peer_ble_event_handler(uint16_t event_id, uint8_t *data, uint16_t len,
                                  uint32_t context)
{
    (void)len;
    (void)context;

    switch (event_id)
    {
    case BLE_POWER_ON_IND:
        g_powered_on = 1;
        peer_ble_apply_name();
        break;

    case BLE_GAP_SCAN_START_CNF:
    {
        ble_gap_start_scan_cnf_t *cnf = (ble_gap_start_scan_cnf_t *)data;

        if (cnf->status != HL_ERR_NO_ERROR)
        {
            g_scan_failed = 1;
            g_scanning = 0;
        }
        break;
    }

    case BLE_GAP_SCAN_STOPPED_IND:
        g_scanning = 0;
        peer_emit_evt("scan.done", "count=%u", g_adv_count);
        peer_ble_signal(PEER_BIT_SCAN_DONE);
        break;

    case BLE_GAP_EXT_ADV_REPORT_IND:
        peer_ble_handle_adv((ble_gap_ext_adv_report_ind_t *)data);
        break;

    case BLE_GAP_CONNECTED_IND:
        peer_ble_handle_connected((ble_gap_connect_ind_t *)data);
        break;

    case BLE_GAP_CREATE_CONNECTION_CNF:
    {
        ble_gap_create_connection_cnf_t *cnf = (ble_gap_create_connection_cnf_t *)data;

        if (g_connect_pending && cnf->status != HL_ERR_NO_ERROR)
        {
            g_connect_result = -1;
            peer_ble_signal(PEER_BIT_CONN_FAIL);
        }
        break;
    }

    case BLE_GAP_CANCEL_CREATE_CONNECTION_CNF:
        if (g_connect_pending)
        {
            g_connect_result = -1;
            peer_ble_signal(PEER_BIT_CONN_FAIL);
        }
        break;

    case BLE_GAP_DISCONNECTED_IND:
        peer_ble_handle_disconnected((ble_gap_disconnected_ind_t *)data);
        break;

    case BLE_GAP_UPDATE_CONN_PARAM_IND:
    {
        ble_gap_update_conn_param_ind_t *ind = (ble_gap_update_conn_param_ind_t *)data;
        peer_link_t *link = peer_link_find_conn(ind->conn_idx);

        if (link != RT_NULL)
        {
            link->interval = ind->con_interval;
            /* con_interval is 1.25 ms units; sup_to is 10 ms units. */
            peer_emit_evt("conn_params",
                          "link=%u interval_ms=%u latency=%u timeout_ms=%u",
                          link->link_id,
                          (unsigned)((uint32_t)ind->con_interval * 5 / 4),
                          ind->con_latency,
                          (unsigned)((uint32_t)ind->sup_to * 10));
        }
        break;
    }

    case BLE_GAP_UPDATE_CONN_PARAM_CNF:
    {
        ble_gap_update_conn_param_cnf_t *cnf =
            (ble_gap_update_conn_param_cnf_t *)data;

        if (cnf->conn_idx != g_conn_param_conn)
            break;
        g_conn_param_conn = PEER_NO_CONN;
        g_conn_param_status = cnf->status;
        peer_ble_signal(PEER_BIT_CONN_PARAM_DONE);
        break;
    }

    case BLE_GAP_UPDATE_DATA_LENGTH_IND:
    {
        ble_gap_update_data_length_ind_t *ind =
            (ble_gap_update_data_length_ind_t *)data;
        peer_link_t *link = peer_link_find_conn(ind->conn_idx);

        if (link != RT_NULL)
            peer_emit_evt("data_len", "link=%u tx_octets=%u tx_time=%u",
                          link->link_id, ind->max_tx_octets, ind->max_tx_time);
        break;
    }

    case BLE_GAP_UPDATE_DATA_LENGTH_CNF:
    {
        ble_gap_update_data_length_cnf_t *cnf =
            (ble_gap_update_data_length_cnf_t *)data;

        if (cnf->conn_idx != g_data_len_conn)
            break;
        g_data_len_conn = PEER_NO_CONN;
        g_data_len_status = cnf->status;
        peer_ble_signal(PEER_BIT_DATA_LEN_DONE);
        break;
    }

    case BLE_GAP_REMOTE_PHY_IND:
    {
        ble_gap_remote_phy_ind_t *ind = (ble_gap_remote_phy_ind_t *)data;

        if (ind->conn_idx != g_phy_conn)
            break;
        g_phy_conn = PEER_NO_CONN;
        g_phy_rx = ind->rx_phy;
        g_phy_tx = ind->tx_phy;
        peer_ble_signal(PEER_BIT_PHY_DONE);
        break;
    }

    case SIBLES_MTU_EXCHANGE_IND:
    {
        sibles_mtu_exchange_ind_t *ind = (sibles_mtu_exchange_ind_t *)data;
        peer_link_t *link = peer_link_find_conn(ind->conn_idx);

        if (link != RT_NULL)
        {
            link->mtu = ind->mtu;
            peer_emit_evt("mtu", "link=%u mtu=%u", link->link_id, ind->mtu);
            peer_ble_signal(PEER_BIT_MTU_DONE);
        }
        break;
    }

    case SIBLES_SEARCH_SVC_RSP:
        peer_gatt_client_on_search(data);
        break;

    case SIBLES_WRITE_REMOTE_VALUE_RSP:
        peer_gatt_client_on_write_rsp(data);
        break;

    case SIBLES_READ_REMOTE_VALUE_ERROR_IND:
        peer_gatt_client_on_read_error(data);
        break;

    case SIBLES_WRITE_VALUE_RSP:
        peer_ble_signal(PEER_BIT_NOTIFY_DONE);
        break;

    default:
        break;
    }

    return 0;
}

BLE_EVENT_REGISTER(peer_ble_event_handler, NULL);

int peer_ble_scan_start(uint32_t duration_ms, uint32_t interval_ms,
                        uint32_t window_ms, uint8_t dup, int8_t rssi_min,
                        peer_reply_t *reply)
{
    ble_gap_scan_start_t param;
    rt_tick_t start;
    rt_tick_t budget;
    uint32_t first_seq;
    uint8_t ret;

    if (!g_powered_on)
    {
        peer_reply_err(reply, PEER_ERR_NOT_READY, "ble");
        return -1;
    }

    if (g_scanning)
    {
        peer_reply_err(reply, PEER_ERR_STATE, "scan_running");
        return -1;
    }

    memset(&param, 0, sizeof(param));
    memset(g_adv, 0, sizeof(g_adv));
    g_adv_count = 0;
    g_scan_rssi_min = rssi_min;
    g_scan_failed = 0;

    /*
     * Take the boundary before the scan can produce anything. The host filters
     * on it, so a report still in flight from an earlier scan cannot be counted
     * as one of this scan's.
     */
    first_seq = peer_evt_seq_current();

    param.own_addr_type = GAPM_STATIC_ADDR;
    /* Observer receives legacy, extended and periodic reports alike. */
    param.type = GAPM_SCAN_TYPE_OBSERVER;
    param.dup_filt_pol = dup;
    param.scan_param_1m.scan_intv = (uint16_t)(interval_ms * 8 / 5);
    param.scan_param_1m.scan_wd = (uint16_t)(window_ms * 8 / 5);
    param.duration = (uint16_t)(duration_ms / 10);
    param.period = 0;

    ret = ble_gap_scan_start(&param);
    if (ret != HL_ERR_NO_ERROR)
    {
        peer_reply_err(reply, PEER_ERR_STATE, "scan_start");
        return -1;
    }

    g_scanning = 1;

    /*
     * The scan is bounded by its own duration, so wait for the controller to
     * stop it before answering: only then is the report count final, and the
     * host can consume exactly as many `adv` events as `count` promises.
     */
    start = rt_tick_get();
    budget = rt_tick_from_millisecond((rt_int32_t)duration_ms + 3000);
    while (g_scanning && (rt_tick_get() - start) < budget)
        rt_thread_mdelay(10);

    if (g_scanning)
    {
        ble_gap_scan_stop();
        g_scanning = 0;
        peer_reply_err(reply, PEER_ERR_TIMEOUT, "gap.scan");
        return -1;
    }

    if (g_scan_failed)
    {
        peer_reply_err(reply, PEER_ERR_STATE, "scan_start");
        return -1;
    }

    peer_reply_ok(reply, "count=%u first_seq=%u", g_adv_count,
                  (unsigned)first_seq);
    return 0;
}

int peer_ble_scan_stop(peer_reply_t *reply)
{
    if (!g_scanning)
    {
        peer_reply_ok(reply, "");
        return 0;
    }

    /* Answer only once the controller has stopped; an immediate next scan
     * would otherwise see g_scanning still set and get @err state. */
    peer_ble_clear_bits(PEER_BIT_SCAN_DONE);
    if (ble_gap_scan_stop() != HL_ERR_NO_ERROR)
    {
        peer_reply_err(reply, PEER_ERR_STATE, "scan_stop");
        return -1;
    }

    if (peer_ble_wait(PEER_BIT_SCAN_DONE, 2000) != RT_EOK)
    {
        peer_reply_err(reply, PEER_ERR_TIMEOUT, "scan_stop");
        return -1;
    }

    peer_reply_ok(reply, "");
    return 0;
}

int peer_ble_connect(const peer_adv_report_t *report, const uint8_t *addr,
                     uint8_t addr_type, uint32_t timeout_ms,
                     uint32_t interval_ms, uint8_t *link_id,
                     peer_reply_t *reply)
{
    ble_gap_connection_create_param_t param;
    uint16_t conn_interval;
    uint8_t ret;

    if (!g_powered_on)
    {
        peer_reply_err(reply, PEER_ERR_NOT_READY, "ble");
        return -1;
    }

    if (g_connect_pending)
    {
        peer_reply_err(reply, PEER_ERR_BUSY, "connect");
        return -1;
    }

    if (g_scanning)
    {
        ble_gap_scan_stop();
        g_scanning = 0;
    }

    memset(&param, 0, sizeof(param));
    conn_interval = (uint16_t)(interval_ms * 4 / 5);

    param.own_addr_type = GAPM_STATIC_ADDR;
    param.type = GAPM_INIT_TYPE_DIRECT_CONN_EST;
    param.conn_to = (uint16_t)(timeout_ms / 10);
    param.conn_param_1m.scan_intv = 0x30;
    param.conn_param_1m.scan_wd = 0x30;
    param.conn_param_1m.conn_intv_min = conn_interval;
    param.conn_param_1m.conn_intv_max = conn_interval;
    param.conn_param_1m.conn_latency = 0;
    param.conn_param_1m.supervision_to = 500;
    param.conn_param_1m.ce_len_min = 0;
    param.conn_param_1m.ce_len_max = 48;

    if (report != RT_NULL)
    {
        param.peer_addr.addr_type = report->addr_type;
        param.peer_addr.addr = report->addr;
    }
    else
    {
        param.peer_addr.addr_type = addr_type;
        memcpy(param.peer_addr.addr.addr, addr, BD_ADDR_LEN);
    }

    g_connect_pending = 1;
    g_connect_result = -1;
    g_connect_link = 0;
    /* A second connect must not be released by the first one's bit. */
    peer_ble_clear_bits(PEER_BIT_CONNECTED | PEER_BIT_CONN_FAIL);

    ret = ble_gap_create_connection(&param);
    if (ret != HL_ERR_NO_ERROR)
    {
        g_connect_pending = 0;
        peer_reply_err(reply, PEER_ERR_STATE, "connect_start");
        return -1;
    }

    if (peer_ble_wait(PEER_BIT_CONNECTED | PEER_BIT_CONN_FAIL,
                      (int32_t)timeout_ms + 2000) != RT_EOK ||
            g_connect_result != 0)
    {
        g_connect_pending = 0;
        ble_gap_cancel_create_connection();
        peer_reply_err(reply, PEER_ERR_TIMEOUT, "connect");
        return -1;
    }

    g_connect_pending = 0;
    *link_id = g_connect_link;
    return 0;
}

int peer_ble_disconnect(uint8_t link_id, peer_reply_t *reply)
{
    peer_link_t *link = peer_link_find(link_id);
    ble_gap_disconnect_t param;

    if (link == RT_NULL)
    {
        peer_reply_err(reply, PEER_ERR_STATE, "link");
        return -1;
    }

    param.conn_idx = link->conn_idx;
    param.reason = CO_ERROR_REMOTE_USER_TERM_CON;

    if (ble_gap_disconnect(&param) != HL_ERR_NO_ERROR)
    {
        peer_reply_err(reply, PEER_ERR_STATE, "disconnect");
        return -1;
    }

    return 0;
}

void peer_ble_reset(void)
{
    uint8_t i;

    if (g_scanning)
    {
        ble_gap_scan_stop();
        g_scanning = 0;
    }

    for (i = 0; i < PEER_MAX_LINKS; i++)
    {
        if (!g_links[i].used)
            continue;

        {
            ble_gap_disconnect_t param;

            param.conn_idx = g_links[i].conn_idx;
            param.reason = CO_ERROR_REMOTE_USER_TERM_CON;
            ble_gap_disconnect(&param);
        }
    }

    memset(g_links, 0, sizeof(g_links));
    memset(g_adv, 0, sizeof(g_adv));
    g_adv_count = 0;
    g_scan_failed = 0;
    g_connect_pending = 0;
    g_conn_param_conn = PEER_NO_CONN;
    g_data_len_conn = PEER_NO_CONN;
    g_phy_conn = PEER_NO_CONN;
    g_adv_extended = 0;

    if (g_adv_on)
    {
        sibles_advertising_stop(g_adv_context);
        g_adv_on = 0;
    }

    peer_ble_clear_bits(0xFFFFFFFF);
}

static uint8_t peer_adv_event_cb(uint8_t event, void *context, void *data)
{
    (void)context;

    if (event == SIBLES_ADV_EVT_ADV_STARTED)
    {
        /* SIBLES reports a failed start as ADV_STARTED with a status; taking
         * the callback alone as success would report an advertisement that the
         * controller refused. */
        sibles_adv_evt_startted_t *started = (sibles_adv_evt_startted_t *)data;

        g_adv_start_status = started != RT_NULL ? (int)started->status : -1;
        g_adv_on = g_adv_start_status == 0;
        peer_ble_signal(PEER_BIT_ADV_STARTED);
    }
    else if (event == SIBLES_ADV_EVT_ADV_STOPPED)
    {
        g_adv_on = 0;
    }

    return 0;
}

int peer_ble_adv_start(uint8_t extended, uint32_t interval_ms,
                       uint8_t connectable, const uint8_t *data,
                       uint16_t len, peer_reply_t *reply)
{
    sibles_advertising_para_t para;
    uint32_t interval;
    uint8_t ret;

    if (!g_powered_on)
    {
        peer_reply_err(reply, PEER_ERR_NOT_READY, "ble");
        return -1;
    }

    if (g_adv_on)
    {
        peer_reply_err(reply, PEER_ERR_STATE, "adv_running");
        return -1;
    }

    if (len > (extended ? 251 : 31))
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "len");
        return -1;
    }

    interval = interval_ms * 8 / 5;
    if (interval < 32)
        interval = 32;

    memset(&para, 0, sizeof(para));
    para.own_addr_type = GAPM_STATIC_ADDR;
    para.config.max_tx_pwr = 0x7F;
    para.config.is_auto_restart = 0;
    para.evt_handler = peer_adv_event_cb;

    if (extended)
    {
        para.config.adv_mode = SIBLES_ADV_EXTENDED_MODE;
        para.config.mode_config.extended_config.duration = 0;
        para.config.mode_config.extended_config.interval = interval;
        para.config.mode_config.extended_config.max_skip = 0;
        para.config.mode_config.extended_config.phy = GAP_PHY_TYPE_LE_1M;
        para.config.mode_config.extended_config.adv_sid = 0;
        para.config.mode_config.extended_config.connectable_enable = connectable;
    }
    else if (connectable)
    {
        para.config.adv_mode = SIBLES_ADV_CONNECT_MODE;
        para.config.mode_config.conn_config.duration = 0;
        para.config.mode_config.conn_config.interval = interval;
    }
    else
    {
        para.config.adv_mode = SIBLES_ADV_BROADCAST_MODE;
        para.config.mode_config.broadcast_config.duration = 0;
        para.config.mode_config.broadcast_config.interval = interval;
        para.config.mode_config.broadcast_config.scannable_enable = 0;
    }

    if (len > 0)
    {
        para.adv_data.customized_data = rt_malloc(sizeof(sibles_adv_type_customize_t) + len);
        if (para.adv_data.customized_data == RT_NULL)
        {
            peer_reply_err(reply, PEER_ERR_NO_MEM, "adv_data");
            return -1;
        }
        para.adv_data.customized_data->len = (uint8_t)len;
        memcpy(para.adv_data.customized_data->data, data, len);
    }

    g_adv_extended = extended;
    peer_ble_clear_bits(PEER_BIT_ADV_STARTED);

    ret = sibles_advertising_init(g_adv_context, &para);
    if (ret == SIBLES_ADV_NO_ERR)
        ret = sibles_advertising_start(g_adv_context);

    rt_free(para.adv_data.customized_data);

    if (ret != SIBLES_ADV_NO_ERR)
    {
        peer_reply_err(reply, PEER_ERR_STATE, "adv_start");
        return -1;
    }

    /* The SIBLES state stays STARTING until the controller's CNF; answering
     * earlier lets an immediate adv.data hit SIBLES_ADV_NOT_ALLOWED. */
    if (peer_ble_wait(PEER_BIT_ADV_STARTED, 2000) != RT_EOK)
    {
        peer_reply_err(reply, PEER_ERR_TIMEOUT, "adv_start");
        return -1;
    }

    if (g_adv_start_status != 0)
    {
        peer_reply_err(reply, PEER_ERR_STATE, "adv_start rc=%d", g_adv_start_status);
        return -1;
    }

    peer_reply_ok(reply, "mode=%s interval_ms=%u",
                  extended ? "extended" : "legacy", (unsigned)interval_ms);
    return 0;
}

int peer_ble_adv_stop(peer_reply_t *reply)
{
    uint8_t ret;
    uint8_t i;

    if (!g_adv_on)
    {
        peer_reply_ok(reply, "");
        return 0;
    }

    /* Same STARTING window as the update path. */
    ret = SIBLES_ADV_NOT_ALLOWED;
    for (i = 0; i < 20; i++)
    {
        ret = sibles_advertising_stop(g_adv_context);
        if (ret != SIBLES_ADV_NOT_ALLOWED)
            break;
        rt_thread_mdelay(20);
    }
    g_adv_on = 0;

    if (ret != SIBLES_ADV_NO_ERR)
    {
        peer_reply_err(reply, PEER_ERR_STATE, "adv_stop");
        return -1;
    }

    peer_reply_ok(reply, "");
    return 0;
}

int peer_ble_adv_update(const uint8_t *data, uint16_t len, peer_reply_t *reply)
{
    sibles_adv_data_t adv;
    sibles_adv_type_customize_t *custom = RT_NULL;
    uint16_t max_len = g_adv_extended ? 248 : 31;
    uint8_t ret;
    uint8_t i;

    if (!g_adv_on)
    {
        peer_reply_err(reply, PEER_ERR_STATE, "adv_stopped");
        return -1;
    }

    /* compose reserves 3 bytes, so the extended ceiling is 248; legacy is 31.
     * An empty payload cannot clear an advertisement through SIBLES, so it is
     * refused rather than answered @ok with the old data still on air. */
    if (len == 0 || len > max_len)
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "len");
        return -1;
    }

    memset(&adv, 0, sizeof(adv));
    if (len > 0)
    {
        custom = rt_malloc(sizeof(*custom) + len);
        if (custom == RT_NULL)
        {
            peer_reply_err(reply, PEER_ERR_NO_MEM, "adv_data");
            return -1;
        }
        custom->len = (uint8_t)len;
        memcpy(custom->data, data, len);
        adv.customized_data = custom;
    }

    /* The ADV_STARTED callback can fire just before SIBLES flips its state to
     * STARTED; a caller that updates immediately then sees NOT_ALLOWED. Retry
     * briefly instead of reporting a state error for a transient one. */
    ret = SIBLES_ADV_NOT_ALLOWED;
    for (i = 0; i < 20; i++)
    {
        ret = sibles_advertising_update_adv_and_scan_rsp_data(g_adv_context,
                                                              &adv, RT_NULL);
        if (ret != SIBLES_ADV_NOT_ALLOWED)
            break;
        rt_thread_mdelay(20);
    }
    rt_free(custom);

    if (ret == SIBLES_ADV_DATA_LENGTH_EXCEED)
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "len");
        return -1;
    }
    if (ret != SIBLES_ADV_NO_ERR)
    {
        peer_reply_err(reply, PEER_ERR_STATE, "adv_data");
        return -1;
    }

    peer_reply_ok(reply, "len=%u", (unsigned)len);
    return 0;
}

int peer_ble_update_conn_param(uint8_t link_id, uint16_t intv_min_ms,
                               uint16_t intv_max_ms, uint16_t latency,
                               uint16_t timeout_ms, peer_reply_t *reply)
{
    peer_link_t *link = peer_link_find(link_id);
    ble_gap_update_conn_param_t param;

    if (link == RT_NULL)
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "link");
        return -1;
    }
    if (intv_min_ms < 8 || intv_max_ms < intv_min_ms || timeout_ms < 100)
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "params");
        return -1;
    }

    memset(&param, 0, sizeof(param));
    param.conn_idx = link->conn_idx;
    param.intv_min = (uint16_t)(intv_min_ms * 4 / 5);
    param.intv_max = (uint16_t)(intv_max_ms * 4 / 5);
    param.latency = latency;
    param.time_out = (uint16_t)(timeout_ms / 10);
    param.ce_len_min = 1;
    param.ce_len_max = 0x100;

    g_conn_param_status = 0xFF;
    g_conn_param_conn = link->conn_idx;
    peer_ble_clear_bits(PEER_BIT_CONN_PARAM_DONE);

    if (ble_gap_update_conn_param(&param) != HL_ERR_NO_ERROR)
    {
        g_conn_param_conn = PEER_NO_CONN;
        peer_reply_err(reply, PEER_ERR_STATE, "conn_params");
        return -1;
    }

    if (peer_ble_wait(PEER_BIT_CONN_PARAM_DONE, 3000) != RT_EOK)
    {
        g_conn_param_conn = PEER_NO_CONN;
        peer_reply_err(reply, PEER_ERR_TIMEOUT, "conn_params");
        return -1;
    }

    if (g_conn_param_status != HL_ERR_NO_ERROR)
    {
        peer_reply_err(reply, PEER_ERR_STATE, "conn_params");
        return -1;
    }

    peer_reply_ok(reply, "link=%u interval_min_ms=%u interval_max_ms=%u latency=%u timeout_ms=%u",
                  link_id, intv_min_ms, intv_max_ms, latency, timeout_ms);
    return 0;
}

int peer_ble_update_phy(uint8_t link_id, uint8_t phy, peer_reply_t *reply)
{
    peer_link_t *link = peer_link_find(link_id);
    ble_gap_update_phy_t param;

    if (link == RT_NULL)
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "link");
        return -1;
    }
    /* 1 = 1M, 2 = 2M, 3 = Coded -- the gap_phy_type numbering the indication
     * reports. The request field is the enum gap_phy *bitfield*, so shift by
     * phy-1: 1 -> 1 (1M), 2 -> 2 (2M), 3 -> 4 (Coded). */
    if (phy < 1 || phy > 3)
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "phy");
        return -1;
    }

    memset(&param, 0, sizeof(param));
    param.conn_idx = link->conn_idx;
    param.rx_phy = (uint8_t)(1 << (phy - 1));
    param.tx_phy = (uint8_t)(1 << (phy - 1));
    param.phy_opt = 0;

    g_phy_rx = 0;
    g_phy_tx = 0;
    g_phy_conn = link->conn_idx;
    peer_ble_clear_bits(PEER_BIT_PHY_DONE);

    if (ble_gap_update_phy(&param) != HL_ERR_NO_ERROR)
    {
        g_phy_conn = PEER_NO_CONN;
        peer_reply_err(reply, PEER_ERR_STATE, "phy");
        return -1;
    }

    if (peer_ble_wait(PEER_BIT_PHY_DONE, 3000) != RT_EOK)
    {
        g_phy_conn = PEER_NO_CONN;
        peer_reply_err(reply, PEER_ERR_TIMEOUT, "phy");
        return -1;
    }

    peer_reply_ok(reply, "link=%u rx_phy=%u tx_phy=%u",
                  link_id, g_phy_rx, g_phy_tx);
    return 0;
}

int peer_ble_update_data_len(uint8_t link_id, uint16_t tx_octets,
                             uint16_t tx_time, peer_reply_t *reply)
{
    peer_link_t *link = peer_link_find(link_id);
    ble_gap_update_data_len_t param;

    if (link == RT_NULL)
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "link");
        return -1;
    }
    if (tx_octets < 27 || tx_octets > 251 || tx_time > 2120)
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "tx_octets");
        return -1;
    }

    memset(&param, 0, sizeof(param));
    param.conn_idx = link->conn_idx;
    param.tx_octets = tx_octets;
    param.tx_time = tx_time;

    g_data_len_status = 0xFF;
    g_data_len_conn = link->conn_idx;
    peer_ble_clear_bits(PEER_BIT_DATA_LEN_DONE);

    if (ble_gap_update_data_len(&param) != HL_ERR_NO_ERROR)
    {
        g_data_len_conn = PEER_NO_CONN;
        peer_reply_err(reply, PEER_ERR_STATE, "data_len");
        return -1;
    }

    if (peer_ble_wait(PEER_BIT_DATA_LEN_DONE, 3000) != RT_EOK)
    {
        g_data_len_conn = PEER_NO_CONN;
        peer_reply_err(reply, PEER_ERR_TIMEOUT, "data_len");
        return -1;
    }

    if (g_data_len_status != HL_ERR_NO_ERROR)
    {
        peer_reply_err(reply, PEER_ERR_STATE, "data_len");
        return -1;
    }

    peer_reply_ok(reply, "link=%u tx_octets=%u tx_time=%u",
                  link_id, tx_octets, tx_time);
    return 0;
}
