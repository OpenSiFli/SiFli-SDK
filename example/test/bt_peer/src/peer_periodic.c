/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Periodic advertising for the bt_peer instrument.
 *
 * `periodic.adv.start` runs a periodic advertiser with host-supplied data.
 * `periodic.sync` synchronises to a periodic advertiser found by `gap.scan`
 * and reports every periodic report as an event.
 */

#include <rtthread.h>
#include <string.h>

#include "peer.h"
#include "bf0_sibles.h"
#include "bf0_sibles_advertising.h"

/** Create/establish budget for a periodic sync. */
#define PEER_PERIODIC_CREATE_MS     3000
#define PEER_PERIODIC_SYNC_MS       15000
/** A scan started only to establish a sync lives this long. */
#define PEER_PERIODIC_SCAN_MS      (PEER_PERIODIC_SYNC_MS + 5000)
/** Sync timeout passed to the controller, in 10 ms units. */
#define PEER_PERIODIC_SYNC_TO       1000
/** Periodic advertising interval must exceed 20 ms, in 1.25 ms units. */
#define PEER_PERIODIC_MIN_INTV      16
/** Activities are 0-based; 0xFF means none. */
#define PEER_PERIODIC_NO_ACTV       0xFF

SIBLES_ADVERTISING_CONTEXT_DECLAR(g_periodic_context);

static uint8_t g_periodic_on;
/** Status the last ADV_STARTED callback carried; 0 means started. */
static int g_periodic_start_status;
static uint8_t g_sync_idx = PEER_PERIODIC_NO_ACTV;
/** Last periodic payload reported, so repeats are not re-sent. */
static uint8_t g_last_per[PEER_ADV_DATA_MAX];
static uint16_t g_last_per_len;
static int8_t g_sync_result;

/** Primary advertising interval, in 0.625 ms units. */
static uint16_t peer_periodic_intv(uint32_t interval_ms)
{
    uint32_t intv = interval_ms * 8 / 5;

    if (intv < PEER_PERIODIC_MIN_INTV)
        intv = PEER_PERIODIC_MIN_INTV;
    if (intv > 0xFFFF)
        intv = 0xFFFF;

    return (uint16_t)intv;
}

/**
 * Periodic advertising interval, in 1.25 ms units.
 *
 * The SDK maps `periodic_config.interval` to the *primary* channel interval
 * and `adv_intv_min/max` to the periodic interval; using the same value for
 * both made the periodic rate half the requested one.
 */
static uint16_t peer_periodic_adv_intv(uint32_t interval_ms)
{
    uint32_t intv = interval_ms * 4 / 5;

    if (intv < PEER_PERIODIC_MIN_INTV / 2)
        intv = PEER_PERIODIC_MIN_INTV / 2;
    if (intv > 0xFFFF)
        intv = 0xFFFF;

    return (uint16_t)intv;
}

static uint8_t peer_periodic_adv_event(uint8_t event, void *context, void *data)
{
    (void)context;

    if (event == SIBLES_ADV_EVT_ADV_STARTED)
    {
        sibles_adv_evt_startted_t *started = (sibles_adv_evt_startted_t *)data;

        g_periodic_start_status = started != RT_NULL ? (int)started->status : -1;
        g_periodic_on = g_periodic_start_status == 0;
        peer_ble_signal(PEER_BIT_PERIODIC_STARTED);
    }
    else if (event == SIBLES_ADV_EVT_ADV_STOPPED)
    {
        g_periodic_on = 0;
        peer_ble_signal(PEER_BIT_PERIODIC_STOPPED);
    }

    return 0;
}

void peer_periodic_init(void)
{
    g_periodic_on = 0;
    g_sync_idx = PEER_PERIODIC_NO_ACTV;
    g_sync_result = -1;
}

void peer_periodic_reset(void)
{
    if (g_periodic_on)
    {
        /*
         * The stop is asynchronous: a start issued before the controller's
         * stopped indication is refused with GAP_ERR_COMMAND_DISALLOWED, which
         * is exactly what the next suite's `periodic.adv.start` would hit.
         */
        peer_ble_clear_bits(PEER_BIT_PERIODIC_STOPPED);
        sibles_advertising_stop(g_periodic_context);
        peer_ble_wait(PEER_BIT_PERIODIC_STOPPED, 1000);
        g_periodic_on = 0;
    }

    if (g_sync_idx != PEER_PERIODIC_NO_ACTV)
    {
        ble_gap_eriodic_advertising_sync_stop_t stop;
        ble_gap_eriodic_advertising_sync_delete_t del;

        stop.actv_idx = g_sync_idx;
        ble_gap_stop_periodic_advertising_sync(&stop);

        del.actv_idx = g_sync_idx;
        ble_gap_delete_periodic_advertising_sync(&del);

        g_sync_idx = PEER_PERIODIC_NO_ACTV;
    }

    peer_ble_clear_bits(PEER_BIT_PERIODIC_CREATED | PEER_BIT_PERIODIC_SYNC |
                        PEER_BIT_PERIODIC_FAIL | PEER_BIT_PERIODIC_STOPPED);
}

int peer_periodic_adv_start(uint32_t interval_ms, const uint8_t *data,
                            uint16_t len, peer_reply_t *reply)
{
    sibles_advertising_para_t para;
    uint16_t intv;
    uint16_t adv_intv;
    uint8_t ret;

    if (!peer_ble_ready())
    {
        peer_reply_err(reply, PEER_ERR_NOT_READY, "ble");
        return -1;
    }

    if (g_periodic_on)
    {
        peer_reply_err(reply, PEER_ERR_STATE, "periodic_adv_running");
        return -1;
    }

    if (len > PEER_ADV_DATA_MAX)
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "len");
        return -1;
    }

    intv = peer_periodic_intv(interval_ms);
    adv_intv = peer_periodic_adv_intv(interval_ms);

    memset(&para, 0, sizeof(para));
    para.own_addr_type = GAPM_STATIC_ADDR;
    para.config.adv_mode = SIBLES_ADV_PERIODIC_MODE;
    para.config.mode_config.periodic_config.duration = 0;
    para.config.mode_config.periodic_config.interval = intv;
    para.config.mode_config.periodic_config.max_skip = 0;
    para.config.mode_config.periodic_config.phy = GAP_PHY_TYPE_LE_1M;
    para.config.mode_config.periodic_config.adv_sid = 0;
    para.config.mode_config.periodic_config.connectable_enable = 0;
    para.config.mode_config.periodic_config.adv_intv_min = adv_intv;
    para.config.mode_config.periodic_config.adv_intv_max = adv_intv;
    para.config.max_tx_pwr = 0x7F;
    para.config.is_auto_restart = 0;
    para.evt_handler = peer_periodic_adv_event;

    /*
     * The primary advertisement that carries the sync info is not empty in the
     * reference example (name and manufacturer data). An empty set was refused
     * by the controller with GAP_ERR_COMMAND_DISALLOWED, so carry the instrument
     * name: a scan can then attribute the advertisement as well.
     */
    {
        const char *name = peer_ble_local_name();
        uint8_t name_len = (uint8_t)strlen(name);

        if (name_len > 0 && name_len <= 0xFE)
        {
            para.adv_data.customized_data =
                rt_malloc(sizeof(sibles_adv_type_customize_t) + 2 + name_len);
            if (para.adv_data.customized_data == RT_NULL)
            {
                peer_reply_err(reply, PEER_ERR_NO_MEM, "adv_data");
                return -1;
            }
            para.adv_data.customized_data->len = (uint8_t)(2 + name_len);
            para.adv_data.customized_data->data[0] = (uint8_t)(1 + name_len);
            para.adv_data.customized_data->data[1] = 0x09; /* complete name */
            memcpy(&para.adv_data.customized_data->data[2], name, name_len);
        }
    }

    /*
     * Always allocate, even for an empty payload: sibles_advertising_init()
     * dereferences periodic_data unconditionally in periodic mode, and the
     * protocol explicitly allows len=0.
     */
    para.periodic_data = rt_malloc(sizeof(sibles_periodic_adv_t) + len);
    if (para.periodic_data == RT_NULL)
    {
        rt_free(para.adv_data.customized_data);
        peer_reply_err(reply, PEER_ERR_NO_MEM, "periodic_data");
        return -1;
    }
    para.periodic_data->len = len;
    if (len > 0)
        memcpy(para.periodic_data->data, data, len);

    ret = sibles_advertising_init(g_periodic_context, &para);
    if (ret == SIBLES_ADV_NO_ERR)
    {
        peer_ble_clear_bits(PEER_BIT_PERIODIC_STARTED);
        ret = sibles_advertising_start(g_periodic_context);
    }

    rt_free(para.periodic_data);
    rt_free(para.adv_data.customized_data);

    if (ret != SIBLES_ADV_NO_ERR)
    {
        peer_reply_err(reply, PEER_ERR_STATE, "periodic_adv_start");
        return -1;
    }

    /* As in `adv.start`: the controller's CNF is what says the set is on air. */
    if (peer_ble_wait(PEER_BIT_PERIODIC_STARTED, 2000) != RT_EOK)
    {
        sibles_advertising_stop(g_periodic_context);
        peer_reply_err(reply, PEER_ERR_TIMEOUT, "periodic_adv_start");
        return -1;
    }

    if (g_periodic_start_status != 0)
    {
        sibles_advertising_stop(g_periodic_context);
        peer_reply_err(reply, PEER_ERR_STATE, "periodic_adv_start rc=%d",
                       g_periodic_start_status);
        return -1;
    }

    g_periodic_on = 1;
    peer_reply_ok(reply, "interval_ms=%u", (unsigned)interval_ms);
    return 0;
}

int peer_periodic_adv_stop(peer_reply_t *reply)
{
    if (!g_periodic_on)
    {
        peer_reply_ok(reply, "");
        return 0;
    }

    peer_ble_clear_bits(PEER_BIT_PERIODIC_STOPPED);
    if (sibles_advertising_stop(g_periodic_context) != SIBLES_ADV_NO_ERR)
    {
        peer_reply_err(reply, PEER_ERR_STATE, "periodic_adv_stop");
        return -1;
    }

    /* Answer only once the controller agrees: an immediate restart would be
     * refused with GAP_ERR_COMMAND_DISALLOWED. */
    if (peer_ble_wait(PEER_BIT_PERIODIC_STOPPED, 1000) != RT_EOK)
    {
        peer_reply_err(reply, PEER_ERR_TIMEOUT, "periodic_adv_stop");
        return -1;
    }

    g_periodic_on = 0;
    peer_reply_ok(reply, "");
    return 0;
}

int peer_periodic_sync(const uint8_t *addr, uint8_t addr_type,
                       uint8_t adv_sid, peer_reply_t *reply)
{
    ble_gap_periodic_advertising_sync_start_t sync;
    uint8_t started_scan = 0;
    uint8_t ret;

    if (!peer_ble_ready())
    {
        peer_reply_err(reply, PEER_ERR_NOT_READY, "ble");
        return -1;
    }

    if (g_sync_idx != PEER_PERIODIC_NO_ACTV)
    {
        peer_reply_err(reply, PEER_ERR_BUSY, "sync");
        return -1;
    }

    /*
     * Create the sync activity before the scan: creating it while a scan is
     * running makes the controller stop that scan, and the sync then has
     * nothing to listen with. The order is the one the periodic_adv_sync
     * example uses (create, scan, start).
     */
    g_sync_result = -1;
    g_last_per_len = 0;
    peer_ble_clear_bits(PEER_BIT_PERIODIC_CREATED | PEER_BIT_PERIODIC_SYNC |
                        PEER_BIT_PERIODIC_FAIL | PEER_BIT_PERIODIC_STOPPED);

    ret = ble_gap_create_periodic_advertising_sync();
    if (ret != HL_ERR_NO_ERROR)
    {
        peer_reply_err(reply, PEER_ERR_STATE, "sync_create");
        goto cleanup;
    }

    if (peer_ble_wait(PEER_BIT_PERIODIC_CREATED, PEER_PERIODIC_CREATE_MS) != RT_EOK)
    {
        peer_reply_err(reply, PEER_ERR_TIMEOUT, "sync_create");
        goto cleanup;
    }

    if (g_sync_result != 0 || g_sync_idx == PEER_PERIODIC_NO_ACTV)
    {
        peer_reply_err(reply, PEER_ERR_STATE, "sync_create");
        goto cleanup;
    }

    /*
     * The controller finds the periodic channel through a running scan: the
     * sync procedure listens for the advertisement that carries the sync info.
     * A caller that is not scanning gets one started for it, and it is stopped
     * again once the sync is established (or has failed).
     */
    if (!peer_ble_scan_active())
    {
        if (peer_ble_scan_begin(PEER_PERIODIC_SCAN_MS, 100, 50, 1, -127,
                                reply) < 0)
            goto cleanup;
        started_scan = 1;
    }

    memset(&sync, 0, sizeof(sync));
    sync.actv_idx = g_sync_idx;
    sync.type = GAP_PER_SYNC_TYPE_GENERAL;
    sync.skip = 0;
    sync.sync_to = PEER_PERIODIC_SYNC_TO;

    sync.addr.addr_type = addr_type;
    memcpy(sync.addr.addr.addr, addr, BD_ADDR_LEN);
    sync.adv_sid = adv_sid;

    ret = ble_gap_start_periodic_advertising_sync(&sync);
    if (ret != HL_ERR_NO_ERROR)
    {
        peer_reply_err(reply, PEER_ERR_STATE, "sync_start");
        goto cleanup;
    }

    if (peer_ble_wait(PEER_BIT_PERIODIC_SYNC | PEER_BIT_PERIODIC_FAIL,
                      PEER_PERIODIC_SYNC_MS) != RT_EOK)
    {
        peer_reply_err(reply, PEER_ERR_TIMEOUT, "periodic.sync");
        goto cleanup;
    }

    if (g_sync_result != 0)
    {
        peer_reply_err(reply, PEER_ERR_STATE, "sync_failed");
        goto cleanup;
    }

    if (started_scan)
        peer_ble_scan_end();

    peer_reply_ok(reply, "link=%u", g_sync_idx);
    return 0;

cleanup:
    if (started_scan)
        peer_ble_scan_end();

    /* A created activity that never established must not strand the next
     * sync behind `@err busy`. */
    if (g_sync_idx != PEER_PERIODIC_NO_ACTV)
    {
        ble_gap_eriodic_advertising_sync_stop_t stop;
        ble_gap_eriodic_advertising_sync_delete_t del;

        stop.actv_idx = g_sync_idx;
        del.actv_idx = g_sync_idx;
        ble_gap_stop_periodic_advertising_sync(&stop);
        ble_gap_delete_periodic_advertising_sync(&del);
        g_sync_idx = PEER_PERIODIC_NO_ACTV;
    }
    return -1;
}

int peer_periodic_sync_stop(peer_reply_t *reply)
{
    ble_gap_eriodic_advertising_sync_stop_t stop;
    ble_gap_eriodic_advertising_sync_delete_t del;
    /* The STOPPED indication clears g_sync_idx; capture it before issuing the
     * stop, or the delete is sent for the sentinel instead of the activity. */
    uint8_t actv = g_sync_idx;

    if (actv == PEER_PERIODIC_NO_ACTV)
    {
        peer_reply_ok(reply, "");
        return 0;
    }

    stop.actv_idx = actv;
    peer_ble_clear_bits(PEER_BIT_PERIODIC_STOPPED);
    ble_gap_stop_periodic_advertising_sync(&stop);
    peer_ble_wait(PEER_BIT_PERIODIC_STOPPED, PEER_PERIODIC_CREATE_MS);

    del.actv_idx = actv;
    ble_gap_delete_periodic_advertising_sync(&del);

    g_sync_idx = PEER_PERIODIC_NO_ACTV;
    peer_reply_ok(reply, "");
    return 0;
}

static int peer_periodic_event_handler(uint16_t event_id, uint8_t *data,
                                       uint16_t len, uint32_t context)
{
    (void)len;
    (void)context;

    switch (event_id)
    {
    case BLE_GAP_PERIODIC_ADV_SYNC_CREATED_IND:
    {
        ble_gap_per_adv_sync_created_ind_t *ind =
            (ble_gap_per_adv_sync_created_ind_t *)data;

        g_sync_idx = ind->actv_idx;
        g_sync_result = 0;
        peer_ble_signal(PEER_BIT_PERIODIC_CREATED);
        break;
    }

    case BLE_GAP_CREATE_PERIODIC_ADV_SYNC_CNF:
    {
        ble_gap_create_per_adv_sync_cnf_t *cnf =
            (ble_gap_create_per_adv_sync_cnf_t *)data;

        /* Fail the wait immediately instead of burning the full timeout; the
         * success path still waits for the CREATED indication to get actv. */
        if (cnf->status != HL_ERR_NO_ERROR)
        {
            g_sync_result = -1;
            peer_ble_signal(PEER_BIT_PERIODIC_CREATED);
        }
        break;
    }

    case BLE_GAP_START_PERIODIC_ADV_SYNC_CNF:
    {
        ble_gap_start_per_adv_sync_cnf_t *cnf =
            (ble_gap_start_per_adv_sync_cnf_t *)data;

        if (cnf->status != HL_ERR_NO_ERROR)
        {
            g_sync_result = -1;
            peer_ble_signal(PEER_BIT_PERIODIC_FAIL);
        }
        break;
    }

    case BLE_GAP_PERIODIC_ADV_SYNC_ESTABLISHED_IND:
    {
        g_sync_result = 0;
        peer_ble_signal(PEER_BIT_PERIODIC_SYNC);
        break;
    }

    case BLE_GAP_PERIODIC_ADV_SYNC_STOPPED_IND:
    {
        g_sync_idx = PEER_PERIODIC_NO_ACTV;
        peer_ble_signal(PEER_BIT_PERIODIC_STOPPED);
        break;
    }

    case BLE_GAP_DELETE_PERIODIC_ADV_SYNC_CNF:
    {
        ble_gap_delete_per_adv_sync_cnf_t *cnf =
            (ble_gap_delete_per_adv_sync_cnf_t *)data;

        if (cnf->actv_index == g_sync_idx)
            g_sync_idx = PEER_PERIODIC_NO_ACTV;
        break;
    }

    case BLE_GAP_EXT_ADV_REPORT_IND:
    {
        ble_gap_ext_adv_report_ind_t *ind = (ble_gap_ext_adv_report_ind_t *)data;
        char payload[PEER_ADV_DATA_MAX * 2 + 32];
        uint16_t copy;

        if ((ind->info & GAPM_REPORT_INFO_REPORT_TYPE_MASK) != GAPM_REPORT_TYPE_PER_ADV)
            break;

        copy = ind->length;
        if (copy > PEER_ADV_DATA_MAX)
            copy = PEER_ADV_DATA_MAX;

        /*
         * The periodic channel repeats the same payload many times a second
         * (every 20 ms in the examples), and a host that reads one line at a
         * time drowns in it. Report a payload when it changes: that is what a
         * suite can assert on, and it keeps the console usable for everything
         * else. A caller that needs cadence has to measure it on the DUT side.
         */
        if (copy == g_last_per_len &&
                memcmp(g_last_per, ind->data, copy) == 0)
            break;

        memcpy(g_last_per, ind->data, copy);
        g_last_per_len = copy;

        peer_format_payload(payload, sizeof(payload), ind->data, copy);
        peer_emit_evt("periodic", "link=%u %s", ind->actv_idx, payload);
        break;
    }

    default:
        break;
    }

    return 0;
}

BLE_EVENT_REGISTER(peer_periodic_event_handler, NULL);
