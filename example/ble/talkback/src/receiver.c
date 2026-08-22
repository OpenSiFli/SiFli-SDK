/*
 * SPDX-FileCopyrightText: 2024-2024 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>
#include <stdlib.h>

#include "bf0_ble_gap.h"
#include "bf0_sibles.h"
#include "bf0_sibles_internal.h"
#include "bf0_sibles_advertising.h"

#include "main.h"

#define DEFAULT_SYNC_TO 800
#define INVALID_SYNC_IDX 0xFF
#define DEFAULT_SYNCING_PERIOD 10 // seconds
#define DEFAULT_SCAN_WIN 30 // miniseconds
#define DEFAULT_SCAN_SLOW_INTERVAL 100 // miniseconds
#define DEFAULT_SCAN_FAST_INTERVAL 60 // miniseconds

typedef enum
{
    APP_RECV_DEV_STATE_IDLE,
    APP_RECV_DEV_STATE_CREATED,
    APP_RECV_DEV_STATE_SYNCING,
    APP_RECV_DEV_STATE_SYNCED,
} app_recv_dev_state_t;

typedef enum
{
    APP_RECV_STATE_IDLE,
    APP_RECV_STATE_SYNCING,
} app_recv_state_t;

uint8_t ble_app_scan_enable(void)
{
    uint8_t ret = 1;
    app_env_t *env = ble_app_get_env();
    LOG_D("scan enable %d", env->r_env.is_scaning);
    if (env->r_env.is_scaning == 0)
    {
        uint8_t is_slow = (env->r_env.synced_dev_num != 0 || ble_app_sender_is_working());
        uint16_t inv = is_slow ? DEFAULT_SCAN_SLOW_INTERVAL : DEFAULT_SCAN_FAST_INTERVAL;
        ble_gap_scan_start_t scan_param =
        {
            .own_addr_type = GAPM_STATIC_ADDR,
            .type = GAPM_SCAN_TYPE_OBSERVER,
            .dup_filt_pol = 0,
            .scan_param_1m.scan_intv = inv * 8 / 5,
            .scan_param_1m.scan_wd = DEFAULT_SCAN_WIN * 8 / 5,
            .duration = 0,
            .period = 0,
            .prop = GAPM_SCAN_PROP_PHY_1M_BIT,
        };
        ret = ble_gap_scan_start_ex(&scan_param);
    }

    return ret;

}

uint8_t ble_app_scan_stop(void)
{
    app_env_t *env = ble_app_get_env();
    uint8_t ret = 0;
    if (env->r_env.is_scaning)
        ret = ble_gap_scan_stop();
    return ret;
}

uint8_t ble_app_scan_restart(void)
{
    LOG_D("scan restart");
    app_env_t *env = ble_app_get_env();
    if (env->r_env.is_scaning)
    {
        env->r_env.is_scan_restart = 1;
        ble_gap_scan_stop();
    }
    return 0;
}

static void ble_app_create_per_adv_sync(void)
{
    app_env_t *env = ble_app_get_env();
    for (uint32_t i = 0; i < MAX_SYNCD_DEVICE; i++)
        ble_gap_create_periodic_advertising_sync();
}

static sync_info_t *ble_app_find_idle_sync_dev(app_recv_env_t *r_env)
{
    sync_info_t *dev = NULL;
    for (uint32_t i = 0; i < MAX_SYNCD_DEVICE; i++)
    {
        if (r_env->sync_dev[i].dev_state == APP_RECV_DEV_STATE_IDLE)
        {
            dev = &r_env->sync_dev[i];
            break;
        }
    }

    return dev;
}

static sync_info_t *ble_app_find_available_sync_dev(app_recv_env_t *r_env, ble_gap_addr_t *addr)
{
    sync_info_t *dev = NULL;
    uint8_t is_dup = 0;
    for (uint32_t i = 0; i < MAX_SYNCD_DEVICE; i++)
    {
        if (r_env->sync_dev[i].dev_state == APP_RECV_DEV_STATE_CREATED)
        {
            dev = &r_env->sync_dev[i];
        }

        if ((memcmp(&r_env->sync_dev[i].addr, addr, sizeof(ble_gap_addr_t)) == 0) &&
                (r_env->sync_dev[i].dev_state > APP_RECV_DEV_STATE_CREATED))
        {
            is_dup = 1;
        }
    }

    return is_dup ? NULL : dev;
}

static sync_info_t *ble_app_find_sync_dev_by_idx(app_recv_env_t *r_env, uint8_t idx)
{
    sync_info_t *dev = NULL;
    for (uint32_t i = 0; i < MAX_SYNCD_DEVICE; i++)
    {
        if ((r_env->sync_dev[i].dev_state >= APP_RECV_DEV_STATE_CREATED) &&
                (r_env->sync_dev[i].sync_idx == idx))
        {
            dev = &r_env->sync_dev[i];
            break;
        }
    }

    return dev;
}

static uint8_t ble_app_sync_dev_created(app_recv_env_t *r_env, uint8_t actv_idx)
{
    sync_info_t *dev = NULL;
    // 1 means failed
    uint8_t ret = 1;
    if ((dev = ble_app_find_idle_sync_dev(r_env)) != NULL)
    {
        r_env->sync_created_dev++;
        dev->sync_idx = actv_idx;
        dev->dev_state = APP_RECV_DEV_STATE_CREATED;
        ret = 0;
    }

    return ret;
}

static uint8_t ble_app_sync_dev_synced(app_recv_env_t *r_env, uint8_t actv_idx)
{
    sync_info_t *dev = NULL;
    // 1 means failed
    uint8_t ret = 1;
    if ((dev = ble_app_find_sync_dev_by_idx(r_env, actv_idx)) != NULL)
    {
        r_env->synced_dev_num++;
        r_env->syncing_idx = INVALID_CONN_IDX;
        dev->dev_state = APP_RECV_DEV_STATE_SYNCED;
        ret = 0;
    }

    return ret;
}

static uint8_t ble_app_sync_dev_stopped(app_recv_env_t *r_env, uint8_t actv_idx)
{
    sync_info_t *dev = NULL;
    // 1 means failed
    uint8_t ret = 1;
    if ((dev = ble_app_find_sync_dev_by_idx(r_env, actv_idx)) != NULL)
    {
        if (dev->dev_state == APP_RECV_DEV_STATE_SYNCED)
            r_env->synced_dev_num--;
        else if (dev->dev_state == APP_RECV_DEV_STATE_SYNCING)
            r_env->syncing_idx = INVALID_CONN_IDX;

        memset(&dev->addr, 0, sizeof(dev->addr));
        dev->dev_state = APP_RECV_DEV_STATE_CREATED;
        ret = 0;
    }
    return ret;
}

static uint8_t ble_app_sync_dev_deleted(app_recv_env_t *r_env, uint8_t actv_idx)
{
    sync_info_t *dev = NULL;
    // 1 means failed
    uint8_t ret = 1;
    if ((dev = ble_app_find_sync_dev_by_idx(r_env, actv_idx)) != NULL)
    {
        r_env->sync_created_dev--;
        memset(&dev->addr, 0, sizeof(dev->addr));
        dev->sync_idx = INVALID_SYNC_IDX;
        dev->dev_state = APP_RECV_DEV_STATE_IDLE;
        ret = 0;
    }

    return ret;
}

static uint8_t ble_app_start_per_adv_sync(uint8_t sync_idx, ble_gap_addr_t *addr, uint8_t adv_sid, uint16_t sync_to)
{
    uint8_t ret = GAP_ERR_INVALID_PARAM;
    app_env_t *env = ble_app_get_env();
    if (sync_idx != INVALID_SYNC_IDX)
    {
        ble_gap_periodic_advertising_sync_start_t sync_param =
        {
            .actv_idx = sync_idx,
            .addr = *addr,
            .skip = 0,
            .sync_to = sync_to,
            .type = GAP_PER_SYNC_TYPE_GENERAL,
            .adv_sid = adv_sid,
        };

        ret = ble_gap_start_periodic_advertising_sync(&sync_param);
    }
    return ret;
}

static void ble_app_stop_per_adv_sync(uint8_t sync_idx)
{
    app_env_t *env = ble_app_get_env();
    if (sync_idx != INVALID_SYNC_IDX)
    {
        ble_gap_eriodic_advertising_sync_stop_t stop_param;
        stop_param.actv_idx = sync_idx;
        ble_gap_stop_periodic_advertising_sync(&stop_param);
    }

}

static void ble_app_receiver_data_parser(ble_gap_ext_adv_report_ind_t *ind)
{
    // ind->data is audio data
    //LOG_HEX("per data", 16, ind->data, ind->length);
    ble_talk_downlink(ind->actv_idx, ind->data, ind->length);
}

uint8_t *ble_app_adv_data_found(uint8_t *p_data, uint8_t type, uint16_t *length)
{
    if (!p_data || !length)
        return NULL;

    // Cursor
    uint8_t *p_cursor = p_data;
    // End of data
    uint8_t *p_end_cursor = p_data + *length;

    while (p_cursor < p_end_cursor)
    {
        // Extract AD type
        uint8_t ad_type = *(p_cursor + 1);

        // Check if it's AD Type which shall be unique
        if (ad_type == type)
        {
            *length = *p_cursor - 1;
            break;
        }

        /* Go to next advertising info */
        p_cursor += (*p_cursor + 1);
    }

    return (p_cursor >= p_end_cursor) ? NULL : (p_cursor + 2);
}

static void ble_app_network_parser(ble_gap_ext_adv_report_ind_t *ind)
{
    app_env_t *env = ble_app_get_env();

    // Not connectable/scannable/directable
    if ((ind->info & (~GAPM_REPORT_INFO_REPORT_TYPE_MASK)) != GAPM_REPORT_INFO_COMPLETE_BIT)
        return;

    uint8_t *data = ind->data;
    uint16_t len = ind->length;

    if ((data = ble_app_adv_data_found(data, BLE_GAP_AD_TYPE_MANU_SPECIFIC_DATA, &len)) != NULL)
    {
        uint16_t company = (uint8_t)data[0] | data[1] << 8;
        uint8_t cod_len = data[2];
        // Find target device
        if (strncmp((const char *)&data[3], DEFAULT_NETWORK_CODE, cod_len) == 0)
        {
            sync_info_t *dev;
            if ((env->r_env.state == APP_RECV_STATE_IDLE)
                    && ((dev = ble_app_find_available_sync_dev(&env->r_env, &ind->addr)) != NULL))
            {
                if (ble_app_start_per_adv_sync(dev->sync_idx, &ind->addr, ind->adv_sid, DEFAULT_SYNC_TO / 10) == HL_ERR_NO_ERROR)
                {
                    LOG_D("sync start");
                    env->r_env.state = APP_RECV_STATE_SYNCING;
                    env->r_env.syncing_idx = dev->sync_idx;
                    dev->dev_state = APP_RECV_DEV_STATE_SYNCING;
                    dev->addr = ind->addr;
                }
            }
        }
    }
}

int ble_app_receiver_event_handler(uint16_t event_id, uint8_t *data, uint16_t len, uint32_t context)
{
    app_env_t *env = ble_app_get_env();

    switch (event_id)
    {
    case BLE_GAP_SCAN_START_CNF:
    {
        ble_gap_start_scan_cnf_t *cnf = (ble_gap_start_scan_cnf_t *)data;
        LOG_I("Scan start status %d", cnf->status);
        if (cnf->status == HL_ERR_NO_ERROR)
        {
            env->r_env.is_scaning = 1;
        }
        break;
    }
    case BLE_GAP_SCAN_STOP_CNF:
    {
        ble_gap_stop_scan_cnf_t *cnf = (ble_gap_stop_scan_cnf_t *)data;
        LOG_I("Scan stop");
        if (env->r_env.is_scan_restart)
        {
            env->r_env.is_scan_restart = 0;
            ble_app_scan_enable();
        }
        break;
    }
    case BLE_GAP_SCAN_STOPPED_IND:
    {
        ble_gap_scan_stopped_ind_t *ind = (ble_gap_scan_stopped_ind_t *)data;
        LOG_I("Scan stopped %d", ind->reason);
        env->r_env.is_scaning = 0;

        break;
    }
    case BLE_GAP_EXT_ADV_REPORT_IND:
    {
        ble_gap_ext_adv_report_ind_t *ind = (ble_gap_ext_adv_report_ind_t *)data;
        if ((ind->info & GAPM_REPORT_INFO_REPORT_TYPE_MASK) == GAPM_REPORT_TYPE_PER_ADV)
        {
            // Only for the context of periodic adv is fulfilled duplicated numeric which increased one by one for a new adv.
            ble_app_receiver_data_parser(ind);
        }
        else if ((ind->info & GAPM_REPORT_INFO_REPORT_TYPE_MASK) == GAPM_REPORT_TYPE_ADV_EXT)
        {
            ble_app_network_parser(ind);
        }
        break;
    }
    case BLE_GAP_CREATE_PERIODIC_ADV_SYNC_CNF:
    {
        ble_gap_create_per_adv_sync_cnf_t *cnf = (ble_gap_create_per_adv_sync_cnf_t *)data;
        LOG_I("Create PER_ADV_SYNC result %d", cnf->status);
        break;
    }
    case BLE_GAP_PERIODIC_ADV_SYNC_CREATED_IND:
    {
        ble_gap_per_adv_sync_created_ind_t *ind = (ble_gap_per_adv_sync_created_ind_t *)data;
        sync_info_t *dev = NULL;
        if (ble_app_sync_dev_created(&env->r_env, ind->actv_idx) != 0)
            RT_ASSERT("sync created failed" && 0);

        LOG_D("PER_ADV_SYNC created %d", ind->actv_idx);
        break;
    }
    case BLE_GAP_START_PERIODIC_ADV_SYNC_CNF:
    {
        ble_gap_start_per_adv_sync_cnf_t *cnf = (ble_gap_start_per_adv_sync_cnf_t *)data;
        LOG_I("Start PER_ADV_SYNC result %d", cnf->status);
        if (cnf->status == HL_ERR_NO_ERROR)
        {
            rt_work_submit(&env->r_env.work, DEFAULT_SYNCING_PERIOD * 1000);
        }
        else
        {
            env->r_env.state = APP_RECV_STATE_SYNCING;
        }
        break;
    }
    case BLE_GAP_PERIODIC_ADV_SYNC_ESTABLISHED_IND:
    {
        ble_gap_per_adv_sync_established_t *ind = (ble_gap_per_adv_sync_established_t *)data;
        LOG_I("PER_ADV_SYNC established(%x-%x-%x-%x-%x-%x)", ind->addr.addr.addr[0], ind->addr.addr.addr[1],
              ind->addr.addr.addr[2], ind->addr.addr.addr[3],
              ind->addr.addr.addr[4], ind->addr.addr.addr[5]);

        ble_app_sync_dev_synced(&env->r_env, ind->actv_idx);
        env->r_env.state = APP_RECV_STATE_IDLE;

        if (env->r_env.synced_dev_num >= MAX_SYNCD_DEVICE)
        {
            ble_gap_scan_stop();
        }
        else
        {
            // Adjust scan parameters
            ble_app_scan_restart();
        }
        break;
    }
    case BLE_GAP_STOP_PERIODIC_ADV_SYNC_CNF:
    {
        ble_gap_stop_per_adv_sync_cnf_t *cnf = (ble_gap_stop_per_adv_sync_cnf_t *)data;
        LOG_D("PER_ADV_SYNC stop cnf %d %d", cnf->status, cnf->actv_index);
        break;
    }
    case BLE_GAP_PERIODIC_ADV_SYNC_STOPPED_IND:
    {
        ble_gap_per_adv_sync_stopped_ind_t *ind = (ble_gap_per_adv_sync_stopped_ind_t *)data;
        LOG_D("PER_ADV_ SYNC stopped reason %d, idx %d ", ind->reason, ind->actv_idx);

        ble_app_sync_dev_stopped(&env->r_env, ind->actv_idx);
        env->r_env.state = APP_RECV_STATE_IDLE;

        if (env->r_env.synced_dev_num < MAX_SYNCD_DEVICE
                && !env->r_env.is_scaning)
        {
            ble_app_scan_enable();
        }
        break;
    }
    case BLE_GAP_DELETE_PERIODIC_ADV_SYNC_CNF:
    {
        ble_gap_delete_per_adv_sync_cnf_t *cnf = (ble_gap_delete_per_adv_sync_cnf_t *)data;
        LOG_D("per_adv_sync(%d) deleted %d", cnf->actv_index, cnf->status);
        if (cnf->status == HL_ERR_NO_ERROR)
            ble_app_sync_dev_deleted(&env->r_env, cnf->actv_index);

        break;
    }
    default:
        break;
    }
    return 0;
}

uint8_t ble_app_scan_init()
{
    uint8_t ret = ble_app_scan_enable();
    ble_app_create_per_adv_sync();
    LOG_I("scan enabled %d", ret);
    return 0;
}

static void ble_app_per_sync_check(struct rt_work *work, void *work_data)
{
    app_env_t *env = (app_env_t *)work_data;
    if (env->r_env.state == APP_RECV_STATE_SYNCING)
    {
        ble_app_stop_per_adv_sync(env->r_env.syncing_idx);
    }
}

void ble_app_receviver_init(void)
{
    app_env_t *env = ble_app_get_env();
    rt_work_init(&env->r_env.work, ble_app_per_sync_check, env);

#ifdef APP_MIC_ALWAYS_ON
    talk_init(AUDIO_TXRX);
#else
    talk_init(AUDIO_RX);
#endif
}

int recv(int argc, char *argv[])
{
    app_env_t *env = ble_app_get_env();

    if (argc > 1)
    {
        if (strcmp(argv[1], "scan") == 0)
        {
            if (strcmp(argv[2], "start") == 0)
            {
                ble_gap_scan_start_t scan_param =
                {
                    .own_addr_type = GAPM_STATIC_ADDR,
                    .type = GAPM_SCAN_TYPE_OBSERVER,
                    .dup_filt_pol = atoi(argv[3]),
                    .scan_param_1m.scan_intv = atoi(argv[4]) * 8 / 5,
                    .scan_param_1m.scan_wd = atoi(argv[5]) * 8 / 5,
                    .duration = atoi(argv[6]) / 10,
                    .period = 0,
                };

                uint8_t ret = ble_gap_scan_start(&scan_param);
                LOG_D("scan start %d", ret);
            }
            else if (strcmp(argv[2], "stop") == 0)
            {
                ble_gap_scan_stop();
            }
            else
            {
                LOG_I("Scan start: diss scan start [dup, 0/1] [interval, ms] [window, ms] [duration, ms]");
                LOG_I("Scan stop: diss scan stop");
            }
        }
        else if (strcmp(argv[1], "sync") == 0)
        {
            if (strcmp(argv[2], "create") == 0)
            {
                ble_app_create_per_adv_sync();
            }
            else if (strcmp(argv[2], "start") == 0)
            {
                ble_gap_addr_t peer_addr;
                uint8_t sync_idx = atoi(argv[3]);
                hex2data(argv[4], peer_addr.addr.addr, BD_ADDR_LEN);
                LOG_HEX("enter addr", 16, peer_addr.addr.addr, BD_ADDR_LEN);
                peer_addr.addr_type = atoi(argv[5]);
                uint8_t adv_sid = atoi(argv[6]);
                uint16_t sync_to = atoi(argv[7]);
                sync_info_t *dev = ble_app_find_sync_dev_by_idx(&env->r_env, sync_idx);
                if (dev->dev_state == APP_RECV_DEV_STATE_CREATED)
                    ble_app_start_per_adv_sync(sync_idx, &peer_addr, adv_sid, sync_to);
                else
                    LOG_W("Enter wrongly idx, dev status is %d", dev->dev_state);
            }
            else if (strcmp(argv[2], "stop") == 0)
            {
                uint8_t sync_idx = atoi(argv[3]);
                sync_info_t *dev = ble_app_find_sync_dev_by_idx(&env->r_env, sync_idx);
                if (dev->dev_state == APP_RECV_DEV_STATE_SYNCING || dev->dev_state == APP_RECV_DEV_STATE_SYNCED)
                    ble_app_stop_per_adv_sync(sync_idx);
                else
                    LOG_W("Enter wrongly idx, dev status is %d", dev->dev_state);
            }
        }
    }

    return 0;
}

#ifdef RT_USING_FINSH
    MSH_CMD_EXPORT(recv, receiver commander);
#endif

