/**
  ******************************************************************************
  * @file   receiver.c
  * @author Sifli software development team
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2024 - 2024,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
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
#define DEFAULT_SCAN_SLOW_INTERVAL 300 // miniseconds
#define DEFAULT_SCAN_FAST_INTERVAL 100 // miniseconds


typedef enum
{
    APP_RECV_STATE_IDLE,
    APP_RECV_STATE_SYNC_START,
    APP_RECV_STATE_SYNCING,
    APP_RECV_STATE_SYNCED,
} app_recv_state_t;


uint8_t ble_app_scan_enable(void)
{
    uint8_t ret = 1;
    app_env_t *env = ble_app_get_env();
    LOG_D("scan enable %d", env->r_env.is_scaning);
    if (env->r_env.is_scaning == 0)
    {
        uint8_t is_slow = (env->r_env.state == APP_RECV_STATE_SYNCED) || ble_app_sender_is_working();
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
        };
        ret = ble_gap_scan_start(&scan_param);
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
    ble_gap_create_periodic_advertising_sync();
}

static void ble_app_start_per_adv_sync(ble_gap_addr_t *addr, uint8_t adv_sid, uint16_t sync_to)
{
    app_env_t *env = ble_app_get_env();
    if (env->r_env.sync_idx != INVALID_SYNC_IDX)
    {
        ble_gap_periodic_advertising_sync_start_t sync_param =
        {
            .actv_idx = env->r_env.sync_idx,
            .addr = *addr,
            .skip = 0,
            .sync_to = sync_to,
            .type = GAP_PER_SYNC_TYPE_GENERAL,
            .adv_sid = adv_sid,
        };

        ble_gap_start_periodic_advertising_sync(&sync_param);
    }

}

static void ble_app_stop_per_adv_sync(void)
{
    app_env_t *env = ble_app_get_env();
    if (env->r_env.sync_idx != INVALID_SYNC_IDX)
    {
        ble_gap_eriodic_advertising_sync_stop_t stop_param;
        stop_param.actv_idx = env->r_env.sync_idx;
        ble_gap_stop_periodic_advertising_sync(&stop_param);
    }

}

static void ble_app_receiver_data_parser(ble_gap_ext_adv_report_ind_t *ind)
{
    // ind->data is audio data
    //LOG_HEX("per data", 16, ind->data, ind->length);
    ble_talk_downlink(ind->data, ind->length);
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
            if (env->r_env.sync_idx != INVALID_SYNC_IDX)
            {
                LOG_D("sync start");
                ble_app_start_per_adv_sync(&ind->addr, ind->adv_sid, DEFAULT_SYNC_TO / 10);
                env->r_env.state = APP_RECV_STATE_SYNC_START;
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
        env->r_env.sync_idx = ind->actv_idx;
        LOG_D("PER_ADV_SYNC created %d", env->r_env.sync_idx);
        break;
    }
    case BLE_GAP_START_PERIODIC_ADV_SYNC_CNF:
    {
        ble_gap_start_per_adv_sync_cnf_t *cnf = (ble_gap_start_per_adv_sync_cnf_t *)data;
        LOG_I("Start PER_ADV_SYNC result %d", cnf->status);
        if (cnf->status == HL_ERR_NO_ERROR)
        {
            env->r_env.state = APP_RECV_STATE_SYNCING;
            rt_work_submit(&env->r_env.work.work, DEFAULT_SYNCING_PERIOD * 1000);
        }
        break;
    }
    case BLE_GAP_PERIODIC_ADV_SYNC_ESTABLISHED_IND:
    {
        ble_gap_per_adv_sync_established_t *ind = (ble_gap_per_adv_sync_established_t *)data;
        LOG_I("PER_ADV_SYNC established(%x-%x-%x-%x-%x-%x)", ind->addr.addr.addr[0], ind->addr.addr.addr[1],
              ind->addr.addr.addr[2], ind->addr.addr.addr[3],
              ind->addr.addr.addr[4], ind->addr.addr.addr[5]);
        env->r_env.synced_dev++;
        if (env->r_env.synced_dev >= MAX_SYNCD_DEVICE)
        {
            ble_gap_scan_stop();
        }
        break;
    }
    case BLE_GAP_PERIODIC_ADV_SYNC_STOPPED_IND:
    {
        ble_gap_per_adv_sync_stopped_ind_t *ind = (ble_gap_per_adv_sync_stopped_ind_t *)data;
        LOG_D("PER_ADV_ SYNC stopped %d, total %d, lost %d", env->r_env.sync_idx);
        if (env->r_env.synced_dev)
            env->r_env.synced_dev--;

        if (env->r_env.synced_dev < MAX_SYNCD_DEVICE
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
        env->r_env.sync_idx = INVALID_SYNC_IDX;
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
        ble_app_stop_per_adv_sync();
    }
}



void ble_app_receviver_init(void)
{
    app_env_t *env = ble_app_get_env();
    env->r_env.sync_idx = INVALID_SYNC_IDX;
    rt_delayed_work_init(&env->r_env.work, ble_app_per_sync_check, env);

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
                hex2data(argv[3], peer_addr.addr.addr, BD_ADDR_LEN);
                LOG_HEX("enter addr", 16, peer_addr.addr.addr, BD_ADDR_LEN);
                peer_addr.addr_type = atoi(argv[4]);
                uint8_t adv_sid = atoi(argv[5]);
                uint16_t sync_to = atoi(argv[6]);
                ble_app_start_per_adv_sync(&peer_addr, adv_sid, sync_to);
            }
            else if (strcmp(argv[2], "stop") == 0)
            {
                ble_app_stop_per_adv_sync();
            }
        }
    }

    return 0;
}

#ifdef RT_USING_FINSH
    MSH_CMD_EXPORT(recv, receiver commander);
#endif


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
