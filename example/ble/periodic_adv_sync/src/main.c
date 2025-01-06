/**
  ******************************************************************************
  * @file   main.c
  * @author Sifli software development team
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2021 - 2021,  Sifli Technology
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

#include "bf0_ble_gap.h"
#include "bf0_sibles.h"
#include "bf0_sibles_internal.h"

#define LOG_TAG "ble_app"
#include "log.h"

#define BLE_SYNC_DEBUG_TEST

typedef struct
{
    uint8_t is_power_on;
    uint8_t sync_idx;
#ifdef BLE_SYNC_DEBUG_TEST
    uint8_t sync_detect;
#endif
    rt_mailbox_t mb_handle;
} app_env_t;

static app_env_t g_app_env;
static rt_mailbox_t g_app_mb;

static app_env_t *ble_app_get_env(void)
{
    return &g_app_env;
}

#ifdef ULOG_USING_FILTER
static void app_log_filter_set(void)
{
    ulog_tag_lvl_filter_set("BLE_GAP", LOG_LVL_WARNING);

    ulog_tag_lvl_filter_set("sibles", LOG_LVL_WARNING);
}
#endif // ULOG_USING_FILTER

int main(void)
{
    int count = 0;
    app_env_t *env = ble_app_get_env();
    env->mb_handle = rt_mb_create("app", 8, RT_IPC_FLAG_FIFO);
    sifli_ble_enable();
#ifdef ULOG_USING_FILTER
    app_log_filter_set();
#endif // ULOG_USING_FILTER

#ifdef BLE_SYNC_DEBUG_TEST
    env->sync_detect = 1;
#endif //BLE_SYNC_DEBUG_TEST
    while (1)
    {
        uint32_t value;
        int ret;
        rt_mb_recv(env->mb_handle, (rt_uint32_t *)&value, RT_WAITING_FOREVER);
        if (value == BLE_POWER_ON_IND)
        {
            env->is_power_on = 1;
            LOG_I("receive BLE power on!\r\n");
        }
    }
    return RT_EOK;
}


static void ble_app_create_per_adv_sync(void)
{
    app_env_t *env = ble_app_get_env();
    ble_gap_create_periodic_advertising_sync();
}

static void ble_app_start_per_adv_sync(ble_gap_addr_t *addr, uint8_t adv_sid, uint16_t sync_to)
{
    app_env_t *env = ble_app_get_env();
    if (env->sync_idx != 0xFF)
    {
        ble_gap_periodic_advertising_sync_start_t sync_param =
        {
            .actv_idx = env->sync_idx,
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
    if (env->sync_idx != 0xFF)
    {
        ble_gap_eriodic_advertising_sync_stop_t stop_param;
        stop_param.actv_idx = env->sync_idx;
        ble_gap_stop_periodic_advertising_sync(&stop_param);
    }

}


int ble_app_event_handler(uint16_t event_id, uint8_t *data, uint16_t len, uint32_t context)
{
    app_env_t *env = ble_app_get_env();
#ifdef BLE_SYNC_DEBUG_TEST
    static uint8_t sync_begin = 0;
    static uint32_t lost = 0, total = 0;
#endif //BLE_SYNC_DEBUG_TEST
    switch (event_id)
    {
    case BLE_POWER_ON_IND:
    {
        /* Handle in own thread to avoid conflict */
        if (env->mb_handle)
            rt_mb_send(env->mb_handle, BLE_POWER_ON_IND);
        break;
    }
    case BLE_GAP_SCAN_START_CNF:
    {
        ble_gap_start_scan_cnf_t *cnf = (ble_gap_start_scan_cnf_t *)data;
        LOG_I("Scan start status %d", cnf->status);
        break;
    }
    case BLE_GAP_SCAN_STOPPED_IND:
    {
        ble_gap_scan_stopped_ind_t *ind = (ble_gap_scan_stopped_ind_t *)data;
        LOG_I("Scan stopped %d", ind->reason);
        break;
    }
    case BLE_GAP_EXT_ADV_REPORT_IND:
    {
        ble_gap_ext_adv_report_ind_t *ind = (ble_gap_ext_adv_report_ind_t *)data;
        if ((ind->info & GAPM_REPORT_INFO_REPORT_TYPE_MASK) == GAPM_REPORT_TYPE_PER_ADV)
        {
            // Only for the context of periodic adv is fulfilled duplicated numeric which increased one by one for a new adv.
#ifdef BLE_SYNC_DEBUG_TEST
            static uint8_t chk_data;

            if (sync_begin == 0)
            {
                chk_data = ind->data[0];
            }

            if (sync_begin == 0)
                sync_begin = 1;

            if (chk_data != ind->data[0])
                total++;

            if (sync_begin == 1)
            {
                if ((ind->data[0] - chk_data) > 1)
                {
                    total += ind->data[0] - chk_data - 1;
                    if ((ind->data[0] - chk_data) > env->sync_detect)
                    {
                        lost += ind->data[0] - chk_data - env->sync_detect;
                        LOG_D("lost %d limit %d", ind->data[0] - chk_data - 1, env->sync_detect);
                    }
                }
            }

            chk_data = ind->data[0];
            LOG_D("per_adv_data %d", ind->data[0]);
#else // !BLE_SYNC_DEBUG_TEST
            LOG_HEX("per_adv_data", 16, ind->data, ind->length);
#endif // BLE_SYNC_DEBUG_TEST
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
        env->sync_idx = ind->actv_idx;
        LOG_D("PER_ADV_SYNC created %d", env->sync_idx);
        break;
    }
    case BLE_GAP_PERIODIC_ADV_SYNC_STOPPED_IND:
    {
        ble_gap_per_adv_sync_stopped_ind_t *ind = (ble_gap_per_adv_sync_stopped_ind_t *)data;
        LOG_D("PER_ADV_ SYNC stopped %d, total %d, lost %d", env->sync_idx, total, lost);
#ifdef BLE_SYNC_DEBUG_TEST
        sync_begin = 0;
        lost = 0;
        total = 0;
#endif //BLE_SYNC_DEBUG_TEST
        break;
    }
    case BLE_GAP_DELETE_PERIODIC_ADV_SYNC_CNF:
    {
        ble_gap_delete_per_adv_sync_cnf_t *cnf = (ble_gap_delete_per_adv_sync_cnf_t *)data;
        LOG_D("per_adv_sync(%d) deleted %d", cnf->actv_index, cnf->status);
        env->sync_idx = 0xFF;
        break;
    }
    default:
        break;
    }
    return 0;
}
BLE_EVENT_REGISTER(ble_app_event_handler, NULL);

int cmd_diss(int argc, char *argv[])
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

                ble_gap_scan_start(&scan_param);
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
                env->sync_detect = atoi(argv[7]);
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


FINSH_FUNCTION_EXPORT_ALIAS(cmd_diss, __cmd_diss, My device information service.);


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

