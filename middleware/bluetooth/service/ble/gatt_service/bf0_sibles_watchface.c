/**
  ******************************************************************************
  * @file   bf0_sibles_watchface.c
  * @author Sifli software development team
  * @brief Sibles watchface transport.
 *
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2019 - 2022,  Sifli Technology
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

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <rtthread.h>
#include <rtdevice.h>

#include "board.h"
#include "os_adaptor.h"

#include "bf0_sibles.h"
#include "bf0_sibles_serial_trans_service.h"
#include "att.h"
#include "bf0_ble_gap.h"
#include "bf0_sibles.h"
#include "bf0_sibles_watchface.h"
#ifdef OTA_56X_NAND
    #include "dfu_internal.h"
#endif
#ifdef BSP_BLE_CONNECTION_MANAGER
    #include "ble_connection_manager.h"
#endif //BSP_BLE_CONNECTION_MANAGER

#ifdef BSP_BLE_WATCH_FACE


#ifndef DFU_OTA_MANAGER
#define LOG_TAG "BLE_WATCHFACE"
#include "log.h"

//#define WF_PHOTO_SYNC 1

static ble_watchface_env_t g_ble_watchface;
rt_timer_t g_update_time_handle;

OS_TIMER_DECLAR(g_watchface_timer);

static void ble_watchface_error_handler(uint16_t error_type);
static void watchface_sync_start(uint8_t type);
static void watchface_sync_end();
static void ble_watchface_lose_check(uint16_t result, uint32_t data_index);

static ble_watchface_env_t *ble_watchface_get_env(void)
{
    return &g_ble_watchface;
}

void watchface_register(watchface_callback callback)
{
    ble_watchface_env_t *env = ble_watchface_get_env();
    env->callback = callback;
}

static void watchface_sync_rsp_timer_handler(void *param)
{
    LOG_W("remote lost");
    watchface_sync_end();
    ble_watchface_error_handler(BLE_WATCHFACE_STATUS_DOWNLOAD_NOT_ONGOING);
}

static void watchface_sync_file_timer_handler(void *param)
{
    LOG_I("watchface_sync_file_timer_handler");
    ble_watchface_env_t *env = ble_watchface_get_env();
    watchface_sync_end();

    // check download process
    if (env->sync.last_index == env->current_index)
    {
        // download is not ongoing
        // ble_watchface_error_handler(BLE_WATCHFACE_STATUS_DOWNLOAD_NOT_ONGOING);
        ble_watchface_lose_check(BLE_WATCHFACE_STATUS_DOWNLOAD_NOT_ONGOING, env->current_index);
        watchface_sync_start(WATCHFACE_SYNC_TYPE_RSP);
    }
    else
    {
        env->sync.last_index = env->current_index;
        watchface_sync_start(WATCHFACE_SYNC_TYPE_FILE);
    }
}

static void watchface_sync_start(uint8_t type)
{
    LOG_I("watchface_sync_start!");
    ble_watchface_env_t *env = ble_watchface_get_env();
    rt_err_t ret = rt_sem_take(env->sem, WF_SEM_WAIT_TIME);
    if (ret != RT_EOK)
    {
        LOG_I("sem take over time");
        return;
    }

    if (env->sync.is_sync_on == 1)
    {
        rt_sem_release(env->sem);
        LOG_I("timer alread start!");
        return;
    }
    env->sync.is_sync_on = 1;
    env->sync.last_index = env->current_index;

    if (type == WATCHFACE_SYNC_TYPE_FILE)
    {
        os_timer_create(g_watchface_timer, watchface_sync_file_timer_handler, NULL, OS_TIMER_FLAG_ONE_SHOT | RT_TIMER_FLAG_SOFT_TIMER);
        os_timer_start(g_watchface_timer, WATCHFACE_SYNC_TIMEOUT);
    }
    else if (type == WATCHFACE_SYNC_TYPE_RSP)
    {
        os_timer_create(g_watchface_timer, watchface_sync_rsp_timer_handler, NULL, OS_TIMER_FLAG_ONE_SHOT | RT_TIMER_FLAG_SOFT_TIMER);
        os_timer_start(g_watchface_timer, WATCHFACE_SYNC_TIMEOUT);
    }

    rt_sem_release(env->sem);
}

static void watchface_sync_end()
{
    LOG_I("watchface_sync_end!");
    ble_watchface_env_t *env = ble_watchface_get_env();

    rt_err_t ret = rt_sem_take(env->sem, WF_SEM_WAIT_TIME);
    if (ret != RT_EOK)
    {
        LOG_I("sem take over time");
        return;
    }

    if (env->sync.is_sync_on == 0)
    {
        rt_sem_release(env->sem);
        LOG_I("no watchface timer to stop!");
        return;
    }

    env->sync.is_sync_on = 0;
    os_timer_stop(g_watchface_timer);
    os_timer_delete(g_watchface_timer);
    rt_sem_release(env->sem);
}


static int ble_watchface_data_send(uint8_t *raw_data, uint16_t size)
{
    ble_watchface_env_t *env = ble_watchface_get_env();

    ble_serial_tran_data_t t_data;
    t_data.cate_id = BLE_WATCHFACE_CATEID;
    t_data.handle = env->handle;
    t_data.data = (uint8_t *)raw_data;
    t_data.len = size;
    return ble_serial_tran_send_data(&t_data);
}

static void ble_watchface_lose_check(uint16_t result, uint32_t data_index)
{
    LOG_I("ble_watchface_lose_check %d, %d", result, data_index);

    uint16_t data_len = 8;
    uint8_t send_data[8];

    uint16_t command = BLE_WATCHFACE_LOSE_CHECK_REQ;

    data_index++;

    rt_memcpy(send_data, &command, sizeof(uint16_t));
    rt_memcpy(send_data + 2, &result, sizeof(uint16_t));
    rt_memcpy(send_data + 4, &data_index, sizeof(uint32_t));
    ble_watchface_data_send(send_data, data_len);
}

static void ble_watchface_lose_check_handle(ble_watchface_env_t *env, uint8_t *data, uint16_t length)
{
    LOG_I("ble_watchface_lose_check_handle");
    watchface_sync_end();
    watchface_sync_start(WATCHFACE_SYNC_TYPE_FILE);
}

uint8_t ble_watchface_abort()
{
    ble_watchface_env_t *env = ble_watchface_get_env();
    if (env->state == BLE_WATCHFACE_IDLE)
    {
        return 0;
    }
    LOG_I("ble_watchface_abort");
    watchface_sync_end();
    env->state = BLE_WATCHFACE_IDLE;

#ifdef BSP_BLE_CONNECTION_MANAGER
    connection_manager_update_parameter(env->conn_idx, CONNECTION_MANAGER_INTERVAL_LOW_POWER, NULL);
#endif //BSP_BLE_CONNECTION_MANAGER
    ble_watchface_lose_check(BLE_WATCHFACE_STATUS_REMOTE_ABORT, 0);
    return 1;
}

#ifdef OTA_56X_NAND
__WEAK uint8_t dfu_get_download_state()
{
    return 0;
}
#endif

static void ble_watchface_error_handler(uint16_t error_type)
{
    ble_watchface_env_t *env = ble_watchface_get_env();

    ble_watchface_error_ind_t ind;
    ind.error_type = error_type;
    ind.event = WATCHFACE_APP_ERROR;
    if (env->callback)
    {
        uint16_t callback_len = sizeof(uint32_t) * 2;
        env->callback(WATCHFACE_APP_ERROR, callback_len, &ind);
    }

    if (env->state == BLE_WATCHFACE_IDLE)
    {
        LOG_D("ble_watchface_error_handler idle");
        return;
    }

#ifdef BSP_BLE_CONNECTION_MANAGER
#ifdef OTA_56X_NAND
    if (dfu_get_download_state())
    {
        LOG_I("skip update due to ota");
    }
    else
    {
        connection_manager_update_parameter(env->conn_idx, CONNECTION_MANAGER_INTERVAL_LOW_POWER, NULL);
    }
#else
    connection_manager_update_parameter(env->conn_idx, CONNECTION_MANAGER_INTERVAL_LOW_POWER, NULL);
#endif
#endif //BSP_BLE_CONNECTION_MANAGER
    env->state = BLE_WATCHFACE_IDLE;
}

static void ble_watchface_reset_state()
{
    ble_watchface_env_t *env = ble_watchface_get_env();
    if (env->state == BLE_WATCHFACE_IDLE)
    {
        return;
    }

    cm_conneciont_parameter_value_t data;
    uint16_t interval;
    uint16_t latency;
    connection_manager_get_connetion_parameter(env->conn_idx, (uint8_t *)&data);
    interval = data.interval;
    latency = data.slave_latency;

    LOG_I("ble_watchface_check_connection %d, %d", interval, latency);
    if (interval < LOW_POWER_INTERVAL_MIN || latency < LOW_POWER_LATENCY)
    {
        LOG_I("going to update for watchface");
#ifdef OTA_56X_NAND
        if (dfu_get_download_state())
        {
            LOG_I("skip update due to ota");
        }
        else
        {
            connection_manager_update_parameter(env->conn_idx, CONNECTION_MANAGER_INTERVAL_LOW_POWER, NULL);
        }
#else
        connection_manager_update_parameter(env->conn_idx, CONNECTION_MANAGER_INTERVAL_LOW_POWER, NULL);
#endif
    }

    env->state = BLE_WATCHFACE_IDLE;
}

static void ble_watchface_check_connection()
{
#ifdef BSP_BLE_CONNECTION_MANAGER
    ble_watchface_env_t *env = ble_watchface_get_env();
    cm_conneciont_parameter_value_t data;
    uint16_t interval;
    uint16_t latency;
    connection_manager_get_connetion_parameter(env->conn_idx, (uint8_t *)&data);
    interval = data.interval;
    latency = data.slave_latency;

    LOG_I("ble_watchface_check_connection %d, %d", interval, latency);
    if (interval > HIGH_PERFORMANCE_INTERVAL_MAX || latency > HIGH_PERFORMANCE_LATENCY)
    {
        LOG_I("going to update for watchface");
        if (env->update_state == BLE_WF_UPDATE_NONE)
        {
            env->update_state = BLE_WF_UPDATE_UPDATING;
            connection_manager_update_parameter(env->conn_idx, CONNECTION_MANAGER_INTERVAL_HIGH_PERFORMANCE, NULL);
        }
        else
        {
            LOG_I("skip update due to updating");
        }
    }
#endif
}

void ble_watchface_send_start_rsp_file_info(uint16_t result, uint16_t block_length, uint32_t block_left)
{
    LOG_D("ble_watchface_send_start_rsp_file_info %d, %d, %d", result, block_length, block_left);
    ble_watchface_env_t *env = ble_watchface_get_env();
    if (env->state != BLE_WATCHFACE_PREPARE)
    {
        LOG_I("ble_watchface_send_start_rsp_file_info unexpect state %d", env->state);
        return;
    }
    uint16_t data_len = 14;
    uint8_t send_data[14];

    uint16_t command = BLE_WATCHFACE_START_RSP;
    // TODO: set this value by app
    uint16_t max_data_len = MAX_DATA_LEN;

    if (env->phone_type == PHONE_TYPE_IOS)
    {
        max_data_len = MAX_IOS_DATA_LEN;
    }

    rt_memcpy(send_data, &command, sizeof(uint16_t));
    rt_memcpy(send_data + 2, &result, sizeof(uint16_t));
    rt_memcpy(send_data + 4, &max_data_len, sizeof(uint16_t));

    uint16_t version = WF_VERSION;
    rt_memcpy(send_data + 6, &version, sizeof(uint16_t));
    rt_memcpy(send_data + 8, &block_length, sizeof(uint16_t));
    rt_memcpy(send_data + 10, &block_left, sizeof(uint32_t));

    if (env->file_type == WATCHFACE_FILE_TYPE_PHOTO_PREVIEW)
    {
        if (result == 0)
        {
#ifdef WF_PHOTO_SYNC
            watchface_sync_start(WATCHFACE_SYNC_TYPE_FILE);
#endif
            env->state = BLE_WATCHFACE_FILE_START;
            ble_watchface_check_connection();
        }
    }

    ble_watchface_data_send(send_data, data_len);

    if (result != BLE_WATCHFACE_STATUS_OK)
    {
        ble_watchface_reset_state();
    }
}

void ble_watchface_send_start_rsp(uint16_t result)
{
    LOG_I("ble_watchface_send_start_rsp %d", result);
    ble_watchface_env_t *env = ble_watchface_get_env();

    uint16_t data_len = 6;
    uint8_t send_data[6];

    uint16_t command = BLE_WATCHFACE_START_RSP;
    // TODO: set this value by app
    uint16_t max_data_len = MAX_DATA_LEN;

    if (env->phone_type == PHONE_TYPE_IOS)
    {
        max_data_len = MAX_IOS_DATA_LEN;
    }

    if (env->file_type == WATCHFACE_FILE_TYPE_PHOTO_PREVIEW)
    {
        if (result == 0)
        {
#ifdef WF_PHOTO_SYNC
            watchface_sync_start(WATCHFACE_SYNC_TYPE_FILE);
#endif
            env->state = BLE_WATCHFACE_FILE_START;
            ble_watchface_check_connection();
        }
    }

    rt_memcpy(send_data, &command, sizeof(uint16_t));
    rt_memcpy(send_data + 2, &result, sizeof(uint16_t));
    rt_memcpy(send_data + 4, &max_data_len, sizeof(uint16_t));
    ble_watchface_data_send(send_data, data_len);

    if (result != BLE_WATCHFACE_STATUS_OK)
    {
        ble_watchface_reset_state();
    }
}

__WEAK uint16_t ble_watchface_customize_state_check()
{
    return BLE_WATCHFACE_STATUS_STATE_ERROR;
}

static void ble_watchface_start_handler(ble_watchface_env_t *env, uint8_t *data, uint16_t length)
{
    LOG_I("ble_watchface_start_handler %d", env->state);
    if (env->state != BLE_WATCHFACE_IDLE)
    {
        uint16_t rsp_type;
        rsp_type = ble_watchface_customize_state_check();
        ble_watchface_send_start_rsp(rsp_type);
        ble_watchface_error_handler(BLE_WATCHFACE_STATUS_STATE_ERROR);
        return;
    }
    env->state = BLE_WATCHFACE_PREPARE;
    ble_watchface_start_ind_t ind;
    ind.event = WATCHFACE_APP_START;
    rt_memcpy(&ind.type, data, sizeof(uint16_t));
    env->current_type = ind.type;
    env->phone_type = 0;
    env->update_repeat = 0;

    env->file_type = ind.type;

    if (length > 2)
    {
        env->phone_type = *(data + 2);
        LOG_I("phone type %d", env->phone_type);
    }

    if (length > 6)
    {
        rt_memcpy(&ind.all_files_len, data + 3, sizeof(uint32_t));
        LOG_I("all files len %d", ind.all_files_len);
    }
    else
    {
        ind.all_files_len = 0;
    }

    if (env->callback)
    {
        uint16_t callback_len = sizeof(uint32_t) * 3;
        env->callback(WATCHFACE_APP_START, callback_len, &ind);
    }
    else
    {
        // TODO: REMOVE
        ble_watchface_send_start_rsp(0);
    }
}

void ble_watchface_file_info_rsp(uint16_t result)
{
    LOG_I("ble_watchface_file_info_rsp %d", result);

    uint16_t data_len = 4;
    uint8_t send_data[4];

    uint16_t command = BLE_WATCHFACE_FILE_INFO_RSP;
    rt_memcpy(send_data, &command, sizeof(uint16_t));
    rt_memcpy(send_data + 2, &result, sizeof(uint16_t));

    ble_watchface_data_send(send_data, data_len);

    if (result != BLE_WATCHFACE_STATUS_OK)
    {
        ble_watchface_reset_state();
    }
}

void ble_watchface_file_info_rsp_with_resume(uint16_t result, uint16_t resume_state, uint16_t resume_count)
{
    LOG_I("ble_watchface_file_info_rsp_with_resume %d, %d, %d", result, resume_state, resume_count);

    uint16_t data_len = 4;
    if (resume_state != WATCHFACE_RESUME_OLD_VERSION)
    {
        data_len = 8;
    }
    // Maximum size is 8
    uint8_t send_data[8];

    uint16_t command = BLE_WATCHFACE_FILE_INFO_RSP;
    rt_memcpy(send_data, &command, sizeof(uint16_t));
    rt_memcpy(send_data + 2, &result, sizeof(uint16_t));

    if (resume_state != WATCHFACE_RESUME_OLD_VERSION)
    {
        rt_memcpy(send_data + 4, &resume_state, sizeof(uint16_t));
        rt_memcpy(send_data + 6, &resume_count, sizeof(uint16_t));
    }

    ble_watchface_data_send(send_data, data_len);

    if (result != BLE_WATCHFACE_STATUS_OK)
    {
        ble_watchface_reset_state();
    }
}

static void ble_watchface_file_info_req_handler(ble_watchface_env_t *env, uint8_t *data, uint16_t length)
{
    if (env->state != BLE_WATCHFACE_PREPARE)
    {
        LOG_I("ble_watchface_file_info_req unexpect state %d", env->state);
        return;
    }

    ble_watchface_file_info_ind_t ind;
    ind.event = WATCHFACE_APP_FILE_INFO;
    rt_memcpy(&ind.file_blocks, data, sizeof(uint32_t));
    LOG_D("ble_watchface_file_info_req_handler %d", ind.file_blocks);
    if (length >= MD5_LEN + 4)
    {
        memcpy(&ind.md5, data + 4, MD5_LEN);
        LOG_HEX("WF MD5", 16, ind.md5, 32);
    }
    else
    {
        memset(&ind.md5, 0, MD5_LEN);
    }

    if (env->callback)
    {
        uint16_t callback_len = sizeof(ble_watchface_file_info_ind_t);
        env->callback(WATCHFACE_APP_FILE_INFO, callback_len, &ind);
    }
    else
    {
        ble_watchface_file_info_rsp(0);
    }
}

void ble_watchface_file_start_rsp(uint16_t result)
{
    LOG_I("ble_watchface_file_start_rsp %d", result);

    if (result == BLE_WATCHFACE_STATUS_OK)
    {
        watchface_sync_start(WATCHFACE_SYNC_TYPE_FILE);
        ble_watchface_check_connection();
    }

    ble_watchface_env_t *env = ble_watchface_get_env();

    if (env->state != BLE_WATCHFACE_FILE_PRE_START)
    {
        LOG_I("ble_watchface_file_start_rsp unexpect state %d", env->state);
        return;
    }
    env->state = BLE_WATCHFACE_FILE_START;

    uint16_t data_len = 4;
    uint8_t send_data[4];

    uint16_t command = BLE_WATCHFACE_FILE_SEND_START_RSP;
    rt_memcpy(send_data, &command, sizeof(uint16_t));
    rt_memcpy(send_data + 2, &result, sizeof(uint16_t));

    ble_watchface_data_send(send_data, data_len);

    if (result != BLE_WATCHFACE_STATUS_OK)
    {
        ble_watchface_reset_state();
    }
}

static void ble_watchface_file_start_handler(ble_watchface_env_t *env, uint8_t *data, uint16_t length)
{
    LOG_I("ble_watchface_file_start_handler %d", env->state);
    watchface_sync_end();
    if (env->state == BLE_WATCHFACE_FILE_END)
    {
        env->state = BLE_WATCHFACE_PREPARE;
    }

    if (env->state != BLE_WATCHFACE_PREPARE)
    {
        ble_watchface_error_handler(BLE_WATCHFACE_STATUS_STATE_ERROR);
        return;
    }

    ble_watchface_file_start_ind_t ind;

    env->state = BLE_WATCHFACE_FILE_PRE_START;
    ind.event = WATCHFACE_APP_FILE_START;
    rt_memcpy(&ind.file_len, data, sizeof(uint32_t));
    rt_memcpy(&ind.file_name_len, data + 4, sizeof(uint16_t));;
    ind.file_name = data + 6;

    //LOG_I("FILE NAME %c %c %c", *(ind.file_name), *(ind.file_name + 1), *(ind.file_name + 2));

    env->current_index = 0;
    env->receive_size = 0;
    env->total_size = ind.file_len;

    if (env->callback)
    {
        //uint16_t callback_len = sizeof(uint32_t) * 3 + ind.file_name_len;
        uint16_t callback_len = sizeof(ble_watchface_file_start_ind_t) + ind.file_name_len;
        env->callback(WATCHFACE_APP_FILE_START, callback_len, &ind);
    }
    else
    {
        // TODO: REMOVE
        ble_watchface_file_start_rsp(0);
    }

}

void ble_watchface_file_download_rsp(uint16_t result)
{
    LOG_I("ble_watchface_file_download_rsp %d", result);
    ble_watchface_env_t *env = ble_watchface_get_env();
    uint32_t index = 0;
    if (env->state != BLE_WATCHFACE_FILE_DOWNLOAD)
    {
        return;
    }

    if (result == BLE_WATCHFACE_STATUS_OK)
    {
        env->state = BLE_WATCHFACE_FILE_PROCESS;
        watchface_sync_start(WATCHFACE_SYNC_TYPE_FILE);
    }
    else if (result == BLE_WATCHFACE_STATUS_INDEX_ERROR)
    {
        env->state = BLE_WATCHFACE_FILE_PROCESS;
        index = env->current_index + 1;
    }
    else if (result > BLE_WATCHFACE_STATUS_APP_ERROR)
    {
        watchface_sync_end();
    }

    uint16_t data_len = 4 + 4;
    uint8_t send_data[8];

    uint16_t command = BLE_WATCHFACE_FILE_SEND_DATA_RSP;
    rt_memcpy(send_data, &command, sizeof(uint16_t));
    rt_memcpy(send_data + 2, &result, sizeof(uint16_t));
    rt_memcpy(send_data + 4, &index, sizeof(uint32_t));

    ble_watchface_data_send(send_data, data_len);

    if (result > BLE_WATCHFACE_STATUS_APP_ERROR)
    {
        ble_watchface_reset_state();
    }
}

void ble_watchface_file_photo_preview_download_rsp(uint16_t result)
{
#ifdef WF_PHOTO_SYNC
    if (result == BLE_WATCHFACE_STATUS_OK)
    {
        watchface_sync_start(WATCHFACE_SYNC_TYPE_FILE);
    }
#endif

    LOG_I("ble_watchface_file_photo_preview_download_rsp %d", result);
    ble_watchface_env_t *env = ble_watchface_get_env();
    uint32_t index = 0;
    if (env->state != BLE_WATCHFACE_FILE_DOWNLOAD)
    {
        return;
    }

    if (result == BLE_WATCHFACE_STATUS_OK)
    {
        env->state = BLE_WATCHFACE_FILE_PROCESS;
    }
    else if (result == BLE_WATCHFACE_STATUS_INDEX_ERROR)
    {
        env->state = BLE_WATCHFACE_FILE_PROCESS;
        index = env->current_index + 1;
    }
    else if (result > BLE_WATCHFACE_STATUS_APP_ERROR)
    {
#ifdef WF_PHOTO_SYNC
        watchface_sync_end();
#endif
    }

    uint16_t data_len = 4 + 4;
    uint8_t send_data[8];

    uint16_t command = BLE_WATCHFACE_FILE_PHOTO_PREVIEW_DATA_RSP;
    rt_memcpy(send_data, &command, sizeof(uint16_t));
    rt_memcpy(send_data + 2, &result, sizeof(uint16_t));
    rt_memcpy(send_data + 4, &index, sizeof(uint32_t));

    ble_watchface_data_send(send_data, data_len);

    if (result > BLE_WATCHFACE_STATUS_APP_ERROR)
    {
        ble_watchface_reset_state();
    }
}

static void ble_watchface_file_photo_preview_download_handler(ble_watchface_env_t *env, uint8_t *data, uint16_t length, uint16_t all_length)
{
    if (env->state == BLE_WATCHFACE_FILE_START || env->state == BLE_WATCHFACE_FILE_PROCESS)
    {
        env->state = BLE_WATCHFACE_FILE_DOWNLOAD;
    }
    else
    {
        LOG_I("photo preview data unexpected state %d", env->state);
        ble_watchface_error_handler(BLE_WATCHFACE_STATUS_STATE_ERROR);
        return;
    }

    uint8_t status = BLE_WATCHFACE_STATUS_GENERAL_ERROR;
    if (all_length == 0)
    {
        LOG_E("photo preview data download missing, current index: %d", env->current_index);
        status = BLE_WATCHFACE_STATUS_INDEX_ERROR;
        ble_watchface_file_download_rsp(status);
        return;
    }

    uint8_t packet_type;
    uint16_t file_name_len = 0;
    uint32_t file_len = 0;
    uint16_t process_index = 0;
    uint32_t data_index;
    uint16_t callback_len;

    uint16_t current_file_data_len = 0;

#ifdef WF_PHOTO_SYNC
    watchface_sync_end();
#endif
    ble_watchface_photo_preview_file_download_ind_t ind;

    packet_type = (*data);
    process_index++;
    ind.packet_type = packet_type;

    if (packet_type == WATCHFACE_PHOTO_PREVIEW_PACKET_TYPE_COMPLETE ||
            packet_type == WATCHFACE_PHOTO_PREVIEW_PACKET_TYPE_START)
    {
        memcpy(&file_name_len, data + process_index, sizeof(uint16_t));
        ind.file_name_len = file_name_len;

        process_index += sizeof(uint16_t);
        ind.file_name = data + process_index;

        process_index += file_name_len;
        memcpy(&file_len, data + process_index, sizeof(uint32_t));

        LOG_I("ble_watchface_file_photo_preview_download_handler type %d, len %d", packet_type, file_len);

        env->total_size = file_len;
        ind.file_len = file_len;

        process_index += sizeof(uint32_t);

        env->current_index = 0;
        env->receive_size = 0;
    }

    memcpy(&data_index, data + process_index, sizeof(uint32_t));

    LOG_I("ble_watchface_file_photo_preview_download_handler index %d", data_index);
    process_index += sizeof(uint32_t);

    if (env->current_index + 1 == data_index)
    {
        status = BLE_WATCHFACE_STATUS_OK;
        env->status = status;
        ind.data = data + process_index;

        current_file_data_len = length - process_index;

        env->receive_size += current_file_data_len;
        env->current_index = data_index;
        ind.data_len = current_file_data_len;
    }
    else
    {
        LOG_W("index error, expect %d, receive %d", env->current_index + 1, data_index);
        status = BLE_WATCHFACE_STATUS_INDEX_ERROR;
        env->status = status;
        ble_watchface_file_photo_preview_download_rsp(status);
        return;
    }

    if (packet_type == WATCHFACE_PHOTO_PREVIEW_PACKET_TYPE_LAST)
    {
        LOG_I("ble_watchface_file_photo_preview_download_handler last");
        if (env->status == BLE_WATCHFACE_STATUS_OK)
        {
            if (env->receive_size != env->total_size)
            {
                LOG_I("watchface length error, expect %d, receive %d", env->total_size, env->receive_size);
                status = BLE_WATCHFACE_STATUS_LEN_ERROR;
                env->status = status;
                ble_watchface_file_photo_preview_download_rsp(status);
                return;
            }
        }
    }

    if (env->callback)
    {
        ind.event = WATCHFACE_APP_FILE_PHOTO_PREVIEW_DOWNLOAD;
        callback_len = sizeof(ble_watchface_photo_preview_file_download_ind_t) + ind.file_name_len + ind.file_len;
        env->callback(WATCHFACE_APP_FILE_PHOTO_PREVIEW_DOWNLOAD, callback_len, &ind);
    }
    else
    {
        ble_watchface_file_photo_preview_download_rsp(status);
    }
}

static void ble_watchface_file_download_handler(ble_watchface_env_t *env, uint8_t *data, uint16_t length, uint16_t all_length)
{
    if (env->state == BLE_WATCHFACE_FILE_START || env->state == BLE_WATCHFACE_FILE_PROCESS)
    {
        env->state = BLE_WATCHFACE_FILE_DOWNLOAD;
    }
    else
    {
        LOG_I("ble_watchface_file_download_handler unexpected state %d", env->state);
        ble_watchface_error_handler(BLE_WATCHFACE_STATUS_STATE_ERROR);
        return;
    }

    uint8_t status = BLE_WATCHFACE_STATUS_GENERAL_ERROR;
    uint32_t index = 0;
    uint16_t callback_len;
    if (all_length == 0)
    {
        LOG_E("ble_watchface_file_download_handler download missing, current index: %d", env->current_index);
        status = BLE_WATCHFACE_STATUS_INDEX_ERROR;
        ble_watchface_file_download_rsp(status);
        return;
    }
    memcpy(&index, data, sizeof(uint32_t));

    LOG_D("receive data index %d", index);
    ble_watchface_file_download_ind_t ind;
    ind.event = WATCHFACE_APP_FILE_DOWNLOAD;
    if (index == 0)
    {
        watchface_sync_end();
        status = BLE_WATCHFACE_STATUS_OK;
        env->receive_size = length - 4;

        ind.data_len = length - 4;
        ind.data = data + 4;

        // TODO: add process
        if (env->callback)
        {
            //callback_len = sizeof(uint32_t) * 2 + ind.data_len;
            callback_len = sizeof(ble_watchface_file_download_ind_t) + ind.data_len;
            env->callback(WATCHFACE_APP_FILE_DOWNLOAD, callback_len, &ind);
        }
    }
    else
    {
        if (index == env->current_index + 1)
        {
            watchface_sync_end();
            env->current_index = index;
            env->receive_size += length - 4;
            status = BLE_WATCHFACE_STATUS_OK;
            ind.data_len = length - 4;
            ind.data = data + 4;

            // TODO: add process
            if (env->callback)
            {
                //callback_len = sizeof(uint32_t) * 2 + ind.data_len;
                callback_len = sizeof(ble_watchface_file_download_ind_t) + ind.data_len;
                env->callback(WATCHFACE_APP_FILE_DOWNLOAD, callback_len, &ind);
            }
        }
        else
        {
            watchface_sync_end();
            watchface_sync_start(WATCHFACE_SYNC_TYPE_RSP);
            // index error
            LOG_W("index error, expect %d, receive %d", env->current_index + 1, index);
            status = BLE_WATCHFACE_STATUS_INDEX_ERROR;
            env->status = status;
            ble_watchface_file_download_rsp(status);
            return;
        }
    }
    env->status = status;

    if (!env->callback)
    {
        // TODO: REMOVE
        ble_watchface_file_download_rsp(status);
    }
}

void ble_watchface_file_end_rsp(uint16_t result)
{
    LOG_I("ble_watchface_file_end_rsp %d", result);
    ble_watchface_env_t *env = ble_watchface_get_env();
    if (env->state != BLE_WATCHFACE_FILE_PROCESS)
    {
        return;
    }
    env->state = BLE_WATCHFACE_FILE_END;

    uint16_t data_len = 4;
    uint8_t send_data[4];

    uint16_t command = BLE_WATCHFACE_FILE_SEND_END_RSP;
    rt_memcpy(send_data, &command, sizeof(uint16_t));
    rt_memcpy(send_data + 2, &result, sizeof(uint16_t));

    ble_watchface_data_send(send_data, data_len);

    if (result != BLE_WATCHFACE_STATUS_OK)
    {
        ble_watchface_reset_state();
    }
    else
    {
        watchface_sync_start(WATCHFACE_SYNC_TYPE_RSP);
    }
}

static void ble_watchface_file_end_handler(ble_watchface_env_t *env, uint8_t *data, uint16_t length)
{
    if (env->state != BLE_WATCHFACE_FILE_PROCESS)
    {
        ble_watchface_error_handler(BLE_WATCHFACE_STATUS_STATE_ERROR);
        return;
    }

    watchface_sync_end();
    ble_watchface_file_end_ind_t ind;
    uint8_t end_status = 0;
    if (env->status == BLE_WATCHFACE_STATUS_OK)
    {
        if (env->receive_size != env->total_size)
        {
            LOG_I("watch face length error, expect %d, receive %d", env->total_size, env->receive_size);
            end_status = BLE_WATCHFACE_STATUS_LEN_ERROR;
        }
    }
    else
    {
        end_status = env->status;
    }

    ind.event = WATCHFACE_APP_FILE_END;
    ind.end_status = end_status;

    LOG_I("BLE_WATCHFACE_SEND_END %d", end_status);

    if (env->callback)
    {
        uint16_t callback_len = sizeof(uint32_t) * 2;
        env->callback(WATCHFACE_APP_FILE_END, callback_len, &ind);
    }
    else
    {
        // TODO: REMOVE
        ble_watchface_file_end_rsp(end_status);
    }

}

void ble_watchface_end_rsp(uint16_t result)
{
    LOG_I("ble_watchface_end_rsp %d", result);
    ble_watchface_env_t *env = ble_watchface_get_env();
    env->state = BLE_WATCHFACE_IDLE;
    uint16_t data_len = 4;
    uint8_t send_data[4];

    uint16_t command = BLE_WATCHFACE_END_RSP;
    rt_memcpy(send_data, &command, sizeof(uint16_t));
    rt_memcpy(send_data + 2, &result, sizeof(uint16_t));

    ble_watchface_data_send(send_data, data_len);
}

static void wf_timeout_handler(void *parameter)
{
    ble_watchface_env_t *env = ble_watchface_get_env();
    if (env->state != BLE_WATCHFACE_IDLE)
    {
        LOG_I("skip update due to state %d", env->state);
        return;
    }
#ifdef BSP_BLE_CONNECTION_MANAGER
    connection_manager_update_parameter(env->conn_idx, CONNECTION_MANAGER_INTERVAL_LOW_POWER, NULL);
#endif //BSP_BLE_CONNECTION_MANAGER
}

static void ble_watchface_abort_handler(ble_watchface_env_t *env, uint8_t *data, uint16_t length)
{
    uint8_t abort_reason = data[0];
    LOG_I("ble_watchface_abort_handler %d", abort_reason);
    watchface_sync_end();
    ble_watchface_error_handler(abort_reason);
}

static void ble_watchface_end_handler(ble_watchface_env_t *env, uint8_t *data, uint16_t length)
{
    LOG_I("ble_watchface_end_handler");
    watchface_sync_end();
    // delay update for iOS 16
    if (!g_update_time_handle)
    {
        g_update_time_handle = rt_timer_create("ble_wf", wf_timeout_handler, NULL,
                                               rt_tick_from_millisecond(3000), RT_TIMER_FLAG_SOFT_TIMER);
    }
    else
    {
        rt_timer_stop(g_update_time_handle);
    }
    rt_timer_start(g_update_time_handle);

    if (env->file_type == WATCHFACE_FILE_TYPE_PHOTO_PREVIEW)
    {
#ifdef WF_PHOTO_SYNC
        watchface_sync_end();
#endif
    }
    else
    {
        if (env->state != BLE_WATCHFACE_FILE_END)
        {
            ble_watchface_error_handler(BLE_WATCHFACE_STATUS_STATE_ERROR);
            return;
        }
    }

    ble_watchface_end_ind_t ind;
    ind.event = WATCHFACE_APP_END;
    if (env->callback)
    {
        uint16_t callback_len = sizeof(uint32_t);
        env->callback(WATCHFACE_APP_END, callback_len, &ind);
    }
    else
    {
        // TODO: REMOVE
        ble_watchface_end_rsp(0);
    }
}

uint8_t ble_watchface_get_state()
{
    ble_watchface_env_t *env = ble_watchface_get_env();
    return env->state;
}

static void ble_watchface_packet_handler(ble_watchface_protocol_t *msg, uint16_t length)
{
    ble_watchface_env_t *env = ble_watchface_get_env();
    switch (msg->message_id)
    {
    case BLE_WATCHFACE_START_REQ:
    {
        ble_watchface_start_handler(env, msg->data, msg->length);
        break;
    }
    case BLE_WATCHFACE_FILE_SEND_START_REQ:
    {
        ble_watchface_file_start_handler(env, msg->data, msg->length);
        break;
    }
    case BLE_WATCHFACE_FILE_SEND_DATA:
    {
        //LOG_I("BLE_WATCHFACE_SEND_DATA %d", length);
        ble_watchface_file_download_handler(env, msg->data, msg->length, length);

        break;
    }
    case BLE_WATCHFACE_FILE_SEND_END_REQ:
    {
        ble_watchface_file_end_handler(env, msg->data, msg->length);
        break;
    }
    case BLE_WATCHFACE_END_REQ:
    {
        ble_watchface_end_handler(env, msg->data, msg->length);
        break;
    }
    case BLE_WATCHFACE_LOSE_CHECK_RSP:
    {
        ble_watchface_lose_check_handle(env, msg->data, msg->length);
        break;
    }
    case BLE_WATCHFACE_ABORT_CMD:
    {
        ble_watchface_abort_handler(env, msg->data, msg->length);
        break;
    }
    case BLE_WATCHFACE_FILE_INFO_REQ:
    {
        ble_watchface_file_info_req_handler(env, msg->data, msg->length);
        break;
    }
    case BLE_WATCHFACE_FILE_PHOTO_PREVIEW_DATA:
    {
        ble_watchface_file_photo_preview_download_handler(env, msg->data, msg->length, length);
        break;
    }
    default:
        break;
    }
}

static void ble_watchface_serial_callback(ble_serial_tran_event_t event, uint8_t *data)
{
    if (!data)
        return;

    ble_watchface_env_t *env = ble_watchface_get_env();
    switch (event)
    {
    case BLE_SERIAL_TRAN_OPEN:
    {
        ble_serial_open_t *open = (ble_serial_open_t *)data;
        env->is_open = 1;
        env->handle = open->handle;
    }
    break;
    case BLE_SERIAL_TRAN_DATA:
    {
        ble_serial_tran_data_t *t_data = (ble_serial_tran_data_t *)data;
        if (env->handle == t_data->handle
                && t_data->cate_id == BLE_WATCHFACE_CATEID)
        {
            ble_watchface_packet_handler((ble_watchface_protocol_t *)t_data->data, t_data->len);
        }
    }
    break;
    case BLE_SERIAL_TRAN_CLOSE:
    {
        ble_serial_close_t *close = (ble_serial_close_t *)data;
        if (env->handle == close->handle)
        {
            env->is_open = 0;
        }
    }
    break;
    case BLE_SERIAL_TRAN_ERROR:
    {
        ble_watchface_lose_check(BLE_WATCHFACE_STATUS_RECEIVE_ERROR, 0);
        break;
    }
    default:
        break;
    }
}

int ble_watchface_event_handler(uint16_t event_id, uint8_t *data, uint16_t len, uint32_t context)
{
    ble_watchface_env_t *env = ble_watchface_get_env();

    switch (event_id)
    {
    case BLE_POWER_ON_IND:
    {
        env->sem = rt_sem_create("wf_sem", 1, RT_IPC_FLAG_FIFO);
        OS_ASSERT(env->sem);
        break;
    }
    case BLE_GAP_CONNECTED_IND:
    {
        ble_gap_connect_ind_t *ind = (ble_gap_connect_ind_t *)data;
        env->conn_idx = ind->conn_idx;
        env->mtu = 23;
        env->state = BLE_WATCHFACE_IDLE;
        env->update_state = BLE_WF_UPDATE_NONE;
        if (env->sync.is_sync_on == 1)
        {
            watchface_sync_end();
        }
        break;
    }
    case SIBLES_MTU_EXCHANGE_IND:
    {
        /* Negotiated MTU. */
        sibles_mtu_exchange_ind_t *ind = (sibles_mtu_exchange_ind_t *)data;
        env->mtu = ind->mtu;
        break;
    }
    case BLE_GAP_DISCONNECTED_IND:
    {
        if (g_update_time_handle)
        {
            rt_timer_stop(g_update_time_handle);
        }

        if (env->state != BLE_WATCHFACE_IDLE)
        {
            if (env->sync.is_sync_on == 1)
            {
                watchface_sync_end();
            }
            if (env->callback)
            {
                ble_watchface_error_handler(BLE_WATCHFACE_STATUS_DISCONNECT);
            }
            env->state = BLE_WATCHFACE_IDLE;
        }
        break;
    }
    case BLE_GAP_UPDATE_CONN_PARAM_CNF:
    {
        ble_gap_update_conn_param_cnf_t *cnf = (ble_gap_update_conn_param_cnf_t *)data;
        if (cnf->status != 0)
        {
            LOG_I("update fail with %d", cnf->status);
            if (env->state != BLE_WATCHFACE_IDLE && env->update_repeat < MAX_UPDATE_REPEAT && env->update_state == BLE_WF_UPDATE_UPDATING)
            {
                LOG_I("update again due to collision %d", env->update_repeat);
                env->update_repeat++;
                env->update_state = BLE_WF_UPDATE_NONE;
                ble_watchface_check_connection();
            }
        }
        env->update_state = BLE_WF_UPDATE_NONE;
        break;
    }
    default:
        break;
    }
    return 0;

}

BLE_EVENT_REGISTER(ble_watchface_event_handler, NULL);


BLE_SERIAL_TRAN_EXPORT(BLE_WATCHFACE_CATEID, ble_watchface_serial_callback);

#endif // DFU_OTA_MANAGER

#endif // BSP_BLE_WATCH_FACE

