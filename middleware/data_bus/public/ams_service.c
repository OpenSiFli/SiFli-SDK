/**
  ******************************************************************************
  * @file   ams_service.c
  * @author Sifli software development team
  * @brief Source file - Data service for AMS.
 *
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


#include <string.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <rtdef.h>
#include <board.h>
#include "bf0_ble_gap.h"
#include "bf0_sibles.h"
#include "bf0_ble_ams.h"
#include "ams_service.h"
#include "data_service_provider.h"




#if defined(BSP_BLE_AMS) && defined(BSP_USING_DATA_SVC)

#define LOG_TAG "ams_dev"
#include "log.h"

typedef struct
{
    uint8_t conn_idx;
    struct
    {
        uint8_t ref_count;
        datas_handle_t service;
    } srv;
    struct
    {
        uint8_t player_mask;
        uint8_t queue_mask;
        uint8_t track_mask;
        uint8_t is_cccd_enable;
    } config;
} ams_service_env_t;



static ams_service_env_t g_ams_srv;

static ams_service_env_t *ams_service_get_env(void)
{
    return &g_ams_srv;
}


static void ams_service_enable(ams_service_env_t *env)
{

    ble_ams_player_attr_enable(env->config.player_mask);
    ble_ams_queue_attr_enable(env->config.queue_mask);
    ble_ams_track_attr_enable(env->config.track_mask);
    ble_ams_cccd_enable(env->config.is_cccd_enable);
    ble_ams_enable(env->conn_idx);
}



int ams_service_event_handler(uint16_t event_id, uint8_t *data, uint16_t len, uint32_t context)
{
    ams_service_env_t *env = ams_service_get_env();
    switch (event_id)
    {
    case BLE_GAP_BOND_IND:
    {
        ble_gap_bond_ind_t *evt = (ble_gap_bond_ind_t *)data;
        env->conn_idx = evt->conn_idx;
        LOG_I("bond ind %d, info %d", evt->conn_idx, evt->info);
        if (evt->info == GAPC_PAIRING_SUCCEED)
            ams_service_enable(env);
        break;
    }
    case BLE_GAP_ENCRYPT_IND:
    {
        ble_gap_encrypt_ind_t *ind = (ble_gap_encrypt_ind_t *)data;
        env->conn_idx = ind->conn_idx;
        LOG_I("encryt ind %d", ind->conn_idx);
        ams_service_enable(env);
        break;
    }
    case BLE_AMS_ENTITY_ATTRIBUTE_PAIR_IND:
    {
        ble_ams_entity_attr_value_t *value = (ble_ams_entity_attr_value_t *)data;
        int32_t ret = datas_push_data_to_client(env->srv.service, len, (uint8_t *)data);
        if (ret < 0)
            LOG_E("FWD attribute failed!");
        break;
    }
    case BLE_AMS_SUPPORTED_CMD_NOTIFY_IND:
    {
        // Do nothing first.
        break;
    }
    case BLE_AMS_ENABLE_RSP:
    {
        ble_ams_enable_rsp_t *rsp = (ble_ams_enable_rsp_t *)data;
        if (rsp->result != BLE_AMS_ERR_NO_ERR)
            LOG_E("Enable failed!!");
        break;
    }
    default:
        break;

    }

    return 0;
}


BLE_EVENT_REGISTER(ams_service_event_handler, NULL);



static int32_t ams_service_subscribe(datas_handle_t service)
{
    ams_service_env_t *env = ams_service_get_env();

    env->srv.ref_count++;
    return 0;
}

static int32_t ams_service_unsubscribe(datas_handle_t service)
{
    ams_service_env_t *env = ams_service_get_env();

    if (env->srv.ref_count > 0)
        env->srv.ref_count--;
    return 0;
}

static int32_t ams_service_config(datas_handle_t service, void *config)
{
    ams_service_env_t *env = ams_service_get_env();
    ams_service_config_t *setting = (ams_service_config_t *)config;
    int32_t ret = 0;
    switch (setting->command)
    {
    case AMS_SERVICE_SET_PLAYER_ATTRIBUTE_MASK:
    {
        env->config.player_mask = setting->data.player_mask;
        break;
    }
    case AMS_SERVICE_SET_QUEUE_ATTRIBUTE_MASK:
    {
        env->config.queue_mask = setting->data.queue_mask;
        break;
    }
    case AMS_SERVICE_SET_TRACK_ATTRIBUTE_MASK:
    {
        env->config.track_mask = setting->data.track_mask;
        break;
    }
    case AMS_SERVICE_SEND_REMOTE_COMMAND:
    {
        uint8_t ret1 = ble_ams_send_command(setting->data.remote_cmd);
        if (ret1 != BLE_AMS_ERR_NO_ERR)
            ret = -1;
        break;
    }
    case AMS_SERVICE_ENABLE_CCCD:
    {
        env->config.is_cccd_enable = setting->data.enable_cccd;
        break;
    }
    default:
        break;

    }
    return 0;
}




static int32_t ams_service_msg_handler(datas_handle_t service, data_msg_t *msg)
{
    ams_service_env_t *env = ams_service_get_env();

    switch (msg->msg_id)
    {
    case MSG_SERVICE_SUBSCRIBE_REQ:
    {
        ams_service_subscribe(service);
        break;
    }
    case MSG_SERVICE_UNSUBSCRIBE_REQ:
    {
        ams_service_unsubscribe(service);
        break;
    }
    case MSG_SERVICE_CONFIG_REQ:
    {
        data_req_t *req = (data_req_t *)data_service_get_msg_body(msg);
        int32_t result = ams_service_config(service, &req->data[0]);
        datas_send_response(service, msg, result);
        break;
    }
    case MSG_SERVICE_TX_REQ:
    {
        break;
    }
    case MSG_SERVICE_RX_REQ:
    {
        break;
    }
    case MSG_SERVICE_DATA_RDY_IND:
    {
        break;
    }
    default:
    {
        RT_ASSERT(0);
    }
    }

    return 0;
}


static data_service_config_t ams_service_cb =
{
    .max_client_num = 5,
    .queue = RT_NULL,
    .msg_handler = ams_service_msg_handler,
};



int ams_service_register(void)
{
    ams_service_env_t *env = ams_service_get_env();
    rt_err_t err;
    env->config.player_mask = AMS_SERVICE_DEFAULT_PLAYER_MASK;
    env->config.queue_mask = AMS_SERVICE_DEFAULT_QUEUE_MASK;
    env->config.track_mask = AMS_SERVICE_DEFAULT_TRACK_MASK;
    env->config.is_cccd_enable = 1;
    env->srv.service = datas_register("AMS", &ams_service_cb);
    RT_ASSERT(env->srv.service);

    return 0;
}

INIT_COMPONENT_EXPORT(ams_service_register);


#endif /* BSP_BLE_ANCS  */

