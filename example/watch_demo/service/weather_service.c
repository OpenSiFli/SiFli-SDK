/**
  ******************************************************************************
  * @file   weather_service.c
  * @author Sifli software development team
  * @brief Sibles ble weather service source.
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


#include <string.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <rtdef.h>
#include <board.h>
#include "bf0_sibles.h"
#include "bf0_sibles_serial_trans_service.h"
#include "bf0_sibles_weather_service.h"
#include "data_service_provider.h"


typedef struct
{
    uint8_t ref_count;
    rt_device_t device;
    datas_handle_t service;
    ble_weather_srv_data_t data;
} weather_service_env_t;


static weather_service_env_t weather_service_env;


static rt_err_t rx_ind(rt_device_t dev, rt_size_t size)
{
    rt_err_t result = -RT_ERROR;
    if (weather_service_env.service)
    {
        result = datas_ind_size(weather_service_env.service, size);
        RT_ASSERT(RT_EOK == result);
    }

    return result;
}


static int32_t weather_service_subscribe(datas_handle_t service)
{
    if (weather_service_env.device && weather_service_env.ref_count == 0)
    {
        rt_device_set_rx_indicate(weather_service_env.device, rx_ind);
    }
    weather_service_env.ref_count++;
    return 0;
}

static int32_t weather_service_unsubscribe(datas_handle_t service)
{
    weather_service_env.ref_count--;
    if (weather_service_env.device && weather_service_env.ref_count == 0)
    {
        rt_device_set_rx_indicate(weather_service_env.device, NULL);
    }

    return 0;
}

static int32_t weather_service_config(datas_handle_t service, void *config)
{
    return 0;
}

static int32_t weather_service_data_fetch(datas_handle_t service, uint32_t data_size, uint8_t **data)
{
    weather_service_env_t *env = &weather_service_env;
    rt_size_t size;

    RT_ASSERT(data);
    RT_ASSERT(env->device);

    *data = (uint8_t *)&env->data;

    size = rt_device_read(env->device, 0, *data, sizeof(env->data));

    RT_ASSERT(size == sizeof(env->data));

    return size;
}

static int32_t data_service_msg_handler(datas_handle_t service, data_msg_t *msg)
{
    switch (msg->msg_id)
    {
    case MSG_SERVICE_SUBSCRIBE_REQ:
    {
        weather_service_subscribe(service);
        break;
    }
    case MSG_SERVICE_UNSUBSCRIBE_REQ:
    {
        weather_service_unsubscribe(service);
        break;
    }
    case MSG_SERVICE_CONFIG_REQ:
    {
        data_req_t *req = (data_req_t *)data_service_get_msg_body(msg);
        int32_t result = weather_service_config(service, req);

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
        data_rdy_ind_t *data_ind = (data_rdy_ind_t *)(data_service_get_msg_body(msg));
        uint8_t *data;

        RT_ASSERT(data_ind);

        int32_t size = weather_service_data_fetch(service, data_ind->len, &data);
        if (size > 0)
        {
            datas_push_data_to_client(service, size, data);
        }

        break;
    }
    default:
    {
        RT_ASSERT(0);
    }
    }

    return 0;
}


static data_service_config_t weather_service_cb =
{
    .max_client_num = 5,
    .queue = RT_NULL,
    .msg_handler = data_service_msg_handler,
};


int weather_service_register(void)
{
    rt_err_t err;
    weather_service_env.device = rt_device_find("BLE_WEA");
    if (weather_service_env.device)
    {
        err = rt_device_open(weather_service_env.device, RT_DEVICE_OFLAG_RDONLY);
        RT_ASSERT(RT_EOK == err);
        weather_service_env.service = datas_register("WEA", &weather_service_cb);
    }

    return 0;
}

INIT_COMPONENT_EXPORT(weather_service_register);


