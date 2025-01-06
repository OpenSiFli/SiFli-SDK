/**
  ******************************************************************************
  * @file   mag_service.c
  * @author Sifli software development team
  * @brief magnetic service source.
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
#include "data_service_provider.h"
#include "sensor.h"
#include "sensor_memsic_mmc36x0kj.h"

#define DBG_TAG           "DS.MAG"
#define DBG_LVL           DBG_INFO
#include "rtdbg.h"

#define MAG_MODEL_NAME   "mmc36x"
#define MAG_DEV_NAME "mag_"MAG_MODEL_NAME

typedef struct
{
    uint8_t ref_count;
    rt_device_t device;
    datas_handle_t service;

    //TODO: need FIFO
    struct rt_sensor_data mag_raw_data;
    rt_timer_t timer;
} mag_service_env_t;

static mag_service_env_t mag_service_env;

static int32_t mag_service_subscribe(datas_handle_t service)
{
    if (mag_service_env.timer && mag_service_env.ref_count == 0)
    {
        rt_timer_start(mag_service_env.timer);
    }
    mag_service_env.ref_count++;
    return 0;
}

static int32_t mag_service_unsubscribe(datas_handle_t service)
{
    mag_service_env.ref_count--;
    if (mag_service_env.timer && mag_service_env.ref_count == 0)
    {
        rt_timer_stop(mag_service_env.timer);
    }

    return 0;
}

static int32_t mag_service_config(datas_handle_t service, void *config)
{

    return 0;
}

static rt_err_t mag_service_ping(datas_handle_t service, uint8_t *data)
{
    mag_service_env_t *env = &mag_service_env;

    if (!env->device)
    {
        return RT_ERROR;
    }

    return rt_device_control(env->device, RT_SENSOR_CTRL_SELF_TEST, data);
}


static int32_t mag_service_data_fetch(datas_handle_t service, uint32_t data_size, uint8_t **data)
{
    mag_service_env_t *env = &mag_service_env;
    rt_size_t size;

    RT_ASSERT(data);

    *data = NULL;
    if (!env->device)
    {
        return 0;
    }

    *data = (uint8_t *)&env->mag_raw_data;

    size = rt_device_read(env->device, 0, *data, 1);
    RT_ASSERT(1 == size);

    return size;
}

static int32_t mag_service_msg_handler(datas_handle_t service, data_msg_t *msg)
{
    switch (msg->msg_id)
    {
    case MSG_SERVICE_SUBSCRIBE_REQ:
    {
        mag_service_subscribe(service);
        break;
    }
    case MSG_SERVICE_UNSUBSCRIBE_REQ:
    {
        mag_service_unsubscribe(service);
        break;
    }
    case MSG_SERVICE_CONFIG_REQ:
    {
        data_req_t *req = (data_req_t *)data_service_get_msg_body(msg);
        int32_t result = mag_service_config(service, &req->data[0]);
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
    case MSG_SERVICE_PING_REQ:
    {
        data_req_t *req = (data_req_t *)data_service_get_msg_body(msg);
        rt_err_t result = mag_service_ping(service, &req->data[0]);
        datas_send_response(service, msg, result);
        break;
    }
    case MSG_SERVICE_DATA_RDY_IND:
    {
        data_rdy_ind_t *data_ind = (data_rdy_ind_t *)(data_service_get_msg_body(msg));
        uint8_t *data;

        RT_ASSERT(data_ind);

        int32_t size = mag_service_data_fetch(service, data_ind->len, &data);
        if (size > 0)
        {
            datas_push_data_to_client(service, data_ind->len, data);
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


static data_service_config_t mag_service_cb =
{
    .max_client_num = 5,
    .queue = RT_NULL,
    .msg_handler = mag_service_msg_handler,
};


static void timeout_ind(void *param)
{
    rt_err_t result = -RT_ERROR;
    data_msg_t msg;
    if (mag_service_env.service)
    {
        result = datas_ind_size(mag_service_env.service, sizeof(mag_service_env.mag_raw_data));
        RT_ASSERT(RT_EOK == result);
    }
}


int mag_service_register(void)
{
    struct rt_sensor_config cfg;
    mag_service_env_t *env = &mag_service_env;
    rt_err_t err;
    int res = 0;

    //TODO: no use
    cfg.intf.dev_name = "i2c2";
    res = rt_hw_mmc36x0kj_init(MAG_MODEL_NAME, &cfg);
    //RT_ASSERT(res == 0);
    if (res != 0)
    {
        LOG_W("DataS init %s fail\n", MAG_MODEL_NAME);
        return 1;
    }
    env->device = rt_device_find(MAG_DEV_NAME);
    //RT_ASSERT(env->device);
    if (env->device == NULL)
    {
        LOG_W("DataS find device %s fail\n", MAG_DEV_NAME);
        return 1;
    }

    err = rt_device_open(env->device, RT_DEVICE_OFLAG_RDONLY);
    //RT_ASSERT(RT_EOK == err);
    if (err != RT_EOK)
    {
        LOG_W("DataS open device %s fail\n", MAG_DEV_NAME);
        return 1;
    }

    mag_service_env.service = datas_register("MAG", &mag_service_cb);
    mag_service_env.timer = rt_timer_create("MAG", timeout_ind, 0, rt_tick_from_millisecond(20), RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    RT_ASSERT(mag_service_env.timer);
    RT_ASSERT(mag_service_env.service);

    return 0;
}

INIT_COMPONENT_EXPORT(mag_service_register);


