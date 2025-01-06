/**
  ******************************************************************************
  * @file   baro_service.c
  * @author Sifli software development team
  * @brief barometer service source.
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
#include <math.h>

#include "data_service_provider.h"
#include "sensor.h"
#include "sensor_bst_bmp280.h"
#include "baro_service.h"

#define DBG_TAG           "DS.BARO"
#define DBG_LVL           DBG_INFO
#include "rtdbg.h"


#define BARO_MODEL_NAME   "bmp280"
#define BARO_DEV_NAME "baro_"BARO_MODEL_NAME

enum
{
    BARO_UPDATE_MAX = (1 << 0),
    BARO_UPDATE_MIN = (1 << 1),
    ALTI_UPDATE_CUR = (1 << 2),
    ALTI_UPDATE_MAX = (1 << 3),
    ALTI_UPDATE_MIN = (1 << 4),
};


typedef struct
{
    int ref_count;
    rt_device_t device;
    datas_handle_t service;

    //TODO: need FIFO
    struct rt_sensor_data data;
    rt_timer_t timer;
    custom_baro_data_table_t table;
} baro_service_env_t;

static baro_service_env_t baro_service_env;
static uint8_t baro_update_flag = 0;


#define BARO_MSLP     101325          // Mean Sea Level Pressure = 1013.25 hPA (1hPa = 100Pa = 1mbar)

static int32_t baro_get_altitude(int32_t PressureVal)
{
    int32_t mslp = BARO_MSLP;
    float value = pow((PressureVal / (float)mslp), 0.1903);
    int32_t altitude = 4430000 * (1 - value);

    return altitude;
}


static int32_t baro_subscribe(datas_handle_t service)
{

    if (baro_service_env.timer && baro_service_env.ref_count == 0)
    {
        rt_timer_start(baro_service_env.timer);
    }

    baro_service_env.ref_count++;
    return 0;
}

static int32_t baro_unsubscribe(datas_handle_t service)
{
    baro_service_env.ref_count--;
    if (baro_service_env.timer && baro_service_env.ref_count == 0)
    {
        rt_timer_stop(baro_service_env.timer);
    }

    return 0;
}

static rt_err_t baro_service_config(datas_handle_t service, uint8_t *config)
{
    // Update service configuration. Save config for future filtering.
    return RT_EOK;
}

static rt_err_t baro_service_ping(datas_handle_t service, uint8_t *data)
{
    baro_service_env_t *env = &baro_service_env;

    if (!env->device)
    {
        return RT_ERROR;
    }

    return rt_device_control(env->device, RT_SENSOR_CTRL_SELF_TEST, data);
}

static data_req_t *baro_service_get_borameter(datas_handle_t service, uint16_t len)
{
    baro_service_env_t *env = &baro_service_env;
    data_req_t *r = NULL;

    r = rt_malloc(len + sizeof(data_req_t));
    if (r != NULL)
    {
        r->len = len;
        if (len == BAROS_BARO_RANGE_LEN) // get max baro, min baro
        {
            uint8_t maxmin[BAROS_BARO_RANGE_LEN];
            uint32_t *value = (uint32_t *)maxmin;
            value[0] = env->table.max_baro;
            value[1] = env->table.min_baro;
            memcpy(r->data, maxmin, len);
        }
    }

    return r;
}

static data_req_t *baro_service_get_altitude(datas_handle_t service, uint16_t len)
{
    baro_service_env_t *env = &baro_service_env;
    data_req_t *r = NULL;

    r = rt_malloc(len + sizeof(data_req_t));
    if (r != NULL)
    {
        r->len = len;
        if (len == BAROS_ALTI_RANGE_LEN) // get elevation current, max, min
        {
            uint8_t maxmin[BAROS_ALTI_RANGE_LEN];
            uint32_t *value = (uint32_t *)maxmin;
            value[0] = env->table.cur_alti;
            value[1] = env->table.max_alti;
            value[2] = env->table.min_alti;
            memcpy(r->data, maxmin, len);
        }
    }

    return r;
}

static int32_t baro_service_data_proc(datas_handle_t service, uint8_t *data)
{
    baro_service_env_t *env = &baro_service_env;
    struct rt_sensor_data *value = (struct rt_sensor_data *)data;
    int32_t alti;

    baro_update_flag = 0;
    // update current barometer
    if ((value->data.baro == 0) || (value->data.baro == env->table.cur_baro))
    {
        return 0;
    }
    env->table.cur_baro = value->data.baro;

    // check max
    if (value->data.baro > env->table.max_baro)
    {
        env->table.max_baro = value->data.baro;
        baro_update_flag |= BARO_UPDATE_MAX;
    }

    if (value->data.baro < env->table.min_baro)
    {
        env->table.min_baro = value->data.baro;
        baro_update_flag |= BARO_UPDATE_MIN;
    }

    alti = baro_get_altitude(env->table.cur_baro);
    if (alti != env->table.cur_alti)
    {
        env->table.cur_alti = alti;
        baro_update_flag |= ALTI_UPDATE_CUR;
    }

    if (alti > env->table.max_alti)
    {
        env->table.max_alti = alti;
        baro_update_flag |= ALTI_UPDATE_MAX;
    }

    if (alti < env->table.min_alti)
    {
        env->table.min_alti = alti;
        baro_update_flag |= ALTI_UPDATE_MIN;
    }
    if (baro_update_flag != 0)
    {
        if ((baro_update_flag & BARO_UPDATE_MAX) || (baro_update_flag & BARO_UPDATE_MIN))
        {
            data_req_t *result = baro_service_get_borameter(service, BAROS_BARO_RANGE_LEN);
            if (result)
            {
                datas_push_msg_to_client(service, MSG_SERVICE_BARO_RANGE_RSP, result->len, result->data);
                //datas_send_response_data(service, msg,result->len, result->data);
                rt_free(result);
            }
        }
        if ((baro_update_flag & ALTI_UPDATE_CUR) || (baro_update_flag & ALTI_UPDATE_MAX) || (baro_update_flag & ALTI_UPDATE_MIN))
        {
            data_req_t *result = baro_service_get_altitude(service, BAROS_ALTI_RANGE_LEN);
            if (result)
            {
                datas_push_msg_to_client(service, MSG_SERVICE_ALTITUDE_RANGE_RSP, result->len, result->data);
                //datas_send_response_data(service, msg,result->len, result->data);
                rt_free(result);
            }
        }
        baro_update_flag = 0;
    }

    return 0;
}

static int32_t baro_service_data_fetch(datas_handle_t service, uint32_t data_size, uint8_t **data)
{
    baro_service_env_t *env = &baro_service_env;
    rt_size_t size;

    RT_ASSERT(data);
    *data = NULL;
    if (!env->device)
    {
        return -1;
    }

    *data = (uint8_t *)&env->data;
    size = rt_device_read(env->device, 0, *data, 1);
    //RT_ASSERT(1 == size);
    if (size != 1)
        return 0;

    return sizeof(env->data);
}

bool baro_service_filter(data_req_t *config, uint16_t msg_id, uint32_t len, uint8_t *data)
{
    // Check if config is compatible with current config.
    return true;
}

static int32_t baro_service_msg_handler(datas_handle_t service, data_msg_t *msg)
{
    switch (msg->msg_id)
    {
    case MSG_SERVICE_SUBSCRIBE_REQ:
    {
        baro_subscribe(service);
        break;
    }
    case MSG_SERVICE_UNSUBSCRIBE_REQ:
    {
        baro_unsubscribe(service);
        break;
    }
    case MSG_SERVICE_CONFIG_REQ:
    {
        data_req_t *req = (data_req_t *)data_service_get_msg_body(msg);
        rt_err_t result = baro_service_config(service, &req->data[0]);
        datas_send_response(service, msg, result);
        break;
    }
    case MSG_SERVICE_PING_REQ:
    {
        data_req_t *req = (data_req_t *)data_service_get_msg_body(msg);
        rt_err_t result = baro_service_ping(service, &req->data[0]);
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
        data_rdy_ind_t *data_ind = (data_rdy_ind_t *)(data_service_get_msg_body(msg));
        uint8_t *data;

        RT_ASSERT(data_ind);

        int32_t size = baro_service_data_fetch(service, data_ind->len, &data);
        if (size > 0)
        {
            datas_push_data_to_client(service, data_ind->len, data);
            baro_service_data_proc(service,  data);
        }
        break;
    }
    case MSG_SERVICE_BARO_RANGE_REQ:
    {
        uint8_t *req = (uint8_t *)data_service_get_msg_body(msg);
        data_req_t *result = baro_service_get_borameter(service, (uint16_t) req[0]);
        if (result)
        {
            datas_send_response_data(service, msg, result->len, result->data);
            rt_free(result);
        }
        break;
    }
    case MSG_SERVICE_ALTITUDE_RANGE_REQ:
    {
        uint8_t *req = (uint8_t *)data_service_get_msg_body(msg);
        data_req_t *result = baro_service_get_altitude(service, (uint16_t) req[0]);
        if (result)
        {
            datas_send_response_data(service, msg, result->len, result->data);
            rt_free(result);
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


static data_service_config_t baro_service_cb =
{
    .max_client_num = 5,
    .queue = RT_NULL,
    .data_filter = baro_service_filter,
    .msg_handler = baro_service_msg_handler,
};

static void timeout_ind(void *param)
{
    if (baro_service_env.service)
        datas_ind_size(baro_service_env.service, sizeof(baro_service_env.data));
}


int baro_service_register(void)
{
    struct rt_sensor_config cfg;
    baro_service_env_t *env = &baro_service_env;
    rt_err_t err;
    int res = 0;

    //TODO: no use
    cfg.intf.dev_name = "i2c2";
    res = rt_hw_bmp280_init(BARO_MODEL_NAME, &cfg);
    //RT_ASSERT(res == 0);
    if (res != 0)
    {
        LOG_W("DataS init %s fail\n", BARO_MODEL_NAME);
        return 1;
    }
    env->device = rt_device_find(BARO_DEV_NAME);
    //RT_ASSERT(env->device);
    if (env->device == NULL)
    {
        LOG_W("DataS find device %s fail\n", BARO_DEV_NAME);
        return 1;
    }

    err = rt_device_open(env->device, RT_DEVICE_OFLAG_RDONLY);
    //RT_ASSERT(RT_EOK == err);
    if (err != RT_EOK)
    {
        LOG_W("DataS open device %s fail\n", BARO_DEV_NAME);
        return 1;
    }

    baro_service_env.service = datas_register("BARO", &baro_service_cb);
    baro_service_env.timer = rt_timer_create("BARO", timeout_ind, 0, rt_tick_from_millisecond(500), RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    RT_ASSERT(baro_service_env.timer);
    RT_ASSERT(baro_service_env.service);

    baro_service_env.table.cur_alti = 0;
    baro_service_env.table.max_alti = -10000;
    baro_service_env.table.min_alti = 200000;
    baro_service_env.table.cur_baro = 0;
    baro_service_env.table.max_baro = 0;
    baro_service_env.table.min_baro = 1000000;

    return 0;
}

INIT_COMPONENT_EXPORT(baro_service_register);


