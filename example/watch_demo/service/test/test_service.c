/**
  ******************************************************************************
  * @file   test_service.c
  * @author Sifli software development team
  * @brief heart rate service source.
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
#include "bf0_hal_hlp.h"

#define MAX_TEST_SERVICE    10
#define HR_TEST_ID          (1)
#define COMP_TEST_ID        (2)
#define BARO_TEST_ID        (3)
//#define PM_TEST_ID          (4)
#define ALARM_TEST_ID       (5)

#ifdef HR_TEST_ID
    #include "../hr_service.h"
#endif
#ifdef COMP_TEST_ID
    #include "../compass_service.h"
#endif /*COMP_TEST_ID*/
#ifdef BARO_TEST_ID
    #include "../baro_service.h"
#endif /*BARO_TEST_ID*/
static int g_count = 0;

typedef struct
{
    int ref_count;
    datas_handle_t service;
    data_req_t *data;
    rt_timer_t timer;
} test_service_env_t;

static test_service_env_t test_service_env[MAX_TEST_SERVICE];
//uint8_t g_test_svc_num = 0;


static int8_t test_service_getidx(datas_handle_t service)
{
    int8_t r;

    for (r = 0; r < MAX_TEST_SERVICE; r++)
        if (service == test_service_env[r].service)
            break;
    if (r == MAX_TEST_SERVICE)
        r = -1;
    return r;
}

static int32_t test_subscribe(datas_handle_t service)
{
    int8_t index = test_service_getidx(service);
    test_service_env_t *env = &test_service_env[index];

    if (env->timer && env->ref_count == 0)
    {
        rt_timer_start(env->timer);
    }

    env->ref_count++;
    return 0;
}

static int32_t test_unsubscribe(datas_handle_t service)
{
    int8_t index = test_service_getidx(service);
    test_service_env_t *env = &test_service_env[index];

    env->ref_count--;
    if (env->timer && env->ref_count == 0)
    {
        rt_timer_stop(env->timer);
    }

    return 0;
}

static rt_err_t test_service_config(datas_handle_t service, uint8_t *config)
{
    // Update service configuration. Save config for future filtering.
    return RT_EOK;
}

static rt_err_t test_service_tx(datas_handle_t service, int len, uint8_t *data)
{
    HAL_DBG_print_data((char *)data, 0, len);
    return RT_EOK;
}

static data_req_t *test_service_rx(datas_handle_t service, uint16_t *len)
{
    int8_t index = test_service_getidx(service);
    data_req_t *r = NULL;

    if (index >= 0)
    {
        test_service_env_t *env;
        env = &test_service_env[index];
        if (env->data)
        {
            if (*len > env->data->len)
                *len = env->data->len;
            if (*len)
                r = rt_malloc(*len + sizeof(data_req_t));
            r->len = *len;
            memcpy(r->data, env->data->data, r->len);
        }
    }

    return r;
}

#ifdef BARO_TEST_ID

static data_req_t *test_service_bora_range(datas_handle_t service, uint16_t len)
{
    int8_t index = test_service_getidx(service);
    data_req_t *r = NULL;

    if (BARO_TEST_ID == index)
    {
        r = rt_malloc(len + sizeof(data_req_t));
        r->len = len;
        if (len == BAROS_BARO_RANGE_LEN) // get bora max, min
        {
            uint8_t maxmin[BAROS_BARO_RANGE_LEN];
            uint32_t *value = (uint32_t *)maxmin;
            value[0] = 110000;
            value[1] = 70000;
            memcpy(r->data, maxmin, len);
        }
    }
    return r;
}

static data_req_t *test_service_elev_range(datas_handle_t service, uint16_t len)
{
    int8_t index = test_service_getidx(service);
    data_req_t *r = NULL;

    if (BARO_TEST_ID == index)
    {
        r = rt_malloc(len + sizeof(data_req_t));
        r->len = len;
        if (len == BAROS_ALTI_RANGE_LEN) // get  max rhr, min rhr, average rhr
        {
            uint8_t maxmin[BAROS_ALTI_RANGE_LEN];
            int32_t *value = (int32_t *)maxmin;
            value[0] = 13000;
            value[1] = 230000;
            value[2] = -3000;
            memcpy(r->data, maxmin, len);
        }
    }
    return r;
}

#endif


static int32_t test_service_data_fetch(datas_handle_t service, uint32_t data_size, uint8_t **data)
{
    rt_size_t size = 0;
    int8_t index = test_service_getidx(service);

    if (index >= 0)
    {
        test_service_env_t *env;
        env = &test_service_env[index];
        *data = env->data->data;
        size = env->data->len;
        if (size == sizeof(struct rt_sensor_data))   // check if sensor service
        {
            struct rt_sensor_data *ptr = (struct rt_sensor_data *)(env->data->data);
            if ((ptr->type >= RT_SENSOR_CLASS_ACCE) && (ptr->type <= RT_SENSOR_CLASS_GPS)) // it should be sensor, double check
            {
                ptr->timestamp = rt_sensor_get_ts();
                switch (ptr->type)
                {
#ifdef HR_TEST_ID
                case RT_SENSOR_CLASS_HR:
                    ptr->data.hr = ptr->data.hr + (ptr->timestamp % 3) - 1;
                    break;
#endif
#ifdef BARO_TEST_ID
                case RT_SENSOR_CLASS_BARO:
                {
                    static int inc_flag = 0;
                    if (inc_flag == 0)
                        ptr->data.baro = ptr->data.baro + 1000;
                    else
                        ptr->data.baro = ptr->data.baro - 1000;

                    if (ptr->data.baro >= 110000)
                    {
                        //ptr->data.baro = 70000;
                        inc_flag = 1;
                        data_req_t *result = test_service_bora_range(service, BAROS_BARO_RANGE_LEN);
                        if (result)
                        {
                            datas_push_msg_to_client(service, MSG_SERVICE_BARO_RANGE_RSP, result->len, result->data);
                            rt_free(result);
                            result = NULL;
                        }
                        result = test_service_elev_range(service, BAROS_ALTI_RANGE_LEN);
                        if (result)
                        {
                            datas_push_msg_to_client(service, MSG_SERVICE_ALTITUDE_RANGE_RSP, result->len, result->data);
                            rt_free(result);
                        }
                    }
                    else if (ptr->data.baro <= 70000)
                    {
                        inc_flag = 0;
                    }

                    break;
                }

#endif
                }
            }
        }
#ifdef COMP_TEST_ID
        if ((size == sizeof(compass_data_t)) && (index == COMP_TEST_ID))
        {
            compass_data_t *ptr = (compass_data_t *)(env->data->data);
            //
            ptr->accuracy = 1;
            ptr->azimuth = ptr->azimuth + 1.0;
            if (ptr->azimuth > 360.0)
                ptr->azimuth = 0.0;
        }
#endif /* COMP_TEST_ID */
    }

    return size;
}

#ifdef HR_TEST_ID
static data_req_t *test_service_hr_max_min(datas_handle_t service, uint16_t len)
{
    int8_t index = test_service_getidx(service);
    data_req_t *r = NULL;

    if (HR_TEST_ID == index)
    {
        r = rt_malloc(len + sizeof(data_req_t));
        r->len = len;
        if (len == HRS_MAX_MIN_LEN)   // just return max , min , current rhrfor test
        {
            uint8_t maxmin[HRS_MAX_MIN_LEN] = {60, 80, 70};
            memcpy(r->data, maxmin, len);
        }
    }
    return r;
}

static data_req_t *test_service_hr_day_table(datas_handle_t service, uint16_t len)
{
    int8_t index = test_service_getidx(service);
    data_req_t *r = NULL;

    if (HR_TEST_ID == index)
    {
        r = rt_malloc(len + sizeof(data_req_t));
        r->len = len;
        if (len == HRS_DAY_TABLE_LEN) // get day value : 24 hour
        {
            uint8_t hour[HRS_DAY_TABLE_LEN] = { 0, 0, 0, 0, 0, 0, 0, 0,
                                                60, 70, 80, 90, 100, 0, 0, 90,
                                                70, 70, 80, 90, 0, 0, 0, 0
                                              };
            memcpy(r->data, hour, len);
        }
    }
    return r;
}

static data_req_t *test_service_hr_mon_table(datas_handle_t service, uint16_t len)
{
    int8_t index = test_service_getidx(service);
    data_req_t *r = NULL;

    if (HR_TEST_ID == index)
    {
        r = rt_malloc(len + sizeof(data_req_t));
        r->len = len;
        if (len == HRS_MON_TABLE_LEN) // get rhr: 30 days
        {
            uint8_t maxmin[HRS_MON_TABLE_LEN] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                 0, 0, 0, 0, 0, 0, 65, 67, 0, 0
                                                };
            memcpy(r->data, maxmin, len);
        }
    }
    return r;
}

static data_req_t *test_service_hr_region(datas_handle_t service, uint16_t len)
{
    int8_t index = test_service_getidx(service);
    data_req_t *r = NULL;

    if (HR_TEST_ID == index)
    {
        r = rt_malloc(len + sizeof(data_req_t));
        r->len = len;
        if (len == HRS_REGION_LEN) // get region, ul, ana, aer, hiit, warmup
        {
            uint8_t maxmin[HRS_REGION_LEN] = {0, 0, 0, 3, 10};
            memcpy(r->data, maxmin, len);
        }
    }
    return r;
}

static data_req_t *test_service_rhr_value(datas_handle_t service, uint16_t len)
{
    int8_t index = test_service_getidx(service);
    data_req_t *r = NULL;

    if (HR_TEST_ID == index)
    {
        r = rt_malloc(len + sizeof(data_req_t));
        r->len = len;
        if (len == HRS_RHR_HIST_LEN) // get  max rhr, min rhr, average rhr
        {
            uint8_t maxmin[HRS_RHR_HIST_LEN] = {76, 65, 70};
            memcpy(r->data, maxmin, len);
        }
    }
    return r;
}

#endif

bool test_service_filter(data_req_t *config, uint16_t msg_id, uint32_t len, uint8_t *data)
{
    uint8_t count_mod = config->data[0];

    if (count_mod == 0)
        count_mod = 1;
    if ((g_count % count_mod) == 0)
        return true;
    else
        return false;
}

static int32_t test_service_msg_handler(datas_handle_t service, data_msg_t *msg)
{
    switch (msg->msg_id)
    {
    case MSG_SERVICE_SUBSCRIBE_REQ:
    {
        test_subscribe(service);
        break;
    }
    case MSG_SERVICE_UNSUBSCRIBE_REQ:
    {
        test_unsubscribe(service);
        break;
    }
    case MSG_SERVICE_CONFIG_REQ:
    {
        data_req_t *req = (data_req_t *)data_service_get_msg_body(msg);
        rt_err_t result = test_service_config(service, &req->data[0]);
        datas_send_response(service, msg, result);
        break;
    }
    case MSG_SERVICE_TX_REQ:
    {
        data_req_t *req = (data_req_t *)data_service_get_msg_body(msg);
        rt_err_t result = test_service_tx(service, req->len, &req->data[0]);
        datas_send_response(service, msg, result);
        break;
    }
    case MSG_SERVICE_RX_REQ:
    {
        data_req_t *req = (data_req_t *)data_service_get_msg_body(msg);
        data_req_t *result = test_service_rx(service, (uint16_t *) &req->data[0]);
        datas_send_response(service, msg, result == NULL ? -RT_ERROR : RT_EOK);
        if (result)
        {
            g_count++;
            datas_push_data_to_client(service, result->len, result->data);
            rt_free(result);
        }
        break;
    }
    case MSG_SERVICE_DATA_RDY_IND:
    {
        data_rdy_ind_t *data_ind = (data_rdy_ind_t *)(data_service_get_msg_body(msg));
        uint8_t *data;

        RT_ASSERT(data_ind);

        int32_t size = test_service_data_fetch(service, data_ind->len, &data);
        if (size > 0)
        {
            if (size > (int32_t)data_ind->len)
                size = data_ind->len;
            g_count++;
            datas_push_data_to_client(service, size, data);
        }
        break;
    }
#ifdef HR_TEST_ID
    case MSG_SERVICE_HR_DAY_TABLE_REQ:
    {
        uint8_t *req = (uint8_t *)data_service_get_msg_body(msg);
        data_req_t *result = test_service_hr_day_table(service, (uint16_t) req[0]);
        //datas_send_response(service, msg, result == NULL ? -RT_ERROR : RT_EOK);
        if (result)
        {
            datas_push_day_table_to_client(service, result->len, result->data);
            rt_free(result);
        }
        break;
    }
    case MSG_SERVICE_HR_MON_TABLE_REQ:
    {
        uint8_t *req = (uint8_t *)data_service_get_msg_body(msg);
        data_req_t *result = test_service_hr_mon_table(service, (uint16_t) req[0]);
        //datas_send_response(service, msg, result == NULL ? -RT_ERROR : RT_EOK);
        if (result)
        {
            datas_push_mon_table_to_client(service, result->len, result->data);
            rt_free(result);
        }
        break;
    }
    case MSG_SERVICE_HR_REGION_REQ:
    {
        uint8_t *req = (uint8_t *)data_service_get_msg_body(msg);
        data_req_t *result = test_service_hr_region(service, (uint16_t) req[0]);
        //datas_send_response(service, msg, result == NULL ? -RT_ERROR : RT_EOK);
        if (result)
        {
            datas_push_region_to_client(service, result->len, result->data);
            rt_free(result);
        }
        break;
    }
    case MSG_SERVICE_HR_MAX_MIN_REQ:
    {
        uint8_t *req = data_service_get_msg_body(msg);
        data_req_t *result = test_service_hr_max_min(service, (uint16_t) req[0]);
        //datas_send_response(service, msg, result == NULL ? -RT_ERROR : RT_EOK);
        if (result)
        {
            datas_push_maxmin_to_client(service, result->len, result->data);
            rt_free(result);
        }
        break;
    }
    case MSG_SERVICE_RHR_VALUE_REQ:
    {
        uint8_t *req = (uint8_t *)data_service_get_msg_body(msg);
        data_req_t *result = test_service_rhr_value(service, (uint16_t) req[0]);
        //datas_send_response(service, msg, result == NULL ? -RT_ERROR : RT_EOK);
        if (result)
        {
            datas_push_rhr_value_to_client(service, result->len, result->data);
            rt_free(result);
        }
        break;
    }
#endif
#ifdef BARO_TEST_ID
    case MSG_SERVICE_BARO_RANGE_REQ:
    {
        uint8_t *req = (uint8_t *)data_service_get_msg_body(msg);
        data_req_t *result = test_service_bora_range(service, (uint16_t) req[0]);
        //datas_send_response(service, msg, result == NULL ? -RT_ERROR : RT_EOK);
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
        data_req_t *result = test_service_elev_range(service, (uint16_t) req[0]);
        //datas_send_response(service, msg, result == NULL ? -RT_ERROR : RT_EOK);
        if (result)
        {
            datas_send_response_data(service, msg, result->len, result->data);
            rt_free(result);
        }
        break;
    }

#endif
#ifdef COMP_TEST_ID
    case MSG_SRV_COMPASS_CUR_VAL_GET_REQ:
    {
        int8_t index = test_service_getidx(service);

        if (index >= 0)
        {
            test_service_env_t *env;
            env = &test_service_env[index];

            compass_data_t *ptr = (compass_data_t *)(&(env->data->data[2]));
            //
            ptr->accuracy = 1;
            ptr->azimuth = ptr->azimuth + 1.0;
            if (ptr->azimuth > 360.0)
                ptr->azimuth = 0.0;
            datas_send_response_data(service, msg, sizeof(compass_data_t), (uint8_t *)ptr);
        }

        break;
    }
#endif

    default:
    {
        RT_ASSERT(0);
    }
    }

    return 0;
}


static data_service_config_t test_service_cb =
{
    .max_client_num = 5,
    .queue = RT_NULL,
    .data_filter = test_service_filter,
    .msg_handler = test_service_msg_handler,
};

static data_service_config_t test_service_cb2 =
{
    .max_client_num = 5,
    .queue = RT_NULL,
    .data_filter = test_service_filter,
    .msg_handler = test_service_msg_handler,
};

static void timeout_ind(void *param)
{
    uint8_t index = (uint8_t) param;

    g_count++;
    if (test_service_env[index].service && test_service_env[index].data)
    {
        datas_ind_size(test_service_env[index].service, test_service_env[index].data->len);
    }
}

static rt_mq_t g_svc_queue;

static int test_service_register(char *name, uint8_t index, int interval)
{
    test_service_env_t *env = &(test_service_env[index]);
    RT_ASSERT(index < MAX_TEST_SERVICE)

    if (interval < 0)
    {
        rt_thread_t tid;

        g_svc_queue = rt_mq_create(name, sizeof(data_msg_t), 30, RT_IPC_FLAG_FIFO);
        tid = rt_thread_create(name, data_service_entry, g_svc_queue, 1024, RT_THREAD_PRIORITY_MIDDLE, RT_THREAD_TICK_DEFAULT);
        rt_thread_startup(tid);
        test_service_cb2.queue = g_svc_queue;
        env->service = datas_register(name, &test_service_cb2);
    }
    else
        env->service = datas_register(name, &test_service_cb);

    if (interval > 0)
    {
        env->timer = rt_timer_create((const char *)name, timeout_ind, (void *)(uint32_t)index, interval, RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
        RT_ASSERT(env->timer);
    }


    RT_ASSERT(env->service);

    return index;
}

static void test_service_setdata(uint8_t index, char *data_str)
{
    test_service_env_t *env = &(test_service_env[index]);
    uint8_t len = (strlen(data_str) >> 1);

    if (env->data)
        rt_free(env->data);
    env->data = (data_req_t *)rt_malloc(len + sizeof(data_req_t));
    env->data->len = len;
    hex2data(data_str, env->data->data, len);
    if (env->timer == NULL)
    {
        g_count++;
        datas_push_data_to_client(env->service, env->data->len, env->data->data);
    }
}

static void test_service_setdata_len(uint8_t index, void *data, uint8_t length)
{
    test_service_env_t *env = &(test_service_env[index]);
    uint8_t len = length;
    void *ptr;

    if (env->data)
        rt_free(env->data);
    env->data = (data_req_t *)rt_malloc(len + sizeof(data_req_t));
    env->data->len = len;
    ptr = (void *)(env->data->data);
    memcpy(ptr, data, len);

    if (env->timer == NULL)
    {
        g_count++;
        datas_push_data_to_client(env->service, env->data->len, env->data->data);
    }
}


static void datas(int argc, char *argv[])
{
    if (argc >= 4)
    {
        if (strcmp(argv[1], "register") == 0)
            test_service_register(argv[2], atoi(argv[3]), atoi(argv[4]));
        else if (strcmp(argv[1], "data") == 0)
            test_service_setdata(atoi(argv[2]), argv[3]);
    }
}
MSH_CMD_EXPORT(datas, Test data service provider);

int simu_service_register(void)
{
    struct rt_sensor_data data;

    test_service_register("test", 0, -1);

#ifdef HR_TEST_ID
    test_service_register("HR", HR_TEST_ID, rt_tick_from_millisecond(100));
    data.type = RT_SENSOR_CLASS_HR;
    data.data.hr = 72;
    test_service_setdata_len(HR_TEST_ID, &data, sizeof(struct rt_sensor_data));
#endif
#ifdef COMP_TEST_ID
    compass_data_t comp;
    test_service_register("COMP", COMP_TEST_ID, 0);
    comp.accuracy = 1;
    comp.azimuth = 90.0;
    comp.pitch = 0.0;
    comp.roll = 0.0;
    test_service_setdata_len(COMP_TEST_ID, &comp, sizeof(compass_data_t));
#endif
#ifdef BARO_TEST_ID
    test_service_register("BARO", BARO_TEST_ID, rt_tick_from_millisecond(500));
    data.type = RT_SENSOR_CLASS_BARO;
    data.data.baro = 90000;
    test_service_setdata_len(BARO_TEST_ID, &data, sizeof(struct rt_sensor_data));
#endif
#ifdef PM_TEST_ID
    {
        //extern int power_manager_service_register(void);
        //power_manager_service_register();
    }
#endif
#if defined(ALARM_TEST_ID)&&defined(_MSC_VER)
    {
        //extern int alarm_manager_service_register(void);
        //alarm_manager_service_register();
    }
#endif

    return 0;
}
INIT_COMPONENT_EXPORT(simu_service_register);



