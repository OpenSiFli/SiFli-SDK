/**
  ******************************************************************************
  * @file   hr_service.c
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
#include "hr_service.h"
#include "sensor.h"

#define DBG_TAG           "DS.HR"
#define DBG_LVL           DBG_INFO
#include "rtdbg.h"

#ifdef HR_USING_GH3011
    #include "sensor_goodix_gh3011.h"
    #define HR_MODEL_NAME   "gh3011"

#elif defined (HR_USING_AFE4404)
    #include "sensor_ti_afe4404.h"
    #define HR_MODEL_NAME   "afe4404"

#endif  //HR_USING_AFE4404
#define HR_DEV_NAME "hr_"HR_MODEL_NAME


#define HR_TIMER_PERIOD_MS      (500)

// timestamp to minute, current ts unit is 1 second
#define HR_TS2MIN           (60*1)
// how long to save hr, in minute
#define HR_HOUR_GAP_MIN     (60)
//#define HR_HOUR_GAP_TS     (HR_HOUR_GAP_MIN*HR_TS2MIN)
// data count for 1 day
#define HR_HOUR_CNT         (24*60/HR_HOUR_GAP_MIN)

enum
{
    HR_GET_DAY_HOURS = (1 << 0),
    HR_GET_MON_RHR = (1 << 1),
    HR_GET_REGION = (1 << 2),
    HR_GET_MAX = (1 << 3),
    HR_GET_MIN = (1 << 4),
    HR_GET_RHR = (1 << 5)
};

typedef struct
{
    uint8_t timestamp;
    uint8_t hr_value;
} hr_service_value_t;

enum
{
    HR_ULTIMATE_LIMIT = 0,
    HR_ANAEROBIC,
    HR_AEROBIC,
    HR_HIIT,
    HR_WARM_UP,
    HR_SPORT_CNT
};
// define heart rate threshold for each sport mode, base on rhr
#define HR_ULTIMATE_THD(x)           ((x)*3)
#define HR_ANAEROBIC_THD(x)          ((x)*5/2 )
#define HR_AEROBIC_THD(x)            ((x)*2)
#define HR_HIIT_THD(x)               ((x)*3/2)
#define HR_WARMUP_THD(x)             ((x)*5/4)

// continue counter for same hr to rhr
#define HR_RHR_THRESHOLD        (60)
typedef struct
{
    uint8_t rhr_cnt;
    uint8_t rhr_value;
} hr_service_rhr_t;

typedef struct
{
    hr_service_value_t hour[HR_HOUR_CNT];    // today 24h hr, 1 value per 30 min(HR_HOUR_GAP_SEC) : half hour + value
    hr_service_value_t rhr_arr[30];   // priv 30 days resting heart rate : day + value
    uint8_t status_arr[HR_SPORT_CNT];    // minute for each mode
    uint8_t max_hr;     // today max heart rate
    uint8_t min_hr;     // today min heart rate
    uint8_t rhr;        // current resting heart rate
    time_t today;       // save today time stamp, to detect date changed
} hr_service_data_t;

typedef struct
{
    int ref_count;
    rt_device_t device;
    datas_handle_t service;

    //TODO: need FIFO
    struct rt_sensor_data data;
    rt_timer_t timer;
    hr_service_data_t env;  // data need saved, app and ui need them.
    uint8_t cur_mode;   // current sport mode
    uint8_t cur_mcnt;   // current sport mode duration, mode minute increase after it over a threshold
    hr_service_rhr_t get_rhr;   // sturcture for checking resting heart rate(RHR)
} hr_service_env_t;

static hr_service_env_t hr_service_env;
static uint8_t hr_update_flag = 0;

static int32_t hr_subscribe(datas_handle_t service)
{

    if (hr_service_env.timer && hr_service_env.ref_count == 0)
    {
        rt_timer_start(hr_service_env.timer);
    }

    hr_service_env.ref_count++;
    return 0;
}

static int32_t hr_unsubscribe(datas_handle_t service)
{
    hr_service_env.ref_count--;
    if (hr_service_env.timer && hr_service_env.ref_count == 0)
    {
        rt_timer_stop(hr_service_env.timer);
    }

    return 0;
}

static rt_err_t hr_service_config(datas_handle_t service, uint8_t *config)
{
    // Update service configuration. Save config for future filtering.
    return RT_EOK;
}

static rt_err_t hr_service_ping(datas_handle_t service, uint8_t *data)
{
    hr_service_env_t *env = &hr_service_env;

    if (!env->device)
    {
        return RT_ERROR;
    }
    LOG_D("hr_service_ping %d", *data);

    return rt_device_control(env->device, RT_SENSOR_CTRL_SELF_TEST, data);
}

static rt_err_t hr_service_save(datas_handle_t service, uint8_t flag)
{
    hr_service_env_t *env = &hr_service_env;

    if (!env->device)
    {
        return RT_ERROR;
    }
    // check flag, save max hr, min hr, peace hr, normal data?

    return RT_EOK;
}

static data_req_t *hr_service_get_max_min(datas_handle_t service, uint16_t len)
{
    hr_service_env_t *env = &hr_service_env;
    data_req_t *r = NULL;

    r = rt_malloc(len + sizeof(data_req_t));
    if (r != NULL)
    {
        r->len = len;
        if (len == HRS_MAX_MIN_LEN)   // just return max , min , current rhr for test
        {
            uint8_t maxmin[HRS_MAX_MIN_LEN];
            maxmin[0] = env->env.max_hr;
            maxmin[1] = env->env.min_hr;
            maxmin[2] = env->env.rhr;
            memcpy(r->data, maxmin, len);
        }
    }

    return r;
}

static data_req_t *hr_service_get_day_table(datas_handle_t service, uint16_t len)
{
    hr_service_env_t *env = &hr_service_env;
    data_req_t *r = NULL;

    r = rt_malloc(len + sizeof(data_req_t));
    if (r != NULL)
    {
        if (len == HRS_DAY_TABLE_LEN) // get day value : 24 hour
        {
            uint8_t today[HRS_DAY_TABLE_LEN];
            int i;

            r->len = len;
            for (i = 0; i < HRS_DAY_TABLE_LEN; i++)
                today[i] = env->env.hour[i].hr_value;
            memcpy(r->data, today, len);
        }
    }
    return r;
}

static data_req_t *hr_service_get_mon_table(datas_handle_t service, uint16_t len)
{
    hr_service_env_t *env = &hr_service_env;
    data_req_t *r = NULL;

    r = rt_malloc(len + sizeof(data_req_t));
    if (r != NULL)
    {
        r->len = len;
        if (len == HRS_MON_TABLE_LEN) // get rhr: 30 days
        {
            uint8_t mon[HRS_MON_TABLE_LEN];
            int i;
            for (i = 0; i < HRS_MON_TABLE_LEN; i++)
                mon[i] = env->env.rhr_arr[i].hr_value;
            memcpy(r->data, mon, len);
        }
    }
    return r;
}

static data_req_t *hr_service_get_region(datas_handle_t service, uint16_t len)
{
    hr_service_env_t *env = &hr_service_env;
    data_req_t *r = NULL;

    r = rt_malloc(len + sizeof(data_req_t));
    if (r != NULL)
    {
        r->len = len;
        if (len == HRS_REGION_LEN) // get region, ul, ana, aer, hiit, warmup
        {
            uint8_t region[HRS_REGION_LEN];
            int i;
            for (i = 0; i < HRS_REGION_LEN; i++)
                region[i] = env->env.status_arr[i];
            memcpy(r->data, region, len);
        }
    }
    return r;
}

static data_req_t *hr_service_get_hist_rhr(datas_handle_t service, uint16_t len)
{
    hr_service_env_t *env = &hr_service_env;
    data_req_t *r = NULL;

    r = rt_malloc(len + sizeof(data_req_t));
    if (r != NULL)
    {
        r->len = len;
        if (len == HRS_RHR_HIST_LEN) // get  max rhr, min rhr, average rhr
        {
            uint8_t rhr[HRS_RHR_HIST_LEN];
            int i;
            uint32_t total, ave;
            // get max rhr
            rhr[0] = env->env.rhr_arr[0].hr_value;
            rhr[1] = env->env.rhr_arr[0].hr_value;
            total = 0;
            for (i = 0; i < HRS_MON_TABLE_LEN; i++)
            {
                if (env->env.rhr_arr[i].hr_value > rhr[0])
                    rhr[0] = env->env.rhr_arr[i].hr_value;
                if (env->env.rhr_arr[i].hr_value < rhr[1])
                    rhr[1] = env->env.rhr_arr[i].hr_value;
                total += (uint32_t)env->env.rhr_arr[i].hr_value;
            }
            rhr[2] = (uint8_t)(total / HRS_MON_TABLE_LEN);
            memcpy(r->data, rhr, len);
        }
    }
    return r;
}

// update data buffer after day switch
static int32_t hr_service_update(datas_handle_t service)
{
    hr_service_env_t *env = &hr_service_env;
    //hr_service_env_t *env = rt_container_of(&service, hr_service_env_t, service);
    int i;
    time_t now  = time(NULL);
    struct tm today = *localtime(&now);
    struct tm oldday = *localtime(&(env->env.today));
    if (today.tm_yday != oldday.tm_yday) // a new day, update saved day
    {
        today.tm_hour = 0;
        today.tm_min = 0;
        today.tm_sec = 0;
        env->env.today = mktime(&today);
        LOG_D("Hello new day\n");
    }
    else // day not change, do not need reset table
    {
        return 1;
    }

    env->cur_mode = HR_SPORT_CNT;
    env->cur_mcnt = 0;
    env->get_rhr.rhr_cnt = 0;
    env->get_rhr.rhr_value = 0;

    for (i = 0; i < HR_HOUR_CNT; i++)
        env->env.hour[i].hr_value = 0;

    for (i = 0; i < 29; i++)
        env->env.rhr_arr[i].hr_value = env->env.rhr_arr[i + 1].hr_value;
    env->env.rhr_arr[29].hr_value = env->env.rhr;
    env->env.rhr = 0;


    for (i = 0; i < HR_SPORT_CNT; i++)
        env->env.status_arr[i] = 0;

    env->env.max_hr = 0;
    env->env.min_hr = 0xff;

    hr_update_flag = HR_GET_DAY_HOURS | HR_GET_MON_RHR | HR_GET_REGION | HR_GET_MAX | HR_GET_MIN | HR_GET_RHR;

    return 0;
}

static int32_t hr_service_proc(datas_handle_t service, uint32_t data_size, uint8_t *data)
{
    hr_service_env_t *env = &hr_service_env;
    //hr_service_env_t *env = rt_container_of(&service, hr_service_env_t, service);
    struct rt_sensor_data *value = (struct rt_sensor_data *)data;

#define HRABS(x,y)           ((x)>(y)?(x)-(y):(y)-(x))

    if (value->data.hr <= 0) // no valid hr, do not save and process
        return 1;

    hr_service_update(service);

    // 1. max, min check
    if (value->data.hr > env->env.max_hr)
    {
        LOG_D("Udpate Max HR from %d to %d\n", env->env.max_hr, value->data.hr);
        env->env.max_hr = value->data.hr;
        hr_update_flag |= HR_GET_MAX;
    }
    if (value->data.hr < env->env.min_hr)
    {
        env->env.min_hr = value->data.hr;
        LOG_D("Udpate Min HR to %d\n", env->env.min_hr);
        hr_update_flag |= HR_GET_MIN;
    }

    // 2. check timer , if save this value(1 update per 30 min)
    // use system timer instead time stamp check
    {
        time_t now  = time(NULL);
        struct tm *today = localtime(&now);
        uint8_t hour_idx;
        hour_idx = today->tm_hour * 60 / HR_HOUR_GAP_MIN + today->tm_min / HR_HOUR_GAP_MIN;
        if (env->env.hour[hour_idx].hr_value == 0)
        {
            env->env.hour[hour_idx].hr_value = value->data.hr;
            LOG_D("Update hour table %d: %d\n", hour_idx, value->data.hr);
            hr_update_flag |= HR_GET_DAY_HOURS;
        }
    }

    // 3. update RHR, diaplay and today
    if ((value->data.hr > 50) && (value->data.hr < 80))
    {
        if (env->get_rhr.rhr_cnt == 0)
        {
            env->get_rhr.rhr_value = value->data.hr;
            env->get_rhr.rhr_cnt++;
        }
        else
        {
            if (HRABS(value->data.hr, env->get_rhr.rhr_value) < 5)
                env->get_rhr.rhr_cnt++;
            else
                env->get_rhr.rhr_cnt = 0;
            if (env->get_rhr.rhr_cnt >= HR_RHR_THRESHOLD) // update rhr
            {
                LOG_D("Update RHR TO %d\n", env->get_rhr.rhr_value);
                env->get_rhr.rhr_cnt = 0;
                env->env.rhr = env->get_rhr.rhr_value;
                hr_update_flag |= HR_GET_RHR;

                // update last day rhr, move it to 0:00 process
                //if(env->env.rhr_arr[29] != 0)
                //{
                //    env->env.rhr_arr[29] = env->env.rhr;
                //}
                //else // to a new day
                //{
                //    int i;
                //    for(i=0; i<29; i++)
                //        env->env.rhr_arr[i]= env->env.rhr_arr[i+1];
                //    env->env.rhr_arr[29] = env->env.rhr;
                //}
            }
        }
    }
    else
    {
        env->get_rhr.rhr_cnt = 0;
    }

    // 4. sport mode update
    if (env->env.rhr > 0)
    {
        if (value->data.hr >= HR_ULTIMATE_THD(env->env.rhr))
        {
            if (env->cur_mode == HR_ULTIMATE_LIMIT)
            {
                env->cur_mcnt++;
                if (env->cur_mcnt >= (1000 * 60 / HR_TIMER_PERIOD_MS))
                {
                    env->env.status_arr[HR_ULTIMATE_LIMIT]++;
                    env->cur_mcnt = 0;
                    LOG_D("Ultimate limit %d min\n", env->env.status_arr[HR_ULTIMATE_LIMIT]);
                    hr_update_flag |= HR_GET_REGION;
                }
            }
            else
            {
                env->cur_mode = HR_ULTIMATE_LIMIT;
                env->cur_mcnt = 0;
            }
        }
        else if (value->data.hr >= HR_ANAEROBIC_THD(env->env.rhr))
        {
            if (env->cur_mode == HR_ANAEROBIC)
            {
                env->cur_mcnt++;
                if (env->cur_mcnt >= (1000 * 60 / HR_TIMER_PERIOD_MS))
                {
                    env->env.status_arr[HR_ANAEROBIC]++;
                    env->cur_mcnt = 0;
                    LOG_D("Anaerobic %d min\n", env->env.status_arr[HR_ANAEROBIC]);
                    hr_update_flag |= HR_GET_REGION;
                }
            }
            else
            {
                env->cur_mode = HR_ANAEROBIC;
                env->cur_mcnt = 0;
            }
        }
        else if (value->data.hr >= HR_AEROBIC_THD(env->env.rhr))
        {
            if (env->cur_mode == HR_AEROBIC)
            {
                env->cur_mcnt++;
                if (env->cur_mcnt >= (1000 * 60 / HR_TIMER_PERIOD_MS))
                {
                    env->env.status_arr[HR_AEROBIC]++;
                    env->cur_mcnt = 0;
                    LOG_D("Aerobic %d min\n", env->env.status_arr[HR_AEROBIC]);
                    hr_update_flag |= HR_GET_REGION;
                }
            }
            else
            {
                env->cur_mode = HR_AEROBIC;
                env->cur_mcnt = 0;
            }
        }
        else if (value->data.hr >= HR_HIIT_THD(env->env.rhr))
        {
            if (env->cur_mode == HR_HIIT)
            {
                env->cur_mcnt++;
                if (env->cur_mcnt >= (1000 * 60 / HR_TIMER_PERIOD_MS))
                {
                    env->env.status_arr[HR_HIIT]++;
                    env->cur_mcnt = 0;
                    LOG_D("Hiit %d min\n", env->env.status_arr[HR_HIIT]);
                    hr_update_flag |= HR_GET_REGION;
                }
            }
            else
            {
                env->cur_mode = HR_HIIT;
                env->cur_mcnt = 0;
            }
        }
        else if (value->data.hr >= HR_WARMUP_THD(env->env.rhr))
        {
            if (env->cur_mode == HR_WARM_UP)
            {
                env->cur_mcnt++;
                if (env->cur_mcnt >= (1000 * 60 / HR_TIMER_PERIOD_MS))
                {
                    env->env.status_arr[HR_WARM_UP]++;
                    env->cur_mcnt = 0;
                    LOG_D("Warm up %d min\n", env->env.status_arr[HR_WARM_UP]);
                    hr_update_flag |= HR_GET_REGION;
                }
            }
            else
            {
                env->cur_mode = HR_WARM_UP;
                env->cur_mcnt = 0;
            }
        }
        else
        {
            env->cur_mode = HR_SPORT_CNT;
            env->cur_mcnt = 0;
        }
    }

    if ((hr_update_flag & HR_GET_MAX) || (hr_update_flag & HR_GET_MIN) || (hr_update_flag & HR_GET_RHR))
    {
        data_req_t *result = hr_service_get_max_min(service, HRS_MAX_MIN_LEN);
        if (result)
        {
            datas_push_maxmin_to_client(service, result->len, result->data);
            rt_free(result);
        }
    }
    if (hr_update_flag & HR_GET_DAY_HOURS)
    {
        data_req_t *result = hr_service_get_day_table(service, HRS_DAY_TABLE_LEN);
        if (result)
        {
            datas_push_day_table_to_client(service, result->len, result->data);
            rt_free(result);
        }
    }
    if (hr_update_flag & HR_GET_MON_RHR)
    {
        data_req_t *result = hr_service_get_mon_table(service, HRS_MON_TABLE_LEN);
        if (result)
        {
            datas_push_mon_table_to_client(service, result->len, result->data);
            rt_free(result);
            result = NULL;
        }
        result = hr_service_get_hist_rhr(service, HRS_RHR_HIST_LEN);
        if (result)
        {
            datas_push_rhr_value_to_client(service, result->len, result->data);
            rt_free(result);
        }
    }
    if (hr_update_flag & HR_GET_REGION)
    {
        data_req_t *result = hr_service_get_region(service, HRS_REGION_LEN);
        if (result)
        {
            datas_push_region_to_client(service, result->len, result->data);
            rt_free(result);
        }
    }
    hr_update_flag = 0;

    return 0;
}

static int32_t hr_service_data_fetch(datas_handle_t service, uint32_t data_size, uint8_t **data)
{
    hr_service_env_t *env = &hr_service_env;
    rt_size_t size;

    RT_ASSERT(data);
    *data = NULL;
    if (!env->device)
    {
        return -1;
    }

    *data = (uint8_t *)&env->data;
    size = rt_device_read(env->device, 0, *data, 1);
    RT_ASSERT(1 == size);

    return sizeof(env->data);
}

bool hr_service_filter(data_req_t *config, uint16_t msg_id, uint32_t len, uint8_t *data)
{
    // Check if config is compatible with current config.
    return true;
}

static int32_t hr_service_msg_handler(datas_handle_t service, data_msg_t *msg)
{
    //hr_service_update(service);
    switch (msg->msg_id)
    {
    case MSG_SERVICE_SUBSCRIBE_REQ:
    {
        hr_subscribe(service);
        // start a thread for data process ?
        break;
    }
    case MSG_SERVICE_UNSUBSCRIBE_REQ:
    {
        hr_unsubscribe(service);
        break;
    }
    case MSG_SERVICE_CONFIG_REQ:
    {
        data_req_t *req = (data_req_t *)data_service_get_msg_body(msg);
        rt_err_t result = hr_service_config(service, &req->data[0]);
        datas_send_response(service, msg, result);
        break;
    }
    case MSG_SERVICE_PING_REQ:
    {
        data_req_t *req = (data_req_t *)data_service_get_msg_body(msg);
        rt_err_t result = hr_service_ping(service, &req->data[0]);
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

        int32_t size = hr_service_data_fetch(service, data_ind->len, &data);
        if (size > 0)
        {
            datas_push_data_to_client(service, data_ind->len, data);
        }
        // add data process
        hr_service_proc(service, data_ind->len, data);
        break;
    }
    case MSG_SERVICE_SLEEP_REQ:
    {
        data_req_t *req = (data_req_t *)data_service_get_msg_body(msg);
        // before go to sleep, save data, stop algorithm if needed
        rt_err_t result = hr_service_save(service, 0xff);
        datas_send_response(service, msg, result);
        break;
    }
    case MSG_SERVICE_HR_DAY_TABLE_REQ:
    {
        uint8_t *req = (uint8_t *)data_service_get_msg_body(msg);
        data_req_t *result = hr_service_get_day_table(service, (uint16_t) req[0]);
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
        data_req_t *result = hr_service_get_mon_table(service, (uint16_t) req[0]);
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
        data_req_t *result = hr_service_get_region(service, (uint16_t) req[0]);
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
        data_req_t *result = hr_service_get_max_min(service, (uint16_t) req[0]);
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
        data_req_t *result = hr_service_get_hist_rhr(service, (uint16_t) req[0]);
        //datas_send_response(service, msg, result == NULL ? -RT_ERROR : RT_EOK);
        if (result)
        {
            datas_push_rhr_value_to_client(service, result->len, result->data);
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


static data_service_config_t hr_service_cb =
{
    .max_client_num = 5,
    .queue = RT_NULL,
    .data_filter = hr_service_filter,
    .msg_handler = hr_service_msg_handler,
};

static void timeout_ind(void *param)
{
    if (hr_service_env.service)
        datas_ind_size(hr_service_env.service, sizeof(hr_service_env.data));
}


int hr_service_register(void)
{
    struct rt_sensor_config cfg;
    hr_service_env_t *env = &hr_service_env;
    rt_err_t err;
    int i;
    int res = 0;

#ifdef HR_USING_GH3011
    cfg.intf.dev_name = "i2c4";
    res = rt_hw_gh3011_init(HR_MODEL_NAME, &cfg);
#elif defined (HR_USING_AFE4404)
    cfg.intf.dev_name = "i2c3";
    res = rt_hw_afe4404_init(HR_MODEL_NAME, &cfg);
#endif
    //RT_ASSERT(res == 0);
    if (res != 0)
    {
        LOG_W("DataS init %s fail\n", HR_MODEL_NAME);
        return 1;
    }
    env->device = rt_device_find(HR_DEV_NAME);
    //RT_ASSERT(env->device);
    if (env->device == NULL)
    {
        LOG_W("DataS find device %s fail\n", HR_DEV_NAME);
        return 1;
    }

    err = rt_device_open(env->device, RT_DEVICE_OFLAG_RDONLY);
    //RT_ASSERT(RT_EOK == err);
    if (err != RT_EOK)
    {
        LOG_W("DataS open device %s fail\n", HR_DEV_NAME);
        return 1;
    }

    hr_service_env.service = datas_register("HR", &hr_service_cb);
    hr_service_env.timer = rt_timer_create("HR", timeout_ind, 0, rt_tick_from_millisecond(HR_TIMER_PERIOD_MS), RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    RT_ASSERT(hr_service_env.timer);
    RT_ASSERT(hr_service_env.service);

    hr_service_env.env.max_hr = 0;
    hr_service_env.env.min_hr = 0xff;
    for (i = 0; i < HR_HOUR_CNT; i++)
    {
        hr_service_env.env.hour[i].timestamp = i;
        hr_service_env.env.hour[i].hr_value = 0;
    }

    {
        // system timer enabled
        time_t now  = time(NULL);
        struct tm today = *localtime(&now);
        today.tm_hour = 0;
        today.tm_min = 0;
        today.tm_sec = 0;
        hr_service_env.env.today = mktime(&today);
        //LOG_D("Init day: %d, %d:%d:%d\n", today.tm_yday, today.tm_year, today.tm_mon, today.tm_mday);
    }
    return 0;
}

INIT_COMPONENT_EXPORT(hr_service_register);


