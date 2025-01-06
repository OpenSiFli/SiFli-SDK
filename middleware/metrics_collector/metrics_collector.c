/**
  ******************************************************************************
  * @file   metrics_collector.c
  * @author Sifli software development team
  * @brief Metrics Collector source
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

#include <rtthread.h>
#include "board.h"
#include <string.h>
#include "metrics_collector.h"

#if defined(MC_CLIENT_ENABLED) && defined(MC_SERVICE_ENABLED)
    #error "client and service cannot be enabled at the same time"
#endif

#ifdef MC_CLIENT_ENABLED
    #include "data_service.h"
#endif /* MC_CLIENT_ENABLED */

#ifdef MC_SERVICE_ENABLED
    #include "data_service.h"
#endif /* MC_SERVICE_ENABLED */

#include "log.h"


typedef struct
{
    uint16_t num;
    rt_list_t collector_list;
    struct rt_timer timer;
} mc_collector_mng_t;

typedef struct
{
    bool init;
    mc_collector_mng_t mng[MC_PERIOD_MAX];
    struct rt_mutex lock;
#ifdef MC_CLIENT_ENABLED
    datac_handle_t client_handle;
    rt_sem_t sem;
#else
    mc_db_t default_db;
    rt_list_t db_list;

    rt_mq_t queue;
#endif /* MC_CLIENT_ENABLED */

#ifdef MC_SERVICE_ENABLED
    datas_handle_t service_handle;
#endif /* MC_SERVICE_ENABLED */

    mc_err_t err_code;
    uint32_t loss_cnt;
} mc_ctx_t;

typedef struct
{
    uint16_t id:  13;
    uint16_t core: 3;
    uint16_t len;
    uint32_t time;
} mc_metrics_hdr_t;

typedef struct
{
    mc_metrics_hdr_t header;
    uint8_t data[0];
} mc_metrics_t;

#define MC_METRICS_TOTAL_LEN(data_len)   (sizeof(mc_metrics_t) + (data_len))

typedef struct
{
    mc_db_t *db;
    mc_metrics_t *data;
    bool freed;
} mc_mq_msg_t;


static mc_ctx_t mc_ctx;
static struct rt_thread g_metrics_thread;
static uint32_t g_metrics_thread_stack[512];

/* use mc_period_t as index */
static const uint32_t mc_timer_period[MC_PERIOD_MAX] = {24 * 3600 * 1000, 3600 * 1000, 60 * 1000, 1000};

static uint32_t get_time(void)
{
    time_t t = 0;

    time(&t);
    return t;
}

static inline void mc_enter_critical(void)
{
    rt_err_t err;

    err = rt_mutex_take(&mc_ctx.lock, rt_tick_from_millisecond(1000));
    RT_ASSERT(RT_EOK == err);
}

static inline void mc_exit_critical(void)
{
    rt_err_t err;

    err = rt_mutex_release(&mc_ctx.lock);
    RT_ASSERT(RT_EOK == err);
}


static void mc_timer_timeout_handler(void *parameter)
{
    mc_collector_mng_t *mng;
    mc_period_t period = (mc_period_t)parameter;
    rt_list_t *iter;
    mc_collector_t *collector;

    RT_ASSERT(period < MC_PERIOD_MAX);

    mng = &mc_ctx.mng[period];

    mc_enter_critical();
    rt_list_for_each(iter, &mng->collector_list)
    {
        collector = rt_list_entry(iter, mc_collector_t, node);
        RT_ASSERT(collector->callback);
        collector->callback(collector->user_data);
    }
    mc_exit_critical();
}

#ifdef MC_SERVICE_ENABLED
static mc_metrics_t *mc_copy_metrics(mc_metrics_t *metrics)
{
    mc_metrics_t *copy;
    uint32_t total_len;

    RT_ASSERT(metrics);

    total_len = MC_METRICS_TOTAL_LEN(metrics->header.len);

    RT_ASSERT(total_len <= MC_MAX_DATA_LEN);

    copy = rt_malloc(total_len);
    RT_ASSERT(copy);

    memcpy((void *)copy, metrics, total_len);

    return copy;
}
#endif /* MC_SERVICE_ENABLED */

mc_err_t mc_register_collector(mc_collector_t *collector)
{
    rt_list_t *list;
    rt_timer_t timer;
    mc_collector_mng_t *mng;
    rt_err_t err;

    if (!mc_ctx.init)
    {
        return MC_ERROR;
    }
    if ((collector->period >= MC_PERIOD_MAX) || !collector->callback)
    {
        return MC_ERROR;
    }

    rt_list_init(&collector->node);
    mng = &mc_ctx.mng[collector->period];
    list = &mng->collector_list;

    mc_enter_critical();
    if (rt_list_isempty(list))
    {
        timer = &mng->timer;
    }
    else
    {
        timer = NULL;
    }
    rt_list_insert_before(list, &collector->node);
    mng->num++;
    mc_exit_critical();

    if (timer)
    {
        RT_ASSERT(MC_PERIOD_SPEEDUP_FACTOR > 0);
        rt_timer_init(timer, "mc", mc_timer_timeout_handler,
                      (void *)collector->period, rt_tick_from_millisecond(mc_timer_period[collector->period]) / MC_PERIOD_SPEEDUP_FACTOR,
                      RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
        err = rt_timer_start(timer);
        RT_ASSERT(RT_EOK == err);
    }

    return MC_OK;
}

mc_err_t mc_deregister_collector(mc_collector_t *collector)
{
    mc_collector_mng_t *mng;
    bool stop_timer;
    rt_err_t err;

    if (!mc_ctx.init)
    {
        return MC_ERROR;
    }
    if (!collector)
    {
        return MC_ERROR;
    }
    RT_ASSERT(collector->period < MC_PERIOD_MAX);

    mng = &mc_ctx.mng[collector->period];

    mc_enter_critical();
    if (rt_list_isempty(&collector->node))
    {
        mc_exit_critical();
        return MC_ERROR;
    }

    rt_list_remove(&collector->node);
    if (rt_list_isempty(&mng->collector_list))
    {
        stop_timer = true;
    }
    else
    {
        stop_timer = false;
    }
    RT_ASSERT(mng->num > 0);
    mng->num--;
    mc_exit_critical();

    if (stop_timer)
    {
        err = rt_timer_stop(&mng->timer);
        RT_ASSERT(RT_EOK == err);
        err = rt_timer_detach(&mng->timer);
        RT_ASSERT(RT_EOK == err);
    }

    return MC_OK;

}

void *mc_alloc_metrics(uint16_t id, uint16_t data_len)
{
    mc_metrics_t *metrics;
    uint32_t total_len = sizeof(*metrics) + data_len;

    if (MC_MAX_DATA_LEN < total_len)
    {
        return NULL;
    }

    metrics = rt_malloc(total_len);
    RT_ASSERT(metrics);

    metrics->header.id = id;
    metrics->header.core = CORE_ID_CURRENT;
    metrics->header.len = data_len;
    metrics->header.time = get_time();

    return (void *)&metrics->data[0];
}

mc_err_t mc_save_metrics_ex(mc_db_t *db, void *metrics, bool freed)
{
    mc_metrics_hdr_t *header;
#ifdef MC_CLIENT_ENABLED
    data_msg_t msg;
    mc_metrics_save_req_t *save_req;
#else
    mc_mq_msg_t msg;
#endif /* MC_CLIENT_ENABLED */
    rt_err_t err;

    if (!mc_ctx.init)
    {
        return MC_OK;
    }
    if (!db || !metrics)
    {
        return MC_ERROR;
    }

    header = (mc_metrics_hdr_t *)((uint32_t)metrics - sizeof(mc_metrics_hdr_t));

#ifdef MC_CLIENT_ENABLED
    mc_enter_critical();

    mc_ctx.sem = rt_sem_create("mc", 0, RT_IPC_FLAG_FIFO);
    RT_ASSERT(mc_ctx.sem);

    save_req = (mc_metrics_save_req_t *)data_service_init_msg(&msg, MSG_SERVICE_METRICS_SAVE_REQ,
               MC_METRICS_TOTAL_LEN(header->len));
    memcpy((void *)&save_req->data[0], (void *)header, MC_METRICS_TOTAL_LEN(header->len));
    err = datac_send_msg(mc_ctx.client_handle, &msg);
    RT_ASSERT(RT_EOK == err);

    err = rt_sem_take(mc_ctx.sem, 1000);
    RT_ASSERT(RT_EOK == err);
    rt_sem_delete(mc_ctx.sem);
    mc_ctx.sem = NULL;

    mc_exit_critical();

    if (freed)
    {
        rt_free(header);
    }

#else
    msg.db = db;
    msg.data = (mc_metrics_t *)header;
    msg.freed = freed;
    err = rt_mq_send(mc_ctx.queue, &msg, sizeof(msg));
    if (RT_EOK != err)
    {
        mc_ctx.loss_cnt++;
        if (freed)
        {
            rt_free(header);
        }
    }
#endif /* MC_CLIENT_ENABLED */

    return MC_OK;
}

mc_err_t mc_save_metrics(void *metrics, bool freed)
{
    return mc_save_metrics_ex(&mc_ctx.default_db, metrics, freed);
}


mc_err_t mc_free_metrics(void *metrics)
{
    mc_metrics_hdr_t *header;

    if (!metrics)
    {
        return MC_ERROR;
    }

    header = (mc_metrics_hdr_t *)((uint32_t)metrics - sizeof(mc_metrics_hdr_t));
    rt_free(header);

    return MC_OK;
}

mc_err_t mc_flush(void)
{
    mc_err_t err = MC_OK;

    mc_enter_critical();
    if (mc_ctx.default_db.db)
    {
        err = mc_backend_flush(mc_ctx.default_db.db);
        mc_ctx.err_code = err;
    }
    mc_exit_critical();

    return err;
}
MSH_CMD_EXPORT(mc_flush, mc_flush);

mc_err_t mc_close(void)
{
    mc_err_t err = MC_OK;

    mc_enter_critical();
    if (mc_ctx.default_db.db)
    {
        err = mc_backend_close(mc_ctx.default_db.db);
        mc_ctx.default_db.db = NULL;
        mc_ctx.err_code = err;
    }
    mc_exit_critical();

    return err;
}
MSH_CMD_EXPORT(mc_close, mc_close);


mc_err_t mc_flush_ex(mc_db_t *db)
{
    mc_err_t err;

    if (!db)
    {
        return MC_ERROR;
    }

    mc_enter_critical();
    err = mc_backend_flush(db->db);
    mc_ctx.err_code = err;
    mc_exit_critical();

    return err;
}

#ifndef MC_CLIENT_ENABLED

static bool mc_raw_metrics_iter_cb(void *data, uint32_t data_len, void *arg)
{
    mc_metrics_hdr_t *hdr = (mc_metrics_hdr_t *)data;
    mc_raw_metrics_read_callback_t user_cb = (mc_raw_metrics_read_callback_t)arg;

    RT_ASSERT(data_len == MC_METRICS_TOTAL_LEN(hdr->len));

    return user_cb(data, data_len, hdr->time);
}

static bool mc_parsed_metrics_iter_cb(void *data, uint32_t data_len, void *arg)
{
    mc_metrics_hdr_t *hdr = (mc_metrics_hdr_t *)data;
    mc_metrics_read_callback_t user_cb = (mc_metrics_read_callback_t)arg;

    RT_ASSERT(data_len == MC_METRICS_TOTAL_LEN(hdr->len));

    return user_cb(hdr->id, hdr->core, hdr->len, hdr->time, (void *)(hdr + 1));
}


mc_err_t mc_read_raw_metrics(mc_raw_metrics_read_callback_t cb)
{
    mc_err_t err = MC_OK;

    if (!mc_ctx.init)
    {
        return MC_ERROR;
    }

    mc_enter_critical();
    if (mc_ctx.default_db.db)
    {
        mc_backend_iter(mc_ctx.default_db.db, mc_raw_metrics_iter_cb, (void *)cb);
    }
    else
    {
        err = MC_ERROR;
    }
    mc_exit_critical();

    return err;
}


mc_err_t mc_read_metrics(mc_metrics_read_callback_t cb)
{
    mc_err_t err = MC_OK;

    if (!mc_ctx.init)
    {
        return MC_ERROR;
    }

    mc_enter_critical();
    if (mc_ctx.default_db.db)
    {
        mc_backend_iter(mc_ctx.default_db.db, mc_parsed_metrics_iter_cb, (void *)cb);
    }
    else
    {
        err = MC_ERROR;
    }
    mc_exit_critical();

    return err;
}

mc_err_t mc_clear_metrics(void)
{
    if (!mc_ctx.init)
    {
        return MC_ERROR;
    }

    return mc_backend_clear(mc_ctx.default_db.db);
}

mc_err_t mc_init_db(mc_db_t *db, const char *name, uint32_t max_size)
{
    mc_err_t err;

    if (!mc_ctx.init)
    {
        err = MC_NOT_INIT;
        goto __EXIT;
    }
    if (db->db)
    {
        err = MC_OK;
        goto __EXIT;
    }

    if (!db)
    {
        err = MC_ERROR;
        goto __EXIT;
    }

    db->name = name;
    db->max_size = max_size;
    db->db = mc_backend_init(db->name, db->max_size);
    if (!db->db)
    {
        LOG_W("db backend init fails");
        err = MC_DB_INIT_ERR;
        goto __EXIT;
    }

    rt_list_insert_before(&mc_ctx.db_list, &db->node);

    err = MC_OK;
__EXIT:
    return err;
}

#endif /* MC_CLIENT_ENABLED */


#ifdef MC_SERVICE_ENABLED

static int32_t mc_service_msg_handler(datas_handle_t service, data_msg_t *msg)
{
    switch (msg->msg_id)
    {
    case MSG_SERVICE_SUBSCRIBE_REQ:
    {
        break;
    }
    case MSG_SERVICE_UNSUBSCRIBE_REQ:
    {
        break;
    }
    case MSG_SERVICE_CONFIG_REQ:
    {
        break;
    }
    case MSG_SERVICE_METRICS_SAVE_REQ:
    {
        mc_metrics_save_req_t *req;
        mc_metrics_t *metrics;
        mc_err_t err;
        rt_err_t send_res;

        req = (mc_metrics_save_req_t *)data_service_get_msg_body(msg);

        metrics = mc_copy_metrics((mc_metrics_t *)&req->data[0]);

        err = mc_save_metrics((void *)&metrics->data[0], true);
        RT_ASSERT(MC_OK == err);
        send_res = datas_send_response(service, msg, 0);
        RT_ASSERT(RT_EOK == send_res);
        break;
    }
    default:
    {
        RT_ASSERT(0);
    }
    }

    return 0;
}


static data_service_config_t mc_service_cb =
{
    .max_client_num = 1,
    .queue = RT_NULL,
    .data_filter = NULL,
    .msg_handler = mc_service_msg_handler,
};

#endif /* MC_SERVICE_ENABLED */


#ifdef MC_CLIENT_ENABLED

static int mc_service_callback(data_callback_arg_t *arg)
{
    if (MSG_SERVICE_METRICS_SAVE_RSP == arg->msg_id)
    {
        data_rsp_t *rsp;
        RT_ASSERT(arg->data_len == sizeof(*rsp));
        rsp = (data_rsp_t *)arg->data;
        RT_ASSERT(0 == rsp->result);

        rt_sem_release(mc_ctx.sem);
    }
    else if (MSG_SERVICE_SUBSCRIBE_RSP == arg->msg_id)
    {
        data_subscribe_rsp_t *rsp;
        RT_ASSERT(arg->data_len == sizeof(*rsp));
        rsp = (data_subscribe_rsp_t *)arg->data;
        RT_ASSERT(0 == rsp->result);

        rt_sem_release(mc_ctx.sem);
    }

    return 0;

}
static int mc_init(void)
{
    uint32_t i;
    rt_err_t err;
    rt_sem_t sem;

    mc_ctx.sem = rt_sem_create("mc", 0, RT_IPC_FLAG_FIFO);
    RT_ASSERT(mc_ctx.sem);

    mc_ctx.client_handle = datac_open();
    RT_ASSERT(DATA_CLIENT_INVALID_HANDLE != mc_ctx.client_handle);
    datac_subscribe(mc_ctx.client_handle, "mc", mc_service_callback, 0);

    err = rt_sem_take(mc_ctx.sem, 1000);
    RT_ASSERT(RT_EOK == err);
    rt_sem_delete(mc_ctx.sem);
    mc_ctx.sem = NULL;

    for (i = 0; i < MC_PERIOD_MAX; i++)
    {
        rt_list_init(&mc_ctx.mng[i].collector_list);
    }
    err = rt_mutex_init(&mc_ctx.lock, "mc", RT_IPC_FLAG_FIFO);
    RT_ASSERT(RT_EOK == err);

    mc_ctx.init = true;


    return 0;
}
INIT_PRE_APP_EXPORT(mc_init);

#else

static void mc_handle_msg(mc_mq_msg_t *msg)
{
    mc_metrics_t *data;
    mc_err_t err = MC_OK;

    RT_ASSERT(msg && msg->db && msg->data);

    mc_enter_critical();
    if (msg->db->db) /* check whether db has been closed */
    {
        err = mc_backend_write(msg->db->db, msg->data,
                               MC_METRICS_TOTAL_LEN(msg->data->header.len));
        mc_ctx.err_code = err;
    }
    mc_exit_critical();
    RT_ASSERT(MC_OK == err);
    if (msg->freed)
    {
        rt_free(msg->data);
    }
}

static void mc_thread_entry(void *param)
{
    rt_err_t err;
    mc_mq_msg_t msg;

    while (1)
    {
        err = rt_mq_recv(mc_ctx.queue, &msg, sizeof(msg), RT_WAITING_FOREVER);
        RT_ASSERT(RT_EOK == err);

        mc_handle_msg(&msg);
    }
}

#ifdef MC_SERVICE_ENABLED
static int mc_service_init(void)
{

    mc_ctx.service_handle = datas_register("mc", &mc_service_cb);
    RT_ASSERT(mc_ctx.service_handle);

    return 0;
}
INIT_COMPONENT_EXPORT(mc_service_init);
#endif /* MC_SERVICE_ENABLED */

#ifndef MC_AUTO_INIT_DISABLED
    static
#endif /* !MC_AUTO_INIT_DISABLED */
mc_err_t mc_init(void)
{
    uint32_t i;
    rt_err_t err;

    if (mc_ctx.init)
    {
        return MC_ERROR;
    }

    rt_list_init(&mc_ctx.db_list);

    mc_ctx.default_db.name = "metrics";
    mc_ctx.default_db.max_size = MC_DEFAULT_DB_SIZE;
    mc_ctx.default_db.db = mc_backend_init(mc_ctx.default_db.name, mc_ctx.default_db.max_size);

    RT_ASSERT(mc_ctx.default_db.db);

    rt_list_insert_before(&mc_ctx.db_list, &mc_ctx.default_db.node);

    for (i = 0; i < MC_PERIOD_MAX; i++)
    {
        rt_list_init(&mc_ctx.mng[i].collector_list);
    }
    err = rt_mutex_init(&mc_ctx.lock, "mc", RT_IPC_FLAG_FIFO);
    RT_ASSERT(RT_EOK == err);

    mc_ctx.queue = rt_mq_create("metrics", sizeof(mc_mq_msg_t), 256, RT_IPC_FLAG_FIFO);
    RT_ASSERT(mc_ctx.queue);
    err = rt_thread_init(&g_metrics_thread, "metrics", mc_thread_entry, (void *)NULL,
                         (void *)g_metrics_thread_stack, sizeof(g_metrics_thread_stack),
                         RT_THREAD_PRIORITY_IDLE - 1, 10);

    RT_ASSERT(RT_EOK == err);
    rt_thread_startup(&g_metrics_thread);

    mc_ctx.init = true;

    return MC_OK;
}

const char *mc_get_path(void)
{
#ifdef MC_BACKEND_USING_FILE
    return "/metrics";
#else
    return NULL;
#endif /* MC_BACKEND_USING_FILE */
}

#ifndef MC_AUTO_INIT_DISABLED
static int mc_module_init(void)
{
    return (MC_OK == mc_init());
}
INIT_PRE_APP_EXPORT(mc_module_init);
#endif /* !MC_AUTO_INIT_DISABLED */

#endif /* MC_CLIENT_ENABLED */

#if 0
#include "dfs_posix.h"
static int pcap_fid;

static bool metrics_read_callback(void *data, uint32_t data_len, uint32_t time)
{
    uint32_t packet_hdr[4] = {time, 0, data_len, data_len};
    int wr_len;

    wr_len = write(pcap_fid, &packet_hdr[0], sizeof(packet_hdr));
    RT_ASSERT(wr_len == sizeof(packet_hdr));
    wr_len = write(pcap_fid, data, data_len);
    RT_ASSERT(wr_len == data_len);

    return false;
}


static void mc_convert_to_pcap(void)
{
    int wr_len;
    uint8_t *data;
    mc_err_t err;

    uint8_t fileheader[] = { 0xD4, 0xC3, 0xB2, 0xA1, 0x02, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xa1, 0x00, 0x00, 0x00 };


    pcap_fid = open("metrics.pcap", O_CREAT | O_RDWR | O_BINARY | O_TRUNC);
    RT_ASSERT(pcap_fid >= 0);
    wr_len = write(pcap_fid, fileheader, sizeof(fileheader));
    RT_ASSERT(wr_len == sizeof(fileheader));

    err = mc_read_raw_metrics(metrics_read_callback);
    RT_ASSERT(MC_OK == err);

    close(pcap_fid);

}
MSH_CMD_EXPORT(mc_convert_to_pcap, mc_convert_to_pcap);

#endif


