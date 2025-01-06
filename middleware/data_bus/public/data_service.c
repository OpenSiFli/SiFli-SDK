/**
  ******************************************************************************
  * @file   data_service.c
  * @author Sifli software development team
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

#include "rtdef.h"
#include "rtthread.h"
#include "string.h"
#include "data_service.h"
#include "../public/data_prov_int.h"
#include "board.h"
#ifdef USING_IPC_QUEUE
    #include "bf0_mbox_common.h"
#endif /* USING_IPC_QUEUE */

#if !defined(USING_IPC_QUEUE) || defined(DATA_SVC_MBOX_THREAD_DISABLED)
    #define DS_MBOX_DISABLED
#endif /* !USING_IPC_QUEUE || DATA_SVC_MBOX_THREAD_DISABLED */

#ifndef DS_MBOX_DISABLED
    #include "ipc_queue.h"
#endif /* !DS_MBOX_DISABLED */
#include "mem_section.h"

#define DBG_TAG           "DS"
#define DBG_LVL           DBG_LOG
#include "rtdbg.h"


#ifndef RT_MAIN_THREAD_PRIORITY
    #define RT_MAIN_THREAD_PRIORITY       (RT_THREAD_PRIORITY_MAX / 3)
#endif
#ifndef DATA_SVC_CONN_MAX
    #define DATA_SVC_CONN_MAX 16
#endif

#define DATA_MBOX_SIZE  128
#define DATA_SERVICE_MAX_NUM  (32)
#define DATA_SERVICE_SYSTEM_SERVICE_ID   (0)


#define ROUT_ID_CONN_ID_OFFSET     (0)
#define ROUT_ID_CONN_ID_MASK       (0xFF)
#define ROUT_ID_SERV_ID_OFFSET     (8)
#define ROUT_ID_SERV_ID_MASK       (0x7F)
#define ROUT_ID_CORE_ID_OFFSET     (15)
#define ROUT_ID_CORE_ID_MASK       (1)


#define MAKE_ROUT_ID_PART(part, mask, offset)      (((part) & (mask)) << (offset))
#define GET_ROUT_ID_PART(rout_id, mask, offset)    (((rout_id) >> (offset)) & (mask))
#define MAKE_ROUT_ID_CONN_ID(conn_id)           (MAKE_ROUT_ID_PART(conn_id, ROUT_ID_CONN_ID_MASK, ROUT_ID_CONN_ID_OFFSET))
#define MAKE_ROUT_ID_SERV_ID(serv_id)           (MAKE_ROUT_ID_PART(serv_id, ROUT_ID_SERV_ID_MASK, ROUT_ID_SERV_ID_OFFSET))
#define MAKE_ROUT_ID_CORE_ID(core_id)           (MAKE_ROUT_ID_PART(core_id, ROUT_ID_CORE_ID_MASK, ROUT_ID_CORE_ID_OFFSET))
#define MAKE_ROUT_ID(conn_id, serv_id)          (MAKE_ROUT_ID_CONN_ID(conn_id) \
                                                 | MAKE_ROUT_ID_SERV_ID(serv_id) \
                                                 | MAKE_ROUT_ID_CORE_ID(DS_CORE_ID_CURRENT))
#define MAKE_ROUT_ID_PROXY(conn_id, serv_id)    (MAKE_ROUT_ID_CONN_ID(conn_id) \
                                                 | MAKE_ROUT_ID_SERV_ID(serv_id) \
                                                 | MAKE_ROUT_ID_CORE_ID(DS_CORE_ID_PROXY))
#define MAKE_SYSTEM_SERV_ROUT_ID(conn_id)       (MAKE_ROUT_ID_CONN_ID(conn_id) \
                                                 | MAKE_ROUT_ID_SERV_ID(DATA_SERVICE_SYSTEM_SERVICE_ID) \
                                                 | MAKE_ROUT_ID_CORE_ID(DS_CORE_ID_CURRENT))
#define GET_ROUT_ID_CONN_ID(rout_id)            (GET_ROUT_ID_PART(rout_id, ROUT_ID_CONN_ID_MASK, ROUT_ID_CONN_ID_OFFSET))
#define GET_ROUT_ID_SERV_ID(rout_id)            (GET_ROUT_ID_PART(rout_id, ROUT_ID_SERV_ID_MASK, ROUT_ID_SERV_ID_OFFSET))
#define GET_ROUT_ID_CORE_ID(rout_id)            (GET_ROUT_ID_PART(rout_id, ROUT_ID_CORE_ID_MASK, ROUT_ID_CORE_ID_OFFSET))

#define IS_SAME_CORE(core_id)                   ((core_id) == DS_CORE_ID_CURRENT)
#define IS_ROUT_ID_FROM_SAME_CORE(rout_id)      (IS_SAME_CORE(GET_ROUT_ID_CORE_ID(rout_id)))

#define DS_CLIENT_HANDLE_OFFSET  (10)
/** client handle to connection id */
#define DS_CLIENT_HANDLE_2_CID(handle)  ((handle) - DS_CLIENT_HANDLE_OFFSET)
/** connection id to client handle */
#define DS_CID_2_CLIENT_HANDLE(cid)     ((cid) + DS_CLIENT_HANDLE_OFFSET)

#define DATA_CONN_INVALID_ID      (UINT8_MAX)

uint32_t DS_CORE_ID_CURRENT, DS_CORE_ID_PROXY;
uint32_t DS_PROXY_CHANNEL, TX_BUF_SIZE, TX_BUF_ADDR, TX_BUF_ADDR_ALIAS, RX_BUF_ADDR;

enum
{
    DATA_STATE_IDLE,
    DATA_STATE_OPEN,
    DATA_STATE_SUB_PENDING,
    DATA_STATE_UNSUB_PENDING,
    DATA_STATE_CLOSE_PENDING,
    DATA_STATE_READY,
};

typedef struct data_connection
{
    // callback for data service, NULL if it is proxy.
    data_callback_t   callback;
    uint32_t        user_data;
    uint8_t         state;
    uint16_t        dst_cid;
    uint8_t         *data;
    rt_mq_t         mq;
    /** subscribed service name */
    char           svc_name[MAX_SVC_NAME_LEN];
} data_connection_t;

#ifndef DS_MBOX_DISABLED
typedef struct
{
    ipc_queue_handle_t device;
    rt_size_t size;
} data_mbox_mq_data_t;
#endif /* DS_MBOX_DISABLED */

static int32_t sys_service_msg_handler(datas_handle_t service, data_msg_t *msg);

#ifndef DS_MBOX_DISABLED
    static ipc_queue_handle_t g_proxy;
    static struct rt_thread g_data_mbox_thread;
    static rt_mq_t g_data_mbox_mq;
    static struct rt_mutex g_ds_ipc_mutex;
#endif

#ifndef DATA_SVC_PROC_THREAD_DISABLED
    static struct rt_thread g_data_process_thread;
    static rt_mq_t g_ds_queue;
#endif /* DATA_SVC_PROC_THREAD_DISABLED */

static struct rt_mutex g_data_service_mutex;


static void *data_service_list[DATA_SERVICE_MAX_NUM];

static rt_list_t data_service_db;

static data_service_t *sys_service;

static data_service_config_t sys_service_config =
{
    .max_client_num = DATA_SVC_CONN_MAX,
    .queue = RT_NULL,
    .msg_handler = sys_service_msg_handler,
};


#ifndef DS_MBOX_DISABLED
static void ds_init_proxy(ipc_queue_rx_ind_t rx_ind)
{
    ipc_queue_cfg_t q_cfg;
    int32_t ret;

    q_cfg.qid = DS_PROXY_CHANNEL;
    q_cfg.tx_buf_size = TX_BUF_SIZE;
    q_cfg.tx_buf_addr = TX_BUF_ADDR;
    q_cfg.tx_buf_addr_alias = TX_BUF_ADDR_ALIAS;
    q_cfg.rx_buf_addr = RX_BUF_ADDR;
    q_cfg.rx_ind = rx_ind;
    q_cfg.user_data = 0;

    g_proxy = ipc_queue_init(&q_cfg);
    RT_ASSERT(IPC_QUEUE_INVALID_HANDLE != g_proxy);
    ret = ipc_queue_open(g_proxy);
    RT_ASSERT(0 == ret);
}
#endif /* !DS_MBOX_DISABLED */



static inline void ds_enter_critical(void)
{
    rt_err_t err;

    err = rt_mutex_take(&g_data_service_mutex, rt_tick_from_millisecond(1000));
    RT_ASSERT(RT_EOK == err);
}

static inline void ds_exit_critical(void)
{
    rt_err_t err;

    err = rt_mutex_release(&g_data_service_mutex);
    RT_ASSERT(RT_EOK == err);
}

#ifndef DS_MBOX_DISABLED
static inline void ds_ipc_enter_critical(void)
{
    rt_err_t err;

    err = rt_mutex_take(&g_ds_ipc_mutex, RT_WAITING_FOREVER);
    RT_ASSERT(RT_EOK == err);
}

static inline void ds_ipc_exit_critical(void)
{
    rt_err_t err;

    err = rt_mutex_release(&g_ds_ipc_mutex);
    RT_ASSERT(RT_EOK == err);
}
#endif /* DS_MBOX_DISABLED */

static uint8_t **data_service_msg_body_ext(data_msg_t *msg)
{
    uint8_t **body_ext;

    if (msg->len > sizeof(msg->body))
    {
        body_ext = (uint8_t **)&msg->body[0];
    }
    else
    {
        body_ext = NULL;
    }

    return body_ext;

}

static uint8_t *init_msg(data_msg_t *msg, uint16_t msgid, uint16_t src_cid, uint16_t dst_cid, uint16_t body_len)
{
    uint8_t *body;

    RT_ASSERT(RT_NULL != msg);

    msg->msg_id = msgid;
    msg->src_cid = src_cid;
    msg->dst_cid = dst_cid;
    msg->len = body_len;
    msg->no_free = 0;
    if (body_len > SHORT_DATA_MSG_BODY_THRESHOLD) //Long message body, save in allocated new memory.
    {
        uint8_t **body_ext = (uint8_t **)&msg->body[0];
        body = rt_malloc(body_len);
        RT_ASSERT(RT_NULL != body);
        *body_ext = body;
    }
    else                  //Short message body, just save in strut data_msg_t.
    {
        body = &msg->body[0];
    }

    return body;
}

static uint8_t *init_msg_no_copy(data_msg_t *msg, uint16_t msgid, uint16_t src_cid,
                                 uint16_t dst_cid, uint16_t body_len, uint8_t *data)
{
    uint8_t *body;

    RT_ASSERT(RT_NULL != msg);

    msg->msg_id = msgid;
    msg->src_cid = src_cid;
    msg->dst_cid = dst_cid;
    msg->len = body_len;
    msg->no_free = 0;
    if (body_len > SHORT_DATA_MSG_BODY_THRESHOLD)
    {
        uint8_t **body_ext = (uint8_t **)&msg->body[0];
        //Long message body, just save orignal data pointer in header
        *body_ext = data;
        body = data;
        msg->no_free = 1;
    }
    else
    {
        //Short message body, copy origanl data to header
        body = &msg->body[0];
        memcpy((void *)body, (void *)data, body_len);
    }

    return body;
}

static void free_msg(data_msg_t *msg)
{
    uint8_t **body_ext;

    body_ext = data_service_msg_body_ext((data_msg_t *)msg);
    if (body_ext && !msg->no_free)
    {
        rt_free(*body_ext);
        *body_ext = 0;
    }
}

#ifndef DATA_SVC_PROC_THREAD_DISABLED
#if 0
static int32_t send_msg(data_msg_t *msg)
{
    rt_err_t result = -RT_ERROR;

    if (g_ds_queue)
        result = rt_mq_send(g_ds_queue, msg, sizeof(*msg));
    else
        free_msg(msg);

    return result;
}
#endif
#endif /* !DATA_SVC_PROC_THREAD_DISABLED */

// cid: receive connection id
static rt_err_t data_send_proxy(data_msg_t *msg)
{
#ifndef DS_MBOX_DISABLED
    rt_size_t len;
    uint8_t *body;

    if (IPC_QUEUE_INVALID_HANDLE == g_proxy)
    {
        return -RT_ERROR;
    }

    ds_ipc_enter_critical();

    len = ipc_queue_write(g_proxy, msg, sizeof(*msg), 1000);
    RT_ASSERT(len == sizeof(*msg));
    if (len != sizeof(*msg))
    {
        free_msg(msg);
        goto __ERROR;
    }

    if (data_service_msg_body_ext((data_msg_t *)msg))
    {
        body = data_service_get_msg_body((data_msg_t *)msg);
        RT_ASSERT(RT_NULL != body);
        /* workaround: as unknonwn reason, HCPU wakeup time is ~700ms */
        len = ipc_queue_write(g_proxy, body, msg->len, 1000);
        RT_ASSERT(len == msg->len);
        free_msg(msg);

        if (len != msg->len)
        {
            goto __ERROR;
        }
    }

    ds_ipc_exit_critical();

    return RT_EOK;

__ERROR:

    ds_ipc_exit_critical();

#endif /* !DS_MBOX_DISABLED */

    return -RT_ERROR;
}

static data_service_t *find_service(char *name)
{
    data_service_t *service;
    bool found = false;

    /* try to find device object */

    rt_list_t *iter;
    rt_list_for_each(iter, &data_service_db)
    {
        service = rt_list_entry(iter, data_service_t, node);
        if (rt_strncmp(service->name, name, MAX_SVC_NAME_LEN - 1) == 0)
        {
            found = true;
            break;
        }
    }

    if (!found)
    {
        service = NULL;
    }

    return service;
}


static data_service_t *get_service(uint8_t serv_id)
{
    RT_ASSERT(serv_id < DATA_SERVICE_MAX_NUM)

    return data_service_list[serv_id];
}


static data_service_t *add_service(const char *name, data_service_config_t *config, uint8_t serv_id)
{
    data_service_t *service;
    uint32_t size;

    service = rt_malloc(sizeof(data_service_t));
    RT_ASSERT(RT_NULL != service);
    service->config = config;
    strncpy(service->name, name, MAX_SVC_NAME_LEN);
    service->name[MAX_SVC_NAME_LEN - 1] = '\0';
    /* insert new service into service db */
    rt_list_init(&service->node);
    rt_list_insert_before(&data_service_db, &service->node);

    /* init clients link list */
    rt_list_init(&service->clients);
    size = sizeof(*service->client_list) * config->max_client_num;
    service->client_list = rt_malloc(size);
    RT_ASSERT(service->client_list);
    memset(service->client_list, 0, size);

    RT_ASSERT(serv_id < DATA_SERVICE_MAX_NUM);

    data_service_list[serv_id] = service;
    service->id = serv_id;
    service->last_clnt = 0;

    return service;
}

static rt_err_t forward_msg(data_service_t *service, data_msg_t *msg)
{
    rt_mq_t queue;
    rt_err_t result;

    RT_ASSERT(service);
    RT_ASSERT(msg);

#ifndef DATA_SVC_PROC_THREAD_DISABLED
    if (DATA_SERVICE_SYSTEM_SERVICE_ID == service->id)
    {
        queue = NULL;
    }
    else
    {
        if (service->config->queue)
        {
            queue = service->config->queue;
        }
        else
        {
            queue = g_ds_queue;
        }
    }
#else
    if (DATA_SERVICE_SYSTEM_SERVICE_ID == service->id)
    {
        queue = NULL;
    }
    else
    {
        queue = service->config->queue;
    }
#endif /* !DATA_SVC_PROC_THREAD_DISABLED */

    if (queue)
    {
        result = rt_mq_send(queue, msg, sizeof(*msg));
    }
    else
    {
        service->config->msg_handler(service, (data_msg_t *)msg);
        free_msg(msg);
        result = RT_EOK;
    }

    return result;
}


static rt_err_t dispatch_msg(data_msg_t *msg)
{
    rt_err_t result;
    data_service_t *service;

    if (!IS_ROUT_ID_FROM_SAME_CORE(msg->dst_cid))
    {
        RT_ASSERT(IS_ROUT_ID_FROM_SAME_CORE(msg->src_cid));
        // service at other side
        result = data_send_proxy(msg);
    }
    else
    {
        service = get_service(GET_ROUT_ID_SERV_ID(msg->dst_cid));
        RT_ASSERT(service)
        result = forward_msg(service, msg);
    }

    return result;
}

static uint8_t add_clnt(data_service_t *service, uint16_t src_cid)
{
    data_service_client_t *client;
    uint8_t conn_id;
    uint8_t last_conn_id;

    RT_ASSERT(service);

    ds_enter_critical();
    last_conn_id = service->last_clnt;
    conn_id = DATA_CONN_INVALID_ID;
    for (uint32_t i = 0; i < service->config->max_client_num; i++)
    {
        uint8_t id = (i + last_conn_id) % service->config->max_client_num;
        if (NULL == service->client_list[id])
        {
            service->last_clnt = id;
            conn_id = id;
            break;
        }
    }

    if (DATA_CONN_INVALID_ID == conn_id)
    {
        ds_exit_critical();
        return conn_id;
    }

    client = rt_malloc(sizeof(*client));
    RT_ASSERT(RT_NULL != client);
    client->src_cid = src_cid;
    client->conn_id = conn_id;
    client->user_data = NULL;
    client->config = NULL;

    /* add client */
    rt_list_init(&client->node);
    rt_list_insert_before(&service->clients, &client->node);

    service->client_list[conn_id] = client;

    ds_exit_critical();

    return conn_id;
}

static rt_err_t del_clnt(data_service_t *service, uint8_t conn_id)
{
    data_service_client_t *removed_client;

    RT_ASSERT(service);
    RT_ASSERT(conn_id < service->config->max_client_num);

    ds_enter_critical();

    removed_client = (data_service_client_t *)service->client_list[conn_id];
    RT_ASSERT(removed_client);

    if (removed_client->config)
        rt_free(removed_client->config);
    rt_list_remove(&removed_client->node);
    rt_free(removed_client);

    service->client_list[conn_id] = NULL;

    ds_exit_critical();

    return RT_EOK;
}

static data_service_client_t *get_clnt(data_service_t *service, uint8_t conn_id)
{
    data_service_client_t *conn = NULL;

    RT_ASSERT(service);
    ds_enter_critical();
    if (conn_id < service->config->max_client_num)
        conn = service->client_list[conn_id];
    ds_exit_critical();

    return conn;
}

static data_connection_t *get_conn(uint8_t conn_id)
{
    data_service_client_t *clnt;
    data_connection_t *user_data = NULL;

    clnt = get_clnt(sys_service, conn_id);
    if (clnt)
        user_data = (data_connection_t *)clnt->user_data;

    return user_data;
}


static bool is_valid_handle(uint8_t handle)
{
    data_connection_t *conn;
    uint8_t cid;

    cid = DS_CLIENT_HANDLE_2_CID(handle);
    if ((DS_CLIENT_HANDLE_OFFSET > handle)
            || (cid >= DATA_SVC_CONN_MAX))
    {
        goto __ERR;
    }

    conn = get_conn(cid);
    if (!conn || DATA_STATE_READY > conn->state)
    {
        goto __ERR;
    }

    return true;

__ERR:
    return false;
}

static bool is_handle_in_range(uint8_t handle)
{
    uint8_t cid;

    cid = DS_CLIENT_HANDLE_2_CID(handle);
    if ((DS_CLIENT_HANDLE_OFFSET > handle)
            || (cid >= DATA_SVC_CONN_MAX))
    {
        return false;
    }
    else
    {
        return true;
    }
}

static void send_subscribe_rsp(data_msg_t *req_msg, int32_t result)
{
    data_subscribe_rsp_t *rsp;
    data_msg_t rsp_msg;

    rsp = (data_subscribe_rsp_t *)init_msg(&rsp_msg, MSG_SERVICE_SUBSCRIBE_RSP,
                                           req_msg->dst_cid, req_msg->src_cid, sizeof(*rsp));
    rsp->handle = DS_CID_2_CLIENT_HANDLE(GET_ROUT_ID_CONN_ID(req_msg->src_cid));
    rsp->result = result;
    result = dispatch_msg(&rsp_msg);
    RT_ASSERT(RT_EOK == result);
}

static void send_unsubscribe_rsp(data_msg_t *req_msg, int32_t result)
{
    data_rsp_t *rsp;
    data_msg_t rsp_msg;

    rsp = (data_rsp_t *)init_msg(&rsp_msg, MSG_SERVICE_UNSUBSCRIBE_RSP, req_msg->dst_cid, req_msg->src_cid, sizeof(*rsp));
    rsp->result = result;
    result = dispatch_msg(&rsp_msg);
    RT_ASSERT(RT_EOK == result);
}

static rt_err_t datac_subscribe_int(data_msg_t *msg)
{
    data_service_t *service;
    data_subscribe_req_t *req = (data_subscribe_req_t *)data_service_get_msg_body((data_msg_t *)msg);
    uint8_t cid;
    bool client_add_succ;
    rt_err_t result = RT_EOK;

    RT_ASSERT(RT_NULL != req);

    cid = GET_ROUT_ID_CONN_ID(msg->src_cid);

    service = find_service(req->service_name);
    LOG_D("data service subscribe (%s), service (%x)", req->service_name, service);

    if (service)
    {
        uint8_t conn_id;

        conn_id = add_clnt(service, msg->src_cid);
        if (DATA_CONN_INVALID_ID == conn_id)
        {
            client_add_succ = false;
        }
        else
        {
            msg->dst_cid = MAKE_ROUT_ID(conn_id, service->id);
            client_add_succ = true;
        }
        if (client_add_succ)
        {
            /* service client allocation success, update conn tbl by client side */
            LOG_D("datac subscribe dev open");
            forward_msg(service, msg);
            send_subscribe_rsp(msg, 0);
        }
        else
        {
            send_subscribe_rsp(msg, -1);
            result = -RT_ERROR;
        }
    }
    else
    {
        if (!IS_SAME_CORE(GET_ROUT_ID_CORE_ID(msg->src_cid)))
        {
            // Request is from other core, response
            /* service not available */
            send_subscribe_rsp(msg, -1);
            free_msg(msg);
        }
        else
        {
            msg->dst_cid = MAKE_ROUT_ID_PROXY(DATA_CONN_INVALID_ID, DATA_SERVICE_INVALID_ID);
            result = data_send_proxy(msg);
            if (result != RT_EOK)
            {
                /* send response if proxy request sent fails */
                msg->dst_cid = MAKE_ROUT_ID(DATA_CONN_INVALID_ID, DATA_SERVICE_INVALID_ID);
                send_subscribe_rsp(msg, -1);
            }
        }
    }

    return result;
}

static rt_err_t datac_unsubscribe_int(data_msg_t *msg)
{
    data_service_t *service;
    rt_err_t err = RT_EOK;

    if (IS_ROUT_ID_FROM_SAME_CORE(msg->dst_cid))
    {
        service = get_service(GET_ROUT_ID_SERV_ID(msg->dst_cid));
        RT_ASSERT(service);
        err = del_clnt(service, GET_ROUT_ID_CONN_ID(msg->dst_cid));
        RT_ASSERT(RT_EOK == err);

        send_unsubscribe_rsp(msg, 0);
    }

    err = dispatch_msg(msg);

    return err;
}

#if 0
static int8_t datac_rx_int(data_msg_t *msg)
{
    rt_err_t result;
    data_service_t *service;
    uint8_t handle = msg->dst_cid;
    data_connection_t *conn;
    data_rx_rsp_t *rsp;
    data_rx_req_t *req = (data_rx_req_t *)msg_body(msg);

    RT_ASSERT(handle < DATA_SVC_CONN_MAX);
    conn = &g_data_connection[handle];
    RT_ASSERT(DATA_STATE_READY == conn->state);

    if (NULL == conn->service)
    {
        conn->state = DATA_STATE_BUSY;
        conn->len = req->len;
        conn->data = req->data;
        RT_ASSERT(msg->src_cid < 0);
        RT_ASSERT(conn->proxy_id >= 0);
        // service at other side
        msg->src_cid = handle;
        msg->dst_cid = conn->proxy_id;
        req->data = NULL;
        result = data_send_proxy(msg_body(msg));

    }
    else
    {
        uint8_t *data;
        RT_ASSERT(conn->proxy_id == msg->src_cid);

        service = conn->service;
        if (req->data)
        {
            // in the same core, save in data buffer directly
            data = req->data;
            conn->data = data;
        }
        else
        {
            rsp = (data_rx_rsp_t *)create_rsp_msg(msg, MSG_SERVICE_RX_RSP, sizeof(*rsp) + req->len);
            data = &rsp->data[0];
            conn->data = NULL;
        }
        rsp->len = service->action_cb->data_read_cb(service, conn->client, data, req->len);
        conn->state = DATA_STATE_BUSY;
    }

    return result;
}
#endif


#if 0
static int datac_filter(rt_device_t dev, int len, uint8_t *data)
{
    int r = 0;

    if (dev)
    {
        int i;
        for (i = 0; i < DATA_SVC_MAX; i++)
            if ((dev == g_data_connection[i].dev) && g_data_connection[i].callback)
            {
                r = (*g_data_connection[i].callback)(DATA_CBK_DATA_RX_SVC, len, data);
                if (r)
                    break;
            }
    }
    return r;
}
#endif

#ifndef DS_MBOX_DISABLED
static void handle_proxy_msg(data_msg_t *msg)
{
    rt_err_t result;
    data_service_t *service;
    rt_mq_t queue;

    RT_ASSERT(IS_ROUT_ID_FROM_SAME_CORE(msg->dst_cid));

    /* pointer in long message from other core should always be freed */
    msg->no_free = 0;

    switch (msg->msg_id)
    {
    case MSG_SERVICE_SUBSCRIBE_REQ:
    {
        datac_subscribe_int(msg);
        break;
    }
    case MSG_SERVICE_UNSUBSCRIBE_REQ:
    {
        datac_unsubscribe_int(msg);
        break;
    }
    default:
    {
        service = get_service(GET_ROUT_ID_SERV_ID(msg->dst_cid));
        RT_ASSERT(service)

        /* config queue of system service is NULL, can be got from g_ds_queue */
        if (service->config->queue)
        {
            queue = service->config->queue;
        }
        else
        {
            queue = g_ds_queue;
        }

        result = rt_mq_send(queue, msg, sizeof(*msg));
        RT_ASSERT(RT_EOK == result);
    }
    }
}

static rt_uint32_t data_proxy_process(void)
{
    size_t size;
    uint8_t *body;
    uint8_t **body_ext;
    uint8_t *buf;
    /* expected message size to be read */
    size_t exp_size;
    /* size of message that has been read */
    static size_t total_read_len = 0;
    static data_msg_t msg;
    rt_int32_t timeout;

    timeout = RT_WAITING_FOREVER;
    while (1)
    {
        if (total_read_len < sizeof(msg))
        {
            /* read message header */
            exp_size = sizeof(msg) - total_read_len;
            buf = (uint8_t *)&msg + total_read_len;
        }
        else
        {
            body_ext = data_service_msg_body_ext((data_msg_t *)&msg);
            if (body_ext)
            {
                /* has extended body */
                RT_ASSERT(total_read_len < (sizeof(msg) + msg.len));
                exp_size = sizeof(msg) + msg.len - total_read_len;
                buf = *body_ext + msg.len - exp_size;
            }
            else
            {
                /* impossible reach here*/
                RT_ASSERT(0);
            }
        }
        size = ipc_queue_read(g_proxy, buf, exp_size);
        total_read_len += size;
        if (0 == size)
        {
            /* no new data */
            if (total_read_len > 0)
            {
                /* reduce timeout value to avoid going to sleep */
                timeout = rt_tick_from_millisecond(1);
            }
            break;
        }
        if (total_read_len == sizeof(msg))
        {
            body_ext = data_service_msg_body_ext((data_msg_t *)&msg);
            if (body_ext)
            {
                body = rt_malloc(msg.len);
                RT_ASSERT(RT_NULL != body);
                *body_ext = body;
                /* message with extended body, continue read the body */
            }
            else
            {
                /* message without extended body has been received*/
                total_read_len = 0;
            }
        }
        else if (total_read_len == (sizeof(msg) + msg.len))
        {
            /* message with extended bdy has been received */
            total_read_len = 0;
        }
        else
        {
            /* message is not complete yet */
        }

        if (0 == total_read_len)
        {
            /* restore to default timeout value */
            timeout = RT_WAITING_FOREVER;
            handle_proxy_msg(&msg);
        }
    }

    return timeout;
}
#endif /* !DS_MBOX_DISABLED */

static void datac_service_usrcbk(uint8_t cid, uint16_t msg_id, uint16_t data_len, uint8_t *data)
{
    data_service_mq_t arg_mq;
    data_connection_t *conn = get_conn(cid);

    if (conn)
    {
        arg_mq.arg.msg_id = msg_id;
        arg_mq.arg.data_len = data_len;
        arg_mq.arg.user_data = conn->user_data;
        if (conn->mq)
        {
            arg_mq.callback = conn->callback;
            arg_mq.arg.data = rt_malloc(data_len);
            memcpy(arg_mq.arg.data, data, data_len);
            if (RT_EOK != rt_mq_send(conn->mq, &arg_mq, sizeof(arg_mq)))
            {
                LOG_D("datac(%d) usrcbk Qfull,msg:%d,data:%x,len:%d", DS_CID_2_CLIENT_HANDLE(cid), msg_id, data, data_len);
                RT_ASSERT(0); //send msg fail.
            }
        }
        else if (conn->callback)
        {
            arg_mq.arg.data = data;
            conn->callback(&arg_mq.arg);
        }
    }
}

static int32_t sys_service_msg_handler(datas_handle_t svc, data_msg_t *msg)
{
    //data_service_t *service = (data_service_t *)svc;
    switch (msg->msg_id)
    {
    case MSG_SERVICE_SUBSCRIBE_RSP:
    {
        uint8_t conn_id;
        data_connection_t *conn;
        data_msg_t unsub_req_msg;
        bool send_unsub_req;
        bool call_usr_cbk;
        data_rsp_t *rsp = (data_rsp_t *)data_service_get_msg_body(msg);

        conn_id = GET_ROUT_ID_CONN_ID(msg->dst_cid);
        ds_enter_critical();
        conn = get_conn(conn_id);
        RT_ASSERT(conn != NULL);
        send_unsub_req = false;
        call_usr_cbk = false;
        if (DATA_STATE_SUB_PENDING == conn->state)
        {
            if (rsp->result >= 0)
            {
                conn->dst_cid = msg->src_cid;
                conn->state = DATA_STATE_READY;
            }
            else
            {
                conn->state = DATA_STATE_OPEN;
            }
            call_usr_cbk = true;
        }
        else if ((DATA_STATE_CLOSE_PENDING == conn->state)
                 && (rsp->result >= 0))
        {
            conn->dst_cid = msg->src_cid;
            send_unsub_req = true;
        }
        ds_exit_critical();
        if (call_usr_cbk)
        {
            datac_service_usrcbk(conn_id, msg->msg_id, msg->len, (uint8_t *)rsp);
        }

        if (send_unsub_req)
        {
            init_msg(&unsub_req_msg, MSG_SERVICE_UNSUBSCRIBE_REQ,
                     MAKE_SYSTEM_SERV_ROUT_ID(conn_id), conn->dst_cid, 0);
            RT_ASSERT(RT_EOK == datac_unsubscribe_int(&unsub_req_msg));

        }
        break;
    }
    case MSG_SERVICE_UNSUBSCRIBE_RSP:
    {
        data_rsp_t *rsp = (data_rsp_t *)data_service_get_msg_body(msg);
        uint8_t conn_id;
        data_connection_t *conn;
        bool call_usr_callback;

        conn_id = GET_ROUT_ID_CONN_ID(msg->dst_cid);
        ds_enter_critical();
        conn = get_conn(conn_id);
        call_usr_callback = false;
        if (DATA_STATE_CLOSE_PENDING == conn->state)
        {
            /* free connection */
            rt_free(conn);
            del_clnt(sys_service, conn_id);
        }
        else if (DATA_STATE_UNSUB_PENDING == conn->state)
        {
            conn->state = DATA_STATE_OPEN;
            call_usr_callback = true;
        }
        ds_exit_critical();

        if (call_usr_callback)
        {
            datac_service_usrcbk(conn_id, msg->msg_id, msg->len, (uint8_t *)rsp);
        }
        break;
    }

    case MSG_SERVICE_DATA_NTF_IND:
    case MSG_SERVICE_CONFIG_RSP:
    case MSG_SERVICE_TX_RSP:
    default:
    {
        uint8_t *data = data_service_get_msg_body(msg);
        uint8_t conn_id;
        datac_handle_t handle;

        conn_id = GET_ROUT_ID_CONN_ID(msg->dst_cid);
        handle = DS_CID_2_CLIENT_HANDLE(conn_id);
        if (is_valid_handle(handle))
        {
            datac_service_usrcbk(conn_id, msg->msg_id, msg->len, data);
        }
    }
    }

    return 0;
}

void data_service_entry(void *param)
{
    data_msg_t msg;
    rt_err_t ret;
    rt_mq_t msg_queue = (rt_mq_t) param;

#ifndef DS_MBOX_DISABLED
#if (defined(CORE_ID_CURRENT) && (CORE_ID_CURRENT == CORE_ID_HCPU))
    {
        if (g_data_mbox_mq && (g_ds_queue == msg_queue))
        {
            /* only execute by ds_proc thread as this entry may be used by other thread */
            lcpu_power_on();
        }
    }
#endif /* CORE_ID_HCPU */
#endif /* !DS_MBOX_DISABLED */

    while (1)
    {
        data_service_t *service;

        ret = rt_mq_recv(msg_queue, &msg, sizeof(msg), RT_WAITING_FOREVER);

        RT_ASSERT(RT_EOK == ret);

        service = get_service(GET_ROUT_ID_SERV_ID(msg.dst_cid));
        RT_ASSERT(service);
        service->config->msg_handler(service, (data_msg_t *)&msg);

        free_msg(&msg);
    }
    RT_ASSERT(0);
}

#ifndef DS_MBOX_DISABLED
static int32_t proxy_rx_ind(ipc_queue_handle_t dev, size_t size)

{
    rt_err_t result;
    data_mbox_mq_data_t mq_data;

    mq_data.device = dev;
    mq_data.size = size;
    if (g_data_mbox_mq)
    {
        result = rt_mq_send(g_data_mbox_mq, &mq_data, sizeof(mq_data));
    }
    else
    {
        result = -RT_ERROR;
    }
    return result;
}

static void data_mbox_entry(void *param)
{
    data_mbox_mq_data_t data;
    rt_int32_t timeout;
    rt_err_t err;

    timeout = RT_WAITING_FOREVER;
    while (1)
    {
        err = rt_mq_recv(g_data_mbox_mq, &data, sizeof(data), timeout);

        if (RT_EOK == err)
        {
            timeout = data_proxy_process();
        }
    }
}
#endif  /* !DS_MBOX_DISABLED */

/***************************** Data_service API functions****************************************/

void datac_delayed_usr_cbk(data_service_mq_t *arg_msg)
{
    RT_ASSERT(arg_msg);

    if (arg_msg->callback)
    {
        arg_msg->callback(&arg_msg->arg);
    }

    if (arg_msg->arg.data)
    {
        rt_free(arg_msg->arg.data);
    }
}


datac_handle_t datac_open(void)
{
    uint8_t cid;
    data_service_client_t *clnt;
    data_connection_t *conn;
    datac_handle_t handle;

    cid = add_clnt(sys_service, MAKE_ROUT_ID(DATA_CONN_INVALID_ID, DATA_SERVICE_INVALID_ID));

    if (DATA_CONN_INVALID_ID != cid)
    {
        ds_enter_critical();
        clnt = get_clnt(sys_service, cid);
        clnt->user_data = rt_malloc(sizeof(data_connection_t));
        RT_ASSERT(clnt->user_data);
        memset(clnt->user_data, 0, sizeof(data_connection_t));

        conn = (data_connection_t *)clnt->user_data;
        conn->state = DATA_STATE_OPEN;
        ds_exit_critical();

        handle = DS_CID_2_CLIENT_HANDLE(cid);
    }
    else
    {
        handle = DATA_CLIENT_INVALID_HANDLE;
    }

    return handle;
}

rt_err_t datac_close(datac_handle_t handle)
{
    data_msg_t msg;
    rt_err_t result;
    data_connection_t *conn;
    uint8_t cid;
    bool send_unsubscribe_req;

    if (!is_handle_in_range(handle))
    {
        goto __ERR;
    }

    ds_enter_critical();

    cid = DS_CLIENT_HANDLE_2_CID(handle);
    conn = get_conn(cid);
    if (!conn || DATA_STATE_OPEN > conn->state)
    {
        goto __ERR;
    }

    send_unsubscribe_req = false;
    result = RT_EOK;
    if (DATA_STATE_READY == conn->state)
    {
        conn->state = DATA_STATE_CLOSE_PENDING;
        send_unsubscribe_req = true;
    }
    else if (DATA_STATE_OPEN == conn->state)
    {
        rt_free(conn);
        result = del_clnt(sys_service, cid);
    }
    else
    {
        /* subscribe/unsubscribe_req is pending, do nothing, wait for rsp */
        conn->state = DATA_STATE_CLOSE_PENDING;
    }

    ds_exit_critical();

    if (send_unsubscribe_req)
    {
        init_msg(&msg, MSG_SERVICE_UNSUBSCRIBE_REQ,
                 MAKE_SYSTEM_SERV_ROUT_ID(cid), conn->dst_cid, 0);

        result = datac_unsubscribe_int(&msg);
    }

    return result;

__ERR:

    ds_exit_critical();

    return -RT_ERROR;
}

void datac_subscribe_ex(datac_handle_t handle, char *service_name,
                        data_callback_t cbk, uint32_t user_data, rt_mq_t mq)
{
    data_subscribe_req_t *req;
    data_msg_t msg;
    uint16_t name_len = strlen(service_name) + 1;
    uint16_t body_len = name_len + sizeof(data_subscribe_req_t);
    uint8_t cid;
    data_connection_t *conn;
    rt_err_t result;
    data_service_mq_t arg_mq;
    data_subscribe_rsp_t subscribe_rsp;

    if (!is_handle_in_range(handle))
    {
        goto __ERR;
    }

    ds_enter_critical();

    cid = DS_CLIENT_HANDLE_2_CID(handle);
    conn = get_conn(cid);

    if (DATA_STATE_OPEN != conn->state)
    {
        /* connection has connected to a service */
        goto __ERR;
    }

    conn->state = DATA_STATE_SUB_PENDING;
    conn->callback = cbk;
    conn->mq = mq;
    conn->user_data = user_data;
    conn->data = NULL;
    strncpy(conn->svc_name, service_name, MAX_SVC_NAME_LEN - 1);
    conn->svc_name[MAX_SVC_NAME_LEN - 1] = 0;

    ds_exit_critical();

    req = (data_subscribe_req_t *)init_msg(&msg, MSG_SERVICE_SUBSCRIBE_REQ, MAKE_SYSTEM_SERV_ROUT_ID(cid),
                                           MAKE_ROUT_ID(DATA_CONN_INVALID_ID, DATA_SERVICE_INVALID_ID), body_len);
    strncpy(req->service_name, service_name, name_len);
    req->service_name[name_len - 1]  = 0;
    result = datac_subscribe_int(&msg);

    return;

__ERR:

    ds_exit_critical();

    subscribe_rsp.result = -1;
    subscribe_rsp.handle = handle;
    arg_mq.arg.msg_id = MSG_SERVICE_SUBSCRIBE_RSP;
    arg_mq.arg.data_len = sizeof(subscribe_rsp);
    arg_mq.arg.user_data = user_data;
    if (mq)
    {
        arg_mq.callback = cbk;
        arg_mq.arg.data = rt_malloc(sizeof(subscribe_rsp));
        RT_ASSERT(arg_mq.arg.data);
        memcpy(arg_mq.arg.data, &subscribe_rsp, sizeof(subscribe_rsp));
        if (RT_EOK != rt_mq_send(mq, &arg_mq, sizeof(arg_mq)))
        {
            LOG_D("datac(%d) usrcbk Qfull,msg:%d,data:%x,len:%d",
                  DATA_CLIENT_INVALID_HANDLE, arg_mq.arg.msg_id, arg_mq.arg.data, arg_mq.arg.data_len);
            RT_ASSERT(0); //send msg fail.
        }
    }
    else if (cbk)
    {
        arg_mq.arg.data = (uint8_t *)&subscribe_rsp;
        cbk(&arg_mq.arg);
    }

    return;

}

rt_err_t datac_unsubscribe_ex(datac_handle_t handle, bool force)
{
    data_msg_t msg;
    rt_err_t result;
    data_connection_t *conn;
    uint8_t cid;

    if (!is_valid_handle(handle))
    {
        goto __ERR;
    }

    ds_enter_critical();

    cid = DS_CLIENT_HANDLE_2_CID(handle);
    conn = get_conn(cid);
    conn->state = DATA_STATE_UNSUB_PENDING;
    ds_exit_critical();

    if (force)
        conn->callback = NULL;
    init_msg(&msg, MSG_SERVICE_UNSUBSCRIBE_REQ,
             MAKE_SYSTEM_SERV_ROUT_ID(cid), conn->dst_cid, 0);

    result = datac_unsubscribe_int(&msg);

    return result;

__ERR:

    return -RT_ERROR;
}

rt_err_t datac_config(datac_handle_t handle, uint16_t len, uint8_t *config)
{
    rt_err_t result = RT_EOK;
    data_req_t *req;
    data_msg_t msg;
    data_connection_t *conn;
    uint8_t cid;

    if (!is_valid_handle(handle))
    {
        goto __ERR;
    }

    cid = DS_CLIENT_HANDLE_2_CID(handle);
    conn = get_conn(cid);

    req = (data_req_t *)init_msg(&msg, MSG_SERVICE_CONFIG_REQ, MAKE_SYSTEM_SERV_ROUT_ID(cid), conn->dst_cid, sizeof(*req) + len);
    req->len = len;
    memcpy(&req->data[0], config, len);

    result = dispatch_msg(&msg);

    return result;

__ERR:

    return -RT_ERROR;
}

rt_err_t datac_send_msg(datac_handle_t handle, data_msg_t *msg)
{
    rt_err_t result = RT_EOK;
    data_connection_t *conn;
    uint8_t cid;

    RT_ASSERT(msg);

    if (!is_valid_handle(handle))
    {
        goto __ERR;
    }

    cid = DS_CLIENT_HANDLE_2_CID(handle);
    conn = get_conn(cid);

    msg->src_cid = MAKE_SYSTEM_SERV_ROUT_ID(cid);
    msg->dst_cid = conn->dst_cid;

    result = dispatch_msg(msg);

    return result;

__ERR:

    return -RT_ERROR;
}

rt_err_t datac_tx(datac_handle_t handle, uint16_t len, uint8_t *data)
{
    rt_err_t result = RT_EOK;
    data_req_t *req;
    data_msg_t msg;
    data_connection_t *conn;
    uint8_t cid;

    if (!is_valid_handle(handle))
    {
        goto __ERR;
    }

    cid = DS_CLIENT_HANDLE_2_CID(handle);
    conn = get_conn(cid);

    req = (data_req_t *)init_msg(&msg, MSG_SERVICE_TX_REQ, MAKE_SYSTEM_SERV_ROUT_ID(cid), conn->dst_cid, sizeof(*req) + len);
    req->len = len;

    memcpy(&req->data[0], data, len);
    result = dispatch_msg(&msg);

    return result;

__ERR:

    return -RT_ERROR;
}

rt_err_t datac_rx(datac_handle_t handle, uint16_t len, uint8_t *data)
{
    rt_err_t result = RT_EOK;
    data_req_t *req;
    data_msg_t msg;
    data_connection_t *conn;
    uint8_t cid;

    if (!is_valid_handle(handle))
    {
        goto __ERR;
    }

    cid = DS_CLIENT_HANDLE_2_CID(handle);
    conn = get_conn(cid);
    conn->data = data;

    req = (data_req_t *)init_msg(&msg, MSG_SERVICE_RX_REQ, MAKE_SYSTEM_SERV_ROUT_ID(cid), conn->dst_cid, sizeof(*req));
    req->len = sizeof(len);
    req->data[0] = len & 0xff;
    req->data[1] = len >> 8;
    result = dispatch_msg(&msg);

    return result;

__ERR:
    return -RT_ERROR;
}

rt_err_t datac_ping(datac_handle_t handle, uint8_t mode)
{
    rt_err_t result = RT_EOK;
    data_req_t *req;
    data_msg_t msg;
    data_connection_t *conn;
    uint8_t cid;

    if (!is_valid_handle(handle))
    {
        goto __ERR;
    }

    cid = DS_CLIENT_HANDLE_2_CID(handle);
    conn = get_conn(cid);

    req = (data_req_t *)init_msg(&msg, MSG_SERVICE_PING_REQ, MAKE_SYSTEM_SERV_ROUT_ID(cid), conn->dst_cid, sizeof(*req) + 1);
    req->len = 1;
    req->data[0] = mode;

    result = dispatch_msg(&msg);

    return result;

__ERR:

    return -RT_ERROR;
}


/*
    Start data service,
*/
int datas_start(data_service_init_param_t *init_param)
{
    rt_err_t err;
    void *stack;

    if (!init_param)
    {
        return -1;
    }

    rt_mutex_init(&g_data_service_mutex, "dserv", 0);
#ifndef DS_MBOX_DISABLED
    if (init_param->mbox_thread_stack_size > 0)
    {
        rt_mutex_init(&g_ds_ipc_mutex, "ds_ipc", 0);
    }
#endif /* !DS_MBOX_DISABLED */

    /* init service db link list */
    if (NULL == data_service_db.next)
    {
        rt_list_init(&data_service_db);
    }

    sys_service = add_service("SYS", &sys_service_config, DATA_SERVICE_SYSTEM_SERVICE_ID);

#ifndef DATA_SVC_PROC_THREAD_DISABLED
    if (init_param->proc_thread_stack_size > 0)
    {
        g_ds_queue = rt_mq_create("dserv", sizeof(data_msg_t), 30, RT_IPC_FLAG_FIFO);
        RT_ASSERT(g_ds_queue);
    }
#endif /* !DATA_SVC_PROC_THREAD_DISABLED */

#ifndef DS_MBOX_DISABLED
    if (init_param->mbox_thread_stack_size > 0)
    {
        g_data_mbox_mq = rt_mq_create("data_mb_mq", sizeof(data_mbox_mq_data_t), 30, RT_IPC_FLAG_FIFO);
        RT_ASSERT(RT_NULL != g_data_mbox_mq);

        if (init_param->mbox_thread_stack)
        {
            stack = init_param->mbox_thread_stack;
            rt_kprintf("mbox_stack:%p,%d\n", stack, init_param->mbox_thread_stack_size);
        }
        else
        {
            stack = rt_malloc(init_param->mbox_thread_stack_size);
            RT_ASSERT(stack);
        }
        err = rt_thread_init(&g_data_mbox_thread, "ds_mb", data_mbox_entry, NULL, stack,
                             init_param->mbox_thread_stack_size,
                             init_param->mbox_thread_priority, RT_THREAD_TICK_DEFAULT);

        RT_ASSERT(RT_EOK == err);
        rt_thread_startup(&g_data_mbox_thread);
    }
#endif /* !DS_MBOX_DISABLED */

#ifndef DATA_SVC_PROC_THREAD_DISABLED
    if (init_param->proc_thread_stack_size > 0)
    {
        if (init_param->proc_thread_stack)
        {
            stack = init_param->proc_thread_stack;
            rt_kprintf("proc_stack:%p,%d\n", stack, init_param->proc_thread_stack_size);
        }
        else
        {
            stack = rt_malloc(init_param->proc_thread_stack_size);
            RT_ASSERT(stack);
        }
        err = rt_thread_init(&g_data_process_thread, "ds_proc", data_service_entry, (void *)g_ds_queue,
                             stack, init_param->proc_thread_stack_size,
                             init_param->proc_thread_priority, RT_THREAD_TICK_DEFAULT);

        rt_thread_startup(&g_data_process_thread);
    }
#endif /* !DATA_SVC_PROC_THREAD_DISABLED */

#ifndef DS_MBOX_DISABLED
    if (init_param->mbox_thread_stack_size > 0)
    {
        ds_init_proxy(proxy_rx_ind);
    }
    else
    {
        g_proxy = IPC_QUEUE_INVALID_HANDLE;
    }
#endif /* !DS_MBOX_DISABLED */

#if defined(SOC_BF0_HCPU) && !defined(DS_MBOX_DISABLED)
#ifndef DATA_SVC_PROC_THREAD_DISABLED
    if (0 == init_param->proc_thread_stack_size)
#else
    if (1)
#endif /* DATA_SVC_PROC_THREAD_DISABLED */
    {
        if (g_data_mbox_mq)
        {
            lcpu_power_on();
        }
    }
#endif /* SOC_BF0_HCPU */

    return 0;
}

uint8_t *data_service_get_msg_body(data_msg_t *msg)
{
    uint8_t *body;
    uint8_t **body_ext;

    body_ext = data_service_msg_body_ext(msg);
    if (body_ext)
    {
        body = *body_ext;
    }
    else
    {
        body = &((data_msg_t *)msg)->body[0];
    }

    return body;
}


uint8_t *data_service_init_msg(data_msg_t *msg, uint16_t msg_id, uint16_t body_len)
{
    uint8_t *body;
    body = init_msg(msg, msg_id, 0, 0, body_len);

    return body;
}
void data_service_deinit_msg(data_msg_t *msg)
{
    if (data_service_msg_body_ext((data_msg_t *)msg))
    {
        free_msg(msg);
    }
}

void *datas_service_get_client(data_service_t *service, data_msg_t *msg)
{
    uint8_t conn_id;

    conn_id = GET_ROUT_ID_CONN_ID(msg->dst_cid);

    RT_ASSERT(conn_id < service->config->max_client_num);

    return service->client_list[conn_id];
}

void *data_service_get_thread(uint8_t ds_id)
{
#ifndef DS_MBOX_DISABLED
    if (ds_id == DATA_SVC_THREAD_MB)
        return (void *) &g_data_mbox_thread;
#endif /* !DS_MBOX_DISABLED */

#ifndef DATA_SVC_PROC_THREAD_DISABLED
    if (ds_id == DATA_SVC_THREAD_PROC)
        return (void *) &g_data_process_thread;
#endif /* !DATA_SVC_PROC_THREAD_DISABLED */

    return NULL;
}

int32_t datas_push_msg_to_client(datas_handle_t svc, uint16_t msg_id, uint32_t len, uint8_t *data)
{
    rt_list_t *iter;
    data_service_t *service = (data_service_t *)svc;
    uint8_t *conn_id;
    uint16_t *src_cid;
    uint8_t client_num;
    uint8_t max_client_num;
    uint16_t buf_size;
    uint32_t i;

    if (!sys_service)
    {
        /* data service not ready yet */
        return -1;
    }

    ds_enter_critical();

    max_client_num = service->config->max_client_num;
    buf_size = max_client_num * (sizeof(*conn_id) + sizeof(*src_cid));
    src_cid = rt_malloc(buf_size);
    RT_ASSERT(src_cid);
    conn_id = (uint8_t *)(src_cid + max_client_num);
    client_num = 0;
    rt_list_for_each(iter, (&service->clients))
    {
        data_service_client_t *client = rt_list_entry(iter, data_service_client_t, node);
        bool send;

        //check report conditon of each client
        if (service->config->data_filter && client->config)
            send = service->config->data_filter(client->config, msg_id, len, data);
        else
            send = true;

        if (send)
        {
            /* save client info in buffer as user callback may delete client */
            RT_ASSERT(client_num < max_client_num);
            conn_id[client_num] = client->conn_id;
            src_cid[client_num] = client->src_cid;
            client_num++;
        }
    }
    ds_exit_critical();

    for (i = 0; i < client_num; i++)
    {
        uint8_t *body;
        data_msg_t msg;
        uint16_t body_len;
        body_len = len;
        body = init_msg(&msg, msg_id,
                        MAKE_ROUT_ID(conn_id[i], service->id),
                        src_cid[i], body_len);
        memcpy(body, data, len);
        dispatch_msg(&msg);
    }

    rt_free(src_cid);

    return 0;
}

int32_t datas_push_msg_to_client_no_copy(datas_handle_t svc, uint16_t msg_id, uint32_t len, uint8_t *data)
{
    rt_list_t *iter;
    data_service_t *service = (data_service_t *)svc;
    uint8_t *conn_id;
    uint16_t *src_cid;
    uint8_t client_num;
    uint8_t max_client_num;
    uint16_t buf_size;
    uint32_t i;

    if (!sys_service)
    {
        /* data service not ready yet */
        return -1;
    }

    ds_enter_critical();

    max_client_num = service->config->max_client_num;
    buf_size = max_client_num * (sizeof(*conn_id) + sizeof(*src_cid));
    src_cid = rt_malloc(buf_size);
    conn_id = (uint8_t *)(src_cid + max_client_num);
    client_num = 0;
    rt_list_for_each(iter, (&service->clients))
    {
        data_service_client_t *client = rt_list_entry(iter, data_service_client_t, node);
        bool send;

        //check report conditon of each client
        if (service->config->data_filter && client->config)
            send = service->config->data_filter(client->config, msg_id, len, data);
        else
            send = true;

        if (send)
        {
            /* save client info in buffer as user callback may delete client */
            RT_ASSERT(client_num < max_client_num);
            conn_id[client_num] = client->conn_id;
            src_cid[client_num] = client->src_cid;
            client_num++;
        }
    }
    ds_exit_critical();

    for (i = 0; i < client_num; i++)
    {
        uint8_t *body;
        data_msg_t msg;
        uint16_t body_len;
        body_len = len;
        body = init_msg_no_copy(&msg, msg_id,
                                MAKE_ROUT_ID(conn_id[i], service->id),
                                src_cid[i], body_len, data);
        dispatch_msg(&msg);
    }

    rt_free(src_cid);

    return 0;
}

rt_err_t datas_data_ready(datas_handle_t svc, uint32_t size, uint8_t *data)
{
    data_service_t *service = (data_service_t *)svc;
    rt_err_t result = -RT_ERROR;
    if (service)
    {
        data_msg_t msg;
        data_rdy_ind_t *body;
        body = (data_rdy_ind_t *)data_service_init_msg((data_msg_t *)&msg, MSG_SERVICE_DATA_RDY_IND, sizeof(data_rdy_ind_t));
        body->len = size;
        body->data = data;
        msg.dst_cid = MAKE_ROUT_ID(DATA_CONN_INVALID_ID, service->id);
        result = dispatch_msg(&msg);
        RT_ASSERT(RT_EOK == result);
    }
    return result;
}



rt_err_t datas_send_response_data(datas_handle_t svc, data_msg_t *msg_req, uint32_t len, uint8_t *data)
{
    data_service_t *service = (data_service_t *)svc;
    data_msg_t msg_rsp;
    uint8_t *rsp;
    rsp = data_service_init_msg((data_msg_t *)&msg_rsp, (msg_req->msg_id) | RSP_MSG_TYPE, len);
    msg_rsp.src_cid = msg_req->dst_cid;
    msg_rsp.dst_cid = msg_req->src_cid;

    memcpy(rsp, data, len);

    if (msg_req->msg_id == MSG_SERVICE_CONFIG_REQ && service->config->data_filter)
    {
        // Save config in client for future data filtering.
        uint8_t conn_id;
        data_service_client_t *clnt;
        conn_id = GET_ROUT_ID_CONN_ID(msg_req->dst_cid);
        clnt = get_clnt(service, conn_id);
        if (clnt)
        {
            data_req_t *cfg;
            if (clnt->config)
                rt_free(clnt->config);
            clnt->config = rt_malloc(msg_req->len + sizeof(data_req_t));
            cfg = (data_req_t *)data_service_get_msg_body(msg_req);
            clnt->config->len = cfg->len;
            memcpy(clnt->config->data, cfg->data, cfg->len);
        }
    }

    return dispatch_msg(&msg_rsp);
}

rt_err_t datas_send_response(datas_handle_t svc, data_msg_t *msg_req, rt_err_t result)
{
    data_rsp_t rsp;
    rsp.result = result;

    return datas_send_response_data(svc, msg_req, sizeof(data_rsp_t), (uint8_t *)&rsp);
}

datas_handle_t datas_register(const char *name, data_service_config_t *config)
{
    uint8_t serv_id;

    /* init service db link list */
    if (NULL == data_service_db.next)
    {
        rt_list_init(&data_service_db);
    }

    serv_id = DATA_SERVICE_INVALID_ID;
    /* position 0 is reserved to system service */
    for (uint32_t i = 1; i <  DATA_SERVICE_MAX_NUM; i++)
    {
        if (NULL == data_service_list[i])
        {
            serv_id = i;
            break;
        }
    }

    if (DATA_SERVICE_INVALID_ID == serv_id)
    {
        return NULL;
    }

    return (datas_handle_t)add_service(name, config, serv_id);
}

#if defined(RT_USING_FINSH) && !defined(LCPU_MEM_OPTIMIZE)
void list_data_client(void)
{
    uint32_t i;
    data_connection_t *conn_user_data;
    data_service_client_t *clnt;
    const char *serv_name;
    uint8_t state;
    uint16_t dst_cid;
    data_callback_t callback;

    LOG_D("hdl    service    state   dst_cid  callback  ");
    for (i = 0; i < sys_service->config->max_client_num; i++, clnt++)
    {
        state = DATA_STATE_IDLE;
        dst_cid = 0;
        callback = NULL;
        serv_name = "NULL";

        clnt = get_clnt(sys_service, i);
        conn_user_data = NULL;
        if (clnt)
        {
            conn_user_data = (data_connection_t *)clnt->user_data;
            state = conn_user_data->state;
            dst_cid = conn_user_data->dst_cid;
            callback = conn_user_data->callback;
            serv_name = conn_user_data->svc_name;
            LOG_D("[%02d]   %-10s %-7d %04x     0x%08x",
                  DS_CID_2_CLIENT_HANDLE(i), serv_name, state, dst_cid, callback);
        }
    }
}
MSH_CMD_EXPORT(list_data_client, view data service client);

void list_data_service_summary(void)
{
    uint32_t j;
    const char *serv_name;
    uint8_t client_num;
    data_service_t *service;
    rt_list_t *iter;

    LOG_D("id    service    c_num ");
    rt_list_for_each(iter, &data_service_db)
    {
        service = rt_list_entry(iter, data_service_t, node);
        serv_name = &service->name[0];
        client_num = 0;
        for (j = 0; j < service->config->max_client_num; j++)
        {
            if (service->client_list[j])
            {
                client_num++;
            }
        }

        LOG_D("[%02d]  %-10s %d ",
              service->id, serv_name, client_num);
    }
}

void list_data_service_detail(char *serv_name)
{
    data_service_t *service;
    rt_list_t *iter;
    data_service_client_t *client;

    service = find_service(serv_name);

    if (!service)
    {
        LOG_D("Service %s not found", serv_name);
        return;
    }

    LOG_D("-------------- ");
    LOG_D("serv_id    name ");
    LOG_D("[%02d]       %-15s", service->id, serv_name);

    LOG_D("-------------- ");
    LOG_D("cid    src_cid ");
    rt_list_for_each(iter, (&service->clients))
    {
        client = rt_list_entry(iter, data_service_client_t, node);
        LOG_D("[%02d]   %04x ", client->conn_id, client->src_cid);
    }
}


void list_data_service(int argc, char **argv)
{
    if (argc > 1)
    {
        if (0 == strcmp(argv[1], "summary"))
        {
            list_data_service_summary();
        }
        else if (0 == strcmp(argv[1], "detail"))
        {
            if (argc > 2)
            {
                list_data_service_detail(argv[2]);
            }
            else
            {
                LOG_D("No service name specified");
            }
        }
        else
        {
            LOG_D("Wrong param: %s", argv[1]);
        }
    }
    else
    {
        list_data_service_summary();
    }

}
MSH_CMD_EXPORT(list_data_service, view data service provider);
#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
