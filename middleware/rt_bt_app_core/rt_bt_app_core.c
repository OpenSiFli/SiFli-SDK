/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include "rt_bt_app.h"

#define DBG_TAG "rt_bt_app.core"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#ifndef BT_DEVICE_NAME
    #define BT_DEVICE_NAME "bt_device"
#endif

/* Service thread and queue sizes. */
#define BT_APP_EVT_ARG_MAX     64
#define BT_APP_EVT_QUEUE_LEN   30
#define BT_APP_SVC_STACK_SIZE  3072
#define BT_APP_SVC_PRIORITY    RT_THREAD_PRIORITY_MIDDLE

/* Local device name broadcast after the stack becomes ready. */
#ifndef BT_APP_LOCAL_NAME
    #define BT_APP_LOCAL_NAME "sifli_rt_bt"
#endif

/**
 * @brief   Self-contained copy of a single BT event, passed through the queue by
 *          pointer. The parameter payload is stored inline in @ref arg, so it
 *          remains valid after the notification context returns.
 */
typedef struct
{
    rt_uint16_t event;
    rt_size_t   arg_len;                 /**< Number of valid bytes in @ref arg (0 = none) */
    rt_uint8_t  arg[BT_APP_EVT_ARG_MAX]; /**< Deep-copied parameter payload */
} bt_app_evt_msg_t;

/* ---------------------------------------------------------------------------
 * Module state
 * ------------------------------------------------------------------------- */

static rt_device_t      s_bt_dev = RT_NULL;
static volatile rt_bool_t s_stack_ready = RT_FALSE;

static rt_mq_t          s_evt_queue = RT_NULL;
static struct rt_thread s_svc_thread;
static rt_uint8_t       s_svc_stack[BT_APP_SVC_STACK_SIZE];

/* Head of the registered service linked list. */
static rt_bt_service_t    *s_services = RT_NULL;

static struct rt_mutex    s_svc_lock;

/* ---------------------------------------------------------------------------
 * Service registry
 * ------------------------------------------------------------------------- */

static int rt_bt_app_svc_lock_init(void)
{
    rt_mutex_init(&s_svc_lock, "bt_svc", RT_IPC_FLAG_FIFO);
    return 0;
}
INIT_PREV_EXPORT(rt_bt_app_svc_lock_init);

rt_err_t rt_bt_app_register_service(rt_bt_service_t *svc)
{
    if (svc == RT_NULL || svc->name == RT_NULL)
    {
        return -RT_EINVAL;
    }

    /* Insert at the list head; ordering does not matter for routing. */
    rt_mutex_take(&s_svc_lock, RT_WAITING_FOREVER);
    svc->next  = s_services;
    s_services = svc;
    rt_mutex_release(&s_svc_lock);
    LOG_I("service \"%s\" registered (group 0x%x)", svc->name, svc->event_group);
    return RT_EOK;
}

rt_bt_service_t *rt_bt_app_service_list(void)
{
    return s_services;
}

rt_bt_service_t *rt_bt_app_service_find(const char *name)
{
    rt_bt_service_t *s;

    rt_mutex_take(&s_svc_lock, RT_WAITING_FOREVER);
    for (s = s_services; s != RT_NULL; s = s->next)
    {
        if (rt_strcmp(s->name, name) == 0)
        {
            break;
        }
    }
    rt_mutex_release(&s_svc_lock);
    return s;
}

/** @brief  Find the service that owns an event by its high byte. */
static rt_bt_service_t *bt_app_service_by_event(rt_uint16_t event)
{
    rt_uint8_t    group = (rt_uint8_t)(event >> 8);
    rt_bt_service_t *s;

    rt_mutex_take(&s_svc_lock, RT_WAITING_FOREVER);
    for (s = s_services; s != RT_NULL; s = s->next)
    {
        if (s->event_group == group)
        {
            break;
        }
    }
    rt_mutex_release(&s_svc_lock);
    return s;
}

/* ---------------------------------------------------------------------------
 * Control interface
 * ------------------------------------------------------------------------- */

bt_err_t rt_bt_app_control(int cmd, void *args)
{
    if (s_bt_dev == RT_NULL)
    {
        LOG_W("BT device not open, cmd 0x%x ignored", cmd);
        return BT_ERROR_STATE;
    }
    return (bt_err_t)rt_device_control(s_bt_dev, cmd, args);
}

rt_bool_t rt_bt_app_is_stack_ready(void)
{
    return s_stack_ready;
}

/* ---------------------------------------------------------------------------
 * Built-in handling for common events (BT_COMMON_TYPE_ID group)
 * ------------------------------------------------------------------------- */

static void log_bt_addr(const char *prefix, const bt_mac_t *mac)
{
    LOG_I("%s %02x:%02x:%02x:%02x:%02x:%02x", prefix,
          (rt_uint8_t)mac->addr[0], (rt_uint8_t)mac->addr[1],
          (rt_uint8_t)mac->addr[2], (rt_uint8_t)mac->addr[3],
          (rt_uint8_t)mac->addr[4], (rt_uint8_t)mac->addr[5]);
}

static void bt_app_handle_common_event(rt_uint16_t event, void *args)
{
    switch (event)
    {
    case BT_EVENT_BT_STACK_READY:
    {
        set_name_t name;
        s_stack_ready = RT_TRUE;
        LOG_I("BT stack ready");
        rt_bt_app_control(BT_CONTROL_OPEN_DEVICE, RT_NULL);
        name.name = BT_APP_LOCAL_NAME;
        name.size = (int)rt_strlen(BT_APP_LOCAL_NAME);
        rt_bt_app_control(BT_CONTROL_SET_LOCAL_NAME, &name);
        break;
    }

    case BT_EVENT_INQ:
    {
        bt_serached_device_info_t *inq = (bt_serached_device_info_t *)args;
        LOG_I("device \"%s\" searched", inq->bt_name ? inq->bt_name : "");
        LOG_I("device COD is 0x%x, rssi %d", (unsigned)inq->dev_cls, inq->rssi);
        log_bt_addr("device addr is", &inq->mac_addr);
        break;
    }

    case BT_EVENT_INQ_FINISHED:
        LOG_I("inquiry completed");
        break;

    case BT_EVENT_CONNECT_COMPLETE:
        LOG_I("profile 0x%x connected", ((bt_connect_info_t *)args)->profile);
        break;

    case BT_EVENT_PROFILE_DISCONNECT:
        LOG_I("profile 0x%x disconnected", ((bt_disconnect_info_t *)args)->profile);
        break;

    case BT_EVENT_DISCONNECT:
        LOG_I("BT device disconnected, reason %d", ((bt_acl_disconnect_info_t *)args)->reason);
        break;

    default:
        LOG_I("unhandled common event 0x%x", event);
        break;
    }
}

/* ---------------------------------------------------------------------------
 * Deep copy: clone implementation for common events
 * ------------------------------------------------------------------------- */

/**
 * @brief   Deep copy for common events (BT_COMMON_TYPE_ID).
 *
 * The device name in BT_EVENT_INQ is a pointer and needs to be expanded into the
 * buffer tail; other common events are fixed-size structures.
 */
static rt_size_t bt_app_common_clone(rt_uint16_t event, void *args,
                                     void *buf, rt_size_t cap)
{
    if (event == BT_EVENT_INQ)
    {
        /* Expand the device name into the buffer after the structure so the pointer
         * remains valid in the service thread. */
        bt_serached_device_info_t *src = (bt_serached_device_info_t *)args;
        bt_serached_device_info_t *dst = (bt_serached_device_info_t *)buf;
        rt_size_t hdr = sizeof(*dst);
        rt_size_t nlen;

        if (cap < hdr + 1)
        {
            return 0;
        }
        *dst = *src;
        nlen = src->name_size;
        if (nlen > cap - hdr - 1)
        {
            nlen = cap - hdr - 1;
        }
        if (src->bt_name != RT_NULL && nlen > 0)
        {
            rt_memcpy((rt_uint8_t *)buf + hdr, src->bt_name, nlen);
        }
        ((char *)buf)[hdr + nlen] = '\0';
        dst->bt_name   = (char *)buf + hdr;
        dst->name_size = nlen;
        return hdr + nlen + 1;
    }

    /* Fixed-size common events: copy exactly by structure size. */
    {
        rt_size_t n;

        switch (event)
        {
        case BT_EVENT_CONNECT_COMPLETE:
            n = sizeof(bt_connect_info_t);
            break;
        case BT_EVENT_PROFILE_DISCONNECT:
            n = sizeof(bt_disconnect_info_t);
            break;
        case BT_EVENT_DISCONNECT:
            n = sizeof(bt_acl_disconnect_info_t);
            break;
        default:
            n = 0;
            break;
        }
        if (n == 0 || n > cap)
        {
            return 0;
        }
        rt_memcpy(buf, args, n);
        return n;
    }
}

/* ---------------------------------------------------------------------------
 * Service thread: dispatch events from the queue
 * ------------------------------------------------------------------------- */

static void bt_app_dispatch(bt_app_evt_msg_t *msg)
{
    void         *args = (msg->arg_len > 0) ? (void *)msg->arg : RT_NULL;
    rt_bt_service_t *svc  = bt_app_service_by_event(msg->event);

    if (svc != RT_NULL && svc->on_event != RT_NULL)
    {
        svc->on_event(msg->event, args);
    }
    else if ((rt_uint8_t)(msg->event >> 8) == BT_COMMON_TYPE_ID)
    {
        bt_app_handle_common_event(msg->event, args);
    }
    else
    {
        LOG_I("no service for event 0x%x", msg->event);
    }
}

static void bt_app_svc_thread_entry(void *parameter)
{
    bt_app_evt_msg_t *msg = RT_NULL;

    (void)parameter;

    while (1)
    {
        if (rt_mq_recv(s_evt_queue, &msg, sizeof(msg), RT_WAITING_FOREVER) == RT_EOK)
        {
            bt_app_dispatch(msg);
            rt_free(msg);
        }
    }
}

/* ---------------------------------------------------------------------------
 * BT event callback (called in the bt_device notification context)
 * ------------------------------------------------------------------------- */

static void bt_app_event_callback(bt_notify_t *param)
{
    bt_app_evt_msg_t *msg;
    rt_bt_service_t  *svc;
    rt_uint16_t       event;
    void             *args;
    rt_size_t         arg_len = 0;

    if (param == RT_NULL)
    {
        return;
    }
    event = param->event;
    args  = param->args;

    /* Allocate the message (including the deep-copied arguments). */
    msg = (bt_app_evt_msg_t *)rt_malloc(sizeof(*msg));
    if (msg == RT_NULL)
    {
        LOG_E("no memory for event 0x%x", event);
        return;
    }

    msg->event = event;

    /* Complete the deep copy while the arguments are still valid. Choose the clone
     * strategy by ownership: first try the service's own clone(), otherwise use the
     * generic default implementation. */
    if (args != RT_NULL)
    {
        svc = bt_app_service_by_event(event);
        if (svc != RT_NULL && svc->clone != RT_NULL)
        {
            arg_len = svc->clone(event, args, msg->arg, sizeof(msg->arg));
        }
        else if (svc == RT_NULL && (rt_uint8_t)(event >> 8) == BT_COMMON_TYPE_ID)
        {
            arg_len = bt_app_common_clone(event, args, msg->arg, sizeof(msg->arg));
        }
        else
        {
            LOG_W("event 0x%x has no clone hook, args dropped", event);
            arg_len = 0;
        }
    }

    msg->arg_len = arg_len;

    /* Post the message to the service thread queue. */
    if (rt_mq_send(s_evt_queue, &msg, sizeof(msg)) != RT_EOK)
    {
        LOG_E("send event 0x%x to queue failed", event);
        rt_free(msg);
    }
}

/* ---------------------------------------------------------------------------
 * Core initialization
 * ------------------------------------------------------------------------- */

rt_err_t rt_bt_app_core_init(void)
{
    rt_err_t ret;

    LOG_I("initializing RT-Thread BT app core");

    /* Create the event queue. */
    s_evt_queue = rt_mq_create("bt_evt", sizeof(bt_app_evt_msg_t *),
                               BT_APP_EVT_QUEUE_LEN, RT_IPC_FLAG_FIFO);
    if (s_evt_queue == RT_NULL)
    {
        LOG_E("create event queue failed");
        return -RT_ENOMEM;
    }

    ret = rt_thread_init(&s_svc_thread, "bt_svc", bt_app_svc_thread_entry, RT_NULL,
                         s_svc_stack, sizeof(s_svc_stack), BT_APP_SVC_PRIORITY, 4);
    if (ret != RT_EOK)
    {
        LOG_E("init service thread failed: %d", ret);
        rt_mq_delete(s_evt_queue);
        s_evt_queue = RT_NULL;
        return ret;
    }
    rt_thread_startup(&s_svc_thread);

    /* Find the BT device and subscribe to its events. */
    s_bt_dev = rt_device_find(BT_DEVICE_NAME);
    if (s_bt_dev == RT_NULL)
    {
        LOG_E("BT device \"%s\" not found", BT_DEVICE_NAME);
        return -RT_ERROR;
    }
    LOG_I("BT device \"%s\" found", BT_DEVICE_NAME);

    ret = (rt_err_t)rt_bt_app_control(BT_CONTROL_REGISTER_NOTIFY,
                                      (void *)bt_app_event_callback);
    if (ret != BT_EOK)
    {
        LOG_E("register notify callback failed: 0x%x", ret);
        return ret;
    }
    return RT_EOK;
}
