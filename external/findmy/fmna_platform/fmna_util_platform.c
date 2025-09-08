/*
 * SPDX-FileCopyrightText: 2026-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>
#include <stdlib.h>

#include <rtthread.h>
#include <rtdevice.h>

#include "bf0_ble_gap.h"
#include "bf0_sibles.h"
#include "bf0_sibles_internal.h"
#include "bf0_sibles_advertising.h"
#include "ble_connection_manager.h"

#include "fmna_util_platform.h"
#include "fmna_state_machine.h"

#include "log.h"

#ifndef RT_USING_SYSTEM_WORKQUEUE
/* 配置宏：根据实际需求调整 */
/* --- 配置参数 --- */
#define SCHED_QUEUE_MAX_MSGS    16      // 队列深度
#define SCHED_DATA_MAX_SIZE     32      // 单个事件最大负载 (根据需要调大)
#define SCHED_THREAD_STACK      4096    // 调度线程栈大小
#define SCHED_THREAD_PRIO       15      // 优先级

/* 定义事件处理函数指针类型 */
typedef void (*app_sched_event_handler_t)(void *p_event_data, uint16_t event_size);

/* 定义回调函数原型 */
typedef void (*app_sched_handler_t)(void *p_data, uint16_t size);

typedef struct {
    app_sched_handler_t handler_func;
    uint16_t            data_size;
    uint16_t            reserved;       // 显式对齐填充

    uint8_t             data_buf[SCHED_DATA_MAX_SIZE];
} sched_evt_t;

static rt_mq_t _sched_mq = RT_NULL;
static rt_thread_t _sched_tid = RT_NULL;

/* 调度线程入口 */
static void _sched_thread_entry(void *parameter)
{
    /* * 关键点：将 current_evt 放在静态区或确保栈足够大
     * 避免在循环中反复在栈上创建大的结构体
     */
    static sched_evt_t current_evt; 

    while (1)
    {
        // 阻塞等待，直到收到事件
        if (rt_mq_recv(_sched_mq, &current_evt, sizeof(sched_evt_t), RT_WAITING_FOREVER) == RT_EOK)
        {
            if (current_evt.handler_func != RT_NULL)
            {
                // 执行回调，传入 payload 的起始地址
                current_evt.handler_func(current_evt.data_buf, current_evt.data_size);
            }
        }
    }
}

/**
 * event put
 * @param event_data      数据指针
 * @param event_data_size 数据大小
 * @param handler         回调函数 (void* 类型，内部转为函数指针)
 */
int sched_event_put(void *event_data, uint16_t event_data_size, void *handler)
{
    // 1. 安全检查
    if (_sched_mq == RT_NULL || handler == RT_NULL) 
    {
        return -RT_ERROR;
    }
    
    if (event_data_size > SCHED_DATA_MAX_SIZE)
    {
        rt_kprintf("Sched Error: Data too large (%d > %d)\n", event_data_size, SCHED_DATA_MAX_SIZE);
        return -RT_EINVAL;
    }

    // 2. 构造消息体
    sched_evt_t evt;
    rt_memset(&evt, 0, sizeof(sched_evt_t));
    
    evt.handler_func = (app_sched_handler_t)handler;
    evt.data_size    = event_data_size;

    if (event_data != RT_NULL && event_data_size > 0)
    {
        rt_memcpy(evt.data_buf, event_data, event_data_size);
    }

    /* * 3. 发送消息到队列
     * 注意：在中断中使用必须设为 RT_WAITING_NOWAIT
     */
    return rt_mq_send(_sched_mq, &evt, sizeof(sched_evt_t));
}

/* 初始化函数 */
int app_sched_init(void)
{
    if (_sched_mq != RT_NULL) return RT_EOK;

    // 创建消息队列
    _sched_mq = rt_mq_create("app_sch", 
                             sizeof(sched_evt_t), 
                             SCHED_QUEUE_MAX_MSGS, 
                             RT_IPC_FLAG_FIFO);
    
    if (_sched_mq == RT_NULL) return -RT_ENOMEM;

    // 创建调度线程
    _sched_tid = rt_thread_create("app_sch",
                                  _sched_thread_entry,
                                  RT_NULL,
                                  SCHED_THREAD_STACK,
                                  SCHED_THREAD_PRIO,
                                  20);

    if (_sched_tid != RT_NULL)
    {
        rt_thread_startup(_sched_tid);
    }
    else
    {
        return -RT_ENOMEM;
    }

    return RT_EOK;
}
INIT_APP_EXPORT(app_sched_init);

void app_sched_event_put(void *event_data, uint16_t event_data_size, void *handler)
{
    LOG_D("app_sched_event_put %d", event_data_size);
    sched_event_put(event_data, event_data_size, handler);
}

#else

/* 定义回调函数原型 */
typedef void (*app_sched_handler_t)(void *p_data, uint16_t size);

/* 工作项结构体：包含 rt_work 和 数据 */
struct app_sched_work_item
{
    struct rt_work work;          /* RT-Thread 标准工作结构体 */
    app_sched_handler_t handler;  /* 用户回调 */
    uint16_t size;                /* 数据长度 */
    /* 使用柔性数组或紧跟其后的内存存放数据，确保对齐 */
    uint8_t data_storage[];   
};

/**
 * 工作队列回调包装函数
 */
static void _app_sched_work_wrapper(struct rt_work *work, void *work_data)
{
    // 通过 rt_container_of 找回结构体首地址
    struct app_sched_work_item *item = (struct app_sched_work_item *)work;

    if (item != RT_NULL)
    {
        if (item->handler != RT_NULL)
        {
            // 执行用户回调，数据地址紧跟在结构体后面
            item->handler((void *)item->data_storage, item->size);
        }
    }

    // 执行完毕，必须释放动态分配的内存
    rt_free(item);
}


void app_sched_event_put(void *event_data, uint16_t event_data_size, void *handler)
{
    LOG_I("app_sched_event_put %d", event_data_size);
    struct app_sched_work_item *item;

    if (handler == RT_NULL) return;

    // 1. 动态分配内存：结构体大小 + 数据大小
    // rt_malloc 返回的地址天然是 4 字节对齐的，解决 Hard Fault 核心问题
    item = (struct app_sched_work_item *)rt_malloc(sizeof(struct app_sched_work_item) + event_data_size);
    rt_size_t total_size = sizeof(struct app_sched_work_item) + event_data_size;

    if (item == RT_NULL)
    {
        rt_kprintf("app_sched: malloc failed!\n");
        return;
    }

    LOG_I("app_sched_event_put malloc 0x%x", item);
    // 2. 填充数据
    rt_memset(item, 0, total_size);
    item->handler = (app_sched_handler_t)handler;
    item->size    = event_data_size;

    if (event_data != RT_NULL && event_data_size > 0)
    {
        rt_memcpy(item->data_storage, event_data, event_data_size);
    }

    // 3. 初始化工作项并提交到系统工作队列
    rt_work_init(&(item->work), _app_sched_work_wrapper, RT_NULL);
    
    // 提交到系统工作队列 (System Workqueue)
    rt_err_t err = rt_work_submit(&(item->work), 0);
    
    if (err != RT_EOK)
    {
        rt_free(item); // 提交失败必须手动释放
    }
}
#endif


uint8_t app_timer_create(ot_timer_id_t timer_id, uint8_t flag, os_timer_func_t func)
{
    os_timer_create(timer_id, func, NULL, flag | RT_TIMER_FLAG_SOFT_TIMER);
    return 0;
}

uint8_t app_timer_stop(ot_timer_id_t timer_id)
{
    return os_timer_stop(timer_id);
}

uint8_t app_timer_start(ot_timer_id_t timer_id, uint32_t duration, uint8_t reserved)
{
    return os_timer_start(timer_id, duration);
}

int min(int a, int b) {
    return (a < b) ? a : b;
}