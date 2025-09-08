/*
 * SPDX-FileCopyrightText: 2026-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "fmna_queue.h"
#include <string.h>

/**
 * 初始化队列
 * @param q             队列句柄
 * @param element_size  单个元素大小（字节）
 * @param capacity      队列容量（元素数量）
 * @return 成功返回RT_QUEUE_OK，失败返回-RT_ENOMEM
 */
int fmna_queue_init(fmna_queue_t *q, rt_uint16_t element_size, rt_uint16_t capacity)
{
    RT_ASSERT(q != RT_NULL);

    // 分配缓冲区内存
    q->buffer = (rt_uint8_t *)rt_malloc(element_size * capacity);
    if (!q->buffer) return -RT_ENOMEM;

    // 初始化成员
    q->element_size = element_size;
    q->capacity = capacity;
    q->head = 0;
    q->tail = 0;

    // 创建互斥锁
    q->lock = rt_mutex_create("queue_lock", RT_IPC_FLAG_PRIO);
    if (!q->lock)
    {
        rt_free(q->buffer);
        return -RT_ENOMEM;
    }

    return RT_QUEUE_OK;
}

/**
 * 销毁队列
 * @param q 队列句柄
 */
void fmna_queue_deinit(fmna_queue_t *q)
{
    if (q->buffer)
    {
        rt_free(q->buffer);
        q->buffer = RT_NULL;
    }
    if (q->lock)
    {
        rt_mutex_delete(q->lock);
    }
}

/**
 * 数据入队
 * @param q    队列句柄
 * @param data 待入队数据指针
 * @return 成功返回RT_QUEUE_OK，队列满返回RT_QUEUE_FULL
 */
int fmna_queue_push(fmna_queue_t *q, const void *data)
{
    RT_ASSERT(q && data);

    rt_mutex_take(q->lock, RT_WAITING_FOREVER); // 加锁

    // 检查队列是否满
    if (((q->head + 1) % q->capacity) == q->tail)
    {
        rt_mutex_release(q->lock);
        return RT_QUEUE_FULL;
    }

    // 复制数据到缓冲区
    rt_uint16_t pos = q->head * q->element_size;
    memcpy(&q->buffer[pos], data, q->element_size);

    // 更新头指针
    q->head = (q->head + 1) % q->capacity;

    rt_mutex_release(q->lock); // 解锁
    return RT_QUEUE_OK;
}

/**
 * 数据出队
 * @param q    队列句柄
 * @param data 接收数据的缓冲区指针
 * @return 成功返回RT_QUEUE_OK，队列空返回RT_QUEUE_EMPTY
 */
int fmna_queue_pop(fmna_queue_t *q, void *data)
{
    RT_ASSERT(q && data);

    rt_mutex_take(q->lock, RT_WAITING_FOREVER); // 加锁

    // 检查队列是否空
    if (q->tail == q->head)
    {
        rt_mutex_release(q->lock);
        return RT_QUEUE_EMPTY;
    }

    // 从缓冲区复制数据
    rt_uint16_t pos = q->tail * q->element_size;
    memcpy(data, &q->buffer[pos], q->element_size);

    // 更新尾指针
    q->tail = (q->tail + 1) % q->capacity;

    rt_mutex_release(q->lock); // 解锁
    return RT_QUEUE_OK;
}

void fmna_queue_reset(fmna_queue_t *q)
{
    RT_ASSERT(q != RT_NULL);

    rt_mutex_take(q->lock, RT_WAITING_FOREVER); // 加锁
    q->head = 0;  // 重置头指针
    q->tail = 0;  // 重置尾指针
    rt_mutex_release(q->lock); // 解锁
}

/**
 * 获取队列中元素数量
 * @param q 队列句柄
 * @return 当前元素数量
 */
rt_uint16_t fmna_queue_count(fmna_queue_t *q)
{
    rt_uint16_t count;
    rt_mutex_take(q->lock, RT_WAITING_FOREVER);
    count = (q->head >= q->tail) ?
            (q->head - q->tail) :
            (q->capacity - q->tail + q->head);
    rt_mutex_release(q->lock);
    return count;
}