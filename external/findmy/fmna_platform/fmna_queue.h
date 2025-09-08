/*
 * SPDX-FileCopyrightText: 2026-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef FMNA_QUEUE_H
#define FMNA_QUEUE_H

#include <rtthread.h>
#include <rtdef.h>

/* 队列结构体 */
typedef struct
{
    rt_uint8_t *buffer;      // 数据缓冲区
    rt_uint16_t element_size; // 单个元素大小
    rt_uint16_t capacity;     // 队列容量
    volatile rt_uint16_t head; // 头指针（写位置）
    volatile rt_uint16_t tail; // 尾指针（读位置）
    rt_mutex_t lock;         // 互斥锁
} fmna_queue_t;

/* 队列操作结果 */
#define RT_QUEUE_OK      0
#define RT_QUEUE_FULL   -1
#define RT_QUEUE_EMPTY  -2

/* API声明 */
int fmna_queue_init(fmna_queue_t *q, rt_uint16_t element_size, rt_uint16_t capacity);
void fmna_queue_deinit(fmna_queue_t *q);
int fmna_queue_push(fmna_queue_t *q, const void *data);
int fmna_queue_pop(fmna_queue_t *q, void *data);
int fmna_queue_pop(fmna_queue_t *q, void *data);
void fmna_queue_reset(fmna_queue_t *q);
rt_uint16_t fmna_queue_count(fmna_queue_t *q);

#endif