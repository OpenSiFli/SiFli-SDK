/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-08-01     Meco Man     remove rt_delayed_work_init() and rt_delayed_work structure
 * 2021-08-14     Jackistang   add comments for rt_work_init()
 */
#ifndef WORKQUEUE_H__
#define WORKQUEUE_H__

#include <rtdef.h>
#include <rtconfig.h>
#include "completion.h"

#ifdef __cplusplus
extern "C" {
#endif

enum
{
    RT_WORK_STATE_PENDING    = 0x0001,     /* Work item pending state */
    RT_WORK_STATE_SUBMITTING = 0x0002,     /* Work item submitting state */
    RT_WORK_STATE_RUNNING    = 0x0004,     /* Work item in running state */
};

/**
 * work type definitions
 */
enum
{
    RT_WORK_TYPE_DELAYED     = 0x0001,
};

/**
 * Spinlock
 */
#ifdef RT_USING_SMP
#include <cpuport.h> /* for spinlock from arch */

struct rt_spinlock
{
    rt_hw_spinlock_t lock;
#ifdef RT_USING_DEBUG
    rt_uint32_t critical_level;
#endif /* RT_USING_DEBUG */
#if defined(RT_DEBUGING_SPINLOCK)
    void *owner;
    void *pc;
#endif /* RT_DEBUGING_SPINLOCK */
};

#ifndef RT_SPINLOCK_INIT
#define RT_SPINLOCK_INIT {{0}} /* can be overridden by cpuport.h */
#endif /* RT_SPINLOCK_INIT */

#else /* !RT_USING_SMP */

struct rt_spinlock
{
#ifdef RT_USING_DEBUG
    rt_uint32_t critical_level;
#endif /* RT_USING_DEBUG */
    rt_ubase_t lock;
};
#define RT_SPINLOCK_INIT {0}
#endif /* RT_USING_SMP */

/* workqueue implementation */
struct rt_workqueue
{
    rt_list_t      work_list;
    rt_list_t      delayed_list;
    struct rt_work *work_current; /* current work */

    struct rt_semaphore sem;
    rt_thread_t    work_thread;
    struct rt_spinlock spinlock;
    struct rt_completion wakeup_completion;
};

struct rt_work
{
    rt_list_t list;

    void (*work_func)(struct rt_work *work, void *work_data);
    void *work_data;
    rt_uint16_t flags;
    rt_uint16_t type;
    rt_tick_t timeout_tick;
    struct rt_workqueue *workqueue;
};

/* kept for compatibility */
struct rt_delayed_work
{
    struct rt_work work;
};

#ifdef RT_USING_HEAP
/**
 * WorkQueue for DeviceDriver
 */
void rt_work_init(struct rt_work *work, void (*work_func)(struct rt_work *work, void *work_data), void *work_data);
struct rt_workqueue *rt_workqueue_init(struct rt_workqueue *queue);
struct rt_workqueue *rt_workqueue_start(struct rt_workqueue *queue, const char *name, void *stack_start, rt_uint16_t stack_size, rt_uint8_t priority);
struct rt_workqueue *rt_workqueue_create(const char *name, rt_uint16_t stack_size, rt_uint8_t priority);
rt_err_t rt_workqueue_destroy(struct rt_workqueue *queue);
rt_err_t rt_workqueue_dowork(struct rt_workqueue *queue, struct rt_work *work);
rt_err_t rt_workqueue_submit_work(struct rt_workqueue *queue, struct rt_work *work, rt_tick_t ticks);
rt_err_t rt_workqueue_cancel_work(struct rt_workqueue *queue, struct rt_work *work);
rt_err_t rt_workqueue_cancel_work_sync(struct rt_workqueue *queue, struct rt_work *work);
rt_err_t rt_workqueue_cancel_all_work(struct rt_workqueue *queue);
rt_err_t rt_workqueue_urgent_work(struct rt_workqueue *queue, struct rt_work *work);

/* API mapping for compatibility */
rt_inline void rt_delayed_work_init(struct rt_delayed_work *delayed_work, void (*work_func)(struct rt_work *work, void *work_data),
                                    void *work_data)
{
    rt_work_init(&delayed_work->work, work_func, work_data);
}

#ifdef RT_USING_SYSTEM_WORKQUEUE
struct rt_workqueue *rt_workqueue_sysq(void);
rt_err_t rt_work_submit(struct rt_work *work, rt_tick_t ticks);
rt_err_t rt_work_urgent(struct rt_work *work);
rt_err_t rt_work_cancel(struct rt_work *work);
rt_err_t rt_work_cancel_sync(struct rt_work *work);
#endif /* RT_USING_SYSTEM_WORKQUEUE */

#endif /* RT_USING_HEAP */

#ifdef __cplusplus
}
#endif

#endif
