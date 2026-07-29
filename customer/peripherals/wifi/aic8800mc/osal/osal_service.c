/**
 * @attention
 * Copyright (c) 2018-2024 AICSemi Ltd. All rights reserved.
 * Copyright (c) 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "osal_service.h"
#include "osal_porting.h"
#include "osal_debug.h"
#include <string.h>

unsigned long rtos_now(bool isr)
{
    unsigned long tick = 0;
    if (osal_porting_ops.osal_now)
    {
        tick = osal_porting_ops.osal_now(isr);
    }
    else
    {
        DBG_OSAL_ERR("not implement ops: osal_now\r\n");
    }
    return tick;
}

#ifdef CONFIG_NOT_DEFINED
void rtos_msleep(uint32 time_in_ms)
{
    rt_thread_delay(rtos_ms_to_tick(time_in_ms));
}

void rtos_udelay(unsigned int us)
{
    hal_udelay(us);
}
#endif

void *rtos_malloc(uint32_t size)
{
    void *ptr = NULL;
    if (osal_porting_ops.osal_malloc)
    {
        ptr = osal_porting_ops.osal_malloc(size);
    }
    else
    {
        DBG_OSAL_ERR("not implement ops: osal_malloc\r\n");
    }
    return ptr;
}

void *rtos_calloc(uint32_t nb_elt, uint32_t size)
{
    void *ptr = NULL;
    if (osal_porting_ops.osal_calloc)
    {
        ptr = osal_porting_ops.osal_calloc(nb_elt, size);
    }
    else
    {
        DBG_OSAL_ERR("not implement ops: osal_calloc\r\n");
    }
    return ptr;
}

void rtos_free(void *ptr)
{
    if (osal_porting_ops.osal_free)
    {
        osal_porting_ops.osal_free(ptr);
    }
    else
    {
        DBG_OSAL_ERR("not implement ops: osal_free\r\n");
    }
}

void rtos_memcpy(void *pdest, const void *psrc, uint32_t size)
{
    if (osal_porting_ops.osal_memcpy)
    {
        osal_porting_ops.osal_memcpy(pdest, psrc, size);
    }
    else
    {
        DBG_OSAL_ERR("not implement ops: osal_memcpy\r\n");
    }
}

void rtos_memset(void *pdest, uint8_t byte, uint32_t size)
{
    if (osal_porting_ops.osal_memset)
    {
        osal_porting_ops.osal_memset(pdest, byte, size);
    }
    else
    {
        DBG_OSAL_ERR("not implement ops: osal_memset\r\n");
    }
}

//void rtos_heap_info(int *total_size, int *free_size, int *min_free_size)
//{
//
//}

void rtos_task_critical_enter(void)
{
    if (osal_porting_ops.osal_task_critical_enter)
    {
        osal_porting_ops.osal_task_critical_enter();
    }
    else
    {
        DBG_OSAL_ERR("not implement ops: osal_task_critical_enter\r\n");
    }
}

void rtos_task_critical_exit(void)
{
    if (osal_porting_ops.osal_task_critical_exit)
    {
        osal_porting_ops.osal_task_critical_exit();
    }
    else
    {
        DBG_OSAL_ERR("not implement ops: osal_task_critical_exit\r\n");
    }
}

int rtos_task_create(rtos_task_fct func,
                     const char *const name,
                     int task_id,
                     const uint16_t stack_depth,
                     void *const params,
                     rtos_prio prio,
                     rtos_task_handle *task_handle)
{
    int ret = -1;
    if (osal_porting_ops.osal_task_create)
    {
        ret = osal_porting_ops.osal_task_create(func, name, task_id, stack_depth, params, prio, task_handle);
    }
    else
    {
        DBG_OSAL_ERR("not implement ops: osal_task_create\r\n");
    }
    return ret;
}

void rtos_task_delete(rtos_task_handle task_handle)
{
    if (osal_porting_ops.osal_task_delete)
    {
        osal_porting_ops.osal_task_delete(task_handle);
    }
    else
    {
        DBG_OSAL_ERR("not implement ops: osal_task_delete\r\n");
    }
}

void rtos_task_suspend(int duration)
{
    if (osal_porting_ops.osal_task_suspend)
    {
        osal_porting_ops.osal_task_suspend(duration);
    }
    else
    {
        DBG_OSAL_ERR("not implement ops: osal_task_suspend\r\n");
    }
}

void rtos_task_resume(rtos_task_handle task_handle)
{
    if (osal_porting_ops.osal_task_resume)
    {
        osal_porting_ops.osal_task_resume(task_handle);
    }
    else
    {
        DBG_OSAL_ERR("not implement ops: osal_task_resume\r\n");
    }
}

rtos_task_handle rtos_get_task_handle(void)
{
    rtos_task_handle task_handle = NULL;
    if (osal_porting_ops.osal_get_task_handle)
    {
        task_handle = osal_porting_ops.osal_get_task_handle();
    }
    else
    {
        DBG_OSAL_ERR("not implement ops: osal_get_task_handle\r\n");
    }
    return task_handle;
}

//int rtos_task_init_notification(rtos_task_handle task)
//{
//    return 0;
//}
//
//uint32_t rtos_task_wait_notification(int timeout)
//{
//    return 0;
//}
//
//void rtos_task_notify(rtos_task_handle task_handle, uint32_t value, bool isr)
//{
//
//}
//
//void rtos_task_notify_setbits(rtos_task_handle task_handle, uint32_t value, bool isr)
//{
//
//}

#ifdef CONFIG_NOT_DEFINED
uint32_t rtos_task_get_priority(rtos_task_handle task_handle)
{
    rt_thread_t handle = (rt_thread_t)task_handle;

    return handle->current_priority;
}

void rtos_task_set_priority(rtos_task_handle task_handle, uint32_t priority)
{
    rt_thread_t handle = (rt_thread_t)task_handle;
    rt_uint8_t rt_priority = (rt_uint8_t)priority;

    rt_thread_control(handle, RT_THREAD_CTRL_CHANGE_PRIORITY, &rt_priority);
}
#endif

#ifdef CONFIG_NOT_DEFINED
int rtos_queue_create(int elt_size, int nb_elt, rtos_queue *queue, const char *const name)
{
    OsQueue *pqueue = NULL;

    if (queue == NULL)
    {
        return -1;
    }

    pqueue = rtos_malloc(sizeof(OsQueue));
    if (pqueue == NULL)
    {
        return -2;
    }

    pqueue->elt_size = elt_size;
    pqueue->nb_elt = nb_elt;
    pqueue->mq = rt_mq_create(name, elt_size, nb_elt, RT_IPC_FLAG_FIFO);
    if (pqueue->mq == RT_NULL)
    {
        AIC_LOG_PRINTF("create queue %s error\n", name);
        rtos_free(pqueue);
        return -1;
    }

#if RTOS_AL_INFO_DUMP
    AIC_LOG_PRINTF("%s '%s', inner_handle = %p, handle = %p, elt_size = %#x, nb_elt = %#x, inner msg_size = %#x\n",
                   __func__, name, pqueue->mq, pqueue, elt_size, nb_elt, pqueue->mq->msg_size);
#endif

    *queue = (rtos_queue)pqueue;

    return 0;
}

void rtos_queue_delete(rtos_queue queue)
{
#if QUEUE_TRACE
    aic_dbg("qdel:%p\n", queue);
#endif
    OsQueue *pqueue = (OsQueue *)queue;
    if (pqueue)
    {
#if RTOS_AL_INFO_DUMP
        AIC_LOG_PRINTF("%s, inner_handle = %p, handle = %p\n", __func__, pqueue->mq, pqueue);
#endif
        rt_mq_delete(pqueue->mq);
        rtos_free(pqueue);
    }
}

bool rtos_queue_is_empty(rtos_queue queue)
{
    OsQueue *pqueue = (OsQueue *)queue;
    return (pqueue->mq->msg_queue_head == RT_NULL);
}

bool rtos_queue_is_full(rtos_queue queue)
{
    OsQueue *pqueue = (OsQueue *)queue;
    return (pqueue->mq->msg_queue_free == RT_NULL);
}

int rtos_queue_cnt(rtos_queue queue)
{
    return 0;
}

int rtos_queue_write(rtos_queue queue, void *msg, int timeout, bool isr)
{
#if QUEUE_TRACE
    aic_dbg("wqin:%p/%d\n", queue, timeout);
#endif
    rt_err_t status;
    int ret = 0;
    OsQueue *pqueue = (OsQueue *)queue;

    if (pqueue == NULL)
    {
        ret = -1;
        goto exit;
    }
    status = rt_mq_send_wait(pqueue->mq, msg, pqueue->elt_size, rt_tick_from_millisecond(timeout));
    if (status != RT_EOK)
    {
        ret = -2;
        goto exit;
    }

exit:
#if QUEUE_TRACE
    aic_dbg("wqout:%p/%d\n", queue, ret);
#endif
    return ret;
}

int rtos_queue_read(rtos_queue queue, void *msg, int timeout, bool isr)
{
#if QUEUE_TRACE
    aic_dbg("rqin:%p/%d\n", queue, timeout);
#endif
    rt_err_t status;
    int ret = 0;
    OsQueue *pqueue = (OsQueue *)queue;

    if (pqueue == NULL)
    {
        ret = -1;
        goto exit;
    }

    status = rt_mq_recv(pqueue->mq, msg, pqueue->elt_size, rt_tick_from_millisecond(timeout));
    if (status != RT_EOK)
    {
        ret = -2;
        goto exit;
    }

exit:
#if QUEUE_TRACE
    aic_dbg("rqout:%p/%d\n", queue, ret);
#endif
    return ret;
}
#endif

//int rtos_queue_peek(rtos_queue queue, void *msg, int timeout, bool isr)
//{
//    return 0;
//}
//
//int rtos_queue_reset(rtos_queue queue)
//{
//    return 0;
//}
//
//int rtos_signal_send(rtos_task_handle dest_MsgQ, void *MsgItem, int msgLen, bool isr)
//{
//    return 0;
//}
//
//int rtos_signal_recv(rtos_task_handle MsgQ, void *MsgItem, int msgLen, bool isr)
//{
//    return 0;
//}

int rtos_semaphore_create(rtos_semaphore *semaphore, const char *const name, int max_count, int init_count)
{
    int ret = -1;
    if (osal_porting_ops.osal_semaphore_create)
    {
        ret = osal_porting_ops.osal_semaphore_create(semaphore, name, max_count, init_count);
    }
    else
    {
        DBG_OSAL_ERR("not implement ops: osal_semaphore_create\r\n");
    }
    return ret;
}

void rtos_semaphore_delete(rtos_semaphore semaphore)
{
    if (osal_porting_ops.osal_semaphore_delete)
    {
        osal_porting_ops.osal_semaphore_delete(semaphore);
    }
    else
    {
        DBG_OSAL_ERR("not implement ops: osal_semaphore_delete\r\n");
    }
}

int rtos_semaphore_get_count(rtos_semaphore semaphore)
{
    int ret = -1;
    if (osal_porting_ops.osal_semaphore_get_count)
    {
        ret = osal_porting_ops.osal_semaphore_get_count(semaphore);
    }
    else
    {
        DBG_OSAL_ERR("not implement ops: osal_semaphore_get_count\r\n");
    }
    return ret;
}

int rtos_semaphore_wait(rtos_semaphore semaphore, int timeout)
{
    int ret = -1;
    if (osal_porting_ops.osal_semaphore_wait)
    {
        ret = osal_porting_ops.osal_semaphore_wait(semaphore, timeout);
    }
    else
    {
        DBG_OSAL_ERR("not implement ops: osal_semaphore_wait\r\n");
    }
    return ret;
}

int rtos_semaphore_signal(rtos_semaphore semaphore, bool isr)
{
    int ret = -1;
    if (osal_porting_ops.osal_semaphore_signal)
    {
        ret = osal_porting_ops.osal_semaphore_signal(semaphore, isr);
    }
    else
    {
        DBG_OSAL_ERR("not implement ops: osal_semaphore_signal\r\n");
    }
    return ret;
}


#ifdef CONFIG_NOT_DEFINED
uint32_t rtos_timer_create(const char *const name,
                           rtos_timer *timer,
                           const uint32_t ms,
                           const uint8_t autoReload,
                           void *const args,
                           rtos_timer_fct func)
{
    OsTimer *ptimer = NULL;
    rt_uint8_t flag;

    if (timer == NULL)
    {
        return -1;
    }

    flag = RT_TIMER_FLAG_SOFT_TIMER;
    if (autoReload)
    {
        flag |= RT_TIMER_FLAG_PERIODIC;
    }
    else
    {
        flag |= RT_TIMER_FLAG_ONE_SHOT;
    }

    ptimer = (OsTimer *)rtos_malloc(sizeof(OsTimer));
    if (ptimer == NULL)
    {
        return -2;
    }
    memset(ptimer, 0, sizeof(OsTimer));

    ptimer->ms = ms;
    ptimer->autoReload = autoReload;
    ptimer->func = func;
    ptimer->args = args;
    ptimer->handle = rt_timer_create(name,
                                     func,
                                     args,
                                     ms,
                                     flag);
#if RTOS_AL_INFO_DUMP
    AIC_LOG_PRINTF("%s '%s', inner_handle = %p, handle = %p, period = %d, reload = %d\n",
                   __func__, name, ptimer->handle, ptimer, ms, autoReload);
#endif

    if (ptimer->handle == NULL)
    {
        rtos_free(ptimer);
        return -3;
    }

    *timer = (rtos_timer)ptimer;
    return 0;
}

int rtos_timer_start(rtos_timer timer, uint32_t ms, bool isr)
{
    OsTimer *ptimer = (OsTimer *)timer;

    rt_timer_start(ptimer->handle);

    return 0;
}

int rtos_timer_stop(rtos_timer timer, uint32_t wait_ms)
{
    OsTimer *ptimer = (OsTimer *)timer;

    rt_timer_stop(ptimer->handle);

    return 0;
}

int rtos_timer_stop_isr(rtos_timer timer)
{
    OsTimer *ptimer = (OsTimer *)timer;

    rt_timer_stop(ptimer->handle);

    return 0;
}

int rtos_timer_delete(rtos_timer timer, uint32_t wait_ms)
{
    OsTimer *ptimer = (OsTimer *)timer;

    rt_timer_delete(ptimer->handle);
    rtos_free(ptimer);

    return 0;
}
#endif

//int rtos_mutex_recursive_create(rtos_mutex *mutex)
//{
//    return -1;
//}
//
//int rtos_mutex_recursive_lock(rtos_mutex mutex)
//{
//    return -1;
//}
//
//int rtos_mutex_recursive_unlock(rtos_mutex mutex)
//{
//    return -1;
//}

int rtos_mutex_create(rtos_mutex *mutex, const char *const name)
{
    int ret = -1;
    if (osal_porting_ops.osal_mutex_create)
    {
        ret = osal_porting_ops.osal_mutex_create(mutex, name);
    }
    else
    {
        DBG_OSAL_ERR("not implement ops: osal_mutex_create\r\n");
    }
    return ret;
}

void rtos_mutex_delete(rtos_mutex mutex)
{
    if (osal_porting_ops.osal_mutex_delete)
    {
        osal_porting_ops.osal_mutex_delete(mutex);
    }
    else
    {
        DBG_OSAL_ERR("not implement ops: osal_mutex_delete\r\n");
    }
}

int rtos_mutex_lock(rtos_mutex mutex, int timeout)
{
    int ret = -1;
    if (osal_porting_ops.osal_mutex_lock)
    {
        ret = osal_porting_ops.osal_mutex_lock(mutex, timeout);
    }
    else
    {
        DBG_OSAL_ERR("not implement ops: osal_mutex_lock\r\n");
    }
    return ret;
}

int rtos_mutex_unlock(rtos_mutex mutex)
{
    int ret = -1;
    if (osal_porting_ops.osal_mutex_unlock)
    {
        ret = osal_porting_ops.osal_mutex_unlock(mutex);
    }
    else
    {
        DBG_OSAL_ERR("not implement ops: osal_mutex_unlock\r\n");
    }
    return ret;
}

void rtos_atomic_set(rtos_atomic_t *atom, int val)
{
    if (osal_porting_ops.osal_atomic_set)
    {
        osal_porting_ops.osal_atomic_set(atom, val);
    }
    else
    {
        DBG_OSAL_ERR("not implement ops: osal_atomic_set\r\n");
    }
}

int rtos_atomic_read(rtos_atomic_t *atom)
{
    int ret = -1;
    if (osal_porting_ops.osal_atomic_read)
    {
        ret = osal_porting_ops.osal_atomic_read(atom);
    }
    else
    {
        DBG_OSAL_ERR("not implement ops: osal_atomic_set\r\n");
    }
    return ret;
}

void rtos_atomic_add(rtos_atomic_t *atom, int val)
{
    if (osal_porting_ops.osal_atomic_add)
    {
        osal_porting_ops.osal_atomic_add(atom, val);
    }
    else
    {
        DBG_OSAL_ERR("not implement ops: osal_atomic_add\r\n");
    }
}

void rtos_atomic_sub(rtos_atomic_t *atom, int val)
{
    if (osal_porting_ops.osal_atomic_sub)
    {
        osal_porting_ops.osal_atomic_sub(atom, val);
    }
    else
    {
        DBG_OSAL_ERR("not implement ops: osal_atomic_sub\r\n");
    }
}

void rtos_atomic_inc(rtos_atomic_t *atom)
{
    if (osal_porting_ops.osal_atomic_inc)
    {
        osal_porting_ops.osal_atomic_inc(atom);
    }
    else
    {
        DBG_OSAL_ERR("not implement ops: osal_atomic_inc\r\n");
    }
}

void rtos_atomic_dec(rtos_atomic_t *atom)
{
    if (osal_porting_ops.osal_atomic_dec)
    {
        osal_porting_ops.osal_atomic_dec(atom);
    }
    else
    {
        DBG_OSAL_ERR("not implement ops: osal_atomic_dec\r\n");
    }
}

int rtos_atomic_add_ret(rtos_atomic_t *atom, int val)
{
    int ret = -1;
    if (osal_porting_ops.osal_atomic_add_ret)
    {
        ret = osal_porting_ops.osal_atomic_add_ret(atom, val);
    }
    else
    {
        DBG_OSAL_ERR("not implement ops: osal_atomic_add_ret\r\n");
    }
    return ret;
}

int rtos_atomic_sub_ret(rtos_atomic_t *atom, int val)
{
    int ret = -1;
    if (osal_porting_ops.osal_atomic_sub_ret)
    {
        ret = osal_porting_ops.osal_atomic_sub_ret(atom, val);
    }
    else
    {
        DBG_OSAL_ERR("not implement ops: osal_atomic_sub_ret\r\n");
    }
    return ret;
}

int rtos_atomic_inc_ret(rtos_atomic_t *atom)
{
    int ret = -1;
    if (osal_porting_ops.osal_atomic_inc_ret)
    {
        ret = osal_porting_ops.osal_atomic_inc_ret(atom);
    }
    else
    {
        DBG_OSAL_ERR("not implement ops: osal_atomic_inc_ret\r\n");
    }
    return ret;
}

int rtos_atomic_dec_ret(rtos_atomic_t *atom)
{
    int ret = -1;
    if (osal_porting_ops.osal_atomic_dec_ret)
    {
        ret = osal_porting_ops.osal_atomic_dec_ret(atom);
    }
    else
    {
        DBG_OSAL_ERR("not implement ops: osal_atomic_dec_ret\r\n");
    }
    return ret;
}

void rtos_dcache_clean(void *adr, unsigned int len)
{
    if (osal_porting_ops.osal_dcache_clean)
    {
        osal_porting_ops.osal_dcache_clean(adr, len);
    }
    else
    {
        DBG_OSAL_ERR("not implement ops: osal_dcache_clean\r\n");
    }
}

void rtos_dcache_invalidate(void *adr, unsigned int len)
{
    if (osal_porting_ops.osal_dcache_invalidate)
    {
        osal_porting_ops.osal_dcache_invalidate(adr, len);
    }
    else
    {
        DBG_OSAL_ERR("not implement ops: osal_dcache_invalidate\r\n");
    }
}

