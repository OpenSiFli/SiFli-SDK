/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Copyright (C) 2018-2020 AICSemi Ltd.
 *
 * All Rights Reserved
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rtos_port.h"
#include "clock_time.h"
#include "dbg_assert.h"
#include "aic_plat_log.h"
#include "aic_plat_time.h"

#if defined(CONFIG_PLAT_RTTHREAD)
    #include <rtdbg.h>
    #define AIC_PLAT_PRINTF_API         rt_kprintf
#endif

#ifndef rt_mq_send_wait
    #define rt_mq_send_wait(a, b, c, d) rt_mq_send(a, (void *)b, c)
#endif

#define RTOS_AL_INFO_DUMP   0
#define RTOS_INFO_SHORT     0

static uint32_t rtos_mem_cnt = 0;
static uint32_t rtos_free_cnt = 0;
static uint32_t rtos_remain_cnt = 0;
struct co_list res_list[RES_TYPE_MAX] = {NULL, NULL};

static rtos_mutex rtos_mutexRef = NULL;

#ifdef CONFIG_RX_NOCOPY
    #define AIC_RTOS_MEM_MAX        ((1024)*1024)
#else
    #define AIC_RTOS_MEM_MAX        ((1024+256)*1024)
#endif
const osal_porting_ops_t osal_porting_ops =
{
    .osal_dbg_printf    = AIC_PLAT_PRINTF_API,
};

/*
 * FUNCTIONS
 ****************************************************************************************
 */
/**
 ****************************************************************************************
 * @brief Convert ms to ticks
 *
 * @param[in] timeout_ms Timeout value in ms (use -1 for no timeout).
 * @return number of ticks for the specified timeout value.
 *
 ****************************************************************************************
 */
void rtos_remove_all(void)
{
    AIC_LOG_PRINTF("[rtos]%s not support!!!\n", __func__);
}

void rtos_res_list_info_show(void)
{
    AIC_LOG_PRINTF("[rtos]%s not support!!!\n", __func__);
}

__STATIC_INLINE uint32_t rtos_ms_to_tick(unsigned long ms)
{
    return ms / (1000 / RT_TICK_PER_SECOND);
}

__STATIC_INLINE unsigned long rtos_tick_to_ms(uint32_t tick)
{
    return tick * (1000 / RT_TICK_PER_SECOND);
}

unsigned long rtos_now(bool isr)
{
    rtos_tick_t tick = rt_tick_get();

    return rtos_tick_to_ms(tick);
}

void rtos_msleep(uint32_t time_in_ms)
{
    rtos_tick_t ticks = rtos_ms_to_tick(time_in_ms);

    if (ticks == 0)
        ticks = 1;

    rt_thread_delay(ticks);
}

void rt_hw_us_delay(rt_uint32_t us);
void rtos_udelay(unsigned int us)
{
    rt_hw_us_delay(us);
}

void rtos_mem_free_task(void *taskref)
{
    AIC_LOG_PRINTF("[rtos]%s not support!!! task=%p\n", __func__, taskref);
}
void *rtos_malloc(uint32_t size)
{
    return rt_malloc((rt_size_t)size);
}

void *rtos_calloc(uint32_t nb_elt, uint32_t size)
{
    return rt_calloc((rt_size_t)nb_elt, (rt_size_t)size);
}

void rtos_free(void *ptr)
{
    rt_free(ptr);
}

void rtos_mem_cur_cnt(uint32_t *mtot, uint32_t *ftot, uint32_t *rtot)
{
    *mtot = rtos_mem_cnt;
    *ftot = rtos_free_cnt;
    *rtot = rtos_remain_cnt;
}


void rtos_memcpy(void *pdest, const void *psrc, uint32_t size)
{
    memcpy(pdest, psrc, size);
}

void rtos_memset(void *pdest, uint8_t byte, uint32_t size)
{
    memset(pdest, byte, size);
}

#if 0
void rtos_net_mempush(void *pbuf, uint32_t size, void *pmsg, uint16_t type)
{
    OsResSt st;

    if (pbuf)
    {
        OsResInit(&st, NULL, pmsg);
        st.para0 = size;
        st.ref = pmsg;
        st.para1 = (uint32_t)pbuf;
        st.type = type;
        rtos_res_list_push(&st);

#if RTOS_AL_INFO_DUMP
        //AIC_LOG_PRINTF("[rtos]type[%d] name:%s,ref:%#x para0:%#x,para1:%#x\n",st.type,st.name,st.ref,st.para0,st.para1);
#endif
    }

}

void rtos_net_mempop(void *pmsg, uint16_t type)
{
    rtos_res_list_rellist(pmsg, type);
}

void rtos_net_mem_free(void *pmsg, uint16_t type)
{
    rtos_res_list_release(pmsg, type);
}

void rtos_socket_push(int sock)
{
    OsResSt st;
    char name[10];

    if (sock != -1)
    {
        sprintf(name, "sock%d", sock);
        OsResInit(&st, name, NULL);
        st.para0 = sock;
        st.type = RES_TYPE_SOCKET;
        rtos_res_list_push(&st);
        AIC_LOG_PRINTF("[rtos]type[%d] name:%s, para0:%#x\n", st.type, st.name, st.para0);
    }

}

void rtos_socket_pop(int sock)
{
    uint16_t rsock = (uint16_t)sock << 8;
    rsock &= 0xff00;

    rtos_res_list_rellist(NULL, rsock | RES_TYPE_SOCKET);
}
#endif

uint32_t rtos_get_task_status(rtos_task_handle ref)
{
    uint8_t st = ref->stat;


    AIC_LOG_PRINTF("[rtos]%s :, handle = %p st:%d\n", __func__, ref, st);

    return st;
}

uint32_t rtos_wait_task_suspend(rtos_task_handle ref)
{
    uint32_t st;

    while (1)
    {
        st = rtos_get_task_status(ref);

        if (((st == AIC_RTOS_READY) || (st == AIC_RTOS_MUTEX_SUSP)))
            rtos_msleep(1);
        else
            break;
    };

    if ((st != AIC_RTOS_COMPLETED) && (st != AIC_RTOS_TERMINATED))
        rt_thread_suspend(ref);

    return 0;//OSATaskGetStatus(ref);
}

uint32_t rtos_wait_task_suspend_only(rtos_task_handle ref)
{
    uint32_t st;

    while (1)
    {
        st = rtos_get_task_status(ref);

        if (((st == AIC_RTOS_READY) || (st == AIC_RTOS_MUTEX_SUSP)))
            rtos_msleep(1);
        else
            break;
    };

    return 0;
}

void rtos_wait_all_task_suspend(void)
{
#if 1
    AIC_LOG_PRINTF("[rtos]%s not support!!\n", __func__);
#else
    OsResList *pRes;
    struct co_list *plist;

    rtos_res_mutex_lock();
    plist = &res_list[RES_TYPE_TASK];
    pRes = (OsResList *)co_list_pick(plist);
    rtos_res_mutex_unlock();

    while (pRes)
    {
        if (pRes->res.ref)
        {
            rtos_wait_task_suspend(pRes->res.ref);
        }

        pRes = (OsResList *)co_list_next(&(pRes->hdr));
    }
#endif

}

rtos_task_handle rtos_get_current_task(void)
{
    return (rtos_task_handle)rt_thread_self();
}

char *rtos_get_task_name(void *ref)
{
    rtos_task_handle th = (rtos_task_handle)ref;
#if (RTTHREAD_VERSION >= RT_VERSION_CHECK(5, 0, 2))
    return th->parent.name;
#else
    return th->name;
#endif
}

int rtos_task_create(rtos_task_fct func,
                     const char *const name,
                     int task_id,
                     const uint16_t stack_depth,
                     void *const params,
                     rtos_prio prio,
                     rtos_task_handle *task_handle)
{
    rtos_task_handle th;
    OsResSt st;

    AIC_ASSERT_ERR(task_handle != NULL);

    if (prio < 1) prio = 1;
    if (prio > (RT_THREAD_PRIORITY_MAX - 1)) prio = RT_THREAD_PRIORITY_MAX - 1;

    th = rt_thread_create(name, func, params, stack_depth,
                          prio, AIC_WIFI_TASK_STICK);
    if (th == NULL)
    {
        return -3;
    }

    *task_handle = th;
#if 0
    OsResInit(&st, (char *)name, *task_handle);
    st.para0 = (uint32_t)stack_base;
    st.para1 = stack_depth;
    st.type = RES_TYPE_TASK;
#endif

#if RTOS_AL_INFO_DUMP
    AIC_LOG_PRINTF("[rtos]%s '%s', handle = %#x, stack_base = %#x, stack_size = %#x\n", __func__, name, *task_handle, stack_base, stack_depth);
#endif

    //rtos_res_list_push(&st);

    rt_thread_startup(th);

    return 0;
}

int rtos_task_create_only(rtos_task_fct func,
                          const char *const name,
                          int task_id,
                          const uint16_t stack_depth,
                          void *const params,
                          rtos_prio prio,
                          rtos_task_handle *task_handle)
{
    rtos_task_handle th;

    AIC_ASSERT_ERR(task_handle != NULL);

    th = rt_thread_create(name, func, params, stack_depth,
                          prio, AIC_WIFI_TASK_STICK);
    if (th = NULL)
    {
        return -3;
    }

    *task_handle = th;
#if RTOS_AL_INFO_DUMP
    AIC_LOG_PRINTF("[rtos]%s '%s', handle = %#x, stack_base = %#x, stack_size = %#x\n", __func__, name, *task_handle, stack_base, stack_depth);
#endif

    rt_thread_startup(th);

    return 0;
}

void rtos_task_delete(rtos_task_handle task_handle)
{
    rt_thread_t handle = (rt_thread_t)task_handle;
    rt_err_t ret = RT_EOK;
    if (!handle)
    {
        rt_thread_exit();
        return;
    }

    ret = rt_thread_delete(handle);
    if (ret)
    {
        AIC_LOG_PRINTF("thread del fail, ret=%d\n", ret);
    }
}

bool rtos_task_isvalid(rtos_task_handle task_handle)
{
    return true;
}

void rtos_task_suspend(int duration)
{
    rtos_task_handle taskref;

    taskref = rt_thread_self();

    if (-1 == duration)
    {
        rt_thread_suspend(taskref);
    }
    else
    {
        rt_thread_delay(rtos_ms_to_tick(duration));
    }
}

void rtos_task_resume(rtos_task_handle task_handle)
{
    rt_thread_resume(task_handle);
}

uint32_t rtos_task_get_priority(rtos_task_handle task_handle)
{
    uint8_t priority = 0;

    return priority;
}

void rtos_task_set_priority(rtos_task_handle task_handle, uint32_t priority)
{
    //bool task_found = false;
    //UINT8 old = 0;

}

int rtos_queue_create(int elt_size, int nb_elt, rtos_queue *queue, const char *const name)
{

    AIC_ASSERT_ERR(queue != NULL);

    *queue = rt_mq_create((char *)name,
                          elt_size,
                          nb_elt,
                          RT_IPC_FLAG_FIFO);

    if (*queue == NULL)
    {
        AIC_LOG_PRINTF("[rtos]create queue %s error\n", name);
        return -2;
    }

#if 0
    OsResInit(&st, (char *)name, *queue);
    st.type = RES_TYPE_Q;
    st.para0 = nb_elt;
    st.para1 = elt_size;
#endif

#if RTOS_AL_INFO_DUMP
    AIC_LOG_PRINTF("[rtos]%s '%s', queue = %#x\n", __func__, name, *queue);
#endif

    //rtos_res_list_push(&st);

    return 0;
}

void rtos_queue_delete(rtos_queue queue)
{
    rt_err_t err;
    rt_mq_t q = (rt_mq_t)queue;
    err = rt_mq_delete(q);
    if (err)
    {
        AIC_LOG_PRINTF("[rtos]delete queue error: %d\n", err);
    }
}

bool rtos_queue_is_empty(rtos_queue queue)
{
    rt_mq_t q = (rt_mq_t)queue;
    return (q->msg_queue_head == RT_NULL);
}

bool rtos_queue_is_full(rtos_queue queue)
{
    rt_mq_t q = (rt_mq_t)queue;
    return (q->msg_queue_free == RT_NULL);
}

int rtos_queue_cnt(rtos_queue queue)
{
    rt_mq_t q = (rt_mq_t)queue;
    return (int)q->entry;
}

int rtos_queue_write(rtos_queue queue, void *msg, int timeout, bool isr)
{
    rtos_err osaStatus;
    int ret = 0;
    rt_mq_t rt_mq;

    if (queue == NULL)
    {
        return -1;
    }

    rt_mq = (rt_mq_t)queue;

    osaStatus = rt_mq_send_wait(queue, msg, rt_mq->msg_size, rtos_ms_to_tick(timeout));

    if (osaStatus != AIC_RTOS_SUCCESS)
    {
        ret = -2;
    }

    return ret;
}

int rtos_queue_read(rtos_queue queue, void *msg, int timeout, bool isr)
{
    rtos_err osaStatus;
    int ret = 0;
    rt_mq_t rt_mq;

    if (queue == NULL)
    {
        return -1;
    }

    rt_mq = (rt_mq_t)queue;

    osaStatus = rt_mq_recv(queue, msg, rt_mq->msg_size, rtos_ms_to_tick(timeout));

    if (osaStatus == AIC_OS_TIMEOUT)
    {
        ret = AIC_RTOS_WAIT_ERROR;
    }
    else if (osaStatus != AIC_RTOS_SUCCESS)
    {
        ret = AIC_RTOS_ERR;
    }

    return ret;
}

int rtos_semaphore_create(rtos_semaphore *semaphore, const char *const name, int max_count, int init_count)
{
    rt_sem_t psem;
    int ret = 0;

    if (semaphore == NULL)
    {
        ret = -1;
    }
    psem =  rt_sem_create(name, init_count, RT_IPC_FLAG_FIFO);
    if (psem == NULL)
    {
        ret = -2;
    }
    *semaphore = (rtos_semaphore)psem;

    return ret;
}

int rtos_semaphore_create_only(rtos_semaphore *semaphore, const char *const name, int max_count, int init_count)
{
    int ret = 0;

    *semaphore =  rt_sem_create(name, init_count, RT_IPC_FLAG_FIFO);

    if (*semaphore != NULL)
    {
        //tbd
    }
    else
    {
        ret = AIC_RTOS_ERR;
    }

    return ret;
}

void rtos_semaphore_delete(rtos_semaphore semaphore)
{
    rt_sem_t psem = (rt_sem_t)semaphore;
    if (psem)
    {
        rt_sem_delete(psem);
    }
}

int rtos_semaphore_get_count(rtos_semaphore semaphore)
{
    uint32_t count = 0;

    count = semaphore->value;

    return (int)count;
}

int rtos_semaphore_wait(rtos_semaphore semaphore, int timeout)
{
    rtos_err osaStatus;
    int ret;

    osaStatus = rt_sem_take(semaphore, rtos_ms_to_tick(timeout));

    if (osaStatus == 0)
        ret = 0;
    else if (osaStatus == AIC_OS_TIMEOUT)
        ret = 1;
    else
        ret = -1;

    return ret;
}

int rtos_semaphore_signal(rtos_semaphore semaphore, bool isr)
{
    if (semaphore == NULL)
    {
        aic_dbg("NULL SEM!!!\n");
        return -1;
    }

    return rt_sem_release(semaphore);
}

int rtos_timer_create(const char *const name,
                      rtos_timer *timer,
                      const uint32_t ms,
                      const uint8_t periodic,
                      void *const args,
                      rtos_timer_fct func)
{
    int ret = 0;
    rt_uint8_t flag;
    if (!timer)
    {
        AIC_LOG_PRINTF("[rtos] null timer: %s\n", name);
        return -1;
    }
    flag = RT_TIMER_FLAG_SOFT_TIMER;
    if (periodic)
    {
        flag |= RT_TIMER_FLAG_PERIODIC;
    }
    else
    {
        flag |= RT_TIMER_FLAG_ONE_SHOT;
    }
    *timer = rt_timer_create(name,
                             func,
                             args,
                             rt_tick_from_millisecond(ms),
                             flag);
    return ret;
}

int rtos_timer_start(rtos_timer timer, const uint32_t ms)
{
    rt_err_t err;
    if (timer == NULL)
    {
        AIC_LOG_PRINTF("[rtos] null timer\n");
        return -1;
    }
    if (ms)
    {
        rt_tick_t tick = rt_tick_from_millisecond(ms);
        err = rt_timer_control(timer, RT_TIMER_CTRL_SET_TIME, &tick);
        if (err)
        {
            AIC_LOG_PRINTF("[rtos] timer ctrl fail, err=%d\n", err);
            return -2;
        }
    }
    err = rt_timer_start(timer);
    if (err)
    {
        AIC_LOG_PRINTF("[rtos] timer start fail, err=%d\n", err);
        return -3;
    }
    return 0;
}

int rtos_timer_get_status(rtos_timer timer, rtos_timer_status *timer_status)
{
    rt_err_t err;
    rt_uint32_t cur_state;
    if (timer == NULL)
    {
        AIC_LOG_PRINTF("[rtos] null timer\n");
        return -1;
    }
    err = rt_timer_control(timer, RT_TIMER_CTRL_GET_STATE, &cur_state);
    if (err)
    {
        AIC_LOG_PRINTF("[rtos] timer ctrl fail, err=%d\n", err);
        return -2;
    }
    if (timer_status)
    {
        *timer_status = (cur_state == RT_TIMER_FLAG_DEACTIVATED) ? AIC_RTOS_TIMER_DACT : AIC_RTOS_TIMER_ACT;
    }
    return 0;
}

int rtos_timer_stop(rtos_timer timer)
{
    rt_err_t err;
    if (timer == NULL)
    {
        AIC_LOG_PRINTF("[rtos] null timer\n");
        return -1;
    }
    err = rt_timer_stop(timer);
    if (err)
    {
        AIC_LOG_PRINTF("[rtos] timer stop fail, err=%d\n", err);
        return -2;
    }
    return 0;
}

int rtos_timer_stop_isr(rtos_timer timer)
{
    AIC_ASSERT_ERR(0);
    return 0;
}

int rtos_timer_delete(rtos_timer timer)
{
    rt_err_t err;
    if (timer == NULL)
    {
        AIC_LOG_PRINTF("[rtos] null timer\n");
        return -1;
    }
    err = rt_timer_delete(timer);
    if (err)
    {
        AIC_LOG_PRINTF("[rtos] timer del fail, err=%d\n", err);
        return -2;
    }
    return 0;
}

int rtos_mutex_create(rtos_mutex *mutex, const char *const name)
{
    rt_mutex_t pmutex;

    if (mutex == NULL)
    {
        return -1;
    }

    pmutex = rt_mutex_create(name, RT_IPC_FLAG_PRIO);

#if RTOS_AL_INFO_DUMP
    //DBG_OSAL_INF("%s mutex = %p, name = %s\n", __func__, pmutex, name);
#endif

    if (pmutex == NULL)
    {
        return -3;
    }

    *mutex = (rtos_mutex)pmutex;
    return 0;
}

void rtos_mutex_delete(rtos_mutex mutex)
{
    rt_mutex_t pmutex = (rt_mutex_t)mutex;
    rt_mutex_delete(pmutex);
}

int rtos_mutex_lock(rtos_mutex mutex, int timeout)
{
    rtos_err ret;

    if (mutex == NULL)
    {
        AIC_LOG_PRINTF("[rtos]lock mutex null:%p %x\n", &mutex, ((rt_uint32_t) __builtin_return_address(0)));
        return -1;
    }

    //OsResSt *st = rtos_res_find_res(mutex,RES_TYPE_MUTEX);
    //AIC_LOG_PRINTF("[rtos]lock %s->%#x %#x\n",st->name,st->ref,mutex);

    ret = rt_mutex_take(mutex, rtos_ms_to_tick(timeout));

    return ret;
}

int rtos_mutex_unlock(rtos_mutex mutex)
{
    rtos_err ret;
    if (mutex == NULL)
    {
        AIC_LOG_PRINTF("[rtos]unlock mutex null\n");
        return -1;
    }
    //AIC_LOG_PRINTF("[rtos]unlock mutex: %p %x\n",&mutex, ((rt_uint32_t) __builtin_return_address(0)));
    //OsResSt *st = rtos_res_find_res(mutex,RES_TYPE_MUTEX);
    //AIC_LOG_PRINTF("[rtos]unlock %s->%#x %#x\n",st->name,st->ref,mutex);
    ret = rt_mutex_release(mutex);

    return ret;
}

extern int clock_gettime(clockid_t clockid, struct timespec *tp);

int aic_time_get(enum time_origin_t origin, uint32_t *sec, uint32_t *usec)
{
    struct timespec tp;
    clock_gettime(CLOCK_REALTIME, &tp);

    *sec = tp.tv_sec;
    *usec = tp.tv_nsec / 1000;

    //aic_dbg("%s:%d %d",__func__,*sec,*usec);

    return 0;
}

void rtos_critical_enter(void)
{
    rt_enter_critical();
}

void rtos_critical_exit(void)
{
    rt_exit_critical();
}


