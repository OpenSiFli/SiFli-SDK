/**
 ****************************************************************************************
 *
 * @file rw_os.c
 *
 * @brief This file contains the event handling primitives.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */



/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rtconfig.h"
#include "arch.h"         // architecture
#include "rwip.h"
#include "dbg_trc.h"
#include <stdint.h>       // standard integer definition
#include <stddef.h>       // standard definition
#include <string.h>       // memcpy defintion
#include "ke_event.h"


#ifdef BSP_USING_RTTHREAD

#include <rtthread.h>
#ifdef BF0_LCPU
    #define KE_EVT_TASK_STACK_SIZE    (1380)
#else
    #define KE_EVT_TASK_STACK_SIZE    (4096)
#endif
#define KE_EVT_DBG_TASK_STACK_SIZE (1024)
#define KE_EVT_RECV_TASK_STACK_SIZE (1024)
//#define KE_EVT_TASK_PRIOIRTY      (10)

const static int rw_os_priority_mapping[KE_EVNET_PRI_MAX] =
{
    RT_THREAD_PRIORITY_LOW, RT_THREAD_PRIORITY_MIDDLE, RT_THREAD_PRIORITY_HIGH, RT_THREAD_PRIORITY_HIGH + 2 * RT_THREAD_PRIORITY_HIGHER
};

const static int rw_os_stack_size_mapping[KE_EVNET_PRI_MAX] =
{
    KE_EVT_DBG_TASK_STACK_SIZE, KE_EVT_TASK_STACK_SIZE, KE_EVT_TASK_STACK_SIZE, KE_EVT_RECV_TASK_STACK_SIZE
};



void *rw_os_thread_create(int8_t *name, void *task_entry, void *parameter, uint8_t int_priority)
{
    if (int_priority >= KE_EVNET_PRI_MAX)
        int_priority = KE_EVNET_PRI_HIGH;
    char t_name[10] = {0};
    rt_snprintf(t_name, 10, "KE_EVT%d", int_priority);
    rt_thread_t task_handle = rt_thread_create(t_name, task_entry, parameter,
                              rw_os_stack_size_mapping[int_priority], rw_os_priority_mapping[int_priority], RT_THREAD_TICK_DEFAULT * 2);
    if (RT_NULL != task_handle)
        rt_thread_startup(task_handle);
    else
        ASSERT_ERR_FORCE(0);
    return task_handle;
}
void *rw_os_sem_create(int8_t *name, int value)
{
    return rt_sem_create((const char *) name, value, RT_IPC_FLAG_FIFO);
}
void rw_os_sem_set(void *event)
{
    rt_sem_release(event);
}
void  rw_os_sem_get(void *event)
{
    rt_sem_t sem;
    sem = event;
    rt_sem_take(sem, RT_WAITING_FOREVER);
}

void rw_os_timer_del(void *timer)
{
    rt_timer_delete(timer);
}

void *rw_os_timer_create(int8_t *name, void *timeout_cbk, void *param, int ms)
{
    return rt_timer_create("KE", timeout_cbk, param,
                           rt_tick_from_millisecond(ms * 10),  RT_TIMER_FLAG_SOFT_TIMER);
}

void  rw_os_timer_start(void *timer)
{
    rt_timer_start(timer);
}

uint64_t rw_os_time_get()
{
    return ((uint64_t)rt_tick_get() * 100 / RT_TICK_PER_SECOND)&RWIP_MAX_10MS_TIME;
}

void rw_os_delay_ms(int ms)
{
    rt_thread_delay(rt_tick_from_millisecond(ms));
}

#ifdef SOC_BF0_BCPU
void dbg_assert_hook(const char *ex, const char *func, rt_size_t line)
{
    dbg_trc_req_sw_ass(ASSERT_ERROR, func, line, 0, 0);
}
#endif

void rw_os_init(void)
{
#ifdef SOC_BF0_BCPU

#ifdef ARCH_ARM
    rt_hw_exception_install(dbg_hard_fault_crash_dump);
#endif
    rt_assert_set_hook(&dbg_assert_hook);

#endif //SOC_BF0_BCPU
}

#endif

#if !EMB_PRESENT

static struct rt_timer rw_rttimer;

rwip_time_t rwip_time_get(void)
{
    rwip_time_t res;
    rt_tick_t ticks = rt_tick_get_millisecond();

    res.bts = ticks;
    // Other files of res is not used.
    return res;
};

void rwip_timer_co_set(uint32_t target_bts)
{
    rt_tick_t rwip_ticks = rt_tick_from_millisecond(target_bts / 1000);
    void timeout_isr(void *param);

    if (rw_rttimer.timeout_func == NULL)
    {
        rt_timer_init(&rw_rttimer, "BLEHost", timeout_isr, NULL, rwip_ticks, RT_TIMER_FLAG_ONE_SHOT);
        rt_timer_start(&rw_rttimer);
    }
    else
    {
        rt_timer_stop(&rw_rttimer);
        rt_timer_control(&rw_rttimer, RT_TIMER_CTRL_SET_TIME, (void *)&rwip_ticks);
        rt_timer_start(&rw_rttimer);
    }
}


#endif



#ifndef BSP_USING_PC_SIMULATOR
#if defined(BF0_HCPU)
__WEAK void bluetooth_wakeup_lcpu(void)
{
}

__WEAK void bluetooth_release_lcpu(void)
{
}
#endif // !BSP_USING_PC_SIMULATOR

#else
void bluetooth_wakeup_lcpu_weak(void)
{
}

void bluetooth_release_lcpu_weak(void)
{
}

#pragma comment(linker, "/alternatename:_bluetooth_wakeup_lcpu=_bluetooth_wakeup_lcpu_weak")
#pragma comment(linker, "/alternatename:_bluetooth_release_lcpu=_bluetooth_release_lcpu_weak")

#endif


#ifdef BSP_USING_NO_OS
void *rw_os_thread_create(int8_t *name, void *task_entry, void *parameter, int priority)
{

    return NULL;
}

void *rw_os_sem_create(int8_t *name, int value)
{
    return NULL;
}

void rw_os_sem_set(void *event)
{
}

void  rw_os_sem_get(void *event)
{
}
#endif

