/**
  ******************************************************************************
  * @file   cpu_usage_profiler.c
  * @author Sifli software development team
  * @brief Sibles source of wrapper device for ipc queue
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
#include "rtthread.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "board.h"
//#include "hwtimer.h"
#include "bf0_hal_lrc_cal.h"
#include "cpu_usage_profiler.h"
#include "log.h"

#ifdef SOC_BF0_HCPU
    #define CPU_READ_GTIMER()  HAL_HPAON_READ_GTIMER()
#elif defined(SOC_BF0_LCPU)
    #define CPU_READ_GTIMER()  HAL_LPAON_READ_GTIMER()
#else
    #error "Invalid config"
#endif /* SOC_BF0_HCPU */

#define CPU_GET_LPTIM_FREQ()   (cpu_lptim_freq)

#define CPU_USEC_PER_SECOND     (1000000)

#ifdef CPU_USAGE_METRICS_USE_COLLECTOR
    #include "metrics_collector.h"
    #include "metrics_id_middleware.h"
#endif /* CPU_USAGE_METRICS_USE_COLLECTOR */


#ifdef CPU_USAGE_METRICS_ENABLED
    #define CPU_THREAD_NAME_LEN  (8)
    #ifdef RT_USING_PTHREADS
        #error "RT_USING_PTHREADS cannot be enabled when CPU_USAGE_METRICS_ENABLED is enabled"
    #endif /* RT_USING_PTHREADS */
#endif /* CPU_USAGE_METRICS_ENABLED */


typedef struct
{
    rt_int32_t sec;      /* second */
    rt_int32_t usec;     /* microsecond */
} thread_switch_time_t;


typedef struct
{
    struct rt_thread *thread;
    thread_switch_time_t time;
    char timer_name[4];
} thread_switch_hist_item_t;

typedef struct
{
    uint32_t index;
    thread_switch_hist_item_t hist[CPU_PROFILER_MAX_THREAD_SWITCH_HIST_LEN];
} thread_switch_hist_t;

#ifdef CPU_PROFILER_RECORD_ISR_HISTORY_ENABLED
typedef struct
{
    IRQn_Type irq_no;
    thread_switch_time_t time;
} isr_hist_item_t;

typedef struct
{
    uint32_t index;
    isr_hist_item_t hist[CPU_PROFILER_MAX_ISR_HIST_LEN];
} isr_hist_t;

#endif /* CPU_PROFILER_RECORD_ISR_HISTORY_ENABLED */

#ifdef CPU_USAGE_METRICS_ENABLED
typedef struct
{
    char name[CPU_THREAD_NAME_LEN];
    float run_time;
} cpu_thread_run_time_t;

typedef struct
{
    float idle_run_time;
    float other_run_time;
    uint8_t thread_num;
    uint8_t reserved[3];
    cpu_thread_run_time_t thread_run_time[0];
} cpu_usage_metrics_t;

static cpu_usage_metrics_t cpu_metrics;
#endif /* CPU_USAGE_METRICS_ENABLED */

#ifdef CPU_USAGE_METRICS_USE_COLLECTOR
    mc_collector_t cpu_usage_metrics_collector;
#endif /* CPU_USAGE_METRICS_USE_COLLECTOR */




RT_USED static thread_switch_hist_t thread_switch_hist;

#ifdef CPU_PROFILER_RECORD_ISR_HISTORY_ENABLED
    RT_USED static isr_hist_t isr_hist;
#endif /* CPU_PROFILER_RECORD_ISR_HISTORY_ENABLED */




static bool first_switch;
static thread_switch_time_t switch_in_time;
static uint32_t switch_in_gtimer;
/** other threads total run time in microsecond */
static uint32_t other_run_time;
/** idle thread run time in microsecond */
static uint32_t idle_run_time;
/** cpu usage percentage */
static float cpu_usage;
static uint32_t cpu_lptim_freq;

static void cpu_prof_reset(void);
#ifdef CPU_USAGE_METRICS_ENABLED
    static void print_timer_callback(void *parameter);
#endif

#ifdef CPU_PROFILER_RECORD_ISR_HISTORY_ENABLED
static void isr_enter_hook(void)
{
    uint32_t run_time;
    uint32_t curr_gtimer;
    rt_hwtimerval_t curr_time;
    uint32_t freq = CPU_GET_LPTIM_FREQ();
    IRQn_Type irqn;

    irqn = (int32_t)(__get_xPSR() & 0x1FF) - 16;
    if (SysTick_IRQn == irqn)
    {
        return;
    }

    curr_gtimer = CPU_READ_GTIMER();
    curr_time.sec = curr_gtimer / freq;
    curr_time.usec = (curr_gtimer - curr_time.sec * freq) * (float)CPU_USEC_PER_SECOND / freq;

    if ((isr_hist.index + 1) >= CPU_PROFILER_MAX_ISR_HIST_LEN)
    {
        isr_hist.index = 0;
    }
    else
    {
        isr_hist.index = isr_hist.index + 1;
    }
    isr_hist.hist[isr_hist.index].time.sec = curr_time.sec;
    isr_hist.hist[isr_hist.index].time.usec = curr_time.usec;
    isr_hist.hist[isr_hist.index].irq_no = irqn;
}
#endif /* CPU_PROFILER_RECORD_ISR_HISTORY_ENABLED */

#ifdef PM_USE_RC48
extern uint8_t g_xt48_used;
static void update_clock_source(void)
{
#ifdef LPSYS_AON_SLP_CTRL_BLE_WKUP_Msk
    if ((0 == GET_REG_VAL(hwp_lpsys_aon->SLP_CTRL, LPSYS_AON_SLP_CTRL_BLE_WKUP_Msk, LPSYS_AON_SLP_CTRL_BLE_WKUP_Pos)) && (g_xt48_used == 0))
#else
    if ((0 == GET_REG_VAL(hwp_lpsys_aon->SLP_CTRL, LPSYS_AON_SLP_CTRL_BT_WKUP_Msk, LPSYS_AON_SLP_CTRL_BT_WKUP_Pos)) && (g_xt48_used == 0))
#endif
    {
        /* switch to RC48 if BT is in sleep */
        HAL_RCC_LCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_HRC48);
        HAL_LPAON_DisableXT48();
    }
}
#endif /* PM_USE_RC48 */

static void thread_tick_count(struct rt_thread *from, struct rt_thread *to)
{
    uint32_t run_time;
    uint32_t curr_gtimer;
    uint32_t freq = CPU_GET_LPTIM_FREQ();
    float run_time_float;
    float temp;
    uint32_t sec_delta;
    uint32_t usec_delta;
#ifdef RT_USING_TIMER_SOFT
    struct rt_timer *next_timer;
#endif /* RT_USING_TIMER_SOFT */

    curr_gtimer = CPU_READ_GTIMER();
    if (!first_switch)
    {
        run_time = curr_gtimer - switch_in_gtimer;
        /* calculate run_time in microsecond */
        run_time = run_time * (float)CPU_USEC_PER_SECOND / freq;

        sec_delta = run_time / CPU_USEC_PER_SECOND;
        usec_delta = run_time - sec_delta * CPU_USEC_PER_SECOND;

        /* calculate switch_in_time by delta */
        switch_in_time.sec += sec_delta;
        switch_in_time.usec += usec_delta;
        if (switch_in_time.usec >= CPU_USEC_PER_SECOND)
        {
            switch_in_time.usec -= CPU_USEC_PER_SECOND;
            switch_in_time.sec += 1;
        }

#ifdef CPU_USAGE_METRICS_ENABLED
        run_time_float = (float)run_time / CPU_USEC_PER_SECOND;
#endif /* CPU_USAGE_METRICS_ENABLED */


        if ((RT_THREAD_PRIORITY_MAX - 1) == from->init_priority)
        {
            /* idle task */
            idle_run_time += run_time;

#ifdef CPU_USAGE_METRICS_ENABLED
            cpu_metrics.idle_run_time += run_time_float;
#endif /* CPU_USAGE_METRICS_ENABLED */

        }
        else
        {
            /* other task */
            other_run_time += run_time;

#ifdef CPU_USAGE_METRICS_ENABLED
            cpu_metrics.other_run_time += run_time_float;
#endif /* CPU_USAGE_METRICS_ENABLED */

        }

#ifdef CPU_USAGE_METRICS_ENABLED

        memcpy(&temp, &from->user_data, sizeof(temp));
        temp += run_time_float;
        memcpy(&from->user_data, &temp, sizeof(from->user_data));

#endif /* CPU_USAGE_METRICS_ENABLED */
    }
    else
    {
        switch_in_time.sec = curr_gtimer / freq;
        switch_in_time.usec = (curr_gtimer - switch_in_time.sec * freq) * (float)CPU_USEC_PER_SECOND / freq;
    }

    switch_in_gtimer = curr_gtimer;

    first_switch = false;
    if ((thread_switch_hist.index + 1) >= CPU_PROFILER_MAX_THREAD_SWITCH_HIST_LEN)
    {
        thread_switch_hist.index = 0;
    }
    else
    {
        thread_switch_hist.index = thread_switch_hist.index + 1;
    }
    thread_switch_hist.hist[thread_switch_hist.index].time.sec = switch_in_time.sec;
    thread_switch_hist.hist[thread_switch_hist.index].time.usec = switch_in_time.usec;
    thread_switch_hist.hist[thread_switch_hist.index].thread = to;
#ifdef RT_USING_TIMER_SOFT
    if ((to->name[0] == 't')
            && (to->name[1] == 'i')
            && (to->name[2] == 'm')
            && (to->name[3] == 'e'))
    {
        next_timer = rt_timer_next_soft_timer();
        if (next_timer)
        {
            thread_switch_hist.hist[thread_switch_hist.index].timer_name[0] = next_timer->parent.name[0];
            thread_switch_hist.hist[thread_switch_hist.index].timer_name[1] = next_timer->parent.name[1];
            thread_switch_hist.hist[thread_switch_hist.index].timer_name[2] = next_timer->parent.name[2];
            thread_switch_hist.hist[thread_switch_hist.index].timer_name[3] = next_timer->parent.name[3];
        }
    }
    else
    {
        thread_switch_hist.hist[thread_switch_hist.index].timer_name[0] = 0;
    }
#endif /* RT_USING_TIMER_SOFT */

    if ((idle_run_time + other_run_time) > 2000000)
    {
        cpu_prof_reset();
    }

#ifdef PM_USE_RC48
    update_clock_source();
#endif /* PM_USE_RC48 */
}

static int cpu_prof_init(void)
{
    cpu_prof_reset();
    thread_switch_hist.index = 0;
    cpu_lptim_freq = (uint32_t)HAL_LPTIM_GetFreq();
    /* set scheduler hook */
    rt_scheduler_sethook(thread_tick_count);

#ifdef CPU_PROFILER_RECORD_ISR_HISTORY_ENABLED
    rt_interrupt_enter_sethook(isr_enter_hook);
#endif /* CPU_PROFILER_RECORD_ISR_HISTORY_ENABLED */

    return 0;
}
MSH_CMD_EXPORT(cpu_prof_init, init prof);
#ifndef PKG_USING_SYSTEMVIEW
    INIT_COMPONENT_EXPORT(cpu_prof_init);
#endif /* PKG_USING_SYSTEMVIEW */

static void cpu_prof_deinit()
{
    rt_scheduler_sethook(RT_NULL);
}
MSH_CMD_EXPORT(cpu_prof_deinit, detach profiling hook);

static void cpu_prof_reset()
{
    rt_enter_critical();
    if ((idle_run_time + other_run_time) > 0)
    {
        cpu_usage = (float)other_run_time * 100 / (idle_run_time + other_run_time);
    }
    else
    {
        cpu_usage = -1;
    }
    idle_run_time = 0;
    other_run_time = 0;
    first_switch = true;
    rt_exit_critical();
}
MSH_CMD_EXPORT(cpu_prof_reset, reset profiling counter);

static void cpu_prof_view()
{
    if (cpu_usage > 0)
    {
        LOG_I("CPU usage: %f\n", cpu_usage);
    }
    else
    {
        LOG_I("CPU usage: NaN,%d,%d\n", idle_run_time, other_run_time);
    }

#ifdef CPU_USAGE_METRICS_ENABLED
    print_timer_callback(NULL);
#endif
}
MSH_CMD_EXPORT_ALIAS(cpu_prof_view, cpu, cpu: view CPU profiling result);

float cpu_get_usage(void)
{
    return cpu_usage;
}

uint32_t cpu_get_hw_us(void)
{
    uint32_t us;
    if (0 == cpu_lptim_freq)
    {
        cpu_lptim_freq = (uint32_t)HAL_LPTIM_GetFreq();
    }

    us = CPU_READ_GTIMER() * (uint64_t)1000000 / cpu_lptim_freq;
    return us;
}

#ifdef CPU_USAGE_METRICS_USE_COLLECTOR
static void cpu_usage_metrics_collect(void *user_data)
{
    struct rt_object_information *info;
    uint8_t thread_num;
    rt_uint32_t used_size;
    struct rt_thread *thread;
    struct rt_list_node *node;
    struct rt_list_node *thread_list;
    uint16_t data_len;
    cpu_usage_metrics_t *metrics;
    cpu_thread_run_time_t *thread_run_time;
    uint32_t i;
    uint8_t name_len;

    info = rt_object_get_information(RT_Object_Class_Thread);

    thread_list = &info->object_list;
    thread_num = 0;
    for (node = thread_list->next; node != thread_list; node = node->next)
    {
        thread_num++;
    }

    data_len = sizeof(cpu_usage_metrics_t) + thread_num * sizeof(cpu_thread_run_time_t);
    metrics = mc_alloc_metrics(METRICS_MW_CPU_USAGE, data_len);
    RT_ASSERT(metrics);
    metrics->idle_run_time = cpu_metrics.idle_run_time;
    metrics->other_run_time = cpu_metrics.other_run_time;
    metrics->thread_num = thread_num;
    i = 0;
    thread_run_time = &metrics->thread_run_time[0];
    for (node = thread_list->next; (node != thread_list) && (i < thread_num);
            node = node->next)
    {
        thread = rt_list_entry(node, struct rt_thread, list);

        if ((CPU_THREAD_NAME_LEN - 1) > RT_NAME_MAX)
        {
            name_len = RT_NAME_MAX;
        }
        else
        {
            name_len = (CPU_THREAD_NAME_LEN - 1);
        }
        strncpy(thread_run_time->name, thread->name, name_len);
        thread_run_time->name[name_len] = 0;
        memcpy(&thread_run_time->run_time, &thread->user_data,
               sizeof(thread_run_time->run_time));
        thread->user_data = 0;
        thread_run_time++;
    }
    cpu_metrics.idle_run_time = 0;
    cpu_metrics.other_run_time = 0;
    mc_save_metrics(metrics, true);
}


static int cpu_usage_metrics_init(void)
{
    mc_err_t err;

    cpu_usage_metrics_collector.callback = cpu_usage_metrics_collect;
    cpu_usage_metrics_collector.period = MC_PERIOD_EVERY_HOUR;
    cpu_usage_metrics_collector.user_data = 0;

    err = mc_register_collector(&cpu_usage_metrics_collector);
    RT_ASSERT(MC_OK == err);
    return 0;

}
INIT_APP_EXPORT(cpu_usage_metrics_init);

#ifndef MC_CLIENT_ENABLED
#if 0
static bool metrics_read_callback(uint16_t id, uint8_t core, uint16_t data_len, uint32_t time, void *data)
{
    cpu_usage_metrics_t *metrics;
    uint32_t i;
    cpu_thread_run_time_t *thread_run_time;

    if (0 == id)
    {
        metrics = (cpu_usage_metrics_t *)data;
        LOG_I("============================");
        LOG_I("[%d][%d]: %7.2f/%7.2f", time, core, metrics->idle_run_time, metrics->other_run_time + metrics->idle_run_time);
        thread_run_time = &metrics->thread_run_time[0];
        for (i = 0; i < metrics->thread_num; i++)
        {
            LOG_I("[%-8s]: %7.2f", thread_run_time->name, thread_run_time->run_time);
            thread_run_time++;
        }
    }

    return false;

}

static void cpu_metrics_list(int argc, char **argv)
{
    mc_err_t err;

    err = mc_read_metrics(metrics_read_callback);
    RT_ASSERT(MC_OK == err);

}
MSH_CMD_EXPORT(cpu_metrics_list, list cpu usage metrics);
#endif
#endif /* MC_CLIENT_ENABLED */

#endif /* CPU_USAGE_METRICS_USE_COLLECTOR */



#ifdef CPU_USAGE_METRICS_ENABLED
static void print_timer_callback(void *parameter)
{
    struct rt_object_information *info;
    uint8_t thread_num;
    rt_uint32_t used_size;
    struct rt_thread *thread;
    struct rt_list_node *node;
    struct rt_list_node *thread_list;
    uint16_t data_len;
    cpu_usage_metrics_t *metrics;
    cpu_thread_run_time_t *thread_run_time;
    uint32_t i;
    uint8_t name_len;

    info = rt_object_get_information(RT_Object_Class_Thread);

    thread_list = &info->object_list;
    thread_num = 0;
    for (node = thread_list->next; node != thread_list; node = node->next)
    {
        thread_num++;
    }

    data_len = sizeof(cpu_usage_metrics_t) + thread_num * sizeof(cpu_thread_run_time_t);
    metrics = rt_malloc(data_len);
    RT_ASSERT(metrics);
    metrics->idle_run_time = cpu_metrics.idle_run_time;
    metrics->other_run_time = cpu_metrics.other_run_time;
    metrics->thread_num = thread_num;
    i = 0;
    thread_run_time = &metrics->thread_run_time[0];
    for (node = thread_list->next; (node != thread_list) && (i < thread_num);
            node = node->next)
    {
        thread = rt_list_entry(node, struct rt_thread, list);

        if ((CPU_THREAD_NAME_LEN - 1) > RT_NAME_MAX)
        {
            name_len = RT_NAME_MAX;
        }
        else
        {
            name_len = (CPU_THREAD_NAME_LEN - 1);
        }
        strncpy(thread_run_time->name, thread->name, name_len);
        thread_run_time->name[name_len] = 0;
        memcpy(&thread_run_time->run_time, &thread->user_data,
               sizeof(thread_run_time->run_time));
        thread->user_data = 0;
        thread_run_time++;
    }
    cpu_metrics.idle_run_time = 0;
    cpu_metrics.other_run_time = 0;

    float toal_run_time = (float)(metrics->other_run_time + metrics->idle_run_time);
    LOG_I("============================");
    LOG_I("CPU Usage: %7.2f/%7.2f (%7.2f%%)", metrics->idle_run_time, metrics->other_run_time + metrics->idle_run_time, (float)((float)(metrics->other_run_time / toal_run_time) * 100));
    LOG_I("========================================================");
    LOG_I("%-*.s    run_time(s)    load(%)", CPU_THREAD_NAME_LEN, "thread");
    LOG_I("--------------------------------------------------------");

    thread_run_time = &metrics->thread_run_time[0];
    for (i = 0; i < metrics->thread_num; i++)
    {
        LOG_I("[%-*s]: %7.2f %7.2f%% ", CPU_THREAD_NAME_LEN, thread_run_time->name, thread_run_time->run_time, (float)((float)(thread_run_time->run_time / toal_run_time) * 100));
        thread_run_time++;
    }

    rt_free(metrics);
}
#endif

#ifdef CPU_USAGE_METRICS_PRINT_DIRECTLY

static int cpu_usage_metrics_init(void)
{
    rt_err_t err;
    rt_timer_t timer;

    timer = rt_timer_create("cpu_usage", print_timer_callback, 0, rt_tick_from_millisecond(CPU_USAGE_METRICS_PRINT_PERIOD * 1000),
                            RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    RT_ASSERT(timer);
    err = rt_timer_start(timer);

    RT_ASSERT(RT_EOK == err);
    return 0;

}
INIT_APP_EXPORT(cpu_usage_metrics_init);

#endif /* CPU_USAGE_METRICS_PRINT_DIRECTLY */

