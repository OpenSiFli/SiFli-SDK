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

#include "rtconfig.h"
#include "rtthread.h"

#include "register.h"

#define CFG_ALLOC_IN_SRAM           1

// task priority
uint32_t xhci_busrx_priority        = 10; // (RT_THREAD_PRIORITY_MAX - 22)
uint32_t xhci_bustx_priority        = 10; // (RT_THREAD_PRIORITY_MAX - 22)
#if defined(CONFIG_VNET_MODE)
    uint32_t xdrv_vnet_init_priority    = 20;
#endif
uint32_t app_wifi_cli_priority      = 20;

// stack size (bytes)
uint32_t xhci_busrx_stack_size      = 2048;
uint32_t xhci_bustx_stack_size      = 2048;
#if defined(CONFIG_VNET_MODE)
    uint32_t xdrv_vnet_init_stack_size  = 2048;
#endif
uint32_t app_wifi_cli_stack_size    = 2048;

rt_tick_t rtos_ms_to_tick(unsigned long ms)
{
    rt_tick_t tick;
    tick = RT_TICK_PER_SECOND * (ms / 1000);
    tick += (RT_TICK_PER_SECOND * (ms % 1000) + 999) / 1000;
    return tick;
}

__INLINE unsigned long rtos_tick_to_ms(unsigned long tick)
{
    return tick * (1000 / RT_TICK_PER_SECOND);
}

unsigned long rt_now_veneer(bool isr)
{
    unsigned long tick = rt_tick_get();

    return rtos_tick_to_ms(tick);
}

#ifndef RT_USING_CONSOLE
/**
 * This function will print a formatted string on system console
 *
 * @param fmt the format
 */
void rt_kprintf(const char *fmt, ...)
{
    va_list args;
    rt_size_t length;
    static char rt_log_buf[128];
    va_start(args, fmt);
    length = rt_vsnprintf(rt_log_buf, sizeof(rt_log_buf) - 1, fmt, args);
    if (length > 128 - 1)
        length = 128 - 1;
    rt_hw_console_output(rt_log_buf);
    va_end(args);
}
#endif

#if CFG_ALLOC_IN_SRAM
void *rt_malloc_veneer(uint32_t size)
{
//    return rt_malloc((rt_size_t)size);
#if !defined(SYS_HEAP_IN_PSRAM)
    return (void *)RT_KERNEL_MALLOC((rt_size_t)size);
#else
    extern void *app_sram_alloc(size_t size);
    return app_sram_alloc((size_t)size);
#endif
}

void *rt_calloc_veneer(uint32_t nb_elt, uint32_t size)
{
//    return rt_calloc((rt_size_t)nb_elt, (rt_size_t)size);
#if !defined(SYS_HEAP_IN_PSRAM)
    void *ptr = (void *)RT_KERNEL_MALLOC((rt_size_t)nb_elt * size);
    if (ptr)
        rt_memset(ptr, 0, nb_elt * size);
    return ptr;
#else
    extern void *app_sram_calloc(rt_size_t count, rt_size_t size);
    return app_sram_calloc((rt_size_t)nb_elt, (rt_size_t)size);
#endif
}

void rt_free_veneer(void *ptr)
{
#if !defined(SYS_HEAP_IN_PSRAM)
    RT_KERNEL_FREE(ptr);
#else
    extern void app_sram_free(void *ptr);;
    app_sram_free(ptr);
#endif
}
#else
void *rt_malloc_veneer(uint32_t size)
{
    return rt_malloc((rt_size_t)size);
}

void *rt_calloc_veneer(uint32_t nb_elt, uint32_t size)
{
    return rt_calloc((rt_size_t)nb_elt, (rt_size_t)size);
}
#endif

void *rt_memcpy_veneer(void *pdest, const void *psrc, uint32_t size)
{
    return rt_memcpy(pdest, psrc, (rt_ubase_t)size);
}

void *rt_memset_veneer(void *pdest, uint8_t byte, uint32_t size)
{
    return rt_memset(pdest, (int)byte, (rt_ubase_t)size);
}

void rt_task_critical_enter_veneer(void)
{
    rt_enter_critical();
}

void rt_task_critical_exit_veneer(void)
{
    rt_exit_critical();
}

int rt_task_create_veneer(rtos_task_fct func,
                          const char *const name,
                          int task_id,
                          const uint16_t stack_depth,
                          void *const params,
                          rtos_prio prio,
                          rtos_task_handle *task_handle)
{
    rt_thread_t handle;

    if (task_handle == NULL)
    {
        return -1;
    }

    handle = rt_thread_create((char *)name,
                              func,
                              params,
                              stack_depth,
                              prio,
                              10);

#if RTOS_AL_INFO_DUMP
    DBG_OSAL_INF("%s '%s', handle = %p, stack_size = %#x\n", __func__, name, handle, stack_depth);
#endif

    if (handle == RT_NULL)
    {
        return -3;
    }

    *task_handle = (rtos_task_handle)handle;

    rt_thread_startup(handle);

    return 0;
}

void rt_task_delete_veneer(rtos_task_handle task_handle)
{
    rt_thread_t handle = (rt_thread_t)task_handle;
    rt_err_t ret;

#if RTOS_AL_INFO_DUMP
    DBG_OSAL_INF("%s, handle = %p\n", __func__, handle);
#endif

    if (!handle)
    {
        handle = rt_thread_self();
    }
    ret = rt_thread_delete(handle);
    if (ret)
    {
        DBG_OSAL_ERR("thread del fail, ret=%d\n", ret);
    }
}

void rt_task_suspend_veneer(int duration)
{
    rt_thread_t handle = rt_thread_self();

    if (-1 == duration)
    {
        rt_thread_suspend(handle);
    }
    else
    {
        rt_thread_delay(rtos_ms_to_tick(duration));
    }
}

void rt_task_resume_veneer(rtos_task_handle task_handle)
{
    rt_thread_t handle = (rt_thread_t)task_handle;
    rt_thread_resume(handle);
}

rtos_task_handle rt_get_task_handle_veneer(void)
{
    return (rtos_task_handle)rt_thread_self();
}

int rt_semaphore_create_veneer(rtos_semaphore *semaphore, const char *const name, int max_count, int init_count)
{
    rt_sem_t psem;

    if (semaphore == NULL)
    {
        return -1;
    }

    psem = rt_sem_create(name, init_count, RT_IPC_FLAG_FIFO);

#if RTOS_AL_INFO_DUMP
    DBG_OSAL_INF("%s sema = %p, name = %s\n", __func__, psem, name);
#endif

    if (psem == NULL)
    {
        return -3;
    }

    *semaphore = (rtos_semaphore)psem;

    return 0;
}

void rt_semaphore_delete_veneer(rtos_semaphore semaphore)
{
    rt_sem_t psem = (rt_sem_t)semaphore;
    if (psem)
    {
        rt_sem_delete(psem);
    }
#if RTOS_AL_INFO_DUMP
    DBG_OSAL_INF("%s sema = %p, name = %s\n", __func__, psem);
#endif
}

int rt_semaphore_get_count_veneer(rtos_semaphore semaphore)
{
    rt_sem_t psem = (rt_sem_t)semaphore;
    uint16_t count = 0;

    /// TBD: RT_IPC_CMD_GET_STATE
//    /rt_sem_control(psem, RT_IPC_CMD_GET_STATE, &count);

    return (int)count;
}

int rt_semaphore_wait_veneer(rtos_semaphore semaphore, int timeout)
{
    rt_sem_t psem = (rt_sem_t)semaphore;
    rt_err_t status;

    if (psem == NULL)
    {
        return -1;
    }

    status = rt_sem_take(psem, rt_tick_from_millisecond(timeout));
    if (status == RT_EOK)
    {
        return 0;
    }
    return -1;
}

int rt_semaphore_signal_veneer(rtos_semaphore semaphore, bool isr)
{
    rt_sem_t psem = (rt_sem_t)semaphore;

    if (psem == NULL)
    {
        return -1;
    }

    return rt_sem_release(psem);
}

int rt_mutex_create_veneer(rtos_mutex *mutex, const char *const name)
{
    rt_mutex_t pmutex;

    if (mutex == NULL)
    {
        return -1;
    }

    pmutex = rt_mutex_create(name, RT_IPC_FLAG_PRIO);

#if RTOS_AL_INFO_DUMP
    DBG_OSAL_INF("%s mutex = %p, name = %s\n", __func__, pmutex, name);
#endif

    if (pmutex == NULL)
    {
        return -3;
    }

    *mutex = (rtos_mutex)pmutex;
    return 0;
}

void rt_mutex_delete_veneer(rtos_mutex mutex)
{
    rt_mutex_t pmutex = (rt_mutex_t)mutex;
    rt_mutex_delete(pmutex);
}

int rt_mutex_lock_veneer(rtos_mutex mutex, int timeout)
{
    rt_err_t status;
    rt_mutex_t pmutex = (rt_mutex_t)mutex;
    //DBG_OSAL_INF("l: %p, locked:%d owner:%p\n", mutex, pmutex->locked, pmutex->owner);

    status = rt_mutex_take(pmutex, rt_tick_from_millisecond(timeout));
    if (status == RT_EOK)
    {
        return 0;
    }
    return -1;
}

int rt_mutex_unlock_veneer(rtos_mutex mutex)
{
    rt_mutex_t pmutex = (rt_mutex_t)mutex;
    //DBG_OSAL_INF("ul: %p\n",mutex);
    return rt_mutex_release(pmutex);
}

void rt_atomic_set_veneer(rtos_atomic_t *atom, int val)
{
    atom->counter = val;
}

int rt_atomic_read_veneer(rtos_atomic_t *atom)
{
    return atom->counter;
}

void rt_atomic_add_veneer(rtos_atomic_t *atom, int val)
{
    rt_enter_critical();
    atom->counter += val;
    rt_exit_critical();
}

void rt_atomic_sub_veneer(rtos_atomic_t *atom, int val)
{
    rt_enter_critical();
    atom->counter -= val;
    rt_exit_critical();
}

void rt_atomic_inc_veneer(rtos_atomic_t *atom)
{
    rt_enter_critical();
    atom->counter++;
    rt_exit_critical();
}

void rt_atomic_dec_veneer(rtos_atomic_t *atom)
{
    rt_enter_critical();
    atom->counter--;
    rt_exit_critical();
}

int rt_atomic_add_ret_veneer(rtos_atomic_t *atom, int val)
{
    int ret;
    rt_enter_critical();
    ret = atom->counter + val;
    atom->counter = ret;
    rt_exit_critical();
    return ret;
}

int rt_atomic_sub_ret_veneer(rtos_atomic_t *atom, int val)
{
    int ret;
    rt_enter_critical();
    ret = atom->counter - val;
    atom->counter = ret;
    rt_exit_critical();
    return ret;
}

int rt_atomic_inc_ret_veneer(rtos_atomic_t *atom)
{
    int ret;
    rt_enter_critical();
    ret = atom->counter + 1;
    atom->counter = ret;
    rt_exit_critical();
    return ret;
}

int rt_atomic_dec_ret_veneer(rtos_atomic_t *atom)
{
    int ret;
    rt_enter_critical();
    ret = atom->counter - 1;
    atom->counter = ret;
    rt_exit_critical();
    return ret;
}

#ifndef IS_DCACHED_RAM
    #define IS_DCACHED_RAM(addr)    (((uint32_t) addr) >= (PSRAM_BASE))
#endif

#ifndef DCACH_SIZE
    #define DCACH_SIZE  16384
#endif

void rt_dcache_clean_veneer(void *adr, unsigned int len)
{
    if (IS_DCACHED_RAM(adr))
    {
        if (len >= DCACH_SIZE)
        {
            SCB_CleanDCache();
        }
        else
        {
            SCB_CleanDCache_by_Addr(adr, len);
        }
    }
}

void rt_dcache_invalidate_veneer(void *adr, unsigned int len)
{
    if (IS_DCACHED_RAM(adr))
    {
        if (len >= DCACH_SIZE)
        {
            SCB_InvalidateDCache();
        }
        else
        {
            SCB_InvalidateDCache_by_Addr(adr, len);
        }
    }
}

const osal_porting_ops_t osal_porting_ops =
{
    .osal_now           = rt_now_veneer,
    .osal_dbg_printf    = rt_kprintf,
    .osal_malloc        = rt_malloc_veneer,
    .osal_calloc        = rt_calloc_veneer,
#if CFG_ALLOC_IN_SRAM
    .osal_free          = rt_free_veneer,
#else
    .osal_free          = rt_free,
#endif
    .osal_memcpy        = rt_memcpy_veneer,
    .osal_memset        = rt_memset_veneer,
    .osal_task_critical_enter = rt_task_critical_enter_veneer,
    .osal_task_critical_exit  = rt_task_critical_exit_veneer,
    .osal_task_create       = rt_task_create_veneer,
    .osal_task_delete       = rt_task_delete_veneer,
    .osal_task_suspend      = rt_task_suspend_veneer,
    .osal_task_resume       = rt_task_resume_veneer,
    .osal_get_task_handle   = rt_get_task_handle_veneer,
    .osal_semaphore_create      = rt_semaphore_create_veneer,
    .osal_semaphore_delete      = rt_semaphore_delete_veneer,
    .osal_semaphore_get_count   = rt_semaphore_get_count_veneer,
    .osal_semaphore_wait        = rt_semaphore_wait_veneer,
    .osal_semaphore_signal      = rt_semaphore_signal_veneer,
    .osal_mutex_create  = rt_mutex_create_veneer,
    .osal_mutex_delete  = rt_mutex_delete_veneer,
    .osal_mutex_lock    = rt_mutex_lock_veneer,
    .osal_mutex_unlock  = rt_mutex_unlock_veneer,
    .osal_atomic_set        = rt_atomic_set_veneer,
    .osal_atomic_read       = rt_atomic_read_veneer,
    .osal_atomic_add        = rt_atomic_add_veneer,
    .osal_atomic_sub        = rt_atomic_sub_veneer,
    .osal_atomic_inc        = rt_atomic_inc_veneer,
    .osal_atomic_dec        = rt_atomic_dec_veneer,
    .osal_atomic_add_ret    = rt_atomic_add_ret_veneer,
    .osal_atomic_sub_ret    = rt_atomic_sub_ret_veneer,
    .osal_atomic_inc_ret    = rt_atomic_inc_ret_veneer,
    .osal_atomic_dec_ret    = rt_atomic_dec_ret_veneer,
    .osal_dcache_clean      = rt_dcache_clean_veneer,
    .osal_dcache_invalidate = rt_dcache_invalidate_veneer,
};

