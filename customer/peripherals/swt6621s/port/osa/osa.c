/** @file osa.c
 *
 *  @brief OS Abstraction API
 *
 *  Copyright 2023 NXP
 *
 *  SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include <inttypes.h>
#include <stdio.h>
#include <osa.h>
#include <wmlog.h>

/** Check if cpu is in isr context
 *
 * \return bool value - true if cpu is in isr context
 */
bool is_isr_context(void)
{
#ifdef __CA7_REV
    return (0U != if (SystemGetIRQNestingLevel()))
#else /* __CA7_REV */
    return (0U != __get_IPSR());
#endif
    }

/*** OS Reader Writer Locks ***/
int OSA_RWLockCreate(osa_rw_lock_t *plock, const char *mutex_name, const char *lock_name)
{
    return OSA_RWLockCreateWithCB(plock, mutex_name, lock_name, NULL);
}

int OSA_RWLockCreateWithCB(osa_rw_lock_t *plock, const char *mutex_name, const char *lock_name, cb_fn r_fn)
{
    osa_status_t status;

    status = OSA_MutexCreate((osa_mutex_handle_t)plock->reader_mutex);
    if (status != KOSA_StatusSuccess)
    {
        return -WM_FAIL;
    }
    status = OSA_MutexCreate((osa_mutex_handle_t)plock->write_mutex);
    if (status != KOSA_StatusSuccess)
    {
        return -WM_FAIL;
    }
    status = OSA_SemaphoreCreateBinary((osa_semaphore_handle_t)plock->rw_lock);
    if (status != KOSA_StatusSuccess)
    {
        return -WM_FAIL;
    }
    OSA_SemaphorePost((osa_semaphore_handle_t)plock->rw_lock);
    plock->reader_count = 0;
    plock->reader_cb    = r_fn;
    return WM_SUCCESS;
}

int OSA_RWLockReadLock(osa_rw_lock_t *lock, unsigned int wait_time)
{
    int ret;
    osa_status_t status = OSA_MutexLock((osa_mutex_handle_t)lock->reader_mutex, osaWaitForever_c);
    if (status != KOSA_StatusSuccess)
    {
        return -WM_FAIL;
    }
    lock->reader_count++;
    if (lock->reader_count == 1U)
    {
        if (lock->reader_cb != NULL)
        {
            ret = lock->reader_cb(lock, wait_time);
            if (ret != WM_SUCCESS)
            {
                lock->reader_count--;
                (void)OSA_MutexUnlock((osa_mutex_handle_t)lock->reader_mutex);
                return ret;
            }
        }
        else
        {
            /* If  1 it is the first reader and
             * if writer is not active, reader will get access
             * else reader will block.
             */
            status = OSA_SemaphoreWait((osa_semaphore_handle_t)lock->rw_lock, wait_time);
            if (status != KOSA_StatusSuccess)
            {
                lock->reader_count--;
                (void)OSA_MutexUnlock((osa_mutex_handle_t)lock->reader_mutex);
                return -WM_FAIL;
            }
        }
    }
    (void)OSA_MutexUnlock((osa_mutex_handle_t)lock->reader_mutex);
    return WM_SUCCESS;
}

int OSA_RWLockReadUnlock(osa_rw_lock_t *lock)
{
    osa_status_t status = OSA_MutexLock((osa_mutex_handle_t)lock->reader_mutex, osaWaitForever_c);
    if (status != KOSA_StatusSuccess)
    {
        return -WM_FAIL;
    }
    lock->reader_count--;
    if (lock->reader_count == 0U)
    {
        /* This is last reader so
         * give a chance to writer now
         */
        (void)OSA_SemaphorePost((osa_semaphore_handle_t)lock->rw_lock);
    }
    (void)OSA_MutexUnlock((osa_mutex_handle_t)lock->reader_mutex);
    return WM_SUCCESS;
}

int OSA_RWLockWriteLock(osa_rw_lock_t *lock, unsigned int wait_time)
{
    osa_status_t status = OSA_SemaphoreWait((osa_semaphore_handle_t)lock->rw_lock, wait_time);
    if (status != KOSA_StatusSuccess)
    {
        return -WM_FAIL;
    }

    return WM_SUCCESS;
}

void OSA_RWLockWriteUnlock(osa_rw_lock_t *lock)
{
    (void)OSA_SemaphorePost((osa_semaphore_handle_t)lock->rw_lock);
}

void OSA_RWLockDestroy(osa_rw_lock_t *lock)
{
    lock->reader_cb = NULL;

    (void)OSA_SemaphoreDestroy((osa_semaphore_handle_t)lock->rw_lock);

    (void)OSA_MutexDestroy((osa_mutex_handle_t)lock->reader_mutex);

    (void)OSA_MutexDestroy((osa_mutex_handle_t)lock->write_mutex);

    lock->reader_count = 0;
}
void os_thread_sleep(uint32_t ticks)
{
    //os_dprintf("OS: Thread Sleep: %d\r\n", ticks);
    vTaskDelay(ticks);
    return;
}
int os_timer_deactivate(os_timer_t *timer_t)
{
    int ret;
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    if (timer_t == NULL || (*timer_t) == NULL)
    {
        return -WM_E_INVAL;
    }
    /* Note:
     * XTimerStop, seconds argument is xBlockTime which means, the time,
     * in ticks, that the calling task should be held in the Blocked
     * state, until timer command succeeds.
     * We are giving as 0, to be consistent with threadx logic.
     */
    if (is_isr_context())
    {
        /* This call is from Cortex-M3 handler mode, i.e. exception
         * context, hence use FromISR FreeRTOS APIs.
         */
        ret = xTimerStopFromISR(*timer_t, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR((bool)(xHigherPriorityTaskWoken));
    }
    else
    {
        ret = xTimerStop(*timer_t, 0);
    }
    return ret == pdPASS ? WM_SUCCESS : -WM_FAIL;
}
int os_timer_activate(os_timer_t *timer_t)
{
    int ret;
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    if (timer_t == NULL || (*timer_t) == NULL)
    {
        return -WM_E_INVAL;
    }

    /* Note:
     * XTimerStart, seconds argument is xBlockTime which means, the time,
     * in ticks, that the calling task should be held in the Blocked
     * state, until timer command succeeds.
     * We are giving as 0, to be consistent with threadx logic.
     */
    if (is_isr_context() != 0U)
    {
        /* This call is from Cortex-M3 handler mode, i.e. exception
         * context, hence use FromISR FreeRTOS APIs.
         */
        ret = xTimerStartFromISR(*timer_t, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR((bool)(xHigherPriorityTaskWoken));
    }
    else
    {
        ret = xTimerStart(*timer_t, 0);
    }
    return ret == pdPASS ? WM_SUCCESS : -WM_FAIL;
}

int os_timer_create_t(os_timer_t *timer_t,
                      const char *name,
                      osa_timer_tick ticks,
                      void (*call_back)(osa_timer_arg_t xTimer),
                      void *cb_arg,
                      os_timer_reload_t reload,
                      osa_timer_activate_t activate)
{
    int auto_reload = (reload == OS_TIMER_ONE_SHOT) ? pdFALSE : pdTRUE;

    *timer_t = xTimerCreate(name, ticks, (UBaseType_t)auto_reload, cb_arg, call_back);
    if (*timer_t == NULL)
    {
        return -WM_FAIL;
    }

    if (activate == OS_TIMER_AUTO_ACTIVATE)
    {
        return os_timer_activate(timer_t);
    }

    return WM_SUCCESS;
}