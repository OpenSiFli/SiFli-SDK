/** @file os.c
 *
 *  @brief OS interaction API
 *
 *  Copyright 2008-2022 NXP
 *
 *  SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include <inttypes.h>
#include <stdio.h>
#include <wm_os.h>
#include <wmlog.h>

#define mainTEST_TASK_PRIORITY (tskIDLE_PRIORITY)
#define mainTEST_DELAY         (400 / portTICK_PERIOD_MS)

void vApplicationIdleHook(void);
void os_thread_stackmark(char *name);
int os_event_flags_delete(event_group_handle_t *hnd);

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

/** Get current OS tick counter value
 *
 * \return 32 bit value of ticks since boot-up
 */
unsigned os_ticks_get(void)
{
    if (is_isr_context())
    {
        return xTaskGetTickCountFromISR();
    }
    else
    {
        return xTaskGetTickCount();
    }
}

uint32_t os_msec_to_ticks(uint32_t msecs)
{
    return (msecs) / (portTICK_PERIOD_MS);
}

unsigned long os_ticks_to_msec(unsigned long ticks)
{
    return (ticks) * (portTICK_PERIOD_MS);
}

/*** Thread Management ***/
const char *get_current_taskname(void)
{
    os_thread_t handle = xTaskGetCurrentTaskHandle();
    if (handle != NULL)
    {
        return pcTaskGetName(handle);
    }
    else
    {
        return "Unknown";
    }
}

int os_thread_create(os_thread_t *thandle,
                     const char *name,
                     void (*main_func)(os_thread_arg_t arg),
                     void *arg,
                     os_thread_stack_t *stack,
                     int prio)
{
    int ret;

    ret = xTaskCreate(main_func, name, (uint16_t)stack->size, arg, (uint32_t)prio, thandle);

    os_dprintf(
        " Thread Create: ret %d thandle %p"
        " stacksize = %d\r\n",
        ret, thandle ? *thandle : NULL, stack->size);
    return ret == pdPASS ? WM_SUCCESS : -WM_FAIL;
}

os_thread_t os_get_current_task_handle(void)
{
    return xTaskGetCurrentTaskHandle();
}

int os_thread_delete(os_thread_t *thandle)
{
    if (thandle == NULL)
    {
        os_dprintf("OS: Thread Self Delete\r\n");
        vTaskDelete(NULL);
    }
    else
    {
        os_dprintf("OS: Thread Delete: %p\r\n", *thandle);
        vTaskDelete(*thandle);
    }

    *thandle = NULL;

    return WM_SUCCESS;
}

void os_thread_sleep(uint32_t ticks)
{
    os_dprintf("OS: Thread Sleep: %d\r\n", ticks);
    vTaskDelay(ticks);
    return;
}

void os_thread_self_complete(os_thread_t *thandle)
{
    /* Suspend self until someone calls delete. This is required because in
     * freeRTOS, main functions of a thread cannot return.
     */
    if (thandle != NULL)
    {
        os_dprintf("OS: Thread Complete: %p\r\n", *thandle);
        vTaskSuspend(*thandle);
    }
    else
    {
        os_dprintf("OS: Thread Complete: SELF\r\n");
        vTaskSuspend(NULL);
    }

    /*
     * We do not want this function to return ever.
     */
    while (true)
    {
        os_thread_sleep(os_msec_to_ticks(60000));
    }
}

/*** Timer Management ***/
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

int os_timer_create(os_timer_t *timer_t,
                    const char *name,
                    os_timer_tick ticks,
                    void (*call_back)(os_timer_arg_t xTimer),
                    void *cb_arg,
                    os_timer_reload_t reload,
                    os_timer_activate_t activate)
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

int os_timer_change(os_timer_t *timer_t, os_timer_tick ntime, os_timer_tick block_time)
{
    int ret;
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    if (timer_t == NULL || (*timer_t) == NULL)
    {
        return -WM_E_INVAL;
    }
    if (is_isr_context())
    {
        /* This call is from Cortex-M3 handler mode, i.e. exception
         * context, hence use FromISR FreeRTOS APIs.
         */
        ret = xTimerChangePeriodFromISR(*timer_t, ntime, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR((bool)(xHigherPriorityTaskWoken));
    }
    else
    {
        /* Fixme: What should be value of xBlockTime? */
        ret = xTimerChangePeriod(*timer_t, ntime, 100);
    }
    return ret == pdPASS ? WM_SUCCESS : -WM_FAIL;
}

bool os_timer_is_running(os_timer_t *timer_t)
{
    int ret;

    if (timer_t == NULL || (*timer_t) == NULL)
    {
        return false;
    }

    ret = xTimerIsTimerActive(*timer_t);
    return ret == pdPASS ? true : false;
}

void *os_timer_get_context(os_timer_t *timer_t)
{
    if (timer_t == NULL || (*timer_t) == NULL)
    {
        os_dprintf("OS: Failed to get timer context\r\n");
        return NULL;
    }

    return pvTimerGetTimerID(*timer_t);
}

int os_timer_reset(os_timer_t *timer_t)
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
        ret = xTimerResetFromISR(*timer_t, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR((bool)(xHigherPriorityTaskWoken));
    }
    else
    {
        ret = xTimerReset(*timer_t, 0);
    }
    return ret == pdPASS ? WM_SUCCESS : -WM_FAIL;
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

int os_timer_delete(os_timer_t *timer_t)
{
    int ret;

    if (timer_t == NULL || (*timer_t) == NULL)
    {
        return -WM_E_INVAL;
    }

    /* Below timer handle invalidation needs to be protected as a context
     * switch may create issues if same handle is used before
     * invalidation.
     */
    unsigned long sta = os_enter_critical_section();
    /* Note: Block time is set as 0, thus signifying non-blocking
       API. Can be changed later if required. */
    ret      = xTimerDelete(*timer_t, 0);
    *timer_t = NULL;
    os_exit_critical_section(sta);

    return ret == pdPASS ? WM_SUCCESS : -WM_FAIL;
}

/*** Os Queue Functions ***/
int os_queue_create(os_queue_t *qhandle, const char *name, int msgsize, os_queue_pool_t *poolname)
{
    /** The size of the pool divided by the max. message size gives the
     * max. number of items in the queue. */
    os_dprintf(" Queue Create: name = %s poolsize = %d msgsize = %d\r\n", name, poolname->size, msgsize);
    *qhandle = xQueueCreate((UBaseType_t)(poolname->size / msgsize), (UBaseType_t)msgsize);
    os_dprintf(" Queue Create: handle %p\r\n", *qhandle);
    if (*qhandle != NULL)
    {
        return WM_SUCCESS;
    }
    return -WM_FAIL;
}

int os_queue_send(os_queue_t *qhandle, const void *msg, unsigned long wait)
{
    int ret;
    signed portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    if (qhandle == NULL || (*qhandle) == NULL)
    {
        return -WM_E_INVAL;
    }

    os_dprintf("OS: Queue Send: handle %p, msg %p, wait %d\r\n", *qhandle, msg, wait);

    if (is_isr_context())
    {
        /* This call is from Cortex-M3 handler mode, i.e. exception
         * context, hence use FromISR FreeRTOS APIs.
         */
        ret = xQueueSendToBackFromISR(*qhandle, msg, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR((bool)(xHigherPriorityTaskWoken));
    }
    else
    {
        ret = xQueueSendToBack(*qhandle, msg, wait);
    }
    os_dprintf("OS: Queue Send: done\r\n");

    return ret == pdTRUE ? WM_SUCCESS : -WM_FAIL;
}

int os_queue_recv(os_queue_t *qhandle, void *msg, unsigned long wait)
{
    int ret;
    if (qhandle == NULL || (*qhandle) == NULL)
    {
        return -WM_E_INVAL;
    }

    os_dprintf("OS: Queue Receive: handle %p, msg %p, wait %d\r\n", *qhandle, msg, wait);
    ret = xQueueReceive(*qhandle, msg, wait);
    os_dprintf("OS: Queue Receive: done\r\n");
    return ret == pdTRUE ? WM_SUCCESS : -WM_FAIL;
}

int os_queue_delete(os_queue_t *qhandle)
{
    os_dprintf("OS: Queue Delete: handle %p\r\n", *qhandle);

    vQueueDelete(*qhandle);
    // sem_debug_delete((const xSemaphoreHandle)*qhandle);
    *qhandle = NULL;

    return WM_SUCCESS;
}

int os_queue_get_msgs_waiting(os_queue_t *qhandle)
{
    int nmsg = 0;
    if (qhandle == NULL || (*qhandle) == NULL)
    {
        return -WM_E_INVAL;
    }
    nmsg = (int)uxQueueMessagesWaiting(*qhandle);
    os_dprintf("OS: Queue Msg Count: handle %p, count %d\r\n", *qhandle, nmsg);
    return nmsg;
}

void (*g_os_tick_hooks[MAX_CUSTOM_HOOKS])(void) = {NULL};
void (*g_os_idle_hooks[MAX_CUSTOM_HOOKS])(void) = {NULL};

/** The FreeRTOS Tick hook function. */
void vApplicationTickHook(void)
{
    int i;

    for (i = 0; i < MAX_CUSTOM_HOOKS; i++)
    {
        if (g_os_tick_hooks[i] != NULL)
        {
            g_os_tick_hooks[i]();
        }
    }
}

void vApplicationIdleHook(void)
{
    int i;
    for (i = 0; i < MAX_CUSTOM_HOOKS; i++)
    {
        if (g_os_idle_hooks[i] != NULL)
        {
            g_os_idle_hooks[i]();
        }
    }
}

/* Freertos handles this internally? */
void os_thread_stackmark(char *name)
{
    /* Nothing to-do */
}

int os_setup_idle_function(void (*func)(void))
{
    unsigned int i;

    for (i = 0; i < MAX_CUSTOM_HOOKS; i++)
    {
        if (g_os_idle_hooks[i] != NULL && g_os_idle_hooks[i] == func)
        {
            return WM_SUCCESS;
        }
    }

    for (i = 0; i < MAX_CUSTOM_HOOKS; i++)
    {
        if (g_os_idle_hooks[i] == NULL)
        {
            g_os_idle_hooks[i] = func;
            break;
        }
    }

    if (i == MAX_CUSTOM_HOOKS)
    {
        return -WM_FAIL;
    }

    return WM_SUCCESS;
}

int os_setup_tick_function(void (*func)(void))
{
    unsigned int i;

    for (i = 0; i < MAX_CUSTOM_HOOKS; i++)
    {
        if (g_os_tick_hooks[i] != NULL && g_os_tick_hooks[i] == func)
        {
            return WM_SUCCESS;
        }
    }

    for (i = 0; i < MAX_CUSTOM_HOOKS; i++)
    {
        if (g_os_tick_hooks[i] == NULL)
        {
            g_os_tick_hooks[i] = func;
            break;
        }
    }

    if (i == MAX_CUSTOM_HOOKS)
    {
        return -WM_FAIL;
    }

    return WM_SUCCESS;
}

int os_remove_idle_function(void (*func)(void))
{
    unsigned int i;

    for (i = 0; i < MAX_CUSTOM_HOOKS; i++)
    {
        if (g_os_idle_hooks[i] == func)
        {
            g_os_idle_hooks[i] = NULL;
            break;
        }
    }

    if (i == MAX_CUSTOM_HOOKS)
    {
        return -WM_FAIL;
    }

    return WM_SUCCESS;
}

int os_remove_tick_function(void (*func)(void))
{
    unsigned int i;

    for (i = 0; i < MAX_CUSTOM_HOOKS; i++)
    {
        if (g_os_tick_hooks[i] == func)
        {
            g_os_tick_hooks[i] = NULL;
            break;
        }
    }

    if (i == MAX_CUSTOM_HOOKS)
    {
        return -WM_FAIL;
    }

    return WM_SUCCESS;
}

/*** Mutex ***/
int os_mutex_create(os_mutex_t *mhandle, const char *name, int flags)
{
    if (flags == OS_MUTEX_NO_INHERIT)
    {
        *mhandle = NULL;
        os_dprintf("Cannot create mutex for non-inheritance yet \r\n");
        return -WM_FAIL;
    }
    os_dprintf("OS: Mutex Create: name = %s \r\n", name);
    *mhandle = xSemaphoreCreateMutex();
    os_dprintf("OS: Mutex Create: handle = %p\r\n", *mhandle);
    if (*mhandle != NULL)
    {
        // sem_debug_add((const xQueueHandle)*mhandle,
        //        name, 1);
        return WM_SUCCESS;
    }
    else
    {
        return -WM_FAIL;
    }
}

int os_mutex_get(os_mutex_t *mhandle, unsigned long wait)
{
    int ret;
    if (mhandle == NULL || (*mhandle) == NULL)
    {
        return -WM_E_INVAL;
    }
    os_dprintf("OS: Mutex Get: handle %p\r\n", *mhandle);
    ret = xSemaphoreTake(*mhandle, wait);
    return ret == pdTRUE ? WM_SUCCESS : -WM_FAIL;
}

int os_mutex_put(os_mutex_t *mhandle)
{
    int ret;

    if (mhandle == NULL || (*mhandle) == NULL)
    {
        return -WM_E_INVAL;
    }

    os_dprintf("OS: Mutex Put: %p\r\n", *mhandle);

    ret = xSemaphoreGive(*mhandle);
    return ret == pdTRUE ? WM_SUCCESS : -WM_FAIL;
}

int os_recursive_mutex_create(os_mutex_t *mhandle, const char *name)
{
    if (mhandle == NULL)
    {
        return -WM_E_INVAL;
    }

    os_dprintf("OS: Recursive Mutex Create: name = %s \r\n", name);
    *mhandle = xSemaphoreCreateRecursiveMutex();
    os_dprintf("OS: Recursive Mutex Create: handle = %p\r\n", *mhandle);
    if (*mhandle == NULL)
    {
        return -WM_FAIL;
    }

    // sem_debug_add(*mhandle, name, 1);
    return WM_SUCCESS;
}

int os_recursive_mutex_get(os_mutex_t *mhandle, unsigned long wait)
{
    os_dprintf("OS: Recursive Mutex Get: handle %p\r\n", *mhandle);
    int ret = xSemaphoreTakeRecursive(*mhandle, wait);
    return ret == pdTRUE ? WM_SUCCESS : -WM_FAIL;
}

int os_recursive_mutex_put(os_mutex_t *mhandle)
{
    os_dprintf("OS: Recursive Mutex Put: %p\r\n", *mhandle);
    int ret = xSemaphoreGiveRecursive(*mhandle);
    return ret == pdTRUE ? WM_SUCCESS : -WM_FAIL;
}

int os_mutex_delete(os_mutex_t *mhandle)
{
    vSemaphoreDelete(*mhandle);
    // sem_debug_delete((const xSemaphoreHandle)*mhandle);
    *mhandle = NULL;
    return WM_SUCCESS;
}

/*** Event ***/
typedef struct event_wait_t
{
    /* parameter passed in the event get call */
    unsigned thread_mask;
    /* The 'get' thread will wait on this sem */
    os_semaphore_t sem;
    struct event_wait_t *next;
    struct event_wait_t *prev;
} event_wait_t;

typedef struct event_group_t
{
    /* Main event flags will be stored here */
    unsigned flags;
    /* This flag is used to indicate deletion
     * of event group */
    bool delete_group;
    /* to protect this structure and the waiting list */
    os_mutex_t mutex;
    event_wait_t *list;
} event_group_t;

static inline void os_event_flags_remove_node(event_wait_t *node, event_group_t *grp_ptr)
{
    if (node->prev != NULL)
    {
        node->prev->next = node->next;
    }
    if (node->next != NULL)
    {
        node->next->prev = node->prev;
    }
    /* If only one node is present */
    if (node->next == NULL && node->prev == NULL)
    {
        grp_ptr->list = NULL;
    }
    os_mem_free(node);
}

int os_event_flags_create(event_group_handle_t *hnd)
{
    int ret;
    event_group_t *eG = os_mem_alloc(sizeof(event_group_t));
    if (eG == NULL)
    {
        os_dprintf("ERROR:Mem allocation\r\n");
        return -WM_FAIL;
    }
    (void)memset(eG, 0x00, sizeof(event_group_t));
    ret = os_mutex_create(&eG->mutex, "event-flag", OS_MUTEX_INHERIT);
    if (ret != WM_SUCCESS)
    {
        os_mem_free(eG);
        return -WM_FAIL;
    }
    *hnd = (event_group_handle_t)eG;
    return WM_SUCCESS;
}

int os_event_flags_get(event_group_handle_t hnd,
                       unsigned requested_flags,
                       flag_rtrv_option_t option,
                       unsigned *actual_flags_ptr,
                       unsigned wait_option)
{
    bool wait_done = false;
    unsigned status;
    int ret;
    *actual_flags_ptr = 0;
    event_wait_t *tmp = NULL, *node = NULL;

    if (hnd == 0U)
    {
        os_dprintf("ERROR:Invalid event flag handle\r\n");
        return -WM_FAIL;
    }
    if (requested_flags == 0U)
    {
        os_dprintf("ERROR:Requested flag is zero\r\n");
        return -WM_FAIL;
    }
    if (actual_flags_ptr == NULL)
    {
        os_dprintf("ERROR:Flags pointer is NULL\r\n");
        return -WM_FAIL;
    }
    event_group_t *eG = (event_group_t *)hnd;

    while (true)
    {
        (void)os_mutex_get(&eG->mutex, OS_WAIT_FOREVER);

        if ((option == EF_AND) || (option == EF_AND_CLEAR))
        {
            if ((eG->flags & requested_flags) == requested_flags)
            {
                status = eG->flags;
            }
            else
            {
                status = 0;
            }
        }
        else if ((option == EF_OR) || (option == EF_OR_CLEAR))
        {
            status = (requested_flags & eG->flags);
        }
        else
        {
            os_dprintf("ERROR:Invalid event flag get option\r\n");
            (void)os_mutex_put(&eG->mutex);
            return -WM_FAIL;
        }
        /* Check flags */
        if (status != 0U)
        {
            *actual_flags_ptr = status;

            /* Clear the requested flags from main flag */
            if ((option == EF_AND_CLEAR) || (option == EF_OR_CLEAR))
            {
                eG->flags &= ~status;
            }

            if (wait_done)
            {
                /*Delete the created semaphore */
                (void)os_semaphore_delete(&tmp->sem);
                /* Remove ourselves from the list */
                os_event_flags_remove_node(tmp, eG);
            }
            (void)os_mutex_put(&eG->mutex);
            return WM_SUCCESS;
        }
        else
        {
            if (wait_option != 0U)
            {
                if (wait_done == false)
                {
                    /* Add to link list */
                    /* Prepare a node to add in the link list */
                    node = os_mem_alloc(sizeof(event_wait_t));
                    if (node == NULL)
                    {
                        os_dprintf("ERROR:memory alloc\r\n");
                        (void)os_mutex_put(&eG->mutex);
                        return -WM_FAIL;
                    }
                    (void)memset(node, 0x00, sizeof(event_wait_t));
                    /* Set the requested flag in the node */
                    node->thread_mask = requested_flags;
                    /* Create a semaophore */
                    ret = os_semaphore_create(&node->sem, "wait_thread");
                    if (ret != 0)
                    {
                        os_dprintf("ERROR:In creating semaphore\r\n");
                        os_mem_free(node);
                        (void)os_mutex_put(&eG->mutex);
                        return -WM_FAIL;
                    }
                    /* If there is no node present */
                    if (eG->list == NULL)
                    {
                        eG->list = node;
                        tmp      = eG->list;
                    }
                    else
                    {
                        tmp = eG->list;
                        /* Move to last node */
                        while (tmp->next != NULL)
                        {
                            os_dprintf("waiting \r\n");
                            tmp = tmp->next;
                        }
                        tmp->next  = node;
                        node->prev = tmp;
                        tmp        = tmp->next;
                    }
                    /* Take semaphore first time */
                    ret = os_semaphore_get(&tmp->sem, OS_WAIT_FOREVER);
                    if (ret != WM_SUCCESS)
                    {
                        os_dprintf("ERROR:1st sem get error\r\n");
                        (void)os_mutex_put(&eG->mutex);
                        /*Delete the created semaphore */
                        (void)os_semaphore_delete(&tmp->sem);
                        /* Remove ourselves from the list */
                        os_event_flags_remove_node(tmp, eG);
                        return -WM_FAIL;
                    }
                }
                (void)os_mutex_put(&eG->mutex);
                /* Second time get is performed for work-around purpose
                as in current implementation of semaphore 1st request
                is always satisfied */
                ret = os_semaphore_get(&tmp->sem, os_msec_to_ticks(wait_option));
                if (ret != WM_SUCCESS)
                {
                    (void)os_mutex_get(&eG->mutex, OS_WAIT_FOREVER);
                    /*Delete the created semaphore */
                    (void)os_semaphore_delete(&tmp->sem);
                    /* Remove ourselves from the list */
                    os_event_flags_remove_node(tmp, eG);
                    (void)os_mutex_put(&eG->mutex);
                    return EF_NO_EVENTS;
                }

                /* We have woken up */
                /* If the event group deletion has been requested */
                if (eG->delete_group)
                {
                    (void)os_mutex_get(&eG->mutex, OS_WAIT_FOREVER);
                    /*Delete the created semaphore */
                    (void)os_semaphore_delete(&tmp->sem);
                    /* Remove ourselves from the list */
                    os_event_flags_remove_node(tmp, eG);
                    (void)os_mutex_put(&eG->mutex);
                    return -WM_FAIL;
                }
                wait_done = true;
                continue;
            }
            else
            {
                (void)os_mutex_put(&eG->mutex);
                return EF_NO_EVENTS;
            }
        }
    } /* while(true) */
}

int os_event_flags_set(event_group_handle_t hnd, unsigned flags_to_set, flag_rtrv_option_t option)
{
    event_wait_t *tmp = NULL;

    if (hnd == 0U)
    {
        os_dprintf("ERROR:Invalid event flag handle\r\n");
        return -WM_FAIL;
    }
    if (flags_to_set == 0U)
    {
        os_dprintf("ERROR:Flags to be set is zero\r\n");
        return -WM_FAIL;
    }

    event_group_t *eG = (event_group_t *)hnd;

    (void)os_mutex_get(&eG->mutex, OS_WAIT_FOREVER);

    /* Set flags according to the set_option */
    if (option == EF_OR)
    {
        eG->flags |= flags_to_set;
    }
    else if (option == EF_AND)
    {
        eG->flags &= flags_to_set;
    }
    else
    {
        os_dprintf("ERROR:Invalid flag set option\r\n");
        (void)os_mutex_put(&eG->mutex);
        return -WM_FAIL;
    }

    if (eG->list != NULL)
    {
        tmp = eG->list;
        if (tmp->next == NULL)
        {
            if ((tmp->thread_mask & eG->flags) != 0U)
            {
                (void)os_semaphore_put(&tmp->sem);
            }
        }
        else
        {
            while (tmp != NULL)
            {
                if ((tmp->thread_mask & eG->flags) != 0U)
                {
                    (void)os_semaphore_put(&tmp->sem);
                }
                tmp = tmp->next;
            }
        }
    }
    (void)os_mutex_put(&eG->mutex);
    return WM_SUCCESS;
}

int os_event_flags_delete(event_group_handle_t *hnd)
{
    int i, max_attempt = 3;
    event_wait_t *tmp = NULL;

    if (*hnd == 0U)
    {
        os_dprintf("ERROR:Invalid event flag handle\r\n");
        return -WM_FAIL;
    }
    event_group_t *eG = (event_group_t *)*hnd;

    (void)os_mutex_get(&eG->mutex, OS_WAIT_FOREVER);

    /* Set the flag to delete the group */
    eG->delete_group = 1;

    if (eG->list != NULL)
    {
        tmp = eG->list;
        if (tmp->next == NULL)
        {
            (void)os_semaphore_put(&tmp->sem);
        }
        else
        {
            while (tmp != NULL)
            {
                (void)os_semaphore_put(&tmp->sem);
                tmp = tmp->next;
            }
        }
    }
    (void)os_mutex_put(&eG->mutex);

    /* If still list is not empty then wait for 3 seconds */
    for (i = 0; i < max_attempt; i++)
    {
        (void)os_mutex_get(&eG->mutex, OS_WAIT_FOREVER);
        if (eG->list != NULL)
        {
            (void)os_mutex_put(&eG->mutex);
            os_thread_sleep(os_msec_to_ticks(1000));
        }
        else
        {
            (void)os_mutex_put(&eG->mutex);
            break;
        }
    }

    (void)os_mutex_get(&eG->mutex, OS_WAIT_FOREVER);
    if (eG->list != NULL)
    {
        (void)os_mutex_put(&eG->mutex);
        return -WM_FAIL;
    }
    else
    {
        (void)os_mutex_put(&eG->mutex);
    }

    /* Delete the event group */
    os_mem_free(eG);
    *hnd = 0;
    return WM_SUCCESS;
}

/*** Event Notification ***/

int os_event_notify_get(unsigned long wait_time)
{
    int ret = (int)ulTaskNotifyTake(pdTRUE, wait_time);
    return ret == pdTRUE ? WM_SUCCESS : -WM_FAIL;
}

int os_event_notify_put(os_thread_t task)
{
    int ret                                       = pdTRUE;
    signed portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    if (task == NULL)
    {
        return -WM_E_INVAL;
    }
    if (is_isr_context())
    {
        /* This call is from Cortex-M3/4 handler mode, i.e. exception
         * context, hence use FromISR FreeRTOS APIs.
         */
        vTaskNotifyGiveFromISR(task, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR((bool)(xHigherPriorityTaskWoken));
    }
    else
    {
        ret = xTaskNotifyGive(task);
    }

    return ret == pdTRUE ? WM_SUCCESS : -WM_FAIL;
}

/*** Semaphore ***/

int os_semaphore_create(os_semaphore_t *mhandle, const char *name)
{
    vSemaphoreCreateBinary(*mhandle);
    if (*mhandle != NULL)
    {
        // sem_debug_add((const xSemaphoreHandle)*mhandle,
        //        name, 1);
        return WM_SUCCESS;
    }
    else
    {
        return -WM_FAIL;
    }
}

int os_semaphore_create_counting(os_semaphore_t *mhandle,
                                 const char *name,
                                 unsigned long maxcount,
                                 unsigned long initcount)
{
    *mhandle = xSemaphoreCreateCounting(maxcount, initcount);
    if (*mhandle != NULL)
    {
        ////sem_debug_add((const xQueueHandle)*mhandle,
        //        name, 1);
        return WM_SUCCESS;
    }
    else
    {
        return -WM_FAIL;
    }
}

int os_semaphore_get(os_semaphore_t *mhandle, unsigned long wait)
{
    int ret;
    signed portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    if (mhandle == NULL || (*mhandle) == NULL)
    {
        return -WM_E_INVAL;
    }
    os_dprintf("OS: Semaphore Get: handle %p\r\n", *mhandle);
    if (is_isr_context())
    {
        /* This call is from Cortex-M3 handler mode, i.e. exception
         * context, hence use FromISR FreeRTOS APIs.
         */
        ret = xSemaphoreTakeFromISR(*mhandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR((bool)(xHigherPriorityTaskWoken));
    }
    else
    {
        ret = xSemaphoreTake(*mhandle, wait);
    }
    return ret == pdTRUE ? WM_SUCCESS : -WM_FAIL;
}

int os_semaphore_put(os_semaphore_t *mhandle)
{
    int ret;
    signed portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    if (mhandle == NULL || (*mhandle) == NULL)
    {
        return -WM_E_INVAL;
    }

    os_dprintf("OS: Semaphore Put: handle %p\r\n", *mhandle);
    if (is_isr_context())
    {
        /* This call is from Cortex-M3 handler mode, i.e. exception
         * context, hence use FromISR FreeRTOS APIs.
         */
        ret = xSemaphoreGiveFromISR(*mhandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR((bool)(xHigherPriorityTaskWoken));
    }
    else
    {
        ret = xSemaphoreGive(*mhandle);
    }
    return ret == pdTRUE ? WM_SUCCESS : -WM_FAIL;
}

int os_semaphore_getcount(os_semaphore_t *mhandle)
{
    os_dprintf("OS: Semaphore Get Count: handle %p\r\n", *mhandle);
    return (int)uxQueueMessagesWaiting(*mhandle);
}

int os_semaphore_delete(os_semaphore_t *mhandle)
{
    vSemaphoreDelete(*mhandle);
    // sem_debug_delete((const xSemaphoreHandle)*mhandle);
    *mhandle = NULL;
    return WM_SUCCESS;
}

/*** OS Reader Writer Locks ***/
int os_rwlock_create(os_rw_lock_t *plock, const char *mutex_name, const char *lock_name)
{
    return os_rwlock_create_with_cb(plock, mutex_name, lock_name, NULL);
}
int os_rwlock_create_with_cb(os_rw_lock_t *plock, const char *mutex_name, const char *lock_name, cb_fn r_fn)
{
    int ret = WM_SUCCESS;
    ret     = os_mutex_create(&(plock->reader_mutex), mutex_name, OS_MUTEX_INHERIT);
    if (ret == -WM_FAIL)
    {
        return -WM_FAIL;
    }
    ret     = os_mutex_create(&(plock->write_mutex), mutex_name, OS_MUTEX_INHERIT);
    if (ret == -WM_FAIL)
    {
        return -WM_FAIL;
    }
    ret = os_semaphore_create(&(plock->rw_lock), lock_name);
    if (ret == -WM_FAIL)
    {
        return -WM_FAIL;
    }
    plock->reader_count = 0;
    plock->reader_cb    = r_fn;
    return ret;
}

int os_rwlock_read_lock(os_rw_lock_t *lock, unsigned int wait_time)
{
    int ret = WM_SUCCESS;
    ret     = os_mutex_get(&(lock->reader_mutex), OS_WAIT_FOREVER);
    if (ret != WM_SUCCESS)
    {
        return ret;
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
                (void)os_mutex_put(&(lock->reader_mutex));
                return ret;
            }
        }
        else
        {
            /* If  1 it is the first reader and
             * if writer is not active, reader will get access
             * else reader will block.
             */
            ret = os_semaphore_get(&(lock->rw_lock), wait_time);
            if (ret != WM_SUCCESS)
            {
                lock->reader_count--;
                (void)os_mutex_put(&(lock->reader_mutex));
                return ret;
            }
        }
    }
    (void)os_mutex_put(&(lock->reader_mutex));
    return ret;
}

int os_rwlock_read_unlock(os_rw_lock_t *lock)
{
    int ret = os_mutex_get(&(lock->reader_mutex), OS_WAIT_FOREVER);
    if (ret == -WM_FAIL)
    {
        return ret;
    }
    lock->reader_count--;
    if (lock->reader_count == 0U)
    {
        /* This is last reader so
         * give a chance to writer now
         */
        (void)os_semaphore_put(&(lock->rw_lock));
    }
    (void)os_mutex_put(&(lock->reader_mutex));
    return ret;
}

int os_rwlock_write_lock(os_rw_lock_t *lock, unsigned int wait_time)
{
    int ret = os_semaphore_get(&(lock->rw_lock), wait_time);
    return ret;
}

void os_rwlock_write_unlock(os_rw_lock_t *lock)
{
    (void)os_semaphore_put(&(lock->rw_lock));
}

void os_rwlock_delete(os_rw_lock_t *lock)
{
    lock->reader_cb = NULL;
    if (lock->rw_lock)
        (void)os_semaphore_delete(&(lock->rw_lock));
    if (lock->reader_mutex)
        (void)os_mutex_delete(&(lock->reader_mutex));
    if (lock->write_mutex)
        os_mutex_delete(&(lock->write_mutex));
    lock->reader_count = 0;
}

/* returns time in micro-secs since time began */
unsigned int os_get_timestamp(void)
{
    uint32_t nticks;
    uint32_t counter;

    vPortEnterCritical();
    nticks  = xTaskGetTickCount();
    counter = SysTick->VAL;

    /*
     * If this is called after SysTick counter
     * expired but before SysTick Handler got a
     * chance to run, then set the return value
     * to the start of next tick.
     */
    if ((SCB->ICSR & SCB_ICSR_PENDSTSET_Msk) != 0U)
    {
        nticks++;
        counter = CNTMAX;
    }

    vPortExitCritical();
    return ((CNTMAX - counter) / CPU_CLOCK_TICKSPERUSEC) + (nticks * USECSPERTICK);
}

/* OS Memory allocation API's */
#ifndef CONFIG_HEAP_DEBUG
void *os_mem_alloc(size_t size)
{
    void *ptr = pvPortMalloc(size);
    return ptr;
}

void *os_mem_calloc(size_t size)
{
    void *ptr = pvPortMalloc(size);
    if (ptr != NULL)
    {
        (void)memset(ptr, 0x00, size);
    }
    else
    {
        /* Do Nothing */
    }

    return ptr;
}

void os_mem_free(void *ptr)
{
    vPortFree(ptr);
}
#else  /* ! CONFIG_HEAP_DEBUG */
extern int os_mem_alloc_cnt;
extern void record_os_mem_alloc(unsigned int size, char const *func, unsigned int line_num);

static void *os_mem_alloc_priv(unsigned int size, char const *func, unsigned int line_num)
{
    void *ptr = pvPortMalloc(size);

    os_mem_alloc_cnt++;
    record_os_mem_alloc(size, func, line_num);

    return ptr;
}

void *os_mem_alloc(size_t size)
{
    void *ptr = os_mem_alloc_priv((size), __func__, __LINE__);
    if (ptr != NULL)
    {
        (void)PRINTF("MDC:A:%x:%d\r\n", ptr, size);
    }
    else
    {
        /* Do Nothing */
    }

    return ptr;
}

void *os_mem_calloc(size_t size)
{
    void *ptr = pvPortMalloc(size);
    if (ptr != NULL)
    {
        (void)memset(ptr, 0x00, size);
        (void)PRINTF("MDC:A:%x:%d\r\n", ptr, size);
    }
    else
    {
        /* Do Nothing */
    }

    return ptr;
}

extern int os_mem_free_cnt;
extern void record_os_mem_free(char const *func, unsigned int line_num);

static void os_mem_free_priv(void *ptr, char const *func, unsigned int line_num)
{
    vPortFree(ptr);

    os_mem_free_cnt++;
    record_os_mem_free(func, line_num);
}

void os_mem_free(void *ptr)
{
    os_mem_free_priv((ptr), __func__, __LINE__);
    (void)PRINTF("MDC:F:%x\r\n", ptr);
}
#endif /* CONFIG_HEAP_DEBUG */

#ifdef CONFIG_HEAP_STAT
/** This function dumps complete statistics
 *  of the heap memory.
 */
void os_dump_mem_stats(void)
{
    unsigned sta = os_enter_critical_section();
    HeapStats_t HS;

    HS.xAvailableHeapSpaceInBytes      = 0;
    HS.xSizeOfLargestFreeBlockInBytes  = 0;
    HS.xSizeOfSmallestFreeBlockInBytes = 0;
    HS.xNumberOfFreeBlocks             = 0;
    HS.xNumberOfSuccessfulAllocations  = 0;
    HS.xNumberOfSuccessfulFrees        = 0;
    HS.xMinimumEverFreeBytesRemaining  = 0;

    vPortGetHeapStats(&HS);

    os_exit_critical_section(sta);

    (void)PRINTF("\n\r");
    (void)PRINTF("Heap size ---------------------- : %d\n\r", HS.xAvailableHeapSpaceInBytes);
    (void)PRINTF("Largest Free Block size -------- : %d\n\r", HS.xSizeOfLargestFreeBlockInBytes);
    (void)PRINTF("Smallest Free Block size ------- : %d\n\r", HS.xSizeOfSmallestFreeBlockInBytes);
    (void)PRINTF("Number of Free Blocks ---------- : %d\n\r", HS.xNumberOfFreeBlocks);
    (void)PRINTF("Total successful allocations --- : %d\n\r", HS.xNumberOfSuccessfulAllocations);
    (void)PRINTF("Total successful frees --------- : %d\n\r", HS.xNumberOfSuccessfulFrees);
    (void)PRINTF("Min Free since system boot ----- : %d\n\r", HS.xMinimumEverFreeBytesRemaining);
}
#endif

/** Disables all interrupts at NVIC level */
void os_disable_all_interrupts(void)
{
    taskDISABLE_INTERRUPTS();
}

/** Enable all interrupts at NVIC lebel */
void os_enable_all_interrupts(void)
{
    taskENABLE_INTERRUPTS();
}

