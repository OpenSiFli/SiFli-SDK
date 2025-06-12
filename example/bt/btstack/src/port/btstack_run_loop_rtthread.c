#define BTSTACK_FILE__ "btstack_run_loop_rtthread.c"

#include <stddef.h> // NULL

#include "btstack_run_loop_rtthread.h"

#include "btstack_debug.h"
#include "btstack_linked_list.h"
#include "btstack_util.h"
#include "hal_time_ms.h"

// some SDKs, e.g. esp-idf, place FreeRTOS headers into an 'rtthread' folder to
// avoid name collisions (e.g. list.h, queue.h, ..) wih this flag, the headers
// are properly found

#include <rtthread.h>

typedef struct function_call
{
    void (*fn)(void *arg);
    void *arg;
} function_call_t;

// queue to receive events: up to 2 calls from transport, rest for app
#define RUN_LOOP_QUEUE_LENGTH 20
#define RUN_LOOP_QUEUE_ITEM_SIZE sizeof(function_call_t)

static rt_mq_t btstack_run_loop_queue;
static rt_thread_t btstack_run_loop_task;
static rt_mutex_t btstack_run_loop_callbacks_mutex;

static rt_event_t btstack_run_loop_event_group;

// bit 0 event group reserved to wakeup run loop
#define EVENT_GROUP_FLAG_RUN_LOOP 1

// the run loop
static bool run_loop_exit_requested;

static uint32_t btstack_run_loop_rtthread_get_time_ms(void)
{
    return hal_time_ms();
}

// set timer
static void btstack_run_loop_rtthread_set_timer(btstack_timer_source_t *ts,
                                                uint32_t timeout_in_ms)
{
    ts->timeout = btstack_run_loop_rtthread_get_time_ms() + timeout_in_ms + 1;
}

static void btstack_run_loop_rtthread_trigger_from_thread(void)
{
    rt_event_send(btstack_run_loop_event_group, EVENT_GROUP_FLAG_RUN_LOOP);
}

static void btstack_run_loop_rtthread_poll_data_sources_from_irq(void)
{
    btstack_run_loop_rtthread_trigger_from_thread();
#if 0
    BaseType_t xHigherPriorityTaskWoken;

    xEventGroupSetBitsFromISR(btstack_run_loop_event_group,
                              EVENT_GROUP_FLAG_RUN_LOOP,
                              &xHigherPriorityTaskWoken);
#endif
}

static void btstack_run_loop_rtthread_trigger_exit_internal(void)
{
    run_loop_exit_requested = true;
}

/**
 * Execute run_loop
 */
static void btstack_run_loop_rtthread_execute(void)
{
    log_debug("RL: execute");

    run_loop_exit_requested = false;

    while (true)
    {

        // process data sources
        btstack_run_loop_base_poll_data_sources();

        // execute callbacks - protect list with mutex
        while (1)
        {
            rt_mutex_take(btstack_run_loop_callbacks_mutex, RT_WAITING_FOREVER);
            btstack_context_callback_registration_t *callback_registration =
                (btstack_context_callback_registration_t *)
                    btstack_linked_list_pop(&btstack_run_loop_base_callbacks);
            rt_mutex_release(btstack_run_loop_callbacks_mutex);
            if (callback_registration == NULL)
            {
                break;
            }
            (*callback_registration->callback)(callback_registration->context);
        }

        // process registered function calls on run loop thread (deprecated)
        while (true)
        {
            function_call_t message = {NULL, NULL};
            rt_err_t res = rt_mq_recv(btstack_run_loop_queue, &message,
                                      RUN_LOOP_QUEUE_ITEM_SIZE, RT_WAITING_NO);
            if (res)
                break;
            if (message.fn)
            {
                message.fn(message.arg);
            }
        }

        // process timers
        uint32_t now = btstack_run_loop_rtthread_get_time_ms();
        btstack_run_loop_base_process_timers(now);

        // exit triggered by btstack_run_loop_trigger_exit (main thread or other
        // thread)
        if (run_loop_exit_requested)
            break;

        // wait for timeout or event group/task notification
        int32_t timeout_next_timer_ms =
            btstack_run_loop_base_get_time_until_timeout(now);

        uint32_t timeout_ms = RT_WAITING_FOREVER;
        if (timeout_next_timer_ms >= 0)
        {
            timeout_ms =
                (uint32_t)timeout_next_timer_ms * RT_TICK_PER_SECOND / 1000;
        }

        log_debug("RL: wait with timeout %u", (int)timeout_ms);

        rt_event_recv(btstack_run_loop_event_group, EVENT_GROUP_FLAG_RUN_LOOP,
                      RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, timeout_ms, NULL);
    }
}

static void btstack_run_loop_rtthread_execute_on_main_thread(
    btstack_context_callback_registration_t *callback_registration)
{
    // protect list with mutex
    rt_mutex_take(btstack_run_loop_callbacks_mutex, RT_WAITING_FOREVER);
    btstack_run_loop_base_add_callback(callback_registration);
    rt_mutex_release(btstack_run_loop_callbacks_mutex);
    btstack_run_loop_rtthread_trigger_from_thread();
}

static void btstack_run_loop_rtthread_init(void)
{
    btstack_run_loop_base_init();

    btstack_run_loop_queue =
        rt_mq_create("bt_queue", RUN_LOOP_QUEUE_ITEM_SIZE,
                     RUN_LOOP_QUEUE_LENGTH, RT_IPC_FLAG_FIFO);

    btstack_run_loop_callbacks_mutex =
        rt_mutex_create("bt_mutex", RT_IPC_FLAG_FIFO);

    // event group to wake run loop
    btstack_run_loop_event_group =
        rt_event_create("player_event", RT_IPC_FLAG_FIFO);

    // task to handle to optimize 'run on main thread'
    btstack_run_loop_task = rt_thread_self();

    log_info("run loop init, task %p, queue item size %u",
             btstack_run_loop_task, (int)sizeof(function_call_t));
}

/**
 * @brief Provide btstack_run_loop_posix instance for use with
 * btstack_run_loop_init
 */

static const btstack_run_loop_t btstack_run_loop_rtthread = {
    &btstack_run_loop_rtthread_init,
    &btstack_run_loop_base_add_data_source,
    &btstack_run_loop_base_remove_data_source,
    &btstack_run_loop_base_enable_data_source_callbacks,
    &btstack_run_loop_base_disable_data_source_callbacks,
    &btstack_run_loop_rtthread_set_timer,
    &btstack_run_loop_base_add_timer,
    &btstack_run_loop_base_remove_timer,
    &btstack_run_loop_rtthread_execute,
    &btstack_run_loop_base_dump_timer,
    &btstack_run_loop_rtthread_get_time_ms,
    &btstack_run_loop_rtthread_poll_data_sources_from_irq,
    &btstack_run_loop_rtthread_execute_on_main_thread,
    &btstack_run_loop_rtthread_trigger_exit_internal,
};

const btstack_run_loop_t *btstack_run_loop_rtthread_get_instance(void)
{
    return &btstack_run_loop_rtthread;
}

// @deprecated functions

// schedules execution from regular thread
void btstack_run_loop_rtthread_trigger(void)
{
    btstack_run_loop_rtthread_trigger_from_thread();
}

void btstack_run_loop_rtthread_execute_code_on_main_thread(
    void (*fn)(void *arg), void *arg)
{
    // directly call function if already on btstack task
    if (rt_thread_self() == btstack_run_loop_task)
    {
        (*fn)(arg);
        return;
    }

    function_call_t message;
    message.fn = fn;
    message.arg = arg;
    rt_err_t res = rt_mq_urgent(btstack_run_loop_queue, &message,
                                RUN_LOOP_QUEUE_ITEM_SIZE);
    if (res)
    {
        log_error("Failed to post fn %p", fn);
    }
    btstack_run_loop_rtthread_trigger();
}

void btstack_run_loop_rtthread_trigger_exit(void)
{
    btstack_run_loop_rtthread_trigger_exit_internal();
}

void btstack_run_loop_rtthread_trigger_from_isr(void)
{
    btstack_run_loop_rtthread_poll_data_sources_from_irq();
}
