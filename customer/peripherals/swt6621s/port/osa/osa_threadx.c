/** @file osa_threadx.c
 *
 *  @brief OS Threadx interaction APIs
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

#if defined(FSL_RTOS_THREADX)

/*! @brief Converts milliseconds to ticks*/
#define MSEC_TO_TICK(msec) \
    (((uint32_t)(msec) + 500uL / (uint32_t)TX_TIMER_TICKS_PER_SECOND) * (uint32_t)TX_TIMER_TICKS_PER_SECOND / 1000uL)

#if !CONFIG_MEM_POOLS

/**
 * These are example addresses for the purposes of compiling.
 * The addresses used should be externally defined in another file for your system
 */
#define HEAP_START 0x10008000
#define HEAP_END   0x1000FFFF

/*
 * In this example, we treat malloc as a function pointer
 * we initialize the value as init_malloc.
 * The first function to call malloc() will take the hit on initializing the pool
 * Afterwards, all calls to malloc() pass through to do_malloc().
 */
typedef void *(*malloc_ptr_t)(size_t);

/*
 * In this example, I am using the compiler's builtin atomic compare and swap
 * Routine. This will provide atomic access to swapping the malloc pointer,
 * and only one function will initialize the memory pool.
 */
#define atomic_compare_and_swap __sync_val_compare_and_swap

static void *init_malloc(size_t size);
static void *do_malloc(size_t size);

// ThreadX internal memory pool stucture
static TX_BYTE_POOL malloc_pool_ = {0};

/*
 * Flag that is used in do_malloc() to cause competing threads to wait until
 * initialization is completed before allocating memory.
 */
volatile static bool initialized_ = false;

/**
 * Note well that I have redefined 'malloc' to be a volatile function pointer.
 * This will cause compiler errors if you include the compiler's stdlib.h.
 * You will need to redefine another header for malloc/free/etc. (e.g. memory.h)
 * or write your own string.h/stdlib.h headers that will be included over the compiler's version
 *
 * This also means you need to compile with -fno-builtin and link with -nodefaultlibs
 */
volatile malloc_ptr_t malloc = &init_malloc;

/*
 * init_malloc must be called before memory allocation calls are made
 * This sets up a byte pool for the heap using the defined HEAP_START and HEAP_END macros
 * Size is passed to do_malloc and allocated to the caller
 */

void *init_malloc(size_t size)
{
    RT_ASSERT(size > 0);

    uintptr_t heap_start = (uintptr_t)HEAP_START;
    uintptr_t heap_size  = (uintptr_t)HEAP_END - heap_start;

    /**
     * When we enter into init_malloc, we check the current value of the malloc pointer
     * If it's still init_malloc, we swap the value to do_malloc and return true.
     * If the value is do_malloc, another thread beat us to the punch and we fall
     * through to do_malloc(), skipping initialization.
     */
    if (atomic_compare_and_swap(&malloc, &init_malloc, &do_malloc))
    {
        uint8_t r;

        /**
         * This is ThreadX's API to create a byte pool using a memory block.
         * We are essentially just wrapping ThreadX APIs into a simpler form
         */
        r = tx_byte_pool_create(&malloc_pool_, "Heap Memory Pool", (void *)heap_start, heap_size);
        RT_ASSERT(r == TX_SUCCESS);

        // Signal to any threads waiting on do_malloc that we are done
        initialized_ = true;
    }

    /*
     * Remember - two situations happen here:
     *  1) malloc initialized on the first call, and then passed through to normal malloc
     *  2) two threads raced to init. One initializes, and the other falls through to malloc
     */
    return do_malloc(size);
}

void *OSA_MemoryAllocate(uint32_t size)
{
    void *ptr = NULL;

    /**
     * Since multiple threads could be racing to malloc, if we lost the race
     * we need to make sure the ThreadX pool has been created before we
     * try to allocate memory, or there will be an error
     */
    while (!initialized_)
    {
        tx_thread_sleep(1);
    }

    if (size > 0)
    {
        // We simply wrap the threadX call into a standard form
        uint8_t r = tx_byte_allocate(&malloc_pool_, &ptr, size, TX_WAIT_FOREVER);

        // I add the string to provide a more helpful error output.  It's value is always true.
        RT_ASSERT(r == TX_SUCCESS && "malloc failed");
    } // else NULL if there was an error

    return ptr;
}

void OSA_MemoryFree(void *ptr)
{
    // free should NEVER be called before malloc is init'd
    RT_ASSERT(initialized_);

    if (ptr)
    {
        // We simply wrap the threadX call into a standard form
        uint8_t r = tx_byte_release(ptr);
        ptr       = NULL;
        RT_ASSERT(r == TX_SUCCESS);
    }
}

#endif

/*** Timer Management ***/
osa_status_t OSA_TimerCreate(osa_timer_handle_t timerHandle,
                             osa_timer_tick ticks,
                             void (*call_back)(osa_timer_arg_t),
                             void *cb_arg,
                             osa_timer_t reload,
                             osa_timer_activate_t activate)
{
    int auto_activate = (activate == OSA_TIMER_NO_ACTIVATE) ? TX_NO_ACTIVATE : TX_AUTO_ACTIVATE;

    osa_status_t status = KOSA_StatusSuccess;

    RT_ASSERT(timerHandle != NULL);

    struct timer_data *ptimer = (struct timer_data *)timerHandle;

    ptimer->callback = call_back;
    ptimer->user_arg = cb_arg;

    if (TX_SUCCESS != tx_timer_create((TX_TIMER *)&ptimer->timer, NULL, (void (*)(ULONG))call_back, (ULONG)cb_arg,
                                      (ULONG)ticks, reload == KOSA_TimerOnce ? 0 : (ULONG)ticks, auto_activate))
    {
        status = KOSA_StatusError;
    }

    return status;
}

osa_status_t OSA_TimerActivate(osa_timer_handle_t timerHandle)
{
    osa_status_t status = KOSA_StatusSuccess;

    RT_ASSERT(timerHandle != NULL);

    struct timer_data *ptimer = (struct timer_data *)timerHandle;

    if (TX_SUCCESS != tx_timer_activate((TX_TIMER *)&ptimer->timer))
    {
        status = KOSA_StatusError;
    }

    return status;
}

osa_status_t OSA_TimerChange(osa_timer_handle_t timerHandle, osa_timer_tick ntime, osa_timer_tick block_time)

{
    osa_status_t status = KOSA_StatusSuccess;
    CHAR *name;
    UINT active;
    ULONG remaining_ticks;
    ULONG reschedule_ticks;
    TX_TIMER *next_timer;

    RT_ASSERT(timerHandle != NULL);

    struct timer_data *ptimer = (struct timer_data *)timerHandle;

    if (TX_SUCCESS !=
            tx_timer_info_get((TX_TIMER *)&ptimer->timer, &name, &active, &remaining_ticks, &reschedule_ticks, &next_timer))
    {
        return KOSA_StatusError;
    }

    if (TX_SUCCESS !=
            tx_timer_change((TX_TIMER *)&ptimer->timer, (ULONG)ntime, reschedule_ticks == 0 ? 0 : (ULONG)ntime))
    {
        status = KOSA_StatusError;
    }

    return status;
}

bool OSA_TimerIsRunning(osa_timer_handle_t timerHandle)
{
    //    osa_status_t status = KOSA_StatusSuccess;
    CHAR *name;
    UINT active;
    ULONG remaining_ticks;
    ULONG reschedule_ticks;
    TX_TIMER *next_timer;

    RT_ASSERT(timerHandle != NULL);

    struct timer_data *ptimer = (struct timer_data *)timerHandle;

    if (TX_SUCCESS !=
            tx_timer_info_get((TX_TIMER *)&ptimer->timer, &name, &active, &remaining_ticks, &reschedule_ticks, &next_timer))
    {
        return false;
    }

    return (active == TX_TRUE ? true : false);
}

void *OSA_TimerGetContext(osa_timer_handle_t timerHandle)
{
    RT_ASSERT(timerHandle != NULL);

    struct timer_data *ptimer = (struct timer_data *)timerHandle;

    return ptimer->user_arg;
}

osa_status_t OSA_TimerReset(osa_timer_handle_t timerHandle)
{
    osa_status_t status = KOSA_StatusSuccess;

    RT_ASSERT(timerHandle != NULL);

    struct timer_data *ptimer = (struct timer_data *)timerHandle;

    if (TX_SUCCESS != tx_timer_deactivate((TX_TIMER *)&ptimer->timer))
    {
        return KOSA_StatusError;
    }

    if (TX_SUCCESS != tx_timer_activate((TX_TIMER *)&ptimer->timer))
    {
        status = KOSA_StatusError;
    }

    return status;
}

osa_status_t OSA_TimerDeactivate(osa_timer_handle_t timerHandle)
{
    osa_status_t status = KOSA_StatusSuccess;

    RT_ASSERT(timerHandle != NULL);

    struct timer_data *ptimer = (struct timer_data *)timerHandle;

    if (TX_SUCCESS != tx_timer_deactivate((TX_TIMER *)&ptimer->timer))
    {
        status = KOSA_StatusError;
    }

    return status;
}

osa_status_t OSA_TimerDestroy(osa_timer_handle_t timerHandle)
{
    osa_status_t status = KOSA_StatusSuccess;

    RT_ASSERT(timerHandle != NULL);

    struct timer_data *ptimer = (struct timer_data *)timerHandle;

    if (TX_SUCCESS != tx_timer_delete((TX_TIMER *)&ptimer->timer))
    {
        status = KOSA_StatusError;
    }

    return status;
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

int OSA_SetupIdleFunction(void (*func)(void))
{
    return 0;
}

int OSA_RemoveIdleFunction(void (*func)(void))
{
    return 0;
}

void OSA_ThreadSelfComplete(osa_task_handle_t taskHandle)
{
    return 0;
}

uint32_t OSA_MsgQWaiting(osa_msgq_handle_t msgqHandle)
{
    return 0;
}

#endif
