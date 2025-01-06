/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2012-09-20     Bernard      Change the name to components.c
 *                             And all components related header files.
 * 2012-12-23     Bernard      fix the pthread initialization issue.
 * 2013-06-23     Bernard      Add the init_call for components initialization.
 * 2013-07-05     Bernard      Remove initialization feature for MS VC++ compiler
 * 2015-02-06     Bernard      Remove the MS VC++ support and move to the kernel
 * 2015-05-04     Bernard      Rename it to components.c because compiling issue
 *                             in some IDEs.
 * 2015-07-29     Arda.Fu      Add support to use RT_USING_USER_MAIN with IAR
 */
#include <stdint.h>
#include <rthw.h>
#include <rtthread.h>
#if defined(RT_USING_PM)||defined(PSRAM_CACHE_WB)
    #include <register.h>
#endif

#ifdef RT_USING_USER_MAIN
    #ifndef RT_MAIN_THREAD_STACK_SIZE
        #define RT_MAIN_THREAD_STACK_SIZE     (1024*3)
    #endif
    #ifndef RT_MAIN_THREAD_PRIORITY
        #define RT_MAIN_THREAD_PRIORITY       (RT_THREAD_PRIORITY_MAX / 3)
    #endif
#endif

#ifdef RT_USING_COMPONENTS_INIT
/*
 * Components Initialization will initialize some driver and components as following
 * order:
 * rti_start         --> 0
 * BOARD_EXPORT      --> 1
 * rti_board_end     --> 1.end
 *
 * DEVICE_EXPORT     --> 2
 * COMPONENT_EXPORT  --> 3
 * FS_EXPORT         --> 4
 * ENV_EXPORT        --> 5
 * PRE_APP_EXPORT    --> 6
 * APP_EXPORT        --> 7
 *
 * rti_end           --> 7.end
 *
 * These automatically initialization, the driver or component initial function must
 * be defined with:
 * INIT_BOARD_EXPORT(fn);
 * INIT_DEVICE_EXPORT(fn);
 * ...
 * INIT_APP_EXPORT(fn);
 * etc.
 */
static int rti_start(void)
{
    return 0;
}
#ifndef _MSC_VER
    INIT_EXPORT(rti_start, "0", "0");
#else
    INIT_EXPORT(rti_start, STR_CONCAT(rti_fn$, 0, 0));
#endif

static int rti_board_start(void)
{
    return 0;
}
#ifndef _MSC_VER
    INIT_EXPORT(rti_board_start, "0", "0.end");
#else
    INIT_EXPORT(rti_board_start, STR_CONCAT(rti_fn$, 0, 0.end));
#endif

static int rti_board_end(void)
{
    return 0;
}
#ifndef _MSC_VER
    INIT_EXPORT(rti_board_end, "1", "9.end");
#else
    INIT_EXPORT(rti_board_end, STR_CONCAT(rti_fn$, 1, 9.end));
#endif

static int rti_end(void)
{
    return 0;
}
#ifndef _MSC_VER
    INIT_EXPORT(rti_end, "7", "9.end");
#else
    INIT_EXPORT(rti_end, STR_CONCAT(rti_fn$, 7, 9.end));
#endif

/**
 * RT-Thread Components Initialization for board
 */
__ROM_USED void rt_components_board_init(void)
{
#if RT_DEBUG_INIT
    int result;
    const struct rt_init_desc *desc;
    for (desc = &__rt_init_desc_rti_board_start; desc < &__rt_init_desc_rti_board_end; desc ++)
    {
        rt_kprintf("initialize %s", desc->fn_name);
        result = desc->fn();
        rt_kprintf(":%d done\n", result);
    }
#else
    const init_fn_t *fn_ptr;

    for (fn_ptr = &__rt_init_rti_board_start; fn_ptr < &__rt_init_rti_board_end; fn_ptr++)
    {
        (*fn_ptr)();
    }
#endif
}

/**
 * RT-Thread Components Initialization
 */
__ROM_USED void rt_components_init(void)
{
#if RT_DEBUG_INIT
    int result;
    const struct rt_init_desc *desc;

    rt_kprintf("do components initialization.\n");
    for (desc = &__rt_init_desc_rti_board_end; desc < &__rt_init_desc_rti_end; desc ++)
    {
        if (desc->fn_name && desc->fn)
        {
            rt_kprintf("initialize %s", desc->fn_name);
            result = desc->fn();
            rt_kprintf(":%d done\n", result);
        }
    }
#else
    const init_fn_t *fn_ptr;

    for (fn_ptr = &__rt_init_rti_board_end; fn_ptr < &__rt_init_rti_end; fn_ptr ++)
    {
        if (*fn_ptr)
        {
            (*fn_ptr)();
        }
    }
#endif
}

#ifdef RT_USING_USER_MAIN

void rt_application_init(void);
void rt_hw_board_init(void);
int rtthread_startup(void);

#if defined(__CC_ARM) || defined(__CLANG_ARM)
extern int $Super$$main(void);
RT_WEAK void mpu_reconfig(void)
{
}

/* re-define main function */
int $Sub$$main(void)
{
#ifdef PSRAM_CACHE_WB
    // If scatter loading copy code to PSRAM and XIP, need to clean DCache first.
    // Please note that this function itself could not be loaded to PSRAM in this case.
    SCB_CleanDCache();
#endif
    mpu_reconfig();
    rtthread_startup();
    return 0;
}
#elif defined(__ICCARM__)
extern int main(void);
/* __low_level_init will auto called by IAR cstartup */
extern void __iar_data_init3(void);
int __low_level_init(void)
{
    // call IAR table copy function.
    __iar_data_init3();
    rtthread_startup();
    return 0;
}
#elif defined(__GNUC__)
extern int main(void);
/* Add -eentry to arm-none-eabi-gcc argument */
int entry(void)
{
    rtthread_startup();
    return 0;
}
#endif

#ifdef RT_USING_PM
RT_WEAK void rt_application_init_power_on_mode(void)
{
}

#endif // RT_USING_PM

#ifndef RT_USING_HEAP
    /* if there is not enable heap, we should use static thread and stack. */
    ALIGN(8)
    #if defined(LCPU_MEM_OPTIMIZE) && (LB55X_CHIP_ID < 3)
        extern rt_uint8_t main_stack[RT_MAIN_THREAD_STACK_SIZE];
    #else
        static rt_uint8_t main_stack[RT_MAIN_THREAD_STACK_SIZE];
    #endif
    struct rt_thread main_thread;
#endif

#ifdef RT_USING_PM
#ifdef PM_STANDBY_ENABLE
#ifdef SOC_BF_Z0

typedef struct
{
    void (*wakeup_callback)(uint8_t *user_data);
    uint8_t *user_data;
    uint8_t is_used;
} deep_slepp_wakeup_t;

#ifdef __CC_ARM
    #pragma arm section rwdata="UNINITZI"
#elif defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
    #pragma clang section bss="OTHERZI"
#else
#endif

static deep_slepp_wakeup_t g_deep_sleep_wakeup[5];

#ifdef __CC_ARM
    #pragma arm section rwdata
#elif defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
    #pragma clang section bss=""
#else
#endif


rt_err_t deep_sleep_register(void (*wakeup_callback)(uint8_t *user_data), uint8_t *user_data)
{
    uint32_t i;
    rt_err_t ret = RT_EFULL;
    for (i = 0; i < 5; i++)
    {
        if (!g_deep_sleep_wakeup[i].is_used)
            break;
    }

    if (i < 5)
    {
        g_deep_sleep_wakeup[i].is_used = 1;
        g_deep_sleep_wakeup[i].wakeup_callback = wakeup_callback;
        g_deep_sleep_wakeup[i].user_data = user_data;
        ret = RT_EOK;
    }
    return ret;
}

extern void rt_system_timer_recovery(uint8_t *user_data);
void deep_sleep_wakeup_notify(void)
{
    uint32_t i;
    for (i = 0; i < 5; i++)
    {
        if (g_deep_sleep_wakeup[i].is_used && g_deep_sleep_wakeup[i].wakeup_callback)
        {
            g_deep_sleep_wakeup[i].wakeup_callback(g_deep_sleep_wakeup[i].user_data);
            //g_deep_sleep_wakeup[i].is_used = 0;
        }
    }
    // To avoid timer coming before recovery compelted, receovery timer in the last.
    rt_system_timer_recovery(NULL);
}
#endif // SOC_BF_Z0
#endif // PM_STANDBY_ENABLE
#endif // RT_USING_PM


__ROM_USED uint32_t rwip_get_hw_time_us_3(void)
{
#ifdef SOC_BF_Z0
    volatile uint32_t *time_hi = (volatile uint32_t *)0x4102001C;
    volatile uint32_t *time_lo = (volatile uint32_t *)0x41020020;
#else
    volatile uint32_t *time_hi = (volatile uint32_t *)0x5005001C;
    volatile uint32_t *time_lo = (volatile uint32_t *)0x50050020;
#endif
    uint32_t result;
    uint32_t t_hi;
    uint32_t t_lo;


    *time_hi = 0x80000000;

    while (*time_hi & 0x80000000)
    {
        //wait for time capture
    }

    t_hi = *time_hi;
    t_lo = *time_lo;

    result = (t_hi * (625 >> 1)) + ((624 - t_lo) >> 1);

    return result;
}

/* the system main thread */
__ROM_USED void main_thread_entry(void *parameter)
{
    extern int main(void);
    extern int $Super$$main(void);
#ifdef SOC_BF0_LCPU // No need switch priority
    uint8_t read_priority = RT_THREAD_PRIORITY_HIGH - 1;
    uint8_t ori_pri = rt_thread_self()->current_priority;
    rt_thread_control(rt_thread_self(), RT_THREAD_CTRL_CHANGE_PRIORITY, &read_priority);
#endif
    /* RT-Thread components initialization */
    rt_components_init();
#ifdef SOC_BF0_LCPU // No need switch priority
    rt_thread_control(rt_thread_self(), RT_THREAD_CTRL_CHANGE_PRIORITY, &ori_pri);
#endif

    /* invoke system main function */
#if defined(__CC_ARM) || defined(__CLANG_ARM)
    $Super$$main(); /* for ARMCC. */
#elif defined(__ICCARM__) || defined(__GNUC__)
    main();
#endif
}

__ROM_USED void rt_application_init(void)
{
    rt_thread_t tid;

#ifdef RT_USING_HEAP
    tid = rt_thread_create("main", main_thread_entry, RT_NULL,
                           RT_MAIN_THREAD_STACK_SIZE, RT_MAIN_THREAD_PRIORITY, RT_THREAD_TICK_DEFAULT * 2);
    RT_ASSERT(tid != RT_NULL);
#else
    rt_err_t result;

    tid = &main_thread;
    result = rt_thread_init(tid, "main", main_thread_entry, RT_NULL,
                            main_stack, sizeof(main_stack), RT_MAIN_THREAD_PRIORITY, RT_THREAD_TICK_DEFAULT * 2);
    RT_ASSERT(result == RT_EOK);

    /* if not define RT_USING_HEAP, using to eliminate the warning */
    (void)result;
#endif

    rt_thread_startup(tid);
}

__ROM_USED int rtthread_startup(void)
{
    rt_hw_interrupt_disable();

    /* board level initialization
     * NOTE: please initialize heap inside board initialization.
     */
#ifdef RT_USING_PM
    rt_application_init_power_on_mode();
#endif // RT_USING_PM

    rt_hw_board_init();
#ifdef RT_USING_PM
    if (SystemPowerOnModeGet() == 0)
#endif // RT_USING_PM
    {
        /* show RT-Thread version */
        rt_show_version();
    }

    /* timer system initialization */
    rt_system_timer_init();

    /* scheduler system initialization */
    rt_system_scheduler_init();

#ifdef RT_USING_SIGNALS
    /* signal system initialization */
    rt_system_signal_init();
#endif

    /* create init_thread */
    rt_application_init();

    /* timer thread initialization */
    rt_system_timer_thread_init();

    /* idle thread initialization */
    rt_thread_idle_init();

    /* start scheduler */
    rt_system_scheduler_start();

    /* never reach here */
    return 0;
}
#endif
#endif
