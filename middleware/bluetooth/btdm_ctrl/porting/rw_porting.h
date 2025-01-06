/**
 ****************************************************************************************
 *
 * @file plf.h
 *
 * @brief This file contains the definitions of the macros and functions that are
 * platform dependent.  The implementation of those is implemented in the
 * appropriate platform directory.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */
#ifndef _RW_PORTING_H_
#define _RW_PORTING_H_

#ifndef WIN32
    #include "cmsis_compiler.h"
#endif

#ifdef BSP_USING_RTTHREAD
#include <stdint.h>
#include <rtthread.h>
#include <rthw.h>

#define GLOBAL_INT_DISABLE();                                               \
do {                                                                        \
    uint32_t __old;                                                         \
    __old = rt_hw_interrupt_disable();                                  \

/** @brief Restore interrupts from the previous global disable.
 * @sa GLOBAL_INT_DISABLE
 */
#define GLOBAL_INT_RESTORE();                                               \
    rt_hw_interrupt_enable(__old);                                      \
} while(0)


#define RW_ASSERT(EX) RT_ASSERT(EX)
#endif

#ifdef BSP_USING_NO_OS
#define GLOBAL_INT_DISABLE();                                               \
do {                                                                        \
    __disable_irq();                                  \


#define GLOBAL_INT_RESTORE();                                               \
    __enable_irq();                                      \
} while(0)

#define RW_ASSERT(EX)
#endif

void *rw_os_thread_create(int8_t *name, void *task_entry, void *parameter, uint8_t int_priority);
void *rw_os_sem_create(int8_t *name, int value);
void rw_os_sem_set(void *event);
void rw_os_sem_get(void *event);

void *rw_os_timer_create(int8_t *name, void *timeout_cbk, void *param, int ms);
void rw_os_timer_start(void *timer);
void rw_os_timer_del(void *timer);

uint64_t rw_os_time_get(void);
void rw_os_delay_ms(int ms);

void rw_os_init(void);

void rw_os_tl_reconfig_for_flush(uint8_t idx);
extern void *rw_os_get_if;

#if defined(BF0_HCPU)
    void bluetooth_wakeup_lcpu(void);
    void bluetooth_release_lcpu(void);
#endif

#endif // _PLF_H_

