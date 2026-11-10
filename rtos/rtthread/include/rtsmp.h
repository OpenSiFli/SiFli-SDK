/* SPDX-License-Identifier: Apache-2.0 */
#ifndef __RTSMP_H__
#define __RTSMP_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef RT_USING_SMP
    #define RT_CPUS_NR 2
    #define RT_CPU_DETACHED 0xff
    #define RT_THREAD_CTRL_BIND_CPU 0x04

/* Place application code/constants at addresses reachable by both cores.
 * Referenced functions and data need their own shared placement. */
    #define RT_SMP_TEXT __attribute__((section(".smp_text")))
    #define RT_SMP_RODATA __attribute__((section(".smp_rodata")))

struct rt_cpu
{
    rt_thread_t current_thread;
    rt_uint16_t lock_depth;
    rt_uint16_t critical_depth;
    rt_base_t critical_irq;
    volatile rt_uint8_t interrupt_nest;
    volatile rt_uint8_t online;
};

extern struct rt_cpu rt_cpu_table[RT_CPUS_NR];

unsigned rt_hw_cpu_id(void);
/* Internal: hold local IRQ masking or the kernel lock while using this
 * pointer. It must not survive a migration/preemption point. */
struct rt_cpu *rt_cpu_self(void);
void rt_hw_smp_notify(unsigned cpu);
void rt_hw_smp_pend(void);
void *rt_smp_switch(void *saved_sp);
void rt_smp_tick(void);
rt_err_t rt_smp_start_secondary(void);
rt_err_t rt_smp_bind(rt_thread_t thread, unsigned cpu);
void rt_smp_idle_init(void);
/* Internal defunct-list publication; does not change the application API. */
void rt_smp_reaper_notify(void);
#else
    #define RT_SMP_TEXT
    #define RT_SMP_RODATA
#endif /* RT_USING_SMP */

#ifdef __cplusplus
}
#endif

#endif /* __RTSMP_H__ */
