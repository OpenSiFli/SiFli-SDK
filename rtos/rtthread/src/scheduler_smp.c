/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 * SPDX-License-Identifier: Apache-2.0
 * Experimental RT-Thread 3.1.3 SMP scheduler, SF32LB57 HCPU/ACPU.
 * Keep the SDK's READY-includes-running model and public thread fields.
 * Selection happens in PendSV after the complete outgoing frame is saved.
 */
#include <rthw.h>
#include <rtsmp.h>

#ifdef RT_USING_SMP
struct rt_cpu rt_cpu_table[RT_CPUS_NR];
rt_list_t rt_thread_priority_table[RT_THREAD_PRIORITY_MAX];
rt_list_t rt_thread_defunct;
rt_uint32_t rt_thread_ready_priority_group;
    #if RT_THREAD_PRIORITY_MAX > 32
rt_uint8_t rt_thread_ready_table[32];
    #endif

static struct rt_thread idle_secondary;
static rt_uint8_t idle_stack[1024] __attribute__((aligned(8)));

    #ifdef RT_USING_HOOK
static void (*scheduler_hook)(rt_thread_t, rt_thread_t);

void rt_scheduler_sethook(void (*hook)(rt_thread_t, rt_thread_t))
{
    rt_base_t level = rt_cpus_lock();
    scheduler_hook = hook;
    rt_cpus_unlock(level);
}
    #endif

struct rt_cpu *rt_cpu_self(void)
{
    return &rt_cpu_table[rt_hw_cpu_id()];
}

void rt_system_scheduler_init(void)
{
    for (unsigned i = 0; i < RT_THREAD_PRIORITY_MAX; ++i)
    {
        rt_list_init(&rt_thread_priority_table[i]);
    }
    rt_list_init(&rt_thread_defunct);
}

void rt_system_scheduler_start(void)
{
    RT_ASSERT(rt_cpu_self()->lock_depth == 0);
    rt_cpu_self()->online = 1;
    rt_hw_context_switch_to(0);
}

void rt_schedule(void)
{
    if (rt_cpu_self()->online)
    {
        rt_hw_smp_pend();
    }
}

/** Called with local IRQs off on the exception stack. No kernel lock can be
 * carried through a switch: acquiring that lock also masks local interrupts. */
void *rt_smp_switch(void *saved_sp)
{
    rt_base_t level = rt_cpus_lock();
    unsigned cpu = rt_hw_cpu_id();
    struct rt_cpu *c = &rt_cpu_table[cpu];
    rt_thread_t from = c->current_thread, next = RT_NULL;
    if (from)
    {
        from->sp = saved_sp;
        RT_ASSERT(from->oncpu == cpu);
        from->oncpu = RT_CPU_DETACHED;
    }
    for (unsigned p = 0; p < RT_THREAD_PRIORITY_MAX && !next; ++p)
    {
        rt_list_t *head = &rt_thread_priority_table[p];
        for (rt_list_t *n = head->next; n != head; n = n->next)
        {
            rt_thread_t t = rt_list_entry(n, struct rt_thread, tlist);
            if (t->oncpu == RT_CPU_DETACHED &&
                (t->bind_cpu == cpu || t->bind_cpu == RT_CPU_DETACHED))
            {
                next = t;
                break;
            }
        }
    }
    RT_ASSERT(next != RT_NULL);
    next->oncpu = cpu;
    c->current_thread = next;
    if (next != from)
    {
    #ifdef RT_USING_OVERFLOW_CHECK
        RT_ASSERT(*((rt_uint8_t *)next->stack_addr) == '#');
        RT_ASSERT((rt_ubase_t)next->sp >= (rt_ubase_t)next->stack_addr);
        RT_ASSERT((rt_ubase_t)next->sp < (rt_ubase_t)next->stack_addr + next->stack_size);
    #endif
    #ifdef RT_USING_HOOK
        if (from && scheduler_hook)
        {
            scheduler_hook(from, next);
        }
    #endif
    }
    void *sp = next->sp;
    /* An outgoing unbound/retargeted thread becomes eligible only now. Wake
     * the peer again to cover an IPI received before this save completed. */
    if (from && from != next && (from->stat & RT_THREAD_STAT_MASK) == RT_THREAD_READY)
    {
        rt_hw_smp_notify(cpu ^ 1u);
    }
    rt_cpus_unlock(level);
    return sp;
}

void rt_schedule_insert_thread(struct rt_thread *thread)
{
    rt_base_t level = rt_cpus_lock();
    thread->stat = RT_THREAD_READY | (thread->stat & ~RT_THREAD_STAT_MASK);
    rt_list_insert_before(&rt_thread_priority_table[thread->current_priority], &thread->tlist);
    #if RT_THREAD_PRIORITY_MAX > 32
    rt_thread_ready_table[thread->number] |= thread->high_mask;
    #endif
    rt_thread_ready_priority_group |= thread->number_mask;
    unsigned peer = rt_hw_cpu_id() ^ 1u;
    if (thread->bind_cpu == peer || thread->bind_cpu == RT_CPU_DETACHED)
    {
        rt_hw_smp_notify(peer);
    }
    rt_cpus_unlock(level);
}

void rt_schedule_remove_thread(struct rt_thread *thread)
{
    rt_base_t level = rt_cpus_lock();
    rt_list_remove(&thread->tlist);
    if (rt_list_isempty(&rt_thread_priority_table[thread->current_priority]))
    {
    #if RT_THREAD_PRIORITY_MAX > 32
        rt_thread_ready_table[thread->number] &= ~thread->high_mask;
        if (!rt_thread_ready_table[thread->number])
    #endif
        {
            rt_thread_ready_priority_group &= ~thread->number_mask;
        }
    }
    if (thread->oncpu != RT_CPU_DETACHED && thread->oncpu != rt_hw_cpu_id())
    {
        rt_hw_smp_notify(thread->oncpu);
    }
    rt_cpus_unlock(level);
}

/** The SDK also uses scheduler critical sections for shared object/heap
 * traversal. This port holds the global lock and masks
 * IRQs until the outer exit; callers must keep these sections nonblocking. */
void rt_enter_critical(void)
{
    rt_base_t level = rt_cpus_lock();
    struct rt_cpu *c = rt_cpu_self();
    if (c->critical_depth++ == 0)
    {
        c->critical_irq = level;
    }
}

void rt_exit_critical(void)
{
    struct rt_cpu *c = rt_cpu_self();
    RT_ASSERT(c->critical_depth);
    rt_base_t level = --c->critical_depth ? 1 : c->critical_irq;
    if (!c->critical_depth)
    {
        rt_schedule();
    }
    rt_cpus_unlock(level);
}

rt_uint16_t rt_critical_level(void)
{
    rt_base_t level = rt_hw_local_irq_disable();
    rt_uint16_t depth = rt_cpu_self()->critical_depth;
    rt_hw_local_irq_enable(level);
    return depth;
}
RTM_EXPORT(rt_enter_critical);
RTM_EXPORT(rt_exit_critical);
RTM_EXPORT(rt_critical_level);

rt_err_t rt_smp_bind(rt_thread_t thread, unsigned cpu)
{
    if (!thread || (cpu >= RT_CPUS_NR && cpu != RT_CPU_DETACHED))
    {
        return -RT_EINVAL;
    }
    rt_base_t level = rt_cpus_lock();
    thread->bind_cpu = cpu;
    rt_hw_smp_notify(rt_hw_cpu_id() ^ 1u);
    rt_schedule();
    rt_cpus_unlock(level);
    return RT_EOK;
}

void rt_smp_tick(void)
{
    rt_base_t level = rt_cpus_lock();
    struct rt_cpu *c = rt_cpu_self();
    rt_thread_t t = c->current_thread;
    if (t && --t->remaining_tick == 0)
    {
        t->remaining_tick = t->init_tick;
        rt_thread_yield();
    }
    rt_cpus_unlock(level);
}

static void _secondary_idle(void *arg)
{
    (void)arg;
    for (;;)
    {
        rt_hw_cpu_idle();
    }
}

void rt_smp_idle_init(void)
{
    rt_thread_init(&idle_secondary, "tidle1", _secondary_idle, RT_NULL,
                   idle_stack, sizeof(idle_stack), RT_THREAD_PRIORITY_MAX - 1, 32);
    idle_secondary.bind_cpu = 1;
    rt_thread_startup(&idle_secondary);
}
#endif /* RT_USING_SMP */
