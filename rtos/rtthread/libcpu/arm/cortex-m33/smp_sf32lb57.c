/* SPDX-License-Identifier: Apache-2.0 */
#include <rtconfig.h>

#ifdef RT_USING_SMP
    #if !defined(SF32LB57X) || !defined(SOC_BF0_HCPU)
        #error This experimental SMP port requires SF32LB57 HCPU and GCC.
    #endif
    #if defined(RT_USING_PM) || defined(RT_USING_SIGNALS) || defined(RT_USING_MODULE) || \
        defined(RT_USING_LWP) || defined(RT_USING_TIMER_SOFT) || \
        defined(RT_SCHEDULER_LOCK_DEBUG) || defined(ACPU_CALLER_ENABLED)
        #error Experimental SMP excludes PM, signals, modules, LWP, soft timers, scheduler debug and the AMP ACPU service.
    #endif

    #include "bf0_hal.h"
    #include <rthw.h>
    #include <rtsmp.h>

    #if defined(RT_SMP_CACHE_SYNC) && defined(PSRAM_CACHE_WB)
        #error WT synchronization cannot coexist with writable write-back mappings.
    #endif

    #define SMP_SRAM_CODE __attribute__((section(".l1_ret_text_smp"), noinline))
    #define SMP_IPI_BIT (1u << 15)
    #define SMP_IPI_IRQN ((IRQn_Type)80)
    #define SMP_IPI_PRIORITY 2
    #define SMP_MAILBOX_LOCK (1u << 31)
    #define SMP_VECTOR_COUNT 256
    #define SMP_VECTOR_ALIGNMENT 1024
    #define SMP_EXCEPTION_VECTOR_COUNT 16
    #define SMP_IRQ_STACK_SIZE 4096

static uint32_t ap_vectors[SMP_VECTOR_COUNT] __attribute__((aligned(SMP_VECTOR_ALIGNMENT)));
static uint8_t ap_irq_stack[SMP_IRQ_STACK_SIZE] __attribute__((aligned(8)));
static volatile uint32_t ap_ready;
/* Written only by the ACPU fault handler; retained for debugger inspection.
 * This path cannot call HCPU-only logging code or take the kernel lock. */
static volatile uint32_t ap_fault_cfsr;
static volatile uint32_t ap_fault_hfsr;
static volatile uint32_t ap_fault_pc;
static volatile uint32_t ap_fault_lr;

    #ifdef RT_SMP_SRAM_WT_SYNC
extern unsigned char __smp_sram_cache_start__[];
    #endif

    #ifdef RT_SMP_CACHE_SYNC
/* These flags and per-CPU kernel state MUST remain in noncached SRAM.
 * All flag changes are protected by C4EXR, so acknowledgements cannot race
 * publication. Unlike a generation counter this cannot wrap around. */
static volatile uint32_t cache_stale[RT_CPUS_NR] = {1, 1};
    #endif

/** Wait with IRQs enabled until the initial PendSV selects a thread. */
void SMP_SRAM_CODE rt_smp_boot_wait(void)
{
    for (;;)
    {
    }
}

extern void PendSV_Handler(void);

unsigned SMP_SRAM_CODE rt_hw_cpu_id(void)
{
    return SCB->VTOR == (uint32_t)ap_vectors;
}

rt_base_t SMP_SRAM_CODE rt_hw_local_irq_disable(void)
{
    rt_base_t level = __get_PRIMASK();
    __disable_irq();
    return level;
}

void SMP_SRAM_CODE rt_hw_local_irq_enable(rt_base_t level)
{
    __set_PRIMASK(level);
}

rt_base_t SMP_SRAM_CODE rt_cpus_lock(void)
{
    rt_base_t level = rt_hw_local_irq_disable();
    struct rt_cpu *c = &rt_cpu_table[rt_hw_cpu_id()];
    if (c->lock_depth++ == 0)
    {
        while (!(hwp_mailbox1->C4EXR & SMP_MAILBOX_LOCK))
        {
        }
        __DMB();
    #ifdef RT_SMP_CACHE_SYNC
        unsigned cpu = rt_hw_cpu_id();
        if (cache_stale[cpu])
        {
            /* Only WT data may be cached: there are no dirty lines to lose.
             * CMSIS drains stores before invalidation and completes with
             * DSB/ISB; cache geometry comes from this core's CCSIDR. */
            if (SCB->CCR & SCB_CCR_DC_Msk)
            {
                SCB_InvalidateDCache();
            }
            cache_stale[cpu] = 0;
        }
    #endif
    }
    return level;
}

void SMP_SRAM_CODE rt_cpus_unlock(rt_base_t level)
{
    struct rt_cpu *c = &rt_cpu_table[rt_hw_cpu_id()];
    RT_ASSERT(c->lock_depth);
    if (--c->lock_depth == 0)
    {
    #ifdef RT_SMP_CACHE_SYNC
        /* Publish WT data AND the saved PSP/FP frame before another core
         * can take the lock and select this thread. No dirty-line writeback
         * is used, so independent words in one cache line cannot clobber
         * each other. The peer refreshes on its next outer acquisition. */
        __DSB();
        cache_stale[rt_hw_cpu_id() ^ 1u] = 1;
    #endif
        __DMB();
        hwp_mailbox1->C4EXR = SMP_MAILBOX_LOCK;
        __DSB();
    }
    rt_hw_local_irq_enable(level);
}

void rt_hw_smp_pend(void)
{
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
    __DSB();
    __ISB();
}

void rt_hw_smp_notify(unsigned cpu)
{
    if (!rt_cpu_table[cpu].online)
    {
        return;
    }
    __DMB();
    if (cpu)
    {
        hwp_mailbox1->C3ITR = SMP_IPI_BIT;
    }
    else
    {
        hwp_mailbox1->C4ITR = SMP_IPI_BIT;
    }
    __DSB();
}

void ACPU2HCPU_IRQHandler(void)
{
    rt_interrupt_enter();
    if (rt_hw_cpu_id())
    {
        hwp_mailbox1->C3ICR = SMP_IPI_BIT;
    }
    else
    {
        hwp_mailbox1->C4ICR = SMP_IPI_BIT;
    }
    rt_schedule();
    rt_interrupt_leave();
    __DSB();
}

static void _ap_systick(void)
{
    rt_interrupt_enter();
    rt_smp_tick();
    rt_interrupt_leave();
}

static void SMP_SRAM_CODE __attribute__((used)) _ap_record_fault(uint32_t *frame, uint32_t exc_return)
{
    ap_fault_cfsr = SCB->CFSR;
    ap_fault_hfsr = SCB->HFSR;
    ap_fault_pc = frame[6];
    ap_fault_lr = exc_return;
    __DSB();
    for (;;)
    {
    }
}

static void __attribute__((naked, section(".l1_ret_text_smp"))) _ap_fault(void)
{
    __asm volatile("tst lr, #4\n"
                   "ite eq\n"
                   "mrseq r0, msp\n"
                   "mrsne r0, psp\n"
                   "mov r1, lr\n"
                   "b _ap_record_fault");
}

    #ifdef RT_SMP_SHARED_XIP
static uint32_t _memory_attribute_at(uint32_t address, uint32_t *rbar)
{
    uint32_t old = MPU->RNR, result = 0xffffffffu;
    *rbar = 0;
    for (unsigned i = 0; i < ((MPU->TYPE >> 8) & 0xff); i++)
    {
        MPU->RNR = i;
        uint32_t base = MPU->RBAR, limit = MPU->RLAR;
        if ((limit & 1) && (base & ~31u) <= address && (limit | 31u) >= address)
        {
            /* Overlapping enabled regions are not a supported policy. */
            if (result != 0xffffffffu)
            {
                result = 0xfffffffeu;
                break;
            }
            unsigned attr = (limit >> 1) & 7;
            result = (MPU->MAIR[attr / 4] >> ((attr % 4) * 8)) & 0xff;
            *rbar = base;
        }
    }
    MPU->RNR = old;
    return result;
}

static int _memory_policy_ready(void)
{
    rt_base_t level = rt_hw_local_irq_disable();
    uint32_t code_base, ram_base;
    uint32_t code_attr = _memory_attribute_at((uint32_t)rt_system_scheduler_start, &code_base);
    uint32_t ram_attr = _memory_attribute_at(0x60000000u, &ram_base);
    int ok = (MPU->CTRL & MPU_CTRL_ENABLE_Msk) &&
             ((SCB->CCR & (SCB_CCR_IC_Msk | SCB_CCR_DC_Msk)) == (SCB_CCR_IC_Msk | SCB_CCR_DC_Msk)) &&
             (uint32_t)rt_system_scheduler_start >= 0x62000000u &&
             (uint32_t)rt_system_scheduler_start < 0x64000000u && code_attr == 0x22 &&
             (code_base & 31u) == (ARM_MPU_RBAR(0, ARM_MPU_SH_NON, 1, 1, 0) & 31u);
        #ifdef RT_SMP_PSRAM_NONCACHE
    ok = ok && ram_attr == 0x44 &&
         (ram_base & 31u) == (ARM_MPU_RBAR(0, ARM_MPU_SH_OUTER, 0, 1, 1) & 31u);
        #endif
        #ifdef RT_SMP_PSRAM_WT_SYNC
    ok = ok && ram_attr == 0x33 &&
         (ram_base & 31u) == (ARM_MPU_RBAR(0, ARM_MPU_SH_NON, 0, 1, 1) & 31u);
        #endif
        #ifdef RT_SMP_CACHE_SYNC
    uint32_t old = MPU->RNR;
    for (unsigned i = 0; i < ((MPU->TYPE >> 8) & 0xff); i++)
    {
        MPU->RNR = i;
        uint32_t base = MPU->RBAR, limit = MPU->RLAR;
        if ((limit & 1) && !(base & 4))
        {
            unsigned index = (limit >> 1) & 7;
            uint32_t attr = (MPU->MAIR[index / 4] >> ((index % 4) * 8)) & 0xff;
            unsigned inner = attr & 15, outer = attr >> 4;
            if ((inner != 4 && (inner & 4)) || (outer != 4 && (outer & 4)))
            {
                ok = 0;
            }
        }
    }
    MPU->RNR = old;
        #endif
        #ifdef RT_SMP_SRAM_WT_SYNC
    uint32_t cached = (uint32_t)__smp_sram_cache_start__, control_base, sram_base;
    ok = ok && cached > 0x20000000u && cached < 0x20080000u && !(cached & 31u) &&
         (uint32_t)rt_cpu_table < cached && (uint32_t)ap_vectors < cached &&
         (uint32_t)(ap_irq_stack + sizeof(ap_irq_stack)) <= cached &&
         (uint32_t)cache_stale < cached &&
         _memory_attribute_at((uint32_t)rt_cpu_table, &control_base) == 0x44 &&
         _memory_attribute_at(cached, &sram_base) == 0x44;
        #endif
    rt_hw_local_irq_enable(level);
    return ok;
}

static void SMP_SRAM_CODE _ap_memory_init(void)
{
    /* Runs from SRAM before the first branch to shared Flash. ACPU has
     * just been reset; caches have no dirty lines and are initially off. */
    ARM_MPU_Disable();
    for (unsigned i = 0; i < ((MPU->TYPE >> 8) & 0xff); i++)
    {
        ARM_MPU_ClrRegion(i);
    }
    ARM_MPU_SetMemAttr(0, ARM_MPU_ATTR(ARM_MPU_ATTR_MEMORY_(0, 0, 1, 0), ARM_MPU_ATTR_MEMORY_(0, 0, 1, 0)));
    ARM_MPU_SetMemAttr(1, ARM_MPU_ATTR(ARM_MPU_ATTR_NON_CACHEABLE, ARM_MPU_ATTR_NON_CACHEABLE));
    ARM_MPU_SetMemAttr(2, ARM_MPU_ATTR(ARM_MPU_ATTR_DEVICE, ARM_MPU_ATTR_DEVICE_nGnRnE));
    ARM_MPU_SetRegion(0, ARM_MPU_RBAR(0x62000000, ARM_MPU_SH_NON, 1, 1, 0), ARM_MPU_RLAR(0x6fffffff, 0));
        #ifdef RT_SMP_SRAM_WT_SYNC
    uint32_t cached = (uint32_t)__smp_sram_cache_start__;
    ARM_MPU_SetMemAttr(4, 0x33); /* inner/outer WT, read/write allocate */
    ARM_MPU_SetRegion(1, ARM_MPU_RBAR(0x20000000, ARM_MPU_SH_OUTER, 0, 1, 0), ARM_MPU_RLAR(cached - 1, 1));
    ARM_MPU_SetRegion(4, ARM_MPU_RBAR(cached, ARM_MPU_SH_NON, 0, 1, 1), ARM_MPU_RLAR(0x2007ffff, 4));
        #else
    ARM_MPU_SetRegion(1, ARM_MPU_RBAR(0x20000000, ARM_MPU_SH_OUTER, 0, 1, 0), ARM_MPU_RLAR(0x2007ffff, 1));
        #endif
    ARM_MPU_SetRegion(2, ARM_MPU_RBAR(0x40000000, ARM_MPU_SH_NON, 0, 1, 1), ARM_MPU_RLAR(0x5fffffff, 2));
        #ifdef RT_SMP_PSRAM_NONCACHE
    ARM_MPU_SetRegion(3, ARM_MPU_RBAR(0x60000000, ARM_MPU_SH_OUTER, 0, 1, 1), ARM_MPU_RLAR(0x61ffffff, 1));
        #elif defined(RT_SMP_PSRAM_WT_SYNC)
    ARM_MPU_SetMemAttr(3, ARM_MPU_ATTR(ARM_MPU_ATTR_MEMORY_(0, 0, 1, 1), ARM_MPU_ATTR_MEMORY_(0, 0, 1, 1)));
    ARM_MPU_SetRegion(3, ARM_MPU_RBAR(0x60000000, ARM_MPU_SH_NON, 0, 1, 1), ARM_MPU_RLAR(0x61ffffff, 3));
        #else
            #ifdef PSRAM_CACHE_WB
    ARM_MPU_SetMemAttr(3, ARM_MPU_ATTR(ARM_MPU_ATTR_MEMORY_(0, 1, 1, 1), ARM_MPU_ATTR_MEMORY_(0, 1, 1, 1)));
            #else
    ARM_MPU_SetMemAttr(3, ARM_MPU_ATTR(ARM_MPU_ATTR_MEMORY_(0, 0, 1, 1), ARM_MPU_ATTR_MEMORY_(0, 0, 1, 1)));
            #endif
    ARM_MPU_SetRegion(3, ARM_MPU_RBAR(0x60000000, ARM_MPU_SH_NON, 0, 1, 0), ARM_MPU_RLAR(0x61ffffff, 3));
        #endif
    ARM_MPU_Enable(MPU_CTRL_HFNMIENA_Msk);
    SCB_EnableDCache();
    SCB_EnableICache();
    __DSB();
    __ISB();
}
    #endif /* RT_SMP_SHARED_XIP */

static void SMP_SRAM_CODE _ap_entry(void)
{
    __disable_irq();
    SCB->VTOR = (uint32_t)ap_vectors;
    __set_MSPLIM((uint32_t)ap_irq_stack);
    SCB->CPACR |= (3u << 0) | (3u << 2) | (3u << 4) | (15u << 20);
    __DSB();
    __ISB();
    #ifdef RT_SMP_SHARED_XIP
    _ap_memory_init();
    #endif
    #ifdef RT_SMP_SRAM_WT_SYNC
    uint32_t sram_base, control_base;
    uint32_t sram_attr = _memory_attribute_at((uint32_t)__smp_sram_cache_start__, &sram_base);
    uint32_t control_attr = _memory_attribute_at((uint32_t)rt_cpu_table, &control_base);
    if (sram_attr != 0x33 || control_attr != 0x44 ||
        (sram_base & 31u) != (ARM_MPU_RBAR(0, ARM_MPU_SH_NON, 0, 1, 1) & 31u))
    {
        __DSB();
        for (;;)
        {
        }
    }
    #endif
    /* Configure the FPU for eager save: migration must not leave a lazy
     * lower-register frame owned by the previous core. */
    FPU->FPCCR = (FPU->FPCCR | FPU_FPCCR_ASPEN_Msk) & ~FPU_FPCCR_LSPEN_Msk;
    NVIC_SetPriority(SMP_IPI_IRQN, SMP_IPI_PRIORITY);
    NVIC_ClearPendingIRQ(SMP_IPI_IRQN);
    NVIC_EnableIRQ(SMP_IPI_IRQN);
    hwp_mailbox1->C3IER = SMP_IPI_BIT;
    SysTick_Config(SystemCoreClock / RT_TICK_PER_SECOND);
    ap_ready = 1;
    __DSB();
    rt_system_scheduler_start();
    for (;;)
    {
    }
}

rt_err_t rt_smp_start_secondary(void)
{
    if (rt_hw_cpu_id() || ap_ready)
    {
        return -RT_EBUSY;
    }
    #ifdef RT_SMP_SHARED_XIP
    /* A board-specific MPU override must honor the selected policy too. */
    if (!_memory_policy_ready())
    {
        return -RT_EINVAL;
    }
    #endif
    HAL_RCC_ResetACPU();
    HAL_RCC_EnableModule(RCC_MOD_ACPU);
    for (unsigned i = 0; i < SMP_VECTOR_COUNT; i++)
    {
        ap_vectors[i] = (uint32_t)_ap_fault;
    }
    ap_vectors[0] = (uint32_t)(ap_irq_stack + sizeof(ap_irq_stack));
    ap_vectors[1] = (uint32_t)_ap_entry;
    ap_vectors[14] = (uint32_t)PendSV_Handler;
    ap_vectors[15] = (uint32_t)_ap_systick;
    ap_vectors[SMP_EXCEPTION_VECTOR_COUNT + SMP_IPI_IRQN] = (uint32_t)ACPU2HCPU_IRQHandler;
    FPU->FPCCR = (FPU->FPCCR | FPU_FPCCR_ASPEN_Msk) & ~FPU_FPCCR_LSPEN_Msk;
    hwp_mailbox1->C3ICR = SMP_IPI_BIT;
    hwp_mailbox1->C4ICR = SMP_IPI_BIT;
    hwp_mailbox1->C4IER = SMP_IPI_BIT;
    NVIC_SetPriority(SMP_IPI_IRQN, SMP_IPI_PRIORITY);
    NVIC_ClearPendingIRQ(SMP_IPI_IRQN);
    NVIC_EnableIRQ(SMP_IPI_IRQN);
    SCB_CleanDCache();
    __DSB();
    HAL_RCC_ReleaseACPU((uint32_t)ap_vectors);
    rt_tick_t deadline = rt_tick_get();
    while (!rt_cpu_table[1].current_thread && rt_tick_get() - deadline < RT_TICK_PER_SECOND)
    {
        rt_thread_mdelay(1);
    }
    return rt_cpu_table[1].current_thread ? RT_EOK : -RT_ETIMEOUT;
}
#endif /* RT_USING_SMP */
