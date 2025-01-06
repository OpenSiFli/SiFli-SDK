/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2011-10-21     Bernard      the first version.
 * 2011-10-27     aozima       update for cortex-M4 FPU.
 * 2011-12-31     aozima       fixed stack align issues.
 * 2012-01-01     aozima       support context switch load/store FPU register.
 * 2012-12-11     lgnq         fixed the coding style.
 * 2012-12-23     aozima       stack addr align to 8byte.
 * 2012-12-29     Bernard      Add exception hook.
 * 2013-06-23     aozima       support lazy stack optimized.
 * 2018-07-24     aozima       enhancement hard fault exception handler.
 */

#include <rtthread.h>
#include <string.h>
#include <register.h>

#ifdef RT_USING_ULOG
    #include "ulog.h"
#endif

#if               /* ARMCC */ (  (defined ( __CC_ARM ) && defined ( __TARGET_FPU_VFP ))    \
                  /* Clang */ || (defined ( __CLANG_ARM ) && defined ( __VFP_FP__ ) && !defined(__SOFTFP__)) \
                  /* IAR */   || (defined ( __ICCARM__ ) && defined ( __ARMVFP__ ))        \
                  /* GNU */   || (defined ( __GNUC__ ) && defined ( __VFP_FP__ ) && !defined(__SOFTFP__)) )
#define USE_FPU   1
#else
#define USE_FPU   0
#endif

/* exception and interrupt handler table */
__ROM_USED rt_uint32_t rt_interrupt_from_thread;
__ROM_USED rt_uint32_t rt_interrupt_to_thread;
__ROM_USED rt_uint32_t rt_thread_switch_interrupt_flag;
/* exception hook */
static rt_err_t (*rt_exception_hook)(void *context) = RT_NULL;

struct exception_stack_frame
{
    rt_uint32_t r0;
    rt_uint32_t r1;
    rt_uint32_t r2;
    rt_uint32_t r3;
    rt_uint32_t r12;
    rt_uint32_t lr;
    rt_uint32_t pc;
    rt_uint32_t psr;
};

struct stack_frame
{
#if USE_FPU
    rt_uint32_t flag;
#endif /* USE_FPU */

    rt_uint32_t psplim;

    /* r4 ~ r11 register */
    rt_uint32_t r4;
    rt_uint32_t r5;
    rt_uint32_t r6;
    rt_uint32_t r7;
    rt_uint32_t r8;
    rt_uint32_t r9;
    rt_uint32_t r10;
    rt_uint32_t r11;

    struct exception_stack_frame exception_stack_frame;
};

struct exception_stack_frame_fpu
{
    rt_uint32_t r0;
    rt_uint32_t r1;
    rt_uint32_t r2;
    rt_uint32_t r3;
    rt_uint32_t r12;
    rt_uint32_t lr;
    rt_uint32_t pc;
    rt_uint32_t psr;

#if USE_FPU
    /* FPU register */
    rt_uint32_t S0;
    rt_uint32_t S1;
    rt_uint32_t S2;
    rt_uint32_t S3;
    rt_uint32_t S4;
    rt_uint32_t S5;
    rt_uint32_t S6;
    rt_uint32_t S7;
    rt_uint32_t S8;
    rt_uint32_t S9;
    rt_uint32_t S10;
    rt_uint32_t S11;
    rt_uint32_t S12;
    rt_uint32_t S13;
    rt_uint32_t S14;
    rt_uint32_t S15;
    rt_uint32_t FPSCR;
    rt_uint32_t NO_NAME;
#endif
};

struct stack_frame_fpu
{
    rt_uint32_t flag;

    /* r4 ~ r11 register */
    rt_uint32_t r4;
    rt_uint32_t r5;
    rt_uint32_t r6;
    rt_uint32_t r7;
    rt_uint32_t r8;
    rt_uint32_t r9;
    rt_uint32_t r10;
    rt_uint32_t r11;

#if USE_FPU
    /* FPU register s16 ~ s31 */
    rt_uint32_t s16;
    rt_uint32_t s17;
    rt_uint32_t s18;
    rt_uint32_t s19;
    rt_uint32_t s20;
    rt_uint32_t s21;
    rt_uint32_t s22;
    rt_uint32_t s23;
    rt_uint32_t s24;
    rt_uint32_t s25;
    rt_uint32_t s26;
    rt_uint32_t s27;
    rt_uint32_t s28;
    rt_uint32_t s29;
    rt_uint32_t s30;
    rt_uint32_t s31;
#endif

    struct exception_stack_frame_fpu exception_stack_frame;
};
__ROM_USED struct stack_frame saved_stack_frame;
__ROM_USED uint32_t saved_stack_pointer;
__ROM_USED rt_error_reason_t error_reason;

typedef struct
{
    uint32_t cfsr;
    uint32_t hfsr;
    uint32_t mmar;
    uint32_t bfar;
    uint32_t shcsr;
} saved_scb_reg_t;

saved_scb_reg_t saved_scb_reg;
#ifdef SOC_BF0_HCPU
    uint32_t saved_hpsys_aon_issr_reg;
#endif /* SOC_BF0_HCPU */

__ROM_USED rt_uint8_t *rt_hw_stack_init(void       *tentry,
                                        void       *parameter,
                                        rt_uint8_t *stack_addr,
                                        void       *texit)
{
    struct stack_frame *stack_frame;
    rt_uint8_t         *stk;
    unsigned long       i;

    stk  = stack_addr + sizeof(rt_uint32_t);
    stk  = (rt_uint8_t *)RT_ALIGN_DOWN((rt_uint32_t)stk, 8);
    stk -= sizeof(struct stack_frame);

    stack_frame = (struct stack_frame *)stk;

    /* init all register */
    for (i = 0; i < sizeof(struct stack_frame) / sizeof(rt_uint32_t); i ++)
    {
        ((rt_uint32_t *)stack_frame)[i] = 0xdeadbeef;
    }

    stack_frame->exception_stack_frame.r0  = (unsigned long)parameter; /* r0 : argument */
    stack_frame->exception_stack_frame.r1  = 0;                        /* r1 */
    stack_frame->exception_stack_frame.r2  = 0;                        /* r2 */
    stack_frame->exception_stack_frame.r3  = 0;                        /* r3 */
    stack_frame->exception_stack_frame.r12 = 0;                        /* r12 */
    stack_frame->exception_stack_frame.lr  = (unsigned long)texit;     /* lr */
    stack_frame->exception_stack_frame.pc  = (unsigned long)tentry;    /* entry point, pc */
    stack_frame->exception_stack_frame.psr = 0x01000000L;              /* PSR */

#if USE_FPU
    stack_frame->flag = 0;
#endif /* USE_FPU */

    stack_frame->psplim = 0;

    /* return task's current stack address */
    return stk;
}

__ROM_USED void rt_hw_set_stack_limit(rt_uint8_t *stack_addr, void *stack_limit)
{
    struct stack_frame *stack_frame = (struct stack_frame *)stack_addr;

    stack_frame->psplim = (rt_uint32_t)stack_limit;

    return;
}

/**
 * This function set the hook, which is invoked on fault exception handling.
 *
 * @param exception_handle the exception handling hook function.
 */
__ROM_USED void rt_hw_exception_install(rt_err_t (*exception_handle)(void *context))
{
    rt_exception_hook = exception_handle;
}

#define SCB_SHCSR       (*(volatile const unsigned *)0xE000ED24) /* System Handler Control and State Register */
#define SCB_CFSR        (*(volatile const unsigned *)0xE000ED28) /* Configurable Fault Status Register */
#define SCB_HFSR        (*(volatile const unsigned *)0xE000ED2C) /* HardFault Status Register */
#define SCB_MMAR        (*(volatile const unsigned *)0xE000ED34) /* MemManage Fault Address register */
#define SCB_BFAR        (*(volatile const unsigned *)0xE000ED38) /* Bus Fault Address Register */
#define SCB_AIRCR       (*(volatile unsigned long *)0xE000ED0C)  /* Reset control Address Register */
#define SCB_RESET_VALUE 0x05FA0004                               /* Reset value, write to SCB_AIRCR can reset cpu */

#define SCB_CFSR_MFSR   (*(volatile const unsigned char*)0xE000ED28)  /* Memory-management Fault Status Register */
#define SCB_CFSR_BFSR   (*(volatile const unsigned char*)0xE000ED29)  /* Bus Fault Status Register */
#define SCB_CFSR_UFSR   (*(volatile const unsigned short*)0xE000ED2A) /* Usage Fault Status Register */

#ifdef RT_USING_FINSH
static void usage_fault_track(void)
{
    rt_kprintf("usage fault:\n");
    rt_kprintf("SCB_CFSR_UFSR:0x%02X ", SCB_CFSR_UFSR);

    if (SCB_CFSR_UFSR & (1 << 0))
    {
        /* [0]:UNDEFINSTR */
        rt_kprintf("UNDEFINSTR ");
    }

    if (SCB_CFSR_UFSR & (1 << 1))
    {
        /* [1]:INVSTATE */
        rt_kprintf("INVSTATE ");
    }

    if (SCB_CFSR_UFSR & (1 << 2))
    {
        /* [2]:INVPC */
        rt_kprintf("INVPC ");
    }

    if (SCB_CFSR_UFSR & (1 << 3))
    {
        /* [3]:NOCP */
        rt_kprintf("NOCP ");
    }

    if (SCB_CFSR_UFSR & (1 << 4))
    {
        /* [4]:STKOF */
        rt_kprintf("STKOF ");
    }

    if (SCB_CFSR_UFSR & (1 << 8))
    {
        /* [8]:UNALIGNED */
        rt_kprintf("UNALIGNED ");
    }

    if (SCB_CFSR_UFSR & (1 << 9))
    {
        /* [9]:DIVBYZERO */
        rt_kprintf("DIVBYZERO ");
    }

    rt_kprintf("\n");
}

static void bus_fault_track(void)
{
    rt_kprintf("bus fault:\n");
    rt_kprintf("SCB_CFSR_BFSR:0x%02X ", SCB_CFSR_BFSR);

    if (SCB_CFSR_BFSR & (1 << 0))
    {
        /* [0]:IBUSERR */
        rt_kprintf("IBUSERR ");
    }

    if (SCB_CFSR_BFSR & (1 << 1))
    {
        /* [1]:PRECISERR */
        rt_kprintf("PRECISERR ");
    }

    if (SCB_CFSR_BFSR & (1 << 2))
    {
        /* [2]:IMPRECISERR */
        rt_kprintf("IMPRECISERR ");
    }

    if (SCB_CFSR_BFSR & (1 << 3))
    {
        /* [3]:UNSTKERR */
        rt_kprintf("UNSTKERR ");
    }

    if (SCB_CFSR_BFSR & (1 << 4))
    {
        /* [4]:STKERR */
        rt_kprintf("STKERR ");
    }

    if (SCB_CFSR_BFSR & (1 << 7))
    {
        rt_kprintf("SCB->BFAR:%08X\n", SCB_BFAR);
    }
    else
    {
        rt_kprintf("\n");
    }
}

static void mem_manage_fault_track(void)
{
    rt_kprintf("mem manage fault:\n");
    rt_kprintf("SCB_CFSR_MFSR:0x%02X ", SCB_CFSR_MFSR);

    if (SCB_CFSR_MFSR & (1 << 0))
    {
        /* [0]:IACCVIOL */
        rt_kprintf("IACCVIOL ");
    }

    if (SCB_CFSR_MFSR & (1 << 1))
    {
        /* [1]:DACCVIOL */
        rt_kprintf("DACCVIOL ");
    }

    if (SCB_CFSR_MFSR & (1 << 3))
    {
        /* [3]:MUNSTKERR */
        rt_kprintf("MUNSTKERR ");
    }

    if (SCB_CFSR_MFSR & (1 << 4))
    {
        /* [4]:MSTKERR */
        rt_kprintf("MSTKERR ");
    }

    if (SCB_CFSR_MFSR & (1 << 7))
    {
        /* [7]:MMARVALID */
        rt_kprintf("SCB->MMAR:%08X\n", SCB_MMAR);
    }
    else
    {
        rt_kprintf("\n");
    }
}

static void hard_fault_track(void)
{
    if (SCB_HFSR & (1UL << 1))
    {
        /* [1]:VECTBL, Indicates hard fault is caused by failed vector fetch. */
        rt_kprintf("failed vector fetch\n");
    }

    if (SCB_HFSR & (1UL << 30))
    {
        /* [30]:FORCED, Indicates hard fault is taken because of bus fault,
                        memory management fault, or usage fault. */
        if (SCB_CFSR_BFSR)
        {
            bus_fault_track();
        }

        if (SCB_CFSR_MFSR)
        {
            mem_manage_fault_track();
        }

        if (SCB_CFSR_UFSR)
        {
            usage_fault_track();
        }
    }

    if (SCB_HFSR & (1UL << 31))
    {
        /* [31]:DEBUGEVT, Indicates hard fault is triggered by debug event. */
        rt_kprintf("debug event\n");
    }
}
#endif /* RT_USING_FINSH */


struct exception_info
{
    rt_uint32_t exc_return;
    struct stack_frame stack_frame;
};

static void handle_exception(struct exception_info *exception_info)
{
    extern long list_thread(void);
    struct exception_stack_frame *exception_stack = &exception_info->stack_frame.exception_stack_frame;
    struct stack_frame *context = &exception_info->stack_frame;

#ifdef SOC_BF0_HCPU
    saved_hpsys_aon_issr_reg = hwp_hpsys_aon->ISSR;
#endif /* SOC_BF0_HCPU */

    memcpy(&saved_stack_frame, context, sizeof(saved_stack_frame));
    if (exception_info->exc_return & 0x10)
    {
        /* FPU is not active */
        saved_stack_pointer = (uint32_t)((struct exception_stack_frame *)&exception_info->stack_frame.exception_stack_frame + 1);
    }
    else
    {
        saved_stack_pointer = (uint32_t)((struct exception_stack_frame_fpu *)&exception_info->stack_frame.exception_stack_frame + 1);
    }
    error_reason = RT_ERROR_HW_EXCEPTION;

    saved_scb_reg.cfsr = SCB_CFSR;
    saved_scb_reg.hfsr = SCB_HFSR;
    saved_scb_reg.mmar = SCB_MMAR;
    saved_scb_reg.bfar = SCB_BFAR;
    saved_scb_reg.shcsr = SCB_SHCSR;

#if defined(RT_USING_ULOG)
    rt_device_t dev = rt_console_get_device();
    if (dev)
    {
        rt_device_close(dev);
        rt_device_open(dev, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_STREAM);
    }
    ulog_flush();
#endif


    if (rt_exception_hook != RT_NULL)
    {
        rt_err_t result;

        result = rt_exception_hook(exception_stack);
        if (result == RT_EOK) return;
    }

    rt_kprintf(" sp: 0x%08x\n", (uint32_t)((struct exception_stack_frame_fpu *)&exception_info->stack_frame.exception_stack_frame + 1));

    rt_kprintf("psr: 0x%08x\n", context->exception_stack_frame.psr);

    rt_kprintf("r00: 0x%08x\n", context->exception_stack_frame.r0);
    rt_kprintf("r01: 0x%08x\n", context->exception_stack_frame.r1);
    rt_kprintf("r02: 0x%08x\n", context->exception_stack_frame.r2);
    rt_kprintf("r03: 0x%08x\n", context->exception_stack_frame.r3);
    rt_kprintf("r04: 0x%08x\n", context->r4);
    rt_kprintf("r05: 0x%08x\n", context->r5);
    rt_kprintf("r06: 0x%08x\n", context->r6);
    rt_kprintf("r07: 0x%08x\n", context->r7);
    rt_kprintf("r08: 0x%08x\n", context->r8);
    rt_kprintf("r09: 0x%08x\n", context->r9);
    rt_kprintf("r10: 0x%08x\n", context->r10);
    rt_kprintf("r11: 0x%08x\n", context->r11);
    rt_kprintf("r12: 0x%08x\n", context->exception_stack_frame.r12);
    rt_kprintf(" lr: 0x%08x\n", context->exception_stack_frame.lr);
    rt_kprintf(" pc: 0x%08x\n", context->exception_stack_frame.pc);

    if (exception_info->exc_return & (1 << 2))
    {
        rt_kprintf("hard fault on thread: %s\r\n\r\n", rt_thread_self()->name);
    }
    else
    {
        rt_kprintf("hard fault on handler\r\n\r\n");
    }
    rt_kprintf("=====================\n");
    rt_kprintf("PSP: 0x%p, MSP: 0x%p\n", __get_PSP(), __get_MSP());

#if defined(RT_USING_ULOG)
    ulog_flush();
#endif

    rt_show_sys_info();

    if ((exception_info->exc_return & 0x10) == 0)
    {
        rt_kprintf("FPU active!\r\n");
    }

#if defined(RT_USING_ULOG)
    ulog_flush();
#endif

#if defined (__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    SCB_CleanDCache();
#endif

#if (defined(SOC_BF0_LCPU) && (!defined(FPGA)))
    extern void HAL_LCPU_ASSERT_INFO_set(void);
    HAL_LCPU_ASSERT_INFO_set();
#endif

#ifdef SOC_BF0_LCPU
    HAL_LPAON_WakeCore(CORE_ID_HCPU);
#endif /* SOC_BF0_LCPU */
}

__ROM_USED void rt_hw_hard_fault_exception(struct exception_info *exception_info)
{
    handle_exception(exception_info);

#ifdef RT_USING_FINSH
    hard_fault_track();
#endif /* RT_USING_FINSH */

#if defined(RT_USING_ULOG)
    ulog_flush();
#endif

#if defined(SOC_BF0_HCPU) && defined(SAVE_ASSERT_CONTEXT_IN_FLASH)
    extern void HAL_LCPU_ASSERT_INFO_clear(void);
    extern rt_err_t save_assert_context_in_flash();
    save_assert_context_in_flash();
    HAL_LCPU_ASSERT_INFO_clear();
#endif

    while (1);
}

__ROM_USED void rt_hw_mem_manage_exception(struct exception_info *exception_info)
{
    handle_exception(exception_info);

#ifdef RT_USING_FINSH
    mem_manage_fault_track();
#endif /* RT_USING_FINSH */

#if defined(RT_USING_ULOG)
    ulog_flush();
#endif

#if defined(SOC_BF0_HCPU) && defined(SAVE_ASSERT_CONTEXT_IN_FLASH)
    extern void HAL_LCPU_ASSERT_INFO_clear(void);
    extern rt_err_t save_assert_context_in_flash();
    save_assert_context_in_flash();
    HAL_LCPU_ASSERT_INFO_clear();
#endif

    while (1);
}
__ROM_USED void rt_hw_do_fatal_error(struct stack_frame *stack_frame)
{
    /* Avoid assertion happening after crashdump for hardfault */
    if (RT_NO_ERROR == error_reason)
    {
        error_reason = RT_ERROR_ASSERT;
    }
    else
    {
        while (1);
    }
    /* skip 32bit dummy field */
    saved_stack_pointer = (uint32_t)(stack_frame + 1) + 4;
    memcpy(&saved_stack_frame, stack_frame, sizeof(*stack_frame));

    rt_kprintf("=====================\n");
    rt_kprintf("PSP: 0x%p, MSP: 0x%p\n", __get_PSP(), __get_MSP());
    rt_kprintf("=====================\n");
    rt_kprintf(" sp: 0x%08x\n", saved_stack_pointer);
    rt_kprintf("psr: 0x%08x\n", stack_frame->exception_stack_frame.psr);
    rt_kprintf("r00: 0x%08x\n", stack_frame->exception_stack_frame.r0);
    rt_kprintf("r01: 0x%08x\n", stack_frame->exception_stack_frame.r1);
    rt_kprintf("r02: 0x%08x\n", stack_frame->exception_stack_frame.r2);
    rt_kprintf("r03: 0x%08x\n", stack_frame->exception_stack_frame.r3);
    rt_kprintf("r12: 0x%08x\n", stack_frame->exception_stack_frame.r12);
    rt_kprintf(" lr: 0x%08x\n", stack_frame->exception_stack_frame.lr);
    rt_kprintf(" pc: 0x%08x\n", stack_frame->exception_stack_frame.pc);

    rt_kprintf("=====================\n");
    if (rt_interrupt_get_nest() == 0)
    {
        rt_kprintf("fatal error on thread: %s\n", rt_thread_self()->name);
    }
    else
    {
        rt_kprintf("fatal error on ISR\n");
    }

#if defined(RT_USING_ULOG)
    ulog_flush();
#endif

#if defined (__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    SCB_CleanDCache();
#endif

#if (defined(SOC_BF0_LCPU) && (!defined(FPGA)))
    extern void HAL_LCPU_ASSERT_INFO_set(void);
    HAL_LCPU_ASSERT_INFO_set();
#endif

#ifdef SOC_BF0_LCPU
    HAL_LPAON_WakeCore(CORE_ID_HCPU);
#endif /* SOC_BF0_LCPU */

#if defined(SOC_BF0_HCPU) && defined(SAVE_ASSERT_CONTEXT_IN_FLASH)
    extern void HAL_LCPU_ASSERT_INFO_clear(void);
    extern rt_err_t save_assert_context_in_flash();
    save_assert_context_in_flash();
    HAL_LCPU_ASSERT_INFO_clear();
#endif


    return;

}

/**
 * shutdown CPU
 */
__ROM_USED void rt_hw_cpu_shutdown(void)
{
    rt_kprintf("shutdown...\n");

    RT_ASSERT(0);
}

/**
 * reset CPU
 */
RT_WEAK void rt_hw_cpu_reset(void)
{
    SCB_AIRCR = SCB_RESET_VALUE;
}

#ifdef RT_USING_CPU_FFS
/**
 * This function finds the first bit set (beginning with the least significant bit)
 * in value and return the index of that bit.
 *
 * Bits are numbered starting at 1 (the least significant bit).  A return value of
 * zero from any of these functions means that the argument was zero.
 *
 * @return return the index of the first bit set. If value is 0, then this function
 * shall return 0.
 */
#if defined(__CC_ARM) || defined(__CLANG_ARM)
__asm int __rt_ffs(int value)
{
    CMP     r0, #0x00
    BEQ     exit

    RBIT    r0, r0
    CLZ     r0, r0
    ADDS    r0, r0, #0x01

    exit
    BX      lr
}
#elif defined(__IAR_SYSTEMS_ICC__)
int __rt_ffs(int value)
{
    if (value == 0) return value;

    asm("RBIT %0, %1" : "=r"(value) : "r"(value));
    asm("CLZ  %0, %1" : "=r"(value) : "r"(value));
    asm("ADDS %0, %1, #0x01" : "=r"(value) : "r"(value));

    return value;
}
#elif defined(__GNUC__)
int __rt_ffs(int value)
{
    return __builtin_ffs(value);
}
#endif

#endif
