/* SPDX-License-Identifier: Apache-2.0
 * Same frame as the SiFli Cortex-M33 stack initializer. Selection and
 * outgoing ownership release occur only after r4-r11/FPU/PSPLIM are saved.
 */

#ifdef RT_USING_SMP
.cpu cortex-m33
.syntax unified
.thumb
.text

.equ SCB_VTOR,        0xE000ED08
.equ NVIC_INT_CTRL,   0xE000ED04
.equ NVIC_SYSPRI2,    0xE000ED20
.equ NVIC_PENDSV_PRI, 0x00FF0000
.equ NVIC_PENDSVSET,  0x10000000

.global PendSV_Handler
.type PendSV_Handler, %function
.thumb_func
PendSV_Handler:
    cpsid i
    mrs r0, psp
    cbz r0, 1f
    #if defined(__VFP_FP__) && !defined(__SOFTFP__)
    tst lr, #0x10
    it eq
    vstmdbeq r0!, {d8-d15}
    #endif
    stmdb r0!, {r4-r11}
    mrs r1, psplim
    stmdb r0!, {r1}
    #if defined(__VFP_FP__) && !defined(__SOFTFP__)
    movs r1, #0
    tst lr, #0x10
    it eq
    moveq r1, #1
    stmdb r0!, {r1}
    #endif
1:
    push {r3, lr}
    bl rt_smp_switch
    pop {r3, lr}
    #if defined(__VFP_FP__) && !defined(__SOFTFP__)
    ldmia r0!, {r3}
    #endif
    ldmia r0!, {r1}
    msr psplim, r1
    ldmia r0!, {r4-r11}
    #if defined(__VFP_FP__) && !defined(__SOFTFP__)
    cmp r3, #0
    it ne
    vldmiane r0!, {d8-d15}
    #endif
    msr psp, r0
    orr lr, lr, #0x14
    #if defined(__VFP_FP__) && !defined(__SOFTFP__)
    cmp r3, #0
    it ne
    bicne lr, lr, #0x10
    #endif
    cpsie i
    bx lr
.size PendSV_Handler, .-PendSV_Handler

.global rt_hw_cfg_pendsv_pri
.type rt_hw_cfg_pendsv_pri, %function
.thumb_func
rt_hw_cfg_pendsv_pri:
    ldr r0, =NVIC_SYSPRI2
    ldr r1, [r0]
    orr r1, r1, #NVIC_PENDSV_PRI
    str r1, [r0]
    bx lr

.global rt_hw_context_switch_to
.type rt_hw_context_switch_to, %function
.thumb_func
rt_hw_context_switch_to:
    cpsid i
    bl rt_hw_cfg_pendsv_pri
    movs r0, #0
    msr psp, r0
    msr control, r0
    isb
    ldr r0, =SCB_VTOR
    ldr r0, [r0]
    ldr r0, [r0]
    msr msp, r0
    ldr r0, =NVIC_INT_CTRL
    ldr r1, =NVIC_PENDSVSET
    str r1, [r0]
    cpsie f
    cpsie i
    b rt_smp_boot_wait

.global rt_hw_context_switch
.global rt_hw_context_switch_interrupt
.type rt_hw_context_switch, %function
.type rt_hw_context_switch_interrupt, %function
.thumb_func
rt_hw_context_switch:
rt_hw_context_switch_interrupt:
    b rt_hw_smp_pend

.global rt_hw_cpu_idle
.type rt_hw_cpu_idle, %function
.thumb_func
rt_hw_cpu_idle:
    wfi
    bx lr

.global rt_hw_interrupt_thread_switch
.type rt_hw_interrupt_thread_switch, %function
.thumb_func
rt_hw_interrupt_thread_switch:
    bx lr
#endif /* RT_USING_SMP */
