;/*
;* Copyright (c) 2006-2018, RT-Thread Development Team
;*
;* SPDX-License-Identifier: Apache-2.0
;*
; * Change Logs:
; * Date           Author       Notes
; * 2009-01-17     Bernard      first version.
; * 2012-01-01     aozima       support context switch load/store FPU register.
; * 2013-06-18     aozima       add restore MSP feature.
; * 2013-06-23     aozima       support lazy stack optimized.
; * 2018-07-24     aozima       enhancement hard fault exception handler.
; */

;/**
; * @addtogroup cortex-m33
; */
;/*@{*/

SCB_VTOR        EQU     0xE000ED08               ; Vector Table Offset Register
NVIC_INT_CTRL   EQU     0xE000ED04               ; interrupt control state register
NVIC_SYSPRI2    EQU     0xE000ED20               ; system priority register (2)
NVIC_PENDSV_PRI EQU     0x00FF0000               ; PendSV priority value (lowest)
NVIC_PENDSVSET  EQU     0x10000000               ; value to trigger PendSV exception

    AREA |.text|, CODE, READONLY, ALIGN=2
    THUMB
    REQUIRE8
    PRESERVE8

    IMPORT rt_thread_switch_interrupt_flag
    IMPORT rt_interrupt_from_thread
    IMPORT rt_interrupt_to_thread

    IF :DEF:SECURITY_PENDSV
    IMPORT PendSv_DBG_Trigger
    IMPORT security_pendsv_flag
    ENDIF

;/*
; * rt_base_t rt_hw_interrupt_disable();
; */
rt_hw_interrupt_disable    PROC
    EXPORT  rt_hw_interrupt_disable [WEAK]
    MRS     r0, PRIMASK
    CPSID   I
    BX      LR
    ENDP

;/*
; * void rt_hw_interrupt_enable(rt_base_t level);
; */
rt_hw_interrupt_enable    PROC
    EXPORT  rt_hw_interrupt_enable [WEAK]
    MSR     PRIMASK, r0
    BX      LR
    ENDP

;/*
; * void rt_hw_context_switch(rt_uint32 from, rt_uint32 to);
; * r0 --> from
; * r1 --> to
; */
rt_hw_context_switch_interrupt
    EXPORT rt_hw_context_switch_interrupt [WEAK]
rt_hw_context_switch    PROC
    EXPORT rt_hw_context_switch [WEAK]
       
    SEV    

    ; set rt_thread_switch_interrupt_flag to 1
    LDR     r2, =rt_thread_switch_interrupt_flag
    LDR     r3, [r2]
    CMP     r3, #1
    BEQ     _reswitch
    MOV     r3, #1
    STR     r3, [r2]

    LDR     r2, =rt_interrupt_from_thread   ; set rt_interrupt_from_thread
    STR     r0, [r2]

_reswitch
    LDR     r2, =rt_interrupt_to_thread     ; set rt_interrupt_to_thread
    STR     r1, [r2]

    LDR     r0, =NVIC_INT_CTRL              ; trigger the PendSV exception (causes context switch)
    LDR     r1, =NVIC_PENDSVSET
    STR     r1, [r0]
    BX      LR
    ENDP

; r0 --> switch from thread stack
; r1 --> switch to thread stack
; psr, pc, lr, r12, r3, r2, r1, r0 are pushed into [from] stack
PendSV_Handler   PROC
    EXPORT PendSV_Handler

    ; disable interrupt to protect context switch
    MRS     r2, PRIMASK
    CPSID   I

    IF :DEF:SECURITY_PENDSV
    ; security_pendsv_flag = 1, use for 52x security mode only,
    ; pendsv trigger dbg assert replace of nmi handler
    LDR     r0, =security_pendsv_flag
    LDR     r1, [r0]
    CBZ     r1, thread_switch_next
    ; clear security_pendsv_flag to 0
    BL      rt_hw_cfg_pendsv_pri
    MOV     r1, #0x00
    STR     r1, [r0]
    PUSH    {lr}
    BL      PendSv_DBG_Trigger 
    POP     {lr}
    B       pendsv_exit 
    ENDIF

thread_switch_next
    ; get rt_thread_switch_interrupt_flag
    LDR     r0, =rt_thread_switch_interrupt_flag
    LDR     r1, [r0]
    CBZ     r1, pendsv_exit         ; pendsv already handled

    ; clear rt_thread_switch_interrupt_flag to 0
    MOV     r1, #0x00
    STR     r1, [r0]

    LDR     r0, =rt_interrupt_from_thread
    LDR     r1, [r0]
    CBZ     r1, switch_to_thread    ; skip register save at the first time

    MRS     r1, psp                 ; get from thread stack pointer

    IF      {FPU} != "SoftVFP"
    TST     lr, #0x10               ; if(!EXC_RETURN[4])
    VSTMFDEQ  r1!, {d8 - d15}       ; push FPU register s16~s31
    ENDIF

    STMFD   r1!, {r4 - r11}         ; push r4 - r11 register

    MRS     r4, psplim              ; r4 = psplim
    STMFD   r1!, {r4}               ; push psplim

    IF      {FPU} != "SoftVFP"
    MOV     r4, #0x00               ; flag = 0

    TST     lr, #0x10               ; if(!EXC_RETURN[4])
    MOVEQ   r4, #0x01               ; flag = 1

    STMFD   r1!, {r4}               ; push flag
    ENDIF

    LDR     r0, [r0]
    STR     r1, [r0]                ; update from thread stack pointer

switch_to_thread
    LDR     r1, =rt_interrupt_to_thread
    LDR     r1, [r1]
    LDR     r1, [r1]                ; load thread stack pointer

    IF      {FPU} != "SoftVFP"
    LDMFD   r1!, {r3}               ; pop flag
    ENDIF
    
    LDMFD   r1!, {r4}               ; pop psplim
    MSR     psplim, r4              ; psplim = r4    

    LDMFD   r1!, {r4 - r11}         ; pop r4 - r11 register

    IF      {FPU} != "SoftVFP"
    CMP     r3,  #0                 ; if(flag_r3 != 0)
    VLDMFDNE  r1!, {d8 - d15}       ; pop FPU register s16~s31
    ENDIF

    MSR     psp, r1                 ; update stack pointer

    IF      {FPU} != "SoftVFP"
    ORR     lr, lr, #0x10           ; lr |=  (1 << 4), clean FPCA.
    CMP     r3,  #0                 ; if(flag_r3 != 0)
    BICNE   lr, lr, #0x10           ; lr &= ~(1 << 4), set FPCA.
    ENDIF

pendsv_exit
    ; restore interrupt
    MSR     PRIMASK, r2

    ORR     lr, lr, #0x04
    BX      lr
    ENDP

rt_hw_cfg_pendsv_pri PROC
    EXPORT rt_hw_cfg_pendsv_pri [WEAK]
    ; set the PendSV exception priority
    LDR     r0, =NVIC_SYSPRI2
    LDR     r1, =NVIC_PENDSV_PRI
    LDR.W   r2, [r0,#0x00]       ; read
    ORR     r1,r1,r2             ; modify
    STR     r1, [r0]             ; write-back
    BX      lr
    ENDP

;/*
; * void rt_hw_context_switch_to(rt_uint32 to);
; * r0 --> to
; * this fucntion is used to perform the first thread switch
; */
rt_hw_context_switch_to    PROC
    EXPORT rt_hw_context_switch_to [WEAK]
    ; set to thread
    LDR     r1, =rt_interrupt_to_thread
    STR     r0, [r1]

    IF      {FPU} != "SoftVFP"
    ; CLEAR CONTROL.FPCA
    MRS     r2, CONTROL             ; read
    BIC     r2, #0x04               ; modify
    MSR     CONTROL, r2             ; write-back
    ENDIF

    ; set from thread to 0
    LDR     r1, =rt_interrupt_from_thread
    MOV     r0, #0x0
    STR     r0, [r1]

    ; set interrupt flag to 1
    LDR     r1, =rt_thread_switch_interrupt_flag
    MOV     r0, #1
    STR     r0, [r1]

    ; set the PendSV exception priority
    LDR     r0, =NVIC_SYSPRI2
    LDR     r1, =NVIC_PENDSV_PRI
    LDR.W   r2, [r0,#0x00]       ; read
    ORR     r1,r1,r2             ; modify
    STR     r1, [r0]             ; write-back

    ; trigger the PendSV exception (causes context switch)
    LDR     r0, =NVIC_INT_CTRL
    LDR     r1, =NVIC_PENDSVSET
    STR     r1, [r0]

    ; restore MSP
    LDR     r0, =SCB_VTOR
    LDR     r0, [r0]
    LDR     r0, [r0]
    MSR     msp, r0

    ; enable interrupts at processor level
    CPSIE   F
    CPSIE   I

    ; never reach here!
    ENDP

; compatible with old version
rt_hw_interrupt_thread_switch PROC
    EXPORT rt_hw_interrupt_thread_switch [WEAK]
    BX      lr
    ENDP

    EXPORT rt_hw_cpu_idle [WEAK]
rt_hw_cpu_idle    PROC
    WFE
    BX      lr
    ENDP

    IMPORT rt_hw_hard_fault_exception
    EXPORT HardFault_Handler
HardFault_Handler    PROC

    ; get current context
    TST     lr, #0x04               ; if(!EXC_RETURN[2])
    ITE     EQ
    MRSEQ   r0, msp                 ; [2]=0 ==> Z=1, get fault context from handler.
    MRSNE   r0, psp                 ; [2]=1 ==> Z=0, get fault context from thread.

    STMFD   r0!, {r4 - r11}         ; push r4 - r11 register
    MRS     r4, psplim              ; r4 = psplim
    STMFD   r0!, {r4}               ; push psplim
    IF      {FPU} != "SoftVFP"
    STMFD   r0!, {lr}               ; push dummy for flag
    ENDIF
    STMFD   r0!, {lr}               ; push exec_return register

    TST     lr, #0x04               ; if(!EXC_RETURN[2])
    ITE     EQ
    MSREQ   msp, r0                 ; [2]=0 ==> Z=1, update stack pointer to MSP.
    MSRNE   psp, r0                 ; [2]=1 ==> Z=0, update stack pointer to PSP.

    PUSH    {lr}
    BL      rt_hw_hard_fault_exception
    POP     {lr}

    ORR     lr, lr, #0x04
    BX      lr
    ENDP

    IMPORT rt_hw_mem_manage_exception
    EXPORT MemManage_Handler
MemManage_Handler    PROC

    ; get current context
    TST     lr, #0x04               ; if(!EXC_RETURN[2])
    ITE     EQ
    MRSEQ   r0, msp                 ; [2]=0 ==> Z=1, get fault context from handler.
    MRSNE   r0, psp                 ; [2]=1 ==> Z=0, get fault context from thread.

    STMFD   r0!, {r4 - r11}         ; push r4 - r11 register
    MRS     r4, psplim              ; r4 = psplim
    STMFD   r0!, {r4}               ; push psplim    
    IF      {FPU} != "SoftVFP"
    STMFD   r0!, {lr}               ; push dummy for flag
    ENDIF
    STMFD   r0!, {lr}               ; push exec_return register

    TST     lr, #0x04               ; if(!EXC_RETURN[2])
    ITE     EQ
    MSREQ   msp, r0                 ; [2]=0 ==> Z=1, update stack pointer to MSP.
    MSRNE   psp, r0                 ; [2]=1 ==> Z=0, update stack pointer to PSP.

    PUSH    {lr}
    BL      rt_hw_mem_manage_exception
    POP     {lr}

    ORR     lr, lr, #0x04
    BX      lr
    ENDP

;struct exception_stack_frame
;{
;    rt_uint32_t r0;
;    rt_uint32_t r1;
;    rt_uint32_t r2;
;    rt_uint32_t r3;
;    rt_uint32_t r12;
;    rt_uint32_t lr;
;    rt_uint32_t pc;
;    rt_uint32_t psr;
;};

;struct stack_frame
;{
;#if USE_FPU
;    rt_uint32_t flag;
;#endif /* USE_FPU */
;
;    /* r4 ~ r11 register */
;    rt_uint32_t r4;
;    rt_uint32_t r5;
;    rt_uint32_t r6;
;    rt_uint32_t r7;
;    rt_uint32_t r8;
;    rt_uint32_t r9;
;    rt_uint32_t r10;
;    rt_uint32_t r11;
;
;    struct exception_stack_frame exception_stack_frame;
;    rt_uint32_t dummy;
;};

    IMPORT rt_hw_do_fatal_error
    EXPORT rt_hw_fatal_error [WEAK]
rt_hw_fatal_error    PROC
    PUSH {r4}         ; backup r4
    MRS r4, psr
    PUSH {r4}         ; push psr
    MOV r4, pc    
    PUSH {r4}         ; push pc
    PUSH {lr}         ; push lr
    PUSH {r12}        ; push r12
    PUSH {r0 - r3}    ; push r0~r3
    LDR r4, [sp, #32] ; load old r4
    PUSH {r4 - r11}   ; push r4~r11
    MRS  r4, psplim   ; r4 = psplim
    PUSH {r4}         ; push psplim    
    IF     {FPU} != "SoftVFP"
    PUSH   {lr}       ; push dummy for flag
    ENDIF    
    MOV  r0, sp
    BL   rt_hw_do_fatal_error
    IF     {FPU} != "SoftVFP"
    POP {r0}          ; pop dummy flag
    ENDIF   
    POP {r4}          ; pop psplim
    MSR psplim, r4    ; psplim = r4   
    POP {r4-r11}      
    POP {r0-r3}
    POP {r12}
    POP {lr} ;pop lr
    POP {r4} ;pop pc
    POP {r4} ;pop psr
    POP {r4} ;pop r4
    B   .


    ENDP

    ALIGN   4

    END
        

