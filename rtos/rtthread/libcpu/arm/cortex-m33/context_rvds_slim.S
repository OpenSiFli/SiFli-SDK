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



    IMPORT rt_hw_do_fatal_error
    EXPORT rt_hw_fatal_error
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
        

