/**************************************************************************//**
 * @file     startup_qemu.S
 * @brief    Minimal Cortex-M33 startup for the QEMU MPS2-AN505 virtual board
 *
 * Core exception handlers (PendSV/HardFault) are provided by the RT-Thread
 * Cortex-M33 port; SysTick_Handler is provided by board.c. Every other vector
 * gets a weak default so the vector table always resolves.
 ******************************************************************************/

                .syntax  unified
                .arch    armv8-m.main

                .section .vectors, "ax", %progbits
                .align   2
                .globl   __Vectors
                .globl   __Vectors_End
                .globl   __Vectors_Size
__Vectors:
                .long    __StackTop                         /*  0 Top of Stack */
                .long    Reset_Handler                      /*  1 Reset Handler */
                .long    NMI_Handler                        /*  2 NMI */
                .long    HardFault_Handler                  /*  3 Hard Fault */
                .long    MemManage_Handler                  /*  4 MPU Fault */
                .long    BusFault_Handler                   /*  5 Bus Fault */
                .long    UsageFault_Handler                 /*  6 Usage Fault */
                .long    SecureFault_Handler                /*  7 Secure Fault */
                .long    0                                  /*  8 Reserved */
                .long    0                                  /*  9 Reserved */
                .long    0                                  /* 10 Reserved */
                .long    SVC_Handler                        /* 11 SVCall */
                .long    DebugMon_Handler                   /* 12 Debug Monitor */
                .long    0                                  /* 13 Reserved */
                .long    PendSV_Handler                     /* 14 PendSV */
                .long    SysTick_Handler                    /* 15 SysTick */
                /* device IRQ 0..31 (all default for now) */
                .rept    32
                .long    Default_Handler
                .endr
__Vectors_End:
                .equ     __Vectors_Size, __Vectors_End - __Vectors
                .size    __Vectors, . - __Vectors

                .section .text
                .align   2
                .thumb_func
                .type    Reset_Handler, %function
                .globl   Reset_Handler
Reset_Handler:
                ldr   sp, =__StackTop          /* set stack pointer */
                ldr   r0, =__StackLimit
                msr   msplim, r0

                /* system init (VTOR/FPU only, no real clocks) */
                bl    SystemInit

                /* copy .data from ROM load region to RAM */
                ldr   r4, =__copy_table_start__
                ldr   r5, =__copy_table_end__
.L_loop0:
                cmp   r4, r5
                bge   .L_loop0_done
                ldr   r1, [r4]                 /* source */
                ldr   r2, [r4, #4]             /* dest   */
                ldr   r3, [r4, #8]             /* size   */
.L_loop0_0:
                subs  r3, #4
                ittt  ge
                ldrge r0, [r1, r3]
                strge r0, [r2, r3]
                bge   .L_loop0_0
                adds  r4, #12
                b     .L_loop0
.L_loop0_done:

                /* zero .bss */
                ldr   r3, =__zero_table_start__
                ldr   r4, =__zero_table_end__
.L_loop2:
                cmp   r3, r4
                bge   .L_loop2_done
                ldr   r1, [r3]                 /* dest   */
                ldr   r2, [r3, #4]             /* size   */
                movs  r0, 0
.L_loop2_0:
                subs  r2, #4
                itt   ge
                strge r0, [r1, r2]
                bge   .L_loop2_0
                adds  r3, #8
                b     .L_loop2
.L_loop2_done:

                bl    entry
                b     .                        /* entry should not return */
                .size  Reset_Handler, . - Reset_Handler

                /* Default handler: infinite loop */
                .weak   Default_Handler
                .type   Default_Handler, %function
Default_Handler:
                b       .
                .size   Default_Handler, . - Default_Handler

                /* Weak aliases so the vector table resolves even without
                 * strong definitions (RT-Thread overrides PendSV/HardFault,
                 * board.c overrides SysTick). */
                .macro  Set_Default_Handler  Handler_Name
                .weak   \Handler_Name
                .set    \Handler_Name, Default_Handler
                .endm

                Set_Default_Handler  NMI_Handler
                Set_Default_Handler  HardFault_Handler
                Set_Default_Handler  MemManage_Handler
                Set_Default_Handler  BusFault_Handler
                Set_Default_Handler  UsageFault_Handler
                Set_Default_Handler  SecureFault_Handler
                Set_Default_Handler  SVC_Handler
                Set_Default_Handler  DebugMon_Handler
                Set_Default_Handler  PendSV_Handler
                Set_Default_Handler  SysTick_Handler

                .end
