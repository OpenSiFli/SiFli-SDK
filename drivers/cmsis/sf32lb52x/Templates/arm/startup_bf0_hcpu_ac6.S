/**************************************************************************//**
 * @file     startup_ARMCM33.S
 * @brief    CMSIS Core Device Startup File for
 *           ARMCM33 Device
 * @version  V5.3.1
 * @date     09. July 2018
 ******************************************************************************/
/*
 * Copyright (c) 2009-2018 Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
;//-------- <<< Use Configuration Wizard in Context Menu >>> ------------------
*/

                .syntax  unified
                .arch    armv8-m.main

                .eabi_attribute Tag_ABI_align_preserved, 1

/*
;<h> Stack Configuration
;  <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
;</h>
*/
                .equ     Stack_Size, 0x00000400

                .section STACK, "w",%nobits
                .align   3
__stack_limit:
                .space   Stack_Size
                .size    __stack_limit, . - __stack_limit
__initial_sp:
                .size    __initial_sp, . - __initial_sp


/*
;<h> Heap Configuration
;  <o> Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
;</h>
*/
                .equ     Heap_Size, 0x00000C00

                .if      Heap_Size != 0                     /* Heap is provided */
                .section HEAP, "w",%nobits
                .align   3
__heap_base:
                .space   Heap_Size
                .size    __heap_base, . - __heap_base
__heap_limit:
                .size    __heap_limit, . - __heap_limit
                .endif


                .section RESET
                .align   2
                .globl   __Vectors
                .globl   __Vectors_End
                .globl   __Vectors_Size
__Vectors:
                .long    __initial_sp                       /*     Top of Stack */
                .long    Reset_Handler                      /*     Reset Handler */
                .long    NMI_Handler                        /* -14 NMI Handler */
                .long    HardFault_Handler                  /* -13 Hard Fault Handler */
                .long    MemManage_Handler                  /* -12 MPU Fault Handler */
                .long    BusFault_Handler                   /* -11 Bus Fault Handler */
                .long    UsageFault_Handler                 /* -10 Usage Fault Handler */
                .long    SecureFault_Handler                /*  -9 Secure Fault Handler */
                .long    0                                  /*     Reserved */
                .long    0                                  /*     Reserved */
                .long    0                                  /*     Reserved */
                .long    SVC_Handler                        /*  -5 SVCall Handler */
                .long    DebugMon_Handler                   /*  -4 Debug Monitor Handler */
                .long    0                                  /*     Reserved */
                .long    PendSV_Handler                     /*  -2 PendSV Handler */
                .long    SysTick_Handler                    /*  -1 SysTick Handler */

                /* Interrupts */
                .long    RCC_IRQHandler                 /*   0 Interrupt 0 */
                .long    USART1_IRQHandler                 /*   1 Interrupt 1 */
                .long    USART2_IRQHandler                 /*   0 Interrupt 0 */
                .long    USART3_IRQHandler                 /*   1 Interrupt 1 */
                .long    USART4_IRQHandler                 /*   0 Interrupt 0 */
                .long    SPI1_IRQHandler                 /*   1 Interrupt 1 */
                .long    SPI2_IRQHandler                 /*   0 Interrupt 0 */
                .long    SPI3_IRQHandler                 /*   1 Interrupt 1 */
                .long    I2C1_IRQHandler                 /*   0 Interrupt 0 */
                .long    I2C2_IRQHandler                 /*   1 Interrupt 1 */
                .long    I2C3_IRQHandler                 /*   0 Interrupt 0 */
                .long    TIM2_IRQHandler                 /*   1 Interrupt 1 */
                .long    TIM3_IRQHandler                 /*   0 Interrupt 0 */
                .long    TIM4_IRQHandler                 /*   1 Interrupt 1 */
                .long    GPIO_IRQHandler                 /*   0 Interrupt 0 */
                .long    DMAC_IRQHandler                 /*   1 Interrupt 1 */
                .long    GPADC1_IRQHandler                 /*   0 Interrupt 0 */
                .long    GPADC2_IRQHandler                 /*   1 Interrupt 1 */
                .long    FLASHC_IRQHandler                 /*   0 Interrupt 0 */
                .long    PSRAMC_IRQHandler                 /*   1 Interrupt 1 */
                .long    TRNG_IRQHandler                 /*   2 Interrupt 2 */
                .long    AES_IRQHandler                 /*   3 Interrupt 3 */
                .long    CRC_IRQHandler                 /*   4 Interrupt 4 */
                .long    TIM1_IRQHandler                 /*   5 Interrupt 5 */
                .long    WDT1_IRQHandler                 /*   6 Interrupt 6 */
                .long    I2S_IRQHandler                 /*   7 Interrupt 7 */
                .long    EPIC_IRQHandler                 /*   8 Interrupt 8 */
                .long    KEYPAD_IRQHandler                 /*   9 Interrupt 9 */
                .long    USBC_IRQHandler                 /*   5 Interrupt 5 */
                .long    GPDAC_IRQHandler                 /*   5 Interrupt 5 */
                .long    LPTIM_IRQHandler                 /*   6 Interrupt 6 */
                .long    DMAC2_IRQHandler                 /*   7 Interrupt 7 */
                .long    BLE2MCU_IRQHandler                 /*   8 Interrupt 8 */
                .long    RTC_IRQHandler                 /*   9 Interrupt 9 */
                .long    Interrupt34_IRQHandler                 /*   9 Interrupt 9 */
                .long    Interrupt35_IRQHandler                 /*   9 Interrupt 9 */
                .long    Interrupt36_IRQHandler                 /*   9 Interrupt 9 */
                .long    Interrupt37_IRQHandler                 /*   9 Interrupt 9 */
                .long    Interrupt38_IRQHandler                 /*   9 Interrupt 9 */
                .long    Interrupt39_IRQHandler                 /*   9 Interrupt 9 */

                .space   (440 * 4)                          /* Interrupts 10 .. 480 are left out */
__Vectors_End:
                .equ     __Vectors_Size, __Vectors_End - __Vectors
                .size    __Vectors, . - __Vectors


                .thumb
                .section .text
                .align   2

                .thumb_func
                .type    Reset_Handler, %function
                .globl   Reset_Handler
                .fnstart
                .cantunwind
Reset_Handler:
                ldr      r0, =__stack_limit
                msr      msplim, r0

                bl       SystemInit
                bl       __main

                .fnend
                .size    Reset_Handler, . - Reset_Handler


                .thumb_func
                .type    Default_Handler, %function
                .weak    Default_Handler
                .fnstart
                .cantunwind
Default_Handler:
                b        .
                .fnend
                .size    Default_Handler, . - Default_Handler

/* Macro to define default exception/interrupt handlers.
 * Default handler are weak symbols with an endless loop.
 * They can be overwritten by real handlers.
 */
                .macro   Set_Default_Handler  Handler_Name
                .weak    \Handler_Name
                .set     \Handler_Name, Default_Handler
                .endm


/* Default exception/interrupt handler */

                /* Set_Default_Handler  NMI_Handler */
                Set_Default_Handler  HardFault_Handler
                Set_Default_Handler  MemManage_Handler
                Set_Default_Handler  BusFault_Handler
                Set_Default_Handler  UsageFault_Handler
                Set_Default_Handler  SecureFault_Handler
                Set_Default_Handler  SVC_Handler
                Set_Default_Handler  DebugMon_Handler
                Set_Default_Handler  PendSV_Handler
                Set_Default_Handler  SysTick_Handler

                Set_Default_Handler  RCC_IRQHandler
                Set_Default_Handler  USART1_IRQHandler
                Set_Default_Handler  USART2_IRQHandler
                Set_Default_Handler  USART3_IRQHandler
                Set_Default_Handler  USART4_IRQHandler
                Set_Default_Handler  SPI1_IRQHandler
                Set_Default_Handler  SPI2_IRQHandler
                Set_Default_Handler  SPI3_IRQHandler
                Set_Default_Handler  I2C1_IRQHandler
                Set_Default_Handler  I2C2_IRQHandler
                Set_Default_Handler  I2C3_IRQHandler
                Set_Default_Handler  TIM2_IRQHandler
                Set_Default_Handler  TIM3_IRQHandler
                Set_Default_Handler  TIM4_IRQHandler
                Set_Default_Handler  GPIO_IRQHandler
                Set_Default_Handler  DMAC_IRQHandler
                Set_Default_Handler  GPADC1_IRQHandler
                Set_Default_Handler  GPADC2_IRQHandler
                Set_Default_Handler  FLASHC_IRQHandler
                Set_Default_Handler  PSRAMC_IRQHandler
                Set_Default_Handler  TRNG_IRQHandler
                Set_Default_Handler  AES_IRQHandler
                Set_Default_Handler  CRC_IRQHandler
                Set_Default_Handler  TIM1_IRQHandler
                Set_Default_Handler  WDT1_IRQHandler
                Set_Default_Handler  I2S_IRQHandler
                Set_Default_Handler  EPIC_IRQHandler
                Set_Default_Handler  KEYPAD_IRQHandler
                Set_Default_Handler  USBC_IRQHandler
                Set_Default_Handler  GPDAC_IRQHandler
                Set_Default_Handler  LPTIM_IRQHandler
                Set_Default_Handler  DMAC2_IRQHandler
                Set_Default_Handler  BLE2MCU_IRQHandler
                Set_Default_Handler  RTC_IRQHandler
                Set_Default_Handler  Interrupt34_IRQHandler
                Set_Default_Handler  Interrupt35_IRQHandler
                Set_Default_Handler  Interrupt36_IRQHandler
                Set_Default_Handler  Interrupt37_IRQHandler
                Set_Default_Handler  Interrupt38_IRQHandler
                Set_Default_Handler  Interrupt39_IRQHandler


/* User setup Stack & Heap */

                .global  __stack_limit
                .global  __initial_sp
                .if      Heap_Size != 0                     /* Heap is provided */
                .global  __heap_base
                .global  __heap_limit
                .endif

                .end
