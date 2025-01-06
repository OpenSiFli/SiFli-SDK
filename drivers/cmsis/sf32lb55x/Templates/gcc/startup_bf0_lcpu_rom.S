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
                .equ     MCPU_AON, 0x4005f000
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
                .long    AON_IRQHandler                         
                .long    USART1_IRQHandler                      
                .long    USART2_IRQHandler                      
                .long    USART3_IRQHandler                      
                .long    SPI1_IRQHandler                        
                .long    SPI2_IRQHandler                        
                .long    SPI3_IRQHandler                        
                .long    I2C1_IRQHandler                        
                .long    I2C2_IRQHandler                        
                .long    I2C3_IRQHandler                        
                .long    I2C4_IRQHandler                        
                .long    GPTIM1_IRQHandler                      
                .long    GPTIM2_IRQHandler                      
                .long    GPTIM3_IRQHandler                      
                .long    GPIO_IRQHandler                        
                .long    TSEN_IRQHandler                        
                .long    DMAC2_CH1_IRQHandler                   
                .long    DMAC2_CH2_IRQHandler                   
                .long    DMAC2_CH3_IRQHandler                   
                .long    DMAC2_CH4_IRQHandler                   
                .long    DMAC2_CH5_IRQHandler                   
                .long    DMAC2_CH6_IRQHandler                   
                .long    DMAC2_CH7_IRQHandler                   
                .long    DMAC2_CH8_IRQHandler                   
                .long    GPADC1_IRQHandler                      
                .long    GPADC2_IRQHandler                      
                .long    LPCOMP1_IRQHandler                     
                .long    LPCOMP2_IRQHandler                     
                .long    BTIM1_IRQHandler                       
                .long    BTIM2_IRQHandler                       
                .long    WDT2_IRQHandler                        
                .long    HCPU2LCPU_IRQHandler                   
                .long    BCPU2LCPU_IRQHandler                   
                .long    LPTIM2_IRQHandler                      
                .long    LPTIM3_IRQHandler                    	
                .long    LPUART2_IRQHandler                     
                .long    Interrupt36_IRQHandler               	
                .long    Interrupt37_IRQHandler                 
                .long    Interrupt38_IRQHandler                 
                .long    RTC_IRQHandler                         

                .space   (440 * 4)                          /* Interrupts 80 .. 480 are left out */
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
                LDR      R0, =MCPU_AON
                LDR      SP, [R0]
                LDR      R0, [R0,#4]      
                BX       R0
                
                
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

                Set_Default_Handler  AON_IRQHandler
                Set_Default_Handler  USART1_IRQHandler
                Set_Default_Handler  USART2_IRQHandler
                Set_Default_Handler  USART3_IRQHandler
                Set_Default_Handler  SPI1_IRQHandler
                Set_Default_Handler  SPI2_IRQHandler
                Set_Default_Handler  SPI3_IRQHandler
                Set_Default_Handler  I2C1_IRQHandler
                Set_Default_Handler  I2C2_IRQHandler
                Set_Default_Handler  I2C3_IRQHandler
                Set_Default_Handler  I2C4_IRQHandler
                Set_Default_Handler  GPTIM1_IRQHandler
                Set_Default_Handler  GPTIM2_IRQHandler
                Set_Default_Handler  GPTIM3_IRQHandler
                Set_Default_Handler  GPIO_IRQHandler
                Set_Default_Handler  TSEN_IRQHandler
                Set_Default_Handler  DMAC2_CH1_IRQHandler
                Set_Default_Handler  DMAC2_CH2_IRQHandler
                Set_Default_Handler  DMAC2_CH3_IRQHandler
                Set_Default_Handler  DMAC2_CH4_IRQHandler
                Set_Default_Handler  DMAC2_CH5_IRQHandler
                Set_Default_Handler  DMAC2_CH6_IRQHandler
                Set_Default_Handler  DMAC2_CH7_IRQHandler
                Set_Default_Handler  DMAC2_CH8_IRQHandler
                Set_Default_Handler  GPADC1_IRQHandler
                Set_Default_Handler  GPADC2_IRQHandler
                Set_Default_Handler  LPCOMP1_IRQHandler
                Set_Default_Handler  LPCOMP2_IRQHandler
                Set_Default_Handler  BTIM1_IRQHandler
                Set_Default_Handler  BTIM2_IRQHandler
                Set_Default_Handler  WDT2_IRQHandler
                Set_Default_Handler  HCPU2LCPU_IRQHandler
                Set_Default_Handler  BCPU2LCPU_IRQHandler
                Set_Default_Handler  LPTIM2_IRQHandler
                Set_Default_Handler  LPTIM3_IRQHandler
                Set_Default_Handler  LPUART2_IRQHandler
                Set_Default_Handler  Interrupt36_IRQHandler		                
                Set_Default_Handler  Interrupt37_IRQHandler		                
                Set_Default_Handler  Interrupt38_IRQHandler		                
                Set_Default_Handler  RTC_IRQHandler		
 


/* User setup Stack & Heap */

                .global  __stack_limit
                .global  __initial_sp
                .if      Heap_Size != 0                     /* Heap is provided */
                .global  __heap_base
                .global  __heap_limit
                .endif

                .end
