/**************************************************************************//**
 * @file     startup_ARMCM33.S
 * @brief    CMSIS-Core(M) Device Startup File for Cortex-M33 Device
 * @version  V2.0.1
 * @date     23. July 2019
 ******************************************************************************/
/*
 * Copyright (c) 2009-2019 Arm Limited. All rights reserved.
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

                .syntax  unified
                .arch    armv8-m.main

                .section .vectors
                .align   2
                .globl   __Vectors
                .globl   __Vectors_End
                .globl   __Vectors_Size
__Vectors:
                .long    __StackTop                         /*     Top of Stack */
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
                .long     AON_IRQHandler                         /*   0 Interrupt 0  */
                .long     ble_isr                                /*   1 Interrupt 1  */
                .long     DMAC2_CH1_IRQHandler                   /*   2 Interrupt 2  */
                .long     DMAC2_CH2_IRQHandler                   /*   3 Interrupt 3  */
                .long     DMAC2_CH3_IRQHandler                   /*   4 Interrupt 4  */
                .long     DMAC2_CH4_IRQHandler                   /*   5 Interrupt 5  */
                .long     DMAC2_CH5_IRQHandler                   /*   6 Interrupt 6  */
                .long     DMAC2_CH6_IRQHandler                   /*   7 Interrupt 7  */
                .long     DMAC2_CH7_IRQHandler                   /*   8 Interrupt 8  */
                .long     DMAC2_CH8_IRQHandler                   /*   9 Interrupt 9  */                  
                .long     PATCH_IRQHandler                       /*  10 Interrupt 10 */
                .long     dm_isr                                 /*  11 Interrupt 11 */
                .long     USART4_IRQHandler                      /*  12 Interrupt 12 */
                .long     USART5_IRQHandler                      /*  13 Interrupt 13 */
                .long     USART6_IRQHandler                      /*  14 Interrupt 14 */   
                .long     bt_isr                                 /*  15 Interrupt 15 */
                .long     SPI3_IRQHandler                        /*  16 Interrupt 16 */
                .long     SPI4_IRQHandler                        /*  17 Interrupt 17 */
                .long     Interrupt18_IRQHandler                 /*  18 Interrupt 18 */
                .long     I2C5_IRQHandler                        /*  19 Interrupt 19 */
                .long     I2C6_IRQHandler                        /*  20 Interrupt 20 */
                .long     I2C7_IRQHandler                        /*  21 Interrupt 21 */
                .long     GPTIM3_IRQHandler                      /*  22 Interrupt 22 */
                .long     GPTIM4_IRQHandler                      /*  23 Interrupt 23 */
                .long     GPTIM5_IRQHandler                      /*  24 Interrupt 24 */
                .long     BTIM3_IRQHandler                       /*  25 Interrupt 25 */
                .long     BTIM4_IRQHandler                       /*  26 Interrupt 26 */
                .long     Interrupt27_IRQHandler                 /*  27 Interrupt 27 */
                .long     GPADC_IRQHandler                       /*  28 Interrupt 28 */
                .long     Interrupt29_IRQHandler                 /*  29 Interrupt 29 */
                .long     Interrupt30_IRQHandler                 /*  30 Interrupt 30 */
                .long     Interrupt31_IRQHandler                 /*  31 Interrupt 31 */
                .long     TSEN_IRQHandler                        /*  32 Interrupt 32 */
                .long     PTC2_IRQHandler                        /*  33 Interrupt 33 */    
                .long     Interrupt34_IRQHandler                 /*  34 Interrupt 34 */   
                .long     GPIO2_IRQHandler                       /*  35 Interrupt 35 */                  
                .long     MPI5_IRQHandler                        /*  36 Interrupt 36 */                  
                .long     Interrupt37_IRQHandler                 /*  37 Interrupt 37 */
                .long     Interrupt38_IRQHandler                 /*  38 Interrupt 38 */                  
                .long     Interrupt39_IRQHandler                 /*  39 Interrupt 39 */
                .long     Interrupt40_IRQHandler                 /*  40 Interrupt 40 */                  
                .long     LPCOMP_IRQHandler                      /*  41 Interrupt 41 */
                .long     LPTIM2_IRQHandler                      /*  42 Interrupt 42 */
                .long     LPTIM3_IRQHandler                      /*  43 Interrupt 43 */
                .long     Interrupt44_IRQHandler                 /*  44 Interrupt 44 */
                .long     Interrupt45_IRQHandler                 /*  45 Interrupt 45 */                  
                .long     LPTIM1_IRQHandler                      /*  46 Interrupt 46 */
                .long     Interrupt47_IRQHandler                 /*  47 Interrupt 47 */
                .long     IWDT_IRQHandler                        /*  48 Interrupt 48 */
                .long     RTC_IRQHandler                         /*  49 Interrupt 49 */
                .long     DMAC1_CH1_IRQHandler                   /*  50 Interrupt 50 */
                .long     DMAC1_CH2_IRQHandler                   /*  51 Interrupt 51 */
                .long     DMAC1_CH3_IRQHandler                   /*  52 Interrupt 52 */
                .long     DMAC1_CH4_IRQHandler                   /*  53 Interrupt 53 */
                .long     DMAC1_CH5_IRQHandler                   /*  54 Interrupt 54 */
                .long     DMAC1_CH6_IRQHandler                   /*  55 Interrupt 55 */
                .long     DMAC1_CH7_IRQHandler                   /*  56 Interrupt 56 */
                .long     DMAC1_CH8_IRQHandler                   /*  57 Interrupt 57 */
                .long     LCPU2HCPU_IRQHandler                   /*  58 Interrupt 58 */
                .long     USART1_IRQHandler                      /*  59 Interrupt 59 */
                .long     SPI1_IRQHandler                        /*  60 Interrupt 60 */
                .long     I2C1_IRQHandler                        /*  61 Interrupt 61 */               
                .long     EPIC_IRQHandler                        /*  62 Interrupt 62 */
                .long     LCDC1_IRQHandler                       /*  63 Interrupt 63 */
                .long     I2S1_IRQHandler                        /*  64 Interrupt 64 */
                .long     Interrupt65_IRQHandler                 /*  65 Interrupt 65 */
                .long     EFUSEC_IRQHandler                      /*  66 Interrupt 66 */
                .long     AES_IRQHandler                         /*  67 Interrupt 67 */
                .long     PTC1_IRQHandler                        /*  68 Interrupt 68 */
                .long     TRNG_IRQHandler                        /*  69 Interrupt 69 */
                .long     GPTIM1_IRQHandler                      /*  70 Interrupt 70 */
                .long     GPTIM2_IRQHandler                      /*  71 Interrupt 71 */
                .long     BTIM1_IRQHandler                       /*  72 Interrupt 72 */
                .long     BTIM2_IRQHandler                       /*  73 Interrupt 73 */
                .long     USART2_IRQHandler                      /*  74 Interrupt 74 */ 
                .long     SPI2_IRQHandler                        /*  75 Interrupt 75 */
                .long     I2C2_IRQHandler                        /*  76 Interrupt 76 */
                .long     EXTDMA_IRQHandler                      /*  77 Interrupt 77 */
                .long     I2C4_IRQHandler                        /*  78 Interrupt 78 */
                .long     SDMMC1_IRQHandler                      /*  79 Interrupt 79 */
                .long     SDMMC2_IRQHandler                      /*  80 Interrupt 80 */
                .long     NNACC_IRQHandler                       /*  81 Interrupt 81 */
                .long     PDM1_IRQHandler                        /*  82 Interrupt 82 */
                .long     CAN1_IRQHandler                        /*  83 Interrupt 83 */ 
                .long     GPIO1_IRQHandler                       /*  84 Interrupt 84 */ 
                .long     QSPI1_IRQHandler                       /*  85 Interrupt 85 */
                .long     QSPI2_IRQHandler                       /*  86 Interrupt 86 */
                .long     QSPI3_IRQHandler                       /*  87 Interrupt 87 */
                .long     FFT1_IRQHandler                        /*  88 Interrupt 88 */
                .long     EZIP_IRQHandler                        /*  89 Interrupt 89 */
                .long     AUDPRC_IRQHandler                      /*  90 Interrupt 90 */
                .long     PDM2_IRQHandler                        /*  91 Interrupt 91 */
                .long     USBC_IRQHandler                        /*  92 Interrupt 92 */
                .long     I2C3_IRQHandler                        /*  93 Interrupt 93 */
                .long     ATIM1_IRQHandler                     	 /*  94 Interrupt 94 */
                .long     USART3_IRQHandler                      /*  95 Interrupt 95 */
                .long     AUD_HP_IRQHandler                      /*  96 Interrupt 96 */
                .long     SCI_IRQHandler                         /*  97 Interrupt 97 */
                .long     FACC1_IRQHandler                     	 /*  98 Interrupt 98 */
                                                                                     
                .space   (381 * 4)                               /* Interrupts 99 .. 479 are left out */

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
Reset_Handler:  
                /*b .*/
                ldr   sp, = __StackTop     /* set stack pointer */
                ldr   r0, = __StackLimit
                msr   msplim,r0
/* CMSIS System Initialization */
                bl       SystemInit

                ldr      r4, =__copy_table_start__
                ldr      r5, =__copy_table_end__

.L_loop0:
                cmp      r4, r5
                bge      .L_loop0_done
                ldr      r1, [r4]
                ldr      r2, [r4, #4]
                ldr      r3, [r4, #8]

.L_loop0_0:
                subs     r3, #4
                ittt     ge
                ldrge    r0, [r1, r3]
                strge    r0, [r2, r3]
                bge      .L_loop0_0

                adds     r4, #12
                b        .L_loop0
.L_loop0_done:

                ldr      r3, =__zero_table_start__
                ldr      r4, =__zero_table_end__

.L_loop2:
                cmp      r3, r4
                bge      .L_loop2_done
                ldr      r1, [r3]
                ldr      r2, [r3, #4]
                movs     r0, 0

.L_loop2_0:
                subs     r2, #4
                itt      ge
                strge    r0, [r1, r2]
                bge      .L_loop2_0

                adds     r3, #8
                b        .L_loop2
.L_loop2_done:

                bl       entry
                .fnend
                .size    Reset_Handler, . - Reset_Handler

/* The default macro is not used for HardFault_Handler
 * because this results in a poor debug illusion.
 */
                .thumb_func
                .type    HardFault_Handler, %function
                .weak    HardFault_Handler
                .fnstart
HardFault_Handler:
                b        .
                .fnend
                .size    HardFault_Handler, . - HardFault_Handler

                .thumb_func
                .type    Default_Handler, %function
                .weak    Default_Handler
                .fnstart
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
                Set_Default_Handler  MemManage_Handler
                Set_Default_Handler  BusFault_Handler
                Set_Default_Handler  UsageFault_Handler
                Set_Default_Handler  SecureFault_Handler
                Set_Default_Handler  SVC_Handler
                Set_Default_Handler  DebugMon_Handler
                Set_Default_Handler  PendSV_Handler
                Set_Default_Handler  SysTick_Handler

                Set_Default_Handler      AON_IRQHandler        
                Set_Default_Handler      ble_isr    
                Set_Default_Handler      DMAC2_CH1_IRQHandler  
                Set_Default_Handler      DMAC2_CH2_IRQHandler  
                Set_Default_Handler      DMAC2_CH3_IRQHandler  
                Set_Default_Handler      DMAC2_CH4_IRQHandler  
                Set_Default_Handler      DMAC2_CH5_IRQHandler  
                Set_Default_Handler      DMAC2_CH6_IRQHandler  
                Set_Default_Handler      DMAC2_CH7_IRQHandler  
                Set_Default_Handler      DMAC2_CH8_IRQHandler  
                Set_Default_Handler      PATCH_IRQHandler      
                Set_Default_Handler      dm_isr     
                Set_Default_Handler      USART4_IRQHandler     
                Set_Default_Handler      USART5_IRQHandler     
                Set_Default_Handler      USART6_IRQHandler     
                Set_Default_Handler      bt_isr       
                Set_Default_Handler      SPI3_IRQHandler       
                Set_Default_Handler      SPI4_IRQHandler       
                Set_Default_Handler      Interrupt18_IRQHandler       
                Set_Default_Handler      I2C5_IRQHandler       
                Set_Default_Handler      I2C6_IRQHandler       
                Set_Default_Handler      I2C7_IRQHandler       
                Set_Default_Handler      GPTIM3_IRQHandler     
                Set_Default_Handler      GPTIM4_IRQHandler     
                Set_Default_Handler      GPTIM5_IRQHandler     
                Set_Default_Handler      BTIM3_IRQHandler      
                Set_Default_Handler      BTIM4_IRQHandler      
                Set_Default_Handler      Interrupt27_IRQHandler
                Set_Default_Handler      GPADC_IRQHandler     
                Set_Default_Handler      Interrupt29_IRQHandler     
                Set_Default_Handler      Interrupt30_IRQHandler    
                Set_Default_Handler      Interrupt31_IRQHandler    
                Set_Default_Handler      TSEN_IRQHandler       
                Set_Default_Handler      PTC2_IRQHandler       
                Set_Default_Handler      Interrupt34_IRQHandler      
                Set_Default_Handler      GPIO2_IRQHandler      
                Set_Default_Handler      MPI5_IRQHandler      
                Set_Default_Handler      Interrupt37_IRQHandler
                Set_Default_Handler      Interrupt38_IRQHandler
                Set_Default_Handler      Interrupt39_IRQHandler
                Set_Default_Handler      Interrupt40_IRQHandler
                Set_Default_Handler      LPCOMP_IRQHandler    
                Set_Default_Handler      LPTIM2_IRQHandler     
                Set_Default_Handler      LPTIM3_IRQHandler     
                Set_Default_Handler      Interrupt44_IRQHandler
                Set_Default_Handler      Interrupt45_IRQHandler
                Set_Default_Handler      LPTIM1_IRQHandler     
                Set_Default_Handler      Interrupt47_IRQHandler    
                Set_Default_Handler      IWDT_IRQHandler       
                Set_Default_Handler      RTC_IRQHandler        
                Set_Default_Handler      DMAC1_CH1_IRQHandler  
                Set_Default_Handler      DMAC1_CH2_IRQHandler  
                Set_Default_Handler      DMAC1_CH3_IRQHandler  
                Set_Default_Handler      DMAC1_CH4_IRQHandler  
                Set_Default_Handler      DMAC1_CH5_IRQHandler  
                Set_Default_Handler      DMAC1_CH6_IRQHandler  
                Set_Default_Handler      DMAC1_CH7_IRQHandler  
                Set_Default_Handler      DMAC1_CH8_IRQHandler  
                Set_Default_Handler      LCPU2HCPU_IRQHandler  
                Set_Default_Handler      USART1_IRQHandler     
                Set_Default_Handler      SPI1_IRQHandler       
                Set_Default_Handler      I2C1_IRQHandler       
                Set_Default_Handler      EPIC_IRQHandler       
                Set_Default_Handler      LCDC1_IRQHandler      
                Set_Default_Handler      I2S1_IRQHandler       
                Set_Default_Handler      Interrupt65_IRQHandler       
                Set_Default_Handler      EFUSEC_IRQHandler     
                Set_Default_Handler      AES_IRQHandler        
                Set_Default_Handler      PTC1_IRQHandler
                Set_Default_Handler      TRNG_IRQHandler
                Set_Default_Handler      GPTIM1_IRQHandler     
                Set_Default_Handler      GPTIM2_IRQHandler     
                Set_Default_Handler      BTIM1_IRQHandler      
                Set_Default_Handler      BTIM2_IRQHandler      
                Set_Default_Handler      USART2_IRQHandler      
                Set_Default_Handler      SPI2_IRQHandler      
                Set_Default_Handler      I2C2_IRQHandler      
                Set_Default_Handler      EXTDMA_IRQHandler     
                Set_Default_Handler      I2C4_IRQHandler     
                Set_Default_Handler      SDMMC1_IRQHandler      
                Set_Default_Handler      SDMMC2_IRQHandler       
                Set_Default_Handler      NNACC_IRQHandler      
                Set_Default_Handler      PDM1_IRQHandler        
                Set_Default_Handler      CAN1_IRQHandler    
                Set_Default_Handler      GPIO1_IRQHandler      
                Set_Default_Handler      QSPI1_IRQHandler      
                Set_Default_Handler      QSPI2_IRQHandler      
                Set_Default_Handler      QSPI3_IRQHandler      
                Set_Default_Handler      FFT1_IRQHandler  
                Set_Default_Handler      EZIP_IRQHandler
                Set_Default_Handler      AUDPRC_IRQHandler
                Set_Default_Handler      PDM2_IRQHandler
                Set_Default_Handler      USBC_IRQHandler
                Set_Default_Handler      I2C3_IRQHandler  
                Set_Default_Handler      ATIM1_IRQHandler 
                Set_Default_Handler      USART3_IRQHandler          
                Set_Default_Handler      AUD_HP_IRQHandler 
                Set_Default_Handler      SCI_IRQHandler  
                Set_Default_Handler      FACC1_IRQHandler 


                .end
