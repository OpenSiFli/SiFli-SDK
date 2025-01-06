;/**************************************************************************//**
; * @file     startup_ARMCM33.s
; * @brief    CMSIS Core Device Startup File for
; *           ARMCM33 Device
; * @version  V5.3.1
; * @date     09. July 2018
; ******************************************************************************/
;/*
; * Copyright (c) 2009-2018 Arm Limited. All rights reserved.
; *
; * SPDX-License-Identifier: Apache-2.0
; *
; * Licensed under the Apache License, Version 2.0 (the License); you may
; * not use this file except in compliance with the License.
; * You may obtain a copy of the License at
; *
; * www.apache.org/licenses/LICENSE-2.0
; *
; * Unless required by applicable law or agreed to in writing, software
; * distributed under the License is distributed on an AS IS BASIS, WITHOUT
; * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
; * See the License for the specific language governing permissions and
; * limitations under the License.
; */

;
; The modules in this file are included in the libraries, and may be replaced
; by any user-defined modules that define the PUBLIC symbol _program_start or
; a user defined start symbol.
; To override the cstartup defined in the library, simply add your modified
; version to the workbench project.
;
; The vector table is normally located at address 0.
; When debugging in RAM, it can be located in RAM, aligned to at least 2^6.
; The name "__vector_table" has special meaning for C-SPY:
; it is where the SP start value is found, and the NVIC vector
; table register (VTOR) is initialized to this address if != 0.
;
; Cortex-M version
;

                MODULE   ?cstartup

                ;; Forward declaration of sections.
                SECTION  CSTACK:DATA:NOROOT(3)

                SECTION  .intvec:CODE:NOROOT(2)

                EXTERN   __iar_program_start
                EXTERN   SystemInit
                PUBLIC   __vector_table
                PUBLIC   __vector_table_0x1c
                PUBLIC   __Vectors
                PUBLIC   __Vectors_End
                PUBLIC   __Vectors_Size

                DATA

__vector_table
                DCD      sfe(CSTACK)                         ;     Top of Stack
                DCD      Reset_Handler                       ;     Reset Handler
                DCD      NMI_Handler                         ; -14 NMI Handler
                DCD      HardFault_Handler                   ; -13 Hard Fault Handler
                DCD      MemManage_Handler                   ; -12 MPU Fault Handler
                DCD      BusFault_Handler                    ; -11 Bus Fault Handler
                DCD      UsageFault_Handler                  ; -10 Usage Fault Handler
__vector_table_0x1c
                DCD      SecureFault_Handler                 ;  -9 Security Fault Handler
                DCD      0                                   ;     Reserved
                DCD      0                                   ;     Reserved
                DCD      0                                   ;     Reserved
                DCD      SVC_Handler                         ;  -5 SVCall Handler
                DCD      DebugMon_Handler                    ;  -4 Debug Monitor Handler
                DCD      0                                   ;     Reserved
                DCD      PendSV_Handler                      ;  -2 PendSV Handler
                DCD      SysTick_Handler                     ;  -1 SysTick Handler

                ; Interrupts
                DCD      AON_IRQHandler                         ;   0 Interrupt 0
                DCD      rwble_isr                              ;   1 Interrupt 1
                DCD      DMAC2_CH1_IRQHandler                   ;   2 Interrupt 2
                DCD      DMAC2_CH2_IRQHandler                   ;   3 Interrupt 3
                DCD      DMAC2_CH3_IRQHandler                   ;   4 Interrupt 4
                DCD      DMAC2_CH4_IRQHandler                   ;   5 Interrupt 5
                DCD      DMAC2_CH5_IRQHandler                   ;   6 Interrupt 6
                DCD      DMAC2_CH6_IRQHandler                   ;   7 Interrupt 7
                DCD      DMAC2_CH7_IRQHandler                   ;   8 Interrupt 8
                DCD      DMAC2_CH8_IRQHandler                   ;   9 Interrupt 9                    
                DCD      PATCH_IRQHandler                       ;  10 Interrupt 10
                DCD      Interrupt11_IRQHandler                 ;  11 Interrupt 11
                DCD      USART3_IRQHandler                      ;  12 Interrupt 12
                DCD      USART4_IRQHandler                      ;  13 Interrupt 13
                DCD      USART5_IRQHandler                      ;  14 Interrupt 14   
                DCD      Interrupt15_IRQHandler                 ;  15 Interrupt 15
                DCD      SPI3_IRQHandler                        ;  16 Interrupt 16
                DCD      SPI4_IRQHandler                        ;  17 Interrupt 17
                DCD      Interrupt18_IRQHandler                 ;  18 Interrupt 18
                DCD      I2C4_IRQHandler                        ;  19 Interrupt 19
                DCD      I2C5_IRQHandler                        ;  20 Interrupt 20
                DCD      I2C6_IRQHandler                        ;  21 Interrupt 21
                DCD      GPTIM3_IRQHandler                      ;  22 Interrupt 22
                DCD      GPTIM4_IRQHandler                      ;  23 Interrupt 23
                DCD      GPTIM5_IRQHandler                      ;  24 Interrupt 24
                DCD      BTIM3_IRQHandler                       ;  25 Interrupt 25
                DCD      BTIM4_IRQHandler                       ;  26 Interrupt 26
                DCD      Interrupt27_IRQHandler                 ;  27 Interrupt 27
                DCD      GPADC_IRQHandler                       ;  28 Interrupt 28
                DCD      SDADC_IRQHandler                       ;  29 Interrupt 29
                DCD      Interrupt30_IRQHandler                 ;  30 Interrupt 30
                DCD      Interrupt31_IRQHandler                 ;  31 Interrupt 31
                DCD      TSEN_IRQHandler                        ;  32 Interrupt 32
                DCD      PTC2_IRQHandler                        ;  33 Interrupt 33    
                DCD      LCDC2_IRQHandler                       ;  34 Interrupt 34   
                DCD      GPIO2_IRQHandler                       ;  35 Interrupt 35                    
                DCD      QSPI4_IRQHandler                       ;  36 Interrupt 36                                        
                DCD      Interrupt37_IRQHandler                 ;  37 Interrupt 37
                DCD      Interrupt38_IRQHandler                 ;  38 Interrupt 38                    
                DCD      Interrupt39_IRQHandler                 ;  39 Interrupt 39
                DCD      Interrupt40_IRQHandler                 ;  40 Interrupt 40                                         
                DCD      LPCOMP_IRQHandler                      ;  41 Interrupt 41
                DCD      LPTIM2_IRQHandler                      ;  42 Interrupt 42
                DCD      LPTIM3_IRQHandler                      ;  43 Interrupt 43
                DCD      Interrupt44_IRQHandler                 ;  44 Interrupt 44
                DCD      Interrupt45_IRQHandler                 ;  45 Interrupt 45                                         
                DCD      HCPU2LCPU_IRQHandler                   ;  46 Interrupt 46
                DCD      RTC_IRQHandler                         ;  47 Interrupt 47

__Vectors_End

__Vectors       EQU      __vector_table
__Vectors_Size  EQU      __Vectors_End - __Vectors


                THUMB

; Reset Handler

                PUBWEAK  Reset_Handler
                SECTION  .text:CODE:REORDER:NOROOT(2)
Reset_Handler
                ;B  .
                LDR      R0, =SystemInit
                BLX      R0

                LDR      R0, =__iar_program_start
                BX       R0


                PUBWEAK NMI_Handler
                PUBWEAK HardFault_Handler
                PUBWEAK MemManage_Handler
                PUBWEAK BusFault_Handler
                PUBWEAK UsageFault_Handler
                PUBWEAK SecureFault_Handler
                PUBWEAK SVC_Handler
                PUBWEAK DebugMon_Handler
                PUBWEAK PendSV_Handler
                PUBWEAK SysTick_Handler

                PUBWEAK AON_IRQHandler        
                PUBWEAK rwble_isr    
                PUBWEAK DMAC2_CH1_IRQHandler  
                PUBWEAK DMAC2_CH2_IRQHandler  
                PUBWEAK DMAC2_CH3_IRQHandler  
                PUBWEAK DMAC2_CH4_IRQHandler  
                PUBWEAK DMAC2_CH5_IRQHandler  
                PUBWEAK DMAC2_CH6_IRQHandler  
                PUBWEAK DMAC2_CH7_IRQHandler  
                PUBWEAK DMAC2_CH8_IRQHandler  
                PUBWEAK PATCH_IRQHandler      
                PUBWEAK Interrupt11_IRQHandler     
                PUBWEAK USART3_IRQHandler     
                PUBWEAK USART4_IRQHandler     
                PUBWEAK USART5_IRQHandler     
                PUBWEAK Interrupt15_IRQHandler       
                PUBWEAK SPI3_IRQHandler       
                PUBWEAK SPI4_IRQHandler       
                PUBWEAK Interrupt18_IRQHandler       
                PUBWEAK I2C4_IRQHandler       
                PUBWEAK I2C5_IRQHandler       
                PUBWEAK I2C6_IRQHandler       
                PUBWEAK GPTIM3_IRQHandler     
                PUBWEAK GPTIM4_IRQHandler     
                PUBWEAK GPTIM5_IRQHandler     
                PUBWEAK BTIM3_IRQHandler      
                PUBWEAK BTIM4_IRQHandler      
                PUBWEAK Interrupt27_IRQHandler
                PUBWEAK GPADC_IRQHandler     
                PUBWEAK SDADC_IRQHandler     
                PUBWEAK Interrupt30_IRQHandler    
                PUBWEAK Interrupt31_IRQHandler    
                PUBWEAK TSEN_IRQHandler       
                PUBWEAK PTC2_IRQHandler       
                PUBWEAK LCDC2_IRQHandler      
                PUBWEAK GPIO2_IRQHandler      
                PUBWEAK QSPI4_IRQHandler      
                PUBWEAK Interrupt37_IRQHandler
                PUBWEAK Interrupt38_IRQHandler
                PUBWEAK Interrupt39_IRQHandler
                PUBWEAK Interrupt40_IRQHandler
                PUBWEAK LPCOMP_IRQHandler    
                PUBWEAK LPTIM2_IRQHandler     
                PUBWEAK LPTIM3_IRQHandler     
                PUBWEAK Interrupt44_IRQHandler
                PUBWEAK Interrupt45_IRQHandler
                PUBWEAK HCPU2LCPU_IRQHandler     
                PUBWEAK RTC_IRQHandler    
                SECTION .text:CODE:REORDER:NOROOT(1)
NMI_Handler
HardFault_Handler
MemManage_Handler
BusFault_Handler
UsageFault_Handler
SecureFault_Handler
SVC_Handler
DebugMon_Handler
PendSV_Handler
SysTick_Handler

AON_IRQHandler        
rwble_isr    
DMAC2_CH1_IRQHandler  
DMAC2_CH2_IRQHandler  
DMAC2_CH3_IRQHandler  
DMAC2_CH4_IRQHandler  
DMAC2_CH5_IRQHandler  
DMAC2_CH6_IRQHandler  
DMAC2_CH7_IRQHandler  
DMAC2_CH8_IRQHandler  
PATCH_IRQHandler      
Interrupt11_IRQHandler     
USART3_IRQHandler     
USART4_IRQHandler     
USART5_IRQHandler     
Interrupt15_IRQHandler       
SPI3_IRQHandler       
SPI4_IRQHandler       
Interrupt18_IRQHandler       
I2C4_IRQHandler       
I2C5_IRQHandler       
I2C6_IRQHandler       
GPTIM3_IRQHandler     
GPTIM4_IRQHandler     
GPTIM5_IRQHandler     
BTIM3_IRQHandler      
BTIM4_IRQHandler      
Interrupt27_IRQHandler
GPADC_IRQHandler     
SDADC_IRQHandler     
Interrupt30_IRQHandler    
Interrupt31_IRQHandler    
TSEN_IRQHandler       
PTC2_IRQHandler       
LCDC2_IRQHandler      
GPIO2_IRQHandler      
QSPI4_IRQHandler      
Interrupt37_IRQHandler
Interrupt38_IRQHandler
Interrupt39_IRQHandler
Interrupt40_IRQHandler
LPCOMP_IRQHandler    
LPTIM2_IRQHandler     
LPTIM3_IRQHandler     
Interrupt44_IRQHandler
Interrupt45_IRQHandler
HCPU2LCPU_IRQHandler     
RTC_IRQHandler    
Default_Handler
                B        .


                END
