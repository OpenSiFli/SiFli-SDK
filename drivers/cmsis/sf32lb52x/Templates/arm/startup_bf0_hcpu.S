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

;//-------- <<< Use Configuration Wizard in Context Menu >>> ------------------


;<h> Stack Configuration
;  <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
;</h>

Stack_Size      EQU      0x00001000

                AREA     STACK, NOINIT, READWRITE, ALIGN=3
__stack_limit
Stack_Mem       SPACE    Stack_Size
__initial_sp


;<h> Heap Configuration
;  <o> Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
;</h>

Heap_Size       EQU      0x00001400

                IF       Heap_Size != 0                      ; Heap is provided
                AREA     HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE    Heap_Size
__heap_limit
                ENDIF


                PRESERVE8
                THUMB


; Vector Table Mapped to Address 0 at Reset

                AREA     RESET, DATA, READONLY
                EXPORT   __Vectors
                EXPORT   __Vectors_End
                EXPORT   __Vectors_Size
                IMPORT   NMI_Handler

__Vectors       DCD      __initial_sp                        ;     Top of Stack
                DCD      Reset_Handler                       ;     Reset Handler
                DCD      NMI_Handler                         ; -14 NMI Handler
                DCD      HardFault_Handler                   ; -13 Hard Fault Handler
                DCD      MemManage_Handler                   ; -12 MPU Fault Handler
                DCD      BusFault_Handler                    ; -11 Bus Fault Handler
                DCD      UsageFault_Handler                  ; -10 Usage Fault Handler
                DCD      SecureFault_Handler                 ;  -9 Secure Fault Handler
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
                DCD      ble_isr                                ;   1 Interrupt 1
                DCD      DMAC2_CH1_IRQHandler                   ;   2 Interrupt 2
                DCD      DMAC2_CH2_IRQHandler                   ;   3 Interrupt 3
                DCD      DMAC2_CH3_IRQHandler                   ;   4 Interrupt 4
                DCD      DMAC2_CH4_IRQHandler                   ;   5 Interrupt 5
                DCD      DMAC2_CH5_IRQHandler                   ;   6 Interrupt 6
                DCD      DMAC2_CH6_IRQHandler                   ;   7 Interrupt 7
                DCD      DMAC2_CH7_IRQHandler                   ;   8 Interrupt 8
                DCD      DMAC2_CH8_IRQHandler                   ;   9 Interrupt 9                    
                DCD      PATCH_IRQHandler                       ;  10 Interrupt 10
                DCD      dm_isr                                 ;  11 Interrupt 11
                DCD      USART4_IRQHandler                      ;  12 Interrupt 12
                DCD      USART5_IRQHandler                      ;  13 Interrupt 13
                DCD      SECU2_IRQHandler                       ;  14 Interrupt 14   
                DCD      bt_isr                                 ;  15 Interrupt 15
                DCD      BTIM3_IRQHandler                       ;  16 Interrupt 16
                DCD      BTIM4_IRQHandler                       ;  17 Interrupt 17
                DCD      PTC2_IRQHandler                        ;  18 Interrupt 18
                DCD      LPTIM3_IRQHandler                      ;  19 Interrupt 19
                DCD      GPIO2_IRQHandler                       ;  20 Interrupt 20
                DCD      HPSYS0_IRQHandler                      ;  21 Interrupt 21
                DCD      HPSYS1_IRQHandler                      ;  22 Interrupt 22
                DCD      Interrupt23_IRQHandler                 ;  23 Interrupt 23
                DCD      Interrupt24_IRQHandler                 ;  24 Interrupt 24
                DCD      Interrupt25_IRQHandler                 ;  25 Interrupt 25
                DCD      Interrupt26_IRQHandler                 ;  26 Interrupt 26
                DCD      Interrupt27_IRQHandler                 ;  27 Interrupt 27
                DCD      Interrupt28_IRQHandler                 ;  28 Interrupt 28
                DCD      Interrupt29_IRQHandler                 ;  29 Interrupt 29
                DCD      Interrupt30_IRQHandler                 ;  30 Interrupt 30
                DCD      Interrupt31_IRQHandler                 ;  31 Interrupt 31
                DCD      Interrupt32_IRQHandler                 ;  32 Interrupt 32
                DCD      Interrupt33_IRQHandler                 ;  33 Interrupt 33    
                DCD      Interrupt34_IRQHandler                 ;  34 Interrupt 34   
                DCD      Interrupt35_IRQHandler                 ;  35 Interrupt 35                    
                DCD      Interrupt36_IRQHandler                 ;  36 Interrupt 36                                        
                DCD      Interrupt37_IRQHandler                 ;  37 Interrupt 37
                DCD      Interrupt38_IRQHandler                 ;  38 Interrupt 38                    
                DCD      Interrupt39_IRQHandler                 ;  39 Interrupt 39
                DCD      Interrupt40_IRQHandler                 ;  40 Interrupt 40                                         
                DCD      Interrupt41_IRQHandler                 ;  41 Interrupt 41
                DCD      Interrupt42_IRQHandler                 ;  42 Interrupt 42
                DCD      Interrupt43_IRQHandler                 ;  43 Interrupt 43
                DCD      Interrupt44_IRQHandler                 ;  44 Interrupt 44
                DCD      Interrupt45_IRQHandler                 ;  45 Interrupt 45                                         
                DCD      LPTIM1_IRQHandler                      ;  46 Interrupt 46
                DCD      LPTIM2_IRQHandler                      ;  47 Interrupt 47
                DCD      PMUC_IRQHandler                        ;  48 Interrupt 48
                DCD      RTC_IRQHandler                         ;  49 Interrupt 49
                DCD      DMAC1_CH1_IRQHandler                   ;  50 Interrupt 50
                DCD      DMAC1_CH2_IRQHandler                   ;  51 Interrupt 51
                DCD      DMAC1_CH3_IRQHandler                   ;  52 Interrupt 52
                DCD      DMAC1_CH4_IRQHandler                   ;  53 Interrupt 53
                DCD      DMAC1_CH5_IRQHandler                   ;  54 Interrupt 54
                DCD      DMAC1_CH6_IRQHandler                   ;  55 Interrupt 55
                DCD      DMAC1_CH7_IRQHandler                   ;  56 Interrupt 56
                DCD      DMAC1_CH8_IRQHandler                   ;  57 Interrupt 57
                DCD      LCPU2HCPU_IRQHandler                   ;  58 Interrupt 58
                DCD      USART1_IRQHandler                      ;  59 Interrupt 59
                DCD      SPI1_IRQHandler                        ;  60 Interrupt 60
                DCD      I2C1_IRQHandler                        ;  61 Interrupt 61               
                DCD      EPIC_IRQHandler                        ;  62 Interrupt 62
                DCD      LCDC1_IRQHandler                       ;  63 Interrupt 63
                DCD      I2S1_IRQHandler                        ;  64 Interrupt 64
                DCD      GPADC_IRQHandler                       ;  65 Interrupt 65
                DCD      EFUSEC_IRQHandler                      ;  66 Interrupt 66
                DCD      AES_IRQHandler                         ;  67 Interrupt 67
                DCD      PTC1_IRQHandler                        ;  68 Interrupt 68
                DCD      TRNG_IRQHandler                        ;  69 Interrupt 69
                DCD      GPTIM1_IRQHandler                      ;  70 Interrupt 70
                DCD      GPTIM2_IRQHandler                      ;  71 Interrupt 71
                DCD      BTIM1_IRQHandler                       ;  72 Interrupt 72
                DCD      BTIM2_IRQHandler                       ;  73 Interrupt 73
                DCD      USART2_IRQHandler                      ;  74 Interrupt 74 
                DCD      SPI2_IRQHandler                        ;  75 Interrupt 75
                DCD      I2C2_IRQHandler                        ;  76 Interrupt 76
                DCD      EXTDMA_IRQHandler                      ;  77 Interrupt 77
                DCD      I2C4_IRQHandler                        ;  78 Interrupt 78
                DCD      SDMMC1_IRQHandler                      ;  79 Interrupt 79
                DCD      Interrupt80_IRQHandler                 ;  80 Interrupt 80
                DCD      Interrupt81_IRQHandler                 ;  81 Interrupt 81
                DCD      PDM1_IRQHandler                        ;  82 Interrupt 82
                DCD      Interrupt83_IRQHandler                 ;  83 Interrupt 83 
                DCD      GPIO1_IRQHandler                       ;  84 Interrupt 84 
                DCD      QSPI1_IRQHandler                       ;  85 Interrupt 85
                DCD      QSPI2_IRQHandler                       ;  86 Interrupt 86
                DCD      Interrupt87_IRQHandler                 ;  87 Interrupt 87
                DCD      Interrupt88_IRQHandler                 ;  88 Interrupt 88
                DCD      EZIP_IRQHandler                     	;  89 Interrupt 89
                DCD      AUDPRC_IRQHandler                     	;  90 Interrupt 90
                DCD      TSEN_IRQHandler                        ;  91 Interrupt 91
                DCD      USBC_IRQHandler                     	;  92 Interrupt 92
                DCD      I2C3_IRQHandler                        ;  93 Interrupt 93
                DCD      ATIM1_IRQHandler                     	;  94 Interrupt 94
                DCD      USART3_IRQHandler                     	;  95 Interrupt 95
                DCD      AUD_HP_IRQHandler                     	;  96 Interrupt 96
                DCD      Interrupt97_IRQHandler                 ;  97 Interrupt 97
                DCD      SECU1_IRQHandler                       ;  98 Interrupt 98
                
                SPACE    (13 * 4)                              ; Interrupts 99 .. 127 are left out
__Vectors_End
__Vectors_Size  EQU      __Vectors_End - __Vectors


                AREA     |.text|, CODE, READONLY

; Reset Handler


Reset_Handler   PROC
                EXPORT   Reset_Handler             [WEAK]
                IMPORT   SystemInit
                IMPORT   __main

;                B        .

                LDR      R0, =__stack_limit
                MSR      MSPLIM, R0                          ; Non-secure version of MSPLIM is RAZ/WI

                LDR      R0, =SystemInit
                BLX      R0
                LDR      R0, =__main
                BX       R0
                ENDP


; Macro to define default exception/interrupt handlers.
; Default handler are weak symbols with an endless loop.
; They can be overwritten by real handlers.
                MACRO
                Set_Default_Handler  $Handler_Name
$Handler_Name   PROC
                EXPORT   $Handler_Name             [WEAK]
                B        .
                ENDP
                MEND


; Default exception/interrupt handler

                ; Set_Default_Handler  NMI_Handler
                Set_Default_Handler  HardFault_Handler
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
                Set_Default_Handler      SECU2_IRQHandler     
                Set_Default_Handler      bt_isr       
                Set_Default_Handler      BTIM3_IRQHandler      
                Set_Default_Handler      BTIM4_IRQHandler   
                Set_Default_Handler      PTC2_IRQHandler       		
                Set_Default_Handler      LPTIM3_IRQHandler   		
                Set_Default_Handler      GPIO2_IRQHandler     
                Set_Default_Handler      HPSYS0_IRQHandler     
                Set_Default_Handler      HPSYS1_IRQHandler     				
                Set_Default_Handler      Interrupt23_IRQHandler       
                Set_Default_Handler      Interrupt24_IRQHandler       
                Set_Default_Handler      Interrupt25_IRQHandler       
                Set_Default_Handler      Interrupt26_IRQHandler       
                Set_Default_Handler      Interrupt27_IRQHandler       
                Set_Default_Handler      Interrupt28_IRQHandler       
                Set_Default_Handler      Interrupt29_IRQHandler       
                Set_Default_Handler      Interrupt30_IRQHandler       
                Set_Default_Handler      Interrupt31_IRQHandler       
                Set_Default_Handler      Interrupt32_IRQHandler       
                Set_Default_Handler      Interrupt33_IRQHandler       
                Set_Default_Handler      Interrupt34_IRQHandler       
                Set_Default_Handler      Interrupt35_IRQHandler       
                Set_Default_Handler      Interrupt36_IRQHandler       
                Set_Default_Handler      Interrupt37_IRQHandler       
                Set_Default_Handler      Interrupt38_IRQHandler       
                Set_Default_Handler      Interrupt39_IRQHandler       
                Set_Default_Handler      Interrupt40_IRQHandler       
                Set_Default_Handler      Interrupt41_IRQHandler       
                Set_Default_Handler      Interrupt42_IRQHandler       
                Set_Default_Handler      Interrupt43_IRQHandler       
                Set_Default_Handler      Interrupt44_IRQHandler       
                Set_Default_Handler      Interrupt45_IRQHandler       
                Set_Default_Handler      LPTIM1_IRQHandler          
                Set_Default_Handler      LPTIM2_IRQHandler   
                Set_Default_Handler      PMUC_IRQHandler          		
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
                Set_Default_Handler      GPADC_IRQHandler       
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
                Set_Default_Handler      Interrupt80_IRQHandler       
                Set_Default_Handler      Interrupt81_IRQHandler      
                Set_Default_Handler      PDM1_IRQHandler        
                Set_Default_Handler      Interrupt83_IRQHandler    
                Set_Default_Handler      GPIO1_IRQHandler      
                Set_Default_Handler      QSPI1_IRQHandler      
                Set_Default_Handler      QSPI2_IRQHandler      
                Set_Default_Handler      Interrupt87_IRQHandler      
                Set_Default_Handler      Interrupt88_IRQHandler  
                Set_Default_Handler      EZIP_IRQHandler
                Set_Default_Handler      AUDPRC_IRQHandler
                Set_Default_Handler      TSEN_IRQHandler
                Set_Default_Handler      USBC_IRQHandler
                Set_Default_Handler      I2C3_IRQHandler  
                Set_Default_Handler      ATIM1_IRQHandler 
                Set_Default_Handler      USART3_IRQHandler          
                Set_Default_Handler      AUD_HP_IRQHandler 
                Set_Default_Handler      Interrupt97_IRQHandler  
                Set_Default_Handler      SECU1_IRQHandler 
                
                ALIGN


; User setup Stack & Heap

                EXPORT   __stack_limit
                EXPORT   __initial_sp
                IF       Heap_Size != 0                      ; Heap is provided
                EXPORT   __heap_base
                EXPORT   __heap_limit
                ENDIF

                END
