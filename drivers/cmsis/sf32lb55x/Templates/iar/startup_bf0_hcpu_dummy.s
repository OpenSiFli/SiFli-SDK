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

                DATA

__vector_table
                DCD      sfe(CSTACK)                         ;     Top of Stack
                DCD      Reset_Handler                       ;     Reset Handler
                DCD      Default_Handler                         ; -14 NMI Handler
                DCD      Default_Handler                   ; -13 Hard Fault Handler
                DCD      Default_Handler                   ; -12 MPU Fault Handler
                DCD      Default_Handler                    ; -11 Bus Fault Handler
                DCD      Default_Handler                  ; -10 Usage Fault Handler
__vector_table_0x1c
                DCD      Default_Handler                 ;  -9 Security Fault Handler
                DCD      0                                   ;     Reserved
                DCD      0                                   ;     Reserved
                DCD      0                                   ;     Reserved
                DCD      Default_Handler                         ;  -5 SVCall Handler
                DCD      Default_Handler                    ;  -4 Debug Monitor Handler
                DCD      0                                   ;     Reserved
                DCD      Default_Handler                      ;  -2 PendSV Handler
                DCD      Default_Handler                     ;  -1 SysTick Handler

                ; Interrupts
                DCD      Default_Handler                         ;   0 Interrupt 0
                DCD      Default_Handler                     ;   1 Interrupt 1
                DCD      Default_Handler                   ;   2 Interrupt 2
                DCD      Default_Handler                   ;   3 Interrupt 3
                DCD      Default_Handler                   ;   4 Interrupt 4
                DCD      Default_Handler                   ;   5 Interrupt 5
                DCD      Default_Handler                   ;   6 Interrupt 6
                DCD      Default_Handler                   ;   7 Interrupt 7
                DCD      Default_Handler                   ;   8 Interrupt 8
                DCD      Default_Handler                   ;   9 Interrupt 9                    
                DCD      Default_Handler                       ;  10 Interrupt 10
                DCD      Default_Handler                 ;  11 Interrupt 11
                DCD      Default_Handler                      ;  12 Interrupt 12
                DCD      Default_Handler                      ;  13 Interrupt 13
                DCD      Default_Handler                      ;  14 Interrupt 14   
                DCD      Default_Handler                 ;  15 Interrupt 15
                DCD      Default_Handler                        ;  16 Interrupt 16
                DCD      Default_Handler                        ;  17 Interrupt 17
                DCD      Default_Handler                 ;  18 Interrupt 18
                DCD      Default_Handler                        ;  19 Interrupt 19
                DCD      Default_Handler                        ;  20 Interrupt 20
                DCD      Default_Handler                        ;  21 Interrupt 21
                DCD      Default_Handler                      ;  22 Interrupt 22
                DCD      Default_Handler                      ;  23 Interrupt 23
                DCD      Default_Handler                      ;  24 Interrupt 24
                DCD      Default_Handler                       ;  25 Interrupt 25
                DCD      Default_Handler                       ;  26 Interrupt 26
                DCD      Default_Handler                 ;  27 Interrupt 27
                DCD      Default_Handler                       ;  28 Interrupt 28
                DCD      Default_Handler                       ;  29 Interrupt 29
                DCD      Default_Handler                 ;  30 Interrupt 30
                DCD      Default_Handler                 ;  31 Interrupt 31
                DCD      Default_Handler                        ;  32 Interrupt 32
                DCD      Default_Handler                        ;  33 Interrupt 33    
                DCD      Default_Handler                       ;  34 Interrupt 34   
                DCD      Default_Handler                       ;  35 Interrupt 35                    
                DCD      Default_Handler                       ;  36 Interrupt 36                                        
                DCD      Default_Handler                 ;  37 Interrupt 37
                DCD      Default_Handler                 ;  38 Interrupt 38                    
                DCD      Default_Handler                 ;  39 Interrupt 39
                DCD      Default_Handler                 ;  40 Interrupt 40                                         
                DCD      Default_Handler                      ;  41 Interrupt 41
                DCD      Default_Handler                      ;  42 Interrupt 42
                DCD      Default_Handler                      ;  43 Interrupt 43
                DCD      Default_Handler                 ;  44 Interrupt 44
                DCD      Default_Handler                 ;  45 Interrupt 45                                         
                DCD      Default_Handler                      ;  46 Interrupt 46
                DCD      Default_Handler                 ;  47 Interrupt 47
                DCD      Default_Handler                 ;  48 Interrupt 48
                DCD      Default_Handler                         ;  49 Interrupt 49
                DCD      Default_Handler                   ;  50 Interrupt 50
                DCD      Default_Handler                   ;  51 Interrupt 51
                DCD      Default_Handler                   ;  52 Interrupt 52
                DCD      Default_Handler                   ;  53 Interrupt 53
                DCD      Default_Handler                   ;  54 Interrupt 54
                DCD      Default_Handler                   ;  55 Interrupt 55
                DCD      Default_Handler                   ;  56 Interrupt 56
                DCD      Default_Handler                   ;  57 Interrupt 57
                DCD      Default_Handler                   ;  58 Interrupt 58
                DCD      Default_Handler                      ;  59 Interrupt 59
                DCD      Default_Handler                        ;  60 Interrupt 60
                DCD      Default_Handler                        ;  61 Interrupt 61               
                DCD      Default_Handler                        ;  62 Interrupt 62
                DCD      Default_Handler                       ;  63 Interrupt 63
                DCD      Default_Handler                        ;  64 Interrupt 64
                DCD      Default_Handler                        ;  65 Interrupt 65
                DCD      Default_Handler                      ;  66 Interrupt 66
                DCD      Default_Handler                         ;  67 Interrupt 67
                DCD      Default_Handler                        ;  68 Interrupt 68
                DCD      Default_Handler                        ;  69 Interrupt 69
                DCD      Default_Handler                      ;  70 Interrupt 70
                DCD      Default_Handler                      ;  71 Interrupt 71
                DCD      Default_Handler                       ;  72 Interrupt 72
                DCD      Default_Handler                       ;  73 Interrupt 73
                DCD      Default_Handler                      ;  74 Interrupt 74 
                DCD      Default_Handler                        ;  75 Interrupt 75
                DCD      Default_Handler                        ;  76 Interrupt 76
                DCD      Default_Handler                      ;  77 Interrupt 77
                DCD      Default_Handler                      ;  78 Interrupt 78
                DCD      Default_Handler                      ;  79 Interrupt 79
                DCD      Default_Handler                      ;  80 Interrupt 80
                DCD      Default_Handler                       ;  81 Interrupt 81
                DCD      Default_Handler                        ;  82 Interrupt 82
                DCD      Default_Handler                     ;  83 Interrupt 83 
                DCD      Default_Handler                       ;  84 Interrupt 84 
                DCD      Default_Handler                       ;  85 Interrupt 85
                DCD      Default_Handler                       ;  86 Interrupt 86
                DCD      Default_Handler                       ;  87 Interrupt 87
                DCD      Default_Handler                     	;  88 Interrupt 88
                DCD      Default_Handler                        ;  89 Interrupt 89
                DCD      Default_Handler                     	;  90 Interrupt 90
                DCD      Default_Handler                        ;  91 Interrupt 91
                DCD      Default_Handler               	;  92 Interrupt 92
                DCD      Default_Handler                 ;  93 Interrupt 93
                DCD      Default_Handler               	;  94 Interrupt 94
                DCD      Default_Handler                 ;  95 Interrupt 95    

                DS32    (384)                                   ; Interrupts 96 .. 480 are left out


                THUMB

; Reset Handler

                PUBWEAK  Reset_Handler
                SECTION  .text:CODE:REORDER:NOROOT(2)
Reset_Handler
                ;B .
                LDR      R0, =sfb(CSTACK)
                MSR      MSPLIM, R0                          ; Non-secure version of MSPLIM is RAZ/WI                
                LDR      R0, =SystemInit
                BLX      R0
                
                LDR      R0, =__iar_program_start
                BX       R0



                SECTION .text:CODE:REORDER:NOROOT(1)

Default_Handler
                B        .


                END
