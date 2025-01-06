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

MCPU_AON        EQU      0x4005f000

Stack_Size      EQU      0x00000100

                AREA     STACK, NOINIT, READWRITE, ALIGN=3
__stack_limit2
Stack_Mem       SPACE    Stack_Size
__initial_sp2


;<h> Heap Configuration
;  <o> Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
;</h>

Heap_Size       EQU      0x00000100

                IF       Heap_Size != 0                      ; Heap is provided
                AREA     HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base2
Heap_Mem        SPACE    Heap_Size
__heap_limit2
                ENDIF


                PRESERVE8
                THUMB


; Vector Table Mapped to Address 0 at Reset

                AREA     RESET2, DATA, READONLY
                EXPORT   __Vectors2
                EXPORT   __Vectors_End2
                EXPORT   __Vectors_Size2

__Vectors2      DCD      __initial_sp2                       ;     Top of Stack
                DCD      Reset_Handler2                      ;     Reset Handler
                DCD      0                         ; -14 NMI Handler
                DCD      0                   ; -13 Hard Fault Handler
                DCD      0                   ; -12 MPU Fault Handler
                DCD      0                    ; -11 Bus Fault Handler
                DCD      0                  ; -10 Usage Fault Handler
                DCD      0                 ;  -9 Secure Fault Handler
                DCD      0                                   ;     Reserved
                DCD      0                                   ;     Reserved
                DCD      0                                   ;     Reserved
                DCD      0                         ;  -5 SVCall Handler
                DCD      0                    ;  -4 Debug Monitor Handler
                DCD      0                                   ;     Reserved
                DCD      0                      ;  -2 PendSV Handler
                DCD      0                     ;  -1 SysTick Handler

                ; Interrupts
                DCD      0                   ;   0 Interrupt 0
                DCD      0                      ;   1 Interrupt 1
                DCD      0                      ;   2 Interrupt 2
                DCD      0                      ;   3 Interrupt 3
                DCD      0                      ;   4 Interrupt 4
                DCD      0                 ;   5 Interrupt 5
                DCD      0   ;   6 Interrupt 6
                DCD      0                 ;   7 Interrupt 7
                DCD      0                 ;   8 Interrupt 8
                DCD      0                 ;   9 Interrupt 9
                DCD      0                ;  10 Interrupt 10
                DCD      0                ;  11 Interrupt 11
                DCD      0                ;  12 Interrupt 12
                DCD      0                ;  13 Interrupt 13
                DCD      0                ;  14 Interrupt 14
                DCD      0                ;  15 Interrupt 15
                DCD      0                ;  16 Interrupt 16
                DCD      0                ;  17 Interrupt 17
                DCD      0                ;  18 Interrupt 18
                DCD      0                ;  19 Interrupt 19
                DCD      0               ;  20 Interrupt 20
                DCD      0                ;  21 Interrupt 21
                DCD      0                ;  22 Interrupt 22
                DCD      0                ;  23 Interrupt 23
                DCD      0                ;  24 Interrupt 24
                DCD      0                ;  25 Interrupt 25
                DCD      0                ;  26 Interrupt 26
                DCD      0                ;  27 Interrupt 27
                DCD      0                ;  28 Interrupt 28
                DCD      0                ;  29 Interrupt 29
                DCD      0                ;  30 Interrupt 30
                DCD      0                ;  31 Interrupt 31
                DCD      0                ;  32 Interrupt 32
                DCD      0                ;  33 Interrupt 33
                DCD      0              	; 34 Interrupt 34
                DCD      0                ;  35 Interrupt 35
                DCD      0              	;  36 Interrupt 36
                DCD      0                ;  37 Interrupt 37
                DCD      0                ;  38 Interrupt 38
                DCD      0                ;  39 Interrupt 39
                SPACE    (440 * 4)                             ; Interrupts 80 .. 479 are left out
__Vectors_End2
__Vectors_Size2  EQU      __Vectors_End2 - __Vectors2


                AREA     |.text|, CODE, READONLY

; Reset Handler


Reset_Handler2  PROC
                IMPORT   lcpu_rom_jump

                LDR      R0, =lcpu_rom_jump
                BLX      R0                    
                ENDP

; User setup Stack & Heap

                EXPORT   __stack_limit2
                EXPORT   __initial_sp2
                IF       Heap_Size != 0                      ; Heap is provided
                EXPORT   __heap_base2
                EXPORT   __heap_limit2
                ENDIF
                    
                END
