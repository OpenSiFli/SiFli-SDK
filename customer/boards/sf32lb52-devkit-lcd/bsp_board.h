/**
  ******************************************************************************
  * @file   bsp_board.h
  * @author Sifli software development team
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2019 - 2022,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef __BSP_BOARD_H__
#define __BSP_BOARD_H__

#include "rtconfig.h"
#include "drv_io.h"
#include "bf0_hal.h"
#ifdef PMIC_CTRL_ENABLE
    #include "pmic_controller.h"
#endif /* PMIC_CTRL_ENABLE */

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __CC_ARM
extern int Image$$RW_IRAM1$$ZI$$Limit;
#define HEAP_BEGIN      ((void *)&Image$$RW_IRAM1$$ZI$$Limit)
#elif __ICCARM__
#pragma section="CSTACK"
#define HEAP_BEGIN      (__segment_end("CSTACK"))
#elif defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
extern int Image$$RW_IRAM1$$ZI$$Limit;
#define HEAP_BEGIN      ((void *)&Image$$RW_IRAM1$$ZI$$Limit)
#elif defined ( __GNUC__ )
extern int __bss_end;
#define HEAP_BEGIN      ((void *)&__bss_end)
#endif

#ifdef SOC_BF0_HCPU
#ifndef HEAP_END
#define HEAP_END       (HCPU_RAM_DATA_START_ADDR + HCPU_RAM_DATA_SIZE) //TODO:
#endif
#else
#define HEAP_END       (LCPU_RAM_DATA_START_ADDR + LCPU_RAM_DATA_SIZE) //TODO:
#endif

void SystemClock_Config(void);

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H__ */
