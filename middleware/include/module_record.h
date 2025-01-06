/**
  ******************************************************************************
  * @file   module_record.h
  * @author Sifli software development team
  * @brief Metrics Collector source.
  *
  ******************************************************************************
*/
/*
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

#ifndef _MODULE_RECORD_H_
#define _MODULE_RECORD_H_

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif


#define RECORD_MODULE_POS  (0)
#define RECORD_MODULE_MASK  (0xFFFFUL << RECORD_MODULE_POS)

#define RECORD_PSRAM_HALF_STATUS_POS  (16)
#define RECORD_PSRAM_HALF_STATUS_MASK  (0x1UL << RECORD_PSRAM_HALF_STATUS_POS)

#define RECORD_WDT_IRQ_FLAG_POS  (17)
#define RECORD_WDT_IRQ_FLAG_MASK  (0x1UL << RECORD_WDT_IRQ_FLAG_POS)

#define RECORD_CRASH_SAVE_PROCESS_POS  (18)
#define RECORD_CRASH_SAVE_PROCESS_MASK  (0xFUL << RECORD_CRASH_SAVE_PROCESS_POS)

#define RECORD_CRASH_FLASG_POS  (31)
#define RECORD_CRASH_FLASG_MASK  (0x1UL << RECORD_CRASH_FLASG_POS)

typedef struct
{
    uint32_t module : 16;
    uint32_t psram_half_status : 1;
    uint32_t wdt_irq_flag : 1;
    uint32_t crash_save_process : 4;
    uint32_t reserve : 9;
    uint32_t crash_flag : 1;
} record_module_t;


typedef enum
{
    RECORD_MODULE_TYPE_START = 0x0000,

    RECORD_PM_START = 0x1100,

    RECORD_PM_SCENARIO_START = 0x1101,

    RECORD_PM_SCENARIO_HIGH_SPEED_CLK_CONFIG = 0x1101,
    RECORD_PM_SCENARIO_HIGH_SPEED_PSRAM_INIT = 0x1102,
    RECORD_PM_SCENARIO_HIGH_SPEED_FLASH_INIT = 0x1103,
    RECORD_PM_SCENARIO_HIGH_SPEED_END        = 0x1104,


    RECORD_PM_SCENARIO_MEDIUM_SPEED_CLK_CONFIG = 0x1105,
    RECORD_PM_SCENARIO_MEDIUM_SPEED_PSRAM_INIT = 0x1106,
    RECORD_PM_SCENARIO_MEDIUM_SPEED_FLASH_INIT = 0x1107,
    RECORD_PM_SCENARIO_END = 0x1108,

    RECORD_PM_HCPU_RESUME_START = 0x1109,
    RECORD_PM_HCPU_RESUME_END = 0x110A,

    RECORD_PM_HCPU_SUSPEND_START = 0x110B,
    RECORD_PM_HCPU_SUSPEND_END = 0x110C,

    RECORD_PM_GUI_SUSPEND_CLOSE_DISPLAY = 0x110D,
    RECORD_PM_GUI_SUSPEND_SLEEP = 0x110E,
    RECORD_PM_GUI_SUSPEND_WAKEUP = 0x110F,
    RECORD_PM_GUI_SUSPEND_OPEN_DISPLAY = 0x1110,
    RECORD_PM_GUI_SUSPEND_END = 0x1111,

    RECORD_PM_DEEP_IO_DOWN = 0x1112,
    RECORD_PM_DEEP_ENABLE_DLL1 = 0x1113,
    RECORD_PM_DEEP_ENABLE_DLL2 = 0x1114,
    RECORD_PM_DEEP_IO_UP = 0x1115,

    RECORD_PM_END = 0x11FF,

    RECORD_FLASH_START = 0x1200,
    RECORD_FLASH_INIT_WITHID = 0x1201,
    RECORD_FLASH_HAL_INIT = 0x1202,
    RECORD_FLASH_END = 0x12FF,


    RECORD_PSRAM_START = 0x1300,

    RECORD_PSRAM_ENTER_LOW_POWER_BEGIN = 0x1301,
    RECORD_PSRAM_ENTER_LOW_POWER_END = 0x1302,

    RECORD_PSRAM_EXIT_LOW_POWER_BEGIN = 0x1303,
    RECORD_PSRAM_EXIT_LOW_POWER_END = 0x1304,

    RECORD_PSRAM_END = 0x13FF,

} record_module_type_t;


typedef enum
{
    RECORD_CRASH_SAVE_START = 0x00,
    RECORD_CRASH_SAVE_WDT_START = 0x01,
    RECORD_CRASH_SAVE_VAR = 0x02,
    RECORD_CRASH_SAVE_STATIC = 0x03,
    RECORD_CRASH_SAVE_STACK = 0x04,
    RECORD_CRASH_SAVE_LOG = 0x05,
    RECORD_CRASH_SAVE_REGISTER = 0x06,
    RECORD_CRASH_SAVE_LCPU_STATIC = 0x07,
    RECORD_CRASH_SAVE_LCPU_DTCM = 0x08,

    RECORD_CRASH_SAVE_HCPU_HEAP = 0x09,
    RECORD_CRASH_SAVE_STACK_1 = 0x0A,
    RECORD_CRASH_SAVE_HCPU_HEAP_1 = 0x0A,
    RECORD_CRASH_SAVE_LOG_1 = 0x0A,
    RECORD_CRASH_SAVE_END = 0x0F
} record_crash_save_process_t;


void sifli_record_clear(void);
void sifli_record_psram_half_status(uint8_t status);
void sifli_record_module(uint16_t module);
void sifli_record_wdt_irq_status(uint8_t value);
void sifli_record_crash_save_process(uint8_t value);
void sifli_record_crash_status(uint8_t status);

#ifdef __cplusplus
}
#endif



#endif /* _MODULE_RECORD_H_ */
