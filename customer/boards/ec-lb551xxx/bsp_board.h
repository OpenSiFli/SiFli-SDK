/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-5      SummerGift   first version
 */

#ifndef __BSP_BOARD_H__
#define __BSP_BOARD_H__

#include <rtconfig.h>
#include "bf0_hal.h"
#ifdef __cplusplus
extern "C" {
#endif

//#define SF32_FLASH_START_ADRESS     ((uint32_t)0x08000000)
//#define SF32_FLASH_SIZE             (256 * 1024)
//#define SF32_FLASH_END_ADDRESS      ((uint32_t)(SF32_FLASH_START_ADRESS + SF32_FLASH_SIZE))

/* Internal SRAM memory size[Kbytes] <16-256>, Default: 64*/
#ifdef SOC_BF0_HCPU
#define SF32_SRAM_SIZE      (HCPU_RAM_DATA_SIZE)
#define SF32_SRAM_END       (HCPU_RAM_DATA_START_ADDR + HCPU_RAM_DATA_SIZE) //TODO:
#else
#define SF32_SRAM_SIZE      (LCPU_RAM_DATA_SIZE)
#define SF32_SRAM_END       (LCPU_RAM_DATA_START_ADDR + LCPU_RAM_DATA_SIZE) //TODO:
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

#ifndef HEAP_END
#define HEAP_END        (SF32_SRAM_END)
#endif

void SystemClock_Config(void);
void sifli_mbox_init(void);
#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H__ */
