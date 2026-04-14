/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-5      SummerGift   first version
 */

#ifndef __BOARD_H__
#define __BOARD_H__

#include <rtconfig.h>
#include <register.h>
#include "drv_io.h"

#ifdef __cplusplus
extern "C" {
#endif

//#define SF32_FLASH_START_ADRESS     ((uint32_t)0x08000000)
//#define SF32_FLASH_SIZE             (256 * 1024)
//#define SF32_FLASH_END_ADDRESS      ((uint32_t)(SF32_FLASH_START_ADRESS + SF32_FLASH_SIZE))

#ifndef ALIGN
#define ALIGN(n)                    __attribute__((aligned(n)))
#endif /* ALIGN */

/* Internal SRAM memory size[Kbytes] <16-256>, Default: 64*/
#define SF32_SRAM_SIZE      BOOTLOADER_RAM_DATA_SIZE
#define SF32_SRAM_END       (BOOTLOADER_RAM_DATA_START_ADDR + BOOTLOADER_RAM_DATA_SIZE) //TODO:

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

#define HEAP_END        SF32_SRAM_END

void SystemClock_Config(void);

void boot_sd1_power_on(void);
void boot_sd2_power_on(void);
#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H__ */
