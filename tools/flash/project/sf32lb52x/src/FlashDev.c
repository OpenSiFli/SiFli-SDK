/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*********************************************************************
*            (c) 1995 - 2018 SEGGER Microcontroller GmbH             *
*                        The Embedded Experts                        *
*                           www.segger.com                           *
**********************************************************************
----------------------------------------------------------------------
File    : FlashDev.c
Purpose : Flash device description Template
--------  END-OF-HEADER  ---------------------------------------------
*/

#include "FlashOS.h"

#ifdef JLINK

#ifdef JLINK_FLASH_1
    #define FLASH_NAME  "SF32LB52X Internal Flash1"
    #define FLASH_BASE  0x10000000
    #define FLASH_SIZE  0x02000000
#elif defined(JLINK_FLASH_2)
    #define FLASH_NAME  "SF32LB52X External Flash2"
    #define FLASH_BASE  0x12000000
    #define FLASH_SIZE  0x02000000
#endif

struct FlashDevice const FlashDevice __attribute__((section("DevDscr"), used)) =
{
    ALGO_VERSION,              // Algo version
    FLASH_NAME,                // Flash device name
    ONCHIP,                    // Flash device type
    FLASH_BASE,                // Flash base address
    FLASH_SIZE,                // Total flash device size in Bytes
    // In rtconfig.h, Flash 1 (0x10000000,2M), Flash2(0x64000000, 32M), Flash3(0x68000000,32M)
#ifdef _USE_PRODUCTLINE
    PAGE_SIZE,                 // Affect the FLASH write action. Page Size (number of bytes that will be passed to ProgramPage(). May be multiple of min alignment in order to reduce overhead for calling ProgramPage multiple times
#else
    0x1000,                    // Affect the FLASH write action. Page Size (number of bytes that will be passed to ProgramPage(). May be multiple of min alignment in order to reduce overhead for calling ProgramPage multiple times
#endif
    0,                         // Reserved, should be 0
    0xFF,                      // Flash erased value
    6000,                       // Program page timeout in ms
    6000,                      // Erase sector timeout in ms
    //
    // Flash sector layout definition
    //
    PAGE_SIZE,        //Affect the FLASH erase action.
    0x000000,         // // 4096 * 4 KB = 4096  KB  Sector Size  4kB (1024 Sectors)
    0xFFFFFFFF, 0xFFFFFFFF,    // Indicates the end of the flash sector layout. Must be present.
};
#else  //for KEIL
struct FlashDevice const FlashDevice __attribute__((section("DevDscr"), used)) =
{
    ALGO_VERSION,              // Algo version
    "SF32LB54X UNI_FLASH",    // Flash device name
    ONCHIP,                    // Flash device type
    0x10000000,                // Flash base address
    0x60000000,                // Total flash device size in Bytes
    // In rtconfig.h, Flash 1 (0x10000000,2M), Flash2(0x64000000, 32M), Flash3(0x68000000,32M)
    0x2000,                       // Affect the FLASH write action. Page Size (number of bytes that will be passed to ProgramPage(). May be multiple of min alignment in order to reduce overhead for calling ProgramPage multiple times
    0,                         // Reserved, should be 0
    0xFF,                      // Flash erased value
    6000,                       // Program page timeout in ms
    3000,                      // Erase sector timeout in ms
    //
    // Flash sector layout definition
    //
    PAGE_SIZE,        //Affect the FLASH erase action.
    0x000000,         // // 4096 * 4 KB = 4096  KB  Sector Size  4kB (1024 Sectors)
    0xFFFFFFFF, 0xFFFFFFFF,    // Indicates the end of the flash sector layout. Must be present.
};
#endif  //#ifdef JLINK
