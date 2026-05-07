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

#include "SdioOS.h"

#ifdef JLINK

#ifdef JLINK_SDIO_1
    #define SDIO_NAME  "SF32LB58X External SDIO1"
    #define SDIO_BASE  0x68000000
    #define SDIO_SIZE  0x38000000
#elif defined(JLINK_SDIO_2)
    #define SDIO_NAME  "SF32LB58X External SDIO2"
    #define SDIO_BASE  0xa0000000
    #define SDIO_SIZE  0x40000000
#endif

struct FlashDevice const FlashDevice __attribute__((section("DevDscr"), used)) =
{
    ALGO_VERSION,              // Algo version
    SDIO_NAME,                // Flash device name
    ONCHIP,                    // Flash device type
    SDIO_BASE,                // Flash base address
    SDIO_SIZE,                // Total flash device size in Bytes
    PAGE_SIZE,                       // Affect the FLASH write action. Page Size (number of bytes that will be passed to ProgramPage(). May be multiple of min alignment in order to reduce overhead for calling ProgramPage multiple times
    0,                         // Reserved, should be 0
    0x37,                      // Flash erased value
    6000,                       // Program page timeout in ms
    6000,                      // Erase sector timeout in ms
    //
    // Flash sector layout definition
    //
    PAGE_SIZE,        //Affect the FLASH erase action.
    0x000000,         // // 4096 * 4 KB = 4096  KB  Sector Size  4kB (1024 Sectors)
    0xFFFFFFFF, 0xFFFFFFFF,    // Indicates the end of the flash sector layout. Must be present.
};
#endif  //#ifdef JLINK
