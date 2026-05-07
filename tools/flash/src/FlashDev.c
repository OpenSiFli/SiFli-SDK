/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifdef FLASH_DEVICE_USE_SDIO_OS
#include "SdioOS.h"
#else
#include "FlashOS.h"
#endif

#ifndef FLASH_DEVICE_NAME
#error "FLASH_DEVICE_NAME must be defined"
#endif

#ifndef FLASH_DEVICE_BASE
#error "FLASH_DEVICE_BASE must be defined"
#endif

#ifndef FLASH_DEVICE_SIZE
#error "FLASH_DEVICE_SIZE must be defined"
#endif

#ifndef FLASH_DEVICE_PAGE_SIZE_VALUE
#error "FLASH_DEVICE_PAGE_SIZE_VALUE must be defined"
#endif

#ifndef FLASH_DEVICE_SECTOR_SIZE_VALUE
#error "FLASH_DEVICE_SECTOR_SIZE_VALUE must be defined"
#endif

#ifndef FLASH_DEVICE_ERASED_VALUE
#error "FLASH_DEVICE_ERASED_VALUE must be defined"
#endif

#ifndef FLASH_DEVICE_TIMEOUT_PROG
#define FLASH_DEVICE_TIMEOUT_PROG 6000
#endif

#ifndef FLASH_DEVICE_TIMEOUT_ERASE
#define FLASH_DEVICE_TIMEOUT_ERASE 6000
#endif

struct FlashDevice const FlashDevice __attribute__((section("DevDscr"), used)) =
{
    ALGO_VERSION,
    FLASH_DEVICE_NAME,
    ONCHIP,
    FLASH_DEVICE_BASE,
    FLASH_DEVICE_SIZE,
    FLASH_DEVICE_PAGE_SIZE_VALUE,
    0,
    FLASH_DEVICE_ERASED_VALUE,
    FLASH_DEVICE_TIMEOUT_PROG,
    FLASH_DEVICE_TIMEOUT_ERASE,
    FLASH_DEVICE_SECTOR_SIZE_VALUE,
    0x000000,
    0xFFFFFFFF, 0xFFFFFFFF,
};
