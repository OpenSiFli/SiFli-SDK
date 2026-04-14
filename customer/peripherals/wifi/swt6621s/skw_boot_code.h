/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef SKW_BOOT_CODE_H
#define SKW_BOOT_CODE_H
#include "rtconfig.h"
#ifdef SKW_BOOT_MODE_RAM
    extern const unsigned char boot_dram_buff[192492];
    extern const unsigned char boot_iram_buff[357024];
    extern const unsigned char seekwave_buff[2372];
    #define BOOT_DRAM_SIZE 192492
    #define BOOT_IRAM_SIZE 357024
    #define SEEKWAVE_SIZE 2372
#endif
#endif // SKW_BOOT_CODE_H