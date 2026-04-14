/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __BOARD_H__
#define __BOARD_H__

#include <rtthread.h>

#define FLASH_BASE_ADDR   (0x10000000)
#define FLASH2_BASE_ADDR  (0x64000000)
#define FLASH3_BASE_ADDR  (0x68000000)
#define FLASH4_BASE_ADDR  (0x6C000000)
#define SDMMC1_MEM_BASE   (0x68000000)
#define SDMMC2_MEM_BASE   (0xA0000000)

void rt_hw_board_init(void);
rt_uint8_t *rt_hw_sram_init(void);

/* SD Card init function */
void rt_hw_sdcard_init(void);

int rt_hw_mtd_nand_init(void);
int sst25vfxx_mtd_init(const char *, unsigned int, unsigned int);
void pcap_netif_hw_init(void);
void rt_platform_init(void);
void rt_hw_usart_init(void);
void rt_hw_serial_init(void);
void rt_hw_sdl_start(void);
void rt_hw_win32_low_cpu(void);

void rt_hw_exit(void);

#ifdef CUSTOM_MEM_MAP
    #include "flash_map.h"
#endif /* CUSTOM_MEM_MAP */

#endif

