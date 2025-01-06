/**
  ******************************************************************************
  * @file   board.h
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

#ifndef __BOARD_H__
#define __BOARD_H__

#include <rtthread.h>


#define FLASH_BASE_ADDR   (0x10000000)
#define FLASH2_BASE_ADDR  (0x64000000)
#define FLASH3_BASE_ADDR  (0x68000000)
#define FLASH4_BASE_ADDR  (0x6C000000)

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
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
