/**
  ******************************************************************************
  * @file   sec_flash.c
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

#include "board.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <rtthread.h>
#include "mem_map.h"
#include "dfu.h"
#include "drv_flash.h"
#include "log.h"

flash_read_func g_flash_read;
flash_write_func g_flash_write;
flash_erase_func g_flash_erase;
extern FLASH_HandleTypeDef *Addr2Handle(uint32_t addr);

void sec_flash_func_init()
{
    g_flash_read = NULL;
    g_flash_write = NULL;
    g_flash_erase = NULL;
}

#ifdef BSP_USING_DFU_COMPRESS
static uint8_t *sec_flash_variable_addr(int flashid, uint8_t is_target)
{
    uint32_t i = g_dfu_compress_config->img_count;
    uint8_t *dest = NULL;
    while (i--)
    {
        if (flashid == DFU_FLASH_COMPRESS &&
                g_dfu_compress_config->imgs[i].compress_img_id <= (CORE_MAX * 2))
            dest = (uint8_t *)g_dfu_compress_config->ctab[i].base;
        else if (flashid == g_dfu_compress_config->imgs[i].compress_img_id)
        {
            if (is_target)
                dest = (uint8_t *)g_dfu_compress_config->imgs[i].img.none_enc_img.target_base;
            else
                dest = (uint8_t *)g_dfu_compress_config->ctab[i].base;
        }
    }
    return dest;
}
#endif

// for testing purpose, simulate flash use memory start from FLASH_START,
// bootloader ROM should smaller than 64k (0x10000)
static uint8_t *sec_flash_addr(int flashid, uint32_t offset)
{
    uint8_t *dest;

    if (flashid == DFU_FLASH_SEC_CONFIG)
        dest = (uint8_t *)g_sec_config;
    else if (flashid == DFU_FLASH_SINGLE)
        dest = (uint8_t *)DFU_SING_IMG_START;
#ifdef BSP_USING_DFU_COMPRESS
    else if (flashid == DFU_FLASH_COMPRESS_CONFIG)
        dest = (uint8_t *)g_dfu_compress_config;
    else if (flashid == DFU_FLASH_COMPRESS)
        dest = sec_flash_variable_addr(flashid, 0);
    else if (flashid == DFU_FLASH_COMPRESS_IMG_HCPU)
        dest = (uint8_t *)HCPU_FLASH_CODE_START_ADDR;
    else if (flashid == DFU_FLASH_IMAGE)
        dest = sec_flash_variable_addr(flashid, 0);
    else if (flashid == DFU_FLASH_FONT)
        dest = sec_flash_variable_addr(flashid, 0);
    else if (flashid == DFU_FLASH_IMAGE_COMPRESS)
        dest = sec_flash_variable_addr(flashid, 0);
    else if (flashid == DFU_FLASH_FONT_COMPRESS)
        dest = sec_flash_variable_addr(flashid, 0);
#endif

    else
        dest = (uint8_t *)g_sec_config->ftab[flashid].base;

    // Flash need absolute addr
    //dest -= FLASH_START;
    dest += offset;

    return dest;
}

#ifdef BSP_USING_DFU_COMPRESS
static uint8_t *sec_flash_addr_target(int flashid, uint32_t offset)
{
    uint8_t *dest;

#ifdef BSP_USING_DFU_COMPRESS
    if (flashid == DFU_FLASH_IMAGE)
        dest = sec_flash_variable_addr(flashid, 1);
    else if (flashid == DFU_FLASH_FONT)
        dest = sec_flash_variable_addr(flashid, 1);
    else if (flashid == DFU_FLASH_IMAGE_COMPRESS)
        dest = sec_flash_variable_addr(flashid, 1);
    else if (flashid == DFU_FLASH_FONT_COMPRESS)
        dest = sec_flash_variable_addr(flashid, 1);
#endif

    else
        dest = NULL;

    // Flash need absolute addr
    //dest -= FLASH_START;
    dest += offset;

    return dest;
}
#endif

void sec_flash_write(int flashid, uint32_t offset, uint8_t *data, uint32_t size)
{
    if (flashid == DFU_FLASH_IMG_BOOT2)
    {
        memcpy((uint8_t *)(BOOTLOADER_PATCH_CODE_ADDR + offset), data, size);
    }
    else
    {
        uint8_t *dest = sec_flash_addr(flashid, offset);
        LOG_D("write %x %d", dest, size);
        if (flashid == DFU_FLASH_SEC_CONFIG)    // Only cache for DFU_FLASH_SEC_CONFIG
        {
            RT_ASSERT((offset + size) < FLASH_SEC_CACHE_SIZE);
            memcpy((uint8_t *)(FLASH_SEC_CACHE + offset), data, size);
        }
        if ((g_flash_write == NULL)
                || (g_flash_write((rt_uint32_t)dest, (const int8_t *)data, size) < 0))
        {
            FLASH_HandleTypeDef *fhandle = Addr2Handle((uint32_t)dest);

            if (fhandle == NULL)    // get nor flash handler fail, it should be nand
                rt_nand_write((rt_uint32_t)dest, data, size);
            else
                rt_flash_write((rt_uint32_t)dest, data, size);
        }
    }
}

#ifdef BSP_USING_DFU_COMPRESS
void sec_flash_write_target(int flashid, uint32_t offset, uint8_t *data, uint32_t size)
{

    uint8_t *dest = sec_flash_addr_target(flashid, offset);
    LOG_D("write target %x %d", dest, size);

    if ((g_flash_write == NULL)
            || (g_flash_write((rt_uint32_t)dest, (const int8_t *)data, size) < 0))
    {
        FLASH_HandleTypeDef *fhandle = Addr2Handle((uint32_t)dest);

        if (fhandle == NULL)    // get nor flash handler fail, it should be nand
            rt_nand_write((rt_uint32_t)dest, data, size);
        else
            rt_flash_write((rt_uint32_t)dest, data, size);
    }
}
#endif

void sec_flash_erase(int flashid, uint32_t offset, uint32_t size)
{
    uint8_t *dest = sec_flash_addr(flashid, offset);
    LOG_D("erase %x %d", dest, size);
    if (dest == NULL)
        RT_ASSERT(0);

    if (flashid == DFU_FLASH_IMG_BOOT2)
        memset((uint8_t *)BOOTLOADER_PATCH_CODE_ADDR, 0, size);
    else if (g_flash_erase == NULL || g_flash_erase((rt_uint32_t)dest, size) < 0)
    {
        size = (size + SPI_NOR_SECT_SIZE * 2 - 1) & (~(SPI_NOR_SECT_SIZE * 2 - 1));
        FLASH_HandleTypeDef *fhandle = Addr2Handle((uint32_t)dest);

        if (fhandle == NULL)    // get nor flash handler fail, it should be nand
            rt_nand_erase((rt_uint32_t)dest, size);
        else
            rt_flash_erase((rt_uint32_t)dest, size);
    }
}

#ifdef BSP_USING_DFU_COMPRESS
void sec_flash_erase_target(int flashid, uint32_t offset, uint32_t size)
{
    uint8_t *dest = sec_flash_addr_target(flashid, offset);
    LOG_D("erase target %x %d", dest, size);
    if (dest == NULL)
        RT_ASSERT(0);

    if (g_flash_erase == NULL || g_flash_erase((rt_uint32_t)dest, size) < 0)
    {
        FLASH_HandleTypeDef *fhandle = Addr2Handle((uint32_t)dest);

        if (fhandle == NULL)    // get nor flash handler fail, it should be nand
            rt_nand_erase((rt_uint32_t)dest, size);
        else
            rt_flash_erase((rt_uint32_t)dest, size);
    }
}
#endif


int sec_flash_read(int flashid, uint32_t offset, uint8_t *data, uint32_t size)
{
    if (g_flash_read == NULL || g_flash_read((rt_uint32_t)offset, (const int8_t *)data, size) < 0)
    {
        if (flashid == DFU_FLASH_IMG_BOOT2)
        {
            memcpy(data, (uint8_t *)(BOOTLOADER_PATCH_CODE_ADDR + offset), size);
        }
        else
        {
            uint8_t *src = sec_flash_addr(flashid, offset);
            FLASH_HandleTypeDef *fhandle = Addr2Handle((uint32_t)src);

            if (fhandle == NULL)    // get nor flash handler fail, it should be nand
            {
                rt_nand_read((uint32_t)src, data, size);
            }
            else
                rt_flash_read((rt_uint32_t)src, data, size);
        }
    }
    return size;
}

void sec_flash_init()
{
    uint32_t start_addr = NULL;
    uint8_t is_enable = flash_is_enabled(0);

    // Boot order will be : flash1 -> flash2
    if (is_enable)
        start_addr = QSPI1_MEM_BASE;
#ifdef BSP_ENABLE_QSPI5
    else if (flash_is_enabled(4))
        start_addr = QSPI5_MEM_BASE;
#endif
#ifdef BSP_ENABLE_QSPI2
    else
    {
        is_enable = flash_is_enabled(1);
        if (is_enable)
            start_addr = QSPI2_MEM_BASE;
    }
#endif

    RT_ASSERT(start_addr != NULL);
    g_sec_config = (struct sec_configuration *)start_addr;
    sec_flash_read(DFU_FLASH_SEC_CONFIG, 0, (uint8_t *)FLASH_SEC_CACHE, sizeof(struct sec_configuration));
}


extern uint8_t dfu_data[];
void sec_flash_update(int flashid, uint32_t offset, uint8_t *data, uint32_t size)
{
    if (flashid == DFU_FLASH_SEC_CONFIG)  // Only for DFU_FLASH_SEC_CONFIG flash area
    {
        //uint8_t *dest = sec_flash_addr(flashid, offset);
        int i, dirty = 0;
        uint32_t *p = (uint32_t *)(FLASH_SEC_CACHE + offset);
        for (i = 0; i < (size >> 2); i++, p++)
        {
            if ((*p) != FLASH_UNINIT_32)
            {
                dirty = 1;
                break;
            }
        }
        RT_ASSERT((offset + size) < FLASH_SEC_CACHE_SIZE);
        memcpy((uint8_t *)(FLASH_SEC_CACHE + offset), data, size);
        if (dirty)
        {
            offset = offset & (~(SPI_NOR_SECT_SIZE * 2 - 1));
            sec_flash_erase(flashid, offset, SPI_NOR_SECT_SIZE * 2);
            sec_flash_write(flashid, offset, (uint8_t *)(FLASH_SEC_CACHE + offset), SPI_NOR_SECT_SIZE * 2);
        }
        else
        {
            sec_flash_write(flashid, offset, data, size);
        }
    }
}

// Move secure key to flash5
void sec_flash_key_update(void)
{
#ifdef BSP_ENABLE_MPI5
    uint32_t count = 0xFFFFF;
    // Move secure key to lpaon
    hwp_hpsys_rcc->ENR1 |= HPSYS_RCC_ENR1_PTC1;
    hwp_ptc1->TCR0 = PTC_TCR0_SWTRIG | 0x08;

    while (((hwp_ptc1->ISR & PTC_ISR_TCIF0) == 0) && count)
        count--;

    // No need to 2nd move since 1st failed
    if (!count)
        return;

    hwp_ptc1->ICR |= PTC_ICR_CTCIF0;
    count = 0xFFFFF;

    // Move lpaon to mpi5
    hwp_lpsys_rcc->ENR1 |= LPSYS_RCC_ENR1_PTC2;
    hwp_ptc2->TCR0 = PTC_TCR0_SWTRIG | 0x08;

    while (((hwp_ptc2->ISR & PTC_ISR_TCIF0) == 0) && count)
        count--;

    hwp_ptc2->ICR |= PTC_ICR_CTCIF0;
#endif
}

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
