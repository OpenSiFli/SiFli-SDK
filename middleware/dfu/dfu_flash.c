/**
  ******************************************************************************
  * @file   dfu_flash.c
  * @author Sifli software development team
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2021 - 2021,  Sifli Technology
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

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <rtthread.h>
#include "board.h"
#include "rtconfig.h"
#include "dfu.h"
#include "drv_flash.h"
#include "dfu_internal.h"
#include "os_adaptor.h"

#include "mem_map.h"


#define LOG_TAG "DFUFLASH"
#include "log.h"



#if defined(OTA_55X)

#ifndef OTA_NOR_LCPU_ROM_PATCH_SIZE
    #define OTA_NOR_LCPU_ROM_PATCH_SIZE 0
#endif

uint32_t image_offset = 0;

void set_image_offset(uint32_t offset)
{
    image_offset = offset;
}

static uint32_t dfu_hcpu_compress_size()
{
    uint32_t size;
    size = HCPU_FLASH_CODE_SIZE * 7 / 10;
    if (size % 0x2000 != 0)
    {
        size = (size + 0x2000) / 0x2000 * 0x2000;
    }
    return size;
}

// check download buffer is enough or not
static void dfu_erase_download_buffer_size_check(uint8_t img_id, uint8_t flag, uint32_t size)
{
#ifdef DFU_USING_BACK_UP_FLASH
    switch (img_id)
    {
    case DFU_IMG_ID_HCPU:
        if (flag & DFU_FLAG_COMPRESS)
        {
            size += OTA_NOR_LCPU_ROM_PATCH_SIZE * 2;
            if (size > DFU_RES_FLASH_CODE_SIZE)
            {
                LOG_I("compress len over %d, %d", size, DFU_RES_FLASH_CODE_SIZE);
                OS_ASSERT(0);
            }
        }
        break;
    case DFU_IMG_ID_PATCH:
        if (flag & DFU_FLAG_COMPRESS)
        {
            if (size * 2 > DFU_RES_FLASH_CODE_SIZE)
            {
                LOG_I("compress patch len over %d, %d", size, DFU_RES_FLASH_CODE_SIZE);
                OS_ASSERT(0);
            }
            if (size > OTA_NOR_LCPU_ROM_PATCH_SIZE)
            {
                LOG_I("compress patch offset over %d, %d", size, OTA_NOR_LCPU_ROM_PATCH_SIZE);
                OS_ASSERT(0);
            }
        }
        break;
    case DFU_IMG_ID_LCPU:
    {
        uint32_t hcpu_compress_size = dfu_hcpu_compress_size();
        if (flag & DFU_FLAG_COMPRESS)
        {
            size += OTA_NOR_LCPU_ROM_PATCH_SIZE * 2 + hcpu_compress_size;
            if (size > DFU_RES_FLASH_CODE_SIZE)
            {
                LOG_I("compress lcpu len over %d, %d", size, DFU_RES_FLASH_CODE_SIZE);
                OS_ASSERT(0);
            }
        }
        break;
    }
    }
#endif
}

static uint32_t dfu_backup_addr_res_get()
{
    uint32_t flash_addr = 0xFFFFFFFF;

    flash_addr = DFU_RES_FLASH_CODE_START_ADDR + OTA_NOR_LCPU_ROM_PATCH_SIZE * 2 + HCPU_FLASH_CODE_SIZE * 0.7;
    if (flash_addr % 0x1000 != 0)
    {
        flash_addr = (flash_addr + 0x1000) / 0x1000 * 0x1000;
    }

    return flash_addr;
}

static uint32_t dfu_backup_addr_font_get()
{
    uint32_t flash_addr = 0xFFFFFFFF;

    flash_addr = dfu_backup_addr_res_get() + HCPU_FLASH2_IMG_SIZE;
    if (flash_addr % 0x1000 != 0)
    {
        flash_addr = (flash_addr + 0x1000) / 0x1000 * 0x1000;
    }

    return flash_addr;
}

static uint32_t dfu_backup_addr_ex_get()
{
    uint32_t flash_addr = 0xFFFFFFFF;

    flash_addr = dfu_backup_addr_font_get() + HCPU_FLASH2_FONT_SIZE * 0.7;
    if (flash_addr % 0x1000 != 0)
    {
        flash_addr = (flash_addr + 0x1000) / 0x1000 * 0x1000;
    }

    return flash_addr;
}

uint32_t dfu_get_download_addr_by_imgid(uint8_t img_id, uint8_t flag)
{
    uint32_t flash_addr = 0xFFFFFFFF;

    switch (img_id)
    {
    case DFU_IMG_ID_OTA_MANAGER:
        if (flag & DFU_FLAG_COMPRESS)
            flash_addr = DFU_RES_FLASH_CODE_START_ADDR;
        else
            flash_addr = DFU_FLASH_CODE_START_ADDR;
        break;
    case DFU_IMG_ID_HCPU:
        if (flag & DFU_FLAG_COMPRESS)
        {
            flash_addr = DFU_RES_FLASH_CODE_START_ADDR + OTA_NOR_LCPU_ROM_PATCH_SIZE * 2;
        }
        else
            flash_addr = HCPU_FLASH_CODE_START_ADDR;
        break;
#ifndef SOC_SF32LB52X
    case DFU_IMG_ID_LCPU:
    {
        uint32_t hcpu_compress_size = dfu_hcpu_compress_size();
        if (flag & DFU_FLAG_COMPRESS)
            flash_addr = DFU_RES_FLASH_CODE_START_ADDR + OTA_NOR_LCPU_ROM_PATCH_SIZE * 2 + hcpu_compress_size;
        else
            flash_addr = LCPU_FLASH_CODE_START_ADDR;
        break;
    }
    case DFU_IMG_ID_PATCH:
        if (flag & DFU_FLAG_COMPRESS)
            flash_addr = DFU_RES_FLASH_CODE_START_ADDR;
        else
            flash_addr = LCPU_PATCH_START_ADDR;
        break;
#endif
    case DFU_IMG_ID_FONT:
        if (flag & DFU_FLAG_COMPRESS)
            flash_addr = dfu_backup_addr_font_get();
        else
            flash_addr = HCPU_FLASH2_FONT_START_ADDR;
        break;
    case DFU_IMG_ID_TINY_FONT:
#ifdef HCPU_FLASH2_TINY_FONT_START_ADDR
        flash_addr = HCPU_FLASH2_TINY_FONT_START_ADDR;
#endif
        break;
    case DFU_IMG_ID_RES:
        if (flag & DFU_FLAG_COMPRESS)
            flash_addr = dfu_backup_addr_res_get();
        else
            flash_addr = HCPU_FLASH2_IMG_START_ADDR;
        break;
    case DFU_IMG_ID_EX:
#ifdef HCPU_FS_ROOT_BURN_ADDR
        if (flag & DFU_FLAG_COMPRESS)
            flash_addr = dfu_backup_addr_ex_get();
        else
            flash_addr = HCPU_FS_ROOT_BURN_ADDR;
#endif
        break;
    case DFU_IMG_ID_RES_UPGRADE:
        flash_addr = HCPU_FLASH2_IMG_UPGRADE_START_ADDR;
        break;
#ifdef DFU_LCPU_ROM_PATCH_COMPARE
    case DFU_IMG_ID_PATCH_TEMP:
        if (flag & DFU_FLAG_COMPRESS)
            flash_addr = DFU_RES_FLASH_CODE_START_ADDR;
        else
            flash_addr = DFU_RES_FLASH_CODE_START_ADDR + OTA_NOR_LCPU_ROM_PATCH_SIZE;
        break;
#endif

#ifdef FLASH_BOOT_LOADER_START_ADDR
    case DFU_IMG_ID_BOOTLOADER:
        flash_addr = FLASH_BOOT_LOADER_START_ADDR;
        break;
#endif
    default:
        break;
    }

#ifdef OTA_55X
    dfu_flash_info_t info;
    if (dfu_flash_addr_get(img_id, &info) == DFU_ERR_NO_ERR)
    {
        flash_addr = info.addr;
    }
#endif

    return flash_addr;
}


int8_t dfu_get_flashid_by_imgid(uint8_t img_id)
{
    int8_t flash_id = -1;;
    switch (img_id)
    {
    case DFU_IMG_ID_HCPU:
        flash_id = DFU_FLASH_IMG_HCPU2;
        break;
    case DFU_IMG_ID_LCPU:
    case DFU_IMG_ID_PATCH:
    case DFU_IMG_ID_FONT:
    case DFU_IMG_ID_RES:
    case DFU_IMG_ID_RES_UPGRADE:
    case DFU_IMG_ID_TINY_FONT:
    case DFU_IMG_ID_EX:
    default:
        break;

    }

    return flash_id;
}


int dfu_packet_erase_flash(dfu_image_header_int_t *header, uint32_t offset, uint32_t size)
{
    uint32_t dest = dfu_get_download_addr_by_imgid(header->img_id, header->flag);
    int ret = -1;

    uint32_t align_size = 1;
    int8_t flash_type = dfu_get_flash_type(dest);

    if (flash_type == DFU_FLASH_TYPE_NOR)
    {
#ifdef SOC_SF32LB55X
        // erase should 8k aligned
        align_size = 0x2000;
#else
        // none 55x 4k aligned
        align_size = 0x1000;
#endif
    }
    else if (flash_type == DFU_FLASH_TYPE_NAND)
    {
        align_size = 0x20000;
    }
    else if (flash_type == DFU_FLASH_TYPE_EMMC)
    {
        align_size = 1;
    }

    if (size % align_size != 0)
    {
        size = (size + align_size) / align_size * align_size;
    }

    dfu_erase_download_buffer_size_check(header->img_id, header->flag, size);

    LOG_I("dfu_packet_erase_flash dest 0x%x, size %d", dest, size);
    ret = dfu_flash_erase(dest, size);
    return ret;
}


int dfu_packet_write_flash(dfu_image_header_int_t *header, uint32_t offset, uint8_t *data, uint32_t size)
{
    uint32_t dest = dfu_get_download_addr_by_imgid(header->img_id, header->flag);
    int ret = -1;
    if (dest != 0xFFFFFFFF)
    {
        ret = dfu_flash_write(dest + offset, data, size);
    }
    return ret;
}

int dfu_packet_read_flash(dfu_image_header_int_t *header, uint32_t offset, uint8_t *data, uint32_t size)
{
    uint32_t dest = dfu_get_download_addr_by_imgid(header->img_id, header->flag);
    int ret = -1;
    if (dest != 0xFFFFFFFF)
    {
        ret = dfu_flash_read(dest + offset, data, size);
    }
    return ret;
}

uint8_t is_addr_in_flash(uint32_t addr)
{
    uint8_t is_in_flash = 0;

    if ((addr >= FLASH_BASE_ADDR && addr < HPSYS_RAM0_BASE) ||
#ifndef SOC_SF32LB52X
            (addr >= FLASH2_BASE_ADDR && addr < (FLASH2_BASE_ADDR + FLASH2_SIZE)) ||
            (addr >= FLASH3_BASE_ADDR && addr < (FLASH3_BASE_ADDR + FLASH3_SIZE)))
#else
            (addr >= FLASH2_BASE_ADDR && addr < (FLASH2_BASE_ADDR + FLASH2_SIZE)))
#endif
        is_in_flash = 1;
    return is_in_flash;
}

int8_t dfu_get_flash_type(uint32_t dest)
{
    uint8_t flash_type;
    FLASH_HandleTypeDef *fhandle = rt_nand_get_handle(dest);
    if (fhandle != NULL)
    {
        // nand flash
        flash_type = DFU_FLASH_TYPE_NAND;
    }
    else
    {
        extern FLASH_HandleTypeDef *Addr2Handle(uint32_t addr);
        fhandle = Addr2Handle(dest);
        if (fhandle == NULL)
        {
            if ((dest & DFU_EMMC_ADDR_RANGE_FLAG) == DFU_EMMC_ADDR_RANGE)
            {
                flash_type = DFU_FLASH_TYPE_EMMC;
            }
            else
            {
                LOG_I("invalid addr 0x%x", dest);
                return -3;
            }
        }
        else
        {
            flash_type = DFU_FLASH_TYPE_NOR;
        }
    }

    return flash_type;
}

int dfu_flash_read(uint32_t addr, uint8_t *data, int size)
{
    int8_t flash_type = dfu_get_flash_type(addr);

    int ret;
    uint32_t rd_size;
    if (flash_type == DFU_FLASH_TYPE_NOR)
    {
        rd_size = rt_flash_read(addr, data, size);
    }
    else if (flash_type == DFU_FLASH_TYPE_NAND)
    {
        rd_size = rt_nand_read(addr, data, size);
    }
    else if (flash_type == DFU_FLASH_TYPE_EMMC)
    {
        // TODO: implementation emmc read
    }
    else
    {
        rd_size = 0;
    }

    if (rd_size != size)
        ret = -2;
    else
        ret = 0;
    return ret;
}

int dfu_flash_write(uint32_t addr, uint8_t *data, int size)
{
    int8_t flash_type = dfu_get_flash_type(addr);
    int ret;
    uint32_t wr_size;

    if (flash_type == DFU_FLASH_TYPE_NOR)
    {
        wr_size = rt_flash_write(addr, data, size);
    }
#ifndef SOC_SF32LB55X
    else if (flash_type == DFU_FLASH_TYPE_NAND)
    {
        FLASH_HandleTypeDef *fhandle = rt_nand_get_handle(addr);
        uint32_t page_size = 2048;
        uint32_t wr_size;
        wr_size = rt_nand_write_page(addr, data, size, NULL, 0);
        if (size != page_size)
        {
            size = page_size;
        }
    }
#endif
    else if (flash_type == DFU_FLASH_TYPE_EMMC)
    {
        // TODO: implementation emmc write
    }
    else
    {
        wr_size = 0;
    }

    if (wr_size != size)
        ret = -2;
    else
        ret = 0;
    return ret;
}

int dfu_flash_erase(uint32_t dest, uint32_t size)
{
    int8_t flash_type = dfu_get_flash_type(dest);
    int ret, ret1;
    uint32_t align_size = 1;

    if (flash_type == DFU_FLASH_TYPE_NOR)
    {
#ifdef SOC_SF32LB55X
        // erase should 8k aligned
        align_size = 0x2000;
#else
        // none 55x 4k aligned
        align_size = 0x1000;
#endif
    }
    else if (flash_type == DFU_FLASH_TYPE_NAND)
    {
        align_size = 0x20000;
    }
    else if (flash_type == DFU_FLASH_TYPE_EMMC)
    {
        align_size = 1;
    }

    if (size % align_size != 0)
    {
        size = (size + align_size) / align_size * align_size;
    }

    LOG_I("dfu_flash_erase addr 0x%x, size 0x%x", dest, size);

    if (flash_type == DFU_FLASH_TYPE_NOR)
    {
        ret1 = rt_flash_erase(dest, size);
    }
    else if (flash_type == DFU_FLASH_TYPE_NAND)
    {
        ret1 = rt_nand_erase(dest, size);
    }
    else if (flash_type == DFU_FLASH_TYPE_EMMC)
    {
        ret1 = 0;
    }
    else
    {
        ret = -3;
        return ret;
    }

    if (ret1 != 0)
    {
        ret = -2;
        LOG_E("dfu_packet_erase_flash Fail! %d", ret1);
    }
    else
    {
        ret = 0;
    }

    return ret;
}

#endif /* OTA_55X */



#ifdef OTA_56X_NAND

//#include "flash_map.h"

__WEAK uint32_t dfu_res_addr_get()
{
    uint32_t addr;
#ifdef HCPU_FS_ROOT_BURN_ADDR
    addr = HCPU_FS_ROOT_BURN_ADDR;
#else
    OS_ASSERT(0);
#endif
    return addr;
}

__WEAK uint32_t dfu_dyn_addr_get()
{
    uint32_t addr;
#ifdef HCPU_FS_DYN_BURN_ADDR
    addr = HCPU_FS_DYN_BURN_ADDR;
#else
    OS_ASSERT(0);
#endif
    return addr;
}

__WEAK uint32_t dfu_music_addr_get()
{
    uint32_t addr;
#ifdef HCPU_FS_MUSIC_BURN_ADDR
    addr = HCPU_FS_MUSIC_BURN_ADDR;
#else
    OS_ASSERT(0);
#endif
    return addr;
}

__WEAK uint32_t dfu_pic_addr_get()
{
    uint32_t addr;
#ifdef HCPU_FLASH2_IMG_BURN_ADDR
    addr = HCPU_FLASH2_IMG_BURN_ADDR;
#else
    OS_ASSERT(0);
#endif
    return addr;
}

__WEAK uint32_t dfu_font_addr_get()
{
    uint32_t addr;
#ifdef HCPU_FLASH2_FONT_BURN_ADDR
    addr = HCPU_FLASH2_FONT_BURN_ADDR;
#else
    OS_ASSERT(0);
#endif
    return addr;
}

__WEAK uint32_t dfu_lang_addr_get()
{
    uint32_t addr;
#ifdef HCPU_FLASH2_LANG_START_ADDR
    addr = HCPU_FLASH2_LANG_START_ADDR;
#else
    OS_ASSERT(0);
#endif
    return addr;
}

__WEAK uint32_t dfu_ring_addr_get()
{
    uint32_t addr;
#ifdef HCPU_FLASH2_RING_BURN_ADDR
    addr = HCPU_FLASH2_RING_BURN_ADDR;
#else
    OS_ASSERT(0);
#endif
    return addr;
}

uint32_t dfu_get_download_addr_by_imgid(uint8_t img_id, uint8_t flag)
{
    uint32_t flash_addr = 0xFFFFFFFF;

    switch (img_id)
    {
    case DFU_IMG_ID_NAND_HCPU:
        flash_addr = dfu_get_hcpu_download_addr();
        break;
    case DFU_IMG_ID_LCPU:
        flash_addr = DFU_LCPU_DOWNLOAD_ADDR;
        break;
    case DFU_IMG_ID_NAND_HCPU_PATCH:
        flash_addr = DFU_NAND_PATCH_DOWNLOAD_ADDR;
        break;
    case DFU_IMG_ID_LCPU_PATCH:
        flash_addr = DFU_LCPU_PATCH_DOWNLOAD_ADDR;
        break;
    case DFU_IMG_ID_RES:
        flash_addr = dfu_res_addr_get();
        break;
    case DFU_IMG_ID_DYN:
        flash_addr = dfu_dyn_addr_get();
        break;
    case DFU_IMG_ID_MUSIC:
        flash_addr = dfu_music_addr_get();
        break;
    case DFU_IMG_ID_PIC:
        flash_addr = dfu_pic_addr_get();
        break;
    case DFU_IMG_ID_FONT:
        flash_addr = dfu_font_addr_get();
        break;
    case DFU_IMG_ID_RING:
        flash_addr = dfu_ring_addr_get();
        break;
    case DFU_IMG_ID_LANG:
        flash_addr = dfu_lang_addr_get();
        break;
    default:
        break;
    }

    dfu_flash_info_t info;
    if (dfu_flash_addr_get(img_id, &info) == DFU_ERR_NO_ERR)
    {
        flash_addr = info.addr;
    }

    return flash_addr;
}

int dfu_packet_erase_flash_ext(uint32_t dest, uint32_t offset, uint32_t size, uint8_t type)
{
    int ret = -1;
    // erase should 8k aligned
    if (type == DFU_FLASH_TYPE_NAND)
    {
        if (size % 0x20000 != 0)
        {
            size = (size + 0x20000) / 0x20000 * 0x20000;
        }
    }
    else if (type == DFU_FLASH_TYPE_NOR)
    {
        if (size % 0x2000 != 0)
        {
            size = (size + 0x2000) / 0x2000 * 0x2000;
        }
    }

    LOG_I("dfu_packet_erase_flash dest 0x%x, size %d", dest, size);
    if (dest != 0xFFFFFFFF)
    {
        int ret1 = -1;
        if (type == DFU_FLASH_TYPE_NAND)
        {
            ret1 = rt_nand_erase(dest, size);
        }
        else if (type == DFU_FLASH_TYPE_NOR)
        {
            ret1 = rt_flash_erase(dest, size);
        }

        //int ret1 = 0;
        if (ret1 != 0)
        {
            ret = -2;
            LOG_E("dfu_packet_erase_flash Fail! %d", ret1);
        }
        else
        {
            ret = 0;
        }
    }
    return ret;
}

int dfu_packet_write_flash_ext(uint32_t dest, uint32_t offset, uint8_t *data, uint32_t size, uint8_t type)
{
    int ret = -1;
    if (dest != 0xFFFFFFFF)
    {
        uint32_t wr_size;
        if (type == DFU_FLASH_TYPE_NAND)
        {
            wr_size  = rt_nand_write_page(dest + offset, data, size, NULL, 0);
        }
        else if (type == DFU_FLASH_TYPE_NOR)
        {
            wr_size = rt_flash_write(dest + offset, data, size);
        }
        //uint32_t wr_size = size;
        if (wr_size != size)
            ret = -2;
        else
            ret = 0;
    }
    return ret;
}

int dfu_packet_read_flash_ext(uint32_t dest, uint32_t offset, uint8_t *data, uint32_t size)
{
    int ret = -1;
    if (dest != 0xFFFFFFFF)
    {
        uint32_t rd_size = rt_nand_read(dest + offset, data, size);
        //uint32_t rd_size = size;
        if (rd_size != size)
            ret = -2;
        else
            ret = 0;
    }
    return ret;
}

int dfu_packet_erase_flash(dfu_image_header_int_t *header, uint32_t offset, uint32_t size)
{
    uint32_t dest;
    uint32_t align_size;
    dest = dfu_get_download_addr_by_imgid(header->img_id, header->flag);

    int ret = -1;

    uint8_t flash_type;
    FLASH_HandleTypeDef *fhandle = rt_nand_get_handle(dest);
    if (fhandle != NULL)
    {
        // nand flash
        flash_type = DFU_FLASH_TYPE_NAND;
    }
    else
    {
        extern FLASH_HandleTypeDef *Addr2Handle(uint32_t addr);
        fhandle = Addr2Handle(dest);
        if (fhandle == NULL)
        {
            LOG_I("invalid addr 0x%x", dest);
            return -3;
        }
        else
        {
            flash_type = DFU_FLASH_TYPE_NOR;
        }
    }

    // dfu_erase_download_buffer_size_check(header->img_id, header->flag, size);


    if (flash_type == DFU_FLASH_TYPE_NOR)
    {
        // 55x is 8k, other is 4k
        align_size = rt_flash_get_erase_alignment(dest);
    }
    else
    {
        align_size = HAL_NAND_BLOCK_SIZE(fhandle);
    }

    if (size % align_size != 0)
    {
        size = (size + align_size) / align_size * align_size;
    }

    LOG_I("dfu_packet_erase_flash dest 0x%x, size %d, align 0x%x, %d", dest, size, align_size, flash_type);
    int ret1;


    if (dest != 0xFFFFFFFF)
    {
        if (flash_type == DFU_FLASH_TYPE_NOR)
        {
            ret1 = rt_flash_erase(dest, size);
        }
        else
        {
            ret1 = rt_nand_erase(dest, size);
        }

        if (ret1 != 0)
        {
            ret = -2;
            LOG_E("dfu_packet_erase_flash Fail! %d", ret1);
        }
        else
        {
            ret = 0;
        }
    }
    return ret;
}


int dfu_packet_write_flash(dfu_image_header_int_t *header, uint32_t offset, uint8_t *data, uint32_t size)
{
    uint32_t dest;
    uint32_t align_size;
    dest = dfu_get_download_addr_by_imgid(header->img_id, header->flag);

    uint8_t flash_type;
    FLASH_HandleTypeDef *fhandle = rt_nand_get_handle(dest);
    if (fhandle != NULL)
    {
        // nand flash
        flash_type = DFU_FLASH_TYPE_NAND;
    }
    else
    {
        extern FLASH_HandleTypeDef *Addr2Handle(uint32_t addr);
        fhandle = Addr2Handle(dest);
        if (fhandle == NULL)
        {
            LOG_I("invalid addr 0x%x", dest);
            return -3;
        }
        else
        {
            flash_type = DFU_FLASH_TYPE_NOR;
        }
    }


    int ret = -1;
    uint32_t wr_size;
    if (dest != 0xFFFFFFFF)
    {
        if (flash_type == DFU_FLASH_TYPE_NOR)
        {
            wr_size = rt_flash_write(dest + offset, data, size);
        }
        else
        {
            uint32_t page_size = HAL_NAND_PAGE_SIZE(fhandle);
            uint32_t wr_size;
            wr_size = rt_nand_write_page(dest + offset, data, size, NULL, 0);
            if (size != page_size)
            {
                size = page_size;
            }
        }

        if (wr_size != size)
            ret = -2;
        else
            ret = 0;
    }
    return ret;
}

int dfu_packet_read_flash(dfu_image_header_int_t *header, uint32_t offset, uint8_t *data, uint32_t size)
{
    uint32_t dest;
    dest = dfu_get_download_addr_by_imgid(header->img_id, header->flag);

    uint8_t flash_type;
    FLASH_HandleTypeDef *fhandle = rt_nand_get_handle(dest);
    if (fhandle != NULL)
    {
        // nand flash
        flash_type = DFU_FLASH_TYPE_NAND;
    }
    else
    {
        extern FLASH_HandleTypeDef *Addr2Handle(uint32_t addr);
        fhandle = Addr2Handle(dest);
        if (fhandle == NULL)
        {
            LOG_I("invalid addr 0x%x", dest);
            return -3;
        }
        else
        {
            flash_type = DFU_FLASH_TYPE_NOR;
        }
    }

    int ret = -1;
    uint32_t rd_size;
    if (dest != 0xFFFFFFFF)
    {
        if (flash_type == DFU_FLASH_TYPE_NOR)
        {
            rd_size = rt_flash_read(dest + offset, data, size);
        }
        else
        {
            rd_size = rt_nand_read(dest + offset, data, size);
        }

        if (rd_size != size)
            ret = -2;
        else
            ret = 0;
    }

    return ret;
}
#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
