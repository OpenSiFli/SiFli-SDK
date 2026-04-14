/*
 * SPDX-FileCopyrightText: 2021-2021 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include "fal.h"

#include "mem_map.h"

#define LOG_TAG "DFUFLASH"
#include "log.h"

#if defined(OTA_55X)

#ifndef OTA_NOR_LCPU_ROM_PATCH_SIZE
    #define OTA_NOR_LCPU_ROM_PATCH_SIZE 0
#endif

typedef struct
{
    rt_device_t device;
    uint32_t logic_addr;
    struct rt_mutex sdio_mutex;
} dfu_flash_env_t;

static dfu_flash_env_t g_dfu_flash_env = {0};
static dfu_flash_env_t *dfu_flash_get_env()
{
    return &g_dfu_flash_env;
}

#define cache_max_blks  8
#define cache_blk_size 512
ALIGN(32) static rt_uint8_t  blk_cache_buf[cache_blk_size * cache_max_blks];
ALIGN(32) static rt_uint8_t  sdio_cache_buf[cache_blk_size];

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

static int8_t dfu_get_flash_type_by_part_table(uint32_t addr)
{
    dfu_flash_env_t *env = dfu_flash_get_env();
    const struct fal_flash_dev *fal_dev;
    const struct fal_partition *fal_table;

    int8_t ret = -1;
    uint32_t start_addr;
    uint32_t end_addr;

    size_t fal_table_len;

    fal_table = fal_get_partition_table(&fal_table_len);
    for (int i = 0; i < fal_table_len; i++)
    {
        fal_dev = fal_flash_device_find(fal_table[i].flash_name);

        if (fal_dev != NULL)
        {
            //LOG_I("device flag, %s, %d, addr 0x%x, offset 0x%x, len 0x%x ", fal_table[i].name, fal_dev->nand_flag, fal_dev->addr, fal_table[i].offset, fal_table[i].len);
            start_addr = fal_dev->addr + fal_table[i].offset;
            end_addr = fal_dev->addr + fal_table[i].offset + fal_table[i].len;

            //LOG_I("start_addr 0x%x, end_addr 0x%x, addr 0x%x", start_addr, end_addr, addr);
            if (addr >= start_addr && addr <= end_addr)
            {
                if (fal_dev->nand_flag == 2)
                {
#ifdef RT_USING_DEVICE
                    env->device = rt_device_find(fal_table[i].name);
#endif
                    env->logic_addr = start_addr;
                    ret = DFU_FLASH_TYPE_EMMC;
                }
                else if (fal_dev->nand_flag == 1)
                {
                    ret = DFU_FLASH_TYPE_NAND;
                }
                else if (fal_dev->nand_flag == 0)
                {
                    ret = DFU_FLASH_TYPE_NOR;
                }
                break;
            }
        }
    }

    return ret;
}

int8_t dfu_get_flash_type(uint32_t dest)
{
    uint8_t flash_type;
    FLASH_HandleTypeDef *fhandle = rt_nand_get_handle(dest);
    dfu_flash_env_t *env = dfu_flash_get_env();

    int type_ret = dfu_get_flash_type_by_part_table(dest);
    if (type_ret == -1)
    {
        LOG_W("dfu_get_flash_type: get flash type by fal part table failed 0x%x", dest);
    }
    else
    {
        return type_ret;
    }

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
            // if ((dest & DFU_EMMC_ADDR_RANGE_FLAG) == DFU_EMMC_ADDR_RANGE)
            // {
            //     flash_type = DFU_FLASH_TYPE_EMMC;
            // }
            // else
            // {

            // }

            LOG_I("invalid addr 0x%x", dest);
            return -3;
        }
        else
        {
            flash_type = DFU_FLASH_TYPE_NOR;
        }
    }

    return flash_type;
}


#ifdef RT_USING_DEVICE
static int dfu_sdio_read(uint32_t addr, uint8_t *buf, int size)
{
    dfu_flash_env_t *env = dfu_flash_get_env();

    uint32_t pos = (addr - env->logic_addr) / cache_blk_size;
    uint32_t pos_offset = (addr - env->logic_addr) % cache_blk_size;
    uint32_t req_size;
    int offset,  remain, fill;

    if (!buf || !size)    return 0;

    if (addr < env->logic_addr)
    {
        LOG_W("addr %x < logic_addr %x", addr, env->logic_addr);
        return -1;
    }

    rt_device_t dev = env->device;    // get block device
    if (dev == NULL)
    {
        LOG_E("device is NULL");
        return -1;
    }

    offset = 0;
    fill = 0;
    remain = size;

    if (rt_object_get_type(&env->sdio_mutex.parent.parent) != RT_Object_Class_Mutex)
    {
        rt_mutex_init(&env->sdio_mutex, "sdio_mutex", RT_IPC_FLAG_FIFO);
    }

    rt_mutex_take(&env->sdio_mutex, RT_WAITING_FOREVER);
    rt_err_t ret = rt_device_open(dev, RT_DEVICE_FLAG_RDWR);
    if (RT_EOK != ret)
    {
        LOG_W("device open fail %d", ret);
        return -1;
    }
    if (pos_offset) // not page aligned
    {
        rt_device_read(dev, pos, sdio_cache_buf, 1);
        if (remain >= cache_blk_size - pos_offset)
            fill += (cache_blk_size - pos_offset);
        else
            fill = remain;
        memcpy(buf, sdio_cache_buf + pos_offset, fill);
        remain -= fill;
        offset += fill;
        pos += 1;
    }

    if (remain >= cache_blk_size)
    {
        uint32_t req_blk = remain / cache_blk_size;
        fill = (req_blk << 9);
        rt_device_read(dev, pos, buf + offset, req_blk);
        remain -= fill;
        offset += fill;
        pos += req_blk;
    }

    if (remain > 0)
    {
        rt_device_read(dev, pos, sdio_cache_buf, 1);
        memcpy(buf + offset, sdio_cache_buf, remain);
        offset += remain;
    }
    rt_device_close(dev);
    rt_mutex_release(&env->sdio_mutex);

    return offset;
}

static int dfu_sdio_write(uint32_t addr, uint8_t *buf, int size)
{
    dfu_flash_env_t *env = dfu_flash_get_env();

    uint32_t pos = (addr - env->logic_addr) / cache_blk_size;
    uint32_t pos_offset = (addr - env->logic_addr) % cache_blk_size;
    uint32_t req_size;
    int offset = 0, remain = 0, fill = 0;

    if (!buf || !size)    return 0;

    if (addr < env->logic_addr)
    {
        LOG_I("addr %x < logic_addr %x", addr, env->logic_addr);
        return -1;
    }

    rt_device_t dev = env->device;    // get block device
    if (dev == NULL)
    {
        LOG_E("device is NULL");
        return -1;
    }

    if (rt_object_get_type(&env->sdio_mutex.parent.parent) != RT_Object_Class_Mutex)
    {
        rt_mutex_init(&env->sdio_mutex, "sdio_mutex", RT_IPC_FLAG_FIFO);
    }
    rt_mutex_take(&env->sdio_mutex, RT_WAITING_FOREVER);
    rt_err_t ret = rt_device_open(dev, RT_DEVICE_FLAG_RDWR);
    RT_ASSERT(RT_EOK == ret);
    remain = size;
    if (pos_offset)
    {
        rt_memset(sdio_cache_buf, 0, cache_blk_size);
        rt_device_read(dev, pos, sdio_cache_buf, 1);
        if ((remain + pos_offset) >= cache_blk_size)
            fill = cache_blk_size - pos_offset;
        else
            fill = remain;
        rt_memcpy(sdio_cache_buf + pos_offset, buf, fill);
        rt_device_write(dev, pos, sdio_cache_buf, 1);
        remain -= fill;
        pos += 1;
        buf += fill;
        offset += fill;
    }
    if (remain >= cache_blk_size)
    {
        uint32_t req_blk = remain / cache_blk_size;
        rt_device_write(dev, pos, buf, req_blk);
        pos += req_blk;
        buf += (req_blk << 9);
        remain -= (req_blk << 9);
        offset += (req_blk << 9);
    }
    if (remain > 0)
    {
        rt_memset(sdio_cache_buf, 0, cache_blk_size);
        rt_device_read(dev, pos, sdio_cache_buf, 1);
        rt_memcpy(sdio_cache_buf, buf, remain);
        rt_device_write(dev, pos, sdio_cache_buf, 1);
        offset += remain;
    }
    rt_device_close(dev);
    rt_mutex_release(&env->sdio_mutex);

    return offset;
}
#endif

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
#ifdef RT_USING_DEVICE
        rd_size = dfu_sdio_read(addr, data, size);
#else
        rd_szie = 0;
#endif
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
#ifdef RT_USING_DEVICE
        wr_size = dfu_sdio_write(addr, data, size);
#elif
        wr_size = 0;
#endif
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

