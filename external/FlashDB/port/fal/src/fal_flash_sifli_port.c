/*
 * Copyright (c) 2020, Armink, <armink.ztl@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <fal.h>
#include "drv_flash.h"

static int init(void)
{
    /* do nothing now */
    return 1;
}

static int ef_err_port_cnt = 0;
static int on_ic_read_cnt  = 0;
static int on_ic_write_cnt = 0;

static int read(long base, long offset, uint8_t *buf, size_t size, int nand_flag)
{

    uint32_t addr = base + offset;

    if (addr % 4 != 0)
        ef_err_port_cnt++;

    //rt_kprintf("addr:%p,%d\n", addr, size);
    if (nand_flag)
    {
        size = rt_nand_read(addr, buf, size);
    }
    else
    {
        /* always call driver to support dual flash */
        size = rt_flash_read(addr, buf, size);
    }
    on_ic_read_cnt++;
    return size;
}


static int write(long base, long offset, const uint8_t *buf, size_t size, int nand_flag)
{
    size_t   i;
    uint32_t addr = base + offset;

    if (addr % 4 != 0)
        ef_err_port_cnt++;

    if (nand_flag)
    {
        size = rt_nand_write(addr, buf, size);
    }
    else 
    {
    rt_flash_write(addr, buf, size);
    }

    on_ic_write_cnt++;
    return size;
}


static int erase(long base, long offset, size_t size, int page_size, int nand_flag)
{
    uint32_t addr = base + offset;

    HAL_StatusTypeDef flash_status;
    size_t erase_pages, i;
    uint32_t PAGEError = 0;

    erase_pages = size / page_size;
    if (size % page_size != 0)
    {
        erase_pages++;
    }
    
    if (nand_flag)
    {
        size = rt_nand_erase(addr, erase_pages * page_size);
    }
    else 
    {
    rt_flash_erase(addr, erase_pages * page_size);
    }

    return size;
}

#if defined(BSP_ENABLE_QSPI1)&&(BSP_QSPI1_MODE<2)
#undef FAL_DEV
#undef FAL_NAME
#undef FAL_SIZE
#undef PAGE_SIZE
#undef SECTOR_SIZE
#undef NAND_FLAG
#undef FAL_BASE

#define FAL_NAME    "flash1"
#define FAL_DEV     nor_flash1
#define FAL_SIZE    (BSP_QSPI1_MEM_SIZE*1024*1024)
// Sector size
#if (BSP_QSPI1_MODE == SPI_MODE_NOR)
#ifdef HDK_U8N5
#define PAGE_SIZE       8192
#define SECTOR_SIZE     8192
#else
#define PAGE_SIZE       4096
#define SECTOR_SIZE     4096
#endif
#define NAND_FLAG       0
#define FAL_BASE        FLASH_BASE_ADDR
#else
#define PAGE_SIZE       128*1024
#define SECTOR_SIZE     2*1024
#define NAND_FLAG       1
#define FAL_BASE        HCPU_MPI_SBUS_ADDR(FLASH_BASE_ADDR)
#endif

static int read1(long offset, uint8_t *buf, size_t size)
{
    return read(FAL_DEV.addr, offset, buf, size, FAL_DEV.nand_flag);
}
static int write1( long offset, const uint8_t *buf, size_t size)
{
    return write(FAL_DEV.addr, offset, buf, size, FAL_DEV.nand_flag);
}

static int erase1( long offset, size_t size)
{
    return erase(FAL_DEV.addr, offset, size, PAGE_SIZE, FAL_DEV.nand_flag);
}

const struct fal_flash_dev FAL_DEV =
{
    .name        = FAL_NAME,
    .addr        = FAL_BASE,
    .len         = FAL_SIZE,
    .blk_size    = PAGE_SIZE,
    .sector_size = SECTOR_SIZE,        
    .nand_flag   = NAND_FLAG,
    .ops         = {init, read1, write1, erase1},
    .write_gran  = 32
};
#endif

#if defined(BSP_ENABLE_QSPI2)&&(BSP_QSPI2_MODE<2)
#undef FAL_DEV
#undef FAL_NAME
#undef FAL_SIZE
#undef PAGE_SIZE
#undef SECTOR_SIZE
#undef NAND_FLAG
#undef FAL_BASE

#define FAL_NAME    "flash2"
#define FAL_DEV     nor_flash2
#define FAL_SIZE    FLASH2_SIZE
// Sector size
#if (BSP_QSPI2_MODE == SPI_MODE_NOR)
#define PAGE_SIZE       8192
#define SECTOR_SIZE     8192
#define NAND_FLAG       0
#define FAL_BASE        FLASH2_BASE_ADDR
#else
#define PAGE_SIZE       128*1024
#define SECTOR_SIZE     2*1024
#define NAND_FLAG       1
#define FAL_BASE        HCPU_MPI_SBUS_ADDR(FLASH2_BASE_ADDR)
#endif

static int read2(long offset, uint8_t *buf, size_t size)
{
    return read(FAL_DEV.addr, offset, buf, size, FAL_DEV.nand_flag);
}
static int write2( long offset, const uint8_t *buf, size_t size)
{
    return write(FAL_DEV.addr, offset, buf, size, FAL_DEV.nand_flag);
}

static int erase2( long offset, size_t size)
{
    return erase(FAL_DEV.addr, offset, size, PAGE_SIZE, FAL_DEV.nand_flag);
}

const struct fal_flash_dev FAL_DEV =
{
    .name        = FAL_NAME,
    .addr        = FAL_BASE,
    .len         = FAL_SIZE,
    .blk_size    = PAGE_SIZE,
    .sector_size = SECTOR_SIZE,        
    .nand_flag   = NAND_FLAG,
    .ops         = {init, read2, write2, erase2},
    .write_gran  = 32
};
#endif


#if defined(BSP_ENABLE_QSPI3)&&(BSP_QSPI3_MODE<2)
#undef FAL_DEV
#undef FAL_NAME
#undef FAL_SIZE
#undef PAGE_SIZE
#undef SECTOR_SIZE
#undef NAND_FLAG
#undef FAL_BASE

#define FAL_NAME    "flash3"
#define FAL_DEV     nor_flash3
#define FAL_SIZE    FLASH3_SIZE
// Sector size
#if (BSP_QSPI3_MODE == SPI_MODE_NOR)
#define PAGE_SIZE       8192
#define SECTOR_SIZE     8192
#define NAND_FLAG       0
#define FAL_BASE        FLASH3_BASE_ADDR
#else
#define PAGE_SIZE       128*1024
#define SECTOR_SIZE     2*1024
#define NAND_FLAG       1
#define FAL_BASE        HCPU_MPI_SBUS_ADDR(FLASH3_BASE_ADDR)
#endif

static int read3(long offset, uint8_t *buf, size_t size)
{
    return read(FAL_DEV.addr, offset, buf, size, FAL_DEV.nand_flag);
}
static int write3( long offset, const uint8_t *buf, size_t size)
{
    return write(FAL_DEV.addr, offset, buf, size, FAL_DEV.nand_flag);
}

static int erase3( long offset, size_t size)
{
    return erase(FAL_DEV.addr, offset, size, PAGE_SIZE, FAL_DEV.nand_flag);
}

const struct fal_flash_dev FAL_DEV =
{
    .name        = FAL_NAME,
    .addr        = FAL_BASE,
    .len         = FAL_SIZE,
    .blk_size    = PAGE_SIZE,
    .sector_size = SECTOR_SIZE,        
    .nand_flag   = NAND_FLAG,
    .ops         = {init, read3, write3, erase3},
    .write_gran  = 32
};
#endif

#if defined(BSP_ENABLE_QSPI4)&&(BSP_QSPI4_MODE<2)

#undef FAL_DEV
#undef FAL_NAME
#undef FAL_SIZE
#undef PAGE_SIZE
#undef SECTOR_SIZE
#undef NAND_FLAG
#undef FAL_BASE

#define FAL_NAME    "flash4"
#define FAL_DEV     nor_flash4
#define FAL_SIZE    FLASH4_SIZE
// Sector size
#if (BSP_QSPI4_MODE == SPI_MODE_NOR)
#define PAGE_SIZE       8192
#define SECTOR_SIZE     8192
#define NAND_FLAG       0
#define FAL_BASE        FLASH4_BASE_ADDR
#else
#define PAGE_SIZE       128*1024
#define SECTOR_SIZE     2*1024
#define NAND_FLAG       1
#define FAL_BASE        HCPU_MPI_SBUS_ADDR(FLASH4_BASE_ADDR)
#endif

static int read4(long offset, uint8_t *buf, size_t size)
{
    return read(FAL_DEV.addr, offset, buf, size, FAL_DEV.nand_flag);
}
static int write4( long offset, const uint8_t *buf, size_t size)
{
    return write(FAL_DEV.addr, offset, buf, size, FAL_DEV.nand_flag);
}

static int erase4( long offset, size_t size)
{
    return erase(FAL_DEV.addr, offset, size, PAGE_SIZE, FAL_DEV.nand_flag);
}

const struct fal_flash_dev FAL_DEV =
{
    .name        = FAL_NAME,
    .addr        = FAL_BASE,
    .len         = FAL_SIZE,
    .blk_size    = PAGE_SIZE,
    .sector_size = SECTOR_SIZE,        
    .nand_flag   = NAND_FLAG,
    .ops         = {init, read4, write4, erase4},
    .write_gran  = 32
};
#endif


#if defined(BSP_USING_SDMMC1) || defined(RT_USING_SPI_MSD)

#undef FAL_DEV
#undef FAL_NAME
#undef FAL_SIZE
#undef PAGE_SIZE
#undef SECTOR_SIZE
#undef NAND_FLAG
#undef FAL_BASE

#define FAL_NAME    "sd0"
#define FAL_DEV     fal_sdmmc1
#define FAL_SIZE    0x100000000

#define PAGE_SIZE       512
#define SECTOR_SIZE     512
#define NAND_FLAG       2
#define FAL_BASE        SDMMC1_MEM_BASE

static int sdmmc1_read(long offset, uint8_t *buf, size_t size)
{
    return 0;
}
static int sdmmc1_write(long offset, const uint8_t *buf, size_t size)
{
    return 0;
}

static int sdmmc1_erase(long offset, size_t size)
{
    return 0;
}

const struct fal_flash_dev FAL_DEV =
{
    .name        = FAL_NAME,
    .addr        = FAL_BASE,
    .len         = (FAL_SIZE >> 9),
    .blk_size    = PAGE_SIZE,
    .sector_size = SECTOR_SIZE,
    .nand_flag   = NAND_FLAG,
    .ops         = {init, sdmmc1_read, sdmmc1_write, sdmmc1_erase},
    .write_gran  = 32
};
#endif /* BSP_USING_SDMMC1 || RT_USING_SPI_MSD */


#if defined(BSP_USING_SDMMC2)

#undef FAL_DEV
#undef FAL_NAME
#undef FAL_SIZE
#undef PAGE_SIZE
#undef SECTOR_SIZE
#undef NAND_FLAG
#undef FAL_BASE

#define FAL_NAME    "sd1"
#define FAL_DEV     fal_sdmmc2
#define FAL_SIZE    0x100000000

#define PAGE_SIZE       512
#define SECTOR_SIZE     512
#define NAND_FLAG       2
#define FAL_BASE        SDMMC2_MEM_BASE

static int sdmmc2_read(long offset, uint8_t *buf, size_t size)
{
    return 0;
}
static int sdmmc2_write(long offset, const uint8_t *buf, size_t size)
{
    return 0;
}

static int sdmmc2_erase(long offset, size_t size)
{
    return 0;
}

const struct fal_flash_dev FAL_DEV =
{
    .name        = FAL_NAME,
    .addr        = FAL_BASE,
    .len         = (FAL_SIZE >> 9),
    .blk_size    = PAGE_SIZE,
    .sector_size = SECTOR_SIZE,
    .nand_flag   = NAND_FLAG,
    .ops         = {init, sdmmc2_read, sdmmc2_write, sdmmc2_erase},
    .write_gran  = 32
};
#endif /* BSP_USING_SDMMC2 */
