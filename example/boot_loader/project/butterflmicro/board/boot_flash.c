/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     zylx         first version
 */

#include <rtconfig.h>
#include <string.h>
#include <register.h>
#include "../dfu/dfu.h"
#include "bf0_hal.h"
#include "board.h"
#include "boot_flash.h"
#include "sifli_bbm.h"

QSPI_FLASH_CTX_T spi_flash_handle[FLASH_MAX_INSTANCE];
DMA_HandleTypeDef spi_flash_dma_handle[FLASH_MAX_INSTANCE];
flash_read_func g_flash_read;
flash_write_func g_flash_write;
flash_erase_func g_flash_erase;

FLASH_HandleTypeDef *boot_handle;
uint32_t g_config_addr;
static uint8_t sd_cache[512];
static uint32_t nand_pagesize = 2048;
static uint32_t nand_blksize = 0x20000;

int port_read_page(int blk, int page, int offset, uint8_t *buff, uint32_t size, uint8_t *spare, uint32_t spare_len)
{
    int res;
    uint32_t addr = (blk * nand_blksize) + (page * nand_pagesize) + offset;

    FLASH_HandleTypeDef *hflash = (FLASH_HandleTypeDef *)&spi_flash_handle[1].handle;
    if ((offset + size) > nand_pagesize)
    {
        HAL_ASSERT(0);
    }
#if (NAND_BUF_CPY_MODE == 1)
    SCB_InvalidateDCache_by_Addr(buff, size);
#endif
    SCB_InvalidateDCache_by_Addr((void *)hflash->base, nand_pagesize + SPI_NAND_MAXOOB_SIZE);
    if (((blk & 1) != 0) && (hflash->wakeup != 0))
        SCB_InvalidateDCache_by_Addr((void *)(hflash->base + (1 << 12)), SPI_NAND_PAGE_SIZE + SPI_NAND_MAXOOB_SIZE);
    res = HAL_NAND_READ_WITHOOB(hflash, addr, buff, size, spare, spare_len);

    return res; //RET_NOERROR;
}

int bbm_get_bb(int blk)
{
    FLASH_HandleTypeDef *hflash = (FLASH_HandleTypeDef *)&spi_flash_handle[1].handle;

    SCB_InvalidateDCache_by_Addr((void *)hflash->base, nand_pagesize + SPI_NAND_MAXOOB_SIZE);
    if (((blk & 1) != 0) && (hflash->wakeup != 0))
        SCB_InvalidateDCache_by_Addr((void *)(hflash->base + (1 << 12)), SPI_NAND_PAGE_SIZE + SPI_NAND_MAXOOB_SIZE);
    int bad = HAL_NAND_GET_BADBLK(hflash, blk);

    return bad;
}

static int read_nand(uint32_t addr, const int8_t *buf, uint32_t size)
{
    int cnt;
    uint32_t fill, offset;

#if (NAND_BUF_CPY_MODE == 1)
    SCB_InvalidateDCache_by_Addr((void *)buf, cnt);
#endif
    cnt = 0;
    offset = addr >= boot_handle->base ? addr - boot_handle->base : addr;
    while (size > 0)
    {
        fill = size > nand_pagesize ? nand_pagesize : size;

#ifdef CFG_BOOTROM  // ROM code will bypass BBM.
        if (port_read_page(offset / nand_blksize, (offset / nand_pagesize) & (nand_blksize / nand_pagesize - 1), offset & (nand_pagesize - 1), (uint8_t *)(buf + cnt), fill, NULL, 0) != fill)
            break;
#else
        if (bbm_read_page(offset / nand_blksize, (offset / nand_pagesize) & (nand_blksize / nand_pagesize - 1), offset & (nand_pagesize - 1), (uint8_t *)(buf + cnt), fill, NULL, 0) != fill)
            break;
#endif
        cnt += fill;
        size -= fill;
        offset += fill;
    }

    return cnt;
}

static int read_nor(uint32_t addr, const int8_t *buf, uint32_t size)
{
    memcpy((void *)buf, (uint8_t *)addr, size);
    return size;
}

static int write_nor(uint32_t addr, const int8_t *buf, uint32_t size)
{
    FLASH_HandleTypeDef *hflash;
    uint32_t taddr, start, remain, fill;
    uint8_t *tbuf;
    int res;

    if (addr >= MPI2_MEM_BASE)
    {
        hflash = &(spi_flash_handle[1].handle);
    }
    else
    {
        hflash = &(spi_flash_handle[0].handle);
    }

    if ((addr < hflash->base) || (addr > (hflash->base + hflash->size)))
        return 0;

    taddr = addr - hflash->base;
    tbuf = (uint8_t *)buf;
    remain = size;

    start = taddr & (QSPI_NOR_PAGE_SIZE - 1);
    if (start > 0)    // start address not page aligned
    {
        fill = QSPI_NOR_PAGE_SIZE - start;   // remained size in one page
        if (fill > size)    // not over one page
        {
            fill = size;
        }
        res = HAL_QSPIEX_WRITE_PAGE(hflash, taddr, tbuf, fill);
        if (res != fill)
            return 0;
        taddr += fill;
        tbuf += fill;
        remain -= fill;
    }
    while (remain > 0)
    {
        fill = remain > QSPI_NOR_PAGE_SIZE ? QSPI_NOR_PAGE_SIZE : remain;
        res = HAL_QSPIEX_WRITE_PAGE(hflash, taddr, tbuf, fill);
        if (res != fill)
            return 0;
        taddr += fill;
        tbuf += fill;
        remain -= fill;
    }

    return size;
}

static int erase_nor(uint32_t addr, uint32_t size)
{
    FLASH_HandleTypeDef *hflash;
    uint32_t taddr, remain;
    int res;

    if (addr >= MPI2_MEM_BASE)
    {
        hflash = &(spi_flash_handle[1].handle);
    }
    else
    {
        hflash = &(spi_flash_handle[0].handle);
    }

    if ((addr < hflash->base) || (addr > (hflash->base + hflash->size)))
        return 1;

    taddr = addr - hflash->base;
    remain = size;

    if ((taddr & (QSPI_NOR_SECT_SIZE - 1)) != 0)
        return -1;
    if ((remain & (QSPI_NOR_SECT_SIZE - 1)) != 0)
        return -2;

    while (remain > 0)
    {
        res = HAL_QSPIEX_SECT_ERASE(hflash, taddr);
        if (res != 0)
            return 1;

        remain -= QSPI_NOR_SECT_SIZE;
        taddr += QSPI_NOR_SECT_SIZE;
    }

    return 0;
}



static uint32_t init_mpi1()
{
    // Initialize MPI1
    qspi_configure_t flash_cfg = FLASH1_CONFIG;
    struct dma_config flash_dma = FLASH1_DMA_CONFIG;

    spi_flash_handle[0].dual_mode = 1;
    flash_cfg.line = HAL_FLASH_NOR_MODE;
    if (board_boot_src == BOOT_FROM_SIP_PUYA)
    {
        board_pinmux_mpi1_puya_base();
    }
    else
    {
        board_pinmux_mpi1_gd();
#ifndef CFG_BOOTROM
        flash_cfg.line = HAL_FLASH_QMODE;
#endif
    }
    HAL_FLASH_Init(&(spi_flash_handle[0]), &flash_cfg, &spi_flash_dma_handle[0],     &flash_dma, BSP_GetFlash1DIV());
#ifndef CFG_BOOTROM
    if (board_boot_src == BOOT_FROM_SIP_PUYA)
    {
        // check spi_flash_handle[0].dev_id == 0x176085 // 0x856017
        board_pinmux_mpi1_puya_ext(spi_flash_handle[0].dev_id == 0x176085);
        HAL_FLASH_SET_QUAL_SPI((FLASH_HandleTypeDef *) & (spi_flash_handle[0].handle), true);
        spi_flash_handle[0].handle.Mode = HAL_FLASH_QMODE;
    }
#endif
    g_flash_read = read_nor;
    g_flash_write = write_nor;
    g_flash_erase = erase_nor;
    return (spi_flash_handle[0].base_addr);
}

static uint32_t nand_cache[(4096 + 128) / 4];

#ifndef  CFG_BOOTROM
    static uint32_t bbm_cache_buf[(4096 + 128) / 4];
#endif

static uint32_t init_mpi2(int nand)
{
    // Initialize MPI2
    qspi_configure_t flash_cfg = FLASH2_CONFIG;
    struct dma_config flash_dma = FLASH2_DMA_CONFIG;

    board_pinmux_mpi2();
    // for bootrom, force to 1 line spi, for bootloader use 4 line
#ifdef  CFG_BOOTROM
    flash_cfg.line = HAL_FLASH_NOR_MODE;
#else
    flash_cfg.line = HAL_FLASH_QMODE ;
#endif
    g_flash_read = nand ? read_nand : read_nor;
    if (nand)
    {
        flash_cfg.base += HPSYS_MPI_MEM_CBUS_2_SBUS_OFFSET;
        flash_cfg.SpiMode = SPI_MODE_NAND;
        spi_flash_handle[1].handle.data_buf = (uint8_t *)nand_cache;
    }
    HAL_Delay_us(0);
    spi_flash_handle[1].dual_mode = 1;
    spi_flash_handle[1].flash_mode = nand;
    HAL_StatusTypeDef res = HAL_FLASH_Init(&(spi_flash_handle[1]), &flash_cfg, &spi_flash_dma_handle[1], &flash_dma, BSP_GetFlash2DIV());
    if ((res == HAL_OK) && (nand != 0))
    {
        nand_pagesize = HAL_NAND_PAGE_SIZE(&spi_flash_handle[1].handle);
        nand_blksize = HAL_NAND_BLOCK_SIZE(&spi_flash_handle[1].handle);

        spi_flash_handle[1].handle.buf_mode = 1;    // default set to buffer mode for nand
        HAL_NAND_CONF_ECC(&spi_flash_handle[1].handle, 1); // default enable ECC if support !
#ifndef  CFG_BOOTROM
        bbm_set_page_size(nand_pagesize);
        bbm_set_blk_size(nand_blksize);
        sif_bbm_init(spi_flash_handle[1].total_size, (uint8_t *)bbm_cache_buf);
#endif
    }
    return (spi_flash_handle[1].base_addr);
}


/*****************************SD functions*************************************/
#include "sd_nand_drv.h"
static int read_sdnand(uint32_t addr, const int8_t *buf, uint32_t size)
{
    // TODO: Add SD read.
    uint32_t offset = addr - SDNAND_MEM_ADDR;
    uint32_t remain = size;
    uint8_t *data = (uint8_t *)buf;

    while (remain >= 512)
    {
        sd_read_data(offset, data, 512);
        remain -= 512;
        offset += 512;
        data += 512;
    }
    if (remain > 0)
    {
        sd_read_data(offset, sd_cache, 512);
        memcpy(data, sd_cache, remain);
    }
    return size;
}

static uint32_t init_sdnand()
{
    // TODO: Add SD initial sequence
    board_pinmux_sd();
    uint8_t res = sdmmc1_sdnand();
    if (res != 1)
    {
        // TODO: UART output error message.
        boot_error(res);
    }
    g_flash_read = read_sdnand;
    return SDNAND_MEM_ADDR + SDNAND_START_OFFSET;
}

static int read_sdemmc(uint32_t addr, const int8_t *buf, uint32_t size)
{
    // TODO: Add SD read.
    uint32_t offset = addr - SDNAND_MEM_ADDR;
    uint32_t remain = size;
    uint8_t *data = (uint8_t *)buf;

    while (remain >= 512)
    {
        emmc_read_data(offset, data, 512);
        remain -= 512;
        offset += 512;
        data += 512;
    }
    if (remain > 0)
    {
        emmc_read_data(offset, sd_cache, 512);
        memcpy(data, sd_cache, remain);
    }
    return size;
}

volatile int emmc_init_res = 0;
static uint32_t init_sdemmc()
{
    // TODO: Add SD initial sequence
    board_pinmux_sd();
    int res = sdio_emmc_init();
    emmc_init_res = res;
    if (res != 0)
    {
        // TODO: UART output error message.
        boot_error(res);
        g_flash_read = NULL;
    }

    g_flash_read = read_sdemmc;
    return SDNAND_MEM_ADDR + SDNAND_START_OFFSET;
}


/******************************************************************************/


void dfu_flash_init()
{
#ifndef  CFG_BOOTROM
    // set clock to high speed for bootloader
    HAL_HPAON_EnableXT48();
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_HXT48);
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_HP_PERI, RCC_CLK_PERI_HXT48);


    HAL_PMU_EnableDLL(1);

    HAL_RCC_HCPU_ConfigHCLK(144);
    //HAL_RCC_HCPU_EnableDLL1(240000000);
    //HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_DLL1);
    // Reset sysclk used by HAL_Delay_us
    HAL_Delay_us(0);
#endif

    switch (board_boot_src)
    {
    case BOOT_FROM_SIP_PUYA:
    case BOOT_FROM_SIP_GD:
#ifdef CFG_BOOTROM
        BSP_SetFlash1DIV(2);    // rom use default rc48 / 2, 48/2 = 24
#else
        //TODO: increase speed for SIP NOR flash
        //HAL_PMU_EnableDLL(1);
        //HAL_RCC_HCPU_EnableDLL1(144000000);
        //HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_DLL1);
        HAL_RCC_HCPU_EnableDLL2(288000000);
        BSP_SetFlash1DIV(6);    // bootloader use DLL2 / 6, 288/6 = 48
        HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_FLASH1, RCC_CLK_FLASH_DLL2);
#endif
        g_config_addr = init_mpi1();
        boot_handle = (FLASH_HandleTypeDef *)&spi_flash_handle[0].handle;
        break;
    case BOOT_FROM_NOR:
    case BOOT_FROM_NAND:
#ifdef CFG_BOOTROM
        BSP_SetFlash2DIV(2);    // rom use default rc48 / 2, 48/2 = 24
#else
        HAL_RCC_HCPU_EnableDLL2(288000000);
        BSP_SetFlash2DIV(6);    // bootloader use DLL2 / 6, 288/6 = 48
        HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_FLASH2, RCC_CLK_FLASH_DLL2);
#endif
        g_config_addr = init_mpi2(board_boot_src == BOOT_FROM_NOR ? 0 : 1);
        boot_handle = (FLASH_HandleTypeDef *)&spi_flash_handle[1].handle;
        break;
    case BOOT_FROM_SD:
        HAL_RCC_HCPU_EnableDLL2(288000000);
        g_config_addr = init_sdnand();  // TODO, update sd clock divider after clock changed
        boot_handle = NULL;
        break;
    case BOOT_FROM_EMMC:
        HAL_RCC_HCPU_EnableDLL2(288000000);
        g_config_addr = init_sdemmc();
        boot_handle = NULL;
        break;
    default:
        boot_error('S');
    }

    if (g_flash_read)
    {
#define MAX_RETRY 5
        int count = MAX_RETRY;

        while (count)
        {
            g_flash_read(g_config_addr, (const int8_t *)&sec_config_cache, sizeof(struct sec_configuration));
            if (sec_config_cache.magic == SEC_CONFIG_MAGIC)
            {
                if (board_boot_src == BOOT_FROM_NAND)
                {
                    struct flash_config *cfg = (struct flash_config *)&sec_config_cache.ftab[DFU_FLASH_CONFIG];
                    if ((cfg->page_size != 0 &&  cfg->page_size != FLASH_UNINIT_32))
                    {
                        nand_pagesize = cfg->page_size;
                        g_flash_read(g_config_addr, (const int8_t *)&sec_config_cache, sizeof(struct sec_configuration));
                    }

                }
                break;
            }
#ifdef TARMAC
            HAL_Delay_us(10);
#else
            HAL_Delay_us(1000000);
#endif
            count--;
        }
        if (count == 0)
            boot_error('C');
    }
}


void sec_flash_func_init()
{
    g_flash_read = NULL;
}


