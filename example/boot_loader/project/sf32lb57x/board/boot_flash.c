/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtconfig.h>
#include <string.h>
#include <register.h>
#include "../dfu/dfu.h"
#include "bf0_hal.h"
#include "board.h"
#include "boot_flash.h"
#include "sifli_bbm.h"
#include "secboot.h"
#include "sd_drv.h"

#define SD_CACHE_SIZE   512

QSPI_FLASH_CTX_T spi_flash_handle[FLASH_MAX_INSTANCE];
DMA_HandleTypeDef spi_flash_dma_handle[FLASH_MAX_INSTANCE];
flash_read_func g_flash_read;
flash_write_func g_flash_write;
flash_erase_func g_flash_erase;

FLASH_HandleTypeDef *boot_handle;
uint32_t g_config_addr;
static uint8_t sd_cache[SD_CACHE_SIZE];
static uint32_t nand_pagesize = 2048;
static uint32_t nand_blksize = 0x20000;
#ifdef BSP_NO_BOARD_USED
    static uint16_t mpi1_div = 1;
    static uint16_t mpi2_div = 2;
    static uint16_t mpi3_div = 2;
#endif /* BSP_NO_BOARD_USED */

static uint32_t nand_cache[(4096 + 128) / 4];
#ifndef  CFG_BOOTROM
    static uint32_t bbm_cache_buf[(4096 + 128) / 4];
#endif

#ifdef BSP_NO_BOARD_USED
uint16_t BSP_GetFlash1DIV(void)
{
    return mpi1_div;
}

uint16_t BSP_GetFlash2DIV(void)
{
    return mpi2_div;
}

uint16_t BSP_GetFlash3DIV(void)
{
    return mpi3_div;
}

void BSP_SetFlash1DIV(uint16_t div)
{
    mpi1_div = div;
}

void BSP_SetFlash2DIV(uint16_t div)
{
    mpi2_div = div;
}

void BSP_SetFlash3DIV(uint16_t div)
{
    mpi3_div = div;
}

#endif /* BSP_NO_BOARD_USED */

#ifdef CFG_BOOTROM
/* provide default type as bootrom cannot identify all flash chipid */
int HAL_GET_NAND_FLASH_DEFAUT_IDX(void)
{
    return 0;
}
#endif /* CFG_BOOTROM */

int port_read_page(int blk, int page, int offset, uint8_t *buff, uint32_t size, uint8_t *spare, uint32_t spare_len)
{
    int res;
    uint32_t addr = (blk * nand_blksize) + (page * nand_pagesize) + offset;

    FLASH_HandleTypeDef *hflash = (FLASH_HandleTypeDef *)&spi_flash_handle[2].handle;
    if ((offset + size) > nand_pagesize)
    {
        HAL_ASSERT(0);
    }
//TODO: no need to invalidate as cache is not enabled
#if (NAND_BUF_CPY_MODE == NAND_CACHE_USE_EXTDMA)
    SCB_InvalidateDCache_by_Addr(buff, size);
#endif

    SCB_InvalidateDCache_by_Addr((void *)hflash->base, nand_pagesize + SPI_NAND_MAXOOB_SIZE);
    if (((blk & 1) != 0) && (hflash->wakeup != 0))
    {
        //TODO: need to check how plane is select
        SCB_InvalidateDCache_by_Addr((void *)(hflash->base + (1 << 12)), SPI_NAND_PAGE_SIZE + SPI_NAND_MAXOOB_SIZE);
    }
    res = HAL_NAND_READ_WITHOOB(hflash, addr, buff, size, spare, spare_len);

    return res; //RET_NOERROR;
}

int bbm_get_bb(int blk)
{
    FLASH_HandleTypeDef *hflash = (FLASH_HandleTypeDef *)&spi_flash_handle[1].handle;

    SCB_InvalidateDCache_by_Addr((void *)hflash->base, nand_pagesize + SPI_NAND_MAXOOB_SIZE);
    if (((blk & 1) != 0) && (hflash->wakeup != 0))
    {
        SCB_InvalidateDCache_by_Addr((void *)(hflash->base + (1 << 12)), SPI_NAND_PAGE_SIZE + SPI_NAND_MAXOOB_SIZE);
    }
    int bad = HAL_NAND_GET_BADBLK(hflash, blk);

    return bad;
}

static int read_nand(uint32_t addr, const int8_t *buf, uint32_t size)
{
    int cnt;
    uint32_t fill, offset;

#if (NAND_BUF_CPY_MODE == NAND_CACHE_USE_EXTDMA)
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

#ifndef CFG_BOOTROM
static int write_nor(uint32_t addr, const int8_t *buf, uint32_t size)
{
    FLASH_HandleTypeDef *hflash;
    uint32_t taddr, start, remain, fill;
    uint8_t *tbuf;
    int res;

    hflash = boot_handle;

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

    hflash = boot_handle;

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
#endif /* !CFG_BOOTROM */


static uint32_t init_mpi1()
{
    // Initialize MPI1
    qspi_configure_t flash_cfg = FLASH1_CONFIG;
    struct dma_config flash_dma = FLASH1_DMA_CONFIG;

    spi_flash_handle[0].dual_mode = 1;
    if (BOARD_BOOT_DEVICE_MPI1_TYPE1 == board_boot_device)
    {
        board_pinmux_mpi1_type1();
    }
    else
    {
        board_pinmux_mpi1_type2();
    }

#ifdef CFG_BOOTROM
    flash_cfg.line = HAL_FLASH_NOR_MODE;
#else
    flash_cfg.line = HAL_FLASH_QMODE;
#endif /* CFG_BOOTROM */
    HAL_FLASH_Init(&(spi_flash_handle[0]), &flash_cfg, &spi_flash_dma_handle[0], &flash_dma, BSP_GetFlash1DIV());

    g_flash_read = read_nor;

#ifndef CFG_BOOTROM
    g_flash_write = write_nor;
    g_flash_erase = erase_nor;

#endif /* !CFG_BOOTROM */

    return (spi_flash_handle[0].base_addr);
}

static uint32_t init_mpi2(void)
{
    HAL_StatusTypeDef res;
    // Initialize MPI2
    qspi_configure_t flash_cfg = FLASH2_CONFIG;
    struct dma_config flash_dma = FLASH2_DMA_CONFIG;

    board_pinmux_mpi2();
    // for bootrom, force to 1 line spi, for bootloader use 4 line
#ifdef  CFG_BOOTROM
    flash_cfg.line = HAL_FLASH_NOR_MODE;
#else
    flash_cfg.line = HAL_FLASH_QMODE;
#endif /* CFG_BOOTROM */
    g_flash_read = read_nor;
    HAL_Delay_us(0);
    spi_flash_handle[1].dual_mode = 1;
    res = HAL_FLASH_Init(&(spi_flash_handle[1]), &flash_cfg, &spi_flash_dma_handle[1], &flash_dma, BSP_GetFlash2DIV());

#ifndef CFG_BOOTROM
    g_flash_write = write_nor;
    g_flash_erase = erase_nor;
#endif /* !CFG_BOOTROM */

    return (spi_flash_handle[1].base_addr);
}
static uint32_t init_mpi3(int nand)
{
    // Initialize MPI3
    qspi_configure_t flash_cfg = FLASH3_CONFIG;
    struct dma_config flash_dma = FLASH3_DMA_CONFIG;

    board_pinmux_mpi3();
    // for bootrom, force to 1 line spi, for bootloader use 4 line
#ifdef CFG_BOOTROM
    flash_cfg.line = HAL_FLASH_NOR_MODE;
#else
    flash_cfg.line = HAL_FLASH_QMODE;
#endif
    g_flash_read = nand ? read_nand : read_nor;
    if (nand)
    {
        flash_cfg.base += HPSYS_MPI_MEM_CBUS_2_SBUS_OFFSET;
        flash_cfg.SpiMode = SPI_MODE_NAND;
        spi_flash_handle[2].handle.data_buf = (uint8_t *)nand_cache;
    }
    HAL_Delay_us(0);
    spi_flash_handle[2].dual_mode = 1;
    spi_flash_handle[2].flash_mode = nand;
    HAL_StatusTypeDef res = HAL_FLASH_Init(&(spi_flash_handle[2]), &flash_cfg, &spi_flash_dma_handle[2], &flash_dma, BSP_GetFlash3DIV());
    if ((res == HAL_OK) && (nand != 0))
    {
        nand_pagesize = HAL_NAND_PAGE_SIZE(&spi_flash_handle[2].handle);
        nand_blksize = HAL_NAND_BLOCK_SIZE(&spi_flash_handle[2].handle);
        spi_flash_handle[2].handle.buf_mode = 1;    // default set to buffer mode for nand
#ifndef CFG_BOOTROM
        HAL_NAND_CONF_ECC(&spi_flash_handle[2].handle, 1); // default enable ECC if support !
        bbm_set_page_size(nand_pagesize);
        bbm_set_blk_size(nand_blksize);
        sif_bbm_init(spi_flash_handle[2].total_size, (uint8_t *)bbm_cache_buf);
#endif /* CFG_BOOTROM */
    }
#ifndef CFG_BOOTROM
    if (!nand)
    {
        g_flash_write = write_nor;
        g_flash_erase = erase_nor;
    }
#endif /* CFG_BOOTROM */

    return (spi_flash_handle[2].base_addr);
}

/*****************************SD functions*************************************/
static int read_sdnand(uint32_t addr, const int8_t *buf, uint32_t size)
{
    uint32_t offset = addr - SDNAND_MEM_ADDR;
    uint32_t remain = size;
    uint8_t *data = (uint8_t *)buf;
    uint32_t fill;
    uint32_t block_offset;

    /* handle unaligned start */
    block_offset = offset & (SD_CACHE_SIZE - 1);
    if (block_offset > 0)
    {
        sd_read_data(offset - block_offset, sd_cache, SD_CACHE_SIZE);
        fill = SD_CACHE_SIZE - block_offset;
        if (fill > remain)
        {
            fill = remain;
        }
        memcpy(data, sd_cache + block_offset, fill);
        remain -= fill;
        offset += fill;
        data += fill;
    }

    while (remain >= SD_CACHE_SIZE)
    {
        sd_read_data(offset, data, SD_CACHE_SIZE);
        remain -= SD_CACHE_SIZE;
        offset += SD_CACHE_SIZE;
        data += SD_CACHE_SIZE;
    }
    if (remain > 0)
    {
        sd_read_data(offset, sd_cache, SD_CACHE_SIZE);
        memcpy(data, sd_cache, remain);
    }
    return size;
}

static uint32_t init_sdnand()
{
    board_pinmux_sd();
    uint8_t res = sdio_sd_init();
    if (res != 1)
    {
        return 0;
    }
    g_flash_read = read_sdnand;
    return SDNAND_MEM_ADDR + SDNAND_START_OFFSET;
}

static int read_sdemmc(uint32_t addr, const int8_t *buf, uint32_t size)
{
    uint32_t offset = addr - SDNAND_MEM_ADDR;
    uint32_t remain = size;
    uint8_t *data = (uint8_t *)buf;
    uint32_t fill;
    uint32_t block_offset;

    /* handle unaligned start */
    block_offset = offset & (SD_CACHE_SIZE - 1);
    if (block_offset > 0)
    {
        emmc_read_data(offset - block_offset, sd_cache, SD_CACHE_SIZE);
        fill = SD_CACHE_SIZE - block_offset;
        if (fill > remain)
        {
            fill = remain;
        }
        memcpy(data, sd_cache + block_offset, fill);
        remain -= fill;
        offset += fill;
        data += fill;
    }

    while (remain >= SD_CACHE_SIZE)
    {
        emmc_read_data(offset, data, SD_CACHE_SIZE);
        remain -= SD_CACHE_SIZE;
        offset += SD_CACHE_SIZE;
        data += SD_CACHE_SIZE;
    }
    if (remain > 0)
    {
        emmc_read_data(offset, sd_cache, SD_CACHE_SIZE);
        memcpy(data, sd_cache, remain);
    }
    return size;
}

volatile int emmc_init_res = 0;
static uint32_t init_sdemmc()
{
    board_pinmux_sd();
    int res = sdio_emmc_init();
    emmc_init_res = res;
    if (res != 0)
    {
        return 0;
    }

    g_flash_read = read_sdemmc;
    return SDNAND_MEM_ADDR + SDNAND_START_OFFSET;
}

bool boot_device_init(void)
{
    bool succ = false;
    uint8_t hash[DFU_FTAB_CMAC_HASH_SIZE];

#if !defined(CFG_BOOTROM) && !defined(FPGA)
    // set clock to high speed for bootloader
    HAL_HPAON_EnableXT48();
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_HXT48);
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_HP_PERI, RCC_CLK_PERI_HXT48);

    HAL_PMU_EnableDLL(1);
    HAL_RCC_HCPU_ConfigHCLK(144);
    HAL_RCC_HCPU_EnableDLL2(288000000);
#endif

    switch (board_boot_device)
    {
    case BOARD_BOOT_DEVICE_MPI2:
    {
#ifdef CFG_BOOTROM
        BSP_SetFlash2DIV(2);    // rom use default rc48 / 2, 48/2 = 24
#else
        BSP_SetFlash2DIV(6);
        HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_FLASH2, RCC_CLK_FLASH_DLL2);
#endif /* CFG_BOOTROM */
        g_config_addr = init_mpi2();
        boot_handle = (FLASH_HandleTypeDef *)&spi_flash_handle[1].handle;
        break;
    }
    case BOARD_BOOT_DEVICE_MPI1_TYPE1:
    case BOARD_BOOT_DEVICE_MPI1_TYPE2:
    {
#ifdef CFG_BOOTROM
        BSP_SetFlash1DIV(2);    // rom use default rc48 / 2, 48/2 = 24
#else
        BSP_SetFlash1DIV(6);
        HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_FLASH1, RCC_CLK_FLASH_DLL2);
#endif /* CFG_BOOTROM */
        g_config_addr = init_mpi1();
        boot_handle = (FLASH_HandleTypeDef *)&spi_flash_handle[0].handle;
        break;
    }
    case BOARD_BOOT_DEVICE_MPI3_NOR:
    case BOARD_BOOT_DEVICE_MPI3_NAND:
    {
#ifdef CFG_BOOTROM
        BSP_SetFlash3DIV(2);    // rom use default rc48 / 2, 48/2 = 24
#else
        BSP_SetFlash3DIV(6);
        HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_FLASH3, RCC_CLK_FLASH_DLL2);
#endif /* CFG_BOOTROM */
        g_config_addr = init_mpi3(board_boot_device == BOARD_BOOT_DEVICE_MPI3_NOR ? 0 : 1);
        boot_handle = (FLASH_HandleTypeDef *)&spi_flash_handle[2].handle;
        break;
    }
    case BOARD_BOOT_DEVICE_SD:
    {
        g_config_addr = init_sdnand();
        boot_handle = NULL;
        break;
    }
    case BOARD_BOOT_DEVICE_EMMC:
    {
        g_config_addr = init_sdemmc();
        boot_handle = NULL;
        break;
    }
    default:
        printf("invalid boot device");
    }

    if (g_flash_read && g_config_addr)
    {
        g_flash_read(g_config_addr, (const int8_t *)&sec_config_cache, sizeof(sec_config_cache));
        if (sec_config_cache.magic == SEC_CONFIG_MAGIC)
        {
            if (BOARD_BOOT_DEVICE_MPI3_NAND == board_boot_device)
            {
                struct flash_config *cfg = (struct flash_config *)&sec_config_cache.ftab[DFU_FLASH_CONFIG];
                if ((cfg->page_size != 0 &&  cfg->page_size != FLASH_UNINIT_32))
                {
                    nand_pagesize = cfg->page_size;
                    boot_handle->dualFlash |= NAND_FLAG_PAGE_DOUBLE;
                    g_flash_read(g_config_addr, (const int8_t *)&sec_config_cache, sizeof(sec_config_cache));
                }
            }
#if defined(CFG_BOOTROM)
            if (sboot_ctx.sec_en)
            {
                g_flash_read(g_config_addr + sizeof(sec_config_cache), hash, sizeof(hash));
                if (sifli_verify_img_cmac_hash(NULL, (uint8_t *)&sec_config_cache, sizeof(sec_config_cache),
                                               hash))
                {
                    succ = true;
                }
                else
                {
                    printf("ftab signature verification fails\n");
                }
            }

            else
#endif /* CFG_BOOTROM */
            {
                succ = true;
            }
        }
        else
        {
#if defined(CFG_BOOTROM)
            printf("ROM:invalid ftab magic from device:%d\n", board_boot_device);
#else
            printf("BL:invalid ftab magic from device:%d\n", board_boot_device);
#endif /* CFG_BOOTROM */
        }
    }
    else
    {
#if defined(CFG_BOOTROM)
        printf("ROM:boot device init fail from :%d\n", board_boot_device);
#else
        printf("BL:boot device init fail from:%d\n", board_boot_device);
#endif /* CFG_BOOTROM */
    }

    return succ;
}


void sec_flash_func_init()
{
    g_flash_read = NULL;
}


