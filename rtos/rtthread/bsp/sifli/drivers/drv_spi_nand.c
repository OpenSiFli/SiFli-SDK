/**
  ******************************************************************************
  * @file   drv_spi_nand.c
  * @author Sifli software development team
  * @brief Nor Flash Controller BSP driver
  This driver is validated by using MSH command 'date'.
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

/** @addtogroup bsp_driver Driver IO
  * @{
  */

/** @defgroup drv_nand FLASH
  * @brief Nand Flash Controller BSP driver
  This driver is validated by using MSH command 'nand'.
  * @{
  */


#if defined (BSP_USING_SPI_NAND) || defined(_SIFLI_DOXYGEN_)
#include "drv_config.h"
#include "drv_flash.h"
#include "string.h"
#include "drv_io.h"
#include "flash_table.h"

#ifdef RT_USING_ULOG
    //#define DRV_DEBUG
    #define LOG_TAG                "drv.spi_nand"
    #include <drv_log.h>
#else
    #define LOG_D(...)
    //#define LOG_I(...)
    #define LOG_I(fmt, ...)         rt_kprintf(fmt, ##__VA_ARGS__)
    #define LOG_E(fmt, ...)         rt_kprintf(fmt, ##__VA_ARGS__)
    //#define LOG_RAW(...)
    #define LOG_RAW(fmt,...)         rt_kprintf(fmt, ##__VA_ARGS__)
#endif

#include <stdlib.h>

#define BSP_NAND_CHECK_BEFORE_WRITE

#ifndef BSP_DISABLE_BBM
    #define BSP_USING_BBM
    #include "sifli_bbm.h"
#endif


// global value
static int nand_index = -1;   // only ONE nand support in system.
static QSPI_FLASH_CTX_T spi_nand_handle;
static DMA_HandleTypeDef spi_nand_dma_handle;
static struct rt_semaphore         nand_lock;
static int nand_sys_init = 0;
static uint32_t nand_pagesize = 2048;
static uint32_t nand_blksize = 0x20000;


#if ((NAND_BUF_CPY_MODE == 1) && defined(BSP_USING_EXT_DMA))
extern rt_sem_t ExtDma_sema;
static void local_edma_lock()
{
    if (ExtDma_sema == NULL) // ext-dma driver not initialized
        return;

    rt_err_t err;
    err = rt_sem_take(ExtDma_sema, rt_tick_from_millisecond(1000));
    RT_ASSERT(RT_EOK == err);

}
static void local_edma_unlock()
{
    if (ExtDma_sema == NULL) // ext-dma driver not initialized
        return;

    rt_err_t err;
    err = rt_sem_release(ExtDma_sema);
    RT_ASSERT(RT_EOK == err);
}

#else
static void local_edma_lock()
{
    return;

}
static void local_edma_unlock()
{
    return;
}

#endif

__HAL_ROM_USED void rt_nand_lock()
{
    if (nand_sys_init == 0)
        return;
    rt_err_t r = rt_sem_take(&nand_lock, RT_WAITING_FOREVER);
    RT_ASSERT(RT_EOK == r);
}


__HAL_ROM_USED void rt_nand_unlock()
{
    rt_err_t r;
    if (nand_sys_init == 0)
        return;

    r = rt_sem_release(&nand_lock);
    RT_ASSERT(RT_EOK == r);
}

void *rt_nand_get_handle(uint32_t addr)
{
    if (addr < spi_nand_handle.base_addr || addr > (spi_nand_handle.base_addr + spi_nand_handle.total_size))
        return NULL;

    return (void *)(&spi_nand_handle.handle);
}

int rt_nand_read(uint32_t addr, uint8_t *buf, int size)
{
    int offset, cnt, remain, fill, res;
    uint32_t taddr = addr;
    uint8_t *tbuf = buf;
    if (nand_index < 0)
        return 0;

    FLASH_HandleTypeDef *hflash = &(spi_nand_handle.handle);
    if ((addr < hflash->base) || (addr + size > (hflash->base + hflash->size)))
        return 0;

    offset = 0;
    fill = 0;
    cnt = 0;
    remain = size;
    if (taddr & (nand_pagesize - 1)) // not page aligned
    {
        uint8_t *cach_buf = hflash->data_buf;
        offset = taddr & (nand_pagesize - 1); //
        fill = nand_pagesize - offset;
        fill = fill < remain ? fill : remain;
        taddr &= ~(nand_pagesize - 1);
        res = rt_nand_read_page(taddr, cach_buf, nand_pagesize, NULL, 0);
        if (res != nand_pagesize)
            return 0;
        memcpy(tbuf, cach_buf + offset, fill);
        remain -= fill;
        taddr += nand_pagesize;
        tbuf += fill;
        cnt += fill;
    }
    while (remain >= nand_pagesize)
    {
        res = rt_nand_read_page(taddr, tbuf, nand_pagesize, NULL, 0);
        if (res != nand_pagesize)
            return 0;
        remain -= nand_pagesize;
        taddr += nand_pagesize;
        tbuf += nand_pagesize;
        cnt += nand_pagesize;
    }
    if (remain > 0)
    {
        res = rt_nand_read_page(taddr, tbuf, remain, NULL, 0);
        if (res != remain)
            return 0;
        cnt += remain;
    }
    return cnt;
}

int rt_nand_write(uint32_t addr, const uint8_t *buf, int size)
{
    int offset, cnt, remain;
    uint32_t taddr = addr;
    uint8_t *tbuf = (uint8_t *)buf;
    if (nand_index < 0)
        return 0;

    FLASH_HandleTypeDef *hflash = &(spi_nand_handle.handle);
    if ((addr < hflash->base) || (addr + size > (hflash->base + hflash->size)))
        return 0;

    offset = 0;
    cnt = 0;
    remain = size;
    if (taddr & (nand_pagesize - 1)) // not page aligned
    {
        RT_ASSERT(0); // NOT SUUPPORT unaligned write
        offset = nand_pagesize - (taddr & (nand_pagesize - 1));
        cnt = rt_nand_write_page(taddr, tbuf, offset, NULL, 0);
        remain -= offset;
        taddr += offset;
        tbuf += offset;
    }
    while (remain >= nand_pagesize)
    {
        cnt += rt_nand_write_page(taddr, tbuf, nand_pagesize, NULL, 0);
        remain -= nand_pagesize;
        taddr += nand_pagesize;
        tbuf += nand_pagesize;
    }
    if (remain > 0)
    {
        RT_ASSERT(0); // NOT SUUPPORT unaligned write
        cnt += rt_nand_write_page(taddr, tbuf, nand_pagesize, NULL, 0);
    }
    return cnt;

}

int rt_nand_erase(uint32_t addr, int size)
{
    uint32_t remain = size;
    uint32_t taddr = addr;
    int res = 0;
    if ((addr & (nand_blksize - 1)) || (size & (nand_blksize - 1)))
    {
        RT_ASSERT(0);
    }
    if (nand_index < 0)
        return -1;

    FLASH_HandleTypeDef *hflash = &(spi_nand_handle.handle);
    if ((addr < hflash->base) || (addr + size > (hflash->base + hflash->size)))
        return -1;

    while (remain > 0)
    {
        res = rt_nand_erase_block(taddr);
        taddr += nand_blksize;
        remain -= nand_blksize;
        if (res != 0)
            return res;
    }

    return res;
}

int rt_nand_read_page(uint32_t addr, uint8_t *data, int size, uint8_t *spare, int spare_len)
{
    int res;
    if (nand_index < 0)
        return 0;

    FLASH_HandleTypeDef *hflash = &(spi_nand_handle.handle);

    if (hflash == NULL || size > nand_pagesize || hflash->buf_mode == 0)  // only support read 1 page
    {
        RT_ASSERT(0);
        return 0;
    }
    if (addr >= hflash->base)
        addr -= hflash->base;
#ifdef BSP_USING_BBM
    int blk, page, offset;

    blk = addr / nand_blksize;
    page = (addr / nand_pagesize) & (nand_blksize / nand_pagesize - 1);
    offset = addr & (nand_pagesize - 1);

    if (blk >= 8192) // max support 8Gb ? for address error issue
        RT_ASSERT(0);

    rt_nand_lock();
    res = bbm_read_page(blk, page, offset, data, size, spare, spare_len);
    rt_nand_unlock();
#else

    //rt_thread_delay(1);
    rt_nand_lock();
    SCB_InvalidateDCache_by_Addr((void *)hflash->base, nand_pagesize + SPI_NAND_MAXOOB_SIZE);
    if (((addr & nand_blksize) != 0) && (hflash->wakeup != 0))
        SCB_InvalidateDCache_by_Addr((void *)(hflash->base + (1 << 12)), nand_pagesize + SPI_NAND_MAXOOB_SIZE);// TODO: only 2048 page size has plane select issue
#if (NAND_BUF_CPY_MODE == 1)
    SCB_InvalidateDCache_by_Addr(data, size);
#endif
    local_edma_lock();
    res = HAL_NAND_READ_WITHOOB(hflash, addr, data, size, spare, spare_len);
    local_edma_unlock();

    rt_nand_unlock();
    if (res <= 0)
        LOG_E("NAND Read2 error code %d\n", hflash->ErrorCode);
#endif

#if 0   // debug use, for some case controller sames not work
    int i;
    uint8_t *tbuf = data;
    for (i = 0; i < 128; i++)
    {
        if (*tbuf != 0xcc)
            break;
        tbuf++;
    }
    if (i == 128)   // read all cc ? status error?
        RT_ASSERT(0);
#endif
    return res;
}

int rt_nand_write_page(uint32_t addr, const uint8_t *buf, int size, const uint8_t *spare, int spare_len)
{
    int res = 0;
    if (nand_index < 0)
        return 0;

    FLASH_HandleTypeDef *hflash = &(spi_nand_handle.handle);
    if (addr >= hflash->base)
        addr -= hflash->base;
    // write must be page aligned
    if ((addr & (nand_pagesize - 1)) != 0 || size > nand_pagesize)
    {
        LOG_E("Write nand %x must be page aligned\n", addr);
        return 0;
    }
    if ((size == nand_pagesize) && (spare_len == 0)) // full user page write, check if al FF
    {
        int i;
        if (((uint32_t)buf & 3) == 0)
        {
            uint32_t *chk = (uint32_t *)buf;
            for (i = 0; i < nand_pagesize / 4; i++)
            {
                if (chk[i] != 0xffffffff)
                    break;
            }
            if (i == nand_pagesize / 4) // full page data is 0xff, do not need write to flash
            {
                LOG_D("Full page 0xff, do not need write to flash at pos 0x%x\n", addr);
                return size;
            }
        }
        else
        {
            uint8_t *chk8 = (uint8_t *)buf;
            for (i = 0; i < nand_pagesize; i++)
            {
                if (chk8[i] != 0xff)
                    break;
            }
            if (i == nand_pagesize) // full page data is 0xff, do not need write to flash
            {
                LOG_D("Full page 0xff, do not need write to flash at pos 0x%x\n", addr);
                return size;
            }
        }
    }
#ifdef BSP_NAND_CHECK_BEFORE_WRITE
    SCB_InvalidateDCache_by_Addr((void *)hflash->base, nand_pagesize + SPI_NAND_MAXOOB_SIZE);
    SCB_InvalidateDCache_by_Addr((void *)hflash->data_buf, nand_pagesize + SPI_NAND_MAXOOB_SIZE);

    rt_nand_lock();
#ifdef BSP_USING_BBM
    int blk2, page2;
    blk2 = addr / nand_blksize;
    page2 = (addr / nand_pagesize) & (nand_blksize / nand_pagesize - 1);

    res = bbm_read_page(blk2, page2, 0, hflash->data_buf, nand_pagesize, NULL, 0);
#else
    res = HAL_NAND_READ_WITHOOB(hflash, addr, hflash->data_buf, nand_pagesize, NULL, 0);
#endif  //BSP_USING_BBM

    if (res != nand_pagesize)
    {
        rt_nand_unlock();
        LOG_E("Read fail when check before write, err 0x%x with addr 0x%x\n", hflash->ErrorCode, addr);
        RT_ASSERT(0);
    }
    else
    {
        int i = 0;
        uint32_t *chk = (uint32_t *)hflash->data_buf;
        for (i = 0; i < nand_pagesize / 4; i++)
        {
            if (chk[i] != 0xffffffff)
            {
                rt_nand_unlock();
                LOG_E("NAND not clean before write, 0x%08x is not FF at POS %d with addr 0x%x\n", chk[i], i, addr);
                RT_ASSERT(0);
            }
        }
    }
    rt_nand_unlock();
#endif  //BSP_NAND_CHECK_BEFORE_WRITE
#ifdef BSP_USING_BBM
    int blk, page;

    blk = addr / nand_blksize;
    page = (addr / nand_pagesize) & (nand_blksize / nand_pagesize - 1);
    rt_nand_lock();
    res = bbm_write_page(blk, page, (uint8_t *)buf, (uint8_t *)spare, spare_len);
    rt_nand_unlock();
#else
    if (hflash == NULL || size > nand_pagesize)  // only support read 1 page
        return 0;

    rt_nand_lock();
    res = HAL_NAND_WRITE_WITHOOB(hflash, addr, buf, size, spare, spare_len);
    rt_nand_unlock();
    if (res <= 0)
        LOG_E("NAND Write2 fail, error code 0x%x\n", hflash->ErrorCode);
#endif
    return res;
}

int rt_nand_erase_block(uint32_t addr)
{
    int res;
    if (nand_index < 0)
    {
        LOG_E("NAND erase id failed: 0x%x\n", addr);
        return -1;
    }

    FLASH_HandleTypeDef *hflash = &(spi_nand_handle.handle);
    if (hflash == NULL)
        return -1;

    if (addr >= hflash->base)
        addr -= hflash->base;
#ifdef BSP_USING_BBM
    int blk;

    blk = addr / nand_blksize;
    rt_nand_lock();
    res = bbm_erase_block(blk);
    rt_nand_unlock();
#else
    if (!IS_ALIGNED(nand_blksize, addr))  // address should be block aligned
        return -2;

#ifndef BSP_FORCE_ERASE_NAND
    SCB_InvalidateDCache_by_Addr((void *)hflash->base, nand_pagesize + SPI_NAND_MAXOOB_SIZE);
    if (((addr & nand_blksize) != 0) && (hflash->wakeup != 0))
        SCB_InvalidateDCache_by_Addr((void *)(hflash->base + (1 << 12)), nand_pagesize + SPI_NAND_MAXOOB_SIZE); // TODO: only 2048 page size has plane select issue

    int bad = HAL_NAND_GET_BADBLK(hflash, addr / nand_blksize);
    if (bad)   // block is bad, do not erase, or bad mark will be cover
    {
        LOG_E("Blk with address 0x%x is bad, do not erase2\n", addr);
        return -3;
    }
#endif

    rt_nand_lock();
    res = HAL_NAND_ERASE_BLK(hflash, addr);
    rt_nand_unlock();

    if (res != 0)
        LOG_E("Erase error code 0x%x at pos 0x%x\n", hflash->ErrorCode, addr);
#endif
    return res;
}

int rt_nand_conti_read(uint32_t addr, uint8_t en)
{
    if (nand_index < 0)
        return RT_ERROR;

    FLASH_HandleTypeDef *hflash = &(spi_nand_handle.handle);

    return HAL_NAND_SET_CONTINUE(hflash, addr, en, 0);
}

int rt_nand_ahb_conti(uint32_t addr, uint8_t en)
{
    if (nand_index < 0)
        return RT_ERROR;

    FLASH_HandleTypeDef *hflash = &(spi_nand_handle.handle);

    return HAL_NAND_AHB_CONTINUE(hflash, addr, en);
}

int rt_nand_set_otp(uint32_t addr, uint8_t en)
{
    if (nand_index < 0)
        return RT_ERROR;

    FLASH_HandleTypeDef *hflash = &(spi_nand_handle.handle);

    return HAL_NAND_SWITCH_OTP(hflash, en);
}

int rt_nand_register_cache(uint32_t addr, uint8_t *buf)
{
    if (nand_index < 0)
        return RT_ERROR;

    //if(buf == NULL)
    //    return RT_ERROR;
#ifdef RT_USING_MTD_NAND
    if (spi_nand_handle.mtd != NULL)
    {
        struct rt_mtd_nand_device *mtd = (struct rt_mtd_nand_device *)spi_nand_handle.mtd;
        mtd->cache_buf = (void *)buf;
        mtd->blk_cnum = 0xffffffff;
        return 0;
    }
#endif

    return RT_ERROR;
}


int rt_nand_get_total_size(uint32_t addr)
{
    if (nand_index < 0)
        return 0;
    if (addr < spi_nand_handle.base_addr || addr > (spi_nand_handle.base_addr + spi_nand_handle.total_size))
        return 0;
#ifdef BSP_USING_BBM
    return spi_nand_handle.total_size - bbm_get_total();
#endif
    return spi_nand_handle.total_size;
}

int rt_nand_read_id(uint32_t addr)
{
    if (nand_index < 0)
        return 0;
    if (addr < spi_nand_handle.base_addr || addr > (spi_nand_handle.base_addr + spi_nand_handle.total_size))
        return 0;

    return spi_nand_handle.dev_id;
}

#ifdef CFG_BOOTLOADER
    static uint32_t gnand_cache_buf[(4096 + 128) / 4];
    static uint32_t gbbm_cache_buf[(4096 + 128) / 4];
#else
    static uint8_t *gbbm_cache_buf = NULL;
    static uint8_t *gnand_cache_buf = NULL;
#endif  //CFG_BOOTLOADER
extern uint32_t flash_get_freq(int clk_module, uint16_t clk_div, uint8_t hcpu);
int rt_nand_init()
{
    qspi_configure_t flash_cfg; // = FLASH3_CONFIG;
    struct dma_config flash_dma;
    uint16_t div;
    HAL_StatusTypeDef res = HAL_ERROR;
    int clk_mode;
    memset(&spi_nand_handle, 0, sizeof(QSPI_FLASH_CTX_T));
    nand_index = -1;
    nand_sys_init = 0;

#if defined(BSP_ENABLE_QSPI3) && (BSP_QSPI3_MODE == SPI_MODE_NAND)
    qspi_configure_t flash_cfg3 = FLASH3_CONFIG;
    struct dma_config flash_dma3 = FLASH3_DMA_CONFIG;

    // set clock divider, it related with source clock freq
    div = BSP_GetFlash3DIV();
    clk_mode = RCC_CLK_MOD_FLASH3;
    memcpy(&flash_cfg, &flash_cfg3, sizeof(qspi_configure_t));
    memcpy(&flash_dma, &flash_dma3, sizeof(struct dma_config));
    nand_index = 3;
#elif defined(BSP_ENABLE_QSPI4) && (BSP_QSPI4_MODE == SPI_MODE_NAND)
    qspi_configure_t flash_cfg4 = FLASH4_CONFIG;
    struct dma_config flash_dma4 = FLASH4_DMA_CONFIG;

    // set clock divider, it related with source clock freq
    div = BSP_GetFlash4DIV();
    clk_mode = RCC_CLK_MOD_FLASH4;
    memcpy(&flash_cfg, &flash_cfg4, sizeof(qspi_configure_t));
    memcpy(&flash_dma, &flash_dma4, sizeof(struct dma_config));
    nand_index = 4;
#elif defined(BSP_ENABLE_QSPI2) && (BSP_QSPI2_MODE == SPI_MODE_NAND)
    qspi_configure_t flash_cfg2 = FLASH2_CONFIG;
    struct dma_config flash_dma2 = FLASH2_DMA_CONFIG;

    // set clock divider, it related with source clock freq
    div = BSP_GetFlash2DIV();
    clk_mode = RCC_CLK_MOD_FLASH2;
    memcpy(&flash_cfg, &flash_cfg2, sizeof(qspi_configure_t));
    memcpy(&flash_dma, &flash_dma2, sizeof(struct dma_config));
    nand_index = 2;
#endif

    // for nand, use nand interface to read/write, do not use AHB, so do not care address
    if (flash_cfg.SpiMode == SPI_MODE_NAND)
    {
#ifdef SOC_SF32LB55X
        if (flash_cfg.msize > (QSPI3_MAX_SIZE >> 20))
            flash_cfg.msize = QSPI3_MAX_SIZE >> 20;
#else
        flash_cfg.base = HCPU_MPI_SBUS_ADDR(flash_cfg.base);
#endif
    }
    else
    {
        nand_index = -1;
        return 0;
    }

    spi_nand_handle.handle.freq = flash_get_freq(clk_mode, div, 1);
    // init hardware, set dma, clock
    res = HAL_FLASH_Init(&spi_nand_handle, &flash_cfg, &spi_nand_dma_handle, &flash_dma, div);
    if (res == HAL_OK)
    {
        rt_kprintf("NAND ID 0x%x\n", spi_nand_handle.dev_id);
        nand_pagesize = HAL_NAND_PAGE_SIZE(&spi_nand_handle.handle);
        nand_blksize = HAL_NAND_BLOCK_SIZE(&spi_nand_handle.handle);

#ifndef CFG_BOOTLOADER
        if (gnand_cache_buf == NULL)
            gnand_cache_buf = (uint8_t *)rt_malloc(nand_pagesize + SPI_NAND_MAXOOB_SIZE);
        if (gnand_cache_buf == NULL)
        {
            rt_kprintf("Malloc nand cache buffer fail\n");
            return 0;
        }
#endif
        spi_nand_handle.handle.data_buf = (uint8_t *)gnand_cache_buf;

        spi_nand_handle.handle.buf_mode = 1;    // default set to buffer mode for nand
        HAL_NAND_CONF_ECC(&spi_nand_handle.handle, 1); // default enable ECC if support !
#ifdef BSP_USING_BBM

#ifndef CFG_BOOTLOADER
        if (gbbm_cache_buf == NULL)
            gbbm_cache_buf = (uint8_t *)rt_malloc(nand_pagesize + SPI_NAND_MAXOOB_SIZE);
        if (gbbm_cache_buf == NULL)
        {
            rt_kprintf("Malloc bbm cache buffer fail\n");
            if (gnand_cache_buf)
            {
                free(gnand_cache_buf);
                gnand_cache_buf = NULL;
            }
            return 0;
        }
#endif
        bbm_register_log((bbm_log_func)rt_kprintf);
        bbm_set_page_size(nand_pagesize);
        bbm_set_blk_size(nand_blksize);
#ifdef APP_BSP_TEST
        void nand_clear_pattern(void);
        nand_clear_pattern();
#endif
        sif_bbm_init(spi_nand_handle.total_size, (uint8_t *)gbbm_cache_buf);
#endif

        return 1;
    }

    nand_index = -1;
    return 0;
}

void rt_nand_update_clk(int clk_module, uint16_t clk_div)
{
#ifndef SOC_SF32LB55X
    spi_nand_handle.handle.freq = flash_get_freq(clk_module, clk_div, 1);
    if (spi_nand_handle.handle.freq > 60000000)
        HAL_QSPI_SET_CLK_INV(&spi_nand_handle.handle, 1, 0);
    else
        HAL_QSPI_SET_CLK_INV(&spi_nand_handle.handle, 0, 0);

    HAL_FLASH_SET_CLK_rom(&spi_nand_handle.handle, clk_div);
#endif
    return;
}

int rt_hw_nand_init()
{
    int res = rt_nand_init();
    if (res > 0)
    {
        rt_sem_init(&nand_lock, "nandlock", 1, RT_IPC_FLAG_FIFO);
        nand_sys_init = 1;
    }
    return res;
}

__HAL_ROM_USED int rt_hw_nand_deinit(void)
{
    if (nand_index > 0)
    {
        rt_sem_detach(&nand_lock);
    }

    nand_index = -1;

    return 0;
}


#ifdef BSP_USING_RTTHREAD

#ifdef RT_USING_MTD_NAND

static rt_uint32_t _nand_readid(struct rt_mtd_nand_device *device)
{
    uint32_t base = (uint32_t)device->parent.user_data;

    return rt_nand_read_id(base);
}

static rt_err_t _nand_readpage(struct rt_mtd_nand_device *device,
                               rt_off_t page,
                               rt_uint8_t *data, rt_uint32_t data_len,
                               rt_uint8_t *spare, rt_uint32_t spare_len)
{
    uint32_t base = (uint32_t)device->parent.user_data;
    int res;

#if defined(RT_DFS_ELM_DHARA_ENABLED)
    page += device->block_start * device->pages_per_block;
#endif

    //LOG_I("readpage:%d", page);

#if 0
    {
        extern uint8_t mpi_test_data[];
        extern uint8_t mpi_test_data2[];
        res = rt_nand_read_page(base + page * SPI_NAND_PAGE_SIZE, mpi_test_data, data_len, spare, spare_len);
        memcpy(data, mpi_test_data, data_len);
        res = rt_nand_read_page(base + page * SPI_NAND_PAGE_SIZE, mpi_test_data2, data_len, spare, spare_len);
    }
    {
        int sum = 0, sum2 = 0;
        extern uint8_t mpi_test_data[];
        extern uint8_t mpi_test_data2[];
        for (int i = 0; i < data_len; i++)
        {
            sum ^= mpi_test_data[i];
        }
        for (int i = 0; i < data_len; i++)
        {
            sum2 ^= mpi_test_data2[i];
        }
        if (sum != sum2)
        {
            rt_kprintf("%d, sum:0x%x,0x%x\n", page, sum, sum2);
            RT_ASSERT(0);
        }
    }
#else
    res = rt_nand_read_page(base + page * nand_pagesize, data, data_len, spare, spare_len);
#endif

    if (res <= 0)
    {
        LOG_I("rt_nand_read_page RES %d\n", res);
        return RT_MTD_ENOMEM;
    }

    return 0;
}


static rt_err_t _nand_readpage_with_offset(struct rt_mtd_nand_device *device,
        rt_off_t page,     rt_uint32_t offset,
        rt_uint8_t *data, rt_uint32_t data_len,
        rt_uint8_t *spare, rt_uint32_t spare_len)
{
    uint32_t base = (uint32_t)device->parent.user_data;
    int res;

#if defined(RT_DFS_ELM_DHARA_ENABLED)
    page += device->block_start * device->pages_per_block;
#endif

    if (offset + data_len > nand_pagesize)
    {
        LOG_I("_nand_readpage_with_offset offset + length over page %d\n", offset + data_len);
        return RT_MTD_ENOMEM;
    }

    res = rt_nand_read_page(base + page * nand_pagesize + offset, data, data_len, spare, spare_len);
    if (res <= 0)
    {
        LOG_I("rt_nand_read_page RES %d\n", res);
        return RT_MTD_ENOMEM;
    }

    return 0;
}

static rt_err_t _nand_writepage(struct rt_mtd_nand_device *device,
                                rt_off_t page,
                                const rt_uint8_t *data, rt_uint32_t data_len,
                                const rt_uint8_t *spare, rt_uint32_t spare_len)
{
    uint32_t base = (uint32_t)device->parent.user_data;
    int res;

#if defined(RT_DFS_ELM_DHARA_ENABLED)
    page += device->block_start * device->pages_per_block;
#endif

    //LOG_I("writepage:%d", page);

    res = rt_nand_write_page(base + page * nand_pagesize, data, data_len, spare, spare_len);

    if (res <= 0)
        return RT_MTD_ENOMEM;

    return 0;
}

static rt_err_t _nand_movepage(struct rt_mtd_nand_device *device, rt_off_t src_page, rt_off_t dst_page)
{
    uint32_t base = (uint32_t)device->parent.user_data;
    int res;


    return -RT_MTD_ENOMEM;
}

static rt_err_t _nand_eraseblk(struct rt_mtd_nand_device *device, rt_uint32_t block)
{
    uint32_t base = (uint32_t)device->parent.user_data;
    int res;

#if defined(RT_DFS_ELM_DHARA_ENABLED)
    block += device->block_start;
#endif

    //LOG_I("eraseblk:%d", block);


    res = rt_nand_erase_block((block * nand_blksize) + base);
    if (res)
    {
        return RT_MTD_EIO;
    }

    return 0;
}

static rt_err_t _nand_checkblk(struct rt_mtd_nand_device *device, rt_uint32_t block)
{
    return 0;
}

static rt_err_t _nand_mark_badblk(struct rt_mtd_nand_device *device, rt_uint32_t block)
{
    return 0;
}

static const struct rt_mtd_nand_driver_ops spi_nand_ops =
{
    .read_id = _nand_readid,
    .read_page = _nand_readpage,
    .read_page_with_offset = _nand_readpage_with_offset,
    .write_page = _nand_writepage,
    .move_page = _nand_movepage,
    .erase_block = _nand_eraseblk,
    .check_block = _nand_checkblk,
    .mark_badblock = _nand_mark_badblk,
};

void register_nand_device(uint32_t flash_base, uint32_t offset, uint32_t size, char *name)
{
    uint32_t blk_size  = HAL_NAND_BLOCK_SIZE(&(spi_nand_handle.handle));
    uint16_t pages_per_block = blk_size / HAL_NAND_PAGE_SIZE(&(spi_nand_handle.handle));

    struct rt_mtd_nand_device *mtd = (struct rt_mtd_nand_device *)rt_malloc(sizeof(struct rt_mtd_nand_device));

    // move these configure to menuconfig or configure file?
    mtd->page_size = nand_pagesize;
    mtd->oob_size = 64;
    mtd->oob_free = 24;
    mtd->plane_num = 2;
    mtd->pages_per_block = pages_per_block;
    mtd->block_total = size / blk_size;
    mtd->block_start = offset / blk_size;
    mtd->block_end = mtd->block_start + mtd->block_total - 1;
    mtd->cache_buf = NULL;
    mtd->blk_cnum = 0;
    mtd->ops = &spi_nand_ops;
    mtd->parent.user_data = (void *)flash_base;
    rt_mtd_nand_register_device(name, mtd);

    rt_kprintf("Register %s to mtd device with base addr 0x%08x\n", name, mtd->block_start * blk_size + flash_base);
}

#endif  //RT_USING_MTD_NAND

#ifdef BSP_USING_BBM

#ifdef APP_BSP_TEST
enum NAND_PATT_MODE_T
{
    NAND_NO_ERR = 0,
    NAND_READ_ECC_WORK = 1,
    NAND_READ_ECC_FAIL = 2,
    NAND_WRITE_FAIL = 3,
    NAND_ERASE_FAIL = 4,
    NAND_ERR_MARKED = 5,
    NAND_ECC_SET = 6,
    NAND_FILL_DATA = 7,
};

static enum NAND_PATT_MODE_T lnand_mode;
static uint32_t lnand_patt_addr;

void nand_set_pattern(uint32_t addr, uint32_t mode)
{
    lnand_patt_addr = addr;
    lnand_mode = (enum NAND_PATT_MODE_T)mode;
}
int nand_get_pattern(uint32_t addr)
{
    if (addr == lnand_patt_addr)
        return (int)lnand_mode;

    return (int)NAND_NO_ERR;
}

void nand_clear_pattern(void)
{
    lnand_patt_addr = 0xffffffff;
    lnand_mode = 0;
}

void nand_fill_page(uint32_t addr, uint8_t *data)
{
    if (spi_nand_handle.handle.data_buf != NULL)
    {
        nand_set_pattern(addr, NAND_FILL_DATA);
        memcpy(spi_nand_handle.handle.data_buf, data, nand_pagesize);
    }
}

#endif  // APP_BSP_TEST

// 0 if success, -2 for ecc can not recover , 1 for ecc recover error, -1 for others
int port_read_page(int blk, int page, int offset, uint8_t *buff, uint32_t size, uint8_t *spare, uint32_t spare_len)
{
    int res;
    uint32_t addr = (blk * nand_blksize) + (page * nand_pagesize) + offset;
    if (nand_index < 0)
    {
        LOG_E("No valid nand controller init\n");
        RT_ASSERT(0);
        return 0;
    }

    FLASH_HandleTypeDef *hflash = &(spi_nand_handle.handle);
    if ((offset + size) > nand_pagesize)
    {
        LOG_E("Not support cross page read, offset %d, size %d\n", offset, size);
        RT_ASSERT(0);
    }

#ifdef APP_BSP_TEST
    enum NAND_PATT_MODE_T testp = nand_get_pattern(hflash->base + addr);
    if (testp != NAND_NO_ERR)
    {
        if (NAND_READ_ECC_FAIL == testp)
        {
            hflash->ErrorCode = 0x8002;
            LOG_E("Read has ecc fail pat at 0x%x\n", hflash->base + addr);
            nand_clear_pattern();
            return 0;
        }
        if (NAND_FILL_DATA == testp)
        {
            hflash->ErrorCode = 0;
            memcpy(buff, spi_nand_handle.handle.data_buf, size);
            nand_clear_pattern();
            return size + spare_len;
        }
    }
#endif //APP_BSP_TEST

    SCB_InvalidateDCache_by_Addr((void *)hflash->base, nand_pagesize + SPI_NAND_MAXOOB_SIZE);
    if (((addr & nand_blksize) != 0) && (hflash->wakeup != 0))
        SCB_InvalidateDCache_by_Addr((void *)(hflash->base + (1 << 12)), nand_pagesize + SPI_NAND_MAXOOB_SIZE); // TODO: only 2048 page size has plane select issue
#if (NAND_BUF_CPY_MODE == 1)
    SCB_InvalidateDCache_by_Addr(buff, size);   // for cached buffer, need invalidate manual when using dma
#endif
    local_edma_lock();
    res = HAL_NAND_READ_WITHOOB(hflash, addr, buff, size, spare, spare_len);
    local_edma_unlock();

    if (res <= 0)
    {
        LOG_E("NAND Read error code 0x%x, blk %d, page %d, offset %d, size %d\n",
              hflash->ErrorCode, blk, page, offset, size);
        LOG_E("Buffer %p\n", buff);
    }

#ifdef APP_BSP_TEST
    if (testp != NAND_NO_ERR)
    {
        nand_clear_pattern();
        if ((NAND_READ_ECC_WORK == testp) && (res > 0))
        {
            hflash->ErrorCode = 0x8000;
            LOG_E("Read has ecc pat at 0x%x\n", hflash->base + addr);
        }
    }
#endif

    return res;
}

// 0 if success, -3 for p fail , -1 for others error
int port_write_page(int blk, int page, uint8_t *data, uint8_t *spare, uint32_t spare_len)
{
    int res;
    uint32_t addr = (blk * nand_blksize) + (page * nand_pagesize);
    if (nand_index < 0)
        return 0;
    FLASH_HandleTypeDef *hflash = &(spi_nand_handle.handle);

#ifdef APP_BSP_TEST
    enum NAND_PATT_MODE_T testp = nand_get_pattern(hflash->base + addr);
    if (testp != NAND_NO_ERR)
    {
        if (NAND_WRITE_FAIL == testp)
        {
            hflash->ErrorCode = BBM_NAND_P_FAIL;
            LOG_E("Write fail pat at 0x%x\n", hflash->base + addr);
            nand_clear_pattern();
            return 0;
        }
    }
#endif //APP_BSP_TEST

    res = HAL_NAND_WRITE_WITHOOB(hflash, addr, data, nand_pagesize, spare, spare_len);
    if (res <= 0)
    {
        if (hflash->ErrorCode == BBM_NAND_P_FAIL)
        {
            LOG_E("NAND P FAIL,  error code %d\n", hflash->ErrorCode);
            return RET_P_FAIL;
        }
        else if ((hflash->ErrorCode & 0x8000) != 0)
        {
            LOG_E("NAND write ECC FAIL,  error code %d\n", hflash->ErrorCode);
            return RET_P_FAIL;
        }
        else
            return RET_ERROR;
    }

    return res;
}

// 0 if success, -4 for e fail , 1 for others error
int port_erase_block(int blk)
{
    int res;
    uint32_t addr = (blk * nand_blksize);
    if (nand_index < 0)
        return -1; //RET_ERROR;
    FLASH_HandleTypeDef *hflash = &(spi_nand_handle.handle);

#ifdef APP_BSP_TEST
    enum NAND_PATT_MODE_T testp = nand_get_pattern(hflash->base + addr);
    if (testp != NAND_NO_ERR)
    {
        if (NAND_ERASE_FAIL == testp)
        {
            hflash->ErrorCode = BBM_NAND_E_FAIL;
            LOG_E("Erase fail pat at 0x%x\n", hflash->base + addr);
            nand_clear_pattern();
            return RET_E_FAIL;
        }
        if (NAND_ERR_MARKED == testp)
        {
            hflash->ErrorCode = BBM_NAND_E_FAIL;
            LOG_E("Mark as bad pat at 0x%x\n", hflash->base + addr);
            nand_clear_pattern();
            return -1;
        }
    }
#endif  //APP_BSP_TEST
#ifndef BSP_FORCE_ERASE_NAND
    SCB_InvalidateDCache_by_Addr((void *)hflash->base, nand_pagesize + SPI_NAND_MAXOOB_SIZE);
    if (((addr & nand_blksize) != 0) && (hflash->wakeup != 0))
        SCB_InvalidateDCache_by_Addr((void *)(hflash->base + (1 << 12)), nand_pagesize + SPI_NAND_MAXOOB_SIZE); // TODO: only 2048 page size has plane select issue
    int bad = HAL_NAND_GET_BADBLK(hflash, blk);
    if (bad)   // block is bad, do not erase, or bad mark will be cover
    {
        LOG_E("Blk with address 0x%x is bad, do not erase\n", addr);
        return -1; //-RET_ERROR;
    }
#endif
    res = HAL_NAND_ERASE_BLK(hflash, addr);
    if (res != 0)
    {
        if (hflash->ErrorCode == BBM_NAND_E_FAIL)
            return RET_E_FAIL;
        else
            return RET_ERROR;
    }

    return 0;
}


// mark blk as bad block
int bbm_mark_bb(int blk)
{
    int res;
    if (nand_index < 0)
        return RET_ERROR;
    FLASH_HandleTypeDef *hflash = &(spi_nand_handle.handle);

    SCB_InvalidateDCache_by_Addr((void *)hflash->base, nand_pagesize + SPI_NAND_MAXOOB_SIZE);
    if (((blk & 1) != 0) && (hflash->wakeup != 0))
        SCB_InvalidateDCache_by_Addr((void *)(hflash->base + (1 << 12)), nand_pagesize + SPI_NAND_MAXOOB_SIZE); // TODO: only 2048 page size has plane select issue
    int bad = HAL_NAND_GET_BADBLK(hflash, blk);
    if (bad != 0)   // been marked as bad before
        return 0;

    res = HAL_NAND_MARK_BADBLK(hflash, blk, 1);

    return 0;
}

// check blk if bad block
int bbm_get_bb(int blk)
{
    if (nand_index < 0)
        return RET_ERROR;
    FLASH_HandleTypeDef *hflash = &(spi_nand_handle.handle);

    SCB_InvalidateDCache_by_Addr((void *)hflash->base, nand_pagesize + SPI_NAND_MAXOOB_SIZE);
    if (((blk & 1) != 0) && (hflash->wakeup != 0))
        SCB_InvalidateDCache_by_Addr((void *)(hflash->base + (1 << 12)), nand_pagesize + SPI_NAND_MAXOOB_SIZE); // TODO: only 2048 page size has plane select issue
    int bad = HAL_NAND_GET_BADBLK(hflash, blk);

    return bad;
}

#endif

/**
* @brief  Flash initial for rt-thread, need global value initialized before called.
* @retval 0 if success.
*/
__HAL_ROM_USED int rt_sys_spi_nand_init(void)
{
    int res = 0;

    res = rt_hw_nand_init();
    if (res > 0)
    {


    }

    return 0;
}

INIT_BOARD_EXPORT(rt_sys_spi_nand_init);

//#define DRV_SPI_NAND_TEST
#ifdef DRV_SPI_NAND_TEST

/*** Should test : ***********************
    for read, write , erase and read id.
******************************************/

#define SPI_FLASH_TEST_LEN          SPI_NOR_PAGE_SIZE
#define SPI_FLASH_TEST_VALUE        (0x5a)


static void nand_test_help()
{
    LOG_I("spi nand test command line:\n");
    LOG_I("-write address length [value] \n");
    LOG_I("-read address length \n");
    LOG_I("-erase address length \n");
    LOG_I("-id address \n");
}

static void nand_rm_protect(uint32_t addr)
{
    FLASH_HandleTypeDef *hflash = &(spi_nand_handle.handle);

    extern int nand_clear_status(FLASH_HandleTypeDef * handle);
    nand_clear_status(hflash);
}

static void show_nand_register(uint32_t addr)
{
    uint32_t srl;
    FLASH_HandleTypeDef *hflash = &(spi_nand_handle.handle);
    if (hflash == NULL)
        return;
    srl = HAL_QSPI_GET_SR(hflash);
    rt_kprintf("Get flash SR 0x%08x\n", srl);
}

int cmd_spi_nand(int argc, char *argv[])
{
    uint32_t value, addr, len;
    int i, id, ret;
    uint8_t *buf;

    if (strcmp(argv[1], "-write") == 0)
    {
        if ((strncmp(argv[2], "0x", 2) == 0) || (strncmp(argv[2], "0X", 2) == 0))
            addr = strtoul(argv[2], 0, 16);
        else
            addr = strtoul(argv[2], 0, 10);

        if ((strncmp(argv[3], "0x", 2) == 0) || (strncmp(argv[3], "0X", 2) == 0))
            len = strtoul(argv[3], 0, 16);
        else
            len = strtoul(argv[3], 0, 10);

        if (argc > 4)
            value = atoi(argv[4]);
        else
            value = SPI_FLASH_TEST_VALUE;

        if (len >= 4096) // set max lenght to 4096 to avoid rt_malloc bufer too large
            len = 4096;
        else
            len = nand_pagesize;

        buf = rt_malloc(len);
        if (buf == NULL)
        {
            LOG_I("Alloc buffer len %d fail\n", len);
            return 0;
        }
        for (i = 0; i < len; i++)
            buf[i] = (uint8_t)((value + i) & 0xff);

        ret = rt_nand_write(addr, buf, len);
        LOG_I("write addr 0x%x , len %d, value %d, res %d\n", addr, len, value, ret);

        rt_free(buf);
    }
    else if (strcmp(argv[1], "-read") == 0)
    {
        if ((strncmp(argv[2], "0x", 2) == 0) || (strncmp(argv[2], "0X", 2) == 0))
            addr = strtoul(argv[2], 0, 16);
        else
            addr = strtoul(argv[2], 0, 10);

        if ((strncmp(argv[3], "0x", 2) == 0) || (strncmp(argv[3], "0X", 2) == 0))
            len = strtoul(argv[3], 0, 16);
        else
            len = strtoul(argv[3], 0, 10);

        if (len >= 4096)
            len = 4096;
        else
            len = nand_pagesize;

        buf = rt_malloc(len);
        if (buf == NULL)
        {
            LOG_I("Alloc buffer len %d fial\n", len);
            return 0;
        }

        SCB_InvalidateDCache_by_Addr((void *)addr, len);
        ret = rt_nand_read(addr, buf, len);
        LOG_I("read addr 0x%x , length %d , res %d\n", addr, len, ret);
        if (ret > 0)
        {
            int j;
            for (j = 0; j < ret; j++)
            {
                LOG_RAW("0x%02x  ", buf[j]);
                if ((j & 0x7) == 0x7)
                    LOG_RAW("\n");
            }
            LOG_RAW("\n");
        }
        else
        {
            LOG_E("read from 0x%x with length %d, FAIL %d\n", addr, value, ret);
        }
        rt_free(buf);
    }
    else if (strcmp(argv[1], "-erase") == 0)
    {
        rt_err_t res;
        if ((strncmp(argv[2], "0x", 2) == 0) || (strncmp(argv[2], "0X", 2) == 0))
            addr = strtoul(argv[2], 0, 16);
        else
            addr = strtoul(argv[2], 0, 10);

        if ((strncmp(argv[3], "0x", 2) == 0) || (strncmp(argv[3], "0X", 2) == 0))
            len = strtoul(argv[3], 0, 16);
        else
            len = strtoul(argv[3], 0, 10);

        res = rt_nand_erase(addr, len);
        LOG_I("Flash erase from 0x%x with len %d, res %d\n", addr, len, res);
    }
    else if (strcmp(argv[1], "-id") == 0)
    {
        if ((strncmp(argv[2], "0x", 2) == 0) || (strncmp(argv[2], "0X", 2) == 0))
            addr = strtoul(argv[2], 0, 16);
        else
            addr = strtoul(argv[2], 0, 10);

        if (addr >= spi_nand_handle.base_addr && addr < (spi_nand_handle.base_addr + spi_nand_handle.total_size))
        {
            value = spi_nand_handle.dev_id;
            LOG_I("Flash get manufactory id 0x%x\n", value);
        }
        else
        {
            //FLASH_HandleTypeDef *hflash = &(spi_nand_handle.handle);
            //extern int nand_read_id(FLASH_HandleTypeDef * handle, uint8_t dummy);
            //value = nand_read_id(hflash, 0);
            LOG_I("Invalid nand address\n");
        }

    }
    else if (strcmp(argv[1], "-clear") == 0)
    {
        if ((strncmp(argv[2], "0x", 2) == 0) || (strncmp(argv[2], "0X", 2) == 0))
            addr = strtoul(argv[2], 0, 16);
        else
            addr = strtoul(argv[2], 0, 10);

        nand_rm_protect(addr);
    }
    else if (strcmp(argv[1], "-sr") == 0)
    {
        if ((strncmp(argv[2], "0x", 2) == 0) || (strncmp(argv[2], "0X", 2) == 0))
            addr = strtoul(argv[2], 0, 16);
        else
            addr = strtoul(argv[2], 0, 10);

        show_nand_register(addr);
    }
    else if (strcmp(argv[1], "-size") == 0)
    {
        if ((strncmp(argv[2], "0x", 2) == 0) || (strncmp(argv[2], "0X", 2) == 0))
            addr = strtoul(argv[2], 0, 16);
        else
            addr = strtoul(argv[2], 0, 10);

        value = rt_nand_get_total_size(addr);
        rt_kprintf("Total flash mem size: 0x%08x\n", value);
    }
#ifdef BSP_USING_BBM
    else if (strcmp(argv[1], "-bmem") == 0)
    {
        Sifli_NandBBM *ctx = (Sifli_NandBBM *)bbm_get_context(0);
        rt_kprintf("BBM magic 0x%x, version %d, pointer %p\n", ctx->magic, ctx->version, (void *)ctx);
    }
    else if (strcmp(argv[1], "-bbmlist") == 0)
    {
        Sifli_NandBBM *ctx = (Sifli_NandBBM *)bbm_get_context(0);
        int i, cnt;
        cnt = 0;
        for (i = 0; i < 32; i++)
        {
            if (ctx->stru_tbl[i].logic_blk != 0)
            {
                cnt++;
                rt_kprintf("Map %d: logic blk %d, phy blk %d\n", i, ctx->stru_tbl[i].logic_blk, ctx->stru_tbl[i].physical_blk);
            }
        }
        if (cnt == 0)
            rt_kprintf("No BAD block need map\n");
    }
#endif
    else
    {
        LOG_I("Invalid parameters\n");
        nand_test_help();
    }
    return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_spi_nand, __cmd_spi_nand, Test spi_nand driver);

#endif // DRV_SPI_NAND_TEST
#endif /* BSP_USING_RTTHREAD */

#endif /* BSP_USING_SPI_FLASH */

/// @} drv_nand
/// @} bsp_driver



/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
