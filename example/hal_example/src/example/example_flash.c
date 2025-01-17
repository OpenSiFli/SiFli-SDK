/**
  ******************************************************************************
  * @file   example_flash.c
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

#include <rtthread.h>
#include <rtdevice.h>
#include <stdlib.h>
#include <string.h>
#include "utest.h"
#include "bf0_hal.h"
#include "board.h"


/*
    This example demo:
       For Z0 example :
        1. Configure FLASH2 PINMUX if test FLASH2.
        2. Initial FLASH2, this need FLASH2 do not initialized by system.
        3. FLASH2 erase, write, read and check result.
        4. FLASH1 erase, write, read and check result.
        5. 4 bytes address mode access(FLASH2) and 3 bytes address mode access(FLASH1)

      For A0 example:
        1. Flash initial, use FLASH2 , memory base / memory size, DMA  should be configure correctly.
        2. Erase flash
        3. Read flash and check if result is 0xff
        4. Write data to flash
        5. Read back and check is same to input data
        6. Repeat step 2 ~ 5 with different address (too large size may call some error when alloc mem, so do not change size)

        Create 3 function for read/write/erase, erase should set aligned address and size.
        Flash command table in flash_table.c, it's command sequence and timing based on each datasheet.
        To add new flash type, need add new table in flash_table.c with correct device id and command timing.
*/

#ifdef HAL_USING_HTOL
    // for SIP FLASH, it has been initial at beginning, do not need initial again
    // for SIP FLASH, test size and position should double check to avoid conflict with running code.
    #define UT_HAL_FLASH_SIP
#endif

static rt_err_t utest_tc_init(void)
{
    return RT_EOK;
}

static rt_err_t utest_tc_cleanup(void)
{
    return RT_EOK;
}

static DMA_HandleTypeDef fdma_data;

#if defined(HAL_MPI_MODULE_ENABLED)&&!defined(SOC_SF32LB52X)

/*======================================= Sample for PRO =================================*/

#define UT_HAL_FLASH_SIZE           (4096)
#define UT_HAL_FLASH_CASE           (3)
#if defined(BSP_USING_BOARD_EC_LB585XXX)
    #define TEST_FLASH_BASE      FLASH1_BASE_ADDR
    #define TEST_FLASH           FLASH1
    #define TEST_FLASH_RCC       RCC_MOD_MPI1
    #define TEST_FLASH_DMA_REQUEST    DMA_REQUEST_0
#else
    #define TEST_FLASH_BASE      FLASH3_BASE_ADDR
    #define TEST_FLASH           FLASH3
    #define TEST_FLASH_RCC       RCC_MOD_MPI3
    #define TEST_FLASH_DMA_REQUEST    DMA_REQUEST_2
#endif /* BSP_USING_BOARD_EC_LB585XXX */

static QSPI_FLASH_CTX_T spi_flash_ctx;
ALIGN(8)
uint8_t odata[UT_HAL_FLASH_SIZE];

ALIGN(8)
uint8_t idata[UT_HAL_FLASH_SIZE];

static int utest_flash_init(void)
{
    struct dma_config flash_dma;
    qspi_configure_t flash_cfg;
    HAL_StatusTypeDef res = HAL_ERROR;
    DMA_HandleTypeDef *dma_handle;

    // initial flash context and dma handle
    memset(&spi_flash_ctx, 0, sizeof(QSPI_FLASH_CTX_T));
#ifdef UT_HAL_FLASH_SIP
    // when test sip flash, can not initial again to avoid xip issue, copy DRV CTX to use.
    extern FLASH_HandleTypeDef *Addr2Handle(uint32_t addr);
    FLASH_HandleTypeDef *sip_flash = Addr2Handle(FLASH_BASE_ADDR);
    memcpy(&spi_flash_ctx.handle, sip_flash, sizeof(FLASH_HandleTypeDef));
    spi_flash_ctx.base_addr = FLASH_BASE_ADDR;
    return 1;
#else
#if 1
    // when test on xip flash, can not initial again to avoid xip issue, copy DRV CTX to use.
    extern FLASH_HandleTypeDef *Addr2Handle(uint32_t addr);
    FLASH_HandleTypeDef *flash_hand = Addr2Handle(TEST_FLASH_BASE);
    if (flash_hand != NULL) // it has been initial before
    {
        memcpy(&spi_flash_ctx.handle, flash_hand, sizeof(FLASH_HandleTypeDef));
        spi_flash_ctx.base_addr = TEST_FLASH_BASE;
        return 1;
    }
#endif
    dma_handle = &fdma_data; //(DMA_HandleTypeDef *)malloc(sizeof(DMA_HandleTypeDef));

    HAL_RCC_EnableModule(TEST_FLASH_RCC); // enable MPI3

    // initial flash configure
    flash_cfg.Instance = TEST_FLASH;        // flash 3
    flash_cfg.base = TEST_FLASH_BASE;  // each flash has a base address in mem_map.h
    flash_cfg.line = 2;         // 0 single, 2 qual line
    flash_cfg.msize = 8;        // 8 MB
    flash_cfg.SpiMode = 0;      // 0 nor, 1, nand, 2 qspi psram

    // initial dma, flash need dma mode for read/write
    flash_dma.dma_irq = DMAC1_CH3_IRQn;
    flash_dma.dma_irq_prio = 0;
    flash_dma.Instance = DMA1_Channel3;
    flash_dma.request = TEST_FLASH_DMA_REQUEST;


    //uint16_t div = BSP_GetFlash3DIV();
    uint16_t div = 4;   // set clock divider, it decide flash controller clock

    // set hardware, set dma, clock, check command table,
    res = HAL_FLASH_Init(&spi_flash_ctx, &flash_cfg, dma_handle, &flash_dma,   div);
    if (res == HAL_OK)
        return 1;
#endif
    return 0;
}

static int utest_flash_read(uint32_t addr, uint8_t *buf, uint32_t size)
{
    FLASH_HandleTypeDef *hflash = &spi_flash_ctx.handle;
    void *dst;

    if (addr < hflash->base && addr < hflash->size)
        dst = (void *)(hflash->base + addr);
    else if (addr >= hflash->base)
        dst = (void *)addr;
    else
    {
        return 0;
    }
    // for nor flash, use memory copy directly, AHB read command has been set in initial
    memcpy(buf, dst, size);

    return size;
}

static int utest_flash_write(uint32_t addr, const uint8_t *buf, uint32_t size)
{
    FLASH_HandleTypeDef *hflash = &spi_flash_ctx.handle;
    int i, cnt, taddr, tsize, aligned_size, start;
    uint8_t *tbuf;
    uint32_t level;

    if (size == 0)
        return 0;

    cnt = 0;
    tsize = size;
    tbuf = (uint8_t *)buf;
    // address to offset if needed
    if (addr >= hflash->base)
        taddr = addr - hflash->base;
    else
        taddr = addr;

    // check address page align
    aligned_size = QSPI_NOR_PAGE_SIZE ;
    start = taddr & (aligned_size - 1);
    if (start > 0)    // start address not page aligned, fill first page
    {
        //return 0; // not support not page aligned write in example ?
        start = aligned_size - start;   // fill data size to first page
        if (start > tsize)    // not over one page
        {
            start = tsize;
        }
        level = rt_hw_interrupt_disable();
        i = HAL_QSPIEX_WRITE_PAGE(hflash, taddr, tbuf, start);
        rt_hw_interrupt_enable(level);
        if (i != start)
        {
            return 0;
        }
        taddr += start;
        tbuf += start;
        tsize -= start;
        cnt += start;
    }

    // process page aligned data
    while (tsize >= aligned_size)
    {
        level = rt_hw_interrupt_disable();
        i = HAL_QSPIEX_WRITE_PAGE(hflash, taddr, tbuf, aligned_size);
        rt_hw_interrupt_enable(level);
        cnt += aligned_size;
        taddr += aligned_size;
        tbuf += aligned_size;
        tsize -= aligned_size;
    }

    // remain size
    if (tsize > 0)
    {
        level = rt_hw_interrupt_disable();
        i = HAL_QSPIEX_WRITE_PAGE(hflash, taddr, tbuf, tsize);
        rt_hw_interrupt_enable(level);
        if (i != tsize)
        {
            return 0;
        }
        cnt += tsize;
    }

    // invalidate cache after write
    SCB_InvalidateDCache_by_Addr((void *)addr, size);
    return cnt;
}

static int utest_flash_erase(uint32_t addr, uint32_t size)
{
    FLASH_HandleTypeDef *hflash = &spi_flash_ctx.handle;
    uint32_t al_size;
    uint32_t al_addr;
    uint32_t level;
    int ret = 0;

    if (size == 0)
        return 0;

    if (size >= hflash->size)
    {
        ret = HAL_QSPIEX_CHIP_ERASE(hflash);
        return ret;
    }

    // address to offset if needed
    if (addr >= hflash->base)
        addr -= hflash->base;

    if (!IS_ALIGNED(QSPI_NOR_SECT_SIZE, addr) || !IS_ALIGNED(QSPI_NOR_SECT_SIZE, size))
    {
        return -1;  // address and size must sector aligned
    }

    al_addr = addr;
    al_size = size;

alig64k:

    // 1 block 64k aligned, for start addr not aligned do not process
    if (IS_ALIGNED(QSPI_NOR_BLK64_SIZE, al_addr) && (al_size >= QSPI_NOR_BLK64_SIZE)) // block erease first
    {
        while (al_size >= QSPI_NOR_BLK64_SIZE)
        {
            level = rt_hw_interrupt_disable();
            HAL_QSPIEX_BLK64_ERASE(hflash, al_addr);
            rt_hw_interrupt_enable(level);
            al_size -= QSPI_NOR_BLK64_SIZE;
            al_addr += QSPI_NOR_BLK64_SIZE;
        }
    }
#if 0   // for some chip like 32MB winbond, it not support 4 byte block32 erase
    // 2 block 32 aligned.
    if ((al_size >= (QSPI_NOR_BLK32_SIZE)) && IS_ALIGNED((QSPI_NOR_BLK32_SIZE), al_addr))
    {
        while (al_size >= (QSPI_NOR_BLK32_SIZE))
        {
            HAL_QSPIEX_BLK32_ERASE(hflash, al_addr);
            al_size -= QSPI_NOR_BLK32_SIZE ;
            al_addr += QSPI_NOR_BLK32_SIZE;
        }
    }
#endif
    // sector aligned
    if ((al_size >= QSPI_NOR_SECT_SIZE) && IS_ALIGNED(QSPI_NOR_SECT_SIZE, al_addr))
    {
        while (al_size >= QSPI_NOR_SECT_SIZE)
        {
            level = rt_hw_interrupt_disable();
            HAL_QSPIEX_SECT_ERASE(hflash, al_addr);
            rt_hw_interrupt_enable(level);
            al_size -= QSPI_NOR_SECT_SIZE ;
            al_addr += QSPI_NOR_SECT_SIZE ;
            if (IS_ALIGNED(QSPI_NOR_BLK64_SIZE, al_addr) && (al_size >= QSPI_NOR_BLK64_SIZE))
                goto alig64k;
        }
    }


    if (al_size > 0)    // something wrong
    {
        return -1;
    }

    // invalidate cache after erase
    SCB_InvalidateDCache_by_Addr((void *)addr, size);

    return 0;
}

static void testcase(int argc, char **argv)
{
    int res, i, j;
#ifdef UT_HAL_FLASH_SIP
    uint32_t address[UT_HAL_FLASH_CASE] = {0x080000, 0x0c0000, 0x0f0000};
#else
    uint32_t address[UT_HAL_FLASH_CASE] = {0x500000, 0x600000, 0x7f0000};
#endif

    // initial input data buffer
    for (i = 0; i < UT_HAL_FLASH_SIZE; i++)
        idata[i] = (uint8_t)((i * 123) & 0xff);

    // 1. initial flash
    res = utest_flash_init();
    uassert_true(res == 1);

    for (i = 0; i < UT_HAL_FLASH_CASE; i++)
    {
        uint32_t addr = address[i] + spi_flash_ctx.base_addr;

        // 2. erase flash
        res = utest_flash_erase(addr, UT_HAL_FLASH_SIZE);
        uassert_true(res == 0);

        // 3. read flash and check result
        res = utest_flash_read(addr, odata, UT_HAL_FLASH_SIZE);
        uassert_true(res == UT_HAL_FLASH_SIZE);
        for (j = 0; j < UT_HAL_FLASH_SIZE / 4; j++)
        {
            if (*((uint32_t *)odata) != 0xffffffff)
            {
                uassert_true(*((uint32_t *)odata) == 0xffffffff);
                break;
            }
        }

        // 4. write data to flash
        res = utest_flash_write(addr, idata, UT_HAL_FLASH_SIZE);
        uassert_true(res == UT_HAL_FLASH_SIZE);

        // 5. read data and compare data
        res = utest_flash_read(addr, odata, UT_HAL_FLASH_SIZE);
        uassert_true(res == UT_HAL_FLASH_SIZE);
        uassert_true(memcmp(idata, odata, UT_HAL_FLASH_SIZE) == 0);
    }

    return ;
}

UTEST_TC_EXPORT(testcase, "example_flash", utest_tc_init, utest_tc_cleanup, 10);


#elif defined(HAL_QSPI_MODULE_ENABLED)

/*======================================= Sample for A0 B55X =================================*/

#define UT_HAL_FLASH_SIZE           (4096)
#define UT_HAL_FLASH_CASE           (3)

static QSPI_FLASH_CTX_T spi_flash_ctx;
uint8_t odata[UT_HAL_FLASH_SIZE];
uint8_t idata[UT_HAL_FLASH_SIZE];

static int utest_flash_init(void)
{
    struct dma_config flash_dma;
    qspi_configure_t flash_cfg;
    HAL_StatusTypeDef res = HAL_ERROR;
    DMA_HandleTypeDef *dma_handle;

    // initial flash context and dma handle
    memset(&spi_flash_ctx, 0, sizeof(QSPI_FLASH_CTX_T));
#ifdef UT_HAL_FLASH_SIP
    // when test sip flash, can not initial again to avoid xip issue, copy DRV CTX to use.
    extern FLASH_HandleTypeDef *Addr2Handle(uint32_t addr);
    FLASH_HandleTypeDef *sip_flash = Addr2Handle(FLASH_BASE_ADDR);
    memcpy(&spi_flash_ctx.handle, sip_flash, sizeof(FLASH_HandleTypeDef));
    spi_flash_ctx.base_addr = FLASH_BASE_ADDR;
    return 1;
#else
    dma_handle = &fdma_data; //(DMA_HandleTypeDef *)malloc(sizeof(DMA_HandleTypeDef));

    HAL_RCC_EnableModule(RCC_MOD_QSPI2);    // enable qspi2 to aviod it disabled by default

    // initial flash configure
    flash_cfg.Instance = FLASH2;        // flash 2
    flash_cfg.base = FLASH2_BASE_ADDR;  // each flash has a base address in mem_map.h
    flash_cfg.line = 2;         // 0 single, 2 qual line
    flash_cfg.msize = 8;        // 8 MB
    flash_cfg.SpiMode = 0;      // 0 nor, 1, nand, 2 qspi psram

    // initial dma, flash need dma mode for read/write
    flash_dma.dma_irq = DMAC1_CH2_IRQn;
    flash_dma.dma_irq_prio = 0;
    flash_dma.Instance = DMA1_Channel2;
    flash_dma.request = DMA_REQUEST_1;


    //uint16_t div = BSP_GetFlash2DIV();
    uint16_t div = 4;   // set clock divider, it decide flash controller clock

    // set hardware, set dma, clock, check command table,
    res = HAL_FLASH_Init(&spi_flash_ctx, &flash_cfg, dma_handle, &flash_dma,   div);
    if (res == HAL_OK)
        return 1;
#endif
    return 0;
}

static int utest_flash_read(uint32_t addr, uint8_t *buf, uint32_t size)
{
    FLASH_HandleTypeDef *hflash = &spi_flash_ctx.handle;
    void *dst;

    if (addr < hflash->base && addr < hflash->size)
        dst = (void *)(hflash->base + addr);
    else if (addr >= hflash->base)
        dst = (void *)addr;
    else
    {
        return 0;
    }
    // for nor flash, use memory copy directly, AHB read command has been set in initial
    memcpy(buf, dst, size);

    return size;
}

static int utest_flash_write(uint32_t addr, const uint8_t *buf, uint32_t size)
{
    FLASH_HandleTypeDef *hflash = &spi_flash_ctx.handle;
    int i, cnt, taddr, tsize, aligned_size, start;
    uint8_t *tbuf;

    if (size == 0)
        return 0;

    cnt = 0;
    tsize = size;
    tbuf = (uint8_t *)buf;
    // address to offset if needed
    if (addr >= hflash->base)
        taddr = addr - hflash->base;
    else
        taddr = addr;

    // check address page align
    aligned_size = QSPI_NOR_PAGE_SIZE ;
    start = taddr & (aligned_size - 1);
    if (start > 0)    // start address not page aligned, fill first page
    {
        //return 0; // not support not page aligned write in example ?
        start = aligned_size - start;   // fill data size to first page
        if (start > tsize)    // not over one page
        {
            start = tsize;
        }
        i = HAL_QSPIEX_WRITE_PAGE(hflash, taddr, tbuf, start);
        if (i != start)
        {
            return 0;
        }
        taddr += start;
        tbuf += start;
        tsize -= start;
        cnt += start;
    }

    // process page aligned data
    while (tsize >= aligned_size)
    {
        i = HAL_QSPIEX_WRITE_PAGE(hflash, taddr, tbuf, aligned_size);
        cnt += aligned_size;
        taddr += aligned_size;
        tbuf += aligned_size;
        tsize -= aligned_size;
    }

    // remain size
    if (tsize > 0)
    {
        i = HAL_QSPIEX_WRITE_PAGE(hflash, taddr, tbuf, tsize);
        if (i != tsize)
        {
            return 0;
        }
        cnt += tsize;
    }

    // invalidate cache after write
    SCB_InvalidateDCache_by_Addr((void *)addr, size);
    return cnt;
}

static int utest_flash_erase(uint32_t addr, uint32_t size)
{
    FLASH_HandleTypeDef *hflash = &spi_flash_ctx.handle;
    uint32_t al_size;
    uint32_t al_addr;
    int ret = 0;

    if (size == 0)
        return 0;

    if (size >= hflash->size)
    {
        ret = HAL_QSPIEX_CHIP_ERASE(hflash);
        return ret;
    }

    // address to offset if needed
    if (addr >= hflash->base)
        addr -= hflash->base;

    if (!IS_ALIGNED(QSPI_NOR_SECT_SIZE, addr) || !IS_ALIGNED(QSPI_NOR_SECT_SIZE, size))
    {
        return -1;  // address and size must sector aligned
    }

    al_addr = addr;
    al_size = size;

    // 1 block 64k aligned, for start addr not aligned do not process
    if (IS_ALIGNED(QSPI_NOR_BLK64_SIZE, al_addr) && (al_size >= QSPI_NOR_BLK64_SIZE)) // block erease first
    {
        while (al_size >= QSPI_NOR_BLK64_SIZE)
        {
            HAL_QSPIEX_BLK64_ERASE(hflash, al_addr);
            al_size -= QSPI_NOR_BLK64_SIZE;
            al_addr += QSPI_NOR_BLK64_SIZE;
        }
    }
#if 0   // for some chip like 32MB winbond, it not support 4 byte block32 erase
    // 2 block 32 aligned.
    if ((al_size >= (QSPI_NOR_BLK32_SIZE)) && IS_ALIGNED((QSPI_NOR_BLK32_SIZE), al_addr))
    {
        while (al_size >= (QSPI_NOR_BLK32_SIZE))
        {
            HAL_QSPIEX_BLK32_ERASE(hflash, al_addr);
            al_size -= QSPI_NOR_BLK32_SIZE ;
            al_addr += QSPI_NOR_BLK32_SIZE;
        }
    }
#endif
    // sector aligned
    if ((al_size >= QSPI_NOR_SECT_SIZE) && IS_ALIGNED(QSPI_NOR_SECT_SIZE, al_addr))
    {
        while (al_size >= QSPI_NOR_SECT_SIZE)
        {
            HAL_QSPIEX_SECT_ERASE(hflash, al_addr);
            al_size -= QSPI_NOR_SECT_SIZE ;
            al_addr += QSPI_NOR_SECT_SIZE ;
        }
    }


    if (al_size > 0)    // something wrong
    {
        return -1;
    }

    // invalidate cache after erase
    SCB_InvalidateDCache_by_Addr((void *)addr, size);

    return 0;
}

static void testcase(int argc, char **argv)
{
    int res, i, j;
#ifdef UT_HAL_FLASH_SIP
    uint32_t address[UT_HAL_FLASH_CASE] = {0x180000, 0x1c0000, 0x1f0000};
#else
    uint32_t address[UT_HAL_FLASH_CASE] = {0x100000, 0x200000, 0x3f0000};
#endif

    // initial input data buffer
    for (i = 0; i < UT_HAL_FLASH_SIZE; i++)
        idata[i] = (uint8_t)((i * 123) & 0xff);

    // 1. initial flash
    res = utest_flash_init();
    uassert_true(res == 1);

    for (i = 0; i < UT_HAL_FLASH_CASE; i++)
    {
        uint32_t addr = address[i] + spi_flash_ctx.base_addr;

        // 2. erase flash
        res = utest_flash_erase(addr, UT_HAL_FLASH_SIZE);
        uassert_true(res == 0);

        // 3. read flash and check result
        res = utest_flash_read(addr, odata, UT_HAL_FLASH_SIZE);
        uassert_true(res == UT_HAL_FLASH_SIZE);
        for (j = 0; j < UT_HAL_FLASH_SIZE / 4; j++)
        {
            if (*((uint32_t *)odata) != 0xffffffff)
            {
                uassert_true(*((uint32_t *)odata) == 0xffffffff);
                break;
            }
        }

        // 4. write data to flash
        res = utest_flash_write(addr, idata, UT_HAL_FLASH_SIZE);
        uassert_true(res == UT_HAL_FLASH_SIZE);

        // 5. read data and compare data
        res = utest_flash_read(addr, odata, UT_HAL_FLASH_SIZE);
        uassert_true(res == UT_HAL_FLASH_SIZE);
        uassert_true(memcmp(idata, odata, UT_HAL_FLASH_SIZE) == 0);
    }

    //free(idata);
    //free(odata);

    return ;
}

UTEST_TC_EXPORT(testcase, "example_flash", utest_tc_init, utest_tc_cleanup, 10);

#endif /*HAL_FLASH_MODULE_ENABLED*/
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
