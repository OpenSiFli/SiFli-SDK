/**
  ******************************************************************************
  * @file   flash.c
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

#include "rtconfig.h"

#ifndef BSP_USING_PC_SIMULATOR
#include <string.h>
#include "bsp_board.h"
#include "dma_config.h"
#include "flash_config.h"
#include "drv_io.h"
#include "flash_table.h"
#include "register.h"
#if defined(USING_MODULE_RECORD)
    #include "module_record.h"
#endif

/***************************** NOR FLASH *********************************************/

static QSPI_FLASH_CTX_T spi_flash_handle[FLASH_MAX_INSTANCE];
static DMA_HandleTypeDef spi_flash_dma_handle[FLASH_MAX_INSTANCE];

QSPI_FLASH_CTX_T flash_ext_handle = {0};
static int gis_ext_flash = 0;
static int g_ext_flash_id = -1;

int8_t Addr2Id(uint32_t addr)
{
    int8_t id = -1;
    uint32_t i;

    for (i = 0; i < FLASH_MAX_INSTANCE; i++)
    {
        if ((addr >= spi_flash_handle[i].base_addr)
                && (addr < (spi_flash_handle[i].base_addr + spi_flash_handle[i].total_size)))
        {
            id = i;
            break;
        }
    }

    if (i >= FLASH_MAX_INSTANCE)
    {
        if ((addr >= flash_ext_handle.base_addr) && (addr < (flash_ext_handle.base_addr + flash_ext_handle.total_size)))
        {
            id = g_ext_flash_id;
        }
        else
            id = -1;
    }

    return id;
}

__weak int IsExtFlashAddr(uint32_t addr)
{
    return 0;
}

static void *flash_memset(void *s, int c, long count)
{
    char *xs = (char *)s;

    while (count--)
        *xs++ = c;

    return s;
}

__weak void BSP_FLASH_Switch_Ext_Init()
{

}

__weak void BSP_FLASH_Switch_Main_Init()
{

}
__weak void BSP_FLASH_Switch_Ext()
{

}

__weak void BSP_FLASH_Switch_Main()
{

}

__weak void nor_lock(uint32_t addr)
{
    //rt_flash_lock(addr) ;
}

__weak void nor_unlock(uint32_t addr)
{
    //rt_flash_unlock(addr) ;
}

static void flash_lock2(uint32_t addr, uint32_t lock)
{
    if (lock)
        nor_lock(addr);
    else
        nor_unlock(addr);
}

void BSP_Flash_var_init(void)
{
    flash_memset(spi_flash_handle, 0, sizeof(spi_flash_handle));
    flash_memset(&flash_ext_handle, 0, sizeof(flash_ext_handle));
    g_ext_flash_id = -1;
    gis_ext_flash = 0;
    HAL_Delay_us(0);
}

__weak void BSP_FLASH_CS_Ctrl(uint32_t pulldown)
{

}

void *BSP_Flash_get_handle(uint32_t addr)
{
    int8_t id = 0;

    if (IsExtFlashAddr(addr))
    {
        return (void *)&flash_ext_handle.handle;
    }

    id = Addr2Id(addr);

    if (id < 0)
        return NULL;

    return (void *) & (spi_flash_handle[id].handle);
}

void *BSP_Flash_get_handle_by_id(uint8_t id)
{
    if (id >= FLASH_MAX_INSTANCE) return NULL;
    return (void *) & (spi_flash_handle[id]);
}

int BSP_Flash_read_id(uint32_t addr)
{
    if (IsExtFlashAddr(addr))
        return flash_ext_handle.dev_id;

    int8_t id = Addr2Id(addr);
    if (id < 0)
        return FLASH_UNKNOW_ID;

    if (spi_flash_handle[id].dev_id != FLASH_UNKNOW_ID)
        return spi_flash_handle[id].dev_id;

    spi_flash_handle[id].dev_id =   HAL_QSPI_READ_ID(&spi_flash_handle[id].handle);

    return spi_flash_handle[id].dev_id;
}

int BSP_Nor_read(uint32_t addr, uint8_t *buf, int size)
{
    FLASH_HandleTypeDef *hflash;
    int res = 0;

    if (IsExtFlashAddr(addr))
    {
        nor_lock(hflash->base);
#if 0
        hflash = &flash_ext_handle.handle;
        //addr = addr - hflash->base - 0x1000000;
        addr -= flash_ext_handle.handle.base;
        res = HAL_QSPIEX_NOR_READ(hflash, addr, buf, size);
#else
        int i;
        uint8_t *tdst = buf;
        uint8_t *tsrc = (uint8_t *)addr;
        BSP_FLASH_CS_Ctrl(1);
        for (i = 0; i < size; i++)
        {
            //*tdst++ = *tsrc++;
            *tdst = *tsrc;
            tdst++;
            tsrc++;
        }
        res = size;
        BSP_FLASH_CS_Ctrl(0);
#endif
        nor_unlock(hflash->base);

        return res;
    }
    else
    {
        hflash = (FLASH_HandleTypeDef *)BSP_Flash_get_handle(addr);
        if (hflash == NULL)
            return 0;

        // for nor flash, use memory copy directly
        memcpy(buf, (void *)addr, size);
    }
    return size;
}
int BSP_Nor_erase(uint32_t addr, uint32_t size)
{
    uint32_t al_size;
    uint32_t al_addr;
    int ret = 0;
    FLASH_HandleTypeDef *hflash = (FLASH_HandleTypeDef *)BSP_Flash_get_handle(addr);

    if (hflash == NULL)
        return -1;
    if (size == 0)
        return 0;

    if (size >= hflash->size)
    {
        nor_lock(hflash->base);
        ret = HAL_QSPIEX_CHIP_ERASE(hflash);
        nor_unlock(hflash->base);
        return ret;
    }
    // address to offset if needed
    if (addr >= hflash->base)
        addr -= hflash->base;
    if (!IS_ALIGNED((QSPI_NOR_SECT_SIZE << hflash->dualFlash), addr))
    {
        HAL_ASSERT(0);
        ret = -1;
        goto _exit;
    }
    if (!IS_ALIGNED((QSPI_NOR_SECT_SIZE << hflash->dualFlash), size))
    {
        HAL_ASSERT(0);
        ret = -1;
        goto _exit;
    }

    // page erase not support, start addr should be aligned.
    al_addr = GET_ALIGNED_DOWN((QSPI_NOR_SECT_SIZE << hflash->dualFlash), addr);
    al_size = GET_ALIGNED_UP((QSPI_NOR_SECT_SIZE << hflash->dualFlash), size);

alig64k:
    // 1 block 64k aligned, for start addr not aligned do not process, need support later
    if (IS_ALIGNED((QSPI_NOR_BLK64_SIZE << hflash->dualFlash), al_addr) && (al_size >= (QSPI_NOR_BLK64_SIZE << hflash->dualFlash))) // block erease first
    {
        while (al_size >= (QSPI_NOR_BLK64_SIZE << hflash->dualFlash))
        {
            //level = rt_hw_interrupt_disable();
            nor_lock(hflash->base);
            HAL_QSPIEX_BLK64_ERASE(hflash, al_addr);
            //rt_hw_interrupt_enable(level);
            nor_unlock(hflash->base);
            al_size -= QSPI_NOR_BLK64_SIZE << hflash->dualFlash;
            al_addr += QSPI_NOR_BLK64_SIZE << hflash->dualFlash;
        }
    }
    // sector aligned
    if ((al_size >= (QSPI_NOR_SECT_SIZE << hflash->dualFlash)) && IS_ALIGNED((QSPI_NOR_SECT_SIZE << hflash->dualFlash), al_addr))
    {
        while (al_size >= (QSPI_NOR_SECT_SIZE << hflash->dualFlash))
        {
            nor_lock(hflash->base);
            HAL_QSPIEX_SECT_ERASE(hflash, al_addr);
            nor_unlock(hflash->base);
            al_size -= QSPI_NOR_SECT_SIZE << hflash->dualFlash;
            al_addr += QSPI_NOR_SECT_SIZE << hflash->dualFlash;
            if (IS_ALIGNED((QSPI_NOR_BLK64_SIZE << hflash->dualFlash), al_addr) && (al_size >= (QSPI_NOR_BLK64_SIZE << hflash->dualFlash)))
                goto alig64k;
        }
    }

    if (al_size > 0)    // something wrong
    {
        HAL_ASSERT(0);
        ret = -1;
        goto _exit;
    }

_exit:

    return ret;
}

int BSP_Nor_write(uint32_t addr, const uint8_t *buf, uint32_t size)
{
    int i, cnt, taddr, tsize, aligned_size, start;
    uint8_t *tbuf;
    FLASH_HandleTypeDef *hflash = (FLASH_HandleTypeDef *)BSP_Flash_get_handle(addr);

    if (hflash == NULL  || size == 0)
        return 0;

    cnt = 0;
    tsize = size;
    tbuf = (uint8_t *)buf;
    // address to offset if needed
    if (addr >= hflash->base)
        taddr = addr - hflash->base;
    else
        taddr = addr;

    if (hflash->dualFlash) // need lenght and address 2 aligned
    {
        if (taddr & 1) // dst odd, make 2 bytes write
        {
            nor_lock(hflash->base);
            HAL_QSPIEX_FILL_EVEN(hflash, taddr, tbuf, 1);
            nor_unlock(hflash->base);
            //rt_kprintf("Fill prev byte at pos %d\n", taddr);
            // update buffer and address
            taddr++;
            tbuf++;
            tsize--;
            cnt++;
        }
    }

    if (tsize <= 0)
        goto exit;

    // check address page align
    aligned_size = QSPI_NOR_PAGE_SIZE << hflash->dualFlash;
    start = taddr & (aligned_size - 1);
    if (start > 0)    // start address not page aligned
    {
        start = aligned_size - start;   // start to remain size in one page
        if (start > tsize)    // not over one page
        {
            start = tsize;
        }

        if (hflash->dualFlash && (start & 1))   // for this case, it should be the lastest write
        {
            nor_lock(hflash->base);
            i = HAL_QSPIEX_WRITE_PAGE(hflash, taddr, tbuf, start & (~1));

            taddr += i;
            tbuf += i;
            //tsize -= i;
            HAL_QSPIEX_FILL_EVEN(hflash, taddr, tbuf, 0);
            nor_unlock(hflash->base);
            cnt += start;

            goto exit;
        }
        else
        {
            nor_lock(hflash->base);
            i = HAL_QSPIEX_WRITE_PAGE(hflash, taddr, tbuf, start);
            nor_unlock(hflash->base);
            //rt_kprintf("Not page aligned write 0x%x, size %d\n", taddr, start);
            if (i != start)
            {
                //return 0;
                cnt = 0;
                goto exit;
            }
        }
        taddr += start;
        tbuf += start;
        tsize -= start;
        cnt += start;
    }
    // process page aligned data
    while (tsize >= aligned_size)
    {
        nor_lock(hflash->base);
        i = HAL_QSPIEX_WRITE_PAGE(hflash, taddr, tbuf, aligned_size);
        nor_unlock(hflash->base);
        cnt += aligned_size;
        taddr += aligned_size;
        tbuf += aligned_size;
        tsize -= aligned_size;
    }
    // remain size
    if (tsize > 0)
    {
        if (hflash->dualFlash && (tsize & 1))
        {
            nor_lock(hflash->base);
            i = HAL_QSPIEX_WRITE_PAGE(hflash, taddr, tbuf, tsize & (~1));
            nor_unlock(hflash->base);

            if (tsize & 1)  // remain 1 byte
            {
                taddr += i;
                tbuf += i;
                nor_lock(hflash->base);
                HAL_QSPIEX_FILL_EVEN(hflash, taddr, tbuf, 0);
                nor_unlock(hflash->base);
            }
            cnt += tsize;
        }
        else
        {
            nor_lock(hflash->base);
            i = HAL_QSPIEX_WRITE_PAGE(hflash, taddr, tbuf, tsize);
            nor_unlock(hflash->base);
            if (i != tsize)
            {
                //return 0;
                cnt = 0;
                goto exit;
            }
            cnt += tsize;
        }
    }

exit:

    return cnt;
}

__weak uint16_t BSP_GetFlashExtDiv(void)
{
    return 1;
}

uint32_t flash_get_freq(int clk_module, uint16_t clk_div, uint8_t hcpu)
{
    int src;
    uint32_t freq;

    if (clk_div <= 0)
        return 0;

    if (hcpu == 0)
    {
        freq = HAL_RCC_GetSysCLKFreq(CORE_ID_LCPU);
        return freq / clk_div;
    }

    src = HAL_RCC_HCPU_GetClockSrc(clk_module);
#ifdef SOC_SF32LB52X
    if (RCC_CLK_FLASH_DLL2 == src)
    {
        freq = HAL_RCC_HCPU_GetDLL2Freq();
    }
    else if (RCC_CLK_SRC_DLL1 == src)
    {
        freq = HAL_RCC_HCPU_GetDLL1Freq();
    }
    else if (3 == src)  // DBL96
    {
        freq = 96000000;
    }
    else    // CLK_PERI, RC48/XTAL48
    {
        freq = 48000000;
    }
#else
    if (RCC_CLK_FLASH_DLL2 == src)
    {
        freq = HAL_RCC_HCPU_GetDLL2Freq();
    }
    else if (RCC_CLK_FLASH_DLL3 == src)
    {
        freq = HAL_RCC_HCPU_GetDLL3Freq();
    }
    else
    {
        freq = HAL_RCC_GetSysCLKFreq(CORE_ID_HCPU);
    }
#endif
    return freq / clk_div;;
}

__HAL_ROM_USED int BSP_Flash_Init_WithID(uint8_t fid, qspi_configure_t *pflash_cfg, struct dma_config *pdma_cfg, uint8_t dtr)
{
    HAL_StatusTypeDef res = HAL_ERROR;
    uint16_t div;
    QSPI_FLASH_CTX_T *pflash_ctx = &spi_flash_handle[fid];

    pflash_ctx->handle.cs_ctrl = NULL;
    pflash_ctx->handle.lock = NULL;

    if (gis_ext_flash)
    {
        pflash_ctx = &flash_ext_handle;
        pflash_ctx->handle.cs_ctrl = BSP_FLASH_CS_Ctrl;
        pflash_ctx->handle.lock = flash_lock2;
    }

    // check flash size, nor flash max support 32MB
    if (pflash_cfg->msize > 32)
    {
        return 0;
    }

    switch (fid)
    {
    case 0:
        div = BSP_GetFlash1DIV();
        pflash_ctx->handle.freq = flash_get_freq(RCC_CLK_MOD_FLASH1, div, 1);
        break;
    case 1:
        div = BSP_GetFlash2DIV();
        pflash_ctx->handle.freq = flash_get_freq(RCC_CLK_MOD_FLASH2, div, 1);
        break;
#ifdef QSPI3_MEM_BASE
    case 2:
        div = BSP_GetFlash3DIV();
        pflash_ctx->handle.freq = flash_get_freq(RCC_CLK_MOD_FLASH3, div, 1);
        break;
#endif
#ifdef QSPI4_MEM_BASE
    case 3:
        div = BSP_GetFlash4DIV();
#ifdef SOC_SF32LB55X
        pflash_ctx->handle.freq = flash_get_freq(0, div, 0);
#else
        pflash_ctx->handle.freq = flash_get_freq(RCC_CLK_MOD_FLASH4, div, 1);
#endif
        break;
#endif
#ifdef QSPI5_MEM_BASE
    case 4:
        div = BSP_GetFlash5DIV();
        pflash_ctx->handle.freq = flash_get_freq(0, div, 0);
        break;
#endif
    }

#ifdef CFG_BOOTLOADER
#ifndef USE_ATE_MODE
    pflash_cfg->line = 0;    // for bootloader, force to 1 line process
#endif
#endif
    pflash_ctx->handle.buf_mode = dtr;
    if (dtr == 1)
    {
        // todo, adjust with different frequency and version
        pflash_ctx->handle.ecc_en = (1 << 7) | (0xf); // high 1 bits for rx clock inv, low 7 bits for rx clock delay
    }

    if (gis_ext_flash)
    {
        div = BSP_GetFlashExtDiv();
        pflash_ctx->handle.freq = flash_get_freq(RCC_CLK_MOD_FLASH2, div, 1);
    }

    // init hardware, set ctx, dma, clock
    res = HAL_FLASH_Init(pflash_ctx, pflash_cfg, &spi_flash_dma_handle[fid], pdma_cfg, div);
    if (res == HAL_OK)
    {
        // TODO: save local div for dual flash if needed like : pflash_ctx->handle.reserv1 = (uint8_t)div;

        if (gis_ext_flash)
        {
            pflash_ctx->base_addr += spi_flash_handle[fid].total_size;
            pflash_ctx->handle.base = pflash_ctx->base_addr;
            g_ext_flash_id = fid;
        }

        return 1;
    }

    return 0;
}

__HAL_ROM_USED int BSP_Flash_EXT_Init_WithID(uint8_t fid, qspi_configure_t *pflash_cfg, struct dma_config *pdma_cfg, uint8_t dtr)
{
    int ret = 0;
    BSP_FLASH_Switch_Ext_Init();
    gis_ext_flash = 1;
    ret = BSP_Flash_Init_WithID(fid, pflash_cfg, pdma_cfg, dtr);
    gis_ext_flash = 0;
    BSP_FLASH_Switch_Main_Init();

    return ret;
}

__weak int BSP_Flash_hw1_init()
{
#if defined (BSP_ENABLE_QSPI1) && (BSP_QSPI1_MODE == SPI_MODE_NOR)
    /* avoid using memset locating in nor flash */
    flash_memset(&spi_flash_handle[0], 0, sizeof(QSPI_FLASH_CTX_T));

    qspi_configure_t flash_cfg = FLASH1_CONFIG;
    struct dma_config flash_dma = FLASH1_DMA_CONFIG;
    uint8_t dtr;
#ifdef BSP_MPI1_EN_DTR
    // check if use dtr mode, only for nor, for nand ,it should not be defined
    dtr = 1;
#else
    dtr = 0;
#endif //BSP_MPI1_EN_DTR

    return BSP_Flash_Init_WithID(0, &flash_cfg, &flash_dma, dtr);

#endif  // BSP_ENABLE_FLASH1 && (BSP_QSPI1_MODE == 0)

    return 0;
}

__HAL_ROM_USED int BSP_Flash_hw2_init_with_no_dtr(void)
{
#if defined(BSP_ENABLE_QSPI2)  && (BSP_QSPI2_MODE == SPI_MODE_NOR)

#if defined(USING_MODULE_RECORD)
    sifli_record_module(RECORD_FLASH_START);
#endif
    /* avoid using memset locating in nor flash */
    flash_memset(&spi_flash_handle[1], 0, sizeof(QSPI_FLASH_CTX_T));

    qspi_configure_t flash_cfg2 = FLASH2_CONFIG;
    struct dma_config flash_dma2 = FLASH2_DMA_CONFIG;

    int ret = BSP_Flash_Init_WithID(1, &flash_cfg2, &flash_dma2, 0);
#ifdef BSP_QSPI2_DUAL_MODE
    ret |= BSP_Flash_EXT_Init_WithID(1, &flash_cfg2, &flash_dma2, 0);
#endif

    return ret;

#endif  // BSP_ENABLE_QSPI2 && (BSP_QSPI2_MODE == 0)

    return 0;
}

__weak int BSP_Flash_hw2_init()
{
#if defined(BSP_ENABLE_QSPI2)  && (BSP_QSPI2_MODE == SPI_MODE_NOR)

#if defined(USING_MODULE_RECORD)
    sifli_record_module(RECORD_FLASH_START);
#endif
    /* avoid using memset locating in nor flash */
    flash_memset(&spi_flash_handle[1], 0, sizeof(QSPI_FLASH_CTX_T));

    qspi_configure_t flash_cfg2 = FLASH2_CONFIG;
    struct dma_config flash_dma2 = FLASH2_DMA_CONFIG;
    uint8_t dtr = 0;

#ifdef BSP_MPI2_EN_DTR
    // check if use dtr mode, only for nor, for nand ,it should not be defined
    dtr = 1;
#else
    dtr = 0;
#endif //BSP_MPI2_EN_DTR

    return BSP_Flash_Init_WithID(1, &flash_cfg2, &flash_dma2, dtr);

#endif  // BSP_ENABLE_QSPI2 && (BSP_QSPI2_MODE == 0)

    return 0;
}

__weak int BSP_Flash_hw3_init()
{
#if defined(BSP_ENABLE_QSPI3)  && (BSP_QSPI3_MODE == SPI_MODE_NOR)
    //Initial flash3 context
    /* avoid using memset locating in nor flash */
    flash_memset(&spi_flash_handle[2], 0, sizeof(QSPI_FLASH_CTX_T));

    qspi_configure_t flash_cfg3 = FLASH3_CONFIG;
    struct dma_config flash_dma3 = FLASH3_DMA_CONFIG;
    uint8_t dtr;

#ifdef BSP_MPI3_EN_DTR
    // check if use dtr mode, only for nor, for nand ,it should not be defined
    dtr = 1;
#else
    dtr = 0;
#endif //BSP_MPI3_EN_DTR

    return BSP_Flash_Init_WithID(2, &flash_cfg3, &flash_dma3, dtr);
#endif  // BSP_ENABLE_QSPI3 && (BSP_QSPI3_MODE == 0)

    return 0;
}

__weak int BSP_Flash_hw4_init()
{
#if defined BSP_ENABLE_QSPI4 && (BSP_QSPI4_MODE == SPI_MODE_NOR)
    // Init flash4 context
    //memset(&spi_flash_handle[3], 0, sizeof(QSPI_FLASH_CTX_T));

    qspi_configure_t flash_cfg4 = FLASH4_CONFIG;
    struct dma_config flash_dma4 = FLASH4_DMA_CONFIG;
    uint8_t dtr;

#ifdef BSP_MPI4_EN_DTR
    // check if use dtr mode, only for nor, for nand ,it should not be defined
    dtr = 1;
#else
    dtr = 0;
#endif //BSP_MPI4_EN_DTR

    return BSP_Flash_Init_WithID(3, &flash_cfg4, &flash_dma4, dtr);

#endif  // BSP_ENABLE_QSPI4 && (BSP_QSPI4_MODE == 0)

    return 0;
}


__weak int BSP_Flash_hw5_init()
{
#ifdef QSPI5_MEM_BASE

#ifdef BSP_ENABLE_QSPI5
    qspi_configure_t flash_cfg5 = FLASH5_CONFIG;
    struct dma_config flash_dma5 = FLASH5_DMA_CONFIG;

#else
    qspi_configure_t flash_cfg5;
    struct dma_config flash_dma5;

    flash_cfg5.base = FLASH5_BASE_ADDR;
    flash_cfg5.Instance = FLASH5;
    flash_cfg5.line = 2;
    flash_cfg5.msize = 1;
    flash_cfg5.SpiMode = 0;

    flash_dma5.dma_irq = FLASH5_DMA_IRQ;
    flash_dma5.dma_irq_prio = FLASH5_DMA_IRQ_PRIO;
    flash_dma5.Instance = FLASH5_DMA_INSTANCE;
    flash_dma5.request = FLASH5_DMA_REQUEST;
#endif
    uint8_t dtr;

#ifdef BSP_MPI5_EN_DTR
    // check if use dtr mode, only for nor, for nand ,it should not be defined
    dtr = 1;
#else
    dtr = 0;
#endif //BSP_MPI5_EN_DTR

    return BSP_Flash_Init_WithID(4, &flash_cfg5, &flash_dma5, dtr);
#endif
    return 0;
}


/**
* @brief  Flash controller hardware initial.
* @retval each bit for a controller enabled.
*/
__HAL_ROM_USED int BSP_Flash_Init(void)
{
    int fen = 0;

    gis_ext_flash = 0;
    g_ext_flash_id = -1;

    if (BSP_Flash_hw1_init())
        fen |= (1 << 0);

    if (BSP_Flash_hw2_init())
        fen |= (1 << 1);

    if (BSP_Flash_hw3_init())
        fen |= (1 << 2);

    if (BSP_Flash_hw4_init())
        fen |= (1 << 3);

#ifdef BSP_ENABLE_MPI5
    if (BSP_Flash_hw5_init())
        fen |= (1 << 4);
#endif

    return fen;
}

/********************* PSRAM ******************************/

#ifdef BSP_USING_PSRAM

typedef struct
{
    const char *name;
    void *instance;
    uint8_t is_qspi;
    uint16_t msize;  /* size in Mbyte */
    uint32_t base_addr;
} bf0_psram_config_t;

typedef union
{
    FLASH_HandleTypeDef qspi_handle;
#if defined(SF32LB55X) && defined(BSP_USING_PSRAM0)
    PSRAM_HandleTypeDef opi_handle;
#endif
} bf0_psram_handle_t;

static int psram_strcmp(const char *s1, const char *s2)
{
    while (*s1 && *s1 == *s2)
        s1++, s2++;
    return (*s1 - *s2);
}

#ifdef SF32LB55X
const static bf0_psram_config_t bf0_psram_cfg[] =
{
#ifdef BSP_USING_PSRAM0
    {
        .name = "psram0",
        .instance = (void *)PSRAM,
        .is_qspi = 0,
        .msize = PSRAM_FULL_SIZE,
        .base_addr = PSRAM_BASE,
    },
#endif /* BSP_USING_PSRAM0*/
#ifdef BSP_USING_PSRAM4
    {
        .name = "psram4",
        .instance = hwp_qspi4,
        .is_qspi = BSP_QSPI4_MODE,
        .msize = BSP_QSPI4_MEM_SIZE,
        .base_addr = QSPI4_MEM_BASE,
    },
#endif /* BSP_USING_PSRAM4 */

};
#else
const static bf0_psram_config_t bf0_psram_cfg[] =
{
#ifdef BSP_USING_PSRAM1
    {
        .name = "psram1",
        .instance = hwp_qspi1,
        .is_qspi = BSP_QSPI1_MODE,
        .msize = BSP_QSPI1_MEM_SIZE,
        .base_addr = QSPI1_MEM_BASE,
    },
#endif /* BSP_USING_PSRAM1 */
#ifdef BSP_USING_PSRAM2
    {
        .name = "psram2",
        .instance = hwp_qspi2,
        .is_qspi = BSP_QSPI2_MODE,
        .msize = BSP_QSPI2_MEM_SIZE,
        .base_addr = QSPI2_MEM_BASE,
    },
#endif /* BSP_USING_PSRAM2 */
};
#endif

static bf0_psram_handle_t bf0_psram_handle[sizeof(bf0_psram_cfg) / sizeof(bf0_psram_cfg[0])];

static int psram_name2id(char *name)
{
    int i;

    for (i = 0; i < sizeof(bf0_psram_cfg) / sizeof(bf0_psram_config_t); i++)
    {
        if (psram_strcmp(name, bf0_psram_cfg[i].name) == 0)
        {
            return i;
        }
    }

    return -1;
}

#ifdef SF32LB55X

/**
* @brief  psram controller hardware initial.
* @retval 0 if success.
*/
int bsp_psramc_init(void)
{
    HAL_StatusTypeDef res;
    uint32_t i;
    uint32_t psram_num;
    uint8_t idx;
    uint16_t psram_clk_div[] =
    {
#ifdef BSP_USING_PSRAM0
        2,
#endif /* BSP_USING_PSRAM0*/

#ifdef BSP_USING_PSRAM4
        1,
#endif /* BSP_USING_PSRAM4 */
    };

    idx = 0;

#ifdef BSP_USING_PSRAM0
    idx++;
#endif /* BSP_USING_PSRAM0*/
#ifdef BSP_USING_PSRAM4
    psram_clk_div[idx] = BSP_GetFlash4DIV();
    idx++;
#endif /* BSP_USING_PSRAM4 */

    psram_num = sizeof(bf0_psram_cfg) / sizeof(bf0_psram_cfg[0]);

    /* init as 0 in case reach here before bss is initialized */
    flash_memset(&bf0_psram_handle[0], 0, sizeof(bf0_psram_handle));
    for (i = 0; i < psram_num; i++)
    {
        if (bf0_psram_cfg[i].is_qspi)
        {
            FLASH_HandleTypeDef *handle;
            qspi_configure_t qspi_cfg;

            flash_memset(&qspi_cfg, 0, sizeof(qspi_configure_t));

            handle = &bf0_psram_handle[i].qspi_handle;

            qspi_cfg.Instance = (QSPI_TypeDef *)bf0_psram_cfg[i].instance;
            qspi_cfg.SpiMode = SPI_MODE_PSRAM;
            qspi_cfg.msize = bf0_psram_cfg[i].msize;
            qspi_cfg.base = bf0_psram_cfg[i].base_addr;

#ifdef SOC_BF0_HCPU
            if ((AON_PMR_STANDBY == HAL_HPAON_GET_POWER_MODE()) && (qspi_cfg.Instance == hwp_qspi4))
            {
                /* no need to init qspi4 if wakeup from standby */
                continue;
            }
#endif  /* SOC_BF0_HCPU */

            res = HAL_SPI_PSRAM_Init(handle, &qspi_cfg, psram_clk_div[i]);
            HAL_ASSERT(HAL_OK == res);

            /* enable quadline read */
            HAL_FLASH_CFG_AHB_RCMD(handle, 3, 6, 0, 0, 2, 3, 1);
            res = HAL_FLASH_SET_AHB_RCMD(handle, BF0_QPSRAM_QRD);
            HAL_ASSERT(HAL_OK == res);

            /* enable quadline write */
            HAL_FLASH_CFG_AHB_WCMD(handle, 3, 0, 0, 0, 2, 3, 1);
            res = HAL_FLASH_SET_AHB_WCMD(handle, BF0_QPSRAM_QWR);
            HAL_ASSERT(HAL_OK == res);

        }
#ifdef BSP_USING_PSRAM0
        else
        {
            PSRAM_HandleTypeDef *handle;
            handle = &bf0_psram_handle[i].opi_handle;
            handle->Instance = (PSRAMC_TypeDef *)bf0_psram_cfg[i].instance;
#ifdef BSP_USING_DUAL_PSRAM
#ifndef BSP_USING_XCCELA_PSRAM
#error "XCCELA must be enabled for Dual PSRAM"
#endif
            handle->dual_psram = true;
#else
            handle->dual_psram = false;
#endif
#ifdef BSP_USING_XCCELA_PSRAM
            handle->is_xccela = true;
#else
            handle->is_xccela = false;
#endif

            if ((PM_STANDBY_BOOT == SystemPowerOnModeGet()) || (AON_PMR_STANDBY == HAL_HPAON_GET_POWER_MODE()))
            {
                handle->wakeup = true;
            }
            else
            {
                handle->wakeup = false;
            }

            res = HAL_PSRAM_Init(handle);
        }
#endif
    }

    return res;
}

/**
 * @brief Get PSRAM clock frequency.
 * @param addr base address of psram.
 * @return Clock freqency for PSRAM
 */
uint32_t bsp_psram_get_clk(uint32_t addr)
{
    int src;
    uint32_t freq;
    uint32_t i;
    uint16_t psram_num;

    freq = 0;
    psram_num = sizeof(bf0_psram_cfg) / sizeof(bf0_psram_cfg[0]);
    for (i = 0; i < psram_num; i++)
    {
        if (bf0_psram_cfg[i].base_addr == addr)
        {
            if (bf0_psram_cfg[i].is_qspi)
            {
                freq = HAL_QSPI_GET_CLK(&bf0_psram_handle[i].qspi_handle);
            }
            else
            {
                src = HAL_RCC_HCPU_GetClockSrc(RCC_CLK_MOD_PSRAM);
                if (RCC_CLK_PSRAM_DLL2 == src)
                {
                    freq = HAL_RCC_HCPU_GetDLL2Freq();
                }
                else if (RCC_CLK_PSRAM_DLL3 == src)
                {
                    freq = HAL_RCC_HCPU_GetDLL3Freq();
                }
                else
                {
                    freq = HAL_RCC_GetSysCLKFreq(CORE_ID_HCPU);
                }

                freq /= 2;
            }
            break;
        }
    }

    return freq;
}

/**
 * @brief Update PSRAM refresh rate.
 * @param name name of PSRAM controller.
 * @param value self refresh rate value
 * @return RT_EOK if initial success, otherwise, -RT_ERROR.
 */
int bsp_psram_update_refresh_rate(char *name, uint32_t value)
{
    return -1;
}

/**
 * @brief PSRAM hardware enter low power.
 * @param name name of PSRAM controller.
 * @return RT_EOK if initial success, otherwise, -RT_ERROR.
 */
int bsp_psram_enter_low_power(char *name)
{
#ifdef BSP_USING_PSRAM0
    int i = psram_name2id(name);
    if (i < 0)
        return -1;

    if (!bf0_psram_cfg[i].is_qspi)
    {
        if (bf0_psram_handle[i].opi_handle.is_xccela)
            __HAL_PSRAM_SET_LOW_POWER_XCCELA(&bf0_psram_handle[i].opi_handle);
        else
            __HAL_PSRAM_SET_LOW_POWER(&bf0_psram_handle[i].opi_handle);
    }
#endif

    return 0;
}

/**
 * @brief PSRAM hardware enter low power.
 * @param name name of PSRAM controller.
 * @return RT_EOK if initial success, otherwise, -RT_ERROR.
 */
int bsp_psram_deep_power_down(char *name)
{
    return -1;
}

/**
 * @brief PSRAM hardware exit from low power.
 * @param name name of PSRAM controller.
 * @return RT_EOK if initial success, otherwise, -RT_ERROR.
 */
int bsp_psram_exit_low_power(char *name)
{
    int i;

    i = psram_name2id(name);
    if (i < 0)
        return -1;

    if (!bf0_psram_cfg[i].is_qspi)
    {
#ifdef BSP_USING_PSRAM0
        bf0_psram_handle[i].opi_handle.Instance = bf0_psram_cfg[i].instance;
        /* check whether psram handle has been initialized */
        if (bf0_psram_handle[i].opi_handle.Instance)
        {

            HAL_RCC_HCPU_enable(HPSYS_RCC_ENR1_PSRAMC, 1);
            if (bf0_psram_handle[i].opi_handle.is_xccela)
                __HAL_PSRAM_EXIT_LOW_POWER_XCCELA(&bf0_psram_handle[i].opi_handle);
            else
                __HAL_PSRAM_EXIT_LOW_POWER(&bf0_psram_handle[i].opi_handle);

        }
#endif

    }

    return 0;
}

/**
 * @brief PSRAM set partial array self-refresh.
 * @param name name of PSRAM controller.
 * @param top set top part to self-refresh, else set bottom.
 * @param deno denomenator for refresh, like 2 for 1/2 to refresh, only support 2^n,
 *         when larger than 16, all memory not refresh, when 1 or 0, all meory auto refress by default.
 * @return 0 if success.
 */
int bsp_psram_set_pasr(char *name, uint8_t top, uint8_t deno)
{
    return -1;
}

/**
 * @brief PSRAM auto calibrate delay.
 * @param name name of PSRAM controller.
 * @param sck sck delay pointer.
 * @param dqs dqs delay pointer
 * @return 0 if success.
 */
int bsp_psram_auto_calib(char *name, uint8_t *sck, uint8_t *dqs)
{
    return -1;
}

/**
 * @brief Wait psram hardware idle.
 * @return none.
 */
void bsp_psram_wait_idle(char *name)
{
}


#else   // ! SF32LB55X
static uint8_t psram_get_default_type(uint32_t mpi_id)
{
    uint8_t psram_type = SPI_MODE_NOR;
#ifdef SOC_SF32LB52X
    uint32_t pid = (hwp_hpsys_cfg->IDR & HPSYS_CFG_IDR_PID_Msk) >> HPSYS_CFG_IDR_PID_Pos;
    pid &= 7;

    switch (pid)
    {
    case 5: //BOOT_PSRAM_APS_16P:
        psram_type = SPI_MODE_PSRAM;         // 16Mb APM QSPI PSRAM
        break;
    case 4: //BOOT_PSRAM_APS_32P:
        psram_type = SPI_MODE_LEGPSRAM;    // 32Mb APM LEGACY PSRAM
        break;
    case 6: //BOOT_PSRAM_WINBOND:                // Winbond HYPERBUS PSRAM
        psram_type = SPI_MODE_HBPSRAM;
        break;
    case 3: // BOOT_PSRAM_APS_64P:
    case 2: //BOOT_PSRAM_APS_128P:
        psram_type = SPI_MODE_OPSRAM;      // 64Mb APM XCELLA PSRAM
        break;
    default:
        psram_type = SPI_MODE_NOR;
        break;
    }
#elif defined(SF32LB56X)
    uint32_t sip1 = BSP_Get_Sip1_Mode();
    uint32_t sip2 = BSP_Get_Sip2_Mode();
    if (mpi_id == 1)
    {
        if (sip1 != 0)  // if chip pass ate, use it to cover local config?
        {
            switch (sip1)
            {
            case SFPIN_SIP1_APM_XCA64:
                psram_type = SPI_MODE_OPSRAM;
                break;
            case SFPIN_SIP1_APM_LEG32:
                psram_type = SPI_MODE_LEGPSRAM;
                break;
            case SFPIN_SIP1_WINB_HYP64:
            case SFPIN_SIP1_WINB_HYP32:
                psram_type = SPI_MODE_HBPSRAM;
                break;
            default:
                psram_type = SPI_MODE_NOR;
                break;
            }
        }
        else
        {
#ifdef PSRAM_BL_MODE
            psram_type = PSRAM_BL_MODE;
#endif
        }
    }
    else if (mpi_id == 2)
    {
        if (sip2 != 0)  // if chip pass ate, use it to cover local config?
        {
            switch (sip2)
            {
            case SFPIN_SIP2_APM_XCA128:
                psram_type = SPI_MODE_OPSRAM;
                break;
            case SFPIN_SIP2_APM_LEG32:
                psram_type = SPI_MODE_LEGPSRAM;
                break;
            default:
                psram_type = SPI_MODE_NOR;
                break;
            }
        }
        else
        {
#ifdef PSRAM_BL_MODE
            psram_type = PSRAM_BL_MODE;
#endif
        }
    }
#endif

    return psram_type;
}

/**
* @brief  psram controller hardware initial.
* @retval 0 if success.
*/
int bsp_psramc_init(void)
{
    HAL_StatusTypeDef res;
    uint32_t i;
    uint32_t psram_num;
    uint8_t idx;
    uint16_t psram_clk_div[] =
    {
#ifdef BSP_USING_PSRAM1
        1,
#endif /* BSP_USING_PSRAM1 */
#ifdef BSP_USING_PSRAM2
        1,
#endif /* BSP_USING_PSRAM2 */
    };

    idx = 0;

#ifdef BSP_USING_PSRAM1
    psram_clk_div[idx] = BSP_GetFlash1DIV();
    idx++;
#endif /* BSP_USING_PSRAM1 */
#ifdef BSP_USING_PSRAM2
    psram_clk_div[idx] = BSP_GetFlash2DIV();
    idx++;
#endif /* BSP_USING_PSRAM2 */

    psram_num = sizeof(bf0_psram_cfg) / sizeof(bf0_psram_cfg[0]);

#ifdef SOC_SF32LB56X
    /* extend MPI1 space to 64Mb for 56x if psram supported */
    hwp_hpsys_cfg->SYSCR |= HPSYS_CFG_SYSCR_REMAP;
#endif

    /* init as 0 in case reach here before bss is initialized */
    flash_memset(&bf0_psram_handle[0], 0, sizeof(bf0_psram_handle));
    for (i = 0; i < psram_num; i++)
    {
        if (bf0_psram_cfg[i].is_qspi)
        {
            FLASH_HandleTypeDef *handle;
            qspi_configure_t qspi_cfg;
            uint8_t mtype;

            flash_memset(&qspi_cfg, 0, sizeof(qspi_configure_t));

            handle = &bf0_psram_handle[i].qspi_handle;

            qspi_cfg.Instance = (MPI_TypeDef *)bf0_psram_cfg[i].instance;
            mtype = bf0_psram_cfg[i].is_qspi;

            if (qspi_cfg.Instance == hwp_qspi1)
                mtype = psram_get_default_type(1);
            else if (qspi_cfg.Instance == hwp_qspi2)
                mtype = psram_get_default_type(2);

            if (mtype != SPI_MODE_NOR)  // use auto detected type
                qspi_cfg.SpiMode = mtype;
            else    // use configure type
                qspi_cfg.SpiMode = bf0_psram_cfg[i].is_qspi;

            qspi_cfg.msize = bf0_psram_cfg[i].msize;
            qspi_cfg.base = bf0_psram_cfg[i].base_addr;
            if (PM_STANDBY_BOOT == SystemPowerOnModeGet())
                handle->wakeup = 1;
            else
                handle->wakeup = 0;

            HAL_MPI_PSRAM_Init(handle, &qspi_cfg, psram_clk_div[i]);
        }
    }
    return res;
}


/**
 * @brief Get PSRAM clock frequency.
 * @param addr base address of psram.
 * @return Clock freqency for PSRAM
 */
uint32_t bsp_psram_get_clk(uint32_t addr)
{
    uint32_t freq;
    uint32_t i;
    uint16_t psram_num;

    freq = 0;
    psram_num = sizeof(bf0_psram_cfg) / sizeof(bf0_psram_cfg[0]);
    for (i = 0; i < psram_num; i++)
    {
        if (bf0_psram_cfg[i].base_addr == addr
                || (HCPU_MPI_SBUS_ADDR(bf0_psram_cfg[i].base_addr) == HCPU_MPI_SBUS_ADDR(addr)))
        {

            if (bf0_psram_cfg[i].is_qspi)
            {
                freq = HAL_QSPI_GET_CLK(&bf0_psram_handle[i].qspi_handle);

                if (bf0_psram_cfg[i].is_qspi != SPI_MODE_PSRAM) // for ALL OPI, freq auto div 2
                    freq /= 2;
            }

            break;
        }
    }

    return freq;
}

/**
 * @brief Update PSRAM refresh rate.
 * @param name name of PSRAM controller.
 * @param value self refresh rate value
 * @return RT_EOK if initial success, otherwise, -RT_ERROR.
 */
int bsp_psram_update_refresh_rate(char *name, uint32_t value)
{
    int i = psram_name2id(name);
    if (i < 0)
        return -1;

    if (bf0_psram_cfg[i].is_qspi == SPI_MODE_LEGPSRAM)
    {
        HAL_LEGACY_SET_REFRESH(&bf0_psram_handle[i].qspi_handle, value);
    }
    else if ((bf0_psram_cfg[i].is_qspi == SPI_MODE_HPSRAM) || (bf0_psram_cfg[i].is_qspi == SPI_MODE_OPSRAM))
    {
        HAL_MPI_SET_REFRESH(&bf0_psram_handle[i].qspi_handle, value);
    }

    return 0;
}

/**
 * @brief PSRAM hardware enter low power.
 * @param name name of PSRAM controller.
 * @return RT_EOK if initial success, otherwise, -RT_ERROR.
 */
int bsp_psram_enter_low_power(char *name)
{
    int i = psram_name2id(name);
    if (i < 0)
        return -1;

    HAL_MPI_PSRAM_ENT_LOWP(&bf0_psram_handle[i].qspi_handle, bf0_psram_cfg[i].is_qspi);

    return 0;
}

/**
 * @brief PSRAM hardware enter low power.
 * @param name name of PSRAM controller.
 * @return RT_EOK if initial success, otherwise, -RT_ERROR.
 */
int bsp_psram_deep_power_down(char *name)
{
    int i = psram_name2id(name);
    if (i < 0)
        return -1;

    if (bf0_psram_cfg[i].is_qspi == SPI_MODE_LEGPSRAM)  // legacy
    {
        HAL_LEGACY_PSRAM_SLEEP(&bf0_psram_handle[i].qspi_handle);
    }
    else if (bf0_psram_cfg[i].is_qspi == SPI_MODE_HBPSRAM)  // hyper bus
    {
        HAL_HYPER_PSRAM_DPD(&bf0_psram_handle[i].qspi_handle);
    }
    else if (bf0_psram_cfg[i].is_qspi != SPI_MODE_PSRAM)    // opi/hpi
    {
        HAL_MPI_PSRAM_DPD(&bf0_psram_handle[i].qspi_handle);
    }

    return 0;
}

/**
 * @brief PSRAM hardware exit from low power.
 * @param name name of PSRAM controller.
 * @return RT_EOK if initial success, otherwise, -RT_ERROR.
 */
int bsp_psram_exit_low_power(char *name)
{
    int i = psram_name2id(name);
    if (i < 0)
        return -1;

    HAL_MPI_EXIT_LOWP(&bf0_psram_handle[i].qspi_handle, bf0_psram_cfg[i].is_qspi);

    return 0;
}

/**
 * @brief PSRAM set partial array self-refresh.
 * @param name name of PSRAM controller.
 * @param top set top part to self-refresh, else set bottom.
 * @param deno denomenator for refresh, like 2 for 1/2 to refresh, only support 2^n,
 *         when larger than 16, all memory not refresh, when 1 or 0, all meory auto refress by default.
 * @return 0 if success.
 */
int bsp_psram_set_pasr(char *name, uint8_t top, uint8_t deno)
{
    int i = psram_name2id(name);
    if (i < 0)
        return -1;

    if (bf0_psram_cfg[i].is_qspi == SPI_MODE_LEGPSRAM)
    {
        HAL_LEGACY_PSRAM_SET_PASR(&bf0_psram_handle[i].qspi_handle, top, deno);
    }
    else if ((bf0_psram_cfg[i].is_qspi == SPI_MODE_OPSRAM) || (bf0_psram_cfg[i].is_qspi == SPI_MODE_HPSRAM))
    {
        // write a empty command to chip to make psram wake up
        HAL_MPI_PSRAM_SET_PASR(&bf0_psram_handle[i].qspi_handle, top, deno);
    }

    return 0;
}

/**
 * @brief PSRAM auto calibrate delay.
 * @param name name of PSRAM controller.
 * @param sck sck delay pointer.
 * @param dqs dqs delay pointer
 * @return 0 if success.
 */
int bsp_psram_auto_calib(char *name, uint8_t *sck, uint8_t *dqs)
{
    int i = psram_name2id(name);
    if (i < 0)
        return -1;

#if defined(SF32LB56X) || defined(SF32LB52X)
    HAL_MPI_OPSRAM_AUTO_CAL(&bf0_psram_handle[i].qspi_handle, sck, dqs);

    return 0;
#endif

    return -1;
}

/**
 * @brief Wait psram hardware idle.
 * @return none.
 */
void bsp_psram_wait_idle(char *name)
{
    int i;
    volatile int value;

    i = psram_name2id(name);
    if (i < 0)
        return;

    if (bf0_psram_cfg[i].is_qspi == SPI_MODE_LEGPSRAM)
    {
        value = HAL_LEGACY_MR_READ(&bf0_psram_handle[i].qspi_handle, 4);
    }
    else if (bf0_psram_cfg[i].is_qspi == SPI_MODE_HBPSRAM)
    {
        value = HAL_HYPER_PSRAM_ReadID(&bf0_psram_handle[i].qspi_handle, 0);
    }
    else if (bf0_psram_cfg[i].is_qspi == SPI_MODE_PSRAM)
    {
        HAL_FLASH_MANUAL_CMD(&bf0_psram_handle[i].qspi_handle, 0, 0, 0, 0, 0, 0, 0, 1);
        HAL_FLASH_SET_CMD(&bf0_psram_handle[i].qspi_handle, 0, 0);
    }
    else
    {
        value = HAL_MPI_MR_READ(&bf0_psram_handle[i].qspi_handle, 4);
    }

}

#endif  // SF32LB55X
#endif  // BSP_USING_PSRAM
#endif // !BSP_USING_PC_SIMULATOR

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
