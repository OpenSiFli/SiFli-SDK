/**
  ******************************************************************************
  * @file   drv_spi_flash.c
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

#define FLASH_VERSION_STR       "flash2.0.0"
RT_WEAK uint8_t *rt_flash_version()
{
    return (uint8_t *)FLASH_VERSION_STR;
}

#if defined (BSP_USING_SPI_FLASH) || defined(_SIFLI_DOXYGEN_)
#include "drv_config.h"
#include "drv_flash.h"
#include "string.h"
#include "drv_io.h"
#include "flash_table.h"

#ifdef RT_USING_ULOG
    //#define DRV_DEBUG
    #define LOG_TAG                "drv.spi_flash"
    #include <drv_log.h>
#else
    #define LOG_D(...)
    #define LOG_I(...)
    //#define LOG_I(fmt, ...)         rt_kprintf(fmt, ##__VA_ARGS__)
    #define LOG_E(...)
    #define LOG_RAW(...)
    //#define LOG_RAW(fmt,...)         rt_kprintf(fmt, ##__VA_ARGS__)
#endif

#include <stdlib.h>

//#define EN_FLASH_WDT

static struct rt_semaphore         flash_lock[FLASH_MAX_INSTANCE];

extern int IsExtFlashAddr(uint32_t addr);
extern void BSP_FLASH_Switch_Ext();
extern void BSP_FLASH_Switch_Main();

/* Note: this function only support address in memory map range, for some specail cases it not work.
 * Like NAND chip with size larger than 64MB, only low address can get corret result.
*/

extern int8_t Addr2Id(uint32_t addr);

FLASH_HandleTypeDef *Addr2Handle(uint32_t addr)
{
    return (FLASH_HandleTypeDef *)BSP_Flash_get_handle(addr);
}

static uint32_t gflash_lock_value;

void nor_lock(uint32_t addr)
{
    rt_flash_lock(addr) ;
#if !defined(CFG_FACTORY_DEBUG)
    gflash_lock_value = rt_hw_interrupt_disable();
#endif
    if (IsExtFlashAddr(addr))
    {
        BSP_FLASH_Switch_Ext();
    }
}

void nor_unlock(uint32_t addr)
{
    if (IsExtFlashAddr(addr))
    {
        BSP_FLASH_Switch_Main();
    }
#if !defined(CFG_FACTORY_DEBUG)
    rt_hw_interrupt_enable(gflash_lock_value);
#endif

    rt_flash_unlock(addr) ;

}
// local function
static int rt_nor_dtr_cfg(FLASH_HandleTypeDef *hflash, uint8_t dtr_en)
{
    int res = 0;
    rt_base_t level;
#ifndef SOC_SF32LB55X
    if (hflash->buf_mode == 0)  // if dtr not enalbe, do not care, only switch when dtr enable
        return 0;

    level = rt_hw_interrupt_disable();

    HAL_FLASH_NOP_CMD(hflash);  // make sure prev action done

    res = HAL_NOR_CFG_DTR(hflash, dtr_en);
    rt_hw_interrupt_enable(level);
#endif
    return res;
}

static int rt_nor_read_rom(FLASH_HandleTypeDef *hflash, uint32_t addr, uint8_t *buf, int size)
{
#define NOR_READ_THD_SIZE       (128)

    int res = 0;

    if (hflash->cs_ctrl != NULL)
    {
#ifndef SOC_SF32LB55X
        int i, fill, remain;
        uint8_t *tdst = buf;
        uint8_t *tsrc = (uint8_t *)addr;
        uint32_t *aligned_dst;
        uint32_t *aligned_src;
        remain = size;

        while (remain > 0)
        {
            nor_lock(hflash->base);
            hflash->Instance->TIMR = 0xffff;
            fill = remain > NOR_READ_THD_SIZE ? NOR_READ_THD_SIZE : remain;

            i = fill;
            if ((i >= 4) && (((uint32_t)tdst & 3) == 0) && (((uint32_t)tsrc & 3) == 0))
            {
                aligned_dst = (uint32_t *)tdst;
                aligned_src = (uint32_t *)tsrc;

                while (i >= 4)
                {
                    *aligned_dst++ = *aligned_src++;
                    i -= 4;
                }

                tdst = (uint8_t *)aligned_dst;
                tsrc = (uint8_t *)aligned_src;
            }
            while (i--)
                *tdst++ = *tsrc++;

            remain -= fill;
            hflash->Instance->CR &= ~MPI_CR_EN;
            hflash->Instance->TIMR = 0xff;
            hflash->Instance->CR |= MPI_CR_EN;
            nor_unlock(hflash->base);
        }
#endif
    }
    else
    {
        // for nor flash, use memory copy directly
        memcpy(buf, (void *)addr, size);
    }
    return size;
}

static int rt_nor_erase_rom(FLASH_HandleTypeDef *hflash, uint32_t addr, uint32_t size)
{
    uint32_t al_size;
    uint32_t al_addr;
    int ret = 0;

    if (size == 0)
        return 0;

    if (size >= hflash->size)
    {
        //level = rt_hw_interrupt_disable();
        nor_lock(hflash->base);
        rt_nor_dtr_cfg(hflash, 0);
        ret = HAL_QSPIEX_CHIP_ERASE(hflash);
        //rt_hw_interrupt_enable(level);
        rt_nor_dtr_cfg(hflash, 1);
        nor_unlock(hflash->base);
        return ret;
    }

    // address to offset if needed
    if (addr >= hflash->base)
        addr -= hflash->base;
    if (!IS_ALIGNED((QSPI_NOR_SECT_SIZE << hflash->dualFlash), addr))
    {
        RT_ASSERT(0);
        ret = -1;
        goto _exit;
    }
    if (!IS_ALIGNED((QSPI_NOR_SECT_SIZE << hflash->dualFlash), size))
    {
        RT_ASSERT(0);
        ret = -1;
        goto _exit;
    }

    // address alinged down to page, size aligned up to page size
    // page erase not support, start addr should be aligned.
    al_addr = GET_ALIGNED_DOWN((QSPI_NOR_SECT_SIZE << hflash->dualFlash), addr);
    al_size = GET_ALIGNED_UP((QSPI_NOR_SECT_SIZE << hflash->dualFlash), size);

    LOG_D("flash erase from 0x%x + %d to 0x%x + %d\n", addr, size, al_addr, al_size);

    rt_nor_dtr_cfg(hflash, 0);
alig64k:
    // 1 block 64k aligned, for start addr not aligned do not process, need support later
    if (IS_ALIGNED((QSPI_NOR_BLK64_SIZE << hflash->dualFlash), al_addr) && (al_size >= (QSPI_NOR_BLK64_SIZE << hflash->dualFlash))) // block erease first
    {
        while (al_size >= (QSPI_NOR_BLK64_SIZE << hflash->dualFlash))
        {

            nor_lock(hflash->base);
            HAL_QSPIEX_BLK64_ERASE(hflash, al_addr);
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
        ret = -1;
        goto _exit;
    }

_exit:

    rt_nor_dtr_cfg(hflash, 1);
    return ret;

}

static int rt_nor_write_rom(FLASH_HandleTypeDef *hflash, uint32_t addr, const uint8_t *buf, uint32_t size)
{
    int i, cnt, taddr, tsize, aligned_size, start;
    uint8_t *tbuf;
    bool res;
    HAL_StatusTypeDef ret;
    int level;

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

    rt_nor_dtr_cfg(hflash, 0);

    mpu_dcache_clean((void *)buf, size);

    if (hflash->dualFlash) // need lenght and address 2 aligned
    {
        if (taddr & 1) // dst odd, make 2 bytes write
        {
            nor_lock(hflash->base);
            HAL_QSPIEX_FILL_EVEN(hflash, taddr, tbuf, 1);
            nor_unlock(hflash->base);
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
            if (i != start)
            {
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
                cnt = 0;
                goto exit;
            }
            cnt += tsize;
        }
    }

exit:
    rt_nor_dtr_cfg(hflash, 1);

    return cnt;
}



// FLASH operations -------------------------
// Singleton APIs



void *rt_flash_get_handle_by_addr(uint32_t addr)
{
    return (void *)Addr2Handle(addr);
}

/**
  * @brief Enable/disable flash lock, for specail usage only.
  * @param en Enable flash lock function when set to 1.
  * @retval None.
  */
static uint8_t gflash_lock_flag = 1;
void rt_flash_enable_lock(uint8_t en)
{
    gflash_lock_flag = en;
}

void rt_flash_switch_dtr(uint32_t addr, uint8_t dtr_en)
{
    FLASH_HandleTypeDef *hflash = Addr2Handle(addr);
    if (hflash == NULL)
        return;
    rt_nor_dtr_cfg(hflash, dtr_en);
}
__HAL_ROM_USED int rt_flash_get_last_status(uint32_t addr)
{
    FLASH_HandleTypeDef *hflash = Addr2Handle(addr);
    if (hflash == NULL)
        return 0;

    return (int)hflash->ErrorCode;
}

__HAL_ROM_USED void rt_flash_lock(uint32_t addr)
{
    int8_t id = Addr2Id(addr);
    rt_err_t r;

    if (gflash_lock_flag == 0)
        return;

    if (id < 0)
    {
        return;
        //RT_ASSERT(0);
    }
    if (flash_is_enabled(id) == 0)
        return;

    r = rt_sem_take(&flash_lock[id], rt_tick_from_millisecond(RT_WAITING_FOREVER));
    RT_ASSERT(RT_EOK == r);
}


__HAL_ROM_USED void rt_flash_unlock(uint32_t addr)
{
    int8_t id = Addr2Id(addr);
    rt_err_t r;

    if (gflash_lock_flag == 0)
        return;

    if (id < 0)
    {
        return;
        //RT_ASSERT(0);
    }

    r = rt_sem_release(&flash_lock[id]);
    RT_ASSERT(RT_EOK == r);
}

__HAL_ROM_USED int rt_flash_read_id(uint32_t addr)
{
    return BSP_Flash_read_id(addr);
}

__HAL_ROM_USED int rt_flash_read(uint32_t addr, uint8_t *buf, int size)
{
    FLASH_HandleTypeDef *hflash = Addr2Handle(addr);
    if (hflash == NULL || size == 0)
        return 0;

    LOG_D("rt_flash_read: 0x%x, %d\n", addr, size);
    if (hflash->isNand)
        return rt_nand_read(addr, buf, size);
    else
        return rt_nor_read_rom(hflash, addr, buf, size);
}

__HAL_ROM_USED int rt_flash_write(uint32_t addr, const uint8_t *buf, int size)
{
    int cnt = 0;
    uint8_t *locbuf = NULL;
    const uint8_t *tbuf;

    FLASH_HandleTypeDef *hflash = Addr2Handle(addr);
    if (hflash == NULL || size == 0)
        return 0;
    if (hflash->isNand)
        return rt_nand_write(addr, buf, size);

    // source and dest buffer should not on the same flash for flash write
    // and source buffer should not one th buffer with XIP (code read and data read conflict)
    // how to decide XIP running on which flash?
    // And source buffer should not across 1MB border for DMA limit
    // so check source buffer
    if (IS_SAME_FLASH_ADDR(buf, addr) || IS_SPI_NONDMA_RAM_ADDR(buf)
            || (IS_DMA_ACCROSS_1M_BOUNDARY((uint32_t)buf, size)))
    {
        locbuf = (uint8_t *)malloc(size);
        if (locbuf == NULL)
            return 0;
        memcpy(locbuf, buf, size);
        tbuf = (const uint8_t *)locbuf;
    }
    else
        tbuf = buf;

#ifdef EN_FLASH_WDT
    int to_val = HAL_FLASH_GET_WDT_VALUE(hflash);
    if (to_val > 0)
        HAL_FLASH_SET_WDT(hflash, 0);
#endif
    cnt = rt_nor_write_rom(hflash, addr, tbuf, size);
#ifdef EN_FLASH_WDT
    if (to_val > 0)
        HAL_FLASH_SET_WDT(hflash, to_val);
#endif
    SCB_InvalidateDCache_by_Addr((void *)addr, size);
    SCB_InvalidateICache_by_Addr((void *)addr, size);

    if (locbuf)
        free(locbuf);

    return cnt;
}

__HAL_ROM_USED int rt_flash_erase(uint32_t addr, int size)
{
    int ret = RT_EOK;

    FLASH_HandleTypeDef *hflash = Addr2Handle(addr);
    if (hflash == NULL || size == 0)
        return RT_ERROR;
    if (hflash->isNand)
        return rt_nand_erase(addr, size);

    if (size > hflash->size)
        size = hflash->size;

#ifdef EN_FLASH_WDT
    int to_val = HAL_FLASH_GET_WDT_VALUE(hflash);
    if (to_val > 0)
        HAL_FLASH_SET_WDT(hflash, 0);
#endif
    ret = rt_nor_erase_rom(hflash, addr, size);
    ret = (ret == 0) ? RT_EOK : RT_ERROR;
#ifdef EN_FLASH_WDT
    if (to_val > 0)
        HAL_FLASH_SET_WDT(hflash, to_val);
#endif

    SCB_InvalidateDCache_by_Addr((void *)addr, size);
    SCB_InvalidateICache_by_Addr((void *)addr, size);

    return ret;
}

__HAL_ROM_USED void rt_flash_set_alias(uint32_t addr, uint32_t start, uint32_t len, uint32_t offset)
{
    FLASH_HandleTypeDef *fhandle = Addr2Handle(addr);

    if (fhandle == NULL)
        return;

    HAL_FLASH_ALIAS_CFG(fhandle, start, len, offset);
}

__HAL_ROM_USED void rt_flash_set_ctr_nonce(uint32_t addr, uint32_t start, uint32_t end, uint8_t *nonce)
{
    FLASH_HandleTypeDef *fhandle = Addr2Handle(addr);

    if (fhandle == NULL)
        return;

    HAL_FLASH_NONCE_CFG(fhandle, start, end, nonce);
}

__HAL_ROM_USED void rt_flash_enable_aes(uint32_t addr, uint8_t aes256)
{
    FLASH_HandleTypeDef *fhandle = Addr2Handle(addr);

    if (fhandle == NULL)
        return;

    HAL_FLASH_AES_CFG(fhandle, aes256);
}

__HAL_ROM_USED void rt_flash_disable_aes(uint32_t addr)
{
    FLASH_HandleTypeDef *fhandle = Addr2Handle(addr);

    if (fhandle == NULL)
        return;

    HAL_FLASH_ENABLE_AES(fhandle, 0);
}

__HAL_ROM_USED uint32_t rt_flash_get_clk(uint32_t addr)
{
    FLASH_HandleTypeDef *fhandle = Addr2Handle(addr);
    if (fhandle == NULL)
        return 0;
    return HAL_QSPI_GET_CLK(fhandle);
}

__HAL_ROM_USED int rt_flash_get_erase_alignment(uint32_t addr)
{
    FLASH_HandleTypeDef *fhandle = Addr2Handle(addr);
    if (fhandle == NULL)
        return 0;

    if (fhandle->isNand == 0)
        return (QSPI_NOR_SECT_SIZE << fhandle->dualFlash);
    else
        return SPI_NAND_BLK_SIZE; //128*1024;

    return 0;
}

__HAL_ROM_USED int rt_flash_get_total_size(uint32_t addr)
{
    FLASH_HandleTypeDef *fhandle = Addr2Handle(addr);
    if (fhandle == NULL)
    {
        // try if nand valid
        return rt_nand_get_total_size(addr);
    }

    return fhandle->size;
}

__HAL_ROM_USED int rt_flash_get_status_register(uint32_t addr)
{
    FLASH_HandleTypeDef *fhandle = Addr2Handle(addr);
    if (fhandle == NULL)
        return 0;

    return HAL_QSPI_GET_SR(fhandle);
}
/**
* @brief  Set flash to deep power down or wake up, it should not call when XIP.
* @param addr flash base address.
* @param pd   1 means deep power down, 0 means wake up.
*/
__HAL_ROM_USED void rt_flash_power_down(uint32_t addr, int pd)
{
    FLASH_HandleTypeDef *fhandle = Addr2Handle(addr);
    if (fhandle == NULL)
        return;

    nor_lock(fhandle->base);

    if (pd)
        HAL_FLASH_DEEP_PWRDOWN(fhandle);
    else
        HAL_FLASH_RELEASE_DPD(fhandle);
    nor_unlock(fhandle->base);

    //rt_thread_delay(2);
}

void rt_flash_wait_idle(uint32_t addr)
{
    FLASH_HandleTypeDef *fhandle = Addr2Handle(addr);
    if (fhandle == NULL)
        return;
#ifndef SOC_SF32LB55X
    HAL_FLASH_NOP_CMD(fhandle);
#endif
}

/* dual flash mode: for normal image, it should not be changed!
                    for bootloader, it can be set before flash initial.
*/
#ifdef CFG_BOOTLOADER
static uint8_t gflash_dual[FLASH_MAX_INSTANCE] = {0};
#endif

static uint8_t gflash_enabled = 0;
static uint8_t gflash5_enabled = 0;
/**
* @brief  Get dual flash mode.
* @retval dual flash mode.
*/
uint8_t flash_get_dual_mode(uint8_t id)
{
#ifdef CFG_BOOTLOADER
    if (gflash_dual[id] == 2)
        return 1;

    return 0;
#else
    return 0;
#endif
}

/**
* @brief  Get flash enabled or not.
* @retval flash enable status.
*/
uint8_t flash_is_enabled(uint8_t id)
{
    if (gflash_enabled & (1 << id))
        return 1;

    return 0;
}


int rt_flash_get_pass_id(uint32_t addr)
{
    int res;
    uint32_t len;
    uint32_t uid[8];

    FLASH_HandleTypeDef *hflash = Addr2Handle(addr);
    if (hflash == NULL)
        return 0;

    if (hflash->dualFlash)
        len = 32;
    else
        len = 16;

    HAL_QSPI_GET_UID(hflash, uid, len);

    if (hflash->dualFlash)
    {
        if (((uid[7] & 0xff000000) >> 24) == ((uid[7] & 0x00ff0000) >> 16)) // 2 chip pass id same
            res = ((uid[7] & 0xff000000) >> 24);
        else    // pass not same, it should be fail
            res = 0;
    }
    else
        res = (uid[3] & 0xff000000) >> 24;

    return res;
}

int rt_flash_get_uid(uint32_t addr, uint8_t *uid, uint32_t length)
{
    int i, res;
    uint32_t len;
    uint32_t data[8];

    if (uid == NULL || length > 16)
        return 0;

    FLASH_HandleTypeDef *hflash = Addr2Handle(addr);
    if (hflash == NULL)
        return 0;

    if (hflash->dualFlash)
        len = 32;
    else
        len = 16;

    res = HAL_QSPI_GET_UID(hflash, data, len);
    if (res != 0)
        return 0;

    if (hflash->dualFlash)
    {
        // for dual flash, use the first uid
        uint8_t *buf = (uint8_t *)data;
        for (i = 0; i < length; i++)
            uid[i] = buf[2 * i];
    }
    else
    {
        // use byte copy to replace memory copy to avoid function not ready
        uint8_t *buf = (uint8_t *)data;
        for (i = 0; i < length; i++)
            uid[i] = buf[i];
    }

    return length;
}

/**
* @brief  Flash5 controller hardware initial.
* @retval each bit for a controller enabled.
*/
int rt_hw_flash5_init()
{
    if (gflash5_enabled) // has been init before
        return 1;

    int res = BSP_Flash_hw5_init();
    if (res == 1)
    {
        gflash5_enabled = 1;
        rt_sem_init(&flash_lock[4], "flash5", 1, RT_IPC_FLAG_FIFO);
    }

    return res;
}

/**
* @brief  Flash controller hardware initial.
* @retval each bit for a controller enabled.
*/
__HAL_ROM_USED int rt_hw_flash_init(void)
{

    if (gflash_enabled == 0)
    {
        uint32_t i;
        char sem_name[7] = "flash1";
        int fen = 0;

        fen = BSP_Flash_Init();
        gflash_enabled = fen;
        gflash_lock_flag = 1;
        for (i = 0; i < FLASH_MAX_INSTANCE; i++)
        {
            if (flash_is_enabled(i))
            {
                sem_name[5] = '1' + i;
                rt_sem_init(&flash_lock[i], sem_name, 1, RT_IPC_FLAG_FIFO);
            }
        }

#if defined(BSP_ENABLE_QSPI5)  && (BSP_QSPI5_MODE == SPI_MODE_NOR)
        gflash5_enabled = 1;    // flash5 be initialized at system initial
#endif
    }
    return gflash_enabled;
}

/**
* @brief  Flash controller hardware destory.
* @retval 0 if success.
*/
__HAL_ROM_USED int rt_hw_flash_deinit(void)
{
    int i;
    for (i = 0; i < FLASH_MAX_INSTANCE; i++)
    {
        if (flash_is_enabled(i) != 0)
        {
            rt_sem_detach(&flash_lock[i]);
        }
    }
    gflash_enabled = 0;

    return 0;
}

#ifdef BSP_USING_RTTHREAD

#ifdef RT_USING_MTD_NOR

static rt_uint32_t _flash_read_id(struct rt_mtd_nor_device *device)
{
    uint32_t base = (uint32_t)device->parent.user_data;

    return (rt_uint32_t)rt_flash_read_id(base);
}

static rt_size_t _flash_read(struct rt_mtd_nor_device *device, rt_off_t addr, rt_uint8_t *buf, rt_uint32_t size)
{
    uint32_t base = (uint32_t)device->parent.user_data;
    uint32_t offset;

    offset = addr + device->block_start * device->block_size;

    return (rt_size_t)rt_flash_read(base + offset, buf, size);
}

static rt_size_t _flash_write(struct rt_mtd_nor_device *device, rt_off_t addr, const rt_uint8_t *buf, rt_uint32_t size)
{
    uint32_t base = (uint32_t)device->parent.user_data;
    uint32_t offset;

    offset = addr + device->block_start * device->block_size;

    return (rt_size_t) rt_flash_write(base + offset, buf, size);
}

static rt_err_t _flash_erase(struct rt_mtd_nor_device *device, rt_off_t addr, rt_uint32_t size)
{
    uint32_t base = (uint32_t)device->parent.user_data;
    uint32_t offset;

    offset = addr + device->block_start * device->block_size;

    return rt_flash_erase(base + offset,  size);
}

static rt_err_t _flash_control(struct rt_mtd_nor_device *device, int cmd, void *args)
{
    rt_err_t ret = RT_ERROR;
    uint32_t base = (uint32_t)device->parent.user_data;

    if (RT_DEVICE_CTRL_GET_PHY_ADDR == cmd)
    {
        struct rt_device_phy_addr_mapping *addr_mapping = (struct rt_device_phy_addr_mapping *)args;
        addr_mapping->physical_addr = base + addr_mapping->logical_addr;
        ret = RT_EOK;
    }

    return ret;
}


static const struct rt_mtd_nor_driver_ops _flash_ops =
{
    .read_id = _flash_read_id,
    .read = _flash_read,
    .write = _flash_write,
    .erase_block = _flash_erase,     /* erase offset for byte or for block count?*/
    .control = _flash_control,
};


static void register_mtd_nor(uint32_t flash_base, uint32_t offset, uint32_t size, uint32_t sect_size, char *name)
{
    uint32_t fs_start = offset;
    uint32_t fs_size = size;
    uint32_t blk_size;
    uint32_t sector_size;
    FLASH_HandleTypeDef *flash_handle;

    flash_handle = (FLASH_HandleTypeDef *)rt_flash_get_handle_by_addr(flash_base);
    RT_ASSERT(flash_handle);

    blk_size = sect_size;
    sector_size = blk_size;

    struct rt_mtd_nor_device *mtd = (struct rt_mtd_nor_device *)malloc(sizeof(struct rt_mtd_nor_device));
    mtd->block_start = fs_start / blk_size;
    mtd->block_size = blk_size;
    mtd->block_end = mtd->block_start + fs_size / blk_size;
    mtd->sector_size = sector_size;
    mtd->ops = &_flash_ops;
    mtd->parent.user_data = (void *)flash_base;

    rt_mtd_nor_register_device(name, mtd);
    rt_kprintf("Register %s to mtd device with base addr 0x%08x\n", name, mtd->block_start * blk_size + flash_base);

}


void register_nor_device(uint32_t flash_base, uint32_t offset, uint32_t size, char *name)
{
    register_mtd_nor(flash_base, offset, size, 4096, name);
}

#endif  //RT_USING_MTD_NOR


void register_mtd_device(uint32_t address, uint32_t size, char *name)
{
    FLASH_HandleTypeDef *handle = Addr2Handle(address);

    if (handle)
    {
        uint32_t base = handle->base;
        uint32_t offset = address - handle->base;
        register_nor_device(base, offset, size, name);
    }
    else
    {
        handle = rt_nand_get_handle(address);
        if (handle)
        {
            uint32_t base = handle->base;
            uint32_t offset = address - handle->base;
            register_nand_device(base, offset, size, name);
        }
    }
}

/**
* @brief  Flash initial for rt-thread, need global value initialized before called.
* @retval 0 if success.
*/
int rt_sys_spi_flash_init(void)
{
    int res = 0;

    res = rt_hw_flash_init();

    //rt_flash_sys_config();
    return 0;
}

INIT_BOARD_EXPORT(rt_sys_spi_flash_init);

/********************************** SPI FLASH TEST CODE *************************************/
#ifdef RT_USING_FINSH

#ifdef CFG_BOOTLOADER
static void flash_rm_protect(uint32_t addr)
{
    bool res;
    uint32_t srl, srh;
    uint32_t value;
    FLASH_HandleTypeDef *hflash = Addr2Handle(addr);
    if (hflash == NULL)
        return;
    if (hflash->isNand == 0)    // nor
    {
        srl = srh = 0;

        HAL_FLASH_WRITE_DLEN(hflash, 1 << hflash->dualFlash);
        HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_RDSR, 0);
        srl = HAL_FLASH_READ32(hflash) ;

        int ret = HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_RDSR2, 0);
        if (ret == 0)
            srh = HAL_FLASH_READ32(hflash);

        rt_kprintf("Get flash SRL 0x%x, SRH 0x%x, write to 0\n", srl, srh);

        HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_WREN, 0);
        //srl = 0;
        value = 0; //(srh << 8) | srl;

        HAL_FLASH_WRITE_WORD(hflash, value);

        HAL_FLASH_WRITE_DLEN(hflash, 2 << hflash->dualFlash);
        HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_WRSR, 0);
        do
        {
            HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_RDSR, 0);
            res = HAL_FLASH_IS_PROG_DONE(hflash);
        }
        while (!res);

    }

}

int cmd_spi_flash(int argc, char *argv[])
{
    uint32_t value, addr, len;
    int i, id;
    uint8_t *buf;

    extern uint32_t g_reg_lock;
    if (!g_reg_lock)
    {
        return 0;
    }


    if (strcmp(argv[1], "-write") == 0)
    {
        addr = atoi(argv[2]);
        len = atoi(argv[3]);
        if (argc > 4)
            value = atoi(argv[4]);
        else
            value = 0;

        if (argc > 5)
            id = atoi(argv[5]);
        else
            id = 0;

        if (id == 2)
        {
#ifdef BSP_ENABLE_QSPI3
            addr |= QSPI3_MEM_BASE;
#endif
        }
        else if (id == 1)
            addr |= QSPI2_MEM_BASE;
#ifdef BSP_ENABLE_QSPI4
        else if (id == 3)
            addr |= QSPI4_MEM_BASE;
#endif
#ifdef BSP_ENABLE_QSPI5
        else if (id == 4)
            addr |= QSPI5_MEM_BASE;
#endif
        else if (id == 0)
            addr |= QSPI1_MEM_BASE;
        else
            addr |= FLASH_BASE_ADDR;
        if (len > 4096) // set max lenght to 4096 to avoid malloc bufer too large
            len = 4096;

        buf = malloc(len);
        for (i = 0; i < len; i++)
            buf[i] = (uint8_t)((value + i) & 0xff);

        i = rt_flash_write(addr, buf, len);
        rt_kprintf("write addr 0x%x , len %d, value %d, res %d\n", addr, len, value, i);

        free(buf);
    }
    else if (strcmp(argv[1], "-read") == 0)
    {

        addr = atoi(argv[2]);
        len = atoi(argv[3]);
        if (argc > 4)
            id = atoi(argv[4]);
        else
            id = 0;
        if (len > 4096)
            len = 4096;

        if (id == 2)
        {
#ifdef BSP_ENABLE_QSPI3
            addr |= QSPI3_MEM_BASE;
#endif
        }
        else if (id == 1)
            addr |= QSPI2_MEM_BASE;
#ifdef BSP_ENABLE_QSPI4
        else if (id == 3)
            addr |= QSPI4_MEM_BASE;
#endif
#ifdef BSP_ENABLE_QSPI5
        else if (id == 4)
            addr |= QSPI5_MEM_BASE;
#endif
        else if (id == 0)
            addr |= QSPI1_MEM_BASE;
        else
            addr |= FLASH_BASE_ADDR;

        buf = malloc(len);
        if (buf == NULL)
        {
            rt_kprintf("Alloc buffer len %d fial\n", len);
            return 0;
        }

        //SCB_InvalidateDCache_by_Addr((void *)addr, len);
        i = rt_flash_read(addr, buf, len);
        rt_kprintf("read addr 0x%x , length %d , res %d\n", addr, len, i);
        if (i > 0)
        {
            int j;
            for (j = 0; j < i; j++)
            {
                rt_kprintf("0x%02x  ", buf[j]);
                if ((j & 0x7) == 0x7)
                    rt_kprintf("\n");
            }
        }
        else
        {
            rt_kprintf("read from 0x%x with length %d, FAIL %d\n", addr, value, i);
        }
        free(buf);
    }
    else if (strcmp(argv[1], "-erase") == 0)
    {
        rt_err_t res;
        addr = atoi(argv[2]);
        len = atoi(argv[3]);
        if (argc > 4)
            id = atoi(argv[4]);
        else
            id = 0;

        if (id == 2)
        {
#ifdef BSP_ENABLE_QSPI3
            addr |= QSPI3_MEM_BASE;
#endif
        }
        else if (id == 1)
            addr |= QSPI2_MEM_BASE;
#ifdef BSP_ENABLE_QSPI4
        else if (id == 3)
            addr |= QSPI4_MEM_BASE;
#endif
#ifdef BSP_ENABLE_QSPI5
        else if (id == 4)
            addr |= QSPI5_MEM_BASE;
#endif
        else if (id == 0)
            addr |= QSPI1_MEM_BASE;
        else
            addr |= FLASH_BASE_ADDR;

        res = rt_flash_erase(addr, len);
        rt_kprintf("Flash erase from 0x%x with len %d, res %d\n", addr, len, res);
    }
    else if (strcmp(argv[1], "-id") == 0)
    {
        addr = atoi(argv[2]);
        id = atoi(argv[3]);

        if (id == 2)
        {
#ifdef BSP_ENABLE_QSPI3
            addr |= QSPI3_MEM_BASE;
#endif
        }
        else if (id == 1)
            addr |= QSPI2_MEM_BASE;
#ifdef BSP_ENABLE_QSPI4
        else if (id == 3)
            addr |= QSPI4_MEM_BASE;
#endif
#ifdef BSP_ENABLE_QSPI5
        else if (id == 4)
            addr |= QSPI5_MEM_BASE;
#endif
        else if (id == 0)
            addr |= QSPI1_MEM_BASE;
        else
            addr |= FLASH_BASE_ADDR;

        value = rt_flash_read_id(addr);
        rt_kprintf("Flash get manufactory id 0x%x\n", value);
    }
    else if (strcmp(argv[1], "-sr") == 0)
    {
        addr = atoi(argv[2]);
        id = atoi(argv[3]);
        if (id == 2)
        {
#ifdef BSP_ENABLE_QSPI3
            addr |= QSPI3_MEM_BASE;
#endif
        }
        else if (id == 1)
            addr |= QSPI2_MEM_BASE;
#ifdef BSP_ENABLE_QSPI4
        else if (id == 3)
            addr |= QSPI4_MEM_BASE;
#endif
#ifdef BSP_ENABLE_QSPI5
        else if (id == 4)
            addr |= QSPI5_MEM_BASE;
#endif
        else if (id == 0)
            addr |= QSPI1_MEM_BASE;
        else
            addr |= FLASH_BASE_ADDR;
        FLASH_HandleTypeDef *hflash = Addr2Handle(addr);
        if (hflash == NULL)
            return 0;

        rt_kprintf("Get flash SR 0x%08x\n", HAL_QSPI_GET_SR(hflash));
    }
    else if (strcmp(argv[1], "-clear") == 0)
    {
        addr = atoi(argv[2]);
        id = atoi(argv[3]);
        if (id == 2)
        {
#ifdef BSP_ENABLE_QSPI3
            addr |= QSPI3_MEM_BASE;
#endif
        }
        else if (id == 1)
            addr |= QSPI2_MEM_BASE;
#ifdef BSP_ENABLE_QSPI4
        else if (id == 3)
            addr |= QSPI4_MEM_BASE;
#endif
#ifdef BSP_ENABLE_QSPI5
        else if (id == 4)
            addr |= QSPI5_MEM_BASE;
#endif
        else if (id == 0)
            addr |= QSPI1_MEM_BASE;
        else
            addr |= FLASH_BASE_ADDR;
        flash_rm_protect(addr);
    }

    return 0;
}

FINSH_FUNCTION_EXPORT_ALIAS(cmd_spi_flash, __cmd_spi_flash, Test spi_flash driver);

#else
//#define DRV_SPI_FLASH_TEST
#ifdef DRV_SPI_FLASH_TEST

/*** Should test : ***********************
    for read, write , erase and read id.
******************************************/

#define SPI_FLASH_TEST_LEN          SPI_NOR_PAGE_SIZE
#define SPI_FLASH_TEST_VALUE        (0x5a)


static void flsh_test_help()
{
    LOG_I("spi flash test command line:\n");
    LOG_I("-write address length [value] \n");
    LOG_I("-read address length \n");
    LOG_I("-erase address length \n");
    LOG_I("-id address \n");
}

static void flash_rm_protect(uint32_t addr)
{
    bool res;
    uint32_t srl, srh;
    uint32_t value;
    FLASH_HandleTypeDef *hflash = Addr2Handle(addr);
    if (hflash == NULL)
        return;

    if (hflash->isNand == 0)    // nor
    {
        srl = srh = 0;
        //HAL_FLASH_CLEAR_FIFO(hflash, HAL_FLASH_CLR_RX_TX_FIFO);

        HAL_FLASH_WRITE_DLEN(hflash, 1 << hflash->dualFlash);
        HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_RDSR, 0);
        srl = HAL_FLASH_READ32(hflash) ;

        int ret = HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_RDSR2, 0);
        if (ret == 0)
            srh = HAL_FLASH_READ32(hflash);

        LOG_I("Get flash SRL 0x%x, SRH 0x%x\n", srl, srh);
        //LOG_I("Get flash SR 0x%04x%04x\n", srh, srl);
        //LOG_I("Get flash SR 0x%08x\n", srl);
#if 1
        //HAL_FLASH_CLEAR_FIFO(hflash, HAL_FLASH_CLR_RX_TX_FIFO);

        HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_WREN, 0);
        if (hflash->ctable->ecc_sta_mask != 0)
            srl &= ~(hflash->ctable->ecc_sta_mask);
        else
            srl = 0;
        value = (srh << 8) | srl;

        HAL_FLASH_WRITE_WORD(hflash, value);
        //HAL_FLASH_WRITE_DATA(hflash, value);
        //HAL_FLASH_WRITE_WORD(hflash, value);

        HAL_FLASH_WRITE_DLEN(hflash, 2 << hflash->dualFlash);
        HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_WRSR, 0);
        do
        {
            HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_RDSR, 0);
            res = HAL_FLASH_IS_PROG_DONE(hflash);
        }
        while (!res);
#endif
    }
    else // nand
    {
        extern int nand_clear_status(FLASH_HandleTypeDef * handle);
        //extern int nand_reset(FLASH_HandleTypeDef * handle);
        //nand_reset(hflash);
        nand_clear_status(hflash);
    }
}

static void show_status_register(uint32_t addr)
{
    uint32_t srl, srh;
    FLASH_HandleTypeDef *hflash = Addr2Handle(addr);
    if (hflash == NULL)
        return;
    srl = HAL_QSPI_GET_SR(hflash);
    rt_kprintf("Get flash SR 0x%08x\n", srl);
}

static void switch_spi_mode(uint32_t addr, uint32_t qual)
{
    FLASH_HandleTypeDef *hflash = Addr2Handle(addr);
    if (hflash == NULL)
        return;
    if (((hflash->size > NOR_FLASH_MAX_3B_SIZE) && (hflash->dualFlash == 0))
            || (hflash->size > NOR_FLASH_MAX_3B_SIZE * 2))
    {
        if (qual == 0)
            HAL_FLASH_FADDR_SET_QSPI(hflash, false);
        else
            HAL_FLASH_FADDR_SET_QSPI(hflash, true);
    }
    else
    {
        if (qual == 0)
        {
            HAL_FLASH_SET_QUAL_SPI(hflash, false);
        }
        else
        {
            HAL_FLASH_SET_QUAL_SPI(hflash, true);
        }
    }
}

static int set_clk_div(uint32_t addr, uint32_t div)
{
    FLASH_HandleTypeDef *hflash = Addr2Handle(addr);
    if (hflash == NULL)
        return 0;

    HAL_FLASH_SET_CLK_rom(hflash, div);
    return 0;
}

static int get_clk_div(uint32_t addr)
{
    FLASH_HandleTypeDef *hflash = Addr2Handle(addr);
    if (hflash == NULL)
        return 0;

    return (int)HAL_FLASH_GET_DIV(hflash);
}

static void flash_test_dpd(int id, int pd)
{

    FLASH_HandleTypeDef hflash;
    if (id == 0)
        hflash.Instance = FLASH1;
    else if (id == 1)
        hflash.Instance = FLASH2;
#ifdef FLASH3
    else if (id == 2)
        hflash.Instance = FLASH3;
#endif
#ifdef FLASH4
    else if (id == 3)
        hflash.Instance = FLASH4;
#endif
#ifdef FLASH5
    else if (id == 4)
        hflash.Instance = FLASH5;
#endif
    else
        return;

    if (pd)
        HAL_FLASH_DEEP_PWRDOWN(&hflash);
    else
        HAL_FLASH_RELEASE_DPD(&hflash);

    rt_thread_delay(2);
}

static void flash_read_sfdp(int id, uint32_t pos, uint32_t length)
{
    uint32_t *buf;
    FLASH_HandleTypeDef hflash;
    if (id == 0)
        hflash.Instance = FLASH1;
    else if (id == 1)
        hflash.Instance = FLASH2;
#ifdef FLASH3
    else if (id == 2)
        hflash.Instance = FLASH3;
#endif
#ifdef FLASH4
    else if (id == 3)
        hflash.Instance = FLASH4;
#endif
#ifdef FLASH5
    else if (id == 4)
        hflash.Instance = FLASH5;
#endif
    else
        return;

    if ((pos + length) > 256)
    {
        rt_kprintf("pos %d , lenght %d , over 256\n", pos, length);
        return;
    }
    buf = (uint32_t *)malloc(256);
    if (buf == NULL)
    {
        rt_kprintf("malloc buffer fail\n");
        return;
    }

    HAL_FLASH_MANUAL_CMD(&hflash, 0, 1, 8, 0, 0, 2, 1, 1);

    HAL_FLASH_WRITE_DLEN(&hflash, length);

    HAL_FLASH_SET_CMD(&hflash, 0x5a, 0);

    int i;
    for (i = 0; i < length / 4; i++)
        buf[i] = HAL_FLASH_READ32(&hflash);

    for (i = 0; i < length / 4; i++)
    {
        rt_kprintf("0x%08x ", buf[i]);
        if ((i & 3) == 3)
            rt_kprintf("\n");
    }
    // for puya SFDP table:
    // first word fixed 0x50444653 for pos 0
    // ID number at word 0x10
    // DTR support at word 0X30 , bit 19
    if ((pos == 0) && (length >= 52))
    {
        rt_kprintf("SFDF: 0x%08x\n", buf[0]);
        rt_kprintf("MID: 0x%02x\n", buf[4] & 0XFF);
        rt_kprintf("DTR: %d\n", (buf[12] & (1 << 19)) >> 19);
    }

    free(buf);
}

int cmd_spi_flash(int argc, char *argv[])
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

        if (len > 4096) // set max lenght to 4096 to avoid malloc bufer too large
            len = 4096;

        buf = malloc(len);
        if (buf == NULL)
        {
            LOG_I("Alloc buffer len %d fail\n", len);
            return 0;
        }
        for (i = 0; i < len; i++)
            buf[i] = (uint8_t)((value + i) & 0xff);

        ret = rt_flash_write(addr, buf, len);
        LOG_I("write addr 0x%x , len %d, value %d, res %d\n", addr, len, value, ret);

        free(buf);
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

        if (len > 4096)
            len = 4096;

        buf = malloc(len);
        if (buf == NULL)
        {
            LOG_I("Alloc buffer len %d fial\n", len);
            return 0;
        }

        SCB_InvalidateDCache_by_Addr((void *)addr, len);
        ret = rt_flash_read(addr, buf, len);
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
        free(buf);
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

        res = rt_flash_erase(addr, len);
        LOG_I("Flash erase from 0x%x with len %d, res %d\n", addr, len, res);
    }
    else if (strcmp(argv[1], "-id") == 0)
    {
        if ((strncmp(argv[2], "0x", 2) == 0) || (strncmp(argv[2], "0X", 2) == 0))
            addr = strtoul(argv[2], 0, 16);
        else
            addr = strtoul(argv[2], 0, 10);

        value = rt_flash_read_id(addr);
        LOG_I("Flash get manufactory id 0x%x\n", value);
    }
    else if (strcmp(argv[1], "-getdiv") == 0)
    {
        if ((strncmp(argv[2], "0x", 2) == 0) || (strncmp(argv[2], "0X", 2) == 0))
            addr = strtoul(argv[2], 0, 16);
        else
            addr = strtoul(argv[2], 0, 10);

        LOG_I("Flash clock div = %d\n", get_clk_div(addr));
    }
    else if (strcmp(argv[1], "-div") == 0)
    {
        if ((strncmp(argv[2], "0x", 2) == 0) || (strncmp(argv[2], "0X", 2) == 0))
            addr = strtoul(argv[2], 0, 16);
        else
            addr = strtoul(argv[2], 0, 10);

        ret = set_clk_div(addr, value);
        LOG_I("Set clock div res = %d\n", ret);
    }
    else if (strcmp(argv[1], "-clear") == 0)
    {
        if ((strncmp(argv[2], "0x", 2) == 0) || (strncmp(argv[2], "0X", 2) == 0))
            addr = strtoul(argv[2], 0, 16);
        else
            addr = strtoul(argv[2], 0, 10);

        flash_rm_protect(addr);
    }
    else if (strcmp(argv[1], "-sr") == 0)
    {
        if ((strncmp(argv[2], "0x", 2) == 0) || (strncmp(argv[2], "0X", 2) == 0))
            addr = strtoul(argv[2], 0, 16);
        else
            addr = strtoul(argv[2], 0, 10);

        show_status_register(addr);
    }
    else if (strcmp(argv[1], "-size") == 0)
    {
        if ((strncmp(argv[2], "0x", 2) == 0) || (strncmp(argv[2], "0X", 2) == 0))
            addr = strtoul(argv[2], 0, 16);
        else
            addr = strtoul(argv[2], 0, 10);

        value = rt_flash_get_total_size(addr);
        rt_kprintf("Total flash mem size: 0x%08x\n", value);
    }
    else if (strcmp(argv[1], "-uid") == 0)
    {
        if (1)
        {
            uint32_t uid[4];
            if ((strncmp(argv[2], "0x", 2) == 0) || (strncmp(argv[2], "0X", 2) == 0))
                addr = strtoul(argv[2], 0, 16);
            else
                addr = strtoul(argv[2], 0, 10);

            FLASH_HandleTypeDef *fhandle = Addr2Handle(addr);
            ret = rt_flash_get_uid(addr, (uint8_t *)uid, 16);
            if (ret > 0)
                rt_kprintf("FLASH UID 0x%08x 0x%08x 0x%08x 0x%08x\n", uid[0], uid[1], uid[2], uid[3]);
            else
                rt_kprintf("FLASH get UID fail\n");
        }
    }
    else if (strcmp(argv[1], "-otpw") == 0)
    {
        int blk = atoi(argv[3]);
        int res;

        if (blk > 3 || blk < 1)
        {
            LOG_I("OTP block only support block 1/2/3\n");
            return 0;
        }

        if ((strncmp(argv[2], "0x", 2) == 0) || (strncmp(argv[2], "0X", 2) == 0))
            addr = strtoul(argv[2], 0, 16);
        else
            addr = strtoul(argv[2], 0, 10);

        FLASH_HandleTypeDef *fhandle = Addr2Handle(addr);
        if (fhandle == NULL)
            return 0;
        buf = malloc(256);
        if (buf == NULL)
        {
            LOG_I("Malloc buffer fail\n");
            return 0;
        }
        for (i = 0; i < 256; i++)
            buf[i] = (uint8_t)((0 + i) & 0xff);

        int level = rt_hw_interrupt_disable();
        res = HAL_QSPI_WRITE_OTP(fhandle, blk << 12, buf, 256);
        rt_hw_interrupt_enable(level);
        LOG_I("write otp addr 0x%x with res %d\n", blk << 12, res);

        free(buf);
    }
    else if (strcmp(argv[1], "-otpr") == 0)
    {
        int blk = atoi(argv[3]);
        int res;

        if (blk > 3 || blk < 1)
        {
            LOG_I("OTP block only support block 1/2/3\n");
            return 0;
        }

        if ((strncmp(argv[2], "0x", 2) == 0) || (strncmp(argv[2], "0X", 2) == 0))
            addr = strtoul(argv[2], 0, 16);
        else
            addr = strtoul(argv[2], 0, 10);

        FLASH_HandleTypeDef *fhandle = Addr2Handle(addr);
        if (fhandle == NULL)
            return 0;
        buf = malloc(256);
        if (buf == NULL)
        {
            LOG_I("Malloc buffer fail\n");
            return 0;
        }
        int level = rt_hw_interrupt_disable();
        res = HAL_QSPI_READ_OTP(fhandle, blk << 12, buf, 256);
        rt_hw_interrupt_enable(level);
        LOG_I("READ otp addr 0x%x with res %d\n", blk << 12, res);
        if (res > 0)
        {
            int j;
            for (j = 0; j < res; j++)
            {
                LOG_RAW("0x%02x  ", buf[j]);
                if ((j & 0x7) == 0x7)
                    LOG_RAW("\n");
            }
            LOG_RAW("\n");
        }
        free(buf);
    }
    else if (strcmp(argv[1], "-otpe") == 0)
    {
        int blk = atoi(argv[3]);
        int res;

        if (blk > 3 || blk < 1)
        {
            LOG_I("OTP block only support block 1/2/3\n");
            return 0;
        }

        if ((strncmp(argv[2], "0x", 2) == 0) || (strncmp(argv[2], "0X", 2) == 0))
            addr = strtoul(argv[2], 0, 16);
        else
            addr = strtoul(argv[2], 0, 10);
        FLASH_HandleTypeDef *fhandle = Addr2Handle(addr);
        if (fhandle == NULL)
            return 0;

        res = HAL_QSPI_ERASE_OTP(fhandle, blk << 12);
        LOG_I("ERASE otp addr 0x%x with res %d\n", blk << 12, res);

    }
    else if (strcmp(argv[1], "-otpl") == 0)
    {
        int blk = atoi(argv[3]);
        int res;

        if (blk > 3 || blk < 1)
        {
            LOG_I("OTP block only support block 1/2/3\n");
            return 0;
        }

        if ((strncmp(argv[2], "0x", 2) == 0) || (strncmp(argv[2], "0X", 2) == 0))
            addr = strtoul(argv[2], 0, 16);
        else
            addr = strtoul(argv[2], 0, 10);
        FLASH_HandleTypeDef *fhandle = Addr2Handle(addr);
        if (fhandle == NULL)
            return 0;

        res = HAL_QSPI_LOCK_OTP(fhandle, blk << 12);
        LOG_I("LOCK otp addr 0x%x with res %d\n", blk << 12, res);

    }
    else if (strcmp(argv[1], "-pd") == 0)   // deep power down or release
    {
        id = atoi(argv[2]);
        value = atoi(argv[3]);
        flash_test_dpd(id, value);
        LOG_I("set flash %d to dpd or relase %d\n", id, value);
    }
    else if (strcmp(argv[1], "-psid") == 0)
    {
        int res;

        if ((strncmp(argv[2], "0x", 2) == 0) || (strncmp(argv[2], "0X", 2) == 0))
            addr = strtoul(argv[2], 0, 16);
        else
            addr = strtoul(argv[2], 0, 10);

        res = rt_flash_get_pass_id(addr);
        LOG_I("Pass id =  0x%x\n", res);
    }
    else if (strcmp(argv[1], "-m2m") == 0)
    {
        int src;
        if ((strncmp(argv[2], "0x", 2) == 0) || (strncmp(argv[2], "0X", 2) == 0))
            addr = strtoul(argv[2], 0, 16);
        else
            addr = strtoul(argv[2], 0, 10);

        if ((strncmp(argv[3], "0x", 2) == 0) || (strncmp(argv[3], "0X", 2) == 0))
            len = strtoul(argv[3], 0, 16);
        else
            len = strtoul(argv[3], 0, 10);

        if ((strncmp(argv[4], "0x", 2) == 0) || (strncmp(argv[4], "0X", 2) == 0))
            src = strtoul(argv[4], 0, 16);
        else
            src = strtoul(argv[4], 0, 10);

        LOG_I("Write flash src 0x%08x, dst 0x%08x, length %d\n", src, addr, len);
        ret = rt_flash_write(addr, (const uint8_t *)src, len);
        LOG_I("write to addr 0x%x , res %d\n", addr, ret);

        free(buf);
    }
    else if (strcmp(argv[1], "-sfdp") == 0)
    {
        int id = atoi(argv[2]);
        int pos = atoi(argv[3]);
        int len = atoi(argv[4]);
        flash_read_sfdp(id, pos, len);
    }
    else
    {
        LOG_I("Invalid parameters\n");
        flsh_test_help();
    }
    return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_spi_flash, __cmd_spi_flash, Test spi_flash driver);

#endif // DRV_SPI_FLASH_TEST

#endif // CFG_BOOTLOADER

#endif // RT_USING_FINSH

#endif // BSP_USING_RTTHREAD

#endif /* BSP_USING_SPI_FLASH */

/// @} drv_nand
/// @} bsp_driver



/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
