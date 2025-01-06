/**
  ******************************************************************************
  * @file   bf0_hal_qspi_ex.c
  * @author Sifli software development team
  * @brief   QSPI extension HAL module driver.
  *
  ******************************************************************************
*/
/**
 *
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

#include "bf0_hal.h"
#include <string.h>
#include <stdlib.h>

/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @defgroup FLASH FLASH
  * @brief FLASH HAL module driver
  * @{
  */

#define DEBUG_JLINK         0
#if DEBUG_JLINK
    extern void debug_print(char *str);
    extern uint8_t *htoa(uint8_t *p, uint32_t d);
#endif
#if defined(HAL_QSPI_MODULE_ENABLED)||defined(_SIFLI_DOXYGEN_)

#include "flash_table.h"

int nand_read_id(FLASH_HandleTypeDef *handle, uint8_t dummy);

//extern void rt_kprintf(const char *fmt, ...);


#define QSPI_USE_CMD2
#define NAND_DUAL_REPLACE_QUAL          (0)

__HAL_ROM_USED int HAL_FLASH_DETECT_DUAL(QSPI_FLASH_CTX_T *ctx, qspi_configure_t *cfg)
{
    FLASH_HandleTypeDef *hflash = NULL;
    int res = 0;

    if (ctx == NULL || cfg == NULL)
        return 0;

    hflash = &(ctx->handle);
    HAL_QSPI_Init(hflash, cfg);
    //hflash->Mode = 0;

    // set dual mode , it should not set by user
    HAL_FLASH_SET_DUAL_MODE(hflash, 1);

    // enable QSPI
    HAL_FLASH_ENABLE_QSPI(hflash, 1);

    // get device id, then get table,
    res = HAL_FLASH_ID_DUAL_ID(hflash);

    // disable QSPI
    HAL_FLASH_ENABLE_QSPI(hflash, 0);
    // recover to single mode
    HAL_FLASH_SET_DUAL_MODE(hflash, 0);

    return res;
}

__HAL_ROM_USED int HAL_FLASH_DETECT_SINGLE(QSPI_FLASH_CTX_T *ctx, qspi_configure_t *cfg)
{
    FLASH_HandleTypeDef *hflash = NULL;
    int res = 0;

    if (ctx == NULL || cfg == NULL)
        return 0;

    hflash = &(ctx->handle);
    HAL_QSPI_Init(hflash, cfg);
    //hflash->Mode = 0;

    // set single mode , it should not set by user
    HAL_FLASH_SET_DUAL_MODE(hflash, 0);
    hflash->dualFlash = 0;

    // enable QSPI
    HAL_FLASH_ENABLE_QSPI(hflash, 1);

    // get device id, then get table,
    res = HAL_FLASH_GET_NOR_ID(hflash);

    // disable QSPI
    HAL_FLASH_ENABLE_QSPI(hflash, 0);

    if (res == 0 || res == 0xffffff)
        return 0;

    return 1;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_FLASH_BOOT(QSPI_FLASH_CTX_T *ctx, qspi_configure_t *cfg,
        DMA_HandleTypeDef *dma, struct dma_config *dma_cfg, uint16_t clk_div)
{
    FLASH_HandleTypeDef *hflash = NULL;
    uint8_t fid, did, mtype;

    if (ctx == NULL || cfg == NULL)
        return HAL_ERROR;

    hflash = &(ctx->handle);
    HAL_QSPI_Init(hflash, cfg);

    // init context
    //ctx->dev_id = FLASH_UNKNOW_ID;
    ctx->flash_mode = cfg->SpiMode;
    ctx->base_addr = cfg->base;
    ctx->total_size = cfg->msize * 0x100000;
    ctx->cache_flag = 2;

    // for bootloader with reset, forec single line and nor flash mode to avoid set qspi mode
    hflash->isNand = 0;
    hflash->Mode = 0;
    hflash->dma = dma;
    if (hflash->dma != NULL && dma_cfg != NULL)
    {
        hflash->dma->Instance                 = dma_cfg->Instance;
        hflash->dma->Init.Request             = dma_cfg->request;
        hflash->dma->Init.Direction           = DMA_MEMORY_TO_PERIPH;
        hflash->dma->Init.PeriphInc           = DMA_PINC_DISABLE;
        hflash->dma->Init.MemInc              = DMA_MINC_ENABLE;
        //hflash->dma->Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
        hflash->dma->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        //hflash->dma->Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
        hflash->dma->Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        hflash->dma->Init.Mode                = DMA_NORMAL;
        hflash->dma->Init.Priority            = DMA_PRIORITY_MEDIUM;
        hflash->dma->Init.BurstSize           = 8;
    }

    HAL_FLASH_SET_CLK_rom(hflash, clk_div);
    // set dual mode , it should not set by user
    if (ctx->dual_mode != 0)
    {
        if (ctx->dual_mode == 1)
            HAL_FLASH_SET_DUAL_MODE(hflash, 0);
        else if (ctx->dual_mode == 2)
            HAL_FLASH_SET_DUAL_MODE(hflash, 1);
    }
    // get dual mode for user image
    hflash->dualFlash = HAL_FLASH_GET_DUAL_MODE(hflash);

    // enable QSPI
    HAL_FLASH_ENABLE_QSPI(hflash, 1);

    // get device id, then get table,
    ctx->dev_id = HAL_QSPI_READ_ID(hflash);
    fid = (uint8_t)ctx->dev_id & 0xff;
    mtype = (uint8_t)((ctx->dev_id >> 8) & 0xff);
    did = (uint8_t)((ctx->dev_id >> 16) & 0xff);
    if (hflash->isNand)
        hflash->ctable = spi_nand_get_cmd_by_id(fid, did, mtype);
    else
        hflash->ctable = spi_flash_get_cmd_by_id(fid, did, mtype);
    if (hflash->ctable  == NULL)
        return HAL_ERROR;

    // add software reset to make chip reset to default status (single line ?)
    // set local controller to single line, it only work for bootloader?
    HAL_QSPIEX_FLASH_RESET(hflash);
    HAL_Delay_us(30);

    return HAL_OK;
}

int nand_clear_status(FLASH_HandleTypeDef *handle);

__HAL_ROM_USED HAL_StatusTypeDef HAL_FLASH_Init(QSPI_FLASH_CTX_T *ctx, qspi_configure_t *cfg,
        DMA_HandleTypeDef *dma, struct dma_config *dma_cfg, uint16_t clk_div)
{
    FLASH_HandleTypeDef *hflash = NULL;
    uint8_t fid, did, mtype;
    int size = 0;

    if (ctx == NULL || cfg == NULL)
        return HAL_ERROR;

    hflash = &(ctx->handle);
    HAL_QSPI_Init(hflash, cfg);

    // init context
    //ctx->dev_id = FLASH_UNKNOW_ID;
    ctx->flash_mode = cfg->SpiMode;
    ctx->base_addr = cfg->base;
    ctx->total_size = cfg->msize * 0x100000;
    ctx->cache_flag = 2;

    hflash->isNand = cfg->SpiMode == 1 ? 1 : 0;
    hflash->dma = dma;
    if (hflash->dma != NULL && dma_cfg != NULL)
    {
        hflash->dma->Instance                 = dma_cfg->Instance;
        hflash->dma->Init.Request             = dma_cfg->request;
        hflash->dma->Init.Direction           = DMA_MEMORY_TO_PERIPH;
        hflash->dma->Init.PeriphInc           = DMA_PINC_DISABLE;
        hflash->dma->Init.MemInc              = DMA_MINC_ENABLE;
        //hflash->dma->Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
        hflash->dma->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        //hflash->dma->Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
        hflash->dma->Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        hflash->dma->Init.Mode                = DMA_NORMAL;
        hflash->dma->Init.Priority            = DMA_PRIORITY_MEDIUM;
        hflash->dma->Init.BurstSize           = 1;

        HAL_FLASH_SET_TXSLOT(hflash, hflash->dma->Init.BurstSize);
    }

    HAL_FLASH_SET_CLK_rom(hflash, clk_div);
    // set dual mode , it should not set by user
    if (ctx->dual_mode != 0)
    {
//#ifndef  CFG_BOOTLOADER
//        ASSERT(0);  // only  bootloader can use this branch !
//#endif
        if (ctx->dual_mode == 1)
            HAL_FLASH_SET_DUAL_MODE(hflash, 0);
        else if (ctx->dual_mode == 2)
        {
            HAL_FLASH_SET_DUAL_MODE(hflash, 1);
        }
    }
    // get dual mode for user image
    hflash->dualFlash = HAL_FLASH_GET_DUAL_MODE(hflash);

    // enable QSPI
    HAL_FLASH_ENABLE_QSPI(hflash, 1);

    //extern void rt_kprintf(const char *fmt, ...);
    // get device id, then get table,
    //rt_kprintf("Before get Id\n");
    if ((ctx->dev_id == 0) || (ctx->dev_id == 0xff))
        ctx->dev_id = HAL_QSPI_READ_ID(hflash);
    fid = (uint8_t)ctx->dev_id & 0xff;
    mtype = (uint8_t)((ctx->dev_id >> 8) & 0xff);
    did = (uint8_t)((ctx->dev_id >> 16) & 0xff);
    //rt_kprintf("Fid1 0x%x\n",ctx->dev_id);
#if DEBUG_JLINK
    uint8_t hex[16];
    debug_print("fid:");
    debug_print((char *)htoa(hex, fid));
    debug_print(" mtype:");
    debug_print((char *)htoa(hex, mtype));
    debug_print(" did:");
    debug_print((char *)htoa(hex, did));
    debug_print("\r\n");
#endif
    if (hflash->isNand)
        hflash->ctable = spi_nand_get_cmd_by_id(fid, did, mtype);
    else
        hflash->ctable = spi_flash_get_cmd_by_id(fid, did, mtype);

    if (hflash->ctable  == NULL)
    {
        if (hflash->isNand) // for nand, try another timing to read id
        {
            ctx->dev_id = nand_read_id(hflash, 8);
            fid = (uint8_t)ctx->dev_id & 0xff;
            mtype = (uint8_t)((ctx->dev_id >> 8) & 0xff);
            did = (uint8_t)((ctx->dev_id >> 16) & 0xff);
            //rt_kprintf("Fid2 0x%x\n",ctx->dev_id);
            hflash->ctable = spi_nand_get_cmd_by_id(fid, did, mtype);
        }
        if (hflash->ctable  == NULL)
        {
            // disable QSPI
            HAL_FLASH_ENABLE_QSPI(hflash, 0);
            ctx->base_addr = 0;
            ctx->total_size = 0;
            return HAL_ERROR;
        }
    }

    if (hflash->isNand)
        size = spi_nand_get_size_by_id(fid, did, mtype);
    else
        size = spi_flash_get_size_by_id(fid, did, mtype);

    if (size != 0)  // use size from table to replace configure size
    {
        ctx->total_size = size << hflash->dualFlash;    // dual flash size double
        hflash->size = size << hflash->dualFlash; // ?? 2 size ?
    }

    // add software reset to make chip reset to default status (single line ?)
    // set local controller to single line, it only work for bootloader?
    //HAL_QSPIEX_FLASH_RESET(hflash);
    //HAL_Delay_us(300);

    // only nor need set QE mode at initial
    if (hflash->isNand == SPI_MODE_NOR)
    {
        HAL_FLASH_CLR_PROTECT(hflash);
        //HAL_Delay_us(30);
        if (((hflash->size > NOR_FLASH_MAX_3B_SIZE) && (hflash->dualFlash == 0))
                || (hflash->size > NOR_FLASH_MAX_3B_SIZE * 2))
        {
            // enter 4 byte address mode
            HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_EN4BM, 0);

            if (hflash->Mode == HAL_FLASH_NOR_MODE)
            {
                HAL_FLASH_FADDR_SET_QSPI(hflash, false);
            }
            else
            {
                HAL_FLASH_FADDR_SET_QSPI(hflash, true);
            }

            // set 1KB boundary to avoid some large NOR wrap around at 16MB position
            // some 32MB NOR look like 2 16MB connect, it not support continue read.
            HAL_FLASH_SET_ROW_BOUNDARY(hflash, 7);
        }
        else
        {
            if (hflash->Mode == HAL_FLASH_NOR_MODE)
            {
                HAL_FLASH_SET_QUAL_SPI(hflash, false);
            }
            else
            {
                HAL_FLASH_SET_QUAL_SPI(hflash, true);
            }
        }

    }
    else
    {
        uint32_t sta;
        do
        {
            HAL_FLASH_WRITE_DLEN(hflash, 1);
            HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_RDSR, hflash->ctable->status_reg);
            //HAL_FLASH_MANUAL_CMD(hflash, 0, 1, 0, 0, 0, 0, 1, 1);
            //HAL_FLASH_SET_CMD(hflash, 0x0f, 0xc0);
            sta = HAL_FLASH_READ32(hflash);
            HAL_Delay_us(10);
        }
        while (sta & 0x1);  // busy/iop

        nand_clear_status(hflash);
        if (hflash->Mode == HAL_FLASH_NOR_MODE)
        {
            HAL_NAND_EN_QUAL(hflash, 0);
        }
        else
        {
            HAL_NAND_EN_QUAL(hflash, 1);
        }
    }

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_SPI_PSRAM_Init(FLASH_HandleTypeDef *hflash, qspi_configure_t *cfg, uint16_t clk_div)
{
    HAL_StatusTypeDef res;
    uint16_t cs_min;
    uint16_t cs_max;
    uint32_t sys_clk;

    if (hflash == NULL || cfg == NULL)
        return HAL_ERROR;

    HAL_QSPI_Init(hflash, cfg);
    //TODO: delay
    HAL_FLASH_SET_CLK_rom(hflash, clk_div);

    if (FLASH4 == hflash->Instance)
        sys_clk = HAL_RCC_GetSysCLKFreq(CORE_ID_LCPU);
    else
        sys_clk = HAL_RCC_GetSysCLKFreq(CORE_ID_HCPU);
    //sys_clk = HAL_RCC_GetSysCLKFreq(CORE_ID_CURRENT);
    /* 8us */
    cs_max = 8UL * sys_clk / 1000000;
    if (cs_max > 0)
    {
        cs_max -= 1;
    }
    /* 42ns */
    cs_min = 42UL * sys_clk / 1000000000;
    if (cs_min > 0)
    {
        cs_min -= 1;
    }

    res = HAL_FLASH_SET_CS_TIME(hflash, cs_min, cs_max);
    if (res != HAL_OK)
        return res;
    res = HAL_FLASH_SET_ROW_BOUNDARY(hflash, 6);
    if (res != HAL_OK)
        return res;

    // enable QSPI
    HAL_FLASH_ENABLE_QSPI(hflash, 1);
#if 0
    /* enable quadline read */
    HAL_FLASH_CFG_AHB_RCMD(hflash, 3, 6, 0, 0, 2, 3, 1);
    res = HAL_FLASH_SET_AHB_RCMD(hflash, BF0_QPSRAM_QRD);
    if (res != HAL_OK)
        return res;

    /* enable quadline write */
    HAL_FLASH_CFG_AHB_WCMD(hflash, 3, 0, 0, 0, 2, 3, 1);
    res = HAL_FLASH_SET_AHB_WCMD(hflash, BF0_QPSRAM_QWR);
    if (res != HAL_OK)
        return res;
#endif
    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_FLASH_DeInit(FLASH_HandleTypeDef *hflash)
{
    HAL_FLASH_ENABLE_QSPI(hflash, 0);
    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_FLASH_DMA_START(FLASH_HandleTypeDef *hflash, char *buf, int write, uint32_t len)
{
    uint32_t src, dst, value, size;
    HAL_StatusTypeDef res = HAL_OK;
    if ((hflash == NULL) || (hflash->dma == NULL) || (len == 0))
        return HAL_ERROR;

    value = hflash->Instance->CR;

    if (write)
    {
        hflash->dma->Init.Direction           = DMA_MEMORY_TO_PERIPH;
        hflash->dma->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hflash->dma->Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        //nand_dma_handle.Init.PeriphInc           = DMA_PINC_DISABLE;
        //nand_dma_handle.Init.MemInc              = DMA_MINC_ENABLE;
        src = (uint32_t) buf;
        dst = (uint32_t)(&hflash->Instance->DR);
        size = len; //(len + 3) / 4; // must up aligned
    }
    else // read, fifo word mode, length should change to word lenght(/4)
    {
        hflash->dma->Init.Direction           = DMA_PERIPH_TO_MEMORY;
        hflash->dma->Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD; //DMA_PDATAALIGN_BYTE; //DMA_PDATAALIGN_WORD;
        hflash->dma->Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;  //DMA_MDATAALIGN_BYTE; //DMA_MDATAALIGN_WORD;
        //nand_dma_handle.Init.PeriphInc           = DMA_PINC_DISABLE;
        //nand_dma_handle.Init.MemInc              = DMA_MINC_ENABLE;
        dst = (uint32_t) buf;
        src = (uint32_t)(&hflash->Instance->DR);
        size = (len + 3) / 4; // must up aligned
    }

    res = HAL_DMA_DeInit(hflash->dma);
    if (res != HAL_OK)
        return res;
    res = HAL_DMA_Init(hflash->dma);
    if (res != HAL_OK)
        return res;

    //value |= FLASHC_CR_DMAE;
    hflash->Instance->CR = value | QSPI_CR_DMAE;
    //hflash->Instance->DLR = (len - 1) & FLASHC_DLR_DLEN_Msk;
    hflash->Instance->DLR1 = (len - 1);

    res = HAL_DMA_Start(hflash->dma, src, dst, size);
    //res = HAL_DMA_PollForTransfer(hflash->dma, HAL_DMA_FULL_TRANSFER, 1000);

    // recover dmae bit
    //hflash->Instance->CR = value;

    return res;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_FLASH_DMA_WAIT_DONE(FLASH_HandleTypeDef *hflash, uint32_t timeout)
{
    HAL_StatusTypeDef res = HAL_OK;
    uint32_t value;
    if ((hflash == NULL) || (hflash->dma == NULL))
        return HAL_ERROR;

    res = HAL_DMA_PollForTransfer(hflash->dma, HAL_DMA_FULL_TRANSFER, timeout);

    // clear dmae bit
    value = hflash->Instance->CR;
    hflash->Instance->CR = value & (~QSPI_CR_DMAE);

    return res;
}

__HAL_ROM_USED void HAL_FLASH_ALIAS_CFG(FLASH_HandleTypeDef *fhandle, uint32_t start, uint32_t len, uint32_t offset)
{
    if (fhandle == NULL)
        return;

    HAL_FLASH_SET_ALIAS_RANGE(fhandle, start, len);

    HAL_FLASH_SET_ALIAS_OFFSET(fhandle, offset);
}


__HAL_ROM_USED void HAL_FLASH_NONCE_CFG(FLASH_HandleTypeDef *fhandle, uint32_t start, uint32_t end, uint8_t *nonce)
{
    if (fhandle == NULL || nonce == NULL)
        return;
    HAL_FLASH_SET_NONCE(fhandle, nonce);
    HAL_FLASH_SET_CTR(fhandle, start, end);
}


__HAL_ROM_USED void HAL_FLASH_AES_CFG(FLASH_HandleTypeDef *fhandle, uint8_t aes256)
{
    if (fhandle == NULL)
        return;

    if (aes256 != 0)
        HAL_FLASH_SET_AES(fhandle, HAL_FLASH_AES256);
    else
        HAL_FLASH_SET_AES(fhandle, HAL_FLASH_AES128);

    HAL_FLASH_ENABLE_AES(fhandle, 1);
}


static int flash_handle_valid(FLASH_HandleTypeDef *handle)
{
    if ((handle == NULL) || (handle->ctable == NULL))
        return 0;

    return 1;
}

static int flash_cmd_valid(FLASH_HandleTypeDef *handle, SPI_FLASH_CMD_E cmd)
{
    if ((handle->ctable == NULL) || (cmd >= SPI_FLASH_CMD_COUNT)
            || (handle->ctable->cmd_cfg[cmd].cmd == 0))
    {
        return 0;
    }

    return 1;
}

/***************** Interface for nand flash ********************************/

// all nand read id should be same, or cannot recognize them
__HAL_ROM_USED int nand_read_id(FLASH_HandleTypeDef *handle, uint8_t dummy)
{
    uint8_t mid, did1, did2;
    uint32_t rdata;

    if (handle == NULL)
        return FLASH_UNKNOW_ID;

    //HAL_FLASH_CLEAR_FIFO(handle, HAL_FLASH_CLR_RX_FIFO);

    // configure CCR
//#ifdef BSP_NAND_ID_DUMMY
    // for some manufactury, get id need 8 dmmy cycle, like winbond
    HAL_FLASH_MANUAL_CMD(handle, 0, 1, dummy, 0, 0, 0, 0, 1);
//#else
    // some do not need dummy, like giga device
    //HAL_FLASH_MANUAL_CMD(handle, 0, 1, 0, 0, 0, 0, 0, 1);
//#endif
    // configure data length
    HAL_FLASH_WRITE_DLEN(handle, 3);

    // send command, read id command as 0x9f, all device with same read ID?
    HAL_FLASH_SET_CMD(handle, 0x9f, 0);

    rdata = HAL_FLASH_READ32(handle);
    mid  = rdata & 0xff;
    did1 = (rdata >> 8) & 0xff;
    did2 = (rdata >> 16) & 0xff;
    //LOG_D("Nand ID: 0x%x, 0x%x, 0x%x\n", mid, did1, did2);

    return rdata;
}

//extern void rt_kprintf(const char *fmt, ...);
__HAL_ROM_USED int HAL_NAND_CONF_ECC(FLASH_HandleTypeDef *handle, uint8_t en)
{
    uint32_t sta;
    if (handle == NULL || handle->ctable == NULL)
        return -1;

    if (handle->ctable->mode_reg == 0 || handle->ctable->ecc_en_mask == 0)
        return -2;

    HAL_FLASH_WRITE_DLEN(handle, 1);
    HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_RDSR, handle->ctable->mode_reg);
    sta = HAL_FLASH_READ32(handle);
    //rt_kprintf("Sta = 0x%x\n",sta);

    //HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_WREN, 0);
    //HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_WVSR, 0);
    //sta |= 8; // set to buf mode
    if (en)
        sta |= handle->ctable->ecc_en_mask; // SET ECC-E
    else
        sta &= ~handle->ctable->ecc_en_mask; // clear ecc bit
    HAL_FLASH_WRITE_WORD(handle, sta);
    HAL_FLASH_WRITE_DLEN(handle, 1);
    HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_WRSR, handle->ctable->mode_reg);

    handle->ecc_en = en;

    HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_RDSR, handle->ctable->mode_reg);
    sta = HAL_FLASH_READ32(handle);
    //rt_kprintf("Sta2 = 0x%x\n",sta);

    return 0;
}

__HAL_ROM_USED int HAL_NAND_GET_ECC_STATUS(FLASH_HandleTypeDef *handle)
{
    uint32_t sta;

    if (handle == NULL || handle->ctable == NULL)
        return 0;

    if (handle->ctable->status_reg == 0 || handle->ctable->ecc_sta_mask == 0)
        return 0;

    HAL_FLASH_WRITE_DLEN(handle, 1);
    HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_RDSR, handle->ctable->status_reg);
    sta = HAL_FLASH_READ32(handle);

    return (sta & handle->ctable->ecc_sta_mask);
}

__HAL_ROM_USED int HAL_NAND_GET_ECC_RESULT(FLASH_HandleTypeDef *handle)
{
    int sta;
    int bitc, bits, i;

    if (handle->ecc_en == 0)
        return 0;

    sta = HAL_NAND_GET_ECC_STATUS(handle);
    if (sta == 0)
        return 0;

    //if(sta != 0)
    //    return sta;

    handle->ErrorCode |= 0X8000;
    bits = 0;   // ecc status start bit
    //while((handle->ctable->ecc_sta_mask & (1<<bits)) == 0)
    //    bits++;
    for (i = 0; i < 8; i++)
        if ((handle->ctable->ecc_sta_mask & (1 << i)) != 0)
        {
            bits = i;
            break;
        }

    bitc = 0; // ecc status bit count
    for (i = bits; i < 8; i++)
        if ((handle->ctable->ecc_sta_mask & (1 << i)) != 0)
        {
            bitc++;
        }

    // for 2 bit mode, only 0 and 1 correct, for 3 bit mode, only 7 can not correct
    if (bitc == 2)
        if ((sta >> bits) < 2)
            return 0;
        else
            return sta >> bits ;
    else if (bitc == 3)
        if ((sta >> bits) < 7)
            return 0;
        else
            return sta >> bits ;
    else // not support yet
        return 0xfc;

    //return (sta & handle->ctable->ecc_sta_mask)>>bits;
    return 0xfd;
}

// NAND operations ---------------------------------
__HAL_ROM_USED int HAL_NAND_READ_PAGE(FLASH_HandleTypeDef *handle, uint32_t addr, uint8_t *buff, uint32_t len)
{
    return HAL_NAND_READ_WITHOOB(handle, addr, buff, len, NULL, 0);
}

__HAL_ROM_USED int HAL_NAND_READ_WITHOOB(FLASH_HandleTypeDef *handle, uint32_t addr,
        uint8_t *dbuff, uint32_t dlen, uint8_t *oob_buf, uint32_t olen)
{
    int busy;
    uint32_t offset = addr & (SPI_NAND_PAGE_SIZE - 1);

    if (handle == NULL || handle->ctable == NULL || handle->data_buf == NULL
            || (dlen + offset) > SPI_NAND_PAGE_SIZE || olen > SPI_NAND_MAXOOB_SIZE)
    {
        handle->ErrorCode = 1;
        return 0;
    }

    handle->ErrorCode = 0;
    // switch addr to local addr
    if (addr >= handle->base)
        addr -= handle->base;

    // load page to cache, page read with block+page address
    HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_PREAD, (addr / SPI_NAND_PAGE_SIZE) & 0xffff);

    // check busy
    do
    {
        HAL_FLASH_WRITE_DLEN(handle, 1);
        HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_RDSR, handle->ctable->status_reg);
        busy = HAL_FLASH_READ32(handle) & 0x1;
    }
    while (busy);
    int res = HAL_NAND_GET_ECC_RESULT(handle);
    if (res != 0)
    {
        handle->ErrorCode = res | 0x8000;
        return 0;
    }

    // use AHB read
    {
        // set ahb read command
        if (handle->Mode == HAL_FLASH_NOR_MODE)
        {
            HAL_FLASH_SET_AHB_RCMD(handle, handle->ctable->cmd_cfg[SPI_FLASH_CMD_FREAD].cmd);
            HAL_FLASH_CFG_AHB_RCMD(handle, handle->ctable->cmd_cfg[SPI_FLASH_CMD_FREAD].data_mode,
                                   handle->ctable->cmd_cfg[SPI_FLASH_CMD_FREAD].dummy_cycle, handle->ctable->cmd_cfg[SPI_FLASH_CMD_FREAD].ab_size,
                                   handle->ctable->cmd_cfg[SPI_FLASH_CMD_FREAD].ab_mode, handle->ctable->cmd_cfg[SPI_FLASH_CMD_FREAD].addr_size,
                                   handle->ctable->cmd_cfg[SPI_FLASH_CMD_FREAD].addr_mode, handle->ctable->cmd_cfg[SPI_FLASH_CMD_FREAD].ins_mode);
        }
        else
        {
#if NAND_DUAL_REPLACE_QUAL
            HAL_FLASH_SET_AHB_RCMD(handle, handle->ctable->cmd_cfg[SPI_FLASH_CMD_2READ].cmd);
            HAL_FLASH_CFG_AHB_RCMD(handle,  handle->ctable->cmd_cfg[SPI_FLASH_CMD_2READ].data_mode,
                                   handle->ctable->cmd_cfg[SPI_FLASH_CMD_2READ].dummy_cycle, handle->ctable->cmd_cfg[SPI_FLASH_CMD_2READ].ab_size,
                                   handle->ctable->cmd_cfg[SPI_FLASH_CMD_2READ].ab_mode, handle->ctable->cmd_cfg[SPI_FLASH_CMD_2READ].addr_size,
                                   handle->ctable->cmd_cfg[SPI_FLASH_CMD_2READ].addr_mode, handle->ctable->cmd_cfg[SPI_FLASH_CMD_2READ].ins_mode);
#else

            HAL_FLASH_SET_AHB_RCMD(handle, handle->ctable->cmd_cfg[SPI_FLASH_CMD_4READ].cmd);
            HAL_FLASH_CFG_AHB_RCMD(handle,  handle->ctable->cmd_cfg[SPI_FLASH_CMD_4READ].data_mode,
                                   handle->ctable->cmd_cfg[SPI_FLASH_CMD_4READ].dummy_cycle, handle->ctable->cmd_cfg[SPI_FLASH_CMD_4READ].ab_size,
                                   handle->ctable->cmd_cfg[SPI_FLASH_CMD_4READ].ab_mode, handle->ctable->cmd_cfg[SPI_FLASH_CMD_4READ].addr_size,
                                   handle->ctable->cmd_cfg[SPI_FLASH_CMD_4READ].addr_mode, handle->ctable->cmd_cfg[SPI_FLASH_CMD_4READ].ins_mode);
#endif
        }
        //for (i = 0; i < SPI_NAND_PAGE_SIZE+olen / 4; i++)
        //    temp32[i] = *nbuf32++;
        if (handle->data_buf && false) /* no need to copy twice */
        {
            memcpy(handle->data_buf, (const void *)handle->base, SPI_NAND_PAGE_SIZE + handle->ctable->oob_size);
            if ((dbuff != NULL) && (handle->data_buf != dbuff))
                memcpy(dbuff, (const void *)handle->data_buf, dlen);
            if ((oob_buf != NULL) && (oob_buf != (handle->data_buf + SPI_NAND_PAGE_SIZE)))
                memcpy(oob_buf, (const void *)(handle->data_buf + SPI_NAND_PAGE_SIZE), olen);
        }
        else
        {
            if (dbuff != NULL)
                memcpy(dbuff, (const void *)(handle->base + offset), dlen);
            if (oob_buf != NULL)
                memcpy(oob_buf, (const void *)(handle->base + SPI_NAND_PAGE_SIZE), olen);
        }
    }

    return dlen + olen;
}


#define QSPI_FIFO_SIZE      (64)
__HAL_ROM_USED int HAL_NAND_WRITE_PAGE(FLASH_HandleTypeDef *handle, uint32_t addr, const uint8_t *buff, uint32_t len)
{
    return HAL_NAND_WRITE_WITHOOB(handle, addr, buff, len, NULL, 0);
}

__HAL_ROM_USED int HAL_NAND_WRITE_WITHOOB(FLASH_HandleTypeDef *handle, uint32_t addr,
        const uint8_t *buff, uint32_t len, const uint8_t *oob_buf, uint32_t olen)
{
    int busy;
    uint32_t *tbuf = NULL; // = (uint32_t *)buff;
    int res;
    HAL_StatusTypeDef ret;
    int row_addr = 0;

    if (handle == NULL || handle->ctable == NULL || handle->data_buf == NULL)
    {
        handle->ErrorCode = 1;
        return 0;
    }
    if ((buff == NULL) && (oob_buf == NULL))
    {
        handle->ErrorCode = 2;
        return 0;
    }
    if ((len > SPI_NAND_PAGE_SIZE) || (olen > SPI_NAND_MAXOOB_SIZE))
    {
        handle->ErrorCode = 3;
        return 0;
    }

    handle->ErrorCode = 0;

    // if (handle->ecc_en && (olen > 0)) // when ecc en, user can not write oob , it should fill by flash chip
    //     return 0;

    if (addr >= handle->base)
    {
        addr -= handle->base;
    }
    if (oob_buf == NULL || olen == 0)
    {
        tbuf = (uint32_t *)buff;
    }
    else
    {
        // copy buffer data and oob data to internal buffer
        memcpy(handle->data_buf + SPI_NAND_PAGE_SIZE, oob_buf, olen);
        if (buff != NULL)
        {
            memcpy(handle->data_buf, buff, len);
            tbuf = (uint32_t *)handle->data_buf;
            len = SPI_NAND_PAGE_SIZE + olen;
        }
        else
        {
            row_addr = SPI_NAND_PAGE_SIZE;
            tbuf = (uint32_t *)(handle->data_buf + SPI_NAND_PAGE_SIZE);
            len = olen;
        }
        // if olen > 0, data buffer should fill full page
    }

    // address include flash start address, there only use offset
    //addr &= (FLASH_SUPPORT_MAX_SIZE - 1);

    if (handle->dma != NULL) // dma enable
    {
        ret = HAL_FLASH_DMA_START(handle, (char *)tbuf, 1, len);
        if (ret != HAL_OK)
        {
            handle->ErrorCode = 4;
            return 0;
        }
        HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_WREN, 0);
        if (handle->Mode == HAL_FLASH_NOR_MODE)
            HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_PLD, row_addr);
        else
            HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_QPLD, row_addr);

        ret = HAL_FLASH_DMA_WAIT_DONE(handle, 1000);
        if (ret != HAL_OK)
        {
            handle->ErrorCode = 5;
            return 0;
        }
    }
    else
    {
        //return 0;
        int i, cnt;
        int remain = len;
        int fill = remain > QSPI_FIFO_SIZE ? QSPI_FIFO_SIZE : remain;
        cnt = 0;

        // write enable
        res = HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_WREN, 0);
        // controller fifo 64 bytes, for page write, need 32 times to cache
        // first 64 bytes
        // write data to fifo
        for (i = 0; i < fill / 4; i++)
        {
            HAL_FLASH_WRITE_WORD(handle, *tbuf++);
        }

        // for first write in one page, use load program data
        HAL_FLASH_WRITE_DLEN(handle, fill);
        if (handle->Mode == HAL_FLASH_NOR_MODE)
            res = HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_PLD, row_addr);
        else
            res = HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_QPLD, row_addr);

        remain -= fill;
        cnt += fill;

        while (remain > 0)
        {
            fill = remain > QSPI_FIFO_SIZE ? QSPI_FIFO_SIZE : remain;

            // write enable
            res = HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_WREN, 0);
            // write data to fifo
            for (i = 0; i < fill / 4; i++)
            {
                HAL_FLASH_WRITE_WORD(handle, *tbuf++);
            }

            // use random load program data
            HAL_FLASH_WRITE_DLEN(handle, fill);
            if (handle->Mode == HAL_FLASH_NOR_MODE)
                res = HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_PLDR, cnt);
            else
                res = HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_QPLDR, cnt);

            remain -= fill;
            cnt += fill;
        }
    }
    // write cache data to physical memory
    // write enable
    res = HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_WREN, 0);

    // program execute, use page address
    res = HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_PEXE, (addr / SPI_NAND_PAGE_SIZE) & 0xffff);

    // check busy, wait until write done
    do
    {
        HAL_FLASH_WRITE_DLEN(handle, 1);
        res = HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_RDSR, handle->ctable->status_reg);
        busy = HAL_FLASH_READ32(handle);
    }
    while (busy & 0x1);

    if (busy & NAND_P_FAIL_BIT)
    {
        handle->ErrorCode = NAND_P_FAIL_BIT;
        return -1;
    }

    return len;
}

__HAL_ROM_USED int HAL_NAND_ERASE_BLK(FLASH_HandleTypeDef *handle, uint32_t addr)
{
    int busy;
    //int8_t id = Addr2Id(addr);
    if (handle == NULL || handle->ctable == NULL)
    {
        //LOG_E("HAL_NAND_ERASE_BLK error param\n");
        handle->ErrorCode = 1;
        return -1;
    }
    handle->ErrorCode = 0;
    // address include flash start address, there only use offset
    //addr &= (FLASH_SUPPORT_MAX_SIZE - 1);
    if (addr >= handle->base)
        addr -= handle->base;

    // write enable
    HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_WREN, 0);

    // block erase, 16 bit page address, but only high 10 bit block address should work
    HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_BE, (addr >> 11) & 0xffff);

    // check busy
    do
    {
        HAL_FLASH_WRITE_DLEN(handle, 1);
        HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_RDSR, handle->ctable->status_reg);
        busy = HAL_FLASH_READ32(handle);
    }
    while (busy & 0x1);

    if (busy & NAND_E_FAIL_BIT)
    {
        //rt_kprintf("NAND status reg 0x%x : 0x%x\n", handle->ctable->status_reg, busy);
        handle->ErrorCode = NAND_E_FAIL_BIT;
        return -2;
    }

    return 0;
}

__HAL_ROM_USED int HAL_NAND_CONF_BUF(FLASH_HandleTypeDef *handle, uint8_t en)
{
    uint32_t sta;
    if (handle == NULL || handle->ctable == NULL)
        return -1;

    if (handle->ctable->mode_reg == 0 || handle->ctable->buf_mod_mask == 0)
        return -2;

    HAL_FLASH_WRITE_DLEN(handle, 1);
    HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_RDSR, handle->ctable->mode_reg);
    sta = HAL_FLASH_READ32(handle);
    //rt_kprintf("Sta = 0x%x\n",sta);

    //HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_WREN, 0);
    //HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_WVSR, 0);

    if (en)
        sta |= handle->ctable->buf_mod_mask; // set to buf mode
    else
        sta &= ~handle->ctable->buf_mod_mask; // clear buf mode

    HAL_FLASH_WRITE_WORD(handle, sta);
    HAL_FLASH_WRITE_DLEN(handle, 1);
    HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_WRSR, handle->ctable->mode_reg);

    handle->buf_mode = en;

    //HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_RDSR, handle->ctable->mode_reg);
    //sta = HAL_FLASH_READ32(handle);
    //rt_kprintf("Sta2 = 0x%x\n",sta);

    return 0;
}

__HAL_ROM_USED int HAL_NAND_SET_CONTINUE(FLASH_HandleTypeDef *handle, uint32_t addr, uint8_t en, uint32_t data_len)
{
    if ((handle == NULL) || (handle->ctable == NULL) || (handle->ctable->buf_mod_mask == 0))    // NOT SUPPORT MODE SWITCH
        return HAL_ERROR;

    // switch addr to local addr
    if (addr >= handle->base)
        addr -= handle->base;

    if (en) // enable continue mode, set buf = 0, load page as addr, set ahb as continue timing
    {
        // 1. load page to cache, page read with block+page address
        HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_PREAD, (addr / SPI_NAND_PAGE_SIZE) & 0xffff);
        // check busy
        uint32_t busy = 1;
        do
        {
            HAL_FLASH_WRITE_DLEN(handle, 1);
            HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_RDSR, handle->ctable->status_reg);
            busy = HAL_FLASH_READ32(handle) & 0x1;
        }
        while (busy);

        // 2. set buf bit, continue = !buf
        HAL_NAND_CONF_BUF(handle, 0);

        // 3. enable Hardware Interface
        __HAL_QSPI_EN_HWI(handle);

        // 4. configure read command
        if (handle->Mode != 0)
        {
            if (handle->ctable->cmd_cfg[SPI_FLASH_CMD_C4READ].cmd == 0)
                return HAL_ERROR;
#if 1
            HAL_FLASH_MANUAL_CMD(handle, handle->ctable->cmd_cfg[SPI_FLASH_CMD_C4READ].func_mode, handle->ctable->cmd_cfg[SPI_FLASH_CMD_C4READ].data_mode,
                                 handle->ctable->cmd_cfg[SPI_FLASH_CMD_C4READ].dummy_cycle, handle->ctable->cmd_cfg[SPI_FLASH_CMD_C4READ].ab_size,
                                 handle->ctable->cmd_cfg[SPI_FLASH_CMD_C4READ].ab_mode, handle->ctable->cmd_cfg[SPI_FLASH_CMD_C4READ].addr_size,
                                 handle->ctable->cmd_cfg[SPI_FLASH_CMD_C4READ].addr_mode, handle->ctable->cmd_cfg[SPI_FLASH_CMD_C4READ].ins_mode);
            __HAL_QSPI_SET_CMD(handle, handle->ctable->cmd_cfg[SPI_FLASH_CMD_C4READ].cmd);
#else

            HAL_FLASH_CFG_AHB_RCMD(handle, handle->ctable->cmd_cfg[SPI_FLASH_CMD_C4READ].data_mode,
                                   handle->ctable->cmd_cfg[SPI_FLASH_CMD_C4READ].dummy_cycle, handle->ctable->cmd_cfg[SPI_FLASH_CMD_C4READ].ab_size,
                                   handle->ctable->cmd_cfg[SPI_FLASH_CMD_C4READ].ab_mode, handle->ctable->cmd_cfg[SPI_FLASH_CMD_C4READ].addr_size,
                                   handle->ctable->cmd_cfg[SPI_FLASH_CMD_C4READ].addr_mode, 1);
            HAL_FLASH_SET_AHB_RCMD(handle, handle->ctable->cmd_cfg[SPI_FLASH_CMD_C4READ].cmd);
#endif
        }
        else
        {
            if (handle->ctable->cmd_cfg[SPI_FLASH_CMD_CFREAD].cmd == 0)
                return HAL_ERROR;
#if 1
            HAL_FLASH_MANUAL_CMD(handle, handle->ctable->cmd_cfg[SPI_FLASH_CMD_CFREAD].func_mode, handle->ctable->cmd_cfg[SPI_FLASH_CMD_CFREAD].data_mode,
                                 handle->ctable->cmd_cfg[SPI_FLASH_CMD_CFREAD].dummy_cycle, handle->ctable->cmd_cfg[SPI_FLASH_CMD_CFREAD].ab_size,
                                 handle->ctable->cmd_cfg[SPI_FLASH_CMD_CFREAD].ab_mode, handle->ctable->cmd_cfg[SPI_FLASH_CMD_CFREAD].addr_size,
                                 handle->ctable->cmd_cfg[SPI_FLASH_CMD_CFREAD].addr_mode, handle->ctable->cmd_cfg[SPI_FLASH_CMD_CFREAD].ins_mode);
            __HAL_QSPI_SET_CMD(handle, handle->ctable->cmd_cfg[SPI_FLASH_CMD_CFREAD].cmd);
#else

            HAL_FLASH_CFG_AHB_RCMD(handle, handle->ctable->cmd_cfg[SPI_FLASH_CMD_CFREAD].data_mode,
                                   handle->ctable->cmd_cfg[SPI_FLASH_CMD_CFREAD].dummy_cycle, handle->ctable->cmd_cfg[SPI_FLASH_CMD_CFREAD].ab_size,
                                   handle->ctable->cmd_cfg[SPI_FLASH_CMD_CFREAD].ab_mode, handle->ctable->cmd_cfg[SPI_FLASH_CMD_CFREAD].addr_size,
                                   handle->ctable->cmd_cfg[SPI_FLASH_CMD_CFREAD].addr_mode, 1);
            HAL_FLASH_SET_AHB_RCMD(handle, handle->ctable->cmd_cfg[SPI_FLASH_CMD_CFREAD].cmd);
#endif
        }

    }
    else // set buf mode, set AHB read
    {
        __HAL_QSPI_DIS_HWI(handle);
        // clear TCF before next mpi command
        HAL_FLASH_CLR_CMD_DONE(handle);
        //HAL_FLASH_CONFIG_AHB_READ(handle, handle->Mode);
        HAL_NAND_CONF_BUF(handle, 1);
    }

    return HAL_OK;
}

__HAL_ROM_USED int HAL_NAND_AHB_CONTINUE(FLASH_HandleTypeDef *handle, uint32_t addr, uint8_t en)
{
    int res = 0;
    if ((handle == NULL) || (handle->ctable == NULL) || (handle->ctable->buf_mod_mask == 0))    // NOT SUPPORT MODE SWITCH
        return HAL_ERROR;

    // switch addr to local addr
    if (addr >= handle->base)
        addr -= handle->base;

    if (en) // enable continue mode, set buf = 0, load page as addr, set ahb as continue timing
    {
        // 1. load page to cache, page read with block+page address
        HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_PREAD, (addr / SPI_NAND_PAGE_SIZE) & 0xffff);
        // check busy
        uint32_t busy = 1;
        do
        {
            HAL_FLASH_WRITE_DLEN(handle, 1);
            HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_RDSR, handle->ctable->status_reg);
            busy = HAL_FLASH_READ32(handle) & 0x1;
        }
        while (busy);

        // 2. set buf bit, continue = !buf
        res = HAL_NAND_CONF_BUF(handle, 0);
        HAL_ASSERT(res == 0);

        // 3. configure read command
        if (handle->Mode != 0)
        {
            if (handle->ctable->cmd_cfg[SPI_FLASH_CMD_C4READ].cmd == 0)
                return HAL_ERROR;

            HAL_FLASH_CFG_AHB_RCMD(handle, handle->ctable->cmd_cfg[SPI_FLASH_CMD_C4READ].data_mode,
                                   handle->ctable->cmd_cfg[SPI_FLASH_CMD_C4READ].dummy_cycle, handle->ctable->cmd_cfg[SPI_FLASH_CMD_C4READ].ab_size,
                                   handle->ctable->cmd_cfg[SPI_FLASH_CMD_C4READ].ab_mode, handle->ctable->cmd_cfg[SPI_FLASH_CMD_C4READ].addr_size,
                                   handle->ctable->cmd_cfg[SPI_FLASH_CMD_C4READ].addr_mode, 1);
            HAL_FLASH_SET_AHB_RCMD(handle, handle->ctable->cmd_cfg[SPI_FLASH_CMD_C4READ].cmd);
        }
        else
        {
            if (handle->ctable->cmd_cfg[SPI_FLASH_CMD_CFREAD].cmd == 0)
                return HAL_ERROR;

            HAL_FLASH_CFG_AHB_RCMD(handle, handle->ctable->cmd_cfg[SPI_FLASH_CMD_CFREAD].data_mode,
                                   handle->ctable->cmd_cfg[SPI_FLASH_CMD_CFREAD].dummy_cycle, handle->ctable->cmd_cfg[SPI_FLASH_CMD_CFREAD].ab_size,
                                   handle->ctable->cmd_cfg[SPI_FLASH_CMD_CFREAD].ab_mode, handle->ctable->cmd_cfg[SPI_FLASH_CMD_CFREAD].addr_size,
                                   handle->ctable->cmd_cfg[SPI_FLASH_CMD_CFREAD].addr_mode, 1);
            HAL_FLASH_SET_AHB_RCMD(handle, handle->ctable->cmd_cfg[SPI_FLASH_CMD_CFREAD].cmd);
        }

    }
    else // set buf mode, set AHB read
    {
        //__HAL_QSPI_DIS_HWI(handle);
        //HAL_FLASH_CONFIG_AHB_READ(handle, handle->Mode);
        //handle->Instance->TIMR = 0xFF;
        HAL_NAND_CONF_BUF(handle, 1);
    }

    return HAL_OK;
}

__HAL_ROM_USED int HAL_NAND_MARK_BADBLK(FLASH_HandleTypeDef *handle, uint32_t blk, uint8_t bad)
{
    uint32_t addr;
    int res;
    uint8_t tbuf[4];

    if ((handle == NULL) || (handle->data_buf == NULL))
        return HAL_ERROR;

    memset(tbuf, 0xff, 4);
    // block to addr, check first page
    addr = blk << 17;
    //addr += handle->base;

    if (bad)
    {
        tbuf[0] = 0;
        tbuf[1] = 0xbd;
    }
    else    // it can not be recover except erase, bits only 1 -> 0
    {
        tbuf[0] = 0xff;
        tbuf[1] = 0xff;
        return HAL_ERROR;
    }
    res = HAL_NAND_WRITE_WITHOOB(handle, addr, NULL, 0, tbuf, 4);
    if (res <= 0)
        return HAL_ERROR;

    return HAL_OK;
}

__HAL_ROM_USED int HAL_NAND_GET_BADBLK(FLASH_HandleTypeDef *handle, uint32_t blk)
{
    uint32_t addr;
    uint8_t tbuf[4];

    if ((handle == NULL) || (handle->data_buf == NULL))
        return 0;

    // block to addr, check first page
    addr = blk << 17;
    //addr += handle->base;

    int res = HAL_NAND_READ_WITHOOB(handle, addr, NULL, 0, tbuf, 4);
    if (res == 0)
        return 0;

    if (tbuf[0] == 0xff)
        return 0;

    return 1;
}

__HAL_ROM_USED int HAL_NAND_EN_QUAL(FLASH_HandleTypeDef *handle, uint8_t en)
{
    uint32_t sta;
    if (handle == NULL || handle->ctable == NULL)
        return -1;

    if (handle->ctable->mode_reg == 0 || handle->ctable->qe_mod_mask == 0)
        return 0;

#if NAND_DUAL_REPLACE_QUAL
    return 0;
#endif
    HAL_FLASH_WRITE_DLEN(handle, 1);
    HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_RDSR, handle->ctable->mode_reg);
    sta = HAL_FLASH_READ32(handle);

    //HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_WVSR, 0);

    if (en)
        sta |= handle->ctable->qe_mod_mask; // SET QE bit
    else
        sta &= ~handle->ctable->qe_mod_mask; // clear QE bit

    HAL_FLASH_WRITE_WORD(handle, sta);
    HAL_FLASH_WRITE_DLEN(handle, 1);
    HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_WRSR, handle->ctable->mode_reg);

    return 0;
}


__HAL_ROM_USED int nand_clear_status(FLASH_HandleTypeDef *handle)
{
    uint32_t status;

    status = 0;
    HAL_FLASH_WRITE_DLEN(handle, 1);
    HAL_FLASH_WRITE_WORD(handle, 0);
    HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_WRSR, handle->ctable->protect_reg);

    return status;
}

__HAL_ROM_USED int nand_get_status(FLASH_HandleTypeDef *handle)
{
    uint32_t status, sta;

    status = 0;
    HAL_FLASH_WRITE_DLEN(handle, 1);
    HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_RDSR, handle->ctable->protect_reg);
    sta = HAL_FLASH_READ32(handle);
    status = (sta & 0xff);
    HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_RDSR, handle->ctable->mode_reg);
    sta = HAL_FLASH_READ32(handle);
    status = (status << 8) | (sta & 0xff);
    HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_RDSR, handle->ctable->status_reg);
    sta = HAL_FLASH_READ32(handle);
    status = (status << 8) | (sta & 0xff);
    return status;
}

/************************************************
* Check some chip, all otp bit in B0 register is same, so defined to fix bit
* Include: winbond, giga, ziguang, dosilicon, hyf
* If they changed in some new chip, need add a new member for it in SPI_FLASH_FACT_CFG_T
* Note: OTP can only write once even not lock, bits can noly 1 to 0, erase not support !!!
**************************************************/
#define NAND_OTP_EN_BIT     (1<<6)
#define NAND_OTP_LOCK_BIT   (1<<7)
__HAL_ROM_USED int HAL_NAND_SWITCH_OTP(FLASH_HandleTypeDef *handle, uint8_t otp)
{
    uint32_t sta;

    HAL_FLASH_WRITE_DLEN(handle, 1);
    HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_RDSR, handle->ctable->mode_reg);
    sta = HAL_FLASH_READ32(handle);

    if (otp)
    {
        if ((sta & NAND_OTP_EN_BIT) != 0) // otp enable has been set
        {
            return 0;
        }
        else
        {
            sta &= 0xff;
            sta |= NAND_OTP_EN_BIT;
        }
    }
    else
    {
        if ((sta & NAND_OTP_EN_BIT) == 0) // otp enable has been disable
        {
            return 0;
        }
        else
        {
            sta &= 0xff;
            sta &= ~NAND_OTP_EN_BIT;
        }
    }
    HAL_FLASH_WRITE_DLEN(handle, 1);
    HAL_FLASH_WRITE_WORD(handle, sta);
    HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_WRSR, handle->ctable->mode_reg);

    return 0;
}

__HAL_ROM_USED int HAL_NAND_LOCK_OTP(FLASH_HandleTypeDef *handle)
{
    uint32_t busy, sta;

    HAL_FLASH_WRITE_DLEN(handle, 1);
    HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_RDSR, handle->ctable->mode_reg);
    sta = HAL_FLASH_READ32(handle);

    if ((sta & NAND_OTP_LOCK_BIT) != 0) // otp enable has been locked
    {
        return 0;
    }
    else
    {
        sta &= 0xff;
        sta |= (NAND_OTP_EN_BIT | NAND_OTP_LOCK_BIT);
    }

    HAL_FLASH_WRITE_DLEN(handle, 1);
    HAL_FLASH_WRITE_WORD(handle, sta);
    HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_WRSR, handle->ctable->mode_reg);

    // begin lock process
    int res = HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_WREN, 0);

    // program execute, use page address
    res = HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_PEXE, 0);

    // check busy, wait until program done
    HAL_FLASH_WRITE_DLEN(handle, 1);
    do
    {
        res = HAL_FLASH_ISSUE_CMD(handle, SPI_FLASH_CMD_RDSR, handle->ctable->status_reg);
        busy = HAL_FLASH_READ32(handle);
    }
    while (busy & 0x1);

    return 0;
}

__HAL_ROM_USED uint32_t HAL_NAND_PAGE_SIZE(FLASH_HandleTypeDef *handle)
{
    return 2048;
}

__HAL_ROM_USED uint32_t HAL_NAND_BLOCK_SIZE(FLASH_HandleTypeDef *handle)
{
    return 2048 * 64;
}


/***************** Interface for nor flash ********************************/


__HAL_ROM_USED int HAL_QSPIEX_WRITE_PAGE(FLASH_HandleTypeDef *hflash, uint32_t addr, const uint8_t *buf, uint32_t size)
{
    HAL_StatusTypeDef ret;
    int i, aligned_size;
    SPI_FLASH_CMD_E cid;
    uint16_t dlen;
    uint32_t param;
    //rt_base_t level;

    if (!flash_handle_valid(hflash))
        return 0;
    if (size == 0)
        return 0;
    aligned_size = QSPI_NOR_PAGE_SIZE << hflash->dualFlash;
    if (size > aligned_size)
        size = aligned_size;
    //level = rt_hw_interrupt_disable();
    if (hflash->dma != NULL)
    {
        //HAL_FLASH_CLEAR_FIFO(hflash, HAL_FLASH_CLR_RX_FIFO);
        //LOG_D("HAL_QSPIEX_WRITE_PAGE dma: 0x%x\n", addr);
        if (((hflash->size > NOR_FLASH_MAX_3B_SIZE) && (hflash->dualFlash == 0))
                || (hflash->size > NOR_FLASH_MAX_3B_SIZE * 2))
        {
            if (hflash->Mode == HAL_FLASH_QMODE)
                cid = SPI_FLASH_CMD_QPP4BA;
            else
                cid = SPI_FLASH_CMD_PP4BA;
        }
        else
        {
            if (hflash->Mode == HAL_FLASH_QMODE)
                cid = SPI_FLASH_CMD_QPP;
            else
                cid = SPI_FLASH_CMD_PP;
        }
        // add pre command process to make FLASH as write mode , to avoid prev read error.
        HAL_FLASH_PRE_CMD(hflash, cid);
        ret = HAL_FLASH_DMA_START(hflash, (char *)buf, 1, size);
        if (ret != HAL_OK)
        {
            //rt_hw_interrupt_enable(level);
            //LOG_E("nor dma start fail\n");
            return 0;
        }

        //HAL_FLASH_SET_CMD(hflash, FCMD_WREN, addr);
        HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_WREN, addr);

#ifdef QSPI_USE_CMD2
        dlen = 1;
        param = 0x01;
        dlen = dlen << hflash->dualFlash;
        if (hflash->dualFlash)
            param = 0x0101;
        HAL_FLASH_WRITE_DLEN2(hflash, dlen);
        i = HAL_FLASH_ISSUE_CMD_SEQ(hflash, cid, addr, SPI_FLASH_CMD_RDSR, param);
        if (i != 0)
            return 0;
#else

        HAL_FLASH_ISSUE_CMD(hflash, cid, addr);

        ret = HAL_FLASH_DMA_WAIT_DONE(hflash, 1000);
        if (ret != HAL_OK)
        {
            //rt_hw_interrupt_enable(level);
            //LOG_E("nor wait dma done fail\n");
            return 0;
        }

        // make sure program done
        bool res;
        HAL_FLASH_WRITE_DLEN(hflash, 2);    // cover data len
        do
        {
            //HAL_FLASH_SET_CMD(hflash, FCMD_RDSR, addr);
            HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_RDSR, addr);
            res = HAL_FLASH_IS_PROG_DONE(hflash);
        }
        while (!res);
        //rt_hw_interrupt_enable(level);
#endif
        // clear DMAE
        hflash->Instance->CR &= ~QSPI_CR_DMAE;
    }
    else
    {
        //LOG_E("FLASH must enable DMA !");
        //ASSERT(0);
        return 0;
    }

    return size;
}

// write length alway be 1
// if address not 2 aligned, prev = 1 to fill prev flash byte to buffer
// if address 2 aligned, prev = 0 to fill next byte of flash to buffer
__HAL_ROM_USED int HAL_QSPIEX_FILL_EVEN(FLASH_HandleTypeDef *hflash, uint32_t addr, uint8_t *buf, uint8_t prev)
{
    uint16_t temp;
    uint8_t *dst_buf;
    uint8_t *align_buf = (uint8_t *)&temp;
    dst_buf = (uint8_t *)(addr | hflash->base);
    if (prev) // get prev byte from flash
    {
        dst_buf--;
        align_buf[0] = *dst_buf;    // read a byte from flash with prev address
        align_buf[1] = *buf;        // fill the second byte by buf 1st byte
        // fill these 2 bytes to flash
        HAL_QSPIEX_WRITE_PAGE(hflash, addr - 1, align_buf, 2);
    }
    else    // get next byte from flash
    {
        align_buf[0] = *buf;    // fill this byte to buffer
        dst_buf++;
        align_buf[1] = *dst_buf;        // fill the second byte by flash next byte
        // fill these 2 bytes to flash
        HAL_QSPIEX_WRITE_PAGE(hflash, addr, align_buf, 2);
    }

    return 1;
}

__HAL_ROM_USED int nor_write_rom(FLASH_HandleTypeDef *hflash, uint32_t addr, const uint8_t *buf, uint32_t size)
{
    int i, cnt, taddr, tsize, aligned_size, start;
    uint8_t *tbuf;

    if (hflash == NULL || addr < hflash->base || size == 0)
        return 0;

    cnt = 0;
    tsize = size;
    tbuf = (uint8_t *)buf;
    taddr = addr - hflash->base;
    if (hflash->dualFlash) // need lenght and address 2 aligned
    {
        if (taddr & 1) // dst odd, make 2 bytes write
        {
            HAL_QSPIEX_FILL_EVEN(hflash, taddr, tbuf, 1);
            // update buffer and address
            taddr++;
            tbuf++;
            tsize--;
            cnt++;
        }
    }
    if (tsize <= 0)
        return cnt;

    // check address page align
    aligned_size = QSPI_NOR_PAGE_SIZE << hflash->dualFlash;
    //cnt = taddr - (taddr & (~(aligned_size - 1)));
    start = taddr & (aligned_size - 1);
    if (start > 0)    // start address not page aligned
    {
        start = aligned_size - start;
        if (start > tsize)    // not over one page
        {
            start = tsize;
        }
        if (hflash->dualFlash && (tsize & 1))   // for this case, it should be the laster write
        {
            i = HAL_QSPIEX_WRITE_PAGE(hflash, taddr, tbuf, tsize & (~1));

            taddr += i;
            tbuf += i;
            //tsize -= i;
            HAL_QSPIEX_FILL_EVEN(hflash, taddr, tbuf, 0);
            cnt += tsize;

            return cnt;
        }
        else
        {
            i = HAL_QSPIEX_WRITE_PAGE(hflash, taddr, tbuf, start);
            if (i != start)
            {
                return 0;
            }
        }
        taddr += start;
        tbuf += start;
        tsize -= start;
        cnt += start;
        //rt_hw_interrupt_enable(level);
    }
    // process page aligned data
    while (tsize >= aligned_size)
    {
        i = HAL_QSPIEX_WRITE_PAGE(hflash, taddr, tbuf, aligned_size);
        cnt += aligned_size;
        taddr += aligned_size;
        tbuf += aligned_size;
        tsize -= aligned_size;
        //LOG_D("write:  %d\n", cnt);
    }

    //level = rt_hw_interrupt_disable();
    // remain size
    if (tsize > 0)
    {
#if 1
        if (hflash->dualFlash && (tsize & 1))
        {
            i = HAL_QSPIEX_WRITE_PAGE(hflash, taddr, tbuf, tsize & (~1));

            if (tsize & 1)  // remain 1 byte
            {
                //cnt += i;
                taddr += i;
                tbuf += i;
                //tsize -= i;
                HAL_QSPIEX_FILL_EVEN(hflash, taddr, tbuf, 0);
                //taddr++;
                //tbuf++;
                //tsize--;
                //cnt++;
            }
            cnt += tsize;
        }
        else
#endif
        {
            i = HAL_QSPIEX_WRITE_PAGE(hflash, taddr, tbuf, tsize);
            if (i != tsize)
            {
                return 0;
            }
            cnt += tsize;
        }
    }

    return cnt;
}

__HAL_ROM_USED int nor_page_erase(FLASH_HandleTypeDef *hflash, uint32_t addr)
{
    uint16_t dlen;
    uint32_t param;

    if (!flash_handle_valid(hflash))
        return -1;
    if (flash_cmd_valid(hflash, SPI_FLASH_CMD_PE) == 0)
        return -1;
    if (((hflash->size > NOR_FLASH_MAX_3B_SIZE) && (hflash->dualFlash == 0))
            || (hflash->size > NOR_FLASH_MAX_3B_SIZE * 2))
        return -1;

    HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_WREN, addr);
#ifdef QSPI_USE_CMD2
    dlen = 1;
    param = 0x01;
    dlen = dlen << hflash->dualFlash;
    if (hflash->dualFlash)
        param = 0x0101;
    HAL_FLASH_WRITE_DLEN2(hflash, dlen);
    if (HAL_FLASH_ISSUE_CMD_SEQ(hflash, SPI_FLASH_CMD_PE, addr, SPI_FLASH_CMD_RDSR, param) != 0)
        return -1;
#else

    bool res = true;
    HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_PE, addr);
    do
    {
        HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_RDSR, addr);
        res = HAL_FLASH_IS_PROG_DONE(hflash);
    }
    while (!res);
#endif
    return 0;
}

__HAL_ROM_USED int HAL_QSPIEX_SECT_ERASE(FLASH_HandleTypeDef *hflash, uint32_t addr)
{
    uint16_t dlen;
    uint32_t param;
    SPI_FLASH_CMD_E ecmd;

    if (!flash_handle_valid(hflash))
        return -1;

    if (((hflash->size > NOR_FLASH_MAX_3B_SIZE) && (hflash->dualFlash == 0))
            || (hflash->size > NOR_FLASH_MAX_3B_SIZE * 2))
        ecmd = SPI_FLASH_CMD_SE4BA;
    else
        ecmd = SPI_FLASH_CMD_SE;

    HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_WREN, addr);
#ifdef QSPI_USE_CMD2
    dlen = 1;
    param = 0x01;
    dlen = dlen << hflash->dualFlash;
    if (hflash->dualFlash)
        param = 0x0101;

    HAL_FLASH_WRITE_DLEN2(hflash, dlen);
    if (HAL_FLASH_ISSUE_CMD_SEQ(hflash, ecmd, addr, SPI_FLASH_CMD_RDSR, param) != 0)
        return -1;
#else
    bool res = true;

    HAL_FLASH_ISSUE_CMD(hflash, ecmd, addr);
    HAL_FLASH_WRITE_DLEN(hflash, 1);
    do
    {
        HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_RDSR, addr);
        res = HAL_FLASH_IS_PROG_DONE(hflash);
    }
    while (!res);
#endif
    return 0;
}

__HAL_ROM_USED int HAL_QSPIEX_BLK32_ERASE(FLASH_HandleTypeDef *hflash, uint32_t addr)
{
    uint16_t dlen;
    uint32_t param;

    if (!flash_handle_valid(hflash))
        return -1;

    if (((hflash->size > NOR_FLASH_MAX_3B_SIZE) && (hflash->dualFlash == 0))
            || (hflash->size > NOR_FLASH_MAX_3B_SIZE * 2))
        return -1;

    HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_WREN, addr);
#ifdef QSPI_USE_CMD2
    dlen = 1;
    param = 0x01;
    dlen = dlen << hflash->dualFlash;
    if (hflash->dualFlash)
        param = 0x0101;

    HAL_FLASH_WRITE_DLEN2(hflash, dlen);
    if (HAL_FLASH_ISSUE_CMD_SEQ(hflash, SPI_FLASH_CMD_BE32, addr, SPI_FLASH_CMD_RDSR, param) != 0)
        return -1;
#else
    bool res = true;
    HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_BE32, addr);
    HAL_FLASH_WRITE_DLEN(hflash, 1);
    do
    {
        HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_RDSR, addr);
        res = HAL_FLASH_IS_PROG_DONE(hflash);
    }
    while (!res);
#endif
    return 0;
}

__HAL_ROM_USED int HAL_QSPIEX_BLK64_ERASE(FLASH_HandleTypeDef *hflash, uint32_t addr)
{
    uint16_t dlen;
    uint32_t param;
    SPI_FLASH_CMD_E ecmd;

    if (!flash_handle_valid(hflash))
        return -1;

    if (((hflash->size > NOR_FLASH_MAX_3B_SIZE) && (hflash->dualFlash == 0))
            || (hflash->size > NOR_FLASH_MAX_3B_SIZE * 2))
        ecmd = SPI_FLASH_CMD_BE4BA;
    else
        ecmd = SPI_FLASH_CMD_BE64;

    HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_WREN, addr);
#ifdef QSPI_USE_CMD2
    dlen = 1;
    param = 0x01;
    dlen = dlen << hflash->dualFlash;
    if (hflash->dualFlash)
        param = 0x0101;

    HAL_FLASH_WRITE_DLEN2(hflash, dlen);
    if (HAL_FLASH_ISSUE_CMD_SEQ(hflash, ecmd, addr, SPI_FLASH_CMD_RDSR, param) != 0)
        return -1;
#else
    bool res = true;

    HAL_FLASH_ISSUE_CMD(hflash, ecmd, addr);
    HAL_FLASH_WRITE_DLEN(hflash, 1);
    do
    {
        HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_RDSR, addr);
        res = HAL_FLASH_IS_PROG_DONE(hflash);
    }
    while (!res);
#endif
    return 0;
}

__HAL_ROM_USED int HAL_QSPIEX_CHIP_ERASE(FLASH_HandleTypeDef *hflash)
{
    uint16_t dlen;
    uint32_t param;

    if (!flash_handle_valid(hflash))
        return -1;

    HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_WREN, 0);
#ifdef QSPI_USE_CMD2
    dlen = 1;
    param = 0x01;
    dlen = dlen << hflash->dualFlash;
    if (hflash->dualFlash)
        param = 0x0101;

    HAL_FLASH_WRITE_DLEN2(hflash, dlen);
    if (HAL_FLASH_ISSUE_CMD_SEQ(hflash, SPI_FLASH_CMD_CE, 0, SPI_FLASH_CMD_RDSR, param) != 0)
        return -1;
#else
    bool res = true;

    HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_CE, 0);
    HAL_FLASH_WRITE_DLEN(hflash, 1);
    do
    {
        HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_RDSR, 0);
        res = HAL_FLASH_IS_PROG_DONE(hflash);
    }
    while (!res);
#endif

    if (hflash->Mode == HAL_FLASH_QMODE)
    {
        if (((hflash->size > NOR_FLASH_MAX_3B_SIZE) && (hflash->dualFlash == 0))
                || (hflash->size > NOR_FLASH_MAX_3B_SIZE * 2))
            HAL_FLASH_FADDR_SET_QSPI(hflash, true);
        else
            HAL_FLASH_SET_QUAL_SPI(hflash, true);
    }

    return 0;
}

__HAL_ROM_USED int nor_erase_rom(FLASH_HandleTypeDef *hflash, uint32_t addr, uint32_t size)
{
    uint32_t al_size;
    uint32_t al_addr;
    //rt_base_t level;
    int ret = 0;

    if (size == 0)
        return 0;

    if (size >= hflash->size)
        return HAL_QSPIEX_CHIP_ERASE(hflash);
    //level = rt_hw_interrupt_disable();

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
    // set to single before erase, recover later
    //if (hflash->Mode == HAL_FLASH_QMODE)
    //    HAL_FLASH_SET_QUAL_SPI(hflash, false);

    //addr &= (NOR_SUPPORT_MAX_SIZE - 1);

    // address alinged down to page, size aligned up to page size
    //al_addr = GetPage(addr);
    //al_size = GetPSize((addr-al_addr)+size);
    // page erase not support, start addr should be aligned.
    al_addr = GET_ALIGNED_DOWN((QSPI_NOR_SECT_SIZE << hflash->dualFlash), addr);
    al_size = GET_ALIGNED_UP((QSPI_NOR_SECT_SIZE << hflash->dualFlash), size);

    //HAL_FLASH_CLR_SR(hflash);
    //LOG_D("flash erase from 0x%x + %d to 0x%x + %d\n", addr, size, al_addr, al_size);
    // 1 block 64k aligned, for start addr not aligned do not process, need support later
    if (IS_ALIGNED((QSPI_NOR_BLK64_SIZE << hflash->dualFlash), al_addr) && (al_size >= (QSPI_NOR_BLK64_SIZE << hflash->dualFlash))) // block erease first
    {
        while (al_size >= (QSPI_NOR_BLK64_SIZE << hflash->dualFlash))
        {
            HAL_QSPIEX_BLK64_ERASE(hflash, al_addr);
            al_size -= QSPI_NOR_BLK64_SIZE << hflash->dualFlash;
            al_addr += QSPI_NOR_BLK64_SIZE << hflash->dualFlash;
        }
        //LOG_D("Block64 erase to 0x%x\n", al_addr);
    }
#if 0   // for some chip like 32MB winbond, it not support 4 byte blcok32 erase
    // 2 block 32 aligned.
    if ((al_size >= (QSPI_NOR_BLK32_SIZE << hflash->dualFlash)) && IS_ALIGNED((QSPI_NOR_BLK32_SIZE << hflash->dualFlash), al_addr))
    {
        while (al_size >= (QSPI_NOR_BLK32_SIZE << hflash->dualFlash))
        {
            HAL_QSPIEX_BLK32_ERASE(hflash, al_addr);
            al_size -= QSPI_NOR_BLK32_SIZE << hflash->dualFlash;
            al_addr += QSPI_NOR_BLK32_SIZE << hflash->dualFlash;
        }
        //LOG_D("Block32 erase to 0x%x\n", al_addr);
    }
#endif
    // sector aligned
    if ((al_size >= (QSPI_NOR_SECT_SIZE << hflash->dualFlash)) && IS_ALIGNED((QSPI_NOR_SECT_SIZE << hflash->dualFlash), al_addr))
    {
        while (al_size >= (QSPI_NOR_SECT_SIZE << hflash->dualFlash))
        {
            HAL_QSPIEX_SECT_ERASE(hflash, al_addr);
            al_size -= QSPI_NOR_SECT_SIZE << hflash->dualFlash;
            al_addr += QSPI_NOR_SECT_SIZE << hflash->dualFlash;
        }
        //LOG_D("sector erase to 0x%x\n", al_addr);
    }

#if 0
    // page aligned
    if ((al_size >= FLASH_PAGE_SIZE) && ADDR_PAGE_ALIGN(al_addr))
    {
        cnt = 0;
        while (cnt < al_size)
        {
            rt_flash_page_erase(hflash, al_addr);
            cnt += FLASH_PAGE_SIZE;
            al_addr += FLASH_PAGE_SIZE;
        }
        al_size -= cnt;
        //LOG_D("page erase to 0x%x\n", al_addr);
    }
#endif

    if (al_size > 0)    // something wrong
    {
        ret = -1;
        goto _exit;
    }
    //todo , size lager than block but start addr not aligned, only page erase
    //
    if (hflash->Mode == HAL_FLASH_QMODE)
    {
        HAL_FLASH_SET_QUAL_SPI(hflash, true);
    }


_exit:

    //rt_hw_interrupt_enable(level);

    return ret;

}

__HAL_ROM_USED uint32_t HAL_QSPI_GET_CLK(FLASH_HandleTypeDef *fhandle)
{
    int src;
    uint8_t div;
    uint32_t freq;
    int clk_module;

    if (NULL == fhandle)
        return 0;

    div = HAL_FLASH_GET_DIV(fhandle);
    if (div <= 0)
        return 0;

    if (FLASH1 == fhandle->Instance)
    {
        clk_module = RCC_CLK_MOD_FLASH1;
    }
    else if (FLASH2 == fhandle->Instance)
    {
        clk_module = RCC_CLK_MOD_FLASH2;
    }
    else if (FLASH3 == fhandle->Instance)
    {
        clk_module = RCC_CLK_MOD_FLASH3;
    }
    else
    {
        return 0;
    }

    src = HAL_RCC_HCPU_GetClockSrc(clk_module);
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
    freq /= div;

    return freq;
}

__HAL_ROM_USED int HAL_QSPI_READ_ID(FLASH_HandleTypeDef *fhandle)
{
    if (fhandle == NULL)
        return FLASH_UNKNOW_ID;

    if (fhandle->isNand)
        return nand_read_id(fhandle, 0);
    else
        return HAL_FLASH_GET_NOR_ID(fhandle);

    return FLASH_UNKNOW_ID;
}

__HAL_ROM_USED int HAL_QSPI_GET_MEMSIZE(FLASH_HandleTypeDef *fhandle)
{
    if (fhandle == NULL)
        return 0;
    return fhandle->size;
}


// it only called in bootloader to detect dual flash, can not called in xip
__HAL_ROM_USED int qspi_flash_get_id2(FLASH_HandleTypeDef *hflash)
{
    uint32_t id, temp, id2;;
    if (hflash->dualFlash == 0)
        return 0;

    //HAL_FLASH_CLEAR_FIFO(hflash, HAL_FLASH_CLR_RX_FIFO);

    // configure CCR
    //HAL_FLASH_MANUAL_CMD(hflash, 0, 1, 0, 0, 0, 2, 1, 1);
    HAL_FLASH_MANUAL_CMD(hflash, 0, 1, 0, 0, 0, 0, 0, 1);

    // configure data length, 2 byte, manufacturer id and device id, 0X90
    // configure data length, 3 byte, manufacturer id , memory type id and capacity id, 0X9F
    HAL_FLASH_WRITE_DLEN(hflash, 6);

    // send command, read id command
    //HAL_FLASH_SET_CMD(hflash, 0x90, 0);
    HAL_FLASH_SET_CMD(hflash, 0x9f, 0);

    //while (HAL_FLASH_IS_RX_EMPTY(hflash));

    id = HAL_FLASH_READ32(hflash);
    id2 = HAL_FLASH_READ32(hflash);
    // big/little endian?
    // ------- memory type --------| --- manufacturer id
    temp = ((id & 0xff00) >> 8) | ((id & 0xff000000) >> 16);
    // --- mem desity-----
    temp |= (id2 & 0xff00) << 8;
    id = temp;

    return (int)(id & 0xffffff);
}

__HAL_ROM_USED int HAL_QSPIEX_FLASH_WRITE(FLASH_HandleTypeDef *hflash, uint32_t addr, const uint8_t *buf, uint32_t size)
{
    int cnt = 0;

    if (hflash == NULL || addr < hflash->base || buf == NULL)
        return 0;

    if (hflash->isNand == 0)
        cnt = nor_write_rom(hflash, addr, buf, size);
    else
        cnt = HAL_NAND_WRITE_PAGE(hflash, addr, buf, size);

    return cnt;
}

__HAL_ROM_USED int HAL_QSPIEX_FLASH_ERASE(FLASH_HandleTypeDef *hflash, uint32_t addr, uint32_t size)
{
    int res = 0;
    int i;

    if (hflash == NULL || addr < hflash->base)
        return 0;

    if (hflash->isNand == 0)
        res = nor_erase_rom(hflash, addr, size);
    else
        for (i = 0; i < size / SPI_NAND_BLK_SIZE; i++)
        {
            res = HAL_NAND_GET_BADBLK(hflash, (addr >> 17) + i);
            if (res) // block is bad, do not erase, or bad mark will be cover
                return -2;
            res = HAL_NAND_ERASE_BLK(hflash, addr + i * SPI_NAND_BLK_SIZE);
            if (res) // erase fail
                return -1;
        }

    return res;

}

__HAL_ROM_USED void HAL_QSPIEX_FLASH_RESET(FLASH_HandleTypeDef *hflash)
{
    if (hflash == NULL)
        return ;

    HAL_FLASH_MANUAL_CMD(hflash, 0, 0, 0, 0, 0, 0, 0, 1);
    HAL_FLASH_SET_CMD(hflash, 0x66, 0);

    HAL_Delay_us(300);
    // add a delay?
    HAL_FLASH_MANUAL_CMD(hflash, 0, 0, 0, 0, 0, 0, 0, 1);
    HAL_FLASH_SET_CMD(hflash, 0x99, 0);

    return ;
}

__HAL_ROM_USED void HAL_QSPIEX_FLASH_RESET2(QSPI_TypeDef *hmpi)
{
    uint32_t value, en;
    if (hmpi == NULL)
        return ;

    en = hmpi->CR & QSPI_CR_EN_Msk;
    hmpi->CR |= QSPI_CR_EN;

    value = ((0 << QSPI_CCR1_FMODE_Pos) | (0 << QSPI_CCR1_DMODE_Pos)
             | (0 << QSPI_CCR1_DCYC_Pos) | (0 << QSPI_CCR1_ADSIZE_Pos)
             | (0 << QSPI_CCR1_ABMODE_Pos) | (0 << QSPI_CCR1_ABSIZE_Pos)
             | (0 << QSPI_CCR1_ADMODE_Pos) | (1 << QSPI_CCR1_IMODE_Pos));
    hmpi->CCR1 = value;

    hmpi->AR1 = 0;
    hmpi->CMDR1 = 0x66;

    while (!(hmpi->SR & QSPI_SR_TCF));
    hmpi->SCR |= QSPI_SCR_TCFC;


    HAL_Delay_us(30);   // need delay 30 us
    // add a delay?
    value = ((0 << QSPI_CCR1_FMODE_Pos) | (0 << QSPI_CCR1_DMODE_Pos)
             | (0 << QSPI_CCR1_DCYC_Pos) | (0 << QSPI_CCR1_ADSIZE_Pos)
             | (0 << QSPI_CCR1_ABMODE_Pos) | (0 << QSPI_CCR1_ABSIZE_Pos)
             | (0 << QSPI_CCR1_ADMODE_Pos) | (1 << QSPI_CCR1_IMODE_Pos));
    hmpi->CCR1 = value;

    hmpi->AR1 = 0;
    hmpi->CMDR1 = 0x99;

    while (!(hmpi->SR & QSPI_SR_TCF));
    hmpi->SCR |= QSPI_SCR_TCFC;

    HAL_Delay_us(12000);    // delay 12ms

    if (en == 0)
    {
        hmpi->CR &= ~QSPI_CR_EN;
    }

    return ;
}

__HAL_ROM_USED int HAL_QSPI_ENABLE_WDT()
{
    FLASH_HandleTypeDef hflash;
    uint16_t value = 0x3ff;

    hflash.Instance = FLASH1;
    HAL_FLASH_SET_WDT(&hflash, value);
    hflash.Instance = FLASH2;
    HAL_FLASH_SET_WDT(&hflash, value);
    hflash.Instance = FLASH3;
    HAL_FLASH_SET_WDT(&hflash, value);
    return 0;
}

__HAL_ROM_USED uint32_t HAL_QSPI_GET_SR(FLASH_HandleTypeDef *hflash)
{
    uint32_t srl, srh, sr;

    if (hflash == NULL)
        return 0;

    srl = srh = sr = 0;
    if (hflash->isNand == 0)
    {
        //HAL_FLASH_CLEAR_FIFO(hflash, HAL_FLASH_CLR_RX_TX_FIFO);

        HAL_FLASH_WRITE_DLEN(hflash, 1 << hflash->dualFlash);
        HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_RDSR, 0);
        srl = HAL_FLASH_READ32(hflash) ;

        int res = HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_RDSR2, 0);
        if (res == 0)
            srh = HAL_FLASH_READ32(hflash);

        //rt_kprintf("Get flash SRL 0x%x, SRH 0x%x\n", srl, srh);
        sr = ((srh) << 16) | (srl);
    }
    else
    {
        sr = nand_get_status(hflash);
        //rt_kprintf("Get flash SR 0x%08x\n", srl);
    }

    return sr;
}

__HAL_ROM_USED int HAL_QSPI_ERASE_OTP(FLASH_HandleTypeDef *hflash, uint32_t addr)
{
    uint32_t srh;
    uint16_t dlen;
    int res, opbit;
    uint32_t param;

    if (hflash == NULL || hflash->ctable == NULL)
        return -1;
    if (addr < SPI_FLASH_OTP_BASE || addr > SPI_FLASH_OTP_BASE + (hflash->ctable->mode_reg << 12))
        return -1;

    srh = HAL_QSPI_GET_OTP_LB(hflash, addr);
    //rt_kprintf("srh = %d\n", srh);
    opbit = addr >> 12;
    if (opbit < 1 || opbit > hflash->ctable->mode_reg)
        return -1;
    opbit = 1 << (opbit - 1);
    if (opbit & srh) // this security register has been locked, can not erase any more
        return -2;
    //rt_kprintf("opbit = %d\n", opbit);

    addr = addr << hflash->dualFlash;
    HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_WREN, 0);
#ifdef QSPI_USE_CMD2
    dlen = 1;
    param = 0x01;
    dlen = dlen << hflash->dualFlash;
    if (hflash->dualFlash)
        param = param | (param << 8);
    HAL_FLASH_WRITE_DLEN2(hflash, dlen);
    res = HAL_FLASH_ISSUE_CMD_SEQ(hflash, SPI_FLASH_CMD_ERSCUR, addr, SPI_FLASH_CMD_RDSR, param);
    if (res != 0)
        return -3;
#else
    res = HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_ERSCUR, addr);

    HAL_FLASH_WRITE_DLEN(hflash, 1 << hflash->dualFlash);
    if (res == 0)
    {
        do
        {
            HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_RDSR, 0);
            res = HAL_FLASH_IS_PROG_DONE(hflash);
        }
        while (!res);
    }
#endif
    return 0;
}

__HAL_ROM_USED int HAL_QSPI_WRITE_OTP(FLASH_HandleTypeDef *hflash, uint32_t addr, uint8_t *buf, uint32_t size)
{
    uint32_t srh;
    uint16_t dlen;
    int res, opbit;
    HAL_StatusTypeDef ret;
    uint32_t param;

    if (hflash == NULL || hflash->ctable == NULL)
        return 0;
    if (addr < SPI_FLASH_OTP_BASE || addr > SPI_FLASH_OTP_BASE + (hflash->ctable->mode_reg << 12))
        return 0;

    if ((addr & 0x3ff) + size   > hflash->ctable->oob_size * 256)
        return 0;

    srh = HAL_QSPI_GET_OTP_LB(hflash, addr);
    opbit = addr >> 12;
    if (opbit < 1 || opbit > hflash->ctable->mode_reg)
        return 0;
    opbit = 1 << (opbit - 1);
    if (opbit & srh) // this security register has been locked, can not write any more
        return 0;

    //HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_WREN, 0);
    //res = HAL_FLASH_ISSUE_CMD(hflash,SPI_FLASH_CMD_PRSCUR, addr);
    addr = addr << hflash->dualFlash;
    if (hflash->dma != NULL)
    {
        // add pre command process to make FLASH as write mode , to avoid prev read error.
        HAL_FLASH_PRE_CMD(hflash, SPI_FLASH_CMD_PRSCUR);
        ret = HAL_FLASH_DMA_START(hflash, (char *)buf, 1, size);
        if (ret != HAL_OK)
        {
            return 0;
        }

        HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_WREN, 0);

#ifdef QSPI_USE_CMD2
        dlen = 1;
        param = 0x01;
        dlen = dlen << hflash->dualFlash;
        if (hflash->dualFlash)
            param = param | (param << 8);
        HAL_FLASH_WRITE_DLEN2(hflash, dlen);
        res = HAL_FLASH_ISSUE_CMD_SEQ(hflash, SPI_FLASH_CMD_PRSCUR, addr, SPI_FLASH_CMD_RDSR, param);
        if (res != 0)
            return 0;
#else
        HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_PRSCUR, addr);

        ret = HAL_FLASH_DMA_WAIT_DONE(hflash, 1000);
        if (ret != HAL_OK)
        {
            return 0;
        }

        // make sure program done
        HAL_FLASH_WRITE_DLEN(hflash, 2);    // cover data len
        do
        {
            HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_RDSR, addr);
            res = HAL_FLASH_IS_PROG_DONE(hflash);
        }
        while (!res);
        //rt_hw_interrupt_enable(level);
#endif
        // clear DMAE
        hflash->Instance->CR &= ~QSPI_CR_DMAE;
    }

    return size;
}

#endif  // HAL_QSPI_MODULE_ENABLED

/// @} FLASH

/// @} BF0_HAL_Driver


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
