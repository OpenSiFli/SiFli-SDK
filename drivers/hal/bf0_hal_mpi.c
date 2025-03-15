/**
  ******************************************************************************
  * @file   bf0_hal_mpi.c
  * @author Sifli software development team
  * @brief   QSPI HAL module driver.
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
#include "drv_io.h"
/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @defgroup FLASH FLASH
  * @brief FLASH HAL module driver
  * @{
  */


#if defined(HAL_MPI_MODULE_ENABLED)||defined(_SIFLI_DOXYGEN_)

#include "flash_table.h"

__HAL_ROM_USED HAL_StatusTypeDef HAL_QSPI_Init(FLASH_HandleTypeDef *hflash, qspi_configure_t *cfg)
{
    if (hflash == NULL || cfg == NULL)
        return HAL_ERROR;

    hflash->ErrorCode = 0;
    hflash->State = HAL_FLASH_STATE_READY;

    hflash->Instance = cfg->Instance;
    hflash->Mode = cfg->line;
    hflash->base = cfg->base;
    hflash->size = cfg->msize * 0x100000;

    hflash->Instance->TIMR = 0xFF;
    hflash->Instance->CIR = 0x50005000;
    hflash->Instance->ABR1 = 0xFF;
    hflash->Instance->HRABR = 0xff;

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_FLASH_PreInit(FLASH_HandleTypeDef *hflash)
{
    uint8_t fid, did, mtype;
    int size = 0;

    if (hflash == NULL)
        return HAL_ERROR;

    hflash->ErrorCode = 0;
    hflash->State = HAL_FLASH_STATE_READY;

    //hflash->Instance = cfg->Instance;
    hflash->Mode = 0;
    //hflash->base = cfg->base;
    //hflash->size = cfg->msize * 0x100000;

    hflash->Instance->TIMR = 0xFF;
    hflash->Instance->CIR = 0x50005000;
    hflash->Instance->ABR1 = 0xFF;
    hflash->Instance->HRABR = 0xff;

    hflash->isNand =  0;
    hflash->dma = NULL;

    // enable QSPI
    HAL_FLASH_ENABLE_QSPI(hflash, 1);

    uint32_t dev_id = HAL_FLASH_GET_NOR_ID(hflash);
    fid = (uint8_t)dev_id & 0xff;
    mtype = (uint8_t)((dev_id >> 8) & 0xff);
    did = (uint8_t)((dev_id >> 16) & 0xff);

    hflash->ctable = spi_flash_get_cmd_by_id(fid, did, mtype);
    if (hflash->ctable  == NULL)
    {
        // disable QSPI
        HAL_FLASH_ENABLE_QSPI(hflash, 0);
        return HAL_ERROR;
    }

    size = spi_flash_get_size_by_id(fid, did, mtype);
    if (size != 0)  // use size from table to replace configure size
    {
        hflash->size = size;
    }

    // only nor need set QE mode at initial
    if (hflash->isNand == SPI_MODE_NOR)
    {
        HAL_FLASH_CLR_PROTECT(hflash);
        //HAL_Delay_us(30);
        if (hflash->size > NOR_FLASH_MAX_3B_SIZE)
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

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_FLASH_SET_AHB_RCMD(FLASH_HandleTypeDef *hflash, uint8_t cmd)
{
    uint32_t value;
    if (hflash == NULL)
        return HAL_ERROR;

    value = hflash->Instance->HCMDR & (~MPI_HCMDR_RCMD_Msk);

    value |= ((cmd & 0xff) << MPI_HCMDR_RCMD_Pos);
    hflash->Instance->HCMDR = value;

    return HAL_OK;
}

__HAL_ROM_USED void HAL_FLASH_CFG_AHB_RCMD(FLASH_HandleTypeDef *hflash, int8_t data_mod,
        int8_t dum_cyc, int8_t ab_size, int8_t ab_mod,
        int8_t addr_size, int8_t addr_mod, int8_t cmd_mode)
{
    if (hflash == NULL)
        return;

    uint32_t value = hflash->Instance->HRCCR;
    uint32_t mask = MPI_HRCCR_DMODE | MPI_HRCCR_DCYC
                    | MPI_HRCCR_ABSIZE | MPI_HRCCR_ABMODE | MPI_HRCCR_ADSIZE
                    | MPI_HRCCR_ADMODE | MPI_HRCCR_IMODE;

    value &= ~mask;
    value |= ((data_mod << MPI_HRCCR_DMODE_Pos)
              | (dum_cyc << MPI_HRCCR_DCYC_Pos) | (addr_size << MPI_HRCCR_ADSIZE_Pos)
              | (ab_mod << MPI_HRCCR_ABMODE_Pos) | (ab_size << MPI_HRCCR_ABSIZE_Pos)
              | (addr_mod << MPI_HRCCR_ADMODE_Pos) | (cmd_mode << MPI_HRCCR_IMODE_Pos));

    hflash->Instance->HRCCR = value;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_FLASH_SET_AHB_WCMD(FLASH_HandleTypeDef *hflash, uint8_t cmd)
{
    uint32_t value;
    if (hflash == NULL)
        return HAL_ERROR;

    value = hflash->Instance->HCMDR & (~MPI_HCMDR_WCMD_Msk);

    value |= ((cmd & 0xff) << MPI_HCMDR_WCMD_Pos);
    hflash->Instance->HCMDR = value;

    return HAL_OK;
}

__HAL_ROM_USED void HAL_FLASH_CFG_AHB_WCMD(FLASH_HandleTypeDef *hflash, int8_t data_mod,
        int8_t dum_cyc, int8_t ab_size, int8_t ab_mod,
        int8_t addr_size, int8_t addr_mod, int8_t cmd_mode)
{
    if (hflash == NULL)
        return;

    uint32_t value = hflash->Instance->HWCCR;
    uint32_t mask = MPI_HWCCR_DMODE | MPI_HWCCR_DCYC
                    | MPI_HWCCR_ABSIZE | MPI_HWCCR_ABMODE | MPI_HWCCR_ADSIZE
                    | MPI_HWCCR_ADMODE | MPI_HWCCR_IMODE;

    value &= ~mask;
    value |= ((data_mod << MPI_HWCCR_DMODE_Pos)
              | (dum_cyc << MPI_HWCCR_DCYC_Pos) | (addr_size << MPI_HWCCR_ADSIZE_Pos)
              | (ab_mod << MPI_HWCCR_ABMODE_Pos) | (ab_size << MPI_HWCCR_ABSIZE_Pos)
              | (addr_mod << MPI_HWCCR_ADMODE_Pos) | (cmd_mode << MPI_HWCCR_IMODE_Pos));

    hflash->Instance->HWCCR = value;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_FLASH_SET_CMD(FLASH_HandleTypeDef *hflash, uint8_t cmd, uint32_t addr)
{

    HAL_StatusTypeDef res = HAL_OK;

    if (hflash == NULL)
        return HAL_ERROR;

    hflash->Instance->AR1 = addr;
    if (hflash->cs_ctrl != NULL)
    {
        hflash->cs_ctrl(0);
        // DELAY
        volatile uint32_t i = 240 / 5; // max 240m?
        while (i-- > 0);
        hflash->cs_ctrl(1);
    }
    hflash->Instance->CMDR1 = cmd & 0XFF;

    while (!HAL_FLASH_IS_CMD_DONE(hflash));
    HAL_FLASH_CLR_CMD_DONE(hflash);

    return res;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_FLASH_CFG_CMD(FLASH_HandleTypeDef *hflash, uint8_t cmd, uint32_t addr, uint8_t is_cmd2)
{

    HAL_StatusTypeDef res = HAL_OK;

    if (hflash == NULL)
        return HAL_ERROR;

    if (is_cmd2)
    {
        hflash->Instance->AR2 = addr;
        hflash->Instance->CMDR2 = cmd & 0XFF;
    }
    else
    {
        hflash->Instance->AR1 = addr;
        hflash->Instance->CMDR1 = cmd & 0XFF;
    }

    return res;
}


__HAL_ROM_USED HAL_StatusTypeDef HAL_FLASH_WRITE_WORD(FLASH_HandleTypeDef *hflash, uint32_t data)
{
    if (hflash == NULL)
        return HAL_ERROR;

    hflash->Instance->DR = data;

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_FLASH_WRITE_DLEN(FLASH_HandleTypeDef *hflash, uint32_t len)
{
    if (hflash == NULL)
        return HAL_ERROR;

    hflash->Instance->DLR1 = (len - 1) & MPI_DLR1_DLEN_Msk;

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_FLASH_WRITE_DLEN2(FLASH_HandleTypeDef *hflash, uint16_t len)
{
    if (hflash == NULL)
        return HAL_ERROR;

    hflash->Instance->DLR2 = (len - 1) & MPI_DLR2_DLEN_Msk;

    return HAL_OK;
}

__HAL_ROM_USED void HAL_FLASH_WRITE_ABYTE(FLASH_HandleTypeDef *hflash, uint32_t ab)
{
    if (hflash == NULL)
        return;

    hflash->Instance->ABR1 = ab;
}

__HAL_ROM_USED bool HAL_FLASH_IS_CMD_DONE(FLASH_HandleTypeDef *hflash)
{
    if (hflash == NULL)
        return false;

    return (hflash->Instance->SR & MPI_SR_TCF);
}

__HAL_ROM_USED void HAL_FLASH_CLR_CMD_DONE(FLASH_HandleTypeDef *hflash)
{
    if (hflash == NULL)
        return ;

    hflash->Instance->SCR |= MPI_SCR_TCFC;
}

__HAL_ROM_USED void HAL_FLASH_CLR_STATUS(FLASH_HandleTypeDef *hflash, uint32_t status)
{
    if (hflash == NULL)
        return ;

    hflash->Instance->SCR |= status;
}

__HAL_ROM_USED bool HAL_FLASH_IS_BUSY(FLASH_HandleTypeDef *hflash)
{
    if (hflash == NULL)
        return false;

    return (hflash->Instance->SR & MPI_SR_BUSY);
}

__HAL_ROM_USED bool HAL_FLASH_STATUS_MATCH(FLASH_HandleTypeDef *hflash)
{
    if (hflash == NULL)
        return false;

    return (hflash->Instance->SR & MPI_SR_SMF_Msk);
}

__HAL_ROM_USED bool HAL_FLASH_IS_RX_EMPTY(FLASH_HandleTypeDef *hflash)
{
    if (hflash == NULL)
        return false;

    return (hflash->Instance->FIFOCR & MPI_FIFOCR_RXE);
}

__HAL_ROM_USED bool HAL_FLASH_IS_TX_FULL(FLASH_HandleTypeDef *hflash)
{
    if (hflash == NULL)
        return false;

    return (hflash->Instance->FIFOCR & MPI_FIFOCR_TXF);
}
__HAL_ROM_USED bool HAL_FLASH_IS_TX_EMPTY(FLASH_HandleTypeDef *hflash)
{
    if (hflash == NULL)
        return false;

    return (hflash->Instance->SR & MPI_SR_FUF);
}

__HAL_ROM_USED bool HAL_FLASH_IS_PROG_DONE(FLASH_HandleTypeDef *hflash)
{
    if (hflash == NULL)
        return true;

    // CHECK WIP : Write in Progress, 1 means device busy, 0 means device idle
    // tdr and rdr all in DR now
    return !(hflash->Instance->DR & 0X01);
}

__HAL_ROM_USED uint32_t HAL_FLASH_READ32(FLASH_HandleTypeDef *hflash)
{
    if (hflash == NULL)
        return 0;

    return (uint32_t)(hflash->Instance->DR);
}

__HAL_ROM_USED void HAL_FLASH_CLEAR_FIFO(FLASH_HandleTypeDef *hflash, HAL_FLASH_ClearFifo mode)
{
    if (hflash == NULL)
        return;

    switch (mode)
    {
    case HAL_FLASH_CLR_RX_FIFO:
        hflash->Instance->FIFOCR |= MPI_FIFOCR_RXCLR;
        break;
    case HAL_FLASH_CLR_TX_FIFO:
        hflash->Instance->FIFOCR |= MPI_FIFOCR_TXCLR;
        break;
    case HAL_FLASH_CLR_RX_TX_FIFO:
        hflash->Instance->FIFOCR |= (MPI_FIFOCR_RXCLR | MPI_FIFOCR_TXCLR);
        //hflash->Instance->FIFOCR = MPI_FIFOCR_TXCLR;
        break;
    }

    //while (HAL_FLASH_IS_BUSY(hflash));
}

__HAL_ROM_USED void HAL_FLASH_SET_TXSLOT(FLASH_HandleTypeDef *hflash, uint8_t slot)
{
    if (hflash == NULL)
        return ;

    uint32_t value = hflash->Instance->FIFOCR & (~MPI_FIFOCR_TXSLOTS);

    hflash->Instance->FIFOCR = value | ((slot & 0x1f) << MPI_FIFOCR_TXSLOTS_Pos);
}

__WEAK void HAL_FLASH_SET_CLK_rom(FLASH_HandleTypeDef *hflash, uint8_t div)
{
    if (hflash == NULL)
        return;

    //uint32_t value = hflash->Instance->CCR1;
    //value &= ~QSPI_CCR1_DCYC;
    //value |= ((delay & 0xf) << QSPI_CCR1_DCYC_Pos);

    //hflash->Instance->CCR1 = value;
    hflash->Instance->PSCLR = div;
}

__HAL_ROM_USED uint8_t HAL_FLASH_GET_DIV(FLASH_HandleTypeDef *hflash)
{
    uint8_t div = 0;
    if (hflash == NULL)
        return div;

    div = GET_REG_VAL(hflash->Instance->PSCLR, MPI_PSCLR_DIV_Msk, MPI_PSCLR_DIV_Pos);
    return div;
}

__HAL_ROM_USED void HAL_FLASH_MANUAL_CMD(FLASH_HandleTypeDef *hflash, int8_t func_mod, int8_t data_mod,
        int8_t dum_cyc, int8_t ab_size, int8_t ab_mod,
        int8_t addr_size, int8_t addr_mod, int8_t cmd_mode)
{
    if (hflash == NULL)
        return;

    uint32_t value = hflash->Instance->CCR1;
    uint32_t mask = MPI_CCR1_FMODE | MPI_CCR1_DMODE | MPI_CCR1_DCYC
                    | MPI_CCR1_ABMODE | MPI_CCR1_ABSIZE | MPI_CCR1_ADSIZE
                    | MPI_CCR1_ADMODE | MPI_CCR1_IMODE;

    value &= ~mask;
    value |= ((func_mod << MPI_CCR1_FMODE_Pos) | (data_mod << MPI_CCR1_DMODE_Pos)
              | (dum_cyc << MPI_CCR1_DCYC_Pos) | (addr_size << MPI_CCR1_ADSIZE_Pos)
              | (ab_mod << MPI_CCR1_ABMODE_Pos) | (ab_size << MPI_CCR1_ABSIZE_Pos)
              | (addr_mod << MPI_CCR1_ADMODE_Pos) | (cmd_mode << MPI_CCR1_IMODE_Pos));

    hflash->Instance->CCR1 = value;
}

__HAL_ROM_USED void HAL_FLASH_MANUAL_CMD2(FLASH_HandleTypeDef *hflash, int8_t func_mod, int8_t data_mod,
        int8_t dum_cyc, int8_t ab_size, int8_t ab_mod,
        int8_t addr_size, int8_t addr_mod, int8_t cmd_mode)
{
    if (hflash == NULL)
        return;

    uint32_t value = hflash->Instance->CCR2;
    uint32_t mask = MPI_CCR2_FMODE | MPI_CCR2_DMODE | MPI_CCR2_DCYC
                    | MPI_CCR2_ABMODE | MPI_CCR2_ABSIZE | MPI_CCR2_ADSIZE
                    | MPI_CCR2_ADMODE | MPI_CCR2_IMODE;

    value &= ~mask;
    value |= ((func_mod << MPI_CCR2_FMODE_Pos) | (data_mod << MPI_CCR2_DMODE_Pos)
              | (dum_cyc << MPI_CCR2_DCYC_Pos) | (addr_size << MPI_CCR2_ADSIZE_Pos)
              | (ab_mod << MPI_CCR2_ABMODE_Pos) | (ab_size << MPI_CCR2_ABSIZE_Pos)
              | (addr_mod << MPI_CCR2_ADMODE_Pos) | (cmd_mode << MPI_CCR2_IMODE_Pos));

    hflash->Instance->CCR2 = value;
}


__HAL_ROM_USED HAL_StatusTypeDef HAL_FLASH_SET_ALIAS_RANGE(FLASH_HandleTypeDef *hflash, uint32_t start, uint32_t len)
{
    HAL_StatusTypeDef res = HAL_OK;
    uint32_t end;
    if (hflash == NULL)
        return HAL_ERROR;

    end = (start + len + 0x3ff) & MPI_AAEAR_EA;  // make end address aligned to 1024
    hflash->Instance->AASAR = start & MPI_AASAR_SA;
    hflash->Instance->AAEAR = end;

    return res;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_FLASH_SET_ALIAS_OFFSET(FLASH_HandleTypeDef *hflash, uint32_t offset)
{
    HAL_StatusTypeDef res = HAL_OK;
    if (hflash == NULL)
        return HAL_ERROR;

    hflash->Instance->AAOAR = offset & MPI_AAOAR_OA;

    return res;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_FLASH_SET_CTR(FLASH_HandleTypeDef *hflash, uint32_t start, uint32_t end)
{
    if (hflash == NULL)
        return HAL_ERROR;

    hflash->Instance->CTRSAR = start & MPI_CTRSAR_SA;    // make start address aligned to 1024
    hflash->Instance->CTREAR = (end + 0x3ff) & MPI_CTREAR_EA; // make end address aligned to 1024

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_FLASH_SET_NONCE(FLASH_HandleTypeDef *hflash, uint8_t *data)
{
    if (hflash == NULL || data == NULL)
        return HAL_ERROR;

    hflash->Instance->NONCEA = data[3] | ((uint32_t)data[2] << 8) | ((uint32_t)data[1] << 16) | ((uint32_t)data[0] << 24);
    hflash->Instance->NONCEB = data[7] | ((uint32_t)data[6] << 8) | ((uint32_t)data[5] << 16) | ((uint32_t)data[4] << 24);

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_FLASH_SET_AES(FLASH_HandleTypeDef *hflash, HAL_FLASH_AES_Mode mode)
{
    if (hflash == NULL)
        return HAL_ERROR;

    if (mode == HAL_FLASH_AES256)
    {
        hflash->Instance->CR |= MPI_CR_CTRM;
    }
    else    // AES 128
    {
        hflash->Instance->CR &= (~MPI_CR_CTRM);
        //hflash->Instance->CR |= (FLASHC_CR_CTRM_AES128);
    }

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_FLASH_ENABLE_AES(FLASH_HandleTypeDef *hflash, uint8_t en)
{
    if (hflash == NULL)
        return HAL_ERROR;

    if (en)
    {
        hflash->Instance->CR |= MPI_CR_CTRE;
    }
    else
    {
        hflash->Instance->CR &= (~MPI_CR_CTRE);
    }

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_FLASH_ENABLE_QSPI(FLASH_HandleTypeDef *hflash, uint8_t en)
{
    if (hflash == NULL)
        return HAL_ERROR;

    if (en)
    {
        hflash->Instance->CR |= MPI_CR_EN;
    }
    else
    {
        hflash->Instance->CR &= (~MPI_CR_EN);
    }

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_FLASH_ENABLE_OPI(FLASH_HandleTypeDef *hflash, uint8_t en)
{
    if (hflash == NULL)
        return HAL_ERROR;

    if (en)
    {
        hflash->Instance->CR |= MPI_CR_OPIE;
    }
    else
    {
        hflash->Instance->CR &= (~MPI_CR_OPIE);
    }

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_FLASH_ENABLE_HYPER(FLASH_HandleTypeDef *hflash, uint8_t en)
{
    if (hflash == NULL)
        return HAL_ERROR;

    if (en)
    {
        hflash->Instance->DCR |= MPI_DCR_HYPER;
    }
    else
    {
        hflash->Instance->DCR &= (~MPI_DCR_HYPER);
    }

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_FLASH_ENABLE_CMD2(FLASH_HandleTypeDef *hflash, uint8_t en)
{
    if (hflash == NULL)
        return HAL_ERROR;

    if (en)
    {
        hflash->Instance->CR |= MPI_CR_CMD2E ;
    }
    else
    {
        hflash->Instance->CR &= ~(MPI_CR_CMD2E);
    }

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_FLASH_STAUS_MATCH_CMD2(FLASH_HandleTypeDef *hflash, uint8_t en)
{
    if (hflash == NULL)
        return HAL_ERROR;

    if (en)
    {
        hflash->Instance->CR |=  MPI_CR_SME2;
    }
    else
    {
        hflash->Instance->CR &= ~(MPI_CR_SME2);
    }

    return HAL_OK;
}


__HAL_ROM_USED HAL_StatusTypeDef HAL_FLASH_SET_CS_TIME(FLASH_HandleTypeDef *hflash,
        uint16_t cslmin, uint16_t cslmax, uint16_t cshmin, uint16_t trcmin)
{
    uint32_t val;

    if (hflash == NULL)
        return HAL_ERROR;

    val = hflash->Instance->DCR;
    val &= ~(MPI_DCR_CSLMAX | MPI_DCR_CSLMIN | MPI_DCR_CSHMIN | MPI_DCR_TRCMIN);
    val |= (cslmax << MPI_DCR_CSLMAX_Pos) | (cslmin << MPI_DCR_CSLMIN_Pos) | (cshmin << MPI_DCR_CSHMIN_Pos) | (trcmin << MPI_DCR_TRCMIN_Pos);
    hflash->Instance->DCR = val;

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_FLASH_SET_ROW_BOUNDARY(FLASH_HandleTypeDef *hflash, uint8_t rb_size)
{
    if (hflash == NULL)
        return HAL_ERROR;

    MODIFY_REG(hflash->Instance->DCR, MPI_DCR_RBSIZE_Msk, MAKE_REG_VAL(rb_size, MPI_DCR_RBSIZE_Msk, MPI_DCR_RBSIZE_Pos));

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_FLASH_SET_LEGACY(FLASH_HandleTypeDef *hflash, uint8_t en)
{
    if (hflash == NULL)
        return HAL_ERROR;

    if (en)
        hflash->Instance->DCR |= MPI_DCR_XLEGACY;
    else
        hflash->Instance->DCR &= ~MPI_DCR_XLEGACY;

    return HAL_OK;
}


__HAL_ROM_USED HAL_StatusTypeDef HAL_FLASH_SET_DUAL_MODE(FLASH_HandleTypeDef *hflash, uint8_t dual)
{
    if (hflash == NULL)
        return HAL_ERROR;

    if (dual)
        hflash->Instance->CR |= MPI_CR_DFM;
    else
        hflash->Instance->CR &= ~MPI_CR_DFM;

    return HAL_OK;
}

__HAL_ROM_USED int HAL_FLASH_GET_DUAL_MODE(FLASH_HandleTypeDef *hflash)
{
    if (hflash == NULL)
        return 0;

    if (hflash->Instance->CR & MPI_CR_DFM)
        return 1;

    return 0;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_FLASH_SET_X16_MODE(FLASH_HandleTypeDef *hflash, uint8_t en)
{
    if (hflash == NULL)
        return HAL_ERROR;

    if (en)
        hflash->Instance->CR |= MPI_CR_MX16;
    else
        hflash->Instance->CR &= ~MPI_CR_MX16;

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_MPI_EN_FIXLAT(FLASH_HandleTypeDef *hflash, uint8_t fix)
{
    if (hflash == NULL)
        return HAL_ERROR;

    if (fix)
        hflash->Instance->DCR |= MPI_DCR_FIXLAT;
    else
        hflash->Instance->DCR &= ~MPI_DCR_FIXLAT;

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_MPI_SET_HYPER(FLASH_HandleTypeDef *hflash, uint8_t hyper)
{
    if (hflash == NULL)
        return HAL_ERROR;

    if (hyper)
        hflash->Instance->DCR |= MPI_DCR_HYPER;
    else
        hflash->Instance->DCR &= ~MPI_DCR_HYPER;

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_MPI_ENABLE_DQS(FLASH_HandleTypeDef *hflash, uint8_t dqs)
{
    if (hflash == NULL)
        return HAL_ERROR;

    if (dqs)
        hflash->Instance->DCR |= MPI_DCR_DQSE;
    else
        hflash->Instance->DCR &= ~MPI_DCR_DQSE;

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_MPI_SET_DQS_DELAY(FLASH_HandleTypeDef *hflash, uint8_t delay)
{
    if (hflash == NULL)
        return HAL_ERROR;

    uint32_t val = hflash->Instance->MISCR;
    val &= ~MPI_MISCR_DQSDLY_Msk;
    val |= (delay << MPI_MISCR_DQSDLY_Pos);
    hflash->Instance->MISCR = val;

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_MPI_SET_SCK(FLASH_HandleTypeDef *hflash, uint8_t delay, uint8_t inv)
{
    if (hflash == NULL)
        return HAL_ERROR;

    uint32_t val = hflash->Instance->MISCR;
    val &= ~(MPI_MISCR_SCKINV_Msk | MPI_MISCR_SCKDLY_Msk);
    val |= (delay << MPI_MISCR_SCKDLY_Pos) | (inv << MPI_MISCR_SCKINV_Pos);
    hflash->Instance->MISCR = val;

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_MPI_CFG_DTR(FLASH_HandleTypeDef *hflash, uint8_t en, uint8_t rclk_inv_delay)
{
    uint32_t value;
    if (hflash == NULL)
        return HAL_ERROR;

    value = hflash->Instance->MISCR;
    if (en)
    {
        value |= (MPI_MISCR_DTRPRE);
        value &= (~MPI_MISCR_RXCLKDLY);
        value |= (0xa << MPI_MISCR_RXCLKDLY_Pos);   // todo, auto calibrate or manual set ? 4 ~ F
        value &= (~MPI_MISCR_SCKDLY);
        //value |= ((0xa & 0x7f) << MPI_MISCR_SCKDLY_Pos);
        rclk_inv_delay = rclk_inv_delay > 2 ? rclk_inv_delay - 2 : rclk_inv_delay;
        value |= ((rclk_inv_delay & 0x7f) << MPI_MISCR_SCKDLY_Pos);
        if (rclk_inv_delay & 0x80)
            value |= (MPI_MISCR_RXCLKINV);

        value &= ~(MPI_MISCR_SCKINV);
    }
    else
    {
        value &= ~(MPI_MISCR_DTRPRE | MPI_MISCR_RXCLKINV | MPI_MISCR_RXCLKDLY | MPI_MISCR_SCKDLY);
        value |= (MPI_MISCR_SCKINV);
    }
    hflash->Instance->MISCR = value;

    return HAL_OK;
}

__HAL_ROM_USED void HAL_MPI_MODIFY_RCMD_DELAY(FLASH_HandleTypeDef *hflash, int8_t dum_cyc)
{
    if (hflash == NULL)
        return;

    uint32_t value = hflash->Instance->HRCCR;

    value &= ~MPI_HRCCR_DCYC;
    value |= (dum_cyc << MPI_HRCCR_DCYC_Pos) ;

    hflash->Instance->HRCCR = value;
}

__HAL_ROM_USED void HAL_MPI_MODIFY_WCMD_DELAY(FLASH_HandleTypeDef *hflash, int8_t dum_cyc)
{
    if (hflash == NULL)
        return;

    uint32_t value = hflash->Instance->HWCCR;

    value &= ~MPI_HWCCR_DCYC;
    value |= (dum_cyc << MPI_HWCCR_DCYC_Pos) ;

    hflash->Instance->HWCCR = value;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_FLASH_SET_WDT(FLASH_HandleTypeDef *hflash, uint16_t timeout)
{
    if (hflash == NULL)
        return HAL_ERROR;

    uint32_t value = hflash->Instance->WDTR;

    if (timeout == 0)
        value = 0;
    else
    {
        value = timeout & MPI_WDTR_TIMEOUT_Msk;
#ifndef SF32LB58X
        value |= MPI_WDTR_EN;
#endif
    }

    hflash->Instance->WDTR = value;

    return HAL_OK;
}

__HAL_ROM_USED int HAL_FLASH_GET_WDT_VALUE(FLASH_HandleTypeDef *hflash)
{
    uint32_t value = 0;

    if (hflash == NULL)
        return 0;

    value = hflash->Instance->WDTR;

    return (value & MPI_WDTR_TIMEOUT_Msk);
}

__HAL_ROM_USED int HAL_FLASH_GET_WDT_STATUS(FLASH_HandleTypeDef *hflash)
{
    uint32_t value = 0;

    if (hflash == NULL)
        return 0;

    value = hflash->Instance->WDTR;
    if (value & MPI_WDTR_TOF_Msk)
        return 1;

    return 0;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_FLASH_SET_INTERVAL(FLASH_HandleTypeDef *hflash, uint32_t val12, uint32_t val21)
{
    if (hflash == NULL)
        return HAL_ERROR;
    uint32_t value = (val12 & MPI_CIR_INTERVAL1) | ((val21 << MPI_CIR_INTERVAL2_Pos) & MPI_CIR_INTERVAL2);

    hflash->Instance->CIR = value;

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_FLASH_DMA_EN(FLASH_HandleTypeDef *hflash, uint8_t en)
{
    if (hflash == NULL)
        return HAL_ERROR;

    if (en)
        hflash->Instance->CR |= MPI_CR_DMAE;
    else
        hflash->Instance->CR &= ~MPI_CR_DMAE;

    return HAL_OK;
}

#if defined(SF32LB56X) || defined(SF32LB52X)

__HAL_ROM_USED HAL_StatusTypeDef HAL_FLASH_SET_LOOP(FLASH_HandleTypeDef *hflash, uint32_t loop)
{
    if ((hflash == NULL) || (loop > MPI_CR2_LOOP))
        return HAL_ERROR;

    hflash->Instance->CR2 = loop & MPI_CR2_LOOP;

    return HAL_OK;
}
#endif

__HAL_ROM_USED HAL_StatusTypeDef HAL_FLASH_CONFIG_AHB_READ(FLASH_HandleTypeDef *hflash, bool qmode)
{
    if (hflash == NULL)
        return HAL_ERROR;

    if (qmode)
    {
        if (hflash->ctable->cmd_cfg[SPI_FLASH_CMD_4READ].cmd == 0)
            return HAL_ERROR;
        HAL_FLASH_CFG_AHB_RCMD(hflash, hflash->ctable->cmd_cfg[SPI_FLASH_CMD_4READ].data_mode,
                               hflash->ctable->cmd_cfg[SPI_FLASH_CMD_4READ].dummy_cycle, hflash->ctable->cmd_cfg[SPI_FLASH_CMD_4READ].ab_size,
                               hflash->ctable->cmd_cfg[SPI_FLASH_CMD_4READ].ab_mode, hflash->ctable->cmd_cfg[SPI_FLASH_CMD_4READ].addr_size,
                               hflash->ctable->cmd_cfg[SPI_FLASH_CMD_4READ].addr_mode, hflash->ctable->cmd_cfg[SPI_FLASH_CMD_4READ].ins_mode);
        HAL_FLASH_SET_AHB_RCMD(hflash, hflash->ctable->cmd_cfg[SPI_FLASH_CMD_4READ].cmd);
    }
    else
    {
        if (hflash->ctable->cmd_cfg[SPI_FLASH_CMD_FREAD].cmd == 0)
            return HAL_ERROR;
        HAL_FLASH_CFG_AHB_RCMD(hflash, hflash->ctable->cmd_cfg[SPI_FLASH_CMD_FREAD].data_mode,
                               hflash->ctable->cmd_cfg[SPI_FLASH_CMD_FREAD].dummy_cycle, hflash->ctable->cmd_cfg[SPI_FLASH_CMD_FREAD].ab_size,
                               hflash->ctable->cmd_cfg[SPI_FLASH_CMD_FREAD].ab_mode, hflash->ctable->cmd_cfg[SPI_FLASH_CMD_FREAD].addr_size,
                               hflash->ctable->cmd_cfg[SPI_FLASH_CMD_FREAD].addr_mode, hflash->ctable->cmd_cfg[SPI_FLASH_CMD_FREAD].ins_mode);
        HAL_FLASH_SET_AHB_RCMD(hflash, hflash->ctable->cmd_cfg[SPI_FLASH_CMD_FREAD].cmd);
    }

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_FLASH_CONFIG_FULL_AHB_READ(FLASH_HandleTypeDef *hflash, bool qmode)
{
    if (hflash == NULL)
        return HAL_ERROR;

    if (qmode)
    {
        HAL_FLASH_CFG_AHB_RCMD(hflash, hflash->ctable->cmd_cfg[SPI_FLASH_CMD_4RD4BA].data_mode,
                               hflash->ctable->cmd_cfg[SPI_FLASH_CMD_4RD4BA].dummy_cycle, hflash->ctable->cmd_cfg[SPI_FLASH_CMD_4RD4BA].ab_size,
                               hflash->ctable->cmd_cfg[SPI_FLASH_CMD_4RD4BA].ab_mode, hflash->ctable->cmd_cfg[SPI_FLASH_CMD_4RD4BA].addr_size,
                               hflash->ctable->cmd_cfg[SPI_FLASH_CMD_4RD4BA].addr_mode, hflash->ctable->cmd_cfg[SPI_FLASH_CMD_4RD4BA].ins_mode);
        HAL_FLASH_SET_AHB_RCMD(hflash, hflash->ctable->cmd_cfg[SPI_FLASH_CMD_4RD4BA].cmd);
    }
    else
    {
        HAL_FLASH_CFG_AHB_RCMD(hflash, hflash->ctable->cmd_cfg[SPI_FLASH_CMD_FR4BA].data_mode,
                               hflash->ctable->cmd_cfg[SPI_FLASH_CMD_FR4BA].dummy_cycle, hflash->ctable->cmd_cfg[SPI_FLASH_CMD_FR4BA].ab_size,
                               hflash->ctable->cmd_cfg[SPI_FLASH_CMD_FR4BA].ab_mode, hflash->ctable->cmd_cfg[SPI_FLASH_CMD_FR4BA].addr_size,
                               hflash->ctable->cmd_cfg[SPI_FLASH_CMD_FR4BA].addr_mode, hflash->ctable->cmd_cfg[SPI_FLASH_CMD_FR4BA].ins_mode);
        HAL_FLASH_SET_AHB_RCMD(hflash, hflash->ctable->cmd_cfg[SPI_FLASH_CMD_FR4BA].cmd);
    }

    return HAL_OK;
}

__HAL_ROM_USED int HAL_FLASH_PRE_CMD(FLASH_HandleTypeDef *handle, SPI_FLASH_CMD_E cmd)
{
    if ((handle->ctable == NULL) || (cmd >= SPI_FLASH_CMD_COUNT)
            || (handle->ctable->cmd_cfg[cmd].cmd == 0))
    {
        return 1;
    }
    // get command configure
    HAL_FLASH_MANUAL_CMD(handle, handle->ctable->cmd_cfg[cmd].func_mode, handle->ctable->cmd_cfg[cmd].data_mode,
                         handle->ctable->cmd_cfg[cmd].dummy_cycle, handle->ctable->cmd_cfg[cmd].ab_size,
                         handle->ctable->cmd_cfg[cmd].ab_mode, handle->ctable->cmd_cfg[cmd].addr_size,
                         handle->ctable->cmd_cfg[cmd].addr_mode, handle->ctable->cmd_cfg[cmd].ins_mode);

    return 0;
}

__HAL_ROM_USED int HAL_FLASH_ISSUE_CMD(FLASH_HandleTypeDef *handle, SPI_FLASH_CMD_E cmd, uint32_t addr)
{
    HAL_StatusTypeDef res;
    if ((handle->ctable == NULL) || (cmd >= SPI_FLASH_CMD_COUNT)
            || (handle->ctable->cmd_cfg[cmd].cmd == 0))
    {
        return 1;
    }
    // get command configure
    HAL_FLASH_MANUAL_CMD(handle, handle->ctable->cmd_cfg[cmd].func_mode, handle->ctable->cmd_cfg[cmd].data_mode,
                         handle->ctable->cmd_cfg[cmd].dummy_cycle, handle->ctable->cmd_cfg[cmd].ab_size,
                         handle->ctable->cmd_cfg[cmd].ab_mode, handle->ctable->cmd_cfg[cmd].addr_size,
                         handle->ctable->cmd_cfg[cmd].addr_mode, handle->ctable->cmd_cfg[cmd].ins_mode);

    res = HAL_FLASH_SET_CMD(handle, handle->ctable->cmd_cfg[cmd].cmd, addr);
    if (res != HAL_OK)
        return 1;

    return 0;
}

__HAL_ROM_USED int HAL_FLASH_ISSUE_CMD_SEQ(FLASH_HandleTypeDef *handle, SPI_FLASH_CMD_E cmd1, uint32_t addr, SPI_FLASH_CMD_E cmd2, uint32_t param)
{
    //HAL_StatusTypeDef res;
    if ((handle->ctable == NULL) || (cmd1 >= SPI_FLASH_CMD_COUNT) || (handle->ctable->cmd_cfg[cmd1].cmd == 0)
            || (cmd2 >= SPI_FLASH_CMD_COUNT) || (handle->ctable->cmd_cfg[cmd2].cmd == 0))
    {
        //LOG_E("Error nand_issue_cmd param\n");
        return 1;
    }
    // get command configure
    HAL_FLASH_MANUAL_CMD(handle, handle->ctable->cmd_cfg[cmd1].func_mode, handle->ctable->cmd_cfg[cmd1].data_mode,
                         handle->ctable->cmd_cfg[cmd1].dummy_cycle, handle->ctable->cmd_cfg[cmd1].ab_size,
                         handle->ctable->cmd_cfg[cmd1].ab_mode, handle->ctable->cmd_cfg[cmd1].addr_size,
                         handle->ctable->cmd_cfg[cmd1].addr_mode, handle->ctable->cmd_cfg[cmd1].ins_mode);
    HAL_FLASH_MANUAL_CMD2(handle, handle->ctable->cmd_cfg[cmd2].func_mode, handle->ctable->cmd_cfg[cmd2].data_mode,
                          handle->ctable->cmd_cfg[cmd2].dummy_cycle, handle->ctable->cmd_cfg[cmd2].ab_size,
                          handle->ctable->cmd_cfg[cmd2].ab_mode, handle->ctable->cmd_cfg[cmd2].addr_size,
                          handle->ctable->cmd_cfg[cmd2].addr_mode, handle->ctable->cmd_cfg[cmd2].ins_mode);

    //res = HAL_FLASH_SET_CMD(handle, handle->ctable->cmd_cfg[cmd1].cmd, addr);

    handle->Instance->SMR = 0;  // clear match status
    handle->Instance->CMDR2 = handle->ctable->cmd_cfg[cmd2].cmd & 0XFF;
    handle->Instance->SMKR = param; // set status mask
    //handle->Instance->CR |= QSPI_CR_CMD2E | QSPI_CR_SME2;
    HAL_FLASH_ENABLE_CMD2(handle, 1);
    HAL_FLASH_STAUS_MATCH_CMD2(handle, 1);

    handle->Instance->AR1 = addr;
    handle->Instance->CMDR1 = handle->ctable->cmd_cfg[cmd1].cmd & 0XFF;  // set cmd 1 to start

    // wait command 2 done
    //while (HAL_FLASH_IS_BUSY(handle));
    while (!HAL_FLASH_STATUS_MATCH(handle));

    HAL_FLASH_CLR_STATUS(handle, MPI_SCR_SMFC | MPI_SCR_TCFC);

    // disable command 2
    HAL_FLASH_ENABLE_CMD2(handle, 0);
    HAL_FLASH_STAUS_MATCH_CMD2(handle, 0);

    return 0;
}

__HAL_ROM_USED void nor_qspi_switch(FLASH_HandleTypeDef *hflash, bool en)
{
    //bool res;
    uint8_t srl, srh;
    uint32_t dlen;
    uint32_t value;
    int res;

    if ((hflash == NULL) || (hflash->ctable == NULL))
        return;

    if (en == 0) // for spi mode, defaut setting should work, do not need change
        return;

    //HAL_FLASH_CLEAR_FIFO(hflash, HAL_FLASH_CLR_RX_TX_FIFO);

    srl = 0;
    srh = 0;
    if ((hflash->ctable->cmd_cfg[SPI_FLASH_CMD_WRSR2].cmd != 0))
    {
        // get current status register value
        dlen = 1;
        dlen = dlen;
        HAL_FLASH_WRITE_DLEN(hflash, dlen);
        HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_RDSR2, 0);
        srh = (uint8_t)HAL_FLASH_READ32(hflash) & 0XFF;
#if 1
        if (srh & 0x1)  // SRP1 TO protect flash SR, it should never happened!!!
        {
            HAL_ASSERT(0);
        }
#endif
        // set/clear QE bit
        if (en)
        {
            //srh |= 2;   // set qe bit
            srh = 2;    // clear other bits to avoid error
        }
        else
        {
            //srh &= 0xfd;    // clear qe bit
            srh = 0;    // clear other bits to avoid error
        }

        value = (uint32_t)srh ;

        res = HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_WVSR, 0);
        if (res != 0)   // no SPI_FLASH_CMD_WVSR support, need write en !
            HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_WREN, 0);
        HAL_FLASH_WRITE_WORD(hflash, value);

        HAL_FLASH_WRITE_DLEN(hflash, dlen);
        //HAL_FLASH_SET_CMD(hflash, FCMD_WVSR, 0);
        //HAL_FLASH_SET_CMD(hflash, FCMD_WRSR, 0);
        HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_WRSR2, 0);
        if (res != 0)   // no SPI_FLASH_CMD_WVSR support, WREN need check WIP bit
        {
            HAL_FLASH_WRITE_DLEN(hflash, dlen);
            do
            {
                HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_RDSR, 0);
                res = HAL_FLASH_IS_PROG_DONE(hflash);
            }
            while (!res);
        }
    }
    else
    {
        dlen = 1;
        dlen = dlen;
        // get current status register value
        HAL_FLASH_WRITE_DLEN(hflash, dlen);
        //HAL_FLASH_SET_CMD(hflash, FCMD_RDSR, 0);
        HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_RDSR, 0);
        srl = (uint8_t)HAL_FLASH_READ32(hflash) & 0XFF;

        //HAL_FLASH_SET_CMD(hflash, FCMD_RDSR2, 0);
        res = HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_RDSR2, 0);
        if (res == 0)
            srh = (uint8_t)HAL_FLASH_READ32(hflash) & 0XFF;

        // set/clear QE bit ，for word access, it 2 bytes should merge to 1 word
        if (en)
        {
            if (hflash->ctable->qe_mod_mask != 0)
            {
                if ((hflash->ctable->qe_mod_mask & 0xf) == 0) // at register 1
                {
                    //srl |= (1 << (hflash->ctable->qe_mod_mask >> 4));
                    srl = (1 << (hflash->ctable->qe_mod_mask >> 4));
                    srh = 0;
                }
                else // at register 2
                {
                    //srh |= (1 << (hflash->ctable->qe_mod_mask >> 4));
                    srh = (1 << (hflash->ctable->qe_mod_mask >> 4));
                    srl = 0;
                }
                // todo , qe bit not on register 1 and register 2, not support
            }
            else // default qe bit at bit 2 register 2 (s9)
            {
                //srh |= 2;   // set qe bit
                srh = 2;    // clear other bits to avoid error
                srl = 0;
            }
        }
        else
        {
            if (hflash->ctable->qe_mod_mask != 0)
            {
                if ((hflash->ctable->qe_mod_mask & 0xf) == 0) // at register 1
                    srl &= ~(1 << (hflash->ctable->qe_mod_mask >> 4));
                else // at register 2
                    srh &= ~(1 << (hflash->ctable->qe_mod_mask >> 4));
                // todo , qe bit not on register 1 and register 2, not support
            }
            else
            {
                //srh &= 0xfd;    // clear qe bit
                srh = 0;    // clear other bits to avoid error
                srl = 0;
            }
            srh = 0;    // clear all sr bits to avoid error
            srl = 0;
        }


        value = ((uint32_t)srh << 8) | (uint32_t)srl;

        res = HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_WVSR, 0);

        if (res != 0)   // no SPI_FLASH_CMD_WVSR support, need write en !
            HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_WREN, 0);

        HAL_FLASH_WRITE_WORD(hflash, value);

        dlen *= 2;
        HAL_FLASH_WRITE_DLEN(hflash, dlen);

        //HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_WREN, 0);
        HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_WRSR, 0);
        if (res != 0)   // no SPI_FLASH_CMD_WVSR support, WREN need check WIP bit
        {
            HAL_FLASH_WRITE_DLEN(hflash, 1);
            do
            {
                HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_RDSR, 0);
                res = HAL_FLASH_IS_PROG_DONE(hflash);
            }
            while (!res);
        }
    }

}

__HAL_ROM_USED void HAL_FLASH_SET_QUAL_SPI(FLASH_HandleTypeDef *hflash, bool en)
{
    nor_qspi_switch(hflash, en);

    HAL_FLASH_CONFIG_AHB_READ(hflash, en);
}

__HAL_ROM_USED void HAL_FLASH_FADDR_SET_QSPI(FLASH_HandleTypeDef *hflash, bool en)
{
    nor_qspi_switch(hflash, en);

    HAL_FLASH_CONFIG_FULL_AHB_READ(hflash, en);
}

__HAL_ROM_USED int HAL_FLASH_GET_NOR_ID(FLASH_HandleTypeDef *hflash)
{
    uint32_t id;
    uint32_t dlen = 0;

    //HAL_FLASH_CLEAR_FIFO(hflash, HAL_FLASH_CLR_RX_FIFO);

    // configure CCR
    //HAL_FLASH_MANUAL_CMD(hflash, 0, 1, 0, 0, 0, 2, 1, 1);
    HAL_FLASH_MANUAL_CMD(hflash, 0, 1, 0, 0, 0, 0, 0, 1);

    // configure data length, 2 byte, manufacturer id and device id, 0X90
    // configure data length, 3 byte, manufacturer id , memory type id and capacity id, 0X9F
    dlen = 3;
    HAL_FLASH_WRITE_DLEN(hflash, dlen);

    // send command, read id command
    //HAL_FLASH_SET_CMD(hflash, 0x90, 0);
    HAL_FLASH_SET_CMD(hflash, 0x9f, 0);

    //while (HAL_FLASH_IS_RX_EMPTY(hflash));

    id = HAL_FLASH_READ32(hflash);

    return (int)(id & 0xffffff);
}

__HAL_ROM_USED int HAL_FLASH_ID_DUAL_ID(FLASH_HandleTypeDef *hflash)
{
    uint32_t id, id2;;
    if (HAL_FLASH_GET_DUAL_MODE(hflash) == 0)
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
    if ((id == 0xffffffff) || (id == 0x00)) // invalid id
        return 0;

    id2 = HAL_FLASH_READ32(hflash);

    if (((id & 0xff) == ((id & 0xff00) >> 8)) && (((id & 0xff0000) >> 16) == ((id & 0xff000000) >> 24))
            && ((id2 & 0xff) == ((id2 & 0xff00) >> 8)))
        return 1;

    return 0;
}

__HAL_ROM_USED void HAL_FLASH_CLR_PROTECT(FLASH_HandleTypeDef *hflash)
{
    uint8_t srl, srh;
    uint32_t dlen;
    uint32_t value;
    int res;

    if (hflash == NULL)
        return;

    srl = 0;
    srh = 0;

    if ((hflash->ctable->cmd_cfg[SPI_FLASH_CMD_WRSR2].cmd != 0))
    {
        // WRSR only work for low 8 bits
        dlen = 1;
        dlen = dlen;;
        HAL_FLASH_WRITE_DLEN(hflash, dlen);
        res = HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_RDSR, 0);
        if (res == 0)
            srl = (uint8_t)HAL_FLASH_READ32(hflash) & 0XFF;

        if (hflash->ctable->ecc_sta_mask != 0)
            value = (~hflash->ctable->ecc_sta_mask) & srl;
        else
            value = 0;

        res = HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_WVSR, 0);
        if (res != 0)   // no SPI_FLASH_CMD_WVSR support, need write en !
            HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_WREN, 0);

        HAL_FLASH_WRITE_WORD(hflash, value);
        HAL_FLASH_WRITE_DLEN(hflash, dlen);

        HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_WRSR, 0);
        if (res != 0)   // no SPI_FLASH_CMD_WVSR support, WREN need check WIP bit
        {
            HAL_FLASH_WRITE_DLEN(hflash, 1);
            do
            {
                HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_RDSR, 0);
                res = HAL_FLASH_IS_PROG_DONE(hflash);
            }
            while (!res);
        }
    }
    else
    {
        // WRSR work on low 8 bits and high 8 bits, need read all back
        dlen = 1;
        dlen = dlen;
        // get current status register value
        HAL_FLASH_WRITE_DLEN(hflash, dlen);
        res = HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_RDSR, 0);
        if (res == 0)
            srl = (uint8_t)HAL_FLASH_READ32(hflash) & 0XFF;

        res = HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_RDSR2, 0);
        if (res == 0)
            srh = (uint8_t)HAL_FLASH_READ32(hflash) & 0XFF;

        if (hflash->ctable->ecc_sta_mask != 0)
            srl &= (~hflash->ctable->ecc_sta_mask);
        else
            srl = 0; // clear low 8 bit to 0 to remove protect bits

        value = ((uint32_t)srh << 8) | (uint32_t)srl;

        res = HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_WVSR, 0);
        if (res != 0)   // no SPI_FLASH_CMD_WVSR support, need write en !
            HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_WREN, 0);

        HAL_FLASH_WRITE_WORD(hflash, value);

        dlen *= 2;  // write 2 byte status register
        HAL_FLASH_WRITE_DLEN(hflash, dlen);

        //HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_WREN, 0);
        HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_WRSR, 0);
        if (res != 0)   // no SPI_FLASH_CMD_WVSR support, WREN need check WIP bit
        {
            HAL_FLASH_WRITE_DLEN(hflash, 1);
            do
            {
                HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_RDSR, 0);
                res = HAL_FLASH_IS_PROG_DONE(hflash);
            }
            while (!res);
        }
    }

}

__HAL_ROM_USED void HAL_QSPI_SET_RXDELAY(uint8_t qspi_id, uint8_t clk_inv, uint8_t clk_delay)
{
    MPI_TypeDef *handle = NULL;
    uint32_t value;

    switch (qspi_id)
    {
    case 0:
        handle = FLASH1;
        break;
    case 1:
        handle = FLASH2;
        break;
#ifdef FLASH3
    case 2:
        handle = FLASH3;
        break;
#endif
#ifdef FLASH4
    case 3:
        handle = FLASH4;
        break;
#endif
#ifdef FLASH5
    case 4:
        handle = FLASH5;
        break;
#endif
    default:
        return;
        break;
    }

    if (handle != NULL)
    {
        value = handle->MISCR & (~(MPI_MISCR_RXCLKDLY | MPI_MISCR_RXCLKINV));
        value |= ((clk_delay << MPI_MISCR_RXCLKDLY_Pos) & MPI_MISCR_RXCLKDLY_Msk)
                 | ((clk_inv << MPI_MISCR_RXCLKINV_Pos)& MPI_MISCR_RXCLKINV_Msk);
        handle->MISCR = value;
    }
}

__HAL_ROM_USED void HAL_QSPI_SET_CLK_INV(FLASH_HandleTypeDef *hflash, uint8_t clk_inv, uint8_t clk_delay)
{
    uint32_t value;
    if (hflash == NULL)
        return;

    if (hflash->Instance != NULL)
    {
        value = hflash->Instance->MISCR & (~(MPI_MISCR_RXCLKDLY | MPI_MISCR_RXCLKINV));
        value |= ((clk_delay << MPI_MISCR_RXCLKDLY_Pos) & MPI_MISCR_RXCLKDLY_Msk)
                 | ((clk_inv << MPI_MISCR_RXCLKINV_Pos)& MPI_MISCR_RXCLKINV_Msk);
        hflash->Instance->MISCR = value;
    }
}


//extern void rt_kprintf(const char * fmt, ...);
__HAL_ROM_USED int HAL_QSPI_LOCK_OTP(FLASH_HandleTypeDef *hflash, uint32_t addr)
{
    uint32_t srh, srl, value;
    uint32_t dlen;
    int res, opbit;

    if (hflash == NULL || hflash->ctable == NULL)
        return -1;
    if (addr < SPI_FLASH_OTP_BASE || addr > SPI_FLASH_OTP_BASE + (hflash->ctable->mode_reg << 12))
        return -1;

    HAL_FLASH_CLEAR_FIFO(hflash, HAL_FLASH_CLR_RX_TX_FIFO);
    // get LB bits to check if OTP LOCKED (S11 ~ S13)
    srh = srl = value = 0;
    dlen = 1;
    HAL_FLASH_WRITE_DLEN(hflash, dlen);

    res = HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_RDSR2, 0);
    if (res == 0)   // support rdsr2, only need status register 2 value
    {
        srh = HAL_FLASH_READ32(hflash);
        srh &= 0xff;
    }
    else // not support rdsr2, rdsr with 2 bytes each
    {
        dlen *= 2;
        HAL_FLASH_WRITE_DLEN(hflash, dlen);
        HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_RDSR, 0);
        srh = HAL_FLASH_READ32(hflash) ;
        srl = srh & 0xff;

        srh = (srh >> 8) & 0xff;
    }
    opbit = addr >> 12;
    if (opbit < 1 || opbit > hflash->ctable->mode_reg)
        return -1;
    opbit = 1 << (opbit - 1);
    srh = (srh & 0xc7) | (opbit << 3);

    if ((hflash->ctable->cmd_cfg[SPI_FLASH_CMD_WRSR2].cmd != 0))
    {
        value = (uint32_t)srh ;

        HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_WREN, 0);
        HAL_FLASH_WRITE_WORD(hflash, value);

        dlen = 1;
        HAL_FLASH_WRITE_DLEN(hflash, dlen);

        HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_WRSR2, 0);

        HAL_FLASH_WRITE_DLEN(hflash, dlen);
        do
        {
            HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_RDSR, 0);
            res = HAL_FLASH_IS_PROG_DONE(hflash);
        }
        while (!res);
    }
    else
    {
        dlen = 1;
        if (res == 0)   // support rdsr2, srl not get from sr1
        {
            HAL_FLASH_WRITE_DLEN(hflash, dlen);
            HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_RDSR, 0);
            srl = HAL_FLASH_READ32(hflash) & 0xff;
        }

        value = ((uint32_t)srh << 8) | (uint32_t)srl;

        HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_WREN, 0);

        HAL_FLASH_WRITE_WORD(hflash, value);
        dlen *= 2;
        HAL_FLASH_WRITE_DLEN(hflash, dlen);

        HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_WRSR, 0);

        HAL_FLASH_WRITE_DLEN(hflash, 1);
        do
        {
            HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_RDSR, 0);
            res = HAL_FLASH_IS_PROG_DONE(hflash);
        }
        while (!res);
    }

    return 0;
}

#define MPI_MAX_FIFO       (64)
__HAL_ROM_USED int HAL_QSPI_READ_OTP(FLASH_HandleTypeDef *hflash, uint32_t addr, uint8_t *buf, uint32_t size)
{
    if (hflash == NULL || hflash->ctable == NULL)
    {
        //rt_kprintf("Invalid parameter\n");
        return 0;
    }
    if (addr < SPI_FLASH_OTP_BASE || addr > SPI_FLASH_OTP_BASE + (hflash->ctable->mode_reg << 12))
    {
        //rt_kprintf("addr 0x%x, range 0x%x\n",SPI_FLASH_OTP_BASE+(hflash->ctable->mode_reg<<12));
        return 0;
    }

    if ((addr & 0x3ff) + size > hflash->ctable->oob_size * 256)
    {
        //rt_kprintf("size 0x%x, max 0x%x\n",(addr & 0x3ff)+size, hflash->ctable->oob_size *256);
        return 0;
    }

    uint32_t *data = (uint32_t *)buf;
    int remain = size;
    int dlen = 0;
    int cnt = 0;
    int i;
    uint32_t temp_addr = addr;
    while (remain > 0)
    {
        dlen = remain > MPI_MAX_FIFO ? MPI_MAX_FIFO : remain;
        HAL_FLASH_WRITE_DLEN(hflash, dlen);
        HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_RDSCUR, temp_addr);
        for (i = 0; i < dlen / 4; i++)
            data[i + cnt / 4] = HAL_FLASH_READ32(hflash);
        cnt += dlen;
        temp_addr += dlen;
        remain -= dlen;
    }

    return size;
}

__HAL_ROM_USED uint8_t HAL_QSPI_GET_OTP_LB(FLASH_HandleTypeDef *hflash, uint32_t addr)
{
    uint32_t srh;
    uint32_t dlen;
    int res;

    if (hflash == NULL || hflash->ctable == NULL)
        return 0xff;
    if (addr < SPI_FLASH_OTP_BASE || addr > SPI_FLASH_OTP_BASE + (hflash->ctable->mode_reg << 12))
        return 0xff;

    // get LB bits to check if OTP LOCKED (S11 ~ S13)
    srh = 0;
    dlen = 1;
    HAL_FLASH_WRITE_DLEN(hflash, dlen);

    res = HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_RDSR2, 0);
    if (res == 0)   // support rdsr2, only need status register 2 value
    {
        srh = HAL_FLASH_READ32(hflash);
        srh &= 0xff;
    }
    else // not support rdsr2, rdsr with 2 bytes each
    {
        dlen *= 2;
        HAL_FLASH_WRITE_DLEN(hflash, dlen);
        HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_RDSR, 0);
        srh = HAL_FLASH_READ32(hflash) ;
        srh = (srh >> 8) & 0xff;
    }

    return srh >> 3;
}

__HAL_ROM_USED int HAL_FLASH_DEEP_PWRDOWN(FLASH_HandleTypeDef *hflash)
{
    int res = 1;
    if (hflash == NULL || (hflash->Instance->CR & MPI_CR_EN) == 0)
        return res;

    //res = HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_DPD, 0);

    // hardcode DPD command configure and id
    HAL_FLASH_MANUAL_CMD(hflash, 0, 0, 0, 0, 0, 0, 0, 1);

    // send command release deep power-mode
    res = HAL_FLASH_SET_CMD(hflash, 0xb9, 0);

    // add a delay to make sure into deep power down mode -- tDP

    return res;
}

__HAL_ROM_USED int HAL_FLASH_RELEASE_DPD(FLASH_HandleTypeDef *hflash)
{
    int res = 1;
    uint32_t qspi_en = 0;
    if (hflash == NULL)
        return res;

    qspi_en = hflash->Instance->CR & MPI_CR_EN;
    if (qspi_en == 0)
        hflash->Instance->CR |= MPI_CR_EN;

    //res = HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_RDP, 0);

    // when calling this function, flash in sleep mode, flash handle context on flash, so hflash->ctable can not work !!!
    // hardcode rdp command configure and id
    HAL_FLASH_MANUAL_CMD(hflash, 0, 0, 0, 0, 0, 0, 0, 1);

    // send command release deep power-mode
    res = HAL_FLASH_SET_CMD(hflash, 0xab, 0);

    // add a delay to make sure release done -- tRES1

    if (qspi_en == 0)
        hflash->Instance->CR &= ~MPI_CR_EN;

    return res;
}

__HAL_ROM_USED int HAL_QSPI_GET_UID(FLASH_HandleTypeDef *hflash, uint32_t *uid, uint32_t len)
{
    int i, res;
    //uint16_t len = 16;

    if (hflash == NULL || uid == NULL)
        return -1;

    HAL_FLASH_WRITE_DLEN(hflash, len);    // unique id include 64 bits or 128 bits

    //res = HAL_FLASH_ISSUE_CMD(hflash, SPI_FLASH_CMD_RUID, 0);
    // ALL NOR FLASH should have same READ UID COMMAND TIMMING !!!
    HAL_FLASH_MANUAL_CMD(hflash, 0, 1, 8, 0, 0, 2, 1, 1);
    res = HAL_FLASH_SET_CMD(hflash, 0X4B, 0);
    if (res != 0)
        return -2;

    //uid[0] = HAL_FLASH_READ32(hflash);
    //uid[1] = HAL_FLASH_READ32(hflash);
    //uid[2] = HAL_FLASH_READ32(hflash);
    //uid[3] = HAL_FLASH_READ32(hflash);
    for (i = 0; i < (int)len / 4; i++)
        uid[i] = HAL_FLASH_READ32(hflash);

    return 0;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_MPI_CFG_PREFETCH(FLASH_HandleTypeDef *hflash, uint32_t start, uint32_t len)
{
    if (hflash == NULL)
        return HAL_ERROR;

    if (((start & 0x3ff) != 0) || ((len & 0x3ff) != 0))
        return HAL_ERROR;

    hflash->Instance->PRSAR = start;
    hflash->Instance->PREAR = start + len;

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_MPI_EN_PREFETCH(FLASH_HandleTypeDef *hflash, uint8_t en)
{
    if (hflash == NULL)
        return HAL_ERROR;

    if (en)
        hflash->Instance->CR |= MPI_CR_PREFE;
    else
        hflash->Instance->CR &= (~MPI_CR_PREFE);

    return HAL_OK;
}


#endif  // HAL_QSPI_MODULE_ENABLED

/// @} FLASH

/// @} BF0_HAL_Driver


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
