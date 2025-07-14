/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bf0_hal.h"

/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @defgroup SDMMC_HAL SDMMC Low Layer
  * @brief Low layer module for SD
  * @{
  */
#if defined(HAL_SD_MODULE_ENABLED)||defined(_SIFLI_DOXYGEN_)

HAL_StatusTypeDef HAL_SDMMC_INIT(SD_TypeDef *hsd)
{
    if (hsd == NULL)
        return HAL_ERROR;

    //hsd->PCR = 1;
    hsd->DCR = 0x1ff << SD_DCR_BLOCK_SIZE_Pos;
    hsd->CLKCR |= SD_CLKCR_VOID_FIFO_ERROR;
    hsd->SR = 0X3FFFF;
    hsd->IER = 0;
#if defined(SF32LB52X)
    hsd->CDR &= ~SD_CDR_ITIMING_SEL;
#endif
    HAL_SDMMC_SET_TIMEOUT(hsd, HAL_SDMMC_DEFAULT_TIMEOUT);

    return HAL_OK;
}


HAL_StatusTypeDef HAL_SDMMC_GET_RESP(SD_TypeDef *hsd, uint32_t *resp)
{
    if (hsd == NULL || resp == NULL)
        return HAL_ERROR;

    resp[0] = hsd->RAR1;
    resp[1] = hsd->RAR2;
    resp[2] = hsd->RAR3;
    resp[3] = hsd->RAR4;

    return HAL_OK;
}

HAL_StatusTypeDef HAL_SDMMC_SET_TIMEOUT(SD_TypeDef *hsd, uint32_t timeout)
{
    if (hsd == NULL)
        return HAL_ERROR;

    hsd->TOR = timeout;

    return HAL_OK;
}

HAL_StatusTypeDef HAL_SDMMC_SET_DATALEN(SD_TypeDef *hsd, uint32_t len)
{
    if (hsd == NULL)
        return HAL_ERROR;

    hsd->DLR = len - 1;

    return HAL_OK;

}

HAL_StatusTypeDef HAL_SDMMC_CLR_DATA_CTRL(SD_TypeDef *hsd)
{
    if (hsd == NULL)
        return HAL_ERROR;

    //hsd->DCR = 0x1ff << SDMMC_DCR_BLOCK_SIZE_Pos;
    hsd->DCR = 0;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_SDMMC_SET_DATA_CTRL(SD_TypeDef *hsd, uint32_t blk_size,
        uint32_t direct, uint32_t wire, uint32_t mode)
{
    uint32_t value = 0;
    if (hsd == NULL)
        return HAL_ERROR;
    if (blk_size > 2048)
        return HAL_ERROR;

    value = direct | wire | mode | SD_DCR_TRAN_DATA_EN;
    value |= ((blk_size - 1) << SD_DCR_BLOCK_SIZE_Pos);
    hsd->DCR = value;

    return HAL_OK;
}

HAL_StatusTypeDef HAL_SDMMC_SET_WIRE(SD_TypeDef *hsd, uint32_t wire)
{
    uint32_t value = 0;
    if (hsd == NULL)
        return HAL_ERROR;

    value = hsd->DCR;
    value &= ~SD_DCR_WIRE_MODE;
    value |= wire;
    hsd->DCR = value;

    return HAL_OK;
}

HAL_StatusTypeDef HAL_SDMMC_SET_DIRECT(SD_TypeDef *hsd, uint32_t direct)
{
    uint32_t value = 0;
    if (hsd == NULL)
        return HAL_ERROR;

    value = hsd->DCR;
    value &= ~SD_DCR_R_WN;
    value |= direct;
    hsd->DCR = value;

    return HAL_OK;
}

HAL_StatusTypeDef HAL_SDMMC_SET_STREAM(SD_TypeDef *hsd, uint32_t stream)
{
    uint32_t value = 0;
    if (hsd == NULL)
        return HAL_ERROR;

    value = hsd->DCR;
    value &= ~SD_DCR_STREAM_MODE;
    value |= stream;
    hsd->DCR = value;

    return HAL_OK;
}

HAL_StatusTypeDef HAL_SDMMC_SET_BLOCK_SIZE(SD_TypeDef *hsd, uint32_t blk_size)
{
    uint32_t value = 0;
    if (hsd == NULL)
        return HAL_ERROR;
    if ((blk_size > 2048) || (blk_size <= 0))
        return HAL_ERROR;

    value = hsd->DCR;
    value &= ~SD_DCR_BLOCK_SIZE;
    value |= ((blk_size - 1) << SD_DCR_BLOCK_SIZE_Pos);
    hsd->DCR = value;

    return HAL_OK;

}

HAL_StatusTypeDef HAL_SDMMC_SET_DATA_EN(SD_TypeDef *hsd, uint32_t data_en)
{
    uint32_t value = 0;
    if (hsd == NULL)
        return HAL_ERROR;

    value = hsd->DCR;
    value &= ~SD_DCR_TRAN_DATA_EN;
    value |= data_en;
    hsd->DCR = value;

    return HAL_OK;
}

HAL_StatusTypeDef HAL_SDMMC_SET_DATA_START(SD_TypeDef *hsd, uint32_t start)
{
    uint32_t value = 0;
    if (hsd == NULL)
        return HAL_ERROR;

    value = hsd->DCR;
    if (start)
        value |= SD_DCR_DATA_START;
    else
        value &= ~SD_DCR_DATA_START;

    hsd->DCR = value;

    return HAL_OK;
}

HAL_StatusTypeDef HAL_SDMMC_SET_IRQ_MASK(SD_TypeDef *hsd, uint32_t mask)
{
    if (hsd == NULL)
        return HAL_ERROR;
    hsd->IER = mask;

    return HAL_OK;
}

uint32_t HAL_SDMMC_GET_IRQ_MASK(SD_TypeDef *hsd)
{

    if (hsd == NULL)
        return 0;

    return hsd->IER;
}

HAL_StatusTypeDef HAL_SDMMC_SET_CMD(SD_TypeDef *hsd, uint32_t cmd_idx,
                                    uint32_t resp, uint32_t arg)
{
    if (hsd == NULL)
        return HAL_ERROR;

    // clear all interrupt status before new command set
    uint32_t sr = SD_SR_CMD_DONE | SD_SR_CMD_RSP_CRC | SD_SR_CMD_TIMEOUT |
                  SD_SR_DATA_DONE | SD_SR_DATA_CRC | SD_SR_DATA_TIMEOUT |
                  SD_SR_STARTBIT_ERROR | SD_SR_FIFO_UNDERRUN | SD_SR_FIFO_OVERRUN |
                  SD_SR_CMD_SENT | SD_SR_CARD_EXIT_INT |
                  SD_SR_CARD_REMOVE | SD_SR_SDIO
#if !defined(SF32LB52X)&&!defined(SF32LB56X)
                  | SD_SR_CARD_DET
#endif
                  ;
    HAL_SDMMC_CLR_INT(hsd, sr);

    hsd->CAR = arg;

    uint32_t cmd = ((cmd_idx & 0x3f) << SD_CCR_CMD_INDEX_Pos) | ((resp & 0x3) << SD_CCR_CMD_HAS_RSP_Pos) |
                   SD_CCR_CMD_TX_EN | SD_CCR_CMD_START;

    hsd->CCR = cmd;

    return HAL_OK;
}

uint32_t HAL_SDMMC_GET_STA(SD_TypeDef *hsd)
{
    if (hsd == NULL)
        return 0;

    return hsd->SR;
}

HAL_StatusTypeDef HAL_SDMMC_CLR_INT(SD_TypeDef *hsd, uint32_t irq)
{
    if (hsd == NULL)
        return HAL_ERROR;

    hsd->SR = irq;

    return HAL_OK;
}

HAL_StatusTypeDef HAL_SDMMC_CLK_SET(SD_TypeDef *hsd, uint32_t div, uint8_t en)
{
    if (hsd == NULL)
        return HAL_ERROR;

    uint32_t clk = hsd->CLKCR;
    clk &= ~SD_CLKCR_DIV;
    if (div > 0)
        div -= 1;
    if (div > 0x1FFF)
        div = 0x1FFF;

    clk |= ((div & 0x1FFF) << SD_CLKCR_DIV_Pos);
    if (en)
        clk &= ~SD_CLKCR_STOP_CLK;
    else
        clk |= SD_CLKCR_STOP_CLK;
    hsd->CLKCR = clk;

    return HAL_OK;
}

HAL_StatusTypeDef HAL_SDMMC_POWER_SET(SD_TypeDef *hsd, uint32_t pow)
{
    if (hsd == NULL)
        return HAL_ERROR;

    uint32_t value = hsd->PCR;
    if (pow == 0)
        value &= ~SD_PCR_PWR;
    else
    {
        value &= ~SD_PCR_PWR;
        //value |= (1 << SD_PCR_PWR_Pos);  // only 1 bit used? why define 2 bits ??
        value |= ((pow & 0x3) << SD_PCR_PWR_Pos); // try use 2 bit
    }

    hsd->PCR = value;

    return HAL_OK;
}

HAL_StatusTypeDef HAL_SDMMC_WIRTE(SD_TypeDef *hsd, uint32_t *buf, uint32_t size)
{
    int i;
    if (hsd == NULL || buf == NULL)
        return HAL_ERROR;

    for (i = 0; i < (int)size / 4; i++)
        hsd->FIFO = *(buf + i);

    return HAL_OK;
}

HAL_StatusTypeDef HAL_SDMMC_READ(SD_TypeDef *hsd, uint32_t *buf, uint32_t size)
{
    int i;
    if (hsd == NULL || buf == NULL)
        return HAL_ERROR;

    for (i = 0; i < (int)size / 4; i++)
        *(buf + i) = hsd->FIFO;

    return HAL_OK;
}

uint32_t HAL_SDMMC_GET_RCI(SD_TypeDef *hsd)
{
    if (hsd == NULL)
        return HAL_ERROR;

    return (hsd->RIR) & 0x3f;
}

HAL_StatusTypeDef HAL_SDMMC_SWITCH_NORMAL(SD_TypeDef *hsd)
{
    if (hsd == NULL)
        return HAL_ERROR;

    // set sd request and wait sd busy
    hsd->CASR |= SD_CASR_SD_REQ;
    while ((hsd->CASR & SD_CASR_SD_BUSY) == 0);

    return HAL_OK;
}

HAL_StatusTypeDef HAL_SDMMC_SWITCH_AHB(SD_TypeDef *hsd)
{
    if (hsd == NULL)
        return HAL_ERROR;

    // clear sd busy and flush cache
    hsd->CASR |= (SD_CASR_SD_BUSY | SD_CASR_CACHE_FLUSH);

    return HAL_OK;
}

HAL_StatusTypeDef HAL_SDMMC_SELECT_VERSION(SD_TypeDef *hsd, uint8_t blk_mode)
{
    if (hsd == NULL)
        return HAL_ERROR;

    if (blk_mode) // card size larger than 2 GB, with block mode
    {
        hsd->CACR &= (~SD_CACR_CACHE_SDSC);
    }
    else // sdcard 1.0/1.1, size less than 2GB , with byte mode
    {
        hsd->CACR |= SD_CACR_CACHE_SDSC;
    }

    return HAL_OK;
}

HAL_StatusTypeDef HAL_SDMMC_CACHE_EN(SD_TypeDef *hsd, uint8_t en)
{
    if (hsd == NULL)
        return HAL_ERROR;

    if (en)
    {
        hsd->CACR |= SD_CACR_CACHE_EN;
    }
    else
    {
        hsd->CACR &= ~SD_CACR_CACHE_EN;
    }

    return HAL_OK;
}

HAL_StatusTypeDef HAL_SDMMC_CACHE_TO_EN(SD_TypeDef *hsd, uint8_t en)
{
    if (hsd == NULL)
        return HAL_ERROR;

    if (en)
    {
        hsd->CACR |= SD_CACR_CACHE_TO_EN;
    }
    else
    {
        hsd->CACR &= ~SD_CACR_CACHE_TO_EN;
    }

    return HAL_OK;
}

HAL_StatusTypeDef HAL_SDMMC_ENABLE_AHB_MAP(SD_TypeDef *hsd, uint8_t sd_map_en)
{
    if (sd_map_en)  // ahb space map for sd2
    {
        hwp_hpsys_cfg->SYSCR |= HPSYS_CFG_SYSCR_SDNAND;
    }
    else    // ahb space map for mpi3
    {
        hwp_hpsys_cfg->SYSCR &= ~HPSYS_CFG_SYSCR_SDNAND;
    }

    return HAL_OK;
}

HAL_StatusTypeDef HAL_SDMMC_SET_CAOFFSET(SD_TypeDef *hsd, uint32_t offset)
{
    if (hsd == NULL)
        return HAL_ERROR;

    hsd->CAOFF = offset;

    return HAL_OK;
}

HAL_StatusTypeDef HAL_SDMMC_VOID_FIFO(SD_TypeDef *hsd, uint8_t en)
{
    if (hsd == NULL)
        return HAL_ERROR;

    if (en)
        hsd->CLKCR |= SD_CLKCR_VOID_FIFO_ERROR;
    else
        hsd->CLKCR &= ~SD_CLKCR_VOID_FIFO_ERROR;

    return HAL_OK;
}


#endif
/**
  * @}
  */

/**
  * @}
  */

/**
  * @} file
  */