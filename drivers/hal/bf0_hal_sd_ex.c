/**
  ******************************************************************************
  * @file   bf0_hal_sd_ex.c
  * @author Sifli software development team
  * @brief SD/MMC HCI Controller BSP driver
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

#include "bf0_hal_sd_ex.h"
#include "bf0_hal.h"

#ifdef HAL_SDHCI_MODULE_ENABLED

#define SDEX_DEBUG      HAL_DBG_printf

/** @defgroup HAL_SDMMC_LL_Group4 Command management functions
 *  @brief   Data transfers functions
 *
@verbatim
 ===============================================================================
                   ##### Commands management functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to manage the needed commands.

@endverbatim
  * @{
  */

#define SDIO_CMDTIMEOUT                   1000U        /* Command send and response timeout */
#define SDIO_MAXERASETIMEOUT              63000U       /* Max erase Timeout 63 s            */


static uint32_t SDMMC_GetCmdError(SDHCI_HandleTypeDef *SDIOx);
static uint32_t SDMMC_GetCmdResp1(SDHCI_HandleTypeDef *SDIOx, uint8_t SD_CMD, uint32_t Timeout);
static uint32_t SDMMC_GetCmdResp2(SDHCI_HandleTypeDef *SDIOx);
static uint32_t SDMMC_GetCmdResp3(SDHCI_HandleTypeDef *SDIOx);
static uint32_t SDMMC_GetCmdResp6(SDHCI_HandleTypeDef *SDIOx, uint8_t SD_CMD, uint16_t *pRCA);
static uint32_t SDMMC_GetCmdResp7(SDHCI_HandleTypeDef *SDIOx);

/**
  * @brief  Initializes the SDMMC according to the specified
  *         parameters in the SDMMC_InitTypeDef and create the associated handle.
  * @param  SDIOx Pointer to SDMMC register base
  * @param  Init SDMMC initialization structure
  * @retval HAL status
  */
HAL_StatusTypeDef SDIO_Init(SDHCI_HandleTypeDef *SDIOx, SDIO_InitTypeDef Init)
{
    uint32_t tmpreg = 0U;


    /* Set SDMMC configuration parameters */
    tmpreg |= (Init.ClockEdge           | \
               Init.ClockBypass         | \
               Init.ClockPowerSave      | \
               Init.BusWide             | \
               Init.HardwareFlowControl | \
               Init.ClockDiv
              );

    /* Write to SDMMC CLKCR */
    //MODIFY_REG(SDIOx->CLKCR, CLKCR_CLEAR_MASK, tmpreg);

    hal_sdhci_set_timeout(SDIOx, 0xe);

    hal_sdhci_set_clk(SDIOx, 400000, 48000000);

    hal_sdhci_set_power(SDIOx, 18);

    hal_sdhci_set_bus_width(SDIOx, Init.BusWide);

    return HAL_OK;
}


/**
  * @}
  */

/** @defgroup HAL_SDMMC_LL_Group2 IO operation functions
 *  @brief   Data transfers functions
 *
@verbatim
 ===============================================================================
                      ##### I/O operation functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to manage the SDMMC data
    transfers.

@endverbatim
  * @{
  */

/**
  * @brief  Read data (word) from Rx FIFO in blocking mode (polling)
  * @param  SDIOx Pointer to SDMMC register base
  * @retval HAL status
  */
uint32_t SDIO_ReadFIFO(SDHCI_HandleTypeDef *SDIOx)
{
    /* Read data from Rx FIFO */
    return hal_sdhci_read_fifo(SDIOx);
}

/**
  * @brief  Write data (word) to Tx FIFO in blocking mode (polling)
  * @param  SDIOx Pointer to SDMMC register base
  * @param  pWriteData pointer to data to write
  * @retval HAL status
  */
HAL_StatusTypeDef SDIO_WriteFIFO(SDHCI_HandleTypeDef *SDIOx, uint32_t *pWriteData)
{
    /* Write data to FIFO */
    hal_sdhci_write_fifo(SDIOx, *pWriteData);

    return HAL_OK;
}

/**
  * @}
  */

/** @defgroup HAL_SDMMC_LL_Group3 Peripheral Control functions
 *  @brief   management functions
 *
@verbatim
 ===============================================================================
                      ##### Peripheral Control functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to control the SDMMC data
    transfers.

@endverbatim
  * @{
  */

/**
  * @brief  Set SDMMC Power state to ON.
  * @param  SDIOx Pointer to SDMMC register base
  * @retval HAL status
  */
HAL_StatusTypeDef SDIO_PowerState_ON(SDHCI_HandleTypeDef *SDIOx)
{
    /* Set power state to ON */
    //SDIOx->POWER = SDIO_POWER_PWRCTRL;

    return HAL_OK;
}

/**
  * @brief  Set SDMMC Power state to OFF.
  * @param  SDIOx Pointer to SDMMC register base
  * @retval HAL status
  */
HAL_StatusTypeDef SDIO_PowerState_OFF(SDHCI_HandleTypeDef *SDIOx)
{
    /* Set power state to OFF */
    //SDIOx->POWER = 0x00000000U;

    return HAL_OK;
}

/**
  * @brief  Get SDMMC Power state.
  * @param  SDIOx Pointer to SDMMC register base
  * @retval Power status of the controller. The returned value can be one of the
  *         following values:
  *            - 0x00: Power OFF
  *            - 0x02: Power UP
  *            - 0x03: Power ON
  */
uint32_t SDIO_GetPowerState(SDHCI_HandleTypeDef *SDIOx)
{
    return 3; //(SDIOx->POWER & SDIO_POWER_PWRCTRL);
}

/**
  * @brief  Configure the SDMMC command path according to the specified parameters in
  *         SDIO_CmdInitTypeDef structure and send the command
  * @param  SDIOx Pointer to SDMMC register base
  * @param  Command pointer to a SDIO_CmdInitTypeDef structure that contains
  *         the configuration information for the SDMMC command
  * @retval HAL status
  */
HAL_StatusTypeDef SDIO_SendCommand(SDHCI_HandleTypeDef *SDIOx, SDHCI_CmdArgTypeDef *Command)
{
    hal_sdhci_send_command(SDIOx, Command);

    return HAL_OK;
}



/**
  * @brief  Return the response received from the card for the last command
  * @param  SDIOx Pointer to SDMMC register base
  * @param  Response Specifies the SDMMC response register.
  *          This parameter can be one of the following values:
  *            @arg SDIO_RESP1: Response Register 1
  *            @arg SDIO_RESP1: Response Register 2
  *            @arg SDIO_RESP1: Response Register 3
  *            @arg SDIO_RESP1: Response Register 4
  * @retval The Corresponding response register value
  */
uint32_t SDIO_GetResponse(SDHCI_HandleTypeDef *SDIOx, uint32_t Response)
{
    return hal_sdhci_get_response(SDIOx, Response);
}

/**
  * @brief  Configure the SDMMC data path according to the specified
  *         parameters in the SDIO_DataInitTypeDef.
  * @param  SDIOx Pointer to SDMMC register base
  * @param  Data  pointer to a SDIO_DataInitTypeDef structure
  *         that contains the configuration information for the SDMMC data.
  * @retval HAL status
  */
HAL_StatusTypeDef SDIO_ConfigData(SDHCI_HandleTypeDef *SDIOx, SDIO_DataInitTypeDef *Data)
{
#if 0
    uint32_t tmpreg = 0U;

    /* Check the parameters */
    assert_param(IS_SDIO_DATA_LENGTH(Data->DataLength));
    assert_param(IS_SDIO_BLOCK_SIZE(Data->DataBlockSize));
    assert_param(IS_SDIO_TRANSFER_DIR(Data->TransferDir));
    assert_param(IS_SDIO_TRANSFER_MODE(Data->TransferMode));
    assert_param(IS_SDIO_DPSM(Data->DPSM));

    /* Set the SDMMC Data TimeOut value */
    SDIOx->DTIMER = Data->DataTimeOut;

    /* Set the SDMMC DataLength value */
    SDIOx->DLEN = Data->DataLength;

    /* Set the SDMMC data configuration parameters */
    tmpreg |= (uint32_t)(Data->DataBlockSize | \
                         Data->TransferDir   | \
                         Data->TransferMode  | \
                         Data->DPSM);

    /* Write to SDMMC DCTRL */
    MODIFY_REG(SDIOx->DCTRL, DCTRL_CLEAR_MASK, tmpreg);
#else
    hal_sdhci_set_timeout(SDIOx, 0xe);
    if (SDIOx->Init.flags & (SDHCI_USE_SDMA | SDHCI_USE_ADMA))
        SDIOx->Init.flags |= SDHCI_REQ_USE_DMA;

#if 0
    if (SDIOx->Init.flags & SDHCI_REQ_USE_DMA)
    {
        if (SDIOx->Init.flags & SDHCI_USE_ADMA)
        {
            int ret;
            ret = hal_sdhci_adma_table_pre(SDIOx, (uint8_t *)data->buf, (uint32_t)Data->DataBlockSize);
            if (ret)
            {
                SDIOx->Init.flags &= ~SDHCI_REQ_USE_DMA;
            }
            else
            {
                hal_sdhic_set_adma_addr(SDIOx, data->adma_addr);
            }
        }
        else
        {
            //sdhci_writel(host, (uint32_t)data->buf,   SDHCI_DMA_ADDRESS);
            hal_sdhci_set_dma_addr(SDIOx, (uint32_t)data->buf);
        }
    }
#endif

    hal_sdhci_set_dma_mode(SDIOx, SDIOx->Init.flags);


    if (!(SDIOx->Init.flags & SDHCI_REQ_USE_DMA))
    {
        SDIOx->Init.blocks = Data->DataLength;
    }

    hal_sdhci_set_transfer_irqs(SDIOx);
    //LOG_D("data.no of blocks =%d\n", data->blks);
    /* We do not handle DMA boundaries, so set it to max (512 KiB) */
    //sdhci_writew(host, SDHCI_MAKE_BLKSZ(7, data->blksize), SDHCI_BLOCK_SIZE);
    //sdhci_writew(host, data->blks, SDHCI_BLOCK_COUNT);
    hal_sdhci_set_blk(SDIOx, Data->DataBlockSize, Data->DataLength);

#endif
    return HAL_OK;

}

/**
  * @brief  Returns number of remaining data bytes to be transferred.
  * @param  SDIOx Pointer to SDMMC register base
  * @retval Number of remaining data bytes to be transferred
  */
uint32_t SDIO_GetDataCounter(SDHCI_HandleTypeDef *SDIOx)
{
    return 0; //(SDIOx->DCOUNT);
}

/**
  * @brief  Get the FIFO data
  * @param  SDIOx Pointer to SDMMC register base
  * @retval Data received
  */
uint32_t SDIO_GetFIFOCount(SDHCI_HandleTypeDef *SDIOx)
{
    return 0; //(SDIOx->FIFO);
}

/**
  * @brief  Sets one of the two options of inserting read wait interval.
  * @param  SDIOx Pointer to SDMMC register base
  * @param  SDIO_ReadWaitMode SDMMC Read Wait operation mode.
  *          This parameter can be:
  *            @arg SDIO_READ_WAIT_MODE_CLK: Read Wait control by stopping SDMMCCLK
  *            @arg SDIO_READ_WAIT_MODE_DATA2: Read Wait control using SDMMC_DATA2
  * @retval None
  */
HAL_StatusTypeDef SDIO_SetSDMMCReadWaitMode(SDHCI_HandleTypeDef *SDIOx, uint32_t SDIO_ReadWaitMode)
{
    /* Set SDMMC read wait mode */
    //MODIFY_REG(SDIOx->DCTRL, SDIO_DCTRL_RWMOD, SDIO_ReadWaitMode);

    return HAL_OK;
}

/**
  * @}
  */


/**
  * @brief  Send the Data Block Lenght command and check the response
  * @param  SDIOx Pointer to SDMMC register base
  * @retval HAL status
  */
uint32_t SDMMC_CmdBlockLength(SDHCI_HandleTypeDef *SDIOx, uint32_t BlockSize)
{
    SDHCI_CmdArgTypeDef  sdmmc_cmdinit;
    uint32_t errorstate = SDMMC_ERROR_NONE;

    /* Set Block Size for Card */
    sdmmc_cmdinit.Arg         = (uint32_t)BlockSize;
    sdmmc_cmdinit.CmdIndex         = SDMMC_CMD_SET_BLOCKLEN;
    sdmmc_cmdinit.RespType        = SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC | SDHCI_CMD_INDEX;

    //SDIO_SendCommand(SDIOx, &sdmmc_cmdinit);
    hal_sdhci_send_command(SDIOx, &sdmmc_cmdinit);

    /* Check for error conditions */
    errorstate = SDMMC_GetCmdResp1(SDIOx, SDMMC_CMD_SET_BLOCKLEN, SDIO_CMDTIMEOUT);

    return errorstate;
}

/**
  * @brief  Send the Read Single Block command and check the response
  * @param  SDIOx Pointer to SDMMC register base
  * @retval HAL status
  */
uint32_t SDMMC_CmdReadSingleBlock(SDHCI_HandleTypeDef *SDIOx, uint32_t ReadAdd)
{
    SDHCI_CmdArgTypeDef  sdmmc_cmdinit;
    uint32_t errorstate = SDMMC_ERROR_NONE;

    /* Set Block Size for Card */
    sdmmc_cmdinit.Arg         = (uint32_t)ReadAdd;
    sdmmc_cmdinit.CmdIndex         = SDMMC_CMD_READ_SINGLE_BLOCK;
    sdmmc_cmdinit.RespType         = SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC | SDHCI_CMD_INDEX | SDHCI_CMD_DATA;
    hal_sdhci_send_command(SDIOx, &sdmmc_cmdinit);

    /* Check for error conditions */
    errorstate = SDMMC_GetCmdResp1(SDIOx, SDMMC_CMD_READ_SINGLE_BLOCK, SDIO_CMDTIMEOUT);

    return errorstate;
}

/**
  * @brief  Send the Read Multi Block command and check the response
  * @param  SDIOx Pointer to SDIO register base
  * @retval HAL status
  */
uint32_t SDMMC_CmdReadMultiBlock(SDHCI_HandleTypeDef *SDIOx, uint32_t ReadAdd)
{
    SDHCI_CmdArgTypeDef  sdmmc_cmdinit;
    uint32_t errorstate = SDMMC_ERROR_NONE;

    /* Set Block Size for Card */
    sdmmc_cmdinit.Arg         = (uint32_t)ReadAdd;
    sdmmc_cmdinit.CmdIndex         = SDMMC_CMD_READ_MULT_BLOCK;
    sdmmc_cmdinit.RespType         = SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC | SDHCI_CMD_INDEX | SDHCI_CMD_DATA;

    hal_sdhci_send_command(SDIOx, &sdmmc_cmdinit);

    /* Check for error conditions */
    errorstate = SDMMC_GetCmdResp1(SDIOx, SDMMC_CMD_READ_MULT_BLOCK, SDIO_CMDTIMEOUT);

    return errorstate;
}

/**
  * @brief  Send the Write Single Block command and check the response
  * @param  SDIOx Pointer to SDIO register base
  * @retval HAL status
  */
uint32_t SDMMC_CmdWriteSingleBlock(SDHCI_HandleTypeDef *SDIOx, uint32_t WriteAdd)
{
    SDHCI_CmdArgTypeDef  sdmmc_cmdinit;
    uint32_t errorstate = SDMMC_ERROR_NONE;

    /* Set Block Size for Card */
    sdmmc_cmdinit.Arg         = (uint32_t)WriteAdd;
    sdmmc_cmdinit.CmdIndex         = SDMMC_CMD_WRITE_SINGLE_BLOCK;
    sdmmc_cmdinit.RespType         = SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC | SDHCI_CMD_INDEX | SDHCI_CMD_DATA;

    hal_sdhci_send_command(SDIOx, &sdmmc_cmdinit);

    /* Check for error conditions */
    errorstate = SDMMC_GetCmdResp1(SDIOx, SDMMC_CMD_WRITE_SINGLE_BLOCK, SDIO_CMDTIMEOUT);

    return errorstate;
}

/**
  * @brief  Send the Write Multi Block command and check the response
  * @param  SDIOx Pointer to SDIO register base
  * @retval HAL status
  */
uint32_t SDMMC_CmdWriteMultiBlock(SDHCI_HandleTypeDef *SDIOx, uint32_t WriteAdd)
{
    SDHCI_CmdArgTypeDef  sdmmc_cmdinit;
    uint32_t errorstate = SDMMC_ERROR_NONE;

    /* Set Block Size for Card */
    sdmmc_cmdinit.Arg         = (uint32_t)WriteAdd;
    sdmmc_cmdinit.CmdIndex         = SDMMC_CMD_WRITE_MULT_BLOCK;
    sdmmc_cmdinit.RespType         = SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC | SDHCI_CMD_INDEX | SDHCI_CMD_DATA;

    hal_sdhci_send_command(SDIOx, &sdmmc_cmdinit);

    /* Check for error conditions */
    errorstate = SDMMC_GetCmdResp1(SDIOx, SDMMC_CMD_WRITE_MULT_BLOCK, SDIO_CMDTIMEOUT);

    return errorstate;
}

/**
  * @brief  Send the Start Address Erase command for SD and check the response
  * @param  SDIOx Pointer to SDIO register base
  * @retval HAL status
  */
uint32_t SDMMC_CmdSDEraseStartAdd(SDHCI_HandleTypeDef *SDIOx, uint32_t StartAdd)
{
    SDHCI_CmdArgTypeDef  sdmmc_cmdinit;
    uint32_t errorstate = SDMMC_ERROR_NONE;

    /* Set Block Size for Card */
    sdmmc_cmdinit.Arg         = (uint32_t)StartAdd;
    sdmmc_cmdinit.CmdIndex         = SDMMC_CMD_SD_ERASE_GRP_START;
    sdmmc_cmdinit.RespType         = SDHCI_CMD_RESP_SHORT;
    hal_sdhci_send_command(SDIOx, &sdmmc_cmdinit);

    /* Check for error conditions */
    errorstate = SDMMC_GetCmdResp1(SDIOx, SDMMC_CMD_SD_ERASE_GRP_START, SDIO_CMDTIMEOUT);

    return errorstate;
}

/**
  * @brief  Send the End Address Erase command for SD and check the response
  * @param  SDIOx Pointer to SDIO register base
  * @retval HAL status
  */
uint32_t SDMMC_CmdSDEraseEndAdd(SDHCI_HandleTypeDef *SDIOx, uint32_t EndAdd)
{
    SDHCI_CmdArgTypeDef  sdmmc_cmdinit;
    uint32_t errorstate = SDMMC_ERROR_NONE;

    /* Set Block Size for Card */
    sdmmc_cmdinit.Arg         = (uint32_t)EndAdd;
    sdmmc_cmdinit.CmdIndex         = SDMMC_CMD_SD_ERASE_GRP_END;
    sdmmc_cmdinit.RespType         = SDHCI_CMD_RESP_SHORT;

    hal_sdhci_send_command(SDIOx, &sdmmc_cmdinit);

    /* Check for error conditions */
    errorstate = SDMMC_GetCmdResp1(SDIOx, SDMMC_CMD_SD_ERASE_GRP_END, SDIO_CMDTIMEOUT);

    return errorstate;
}

/**
  * @brief  Send the Start Address Erase command and check the response
  * @param  SDIOx Pointer to SDIO register base
  * @retval HAL status
  */
uint32_t SDMMC_CmdEraseStartAdd(SDHCI_HandleTypeDef *SDIOx, uint32_t StartAdd)
{
    SDHCI_CmdArgTypeDef  sdmmc_cmdinit;
    uint32_t errorstate = SDMMC_ERROR_NONE;

    /* Set Block Size for Card */
    sdmmc_cmdinit.Arg         = (uint32_t)StartAdd;
    sdmmc_cmdinit.CmdIndex         = SDMMC_CMD_ERASE_GRP_START;
    sdmmc_cmdinit.RespType         = SDHCI_CMD_RESP_SHORT;
    hal_sdhci_send_command(SDIOx, &sdmmc_cmdinit);

    /* Check for error conditions */
    errorstate = SDMMC_GetCmdResp1(SDIOx, SDMMC_CMD_ERASE_GRP_START, SDIO_CMDTIMEOUT);

    return errorstate;
}

/**
  * @brief  Send the End Address Erase command and check the response
  * @param  SDIOx Pointer to SDIO register base
  * @retval HAL status
  */
uint32_t SDMMC_CmdEraseEndAdd(SDHCI_HandleTypeDef *SDIOx, uint32_t EndAdd)
{
    SDHCI_CmdArgTypeDef  sdmmc_cmdinit;
    uint32_t errorstate = SDMMC_ERROR_NONE;

    /* Set Block Size for Card */
    sdmmc_cmdinit.Arg         = (uint32_t)EndAdd;
    sdmmc_cmdinit.CmdIndex         = SDMMC_CMD_ERASE_GRP_END;
    sdmmc_cmdinit.RespType         = SDHCI_CMD_RESP_SHORT;

    hal_sdhci_send_command(SDIOx, &sdmmc_cmdinit);

    /* Check for error conditions */
    errorstate = SDMMC_GetCmdResp1(SDIOx, SDMMC_CMD_ERASE_GRP_END, SDIO_CMDTIMEOUT);

    return errorstate;
}

/**
  * @brief  Send the Erase command and check the response
  * @param  SDIOx Pointer to SDIO register base
  * @retval HAL status
  */
uint32_t SDMMC_CmdErase(SDHCI_HandleTypeDef *SDIOx)
{
    SDHCI_CmdArgTypeDef  sdmmc_cmdinit;
    uint32_t errorstate = SDMMC_ERROR_NONE;

    /* Set Block Size for Card */
    sdmmc_cmdinit.Arg         = 0U;
    sdmmc_cmdinit.CmdIndex         = SDMMC_CMD_ERASE;
    sdmmc_cmdinit.RespType         = SDHCI_CMD_RESP_SHORT;

    hal_sdhci_send_command(SDIOx, &sdmmc_cmdinit);

    /* Check for error conditions */
    errorstate = SDMMC_GetCmdResp1(SDIOx, SDMMC_CMD_ERASE, SDIO_MAXERASETIMEOUT);

    return errorstate;
}

/**
  * @brief  Send the Stop Transfer command and check the response.
  * @param  SDIOx Pointer to SDIO register base
  * @retval HAL status
  */
uint32_t SDMMC_CmdStopTransfer(SDHCI_HandleTypeDef *SDIOx)
{
    SDHCI_CmdArgTypeDef  sdmmc_cmdinit;
    uint32_t errorstate = SDMMC_ERROR_NONE;

    /* Send CMD12 STOP_TRANSMISSION  */
    sdmmc_cmdinit.Arg         = 0U;
    sdmmc_cmdinit.CmdIndex         = SDMMC_CMD_STOP_TRANSMISSION;
    sdmmc_cmdinit.RespType         = SDHCI_CMD_RESP_SHORT;
    hal_sdhci_send_command(SDIOx, &sdmmc_cmdinit);

    /* Check for error conditions */
    errorstate = SDMMC_GetCmdResp1(SDIOx, SDMMC_CMD_STOP_TRANSMISSION, 100000000U);

    return errorstate;
}

/**
  * @brief  Send the Select Deselect command and check the response.
  * @param  SDIOx Pointer to SDIO register base
  * @param  addr Address of the card to be selected
  * @retval HAL status
  */
uint32_t SDMMC_CmdSelDesel(SDHCI_HandleTypeDef *SDIOx, uint64_t Addr)
{
    SDHCI_CmdArgTypeDef  sdmmc_cmdinit;
    uint32_t errorstate = SDMMC_ERROR_NONE;

    /* Send CMD7 SDMMC_SEL_DESEL_CARD */
    sdmmc_cmdinit.Arg         = (uint32_t)Addr;
    sdmmc_cmdinit.CmdIndex         = SDMMC_CMD_SEL_DESEL_CARD;
    sdmmc_cmdinit.RespType         = SDHCI_CMD_RESP_SHORT | SDHCI_CMD_INDEX | SDHCI_CMD_CRC;

    hal_sdhci_send_command(SDIOx, &sdmmc_cmdinit);

    /* Check for error conditions */
    errorstate = SDMMC_GetCmdResp1(SDIOx, SDMMC_CMD_SEL_DESEL_CARD, SDIO_CMDTIMEOUT);

    return errorstate;
}

/**
  * @brief  Send the Go Idle State command and check the response.
  * @param  SDIOx Pointer to SDIO register base
  * @retval HAL status
  */
uint32_t SDMMC_CmdGoIdleState(SDHCI_HandleTypeDef *SDIOx)
{
    SDHCI_CmdArgTypeDef  sdmmc_cmdinit;
    uint32_t errorstate = SDMMC_ERROR_NONE;

    sdmmc_cmdinit.Arg         = 0U;
    sdmmc_cmdinit.CmdIndex         = SDMMC_CMD_GO_IDLE_STATE;
    sdmmc_cmdinit.RespType         = SDHCI_CMD_RESP_NONE;

    hal_sdhci_send_command(SDIOx, &sdmmc_cmdinit);

    /* Check for error conditions */
    errorstate = SDMMC_GetCmdError(SDIOx);

    return errorstate;
}

/**
  * @brief  Send the Operating Condition command and check the response.
  * @param  SDIOx Pointer to SDIO register base
  * @retval HAL status
  */
uint32_t SDMMC_CmdOperCond(SDHCI_HandleTypeDef *SDIOx)
{
    SDHCI_CmdArgTypeDef  sdmmc_cmdinit;
    uint32_t errorstate = SDMMC_ERROR_NONE;

    /* Send CMD8 to verify SD card interface operating condition */
    /* Argument: - [31:12]: Reserved (shall be set to '0')
    - [11:8]: Supply Voltage (VHS) 0x1 (Range: 2.7-3.6 V)
    - [7:0]: Check Pattern (recommended 0xAA) */
    /* CMD Response: R7 */
    sdmmc_cmdinit.Arg         = SDMMC_CHECK_PATTERN;
    sdmmc_cmdinit.CmdIndex         = SDMMC_CMD_HS_SEND_EXT_CSD;
    sdmmc_cmdinit.RespType         = SDHCI_CMD_RESP_SHORT | SDHCI_CMD_INDEX | SDHCI_CMD_CRC;

    hal_sdhci_send_command(SDIOx, &sdmmc_cmdinit);

    /* Check for error conditions */
    errorstate = SDMMC_GetCmdResp7(SDIOx);

    return errorstate;
}

/**
  * @brief  Send the Application command to verify that that the next command
  *         is an application specific com-mand rather than a standard command
  *         and check the response.
  * @param  SDIOx Pointer to SDIO register base
  * @retval HAL status
  */
uint32_t SDMMC_CmdAppCommand(SDHCI_HandleTypeDef *SDIOx, uint32_t Argument)
{
    SDHCI_CmdArgTypeDef  sdmmc_cmdinit;
    uint32_t errorstate = SDMMC_ERROR_NONE;

    sdmmc_cmdinit.Arg         = (uint32_t)Argument;
    sdmmc_cmdinit.CmdIndex         = SDMMC_CMD_APP_CMD;
    sdmmc_cmdinit.RespType         = SDHCI_CMD_RESP_SHORT | SDHCI_CMD_INDEX | SDHCI_CMD_CRC;

    hal_sdhci_send_command(SDIOx, &sdmmc_cmdinit);

    /* Check for error conditions */
    /* If there is a HAL_ERROR, it is a MMC card, else
    it is a SD card: SD card 2.0 (voltage range mismatch)
       or SD card 1.x */
    errorstate = SDMMC_GetCmdResp1(SDIOx, SDMMC_CMD_APP_CMD, SDIO_CMDTIMEOUT);

    return errorstate;
}

/**
  * @brief  Send the command asking the accessed card to send its operating
  *         condition register (OCR)
  * @param  SDIOx Pointer to SDIO register base
  * @retval HAL status
  */
uint32_t SDMMC_CmdAppOperCommand(SDHCI_HandleTypeDef *SDIOx, uint32_t SdType)
{
    SDHCI_CmdArgTypeDef  sdmmc_cmdinit;
    uint32_t errorstate = SDMMC_ERROR_NONE;

    sdmmc_cmdinit.Arg         = SDMMC_VOLTAGE_WINDOW_SD | SdType;
    sdmmc_cmdinit.CmdIndex         = SDMMC_CMD_SD_APP_OP_COND;
    sdmmc_cmdinit.RespType         = SDHCI_CMD_RESP_SHORT;

    hal_sdhci_send_command(SDIOx, &sdmmc_cmdinit);

    /* Check for error conditions */
    errorstate = SDMMC_GetCmdResp3(SDIOx);

    return errorstate;
}

/**
  * @brief  Send the Bus Width command and check the response.
  * @param  SDIOx Pointer to SDIO register base
  * @retval HAL status
  */
uint32_t SDMMC_CmdBusWidth(SDHCI_HandleTypeDef *SDIOx, uint32_t BusWidth)
{
    SDHCI_CmdArgTypeDef  sdmmc_cmdinit;
    uint32_t errorstate = SDMMC_ERROR_NONE;

    sdmmc_cmdinit.Arg         = (uint32_t)BusWidth;
    sdmmc_cmdinit.CmdIndex         = SDMMC_CMD_APP_SD_SET_BUSWIDTH;
    sdmmc_cmdinit.RespType         = SDHCI_CMD_RESP_SHORT | SDHCI_CMD_INDEX | SDHCI_CMD_CRC;

    hal_sdhci_send_command(SDIOx, &sdmmc_cmdinit);

    /* Check for error conditions */
    errorstate = SDMMC_GetCmdResp1(SDIOx, SDMMC_CMD_APP_SD_SET_BUSWIDTH, SDIO_CMDTIMEOUT);

    return errorstate;
}

/**
  * @brief  Send the Send SCR command and check the response.
  * @param  SDIOx Pointer to SDMMC register base
  * @retval HAL status
  */
uint32_t SDMMC_CmdSendSCR(SDHCI_HandleTypeDef *SDIOx)
{
    SDHCI_CmdArgTypeDef  sdmmc_cmdinit;
    uint32_t errorstate = SDMMC_ERROR_NONE;

    /* Send CMD51 SD_APP_SEND_SCR */
    sdmmc_cmdinit.Arg         = 0U;
    sdmmc_cmdinit.CmdIndex         = SDMMC_CMD_SD_APP_SEND_SCR;
    sdmmc_cmdinit.RespType         = SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC | SDHCI_CMD_INDEX | SDHCI_CMD_DATA;

    hal_sdhci_send_command(SDIOx, &sdmmc_cmdinit);

    /* Check for error conditions */
    errorstate = SDMMC_GetCmdResp1(SDIOx, SDMMC_CMD_SD_APP_SEND_SCR, SDIO_CMDTIMEOUT);

    return errorstate;
}

/**
  * @brief  Send the Send CID command and check the response.
  * @param  SDIOx Pointer to SDIO register base
  * @retval HAL status
  */
uint32_t SDMMC_CmdSendCID(SDHCI_HandleTypeDef *SDIOx)
{
    SDHCI_CmdArgTypeDef  sdmmc_cmdinit;
    uint32_t errorstate = SDMMC_ERROR_NONE;

    /* Send CMD2 ALL_SEND_CID */
    sdmmc_cmdinit.Arg         = 0U;
    sdmmc_cmdinit.CmdIndex         = SDMMC_CMD_ALL_SEND_CID;
    sdmmc_cmdinit.RespType         = SDHCI_CMD_RESP_LONG;

    hal_sdhci_send_command(SDIOx, &sdmmc_cmdinit);

    /* Check for error conditions */
    errorstate = SDMMC_GetCmdResp2(SDIOx);

    return errorstate;
}

/**
  * @brief  Send the Send CSD command and check the response.
  * @param  SDIOx Pointer to SDIO register base
  * @retval HAL status
  */
uint32_t SDMMC_CmdSendCSD(SDHCI_HandleTypeDef *SDIOx, uint32_t Argument)
{
    SDHCI_CmdArgTypeDef  sdmmc_cmdinit;
    uint32_t errorstate = SDMMC_ERROR_NONE;

    /* Send CMD9 SEND_CSD */
    sdmmc_cmdinit.Arg         = (uint32_t)Argument;
    sdmmc_cmdinit.CmdIndex         = SDMMC_CMD_SEND_CSD;
    sdmmc_cmdinit.RespType         = SDHCI_CMD_RESP_LONG | SDHCI_CMD_CRC;

    SDEX_DEBUG("SDMMC_CmdSendCSD: 0x%x, 0x%x, 0x%x\n", sdmmc_cmdinit.Arg, sdmmc_cmdinit.CmdIndex, sdmmc_cmdinit.RespType);
    hal_sdhci_send_command(SDIOx, &sdmmc_cmdinit);

    /* Check for error conditions */
    errorstate = SDMMC_GetCmdResp2(SDIOx);

    return errorstate;
}

/**
  * @brief  Send the Send CSD command and check the response.
  * @param  SDIOx Pointer to SDIO register base
  * @retval HAL status
  */
uint32_t SDMMC_CmdSetRelAdd(SDHCI_HandleTypeDef *SDIOx, uint16_t *pRCA)
{
    SDHCI_CmdArgTypeDef  sdmmc_cmdinit;
    uint32_t errorstate = SDMMC_ERROR_NONE;

    /* Send CMD3 SD_CMD_SET_REL_ADDR */
    sdmmc_cmdinit.Arg         = 0U;
    sdmmc_cmdinit.CmdIndex         = SDMMC_CMD_SET_REL_ADDR;
    sdmmc_cmdinit.RespType         = SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC | SDHCI_CMD_INDEX;

    hal_sdhci_send_command(SDIOx, &sdmmc_cmdinit);

    /* Check for error conditions */
    errorstate = SDMMC_GetCmdResp6(SDIOx, SDMMC_CMD_SET_REL_ADDR, pRCA);

    return errorstate;
}

/**
  * @brief  Send the Status command and check the response.
  * @param  SDIOx Pointer to SDIO register base
  * @retval HAL status
  */
uint32_t SDMMC_CmdSendStatus(SDHCI_HandleTypeDef *SDIOx, uint32_t Argument)
{
    SDHCI_CmdArgTypeDef  sdmmc_cmdinit;
    uint32_t errorstate = SDMMC_ERROR_NONE;

    sdmmc_cmdinit.Arg         = (uint32_t)Argument;
    sdmmc_cmdinit.CmdIndex         = SDMMC_CMD_SEND_STATUS;
    sdmmc_cmdinit.RespType         = SDHCI_CMD_RESP_SHORT;

    hal_sdhci_send_command(SDIOx, &sdmmc_cmdinit);

    /* Check for error conditions */
    errorstate = SDMMC_GetCmdResp1(SDIOx, SDMMC_CMD_SEND_STATUS, SDIO_CMDTIMEOUT);

    return errorstate;
}

/**
  * @brief  Send the Status register command and check the response.
  * @param  SDIOx Pointer to SDIO register base
  * @retval HAL status
  */
uint32_t SDMMC_CmdStatusRegister(SDHCI_HandleTypeDef *SDIOx)
{
    SDHCI_CmdArgTypeDef  sdmmc_cmdinit;
    uint32_t errorstate = SDMMC_ERROR_NONE;

    sdmmc_cmdinit.Arg         = 0U;
    sdmmc_cmdinit.CmdIndex         = SDMMC_CMD_SD_APP_STATUS;
    sdmmc_cmdinit.RespType         = SDHCI_CMD_RESP_SHORT;

    hal_sdhci_send_command(SDIOx, &sdmmc_cmdinit);

    /* Check for error conditions */
    errorstate = SDMMC_GetCmdResp1(SDIOx, SDMMC_CMD_SD_APP_STATUS, SDIO_CMDTIMEOUT);

    return errorstate;
}

/**
  * @brief  Sends host capacity support information and activates the card's
  *         initialization process. Send SDMMC_CMD_SEND_OP_COND command
  * @param  SDIOx Pointer to SDIO register base
  * @parame Argument Argument used for the command
  * @retval HAL status
  */
uint32_t SDMMC_CmdOpCondition(SDHCI_HandleTypeDef *SDIOx, uint32_t Argument)
{
    SDHCI_CmdArgTypeDef  sdmmc_cmdinit;
    uint32_t errorstate = SDMMC_ERROR_NONE;

    sdmmc_cmdinit.Arg         = Argument;
    sdmmc_cmdinit.CmdIndex         = SDMMC_CMD_SEND_OP_COND;
    sdmmc_cmdinit.RespType         = SDHCI_CMD_RESP_SHORT;

    hal_sdhci_send_command(SDIOx, &sdmmc_cmdinit);

    /* Check for error conditions */
    errorstate = SDMMC_GetCmdResp3(SDIOx);

    return errorstate;
}

/**
  * @brief  Checks switchable function and switch card function. SDMMC_CMD_HS_SWITCH comand
  * @param  SDIOx Pointer to SDIO register base
  * @parame Argument Argument used for the command
  * @retval HAL status
  */
uint32_t SDMMC_CmdSwitch(SDHCI_HandleTypeDef *SDIOx, uint32_t Argument)
{
    SDHCI_CmdArgTypeDef  sdmmc_cmdinit;
    uint32_t errorstate = SDMMC_ERROR_NONE;

    sdmmc_cmdinit.Arg         = Argument;
    sdmmc_cmdinit.CmdIndex         = SDMMC_CMD_HS_SWITCH;
    sdmmc_cmdinit.RespType         = SDHCI_CMD_RESP_SHORT;

    hal_sdhci_send_command(SDIOx, &sdmmc_cmdinit);

    /* Check for error conditions */
    errorstate = SDMMC_GetCmdResp1(SDIOx, SDMMC_CMD_HS_SWITCH, SDIO_CMDTIMEOUT);

    return errorstate;
}

/**
  * @}
  */

/* Private function ----------------------------------------------------------*/
/** @addtogroup SD_Private_Functions
  * @{
  */

/**
  * @brief  Checks for error conditions for CMD0.
  * @param  hsd SD handle
  * @retval SD Card error state
  */
static uint32_t SDMMC_GetCmdError(SDHCI_HandleTypeDef *SDIOx)
{
    int res;

    res = hal_sdhci_command_finish(SDIOx, SDIO_CMDTIMEOUT);
    if (res <= 0)
    {
        //hal_sdhci_clear_int(handle, SDHCI_INT_ERROR|SDHCI_INT_CMD_MASK);
        hal_sdhci_clear_int(SDIOx, SDHCI_INT_ERROR);
        return SDMMC_ERROR_CMD_CRC_FAIL;
    }

    /* Clear all the static flags */
    hal_sdhci_clear_int(SDIOx, res);

    return SDMMC_ERROR_NONE;
}

/**
  * @brief  Checks for error conditions for R1 response.
  * @param  hsd SD handle
  * @param  SD_CMD The sent command index
  * @retval SD Card error state
  */
static uint32_t SDMMC_GetCmdResp1(SDHCI_HandleTypeDef *SDIOx, uint8_t SD_CMD, uint32_t Timeout)
{
    uint32_t response_r1;
    int res;

    res = hal_sdhci_command_finish(SDIOx, Timeout);
    if (res <= 0)
    {
        SDEX_DEBUG("SDMMC_GetCmdResp1 wait cmd done fail with 0x%x\n", res);
        //hal_sdhci_clear_int(handle, SDHCI_INT_ERROR|SDHCI_INT_CMD_MASK);
        hal_sdhci_clear_int(SDIOx, SDHCI_INT_ERROR);
        return SDMMC_ERROR_CMD_CRC_FAIL;
    }

    /* Clear all the static flags */
    hal_sdhci_clear_int(SDIOx, SDHCI_INT_CMD_MASK);

    /* We have received response, retrieve it for analysis  */
    response_r1 = hal_sdhci_get_response(SDIOx, 0);
    SDEX_DEBUG("Res0 : 0x%08x\n", response_r1);

    if ((response_r1 & SDMMC_OCR_ERRORBITS) == 0)
    {
        return SDMMC_ERROR_NONE;
    }
    else if ((response_r1 & SDMMC_OCR_ADDR_OUT_OF_RANGE) == SDMMC_OCR_ADDR_OUT_OF_RANGE)
    {
        return SDMMC_ERROR_ADDR_OUT_OF_RANGE;
    }
    else if ((response_r1 & SDMMC_OCR_ADDR_MISALIGNED) == SDMMC_OCR_ADDR_MISALIGNED)
    {
        return SDMMC_ERROR_ADDR_MISALIGNED;
    }
    else if ((response_r1 & SDMMC_OCR_BLOCK_LEN_ERR) == SDMMC_OCR_BLOCK_LEN_ERR)
    {
        return SDMMC_ERROR_BLOCK_LEN_ERR;
    }
    else if ((response_r1 & SDMMC_OCR_ERASE_SEQ_ERR) == SDMMC_OCR_ERASE_SEQ_ERR)
    {
        return SDMMC_ERROR_ERASE_SEQ_ERR;
    }
    else if ((response_r1 & SDMMC_OCR_BAD_ERASE_PARAM) == SDMMC_OCR_BAD_ERASE_PARAM)
    {
        return SDMMC_ERROR_BAD_ERASE_PARAM;
    }
    else if ((response_r1 & SDMMC_OCR_WRITE_PROT_VIOLATION) == SDMMC_OCR_WRITE_PROT_VIOLATION)
    {
        return SDMMC_ERROR_WRITE_PROT_VIOLATION;
    }
    else if ((response_r1 & SDMMC_OCR_LOCK_UNLOCK_FAILED) == SDMMC_OCR_LOCK_UNLOCK_FAILED)
    {
        return SDMMC_ERROR_LOCK_UNLOCK_FAILED;
    }
    else if ((response_r1 & SDMMC_OCR_COM_CRC_FAILED) == SDMMC_OCR_COM_CRC_FAILED)
    {
        return SDMMC_ERROR_COM_CRC_FAILED;
    }
    else if ((response_r1 & SDMMC_OCR_ILLEGAL_CMD) == SDMMC_OCR_ILLEGAL_CMD)
    {
        return SDMMC_ERROR_ILLEGAL_CMD;
    }
    else if ((response_r1 & SDMMC_OCR_CARD_ECC_FAILED) == SDMMC_OCR_CARD_ECC_FAILED)
    {
        return SDMMC_ERROR_CARD_ECC_FAILED;
    }
    else if ((response_r1 & SDMMC_OCR_CC_ERROR) == SDMMC_OCR_CC_ERROR)
    {
        return SDMMC_ERROR_CC_ERR;
    }
    else if ((response_r1 & SDMMC_OCR_STREAM_READ_UNDERRUN) == SDMMC_OCR_STREAM_READ_UNDERRUN)
    {
        return SDMMC_ERROR_STREAM_READ_UNDERRUN;
    }
    else if ((response_r1 & SDMMC_OCR_STREAM_WRITE_OVERRUN) == SDMMC_OCR_STREAM_WRITE_OVERRUN)
    {
        return SDMMC_ERROR_STREAM_WRITE_OVERRUN;
    }
    else if ((response_r1 & SDMMC_OCR_CID_CSD_OVERWRITE) == SDMMC_OCR_CID_CSD_OVERWRITE)
    {
        return SDMMC_ERROR_CID_CSD_OVERWRITE;
    }
    else if ((response_r1 & SDMMC_OCR_WP_ERASE_SKIP) == SDMMC_OCR_WP_ERASE_SKIP)
    {
        return SDMMC_ERROR_WP_ERASE_SKIP;
    }
    else if ((response_r1 & SDMMC_OCR_CARD_ECC_DISABLED) == SDMMC_OCR_CARD_ECC_DISABLED)
    {
        return SDMMC_ERROR_CARD_ECC_DISABLED;
    }
    else if ((response_r1 & SDMMC_OCR_ERASE_RESET) == SDMMC_OCR_ERASE_RESET)
    {
        return SDMMC_ERROR_ERASE_RESET;
    }
    else if ((response_r1 & SDMMC_OCR_AKE_SEQ_ERROR) == SDMMC_OCR_AKE_SEQ_ERROR)
    {
        return SDMMC_ERROR_AKE_SEQ_ERR;
    }
    else
    {
        return SDMMC_ERROR_GENERAL_UNKNOWN_ERR;
    }
}

/**
  * @brief  Checks for error conditions for R2 (CID or CSD) response.
  * @param  hsd SD handle
  * @retval SD Card error state
  */
static uint32_t SDMMC_GetCmdResp2(SDHCI_HandleTypeDef *SDIOx)
{
    int res;

    res = hal_sdhci_command_finish(SDIOx, 1000);
    SDEX_DEBUG("irq = 0x%x\n", res);
    if (res <= 0 || (res & SDHCI_INT_ERROR_MASK))
    {
        //hal_sdhci_clear_int(handle, SDHCI_INT_ERROR|SDHCI_INT_CMD_MASK);
        hal_sdhci_clear_int(SDIOx, SDHCI_INT_ERROR_MASK);
        return SDMMC_ERROR_CMD_CRC_FAIL;
    }

    /* Clear all the static flags */
    hal_sdhci_clear_int(SDIOx, res);

    /* We have received response, retrieve it for analysis  */
    // uint32_t response_r1 =
    hal_sdhci_get_response(SDIOx, 1);

    return SDMMC_ERROR_NONE;
}

/**
  * @brief  Checks for error conditions for R3 (OCR) response.
  * @param  hsd SD handle
  * @retval SD Card error state
  */
static uint32_t SDMMC_GetCmdResp3(SDHCI_HandleTypeDef *SDIOx)
{
    int res;

    res = hal_sdhci_command_finish(SDIOx, 1000);
    if (res <= 0)
    {
        //hal_sdhci_clear_int(handle, SDHCI_INT_ERROR|SDHCI_INT_CMD_MASK);
        hal_sdhci_clear_int(SDIOx, SDHCI_INT_ERROR);
        return SDMMC_ERROR_CMD_CRC_FAIL;
    }

    /* Clear all the static flags */
    hal_sdhci_clear_int(SDIOx, res);

    /* We have received response, retrieve it for analysis  */
    //uint32_t response_r1 =
    hal_sdhci_get_response(SDIOx, 2);

    return SDMMC_ERROR_NONE;
}

/**
  * @brief  Checks for error conditions for R6 (RCA) response.
  * @param  hsd SD handle
  * @param  SD_CMD The sent command index
  * @param  pRCA Pointer to the variable that will contain the SD card relative
  *         address RCA
  * @retval SD Card error state
  */
static uint32_t SDMMC_GetCmdResp6(SDHCI_HandleTypeDef *SDIOx, uint8_t SD_CMD, uint16_t *pRCA)
{
    uint32_t response_r1;
    int res;

    res = hal_sdhci_command_finish(SDIOx, 1000);
    if (res <= 0)
    {
        //hal_sdhci_clear_int(handle, SDHCI_INT_ERROR|SDHCI_INT_CMD_MASK);
        hal_sdhci_clear_int(SDIOx, SDHCI_INT_ERROR);
        return SDMMC_ERROR_CMD_CRC_FAIL;
    }

    /* Clear all the static flags */
    hal_sdhci_clear_int(SDIOx, res);

    /* We have received response, retrieve it.  */
    response_r1 = hal_sdhci_get_response(SDIOx, 0);

    if ((response_r1 & (SDMMC_R6_GENERAL_UNKNOWN_ERROR | SDMMC_R6_ILLEGAL_CMD | SDMMC_R6_COM_CRC_FAILED)) == 0)
    {
        *pRCA = (uint16_t)(response_r1 >> 16);

        return SDMMC_ERROR_NONE;
    }
    else if ((response_r1 & SDMMC_R6_ILLEGAL_CMD) == SDMMC_R6_ILLEGAL_CMD)
    {
        return SDMMC_ERROR_ILLEGAL_CMD;
    }
    else if ((response_r1 & SDMMC_R6_COM_CRC_FAILED) == SDMMC_R6_COM_CRC_FAILED)
    {
        return SDMMC_ERROR_COM_CRC_FAILED;
    }
    else
    {
        return SDMMC_ERROR_GENERAL_UNKNOWN_ERR;
    }
}

/**
  * @brief  Checks for error conditions for R7 response.
  * @param  hsd SD handle
  * @retval SD Card error state
  */
static uint32_t SDMMC_GetCmdResp7(SDHCI_HandleTypeDef *SDIOx)
{
    int res;

    res = hal_sdhci_command_finish(SDIOx, 1000);
    if (res <= 0)
    {
        //hal_sdhci_clear_int(handle, SDHCI_INT_ERROR|SDHCI_INT_CMD_MASK);
        hal_sdhci_clear_int(SDIOx, SDHCI_INT_ERROR);
        return SDMMC_ERROR_CMD_CRC_FAIL;
    }

    /* Clear all the static flags */
    hal_sdhci_clear_int(SDIOx, res);

    /* We have received response, retrieve it for analysis  */
    // uint32_t response_r1 =
    hal_sdhci_get_response(SDIOx, 2);

    return SDMMC_ERROR_NONE;
}

/**
  * @}
*/


#if 1
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/** @defgroup SD_Private_Functions SD Private Functions
  * @{
  */
static uint32_t SD_InitCard(SD_HandleTypeDef *hsd);
static uint32_t SD_PowerON(SD_HandleTypeDef *hsd);
static uint32_t SD_SendSDStatus(SD_HandleTypeDef *hsd, uint32_t *pSDstatus);
static uint32_t SD_SendStatus(SD_HandleTypeDef *hsd, uint32_t *pCardStatus);
static uint32_t SD_WideBus_Enable(SD_HandleTypeDef *hsd);
static uint32_t SD_WideBus_Disable(SD_HandleTypeDef *hsd);
static uint32_t SD_FindSCR(SD_HandleTypeDef *hsd, uint32_t *pSCR);
static HAL_StatusTypeDef SD_PowerOFF(SD_HandleTypeDef *hsd);

#if 0
    static HAL_StatusTypeDef SD_Write_IT(SD_HandleTypeDef *hsd);
    static HAL_StatusTypeDef SD_Read_IT(SD_HandleTypeDef *hsd);
    static void SD_DMATransmitCplt(DMA_HandleTypeDef *hdma);
    static void SD_DMAReceiveCplt(DMA_HandleTypeDef *hdma);
    static void SD_DMAError(DMA_HandleTypeDef *hdma);
    static void SD_DMATxAbort(DMA_HandleTypeDef *hdma);
    static void SD_DMARxAbort(DMA_HandleTypeDef *hdma);
#endif
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup SD_Exported_Functions
  * @{
  */

/** @addtogroup SD_Exported_Functions_Group1
 *  @brief   Initialization and de-initialization functions
 *
@verbatim
  ==============================================================================
          ##### Initialization and de-initialization functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to initialize/de-initialize the SD
    card device to be ready for use.

@endverbatim
  * @{
  */

/**
  * @brief  Initializes the SD according to the specified parameters in the
            SD_HandleTypeDef and create the associated handle.
  * @param  hsd Pointer to the SD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SD_Init(SD_HandleTypeDef *hsd)
{
    HAL_StatusTypeDef ret = HAL_OK;
    /* Check the SD handle allocation */
    if (hsd == NULL)
    {
        return HAL_ERROR;
    }

    if (hsd->State == HAL_SD_STATE_RESET)
    {
        /* Allocate lock resource and initialize it */
        hsd->Lock = HAL_UNLOCKED;
        /* Init the low level hardware : GPIO, CLOCK, CORTEX...etc */
        HAL_SD_MspInit(hsd);
    }

    hal_sdhci_reset(hsd->Instance, SDHCI_RESET_ALL);

    int version = hal_sdhci_get_host_version(hsd->Instance);
    SDEX_DEBUG("Reg VERSION 0x%x\n", version);

    int caps = hal_sdhci_get_host_cap(hsd->Instance);
    SDEX_DEBUG("SDHCI caps 0x%x\n", caps);

    hal_sdhci_init(hsd->Instance, 0);

    hsd->State = HAL_SD_STATE_BUSY;

    /* Initialize the Card parameters */
    ret = HAL_SD_InitCard(hsd);

    /* Initialize the error code */
    hsd->ErrorCode = HAL_DMA_ERROR_NONE;

    /* Initialize the SD operation */
    hsd->Context = SD_CONTEXT_NONE;

    /* Initialize the SD state */
    hsd->State = HAL_SD_STATE_READY;

    return ret;
}

/**
  * @brief  Initializes the SD Card.
  * @param  hsd Pointer to SD handle
  * @note   This function initializes the SD card. It could be used when a card
            re-initialization is needed.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SD_InitCard(SD_HandleTypeDef *hsd)
{
    uint32_t errorstate = HAL_SD_ERROR_NONE;
    SDIO_InitTypeDef Init;

    /* Default SDIO peripheral configuration for SD card initialization */
    Init.ClockEdge           = 0; //SDIO_CLOCK_EDGE_RISING;
    Init.ClockBypass         = 0; //SDIO_CLOCK_BYPASS_DISABLE;
    Init.ClockPowerSave      = 0; //SDIO_CLOCK_POWER_SAVE_DISABLE;
    Init.BusWide             = 1; //SDIO_BUS_WIDE_1B;
    Init.HardwareFlowControl = 0; //SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
    Init.ClockDiv            = 1; //SDIO_INIT_CLK_DIV;

    /* Initialize SDIO peripheral interface with default configuration */
    SDIO_Init(hsd->Instance, Init);

    /* Disable SDIO Clock */
    //__HAL_SD_DISABLE(hsd);

    /* Set Power State to ON */
    //SDIO_PowerState_ON(hsd->Instance);

    /* Enable SDIO Clock */
    //__HAL_SD_ENABLE(hsd);

    /* Required power up waiting time before starting the SD initialization
    sequence */
    HAL_Delay(2U);

    /* Identify card operating voltage */
    errorstate = SD_PowerON(hsd);
    if (errorstate != HAL_SD_ERROR_NONE)
    {
        hsd->State = HAL_SD_STATE_READY;
        hsd->ErrorCode |= errorstate;
        SDEX_DEBUG("error SD_PowerON\n");
        return HAL_ERROR;
    }

    /* Card initialization */
    errorstate = SD_InitCard(hsd);
    if (errorstate != HAL_SD_ERROR_NONE)
    {
        hsd->State = HAL_SD_STATE_READY;
        hsd->ErrorCode |= errorstate;
        SDEX_DEBUG("error SD_InitCard\n");
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
  * @brief  De-Initializes the SD card.
  * @param  hsd Pointer to SD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SD_DeInit(SD_HandleTypeDef *hsd)
{
    /* Check the SD handle allocation */
    if (hsd == NULL)
    {
        return HAL_ERROR;
    }

    hsd->State = HAL_SD_STATE_BUSY;

    /* Set SD power state to off */
    SD_PowerOFF(hsd);

    /* De-Initialize the MSP layer */
    HAL_SD_MspDeInit(hsd);

    hsd->ErrorCode = HAL_SD_ERROR_NONE;
    hsd->State = HAL_SD_STATE_RESET;

    return HAL_OK;
}


/**
  * @brief  Initializes the SD MSP.
  * @param  hsd Pointer to SD handle
  * @retval None
  */
__weak void HAL_SD_MspInit(SD_HandleTypeDef *hsd)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hsd);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_SD_MspInit could be implemented in the user file
     */
}

/**
  * @brief  De-Initialize SD MSP.
  * @param  hsd Pointer to SD handle
  * @retval None
  */
__weak void HAL_SD_MspDeInit(SD_HandleTypeDef *hsd)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hsd);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_SD_MspDeInit could be implemented in the user file
     */
}

/**
  * @}
  */

/** @addtogroup SD_Exported_Functions_Group2
 *  @brief   Data transfer functions
 *
@verbatim
  ==============================================================================
                        ##### IO operation functions #####
  ==============================================================================
  [..]
    This subsection provides a set of functions allowing to manage the data
    transfer from/to SD card.

@endverbatim
  * @{
  */

/**
  * @brief  Reads block(s) from a specified address in a card. The Data transfer
  *         is managed by polling mode.
  * @note   This API should be followed by a check on the card state through
  *         HAL_SD_GetCardState().
  * @param  hsd Pointer to SD handle
  * @param  pData pointer to the buffer that will contain the received data
  * @param  BlockAdd Block Address from where data is to be read
  * @param  NumberOfBlocks Number of SD blocks to read
  * @param  Timeout Specify timeout value
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SD_ReadBlocks(SD_HandleTypeDef *hsd, uint8_t *pData, uint32_t BlockAdd, uint32_t NumberOfBlocks, uint32_t Timeout)
{
    SDIO_DataInitTypeDef config;
    uint32_t errorstate = HAL_SD_ERROR_NONE;
    uint32_t *tempbuff = (uint32_t *)pData;

    if (NULL == pData)
    {
        SDEX_DEBUG("error HAL_SD_ReadBlocks no data buffer\n");
        hsd->ErrorCode |= HAL_SD_ERROR_PARAM;
        return HAL_ERROR;
    }

    if (hsd->State == HAL_SD_STATE_READY)
    {
        hsd->ErrorCode = HAL_DMA_ERROR_NONE;

        if (0) //((BlockAdd + NumberOfBlocks) > (hsd->SdCard.LogBlockNbr))
        {
            SDEX_DEBUG("error LogBlockNbr\n");
            hsd->ErrorCode |= HAL_SD_ERROR_ADDR_OUT_OF_RANGE;
            return HAL_ERROR;
        }

        hsd->State = HAL_SD_STATE_BUSY;

        /* Initialize data control register */
        //hsd->Instance->DCTRL = 0U;

        if (hsd->SdCard.CardType != CARD_SDHC_SDXC)
        {
            BlockAdd *= 512U;
        }

        /* Set Block Size for Card */
        errorstate = SDMMC_CmdBlockLength(hsd->Instance, BLOCKSIZE);
        if (0)//(errorstate != HAL_SD_ERROR_NONE)
        {
            /* Clear all the static flags */
            //__HAL_SD_CLEAR_FLAG(hsd, SDIO_STATIC_FLAGS);
            hsd->ErrorCode |= errorstate;
            hsd->State = HAL_SD_STATE_READY;
            SDEX_DEBUG("error set blck length\n");
            return HAL_ERROR;
        }

        /* Configure the SD DPSM (Data Path State Machine) */
        config.DataTimeOut   = 1000;
        config.DataLength    = NumberOfBlocks;
        config.DataBlockSize = 512; //SDIO_DATABLOCK_SIZE_512B;
        config.TransferDir   = 0; //SDIO_TRANSFER_DIR_TO_SDIO;
        config.TransferMode  = 0; //SDIO_TRANSFER_MODE_BLOCK;
        SDIO_ConfigData(hsd->Instance, &config);

        /* Read block(s) in polling mode */
        if (NumberOfBlocks > 1U)
        {
            hsd->Context = SD_CONTEXT_READ_MULTIPLE_BLOCK;

            /* Read Multi Block command */
            errorstate = SDMMC_CmdReadMultiBlock(hsd->Instance, BlockAdd);
        }
        else
        {
            hsd->Context = SD_CONTEXT_READ_SINGLE_BLOCK;

            /* Read Single Block command */
            errorstate = SDMMC_CmdReadSingleBlock(hsd->Instance, BlockAdd);
        }
        if (0) //if (errorstate != HAL_SD_ERROR_NONE)
        {
            /* Clear all the static flags */
            //__HAL_SD_CLEAR_FLAG(hsd, SDIO_STATIC_FLAGS);
            hsd->ErrorCode |= errorstate;
            hsd->State = HAL_SD_STATE_READY;
            SDEX_DEBUG("error read single/multi block\n");
            return HAL_ERROR;
        }

        int intmask = hal_sdhci_data_finish(hsd->Instance, 1000);
        hal_sdhci_clear_int(hsd->Instance, intmask);
        if (intmask > 0)
        {
            if (intmask & (SDHCI_INT_DATA_AVAIL | SDHCI_INT_SPACE_AVAIL))
                hal_sdhci_transfer_pio(hsd->Instance, 1, (uint8_t *)tempbuff, (uint32_t)config.DataLength);
        }

        hsd->State = HAL_SD_STATE_READY;

        return HAL_OK;
    }
    else
    {
        SDEX_DEBUG("error not ready\n");
        hsd->ErrorCode |= HAL_SD_ERROR_BUSY;
        return HAL_ERROR;
    }
}

/**
  * @brief  Allows to write block(s) to a specified address in a card. The Data
  *         transfer is managed by polling mode.
  * @note   This API should be followed by a check on the card state through
  *         HAL_SD_GetCardState().
  * @param  hsd Pointer to SD handle
  * @param  pData pointer to the buffer that will contain the data to transmit
  * @param  BlockAdd Block Address where data will be written
  * @param  NumberOfBlocks Number of SD blocks to write
  * @param  Timeout Specify timeout value
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SD_WriteBlocks(SD_HandleTypeDef *hsd, uint8_t *pData, uint32_t BlockAdd, uint32_t NumberOfBlocks, uint32_t Timeout)
{
    SDIO_DataInitTypeDef config;
    uint32_t errorstate = HAL_SD_ERROR_NONE;
    uint32_t *tempbuff = (uint32_t *)pData;

    if (NULL == pData)
    {
        SDEX_DEBUG("error HAL_SD_WriteBlocks no data buffer\n");
        hsd->ErrorCode |= HAL_SD_ERROR_PARAM;
        return HAL_ERROR;
    }

    if (hsd->State == HAL_SD_STATE_READY)
    {
        hsd->ErrorCode = HAL_DMA_ERROR_NONE;

        if (0) //((BlockAdd + NumberOfBlocks) > (hsd->SdCard.LogBlockNbr))
        {
            hsd->ErrorCode |= HAL_SD_ERROR_ADDR_OUT_OF_RANGE;
            return HAL_ERROR;
        }

        hsd->State = HAL_SD_STATE_BUSY;

        /* Initialize data control register */
        //hsd->Instance->DCTRL = 0U;

        if (hsd->SdCard.CardType != CARD_SDHC_SDXC)
        {
            BlockAdd *= 512U;
        }

        /* Set Block Size for Card */
        errorstate = SDMMC_CmdBlockLength(hsd->Instance, BLOCKSIZE);
        if (errorstate != HAL_SD_ERROR_NONE)
        {
            /* Clear all the static flags */
            //__HAL_SD_CLEAR_FLAG(hsd, SDIO_STATIC_FLAGS);
            hsd->ErrorCode |= errorstate;
            hsd->State = HAL_SD_STATE_READY;
            return HAL_ERROR;
        }

        /* Write Blocks in Polling mode */
        if (NumberOfBlocks > 1U)
        {
            hsd->Context = SD_CONTEXT_WRITE_MULTIPLE_BLOCK;

            /* Write Multi Block command */
            errorstate = SDMMC_CmdWriteMultiBlock(hsd->Instance, BlockAdd);
        }
        else
        {
            hsd->Context = SD_CONTEXT_WRITE_SINGLE_BLOCK;

            /* Write Single Block command */
            errorstate = SDMMC_CmdWriteSingleBlock(hsd->Instance, BlockAdd);
        }
        if (errorstate != HAL_SD_ERROR_NONE)
        {
            /* Clear all the static flags */
            //__HAL_SD_CLEAR_FLAG(hsd, SDIO_STATIC_FLAGS);
            hsd->ErrorCode |= errorstate;
            hsd->State = HAL_SD_STATE_READY;
            return HAL_ERROR;
        }

        /* Configure the SD DPSM (Data Path State Machine) */
        config.DataTimeOut   = 1000;
        config.DataLength    = NumberOfBlocks ;
        config.DataBlockSize = 512; //SDIO_DATABLOCK_SIZE_512B;
        config.TransferDir   = 1; //SDIO_TRANSFER_DIR_TO_CARD;
        config.TransferMode  = 0; //SDIO_TRANSFER_MODE_BLOCK;
        SDIO_ConfigData(hsd->Instance, &config);

        int intmask = hal_sdhci_data_finish(hsd->Instance, 1000);
        hal_sdhci_clear_int(hsd->Instance, intmask);
        if (intmask > 0)
        {
            if (intmask & (SDHCI_INT_DATA_AVAIL | SDHCI_INT_SPACE_AVAIL))
                hal_sdhci_transfer_pio(hsd->Instance, 0, (uint8_t *)tempbuff, (uint32_t)config.DataLength);
        }

        hsd->State = HAL_SD_STATE_READY;

        return HAL_OK;
    }
    else
    {
        hsd->ErrorCode |= HAL_SD_ERROR_BUSY;
        return HAL_ERROR;
    }
}

/**
  * @brief  Reads block(s) from a specified address in a card. The Data transfer
  *         is managed in interrupt mode.
  * @note   This API should be followed by a check on the card state through
  *         HAL_SD_GetCardState().
  * @note   You could also check the IT transfer process through the SD Rx
  *         interrupt event.
  * @param  hsd Pointer to SD handle
  * @param  pData Pointer to the buffer that will contain the received data
  * @param  BlockAdd Block Address from where data is to be read
  * @param  NumberOfBlocks Number of blocks to read.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SD_ReadBlocks_IT(SD_HandleTypeDef *hsd, uint8_t *pData, uint32_t BlockAdd, uint32_t NumberOfBlocks)
{
    SDIO_DataInitTypeDef config;
    uint32_t errorstate = HAL_SD_ERROR_NONE;

    if (NULL == pData)
    {
        hsd->ErrorCode |= HAL_SD_ERROR_PARAM;
        return HAL_ERROR;
    }

    if (hsd->State == HAL_SD_STATE_READY)
    {
        hsd->ErrorCode = HAL_DMA_ERROR_NONE;

        if ((BlockAdd + NumberOfBlocks) > (hsd->SdCard.LogBlockNbr))
        {
            hsd->ErrorCode |= HAL_SD_ERROR_ADDR_OUT_OF_RANGE;
            return HAL_ERROR;
        }

        hsd->State = HAL_SD_STATE_BUSY;

        /* Initialize data control register */
        //hsd->Instance->DCTRL = 0U;

        hsd->pRxBuffPtr = (uint32_t *)pData;
        hsd->RxXferSize = BLOCKSIZE * NumberOfBlocks;

        if (hsd->SdCard.CardType != CARD_SDHC_SDXC)
        {
            BlockAdd *= 512U;
        }

        /* Configure the SD DPSM (Data Path State Machine) */
        config.DataTimeOut   = 1000; //SDMMC_DATATIMEOUT;
        config.DataLength    = BLOCKSIZE * NumberOfBlocks;
        config.DataBlockSize = 512; //SDIO_DATABLOCK_SIZE_512B;
        config.TransferDir   = 0; //SDIO_TRANSFER_DIR_TO_SDIO;
        config.TransferMode  = 0; //SDIO_TRANSFER_MODE_BLOCK;
        SDIO_ConfigData(hsd->Instance, &config);

        /* Set Block Size for Card */
        errorstate = SDMMC_CmdBlockLength(hsd->Instance, BLOCKSIZE);
        if (errorstate != HAL_SD_ERROR_NONE)
        {
            /* Clear all the static flags */
            //__HAL_SD_CLEAR_FLAG(hsd, SDIO_STATIC_FLAGS);
            hsd->ErrorCode |= errorstate;
            hsd->State = HAL_SD_STATE_READY;
            return HAL_ERROR;
        }

        /* Read Blocks in IT mode */
        if (NumberOfBlocks > 1U)
        {
            hsd->Context = (SD_CONTEXT_READ_MULTIPLE_BLOCK | SD_CONTEXT_IT);

            /* Read Multi Block command */
            errorstate = SDMMC_CmdReadMultiBlock(hsd->Instance, BlockAdd);
        }
        else
        {
            hsd->Context = (SD_CONTEXT_READ_SINGLE_BLOCK | SD_CONTEXT_IT);

            /* Read Single Block command */
            errorstate = SDMMC_CmdReadSingleBlock(hsd->Instance, BlockAdd);
        }
        if (errorstate != HAL_SD_ERROR_NONE)
        {
            /* Clear all the static flags */
            //__HAL_SD_CLEAR_FLAG(hsd, SDIO_STATIC_FLAGS);
            hsd->ErrorCode |= errorstate;
            hsd->State = HAL_SD_STATE_READY;
            return HAL_ERROR;
        }

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

/**
  * @brief  Writes block(s) to a specified address in a card. The Data transfer
  *         is managed in interrupt mode.
  * @note   This API should be followed by a check on the card state through
  *         HAL_SD_GetCardState().
  * @note   You could also check the IT transfer process through the SD Tx
  *         interrupt event.
  * @param  hsd Pointer to SD handle
  * @param  pData Pointer to the buffer that will contain the data to transmit
  * @param  BlockAdd Block Address where data will be written
  * @param  NumberOfBlocks Number of blocks to write
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SD_WriteBlocks_IT(SD_HandleTypeDef *hsd, uint8_t *pData, uint32_t BlockAdd, uint32_t NumberOfBlocks)
{
    SDIO_DataInitTypeDef config;
    uint32_t errorstate = HAL_SD_ERROR_NONE;

    if (NULL == pData)
    {
        hsd->ErrorCode |= HAL_SD_ERROR_PARAM;
        return HAL_ERROR;
    }

    if (hsd->State == HAL_SD_STATE_READY)
    {
        hsd->ErrorCode = HAL_DMA_ERROR_NONE;

        if ((BlockAdd + NumberOfBlocks) > (hsd->SdCard.LogBlockNbr))
        {
            hsd->ErrorCode |= HAL_SD_ERROR_ADDR_OUT_OF_RANGE;
            return HAL_ERROR;
        }

        hsd->State = HAL_SD_STATE_BUSY;

        /* Initialize data control register */
        //hsd->Instance->DCTRL = 0U;

        hsd->pTxBuffPtr = (uint32_t *)pData;
        hsd->TxXferSize = BLOCKSIZE * NumberOfBlocks;

        /* Enable transfer interrupts */

        if (hsd->SdCard.CardType != CARD_SDHC_SDXC)
        {
            BlockAdd *= 512U;
        }

        /* Set Block Size for Card */
        errorstate = SDMMC_CmdBlockLength(hsd->Instance, BLOCKSIZE);
        if (errorstate != HAL_SD_ERROR_NONE)
        {
            /* Clear all the static flags */
            //__HAL_SD_CLEAR_FLAG(hsd, SDIO_STATIC_FLAGS);
            hsd->ErrorCode |= errorstate;
            hsd->State = HAL_SD_STATE_READY;
            return HAL_ERROR;
        }

        /* Write Blocks in Polling mode */
        if (NumberOfBlocks > 1U)
        {
            hsd->Context = (SD_CONTEXT_WRITE_MULTIPLE_BLOCK | SD_CONTEXT_IT);

            /* Write Multi Block command */
            errorstate = SDMMC_CmdWriteMultiBlock(hsd->Instance, BlockAdd);
        }
        else
        {
            hsd->Context = (SD_CONTEXT_WRITE_SINGLE_BLOCK | SD_CONTEXT_IT);

            /* Write Single Block command */
            errorstate = SDMMC_CmdWriteSingleBlock(hsd->Instance, BlockAdd);
        }
        if (errorstate != HAL_SD_ERROR_NONE)
        {
            /* Clear all the static flags */
            //__HAL_SD_CLEAR_FLAG(hsd, SDIO_STATIC_FLAGS);
            hsd->ErrorCode |= errorstate;
            hsd->State = HAL_SD_STATE_READY;
            return HAL_ERROR;
        }

        /* Configure the SD DPSM (Data Path State Machine) */
        config.DataTimeOut   = 1000; //SDMMC_DATATIMEOUT;
        config.DataLength    = NumberOfBlocks;
        config.DataBlockSize = 512; //SDIO_DATABLOCK_SIZE_512B;
        config.TransferDir   = 1; //SDIO_TRANSFER_DIR_TO_CARD;
        config.TransferMode  = 0; //SDIO_TRANSFER_MODE_BLOCK;
        SDIO_ConfigData(hsd->Instance, &config);

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

/**
  * @brief  Reads block(s) from a specified address in a card. The Data transfer
  *         is managed by DMA mode.
  * @note   This API should be followed by a check on the card state through
  *         HAL_SD_GetCardState().
  * @note   You could also check the DMA transfer process through the SD Rx
  *         interrupt event.
  * @param  hsd Pointer SD handle
  * @param  pData Pointer to the buffer that will contain the received data
  * @param  BlockAdd Block Address from where data is to be read
  * @param  NumberOfBlocks Number of blocks to read.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SD_ReadBlocks_DMA(SD_HandleTypeDef *hsd, uint8_t *pData, uint32_t BlockAdd, uint32_t NumberOfBlocks)
{
    SDIO_DataInitTypeDef config;
    uint32_t errorstate = HAL_SD_ERROR_NONE;

    if (NULL == pData)
    {
        hsd->ErrorCode |= HAL_SD_ERROR_PARAM;
        return HAL_ERROR;
    }

    if (hsd->State == HAL_SD_STATE_READY)
    {
        hsd->ErrorCode = HAL_DMA_ERROR_NONE;

        if ((BlockAdd + NumberOfBlocks) > (hsd->SdCard.LogBlockNbr))
        {
            hsd->ErrorCode |= HAL_SD_ERROR_ADDR_OUT_OF_RANGE;
            return HAL_ERROR;
        }

        hsd->State = HAL_SD_STATE_BUSY;

        /* Initialize data control register */
        //hsd->Instance->DCTRL = 0U;

        if (hsd->SdCard.CardType != CARD_SDHC_SDXC)
        {
            BlockAdd *= 512U;
        }

        /* Configure the SD DPSM (Data Path State Machine) */
        config.DataTimeOut   = 1000;
        config.DataLength    = NumberOfBlocks;
        config.DataBlockSize = 512; //SDIO_DATABLOCK_SIZE_512B;
        config.TransferDir   = 0; //SDIO_TRANSFER_DIR_TO_SDIO;
        config.TransferMode  = 1; //SDIO_TRANSFER_MODE_BLOCK;
        SDIO_ConfigData(hsd->Instance, &config);

        /* Set Block Size for Card */
        errorstate = SDMMC_CmdBlockLength(hsd->Instance, BLOCKSIZE);
        if (errorstate != HAL_SD_ERROR_NONE)
        {
            /* Clear all the static flags */
            //__HAL_SD_CLEAR_FLAG(hsd, SDIO_STATIC_FLAGS);
            hsd->ErrorCode |= errorstate;
            hsd->State = HAL_SD_STATE_READY;
            return HAL_ERROR;
        }

        /* Read Blocks in DMA mode */
        if (NumberOfBlocks > 1U)
        {
            hsd->Context = (SD_CONTEXT_READ_MULTIPLE_BLOCK | SD_CONTEXT_DMA);

            /* Read Multi Block command */
            errorstate = SDMMC_CmdReadMultiBlock(hsd->Instance, BlockAdd);
        }
        else
        {
            hsd->Context = (SD_CONTEXT_READ_SINGLE_BLOCK | SD_CONTEXT_DMA);

            /* Read Single Block command */
            errorstate = SDMMC_CmdReadSingleBlock(hsd->Instance, BlockAdd);
        }
        if (errorstate != HAL_SD_ERROR_NONE)
        {
            /* Clear all the static flags */
            //__HAL_SD_CLEAR_FLAG(hsd, SDIO_STATIC_FLAGS);
            hsd->ErrorCode |= errorstate;
            hsd->State = HAL_SD_STATE_READY;
            return HAL_ERROR;
        }

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

/**
  * @brief  Writes block(s) to a specified address in a card. The Data transfer
  *         is managed by DMA mode.
  * @note   This API should be followed by a check on the card state through
  *         HAL_SD_GetCardState().
  * @note   You could also check the DMA transfer process through the SD Tx
  *         interrupt event.
  * @param  hsd Pointer to SD handle
  * @param  pData Pointer to the buffer that will contain the data to transmit
  * @param  BlockAdd Block Address where data will be written
  * @param  NumberOfBlocks Number of blocks to write
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SD_WriteBlocks_DMA(SD_HandleTypeDef *hsd, uint8_t *pData, uint32_t BlockAdd, uint32_t NumberOfBlocks)
{
    SDIO_DataInitTypeDef config;
    uint32_t errorstate = HAL_SD_ERROR_NONE;

    if (NULL == pData)
    {
        hsd->ErrorCode |= HAL_SD_ERROR_PARAM;
        return HAL_ERROR;
    }

    if (hsd->State == HAL_SD_STATE_READY)
    {
        hsd->ErrorCode = HAL_DMA_ERROR_NONE;

        if ((BlockAdd + NumberOfBlocks) > (hsd->SdCard.LogBlockNbr))
        {
            hsd->ErrorCode |= HAL_SD_ERROR_ADDR_OUT_OF_RANGE;
            return HAL_ERROR;
        }

        hsd->State = HAL_SD_STATE_BUSY;

        /* Initialize data control register */
        //hsd->Instance->DCTRL = 0U;

        /* Enable SD Error interrupts */


        /* Set the DMA Abort callback */
        hsd->hdmatx->XferAbortCallback = NULL;

        if (hsd->SdCard.CardType != CARD_SDHC_SDXC)
        {
            BlockAdd *= 512U;
        }

        /* Set Block Size for Card */
        errorstate = SDMMC_CmdBlockLength(hsd->Instance, BLOCKSIZE);
        if (errorstate != HAL_SD_ERROR_NONE)
        {
            /* Clear all the static flags */
            //__HAL_SD_CLEAR_FLAG(hsd, SDIO_STATIC_FLAGS);
            hsd->ErrorCode |= errorstate;
            hsd->State = HAL_SD_STATE_READY;
            return HAL_ERROR;
        }

        /* Write Blocks in Polling mode */
        if (NumberOfBlocks > 1U)
        {
            hsd->Context = (SD_CONTEXT_WRITE_MULTIPLE_BLOCK | SD_CONTEXT_DMA);

            /* Write Multi Block command */
            errorstate = SDMMC_CmdWriteMultiBlock(hsd->Instance, BlockAdd);
        }
        else
        {
            hsd->Context = (SD_CONTEXT_WRITE_SINGLE_BLOCK | SD_CONTEXT_DMA);

            /* Write Single Block command */
            errorstate = SDMMC_CmdWriteSingleBlock(hsd->Instance, BlockAdd);
        }
        if (errorstate != HAL_SD_ERROR_NONE)
        {
            /* Clear all the static flags */
            //__HAL_SD_CLEAR_FLAG(hsd, SDIO_STATIC_FLAGS);
            hsd->ErrorCode |= errorstate;
            hsd->State = HAL_SD_STATE_READY;
            return HAL_ERROR;
        }

        /* Enable SDIO DMA transfer */

        /* Enable the DMA Channel */

        /* Configure the SD DPSM (Data Path State Machine) */
        config.DataTimeOut   = 1000;
        config.DataLength    = NumberOfBlocks;
        config.DataBlockSize = 512; //SDIO_DATABLOCK_SIZE_512B;
        config.TransferDir   = 1; //SDIO_TRANSFER_DIR_TO_CARD;
        config.TransferMode  = 0; //SDIO_TRANSFER_MODE_BLOCK;
        SDIO_ConfigData(hsd->Instance, &config);

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}
/*
  * @brief  This function handles SD card interrupt request.
  * @param  hsd Pointer to SD handle
  * @retval None
  */
void HAL_SD_IRQHandler(SD_HandleTypeDef *hsd)
{
    //uint32_t errorstate = HAL_SD_ERROR_NONE;
}

/**
  * @brief return the SD state
  * @param hsd Pointer to sd handle
  * @retval HAL state
  */
HAL_SD_StateTypeDef HAL_SD_GetState(SD_HandleTypeDef *hsd)
{
    return hsd->State;
}

/**
* @brief  Return the SD error code
* @param  hsd  Pointer to a SD_HandleTypeDef structure that contains
  *              the configuration information.
* @retval SD Error Code
*/
uint32_t HAL_SD_GetError(SD_HandleTypeDef *hsd)
{
    return hsd->ErrorCode;
}

/**
  * @brief Tx Transfer completed callbacks
  * @param hsd Pointer to SD handle
  * @retval None
  */
__weak void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hsd);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_SD_TxCpltCallback can be implemented in the user file
     */
}

/**
  * @brief Rx Transfer completed callbacks
  * @param hsd Pointer SD handle
  * @retval None
  */
__weak void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hsd);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_SD_RxCpltCallback can be implemented in the user file
     */
}

/**
  * @brief SD error callbacks
  * @param hsd Pointer SD handle
  * @retval None
  */
__weak void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hsd);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_SD_ErrorCallback can be implemented in the user file
     */
}

/**
  * @brief SD Abort callbacks
  * @param hsd Pointer SD handle
  * @retval None
  */
__weak void HAL_SD_AbortCallback(SD_HandleTypeDef *hsd)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hsd);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_SD_ErrorCallback can be implemented in the user file
     */
}


/**
  * @}
  */

/** @addtogroup SD_Exported_Functions_Group3
 *  @brief   management functions
 *
@verbatim
  ==============================================================================
                      ##### Peripheral Control functions #####
  ==============================================================================
  [..]
    This subsection provides a set of functions allowing to control the SD card
    operations and get the related information

@endverbatim
  * @{
  */

/**
  * @brief  Returns information the information of the card which are stored on
  *         the CID register.
  * @param  hsd Pointer to SD handle
  * @param  pCID Pointer to a HAL_SD_CIDTypeDef structure that
  *         contains all CID register parameters
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SD_GetCardCID(SD_HandleTypeDef *hsd, HAL_SD_CardCIDTypeDef *pCID)
{
    uint32_t tmp = 0U;

    /* Byte 0 */
    tmp = (uint8_t)((hsd->CID[0U] & 0xFF000000U) >> 24U);
    pCID->ManufacturerID = tmp;

    /* Byte 1 */
    tmp = (uint8_t)((hsd->CID[0U] & 0x00FF0000U) >> 16U);
    pCID->OEM_AppliID = tmp << 8U;

    /* Byte 2 */
    tmp = (uint8_t)((hsd->CID[0U] & 0x000000FF00U) >> 8U);
    pCID->OEM_AppliID |= tmp;

    /* Byte 3 */
    tmp = (uint8_t)(hsd->CID[0U] & 0x000000FFU);
    pCID->ProdName1 = tmp << 24U;

    /* Byte 4 */
    tmp = (uint8_t)((hsd->CID[1U] & 0xFF000000U) >> 24U);
    pCID->ProdName1 |= tmp << 16;

    /* Byte 5 */
    tmp = (uint8_t)((hsd->CID[1U] & 0x00FF0000U) >> 16U);
    pCID->ProdName1 |= tmp << 8U;

    /* Byte 6 */
    tmp = (uint8_t)((hsd->CID[1U] & 0x0000FF00U) >> 8U);
    pCID->ProdName1 |= tmp;

    /* Byte 7 */
    tmp = (uint8_t)(hsd->CID[1U] & 0x000000FFU);
    pCID->ProdName2 = tmp;

    /* Byte 8 */
    tmp = (uint8_t)((hsd->CID[2U] & 0xFF000000U) >> 24U);
    pCID->ProdRev = tmp;

    /* Byte 9 */
    tmp = (uint8_t)((hsd->CID[2U] & 0x00FF0000U) >> 16U);
    pCID->ProdSN = tmp << 24U;

    /* Byte 10 */
    tmp = (uint8_t)((hsd->CID[2U] & 0x0000FF00U) >> 8U);
    pCID->ProdSN |= tmp << 16U;

    /* Byte 11 */
    tmp = (uint8_t)(hsd->CID[2U] & 0x000000FFU);
    pCID->ProdSN |= tmp << 8U;

    /* Byte 12 */
    tmp = (uint8_t)((hsd->CID[3U] & 0xFF000000U) >> 24U);
    pCID->ProdSN |= tmp;

    /* Byte 13 */
    tmp = (uint8_t)((hsd->CID[3U] & 0x00FF0000U) >> 16U);
    pCID->Reserved1   |= (tmp & 0xF0U) >> 4U;
    pCID->ManufactDate = (tmp & 0x0FU) << 8U;

    /* Byte 14 */
    tmp = (uint8_t)((hsd->CID[3U] & 0x0000FF00U) >> 8U);
    pCID->ManufactDate |= tmp;

    /* Byte 15 */
    tmp = (uint8_t)(hsd->CID[3U] & 0x000000FFU);
    pCID->CID_CRC   = (tmp & 0xFEU) >> 1U;
    pCID->Reserved2 = 1U;

    return HAL_OK;
}

/**
  * @brief  Returns information the information of the card which are stored on
  *         the CSD register.
  * @param  hsd Pointer to SD handle
  * @param  pCSD Pointer to a HAL_SD_CardCSDTypeDef structure that
  *         contains all CSD register parameters
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SD_GetCardCSD(SD_HandleTypeDef *hsd, HAL_SD_CardCSDTypeDef *pCSD)
{
    uint32_t tmp = 0U;

    /* Byte 0 */
    tmp = (hsd->CSD[0U] & 0xFF000000U) >> 24U;
    pCSD->CSDStruct      = (uint8_t)((tmp & 0xC0U) >> 6U);
    pCSD->SysSpecVersion = (uint8_t)((tmp & 0x3CU) >> 2U);
    pCSD->Reserved1      = tmp & 0x03U;

    /* Byte 1 */
    tmp = (hsd->CSD[0U] & 0x00FF0000U) >> 16U;
    pCSD->TAAC = (uint8_t)tmp;

    /* Byte 2 */
    tmp = (hsd->CSD[0U] & 0x0000FF00U) >> 8U;
    pCSD->NSAC = (uint8_t)tmp;

    /* Byte 3 */
    tmp = hsd->CSD[0U] & 0x000000FFU;
    pCSD->MaxBusClkFrec = (uint8_t)tmp;

    /* Byte 4 */
    tmp = (hsd->CSD[1U] & 0xFF000000U) >> 24U;
    pCSD->CardComdClasses = (uint16_t)(tmp << 4U);

    /* Byte 5 */
    tmp = (hsd->CSD[1U] & 0x00FF0000U) >> 16U;
    pCSD->CardComdClasses |= (uint16_t)((tmp & 0xF0U) >> 4U);
    pCSD->RdBlockLen       = (uint8_t)(tmp & 0x0FU);

    /* Byte 6 */
    tmp = (hsd->CSD[1U] & 0x0000FF00U) >> 8U;
    pCSD->PartBlockRead   = (uint8_t)((tmp & 0x80U) >> 7U);
    pCSD->WrBlockMisalign = (uint8_t)((tmp & 0x40U) >> 6U);
    pCSD->RdBlockMisalign = (uint8_t)((tmp & 0x20U) >> 5U);
    pCSD->DSRImpl         = (uint8_t)((tmp & 0x10U) >> 4U);
    pCSD->Reserved2       = 0U; /*!< Reserved */

    SDEX_DEBUG("hsd->SdCard.CardType  = %d\n", hsd->SdCard.CardType);
    if (hsd->SdCard.CardType == CARD_SDSC)
    {
        pCSD->DeviceSize = (tmp & 0x03U) << 10U;

        /* Byte 7 */
        tmp = (uint8_t)(hsd->CSD[1U] & 0x000000FFU);
        pCSD->DeviceSize |= (tmp) << 2U;

        /* Byte 8 */
        tmp = (uint8_t)((hsd->CSD[2U] & 0xFF000000U) >> 24U);
        pCSD->DeviceSize |= (tmp & 0xC0U) >> 6U;

        pCSD->MaxRdCurrentVDDMin = (tmp & 0x38U) >> 3U;
        pCSD->MaxRdCurrentVDDMax = (tmp & 0x07U);

        /* Byte 9 */
        tmp = (uint8_t)((hsd->CSD[2U] & 0x00FF0000U) >> 16U);
        pCSD->MaxWrCurrentVDDMin = (tmp & 0xE0U) >> 5U;
        pCSD->MaxWrCurrentVDDMax = (tmp & 0x1CU) >> 2U;
        pCSD->DeviceSizeMul      = (tmp & 0x03U) << 1U;
        /* Byte 10 */
        tmp = (uint8_t)((hsd->CSD[2U] & 0x0000FF00U) >> 8U);
        pCSD->DeviceSizeMul |= (tmp & 0x80U) >> 7U;

        hsd->SdCard.BlockNbr  = (pCSD->DeviceSize + 1U) ;
        hsd->SdCard.BlockNbr *= (1U << (pCSD->DeviceSizeMul + 2U));
        hsd->SdCard.BlockSize = 1U << (pCSD->RdBlockLen);

        hsd->SdCard.LogBlockNbr = (hsd->SdCard.BlockNbr) * ((hsd->SdCard.BlockSize) / 512U);
        hsd->SdCard.LogBlockSize = 512U;
        SDEX_DEBUG("CARD: blk %d, cnt %d\n", hsd->SdCard.BlockSize, hsd->SdCard.BlockNbr);
    }
    else if (hsd->SdCard.CardType == CARD_SDHC_SDXC)
    {
        /* Byte 7 */
        tmp = (uint8_t)(hsd->CSD[1U] & 0x000000FFU);
        pCSD->DeviceSize = (tmp & 0x3FU) << 16U;

        /* Byte 8 */
        tmp = (uint8_t)((hsd->CSD[2U] & 0xFF000000U) >> 24U);

        pCSD->DeviceSize |= (tmp << 8U);

        /* Byte 9 */
        tmp = (uint8_t)((hsd->CSD[2U] & 0x00FF0000U) >> 16U);

        pCSD->DeviceSize |= (tmp);

        /* Byte 10 */
        tmp = (uint8_t)((hsd->CSD[2U] & 0x0000FF00U) >> 8U);

        hsd->SdCard.LogBlockNbr = hsd->SdCard.BlockNbr = (((uint64_t)pCSD->DeviceSize + 1U) * 1024U);
        hsd->SdCard.LogBlockSize = hsd->SdCard.BlockSize = 512U;
        SDEX_DEBUG("SDXC: blk %d, cnt %d\n", hsd->SdCard.BlockSize, hsd->SdCard.BlockNbr);
    }
    else
    {
        /* Clear all the static flags */
        //__HAL_SD_CLEAR_FLAG(hsd, SDIO_STATIC_FLAGS);
        hsd->ErrorCode |= HAL_SD_ERROR_UNSUPPORTED_FEATURE;
        hsd->State = HAL_SD_STATE_READY;
        return HAL_ERROR;
    }

    pCSD->EraseGrSize = (tmp & 0x40U) >> 6U;
    pCSD->EraseGrMul  = (tmp & 0x3FU) << 1U;

    /* Byte 11 */
    tmp = (uint8_t)(hsd->CSD[2U] & 0x000000FFU);
    pCSD->EraseGrMul     |= (tmp & 0x80U) >> 7U;
    pCSD->WrProtectGrSize = (tmp & 0x7FU);

    /* Byte 12 */
    tmp = (uint8_t)((hsd->CSD[3U] & 0xFF000000U) >> 24U);
    pCSD->WrProtectGrEnable = (tmp & 0x80U) >> 7U;
    pCSD->ManDeflECC        = (tmp & 0x60U) >> 5U;
    pCSD->WrSpeedFact       = (tmp & 0x1CU) >> 2U;
    pCSD->MaxWrBlockLen     = (tmp & 0x03U) << 2U;

    /* Byte 13 */
    tmp = (uint8_t)((hsd->CSD[3U] & 0x00FF0000U) >> 16U);
    pCSD->MaxWrBlockLen      |= (tmp & 0xC0U) >> 6U;
    pCSD->WriteBlockPaPartial = (tmp & 0x20U) >> 5U;
    pCSD->Reserved3           = 0U;
    pCSD->ContentProtectAppli = (tmp & 0x01U);

    /* Byte 14 */
    tmp = (uint8_t)((hsd->CSD[3U] & 0x0000FF00U) >> 8U);
    pCSD->FileFormatGrouop = (tmp & 0x80U) >> 7U;
    pCSD->CopyFlag         = (tmp & 0x40U) >> 6U;
    pCSD->PermWrProtect    = (tmp & 0x20U) >> 5U;
    pCSD->TempWrProtect    = (tmp & 0x10U) >> 4U;
    pCSD->FileFormat       = (tmp & 0x0CU) >> 2U;
    pCSD->ECC              = (tmp & 0x03U);

    /* Byte 15 */
    tmp = (uint8_t)(hsd->CSD[3U] & 0x000000FFU);
    pCSD->CSD_CRC   = (tmp & 0xFEU) >> 1U;
    pCSD->Reserved4 = 1U;

    return HAL_OK;
}

/**
  * @brief  Gets the SD status info.
  * @param  hsd Pointer to SD handle
  * @param  pStatus Pointer to the HAL_SD_CardStatusTypeDef structure that
  *         will contain the SD card status information
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SD_GetCardStatus(SD_HandleTypeDef *hsd, HAL_SD_CardStatusTypeDef *pStatus)
{
    uint32_t tmp = 0U;
    uint32_t sd_status[16U];
    uint32_t errorstate = HAL_SD_ERROR_NONE;

    errorstate = SD_SendSDStatus(hsd, sd_status);
    if (errorstate != HAL_OK)
    {
        /* Clear all the static flags */
        //__HAL_SD_CLEAR_FLAG(hsd, SDIO_STATIC_FLAGS);
        hsd->ErrorCode |= errorstate;
        hsd->State = HAL_SD_STATE_READY;
        return HAL_ERROR;
    }
    else
    {
        /* Byte 0 */
        tmp = (sd_status[0U] & 0xC0U) >> 6U;
        pStatus->DataBusWidth = (uint8_t)tmp;

        /* Byte 0 */
        tmp = (sd_status[0U] & 0x20U) >> 5U;
        pStatus->SecuredMode = (uint8_t)tmp;

        /* Byte 2 */
        tmp = (sd_status[0U] & 0x00FF0000U) >> 16U;
        pStatus->CardType = (uint16_t)(tmp << 8U);

        /* Byte 3 */
        tmp = (sd_status[0U] & 0xFF000000U) >> 24U;
        pStatus->CardType |= (uint16_t)tmp;

        /* Byte 4 */
        tmp = (sd_status[1U] & 0xFFU);
        pStatus->ProtectedAreaSize = (uint32_t)(tmp << 24U);

        /* Byte 5 */
        tmp = (sd_status[1U] & 0xFF00U) >> 8U;
        pStatus->ProtectedAreaSize |= (uint32_t)(tmp << 16U);

        /* Byte 6 */
        tmp = (sd_status[1U] & 0xFF0000U) >> 16U;
        pStatus->ProtectedAreaSize |= (uint32_t)(tmp << 8U);

        /* Byte 7 */
        tmp = (sd_status[1U] & 0xFF000000U) >> 24U;
        pStatus->ProtectedAreaSize |= (uint32_t)tmp;

        /* Byte 8 */
        tmp = (sd_status[2U] & 0xFFU);
        pStatus->SpeedClass = (uint8_t)tmp;

        /* Byte 9 */
        tmp = (sd_status[2U] & 0xFF00U) >> 8U;
        pStatus->PerformanceMove = (uint8_t)tmp;

        /* Byte 10 */
        tmp = (sd_status[2U] & 0xF00000U) >> 20U;
        pStatus->AllocationUnitSize = (uint8_t)tmp;

        /* Byte 11 */
        tmp = (sd_status[2U] & 0xFF000000U) >> 24U;
        pStatus->EraseSize = (uint16_t)(tmp << 8U);

        /* Byte 12 */
        tmp = (sd_status[3U] & 0xFFU);
        pStatus->EraseSize |= (uint16_t)tmp;

        /* Byte 13 */
        tmp = (sd_status[3U] & 0xFC00U) >> 10U;
        pStatus->EraseTimeout = (uint8_t)tmp;

        /* Byte 13 */
        tmp = (sd_status[3U] & 0x0300U) >> 8U;
        pStatus->EraseOffset = (uint8_t)tmp;
    }

    return HAL_OK;
}

/**
  * @brief  Gets the SD card info.
  * @param  hsd Pointer to SD handle
  * @param  pCardInfo Pointer to the HAL_SD_CardInfoTypeDef structure that
  *         will contain the SD card status information
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SD_GetCardInfo(SD_HandleTypeDef *hsd, HAL_SD_CardInfoTypeDef *pCardInfo)
{
    pCardInfo->CardType     = (uint32_t)(hsd->SdCard.CardType);
    pCardInfo->CardVersion  = (uint32_t)(hsd->SdCard.CardVersion);
    pCardInfo->Class        = (uint32_t)(hsd->SdCard.Class);
    pCardInfo->RelCardAdd   = (uint32_t)(hsd->SdCard.RelCardAdd);
    pCardInfo->BlockNbr     = (uint32_t)(hsd->SdCard.BlockNbr);
    pCardInfo->BlockSize    = (uint32_t)(hsd->SdCard.BlockSize);
    pCardInfo->LogBlockNbr  = (uint32_t)(hsd->SdCard.LogBlockNbr);
    pCardInfo->LogBlockSize = (uint32_t)(hsd->SdCard.LogBlockSize);

    return HAL_OK;
}

/**
  * @brief  Enables wide bus operation for the requested card if supported by
  *         card.
  * @param  hsd Pointer to SD handle
  * @param  WideMode Specifies the SD card wide bus mode
  *          This parameter can be one of the following values:
  *            @arg SDIO_BUS_WIDE_8B: 8-bit data transfer
  *            @arg SDIO_BUS_WIDE_4B: 4-bit data transfer
  *            @arg SDIO_BUS_WIDE_1B: 1-bit data transfer
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SD_ConfigWideBusOperation(SD_HandleTypeDef *hsd, uint32_t WideMode)
{
    SDIO_InitTypeDef Init;
    uint32_t errorstate = HAL_SD_ERROR_NONE;

    /* Check the parameters */

    /* Chnage Satte */
    hsd->State = HAL_SD_STATE_BUSY;

    if (hsd->SdCard.CardType != CARD_SECURED)
    {
        if (WideMode == 8)
        {
            hsd->ErrorCode |= HAL_SD_ERROR_UNSUPPORTED_FEATURE;
        }
        else if (WideMode == 4)
        {
            errorstate = SD_WideBus_Enable(hsd);
            SDEX_DEBUG("set to 4bit res: 0x%x\n", errorstate);

            hsd->ErrorCode |= errorstate;
        }
        else if (WideMode == 1)
        {
            errorstate = SD_WideBus_Disable(hsd);
            SDEX_DEBUG("set to 1bit res: 0x%x\n", errorstate);

            hsd->ErrorCode |= errorstate;
        }
        else
        {
            /* WideMode is not a valid argument*/
            hsd->ErrorCode |= HAL_SD_ERROR_PARAM;
        }
    }
    else
    {
        /* MMC Card does not support this feature */
        hsd->ErrorCode |= HAL_SD_ERROR_UNSUPPORTED_FEATURE;
    }

    if (hsd->ErrorCode != HAL_SD_ERROR_NONE)
    {
        /* Clear all the static flags */
        //__HAL_SD_CLEAR_FLAG(hsd, SDIO_STATIC_FLAGS);
        hsd->State = HAL_SD_STATE_READY;
        return HAL_ERROR;
    }
    else
    {
        /* Configure the SDIO peripheral */
        Init.ClockEdge           = hsd->Init.ClockEdge;
        Init.ClockBypass         = hsd->Init.ClockBypass;
        Init.ClockPowerSave      = hsd->Init.ClockPowerSave;
        Init.BusWide             = WideMode;
        Init.HardwareFlowControl = hsd->Init.HardwareFlowControl;
        Init.ClockDiv            = hsd->Init.ClockDiv;
        SDIO_Init(hsd->Instance, Init);
    }

    /* Change State */
    hsd->State = HAL_SD_STATE_READY;

    return HAL_OK;
}


/**
  * @brief  Gets the current sd card data state.
  * @param  hsd pointer to SD handle
  * @retval Card state
  */
HAL_SD_CardStateTypeDef HAL_SD_GetCardState(SD_HandleTypeDef *hsd)
{
    HAL_SD_CardStateTypeDef cardstate =  HAL_SD_CARD_TRANSFER;
    uint32_t errorstate = HAL_SD_ERROR_NONE;
    uint32_t resp1 = 0;

    errorstate = SD_SendStatus(hsd, &resp1);
    if (errorstate != HAL_OK)
    {
        hsd->ErrorCode |= errorstate;
    }

    cardstate = (HAL_SD_CardStateTypeDef)((resp1 >> 9U) & 0x0FU);

    return cardstate;
}

/**
  * @brief  Abort the current transfer and disable the SD.
  * @param  hsd pointer to a SD_HandleTypeDef structure that contains
  *                the configuration information for SD module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SD_Abort(SD_HandleTypeDef *hsd)
{
    HAL_SD_CardStateTypeDef CardState;

    /* DIsable All interrupts */

    hsd->State = HAL_SD_STATE_READY;
    CardState = HAL_SD_GetCardState(hsd);
    if ((CardState == HAL_SD_CARD_RECEIVING) || (CardState == HAL_SD_CARD_SENDING))
    {
        hsd->ErrorCode = SDMMC_CmdStopTransfer(hsd->Instance);
    }
    if (hsd->ErrorCode != HAL_SD_ERROR_NONE)
    {
        return HAL_ERROR;
    }
    return HAL_OK;
}

/**
  * @brief  Abort the current transfer and disable the SD (IT mode).
  * @param  hsd pointer to a SD_HandleTypeDef structure that contains
  *                the configuration information for SD module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SD_Abort_IT(SD_HandleTypeDef *hsd)
{
    HAL_SD_CardStateTypeDef CardState;

    /* DIsable All interrupts */

    /* No transfer ongoing on both DMA channels*/
    if ((hsd->hdmatx == NULL) && (hsd->hdmarx == NULL))
    {
        CardState = HAL_SD_GetCardState(hsd);
        hsd->State = HAL_SD_STATE_READY;
        if ((CardState == HAL_SD_CARD_RECEIVING) || (CardState == HAL_SD_CARD_SENDING))
        {
            hsd->ErrorCode = SDMMC_CmdStopTransfer(hsd->Instance);
        }
        if (hsd->ErrorCode != HAL_SD_ERROR_NONE)
        {
            return HAL_ERROR;
        }
        else
        {
            HAL_SD_AbortCallback(hsd);
        }
    }

    return HAL_OK;
}

/**
  * @}
  */

/**
  * @}
  */

/* Private function ----------------------------------------------------------*/
/** @addtogroup SD_Private_Functions
  * @{
  */
#if 0
/**
  * @brief  DMA SD transmit process complete callback
  * @param  hdma DMA handle
  * @retval None
  */
static void SD_DMATransmitCplt(DMA_HandleTypeDef *hdma)
{
    //SD_HandleTypeDef *hsd = (SD_HandleTypeDef *)(hdma->Parent);

    /* Enable DATAEND Interrupt */
    //__HAL_SD_ENABLE_IT(hsd, (SDIO_IT_DATAEND));
}

/**
  * @brief  DMA SD receive process complete callback
  * @param  hdma DMA handle
  * @retval None
  */
static void SD_DMAReceiveCplt(DMA_HandleTypeDef *hdma)
{
    SD_HandleTypeDef *hsd = (SD_HandleTypeDef *)(hdma->Parent);
    uint32_t errorstate = HAL_SD_ERROR_NONE;

    /* Send stop command in multiblock write */
    if (hsd->Context == (SD_CONTEXT_READ_MULTIPLE_BLOCK | SD_CONTEXT_DMA))
    {
        errorstate = SDMMC_CmdStopTransfer(hsd->Instance);
        if (errorstate != HAL_SD_ERROR_NONE)
        {
            hsd->ErrorCode |= errorstate;
            HAL_SD_ErrorCallback(hsd);
        }
    }

    /* Disable the DMA transfer for transmit request by setting the DMAEN bit
    in the SD DCTRL register */
    //hsd->Instance->DCTRL &= (uint32_t)~((uint32_t)SDIO_DCTRL_DMAEN);

    /* Clear all the static flags */
    //__HAL_SD_CLEAR_FLAG(hsd, SDIO_STATIC_FLAGS);

    hsd->State = HAL_SD_STATE_READY;

    HAL_SD_RxCpltCallback(hsd);
}

/**
  * @brief  DMA SD communication error callback
  * @param  hdma DMA handle
  * @retval None
  */
static void SD_DMAError(DMA_HandleTypeDef *hdma)
{
    SD_HandleTypeDef *hsd = (SD_HandleTypeDef *)(hdma->Parent);
    HAL_SD_CardStateTypeDef CardState;

    if ((hsd->hdmarx->ErrorCode == HAL_DMA_ERROR_TE) || (hsd->hdmatx->ErrorCode == HAL_DMA_ERROR_TE))
    {

        hsd->ErrorCode |= HAL_SD_ERROR_DMA;
        CardState = HAL_SD_GetCardState(hsd);
        if ((CardState == HAL_SD_CARD_RECEIVING) || (CardState == HAL_SD_CARD_SENDING))
        {
            hsd->ErrorCode |= SDMMC_CmdStopTransfer(hsd->Instance);
        }

        hsd->State = HAL_SD_STATE_READY;
    }

    HAL_SD_ErrorCallback(hsd);
}

/**
  * @brief  DMA SD Tx Abort callback
  * @param  hdma DMA handle
  * @retval None
  */
static void SD_DMATxAbort(DMA_HandleTypeDef *hdma)
{
    SD_HandleTypeDef *hsd = (SD_HandleTypeDef *)(hdma->Parent);
    HAL_SD_CardStateTypeDef CardState;

    if (hsd->hdmatx != NULL)
    {
        hsd->hdmatx = NULL;
    }

    /* All DMA channels are aborted */
    if (hsd->hdmarx == NULL)
    {
        CardState = HAL_SD_GetCardState(hsd);
        hsd->ErrorCode = HAL_SD_ERROR_NONE;
        hsd->State = HAL_SD_STATE_READY;
        if ((CardState == HAL_SD_CARD_RECEIVING) || (CardState == HAL_SD_CARD_SENDING))
        {
            hsd->ErrorCode |= SDMMC_CmdStopTransfer(hsd->Instance);

            if (hsd->ErrorCode != HAL_SD_ERROR_NONE)
            {
                HAL_SD_AbortCallback(hsd);
            }
            else
            {
                HAL_SD_ErrorCallback(hsd);
            }
        }
    }
}

/**
  * @brief  DMA SD Rx Abort callback
  * @param  hdma DMA handle
  * @retval None
  */
static void SD_DMARxAbort(DMA_HandleTypeDef *hdma)
{
    SD_HandleTypeDef *hsd = (SD_HandleTypeDef *)(hdma->Parent);
    HAL_SD_CardStateTypeDef CardState;

    if (hsd->hdmarx != NULL)
    {
        hsd->hdmarx = NULL;
    }

    /* All DMA channels are aborted */
    if (hsd->hdmatx == NULL)
    {
        CardState = HAL_SD_GetCardState(hsd);
        hsd->ErrorCode = HAL_SD_ERROR_NONE;
        hsd->State = HAL_SD_STATE_READY;
        if ((CardState == HAL_SD_CARD_RECEIVING) || (CardState == HAL_SD_CARD_SENDING))
        {
            hsd->ErrorCode |= SDMMC_CmdStopTransfer(hsd->Instance);

            if (hsd->ErrorCode != HAL_SD_ERROR_NONE)
            {
                HAL_SD_AbortCallback(hsd);
            }
            else
            {
                HAL_SD_ErrorCallback(hsd);
            }
        }
    }
}
#endif

/**
  * @brief  Initializes the sd card.
  * @param  hsd Pointer to SD handle
  * @retval SD Card error state
  */
static uint32_t SD_InitCard(SD_HandleTypeDef *hsd)
{
    HAL_SD_CardCSDTypeDef CSD;
    uint32_t errorstate = HAL_SD_ERROR_NONE;
    uint16_t sd_rca = 1U;

    /* Check the power State */
    if (SDIO_GetPowerState(hsd->Instance) == 0U)
    {
        /* Power off */
        SDEX_DEBUG("HAL_SD_ERROR_REQUEST_NOT_APPLICABLE\n");
        return HAL_SD_ERROR_REQUEST_NOT_APPLICABLE;
    }

    if (hsd->SdCard.CardType != CARD_SECURED)
    {
        /* Send CMD2 ALL_SEND_CID */
        errorstate = SDMMC_CmdSendCID(hsd->Instance);
        if (errorstate != HAL_SD_ERROR_NONE)
        {
            SDEX_DEBUG("SDMMC_CmdSendCID %d\n", errorstate);
            return errorstate;
        }
        else
        {
            /* Get Card identification number data */
            hsd->CID[0U] = SDIO_GetResponse(hsd->Instance, 0);
            hsd->CID[1U] = SDIO_GetResponse(hsd->Instance, 1);
            hsd->CID[2U] = SDIO_GetResponse(hsd->Instance, 2);
            hsd->CID[3U] = SDIO_GetResponse(hsd->Instance, 3);
        }
    }
#if 1
    if (hsd->SdCard.CardType != CARD_SECURED)
    {
        /* Send CMD3 SET_REL_ADDR with argument 0 */
        /* SD Card publishes its RCA. */
        errorstate = SDMMC_CmdSetRelAdd(hsd->Instance, &sd_rca);
        if (errorstate != HAL_SD_ERROR_NONE)
        {
            SDEX_DEBUG("SDMMC_CmdSetRelAdd %d\n", errorstate);
            return errorstate;
        }
    }
#endif
    if (hsd->SdCard.CardType != CARD_SECURED)
    {
        /* Get the SD card RCA */
        hsd->SdCard.RelCardAdd = sd_rca;
        //hsd->SdCard.RelCardAdd = 1;
        SDEX_DEBUG("RelCardAdd = %d\n", sd_rca);

        /* Send CMD9 SEND_CSD with argument as card's RCA */
        errorstate = SDMMC_CmdSendCSD(hsd->Instance, (uint32_t)(hsd->SdCard.RelCardAdd << 16U));
        if (errorstate != HAL_SD_ERROR_NONE)
        {
            SDEX_DEBUG("SDMMC_CmdSendCSD %d\n", errorstate);
            return errorstate;
        }
        else
        {
            /* Get Card Specific Data */
            hsd->CSD[0U] = SDIO_GetResponse(hsd->Instance, 0);
            hsd->CSD[1U] = SDIO_GetResponse(hsd->Instance, 1);
            hsd->CSD[2U] = SDIO_GetResponse(hsd->Instance, 2);
            hsd->CSD[3U] = SDIO_GetResponse(hsd->Instance, 3);
        }
    }

    /* Get the Card Class */
    hsd->SdCard.Class = (SDIO_GetResponse(hsd->Instance, 1) >> 20U);
    SDEX_DEBUG("hsd->SdCard.Class = %d\n", hsd->SdCard.Class);

    /* Get CSD parameters */
    HAL_SD_GetCardCSD(hsd, &CSD);

    /* Select the Card */
    errorstate = SDMMC_CmdSelDesel(hsd->Instance, (uint32_t)(((uint32_t)hsd->SdCard.RelCardAdd) << 16U));
    if (errorstate != HAL_SD_ERROR_NONE)
    {
        SDEX_DEBUG("SDMMC_CmdSelDesel %d\n", errorstate);
        return errorstate;
    }

    /* Configure SDIO peripheral interface */
    //SDIO_Init(hsd->Instance, hsd->Init);

    /* All cards are initialized */
    return HAL_SD_ERROR_NONE;
}

/**
  * @brief  Enquires cards about their operating voltage and configures clock
  *         controls and stores SD information that will be needed in future
  *         in the SD handle.
  * @param  hsd Pointer to SD handle
  * @retval error state
  */
static uint32_t SD_PowerON(SD_HandleTypeDef *hsd)
{
    __IO uint32_t count = 0U;
    uint32_t response = 0U, validvoltage = 0U;
    uint32_t errorstate = HAL_SD_ERROR_NONE;

    /* CMD0: GO_IDLE_STATE */
    errorstate = SDMMC_CmdGoIdleState(hsd->Instance);
    if (errorstate != HAL_SD_ERROR_NONE)
    {
        SDEX_DEBUG("error SDMMC_CmdGoIdleState\n");
        return errorstate;
    }

    /* CMD8: SEND_IF_COND: Command available only on V2.0 cards */
    errorstate = SDMMC_CmdOperCond(hsd->Instance);
    if (errorstate != HAL_SD_ERROR_NONE)
    {
        SDEX_DEBUG("SDMMC_CmdOperCond %d\n", errorstate);
        hsd->SdCard.CardVersion = CARD_V1_X;

        /* Send ACMD41 SD_APP_OP_COND with Argument 0x80100000 */
        while (validvoltage == 0U)
        {
            if (count++ == 0xffff)
            {
                SDEX_DEBUG("error timeout\n");
                return HAL_SD_ERROR_INVALID_VOLTRANGE;
            }

            /* SEND CMD55 APP_CMD with RCA as 0 */
            errorstate = SDMMC_CmdAppCommand(hsd->Instance, 0U);
            if (errorstate != HAL_SD_ERROR_NONE)
            {
                SDEX_DEBUG("SDMMC_CmdAppCommand %d\n", errorstate);
                return HAL_SD_ERROR_UNSUPPORTED_FEATURE;
            }

            /* Send CMD41 */
            errorstate = SDMMC_CmdAppOperCommand(hsd->Instance, SDMMC_STD_CAPACITY);
            if (errorstate != HAL_SD_ERROR_NONE)
            {
                SDEX_DEBUG("SDMMC_CmdAppOperCommand %d\n", errorstate);
                return HAL_SD_ERROR_UNSUPPORTED_FEATURE;
            }

            /* Get command response */
            //response = SDIO_GetResponse(hsd->Instance, SDIO_RESP1);
            response = hal_sdhci_get_response(hsd->Instance, 0);

            /* Get operating voltage*/
            validvoltage = (((response >> 31U) == 1U) ? 1U : 0U);
        }
        /* Card type is SDSC */
        hsd->SdCard.CardType = CARD_SDSC;
    }
    else
    {
        hsd->SdCard.CardVersion = CARD_V2_X;
        SDEX_DEBUG("CardVersion %d\n", CARD_V2_X);

        /* Send ACMD41 SD_APP_OP_COND with Argument 0x80100000 */
        while (validvoltage == 0U)
        {
            if (count++ == 0xffff)
            {
                SDEX_DEBUG("error timeout 2\n");
                return HAL_SD_ERROR_INVALID_VOLTRANGE;
            }

            /* SEND CMD55 APP_CMD with RCA as 0 */
            errorstate = SDMMC_CmdAppCommand(hsd->Instance, 0U);
            if (errorstate != HAL_SD_ERROR_NONE)
            {
                SDEX_DEBUG("SDMMC_CmdAppCommand %d\n", errorstate);
                return errorstate;
            }

            /* Send CMD41 */
            errorstate = SDMMC_CmdAppOperCommand(hsd->Instance, SDMMC_HIGH_CAPACITY);
            if (errorstate != HAL_SD_ERROR_NONE)
            {
                SDEX_DEBUG("SDMMC_CmdAppOperCommand %d\n", errorstate);
                return errorstate;
            }

            /* Get command response */
            //response = SDIO_GetResponse(hsd->Instance, SDIO_RESP1);
            response = hal_sdhci_get_response(hsd->Instance, 0);

            /* Get operating voltage*/
            validvoltage = (((response >> 31U) == 1U) ? 1U : 0U);
        }

        if ((response & SDMMC_HIGH_CAPACITY) == SDMMC_HIGH_CAPACITY) /* (response &= SD_HIGH_CAPACITY) */
        {
            hsd->SdCard.CardType = CARD_SDHC_SDXC;
        }
        else
        {
            hsd->SdCard.CardType = CARD_SDSC;
        }
        hsd->SdCard.CardType = CARD_SDSC;
        SDEX_DEBUG("card type %d\n", hsd->SdCard.CardType);
    }

    return HAL_SD_ERROR_NONE;
}

/**
  * @brief  Turns the SDIO output signals off.
  * @param  hsd Pointer to SD handle
  * @retval HAL status
  */
static HAL_StatusTypeDef SD_PowerOFF(SD_HandleTypeDef *hsd)
{
    /* Set Power State to OFF */
    //SDIO_PowerState_OFF(hsd->Instance);

    return HAL_OK;
}

/**
  * @brief  Send Status info command.
  * @param  hsd pointer to SD handle
  * @param  pSDstatus Pointer to the buffer that will contain the SD card status
  *         SD Status register)
  * @retval error state
  */
static uint32_t SD_SendSDStatus(SD_HandleTypeDef *hsd, uint32_t *pSDstatus)
{
    SDIO_DataInitTypeDef config;
    uint32_t errorstate = HAL_SD_ERROR_NONE;

    /* Check SD response */
    if ((SDIO_GetResponse(hsd->Instance, 0) & SDMMC_CARD_LOCKED) == SDMMC_CARD_LOCKED)
    {
        return HAL_SD_ERROR_LOCK_UNLOCK_FAILED;
    }

    /* Set block size for card if it is not equal to current block size for card */
    errorstate = SDMMC_CmdBlockLength(hsd->Instance, 64U);
    if (errorstate != HAL_SD_ERROR_NONE)
    {
        hsd->ErrorCode |= HAL_SD_ERROR_NONE;
        return errorstate;
    }

    /* Send CMD55 */
    errorstate = SDMMC_CmdAppCommand(hsd->Instance, (uint32_t)(hsd->SdCard.RelCardAdd << 16U));
    if (errorstate != HAL_SD_ERROR_NONE)
    {
        hsd->ErrorCode |= HAL_SD_ERROR_NONE;
        return errorstate;
    }

    /* Configure the SD DPSM (Data Path State Machine) */
    config.DataTimeOut   = SDMMC_DATATIMEOUT;
    config.DataLength    = 1U;
    config.DataBlockSize = 64;
    config.TransferDir   = 0; //SDIO_TRANSFER_DIR_TO_SDIO;
    config.TransferMode  = 0; //SDIO_TRANSFER_MODE_BLOCK;
    SDIO_ConfigData(hsd->Instance, &config);

    /* Send ACMD13 (SD_APP_STAUS)  with argument as card's RCA */
    errorstate = SDMMC_CmdStatusRegister(hsd->Instance);
    if (errorstate != HAL_SD_ERROR_NONE)
    {
        hsd->ErrorCode |= HAL_SD_ERROR_NONE;
        return errorstate;
    }

    int intmask = hal_sdhci_data_finish(hsd->Instance, 1000);
    hal_sdhci_clear_int(hsd->Instance, intmask);
    if (intmask > 0)
    {
        if (intmask & (SDHCI_INT_DATA_AVAIL | SDHCI_INT_SPACE_AVAIL))
            hal_sdhci_transfer_pio(hsd->Instance, 1, (uint8_t *)pSDstatus, (uint32_t)config.DataLength);
    }
    /* Clear all the static status flags*/
    //__HAL_SD_CLEAR_FLAG(hsd, SDIO_STATIC_FLAGS);

    return HAL_SD_ERROR_NONE;
}

/**
  * @brief  Returns the current card's status.
  * @param  hsd Pointer to SD handle
  * @param  pCardStatus pointer to the buffer that will contain the SD card
  *         status (Card Status register)
  * @retval error state
  */
static uint32_t SD_SendStatus(SD_HandleTypeDef *hsd, uint32_t *pCardStatus)
{
    uint32_t errorstate = HAL_SD_ERROR_NONE;

    if (pCardStatus == NULL)
    {
        return HAL_SD_ERROR_PARAM;
    }

    /* Send Status command */
    errorstate = SDMMC_CmdSendStatus(hsd->Instance, (uint32_t)(hsd->SdCard.RelCardAdd << 16U));
    if (errorstate != HAL_OK)
    {
        return errorstate;
    }

    /* Get SD card status */
    *pCardStatus = SDIO_GetResponse(hsd->Instance, 0);

    return HAL_SD_ERROR_NONE;
}

/**
  * @brief  Enables the SDIO wide bus mode.
  * @param  hsd pointer to SD handle
  * @retval error state
  */
static uint32_t SD_WideBus_Enable(SD_HandleTypeDef *hsd)
{
    uint32_t scr[2U] = {0U, 0U};
    uint32_t errorstate = HAL_SD_ERROR_NONE;

    if ((SDIO_GetResponse(hsd->Instance, 0) & SDMMC_CARD_LOCKED) == SDMMC_CARD_LOCKED)
    {
        return HAL_SD_ERROR_LOCK_UNLOCK_FAILED;
    }

    /* Get SCR Register */
    errorstate = SD_FindSCR(hsd, scr);
    if (errorstate != HAL_OK)
    {
        SDEX_DEBUG("SD_FindSCR fail 0x%x\n", errorstate);
        return errorstate;
    }

    /* If requested card supports wide bus operation */
    if ((scr[1U] & SDMMC_WIDE_BUS_SUPPORT) != SDMMC_ALLZERO)
    {
        /* Send CMD55 APP_CMD with argument as card's RCA.*/
        errorstate = SDMMC_CmdAppCommand(hsd->Instance, (uint32_t)(hsd->SdCard.RelCardAdd << 16U));
        if (errorstate != HAL_OK)
        {
            SDEX_DEBUG("SDMMC_CmdAppCommand fail 0x%x\n", errorstate);
            return errorstate;
        }

        /* Send ACMD6 APP_CMD with argument as 2 for wide bus mode */
        errorstate = SDMMC_CmdBusWidth(hsd->Instance, 2U);
        if (errorstate != HAL_OK)
        {
            SDEX_DEBUG("SDMMC_CmdBusWidth fail 0x%x\n", errorstate);
            return errorstate;
        }

        return HAL_SD_ERROR_NONE;
    }
    else
    {
        return HAL_SD_ERROR_REQUEST_NOT_APPLICABLE;
    }
}

/**
  * @brief  Disables the SDIO wide bus mode.
  * @param  hsd Pointer to SD handle
  * @retval error state
  */
static uint32_t SD_WideBus_Disable(SD_HandleTypeDef *hsd)
{
    uint32_t scr[2U] = {0U, 0U};
    uint32_t errorstate = HAL_SD_ERROR_NONE;

    if ((SDIO_GetResponse(hsd->Instance, 0) & SDMMC_CARD_LOCKED) == SDMMC_CARD_LOCKED)
    {
        return HAL_SD_ERROR_LOCK_UNLOCK_FAILED;
    }

    /* Get SCR Register */
    errorstate = SD_FindSCR(hsd, scr);
    if (errorstate != HAL_OK)
    {
        return errorstate;
    }

    /* If requested card supports 1 bit mode operation */
    if ((scr[1U] & SDMMC_SINGLE_BUS_SUPPORT) != SDMMC_ALLZERO)
    {
        /* Send CMD55 APP_CMD with argument as card's RCA */
        errorstate = SDMMC_CmdAppCommand(hsd->Instance, (uint32_t)(hsd->SdCard.RelCardAdd << 16U));
        if (errorstate != HAL_OK)
        {
            return errorstate;
        }

        /* Send ACMD6 APP_CMD with argument as 0 for single bus mode */
        errorstate = SDMMC_CmdBusWidth(hsd->Instance, 0U);
        if (errorstate != HAL_OK)
        {
            return errorstate;
        }

        return HAL_SD_ERROR_NONE;
    }
    else
    {
        return HAL_SD_ERROR_REQUEST_NOT_APPLICABLE;
    }
}


/**
  * @brief  Finds the SD card SCR register value.
  * @param  hsd Pointer to SD handle
  * @param  pSCR pointer to the buffer that will contain the SCR value
  * @retval error state
  */
static uint32_t SD_FindSCR(SD_HandleTypeDef *hsd, uint32_t *pSCR)
{
    SDIO_DataInitTypeDef config;
    uint32_t errorstate = HAL_SD_ERROR_NONE;
    uint32_t tempscr[2U] = {0U, 0U};


    hal_sdhci_set_transfer_irqs(hsd->Instance);

    hal_sdhci_set_blk(hsd->Instance, 8, 1);

    /* Set Block Size To 8 Bytes */
    errorstate = SDMMC_CmdBlockLength(hsd->Instance, 8U);
    if (0) //if (errorstate != HAL_OK)
    {
        return errorstate;
    }

    /* Send CMD55 APP_CMD with argument as card's RCA */
    errorstate = SDMMC_CmdAppCommand(hsd->Instance, (uint32_t)((hsd->SdCard.RelCardAdd) << 16U));
    if (errorstate != HAL_OK)
    {
        return errorstate;
    }
#if 1
    config.DataTimeOut   = SDMMC_DATATIMEOUT;
    config.DataLength    = 1U;
    config.DataBlockSize = 8U;
    config.TransferDir   = 0;
    config.TransferMode  = 0;
    SDIO_ConfigData(hsd->Instance, &config);
#endif
    /* Send ACMD51 SD_APP_SEND_SCR with argument as 0 */
    errorstate = SDMMC_CmdSendSCR(hsd->Instance);
    if (errorstate != HAL_OK)
    {
        return errorstate;
    }

    int res = hal_sdhci_data_finish(hsd->Instance, 1000);
#if 0
    uint32_t index = 0U;
    uint32_t tickstart = HAL_GetTick();

    while (!__HAL_SD_GET_FLAG(hsd, SDIO_FLAG_RXOVERR | SDIO_FLAG_DCRCFAIL | SDIO_FLAG_DTIMEOUT | SDIO_FLAG_DBCKEND))
    {
        if (__HAL_SD_GET_FLAG(hsd, SDIO_FLAG_RXDAVL))
        {
            *(tempscr + index) = SDIO_ReadFIFO(hsd->Instance);
            index++;
        }

        if ((HAL_GetTick() - tickstart) >=  SDMMC_DATATIMEOUT)
        {
            return HAL_SD_ERROR_TIMEOUT;
        }
    }
#endif
    if (res < 0)
    {
        hal_sdhci_clear_int(hsd->Instance, SDHCI_INT_ERROR | SDHCI_INT_DATA_MASK);

        return HAL_SD_ERROR_DATA_TIMEOUT;
    }
    else
    {
        /* No error flag set */
        /* Clear all the static flags */
        //__HAL_SD_CLEAR_FLAG(hsd, SDIO_STATIC_FLAGS);
        hal_sdhci_transfer_pio(hsd->Instance, 1, (uint8_t *)tempscr, 8);

        *(pSCR + 1U) = ((tempscr[0U] & SDMMC_0TO7BITS) << 24U)  | ((tempscr[0U] & SDMMC_8TO15BITS) << 8U) | \
                       ((tempscr[0U] & SDMMC_16TO23BITS) >> 8U) | ((tempscr[0U] & SDMMC_24TO31BITS) >> 24U);

        *(pSCR) = ((tempscr[1U] & SDMMC_0TO7BITS) << 24U)  | ((tempscr[1U] & SDMMC_8TO15BITS) << 8U) | \
                  ((tempscr[1U] & SDMMC_16TO23BITS) >> 8U) | ((tempscr[1U] & SDMMC_24TO31BITS) >> 24U);
    }
    SDEX_DEBUG("pSCR: 0X%08x, 0x%08x\n", *pSCR, *(pSCR + 1U));

    return HAL_SD_ERROR_NONE;
}

#if 0
/**
  * @brief  Wrap up reading in non-blocking mode.
  * @param  hsd pointer to a SD_HandleTypeDef structure that contains
  *              the configuration information.
  * @retval HAL status
  */
static HAL_StatusTypeDef SD_Read_IT(SD_HandleTypeDef *hsd)
{
    uint8_t *tmp;

    tmp = (uint8_t *)hsd->pRxBuffPtr;

    /* Read data from SDIO Rx FIFO */
    //uint32_t count = 0U;
    //for (count = 0U; count < 8U; count++)
    //{
    //    *(tmp + count) = SDIO_ReadFIFO(hsd->Instance);
    //}
    hsd->Instance->Init.blocks = 1;
    hal_sdhci_transfer_pio(hsd->Instance, 1, tmp, 8);

    hsd->pRxBuffPtr += 8U;

    return HAL_OK;
}

/**
  * @brief  Wrap up writing in non-blocking mode.
  * @param  hsd pointer to a SD_HandleTypeDef structure that contains
  *              the configuration information.
  * @retval HAL status
  */
static HAL_StatusTypeDef SD_Write_IT(SD_HandleTypeDef *hsd)
{
    uint8_t *tmp;

    tmp = (uint8_t *)hsd->pTxBuffPtr;

    /* Write data to SDIO Tx FIFO */
    //uint32_t count;
    //for (count = 0U; count < 8U; count++)
    //{
    //    SDIO_WriteFIFO(hsd->Instance, (tmp + count));
    //}
    hsd->Instance->Init.blocks = 1;
    hal_sdhci_transfer_pio(hsd->Instance, 0, tmp, 8);

    hsd->pTxBuffPtr += 8U;

    return HAL_OK;
}
#endif

/**
  * @}
  */
#endif

#endif /* HAL_SDHCI_MODULE_ENABLED */
