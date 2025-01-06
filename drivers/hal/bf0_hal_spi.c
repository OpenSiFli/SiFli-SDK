/**
  ******************************************************************************
  * @file   bf0_hal_spi.c
  * @author Sifli software development team
  * @brief   SPI HAL module driver.
  *          This file provides firmware functions to manage the following
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

/* define ssp version for special usage */
#define HAL_SSP_VER

//extern  void rt_kprintf(const char *fmt, ...);
#define SPI_PRINTF HAL_DBG_printf

/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @defgroup SPI SPI
  * @brief SPI HAL module driver
  * @{
  */
#if defined(HAL_SPI_MODULE_ENABLED) ||defined(_SIFLI_DOXYGEN_)
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/** @defgroup SPI_Private_Constants SPI Private Constants
  * @{
  */
#define SPI_DEFAULT_TIMEOUT 100U
/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/** @defgroup SPI_Private_Functions SPI Private Functions
  * @{
  */
static void SPI_DMATransmitCplt(DMA_HandleTypeDef *hdma);
static void SPI_DMAReceiveCplt(DMA_HandleTypeDef *hdma);
static void SPI_DMATransmitReceiveCplt(DMA_HandleTypeDef *hdma);
static void SPI_DMAHalfTransmitCplt(DMA_HandleTypeDef *hdma);
static void SPI_DMAHalfReceiveCplt(DMA_HandleTypeDef *hdma);
static void SPI_DMAHalfTransmitReceiveCplt(DMA_HandleTypeDef *hdma);
static void SPI_DMAError(DMA_HandleTypeDef *hdma);
static void SPI_DMAAbortOnError(DMA_HandleTypeDef *hdma);
#if 0
    static void SPI_DMATxAbortCallback(DMA_HandleTypeDef *hdma);
    static void SPI_DMARxAbortCallback(DMA_HandleTypeDef *hdma);
#endif
static HAL_StatusTypeDef SPI_WaitFlagStateUntilTimeout(SPI_HandleTypeDef *hspi, uint32_t Flag, uint32_t State,
        uint32_t Timeout, uint32_t Tickstart);
static HAL_StatusTypeDef SPI_WaitFifoStateUntilTimeout(SPI_HandleTypeDef *hspi, uint32_t Fifo, uint32_t State,
        uint32_t Timeout, uint32_t Tickstart);
static void SPI_TxISR_8BIT(struct __SPI_HandleTypeDef *hspi);
static void SPI_TxISR_16BIT(struct __SPI_HandleTypeDef *hspi);
static void SPI_RxISR_8BIT(struct __SPI_HandleTypeDef *hspi);
static void SPI_RxISR_16BIT(struct __SPI_HandleTypeDef *hspi);
static void SPI_2linesRxISR_8BIT(struct __SPI_HandleTypeDef *hspi);
static void SPI_2linesTxISR_8BIT(struct __SPI_HandleTypeDef *hspi);
static void SPI_2linesTxISR_16BIT(struct __SPI_HandleTypeDef *hspi);
static void SPI_2linesRxISR_16BIT(struct __SPI_HandleTypeDef *hspi);
#if (USE_SPI_CRC != 0U)
    static void SPI_RxISR_8BITCRC(struct __SPI_HandleTypeDef *hspi);
    static void SPI_RxISR_16BITCRC(struct __SPI_HandleTypeDef *hspi);
    static void SPI_2linesRxISR_8BITCRC(struct __SPI_HandleTypeDef *hspi);
    static void SPI_2linesRxISR_16BITCRC(struct __SPI_HandleTypeDef *hspi);
#endif /* USE_SPI_CRC */
static void SPI_AbortRx_ISR(SPI_HandleTypeDef *hspi);
static void SPI_AbortTx_ISR(SPI_HandleTypeDef *hspi);
static void SPI_CloseRxTx_ISR(SPI_HandleTypeDef *hspi);
static void SPI_CloseRx_ISR(SPI_HandleTypeDef *hspi);
static void SPI_CloseTx_ISR(SPI_HandleTypeDef *hspi);
static HAL_StatusTypeDef SPI_EndRxTransaction(SPI_HandleTypeDef *hspi, uint32_t Timeout, uint32_t Tickstart);
static HAL_StatusTypeDef SPI_EndRxTxTransaction(SPI_HandleTypeDef *hspi, uint32_t Timeout, uint32_t Tickstart);
static uint8_t SPI_GetIdx(SPI_HandleTypeDef *hspi);

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup SPI_Exported_Functions SPI Exported Functions
  * @{
  */

/** @defgroup SPI_Exported_Functions_Group1 Initialization and de-initialization functions
 *  @brief    Initialization and Configuration functions
 *
@verbatim
 ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]  This subsection provides a set of functions allowing to initialize and
          de-initialize the SPIx peripheral:

      (+) User must implement HAL_SPI_MspInit() function in which he configures
          all related peripherals resources (CLOCK, GPIO, DMA, IT and NVIC ).

      (+) Call the function HAL_SPI_Init() to configure the selected device with
          the selected configuration:
        (++) Mode
        (++) Direction
        (++) Data Size
        (++) Clock Polarity and Phase
        (++) BaudRate Prescaler
        (++) FrameFormat
        (++) CRC Calculation
        (++) CRC Polynomial if CRC enabled
        (++) CRC Length, used only with Data8 and Data16
        (++) FIFO reception threshold

      (+) Call the function HAL_SPI_DeInit() to restore the default configuration
          of the selected SPIx peripheral.

@endverbatim
  * @{
  */

/**
  * @brief  Initialize the SPI according to the specified parameters
  *         in the SPI_InitTypeDef and initialize the associated handle.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *hspi)
{
    uint32_t value;

    /* Check the SPI handle allocation */
    if (hspi == NULL)
    {
        return HAL_ERROR;
    }

    /* Check the parameters */
    HAL_ASSERT(IS_SPI_ALL_INSTANCE(hspi->Instance));
    HAL_ASSERT(IS_SPI_MODE(hspi->Init.Mode));
    HAL_ASSERT(IS_SPI_DIRECTION(hspi->Init.Direction));
    HAL_ASSERT(IS_SPI_DATASIZE(hspi->Init.DataSize));
    HAL_ASSERT(IS_SPI_BAUDRATE_PRESCALER(hspi->Init.BaudRatePrescaler));
    HAL_ASSERT(IS_SPI_FRAME_FORMAT(hspi->Init.FrameFormat));
    HAL_ASSERT(IS_SPI_SFRMPOL(hspi->Init.SFRMPol));

    if (hspi->Init.FrameFormat != SPI_FRAME_FORMAT_SSP)
    {
        HAL_ASSERT(IS_SPI_CPOL(hspi->Init.CLKPolarity));
        HAL_ASSERT(IS_SPI_CPHA(hspi->Init.CLKPhase));
    }
#if (USE_SPI_CRC != 0U)
    HAL_ASSERT(IS_SPI_CRC_CALCULATION(hspi->Init.CRCCalculation));
    if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
    {
        HAL_ASSERT(IS_SPI_CRC_POLYNOMIAL(hspi->Init.CRCPolynomial));
        HAL_ASSERT(IS_SPI_CRC_LENGTH(hspi->Init.CRCLength));
    }
#else
    //hspi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
#endif /* USE_SPI_CRC */

    if (hspi->State == HAL_SPI_STATE_RESET)
    {
        /* Allocate lock resource and initialize it */
        hspi->Lock = HAL_UNLOCKED;

        /* Init the low level hardware : GPIO, CLOCK, NVIC... */
        HAL_SPI_MspInit(hspi);
    }

    hspi->State = HAL_SPI_STATE_BUSY;

    /* Disable the selected SPI peripheral */
    __HAL_SPI_DISABLE(hspi);
    /* Disable TXE, RXNE and ERR interrupts for the interrupt process */
    __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));

#if (USE_SPI_CRC != 0U)
    /* CRC calculation is valid only for 16Bit and 8 Bit */
    if ((hspi->Init.DataSize != SPI_DATASIZE_16BIT) && (hspi->Init.DataSize != SPI_DATASIZE_8BIT))
    {
        /* CRC must be disabled */
        hspi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    }

    /* Align the CRC Length on the data size */
    if (hspi->Init.CRCLength == SPI_CRC_LENGTH_DATASIZE)
    {
        /* CRC Length aligned on the data size : value set by default */
        if (hspi->Init.DataSize > SPI_DATASIZE_8BIT)
        {
            hspi->Init.CRCLength = SPI_CRC_LENGTH_16BIT;
        }
        else
        {
            hspi->Init.CRCLength = SPI_CRC_LENGTH_8BIT;
        }
    }
#endif

    if (1 == hspi->Init.BaudRatePrescaler)
    {
        WRITE_REG(hspi->Instance->CLK_CTRL, SPI_CLK_CTRL_CLK_SEL);
    }
    else
    {
        WRITE_REG(hspi->Instance->CLK_CTRL, hspi->Init.BaudRatePrescaler);
    }

#ifdef SPI_CLK_CTRL_CLK_SSP_EN
    SET_BIT(hspi->Instance->CLK_CTRL, SPI_CLK_CTRL_CLK_SSP_EN);
#endif /* SPI_CLK_CTRL_CLK_SSP_EN */

    if (SPI_MODE_MASTER == hspi->Init.Mode)
    {
        value = 0;
    }
    else
    {
        value = SPI_TOP_CTRL_SFRMDIR | SPI_TOP_CTRL_SCLKDIR;
    }

    value |= (hspi->Init.DataSize | hspi->Init.FrameFormat |
              hspi->Init.CLKPolarity | hspi->Init.CLKPhase);

    WRITE_REG(hspi->Instance->TOP_CTRL, value);

    /* Configure : SPI_SFRM_POL*/
    //WRITE_REG(hspi->Instance->SFRM_POL, (hspi->Init.SFRMPol));

    if (hspi->Init.Direction == SPI_DIRECTION_1LINE)
    {
        hspi->Instance->TRIWIRE_CTRL |= SPI_TRIWIRE_CTRL_SPI_TRI_WIRE_EN;
        hspi->Instance->TOP_CTRL |= SPI_TOP_CTRL_TTE;
#ifdef SPI_CLK_CTRL_SPI_DI_SEL
        hspi->Instance->CLK_CTRL |= SPI_CLK_CTRL_SPI_DI_SEL;
#endif /* SPI_CLK_CTRL_SPI_DI_SEL */
    }
    else
    {
        hspi->Instance->TRIWIRE_CTRL &= ~SPI_TRIWIRE_CTRL_SPI_TRI_WIRE_EN;
        hspi->Instance->TOP_CTRL &= ~SPI_TOP_CTRL_TTE;
#ifdef SPI_CLK_CTRL_SPI_DI_SEL
        hspi->Instance->CLK_CTRL &= ~SPI_CLK_CTRL_SPI_DI_SEL;
#endif /* SPI_CLK_CTRL_SPI_DI_SEL */

    }

    /* Issue 1401: Make SPO setting is valid before start transfer data*/
    __HAL_SPI_ENABLE(hspi);
    __HAL_SPI_DISABLE(hspi);

    /*
       1. FIFO can't be 0 in slave mode, or TX FIFO maybe underrun.
       2. FIFO can't be large than 6 ( 6 means FIFO threshold 7, unisoc modem need send 8 enties once at least in slave mode),
          as DMA will call 'TransmitReceiveCplt' after fill all data into FIFO althrough there is NO spi clock.
    */
    if (SPI_MODE_SLAVE == hspi->Init.Mode)
    {
        hspi->Instance->FIFO_CTRL |= 5 << SPI_FIFO_CTRL_TFT_Pos;
    }


    hspi->ErrorCode = HAL_SPI_ERROR_NONE;
    hspi->State     = HAL_SPI_STATE_READY;

    return HAL_OK;
}

/**
  * @brief  De-Initialize the SPI peripheral.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_SPI_DeInit(SPI_HandleTypeDef *hspi)
{
    /* Check the SPI handle allocation */
    if (hspi == NULL)
    {
        return HAL_ERROR;
    }

    /* Check SPI Instance parameter */
    HAL_ASSERT(IS_SPI_ALL_INSTANCE(hspi->Instance));

    hspi->State = HAL_SPI_STATE_BUSY;

    /* Disable the SPI Peripheral Clock */
    __HAL_SPI_DISABLE(hspi);

    /* DeInit the low level hardware: GPIO, CLOCK, NVIC... */
    HAL_SPI_MspDeInit(hspi);

    hspi->ErrorCode = HAL_SPI_ERROR_NONE;
    hspi->State = HAL_SPI_STATE_RESET;

    /* Release Lock */
    __HAL_UNLOCK(hspi);

    return HAL_OK;
}

/**
  * @brief  Initialize the SPI MSP.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
__weak void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hspi);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_SPI_MspInit should be implemented in the user file
     */
}

/**
  * @brief  De-Initialize the SPI MSP.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
__weak void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hspi);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_SPI_MspDeInit should be implemented in the user file
     */
}

/**
  * @}
  */

/** @defgroup SPI_Exported_Functions_Group2 IO operation functions
 *  @brief   Data transfers functions
 *
@verbatim
  ==============================================================================
                      ##### IO operation functions #####
 ===============================================================================
 [..]
    This subsection provides a set of functions allowing to manage the SPI
    data transfers.

    [..] The SPI supports master and slave mode :

    (#) There are two modes of transfer:
       (++) Blocking mode: The communication is performed in polling mode.
            The HAL status of all data processing is returned by the same function
            after finishing transfer.
       (++) No-Blocking mode: The communication is performed using Interrupts
            or DMA, These APIs return the HAL status.
            The end of the data processing will be indicated through the
            dedicated SPI IRQ when using Interrupt mode or the DMA IRQ when
            using DMA mode.
            The HAL_SPI_TxCpltCallback(), HAL_SPI_RxCpltCallback() and HAL_SPI_TxRxCpltCallback() user callbacks
            will be executed respectively at the end of the transmit or Receive process
            The HAL_SPI_ErrorCallback()user callback will be executed when a communication error is detected

    (#) APIs provided for these 2 transfer modes (Blocking mode or Non blocking mode using either Interrupt or DMA)
        exist for 1Line (simplex) and 2Lines (full duplex) modes.

@endverbatim
  * @{
  */

/**
  * @brief  Transmit an amount of data in blocking mode.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  pData pointer to data buffer
  * @param  Size amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    uint32_t tickstart = 0U;
    HAL_StatusTypeDef errorcode = HAL_OK;
    SPI_PRINTF("HAL_SPI_Transmit %x,data=%x,size=%d \r\n", hspi, pData, Size);

    if ((hspi->Init.DataSize > SPI_DATASIZE_8BIT)/* || (Size > 1U)*/)
    {
        /* in this case, 16-bit access is performed on Data
           So, check Data is 16-bit aligned address */
        HAL_ASSERT(IS_SPI_16BIT_ALIGNED_ADDRESS(pData));
    }

    //MODIFY_REG(hspi->Instance->CR0, SPI_CR0_FRF, SPI_CR0_FRF_MOTO);
    //MODIFY_REG(hspi->Instance->CR0, SPI_CR0_DSS_Msk, hspi->Init.DataSize);

    /* Check Direction parameter */
    HAL_ASSERT(IS_SPI_DIRECTION_2LINES_OR_1LINE(hspi->Init.Direction));



    /* Process Locked */
    __HAL_LOCK(hspi);

    /* Init tickstart for timeout management*/
    tickstart = HAL_GetTick();

    if (hspi->State != HAL_SPI_STATE_READY)
    {
        errorcode = HAL_BUSY;
        goto error;
    }

    if ((pData == NULL) || (Size == 0U))
    {
        errorcode = HAL_ERROR;
        goto error;
    }

    /* Set the transaction information */
    hspi->State       = HAL_SPI_STATE_BUSY_TX;
    hspi->ErrorCode   = HAL_SPI_ERROR_NONE;
    hspi->pTxBuffPtr  = (uint8_t *)pData;
    hspi->TxXferSize  = Size;
    hspi->TxXferCount = Size;

    /*Init field not used in handle to zero */
    hspi->pRxBuffPtr  = (uint8_t *)NULL;
    hspi->RxXferSize  = 0U;
    hspi->RxXferCount = 0U;
    hspi->TxISR       = NULL;
    hspi->RxISR       = NULL;

    if (hspi->Init.Direction == SPI_DIRECTION_1LINE)
    {
        /*1line tx enable*/
        SPI_1LINE_TX(hspi);
    }

    /* Check if the SPI is already enabled */
    if ((hspi->Instance->TOP_CTRL & SPI_TOP_CTRL_SSE) != SPI_TOP_CTRL_SSE)
    {
        /* Enable SPI peripheral */
        __HAL_SPI_ENABLE(hspi);
    }

    /* Transmit data in 16 Bit mode */
    if (hspi->Init.DataSize > SPI_DATASIZE_8BIT)
    {
        if ((hspi->Init.Mode == SPI_MODE_SLAVE) || (hspi->TxXferCount == 0x01U))
        {
            hspi->Instance->DATA = *((uint16_t *)pData);
            pData += sizeof(uint16_t);
            hspi->TxXferCount--;
        }
        /* Transmit data in 16 Bit mode */
        while (hspi->TxXferCount > 0U)
        {
            /* Wait until TXE flag is set to send data */
            if (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_TXE))
            {
                hspi->Instance->DATA = *((uint16_t *)pData);
                pData += sizeof(uint16_t);
                hspi->TxXferCount--;
            }
            else
            {
                /* Timeout management */
                if ((Timeout == 0U) || ((Timeout != HAL_MAX_DELAY) && ((HAL_GetTick() - tickstart) >=  Timeout)))
                {
                    errorcode = HAL_TIMEOUT;
                    goto error;
                }
            }
        }
    }
    /* Transmit data in 8 Bit mode */
    else
    {
        if ((hspi->Init.Mode == SPI_MODE_SLAVE) || (hspi->TxXferCount == 0x01U))
        {
            *((__IO uint8_t *)&hspi->Instance->DATA) = (*pData++);
            hspi->TxXferCount--;
        }
        while (hspi->TxXferCount > 0U)
        {
            /* Wait until TXE flag is set to send data */
            if (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_TXE))
            {
                *((__IO uint8_t *)&hspi->Instance->DATA) = (*pData++);
                hspi->TxXferCount--;
            }
            else
            {
                /* Timeout management */
                if ((Timeout == 0U) || ((Timeout != HAL_MAX_DELAY) && ((HAL_GetTick() - tickstart) >=  Timeout)))
                {
                    errorcode = HAL_TIMEOUT;
                    goto error;
                }
            }
        }
    }
#if (USE_SPI_CRC != 0U)
    /* Enable CRC Transmission */
    if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
    {
        SET_BIT(hspi->Instance->CR1, SPI_CR1_CRCNEXT);
    }
#endif /* USE_SPI_CRC */

    /* Check the end of the transaction */
    if (SPI_EndRxTxTransaction(hspi, Timeout, tickstart) != HAL_OK)
    {
        hspi->ErrorCode = HAL_SPI_ERROR_FLAG;
    }

    if (hspi->ErrorCode != HAL_SPI_ERROR_NONE)
    {
        errorcode = HAL_ERROR;
    }

error:
    hspi->State = HAL_SPI_STATE_READY;
    /* Process Unlocked */
    __HAL_UNLOCK(hspi);
    return errorcode;
}


#define SPI_FIFO_SIZE       16
/**
  * @brief Flush the RX fifo.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for the specified SPI module.
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_SPI_FlushRxFifo(SPI_HandleTypeDef *hspi)
{
    __IO uint32_t tmpreg;
    uint8_t  count = 0U;
    while ((hspi->Instance->STATUS & SPI_FLAG_FRLVL) !=  SPI_FRLVL_EMPTY)
    {
        count++;
        tmpreg = hspi->Instance->DATA;
        UNUSED(tmpreg); /* To avoid GCC warning */
        if (count == SPI_FIFO_SIZE)
        {
            return HAL_TIMEOUT;
        }
    }
    return HAL_OK;
}

/**
  * @brief  Receive an amount of data in blocking mode.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  pData pointer to data buffer
  * @param  Size amount of data to be received
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
#if (USE_SPI_CRC != 0U)
    __IO uint16_t tmpreg = 0U;
#endif /* USE_SPI_CRC */
    uint32_t tickstart = 0U;
    HAL_StatusTypeDef errorcode = HAL_OK;
    SPI_PRINTF("HAL_SPI_Receive %x,data=%x,size=%d \r\n", hspi, pData, Size);

    if ((hspi->Init.DataSize > SPI_DATASIZE_8BIT)/* || (Size > 1U)*/)
    {
        /* in this case, 16-bit access is performed on Data
           So, check Data is 16-bit aligned address */
        HAL_ASSERT(IS_SPI_16BIT_ALIGNED_ADDRESS(pData));
    }

    if ((hspi->Init.Mode == SPI_MODE_MASTER) && (hspi->Init.Direction == SPI_DIRECTION_2LINES))
    {
        hspi->State = HAL_SPI_STATE_BUSY_RX;
        /* Call transmit-receive function to send Dummy data on Tx line and generate clock on CLK line */
        return HAL_SPI_TransmitReceive(hspi, pData, pData, Size, Timeout);
    }


    /* Process Locked */
    __HAL_LOCK(hspi);

    /* Init tickstart for timeout management*/
    tickstart = HAL_GetTick();

    if (hspi->State != HAL_SPI_STATE_READY)
    {
        errorcode = HAL_BUSY;
        goto error;
    }

    if ((pData == NULL) || (Size == 0U))
    {
        errorcode = HAL_ERROR;
        goto error;
    }

    /* Set the transaction information */
    hspi->State       = HAL_SPI_STATE_BUSY_RX;
    hspi->ErrorCode   = HAL_SPI_ERROR_NONE;
    hspi->pRxBuffPtr  = (uint8_t *)pData;
    hspi->RxXferSize  = Size;
    hspi->RxXferCount = Size;

    /*Init field not used in handle to zero */
    hspi->pTxBuffPtr  = (uint8_t *)NULL;
    hspi->TxXferSize  = 0U;
    hspi->TxXferCount = 0U;
    hspi->RxISR       = NULL;
    hspi->TxISR       = NULL;

#if (USE_SPI_CRC != 0U)
    /* Reset CRC Calculation */
    if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
    {
        SPI_RESET_CRC(hspi);
        /* this is done to handle the CRCNEXT before the latest data */
        hspi->RxXferCount--;
    }
#endif /* USE_SPI_CRC */

    if (hspi->Init.Direction == SPI_DIRECTION_1LINE)
    {
        /*1line rx enable*/
        SPI_1LINE_RX(hspi);
    }

    /* Check if the SPI is already enabled */
    if ((hspi->Instance->TOP_CTRL & SPI_TOP_CTRL_SSE) != SPI_TOP_CTRL_SSE)
    {
        /* Enable SPI peripheral */
        __HAL_SPI_ENABLE(hspi);
    }

    /* Receive data in 8 Bit mode */
    if (hspi->Init.DataSize <= SPI_DATASIZE_8BIT)
    {
        /* Transfer loop */
        while (hspi->RxXferCount > 0U)
        {
            /* Check the RXNE flag */
            if (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE))
            {
                /* read the received data */
                (* (uint8_t *)pData) = *(__IO uint8_t *)&hspi->Instance->DATA;
                pData += sizeof(uint8_t);
                hspi->RxXferCount--;
            }
            else
            {
                /* Timeout management */
                if ((Timeout == 0U) || ((Timeout != HAL_MAX_DELAY) && ((HAL_GetTick() - tickstart) >=  Timeout)))
                {
                    errorcode = HAL_TIMEOUT;
                    goto error;
                }
            }
        }
    }
    else
    {
        /* Transfer loop */
        while (hspi->RxXferCount > 0U)
        {
            /* Check the RXNE flag */
            if (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE))
            {
                *((uint16_t *)pData) = hspi->Instance->DATA;
                pData += sizeof(uint16_t);
                hspi->RxXferCount--;
            }
            else
            {
                /* Timeout management */
                if ((Timeout == 0U) || ((Timeout != HAL_MAX_DELAY) && ((HAL_GetTick() - tickstart) >=  Timeout)))
                {
                    errorcode = HAL_TIMEOUT;
                    goto error;
                }
            }
        }
    }

    /* Check the end of the transaction */
    if (SPI_EndRxTransaction(hspi, Timeout, tickstart) != HAL_OK)
    {
        hspi->ErrorCode = HAL_SPI_ERROR_FLAG;
    }

#if (USE_SPI_CRC != 0U)
    /* Check if CRC error occurred */
    if (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_CRCERR))
    {
        SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_CRC);
        __HAL_SPI_CLEAR_CRCERRFLAG(hspi);
    }
#endif /* USE_SPI_CRC */

    if (hspi->ErrorCode != HAL_SPI_ERROR_NONE)
    {
        errorcode = HAL_ERROR;
    }

error :
    hspi->State = HAL_SPI_STATE_READY;
    __HAL_UNLOCK(hspi);
    return errorcode;
}

/**
  * @brief  Transmit and Receive an amount of data in blocking mode.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  pTxData pointer to transmission data buffer
  * @param  pRxData pointer to reception data buffer
  * @param  Size amount of data to be sent and received
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size,
        uint32_t Timeout)
{
    uint32_t tmp = 0U, tmp1 = 0U;
#if (USE_SPI_CRC != 0U)
    __IO uint16_t tmpreg = 0U;
#endif /* USE_SPI_CRC */
    uint32_t tickstart = 0U;
    /* Variable used to prevent RX fifo overrun */
    uint32_t rx_fifo_left = SPI_FIFO_SIZE;
    HAL_StatusTypeDef errorcode = HAL_OK;

    SPI_PRINTF("HAL_SPI_TransmitReceive %x,Txdata=%x, Rxdata=%x, Size=%d \r\n", hspi, pTxData, pRxData, Size);

#ifndef HAL_SSP_VER
    //MODIFY_REG(hspi->Instance->CR0, SPI_CR0_FRF, SPI_CR0_FRF_NM);
    //MODIFY_REG(hspi->Instance->CR0, SPI_CR0_DSS_Msk, MAKE_REG_VAL(SPI_CR0_DSS_16BIT, SPI_CR0_DSS_Msk, SPI_CR0_DSS_Pos));

    if ((hspi->Init.DataSize > SPI_DATASIZE_8BIT)/* || (Size > 1U)*/)
    {
        /* in this case, 16-bit access is performed on Data
           So, check Data is 16-bit aligned address */
        HAL_ASSERT(IS_SPI_16BIT_ALIGNED_ADDRESS(pTxData));
        HAL_ASSERT(IS_SPI_16BIT_ALIGNED_ADDRESS(pRxData));
    }
#endif

    /* Check Direction parameter */
    HAL_ASSERT(IS_SPI_DIRECTION_2LINES(hspi->Init.Direction));

    /* Process Locked */
    __HAL_LOCK(hspi);

    /* Init tickstart for timeout management*/
    tickstart = HAL_GetTick();

    tmp  = hspi->State;
    tmp1 = hspi->Init.Mode;

    if (!((tmp == HAL_SPI_STATE_READY) || \
            ((tmp1 == SPI_MODE_MASTER) && (hspi->Init.Direction == SPI_DIRECTION_2LINES) && (tmp == HAL_SPI_STATE_BUSY_RX))))
    {
        errorcode = HAL_BUSY;
        goto error;
    }

    if ((pTxData == NULL) || (pRxData == NULL) || (Size == 0U))
    {
        errorcode = HAL_ERROR;
        goto error;
    }

    /* Don't overwrite in case of HAL_SPI_STATE_BUSY_RX */
    if (hspi->State != HAL_SPI_STATE_BUSY_RX)
    {
        hspi->State = HAL_SPI_STATE_BUSY_TX_RX;
    }
#ifdef HAL_SSP_VER
    /* Set the transaction information */
    hspi->ErrorCode   = HAL_SPI_ERROR_NONE;
    hspi->pRxBuffPtr  = (uint8_t *)pRxData;
    hspi->RxXferCount = Size;
    hspi->RxXferSize  = Size;
    hspi->pTxBuffPtr  = (uint8_t *)pTxData;
    hspi->TxXferCount = Size;
    hspi->TxXferSize  = Size;

#else
    /* Set the transaction information */
    hspi->ErrorCode   = HAL_SPI_ERROR_NONE;
    hspi->pRxBuffPtr  = (uint8_t *)pRxData;
    hspi->RxXferCount = Size;
    hspi->RxXferSize  = Size;
    hspi->pTxBuffPtr  = (uint8_t *)pTxData;
    hspi->TxXferCount = 1;
    hspi->TxXferSize  = 1;
#endif
    /*Init field not used in handle to zero */
    hspi->RxISR       = NULL;
    hspi->TxISR       = NULL;

    /* Check if the SPI is already enabled */
    if ((hspi->Instance->TOP_CTRL & SPI_TOP_CTRL_SSE) != SPI_TOP_CTRL_SSE)
    {
        /* Enable SPI peripheral */
        __HAL_SPI_ENABLE(hspi);
    }

    /* Transmit and Receive data in 16 Bit mode */
    if (hspi->Init.DataSize > SPI_DATASIZE_8BIT)
    {

        if ((hspi->Init.Mode == SPI_MODE_SLAVE) || (hspi->TxXferCount == 0x01U))
        {
            hspi->Instance->DATA = *((uint16_t *)pTxData);
            pTxData += sizeof(uint16_t);
            hspi->TxXferCount--;
        }
        while ((hspi->TxXferCount > 0U) || (hspi->RxXferCount > 0U))
        {
            /* Check TXE flag */
            if ((rx_fifo_left > 0) && (hspi->TxXferCount > 0U) && (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_TXE)))
            {
                hspi->Instance->DATA = *((uint16_t *)pTxData);
                pTxData += sizeof(uint16_t);
                hspi->TxXferCount--;
                rx_fifo_left--;

#if (USE_SPI_CRC != 0U)
                /* Enable CRC Transmission */
                if ((hspi->TxXferCount == 0U) && (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE))
                {
                    /* Set NSS Soft to received correctly the CRC on slave mode with NSS pulse activated */
                    if (((hspi->Instance->CR1 & SPI_CR1_MSTR) == 0U) && ((hspi->Instance->CR2 & SPI_CR2_NSSP) == SPI_CR2_NSSP))
                    {
                        SET_BIT(hspi->Instance->CR1, SPI_CR1_SSM);
                    }
                    SET_BIT(hspi->Instance->CR1, SPI_CR1_CRCNEXT);
                }
#endif /* USE_SPI_CRC */
            }

            /* Check RXNE flag */
            if ((hspi->RxXferCount > 0U) && (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE)))
            {
                *((uint16_t *)pRxData) = hspi->Instance->DATA;
                pRxData += sizeof(uint16_t);
                hspi->RxXferCount--;
                rx_fifo_left++;
            }
            if ((Timeout != HAL_MAX_DELAY) && ((HAL_GetTick() - tickstart) >=  Timeout))
            {
                errorcode = HAL_TIMEOUT;
                goto error;
            }
        }
    }
    /* Transmit and Receive data in 8 Bit mode */
    else
    {
//#ifndef HAL_SSP_VER
#if 1
        if ((hspi->Init.Mode == SPI_MODE_SLAVE) || (hspi->TxXferCount == 0x01U))
        {
            *((__IO uint8_t *)&hspi->Instance->DATA) = *((uint8_t *)pTxData);
            pTxData += sizeof(uint8_t);
            hspi->TxXferCount--;
        }


        while ((hspi->TxXferCount > 0U) || (hspi->RxXferCount > 0U))
        {
            /* check TXE flag */
            if ((rx_fifo_left > 0) && (hspi->TxXferCount > 0U) && (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_TXE)))
            {
                *(__IO uint8_t *)&hspi->Instance->DATA = *((uint8_t *)pTxData);
                pTxData += sizeof(uint8_t);
                hspi->TxXferCount--;
                rx_fifo_left--;
            }

            /* Wait until RXNE flag is reset */
            if ((hspi->RxXferCount > 0U) && (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE)))
            {
                (* (uint8_t *)pRxData) = *(__IO uint8_t *)&hspi->Instance->DATA;
                pRxData += sizeof(uint8_t);
                hspi->RxXferCount--;

                rx_fifo_left++;
            }
            if ((Timeout != HAL_MAX_DELAY) && ((HAL_GetTick() - tickstart) >=  Timeout))
            {
                errorcode = HAL_TIMEOUT;
                goto error;
            }
        }
#else

        uint8_t temp;
        //LOG_I("hspi->TxXferCount = %d,hspi->RxXferCount = %d",hspi->TxXferCount,hspi->RxXferCount);
        while ((hspi->TxXferCount > 0U))
        {
            hspi->Instance->DATA = (*pTxData++);
            hspi->TxXferCount--;
        }
        while ((hspi->RxXferCount > 0U))
        {
            /* Wait until RXNE flag is reset */
            if ((hspi->RxXferCount > 0U) && (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE)))
            {
                temp = hspi->Instance->DATA;
                (*(uint8_t *)pRxData++) = temp & 0xFF;
                hspi->RxXferCount--;
            }
            if ((Timeout != HAL_MAX_DELAY) && ((HAL_GetTick() - tickstart) >=  Timeout))
            {
                errorcode = HAL_TIMEOUT;
                goto error;
            }
        }

#endif
    }

    /* Check the end of the transaction */
    if (SPI_EndRxTxTransaction(hspi, Timeout, tickstart) != HAL_OK)
    {
        hspi->ErrorCode = HAL_SPI_ERROR_FLAG;
    }

    if (hspi->ErrorCode != HAL_SPI_ERROR_NONE)
    {
        errorcode = HAL_ERROR;
    }

error :
    hspi->State = HAL_SPI_STATE_READY;
    __HAL_UNLOCK(hspi);
    return errorcode;
}

/**
  * @brief  Transmit an amount of data in non-blocking mode with Interrupt.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  pData pointer to data buffer
  * @param  Size amount of data to be sent
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_SPI_Transmit_IT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size)
{
    HAL_StatusTypeDef errorcode = HAL_OK;
    SPI_PRINTF("HAL_SPI_Transmit_IT %x,data=%x,size=%d \r\n", hspi, pData, Size);

    if ((hspi->Init.DataSize > SPI_DATASIZE_8BIT)/* || (Size > 1U)*/)
    {
        /* in this case, 16-bit access is performed on Data
           So, check Data is 16-bit aligned address */
        HAL_ASSERT(IS_SPI_16BIT_ALIGNED_ADDRESS(pData));
    }

    /* Check Direction parameter */
    HAL_ASSERT(IS_SPI_DIRECTION_2LINES_OR_1LINE(hspi->Init.Direction));

    /* Process Locked */
    __HAL_LOCK(hspi);

    __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));
    if ((pData == NULL) || (Size == 0U))
    {
        errorcode = HAL_ERROR;
        goto error;
    }

    if (hspi->State != HAL_SPI_STATE_READY)
    {
        errorcode = HAL_BUSY;
        goto error;
    }

    /* Set the transaction information */
    hspi->State       = HAL_SPI_STATE_BUSY_TX;
    hspi->ErrorCode   = HAL_SPI_ERROR_NONE;
    hspi->pTxBuffPtr  = (uint8_t *)pData;
    hspi->TxXferSize  = Size;
    hspi->TxXferCount = Size;

    /* Init field not used in handle to zero */
    hspi->pRxBuffPtr  = (uint8_t *)NULL;
    hspi->RxXferSize  = 0U;
    hspi->RxXferCount = 0U;
    hspi->RxISR       = NULL;

    /* Set the function for IT treatment */
    if (hspi->Init.DataSize > SPI_DATASIZE_8BIT)
    {
        hspi->TxISR = SPI_TxISR_16BIT;
    }
    else
    {
        hspi->TxISR = SPI_TxISR_8BIT;
    }

#if (USE_SPI_CRC != 0U)
    /* Reset CRC Calculation */
    if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
    {
        SPI_RESET_CRC(hspi);
    }
#endif /* USE_SPI_CRC */

    if (hspi->Init.Direction == SPI_DIRECTION_1LINE)
    {
        /*1line tx enable*/
        SPI_1LINE_TX(hspi);
    }

    /* Enable TXE and ERR interrupt */
    __HAL_SPI_ENABLE_IT(hspi, (SPI_IT_TXE | SPI_IT_ERR));

    /* Check if the SPI is already enabled */
    if ((hspi->Instance->TOP_CTRL & SPI_TOP_CTRL_SSE) != SPI_TOP_CTRL_SSE)
    {
        /* Enable SPI peripheral */
        __HAL_SPI_ENABLE(hspi);
    }

error :
    __HAL_UNLOCK(hspi);
    return errorcode;
}

/**
  * @brief  Receive an amount of data in non-blocking mode with Interrupt.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  pData pointer to data buffer
  * @param  Size amount of data to be sent
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_SPI_Receive_IT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size)
{
    HAL_StatusTypeDef errorcode = HAL_OK;
    SPI_PRINTF("HAL_SPI_Receive_IT %x,data=%x,size=%d \r\n", hspi, pData, Size);

    if ((hspi->Init.DataSize > SPI_DATASIZE_8BIT)/* || (Size > 1U)*/)
    {
        /* in this case, 16-bit access is performed on Data
           So, check Data is 16-bit aligned address */
        HAL_ASSERT(IS_SPI_16BIT_ALIGNED_ADDRESS(pData));
    }

    if ((hspi->Init.Direction == SPI_DIRECTION_2LINES) && (hspi->Init.Mode == SPI_MODE_MASTER))
    {
        hspi->State = HAL_SPI_STATE_BUSY_RX;
        /* Call transmit-receive function to send Dummy data on Tx line and generate clock on CLK line */
        return HAL_SPI_TransmitReceive_IT(hspi, pData, pData, Size);
    }


    /* Process Locked */
    __HAL_LOCK(hspi);

    __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));

    if (hspi->State != HAL_SPI_STATE_READY)
    {
        errorcode = HAL_BUSY;
        goto error;
    }

    if ((pData == NULL) || (Size == 0U))
    {
        errorcode = HAL_ERROR;
        goto error;
    }

    /* Set the transaction information */
    hspi->State       = HAL_SPI_STATE_BUSY_RX;
    hspi->ErrorCode   = HAL_SPI_ERROR_NONE;
    hspi->pRxBuffPtr  = (uint8_t *)pData;
    hspi->RxXferSize  = Size;
    hspi->RxXferCount = Size;

    /* Init field not used in handle to zero */
    hspi->pTxBuffPtr  = (uint8_t *)NULL;
    hspi->TxXferSize  = 0U;
    hspi->TxXferCount = 0U;
    hspi->TxISR       = NULL;

    /* Check the data size to adapt Rx threshold and the set the function for IT treatment */
    if (hspi->Init.DataSize > SPI_DATASIZE_8BIT)
    {
        hspi->RxISR = SPI_RxISR_16BIT;
    }
    else
    {
        hspi->RxISR = SPI_RxISR_8BIT;
    }

#if (USE_SPI_CRC != 0U)
    /* Reset CRC Calculation */
    if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
    {
        hspi->CRCSize = 1U;
        if ((hspi->Init.DataSize <= SPI_DATASIZE_8BIT) && (hspi->Init.CRCLength == SPI_CRC_LENGTH_16BIT))
        {
            hspi->CRCSize = 2U;
        }
        SPI_RESET_CRC(hspi);
    }
    else
    {
        hspi->CRCSize = 0U;
    }
#endif /* USE_SPI_CRC */

    if (hspi->Init.Direction == SPI_DIRECTION_1LINE)
    {
        /*1line rx enable*/
        SPI_1LINE_RX(hspi);
    }

    /* Enable TXE and ERR interrupt */
    __HAL_SPI_ENABLE_IT(hspi, (SPI_IT_RXNE | SPI_IT_ERR));

    /* Note : The SPI must be enabled after unlocking current process
              to avoid the risk of SPI interrupt handle execution before current
              process unlock */

    /* Check if the SPI is already enabled */
    if ((hspi->Instance->TOP_CTRL & SPI_TOP_CTRL_SSE) != SPI_TOP_CTRL_SSE)
    {
        /* Enable SPI peripheral */
        __HAL_SPI_ENABLE(hspi);
    }

error :
    /* Process Unlocked */
    __HAL_UNLOCK(hspi);
    return errorcode;
}

/**
  * @brief  Transmit and Receive an amount of data in non-blocking mode with Interrupt.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  pTxData pointer to transmission data buffer
  * @param  pRxData pointer to reception data buffer
  * @param  Size amount of data to be sent and received
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_SPI_TransmitReceive_IT(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size)
{
    uint32_t tmp = 0U, tmp1 = 0U;
    HAL_StatusTypeDef errorcode = HAL_OK;

    SPI_PRINTF("HAL_SPI_TransmitReceive_IT %x,Txdata=%x, Rxdata=%x, Size=%d \r\n", hspi, pTxData, pRxData, Size);

    if ((hspi->Init.DataSize > SPI_DATASIZE_8BIT)/* || (Size > 1U)*/)
    {
        /* in this case, 16-bit access is performed on Data
           So, check Data is 16-bit aligned address */
        HAL_ASSERT(IS_SPI_16BIT_ALIGNED_ADDRESS(pTxData));
        HAL_ASSERT(IS_SPI_16BIT_ALIGNED_ADDRESS(pRxData));
    }

    /* Check Direction parameter */
    HAL_ASSERT(IS_SPI_DIRECTION_2LINES(hspi->Init.Direction));

    /* Process locked */
    __HAL_LOCK(hspi);

    __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));
    tmp  = hspi->State;
    tmp1 = hspi->Init.Mode;

    if (!((tmp == HAL_SPI_STATE_READY) || \
            ((tmp1 == SPI_MODE_MASTER) && (hspi->Init.Direction == SPI_DIRECTION_2LINES) && (tmp == HAL_SPI_STATE_BUSY_RX))))
    {
        errorcode = HAL_BUSY;
        goto error;
    }

    if ((pTxData == NULL) || (pRxData == NULL) || (Size == 0U))
    {
        errorcode = HAL_ERROR;
        goto error;
    }

    /* Don't overwrite in case of HAL_SPI_STATE_BUSY_RX */
    if (hspi->State != HAL_SPI_STATE_BUSY_RX)
    {
        hspi->State = HAL_SPI_STATE_BUSY_TX_RX;
    }

    /* Set the transaction information */
    hspi->ErrorCode   = HAL_SPI_ERROR_NONE;
    hspi->pTxBuffPtr  = (uint8_t *)pTxData;
    hspi->TxXferSize  = Size;
    hspi->TxXferCount = Size;
    hspi->pRxBuffPtr  = (uint8_t *)pRxData;
    hspi->RxXferSize  = Size;
    hspi->RxXferCount = Size;

    /* Set the function for IT treatment */
    if (hspi->Init.DataSize > SPI_DATASIZE_8BIT)
    {
        hspi->RxISR     = SPI_2linesRxISR_16BIT;
        hspi->TxISR     = SPI_2linesTxISR_16BIT;
    }
    else
    {
        hspi->RxISR     = SPI_2linesRxISR_8BIT;
        hspi->TxISR     = SPI_2linesTxISR_8BIT;
    }

#if (USE_SPI_CRC != 0U)
    /* Reset CRC Calculation */
    if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
    {
        hspi->CRCSize = 1U;
        if ((hspi->Init.DataSize <= SPI_DATASIZE_8BIT) && (hspi->Init.CRCLength == SPI_CRC_LENGTH_16BIT))
        {
            hspi->CRCSize = 2U;
        }
        SPI_RESET_CRC(hspi);
    }
    else
    {
        hspi->CRCSize = 0U;
    }
#endif /* USE_SPI_CRC */

    /* Enable TXE, RXNE and ERR interrupt */
    __HAL_SPI_ENABLE_IT(hspi, (SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));

    /* Check if the SPI is already enabled */
    if ((hspi->Instance->TOP_CTRL & SPI_TOP_CTRL_SSE) != SPI_TOP_CTRL_SSE)
    {
        /* Enable SPI peripheral */
        __HAL_SPI_ENABLE(hspi);
    }

error :
    /* Process Unlocked */
    __HAL_UNLOCK(hspi);
    return errorcode;
}

/**
  * @brief  Transmit an amount of data in non-blocking mode with DMA.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  pData pointer to data buffer
  * @param  Size amount of data to be sent
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_SPI_Transmit_DMA(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size)
{
    HAL_StatusTypeDef errorcode = HAL_OK;

    SPI_PRINTF("HAL_SPI_Transmit_DMA %x,data=%x,size=%d \r\n", hspi, pData, Size);

    /* check tx dma handle */
    HAL_ASSERT(IS_SPI_DMA_HANDLE(hspi->hdmatx));

    /* Check Direction parameter */
    HAL_ASSERT(IS_SPI_DIRECTION_2LINES_OR_1LINE(hspi->Init.Direction));

    /* Process Locked */
    __HAL_LOCK(hspi);

    __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));
    if (hspi->State != HAL_SPI_STATE_READY)
    {
        errorcode = HAL_BUSY;
        goto error;
    }

    if ((pData == NULL) || (Size == 0U))
    {
        errorcode = HAL_ERROR;
        goto error;
    }

    /* Set the transaction information */
    hspi->State       = HAL_SPI_STATE_BUSY_TX;
    hspi->ErrorCode   = HAL_SPI_ERROR_NONE;
    hspi->pTxBuffPtr  = (uint8_t *)pData;
    hspi->TxXferSize  = Size;
    hspi->TxXferCount = Size;

    /* Init field not used in handle to zero */
    hspi->pRxBuffPtr  = (uint8_t *)NULL;
    hspi->TxISR       = NULL;
    hspi->RxISR       = NULL;
    hspi->RxXferSize  = 0U;
    hspi->RxXferCount = 0U;


#if (USE_SPI_CRC != 0U)
    /* Reset CRC Calculation */
    if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
    {
        SPI_RESET_CRC(hspi);
    }
#endif /* USE_SPI_CRC */

    if (hspi->Init.Direction == SPI_DIRECTION_1LINE)
    {
        /*1line tx enable*/
        SPI_1LINE_TX(hspi);
    }

    /* Set the SPI TxDMA Half transfer complete callback */
    hspi->hdmatx->XferHalfCpltCallback = SPI_DMAHalfTransmitCplt;

    /* Set the SPI TxDMA transfer complete callback */
    hspi->hdmatx->XferCpltCallback = SPI_DMATransmitCplt;

    /* Set the DMA error callback */
    hspi->hdmatx->XferErrorCallback = SPI_DMAError;

    /* Set the DMA AbortCpltCallback */
    hspi->hdmatx->XferAbortCallback = NULL;

    /* Packing mode is enabled only if the DMA setting is HALWORD */
    if ((hspi->Init.DataSize <= SPI_DATASIZE_8BIT) && (hspi->hdmatx->Init.MemDataAlignment == DMA_MDATAALIGN_HALFWORD))
    {
        /* Check the even/odd of the data size + crc if enabled */
        if ((hspi->TxXferCount & 0x1U) == 0U)
        {
            hspi->TxXferCount = (hspi->TxXferCount >> 1U);
        }
        else
        {
            hspi->TxXferCount = (hspi->TxXferCount >> 1U) + 1U;
        }
    }

    __HAL_SPI_ENABLE_IT(hspi, (SPI_IT_TXE));
    hspi->Instance->FIFO_CTRL |= SPI_FIFO_CTRL_TSRE;
    hspi->Instance->FIFO_CTRL &= ~SPI_FIFO_CTRL_RSRE;

    /* Enable the Tx DMA Stream/Channel */
    HAL_DMA_Start_IT(hspi->hdmatx, (uint32_t)hspi->pTxBuffPtr, (uint32_t)&hspi->Instance->DATA, hspi->TxXferCount);

    /* Check if the SPI is already enabled */
    if ((hspi->Instance->TOP_CTRL & SPI_TOP_CTRL_SSE) != SPI_TOP_CTRL_SSE)
    {
        /* Enable SPI peripheral */
        __HAL_SPI_ENABLE(hspi);
    }

    /* Enable the SPI Error Interrupt Bit */
    __HAL_SPI_ENABLE_IT(hspi, (SPI_IT_ERR));

error :
    /* Process Unlocked */
    __HAL_UNLOCK(hspi);
    return errorcode;
}

/**
  * @brief  Receive an amount of data in non-blocking mode with DMA.
  * @note   In case of MASTER mode and SPI_DIRECTION_2LINES direction, hdmatx shall be defined.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  pData pointer to data buffer
  * @note   When the CRC feature is enabled the pData Length must be Size + 1.
  * @param  Size amount of data to be sent
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_SPI_Receive_DMA(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size)
{
    HAL_StatusTypeDef errorcode = HAL_OK;
    SPI_PRINTF("HAL_SPI_Receive_DMA %x,data=%x,size=%d \r\n", hspi, pData, Size);

    /* check rx dma handle */
    HAL_ASSERT(IS_SPI_DMA_HANDLE(hspi->hdmarx));

    if ((hspi->Init.Direction == SPI_DIRECTION_2LINES) && (hspi->Init.Mode == SPI_MODE_MASTER))
    {
        hspi->State = HAL_SPI_STATE_BUSY_RX;

        /* check tx dma handle */
        HAL_ASSERT(IS_SPI_DMA_HANDLE(hspi->hdmatx));

        /* Call transmit-receive function to send Dummy data on Tx line and generate clock on CLK line */
        return HAL_SPI_TransmitReceive_DMA(hspi, pData, pData, Size);
    }

    /* Process Locked */
    __HAL_LOCK(hspi);
    __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));

    if (hspi->State != HAL_SPI_STATE_READY)
    {
        errorcode = HAL_BUSY;
        goto error;
    }

    if ((pData == NULL) || (Size == 0U))
    {
        errorcode = HAL_ERROR;
        goto error;
    }

    /* Set the transaction information */
    hspi->State       = HAL_SPI_STATE_BUSY_RX;
    hspi->ErrorCode   = HAL_SPI_ERROR_NONE;
    hspi->pRxBuffPtr  = (uint8_t *)pData;
    hspi->RxXferSize  = Size;
    hspi->RxXferCount = Size;

    /*Init field not used in handle to zero */
    hspi->RxISR       = NULL;
    hspi->TxISR       = NULL;
    hspi->TxXferSize  = 0U;
    hspi->TxXferCount = 0U;


#if (USE_SPI_CRC != 0U)
    /* Reset CRC Calculation */
    if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
    {
        SPI_RESET_CRC(hspi);
    }
#endif /* USE_SPI_CRC */

    if (hspi->Init.Direction == SPI_DIRECTION_1LINE)
    {
        /*1line rx enable*/
        SPI_1LINE_RX(hspi);
    }

    if (hspi->Init.DataSize > SPI_DATASIZE_8BIT)
    {
    }
    else
    {
        if (hspi->hdmarx->Init.MemDataAlignment == DMA_MDATAALIGN_HALFWORD)
        {
            if ((hspi->RxXferCount & 0x1U) == 0x0U)
            {
                hspi->RxXferCount = hspi->RxXferCount >> 1U;
            }
            else
            {
                hspi->RxXferCount = (hspi->RxXferCount >> 1U) + 1U;
            }
        }
    }

    /* Set the SPI RxDMA Half transfer complete callback */
    hspi->hdmarx->XferHalfCpltCallback = SPI_DMAHalfReceiveCplt;

    /* Set the SPI Rx DMA transfer complete callback */
    hspi->hdmarx->XferCpltCallback = SPI_DMAReceiveCplt;

    /* Set the DMA error callback */
    hspi->hdmarx->XferErrorCallback = SPI_DMAError;

    /* Set the DMA AbortCpltCallback */
    hspi->hdmarx->XferAbortCallback = NULL;

    __HAL_SPI_ENABLE_IT(hspi, (SPI_IT_RXNE));
    hspi->Instance->FIFO_CTRL |= SPI_FIFO_CTRL_RSRE;
    hspi->Instance->FIFO_CTRL &= ~SPI_FIFO_CTRL_TSRE;

    /* Enable the Rx DMA Stream/Channel  */
    HAL_DMA_Start_IT(hspi->hdmarx, (uint32_t)&hspi->Instance->DATA, (uint32_t)hspi->pRxBuffPtr, hspi->RxXferCount);

    /* Check if the SPI is already enabled */
    if ((hspi->Instance->TOP_CTRL & SPI_TOP_CTRL_SSE) != SPI_TOP_CTRL_SSE)
    {
        /* Enable SPI peripheral */
        __HAL_SPI_ENABLE(hspi);
    }

    /* Enable the SPI Error Interrupt Bit */
    __HAL_SPI_ENABLE_IT(hspi, (SPI_IT_ERR));

error:
    /* Process Unlocked */
    __HAL_UNLOCK(hspi);
    return errorcode;
}

/**
  * @brief  Transmit and Receive an amount of data in non-blocking mode with DMA.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  pTxData pointer to transmission data buffer
  * @param  pRxData pointer to reception data buffer
  * @note   When the CRC feature is enabled the pRxData Length must be Size + 1
  * @param  Size amount of data to be sent
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData,
        uint16_t Size)
{
    uint32_t tmp = 0U, tmp1 = 0U;
    HAL_StatusTypeDef errorcode = HAL_OK;

    SPI_PRINTF("HAL_SPI_TransmitReceive_DMA %x,Txdata=%x, Rxdata=%x, Size=%d \r\n", hspi, pTxData, pRxData, Size);

    /* check rx & tx dma handles */
    HAL_ASSERT(IS_SPI_DMA_HANDLE(hspi->hdmarx));
    HAL_ASSERT(IS_SPI_DMA_HANDLE(hspi->hdmatx));

    /* Check Direction parameter */
    HAL_ASSERT(IS_SPI_DIRECTION_2LINES(hspi->Init.Direction));

    /* Process locked */
    __HAL_LOCK(hspi);
    __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));

    tmp  = hspi->State;
    tmp1 = hspi->Init.Mode;
    if (!((tmp == HAL_SPI_STATE_READY) ||
            ((tmp1 == SPI_MODE_MASTER) && (hspi->Init.Direction == SPI_DIRECTION_2LINES) && (tmp == HAL_SPI_STATE_BUSY_RX))))
    {
        errorcode = HAL_BUSY;
        goto error;
    }

    if ((pTxData == NULL) || (pRxData == NULL) || (Size == 0U))
    {
        errorcode = HAL_ERROR;
        goto error;
    }

    /* Don't overwrite in case of HAL_SPI_STATE_BUSY_RX */
    if (hspi->State != HAL_SPI_STATE_BUSY_RX)
    {
        hspi->State = HAL_SPI_STATE_BUSY_TX_RX;
    }

    /* Set the transaction information */
    hspi->ErrorCode   = HAL_SPI_ERROR_NONE;
    hspi->pTxBuffPtr  = (uint8_t *)pTxData;
    hspi->TxXferSize  = Size;
    hspi->TxXferCount = Size;
    hspi->pRxBuffPtr  = (uint8_t *)pRxData;
    hspi->RxXferSize  = Size;
    hspi->RxXferCount = Size;

    /* Init field not used in handle to zero */
    hspi->RxISR       = NULL;
    hspi->TxISR       = NULL;

#if (USE_SPI_CRC != 0U)
    /* Reset CRC Calculation */
    if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
    {
        SPI_RESET_CRC(hspi);
    }
#endif /* USE_SPI_CRC */

    /* The packing mode management is enabled by the DMA settings according the spi data size */
    if (hspi->Init.DataSize > SPI_DATASIZE_8BIT)
    {
    }
    else
    {
        if (hspi->hdmatx->Init.MemDataAlignment == DMA_MDATAALIGN_HALFWORD)
        {
            if ((hspi->TxXferSize & 0x1U) == 0x0U)
            {
                hspi->TxXferCount = hspi->TxXferCount >> 1U;
            }
            else
            {
                hspi->TxXferCount = (hspi->TxXferCount >> 1U) + 1U;
            }
        }

        if (hspi->hdmarx->Init.MemDataAlignment == DMA_MDATAALIGN_HALFWORD)
        {
            if ((hspi->RxXferCount & 0x1U) == 0x0U)
            {
                hspi->RxXferCount = hspi->RxXferCount >> 1U;
            }
            else
            {
                hspi->RxXferCount = (hspi->RxXferCount >> 1U) + 1U;
            }
        }
    }

    /* Check if we are in Rx only or in Rx/Tx Mode and configure the DMA transfer complete callback */
    if (hspi->State == HAL_SPI_STATE_BUSY_RX)
    {
        /* Set the SPI Rx DMA Half transfer complete callback */
        hspi->hdmarx->XferHalfCpltCallback = SPI_DMAHalfReceiveCplt;
        hspi->hdmarx->XferCpltCallback     = SPI_DMAReceiveCplt;
    }
    else
    {
        /* Set the SPI Tx/Rx DMA Half transfer complete callback */
        hspi->hdmarx->XferHalfCpltCallback = SPI_DMAHalfTransmitReceiveCplt;
        hspi->hdmarx->XferCpltCallback     = SPI_DMATransmitReceiveCplt;
    }

    /* Set the DMA error callback */
    hspi->hdmarx->XferErrorCallback = SPI_DMAError;

    /* Set the DMA AbortCpltCallback */
    hspi->hdmarx->XferAbortCallback = NULL;

    __HAL_SPI_ENABLE_IT(hspi, (SPI_IT_RXNE));
    hspi->Instance->FIFO_CTRL |= SPI_FIFO_CTRL_RSRE;

    /* Enable the Rx DMA Stream/Channel  */
    HAL_DMA_Start_IT(hspi->hdmarx, (uint32_t)&hspi->Instance->DATA, (uint32_t)hspi->pRxBuffPtr, hspi->RxXferCount);

    /* Set the SPI Tx DMA transfer complete callback as NULL because the communication closing
    is performed in DMA reception complete callback  */
    hspi->hdmatx->XferHalfCpltCallback = NULL;
    hspi->hdmatx->XferCpltCallback     = NULL;
    hspi->hdmatx->XferErrorCallback    = NULL;
    hspi->hdmatx->XferAbortCallback    = NULL;

    __HAL_SPI_ENABLE_IT(hspi, (SPI_IT_TXE));
    hspi->Instance->FIFO_CTRL |= SPI_FIFO_CTRL_TSRE | SPI_FIFO_CTRL_RXFIFO_AUTO_FULL_CTRL;
    /* Enable the Tx DMA Stream/Channel  */
    HAL_DMA_Start_IT(hspi->hdmatx, (uint32_t)hspi->pTxBuffPtr, (uint32_t)&hspi->Instance->DATA, hspi->TxXferCount);

    /* Check if the SPI is already enabled */
    if ((hspi->Instance->TOP_CTRL & SPI_TOP_CTRL_SSE) != SPI_TOP_CTRL_SSE)
    {
        /* Enable SPI peripheral */
        __HAL_SPI_ENABLE(hspi);
    }
    /* Enable the SPI Error Interrupt Bit */
    __HAL_SPI_ENABLE_IT(hspi, (SPI_IT_ERR));

error :
    /* Process Unlocked */
    __HAL_UNLOCK(hspi);
    return errorcode;
}

/**
  * @brief  Abort ongoing transfer (blocking mode).
  * @param  hspi SPI handle.
  * @note   This procedure could be used for aborting any ongoing transfer (Tx and Rx),
  *         started in Interrupt or DMA mode.
  *         This procedure performs following operations :
  *           - Disable SPI Interrupts (depending of transfer direction)
  *           - Disable the DMA transfer in the peripheral register (if enabled)
  *           - Abort DMA transfer by calling HAL_DMA_Abort (in case of transfer in DMA mode)
  *           - Set handle State to READY
  * @note   This procedure is executed in blocking mode : when exiting function, Abort is considered as completed.
  * @retval HAL status
*/
__HAL_ROM_USED HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *hspi)
{
    HAL_StatusTypeDef errorcode;
    __IO uint32_t count, resetcount;

    /* Initialized local variable  */
    errorcode = HAL_OK;
    resetcount = SPI_DEFAULT_TIMEOUT * (HAL_RCC_GetPCLKFreq(hspi->core, 1) / 24U / 1000U);
    count = resetcount;

    /* Disable TXEIE, RXNEIE and ERRIE(mode fault event, overrun error, TI frame error) interrupts */
    if (__HAL_SPI_IS_ENABLE_IT(hspi, SPI_IT_TXE))
    {
        hspi->TxISR = SPI_AbortTx_ISR;
        /* Wait HAL_SPI_STATE_ABORT state */
        do
        {
            if (count-- == 0U)
            {
                SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_ABORT);
                break;
            }
        }
        while (hspi->State != HAL_SPI_STATE_ABORT);
        /* Reset Timeout Counter */
        count = resetcount;
    }

    if (__HAL_SPI_IS_ENABLE_IT(hspi, SPI_IT_RXNE))
    {
        hspi->RxISR = SPI_AbortRx_ISR;
        /* Wait HAL_SPI_STATE_ABORT state */
        do
        {
            if (count-- == 0U)
            {
                SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_ABORT);
                break;
            }
        }
        while (hspi->State != HAL_SPI_STATE_ABORT);
        /* Reset Timeout Counter */
        count = resetcount;
    }

    /* Clear ERRIE interrupts in case of DMA Mode */
    __HAL_SPI_DISABLE_IT(hspi, SPI_IT_ERR);


#if 0
    /* Disable the SPI DMA Tx or SPI DMA Rx request if enabled */
    if ((HAL_IS_BIT_SET(hspi->Instance->CR2, SPI_CR2_TXDMAEN)) || (HAL_IS_BIT_SET(hspi->Instance->CR2, SPI_CR2_RXDMAEN)))
    {
        /* Abort the SPI DMA Tx Stream/Channel : use blocking DMA Abort API (no callback) */
        if (hspi->hdmatx != NULL)
        {
            /* Set the SPI DMA Abort callback :
            will lead to call HAL_SPI_AbortCpltCallback() at end of DMA abort procedure */
            hspi->hdmatx->XferAbortCallback = NULL;

            /* Abort DMA Tx Handle linked to SPI Peripheral */
            if (HAL_DMA_Abort(hspi->hdmatx) != HAL_OK)
            {
                hspi->ErrorCode = HAL_SPI_ERROR_ABORT;
            }

            /* Disable Tx DMA Request */
            CLEAR_BIT(hspi->Instance->CR2, (SPI_CR2_TXDMAEN));

            if (SPI_EndRxTxTransaction(hspi, SPI_DEFAULT_TIMEOUT, HAL_GetTick()) != HAL_OK)
            {
                hspi->ErrorCode = HAL_SPI_ERROR_ABORT;
            }

            /* Disable SPI Peripheral */
            __HAL_SPI_DISABLE(hspi);

            /* Empty the FRLVL fifo */
            if (SPI_WaitFifoStateUntilTimeout(hspi, SPI_FLAG_FRLVL, SPI_FRLVL_EMPTY, SPI_DEFAULT_TIMEOUT, HAL_GetTick()) != HAL_OK)
            {
                hspi->ErrorCode = HAL_SPI_ERROR_ABORT;
            }
        }
        /* Abort the SPI DMA Rx Stream/Channel : use blocking DMA Abort API (no callback) */
        if (hspi->hdmarx != NULL)
        {
            /* Set the SPI DMA Abort callback :
            will lead to call HAL_SPI_AbortCpltCallback() at end of DMA abort procedure */
            hspi->hdmarx->XferAbortCallback = NULL;

            /* Abort DMA Rx Handle linked to SPI Peripheral */
            if (HAL_DMA_Abort(hspi->hdmarx) != HAL_OK)
            {
                hspi->ErrorCode = HAL_SPI_ERROR_ABORT;
            }

            /* Disable peripheral */
            __HAL_SPI_DISABLE(hspi);

            /* Control the BSY flag */
            if (SPI_WaitFlagStateUntilTimeout(hspi, SPI_FLAG_BSY, RESET, SPI_DEFAULT_TIMEOUT, HAL_GetTick()) != HAL_OK)
            {
                hspi->ErrorCode = HAL_SPI_ERROR_ABORT;
            }

            /* Empty the FRLVL fifo */
            if (SPI_WaitFifoStateUntilTimeout(hspi, SPI_FLAG_FRLVL, SPI_FRLVL_EMPTY, SPI_DEFAULT_TIMEOUT, HAL_GetTick()) != HAL_OK)
            {
                hspi->ErrorCode = HAL_SPI_ERROR_ABORT;
            }

            /* Disable Rx DMA Request */
            CLEAR_BIT(hspi->Instance->CR2, (SPI_CR2_RXDMAEN));
        }
    }

#endif
    /* Reset Tx and Rx transfer counters */
    hspi->RxXferCount = 0U;
    hspi->TxXferCount = 0U;

    /* Check error during Abort procedure */
    if (hspi->ErrorCode == HAL_SPI_ERROR_ABORT)
    {
        /* return HAL_Error in case of error during Abort procedure */
        errorcode = HAL_ERROR;
    }
    else
    {
        /* Reset errorCode */
        hspi->ErrorCode = HAL_SPI_ERROR_NONE;
    }


#if 0
    /* Clear the Error flags in the SR register */
    __HAL_SPI_CLEAR_OVRFLAG(hspi);
    __HAL_SPI_CLEAR_FREFLAG(hspi);
#endif

    /* Restore hspi->state to ready */
    hspi->State = HAL_SPI_STATE_READY;

    return errorcode;
}

/**
  * @brief  Abort ongoing transfer (Interrupt mode).
  * @param  hspi SPI handle.
  * @note   This procedure could be used for aborting any ongoing transfer (Tx and Rx),
  *         started in Interrupt or DMA mode.
  *         This procedure performs following operations :
  *           - Disable SPI Interrupts (depending of transfer direction)
  *           - Disable the DMA transfer in the peripheral register (if enabled)
  *           - Abort DMA transfer by calling HAL_DMA_Abort_IT (in case of transfer in DMA mode)
  *           - Set handle State to READY
  *           - At abort completion, call user abort complete callback
  * @note   This procedure is executed in Interrupt mode, meaning that abort procedure could be
  *         considered as completed only when user abort complete callback is executed (not when exiting function).
  * @retval HAL status
*/
__HAL_ROM_USED HAL_StatusTypeDef HAL_SPI_Abort_IT(SPI_HandleTypeDef *hspi)
{
    HAL_StatusTypeDef errorcode;
    uint32_t abortcplt ;
    __IO uint32_t count, resetcount;

    /* Initialized local variable  */
    errorcode = HAL_OK;
    abortcplt = 1U;
    resetcount = SPI_DEFAULT_TIMEOUT * (HAL_RCC_GetPCLKFreq(hspi->core, 1) / 24U / 1000U);
    count = resetcount;

    /* Change Rx and Tx Irq Handler to Disable TXEIE, RXNEIE and ERRIE interrupts */
    if (__HAL_SPI_IS_ENABLE_IT(hspi, SPI_IT_TXE))
    {
        hspi->TxISR = SPI_AbortTx_ISR;
        /* Wait HAL_SPI_STATE_ABORT state */
        do
        {
            if (count-- == 0U)
            {
                SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_ABORT);
                break;
            }
        }
        while (hspi->State != HAL_SPI_STATE_ABORT);
        /* Reset Timeout Counter */
        count = resetcount;
    }

    if (__HAL_SPI_IS_ENABLE_IT(hspi, SPI_IT_RXNE))
    {
        hspi->RxISR = SPI_AbortRx_ISR;
        /* Wait HAL_SPI_STATE_ABORT state */
        do
        {
            if (count-- == 0U)
            {
                SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_ABORT);
                break;
            }
        }
        while (hspi->State != HAL_SPI_STATE_ABORT);
        /* Reset Timeout Counter */
        count = resetcount;
    }

    /* Clear ERRIE interrupts in case of DMA Mode */
    __HAL_SPI_DISABLE_IT(hspi, SPI_IT_ERR);


#if 0
    /* If DMA Tx and/or DMA Rx Handles are associated to SPI Handle, DMA Abort complete callbacks should be initialised
       before any call to DMA Abort functions */
    /* DMA Tx Handle is valid */
    if (hspi->hdmatx != NULL)
    {
        /* Set DMA Abort Complete callback if UART DMA Tx request if enabled.
           Otherwise, set it to NULL */
        if (HAL_IS_BIT_SET(hspi->Instance->CR1, SPI_CR1_TXDMAEN))
        {
            hspi->hdmatx->XferAbortCallback = SPI_DMATxAbortCallback;
        }
        else
        {
            hspi->hdmatx->XferAbortCallback = NULL;
        }
    }
    /* DMA Rx Handle is valid */
    if (hspi->hdmarx != NULL)
    {
        /* Set DMA Abort Complete callback if UART DMA Rx request if enabled.
           Otherwise, set it to NULL */
        if (HAL_IS_BIT_SET(hspi->Instance->CR2, SPI_CR2_RXDMAEN))
        {
            hspi->hdmarx->XferAbortCallback = SPI_DMARxAbortCallback;
        }
        else
        {
            hspi->hdmarx->XferAbortCallback = NULL;
        }
    }

    /* Disable the SPI DMA Tx or the SPI Rx request if enabled */
    if ((HAL_IS_BIT_SET(hspi->Instance->CR2, SPI_CR2_TXDMAEN)) && (HAL_IS_BIT_SET(hspi->Instance->CR2, SPI_CR2_RXDMAEN)))
    {
        /* Abort the SPI DMA Tx Stream/Channel */
        if (hspi->hdmatx != NULL)
        {
            /* Abort DMA Tx Handle linked to SPI Peripheral */
            if (HAL_DMA_Abort_IT(hspi->hdmatx) != HAL_OK)
            {
                hspi->hdmatx->XferAbortCallback = NULL;
                hspi->ErrorCode = HAL_SPI_ERROR_ABORT;
            }
            else
            {
                abortcplt = 0U;
            }
        }
        /* Abort the SPI DMA Rx Stream/Channel */
        if (hspi->hdmarx != NULL)
        {
            /* Abort DMA Rx Handle linked to SPI Peripheral */
            if (HAL_DMA_Abort_IT(hspi->hdmarx) !=  HAL_OK)
            {
                hspi->hdmarx->XferAbortCallback = NULL;
                hspi->ErrorCode = HAL_SPI_ERROR_ABORT;
                abortcplt = 1U;
            }
            else
            {
                abortcplt = 0U;
            }
        }
    }

    /* Disable the SPI DMA Tx or the SPI Rx request if enabled */
    if (HAL_IS_BIT_SET(hspi->Instance->CR2, SPI_CR2_TXDMAEN))
    {
        /* Abort the SPI DMA Tx Stream/Channel */
        if (hspi->hdmatx != NULL)
        {
            /* Abort DMA Tx Handle linked to SPI Peripheral */
            if (HAL_DMA_Abort_IT(hspi->hdmatx) != HAL_OK)
            {
                hspi->hdmatx->XferAbortCallback = NULL;
                hspi->ErrorCode = HAL_SPI_ERROR_ABORT;
            }
            else
            {
                abortcplt = 0U;
            }
        }
    }
    /* Disable the SPI DMA Tx or the SPI Rx request if enabled */
    if (HAL_IS_BIT_SET(hspi->Instance->CR2, SPI_CR2_RXDMAEN))
    {
        /* Abort the SPI DMA Rx Stream/Channel */
        if (hspi->hdmarx != NULL)
        {
            /* Abort DMA Rx Handle linked to SPI Peripheral */
            if (HAL_DMA_Abort_IT(hspi->hdmarx) !=  HAL_OK)
            {
                hspi->hdmarx->XferAbortCallback = NULL;
                hspi->ErrorCode = HAL_SPI_ERROR_ABORT;
            }
            else
            {
                abortcplt = 0U;
            }
        }
    }


#endif

    abortcplt = 1;

    if (abortcplt == 1U)
    {
        /* Reset Tx and Rx transfer counters */
        hspi->RxXferCount = 0U;
        hspi->TxXferCount = 0U;

        /* Check error during Abort procedure */
        if (hspi->ErrorCode == HAL_SPI_ERROR_ABORT)
        {
            /* return HAL_Error in case of error during Abort procedure */
            errorcode = HAL_ERROR;
        }
        else
        {
            /* Reset errorCode */
            hspi->ErrorCode = HAL_SPI_ERROR_NONE;
        }

#if 0
        /* Clear the Error flags in the SR register */
        __HAL_SPI_CLEAR_OVRFLAG(hspi);
        __HAL_SPI_CLEAR_FREFLAG(hspi);
#endif

        /* Restore hspi->State to Ready */
        hspi->State = HAL_SPI_STATE_READY;

        /* As no DMA to be aborted, call directly user Abort complete callback */
        HAL_SPI_AbortCpltCallback(hspi);
    }

    return errorcode;
}

/**
  * @brief  Pause the DMA Transfer.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for the specified SPI module.
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_SPI_DMAPause(SPI_HandleTypeDef *hspi)
{
    /* Process Locked */
    __HAL_LOCK(hspi);

    /* Disable the SPI DMA Tx & Rx requests */
    //CLEAR_BIT(hspi->Instance->CR2, SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);

    /* Process Unlocked */
    __HAL_UNLOCK(hspi);

    return HAL_OK;
}

/**
  * @brief  Resume the DMA Transfer.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for the specified SPI module.
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_SPI_DMAResume(SPI_HandleTypeDef *hspi)
{
    /* Process Locked */
    __HAL_LOCK(hspi);

    /* Enable the SPI DMA Tx & Rx requests */
    //SET_BIT(hspi->Instance->CR2, SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);

    /* Process Unlocked */
    __HAL_UNLOCK(hspi);

    return HAL_OK;
}

/**
  * @brief Stop the DMA Transfer.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for the specified SPI module.
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_SPI_DMAStop(SPI_HandleTypeDef *hspi)
{
    /* The Lock is not implemented on this API to allow the user application
       to call the HAL SPI API under callbacks HAL_SPI_TxCpltCallback() or HAL_SPI_RxCpltCallback() or HAL_SPI_TxRxCpltCallback():
       when calling HAL_DMA_Abort() API the DMA TX/RX Transfer complete interrupt is generated
       and the correspond call back is executed HAL_SPI_TxCpltCallback() or HAL_SPI_RxCpltCallback() or HAL_SPI_TxRxCpltCallback()
       */

    /* Abort the SPI DMA tx Stream/Channel  */
    if (hspi->hdmatx != NULL)
    {
        HAL_DMA_Abort(hspi->hdmatx);
    }
    /* Abort the SPI DMA rx Stream/Channel  */
    if (hspi->hdmarx != NULL)
    {
        HAL_DMA_Abort(hspi->hdmarx);
    }

    /* Disable the SPI DMA Tx & Rx requests */
    //CLEAR_BIT(hspi->Instance->CR2, SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);
    hspi->State = HAL_SPI_STATE_READY;
    return HAL_OK;
}

/**
  * @brief  Handle SPI interrupt request.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for the specified SPI module.
  * @retval None
  */
__HAL_ROM_USED void HAL_SPI_IRQHandler(SPI_HandleTypeDef *hspi)
{
    uint32_t itsource = hspi->Instance->INTE;
    uint32_t itflag   = hspi->Instance->STATUS;
    //SPI_PRINTF("SPI%d IRQ, mask=%x, it=%x  \r\n", SPI_GetIdx(hspi), itsource, itflag);

    /* SPI in mode Receiver ----------------------------------------------------*/
    if (((itflag & SPI_FLAG_OVR) == RESET) &&
            ((itflag & SPI_STATUS_RFS) != RESET) && ((itsource & SPI_IT_RXNE) != RESET))
    {
        HAL_ASSERT(hspi->RxISR);
        hspi->RxISR(hspi);
    }

    /* SPI in mode Transmitter -------------------------------------------------*/
    if (((itflag & SPI_FLAG_UDR) == RESET) &&
            ((itflag & SPI_STATUS_TFS) != RESET) && ((itsource & SPI_IT_TXE) != RESET))
    {
        HAL_ASSERT(hspi->TxISR);
        hspi->TxISR(hspi);
    }


    /* SPI in Error Treatment --------------------------------------------------*/
    if (((itflag & (SPI_FLAG_OVR | SPI_FLAG_UDR)) != RESET) && ((itsource & SPI_IT_ERR) != RESET))
    {
        SPI_PRINTF("SPI%d ERR, mask=%x, it=%x  \r\n", SPI_GetIdx(hspi), itsource, itflag);

        /* SPI Overrun error interrupt occurred ----------------------------------*/
        if ((itflag & SPI_FLAG_OVR) != RESET)
        {
            if (hspi->State != HAL_SPI_STATE_BUSY_TX)
            {
                SPI_PRINTF("SPI%d Rx overrun \r\n", SPI_GetIdx(hspi));
                SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_OVR);
                __HAL_SPI_CLEAR_OVRFLAG(hspi);
            }
            else
            {
                /*Ignore rx overrun if we are TX only,as rx&tx always work together*/
                __HAL_SPI_CLEAR_OVRFLAG(hspi);
                return;
            }
        }

        /* SPI Underrun error interrupt occurred ----------------------------------*/
        if ((itflag & SPI_FLAG_UDR) != RESET)
        {
            if (hspi->State != HAL_SPI_STATE_BUSY_RX)
            {
                SPI_PRINTF("SPI%d Tx underrun \r\n", SPI_GetIdx(hspi));
                SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_UDR);
                __HAL_SPI_CLEAR_UDRFLAG(hspi);
            }
            else
            {
                /*Ignore tx underrun if we are RX only,as rx&tx always work together*/
                __HAL_SPI_CLEAR_UDRFLAG(hspi);
                return;
            }
        }

        if ((itflag & SPI_FLAG_EXT) != RESET)
        {
            SPI_PRINTF("Ext error \r\n", itflag & SPI_FLAG_EXT);

            HAL_ASSERT(0);
            //__HAL_SPI_CLEAR_EXT_ERR_FLAG(hspi);
        }

#if 0 //Unsupported error
        /* SPI Mode Fault error interrupt occurred -------------------------------*/
        if ((itflag & SPI_FLAG_MODF) != RESET)
        {
            SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_MODF);
            __HAL_SPI_CLEAR_MODFFLAG(hspi);
        }

        /* SPI Frame error interrupt occurred ------------------------------------*/
        if ((itflag & SPI_FLAG_FRE) != RESET)
        {
            SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FRE);
            __HAL_SPI_CLEAR_FREFLAG(hspi);
        }
#endif /* 0 */

        /*SPI error occurred, STOP it.*/
        if (hspi->ErrorCode != HAL_SPI_ERROR_NONE)
        {
            /* Disable all interrupts */
            __HAL_SPI_DISABLE_IT(hspi, SPI_IT_RXNE | SPI_IT_TXE | SPI_IT_ERR);

            hspi->State = HAL_SPI_STATE_READY;

            SPI_PRINTF("SPI%d, FIFO_CTRL=%x \r\n", SPI_GetIdx(hspi), hspi->Instance->FIFO_CTRL);

            /* Disable the SPI DMA requests if enabled */
            if ((HAL_IS_BIT_SET(hspi->Instance->FIFO_CTRL, SPI_FIFO_CTRL_TSRE)) || (HAL_IS_BIT_SET(hspi->Instance->FIFO_CTRL, SPI_FIFO_CTRL_RSRE)))
            {
                CLEAR_BIT(hspi->Instance->FIFO_CTRL, (SPI_FIFO_CTRL_TSRE | SPI_FIFO_CTRL_RSRE));

                /* Abort the SPI DMA Rx channel */
                if (hspi->hdmarx != NULL)
                {
                    /* Set the SPI DMA Abort callback :
                    will lead to call HAL_SPI_ErrorCallback() at end of DMA abort procedure */
                    hspi->hdmarx->XferAbortCallback = SPI_DMAAbortOnError;
                    HAL_DMA_Abort_IT(hspi->hdmarx);
                }
                /* Abort the SPI DMA Tx channel */
                if (hspi->hdmatx != NULL)
                {
                    /* Set the SPI DMA Abort callback :
                    will lead to call HAL_SPI_ErrorCallback() at end of DMA abort procedure */
                    hspi->hdmatx->XferAbortCallback = SPI_DMAAbortOnError;
                    HAL_DMA_Abort_IT(hspi->hdmatx);
                }
            }
            else
            {
                /* Call user error callback */
                HAL_SPI_ErrorCallback(hspi);
            }
        }


        return;
    }

}

/**
  * @brief Tx Transfer completed callback.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
__weak void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hspi);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_SPI_TxCpltCallback should be implemented in the user file
     */
}

/**
  * @brief Rx Transfer completed callback.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
__weak void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hspi);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_SPI_RxCpltCallback should be implemented in the user file
     */
}

/**
  * @brief Tx and Rx Transfer completed callback.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
__weak void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hspi);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_SPI_TxRxCpltCallback should be implemented in the user file
     */
}

/**
  * @brief Tx Half Transfer completed callback.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
__weak void HAL_SPI_TxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hspi);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_SPI_TxHalfCpltCallback should be implemented in the user file
     */
}

/**
  * @brief Rx Half Transfer completed callback.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
__weak void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hspi);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_SPI_RxHalfCpltCallback() should be implemented in the user file
     */
}

/**
  * @brief Tx and Rx Half Transfer callback.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
__weak void HAL_SPI_TxRxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hspi);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_SPI_TxRxHalfCpltCallback() should be implemented in the user file
     */
}

/**
  * @brief SPI error callback.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
__weak void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hspi);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_SPI_ErrorCallback should be implemented in the user file
     */
    /* NOTE : The ErrorCode parameter in the hspi handle is updated by the SPI processes
              and user can use HAL_SPI_GetError() API to check the latest error occurred
     */
}

/**
  * @brief  SPI Abort Complete callback.
  * @param  hspi SPI handle.
  * @retval None
  */
__weak void HAL_SPI_AbortCpltCallback(SPI_HandleTypeDef *hspi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hspi);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_SPI_AbortCpltCallback can be implemented in the user file.
     */
}

/**
  * @}
  */

/** @defgroup SPI_Exported_Functions_Group3 Peripheral State and Errors functions
  * @brief   SPI control functions
  *
@verbatim
 ===============================================================================
                      ##### Peripheral State and Errors functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to control the SPI.
     (+) HAL_SPI_GetState() API can be helpful to check in run-time the state of the SPI peripheral
     (+) HAL_SPI_GetError() check in run-time Errors occurring during communication
@endverbatim
  * @{
  */

/**
  * @brief  Return the SPI handle state.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval SPI state
  */
__HAL_ROM_USED HAL_SPI_StateTypeDef HAL_SPI_GetState(SPI_HandleTypeDef *hspi)
{
    /* Return SPI handle state */
    return hspi->State;
}

/**
  * @brief  Return the SPI error code.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval SPI error code in bitmap format
  */
__HAL_ROM_USED uint32_t HAL_SPI_GetError(SPI_HandleTypeDef *hspi)
{
    /* Return SPI ErrorCode */
    return hspi->ErrorCode;
}

/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup SPI_Private_Functions
  * @brief   Private functions
  * @{
  */

/**
  * @brief DMA SPI transmit process complete callback.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void SPI_DMATransmitCplt(DMA_HandleTypeDef *hdma)
{
    SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
    uint32_t tickstart = 0U;

    /* Init tickstart for timeout managment*/
    tickstart = HAL_GetTick();

    /* DMA Normal Mode */
    if ((hdma->Instance->CCR & DMA_CCR_CIRC) != DMA_CCR_CIRC)
    {
        /* Disable ERR interrupt */
        __HAL_SPI_DISABLE_IT(hspi, SPI_IT_ERR);

        /* Check the end of the transaction */
        if (SPI_EndRxTxTransaction(hspi, SPI_DEFAULT_TIMEOUT, tickstart) != HAL_OK)
        {
            SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
        }

        /* Clear overrun flag in 2 Lines communication mode because received data is not read */
        if (hspi->Init.Direction == SPI_DIRECTION_2LINES)
        {
            __HAL_SPI_CLEAR_OVRFLAG(hspi);
        }

        hspi->TxXferCount = 0U;
        hspi->State = HAL_SPI_STATE_READY;

        if (hspi->ErrorCode != HAL_SPI_ERROR_NONE)
        {
            HAL_SPI_ErrorCallback(hspi);
            return;
        }
    }
    HAL_SPI_TxCpltCallback(hspi);
}

/**
  * @brief DMA SPI receive process complete callback.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void SPI_DMAReceiveCplt(DMA_HandleTypeDef *hdma)
{
    SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
    uint32_t tickstart = 0U;
#if (USE_SPI_CRC != 0U)
    __IO uint16_t tmpreg = 0U;
#endif /* USE_SPI_CRC */

    /* Init tickstart for timeout management*/
    tickstart = HAL_GetTick();

    /* DMA Normal Mode */
    if ((hdma->Instance->CCR & DMA_CCR_CIRC) != DMA_CCR_CIRC)
    {
        /* Disable ERR interrupt */
        __HAL_SPI_DISABLE_IT(hspi, SPI_IT_ERR);

#if (USE_SPI_CRC != 0U)
        /* CRC handling */
        if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
        {
            /* Wait until RXNE flag */
            if (SPI_WaitFlagStateUntilTimeout(hspi, SPI_FLAG_RXNE, SPI_FLAG_RXNE, SPI_DEFAULT_TIMEOUT, tickstart) != HAL_OK)
            {
                /* Error on the CRC reception */
                SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_CRC);
            }
            /* Read CRC */
            if (hspi->Init.DataSize > SPI_DATASIZE_8BIT)
            {
                tmpreg = hspi->Instance->DATA;
                /* To avoid GCC warning */
                UNUSED(tmpreg);
            }
            else
            {
                tmpreg = *(__IO uint8_t *)&hspi->Instance->DATA;
                /* To avoid GCC warning */
                UNUSED(tmpreg);

                if (hspi->Init.CRCLength == SPI_CRC_LENGTH_16BIT)
                {
                    if (SPI_WaitFlagStateUntilTimeout(hspi, SPI_FLAG_RXNE, SPI_FLAG_RXNE, SPI_DEFAULT_TIMEOUT, tickstart) != HAL_OK)
                    {
                        /* Error on the CRC reception */
                        SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_CRC);
                    }
                    tmpreg = *(__IO uint8_t *)&hspi->Instance->DATA;
                    /* To avoid GCC warning */
                    UNUSED(tmpreg);
                }
            }
        }
#endif /* USE_SPI_CRC */


        /* Check the end of the transaction */
        if (SPI_EndRxTransaction(hspi, SPI_DEFAULT_TIMEOUT, tickstart) != HAL_OK)
        {
            hspi->ErrorCode = HAL_SPI_ERROR_FLAG;
        }

        hspi->RxXferCount = 0U;
        hspi->State = HAL_SPI_STATE_READY;

#if (USE_SPI_CRC != 0U)
        /* Check if CRC error occurred */
        if (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_CRCERR))
        {
            SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_CRC);
            __HAL_SPI_CLEAR_CRCERRFLAG(hspi);
        }
#endif /* USE_SPI_CRC */

        if (hspi->ErrorCode != HAL_SPI_ERROR_NONE)
        {
            HAL_SPI_ErrorCallback(hspi);
            return;
        }
    }
    HAL_SPI_RxCpltCallback(hspi);
}

/**
  * @brief  DMA SPI transmit receive process complete callback.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void SPI_DMATransmitReceiveCplt(DMA_HandleTypeDef *hdma)
{
    SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
    uint32_t tickstart = 0U;
#if (USE_SPI_CRC != 0U)
    __IO int16_t tmpreg = 0U;
#endif /* USE_SPI_CRC */
    /* Init tickstart for timeout management*/
    tickstart = HAL_GetTick();

    /* DMA Normal Mode */
    if ((hdma->Instance->CCR & DMA_CCR_CIRC) != DMA_CCR_CIRC)
    {
        /* Disable ERR interrupt */
        __HAL_SPI_DISABLE_IT(hspi, SPI_IT_ERR);

#if (USE_SPI_CRC != 0U)
        /* CRC handling */
        if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
        {
            if ((hspi->Init.DataSize == SPI_DATASIZE_8BIT) && (hspi->Init.CRCLength == SPI_CRC_LENGTH_8BIT))
            {
                if (SPI_WaitFifoStateUntilTimeout(hspi, SPI_FLAG_FRLVL, SPI_FRLVL_QUARTER_FULL, SPI_DEFAULT_TIMEOUT,
                                                  tickstart) != HAL_OK)
                {
                    /* Error on the CRC reception */
                    SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_CRC);
                }
                /* Read CRC to Flush DR and RXNE flag */
                tmpreg = *(__IO uint8_t *)&hspi->Instance->DATA;
                /* To avoid GCC warning */
                UNUSED(tmpreg);
            }
            else
            {
                if (SPI_WaitFifoStateUntilTimeout(hspi, SPI_FLAG_FRLVL, SPI_FRLVL_HALF_FULL, SPI_DEFAULT_TIMEOUT, tickstart) != HAL_OK)
                {
                    /* Error on the CRC reception */
                    SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_CRC);
                }
                /* Read CRC to Flush DR and RXNE flag */
                tmpreg = hspi->Instance->DATA;
                /* To avoid GCC warning */
                UNUSED(tmpreg);
            }
        }
#endif /* USE_SPI_CRC */

        /* Check the end of the transaction */
        if (SPI_EndRxTxTransaction(hspi, SPI_DEFAULT_TIMEOUT, tickstart) != HAL_OK)
        {
            SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
        }

        hspi->TxXferCount = 0U;
        hspi->RxXferCount = 0U;
        hspi->State = HAL_SPI_STATE_READY;

#if (USE_SPI_CRC != 0U)
        /* Check if CRC error occurred */
        if (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_CRCERR))
        {
            SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_CRC);
            __HAL_SPI_CLEAR_CRCERRFLAG(hspi);
        }
#endif /* USE_SPI_CRC */

        if (hspi->ErrorCode != HAL_SPI_ERROR_NONE)
        {
            HAL_SPI_ErrorCallback(hspi);
            return;
        }
    }
    HAL_SPI_TxRxCpltCallback(hspi);
}

/**
  * @brief  DMA SPI half transmit process complete callback.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void SPI_DMAHalfTransmitCplt(DMA_HandleTypeDef *hdma)
{
    SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

    HAL_SPI_TxHalfCpltCallback(hspi);
}

/**
  * @brief  DMA SPI half receive process complete callback
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void SPI_DMAHalfReceiveCplt(DMA_HandleTypeDef *hdma)
{
    SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

    HAL_SPI_RxHalfCpltCallback(hspi);
}

/**
  * @brief  DMA SPI half transmit receive process complete callback.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void SPI_DMAHalfTransmitReceiveCplt(DMA_HandleTypeDef *hdma)
{
    SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

    HAL_SPI_TxRxHalfCpltCallback(hspi);
}

/**
  * @brief  DMA SPI communication error callback.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void SPI_DMAError(DMA_HandleTypeDef *hdma)
{
    SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

    SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_DMA);
    hspi->State = HAL_SPI_STATE_READY;
    HAL_SPI_ErrorCallback(hspi);
}

/**
  * @brief  DMA SPI communication abort callback, when initiated by HAL services on Error
  *         (To be called at end of DMA Abort procedure following error occurrence).
  * @param  hdma DMA handle.
  * @retval None
  */
static void SPI_DMAAbortOnError(DMA_HandleTypeDef *hdma)
{
    SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
    hspi->RxXferCount = 0U;
    hspi->TxXferCount = 0U;

    HAL_SPI_ErrorCallback(hspi);
}

#if 0
/**
  * @brief  DMA SPI Tx communication abort callback, when initiated by user
  *         (To be called at end of DMA Tx Abort procedure following user abort request).
  * @note   When this callback is executed, User Abort complete call back is called only if no
  *         Abort still ongoing for Rx DMA Handle.
  * @param  hdma DMA handle.
  * @retval None
  */
static void SPI_DMATxAbortCallback(DMA_HandleTypeDef *hdma)
{
    SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

    hspi->hdmatx->XferAbortCallback = NULL;

    if (SPI_EndRxTxTransaction(hspi, SPI_DEFAULT_TIMEOUT, HAL_GetTick()) != HAL_OK)
    {
        hspi->ErrorCode = HAL_SPI_ERROR_ABORT;
    }

    /* Disable SPI Peripheral */
    __HAL_SPI_DISABLE(hspi);

    /* Empty the FRLVL fifo */
    if (SPI_WaitFifoStateUntilTimeout(hspi, SPI_FLAG_FRLVL, SPI_FRLVL_EMPTY, SPI_DEFAULT_TIMEOUT, HAL_GetTick()) != HAL_OK)
    {
        hspi->ErrorCode = HAL_SPI_ERROR_ABORT;
    }

    /* Check if an Abort process is still ongoing */
    if (hspi->hdmarx != NULL)
    {
        if (hspi->hdmarx->XferAbortCallback != NULL)
        {
            return;
        }
    }

    /* No Abort process still ongoing : All DMA Stream/Channel are aborted, call user Abort Complete callback */
    hspi->RxXferCount = 0U;
    hspi->TxXferCount = 0U;

    /* Check no error during Abort procedure */
    if (hspi->ErrorCode != HAL_SPI_ERROR_ABORT)
    {
        /* Reset errorCode */
        hspi->ErrorCode = HAL_SPI_ERROR_NONE;
    }

    /* Clear the Error flags in the SR register */
    __HAL_SPI_CLEAR_OVRFLAG(hspi);
    __HAL_SPI_CLEAR_FREFLAG(hspi);

    /* Restore hspi->State to Ready */
    hspi->State  = HAL_SPI_STATE_READY;

    /* Call user Abort complete callback */
    HAL_SPI_AbortCpltCallback(hspi);
}

/**
  * @brief  DMA SPI Rx communication abort callback, when initiated by user
  *         (To be called at end of DMA Rx Abort procedure following user abort request).
  * @note   When this callback is executed, User Abort complete call back is called only if no
  *         Abort still ongoing for Tx DMA Handle.
  * @param  hdma DMA handle.
  * @retval None
  */
static void SPI_DMARxAbortCallback(DMA_HandleTypeDef *hdma)
{
    SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

    /* Disable SPI Peripheral */
    __HAL_SPI_DISABLE(hspi);

    hspi->hdmarx->XferAbortCallback = NULL;

    /* Control the BSY flag */
    if (SPI_WaitFlagStateUntilTimeout(hspi, SPI_FLAG_BSY, RESET, SPI_DEFAULT_TIMEOUT, HAL_GetTick()) != HAL_OK)
    {
        hspi->ErrorCode = HAL_SPI_ERROR_ABORT;
    }

    /* Empty the FRLVL fifo */
    if (SPI_WaitFifoStateUntilTimeout(hspi, SPI_FLAG_FRLVL, SPI_FRLVL_EMPTY, SPI_DEFAULT_TIMEOUT, HAL_GetTick()) != HAL_OK)
    {
        hspi->ErrorCode = HAL_SPI_ERROR_ABORT;
    }

    /* Check if an Abort process is still ongoing */
    if (hspi->hdmatx != NULL)
    {
        if (hspi->hdmatx->XferAbortCallback != NULL)
        {
            return;
        }
    }

    /* No Abort process still ongoing : All DMA Stream/Channel are aborted, call user Abort Complete callback */
    hspi->RxXferCount = 0U;
    hspi->TxXferCount = 0U;

    /* Check no error during Abort procedure */
    if (hspi->ErrorCode != HAL_SPI_ERROR_ABORT)
    {
        /* Reset errorCode */
        hspi->ErrorCode = HAL_SPI_ERROR_NONE;
    }

    /* Clear the Error flags in the SR register */
    __HAL_SPI_CLEAR_OVRFLAG(hspi);
    __HAL_SPI_CLEAR_FREFLAG(hspi);

    /* Restore hspi->State to Ready */
    hspi->State  = HAL_SPI_STATE_READY;

    /* Call user Abort complete callback */
    HAL_SPI_AbortCpltCallback(hspi);
}
#endif

/**
  * @brief  Rx 8-bit handler for Transmit and Receive in Interrupt mode.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_2linesRxISR_8BIT(struct __SPI_HandleTypeDef *hspi)
{
#if 0
    /* Receive data in packing mode */
    if (hspi->RxXferCount > 1U)
    {
        *((uint16_t *)hspi->pRxBuffPtr) = hspi->Instance->DATA;
        hspi->pRxBuffPtr += sizeof(uint16_t);
        hspi->RxXferCount -= 2U;
        if (hspi->RxXferCount == 1U)
        {
        }
    }
    /* Receive data in 8 Bit mode */
    else
#endif
    {
        *hspi->pRxBuffPtr++ = *((__IO uint8_t *)&hspi->Instance->DATA);
        hspi->RxXferCount--;
    }

    /* check end of the reception */
    if (hspi->RxXferCount == 0U)
    {
#if (USE_SPI_CRC != 0U)
        if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
        {
            SET_BIT(hspi->Instance->CR2, SPI_RXFIFO_THRESHOLD);
            hspi->RxISR =  SPI_2linesRxISR_8BITCRC;
            return;
        }
#endif /* USE_SPI_CRC */

        /* Disable RXNE  and ERR interrupt */
        __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_RXNE | SPI_IT_ERR));

        if (hspi->TxXferCount == 0U)
        {
            SPI_CloseRxTx_ISR(hspi);
        }
    }
}

#if (USE_SPI_CRC != 0U)
/**
  * @brief  Rx 8-bit handler for Transmit and Receive in Interrupt mode.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_2linesRxISR_8BITCRC(struct __SPI_HandleTypeDef *hspi)
{
    __IO uint8_t tmpreg = 0U;

    /* Read data register to flush CRC */
    tmpreg = *((__IO uint8_t *)&hspi->Instance->DATA);

    /* To avoid GCC warning */
    UNUSED(tmpreg);

    hspi->CRCSize--;

    /* check end of the reception */
    if (hspi->CRCSize == 0U)
    {
        /* Disable RXNE and ERR interrupt */
        __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_RXNE | SPI_IT_ERR));

        if (hspi->TxXferCount == 0U)
        {
            SPI_CloseRxTx_ISR(hspi);
        }
    }
}
#endif /* USE_SPI_CRC */

/**
  * @brief  Tx 8-bit handler for Transmit and Receive in Interrupt mode.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_2linesTxISR_8BIT(struct __SPI_HandleTypeDef *hspi)
{
#if 0
    /* Transmit data in packing Bit mode */
    if (hspi->TxXferCount >= 2U)
    {
        hspi->Instance->DATA = *((uint16_t *)hspi->pTxBuffPtr);
        hspi->pTxBuffPtr += sizeof(uint16_t);
        hspi->TxXferCount -= 2U;
    }
    /* Transmit data in 8 Bit mode */
    else
#endif
    {
        *(__IO uint8_t *)&hspi->Instance->DATA = (*hspi->pTxBuffPtr++);
        hspi->TxXferCount--;
    }

    /* check the end of the transmission */
    if (hspi->TxXferCount == 0U)
    {
#if (USE_SPI_CRC != 0U)
        if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
        {
            /* Set CRC Next Bit to send CRC */
            SET_BIT(hspi->Instance->CR1, SPI_CR1_CRCNEXT);
            /* Disable TXE interrupt */
            __HAL_SPI_DISABLE_IT(hspi, SPI_IT_TXE);
            return;
        }
#endif /* USE_SPI_CRC */

        /* Disable TXE interrupt */
        __HAL_SPI_DISABLE_IT(hspi, SPI_IT_TXE);

        if (hspi->RxXferCount == 0U)
        {
            SPI_CloseRxTx_ISR(hspi);
        }
    }
}

/**
  * @brief  Rx 16-bit handler for Transmit and Receive in Interrupt mode.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_2linesRxISR_16BIT(struct __SPI_HandleTypeDef *hspi)
{
    /* Receive data in 16 Bit mode */
    *((uint16_t *)hspi->pRxBuffPtr) = hspi->Instance->DATA;
    hspi->pRxBuffPtr += sizeof(uint16_t);
    hspi->RxXferCount--;

    if (hspi->RxXferCount == 0U)
    {
#if (USE_SPI_CRC != 0U)
        if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
        {
            hspi->RxISR =  SPI_2linesRxISR_16BITCRC;
            return;
        }
#endif /* USE_SPI_CRC */

        /* Disable RXNE interrupt */
        __HAL_SPI_DISABLE_IT(hspi, SPI_IT_RXNE);

        if (hspi->TxXferCount == 0U)
        {
            SPI_CloseRxTx_ISR(hspi);
        }
    }
}

#if (USE_SPI_CRC != 0U)
/**
  * @brief  Manage the CRC 16-bit receive for Transmit and Receive in Interrupt mode.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_2linesRxISR_16BITCRC(struct __SPI_HandleTypeDef *hspi)
{
    /* Receive data in 16 Bit mode */
    __IO uint16_t tmpreg = 0U;

    /* Read data register to flush CRC */
    tmpreg = hspi->Instance->DATA;

    /* To avoid GCC warning */
    UNUSED(tmpreg);

    /* Disable RXNE interrupt */
    __HAL_SPI_DISABLE_IT(hspi, SPI_IT_RXNE);

    SPI_CloseRxTx_ISR(hspi);
}
#endif /* USE_SPI_CRC */

/**
  * @brief  Tx 16-bit handler for Transmit and Receive in Interrupt mode.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_2linesTxISR_16BIT(struct __SPI_HandleTypeDef *hspi)
{
    /* Transmit data in 16 Bit mode */
    hspi->Instance->DATA = *((uint16_t *)hspi->pTxBuffPtr);
    hspi->pTxBuffPtr += sizeof(uint16_t);
    hspi->TxXferCount--;

    /* Enable CRC Transmission */
    if (hspi->TxXferCount == 0U)
    {
#if (USE_SPI_CRC != 0U)
        if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
        {
            /* Set CRC Next Bit to send CRC */
            SET_BIT(hspi->Instance->CR1, SPI_CR1_CRCNEXT);
            /* Disable TXE interrupt */
            __HAL_SPI_DISABLE_IT(hspi, SPI_IT_TXE);
            return;
        }
#endif /* USE_SPI_CRC */

        /* Disable TXE interrupt */
        __HAL_SPI_DISABLE_IT(hspi, SPI_IT_TXE);

        while (hspi->RxXferCount > 0U)
        {
            /* Check RXNE flag */
            if (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE))
            {
                *((uint16_t *)hspi->pRxBuffPtr) = hspi->Instance->DATA;
                hspi->pRxBuffPtr += sizeof(uint16_t);
                hspi->RxXferCount--;
            }
        }

        if (hspi->RxXferCount == 0U)
        {
            __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_RXNE | SPI_IT_ERR));
            SPI_CloseRxTx_ISR(hspi);
        }
    }
}

#if (USE_SPI_CRC != 0U)
/**
  * @brief  Manage the CRC 8-bit receive in Interrupt context.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_RxISR_8BITCRC(struct __SPI_HandleTypeDef *hspi)
{
    __IO uint8_t tmpreg = 0U;

    /* Read data register to flush CRC */
    tmpreg = *((__IO uint8_t *)&hspi->Instance->DATA);

    /* To avoid GCC warning */
    UNUSED(tmpreg);

    hspi->CRCSize--;

    if (hspi->CRCSize == 0U)
    {
        SPI_CloseRx_ISR(hspi);
    }
}
#endif /* USE_SPI_CRC */

/**
  * @brief  Manage the receive 8-bit in Interrupt context.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_RxISR_8BIT(struct __SPI_HandleTypeDef *hspi)
{
    *hspi->pRxBuffPtr++ = (*(__IO uint8_t *)&hspi->Instance->DATA);
    hspi->RxXferCount--;

#if (USE_SPI_CRC != 0U)
    /* Enable CRC Transmission */
    if ((hspi->RxXferCount == 1U) && (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE))
    {
        SET_BIT(hspi->Instance->CR1, SPI_CR1_CRCNEXT);
    }
#endif /* USE_SPI_CRC */

#if 0 //We will be trapped here if Rx fifo overflowed in slave mode.
    if (hspi->RxXferCount <= 16)
    {
        /* Transfer loop */
        while (hspi->RxXferCount > 0U)
        {
            /* Check the RXNE flag */
            if (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE))
            {
                /* read the received data */
                *hspi->pRxBuffPtr++ = *(__IO uint8_t *)&hspi->Instance->DATA;
                hspi->RxXferCount--;
            }
        }
    }
#endif /* 0 */

    if (hspi->RxXferCount == 0U)
    {
#if (USE_SPI_CRC != 0U)
        if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
        {
            hspi->RxISR =  SPI_RxISR_8BITCRC;
            return;
        }
#endif /* USE_SPI_CRC */
        SPI_CloseRx_ISR(hspi);
    }
}

#if (USE_SPI_CRC != 0U)
/**
  * @brief  Manage the CRC 16-bit receive in Interrupt context.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_RxISR_16BITCRC(struct __SPI_HandleTypeDef *hspi)
{
    __IO uint16_t tmpreg = 0U;

    /* Read data register to flush CRC */
    tmpreg = hspi->Instance->DATA;

    /* To avoid GCC warning */
    UNUSED(tmpreg);

    /* Disable RXNE and ERR interrupt */
    __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_RXNE | SPI_IT_ERR));

    SPI_CloseRx_ISR(hspi);
}
#endif /* USE_SPI_CRC */

/**
  * @brief  Manage the 16-bit receive in Interrupt context.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_RxISR_16BIT(struct __SPI_HandleTypeDef *hspi)
{
    *((uint16_t *)hspi->pRxBuffPtr) = hspi->Instance->DATA;
    hspi->pRxBuffPtr += sizeof(uint16_t);
    hspi->RxXferCount--;

#if (USE_SPI_CRC != 0U)
    /* Enable CRC Transmission */
    if ((hspi->RxXferCount == 1U) && (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE))
    {
        SET_BIT(hspi->Instance->CR1, SPI_CR1_CRCNEXT);
    }
#endif /* USE_SPI_CRC */

#if 0 //We will be trapped here if Rx fifo overflowed in slave mode.
    if (hspi->RxXferCount <= 8)
    {
        /* Transfer loop */
        while (hspi->RxXferCount > 0U)
        {
            /* Check the RXNE flag */
            if (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE))
            {
                *((uint16_t *)hspi->pRxBuffPtr) = hspi->Instance->DATA;
                hspi->pRxBuffPtr += sizeof(uint16_t);
                hspi->RxXferCount--;
            }
        }
    }
#endif

    if (hspi->RxXferCount == 0U)
    {
#if (USE_SPI_CRC != 0U)
        if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
        {
            hspi->RxISR = SPI_RxISR_16BITCRC;
            return;
        }
#endif /* USE_SPI_CRC */
        SPI_CloseRx_ISR(hspi);
    }
}

/**
  * @brief  Handle the data 8-bit transmit in Interrupt mode.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_TxISR_8BIT(struct __SPI_HandleTypeDef *hspi)
{
    *(__IO uint8_t *)&hspi->Instance->DATA = (*hspi->pTxBuffPtr++);
    hspi->TxXferCount--;

    if (hspi->TxXferCount == 0U)
    {
#if (USE_SPI_CRC != 0U)
        if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
        {
            /* Enable CRC Transmission */
            SET_BIT(hspi->Instance->CR1, SPI_CR1_CRCNEXT);
        }
#endif /* USE_SPI_CRC */
        SPI_CloseTx_ISR(hspi);
    }
}

/**
  * @brief  Handle the data 16-bit transmit in Interrupt mode.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_TxISR_16BIT(struct __SPI_HandleTypeDef *hspi)
{
    /* Transmit data in 16 Bit mode */
    hspi->Instance->DATA = *((uint16_t *)hspi->pTxBuffPtr);
    hspi->pTxBuffPtr += sizeof(uint16_t);
    hspi->TxXferCount--;

    if (hspi->TxXferCount == 0U)
    {
#if (USE_SPI_CRC != 0U)
        if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
        {
            /* Enable CRC Transmission */
            SET_BIT(hspi->Instance->CR1, SPI_CR1_CRCNEXT);
        }
#endif /* USE_SPI_CRC */
        SPI_CloseTx_ISR(hspi);
    }
}

/**
  * @brief Handle SPI Communication Timeout.
  * @param hspi pointer to a SPI_HandleTypeDef structure that contains
  *              the configuration information for SPI module.
  * @param Flag SPI flag to check
  * @param State flag state to check
  * @param Timeout Timeout duration
  * @param Tickstart tick start value
  * @retval HAL status
  */
static HAL_StatusTypeDef SPI_WaitFlagStateUntilTimeout(SPI_HandleTypeDef *hspi, uint32_t Flag, uint32_t State,
        uint32_t Timeout, uint32_t Tickstart)
{
    while ((__HAL_SPI_GET_FLAG(hspi, Flag) ? SET : RESET) != State)
    {
        if (Timeout != HAL_MAX_DELAY)
        {
            if ((Timeout == 0U) || ((HAL_GetTick() - Tickstart) >= Timeout))
            {
                /* Disable the SPI and reset the CRC: the CRC value should be cleared
                on both master and slave sides in order to resynchronize the master
                and slave for their respective CRC calculation */

                /* Disable TXE, RXNE and ERR interrupts for the interrupt process */
                __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));

                if ((hspi->Init.Mode == SPI_MODE_MASTER) && ((hspi->Init.Direction == SPI_DIRECTION_1LINE)
                        || (hspi->Init.Direction == SPI_DIRECTION_2LINES_RXONLY)))
                {
                    /* Disable SPI peripheral */
                    __HAL_SPI_DISABLE(hspi);
                }


                hspi->State = HAL_SPI_STATE_READY;

                /* Process Unlocked */
                __HAL_UNLOCK(hspi);

                return HAL_TIMEOUT;
            }
        }
    }

    return HAL_OK;
}

/**
  * @brief Handle SPI FIFO Communication Timeout.
  * @param hspi pointer to a SPI_HandleTypeDef structure that contains
  *              the configuration information for SPI module.
  * @param Fifo Fifo to check
  * @param State Fifo state to check
  * @param Timeout Timeout duration
  * @param Tickstart tick start value
  * @retval HAL status
  */
static HAL_StatusTypeDef SPI_WaitFifoStateUntilTimeout(SPI_HandleTypeDef *hspi, uint32_t Fifo, uint32_t State,
        uint32_t Timeout, uint32_t Tickstart)
{
    __IO uint8_t tmpreg;

    while ((hspi->Instance->STATUS & Fifo) != State)
    {
        if ((Fifo == SPI_FLAG_FRLVL) && (State == SPI_FRLVL_EMPTY))
        {
            tmpreg = *((__IO uint8_t *)&hspi->Instance->DATA);
            /* To avoid GCC warning */
            UNUSED(tmpreg);
        }

        if (Timeout != HAL_MAX_DELAY)
        {
            if ((Timeout == 0U) || ((HAL_GetTick() - Tickstart) >= Timeout))
            {
                /* Disable the SPI and reset the CRC: the CRC value should be cleared
                   on both master and slave sides in order to resynchronize the master
                   and slave for their respective CRC calculation */

                /* Disable TXE, RXNE and ERR interrupts for the interrupt process */
                __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));

                if ((hspi->Init.Mode == SPI_MODE_MASTER) && ((hspi->Init.Direction == SPI_DIRECTION_1LINE)
                        || (hspi->Init.Direction == SPI_DIRECTION_2LINES_RXONLY)))
                {
                    /* Disable SPI peripheral */
                    __HAL_SPI_DISABLE(hspi);
                }


                hspi->State = HAL_SPI_STATE_READY;

                /* Process Unlocked */
                __HAL_UNLOCK(hspi);

                return HAL_TIMEOUT;
            }
        }
    }

    return HAL_OK;
}

/**
  * @brief  Handle the check of the RX transaction complete.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  Timeout Timeout duration
  * @param  Tickstart tick start value
  * @retval HAL status
  */
static HAL_StatusTypeDef SPI_EndRxTransaction(SPI_HandleTypeDef *hspi,  uint32_t Timeout, uint32_t Tickstart)
{
    SPI_PRINTF("SPI_EndRxTransaction %x \r\n", hspi);

    if ((hspi->Init.Mode == SPI_MODE_MASTER) && ((hspi->Init.Direction == SPI_DIRECTION_1LINE)
            || (hspi->Init.Direction == SPI_DIRECTION_2LINES_RXONLY)))
    {
        /* Disable SPI peripheral */
        __HAL_SPI_DISABLE(hspi);
    }

    /* Control the BSY flag */
    if (SPI_WaitFlagStateUntilTimeout(hspi, SPI_FLAG_BSY, RESET, Timeout, Tickstart) != HAL_OK)
    {
        SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
        return HAL_TIMEOUT;
    }

    if ((hspi->Init.Mode == SPI_MODE_MASTER) && ((hspi->Init.Direction == SPI_DIRECTION_1LINE)
            || (hspi->Init.Direction == SPI_DIRECTION_2LINES_RXONLY)))
    {
        /* Empty the FRLVL fifo */
        if (SPI_WaitFifoStateUntilTimeout(hspi, SPI_FLAG_FRLVL, SPI_FRLVL_EMPTY, Timeout, Tickstart) != HAL_OK)
        {
            SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
            return HAL_TIMEOUT;
        }
    }
    return HAL_OK;
}

/**
  * @brief  Handle the check of the RXTX or TX transaction complete.
  * @param  hspi SPI handle
  * @param  Timeout Timeout duration
  * @param  Tickstart tick start value
  * @retval HAL status
  */
static HAL_StatusTypeDef SPI_EndRxTxTransaction(SPI_HandleTypeDef *hspi, uint32_t Timeout, uint32_t Tickstart)
{
    SPI_PRINTF("SPI_EndRxTxTransaction %x \r\n", hspi);

    /* Control if the TX fifo is empty */
    if (SPI_WaitFifoStateUntilTimeout(hspi, SPI_FLAG_FTLVL, SPI_FTLVL_EMPTY, Timeout, Tickstart) != HAL_OK)
    {
        SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
        return HAL_TIMEOUT;
    }

    /* Control the BSY flag */
    if (SPI_WaitFlagStateUntilTimeout(hspi, SPI_FLAG_BSY, RESET, Timeout, Tickstart) != HAL_OK)
    {
        SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
        return HAL_TIMEOUT;
    }

    /* Control if the RX fifo is empty */
    if (SPI_WaitFifoStateUntilTimeout(hspi, SPI_FLAG_FRLVL, SPI_FRLVL_EMPTY, Timeout, Tickstart) != HAL_OK)
    {
        SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
        return HAL_TIMEOUT;
    }
    return HAL_OK;
}

/**
  * @brief  Handle the end of the RXTX transaction.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_CloseRxTx_ISR(SPI_HandleTypeDef *hspi)
{
    uint32_t tickstart = 0U;

    SPI_PRINTF("SPI_CloseRxTx_ISR %x \r\n", hspi);

    /* Init tickstart for timeout managment*/
    tickstart = HAL_GetTick();

    /* Disable ERR interrupt */
    __HAL_SPI_DISABLE_IT(hspi, SPI_IT_ERR);

    /* Check the end of the transaction */
    if (SPI_EndRxTxTransaction(hspi, SPI_DEFAULT_TIMEOUT, tickstart) != HAL_OK)
    {
        SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
    }

#if (USE_SPI_CRC != 0U)
    /* Check if CRC error occurred */
    if (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_CRCERR) != RESET)
    {
        hspi->State = HAL_SPI_STATE_READY;
        SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_CRC);
        __HAL_SPI_CLEAR_CRCERRFLAG(hspi);
        HAL_SPI_ErrorCallback(hspi);
    }
    else
    {
#endif /* USE_SPI_CRC */
        if (hspi->ErrorCode == HAL_SPI_ERROR_NONE)
        {
            if (hspi->State == HAL_SPI_STATE_BUSY_RX)
            {
                hspi->State = HAL_SPI_STATE_READY;
                HAL_SPI_RxCpltCallback(hspi);
            }
            else
            {
                hspi->State = HAL_SPI_STATE_READY;
                HAL_SPI_TxRxCpltCallback(hspi);
            }
        }
        else
        {
            hspi->State = HAL_SPI_STATE_READY;
            HAL_SPI_ErrorCallback(hspi);
        }
#if (USE_SPI_CRC != 0U)
    }
#endif /* USE_SPI_CRC */
}

/**
  * @brief  Handle the end of the RX transaction.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_CloseRx_ISR(SPI_HandleTypeDef *hspi)
{
    SPI_PRINTF("SPI_CloseRx_ISR %x \r\n", hspi);

    /* Disable RXNE and ERR interrupt */
    __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_RXNE | SPI_IT_ERR));

    /* Check the end of the transaction */
    if (SPI_EndRxTransaction(hspi, SPI_DEFAULT_TIMEOUT, HAL_GetTick()) != HAL_OK)
    {
        SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
    }
    hspi->State = HAL_SPI_STATE_READY;

#if (USE_SPI_CRC != 0U)
    /* Check if CRC error occurred */
    if (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_CRCERR) != RESET)
    {
        SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_CRC);
        __HAL_SPI_CLEAR_CRCERRFLAG(hspi);
        HAL_SPI_ErrorCallback(hspi);
    }
    else
    {
#endif /* USE_SPI_CRC */
        if (hspi->ErrorCode == HAL_SPI_ERROR_NONE)
        {
            HAL_SPI_RxCpltCallback(hspi);
        }
        else
        {
            HAL_SPI_ErrorCallback(hspi);
        }
#if (USE_SPI_CRC != 0U)
    }
#endif /* USE_SPI_CRC */
}

/**
  * @brief  Handle the end of the TX transaction.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_CloseTx_ISR(SPI_HandleTypeDef *hspi)
{
    uint32_t tickstart = 0U;
    SPI_PRINTF("SPI_CloseTx_ISR %x \r\n", hspi);

    /* Init tickstart for timeout management*/
    tickstart = HAL_GetTick();

    /* Disable TXE and ERR interrupt */
    __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_TXE | SPI_IT_ERR));

    /* Check the end of the transaction */
    if (SPI_EndRxTxTransaction(hspi, SPI_DEFAULT_TIMEOUT, tickstart) != HAL_OK)
    {
        SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
    }

    /* Clear overrun flag in 2 Lines communication mode because received is not read */
    if (hspi->Init.Direction == SPI_DIRECTION_2LINES)
    {
        __HAL_SPI_CLEAR_OVRFLAG(hspi);
    }

    hspi->State = HAL_SPI_STATE_READY;
    if (hspi->ErrorCode != HAL_SPI_ERROR_NONE)
    {
        HAL_SPI_ErrorCallback(hspi);
    }
    else
    {
        HAL_SPI_TxCpltCallback(hspi);
    }
}

/**
  * @brief  Handle abort a Rx transaction.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_AbortRx_ISR(SPI_HandleTypeDef *hspi)
{
    __IO uint32_t count;

    SPI_PRINTF("SPI_AbortRx_ISR %x \r\n", hspi);

    /* Disable SPI Peripheral */
    __HAL_SPI_DISABLE(hspi);

    count = SPI_DEFAULT_TIMEOUT * (HAL_RCC_GetPCLKFreq(hspi->core, 1) / 24U / 1000U);

    /* Disable TXEIE, RXNEIE and ERRIE(mode fault event, overrun error, TI frame error) interrupts */
    __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));

    /* Check RXNEIE is disabled */
    do
    {
        if (count-- == 0U)
        {
            SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_ABORT);
            break;
        }
    }
    while (__HAL_SPI_IS_ENABLE_IT(hspi, SPI_IT_RXNE));

    /* Control the BSY flag */
    if (SPI_WaitFlagStateUntilTimeout(hspi, SPI_FLAG_BSY, RESET, SPI_DEFAULT_TIMEOUT, HAL_GetTick()) != HAL_OK)
    {
        hspi->ErrorCode = HAL_SPI_ERROR_ABORT;
    }

    /* Empty the FRLVL fifo */
    if (SPI_WaitFifoStateUntilTimeout(hspi, SPI_FLAG_FRLVL, SPI_FRLVL_EMPTY, SPI_DEFAULT_TIMEOUT, HAL_GetTick()) != HAL_OK)
    {
        hspi->ErrorCode = HAL_SPI_ERROR_ABORT;
    }

    hspi->State = HAL_SPI_STATE_ABORT;
}

/**
  * @brief  Handle abort a Tx or Rx/Tx transaction.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_AbortTx_ISR(SPI_HandleTypeDef *hspi)
{
    __IO uint32_t count;

    SPI_PRINTF("SPI_AbortTx_ISR %x \r\n", hspi);

    count = SPI_DEFAULT_TIMEOUT * (HAL_RCC_GetPCLKFreq(hspi->core, 1) / 24U / 1000U);

    /* Disable TXEIE, RXNEIE and ERRIE(mode fault event, overrun error, TI frame error) interrupts */
    __HAL_SPI_DISABLE_IT(hspi, SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR);

    /* Check TXEIE is disabled */
    do
    {
        if (count-- == 0U)
        {
            SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_ABORT);
            break;
        }
    }
    while (__HAL_SPI_IS_ENABLE_IT(hspi, SPI_IT_TXE));

    if (SPI_EndRxTxTransaction(hspi, SPI_DEFAULT_TIMEOUT, HAL_GetTick()) != HAL_OK)
    {
        hspi->ErrorCode = HAL_SPI_ERROR_ABORT;
    }

    /* Disable SPI Peripheral */
    __HAL_SPI_DISABLE(hspi);

    /* Empty the FRLVL fifo */
    if (SPI_WaitFifoStateUntilTimeout(hspi, SPI_FLAG_FRLVL, SPI_FRLVL_EMPTY, SPI_DEFAULT_TIMEOUT, HAL_GetTick()) != HAL_OK)
    {
        hspi->ErrorCode = HAL_SPI_ERROR_ABORT;
    }

    hspi->State = HAL_SPI_STATE_ABORT;
}


static uint8_t SPI_GetIdx(SPI_HandleTypeDef *hspi)
{
    if (SPI1 == hspi->Instance) return 1;
    if (SPI2 == hspi->Instance) return 2;

#ifndef SF32LB52X
    if (SPI3 == hspi->Instance) return 3;
    if (SPI4 == hspi->Instance) return 4;
#endif

    return 0;
}

/**
  * @}
  */

#endif /* HAL_SPI_MODULE_ENABLED */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
