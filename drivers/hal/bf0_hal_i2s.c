/**
  ******************************************************************************
  * @file   bf0_hal_i2s.c
  * @author Sifli software development team
  * @brief   I2S HAL module driver.
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
/* Includes ------------------------------------------------------------------*/
#include "bf0_hal.h"

#if defined(HAL_I2S_MODULE_ENABLED)||defined(_SIFLI_DOXYGEN_)

/** @addtogroup I2S I2S
  * @brief I2S HAL module driver
  * @{
  */


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/** @addtogroup I2S_Private
  * @{
  */
static void               I2S_DMATxCplt(DMA_HandleTypeDef *hdma);
static void               I2S_DMATxHalfCplt(DMA_HandleTypeDef *hdma);
static void               I2S_DMARxCplt(DMA_HandleTypeDef *hdma);
static void               I2S_DMARxHalfCplt(DMA_HandleTypeDef *hdma);
static void               I2S_DMAError(DMA_HandleTypeDef *hdma);
static void               I2S_Transmit_IT(I2S_HandleTypeDef *hi2s);
static void               I2S_Receive_IT(I2S_HandleTypeDef *hi2s);
static HAL_StatusTypeDef  I2S_WaitFlagStateUntilTimeout(I2S_HandleTypeDef *hi2s, uint32_t Flag, uint32_t Status, uint32_t Timeout);
//static int                I2S_GetBCLKDiv(I2S_HandleTypeDef *hi2s, uint32_t sampleRate);

/**
  * @}
  */

/* Exported functions ---------------------------------------------------------*/
/** @addtogroup I2S_Exported_Functions I2S Exported Functions
  * @{
  */

/** @addtogroup  I2S_Exported_Functions_Group1
  *  @brief    Initialization and Configuration functions
  *
@verbatim
 ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]  This subsection provides a set of functions allowing to initialize and
          de-initialiaze the I2Sx peripheral in simplex mode:

      (+) User must Implement HAL_I2S_MspInit() function in which he configures
          all related peripherals resources (CLOCK, GPIO, DMA, IT and NVIC ).

      (+) Call the function HAL_I2S_Init() to configure the selected device with
          the selected configuration:
        (++) Mode
        (++) Standard
        (++) Data Format
        (++) MCLK Output
        (++) Audio frequency
        (++) Polarity

     (+) Call the function HAL_I2S_DeInit() to restore the default configuration
         of the selected I2Sx periperal.
  @endverbatim
  * @{
  */

/**
  * @brief Initializes the I2S according to the specified parameters
  *         in the I2S_InitTypeDef and create the associated handle.
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_I2S_Init(I2S_HandleTypeDef *hi2s)
{

    /* Check the I2S handle allocation */
    if (hi2s == NULL)
    {
        return HAL_ERROR;
    }

    if (hi2s->State == HAL_I2S_STATE_RESET)
    {
        /* Allocate lock resource and initialize it */
        hi2s->Lock = HAL_UNLOCKED;

        /* Init the low level hardware : GPIO, CLOCK, CORTEX...etc */
        HAL_I2S_MspInit(hi2s);
    }

    hi2s->State |= HAL_I2S_STATE_BUSY;

    // configure TX
    HAL_I2S_Config_Transmit(hi2s, &(hi2s->Init.tx_cfg));

    // configure RX
    HAL_I2S_Config_Receive(hi2s, &(hi2s->Init.rx_cfg));

    // BT/BB configure not support

    /* Disable+pause the I2S  */
    __HAL_I2S_TX_DISABLE(hi2s);
    __HAL_I2S_RX_DISABLE(hi2s);
    SET_BIT(hi2s->Instance->DMA_MASK, I2S_DMA_MASK_TX_DMA_MASK);
    SET_BIT(hi2s->Instance->DMA_MASK, I2S_DMA_MASK_RX_DMA_MASK);

    //SET_BIT(hi2s->Instance->AUDIO_RX_PAUSE, I2S_AUDIO_RX_PAUSE_RX_PAUSE);
    //SET_BIT(hi2s->Instance->AUDIO_TX_PAUSE, I2S_AUDIO_TX_PAUSE_TX_PAUSE);

    // configure TX DMA, instance and request should be initialized by caller
    if (hi2s->hdmatx != NULL)
    {
        //hi2s->hdmatx->Instance                 = I2S_TX_DMA_INSTANCE;
        //hi2s->hdmatx->Init.Request             = I2S_TX_DMA_REQUEST;
        hi2s->hdmatx->Init.Direction           = DMA_MEMORY_TO_PERIPH;
        hi2s->hdmatx->Init.PeriphInc           = DMA_PINC_DISABLE;
        hi2s->hdmatx->Init.MemInc              = DMA_MINC_ENABLE;
        hi2s->hdmatx->Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
        hi2s->hdmatx->Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
        hi2s->hdmatx->Init.Mode                = DMA_CIRCULAR; //DMA_CIRCULAR; //DMA_CIRCULAR; //DMA_NORMAL;
        hi2s->hdmatx->Init.Priority            = DMA_PRIORITY_HIGH;
        hi2s->hdmatx->Init.BurstSize = 0;
        hi2s->hdmatx->Parent = (void *)hi2s;

        HAL_DMA_Init(hi2s->hdmatx);
    }

    // configure RX DMA, instance and request should be initialized by caller
    if (hi2s->hdmarx != NULL)
    {
        //hi2s->hdmarx->Instance                 = MIC_DMA_INSTANCE;
        //hi2s->hdmarx->Init.Request             = MIC_DMA_REQUEST;
        hi2s->hdmarx->Init.Direction           = DMA_PERIPH_TO_MEMORY;
        hi2s->hdmarx->Init.PeriphInc           = DMA_PINC_DISABLE;
        hi2s->hdmarx->Init.MemInc              = DMA_MINC_ENABLE;
        hi2s->hdmarx->Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
        hi2s->hdmarx->Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
        hi2s->hdmarx->Init.Mode                = DMA_CIRCULAR;
        hi2s->hdmarx->Init.Priority            = DMA_PRIORITY_LOW;
        hi2s->hdmarx->Init.BurstSize = 0;
        hi2s->hdmarx->Parent = (void *)hi2s;

        HAL_DMA_Init(hi2s->hdmarx);
    }

    hi2s->ErrorCode = HAL_I2S_ERROR_NONE;
    hi2s->State = HAL_I2S_STATE_READY;

    return HAL_OK;
}

/**
  * @brief DeInitializes the I2S peripheral
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_I2S_DeInit(I2S_HandleTypeDef *hi2s)
{
    /* Check the I2S handle allocation */
    if (hi2s == NULL)
    {
        return HAL_ERROR;
    }

    hi2s->State |= HAL_I2S_STATE_BUSY;

    if (hi2s->hdmatx != NULL)
    {
        HAL_DMA_DeInit(hi2s->hdmatx);
    }
    if (hi2s->hdmarx != NULL)
    {
        HAL_DMA_DeInit(hi2s->hdmarx);
    }

    /* Disable the I2S Peripheral Clock */
    __HAL_I2S_TX_DISABLE(hi2s);
    __HAL_I2S_RX_DISABLE(hi2s);

    /* DeInit the low level hardware: GPIO, CLOCK, NVIC... */
    HAL_I2S_MspDeInit(hi2s);

    hi2s->ErrorCode = HAL_I2S_ERROR_NONE;
    hi2s->State = HAL_I2S_STATE_RESET;

    /* Release Lock */
    __HAL_UNLOCK(hi2s);

    return HAL_OK;
}

/**
  * @brief I2S MSP Init
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval None
  */
__weak void HAL_I2S_MspInit(I2S_HandleTypeDef *hi2s)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hi2s);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_I2S_MspInit could be implemented in the user file
     */
}

/**
  * @brief I2S MSP DeInit
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval None
  */
__weak void HAL_I2S_MspDeInit(I2S_HandleTypeDef *hi2s)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hi2s);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_I2S_MspDeInit could be implemented in the user file
     */
}

/**
  * @}
  */

/** @addtogroup I2S_Exported_Functions_Group2
  *  @brief Data transfers functions
  *
@verbatim
 ===============================================================================
                      ##### IO operation functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to manage the I2S data
    transfers.

    (#) There are two modes of transfer:
       (++) Blocking mode : The communication is performed in the polling mode.
            The status of all data processing is returned by the same function
            after finishing transfer.
       (++) No-Blocking mode : The communication is performed using Interrupts
            or DMA. These functions return the status of the transfer startup.
            The end of the data processing will be indicated through the
            dedicated I2S IRQ when using Interrupt mode or the DMA IRQ when
            using DMA mode.

    (#) Blocking mode functions are :
        (++) HAL_I2S_Transmit()
        (++) HAL_I2S_Receive()

    (#) No-Blocking mode functions with Interrupt are :
        (++) HAL_I2S_Transmit_IT()
        (++) HAL_I2S_Receive_IT()

    (#) No-Blocking mode functions with DMA are :
        (++) HAL_I2S_Transmit_DMA()
        (++) HAL_I2S_Receive_DMA()

    (#) A set of Transfer Complete Callbacks are provided in non Blocking mode:
        (++) HAL_I2S_TxCpltCallback()
        (++) HAL_I2S_RxCpltCallback()
        (++) HAL_I2S_ErrorCallback()

@endverbatim
  * @{
  */

__HAL_ROM_USED HAL_StatusTypeDef HAL_I2S_Config_Transmit(I2S_HandleTypeDef *hi2s, I2S_CFG_T *cfg)
{
    if ((hi2s == NULL) || (cfg == 0U))
    {
        return  HAL_ERROR;
    }

    hi2s->Instance->TX_PCM_SAMPLE_CLK = ((cfg->clk_div->lr_clk_duty_low + cfg->clk_div->lr_clk_duty_high) << I2S_TX_PCM_SAMPLE_CLK_FS_DUTY_Pos);
    hi2s->Instance->AUDIO_TX_BCLK_DIV = (cfg->clk_div->blck_duty << I2S_AUDIO_RX_BCLK_DIV_DUTY_Pos);
    hi2s->Instance->AUDIO_TX_LRCK_DIV = (cfg->clk_div->lr_clk_duty_low << I2S_AUDIO_TX_LRCK_DIV_DUTY_LOW_Pos) |
                                        (cfg->clk_div->lr_clk_duty_high << I2S_AUDIO_TX_LRCK_DIV_DUTY_HIGH_Pos);

    hi2s->Instance->AUDIO_SERIAL_TIMING = (0 << I2S_AUDIO_SERIAL_TIMING_TIMING_Pos) |
                                          ((cfg->slave_mode & 0x1) << I2S_AUDIO_SERIAL_TIMING_SLAVE_EN_Pos) |
                                          ((cfg->lrck_invert & 0x1) << I2S_AUDIO_SERIAL_TIMING_LRCK_POL_Pos);
    if (cfg->slave_mode)
        hi2s->Instance->AUDIO_I2S_SL_MERGE = 0x1 << I2S_AUDIO_I2S_SL_MERGE_SLAVE_TIMING_MERGE_Pos;
    hi2s->Instance->AUDIO_TX_FORMAT = (cfg->pcm_dw << I2S_AUDIO_TX_FORMAT_PCM_DATA_WIDTH_Pos);
    hi2s->Instance->TX_PCM_FORMAT =
        (((cfg->data_dw & 0x1f)) << I2S_TX_PCM_FORMAT_DW_Pos) |             // dw
        ((cfg->track & 0x1) << I2S_TX_PCM_FORMAT_TRACK_FLAG_Pos);              // Mono or storeo
    hi2s->Instance->TX_PCM_CH_SEL = cfg->chnl_sel;
    hi2s->Instance->TX_VOL_CTRL = (cfg->vol & 0xf) << I2S_TX_VOL_CTRL_VOL_Pos;

    if (cfg->balance_en != 0)
    {
        hi2s->Instance->TX_LR_BAL_CTRL = (cfg->balance_en << I2S_TX_LR_BAL_CTRL_EN_Pos) | ((cfg->balance_vol) << I2S_TX_LR_BAL_CTRL_BAL_VOL_Pos);
    }
    else
        hi2s->Instance->TX_LR_BAL_CTRL = 0;

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_I2S_Config_Receive(I2S_HandleTypeDef *hi2s, I2S_CFG_T *cfg)
{
    if ((hi2s == NULL) || (cfg == 0U))
    {
        return  HAL_ERROR;
    }


    hi2s->Instance->RX_RE_SAMPLE_CLK_DIV = ((cfg->clk_div->lr_clk_duty_low + cfg->clk_div->lr_clk_duty_high) << I2S_RX_RE_SAMPLE_CLK_DIV_RS_DUTY_Pos);
    hi2s->Instance->AUDIO_RX_BCLK_DIV = (cfg->clk_div->blck_duty << I2S_AUDIO_RX_BCLK_DIV_DUTY_Pos);
    hi2s->Instance->AUDIO_RX_LRCK_DIV = (cfg->clk_div->lr_clk_duty_low << I2S_AUDIO_RX_LRCK_DIV_DUTY_LOW_Pos) |
                                        (cfg->clk_div->lr_clk_duty_high << I2S_AUDIO_RX_LRCK_DIV_DUTY_HIGH_Pos);


    hi2s->Instance->AUDIO_RX_SERIAL_TIMING = (0 << I2S_AUDIO_RX_SERIAL_TIMING_TIMING_Pos) |
            ((cfg->slave_mode & 0x1) << I2S_AUDIO_RX_SERIAL_TIMING_SLAVE_EN_Pos) |
            ((cfg->lrck_invert & 0x1) << I2S_AUDIO_RX_SERIAL_TIMING_LRCK_POL_Pos);
    hi2s->Instance->AUDIO_RX_PCM_DW = (cfg->pcm_dw << I2S_AUDIO_RX_PCM_DW_PCM_DATA_WIDTH_Pos);//(cfg->data_dw << I2S_AUDIO_RX_PCM_DW_PCM_DATA_WIDTH_Pos);
    hi2s->Instance->RECORD_FORMAT =
        (((cfg->data_dw >> 4) & 0x1) << I2S_RECORD_FORMAT_DW_Pos) |              // dw
        ((cfg->track & 0x1) << I2S_RECORD_FORMAT_TRACK_Pos);              // Mono or storeo
    hi2s->Instance->RX_CH_SEL = cfg->chnl_sel;

    return HAL_OK;
}

/**
  * @brief Transmit an amount of data in blocking mode
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @param pData: a 16-bit pointer to data buffer.
  * @param Size: number of data sample to be sent:
  * @note When a 16-bit data frame or a 16-bit data frame extended is selected during the I2S
  *       configuration phase, the Size parameter means the number of 16-bit data length
  *       in the transaction and when a 24-bit data frame or a 32-bit data frame is selected
  *       the Size parameter means the number of 16-bit data length.
  * @param  Timeout: Timeout duration
  * @note The I2S is kept enabled at the end of transaction to avoid the clock de-synchronization
  *       between Master and Slave(example: audio streaming).
  * @note This function can use an Audio Frequency up to 48KHz when I2S Clock Source is 32MHz
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_I2S_Transmit(I2S_HandleTypeDef *hi2s, uint8_t *pData, uint32_t Size, uint32_t Timeout)
{
    if ((pData == NULL) || (Size == 0U))
    {
        return  HAL_ERROR;
    }

    /* Process Locked */
    __HAL_LOCK(hi2s);

    if (!(hi2s->State & HAL_I2S_STATE_BUSY_TX))
    {

        hi2s->TxXferSize = Size / 4; // byte to word
        hi2s->TxXferCount = Size / 4;

        /* Set state and reset error code */
        hi2s->ErrorCode = HAL_I2S_ERROR_NONE;
        hi2s->State |= HAL_I2S_STATE_BUSY_TX;
        hi2s->pTxBuffPtr = (uint32_t *)pData;

        /* Check if the I2S is already enabled */
        if (HAL_IS_BIT_CLR(hi2s->Instance->AUDIO_TX_FUNC_EN, I2S_AUDIO_TX_FUNC_EN_TX_EN))
        {
            /* Enable I2S peripheral */
            __HAL_I2S_TX_ENABLE(hi2s);
        }
        /* Clear I2S pause bit */
        CLEAR_BIT(hi2s->Instance->AUDIO_TX_PAUSE, I2S_AUDIO_TX_PAUSE_TX_PAUSE);

        while (hi2s->TxXferCount > 0U)
        {
            /* Wait until TXE flag is set */
            if (I2S_WaitFlagStateUntilTimeout(hi2s, I2S_FIFO_TX_EMPTY, RESET, Timeout) != HAL_OK)
            {
                return HAL_TIMEOUT;
            }
            hi2s->Instance->TX_DMA_ENTRY = (*hi2s->pTxBuffPtr++);
            hi2s->TxXferCount--;
        }

        /* Wait until TXE flag is set, to confirm the end of the transaction */
        if (I2S_WaitFlagStateUntilTimeout(hi2s, I2S_FIFO_TX_EMPTY, RESET, Timeout) != HAL_OK)
        {
            return HAL_TIMEOUT;
        }

        hi2s->State &= (~HAL_I2S_STATE_BUSY_TX);
        __HAL_I2S_TX_DISABLE(hi2s);

        /* Process Unlocked */
        __HAL_UNLOCK(hi2s);

        return HAL_OK;
    }
    else
    {
        /* Process Unlocked */
        __HAL_UNLOCK(hi2s);
        return HAL_BUSY;
    }
}

/**
  * @brief Receive an amount of data in blocking mode
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @param pData: a 16-bit pointer to data buffer.
  * @param Size: number of data sample to be sent:
  * @note When a 16-bit data frame or a 16-bit data frame extended is selected during the I2S
  *       configuration phase, the Size parameter means the number of 16-bit data length
  *       in the transaction and when a 24-bit data frame or a 32-bit data frame is selected
  *       the Size parameter means the number of 16-bit data length.
  * @param Timeout: Timeout duration
  * @note The I2S is kept enabled at the end of transaction to avoid the clock de-synchronization
  *       between Master and Slave(example: audio streaming).
  * @note In I2S Master Receiver mode, just after enabling the peripheral the clock will be generate
  *       in continouse way and as the I2S is not disabled at the end of the I2S transaction.
  * @note This function can use an Audio Frequency up to 44KHz when I2S Clock Source is 32MHz
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_I2S_Receive(I2S_HandleTypeDef *hi2s, uint8_t *pData, uint32_t Size, uint32_t Timeout)
{
    if ((pData == NULL) || (Size == 0U))
    {
        return  HAL_ERROR;
    }

    /* Process Locked */
    __HAL_LOCK(hi2s);

    if (!(hi2s->State & HAL_I2S_STATE_BUSY_RX))
    {
        hi2s->RxXferSize = Size / 4;
        hi2s->RxXferCount = Size / 4;


        /* Set state and reset error code */
        hi2s->ErrorCode = HAL_I2S_ERROR_NONE;
        hi2s->State |= HAL_I2S_STATE_BUSY_RX;
        hi2s->pRxBuffPtr = (uint32_t *)pData;

        /* Check if the I2S is already enabled */
        if (HAL_IS_BIT_CLR(hi2s->Instance->AUDIO_RX_FUNC_EN, I2S_AUDIO_RX_FUNC_EN_RX_EN))
        {
            /* Enable I2S peripheral */
            __HAL_I2S_RX_ENABLE(hi2s);
        }

        /* Clear I2S pause bit */
        CLEAR_BIT(hi2s->Instance->AUDIO_RX_PAUSE, I2S_AUDIO_RX_PAUSE_RX_PAUSE);

        /* Check if Master Receiver mode is selected */
        if (HAL_IS_BIT_CLR(hi2s->Instance->AUDIO_RX_SERIAL_TIMING, I2S_AUDIO_RX_SERIAL_TIMING_SLAVE_EN))
        {
            /* Clear the Overrun Flag by a read operation to the SPI_DR register followed by a read
            access to the SPI_SR register. */
            __HAL_I2S_CLEAR_OVRFLAG(hi2s);
        }

        /* Receive data */
        while (hi2s->RxXferCount > 0U)
        {
            /* Wait until RXNE flag is set */
            if (I2S_WaitFlagStateUntilTimeout(hi2s, I2S_FIFO_RX_EMPTY, SET, Timeout) != HAL_OK)
            {
                return HAL_TIMEOUT;
            }

            (*hi2s->pRxBuffPtr++) = hi2s->Instance->RX_DMA_ENTRY;
            hi2s->RxXferCount--;
        }

        hi2s->State &= ~HAL_I2S_STATE_BUSY_RX;

        __HAL_I2S_RX_DISABLE(hi2s);

        /* Process Unlocked */
        __HAL_UNLOCK(hi2s);

        return HAL_OK;
    }
    else
    {
        /* Process Unlocked */
        __HAL_UNLOCK(hi2s);
        return HAL_BUSY;
    }
}

/**
  * @brief Transmit an amount of data in non-blocking mode with Interrupt
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @param pData: a 16-bit pointer to data buffer.
  * @param Size: number of data sample to be sent:
  * @note When a 16-bit data frame or a 16-bit data frame extended is selected during the I2S
  *       configuration phase, the Size parameter means the number of 16-bit data length
  *       in the transaction and when a 24-bit data frame or a 32-bit data frame is selected
  *       the Size parameter means the number of 16-bit data length.
  * @note The I2S is kept enabled at the end of transaction to avoid the clock de-synchronization
  *       between Master and Slave(example: audio streaming).
  * @note This function can use an Audio Frequency up to 48KHz when I2S Clock Source is 32MHz
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_I2S_Transmit_IT(I2S_HandleTypeDef *hi2s, uint8_t *pData, uint32_t Size)
{
    if ((pData == NULL) || (Size == 0U))
    {
        return  HAL_ERROR;
    }

    /* Process Locked */
    __HAL_LOCK(hi2s);

    if (!(hi2s->State & HAL_I2S_STATE_BUSY_TX))
    {
        hi2s->pTxBuffPtr = (uint32_t *)pData;
        hi2s->State |= HAL_I2S_STATE_BUSY_TX;
        hi2s->ErrorCode = HAL_I2S_ERROR_NONE;


        hi2s->TxXferSize = Size;
        hi2s->TxXferCount = Size;


        /* Enable TXE and ERR interrupt */
        __HAL_I2S_ENABLE_IT(hi2s, I2S_IT_TXE);

        /* Check if the I2S is already enabled */
        if (HAL_IS_BIT_CLR(hi2s->Instance->AUDIO_TX_FUNC_EN, I2S_AUDIO_TX_FUNC_EN_TX_EN))
        {
            /* Enable I2S peripheral */
            __HAL_I2S_TX_ENABLE(hi2s);
        }

        /* Process Unlocked */
        __HAL_UNLOCK(hi2s);

        return HAL_OK;
    }
    else
    {
        /* Process Unlocked */
        __HAL_UNLOCK(hi2s);
        return HAL_BUSY;
    }
}

/**
  * @brief Receive an amount of data in non-blocking mode with Interrupt
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @param pData: a 16-bit pointer to the Receive data buffer.
  * @param Size: number of data sample to be sent:
  * @note When a 16-bit data frame or a 16-bit data frame extended is selected during the I2S
  *       configuration phase, the Size parameter means the number of 16-bit data length
  *       in the transaction and when a 24-bit data frame or a 32-bit data frame is selected
  *       the Size parameter means the number of 16-bit data length.
  * @note The I2S is kept enabled at the end of transaction to avoid the clock de-synchronization
  *       between Master and Slave(example: audio streaming).
  * @note It is recommended to use DMA for the I2S receiver to avoid de-synchronisation
  * between Master and Slave otherwise the I2S interrupt should be optimized.
  * @note This function can use an Audio Frequency up to 48KHz when I2S Clock Source is 32MHz
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_I2S_Receive_IT(I2S_HandleTypeDef *hi2s, uint8_t *pData, uint32_t Size)
{
    if ((pData == NULL) || (Size == 0U))
    {
        return  HAL_ERROR;
    }

    /* Process Locked */
    __HAL_LOCK(hi2s);

    if (!(hi2s->State & HAL_I2S_STATE_BUSY_RX))
    {
        hi2s->pRxBuffPtr = (uint32_t *)pData;
        hi2s->State |= HAL_I2S_STATE_BUSY_RX;
        hi2s->ErrorCode = HAL_I2S_ERROR_NONE;


        hi2s->RxXferSize = Size / 4;
        hi2s->RxXferCount = Size / 4;

        /* Enable RXNE and ERR interrupt */
        __HAL_I2S_ENABLE_IT(hi2s, I2S_IT_RXNE);

        /* Check if the I2S is already enabled */
        if (HAL_IS_BIT_CLR(hi2s->Instance->AUDIO_RX_FUNC_EN, I2S_AUDIO_RX_FUNC_EN_RX_EN))
        {
            /* Enable I2S peripheral */
            __HAL_I2S_RX_ENABLE(hi2s);
        }

        /* Process Unlocked */
        __HAL_UNLOCK(hi2s);

        return HAL_OK;
    }
    else
    {
        /* Process Unlocked */
        __HAL_UNLOCK(hi2s);
        return HAL_BUSY;
    }
}

/**
  * @brief Transmit an amount of data in non-blocking mode with DMA
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @param pData: a 16-bit pointer to the Transmit data buffer.
  * @param Size: number of data sample to be sent:
  * @note When a 16-bit data frame or a 16-bit data frame extended is selected during the I2S
  *       configuration phase, the Size parameter means the number of 16-bit data length
  *       in the transaction and when a 24-bit data frame or a 32-bit data frame is selected
  *       the Size parameter means the number of 16-bit data length.
  * @note The I2S is kept enabled at the end of transaction to avoid the clock de-synchronization
  *       between Master and Slave(example: audio streaming).
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_I2S_Transmit_DMA(I2S_HandleTypeDef *hi2s, uint8_t *pData, uint32_t Size)
{
    if ((hi2s == NULL) || (hi2s->hdmatx == NULL) || (pData == NULL) || (Size == 0U))
    {
        return  HAL_ERROR;
    }

    /* Process Locked */
    __HAL_LOCK(hi2s);

    if (!(hi2s->State & HAL_I2S_STATE_BUSY_TX))
    {
        hi2s->pTxBuffPtr = (uint32_t *)pData;
        hi2s->State |= HAL_I2S_STATE_BUSY_TX;
        hi2s->ErrorCode = HAL_I2S_ERROR_NONE;


        // DMA transmit as 32 bits
        hi2s->TxXferSize = Size >> 2;
        hi2s->TxXferCount = Size >> 2;

        /* Set the I2S Tx DMA Half transfert complete callback */
        hi2s->hdmatx->XferHalfCpltCallback = I2S_DMATxHalfCplt;

        /* Set the I2S Tx DMA transfert complete callback */
        hi2s->hdmatx->XferCpltCallback = I2S_DMATxCplt;

        /* Set the DMA error callback */
        hi2s->hdmatx->XferErrorCallback = I2S_DMAError;

        /* Clear I2S pause bit */
        CLEAR_BIT(hi2s->Instance->AUDIO_TX_PAUSE, I2S_AUDIO_TX_PAUSE_TX_PAUSE);

        /* Enable the Tx DMA Channel */
        HAL_DMA_Start_IT(hi2s->hdmatx, (uint32_t)hi2s->pTxBuffPtr, (uint32_t)&hi2s->Instance->TX_DMA_ENTRY, hi2s->TxXferSize);

        /* Check if the I2S Tx request is already enabled */
        if (HAL_IS_BIT_SET(hi2s->Instance->DMA_MASK, I2S_DMA_MASK_TX_DMA_MASK))
        {
            /* Enable Tx DMA Request */
            CLEAR_BIT(hi2s->Instance->DMA_MASK, I2S_DMA_MASK_TX_DMA_MASK);
        }

        /* Check if the I2S is already enabled */
        if (HAL_IS_BIT_CLR(hi2s->Instance->AUDIO_TX_FUNC_EN, I2S_AUDIO_TX_FUNC_EN_TX_EN))
        {
            /* Enable I2S peripheral */
            __HAL_I2S_TX_ENABLE(hi2s);
        }

        /* Process Unlocked */
        __HAL_UNLOCK(hi2s);

        return HAL_OK;
    }
    else
    {
        /* Process Unlocked */
        __HAL_UNLOCK(hi2s);
        return HAL_BUSY;
    }
}

/**
  * @brief Receive an amount of data in non-blocking mode with DMA
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @param pData: a 16-bit pointer to the Receive data buffer.
  * @param Size: number of data sample to be sent:
  * @note When a 16-bit data frame or a 16-bit data frame extended is selected during the I2S
  *       configuration phase, the Size parameter means the number of 16-bit data length
  *       in the transaction and when a 24-bit data frame or a 32-bit data frame is selected
  *       the Size parameter means the number of 16-bit data length.
  * @note The I2S is kept enabled at the end of transaction to avoid the clock de-synchronization
  *       between Master and Slave(example: audio streaming).
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_I2S_Receive_DMA(I2S_HandleTypeDef *hi2s, uint8_t *pData, uint32_t Size)
{
    if ((hi2s == NULL) || (hi2s->hdmarx == NULL) || (pData == NULL) || (Size == 0U))
    {
        return  HAL_ERROR;
    }

    /* Process Locked */
    __HAL_LOCK(hi2s);

    if (!(hi2s->State & HAL_I2S_STATE_BUSY_RX))
    {
        hi2s->pRxBuffPtr = (uint32_t *)pData;
        hi2s->State |= HAL_I2S_STATE_BUSY_RX;
        hi2s->ErrorCode = HAL_I2S_ERROR_NONE;

        // dma transmit with 32 bit
        hi2s->RxXferSize = Size >> 2;
        hi2s->RxXferCount = Size >> 2;


        /* Set the I2S Rx DMA Half transfert complete callback */
        hi2s->hdmarx->XferHalfCpltCallback = I2S_DMARxHalfCplt;

        /* Set the I2S Rx DMA transfert complete callback */
        hi2s->hdmarx->XferCpltCallback = I2S_DMARxCplt;

        /* Set the DMA error callback */
        hi2s->hdmarx->XferErrorCallback = I2S_DMAError;

        /* Clear I2S pause bit */
        CLEAR_BIT(hi2s->Instance->AUDIO_RX_PAUSE, I2S_AUDIO_RX_PAUSE_RX_PAUSE);

        /* Check if Master Receiver mode is selected */
        if (HAL_IS_BIT_CLR(hi2s->Instance->AUDIO_RX_SERIAL_TIMING, I2S_AUDIO_RX_SERIAL_TIMING_SLAVE_EN))
        {
            /* Clear the Overrun Flag by a read operation to the SPI_DR register followed by a read
            access to the SPI_SR register. */
            __HAL_I2S_CLEAR_OVRFLAG(hi2s);
        }

        /* Enable the Rx DMA Channel */
        HAL_DMA_Start_IT(hi2s->hdmarx, (uint32_t)&hi2s->Instance->RX_DMA_ENTRY, (uint32_t)hi2s->pRxBuffPtr, hi2s->RxXferSize);

        /* Check if the I2S Rx request is already enabled */
        if (HAL_IS_BIT_SET(hi2s->Instance->DMA_MASK, I2S_DMA_MASK_RX_DMA_MASK))
        {
            /* Enable Rx DMA Request */
            CLEAR_BIT(hi2s->Instance->DMA_MASK, I2S_DMA_MASK_RX_DMA_MASK);
        }

        /* Check if the I2S is already enabled */
        if (HAL_IS_BIT_CLR(hi2s->Instance->AUDIO_RX_FUNC_EN, I2S_AUDIO_RX_FUNC_EN_RX_EN))
        {
            /* Enable I2S peripheral */
            __HAL_I2S_RX_ENABLE(hi2s);
        }

        /* Process Unlocked */
        __HAL_UNLOCK(hi2s);

        return HAL_OK;
    }
    else
    {
        /* Process Unlocked */
        __HAL_UNLOCK(hi2s);
        return HAL_BUSY;
    }
}

/**
  * @brief Pauses the audio stream playing from the Media.
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_I2S_DMAPause(I2S_HandleTypeDef *hi2s)
{
    /* Process Locked */
    __HAL_LOCK(hi2s);

    if (hi2s->State & HAL_I2S_STATE_BUSY_TX)
    {
        /* Disable the I2S DMA Tx request */
        SET_BIT(hi2s->Instance->AUDIO_TX_PAUSE, I2S_AUDIO_TX_PAUSE_TX_PAUSE);
        hi2s->State |= HAL_I2S_STATE_PAUSE;
    }
    if (hi2s->State & HAL_I2S_STATE_BUSY_RX)
    {
        /* Disable the I2S DMA Rx request */
        SET_BIT(hi2s->Instance->AUDIO_RX_PAUSE, I2S_AUDIO_RX_PAUSE_RX_PAUSE);
        hi2s->State |= HAL_I2S_STATE_PAUSE;
    }

    /* Process Unlocked */
    __HAL_UNLOCK(hi2s);

    return HAL_OK;
}

/**
  * @brief Resumes the audio stream playing from the Media.
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_I2S_DMAResume(I2S_HandleTypeDef *hi2s)
{
    /* Process Locked */
    __HAL_LOCK(hi2s);

    if (hi2s->State == HAL_I2S_STATE_BUSY_TX)
    {
        /* Enable the I2S DMA Tx request */
        CLEAR_BIT(hi2s->Instance->AUDIO_TX_PAUSE, I2S_AUDIO_TX_PAUSE_TX_PAUSE);
        __HAL_I2S_TX_ENABLE(hi2s);
        hi2s->State &= ~HAL_I2S_STATE_PAUSE;
    }
    if (hi2s->State == HAL_I2S_STATE_BUSY_RX)
    {
        /* Enable the I2S DMA Rx request */
        CLEAR_BIT(hi2s->Instance->AUDIO_RX_PAUSE, I2S_AUDIO_RX_PAUSE_RX_PAUSE);
        __HAL_I2S_RX_ENABLE(hi2s);
        hi2s->State &= ~HAL_I2S_STATE_PAUSE;
    }

    /* Process Unlocked */
    __HAL_UNLOCK(hi2s);

    return HAL_OK;
}

/**
  * @brief Stops the audio stream playing from the Media.
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_I2S_DMAStop(I2S_HandleTypeDef *hi2s)
{
    /* Process Locked */
    __HAL_LOCK(hi2s);

    /* Disable the I2S Tx/Rx DMA requests */
    SET_BIT(hi2s->Instance->AUDIO_TX_PAUSE, I2S_AUDIO_TX_PAUSE_TX_PAUSE);
    SET_BIT(hi2s->Instance->AUDIO_RX_PAUSE, I2S_AUDIO_RX_PAUSE_RX_PAUSE);

    SET_BIT(hi2s->Instance->DMA_MASK, I2S_DMA_MASK_TX_DMA_MASK);
    SET_BIT(hi2s->Instance->DMA_MASK, I2S_DMA_MASK_RX_DMA_MASK);

    /* Abort the I2S DMA Channel tx */
    if (hi2s->hdmatx != NULL)
    {
        /* Disable the I2S DMA channel */
        __HAL_DMA_DISABLE(hi2s->hdmatx);
        HAL_DMA_Abort(hi2s->hdmatx);
    }
    /* Abort the I2S DMA Channel rx */
    if (hi2s->hdmarx != NULL)
    {
        /* Disable the I2S DMA channel */
        __HAL_DMA_DISABLE(hi2s->hdmarx);
        HAL_DMA_Abort(hi2s->hdmarx);
    }

    /* Disable I2S peripheral */
    __HAL_I2S_RX_DISABLE(hi2s);
    __HAL_I2S_TX_DISABLE(hi2s);

    hi2s->State = HAL_I2S_STATE_READY;

    /* Process Unlocked */
    __HAL_UNLOCK(hi2s);

    return HAL_OK;
}


/**
  * @brief Pauses the audio stream playing from the Media.
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_I2S_TX_DMAPause(I2S_HandleTypeDef *hi2s)
{
    /* Process Locked */
    __HAL_LOCK(hi2s);

    if (hi2s->State & HAL_I2S_STATE_BUSY_TX)
    {
        /* Disable the I2S DMA Tx request */
        SET_BIT(hi2s->Instance->AUDIO_TX_PAUSE, I2S_AUDIO_TX_PAUSE_TX_PAUSE);
        hi2s->State |= HAL_I2S_STATE_PAUSE;
    }

    /* Process Unlocked */
    __HAL_UNLOCK(hi2s);

    return HAL_OK;
}

/**
  * @brief Resumes the audio stream playing from the Media.
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_I2S_TX_DMAResume(I2S_HandleTypeDef *hi2s)
{
    /* Process Locked */
    __HAL_LOCK(hi2s);

    if (hi2s->State == HAL_I2S_STATE_BUSY_TX)
    {
        /* Enable the I2S DMA Tx request */
        CLEAR_BIT(hi2s->Instance->AUDIO_TX_PAUSE, I2S_AUDIO_TX_PAUSE_TX_PAUSE);
        __HAL_I2S_TX_ENABLE(hi2s);
        hi2s->State &= ~HAL_I2S_STATE_PAUSE;
    }

    /* Process Unlocked */
    __HAL_UNLOCK(hi2s);

    return HAL_OK;
}

/**
  * @brief Stops the audio stream playing from the Media.
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_I2S_TX_DMAStop(I2S_HandleTypeDef *hi2s)
{
    /* Process Locked */
    __HAL_LOCK(hi2s);

    /* Disable the I2S Tx/Rx DMA requests */
    SET_BIT(hi2s->Instance->AUDIO_TX_PAUSE, I2S_AUDIO_TX_PAUSE_TX_PAUSE);

    SET_BIT(hi2s->Instance->DMA_MASK, I2S_DMA_MASK_TX_DMA_MASK);

    /* Abort the I2S DMA Channel tx */
    if (hi2s->hdmatx != NULL)
    {
        /* Disable the I2S DMA channel */
        __HAL_DMA_DISABLE(hi2s->hdmatx);
        HAL_DMA_Abort(hi2s->hdmatx);
    }

    /* Disable I2S peripheral */
    __HAL_I2S_TX_DISABLE(hi2s);

    hi2s->State &= ~HAL_I2S_STATE_BUSY_TX;

    /* Process Unlocked */
    __HAL_UNLOCK(hi2s);

    return HAL_OK;
}

/**
  * @brief Pauses the audio stream recording.
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_I2S_RX_DMAPause(I2S_HandleTypeDef *hi2s)
{
    /* Process Locked */
    __HAL_LOCK(hi2s);

    if (hi2s->State & HAL_I2S_STATE_BUSY_RX)
    {
        /* Disable the I2S DMA Rx request */
        SET_BIT(hi2s->Instance->AUDIO_RX_PAUSE, I2S_AUDIO_RX_PAUSE_RX_PAUSE);
        hi2s->State |= HAL_I2S_STATE_PAUSE;
    }

    /* Process Unlocked */
    __HAL_UNLOCK(hi2s);

    return HAL_OK;
}

/**
  * @brief Resumes the audio stream recording.
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_I2S_RX_DMAResume(I2S_HandleTypeDef *hi2s)
{
    /* Process Locked */
    __HAL_LOCK(hi2s);

    if (hi2s->State == HAL_I2S_STATE_BUSY_RX)
    {
        /* Enable the I2S DMA Rx request */
        CLEAR_BIT(hi2s->Instance->AUDIO_RX_PAUSE, I2S_AUDIO_RX_PAUSE_RX_PAUSE);
        __HAL_I2S_RX_ENABLE(hi2s);
        hi2s->State &= ~HAL_I2S_STATE_PAUSE;
    }

    /* Process Unlocked */
    __HAL_UNLOCK(hi2s);

    return HAL_OK;
}

/**
  * @brief Stops the audio stream recording.
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_I2S_RX_DMAStop(I2S_HandleTypeDef *hi2s)
{
    /* Process Locked */
    __HAL_LOCK(hi2s);

    /* Disable the I2S Tx/Rx DMA requests */
    SET_BIT(hi2s->Instance->AUDIO_RX_PAUSE, I2S_AUDIO_RX_PAUSE_RX_PAUSE);

    SET_BIT(hi2s->Instance->DMA_MASK, I2S_DMA_MASK_RX_DMA_MASK);

    /* Abort the I2S DMA Channel rx */
    if (hi2s->hdmarx != NULL)
    {
        /* Disable the I2S DMA channel */
        __HAL_DMA_DISABLE(hi2s->hdmarx);
        HAL_DMA_Abort(hi2s->hdmarx);
    }

    /* Disable I2S peripheral */
    __HAL_I2S_RX_DISABLE(hi2s);

    hi2s->State &= ~HAL_I2S_STATE_BUSY_RX;

    /* Process Unlocked */
    __HAL_UNLOCK(hi2s);

    return HAL_OK;
}


/**
  * @brief  This function handles I2S interrupt request.
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval None
  */
__HAL_ROM_USED void HAL_I2S_IRQHandler(I2S_HandleTypeDef *hi2s)
{
    uint32_t i2ssr = hi2s->Instance->INT_STATUS;

    /* I2S in mode Receiver ------------------------------------------------*/
    if (((i2ssr & I2S_FLAG_RXNE) == I2S_FLAG_RXNE) && (__HAL_I2S_GET_IT_SOURCE(hi2s, I2S_IT_RXNE) != RESET))
    {
        I2S_Receive_IT(hi2s);
        return;
    }

    /* I2S in mode Tramitter -----------------------------------------------*/
    if (((i2ssr & I2S_FLAG_TXE) == I2S_FLAG_TXE) && (__HAL_I2S_GET_IT_SOURCE(hi2s, I2S_IT_TXE) != RESET))
    {
        I2S_Transmit_IT(hi2s);
        return;
    }

}

/**
  * @brief Tx Transfer Half completed callbacks
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval None
  */
__weak void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hi2s);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_I2S_TxHalfCpltCallback could be implemented in the user file
     */
}

/**
  * @brief Tx Transfer completed callbacks
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval None
  */
__weak void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hi2s);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_I2S_TxCpltCallback could be implemented in the user file
     */
}

/**
  * @brief Rx Transfer half completed callbacks
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval None
  */
__weak void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hi2s);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_I2S_RxHalfCpltCallback could be implemented in the user file
     */
}

/**
  * @brief Rx Transfer completed callbacks
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval None
  */
__weak void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hi2s);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_I2S_RxCpltCallback could be implemented in the user file
     */
}

/**
  * @brief I2S error callbacks
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval None
  */
__weak void HAL_I2S_ErrorCallback(I2S_HandleTypeDef *hi2s)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hi2s);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_I2S_ErrorCallback could be implemented in the user file
     */
}

/**
  * @}
  */

/** @addtogroup I2S_Exported_Functions_Group3
  *  @brief   Peripheral State functions
  *
@verbatim
 ===============================================================================
                      ##### Peripheral State and Errors functions #####
 ===============================================================================
    [..]
    This subsection permits to get in run-time the status of the peripheral
    and the data flow.

@endverbatim
  * @{
  */

/**
  * @brief  Return the I2S state
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval HAL state
  */
__HAL_ROM_USED HAL_I2S_StateTypeDef HAL_I2S_GetState(I2S_HandleTypeDef *hi2s)
{
    return hi2s->State;
}

/**
  * @brief  Return the I2S error code
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval I2S Error Code
  */
__HAL_ROM_USED uint32_t HAL_I2S_GetError(I2S_HandleTypeDef *hi2s)
{
    return hi2s->ErrorCode;
}
/**
  * @}
  */

/**
  * @}
  */

/* Private functions ---------------------------------------------------------*/
/** @addtogroup I2S_Private
  * @{
  */
/**
  * @brief DMA I2S transmit process complete callback
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
static void I2S_DMATxCplt(DMA_HandleTypeDef *hdma)
{
    I2S_HandleTypeDef *hi2s = (I2S_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

    if (HAL_IS_BIT_CLR(hdma->Instance->CCR, DMA_CCR_CIRC))
    {
        /* Disable Tx DMA Request */
        CLEAR_BIT(hi2s->Instance->DMA_MASK, I2S_DMA_MASK_TX_DMA_MASK);

        hi2s->TxXferCount = 0U;
        //hi2s->State = HAL_I2S_STATE_READY;
        hi2s->State &= ~HAL_I2S_STATE_BUSY_TX;
    }
    HAL_I2S_TxCpltCallback(hi2s);
}

/**
  * @brief DMA I2S transmit process half complete callback
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
static void I2S_DMATxHalfCplt(DMA_HandleTypeDef *hdma)
{
    I2S_HandleTypeDef *hi2s = (I2S_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

    HAL_I2S_TxHalfCpltCallback(hi2s);
}

/**
  * @brief DMA I2S receive process complete callback
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
static void I2S_DMARxCplt(DMA_HandleTypeDef *hdma)
{
    I2S_HandleTypeDef *hi2s = (I2S_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

    if (HAL_IS_BIT_CLR(hdma->Instance->CCR, DMA_CCR_CIRC))
    {
        /* Disable Rx DMA Request */
        CLEAR_BIT(hi2s->Instance->DMA_MASK, I2S_DMA_MASK_RX_DMA_MASK);

        hi2s->RxXferCount = 0U;
        //hi2s->State = HAL_I2S_STATE_READY;
        hi2s->State &= ~HAL_I2S_STATE_BUSY_RX;
    }
    HAL_I2S_RxCpltCallback(hi2s);
}

/**
  * @brief DMA I2S receive process half complete callback
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
static void I2S_DMARxHalfCplt(DMA_HandleTypeDef *hdma)
{
    I2S_HandleTypeDef *hi2s = (I2S_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

    HAL_I2S_RxHalfCpltCallback(hi2s);
}

/**
  * @brief DMA I2S communication error callback
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
static void I2S_DMAError(DMA_HandleTypeDef *hdma)
{
    I2S_HandleTypeDef *hi2s = (I2S_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

    /* Disable Rx and Tx DMA Request */
    CLEAR_BIT(hi2s->Instance->DMA_MASK, (I2S_DMA_MASK_TX_DMA_MASK | I2S_DMA_MASK_RX_DMA_MASK));
    hi2s->TxXferCount = 0U;
    hi2s->RxXferCount = 0U;

    hi2s->State = HAL_I2S_STATE_READY;

    /* Set the error code and execute error callback*/
    SET_BIT(hi2s->ErrorCode, HAL_I2S_ERROR_DMA);
    HAL_I2S_ErrorCallback(hi2s);
}

/**
  * @brief Transmit an amount of data in non-blocking mode with Interrupt
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval None
  */
static void I2S_Transmit_IT(I2S_HandleTypeDef *hi2s)
{
    /* Transmit data */
    hi2s->Instance->TX_DMA_ENTRY = (*hi2s->pTxBuffPtr++);
    hi2s->TxXferCount--;

    if (hi2s->TxXferCount == 0U)
    {
        /* Disable TXE and ERR interrupt */
        __HAL_I2S_DISABLE_IT(hi2s, I2S_IT_TXE);

        //hi2s->State = HAL_I2S_STATE_READY;
        hi2s->State &= ~HAL_I2S_STATE_BUSY_TX;
        HAL_I2S_TxCpltCallback(hi2s);
    }
}

/**
  * @brief Receive an amount of data in non-blocking mode with Interrupt
  * @param hi2s: I2S handle
  * @retval None
  */
static void I2S_Receive_IT(I2S_HandleTypeDef *hi2s)
{
    /* Receive data */
    (*hi2s->pRxBuffPtr++) = hi2s->Instance->RX_DMA_ENTRY;
    hi2s->RxXferCount--;

    if (hi2s->RxXferCount == 0U)
    {
        /* Disable RXNE and ERR interrupt */
        __HAL_I2S_DISABLE_IT(hi2s, I2S_IT_RXNE);

        //hi2s->State = HAL_I2S_STATE_READY;
        hi2s->State &= ~HAL_I2S_STATE_BUSY_RX;
        HAL_I2S_RxCpltCallback(hi2s);
    }
}


/**
  * @brief This function handles I2S Communication Timeout.
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @param Flag: Flag checked
  * @param Status: Value of the flag expected
  * @param Timeout: Duration of the timeout
  * @retval HAL status
  */
static HAL_StatusTypeDef I2S_WaitFlagStateUntilTimeout(I2S_HandleTypeDef *hi2s, uint32_t Flag, uint32_t Status, uint32_t Timeout)
{
    uint32_t tickstart = 0U;

    /* Get tick */
    tickstart = HAL_GetTick();

    /* Wait until flag is set */
    if (Status == RESET)
    {
        while (__HAL_I2S_GET_FLAG(hi2s, Flag) == RESET)
        {
            if (Timeout != HAL_MAX_DELAY)
            {
                if ((Timeout == 0U) || ((HAL_GetTick() - tickstart) > Timeout))
                {
                    /* Set the I2S State ready */
                    hi2s->State = HAL_I2S_STATE_READY;

                    /* Process Unlocked */
                    __HAL_UNLOCK(hi2s);

                    return HAL_TIMEOUT;
                }
            }
        }
    }
    else
    {
        while (__HAL_I2S_GET_FLAG(hi2s, Flag) != RESET)
        {
            if (Timeout != HAL_MAX_DELAY)
            {
                if ((Timeout == 0U) || ((HAL_GetTick() - tickstart) > Timeout))
                {
                    /* Set the I2S State ready */
                    hi2s->State = HAL_I2S_STATE_READY;

                    /* Process Unlocked */
                    __HAL_UNLOCK(hi2s);

                    return HAL_TIMEOUT;
                }
            }
        }
    }
    return HAL_OK;
}


#if  0 //obsolete

static int I2S_GetBCLKDiv(I2S_HandleTypeDef *hi2s, uint32_t sampleRate)
{
    uint32_t div, res;
    if (hi2s == NULL)
        return 0;

    if (!IS_I2S_SAMPLE_RATE(sampleRate))
        return 0;

    div = I2S_CLOCK_SYSCLK / sampleRate;

    if (HAL_I2S_FPGA_SYSCLK == I2S_CLOCK_SYSCLK) // FPGA clock, divider process
    {
        switch (sampleRate)
        {
        case I2S_SAMPLERATE_8000:
        case I2S_SAMPLERATE_16000:
        case I2S_SAMPLERATE_24000:
        case I2S_SAMPLERATE_32000:
        case I2S_SAMPLERATE_48000:
            res = div / 64;
            break;
        case I2S_SAMPLERATE_44100:
        case I2S_SAMPLERATE_22050:
        case I2S_SAMPLERATE_11025:
            res = 0;    // for FPGA,  not support !
            break;
        default:
            res = 0;
        }
    }
    else // asic clock
    {
        switch (sampleRate)
        {
        case I2S_SAMPLERATE_8000:
        case I2S_SAMPLERATE_16000:
        case I2S_SAMPLERATE_24000:
            res = div / 50;
            break;
        case I2S_SAMPLERATE_32000:
        case I2S_SAMPLERATE_48000:
            res = 5 ;
            break;
        case I2S_SAMPLERATE_44100:
        case I2S_SAMPLERATE_22050:
        case I2S_SAMPLERATE_11025:
            res = I2S_SAMPLERATE_44100 * 4 / sampleRate;
            break;
        default:
            res = 0;
        }
    }

    return res;
}
#endif /* 0 */


/**
  * @}
  */

/**
  * @}
  */
#endif /* HAL_I2S_MODULE_ENABLED */


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/


