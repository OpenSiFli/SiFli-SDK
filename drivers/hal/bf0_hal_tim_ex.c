/**
  ******************************************************************************
  * @file   bf0_hal_tim_ex.c
  * @author Sifli software development team
  * @brief   Header file of TIM HAL Extension module.
  * @attention
  ******************************************************************************
*/
/*
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

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal.h"

/** @addtogroup STM32H7xx_HAL_Driver
  * @{
  */

/** @defgroup TIMEx TIMEx
  * @brief TIM Extended HAL module driver
  * @{
  */

#ifdef HAL_ATIM_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void GPT_DMADelayPulseNCplt(DMA_HandleTypeDef *hdma);
static void GPT_DMADelayPulseHalfCplt(DMA_HandleTypeDef *hdma);
static void GPT_DMAErrorCCxN(DMA_HandleTypeDef *hdma);
static void GPT_CCxNChannelCmd(GPT_TypeDef *TIMx, uint32_t Channel, uint32_t ChannelNState);

/* Exported functions --------------------------------------------------------*/
/** @defgroup TIMEx_Exported_Functions TIM Extended Exported Functions
  * @{
  */

/** @defgroup TIMEx_Exported_Functions_Group1 Extended Timer Hall Sensor functions
  * @brief    Timer Hall Sensor functions
  *
@verbatim
  ==============================================================================
                      ##### Timer Hall Sensor functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
    (+) Initialize and configure TIM HAL Sensor.
    (+) De-initialize TIM HAL Sensor.
    (+) Start the Hall Sensor Interface.
    (+) Stop the Hall Sensor Interface.
    (+) Start the Hall Sensor Interface and enable interrupts.
    (+) Stop the Hall Sensor Interface and disable interrupts.
    (+) Start the Hall Sensor Interface and enable DMA transfers.
    (+) Stop the Hall Sensor Interface and disable DMA transfers.

@endverbatim
  * @{
  */
/**
  * @brief  Initializes the TIM Hall Sensor Interface and initialize the associated handle.
  * @note   When the timer instance is initialized in Hall Sensor Interface mode,
  *         timer channels 1 and channel 2 are reserved and cannot be used for
  *         other purpose.
  * @param  htim TIM Hall Sensor Interface handle
  * @param  sConfig TIM Hall Sensor configuration structure
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Init(GPT_HandleTypeDef *htim, GPT_HallSensor_InitTypeDef *sConfig)
{
    GPT_OC_InitTypeDef OC_Config;

    /* Check the TIM handle allocation */
    if (htim == NULL)
    {
        return HAL_ERROR;
    }

    if (htim->State == HAL_GPT_STATE_RESET)
    {
        /* Allocate lock resource and initialize it */
        htim->Lock = HAL_UNLOCKED;

        /* Init the low level hardware : GPIO, CLOCK, NVIC and DMA */
        HAL_TIMEx_HallSensor_MspInit(htim);
    }

    /* Set the TIM state */
    htim->State = HAL_GPT_STATE_BUSY;

    /* Configure the Time base in the Encoder Mode */
    GPT_Base_SetConfig(htim->Instance, &htim->Init);

    /* Configure the Channel 1 as Input Channel to interface with the three Outputs of the  Hall sensor */
    GPT_TI1_SetConfig(htim->Instance, sConfig->IC1Polarity, GPT_ICSELECTION_TRC, sConfig->IC1Filter);

    /* Reset the IC1PSC Bits */
    htim->Instance->CCMR1 &= ~GPT_CCMR1_IC1PSC;
    /* Set the IC1PSC value */
    htim->Instance->CCMR1 |= sConfig->IC1Prescaler;

    /* Enable the Hall sensor interface (XOR function of the three inputs) */
    htim->Instance->CR2 |= GPT_CR2_TI1S;

    /* Select the GPT_TS_TI1F_ED signal as Input trigger for the TIM */
    htim->Instance->SMCR &= ~GPT_SMCR_TS;
    htim->Instance->SMCR |= GPT_TS_TI1F_ED;

    /* Use the GPT_TS_TI1F_ED signal to reset the TIM counter each edge detection */
    htim->Instance->SMCR &= ~GPT_SMCR_SMS;
    htim->Instance->SMCR |= GPT_SLAVEMODE_RESET;

    /* Program channel 2 in PWM 2 mode with the desired Commutation_Delay*/
    OC_Config.OCFastMode = GPT_OCFAST_DISABLE;
    OC_Config.OCIdleState = GPT_OCIDLESTATE_RESET;
    OC_Config.OCMode = GPT_OCMODE_PWM2;
    OC_Config.OCNIdleState = GPT_OCNIDLESTATE_RESET;
    OC_Config.OCNPolarity = GPT_OCNPOLARITY_HIGH;
    OC_Config.OCPolarity = GPT_OCPOLARITY_HIGH;
    OC_Config.Pulse = sConfig->Commutation_Delay;

    GPT_OC2_SetConfig(htim->Instance, &OC_Config);

    /* Select OC2REF as trigger output on TRGO: write the MMS bits in the TIMx_CR2
      register to 101 */
    htim->Instance->CR2 &= ~GPT_CR2_MMS;
    htim->Instance->CR2 |= GPT_TRGO_OC2REF;

    /* Initialize the DMA burst operation state */
    htim->DMABurstState = HAL_DMA_BURST_STATE_READY;

    /* Initialize the TIM channels state */
    GPT_CHANNEL_STATE_SET(htim, GPT_CHANNEL_1, HAL_GPT_CHANNEL_STATE_READY);
    GPT_CHANNEL_STATE_SET(htim, GPT_CHANNEL_2, HAL_GPT_CHANNEL_STATE_READY);
    GPT_CHANNEL_N_STATE_SET(htim, GPT_CHANNEL_1, HAL_GPT_CHANNEL_STATE_READY);
    GPT_CHANNEL_N_STATE_SET(htim, GPT_CHANNEL_2, HAL_GPT_CHANNEL_STATE_READY);

    /* Initialize the TIM state*/
    htim->State = HAL_GPT_STATE_READY;

    return HAL_OK;
}

/**
  * @brief  DeInitializes the TIM Hall Sensor interface
  * @param  htim TIM Hall Sensor Interface handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_TIMEx_HallSensor_DeInit(GPT_HandleTypeDef *htim)
{
    htim->State = HAL_GPT_STATE_BUSY;

    /* Disable the TIM Peripheral Clock */
    __HAL_GPT_DISABLE(htim);

    /* DeInit the low level hardware: GPIO, CLOCK, NVIC */
    HAL_TIMEx_HallSensor_MspDeInit(htim);

    /* Change the DMA burst operation state */
    htim->DMABurstState = HAL_DMA_BURST_STATE_RESET;

    /* Change the TIM channels state */
    GPT_CHANNEL_STATE_SET(htim, GPT_CHANNEL_1, HAL_GPT_CHANNEL_STATE_RESET);
    GPT_CHANNEL_STATE_SET(htim, GPT_CHANNEL_2, HAL_GPT_CHANNEL_STATE_RESET);
    GPT_CHANNEL_N_STATE_SET(htim, GPT_CHANNEL_1, HAL_GPT_CHANNEL_STATE_RESET);
    GPT_CHANNEL_N_STATE_SET(htim, GPT_CHANNEL_2, HAL_GPT_CHANNEL_STATE_RESET);

    /* Change TIM state */
    htim->State = HAL_GPT_STATE_RESET;

    /* Release Lock */
    __HAL_UNLOCK(htim);

    return HAL_OK;
}

/**
  * @brief  Initializes the TIM Hall Sensor MSP.
  * @param  htim TIM Hall Sensor Interface handle
  * @retval None
  */
__weak void HAL_TIMEx_HallSensor_MspInit(GPT_HandleTypeDef *htim)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(htim);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_TIMEx_HallSensor_MspInit could be implemented in the user file
     */
}

/**
  * @brief  DeInitializes TIM Hall Sensor MSP.
  * @param  htim TIM Hall Sensor Interface handle
  * @retval None
  */
__weak void HAL_TIMEx_HallSensor_MspDeInit(GPT_HandleTypeDef *htim)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(htim);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_TIMEx_HallSensor_MspDeInit could be implemented in the user file
     */
}

/**
  * @brief  Starts the TIM Hall Sensor Interface.
  * @param  htim TIM Hall Sensor Interface handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Start(GPT_HandleTypeDef *htim)
{
    uint32_t tmpsmcr;
    HAL_GPT_ChannelStateTypeDef channel_1_state = GPT_CHANNEL_STATE_GET(htim, GPT_CHANNEL_1);
    HAL_GPT_ChannelStateTypeDef channel_2_state = GPT_CHANNEL_STATE_GET(htim, GPT_CHANNEL_2);
    HAL_GPT_ChannelStateTypeDef complementary_channel_1_state = GPT_CHANNEL_N_STATE_GET(htim, GPT_CHANNEL_1);
    HAL_GPT_ChannelStateTypeDef complementary_channel_2_state = GPT_CHANNEL_N_STATE_GET(htim, GPT_CHANNEL_2);


    /* Check the TIM channels state */
    if ((channel_1_state != HAL_GPT_CHANNEL_STATE_READY)
            || (channel_2_state != HAL_GPT_CHANNEL_STATE_READY)
            || (complementary_channel_1_state != HAL_GPT_CHANNEL_STATE_READY)
            || (complementary_channel_2_state != HAL_GPT_CHANNEL_STATE_READY))
    {
        return HAL_ERROR;
    }

    /* Set the TIM channels state */
    GPT_CHANNEL_STATE_SET(htim, GPT_CHANNEL_1, HAL_GPT_CHANNEL_STATE_BUSY);
    GPT_CHANNEL_STATE_SET(htim, GPT_CHANNEL_2, HAL_GPT_CHANNEL_STATE_BUSY);
    GPT_CHANNEL_N_STATE_SET(htim, GPT_CHANNEL_1, HAL_GPT_CHANNEL_STATE_BUSY);
    GPT_CHANNEL_N_STATE_SET(htim, GPT_CHANNEL_2, HAL_GPT_CHANNEL_STATE_BUSY);

    /* Enable the Input Capture channel 1
    (in the Hall Sensor Interface the three possible channels that can be used are GPT_CHANNEL_1, GPT_CHANNEL_2 and GPT_CHANNEL_3) */
    GPT_CCxChannelCmd(htim->Instance, GPT_CHANNEL_1, GPT_CCx_ENABLE);

    /* Enable the Peripheral, except in trigger mode where enable is automatically done with trigger */
    if (IS_GPT_SLAVE_INSTANCE(htim->Instance))
    {
        tmpsmcr = htim->Instance->SMCR & GPT_SMCR_SMS;
        if (!IS_GPT_SLAVEMODE_TRIGGER_ENABLED(tmpsmcr))
        {
            __HAL_GPT_ENABLE(htim);
        }
    }
    else
    {
        __HAL_GPT_ENABLE(htim);
    }

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  Stops the TIM Hall sensor Interface.
  * @param  htim TIM Hall Sensor Interface handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Stop(GPT_HandleTypeDef *htim)
{

    /* Disable the Input Capture channels 1, 2 and 3
      (in the Hall Sensor Interface the three possible channels that can be used are GPT_CHANNEL_1, GPT_CHANNEL_2 and GPT_CHANNEL_3) */
    GPT_CCxChannelCmd(htim->Instance, GPT_CHANNEL_1, GPT_CCx_DISABLE);

    /* Disable the Peripheral */
    __HAL_GPT_DISABLE(htim);

    /* Set the TIM channels state */
    GPT_CHANNEL_STATE_SET(htim, GPT_CHANNEL_1, HAL_GPT_CHANNEL_STATE_READY);
    GPT_CHANNEL_STATE_SET(htim, GPT_CHANNEL_2, HAL_GPT_CHANNEL_STATE_READY);
    GPT_CHANNEL_N_STATE_SET(htim, GPT_CHANNEL_1, HAL_GPT_CHANNEL_STATE_READY);
    GPT_CHANNEL_N_STATE_SET(htim, GPT_CHANNEL_2, HAL_GPT_CHANNEL_STATE_READY);

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  Starts the TIM Hall Sensor Interface in interrupt mode.
  * @param  htim TIM Hall Sensor Interface handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Start_IT(GPT_HandleTypeDef *htim)
{
    uint32_t tmpsmcr;
    HAL_GPT_ChannelStateTypeDef channel_1_state = GPT_CHANNEL_STATE_GET(htim, GPT_CHANNEL_1);
    HAL_GPT_ChannelStateTypeDef channel_2_state = GPT_CHANNEL_STATE_GET(htim, GPT_CHANNEL_2);
    HAL_GPT_ChannelStateTypeDef complementary_channel_1_state = GPT_CHANNEL_N_STATE_GET(htim, GPT_CHANNEL_1);
    HAL_GPT_ChannelStateTypeDef complementary_channel_2_state = GPT_CHANNEL_N_STATE_GET(htim, GPT_CHANNEL_2);

    /* Check the TIM channels state */
    if ((channel_1_state != HAL_GPT_CHANNEL_STATE_READY)
            || (channel_2_state != HAL_GPT_CHANNEL_STATE_READY)
            || (complementary_channel_1_state != HAL_GPT_CHANNEL_STATE_READY)
            || (complementary_channel_2_state != HAL_GPT_CHANNEL_STATE_READY))
    {
        return HAL_ERROR;
    }

    /* Set the TIM channels state */
    GPT_CHANNEL_STATE_SET(htim, GPT_CHANNEL_1, HAL_GPT_CHANNEL_STATE_BUSY);
    GPT_CHANNEL_STATE_SET(htim, GPT_CHANNEL_2, HAL_GPT_CHANNEL_STATE_BUSY);
    GPT_CHANNEL_N_STATE_SET(htim, GPT_CHANNEL_1, HAL_GPT_CHANNEL_STATE_BUSY);
    GPT_CHANNEL_N_STATE_SET(htim, GPT_CHANNEL_2, HAL_GPT_CHANNEL_STATE_BUSY);

    /* Enable the capture compare Interrupts 1 event */
    __HAL_GPT_ENABLE_IT(htim, GPT_IT_CC1);

    /* Enable the Input Capture channel 1
      (in the Hall Sensor Interface the three possible channels that can be used are GPT_CHANNEL_1, GPT_CHANNEL_2 and GPT_CHANNEL_3) */
    GPT_CCxChannelCmd(htim->Instance, GPT_CHANNEL_1, GPT_CCx_ENABLE);

    /* Enable the Peripheral, except in trigger mode where enable is automatically done with trigger */
    if (IS_GPT_SLAVE_INSTANCE(htim->Instance))
    {
        tmpsmcr = htim->Instance->SMCR & GPT_SMCR_SMS;
        if (!IS_GPT_SLAVEMODE_TRIGGER_ENABLED(tmpsmcr))
        {
            __HAL_GPT_ENABLE(htim);
        }
    }
    else
    {
        __HAL_GPT_ENABLE(htim);
    }

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  Stops the TIM Hall Sensor Interface in interrupt mode.
  * @param  htim TIM Hall Sensor Interface handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Stop_IT(GPT_HandleTypeDef *htim)
{

    /* Disable the Input Capture channel 1
      (in the Hall Sensor Interface the three possible channels that can be used are GPT_CHANNEL_1, GPT_CHANNEL_2 and GPT_CHANNEL_3) */
    GPT_CCxChannelCmd(htim->Instance, GPT_CHANNEL_1, GPT_CCx_DISABLE);

    /* Disable the capture compare Interrupts event */
    __HAL_GPT_DISABLE_IT(htim, GPT_IT_CC1);

    /* Disable the Peripheral */
    __HAL_GPT_DISABLE(htim);

    /* Set the TIM channels state */
    GPT_CHANNEL_STATE_SET(htim, GPT_CHANNEL_1, HAL_GPT_CHANNEL_STATE_READY);
    GPT_CHANNEL_STATE_SET(htim, GPT_CHANNEL_2, HAL_GPT_CHANNEL_STATE_READY);
    GPT_CHANNEL_N_STATE_SET(htim, GPT_CHANNEL_1, HAL_GPT_CHANNEL_STATE_READY);
    GPT_CHANNEL_N_STATE_SET(htim, GPT_CHANNEL_2, HAL_GPT_CHANNEL_STATE_READY);

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  Starts the TIM Hall Sensor Interface in DMA mode.
  * @param  htim TIM Hall Sensor Interface handle
  * @param  pData The destination Buffer address.
  * @param  Length The length of data to be transferred from TIM peripheral to memory.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Start_DMA(GPT_HandleTypeDef *htim, uint32_t *pData, uint16_t Length)
{
    uint32_t tmpsmcr;
    HAL_GPT_ChannelStateTypeDef channel_1_state = GPT_CHANNEL_STATE_GET(htim, GPT_CHANNEL_1);
    HAL_GPT_ChannelStateTypeDef complementary_channel_1_state = GPT_CHANNEL_N_STATE_GET(htim, GPT_CHANNEL_1);

    /* Set the TIM channel state */
    if ((channel_1_state == HAL_GPT_CHANNEL_STATE_BUSY)
            || (complementary_channel_1_state == HAL_GPT_CHANNEL_STATE_BUSY))
    {
        return HAL_BUSY;
    }
    else if ((channel_1_state == HAL_GPT_CHANNEL_STATE_READY)
             && (complementary_channel_1_state == HAL_GPT_CHANNEL_STATE_READY))
    {
        if ((pData == NULL) && (Length > 0U))
        {
            return HAL_ERROR;
        }
        else
        {
            GPT_CHANNEL_STATE_SET(htim, GPT_CHANNEL_1, HAL_GPT_CHANNEL_STATE_BUSY);
            GPT_CHANNEL_N_STATE_SET(htim, GPT_CHANNEL_1, HAL_GPT_CHANNEL_STATE_BUSY);
        }
    }
    else
    {
        return HAL_ERROR;
    }

    /* Enable the Input Capture channel 1
      (in the Hall Sensor Interface the three possible channels that can be used are GPT_CHANNEL_1, GPT_CHANNEL_2 and GPT_CHANNEL_3) */
    GPT_CCxChannelCmd(htim->Instance, GPT_CHANNEL_1, GPT_CCx_ENABLE);

    /* Set the DMA Input Capture 1 Callbacks */
    htim->hdma[GPT_DMA_ID_CC1]->XferCpltCallback = GPT_DMACaptureCplt;
    htim->hdma[GPT_DMA_ID_CC1]->XferHalfCpltCallback = GPT_DMACaptureHalfCplt;
    /* Set the DMA error callback */
    htim->hdma[GPT_DMA_ID_CC1]->XferErrorCallback = GPT_DMAError ;

    /* Enable the DMA stream for Capture 1*/
    if (HAL_DMA_Start_IT(htim->hdma[GPT_DMA_ID_CC1], (uint32_t)&htim->Instance->CCR1, (uint32_t)pData, Length) != HAL_OK)
    {
        /* Return error status */
        return HAL_ERROR;
    }
    /* Enable the capture compare 1 Interrupt */
    __HAL_GPT_ENABLE_DMA(htim, GPT_DMA_CC1);

    /* Enable the Peripheral, except in trigger mode where enable is automatically done with trigger */
    if (IS_GPT_SLAVE_INSTANCE(htim->Instance))
    {
        tmpsmcr = htim->Instance->SMCR & GPT_SMCR_SMS;
        if (!IS_GPT_SLAVEMODE_TRIGGER_ENABLED(tmpsmcr))
        {
            __HAL_GPT_ENABLE(htim);
        }
    }
    else
    {
        __HAL_GPT_ENABLE(htim);
    }

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  Stops the TIM Hall Sensor Interface in DMA mode.
  * @param  htim TIM Hall Sensor Interface handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Stop_DMA(GPT_HandleTypeDef *htim)
{

    /* Disable the Input Capture channel 1
      (in the Hall Sensor Interface the three possible channels that can be used are GPT_CHANNEL_1, GPT_CHANNEL_2 and GPT_CHANNEL_3) */
    GPT_CCxChannelCmd(htim->Instance, GPT_CHANNEL_1, GPT_CCx_DISABLE);


    /* Disable the capture compare Interrupts 1 event */
    __HAL_GPT_DISABLE_DMA(htim, GPT_DMA_CC1);

    (void)HAL_DMA_Abort_IT(htim->hdma[GPT_DMA_ID_CC1]);

    /* Disable the Peripheral */
    __HAL_GPT_DISABLE(htim);

    /* Set the TIM channel state */
    GPT_CHANNEL_STATE_SET(htim, GPT_CHANNEL_1, HAL_GPT_CHANNEL_STATE_READY);
    GPT_CHANNEL_N_STATE_SET(htim, GPT_CHANNEL_1, HAL_GPT_CHANNEL_STATE_READY);

    /* Return function status */
    return HAL_OK;
}

/**
  * @}
  */

/** @defgroup TIMEx_Exported_Functions_Group2 Extended Timer Complementary Output Compare functions
  *  @brief   Timer Complementary Output Compare functions
  *
@verbatim
  ==============================================================================
              ##### Timer Complementary Output Compare functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
    (+) Start the Complementary Output Compare/PWM.
    (+) Stop the Complementary Output Compare/PWM.
    (+) Start the Complementary Output Compare/PWM and enable interrupts.
    (+) Stop the Complementary Output Compare/PWM and disable interrupts.
    (+) Start the Complementary Output Compare/PWM and enable DMA transfers.
    (+) Stop the Complementary Output Compare/PWM and disable DMA transfers.

@endverbatim
  * @{
  */

/**
  * @brief  Starts the TIM Output Compare signal generation on the complementary
  *         output.
  * @param  htim TIM Output Compare handle
  * @param  Channel TIM Channel to be enabled
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  *            @arg GPT_CHANNEL_3: TIM Channel 3 selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_TIMEx_OCN_Start(GPT_HandleTypeDef *htim, uint32_t Channel)
{
    uint32_t tmpsmcr;


    /* Check the TIM complementary channel state */
    if (GPT_CHANNEL_N_STATE_GET(htim, Channel) != HAL_GPT_CHANNEL_STATE_READY)
    {
        return HAL_ERROR;
    }

    /* Set the TIM complementary channel state */
    GPT_CHANNEL_N_STATE_SET(htim, Channel, HAL_GPT_CHANNEL_STATE_BUSY);

    /* Enable the Capture compare channel N */
    GPT_CCxNChannelCmd(htim->Instance, Channel, GPT_CCxN_ENABLE);

    /* Enable the Main Output */
    __HAL_GPT_MOE_ENABLE(htim);

    /* Enable the Peripheral, except in trigger mode where enable is automatically done with trigger */
    if (IS_GPT_SLAVE_INSTANCE(htim->Instance))
    {
        tmpsmcr = htim->Instance->SMCR & GPT_SMCR_SMS;
        if (!IS_GPT_SLAVEMODE_TRIGGER_ENABLED(tmpsmcr))
        {
            __HAL_GPT_ENABLE(htim);
        }
    }
    else
    {
        __HAL_GPT_ENABLE(htim);
    }

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  Stops the TIM Output Compare signal generation on the complementary
  *         output.
  * @param  htim TIM handle
  * @param  Channel TIM Channel to be disabled
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  *            @arg GPT_CHANNEL_3: TIM Channel 3 selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop(GPT_HandleTypeDef *htim, uint32_t Channel)
{
    /* Disable the Capture compare channel N */
    GPT_CCxNChannelCmd(htim->Instance, Channel, GPT_CCxN_DISABLE);

    /* Disable the Main Output */
    __HAL_GPT_MOE_DISABLE(htim);

    /* Disable the Peripheral */
    __HAL_GPT_DISABLE(htim);

    /* Set the TIM complementary channel state */
    GPT_CHANNEL_N_STATE_SET(htim, Channel, HAL_GPT_CHANNEL_STATE_READY);

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  Starts the TIM Output Compare signal generation in interrupt mode
  *         on the complementary output.
  * @param  htim TIM OC handle
  * @param  Channel TIM Channel to be enabled
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  *            @arg GPT_CHANNEL_3: TIM Channel 3 selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_TIMEx_OCN_Start_IT(GPT_HandleTypeDef *htim, uint32_t Channel)
{
    uint32_t tmpsmcr;


    /* Check the TIM complementary channel state */
    if (GPT_CHANNEL_N_STATE_GET(htim, Channel) != HAL_GPT_CHANNEL_STATE_READY)
    {
        return HAL_ERROR;
    }

    /* Set the TIM complementary channel state */
    GPT_CHANNEL_N_STATE_SET(htim, Channel, HAL_GPT_CHANNEL_STATE_BUSY);

    switch (Channel)
    {
    case GPT_CHANNEL_1:
    {
        /* Enable the TIM Output Compare interrupt */
        __HAL_GPT_ENABLE_IT(htim, GPT_IT_CC1);
        break;
    }

    case GPT_CHANNEL_2:
    {
        /* Enable the TIM Output Compare interrupt */
        __HAL_GPT_ENABLE_IT(htim, GPT_IT_CC2);
        break;
    }

    case GPT_CHANNEL_3:
    {
        /* Enable the TIM Output Compare interrupt */
        __HAL_GPT_ENABLE_IT(htim, GPT_IT_CC3);
        break;
    }


    default:
        break;
    }

    /* Enable the TIM Break interrupt */
    __HAL_GPT_ENABLE_IT(htim, GPT_IT_BREAK);

    /* Enable the Capture compare channel N */
    GPT_CCxNChannelCmd(htim->Instance, Channel, GPT_CCxN_ENABLE);

    /* Enable the Main Output */
    __HAL_GPT_MOE_ENABLE(htim);

    /* Enable the Peripheral, except in trigger mode where enable is automatically done with trigger */
    if (IS_GPT_SLAVE_INSTANCE(htim->Instance))
    {
        tmpsmcr = htim->Instance->SMCR & GPT_SMCR_SMS;
        if (!IS_GPT_SLAVEMODE_TRIGGER_ENABLED(tmpsmcr))
        {
            __HAL_GPT_ENABLE(htim);
        }
    }
    else
    {
        __HAL_GPT_ENABLE(htim);
    }

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  Stops the TIM Output Compare signal generation in interrupt mode
  *         on the complementary output.
  * @param  htim TIM Output Compare handle
  * @param  Channel TIM Channel to be disabled
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  *            @arg GPT_CHANNEL_3: TIM Channel 3 selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop_IT(GPT_HandleTypeDef *htim, uint32_t Channel)
{
    uint32_t tmpccer;

    switch (Channel)
    {
    case GPT_CHANNEL_1:
    {
        /* Disable the TIM Output Compare interrupt */
        __HAL_GPT_DISABLE_IT(htim, GPT_IT_CC1);
        break;
    }

    case GPT_CHANNEL_2:
    {
        /* Disable the TIM Output Compare interrupt */
        __HAL_GPT_DISABLE_IT(htim, GPT_IT_CC2);
        break;
    }

    case GPT_CHANNEL_3:
    {
        /* Disable the TIM Output Compare interrupt */
        __HAL_GPT_DISABLE_IT(htim, GPT_IT_CC3);
        break;
    }

    default:
        break;
    }

    /* Disable the Capture compare channel N */
    GPT_CCxNChannelCmd(htim->Instance, Channel, GPT_CCxN_DISABLE);

    /* Disable the TIM Break interrupt (only if no more channel is active) */
    tmpccer = htim->Instance->CCER;
    if ((tmpccer & (GPT_CCER_CC1NE | GPT_CCER_CC2NE | GPT_CCER_CC3NE)) == (uint32_t)RESET)
    {
        __HAL_GPT_DISABLE_IT(htim, GPT_IT_BREAK);
    }

    /* Disable the Main Output */
    __HAL_GPT_MOE_DISABLE(htim);

    /* Disable the Peripheral */
    __HAL_GPT_DISABLE(htim);

    /* Set the TIM complementary channel state */
    GPT_CHANNEL_N_STATE_SET(htim, Channel, HAL_GPT_CHANNEL_STATE_READY);

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  Starts the TIM Output Compare signal generation in DMA mode
  *         on the complementary output.
  * @param  htim TIM Output Compare handle
  * @param  Channel TIM Channel to be enabled
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  *            @arg GPT_CHANNEL_3: TIM Channel 3 selected
  * @param  pData The source Buffer address.
  * @param  Length The length of data to be transferred from memory to TIM peripheral
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_TIMEx_OCN_Start_DMA(GPT_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length)
{
    uint32_t tmpsmcr;


    /* Set the TIM complementary channel state */
    if (GPT_CHANNEL_N_STATE_GET(htim, Channel) == HAL_GPT_CHANNEL_STATE_BUSY)
    {
        return HAL_BUSY;
    }
    else if (GPT_CHANNEL_N_STATE_GET(htim, Channel) == HAL_GPT_CHANNEL_STATE_READY)
    {
        if ((pData == NULL) && (Length > 0U))
        {
            return HAL_ERROR;
        }
        else
        {
            GPT_CHANNEL_N_STATE_SET(htim, Channel, HAL_GPT_CHANNEL_STATE_BUSY);
        }
    }
    else
    {
        return HAL_ERROR;
    }

    switch (Channel)
    {
    case GPT_CHANNEL_1:
    {
        /* Set the DMA compare callbacks */
        htim->hdma[GPT_DMA_ID_CC1]->XferCpltCallback = GPT_DMADelayPulseNCplt;
        htim->hdma[GPT_DMA_ID_CC1]->XferHalfCpltCallback = GPT_DMADelayPulseHalfCplt;

        /* Set the DMA error callback */
        htim->hdma[GPT_DMA_ID_CC1]->XferErrorCallback = GPT_DMAErrorCCxN ;

        /* Enable the DMA stream */
        if (HAL_DMA_Start_IT(htim->hdma[GPT_DMA_ID_CC1], (uint32_t)pData, (uint32_t)&htim->Instance->CCR1, Length) != HAL_OK)
        {
            /* Return error status */
            return HAL_ERROR;
        }
        /* Enable the TIM Output Compare DMA request */
        __HAL_GPT_ENABLE_DMA(htim, GPT_DMA_CC1);
        break;
    }

    case GPT_CHANNEL_2:
    {
        /* Set the DMA compare callbacks */
        htim->hdma[GPT_DMA_ID_CC2]->XferCpltCallback = GPT_DMADelayPulseNCplt;
        htim->hdma[GPT_DMA_ID_CC2]->XferHalfCpltCallback = GPT_DMADelayPulseHalfCplt;

        /* Set the DMA error callback */
        htim->hdma[GPT_DMA_ID_CC2]->XferErrorCallback = GPT_DMAErrorCCxN ;

        /* Enable the DMA stream */
        if (HAL_DMA_Start_IT(htim->hdma[GPT_DMA_ID_CC2], (uint32_t)pData, (uint32_t)&htim->Instance->CCR2, Length) != HAL_OK)
        {
            /* Return error status */
            return HAL_ERROR;
        }
        /* Enable the TIM Output Compare DMA request */
        __HAL_GPT_ENABLE_DMA(htim, GPT_DMA_CC2);
        break;
    }

    case GPT_CHANNEL_3:
    {
        /* Set the DMA compare callbacks */
        htim->hdma[GPT_DMA_ID_CC3]->XferCpltCallback = GPT_DMADelayPulseNCplt;
        htim->hdma[GPT_DMA_ID_CC3]->XferHalfCpltCallback = GPT_DMADelayPulseHalfCplt;

        /* Set the DMA error callback */
        htim->hdma[GPT_DMA_ID_CC3]->XferErrorCallback = GPT_DMAErrorCCxN ;

        /* Enable the DMA stream */
        if (HAL_DMA_Start_IT(htim->hdma[GPT_DMA_ID_CC3], (uint32_t)pData, (uint32_t)&htim->Instance->CCR3, Length) != HAL_OK)
        {
            /* Return error status */
            return HAL_ERROR;
        }
        /* Enable the TIM Output Compare DMA request */
        __HAL_GPT_ENABLE_DMA(htim, GPT_DMA_CC3);
        break;
    }

    default:
        break;
    }

    /* Enable the Capture compare channel N */
    GPT_CCxNChannelCmd(htim->Instance, Channel, GPT_CCxN_ENABLE);

    /* Enable the Main Output */
    __HAL_GPT_MOE_ENABLE(htim);

    /* Enable the Peripheral, except in trigger mode where enable is automatically done with trigger */
    if (IS_GPT_SLAVE_INSTANCE(htim->Instance))
    {
        tmpsmcr = htim->Instance->SMCR & GPT_SMCR_SMS;
        if (!IS_GPT_SLAVEMODE_TRIGGER_ENABLED(tmpsmcr))
        {
            __HAL_GPT_ENABLE(htim);
        }
    }
    else
    {
        __HAL_GPT_ENABLE(htim);
    }

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  Stops the TIM Output Compare signal generation in DMA mode
  *         on the complementary output.
  * @param  htim TIM Output Compare handle
  * @param  Channel TIM Channel to be disabled
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  *            @arg GPT_CHANNEL_3: TIM Channel 3 selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop_DMA(GPT_HandleTypeDef *htim, uint32_t Channel)
{

    switch (Channel)
    {
    case GPT_CHANNEL_1:
    {
        /* Disable the TIM Output Compare DMA request */
        __HAL_GPT_DISABLE_DMA(htim, GPT_DMA_CC1);
        (void)HAL_DMA_Abort_IT(htim->hdma[GPT_DMA_ID_CC1]);
        break;
    }

    case GPT_CHANNEL_2:
    {
        /* Disable the TIM Output Compare DMA request */
        __HAL_GPT_DISABLE_DMA(htim, GPT_DMA_CC2);
        (void)HAL_DMA_Abort_IT(htim->hdma[GPT_DMA_ID_CC2]);
        break;
    }

    case GPT_CHANNEL_3:
    {
        /* Disable the TIM Output Compare DMA request */
        __HAL_GPT_DISABLE_DMA(htim, GPT_DMA_CC3);
        (void)HAL_DMA_Abort_IT(htim->hdma[GPT_DMA_ID_CC3]);
        break;
    }

    default:
        break;
    }

    /* Disable the Capture compare channel N */
    GPT_CCxNChannelCmd(htim->Instance, Channel, GPT_CCxN_DISABLE);

    /* Disable the Main Output */
    __HAL_GPT_MOE_DISABLE(htim);

    /* Disable the Peripheral */
    __HAL_GPT_DISABLE(htim);

    /* Set the TIM complementary channel state */
    GPT_CHANNEL_N_STATE_SET(htim, Channel, HAL_GPT_CHANNEL_STATE_READY);

    /* Return function status */
    return HAL_OK;
}

/**
  * @}
  */

/** @defgroup TIMEx_Exported_Functions_Group3 Extended Timer Complementary PWM functions
  * @brief    Timer Complementary PWM functions
  *
@verbatim
  ==============================================================================
                 ##### Timer Complementary PWM functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
    (+) Start the Complementary PWM.
    (+) Stop the Complementary PWM.
    (+) Start the Complementary PWM and enable interrupts.
    (+) Stop the Complementary PWM and disable interrupts.
    (+) Start the Complementary PWM and enable DMA transfers.
    (+) Stop the Complementary PWM and disable DMA transfers.
    (+) Start the Complementary Input Capture measurement.
    (+) Stop the Complementary Input Capture.
    (+) Start the Complementary Input Capture and enable interrupts.
    (+) Stop the Complementary Input Capture and disable interrupts.
    (+) Start the Complementary Input Capture and enable DMA transfers.
    (+) Stop the Complementary Input Capture and disable DMA transfers.
    (+) Start the Complementary One Pulse generation.
    (+) Stop the Complementary One Pulse.
    (+) Start the Complementary One Pulse and enable interrupts.
    (+) Stop the Complementary One Pulse and disable interrupts.

@endverbatim
  * @{
  */

/**
  * @brief  Starts the PWM signal generation on the complementary output.
  * @param  htim TIM handle
  * @param  Channel TIM Channel to be enabled
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  *            @arg GPT_CHANNEL_3: TIM Channel 3 selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start(GPT_HandleTypeDef *htim, uint32_t Channel)
{
    uint32_t tmpsmcr;

    /* Set the TIM complementary channel state */
    GPT_CHANNEL_N_STATE_SET(htim, Channel, HAL_GPT_CHANNEL_STATE_BUSY);

    /* Enable the complementary PWM output  */
    GPT_CCxNChannelCmd(htim->Instance, Channel, GPT_CCxN_ENABLE);

    /* Enable the Main Output */
    __HAL_GPT_MOE_ENABLE(htim);

    /* Enable the Peripheral, except in trigger mode where enable is automatically done with trigger */
    if (IS_GPT_SLAVE_INSTANCE(htim->Instance))
    {
        tmpsmcr = htim->Instance->SMCR & GPT_SMCR_SMS;
        if (!IS_GPT_SLAVEMODE_TRIGGER_ENABLED(tmpsmcr))
        {
            __HAL_GPT_ENABLE(htim);
        }
    }
    else
    {
        __HAL_GPT_ENABLE(htim);
    }

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  Stops the PWM signal generation on the complementary output.
  * @param  htim TIM handle
  * @param  Channel TIM Channel to be disabled
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  *            @arg GPT_CHANNEL_3: TIM Channel 3 selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop(GPT_HandleTypeDef *htim, uint32_t Channel)
{
    /* Disable the complementary PWM output  */
    GPT_CCxNChannelCmd(htim->Instance, Channel, GPT_CCxN_DISABLE);

    /* Disable the Main Output */
    __HAL_GPT_MOE_DISABLE(htim);

    /* Disable the Peripheral */
    __HAL_GPT_DISABLE(htim);

    /* Set the TIM complementary channel state */
    GPT_CHANNEL_N_STATE_SET(htim, Channel, HAL_GPT_CHANNEL_STATE_READY);

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  Starts the PWM signal generation in interrupt mode on the
  *         complementary output.
  * @param  htim TIM handle
  * @param  Channel TIM Channel to be disabled
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  *            @arg GPT_CHANNEL_3: TIM Channel 3 selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start_IT(GPT_HandleTypeDef *htim, uint32_t Channel)
{
    uint32_t tmpsmcr;

    /* Check the TIM complementary channel state */
    if (GPT_CHANNEL_N_STATE_GET(htim, Channel) != HAL_GPT_CHANNEL_STATE_READY)
    {
        return HAL_ERROR;
    }

    /* Set the TIM complementary channel state */
    GPT_CHANNEL_N_STATE_SET(htim, Channel, HAL_GPT_CHANNEL_STATE_BUSY);

    switch (Channel)
    {
    case GPT_CHANNEL_1:
    {
        /* Enable the TIM Capture/Compare 1 interrupt */
        __HAL_GPT_ENABLE_IT(htim, GPT_IT_CC1);
        break;
    }

    case GPT_CHANNEL_2:
    {
        /* Enable the TIM Capture/Compare 2 interrupt */
        __HAL_GPT_ENABLE_IT(htim, GPT_IT_CC2);
        break;
    }

    case GPT_CHANNEL_3:
    {
        /* Enable the TIM Capture/Compare 3 interrupt */
        __HAL_GPT_ENABLE_IT(htim, GPT_IT_CC3);
        break;
    }

    default:
        break;
    }

    /* Enable the TIM Break interrupt */
    __HAL_GPT_ENABLE_IT(htim, GPT_IT_BREAK);

    /* Enable the complementary PWM output  */
    GPT_CCxNChannelCmd(htim->Instance, Channel, GPT_CCxN_ENABLE);

    /* Enable the Main Output */
    __HAL_GPT_MOE_ENABLE(htim);

    /* Enable the Peripheral, except in trigger mode where enable is automatically done with trigger */
    if (IS_GPT_SLAVE_INSTANCE(htim->Instance))
    {
        tmpsmcr = htim->Instance->SMCR & GPT_SMCR_SMS;
        if (!IS_GPT_SLAVEMODE_TRIGGER_ENABLED(tmpsmcr))
        {
            __HAL_GPT_ENABLE(htim);
        }
    }
    else
    {
        __HAL_GPT_ENABLE(htim);
    }

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  Stops the PWM signal generation in interrupt mode on the
  *         complementary output.
  * @param  htim TIM handle
  * @param  Channel TIM Channel to be disabled
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  *            @arg GPT_CHANNEL_3: TIM Channel 3 selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop_IT(GPT_HandleTypeDef *htim, uint32_t Channel)
{
    uint32_t tmpccer;


    switch (Channel)
    {
    case GPT_CHANNEL_1:
    {
        /* Disable the TIM Capture/Compare 1 interrupt */
        __HAL_GPT_DISABLE_IT(htim, GPT_IT_CC1);
        break;
    }

    case GPT_CHANNEL_2:
    {
        /* Disable the TIM Capture/Compare 2 interrupt */
        __HAL_GPT_DISABLE_IT(htim, GPT_IT_CC2);
        break;
    }

    case GPT_CHANNEL_3:
    {
        /* Disable the TIM Capture/Compare 3 interrupt */
        __HAL_GPT_DISABLE_IT(htim, GPT_IT_CC3);
        break;
    }

    default:
        break;
    }

    /* Disable the complementary PWM output  */
    GPT_CCxNChannelCmd(htim->Instance, Channel, GPT_CCxN_DISABLE);

    /* Disable the TIM Break interrupt (only if no more channel is active) */
    tmpccer = htim->Instance->CCER;
    if ((tmpccer & (GPT_CCER_CC1NE | GPT_CCER_CC2NE | GPT_CCER_CC3NE)) == (uint32_t)RESET)
    {
        __HAL_GPT_DISABLE_IT(htim, GPT_IT_BREAK);
    }

    /* Disable the Main Output */
    __HAL_GPT_MOE_DISABLE(htim);

    /* Disable the Peripheral */
    __HAL_GPT_DISABLE(htim);

    /* Set the TIM complementary channel state */
    GPT_CHANNEL_N_STATE_SET(htim, Channel, HAL_GPT_CHANNEL_STATE_READY);

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  Starts the TIM PWM signal generation in DMA mode on the
  *         complementary output
  * @param  htim TIM handle
  * @param  Channel TIM Channel to be enabled
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  *            @arg GPT_CHANNEL_3: TIM Channel 3 selected
  * @param  pData The source Buffer address.
  * @param  Length The length of data to be transferred from memory to TIM peripheral
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start_DMA(GPT_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length)
{
    uint32_t tmpsmcr;

    /* Set the TIM complementary channel state */
    if (GPT_CHANNEL_N_STATE_GET(htim, Channel) == HAL_GPT_CHANNEL_STATE_BUSY)
    {
        return HAL_BUSY;
    }
    else if (GPT_CHANNEL_N_STATE_GET(htim, Channel) == HAL_GPT_CHANNEL_STATE_READY)
    {
        if ((pData == NULL) && (Length > 0U))
        {
            return HAL_ERROR;
        }
        else
        {
            GPT_CHANNEL_N_STATE_SET(htim, Channel, HAL_GPT_CHANNEL_STATE_BUSY);
        }
    }
    else
    {
        return HAL_ERROR;
    }

    switch (Channel)
    {
    case GPT_CHANNEL_1:
    {
        /* Set the DMA compare callbacks */
        htim->hdma[GPT_DMA_ID_CC1]->XferCpltCallback = GPT_DMADelayPulseNCplt;
        htim->hdma[GPT_DMA_ID_CC1]->XferHalfCpltCallback = GPT_DMADelayPulseHalfCplt;

        /* Set the DMA error callback */
        htim->hdma[GPT_DMA_ID_CC1]->XferErrorCallback = GPT_DMAErrorCCxN ;

        /* Enable the DMA stream */
        if (HAL_DMA_Start_IT(htim->hdma[GPT_DMA_ID_CC1], (uint32_t)pData, (uint32_t)&htim->Instance->CCR1, Length) != HAL_OK)
        {
            /* Return error status */
            return HAL_ERROR;
        }
        /* Enable the TIM Capture/Compare 1 DMA request */
        __HAL_GPT_ENABLE_DMA(htim, GPT_DMA_CC1);
        break;
    }

    case GPT_CHANNEL_2:
    {
        /* Set the DMA compare callbacks */
        htim->hdma[GPT_DMA_ID_CC2]->XferCpltCallback = GPT_DMADelayPulseNCplt;
        htim->hdma[GPT_DMA_ID_CC2]->XferHalfCpltCallback = GPT_DMADelayPulseHalfCplt;

        /* Set the DMA error callback */
        htim->hdma[GPT_DMA_ID_CC2]->XferErrorCallback = GPT_DMAErrorCCxN ;

        /* Enable the DMA stream */
        if (HAL_DMA_Start_IT(htim->hdma[GPT_DMA_ID_CC2], (uint32_t)pData, (uint32_t)&htim->Instance->CCR2, Length) != HAL_OK)
        {
            /* Return error status */
            return HAL_ERROR;
        }
        /* Enable the TIM Capture/Compare 2 DMA request */
        __HAL_GPT_ENABLE_DMA(htim, GPT_DMA_CC2);
        break;
    }

    case GPT_CHANNEL_3:
    {
        /* Set the DMA compare callbacks */
        htim->hdma[GPT_DMA_ID_CC3]->XferCpltCallback = GPT_DMADelayPulseNCplt;
        htim->hdma[GPT_DMA_ID_CC3]->XferHalfCpltCallback = GPT_DMADelayPulseHalfCplt;

        /* Set the DMA error callback */
        htim->hdma[GPT_DMA_ID_CC3]->XferErrorCallback = GPT_DMAErrorCCxN ;

        /* Enable the DMA stream */
        if (HAL_DMA_Start_IT(htim->hdma[GPT_DMA_ID_CC3], (uint32_t)pData, (uint32_t)&htim->Instance->CCR3, Length) != HAL_OK)
        {
            /* Return error status */
            return HAL_ERROR;
        }
        /* Enable the TIM Capture/Compare 3 DMA request */
        __HAL_GPT_ENABLE_DMA(htim, GPT_DMA_CC3);
        break;
    }

    default:
        break;
    }

    /* Enable the complementary PWM output  */
    GPT_CCxNChannelCmd(htim->Instance, Channel, GPT_CCxN_ENABLE);

    /* Enable the Main Output */
    __HAL_GPT_MOE_ENABLE(htim);

    /* Enable the Peripheral, except in trigger mode where enable is automatically done with trigger */
    if (IS_GPT_SLAVE_INSTANCE(htim->Instance))
    {
        tmpsmcr = htim->Instance->SMCR & GPT_SMCR_SMS;
        if (!IS_GPT_SLAVEMODE_TRIGGER_ENABLED(tmpsmcr))
        {
            __HAL_GPT_ENABLE(htim);
        }
    }
    else
    {
        __HAL_GPT_ENABLE(htim);
    }

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  Stops the TIM PWM signal generation in DMA mode on the complementary
  *         output
  * @param  htim TIM handle
  * @param  Channel TIM Channel to be disabled
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  *            @arg GPT_CHANNEL_3: TIM Channel 3 selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop_DMA(GPT_HandleTypeDef *htim, uint32_t Channel)
{

    switch (Channel)
    {
    case GPT_CHANNEL_1:
    {
        /* Disable the TIM Capture/Compare 1 DMA request */
        __HAL_GPT_DISABLE_DMA(htim, GPT_DMA_CC1);
        (void)HAL_DMA_Abort_IT(htim->hdma[GPT_DMA_ID_CC1]);
        break;
    }

    case GPT_CHANNEL_2:
    {
        /* Disable the TIM Capture/Compare 2 DMA request */
        __HAL_GPT_DISABLE_DMA(htim, GPT_DMA_CC2);
        (void)HAL_DMA_Abort_IT(htim->hdma[GPT_DMA_ID_CC2]);
        break;
    }

    case GPT_CHANNEL_3:
    {
        /* Disable the TIM Capture/Compare 3 DMA request */
        __HAL_GPT_DISABLE_DMA(htim, GPT_DMA_CC3);
        (void)HAL_DMA_Abort_IT(htim->hdma[GPT_DMA_ID_CC3]);
        break;
    }

    default:
        break;
    }

    /* Disable the complementary PWM output */
    GPT_CCxNChannelCmd(htim->Instance, Channel, GPT_CCxN_DISABLE);

    /* Disable the Main Output */
    __HAL_GPT_MOE_DISABLE(htim);

    /* Disable the Peripheral */
    __HAL_GPT_DISABLE(htim);

    /* Set the TIM complementary channel state */
    GPT_CHANNEL_N_STATE_SET(htim, Channel, HAL_GPT_CHANNEL_STATE_READY);

    /* Return function status */
    return HAL_OK;
}

/**
  * @}
  */

/** @defgroup TIMEx_Exported_Functions_Group4 Extended Timer Complementary One Pulse functions
  * @brief    Timer Complementary One Pulse functions
  *
@verbatim
  ==============================================================================
                ##### Timer Complementary One Pulse functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
    (+) Start the Complementary One Pulse generation.
    (+) Stop the Complementary One Pulse.
    (+) Start the Complementary One Pulse and enable interrupts.
    (+) Stop the Complementary One Pulse and disable interrupts.

@endverbatim
  * @{
  */

/**
  * @brief  Starts the TIM One Pulse signal generation on the complementary
  *         output.
  * @note OutputChannel must match the pulse output channel chosen when calling
  *       @ref HAL_GPT_OnePulse_ConfigChannel().
  * @param  htim TIM One Pulse handle
  * @param  OutputChannel pulse output channel to enable
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Start(GPT_HandleTypeDef *htim, uint32_t OutputChannel)
{
    uint32_t input_channel = (OutputChannel == GPT_CHANNEL_1) ? GPT_CHANNEL_2 : GPT_CHANNEL_1;
    HAL_GPT_ChannelStateTypeDef channel_1_state = GPT_CHANNEL_STATE_GET(htim, GPT_CHANNEL_1);
    HAL_GPT_ChannelStateTypeDef channel_2_state = GPT_CHANNEL_STATE_GET(htim, GPT_CHANNEL_2);
    HAL_GPT_ChannelStateTypeDef complementary_channel_1_state = GPT_CHANNEL_N_STATE_GET(htim, GPT_CHANNEL_1);
    HAL_GPT_ChannelStateTypeDef complementary_channel_2_state = GPT_CHANNEL_N_STATE_GET(htim, GPT_CHANNEL_2);


    /* Check the TIM channels state */
    if ((channel_1_state != HAL_GPT_CHANNEL_STATE_READY)
            || (channel_2_state != HAL_GPT_CHANNEL_STATE_READY)
            || (complementary_channel_1_state != HAL_GPT_CHANNEL_STATE_READY)
            || (complementary_channel_2_state != HAL_GPT_CHANNEL_STATE_READY))
    {
        return HAL_ERROR;
    }

    /* Set the TIM channels state */
    GPT_CHANNEL_STATE_SET(htim, GPT_CHANNEL_1, HAL_GPT_CHANNEL_STATE_BUSY);
    GPT_CHANNEL_STATE_SET(htim, GPT_CHANNEL_2, HAL_GPT_CHANNEL_STATE_BUSY);
    GPT_CHANNEL_N_STATE_SET(htim, GPT_CHANNEL_1, HAL_GPT_CHANNEL_STATE_BUSY);
    GPT_CHANNEL_N_STATE_SET(htim, GPT_CHANNEL_2, HAL_GPT_CHANNEL_STATE_BUSY);

    /* Enable the complementary One Pulse output channel and the Input Capture channel */
    GPT_CCxNChannelCmd(htim->Instance, OutputChannel, GPT_CCxN_ENABLE);
    GPT_CCxChannelCmd(htim->Instance, input_channel, GPT_CCx_ENABLE);

    /* Enable the Main Output */
    __HAL_GPT_MOE_ENABLE(htim);

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  Stops the TIM One Pulse signal generation on the complementary
  *         output.
  * @note OutputChannel must match the pulse output channel chosen when calling
  *       @ref HAL_GPT_OnePulse_ConfigChannel().
  * @param  htim TIM One Pulse handle
  * @param  OutputChannel pulse output channel to disable
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Stop(GPT_HandleTypeDef *htim, uint32_t OutputChannel)
{
    uint32_t input_channel = (OutputChannel == GPT_CHANNEL_1) ? GPT_CHANNEL_2 : GPT_CHANNEL_1;


    /* Disable the complementary One Pulse output channel and the Input Capture channel */
    GPT_CCxNChannelCmd(htim->Instance, OutputChannel, GPT_CCxN_DISABLE);
    GPT_CCxChannelCmd(htim->Instance, input_channel, GPT_CCx_DISABLE);

    /* Disable the Main Output */
    __HAL_GPT_MOE_DISABLE(htim);

    /* Disable the Peripheral */
    __HAL_GPT_DISABLE(htim);

    /* Set the TIM  channels state */
    GPT_CHANNEL_STATE_SET(htim, GPT_CHANNEL_1, HAL_GPT_CHANNEL_STATE_READY);
    GPT_CHANNEL_STATE_SET(htim, GPT_CHANNEL_2, HAL_GPT_CHANNEL_STATE_READY);
    GPT_CHANNEL_N_STATE_SET(htim, GPT_CHANNEL_1, HAL_GPT_CHANNEL_STATE_READY);
    GPT_CHANNEL_N_STATE_SET(htim, GPT_CHANNEL_2, HAL_GPT_CHANNEL_STATE_READY);

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  Starts the TIM One Pulse signal generation in interrupt mode on the
  *         complementary channel.
  * @note OutputChannel must match the pulse output channel chosen when calling
  *       @ref HAL_GPT_OnePulse_ConfigChannel().
  * @param  htim TIM One Pulse handle
  * @param  OutputChannel pulse output channel to enable
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Start_IT(GPT_HandleTypeDef *htim, uint32_t OutputChannel)
{
    uint32_t input_channel = (OutputChannel == GPT_CHANNEL_1) ? GPT_CHANNEL_2 : GPT_CHANNEL_1;
    HAL_GPT_ChannelStateTypeDef channel_1_state = GPT_CHANNEL_STATE_GET(htim, GPT_CHANNEL_1);
    HAL_GPT_ChannelStateTypeDef channel_2_state = GPT_CHANNEL_STATE_GET(htim, GPT_CHANNEL_2);
    HAL_GPT_ChannelStateTypeDef complementary_channel_1_state = GPT_CHANNEL_N_STATE_GET(htim, GPT_CHANNEL_1);
    HAL_GPT_ChannelStateTypeDef complementary_channel_2_state = GPT_CHANNEL_N_STATE_GET(htim, GPT_CHANNEL_2);


    /* Check the TIM channels state */
    if ((channel_1_state != HAL_GPT_CHANNEL_STATE_READY)
            || (channel_2_state != HAL_GPT_CHANNEL_STATE_READY)
            || (complementary_channel_1_state != HAL_GPT_CHANNEL_STATE_READY)
            || (complementary_channel_2_state != HAL_GPT_CHANNEL_STATE_READY))
    {
        return HAL_ERROR;
    }

    /* Set the TIM channels state */
    GPT_CHANNEL_STATE_SET(htim, GPT_CHANNEL_1, HAL_GPT_CHANNEL_STATE_BUSY);
    GPT_CHANNEL_STATE_SET(htim, GPT_CHANNEL_2, HAL_GPT_CHANNEL_STATE_BUSY);
    GPT_CHANNEL_N_STATE_SET(htim, GPT_CHANNEL_1, HAL_GPT_CHANNEL_STATE_BUSY);
    GPT_CHANNEL_N_STATE_SET(htim, GPT_CHANNEL_2, HAL_GPT_CHANNEL_STATE_BUSY);

    /* Enable the TIM Capture/Compare 1 interrupt */
    __HAL_GPT_ENABLE_IT(htim, GPT_IT_CC1);

    /* Enable the TIM Capture/Compare 2 interrupt */
    __HAL_GPT_ENABLE_IT(htim, GPT_IT_CC2);

    /* Enable the complementary One Pulse output channel and the Input Capture channel */
    GPT_CCxNChannelCmd(htim->Instance, OutputChannel, GPT_CCxN_ENABLE);
    GPT_CCxChannelCmd(htim->Instance, input_channel, GPT_CCx_ENABLE);

    /* Enable the Main Output */
    __HAL_GPT_MOE_ENABLE(htim);

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  Stops the TIM One Pulse signal generation in interrupt mode on the
  *         complementary channel.
  * @note OutputChannel must match the pulse output channel chosen when calling
  *       @ref HAL_GPT_OnePulse_ConfigChannel().
  * @param  htim TIM One Pulse handle
  * @param  OutputChannel pulse output channel to disable
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Stop_IT(GPT_HandleTypeDef *htim, uint32_t OutputChannel)
{
    uint32_t input_channel = (OutputChannel == GPT_CHANNEL_1) ? GPT_CHANNEL_2 : GPT_CHANNEL_1;


    /* Disable the TIM Capture/Compare 1 interrupt */
    __HAL_GPT_DISABLE_IT(htim, GPT_IT_CC1);

    /* Disable the TIM Capture/Compare 2 interrupt */
    __HAL_GPT_DISABLE_IT(htim, GPT_IT_CC2);

    /* Disable the complementary One Pulse output channel and the Input Capture channel */
    GPT_CCxNChannelCmd(htim->Instance, OutputChannel, GPT_CCxN_DISABLE);
    GPT_CCxChannelCmd(htim->Instance, input_channel, GPT_CCx_DISABLE);

    /* Disable the Main Output */
    __HAL_GPT_MOE_DISABLE(htim);

    /* Disable the Peripheral */
    __HAL_GPT_DISABLE(htim);

    /* Set the TIM  channels state */
    GPT_CHANNEL_STATE_SET(htim, GPT_CHANNEL_1, HAL_GPT_CHANNEL_STATE_READY);
    GPT_CHANNEL_STATE_SET(htim, GPT_CHANNEL_2, HAL_GPT_CHANNEL_STATE_READY);
    GPT_CHANNEL_N_STATE_SET(htim, GPT_CHANNEL_1, HAL_GPT_CHANNEL_STATE_READY);
    GPT_CHANNEL_N_STATE_SET(htim, GPT_CHANNEL_2, HAL_GPT_CHANNEL_STATE_READY);

    /* Return function status */
    return HAL_OK;
}

/**
  * @}
  */

/** @defgroup TIMEx_Exported_Functions_Group5 Extended Peripheral Control functions
  * @brief    Peripheral Control functions
  *
@verbatim
  ==============================================================================
                    ##### Peripheral Control functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
      (+) Configure the commutation event in case of use of the Hall sensor interface.
      (+) Configure Output channels for OC and PWM mode.

      (+) Configure Complementary channels, break features and dead time.
      (+) Configure Master synchronization.
      (+) Configure timer remapping capabilities.
      (+) Select timer input source.
      (+) Enable or disable channel grouping.

@endverbatim
  * @{
  */

/**
  * @brief  Configure the TIM commutation event sequence.
  * @note  This function is mandatory to use the commutation event in order to
  *        update the configuration at each commutation detection on the TRGI input of the Timer,
  *        the typical use of this feature is with the use of another Timer(interface Timer)
  *        configured in Hall sensor interface, this interface Timer will generate the
  *        commutation at its TRGO output (connected to Timer used in this function) each time
  *        the TI1 of the Interface Timer detect a commutation at its input TI1.
  * @param  htim TIM handle
  * @param  InputTrigger the Internal trigger corresponding to the Timer Interfacing with the Hall sensor
  *          This parameter can be one of the following values:
  *            @arg TIM_TS_ITR0: Internal trigger 0 selected
  *            @arg TIM_TS_ITR1: Internal trigger 1 selected
  *            @arg TIM_TS_ITR2: Internal trigger 2 selected
  *            @arg TIM_TS_ITR3: Internal trigger 3 selected
  *            @arg TIM_TS_ITR12: Internal trigger 12 selected (*)
  *            @arg TIM_TS_ITR13: Internal trigger 13 selected (*)
  *            @arg TIM_TS_NONE: No trigger is needed
  * @param  CommutationSource the Commutation Event source
  *          This parameter can be one of the following values:
  *            @arg TIM_COMMUTATION_TRGI: Commutation source is the TRGI of the Interface Timer
  *            @arg TIM_COMMUTATION_SOFTWARE:  Commutation source is set by software using the COMG bit
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_TIMEx_ConfigCommutEvent(GPT_HandleTypeDef *htim, uint32_t  InputTrigger,
        uint32_t  CommutationSource)
{
    __HAL_LOCK(htim);

    if ((InputTrigger == GPT_TS_ITR0)  || (InputTrigger == GPT_TS_ITR1) ||
            (InputTrigger == GPT_TS_ITR2)  || (InputTrigger == GPT_TS_ITR3))
    {
        /* Select the Input trigger */
        htim->Instance->SMCR &= ~GPT_SMCR_TS;
        htim->Instance->SMCR |= InputTrigger;
    }

    /* Select the Capture Compare preload feature */
    htim->Instance->CR2 |= ATIM_CR2_CCPC;
    /* Select the Commutation event source */
    htim->Instance->CR2 &= ~GPT_CR2_CCUS;
    htim->Instance->CR2 |= CommutationSource;

    /* Disable Commutation Interrupt */
    __HAL_GPT_DISABLE_IT(htim, GPT_IT_COM);

    /* Disable Commutation DMA request */
    __HAL_GPT_DISABLE_DMA(htim, GPT_DMA_COM);

    __HAL_UNLOCK(htim);

    return HAL_OK;
}

/**
  * @brief  Configure the TIM commutation event sequence with interrupt.
  * @note  This function is mandatory to use the commutation event in order to
  *        update the configuration at each commutation detection on the TRGI input of the Timer,
  *        the typical use of this feature is with the use of another Timer(interface Timer)
  *        configured in Hall sensor interface, this interface Timer will generate the
  *        commutation at its TRGO output (connected to Timer used in this function) each time
  *        the TI1 of the Interface Timer detect a commutation at its input TI1.
  * @param  htim TIM handle
  * @param  InputTrigger the Internal trigger corresponding to the Timer Interfacing with the Hall sensor
  *          This parameter can be one of the following values:
  *            @arg TIM_TS_ITR0: Internal trigger 0 selected
  *            @arg TIM_TS_ITR1: Internal trigger 1 selected
  *            @arg TIM_TS_ITR2: Internal trigger 2 selected
  *            @arg TIM_TS_ITR3: Internal trigger 3 selected
  *            @arg TIM_TS_ITR2: Internal trigger 12 selected (*)
  *            @arg TIM_TS_ITR3: Internal trigger 13 selected (*)
  *            @arg TIM_TS_NONE: No trigger is needed
  * @param  CommutationSource the Commutation Event source
  *          This parameter can be one of the following values:
  *            @arg TIM_COMMUTATION_TRGI: Commutation source is the TRGI of the Interface Timer
  *            @arg TIM_COMMUTATION_SOFTWARE:  Commutation source is set by software using the COMG bit
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_TIMEx_ConfigCommutEvent_IT(GPT_HandleTypeDef *htim, uint32_t  InputTrigger,
        uint32_t  CommutationSource)
{
    __HAL_LOCK(htim);

    if ((InputTrigger == GPT_TS_ITR0)  || (InputTrigger == GPT_TS_ITR1) ||
            (InputTrigger == GPT_TS_ITR2)  || (InputTrigger == GPT_TS_ITR3))
    {
        /* Select the Input trigger */
        htim->Instance->SMCR &= ~GPT_SMCR_TS;
        htim->Instance->SMCR |= InputTrigger;
    }

    /* Select the Capture Compare preload feature */
    htim->Instance->CR2 |= ATIM_CR2_CCPC;
    /* Select the Commutation event source */
    htim->Instance->CR2 &= ~GPT_CR2_CCUS;
    htim->Instance->CR2 |= CommutationSource;

    /* Disable Commutation DMA request */
    __HAL_GPT_DISABLE_DMA(htim, GPT_DMA_COM);

    /* Enable the Commutation Interrupt */
    __HAL_GPT_ENABLE_IT(htim, GPT_IT_COM);

    __HAL_UNLOCK(htim);

    return HAL_OK;
}

/**
  * @brief  Configure the TIM commutation event sequence with DMA.
  * @note  This function is mandatory to use the commutation event in order to
  *        update the configuration at each commutation detection on the TRGI input of the Timer,
  *        the typical use of this feature is with the use of another Timer(interface Timer)
  *        configured in Hall sensor interface, this interface Timer will generate the
  *        commutation at its TRGO output (connected to Timer used in this function) each time
  *        the TI1 of the Interface Timer detect a commutation at its input TI1.
  * @note  The user should configure the DMA in his own software, in This function only the COMDE bit is set
  * @param  htim TIM handle
  * @param  InputTrigger the Internal trigger corresponding to the Timer Interfacing with the Hall sensor
  *          This parameter can be one of the following values:
  *            @arg TIM_TS_ITR0: Internal trigger 0 selected
  *            @arg TIM_TS_ITR1: Internal trigger 1 selected
  *            @arg TIM_TS_ITR2: Internal trigger 2 selected
  *            @arg TIM_TS_ITR3: Internal trigger 3 selected
  *            @arg TIM_TS_ITR2: Internal trigger 12 selected (*)
  *            @arg TIM_TS_ITR3: Internal trigger 13 selected (*)
  *            @arg TIM_TS_NONE: No trigger is needed
  *
  *         (*)  Value not defined in all devices.
  *
  * @param  CommutationSource the Commutation Event source
  *          This parameter can be one of the following values:
  *            @arg TIM_COMMUTATION_TRGI: Commutation source is the TRGI of the Interface Timer
  *            @arg TIM_COMMUTATION_SOFTWARE:  Commutation source is set by software using the COMG bit
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_TIMEx_ConfigCommutEvent_DMA(GPT_HandleTypeDef *htim, uint32_t  InputTrigger,
        uint32_t  CommutationSource)
{

    __HAL_LOCK(htim);

    if ((InputTrigger == GPT_TS_ITR0)  || (InputTrigger == GPT_TS_ITR1) ||
            (InputTrigger == GPT_TS_ITR2)  || (InputTrigger == GPT_TS_ITR3))
    {
        /* Select the Input trigger */
        htim->Instance->SMCR &= ~GPT_SMCR_TS;
        htim->Instance->SMCR |= InputTrigger;
    }

    /* Select the Capture Compare preload feature */
    htim->Instance->CR2 |= ATIM_CR2_CCPC;
    /* Select the Commutation event source */
    htim->Instance->CR2 &= ~GPT_CR2_CCUS;
    htim->Instance->CR2 |= CommutationSource;

    /* Enable the Commutation DMA Request */
    /* Set the DMA Commutation Callback */
    htim->hdma[GPT_DMA_ID_COMMUTATION]->XferCpltCallback = TIMEx_DMACommutationCplt;
    htim->hdma[GPT_DMA_ID_COMMUTATION]->XferHalfCpltCallback = TIMEx_DMACommutationHalfCplt;
    /* Set the DMA error callback */
    htim->hdma[GPT_DMA_ID_COMMUTATION]->XferErrorCallback = GPT_DMAError;

    /* Disable Commutation Interrupt */
    __HAL_GPT_DISABLE_IT(htim, GPT_IT_COM);

    /* Enable the Commutation DMA Request */
    __HAL_GPT_ENABLE_DMA(htim, GPT_DMA_COM);

    __HAL_UNLOCK(htim);

    return HAL_OK;
}

/**
  * @brief  Configures the TIM in master mode.
  * @param  htim TIM handle.
  * @param  sMasterConfig pointer to a TIMEx_MasterConfigTypeDef structure that
  *         contains the selected trigger output (TRGO) and the Master/Slave
  *         mode.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_TIMEx_MasterConfigSynchronization(GPT_HandleTypeDef *htim,
        TIMEx_MasterConfigTypeDef *sMasterConfig)
{
    uint32_t tmpcr2;
    uint32_t tmpsmcr;

    /* Check input state */
    __HAL_LOCK(htim);

    /* Change the handler state */
    htim->State = HAL_GPT_STATE_BUSY;

    /* Get the TIMx CR2 register value */
    tmpcr2 = htim->Instance->CR2;

    /* Get the TIMx SMCR register value */
    tmpsmcr = htim->Instance->SMCR;

    /* If the timer supports ADC synchronization through TRGO2, set the master mode selection 2 */
    {
        /* Clear the MMS2 bits */
        tmpcr2 &= ~ATIM_CR2_MMS2;
        /* Select the TRGO2 source*/
        tmpcr2 |= sMasterConfig->MasterOutputTrigger2;
    }

    /* Reset the MMS Bits */
    tmpcr2 &= ~ATIM_CR2_MMS;
    /* Select the TRGO source */
    tmpcr2 |=  sMasterConfig->MasterOutputTrigger;

    /* Update TIMx CR2 */
    htim->Instance->CR2 = tmpcr2;

    if (IS_GPT_SLAVE_INSTANCE(htim->Instance))
    {
        /* Reset the MSM Bit */
        tmpsmcr &= ~ATIM_SMCR_MSM;
        /* Set master mode */
        tmpsmcr |= sMasterConfig->MasterSlaveMode;

        /* Update TIMx SMCR */
        htim->Instance->SMCR = tmpsmcr;
    }

    /* Change the htim state */
    htim->State = HAL_GPT_STATE_READY;

    __HAL_UNLOCK(htim);

    return HAL_OK;
}

/**
  * @brief  Configures the Break feature, dead time, Lock level, OSSI/OSSR State
  *         and the AOE(automatic output enable).
  * @param  htim TIM handle
  * @param  sBreakDeadTimeConfig pointer to a TIM_ConfigBreakDeadConfigTypeDef structure that
  *         contains the BDTR Register configuration  information for the TIM peripheral.
  * @note   Interrupts can be generated when an active level is detected on the
  *         break input, the break 2 input or the system break input. Break
  *         interrupt can be enabled by calling the @ref __HAL_GPT_ENABLE_IT macro.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_TIMEx_ConfigBreakDeadTime(GPT_HandleTypeDef *htim,
        TIMEx_BreakDeadTimeConfigTypeDef *sBreakDeadTimeConfig)
{
    ATIM_TypeDef *instance = (ATIM_TypeDef *)htim->Instance;

    /* Keep this variable initialized to 0 as it is used to configure BDTR register */
    uint32_t tmpbdtr = 0U;

    /* Check input state */
    __HAL_LOCK(htim);

    /* Set the Lock level, the Break enable Bit and the Polarity, the OSSR State,
       the OSSI State, the dead time value and the Automatic Output Enable Bit */

    /* Set the BDTR bits */
    MODIFY_REG(tmpbdtr, ATIM_BDTR_DTG, ((sBreakDeadTimeConfig->DeadTime & 0x3FFUL) << ATIM_BDTR_DTG_Pos));
#if 0
    MODIFY_REG(tmpbdtr, ATIM_BDTR_LOCK, sBreakDeadTimeConfig->LockLevel);
#endif
    MODIFY_REG(tmpbdtr, ATIM_BDTR_OSSI, ((sBreakDeadTimeConfig->OffStateIDLEMode & 0x1UL) << ATIM_BDTR_OSSI_Pos));
    MODIFY_REG(tmpbdtr, ATIM_BDTR_OSSR, ((sBreakDeadTimeConfig->OffStateRunMode & 0x1UL) << ATIM_BDTR_OSSR_Pos));
    MODIFY_REG(tmpbdtr, ATIM_BDTR_AOE, ((sBreakDeadTimeConfig->AutomaticOutput & 0x1UL) << ATIM_BDTR_AOE_Pos));
    MODIFY_REG(tmpbdtr, ATIM_BDTR_DTPSC, ((sBreakDeadTimeConfig->DeadTimePsc & 0x1UL) << ATIM_BDTR_DTPSC_Pos));
    /* Set the BREAK input related BDTR bits */
    MODIFY_REG(tmpbdtr, ATIM_BDTR_BKF, ((sBreakDeadTimeConfig->BreakFilter & 0xFUL) << ATIM_BDTR_BKF_Pos));
    MODIFY_REG(tmpbdtr, ATIM_BDTR_BKE, ((sBreakDeadTimeConfig->BreakState & 0x1UL) << ATIM_BDTR_BKE_Pos));
    MODIFY_REG(tmpbdtr, ATIM_BDTR_BKP, ((sBreakDeadTimeConfig->BreakPolarity & 0x1UL) << ATIM_BDTR_BKP_Pos));
    /* Set the BREAK2 input related BDTR bits */
    MODIFY_REG(tmpbdtr, ATIM_BDTR_BK2F, ((sBreakDeadTimeConfig->Break2Filter & 0xFUL) << ATIM_BDTR_BK2F_Pos));
    MODIFY_REG(tmpbdtr, ATIM_BDTR_BK2E, ((sBreakDeadTimeConfig->Break2State & 0x1UL) << ATIM_BDTR_BK2E_Pos));
    MODIFY_REG(tmpbdtr, ATIM_BDTR_BK2P, ((sBreakDeadTimeConfig->Break2Polarity & 0x1UL) << ATIM_BDTR_BK2P_Pos));

    /* Set TIMx_BDTR */
    instance->BDTR = tmpbdtr;

    __HAL_UNLOCK(htim);

    return HAL_OK;
}

#if defined(GPT_BREAK_INPUT_SUPPORT)

/**
  * @brief  Configures the break input source.
  * @param  htim TIM handle.
  * @param  BreakInput Break input to configure
  *          This parameter can be one of the following values:
  *            @arg TIM_BREAKINPUT_BRK: Timer break input
  *            @arg TIM_BREAKINPUT_BRK2: Timer break 2 input
  * @param  sBreakInputConfig Break input source configuration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_TIMEx_ConfigBreakInput(GPT_HandleTypeDef *htim,
        uint32_t BreakInput,
        TIMEx_BreakInputConfigTypeDef *sBreakInputConfig)

{
    uint32_t tmporx;
    uint32_t bkin_enable_mask;
    uint32_t bkin_polarity_mask;
    uint32_t bkin_enable_bitpos;
    uint32_t bkin_polarity_bitpos;
    ATIM_TypeDef *instance = (ATIM_TypeDef *)htim->Instance;

    /* Check input state */
    __HAL_LOCK(htim);

    switch (sBreakInputConfig->Source)
    {
    case TIM_BREAKINPUTSOURCE_BKIN:
    {
        bkin_enable_mask = ATIM_AF1_BKINE;
        bkin_enable_bitpos = ATIM_AF1_BKINE_Pos;
        bkin_polarity_mask = ATIM_AF1_BKINP;
        bkin_polarity_bitpos = ATIM_AF1_BKINP_Pos;
        break;
    }
    case TIM_BREAKINPUTSOURCE_COMP1:
    {
        bkin_enable_mask = ATIM_AF1_BKCMP1E;
        bkin_enable_bitpos = ATIM_AF1_BKCMP1E_Pos;
        bkin_polarity_mask = ATIM_AF1_BKCMP1P;
        bkin_polarity_bitpos = ATIM_AF1_BKCMP1P_Pos;
        break;
    }
    case TIM_BREAKINPUTSOURCE_COMP2:
    {
        bkin_enable_mask = ATIM_AF1_BKCMP2E;
        bkin_enable_bitpos = ATIM_AF1_BKCMP2E_Pos;
        bkin_polarity_mask = ATIM_AF1_BKCMP2P;
        bkin_polarity_bitpos = ATIM_AF1_BKCMP2P_Pos;
        break;
    }
#if 0
    case TIM_BREAKINPUTSOURCE_DFSDM1:
    {
        bkin_enable_mask = ATIM_AF1_BKDF1BK0E;
        bkin_enable_bitpos = 8U;
        bkin_polarity_mask = 0U;
        bkin_polarity_bitpos = 0U;
        break;
    }
#endif
    default:
    {
        bkin_enable_mask = 0U;
        bkin_polarity_mask = 0U;
        bkin_enable_bitpos = 0U;
        bkin_polarity_bitpos = 0U;
        break;
    }
    }

    switch (BreakInput)
    {
    case TIM_BREAKINPUT_BRK:
    {
        /* Get the TIMx_AF1 register value */
        tmporx = instance->AF1;

        /* Enable the break input */
        tmporx &= ~bkin_enable_mask;
        tmporx |= (sBreakInputConfig->Enable << bkin_enable_bitpos) & bkin_enable_mask;

        /* Set the break input polarity */
        if (sBreakInputConfig->Source != TIM_BREAKINPUTSOURCE_DFSDM1)
        {
            tmporx &= ~bkin_polarity_mask;
            tmporx |= (sBreakInputConfig->Polarity << bkin_polarity_bitpos) & bkin_polarity_mask;
        }

        /* Set TIMx_AF1 */
        instance->AF1 = tmporx;
        break;
    }
    case TIM_BREAKINPUT_BRK2:
    {
        /* Get the TIMx_AF2 register value */
        tmporx = instance->AF2;

        /* Enable the break input */
        tmporx &= ~bkin_enable_mask;
        tmporx |= (sBreakInputConfig->Enable << bkin_enable_bitpos) & bkin_enable_mask;

        /* Set the break input polarity */
        if (sBreakInputConfig->Source != TIM_BREAKINPUTSOURCE_DFSDM1)
        {
            tmporx &= ~bkin_polarity_mask;
            tmporx |= (sBreakInputConfig->Polarity << bkin_polarity_bitpos) & bkin_polarity_mask;
        }

        /* Set TIMx_AF2 */
        instance->AF2 = tmporx;
        break;
    }
    default:
        break;
    }

    __HAL_UNLOCK(htim);

    return HAL_OK;
}
#endif /*TIM_BREAK_INPUT_SUPPORT */

/**
  * @brief  Configures the TIMx Remapping input capabilities.
  * @param  htim TIM handle.
  * @param  Remap specifies the TIM remapping source.
  *         For TIM1, the parameter is one of the following values:
  *            @arg TIM_TIM1_ETR_GPIO:               TIM1_ETR is connected to GPIO
  *            @arg TIM_TIM1_ETR_COMP1:              TIM1_ETR is connected to COMP1 output
  *            @arg TIM_TIM1_ETR_COMP2:              TIM1_ETR is connected to COMP2 output
  *            @arg TIM_TIM1_ETR_ADC1_AWD1:          TIM1_ETR is connected to ADC1 AWD1
  *            @arg TIM_TIM1_ETR_ADC1_AWD2:          TIM1_ETR is connected to ADC1 AWD2
  *            @arg TIM_TIM1_ETR_ADC1_AWD3:          TIM1_ETR is connected to ADC1 AWD3
  *            @arg TIM_TIM1_ETR_ADC3_AWD1:          TIM1_ETR is connected to ADC3 AWD1
  *            @arg TIM_TIM1_ETR_ADC3_AWD2:          TIM1_ETR is connected to ADC3 AWD2
  *            @arg TIM_TIM1_ETR_ADC3_AWD3:          TIM1_ETR is connected to ADC3 AWD3
  *
  *         For TIM2, the parameter is one of the following values:
  *            @arg TIM_TIM2_ETR_GPIO:               TIM2_ETR is connected to GPIO
  *            @arg TIM_TIM2_ETR_COMP1:              TIM2_ETR is connected to COMP1 output
  *            @arg TIM_TIM2_ETR_COMP2:              TIM2_ETR is connected to COMP2 output
  *            @arg TIM_TIM2_ETR_LSE:                TIM2_ETR is connected to LSE
  *            @arg TIM_TIM2_ETR_SAI1_FSA:           TIM2_ETR is connected to SAI1 FS_A
  *            @arg TIM_TIM2_ETR_SAI1_FSB:           TIM2_ETR is connected to SAI1 FS_B
  *
  *         For TIM3, the parameter is one of the following values:
  *            @arg TIM_TIM3_ETR_GPIO:               TIM3_ETR is connected to GPIO
  *            @arg TIM_TIM3_ETR_COMP1:              TIM3_ETR is connected to COMP1 output
  *
  *         For TIM5, the parameter is one of the following values:
  *            @arg TIM_TIM5_ETR_GPIO:               TIM5_ETR is connected to GPIO
  *            @arg TIM_TIM5_ETR_SAI2_FSA:           TIM5_ETR is connected to SAI2 FS_A (*)
  *            @arg TIM_TIM5_ETR_SAI2_FSB:           TIM5_ETR is connected to SAI2 FS_B (*)
  *            @arg TIM_TIM5_ETR_SAI4_FSA:           TIM5_ETR is connected to SAI2 FS_A (*)
  *            @arg TIM_TIM5_ETR_SAI4_FSB:           TIM5_ETR is connected to SAI2 FS_B (*)
  *
  *         For TIM8, the parameter is one of the following values:
  *            @arg TIM_TIM8_ETR_GPIO:               TIM8_ETR is connected to GPIO
  *            @arg TIM_TIM8_ETR_COMP1:              TIM8_ETR is connected to COMP1 output
  *            @arg TIM_TIM8_ETR_COMP2:              TIM8_ETR is connected to COMP2 output
  *            @arg TIM_TIM8_ETR_ADC2_AWD1:          TIM8_ETR is connected to ADC2 AWD1
  *            @arg TIM_TIM8_ETR_ADC2_AWD2:          TIM8_ETR is connected to ADC2 AWD2
  *            @arg TIM_TIM8_ETR_ADC2_AWD3:          TIM8_ETR is connected to ADC2 AWD3
  *            @arg TIM_TIM8_ETR_ADC3_AWD1:          TIM8_ETR is connected to ADC3 AWD1
  *            @arg TIM_TIM8_ETR_ADC3_AWD2:          TIM8_ETR is connected to ADC3 AWD2
  *            @arg TIM_TIM8_ETR_ADC3_AWD3:          TIM8_ETR is connected to ADC3 AWD3
  *
  *         For TIM23, the parameter is one of the following values: (*)
  *            @arg TIM_TIM23_ETR_GPIO               TIM23_ETR is connected to GPIO
  *            @arg TIM_TIM23_ETR_COMP1              TIM23_ETR is connected to COMP1 output
  *            @arg TIM_TIM23_ETR_COMP2              TIM23_ETR is connected to COMP2 output
  *
  *         For TIM24, the parameter is one of the following values: (*)
  *           @arg TIM_TIM24_ETR_GPIO                TIM24_ETR is connected to GPIO
  *           @arg TIM_TIM24_ETR_SAI4_FSA            TIM24_ETR is connected to SAI4 FS_A
  *           @arg TIM_TIM24_ETR_SAI4_FSB            TIM24_ETR is connected to SAI4 FS_B
  *           @arg TIM_TIM24_ETR_SAI1_FSA            TIM24_ETR is connected to SAI1 FS_A
  *           @arg TIM_TIM24_ETR_SAI1_FSB            TIM24_ETR is connected to SAI1 FS_B
  *
  *         (*)  Value not defined in all devices.
  *
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_TIMEx_RemapConfig(GPT_HandleTypeDef *htim, uint32_t Remap)
{
    ATIM_TypeDef *instance = (ATIM_TypeDef *)htim->Instance;

    __HAL_LOCK(htim);

    MODIFY_REG(instance->AF1, ATIM_AF1_ETRSEL_Msk, Remap);

    __HAL_UNLOCK(htim);

    return HAL_OK;
}

/**
  * @brief  Group channel 5 and channel 1, 2 or 3
  * @param  htim TIM handle.
  * @param  Channels specifies the reference signal(s) the OC5REF is combined with.
  *         This parameter can be any combination of the following values:
  *         TIM_GROUPCH5_NONE: No effect of OC5REF on OC1REFC, OC2REFC and OC3REFC
  *         TIM_GROUPCH5_OC1REFC: OC1REFC is the logical AND of OC1REFC and OC5REF
  *         TIM_GROUPCH5_OC2REFC: OC2REFC is the logical AND of OC2REFC and OC5REF
  *         TIM_GROUPCH5_OC3REFC: OC3REFC is the logical AND of OC3REFC and OC5REF
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_TIMEx_GroupChannel5(GPT_HandleTypeDef *htim, uint32_t Channels)
{

    ATIM_TypeDef *instance = (ATIM_TypeDef *)htim->Instance;
    /* Process Locked */
    __HAL_LOCK(htim);

    htim->State = HAL_GPT_STATE_BUSY;

#if 0
    /* Clear GC5Cx bit fields */
    instance->CCR5 &= ~(TIM_CCR5_GC5C3 | TIM_CCR5_GC5C2 | TIM_CCR5_GC5C1);
#endif

    /* Set GC5Cx bit fields */
    instance->CCR5 |= Channels;

    /* Change the htim state */
    htim->State = HAL_GPT_STATE_READY;

    __HAL_UNLOCK(htim);

    return HAL_OK;
}

/**
  * @}
  */

/** @defgroup TIMEx_Exported_Functions_Group6 Extended Callbacks functions
  * @brief    Extended Callbacks functions
  *
@verbatim
  ==============================================================================
                    ##### Extended Callbacks functions #####
  ==============================================================================
  [..]
    This section provides Extended TIM callback functions:
    (+) Timer Commutation callback
    (+) Timer Break callback

@endverbatim
  * @{
  */

/**
  * @brief  Hall commutation changed callback in non-blocking mode
  * @param  htim TIM handle
  * @retval None
  */
__weak void HAL_TIMEx_CommutCallback(GPT_HandleTypeDef *htim)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(htim);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_TIMEx_CommutCallback could be implemented in the user file
     */
}
/**
  * @brief  Hall commutation changed half complete callback in non-blocking mode
  * @param  htim TIM handle
  * @retval None
  */
__weak void HAL_TIMEx_CommutHalfCpltCallback(GPT_HandleTypeDef *htim)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(htim);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_TIMEx_CommutHalfCpltCallback could be implemented in the user file
     */
}

/**
  * @brief  Hall Break detection callback in non-blocking mode
  * @param  htim TIM handle
  * @retval None
  */
__weak void HAL_TIMEx_BreakCallback(GPT_HandleTypeDef *htim)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(htim);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_TIMEx_BreakCallback could be implemented in the user file
     */
}

/**
  * @brief  Hall Break2 detection callback in non blocking mode
  * @param  htim: TIM handle
  * @retval None
  */
__weak void HAL_TIMEx_Break2Callback(GPT_HandleTypeDef *htim)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(htim);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_TIMEx_Break2Callback could be implemented in the user file
     */
}
/**
  * @}
  */

/** @defgroup TIMEx_Exported_Functions_Group7 Extended Peripheral State functions
  * @brief    Extended Peripheral State functions
  *
@verbatim
  ==============================================================================
                ##### Extended Peripheral State functions #####
  ==============================================================================
  [..]
    This subsection permits to get in run-time the status of the peripheral
    and the data flow.

@endverbatim
  * @{
  */

/**
  * @brief  Return the TIM Hall Sensor interface handle state.
  * @param  htim TIM Hall Sensor handle
  * @retval HAL state
  */
HAL_GPT_StateTypeDef HAL_TIMEx_HallSensor_GetState(GPT_HandleTypeDef *htim)
{
    return htim->State;
}

/**
  * @brief  Return actual state of the TIM complementary channel.
  * @param  htim TIM handle
  * @param  ChannelN TIM Complementary channel
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1
  *            @arg GPT_CHANNEL_2: TIM Channel 2
  *            @arg GPT_CHANNEL_3: TIM Channel 3
  * @retval TIM Complementary channel state
  */
HAL_GPT_ChannelStateTypeDef HAL_TIMEx_GetChannelNState(GPT_HandleTypeDef *htim,  uint32_t ChannelN)
{
    HAL_GPT_ChannelStateTypeDef channel_state;


    channel_state = GPT_CHANNEL_N_STATE_GET(htim, ChannelN);

    return channel_state;
}
/**
  * @}
  */

/**
  * @}
  */

/* Private functions ---------------------------------------------------------*/
/** @defgroup TIMEx_Private_Functions TIMEx Private Functions
  * @{
  */

/**
  * @brief  TIM DMA Commutation callback.
  * @param  hdma pointer to DMA handle.
  * @retval None
  */
void TIMEx_DMACommutationCplt(DMA_HandleTypeDef *hdma)
{
    GPT_HandleTypeDef *htim = (GPT_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

    /* Change the htim state */
    htim->State = HAL_GPT_STATE_READY;

    HAL_TIMEx_CommutCallback(htim);
}

/**
  * @brief  TIM DMA Commutation half complete callback.
  * @param  hdma pointer to DMA handle.
  * @retval None
  */
void TIMEx_DMACommutationHalfCplt(DMA_HandleTypeDef *hdma)
{
    GPT_HandleTypeDef *htim = (GPT_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

    /* Change the htim state */
    htim->State = HAL_GPT_STATE_READY;

    HAL_TIMEx_CommutHalfCpltCallback(htim);
}


/**
  * @brief  TIM DMA Delay Pulse complete callback (complementary channel).
  * @param  hdma pointer to DMA handle.
  * @retval None
  */
static void GPT_DMADelayPulseNCplt(DMA_HandleTypeDef *hdma)
{
    GPT_HandleTypeDef *htim = (GPT_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

    if (hdma == htim->hdma[GPT_DMA_ID_CC1])
    {
        htim->Channel = HAL_GPT_ACTIVE_CHANNEL_1;

        if (hdma->Init.Mode == DMA_NORMAL)
        {
            GPT_CHANNEL_N_STATE_SET(htim, GPT_CHANNEL_1, HAL_GPT_CHANNEL_STATE_READY);
        }
    }
    else if (hdma == htim->hdma[GPT_DMA_ID_CC2])
    {
        htim->Channel = HAL_GPT_ACTIVE_CHANNEL_2;

        if (hdma->Init.Mode == DMA_NORMAL)
        {
            GPT_CHANNEL_N_STATE_SET(htim, GPT_CHANNEL_2, HAL_GPT_CHANNEL_STATE_READY);
        }
    }
    else if (hdma == htim->hdma[GPT_DMA_ID_CC3])
    {
        htim->Channel = HAL_GPT_ACTIVE_CHANNEL_3;

        if (hdma->Init.Mode == DMA_NORMAL)
        {
            GPT_CHANNEL_N_STATE_SET(htim, GPT_CHANNEL_3, HAL_GPT_CHANNEL_STATE_READY);
        }
    }
    else if (hdma == htim->hdma[GPT_DMA_ID_CC4])
    {
        htim->Channel = HAL_GPT_ACTIVE_CHANNEL_4;

        if (hdma->Init.Mode == DMA_NORMAL)
        {
            GPT_CHANNEL_N_STATE_SET(htim, GPT_CHANNEL_4, HAL_GPT_CHANNEL_STATE_READY);
        }
    }
    else
    {
        /* nothing to do */
    }

    HAL_GPT_PWM_PulseFinishedCallback(htim);

    htim->Channel = HAL_GPT_ACTIVE_CHANNEL_CLEARED;
}

/**
  * @brief  TIM DMA Delay Pulse half complete callback.
  * @param  hdma pointer to DMA handle.
  * @retval None
  */
static void GPT_DMADelayPulseHalfCplt(DMA_HandleTypeDef *hdma)
{
    GPT_HandleTypeDef *htim = (GPT_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

    if (hdma == htim->hdma[GPT_DMA_ID_CC1])
    {
        htim->Channel = HAL_GPT_ACTIVE_CHANNEL_1;
    }
    else if (hdma == htim->hdma[GPT_DMA_ID_CC2])
    {
        htim->Channel = HAL_GPT_ACTIVE_CHANNEL_2;
    }
    else if (hdma == htim->hdma[GPT_DMA_ID_CC3])
    {
        htim->Channel = HAL_GPT_ACTIVE_CHANNEL_3;
    }
    else if (hdma == htim->hdma[GPT_DMA_ID_CC4])
    {
        htim->Channel = HAL_GPT_ACTIVE_CHANNEL_4;
    }
    else
    {
        /* nothing to do */
    }

    HAL_GPT_PWM_PulseFinishedHalfCpltCallback(htim);

    htim->Channel = HAL_GPT_ACTIVE_CHANNEL_CLEARED;
}

/**
  * @brief  TIM DMA error callback (complementary channel)
  * @param  hdma pointer to DMA handle.
  * @retval None
  */
static void GPT_DMAErrorCCxN(DMA_HandleTypeDef *hdma)
{
    GPT_HandleTypeDef *htim = (GPT_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

    if (hdma == htim->hdma[GPT_DMA_ID_CC1])
    {
        htim->Channel = HAL_GPT_ACTIVE_CHANNEL_1;
        GPT_CHANNEL_N_STATE_SET(htim, GPT_CHANNEL_1, HAL_GPT_CHANNEL_STATE_READY);
    }
    else if (hdma == htim->hdma[GPT_DMA_ID_CC2])
    {
        htim->Channel = HAL_GPT_ACTIVE_CHANNEL_2;
        GPT_CHANNEL_N_STATE_SET(htim, GPT_CHANNEL_2, HAL_GPT_CHANNEL_STATE_READY);
    }
    else if (hdma == htim->hdma[GPT_DMA_ID_CC3])
    {
        htim->Channel = HAL_GPT_ACTIVE_CHANNEL_3;
        GPT_CHANNEL_N_STATE_SET(htim, GPT_CHANNEL_3, HAL_GPT_CHANNEL_STATE_READY);
    }
    else
    {
        /* nothing to do */
    }

    HAL_GPT_ErrorCallback(htim);

    htim->Channel = HAL_GPT_ACTIVE_CHANNEL_CLEARED;
}

/**
  * @brief  Enables or disables the TIM Capture Compare Channel xN.
  * @param  TIMx to select the TIM peripheral
  * @param  Channel specifies the TIM Channel
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1
  *            @arg GPT_CHANNEL_2: TIM Channel 2
  *            @arg GPT_CHANNEL_3: TIM Channel 3
  * @param  ChannelNState specifies the TIM Channel CCxNE bit new state.
  *          This parameter can be: GPT_CCxN_ENABLE or GPT_CCxN_Disable.
  * @retval None
  */
static void GPT_CCxNChannelCmd(GPT_TypeDef *TIMx, uint32_t Channel, uint32_t ChannelNState)
{
    uint32_t tmp;

    tmp = GPT_CCER_CC1NE << (Channel & 0x1FU); /* 0x1FU = 31 bits max shift */

    /* Reset the CCxNE Bit */
    TIMx->CCER &=  ~tmp;

    /* Set or reset the CCxNE Bit */
    TIMx->CCER |= (uint32_t)(ChannelNState << (Channel & 0x1FU)); /* 0x1FU = 31 bits max shift */
}
/**
  * @}
  */

#endif /* HAL_GPT_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
