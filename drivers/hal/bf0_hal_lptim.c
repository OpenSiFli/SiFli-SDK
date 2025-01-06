/**
  ******************************************************************************
  * @file   bf0_hal_lptim.c
  * @author Sifli software development team
  * @brief   LPTIM HAL module driver.
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
#include "bf0_hal_lptim.h"
/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @addtogroup LPTIM Low power hardware timer.
  * @brief LPTIM HAL module driver.
  * @{
  */

#if defined(HAL_LPTIM_MODULE_ENABLED)||defined(_SIFLI_DOXYGEN_)
/* Private types -------------------------------------------------------------*/
/** @defgroup LPTIM_Private_Types LPTIM Private Types
  * @{
  */

/**
  * @}
  */

/* Private defines -----------------------------------------------------------*/
/** @defgroup LPTIM_Private_Defines LPTIM Private Defines
  * @{
  */

/**
  * @}
  */

/* Private variables ---------------------------------------------------------*/
/** @addtogroup LPTIM_Private_Variables LPTIM Private Variables
  * @{
  */

/**
  * @}
  */

/* Private constants ---------------------------------------------------------*/
/** @addtogroup LPTIM_Private_Constants LPTIM Private Constants
  * @{
  */

/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @addtogroup LPTIM_Private_Macros LPTIM Private Macros
  * @{
  */

/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/
/** @addtogroup LPTIM_Private_Functions_Prototypes LPTIM Private Functions Prototypes
  * @{
  */

/**
  * @}
  */

/* Private functions ---------------------------------------------------------*/
/** @addtogroup LPTIM_Private_Functions LPTIM Private Functions
  * @{
  */

/**
  * @}
  */

/* Exported functions ---------------------------------------------------------*/
/** @defgroup LPTIM_Exported_Functions LPTIM Exported Functions
  * @{
  */

/** @defgroup LPTIM_Group1 Initialization/de-initialization functions
 *  @brief    Initialization and Configuration functions.
 *
@verbatim
  ==============================================================================
              ##### Initialization and de-initialization functions #####
  ==============================================================================
    [..]  This section provides functions allowing to:
      (+) Initialize the LPTIM according to the specified parameters in the
          LPTIM_InitTypeDef and creates the associated handle.
      (+) DeInitialize the LPTIM peripheral.
      (+) Initialize the LPTIM MSP.
      (+) DeInitialize LPTIM MSP.

@endverbatim
  * @{
  */

__HAL_ROM_USED void HAL_LPTIM_InitDefault(LPTIM_HandleTypeDef *hlptim)
{
    if (hlptim)
    {
        /* Use internal 32K clock source. */
        hlptim->Init.Clock.Source      = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
        hlptim->Init.Clock.IntSource   = LPTIM_INTCLOCKSOURCE_LPCLOCK;
        hlptim->Init.Clock.Prescaler   = LPTIM_PRESCALER_DIV1;

        hlptim->Init.UltraLowPowerClock.Polarity   = LPTIM_ACTIVEEDGE_RISING;
        hlptim->Init.UltraLowPowerClock.SampleTime = LPTIM_CLOCKSAMPLETIME_DIRECTTRANSITION;

        hlptim->Init.Trigger.ActiveEdge  = LPTIM_ACTIVEEDGE_RISING;
        hlptim->Init.Trigger.Source      = LPTIM_TRIGSOURCE_SOFTWARE;
        hlptim->Init.Trigger.SampleTime  = LPTIM_CLOCKSAMPLETIME_DIRECTTRANSITION;
        hlptim->Init.OutputPolarity      = LPTIM_OUTPUTPOLARITY_HIGH;
        //tim->Init.UpdateMode          = LPTIM_UPDATE_IMMEDIATE;
        hlptim->Init.CounterSource       = LPTIM_COUNTERSOURCE_INTERNAL;
    }
}

/**
  * @brief  Initializes the LPTIM according to the specified parameters in the
  *         LPTIM_InitTypeDef and creates the associated handle.
  * @param  hlptim LPTIM handle
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_LPTIM_Init(LPTIM_HandleTypeDef *hlptim)
{
    uint32_t tmpcfgr = 0U;

    /* Check the LPTIM handle allocation */
    if (hlptim == NULL)
    {
        return HAL_ERROR;
    }

    /* Check the parameters */
    HAL_ASSERT(IS_LPTIM_INSTANCE(hlptim->Instance));

    HAL_ASSERT(IS_LPTIM_CLOCK_SOURCE(hlptim->Init.Clock.Source));
    HAL_ASSERT(IS_LPTIM_CLOCK_PRESCALER(hlptim->Init.Clock.Prescaler));
    if ((hlptim->Init.Clock.Source) ==  LPTIM_CLOCKSOURCE_ULPTIM)
    {
        HAL_ASSERT(IS_LPTIM_CLOCK_POLARITY(hlptim->Init.UltraLowPowerClock.Polarity));
        HAL_ASSERT(IS_LPTIM_CLOCK_SAMPLE_TIME(hlptim->Init.UltraLowPowerClock.SampleTime));
    }
    HAL_ASSERT(IS_LPTIM_TRG_SOURCE(hlptim->Init.Trigger.Source));
    //if ((hlptim->Init.Trigger.Source) !=  LPTIM_TRIGSOURCE_SOFTWARE)
    //{
    //    HAL_ASSERT(IS_LPTIM_TRIG_SAMPLE_TIME(hlptim->Init.Trigger.SampleTime));
    //    HAL_ASSERT(IS_LPTIM_EXT_TRG_POLARITY(hlptim->Init.Trigger.ActiveEdge));
    //}
    HAL_ASSERT(IS_LPTIM_OUTPUT_POLARITY(hlptim->Init.OutputPolarity));
    //HAL_ASSERT(IS_LPTIM_UPDATE_MODE(hlptim->Init.UpdateMode));
    HAL_ASSERT(IS_LPTIM_COUNTER_SOURCE(hlptim->Init.CounterSource));

    if (hlptim->State == HAL_LPTIM_STATE_RESET)
    {
        /* Allocate lock resource and initialize it */
        hlptim->Lock = HAL_UNLOCKED;
        /* Init the low level hardware */
        HAL_LPTIM_MspInit(hlptim);
    }

    /* Change the LPTIM state */
    hlptim->State = HAL_LPTIM_STATE_BUSY;

    /* Get the LPTIMx CFGR value */
    tmpcfgr = hlptim->Instance->CFGR;

    if ((hlptim->Init.Clock.Source) ==  LPTIM_CLOCKSOURCE_ULPTIM)
    {
        tmpcfgr &= (uint32_t)(~(LPTIM_CFGR_CKPOL | LPTIM_CFGR_CKFLT | LPTIM_CFGR_EXTCKSEL));
    }
    if ((hlptim->Init.Trigger.Source) !=  LPTIM_TRIGSOURCE_SOFTWARE)
    {
        tmpcfgr &= (uint32_t)(~(LPTIM_CFGR_TRGFLT | LPTIM_CFGR_TRIGSEL));
    }

    /* Clear CKSEL, PRESC, TRIGEN, TRGFLT, WAVPOL, PRELOAD & COUNTMODE bits */
    tmpcfgr &= (uint32_t)(~(LPTIM_CFGR_CKSEL | LPTIM_CFGR_TRIGEN | LPTIM_CFGR_WAVPOL |
                            LPTIM_CFGR_PRESC | LPTIM_CFGR_COUNTMODE));

    /* Set initialization parameters */
    tmpcfgr |= (hlptim->Init.Clock.Source    |
                hlptim->Init.Clock.Prescaler |
                hlptim->Init.OutputPolarity  |
                //hlptim->Init.UpdateMode      |
                hlptim->Init.CounterSource);

    if ((hlptim->Init.Clock.Source) ==  LPTIM_CLOCKSOURCE_ULPTIM)
    {
        tmpcfgr |= (hlptim->Init.UltraLowPowerClock.Polarity |
                    hlptim->Init.UltraLowPowerClock.SampleTime |
                    hlptim->Init.Clock.ExtSource);
    }
    else if ((hlptim->Init.Clock.Source) ==  LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC)
    {
        tmpcfgr |= hlptim->Init.Clock.IntSource;
    }

    if ((hlptim->Init.Trigger.Source) !=  LPTIM_TRIGSOURCE_SOFTWARE)
    {
        /* Enable External trigger and set the trigger source */
        tmpcfgr |= (hlptim->Init.Trigger.Source     |
                    hlptim->Init.Trigger.ActiveEdge |
                    hlptim->Init.Trigger.SampleTime);
    }

    /* Write to LPTIMx CFGR */
    hlptim->Instance->CFGR = tmpcfgr;

    /* Change the LPTIM state */
    hlptim->State = HAL_LPTIM_STATE_READY;

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  DeInitializes the LPTIM peripheral.
  * @param  hlptim LPTIM handle
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_LPTIM_DeInit(LPTIM_HandleTypeDef *hlptim)
{
    /* Check the LPTIM handle allocation */
    if (hlptim == NULL)
    {
        return HAL_ERROR;
    }

    /* Change the LPTIM state */
    hlptim->State = HAL_LPTIM_STATE_BUSY;

    /* Disable the LPTIM Peripheral Clock */
    __HAL_LPTIM_DISABLE(hlptim);

    /* DeInit the low level hardware: CLOCK, NVIC.*/
    HAL_LPTIM_MspDeInit(hlptim);

    /* Change the LPTIM state */
    hlptim->State = HAL_LPTIM_STATE_RESET;

    /* Release Lock */
    __HAL_UNLOCK(hlptim);

    /* Return function status */
    return HAL_OK;
}



/**
  * @brief  Initializes the LPTIM MSP.
  * @param  hlptim LPTIM handle
  * @retval None
  */
__weak void HAL_LPTIM_MspInit(LPTIM_HandleTypeDef *hlptim)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hlptim);
    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_LPTIM_MspInit could be implemented in the user file
     */
}

/**
  * @brief  DeInitializes LPTIM MSP.
  * @param  hlptim LPTIM handle
  * @retval None
  */
__weak void HAL_LPTIM_MspDeInit(LPTIM_HandleTypeDef *hlptim)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hlptim);
    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_LPTIM_MspDeInit could be implemented in the user file
     */
}

/**
  * @}
  */


/** @defgroup LPTIM_Group2 LPTIM Start-Stop operation functions
 *  @brief   Start-Stop operation functions.
 *
@verbatim
  ==============================================================================
                ##### LPTIM Start Stop operation functions #####
  ==============================================================================
    [..]  This section provides functions allowing to:
      (+) Start the PWM mode.
      (+) Stop the PWM mode.
      (+) Start the One pulse mode.
      (+) Stop the One pulse mode.
      (+) Start the Set once mode.
      (+) Stop the Set once mode.
      (+) Start the Encoder mode.
      (+) Stop the Encoder mode.
      (+) Start the Timeout mode.
      (+) Stop the Timeout mode.
      (+) Start the Counter mode.
      (+) Stop the Counter mode.


@endverbatim
  * @{
  */

/**
  * @brief  Starts the LPTIM PWM generation.
  * @param  hlptim  LPTIM handle
  * @param  Period  Specifies the Autoreload value.
  *         This parameter must be a value between 0x000000 and 0xFFFFFF.
  * @param  Pulse  Specifies the compare value.
  *         This parameter must be a value between 0x000000 and 0xFFFFFF.
  * @param  psc_reg Prescale value
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_LPTIM_PWM_Start(LPTIM_HandleTypeDef *hlptim, uint32_t Period, uint32_t Pulse, uint32_t psc_reg)
{
    /* Check the parameters */
    HAL_ASSERT(IS_LPTIM_INSTANCE(hlptim->Instance));
    HAL_ASSERT(IS_LPTIM_PERIOD(Period));
    HAL_ASSERT(IS_LPTIM_PULSE(Pulse));

    /* Set the LPTIM state */
    hlptim->State = HAL_LPTIM_STATE_BUSY;

    /* Reset WAVE bit to set PWM mode */
    hlptim->Instance->CFGR &= ~LPTIM_CFGR_WAVE;

    /* Enable the Peripheral */
    __HAL_LPTIM_ENABLE(hlptim);

    /* Load the period value in the autoreload register */
    __HAL_LPTIM_AUTORELOAD_SET(hlptim, Period);

    /* Load the pulse value in the compare register */
    __HAL_LPTIM_COMPARE_SET(hlptim, Pulse);

    __HAL_LPTIM_CLEAR_PRESCALER(hlptim, LPTIM_PRESCALER_DIV128);
    __HAL_LPTIM_SET_PRESCALER(hlptim, psc_reg);

    /* Start timer in continuous mode */
    __HAL_LPTIM_START_CONTINUOUS(hlptim);

    /* Change the TIM state*/
    hlptim->State = HAL_LPTIM_STATE_READY;

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  Stops the LPTIM PWM generation.
  * @param  hlptim  LPTIM handle
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_LPTIM_PWM_Stop(LPTIM_HandleTypeDef *hlptim)
{
    /* Check the parameters */
    HAL_ASSERT(IS_LPTIM_INSTANCE(hlptim->Instance));

    /* Set the LPTIM state */
    hlptim->State = HAL_LPTIM_STATE_BUSY;

    /* Disable the Peripheral */
    __HAL_LPTIM_DISABLE(hlptim);

    /* Change the TIM state*/
    hlptim->State = HAL_LPTIM_STATE_READY;

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  Starts the LPTIM PWM generation in interrupt mode.
  * @param  hlptim  LPTIM handle
  * @param  Period  Specifies the Autoreload value.
  *         This parameter must be a value between 0x000000 and 0xFFFFFF
  * @param  Pulse  Specifies the compare value.
  *         This parameter must be a value between 0x000000 and 0xFFFFFF
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_LPTIM_PWM_Start_IT(LPTIM_HandleTypeDef *hlptim, uint32_t Period, uint32_t Pulse)
{
    /* Check the parameters */
    HAL_ASSERT(IS_LPTIM_INSTANCE(hlptim->Instance));
    HAL_ASSERT(IS_LPTIM_PERIOD(Period));
    HAL_ASSERT(IS_LPTIM_PULSE(Pulse));

    /* Set the LPTIM state */
    hlptim->State = HAL_LPTIM_STATE_BUSY;

    /* Reset WAVE bit to set PWM mode */
    hlptim->Instance->CFGR &= ~LPTIM_CFGR_WAVE;

    /* Update event interrupt enable */
    __HAL_LPTIM_ENABLE_IT(hlptim, LPTIM_IT_UEIE);

    /* Overflow Interrupt Enable */
    __HAL_LPTIM_ENABLE_IT(hlptim, LPTIM_IT_OFIE);

    /* Output compare Interrupt Enable */
    __HAL_LPTIM_ENABLE_IT(hlptim, LPTIM_IT_OCIE);

    /* If external trigger source is used, then enable external trigger interrupt */
    if ((hlptim->Init.Trigger.Source) !=  LPTIM_TRIGSOURCE_SOFTWARE)
    {
        /* External trigger valid edge Interrupt Enable */
        __HAL_LPTIM_ENABLE_IT(hlptim, LPTIM_IT_ETIE);
    }

    /* Enable the Peripheral */
    __HAL_LPTIM_ENABLE(hlptim);

    /* Load the period value in the autoreload register */
    __HAL_LPTIM_AUTORELOAD_SET(hlptim, Period);

    /* Load the pulse value in the compare register */
    __HAL_LPTIM_COMPARE_SET(hlptim, Pulse);

    /* Start timer in continuous mode */
    __HAL_LPTIM_START_CONTINUOUS(hlptim);

    /* Change the TIM state*/
    hlptim->State = HAL_LPTIM_STATE_READY;

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  Stops the LPTIM PWM generation in interrupt mode.
  * @param  hlptim  LPTIM handle
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_LPTIM_PWM_Stop_IT(LPTIM_HandleTypeDef *hlptim)
{
    /* Check the parameters */
    HAL_ASSERT(IS_LPTIM_INSTANCE(hlptim->Instance));

    /* Set the LPTIM state */
    hlptim->State = HAL_LPTIM_STATE_BUSY;

    /* Disable the Peripheral */
    __HAL_LPTIM_DISABLE(hlptim);

    /* Update event interrupt disable */
    __HAL_LPTIM_DISABLE_IT(hlptim, LPTIM_IT_UEIE);

    /* Overflow Interrupt disable */
    __HAL_LPTIM_DISABLE_IT(hlptim, LPTIM_IT_OFIE);

    /* Output compare Interrupt disable */
    __HAL_LPTIM_DISABLE_IT(hlptim, LPTIM_IT_OCIE);

    /* If external trigger source is used, then enable external trigger interrupt */
    if ((hlptim->Init.Trigger.Source) !=  LPTIM_TRIGSOURCE_SOFTWARE)
    {
        /* External trigger valid edge Interrupt disable */
        __HAL_LPTIM_DISABLE_IT(hlptim, LPTIM_IT_ETIE);
    }

    /* Change the TIM state*/
    hlptim->State = HAL_LPTIM_STATE_READY;

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  Tthe LPTIM PWM set period, pulse, pscaler,  for real-time chang period.
  * @param  hlptim  LPTIM handle
  * @param  Period  Specifies the Autoreload value.
  *         This parameter must be a value between 0x000000 and 0xFFFFFF
  * @param  Pulse  Specifies the compare value.
  *         This parameter must be a value between 0x000000 and 0xFFFFFF
  * @param  psc_reg Prescale value, ref LPTIM_Clock_Prescaler
  * @retval HAL status
  */


__HAL_ROM_USED HAL_StatusTypeDef HAL_LPTIM_PWM_Set_Period(LPTIM_HandleTypeDef *hlptim, uint32_t Period, uint32_t Pulse, uint32_t psc_reg)
{
    /* Check the parameters */
    HAL_ASSERT(IS_LPTIM_INSTANCE(hlptim->Instance));
    HAL_ASSERT(IS_LPTIM_PERIOD(Period));
    HAL_ASSERT(IS_LPTIM_PULSE(Pulse));

    /* Set the LPTIM state */
    hlptim->State = HAL_LPTIM_STATE_BUSY;

    /* Load the period value in the autoreload register */
    while (HAL_LPTIM_ReadCounter(hlptim) > 1); /*wait CNT less than Period*/
    __HAL_LPTIM_AUTORELOAD_SET(hlptim, Period);

    /* Load the pulse value in the compare register */
    __HAL_LPTIM_COMPARE_SET(hlptim, Pulse);

    /* Set prescaler */
    __HAL_LPTIM_CLEAR_PRESCALER(hlptim, LPTIM_PRESCALER_DIV128);
    __HAL_LPTIM_SET_PRESCALER(hlptim, psc_reg);

    /* Change the TIM state*/
    hlptim->State = HAL_LPTIM_STATE_READY;

    /* Return function status */
    return HAL_OK;
}


/**
  * @brief  Starts the LPTIM One pulse generation.
  * @param  hlptim  LPTIM handle
  * @param  Period  Specifies the Autoreload value.
  *         This parameter must be a value between 0x000000 and 0xFFFFFF.
  * @param  Pulse  Specifies the compare value.
  *         This parameter must be a value between 0x000000 and 0xFFFFFF.
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_LPTIM_OnePulse_Start(LPTIM_HandleTypeDef *hlptim, uint32_t Period, uint32_t Pulse)
{
    /* Check the parameters */
    HAL_ASSERT(IS_LPTIM_INSTANCE(hlptim->Instance));
    HAL_ASSERT(IS_LPTIM_PERIOD(Period));
    HAL_ASSERT(IS_LPTIM_PULSE(Pulse));

    /* Set the LPTIM state */
    hlptim->State = HAL_LPTIM_STATE_BUSY;

    /* Reset WAVE bit to set one pulse mode */
    hlptim->Instance->CFGR &= ~LPTIM_CFGR_WAVE;

    /* Enable the Peripheral */
    __HAL_LPTIM_ENABLE(hlptim);

    /* Load the period value in the autoreload register */
    __HAL_LPTIM_AUTORELOAD_SET(hlptim, Period);

    /* Load the pulse value in the compare register */
    __HAL_LPTIM_COMPARE_SET(hlptim, Pulse);

    /* Start timer in continuous mode */
    __HAL_LPTIM_START_SINGLE(hlptim);

    /* Change the TIM state*/
    hlptim->State = HAL_LPTIM_STATE_READY;

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  Stops the LPTIM One pulse generation.
  * @param  hlptim  LPTIM handle
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_LPTIM_OnePulse_Stop(LPTIM_HandleTypeDef *hlptim)
{
    /* Check the parameters */
    HAL_ASSERT(IS_LPTIM_INSTANCE(hlptim->Instance));

    /* Set the LPTIM state */
    hlptim->State = HAL_LPTIM_STATE_BUSY;

    /* Disable the Peripheral */
    __HAL_LPTIM_DISABLE(hlptim);

    /* Change the TIM state*/
    hlptim->State = HAL_LPTIM_STATE_READY;

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  Starts the LPTIM One pulse generation in interrupt mode.
  * @param  hlptim  LPTIM handle
  * @param  Period  Specifies the Autoreload value.
  *         This parameter must be a value between 0x000000 and 0xFFFFFF.
  * @param  Pulse  Specifies the compare value.
  *         This parameter must be a value between 0x000000 and 0xFFFFFF.
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_LPTIM_OnePulse_Start_IT(LPTIM_HandleTypeDef *hlptim, uint32_t Period, uint32_t Pulse)
{
    /* Check the parameters */
    HAL_ASSERT(IS_LPTIM_INSTANCE(hlptim->Instance));
    HAL_ASSERT(IS_LPTIM_PERIOD(Period));
    HAL_ASSERT(IS_LPTIM_PULSE(Pulse));

    /* Set the LPTIM state */
    hlptim->State = HAL_LPTIM_STATE_BUSY;

    /* Reset WAVE bit to set one pulse mode */
    hlptim->Instance->CFGR &= ~LPTIM_CFGR_WAVE;

    /* Update event interrupt enable */
    __HAL_LPTIM_ENABLE_IT(hlptim, LPTIM_IT_UEIE);

    /* Overflow Interrupt Enable */
    __HAL_LPTIM_ENABLE_IT(hlptim, LPTIM_IT_OFIE);

    /* Output compare Interrupt Enablet */
    __HAL_LPTIM_ENABLE_IT(hlptim, LPTIM_IT_OCIE);

    /* If external trigger source is used, then enable external trigger interrupt */
    if ((hlptim->Init.Trigger.Source) !=  LPTIM_TRIGSOURCE_SOFTWARE)
    {
        /* External trigger valid edge Interrupt Enable */
        __HAL_LPTIM_ENABLE_IT(hlptim, LPTIM_IT_ETIE);
    }

    /* Enable the Peripheral */
    __HAL_LPTIM_ENABLE(hlptim);

    /* Load the period value in the autoreload register */
    __HAL_LPTIM_AUTORELOAD_SET(hlptim, Period);

    /* Load the pulse value in the compare register */
    __HAL_LPTIM_COMPARE_SET(hlptim, Pulse);

    /* Start timer in continuous mode */
    __HAL_LPTIM_START_SINGLE(hlptim);

    /* Change the TIM state*/
    hlptim->State = HAL_LPTIM_STATE_READY;

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  Stops the LPTIM One pulse generation in interrupt mode.
  * @param  hlptim  LPTIM handle
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_LPTIM_OnePulse_Stop_IT(LPTIM_HandleTypeDef *hlptim)
{
    /* Check the parameters */
    HAL_ASSERT(IS_LPTIM_INSTANCE(hlptim->Instance));

    /* Set the LPTIM state */
    hlptim->State = HAL_LPTIM_STATE_BUSY;

    /* Disable the Peripheral */
    __HAL_LPTIM_DISABLE(hlptim);


    /* Update event interrupt disable */
    __HAL_LPTIM_DISABLE_IT(hlptim, LPTIM_IT_UEIE);

    /* Overflow Interrupt disable */
    __HAL_LPTIM_DISABLE_IT(hlptim, LPTIM_IT_OFIE);

    /* Output compare Interrupt disable */
    __HAL_LPTIM_DISABLE_IT(hlptim, LPTIM_IT_OCIE);

    /* If external trigger source is used, then enable external trigger interrupt */
    if ((hlptim->Init.Trigger.Source) !=  LPTIM_TRIGSOURCE_SOFTWARE)
    {
        /* External trigger valid edge Interrupt disable */
        __HAL_LPTIM_DISABLE_IT(hlptim, LPTIM_IT_ETIE);
    }

    /* Change the TIM state*/
    hlptim->State = HAL_LPTIM_STATE_READY;

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  Starts the LPTIM in Set once mode.
  * @param  hlptim  LPTIM handle
  * @param  Period  Specifies the Autoreload value.
  *         This parameter must be a value between 0x000000 and 0xFFFFFF.
  * @param  Pulse  Specifies the compare value.
  *         This parameter must be a value between 0x000000 and 0xFFFFFF.
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_LPTIM_SetOnce_Start(LPTIM_HandleTypeDef *hlptim, uint32_t Period, uint32_t Pulse)
{
    /* Check the parameters */
    HAL_ASSERT(IS_LPTIM_INSTANCE(hlptim->Instance));
    HAL_ASSERT(IS_LPTIM_PERIOD(Period));
    HAL_ASSERT(IS_LPTIM_PULSE(Pulse));

    /* Set the LPTIM state */
    hlptim->State = HAL_LPTIM_STATE_BUSY;

    /* Set WAVE bit to enable the set once mode */
    hlptim->Instance->CFGR |= LPTIM_CFGR_WAVE;

    /* Enable the Peripheral */
    __HAL_LPTIM_ENABLE(hlptim);

    /* Load the period value in the autoreload register */
    __HAL_LPTIM_AUTORELOAD_SET(hlptim, Period);

    /* Load the pulse value in the compare register */
    __HAL_LPTIM_COMPARE_SET(hlptim, Pulse);

    /* Start timer in single mode */
    __HAL_LPTIM_START_SINGLE(hlptim);

    /* Change the TIM state*/
    hlptim->State = HAL_LPTIM_STATE_READY;

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  Stops the LPTIM Set once mode.
  * @param  hlptim  LPTIM handle
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_LPTIM_SetOnce_Stop(LPTIM_HandleTypeDef *hlptim)
{
    /* Check the parameters */
    HAL_ASSERT(IS_LPTIM_INSTANCE(hlptim->Instance));

    /* Set the LPTIM state */
    hlptim->State = HAL_LPTIM_STATE_BUSY;

    /* Disable the Peripheral */
    __HAL_LPTIM_DISABLE(hlptim);

    /* Change the TIM state*/
    hlptim->State = HAL_LPTIM_STATE_READY;

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  Starts the LPTIM Set once mode in interrupt mode.
  * @param  hlptim  LPTIM handle
  * @param  Period  Specifies the Autoreload value.
  *         This parameter must be a value between 0x000000 and 0xFFFFFF.
  * @param  Pulse  Specifies the compare value.
  *         This parameter must be a value between 0x000000 and 0xFFFFFF.
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_LPTIM_SetOnce_Start_IT(LPTIM_HandleTypeDef *hlptim, uint32_t Period, uint32_t Pulse)
{
    /* Check the parameters */
    HAL_ASSERT(IS_LPTIM_INSTANCE(hlptim->Instance));
    HAL_ASSERT(IS_LPTIM_PERIOD(Period));
    HAL_ASSERT(IS_LPTIM_PULSE(Pulse));

    /* Set the LPTIM state */
    hlptim->State = HAL_LPTIM_STATE_BUSY;

    /* Set WAVE bit to enable the set once mode */
    hlptim->Instance->CFGR |= LPTIM_CFGR_WAVE;


    /* Update event interrupt enable */
    __HAL_LPTIM_ENABLE_IT(hlptim, LPTIM_IT_UEIE);

    /* Overflow Interrupt Enable */
    __HAL_LPTIM_ENABLE_IT(hlptim, LPTIM_IT_OFIE);

    /* Output compare Interrupt Enablet */
    __HAL_LPTIM_ENABLE_IT(hlptim, LPTIM_IT_OCIE);

    /* If external trigger source is used, then enable external trigger interrupt */
    if ((hlptim->Init.Trigger.Source) !=  LPTIM_TRIGSOURCE_SOFTWARE)
    {
        /* External trigger valid edge Interrupt Enable */
        __HAL_LPTIM_ENABLE_IT(hlptim, LPTIM_IT_ETIE);
    }

    /* Enable the Peripheral */
    __HAL_LPTIM_ENABLE(hlptim);

    /* Load the period value in the autoreload register */
    __HAL_LPTIM_AUTORELOAD_SET(hlptim, Period);

    /* Load the pulse value in the compare register */
    __HAL_LPTIM_COMPARE_SET(hlptim, Pulse);

    /* Start timer in single mode */
    __HAL_LPTIM_START_SINGLE(hlptim);

    /* Change the TIM state*/
    hlptim->State = HAL_LPTIM_STATE_READY;

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  Stops the LPTIM Set once mode in interrupt mode.
  * @param  hlptim  LPTIM handle
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_LPTIM_SetOnce_Stop_IT(LPTIM_HandleTypeDef *hlptim)
{
    /* Check the parameters */
    HAL_ASSERT(IS_LPTIM_INSTANCE(hlptim->Instance));

    /* Set the LPTIM state */
    hlptim->State = HAL_LPTIM_STATE_BUSY;

    /* Disable the Peripheral */
    __HAL_LPTIM_DISABLE(hlptim);


    /* Update event interrupt disable */
    __HAL_LPTIM_DISABLE_IT(hlptim, LPTIM_IT_UEIE);

    /* Overflow Interrupt disable */
    __HAL_LPTIM_DISABLE_IT(hlptim, LPTIM_IT_OFIE);

    /* Output compare Interrupt disable */
    __HAL_LPTIM_DISABLE_IT(hlptim, LPTIM_IT_OCIE);

    /* If external trigger source is used, then enable external trigger interrupt */
    if ((hlptim->Init.Trigger.Source) !=  LPTIM_TRIGSOURCE_SOFTWARE)
    {
        /* External trigger valid edge Interrupt disable */
        __HAL_LPTIM_DISABLE_IT(hlptim, LPTIM_IT_ETIE);
    }

    /* Change the TIM state*/
    hlptim->State = HAL_LPTIM_STATE_READY;

    /* Return function status */
    return HAL_OK;
}


/**
  * @brief  Starts the Timeout function. The first trigger event will start the
  *         timer, any successive trigger event will reset the counter and
  *         the timer restarts.
  * @param  hlptim  LPTIM handle
  * @param  Period  Specifies the Autoreload value.
  *         This parameter must be a value between 0x000000 and 0xFFFFFF.
  * @param  Timeout  Specifies the TimeOut value to rest the counter.
  *         This parameter must be a value between 0x000000 and 0xFFFFFF.
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_LPTIM_TimeOut_Start(LPTIM_HandleTypeDef *hlptim, uint32_t Period, uint32_t Timeout)
{
    /* Check the parameters */
    HAL_ASSERT(IS_LPTIM_INSTANCE(hlptim->Instance));
    HAL_ASSERT(IS_LPTIM_PERIOD(Period));
    HAL_ASSERT(IS_LPTIM_PULSE(Timeout));

    /* Set the LPTIM state */
    hlptim->State = HAL_LPTIM_STATE_BUSY;

    /* Set TIMOUT bit to enable the timeout function */
    hlptim->Instance->CFGR |= LPTIM_CFGR_TIMOUT;

    /* Enable the Peripheral */
    __HAL_LPTIM_ENABLE(hlptim);

    /* Load the period value in the autoreload register */
    __HAL_LPTIM_AUTORELOAD_SET(hlptim, Period);

    /* Load the Timeout value in the compare register */
    __HAL_LPTIM_COMPARE_SET(hlptim, Timeout);

    /* Start timer in continuous mode */
    __HAL_LPTIM_START_CONTINUOUS(hlptim);

    /* Change the TIM state*/
    hlptim->State = HAL_LPTIM_STATE_READY;

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  Stops the Timeout function.
  * @param  hlptim  LPTIM handle
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_LPTIM_TimeOut_Stop(LPTIM_HandleTypeDef *hlptim)
{
    /* Check the parameters */
    HAL_ASSERT(IS_LPTIM_INSTANCE(hlptim->Instance));

    /* Set the LPTIM state */
    hlptim->State = HAL_LPTIM_STATE_BUSY;

    /* Disable the Peripheral */
    __HAL_LPTIM_DISABLE(hlptim);

    /* Reset TIMOUT bit to enable the timeout function */
    hlptim->Instance->CFGR &= ~LPTIM_CFGR_TIMOUT;

    /* Change the TIM state*/
    hlptim->State = HAL_LPTIM_STATE_READY;

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  Starts the Timeout function in interrupt mode. The first trigger
  *         event will start the timer, any successive trigger event will reset
  *         the counter and the timer restarts.
  * @param  hlptim  LPTIM handle
  * @param  Period  Specifies the Autoreload value.
  *         This parameter must be a value between 0x000000 and 0xFFFFFF.
  * @param  Timeout  Specifies the TimeOut value to rest the counter.
  *         This parameter must be a value between 0x000000 and 0xFFFFFF.
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_LPTIM_TimeOut_Start_IT(LPTIM_HandleTypeDef *hlptim, uint32_t Period, uint32_t Timeout)
{
    /* Check the parameters */
    HAL_ASSERT(IS_LPTIM_INSTANCE(hlptim->Instance));
    HAL_ASSERT(IS_LPTIM_PERIOD(Period));
    HAL_ASSERT(IS_LPTIM_PULSE(Timeout));

    /* Set the LPTIM state */
    hlptim->State = HAL_LPTIM_STATE_BUSY;

    /* Set TIMOUT bit to enable the timeout function */
    hlptim->Instance->CFGR |= LPTIM_CFGR_TIMOUT;

    /* Update event interrupt enable */
    __HAL_LPTIM_ENABLE_IT(hlptim, LPTIM_IT_UEIE);

    /* Overflow Interrupt Enable */
    __HAL_LPTIM_ENABLE_IT(hlptim, LPTIM_IT_OFIE);

    /* Output compare Interrupt Enablet */
    __HAL_LPTIM_ENABLE_IT(hlptim, LPTIM_IT_OCIE);

    /* If external trigger source is used, then enable external trigger interrupt */
    if ((hlptim->Init.Trigger.Source) !=  LPTIM_TRIGSOURCE_SOFTWARE)
    {
        /* External trigger valid edge Interrupt Enable */
        __HAL_LPTIM_ENABLE_IT(hlptim, LPTIM_IT_ETIE);
    }

    /* Enable the Peripheral */
    __HAL_LPTIM_ENABLE(hlptim);

    /* Load the period value in the autoreload register */
    __HAL_LPTIM_AUTORELOAD_SET(hlptim, Period);

    /* Load the Timeout value in the compare register */
    __HAL_LPTIM_COMPARE_SET(hlptim, Timeout);

    /* Start timer in continuous mode */
    __HAL_LPTIM_START_CONTINUOUS(hlptim);

    /* Change the TIM state*/
    hlptim->State = HAL_LPTIM_STATE_READY;

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  Stops the Timeout function in interrupt mode.
  * @param  hlptim  LPTIM handle
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_LPTIM_TimeOut_Stop_IT(LPTIM_HandleTypeDef *hlptim)
{
    /* Check the parameters */
    HAL_ASSERT(IS_LPTIM_INSTANCE(hlptim->Instance));

    /* Set the LPTIM state */
    hlptim->State = HAL_LPTIM_STATE_BUSY;

    /* Disable the Peripheral */
    __HAL_LPTIM_DISABLE(hlptim);

    /* Reset TIMOUT bit to enable the timeout function */
    hlptim->Instance->CFGR &= ~LPTIM_CFGR_TIMOUT;

    /* Update event interrupt disable */
    __HAL_LPTIM_DISABLE_IT(hlptim, LPTIM_IT_UEIE);

    /* Overflow Interrupt disable */
    __HAL_LPTIM_DISABLE_IT(hlptim, LPTIM_IT_OFIE);

    /* Output compare Interrupt disable */
    __HAL_LPTIM_DISABLE_IT(hlptim, LPTIM_IT_OCIE);

    /* If external trigger source is used, then enable external trigger interrupt */
    if ((hlptim->Init.Trigger.Source) !=  LPTIM_TRIGSOURCE_SOFTWARE)
    {
        /* External trigger valid edge Interrupt disable */
        __HAL_LPTIM_DISABLE_IT(hlptim, LPTIM_IT_ETIE);
    }


    /* Change the TIM state*/
    hlptim->State = HAL_LPTIM_STATE_READY;

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  Starts the Counter mode.
  * @param  hlptim  LPTIM handle
  * @param  Period  Specifies the Autoreload value.
  *         This parameter must be a value between 0x000000 and 0xFFFFFF.
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_LPTIM_Counter_Start(LPTIM_HandleTypeDef *hlptim, uint32_t Period)
{
    /* Check the parameters */
    HAL_ASSERT(IS_LPTIM_INSTANCE(hlptim->Instance));
    HAL_ASSERT(IS_LPTIM_PERIOD(Period));

    /* Set the LPTIM state */
    hlptim->State = HAL_LPTIM_STATE_BUSY;

    /* If clock source is not ULPTIM clock and counter source is external, then it must not be prescaled */
    if ((hlptim->Init.Clock.Source != LPTIM_CLOCKSOURCE_ULPTIM) && (hlptim->Init.CounterSource == LPTIM_COUNTERSOURCE_EXTERNAL))
    {
        /* Check if clock is prescaled */
        HAL_ASSERT(IS_LPTIM_CLOCK_PRESCALERDIV1(hlptim->Init.Clock.Prescaler));
        /* Set clock prescaler to 0 */
        hlptim->Instance->CFGR &= ~LPTIM_CFGR_PRESC;
    }

    /* Enable the Peripheral */
    __HAL_LPTIM_ENABLE(hlptim);

    /* Load the period value in the autoreload register */
    __HAL_LPTIM_AUTORELOAD_SET(hlptim, Period);

    /* Start timer in continuous mode */
    __HAL_LPTIM_START_CONTINUOUS(hlptim);

    /* Change the TIM state*/
    hlptim->State = HAL_LPTIM_STATE_READY;

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  Stops the Counter mode.
  * @param  hlptim  LPTIM handle
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_LPTIM_Counter_Stop(LPTIM_HandleTypeDef *hlptim)
{
    /* Check the parameters */
    HAL_ASSERT(IS_LPTIM_INSTANCE(hlptim->Instance));

    /* Set the LPTIM state */
    hlptim->State = HAL_LPTIM_STATE_BUSY;

    /* Disable the Peripheral */
    __HAL_LPTIM_DISABLE(hlptim);

    /* Change the TIM state*/
    hlptim->State = HAL_LPTIM_STATE_READY;

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  Starts the Counter mode in interrupt mode.
  * @param  hlptim  LPTIM handle
  * @param  Period  Specifies the Autoreload value.
  *         This parameter must be a value between 0x000000 and 0xFFFFFF.
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_LPTIM_Counter_Start_IT(LPTIM_HandleTypeDef *hlptim, uint32_t Period)
{

    uint32_t tmpcfgr = 0U;

    /* Check the parameters */
    HAL_ASSERT(IS_LPTIM_INSTANCE(hlptim->Instance));
    HAL_ASSERT(IS_LPTIM_PERIOD(Period));

    /* Set the LPTIM state */
    hlptim->State = HAL_LPTIM_STATE_BUSY;


    /* If clock source is not ULPTIM clock and counter source is external, then it must not be prescaled */
    if ((hlptim->Init.Clock.Source != LPTIM_CLOCKSOURCE_ULPTIM) && (hlptim->Init.CounterSource == LPTIM_COUNTERSOURCE_EXTERNAL))
    {
        /* Check if clock is prescaled */
        HAL_ASSERT(IS_LPTIM_CLOCK_PRESCALERDIV1(hlptim->Init.Clock.Prescaler));
        /* Set clock prescaler to 0 */
        hlptim->Instance->CFGR &= ~LPTIM_CFGR_PRESC;
    }

    else if ((hlptim->Init.Clock.Source != LPTIM_CLOCKSOURCE_ULPTIM) && (hlptim->Init.CounterSource != LPTIM_COUNTERSOURCE_EXTERNAL))
    {
        /* Get the LPTIMx CFGR value */
        tmpcfgr = hlptim->Instance->CFGR;

        /* Clear PRESC bits */
        tmpcfgr &= (uint32_t)(~LPTIM_CFGR_PRESC);

        /* Set Input prescaler */
        tmpcfgr |=  hlptim->Init.Clock.Prescaler;

    }

    /* Reset the counter */
    __HAL_LPTIM_COUNTRST_RESET(hlptim);

    while (__HAL_LPTIM_COUNTRST_GET(hlptim));

    /* Enable overflow notify interrupt */
    __HAL_LPTIM_ENABLE_IT(hlptim, LPTIM_IT_OFIE);
    __HAL_LPTIM_ENABLE_IT(hlptim, LPTIM_IT_OFWE);

    /* Enable the Peripheral */
    __HAL_LPTIM_ENABLE(hlptim);

    /* Load the period value in the autoreload register */
    __HAL_LPTIM_AUTORELOAD_SET(hlptim, Period);

    if (hlptim->Mode == HAL_LPTIM_PERIOD)
    {
        /* Start timer in continuous mode */
        __HAL_LPTIM_START_CONTINUOUS(hlptim);
    }
    else
    {
        __HAL_LPTIM_START_SINGLE(hlptim);
    }

    /* Change the TIM state*/
    hlptim->State = HAL_LPTIM_STATE_READY;

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  Stops the Counter mode in interrupt mode.
  * @param  hlptim  LPTIM handle
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_LPTIM_Counter_Stop_IT(LPTIM_HandleTypeDef *hlptim)
{
    /* Check the parameters */
    HAL_ASSERT(IS_LPTIM_INSTANCE(hlptim->Instance));

    /* Set the LPTIM state */
    hlptim->State = HAL_LPTIM_STATE_BUSY;


    /* Disable the Peripheral */
    __HAL_LPTIM_DISABLE(hlptim);

    /* Disable OF notify interrupt */
    __HAL_LPTIM_DISABLE_IT(hlptim, LPTIM_IT_OFIE);
    __HAL_LPTIM_DISABLE_IT(hlptim, LPTIM_IT_OFWE);

    /* Change the TIM state*/
    hlptim->State = HAL_LPTIM_STATE_READY;

    /* Return function status */
    return HAL_OK;
}

/**
  * @}
  */

/** @defgroup LPTIM_Group3 LPTIM Read operation functions
*  @brief  Read operation functions.
*
@verbatim
==============================================================================
##### LPTIM Read operation functions #####
==============================================================================
[..]  This section provides LPTIM Reading functions.
(+) Read the counter value.
(+) Read the period (Auto-reload) value.
(+) Read the pulse (Compare)value.
@endverbatim
* @{
*/

/**
  * @brief  This function returns the current counter value.
  * @param  hlptim LPTIM handle
  * @retval Counter value.
  */
__HAL_ROM_USED uint32_t HAL_LPTIM_ReadCounter(LPTIM_HandleTypeDef *hlptim)
{
    /* Check the parameters */
    HAL_ASSERT(IS_LPTIM_INSTANCE(hlptim->Instance));

    return (hlptim->Instance->CNT);
}

/**
  * @brief  This function return the current Autoreload (Period) value.
  * @param  hlptim LPTIM handle
  * @retval Autoreload value.
  */
__HAL_ROM_USED uint32_t HAL_LPTIM_ReadAutoReload(LPTIM_HandleTypeDef *hlptim)
{
    /* Check the parameters */
    HAL_ASSERT(IS_LPTIM_INSTANCE(hlptim->Instance));

    return (hlptim->Instance->ARR);
}

/**
  * @brief  This function return the current Compare (Pulse) value.
  * @param  hlptim LPTIM handle
  * @retval Compare value.
  */
__HAL_ROM_USED uint32_t HAL_LPTIM_ReadCompare(LPTIM_HandleTypeDef *hlptim)
{
    /* Check the parameters */
    HAL_ASSERT(IS_LPTIM_INSTANCE(hlptim->Instance));

    return (hlptim->Instance->CMP);
}

/**
  * @}
  */

/** @defgroup LPTIM_Group4 LPTIM IRQ handler
 *  @brief  LPTIM  IRQ handler.
 *
@verbatim
  ==============================================================================
                      ##### LPTIM IRQ handler  #####
  ==============================================================================
[..]  This section provides LPTIM IRQ handler function.

@endverbatim
  * @{
  */

/**
  * @brief  This function handles LPTIM interrupt request.
  * @param  hlptim LPTIM handle
  * @retval None
  */
__HAL_ROM_USED void HAL_LPTIM_IRQHandler(LPTIM_HandleTypeDef *hlptim)
{
    /* Update event interrupt */
    if (__HAL_LPTIM_GET_FLAG(hlptim, LPTIM_FLAG_UE) != RESET)
    {
        if (__HAL_LPTIM_GET_IT_SOURCE(hlptim, LPTIM_IT_UEIE) != RESET)
        {
            /* Clear Trigger detected flag */
            __HAL_LPTIM_CLEAR_FLAG(hlptim, LPTIM_ICR_UECLR);
            /* Trigger detected callback */
            HAL_LPTIM_UpdateEventCallback(hlptim);
        }
    }

    /* Overflow interrupt */
    if (__HAL_LPTIM_GET_FLAG(hlptim, LPTIM_FLAG_OF) != RESET)
    {
        /* Clear Autoreload write flag */
        __HAL_LPTIM_CLEAR_FLAG(hlptim, LPTIM_ICR_OFCLR);
        /* Autoreload write Callback */
        HAL_LPTIM_AutoReloadWriteCallback(hlptim);

        //if (hlptim->Instance->CR & LPTIM_CR_CNTSTRT)
        //{
        /* Enable overflow notify interrupt */
        //    __HAL_LPTIM_ENABLE_IT(hlptim, LPTIM_IT_OFIE);
        //}
    }

    /* Output compare interrupt */
    if (__HAL_LPTIM_GET_FLAG(hlptim, LPTIM_FLAG_OC) != RESET)
    {
        if (__HAL_LPTIM_GET_IT_SOURCE(hlptim, LPTIM_IT_OCIE) != RESET)
        {
            /* Clear Trigger detected flag */
            __HAL_LPTIM_CLEAR_FLAG(hlptim, LPTIM_ICR_OCCLR);
            /* Trigger detected callback */
            HAL_LPTIM_OutCompareCallback(hlptim);
        }
    }


    /* Trigger detected interrupt */
    if (__HAL_LPTIM_GET_FLAG(hlptim, LPTIM_FLAG_ET) != RESET)
    {
        if (__HAL_LPTIM_GET_IT_SOURCE(hlptim, LPTIM_IT_ETIE) != RESET)
        {
            /* Clear Trigger detected flag */
            __HAL_LPTIM_CLEAR_FLAG(hlptim, LPTIM_ICR_ETCLR);
            /* Trigger detected callback */
            HAL_LPTIM_TriggerCallback(hlptim);
        }
    }


    /* Update event Wakeup interrupt */
    if (__HAL_LPTIM_GET_FLAG(hlptim, LPTIM_FLAG_UEWKUP) != RESET)
    {
        /* Disable OF wakeup first*/
        __HAL_LPTIM_DISABLE_IT(hlptim, LPTIM_IT_UEWE);
        /* Clear wakeup flag */
        __HAL_LPTIM_CLEAR_FLAG(hlptim, LPTIM_ICR_WKUPCLR);
        /* Autoreload write Callback */
        HAL_LPTIM_OverFlowWakeUpCallback(hlptim);

        //if (hlptim->Instance->CR & LPTIM_CR_CNTSTRT)
        //{
        /* Enable overflow notify interrupt */
        //    __HAL_LPTIM_ENABLE_IT(hlptim, LPTIM_IT_OFIE);
        //}
    }

    /* Overflow Wakeup interrupt */
    if (__HAL_LPTIM_GET_FLAG(hlptim, LPTIM_FLAG_OFWKUP) != RESET)
    {
        /* Disable OF wakeup first*/
        __HAL_LPTIM_DISABLE_IT(hlptim, LPTIM_IT_OFWE);
        /* Clear wakeup flag */
        __HAL_LPTIM_CLEAR_FLAG(hlptim, LPTIM_ICR_WKUPCLR);
        /* Autoreload write Callback */
        HAL_LPTIM_OverFlowWakeUpCallback(hlptim);

        //if (hlptim->Instance->CR & LPTIM_CR_CNTSTRT)
        //{
        /* Enable overflow notify interrupt */
        //    __HAL_LPTIM_ENABLE_IT(hlptim, LPTIM_IT_OFIE);
        //}
    }

    /* Output compare Wakeup interrupt */
    if (__HAL_LPTIM_GET_FLAG(hlptim, LPTIM_FLAG_OCWKUP) != RESET)
    {
        /* Disable OF wakeup first*/
        __HAL_LPTIM_DISABLE_IT(hlptim, LPTIM_IT_OCWE);
        /* Clear wakeup flag */
        __HAL_LPTIM_CLEAR_FLAG(hlptim, LPTIM_ICR_WKUPCLR);
        /* Autoreload write Callback */
        HAL_LPTIM_OverFlowWakeUpCallback(hlptim);

        //if (hlptim->Instance->CR & LPTIM_CR_CNTSTRT)
        //{
        /* Enable overflow notify interrupt */
        //    __HAL_LPTIM_ENABLE_IT(hlptim, LPTIM_IT_OFIE);
        //}
    }

}

/**
  * @brief  Compare match callback in non blocking mode
  * @param  hlptim  LPTIM handle
  * @retval None
  */
__weak void HAL_LPTIM_CompareMatchCallback(LPTIM_HandleTypeDef *hlptim)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hlptim);
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_LPTIM_CompareMatchCallback could be implemented in the user file
    */
}

/**
  * @brief  Autoreload match callback in non blocking mode
  * @param  hlptim  LPTIM handle
  * @retval None
  */
__weak void HAL_LPTIM_AutoReloadMatchCallback(LPTIM_HandleTypeDef *hlptim)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hlptim);
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_LPTIM_AutoReloadMatchCallback could be implemented in the user file
    */
}

/**
  * @brief  Trigger detected callback in non blocking mode
  * @param  hlptim  LPTIM handle
  * @retval None
  */
__weak void HAL_LPTIM_TriggerCallback(LPTIM_HandleTypeDef *hlptim)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hlptim);
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_LPTIM_TriggerCallback could be implemented in the user file
    */
}

/**
  * @brief  Ouput compare callback in non blocking mode
  * @param  hlptim  LPTIM handle
  * @retval None
  */
__weak void HAL_LPTIM_OutCompareCallback(LPTIM_HandleTypeDef *hlptim)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hlptim);
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_LPTIM_OutCompareCallback could be implemented in the user file
    */
}

/**
  * @brief  Update event callback in non blocking mode
  * @param  hlptim  LPTIM handle
  * @retval None
  */
__weak void HAL_LPTIM_UpdateEventCallback(LPTIM_HandleTypeDef *hlptim)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hlptim);
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_LPTIM_UpdateEventCallback could be implemented in the user file
    */
}


/**
  * @brief  Compare write callback in non blocking mode
  * @param  hlptim  LPTIM handle
  * @retval None
  */
__weak void HAL_LPTIM_CompareWriteCallback(LPTIM_HandleTypeDef *hlptim)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hlptim);
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_LPTIM_CompareWriteCallback could be implemented in the user file
    */
}

/**
  * @brief  Autoreload write callback in non blocking mode
  * @param  hlptim  LPTIM handle
  * @retval None
  */
__weak void HAL_LPTIM_AutoReloadWriteCallback(LPTIM_HandleTypeDef *hlptim)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hlptim);
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_LPTIM_AutoReloadWriteCallback could be implemented in the user file
    */
}

/**
  * @brief  Autoreload write callback in non blocking mode
  * @param  hlptim  LPTIM handle
  * @retval None
  */
__weak void HAL_LPTIM_OverFlowWakeUpCallback(LPTIM_HandleTypeDef *hlptim)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hlptim);
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_LPTIM_AutoReloadWriteCallback could be implemented in the user file
    */
}


/**
  * @brief  Direction counter changed from Down to Up callback in non blocking mode
  * @param  hlptim  LPTIM handle
  * @retval None
  */
__weak void HAL_LPTIM_DirectionUpCallback(LPTIM_HandleTypeDef *hlptim)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hlptim);
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_LPTIM_DirectionUpCallback could be implemented in the user file
    */
}

/**
  * @brief  Direction counter changed from Up to Down callback in non blocking mode
  * @param  hlptim  LPTIM handle
  * @retval None
  */
__weak void HAL_LPTIM_DirectionDownCallback(LPTIM_HandleTypeDef *hlptim)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hlptim);
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_LPTIM_DirectionDownCallback could be implemented in the user file
    */
}

/**
  * @}
  */

/** @defgroup LPTIM_Group5 Peripheral State functions
 *  @brief   Peripheral State functions.
 *
@verbatim
  ==============================================================================
                      ##### Peripheral State functions #####
  ==============================================================================
    [..]
    This subsection permits to get in run-time the status of the peripheral.

@endverbatim
  * @{
  */

/**
  * @brief  Returns the LPTIM state.
  * @param  hlptim LPTIM handle
  * @retval HAL state
  */
__HAL_ROM_USED HAL_LPTIM_StateTypeDef HAL_LPTIM_GetState(LPTIM_HandleTypeDef *hlptim)
{
    return hlptim->State;
}

/**
  * @}
  */


/**
  * @}
  */

#endif /* HAL_LPTIM_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
