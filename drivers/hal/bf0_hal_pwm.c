/*
 * SPDX-FileCopyrightText: 2016 STMicroelectronics
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: BSD-3-Clause AND Apache-2.0
 */

#include "bf0_hal.h"

/** @addtogroup BF0_HAL_Driver
 * @{
 */

/** @defgroup TIM Hardware Timer
 * @brief TIM HAL module driver
 * @{
 */
#if defined(HAL_PWM_MODULE_ENABLED)||defined(_SIFLI_DOXYGEN_)
#include "bf0_hal_pwm.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/** @addtogroup PWM_Private_Functions
 * @{
 */
/* Private function prototypes -----------------------------------------------*/
static void PWM_OC_SetConfig(PWM_ChannelTypeDef *PWM_Channel, PWM_OC_InitTypeDef *OC_Config);

/**
 * @}
 */

/* Exported functions --------------------------------------------------------*/
/** @defgroup PWM_Exported_Functions TIM Exported Functions
 * @{
 */

/** @defgroup PWM_Exported_Functions_Group1 Time Base functions
 *  @brief    Time Base functions
 *
@verbatim
  ==============================================================================
              ##### Time Base functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
    (+) Initialize and configure the TIM base.
    (+) De-initialize the TIM base.
    (+) Start the Time Base.
    (+) Stop the Time Base.
    (+) Start the Time Base and enable interrupt.
    (+) Stop the Time Base and disable interrupt.
    (+) Start the Time Base and enable DMA transfer.
    (+) Stop the Time Base and disable DMA transfer.

@endverbatim
  * @{
  */
/**
 * @brief  Initializes the TIM Time base Unit according to the specified
 *         parameters in the PWM_HandleTypeDef and create the associated handle.
 * @param  hpwm pointer to a PWM_HandleTypeDef structure that contains
 *                the configuration information for TIM module.
 * @retval HAL status
 */
__HAL_ROM_USED HAL_StatusTypeDef HAL_PWM_Init(PWM_HandleTypeDef *hpwm)
{
    /* Check the TIM handle allocation */
    if (hpwm == NULL)
    {
        return HAL_ERROR;
    }

#ifdef hwp_pwm1
    if (hpwm->Instance == hwp_pwm1)
        HAL_RCC_EnableModule(RCC_MOD_PWM1);
#endif /* hwp_pwm1 */

    /* Check the parameters */
    HAL_ASSERT(IS_PWM_INSTANCE(hpwm->Instance));

    if (hpwm->State == HAL_PWM_STATE_RESET)
    {
        /* Allocate lock resource and initialize it */
        hpwm->Lock = HAL_UNLOCKED;
        /* Init the low level hardware : GPIO, CLOCK, NVIC */
        HAL_PWM_MspInit(hpwm);
    }

    /* Set the TIM state */
    hpwm->State = HAL_PWM_STATE_BUSY;

    __HAL_PWM_CLEAR_FLAG(hpwm, PWM_FLAG_UPDATE1);     /* clear update flag */
    __HAL_PWM_CLEAR_FLAG(hpwm, PWM_FLAG_UPDATE2);     /* clear update flag */
    __HAL_PWM_CLEAR_FLAG(hpwm, PWM_FLAG_UPDATE3);     /* clear update flag */
    __HAL_PWM_CLEAR_FLAG(hpwm, PWM_FLAG_UPDATE4);     /* clear update flag */
    __HAL_PWM_URS_ENABLE(hpwm, 0);                      /* enable update request source */
    __HAL_PWM_URS_ENABLE(hpwm, 1);                      /* enable update request source */
    __HAL_PWM_URS_ENABLE(hpwm, 2);                      /* enable update request source */
    __HAL_PWM_URS_ENABLE(hpwm, 3);                      /* enable update request source */
    /* Initialize the TIM state*/
    hpwm->State = HAL_PWM_STATE_READY;

    return HAL_OK;
}

/**
 * @brief  DeInitializes the TIM Base peripheral
 * @param  hpwm pointer to a PWM_HandleTypeDef structure that contains
 *                the configuration information for TIM module.
 * @retval HAL status
 */
__HAL_ROM_USED HAL_StatusTypeDef HAL_PWM_DeInit(PWM_HandleTypeDef *hpwm)
{
    /* Check the parameters */
    HAL_ASSERT(IS_PWM_INSTANCE(hpwm->Instance));

    hpwm->State = HAL_PWM_STATE_BUSY;

    /* Disable the TIM Peripheral Clock */
    __HAL_PWM_DISABLE(hpwm, 0);
    __HAL_PWM_DISABLE(hpwm, 1);
    __HAL_PWM_DISABLE(hpwm, 2);
    __HAL_PWM_DISABLE(hpwm, 3);

    /* DeInit the low level hardware: GPIO, CLOCK, NVIC */
    HAL_PWM_MspDeInit(hpwm);

    /* Change TIM state */
    hpwm->State = HAL_PWM_STATE_RESET;

    /* Release Lock */
    __HAL_UNLOCK(hpwm);

    return HAL_OK;
}

/**
 * @brief  Initializes the TIM Base MSP.
 * @param  hpwm pointer to a PWM_HandleTypeDef structure that contains
 *                the configuration information for TIM module.
 * @retval None
 */
__weak void HAL_PWM_MspInit(PWM_HandleTypeDef *hpwm)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hpwm);
    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_PWM_Base_MspInit could be implemented in the user file
     */
}

/**
 * @brief  DeInitializes TIM Base MSP.
 * @param  hpwm pointer to a PWM_HandleTypeDef structure that contains
 *                the configuration information for TIM module.
 * @retval None
 */
__weak void HAL_PWM_MspDeInit(PWM_HandleTypeDef *hpwm)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hpwm);
    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_PWM_Base_MspDeInit could be implemented in the user file
     */
}


/** @defgroup PWM_Exported_Functions_Group3 Time PWM functions
 *  @brief    Time PWM functions
 *
@verbatim
  ==============================================================================
                          ##### Time PWM functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
    (+) Initialize and configure the TIM OPWM.
    (+) De-initialize the TIM PWM.
    (+) Start the Time PWM.
    (+) Stop the Time PWM.
    (+) Start the Time PWM and enable interrupt.
    (+) Stop the Time PWM and disable interrupt.
    (+) Start the Time PWM and enable DMA transfer.
    (+) Stop the Time PWM and disable DMA transfer.

@endverbatim
  * @{
  */

/**
 * @brief  Starts the PWM signal generation.
 * @param  hpwm pointer to a PWM_HandleTypeDef structure that contains
 *                the configuration information for TIM module.
 * @param  Channel TIM Channels to be enabled.
 *          This parameter can be one of the following values:
 *            @arg PWM_CHANNEL_1: TIM Channel 1 selected
 *            @arg PWM_CHANNEL_2: TIM Channel 2 selected
 *            @arg PWM_CHANNEL_3: TIM Channel 3 selected
 *            @arg PWM_CHANNEL_4: TIM Channel 4 selected
 * @retval HAL status
 */
__HAL_ROM_USED HAL_StatusTypeDef HAL_PWM_Start(PWM_HandleTypeDef *hpwm, uint32_t Channel)
{
    /* Check the parameters */
    HAL_ASSERT(IS_PWM_INSTANCE(hpwm->Instance));
    HAL_ASSERT(IS_PWM_CHANNELS(Channel));

    /* Enable the Capture compare channel */
    PWM_CCxChannelCmd(hpwm->Instance, Channel, PWM_CCx_ENABLE);

    /* Enable the Peripheral */
    __HAL_PWM_ENABLE(hpwm, Channel);

    /* Return function status */
    return HAL_OK;
}

/**
 * @brief  Stops the PWM signal generation.
 * @param  hpwm pointer to a PWM_HandleTypeDef structure that contains
 *                the configuration information for TIM module.
 * @param  Channel TIM Channels to be disabled.
 *          This parameter can be one of the following values:
 *            @arg PWM_CHANNEL_1: TIM Channel 1 selected
 *            @arg PWM_CHANNEL_2: TIM Channel 2 selected
 *            @arg PWM_CHANNEL_3: TIM Channel 3 selected
 *            @arg PWM_CHANNEL_4: TIM Channel 4 selected
 * @retval HAL status
 */
__HAL_ROM_USED HAL_StatusTypeDef HAL_PWM_Stop(PWM_HandleTypeDef *hpwm, uint32_t Channel)
{
    /* Check the parameters */
    HAL_ASSERT(IS_PWM_INSTANCE(hpwm->Instance));
    HAL_ASSERT(IS_PWM_CHANNELS(Channel));

    /* Disable the Capture compare channel */
    PWM_CCxChannelCmd(hpwm->Instance, Channel, PWM_CCx_DISABLE);

    /* Disable the Peripheral */
    __HAL_PWM_DISABLE(hpwm, Channel);

    /* Change the hpwm state */
    hpwm->State = HAL_PWM_STATE_READY;

    /* Return function status */
    return HAL_OK;
}

/**
 * @brief  Starts the PWM signal generation in interrupt mode.
 * @param  hpwm pointer to a PWM_HandleTypeDef structure that contains
 *                the configuration information for TIM module.
 * @param  Channel TIM Channel to be enabled.
 *          This parameter can be one of the following values:
 *            @arg PWM_CHANNEL_1: TIM Channel 1 selected
 *            @arg PWM_CHANNEL_2: TIM Channel 2 selected
 *            @arg PWM_CHANNEL_3: TIM Channel 3 selected
 *            @arg PWM_CHANNEL_4: TIM Channel 4 selected
 * @retval HAL status
 */
__HAL_ROM_USED HAL_StatusTypeDef HAL_PWM_Start_IT(PWM_HandleTypeDef *hpwm, uint32_t Channel)
{
    /* Check the parameters */
    HAL_ASSERT(IS_PWM_INSTANCE(hpwm->Instance));
    HAL_ASSERT(IS_PWM_CHANNELS(Channel));

    __HAL_PWM_ENABLE_IT(hpwm, Channel, PWM_IT_CC);

    /* Enable the Capture compare channel */
    PWM_CCxChannelCmd(hpwm->Instance, Channel, PWM_CCx_ENABLE);

    /* Enable the Peripheral */
    __HAL_PWM_ENABLE(hpwm, Channel);

    /* Return function status */
    return HAL_OK;
}

/**
 * @brief  Stops the PWM signal generation in interrupt mode.
 * @param  hpwm pointer to a PWM_HandleTypeDef structure that contains
 *                the configuration information for TIM module.
 * @param  Channel TIM Channels to be disabled.
 *          This parameter can be one of the following values:
 *            @arg PWM_CHANNEL_1: TIM Channel 1 selected
 *            @arg PWM_CHANNEL_2: TIM Channel 2 selected
 *            @arg PWM_CHANNEL_3: TIM Channel 3 selected
 *            @arg PWM_CHANNEL_4: TIM Channel 4 selected
 * @retval HAL status
 */
__HAL_ROM_USED HAL_StatusTypeDef HAL_PWM_Stop_IT(PWM_HandleTypeDef *hpwm, uint32_t Channel)
{
    /* Check the parameters */
    HAL_ASSERT(IS_PWM_INSTANCE(hpwm->Instance));
    HAL_ASSERT(IS_PWM_CHANNELS(Channel));

    __HAL_PWM_DISABLE_IT(hpwm, Channel, PWM_IT_CC);

    /* Disable the Capture compare channel */
    PWM_CCxChannelCmd(hpwm->Instance, Channel, PWM_CCx_DISABLE);

    /* Disable the Peripheral */
    __HAL_PWM_DISABLE(hpwm, Channel);

    /* Return function status */
    return HAL_OK;
}

/**
 * @brief  Starts the TIM PWM signal generation in CCX DMA mode.
 * @param  hpwm pointer to a PWM_HandleTypeDef structure that contains
 *                the configuration information for TIM module.
 * @param  Channel TIM Channels to be enabled.
 *          This parameter can be one of the following values:
 *            @arg PWM_CHANNEL_1: TIM Channel 1 selected
 *            @arg PWM_CHANNEL_2: TIM Channel 2 selected
 *            @arg PWM_CHANNEL_3: TIM Channel 3 selected
 *            @arg PWM_CHANNEL_4: TIM Channel 4 selected
 * @param  pData The source Buffer address.
 * @param  Length The length of data to be transferred from memory to TIM peripheral
 * @retval HAL status
 */
__HAL_ROM_USED HAL_StatusTypeDef HAL_PWM_Start_DMA(PWM_HandleTypeDef *hpwm, uint32_t Channel, uint32_t *pData, uint16_t Length)
{
    /* Check the parameters */
    HAL_ASSERT(IS_PWM_INSTANCE(hpwm->Instance));
    HAL_ASSERT(IS_PWM_CHANNELS(Channel));
//TODO:
#if 0
    if (hpwm->State == HAL_PWM_STATE_BUSY)
    {
        return HAL_BUSY;
    }
    else if (hpwm->State == HAL_PWM_STATE_READY)
    {
        if (((uint32_t)pData == 0U) && (Length > 0))
        {
            return HAL_ERROR;
        }
        else
        {
            hpwm->State = HAL_PWM_STATE_BUSY;
        }
    }
    switch (Channel)
    {
    case PWM_CHANNEL_1:
    {
        /* Set the DMA Period elapsed callback */
        hpwm->hdma[PWM_DMA_ID_CC1]->XferCpltCallback = PWM_DMADelayPulseCplt;

        /* Set the DMA error callback */
        hpwm->hdma[PWM_DMA_ID_CC1]->XferErrorCallback = PWM_DMAError ;

        /* Enable the DMA Stream */
        HAL_DMA_Start_IT(hpwm->hdma[PWM_DMA_ID_CC1], (uint32_t)pData, (uint32_t)&hpwm->Instance->CCR1, Length);

        /* Enable the TIM Capture/Compare 1 DMA request */
        __HAL_PWM_ENABLE_DMA(hpwm, PWM_DMA_CC1);
    }
    break;

    case PWM_CHANNEL_2:
    {
        /* Set the DMA Period elapsed callback */
        hpwm->hdma[PWM_DMA_ID_CC2]->XferCpltCallback = PWM_DMADelayPulseCplt;

        /* Set the DMA error callback */
        hpwm->hdma[PWM_DMA_ID_CC2]->XferErrorCallback = PWM_DMAError ;

        /* Enable the DMA Stream */
        HAL_DMA_Start_IT(hpwm->hdma[PWM_DMA_ID_CC2], (uint32_t)pData, (uint32_t)&hpwm->Instance->CCR2, Length);

        /* Enable the TIM Capture/Compare 2 DMA request */
        __HAL_PWM_ENABLE_DMA(hpwm, PWM_DMA_CC2);
    }
    break;

    case PWM_CHANNEL_3:
    {
        /* Set the DMA Period elapsed callback */
        hpwm->hdma[PWM_DMA_ID_CC3]->XferCpltCallback = PWM_DMADelayPulseCplt;

        /* Set the DMA error callback */
        hpwm->hdma[PWM_DMA_ID_CC3]->XferErrorCallback = PWM_DMAError ;

        /* Enable the DMA Stream */
        HAL_DMA_Start_IT(hpwm->hdma[PWM_DMA_ID_CC3], (uint32_t)pData, (uint32_t)&hpwm->Instance->CCR3, Length);

        /* Enable the TIM Output Capture/Compare 3 request */
        __HAL_PWM_ENABLE_DMA(hpwm, PWM_DMA_CC3);
    }
    break;

    case PWM_CHANNEL_4:
    {
        /* Set the DMA Period elapsed callback */
        hpwm->hdma[PWM_DMA_ID_CC4]->XferCpltCallback = PWM_DMADelayPulseCplt;

        /* Set the DMA error callback */
        hpwm->hdma[PWM_DMA_ID_CC4]->XferErrorCallback = PWM_DMAError ;

        /* Enable the DMA Stream */
        HAL_DMA_Start_IT(hpwm->hdma[PWM_DMA_ID_CC4], (uint32_t)pData, (uint32_t)&hpwm->Instance->CCR4, Length);

        /* Enable the TIM Capture/Compare 4 DMA request */
        __HAL_PWM_ENABLE_DMA(hpwm, PWM_DMA_CC4);
    }
    break;

    default:
        break;
    }

    /* Enable the Capture compare channel */
    PWM_CCxChannelCmd(hpwm->Instance, Channel, PWM_CCx_ENABLE);

    if (IS_PWM_ADVANCED_INSTANCE(hpwm->Instance) != RESET)
    {
        /* Enable the main output */
        __HAL_PWM_MOE_ENABLE(hpwm);
    }

    /* Enable the Peripheral */
    __HAL_PWM_ENABLE(hpwm);
#endif
    /* Return function status */
    return HAL_OK;
}

/**
 * @brief  Initializes the TIM PWM  channels according to the specified
 *         parameters in the PWM_OC_InitTypeDef.
 * @param  hpwm pointer to a PWM_HandleTypeDef structure that contains
 *                the configuration information for TIM module.
 * @param  sConfig TIM PWM configuration structure
 * @param  Channel TIM Channels to be enabled.
 *          This parameter can be one of the following values:
 *            @arg PWM_CHANNEL_1: TIM Channel 1 selected
 *            @arg PWM_CHANNEL_2: TIM Channel 2 selected
 *            @arg PWM_CHANNEL_3: TIM Channel 3 selected
 *            @arg PWM_CHANNEL_4: TIM Channel 4 selected
 * @retval HAL status
 */
__HAL_ROM_USED HAL_StatusTypeDef HAL_PWM_ConfigChannel(PWM_HandleTypeDef *hpwm, PWM_OC_InitTypeDef *sConfig, uint32_t Channel)
{
    PWM_ChannelTypeDef *pwm_channel;
    __HAL_LOCK(hpwm);

    /* Check the parameters */
    HAL_ASSERT(IS_PWM_CHANNELS(Channel));
    HAL_ASSERT(IS_PWM_PWM_MODE(sConfig->OCMode));
    HAL_ASSERT(IS_PWM_OC_POLARITY(sConfig->OCPolarity));
    //HAL_ASSERT(IS_PWM_FAST_STATE(sConfig->OCFastMode));

    hpwm->State = HAL_PWM_STATE_BUSY;

    pwm_channel = (PWM_ChannelTypeDef *)&hpwm->Instance->CR1 + Channel;

    PWM_OC_SetConfig(pwm_channel, sConfig);
    /* Set the Preload enable bit */
    pwm_channel->CR |= PWM_CR1_OCPE;

    hpwm->State = HAL_PWM_STATE_READY;

    __HAL_UNLOCK(hpwm);

    return HAL_OK;
}


/**
 * @brief  Generate a software event
 * @param  htim pointer to a GPT_HandleTypeDef structure that contains
 *                the configuration information for TIM module.
 * @param  EventSource specifies the event source.
 *          This parameter can be one of the following values:
 *            @arg GPT_EVENTSOURCE_UPDATE: Timer update Event source
 *            @arg GPT_EVENTSOURCE_CC1: Timer Capture Compare 1 Event source
 *            @arg GPT_EVENTSOURCE_CC2: Timer Capture Compare 2 Event source
 *            @arg GPT_EVENTSOURCE_CC3: Timer Capture Compare 3 Event source
 *            @arg GPT_EVENTSOURCE_CC4: Timer Capture Compare 4 Event source
 *            @arg GPT_EVENTSOURCE_COM: Timer COM event source
 *            @arg GPT_EVENTSOURCE_TRIGGER: Timer Trigger Event source
 *            @arg GPT_EVENTSOURCE_BREAK: Timer Break event source
 * @note   TIM6 and TIM7 can only generate an update event.
 * @note   GPT_EVENTSOURCE_COM and GPT_EVENTSOURCE_BREAK are used only with TIM1 and TIM8.
 * @retval HAL status
 */

__HAL_ROM_USED HAL_StatusTypeDef HAL_PWM_GenerateEvent(PWM_HandleTypeDef *hpwm, uint32_t EventSource)
{
    /* Check the parameters */
    HAL_ASSERT(IS_PWM_INSTANCE(hpwm->Instance));
    HAL_ASSERT(IS_GPT_EVENT_SOURCE(EventSource));

    /* Process Locked */
    __HAL_LOCK(hpwm);

    /* Change the TIM state */
    hpwm->State = HAL_PWM_STATE_BUSY;

    /* Set the event sources */
    hpwm->Instance->EGR = EventSource;

    /* Change the TIM state */
    hpwm->State = HAL_PWM_STATE_READY;

    __HAL_UNLOCK(hpwm);

    /* Return function status */
    return HAL_OK;
}

__weak void HAL_PWM_ErrorCallback(PWM_HandleTypeDef *hpwm)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hpwm);
    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_PWM_ErrorCallback could be implemented in the user file
     */
}


/** @defgroup PWM_Exported_Functions_Group10 Peripheral State functions
 *  @brief   Peripheral State functions
 *
@verbatim
  ==============================================================================
                        ##### Peripheral State functions #####
  ==============================================================================
  [..]
    This subsection permits to get in run-time the status of the peripheral
    and the data flow.

@endverbatim
  * @{
  */

/**
 * @brief  Return the TIM Base state
 * @param  hpwm pointer to a PWM_HandleTypeDef structure that contains
 *                the configuration information for TIM module.
 * @retval HAL state
 */
__HAL_ROM_USED HAL_PWM_StateTypeDef HAL_PWM_GetState(PWM_HandleTypeDef *hpwm)
{
    return hpwm->State;
}

/**
 * @brief  TIM DMA error callback
 * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
 *                the configuration information for the specified DMA module.
 * @retval None
 */
__HAL_ROM_USED void PWM_DMAError(DMA_HandleTypeDef *hdma)
{
    PWM_HandleTypeDef *hpwm = (PWM_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

    hpwm->State = HAL_PWM_STATE_READY;

    HAL_PWM_ErrorCallback(hpwm);
}

/**
 * @brief  Enables or disables the TIM Capture Compare Channel x.
 * @param  TIMx to select the TIM peripheral
 * @param  Channel specifies the TIM Channel
 *          This parameter can be one of the following values:
 *            @arg PWM_Channel_1: TIM Channel 1
 *            @arg PWM_Channel_2: TIM Channel 2
 *            @arg PWM_Channel_3: TIM Channel 3
 *            @arg PWM_Channel_4: TIM Channel 4
 * @param  ChannelState specifies the TIM Channel CCxE bit new state.
 *          This parameter can be: PWM_CCx_ENABLE or PWM_CCx_Disable.
 * @retval None
 */
void PWM_CCxChannelCmd(PWM_TypeDef *PWMx, uint32_t Channel, uint32_t ChannelState)
{
    PWM_ChannelTypeDef *pwm_channel;

    /* Check the parameters */
    HAL_ASSERT(IS_PWM_INSTANCE(PWMx));
    HAL_ASSERT(IS_PWM_CHANNELS(Channel));

    pwm_channel = (PWM_ChannelTypeDef *)&PWMx->CR1 + Channel;
    /* reset CCE bit */
    pwm_channel->CR &= ~PWM_CR1_CCE;
    pwm_channel->CR |= ChannelState;
}


/**
 * @brief  Time Output Compare 1 configuration
 * @param  TIMx to select the TIM peripheral
 * @param  OC_Config The output configuration structure
 * @retval None
 */
static void PWM_OC_SetConfig(PWM_ChannelTypeDef *PWM_Channel, PWM_OC_InitTypeDef *OC_Config)
{
    uint32_t tmpcr;

    /* Disable the Channel 1: Reset the CC1E Bit */
    PWM_Channel->CR &= ~PWM_CR1_CCE;

    tmpcr = PWM_Channel->CR;

    /* Reset the Output Compare Mode Bits */
    tmpcr &= ~PWM_CR1_OCM;
    /* Select the Output Compare Mode */
    tmpcr |= OC_Config->OCMode;

    /* Reset the Output Polarity level */
    tmpcr &= ~PWM_CR1_CCP;
    /* Set the Output Compare Polarity */
    tmpcr |= OC_Config->OCPolarity;

    /* Set the Capture Compare Register value */
    PWM_Channel->CCR = OC_Config->Pulse;

    /* Write to CR */
    PWM_Channel->CR = tmpcr;
}

#endif /* HAL_PWM_MODULE_ENABLED */
/**
 * @}
 */

/**
 * @}
 */