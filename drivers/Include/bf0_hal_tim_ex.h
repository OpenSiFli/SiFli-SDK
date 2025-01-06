/**
  ******************************************************************************
  * @file   bf0_hal_tim_ex.h
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

#ifndef __BF0_HAL_TIM_EX_H
#define __BF0_HAL_TIM_EX_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal_def.h"


/** @addtogroup TIMEx
  * @ingroup TIM
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup TIMEx_Exported_Types TIM Exported Types
  * @{
  */

/**
  * @brief  TIM Hall sensor Configuration Structure definition
  */

typedef struct
{

    uint32_t IC1Polarity;            /*!< Specifies the active edge of the input signal.
                                        This parameter can be a value of @ref GPT_Input_Capture_Polarity */

    uint32_t IC1Prescaler;        /*!< Specifies the Input Capture Prescaler.
                                     This parameter can be a value of @ref GPT_Input_Capture_Prescaler */

    uint32_t IC1Filter;           /*!< Specifies the input capture filter.
                                     This parameter can be a number between Min_Data = 0x0 and Max_Data = 0xF */

    uint32_t Commutation_Delay;  /*!< Specifies the pulse value to be loaded into the Capture Compare Register.
                                    This parameter can be a number between Min_Data = 0x0000U and Max_Data = 0xFFFFU */
} GPT_HallSensor_InitTypeDef;

/**
  * @brief  TIM Break/Break2 input configuration
  */
typedef struct
{
    uint32_t Source;         /*!< Specifies the source of the timer break input.*/
    uint32_t Enable;         /*!< Specifies whether or not the break input source is enabled.*/
    uint32_t Polarity;       /*!< Specifies the break input source polarity.*/
}
TIMEx_BreakInputConfigTypeDef;

/**
  * @brief  TIM Master configuration Structure definition
  * @note   Advanced timers provide TRGO2 internal line which is redirected
  *         to the ADC
  */
typedef struct
{
    uint32_t  MasterOutputTrigger;   /*!< Trigger output (TRGO) selection */
    uint32_t  MasterOutputTrigger2;  /*!< Trigger output2 (TRGO2) selection*/
    uint32_t  MasterSlaveMode;       /*!< Master/slave mode selection
                                        @note When the Master/slave mode is enabled, the effect of
                                        an event on the trigger input (TRGI) is delayed to allow a
                                        perfect synchronization between the current timer and its
                                        slaves (through TRGO). It is not mandatory in case of timer
                                        synchronization mode. */
} TIMEx_MasterConfigTypeDef;


/**
  * @brief  TIM Break input(s) and Dead time configuration Structure definition
  * @note   2 break inputs can be configured (BKIN and BKIN2) with configurable
  *        filter and polarity.
  */
typedef struct
{
    uint32_t OffStateRunMode;      /*!< TIM off state in run mode*/
    uint32_t OffStateIDLEMode;     /*!< TIM off state in IDLE mode*/
    uint32_t LockLevel;            /*!< TIM Lock level*/
    uint32_t DeadTime;             /*!< TIM dead Time
                                      This parameter can be a number between Min_Data = 0x00 and Max_Data = 0xFF */
    uint32_t BreakState;           /*!< TIM Break State */
    uint32_t BreakPolarity;        /*!< TIM Break input polarity*/
    uint32_t BreakFilter;          /*!< Specifies the break input filter.
                                      This parameter can be a number between Min_Data = 0x0 and Max_Data = 0xF */
    uint32_t Break2State;          /*!< TIM Break2 State */
    uint32_t Break2Polarity;       /*!< TIM Break2 input polarity*/
    uint32_t Break2Filter;         /*!< TIM break2 input filter.
                                      This parameter can be a number between Min_Data = 0x0 and Max_Data = 0xF */
    uint32_t AutomaticOutput;      /*!< TIM Automatic Output Enable state */
    uint32_t DeadTimePsc;           /*dead time prescaler*/
} TIMEx_BreakDeadTimeConfigTypeDef;


/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup TIMEx_Exported_Constants  TIM Extended Exported Constants
  * @{
  */

#if defined(TIM_BREAK_INPUT_SUPPORT)

/** @defgroup TIMEx_Break_Input TIM Extended Break input
  * @{
  */
#define TIM_BREAKINPUT_BRK     0x00000001U                                      /* !< Timer break input  */
#define TIM_BREAKINPUT_BRK2    0x00000002U                                      /* !< Timer break2 input */
/**
  * @}
  */

/** @defgroup TIMEx_Break_Input_Source TIM Extended Break input source
  * @{
  */
#define TIM_BREAKINPUTSOURCE_BKIN     0x00000001U                               /* !< An external source (GPIO) is connected to the BKIN pin  */
#define TIM_BREAKINPUTSOURCE_COMP1    0x00000002U                               /* !< The COMP1 output is connected to the break input */
#define TIM_BREAKINPUTSOURCE_COMP2    0x00000004U                               /* !< The COMP2 output is connected to the break input */
#define TIM_BREAKINPUTSOURCE_DFSDM1   0x00000008U                               /* !< The analog watchdog output of the DFSDM1 peripheral is connected to the break input */
/**
  * @}
  */

/** @defgroup TIMEx_Break_Input_Source_Enable TIM Extended Break input source enabling
  * @{
  */
#define TIM_BREAKINPUTSOURCE_DISABLE     0x00000000U                            /* !< Break input source is disabled */
#define TIM_BREAKINPUTSOURCE_ENABLE      0x00000001U                            /* !< Break input source is enabled */
/**
  * @}
  */

/** @defgroup TIMEx_Break_Input_Source_Polarity TIM Extended Break input polarity
  * @{
  */
#define TIM_BREAKINPUTSOURCE_POLARITY_LOW     0x00000001U                       /* !< Break input source is active low */
#define TIM_BREAKINPUTSOURCE_POLARITY_HIGH    0x00000000U                       /* !< Break input source is active_high */
/**
  * @}
  */
#endif /* TIM_BREAK_INPUT_SUPPORT */


/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @addtogroup TIMEx_Exported_Functions
  * @{
  */

/** @addtogroup TIMEx_Exported_Functions_Group1
  * @{
  */
/*  Timer Hall Sensor functions  **********************************************/
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Init(GPT_HandleTypeDef *htim, GPT_HallSensor_InitTypeDef *sConfig);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_DeInit(GPT_HandleTypeDef *htim);

void HAL_TIMEx_HallSensor_MspInit(GPT_HandleTypeDef *htim);
void HAL_TIMEx_HallSensor_MspDeInit(GPT_HandleTypeDef *htim);

/* Blocking mode: Polling */
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Start(GPT_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Stop(GPT_HandleTypeDef *htim);
/* Non-Blocking mode: Interrupt */
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Start_IT(GPT_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Stop_IT(GPT_HandleTypeDef *htim);
/* Non-Blocking mode: DMA */
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Start_DMA(GPT_HandleTypeDef *htim, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Stop_DMA(GPT_HandleTypeDef *htim);
/**
  * @}
  */

/** @addtogroup TIMEx_Exported_Functions_Group2
  * @{
  */
/*  Timer Complementary Output Compare functions  *****************************/
/* Blocking mode: Polling */
HAL_StatusTypeDef HAL_TIMEx_OCN_Start(GPT_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop(GPT_HandleTypeDef *htim, uint32_t Channel);

/* Non-Blocking mode: Interrupt */
HAL_StatusTypeDef HAL_TIMEx_OCN_Start_IT(GPT_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop_IT(GPT_HandleTypeDef *htim, uint32_t Channel);

/* Non-Blocking mode: DMA */
HAL_StatusTypeDef HAL_TIMEx_OCN_Start_DMA(GPT_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop_DMA(GPT_HandleTypeDef *htim, uint32_t Channel);
/**
  * @}
  */

/** @addtogroup TIMEx_Exported_Functions_Group3
  * @{
  */
/*  Timer Complementary PWM functions  ****************************************/
/* Blocking mode: Polling */
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start(GPT_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop(GPT_HandleTypeDef *htim, uint32_t Channel);

/* Non-Blocking mode: Interrupt */
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start_IT(GPT_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop_IT(GPT_HandleTypeDef *htim, uint32_t Channel);
/* Non-Blocking mode: DMA */
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start_DMA(GPT_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop_DMA(GPT_HandleTypeDef *htim, uint32_t Channel);
/**
  * @}
  */

/** @addtogroup TIMEx_Exported_Functions_Group4
  * @{
  */
/*  Timer Complementary One Pulse functions  **********************************/
/* Blocking mode: Polling */
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Start(GPT_HandleTypeDef *htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Stop(GPT_HandleTypeDef *htim, uint32_t OutputChannel);

/* Non-Blocking mode: Interrupt */
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Start_IT(GPT_HandleTypeDef *htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Stop_IT(GPT_HandleTypeDef *htim, uint32_t OutputChannel);
/**
  * @}
  */

/** @addtogroup TIMEx_Exported_Functions_Group5
  * @{
  */
/* Extension Control functions  ************************************************/
HAL_StatusTypeDef HAL_TIMEx_ConfigCommutationEvent(GPT_HandleTypeDef *htim, uint32_t  InputTrigger, uint32_t  CommutationSource);
HAL_StatusTypeDef HAL_TIMEx_ConfigCommutationEvent_IT(GPT_HandleTypeDef *htim, uint32_t  InputTrigger, uint32_t  CommutationSource);
HAL_StatusTypeDef HAL_TIMEx_ConfigCommutationEvent_DMA(GPT_HandleTypeDef *htim, uint32_t  InputTrigger, uint32_t  CommutationSource);
HAL_StatusTypeDef HAL_TIMEx_MasterConfigSynchronization(GPT_HandleTypeDef *htim, TIMEx_MasterConfigTypeDef *sMasterConfig);
HAL_StatusTypeDef HAL_TIMEx_ConfigBreakDeadTime(GPT_HandleTypeDef *htim, TIMEx_BreakDeadTimeConfigTypeDef *sBreakDeadTimeConfig);
HAL_StatusTypeDef HAL_TIMEx_RemapConfig(GPT_HandleTypeDef *htim, uint32_t Remap);
/**
  * @}
  */

/** @addtogroup TIMEx_Exported_Functions_Group6
  * @{
  */
/* Extension Callback *********************************************************/
void HAL_TIMEx_CommutationCallback(GPT_HandleTypeDef *htim);
void HAL_TIMEx_BreakCallback(GPT_HandleTypeDef *htim);
void HAL_TIMEx_Break2Callback(GPT_HandleTypeDef *htim);
void TIMEx_DMACommutationCplt(DMA_HandleTypeDef *hdma);
void TIMEx_DMACommutationHalfCplt(DMA_HandleTypeDef *hdma);
/**
  * @}
  */

/** @addtogroup TIMEx_Exported_Functions_Group7
  * @{
  */
/* Extension Peripheral State functions  **************************************/
HAL_GPT_StateTypeDef HAL_TIMEx_HallSensor_GetState(GPT_HandleTypeDef *htim);
HAL_GPT_ChannelStateTypeDef HAL_TIMEx_GetChannelNState(GPT_HandleTypeDef *htim,  uint32_t ChannelN);

/**
  * @}
  */

/**
  * @}
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/** @defgroup TIMEx_Private_Macros TIM Private Macros
  * @{
  */
#define IS_GPT_REMAP(GPT_REMAP)   (((GPT_REMAP) == GPT_TIM2_TIM8_TRGO)||\
                                  ((GPT_REMAP) == GPT_TIM2_ETH_PTP)||\
                                  ((GPT_REMAP) == GPT_TIM2_USBFS_SOF)||\
                                  ((GPT_REMAP) == GPT_TIM2_USBHS_SOF)||\
                                  ((GPT_REMAP) == GPT_TIM5_GPIO)||\
                                  ((GPT_REMAP) == GPT_TIM5_LSI)||\
                                  ((GPT_REMAP) == GPT_TIM5_LSE)||\
                                  ((GPT_REMAP) == GPT_TIM5_RTC)||\
                                  ((GPT_REMAP) == GPT_TIM11_GPIO)||\
                                  ((GPT_REMAP) == GPT_TIM11_HSE))

#define GPT_CHANNEL_STATE_GET(__HANDLE__, __CHANNEL__)\
  (((__CHANNEL__) == TIM_CHANNEL_1) ? (__HANDLE__)->ChannelState[0] :\
   ((__CHANNEL__) == TIM_CHANNEL_2) ? (__HANDLE__)->ChannelState[1] :\
   ((__CHANNEL__) == TIM_CHANNEL_3) ? (__HANDLE__)->ChannelState[2] :\
   ((__CHANNEL__) == TIM_CHANNEL_4) ? (__HANDLE__)->ChannelState[3] :\
   ((__CHANNEL__) == TIM_CHANNEL_5) ? (__HANDLE__)->ChannelState[4] :\
   (__HANDLE__)->ChannelState[5])

#define GPT_CHANNEL_STATE_SET(__HANDLE__, __CHANNEL__, __CHANNEL_STATE__) \
  (((__CHANNEL__) == TIM_CHANNEL_1) ? ((__HANDLE__)->ChannelState[0] = (__CHANNEL_STATE__)) :\
   ((__CHANNEL__) == TIM_CHANNEL_2) ? ((__HANDLE__)->ChannelState[1] = (__CHANNEL_STATE__)) :\
   ((__CHANNEL__) == TIM_CHANNEL_3) ? ((__HANDLE__)->ChannelState[2] = (__CHANNEL_STATE__)) :\
   ((__CHANNEL__) == TIM_CHANNEL_4) ? ((__HANDLE__)->ChannelState[3] = (__CHANNEL_STATE__)) :\
   ((__CHANNEL__) == TIM_CHANNEL_5) ? ((__HANDLE__)->ChannelState[4] = (__CHANNEL_STATE__)) :\
   ((__HANDLE__)->ChannelState[5] = (__CHANNEL_STATE__)))

#define GPT_CHANNEL_N_STATE_GET(__HANDLE__, __CHANNEL__)\
  (((__CHANNEL__) == TIM_CHANNEL_1) ? (__HANDLE__)->ChannelNState[0] :\
   ((__CHANNEL__) == TIM_CHANNEL_2) ? (__HANDLE__)->ChannelNState[1] :\
   ((__CHANNEL__) == TIM_CHANNEL_3) ? (__HANDLE__)->ChannelNState[2] :\
   (__HANDLE__)->ChannelNState[3])

#define GPT_CHANNEL_N_STATE_SET(__HANDLE__, __CHANNEL__, __CHANNEL_STATE__) \
  (((__CHANNEL__) == TIM_CHANNEL_1) ? ((__HANDLE__)->ChannelNState[0] = (__CHANNEL_STATE__)) :\
   ((__CHANNEL__) == TIM_CHANNEL_2) ? ((__HANDLE__)->ChannelNState[1] = (__CHANNEL_STATE__)) :\
   ((__CHANNEL__) == TIM_CHANNEL_3) ? ((__HANDLE__)->ChannelNState[2] = (__CHANNEL_STATE__)) :\
   ((__HANDLE__)->ChannelNState[3] = (__CHANNEL_STATE__)))


#define IS_GPT_DEADTIME(DEADTIME)      ((DEADTIME) <= 0xFFU)

#if 0
/**
  * @brief  Gets the TIM dead-time prescaler.
  * @param  \__HANDLE__ TIM handle.
  * @retval 1-bit value of dead-time prescaler, 0:dead-time is tCLK*(DTG+1), 1:dead-time is tCLK*(DTG+1)*16.
  */
#define __HAL_GPT_GET_DTPSC(__HANDLE__) (((__HANDLE__)->Instance->BDTR >> ATIM_BDTR_DTPSC_Pos) & 0x1UL)
#endif

/**
  * @}
  */

/* Private functions ---------------------------------------------------------*/
/** @defgroup TIMEx_Private_Functions TIM Private Functions
  * @{
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __BF0_HAL_GPT_EX_H */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
