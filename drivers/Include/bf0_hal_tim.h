/**
  ******************************************************************************
  * @file   bf0_hal_tim.h
  * @author Sifli software development team
  * @brief   Header file of TIM HAL module.
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

#ifndef __BF0_HAL_GPT_H
#define __BF0_HAL_GPT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal_def.h"

/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @addtogroup TIM Hardware Timer
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup GPT_Exported_Types TIM Exported Types
  * @{
  */

/**
  * @brief  TIM Time base Configuration Structure definition
  */
typedef struct
{
    uint32_t Prescaler;         /*!< Specifies the prescaler value used to divide the TIM clock.
                                   This parameter can be a number between Min_Data = 0x0000U and Max_Data = 0xFFFFU */

    uint32_t CounterMode;       /*!< Specifies the counter mode.
                                   This parameter can be a value of @ref GPT_Counter_Mode */

    uint32_t Period;            /*!< Specifies the period value to be loaded into the active
                                   Auto-Reload Register at the next update event.
                                   This parameter can be a number between Min_Data = 0x0000U and Max_Data = 0xFFFF.  */

    uint32_t RepetitionCounter;  /*!< Specifies the repetition counter value. Each time the RCR downcounter
                                    reaches zero, an update event is generated and counting restarts
                                    from the RCR value (N).
                                    This means in PWM mode that (N+1) corresponds to:
                                        - the number of PWM periods in edge-aligned mode
                                        - the number of half PWM period in center-aligned mode
                                     This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF.
                                     @note This parameter is valid only for TIM1 and TIM8. */
} GPT_Base_InitTypeDef;

/**
  * @brief  TIM Output Compare Configuration Structure definition
  */

typedef struct
{
    uint32_t OCMode;        /*!< Specifies the TIM mode.
                               This parameter can be a value of @ref GPT_Output_Compare_and_PWM_modes */

    uint32_t Pulse;         /*!< Specifies the pulse value to be loaded into the Capture Compare Register.
                               This parameter can be a number between Min_Data = 0x0000U and Max_Data = 0xFFFFU */

    uint32_t OCPolarity;    /*!< Specifies the output polarity.
                               This parameter can be a value of @ref GPT_Output_Compare_Polarity */

    uint32_t OCNPolarity;   /*!< Specifies the complementary output polarity.
                               This parameter can be a value of @ref GPT_Output_Compare_N_Polarity
                               @note This parameter is valid only for TIM1 and TIM8. */

    uint32_t OCFastMode;   /*!< Specifies the Fast mode state.
                               This parameter can be a value of @ref GPT_Output_Fast_State
                               @note This parameter is valid only in PWM1 and PWM2 mode. */


    uint32_t OCIdleState;   /*!< Specifies the TIM Output Compare pin state during Idle state.
                               This parameter can be a value of @ref GPT_Output_Compare_Idle_State
                               @note This parameter is valid only for TIM1 and TIM8. */

    uint32_t OCNIdleState;  /*!< Specifies the TIM Output Compare pin state during Idle state.
                               This parameter can be a value of @ref GPT_Output_Compare_N_Idle_State
                               @note This parameter is valid only for TIM1 and TIM8. */
} GPT_OC_InitTypeDef;

/**
  * @brief  TIM One Pulse Mode Configuration Structure definition
  */
typedef struct
{
    uint32_t OCMode;        /*!< Specifies the TIM mode.
                               This parameter can be a value of @ref GPT_Output_Compare_and_PWM_modes */

    uint32_t Pulse;         /*!< Specifies the pulse value to be loaded into the Capture Compare Register.
                               This parameter can be a number between Min_Data = 0x0000U and Max_Data = 0xFFFFU */

    uint32_t OCPolarity;    /*!< Specifies the output polarity.
                               This parameter can be a value of @ref GPT_Output_Compare_Polarity */

    uint32_t OCNPolarity;   /*!< Specifies the complementary output polarity.
                               This parameter can be a value of @ref GPT_Output_Compare_N_Polarity
                               @note This parameter is valid only for TIM1 and TIM8. */

    uint32_t OCIdleState;   /*!< Specifies the TIM Output Compare pin state during Idle state.
                               This parameter can be a value of @ref GPT_Output_Compare_Idle_State
                               @note This parameter is valid only for TIM1 and TIM8. */

    uint32_t OCNIdleState;  /*!< Specifies the TIM Output Compare pin state during Idle state.
                               This parameter can be a value of @ref GPT_Output_Compare_N_Idle_State
                               @note This parameter is valid only for TIM1 and TIM8. */

    uint32_t ICPolarity;    /*!< Specifies the active edge of the input signal.
                               This parameter can be a value of @ref GPT_Input_Capture_Polarity */

    uint32_t ICSelection;   /*!< Specifies the input.
                              This parameter can be a value of @ref GPT_Input_Capture_Selection */

    uint32_t ICFilter;      /*!< Specifies the input capture filter.
                              This parameter can be a number between Min_Data = 0x0 and Max_Data = 0xF */
} GPT_OnePulse_InitTypeDef;


/**
  * @brief  TIM Input Capture Configuration Structure definition
  */

typedef struct
{
    uint32_t  ICPolarity;   /*!< Specifies the active edge of the input signal.
                               This parameter can be a value of @ref GPT_Input_Capture_Polarity */

    uint32_t ICSelection;  /*!< Specifies the input.
                              This parameter can be a value of @ref GPT_Input_Capture_Selection */

    uint32_t ICPrescaler;  /*!< Specifies the Input Capture Prescaler.
                              This parameter can be a value of @ref GPT_Input_Capture_Prescaler */

    uint32_t ICFilter;     /*!< Specifies the input capture filter.
                              This parameter can be a number between Min_Data = 0x0 and Max_Data = 0xF */
} GPT_IC_InitTypeDef;

/**
  * @brief  TIM Encoder Configuration Structure definition
  */

typedef struct
{
    uint32_t EncoderMode;   /*!< Specifies the active edge of the input signal.
                               This parameter can be a value of @ref GPT_Encoder_Mode */

    uint32_t IC1Polarity;   /*!< Specifies the active edge of the input signal.
                               This parameter can be a value of @ref GPT_Input_Capture_Polarity */

    uint32_t IC1Selection;  /*!< Specifies the input.
                               This parameter can be a value of @ref GPT_Input_Capture_Selection */

    uint32_t IC1Prescaler;  /*!< Specifies the Input Capture Prescaler.
                               This parameter can be a value of @ref GPT_Input_Capture_Prescaler */

    uint32_t IC1Filter;     /*!< Specifies the input capture filter.
                               This parameter can be a number between Min_Data = 0x0 and Max_Data = 0xF */

    uint32_t IC2Polarity;   /*!< Specifies the active edge of the input signal.
                               This parameter can be a value of @ref GPT_Input_Capture_Polarity */

    uint32_t IC2Selection;  /*!< Specifies the input.
                              This parameter can be a value of @ref GPT_Input_Capture_Selection */

    uint32_t IC2Prescaler;  /*!< Specifies the Input Capture Prescaler.
                               This parameter can be a value of @ref GPT_Input_Capture_Prescaler */

    uint32_t IC2Filter;     /*!< Specifies the input capture filter.
                               This parameter can be a number between Min_Data = 0x0 and Max_Data = 0xF */
} GPT_Encoder_InitTypeDef;

/**
  * @brief  Clock Configuration Handle Structure definition
  */
typedef struct
{
    uint32_t ClockSource;     /*!< TIM clock sources.
                                 This parameter can be a value of @ref GPT_Clock_Source */
    uint32_t ClockPolarity;   /*!< TIM clock polarity.
                                 This parameter can be a value of @ref GPT_Clock_Polarity */
    uint32_t ClockPrescaler;  /*!< TIM clock prescaler.
                                 This parameter can be a value of @ref GPT_Clock_Prescaler */
    uint32_t ClockFilter;    /*!< TIM clock filter.
                                This parameter can be a number between Min_Data = 0x0 and Max_Data = 0xF */
} GPT_ClockConfigTypeDef;

/**
  * @brief  Clear Input Configuration Handle Structure definition
  */
typedef struct
{
    uint32_t ClearInputState;      /*!< TIM clear Input state.
                                      This parameter can be ENABLE or DISABLE */
    uint32_t ClearInputSource;     /*!< TIM clear Input sources.
                                      This parameter can be a value of @ref GPT_ClearInput_Source */
    uint32_t ClearInputPolarity;   /*!< TIM Clear Input polarity.
                                      This parameter can be a value of @ref GPT_ClearInput_Polarity */
    uint32_t ClearInputPrescaler;  /*!< TIM Clear Input prescaler.
                                      This parameter can be a value of @ref GPT_ClearInput_Prescaler */
    uint32_t ClearInputFilter;    /*!< TIM Clear Input filter.
                                     This parameter can be a number between Min_Data = 0x0 and Max_Data = 0xF */
} GPT_ClearInputConfigTypeDef;

/**
  * @brief  TIM Slave configuration Structure definition
  */
typedef struct
{
    uint32_t  SlaveMode;         /*!< Slave mode selection
                                  This parameter can be a value of @ref GPT_Slave_Mode */
    uint32_t  InputTrigger;      /*!< Input Trigger source
                                  This parameter can be a value of @ref GPT_Trigger_Selection */
    uint32_t  TriggerPolarity;   /*!< Input Trigger polarity
                                  This parameter can be a value of @ref GPT_Trigger_Polarity */
    uint32_t  TriggerPrescaler;  /*!< Input trigger prescaler
                                  This parameter can be a value of @ref GPT_Trigger_Prescaler */
    uint32_t  TriggerFilter;     /*!< Input trigger filter
                                  This parameter can be a number between Min_Data = 0x0 and Max_Data = 0xF */

} GPT_SlaveConfigTypeDef;

/**
  * @brief  HAL State structures definition
  */
typedef enum
{
    HAL_GPT_STATE_RESET             = 0x00U,    /*!< Peripheral not yet initialized or disabled  */
    HAL_GPT_STATE_READY             = 0x01U,    /*!< Peripheral Initialized and ready for use    */
    HAL_GPT_STATE_BUSY              = 0x02U,    /*!< An internal process is ongoing              */
    HAL_GPT_STATE_TIMEOUT           = 0x03U,    /*!< Timeout state                               */
    HAL_GPT_STATE_ERROR             = 0x04U     /*!< Reception process is ongoing                */
} HAL_GPT_StateTypeDef;

/**
  * @brief  HAL Active channel structures definition
  */
typedef enum
{
    HAL_GPT_ACTIVE_CHANNEL_1        = 0x01U,    /*!< The active channel is 1     */
    HAL_GPT_ACTIVE_CHANNEL_2        = 0x02U,    /*!< The active channel is 2     */
    HAL_GPT_ACTIVE_CHANNEL_3        = 0x04U,    /*!< The active channel is 3     */
    HAL_GPT_ACTIVE_CHANNEL_4        = 0x08U,    /*!< The active channel is 4     */
    HAL_GPT_ACTIVE_CHANNEL_CLEARED  = 0x00U     /*!< All active channels cleared */
} HAL_GPT_ActiveChannel;


/**
  * @brief  DMA Burst States definition
  */
typedef enum
{
    HAL_DMA_BURST_STATE_RESET             = 0x00U,    /*!< DMA Burst initial state */
    HAL_DMA_BURST_STATE_READY             = 0x01U,    /*!< DMA Burst ready for use */
    HAL_DMA_BURST_STATE_BUSY              = 0x02U,    /*!< Ongoing DMA Burst       */
} HAL_GPT_DMABurstStateTypeDef;

/**
  * @brief  TIM Channel States definition
  */
typedef enum
{
    HAL_GPT_CHANNEL_STATE_RESET             = 0x00U,    /*!< TIM Channel initial state                         */
    HAL_GPT_CHANNEL_STATE_READY             = 0x01U,    /*!< TIM Channel ready for use                         */
    HAL_GPT_CHANNEL_STATE_BUSY              = 0x02U,    /*!< An internal process is ongoing on the TIM channel */
} HAL_GPT_ChannelStateTypeDef;

/**
  * @brief  TIM Time Base Handle Structure definition
  */
typedef struct
{
    GPT_TypeDef                 *Instance;     /*!< Register base address             */
    GPT_Base_InitTypeDef        Init;          /*!< TIM Time Base required parameters */
    HAL_GPT_ActiveChannel       Channel;       /*!< Active channel                    */
    DMA_HandleTypeDef           *hdma[7];      /*!< DMA Handlers array
                                             This array is accessed by a @ref DMA_Handle_index */
    HAL_LockTypeDef             Lock;          /*!< Locking object                    */
    uint8_t                     core;          /*!< Clock source from which core*/
    __IO HAL_GPT_StateTypeDef   State;         /*!< TIM operation state               */
#ifdef  HAL_ATIM_MODULE_ENABLED
    __IO HAL_GPT_ChannelStateTypeDef   ChannelState[6];   /*!< TIM channel operation state                       */
    __IO HAL_GPT_ChannelStateTypeDef   ChannelNState[4];  /*!< TIM complementary channel operation state         */
    __IO HAL_GPT_DMABurstStateTypeDef  DMABurstState;     /*!< DMA burst operation state                         */
#endif
} GPT_HandleTypeDef;


/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup GPT_Exported_Constants  TIM Exported Constants
  * @{
  */

/** @defgroup GPT_Input_Channel_Polarity TIM Input Channel Polarity
  * @{
  */
#define  GPT_INPUTCHANNELPOLARITY_RISING      0x00000000U            /*!< Polarity for TIx source */
#define  GPT_INPUTCHANNELPOLARITY_FALLING     (GPT_CCER_CC1P)                   /*!< Polarity for TIx source */
#define  GPT_INPUTCHANNELPOLARITY_BOTHEDGE    (GPT_CCER_CC1P | GPT_CCER_CC1NP)  /*!< Polarity for TIx source */
/**
  * @}
  */

/** @defgroup GPT_ETR_Polarity  TIM ETR Polarity
  * @{
  */
#define GPT_ETRPOLARITY_INVERTED              (GPT_SMCR_ETP)                    /*!< Polarity for ETR source */
#define GPT_ETRPOLARITY_NONINVERTED           0x00000000U                /*!< Polarity for ETR source */
/**
  * @}
  */

/** @defgroup GPT_ETR_Prescaler  TIM ETR Prescaler
  * @{
  */
#define GPT_ETRPRESCALER_DIV1                 0x00000000U                /*!< No prescaler is used */
#define GPT_ETRPRESCALER_DIV2                 (GPT_SMCR_ETPS_0)                 /*!< ETR input source is divided by 2 */
#define GPT_ETRPRESCALER_DIV4                 (GPT_SMCR_ETPS_1)                 /*!< ETR input source is divided by 4 */
#define GPT_ETRPRESCALER_DIV8                 (GPT_SMCR_ETPS)                   /*!< ETR input source is divided by 8 */
/**
  * @}
  */

/** @defgroup GPT_Counter_Mode  TIM Counter Mode
  * @{
  */
#define GPT_COUNTERMODE_UP                 0x00000000U
#define GPT_COUNTERMODE_DOWN               GPT_CR1_DIR
#define GPT_COUNTERMODE_CENTERALIGNED3     GPT_CR1_CMS
/**
  * @}
  */


/** @defgroup GPT_Output_Compare_and_PWM_modes  TIM Output Compare and PWM modes
  * @{
  */
#define GPT_OCMODE_TIMING                   0x00000000U
#define GPT_OCMODE_ACTIVE                   (GPT_CCMR1_OC1M_0)
#define GPT_OCMODE_INACTIVE                 (GPT_CCMR1_OC1M_1)
#define GPT_OCMODE_TOGGLE                   (GPT_CCMR1_OC1M_0 | GPT_CCMR1_OC1M_1)
#define GPT_OCMODE_PWM1                     (GPT_CCMR1_OC1M_1 | GPT_CCMR1_OC1M_2)
#define GPT_OCMODE_PWM2                     (GPT_CCMR1_OC1M)
#define GPT_OCMODE_FORCED_ACTIVE            (GPT_CCMR1_OC1M_0 | GPT_CCMR1_OC1M_2)
#define GPT_OCMODE_FORCED_INACTIVE          (GPT_CCMR1_OC1M_2)

/**
  * @}
  */

/** @defgroup TIM_Channel  TIM Channel
  * @{
  */
#define TIM_CHANNEL_1                      0x00000000U
#define TIM_CHANNEL_2                      0x00000004U
#define TIM_CHANNEL_3                      0x00000008U
#define TIM_CHANNEL_4                      0x0000000CU
#define TIM_CHANNEL_5                      0x00000010U                          /*!< Compare channel 5 identifier              */
#define TIM_CHANNEL_6                      0x00000014U                          /*!< Compare channel 6 identifier              */
#define TIM_CHANNEL_ALL                    0x0000003CU                          /*!< Global Capture/compare channel identifier  */
/**
  * @}
  */


/** @defgroup GPT_Output_Fast_State  TIM Output Fast State
  * @{
  */
#define GPT_OCFAST_DISABLE                0x00000000U
#define GPT_OCFAST_ENABLE                 (GPT_CCMR1_OC1FE)
/**
  * @}
  */

/** @defgroup GPT_Output_Compare_Polarity  TIM Output Compare Polarity
  * @{
  */
#define GPT_OCPOLARITY_HIGH                0x00000000U
#define GPT_OCPOLARITY_LOW                 (GPT_CCER_CC1P)
/**
  * @}
  */

/** @defgroup GPT_Output_Compare_N_Polarity  TIM Output CompareN Polarity
  * @{
  */
#define GPT_OCNPOLARITY_HIGH               0x00000000U
#define GPT_OCNPOLARITY_LOW                (GPT_CCER_CC1NP)
/**
  * @}
  */

/** @defgroup GPT_Output_Compare_Idle_State  TIM Output Compare Idle State
  * @{
  */
#define GPT_OCIDLESTATE_SET                (GPT_CR2_OIS1)
#define GPT_OCIDLESTATE_RESET              0x00000000U
/**
  * @}
  */

/** @defgroup GPT_Output_Compare_N_Idle_State  TIM Output Compare N Idle State
  * @{
  */
#define GPT_OCNIDLESTATE_SET               (GPT_CR2_OIS1N)
#define GPT_OCNIDLESTATE_RESET             0x00000000U
/**
  * @}
  */


/** @defgroup GPT_Channel  TIM Channel
  * @{
  */
#define GPT_CHANNEL_1                      0x00000000U
#define GPT_CHANNEL_2                      0x00000004U
#define GPT_CHANNEL_3                      0x00000008U
#define GPT_CHANNEL_4                      0x0000000CU
#define GPT_CHANNEL_ALL                    0x00000018U

/**
  * @}
  */

/** @defgroup GPT_Input_Capture_Polarity  TIM Input Capture Polarity
  * @{
  */
#define  GPT_ICPOLARITY_RISING             GPT_INPUTCHANNELPOLARITY_RISING
#define  GPT_ICPOLARITY_FALLING            GPT_INPUTCHANNELPOLARITY_FALLING
#define  GPT_ICPOLARITY_BOTHEDGE           GPT_INPUTCHANNELPOLARITY_BOTHEDGE
/**
  * @}
  */

/** @defgroup GPT_Input_Capture_Selection  TIM Input Capture Selection
  * @{
  */
#define GPT_ICSELECTION_DIRECTTI           (GPT_CCMR1_CC1S_0)   /*!< TIM Input 1, 2, 3 or 4 is selected to be 
                                                                     connected to IC1, IC2, IC3 or IC4, respectively */
#define GPT_ICSELECTION_INDIRECTTI         (GPT_CCMR1_CC1S_1)   /*!< TIM Input 1, 2, 3 or 4 is selected to be
                                                                     connected to IC2, IC1, IC4 or IC3, respectively */
#define GPT_ICSELECTION_TRC                (GPT_CCMR1_CC1S)     /*!< TIM Input 1, 2, 3 or 4 is selected to be connected to TRC */

/**
  * @}
  */

/** @defgroup GPT_Input_Capture_Prescaler  TIM Input Capture Prescaler
  * @{
  */
#define GPT_ICPSC_DIV1                     0x00000000U       /*!< Capture performed each time an edge is detected on the capture input */
#define GPT_ICPSC_DIV2                     (1<<GPT_CCMR1_IC1PSC_Pos)     /*!< Capture performed once every 2 events */
#define GPT_ICPSC_DIV4                     (2<<GPT_CCMR1_IC1PSC_Pos)     /*!< Capture performed once every 4 events */
#define GPT_ICPSC_DIV8                     (GPT_CCMR1_IC1PSC)       /*!< Capture performed once every 8 events */
/**
  * @}
  */

/** @defgroup GPT_One_Pulse_Mode TIM One Pulse Mode
  * @{
  */
#define GPT_OPMODE_SINGLE                  (GPT_CR1_OPM)
#define GPT_OPMODE_REPETITIVE              0x00000000U
/**
  * @}
  */

/** @defgroup GPT_Encoder_Mode TIM Encoder Mode
  * @{
  */
#define GPT_ENCODERMODE_TI1                (1 << GPT_SMCR_MSM)
#define GPT_ENCODERMODE_TI2                (2 << GPT_SMCR_MSM)
#define GPT_ENCODERMODE_TI3                (3 << GPT_SMCR_MSM)


/**
  * @}
  */

/** @defgroup GPT_Interrupt_definition  TIM Interrupt definition
  * @{
  */
#define GPT_IT_UPDATE           (GPT_DIER_UIE)
#define GPT_IT_CC1              (GPT_DIER_CC1IE)
#define GPT_IT_CC2              (GPT_DIER_CC2IE)
#define GPT_IT_CC3              (GPT_DIER_CC3IE)
#define GPT_IT_CC4              (GPT_DIER_CC4IE)
#define GPT_IT_COM              (GPT_DIER_COMIE)
#define GPT_IT_TRIGGER          (GPT_DIER_TIE)
#define GPT_IT_BREAK            (GPT_DIER_BIE)
/**
  * @}
  */

/** @defgroup GPT_Commutation_Source  TIM Commutation Source
  * @{
  */
#define GPT_COMMUTATION_TRGI              (GPT_CR2_CCUS)
#define GPT_COMMUTATION_SOFTWARE          0x00000000U
/**
  * @}
  */

/** @defgroup GPT_DMA_sources  TIM DMA sources
  * @{
  */
#define GPT_DMA_UPDATE                     (GPT_DIER_UDE)
#define GPT_DMA_CC1                        (GPT_DIER_CC1DE)
#define GPT_DMA_CC2                        (GPT_DIER_CC2DE)
#define GPT_DMA_CC3                        (GPT_DIER_CC3DE)
#define GPT_DMA_CC4                        (GPT_DIER_CC4DE)
#define GPT_DMA_COM                        (GPT_DIER_COMDE)
#define GPT_DMA_TRIGGER                    (GPT_DIER_TDE)
/**
  * @}
  */

/** @defgroup GPT_Event_Source  TIM Event Source
  * @{
  */
#define GPT_EVENTSOURCE_UPDATE              GPT_EGR_UG
#define GPT_EVENTSOURCE_CC1                 GPT_EGR_CC1G
#define GPT_EVENTSOURCE_CC2                 GPT_EGR_CC2G
#define GPT_EVENTSOURCE_CC3                 GPT_EGR_CC3G
#define GPT_EVENTSOURCE_CC4                 GPT_EGR_CC4G
#define GPT_EVENTSOURCE_COM                 GPT_EGR_COMG
#define GPT_EVENTSOURCE_TRIGGER             GPT_EGR_TG
#define GPT_EVENTSOURCE_BREAK               GPT_EGR_BG

/**
  * @}
  */

/** @defgroup GPT_Flag_definition  TIM Flag definition
  * @{
  */
#define GPT_FLAG_UPDATE                    (GPT_SR_UIF)
#define GPT_FLAG_CC1                       (GPT_SR_CC1IF)
#define GPT_FLAG_CC2                       (GPT_SR_CC2IF)
#define GPT_FLAG_CC3                       (GPT_SR_CC3IF)
#define GPT_FLAG_CC4                       (GPT_SR_CC4IF)
#define GPT_FLAG_COM                       (GPT_SR_COMIF)
#define GPT_FLAG_TRIGGER                   (GPT_SR_TIF)
#define GPT_FLAG_BREAK                     (GPT_SR_BIF)
#define GPT_FLAG_CC1OF                     (GPT_SR_CC1OF)
#define GPT_FLAG_CC2OF                     (GPT_SR_CC2OF)
#define GPT_FLAG_CC3OF                     (GPT_SR_CC3OF)
#define GPT_FLAG_CC4OF                     (GPT_SR_CC4OF)
/**
  * @}
  */

/** @defgroup GPT_Clock_Source  TIM Clock Source
  * @{
  */
#define GPT_CLOCKSOURCE_ETRMODE2    (GPT_SMCR_ETPS_1)
#define GPT_CLOCKSOURCE_INTERNAL    (GPT_SMCR_ETPS_0)
#define GPT_CLOCKSOURCE_ITR0        0x00000000U
#define GPT_CLOCKSOURCE_ITR1        (GPT_SMCR_TS_0)
#define GPT_CLOCKSOURCE_ITR2        (GPT_SMCR_TS_1)
#define GPT_CLOCKSOURCE_ITR3        (GPT_SMCR_TS_0 | GPT_SMCR_TS_1)
#define GPT_CLOCKSOURCE_TI1ED       (GPT_SMCR_TS_2)
#define GPT_CLOCKSOURCE_TI1         (GPT_SMCR_TS_0 | GPT_SMCR_TS_2)
#define GPT_CLOCKSOURCE_TI2         (GPT_SMCR_TS_1 | GPT_SMCR_TS_2)
#define GPT_CLOCKSOURCE_ETRMODE1    (GPT_SMCR_TS)
/**
  * @}
  */

/** @defgroup GPT_Clock_Polarity  TIM Clock Polarity
  * @{
  */
#define GPT_CLOCKPOLARITY_INVERTED           GPT_ETRPOLARITY_INVERTED          /*!< Polarity for ETRx clock sources */
#define GPT_CLOCKPOLARITY_NONINVERTED        GPT_ETRPOLARITY_NONINVERTED       /*!< Polarity for ETRx clock sources */
#define GPT_CLOCKPOLARITY_RISING             GPT_INPUTCHANNELPOLARITY_RISING   /*!< Polarity for TIx clock sources */
#define GPT_CLOCKPOLARITY_FALLING            GPT_INPUTCHANNELPOLARITY_FALLING   /*!< Polarity for TIx clock sources */
#define GPT_CLOCKPOLARITY_BOTHEDGE           GPT_INPUTCHANNELPOLARITY_BOTHEDGE  /*!< Polarity for TIx clock sources */
/**
  * @}
  */

/** @defgroup GPT_Clock_Prescaler  TIM Clock Prescaler
  * @{
  */
#define GPT_CLOCKPRESCALER_DIV1              GPT_ETRPRESCALER_DIV1     /*!< No prescaler is used */
#define GPT_CLOCKPRESCALER_DIV2              GPT_ETRPRESCALER_DIV2     /*!< Prescaler for External ETR Clock: Capture performed once every 2 events. */
#define GPT_CLOCKPRESCALER_DIV4              GPT_ETRPRESCALER_DIV4     /*!< Prescaler for External ETR Clock: Capture performed once every 4 events. */
#define GPT_CLOCKPRESCALER_DIV8              GPT_ETRPRESCALER_DIV8     /*!< Prescaler for External ETR Clock: Capture performed once every 8 events. */
/**
  * @}
  */

/** @defgroup GPT_ClearInput_Source TIM Clear Input Source
  * @{
  */
#define GPT_CLEARINPUTSOURCE_ETR           0x00000001U
#define GPT_CLEARINPUTSOURCE_NONE          0x00000000U
/**
  * @}
  */

/** @defgroup GPT_ClearInput_Polarity  TIM Clear Input Polarity
  * @{
  */
#define GPT_CLEARINPUTPOLARITY_INVERTED           GPT_ETRPOLARITY_INVERTED                    /*!< Polarity for ETRx pin */
#define GPT_CLEARINPUTPOLARITY_NONINVERTED        GPT_ETRPOLARITY_NONINVERTED                 /*!< Polarity for ETRx pin */
/**
  * @}
  */

/** @defgroup GPT_ClearInput_Prescaler TIM Clear Input Prescaler
  * @{
  */
#define GPT_CLEARINPUTPRESCALER_DIV1                    GPT_ETRPRESCALER_DIV1      /*!< No prescaler is used */
#define GPT_CLEARINPUTPRESCALER_DIV2                    GPT_ETRPRESCALER_DIV2      /*!< Prescaler for External ETR pin: Capture performed once every 2 events. */
#define GPT_CLEARINPUTPRESCALER_DIV4                    GPT_ETRPRESCALER_DIV4      /*!< Prescaler for External ETR pin: Capture performed once every 4 events. */
#define GPT_CLEARINPUTPRESCALER_DIV8                    GPT_ETRPRESCALER_DIV8        /*!< Prescaler for External ETR pin: Capture performed once every 8 events. */
/**
  * @}
  */

/** @defgroup GPT_OSSR_Off_State_Selection_for_Run_mode_state TIM OSSR OffState Selection for Run mode state
  * @{
  */
#define GPT_OSSR_ENABLE         (GPT_BDTR_OSSR)
#define GPT_OSSR_DISABLE        0x00000000U
/**
  * @}
  */

/** @defgroup GPT_OSSI_Off_State_Selection_for_Idle_mode_state TIM OSSI OffState Selection for Idle mode state
  * @{
  */
#define GPT_OSSI_ENABLE             (GPT_BDTR_OSSI)
#define GPT_OSSI_DISABLE            0x00000000U
/**
  * @}
  */

/** @defgroup GPT_Lock_level  TIM Lock level
  * @{
  */
#define GPT_LOCKLEVEL_OFF          0x00000000U
#define GPT_LOCKLEVEL_1            (GPT_BDTR_LOCK_0)
#define GPT_LOCKLEVEL_2            (GPT_BDTR_LOCK_1)
#define GPT_LOCKLEVEL_3            (GPT_BDTR_LOCK)
/**
  * @}
  */
/** @defgroup GPT_Break_Input_enable_disable  TIM Break Input State
  * @{
  */
#define GPT_BREAK_ENABLE          (GPT_BDTR_BKE)
#define GPT_BREAK_DISABLE         0x00000000U
/**
  * @}
  */

/** @defgroup GPT_Break_Polarity  TIM Break Polarity
  * @{
  */
#define GPT_BREAKPOLARITY_LOW        0x00000000U
#define GPT_BREAKPOLARITY_HIGH       (GPT_BDTR_BKP)
/**
  * @}
  */

/** @defgroup GPT_AOE_Bit_Set_Reset  TIM AOE Bit State
  * @{
  */
#define GPT_AUTOMATICOUTPUT_ENABLE           (GPT_BDTR_AOE)
#define GPT_AUTOMATICOUTPUT_DISABLE          0x00000000U
/**
  * @}
  */

/** @defgroup GPT_Master_Mode_Selection TIM Master Mode Selection
  * @{
  */
#define   GPT_CR2_MMS_0 (1<<ATIM_CR2_MMS_Pos)
#define   GPT_CR2_MMS_1 (2<<ATIM_CR2_MMS_Pos)
#define   GPT_CR2_MMS_2 (4<<ATIM_CR2_MMS_Pos)

#define GPT_TRGO_RESET            0x00000000U
#define GPT_TRGO_ENABLE           (GPT_CR2_MMS_0)
#define GPT_TRGO_UPDATE           (GPT_CR2_MMS_1)
#define GPT_TRGO_OC1              ((GPT_CR2_MMS_1 | GPT_CR2_MMS_0))
#define GPT_TRGO_OC1REF           (GPT_CR2_MMS_2)
#define GPT_TRGO_OC2REF           ((GPT_CR2_MMS_2 | GPT_CR2_MMS_0))
#define GPT_TRGO_OC3REF           ((GPT_CR2_MMS_2 | GPT_CR2_MMS_1))
#define GPT_TRGO_OC4REF           ((GPT_CR2_MMS_2 | GPT_CR2_MMS_1 | GPT_CR2_MMS_0))
/**
  * @}
  */

/** @defgroup GPT_Slave_Mode TIM Slave Mode
  * @{
  */
#define GPT_SLAVEMODE_DISABLE              0x00000000U
#define GPT_SLAVEMODE_RESET                0x00000004U
#define GPT_SLAVEMODE_GATED                0x00000005U
#define GPT_SLAVEMODE_TRIGGER              0x00000006U
#define GPT_SLAVEMODE_EXTERNAL1            0x00000007U
/**
  * @}
  */

/** @defgroup GPT_Master_Slave_Mode  TIM Master Slave Mode
  * @{
  */
#define GPT_MASTERSLAVEMODE_ENABLE          0x00000080U
#define GPT_MASTERSLAVEMODE_DISABLE         0x00000000U
/**
  * @}
  */

/** @defgroup GPT_Trigger_Selection  TIM Trigger Selection
  * @{
  */
#define GPT_TS_ITR0                        0x00000000U
#define GPT_TS_ITR1                        0x00000010U
#define GPT_TS_ITR2                        0x00000020U
#define GPT_TS_ITR3                        0x00000030U
#define GPT_TS_TI1F_ED                     0x00000040U
#define GPT_TS_TI1FP1                      0x00000050U
#define GPT_TS_TI2FP2                      0x00000060U
#define GPT_TS_ETRF                        0x00000070U
#define GPT_TS_NONE                        0x0000FFFFU
/**
  * @}
  */

/** @defgroup GPT_Trigger_Polarity TIM Trigger Polarity
  * @{
  */
#define GPT_TRIGGERPOLARITY_INVERTED           GPT_ETRPOLARITY_INVERTED      /*!< Polarity for ETRx trigger sources */
#define GPT_TRIGGERPOLARITY_NONINVERTED        GPT_ETRPOLARITY_NONINVERTED   /*!< Polarity for ETRx trigger sources */
#define GPT_TRIGGERPOLARITY_RISING             GPT_INPUTCHANNELPOLARITY_RISING        /*!< Polarity for TIxFPx or TI1_ED trigger sources */
#define GPT_TRIGGERPOLARITY_FALLING            GPT_INPUTCHANNELPOLARITY_FALLING       /*!< Polarity for TIxFPx or TI1_ED trigger sources */
#define GPT_TRIGGERPOLARITY_BOTHEDGE           GPT_INPUTCHANNELPOLARITY_BOTHEDGE      /*!< Polarity for TIxFPx or TI1_ED trigger sources */
/**
  * @}
  */

/** @defgroup GPT_Trigger_Prescaler TIM Trigger Prescaler
  * @{
  */
#define GPT_TRIGGERPRESCALER_DIV1             GPT_ETRPRESCALER_DIV1     /*!< No prescaler is used */
#define GPT_TRIGGERPRESCALER_DIV2             GPT_ETRPRESCALER_DIV2     /*!< Prescaler for External ETR Trigger: Capture performed once every 2 events. */
#define GPT_TRIGGERPRESCALER_DIV4             GPT_ETRPRESCALER_DIV4     /*!< Prescaler for External ETR Trigger: Capture performed once every 4 events. */
#define GPT_TRIGGERPRESCALER_DIV8             GPT_ETRPRESCALER_DIV8     /*!< Prescaler for External ETR Trigger: Capture performed once every 8 events. */
/**
  * @}
  */


/** @defgroup GPT_TI1_Selection TIM TI1 Selection
  * @{
  */
#define GPT_TI1SELECTION_CH1                0x00000000U
#define GPT_TI1SELECTION_XORCOMBINATION     (GPT_CR2_TI1S)
/**
  * @}
  */

/** @defgroup GPT_DMA_Base_address  TIM DMA Base address
  * @{
  */
#define GPT_DMABASE_CR1                    0x00000000U
#define GPT_DMABASE_CR2                    0x00000001U
#define GPT_DMABASE_SMCR                   0x00000002U
#define GPT_DMABASE_DIER                   0x00000003U
#define GPT_DMABASE_SR                     0x00000004U
#define GPT_DMABASE_EGR                    0x00000005U
#define GPT_DMABASE_CCMR1                  0x00000006U
#define GPT_DMABASE_CCMR2                  0x00000007U
#define GPT_DMABASE_CCER                   0x00000008U
#define GPT_DMABASE_CNT                    0x00000009U
#define GPT_DMABASE_PSC                    0x0000000AU
#define GPT_DMABASE_ARR                    0x0000000BU
#define GPT_DMABASE_RCR                    0x0000000CU
#define GPT_DMABASE_CCR1                   0x0000000DU
#define GPT_DMABASE_CCR2                   0x0000000EU
#define GPT_DMABASE_CCR3                   0x0000000FU
#define GPT_DMABASE_CCR4                   0x00000010U
#define GPT_DMABASE_BDTR                   0x00000011U
#define GPT_DMABASE_DCR                    0x00000012U
#define GPT_DMABASE_OR                     0x00000013U
/**
  * @}
  */

/** @defgroup GPT_DMA_Burst_Length  TIM DMA Burst Length
  * @{
  */
#define GPT_DMABURSTLENGTH_1TRANSFER           0x00000000U
#define GPT_DMABURSTLENGTH_2TRANSFERS          0x00000100U
#define GPT_DMABURSTLENGTH_3TRANSFERS          0x00000200U
#define GPT_DMABURSTLENGTH_4TRANSFERS          0x00000300U
#define GPT_DMABURSTLENGTH_5TRANSFERS          0x00000400U
#define GPT_DMABURSTLENGTH_6TRANSFERS          0x00000500U
#define GPT_DMABURSTLENGTH_7TRANSFERS          0x00000600U
#define GPT_DMABURSTLENGTH_8TRANSFERS          0x00000700U
#define GPT_DMABURSTLENGTH_9TRANSFERS          0x00000800U
#define GPT_DMABURSTLENGTH_10TRANSFERS         0x00000900U
#define GPT_DMABURSTLENGTH_11TRANSFERS         0x00000A00U
#define GPT_DMABURSTLENGTH_12TRANSFERS         0x00000B00U
#define GPT_DMABURSTLENGTH_13TRANSFERS         0x00000C00U
#define GPT_DMABURSTLENGTH_14TRANSFERS         0x00000D00U
#define GPT_DMABURSTLENGTH_15TRANSFERS         0x00000E00U
#define GPT_DMABURSTLENGTH_16TRANSFERS         0x00000F00U
#define GPT_DMABURSTLENGTH_17TRANSFERS         0x00001000U
#define GPT_DMABURSTLENGTH_18TRANSFERS         0x00001100U
/**
  * @}
  */

/** @defgroup DMA_Handle_index  DMA Handle index
  * @{
  */
#define GPT_DMA_ID_UPDATE                ((uint16_t)0x0000)       /*!< Index of the DMA handle used for Update DMA requests */
#define GPT_DMA_ID_CC1                   ((uint16_t)0x0001)       /*!< Index of the DMA handle used for Capture/Compare 1 DMA requests */
#define GPT_DMA_ID_CC2                   ((uint16_t)0x0002)       /*!< Index of the DMA handle used for Capture/Compare 2 DMA requests */
#define GPT_DMA_ID_CC3                   ((uint16_t)0x0003)       /*!< Index of the DMA handle used for Capture/Compare 3 DMA requests */
#define GPT_DMA_ID_CC4                   ((uint16_t)0x0004)       /*!< Index of the DMA handle used for Capture/Compare 4 DMA requests */
#define GPT_DMA_ID_COMMUTATION           ((uint16_t)0x0005)       /*!< Index of the DMA handle used for Commutation DMA requests */
#define GPT_DMA_ID_TRIGGER               ((uint16_t)0x0006)       /*!< Index of the DMA handle used for Trigger DMA requests */
/**
  * @}
  */

/** @defgroup Channel_CC_State  Channel CC State
  * @{
  */
#define GPT_CCx_ENABLE                   0x00000001U
#define GPT_CCx_DISABLE                  0x00000000U
#define GPT_CCxN_ENABLE                  0x00000004U
#define GPT_CCxN_DISABLE                 0x00000000U
/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup GPT_Exported_Macros TIM Exported Macros
  * @{
  */
/** @brief Reset TIM handle state
  * @param  \__HANDLE__ TIM handle
  * @retval None
  */
#define __HAL_GPT_RESET_HANDLE_STATE(__HANDLE__) ((__HANDLE__)->State = HAL_GPT_STATE_RESET)

/**
  * @brief  Enable the TIM peripheral.
  * @param  \__HANDLE__ TIM handle
  * @retval None
 */
#define __HAL_GPT_ENABLE(__HANDLE__)                 ((__HANDLE__)->Instance->CR1|=(GPT_CR1_CEN))


#define __HAL_GPT_IS_ENABLED(__INSTANCE__)           ((__INSTANCE__)->CR1 & (GPT_CR1_CEN))

/**
  * @brief  Enable the TIM main Output.
  * @param  \__HANDLE__ TIM handle
  * @retval None
  */
#ifdef hwp_atim1
#define __HAL_GPT_MOE_ENABLE(__HANDLE__)             (((ATIM_TypeDef*)((__HANDLE__)->Instance))->BDTR|=(ATIM_BDTR_MOE))
#else
#define __HAL_GPT_MOE_ENABLE(__HANDLE__)
#endif

/**
  * @brief  Disable the TIM peripheral.
  * @param  \__HANDLE__ TIM handle
  * @retval None
  */
#define __HAL_GPT_DISABLE(__HANDLE__) \
                        do { \
                          if (((__HANDLE__)->Instance->CCER & GPT_CCER_CCxE_MASK) == 0U) \
                          { \
                            if(((__HANDLE__)->Instance->CCER & GPT_CCER_CCxNE_MASK) == 0U) \
                            { \
                              (__HANDLE__)->Instance->CR1 &= ~(GPT_CR1_CEN); \
                            } \
                          } \
                        } while(0U)

/* The Main Output of a timer instance is disabled only if all the CCx and CCxN
   channels have been disabled */
/**
  * @brief  Disable the TIM main Output.
  * @param  \__HANDLE__ TIM handle
  * @retval None
  */
#ifdef hwp_atim1
#define __HAL_GPT_MOE_DISABLE(__HANDLE__) \
                        do { \
                          if (((__HANDLE__)->Instance->CCER & GPT_CCER_CCxE_MASK) == 0U) \
                          { \
                            if(((__HANDLE__)->Instance->CCER & GPT_CCER_CCxNE_MASK) == 0U) \
                            { \
                              ((ATIM_TypeDef*)((__HANDLE__)->Instance))->BDTR &= ~(ATIM_BDTR_MOE); \
                            } \
                          } \
                        } while(0U)
#else
#define __HAL_GPT_MOE_DISABLE(__HANDLE__)
#endif
/**
  * @brief  Disable the TIM main Output.
  * @param  \__HANDLE__ TIM handle
  * @retval None
  * @note The Main Output Enable of a timer instance is disabled unconditionally
  */
#define __HAL_GPT_MOE_DISABLE_UNCONDITIONALLY(__HANDLE__)  (__HANDLE__)->Instance->BDTR &= ~(GPT_BDTR_MOE)

/** @brief  Enable the specified TIM interrupt.
  * @param  \__HANDLE__ specifies the TIM Handle.
  * @param  \__INTERRUPT__ specifies the TIM interrupt source to enable.
  *          This parameter can be one of the following values:
  *            @arg GPT_IT_UPDATE: Update interrupt
  *            @arg GPT_IT_CC1:   Capture/Compare 1 interrupt
  *            @arg GPT_IT_CC2:  Capture/Compare 2 interrupt
  *            @arg GPT_IT_CC3:  Capture/Compare 3 interrupt
  *            @arg GPT_IT_CC4:  Capture/Compare 4 interrupt
  *            @arg GPT_IT_COM:   Commutation interrupt
  *            @arg GPT_IT_TRIGGER: Trigger interrupt
  *            @arg GPT_IT_BREAK: Break interrupt
  * @retval None
  */
#define __HAL_GPT_ENABLE_IT(__HANDLE__, __INTERRUPT__)    ((__HANDLE__)->Instance->DIER |= (__INTERRUPT__))


/** @brief  Disable the specified TIM interrupt.
  * @param  \__HANDLE__ specifies the TIM Handle.
  * @param  \__INTERRUPT__ specifies the TIM interrupt source to disable.
  *          This parameter can be one of the following values:
  *            @arg GPT_IT_UPDATE: Update interrupt
  *            @arg GPT_IT_CC1:   Capture/Compare 1 interrupt
  *            @arg GPT_IT_CC2:  Capture/Compare 2 interrupt
  *            @arg GPT_IT_CC3:  Capture/Compare 3 interrupt
  *            @arg GPT_IT_CC4:  Capture/Compare 4 interrupt
  *            @arg GPT_IT_COM:   Commutation interrupt
  *            @arg GPT_IT_TRIGGER: Trigger interrupt
  *            @arg GPT_IT_BREAK: Break interrupt
  * @retval None
  */
#define __HAL_GPT_DISABLE_IT(__HANDLE__, __INTERRUPT__)   ((__HANDLE__)->Instance->DIER &= ~(__INTERRUPT__))

/** @brief  Enable the specified DMA request.
  * @param  \__HANDLE__ specifies the TIM Handle.
  * @param  \__DMA__ specifies the TIM DMA request to enable.
  *          This parameter can be one of the following values:
  *            @arg GPT_DMA_UPDATE: Update DMA request
  *            @arg GPT_DMA_CC1:   Capture/Compare 1 DMA request
  *            @arg GPT_DMA_CC2:  Capture/Compare 2 DMA request
  *            @arg GPT_DMA_CC3:  Capture/Compare 3 DMA request
  *            @arg GPT_DMA_CC4:  Capture/Compare 4 DMA request
  *            @arg GPT_DMA_COM:   Commutation DMA request
  *            @arg GPT_DMA_TRIGGER: Trigger DMA request
  * @retval None
  */
#define __HAL_GPT_ENABLE_DMA(__HANDLE__, __DMA__)         ((__HANDLE__)->Instance->DIER |= (__DMA__))

/** @brief  Disable the specified DMA request.
  * @param  \__HANDLE__ specifies the TIM Handle.
  * @param  \__DMA__ specifies the TIM DMA request to disable.
  *          This parameter can be one of the following values:
  *            @arg GPT_DMA_UPDATE: Update DMA request
  *            @arg GPT_DMA_CC1:   Capture/Compare 1 DMA request
  *            @arg GPT_DMA_CC2:  Capture/Compare 2 DMA request
  *            @arg GPT_DMA_CC3:  Capture/Compare 3 DMA request
  *            @arg GPT_DMA_CC4:  Capture/Compare 4 DMA request
  *            @arg GPT_DMA_COM:   Commutation DMA request
  *            @arg GPT_DMA_TRIGGER: Trigger DMA request
  * @retval None
  */
#define __HAL_GPT_DISABLE_DMA(__HANDLE__, __DMA__)        ((__HANDLE__)->Instance->DIER &= ~(__DMA__))

/** @brief  Check whether the specified TIM interrupt flag is set or not.
  * @param  \__HANDLE__ specifies the TIM Handle.
  * @param  \__FLAG__ specifies the TIM interrupt flag to check.
  *        This parameter can be one of the following values:
  *            @arg GPT_FLAG_UPDATE: Update interrupt flag
  *            @arg GPT_FLAG_CC1: Capture/Compare 1 interrupt flag
  *            @arg GPT_FLAG_CC2: Capture/Compare 2 interrupt flag
  *            @arg GPT_FLAG_CC3: Capture/Compare 3 interrupt flag
  *            @arg GPT_FLAG_CC4: Capture/Compare 4 interrupt flag
  *            @arg GPT_FLAG_CC5: Compare 5 interrupt flag
  *            @arg GPT_FLAG_CC6: Compare 6 interrupt flag
  *            @arg GPT_FLAG_COM:  Commutation interrupt flag
  *            @arg GPT_FLAG_TRIGGER: Trigger interrupt flag
  *            @arg GPT_FLAG_BREAK: Break interrupt flag
  *            @arg GPT_FLAG_BREAK2: Break 2 interrupt flag
  *            @arg GPT_FLAG_SYSTEM_BREAK: System Break interrupt flag
  *            @arg GPT_FLAG_CC1OF: Capture/Compare 1 overcapture flag
  *            @arg GPT_FLAG_CC2OF: Capture/Compare 2 overcapture flag
  *            @arg GPT_FLAG_CC3OF: Capture/Compare 3 overcapture flag
  *            @arg GPT_FLAG_CC4OF: Capture/Compare 4 overcapture flag
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#define __HAL_GPT_GET_FLAG(__HANDLE__, __FLAG__)          (((__HANDLE__)->Instance->SR &(__FLAG__)) == (__FLAG__))

/** @brief  Clear the specified TIM interrupt flag.
  * @param  \__HANDLE__ specifies the TIM Handle.
  * @param  \__FLAG__ specifies the TIM interrupt flag to clear.
  *        This parameter can be one of the following values:
  *            @arg GPT_FLAG_UPDATE: Update interrupt flag
  *            @arg GPT_FLAG_CC1: Capture/Compare 1 interrupt flag
  *            @arg GPT_FLAG_CC2: Capture/Compare 2 interrupt flag
  *            @arg GPT_FLAG_CC3: Capture/Compare 3 interrupt flag
  *            @arg GPT_FLAG_CC4: Capture/Compare 4 interrupt flag
  *            @arg GPT_FLAG_CC5: Compare 5 interrupt flag
  *            @arg GPT_FLAG_CC6: Compare 6 interrupt flag
  *            @arg GPT_FLAG_COM:  Commutation interrupt flag
  *            @arg GPT_FLAG_TRIGGER: Trigger interrupt flag
  *            @arg GPT_FLAG_BREAK: Break interrupt flag
  *            @arg GPT_FLAG_BREAK2: Break 2 interrupt flag
  *            @arg GPT_FLAG_SYSTEM_BREAK: System Break interrupt flag
  *            @arg GPT_FLAG_CC1OF: Capture/Compare 1 overcapture flag
  *            @arg GPT_FLAG_CC2OF: Capture/Compare 2 overcapture flag
  *            @arg GPT_FLAG_CC3OF: Capture/Compare 3 overcapture flag
  *            @arg GPT_FLAG_CC4OF: Capture/Compare 4 overcapture flag
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#define __HAL_GPT_CLEAR_FLAG(__HANDLE__, __FLAG__)        ((__HANDLE__)->Instance->SR = ~(__FLAG__))

/**
  * @brief  Check whether the specified TIM interrupt source is enabled or not.
  * @param  \__HANDLE__ TIM handle
  * @param  \__INTERRUPT__ specifies the TIM interrupt source to check.
  *          This parameter can be one of the following values:
  *            @arg GPT_IT_UPDATE: Update interrupt
  *            @arg GPT_IT_CC1:   Capture/Compare 1 interrupt
  *            @arg GPT_IT_CC2:  Capture/Compare 2 interrupt
  *            @arg GPT_IT_CC3:  Capture/Compare 3 interrupt
  *            @arg GPT_IT_CC4:  Capture/Compare 4 interrupt
  *            @arg GPT_IT_COM:   Commutation interrupt
  *            @arg GPT_IT_TRIGGER: Trigger interrupt
  *            @arg GPT_IT_BREAK: Break interrupt
  * @retval The state of GPT_IT (SET or RESET).
  */
#define __HAL_GPT_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__) ((((__HANDLE__)->Instance->DIER & (__INTERRUPT__)) == (__INTERRUPT__)) ? SET : RESET)

/** @brief Clear the TIM interrupt pending bits.
  * @param  \__HANDLE__ TIM handle
  * @param  \__INTERRUPT__ specifies the interrupt pending bit to clear.
  *          This parameter can be one of the following values:
  *            @arg GPT_IT_UPDATE: Update interrupt
  *            @arg GPT_IT_CC1:   Capture/Compare 1 interrupt
  *            @arg GPT_IT_CC2:  Capture/Compare 2 interrupt
  *            @arg GPT_IT_CC3:  Capture/Compare 3 interrupt
  *            @arg GPT_IT_CC4:  Capture/Compare 4 interrupt
  *            @arg GPT_IT_COM:   Commutation interrupt
  *            @arg GPT_IT_TRIGGER: Trigger interrupt
  *            @arg GPT_IT_BREAK: Break interrupt
  * @retval None
  */
#define __HAL_GPT_CLEAR_IT(__HANDLE__, __INTERRUPT__)     ((__HANDLE__)->Instance->SR = ~(__INTERRUPT__))

/**
  * @brief  Indicates whether or not the TIM Counter is used as downcounter.
  * @param  \__HANDLE__ TIM handle.
  * @retval False (Counter used as upcounter) or True (Counter used as downcounter)
  * @note This macro is particularly useful to get the counting mode when the timer operates in Center-aligned mode or Encoder
mode.
  */
#define __HAL_GPT_IS_GPT_COUNTING_DOWN(__HANDLE__)            (((__HANDLE__)->Instance->CR1 &(GPT_CR1_DIR)) == (GPT_CR1_DIR))

/**
  * @brief  Set the TIM Prescaler on runtime.
  * @param  \__HANDLE__ TIM handle.
  * @param  \__PRESC__ specifies the Prescaler new value.
  * @retval None
  */
#define __HAL_GPT_SET_PRESCALER(__HANDLE__, __PRESC__)       ((__HANDLE__)->Instance->PSC = (__PRESC__))

/**
  * @brief  Set the TIM single mode.
  * @param  \__HANDLE__ TIM handle.
  * @param  \__MODE__ GPT_OPMODE_REPETITIVE:REPETITIVE, GPT_OPMODE_SINGLE:One pulse.
  * @retval None
  */
#define __HAL_GPT_SET_MODE(__HANDLE__, __MODE__)       (__MODE__?((__HANDLE__)->Instance->CR1|=__MODE__):((__HANDLE__)->Instance->CR1&=~GPT_OPMODE_SINGLE))

#define GPT_SET_ICPRESCALERVALUE(__HANDLE__, __CHANNEL__, __ICPSC__) \
(((__CHANNEL__) == GPT_CHANNEL_1) ? ((__HANDLE__)->Instance->CCMR1 |= (__ICPSC__)) :\
 ((__CHANNEL__) == GPT_CHANNEL_2) ? ((__HANDLE__)->Instance->CCMR1 |= ((__ICPSC__) << 8U)) :\
 ((__CHANNEL__) == GPT_CHANNEL_3) ? ((__HANDLE__)->Instance->CCMR2 |= (__ICPSC__)) :\
 ((__HANDLE__)->Instance->CCMR2 |= ((__ICPSC__) << 8U)))

#define GPT_RESET_ICPRESCALERVALUE(__HANDLE__, __CHANNEL__) \
(((__CHANNEL__) == GPT_CHANNEL_1) ? ((__HANDLE__)->Instance->CCMR1 &= (uint16_t)~GPT_CCMR1_IC1PSC) :\
 ((__CHANNEL__) == GPT_CHANNEL_2) ? ((__HANDLE__)->Instance->CCMR1 &= (uint16_t)~GPT_CCMR1_IC2PSC) :\
 ((__CHANNEL__) == GPT_CHANNEL_3) ? ((__HANDLE__)->Instance->CCMR2 &= (uint16_t)~GPT_CCMR2_IC3PSC) :\
 ((__HANDLE__)->Instance->CCMR2 &= (uint16_t)~GPT_CCMR2_IC4PSC))

#define GPT_SET_CAPTUREPOLARITY(__HANDLE__, __CHANNEL__, __POLARITY__) \
(((__CHANNEL__) == GPT_CHANNEL_1) ? ((__HANDLE__)->Instance->CCER |= (__POLARITY__)) :\
 ((__CHANNEL__) == GPT_CHANNEL_2) ? ((__HANDLE__)->Instance->CCER |= ((__POLARITY__) << 4U)) :\
 ((__CHANNEL__) == GPT_CHANNEL_3) ? ((__HANDLE__)->Instance->CCER |= ((__POLARITY__) << 8U)) :\
 ((__HANDLE__)->Instance->CCER |= (((__POLARITY__) << 12U) & GPT_CCER_CC4P)))

#define GPT_RESET_CAPTUREPOLARITY(__HANDLE__, __CHANNEL__) \
(((__CHANNEL__) == GPT_CHANNEL_1) ? ((__HANDLE__)->Instance->CCER &= (uint16_t)~(GPT_CCER_CC1P | GPT_CCER_CC1NP)) :\
 ((__CHANNEL__) == GPT_CHANNEL_2) ? ((__HANDLE__)->Instance->CCER &= (uint16_t)~(GPT_CCER_CC2P | GPT_CCER_CC2NP)) :\
 ((__CHANNEL__) == GPT_CHANNEL_3) ? ((__HANDLE__)->Instance->CCER &= (uint16_t)~(GPT_CCER_CC3P | GPT_CCER_CC3NP)) :\
 ((__HANDLE__)->Instance->CCER &= (uint16_t)~GPT_CCER_CC4P))

/**
  * @brief  Sets the TIM Capture Compare Register value on runtime without
  *         calling another time ConfigChannel function.
  * @param  \__HANDLE__ TIM handle.
  * @param  \__CHANNEL__  TIM Channels to be configured.
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  *            @arg GPT_CHANNEL_3: TIM Channel 3 selected
  *            @arg GPT_CHANNEL_4: TIM Channel 4 selected
  * @param  \__COMPARE__ specifies the Capture Compare register new value.
  * @retval None
  */
#define __HAL_GPT_SET_COMPARE(__HANDLE__, __CHANNEL__, __COMPARE__) \
(*(__IO uint32_t *)(&((__HANDLE__)->Instance->CCR1) + ((__CHANNEL__) >> 2U)) = (__COMPARE__))

/**
  * @brief  Gets the TIM Capture Compare Register value on runtime.
  * @param  \__HANDLE__ TIM handle.
  * @param  \__CHANNEL__ TIM Channel associated with the capture compare register
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: get capture/compare 1 register value
  *            @arg GPT_CHANNEL_2: get capture/compare 2 register value
  *            @arg GPT_CHANNEL_3: get capture/compare 3 register value
  *            @arg GPT_CHANNEL_4: get capture/compare 4 register value
  *            @arg GPT_CHANNEL_5: get capture/compare 5 register value
  *            @arg GPT_CHANNEL_6: get capture/compare 6 register value
  * @retval 16-bit or 32-bit value of the capture/compare register (TIMx_CCRy)
  */
#define __HAL_GPT_GET_COMPARE(__HANDLE__, __CHANNEL__) \
  (*(__IO uint32_t *)(&((__HANDLE__)->Instance->CCR1) + ((__CHANNEL__) >> 2U)))

/**
  * @brief  Sets the TIM Counter Register value on runtime.
  * @param  \__HANDLE__ TIM handle.
  * @param  \__COUNTER__ specifies the Counter register new value.
  * @retval None
  */
#define __HAL_GPT_SET_COUNTER(__HANDLE__, __COUNTER__) ((__HANDLE__)->Instance->CNT = (__COUNTER__))

/**
  * @brief  Gets the TIM Counter Register value on runtime.
  * @param  \__HANDLE__ TIM handle.
  * @retval 16-bit or 32-bit value of the timer counter register (TIMx_CNT)
  */
#define __HAL_GPT_GET_COUNTER(__HANDLE__) ((__HANDLE__)->Instance->CNT)

/**
  * @brief  Sets the TIM Autoreload Register value on runtime without calling
  *         another time any Init function.
  * @param  \__HANDLE__ TIM handle.
  * @param  \__AUTORELOAD__ specifies the Counter register new value.
  * @retval None
  */
#define __HAL_GPT_SET_AUTORELOAD(__HANDLE__, __AUTORELOAD__)                  \
                        do{                                                  \
                            (__HANDLE__)->Instance->ARR = (__AUTORELOAD__);  \
                            (__HANDLE__)->Init.Period = (__AUTORELOAD__);    \
                          } while(0U)
/**
  * @brief  Gets the TIM Autoreload Register value on runtime.
  * @param  \__HANDLE__ TIM handle.
  * @retval 16-bit or 32-bit value of the timer auto-reload register(TIMx_ARR)
  */
#define __HAL_GPT_GET_AUTORELOAD(__HANDLE__) ((__HANDLE__)->Instance->ARR)


/**
  * @brief  Sets the TIM Input Capture prescaler on runtime without calling
  *         another time HAL_GPT_IC_ConfigChannel() function.
  * @param  \__HANDLE__ TIM handle.
  * @param  \__CHANNEL__  TIM Channels to be configured.
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  *            @arg GPT_CHANNEL_3: TIM Channel 3 selected
  *            @arg GPT_CHANNEL_4: TIM Channel 4 selected
  * @param  \__ICPSC__ specifies the Input Capture4 prescaler new value.
  *          This parameter can be one of the following values:
  *            @arg GPT_ICPSC_DIV1: no prescaler
  *            @arg GPT_ICPSC_DIV2: capture is done once every 2 events
  *            @arg GPT_ICPSC_DIV4: capture is done once every 4 events
  *            @arg GPT_ICPSC_DIV8: capture is done once every 8 events
  * @retval None
  */
#define __HAL_GPT_SET_ICPRESCALER(__HANDLE__, __CHANNEL__, __ICPSC__) \
                        do{                                                    \
                              GPT_RESET_ICPRESCALERVALUE((__HANDLE__), (__CHANNEL__));  \
                              GPT_SET_ICPRESCALERVALUE((__HANDLE__), (__CHANNEL__), (__ICPSC__)); \
                          } while(0U)

/**
  * @brief  Get the TIM Input Capture prescaler on runtime.
  * @param  \__HANDLE__ TIM handle.
  * @param  \__CHANNEL__ TIM Channels to be configured.
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: get input capture 1 prescaler value
  *            @arg GPT_CHANNEL_2: get input capture 2 prescaler value
  *            @arg GPT_CHANNEL_3: get input capture 3 prescaler value
  *            @arg GPT_CHANNEL_4: get input capture 4 prescaler value
  * @retval The input capture prescaler can be one of the following values:
  *            @arg GPT_ICPSC_DIV1: no prescaler
  *            @arg GPT_ICPSC_DIV2: capture is done once every 2 events
  *            @arg GPT_ICPSC_DIV4: capture is done once every 4 events
  *            @arg GPT_ICPSC_DIV8: capture is done once every 8 events
  */
#define __HAL_GPT_GET_ICPRESCALER(__HANDLE__, __CHANNEL__)  \
  (((__CHANNEL__) == GPT_CHANNEL_1) ? ((__HANDLE__)->Instance->CCMR1 & GPT_CCMR1_IC1PSC) :\
   ((__CHANNEL__) == GPT_CHANNEL_2) ? (((__HANDLE__)->Instance->CCMR1 & GPT_CCMR1_IC2PSC) >> 8U) :\
   ((__CHANNEL__) == GPT_CHANNEL_3) ? ((__HANDLE__)->Instance->CCMR2 & GPT_CCMR2_IC3PSC) :\
   (((__HANDLE__)->Instance->CCMR2 & GPT_CCMR2_IC4PSC)) >> 8U)

/**
  * @brief  Set the Update Request Source (URS) bit of the TIMx_CR1 register
  * @param  \__HANDLE__ TIM handle.
  * @note  When the USR bit of the TIMx_CR1 register is set, only counter
  *        overflow/underflow generates an update interrupt or DMA request (if
  *        enabled)
  * @retval None
  */
#define __HAL_GPT_URS_ENABLE(__HANDLE__) \
    ((__HANDLE__)->Instance->CR1|= (GPT_CR1_URS))

/**
  * @brief  Reset the Update Request Source (URS) bit of the TIMx_CR1 register
  * @param  \__HANDLE__ TIM handle.
  * @note  When the USR bit of the TIMx_CR1 register is reset, any of the
  *        following events generate an update interrupt or DMA request (if
  *        enabled):
  *          _ Counter overflow/underflow
  *          _ Setting the UG bit
  *          _ Update generation through the slave mode controller
  * @retval None
  */
#define __HAL_GPT_URS_DISABLE(__HANDLE__) \
      ((__HANDLE__)->Instance->CR1&=~(GPT_CR1_URS))

/**
  * @brief  Sets the TIM Capture x input polarity on runtime.
  * @param  \__HANDLE__ TIM handle.
  * @param  \__CHANNEL__ TIM Channels to be configured.
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  *            @arg GPT_CHANNEL_3: TIM Channel 3 selected
  *            @arg GPT_CHANNEL_4: TIM Channel 4 selected
  * @param  \__POLARITY__ Polarity for TIx source
  *            @arg GPT_INPUTCHANNELPOLARITY_RISING: Rising Edge
  *            @arg GPT_INPUTCHANNELPOLARITY_FALLING: Falling Edge
  *            @arg GPT_INPUTCHANNELPOLARITY_BOTHEDGE: Rising and Falling Edge
  * @note  The polarity GPT_INPUTCHANNELPOLARITY_BOTHEDGE is not authorized  for TIM Channel 4.
  * @retval None
  */
#define __HAL_GPT_SET_CAPTUREPOLARITY(__HANDLE__, __CHANNEL__, __POLARITY__)                          \
                       do{                                                                            \
                           GPT_RESET_CAPTUREPOLARITY((__HANDLE__), (__CHANNEL__));               \
                           GPT_SET_CAPTUREPOLARITY((__HANDLE__), (__CHANNEL__), (__POLARITY__)); \
                         }while(0U)
/**
  * @}
  */

/* Include TIM HAL Extension module */
#include "bf0_hal_tim_ex.h"

/* Exported functions --------------------------------------------------------*/
/** @addtogroup GPT_Exported_Functions
  * @{
  */

/** @addtogroup GPT_Exported_Functions_Group1
  * @{
  */

/* Time Base functions ********************************************************/

/**
  * @brief  Initializes the TIM Time base Unit according to the specified
  *         parameters in the GPT_HandleTypeDef and create the associated handle.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_Base_Init(GPT_HandleTypeDef *htim);

/**
  * @brief  DeInitializes the TIM Base peripheral
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_Base_DeInit(GPT_HandleTypeDef *htim);

/**
  * @brief  Initializes the TIM Base MSP.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval None
  */
void HAL_GPT_Base_MspInit(GPT_HandleTypeDef *htim);

/**
  * @brief  DeInitializes TIM Base MSP.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval None
  */
void HAL_GPT_Base_MspDeInit(GPT_HandleTypeDef *htim);

/* Blocking mode: Polling */
/**
  * @brief  Starts the TIM Base generation.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_Base_Start(GPT_HandleTypeDef *htim);

/**
  * @brief  Stops the TIM Base generation.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_Base_Stop(GPT_HandleTypeDef *htim);

/* Non-Blocking mode: Interrupt */
/**
  * @brief  Starts the TIM Base generation in interrupt mode.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_Base_Start_IT(GPT_HandleTypeDef *htim);

/**
  * @brief  Stops the TIM Base generation in interrupt mode.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_Base_Stop_IT(GPT_HandleTypeDef *htim);

/* Non-Blocking mode: DMA */
/**
  * @brief  Starts the TIM Base generation in DMA mode.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  pData The source Buffer address.
  * @param  Length The length of data to be transferred from memory to peripheral.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_Base_Start_DMA(GPT_HandleTypeDef *htim, uint32_t *pData, uint16_t Length);

/**
  * @brief  Stops the TIM Base generation in DMA mode.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_Base_Stop_DMA(GPT_HandleTypeDef *htim);
/**
  * @}
  */

/** @addtogroup GPT_Exported_Functions_Group2
  * @{
  */

/* Timer Output Compare functions **********************************************/
/**
  * @brief  Initializes the TIM Output Compare according to the specified
  *         parameters in the GPT_HandleTypeDef and create the associated handle.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_OC_Init(GPT_HandleTypeDef *htim);

/**
  * @brief  DeInitializes the TIM peripheral
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_OC_DeInit(GPT_HandleTypeDef *htim);

/**
  * @brief  Initializes the TIM Output Compare MSP.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval None
  */
void HAL_GPT_OC_MspInit(GPT_HandleTypeDef *htim);

/**
  * @brief  DeInitializes TIM Output Compare MSP.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval None
  */
void HAL_GPT_OC_MspDeInit(GPT_HandleTypeDef *htim);

/* Blocking mode: Polling */
/**
  * @brief  Starts the TIM Output Compare signal generation.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Channel TIM Channel to be enabled.
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  *            @arg GPT_CHANNEL_3: TIM Channel 3 selected
  *            @arg GPT_CHANNEL_4: TIM Channel 4 selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_OC_Start(GPT_HandleTypeDef *htim, uint32_t Channel);

/**
  * @brief  Stops the TIM Output Compare signal generation.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Channel TIM Channel to be disabled.
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  *            @arg GPT_CHANNEL_3: TIM Channel 3 selected
  *            @arg GPT_CHANNEL_4: TIM Channel 4 selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_OC_Stop(GPT_HandleTypeDef *htim, uint32_t Channel);

/* Non-Blocking mode: Interrupt */
/**
  * @brief  Starts the TIM Output Compare signal generation in interrupt mode.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Channel TIM Channel to be enabled.
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  *            @arg GPT_CHANNEL_3: TIM Channel 3 selected
  *            @arg GPT_CHANNEL_4: TIM Channel 4 selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_OC_Start_IT(GPT_HandleTypeDef *htim, uint32_t Channel);

/**
  * @brief  Stops the TIM Output Compare signal generation in interrupt mode.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Channel TIM Channel to be disabled.
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  *            @arg GPT_CHANNEL_3: TIM Channel 3 selected
  *            @arg GPT_CHANNEL_4: TIM Channel 4 selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_OC_Stop_IT(GPT_HandleTypeDef *htim, uint32_t Channel);

/* Non-Blocking mode: DMA */
/**
  * @brief  Starts the TIM Output Compare signal generation in DMA mode.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Channel TIM Channel to be enabled.
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  *            @arg GPT_CHANNEL_3: TIM Channel 3 selected
  *            @arg GPT_CHANNEL_4: TIM Channel 4 selected
  * @param  pData The source Buffer address.
  * @param  Length The length of data to be transferred from memory to TIM peripheral
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_OC_Start_DMA(GPT_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);

/**
  * @brief  Stops the TIM Output Compare signal generation in DMA mode.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Channel TIM Channel to be disabled.
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  *            @arg GPT_CHANNEL_3: TIM Channel 3 selected
  *            @arg GPT_CHANNEL_4: TIM Channel 4 selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_OC_Stop_DMA(GPT_HandleTypeDef *htim, uint32_t Channel);

/**
  * @}
  */

/** @addtogroup GPT_Exported_Functions_Group3
  * @{
  */
/* Timer PWM functions *********************************************************/

/**
  * @brief  Initializes the TIM PWM Time Base according to the specified
  *         parameters in the GPT_HandleTypeDef and create the associated handle.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_PWM_Init(GPT_HandleTypeDef *htim);

/**
  * @brief  DeInitializes the TIM peripheral
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_PWM_DeInit(GPT_HandleTypeDef *htim);
/**
  * @brief  Initializes the TIM PWM MSP.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval None
  */
void HAL_GPT_PWM_MspInit(GPT_HandleTypeDef *htim);

/**
  * @brief  DeInitializes TIM PWM MSP.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval None
  */
void HAL_GPT_PWM_MspDeInit(GPT_HandleTypeDef *htim);


/* Blocking mode: Polling */
/**
  * @brief  Starts the PWM signal generation.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Channel TIM Channels to be enabled.
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  *            @arg GPT_CHANNEL_3: TIM Channel 3 selected
  *            @arg GPT_CHANNEL_4: TIM Channel 4 selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_PWM_Start(GPT_HandleTypeDef *htim, uint32_t Channel);
/**
  * @brief  Stops the PWM signal generation.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Channel TIM Channels to be disabled.
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  *            @arg GPT_CHANNEL_3: TIM Channel 3 selected
  *            @arg GPT_CHANNEL_4: TIM Channel 4 selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_PWM_Stop(GPT_HandleTypeDef *htim, uint32_t Channel);

/* Non-Blocking mode: Interrupt */

/**
  * @brief  Starts the PWM signal generation in interrupt mode.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Channel TIM Channel to be enabled.
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  *            @arg GPT_CHANNEL_3: TIM Channel 3 selected
  *            @arg GPT_CHANNEL_4: TIM Channel 4 selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_PWM_Start_IT(GPT_HandleTypeDef *htim, uint32_t Channel);

/**
  * @brief  Stops the PWM signal generation in interrupt mode.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Channel TIM Channels to be disabled.
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  *            @arg GPT_CHANNEL_3: TIM Channel 3 selected
  *            @arg GPT_CHANNEL_4: TIM Channel 4 selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_PWM_Stop_IT(GPT_HandleTypeDef *htim, uint32_t Channel);

/* Non-Blocking mode: DMA */
/**
  * @brief  Starts the TIM PWM signal generation in DMA mode.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Channel TIM Channels to be enabled.
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  *            @arg GPT_CHANNEL_3: TIM Channel 3 selected
  *            @arg GPT_CHANNEL_4: TIM Channel 4 selected
  * @param  pData The source Buffer address.
  * @param  Length The length of data to be transferred from memory to TIM peripheral
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_PWM_Start_DMA(GPT_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);

/**
  * @brief  Stops the TIM PWM signal generation in DMA mode.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Channel TIM Channels to be disabled.
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  *            @arg GPT_CHANNEL_3: TIM Channel 3 selected
  *            @arg GPT_CHANNEL_4: TIM Channel 4 selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_PWM_Stop_DMA(GPT_HandleTypeDef *htim, uint32_t Channel);

/**
  * @}
  */

/** @addtogroup GPT_Exported_Functions_Group4
  * @{
  */
/* Timer Input Capture functions ***********************************************/
/**
  * @brief  Initializes the TIM Input Capture Time base according to the specified
  *         parameters in the GPT_HandleTypeDef and create the associated handle.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_IC_Init(GPT_HandleTypeDef *htim);

/**
  * @brief  DeInitializes the TIM peripheral
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_IC_DeInit(GPT_HandleTypeDef *htim);

/**
  * @brief  Initializes the TIM INput Capture MSP.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval None
  */
void HAL_GPT_IC_MspInit(GPT_HandleTypeDef *htim);

/**
  * @brief  DeInitializes TIM Input Capture MSP.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval None
  */
void HAL_GPT_IC_MspDeInit(GPT_HandleTypeDef *htim);

/* Blocking mode: Polling */
/**
  * @brief  Starts the TIM Input Capture measurement.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Channel TIM Channels to be enabled.
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  *            @arg GPT_CHANNEL_3: TIM Channel 3 selected
  *            @arg GPT_CHANNEL_4: TIM Channel 4 selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_IC_Start(GPT_HandleTypeDef *htim, uint32_t Channel);

/**
  * @brief  Stops the TIM Input Capture measurement.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Channel TIM Channels to be disabled.
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  *            @arg GPT_CHANNEL_3: TIM Channel 3 selected
  *            @arg GPT_CHANNEL_4: TIM Channel 4 selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_IC_Stop(GPT_HandleTypeDef *htim, uint32_t Channel);

/* Non-Blocking mode: Interrupt */
/**
  * @brief  Starts the TIM Input Capture measurement in interrupt mode.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Channel TIM Channels to be enabled.
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  *            @arg GPT_CHANNEL_3: TIM Channel 3 selected
  *            @arg GPT_CHANNEL_4: TIM Channel 4 selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_IC_Start_IT(GPT_HandleTypeDef *htim, uint32_t Channel);

/**
  * @brief  Stops the TIM Input Capture measurement in interrupt mode.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Channel TIM Channels to be disabled.
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  *            @arg GPT_CHANNEL_3: TIM Channel 3 selected
  *            @arg GPT_CHANNEL_4: TIM Channel 4 selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_IC_Stop_IT(GPT_HandleTypeDef *htim, uint32_t Channel);

/* Non-Blocking mode: DMA */
/**
  * @brief  Starts the TIM Input Capture measurement on in DMA mode.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Channel TIM Channels to be enabled.
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  *            @arg GPT_CHANNEL_3: TIM Channel 3 selected
  *            @arg GPT_CHANNEL_4: TIM Channel 4 selected
  * @param  pData The destination Buffer address.
  * @param  Length The length of data to be transferred from TIM peripheral to memory.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_IC_Start_DMA(GPT_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);

/**
  * @brief  Stops the TIM Input Capture measurement on in DMA mode.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Channel TIM Channels to be disabled.
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  *            @arg GPT_CHANNEL_3: TIM Channel 3 selected
  *            @arg GPT_CHANNEL_4: TIM Channel 4 selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_IC_Stop_DMA(GPT_HandleTypeDef *htim, uint32_t Channel);

/**
  * @}
  */

/** @addtogroup GPT_Exported_Functions_Group5
  * @{
  */
/* Timer One Pulse functions ***************************************************/
/**
  * @brief  Initializes the TIM One Pulse Time Base according to the specified
  *         parameters in the GPT_HandleTypeDef and create the associated handle.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  OnePulseMode Select the One pulse mode.
  *         This parameter can be one of the following values:
  *            @arg GPT_OPMODE_SINGLE: Only one pulse will be generated.
  *            @arg GPT_OPMODE_REPETITIVE: Repetitive pulses will be generated.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_OnePulse_Init(GPT_HandleTypeDef *htim, uint32_t OnePulseMode);
/**
  * @brief  DeInitializes the TIM One Pulse
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_OnePulse_DeInit(GPT_HandleTypeDef *htim);
/**
  * @brief  Initializes the TIM One Pulse MSP.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval None
  */
void HAL_GPT_OnePulse_MspInit(GPT_HandleTypeDef *htim);
/**
  * @brief  DeInitializes TIM One Pulse MSP.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval None
  */
void HAL_GPT_OnePulse_MspDeInit(GPT_HandleTypeDef *htim);

/* Blocking mode: Polling */
/**
  * @brief  Starts the TIM One Pulse signal generation.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  OutputChannel  TIM Channels to be enabled.
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  * @retval HAL status
  */

HAL_StatusTypeDef HAL_GPT_OnePulse_Start(GPT_HandleTypeDef *htim, uint32_t OutputChannel);
/**
  * @brief  Stops the TIM One Pulse signal generation.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  OutputChannel  TIM Channels to be disable.
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_OnePulse_Stop(GPT_HandleTypeDef *htim, uint32_t OutputChannel);

/* Non-Blocking mode: Interrupt */
/**
  * @brief  Starts the TIM One Pulse signal generation in interrupt mode.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  OutputChannel  TIM Channels to be enabled.
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_OnePulse_Start_IT(GPT_HandleTypeDef *htim, uint32_t OutputChannel);
/**
  * @brief  Stops the TIM One Pulse signal generation in interrupt mode.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  OutputChannel  TIM Channels to be enabled.
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_OnePulse_Stop_IT(GPT_HandleTypeDef *htim, uint32_t OutputChannel);

/**
  * @}
  */

/** @addtogroup GPT_Exported_Functions_Group6
  * @{
  */
/* Timer Encoder functions *****************************************************/
/**
  * @brief  Initializes the TIM Encoder Interface and create the associated handle.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  sConfig TIM Encoder Interface configuration structure
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_Encoder_Init(GPT_HandleTypeDef *htim,  GPT_Encoder_InitTypeDef *sConfig);
/**
  * @brief  DeInitializes the TIM Encoder interface
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_Encoder_DeInit(GPT_HandleTypeDef *htim);
/**
  * @brief  Initializes the TIM Encoder Interface MSP.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval None
  */
void HAL_GPT_Encoder_MspInit(GPT_HandleTypeDef *htim);
/**
  * @brief  DeInitializes TIM Encoder Interface MSP.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval None
  */
void HAL_GPT_Encoder_MspDeInit(GPT_HandleTypeDef *htim);

/* Blocking mode: Polling */

/**
  * @brief  Starts the TIM Encoder Interface.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Channel TIM Channels to be enabled.
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  *            @arg GPT_CHANNEL_ALL: TIM Channel 1 and TIM Channel 2 are selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_Encoder_Start(GPT_HandleTypeDef *htim, uint32_t Channel);
/**
  * @brief  Stops the TIM Encoder Interface.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Channel TIM Channels to be disabled.
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  *            @arg GPT_CHANNEL_ALL: TIM Channel 1 and TIM Channel 2 are selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_Encoder_Stop(GPT_HandleTypeDef *htim, uint32_t Channel);

/* Non-Blocking mode: Interrupt */
/**
  * @brief  Starts the TIM Encoder Interface in interrupt mode.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Channel TIM Channels to be enabled.
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  *            @arg GPT_CHANNEL_ALL: TIM Channel 1 and TIM Channel 2 are selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_Encoder_Start_IT(GPT_HandleTypeDef *htim, uint32_t Channel);
/**
  * @brief  Stops the TIM Encoder Interface in interrupt mode.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Channel TIM Channels to be disabled.
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  *            @arg GPT_CHANNEL_ALL: TIM Channel 1 and TIM Channel 2 are selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_Encoder_Stop_IT(GPT_HandleTypeDef *htim, uint32_t Channel);

/* Non-Blocking mode: DMA */
/**
  * @brief  Starts the TIM Encoder Interface in DMA mode.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Channel TIM Channels to be enabled.
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  *            @arg GPT_CHANNEL_ALL: TIM Channel 1 and TIM Channel 2 are selected
  * @param  pData1 The destination Buffer address for IC1.
  * @param  pData2 The destination Buffer address for IC2.
  * @param  Length The length of data to be transferred from TIM peripheral to memory.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_Encoder_Start_DMA(GPT_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData1, uint32_t *pData2, uint16_t Length);
/**
  * @brief  Stops the TIM Encoder Interface in DMA mode.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Channel TIM Channels to be enabled.
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  *            @arg GPT_CHANNEL_ALL: TIM Channel 1 and TIM Channel 2 are selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_Encoder_Stop_DMA(GPT_HandleTypeDef *htim, uint32_t Channel);

/**
  * @}
  */

/** @addtogroup GPT_Exported_Functions_Group7
  * @{
  */
/* Interrupt Handler functions  **********************************************/
/**
  * @brief  This function handles TIM interrupts requests.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval None
  */
void HAL_GPT_IRQHandler(GPT_HandleTypeDef *htim);

/**
  * @}
  */

/** @addtogroup GPT_Exported_Functions_Group8
  * @{
  */
/* Control functions  *********************************************************/
/**
  * @brief  Initializes the TIM Output Compare Channels according to the specified
  *         parameters in the GPT_OC_InitTypeDef.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  sConfig TIM Output Compare configuration structure
  * @param  Channel TIM Channels to be enabled.
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  *            @arg GPT_CHANNEL_3: TIM Channel 3 selected
  *            @arg GPT_CHANNEL_4: TIM Channel 4 selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_OC_ConfigChannel(GPT_HandleTypeDef *htim, GPT_OC_InitTypeDef *sConfig, uint32_t Channel);
/**
  * @brief  Initializes the TIM Input Capture Channels according to the specified
  *         parameters in the GPT_IC_InitTypeDef.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  sConfig TIM Input Capture configuration structure
  * @param  Channel TIM Channels to be enabled.
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  *            @arg GPT_CHANNEL_3: TIM Channel 3 selected
  *            @arg GPT_CHANNEL_4: TIM Channel 4 selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_IC_ConfigChannel(GPT_HandleTypeDef *htim, GPT_IC_InitTypeDef *sConfig, uint32_t Channel);
/**
  * @brief  Initializes the TIM PWM  channels according to the specified
  *         parameters in the GPT_OC_InitTypeDef.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  sConfig TIM PWM configuration structure
  * @param  Channel TIM Channels to be enabled.
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  *            @arg GPT_CHANNEL_3: TIM Channel 3 selected
  *            @arg GPT_CHANNEL_4: TIM Channel 4 selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_PWM_ConfigChannel(GPT_HandleTypeDef *htim, GPT_OC_InitTypeDef *sConfig, uint32_t Channel);
/**
  * @brief  Initializes the TIM One Pulse Channels according to the specified
  *         parameters in the GPT_OnePulse_InitTypeDef.
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  sConfig TIM One Pulse configuration structure
  * @param  OutputChannel TIM Channels to be enabled.
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  * @param  InputChannel TIM Channels to be enabled.
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_OnePulse_ConfigChannel(GPT_HandleTypeDef *htim, GPT_OnePulse_InitTypeDef *sConfig, uint32_t OutputChannel,  uint32_t InputChannel);
/**
  * @brief  Configures the OCRef clear feature
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  sClearInputConfig pointer to a GPT_ClearInputConfigTypeDef structure that
  *         contains the OCREF clear feature and parameters for the TIM peripheral.
  * @param  Channel specifies the TIM Channel.
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  *            @arg GPT_CHANNEL_3: TIM Channel 3 selected
  *            @arg GPT_CHANNEL_4: TIM Channel 4 selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_ConfigOCrefClear(GPT_HandleTypeDef *htim, GPT_ClearInputConfigTypeDef *sClearInputConfig, uint32_t Channel);

/**
  * @brief   Configures the clock source to be used
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  sClockSourceConfig pointer to a GPT_ClockConfigTypeDef structure that
  *         contains the clock source information for the TIM peripheral.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_ConfigClockSource(GPT_HandleTypeDef *htim, GPT_ClockConfigTypeDef *sClockSourceConfig);

/**
  * @brief  Selects the signal connected to the TI1 input: direct from CH1_input
  *         or a XOR combination between CH1_input, CH2_input & CH3_input
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  TI1_Selection Indicate whether or not channel 1 is connected to the
  *         output of a XOR gate.
  *         This parameter can be one of the following values:
  *            @arg GPT_TI1SELECTION_CH1: The TIMx_CH1 pin is connected to TI1 input
  *            @arg GPT_TI1SELECTION_XORCOMBINATION: The TIMx_CH1, CH2 and CH3
  *            pins are connected to the TI1 input (XOR combination)
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_ConfigTI1Input(GPT_HandleTypeDef *htim, uint32_t TI1_Selection);
/**
  * @brief  Configures the TIM in Slave mode
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  sSlaveConfig pointer to a GPT_SlaveConfigTypeDef structure that
  *         contains the selected trigger (internal trigger input, filtered
  *         timer input or external trigger input) and the ) and the Slave
  *         mode (Disable, Reset, Gated, Trigger, External clock mode 1).
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_SlaveConfigSynchronization(GPT_HandleTypeDef *htim, GPT_SlaveConfigTypeDef *sSlaveConfig);
/**
  * @brief  Configures the TIM in Slave mode in interrupt mode
  * @param  htim TIM handle.
  * @param  sSlaveConfig pointer to a GPT_SlaveConfigTypeDef structure that
  *         contains the selected trigger (internal trigger input, filtered
  *         timer input or external trigger input) and the ) and the Slave
  *         mode (Disable, Reset, Gated, Trigger, External clock mode 1).
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_SlaveConfigSynchronization_IT(GPT_HandleTypeDef *htim, GPT_SlaveConfigTypeDef *sSlaveConfig);

/**
  * @brief  Configure the DMA Burst to transfer Data from the memory to the TIM peripheral
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  BurstBaseAddress TIM Base address from when the DMA will starts the Data write.
  *         This parameters can be on of the following values:
  *            @arg GPT_DMABASE_CR1
  *            @arg GPT_DMABASE_CR2
  *            @arg GPT_DMABASE_SMCR
  *            @arg GPT_DMABASE_DIER
  *            @arg GPT_DMABASE_SR
  *            @arg GPT_DMABASE_EGR
  *            @arg GPT_DMABASE_CCMR1
  *            @arg GPT_DMABASE_CCMR2
  *            @arg GPT_DMABASE_CCER
  *            @arg GPT_DMABASE_CNT
  *            @arg GPT_DMABASE_PSC
  *            @arg GPT_DMABASE_ARR
  *            @arg GPT_DMABASE_RCR
  *            @arg GPT_DMABASE_CCR1
  *            @arg GPT_DMABASE_CCR2
  *            @arg GPT_DMABASE_CCR3
  *            @arg GPT_DMABASE_CCR4
  *            @arg GPT_DMABASE_BDTR
  *            @arg GPT_DMABASE_DCR
  * @param  BurstRequestSrc TIM DMA Request sources.
  *         This parameters can be on of the following values:
  *            @arg GPT_DMA_UPDATE: TIM update Interrupt source
  *            @arg GPT_DMA_CC1: TIM Capture Compare 1 DMA source
  *            @arg GPT_DMA_CC2: TIM Capture Compare 2 DMA source
  *            @arg GPT_DMA_CC3: TIM Capture Compare 3 DMA source
  *            @arg GPT_DMA_CC4: TIM Capture Compare 4 DMA source
  *            @arg GPT_DMA_COM: TIM Commutation DMA source
  *            @arg GPT_DMA_TRIGGER: TIM Trigger DMA source
  * @param  BurstBuffer The Buffer address.
  * @param  BurstLength DMA Burst length. This parameter can be one value
  *         between GPT_DMABURSTLENGTH_1TRANSFER and GPT_DMABURSTLENGTH_18TRANSFERS.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_DMABurst_WriteStart(GPT_HandleTypeDef *htim, uint32_t BurstBaseAddress, uint32_t BurstRequestSrc, \
        uint32_t  *BurstBuffer, uint32_t  BurstLength);
/**
  * @brief  Stops the TIM DMA Burst mode
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  BurstRequestSrc TIM DMA Request sources to disable
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_DMABurst_WriteStop(GPT_HandleTypeDef *htim, uint32_t BurstRequestSrc);

/**
  * @brief  Configure the DMA Burst to transfer Data from the TIM peripheral to the memory
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  BurstBaseAddress TIM Base address from when the DMA will starts the Data read.
  *         This parameters can be on of the following values:
  *            @arg GPT_DMABASE_CR1
  *            @arg GPT_DMABASE_CR2
  *            @arg GPT_DMABASE_SMCR
  *            @arg GPT_DMABASE_DIER
  *            @arg GPT_DMABASE_SR
  *            @arg GPT_DMABASE_EGR
  *            @arg GPT_DMABASE_CCMR1
  *            @arg GPT_DMABASE_CCMR2
  *            @arg GPT_DMABASE_CCER
  *            @arg GPT_DMABASE_CNT
  *            @arg GPT_DMABASE_PSC
  *            @arg GPT_DMABASE_ARR
  *            @arg GPT_DMABASE_RCR
  *            @arg GPT_DMABASE_CCR1
  *            @arg GPT_DMABASE_CCR2
  *            @arg GPT_DMABASE_CCR3
  *            @arg GPT_DMABASE_CCR4
  *            @arg GPT_DMABASE_BDTR
  *            @arg GPT_DMABASE_DCR
  * @param  BurstRequestSrc TIM DMA Request sources.
  *         This parameters can be on of the following values:
  *            @arg GPT_DMA_UPDATE: TIM update Interrupt source
  *            @arg GPT_DMA_CC1: TIM Capture Compare 1 DMA source
  *            @arg GPT_DMA_CC2: TIM Capture Compare 2 DMA source
  *            @arg GPT_DMA_CC3: TIM Capture Compare 3 DMA source
  *            @arg GPT_DMA_CC4: TIM Capture Compare 4 DMA source
  *            @arg GPT_DMA_COM: TIM Commutation DMA source
  *            @arg GPT_DMA_TRIGGER: TIM Trigger DMA source
  * @param  BurstBuffer The Buffer address.
  * @param  BurstLength DMA Burst length. This parameter can be one value
  *         between GPT_DMABURSTLENGTH_1TRANSFER and GPT_DMABURSTLENGTH_18TRANSFERS.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_DMABurst_ReadStart(GPT_HandleTypeDef *htim, uint32_t BurstBaseAddress, uint32_t BurstRequestSrc, \
        uint32_t  *BurstBuffer, uint32_t  BurstLength);

/**
  * @brief  Stop the DMA burst reading
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  BurstRequestSrc TIM DMA Request sources to disable.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_GPT_DMABurst_ReadStop(GPT_HandleTypeDef *htim, uint32_t BurstRequestSrc);

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
HAL_StatusTypeDef HAL_GPT_GenerateEvent(GPT_HandleTypeDef *htim, uint32_t EventSource);
/**
  * @brief  Read the captured value from Capture Compare unit
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Channel TIM Channels to be enabled.
  *          This parameter can be one of the following values:
  *            @arg GPT_CHANNEL_1: TIM Channel 1 selected
  *            @arg GPT_CHANNEL_2: TIM Channel 2 selected
  *            @arg GPT_CHANNEL_3: TIM Channel 3 selected
  *            @arg GPT_CHANNEL_4: TIM Channel 4 selected
  * @retval Captured value
  */
uint32_t HAL_GPT_ReadCapturedValue(GPT_HandleTypeDef *htim, uint32_t Channel);

/**
  * @}
  */

/** @addtogroup GPT_Exported_Functions_Group9
  * @{
  */
/* Callback in non blocking modes (Interrupt and DMA) *************************/
/**
  * @brief  HW timer periodical timeout call back
  * @param[in]  htim: low leverl HW timer device.
  */
void HAL_GPT_PeriodElapsedCallback(GPT_HandleTypeDef *htim);
/**
  * @brief  Output Compare callback in non blocking mode
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval None
  */
void HAL_GPT_OC_DelayElapsedCallback(GPT_HandleTypeDef *htim);
/**
  * @brief  Input Capture callback in non blocking mode
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval None
  */
void HAL_GPT_IC_CaptureCallback(GPT_HandleTypeDef *htim);
/**
  * @brief  PWM Pulse finished callback in non blocking mode
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval None
  */
void HAL_GPT_PWM_PulseFinishedCallback(GPT_HandleTypeDef *htim);

/**
  * @brief  PWM Pulse finished half complete callback in non-blocking mode
  * @param  htim TIM handle
  * @retval None
  */
void HAL_GPT_PWM_PulseFinishedHalfCpltCallback(GPT_HandleTypeDef *htim);

/**
  * @brief  Hall Trigger detection callback in non blocking mode
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval None
  */
void HAL_GPT_TriggerCallback(GPT_HandleTypeDef *htim);
/**
  * @brief  Timer error callback in non blocking mode
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval None
  */
void HAL_GPT_ErrorCallback(GPT_HandleTypeDef *htim);

/**
  * @}
  */

/** @addtogroup GPT_Exported_Functions_Group10
  * @{
  */
/* Peripheral State functions  **************************************************/
/**
  * @brief  Return the TIM Base state
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval HAL state
  */
HAL_GPT_StateTypeDef HAL_GPT_Base_GetState(GPT_HandleTypeDef *htim);
/**
  * @brief  Return the TIM OC state
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval HAL state
  */
HAL_GPT_StateTypeDef HAL_GPT_OC_GetState(GPT_HandleTypeDef *htim);
/**
  * @brief  Return the TIM PWM state
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval HAL state
  */
HAL_GPT_StateTypeDef HAL_GPT_PWM_GetState(GPT_HandleTypeDef *htim);
/**
  * @brief  Return the TIM Input Capture state
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval HAL state
  */
HAL_GPT_StateTypeDef HAL_GPT_IC_GetState(GPT_HandleTypeDef *htim);
/**
  * @brief  Return the TIM One Pulse Mode state
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval HAL state
  */
HAL_GPT_StateTypeDef HAL_GPT_OnePulse_GetState(GPT_HandleTypeDef *htim);
/**
  * @brief  Return the TIM Encoder Mode state
  * @param  htim pointer to a GPT_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval HAL state
  */
HAL_GPT_StateTypeDef HAL_GPT_Encoder_GetState(GPT_HandleTypeDef *htim);

/**
  * @}
  */

/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup GPT_Private_Macros TIM Private Macros
  * @{
  */

/** @defgroup GPT_IS_GPT_Definitions TIM Private macros to check input parameters
  * @{
  */
#define IS_GPT_COUNTER_MODE(MODE) (((MODE) == GPT_COUNTERMODE_UP)              || \
                                   ((MODE) == GPT_COUNTERMODE_DOWN)            || \
                                   ((MODE) == GPT_COUNTERMODE_CENTERALIGNED3))

#define IS_GPT_PWM_MODE(MODE) (((MODE) == GPT_OCMODE_PWM1) || \
                               ((MODE) == GPT_OCMODE_PWM2))

#define IS_GPT_OC_MODE(MODE) (((MODE) == GPT_OCMODE_TIMING)       || \
                          ((MODE) == GPT_OCMODE_ACTIVE)           || \
                          ((MODE) == GPT_OCMODE_INACTIVE)         || \
                          ((MODE) == GPT_OCMODE_TOGGLE)           || \
                          ((MODE) == GPT_OCMODE_FORCED_ACTIVE)    || \
                          ((MODE) == GPT_OCMODE_FORCED_INACTIVE))

#define IS_GPT_FAST_STATE(STATE) (((STATE) == GPT_OCFAST_DISABLE) || \
                                  ((STATE) == GPT_OCFAST_ENABLE))

#define IS_GPT_OC_POLARITY(POLARITY) (((POLARITY) == GPT_OCPOLARITY_HIGH) || \
                                      ((POLARITY) == GPT_OCPOLARITY_LOW))

#define IS_GPT_OCN_POLARITY(POLARITY) (((POLARITY) == GPT_OCNPOLARITY_HIGH) || \
                                       ((POLARITY) == GPT_OCNPOLARITY_LOW))

#define IS_GPT_OCIDLE_STATE(STATE) (((STATE) == GPT_OCIDLESTATE_SET) || \
                                    ((STATE) == GPT_OCIDLESTATE_RESET))

#define IS_GPT_OCNIDLE_STATE(STATE) (((STATE) == GPT_OCNIDLESTATE_SET) || \
                                    ((STATE) == GPT_OCNIDLESTATE_RESET))

#define IS_GPT_CHANNELS(CHANNEL) (((CHANNEL) == GPT_CHANNEL_1) || \
                                  ((CHANNEL) == GPT_CHANNEL_2) || \
                                  ((CHANNEL) == GPT_CHANNEL_3) || \
                                  ((CHANNEL) == GPT_CHANNEL_4) || \
                                  ((CHANNEL) == GPT_CHANNEL_ALL))

#define IS_GPT_OPM_CHANNELS(CHANNEL) (((CHANNEL) == GPT_CHANNEL_1) || \
                                      ((CHANNEL) == GPT_CHANNEL_2))

#define IS_GPT_COMPLEMENTARY_CHANNELS(CHANNEL) (((CHANNEL) == GPT_CHANNEL_1) || \
                                                ((CHANNEL) == GPT_CHANNEL_2) || \
                                                ((CHANNEL) == GPT_CHANNEL_3))

#define IS_GPT_IC_POLARITY(POLARITY) (((POLARITY) == GPT_ICPOLARITY_RISING)   || \
                                      ((POLARITY) == GPT_ICPOLARITY_FALLING)  || \
                                      ((POLARITY) == GPT_ICPOLARITY_BOTHEDGE))

#define IS_GPT_IC_SELECTION(SELECTION) (((SELECTION) == GPT_ICSELECTION_DIRECTTI) || \
                                        ((SELECTION) == GPT_ICSELECTION_INDIRECTTI) || \
                                        ((SELECTION) == GPT_ICSELECTION_TRC))

#define IS_GPT_IC_PRESCALER(PRESCALER) (((PRESCALER) == GPT_ICPSC_DIV1) || \
                                        ((PRESCALER) == GPT_ICPSC_DIV2) || \
                                        ((PRESCALER) == GPT_ICPSC_DIV4) || \
                                        ((PRESCALER) == GPT_ICPSC_DIV8))

#define IS_GPT_OPM_MODE(MODE) (((MODE) == GPT_OPMODE_SINGLE) || \
                               ((MODE) == GPT_OPMODE_REPETITIVE))

#define IS_GPT_DMA_SOURCE(SOURCE) ((((SOURCE) & 0xFFFF80FFU) == 0x00000000U) && ((SOURCE) != 0x00000000U))

#define IS_GPT_ENCODER_MODE(MODE) (((MODE) == GPT_ENCODERMODE_TI1) || \
                                   ((MODE) == GPT_ENCODERMODE_TI2) || \
                                   ((MODE) == GPT_ENCODERMODE_TI3))

#define IS_GPT_EVENT_SOURCE(SOURCE) ((((SOURCE) & 0xFFFFFF00U) == 0x00000000U) && ((SOURCE) != 0x00000000U))

#define IS_GPT_CLOCKSOURCE(CLOCK) (((CLOCK) == GPT_CLOCKSOURCE_INTERNAL) || \
                                   ((CLOCK) == GPT_CLOCKSOURCE_ETRMODE2) || \
                                   ((CLOCK) == GPT_CLOCKSOURCE_ITR0)     || \
                                   ((CLOCK) == GPT_CLOCKSOURCE_ITR1)     || \
                                   ((CLOCK) == GPT_CLOCKSOURCE_ITR2)     || \
                                   ((CLOCK) == GPT_CLOCKSOURCE_ITR3)     || \
                                   ((CLOCK) == GPT_CLOCKSOURCE_TI1ED)    || \
                                   ((CLOCK) == GPT_CLOCKSOURCE_TI1)      || \
                                   ((CLOCK) == GPT_CLOCKSOURCE_TI2)      || \
                                   ((CLOCK) == GPT_CLOCKSOURCE_ETRMODE1))

#define IS_GPT_CLOCKPOLARITY(POLARITY) (((POLARITY) == GPT_CLOCKPOLARITY_INVERTED)    || \
                                        ((POLARITY) == GPT_CLOCKPOLARITY_NONINVERTED) || \
                                        ((POLARITY) == GPT_CLOCKPOLARITY_RISING)      || \
                                        ((POLARITY) == GPT_CLOCKPOLARITY_FALLING)     || \
                                        ((POLARITY) == GPT_CLOCKPOLARITY_BOTHEDGE))

#define IS_GPT_CLOCKPRESCALER(PRESCALER) (((PRESCALER) == GPT_CLOCKPRESCALER_DIV1) || \
                                          ((PRESCALER) == GPT_CLOCKPRESCALER_DIV2) || \
                                          ((PRESCALER) == GPT_CLOCKPRESCALER_DIV4) || \
                                          ((PRESCALER) == GPT_CLOCKPRESCALER_DIV8))

#define IS_GPT_CLOCKFILTER(ICFILTER)      ((ICFILTER) <= 0x0FU)

#define IS_GPT_CLEARINPUT_SOURCE(SOURCE)  (((SOURCE) == GPT_CLEARINPUTSOURCE_NONE) || \
                                         ((SOURCE) == GPT_CLEARINPUTSOURCE_ETR))

#define IS_GPT_CLEARINPUT_POLARITY(POLARITY)   (((POLARITY) == GPT_CLEARINPUTPOLARITY_INVERTED) || \
                                               ((POLARITY) == GPT_CLEARINPUTPOLARITY_NONINVERTED))

#define IS_GPT_CLEARINPUT_PRESCALER(PRESCALER)   (((PRESCALER) == GPT_CLEARINPUTPRESCALER_DIV1) || \
                                                 ((PRESCALER) == GPT_CLEARINPUTPRESCALER_DIV2) || \
                                                 ((PRESCALER) == GPT_CLEARINPUTPRESCALER_DIV4) || \
                                                 ((PRESCALER) == GPT_CLEARINPUTPRESCALER_DIV8))

#define IS_GPT_CLEARINPUT_FILTER(ICFILTER) ((ICFILTER) <= 0x0FU)

#define IS_GPT_OSSR_STATE(STATE) (((STATE) == GPT_OSSR_ENABLE) || \
                                  ((STATE) == GPT_OSSR_DISABLE))

#define IS_GPT_OSSI_STATE(STATE) (((STATE) == GPT_OSSI_ENABLE) || \
                                  ((STATE) == GPT_OSSI_DISABLE))

#define IS_GPT_LOCK_LEVEL(LEVEL) (((LEVEL) == GPT_LOCKLEVEL_OFF) || \
                                  ((LEVEL) == GPT_LOCKLEVEL_1) || \
                                  ((LEVEL) == GPT_LOCKLEVEL_2) || \
                                  ((LEVEL) == GPT_LOCKLEVEL_3))

#define IS_GPT_BREAK_STATE(STATE) (((STATE) == GPT_BREAK_ENABLE) || \
                                   ((STATE) == GPT_BREAK_DISABLE))

#define IS_GPT_BREAK_POLARITY(POLARITY) (((POLARITY) == GPT_BREAKPOLARITY_LOW) || \
                                         ((POLARITY) == GPT_BREAKPOLARITY_HIGH))

#define IS_GPT_AUTOMATIC_OUTPUT_STATE(STATE) (((STATE) == GPT_AUTOMATICOUTPUT_ENABLE) || \
                                              ((STATE) == GPT_AUTOMATICOUTPUT_DISABLE))

#define IS_GPT_TRGO_SOURCE(SOURCE) (((SOURCE) == GPT_TRGO_RESET) || \
                                    ((SOURCE) == GPT_TRGO_ENABLE) || \
                                    ((SOURCE) == GPT_TRGO_UPDATE) || \
                                    ((SOURCE) == GPT_TRGO_OC1) || \
                                    ((SOURCE) == GPT_TRGO_OC1REF) || \
                                    ((SOURCE) == GPT_TRGO_OC2REF) || \
                                    ((SOURCE) == GPT_TRGO_OC3REF) || \
                                    ((SOURCE) == GPT_TRGO_OC4REF))

#define IS_GPT_SLAVE_MODE(MODE) (((MODE) == GPT_SLAVEMODE_DISABLE) || \
                                 ((MODE) == GPT_SLAVEMODE_GATED) || \
                                 ((MODE) == GPT_SLAVEMODE_RESET) || \
                                 ((MODE) == GPT_SLAVEMODE_TRIGGER) || \
                                 ((MODE) == GPT_SLAVEMODE_EXTERNAL1))

#define IS_GPT_MSM_STATE(STATE) (((STATE) == GPT_MASTERSLAVEMODE_ENABLE) || \
                                 ((STATE) == GPT_MASTERSLAVEMODE_DISABLE))

#define IS_GPT_TRIGGER_SELECTION(SELECTION) (((SELECTION) == GPT_TS_ITR0) || \
                                             ((SELECTION) == GPT_TS_ITR1) || \
                                             ((SELECTION) == GPT_TS_ITR2) || \
                                             ((SELECTION) == GPT_TS_ITR3) || \
                                             ((SELECTION) == GPT_TS_TI1F_ED) || \
                                             ((SELECTION) == GPT_TS_TI1FP1) || \
                                             ((SELECTION) == GPT_TS_TI2FP2) || \
                                             ((SELECTION) == GPT_TS_ETRF))

#define IS_GPT_INTERNAL_TRIGGEREVENT_SELECTION(SELECTION) (((SELECTION) == GPT_TS_ITR0) || \
                                                           ((SELECTION) == GPT_TS_ITR1) || \
                                                           ((SELECTION) == GPT_TS_ITR2) || \
                                                           ((SELECTION) == GPT_TS_ITR3) || \
                                                           ((SELECTION) == GPT_TS_NONE))

#define IS_GPT_TRIGGERPOLARITY(POLARITY)     (((POLARITY) == GPT_TRIGGERPOLARITY_INVERTED   ) || \
                                              ((POLARITY) == GPT_TRIGGERPOLARITY_NONINVERTED) || \
                                              ((POLARITY) == GPT_TRIGGERPOLARITY_RISING     ) || \
                                              ((POLARITY) == GPT_TRIGGERPOLARITY_FALLING    ) || \
                                              ((POLARITY) == GPT_TRIGGERPOLARITY_BOTHEDGE   ))

#define IS_GPT_TRIGGERPRESCALER(PRESCALER)  (((PRESCALER) == GPT_TRIGGERPRESCALER_DIV1) || \
                                             ((PRESCALER) == GPT_TRIGGERPRESCALER_DIV2) || \
                                             ((PRESCALER) == GPT_TRIGGERPRESCALER_DIV4) || \
                                             ((PRESCALER) == GPT_TRIGGERPRESCALER_DIV8))

#define IS_GPT_TRIGGERFILTER(ICFILTER)     ((ICFILTER) <= 0x0FU)

#define IS_GPT_TI1SELECTION(TI1SELECTION)   (((TI1SELECTION) == GPT_TI1SELECTION_CH1) || \
                                             ((TI1SELECTION) == GPT_TI1SELECTION_XORCOMBINATION))

#define IS_GPT_DMA_BASE(BASE) (((BASE) == GPT_DMABASE_CR1) || \
                               ((BASE) == GPT_DMABASE_CR2) || \
                               ((BASE) == GPT_DMABASE_SMCR) || \
                               ((BASE) == GPT_DMABASE_DIER) || \
                               ((BASE) == GPT_DMABASE_SR) || \
                               ((BASE) == GPT_DMABASE_EGR) || \
                               ((BASE) == GPT_DMABASE_CCMR1) || \
                               ((BASE) == GPT_DMABASE_CCMR2) || \
                               ((BASE) == GPT_DMABASE_CCER) || \
                               ((BASE) == GPT_DMABASE_CNT) || \
                               ((BASE) == GPT_DMABASE_PSC) || \
                               ((BASE) == GPT_DMABASE_ARR) || \
                               ((BASE) == GPT_DMABASE_RCR) || \
                               ((BASE) == GPT_DMABASE_CCR1) || \
                               ((BASE) == GPT_DMABASE_CCR2) || \
                               ((BASE) == GPT_DMABASE_CCR3) || \
                               ((BASE) == GPT_DMABASE_CCR4) || \
                               ((BASE) == GPT_DMABASE_BDTR) || \
                               ((BASE) == GPT_DMABASE_DCR) || \
                               ((BASE) == GPT_DMABASE_OR))

#define IS_GPT_DMA_LENGTH(LENGTH) (((LENGTH) == GPT_DMABURSTLENGTH_1TRANSFER) || \
                                   ((LENGTH) == GPT_DMABURSTLENGTH_2TRANSFERS) || \
                                   ((LENGTH) == GPT_DMABURSTLENGTH_3TRANSFERS) || \
                                   ((LENGTH) == GPT_DMABURSTLENGTH_4TRANSFERS) || \
                                   ((LENGTH) == GPT_DMABURSTLENGTH_5TRANSFERS) || \
                                   ((LENGTH) == GPT_DMABURSTLENGTH_6TRANSFERS) || \
                                   ((LENGTH) == GPT_DMABURSTLENGTH_7TRANSFERS) || \
                                   ((LENGTH) == GPT_DMABURSTLENGTH_8TRANSFERS) || \
                                   ((LENGTH) == GPT_DMABURSTLENGTH_9TRANSFERS) || \
                                   ((LENGTH) == GPT_DMABURSTLENGTH_10TRANSFERS) || \
                                   ((LENGTH) == GPT_DMABURSTLENGTH_11TRANSFERS) || \
                                   ((LENGTH) == GPT_DMABURSTLENGTH_12TRANSFERS) || \
                                   ((LENGTH) == GPT_DMABURSTLENGTH_13TRANSFERS) || \
                                   ((LENGTH) == GPT_DMABURSTLENGTH_14TRANSFERS) || \
                                   ((LENGTH) == GPT_DMABURSTLENGTH_15TRANSFERS) || \
                                   ((LENGTH) == GPT_DMABURSTLENGTH_16TRANSFERS) || \
                                   ((LENGTH) == GPT_DMABURSTLENGTH_17TRANSFERS) || \
                                   ((LENGTH) == GPT_DMABURSTLENGTH_18TRANSFERS))

#define IS_GPT_IC_FILTER(ICFILTER) ((ICFILTER) <= 0x0FU)
/**
  * @}
  */

/** @defgroup GPT_Mask_Definitions TIM Mask Definition
  * @{
  */
/* The counter of a timer instance is disabled only if all the CCx and CCxN
   channels have been disabled */
#define GPT_CCER_CCxE_MASK  ((uint32_t)(GPT_CCER_CC1E | GPT_CCER_CC2E | GPT_CCER_CC3E | GPT_CCER_CC4E))
#define GPT_CCER_CCxNE_MASK ((uint32_t)(GPT_CCER_CC1NE | GPT_CCER_CC2NE | GPT_CCER_CC3NE))

/**
  * @}
  */

/**
  * @}
  */

/* Private functions ---------------------------------------------------------*/
/** @defgroup GPT_Private_Functions TIM Private Functions
  * @{
  */
void GPT_Base_SetConfig(GPT_TypeDef *TIMx, GPT_Base_InitTypeDef *Structure);
void GPT_TI1_SetConfig(GPT_TypeDef *TIMx, uint32_t GPT_ICPolarity, uint32_t GPT_ICSelection, uint32_t GPT_ICFilter);
void GPT_OC2_SetConfig(GPT_TypeDef *TIMx, GPT_OC_InitTypeDef *OC_Config);
void GPT_DMADelayPulseCplt(DMA_HandleTypeDef *hdma);
void GPT_DMAError(DMA_HandleTypeDef *hdma);
void GPT_DMACaptureCplt(DMA_HandleTypeDef *hdma);
void GPT_DMACaptureHalfCplt(DMA_HandleTypeDef *hdma);
void GPT_CCxChannelCmd(GPT_TypeDef *TIMx, uint32_t Channel, uint32_t ChannelState);
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef HAL_ATIM_MODULE_ENABLED
#define GPT_CCER_CC1NE            ATIM_CCER_CC1NE_Msk                           /*!<Capture/Compare 1 Complementary output enable */
#define GPT_CCER_CC2NE            ATIM_CCER_CC2NE_Msk                           /*!<Capture/Compare 2 Complementary output enable */
#define GPT_CCER_CC3NE            ATIM_CCER_CC3NE_Msk                           /*!<Capture/Compare 3 Complementary output enable */
#else
#define GPT_CCER_CC1NE_Pos        (2U)
#define GPT_CCER_CC1NE_Msk        (0x1U << GPT_CCER_CC1NE_Pos)                 /*!< 0x00000004 */
#define GPT_CCER_CC1NE            GPT_CCER_CC1NE_Msk                           /*!<Capture/Compare 1 Complementary output enable */
#define GPT_CCER_CC2NE_Pos        (6U)
#define GPT_CCER_CC2NE_Msk        (0x1U << GPT_CCER_CC2NE_Pos)                 /*!< 0x00000040 */
#define GPT_CCER_CC2NE            GPT_CCER_CC2NE_Msk                           /*!<Capture/Compare 2 Complementary output enable */
#define GPT_CCER_CC3NE_Pos        (10U)
#define GPT_CCER_CC3NE_Msk        (0x1U << GPT_CCER_CC3NE_Pos)                 /*!< 0x00000400 */
#define GPT_CCER_CC3NE            GPT_CCER_CC3NE_Msk                           /*!<Capture/Compare 3 Complementary output enable */
#endif


#define GPT_DIER_COMIE_Pos        (5U)
#define GPT_DIER_COMIE_Msk        (0x1U << GPT_DIER_COMIE_Pos)                 /*!< 0x00000020 */
#define GPT_DIER_COMIE            GPT_DIER_COMIE_Msk                           /*!<COM interrupt enable */
#define GPT_DIER_BIE_Pos          (7U)
#define GPT_DIER_BIE_Msk          (0x1U << GPT_DIER_BIE_Pos)                   /*!< 0x00000080 */
#define GPT_DIER_BIE              GPT_DIER_BIE_Msk                             /*!<Break interrupt enable */
#define GPT_DIER_COMDE_Pos        (13U)
#define GPT_DIER_COMDE_Msk        (0x1U << GPT_DIER_COMDE_Pos)                 /*!< 0x00002000 */
#define GPT_DIER_COMDE            GPT_DIER_COMDE_Msk                           /*!<COM DMA request enable               */

#define GPT_CR1_CKD_0             (0x1U << GPT_CR1_CKD_Pos)                    /*!< 0x00000100 */
#define GPT_CR1_CKD_1             (0x2U << GPT_CR1_CKD_Pos)                    /*!< 0x00000200 */

#define GPT_CR2_CCUS_Pos          (2U)
#define GPT_CR2_CCUS_Msk          (0x1U << GPT_CR2_CCUS_Pos)                   /*!< 0x00000004 */
#define GPT_CR2_CCUS              GPT_CR2_CCUS_Msk                             /*!<Capture/Compare Control Update Selection */
#define GPT_CR2_OIS1_Pos          (8U)
#define GPT_CR2_OIS1_Msk          (0x1U << GPT_CR2_OIS1_Pos)                   /*!< 0x00000100 */
#define GPT_CR2_OIS1              GPT_CR2_OIS1_Msk                             /*!<Output Idle state 1 (OC1 output)  */
#define GPT_CR2_OIS1N_Pos         (9U)
#define GPT_CR2_OIS1N_Msk         (0x1U << GPT_CR2_OIS1N_Pos)                  /*!< 0x00000200 */
#define GPT_CR2_OIS1N             GPT_CR2_OIS1N_Msk                            /*!<Output Idle state 1 (OC1N output) */
#define GPT_CR2_OIS2_Pos          (10U)
#define GPT_CR2_OIS2_Msk          (0x1U << GPT_CR2_OIS2_Pos)                   /*!< 0x00000400 */
#define GPT_CR2_OIS2              GPT_CR2_OIS2_Msk                             /*!<Output Idle state 2 (OC2 output)  */
#define GPT_CR2_OIS2N_Pos         (11U)
#define GPT_CR2_OIS2N_Msk         (0x1U << GPT_CR2_OIS2N_Pos)                  /*!< 0x00000800 */
#define GPT_CR2_OIS2N             GPT_CR2_OIS2N_Msk                            /*!<Output Idle state 2 (OC2N output) */
#define GPT_CR2_OIS3_Pos          (12U)
#define GPT_CR2_OIS3_Msk          (0x1U << GPT_CR2_OIS3_Pos)                   /*!< 0x00001000 */
#define GPT_CR2_OIS3              GPT_CR2_OIS3_Msk                             /*!<Output Idle state 3 (OC3 output) */
#define GPT_CR2_OIS3N_Pos         (13U)
#define GPT_CR2_OIS3N_Msk         (0x1U << GPT_CR2_OIS3N_Pos)                  /*!< 0x00002000 */
#define GPT_CR2_OIS3N             GPT_CR2_OIS3N_Msk                            /*!<Output Idle state 3 (OC3N output) */
#define GPT_CR2_OIS4_Pos          (14U)
#define GPT_CR2_OIS4_Msk          (0x1U << GPT_CR2_OIS4_Pos)                   /*!< 0x00004000 */
#define GPT_CR2_OIS4              GPT_CR2_OIS4_Msk                             /*!<Output Idle state 4 (OC4 output) */

#define GPT_SR_COMIF_Pos          (5U)
#define GPT_SR_COMIF_Msk          (0x1U << GPT_SR_COMIF_Pos)                   /*!< 0x00000020 */
#define GPT_SR_COMIF              GPT_SR_COMIF_Msk                             /*!<COM interrupt Flag */
#define GPT_SR_BIF_Pos            (7U)
#define GPT_SR_BIF_Msk            (0x1U << GPT_SR_BIF_Pos)                     /*!< 0x00000080 */
#define GPT_SR_BIF                GPT_SR_BIF_Msk                               /*!<Break interrupt Flag               */

#define GPT_SMCR_ETPS_0           (0x1U << GPT_SMCR_ETPS_Pos)                  /*!< 0x00001000 */
#define GPT_SMCR_ETPS_1           (0x2U << GPT_SMCR_ETPS_Pos)                  /*!< 0x00002000 */
#define GPT_SMCR_TS_0             (0x1U << GPT_SMCR_TS_Pos)                    /*!< 0x00000010 */
#define GPT_SMCR_TS_1             (0x2U << GPT_SMCR_TS_Pos)                    /*!< 0x00000020 */
#define GPT_SMCR_TS_2             (0x4U << GPT_SMCR_TS_Pos)                    /*!< 0x00000040 */

#define GPT_CCMR1_CC1S_0          (0x1U << GPT_CCMR1_CC1S_Pos)                 /*!< 0x00000001 */
#define GPT_CCMR1_CC1S_1          (0x2U << GPT_CCMR1_CC1S_Pos)                 /*!< 0x00000002 */
#define GPT_CCMR1_OC1M_0          (0x1U << GPT_CCMR1_OC1M_Pos)                 /*!< 0x00000010 */
#define GPT_CCMR1_OC1M_1          (0x2U << GPT_CCMR1_OC1M_Pos)                 /*!< 0x00000020 */
#define GPT_CCMR1_OC1M_2          (0x4U << GPT_CCMR1_OC1M_Pos)                 /*!< 0x00000040 */


/************ TIM Instances : at least 1 capture/compare channels *************/
#ifdef hwp_gptim3
#define IS_GPT_CC1_INSTANCE(inst)  (((inst) == hwp_gptim1) || \
                                    ((inst) == hwp_gptim2) || \
                                    ((inst) == hwp_gptim3) || \
                                    ((inst) == hwp_gptim4) || \
                                    ((inst) == hwp_gptim5) || \
                                    IS_ATIM_INSTANCE(inst)    \
                                    )
#else
#define IS_GPT_CC1_INSTANCE(inst)  (((inst) == hwp_gptim1) || \
                                    ((inst) == hwp_gptim2) || \
                                    IS_ATIM_INSTANCE(inst)    \
                                    )
#endif

/************ TIM Instances : at least 2 capture/compare channels *************/
#define IS_GPT_CC2_INSTANCE(inst)  IS_GPT_CC1_INSTANCE(inst)

/************ TIM Instances : at least 3 capture/compare channels *************/
#define IS_GPT_CC3_INSTANCE(inst)  IS_GPT_CC1_INSTANCE(inst)

/************ TIM Instances : at least 4 capture/compare channels *************/

#define IS_GPT_CC4_INSTANCE(__INSTANCE__) IS_GPT_CC1_INSTANCE(__INSTANCE__)


/******************** TIM Instances : Advanced-control timers *****************/
#ifdef hwp_atim2
#define IS_GPT_ADVANCED_INSTANCE(inst) (((inst) == (GPT_TypeDef*)hwp_atim1) || \
                                        ((inst) == (GPT_TypeDef*)hwp_atim2))


#elif defined(hwp_atim1)
#define IS_GPT_ADVANCED_INSTANCE(inst) ((inst) == (GPT_TypeDef*)hwp_atim1)
#else
#define IS_GPT_ADVANCED_INSTANCE(inst) 0
#endif

/****************** TIM Instances : All supported instances *******************/
#ifdef hwp_atim2
#define IS_ATIM_INSTANCE(__INSTANCE__) (((__INSTANCE__) ==(GPT_TypeDef *)hwp_atim1) \
                                         || ((__INSTANCE__) == (GPT_TypeDef *)hwp_atim2)  \
                                         )
#elif defined(hwp_atim1)
#define IS_ATIM_INSTANCE(__INSTANCE__) ((__INSTANCE__) ==(GPT_TypeDef *)hwp_atim1)
#else
#define IS_ATIM_INSTANCE(__INSTANCE__) 0
#endif

#ifdef hwp_gptim3
#define IS_GPT_INSTANCE(__INSTANCE__)   (((__INSTANCE__) == hwp_gptim1)  \
                                         || ((__INSTANCE__) == hwp_gptim2) \
                                         || ((__INSTANCE__) == hwp_gptim3)  \
                                         || ((__INSTANCE__) == hwp_gptim4)  \
                                         || ((__INSTANCE__) == hwp_gptim5)  \
                                         || ((__INSTANCE__) == (GPT_TypeDef *)hwp_btim1)  \
                                         || ((__INSTANCE__) == (GPT_TypeDef *)hwp_btim2)  \
                                         || ((__INSTANCE__) == (GPT_TypeDef *)hwp_btim3)  \
                                         || ((__INSTANCE__) == (GPT_TypeDef *)hwp_btim4) \
                                         || IS_ATIM_INSTANCE(__INSTANCE__) \
                                         )

#define IS_GPT_DMA_INSTANCE(__INSTANCE__) (((__INSTANCE__) == hwp_gptim1)  \
                                             || ((__INSTANCE__) == hwp_gptim2)  \
                                             || ((__INSTANCE__) == hwp_gptim3)  \
                                             || ((__INSTANCE__) == hwp_gptim4)  \
                                             || ((__INSTANCE__) == hwp_gptim5)  \
                                             || ((__INSTANCE__) == (GPT_TypeDef *)hwp_btim1) \
                                             || ((__INSTANCE__) == (GPT_TypeDef *)hwp_btim2) \
                                             || ((__INSTANCE__) == (GPT_TypeDef *)hwp_btim3) \
                                             || ((__INSTANCE__) == (GPT_TypeDef *)hwp_btim4) \
                                             || IS_ATIM_DMA_INSTANCE(__INSTANCE__) \
                                             )


#else
#define IS_GPT_INSTANCE(__INSTANCE__)   (((__INSTANCE__) == hwp_gptim1)  \
                                         || ((__INSTANCE__) == hwp_gptim2) \
                                         || ((__INSTANCE__) == (GPT_TypeDef *)hwp_btim1)  \
                                         || ((__INSTANCE__) == (GPT_TypeDef *)hwp_btim2)  \
                                         || ((__INSTANCE__) == (GPT_TypeDef *)hwp_btim3)  \
                                         || ((__INSTANCE__) == (GPT_TypeDef *)hwp_btim4) \
                                         || IS_ATIM_INSTANCE(__INSTANCE__) \
                                         )

#define IS_GPT_DMA_INSTANCE(__INSTANCE__) (((__INSTANCE__) == hwp_gptim1)  \
                                         || ((__INSTANCE__) == hwp_gptim2)  \
                                         || ((__INSTANCE__) == (GPT_TypeDef *)hwp_btim1) \
                                         || ((__INSTANCE__) == (GPT_TypeDef *)hwp_btim2) \
                                         || ((__INSTANCE__) == (GPT_TypeDef *)hwp_btim3) \
                                         || ((__INSTANCE__) == (GPT_TypeDef *)hwp_btim4) \
                                         || IS_ATIM_DMA_INSTANCE(__INSTANCE__) \
                                         )
#endif

/************ TIM Instances : DMA requests generation (COMDE) *****************/
#ifdef hwp_atim2
#define IS_ATIM_DMA_INSTANCE(__INSTANCE__) (((__INSTANCE__) ==(GPT_TypeDef *)hwp_atim1) \
                                         || ((__INSTANCE__) == (GPT_TypeDef *)hwp_atim2)  \
                                         )
#elif defined(hwp_atim1)
#define IS_ATIM_DMA_INSTANCE(__INSTANCE__) ((__INSTANCE__) ==(GPT_TypeDef *)hwp_atim1)
#else
#define IS_ATIM_DMA_INSTANCE(__INSTANCE__) 0
#endif

#ifdef hwp_atim2
#define IS_ATIM_CCX_INSTANCE(INSTANCE, chn) \
    ((((INSTANCE) == (GPT_TypeDef *)hwp_atim1) &&         \
     (((chn) == TIM_CHANNEL_1) ||          \
      ((chn) == TIM_CHANNEL_2) ||          \
      ((chn) == TIM_CHANNEL_3) ||          \
      ((chn) == TIM_CHANNEL_4) ||          \
      ((chn) == TIM_CHANNEL_5) ||          \
      ((chn) == TIM_CHANNEL_6)))           \
    ||                                     \
    (((INSTANCE) == (GPT_TypeDef *)hwp_atim2) &&          \
     (((chn) == TIM_CHANNEL_1) ||          \
      ((chn) == TIM_CHANNEL_2) ||          \
      ((chn) == TIM_CHANNEL_3) ||          \
      ((chn) == TIM_CHANNEL_4) ||          \
      ((chn) == TIM_CHANNEL_5) ||          \
      ((chn) == TIM_CHANNEL_6))))
#elif defined(hwp_atim1)
#define IS_ATIM_CCX_INSTANCE(INSTANCE, chn) \
     (((INSTANCE) == (GPT_TypeDef *)hwp_atim1) &&         \
     (((chn) == TIM_CHANNEL_1) ||          \
      ((chn) == TIM_CHANNEL_2) ||          \
      ((chn) == TIM_CHANNEL_3) ||          \
      ((chn) == TIM_CHANNEL_4) ||          \
      ((chn) == TIM_CHANNEL_5) ||          \
      ((chn) == TIM_CHANNEL_6)))
#else
#define IS_ATIM_CCX_INSTANCE(INSTANCE, chn) 0
#endif

/******************* TIM Instances : output(s) available **********************/
#ifdef hwp_gptim3
#define IS_GPT_CCX_INSTANCE(INSTANCE, chn) \
    ((((INSTANCE) == hwp_gptim1) &&        \
     (((chn) == TIM_CHANNEL_1) ||          \
      ((chn) == TIM_CHANNEL_2) ||          \
      ((chn) == TIM_CHANNEL_3) ||          \
      ((chn) == TIM_CHANNEL_4)))           \
    ||                                     \
    (((INSTANCE) == hwp_gptim2) &&         \
     (((chn) == TIM_CHANNEL_1) ||          \
      ((chn) == TIM_CHANNEL_2) ||          \
      ((chn) == TIM_CHANNEL_3) ||          \
      ((chn) == TIM_CHANNEL_4)))           \
    ||                                     \
    (((INSTANCE) == hwp_gptim3) &&         \
     (((chn) == TIM_CHANNEL_1) ||          \
      ((chn) == TIM_CHANNEL_2) ||          \
      ((chn) == TIM_CHANNEL_3) ||          \
      ((chn) == TIM_CHANNEL_4)))           \
    ||                                     \
    (((INSTANCE) == hwp_gptim4) &&         \
     (((chn) == TIM_CHANNEL_1) ||          \
      ((chn) == TIM_CHANNEL_2) ||          \
      ((chn) == TIM_CHANNEL_3) ||          \
      ((chn) == TIM_CHANNEL_4)))           \
    ||                                     \
    (((INSTANCE) == hwp_gptim5) &&         \
     (((chn) == TIM_CHANNEL_1) ||          \
      ((chn) == TIM_CHANNEL_2) ||          \
      ((chn) == TIM_CHANNEL_3) ||          \
      ((chn) == TIM_CHANNEL_4)))           \
    ||                                     \
    IS_ATIM_CCX_INSTANCE(INSTANCE, chn)    \
    )
#else

#define IS_GPT_CCX_INSTANCE(INSTANCE, chn) \
    ((((INSTANCE) == hwp_gptim1) &&        \
     (((chn) == TIM_CHANNEL_1) ||          \
      ((chn) == TIM_CHANNEL_2) ||          \
      ((chn) == TIM_CHANNEL_3) ||          \
      ((chn) == TIM_CHANNEL_4)))           \
    ||                                     \
    (((INSTANCE) == hwp_gptim2) &&         \
     (((chn) == TIM_CHANNEL_1) ||          \
      ((chn) == TIM_CHANNEL_2) ||          \
      ((chn) == TIM_CHANNEL_3) ||          \
      ((chn) == TIM_CHANNEL_4)))           \
    ||                                     \
    IS_ATIM_CCX_INSTANCE(INSTANCE, chn)    \
    )
#endif

/*********** TIM Instances : Slave mode available (TIMx_SMCR available )*******/
#ifdef hwp_gptim3
#define IS_GPT_SLAVE_INSTANCE(__INSTANCE__) (((__INSTANCE__) == hwp_gptim1) || \
                                         ((__INSTANCE__) == hwp_gptim2) || \
                                         ((__INSTANCE__) == hwp_gptim3) || \
                                         ((__INSTANCE__) == hwp_gptim4) || \
                                         ((__INSTANCE__) == hwp_gptim5) || \
                                         ((__INSTANCE__) == (GPT_TypeDef *)hwp_btim1) || \
                                         ((__INSTANCE__) == (GPT_TypeDef *)hwp_btim2) || \
                                         ((__INSTANCE__) == (GPT_TypeDef *)hwp_btim3) || \
                                         ((__INSTANCE__) == (GPT_TypeDef *)hwp_btim4))
#else
#define IS_GPT_SLAVE_INSTANCE(__INSTANCE__) (((__INSTANCE__) == hwp_gptim1) || \
                                         ((__INSTANCE__) == hwp_gptim2) || \
                                         ((__INSTANCE__) == (GPT_TypeDef *)hwp_btim1) || \
                                         ((__INSTANCE__) == (GPT_TypeDef *)hwp_btim2) || \
                                         ((__INSTANCE__) == (GPT_TypeDef *)hwp_btim3) || \
                                         ((__INSTANCE__) == (GPT_TypeDef *)hwp_btim4))
#endif


/******************* TIM Instances : Timer input XOR function *****************/
#ifdef hwp_gptim3
#define IS_GPT_XOR_INSTANCE(__INSTANCE__) (((__INSTANCE__) == hwp_gptim1) || \
                                         ((__INSTANCE__) == hwp_gptim2) || \
                                         ((__INSTANCE__) == hwp_gptim3) || \
                                         ((__INSTANCE__) == hwp_gptim4) || \
                                         ((__INSTANCE__) == hwp_gptim5) || \
                                         ((__INSTANCE__) == (GPT_TypeDef *)hwp_btim1) || \
                                         ((__INSTANCE__) == (GPT_TypeDef *)hwp_btim2) || \
                                         ((__INSTANCE__) == (GPT_TypeDef *)hwp_btim3) || \
                                         ((__INSTANCE__) == (GPT_TypeDef *)hwp_btim4))
#else
#define IS_GPT_XOR_INSTANCE(__INSTANCE__) (((__INSTANCE__) == hwp_gptim1) || \
                                         ((__INSTANCE__) == hwp_gptim2) || \
                                         ((__INSTANCE__) == (GPT_TypeDef *)hwp_btim1) || \
                                         ((__INSTANCE__) == (GPT_TypeDef *)hwp_btim2) || \
                                         ((__INSTANCE__) == (GPT_TypeDef *)hwp_btim3) || \
                                         ((__INSTANCE__) == (GPT_TypeDef *)hwp_btim4))
#endif

/***************** TIM Instances : external trigger input availabe ************/
#ifdef hwp_gptim3
#define IS_GPT_ETR_INSTANCE(__INSTANCE__) (((__INSTANCE__) == hwp_gptim1) || \
                                         ((__INSTANCE__) == hwp_gptim2) || \
                                         ((__INSTANCE__) == hwp_gptim3) || \
                                         ((__INSTANCE__) == hwp_gptim4) || \
                                         ((__INSTANCE__) == hwp_gptim5))
#else
#define IS_GPT_ETR_INSTANCE(__INSTANCE__) (((__INSTANCE__) == hwp_gptim1) || \
                                         ((__INSTANCE__) == hwp_gptim2))
#endif

/******************** TIM Instances : DMA burst feature ***********************/
#ifdef hwp_gptim3
#define IS_GPT_DMABURST_INSTANCE(__INSTANCE__)  (((__INSTANCE__) == hwp_gptim1) || \
                                         ((__INSTANCE__) == hwp_gptim2) || \
                                         ((__INSTANCE__) == hwp_gptim3) || \
                                         ((__INSTANCE__) == hwp_gptim4) || \
                                         ((__INSTANCE__) == hwp_gptim5) || \
                                         ((__INSTANCE__) == (GPT_TypeDef *)hwp_btim1) || \
                                         ((__INSTANCE__) == (GPT_TypeDef *)hwp_btim2) || \
                                         ((__INSTANCE__) == (GPT_TypeDef *)hwp_btim3) || \
                                         ((__INSTANCE__) == (GPT_TypeDef *)hwp_btim4))
#else
#define IS_GPT_DMABURST_INSTANCE(__INSTANCE__)  (((__INSTANCE__) == hwp_gptim1) || \
                                         ((__INSTANCE__) == hwp_gptim2) || \
                                         ((__INSTANCE__) == (GPT_TypeDef *)hwp_btim1) || \
                                         ((__INSTANCE__) == (GPT_TypeDef *)hwp_btim2) || \
                                         ((__INSTANCE__) == (GPT_TypeDef *)hwp_btim3) || \
                                         ((__INSTANCE__) == (GPT_TypeDef *)hwp_btim4))
#endif

/************ TIM Instances : DMA requests generation (CCxDE) *****************/
#ifdef hwp_gptim3
#define IS_GPT_DMA_CC_INSTANCE(__INSTANCE__) (((__INSTANCE__) == hwp_gptim1) || \
                                         ((__INSTANCE__) == hwp_gptim2) || \
                                         ((__INSTANCE__) == hwp_gptim3) || \
                                         ((__INSTANCE__) == hwp_gptim4) || \
                                         ((__INSTANCE__) == hwp_gptim5))

#else
#define IS_GPT_DMA_CC_INSTANCE(__INSTANCE__) (((__INSTANCE__) == hwp_gptim1) || \
                                         ((__INSTANCE__) == hwp_gptim2))

#endif


/************************* For ATIM **********************************/

#define GPT_SMCR_SMS_0 (1<<GPT_SMCR_SMS_Pos)
#define GPT_SMCR_SMS_1 (2<<GPT_SMCR_SMS_Pos)
#define GPT_SMCR_SMS_2 (4<<GPT_SMCR_SMS_Pos)

#define IS_GPT_SLAVEMODE_TRIGGER_ENABLED(__TRIGGER__) ((__TRIGGER__) == GPT_SLAVEMODE_TRIGGER)


#ifdef __cplusplus
}
#endif

#endif /* __BF0_HAL_GPT_H */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
