/*
 * SPDX-FileCopyrightText: 2016 STMicroelectronics
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: BSD-3-Clause AND Apache-2.0
 */

#ifndef __BF0_HAL_PWM_H
#define __BF0_HAL_PWM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal_def.h"

/** @addtogroup BF0_HAL_Driver
 * @{
 */

/** @addtogroup PWM Hardware Timer
 * @{
 */

/* Exported types ------------------------------------------------------------*/
/** @defgroup PWM_Exported_Types PWM Exported Types
 * @{
 */

/**
 * @brief  PWM Time base Configuration Structure definition
 */
typedef struct
{
    uint32_t Prescaler;         /*!< Specifies the prescaler value used to divide the TIM clock.
                                   This parameter can be a number between Min_Data = 0x0000U and Max_Data = 0xFFFFU */

    uint32_t CounterMode;       /*!< Specifies the counter mode.
                                   This parameter can be a value of @ref PWM_Counter_Mode */

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
} PWM_InitTypeDef;

/**
 * @brief  PWM Output Compare Configuration Structure definition
 */

typedef struct
{
    uint32_t OCMode;        /*!< Specifies the TIM mode.
                               This parameter can be a value of @ref PWM_Output_Compare_and_PWM_modes */

    uint32_t Pulse;         /*!< Specifies the pulse value to be loaded into the Capture Compare Register.
                               This parameter can be a number between Min_Data = 0x0000U and Max_Data = 0xFFFFU */

    uint32_t OCPolarity;    /*!< Specifies the output polarity.
                               This parameter can be a value of @ref PWM_Output_Compare_Polarity */

    uint32_t OCNPolarity;   /*!< Specifies the complementary output polarity.
                               This parameter can be a value of @ref PWM_Output_Compare_N_Polarity
                               @note This parameter is valid only for TIM1 and TIM8. */

    uint32_t OCIdleState;   /*!< Specifies the TIM Output Compare pin state during Idle state.
                               This parameter can be a value of @ref PWM_Output_Compare_Idle_State
                               @note This parameter is valid only for TIM1 and TIM8. */

    uint32_t OCNIdleState;  /*!< Specifies the TIM Output Compare pin state during Idle state.
                               This parameter can be a value of @ref PWM_Output_Compare_N_Idle_State
                               @note This parameter is valid only for TIM1 and TIM8. */
} PWM_OC_InitTypeDef;

/**
 * @brief  TIM One Pulse Mode Configuration Structure definition
 */
typedef struct
{
    uint32_t OCMode;        /*!< Specifies the TIM mode.
                               This parameter can be a value of @ref PWM_Output_Compare_and_PWM_modes */

    uint32_t Pulse;         /*!< Specifies the pulse value to be loaded into the Capture Compare Register.
                               This parameter can be a number between Min_Data = 0x0000U and Max_Data = 0xFFFFU */

    uint32_t OCPolarity;    /*!< Specifies the output polarity.
                               This parameter can be a value of @ref PWM_Output_Compare_Polarity */

    uint32_t OCIdleState;   /*!< Specifies the TIM Output Compare pin state during Idle state.
                               This parameter can be a value of @ref PWM_Output_Compare_Idle_State
                               @note This parameter is valid only for TIM1 and TIM8. */

} PWM_OnePulse_InitTypeDef;



/**
 * @brief  HAL State structures definition
 */
typedef enum
{
    HAL_PWM_STATE_RESET             = 0x00U,    /*!< Peripheral not yet initialized or disabled  */
    HAL_PWM_STATE_READY             = 0x01U,    /*!< Peripheral Initialized and ready for use    */
    HAL_PWM_STATE_BUSY              = 0x02U,    /*!< An internal process is ongoing              */
    HAL_PWM_STATE_TIMEOUT           = 0x03U,    /*!< Timeout state                               */
    HAL_PWM_STATE_ERROR             = 0x04U     /*!< Reception process is ongoing                */
} HAL_PWM_StateTypeDef;

/**
 * @brief  HAL Active channel structures definition
 */
typedef enum
{
    HAL_PWM_ACTIVE_CHANNEL_1        = 0x01U,    /*!< The active channel is 1     */
    HAL_PWM_ACTIVE_CHANNEL_2        = 0x02U,    /*!< The active channel is 2     */
    HAL_PWM_ACTIVE_CHANNEL_3        = 0x04U,    /*!< The active channel is 3     */
    HAL_PWM_ACTIVE_CHANNEL_4        = 0x08U,    /*!< The active channel is 4     */
    HAL_PWM_ACTIVE_CHANNEL_CLEARED  = 0x00U     /*!< All active channels cleared */
} HAL_PWM_ActiveChannel;


/**
 * @brief  TIM Channel States definition
 */
typedef enum
{
    HAL_PWM_CHANNEL_STATE_RESET             = 0x00U,    /*!< TIM Channel initial state                         */
    HAL_PWM_CHANNEL_STATE_READY             = 0x01U,    /*!< TIM Channel ready for use                         */
    HAL_PWM_CHANNEL_STATE_BUSY              = 0x02U,    /*!< An internal process is ongoing on the TIM channel */
} HAL_PWM_ChannelStateTypeDef;

/**
 * @brief  PWM PWM Handle Structure definition
 */
typedef struct
{
    PWM_TypeDef          *Instance;            /*!< Register base address           */
    PWM_InitTypeDef             Init;          /*!< PWM required parameters */
    HAL_PWM_ActiveChannel       Channel;       /*!< Active channel                    */
    DMA_HandleTypeDef           *hdma[7];      /*!< DMA Handlers array
                                             This array is accessed by a @ref DMA_Handle_index */
    HAL_LockTypeDef             Lock;          /*!< Locking object                    */
    uint8_t                     core;          /*!< Clock source from which core*/
    __IO HAL_PWM_StateTypeDef   State;         /*!< TIM operation state               */
} PWM_HandleTypeDef;


/**
 * @}
 */

/* Exported constants --------------------------------------------------------*/

#define PWM_CR_OCM_0          (0x1U << PWM_CR1_OCM_Pos)                 /*!< 0x00000010 */
#define PWM_CR_OCM_1          (0x2U << PWM_CR1_OCM_Pos)                 /*!< 0x00000020 */
#define PWM_CR_OCM_2          (0x4U << PWM_CR1_OCM_Pos)                 /*!< 0x00000040 */


/** @defgroup PWM_Output_Compare_and_PWM_modes  PWM Output Compare and PWM modes
 * @{
 */
#define PWM_OCMODE_TIMING                   0x00000000U
#define PWM_OCMODE_ACTIVE                   (PWM_CR_OCM_0)
#define PWM_OCMODE_INACTIVE                 (PWM_CR_OCM_1)
#define PWM_OCMODE_TOGGLE                   (PWM_CR_OCM_0 | PWM_CR_OCM_1)
#define PWM_OCMODE_PWM1                     (PWM_CR_OCM_1 | PWM_CR_OCM_2)
#define PWM_OCMODE_PWM2                     (PWM_CR_OCM_0 | PWM_CR_OCM_1 | PWM_CR_OCM_2)
#define PWM_OCMODE_FORCED_ACTIVE            (PWM_CR_OCM_0 | PWM_CR_OCM_2)
#define PWM_OCMODE_FORCED_INACTIVE          (PWM_CR_OCM_2)
/**
 * @}
 */

/** @defgroup PWM_Channel  PWM Channel
 * @{
 */
#define PWM_CHANNEL_1                      0x00000000U
#define PWM_CHANNEL_2                      0x00000001U
#define PWM_CHANNEL_3                      0x00000002U
#define PWM_CHANNEL_4                      0x00000003U
#define PWM_CHANNEL_5                      0x00000004U                          /*!< Compare channel 5 identifier              */
/**
 * @}
 */



/** @defgroup PWM_Output_Compare_Polarity  PWM Output Compare Polarity
 * @{
 */
#define PWM_OCPOLARITY_HIGH                0x00000000U
#define PWM_OCPOLARITY_LOW                 (PWM_CR1_CCP)
/**
 * @}
 */


/** @defgroup PWM_One_Pulse_Mode PWM One Pulse Mode
 * @{
 */
#define PWM_OPMODE_SINGLE                  (PWM_CR1_OPM)
#define PWM_OPMODE_REPETITIVE              0x00000000U
/**
 * @}
 */



/** @defgroup PWM_Interrupt_definition  PWM Interrupt definition
 * @{
 */
#define PWM_IT_UPDATE            (PWM_CR1_UIE)
#define PWM_IT_CC                (PWM_CR1_CCIE)
/**
 * @}
 */

/** @defgroup PWM_Event_Source  TIM Event Source
 * @{
 */
#define PWM_EVENTSOURCE_UPDATE1             PWM_EGR_UG1
#define PWM_EVENTSOURCE_UPDATE2             PWM_EGR_UG2
#define PWM_EVENTSOURCE_UPDATE3             PWM_EGR_UG3
#define PWM_EVENTSOURCE_UPDATE4             PWM_EGR_UG4
#define PWM_EVENTSOURCE_CC1                 PWM_EGR_CC1G
#define PWM_EVENTSOURCE_CC2                 PWM_EGR_CC2G
#define PWM_EVENTSOURCE_CC3                 PWM_EGR_CC3G
#define PWM_EVENTSOURCE_CC4                 PWM_EGR_CC4G
#define PWM_EVENTSOURCE_COM                 PWM_EGR_COMG
#define PWM_EVENTSOURCE_TRIGGER             PWM_EGR_TG

/**
 * @}
 */

/** @defgroup PWM_Flag_definition  TIM Flag definition
 * @{
 */
#define PWM_FLAG_UPDATE1                   (PWM_SR_UIF1)
#define PWM_FLAG_UPDATE2                   (PWM_SR_UIF2)
#define PWM_FLAG_UPDATE3                   (PWM_SR_UIF3)
#define PWM_FLAG_UPDATE4                   (PWM_SR_UIF4)
#define PWM_FLAG_CC1                       (PWM_SR_CCIF1)
#define PWM_FLAG_CC2                       (PWM_SR_CCIF2)
#define PWM_FLAG_CC3                       (PWM_SR_CCIF3)
#define PWM_FLAG_CC4                       (PWM_SR_CCIF4)

/**
 * @}
 */

/** @defgroup PWM_Clock_Source  TIM Clock Source
 * @{
 */
#define PWM_CLOCKSOURCE_ETRMODE2    (PWM_SMCR_ETPS_1)
#define PWM_CLOCKSOURCE_INTERNAL    (PWM_SMCR_ETPS_0)
#define PWM_CLOCKSOURCE_ITR0        0x00000000U
#define PWM_CLOCKSOURCE_ITR1        (PWM_SMCR_TS_0)
#define PWM_CLOCKSOURCE_ITR2        (PWM_SMCR_TS_1)
#define PWM_CLOCKSOURCE_ITR3        (PWM_SMCR_TS_0 | PWM_SMCR_TS_1)
#define PWM_CLOCKSOURCE_TI1ED       (PWM_SMCR_TS_2)
#define PWM_CLOCKSOURCE_TI1         (PWM_SMCR_TS_0 | PWM_SMCR_TS_2)
#define PWM_CLOCKSOURCE_TI2         (PWM_SMCR_TS_1 | PWM_SMCR_TS_2)
#define PWM_CLOCKSOURCE_ETRMODE1    (PWM_SMCR_TS)
/**
 * @}
 */

/** @defgroup PWM_Clock_Polarity  TIM Clock Polarity
 * @{
 */
#define PWM_CLOCKPOLARITY_INVERTED           PWM_ETRPOLARITY_INVERTED          /*!< Polarity for ETRx clock sources */
#define PWM_CLOCKPOLARITY_NONINVERTED        PWM_ETRPOLARITY_NONINVERTED       /*!< Polarity for ETRx clock sources */
#define PWM_CLOCKPOLARITY_RISING             PWM_INPUTCHANNELPOLARITY_RISING   /*!< Polarity for TIx clock sources */
#define PWM_CLOCKPOLARITY_FALLING            PWM_INPUTCHANNELPOLARITY_FALLING   /*!< Polarity for TIx clock sources */
#define PWM_CLOCKPOLARITY_BOTHEDGE           PWM_INPUTCHANNELPOLARITY_BOTHEDGE  /*!< Polarity for TIx clock sources */
/**
 * @}
 */

/** @defgroup PWM_Clock_Prescaler  TIM Clock Prescaler
 * @{
 */
#define PWM_CLOCKPRESCALER_DIV1              PWM_ETRPRESCALER_DIV1     /*!< No prescaler is used */
#define PWM_CLOCKPRESCALER_DIV2              PWM_ETRPRESCALER_DIV2     /*!< Prescaler for External ETR Clock: Capture performed once every 2 events. */
#define PWM_CLOCKPRESCALER_DIV4              PWM_ETRPRESCALER_DIV4     /*!< Prescaler for External ETR Clock: Capture performed once every 4 events. */
#define PWM_CLOCKPRESCALER_DIV8              PWM_ETRPRESCALER_DIV8     /*!< Prescaler for External ETR Clock: Capture performed once every 8 events. */
/**
 * @}
 */

/** @defgroup PWM_ClearInput_Source TIM Clear Input Source
 * @{
 */
#define PWM_CLEARINPUTSOURCE_ETR           0x00000001U
#define PWM_CLEARINPUTSOURCE_NONE          0x00000000U
/**
 * @}
 */

/** @defgroup PWM_ClearInput_Polarity  TIM Clear Input Polarity
 * @{
 */
#define PWM_CLEARINPUTPOLARITY_INVERTED           PWM_ETRPOLARITY_INVERTED                    /*!< Polarity for ETRx pin */
#define PWM_CLEARINPUTPOLARITY_NONINVERTED        PWM_ETRPOLARITY_NONINVERTED                 /*!< Polarity for ETRx pin */
/**
 * @}
 */

/** @defgroup PWM_ClearInput_Prescaler TIM Clear Input Prescaler
 * @{
 */
#define PWM_CLEARINPUTPRESCALER_DIV1                    PWM_ETRPRESCALER_DIV1      /*!< No prescaler is used */
#define PWM_CLEARINPUTPRESCALER_DIV2                    PWM_ETRPRESCALER_DIV2      /*!< Prescaler for External ETR pin: Capture performed once every 2 events. */
#define PWM_CLEARINPUTPRESCALER_DIV4                    PWM_ETRPRESCALER_DIV4      /*!< Prescaler for External ETR pin: Capture performed once every 4 events. */
#define PWM_CLEARINPUTPRESCALER_DIV8                    PWM_ETRPRESCALER_DIV8        /*!< Prescaler for External ETR pin: Capture performed once every 8 events. */
/**
 * @}
 */

/** @defgroup PWM_OSSR_Off_State_Selection_for_Run_mode_state TIM OSSR OffState Selection for Run mode state
 * @{
 */
#define PWM_OSSR_ENABLE         (PWM_BDTR_OSSR)
#define PWM_OSSR_DISABLE        0x00000000U
/**
 * @}
 */

/** @defgroup PWM_OSSI_Off_State_Selection_for_Idle_mode_state TIM OSSI OffState Selection for Idle mode state
 * @{
 */
#define PWM_OSSI_ENABLE             (PWM_BDTR_OSSI)
#define PWM_OSSI_DISABLE            0x00000000U
/**
 * @}
 */

/** @defgroup PWM_Lock_level  TIM Lock level
 * @{
 */
#define PWM_LOCKLEVEL_OFF          0x00000000U
#define PWM_LOCKLEVEL_1            (PWM_BDTR_LOCK_0)
#define PWM_LOCKLEVEL_2            (PWM_BDTR_LOCK_1)
#define PWM_LOCKLEVEL_3            (PWM_BDTR_LOCK)
/**
 * @}
 */
/** @defgroup PWM_Break_Input_enable_disable  TIM Break Input State
 * @{
 */
#define PWM_BREAK_ENABLE          (PWM_BDTR_BKE)
#define PWM_BREAK_DISABLE         0x00000000U
/**
 * @}
 */

/** @defgroup PWM_Break_Polarity  TIM Break Polarity
 * @{
 */
#define PWM_BREAKPOLARITY_LOW        0x00000000U
#define PWM_BREAKPOLARITY_HIGH       (PWM_BDTR_BKP)
/**
 * @}
 */

/** @defgroup PWM_AOE_Bit_Set_Reset  TIM AOE Bit State
 * @{
 */
#define PWM_AUTOMATICOUTPUT_ENABLE           (PWM_BDTR_AOE)
#define PWM_AUTOMATICOUTPUT_DISABLE          0x00000000U
/**
 * @}
 */

/** @defgroup PWM_Master_Mode_Selection TIM Master Mode Selection
 * @{
 */
#define   PWM_CR2_MMS_0 (1<<ATIM_CR2_MMS_Pos)
#define   PWM_CR2_MMS_1 (2<<ATIM_CR2_MMS_Pos)
#define   PWM_CR2_MMS_2 (4<<ATIM_CR2_MMS_Pos)

#define PWM_TRGO_RESET            0x00000000U
#define PWM_TRGO_ENABLE           (PWM_CR2_MMS_0)
#define PWM_TRGO_UPDATE           (PWM_CR2_MMS_1)
#define PWM_TRGO_OC1              ((PWM_CR2_MMS_1 | PWM_CR2_MMS_0))
#define PWM_TRGO_OC1REF           (PWM_CR2_MMS_2)
#define PWM_TRGO_OC2REF           ((PWM_CR2_MMS_2 | PWM_CR2_MMS_0))
#define PWM_TRGO_OC3REF           ((PWM_CR2_MMS_2 | PWM_CR2_MMS_1))
#define PWM_TRGO_OC4REF           ((PWM_CR2_MMS_2 | PWM_CR2_MMS_1 | PWM_CR2_MMS_0))
/**
 * @}
 */

/** @defgroup PWM_Slave_Mode TIM Slave Mode
 * @{
 */
#define PWM_SLAVEMODE_DISABLE              0x00000000U
#define PWM_SLAVEMODE_RESET                0x00000004U
#define PWM_SLAVEMODE_GATED                0x00000005U
#define PWM_SLAVEMODE_TRIGGER              0x00000006U
#define PWM_SLAVEMODE_EXTERNAL1            0x00000007U
/**
 * @}
 */

/** @defgroup PWM_Master_Slave_Mode  TIM Master Slave Mode
 * @{
 */
#define PWM_MASTERSLAVEMODE_ENABLE          0x00000080U
#define PWM_MASTERSLAVEMODE_DISABLE         0x00000000U
/**
 * @}
 */

/** @defgroup PWM_Trigger_Selection  TIM Trigger Selection
 * @{
 */
#define PWM_TS_ITR0                        0x00000000U
#define PWM_TS_ITR1                        0x00000010U
#define PWM_TS_ITR2                        0x00000020U
#define PWM_TS_ITR3                        0x00000030U
#define PWM_TS_TI1F_ED                     0x00000040U
#define PWM_TS_TI1FP1                      0x00000050U
#define PWM_TS_TI2FP2                      0x00000060U
#define PWM_TS_ETRF                        0x00000070U
#define PWM_TS_NONE                        0x0000FFFFU
/**
 * @}
 */

/** @defgroup PWM_Trigger_Polarity TIM Trigger Polarity
 * @{
 */
#define PWM_TRIGGERPOLARITY_INVERTED           PWM_ETRPOLARITY_INVERTED      /*!< Polarity for ETRx trigger sources */
#define PWM_TRIGGERPOLARITY_NONINVERTED        PWM_ETRPOLARITY_NONINVERTED   /*!< Polarity for ETRx trigger sources */
#define PWM_TRIGGERPOLARITY_RISING             PWM_INPUTCHANNELPOLARITY_RISING        /*!< Polarity for TIxFPx or TI1_ED trigger sources */
#define PWM_TRIGGERPOLARITY_FALLING            PWM_INPUTCHANNELPOLARITY_FALLING       /*!< Polarity for TIxFPx or TI1_ED trigger sources */
#define PWM_TRIGGERPOLARITY_BOTHEDGE           PWM_INPUTCHANNELPOLARITY_BOTHEDGE      /*!< Polarity for TIxFPx or TI1_ED trigger sources */
/**
 * @}
 */

/** @defgroup PWM_Trigger_Prescaler TIM Trigger Prescaler
 * @{
 */
#define PWM_TRIGGERPRESCALER_DIV1             PWM_ETRPRESCALER_DIV1     /*!< No prescaler is used */
#define PWM_TRIGGERPRESCALER_DIV2             PWM_ETRPRESCALER_DIV2     /*!< Prescaler for External ETR Trigger: Capture performed once every 2 events. */
#define PWM_TRIGGERPRESCALER_DIV4             PWM_ETRPRESCALER_DIV4     /*!< Prescaler for External ETR Trigger: Capture performed once every 4 events. */
#define PWM_TRIGGERPRESCALER_DIV8             PWM_ETRPRESCALER_DIV8     /*!< Prescaler for External ETR Trigger: Capture performed once every 8 events. */
/**
 * @}
 */


/** @defgroup PWM_TI1_Selection TIM TI1 Selection
 * @{
 */
#define PWM_TI1SELECTION_CH1                0x00000000U
#define PWM_TI1SELECTION_XORCOMBINATION     (PWM_CR2_TI1S)
/**
 * @}
 */

/** @defgroup PWM_DMA_Base_address  TIM DMA Base address
 * @{
 */
#define PWM_DMABASE_CR1                    0x00000000U
#define PWM_DMABASE_CR2                    0x00000001U
#define PWM_DMABASE_SMCR                   0x00000002U
#define PWM_DMABASE_DIER                   0x00000003U
#define PWM_DMABASE_SR                     0x00000004U
#define PWM_DMABASE_EGR                    0x00000005U
#define PWM_DMABASE_CCMR1                  0x00000006U
#define PWM_DMABASE_CCMR2                  0x00000007U
#define PWM_DMABASE_CCER                   0x00000008U
#define PWM_DMABASE_CNT                    0x00000009U
#define PWM_DMABASE_PSC                    0x0000000AU
#define PWM_DMABASE_ARR                    0x0000000BU
#define PWM_DMABASE_RCR                    0x0000000CU
#define PWM_DMABASE_CCR1                   0x0000000DU
#define PWM_DMABASE_CCR2                   0x0000000EU
#define PWM_DMABASE_CCR3                   0x0000000FU
#define PWM_DMABASE_CCR4                   0x00000010U
#define PWM_DMABASE_BDTR                   0x00000011U
#define PWM_DMABASE_DCR                    0x00000012U
#define PWM_DMABASE_OR                     0x00000013U
/**
 * @}
 */

/** @defgroup PWM_DMA_Burst_Length  TIM DMA Burst Length
 * @{
 */
#define PWM_DMABURSTLENGTH_1TRANSFER           0x00000000U
#define PWM_DMABURSTLENGTH_2TRANSFERS          0x00000100U
#define PWM_DMABURSTLENGTH_3TRANSFERS          0x00000200U
#define PWM_DMABURSTLENGTH_4TRANSFERS          0x00000300U
#define PWM_DMABURSTLENGTH_5TRANSFERS          0x00000400U
#define PWM_DMABURSTLENGTH_6TRANSFERS          0x00000500U
#define PWM_DMABURSTLENGTH_7TRANSFERS          0x00000600U
#define PWM_DMABURSTLENGTH_8TRANSFERS          0x00000700U
#define PWM_DMABURSTLENGTH_9TRANSFERS          0x00000800U
#define PWM_DMABURSTLENGTH_10TRANSFERS         0x00000900U
#define PWM_DMABURSTLENGTH_11TRANSFERS         0x00000A00U
#define PWM_DMABURSTLENGTH_12TRANSFERS         0x00000B00U
#define PWM_DMABURSTLENGTH_13TRANSFERS         0x00000C00U
#define PWM_DMABURSTLENGTH_14TRANSFERS         0x00000D00U
#define PWM_DMABURSTLENGTH_15TRANSFERS         0x00000E00U
#define PWM_DMABURSTLENGTH_16TRANSFERS         0x00000F00U
#define PWM_DMABURSTLENGTH_17TRANSFERS         0x00001000U
#define PWM_DMABURSTLENGTH_18TRANSFERS         0x00001100U
/**
 * @}
 */

/** @defgroup DMA_Handle_index  DMA Handle index
 * @{
 */
#define PWM_DMA_ID_UPDATE                ((uint16_t)0x0000)       /*!< Index of the DMA handle used for Update DMA requests */
#define PWM_DMA_ID_CC1                   ((uint16_t)0x0001)       /*!< Index of the DMA handle used for Capture/Compare 1 DMA requests */
#define PWM_DMA_ID_CC2                   ((uint16_t)0x0002)       /*!< Index of the DMA handle used for Capture/Compare 2 DMA requests */
#define PWM_DMA_ID_CC3                   ((uint16_t)0x0003)       /*!< Index of the DMA handle used for Capture/Compare 3 DMA requests */
#define PWM_DMA_ID_CC4                   ((uint16_t)0x0004)       /*!< Index of the DMA handle used for Capture/Compare 4 DMA requests */
#define PWM_DMA_ID_COMMUTATION           ((uint16_t)0x0005)       /*!< Index of the DMA handle used for Commutation DMA requests */
#define PWM_DMA_ID_TRIGGER               ((uint16_t)0x0006)       /*!< Index of the DMA handle used for Trigger DMA requests */
/**
 * @}
 */

/** @defgroup Channel_CC_State  Channel CC State
 * @{
 */
#define PWM_CCx_ENABLE                   PWM_CR1_CCE
#define PWM_CCx_DISABLE                  0x00000000U
/**
 * @}
 */

/**
 * @}
 */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup PWM_Exported_Macros TIM Exported Macros
 * @{
 */
/** @brief Reset TIM handle state
 * @param  \__HANDLE__ TIM handle
 * @retval None
 */
#define __HAL_PWM_RESET_HANDLE_STATE(__HANDLE__) ((__HANDLE__)->State = HAL_PWM_STATE_RESET)

/**
 * @brief  Enable the PWM peripheral.
 * @param  \__HANDLE__ PWM Channel handle
 * @retval None
 */
#define __HAL_PWM_ENABLE(__HANDLE__, __CHANNEL__)          \
    (((PWM_ChannelTypeDef *)&((__HANDLE__)->Instance->CR1) + (__CHANNEL__))->CR |= (PWM_CR1_CEN))

#define __HAL_PWM_IS_ENABLED(__INSTANCE__, __CHANNEL__)     \
    (((PWM_ChannelTypeDef *)&((__INSTANCE__)->CR1) + (__CHANNEL__))->CR & (PWM_CR1_CEN))


/**
 * @brief  Disable the PWM peripheral.
 * @param  \__HANDLE__ PWM Channel handle
 * @retval None
 */
#define __HAL_PWM_DISABLE(__HANDLE__, __CHANNEL__)  \
    (((PWM_ChannelTypeDef *)&((__HANDLE__)->Instance->CR1) + (__CHANNEL__))->CR &= ~(PWM_CR1_CEN))


/** @brief  Enable the specified TIM interrupt.
 * @param  \__HANDLE__ specifies the TIM Handle.
 * @param  \__INTERRUPT__ specifies the TIM interrupt source to enable.
 *          This parameter can be one of the following values:
 *            @arg PWM_IT_UPDATE: Update interrupt
 *            @arg PWM_IT_CC1:   Capture/Compare 1 interrupt
 *            @arg PWM_IT_CC2:  Capture/Compare 2 interrupt
 *            @arg PWM_IT_CC3:  Capture/Compare 3 interrupt
 *            @arg PWM_IT_CC4:  Capture/Compare 4 interrupt
 *            @arg PWM_IT_COM:   Commutation interrupt
 *            @arg PWM_IT_TRIGGER: Trigger interrupt
 *            @arg PWM_IT_BREAK: Break interrupt
 * @retval None
 */
#define __HAL_PWM_ENABLE_IT(__HANDLE__, __CHANNEL__, __INTERRUPT__)    \
    (((PWM_ChannelTypeDef *)&((__HANDLE__)->Instance->CR1) + (__CHANNEL__))->CR |= (__INTERRUPT__))


/** @brief  Disable the specified TIM interrupt.
 * @param  \__HANDLE__ specifies the TIM Handle.
 * @param  \__INTERRUPT__ specifies the TIM interrupt source to disable.
 *          This parameter can be one of the following values:
 *            @arg PWM_IT_UPDATE: Update interrupt
 *            @arg PWM_IT_CC1:   Capture/Compare 1 interrupt
 *            @arg PWM_IT_CC2:  Capture/Compare 2 interrupt
 *            @arg PWM_IT_CC3:  Capture/Compare 3 interrupt
 *            @arg PWM_IT_CC4:  Capture/Compare 4 interrupt
 *            @arg PWM_IT_COM:   Commutation interrupt
 *            @arg PWM_IT_TRIGGER: Trigger interrupt
 *            @arg PWM_IT_BREAK: Break interrupt
 * @retval None
 */
#define __HAL_PWM_DISABLE_IT(__HANDLE__, __CHANNEL__, __INTERRUPT__)    \
    (((PWM_ChannelTypeDef *)&((__HANDLE__)->Instance->CR1) + (__CHANNEL__))->CR &= ~(__INTERRUPT__))


/** @brief  Enable the specified DMA request.
 * @param  \__HANDLE__ specifies the TIM Handle.
 * @param  \__DMA__ specifies the TIM DMA request to enable.
 *          This parameter can be one of the following values:
 *            @arg PWM_DMA_UPDATE: Update DMA request
 *            @arg PWM_DMA_CC1:   Capture/Compare 1 DMA request
 *            @arg PWM_DMA_CC2:  Capture/Compare 2 DMA request
 *            @arg PWM_DMA_CC3:  Capture/Compare 3 DMA request
 *            @arg PWM_DMA_CC4:  Capture/Compare 4 DMA request
 *            @arg PWM_DMA_COM:   Commutation DMA request
 *            @arg PWM_DMA_TRIGGER: Trigger DMA request
 * @retval None
 */
#define __HAL_PWM_ENABLE_DMA(__HANDLE__, __CHANNEL__, __DMA__)      \
    (((PWM_ChannelTypeDef *)&((__HANDLE__)->Instance->CR1) + (__CHANNEL__))->CR |= (__DMA__))

/** @brief  Disable the specified DMA request.
 * @param  \__HANDLE__ specifies the TIM Handle.
 * @param  \__DMA__ specifies the TIM DMA request to disable.
 *          This parameter can be one of the following values:
 *            @arg PWM_DMA_UPDATE: Update DMA request
 *            @arg PWM_DMA_CC1:   Capture/Compare 1 DMA request
 *            @arg PWM_DMA_CC2:  Capture/Compare 2 DMA request
 *            @arg PWM_DMA_CC3:  Capture/Compare 3 DMA request
 *            @arg PWM_DMA_CC4:  Capture/Compare 4 DMA request
 *            @arg PWM_DMA_COM:   Commutation DMA request
 *            @arg PWM_DMA_TRIGGER: Trigger DMA request
 * @retval None
 */
#define __HAL_PWM_DISABLE_DMA(__HANDLE__, __CHANNEL__, __DMA__)       \
    (((PWM_ChannelTypeDef *)&((__HANDLE__)->Instance->CR1) + (__CHANNEL__))->CR &= ~(__DMA__))

/** @brief  Check whether the specified TIM interrupt flag is set or not.
 * @param  \__HANDLE__ specifies the TIM Handle.
 * @param  \__FLAG__ specifies the TIM interrupt flag to check.
 *        This parameter can be one of the following values:
 *            @arg PWM_FLAG_UPDATE: Update interrupt flag
 *            @arg PWM_FLAG_CC1: Capture/Compare 1 interrupt flag
 *            @arg PWM_FLAG_CC2: Capture/Compare 2 interrupt flag
 *            @arg PWM_FLAG_CC3: Capture/Compare 3 interrupt flag
 *            @arg PWM_FLAG_CC4: Capture/Compare 4 interrupt flag
 *            @arg PWM_FLAG_CC5: Compare 5 interrupt flag
 *            @arg PWM_FLAG_CC6: Compare 6 interrupt flag
 *            @arg PWM_FLAG_COM:  Commutation interrupt flag
 *            @arg PWM_FLAG_TRIGGER: Trigger interrupt flag
 *            @arg PWM_FLAG_BREAK: Break interrupt flag
 *            @arg PWM_FLAG_BREAK2: Break 2 interrupt flag
 *            @arg PWM_FLAG_SYSTEM_BREAK: System Break interrupt flag
 *            @arg PWM_FLAG_CC1OF: Capture/Compare 1 overcapture flag
 *            @arg PWM_FLAG_CC2OF: Capture/Compare 2 overcapture flag
 *            @arg PWM_FLAG_CC3OF: Capture/Compare 3 overcapture flag
 *            @arg PWM_FLAG_CC4OF: Capture/Compare 4 overcapture flag
 * @retval The new state of __FLAG__ (TRUE or FALSE).
 */
#define __HAL_PWM_GET_FLAG(__HANDLE__, __FLAG__)          (((__HANDLE__)->Instance->SR &(__FLAG__)) == (__FLAG__))

/** @brief  Clear the specified TIM interrupt flag.
 * @param  \__HANDLE__ specifies the TIM Handle.
 * @param  \__FLAG__ specifies the TIM interrupt flag to clear.
 *        This parameter can be one of the following values:
 *            @arg PWM_FLAG_UPDATE: Update interrupt flag
 *            @arg PWM_FLAG_CC1: Capture/Compare 1 interrupt flag
 *            @arg PWM_FLAG_CC2: Capture/Compare 2 interrupt flag
 *            @arg PWM_FLAG_CC3: Capture/Compare 3 interrupt flag
 *            @arg PWM_FLAG_CC4: Capture/Compare 4 interrupt flag
 *            @arg PWM_FLAG_CC5: Compare 5 interrupt flag
 *            @arg PWM_FLAG_CC6: Compare 6 interrupt flag
 *            @arg PWM_FLAG_COM:  Commutation interrupt flag
 *            @arg PWM_FLAG_TRIGGER: Trigger interrupt flag
 *            @arg PWM_FLAG_BREAK: Break interrupt flag
 *            @arg PWM_FLAG_BREAK2: Break 2 interrupt flag
 *            @arg PWM_FLAG_SYSTEM_BREAK: System Break interrupt flag
 *            @arg PWM_FLAG_CC1OF: Capture/Compare 1 overcapture flag
 *            @arg PWM_FLAG_CC2OF: Capture/Compare 2 overcapture flag
 *            @arg PWM_FLAG_CC3OF: Capture/Compare 3 overcapture flag
 *            @arg PWM_FLAG_CC4OF: Capture/Compare 4 overcapture flag
 * @retval The new state of __FLAG__ (TRUE or FALSE).
 */
#define __HAL_PWM_CLEAR_FLAG(__HANDLE__, __FLAG__)        ((__HANDLE__)->Instance->SR = ~(__FLAG__))

/**
 * @brief  Check whether the specified TIM interrupt source is enabled or not.
 * @param  \__HANDLE__ TIM handle
 * @param  \__INTERRUPT__ specifies the TIM interrupt source to check.
 *          This parameter can be one of the following values:
 *            @arg PWM_IT_UPDATE: Update interrupt
 *            @arg PWM_IT_CC1:   Capture/Compare 1 interrupt
 *            @arg PWM_IT_CC2:  Capture/Compare 2 interrupt
 *            @arg PWM_IT_CC3:  Capture/Compare 3 interrupt
 *            @arg PWM_IT_CC4:  Capture/Compare 4 interrupt
 *            @arg PWM_IT_COM:   Commutation interrupt
 *            @arg PWM_IT_TRIGGER: Trigger interrupt
 *            @arg PWM_IT_BREAK: Break interrupt
 * @retval The state of PWM_IT (SET or RESET).
 */
#define __HAL_PWM_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__) ((((__HANDLE__)->Instance->IER & (__INTERRUPT__)) == (__INTERRUPT__)) ? SET : RESET)

/** @brief Clear the TIM interrupt pending bits.
 * @param  \__HANDLE__ TIM handle
 * @param  \__INTERRUPT__ specifies the interrupt pending bit to clear.
 *          This parameter can be one of the following values:
 *            @arg PWM_IT_UPDATE: Update interrupt
 *            @arg PWM_IT_CC1:   Capture/Compare 1 interrupt
 *            @arg PWM_IT_CC2:  Capture/Compare 2 interrupt
 *            @arg PWM_IT_CC3:  Capture/Compare 3 interrupt
 *            @arg PWM_IT_CC4:  Capture/Compare 4 interrupt
 *            @arg PWM_IT_COM:   Commutation interrupt
 *            @arg PWM_IT_TRIGGER: Trigger interrupt
 *            @arg PWM_IT_BREAK: Break interrupt
 * @retval None
 */
#define __HAL_PWM_CLEAR_IT(__HANDLE__, __INTERRUPT__)     ((__HANDLE__)->Instance->SR = ~(__INTERRUPT__))

/**
 * @brief  Set the TIM Prescaler on runtime.
 * @param  \__HANDLE__ TIM handle.
 * @param  \__CHANNEL__ Channel
 * @param  \__PRESC__ specifies the Prescaler new value.
 * @retval None
 */
#define __HAL_PWM_SET_PRESCALER(__HANDLE__, __CHANNEL__, __PRESC__)     \
    (((PWM_ChannelTypeDef *)&((__HANDLE__)->Instance->CR1) + (__CHANNEL__))->PSC = (__PRESC__))

/**
 * @brief  Get the PWM Prescaler on runtime.
 * @param  \__HANDLE__ TIM handle.
 * @param  \__CHANNEL__ Channel
 * @retval None
 */
#define __HAL_PWM_GET_PRESCALER(__HANDLE__, __CHANNEL__)     \
    (((PWM_ChannelTypeDef *)&((__HANDLE__)->Instance->CR1) + (__CHANNEL__))->PSC)

/**
 * @brief  Set the TIM single mode.
 * @param  \__HANDLE__ TIM handle.
 * @param  \__MODE__ PWM_OPMODE_REPETITIVE:REPETITIVE, PWM_OPMODE_SINGLE:One pulse.
 * @retval None
 */
#define __HAL_PWM_SET_MODE(__HANDLE__, __CHANNEL__, __MODE__)       \
    (__MODE__?(((PWM_ChannelTypeDef *)&((__HANDLE__)->Instance->CR1) + (__CHANNEL__))->CR |= (__MODE__)):(((PWM_ChannelTypeDef *)&((__HANDLE__)->Instance->CR1) + (__CHANNEL__))->CR &= ~PWM_OPMODE_SINGLE))


/**
 * @brief  Sets the TIM Capture Compare Register value on runtime without
 *         calling another time ConfigChannel function.
 * @param  \__HANDLE__ TIM handle.
 * @param  \__CHANNEL__  TIM Channels to be configured.
 *          This parameter can be one of the following values:
 *            @arg PWM_CHANNEL_1: TIM Channel 1 selected
 *            @arg PWM_CHANNEL_2: TIM Channel 2 selected
 *            @arg PWM_CHANNEL_3: TIM Channel 3 selected
 *            @arg PWM_CHANNEL_4: TIM Channel 4 selected
 * @param  \__COMPARE__ specifies the Capture Compare register new value.
 * @retval None
 */
#define __HAL_PWM_SET_COMPARE(__HANDLE__, __CHANNEL__, __COMPARE__) \
  (((PWM_ChannelTypeDef *)&((__HANDLE__)->Instance->CR1) + (__CHANNEL__))->CCR = (__COMPARE__))

/**
 * @brief  Gets the TIM Capture Compare Register value on runtime.
 * @param  \__HANDLE__ TIM handle.
 * @param  \__CHANNEL__ TIM Channel associated with the capture compare register
 *          This parameter can be one of the following values:
 *            @arg PWM_CHANNEL_1: get capture/compare 1 register value
 *            @arg PWM_CHANNEL_2: get capture/compare 2 register value
 *            @arg PWM_CHANNEL_3: get capture/compare 3 register value
 *            @arg PWM_CHANNEL_4: get capture/compare 4 register value
 *            @arg PWM_CHANNEL_5: get capture/compare 5 register value
 *            @arg PWM_CHANNEL_6: get capture/compare 6 register value
 * @retval 16-bit or 32-bit value of the capture/compare register (TIMx_CCRy)
 */
#define __HAL_PWM_GET_COMPARE(__HANDLE__, __CHANNEL__)  (((PWM_ChannelTypeDef *)&((__HANDLE__)->Instance->CR1) + (__CHANNEL__))->CCR)

/**
 * @brief  Sets the TIM Counter Register value on runtime.
 * @param  \__HANDLE__ TIM handle.
 * @param  \__COUNTER__ specifies the Counter register new value.
 * @retval None
 */
#define __HAL_PWM_SET_COUNTER(__HANDLE__, __CHANNEL__, __COUNTER__)    \
    (((PWM_ChannelTypeDef *)&((__HANDLE__)->Instance->CR1) + (__CHANNEL__))->CNT = (__COUNTER__))

/**
 * @brief  Gets the TIM Counter Register value on runtime.
 * @param  \__HANDLE__ TIM handle.
 * @retval 16-bit or 32-bit value of the timer counter register (TIMx_CNT)
 */
#define __HAL_PWM_GET_COUNTER(__HANDLE__, __CHANNEL__) (((PWM_ChannelTypeDef *)&((__HANDLE__)->Instance->CR1) + (__CHANNEL__))->CNT)

/**
 * @brief  Sets the TIM Autoreload Register value on runtime without calling
 *         another time any Init function.
 * @param  \__HANDLE__ TIM handle.
 * @param  \__AUTORELOAD__ specifies the Counter register new value.
 * @retval None
 */
#define __HAL_PWM_SET_AUTORELOAD(__HANDLE__, __CHANNEL__, __AUTORELOAD__)                  \
                        do{                                                  \
                            ((PWM_ChannelTypeDef *)&((__HANDLE__)->Instance->CR1) + (__CHANNEL__))->ARR = (__AUTORELOAD__);  \
                            (__HANDLE__)->Init.Period = (__AUTORELOAD__);    \
                          } while(0U)
/**
 * @brief  Gets the TIM Autoreload Register value on runtime.
 * @param  \__HANDLE__ TIM handle.
 * @retval 16-bit or 32-bit value of the timer auto-reload register(TIMx_ARR)
 */
#define __HAL_PWM_GET_AUTORELOAD(__HANDLE__, __CHANNEL__) (((PWM_ChannelTypeDef *)&((__HANDLE__)->Instance->CR1) + (__CHANNEL__))->ARR)

/**
 * @brief  Set the Update Request Source (URS) bit of the TIMx_CR1 register
 * @param  \__HANDLE__ TIM handle.
 * @note  When the USR bit of the TIMx_CR1 register is set, only counter
 *        overflow/underflow generates an update interrupt or DMA request (if
 *        enabled)
 * @retval None
 */
#define __HAL_PWM_URS_ENABLE(__HANDLE__, __CHANNEL__) \
    (((PWM_ChannelTypeDef *)&((__HANDLE__)->Instance->CR1) + (__CHANNEL__))->CR |= (PWM_CR1_URS))

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
#define __HAL_PWM_URS_DISABLE(__HANDLE__, __CHANNEL__) \
      (((PWM_ChannelTypeDef *)&((__HANDLE__)->Instance->CR1) + (__CHANNEL__))->CR &= ~(PWM_CR1_URS))

/**
 * @}
 */


/* Exported functions --------------------------------------------------------*/
/** @addtogroup PWM_Exported_Functions
 * @{
 */

/** @addtogroup PWM_Exported_Functions_Group1
 * @{
 */

/* Time Base functions ********************************************************/

/**
 * @brief  Initializes the TIM Time base Unit according to the specified
 *         parameters in the PWM_HandleTypeDef and create the associated handle.
 * @param  hpwm pointer to a PWM_HandleTypeDef structure that contains
 *                the configuration information for TIM module.
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_PWM_Init(PWM_HandleTypeDef *hpwm);

/**
 * @brief  DeInitializes the TIM Base peripheral
 * @param  hpwm pointer to a PWM_HandleTypeDef structure that contains
 *                the configuration information for TIM module.
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_PWM_DeInit(PWM_HandleTypeDef *hpwm);

/**
 * @brief  Initializes the TIM Base MSP.
 * @param  hpwm pointer to a PWM_HandleTypeDef structure that contains
 *                the configuration information for TIM module.
 * @retval None
 */
void HAL_PWM_MspInit(PWM_HandleTypeDef *hpwm);

/**
 * @brief  DeInitializes TIM Base MSP.
 * @param  hpwm pointer to a PWM_HandleTypeDef structure that contains
 *                the configuration information for TIM module.
 * @retval None
 */
void HAL_PWM_MspDeInit(PWM_HandleTypeDef *hpwm);

/**
 * @brief  Starts the TIM Base generation.
 * @param  hpwm pointer to a PWM_HandleTypeDef structure that contains
 *                the configuration information for TIM module.
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_PWM_Start(PWM_HandleTypeDef *hpwm, uint32_t Channel);

/**
 * @brief  Stops the TIM Base generation.
 * @param  hpwm pointer to a PWM_HandleTypeDef structure that contains
 *                the configuration information for TIM module.
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_PWM_Stop(PWM_HandleTypeDef *hpwm, uint32_t Channel);

/**
 * @brief  Starts the TIM Base generation in interrupt mode.
 * @param  hpwm pointer to a PWM_HandleTypeDef structure that contains
 *                the configuration information for TIM module.
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_PWM_Start_IT(PWM_HandleTypeDef *hpwm, uint32_t Channel);

/**
 * @brief  Stops the TIM Base generation in interrupt mode.
 * @param  hpwm pointer to a PWM_HandleTypeDef structure that contains
 *                the configuration information for TIM module.
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_PWM_Stop_IT(PWM_HandleTypeDef *hpwm, uint32_t Channel);

/**
 * @brief  Starts the TIM Base generation in DMA mode.
 * @param  hpwm pointer to a PWM_HandleTypeDef structure that contains
 *                the configuration information for TIM module.
 * @param  pData The source Buffer address.
 * @param  Length The length of data to be transferred from memory to peripheral.
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_PWM_Start_DMA(PWM_HandleTypeDef *hpwm, uint32_t Channel, uint32_t *pData, uint16_t Length);

/**
 * @brief  Stops the TIM Base generation in DMA mode.
 * @param  hpwm pointer to a PWM_HandleTypeDef structure that contains
 *                the configuration information for TIM module.
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_PWM_Stop_DMA(PWM_HandleTypeDef *hpwm, uint32_t Channel);
/**
 * @}
 */

/** @addtogroup PWM_Exported_Functions_Group7
 * @{
 */
/* Interrupt Handler functions  **********************************************/
/**
 * @brief  This function handles TIM interrupts requests.
 * @param  hpwm pointer to a PWM_HandleTypeDef structure that contains
 *                the configuration information for TIM module.
 * @retval None
 */
void HAL_PWM_IRQHandler(PWM_HandleTypeDef *hpwm);

/**
 * @}
 */

/** @addtogroup PWM_Exported_Functions_Group8
 * @{
 */
/* Control functions  *********************************************************/
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
HAL_StatusTypeDef HAL_PWM_ConfigChannel(PWM_HandleTypeDef *hpwm, PWM_OC_InitTypeDef *sConfig, uint32_t Channel);

/**
 * @brief  Generate a software event
 * @param  hpwm pointer to a PWM_HandleTypeDef structure that contains
 *                the configuration information for TIM module.
 * @param  EventSource specifies the event source.
 *          This parameter can be one of the following values:
 *            @arg PWM_EVENTSOURCE_UPDATE: Timer update Event source
 *            @arg PWM_EVENTSOURCE_CC1: Timer Capture Compare 1 Event source
 *            @arg PWM_EVENTSOURCE_CC2: Timer Capture Compare 2 Event source
 *            @arg PWM_EVENTSOURCE_CC3: Timer Capture Compare 3 Event source
 *            @arg PWM_EVENTSOURCE_CC4: Timer Capture Compare 4 Event source
 *            @arg PWM_EVENTSOURCE_COM: Timer COM event source
 *            @arg PWM_EVENTSOURCE_TRIGGER: Timer Trigger Event source
 *            @arg PWM_EVENTSOURCE_BREAK: Timer Break event source
 * @note   TIM6 and TIM7 can only generate an update event.
 * @note   PWM_EVENTSOURCE_COM and PWM_EVENTSOURCE_BREAK are used only with TIM1 and TIM8.
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_PWM_GenerateEvent(PWM_HandleTypeDef *hpwm, uint32_t EventSource);


/**
 * @}
 */

/**
 * @brief  Hall Trigger detection callback in non blocking mode
 * @param  hpwm pointer to a PWM_HandleTypeDef structure that contains
 *                the configuration information for TIM module.
 * @retval None
 */
void HAL_PWM_TriggerCallback(PWM_HandleTypeDef *hpwm);
/**
 * @brief  Timer error callback in non blocking mode
 * @param  hpwm pointer to a PWM_HandleTypeDef structure that contains
 *                the configuration information for TIM module.
 * @retval None
 */
void HAL_PWM_ErrorCallback(PWM_HandleTypeDef *hpwm);

/**
 * @}
 */

/** @addtogroup PWM_Exported_Functions_Group10
 * @{
 */
/* Peripheral State functions  **************************************************/
/**
 * @brief  Return the PWM state
 * @param  hpwm pointer to a PWM_HandleTypeDef structure that contains
 *                the configuration information for TIM module.
 * @retval HAL state
 */
HAL_PWM_StateTypeDef HAL_PWM_GetState(PWM_HandleTypeDef *hpwm);

/**
 * @}
 */

/**
 * @}
 */

/* Private macros ------------------------------------------------------------*/
/** @defgroup PWM_Private_Macros TIM Private Macros
 * @{
 */

/** @defgroup PWM_IS_PWM_Definitions TIM Private macros to check input parameters
 * @{
 */

#define IS_PWM_PWM_MODE(MODE) (((MODE) == PWM_OCMODE_PWM1) || \
                               ((MODE) == PWM_OCMODE_PWM2))

#define IS_PWM_OC_MODE(MODE) (((MODE) == PWM_OCMODE_TIMING)       || \
                          ((MODE) == PWM_OCMODE_ACTIVE)           || \
                          ((MODE) == PWM_OCMODE_INACTIVE)         || \
                          ((MODE) == PWM_OCMODE_TOGGLE)           || \
                          ((MODE) == PWM_OCMODE_FORCED_ACTIVE)    || \
                          ((MODE) == PWM_OCMODE_FORCED_INACTIVE))

#define IS_PWM_FAST_STATE(STATE) (((STATE) == PWM_OCFAST_DISABLE) || \
                                  ((STATE) == PWM_OCFAST_ENABLE))

#define IS_PWM_OC_POLARITY(POLARITY) (((POLARITY) == PWM_OCPOLARITY_HIGH) || \
                                      ((POLARITY) == PWM_OCPOLARITY_LOW))

#define IS_PWM_OCN_POLARITY(POLARITY) (((POLARITY) == PWM_OCNPOLARITY_HIGH) || \
                                       ((POLARITY) == PWM_OCNPOLARITY_LOW))

#define IS_PWM_OCIDLE_STATE(STATE) (((STATE) == PWM_OCIDLESTATE_SET) || \
                                    ((STATE) == PWM_OCIDLESTATE_RESET))

#define IS_PWM_OCNIDLE_STATE(STATE) (((STATE) == PWM_OCNIDLESTATE_SET) || \
                                    ((STATE) == PWM_OCNIDLESTATE_RESET))

#define IS_PWM_CHANNELS(CHANNEL) (((CHANNEL) == PWM_CHANNEL_1) || \
                                  ((CHANNEL) == PWM_CHANNEL_2) || \
                                  ((CHANNEL) == PWM_CHANNEL_3) || \
                                  ((CHANNEL) == PWM_CHANNEL_4))

#define IS_PWM_OPM_CHANNELS(CHANNEL) (((CHANNEL) == PWM_CHANNEL_1) || \
                                      ((CHANNEL) == PWM_CHANNEL_2))


#define IS_PWM_OPM_MODE(MODE) (((MODE) == PWM_OPMODE_SINGLE) || \
                               ((MODE) == PWM_OPMODE_REPETITIVE))

#define IS_PWM_DMA_SOURCE(SOURCE) ((((SOURCE) & 0xFFFF80FFU) == 0x00000000U) && ((SOURCE) != 0x00000000U))


#define IS_PWM_EVENT_SOURCE(SOURCE) ((((SOURCE) & 0xFFFFFF00U) == 0x00000000U) && ((SOURCE) != 0x00000000U))


#define IS_PWM_CLEARINPUT_SOURCE(SOURCE)  (((SOURCE) == PWM_CLEARINPUTSOURCE_NONE) || \
                                         ((SOURCE) == PWM_CLEARINPUTSOURCE_ETR))

#define IS_PWM_CLEARINPUT_POLARITY(POLARITY)   (((POLARITY) == PWM_CLEARINPUTPOLARITY_INVERTED) || \
                                               ((POLARITY) == PWM_CLEARINPUTPOLARITY_NONINVERTED))

#define IS_PWM_CLEARINPUT_PRESCALER(PRESCALER)   (((PRESCALER) == PWM_CLEARINPUTPRESCALER_DIV1) || \
                                                 ((PRESCALER) == PWM_CLEARINPUTPRESCALER_DIV2) || \
                                                 ((PRESCALER) == PWM_CLEARINPUTPRESCALER_DIV4) || \
                                                 ((PRESCALER) == PWM_CLEARINPUTPRESCALER_DIV8))


#define IS_PWM_SLAVE_MODE(MODE) (((MODE) == PWM_SLAVEMODE_DISABLE) || \
                                 ((MODE) == PWM_SLAVEMODE_GATED) || \
                                 ((MODE) == PWM_SLAVEMODE_RESET) || \
                                 ((MODE) == PWM_SLAVEMODE_TRIGGER) || \
                                 ((MODE) == PWM_SLAVEMODE_EXTERNAL1))

#define IS_PWM_MSM_STATE(STATE) (((STATE) == PWM_MASTERSLAVEMODE_ENABLE) || \
                                 ((STATE) == PWM_MASTERSLAVEMODE_DISABLE))

#define IS_PWM_TRIGGER_SELECTION(SELECTION) (((SELECTION) == PWM_TS_ITR0) || \
                                             ((SELECTION) == PWM_TS_ITR1) || \
                                             ((SELECTION) == PWM_TS_ITR2) || \
                                             ((SELECTION) == PWM_TS_ITR3) || \
                                             ((SELECTION) == PWM_TS_TI1F_ED) || \
                                             ((SELECTION) == PWM_TS_TI1FP1) || \
                                             ((SELECTION) == PWM_TS_TI2FP2) || \
                                             ((SELECTION) == PWM_TS_ETRF))

#define IS_PWM_INTERNAL_TRIGGEREVENT_SELECTION(SELECTION) (((SELECTION) == PWM_TS_ITR0) || \
                                                           ((SELECTION) == PWM_TS_ITR1) || \
                                                           ((SELECTION) == PWM_TS_ITR2) || \
                                                           ((SELECTION) == PWM_TS_ITR3) || \
                                                           ((SELECTION) == PWM_TS_NONE))

#define IS_PWM_TRIGGERPOLARITY(POLARITY)     (((POLARITY) == PWM_TRIGGERPOLARITY_INVERTED   ) || \
                                              ((POLARITY) == PWM_TRIGGERPOLARITY_NONINVERTED) || \
                                              ((POLARITY) == PWM_TRIGGERPOLARITY_RISING     ) || \
                                              ((POLARITY) == PWM_TRIGGERPOLARITY_FALLING    ) || \
                                              ((POLARITY) == PWM_TRIGGERPOLARITY_BOTHEDGE   ))

#define IS_PWM_TRIGGERPRESCALER(PRESCALER)  (((PRESCALER) == PWM_TRIGGERPRESCALER_DIV1) || \
                                             ((PRESCALER) == PWM_TRIGGERPRESCALER_DIV2) || \
                                             ((PRESCALER) == PWM_TRIGGERPRESCALER_DIV4) || \
                                             ((PRESCALER) == PWM_TRIGGERPRESCALER_DIV8))

#define IS_PWM_TRIGGERFILTER(ICFILTER)     ((ICFILTER) <= 0x0FU)

#define IS_PWM_TI1SELECTION(TI1SELECTION)   (((TI1SELECTION) == PWM_TI1SELECTION_CH1) || \
                                             ((TI1SELECTION) == PWM_TI1SELECTION_XORCOMBINATION))

#define IS_PWM_DMA_BASE(BASE) (((BASE) == PWM_DMABASE_CR1) || \
                               ((BASE) == PWM_DMABASE_CR2) || \
                               ((BASE) == PWM_DMABASE_SMCR) || \
                               ((BASE) == PWM_DMABASE_DIER) || \
                               ((BASE) == PWM_DMABASE_SR) || \
                               ((BASE) == PWM_DMABASE_EGR) || \
                               ((BASE) == PWM_DMABASE_CCMR1) || \
                               ((BASE) == PWM_DMABASE_CCMR2) || \
                               ((BASE) == PWM_DMABASE_CCER) || \
                               ((BASE) == PWM_DMABASE_CNT) || \
                               ((BASE) == PWM_DMABASE_PSC) || \
                               ((BASE) == PWM_DMABASE_ARR) || \
                               ((BASE) == PWM_DMABASE_RCR) || \
                               ((BASE) == PWM_DMABASE_CCR1) || \
                               ((BASE) == PWM_DMABASE_CCR2) || \
                               ((BASE) == PWM_DMABASE_CCR3) || \
                               ((BASE) == PWM_DMABASE_CCR4) || \
                               ((BASE) == PWM_DMABASE_BDTR) || \
                               ((BASE) == PWM_DMABASE_DCR) || \
                               ((BASE) == PWM_DMABASE_OR))

#define IS_PWM_DMA_LENGTH(LENGTH) (((LENGTH) == PWM_DMABURSTLENGTH_1TRANSFER) || \
                                   ((LENGTH) == PWM_DMABURSTLENGTH_2TRANSFERS) || \
                                   ((LENGTH) == PWM_DMABURSTLENGTH_3TRANSFERS) || \
                                   ((LENGTH) == PWM_DMABURSTLENGTH_4TRANSFERS) || \
                                   ((LENGTH) == PWM_DMABURSTLENGTH_5TRANSFERS) || \
                                   ((LENGTH) == PWM_DMABURSTLENGTH_6TRANSFERS) || \
                                   ((LENGTH) == PWM_DMABURSTLENGTH_7TRANSFERS) || \
                                   ((LENGTH) == PWM_DMABURSTLENGTH_8TRANSFERS) || \
                                   ((LENGTH) == PWM_DMABURSTLENGTH_9TRANSFERS) || \
                                   ((LENGTH) == PWM_DMABURSTLENGTH_10TRANSFERS) || \
                                   ((LENGTH) == PWM_DMABURSTLENGTH_11TRANSFERS) || \
                                   ((LENGTH) == PWM_DMABURSTLENGTH_12TRANSFERS) || \
                                   ((LENGTH) == PWM_DMABURSTLENGTH_13TRANSFERS) || \
                                   ((LENGTH) == PWM_DMABURSTLENGTH_14TRANSFERS) || \
                                   ((LENGTH) == PWM_DMABURSTLENGTH_15TRANSFERS) || \
                                   ((LENGTH) == PWM_DMABURSTLENGTH_16TRANSFERS) || \
                                   ((LENGTH) == PWM_DMABURSTLENGTH_17TRANSFERS) || \
                                   ((LENGTH) == PWM_DMABURSTLENGTH_18TRANSFERS))


/**
 * @}
 */

/**
 * @}
 */

/* Private functions ---------------------------------------------------------*/
/** @defgroup PWM_Private_Functions TIM Private Functions
 * @{
 */
void PWM_SetConfig(PWM_ChannelTypeDef *Channel, PWM_InitTypeDef *Structure);
void PWM_DMAError(DMA_HandleTypeDef *hdma);
void PWM_CCxChannelCmd(PWM_TypeDef *PWMx, uint32_t Channel, uint32_t ChannelState);
/**
 * @}
 */

#define IS_PWM_INSTANCE(__INSTANCE__)   ((__INSTANCE__) == hwp_pwm1)


/**
 * @}
 */

/**
 * @}
 */


#ifdef __cplusplus
}
#endif

#endif /* __BF0_HAL_PWM_H */