/**
  ******************************************************************************
  * @file   bf0_hal_sdadc.h
  * @author Sifli software development team
  * @brief   Header file containing functions prototypes of ADC HAL library.
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

#ifndef __BF0_HAL_SDADC_H
#define __BF0_HAL_SDADC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal_def.h"
#include "sdadc.h"
/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @addtogroup SDADC Segma Delta Analog Digital Converter
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup SDADC_Exported_Types SDADC Exported Types
  * @{
  */

/**
  * @brief  ENUM definition of Timer source for timer triger
  * @note   All timer is hardware timer id.
  */
typedef enum
{
    HAL_SDADC_SRC_GPTIM3 = 0,      /*!< SDADC triger source gptim 3  */
    HAL_SDADC_SRC_GPTIM4,           /*!< SDADC triger source gptim 4  */
    HAL_SDADC_SRC_GPTIM5,           /*!< SDADC triger source gptim 5  */
    HAL_SDADC_SRC_BTIM3,            /*!< SDADC triger source bttim 3  */
    HAL_SDADC_SRC_BTIM4,            /*!< SDADC triger source bttim 3  */
    HAL_SDADC_SRC_TIMER_CNT         /*!< SDADC triger source counter  */
} HAL_SDADC_SRC_TIME_T;

/**
  * @brief  ENUM definition of triger source
  * @note   GPIO triger is not work for current version.
  */
typedef enum
{
    HAL_SDADC_SRC_SW = 0,           /*!< SDADC triger source by software  */
    HAL_SDADC_SRC_GPIO = 1,         /*!< SDADC triger source by gpio  */
    HAL_SDADC_SRC_TIMER = 2         /*!< SDADC triger source by timer  */
} HAL_SDADC_SRC__T;

/**
  * @brief  ENUM definition of voltage reference type
  */
typedef enum
{
    HAL_SDADC_VREF_EXTERNAL = 0,           /*!< VREF use outside , only for BGA  */
    HAL_SDADC_VERF_INTERNAL = 1,         /*!< VREF with internal, only for BGA  */
    HAL_SDADC_VREF_POWER = 2         /*!< VREF with power supply  */
} HAL_SDADC_VREF_SEL_T;

/**
  * @brief  ENUM definition of continue and dsample mode
  */
typedef enum
{
    HAL_SDADC_CONTIN_MODE = 0,      /*!< SDADC continue transfer mode  */
    HAL_SDADC_SINGLE_MODE = 1,      /*!< SDADC transfer once  */
    HAL_SDADC_DSAMPLE_MODE = 2      /*!< SDADC dsample? */
} HAL_SDADC_CONTIN_MODE_T;

/**
  * @brief  ENUM definition of channel select
  * @note   This enum for pch define, for nch, it do not include input, it's value is pch -1.
  */
typedef enum
{
    HAL_SDADC_CHN_INPUT = 0,        /*!< SDADC input channel for p channel, it only for BGA  */
    HAL_SDADC_GPIO_CHN0 = 1,        /*!< SDADC input gpio channel 0  */
    HAL_SDADC_GPIO_CHN1 = 2,        /*!< SDADC input gpio channel 1  */
    HAL_SDADC_GPIO_CHN2 = 3,        /*!< SDADC input gpio channel 2  */
    HAL_SDADC_GPIO_CHN3 = 4,        /*!< SDADC input gpio channel 3  */
} HAL_SDADC_GPIO_CHN_T;



/**
  * @brief  Structure definition of SDADC initialization and regular group
  * @note   The setting of these parameters with function HAL_SDADC_Init() is conditioned to SDADC state.
  */
typedef struct
{
    uint8_t adc_se;  /*!<  single channel select */
    uint8_t src_sel;  /*!<  triger source, HAL_SDADC_SRC__T */
    uint8_t vref_sel;  /*!<  voltage reference seltect, HAL_SDADC_VREF_SEL_T */
    uint8_t dma_en;     /*!<  Enable dma */
    uint8_t conti_mode; /*!<  0 once , 1 continuous mode */
    uint8_t dsample_mode; /*!<  0 single channel, 1 multi channel */
    uint8_t diff_sel; /*!<   0x0: single end  0x1:differencial*/
    uint8_t en_slot;   /*!<  Which slot been enable, 1 bit for each slot */
} SDADC_InitTypeDef;

/**
  * @brief  Structure definition of SDADC channel for regular group
  * @note   The setting of these parameters with function HAL_ADC_ConfigChannel() is conditioned to ADC state.
  */
typedef struct
{
    uint8_t Channel;               /*!<  channel index, 0 ~ 4 */
    uint8_t shift_num;             /*!<  Need get average pixel */
    uint8_t nchnl_sel;           /*!<  channel index, 0 ~ 3 , do not used if set single channel*/
    uint8_t pchnl_sel;           /*!<  channel index, HAL_SDADC_GPIO_CHN_T */
    uint8_t slot_en;             /*!<  Set to 1 if enanble slot*/
} SDADC_ChannelConfTypeDef;

/**
  * @brief  Structure definition of SDADC accurate for regular group
  * @note   The setting of these parameters with function HAL_SDADC_ConfigAccu() is conditioned to SDADC state.
  */
typedef struct
{
    uint16_t chop1_num;      /*!<  chop num 1 */
    uint16_t chop2_num;     /*!<  chop num 2 */
    uint16_t chop3_num;     /*!<  chop num 3 */
    uint16_t chop_ref_num;      /*!<  chop reference number */
    uint16_t sample_num;        /*!<  sample number */
} SDADC_AccurateConfTypeDef;

/**
  * @brief  Structure definition of SDADC gain for regular group
  * @note   The setting of these parameters with function HAL_SDADC_ConfigGain() is conditioned to SDADC state.
  */
typedef struct
{
    uint8_t gain_deno;      /*!<  gain denominator */
    uint8_t gain_nume;      /*!<  gain numerator */
} SDADC_GainConfTypeDef;

/**
  * @brief  HAL SDADC state machine: SDADC states definition (bitfields)
  * @note   ADC state machine is managed by bitfields, state must be compared
  *         with bit by bit.
  *         For example:
  *           " if (HAL_IS_BIT_SET(HAL_ADC_GetState(hadc1), HAL_ADC_STATE_REG_BUSY)) "
  *           " if (HAL_IS_BIT_SET(HAL_ADC_GetState(hadc1), HAL_ADC_STATE_AWD1)    ) "
  */
/* States of SDADC global scope */
#define HAL_SDADC_STATE_RESET             (0x00000000U)    /*!< ADC not yet initialized or disabled */
#define HAL_SDADC_STATE_READY             (0x00000001U)    /*!< ADC peripheral ready for use */
#define HAL_SDADC_STATE_BUSY_INTERNAL     (0x00000002U)    /*!< ADC is busy to internal process (initialization, calibration) */
#define HAL_SDADC_STATE_TIMEOUT           (0x00000004U)    /*!< TimeOut occurrence */

/* States of SDADC errors */
#define HAL_SDADC_STATE_ERROR_INTERNAL    (0x00000010U)    /*!< Internal error occurrence */
#define HAL_SDADC_STATE_ERROR_CONFIG      (0x00000020U)    /*!< Configuration error occurrence */
#define HAL_SDADC_STATE_ERROR_DMA         (0x00000040U)    /*!< DMA error occurrence */

/* States of SDADC group regular */
#define HAL_SDADC_STATE_REG_BUSY          (0x00000100U)    /*!< A conversion on group regular is ongoing or can occur (either by continuous mode,
                                                                       external trigger, low power auto power-on, multimode ADC master control) */
#define HAL_SDADC_STATE_REG_EOC           (0x00000200U)    /*!< Conversion data available on group regular */
#define HAL_SDADC_STATE_REG_OVR           (0x00000400U)    /*!< Overrun occurrence */

#define HAL_SDADC_START_IRQ_DONE          (0x00001000)        /*!< Conversion finish */

/**
  * @brief  SDADC handle Structure definition
  */
typedef struct
{
    SDADC_TypeDef                   *Instance;              /*!< Register base address */

    SDADC_InitTypeDef               Init;                   /*!< ADC required parameters */

    DMA_HandleTypeDef             *DMA_Handle;            /*!< Pointer DMA Handler */

    HAL_LockTypeDef               Lock;                   /*!< ADC locking object */

    __IO uint32_t                 State;                  /*!< ADC communication state (bitmap of ADC states) */

    __IO uint32_t                 ErrorCode;              /*!< ADC Error code */
} SDADC_HandleTypeDef;
/**
  * @}
  */



/* Exported constants --------------------------------------------------------*/

/** @defgroup SDADC_Exported_Constants SDADC Exported Constants
  * @{
  */

/** @defgroup SDADC_Error_Code SDADC Error Code
  * @{
  */
#define HAL_SDADC_ERROR_NONE        (0x00U)   /*!< No error                                              */
#define HAL_SDADC_ERROR_INTERNAL    (0x01U)   /*!< ADC IP internal error: if problem of clocking, 
                                                          enable/disable, erroneous state                       */
#define HAL_SDADC_ERROR_OVR         (0x02U)   /*!< Overrun error                                         */
#define HAL_SDADC_ERROR_DMA         (0x04U)   /*!< DMA transfer error                                    */

/**
  * @}
  */



/**
  * @}
  */


/* Private constants ---------------------------------------------------------*/

/** @addtogroup SDADC_Private_Constants SDADC Private Constants
  * @{
  */


/* Exported macro ------------------------------------------------------------*/

/** @defgroup SDADC_Exported_Macros SDADC Exported Macros
  * @{
  */

/**
  * @brief Get the selected SDADC's flag status.
  * @param  \__HANDLE__ SDADC handle
  * @param  \__FLAG__ SDADC flag
  * @retval None
  */
#define __HAL_SDADC_GET_FLAG(__HANDLE__, __FLAG__)                               \
  ((((__HANDLE__)->Instance->INT_ST) & (__FLAG__)) == (__FLAG__))

/**
  * @brief Clear the ASDDC's pending flags
  * @param  \__HANDLE__ SDADC handle
  * @param  \__FLAG__ SDADC flag
  * @retval None
  */
/* Note: bit cleared bit by writing 1 (writing 0 has no effect on any bit of register ISR, ONLY 1 BIT WORK) */
#define __HAL_SDADC_CLEAR_FLAG(__HANDLE__, __FLAG__)                             \
  (((__HANDLE__)->Instance->INT_CLR) |= (1))

/**
  * @brief Enable SDADC irq
  * @param  \__HANDLE__ SDADC handle
  * @param  \__FLAG__ SDADC irq flag
  * @retval None
  */
#define __HAL_SDADC_ENABLE_IRQ(__HANDLE__, __FLAG__)  ((__HANDLE__)->Instance->INT_MSK &= ~(__FLAG__))

/**
  * @brief Disable SDADC irq
  * @param  \__HANDLE__ SDADC handle
  * @param  \__FLAG__ SDADC irq flag
  * @retval None
  */
#define __HAL_SDADC_DISABLE_IRQ(__HANDLE__, __FLAG__)  ((__HANDLE__)->Instance->INT_MSK |= (__FLAG__))


/** @brief  Reset SDADC handle state
  * @param  \__HANDLE__ SDADC handle
  * @retval None
  */
#define __HAL_SDADC_RESET_HANDLE_STATE(__HANDLE__)                               \
  ((__HANDLE__)->State = HAL_SDADC_STATE_RESET)


/** @brief  Start SDADC software convert
  * @param  \__HANDLE__ SDADC handle
  * @retval None
  */
#define __HAL_SDADC_START_SW_CONV(__HANDLE__) ((__HANDLE__)->Instance->TRIG |= SDADC_TRIG_ADC_START)

/** @brief  Start SDADC GPIO convert
  * @param  \__HANDLE__ SDADC handle
  * @retval None
  */
#define __HAL_SDADC_START_GPIO_CONV(__HANDLE__) ((__HANDLE__)->Instance->TRIG |= SDADC_TRIG_GPIO_TRIG_EN)

/** @brief  Start SDADC timer convert
  * @param  \__HANDLE__ SDADC handle
  * @retval None
  */
#define __HAL_SDADC_START_TIMER_CONV(__HANDLE__) ((__HANDLE__)->Instance->TRIG |= SDADC_TRIG_TIMER_TRIG_EN)




/* Private macro -------------------------------------------------------------*/

/** @defgroup SDADC_Private_Macros SDADC Private Macros
  * @{
  */
/* Macro reserved for internal HAL driver usage, not intended to be used in   */
/* code of final user.                                                        */


/**
  * @brief Verification of SDADC state: enabled or disabled
  * @param  \__HANDLE__ SDADC handle
  * @retval SET (SDADC enabled) or RESET (SDADC disabled)
  */
/* Note: If low power mode AutoPowerOff is enabled, power-on/off phases are   */
/*       performed automatically by hardware and flag ADC_FLAG_RDY is not     */
/*       set.                                                                 */
#define SDADC_IS_ENABLE(__HANDLE__)                                                       \
  (( (((__HANDLE__)->Instance->CH0_CFG) & SDADC_CH0_CFG_EN) |  \
  (((__HANDLE__)->Instance->CH1_CFG) & SDADC_CH1_CFG_EN) |  \
  (((__HANDLE__)->Instance->CH2_CFG) & SDADC_CH2_CFG_EN) |  \
  (((__HANDLE__)->Instance->CH3_CFG) & SDADC_CH3_CFG_EN) |  \
  (((__HANDLE__)->Instance->CH4_CFG) & SDADC_CH4_CFG_EN) |  \
   ) ? SET : RESET)

/**
  * @brief Simultaneously clears and sets specific bits of the handle State
  * @note: SDADC_STATE_CLR_SET() macro is merely aliased to generic macro MODIFY_REG(),
  *        the first parameter is the ADC handle State, the second parameter is the
  *        bit field to clear, the third and last parameter is the bit field to set.
  * @retval None
  */
#define SDADC_STATE_CLR_SET MODIFY_REG

/**
  * @brief Clear SDADC error code (set it to error code: "no error")
  * @param  \__HANDLE__ SDADC handle
  * @retval None
  */
#define SDADC_CLEAR_ERRORCODE(__HANDLE__)                                        \
  ((__HANDLE__)->ErrorCode = HAL_SDADC_ERROR_NONE)


/**
  * @brief Configure the channel number into channel selection register
  * @param \_CHANNEL_ SDADC Channel
  * @retval None
  */
/* This function converts SDADC channels from numbers (see defgroup SDADC_channels)
   to bitfields, to get the equivalence of CMSIS channels:
        ADC_CHANNEL_0           ((uint32_t) ADC_CHSELR_CHSEL0)
        ADC_CHANNEL_1           ((uint32_t) ADC_CHSELR_CHSEL1)
        ADC_CHANNEL_2           ((uint32_t) ADC_CHSELR_CHSEL2)
        ADC_CHANNEL_3           ((uint32_t) ADC_CHSELR_CHSEL3)
        ADC_CHANNEL_4           ((uint32_t) ADC_CHSELR_CHSEL4)
*/
#define SDADC_CHSELR_CHANNEL(_CHANNEL_)                                          \
  ( 1U << (_CHANNEL_))


/**
  * @brief Enable SDADC continuous conversion mode.
  * @param \__HANDLE__ SDADC handle.
  * @param \_CONTINUOUS_MODE_ Continuous mode.
  * @retval None
  */
#define SDADC_CFGR_CONTINUOUS(__HANDLE__, _CONTINUOUS_MODE_)                                \
  (((__HANDLE__)->Instance->CFG0) |= (_CONTINUOUS_MODE_) << SDADC_CFG0_CONTIN_LV_Pos)


/**
  * @brief  Enable the selected ADC.
  * @note   Prerequisite condition to use this function: ADC must be disabled
  *         and voltage regulator must be enabled (done into HAL_ADC_Init()).
  * @note   If low power mode AutoPowerOff is enabled, power-on/off phases are
  *         performed automatically by hardware.
  *         In this mode, this function is useless and must not be called because
  *         flag ADC_FLAG_RDY is not usable.
  *         Therefore, this function must be called under condition of
  *         "if (hadc->Init.LowPowerAutoPowerOff != ENABLE)".
  * @param  \__HANDLE__ ADC handle
  * @retval None.
  */
#define SDADC_Enable(__HANDLE__)((  __HANDLE__)->Instance->CFG0 |= SDADC_CFG0_PU_LV)

/**
  * @brief  Disable the selected ADC.
  * @note   Prerequisite condition to use this function: ADC conversions must be
  *         stopped.
  * @param  \__HANDLE__ ADC handle
  * @retval None.
  */
#define SDADC_Disable(__HANDLE__)       ((__HANDLE__)->Instance->CFG0 &= ~SDADC_CFG0_PU_LV)


/**
  * @brief Enable SDADC DMA mode.
  * @param \__HANDLE__ SDADC handle.
  * @retval None
  */
#define SDADC_DMA_ENABLE(__HANDLE__)      ((__HANDLE__)->Instance->DMA_CFG |= SDADC_DMA_CFG_RX_DMA_MSK)


/**
  * @brief Disable SDADC DMA mode.
  * @param \__HANDLE__ SDADC handle.
  * @retval None
  */
#define SDADC_DMA_DISABLE(__HANDLE__)      ((__HANDLE__)->Instance->DMA_CFG &= (~SDADC_DMA_CFG_RX_DMA_MSK))

/**
  * @brief Check if timer triger enable.
  * @param \__HANDLE__ SDADC handle.
  * @retval Not 0 if timer triger enable
  */
#define IS_SDADC_SRC_TIMER(__HANDLE__)    ((__HANDLE__)->Instance->TRIG & SDADC_TRIG_TIMER_TRIG_EN)

/**
  * @brief Enable timer triger.
  * @param \__HANDLE__ SDADC handle.
  * @retval None
  */
#define SDADC_ENABLE_TIMER_TRIGER(__HANDLE__)    ((__HANDLE__)->Instance->TRIG |= SDADC_TRIG_TIMER_TRIG_EN)

/**
  * @brief Enable timer triger.
  * @param \__HANDLE__ SDADC handle.
  * @retval None
  */
#define SDADC_DISABLE_TIMER_TRIGER(__HANDLE__)    ((__HANDLE__)->Instance->TRIG &= (~SDADC_TRIG_TIMER_TRIG_EN))

/**
  * @brief Enable GPIO triger.
  * @param \__HANDLE__ SDADC handle.
  * @retval None
  */
#define SDADC_ENABLE_GPIO_TRIGER(__HANDLE__)    ((__HANDLE__)->Instance->TRIG |= SDADC_TRIG_GPIO_TRIG_EN)

/**
  * @brief Enable GPIO triger.
  * @param \__HANDLE__ SDADC handle.
  * @retval None
  */
#define SDADC_DISABLE_GPIO_TRIGER(__HANDLE__)    ((__HANDLE__)->Instance->TRIG &= (~SDADC_TRIG_GPIO_TRIG_EN))


/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */


/* Exported functions --------------------------------------------------------*/
/** @addtogroup SDADC_Exported_Functions
  * @{
  */

/** @addtogroup SDADC_Exported_Functions_Group1
  * @{
  */


/* Initialization and de-initialization functions  **********************************/

/**
  * @brief  Initializes the SDADC peripheral and regular group according to
  *         parameters specified in structure "ADC_InitTypeDef".
  * @note   As prerequisite, SDADC clock must be configured at RCC top level
  *         depending on both possible clock sources: APB clock of HSI clock.
  * @note   Possibility to update parameters on the fly:
  *         The setting of these parameters is conditioned to ADC state.
  *         For parameters constraints, see comments of structure
  *         "ADC_InitTypeDef".
  * @note   This function configures the ADC within 2 scopes: scope of entire
  *         ADC and scope of regular group. For parameters details, see comments
  *         of structure "SDADC_InitTypeDef".
  * @param  hadc SDADC handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SDADC_Init(SDADC_HandleTypeDef *hadc);

/**
  * @brief Deinitial SDADC hardware.
  * @param hadc SDADC handle.
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef HAL_SDADC_DeInit(SDADC_HandleTypeDef *hadc);

/**
  * @}
  */

/* IO operation functions  *****************************************************/

/** @addtogroup SDADC_Exported_Functions_Group2
  * @{
  */


/* Blocking mode: Polling */


/**
  * @brief  Enables SDADC, starts conversion of regular group.
  *         Interruptions enabled in this function: None.
  * @param  hadc SDADC handle
  * @retval HAL status
  */
HAL_StatusTypeDef       HAL_SDADC_Start(SDADC_HandleTypeDef *hadc);

/**
  * @brief  Stop SDADC conversion of regular group, disable ADC peripheral.
  * @param  hadc SDADC handle
  * @retval HAL status.
  */
HAL_StatusTypeDef       HAL_SDADC_Stop(SDADC_HandleTypeDef *hadc);

/**
  * @brief  Wait for regular group conversion to be completed.
  * @param  hadc ADC handle
  * @param  Timeout Timeout value in millisecond.
  * @retval HAL status
  */
HAL_StatusTypeDef       HAL_SDADC_PollForConversion(SDADC_HandleTypeDef *hadc, uint32_t Timeout);

/* Non-blocking mode: Interruption */

/**
  * @brief Start SDADC with IRQ mode.
  * @param hadc SDADC handle.
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef       HAL_SDADC_Start_IT(SDADC_HandleTypeDef *hadc);

/**
  * @brief Stop SDADC irq mode.
  * @param hadc SDADC handle.
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef       HAL_SDADC_Stop_IT(SDADC_HandleTypeDef *hadc);

/* Non-blocking mode: DMA */

/**
  * @brief start SDADC convert with dma mode.
  * @param hadc SDADC handle.
  * @param pData data buffer.
  * @param Length data lenght.
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef       HAL_SDADC_Start_DMA(SDADC_HandleTypeDef *hadc, uint32_t *pData, uint32_t Length);

/**
  * @brief Stop SDADC DMA transmit.
  * @param hadc SDADC handle.
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef       HAL_SDADC_Stop_DMA(SDADC_HandleTypeDef *hadc);

/* ADC retrieve conversion value intended to be used with polling or interruption */
/**
  * @brief Get SDADC convert result.
  * @param hadc SDADC handle.
  * @param slot SDADC slot number.
  * @retval ADC convert result.
  */
uint32_t                HAL_SDADC_GetValue(SDADC_HandleTypeDef *hadc, uint32_t slot);

/**
  * @brief Set SDADC triger mode.
  * @param hadc SDADC handle.
  * @param src triger source mode.
  * @retval HAL_StatusTypeDef.
  */
HAL_StatusTypeDef       HAL_SDADC_SetSource(SDADC_HandleTypeDef *hadc, HAL_SDADC_SRC__T src);

/**
  * @brief set SDADC timer triger mode.
  * @param hadc SDADC handle.
  * @param src timer index.
  * @retval HAL_StatusTypeDef.
  */
HAL_StatusTypeDef       HAL_SDADC_SetTimer(SDADC_HandleTypeDef *hadc, HAL_SDADC_SRC_TIME_T src);

/**
  * @brief Enable/Disable SDADC slot/channel.
  * @param hadc SDADC handle.
  * @param slot SDADC slot number.
  * @param en enable 1 for disable 0.
  * @retval HAL_StatusTypeDef.
  */
HAL_StatusTypeDef HAL_SDADC_EnableSlot(SDADC_HandleTypeDef *hadc, uint32_t slot, uint8_t en);

/**
  * @}
  */


/* Peripheral Control functions ***********************************************/
/** @addtogroup SDADC_Exported_Functions_Group3
  * @{
  */

/**
  * @brief  Configures the the selected channel to be linked to the regular
  *         group.
  * @note   In case of usage of internal measurement channels:
  *         VrefInt/Vbat/TempSensor.
  *         Sampling time constraints must be respected (sampling time can be
  *         adjusted in function of SDADC clock frequency and sampling time
  *         setting).
  *         Refer to device datasheet for timings values, parameters TS_vrefint,
  *         TS_vbat, TS_temp (values rough order: 5us to 17us).
  *         These internal paths can be be disabled using function
  *         HAL_ADC_DeInit().
  * @note   Possibility to update parameters on the fly:
  *         This function initializes channel into regular group, following
  *         calls to this function can be used to reconfigure some parameters
  *         of structure "SDADC_ChannelConfTypeDef" on the fly, without reseting
  *         the ADC.
  *         The setting of these parameters is conditioned to ADC state.
  *         For parameters constraints, see comments of structure
  *         "ADC_ChannelConfTypeDef".
  * @param  hadc ADC handle
  * @param  sConfig Structure of SDADC channel for regular group.
  * @retval HAL status
  */
HAL_StatusTypeDef       HAL_SDADC_ConfigChannel(SDADC_HandleTypeDef *hadc, SDADC_ChannelConfTypeDef *sConfig);

/**
  * @brief SET SDADC accurate configure.
  * @param hadc SDADC handle.
  * @param sConfig configure structure pointer.
  * @retval HAL status.
  */
HAL_StatusTypeDef       HAL_SDADC_ConfigAccu(SDADC_HandleTypeDef *hadc, SDADC_AccurateConfTypeDef *sConfig);

/**
  * @brief Set SDADC gain configure.
  * @param hadc SDADC handle.
  * @param sConfig configure structure pointer.
  * @retval HAL status.
  */
HAL_StatusTypeDef       HAL_SDADC_ConfigGain(SDADC_HandleTypeDef *hadc, SDADC_GainConfTypeDef *sConfig);

/**
  * @}
  */


/* Peripheral State functions *************************************************/
/** @addtogroup SDADC_Exported_Functions_Group4
  * @{
  */

/**
  * @brief Get SDADC status.
  * @param hadc SDADC handle.
  * @retval current status.
  */
uint32_t                HAL_SDADC_GetState(SDADC_HandleTypeDef *hadc);

/**
  * @brief Get SDADC error code.
  * @param hadc SDADC handle.
  * @retval current error code.
  */
uint32_t                HAL_SDADC_GetError(SDADC_HandleTypeDef *hadc);
/**
  * @}
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


#endif /* __BF0_HAL_SDADC_H */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

