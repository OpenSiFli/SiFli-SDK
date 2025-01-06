/**
  ******************************************************************************
  * @file   bf0_hal_adc.h
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

#ifndef __BF0_HAL_ADC_H
#define __BF0_HAL_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal_def.h"
#include "gpadc.h"
/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @addtogroup ADC Analog Digital Converter
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup ADC_Exported_Types ADC Exported Types
  * @{
  */

/**
  * @brief  ENUM definition of Timer source for timer triger
  * @note   All timer is hardware timer id.
  */
typedef enum
{
    HAL_ADC_SRC_GPTIM1 = 0,
    HAL_ADC_SRC_GPTIM2,
    HAL_ADC_SRC_GPTIM3,
    HAL_ADC_SRC_BTIM1,
    HAL_ADC_SRC_BTIM2,
    HAL_ADC_SRC_TIMER_CNT
} HAL_ADC_SRC_TIME_T;

/**
  * @brief  ENUM definition of triger source
  * @note   GPIO triger is not work for current version.
  */
typedef enum
{
    HAL_ADC_SRC_SW = 0,
    HAL_ADC_SRC_GPIO = 1,
    HAL_ADC_SRC_TIMER = 2
} HAL_ADC_SRC__T;


/**
  * @brief  Structure definition of ADC initialization and regular group
  * @note   The setting of these parameters with function HAL_ADC_Init() is conditioned to ADC state.
  *         ADC state can be either:
  *          - For all parameters: ADC disabled (this is the only possible ADC state to modify parameter 'ClockPrescaler')
  *          - For all parameters except 'ClockPrescaler' and 'resolution': ADC enabled without conversion on going on regular group.
  *         If ADC is not in the appropriate state to modify some parameters, these parameters setting is bypassed
  *         without error reporting (as it can be the expected behaviour in case of intended action to update another parameter (which fulfills the ADC state condition) on the fly).
  */
typedef struct
{
    uint8_t adc_se;  /* single end select */
    uint8_t adc_force_on; /* after force_en set to 1, use ADC_CFG_REG1 FOR channel configure */
    uint8_t dma_en;     /* Enable dma */
    uint8_t atten3;     /* Attenuate input voltage by 3 for GPADC */
    uint8_t op_mode;  /* 0 finite, 1 infinite = continuous mode */
    uint8_t en_slot;    /* 0 for fix slot mode(fix use slot 0), 1 for multi slot mode */
#ifndef SF32LB55X
    uint8_t data_samp_delay;     /* delay cycle after analog conver done */
    uint32_t conv_width: 8; /* cycle counter for ana to convert data */
    uint32_t sample_width: 24; /* cycle counter for ana to convert data */
    uint8_t  avdd_v18_en;   /* 1 for avdd 1.8v, 0 for avdd 3.3v*/
#else
    uint8_t clk_div;    /* clock divider */
#endif
} ADC_InitTypeDef;

/**
  * @brief  Structure definition of ADC channel for regular group
  * @note   The setting of these parameters with function HAL_ADC_ConfigChannel() is conditioned to ADC state.
  *         ADC state can be either:
  *          - For all parameters: ADC disabled or enabled without conversion on going on regular group.
  *         If ADC is not in the appropriate state to modify some parameters, these parameters setting is bypassed
  *         without error reporting (as it can be the expected behaviour in case of intended action to update another parameter (which fulfills the ADC state condition) on the fly).
  */
typedef struct
{
    uint8_t Channel;                /* channel index, 0 ~ 7 */
    uint8_t acc_num;                 /* Set to 0 if disable slot*/
    uint8_t acc_en;                 /* Not used for current version */
    uint8_t nchnl_sel;           /* channel index, 0 ~ 7 , do not used if set single channel*/
    uint8_t pchnl_sel;          /* channel index, 0 ~ 7 */
    uint8_t slot_en;            /* Set to 1 if enanble slot*/
} ADC_ChannelConfTypeDef;


/**
  * @brief  HAL ADC state machine: ADC states definition (bitfields)
  * @note   ADC state machine is managed by bitfields, state must be compared
  *         with bit by bit.
  *         For example:
  *           " if (HAL_IS_BIT_SET(HAL_ADC_GetState(hadc1), HAL_ADC_STATE_REG_BUSY)) "
  *           " if (HAL_IS_BIT_SET(HAL_ADC_GetState(hadc1), HAL_ADC_STATE_AWD1)    ) "
  */
/* States of ADC global scope */
#define HAL_ADC_STATE_RESET             (0x00000000U)    /*!< ADC not yet initialized or disabled */
#define HAL_ADC_STATE_READY             (0x00000001U)    /*!< ADC peripheral ready for use */
#define HAL_ADC_STATE_BUSY_INTERNAL     (0x00000002U)    /*!< ADC is busy to internal process (initialization, calibration) */
#define HAL_ADC_STATE_TIMEOUT           (0x00000004U)    /*!< TimeOut occurrence */

/* States of ADC errors */
#define HAL_ADC_STATE_ERROR_INTERNAL    (0x00000010U)    /*!< Internal error occurrence */
#define HAL_ADC_STATE_ERROR_CONFIG      (0x00000020U)    /*!< Configuration error occurrence */
#define HAL_ADC_STATE_ERROR_DMA         (0x00000040U)    /*!< DMA error occurrence */

/* States of ADC group regular */
#define HAL_ADC_STATE_REG_BUSY          (0x00000100U)    /*!< A conversion on group regular is ongoing or can occur (either by continuous mode,
                                                                       external trigger, low power auto power-on, multimode ADC master control) */
#define HAL_ADC_STATE_REG_EOC           (0x00000200U)    /*!< Conversion data available on group regular */
#define HAL_ADC_STATE_REG_OVR           (0x00000400U)    /*!< Overrun occurrence */

#define HAL_ADC_START_IRQ_DONE          (0x00001000)


/**
  * @brief  ADC handle Structure definition
  */
typedef struct
{
    GPADC_TypeDef                   *Instance;              /*!< Register base address */

    ADC_InitTypeDef               Init;                   /*!< ADC required parameters */

    DMA_HandleTypeDef             *DMA_Handle;            /*!< Pointer DMA Handler */

    HAL_LockTypeDef               Lock;                   /*!< ADC locking object */

    __IO uint32_t                 State;                  /*!< ADC communication state (bitmap of ADC states) */

    __IO uint32_t                 ErrorCode;              /*!< ADC Error code */
} ADC_HandleTypeDef;
/**
  * @}
  */



/* Exported constants --------------------------------------------------------*/

/** @defgroup ADC_Exported_Constants ADC Exported Constants
  * @{
  */

/** @defgroup ADC_Error_Code ADC Error Code
  * @{
  */
#define HAL_ADC_ERROR_NONE        (0x00U)   /*!< No error                                              */
#define HAL_ADC_ERROR_INTERNAL    (0x01U)   /*!< ADC IP internal error: if problem of clocking, 
                                                          enable/disable, erroneous state                       */
#define HAL_ADC_ERROR_OVR         (0x02U)   /*!< Overrun error                                         */
#define HAL_ADC_ERROR_DMA         (0x04U)   /*!< DMA transfer error                                    */

/**
  * @}
  */


/** @defgroup ADC_Resolution ADC Resolution
  * @{
  */
#define ADC_RESOLUTION_10B      (0x00000000U)           /*!<  ADC 10-bit resolution */

/**
  * @}
  */

/** @defgroup ADC_Data_align ADC Data_align
  * @{
  */
#define ADC_DATAALIGN_RIGHT      (0x00000000U)

/**
  * @}
  */

/** @defgroup ADC_Scan_mode ADC Scan mode
  * @{
  */
/*       warned of change of setting choices with assert check.               */
#define ADC_SCAN_DIRECTION_FORWARD        (0x00000001U)        /*!< Scan direction forward: from channel 0 to channel 7 */
#define ADC_SCAN_DIRECTION_BACKWARD       (0x00000002U)        /*!< Scan direction backward: from channel 7 to channel 0 */

#define ADC_SCAN_ENABLE         ADC_SCAN_DIRECTION_FORWARD       /* For compatibility with other sifli devices */

/**
  * @}
  */

/** @defgroup ADC_External_trigger_edge_Regular ADC External trigger edge Regular
  * @{
  */
#define ADC_EXTERNALTRIGCONVEDGE_NONE           (0x00000000U)

/**
  * @}
  */



/**
  * @}
  */


/* Private constants ---------------------------------------------------------*/

/** @addtogroup ADC_Private_Constants ADC Private Constants
  * @{
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/

/** @defgroup ADC_Exported_Macros ADC Exported Macros
  * @{
  */


/**
  * @brief Get the selected ADC's flag status.
  * @param  \__HANDLE__ ADC handle
  * @param  \__FLAG__ ADC flag
  * @retval None
  */
#define __HAL_ADC_GET_FLAG(__HANDLE__, __FLAG__)                               \
  ((((__HANDLE__)->Instance->GPADC_IRQ) & (__FLAG__)) == (__FLAG__))

/**
  * @brief Clear the ADC's pending flags
  * @param  \__HANDLE__ ADC handle
  * @param  \__FLAG__ ADC flag
  * @retval None
  */
/* Note: bit cleared bit by writing 1 (writing 0 has no effect on any bit of register ISR) */
#define __HAL_ADC_CLEAR_FLAG(__HANDLE__, __FLAG__)                             \
  (((__HANDLE__)->Instance->GPADC_IRQ) |= (__FLAG__))

/**
  * @brief Enable ADC irq
  * @param  \__HANDLE__ ADC handle
  * @param  \__FLAG__ ADC irq flag
  * @retval None
  */
#define __HAL_ADC_ENABLE_IRQ(__HANDLE__, __FLAG__)  ((__HANDLE__)->Instance->GPADC_IRQ |= (__FLAG__))

/**
  * @brief Disable ADC irq
  * @param  \__HANDLE__ ADC handle
  * @param  \__FLAG__ ADC irq flag
  * @retval None
  */
#define __HAL_ADC_DISABLE_IRQ(__HANDLE__, __FLAG__)  ((__HANDLE__)->Instance->GPADC_IRQ &= ~(__FLAG__))

/**
  * @brief Enable ADC LdoRef
  * @param  \__HANDLE__ ADC handle
  * @retval None
  */
#define __HAL_ADC_ENABLE_LDO_REF(__HANDLE__)  ((__HANDLE__)->Instance->ADC_CFG_REG1 |= (GPADC_ADC_CFG_REG1_ANAU_GPADC_LDOREF_EN))


/**
  * @brief Disable ADC LdoRef
  * @param  \__HANDLE__ ADC handle
  * @retval None
  */
#define __HAL_ADC_DISABLE_LDO_REF(__HANDLE__)  ((__HANDLE__)->Instance->ADC_CFG_REG1 &= ~(GPADC_ADC_CFG_REG1_ANAU_GPADC_LDOREF_EN))

/** @brief  Set ADC LdoRef Sel
  * @param  \__HANDLE__ ADC handle
  * @param  LdoRef Sel Value.
  * @retval None
  */
#define __HAL_ADC_SET_LDO_REF_SEL(__HANDLE__,value)                               \
    do { \
        MODIFY_REG((__HANDLE__)->Instance->ADC_CFG_REG1, GPADC_ADC_CFG_REG1_ANAU_GPADC_LDOVREF_SEL_Msk, MAKE_REG_VAL(value, GPADC_ADC_CFG_REG1_ANAU_GPADC_LDOVREF_SEL_Msk, GPADC_ADC_CFG_REG1_ANAU_GPADC_LDOVREF_SEL_Pos));  \
    }while(0)\

/** @brief  Reset ADC handle state
  * @param  \__HANDLE__ ADC handle
  * @retval None
  */
#define __HAL_ADC_RESET_HANDLE_STATE(__HANDLE__)                               \
  ((__HANDLE__)->State = HAL_ADC_STATE_RESET)


#ifndef SF32LB55X
/** @brief  Set ADC Sample width
  * @param  \__HANDLE__ ADC handle
  * @param  width Sample width in unit of PCLK cycles.
  * @retval None
  */
#define __HAL_ADC_SET_SAMPLE_WIDTH(__HANDLE__,width)                               \
    do { \
        (__HANDLE__)->Instance->ADC_CTRL_REG2 &= (~GPADC_ADC_CTRL_REG2_SAMP_WIDTH_Msk);    \
        (__HANDLE__)->Instance->ADC_CTRL_REG2 |= (((width)-1)<<GPADC_ADC_CTRL_REG2_SAMP_WIDTH_Pos);  \
    }while(0)\

/** @brief  Set ADC Conversion width
  * @param  \__HANDLE__ ADC handle
  * @param  width Conversion width in unit of PCLK cycles.
  * @retval None
  */
#define __HAL_ADC_SET_CONV_WIDTH(__HANDLE__,width)                               \
    do { \
        (__HANDLE__)->Instance->ADC_CTRL_REG2 &= (~GPADC_ADC_CTRL_REG2_CONV_WIDTH_Msk);    \
        (__HANDLE__)->Instance->ADC_CTRL_REG2 |= (((width)-1)<<GPADC_ADC_CTRL_REG2_CONV_WIDTH_Pos);  \
    }while(0)\

/** @brief  Set ADC Data sample delay
  * @param  \__HANDLE__ ADC handle
  * @param  delay Delay for data in unit of PCLK cycles.
  * @retval None
  */
#define __HAL_ADC_SET_DATA_DELAY(__HANDLE__,delay)                               \
    do { \
        (__HANDLE__)->Instance->ADC_CTRL_REG &= (~GPADC_ADC_CTRL_REG_DATA_SAMP_DLY);    \
        (__HANDLE__)->Instance->ADC_CTRL_REG |= ((delay)<<GPADC_ADC_CTRL_REG_DATA_SAMP_DLY_Pos);  \
    }while(0)\

#else
/** @brief  Set ADC clock divider
  * @param  \__HANDLE__ ADC handle
  * @param  div Clock divider
  * @retval None
  */
#define __HAL_ADC_SET_CLOCK_DIV(__HANDLE__, div)                               \
    do { \
        (__HANDLE__)->Instance->ADC_CTRL_REG &= (~GPADC_ADC_CTRL_REG_ADC_CLK_DIV);    \
        (__HANDLE__)->Instance->ADC_CTRL_REG |= ((div)<<GPADC_ADC_CTRL_REG_ADC_CLK_DIV_Pos);  \
    }while(0)\

#endif

/** @brief  Start ADC convert
  * @param  \__HANDLE__ ADC handle
  * @retval None
  */
#define __HAL_ADC_START_CONV(__HANDLE__) ((__HANDLE__)->Instance->ADC_CTRL_REG |= GPADC_ADC_CTRL_REG_ADC_START)


/** @brief  Stop ADC convert, only work for infinite mode
  * @param  \__HANDLE__ ADC handle
  * @retval None
  */
#define __HAL_ADC_STOP_CONV(__HANDLE__) ((__HANDLE__)->Instance->ADC_CTRL_REG |= GPADC_ADC_CTRL_REG_ADC_STOP)


/** @brief  Set ADC to single end mode
  * @param  \__HANDLE__ ADC handle
  * @retval None
  */
#define __HAL_ADC_SINGLE_END(__HANDLE__) ((__HANDLE__)->Instance->ADC_CFG_REG1 |= GPADC_ADC_CFG_REG1_ANAU_GPADC_SE)

/** @brief  Set ADC to differential mode
  * @param  \__HANDLE__ ADC handle
  * @retval None
  */
#define __HAL_ADC_DIFF_MODE(__HANDLE__) ((__HANDLE__)->Instance->ADC_CFG_REG1 &= (~GPADC_ADC_CFG_REG1_ANAU_GPADC_SE))

/**
  * @}
  */

/* Private macro -------------------------------------------------------------*/

/** @defgroup ADC_Private_Macros ADC Private Macros
  * @{
  */
/* Macro reserved for internal HAL driver usage, not intended to be used in   */
/* code of final user.                                                        */


/**
  * @brief Verification of ADC state: enabled or disabled
  * @param  \__HANDLE__ ADC handle
  * @retval SET (ADC enabled) or RESET (ADC disabled)
  */
/* Note: If low power mode AutoPowerOff is enabled, power-on/off phases are   */
/*       performed automatically by hardware and flag ADC_FLAG_RDY is not     */
/*       set.                                                                 */
#define ADC_IS_ENABLE(__HANDLE__)                                                       \
  (( (((__HANDLE__)->Instance->ADC_SLOT0_REG) & GPADC_ADC_SLOT0_REG_SLOT_EN) |  \
  (((__HANDLE__)->Instance->ADC_SLOT1_REG) & GPADC_ADC_SLOT1_REG_SLOT_EN) |  \
  (((__HANDLE__)->Instance->ADC_SLOT2_REG) & GPADC_ADC_SLOT2_REG_SLOT_EN) |  \
  (((__HANDLE__)->Instance->ADC_SLOT3_REG) & GPADC_ADC_SLOT3_REG_SLOT_EN) |  \
  (((__HANDLE__)->Instance->ADC_SLOT4_REG) & GPADC_ADC_SLOT4_REG_SLOT_EN) |  \
  (((__HANDLE__)->Instance->ADC_SLOT5_REG) & GPADC_ADC_SLOT5_REG_SLOT_EN) |  \
  (((__HANDLE__)->Instance->ADC_SLOT6_REG) & GPADC_ADC_SLOT6_REG_SLOT_EN) |  \
  (((__HANDLE__)->Instance->ADC_SLOT7_REG) & GPADC_ADC_SLOT7_REG_SLOT_EN) |  \
   ) ? SET : RESET)

/**
  * @brief Simultaneously clears and sets specific bits of the handle State
  * @note: ADC_STATE_CLR_SET() macro is merely aliased to generic macro MODIFY_REG(),
  *        the first parameter is the ADC handle State, the second parameter is the
  *        bit field to clear, the third and last parameter is the bit field to set.
  * @retval None
  */
#define ADC_STATE_CLR_SET MODIFY_REG

/**
  * @brief Clear ADC error code (set it to error code: "no error")
  * @param  \__HANDLE__ ADC handle
  * @retval None
  */
#define ADC_CLEAR_ERRORCODE(__HANDLE__)                                        \
  ((__HANDLE__)->ErrorCode = HAL_ADC_ERROR_NONE)


/**
  * @brief Configure the channel number into channel selection register
  * @param \_CHANNEL_ ADC Channel
  * @retval None
  */
/* This function converts ADC channels from numbers (see defgroup ADC_channels)
   to bitfields, to get the equivalence of CMSIS channels:
        ADC_CHANNEL_0           ((uint32_t) ADC_CHSELR_CHSEL0)
        ADC_CHANNEL_1           ((uint32_t) ADC_CHSELR_CHSEL1)
        ADC_CHANNEL_2           ((uint32_t) ADC_CHSELR_CHSEL2)
        ADC_CHANNEL_3           ((uint32_t) ADC_CHSELR_CHSEL3)
        ADC_CHANNEL_4           ((uint32_t) ADC_CHSELR_CHSEL4)
        ADC_CHANNEL_5           ((uint32_t) ADC_CHSELR_CHSEL5)
        ADC_CHANNEL_6           ((uint32_t) ADC_CHSELR_CHSEL6)
        ADC_CHANNEL_7           ((uint32_t) ADC_CHSELR_CHSEL7)
*/
#define ADC_CHSELR_CHANNEL(_CHANNEL_)                                          \
  ( 1U << (_CHANNEL_))


/**
  * @brief Enable ADC continuous conversion mode.
  * @param \__HANDLE__ ADC handle.
  * @param \_CONTINUOUS_MODE_ Continuous mode.
  * @retval None
  */
#define ADC_CFGR_CONTINUOUS(__HANDLE__, _CONTINUOUS_MODE_)                                \
  (((__HANDLE__)->Instance->ADC_CTRL_REG) |= (_CONTINUOUS_MODE_) << GPADC_ADC_CTRL_REG_ADC_OP_MODE_Pos)


/**
  * @brief Enable the ADC DMA continuous request.
  * @param \_DMACONTREQ_MODE_ DMA continuous request mode.
  * @retval None
  */
#define ADC_CFGR_DMACONTREQ(_DMACONTREQ_MODE_)                                \
  ((_DMACONTREQ_MODE_) << 1U)


/**
  * @brief Enable ADC DMA mode.
  * @param \__HANDLE__ ADC handle.
  * @retval None
  */
#define ADC_DMA_ENABLE(__HANDLE__)      ((__HANDLE__)->Instance->ADC_CTRL_REG |= GPADC_ADC_CTRL_REG_DMA_EN)

/**
  * @brief ADC DMA output raw data.
  * @param \__HANDLE__ ADC handle.
  * @retval None
  */
#define ADC_DMA_RAW_DATA(__HANDLE__)      ((__HANDLE__)->Instance->ADC_CTRL_REG |= GPADC_ADC_CTRL_REG_DMA_DATA_SEL)

/**
  * @brief ADC DMA output combined data.
  * @param \__HANDLE__ ADC handle.
  * @retval None
  */
#define ADC_DMA_COMB_DATA(__HANDLE__)      ((__HANDLE__)->Instance->ADC_CTRL_REG &= (~GPADC_ADC_CTRL_REG_DMA_DATA_SEL))

/**
  * @brief Disable ADC DMA mode.
  * @param \__HANDLE__ ADC handle.
  * @retval None
  */
#define ADC_DMA_DISABLE(__HANDLE__)      ((__HANDLE__)->Instance->ADC_CTRL_REG &= (~GPADC_ADC_CTRL_REG_DMA_EN))

/**
  * @brief Check if timer triger enable.
  * @param \__HANDLE__ ADC handle.
  * @retval Not 0 if timer triger enable
  */
#define IS_ADC_SRC_TIMER(__HANDLE__)    ((__HANDLE__)->Instance->ADC_CTRL_REG & GPADC_ADC_CTRL_REG_TIMER_TRIG_EN)

/**
  * @brief Enable timer triger.
  * @param \__HANDLE__ ADC handle.
  * @retval None
  */
#define ADC_ENABLE_TIMER_TRIGER(__HANDLE__)    ((__HANDLE__)->Instance->ADC_CTRL_REG |= GPADC_ADC_CTRL_REG_TIMER_TRIG_EN)

/**
  * @brief Enable timer triger.
  * @param \__HANDLE__ ADC handle.
  * @retval None
  */
#define ADC_DISABLE_TIMER_TRIGER(__HANDLE__)    ((__HANDLE__)->Instance->ADC_CTRL_REG &= (~GPADC_ADC_CTRL_REG_TIMER_TRIG_EN))

/**
  * @brief Enable GPIO triger.
  * @param \__HANDLE__ ADC handle.
  * @retval None
  */
#define ADC_ENABLE_GPIO_TRIGER(__HANDLE__)    ((__HANDLE__)->Instance->ADC_CTRL_REG |= GPADC_ADC_CTRL_REG_GPIO_TRIG_EN)

/**
  * @brief Enable GPIO triger.
  * @param \__HANDLE__ ADC handle.
  * @retval None
  */
#define ADC_DISABLE_GPIO_TRIGER(__HANDLE__)    ((__HANDLE__)->Instance->ADC_CTRL_REG &= (~GPADC_ADC_CTRL_REG_GPIO_TRIG_EN))



/**
  * @brief ADC TIMER triger with level mode, need edge detect.
  * @param \__HANDLE__ ADC handle.
  * @retval None
  */
#define ADC_TIMER_TRIGER_LEVEL(__HANDLE__)    ((__HANDLE__)->Instance->ADC_CTRL_REG |= GPADC_ADC_CTRL_REG_TIMER_TRIG_TYP)

/**
  * @brief ADC TIMER triger with pulse mode, no edge detect.
  * @param \__HANDLE__ ADC handle.
  * @retval None
  */
#define ADC_TIMER_TRIGER_PULSE(__HANDLE__)    ((__HANDLE__)->Instance->ADC_CTRL_REG &=(~GPADC_ADC_CTRL_REG_TIMER_TRIG_TYP))

/**
  * @brief Manual open ADC power.
  * @param \__HANDLE__ ADC handle.
  * @retval None
  */
#define ADC_FRC_EN(__HANDLE__)    ((__HANDLE__)->Instance->ADC_CTRL_REG |=(GPADC_ADC_CTRL_REG_FRC_EN_ADC))

/**
  * @brief AUTO open ADC power.
  * @param \__HANDLE__ ADC handle.
  * @retval None
  */
#define ADC_CLR_FRC_EN(__HANDLE__)    ((__HANDLE__)->Instance->ADC_CTRL_REG &=(~GPADC_ADC_CTRL_REG_FRC_EN_ADC))

/**
  * @brief Use auto chnel selet in configure register.
  * @param \__HANDLE__ ADC handle.
  * @retval None
  */
#define ADC_CHNL_SEL_FRC_EN(__HANDLE__)    ((__HANDLE__)->Instance->ADC_CTRL_REG |=(GPADC_ADC_CTRL_REG_CHNL_SEL_FRC_EN))

/**
  * @brief Disable channel auto select.
  * @param \__HANDLE__ ADC handle.
  * @retval None
  */
#define ADC_CHNL_CLR_FRC_EN(__HANDLE__)    ((__HANDLE__)->Instance->ADC_CTRL_REG &=(~GPADC_ADC_CTRL_REG_CHNL_SEL_FRC_EN))

/**
  * @brief Set ADC mute.
  * @param \__HANDLE__ ADC handle.
  * @retval None
  */
#define ADC_SET_MUTE(__HANDLE__)    ((__HANDLE__)->Instance->ADC_CFG_REG1 |=(GPADC_ADC_CFG_REG1_ANAU_GPADC_MUTE))

/**
  * @brief Set ADC unmute.
  * @param \__HANDLE__ ADC handle.
  * @retval None
  */
#define ADC_SET_UNMUTE(__HANDLE__)    ((__HANDLE__)->Instance->ADC_CFG_REG1 &=(~GPADC_ADC_CFG_REG1_ANAU_GPADC_MUTE))


/** @defgroup ADC_range_verification ADC range verification
  * in function of ADC resolution selected (12, 10, 8 or 6 bits)
  * @{
  */
#define IS_ADC_RANGE(RESOLUTION, ADC_VALUE)      (((RESOLUTION) == ADC_RESOLUTION_10B) && ((ADC_VALUE) <= (0x03FFU)))
/**
  * @}
  */

/** @defgroup ADC_regular_rank_verification ADC regular rank verification
  * @{
  */
#define IS_ADC_REGULAR_RANK(RANK) (((RANK) >= (1U)) && ((RANK) <= (16U)))

/**
  * @}
  */

/**
  * @}
  */


/* Exported functions --------------------------------------------------------*/
/** @addtogroup ADC_Exported_Functions
  * @{
  */

/** @addtogroup ADC_Exported_Functions_Group1
  * @{
  */


/* Initialization and de-initialization functions  **********************************/

/**
  * @brief  Initializes the ADC peripheral and regular group according to
  *         parameters specified in structure "ADC_InitTypeDef".
  * @note   As prerequisite, ADC clock must be configured at RCC top level
  *         depending on both possible clock sources: APB clock of HSI clock.
  * @note   Possibility to update parameters on the fly:
  *         The setting of these parameters is conditioned to ADC state.
  *         For parameters constraints, see comments of structure
  *         "ADC_InitTypeDef".
  * @note   This function configures the ADC within 2 scopes: scope of entire
  *         ADC and scope of regular group. For parameters details, see comments
  *         of structure "ADC_InitTypeDef".
  * @param  hadc ADC handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ADC_Init(ADC_HandleTypeDef *hadc);

/**
  * @brief Deinitial ADC hardware.
  * @param hadc ADC handle.
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef HAL_ADC_DeInit(ADC_HandleTypeDef *hadc);

/**
  * @}
  */

/* IO operation functions  *****************************************************/

/** @addtogroup ADC_Exported_Functions_Group2
  * @{
  */


/* Blocking mode: Polling */

/**
  * @brief  Prepare ADC setting before start triger.
  * @param  hadc ADC handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ADC_Prepare(ADC_HandleTypeDef *hadc);

/**
  * @brief  Enables ADC, starts conversion of regular group.
  *         Interruptions enabled in this function: None.
  * @param  hadc ADC handle
  * @retval HAL status
  */
HAL_StatusTypeDef       HAL_ADC_Start(ADC_HandleTypeDef *hadc);

/**
  * @brief  Stop ADC conversion of regular group, disable ADC peripheral.
  * @param  hadc ADC handle
  * @retval HAL status.
  */
HAL_StatusTypeDef       HAL_ADC_Stop(ADC_HandleTypeDef *hadc);

/**
  * @brief  Wait for regular group conversion to be completed.
  * @param  hadc ADC handle
  * @param  Timeout Timeout value in millisecond.
  * @retval HAL status
  */
HAL_StatusTypeDef       HAL_ADC_PollForConversion(ADC_HandleTypeDef *hadc, uint32_t Timeout);

/* Non-blocking mode: Interruption */

/**
  * @brief Start ADC with IRQ mode.
  * @param hadc ADC handle.
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef       HAL_ADC_Start_IT(ADC_HandleTypeDef *hadc);

/**
  * @brief Stop ADC irq mode.
  * @param hadc ADC handle.
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef       HAL_ADC_Stop_IT(ADC_HandleTypeDef *hadc);

/* Non-blocking mode: DMA */

/**
  * @brief start ADC convert with dma mode.
  * @param hadc ADC handle.
  * @param pData data buffer.
  * @param Length data lenght.
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef       HAL_ADC_Start_DMA(ADC_HandleTypeDef *hadc, uint32_t *pData, uint32_t Length);

/**
  * @brief Stop ADC DMA transmit.
  * @param hadc ADC handle.
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef       HAL_ADC_Stop_DMA(ADC_HandleTypeDef *hadc);

/**
  * @brief Configure adc power before start dma.
  * @param hadc ADC handle.
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef HAL_ADC_DMA_PREPARE(ADC_HandleTypeDef *hadc);

/**
  * @brief Wait ADC DMA done by polling mode.
  * @param hadc ADC handle.
  * @param timeout wait time out in ms
  * @retval HAL_StatusTypeDef
  */

HAL_StatusTypeDef HAL_ADC_DMA_WAIT_DONE(ADC_HandleTypeDef *hadc, uint32_t timeout);

/* ADC retrieve conversion value intended to be used with polling or interruption */
/**
  * @brief Get ADC convert result.
  * @param hadc ADC handle.
  * @param slot ADC slot number.
  * @retval ADC convert result.
  */
uint32_t                HAL_ADC_GetValue(ADC_HandleTypeDef *hadc, uint32_t slot);

/**
  * @brief Get all ADC data register value.
  * @param hadc ADC handle.
  * @param buf  all adc channel register value
  * @retval HAL_StatusTypeDeft.
  */
HAL_StatusTypeDef HAL_ADC_Get_All(ADC_HandleTypeDef *hadc, uint32_t *buf);

/**
  * @brief Set ADC triger mode.
  * @param hadc ADC handle.
  * @param src triger source mode.
  * @retval HAL_StatusTypeDef.
  */
HAL_StatusTypeDef       HAL_ADC_SetSource(ADC_HandleTypeDef *hadc, HAL_ADC_SRC__T src);

/**
  * @brief set ADC timer triger mode.
  * @param hadc ADC handle.
  * @param src timer index.
  * @retval HAL_StatusTypeDef.
  */
HAL_StatusTypeDef       HAL_ADC_SetTimer(ADC_HandleTypeDef *hadc, HAL_ADC_SRC_TIME_T src);

/**
  * @brief Enable/Disable ADC slot/channel.
  * @param hadc ADC handle.
  * @param slot ADC slot number.
  * @param en enable 1 for disable 0.
  * @retval HAL_StatusTypeDef.
  */
HAL_StatusTypeDef HAL_ADC_EnableSlot(ADC_HandleTypeDef *hadc, uint32_t slot, uint8_t en);

/**
  * @brief Configure use multi slots or fixed slot
  * @param hadc ADC handle.
  * @param multi_mode when set to 1 channels set to same slot number, when set to 0 fix use slot 0 for all channel.
  * @retval HAL_StatusTypeDef.
  */
HAL_StatusTypeDef HAL_ADC_Set_MultiMode(ADC_HandleTypeDef *hadc, uint8_t multi_mode);

/**
  * @brief Get GPADC offset.
  * @param hadc ADC handle.
  * @retval adc offset.
  */
int HAL_ADC_Get_Offset(ADC_HandleTypeDef *hadc);


/**
  * @}
  */


/* Peripheral Control functions ***********************************************/
/** @addtogroup ADC_Exported_Functions_Group3
  * @{
  */

/**
  * @brief  Configures the the selected channel to be linked to the regular
  *         group.
  * @note   In case of usage of internal measurement channels:
  *         VrefInt/Vbat/TempSensor.
  *         Sampling time constraints must be respected (sampling time can be
  *         adjusted in function of ADC clock frequency and sampling time
  *         setting).
  *         Refer to device datasheet for timings values, parameters TS_vrefint,
  *         TS_vbat, TS_temp (values rough order: 5us to 17us).
  *         These internal paths can be be disabled using function
  *         HAL_ADC_DeInit().
  * @note   Possibility to update parameters on the fly:
  *         This function initializes channel into regular group, following
  *         calls to this function can be used to reconfigure some parameters
  *         of structure "ADC_ChannelConfTypeDef" on the fly, without reseting
  *         the ADC.
  *         The setting of these parameters is conditioned to ADC state.
  *         For parameters constraints, see comments of structure
  *         "ADC_ChannelConfTypeDef".
  * @param  hadc ADC handle
  * @param  sConfig Structure of ADC channel for regular group.
  * @retval HAL status
  */
HAL_StatusTypeDef       HAL_ADC_ConfigChannel(ADC_HandleTypeDef *hadc, ADC_ChannelConfTypeDef *sConfig);

/**
  * @brief Set ADC sampling frequency.
  * @param hadc ADC handle.
  * @param freq frequence want to be set.
  * @retval actual work frequency, 0 if fail.
  */
uint32_t HAL_ADC_SetFreq(ADC_HandleTypeDef *hadc, uint32_t freq);


/**
  * @}
  */


/* Peripheral State functions *************************************************/
/** @addtogroup ADC_Exported_Functions_Group4
  * @{
  */

/**
  * @brief Get ADC status.
  * @param hadc ADC handle.
  * @retval current status.
  */
uint32_t                HAL_ADC_GetState(ADC_HandleTypeDef *hadc);

/**
  * @brief Get ADC error code.
  * @param hadc ADC handle.
  * @retval current error code.
  */
uint32_t                HAL_ADC_GetError(ADC_HandleTypeDef *hadc);

/**
  * @brief  Initialize the ADC MSP.
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *               the configuration information for ADC module.
  * @retval None
  */
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc);

/**
  * @brief  De-Initialize the ADC MSP.
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *               the configuration information for ADC module.
  * @retval None
  */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc);

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


#endif /* __BF0_HAL_ADC_H */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

