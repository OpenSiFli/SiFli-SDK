/**
  ******************************************************************************
  * @file   bf0_hal_lcdc.h
  * @author Sifli software development team
  * @brief   Header file of LCD controller HAL module.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BF0_HAL_LPCOMP_H
#define BF0_HAL_LPCOMP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal_def.h"


/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @addtogroup COMP
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup COMP_Exported_Types COMP Exported Types
  * @{
  */

/**
  * @brief  COMP Init structure definition
  */
typedef struct
{
    uint32_t Mode;               /*!< Set comparator operating mode to adjust power and speed.
                                    Note: For the characteristics of comparator power modes
                                          (propagation delay and power consumption), refer to device datasheet.
                                    This parameter can be a value of @ref COMP_PowerMode */

    uint32_t NonInvertingInput;  /*!< Set comparator input plus (non-inverting input).
                                    This parameter can be a value of @ref COMP_InputPlus */

    uint32_t InvertingInput;     /*!< Set comparator input minus (inverting input).
                                    This parameter can be a value of @ref COMP_InputMinus */

    uint32_t Hysteresis;         /*!< Set comparator hysteresis mode of the input minus.
                                    This parameter can be a value of @ref COMP_Hysteresis */

    uint32_t TriggerMode;        /*!< Set the comparator output triggering External Interrupt Line (EXTI).
                                    This parameter can be a value of @ref COMP_EXTI_TriggerMode */

    uint32_t InternalVRef;        /*!< Set the comparator internal voltage refrence.
                                This parameter can be a value of @ref COMP_Internal_VRefMode */

    uint32_t WorkingPin;        /*!<  There are 2 comparator, they can not work seperate, need use one instance.
                                Use a flag to check whichi one used , bit 0 for comp0 ,bit 1 for comp1, 2 for composite*/
} COMP_InitTypeDef;

typedef struct
{
    uint32_t Mode;               /*!< Set comparator operating mode to adjust power and speed.
                                    Note: For the characteristics of comparator power modes
                                          (propagation delay and power consumption), refer to device datasheet.
                                    This parameter can be a value of @ref COMP_PowerMode */

    uint32_t InvertingInput;     /*!< Set comparator input minus (inverting input).
                                    This parameter can be a value of @ref COMP_InputMinus */

    uint32_t TriggerMode;        /*!< Set the comparator output triggering External Interrupt Line (EXTI).
                                    This parameter can be a value of @ref COMP_EXTI_TriggerMode */

    uint32_t InternalVRef;        /*!< Set the comparator internal voltage refrence.
                                This parameter can be a value of @ref COMP_Internal_VRefMode */

    uint32_t WorkingPin;        /*!<  There are 2 comparator, they can not work seperate, need use one instance.
                                Use a flag to check whichi one used , bit 0 for comp0 ,bit 1 for comp1, 2 for composite*/
} COMP_ConfigTypeDef;


/**
  * @brief  HAL COMP state machine: HAL COMP states definition
  */
#define COMP_STATE_BITFIELD_LOCK  (0x10U)
typedef enum
{
    HAL_COMP_STATE_RESET             = 0x00U,                                             /*!< COMP not yet initialized                             */
    HAL_COMP_STATE_RESET_LOCKED      = (HAL_COMP_STATE_RESET | COMP_STATE_BITFIELD_LOCK), /*!< COMP not yet initialized and configuration is locked */
    HAL_COMP_STATE_READY             = 0x01U,                                             /*!< COMP initialized and ready for use                   */
    HAL_COMP_STATE_READY_LOCKED      = (HAL_COMP_STATE_READY | COMP_STATE_BITFIELD_LOCK), /*!< COMP initialized but configuration is locked         */
    HAL_COMP_STATE_BUSY              = 0x02U,                                             /*!< COMP is running                                      */
    HAL_COMP_STATE_BUSY_LOCKED       = (HAL_COMP_STATE_BUSY | COMP_STATE_BITFIELD_LOCK)   /*!< COMP is running and configuration is locked          */
} HAL_COMP_StateTypeDef;

/**
  * @brief  COMP Handle Structure definition
  */
typedef struct __COMP_HandleTypeDef
{
    LPCOMP_TypeDef     *Instance;       /*!< Register base address    */
    COMP_InitTypeDef   Init;            /*!< COMP required parameters */
    HAL_LockTypeDef    Lock;            /*!< Locking object           */
    __IO HAL_COMP_StateTypeDef  State;  /*!< COMP communication state */
    __IO uint32_t      ErrorCode;       /*!< COMP error code */
#if (USE_HAL_COMP_REGISTER_CALLBACKS == 1)
    void (* TriggerCallback)(struct __COMP_HandleTypeDef *hcomp);   /*!< COMP trigger callback */
    void (* MspInitCallback)(struct __COMP_HandleTypeDef *hcomp);   /*!< COMP Msp Init callback */
    void (* MspDeInitCallback)(struct __COMP_HandleTypeDef *hcomp); /*!< COMP Msp DeInit callback */
#endif /* USE_HAL_COMP_REGISTER_CALLBACKS */
} COMP_HandleTypeDef;

#if (USE_HAL_COMP_REGISTER_CALLBACKS == 1)
/**
  * @brief  HAL COMP Callback ID enumeration definition
  */
typedef enum
{
    HAL_COMP_TRIGGER_CB_ID                = 0x00U,  /*!< COMP trigger callback ID */
    HAL_COMP_MSPINIT_CB_ID                = 0x01U,  /*!< COMP Msp Init callback ID */
    HAL_COMP_MSPDEINIT_CB_ID              = 0x02U   /*!< COMP Msp DeInit callback ID */
} HAL_COMP_CallbackIDTypeDef;

/**
  * @brief  HAL COMP Callback pointer definition
  */
typedef  void (*pCOMP_CallbackTypeDef)(COMP_HandleTypeDef *hcomp); /*!< pointer to a COMP callback function */

#endif /* USE_HAL_COMP_REGISTER_CALLBACKS */

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup COMP_Exported_Constants COMP Exported Constants
  * @{
  */

/** @defgroup COMP_Error_Code COMP Error Code
  * @{
  */
#define HAL_COMP_ERROR_NONE             (0x00UL)  /*!< No error */
#if (USE_HAL_COMP_REGISTER_CALLBACKS == 1)
#define HAL_COMP_ERROR_INVALID_CALLBACK (0x01UL)  /*!< Invalid Callback error */
#endif /* USE_HAL_COMP_REGISTER_CALLBACKS */
/**
  * @}
  */

/** @defgroup COMP_PowerMode COMP power mode
  * @{
  */
/* Note: For the characteristics of comparator power modes                    */
/*       (propagation delay and power consumption),                           */
/*       refer to device datasheet.                                           */
#define COMP_POWERMODE_HIGHSPEED       (0x00000002UL<<LPCOMP_CR1_MODE_Pos)         /*!< High Speed */
#define COMP_POWERMODE_MEDIUMSPEED     (0x00000001UL<<LPCOMP_CR1_MODE_Pos)   /*!< Medium Speed */
#define COMP_POWERMODE_ULTRALOWPOWER   (0x00000000UL)     /*!< Ultra-low power mode */
/**
  * @}
  */

/** @defgroup COMP_InputPlus COMP input plus (non-inverting input)
  * @{
  */
#define COMP_INPUT_PLUS_IO1            (0x00000000UL)         /*!< Input plus of Comparator 1 connected to LPCOMP1_P OR Comparator 2 connect to LPCOMP2_P*/
#define COMP_INPUT_PLUS_IO2            (0x00000001UL<<LPCOMP_CR1_INPSEL_Pos)    /*!< Comparator input plus connected to IO2 (pin PB2 for COMP1, pin PB6 for COMP2) */

/**
  * @}
  */

/** @defgroup COMP_InputMinus COMP input minus (inverting input)
  * @{
  */
#define COMP_INPUT_MINUS_VREF           (0x00000000UL)                                           /*!< Comparator input minus connected to VREF */
#define COMP_INPUT_MINUS_NPIN           (0x00000001UL<<LPCOMP_CR1_INMSEL_Pos)                    /*!< Comparator input minus connected to OUTSID */

/**
  * @}
  */

/** @defgroup COMP_Hysteresis COMP hysteresis
  * @{
  */
#define COMP_HYSTERESIS_NONE           (0x00000000UL)                       /*!< No hysteresis */
#define COMP_HYSTERESIS_LOW            (0x00000001UL<<LPCOMP_CR1_HYST_Pos)  /*!< Hysteresis level low */
#define COMP_HYSTERESIS_MEDIUM         (0x00000002UL<<LPCOMP_CR1_HYST_Pos  )  /*!< Hysteresis level medium */
#define COMP_HYSTERESIS_HIGH           (0x00000003UL<<LPCOMP_CR1_HYST_Pos)  /*!< Hysteresis level high */
/**
  * @}
  */

/** @defgroup COMP_OutputPolarity COMP output Polarity
  * @{
  */
#define COMP_OUTPUTPOL_NONINVERTED     (0x00000000UL)         /*!< COMP output level is not inverted (comparator output is high when the input plus is at a higher voltage than the input minus) */
#define COMP_OUTPUTPOL_INVERTED        (LPCOMP_CR1_POL)    /*!< COMP output level is inverted     (comparator output is low  when the input plus is at a higher voltage than the input minus) */
/**
  * @}
  */


/** @defgroup COMP_OutputLevel COMP Output Level
  * @{
  */
/* Note: Comparator output level values are fixed to "0" and "1",             */
/* corresponding COMP register bit is managed by HAL function to match        */
/* with these values (independently of bit position in register).             */

/* When output polarity is not inverted, comparator output is low when
   the input plus is at a lower voltage than the input minus */
#define COMP_OUTPUT_LEVEL_LOW              (0x00000000UL)
/* When output polarity is not inverted, comparator output is high when
   the input plus is at a higher voltage than the input minus */
#define COMP_OUTPUT_LEVEL_HIGH             (0x00000001UL)
/**
  * @}
  */

/** @defgroup COMP_EXTI_TriggerMode COMP output to EXTI
  * @{
  */
#define COMP_TRIGGERMODE_NONE                 (0x00000000UL)                                            /*!< Comparator output triggering no External Interrupt Line */
#define COMP_TRIGGERMODE_IT_RISING            (COMP_EXTI_IT | COMP_EXTI_RISING)                         /*!< Comparator output triggering External Interrupt Line event with interruption, on rising edge */
#define COMP_TRIGGERMODE_IT_FALLING           (COMP_EXTI_IT | COMP_EXTI_FALLING)                        /*!< Comparator output triggering External Interrupt Line event with interruption, on falling edge */
#define COMP_TRIGGERMODE_IT_RISING_FALLING    (COMP_EXTI_IT | COMP_EXTI_EDGES)                          /*!< Comparator output triggering External Interrupt Line event with interruption, on both rising and falling edges */
#define COMP_TRIGGERMODE_EVENT_RISING         (COMP_EXTI_EVENT | COMP_EXTI_RISING)                      /*!< Comparator output triggering External Interrupt Line event only (without interruption), on rising edge */
#define COMP_TRIGGERMODE_EVENT_FALLING        (COMP_EXTI_EVENT | COMP_EXTI_FALLING)                     /*!< Comparator output triggering External Interrupt Line event only (without interruption), on falling edge */
#define COMP_TRIGGERMODE_EVENT_RISING_FALLING (COMP_EXTI_EVENT | COMP_EXTI_EDGES)                       /*!< Comparator output triggering External Interrupt Line event only (without interruption), on both rising and falling edges */
/**
  * @}
  */

/** @defgroup COMP_Internal_VRefMode COMP internal voltage reference
  * @{
  */
#define COMP_VREFINT_0D6V               (0x00000000UL)                                   /*!< Comparator internal voltage reference to 0.6v */
#define COMP_VREFINT_1D2V               (0x00000001UL<<LPCOMP_CR1_VREFINT_Pos)          /*!< Comparator internal voltage reference to 1.2v */
#define COMP_VREFINT_1D8V               (0x00000002UL<<LPCOMP_CR1_VREFINT_Pos)          /*!< Comparator internal voltage reference to 1.8v */
#define COMP_VREFINT_2D4V               (0x00000003UL<<LPCOMP_CR1_VREFINT_Pos)          /*!< Comparator internal voltage reference to 2.4v */

/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup COMP_Exported_Macros COMP Exported Macros
  * @{
  */

/** @defgroup COMP_Handle_Management  COMP Handle Management
  * @{
  */

/** @brief  Reset COMP handle state.
  * @param  \__HANDLE__  COMP handle
  * @retval None
  */
#if (USE_HAL_COMP_REGISTER_CALLBACKS == 1)
#define __HAL_COMP_RESET_HANDLE_STATE(__HANDLE__) do{                                                 \
                                                     (__HANDLE__)->State = HAL_COMP_STATE_RESET;      \
                                                     (__HANDLE__)->MspInitCallback = NULL;            \
                                                     (__HANDLE__)->MspDeInitCallback = NULL;          \
                                                    } while(0)
#else
#define __HAL_COMP_RESET_HANDLE_STATE(__HANDLE__) ((__HANDLE__)->State = HAL_COMP_STATE_RESET)
#endif

/**
  * @brief Clear COMP error code (set it to no error code "HAL_COMP_ERROR_NONE").
  * @param \__HANDLE__ COMP handle
  * @retval None
  */
#define COMP_CLEAR_ERRORCODE(__HANDLE__) ((__HANDLE__)->ErrorCode = HAL_COMP_ERROR_NONE)

/**
  * @brief  Enable the specified comparator.
  * @param  \__HANDLE__  COMP handle
  * @retval None
  */
#define __HAL_COMP_ENABLE(__HANDLE__)              SET_BIT((__HANDLE__)->Instance->CR1, LPCOMP_CR1_EN)

/**
  * @brief  Disable the specified comparator.
  * @param  \__HANDLE__  COMP handle
  * @retval None
  */
#define __HAL_COMP_DISABLE(__HANDLE__)             CLEAR_BIT((__HANDLE__)->Instance->CR1, LPCOMP_CR1_EN)

/**
  * @}
  */


/**
  * @}
  */


/* Private types -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/** @defgroup COMP_Private_Constants COMP Private Constants
  * @{
  */


/** @defgroup COMP_ExtiLine COMP EXTI Lines
  * @{
  */
#define COMP_EXTI_IT                        (LPCOMP_CR1_IE)  /*!< EXTI line event with interruption */
#define COMP_EXTI_EVENT                     (0x00000000UL)  /*!< EXTI line event only (without interruption) */
#define COMP_EXTI_HIGH                      (0x00000001UL<<LPCOMP_CR1_SENS_Pos)  /*!< EXTI line event on high level */
#define COMP_EXTI_LOW                       (0x00000002UL<<LPCOMP_CR1_SENS_Pos)  /*!< EXTI line event on low level */
#define COMP_EXTI_RISING                    (0x00000005UL<<LPCOMP_CR1_SENS_Pos)  /*!< EXTI line event on rising edge */
#define COMP_EXTI_FALLING                   (0x00000006UL<<LPCOMP_CR1_SENS_Pos)  /*!< EXTI line event on falling edge */
#define COMP_EXTI_EDGES                     (0x00000007UL<<LPCOMP_CR1_SENS_Pos)  /*!< EXTI line event on both edge */

/**
  * @}
  */

/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/

/** @defgroup COMP_Private_Constants COMP Private Constants
  * @{
  */

/**
  * @brief  Check if power mode.
  * @param  \__POWERMODE__  COMP handle
  * @retval true if success
  */
#define IS_COMP_POWERMODE(__POWERMODE__)    (((__POWERMODE__) == COMP_POWERMODE_HIGHSPEED)    || \
                                             ((__POWERMODE__) == COMP_POWERMODE_MEDIUMSPEED)  || \
                                             ((__POWERMODE__) == COMP_POWERMODE_ULTRALOWPOWER)  )

/**
  * @brief  Check if HYSTERESIS.
  * @param  \__HYSTERESIS__  COMP handle
  * @retval true if success
  */
#define IS_COMP_HYSTERESIS(__HYSTERESIS__)  (((__HYSTERESIS__) == COMP_HYSTERESIS_NONE)   || \
                                             ((__HYSTERESIS__) == COMP_HYSTERESIS_LOW)    || \
                                             ((__HYSTERESIS__) == COMP_HYSTERESIS_MEDIUM) || \
                                             ((__HYSTERESIS__) == COMP_HYSTERESIS_HIGH))

/**
  * @brief  Check if output pol.
  * @param  \__POL__  COMP handle
  * @retval true if success
  */
#define IS_COMP_OUTPUTPOL(__POL__)          (((__POL__) == COMP_OUTPUTPOL_NONINVERTED) || \
                                             ((__POL__) == COMP_OUTPUTPOL_INVERTED))

/**
  * @brief  Check if comp triger mode.
  * @param  \__MODE__  COMP handle
  * @retval true if success
  */
#define IS_COMP_TRIGGERMODE(__MODE__)       (((__MODE__) == COMP_TRIGGERMODE_NONE)                 || \
                                             ((__MODE__) == COMP_TRIGGERMODE_IT_RISING)            || \
                                             ((__MODE__) == COMP_TRIGGERMODE_IT_FALLING)           || \
                                             ((__MODE__) == COMP_TRIGGERMODE_IT_RISING_FALLING)    || \
                                             ((__MODE__) == COMP_TRIGGERMODE_EVENT_RISING)         || \
                                             ((__MODE__) == COMP_TRIGGERMODE_EVENT_FALLING)        || \
                                             ((__MODE__) == COMP_TRIGGERMODE_EVENT_RISING_FALLING))

/**
  * @brief  Check if comp output level.
  * @param  \__OUTPUT_LEVEL__  COMP handle
  * @retval true if success
  */
#define IS_COMP_OUTPUT_LEVEL(__OUTPUT_LEVEL__) (((__OUTPUT_LEVEL__) == COMP_OUTPUT_LEVEL_LOW)     || \
                                                ((__OUTPUT_LEVEL__) == COMP_OUTPUT_LEVEL_HIGH))

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup COMP_Exported_Functions
  * @{
  */

/** @addtogroup COMP_Exported_Functions_Group1
  * @{
  */

/* Initialization and de-initialization functions  **********************************/
/**
  * @brief  Initialize the COMP according to the specified
  *         parameters in the COMP_InitTypeDef and initialize the associated handle.
  * @note   If the selected comparator is locked, initialization can't be performed.
  *         To unlock the configuration, perform a system reset.
  * @param  hcomp  COMP handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_COMP_Init(COMP_HandleTypeDef *hcomp);

/**
  * @brief  DeInitialize the COMP peripheral.
  * @note   Deinitialization cannot be performed if the COMP configuration is locked.
  *         To unlock the configuration, perform a system reset.
  * @param  hcomp  COMP handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_COMP_DeInit(COMP_HandleTypeDef *hcomp);

/**
  * @brief  Initialize the COMP MSP.
  * @param  hcomp  COMP handle
  * @retval None
  */
void              HAL_COMP_MspInit(COMP_HandleTypeDef *hcomp);

/**
  * @brief  select which comparator to work.
  * @param  hcomp  COMP handle
  * @param  cfg configure parameters
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_COMP_Config(COMP_HandleTypeDef *hcomp, COMP_ConfigTypeDef *cfg);

/**
  * @brief  polling compare result.
  * @param  hcomp  COMP handle
  * @param  channel channel select, 0, 1 valid
  * @param  timeout timeout in ms
  * @retval compare result
  */
int HAL_COMP_PollForComp(COMP_HandleTypeDef *hcomp, uint8_t channel, uint32_t timeout);

/**
  * @brief  DeInitialize the COMP MSP.
  * @param  hcomp  COMP handle
  * @retval None
  */
void              HAL_COMP_MspDeInit(COMP_HandleTypeDef *hcomp);

#if (USE_HAL_COMP_REGISTER_CALLBACKS == 1)
/* Callbacks Register/UnRegister functions  ***********************************/
/**
  * @brief  Register a User COMP Callback
  *         To be used instead of the weak predefined callback
  * @param  hcomp Pointer to a COMP_HandleTypeDef structure that contains
  *                the configuration information for the specified COMP.
  * @param  CallbackID ID of the callback to be registered
  *         This parameter can be one of the following values:
  *          @arg @ref HAL_COMP_TRIGGER_CB_ID Trigger callback ID
  *          @arg @ref HAL_COMP_MSPINIT_CB_ID MspInit callback ID
  *          @arg @ref HAL_COMP_MSPDEINIT_CB_ID MspDeInit callback ID
  * @param  pCallback pointer to the Callback function
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_COMP_RegisterCallback(COMP_HandleTypeDef *hcomp, HAL_COMP_CallbackIDTypeDef CallbackID, pCOMP_CallbackTypeDef pCallback);
/**
  * @brief  Unregister a COMP Callback
  *         COMP callback is redirected to the weak predefined callback
  * @param  hcomp Pointer to a COMP_HandleTypeDef structure that contains
  *                the configuration information for the specified COMP.
  * @param  CallbackID ID of the callback to be unregistered
  *         This parameter can be one of the following values:
  *          @arg @ref HAL_COMP_TRIGGER_CB_ID Trigger callback ID
  *          @arg @ref HAL_COMP_MSPINIT_CB_ID MspInit callback ID
  *          @arg @ref HAL_COMP_MSPDEINIT_CB_ID MspDeInit callback ID
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_COMP_UnRegisterCallback(COMP_HandleTypeDef *hcomp, HAL_COMP_CallbackIDTypeDef CallbackID);
#endif /* USE_HAL_COMP_REGISTER_CALLBACKS */
/**
  * @}
  */

/* IO operation functions  *****************************************************/
/** @addtogroup COMP_Exported_Functions_Group2
  * @{
  */

/**
  * @brief  Start the comparator.
  * @param  hcomp  COMP handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_COMP_Start(COMP_HandleTypeDef *hcomp);

/**
  * @brief  Stop the comparator.
  * @param  hcomp  COMP handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_COMP_Stop(COMP_HandleTypeDef *hcomp);

/**
  * @brief  Comparator IRQ handler.
  * @param  hcomp  COMP handle
  * @retval None
  */
void              HAL_COMP_IRQHandler(COMP_HandleTypeDef *hcomp);
/**
  * @}
  */

/* Peripheral Control functions  ************************************************/
/** @addtogroup COMP_Exported_Functions_Group3
  * @{
  */

/**
  * @brief  Return the output level (high or low) of the selected comparator.
  *         The output level depends on the selected polarity.
  *         If the polarity is not inverted:
  *           - Comparator output is low when the input plus is at a lower
  *             voltage than the input minus
  *           - Comparator output is high when the input plus is at a higher
  *             voltage than the input minus
  *         If the polarity is inverted:
  *           - Comparator output is high when the input plus is at a lower
  *             voltage than the input minus
  *           - Comparator output is low when the input plus is at a higher
  *             voltage than the input minus
  * @param  hcomp  COMP handle
  * @retval Returns the selected comparator output level:
  *         @arg COMP_OUTPUT_LEVEL_LOW
  *         @arg COMP_OUTPUT_LEVEL_HIGH
  *
  */
uint32_t          HAL_COMP_GetOutputLevel(COMP_HandleTypeDef *hcomp);

/**
  * @brief  Comparator trigger callback.
  * @param  hcomp  COMP handle
  * @retval None
  */
void              HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp);
/**
  * @}
  */

/* Peripheral State functions  **************************************************/
/** @addtogroup COMP_Exported_Functions_Group4
  * @{
  */
/**
  * @brief  Return the COMP handle state.
  * @param  hcomp  COMP handle
  * @retval HAL state
  */
HAL_COMP_StateTypeDef HAL_COMP_GetState(COMP_HandleTypeDef *hcomp);

/**
  * @brief  Return the COMP error code.
  * @param hcomp COMP handle
  * @retval COMP error code
  */
uint32_t              HAL_COMP_GetError(COMP_HandleTypeDef *hcomp);
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

#endif /* BF0_HAL_LPCOMP_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
