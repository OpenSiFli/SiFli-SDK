/**
  ******************************************************************************
  * @file   bf0_hal_gpio.h
  * @author Sifli software development team
  * @brief   Header file of GPIO HAL module.
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

#ifndef __BF0_HAL_GPIO_H
#define __BF0_HAL_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal_def.h"

/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @addtogroup GPIO
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/** @defgroup GPIO_Exported_Types GPIO Exported Types
  * @{
  */

typedef struct
{
    __IO uint32_t DIR;
    __IO uint32_t DOR;
#ifndef SF32LB55X
    __IO uint32_t DOSR;
    __IO uint32_t DOCR;
#endif
    __IO uint32_t DOER;
    __IO uint32_t DOESR;
    __IO uint32_t DOECR;
    __IO uint32_t IER;
    __IO uint32_t IESR;
    __IO uint32_t IECR;
    __IO uint32_t ITR;
    __IO uint32_t ITSR;
    __IO uint32_t ITCR;
#ifndef SF32LB55X
    __IO uint32_t IPHR;
    __IO uint32_t IPHSR;
    __IO uint32_t IPHCR;
    __IO uint32_t IPLR;
    __IO uint32_t IPLSR;
    __IO uint32_t IPLCR;
#else
    __IO uint32_t IPSR;
    __IO uint32_t IPCR;
#endif
    __IO uint32_t ISR;
#ifndef SF32LB55X
    __IO uint32_t IER_EXT;
    __IO uint32_t IESR_EXT;
    __IO uint32_t IECR_EXT;
    __IO uint32_t ISR_EXT;
    __IO uint32_t OEMR;
    __IO uint32_t OEMSR;
    __IO uint32_t OEMCR;
    __IO uint32_t RSVD2[5];
#endif
} GPIO_TypeDef;



/**
  * @brief   GPIO Init structure definition
*/

typedef struct
{
    uint32_t Pin;        /*!< Specifies the GPIO pins to be configured.
                           pin id of GPIO1 or GPIO2, starting from 0  */

    uint32_t Mode;       /*!< Specifies the operating mode for the selected pins.
                           This parameter can be a value of @ref GPIO_mode */

    uint32_t Pull;       /*!< TODO: not used. Specifies the Pull-up or Pull-Down activation for the selected pins.
                           This parameter can be a value of @ref GPIO_pull */
} GPIO_InitTypeDef;

/**
  * @brief  GPIO Bit SET and Bit RESET enumeration
  */
typedef enum
{
    GPIO_PIN_RESET = 0U,
    GPIO_PIN_SET
} GPIO_PinState;


/**
  * @brief   GPIO save&restore buffer type definition
*/
typedef struct
{
    __IO uint32_t DOR;
    __IO uint32_t DOER;
    __IO uint32_t IER;
    __IO uint32_t ITR;
#ifndef SF32LB55X
    __IO uint32_t IPHR;
    __IO uint32_t IPLR;
    __IO uint32_t IER_EXT;
#else
    __IO uint32_t IPSR;
    __IO uint32_t IPCR;
#endif /* SF32LB55X */
} GPIOxRestore_TypeDef;
/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup GPIO_Exported_Constants GPIO Exported Constants
  * @{
  */

/** @defgroup GPIO_mode GPIO mode
  * @brief GPIO Configuration Mode
  *        Elements values convention: 0xX0yz00YZ
  *           - X  : GPIO mode or EXTI Mode
  *           - y  : External IT or Event trigger detection
  *           - z  : IO configuration on External IT or Event
  *           - Y  : Output type (Push Pull or Open Drain)
  *           - Z  : IO Direction mode (Input, Output, Alternate or Analog)
  * @{
  */
#define  GPIO_MODE_INPUT                        (0x00000000u)   /*!< Input Floating Mode                   */
#define  GPIO_MODE_OUTPUT                       (0x00000001u)   /*!< Output Push Pull Mode                 */
#define  GPIO_MODE_OUTPUT_OD                    (0x00000011u)   /*!< Output Open Drain Mode                 */
#define  GPIO_MODE_IT_RISING                    (0x10110000u)   /*!< External Interrupt Mode with Rising edge trigger detection          */
#define  GPIO_MODE_IT_FALLING                   (0x10210000u)   /*!< External Interrupt Mode with Falling edge trigger detection         */
#define  GPIO_MODE_IT_RISING_FALLING            (0x10310000u)   /*!< External Interrupt Mode with Rising/Falling edge trigger detection  */
#define  GPIO_MODE_IT_HIGH_LEVEL                (0x10410000u)   /*!< External Interrupt Mode with high level detection          */
#define  GPIO_MODE_IT_LOW_LEVEL                 (0x10510000u)   /*!< External Interrupt Mode with low level detection          */
/**
  * @}
  */


/** @defgroup GPIO_pull GPIO pull
  * @brief GPIO Pull-Up or Pull-Down Activation
  * @{
  */
#define  GPIO_NOPULL        (0x00000000u)   /*!< No Pull-up or Pull-down activation  */
#define  GPIO_PULLUP        (0x00000001u)   /*!< Pull-up activation                  */
#define  GPIO_PULLDOWN      (0x00000002u)   /*!< Pull-down activation                */
/**
  * @}
  */

/*The maximum gpios in each group*/
#define GPIO1_PIN_NUM   (96)
#define GPIO2_PIN_NUM   (64)


/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup GPIO_Exported_Macros GPIO Exported Macros
  * @{
  */

/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @addtogroup GPIO_Private_Macros GPIO Private Macros
  * @{
  */
#define IS_GPIO_PIN_ACTION(ACTION)  (((ACTION) == GPIO_PIN_RESET) || ((ACTION) == GPIO_PIN_SET))

#define IS_GPIO_MODE(__MODE__)      (((__MODE__) == GPIO_MODE_INPUT)              ||\
                                     ((__MODE__) == GPIO_MODE_OUTPUT)             ||\
                                     ((__MODE__) == GPIO_MODE_OUTPUT_OD)          ||\
                                     ((__MODE__) == GPIO_MODE_IT_RISING)          ||\
                                     ((__MODE__) == GPIO_MODE_IT_FALLING)         ||\
                                     ((__MODE__) == GPIO_MODE_IT_RISING_FALLING)  ||\
                                     ((__MODE__) == GPIO_MODE_IT_HIGH_LEVEL)  ||\
                                     ((__MODE__) == GPIO_MODE_IT_LOW_LEVEL))

#define IS_GPIO_PULL(__PULL__)      (((__PULL__) == GPIO_NOPULL)   ||\
                                     ((__PULL__) == GPIO_PULLUP)   || \
                                     ((__PULL__) == GPIO_PULLDOWN))
/**
  * @}
  */


/* Exported functions --------------------------------------------------------*/
/** @addtogroup GPIO_Exported_Functions GPIO Exported Functions
  * @{
  */

/** @addtogroup GPIO_Exported_Functions_Group1 Initialization/de-initialization functions
 *  @brief    Initialization and Configuration functions
 * @{
 */

/* Initialization and de-initialization functions *****************************/
/**
 * @brief  Initialize the GPIO peripheral according to the specified parameters in the GPIO_Init.
 * @param  hgpio GPIO handle
 *         This parameter can be one of the following values:
 *            @arg @ref hwp_gpio1 Set hcpu pin
 *            @arg @ref hwp_gpio2 Set lcpu pin
 * @param  GPIO_Init: pointer to a GPIO_InitTypeDef structure that contains
 *         the configuration information for the specified GPIO peripheral.
 * @retval None
 */
void              HAL_GPIO_Init(GPIO_TypeDef *hgpio, GPIO_InitTypeDef *GPIO_Init);
/**
 * @brief  De-initialize the GPIOx peripheral registers to their default reset values.
 * @param  hgpio GPIO handle
 * @param  GPIO_Pin specifies the pin index to be written, starting from 0.
 * @retval None
 */
void              HAL_GPIO_DeInit(GPIO_TypeDef *hgpio, uint32_t GPIO_Pin);

/**
  * @}
  */

/** @addtogroup GPIO_Exported_Functions_Group2 IO operation functions
 * @{
 */

/* IO operation functions *****************************************************/
/**
 * @brief  Read the specified input port pin.
 * @param  hgpio GPIO handle
 * @param  GPIO_Pin specifies the pin index to be read, starting from 0.
 * @retval The input port pin value.
 */
GPIO_PinState     HAL_GPIO_ReadPin(GPIO_TypeDef *hgpio, uint16_t GPIO_Pin);

/**
 * @brief  Set or clear the selected data port bit.
 *
 * @param  hgpio GPIO handle
 * @param  GPIO_Pin specifies the pin index to be written, starting from 0.
 *
 * @param  PinState specifies the value to be written to the selected bit.
 *         This parameter can be one of the GPIO_PinState enum values:
 *            @arg GPIO_PIN_RESET: to clear the port pin
 *            @arg GPIO_PIN_SET: to set the port pin
 * @retval None
 */
void              HAL_GPIO_WritePin(GPIO_TypeDef *hgpio, uint16_t GPIO_Pin, GPIO_PinState PinState);
/**
 * @brief  Toggle the specified GPIO pin.
 * @param  hgpio GPIO handle
 * @param  GPIO_Pin specifies the pin index to be toggoled, starting from 0.
 * @retval None
 */
void              HAL_GPIO_TogglePin(GPIO_TypeDef *hgpio, uint16_t GPIO_Pin);
/**
 * @brief  Handle EXTI interrupt request.
 * @param  hgpio GPIO handle
 * @param  GPIO_Pin Specifies the port pin connected to corresponding EXTI line, starting from 0.
 * @retval None
 */
void              HAL_GPIO_EXTI_IRQHandler(GPIO_TypeDef *hgpio, uint16_t GPIO_Pin);
/**
 * @brief  EXTI line detection callback.
 * @param  hgpio GPIO handle
 * @param  GPIO_Pin: Specifies the port pin connected to corresponding EXTI line, starting from 0.
 * @retval None
 */
void              HAL_GPIO_EXTI_Callback(GPIO_TypeDef *hgpio, uint16_t GPIO_Pin);


/**
 * @brief  Get the GPIO mode.
 * @param  hgpio GPIO handle
 * @param  GPIO_Pin: Specifies the port pin connected to corresponding EXTI line, starting from 0.
 * @retval mode in @ref GPIO_mode, return UINT32_MAX if invalid
 */
uint32_t HAL_GPIO_GetMode(GPIO_TypeDef *hgpio, uint16_t GPIO_Pin);

/**
 * @brief  Handle all GPIO interrupt request which belong current group.
 * @param  hgpio GPIO handle
 * @retval None
 */
void HAL_GPIO_IRQHandler(GPIO_TypeDef *hgpio);

/**
 * @brief Save gpio instance's all configuration to buffer.
 * @param hgpio - GPIO handle
 * @param p_buf - Buffer for saving GPIO configuration
 * @param buf_cnt - Buffer numbers
 * @return Hal status error
 */
HAL_StatusTypeDef HAL_GPIO_Save(GPIO_TypeDef *hgpio, GPIOxRestore_TypeDef *p_buf, uint32_t buf_cnt);

/**
 * @brief Restore gpio instance's all configuration from buffer.
 * @param hgpio - GPIO handle
 * @param p_buf - Buffer for restore GPIO configuration
 * @param buf_cnt - Buffer numbers
 * @return Hal status error
 */
HAL_StatusTypeDef HAL_GPIO_Restore(GPIO_TypeDef *hgpio, GPIOxRestore_TypeDef *p_buf, uint32_t buf_cnt);

/**
 * @brief Clear all interrupt
 *
 * @param hgpio GPIO handle
 *
 * @return Hal status error
 */
HAL_StatusTypeDef HAL_GPIO_ClearInterrupt(GPIO_TypeDef *hgpio);


/**
 * @brief Clear specified pin interrupt
 *
 * @param hgpio GPIO handle
 * @param GPIO_Pin specifies the pin index to be clear, starting from 0.
 *
 * @return HAL_OK if IRQ been clear
 */
HAL_StatusTypeDef HAL_GPIO_ClearPinInterrupt(GPIO_TypeDef *hgpio, uint16_t GPIO_Pin);

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

#endif /* __BF0_HAL_GPIO_H */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
