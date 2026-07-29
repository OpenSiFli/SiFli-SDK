/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __BF0_HAL_TSEN_H
#define __BF0_HAL_TSEN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal_def.h"

/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @defgroup TSEN Temperature sensor
  * @brief Temperature sensor module driver
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup TSEN_Exported_Types TSEN Exported Types
  * @{
  */

/** @defgroup TSEN_Exported_Types_Group1 Temperature sensor State Structure definition
  * @{
  */
typedef enum
{
    HAL_TSEN_STATE_RESET     = 0x00U,  /*!< TSEN not yet initialized or disabled */
    HAL_TSEN_STATE_READY     = 0x01U,  /*!< TSEN initialized and ready for use   */
    HAL_TSEN_STATE_ENABLED   = 0x03U,  /*!< TSEN Power on and enabled */
    HAL_TSEN_STATE_BUSY      = 0x04U,  /*!< TSEN internal process is ongoing     */
    HAL_TSEN_STATE_ERROR     = 0x05U,  /*!< TSEN error state                     */
} HAL_TSEN_StateTypeDef;

/**
  * @}
  */

/** @defgroup TSEN_Exported_Types_Group2 TSEN Handle Structure definition
  * @{
  */
typedef struct
{
    TSEN_TypeDef                *Instance;  /*!< Register base address   */
    HAL_LockTypeDef             Lock;       /*!< TSEN locking object      */
    __IO HAL_TSEN_StateTypeDef  State;      /*!< TSEN communication state */
    int8_t                      temperature; /*!< last read temperature*/
    uint32_t                    pclk;        /*!< PCLK in Hz */
} TSEN_HandleTypeDef;

/**
  * @}
  */

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @defgroup TSEN_Exported_Functions TSEN Exported Functions
  * @{
  */

/** @defgroup TSEN_Exported_Functions_Group1 Initialization and de-initialization functions
  * @{
  */
/**
  * @brief  Initializes the Temperature sensor
  * @param  htsen pointer to a TSEN_HandleTypeDef structure.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_TSEN_Init(TSEN_HandleTypeDef *htsen);

/**
  * @brief  DeInitializes the Temperature sensor.
  * @param  htsen pointer to a TSEN_HandleTypeDef structure.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_TSEN_DeInit(TSEN_HandleTypeDef *htsen);
/**
  * @}
  */

/** @defgroup TSEN_Exported_Functions_Group2 Peripheral Control functions
  * @{
  */


/**
  * @brief  Temeprature readying convert to Centidegree.
  * @param  htsen pointer to a TSEN_HandleTypeDef structure.
  */
#define HAL_TSEN_Data(htsen) ((((int32_t)(htsen->Instance->TSEN_RDATA))+3000)*749/10100-277)

#define HAL_ERROR_TEMPRATURE    1000                /*<! -1000 means ready error*/
#define HAL_TSEN_MAX_DELAY      10                  /*<! Max delay for temperature read, 20ms*/

/**
  * @brief  Synchronized Read the current temperature.
  * @param  htsen pointer to a TSEN_HandleTypeDef structure.
  * @retval Current temeprature of chipset in Centidegree.
            -1000 if error.
  */
int HAL_TSEN_Read(TSEN_HandleTypeDef *htsen);


/**
  * @brief  Interrupt handler for temeprature sensor.
  * @param  htsen pointer to a TSEN_HandleTypeDef structure.
  */
void HAL_TSEN_IRQHandler(TSEN_HandleTypeDef *htsen);


/**
  * @brief  Async Read the current temperature.
  * @param  htsen pointer to a TSEN_HandleTypeDef structure.
  * @retval Current state of temperature sensor module, temperature data will be read in interrupt
  */
HAL_TSEN_StateTypeDef HAL_TSEN_Read_IT(TSEN_HandleTypeDef *htsen);

/**
  * @}
  */

/** @defgroup TSEN_Exported_Functions_Group3 Peripheral State functions
  * @{
  */
/**
  * @brief  Get current state of temperature sensor module
  * @param  htsen pointer to a TSEN_HandleTypeDef structure.
  * @retval Current state of temperature sensor module
*/
HAL_TSEN_StateTypeDef HAL_TSEN_GetState(TSEN_HandleTypeDef *htsen);

/**
  * @}  TSEN_Exported_Functions_Group3
  */

/**
  * @}  TSEN_Exported_Functions
  */

/**
  * @}  TSEN
  */

/**
  * @}  BF0_HAL_Driver
  */


#ifdef __cplusplus
}
#endif

#endif /* __BF0_HAL_TSEN_H */