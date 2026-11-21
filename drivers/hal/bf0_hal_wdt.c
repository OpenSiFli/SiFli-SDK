/*
 * SPDX-FileCopyrightText: 2016 STMicroelectronics
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: BSD-3-Clause AND Apache-2.0
 */

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal.h"


/** @addtogroup BF0_HAL_Driver
 * @{
 */

#ifdef HAL_WDT_MODULE_ENABLED
/** @defgroup WDT WDT
 * @brief WDT HAL module driver.
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @addtogroup WDT_Exported_Functions
 * @{
 */

/** @addtogroup WDT_Exported_Functions_Group1
 *  @brief    Initialization and Start functions.
 *
@verbatim
 ===============================================================================
          ##### Initialization and Start functions #####
 ===============================================================================
 [..]  This section provides functions allowing to:
      (+) Initialize the WDT according to the specified parameters in the
          WDT_InitTypeDef of associated handle.
      (+) Once initialization is performed in HAL_WDT_Init function, Watchdog
          is reloaded in order to exit function with correct time base.

@endverbatim
 * @{
 */

/**
 * @brief  Initialize the WDT according to the specified parameters in the
 *         WDT_InitTypeDef and start watchdog. Before exiting function,
 *         watchdog is refreshed in order to have correct time base.
 * @param  hwdt  pointer to a WDT_HandleTypeDef structure that contains
 *                the configuration information for the specified WDT module.
 * @retval HAL status
 */
__HAL_ROM_USED HAL_StatusTypeDef HAL_WDT_Init(WDT_HandleTypeDef *hwdt)
{

    /* Check the WDT handle allocation */
    if (hwdt == NULL)
    {
        return HAL_ERROR;
    }

    if (hwdt->Instance == NULL)
    {
#ifdef SOC_BF0_HCPU
#ifdef hwp_wdt1
        hwdt->Instance = hwp_wdt1;
#else
        hwdt->Instance = hwp_iwdt;
#endif /* hwp_wdt1 */
#else
        hwdt->Instance = hwp_wdt2;
#endif /* SOC_BF0_HCPU */
    }

    /* Check the parameters */
    HAL_ASSERT(IS_WDT_ALL_INSTANCE(hwdt->Instance));
    HAL_ASSERT(IS_WDT_RELOAD(hwdt->Init.Reload));
    HAL_ASSERT(IS_WDT_RELOAD(hwdt->Init.Reload2));

    /* Stop WDT.*/
    if (hwdt->Instance->WDT_SR & WDT_WDT_SR_WDT_ACTIVE)
        __HAL_WDT_STOP(hwdt);

    __HAL_WDT_RELOAD_COUNTER(hwdt);
    __HAL_WDT_RELOAD_COUNTER2(hwdt);

    /* Enable WDT. LSI is turned on automaticaly */
    __HAL_WDT_START(hwdt);

    /* Return function status */
    return HAL_OK;
}

/**
 * @}
 */

/** @addtogroup WDT_Exported_Functions_Group2
 *  @brief   IO operation functions
 *
@verbatim
 ===============================================================================
                      ##### IO operation functions #####
 ===============================================================================
 [..]  This section provides functions allowing to:
      (+) Refresh the WDT.

@endverbatim
 * @{
 */

/**
 * @brief  Refresh the WDT.
 * @param  hwdt  pointer to a WDT_HandleTypeDef structure that contains
 *                the configuration information for the specified WDT module.
 * @retval HAL status
 */
__HAL_ROM_USED HAL_StatusTypeDef HAL_WDT_Refresh(WDT_HandleTypeDef *hwdt)
{
    __HAL_WDT_START(hwdt);

    /* Return function status */
    return HAL_OK;
}


/**
 * @}
 */

/**
 * @}
 */

#endif /* HAL_WDT_MODULE_ENABLED */
/**
 * @}
 */

/**
 * @}
 */
