/**
  ******************************************************************************
  * @file   bf0_hal_wdt.c
  * @author Sifli software development team
  * @brief   Watch dog timer module.
  *          This is the common part of the HAL initialization
  ******************************************************************************
*/
/**
 *
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
        hwdt->Instance = hwp_wdt1;
#else
        hwdt->Instance = hwp_wdt2;
#endif
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

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

