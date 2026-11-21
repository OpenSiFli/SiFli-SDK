/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bf0_hal.h"

/** @addtogroup BF0_HAL_Driver
 * @{
 */

/** @defgroup AON AON
 * @brief AON HAL module driver
 * @{
 */

#ifdef HAL_AON_MODULE_ENABLED

HAL_StatusTypeDef HAL_LPAON_GetWakeupPinMode(uint8_t wakeup_pin, AON_PinModeTypeDef *mode)
{
    return HAL_ERROR;
}


#endif /* HAL_AON_MODULE_ENABLED */
/**
 * @}
 */

/**
 * @}
 */