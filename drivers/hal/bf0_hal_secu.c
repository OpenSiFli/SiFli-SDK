/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bf0_hal.h"

/** @addtogroup BF0_HAL_Driver
 * @{
 */

/** @defgroup SECU Security
 * @brief Security HAL module driver
 * @{
 */
#if defined(HAL_SECU_MODULE_ENABLED)||defined(_SIFLI_DOXYGEN_)

    #if (SECU_VERSION == 1)
        #include "secu/bf0_hal_secu_v1.c"
    #elif (SECU_VERSION == 2)
        #include "secu/bf0_hal_secu_v2.c"
    #else
        #error "SECU HAL Driver Version Error."
    #endif

#endif
/**
 * @}
 */

/**
 * @}
 */