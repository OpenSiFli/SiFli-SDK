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

#ifdef HAL_BLE_AON_MODULE_ENABLED

/**
 * @brief  BLESYS wakeup the specified HPSYS or LPSYS
 * @param  core_id core id, CORE_ID_HCPU or CORE_ID_LCPU
 * @retval status
 */
HAL_StatusTypeDef HAL_BLEAON_WakeCore(uint8_t core_id)
{
    HAL_StatusTypeDef ret = HAL_OK;
    if (core_id == CORE_ID_HCPU)
    {
        hwp_ble_aon->ISSR |= BLE_AON_ISSR_BLE2HP_REQ;
        while (!(hwp_ble_aon->ISSR & BLE_AON_ISSR_HP_ACTIVE));
    }
    else if (core_id == CORE_ID_LCPU)
    {
        hwp_ble_aon->ISSR |= BLE_AON_ISSR_BLE2LP_REQ;
        while (!(hwp_ble_aon->ISSR & BLE_AON_ISSR_LP_ACTIVE));
    }
    else
    {
        ret = HAL_ERROR;
    }

    return ret;
}

/**
 * @brief  Enable BLESYS wakeup source
 * @param  src wakeup source
 * @retval status
 */
HAL_StatusTypeDef HAL_BLEAON_EnableWakeupSrc(BLEAON_WakeupSrcTypeDef src)
{
    hwp_ble_aon->WER |= (1UL << src);
    return HAL_OK;
}

/**
 * @brief  Disable specified BLESYS wakeup source
 * @param  src wakeup source
 * @retval status
 */
HAL_StatusTypeDef HAL_BLEAON_DisableWakeupSrc(BLEAON_WakeupSrcTypeDef src)
{
    hwp_ble_aon->WER &= ~(1UL << src);
    return HAL_OK;
}

void HAL_BLEAON_DisableRC(void)
{
    hwp_ble_aon->CCR &= ~BLE_AON_CCR_HRC48_EN;
}


#endif /* HAL_AON_MODULE_ENABLED */
/**
 * @}
 */

/**
 * @}
 */
