/**
  ******************************************************************************
  * @file   bf0_hal_bleaon.c
  * @author Sifli software development team
  * @brief   AON HAL module driver.
  *          This file provides firmware functions to manage the following
  ******************************************************************************
*/
/**
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

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
