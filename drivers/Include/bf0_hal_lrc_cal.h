/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __BF0_HAL_LRC_CAL_H
#define __BF0_HAL_LRC_CAL_H

#ifdef __cplusplus
extern "C" {
#endif


#ifdef __cplusplus
}
#endif


/* Includes ------------------------------------------------------------------*/
#include "bf0_hal_def.h"
/** @addtogroup BF0_HAL_Driver
  * @{
  */


/** @defgroup LRCCAL Low power RC calibration
  * @brief Low power RC frequency calibration
  * @{
  */


/**
  * @defgroup LRCCAL_exported_functions Lower poer RC calibration Exported functions
  * @ingroup LRCCAL
  * @{
  *
 */



uint32_t HAL_RC_CAL_get_reference_cycle_on_48M(void);

uint32_t HAL_RC_CAL_get_average_cycle_on_48M(void);

void HAL_RC_CAL_update_ave_cycle(uint32_t ave_cycle);


#define HAL_RC_CAL_update_reference_cycle_on_48M(lp_cycle) HAL_RC_CAL_update_reference_cycle_on_48M_ex(lp_cycle,0,100)
/**
* @brief  Get actual frequency of low power RC. The acutal freqency = 48000000 * lp_cycle / ref_cycle
* @param lp_cycle Target RC cycles to get according reference cycles of 48M crystal clock
* @param clear_ave clear average counts.
* @param ave_window average window counts.
* @retval ref_cycle Cycles of 48000000 crystal clock that equals to lp_cycle
*/
int HAL_RC_CAL_update_reference_cycle_on_48M_ex(uint8_t lp_cycle, int clear_ave, int ave_window);


/**
  * @brief  Return LPTIM freq.
  * @retval freq
  */
float HAL_LPTIM_GetFreq();

int HAL_RC_CAL_SetParameter(uint8_t *lp_cycle, uint16_t *thd0, uint16_t *thd1);

uint8_t HAL_RC_CAL_GetLPCycle(void);

uint8_t HAL_RC_CAL_GetLPCycle_ex(void);

int HAL_RC_CAL_SetLPCycle_ex(uint8_t lpcycle);

int HAL_RC_CALget_curr_cycle_on_48M(uint8_t lp_cycle, uint32_t *count);

/**
 * @brief  Delay function used by LRC calibration
 *
 * It's a weak function that can be overridden by user. The default implementation is to call HAL_Delay which uses while loop and would not yield the CPU.
 *
 */
void HAL_LRC_Delay(uint32_t Delay);


/**
  *@} LRCCAL_exported_functions
*/




/**
  *@} LRCCAL
  */


/**
  *@} BF0_HAL_Driver
  */


#endif // __BF0_HAL_LRC_CAL_H

/**
  *@}
  */