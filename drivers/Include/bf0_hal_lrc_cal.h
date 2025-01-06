/**
  ******************************************************************************
  * @file   bf0_hal_lrc_cal.h
  * @author Sifli software development team
  * @brief Header file of low power RC calibration
  * @{
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2021 - 2021,  Sifli Technology
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


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/


