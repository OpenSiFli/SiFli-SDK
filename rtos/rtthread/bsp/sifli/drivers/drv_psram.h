/**
  ******************************************************************************
  * @file   drv_psram.h
  * @author Sifli software development team
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

#ifndef __DRV_PSRAM_H__
#define __DRV_PSRAM_H__

#include <rtthread.h>
#include "rtdevice.h"
#include <rthw.h>
#include <drv_common.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*** define OPI psram base address for memory **/
#define PSRAM_BASE_ADDR             (PSRAM_BASE)


/* output api */
/**
 * @brief PSRAM hardware initial.
 * @param[none] .
 * @return 0 if initial success.
 */
int rt_psram_init(void);


/**
 * @brief Get PSRAM clock frequency.
 * @param addr PSRAM address
 * @return Clock freqency for PSRAM
 */
uint32_t rt_psram_get_clk(uint32_t addr);

/**
 * @brief PSRAM hardware enter low power.
 * @param name name of PSRAM controller.
 * @return RT_EOK if initial success, otherwise, -RT_ERROR.
 */
int rt_psram_enter_low_power(char *name);

int rt_psram_exit_low_power(char *name);


/**
 * @brief PSRAM set partial array self-refresh.
 * @param name name of PSRAM controller.
 * @param top set top part to self-refresh, else set bottom.
 * @param deno denomenator for refresh, like 2 for 1/2 to refresh, only support 2^n,
 *         when larger than 16, all memory not refresh, when 1 or 0, all meory auto refress by default.
 * @return 0 if success.
 */
int rt_psram_set_pasr(char *name, uint8_t top, uint8_t deno);


/**
 * @brief PSRAM auto calibrate delay.
 * @param name name of PSRAM controller.
 * @param sck sck delay pointer.
 * @param dqs dqs delay pointer
 * @return 0 if success.
 */
int rt_psram_auto_calib(char *name, uint8_t *sck, uint8_t *dqs);

#ifndef SF32LB55X
void rt_psram_wait_idle(char *name);
#endif

#ifdef __cplusplus
}
#endif

#endif  /* __DRV_PSRAM_H__ */
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
