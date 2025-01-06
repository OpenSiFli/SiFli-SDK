/**
  ******************************************************************************
  * @file   bf0_hal_patch.h
  * @author Sifli software development team
  * @brief Handle patch for BCPU core
  ******************************************************************************
*/
/*
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

#ifndef __BF0_HAL_PATCH_H
#define __BF0_HAL_PATCH_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal_def.h"

/** @defgroup PATCH ROM Patch
  * @ingroup BF0_HAL_Driver
  * @brief Handle patch for ROM
  * @{
  */


#ifdef SOC_SF32LB55X
#define PATCH_AON           4
#else
#define PATCH_AON           0
#endif
#define MAX_PATCH_ENTRIES   32

struct patch_entry_desc
{
    uint32_t break_addr;     // break out address, should be 4 bytes-aligned (requreied by hardware)
    uint32_t data;              // data replaced.
};

#define QUOTE_ME(X) #X
#ifndef PATCH_GENERATE
#define PATCH(address,offset,func)
#define PATCH_DATA(address,offset,data)
#endif

#define PATCH_TAG 0x50544348

/**
  * @brief  Advanced API to install patches for LCPU/BCPU
  * @param  patch_entries Patch descriptor arrays
  * @param  size size of patch_entries in bytes.
  * @param  cer Previous patch enable register value, used in restore patch from standby.
  * @retval 32-bit value, each bit represent on patch installed.
*/
int HAL_PATCH_install2(struct patch_entry_desc *patch_entries, uint32_t size, int cer);


/**
  * @brief  Install patches LCPU/BCPU
  * @retval 32-bit value, each bit represent on patch installed.
*/
int HAL_PATCH_install(void);

/**
  * @brief  Save patches
  * @param  patch_entries Patch descriptor arrays
  * @param  size size of patch_entries in bytes.
  * @param[out]  cer Previous patch enable register value, used in restore patch from standby.
  * @retval Save patch numbers.
*/
int HAL_PATCH_save(struct patch_entry_desc *patch_entries, uint32_t size, uint32_t *cer);

/**
  * @brief  Patch image entry, used to install hooks
*/
void HAL_PATCH_Entry(void);


#if 0
__asm("pro_"QUOTE_ME(func)":\n" \
      "    ADD SP, #4\n" \
      "    b "QUOTE_ME(func)" \n")
#endif

/// @} PATCH

#ifdef __cplusplus
}
#endif

#endif

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
