/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
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