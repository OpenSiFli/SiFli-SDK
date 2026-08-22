/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "board.h"
#include "boot_flash.h"
#include "string.h"

void bootloader_switch_clock(int mpi)
{
#ifdef BSP_USING_PSRAM1
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_PSRAM1, RCC_CLK_FLASH_DLL2);
#endif /* BSP_USING_PSRAM1 */    
#ifdef BSP_USING_PSRAM2
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_PSRAM2, RCC_CLK_FLASH_DLL2);
#endif /* BSP_USING_PSRAM2 */    

}

void board_init_psram()
{
#if defined(BSP_USING_PSRAM1) || defined(BSP_USING_PSRAM2)
    HAL_PMU_ConfigPeriLdo(PMU_PERI_LDO_1V8, true, true);
#endif /* BSP_USING_PSRAM1 || BSP_USING_PSRAM2 */

#ifndef CFG_BOOTROM
    bootloader_switch_clock(1);
#endif
    BSP_SetFlash1DIV(2);    // for QSPI PSRAM need set divider to avoid to high, OPI PSRAM do not care.

    bsp_psramc_init();
}


