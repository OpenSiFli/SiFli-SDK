/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtconfig.h"

#ifndef BSP_USING_PC_SIMULATOR
#include "register.h"
#include "drv_io.h"

HAL_RAM_RET_CODE_SECT(HAL_MspInit, __weak void HAL_MspInit(void))
{
    // HAL_sw_breakpoint();        /*For debugging purpose*/
    BSP_IO_Init();
}

__weak void BSP_SD2_PowerUp(void)
{
    // Default implementation - can be overridden by board-specific implementation
}

__weak void BSP_SD_PowerUp(void)
{
    // Default implementation - can be overridden by board-specific implementation
}

#endif /* BSP_USING_PC_SIMULATOR */
