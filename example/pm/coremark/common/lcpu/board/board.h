/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-5      SummerGift   first version
 */

#ifndef __BOARD_H__
#define __BOARD_H__

#include "rtconfig.h"
#include "bf0_hal.h"
#include "bsp_board.h"
#ifdef BSP_USING_RTTHREAD
    #include "drv_common.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif


#ifdef SF32LB58X
#define custom_printf(...)                    \
    do                                        \
    {                                         \
        hwp_lpsys_cfg->ULPMCR &= ~0x40000000; \
        __NOP();                              \
        __NOP();                              \
        __NOP();                              \
        __NOP();                              \
        __NOP();                              \
        __NOP();                              \
        __NOP();                              \
        __NOP();                              \
        __NOP();                              \
        rt_kprintf(__VA_ARGS__);              \
        __NOP();                              \
        __NOP();                              \
        __NOP();                              \
        __NOP();                              \
        __NOP();                              \
        __NOP();                              \
        __NOP();                              \
        __NOP();                              \
        __NOP();                              \
        hwp_lpsys_cfg->ULPMCR |= 0x40000000;  \
        __NOP();                              \
        __NOP();                              \
        __NOP();                              \
        __NOP();                              \
        __NOP();                              \
        __NOP();                              \
        __NOP();                              \
        __NOP();                              \
        __NOP();                              \
    }                                         \
    while (0)

#endif /* SF32LB58X */

#ifdef SF32LB56X
#define custom_printf(...)                    \
    do                                        \
    {                                         \
        hwp_lpsys_cfg->ULPMCR &= ~0x40000000; \
        __NOP();                              \
        __NOP();                              \
        __NOP();                              \
        __NOP();                              \
        __NOP();                              \
        __NOP();                              \
        __NOP();                              \
        __NOP();                              \
        __NOP();                              \
        HAL_RCC_EnableModule(RCC_MOD_MPI5);   \
        rt_kprintf(__VA_ARGS__);              \
        HAL_RCC_DisableModule(RCC_MOD_MPI5);  \
        __NOP();                              \
        __NOP();                              \
        __NOP();                              \
        __NOP();                              \
        __NOP();                              \
        __NOP();                              \
        __NOP();                              \
        __NOP();                              \
        __NOP();                              \
        hwp_lpsys_cfg->ULPMCR |= 0x40000000;  \
        __NOP();                              \
        __NOP();                              \
        __NOP();                              \
        __NOP();                              \
        __NOP();                              \
        __NOP();                              \
        __NOP();                              \
        __NOP();                              \
        __NOP();                              \
    }                                         \
    while (0)
#endif /* SF32LB56X */


#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H__ */
