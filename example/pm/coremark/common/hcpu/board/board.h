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

#include <rtthread.h>
#include <register.h>
#include "bsp_board.h"
#include "drv_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef SF32LB56X
#define custom_printf(...)                    \
    do                                        \
    {                                         \
        rt_kprintf(__VA_ARGS__);              \
    }                                         \
    while (0)
#endif /* SF32LB56X */

#ifdef SF32LB58X
#define custom_printf(...)                    \
    do                                        \
    {                                         \
        HAL_RCC_EnableModule(RCC_MOD_MPI1);   \
        rt_kprintf(__VA_ARGS__);              \
        HAL_RCC_DisableModule(RCC_MOD_MPI1);  \
    }                                         \
    while (0)
#endif /* SF32LB58X */

#ifdef SF32LB52X

void rt_flash_wait_idle(uint32_t addr);

#define custom_printf(...)                    \
    do                                        \
    {                                         \
        HAL_RCC_EnableModule(RCC_MOD_MPI1);   \
        HAL_RCC_EnableModule(RCC_MOD_MPI2);   \
        rt_kprintf(__VA_ARGS__);              \
        rt_flash_wait_idle(MPI2_MEM_BASE);    \
        rt_flash_wait_idle(MPI1_MEM_BASE);    \
        HAL_RCC_DisableModule(RCC_MOD_MPI2);  \
        HAL_RCC_DisableModule(RCC_MOD_MPI1);  \
    }                                         \
    while (0)
#endif /* SF32LB52X */

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H__ */
