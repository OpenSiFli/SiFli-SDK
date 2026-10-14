/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __TIM_CONFIG_H__
#define __TIM_CONFIG_H__

#include <rtconfig.h>
#include "bf0_hal_rcc.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef LXT_FREQ
#define LXT_FREQ 32768
#endif

#define GPTIM1_CORE CORE_ID_HCPU
#define GPTIM2_CORE CORE_ID_HCPU
#define ATIM1_CORE CORE_ID_HCPU
#define ATIM2_CORE CORE_ID_HCPU
#define BTIM1_CORE CORE_ID_HCPU
#define BTIM2_CORE CORE_ID_HCPU
#define BTIM3_CORE CORE_ID_LCPU
#define BTIM4_CORE CORE_ID_LCPU
#define LPTIM1_CORE CORE_ID_HCPU
#define LPTIM2_CORE CORE_ID_HCPU
#define LPTIM3_CORE CORE_ID_LCPU

#ifndef GPT_DEV_INFO_CONFIG
#define GPT_DEV_INFO_CONFIG                     \
    {                                           \
        .maxfreq = 1000000,                     \
        .minfreq = 2000,                        \
        .maxcnt  = 0xFFFF,                      \
        .cntmode = HWTIMER_CNTMODE_UP,          \
    }
#endif /* TIM_DEV_INFO_CONFIG */

#ifndef LPTIM_DEV_INFO_CONFIG
#define LPTIM_MAX_CNT 0xFFFFFF
#define LPTIM_DEV_INFO_CONFIG                   \
    {                                           \
        .maxfreq = LXT_FREQ,                    \
        .minfreq = 256,                         \
        .maxcnt  = LPTIM_MAX_CNT,               \
        .cntmode = HWTIMER_CNTMODE_UP,          \
    }
#endif /* TIM_DEV_INFO_CONFIG */

#ifdef BSP_USING_GPTIM1
#define GPTIM1_CONFIG                           \
    {                                           \
       .tim_handle.Instance     = GPTIM1,       \
       .tim_irqn                = GPTIM1_IRQn,  \
       .name                    = "gptim1",     \
       .core                    = GPTIM1_CORE, \
    }
#endif /* BSP_USING_GPTIM1 */

#ifdef BSP_USING_GPTIM2
#define GPTIM2_CONFIG                           \
    {                                           \
       .tim_handle.Instance     = GPTIM2,       \
       .tim_irqn                = GPTIM2_IRQn,  \
       .name                    = "gptim2",     \
       .core                    = GPTIM2_CORE, \
    }
#endif /* BSP_USING_GPTIM2 */

#ifdef BSP_USING_ATIM1
#define ATIM1_CONFIG                            \
        {                                           \
           .tim_handle.Instance     = (GPT_TypeDef *)ATIM1,         \
           .tim_irqn                = ATIM1_IRQn,   \
           .name                    = "atim1",      \
           .core                    = ATIM1_CORE,   \
        }
#endif /* BSP_USING_BTIM1 */

#ifdef BSP_USING_ATIM2
#define ATIM2_CONFIG                             \
        {                                            \
           .tim_handle.Instance     = (GPT_TypeDef *)ATIM2,         \
           .tim_irqn                = ATIM2_IRQn,    \
           .name                    = "atim2",       \
           .core                    = ATIM2_CORE,   \
        }
#endif /* BSP_USING_BTIM2 */

#ifdef BSP_USING_BTIM1
#define BTIM1_CONFIG                            \
    {                                           \
       .tim_handle.Instance     = (GPT_TypeDef *)BTIM1,         \
       .tim_irqn                = BTIM1_IRQn,   \
       .name                    = "btim1",      \
       .core                    = BTIM1_CORE,   \
    }
#endif /* BSP_USING_BTIM1 */

#ifdef BSP_USING_BTIM2
#define BTIM2_CONFIG                             \
    {                                            \
       .tim_handle.Instance     = (GPT_TypeDef *)BTIM2,         \
       .tim_irqn                = BTIM2_IRQn,    \
       .name                    = "btim2",       \
       .core                    = BTIM2_CORE,   \
    }
#endif /* BSP_USING_BTIM2 */

#ifdef BSP_USING_BTIM3
#define BTIM3_CONFIG                             \
    {                                            \
       .tim_handle.Instance     = (GPT_TypeDef *)BTIM3,         \
       .tim_irqn                = BTIM3_IRQn,    \
       .name                    = "btim3",       \
       .core                    = BTIM3_CORE,    \
    }
#endif /* BSP_USING_BTIM3 */

#ifdef BSP_USING_BTIM4
#define BTIM4_CONFIG                            \
    {                                           \
       .tim_handle.Instance     = (GPT_TypeDef *)BTIM4,         \
       .tim_irqn                = BTIM4_IRQn,   \
       .name                    = "btim4",      \
       .core                    = BTIM4_CORE,    \
    }
#endif /* BSP_USING_BTIM4 */

#ifdef BSP_USING_LPTIM1
#define LPTIM1_CONFIG                             \
    {                                            \
       .tim_handle.Instance     = hwp_lptim1,        \
       .tim_irqn                = LPTIM1_IRQn,   \
       .name                    = "lptim1",     \
    }
#endif /* BSP_USING_LPTIM1 */

#ifdef BSP_USING_LPTIM2
#define LPTIM2_CONFIG                             \
    {                                            \
       .tim_handle.Instance     = hwp_lptim2,    \
       .tim_irqn                = LPTIM2_IRQn,   \
       .name                    = "lptim2",     \
    }
#endif /* BSP_USING_LPTIM2 */

#ifdef BSP_USING_LPTIM3
#define LPTIM3_CONFIG                             \
    {                                            \
       .tim_handle.Instance     = hwp_lptim3,        \
       .tim_irqn                = LPTIM3_IRQn,   \
       .name                    = "lptim3",     \
    }
#endif /* BSP_USING_LPTIM3 */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_CONFIG_H__ */

