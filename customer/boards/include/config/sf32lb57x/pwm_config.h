/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __PWM_CONFIG_H__
#define __PWM_CONFIG_H__

#include <rtconfig.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PWM1_CORE CORE_ID_HCPU
#define PWMT1_CORE CORE_ID_HCPU
#define PWMT2_CORE CORE_ID_HCPU
#define PWMA1_CORE CORE_ID_HCPU
#define PWMA2_CORE CORE_ID_HCPU


#ifdef BSP_USING_PWM1
#define PWM1_CONFIG                                                            \
    {                                                                          \
        .pwm_handle.Instance     = hwp_pwm1,                                   \
        .pwm_handle.core         = PWM1_CORE,                                  \
        .name                    = "pwm1",                                     \
        .channel                 = 0                                           \
    }
#endif /* BSP_USING_PWM1 */

#ifdef BSP_USING_PWMT1
#define PWMT1_CONFIG                                                           \
    {                                                                          \
        .tim_handle.Instance     = GPTIM1,                                     \
        .tim_handle.core         = PWMT1_CORE,                                 \
        .name                    = "pwmt1",                                    \
        .channel                 = 0                                           \
    }
#endif /* BSP_USING_PWMT1 */

#define PWMT1_UPDATE_DMA_CONFIG                                                \
    {                                                                          \
        .dma_handle.Init.Priority            = PWMT1_UPDATE_DMA_IRQ_PRIO,      \
        .dma_handle.Instance                 = PWMT1_UPDATE_DMA_INSTANCE,      \
        .dma_handle.Init.Request             = GPTIM1_UPDATE_DMA_REQUEST,      \
        .dma_irq                             = PWMT1_UPDATE_DMA_IRQ,           \
        .dma_handle_index                    = GPT_DMA_ID_UPDATE,              \
        .dma_handle.Init.PeriphDataAlignment = PWMT1_UPDATE_DMA_PDATAALIGN,    \
        .dma_handle.Init.MemDataAlignment    = PWMT1_UPDATE_DMA_MDATAALIGN     \
    }

#define PWMT1_CC1_DMA_CONFIG                                                   \
    {                                                                          \
        .dma_handle.Init.Priority            = PWMT1_CC1_DMA_IRQ_PRIO,         \
        .dma_handle.Instance                 = PWMT1_CC1_DMA_INSTANCE,         \
        .dma_handle.Init.Request             = PWMT1_CC1_DMA_REQUEST,          \
        .dma_irq                             = PWMT1_CC1_DMA_IRQ,              \
        .dma_handle_index                    = GPT_DMA_ID_CC1,                 \
        .dma_handle.Init.PeriphDataAlignment = PWMT1_CC1_DMA_PDATAALIGN,       \
        .dma_handle.Init.MemDataAlignment    = PWMT1_CC1_DMA_MDATAALIGN        \
    }

#define PWMT1_CC2_DMA_CONFIG                                                   \
    {                                                                          \
        .dma_handle.Init.Priority            = PWMT1_CC2_DMA_IRQ_PRIO,         \
        .dma_handle.Instance                 = PWMT1_CC2_DMA_INSTANCE,         \
        .dma_handle.Init.Request             = PWMT1_CC2_DMA_REQUEST,          \
        .dma_irq                             = PWMT1_CC2_DMA_IRQ,              \
        .dma_handle_index                    = GPT_DMA_ID_CC2,                 \
        .dma_handle.Init.PeriphDataAlignment = PWMT1_CC2_DMA_PDATAALIGN,       \
        .dma_handle.Init.MemDataAlignment    = PWMT1_CC2_DMA_MDATAALIGN        \
    }

#define PWMT1_CC3_DMA_CONFIG                                                   \
    {                                                                          \
        .dma_handle.Init.Priority            = PWMT1_CC3_DMA_IRQ_PRIO,         \
        .dma_handle.Instance                 = PWMT1_CC3_DMA_INSTANCE,         \
        .dma_handle.Init.Request             = PWMT1_CC3_DMA_REQUEST,          \
        .dma_irq                             = PWMT1_CC3_DMA_IRQ,              \
        .dma_handle_index                    = GPT_DMA_ID_CC3,                 \
        .dma_handle.Init.PeriphDataAlignment = PWMT1_CC3_DMA_PDATAALIGN,       \
        .dma_handle.Init.MemDataAlignment    = PWMT1_CC3_DMA_MDATAALIGN        \
    }

#define PWMT1_CC4_DMA_CONFIG                                                   \
    {                                                                          \
        .dma_handle.Init.Priority            = PWMT1_CC4_DMA_IRQ_PRIO,         \
        .dma_handle.Instance                 = PWMT1_CC4_DMA_INSTANCE,         \
        .dma_handle.Init.Request             = PWMT1_CC4_DMA_REQUEST,          \
        .dma_irq                             = PWMT1_CC4_DMA_IRQ,              \
        .dma_handle_index                    = GPT_DMA_ID_CC4,                 \
        .dma_handle.Init.PeriphDataAlignment = PWMT1_CC4_DMA_PDATAALIGN,       \
        .dma_handle.Init.MemDataAlignment    = PWMT1_CC4_DMA_MDATAALIGN        \
    }

#ifdef BSP_USING_PWMT2
#define PWMT2_CONFIG                                                           \
    {                                                                          \
        .tim_handle.Instance     = GPTIM2,                                     \
        .tim_handle.core         = PWMT2_CORE,                                 \
        .name                    = "pwmt2",                                    \
        .channel                 = 0                                           \
    }
#endif /* BSP_USING_PWMT2 */

#define PWMT2_UPDATE_DMA_CONFIG                                                \
    {                                                                          \
        .dma_handle.Init.Priority            = PWMT2_UPDATE_DMA_IRQ_PRIO,      \
        .dma_handle.Instance                 = PWMT2_UPDATE_DMA_INSTANCE,      \
        .dma_handle.Init.Request             = GPTIM2_UPDATE_DMA_REQUEST,      \
        .dma_irq                             = PWMT2_UPDATE_DMA_IRQ,           \
        .dma_handle_index                    = GPT_DMA_ID_UPDATE,              \
        .dma_handle.Init.PeriphDataAlignment = PWMT2_UPDATE_DMA_PDATAALIGN,    \
        .dma_handle.Init.MemDataAlignment    = PWMT2_UPDATE_DMA_MDATAALIGN     \
    }

#define PWMT2_CC1_DMA_CONFIG                                                   \
    {                                                                          \
        .dma_handle.Init.Priority            = PWMT2_CC1_DMA_IRQ_PRIO,         \
        .dma_handle.Instance                 = PWMT2_CC1_DMA_INSTANCE,         \
        .dma_handle.Init.Request             = PWMT2_CC1_DMA_REQUEST,          \
        .dma_irq                             = PWMT2_CC1_DMA_IRQ,              \
        .dma_handle_index                    = GPT_DMA_ID_CC1,                 \
        .dma_handle.Init.PeriphDataAlignment = PWMT2_CC1_DMA_PDATAALIGN,       \
        .dma_handle.Init.MemDataAlignment    = PWMT2_CC1_DMA_MDATAALIGN        \
    }

#define PWMT2_CC2_DMA_CONFIG                                                   \
    {                                                                          \
        .dma_handle.Init.Priority            = PWMT2_CC2_DMA_IRQ_PRIO,         \
        .dma_handle.Instance                 = PWMT2_CC2_DMA_INSTANCE,         \
        .dma_handle.Init.Request             = PWMT2_CC2_DMA_REQUEST,          \
        .dma_irq                             = PWMT2_CC2_DMA_IRQ,              \
        .dma_handle_index                    = GPT_DMA_ID_CC2,                 \
        .dma_handle.Init.PeriphDataAlignment = PWMT2_CC2_DMA_PDATAALIGN,       \
        .dma_handle.Init.MemDataAlignment    = PWMT2_CC2_DMA_MDATAALIGN        \
    }

#define PWMT2_CC3_DMA_CONFIG                                                   \
    {                                                                          \
        .dma_handle.Init.Priority            = PWMT2_CC3_DMA_IRQ_PRIO,         \
        .dma_handle.Instance                 = PWMT2_CC3_DMA_INSTANCE,         \
        .dma_handle.Init.Request             = PWMT2_CC3_DMA_REQUEST,          \
        .dma_irq                             = PWMT2_CC3_DMA_IRQ,              \
        .dma_handle_index                    = GPT_DMA_ID_CC3,                 \
        .dma_handle.Init.PeriphDataAlignment = PWMT2_CC3_DMA_PDATAALIGN,       \
        .dma_handle.Init.MemDataAlignment    = PWMT2_CC3_DMA_MDATAALIGN        \
    }

#define PWMT2_CC4_DMA_CONFIG                                                   \
    {                                                                          \
        .dma_handle.Init.Priority            = PWMT2_CC4_DMA_IRQ_PRIO,         \
        .dma_handle.Instance                 = PWMT2_CC4_DMA_INSTANCE,         \
        .dma_handle.Init.Request             = PWMT2_CC4_DMA_REQUEST,          \
        .dma_irq                             = PWMT2_CC4_DMA_IRQ,              \
        .dma_handle_index                    = GPT_DMA_ID_CC4,                 \
        .dma_handle.Init.PeriphDataAlignment = PWMT2_CC4_DMA_PDATAALIGN,       \
        .dma_handle.Init.MemDataAlignment    = PWMT2_CC4_DMA_MDATAALIGN        \
    }

#ifdef BSP_USING_PWMA1
#define PWMA1_CONFIG                                                           \
    {                                                                          \
        .tim_handle.Instance     = (GPT_TypeDef *)ATIM1,                       \
        .tim_handle.core         = PWMA1_CORE,                                 \
        .name                    = "pwma1",                                    \
        .channel                 = 0                                           \
    }
#endif /* BSP_USING_PWMA1 */

#define PWMA1_UPDATE_DMA_CONFIG                                                \
    {                                                                          \
        .dma_handle.Init.Priority            = PWMA1_UPDATE_DMA_IRQ_PRIO,      \
        .dma_handle.Instance                 = PWMA1_UPDATE_DMA_INSTANCE,      \
        .dma_handle.Init.Request             = PWMA1_UPDATE_DMA_REQUEST,       \
        .dma_irq                             = PWMA1_UPDATE_DMA_IRQ,           \
        .dma_handle_index                    = GPT_DMA_ID_UPDATE,              \
        .dma_handle.Init.PeriphDataAlignment = PWMA1_UPDATE_DMA_PDATAALIGN,    \
        .dma_handle.Init.MemDataAlignment    = PWMA1_UPDATE_DMA_MDATAALIGN     \
    }

#define PWMA1_CC1_DMA_CONFIG                                                   \
    {                                                                          \
        .dma_handle.Init.Priority            = PWMA1_CC1_DMA_IRQ_PRIO,         \
        .dma_handle.Instance                 = PWMA1_CC1_DMA_INSTANCE,         \
        .dma_handle.Init.Request             = PWMA1_CC1_DMA_REQUEST,          \
        .dma_irq                             = PWMA1_CC1_DMA_IRQ,              \
        .dma_handle_index                    = GPT_DMA_ID_CC1,                 \
        .dma_handle.Init.PeriphDataAlignment = PWMA1_CC1_DMA_PDATAALIGN,       \
        .dma_handle.Init.MemDataAlignment    = PWMA1_CC1_DMA_MDATAALIGN        \
    }

#define PWMA1_CC2_DMA_CONFIG                                                   \
    {                                                                          \
        .dma_handle.Init.Priority            = PWMA1_CC2_DMA_IRQ_PRIO,         \
        .dma_handle.Instance                 = PWMA1_CC2_DMA_INSTANCE,         \
        .dma_handle.Init.Request             = PWMA1_CC2_DMA_REQUEST,          \
        .dma_irq                             = PWMA1_CC2_DMA_IRQ,              \
        .dma_handle_index                    = GPT_DMA_ID_CC2,                 \
        .dma_handle.Init.PeriphDataAlignment = PWMA1_CC2_DMA_PDATAALIGN,       \
        .dma_handle.Init.MemDataAlignment    = PWMA1_CC2_DMA_MDATAALIGN        \
    }

#define PWMA1_CC3_DMA_CONFIG                                                   \
    {                                                                          \
        .dma_handle.Init.Priority            = PWMA1_CC3_DMA_IRQ_PRIO,         \
        .dma_handle.Instance                 = PWMA1_CC3_DMA_INSTANCE,         \
        .dma_handle.Init.Request             = PWMA1_CC3_DMA_REQUEST,          \
        .dma_irq                             = PWMA1_CC3_DMA_IRQ,              \
        .dma_handle_index                    = GPT_DMA_ID_CC3,                 \
        .dma_handle.Init.PeriphDataAlignment = PWMA1_CC3_DMA_PDATAALIGN,       \
        .dma_handle.Init.MemDataAlignment    = PWMA1_CC3_DMA_MDATAALIGN        \
    }

#define PWMA1_CC4_DMA_CONFIG                                                   \
    {                                                                          \
        .dma_handle.Init.Priority            = PWMA1_CC4_DMA_IRQ_PRIO,         \
        .dma_handle.Instance                 = PWMA1_CC4_DMA_INSTANCE,         \
        .dma_handle.Init.Request             = PWMA1_CC4_DMA_REQUEST,          \
        .dma_irq                             = PWMA1_CC4_DMA_IRQ,              \
        .dma_handle_index                    = GPT_DMA_ID_CC4,                 \
        .dma_handle.Init.PeriphDataAlignment = PWMA1_CC4_DMA_PDATAALIGN,       \
        .dma_handle.Init.MemDataAlignment    = PWMA1_CC4_DMA_MDATAALIGN        \
    }

#define PWMA2_CONFIG                                                           \
    {                                                                          \
        .tim_handle.Instance     = (GPT_TypeDef *)ATIM2,                       \
        .tim_handle.core         = PWMA2_CORE,                                 \
        .name                    = "pwma2",                                    \
        .channel                 = 0                                           \
    }

#define PWMA2_UPDATE_DMA_CONFIG                                                \
    {                                                                          \
        .dma_handle.Init.Priority = PWMA2_UPDATE_DMA_IRQ_PRIO,                 \
        .dma_handle.Instance = PWMA2_UPDATE_DMA_INSTANCE,                      \
        .dma_handle.Init.Request = PWMA2_UPDATE_DMA_REQUEST,                   \
        .dma_irq = PWMA2_UPDATE_DMA_IRQ,                                       \
        .dma_handle_index = GPT_DMA_ID_UPDATE,                                 \
        .dma_handle.Init.PeriphDataAlignment    = PWMA2_UPDATE_DMA_PDATAALIGN, \
        .dma_handle.Init.MemDataAlignment   = PWMA2_UPDATE_DMA_MDATAALIGN      \
    }

#define PWMA2_CC1_DMA_CONFIG                                                   \
    {                                                                          \
        .dma_handle.Init.Priority = PWMA2_CC1_DMA_IRQ_PRIO,                    \
        .dma_handle.Instance = PWMA2_CC1_DMA_INSTANCE,                         \
        .dma_handle.Init.Request = PWMA2_CC1_DMA_REQUEST,                      \
        .dma_irq = PWMA2_CC1_DMA_IRQ,                                          \
        .dma_handle_index = GPT_DMA_ID_CC1,                                    \
        .dma_handle.Init.PeriphDataAlignment    = PWMA2_CC1_DMA_PDATAALIGN,    \
        .dma_handle.Init.MemDataAlignment   = PWMA2_CC1_DMA_MDATAALIGN         \
    }

#define PWMA2_CC2_DMA_CONFIG                                                   \
    {                                                                          \
        .dma_handle.Init.Priority = PWMA2_CC2_DMA_IRQ_PRIO,                    \
        .dma_handle.Instance = PWMA2_CC2_DMA_INSTANCE,                         \
        .dma_handle.Init.Request = PWMA2_CC2_DMA_REQUEST,                      \
        .dma_irq = PWMA2_CC2_DMA_IRQ,                                          \
        .dma_handle_index = GPT_DMA_ID_CC2,                                    \
        .dma_handle.Init.PeriphDataAlignment    = PWMA2_CC2_DMA_PDATAALIGN,    \
        .dma_handle.Init.MemDataAlignment   = PWMA2_CC2_DMA_MDATAALIGN         \
    }

#define PWMA2_CC3_DMA_CONFIG                                                   \
    {                                                                          \
        .dma_handle.Init.Priority = PWMA2_CC3_DMA_IRQ_PRIO,                    \
        .dma_handle.Instance = PWMA2_CC3_DMA_INSTANCE,                         \
        .dma_handle.Init.Request = PWMA2_CC3_DMA_REQUEST,                      \
        .dma_irq = PWMA2_CC3_DMA_IRQ,                                          \
        .dma_handle_index = GPT_DMA_ID_CC3,                                    \
        .dma_handle.Init.PeriphDataAlignment    = PWMA2_CC3_DMA_PDATAALIGN,    \
        .dma_handle.Init.MemDataAlignment   = PWMA2_CC3_DMA_MDATAALIGN         \
    }

#define PWMA2_CC4_DMA_CONFIG                                                   \
    {                                                                          \
        .dma_handle.Init.Priority = PWMA2_CC4_DMA_IRQ_PRIO,                    \
        .dma_handle.Instance = PWMA2_CC4_DMA_INSTANCE,                         \
        .dma_handle.Init.Request = PWMA2_CC4_DMA_REQUEST,                      \
        .dma_irq = PWMA2_CC4_DMA_IRQ,                                          \
        .dma_handle_index = GPT_DMA_ID_CC4,                                    \
        .dma_handle.Init.PeriphDataAlignment    = PWMA2_CC4_DMA_PDATAALIGN,    \
        .dma_handle.Init.MemDataAlignment   = PWMA2_CC4_DMA_MDATAALIGN         \
    }

#ifdef BSP_USING_PWM_LPTIM1
#define PWM_LPTIM1_CONFIG                                                      \
    {                                                                          \
        .tim_handle.Instance     = hwp_lptim1,                                 \
        .name                    = "pwmlp1",                                   \
    }

#endif /* BSP_USING_PWM_LPTIM1 */

#ifdef BSP_USING_PWM_LPTIM2
#define PWM_LPTIM2_CONFIG                                                      \
    {                                                                          \
        .tim_handle.Instance     = hwp_lptim2,                                 \
        .name                    = "pwmlp2",                                   \
    }
#endif /* BSP_USING_PWM_LPTIM2 */

#ifdef BSP_USING_PWM_LPTIM3
#define PWM_LPTIM3_CONFIG                                                      \
    {                                                                          \
        .tim_handle.Instance     = hwp_lptim3,                                 \
        .name                    = "pwmlp3",                                   \
    }
#endif /* BSP_USING_PWM_LPTIM3 */


#ifdef __cplusplus
}
#endif

#endif /* __PWM_CONFIG_H__ */
