/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __I2C_CONFIG_H__
#define __I2C_CONFIG_H__

#include <rtconfig.h>
#include "bf0_hal_rcc.h"

#ifdef __cplusplus
extern "C" {
#endif


#define I2C1_CORE   CORE_ID_HCPU
#define I2C2_CORE   CORE_ID_HCPU
#define I2C3_CORE   CORE_ID_HCPU
#define I2C4_CORE   CORE_ID_HCPU

#if defined(BSP_I2C1_USING_DMA)
#ifndef I2C1_TRX_DMA_CONFIG
#define I2C1_TRX_DMA_CONFIG                          \
    {                                                \
        .dma_irq_prio = I2C1_DMA_IRQ_PRIO,           \
        .Instance = I2C1_DMA_INSTANCE,               \
        .dma_irq = I2C1_DMA_IRQ,                     \
        .request = I2C1_DMA_REQUEST,                 \
        .end_trigger = I2C1_DMA_END_TRIG,            \
    }
#endif
#endif

#if defined(BSP_I2C2_USING_DMA)
#ifndef I2C2_TRX_DMA_CONFIG
#define I2C2_TRX_DMA_CONFIG                          \
    {                                                \
        .dma_irq_prio = I2C2_DMA_IRQ_PRIO,           \
        .Instance = I2C2_DMA_INSTANCE,               \
        .dma_irq = I2C2_DMA_IRQ,                     \
        .request = I2C2_DMA_REQUEST,                 \
        .end_trigger = I2C2_DMA_END_TRIG,            \
    }
#endif
#endif

#if defined(BSP_I2C3_USING_DMA)
#ifndef I2C3_TRX_DMA_CONFIG
#define I2C3_TRX_DMA_CONFIG                          \
    {                                                \
        .dma_irq_prio = I2C3_DMA_IRQ_PRIO,           \
        .Instance = I2C3_DMA_INSTANCE,               \
        .dma_irq = I2C3_DMA_IRQ,                     \
        .request = I2C3_DMA_REQUEST,                 \
        .end_trigger = I2C3_DMA_END_TRIG,            \
    }
#endif
#endif

#if defined(BSP_I2C4_USING_DMA)
#ifndef I2C4_TRX_DMA_CONFIG
#define I2C4_TRX_DMA_CONFIG                          \
    {                                                \
        .dma_irq_prio = I2C4_DMA_IRQ_PRIO,           \
        .Instance = I2C4_DMA_INSTANCE,               \
        .dma_irq = I2C4_DMA_IRQ,                     \
        .request = I2C4_DMA_REQUEST,                 \
        .end_trigger = I2C4_DMA_END_TRIG,            \
    }
#endif
#endif

#if defined(BSP_USING_I2C1)
#ifndef BF0_I2C1_CFG
#define BF0_I2C1_CFG                      \
    {                                     \
        .device_name = "i2c1",            \
        .Instance = I2C1,                 \
        .irq_type = I2C1_IRQn,            \
        .core     = I2C1_CORE,            \
    }
#endif
#endif

#if defined(BSP_USING_I2C2)
#ifndef BF0_I2C2_CFG
#define BF0_I2C2_CFG                      \
    {                                     \
        .device_name = "i2c2",            \
        .Instance = I2C2,                 \
        .irq_type = I2C2_IRQn,            \
        .core     = I2C2_CORE,            \
    }
#endif
#endif

#if defined(BSP_USING_I2C3)
#ifndef BF0_I2C3_CFG
#define BF0_I2C3_CFG                      \
    {                                     \
        .device_name = "i2c3",            \
        .Instance = I2C3,                 \
        .irq_type = I2C3_IRQn,            \
        .core     = I2C3_CORE,            \
    }
#endif
#endif

#if defined(BSP_USING_I2C4)
#ifndef BF0_I2C4_CFG
#define BF0_I2C4_CFG                      \
    {                                     \
        .device_name = "i2c4",            \
        .Instance = I2C4,                 \
        .irq_type = I2C4_IRQn,            \
        .core     = I2C4_CORE,            \
    }
#endif
#endif

#ifdef __cplusplus
}
#endif

#endif /* __TIM_CONFIG_H__ */
