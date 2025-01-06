/**
  ******************************************************************************
  * @file   tim_config.h
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
#define I2C4_CORE   CORE_ID_LCPU
#define I2C5_CORE   CORE_ID_LCPU
#define I2C6_CORE   CORE_ID_LCPU



#if defined(BSP_I2C1_USING_DMA)
#ifndef I2C1_TRX_DMA_CONFIG
#define I2C1_TRX_DMA_CONFIG                          \
    {                                                \
        .dma_irq_prio = I2C1_DMA_IRQ_PRIO,           \
        .Instance = I2C1_DMA_INSTANCE,               \
        .dma_irq = I2C1_DMA_IRQ,                     \
        .request = I2C1_DMA_REQUEST,                 \
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
    }
#endif
#endif

#if defined(BSP_I2C5_USING_DMA)
#ifndef I2C5_TRX_DMA_CONFIG
#define I2C5_TRX_DMA_CONFIG                          \
    {                                                \
        .dma_irq_prio = I2C5_DMA_IRQ_PRIO,           \
        .Instance = I2C5_DMA_INSTANCE,               \
        .dma_irq = I2C5_DMA_IRQ,                     \
        .request = I2C5_DMA_REQUEST,                 \
    }
#endif
#endif

#if defined(BSP_I2C6_USING_DMA)
#ifndef I2C6_TRX_DMA_CONFIG
#define I2C6_TRX_DMA_CONFIG                          \
    {                                                \
        .dma_irq_prio = I2C6_DMA_IRQ_PRIO,           \
        .Instance = I2C6_DMA_INSTANCE,               \
        .dma_irq = I2C6_DMA_IRQ,                     \
        .request = I2C6_DMA_REQUEST,                 \
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

#if defined(BSP_USING_I2C5)
#ifndef BF0_I2C5_CFG
#define BF0_I2C5_CFG                      \
    {                                     \
        .device_name = "i2c5",            \
        .Instance = I2C5,                 \
        .irq_type = I2C5_IRQn,            \
        .core     = I2C5_CORE,            \
    }
#endif
#endif

#if defined(BSP_USING_I2C6)
#ifndef BF0_I2C6_CFG
#define BF0_I2C6_CFG                      \
    {                                     \
        .device_name = "i2c6",            \
        .Instance = I2C6,                 \
        .irq_type = I2C6_IRQn,            \
        .core     = I2C6_CORE,            \
    }
#endif
#endif

#ifdef __cplusplus
}
#endif

#endif /* __TIM_CONFIG_H__ */
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

