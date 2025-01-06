/**
  ******************************************************************************
  * @file   spi_config.h
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

#ifndef __SPI_CONFIG_H__
#define __SPI_CONFIG_H__

#include <rtconfig.h>

#ifdef __cplusplus
extern "C" {
#endif


#define SPI1_CORE   CORE_ID_HCPU
#define SPI2_CORE   CORE_ID_HCPU
#define SPI3_CORE   CORE_ID_LCPU
#define SPI4_CORE   CORE_ID_LCPU


#ifdef BSP_USING_SPI1
#ifndef SPI1_BUS_CONFIG
#define SPI1_BUS_CONFIG                                  \
    {                                                    \
        .Instance = SPI1,                                \
        .bus_name = "spi1",                              \
        .irq_type = SPI1_IRQn,                           \
        .core     = SPI1_CORE,                           \
    }

#endif  /* SPI1_BUS_CONFIG */
#endif  /* BSP_USING_SPI1 */

#ifdef BSP_SPI1_TX_USING_DMA
#ifndef SPI1_TX_DMA_CONFIG
#define SPI1_TX_DMA_CONFIG                              \
    {                                                   \
        .dma_irq_prio = SPI1_TX_DMA_IRQ_PRIO,           \
        .Instance = SPI1_TX_DMA_INSTANCE,               \
        .dma_irq = SPI1_TX_DMA_IRQ,                     \
        .request = SPI1_TX_DMA_REQUEST,                 \
    }

#endif  /* SPI1_TX_DMA_CONFIG */
#endif  /* BSP_SPI1_TX_USING_DMA */

#ifdef BSP_SPI1_RX_USING_DMA
#ifndef SPI1_RX_DMA_CONFIG
#define SPI1_RX_DMA_CONFIG                              \
    {                                                   \
        .dma_irq_prio = SPI1_RX_DMA_IRQ_PRIO,           \
        .Instance = SPI1_RX_DMA_INSTANCE,               \
        .dma_irq = SPI1_RX_DMA_IRQ,                     \
        .request = SPI1_RX_DMA_REQUEST,                 \
    }

#endif  /* SPI1_RX_DMA_CONFIG */
#endif  /* BSP_SPI1_RX_USING_DMA */


#ifdef BSP_USING_SPI2
#ifndef SPI2_BUS_CONFIG
#define SPI2_BUS_CONFIG                                  \
    {                                                    \
        .Instance = SPI2,                                \
        .bus_name = "spi2",                              \
        .irq_type = SPI2_IRQn,                           \
        .core     = SPI2_CORE,                           \
    }

#endif  /* SPI2_BUS_CONFIG */
#endif  /* BSP_USING_SPI2 */

#ifdef BSP_SPI2_TX_USING_DMA
#ifndef SPI2_TX_DMA_CONFIG
#define SPI2_TX_DMA_CONFIG                              \
    {                                                   \
        .dma_irq_prio = SPI2_TX_DMA_IRQ_PRIO,           \
        .Instance = SPI2_TX_DMA_INSTANCE,               \
        .dma_irq = SPI2_TX_DMA_IRQ,                     \
        .request = SPI2_TX_DMA_REQUEST,                 \
    }

#endif  /* SPI2_TX_DMA_CONFIG */
#endif  /* BSP_SPI2_TX_USING_DMA */

#ifdef BSP_SPI2_RX_USING_DMA
#ifndef SPI2_RX_DMA_CONFIG
#define SPI2_RX_DMA_CONFIG                              \
    {                                                   \
        .dma_irq_prio = SPI2_RX_DMA_IRQ_PRIO,           \
        .Instance = SPI2_RX_DMA_INSTANCE,               \
        .dma_irq = SPI2_RX_DMA_IRQ,                     \
        .request = SPI2_RX_DMA_REQUEST,                 \
    }

#endif  /* SPI2_RX_DMA_CONFIG */
#endif  /* BSP_SPI2_RX_USING_DMA */

#ifdef BSP_USING_SPI3
#ifndef SPI3_BUS_CONFIG
#define SPI3_BUS_CONFIG                                  \
    {                                                    \
        .Instance = SPI3,                                \
        .bus_name = "spi3",                              \
        .irq_type = SPI3_IRQn,                           \
        .core     = SPI3_CORE,                           \
    }

#endif  /* SPI3_BUS_CONFIG */
#endif  /* BSP_USING_SPI3 */

#ifdef BSP_SPI3_TX_USING_DMA
#ifndef SPI3_TX_DMA_CONFIG
#define SPI3_TX_DMA_CONFIG                              \
    {                                                   \
        .dma_irq_prio = SPI3_TX_DMA_IRQ_PRIO,           \
        .Instance = SPI3_TX_DMA_INSTANCE,               \
        .dma_irq = SPI3_TX_DMA_IRQ,                     \
        .request = SPI3_TX_DMA_REQUEST,                 \
    }

#endif  /* SPI3_TX_DMA_CONFIG */
#endif  /* BSP_SPI3_TX_USING_DMA */

#ifdef BSP_SPI3_RX_USING_DMA
#ifndef SPI3_RX_DMA_CONFIG
#define SPI3_RX_DMA_CONFIG                              \
    {                                                   \
        .dma_irq_prio = SPI3_RX_DMA_IRQ_PRIO,           \
        .Instance = SPI3_RX_DMA_INSTANCE,               \
        .dma_irq = SPI3_RX_DMA_IRQ,                     \
        .request = SPI3_RX_DMA_REQUEST,                 \
    }

#endif  /* SPI3_RX_DMA_CONFIG */
#endif  /* BSP_SPI3_RX_USING_DMA */

#ifdef BSP_USING_SPI4
#ifndef SPI4_BUS_CONFIG
#define SPI4_BUS_CONFIG                             \
    {                                               \
        .Instance = SPI4,                           \
        .bus_name = "spi4",                         \
        .irq_type = SPI4_IRQn,                           \
        .core     = SPI4_CORE,                           \
    }
#endif /* SPI4_BUS_CONFIG */
#endif /* BSP_USING_SPI4 */

#ifdef BSP_SPI4_TX_USING_DMA
#ifndef SPI4_TX_DMA_CONFIG
#define SPI4_TX_DMA_CONFIG                          \
    {                                               \
        .dma_irq_prio = SPI4_TX_DMA_IRQ_PRIO,       \
        .Instance = SPI4_TX_DMA_INSTANCE,           \
        .request = SPI4_TX_DMA_REQUEST,             \
        .dma_irq = SPI4_TX_DMA_IRQ,                 \
    }
#endif /* SPI4_TX_DMA_CONFIG */
#endif /* BSP_SPI4_TX_USING_DMA */

#ifdef BSP_SPI4_RX_USING_DMA
#ifndef SPI4_RX_DMA_CONFIG
#define SPI4_RX_DMA_CONFIG                          \
    {                                               \
        .dma_irq_prio = SPI4_RX_DMA_IRQ_PRIO,       \
        .Instance = SPI4_RX_DMA_INSTANCE,           \
        .request = SPI4_RX_DMA_REQUEST,             \
        .dma_irq = SPI4_RX_DMA_IRQ,                 \
    }
#endif /* SPI4_RX_DMA_CONFIG */
#endif /* BSP_SPI4_RX_USING_DMA */

#ifdef __cplusplus
}
#endif

#endif /*__SPI_CONFIG_H__ */



/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
