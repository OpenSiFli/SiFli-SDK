/**
  ******************************************************************************
  * @file   flash_config.h
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

#ifndef __FLASH_CONFIG_H__
#define __FLASH_CONFIG_H__

#include <rtconfig.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef BSP_USING_MPI


#ifdef BSP_ENABLE_QSPI1
#if ((BSP_QSPI1_MODE == SPI_MODE_NOR)||(BSP_QSPI1_MODE == SPI_MODE_NAND))

#ifndef FLASH1_CONFIG
#define FLASH1_CONFIG                                  \
    {                                                  \
        .Instance = FLASH1,                            \
        .line = 2,                                     \
        .base = FLASH1_BASE_ADDR,                       \
        .msize = BSP_QSPI1_MEM_SIZE,                   \
        .SpiMode = BSP_QSPI1_MODE,                     \
    }

#endif  /* FLASH_CONFIG1 */

#ifdef BSP_QSPI1_USING_DMA
#ifndef FLASH1_DMA_CONFIG
#define FLASH1_DMA_CONFIG                              \
    {                                                  \
        .dma_irq_prio = FLASH1_DMA_IRQ_PRIO,           \
        .Instance = FLASH1_DMA_INSTANCE,               \
        .dma_irq = FLASH1_DMA_IRQ,                     \
        .request = FLASH1_DMA_REQUEST,                 \
    }

#endif  /* FLASH1_DMA_CONFIG */
#endif  /* BSP_QSPI1_USING_DMA */

#endif /*#if ((BSP_QSPI1_MODE == SPI_MODE_NOR)||(BSP_QSPI1_MODE == SPI_MODE_NAND))*/
#endif  /* BSP_ENABLE_QSPI1 */

#ifdef BSP_ENABLE_QSPI2
#if ((BSP_QSPI2_MODE == SPI_MODE_NOR)||(BSP_QSPI2_MODE == SPI_MODE_NAND))

#ifndef FLASH2_CONFIG
#define FLASH2_CONFIG                                  \
    {                                                  \
        .Instance = FLASH2,                            \
        .line = 2,                                     \
        .base = FLASH2_BASE_ADDR,                      \
        .msize = BSP_QSPI2_MEM_SIZE,                   \
        .SpiMode = BSP_QSPI2_MODE,                     \
    }

#endif  /* FLASH_CONFIG2 */

#ifdef BSP_QSPI2_USING_DMA
#ifndef FLASH2_DMA_CONFIG
#define FLASH2_DMA_CONFIG                              \
    {                                                  \
        .dma_irq_prio = FLASH2_DMA_IRQ_PRIO,           \
        .Instance = FLASH2_DMA_INSTANCE,               \
        .dma_irq = FLASH2_DMA_IRQ,                     \
        .request = FLASH2_DMA_REQUEST,                 \
    }

#endif  /* FLASH2_DMA_CONFIG */
#endif  /* BSP_QSPI2_USING_DMA */

#endif /*#if ((BSP_QSPI2_MODE == SPI_MODE_NOR)||(BSP_QSPI2_MODE == SPI_MODE_NAND))*/
#endif  /* BSP_ENABLE_QSPI2 */

#ifdef BSP_ENABLE_QSPI3
#if ((BSP_QSPI3_MODE == SPI_MODE_NOR)||(BSP_QSPI3_MODE == SPI_MODE_NAND))

#ifndef FLASH3_CONFIG
#define FLASH3_CONFIG                                  \
    {                                                  \
        .Instance = FLASH3,                            \
        .line = 2,                                     \
        .base = FLASH3_BASE_ADDR,                      \
        .msize = BSP_QSPI3_MEM_SIZE,                   \
        .SpiMode = BSP_QSPI3_MODE,                     \
    }

#endif  /* FLASH_CONFIG3 */

#ifdef BSP_QSPI3_USING_DMA
#ifndef FLASH3_DMA_CONFIG
#define FLASH3_DMA_CONFIG                              \
    {                                                  \
        .dma_irq_prio = FLASH3_DMA_IRQ_PRIO,           \
        .Instance = FLASH3_DMA_INSTANCE,               \
        .dma_irq = FLASH3_DMA_IRQ,                     \
        .request = FLASH3_DMA_REQUEST,                 \
    }

#endif  /* FLASH3_DMA_CONFIG */
#endif  /* BSP_QSPI3_USING_DMA */

#endif /*#if ((BSP_QSPI3_MODE == SPI_MODE_NOR)||(BSP_QSPI3_MODE == SPI_MODE_NAND))*/

#endif  /* BSP_ENABLE_QSPI3 */

#ifdef BSP_ENABLE_QSPI4
#if ((BSP_QSPI4_MODE == SPI_MODE_NOR)||(BSP_QSPI4_MODE == SPI_MODE_NAND))

#ifndef FLASH4_CONFIG
#define FLASH4_CONFIG                                  \
    {                                                  \
        .Instance = FLASH4,                            \
        .line = 2,                                     \
        .base = FLASH4_BASE_ADDR,                      \
        .msize = BSP_QSPI4_MEM_SIZE,                   \
        .SpiMode = BSP_QSPI4_MODE,                     \
    }

#endif  /* FLASH_CONFIG4 */

#ifdef BSP_QSPI4_USING_DMA
#ifndef FLASH4_DMA_CONFIG
#define FLASH4_DMA_CONFIG                              \
    {                                                  \
        .dma_irq_prio = FLASH4_DMA_IRQ_PRIO,           \
        .Instance = FLASH4_DMA_INSTANCE,               \
        .dma_irq = FLASH4_DMA_IRQ,                     \
        .request = FLASH4_DMA_REQUEST,                 \
    }

#endif  /* FLASH4_DMA_CONFIG */
#endif  /* BSP_QSPI4_USING_DMA */

#endif /*#if ((BSP_QSPI4_MODE == SPI_MODE_NOR)||(BSP_QSPI3_MODE == SPI_MODE_NAND))*/

#endif  /* BSP_ENABLE_QSPI4 */

#ifdef BSP_ENABLE_QSPI5
#ifndef FLASH5_CONFIG
#define FLASH5_CONFIG                                  \
    {                                                  \
        .Instance = FLASH5,                            \
        .line = 2,                                     \
        .base = FLASH5_BASE_ADDR,                      \
        .msize = BSP_QSPI5_MEM_SIZE,                   \
        .SpiMode = BSP_QSPI5_MODE,                     \
    }
#endif  /* FLASH_CONFIG5 */
#endif //#ifdef BSP_ENABLE_QSPI5

#ifndef FLASH5_DMA_CONFIG
#define FLASH5_DMA_CONFIG                              \
    {                                                  \
        .dma_irq_prio = FLASH5_DMA_IRQ_PRIO,           \
        .Instance = FLASH5_DMA_INSTANCE,               \
        .dma_irq = FLASH5_DMA_IRQ,                     \
        .request = FLASH5_DMA_REQUEST,                 \
    }

#endif  /* FLASH4_DMA_CONFIG */


#endif  /* BSP_USING_QSPI */

#ifdef __cplusplus
}
#endif

#endif /*__FLASH_CONFIG_H__ */



/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
