/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __UART_CONFIG_H__
#define __UART_CONFIG_H__

#include <rtconfig.h>

#ifdef __cplusplus
extern "C" {
#endif


#if defined(BSP_USING_UART1)
#ifndef UART1_CONFIG
#define UART1_CONFIG                                                \
    {                                                               \
        .name = "uart1",                                            \
        .Instance = USART1,                                         \
        .irq_type = USART1_IRQn,                                    \
    }
#endif /* UART1_CONFIG */
#endif /* BSP_USING_UART1 */

#if defined(BSP_UART1_RX_USING_DMA)
#ifndef UART1_RX_DMA_CONFIG
#define UART1_RX_DMA_CONFIG                                         \
    {                                                               \
        .Instance = UART1_RX_DMA_INSTANCE,                          \
        .request  = UART1_RX_DMA_REQUEST,                           \
        .dma_irq_prio  = UART1_RX_DMA_IRQ_PRIO,                     \
        .dma_irq  = UART1_RX_DMA_IRQ,                               \
    }
#endif /* UART1_RX_DMA_CONFIG */
#endif /* BSP_UART1_RX_USING_DMA */

#if defined(BSP_UART1_TX_USING_DMA)
#ifndef UART1_TX_DMA_CONFIG
#define UART1_TX_DMA_CONFIG                                         \
    {                                                               \
        .Instance = UART1_TX_DMA_INSTANCE,                          \
        .request  = UART1_TX_DMA_REQUEST,                           \
        .dma_irq_prio  = UART1_TX_DMA_IRQ_PRIO,                     \
        .dma_irq  = UART1_TX_DMA_IRQ,                               \
    }
#endif /* UART1_TX_DMA_CONFIG */
#endif /* BSP_UART1_TX_USING_DMA */


#if defined(BSP_USING_UART2)
#ifndef UART2_CONFIG
#define UART2_CONFIG                                                \
    {                                                               \
        .name = "uart2",                                            \
        .Instance = USART2,                                         \
        .irq_type = USART2_IRQn,                                    \
    }
#endif /* UART2_CONFIG */
#endif /* BSP_USING_UART2 */

#if defined(BSP_UART2_RX_USING_DMA)
#ifndef UART2_RX_DMA_CONFIG
#define UART2_RX_DMA_CONFIG                                         \
    {                                                               \
        .Instance = UART2_RX_DMA_INSTANCE,                          \
        .request  = UART2_RX_DMA_REQUEST,                           \
        .dma_irq_prio  = UART2_RX_DMA_IRQ_PRIO,                     \
        .dma_irq  = UART2_RX_DMA_IRQ,                               \
    }
#endif /* UART2_RX_DMA_CONFIG */
#endif /* BSP_UART2_RX_USING_DMA */

#if defined(BSP_UART2_TX_USING_DMA)
#ifndef UART2_TX_DMA_CONFIG
#define UART2_TX_DMA_CONFIG                                         \
    {                                                               \
        .Instance = UART2_TX_DMA_INSTANCE,                          \
        .request  = UART2_TX_DMA_REQUEST,                           \
        .dma_irq_prio  = UART2_TX_DMA_IRQ_PRIO,                     \
        .dma_irq  = UART2_TX_DMA_IRQ,                               \
    }
#endif /* UART2_TX_DMA_CONFIG */
#endif /* BSP_UART2_TX_USING_DMA */


#if defined(BSP_USING_UART3)
#ifndef UART3_CONFIG
#define UART3_CONFIG                                                \
    {                                                               \
        .name = "uart3",                                            \
        .Instance = USART3,                                         \
        .irq_type = USART3_IRQn,                                    \
    }
#endif /* UART2_CONFIG */
#endif /* BSP_USING_UART2 */

#if defined(BSP_UART3_RX_USING_DMA)
#ifndef UART3_RX_DMA_CONFIG
#define UART3_RX_DMA_CONFIG                                         \
    {                                                               \
        .Instance = UART3_RX_DMA_INSTANCE,                          \
        .request  = UART3_RX_DMA_REQUEST,                           \
        .dma_irq_prio  = UART3_RX_DMA_IRQ_PRIO,                     \
        .dma_irq  = UART3_RX_DMA_IRQ,                               \
    }
#endif /* UART3_RX_DMA_CONFIG */
#endif /* BSP_UART3_RX_USING_DMA */

#if defined(BSP_UART3_TX_USING_DMA)
#ifndef UART3_TX_DMA_CONFIG
#define UART3_TX_DMA_CONFIG                                         \
    {                                                               \
        .Instance = UART3_TX_DMA_INSTANCE,                          \
        .request  = UART3_TX_DMA_REQUEST,                           \
        .dma_irq_prio  = UART3_TX_DMA_IRQ_PRIO,                     \
        .dma_irq  = UART3_TX_DMA_IRQ,                               \
    }
#endif /* UART3_TX_DMA_CONFIG */
#endif /* BSP_UART3_TX_USING_DMA */



#if defined(BSP_USING_UART4)
#ifndef UART4_CONFIG
#define UART4_CONFIG                                                \
    {                                                               \
        .name = "uart4",                                            \
        .Instance = USART4,                                         \
        .irq_type = USART4_IRQn,                                    \
    }
#endif /* UART4_CONFIG */
#endif /* BSP_USING_UART4 */

#if defined(BSP_UART4_RX_USING_DMA)
#ifndef UART4_RX_DMA_CONFIG
#define UART4_RX_DMA_CONFIG                                         \
    {                                                               \
        .Instance = UART4_RX_DMA_INSTANCE,                          \
        .request  = UART4_RX_DMA_REQUEST,                           \
        .dma_irq_prio  = UART4_RX_DMA_IRQ_PRIO,                     \
        .dma_irq  = UART4_RX_DMA_IRQ,                               \
    }
#endif /* UART4_RX_DMA_CONFIG */
#endif /* BSP_UART4_RX_USING_DMA */

#if defined(BSP_UART4_TX_USING_DMA)
#ifndef UART4_TX_DMA_CONFIG
#define UART4_TX_DMA_CONFIG                                         \
    {                                                               \
        .Instance = UART4_TX_DMA_INSTANCE,                          \
        .request  = UART4_TX_DMA_REQUEST,                           \
        .dma_irq_prio  = UART4_TX_DMA_IRQ_PRIO,                     \
        .dma_irq  = UART4_TX_DMA_IRQ,                               \
    }
#endif /* UART4_TX_DMA_CONFIG */
#endif /* BSP_UART4_TX_USING_DMA */

#if defined(BSP_USING_UART5)
#ifndef UART5_CONFIG
#define UART5_CONFIG                                                \
    {                                                               \
        .name = "uart5",                                            \
        .Instance = USART5,                                         \
        .irq_type = USART5_IRQn,                                    \
    }
#endif /* UART5_CONFIG */
#endif /* BSP_USING_UART5 */

#if defined(BSP_UART5_RX_USING_DMA)
#ifndef UART5_RX_DMA_CONFIG
#define UART5_RX_DMA_CONFIG                                         \
    {                                                               \
        .Instance = UART5_RX_DMA_INSTANCE,                          \
        .request  = UART5_RX_DMA_REQUEST,                           \
        .dma_irq_prio  = UART5_RX_DMA_IRQ_PRIO,                     \
        .dma_irq  = UART5_RX_DMA_IRQ,                               \
    }
#endif /* UART5_RX_DMA_CONFIG */
#endif /* BSP_UART5_RX_USING_DMA */

#if defined(BSP_UART5_TX_USING_DMA)
#ifndef UART5_TX_DMA_CONFIG
#define UART5_TX_DMA_CONFIG                                         \
    {                                                               \
        .Instance = UART5_TX_DMA_INSTANCE,                          \
        .request  = UART5_TX_DMA_REQUEST,                           \
        .dma_irq_prio  = UART5_TX_DMA_IRQ_PRIO,                     \
        .dma_irq  = UART5_TX_DMA_IRQ,                               \
    }
#endif /* UART5_TX_DMA_CONFIG */
#endif /* BSP_UART5_TX_USING_DMA */


#ifdef __cplusplus
}
#endif

#endif /* __UART_CONFIG_H__ */
