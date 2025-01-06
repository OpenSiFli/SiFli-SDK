/**
  ******************************************************************************
  * @file   drv_usart.c
  * @author Sifli software development team
  * @brief USART BSP driver
  * @{
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
#include "rtthread.h"
#include "board.h"
#include "drv_usart.h"
#include "drv_config.h"
#include "string.h"
#include <stdlib.h>

/** @addtogroup bsp_driver Driver IO
  * @{
  */

/** @defgroup drv_usart UART
  * @brief USART BSP driver
  * @{
  */

#ifdef RT_USING_SERIAL

//#define DRV_DEBUG
#define LOG_TAG             "drv.usart"
#include <drv_log.h>

#ifndef RT_SERIAL_DEFAULT_BAUDRATE
    #define RT_SERIAL_DEFAULT_BAUDRATE (1000000)
#endif

#if !defined(BSP_USING_UART1) && !defined(BSP_USING_UART2) && !defined(BSP_USING_UART3) \
    && !defined(BSP_USING_UART4) && !defined(BSP_USING_UART5) && !defined(BSP_USING_LPUART1)
    //#error "Please define at least one BSP_USING_UARTx"
    /* this driver can be disabled at menuconfig → RT-Thread Components → Device Drivers */
#endif

#ifdef RT_SERIAL_USING_DMA
    static rt_size_t sifli_dma_transmit(struct rt_serial_device *serial, rt_uint8_t *buf, rt_size_t size, int direction);
#endif

enum
{
#ifdef BSP_USING_UART1
    UART1_INDEX,
#endif
#ifdef BSP_USING_UART2
    UART2_INDEX,
#endif
#ifdef BSP_USING_UART3
    UART3_INDEX,
#endif
#ifdef BSP_USING_UART4
    UART4_INDEX,
#endif
#ifdef BSP_USING_UART5
    UART5_INDEX,
#endif
#ifdef BSP_USING_LPUART1
    LPUART1_INDEX,
#endif
#ifdef BSP_USING_UART6
    UART6_INDEX,
#endif
    UART_MAX,
};

static struct sifli_uart_config uart_config[] =
{
#ifdef BSP_USING_UART1
    UART1_CONFIG,
#endif
#ifdef BSP_USING_UART2
    UART2_CONFIG,
#endif
#ifdef BSP_USING_UART3
    UART3_CONFIG,
#endif
#ifdef BSP_USING_UART4
    UART4_CONFIG,
#endif
#ifdef BSP_USING_UART5
    UART5_CONFIG,
#endif
#ifdef BSP_USING_LPUART1
    LPUART1_CONFIG,
#endif
#ifdef BSP_USING_UART6
    UART6_CONFIG,
#endif
};

#ifndef PKG_USING_SEGGER_RTT     // If Jlink RTT is not defined, use this to avoid link error
    SECTION("Jlink_RTT")
#endif
struct rt_serial_device *dmac_serial_device;

struct sifli_uart uart_obj[sizeof(uart_config) / sizeof(uart_config[0])];

static rt_err_t sifli_configure(struct rt_serial_device *serial, struct serial_configure *cfg)
{
    struct sifli_uart *uart;
    RT_ASSERT(serial != RT_NULL);
    RT_ASSERT(cfg != RT_NULL);
    uart = (struct sifli_uart *)serial->parent.user_data;
    RT_ASSERT(uart != RT_NULL);

    uart->handle.Instance          = uart->config->Instance;
    uart->handle.Init.BaudRate     = cfg->baud_rate;
    uart->handle.Init.HwFlowCtl    = cfg->hwfc;
    uart->handle.Init.HwFlowCtl    <<= USART_CR3_RTSE_Pos;
    uart->handle.Init.Mode         = UART_MODE_TX_RX;
    uart->handle.Init.OverSampling = UART_OVERSAMPLING_16;

    if (cfg->parity && cfg->data_bits < DATA_BITS_9)
        cfg->data_bits++;                           // parity is part of data

    switch (cfg->data_bits)
    {
    case DATA_BITS_6:
        uart->handle.Init.WordLength = UART_WORDLENGTH_6B;
        break;
    case DATA_BITS_7:
        uart->handle.Init.WordLength = UART_WORDLENGTH_7B;
        break;
    case DATA_BITS_8:
        uart->handle.Init.WordLength = UART_WORDLENGTH_8B;
        break;
    case DATA_BITS_9:
        uart->handle.Init.WordLength = UART_WORDLENGTH_9B;
        break;
    default:
        uart->handle.Init.WordLength = UART_WORDLENGTH_8B;
        break;
    }
    switch (cfg->stop_bits)
    {
    case STOP_BITS_1:
        uart->handle.Init.StopBits   = UART_STOPBITS_1;
        break;
    case STOP_BITS_2:
        uart->handle.Init.StopBits   = UART_STOPBITS_2;
        break;
    case STOP_BITS_3:
        uart->handle.Init.StopBits   = UART_STOPBITS_0_5;
        break;
    case STOP_BITS_4:
        uart->handle.Init.StopBits   = UART_STOPBITS_1_5;
        break;
    default:
        uart->handle.Init.StopBits   = UART_STOPBITS_1;
        break;
    }
    switch (cfg->parity)
    {
    case PARITY_NONE:
        uart->handle.Init.Parity     = UART_PARITY_NONE;
        break;
    case PARITY_ODD:
        uart->handle.Init.Parity     = UART_PARITY_ODD;
        break;
    case PARITY_EVEN:
        uart->handle.Init.Parity     = UART_PARITY_EVEN;
        break;
    default:
        uart->handle.Init.Parity     = UART_PARITY_NONE;
        break;
    }

    if (HAL_UART_Init(&uart->handle) != HAL_OK)
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}

static rt_err_t sifli_control(struct rt_serial_device *serial, int cmd, void *arg)
{
    struct sifli_uart *uart;
#ifdef RT_SERIAL_USING_DMA
    rt_ubase_t ctrl_arg = (rt_ubase_t)arg;
#endif

    RT_ASSERT(serial != RT_NULL);
    uart = (struct sifli_uart *)serial->parent.user_data;
    RT_ASSERT(uart != RT_NULL);

    switch (cmd)
    {
    /* disable interrupt */
    case RT_DEVICE_CTRL_CLR_INT:
        /* disable rx irq */
        NVIC_DisableIRQ(uart->config->irq_type);
        /* disable interrupt */
        __HAL_UART_DISABLE_IT(&(uart->handle), UART_IT_RXNE);
        if (arg == (void *) RT_DEVICE_FLAG_DMA_RX)
        {
            uart->dma_rx.last_index = 0;
            //HAL_UART_DMAStop(&uart->handle);
            HAL_UART_AbortReceive_IT(&uart->handle);
#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
            NVIC_DisableIRQ(uart->config->dma_rx->dma_irq);
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
            __HAL_UART_DISABLE_IT(&(uart->handle), UART_IT_IDLE);
            __HAL_UART_CLEAR_IT(&(uart->handle), UART_IT_IDLE);

        }
        else if (arg == (void *) RT_DEVICE_FLAG_DMA_TX)
        {
            //HAL_UART_DMAStop(&uart->handle);
            HAL_UART_AbortTransmit_IT(&uart->handle);
#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
            NVIC_DisableIRQ(uart->config->dma_tx->dma_irq);
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
        }
        break;
    /* enable interrupt */
    case RT_DEVICE_CTRL_SET_INT:
        if (RT_DEVICE_FLAG_INT_RX == (uint32_t)arg)
        {
            /* enable rx irq */
            NVIC_EnableIRQ(uart->config->irq_type);
            /* enable interrupt */
            __HAL_UART_ENABLE_IT(&(uart->handle), UART_IT_RXNE);
        }
        break;

#ifdef RT_SERIAL_USING_DMA
    case RT_DEVICE_CTRL_CONFIG:
        if (ctrl_arg == RT_DEVICE_FLAG_DMA_RX)
        {
            // Clear FIFO
            uart->handle.Instance->RQR |= USART_RQR_RXFRQ;
            RT_ASSERT(serial != RT_NULL);

            struct rt_serial_rx_fifo *rx_fifo = (struct rt_serial_rx_fifo *)serial->serial_rx;
            sifli_dma_transmit(serial, rx_fifo->buffer, serial->config.bufsz, RT_SERIAL_DMA_RX);
            uart->dma_rx.last_index = 0;
        }
        break;
#endif
    }
    sifli_configure(serial, &(serial->config));
    return RT_EOK;
}

static int sifli_putc(struct rt_serial_device *serial, char c)
{
    struct sifli_uart *uart;
    RT_ASSERT(serial != RT_NULL);
    rt_base_t level;
    HAL_UART_LOCK_DEF();

    uart = (struct sifli_uart *)serial->parent.user_data;

    HAL_UART_LOCK(&uart->handle);

    level = rt_hw_interrupt_disable();
    UART_INSTANCE_CLEAR_FUNCTION(&(uart->handle), UART_FLAG_TC);
    __HAL_UART_PUTC(&uart->handle, c);
    while (__HAL_UART_GET_FLAG(&(uart->handle), UART_FLAG_TC) == RESET);
    rt_hw_interrupt_enable(level);

    HAL_UART_UNLOCK(&uart->handle);

    return 1;
}

static int sifli_getc(struct rt_serial_device *serial)
{
    int ch;
    struct sifli_uart *uart;
    RT_ASSERT(serial != RT_NULL);
    uart = (struct sifli_uart *)serial->parent.user_data;
    RT_ASSERT(uart != RT_NULL);

    ch = -1;
    if (__HAL_UART_GET_FLAG(&(uart->handle), UART_FLAG_RXNE) != RESET)
    {
        ch = __HAL_UART_GETC(&uart->handle);
    }
    return ch;
}

#ifdef RT_SERIAL_USING_DMA

static rt_size_t sifli_dma_transmit(struct rt_serial_device *serial, rt_uint8_t *buf, rt_size_t size, int direction)
{
    struct sifli_uart *uart;
    IRQn_Type irq;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct sifli_uart *)serial->parent.user_data;
    RT_ASSERT(uart != RT_NULL);
    if (direction == RT_SERIAL_DMA_RX)
    {
        __HAL_LINKDMA(&(uart->handle), hdmarx, uart->dma_rx.handle);
        uart->handle.hdmarx->Instance = uart->config->dma_rx->Instance;
        uart->handle.hdmarx->Init.Request = uart->config->dma_rx->request;
        irq = uart->config->dma_rx->dma_irq;
    }
    else
    {
        __HAL_LINKDMA(&(uart->handle), hdmatx, uart->dma_tx.handle);
        uart->handle.hdmatx->Instance = uart->config->dma_tx->Instance;
        uart->handle.hdmatx->Init.Request = uart->config->dma_tx->request;
        irq = uart->config->dma_tx->dma_irq;
    }
#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
    HAL_NVIC_SetPriority(irq, 0, 0);
    HAL_NVIC_EnableIRQ(irq);
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
    if (direction == RT_SERIAL_DMA_RX)  // For RX DMA, also need to enable UART IRQ.
    {
        __HAL_UART_ENABLE_IT(&(uart->handle), UART_IT_IDLE);
        HAL_NVIC_SetPriority(uart->config->irq_type, 1, 0);
        HAL_NVIC_EnableIRQ(uart->config->irq_type);
    }
    HAL_UART_DmaTransmit(&(uart->handle), buf, size, (direction == RT_SERIAL_DMA_RX) ? DMA_PERIPH_TO_MEMORY : DMA_MEMORY_TO_PERIPH);

    return 0;
}
#endif

static const struct rt_uart_ops sifli_uart_ops =
{
    .configure = sifli_configure,
    .control = sifli_control,
    .putc = sifli_putc,
    .getc = sifli_getc,
#ifdef RT_SERIAL_USING_DMA
    .dma_transmit = sifli_dma_transmit,
#endif
};

/**
 * Uart common interrupt process. This need add to uart ISR.
 *
 * @param serial serial device
 */
__ROM_USED void uart_isr(struct rt_serial_device *serial)
{
    struct sifli_uart *uart;
#ifdef RT_SERIAL_USING_DMA
    rt_size_t recv_total_index, recv_len;
    rt_base_t level;
#endif

    RT_ASSERT(serial != RT_NULL);

    uart = (struct sifli_uart *) serial->parent.user_data;
    RT_ASSERT(uart != RT_NULL);

    /* UART in mode Receiver -------------------------------------------------*/
    if ((__HAL_UART_GET_FLAG(&(uart->handle), UART_FLAG_RXNE) != RESET) &&
            (__HAL_UART_GET_IT_SOURCE(&(uart->handle), UART_IT_RXNE) != RESET))
    {
        rt_hw_serial_isr(serial, RT_SERIAL_EVENT_RX_IND);
    }
#ifdef RT_SERIAL_USING_DMA
    else if ((uart->uart_rx_dma_flag) && (__HAL_UART_GET_FLAG(&(uart->handle), UART_FLAG_IDLE) != RESET) &&
             (__HAL_UART_GET_IT_SOURCE(&(uart->handle), UART_IT_IDLE) != RESET))
    {
        __HAL_UART_CLEAR_IDLEFLAG(&uart->handle);
        level = rt_hw_interrupt_disable();
        recv_total_index = serial->config.bufsz - __HAL_DMA_GET_COUNTER(&(uart->dma_rx.handle));
        if (recv_total_index < uart->dma_rx.last_index)
            recv_len = serial->config.bufsz + recv_total_index - uart->dma_rx.last_index;
        else
            recv_len = recv_total_index - uart->dma_rx.last_index;
        uart->dma_rx.last_index = recv_total_index;
        rt_hw_interrupt_enable(level);

        if (recv_len)
            rt_hw_serial_isr(serial, RT_SERIAL_EVENT_RX_DMADONE | (recv_len << 8));
    }
    else if ((uart->uart_tx_dma_flag)
             && (__HAL_UART_GET_FLAG(&(uart->handle), UART_FLAG_TC) != RESET)
             && (__HAL_UART_GET_IT_SOURCE(&(uart->handle), UART_IT_TC) != RESET))
    {
        __HAL_UART_CLEAR_FLAG(&uart->handle, UART_CLEAR_TCF);
        uart->handle.gState = HAL_UART_STATE_READY;
        rt_hw_serial_isr(serial, RT_SERIAL_EVENT_TX_DMADONE);
        __HAL_UART_DISABLE_IT(&(uart->handle), UART_IT_TC);
    }
#endif
    else
    {
        if (__HAL_UART_GET_FLAG(&(uart->handle), UART_FLAG_ORE) != RESET)
        {
            __HAL_UART_CLEAR_OREFLAG(&uart->handle);
        }
        if (__HAL_UART_GET_FLAG(&(uart->handle), UART_FLAG_NE) != RESET)
        {
            __HAL_UART_CLEAR_NEFLAG(&uart->handle);
        }
        if (__HAL_UART_GET_FLAG(&(uart->handle), UART_FLAG_FE) != RESET)
        {
            __HAL_UART_CLEAR_FEFLAG(&uart->handle);
        }
        if (__HAL_UART_GET_FLAG(&(uart->handle), UART_FLAG_PE) != RESET)
        {
            __HAL_UART_CLEAR_PEFLAG(&uart->handle);
        }
        if (__HAL_UART_GET_FLAG(&(uart->handle), UART_FLAG_CTS) != RESET)
        {
            UART_INSTANCE_CLEAR_FUNCTION(&(uart->handle), UART_FLAG_CTS);
        }
        if (__HAL_UART_GET_FLAG(&(uart->handle), UART_FLAG_TXE) != RESET)
        {
            UART_INSTANCE_CLEAR_FUNCTION(&(uart->handle), UART_FLAG_TXE);
        }
        if (__HAL_UART_GET_FLAG(&(uart->handle), UART_FLAG_TC) != RESET)
        {
            UART_INSTANCE_CLEAR_FUNCTION(&(uart->handle), UART_FLAG_TC);
        }
        if (__HAL_UART_GET_FLAG(&(uart->handle), UART_FLAG_RXNE) != RESET)
        {
            UART_INSTANCE_CLEAR_FUNCTION(&(uart->handle), UART_FLAG_RXNE);
        }
    }
}

#ifdef RT_SERIAL_USING_DMA
/**
  * @brief  Rx Half Transfer completed callback.
  * @param huart UART handle.
  * @retval None
  */
__ROM_USED void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    struct rt_serial_device *serial;
    struct sifli_uart *uart;
    rt_size_t recv_len;
    rt_size_t recv_total_index;
    rt_base_t level;

    RT_ASSERT(huart != NULL);
    uart = (struct sifli_uart *)huart;
    serial = &uart->serial;

    level = rt_hw_interrupt_disable();
    recv_total_index = serial->config.bufsz - __HAL_DMA_GET_COUNTER(&uart->dma_rx.handle);
    if (recv_total_index < uart->dma_rx.last_index)
        recv_len = serial->config.bufsz + recv_total_index - uart->dma_rx.last_index;
    else
        recv_len = recv_total_index - uart->dma_rx.last_index;
    uart->dma_rx.last_index = recv_total_index;
    rt_hw_interrupt_enable(level);

    if (recv_len)
    {
        rt_hw_serial_isr(serial, RT_SERIAL_EVENT_RX_DMADONE | (recv_len << 8));
    }
}


__ROM_USED void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    HAL_UART_RxHalfCpltCallback(huart);
}

#endif

#if defined(BSP_USING_UART1)
void USART1_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_isr(&(uart_obj[UART1_INDEX].serial));

    /* leave interrupt */
    rt_interrupt_leave();
}

#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
#if defined(RT_SERIAL_USING_DMA) && defined(BSP_UART1_RX_USING_DMA)
void UART1_DMA_RX_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_DMA_IRQHandler(&uart_obj[UART1_INDEX].dma_rx.handle);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* defined(RT_SERIAL_USING_DMA) && defined(BSP_UART1_RX_USING_DMA) */
#if defined(RT_SERIAL_USING_DMA) && defined(BSP_UART1_TX_USING_DMA)
void UART1_DMA_TX_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_DMA_IRQHandler(&uart_obj[UART1_INDEX].dma_tx.handle);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* defined(RT_SERIAL_USING_DMA) && defined(BSP_UART1_TX_USING_DMA) */
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
#endif /* BSP_USING_UART1 */

#if defined(BSP_USING_UART2)
void USART2_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_isr(&(uart_obj[UART2_INDEX].serial));

    /* leave interrupt */
    rt_interrupt_leave();
}
#if defined(RT_SERIAL_USING_DMA) && defined(BSP_UART2_RX_USING_DMA) && !defined(DMA_SUPPORT_DYN_CHANNEL_ALLOC)
void UART2_DMA_RX_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_DMA_IRQHandler(&uart_obj[UART2_INDEX].dma_rx.handle);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* defined(RT_SERIAL_USING_DMA) && defined(BSP_UART2_RX_USING_DMA) */
#if defined(RT_SERIAL_USING_DMA) && defined(BSP_UART2_TX_USING_DMA) && !defined(DMA_SUPPORT_DYN_CHANNEL_ALLOC)
void UART2_DMA_TX_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_DMA_IRQHandler(&uart_obj[UART2_INDEX].dma_tx.handle);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* defined(RT_SERIAL_USING_DMA) && defined(BSP_UART2_TX_USING_DMA) */
#endif /* BSP_USING_UART2 */

#if defined(BSP_USING_UART3)
void USART3_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_isr(&(uart_obj[UART3_INDEX].serial));

    /* leave interrupt */
    rt_interrupt_leave();
}
#if defined(RT_SERIAL_USING_DMA) && defined(BSP_UART3_RX_USING_DMA) && !defined(DMA_SUPPORT_DYN_CHANNEL_ALLOC)
void UART3_DMA_RX_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_DMA_IRQHandler(&uart_obj[UART3_INDEX].dma_rx.handle);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* defined(BSP_UART_USING_DMA_RX) && defined(BSP_UART3_RX_USING_DMA) */
#if defined(RT_SERIAL_USING_DMA) && defined(BSP_UART3_TX_USING_DMA) && !defined(DMA_SUPPORT_DYN_CHANNEL_ALLOC)
void UART3_DMA_TX_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_DMA_IRQHandler(&uart_obj[UART3_INDEX].dma_tx.handle);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* defined(RT_SERIAL_USING_DMA) && defined(BSP_UART3_TX_USING_DMA) */
#endif /* BSP_USING_UART3*/

#if defined(BSP_USING_UART4)
void USART4_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_isr(&(uart_obj[UART4_INDEX].serial));

    /* leave interrupt */
    rt_interrupt_leave();
}
#if defined(RT_SERIAL_USING_DMA) && defined(BSP_UART4_RX_USING_DMA) && !defined(DMA_SUPPORT_DYN_CHANNEL_ALLOC)
void UART4_DMA_RX_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_DMA_IRQHandler(&uart_obj[UART4_INDEX].dma_rx.handle);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* defined(BSP_UART_USING_DMA_RX) && defined(BSP_UART4_RX_USING_DMA) */
#if defined(RT_SERIAL_USING_DMA) && defined(BSP_UART4_TX_USING_DMA) && !defined(DMA_SUPPORT_DYN_CHANNEL_ALLOC)
void UART4_DMA_TX_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_DMA_IRQHandler(&uart_obj[UART4_INDEX].dma_tx.handle);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* defined(BSP_UART_USING_DMA_RX) && defined(BSP_UART4_TX_USING_DMA) */
#endif /* BSP_USING_UART4*/

#if defined(BSP_USING_UART5)
void USART5_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_isr(&(uart_obj[UART5_INDEX].serial));

    /* leave interrupt */
    rt_interrupt_leave();
}
#if defined(RT_SERIAL_USING_DMA) && defined(BSP_UART5_RX_USING_DMA) && !defined(DMA_SUPPORT_DYN_CHANNEL_ALLOC)
void UART5_DMA_RX_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_DMA_IRQHandler(&uart_obj[UART5_INDEX].dma_rx.handle);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* defined(RT_SERIAL_USING_DMA) && defined(BSP_UART5_RX_USING_DMA) */

#if defined(RT_SERIAL_USING_DMA) && defined(BSP_UART5_TX_USING_DMA) && !defined(DMA_SUPPORT_DYN_CHANNEL_ALLOC)
void UART5_DMA_TX_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_DMA_IRQHandler(&uart_obj[UART5_INDEX].dma_tx.handle);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* defined(BSP_UART_USING_DMA_RX) && defined(BSP_UART5_TX_USING_DMA) */

#endif /* BSP_USING_UART5*/

#if defined(BSP_USING_LPUART1)
void LPUART1_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_isr(&(uart_obj[LPUART1_INDEX].serial));

    /* leave interrupt */
    rt_interrupt_leave();
}
#if defined(RT_SERIAL_USING_DMA) && defined(BSP_LPUART1_RX_USING_DMA)
void LPUART1_DMA_RX_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_DMA_IRQHandler(&uart_obj[LPUART1_INDEX].dma_rx.handle);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* defined(RT_SERIAL_USING_DMA) && defined(BSP_LPUART1_RX_USING_DMA) */
#endif /* BSP_USING_LPUART1*/

#if defined(BSP_USING_UART6)
void USART6_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_isr(&(uart_obj[UART6_INDEX].serial));

    /* leave interrupt */
    rt_interrupt_leave();
}
#if defined(RT_SERIAL_USING_DMA) && defined(BSP_UART6_RX_USING_DMA) && !defined(DMA_SUPPORT_DYN_CHANNEL_ALLOC)
void UART6_DMA_RX_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_DMA_IRQHandler(&uart_obj[UART6_INDEX].dma_rx.handle);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* defined(BSP_UART_USING_DMA_RX) && defined(BSP_UART6_RX_USING_DMA) */

#if defined(RT_SERIAL_USING_DMA) && defined(BSP_UART6_TX_USING_DMA) && !defined(DMA_SUPPORT_DYN_CHANNEL_ALLOC)
void UART6_DMA_TX_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_DMA_IRQHandler(&uart_obj[UART6_INDEX].dma_tx.handle);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* defined(BSP_UART_USING_DMA_RX) && defined(BSP_UART6_TX_USING_DMA) */

#endif /* BSP_USING_UART6*/



#ifdef RT_SERIAL_USING_DMA

/**
  * @brief  UART error callbacks
  * @param  huart: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
__ROM_USED void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    RT_ASSERT(huart != NULL);
    struct sifli_uart *uart = (struct sifli_uart *)huart;
    LOG_D("%s: %s %d\n", __FUNCTION__, uart->config->name, huart->ErrorCode);
    UNUSED(uart);
}

#endif  /* RT_SERIAL_USING_DMA */

static void sifli_uart_get_dma_config(void)
{
#ifdef BSP_UART1_RX_USING_DMA
    uart_obj[UART1_INDEX].uart_rx_dma_flag = 1;
    static struct dma_config uart1_dma_rx = UART1_RX_DMA_CONFIG;
    uart_config[UART1_INDEX].dma_rx = &uart1_dma_rx;
#endif
#ifdef BSP_UART1_TX_USING_DMA
    uart_obj[UART1_INDEX].uart_tx_dma_flag = 1;
    static struct dma_config uart1_dma_tx = UART1_TX_DMA_CONFIG;
    uart_config[UART1_INDEX].dma_tx = &uart1_dma_tx;
#endif

#ifdef BSP_UART2_RX_USING_DMA
    uart_obj[UART2_INDEX].uart_rx_dma_flag = 1;
    static struct dma_config uart2_dma_rx = UART2_RX_DMA_CONFIG;
    uart_config[UART2_INDEX].dma_rx = &uart2_dma_rx;
#endif
#ifdef BSP_UART2_TX_USING_DMA
    uart_obj[UART2_INDEX].uart_tx_dma_flag = 1;
    static struct dma_config uart2_dma_tx = UART2_TX_DMA_CONFIG;
    uart_config[UART2_INDEX].dma_tx = &uart2_dma_tx;
#endif

#ifdef BSP_UART3_RX_USING_DMA
    uart_obj[UART3_INDEX].uart_rx_dma_flag = 1;
    static struct dma_config uart3_dma_rx = UART3_RX_DMA_CONFIG;
    uart_config[UART3_INDEX].dma_rx = &uart3_dma_rx;
#endif
#ifdef BSP_UART3_TX_USING_DMA
    uart_obj[UART3_INDEX].uart_tx_dma_flag = 1;
    static struct dma_config uart3_dma_tx = UART3_TX_DMA_CONFIG;
    uart_config[UART3_INDEX].dma_tx = &uart3_dma_tx;
#endif

#ifdef BSP_UART4_RX_USING_DMA
    uart_obj[UART4_INDEX].uart_rx_dma_flag = 1;
    static struct dma_config uart4_dma_rx = UART4_RX_DMA_CONFIG;
    uart_config[UART4_INDEX].dma_rx = &uart4_dma_rx;
#endif
#ifdef BSP_UART4_TX_USING_DMA
    uart_obj[UART4_INDEX].uart_tx_dma_flag = 1;
    static struct dma_config uart4_dma_tx = UART4_TX_DMA_CONFIG;
    uart_config[UART4_INDEX].dma_tx = &uart4_dma_tx;
#endif

#ifdef BSP_UART5_RX_USING_DMA
    uart_obj[UART5_INDEX].uart_rx_dma_flag = 1;
    static struct dma_config uart5_dma_rx = UART5_RX_DMA_CONFIG;
    uart_config[UART5_INDEX].dma_rx = &uart5_dma_rx;
#endif
#ifdef BSP_UART5_TX_USING_DMA
    uart_obj[UART5_INDEX].uart_tx_dma_flag = 1;
    static struct dma_config uart5_dma_tx = UART5_TX_DMA_CONFIG;
    uart_config[UART5_INDEX].dma_tx = &uart5_dma_tx;
#endif
#ifdef BSP_UART6_RX_USING_DMA
    uart_obj[UART6_INDEX].uart_rx_dma_flag = 1;
    static struct dma_config uart6_dma_rx = UART6_RX_DMA_CONFIG;
    uart_config[UART6_INDEX].dma_rx = &uart6_dma_rx;
#endif
#ifdef BSP_UART6_TX_USING_DMA
    uart_obj[UART6_INDEX].uart_tx_dma_flag = 1;
    static struct dma_config uart6_dma_tx = UART6_TX_DMA_CONFIG;
    uart_config[UART6_INDEX].dma_tx = &uart6_dma_tx;
#endif

}

__ROM_USED int rt_hw_usart_init2(struct sifli_uart *objs, struct sifli_uart_config *cfg, rt_size_t obj_num, rt_uint32_t baud_rate)
{
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    rt_err_t result = 0;
    rt_uint32_t flag;

    config.baud_rate = baud_rate;

    for (int i = 0; i < obj_num; i++)
    {
        objs[i].config = &cfg[i];
        objs[i].serial.ops = &sifli_uart_ops;
        objs[i].serial.config = config;

        flag = RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_INT_TX;

#if defined(RT_SERIAL_USING_DMA)
        if (objs[i].uart_rx_dma_flag)
        {
            flag |= RT_DEVICE_FLAG_DMA_RX;
        }
        if (objs[i].uart_tx_dma_flag)
        {
            flag |= RT_DEVICE_FLAG_DMA_TX;
        }
#endif
        {
            /* register UART device */
            result = rt_hw_serial_register(&objs[i].serial, objs[i].config->name,
                                           flag, &objs[i]);
        }
        RT_ASSERT(result == RT_EOK);
    }

    return result;
}

int rt_hw_usart_init()
{
    rt_err_t result = RT_EOK;
    rt_size_t obj_num = sizeof(uart_obj) / sizeof(struct sifli_uart);
    if (obj_num > 0)
    {
        sifli_uart_get_dma_config();
        result = rt_hw_usart_init2(uart_obj, uart_config, obj_num, RT_SERIAL_DEFAULT_BAUDRATE);
    }
    return result;
}

#endif /* RT_USING_SERIAL */

#if defined(RT_USING_FINSH)&&!defined(LCPU_MEM_OPTIMIZE)
static void uart(uint8_t argc, char **argv)
{
    if (argc > 3)
    {
        if (strcmp(argv[1], "br") == 0)
        {
            uint32_t idx = atoi(argv[2]);
            uint32_t baudrate = atoi(argv[3]);
            struct serial_configure   *config = &(uart_obj[idx].serial.config);

            config->baud_rate = baudrate;
            sifli_configure(&(uart_obj[idx].serial), config);
        }
    }
    else
    {
        LOG_E("please input: uart br [baudrate] \n");
        LOG_E("e.g: uart br 3000000\n");
    }
}
MSH_CMD_EXPORT(uart, uart setting);
#endif

/// @} drv_usart
/// @} bsp_driver
/// @} file


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
