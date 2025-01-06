/**
  ******************************************************************************
  * @file   drv_usart.h
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

#ifndef __DRV_USART_H__
#define __DRV_USART_H__

#include <rtthread.h>
#include "rtdevice.h"
#include <rthw.h>
#include <drv_common.h>

int rt_hw_usart_init(void);

#define DMA_INSTANCE_TYPE              DMA_Channel_TypeDef

#define UART_INSTANCE_CLEAR_FUNCTION    __HAL_UART_CLEAR_FLAG

/* sifli uart config class */
struct sifli_uart_config
{
    const char         *name;
    USART_TypeDef      *Instance;
    IRQn_Type           irq_type;
    struct dma_config *dma_rx;
    struct dma_config *dma_tx;
};

/* sifli uart dirver class */
struct sifli_uart
{
    UART_HandleTypeDef handle;
    struct sifli_uart_config *config;

#ifdef RT_SERIAL_USING_DMA
    struct
    {
        DMA_HandleTypeDef handle;
        rt_size_t last_index;
    } dma_rx;
    struct
    {
        DMA_HandleTypeDef handle;
        rt_size_t last_index;
    } dma_tx;
#endif
    rt_uint8_t uart_rx_dma_flag;
    rt_uint8_t uart_tx_dma_flag;
    struct rt_serial_device serial;
};

#endif  /* __DRV_USART_H__ */
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
