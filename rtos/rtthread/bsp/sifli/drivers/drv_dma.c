/**
  ******************************************************************************
  * @file   drv_dma.c
  * @author Sifli software development team
  * @brief Debug functions for BSP driver
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

#include <rtthread.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

#include <drv_log.h>
#include <drv_common.h>

#ifdef DMA_SUPPORT_DYN_CHANNEL_ALLOC
void DMAC1_CH1_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_DMAC1_CH1_IRQHandler();

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMAC1_CH2_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_DMAC1_CH2_IRQHandler();

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMAC1_CH3_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_DMAC1_CH3_IRQHandler();

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMAC1_CH4_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_DMAC1_CH4_IRQHandler();

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMAC1_CH5_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_DMAC1_CH5_IRQHandler();

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMAC1_CH6_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_DMAC1_CH6_IRQHandler();

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMAC1_CH7_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_DMAC1_CH7_IRQHandler();

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMAC1_CH8_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_DMAC1_CH8_IRQHandler();

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMAC2_CH1_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_DMAC2_CH1_IRQHandler();

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMAC2_CH2_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_DMAC2_CH2_IRQHandler();

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMAC2_CH3_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_DMAC2_CH3_IRQHandler();

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMAC2_CH4_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_DMAC2_CH4_IRQHandler();

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMAC2_CH5_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_DMAC2_CH5_IRQHandler();

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMAC2_CH6_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_DMAC2_CH6_IRQHandler();

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMAC2_CH7_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_DMAC2_CH7_IRQHandler();

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMAC2_CH8_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_DMAC2_CH8_IRQHandler();

    /* leave interrupt */
    rt_interrupt_leave();
}

#if defined(DMA3)
/* SF32LB58X LCPU still use ROM implementation */
void DMAC3_CH1_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_DMAC3_CH1_IRQHandler();

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMAC3_CH2_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_DMAC3_CH2_IRQHandler();

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMAC3_CH3_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_DMAC3_CH3_IRQHandler();

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMAC3_CH4_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_DMAC3_CH4_IRQHandler();

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMAC3_CH5_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_DMAC3_CH5_IRQHandler();

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMAC3_CH6_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_DMAC3_CH6_IRQHandler();

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMAC3_CH7_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_DMAC3_CH7_IRQHandler();

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMAC3_CH8_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_DMAC3_CH8_IRQHandler();

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* DMA3 */
#endif /* DMA_SUPPORT_DYN_CHANNEL_ALLOC */

/// @} file

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
