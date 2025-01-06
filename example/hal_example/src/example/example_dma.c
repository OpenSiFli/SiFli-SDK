/**
  ******************************************************************************
  * @file   example_dma.c
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

#include <string.h>
#include <stdlib.h>
#include "bf0_hal.h"
#include "utest.h"
#include "tc_utils.h"

/*
    This example demo:
        1. Copy 32KB data from flash to PSRAM with DMAC1 channel 1 in asynchronize mode.
        2. Content of data on flash should match with data on PSRAM
*/

static DMA_HandleTypeDef testdma;

static void dma_done_cb()
{
    LOG_I("DMA Done.");
}

static void dma_err_cb()
{
    LOG_I("DMA Error!");
}

#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
void DMAC1_CH1_IRQHandler(void)
{
    /* enter interrupt */
    ENTER_INTERRUPT();

    HAL_DMA_IRQHandler(&testdma);

    /* leave interrupt */
    LEAVE_INTERRUPT();
}
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */

static void testcase(int argc, char **argv)
{

    uint32_t src, dst, len_in_words;
    HAL_StatusTypeDef res;

#ifdef SOC_SF32LB55X
    src = 0x10000000;
    dst = 0x600a0000;
#elif defined(SOC_SF32LB56X)
    src = 0x10000000;
    dst = 0x60200000;
#ifdef BSP_USING_BOARD_EC_LB567XXX
    src = 0x10800000;
    dst = 0x60a00000;
#endif  //BSP_USING_BOARD_EC_LB567XXX
#elif defined(SOC_SF32LB52X)
    src = 0x12000000;
    dst = 0x60000000;
#else
    src = 0x10200000;
    dst = 0x62200000;
#endif

    len_in_words = 8 * 1024; //8K words == 32K bytes

    testdma.Instance = DMA1_Channel1;
    testdma.Init.Request = 0; //DMA_REQUEST_MEM2MEM;
    testdma.Init.Direction = DMA_MEMORY_TO_MEMORY;
    testdma.Init.PeriphInc = DMA_PINC_ENABLE;
    testdma.Init.MemInc = DMA_MINC_ENABLE;
    testdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    testdma.Init.MemDataAlignment   = DMA_MDATAALIGN_WORD;
    testdma.Init.Mode               = DMA_NORMAL;
    testdma.Init.Priority           = DMA_PRIORITY_HIGH;
    testdma.Init.PeriphInc = DMA_PINC_ENABLE; //src
    testdma.Init.MemInc = DMA_MINC_ENABLE;    //dst

    HAL_DMA_Init(&testdma);

    LOG_I("dma  src:%x(4) dst:%x(4) len: %d bytes", src, dst, len_in_words * 4);

#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
    /* NVIC configuration for DMA transfer complete interrupt */
    HAL_NVIC_SetPriority(DMAC1_CH1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMAC1_CH1_IRQn);
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */

    HAL_DMA_RegisterCallback(&testdma, HAL_DMA_XFER_CPLT_CB_ID, dma_done_cb);
    HAL_DMA_RegisterCallback(&testdma, HAL_DMA_XFER_ERROR_CB_ID, dma_err_cb);

    HAL_DMA_Start_IT(&testdma, (uint32_t)src, (uint32_t)dst, len_in_words);

    rt_thread_mdelay(1000);


    if (memcmp((void *)src, (void *)dst, len_in_words * 4) == 0)       // Destination data should match with source input
        LOG_I("DMA src&dst matched.");
    else
        LOG_I("DMA Error.");


}


UTEST_TC_EXPORT(testcase, "example_dma", NULL, NULL, 10);


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
