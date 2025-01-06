/**
  ******************************************************************************
  * @file   example_audio.c
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
#include "utest.h"
#include "board.h"
#include "tc_utils.h"

#ifdef HAL_PDM_MODULE_ENABLED

#include "dma_config.h"



//static I2S_HandleTypeDef I2S_Handle = {0};
static PDM_HandleTypeDef PDM_Handle = {0};
static DMA_HandleTypeDef DMA_Handle = {0};
static volatile uint8_t overFlag = 0;
static IRQn_Type pdm_dma_irq;


void HAL_PDM_RxCpltCallback(PDM_HandleTypeDef *hpdm)
{
    overFlag = 1;
}

#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
void PDM1_L_DMA_IRQHandler(void)
{
    /* enter interrupt */
    ENTER_INTERRUPT();

    //LOG_I("PDM1_L_DMA_IRQHandler");

    HAL_DMA_IRQHandler(PDM_Handle.hdmarx);

    /* leave interrupt */
    LEAVE_INTERRUPT();
}
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */

static rt_err_t utest_tc_init(void)
{
    return RT_EOK;
}

static rt_err_t utest_tc_cleanup(void)
{
    return RT_EOK;
}



static void bf0_audio_init(PDM_HandleTypeDef *hpdm)
{
    hpdm->Instance = hwp_pdm1;
    hpdm->hdmarx = &DMA_Handle;

    pdm_dma_irq = PDM1_L_DMA_IRQ;
    hpdm->hdmarx->Instance = PDM1_L_DMA_INSTANCE;
    hpdm->hdmarx->Init.Request = PDM1_L_DMA_REQUEST;

    hpdm->Init.Mode = PDM_MODE_LOOP;
    hpdm->Init.Channels = PDM_CHANNEL_LEFT_ONLY;
    hpdm->Init.SampleRate = PDM_SAMPLE_16KHZ;
    hpdm->Init.ChannelDepth = PDM_CHANNEL_DEPTH_16BIT;
    PDM_Handle.RxXferSize = 1024;
}


typedef enum
{
    ARGI_BUF_LENTH_IN_KB,
    ARGI_MAX,
    ARGI_OUTPUT_UART = ARGI_MAX, /*Optional*/
} ARG_IDX;


static void testcase(int argc, char **argv)
{
    rt_device_t p_uart_dev = NULL;
    uint32_t expect_len = 200;
    uint32_t m;


    p_uart_dev = rt_device_find(argv[ARGI_OUTPUT_UART]);
    if (p_uart_dev)
    {
        rt_device_open(p_uart_dev, RT_DEVICE_FLAG_RDWR);
    }
    else
    {
        uassert_true_ret(0);
        goto FINAL_STEP;
    }

    expect_len = strtol(argv[ARGI_BUF_LENTH_IN_KB], 0, 10);

    PDM_Handle.RxXferSize = 1024;
    PDM_Handle.pRxBuffPtr = (uint8_t *) rt_malloc(PDM_Handle.RxXferSize);
    if (PDM_Handle.pRxBuffPtr == NULL)
    {
        uassert_false(RT_TRUE);
        goto FINAL_STEP;
    }
    memset(PDM_Handle.pRxBuffPtr, 0, PDM_Handle.RxXferSize);

    bf0_audio_init(&PDM_Handle);
    PDM_Handle.Init.SampleRate = 8000;
    PDM_Handle.Init.ChannelDepth = 16;
    PDM_Handle.Init.Channels = PDM_CHANNEL_STEREO;
    HAL_PDM_Init(&PDM_Handle);
    HAL_NVIC_EnableIRQ(pdm_dma_irq);

    for (m = 0; m < expect_len; m++)
    {
        overFlag = 0;
        HAL_PDM_Receive_DMA(&PDM_Handle, PDM_Handle.pRxBuffPtr, PDM_Handle.RxXferSize);
        LOG_I("Read data to buffer:%x, len: %d", (uint32_t)PDM_Handle.pRxBuffPtr, PDM_Handle.RxXferSize);

        while (overFlag == 0)
        {

        }

        if (p_uart_dev)
        {
            //LOG_I("Out Audio size=%d\n", len);
            rt_device_write(p_uart_dev, 0, &PDM_Handle.pRxBuffPtr[0], PDM_Handle.RxXferSize);
        }
    }

    HAL_NVIC_DisableIRQ(pdm_dma_irq);
    HAL_PDM_DMAStop(&PDM_Handle);
    HAL_PDM_DeInit(&PDM_Handle);

FINAL_STEP:
    if (PDM_Handle.pRxBuffPtr)
    {
        rt_free(PDM_Handle.pRxBuffPtr);
        PDM_Handle.pRxBuffPtr = NULL;
    }

    if (p_uart_dev)
    {
        rt_device_close(p_uart_dev);
    }

    return;

}

UTEST_TC_EXPORT(testcase, "example_audio", utest_tc_init, utest_tc_cleanup, 10);

#endif

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

