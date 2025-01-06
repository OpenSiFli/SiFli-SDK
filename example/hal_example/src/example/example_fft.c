/**
  ******************************************************************************
  * @file   example_fft.c
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

#include <stdlib.h>
#include <string.h>
#include "utest.h"
#include "bf0_hal.h"
#include "tc_utils.h"

#ifdef HAL_FFT_MODULE_ENABLED
#include "fft_512_hw_input.dat"
#include "fft_512_hw_output.dat"

/* Example Description:
 *
 * Use hwp_fft1 to calculate 512 point complex 16bit fixpoint FFT in polling and interrupt mode
 *
 */

static FFT_HandleTypeDef fft_handle;
volatile static uint8_t fft_done_flag;

void FFT1_IRQHandler(void)
{
    ENTER_INTERRUPT();
    HAL_FFT_IRQHandler(&fft_handle);
    LEAVE_INTERRUPT();
}

static rt_err_t utest_tc_init(void)
{
    return RT_EOK;
}

static rt_err_t utest_tc_cleanup(void)
{
    return RT_EOK;
}

static void fft_done(FFT_HandleTypeDef *fft)
{
    fft_done_flag = 1;
}

static void init_fft(void)
{
    // Initialize driver and enable FFT IRQ
    HAL_NVIC_SetPriority(FFT1_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(FFT1_IRQn);

    HAL_RCC_EnableModule(RCC_MOD_FFT1);

    fft_handle.Instance = hwp_fft1 ;
    HAL_FFT_Init(&fft_handle);
}

static void fft_polling_mode(void)
{
    FFT_ConfigTypeDef config;
    HAL_StatusTypeDef res;
    uint32_t data_size = 512 * 4;

    /* 初始化 */
    memset(&config, 0, sizeof(config));

    /* 1024点16比特的复数FFT */
    config.bitwidth = FFT_BW_16BIT;
    config.fft_length = FFT_LEN_512;
    config.ifft_flag = 0;
    config.rfft_flag = 0;
    config.input_data = rt_malloc(data_size);
    config.output_data = rt_malloc(data_size);
    RT_ASSERT(config.input_data);
    RT_ASSERT(config.output_data);
    memcpy(config.input_data, (void *)fft_512_hw_input, data_size);

    res = HAL_FFT_StartFFT(&fft_handle, &config);
    uassert_true_ret(res == HAL_OK);
    uassert_true_ret(0 == memcmp(config.output_data, (void *)fft_512_hw_output, data_size));

    rt_free(config.input_data);
    rt_free(config.output_data);
}

static void fft_interrupt_mode(void)
{
    FFT_ConfigTypeDef config;
    HAL_StatusTypeDef res;
    uint32_t data_size = 512 * 4;

    /* 初始化 */
    memset(&config, 0, sizeof(config));

    /* 1024点16比特的复数FFT */
    config.bitwidth = FFT_BW_16BIT;
    config.fft_length = FFT_LEN_512;
    config.ifft_flag = 0;
    config.rfft_flag = 0;
    config.input_data = rt_malloc(data_size);
    config.output_data = rt_malloc(data_size);
    RT_ASSERT(config.input_data);
    RT_ASSERT(config.output_data);
    memcpy(config.input_data, (void *)fft_512_hw_input, data_size);


    fft_done_flag = 0;
    fft_handle.CpltCallback = fft_done;

    res = HAL_FFT_StartFFT_IT(&fft_handle, &config);
    uassert_true_ret(res == HAL_OK);

    while (0 == fft_done_flag)
    {
    }

    uassert_true_ret(0 == memcmp(config.output_data, (void *)fft_512_hw_output, data_size));

    rt_free(config.input_data);
    rt_free(config.output_data);
}

static void testcase(int argc, char **argv)
{
    init_fft();
    UTEST_UNIT_RUN(fft_polling_mode);
    UTEST_UNIT_RUN(fft_interrupt_mode);

    return;
}

UTEST_TC_EXPORT(testcase, "example_fft", utest_tc_init, utest_tc_cleanup, 10);

#endif /* HAL_FFT_MODULE_ENABLED */
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
