/**
  ******************************************************************************
  * @file   example_sdadc.c
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

#include <rtthread.h>
#include <rtdevice.h>
#include <stdlib.h>
#include <string.h>
#include "utest.h"
#include "bf0_hal.h"

#ifdef HAL_SDADC_MODULE_ENABLED

/*
 This example demo:
    There are total 5 channel for SDMADC input:
      Channel 0 for analog pin, it only for BGA, do not need pin mux
      Channel 1 to 4 for GPIO input, need set PINMUX when using as SDMADC
    External and internal Voltage reference only work on BGA.

    1. set pinmux and function .
    2. Initial sdmadc handle.
    3. Wait 2 second to make sure reference voltage stable.
    4. Enable slot. for single detect, it is not necessary.
    5. Configure channel.
    6. start SDADC , triger by software.
    7.  Wait for the SDADC to convert .
    8. get ADC register value.
    9. Compare get value with expected value, or convert to voltage to compare.
*/

static rt_err_t utest_tc_init(void)
{
    return RT_EOK;
}

static rt_err_t utest_tc_cleanup(void)
{
    return RT_EOK;
}

static void testcase(int argc, char **argv)
{
    SDADC_HandleTypeDef hadc;
    SDADC_ChannelConfTypeDef ADC_ChanConf;
    uint32_t dst;
    uint32_t lslot = 0;
    HAL_StatusTypeDef ret = HAL_OK;

    // 1. set pinmux and function
    // make sure set SDADC pin to correct mode
    HAL_PIN_Set(PAD_PB26, GPIO_B26, PIN_NOPULL, 0);             // SDADC_CH3
    HAL_PIN_Select(PAD_PB26, 10, 0);
    lslot = HAL_SDADC_GPIO_CHN3;  // set slot to test, SLOT 0 only for BGA

    // 2. Initial sdmadc handle
    memset(&hadc, 0, sizeof(hadc));
    hadc.Instance = hwp_sdadc;
    hadc.Init.adc_se = 1;   // single channel
    hadc.Init.src_sel = HAL_SDADC_SRC_SW;   // software triger
    hadc.Init.vref_sel = HAL_SDADC_VERF_INTERNAL;   // only for BGA
    hadc.Init.dma_en = 0;   // no dma
    hadc.Init.en_slot = 0;  // default slot, update by enable and configure
    hadc.Init.conti_mode = 0;  // once
    hadc.Init.diff_sel = 0;     // single end
    HAL_SDADC_Init(&hadc);

    // 3. Wait 2 second to make sure reference voltage stable
    rt_thread_mdelay(2000);    // wait 2 s to make sure reference stable

    // 4. Enable slot, for single detect, it is not necessary
    HAL_SDADC_EnableSlot(&hadc, lslot, 1);

    // 5. Configure channel
    // Channel to select register, pchnl_sel to choose which pin used, nchnl_sel to 0 for single end
    rt_memset(&ADC_ChanConf, 0, sizeof(ADC_ChanConf));
    ADC_ChanConf.Channel = lslot;
    ADC_ChanConf.shift_num = 2;
    ADC_ChanConf.pchnl_sel = lslot;
    ADC_ChanConf.slot_en = 1;
    HAL_SDADC_ConfigChannel(&hadc, &ADC_ChanConf);

    // 6. start SDADC , triger by software
    HAL_SDADC_Start(&hadc);

    // 7.  Wait for the SDADC to convert
    ret = HAL_SDADC_PollForConversion(&hadc, 100);
    uassert_true_ret(ret == HAL_OK);

    // 8. get ADC register value
    dst = HAL_SDADC_GetValue(&hadc, lslot);
    LOG_I("SDADC reg value %d\n", dst);

    // 9. Compare get value with expected value, or convert to voltage to compare

}


UTEST_TC_EXPORT(testcase, "example_sdadc", utest_tc_init, utest_tc_cleanup, 10);

#endif /*HAL_SDADC_MODULE_ENABLED*/
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
