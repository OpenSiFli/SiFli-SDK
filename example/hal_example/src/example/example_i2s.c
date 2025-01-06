/**
  ******************************************************************************
  * @file   example_i2s.c
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
#include "bf0_hal.h"
#include "tc_utils.h"

#if defined(HAL_I2S_MODULE_ENABLED)&&!defined(SOC_SF32LB52X)

/*
    This example demo:
        1. Configure I2S audio parameters
        2. Transmit data with polling mode
        3. Receive data with polling mode
*/

#define EXAMPLE_I2S_TRANS_SIZE          (480)

static I2S_HandleTypeDef i2s_handle;
static uint8_t pData[EXAMPLE_I2S_TRANS_SIZE];

static long utest_tc_init(void)
{
    return 0;
}

static long utest_tc_cleanup(void)
{
    return 0;
}
#ifndef SF32LB55X
    #define CLOCK_USING_XTAL 0  //PLL is used, need open codec
#else
    #define CLOCK_USING_XTAL 1  //this value must 1 for 55x
#endif
#if CLOCK_USING_XTAL //crystal
static CLK_DIV_T  txrx_clk_div[9]  = {{48000, 125, 125,  5}, {44100, 136, 136,  4}, {32000, 185, 190,  5}, {24000, 250, 250, 10}, {22050, 272, 272,  8},
    {16000, 384, 384, 12}, {12000, 500, 500, 20}, {11025, 544, 544, 16}, { 8000, 750, 750, 30}
};
#else  //PLL
//PLL 16k 49.152M  44.1k  45.1584M
//lrclk_duty_high:PLL/spclk_div/samplerate/2: 64=49.152M/48k/8/2
//bclk:lrclk_duty_high/32
static CLK_DIV_T  txrx_clk_div[9]  = {{48000, 64, 64,  2}, {44100, 64, 64,  2}, {32000, 96, 96,  3}, {24000, 128, 128, 4}, {22050, 128, 128,  4},
    {16000, 192, 192, 6}, {12000, 256, 256, 8}, {11025, 256, 256, 8}, { 8000, 384, 384, 12}
};
#endif
static void testcase(int argc, char **argv)
{
    /* initial I2S controller */
    I2S_HandleTypeDef *hi2s = &i2s_handle;
    HAL_StatusTypeDef ret;

    for (int i = 0; i < EXAMPLE_I2S_TRANS_SIZE; i++)
    {
        pData[i] = 0x55;
    }

#ifdef  hwp_i2s2
    hi2s->Instance = hwp_i2s2;
    HAL_RCC_EnableModule(RCC_MOD_I2S2);
#else
    hi2s->Instance = hwp_i2s1;
    HAL_RCC_EnableModule(RCC_MOD_I2S1);
#endif

    /* Initial tx configure*/
    hi2s->Init.tx_cfg.data_dw = 16; // bit width 16
    hi2s->Init.tx_cfg.pcm_dw = 16;
    hi2s->Init.tx_cfg.bus_dw = 32;
    hi2s->Init.tx_cfg.slave_mode = 0;   // master mode
    hi2s->Init.tx_cfg.track = 0;        // default stereo
    hi2s->Init.tx_cfg.vol = 4;     // default set to mute(15) or 0 db (4)
    hi2s->Init.tx_cfg.balance_en = 0;
    hi2s->Init.tx_cfg.balance_vol = 0;
    hi2s->Init.tx_cfg.chnl_sel = 0;
    hi2s->Init.tx_cfg.lrck_invert = 0;
    hi2s->Init.tx_cfg.sample_rate = 16000;
    hi2s->Init.tx_cfg.extern_intf = 0;
    hi2s->Init.tx_cfg.clk_div_index = 5;//for 16k samplerate
    hi2s->Init.tx_cfg.clk_div = &txrx_clk_div[hi2s->Init.tx_cfg.clk_div_index];


    /* Initial rx configure*/
    hi2s->Init.rx_cfg.data_dw = 16;
    hi2s->Init.rx_cfg.pcm_dw = 16;
    hi2s->Init.rx_cfg.bus_dw = 32;
    hi2s->Init.rx_cfg.slave_mode = 1;   // slave mode
    hi2s->Init.rx_cfg.chnl_sel = 0;     // left/right all set to left
    hi2s->Init.rx_cfg.sample_rate = 16000;
    hi2s->Init.rx_cfg.chnl_sel = 0;        // default stereo
    hi2s->Init.rx_cfg.lrck_invert = 0;
    hi2s->Init.rx_cfg.clk_div_index = 5;//for 16k samplerate
    hi2s->Init.rx_cfg.clk_div = &txrx_clk_div[hi2s->Init.rx_cfg.clk_div_index];


#if CLOCK_USING_XTAL
#ifndef SF32LB55X
    __HAL_I2S_CLK_XTAL(hi2s);   // xtal use 48M for asic
    __HAL_I2S_SET_SPCLK_DIV(hi2s, 4);   // set to 12M to i2s
#endif
#else
    __HAL_I2S_CLK_PLL(hi2s); //PLL
    __HAL_I2S_SET_SPCLK_DIV(hi2s, 8);   // set to 6.144M to i2s   PLL
    bf0_enable_pll(hi2s->Init.tx_cfg.sample_rate, 0);
#endif

    /*Initial I2S controller */
    HAL_I2S_Init(hi2s);

    /*Start I2S TX test */
    /* reconfigure I2S TX before start if any changed*/
    HAL_I2S_Config_Transmit(hi2s, &(hi2s->Init.tx_cfg));

    /* Start I2S transmit with polling mode */
    ret = HAL_I2S_Transmit(hi2s, pData, EXAMPLE_I2S_TRANS_SIZE, 100);
    uassert_true(ret == HAL_OK);
    /*End I2S TX test */

    /*Start I2S RX test */
    /* reconfigure I2S RX before start if any changed*/
    HAL_I2S_Config_Receive(hi2s, &(hi2s->Init.rx_cfg));

    /* For I2S2, RX clock from TX, so need enable TX when start RX */
    __HAL_I2S_TX_ENABLE(hi2s);
    /* Start I2S Receive with polling mode */
    ret = HAL_I2S_Receive(hi2s, pData, EXAMPLE_I2S_TRANS_SIZE, 100);
    uassert_true(ret == HAL_OK);
    /* End I2S RX test */

}


UTEST_TC_EXPORT(testcase, "example_i2s", utest_tc_init, utest_tc_cleanup, 10);

#endif // HAL_I2S_MODULE_ENABLED

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
