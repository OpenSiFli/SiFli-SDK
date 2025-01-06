/**
  ******************************************************************************
  * @file   example_adc.c
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

#ifdef HAL_ADC_MODULE_ENABLED

#define ADC_RATIO_ACCURATE          (1000)

#ifdef SF32LB55X
    #define ADC_MAX_VOLTAGE_MV     (1100)
#else
    #define ADC_MAX_VOLTAGE_MV     (3300)
#endif

#ifdef SF32LB55X
    // default value, they should be over write by calibrate
    // it should be register value offset vs 0 v value.
    static float adc_vol_offset = 199.0;
    // mv per bit, if accuracy not enough, change to 0.1 mv or 0.01 mv later
    static float adc_vol_ratio = 3930.0; // 4296; //6 * ADC_RATIO_ACCURATE; //600; //6;
#elif defined(SF32LB56X)
    // it should be register value offset vs 0 v value.
    static float adc_vol_offset = 822.0;
    // 0.001 mv per bit
    static float adc_vol_ratio = 1068.0; //
#else
    // it should be register value offset vs 0 v value.
    static float adc_vol_offset = 822.0;
    // 0.001 mv per bit,
    static float adc_vol_ratio = 1068.0; //
#endif

static float adc_vbat_factor = 2.01;

// register data for max supported voltage, for A0, voltage = 1.1v, for RPO, voltage = 3.3v
static uint32_t adc_thd_reg;

/*
    This example demo:
        1. Configure ADC parameters
        2. Polling ADC value
*/

static rt_err_t utest_tc_init(void)
{
    return RT_EOK;
}

static rt_err_t utest_tc_cleanup(void)
{
    return RT_EOK;
}

static int example_adc_calibration(uint32_t value1, uint32_t value2,
                                   uint32_t vol1, uint32_t vol2, float *offset, float *ratio)
{
    float gap1, gap2;
    uint32_t reg_max;

    if (offset == NULL || ratio == NULL)
        return 0;

    if (value1 == 0 || value2 == 0)
        return 0;

    gap1 = value1 > value2 ? value1 - value2 : value2 - value1; // register value gap
    gap2 = vol1 > vol2 ? vol1 - vol2 : vol2 - vol1; // voltage gap -- mv

    if (gap1 != 0)
    {
        *ratio = gap2 * ADC_RATIO_ACCURATE / gap1; // gap2 * 10 for 0.1mv, gap2 * 100 for 0.01mv
        adc_vol_ratio = *ratio;
    }
    else //
        return 0;

    *offset = value1 - vol1 * ADC_RATIO_ACCURATE / adc_vol_ratio;
    adc_vol_offset = *offset;

    // get register value for max voltage
    adc_thd_reg = ADC_MAX_VOLTAGE_MV * ADC_RATIO_ACCURATE / adc_vol_ratio + adc_vol_offset;
    reg_max = GPADC_ADC_RDATA0_SLOT0_RDATA >> GPADC_ADC_RDATA0_SLOT0_RDATA_Pos;
    if (adc_thd_reg >= (reg_max - 3))
        adc_thd_reg = reg_max - 3;

    return adc_vol_offset;
}

static void example_adc_vbat_fact_calib(uint32_t voltage, uint32_t reg)
{
    float vol_from_reg;

    // get voltage calculate by register data
    vol_from_reg = (reg - adc_vol_offset) * adc_vol_ratio / ADC_RATIO_ACCURATE;
    adc_vbat_factor = (float)voltage / vol_from_reg;
}

static int utest_adc_calib(void)
{
#ifndef SF32LB52X   //
    HAL_LCPU_CONFIG_ADC_T cfg;
    uint16_t len = (uint16_t)sizeof(HAL_LCPU_CONFIG_ADC_T);
    if (HAL_LCPU_CONFIG_get(HAL_LCPU_CONFIG_ADC_CALIBRATION, (uint8_t *)&cfg, &len) == 0)
#else
    FACTORY_CFG_ADC_T cfg;
    int len = sizeof(FACTORY_CFG_ADC_T);
    if (BSP_CONFIG_get(FACTORY_CFG_ID_ADC, (uint8_t *)&cfg, len))
#endif
    {
        float off, rat;
        uint32_t vol1, vol2;
        if (cfg.vol10 == 0 || cfg.vol25 == 0) // not valid paramter
        {
            // no valid parameter
        }
        else
        {
#ifndef SF32LB55X
            cfg.vol10 &= 0x7fff;
            cfg.vol25 &= 0x7fff;
            vol1 = cfg.low_mv;
            vol2 = cfg.high_mv;
#else
            if ((cfg.vol10 & (1 << 15)) && (cfg.vol25 & (1 << 15))) // small range, use X1 mode
            {
                cfg.vol10 &= 0x7fff;
                cfg.vol25 &= 0x7fff;
                vol1 = 300;
                vol2 = 800;
            }
#endif
            example_adc_calibration(cfg.vol10, cfg.vol25, vol1, vol2, &off, &rat);
#ifdef SF32LB52X
            example_adc_vbat_fact_calib(cfg.vbat_mv, cfg.vbat_reg);
#endif
            //LOG_I("GPADC : %d mv reg %d, %d mv reg %d, offset %d, ratio %d, max reg %d\n",
            //    vol1, cfg.vol10, vol2, cfg.vol25, off, rat, adc_thd_reg);
        }
    }
    else
        return 1;

    return 0;
}

static float example_adc_get_float_mv(float value)
{
    float offset, ratio;
    // get offset
    offset = adc_vol_offset;
    ratio = adc_vol_ratio;
    if (value < offset)
        return 0;

    return (value - offset) * ratio / ADC_RATIO_ACCURATE;
}

static void testcase(int argc, char **argv)
{
    ADC_HandleTypeDef hadc;
    ADC_ChannelConfTypeDef ADC_ChanConf;
    uint32_t dst;
    uint32_t lslot = 0;
    HAL_StatusTypeDef ret = HAL_OK;

    // make sure set CORRECT ADC pin to correct mode
    //HAL_PIN_Set_Analog(PAD_PB27, 0);
#ifdef SF32LB55X
    lslot = 1;  // set slot to test
#ifdef HAL_USING_HTOL   // set to pulldown first as hardware reuqest for htol
    //HAL_PIN_Set(PAD_PB10, GPIO_B10, PIN_PULLDOWN, 0);
    //HAL_PIN_Set_Analog(PAD_PB10, 0);
    hwp_pinmux2->PAD_PB10 = 10 | LPSYS_PINMUX_PAD_PB10_PE_Msk;
#endif  // HAL_USING_HTOL
#elif defined(SF32LB52X)
    lslot = 4;
#else
    lslot = 5;
#endif

    int calib = utest_adc_calib();
    LOG_I("ADC Get calibration res %d\n", calib);

    // initial adc handle
    hadc.Instance = hwp_gpadc1;
#ifndef SF32LB55X
    hadc.Init.data_samp_delay = 2;
    hadc.Init.avdd_v18_en = 0;
#ifdef SF32LB52X
    hadc.Init.conv_width = 75;
    hadc.Init.sample_width = 71;
#else
    hadc.Init.conv_width = 24;
    hadc.Init.sample_width = 22;
#endif
#else
    hadc.Init.clk_div = 31;
#endif
    hadc.Init.adc_se = 1;   // single end
    hadc.Init.adc_force_on = 0;
    hadc.Init.atten3 = 0;
    hadc.Init.dma_en = 0;   // no dma
    hadc.Init.en_slot = 0;  // default slot 0
    hadc.Init.op_mode = 0;  // single mode, not continous

#ifdef BSP_GPADC_USING_DMA

    // example for 52x 6 channel only
    HAL_PIN_Set(PAD_PA28, GPIO_A28, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA29, GPIO_A29, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA30, GPIO_A30, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA31, GPIO_A31, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA32, GPIO_A32, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA33, GPIO_A33, PIN_NOPULL, 1);
    HAL_PIN_Set_Analog(PAD_PA28, 1);
    HAL_PIN_Set_Analog(PAD_PA29, 1);
    HAL_PIN_Set_Analog(PAD_PA30, 1);
    HAL_PIN_Set_Analog(PAD_PA31, 1);
    HAL_PIN_Set_Analog(PAD_PA32, 1);
    HAL_PIN_Set_Analog(PAD_PA33, 1);

#include "dma_config.h"
    hadc.DMA_Handle = (DMA_HandleTypeDef *)malloc(sizeof(DMA_HandleTypeDef));
    if (hadc.DMA_Handle != NULL)
    {
        memset((void *)hadc.DMA_Handle, 0, sizeof(DMA_HandleTypeDef));
        hadc.DMA_Handle->Instance                 = GPADC_DMA_INSTANCE;
        hadc.DMA_Handle->Init.Request             = GPADC_DMA_REQUEST;
        hadc.DMA_Handle->Init.Direction = DMA_PERIPH_TO_MEMORY;
        hadc.DMA_Handle->Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
        hadc.DMA_Handle->Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
        hadc.DMA_Handle->Init.PeriphInc           = DMA_PINC_DISABLE;
        hadc.DMA_Handle->Init.MemInc              = DMA_MINC_ENABLE;
        hadc.DMA_Handle->Init.Mode                = DMA_NORMAL;
        hadc.DMA_Handle->Init.Priority            = DMA_PRIORITY_MEDIUM;
    }
    hadc.Init.en_slot = 1;  // use multi channels, need multi slot
#endif

    HAL_ADC_Init(&hadc);
#ifdef HAL_USING_HTOL
#ifdef SF32LB55X
    hadc.Instance->ADC_CFG_REG1 |= ((1 << GPADC_ADC_CFG_REG1_ANAU_GPADC_ATTN3X_Pos) | (1 << GPADC_ADC_CFG_REG1_ANAU_GPADC_MUTE_Pos));
    hadc.Instance->ADC_CFG_REG1 = (hadc.Instance->ADC_CFG_REG1 & GPADC_ADC_CFG_REG1_ANAU_GPADC_SEL_PCH) | (1 << GPADC_ADC_CFG_REG1_ANAU_GPADC_SEL_PCH_Pos);
    hadc.Instance->ADC_CTRL_REG |= (GPADC_ADC_CTRL_REG_CHNL_SEL_FRC_EN);
#endif
#endif
    // delay 300ms before start adc start, only once
    HAL_Delay(300);
    // enable slot
    //HAL_ADC_EnableSlot(&hadc, lslot, 1);

#ifndef BSP_GPADC_USING_DMA
    // Channel to select register, pchnl_sel to choose which pin used, here use the same number
    rt_memset(&ADC_ChanConf, 0, sizeof(ADC_ChanConf));
    ADC_ChanConf.Channel = lslot;
    ADC_ChanConf.pchnl_sel = lslot;
    ADC_ChanConf.slot_en = 1;
    ADC_ChanConf.acc_num = 0;
    HAL_ADC_ConfigChannel(&hadc, &ADC_ChanConf);

    /* start ADC */
    HAL_ADC_Start(&hadc);

    /* Wait for the ADC to convert */
    ret = HAL_ADC_PollForConversion(&hadc, 100);
    uassert_true_ret(ret == HAL_OK);

    /* get ADC register value */
    dst = HAL_ADC_GetValue(&hadc, lslot);
    LOG_I("ADC reg value %d\n", dst);
    if (calib == 0)
    {
        LOG_I("voltage %f mv\n", example_adc_get_float_mv((float)dst));
    }

    HAL_ADC_Stop(&hadc);
#else
    uint32_t data[8];
    int i, j;

    // configure channel, can support multi channels
    rt_memset(&ADC_ChanConf, 0, sizeof(ADC_ChanConf));
    ADC_ChanConf.Channel = 0;
    ADC_ChanConf.pchnl_sel = 0;
    ADC_ChanConf.slot_en = 1;
    ADC_ChanConf.acc_num = 0;
    HAL_ADC_ConfigChannel(&hadc, &ADC_ChanConf);

    ADC_ChanConf.Channel = 1;
    ADC_ChanConf.pchnl_sel = 1;
    ADC_ChanConf.slot_en = 1;
    ADC_ChanConf.acc_num = 0;
    HAL_ADC_ConfigChannel(&hadc, &ADC_ChanConf);

    ADC_ChanConf.Channel = 2;
    ADC_ChanConf.pchnl_sel = 2;
    ADC_ChanConf.slot_en = 1;
    ADC_ChanConf.acc_num = 0;
    HAL_ADC_ConfigChannel(&hadc, &ADC_ChanConf);

    ADC_ChanConf.Channel = 3;
    ADC_ChanConf.pchnl_sel = 3;
    ADC_ChanConf.slot_en = 1;
    ADC_ChanConf.acc_num = 0;
    HAL_ADC_ConfigChannel(&hadc, &ADC_ChanConf);

    ADC_ChanConf.Channel = 4;
    ADC_ChanConf.pchnl_sel = 4;
    ADC_ChanConf.slot_en = 1;
    ADC_ChanConf.acc_num = 0;
    HAL_ADC_ConfigChannel(&hadc, &ADC_ChanConf);

    ADC_ChanConf.Channel = 5;
    ADC_ChanConf.pchnl_sel = 5;
    ADC_ChanConf.slot_en = 1;
    ADC_ChanConf.acc_num = 0;
    HAL_ADC_ConfigChannel(&hadc, &ADC_ChanConf);

    /* configure power setting */
    HAL_ADC_DMA_PREPARE(&hadc);

    for (j = 0; j < 0xf; j++)
    {
        /* start ADC */
        HAL_ADC_Start_DMA(&hadc, data, 6);

        /* Wait for the ADC to convert */
        ret = HAL_ADC_DMA_WAIT_DONE(&hadc, 100);
        uassert_true_ret(ret == HAL_OK);

        for (i = 0; i < 6; i++)
        {
            LOG_I("ADC reg value %d\n", data[i] & 0xfff);
            if (calib == 0)
            {
                LOG_I("voltage %f mv\n", example_adc_get_float_mv((float)(data[i] & 0xfff)));
            }
        }

        LOG_I("Loop %d done ===\n", j);
        HAL_Delay(1000);
    }

    HAL_ADC_Stop_DMA(&hadc);

#endif
    // TODO, if need get adc more times, need delay 5/10 ms before next start

    // never call Deinit function !!!
}


UTEST_TC_EXPORT(testcase, "example_adc", utest_tc_init, utest_tc_cleanup, 10);

#endif /*HAL_ADC_MODULE_ENABLED*/
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
