/**
  ******************************************************************************
  * @file   example_pwm.c
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

#define MAX_PERIOD_GPT (0xFFFF)
#define MAX_PERIOD_ATM (0xFFFFFFFF)
#define MIN_PERIOD 3
#define MIN_PULSE 2

typedef struct
{
    void *instance;
    uint8_t core;
    uint16_t pad_func;
    uint32_t channel; /* GPT_CHANNEL_1, GPT_CHANNEL_2, GPT_CHANNEL_3, GPT_CHANNEL_4 */
    uint32_t period;  /* unit:ns 1ns~4.29s:1Ghz~0.23hz */
    uint32_t pulse;   /* unit:ns (pulse<=period) */
    uint32_t deadtime; /*dead time from 0 to 1023*/
} T_haltest_pwm_cfg;

T_haltest_pwm_cfg testcfg[] =
{
    {hwp_gptim2, CORE_ID_HCPU, GPTIM2_CH4, GPT_CHANNEL_1, 500000000, 250000000, 0},
#ifndef SF32LB55X
    {hwp_atim1, CORE_ID_HCPU, ATIM1_CH1, GPT_CHANNEL_1, 500000000, 500000000, 1000},
#endif
};  //period:0.5s  pulse:0.25s


static GPT_HandleTypeDef gpt_Handle = {0};

static rt_err_t utest_tc_init(void)
{
    return RT_EOK;
}

static rt_err_t utest_tc_cleanup(void)
{
    return RT_EOK;
}

static HAL_StatusTypeDef pwm_test_init(GPT_HandleTypeDef *htim, T_haltest_pwm_cfg *cfg)
{
    HAL_StatusTypeDef result = HAL_OK;
    GPT_OC_InitTypeDef oc_config = {0};
    GPT_ClockConfigTypeDef clock_config = {0};

    htim->Instance = (GPT_TypeDef *)cfg->instance;
    htim->core = cfg->core;
    htim->Channel = cfg->channel;

    /* configure the timer to pwm mode */
    htim->Init.Prescaler = 0;
    htim->Init.CounterMode = GPT_COUNTERMODE_UP;
    htim->Init.Period = 0;

    if (HAL_GPT_Base_Init(htim) != HAL_OK)
    {
        LOG_E("pwm2_1 base init failed");
        result = HAL_ERROR;
        goto __exit;
    }

    clock_config.ClockSource = GPT_CLOCKSOURCE_INTERNAL;
    if (HAL_GPT_ConfigClockSource(htim, &clock_config) != HAL_OK)
    {
        LOG_E("pwm2_1 clock init failed");
        result = HAL_ERROR;
        goto __exit;
    }

    if (HAL_GPT_PWM_Init(htim) != HAL_OK)
    {
        LOG_E("pwm2_1 init failed");
        result = HAL_ERROR;
        goto __exit;
    }

    oc_config.OCMode = GPT_OCMODE_PWM1;
    oc_config.Pulse = 0;
    oc_config.OCPolarity = GPT_OCPOLARITY_HIGH;
    oc_config.OCFastMode = GPT_OCFAST_DISABLE;

    if (HAL_GPT_PWM_ConfigChannel(htim, &oc_config, cfg->channel) != HAL_OK)
    {
        LOG_E("pwm2_1 config failed");
        result = HAL_ERROR;
        goto __exit;
    }

    /* pwm pin configuration */
    //HAL_GPT_MspPostInit(tim);

    /* enable update request source */
    __HAL_GPT_URS_ENABLE(htim);

__exit:
    return result;
}

static HAL_StatusTypeDef pwm_set(GPT_HandleTypeDef *htim, T_haltest_pwm_cfg *pCfg)
{
    rt_uint32_t period, pulse;
    rt_uint32_t GPT_clock, psc;
    uint32_t max_period;

    if (IS_GPT_ADVANCED_INSTANCE(htim->Instance) != RESET)
        max_period = MAX_PERIOD_ATM;
    else
        max_period = MAX_PERIOD_GPT;

#ifdef  SF32LB52X
    if (pCfg->instance == hwp_gptim2 || pCfg->instance == hwp_btim2)
        GPT_clock = 24000000; /* gptim2 btim2 clk from clk_peri/2 */
    else
#endif
        GPT_clock = HAL_RCC_GetPCLKFreq(htim->core, 1);
    LOG_I("GPT_clock %d", GPT_clock);
    /* Convert nanosecond to frequency and duty cycle. 1s = 1 * 1000 * 1000 * 1000 ns */
    GPT_clock /= 1000000UL;
    period = (unsigned long long)pCfg->period * GPT_clock / 1000ULL;
    psc = period / max_period + 1;
    period = period / psc;
    __HAL_GPT_SET_PRESCALER(htim, psc - 1);
    LOG_I("psc %d, Period %d,", psc, period);

    if (period < MIN_PERIOD)
    {
        period = MIN_PERIOD;
    }
    __HAL_GPT_SET_AUTORELOAD(htim, period - 1);

    pulse = (unsigned long long)pCfg->pulse * GPT_clock / psc / 1000ULL;
    LOG_I("Pulse %d", pulse);
    if (pulse < MIN_PULSE)
    {
        pulse = MIN_PULSE;
    }
    else if (pulse > period)
    {
        pulse = period;
    }
    __HAL_GPT_SET_COMPARE(htim, pCfg->channel, pulse - 1);
    //__HAL_GPT_SET_COUNTER(htim, 0);

    /* Update frequency value */
    HAL_GPT_GenerateEvent(htim, GPT_EVENTSOURCE_UPDATE);
#ifndef SF32LB55X
    TIMEx_BreakDeadTimeConfigTypeDef bdt = {0};
    if (IS_GPT_ADVANCED_INSTANCE(htim->Instance) != RESET)
    {
        bdt.AutomaticOutput = 0;
        bdt.BreakFilter = 0;
        bdt.BreakPolarity = 0;
        bdt.BreakState = 0;
        bdt.Break2Filter = 0;
        bdt.Break2Polarity = 0;
        bdt.Break2State = 0;
        bdt.DeadTime = pCfg->deadtime;
        bdt.OffStateIDLEMode = 0;
        bdt.OffStateRunMode = 0;
        bdt.DeadTimePsc = 0;
        HAL_TIMEx_ConfigBreakDeadTime(htim, &bdt);
    }
#endif

    return RT_EOK;
}

#ifdef  SF32LB55X
    #define PAD_PA_14 PAD_PA14
    //#define GPTIM2_CH_0 GPTIM2_CH4
#elif defined(SF32LB58X)
    #define PAD_PA_25 PAD_PA25
    //#define GPTIM2_CH_0 GPTIM2_CH1
#elif defined(SF32LB56X)
    #define PAD_PA_05 PAD_PA05
    //#define GPTIM2_CH_0 GPTIM2_CH1
#elif defined(SF32LB52X)
    #define PAD_PA_35 PAD_PA35
    #define PAD_PA_36 PAD_PA36
    #define PAD_PA_00 PAD_PA00
    //#define GPTIM2_CH_0 GPTIM2_CH1
#endif

void pwm_test_pinset(T_haltest_pwm_cfg *cfg)
{
#ifdef  SF32LB55X
    HAL_PIN_Set(PAD_PA_14, cfg->pad_func, PIN_PULLUP, 1);
#elif defined(SF32LB58X)
    HAL_PIN_Set(PAD_PA_25, cfg->pad_func, PIN_PULLUP, 1);
#elif defined(SF32LB56X)
    HAL_PIN_Set(PAD_PA_05, cfg->pad_func, PIN_PULLUP, 1);
#elif defined(SF32LB52X)
    if (IS_GPT_ADVANCED_INSTANCE(gpt_Handle.Instance) != RESET)
    {
        HAL_PIN_Set(PAD_PA_36, cfg->pad_func, PIN_PULLUP, 1);
        HAL_PIN_Set(PAD_PA_00, cfg->pad_func + 1, PIN_PULLUP, 1);
    }
    else
    {
        HAL_PIN_Set(PAD_PA_35, cfg->pad_func, PIN_PULLUP, 1);
    }
#endif
}

/* eg: GPT2 CH1 work in pwm mode */
static void testcase(int argc, char **argv)
{
    int pwm_test_num = 10;
    HAL_StatusTypeDef ret;

    for (int i = 0; i < sizeof(testcfg) / sizeof(T_haltest_pwm_cfg); i++)
    {

        /* configure in pwm mode  */
        memset(&gpt_Handle, 0, sizeof(GPT_HandleTypeDef));

        ret = pwm_test_init(&gpt_Handle, &testcfg[i]);

        if (ret != HAL_OK)
        {
            uassert_true(false);
            return;
        }

        /* cal and set the pwm run para  */
        pwm_set(&gpt_Handle, &testcfg[i]);

        /* configure pinmux */
        pwm_test_pinset(&testcfg[i]);

        /* start pwm  */
        HAL_GPT_PWM_Start(&gpt_Handle, testcfg[i].channel);
#ifndef SF32LB55X
        if (IS_GPT_ADVANCED_INSTANCE(gpt_Handle.Instance) != RESET)
        {
            HAL_TIMEx_PWMN_Stop(&gpt_Handle, testcfg[i].channel);
            HAL_TIMEx_PWMN_Start(&gpt_Handle, testcfg[i].channel);
        }
#endif
        /* wait for pwm_test_num period  */
        rt_tick_t cur_tick;
        cur_tick = rt_tick_get();
        while (rt_tick_get() - cur_tick <= testcfg[i].period / 1000000 * (pwm_test_num + 1))
        {
            rt_thread_mdelay(10);
        }

        /* stop pwm  */
        HAL_GPT_PWM_Stop(&gpt_Handle, testcfg[i].channel);
#ifndef SF32LB55X
        if (IS_GPT_ADVANCED_INSTANCE(gpt_Handle.Instance) != RESET)
            HAL_TIMEx_PWMN_Stop(&gpt_Handle, testcfg[i].channel);
#endif
    }
}


UTEST_TC_EXPORT(testcase, "example_pwm", utest_tc_init, utest_tc_cleanup, 10);


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
