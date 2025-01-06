/**
  ******************************************************************************
  * @file   main.c
  * @author Sifli software development team
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2021 - 2021,  Sifli Technology
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

#include "app_common.h"

#define LOG_TAG "pwm_app"
#include "log.h"

//********************************************************************************************************//
//**********************************************pwm_start*************************************************//
//********************************************************************************************************//

#define PWM_CHANNEL2 1
GPT_HandleTypeDef gpt_Handle = {0};
GPT_OC_InitTypeDef oc_config = {0};
GPT_ClockConfigTypeDef clock_config = {0};
rt_uint32_t freq, percentage1, percentage2;

void ATIM1_PWM_Break_dead_time_set(uint32_t freq)
{
    TIMEx_BreakDeadTimeConfigTypeDef bdt = {0};
    uint32_t dead_time;
    if (freq > 1000)
        dead_time = 200 * 2 / (freq / 1000);
    else
        dead_time = 200;
    bdt.AutomaticOutput = 0;
    bdt.BreakFilter = 0;
    bdt.BreakPolarity = 0;
    bdt.BreakState = 0;
    bdt.Break2Filter = 0;
    bdt.Break2Polarity = 0;
    bdt.Break2State = 0;
    bdt.DeadTime = dead_time; /*0~1023*/
    bdt.OffStateIDLEMode = 0;
    bdt.OffStateRunMode = 0;
    bdt.DeadTimePsc = 0;

    HAL_TIMEx_ConfigBreakDeadTime(&gpt_Handle, &bdt);
}

//#define PWM_PERIOD    (500000000) //ns
//#define PWM_PULSE (250000000) //ns
#define MAX_PERIOD_ATM (0xFFFFFFFF) //32bit

// 修改PWM的参数：（频率hz,CH1的占空比%，CH2的占空比%）
void ATIM1_Modify_Param(uint32_t frequency, uint32_t percentage1, uint32_t percentage2)
{
    uint32_t  PWM_PERIOD;
    uint32_t  PWM_PULSE;

    rt_kprintf("frequency:%d,percentage1:%d,percentage2:%d\n", frequency, percentage1, percentage2);

    if ((percentage1 > 100) || (percentage1 < 0) || (frequency > 200000) || (frequency < 1))
    {
        rt_kprintf("parameter err!!! frequency:%d,percentage1:%d\n", frequency, percentage1);
        //return;
    }
    PWM_PERIOD = (1 * 1000 * 1000 * 1000) / frequency;
    PWM_PULSE = (PWM_PERIOD * percentage1) / 100;
    //PWM_PERIOD =  (500000000);
    //PWM_PULSE =   (250000000);

    rt_uint32_t period, pulse;
    rt_uint32_t GPT_clock, psc;

    GPT_clock = HAL_RCC_GetPCLKFreq(CORE_ID_HCPU, 1);
    /* Convert nanosecond to frequency and duty cycle. 1s = 1 * 1000 * 1000 * 1000 ns */
    rt_kprintf("channel1 atim1 plck freq:%d,PWM_PERIOD:%d,PWM_PULSE:%d\n", GPT_clock, PWM_PERIOD, PWM_PULSE);
    GPT_clock /= 1000000UL;
    period = (unsigned long long)PWM_PERIOD * GPT_clock / 1000ULL;
    psc = period / MAX_PERIOD_ATM + 1;
    period = period / psc;
    rt_kprintf("atim1 GPT_clock:%d,period:%d,psc:%d\n", GPT_clock, period, psc);
    /*set atimer prescaler*/
    __HAL_GPT_SET_PRESCALER(&gpt_Handle, psc - 1);
    /*set atimer auto reload*/
    __HAL_GPT_SET_AUTORELOAD(&gpt_Handle, period - 1);
    /*set atimer pulse*/
    pulse = (unsigned long long)PWM_PULSE * GPT_clock / psc / 1000ULL;
    rt_kprintf("atim1 pulse:%d\n", pulse);
    __HAL_GPT_SET_COMPARE(&gpt_Handle, GPT_CHANNEL_1, pulse - 1);
#ifdef PWM_CHANNEL2
    if ((percentage2 > 100) || (percentage2 < 0))
    {
        rt_kprintf("parameter err!!! frequency:%d,percentage2:%d\n", frequency, percentage2);
        //return;
    }
    PWM_PERIOD = (1 * 1000 * 1000 * 1000) / frequency;
    PWM_PULSE = (PWM_PERIOD * percentage2) / 100;
    //PWM_PERIOD =  (500000000);
    //PWM_PULSE =   (250000000);

    GPT_clock = HAL_RCC_GetPCLKFreq(CORE_ID_HCPU, 1);
    /* Convert nanosecond to frequency and duty cycle. 1s = 1 * 1000 * 1000 * 1000 ns */
    rt_kprintf("channel2 atim1 plck freq:%d,PWM_PERIOD:%d,PWM_PULSE:%d\n", GPT_clock, PWM_PERIOD, PWM_PULSE);
    GPT_clock /= 1000000UL;
    period = (unsigned long long)PWM_PERIOD * GPT_clock / 1000ULL;
    psc = period / MAX_PERIOD_ATM + 1;
    period = period / psc;
    rt_kprintf("atim1 GPT_clock:%d,period:%d,psc:%d\n", GPT_clock, period, psc);
    /*set atimer prescaler*/
//  __HAL_GPT_SET_PRESCALER(&gpt_Handle, psc - 1);
    /*set atimer auto reload*/
//  __HAL_GPT_SET_AUTORELOAD(&gpt_Handle, period - 1);
    /*set atimer pulse*/
    pulse = (unsigned long long)PWM_PULSE * GPT_clock / psc / 1000ULL;

    __HAL_GPT_SET_COMPARE(&gpt_Handle, GPT_CHANNEL_2, pulse - 1);
#endif
    ATIM1_PWM_Break_dead_time_set(freq);
    HAL_GPT_GenerateEvent(&gpt_Handle, GPT_EVENTSOURCE_UPDATE);

}

/* ATIM1 init function */
void ATIM1_Init(void)
{
    rt_kprintf("ATIM1_Init entry!\n");

    gpt_Handle.Instance = (GPT_TypeDef *)hwp_atim1;
    gpt_Handle.core = CORE_ID_HCPU;
    gpt_Handle.Channel = GPT_CHANNEL_1;
    gpt_Handle.Init.CounterMode = GPT_COUNTERMODE_UP;
    /*atimer base init*/
    if (HAL_GPT_Base_Init(&gpt_Handle) != HAL_OK)
    {
        LOG_E("atimer base init failed");
        return;
    }
    /*atimer clock source select*/
    clock_config.ClockSource = GPT_CLOCKSOURCE_INTERNAL;
    if (HAL_GPT_ConfigClockSource(&gpt_Handle, &clock_config) != HAL_OK)
    {
        LOG_E("atimer clock init failed");
        return;
    }
    /*atimer pwm init*/
    if (HAL_GPT_PWM_Init(&gpt_Handle) != HAL_OK)
    {
        LOG_E("atimer pwm init failed");
        return;
    }
    /*atimer pwm channel config*/
    oc_config.OCMode = GPT_OCMODE_PWM1;
    oc_config.Pulse = 0;
    oc_config.OCPolarity = GPT_OCPOLARITY_HIGH;
    oc_config.OCFastMode = GPT_OCFAST_DISABLE;
    if (HAL_GPT_PWM_ConfigChannel(&gpt_Handle, &oc_config, GPT_CHANNEL_1) != HAL_OK)
    {
        LOG_E("atimer pwm channel config failed");
        return;
    }
    ATIM1_PWM_Break_dead_time_set(freq);
    HAL_GPT_PWM_Start(&gpt_Handle, GPT_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&gpt_Handle, GPT_CHANNEL_1);

#ifdef PWM_CHANNEL2
    if (HAL_GPT_PWM_ConfigChannel(&gpt_Handle, &oc_config, GPT_CHANNEL_2) != HAL_OK)
    {
        LOG_E("atimer pwm channel config failed");
        return;
    }
    ATIM1_PWM_Break_dead_time_set(freq);
    HAL_GPT_PWM_Start(&gpt_Handle, GPT_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&gpt_Handle, GPT_CHANNEL_2);
#endif
    rt_kprintf("ATIM1_Init exit!\n");

}
void ATIM1_Stop(void)
{
    HAL_GPT_PWM_Stop(&gpt_Handle, GPT_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&gpt_Handle, GPT_CHANNEL_1);
#ifdef PWM_CHANNEL2
    if (HAL_GPT_PWM_ConfigChannel(&gpt_Handle, &oc_config, GPT_CHANNEL_2) != HAL_OK)
    {
        LOG_E("atimer pwm channel config failed");
        return;
    }
    HAL_GPT_PWM_Stop(&gpt_Handle, GPT_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&gpt_Handle, GPT_CHANNEL_2);
#endif
    rt_kprintf("ATIM1_Stop exit!\n");

}

#if defined(BSP_USING_PWM_LPTIM2)
#include "drv_pwm_lptim.h"
static rt_device_t lpwmdevice;
extern void print_sysinfo(char *buf, uint32_t buf_len);
/**
* @brief  lptimer set param,only support prescaler = 1, if clock source is pclk2, peroid scop: 133ns ~ 8.761ms
* @param[in]  hpwm: pwm device object handle.
* @param[in]  pwm_period lptimer pwm period  0~ 65536
* @param[in]  pwm_pulse  lptimer pwm pulse  pulse <= period, 0~ 65536
* @retval RT_EOK if success, otherwise -RT_ERROR
*/
rt_err_t lp_timer_start(struct bf0_pwm_lp *hpwm, rt_uint32_t pwm_period, rt_uint32_t pwm_pulse)
{
    rt_uint32_t period = pwm_period;
    rt_uint32_t pulse = pwm_pulse;
    if (period > MAX_PERIOD)
    {
        rt_kprintf("%s over max period;\n", __FUNCTION__);
        return -RT_ERROR;
    }

    if (period < MIN_PERIOD)
        period = MIN_PERIOD;

    if (pulse < MIN_PULSE)
    {
        pulse = MIN_PULSE;
    }
    else if (pulse > period)
    {
        pulse = period;
    }
    HAL_LPTIM_PWM_Start(&(hpwm->tim_handle), period, MIN_PULSE, LPTIM_PRESCALER_DIV1);
    return RT_EOK;
}


rt_err_t lp_timer_set_param(struct bf0_pwm_lp *hpwm, rt_uint32_t pwm_period, rt_uint32_t pwm_pulse)
{
    rt_uint32_t period = pwm_period;
    rt_uint32_t pulse = pwm_pulse;

    if (period > MAX_PERIOD)
    {
        rt_kprintf("%s over max period;\n", __FUNCTION__);
        return -RT_ERROR;
    }

    if (period < MIN_PERIOD)
        period = MIN_PERIOD;

    if (pulse < MIN_PULSE)
    {
        pulse = MIN_PULSE;
    }
    else if (pulse > period)
    {
        pulse = period;
    }

    HAL_LPTIM_PWM_Set_Period(&(hpwm->tim_handle), period, pulse, LPTIM_PRESCALER_DIV1);
    return RT_EOK;
}


void lp_timer_stop(struct bf0_pwm_lp *hpwm)
{
    __HAL_LPTIM_DISABLE(&(hpwm->tim_handle));
}
#define PERIOD_MIN  10
#define PEROID_MAX  80

void lptimer_pwm_init()
{
    print_sysinfo(NULL, 0);

    HAL_PIN_Set(PAD_PA10, LPTIM2_OUT, PIN_PULLUP, 1);
    lpwmdevice = rt_device_find("pwmlp2");
    if (!lpwmdevice)
    {
        rt_kprintf("Find %s device fail\n", "pwmlp2");
        return;
    }
    struct rt_pwm_configuration lpwm_config;
    lpwm_config.reserved = LPTIME_PWM_CLK_SOURCE_USING_APBCLK;
    rt_device_control(lpwmdevice, PWM_CMD_SET_CLK_SOURECE, (void *)&lpwm_config);
    rt_kprintf("Find %s device ok!\n", "pwmlp2");
}

void lptimer_pwm_start()
{
    if (!lpwmdevice)
    {
        LOG_I("Find lppwme device fail!\n");
        return;
    }
    uint16_t percent = 10;
    uint16_t period = 10;
    struct bf0_pwm_lp *hpwm = (struct bf0_pwm_lp *) lpwmdevice->user_data;
    struct rt_pwm_configuration lpwm_config;
    rt_uint32_t fpclk2 = HAL_RCC_GetPCLKFreq(CORE_ID_HCPU, 0);   /*get pclk2*/
    lpwm_config.period = PERIOD_MIN * (fpclk2 / 1000) / 1000;
    lpwm_config.pulse = lpwm_config.period * 50 / 100;
    lp_timer_start(hpwm, lpwm_config.period, lpwm_config.pulse);

    {
        period = 10;
        for (period = 10; period <= PEROID_MAX; period += 10)
        {
            lpwm_config.period = period * (fpclk2 / 1000) / 1000;
            percent = 10;
            for (percent = 10; percent < 101; percent += 10)
            {
                lpwm_config.pulse =  lpwm_config.period * percent / 100;
                /*rt_kprintf("lptime set peroid = %d; pulse = %d;\n", lpwm_config.period, lpwm_config.pulse);*/
                lp_timer_set_param(hpwm, lpwm_config.period, lpwm_config.pulse);
                HAL_Delay_us(300);
            }
        }
    }

    lpwm_config.period = PERIOD_MIN * (fpclk2 / 1000) / 1000;
    lpwm_config.pulse =  lpwm_config.period * 10 / 100;
    lp_timer_set_param(hpwm, lpwm_config.period, lpwm_config.pulse);
    rt_kprintf(" %s start!\n", "pwmlp2");
}


void lptimer_pwm_stop()
{
    if (!lpwmdevice)
    {
        rt_kprintf("Find lppwme device fail!\n");
        return;
    }
    struct rt_pwm_configuration lpwm_config;
    rt_device_control((rt_device_t)lpwmdevice, PWM_CMD_DISABLE, (void *)&lpwm_config);
    rt_kprintf(" %s stop!\n", "pwmlp1");

}

int cmd_lpwm_test(int argc, char *argv[])
{
    if (strcmp(argv[1], "-init") == 0)
    {
        lptimer_pwm_init();
    }
    else if (strcmp(argv[1], "-start") == 0)
    {
        lptimer_pwm_start();
    }
    else if (strcmp(argv[1], "-stop") == 0)
    {
        lptimer_pwm_stop();

    }
    return (-RT_EINVAL);
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_lpwm_test, __cmd_lpwm_test, Test lptimer pwm driver);
#endif
void pwm_task(void *parameter)
{
    freq = 50000; //hz
    percentage1 = 50;
    percentage2 = 50;
    ATIM1_Init();
    ATIM1_Modify_Param(freq, percentage1, percentage2);
    HAL_PIN_Set(PAD_PA00, ATIM1_CH1,  PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA01, ATIM1_CH1N, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA02, ATIM1_CH2,  PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA03, ATIM1_CH2N, PIN_PULLUP, 1);
}

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
