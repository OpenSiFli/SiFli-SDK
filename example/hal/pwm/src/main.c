#include "rtconfig.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"
#include "rtthread.h"


//********************************************************************************************************//
//**********************************************atimer pwm_start******************************************//
//********************************************************************************************************//

#define PWM_CHANNEL2 1
GPT_HandleTypeDef atim_Handle = {0};
GPT_OC_InitTypeDef oc_config = {0};
GPT_ClockConfigTypeDef clock_config = {0};
unsigned int freq, percentage1, percentage2;

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

    HAL_TIMEx_ConfigBreakDeadTime(&atim_Handle, &bdt);
}

//#define PWM_PERIOD    (500000000) //ns
//#define PWM_PULSE     (250000000) //ns
#define MAX_PERIOD_ATM (0xFFFFFFFF) //32bit

// modify the parameter：（frequency: hz,percentage1: duty cycle:%，duty cycle:% ）
void ATIM1_Modify_Param(unsigned int frequency, unsigned int percentage1, unsigned int percentage2)
{
    unsigned int  PWM_PERIOD;
    unsigned int  PWM_PULSE;

    rt_kprintf("frequency:%d,percentage1:%d,percentage2:%d\n", frequency, percentage1, percentage2);
    /* limite the duty's range of input */
    if ((percentage1 > 100) || (percentage1 < 0) || (frequency > 200000) || (frequency < 1))
    {
        rt_kprintf("parameter err!!! frequency:%d,percentage1:%d\n", frequency, percentage1);
        //return;
    }
    /* set the whole period (ns) */
    PWM_PERIOD = (1 * 1000 * 1000 * 1000) / frequency;
    /* set the peroid of positive pulse (ns) */
    PWM_PULSE = (PWM_PERIOD * percentage1) / 100;

    unsigned int period, pulse;
    unsigned int GPT_clock, psc;

    GPT_clock = HAL_RCC_GetPCLKFreq(CORE_ID_HCPU, 1);
    /* Convert nanosecond to frequency and duty cycle. 1s = 1 * 1000 * 1000 * 1000 ns */
    rt_kprintf("channel1 atim1 plck freq:%d,PWM_PERIOD:%d,PWM_PULSE:%d\n", GPT_clock, PWM_PERIOD, PWM_PULSE);
    GPT_clock /= 1000000UL;
    period = (unsigned long long)PWM_PERIOD * GPT_clock / 1000ULL;
    psc = period / MAX_PERIOD_ATM + 1;
    period = period / psc;
    rt_kprintf("atim1 GPT_clock:%d,period:%d,psc:%d\n", GPT_clock, period, psc);
    /*set atimer prescaler*/
    __HAL_GPT_SET_PRESCALER(&atim_Handle, psc - 1);
    /*set atimer auto reload*/
    __HAL_GPT_SET_AUTORELOAD(&atim_Handle, period - 1);
    /*set atimer pulse*/
    pulse = (unsigned long long)PWM_PULSE * GPT_clock / psc / 1000ULL;
    rt_kprintf("atim1 pulse:%d\n", pulse);
    __HAL_GPT_SET_COMPARE(&atim_Handle, GPT_CHANNEL_1, pulse - 1);
#ifdef PWM_CHANNEL2
    if ((percentage2 > 100) || (percentage2 < 0))
    {
        rt_kprintf("parameter err!!! frequency:%d,percentage2:%d\n", frequency, percentage2);
    }
    PWM_PERIOD = (1 * 1000 * 1000 * 1000) / frequency;
    PWM_PULSE = (PWM_PERIOD * percentage2) / 100;
    //PWM_PERIOD =  (500000000); //ns
    //PWM_PULSE =   (250000000); //ns

    GPT_clock = HAL_RCC_GetPCLKFreq(CORE_ID_HCPU, 1);
    /* Convert nanosecond to frequency and duty cycle. 1s = 1 * 1000 * 1000 * 1000 ns */
    rt_kprintf("channel2 atim1 plck freq:%d,PWM_PERIOD:%d,PWM_PULSE:%d\n", GPT_clock, PWM_PERIOD, PWM_PULSE);
    GPT_clock /= 1000000UL;
    period = (unsigned long long)PWM_PERIOD * GPT_clock / 1000ULL;
    psc = period / MAX_PERIOD_ATM + 1;
    period = period / psc;
    rt_kprintf("atim1 GPT_clock:%d,period:%d,psc:%d\n", GPT_clock, period, psc);
    /*set atimer prescaler*/
//  __HAL_GPT_SET_PRESCALER(&atimt_Handle, psc - 1);
    /*set atimer auto reload*/
//  __HAL_GPT_SET_AUTORELOAD(&atim_Handle, period - 1);
    /*set atimer pulse*/
    pulse = (unsigned long long)PWM_PULSE * GPT_clock / psc / 1000ULL;

    __HAL_GPT_SET_COMPARE(&atim_Handle, GPT_CHANNEL_2, pulse - 1);
#endif
    ATIM1_PWM_Break_dead_time_set(freq);
    HAL_GPT_GenerateEvent(&atim_Handle, GPT_EVENTSOURCE_UPDATE);

}

/* ATIM1 init function */
void ATIM1_Init(void)
{
    rt_kprintf("ATIM1_Init\n");
#if defined(BSP_USING_BOARD_EM_LB525XXX)
    atim_Handle.Instance = (GPT_TypeDef *)hwp_atim1;
#elif defined (BSP_USING_BOARD_EM_LB587XXX)
    atim_Handle.Instance = (GPT_TypeDef *)hwp_atim2;
#endif

    atim_Handle.core = CORE_ID_HCPU;
    atim_Handle.Channel = GPT_CHANNEL_1;
    atim_Handle.Init.CounterMode = GPT_COUNTERMODE_UP;
    /*atimer base init*/
    if (HAL_GPT_Base_Init(&atim_Handle) != HAL_OK)
    {
        rt_kprintf("atimer base init failed");
        return;
    }
    /*atimer clock source select*/
    clock_config.ClockSource = GPT_CLOCKSOURCE_INTERNAL;
    if (HAL_GPT_ConfigClockSource(&atim_Handle, &clock_config) != HAL_OK)
    {
        rt_kprintf("atimer clock init failed");
        return;
    }
    /*atimer pwm init*/
    if (HAL_GPT_PWM_Init(&atim_Handle) != HAL_OK)
    {
        rt_kprintf("atimer pwm init failed");
        return;
    }
    /*atimer pwm channel config*/
    oc_config.OCMode = GPT_OCMODE_PWM1;
    oc_config.Pulse = 0;
    oc_config.OCPolarity = GPT_OCPOLARITY_HIGH;
    oc_config.OCFastMode = GPT_OCFAST_DISABLE;
    if (HAL_GPT_PWM_ConfigChannel(&atim_Handle, &oc_config, GPT_CHANNEL_1) != HAL_OK)
    {
        rt_kprintf("atimer pwm channel config failed");
        return;
    }
    ATIM1_PWM_Break_dead_time_set(freq);
    HAL_GPT_PWM_Start(&atim_Handle, GPT_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&atim_Handle, GPT_CHANNEL_1);

#ifdef PWM_CHANNEL2
    if (HAL_GPT_PWM_ConfigChannel(&atim_Handle, &oc_config, GPT_CHANNEL_2) != HAL_OK)
    {
        rt_kprintf("atimer pwm channel config failed");
        return;
    }
    ATIM1_PWM_Break_dead_time_set(freq);
    HAL_GPT_PWM_Start(&atim_Handle, GPT_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&atim_Handle, GPT_CHANNEL_2);
#endif

}
void ATIM1_Stop(void)
{
    HAL_GPT_PWM_Stop(&atim_Handle, GPT_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&atim_Handle, GPT_CHANNEL_1);
#ifdef PWM_CHANNEL2
    if (HAL_GPT_PWM_ConfigChannel(&atim_Handle, &oc_config, GPT_CHANNEL_2) != HAL_OK)
    {
        rt_kprintf("atimer pwm channel config failed");
        return;
    }
    HAL_GPT_PWM_Stop(&atim_Handle, GPT_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&atim_Handle, GPT_CHANNEL_2);
#endif
    rt_kprintf("ATIM1_Stop!\n");

}
//********************************************************************************************************//
//**********************************************atimer pwm_end******************************************//
//********************************************************************************************************//


#define MAX_PERIOD_GPT (0xFFFF)
#define MAX_PERIOD_ATM (0xFFFFFFFF)
#define MIN_PERIOD 3
#define MIN_PULSE 2

typedef struct
{
    void *instance;
    unsigned char core;
    unsigned int pad_func;
    unsigned int channel; /* GPT_CHANNEL_1, GPT_CHANNEL_2, GPT_CHANNEL_3, GPT_CHANNEL_4 */
    unsigned int period;  /* unit:ns 1ns~4.29s:1Ghz~0.23hz */
    unsigned int pulse;   /* unit:ns (pulse<=period) */
    unsigned int deadtime; /*dead time from 0 to 1023*/
} T_haltest_pwm_cfg;
#if defined(BSP_USING_BOARD_EM_LB525XXX)

T_haltest_pwm_cfg testcfg[] =
{
    {hwp_gptim2, CORE_ID_HCPU, GPTIM2_CH1, GPT_CHANNEL_1, 5000000, 1000000, 0},
};  //period:0.5s  pulse:0.25s
#elif defined BSP_USING_BOARD_EM_LB587XXX
T_haltest_pwm_cfg testcfg[] =
{
    {hwp_gptim1, CORE_ID_HCPU, GPTIM1_CH2, GPT_CHANNEL_2, 5000000, 1000000, 0},
};  //period:0.5s  pulse:0.25s
#endif


static GPT_HandleTypeDef gpt_Handle = {0};

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
        rt_kprintf("pwm2_1 base init failed");
        result = HAL_ERROR;
        goto __exit;
    }

    clock_config.ClockSource = GPT_CLOCKSOURCE_INTERNAL;
    if (HAL_GPT_ConfigClockSource(htim, &clock_config) != HAL_OK)
    {
        rt_kprintf("pwm2_1 clock init failed");
        result = HAL_ERROR;
        goto __exit;
    }

    if (HAL_GPT_PWM_Init(htim) != HAL_OK)
    {
        rt_kprintf("pwm2_1 init failed");
        result = HAL_ERROR;
        goto __exit;
    }

    oc_config.OCMode = GPT_OCMODE_PWM1;
    oc_config.Pulse = 0;
    oc_config.OCPolarity = GPT_OCPOLARITY_HIGH;
    oc_config.OCFastMode = GPT_OCFAST_DISABLE;

    if (HAL_GPT_PWM_ConfigChannel(htim, &oc_config, cfg->channel) != HAL_OK)
    {
        rt_kprintf("pwm2_1 config failed");
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
    unsigned int period, pulse;
    unsigned int GPT_clock, psc;
    unsigned int max_period;

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
    rt_kprintf("GPT_clock %d,", GPT_clock);
    /* Convert nanosecond to frequency and duty cycle. 1s = 1 * 1000 * 1000 * 1000 ns */
    GPT_clock /= 1000000UL;
    period = (unsigned long long)pCfg->period * GPT_clock / 1000ULL;
    psc = period / max_period + 1;
    period = period / psc;
    __HAL_GPT_SET_PRESCALER(htim, psc - 1);
    rt_kprintf("psc %d, Period %d,", psc, period);

    if (period < MIN_PERIOD)
    {
        period = MIN_PERIOD;
    }
    __HAL_GPT_SET_AUTORELOAD(htim, period - 1);

    pulse = (unsigned long long)pCfg->pulse * GPT_clock / psc / 1000ULL;
    rt_kprintf("Pulse %d", pulse);
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

    return HAL_OK;
}

#ifdef  SF32LB55X
    #define PAD_PB_22 PAD_PB22
    //#define GPTIM2_CH_0 GPTIM2_CH1
#elif defined(BSP_USING_BOARD_EM_LB587XXX)
    #define PAD_PA_51 PAD_PA51
    //#define GPTIM2_CH_0 GPTIM2_CH1
#elif defined(SF32LB56X)
    #define PAD_PA_05 PAD_PA05
    //#define GPTIM2_CH_0 GPTIM2_CH1
#elif defined(SF32LB52X)
    #define PAD_PA_09 PAD_PA09

    //#define GPTIM2_CH_0 GPTIM2_CH1
#endif

void pwm_test_pinset(T_haltest_pwm_cfg *cfg)
{
#ifdef  SF32LB55X
    HAL_PIN_Set(PAD_PB_22, cfg->pad_func, PIN_PULLUP, 0);
#elif defined(BSP_USING_BOARD_EM_LB587XXX)
    HAL_PIN_Set(PAD_PA_51, cfg->pad_func, PIN_PULLUP, 1);
#elif defined(SF32LB56X)
    HAL_PIN_Set(PAD_PA_05, cfg->pad_func, PIN_PULLUP, 1);
#elif defined(SF32LB52X)
    HAL_PIN_Set(PAD_PA_09, cfg->pad_func, PIN_PULLUP, 1); /* GPTIM */
#endif
}

/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    HAL_StatusTypeDef  ret = HAL_OK;

    /* Output a message on console using printf function */
    rt_kprintf("Start gtimer pwm demo!\n");

    for (int i = 0; i < sizeof(testcfg) / sizeof(T_haltest_pwm_cfg); i++)
    {

        /* clear the handle of Gptimer */
        memset(&gpt_Handle, 0, sizeof(GPT_HandleTypeDef));
        /* configure in pwm mode  */
        ret = pwm_test_init(&gpt_Handle, &testcfg[i]);

        if (ret != HAL_OK)
        {
            rt_kprintf("pwm_test_init error!\n");
        }

        /* cal and set the pwm run para  */
        pwm_set(&gpt_Handle, &testcfg[i]);

        /* configure pinmux */
        pwm_test_pinset(&testcfg[i]);

        /* start pwm  */
        HAL_GPT_PWM_Start(&gpt_Handle, testcfg[i].channel);

        /* wait for pwm_test_num period  */
        HAL_Delay(1000);

        /* stop pwm  */
        HAL_GPT_PWM_Stop(&gpt_Handle, testcfg[i].channel);
#ifndef SF32LB55X
        if (IS_GPT_ADVANCED_INSTANCE(gpt_Handle.Instance) != RESET)
            HAL_TIMEx_PWMN_Stop(&gpt_Handle, testcfg[i].channel);
#endif
    }

    rt_kprintf("gtimer pwm demo end!\n");

    rt_kprintf("Start atimer pwm demo!\n");
    /* configure pinmux */

#if defined(BSP_USING_BOARD_EM_LB525XXX)
    HAL_PIN_Set(PAD_PA00, ATIM1_CH1,  PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA02, ATIM1_CH1N, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA03, ATIM1_CH2,  PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA04, ATIM1_CH2N, PIN_PULLUP, 1);
#elif defined(BSP_USING_BOARD_EM_LB587XXX)

    HAL_PIN_Set(PAD_PA84, ATIM2_CH1,  PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA86, ATIM2_CH1N, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA90, ATIM2_CH2,  PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA91, ATIM2_CH2N, PIN_PULLUP, 1);
#endif

    freq = 50000;   /* 50000 is frequence of pwm（50000hz）*/
    percentage1 = 50; /* 50 is the duty cycle of Channel 1（50%）*/
    percentage2 = 30; /* 50 is the duty cycle of Channel 2（50%）*/

    ATIM1_Init();
    ATIM1_Modify_Param(freq, percentage1, percentage2);
    HAL_Delay(2000);
    freq = 10000;   /* 10000 is frequence of pwm（10000hz）*/
    percentage1 = 5; /* 5 is the duty cycle of Channel 1（5%）*/
    percentage2 = 90; /* 90 is the duty cycle of Channel 2（90%）*/
    ATIM1_Modify_Param(freq, percentage1, percentage2);
    HAL_Delay(2000);
    freq = 1000; //hz
    percentage1 = 95;
    percentage2 = 5;
    ATIM1_Modify_Param(freq, percentage1, percentage2);
    HAL_Delay(2000);

    ATIM1_Stop();
    rt_kprintf("atimer pwm demo end!\n");
    while (1);
    return 0;
}

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

