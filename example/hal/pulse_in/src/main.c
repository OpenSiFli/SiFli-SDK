/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "rtconfig.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"
#include "rtthread.h"
#include "rthw.h"   /* rt_hw_interrupt_disable/enable for the capture snapshot */

#define MAX_PERIOD_GPT (0xFFFF)
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
} T_haltest_pwm_cfg;
#if defined(SF32LB52X)
static T_haltest_pwm_cfg testcfg[] =
{
    {hwp_gptim2, CORE_ID_HCPU, GPTIM2_CH1, GPT_CHANNEL_1, 5000000, 1000000},
};  //period:5ms  pulse:1ms
#elif defined(SF32LB56X)
static T_haltest_pwm_cfg testcfg[] =
{
    {hwp_gptim2, CORE_ID_HCPU, GPTIM2_CH1, GPT_CHANNEL_1, 5000000, 1000000},
};  //period:5ms  pulse:1ms
#elif defined(SF32LB58X)
static T_haltest_pwm_cfg testcfg[] =
{
    {hwp_gptim1, CORE_ID_HCPU, GPTIM1_CH2, GPT_CHANNEL_2, 5000000, 1000000},
};  //period:5ms  pulse:1ms
#endif

/* input capture: 52x/56x use GPTIM1_CH1 (PA27), 58x uses GPTIM3_CH2
 * (PB01, LPSYS). PWM-input mode: period channel (rising, direct TIx)
 * / pulse channel (falling, indirect TIx, same pin). The counter is reset on
 * the period edge via slave mode reset (TI1FP1; 58x uses TI2FP2 on CH2). */
#if defined(SF32LB52X) || defined(SF32LB56X)

    #define CAPTURE_INSTANCE     hwp_gptim1
    #define CAPTURE_CORE         CORE_ID_HCPU
    #define CAPTURE_TRIGGER      GPT_TS_TI1FP1
    #define CAPTURE_PERIOD_CH    GPT_CHANNEL_1
    #define CAPTURE_PULSE_CH     GPT_CHANNEL_2
    #define CAPTURE_PERIOD_FLAG  GPT_FLAG_CC1
    #define CAPTURE_PULSE_FLAG   GPT_FLAG_CC2
    #define CAPTURE_PERIOD_ACT   HAL_GPT_ACTIVE_CHANNEL_1
    #define CAPTURE_PULSE_ACT    HAL_GPT_ACTIVE_CHANNEL_2
    #define CAPTURE_IRQn         GPTIM1_IRQn
    #define CAPTURE_IRQ_HANDLER  GPTIM1_IRQHandler

#elif defined(SF32LB58X)

    #define CAPTURE_INSTANCE     hwp_gptim3
    #define CAPTURE_CORE         CORE_ID_LCPU
    #define CAPTURE_TRIGGER      GPT_TS_TI2FP2
    #define CAPTURE_PERIOD_CH    GPT_CHANNEL_2
    #define CAPTURE_PULSE_CH     GPT_CHANNEL_1
    #define CAPTURE_PERIOD_FLAG  GPT_FLAG_CC2
    #define CAPTURE_PULSE_FLAG   GPT_FLAG_CC1
    #define CAPTURE_PERIOD_ACT   HAL_GPT_ACTIVE_CHANNEL_2
    #define CAPTURE_PULSE_ACT    HAL_GPT_ACTIVE_CHANNEL_1
    #define CAPTURE_IRQn         GPTIM3_IRQn
    #define CAPTURE_IRQ_HANDLER  GPTIM3_IRQHandler

#endif

#define CAPTURE_PRESCALER 9
#define INPUT_CAPTURE_USE_IT 1

static GPT_HandleTypeDef capture_Handle = {0};
static GPT_HandleTypeDef gpt_Handle = {0};

static void pwm_test_pinset(T_haltest_pwm_cfg *cfg)
{
#if defined(SF32LB52X)
    HAL_PIN_Set(PAD_PA09, cfg->pad_func, PIN_PULLUP, 1); /* GPTIM2_CH1 PWM output */
#elif defined(SF32LB56X)
    HAL_PIN_Set(PAD_PA36, cfg->pad_func, PIN_PULLUP, 1); /* GPTIM2_CH1 PWM output */
#elif defined(SF32LB58X)
    HAL_PIN_Set(PAD_PA51, cfg->pad_func, PIN_PULLUP, 1); /* GPTIM1_CH2 PWM output */
#endif
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
        rt_kprintf("pwm base init failed");
        result = HAL_ERROR;
        goto __exit;
    }

    clock_config.ClockSource = GPT_CLOCKSOURCE_INTERNAL;
    if (HAL_GPT_ConfigClockSource(htim, &clock_config) != HAL_OK)
    {
        rt_kprintf("pwm clock init failed");
        result = HAL_ERROR;
        goto __exit;
    }

    if (HAL_GPT_PWM_Init(htim) != HAL_OK)
    {
        rt_kprintf("pwm init failed");
        result = HAL_ERROR;
        goto __exit;
    }

    oc_config.OCMode = GPT_OCMODE_PWM1;
    oc_config.Pulse = 0;
    oc_config.OCPolarity = GPT_OCPOLARITY_HIGH;
    oc_config.OCFastMode = GPT_OCFAST_DISABLE;

    if (HAL_GPT_PWM_ConfigChannel(htim, &oc_config, cfg->channel) != HAL_OK)
    {
        rt_kprintf("pwm config failed");
        result = HAL_ERROR;
        goto __exit;
    }

    __HAL_GPT_URS_ENABLE(htim);

__exit:
    return result;
}

static HAL_StatusTypeDef pwm_set(GPT_HandleTypeDef *htim, T_haltest_pwm_cfg *pCfg)
{
    unsigned int period, pulse;
    unsigned int GPT_clock, psc;
    unsigned int max_period;

    /* This example only uses 16-bit GPTIMs, so the 32-bit ATM period
     * range never applies. */
    max_period = MAX_PERIOD_GPT;

#if defined(SF32LB52X)
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
    rt_kprintf("Pulse %d\n", pulse);
    if (pulse < MIN_PULSE)
    {
        pulse = MIN_PULSE;
    }
    else if (pulse > period)
    {
        pulse = period;
    }
    __HAL_GPT_SET_COMPARE(htim, pCfg->channel, pulse - 1);

    /* Update frequency value */
    HAL_GPT_GenerateEvent(htim, GPT_EVENTSOURCE_UPDATE);

    return HAL_OK;
}

static uint32_t capture_clock_hz(void)
{
    return HAL_RCC_GetPCLKFreq(CAPTURE_CORE, 1);
}

static void capture_pin_set(void)
{
#if defined(SF32LB52X) || defined(SF32LB56X)
    HAL_PIN_Set(PAD_PA27, GPTIM1_CH1, PIN_NOPULL, 1);
#elif defined(SF32LB58X)
    HAL_PIN_Set(PAD_PB01, GPTIM3_CH2, PIN_NOPULL, 0);   /* PB01: LCPU pad */
#endif
}

static HAL_StatusTypeDef capture_hw_init(void)
{
    GPT_IC_InitTypeDef ic_config = {0};
    GPT_SlaveConfigTypeDef slave_cfg = {0};

    /* HAL_GPT_IC_Init does not enable the timer clock, so turn it on here
     * (52x/56x capture with GPTIM1, 58x captures with GPTIM3, a LPSYS timer). */
#if defined(SF32LB52X) || defined(SF32LB56X)
    HAL_RCC_EnableModule(RCC_MOD_GPTIM1);
#elif defined(SF32LB58X)
    HAL_RCC_EnableModule(RCC_MOD_GPTIM3);
#endif
    capture_pin_set();

    capture_Handle.Instance   = (GPT_TypeDef *)CAPTURE_INSTANCE;
    capture_Handle.core       = CAPTURE_CORE;
    capture_Handle.Channel    = CAPTURE_PERIOD_CH;
    capture_Handle.Init.Prescaler   = CAPTURE_PRESCALER;
    capture_Handle.Init.CounterMode = GPT_COUNTERMODE_UP;
    capture_Handle.Init.Period      = 0xFFFF;

    if (HAL_GPT_IC_Init(&capture_Handle) != HAL_OK)
    {
        rt_kprintf("capture init failed\n");
        return HAL_ERROR;
    }

    /* period channel: rising, direct TIx -> period CCR */
    ic_config.ICPolarity  = GPT_ICPOLARITY_RISING;
    ic_config.ICSelection = GPT_ICSELECTION_DIRECTTI;
    ic_config.ICPrescaler = GPT_ICPSC_DIV1;
    ic_config.ICFilter    = 0;
    HAL_GPT_IC_ConfigChannel(&capture_Handle, &ic_config, CAPTURE_PERIOD_CH);

    /* pulse channel: falling, indirect TIx (same pin) -> pulse CCR */
    ic_config.ICPolarity  = GPT_ICPOLARITY_FALLING;
    ic_config.ICSelection = GPT_ICSELECTION_INDIRECTTI;
    ic_config.ICPrescaler = GPT_ICPSC_DIV1;
    ic_config.ICFilter    = 0;
    HAL_GPT_IC_ConfigChannel(&capture_Handle, &ic_config, CAPTURE_PULSE_CH);

    slave_cfg.SlaveMode        = GPT_SLAVEMODE_RESET;
    slave_cfg.InputTrigger     = CAPTURE_TRIGGER;
    slave_cfg.TriggerPolarity  = GPT_TRIGGERPOLARITY_RISING;
    slave_cfg.TriggerPrescaler = GPT_TRIGGERPRESCALER_DIV1;
    slave_cfg.TriggerFilter    = 0;
    HAL_GPT_SlaveConfigSynchronization(&capture_Handle, &slave_cfg);

    return HAL_OK;
}

#if !INPUT_CAPTURE_USE_IT
static void capture_demo(void)
{
    uint32_t clk_hz;
    uint32_t period_ccr = 0, pulse_ccr = 0;
    uint32_t rise_cnt = 0;          /* rising-edge count since start */
    uint32_t period_us, pulse_us, duty, freq_hz;
    rt_tick_t start;

    rt_kprintf("Start input capture demo!\n");
    rt_kprintf("connect PWM output pin to capture input pin with a jumper wire\n");

    if (capture_hw_init() != HAL_OK)
        return;

    HAL_GPT_IC_Start(&capture_Handle, CAPTURE_PERIOD_CH);
    HAL_GPT_IC_Start(&capture_Handle, CAPTURE_PULSE_CH);

    while (1)
    {
        period_ccr = 0;
        pulse_ccr  = 0;
        rise_cnt   = 0;

        /* Wait up to 100ms (several 200Hz periods) for two rising edges.
         * A fixed busy-loop count is unreliable because it depends on the
         * CPU frequency and compiler optimization. Elapsed time is measured
         * as an unsigned tick difference, which stays correct when
         * rt_tick_get() wraps around. */
        start = rt_tick_get();
        while (rise_cnt < 2 && (rt_tick_get() - start) < rt_tick_from_millisecond(100))
        {
            if (__HAL_GPT_GET_FLAG(&capture_Handle, CAPTURE_PERIOD_FLAG))
            {
                __HAL_GPT_CLEAR_FLAG(&capture_Handle, CAPTURE_PERIOD_FLAG);
                rise_cnt++;
            }

            if (__HAL_GPT_GET_FLAG(&capture_Handle, CAPTURE_PULSE_FLAG))
                __HAL_GPT_CLEAR_FLAG(&capture_Handle, CAPTURE_PULSE_FLAG);
        }

        if (rise_cnt < 2)
        {
            rt_kprintf("no capture event, check the jumper wire!\n");
            HAL_Delay(1000);
            continue;
        }

        /* At the 2nd rising edge both CCRs belong to the cycle that just
         * ended (the pulse CCR was latched on its falling edge), so read the
         * pair back to back. Reading the pulse on its own flag would instead
         * pair a period with the pulse of the next cycle. */
        period_ccr = HAL_GPT_ReadCapturedValue(&capture_Handle, CAPTURE_PERIOD_CH);
        pulse_ccr  = HAL_GPT_ReadCapturedValue(&capture_Handle, CAPTURE_PULSE_CH);

        /* In reset slave mode the trigger edge that latches the CCR also
         * clears the counter, and that clear costs one tick, so the captured
         * value is (interval in ticks - 1): add 1 back to get the interval. */
        clk_hz = capture_clock_hz() / (CAPTURE_PRESCALER + 1);  /* after the prescaler */
        period_us = (uint32_t)((unsigned long long)(period_ccr + 1) * 1000000ULL / clk_hz);
        pulse_us  = (uint32_t)((unsigned long long)(pulse_ccr  + 1) * 1000000ULL / clk_hz);
        duty      = (uint32_t)((unsigned long long)(pulse_ccr  + 1) * 100ULL / (period_ccr + 1));
        freq_hz   = period_us ? (1000000U / period_us) : 0;

        rt_kprintf("captured: period=%u us, freq=%u Hz, pulse=%u us, duty=%u%%\n",
                   (unsigned int)period_us, (unsigned int)freq_hz,
                   (unsigned int)pulse_us, (unsigned int)duty);

        HAL_Delay(1000);
    }
}
#endif /* !INPUT_CAPTURE_USE_IT */

#if INPUT_CAPTURE_USE_IT
/* Results delivered from the ISR to the main loop (event-driven) */
static volatile uint32_t it_period_ccr = 0;
static volatile uint32_t it_pulse_ccr  = 0;
static volatile uint32_t it_rise_cnt   = 0;         /* skip the 1st rising edge (garbage) */
static struct rt_semaphore it_cap_sem;              /* released by the ISR once per period */

/* Timer ISR entry: forward to the HAL IRQ handler, which calls the capture callback */
void CAPTURE_IRQ_HANDLER(void)
{
    rt_interrupt_enter();
    HAL_GPT_IRQHandler(&capture_Handle);
    rt_interrupt_leave();
}

void HAL_GPT_IC_CaptureCallback(GPT_HandleTypeDef *htim)
{
    if (htim != &capture_Handle)
        return;

    if (htim->Channel == CAPTURE_PERIOD_ACT)        /* rising edge -> period CCR */
    {
        it_period_ccr = HAL_GPT_ReadCapturedValue(htim, CAPTURE_PERIOD_CH);
        it_rise_cnt++;
        if (it_rise_cnt >= 2)
            rt_sem_release(&it_cap_sem);    /* one event per PWM period */
    }
    else if (htim->Channel == CAPTURE_PULSE_ACT)    /* falling edge -> pulse CCR */
    {
        it_pulse_ccr = HAL_GPT_ReadCapturedValue(htim, CAPTURE_PULSE_CH);
    }
}

static void capture_demo_it(void)
{
    uint32_t clk_hz;
    uint32_t period_ccr, pulse_ccr;
    uint32_t period_us, pulse_us, duty, freq_hz;
    rt_base_t level;

    rt_kprintf("Start input capture demo (interrupt mode)!\n");
    rt_kprintf("connect PWM output pin to capture input pin with a jumper wire\n");

    rt_sem_init(&it_cap_sem, "cap_sem", 0, RT_IPC_FLAG_PRIO);

    if (capture_hw_init() != HAL_OK)
        return;

    /* 1. Enable the capture timer interrupt in the HCPU NVIC */
    HAL_NVIC_SetPriority(CAPTURE_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(CAPTURE_IRQn);

    /* 2. Start capture in interrupt mode (enables the CC interrupts) */
    HAL_GPT_IC_Start_IT(&capture_Handle, CAPTURE_PERIOD_CH);
    HAL_GPT_IC_Start_IT(&capture_Handle, CAPTURE_PULSE_CH);

    /* 3. Event-driven loop: the ISR releases the semaphore on every rising
     * edge after the initial garbage edge; the loop prints once per second
     * to avoid flooding the console (200Hz -> 200 events/s). */
    uint32_t last_print = rt_tick_get();
    while (1)
    {
        /* Wake up on each capture event. The 1s timeout is only a "no
         * signal" watchdog for the jumper-wire check; normally the ISR
         * releases the semaphore every 5ms period. */
        if (rt_sem_take(&it_cap_sem, rt_tick_from_millisecond(1000)) != RT_EOK)
        {
            rt_kprintf("no capture event, check the jumper wire!\n");
            last_print = rt_tick_get();
            continue;
        }

        /* Throttle: only print the latest result once per second. */
        if ((rt_tick_get() - last_print) < rt_tick_from_millisecond(1000))
            continue;
        last_print = rt_tick_get();

        /* Snapshot the pair atomically: the ISR updates the period CCR on
         * the rising edge and the pulse CCR on the following falling edge,
         * so an unmasked read could mix values from two adjacent cycles. */
        level = rt_hw_interrupt_disable();
        period_ccr = it_period_ccr;
        pulse_ccr  = it_pulse_ccr;
        rt_hw_interrupt_enable(level);

        /* CCR ticks -> time (us), using the capture timer clock. */
        /* In reset slave mode the trigger edge that latches the CCR also
         * clears the counter, and that clear costs one tick, so the captured
         * value is (interval in ticks - 1): add 1 back to get the interval. */
        clk_hz = capture_clock_hz() / (CAPTURE_PRESCALER + 1);  /* after the prescaler */
        period_us = (uint32_t)((unsigned long long)(period_ccr + 1) * 1000000ULL / clk_hz);
        pulse_us  = (uint32_t)((unsigned long long)(pulse_ccr  + 1) * 1000000ULL / clk_hz);
        duty      = (uint32_t)((unsigned long long)(pulse_ccr  + 1) * 100ULL / (period_ccr + 1));
        freq_hz   = period_us ? (1000000U / period_us) : 0;

        rt_kprintf("captured(IT): period=%u us, freq=%u Hz, pulse=%u us, duty=%u%%\n",
                   (unsigned int)period_us, (unsigned int)freq_hz,
                   (unsigned int)pulse_us, (unsigned int)duty);
    }
}
#endif /* INPUT_CAPTURE_USE_IT */

/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    HAL_StatusTypeDef  ret = HAL_OK;

    /* Output a message on console using printf function.
     * The PWM signal source is configured and started below. */
    rt_kprintf("Start PWM signal source!\n");

    for (int i = 0; i < sizeof(testcfg) / sizeof(T_haltest_pwm_cfg); i++)
    {
        /* clear the handle of Gptimer */
        memset(&gpt_Handle, 0, sizeof(GPT_HandleTypeDef));
        /* configure in pwm mode  */
        ret = pwm_test_init(&gpt_Handle, &testcfg[i]);

        if (ret != HAL_OK)
        {
            /* Without the PWM signal source the capture demo can never
             * measure anything, so stop here instead of continuing. */
            rt_kprintf("pwm_test_init error, abort!\n");
            return -1;
        }

        /* cal and set the pwm run para  */
        pwm_set(&gpt_Handle, &testcfg[i]);

        /* configure pinmux */
        pwm_test_pinset(&testcfg[i]);

        /* start pwm and keep it running (52x/56x: GPTIM2 CH1 PA09/PA36,
         * 58x: GPTIM1 CH2 PA51, 200Hz / 20%) so that the input capture demo
         * always has a signal to measure. */
        HAL_GPT_PWM_Start(&gpt_Handle, testcfg[i].channel);
    }

    rt_kprintf("PWM signal source setup done!\n");

    /* input capture demo: entry selected by INPUT_CAPTURE_USE_IT at the top */
#if INPUT_CAPTURE_USE_IT
    capture_demo_it();  /* never returns, measures in an endless loop */
#else
    capture_demo();     /* never returns, measures in an endless loop */
#endif

    return 0;
}
