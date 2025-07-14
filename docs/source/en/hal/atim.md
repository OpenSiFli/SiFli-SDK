# ATIM

ATIM (Advanced Timer) is based on a 32-bit counter, capable of timing, measuring input signal pulse length (input capture) or generating output waveforms (output compare and PWM). It supports 3 channels of complementary PWM output with dead-time protection, supports multi-channel PWM phase switching, and has 2 brake inputs for quickly switching output to safe state. The counter itself can count up, down, or up/down, with counting clock from system pclk or external input signal, and can be prescaled by 1~65536 times. ATIM has 6 channels that can be independently configured as input capture or output mode. Counting, input capture and output compare results can be notified to the system through interrupts or DMA. ATIM contains master-slave mode interface for multi-level cascading, implementing multi-level counting or synchronous triggering functions.

## ATIM Main Features:

- 32-bit up, down, up/down auto-reload counter <br>
- 16-bit programmable (can be modified in real-time) prescaler, counter clock frequency division factor can be any value between 1~65536<br>
- 16-bit configurable repetition counter<br>
- Supports One Pulse Mode (OPM), automatically stops counter when repetition counting is complete<br>
- 6 independent channels<br>
    - Channels 1~3 can be configured as input or output mode, each channel can output two complementary PWM signals with dead-time protection<br>
    - Channel 4 can be configured as input or output mode, can output single PWM<br>
    - Channels 5~6 can be configured as output compare mode<br>
- Input mode<br>
    - Rising/falling edge capture<br>
    - PWM pulse width and period capture (requires two channels)<br>
    - Selectable from 4 input ports or 1 external trigger port, supports anti-jitter filtering and pre-frequency division<br>
- Output mode<br>
    - Force output high/low level<br>
    - Output high/low/toggle level when count reaches compare value<br>
    - PWM output with configurable pulse width and period<br>
    - Multi-channel PWM combined output, can generate multiple PWM signals with mutual relationships<br>
    - Single pulse/retriggerable single pulse mode output<br>
- Master-slave mode<br>
    - Supports multiple GPT interconnection, can generate control signals as master device while being controlled by external input or other master devices as slave device<br>
    - Control modes include reset, trigger, gate, etc.<br>
    - Supports synchronous start, reset of multiple GPTs<br>
- Encoder mode input, controls counter up/down counting<br>
- Supports Hall sensor circuit for positioning<br>
- 2 brake inputs, supports anti-jitter filtering, can quickly put output in safe state. Brake signal sources include:<br>
    - CPU exception<br>
    - Comparator<br>
    - External input<br>
    - Software trigger<br>
- Generate interrupt/DMA when following events occur:<br>
    - Update: counter up overflow/down underflow, counter initialization (by software or internal/external trigger)<br>
    - Trigger event (counter start, stop, initialization or internal/external trigger counting)<br>
    - Input capture<br>
    - Output compare<br>
    - Brake<br>
    - Commutation<br>

SF32LB58X HCPU has two ATIMs: ATIM1 and ATIM2.
SF32LB56X HCPU has one ATIM: ATIM1.

For detailed ATIM interface, please refer to [TIM_EX](#hal-tim-ex)

## Using ATIM

The following ATIM usage is the same as GPT basic function usage, except setting Instance to ATIMx during initialization.
For specific usage, please refer to [](../hal/gpt.md)

### Using ATIM HAL Interface for Timer Function
```c
{
    #define FREQENCY (10000)
    #define TIME_MS  (3500)
    static GPT_HandleTypeDef TIM_Handle = {0};

    TIM_Handle.Instance = (GPT_TypeDef *)hwp_atim1;
    TIM_Handle.Init.Prescaler = HAL_RCC_GetPCLKFreq(CORE_ID_HCPU, 1) / FREQENCY; /*Prescaler is 16 bits, please select correct frequency*/
    TIM_Handle.core = CORE_ID_HCPU;
    TIM_Handle.Init.CounterMode = GPT_COUNTERMODE_UP; /*GPTIM could support counter up/down, BTIM only support count up*/
    TIM_Handle.Init.RepetitionCounter = 0;
    TIM_Handle.Init.Period = TIME_MS * FREQENCY / 1000;
    if (HAL_GPT_Base_Init(&TIM_Handle) == HAL_OK) /*init timer*/
    {
        HAL_NVIC_SetPriority(ATIM1_IRQn, 3, 0); /* set the TIMx priority */
        HAL_NVIC_EnableIRQ(ATIM1_IRQn); /* enable the TIMx global Interrupt */
        __HAL_GPT_CLEAR_FLAG(&TIM_Handle, GPT_FLAG_UPDATE); /* clear update flag */
        __HAL_GPT_URS_ENABLE(&TIM_Handle); /* enable update request source */
    }
    else
    {
        LOG_E("Timer init error");
        return;
    }
    if (HAL_GPT_Base_Start_IT(&TIM_Handle) != HAL_OK) /* start timer */
    {
        LOG_E("Timer start error");
        return;
    }

    /*atimer interrupt handler*/
    void ATIM1_IRQHandler(void)
    {
        ENTER_INTERRUPT();
        HAL_GPT_IRQHandler(&TIM_Handle);
        LEAVE_INTERRUPT();
    }
}
```

### Using ATIM HAL Interface for Complementary PWM Output
ATIM PWM init code:
```c
{
    GPT_HandleTypeDef gpt_Handle = {0};
    GPT_OC_InitTypeDef oc_config = {0};
    GPT_ClockConfigTypeDef clock_config = {0};

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
}
```

ATIM PWM parameter set:
```c
{
    #define PWM_PERIOD  (500000000) //ns
    #define PWM_PULSE   (250000000) //ns
    #define MAX_PERIOD_ATM (0xFFFFFFFF) //32bit

    rt_uint32_t period, pulse;
    rt_uint32_t GPT_clock, psc;

    GPT_clock = HAL_RCC_GetPCLKFreq(CORE_ID_HCPU, 1);
    /* Convert nanosecond to frequency and duty cycle. 1s = 1 * 1000 * 1000 * 1000 ns */
    GPT_clock /= 1000000UL;
    period = (unsigned long long)PWM_PERIOD * GPT_clock / 1000ULL;
    psc = period / MAX_PERIOD_ATM + 1;
    period = period / psc;
    /*set atimer prescaler*/
    __HAL_GPT_SET_PRESCALER(&gpt_Handle, psc - 1);
    /*set atimer auto reload*/
    __HAL_GPT_SET_AUTORELOAD(&gpt_Handle, period - 1);
    /*set atimer pulse*/
    pulse = (unsigned long long)PWM_PULSE * GPT_clock / psc / 1000ULL;
    __HAL_GPT_SET_COMPARE(htim, GPT_CHANNEL_1, pulse - 1);

    HAL_GPT_GenerateEvent(htim, GPT_EVENTSOURCE_UPDATE);
}
```

ATIM PWM brake dead time set:
```c
{
    TIMEx_BreakDeadTimeConfigTypeDef bdt = {0};

    bdt.AutomaticOutput = 0;
    bdt.BreakFilter = 0;
    bdt.BreakPolarity = 0;
    bdt.BreakState = 0;
    bdt.Break2Filter = 0;
    bdt.Break2Polarity = 0;
    bdt.Break2State = 0;
    bdt.DeadTime = 200; /*0~1023*/
    bdt.OffStateIDLEMode = 0;
    bdt.OffStateRunMode = 0;
    bdt.DeadTimePsc = 0;

    HAL_TIMEx_ConfigBreakDeadTime(htim, &bdt);
}
```

## API Reference
[](#hal-tim-ex)
