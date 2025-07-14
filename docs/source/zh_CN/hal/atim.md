# ATIM

ATIM (Advanced Timer) 基于一个32比特计数器，可实现计时、测量输入信号的脉冲长度(输入捕获)或者产生输出波形(输出比较和PWM)等功能，
支持3路带死区保护的PWM互补输出，支持多路PWM同时换相，并有2路刹车输入可快速将输出切换至安全状态。计数器本身可以进行向上、向下或者向上/向下计数，
计数时钟为系统pclk或外部输入信号，并可进行1~65536倍的预分频。ATIM共有6个channel，可以分别独立配置为输入捕获或输出模式。计数，输入捕获和输出比较
的结果可以通过中断或者DMA的方式通知系统。ATIM包含主从模式接口，可以进行多级级联，实现多级计数或同步触发等功能。

## ATIM主要特性：

- 32位向上、向下、向上/向下自动重装载计数器 <br>
- 16位可编程(可以实时修改)预分频器，计数器时钟频率的分频系数为1～65536之间的任意数值<br>
- 16位可配置重复计数<br>
- 支持单笔计数模式(OPM)，当重复计数完成后自动停止计数器<br>
- 6个独立通道<br>
    - 通道1~3可分别配置为输入或输出模式，其中每个通道可输出两路带死区保护的互补PWM<br>
    - 通道4可配置为输入或输出模式，可输出单路PWM<br>
    - 通道5~6可配置为输出比较模式<br>
- 输入模式<br>
    - 上升沿/下降沿捕获<br>
    - PWM脉宽和周期捕获(需占用两个通道)<br>
    - 可选4个输入端口之一或1个外部触发端口，支持防抖动滤波和预降频<br>
- 输出模式<br>
    - 强制输出高/低电平<br>
    - 计数到比较值时输出高/低/翻转电平<br>
    - PWM输出，可配脉宽和周期<br>
    - 多通道PWM组合输出，可产生有相互关系的多路PWM<br>
    - 单脉冲/重触发单脉冲模式输出<br>
- 主从模式<br>
    - 支持多GPT互连，可在作为主设备产生控制信号的同时，作为从设备被外部输入或其它主设备控制<br>
    - 控制模式包括复位、触发、门控等<br>
    - 支持多GPT同步启动、复位等<br>
- 编码模式输入，控制计数器向上/向下计数<br>
- 支持用于定位的霍尔传感器电路<br>
- 2路刹车输入，支持防抖动滤波，可将输出快速置于安全状态。刹车信号源包括：<br>
    - CPU异常<br>
    - 比较器<br>
    - 外部输入<br>
    - 软件触发<br>
- 如下事件发生时产生中断/DMA：<br>
    - 更新：计数器向上溢出/向下溢出，计数器初始化(通过软件或者内部/外部触发)<br>
    - 触发事件(计数器启动、停止、初始化或者由内部/外部触发计数)<br>
    - 输入捕获<br>
    - 输出比较<br>
    - 刹车<br>
    - 换相<br>

SF32LB58X的HCPU有两个ATIM，分别是ATIM1和ATIM2。
SF32LB56X的HCPU有一个ATIM，为ATIM1。

ATIM的详细接口，请参考[TIM_EX](#hal-tim-ex)

## 使用ATIM

以下是ATIM 的使用和GPT基本函数的使用是一样的，只是初始化的时候，将Instance置为ATIMx. 
具体使用方式请参考[](../hal/gpt.md)

### 使用atimer的HAL接口实现定时功能
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

### 使用atimer的HAL接口实现互补PWM输出功能
atimer pwm init code：
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

atime pwm param set:
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

atime pwm break dead time set:
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


## API参考
[](#hal-tim-ex)
