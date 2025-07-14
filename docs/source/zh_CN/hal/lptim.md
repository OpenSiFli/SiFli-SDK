# LPTIM

本设计中的LPTIM (Low Power Timer) 基于一个24比特计数器，可实现计时、产生输出波形(输出比较和PWM)和唤醒系统等功能。计数器为向上计数，计数时钟可以为系统内部的APB时钟或低功耗时钟，或系统外部的输入信号，并可进行最多128倍的预分频以及最多256次的循环计数。根据计数结果可以产生输出波形，并可产生中断通知CPU，或产生唤醒信号将系统从低功耗模式唤醒。当用外部时钟作为计数时钟时，可不依赖于内部时钟进行计数并产生唤醒信号，从而允许系统关闭内部时钟。

## LPTIM主要特性：
- 24位向上自动重装载计数器，最大计数16777215 (Z0 是16位的，最大计数65535) <br>
- 计数时钟选择<br>
    - 内部时钟，包括APB时钟，低功耗时钟等<br>
    - 可选边沿的外部输入信号(IO口或比较器结果)，可利用内部时钟进行防抖动滤波，也可不依赖内部时钟独立计数<br>
- 8档预分频，计数时钟分频系数为2的0~7次方<br>
- 1~256循环次数<br>
- 计数模式<br>
    - 连续计数模式<br>
    - 单笔计数模式，循环次数完成后计数结束<br>
- 可配极性的输出模式<br>
    - PWM输出，可配脉宽，周期<br>
    - 单次翻转输出<br>
    - 单脉冲或指定个数脉冲输出<br>
- 触发模式<br>
    - 软件触发<br>
    - 外部输入信号边沿触发，支持防抖动滤波<br>
- 超时检测，每次外部触发时计数器复位<br>
- 如下事件发生时产生中断或唤醒信号： <br>
    - 更新(计数器溢出且循环次数结束) <br>
    - 计数器溢出<br>
    - 输出比较<br>
    - 外部触发(只产生中断，无唤醒信号)<br>

SF32LB55X/56X/58X的HCPU有一个LPTIM，LPTIM1，LCPU有两个LPTIM，分别是LPTIM2和LPTIM3。

LPTIM的详细接口，请参考[LPTIM](#hal-lptim)

## 使用LPTIM

以下是LPTIM基本函数的使用：
```c
    LPTIM_HandleTypeDef TIM_Handle = {0};
        
	{ 	
        ...
        HAL_LPTIM_InitDefault(&TIM_Handle);                                         // Set default setting for LPTIM
        TIM_Handle.Instance=LPTIM1;                                                 // Use LPTIM1
        HAL_LPTIM_Init(&TIM_Handle);                                                // Initialize Timer

        HAL_NVIC_SetPriority(LPTIM1_IRQn, 3, 0);                                    // Set the TIMx priority
        HAL_NVIC_EnableIRQ(LPTIM1_IRQn);                                            // nable the TIMx global Interrupt 
        
        __HAL_LPTIM_CLEAR_PRESCALER(tim, LPTIM_PRESCALER_DIV128);
        __HAL_LPTIM_SET_PRESCALER(tim, LPTIM_PRESCALER_DIV1);                       // Set prescale
        TIM_Handle.Mode = HAL_LPTIM_ONESHOT;                                        // One shot timer
        HAL_LPTIM_Counter_Start_IT(&TIM_Handle, 1000);                              // Start timer to count 1000 from low power crystal source
        ...        
	}
	
    void LPTIM1_IRQHandler(void)
    {
        HAL_LPTIM_IRQHandler(&TIM_Handle);
    }
    
    void HAL_LPTIM_IRQHandler(LPTIM_HandleTypeDef *htim)
    {
        printf("Timeout\n");
    }
```

LPTIM PWM的使用请参考在 _/rtos/rtthread/bsp/drv_pwm.c_ .

## API参考
[](#hal-lptim)
