# GPT

GPT (General Purpose Timer) 基于一个16比特计数器，可实现计时、测量输入信号的脉冲长度(输入捕获)或者产生输出波形(输出比较和PWM)等功能。计数器本身可以进行向上、向下或者向上/向下计数，计数时钟为系统pclk或外部输入信号，并可进行1~65536倍的预分频。GPT共有4个channel，可以分别独立配置为输入捕获或输出模式。计数，输入捕获和输出比较的结果可以通过中断或者DMA的方式通知系统。GPT包含主从模式接口，可以进行多级级联，实现多级计数或同步触发等功能。

## GPT主要特性：
- 16位向上、向下、向上/向下自动重装载计数器，最大计数65535 <br>
- 16位可编程(可以实时修改)预分频器，计数器时钟频率的分频系数为1～65536之间的任意数值 <br>
- 8位可配置重复计数 <br>
- 支持单笔计数模式(OPM)，当重复计数完成后自动停止计数器 <br>
- 4个独立通道，可分别配置为输入或输出模式 <br>
- 输入模式 <br>
    - 上升沿/下降沿捕获 <br>
    - PWM脉宽和周期捕获(需占用两个通道) <br>
    - 可选4个输入端口之一或1个外部触发端口，支持防抖动滤波和预降频 <br>
- 输出模式 <br>
    - 强制输出高/低电平 <br>
    - 计数到比较值时输出高/低/翻转电平 <br>
    -  PWM输出，可配脉宽和周期 <br>
    - 多通道PWM组合输出，可产生有相互关系的多路PWM <br>
    - 单脉冲/重触发单脉冲模式输出 <br>
- 主从模式 <br>
    - 支持多GPT互连，可在作为主设备产生控制信号的同时，作为从设备被外部输入或其它主设备控制 <br>
    - 控制模式包括复位、触发、门控等 <br>
    - 支持多GPT同步启动、复位等 <br>
- 编码模式输入，控制计数器向上/向下计数 <br>
- 如下事件发生时产生中断/DMA： <br> 
    - 更新：计数器向上溢出/向下溢出，计数器初始化(通过软件或者内部/外部触发)<br>
    - 触发事件(计数器启动、停止、初始化或者由内部/外部触发计数)<br>
    - 输入捕获<br>
    - 输出比较<br>

SF32LB55X/56/58X的HCPU有两个GPT，分别是GPT1和GPT2，LCPU有三个GPT，分别是GPT3，GPT4和GPT5。

## GPT的接口
主要分成以下几组：
- 时钟的基本函数，包括初始化，启动，停止 <br>
- 利用时钟输出电平
- 利用时钟输出波形 (PWM)
- 利用时钟采集输出电平
- 利用时钟生成脉冲<br>
GPT的详细接口，请参考[TIM](#hal-tim)

## 使用GPT

以下是GPT基本函数的使用：
```c
	{ 	
        GPT_HandleTypeDef TIM_Handle = {0};
        
        TIM_Handle.Instance = GPTIM1;                                               // Use GPTIM1
        TIM_Handle.Init.Prescaler = HAL_RCC_GetPCLKFreq(GPTIM1_CORE, 1) / 1000 - 1; // Set prescaler
        TIM_Handle.core = GPTIM1_CORE;                                              // Clock source is from GPTIM1_CORE
        TIM_Handle.Init.CounterMode = GPT_COUNTERMODE_DOWN;                         // Count down
        TIM_Handle.Init.RepetitionCounter = 0;                                      // One shot
        HAL_GPT_Base_Init(&TIM_Handle);                                             // Initialize Timer
         
        HAL_NVIC_SetPriority(GPTIM1_IRQn, 3, 0);                                    // Set the TIMx priority
        HAL_NVIC_EnableIRQ(GPTIM1_IRQn);                                            // Enable the TIMx global Interrupt 
        
        __HAL_GPT_SET_AUTORELOAD(&TIM_Handle, 1500);                                // Set timeout counter, based on Prescaler, it is 1.5 second            
        __HAL_GPT_SET_MODE(&TIM_Handle,GPT_OPMODE_SINGLE);                          // Set timer to single mode
        HAL_GPT_Base_Start_IT(&TIM_Handle);                                         // Start timer.
        ...        
	}
	
    void GPTIM1_IRQHandler(void)
    {
        HAL_GPT_IRQHandler(&TIM_Handle);
    }
    
    void HAL_GPT_PeriodElapsedCallback(GPT_HandleTypeDef *htim)
    {
        printf("Timeout\n");
    }
```

以下是GPT PWM的使用, 在 _rtos/rtthread/bsp/drv_pwm.c_ 中还有更多的PWM的使用可以作为参考。
```c
    GPT_HandleTypeDef TIM_Handle = {0};
    GPT_OC_InitTypeDef oc_config = {0};
    GPT_ClockConfigTypeDef clock_config = {0};

    TIM_Handle.Init.Prescaler = 0;
    TIM_Handle.Init.CounterMode = GPT_COUNTERMODE_UP;
    TIM_Handle.Init.Period = 0;    
    HAL_GPT_Base_Init(&TIM_Handle);                                     // Initialize GPT handle    
    
    clock_config.ClockSource = GPT_CLOCKSOURCE_INTERNAL;    
    HAL_GPT_ConfigClockSource(&TIM_Handle, &clock_config) != HAL_OK)    // Configure the clock source	
    
    HAL_GPT_PWM_Init(&TIM_Handle);                                      // Initialize for PWM
    
    oc_config.OCMode = GPT_OCMODE_PWM1;
    oc_config.Pulse = 0;
    oc_config.OCPolarity = GPT_OCPOLARITY_HIGH;
    oc_config.OCFastMode = GPT_OCFAST_DISABLE;
    HAL_GPT_PWM_ConfigChannel(tim, &oc_config, GPT_CHANNEL_1);          // Configure PWM output to channel 1
    
    __HAL_GPT_SET_AUTORELOAD(htim, period - 1);                         // Configure duty cycle number
    HAL_GPT_GenerateEvent(htim, GPT_EVENTSOURCE_UPDATE);                // Update frequency value 
    
    HAL_GPT_PWM_Start(htim, GPT_CHANNEL_1);                             // Start PWM    
    
```

## API参考
[](#hal-tim)
