# GPT

GPT (General Purpose Timer) is based on a 16-bit counter, capable of timing, measuring input signal pulse length (input capture) or generating output waveforms (output compare and PWM). The counter itself can count up, down, or up/down, with counting clock from system pclk or external input signal, and can be prescaled by 1~65536 times. GPT has 4 channels that can be independently configured as input capture or output mode. Counting, input capture and output compare results can be notified to the system through interrupts or DMA. GPT contains master-slave mode interface for multi-level cascading, implementing multi-level counting or synchronous triggering functions.

## GPT Main Features:
- 16-bit up, down, up/down auto-reload counter, maximum count 65535 <br>
- 16-bit programmable (can be modified in real-time) prescaler, counter clock frequency division factor can be any value between 1~65536 <br>
- 8-bit configurable repetition counter <br>
- Supports One Pulse Mode (OPM), automatically stops counter when repetition counting is complete <br>
- 4 independent channels, can be configured as input or output mode respectively <br>
- Input mode <br>
    - Rising/falling edge capture <br>
    - PWM pulse width and period capture (requires two channels) <br>
    - Selectable from 4 input ports or 1 external trigger port, supports anti-jitter filtering and pre-frequency division <br>
- Output mode <br>
    - Force output high/low level <br>
    - Output high/low/toggle level when count reaches compare value <br>
    - PWM output with configurable pulse width and period <br>
    - Multi-channel PWM combined output, can generate multiple PWM signals with mutual relationships <br>
    - Single pulse/retriggerable single pulse mode output <br>
- Master-slave mode <br>
    - Supports multiple GPT interconnection, can generate control signals as master device while being controlled by external input or other master devices as slave device <br>
    - Control modes include reset, trigger, gate, etc. <br>
    - Supports synchronous start, reset of multiple GPTs <br>
- Encoder mode input, controls counter up/down counting <br>
- Generate interrupt/DMA when following events occur: <br> 
    - Update: counter up overflow/down underflow, counter initialization (by software or internal/external trigger)<br>
    - Trigger event (counter start, stop, initialization or internal/external trigger counting)<br>
    - Input capture<br>
    - Output compare<br>

SF32LB55X/56/58X HCPU has two GPTs: GPT1 and GPT2, LCPU has three GPTs: GPT3, GPT4 and GPT5.

## GPT Interface
Mainly divided into the following groups:
- Clock basic functions, including initialization, start, stop <br>
- Using clock to output level
- Using clock to output waveform (PWM)
- Using clock to collect output level
- Using clock to generate pulses<br>
For detailed GPT interface, please refer to [TIM](#hal-tim)

## Using GPT

The following is the usage of GPT basic functions:
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

The following is GPT PWM usage. There are more PWM usage examples in _rtos/rtthread/bsp/drv_pwm.c_ for reference.
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

## API Reference
[](#hal-tim)