
# LPTIM

The LPTIM (Low Power Timer) in this design is based on a 24-bit counter that can implement timing, output waveform generation (output compare and PWM), and system wake-up functions. The counter is an up-counter, and the counting clock can be the system's internal APB clock, low-power clock, or external input signals from the system. It can perform prescaling up to 128 times and cyclic counting up to 256 times. Output waveforms can be generated based on counting results, and it can generate interrupts to notify the CPU or generate wake-up signals to wake the system from low-power modes. When using external clocks as counting clocks, it can count and generate wake-up signals independent of internal clocks, allowing the system to shut down internal clocks.

## LPTIM Main Features:
- 24-bit up-counter with automatic reload, maximum count 16777215 (Z0 is 16-bit with maximum count 65535) <br>
- Counting clock selection<br>
    - Internal clocks including APB clock, low-power clock, etc.<br>
    - Selectable edge external input signals (IO port or comparator results), can use internal clock for debouncing filtering, or count independently without relying on internal clock<br>
- 8-level prescaling, counting clock division factor from 2^0 to 2^7<br>
- 1~256 cycle counts<br>
- Counting modes<br>
    - Continuous counting mode<br>
    - Single-shot counting mode, counting ends after cycle count completion<br>
- Configurable polarity output modes<br>
    - PWM output with configurable pulse width and period<br>
    - Single toggle output<br>
    - Single pulse or specified number of pulse outputs<br>
- Trigger modes<br>
    - Software trigger<br>
    - External input signal edge trigger with debouncing filtering support<br>
- Timeout detection, counter resets on each external trigger<br>
- Generates interrupts or wake-up signals when the following events occur: <br>
    - Update (counter overflow and cycle count ends) <br>
    - Counter overflow<br>
    - Output compare<br>
    - External trigger (only generates interrupt, no wake-up signal)<br>

SF32LB55X/56X/58X HCPU has one LPTIM (LPTIM1), LCPU has two LPTIMs (LPTIM2 and LPTIM3).

For detailed LPTIM interfaces, please refer to [LPTIM](#hal-lptim)

## Using LPTIM

Here are the basic LPTIM function usages:
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

For LPTIM PWM usage, please refer to _/rtos/rtthread/bsp/drv_pwm.c_.

## API Reference
[](#hal-lptim)
