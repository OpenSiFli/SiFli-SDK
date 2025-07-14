# RCC

HAL_RCC (Reset and Clock Control) module can control system and peripheral clocks. It can set system or specific IP clocks to use different clocks with different frequencies. It can also issue resets to the system or specific IPs.

SF32LB52X supports dynamic frequency and voltage scaling to reduce power consumption, with corresponding interface `HAL_RCC_HCPU_ConfigHCLK`. This interface selects the appropriate voltage mode based on the frequency to be configured. When the set frequency is higher than 48MHz, DLL1 will automatically be used as the system clock. When the set frequency is lower than or equal to 48MHz, other DLLs except DLL1 need to be disabled before calling this function, and DLL1 will be automatically disabled by the function.

## Using RCC

```c
{
    #include "bf0_hal.h"
    
    ...
    HAL_RCC_HCPU_reset(HCPU_RESET_MODULES, 1);                          // Reset HCPU on-chip peripherals
    HAL_Delay(10);
    HAL_RCC_HCPU_reset(HCPU_RESET_MODULES, 0);
    
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_UART4, RCC_CLK_USART_HXT48);   // Switch UART4 to use 48M external Crystal.
    
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS,   RCC_SYSCLK_HXT48);	    // Switch system to use 48M external Crystal.
    HAL_RCC_HCPU_EnableDLL1(192000000);                                 // Enable DLL1 to use 192MHz
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS,   RCC_SYSCLK_DLL1);       // Switch system clock to DLL1

    ...
}    
```

## API Reference
[](../api/hal/rcc.md)
