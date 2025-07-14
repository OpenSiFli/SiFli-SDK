# RCC

HAL_RCC（复位和时钟控制）模块可以控制系统和外围时钟。 它可以设置系统或特定IP的时钟以使用不同频率的不同时钟。 它还可以发出重置系统或特定IP。

SF32LB52X支持动态调频和调压以降低功耗，对应接口为 `HAL_RCC_HCPU_ConfigHCLK` , 该接口根据要配置的频率选择相应的电压模式，当设置的频率高于48MHz时，会自动使用DLL1
作为系统时钟，当设置的频率低于或者等于48MHz时，需要在调用该函数前关闭DLL1以外的其他DLL，DLL1则会被函数自动关闭。

## 使用RCC

```c
{
    #include "bf0_hal.h"
    
    ...
    HAL_RCC_HCPU_reset(HCPU_RESET_MODULES, 1);                          // Reset HCPU on-chip peripherals
    HAL_Delay(10);
    HAL_RCC_HCPU_reset(HCPU_RESET_MODULES, 0);
    
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_UART4, RCC_CLK_USART_HXT48);   // Switch UART4 to use 48M external Crystal.
    
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS,   RCC_SYSCLK_HXT48);	    // Switch system to use 48M external Crystal.	    HAL_RCC_HCPU_EnableDLL1(192000000);                                 // Enable DLL1 to use 192M Hz
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS,   RCC_SYSCLK_DLL1);       // Switch system clock to DLL1

    ...
}    
```


## API参考
[](../api/hal/rcc.md)
