# WDT

WDT(看门狗，WatchDog Timer)是一个APB从外设，可以用来防止由于SiFli芯片中的部件或程序冲突而导致的系统死机。 HCPU 和 LCPU 都有专用的WatchDog定时器。它可以生成中断作为不可屏蔽中断 (NMI) 或为子系统复位。WatchDog可以以独立复位 LCPU 或 HCPU 子系统。芯片还提供一个系统级别的Watchdog(IWDT)，可以复位整个芯片。

## 工作机制
### 计数器
WatchDog有 2 个计数（count1 和 count2），从预设的超时值降序到零。计数器基于 32K 时钟，每 1/32768 秒减一。 1。当计数器 count1 达到零时，根据选择的输出响应模式，系统复位或中断发生。用户可以重新启动计数器到其初始值，或停止它。重新启动看门狗计数器的过程有时被称为喂狗（Pet Watchdog）。

### 中断
可以对WatchDog进行编程,使watchdog在count1 超时时产生中断, 同时count2开始计数。如果中断不被清理，在count2 超时的时候系统复位；如果它在第二次超时（count2 达到 0）之前清除中断，count1被复位，同时count2复位，重新开始计数。
```{note}
IWDT不会产生中断，直接复位芯片
```

### 系统复位
WatchDog也可以被编程为直接产生系统复位。 HCPU 或 LCPU 子系统在计数器count1减为 0后复位。

```{note}
SF32LB55X 有三个WDT, HCPU WDT1, LCPU WDT2以及一个系统IWDT. 其中WDT1/2提供中断功能，触发时只能reset HCPU/LCPU子系统。 IWDT不能提供中断功能，不过触发时可以reset整个芯片。55X的WDT时钟可以在RC10K/32K LXT中选择。
      SF32LB56X/58X WDT1/WDT2 增加了功能，可以触发reset 整个芯片，SF32LB56X/58X的时钟来源于RC10K，不再从32K低功耗时钟获取。
```

## 使用WDT
以下代码将启动WatchDog定时器而不产生中断。

```c
    WDT_HandleTypeDef   WdtHandle;
    
    // Initialize Watchdog timer.
	{ 	
        WdtHandle.Init.Reload = (uint32_t)g_tmout * 32768 ;   // Counter is based on 32K clock, g_tmout is in seconds
        HAL_WDT_Init(&WdtHandle);                 // Initialize the counter  
        __HAL_WDT_INT(&WdtHandle, 0);             // Set to reset only, do not generate interrupt
	}
    __HAL_WDT_START(&WdtHandle);                  // Start watchdog timer.
	
	...
    
	HAL_WDT_Refresh(&WdtHandle);                  // Kick watchdog
    
    HAL_Delay(g_tmout+1);                        // If do not kick watchdog befre g_tmout, reset will happen.
	HAL_WDT_Refresh(&WdtHandle);                  // This will not reach, as watchdog reset system.
    
    ...

    __HAL_WDT_STOP(&WdtHandle);                   // Stop watchdog timer.
    
```

以下代码将启动WatchDog定时器并在第一个计数器达到 0 后产生中断。
```c
    WDT_HandleTypeDef   WdtHandle;
    
    // Watchdog interrupt, It is a non-maskable interrupt
    void WDT_IRQHandler(void)
    {
        __HAL_WDT_CLEAR(&WdtHandle);              //cLear interrupt will reset timer 1.
    }

    // Initialize Watchdog timer.
	{ 	
        WdtHandle.Init.Reload = (uint32_t)g_tmout1 * 32768 ;   // Counter 1, counter is based on 32K clock, g_tmout1 is in seconds.
        WdtHandle.Init.Reload2 = (uint32_t)g_tmout2 * 32768;   // Counter 2, counter is based on 32K clock, g_tmout2 is in seconds.
        HAL_WDT_Init(&WdtHandle);                 // Initialize the counter  
        __HAL_WDT_INT(&WdtHandle, 1);             // Generate interrupt when g_tmout1 times out before you kick interrupt
	}
    __HAL_WDT_START(&WdtHandle);                  // Start watchdog timer.
	
	...
    
	HAL_WDT_Refresh(&WdtHandle);                  // If kick before g_tmout1, it will NOT generate interrupt
    
    HAL_Delay(g_tmout1+g_tmout2+1);               // If kick after g_tmout1,  interrupt will generate.
	HAL_WDT_Refresh(&WdtHandle);                  // This could reach if you kick watchdog in interrupt.
            
    ...
    
    __HAL_WDT_STOP(&WdtHandle);                   // Stop watchdog timer.
```



## API参考
[](../api/hal/wdt.md)

