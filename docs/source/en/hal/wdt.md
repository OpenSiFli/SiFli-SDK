# WDT

WDT (WatchDog Timer) is an APB slave peripheral that can be used to prevent system deadlock caused by component or program conflicts in SiFli chips. Both HCPU and LCPU have dedicated WatchDog timers. It can generate interrupts as Non-Maskable Interrupts (NMI) or reset subsystems. WatchDog can independently reset LCPU or HCPU subsystems. The chip also provides a system-level Watchdog (IWDT) that can reset the entire chip.

## Working Mechanism
### Counter
WatchDog has 2 counters (count1 and count2) that count down from preset timeout values to zero. The counters are based on a 32K clock, decrementing every 1/32768 seconds. When counter count1 reaches zero, either a system reset or interrupt occurs depending on the selected output response mode. Users can restart the counter to its initial value or stop it. The process of restarting the watchdog counter is sometimes called "Pet Watchdog" or "Kick the Dog".

### Interrupt
The WatchDog can be programmed to generate an interrupt when count1 times out, while count2 starts counting. If the interrupt is not cleared, the system resets when count2 times out; if the interrupt is cleared before the second timeout (count2 reaches 0), count1 is reset, count2 is reset, and counting starts again.
```{note}
IWDT does not generate interrupts; it directly resets the chip
```

### System Reset
WatchDog can also be programmed to directly generate a system reset. The HCPU or LCPU subsystem resets after counter count1 decrements to 0.

```{note}
SF32LB55X has three WDTs: HCPU WDT1, LCPU WDT2, and one system IWDT. WDT1/2 provide interrupt functionality and can only reset HCPU/LCPU subsystems when triggered. IWDT cannot provide interrupt functionality, but can reset the entire chip when triggered. The WDT clock for 55X can be selected from RC10K/32K LXT.
      SF32LB56X/58X WDT1/WDT2 have enhanced functionality and can trigger reset of the entire chip. The clock source for SF32LB56X/58X comes from RC10K and no longer derives from the 32K low-power clock.
```

## Using WDT
The following code will start the WatchDog timer without generating interrupts.

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
    
    HAL_Delay(g_tmout+1);                        // If do not kick watchdog before g_tmout, reset will happen.
	HAL_WDT_Refresh(&WdtHandle);                  // This will not reach, as watchdog reset system.
    
    ...

    __HAL_WDT_STOP(&WdtHandle);                   // Stop watchdog timer.
    
```

The following code will start the WatchDog timer and generate an interrupt after the first counter reaches 0.
```c
    WDT_HandleTypeDef   WdtHandle;
    
    // Watchdog interrupt, It is a non-maskable interrupt
    void WDT_IRQHandler(void)
    {
        __HAL_WDT_CLEAR(&WdtHandle);              // Clear interrupt will reset timer 1.
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
    
    HAL_Delay(g_tmout1+g_tmout2+1);               // If kick after g_tmout1, interrupt will generate.
	HAL_WDT_Refresh(&WdtHandle);                  // This could reach if you kick watchdog in interrupt.
            
    ...
    
    __HAL_WDT_STOP(&WdtHandle);                   // Stop watchdog timer.
```

## API Reference
[](../api/hal/wdt.md)