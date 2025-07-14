# RTC

The embedded RTC (Real-Time Counter) in SiFli chipsets is an independent Binary Coded Decimal (BCD) timer counter. The RTC core consists of counter, prescaler, clock divider, alarm data registers, etc. Like any standard RTC chip, the embedded RTC can be used to provide full-featured software-based calendar and alarm functions. Of course, more work needs to be done on the software side rather than the hardware side. When using RTC chips, you only need to read or write separate date-time registers. In SiFli chipsets, we need to do more than this because separate date-time registers do not exist.

Resetting or waking up the MCU from sleep/standby mode does not reinitialize the RTC. If there is battery backup on the battery backup (VBAT) pin, it can better preserve the current date and time. All VDD of SiFli chipsets can be turned off, but battery backup keeps the RTC and backup domain running even when the entire MCU core can be completely shut down. Therefore, time remains unchanged or is not lost during power-off and sleep modes.

Key features of SiFli RTC:
- Programmable prescaler.
- 18-bit programmable counter for long-term wake-up.
- Two independent clock sources: PCLK1 for APB2 interface and RTC clock.
- Programming interface supports dates between 1970-1-1 to 2099-12-31.
For detailed RTC driver API, please refer to [RTC](#hal-rtc).

## Using RTC
The following code will initialize RTC registers and later use them as timestamps.

```c
    {   // Set time to January 7, 2020, 16:02:15
        RTC_TimeTypeDef RTC_TimeStruct = {0};
        RTC_DateTypeDef RTC_DateStruct = {0};
        
        RTC_TimeStruct.Seconds = 15;
        RTC_TimeStruct.Minutes = 2;
        RTC_TimeStruct.Hours   = 16;
        RTC_DateStruct.Date    = 7;
        RTC_DateStruct.Month   = 1;
        RTC_DateStruct.Year    = 2020;
        RTC_DateStruct.WeekDay = 2;  //Tuesday.
        HAL_RTC_SetTime(&RTC_Handler, &RTC_TimeStruct, RTC_FORMAT_BIN);
        HAL_RTC_SetDate(&RTC_Handler, &RTC_DateStruct, RTC_FORMAT_BIN);
    }
    
    {   // Get current date and time.
        RTC_TimeTypeDef RTC_TimeStruct = {0};
        RTC_DateTypeDef RTC_DateStruct = {0};
        HAL_RTC_GetTime(&RTC_Handler, &RTC_TimeStruct, RTC_FORMAT_BIN);
        HAL_RTC_GetDate(&RTC_Handler, &RTC_DateStruct, RTC_FORMAT_BIN);
    }
```

The following code will use RTC for alarm service.
```c

    void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc) 	// Weak symbol implement the interrupt of Alarm.
    {
        printf("Alarm interrupt\n");
    }

    ...
    // Set alarm at 18:30:00
    RTC_AlarmTypeDef sAlarm;
    sAlarm.AlarmTime.Hours = 18;
    sAlarm.AlarmTime.Minutes = 30;
    sAlarm.AlarmTime.Seconds = 0;
    HAL_RTC_SetAlarm(&RTC_Handler, &sAlarm, RTC_FORMAT_BIN);
    ...
    
    // Disable alarm
    HAL_RTC_DeactivateAlarm(&RTC_Handler);
```

## API Reference
[](#hal-rtc)