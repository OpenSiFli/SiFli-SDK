# RTC

IFLI chpset 的嵌入式 RTC（实时计数器）是一个独立的二进制编码 - 十进制（BCD）定时器计数器。 RTC 内核由计数器、预分频器、时钟分频器、闹钟数据寄存器等组成。与任何标准 RTC 芯片一样，嵌入式 RTC 可用于提供全功能的基于软件的日历以及闹钟功能。 当然还需要软件端而不是硬件端做更多的工作。 当使用 RTC 芯片时，只需要读取或写入单独的日期时间寄存器。 在 SiFli 芯片组中，我们需要做的不止这些，因为不存在单独的日期时间寄存器。

从睡眠/待机模式重置或唤醒MCU不会重新初始化RTC。 如果电池备份（VBAT）引脚有电池备份，它可以更好的保存当前日期和时间。 SiFli芯片组的所有VDD可以关闭，但是即使整个MCU核心可以完全关闭，电池备份会使RTC和备份域运行。 因此，在断电和睡眠模式下，时间不变或丢失。

SIFLI RTC的主要功能如下：
- 可编程预分频器。
- 用于长期唤醒的 18 位可编程计数器。
- 两个独立的时钟源：用于 APB2 接口的 PCLK1 和 RTC 时钟。
- 程序接口支持日期在 1970-1-1 到 2099-12-31 之间。
RTC 驱动的详细 API，请参考[RTC](#hal-rtc)。

## 使用RTC
以下代码将初始化 RTC 寄存器，并在稍后用作时间戳。

```c
    {   // Set time to Janurary  7, 2020,  16:02:15
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

以下代码将使用 RTC 进行报警服务。
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

## API参考
[](#hal-rtc)

