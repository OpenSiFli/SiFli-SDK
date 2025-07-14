# RTC设备


SIFLI 芯片组的实时时钟 RTC 是一个独立的二进制编码 - 十进制 (BCD) 定时器计数器。 
RTC 内核由计数器、预分频器、时钟分频器、闹钟数据寄存器等组成。
与任何标准 RTC 芯片一样，嵌入式 RTC 可用于提供全功能的基于软件的日历以及闹钟功能。
然而，需要在软件端而不是硬件端做更多的工作。当使用 RTC 芯片时，只需要读取或写入单独的日期时间寄存器。在 SIFLI chpset 中，我们需要做的不止这些，因为不存在单独的日期时间寄存器。

从睡眠/待机模式重置或唤醒 MCU 不会重新初始化时间一旦设置。
如果备用电池 (VBAT) 引脚上有备用电池，效果会更好。 SIFLI 芯片组的所有 VDD 都可以关闭，换句话说，整个 MCU 内核可以完全关闭，但备用电池保持 RTC 和备用域运行。因此，在断电和睡眠模式期间时间不会改变或丢失。 SIFLI 嵌入式 RTC 的主要特性如下：

可编程预分频器：分频因子高达
- 用于长期唤醒的 18 位可编程计数器。
- 两个独立的时钟源：用于 APB2 接口的 PCLK1 和 RTC 时钟
- 程序接口支持日期在 1970-1-1 到 2099-12-31 之间

RTC 驱动的详细 API，请参考[RTC](#hal-rtc) 。

## 驱动配置

在{menuselection}`On-Chip Peripheral RTOS Drivers --> Enable RTC`菜单中选择要使能RTC设备。

下面的宏开关表示使能了RTC设备
```c
#define BSP_USING_ONCHIP_RTC
```

## 设备名称
- `rtc`

## 示例代码

以下代码将初始化 RTC 寄存器，并在稍后用作时间戳。

```c
{ 	// Set time to Janurary  7, 2020,  16:02:15
    set_date(2020,1,7);
    set_time(16,2,15);
}

...
{	// Get current date and time.
    time_t now;
    now = time(RT_NULL);
}
```

以下代码将使用 RTC 进行报警服务。
```c
...
rt_device_t device;
struct rt_rtc_wkalarm alm;

// Set alarm at 18:30:00
alm.enable = 1
alm.tm_hour = 18;
alm.tm_min  = 30;
alm.tm_sec  = 00;
device = rt_device_find("rtc");
rt_device_control(device, RT_DEVICE_CTRL_RTC_SET_ALARM, &alm);

...
    
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
    LOG_I("Alarm triggered");
}
    
```

[rtc]: https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc

## RT-Thread参考文档

- [RTC设备][rtc]

