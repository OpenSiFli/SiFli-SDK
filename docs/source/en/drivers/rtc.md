# RTC Device

The Real-Time Clock (RTC) of SIFLI chipset is an independent binary coded decimal (BCD) timer/counter. 
The RTC core consists of counter, prescaler, clock divider, alarm data registers, etc.
Like any standard RTC chip, the embedded RTC can be used to provide full-featured software-based calendar as well as alarm functionality.
However, more work needs to be done on the software side rather than hardware side. When using RTC chips, you only need to read or write separate date-time registers. In SIFLI chipset, we need to do more than this because there are no separate date-time registers.

Resetting or waking up the MCU from sleep/standby mode does not reinitialize the time once set.
The effect is better if there is a backup battery on the backup battery (VBAT) pin. All VDD of SIFLI chipset can be turned off, in other words, the entire MCU core can be completely shut down, but the backup battery keeps the RTC and backup domain running. Therefore, time does not change or get lost during power-off and sleep modes. The main features of SIFLI embedded RTC are as follows:

Programmable prescaler: division factor up to
- 18-bit programmable counter for long-term wakeup.
- Two independent clock sources: PCLK1 for APB2 interface and RTC clock
- Program interface supports dates between 1970-1-1 to 2099-12-31

For detailed API of RTC driver, please refer to [RTC](#hal-rtc).

## Driver Configuration

Select to enable RTC device in the {menuselection}`On-Chip Peripheral RTOS Drivers --> Enable RTC` menu.

The following macro switch indicates that RTC device is enabled:
```c
#define BSP_USING_ONCHIP_RTC
```

## Device Names
- `rtc`

## Example Code

The following code will initialize RTC registers and use them as timestamps later.

```c
{ 	// Set time to January 7, 2020, 16:02:15
    set_date(2020,1,7);
    set_time(16,2,15);
}

...
{	// Get current date and time.
    time_t now;
    now = time(RT_NULL);
}
```

The following code will use RTC for alarm service.
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

## RT-Thread Reference Documentation

- [RTC Device][rtc]
