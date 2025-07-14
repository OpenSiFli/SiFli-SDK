# RTC Example
Source path: example/rt_device/rtc
## Supported Platforms
<!-- Which boards and chip platforms are supported -->
+ sf32lb52-lcd_n16r8
+ sf32lb58-lcd_n16r64n4

## Overview
<!-- Example introduction -->
This example demonstrates system time setting, system time reading, and alarm usage based on the RT device framework:
+ Set date and time, read date and time.
+ Set Alarm.

## Example Usage
<!-- Explain how to use the example, such as which hardware pins to connect to observe waveforms, compilation and flashing can reference related documentation.
For rt_device examples, you also need to list the configuration switches used by this example, such as PWM example using PWM1, which needs to be enabled in the onchip menu -->

### Hardware Requirements
Before running this example, you need to prepare a development board supported by this example

### menuconfig Configuration
The following configuration has been set up OK for this example.
1. This example is based on external 32k crystal, need to configure LXT enable (LXT_DISABLE not checked):
![LXT ENABLE](./assets/mc_lxt_enable.png)
2. Enable RTC (`BSP_USING_ONCHIP_RTC` configuration automatically configures `RT_USING_RTC`):
![RTC_USING_ONCHIP_RTC](./assets/mc_onchip_rtc_enable.png)
3. Enable RTC Alarm:
![RTC_USING_ALARM](./assets/mc_rtc_using_alarm.png)

### Compilation and Programming
Switch to the example project directory and run the scons command to execute compilation:
```
scons --board=sf32lb52-lcd_n16r8 -j32
```
Run `build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat`, select the port as prompted to download:
```
$ ./uart_download.bat

     Uart Download

please input the serial port num:5
```
For detailed steps on compilation and downloading, please refer to the relevant introduction in [](/quickstart/get-started.md).

## Expected Results
<!-- Explain the example running results, such as which LEDs will light up, what logs will be printed, so that users can judge whether the example is running normally. The running results can be explained step by step combined with the code -->
After the example starts, the serial port outputs as follows:
1. Set system time to 2024/01/01 08:30:00
```c
10-09 11:01:46:350    set system time (by RT DEVICE):  2024 01 01 08:30:00
10-09 11:01:46:352    current system time:  2024 01 01 08:30:00
```
2. Set system time to 2024/02/01 08:30:00
```c
10-09 11:01:46:354    set system time (by RTT API):  2024 02 01 08:30:00
10-09 11:01:46:356    current system time:  2024 02 01 08:30:00
```
3. Set one-shot alarm, alarm time is 08:32:00
```c
10-09 11:01:46:358    SET ONESHOT ALARM : [08:32:00] 
```
4. Alarm triggered
```c
10-09 11:03:46:301    Alarm triggered at  2024 02 01 08:32:00
```
5. Periodically get system time (every second)
```c
10-09 11:03:56:885    current system time:  2024 02 01 08:32:11
10-09 11:03:57:852    current system time:  2024 02 01 08:32:12
10-09 11:03:58:880    current system time:  2024 02 01 08:32:13
10-09 11:03:59:847    current system time:  2024 02 01 08:32:14
10-09 11:04:00:861    current system time:  2024 02 01 08:32:15
```

## Exception Diagnosis


## Reference Documentation
<!-- For rt_device examples, the RT-Thread official website documentation provides more detailed explanations, you can add webpage links here, for example, refer to RT-Thread's [RTC documentation](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->
[RTC Device](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc)

## Update Log
|Version |Date   |Release Notes |
|:---|:---|:---|
|0.0.1 |10/2024 |Initial version |
| | | |
| | | |