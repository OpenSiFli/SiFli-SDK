# RTC Example
Source code path: example/hal/rtc
## Supported Platforms
<!-- Which boards and chip platforms are supported -->
+ sf32lb52-lcd_n16r8
+ sf32lb58-lcd_n16r64n4
## Overview
<!-- Example introduction -->
This example demonstrates HAL layer RTC usage, including:
+ RTC initialization configuration.
+ Setting date and time, reading date and time.
+ Setting Alarm.

## Example Usage
<!-- Instructions on how to use the example, such as which hardware pins to connect to observe waveforms, compilation and programming can reference related documentation.
For rt_device examples, also need to list the configuration switches used by this example, such as PWM examples using PWM1, need to enable PWM1 in the onchip menu -->

### Hardware Requirements
Before running this example, you need to prepare a development board supported by this example

### menuconfig Configuration
1. This example is based on external 32k crystal, need to configure LXT enable (LXT_DISABLE not checked, this example is already configured OK):
![LXT ENABLE](./assets/mc_lxt_enable.png)

### Compilation and Programming
Switch to the example project directory and run the scons command to compile:
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
<!-- Describe the example execution results, such as which LEDs will light up, what logs will be printed, so users can determine if the example is running normally. Results can be explained step by step combined with code -->
After the example starts, the serial port outputs the following:
1. Initialization
```c
10-08 23:04:30:177    SFBL
10-08 23:04:32:397    RTC use LXT RTC_CR=00000001
10-08 23:04:32:399    RTC Init success.
```
2. Set system time to `2025/01/01 08:30:00`
```c
10-08 23:04:32:401    SET RTC TIME : Wed Jan  1 08:30:00 2025
10-08 23:04:32:403    GET RTC TIME : Wed Jan  1 08:30:00 2025
```
3. Set Alarm, arrival time: `08:31:00`
```c
10-08 23:04:32:405    SET ALARM : [8 31 0]
```
4. Alarm arrives
```c
10-08 23:05:31:464    GET RTC TIME : Wed Jan  1 08:30:59 2025
10-08 23:05:32:394    Alarm triggered.
10-08 23:05:32:462    GET RTC TIME : Wed Jan  1 08:31:00 2025
```
5. Periodically get system time (every second)
```c
10-08 23:05:34:633    GET RTC TIME : Wed Jan  1 08:31:02 2025
10-08 23:05:35:460    GET RTC TIME : Wed Jan  1 08:31:03 2025
10-08 23:05:36:473    GET RTC TIME : Wed Jan  1 08:31:04 2025
```

## Exception Diagnosis


## Reference Documents
<!-- For rt_device examples, the RT-Thread official website documentation provides more detailed descriptions. Web page links can be added here, for example, refer to RT-Thread's [RTC documentation](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Update Log
|Version |Date   |Release Notes |
|:---|:---|:---|
|0.0.1 |10/2024 |Initial version |
| | | |
| | | |