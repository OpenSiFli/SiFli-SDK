# RTC Example
Source code path: example/hal/rtc
## Supported Platforms
<!-- 支持哪些板子和芯片平台 -->
+ sf32lb52-lcd series
+ sf32lb56-lcd series
+ sf32lb58-lcd series

## Overview
<!-- 例程简介 -->
This example demonstrates HAL layer RTC usage, including:
+ RTC initialization configuration.
+ Setting date and time, reading date and time.
+ Setting Alarm.

## Example Usage
<!-- 说明如何使用例程，比如连接哪些硬件管脚观察波形，编译和烧写可以引用相关文档。
对于rt_device的例程，还需要把本例程用到的配置开关列出来，比如PWM例程用到了PWM1，需要在onchip菜单里使能PWM1 -->

### Hardware Requirements
Before running this example, you need to prepare a development board supported
by this example

### menuconfig Configuration
1. This example is based on external 32k crystal, need to configure LXT enable
   (LXT_DISABLE not checked, this example is already configured OK): ![LXT
   ENABLE](./assets/mc_lxt_enable.png)

### Compilation and Programming
Switch to the example project directory and run the scons command to compile:
```
scons --board=sf32lb52-lcd_n16r8 -j32
```
Run `build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat`, select the port as
prompted to download:
```
$ ./uart_download.bat

     UART Download

Please input the serial port number: 5
```
For detailed steps on compilation and downloading, please refer to the relevant
introduction in [](/quickstart/get-started.md).

## Expected Results
<!-- 说明例程运行结果，比如哪几个灯会亮，会打印哪些log，以便用户判断例程是否正常运行，运行结果可以结合代码分步骤说明 -->
After the example starts, the serial port outputs the following:
1. Initialization
```c
10-08 23:04:30:177    SFBL
10-08 23:04:32:397    RTC using LXT; RTC_CR=00000001
10-08 23:04:32:399    RTC initialization successful.
```
2. Set system time to `2025/01/01 08:30:00`
```c
10-08 23:04:32:401    Set RTC time: Wed Jan  1 08:30:00 2025
10-08 23:04:32:403    Get RTC time: Wed Jan  1 08:30:00 2025
```
3. Set Alarm, arrival time: `08:31:00`
```c
10-08 23:04:32:405    Set alarm: [8 31 0]
```
4. Alarm arrives
```c
10-08 23:05:31:464    Get RTC time: Wed Jan  1 08:30:59 2025
10-08 23:05:32:394    Alarm triggered.
10-08 23:05:32:462    Get RTC time: Wed Jan  1 08:31:00 2025
```
5. Periodically get system time (every second)
```c
10-08 23:05:34:633    Get RTC time: Wed Jan  1 08:31:02 2025
10-08 23:05:35:460    Get RTC time: Wed Jan  1 08:31:03 2025
10-08 23:05:36:473    Get RTC time: Wed Jan  1 08:31:04 2025
```

## Exception Diagnosis


## Reference Documents
<!-- 对于rt_device的示例，rt-thread官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考RT-Thread的[RTC文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Update Log
| Version | Date    | Release Notes   |
| ------- | ------- | --------------- |
| 0.0.1   | 10/2024 | Initial version |
|         |         |                 |
|         |         |                 |


