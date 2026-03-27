# USB Mass Storage Device Example (NOR FLASH)

Source Path: example\cherryusb\device\nor_flash_disk

## Supported Platforms
<!-- 支持哪些板子和芯片平台 -->
+ sf32lb52-lcd_n16r8

## Overview
<!-- 例程简介 -->
This example demonstrates implementing a virtual USB mass storage (U disk)
device using CherryUSB MSC with NOR Flash as the backend storage. It includes:
+ A USB drive named "SiFli MSC DEMO" appears in the PC file manager.

## How to Use the Example
<!-- 说明如何使用例程，比如连接哪些硬件管脚观察波形，编译和烧写可以引用相关文档。
对于rt_device的例程，还需要把本例程用到的配置开关列出来，比如PWM例程用到了PWM1，需要在onchip菜单里使能PWM1 -->

### Hardware Requirements
Prepare the following before running the example:
+ A supported development board ([Supported Platforms](quick_start)).
+ A USB-A to Type-C data cable that supports data transfer.
+ A USB-capable host (e.g., PC).

### menuconfig Configuration


### Build and Flash
In the example `project` directory, run the following `scons` command to build:

> scons --board=sf32lb52-lcd_n16r8 -j32

Then switch to the generated `project/build_xx` directory, run
`uart_download.bat`, and follow the prompt to select the serial port for
downloading:

> .\uart_download.bat

> UART Download

> Please enter the serial port number: 5

Example interaction:

## Expected Results
<!-- 说明例程运行结果，比如哪几个灯会亮，会打印哪些log，以便用户判断例程是否正常运行，运行结果可以结合代码分步骤说明 -->
After startup:

## Troubleshooting


## Reference Documents
<!-- 对于rt_device的示例，rt-thread官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考RT-Thread的[RTC文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Revision History
| Version | Date    | Release Notes   |
| ------- | ------- | --------------- |
| 0.0.1   | 09/2025 | Initial version |
|         |         |                 |
|         |         |                 |
