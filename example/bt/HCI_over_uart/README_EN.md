# BT/BLE UART HCI Transceiver Example

Source code path: example/bt/HCI_over_uart/src


## Supported Platforms
<!-- 支持哪些板子和芯片平台 -->
+ eh-lb52x
+ eh-lb56x
+ eh-lb58x

## Overview
<!-- 例程简介 -->
This example demonstrates HCI communication with BT/BLE controller through
uart1.


## Example Usage
<!-- 说明如何使用例程，比如连接哪些硬件管脚观察波形，编译和烧写可以引用相关文档。
对于rt_device的例程，还需要把本例程用到的配置开关列出来，比如PWM例程用到了PWM1，需要在onchip菜单里使能PWM1 -->
After the example starts, HCI commands can be sent to the controller through
uart1, and HCI events will also be sent back to uart1.

The default console log port for 52x is uart3 (see project/sf32lb52x/proj.conf)

The default console log port for 56x/58x is uart4 (see
project/sf32lb56x/proj.conf) (see project/sf32lb58x/proj.conf)


### Hardware Requirements
Before running this example, you need to prepare:
+ One development board supported by this example ([Supported
  Platforms](#Platform_music_sink)).

### menuconfig Configuration

1. None

### Compilation and Flashing
Switch to the example project directory and run the scons command to compile:
```c
&gt; scons --board=eh-lb525 -j32
```
Switch to the example `project/build_xx` directory and run `uart_download.bat`,
select the port as prompted to download:
```c
$ ./uart_download.bat

     UART Download

Please input the serial port number: 5
```
For detailed compilation and download steps, please refer to the relevant
introduction in [Quick Start](/quickstart/get-started.md).

## Expected Results
<!-- 说明例程运行结果，比如哪几个灯会亮，会打印哪些log，以便用户判断例程是否正常运行，运行结果可以结合代码分步骤说明 -->
After the example starts: HCI commands can be sent to the controller through
uart1, and HCI events will also be sent back to uart1.

For example: Send hexadecimal 01 03 0C 00 (need to add carriage return and line
feed) through uart1, uart1 will receive hexadecimal 04 0E 04 06 03 0C 00

## Exception Diagnosis


## Reference Documentation
<!-- 对于rt_device的示例，rt-thread官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考RT-Thread的[RTC文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Update History
| Version | Date    | Release Notes   |
| ------- | ------- | --------------- |
| 0.0.1   | 01/2025 | Initial version |
|         |         |                 |
|         |         |                 |
