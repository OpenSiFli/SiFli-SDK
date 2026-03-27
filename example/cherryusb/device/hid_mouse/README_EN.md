# USB Mouse Example

Source code path: example\cherryusb\device\hid_mouse

## Supported Platforms
<!-- 支持哪些板子和芯片平台 -->
+ sf32lb52-lcd_n16r8

## Overview
<!-- 例程简介 -->
This example demonstrates enumerating a mouse device based on cherryusb USB-HID,
including:
+ Press the KEY2 button on sf32lb52-lcd_n16r8 to sequentially trigger left
  button press, right button press, middle button press, wheel scroll, mouse
  circle, left button press and move right.

## Usage of the Example
<!-- 说明如何使用例程，比如连接哪些硬件管脚观察波形，编译和烧写可以引用相关文档。
对于rt_device的例程，还需要把本例程用到的配置开关列出来，比如PWM例程用到了PWM1，需要在onchip菜单里使能PWM1 -->

### Hardware Requirements
Before running this example, prepare:
+ A development board supported by this example ([Supported
  Platforms](quick_start)).
+ Two USB-typec data cables with data transmission capability, one for burning
  and log viewing, one for connecting to the host.
+ A host device that supports mouse input.

### menuconfig Configuration

### Compilation and Burning
Switch to the example project directory, run the scons command to compile:
```c
scons --board=sf32lb52-lcd_n16r8 -j32
```
Switch to the example `project/build_xx` directory, run `uart_download.bat`, and
select the port as prompted to download:
```c
$ ./uart_download.bat

     Uart Download

please input the serial port num:
```
For detailed steps on compilation and download, please refer to the introduction
in [Quick Start](quick_start).

## Expected Results of the Example
<!-- 说明例程运行结果，比如哪几个灯会亮，会打印哪些log，以便用户判断例程是否正常运行，运行结果可以结合代码分步骤说明 -->
After the example starts: The host connects to the board via data cable, and a
new mouse device will appear in the Mouse and other pointing devices section of
the host's device manager. Press the KEY2 button to sequentially trigger left
button press, right button press, middle button press, wheel scroll, mouse
circle, left button press and move right.


## Exception Diagnosis


## Reference Documents
<!-- 对于rt_device的示例，rt-thread官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考RT-Thread的[RTC文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Update History
| Version | Date    | Release Notes   |
| ------- | ------- | --------------- |
| 0.0.1   | 08/2025 | Initial version |
|         |         |                 |
|         |         |                 |
