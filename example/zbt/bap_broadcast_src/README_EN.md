# MP3 Local Music Player

Source path: example/zbt/bap_broadcast_src

## Supported Platforms
<!-- 支持哪些板子和芯片平台 -->
+ sf32lb52 letter series

## Overview
<!-- 例程简介 -->
This example demonstrates the BLE Audio Source side, flashed to sf32lb52 letter
series boards, such as 52j or 52D. It cannot be flashed to 525 boards. This
example will play mp3 files from the disk directory. The local speaker will not
sound, but will transmit audio to another sink device board for playback.
Another sink device can be found at example/zbt/bap_broadcast_sink. You need to
compile the sink project and flash it to another device.


## Example Usage
<!-- 说明如何使用例程，比如连接哪些硬件管脚观察波形，编译和烧写可以引用相关文档。
对于 rt_device 的例程，还需要把本例程用到的配置开关列出来，比如 PWM 例程用到了 PWM1，需要在 onchip 菜单里使能 PWM1 -->

### Hardware Requirements
Before running this example, you need to prepare:
+ One development board supported by this example ([Supported
  platforms](quick_start)).
+ Speaker.

```{warning}

```



### Compilation and Programming

Switch to the example project directory and run the scons command to execute
compilation:

```bash
scons --board=sf32lb52-nano_52j -j8
```

Switch to the example `project/build_xx` directory and run `uart_download.bat`,
select the port as prompted to download:

```bash
$ ./uart_download.bat

     UART Download

Please input the serial port number: 5
```

For detailed steps on compilation and downloading, please refer to the relevant
introduction in [Quick Start](quick_start).

## Expected Results
<!-- 说明例程运行结果，比如哪几个灯会亮，会打印哪些 log，以便用户判断例程是否正常运行，运行结果可以结合代码分步骤说明 -->
After the example runs, it will decode mp3 files from the disk directory and
transmit them via BLE to connected BLE Audio sink devices for playback. The
local source device speaker will not play.



## Exception Diagnosis

## Reference Documentation
<!-- 对于 rt_device 的示例，rt-thread 官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考 RT-Thread 的 [RTC 文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Update Log
| Version | Date      | Release Notes   |
| ------- | --------- | --------------- |
| 0.0.1   | June 2025 | Initial version |
|         |           |                 |
