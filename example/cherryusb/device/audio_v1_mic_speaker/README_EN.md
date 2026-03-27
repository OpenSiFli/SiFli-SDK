# USB Microphone and Speaker Example

Source path: example\cherryusb\device\audio_v1_mic

## Supported Platforms
<!-- 支持哪些板子和芯片平台 -->
+ sf32lb52-lcd_n16r8

## Overview
<!-- 例程简介 -->
This example demonstrates USB microphone recording and speaker audio playback
functionality based on USB audio class, including:
+ The host can connect to devices via USB to enable microphone recording and
  speaker playback.

## Usage Instructions
<!-- 说明如何使用例程，比如连接哪些硬件管脚观察波形，编译和烧写可以引用相关文档。
对于rt_device的例程，还需要把本例程用到的配置开关列出来，比如PWM例程用到了PWM1，需要在onchip菜单里使能PWM1 -->

### Hardware Requirements
Before running this example, you need to prepare:
+ A development board supported by this example ([Supported
  Platforms](quick_start)).
+ A USB-A to Type-C data cable with data transfer capability.
+ A host device that supports USB.

### menuconfig Configuration

1. Enable AUDIO CODEC and AUDIO PROC: ![AUDIO CODEC &
   PROC](./assets/mc_audcodec_audprc.png)
2. Enable AUDIO (`AUDIO`): ![AUDIO](./assets/mc_audio.png)
3. Enable AUDIO MANAGER (`AUDIO_USING_MANAGER`):
   ![AUDIO_USING_MANAGER](./assets/mc_audio_manager.png)

### Compilation and Flashing
Switch to the example project directory and run the scons command to compile:
```c
scons --board=sf32lb52-lcd_n16r8 -j32
```
Switch to the example `project/build_xx` directory and run `uart_download.bat`,
then follow the prompts to select the port for downloading:
```c
$ ./uart_download.bat

     Uart Download

please input the serial port num:
```
For detailed steps on compilation and downloading, please refer to the relevant
introduction in [Quick Start](quick_start).

## Expected Results
<!-- 说明例程运行结果，比如哪几个灯会亮，会打印哪些log，以便用户判断例程是否正常运行，运行结果可以结合代码分步骤说明 -->
After the example starts: When the host connects to the board via the data
cable, a new microphone device (SiFli UAC DEMO) and speaker device (SiFli UAC
DEMO) will appear in the audio input and output section of the host's device
manager. After the host opens the recording device, it can select the microphone
device for normal recording. In the sound output section, you can select SiFli
UAC DEMO as the output device for audio playback.


## Troubleshooting


## Reference Documents
<!-- 对于rt_device的示例，rt-thread官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考RT-Thread的[RTC文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Change Log
| Version | Date    | Release Notes   |
| ------- | ------- | --------------- |
| 0.0.1   | 09/2025 | Initial version |
|         |         |                 |
|         |         |                 |
