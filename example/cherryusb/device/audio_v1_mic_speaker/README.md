# USB 麦克风和扬声器示例

源码路径：example\cherryusb\device\audio_v1_mic_speaker

## 支持的平台
<!-- 支持哪些板子和芯片平台 -->
+ sf32lb52-lcd_n16r8

## 概述
<!-- 例程简介 -->
本例程演示基于usb audio class实现USB麦克风录音以及扬声器播放音频功能，包含：
+ 主机能通过USB实现连接设备实现mic录音以及扬声器播放。

## 例程的使用
<!-- 说明如何使用例程，比如连接哪些硬件管脚观察波形，编译和烧写可以引用相关文档。
对于rt_device的例程，还需要把本例程用到的配置开关列出来，比如PWM例程用到了PWM1，需要在onchip菜单里使能PWM1 -->

### 硬件需求
运行该例程前，需要准备：
+ 一块本例程支持的开发板（[支持的平台](quick_start)）。
+ 带数据传输功能的USB-A转type-c数据线。
+ 支持usb的主机设备。

### menuconfig配置

1. 使能AUDIO CODEC 和 AUDIO PROC：
![AUDIO CODEC & PROC](./assets/mc_audcodec_audprc.png)
2. 使能AUDIO(`AUDIO`)：
![AUDIO](./assets/mc_audio.png)
3. 使能AUDIO MANAGER.(`AUDIO_USING_MANAGER`)
![AUDIO_USING_MANAGER](./assets/mc_audio_manager.png)

### 编译和烧录
切换到例程project目录，运行scons命令执行编译：
```c
scons --board=sf32lb52-lcd_n16r8 -j32
```
切换到例程`project/build_xx`目录，运行`uart_download.bat`，按提示选择端口即可进行下载：
```c
$ ./uart_download.bat

     Uart Download

please input the serial port num:
```
关于编译、下载的详细步骤，请参考[快速上手](quick_start)的相关介绍。

## 例程的预期结果
<!-- 说明例程运行结果，比如哪几个灯会亮，会打印哪些log，以便用户判断例程是否正常运行，运行结果可以结合代码分步骤说明 -->
例程启动后：
主机通过数据线连接板子，主机的设备管理器的音频输入与输出一栏会出现新增麦克风设备(SiFli UAC DEMO)与扬声器设备(SiFli UAC DEMO),主机打开录音机设备后，可以选择麦克风设备正常录音,在声音输出,输出设备一栏可以选择SiFli UAC DEMO作为输出设备进行音频播放。


## 异常诊断


## 参考文档
<!-- 对于rt_device的示例，rt-thread官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考RT-Thread的[RTC文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## 更新记录
|版本 |日期   |发布说明 |
|:---|:---|:---|
|0.0.1 |09/2025 |初始版本 |
| | | |
| | | |
