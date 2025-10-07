# Ble Audio播放器（接收设备）

源码路径：example/zbt/bap_broadcast_sink

## 支持的平台
<!-- 支持哪些板子和芯片平台 -->
+ sf32lb52x

## 概述
<!-- 例程简介 -->
本例程将演示BLE Audio的Sink端，烧到52x板子上。
同时需要example/zbt/bap_broadcast_src或example/zbt/bap_broadcast_src_with_classic_bt这个工程做src端，src端只能用52j或52d等后面是字母的系列的，不用能525做为src端

## 例程的使用
<!-- 说明如何使用例程，比如连接哪些硬件管脚观察波形，编译和烧写可以引用相关文档。
对于 rt_device 的例程，还需要把本例程用到的配置开关列出来，比如 PWM 例程用到了 PWM1，需要在 onchip 菜单里使能 PWM1 -->

### 硬件需求
运行该例程前，需要准备：
+ 一块本例程支持的开发板（[支持的平台](quick_start)）。
+ 喇叭。

```{warning}

```



### 编译和烧录

切换到例程 project 目录，运行 scons 命令执行编译：

```bash
scons --board=sf32lb52-lcd_n16r8 -j8
```

切换到例程`project/build_xx`目录，运行`uart_download.bat`，按提示选择端口即可进行下载：

```bash
$ ./uart_download.bat

     Uart Download

please input the serial port num:5
```

关于编译、下载的详细步骤，请参考 [快速上手](quick_start) 的相关介绍。

## 例程的预期结果
<!-- 说明例程运行结果，比如哪几个灯会亮，会打印哪些 log，以便用户判断例程是否正常运行，运行结果可以结合代码分步骤说明 -->
例程运行之后，会连接BLE Audio的src端设备，接受音频数据播放。



## 异常诊断

## 参考文档
<!-- 对于 rt_device 的示例，rt-thread 官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考 RT-Thread 的 [RTC 文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## 更新记录
|版本 |日期   |发布说明 |
|:---|:---|:---|
|0.0.1 |6/2025 |初始版本 |
| | | |
