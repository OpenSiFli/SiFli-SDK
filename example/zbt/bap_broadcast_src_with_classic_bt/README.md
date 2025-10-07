# Ble Audio播放器（源设备）
源码路径：example/zbt/bap_broadcast_src_with_classic_bt

## 支持的平台
<!-- 支持哪些板子和芯片平台 -->
+ sf32lb52字母系列

## 概述
<!-- 例程简介 -->
本例程将演示BLE Audio的Src端，烧到52字母系列板子上，比如52j或52D， 不能烧到525板子上。
这个例子打开蓝牙，用手机搜索到一个Broadcast Audio Source的设备并连接，然后手机打开音频播放器播放，本地喇叭播放左声道数据，也会把右声道数据传到另一个sink设备的板子上播放。
另一个sink设备见example/zbt/bap_broadcast_sink, 需要编译sink工程并烧到另外一个设备上，
测试时也看下两个板子声音的同步情况。同一个歌词左右声道声音会不会延迟太大，明显感觉到不是同时出声音。 也需要尝试测试看看是不是放久了声音不同步了。

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
scons --board=sf32lb52-nano_52j -j8
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
本地设备是SRC，另一个设备是Sink，两个设备会分别播放左右声道数据。



## 异常诊断

## 参考文档
<!-- 对于 rt_device 的示例，rt-thread 官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考 RT-Thread 的 [RTC 文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## 更新记录
|版本 |日期   |发布说明 |
|:---|:---|:---|
|0.0.1 |6/2025 |初始版本 |
| | | |
