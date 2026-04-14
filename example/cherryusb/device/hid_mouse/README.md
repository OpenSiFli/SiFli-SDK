# USB 鼠标示例

源码路径：example\cherryusb\device\hid_mouse

## 支持的平台
<!-- 支持哪些板子和芯片平台 -->
+ sf32lb52-lcd_n16r8

## 概述
<!-- 例程简介 -->
本例程演示基于cherryusb的USB-HID枚举出一个鼠标设备，包含：
+ 按下sf32lb52-lcd_n16r8的KEY2按键，依次触发左键按下、右键按下、中键按下、滚轮滚动、鼠标转圈、鼠标左键按下并向右移动。

## 例程的使用
<!-- 说明如何使用例程，比如连接哪些硬件管脚观察波形，编译和烧写可以引用相关文档。
对于rt_device的例程，还需要把本例程用到的配置开关列出来，比如PWM例程用到了PWM1，需要在onchip菜单里使能PWM1 -->

### 硬件需求
运行该例程前，需要准备：
+ 一块本例程支持的开发板（[支持的平台](quick_start)）。
+ 两根带数据传输功能的USB-typec数据线，一根进行烧录，日志查看，一根负责与主机进行连接。
+ 支持鼠标输入的主机设备。

### menuconfig配置

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
主机通过数据线连接板子，主机的设备管理器的鼠标和其他指针设备一栏会出现新增鼠标设备。
按下KEY2按键，会依次触发左键按下、右键按下、中键按下、滚轮滚动、鼠标转圈、鼠标左键按下并向右移动。


## 异常诊断


## 参考文档
<!-- 对于rt_device的示例，rt-thread官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考RT-Thread的[RTC文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## 更新记录
|版本 |日期   |发布说明 |
|:---|:---|:---|
|0.0.1 |08/2025 |初始版本 |
| | | |
| | | |
