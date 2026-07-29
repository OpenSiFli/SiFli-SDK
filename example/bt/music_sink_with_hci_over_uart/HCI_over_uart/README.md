# BT/BLE  uart收发HCI示例

源码路径：example/bt/music_sink_with_hci_over_uart/HCI_over_uart


## 支持的平台
<!-- 支持哪些板子和芯片平台 -->
+ eh-lb52x
+ eh-lb56x
+ eh-lb58x

## 概述
<!-- 例程简介 -->
1. 本例程演示，一块开发板作为bt/ble host，一块开发板作为bt/ble controller，通过uart2进行HCI收发
2. 本例程是作为bt/ble controller的部分


## 例程的使用
<!-- 说明如何使用例程，比如连接哪些硬件管脚观察波形，编译和烧写可以引用相关文档。
对于rt_device的例程，还需要把本例程用到的配置开关列出来，比如PWM例程用到了PWM1，需要在onchip菜单里使能PWM1 -->
1. 一块开发板烧录本工程，另一块开发板烧录music_sink_with_hci_over_uart/music_sink工程

2. 将A开发板的uart2 rx和B开发板的uart2 tx连线。

3. 将A开发板的uart2 tx和B开发板的uart2 rx连线。

4. 将两块开发板的GND连线。

5. 先reset controller开发板，串口日志看见“HCI forward rx_ind installed on uart2”说明controller板启动正常。

6. reset host开发板，例程开机会打开蓝牙的Inquiry scan和psage scan，用手机等A2DP source设备可以搜索到本机并发起连接，连上以后即可播放手机音乐，本机的蓝牙名称默认是sifli_music_sink。


### 硬件需求
运行该例程前，需要准备：
+ 两块块本例程支持的开发板（[支持的平台](#Platform_music_sink)）。

### menuconfig配置

1. 无


### 编译和烧录
切换到例程project目录，运行scons命令执行编译：
```c
> scons --board=eh-lb525 -j32
```
切换到例程`project/build_xx`目录，运行`uart_download.bat`，按提示选择端口即可进行下载：
```c
$ ./uart_download.bat

     Uart Download

please input the serial port num:5
```
关于编译、下载的详细步骤，请参考[快速入门](/quickstart/get-started.md)的相关介绍。

## 例程的预期结果
<!-- 说明例程运行结果，比如哪几个灯会亮，会打印哪些log，以便用户判断例程是否正常运行，运行结果可以结合代码分步骤说明 -->
例程启动后：
手机类A2DP source设备可以连接上本机并播放音乐


## 异常诊断


## 参考文档
<!-- 对于rt_device的示例，rt-thread官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考RT-Thread的[RTC文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## 更新记录
|版本 |日期   |发布说明 |
|:---|:---|:---|
|0.0.1 |06/2026 |初始版本 |
| | | |
| | | |
