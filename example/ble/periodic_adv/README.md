# BLE 周期广播示例

源码路径：example/ble/periodic_adv

(Platform_peri_adv)=
## 支持的平台
<!-- 支持哪些板子和芯片平台 -->
+ eh-lb52x
+ eh-lb56x
+ eh-lb58x

## 概述
<!-- 例程简介 -->
本例程演示周期性广播的使用方式。


## 例程的使用
<!-- 说明如何使用例程，比如连接哪些硬件管脚观察波形，编译和烧写可以引用相关文档。
对于rt_device的例程，还需要把本例程用到的配置开关列出来，比如PWM例程用到了PWM1，需要在onchip菜单里使能PWM1 -->
1. 开机后会开启周期性广播，可以参考ble_app_peri_advertising_start()的实现。默认周期广播的内容为80byte的全0数据.
2. 通过finsh命令"cmd_diss keep_per start [change_period] [len]"改变周期广播内容。其中change_period取值范围是20-255，单位是毫秒；len取值范围是0-100，单位是byte。
    1) 启动后，周期广播会按照设置的改变周期去更新内容，内容为设定长度的重复数字，该数字每次周期更新会递加，在0-255之间循环。
    2) 通过finsh命令"cmd_diss keep_per stop"停止更新周期广播的内容。需要注意，这个命令只是停止更新内容，但是不会停止周期广播。
3. 通过finsh命令"cmd_diss adv_start"和"cmd_diss adv_stop"使能和停止周期广播。


### 硬件需求
运行该例程前，需要准备：
+ 一块本例程支持的开发板([支持的平台](#Platform_peri_adv))。
+ 手机设备。

### menuconfig配置
1. 使能蓝牙(`BLUETOOTH`)：\
![BLUETOOTH](./assets/bluetooth.png)
2. 使能NVDS：\
![NVDS](./assets/bt_nvds.png)


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
1. 能够进行周期广播并能修改广播内容。


## 异常诊断


## 参考文档
<!-- 对于rt_device的示例，rt-thread官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考RT-Thread的[RTC文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## 更新记录
|版本 |日期   |发布说明 |
|:---|:---|:---|
|0.0.1 |01/2025 |初始版本 |
| | | |
| | | |
