# BLE hid示例

源码路径：example/ble/hid

(Platform_hid)=
## 支持的平台
<!-- 支持哪些板子和芯片平台 -->
全平台

## 概述
<!-- 例程简介 -->
1. 本例程演示了本平台如何实现BLE HID。
2. 例程中实现了两类HID type，分把是HID_KEYBOARD和HID_CONSUMER。
    1) main.c默认定义了HID_CONSUMER，可以通过undefine HID_CONSUMER再define HID_KEYBOARD方式来切换编译后支持的type。 、
    ![HID_TYPE_CHOOSE](./assets/hid_type_choose.png)


## 例程的使用
<!-- 说明如何使用例程，比如连接哪些硬件管脚观察波形，编译和烧写可以引用相关文档。
对于rt_device的例程，还需要把本例程用到的配置开关列出来，比如PWM例程用到了PWM1，需要在onchip菜单里使能PWM1 -->
1. 本例程开机会开启广播，广播名字以SIFLI_APP-xx-xx-xx-xx-xx-xx, 其中xx代表本设备的蓝牙地址。可以通过手机的BLE APP进行连接
2. 手机连上本设备后，通过finsh命令"test_hids [key] [p|r]"向手机发起控制命令。其中p|r代表press|release, 需要配对使用；key根据HID type是keyboard还是consumer有不同值：
    1) Keyboard type的key取值范围：\
    ![keyboard_value](./assets/keyboard_value.png)
    2) Consumer type的key取值范围: \
    ![consumer_value](./assets/consumer_value.png)
    3) 比如再consume type下输入"test_hids 0 p"就代表输入key值对应PLAY的press操作。


### 硬件需求
运行该例程前，需要准备：
+ 一块本例程支持的开发板([支持的平台](#Platform_hid))。
+ 手机设备。

### menuconfig配置
1. 使能蓝牙(`BLUETOOTH`)：\
![BLUETOOTH](./assets/bluetooth.png)
2. 使能GAP, GATT Client, BLE connection manager：\
![BLE MIX](./assets/gap_gatt_ble_cm.png)
3. 使能NVDS：\
![NVDS](./assets/bt_nvds.png)


### 编译和烧录
切换到例程project/common目录，运行scons命令执行编译：
```c
> scons --board=eh-lb525 -j32
```
切换到例程`project/common/build_xx`目录，运行`uart_download.bat`，按提示选择端口即可进行下载：
```c
$ ./uart_download.bat

     Uart Download

please input the serial port num:5
```
关于编译、下载的详细步骤，请参考[快速入门](/quickstart/get-started.md)的相关介绍。

## 例程的预期结果
<!-- 说明例程运行结果，比如哪几个灯会亮，会打印哪些log，以便用户判断例程是否正常运行，运行结果可以结合代码分步骤说明 -->
例程启动后：
1. 可以被手机的BLE APP搜到并连上。
2. 通过finsh命令可以对手机进行控制。

## 异常诊断


## 参考文档
<!-- 对于rt_device的示例，rt-thread官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考RT-Thread的[RTC文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## 更新记录
|版本 |日期   |发布说明 |
|:---|:---|:---|
|0.0.1 |01/2025 |初始版本 |
| | | |
| | | |
