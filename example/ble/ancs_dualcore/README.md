# BLE ancs示例

源码路径：example/ble/ancs_dualcore

(Platform_ancs)=
## 支持的平台
<!-- 支持哪些板子和芯片平台 -->
+ eh-lb55x

## 概述
<!-- 例程简介 -->
本例程演示如何触发苹果ANCS(apple notification center service)协议的订阅，以及收到对应事件的简单处理。
ANCS是苹果提供的消息协议，通过该协议可以获取当前IOS设备消息栏中收到的所有消息以及对部分特殊消息的控制。


## 例程的使用
<!-- 说明如何使用例程，比如连接哪些硬件管脚观察波形，编译和烧写可以引用相关文档。
对于rt_device的例程，还需要把本例程用到的配置开关列出来，比如PWM例程用到了PWM1，需要在onchip菜单里使能PWM1 -->
1. 例程开机会开启广播，广播名字以SIFLI_APP-xx-xx-xx-xx-xx-xx, 其中xx代表本设备的蓝牙地址。可以通过finsh命令"nvds get_mac"获取
2. 用IOS设备(iPhone或iPad)的BLE软件(LightBlue, Nrfconnect等)连接本设备，需要注意ANCS得配对才能进一步完成，所以在IOS设备弹出配对后要点击接受。
    2) 较高的ISO版本ANCS除了配对框以外，还会弹一个共享系统通知的确认框，这个也可以在IOS蓝牙设置的对应设备里面去开关。
3. 当有消息接受时，本示例会通过HCPU的log打印出来。
    1) 相关协议可以参考: [ANCS官网](https://developer.apple.com/library/archive/documentation/CoreBluetooth/Reference/AppleNotificationCenterServiceSpecification/Specification/Specification.html)


### 硬件需求
运行该例程前，需要准备：
+ 一块本例程支持的开发板（[支持的平台](#Platform_ancs)）。
+ IOS设备。

### menuconfig配置

- Platform
    - Select board peripherials->Select board: 选择运行平台，请根据实际Board选择LB55XXX系列。该选项需要在两个CPU对应选项中都选中并保证为同一个平台。
- HCPU
    - RTOS->On-chip Peripherial Drivers->Enable LCPU Patch[*]:
    - RTOS->On-chip Peripherial Drivers->Enable LCPU image[*]: 打开LCPU Patch和image以便于LCPU正常运行。
    - Sifli Middleware->Enable Data service[*]: 打开Data service，该service提供了一套数据提供者和数据订阅者的数据交互机制。
        - Enable ble nvds service[*]: 打开Data service中的NVDS数据服务。该服务会提供NVDS的异步操作以便于运行在LCPU的BLE stack/service将数据存储到HCPU的flash。
    - Third party packages->FlashDB[*]: 打开FlashDB提供访问Flash的接口，NVDS需要打开该服务
- LCPU
    - Sifli Middleware->Enable BLE service[*]： 打开BLE service，该service会提供BLE GAP/GATT/COMMON的服务
        - Enable BLE ANCS[*]: 打开BLE ANCS service，该service会提供对IOS端ANCS服务的访问，获取Notification并提供给其用户。
    - Sifli Middleware->Enable BLE stack[*]: 使能BLE协议栈。
    - Sifli Middleware->Enable Data service[*]: 打开Data service，该service提供了一套数据提供者和数据订阅者的数据交互机制。ANCS使用该机制主要为了分离数据和UI。
        - Enable ANCS service[*]: 打开Data service中的ANCS数据服务。该服务会配置及使能BLE ANCS service并对收到的通知进行处理，将处理后的数据提供给该服务的订阅者。


### 编译和烧录
切换到例程project/lcpu/xx目录，运行scons命令执行编译：
```c
> scons -j8
```
切换到例程project/hcpu/xx目录，运行scons命令执行编译：
```c
> scons -j8
```
切换到例程`project/hcpu/xx/build_xx`目录，在Jlink软件使用命令下载对应软件：
```c
loadbin project/hcpu/xx/build_xx/xx.bin addr
loadfile project/hcpu/xx/build_xx/xx.hex
```
关于编译、下载的详细步骤，请参考[快速入门](/quickstart/get-started.md)的相关介绍。

## 例程的预期结果
<!-- 说明例程运行结果，比如哪几个灯会亮，会打印哪些log，以便用户判断例程是否正常运行，运行结果可以结合代码分步骤说明 -->
例程启动后：
1. 可以被IOS上的BLE软件(例如LightBlue，Nrfconnect)等连上并配对成功。
2. IOS收到消息的时候，设备通过log显示相关信息。

## 异常诊断


## 参考文档
<!-- 对于rt_device的示例，rt-thread官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考RT-Thread的[RTC文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## 更新记录
|版本 |日期   |发布说明 |
|:---|:---|:---|
|0.0.1 |01/2025 |初始版本 |
| | | |
| | | |
