# BLE throughput示例

源码路径：example/ble/throughput

(Platform_peri)=
## 支持的平台
<!-- 支持哪些板子和芯片平台 -->
全平台

## 概述
<!-- 例程简介 -->
本例程演示了本平台如何做GAP 的 central和peripheral以及GATT的server,client，连接，搜索服务，注册服务，作为client使用gatt write，接收notify，作为server接收gatt write，发送notify。


## 例程的使用
<!-- 说明如何使用例程，比如连接哪些硬件管脚观察波形，编译和烧写可以引用相关文档。
对于rt_device的例程，还需要把本例程用到的配置开关列出来，比如PWM例程用到了PWM1，需要在onchip菜单里使能PWM1 -->
1. 需要两个开发板均烧录本程序，
2. 烧录运行后，在其中一个开发板上输入finsh命令
diss speed_test 99227214AC03 512 0 100 65535 0
开始测速BLE吞吐率，测速结果稍后会打印在log中。
命令第一个参数是被连接开发板的地址的倒序，此时地址为03:AC:14:72:22:99
第二个参数是协商的MTU
第三个参数是连接参数，0是默认的短间隔连接参数，可以根据需要，自行添加需要测试的连接参数用于测试
第四个参数是一次测试发送的包的数量
第五个参数每一包的长度，0到65535，如果超过MTU限制，则按照最大长度即MTU-3进行填充
第六个参数是发送中对端回复的频率，比如设置成10就是发送10包回复一次，0则是不回复


### 硬件需求
运行该例程前，需要准备：
+ 二块本例程支持的开发板([支持的平台](#Platform_peri))。

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
关于编译、下载的详细步骤，请参考[快速入门](/quickstart/get-started-gcc.md)的相关介绍。

## 例程的预期结果
<!-- 说明例程运行结果，比如哪几个灯会亮，会打印哪些log，以便用户判断例程是否正常运行，运行结果可以结合代码分步骤说明 -->
例程启动后：
1. 执行finsh命令后，可以连接另一开发板并发送数据，发送结束后，会打印传输速度等信息。


## 异常诊断
1. 执行finsh命令后，连接另一块开发板后，马上断开，断开原因是62。
当刚连上后发生0x3e的断线时，重新再次进行测试即可。


## 参考文档
<!-- 对于rt_device的示例，rt-thread官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考RT-Thread的[RTC文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## 更新记录
|版本 |日期   |发布说明 |
|:---|:---|:---|
|0.0.1 |02/2025 |初始版本 |
| | | |
| | | |
