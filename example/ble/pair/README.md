# BLE pair示例

源码路径：example/ble/pair

(Platform_peri)=
## 支持的平台
<!-- 支持哪些板子和芯片平台 -->
全平台

## 概述
<!-- 例程简介 -->
本例程演示了本平台如何做GAP peripheral进行配对。


## 例程的使用
<!-- 说明如何使用例程，比如连接哪些硬件管脚观察波形，编译和烧写可以引用相关文档。
对于rt_device的例程，还需要把本例程用到的配置开关列出来，比如PWM例程用到了PWM1，需要在onchip菜单里使能PWM1 -->

本例程为 BLE Peripheral 角色，开机后自动广播并支持配对。可通过手机 BLE APP 完成配对与 GATT 读写验证。

### 操作流程
1. 上电后，设备自动开始广播。广播名格式为 `SIFLI_APP-xx-xx-xx-xx-xx-xx`。
2. 用手机 BLE APP 搜索并连接该设备。
3. 连接后发送 `cmd_diss set_sec xx xx` 命令会触发配对流程,例如这里以如下命令为例
```
cmd_diss set_sec 0 2
```
- 参数解释：0 是连接索引 conn_idx（当前连接一般是 0）。
2 是安全等级（对应 LE_SECURITY_LEVEL_NO_MITM_BOND，即不要求 MITM、但进行绑定）。

IO Cap 能力说明（配对方式与此相关）：

| io_cap 值 | 枚举 | 行为说明 | 典型表现 |
|:--:|:--|:--|:--|
| 0 | GAP_IO_CAP_DISPLAY_ONLY | 设备仅显示数字 | 手机输入 PIN；设备端打印 `SHOW PIN` |
| 1 | GAP_IO_CAP_DISPLAY_YES_NO | 设备显示数字并确认 | 手机数字确认；设备端打印 `SHOW NC` |
| 2 | GAP_IO_CAP_KB_ONLY | 设备可输入数字但不显示 | 手机显示 PIN；设备端用 `cmd_diss set_key` 输入 |
| 3 | GAP_IO_CAP_NO_INPUT_NO_OUTPUT | 无输入输出 | Just Works，一般无需输入/确认 |
| 4 | GAP_IO_CAP_KB_DISPLAY | 既能显示也能输入 | 手机根据对端策略选择 PIN/确认 |

发送命令后手机端会弹出配对请求

![](./assets/image1.png)

点击配对后会要求输入PIN码，将日志输出的PIN码填入即可完成配对

![](./assets/image2.png)

![](./assets/image3.png)

4. 连接成功后可在ble app上面进行 GATT 读写测试：
    - 读特征值：返回 4 字节（小端）数据。
    - 写特征值：支持 1~4 字节，串口日志会打印新值。
    
![](./assets/image4.png)
![](./assets/image5.png)

### 硬件需求
运行该例程前，需要准备：
+ 一块本例程支持的开发板([支持的平台](#Platform_peri))。
+ 手机设备。

### menuconfig配置
1. 使能蓝牙(`BLUETOOTH`)：
    - 路径：Sifli middleware → Bluetooth
    - 开启：Enable bluetooth
        - 宏开关：`CONFIG_BLUETOOTH`
        - 作用：使能蓝牙功能
2. 使能GAP, GATT Client, BLE connection manager：
    - 路径：Sifli middleware → Bluetooth → Bluetooth service → BLE service
    - 开启：Enable BLE GAP central role
        - 宏开关：`CONFIG_BLE_GAP_CENTRAL`
        - 作用：作为BLE CENTRAL（中心设备）的开关，打开后，提供扫描和主动发起与外设（Peripheral）的连接功能。
    - 开启：Enable BLE GATT client
        - 宏开关：`CONFIG_BLE_GATT_CLIENT`
        - 作用：GATT CLIENT的开关，打开后，可以主动搜索发现服务，读/写数据，接收通知。
    - 开启：Enable BLE connection manager
        - 宏开关：`CONFIG_BSP_BLE_CONNECTION_MANAGER`
        - 作用：提供BLE连接控制管理，包括多连接管理，BLE配对，链路连接参数更新等内容。
3. 使能NVDS：
    - 路径：Sifli middleware → Bluetooth → Bluetooth service → Common service
    - 开启：Enable NVDS synchronous
        - 宏开关：`CONFIG_BSP_BLE_NVDS_SYNC`
        - 作用：蓝牙NVDS同步。当蓝牙被配置到HCPU时，BLE NVDS可以同步访问，打开该选项；蓝牙被配置到LCPU时，需要关闭该选项。

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
1. 上电后串口输出 `receive BLE power on!`，并开始广播。
2. 手机可搜索到设备并连接，日志出现连接信息 `Peer device(xx-xx-xx-xx-xx-xx) connected`。
3. 触发配对时日志打印 `SHOW PIN` 或 `SHOW NC`，完成配对后连接保持。
4. GATT 读写测试时，打印 `Updated app value from:xx to:xx`。


## 异常诊断


## 参考文档
<!-- 对于rt_device的示例，rt-thread官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考RT-Thread的[RTC文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## 更新记录
|版本 |日期   |发布说明 |
|:---|:---|:---|
|0.0.1 |01/2025 |初始版本 |
| | | |
| | | |
