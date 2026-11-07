# BLE file transfer示例

源码路径：example/ble/file_transfer

(Platform_file_rec)=
## 支持的平台
<!-- 支持哪些板子和芯片平台 -->
全平台

## 概述
<!-- 例程简介 -->
本例程演示了本平台如何做GAP peripheral然后接收手机端发送的文件

## 例程的使用
<!-- 说明如何使用例程，比如连接哪些硬件管脚观察波形，编译和烧写可以引用相关文档。
对于rt_device的例程，还需要把本例程用到的配置开关列出来，比如PWM例程用到了PWM1，需要在onchip菜单里使能PWM1 -->
1. 本例程通过 `MSH_CMD_EXPORT(cmd_diss, ...)` 注册命令，命令名为 `cmd_diss`（没有 help 分支）。
2. 作为从设备时开机会开启广播，广播名字以SIFLI_APP-xx-xx-xx-xx-xx-xx, 其中xx代表本设备的蓝牙地址。可以通过手机的BLE APP进行连接
3. 可以使用sifli ble serial transport串行传输协议，从手机端发送文件到设备端，一次可以发送一个或者多个文件。


### 硬件需求
运行该例程前，需要准备：
+ 一块本例程支持的开发板（[支持的平台](#Platform_file_rec)）。
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
        - 作用：蓝牙NVDS同步。当蓝牙被配置到HCPU时，BLE NVDS可以同步访问，打开该选项；蓝牙被配置到LCPU时，需要关闭该选项
4. 打开传输相关的宏：
    - 路径：Sifli middleware → Bluetooth → Bluetooth service → BLE service
    - 开启：Enable BLE serial transmission
        - 宏开关：`CONFIG_BSP_BLE_SERIAL_TRANSMISSION`
        - 作用：串行传输协议，打开后可以提供一个简易的串行传输协议，用户可以根据该协议，通过BLE传输一些内容到开发板
    - 开启：Enable watch face downloaded via BLE
        - 宏开关：`CONFIG_BSP_BLE_WATCH_FACE`
        - 作用：表盘传输协议。依赖serial transmission，启用后，可以通过封装好的手机SDK，使用BLE传输文件到开发板。

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
1. 可以被手机BLE APP搜到并连接。
2. 将需要传输的一个或者多个文件，使用zip格式进行压缩，压缩后只有一个.zip文件，将该文件传入手机
3. 使用SIFLI BLE APP搜索，连接该设备，然后再watchface界面，选择自定义文件，然后选择该文件，该文件就可以传输到开发板的文件系统中
4. 手机使用步骤 :
- 开启位置允许使用权限
![APP1](./assets/app0.png)
- 点击右下角查询按钮进入到扫描蓝牙界面
![APP1](./assets/app1.png)
- 点击SCAN进行扫描
![APP2](./assets/app2.png)
- 搜索到设备后，点击SORT,会将广播名为SIFLI_APP-xx-xx-xx-xx-xx-xx放置前排
![APP3](./assets/app3.png)
- 点击SIFLI_APP-xx-xx-xx-xx-xx-xx广播名进入设备界面
![APP4](./assets/app4.png)
- 点击右上角进入操作界面
![APP4](./assets/app5.png)
- 点击CONNECT确保State 为CONNECT
![APP4](./assets/app6.png)
- 上滑最后一栏框，找到文件传输 WFPUSH2点击进入文件传输界面
![APP4](./assets/app7.png)
- 点击SELECT FILE选择手机里面事先准备好的zip文件，并把File Type设置为3, 勾选With Byte Align,最后点击START
![APP4](./assets/app8.png)

- 传输完成后，在console窗口执行命令进入file_recv文件夹，并执行ls命令查看文件列表,其中就是zip文件内容的显示
![APP4](./assets/app9.png)


## 手机APP和DEMO工程获取使用
### Android sifli ble app下载地址
https://www.pgyer.com/gurSBc

### Android demo工程
https://github.com/OpenSiFli/SiFli_OTA_APP\
对应的部分在"3. SiFli-SDK OTA"

### iOS demo工程
https://github.com/OpenSiFli/SiFli_OTA_APP_IOS\
对应的部分在"SiFli-SDK OTA (Nor Offline)"
SiFli-SDK file transfer

## 异常诊断
当开发板回复某个异常错误时，检查bf0_sibles_watchface.h中的ble_watchface_status_id_t

## 参考文档
<!-- 对于rt_device的示例，rt-thread官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考RT-Thread的[RTC文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## 更新记录
|版本 |日期   |发布说明 |
|:---|:---|:---|
|0.0.1 |07/2025 |初始版本 |
| | | |
| | | |
