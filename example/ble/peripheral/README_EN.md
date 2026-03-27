# BLE peripheral Example

Source code path: example/ble/peripheral

(Platform_peri)=
## Supported Platforms
<!-- 支持哪些板子和芯片平台 -->
All platforms

## Overview
<!-- 例程简介 -->
This example demonstrates how to implement GAP peripheral and GATT server
functionality on this platform.


## Usage Instructions
<!-- 说明如何使用例程，比如连接哪些硬件管脚观察波形，编译和烧写可以引用相关文档。
对于rt_device的例程，还需要把本例程用到的配置开关列出来，比如PWM例程用到了PWM1，需要在onchip菜单里使能PWM1 -->
1. When operating as a slave device, the board starts advertising upon boot with
   the name SIFLI_APP-xx-xx-xx-xx-xx-xx, where xx represents the Bluetooth
   address of the device. It can be connected via BLE apps on mobile phones.
2. When operating as a GATT server, write and read operations can be performed
   from the mobile client, or by enabling CCCD, the device will update the
   characteristic value every second.

### Hardware Requirements
Before running this example, prepare:
+ One development board supported by this example ([Supported
  Platforms](#Platform_peri)).
+ Mobile device.

### menuconfig Configuration
1. Enable Bluetooth (`BLUETOOTH`):
    - Path: Sifli middleware → Bluetooth
    - Enable: Enable bluetooth
        - Macro switch: `CONFIG_BLUETOOTH`
        - Description: Enable Bluetooth functionality
2. Enable GAP, GATT Client, BLE connection manager:
    - Path: Sifli middleware → Bluetooth → Bluetooth service → BLE service
    - Enable: Enable BLE GAP central role
        - Macro switch: `CONFIG_BLE_GAP_CENTRAL`
        - Description: Switch for BLE CENTRAL (central device). When enabled, it
          provides scanning and active connection initiation with peripherals.
    - Enable: Enable BLE GATT client
        - Macro switch: `CONFIG_BLE_GATT_CLIENT`
        - Description: Switch for GATT CLIENT. When enabled, it can actively
          search and discover services, read/write data, and receive
          notifications.
    - Enable: Enable BLE connection manager
        - Macro switch: `CONFIG_BSP_BLE_CONNECTION_MANAGER`
        - Description: Provides BLE connection control management, including
          multi-connection management, BLE pairing, link connection parameter
          updates, etc.
3. Enable NVDS:
    - Path: Sifli middleware → Bluetooth → Bluetooth service → Common service
    - Enable: Enable NVDS synchronous
        - Macro switch: `CONFIG_BSP_BLE_NVDS_SYNC`
        - Description: Bluetooth NVDS synchronization. When Bluetooth is
          configured to HCPU, BLE NVDS can be accessed synchronously, so enable
          this option; when Bluetooth is configured to LCPU, this option needs
          to be disabled.

### Compilation and Flashing
Switch to the example project directory and run the scons command to compile:
```c
&gt; scons --board=eh-lb525 -j32
```
Switch to the example `project/build_xx` directory and run `uart_download.bat`,
then select the port as prompted to download:
```c
$ ./uart_download.bat

     UART Download

Please input the serial port number: 5
```
For detailed compilation and download steps, please refer to the [Quick Start
Guide](/quickstart/get-started.md).

## Expected Results
<!-- 说明例程运行结果，比如哪几个灯会亮，会打印哪些log，以便用户判断例程是否正常运行，运行结果可以结合代码分步骤说明 -->
After the example starts:
1. It can be discovered and connected by mobile BLE apps, allowing corresponding
   GATT characteristic value read/write operations.

## Troubleshooting


## Reference Documentation
<!-- 对于rt_device的示例，rt-thread官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考RT-Thread的[RTC文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Update History
| Version | Date    | Release Notes   |
| ------- | ------- | --------------- |
| 0.0.1   | 01/2025 | Initial version |
|         |         |                 |
|         |         |                 |
