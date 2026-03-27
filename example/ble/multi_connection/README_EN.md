# BLE multi_connection Example

Source code path: example/ble/multi_connection

## Supported Platforms
<!-- 支持哪些平台 -->
All platforms

## Overview
This example demonstrates BLE multi-connection functionality based on GAP
central and peripheral roles and GATT server on this platform.

## Usage Instructions
1. When operating as a slave device, the board automatically starts advertising
   upon boot and can be connected via BLE apps on mobile phones.
2. After connection, the board automatically restarts advertising and can accept
   connections from other mobile devices.
3. It can also operate as a master device, searching for and connecting to other
   slave devices through finsh commands.

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
        - Description: Enables Bluetooth functionality
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
Switch to the example project/common directory and run the scons command to
compile:
```c
&gt; scons --board=eh-lb525 -j8
```
Switch to the example `project/common/build_xx` directory and run
`uart_download.bat`, then select the port as prompted to download:
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
1. It can be discovered and connected by multiple different mobile phones via
   BLE apps, allowing corresponding GATT characteristic value read/write
   operations.
2. It can actively connect to other devices

## Troubleshooting


## Reference Documentation
<!-- 对于rt_device的示例，rt-thread官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考RT-Thread的[RTC文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Update History
| Version | Date          | Release Notes   |
| ------- | ------------- | --------------- |
| 0.0.1   | February 2025 | Initial version |
|         |               |                 |
|         |               |                 |
