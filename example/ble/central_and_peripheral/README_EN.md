# BLE central and peripheral Example

Source code path: example/ble/central_and_peripheral

(Platform_cen_peri)=
## Supported Platforms
<!-- 支持哪些板子和芯片平台 -->
All platforms

## Overview
<!-- 例程简介 -->
This example demonstrates how to simultaneously implement GAP central and
peripheral roles as well as GATT client and server on this platform.


## Usage Instructions
<!-- 说明如何使用例程，比如连接哪些硬件管脚观察波形，编译和烧写可以引用相关文档。
对于rt_device的例程，还需要把本例程用到的配置开关列出来，比如PWM例程用到了PWM1，需要在onchip菜单里使能PWM1 -->
1. The finsh commands for this example can be printed by entering "diss help" to
   show commands and usage methods.
2. When operating as a slave device, the board starts advertising upon boot with
   the name SIFLI_APP-xx-xx-xx-xx-xx-xx, where xx represents the Bluetooth
   address of the device. It can be connected via BLE apps on mobile phones.
3. When operating as a master device, you can search for other slave devices and
   initiate connections through finsh commands.
4. When operating as a GATT server, write and read operations can be performed
   from the mobile client, or by enabling CCCD, the device will update the
   characteristic value every second.
5. When operating as a GATT client, you can search and display the server's
   database through finsh commands, and perform read or write operations on
   characteristic values.
6. When acting as a central device, other peripheral devices can be discovered
   and connected to via finsh commands.
7. When operating as a GATT server, the device supports read and write
   operations from a mobile client. Additionally, if the Client Characteristic
   Configuration Descriptor (CCCD) is enabled, the device updates characteristic
   values at one-second intervals.
8. When operating as a GATT client, finsh commands can be used to discover and
   display the server database, as well as perform read or write operations on
   characteristic values.

### Hardware Requirements
Before running this example, prepare:
+ One development board supported by this example ([Supported
  Platforms](#Platform_cen_peri)).
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
        - Description: Switch for BLE CENTRAL (central device). When enabled,
          provides scanning and active connection initiation with peripherals.
    - Enable: Enable BLE GATT client
        - Macro switch: `CONFIG_BLE_GATT_CLIENT`
        - Description: Switch for GATT CLIENT. When enabled, allows active
          service discovery, data reading/writing, and notification reception.
    - Enable: Enable BLE connection manager
        - Macro switch: `CONFIG_BSP_BLE_CONNECTION_MANAGER`
        - Description: Provides BLE connection control management, including
          multi-connection management, BLE pairing, and link connection
          parameter updates.
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
&gt; scons --board=eh-lb525 -j32
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
1. It can be discovered and connected by mobile BLE apps, allowing corresponding
   GATT characteristic value read/write operations.
2. It can search for other BLE devices, connect to them and search the connected
   device's GATT database, while performing GATT read/write operations.

## Troubleshooting


## Reference Documentation
<!-- 对于rt_device的示例，rt-thread官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考RT-Thread的[RTC文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Update History
| Version | Date    | Release Notes                               |
| ------- | ------- | ------------------------------------------- |
| 0.0.1   | 01/2025 | Initial version                             |
| 0.0.2   | 12/2025 | Extended advertising is enabled by default. |
|         |         |                                             |
