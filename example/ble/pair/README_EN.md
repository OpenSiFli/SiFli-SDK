# BLE Peripheral Example

Source path: example/ble/peripheral

(Platform_peri)=
## Supported Platforms
<!-- 支持哪些板子和芯片平台 -->
All platforms

## Overview
<!-- 例程简介 -->
This example demonstrates how this platform implements GAP peripheral and GATT
server functionalities.


## Usage
<!-- 说明如何使用例程，比如连接哪些硬件管脚观察波形，编译和烧写可以引用相关文档。
对于rt_device的例程，还需要把本例程用到的配置开关列出来，比如PWM例程用到了PWM1，需要在onchip菜单里使能PWM1 -->
1. As a slave device, advertising will be enabled at startup with the
   advertising name SIFLI_APP-xx-xx-xx-xx-xx-xx, where xx represents the
   Bluetooth address of this device. Connection can be established through a
   mobile BLE APP.
2. As a GATT server, write and read operations can be performed from the mobile
   device, or CCCD can be enabled, and the device will update the characteristic
   value every second.
3. In the initialization stage, bond_ack is set to BOND.Pending by default. You
   need to pay attention to CONNECTION_MANAGER_BOND_AUTH_INFOR and call
   connectionManager_bond_ack_reply in time.
```c
connection_manager_set_bond_ack(BOND_PENDING);
```

4. When INPUT_ONLY is set, the key must be entered by the finsh command cmd_diss
   set_key conn_IDX key
```c
connection_manager_set_bond_cnf_iocap(GAP_IO_CAP_DISPLAY_ONLY);
```

5. The finsh command cmd_diss set_key conn_idx sec_level can be used to request
   the central to initiate a pairing

6. The central can be requested to initiate pairing using the finsh command:
   cmd_diss set_key conn_idx sec_level.

### Hardware Requirements
Before running this example, prepare:
+ One development board supported by this example ([Supported
  Platforms](#Platform_peri))
+ Mobile device

### Menuconfig Configuration
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
Navigate to the example project directory and run the scons command to compile:
```c
&gt; scons --board=eh-lb525 -j32
```
Navigate to the example's `project/build_xx` directory, run `uart_download.bat`,
and select the port as prompted to download:
```c
$ ./uart_download.bat

     UART Download

Please input the serial port number: 5
```
For detailed steps on compilation and downloading, please refer to the relevant
section in the [Quick Start Guide](/quickstart/get-started.md).

## Expected Results
<!-- 说明例程运行结果，比如哪几个灯会亮，会打印哪些log，以便用户判断例程是否正常运行，运行结果可以结合代码分步骤说明 -->
After the example starts:
1. The device can be discovered and connected by mobile BLE APP, and
   corresponding GATT characteristic value read/write operations can be
   performed.
2. By modifying io and then initiating pairing on the mobile terminal, different
   pairing authentication methods can be implemented.
3. You can also initiate an encryption request from the periphery development
   board side and then pair it through the finsh command.

## Troubleshooting


## Reference Documentation
<!-- 对于rt_device的示例，rt-thread官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考RT-Thread的[RTC文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Update History
| Version | Date    | Release Notes   |
| ------- | ------- | --------------- |
| 0.0.1   | 01/2025 | Initial version |
|         |         |                 |
|         |         |                 |
