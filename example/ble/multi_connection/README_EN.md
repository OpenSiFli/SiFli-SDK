# BLE multi_connection Example

Source code path: example/ble/multi_connection

## Supported Platforms
<!-- Which platforms are supported -->
All platforms

## Overview
This example demonstrates BLE multi-connection functionality based on GAP central and peripheral roles and GATT server on this platform.

## Usage Instructions
1. When operating as a slave device, the board automatically starts advertising upon boot and can be connected via BLE apps on mobile phones.
2. After connection, the board automatically restarts advertising and can accept connections from other mobile devices.
3. It can also operate as a master device, searching for and connecting to other slave devices through finsh commands.

### Hardware Requirements
Before running this example, prepare:
+ One development board supported by this example ([Supported Platforms](#Platform_peri)).
+ Mobile device.

### menuconfig Configuration
1. Enable Bluetooth (`BLUETOOTH`):\
![BLUETOOTH](./assets/bluetooth.png)
2. Enable GAP, GATT Client, BLE connection manager:\
![BLE MIX](./assets/gap_gatt_ble_cm.png)
3. Enable NVDS:\
![NVDS](./assets/bt_nvds.png)


### Compilation and Flashing
Switch to the example project/common directory and run the scons command to compile:
```c
> scons --board=eh-lb525 -j8
```
Switch to the example `project/common/build_xx` directory and run `uart_download.bat`, then select the port as prompted to download:
```c
$ ./uart_download.bat

     Uart Download

please input the serial port num:5
```
For detailed compilation and download steps, please refer to the [Quick Start Guide](/quickstart/get-started.md).

## Expected Results
<!-- Describe the expected results of running the example, such as which LEDs will light up, what logs will be printed, to help users determine if the example is running normally. Results can be explained step by step in conjunction with the code -->
After the example starts:
1. It can be discovered and connected by multiple different mobile phones via BLE apps, allowing corresponding GATT characteristic value read/write operations.
2. It can actively connect to other devices

## Troubleshooting


## Reference Documentation
<!-- For rt_device examples, RT-Thread official documentation provides detailed explanations. Web links can be added here, for example, refer to RT-Thread's [RTC Documentation](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Update History
|Version |Date   |Release Notes |
|:---|:---|:---|
|0.0.1 |02/2025 |Initial version |
| | | |
| | | |