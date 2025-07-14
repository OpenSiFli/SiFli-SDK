# BLE central and peripheral Example

Source code path: example/ble/central_and_peripheral

(Platform_cen_peri)=
## Supported Platforms
<!-- Which boards and chip platforms are supported -->
All platforms

## Overview
<!-- Example introduction -->
This example demonstrates how to simultaneously implement GAP central and peripheral roles as well as GATT client and server on this platform.


## Usage Instructions
<!-- Explain how to use the example, such as which hardware pins to connect to observe waveforms, compilation and flashing can reference related documentation.
For rt_device examples, also list the configuration switches used in this example, for example PWM example uses PWM1, need to enable PWM1 in onchip menu -->
1. The finsh commands for this example can be printed by entering "diss help" to show commands and usage methods.
2. When operating as a slave device, the board starts advertising upon boot with the name SIFLI_APP-xx-xx-xx-xx-xx-xx, where xx represents the Bluetooth address of the device. It can be connected via BLE apps on mobile phones.
3. When operating as a master device, you can search for other slave devices and initiate connections through finsh commands.
4. When operating as a GATT server, write and read operations can be performed from the mobile client, or by enabling CCCD, the device will update the characteristic value every second.
5. When operating as a GATT client, you can search and display the server's database through finsh commands, and perform read or write operations on characteristic values.


### Hardware Requirements
Before running this example, prepare:
+ One development board supported by this example ([Supported Platforms](#Platform_cen_peri)).
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
> scons --board=eh-lb525 -j32
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
1. It can be discovered and connected by mobile BLE apps, allowing corresponding GATT characteristic value read/write operations.
2. It can search for other BLE devices, connect to them and search the connected device's GATT database, while performing GATT read/write operations.

## Troubleshooting


## Reference Documentation
<!-- For rt_device examples, RT-Thread official documentation provides detailed explanations. Web links can be added here, for example, refer to RT-Thread's [RTC Documentation](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Update History
|Version |Date   |Release Notes |
|:---|:---|:---|
|0.0.1 |01/2025 |Initial version |
| | | |
| | | |