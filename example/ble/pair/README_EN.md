# BLE Peripheral Example

Source path: example/ble/peripheral

(Platform_peri)=
## Supported Platforms
<!-- Which boards and chip platforms are supported -->
All platforms

## Overview
<!-- Introduction to the example -->
This example demonstrates how this platform implements GAP peripheral and GATT server functionalities.

## Usage
<!-- Explain how to use the example, such as which hardware pins to connect for waveform observation, compilation and flashing can reference related documentation.
For rt_device examples, also list the configuration switches used in this example, such as PWM example using PWM1, which needs to be enabled in the onchip menu -->
1. As a slave device, advertising will be enabled at startup with the advertising name SIFLI_APP-xx-xx-xx-xx-xx-xx, where xx represents the Bluetooth address of this device. Connection can be established through a mobile BLE APP.
2. As a GATT server, write and read operations can be performed from the mobile device, or CCCD can be enabled, and the device will update the characteristic value every second.
3. In the initialization stage, bond_ack is set to BOND.Pending by default. You need to pay attention to CONNECTION_MANAGER_BOND_AUTH_INFOR and call connectionManager_bond_ack_reply in time.
```c
connection_manager_set_bond_ack(BOND_PENDING);
```
4.You can reset the IO by setting IO cap to GAP_IO.cap_DISPLAY_ONLY by default, or you can also reset the IO by using the finsh command cmd_diss set_io iocap.
```c
connection_manager_set_bond_cnf_iocap(GAP_IO_CAP_DISPLAY_ONLY);
```
5. When INPUT_ONLY is set, the key must be entered by the finsh command cmd_diss set_key conn_IDX key

6. The finsh command cmd_diss set_key conn_idx sec_level can be used to request the central to initiate a pairing



### Hardware Requirements
Before running this example, prepare:
+ One development board supported by this example ([Supported Platforms](#Platform_peri))
+ Mobile device

### Menuconfig Configuration
1. Enable Bluetooth (`BLUETOOTH`):\
![BLUETOOTH](./assets/bluetooth.png)
2. Enable GAP, GATT Client, BLE connection manager:\
![BLE MIX](./assets/gap_gatt_ble_cm.png)
3. Enable NVDS:\
![NVDS](./assets/bt_nvds.png)

### Compilation and Flashing
Navigate to the example project directory and run the scons command to compile:
```c
> scons --board=eh-lb525 -j32
```
Navigate to the example's `project/build_xx` directory, run `uart_download.bat`, and select the port as prompted to download:
```c
$ ./uart_download.bat

     Uart Download

please input the serial port num:5
```
For detailed steps on compilation and downloading, please refer to the relevant section in the [Quick Start Guide](/quickstart/get-started.md).

## Expected Results
<!-- Explain the expected results of the example, such as which LEDs will light up, what logs will be printed, so users can determine if the example is running correctly. The results can be explained step by step in combination with the code -->
After the example starts:
1. The device can be discovered and connected by mobile BLE APP, and corresponding GATT characteristic value read/write operations can be performed.
2. By modifying io and then initiating pairing on the mobile terminal, different pairing authentication methods can be implemented.
3. You can also initiate an encryption request from the periphery development board side and then pair it through the finsh command.

## Troubleshooting

## Reference Documentation
<!-- For rt_device examples, the RT-Thread official documentation provides detailed explanations. You can add web links here, for example, refer to RT-Thread's [RTC Documentation](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Update History
|Version |Date   |Release Notes |
|:---|:---|:---|
|0.0.1 |01/2025 |Initial version |
| | | |
| | | |