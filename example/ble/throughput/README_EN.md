# BLE Throughput Example

Source path: example/ble/throughput

(Platform_peri)=
## Supported Platforms
<!-- Which boards and chip platforms are supported -->
All platforms

## Overview
<!-- Introduction to the example -->
This example demonstrates how this platform implements GAP central and peripheral roles, as well as GATT server and client functionalities, including connection establishment, service discovery, service registration, GATT write operations as a client, receiving notifications, receiving GATT writes as a server, and sending notifications.

## Usage
<!-- Explain how to use the example, such as which hardware pins to connect for waveform observation, compilation and flashing can reference related documentation.
For rt_device examples, also list the configuration switches used in this example, such as PWM example using PWM1, which needs to be enabled in the onchip menu -->
1. Flash this program to two development boards.
2. After flashing and running, enter the following finsh command on one of the development boards:
```
diss speed_test 99227214AC03 512 0 100 65535 0
```
This will start the BLE throughput speed test, and the results will be printed in the log later.

Command parameters:
- First parameter: Reversed address of the target board to connect (in this case, the address is 03:AC:14:72:22:99)
- Second parameter: Negotiated MTU
- Third parameter: Connection parameters (0 is the default short interval connection parameter; you can add custom connection parameters for testing as needed)
- Fourth parameter: Number of packets to send in one test
- Fifth parameter: Length of each packet (0 to 65535; if exceeding MTU limit, it will be filled according to the maximum length, i.e., MTU-3)
- Sixth parameter: Frequency of peer replies during transmission (e.g., setting to 10 means reply once every 10 packets; 0 means no reply)

### Hardware Requirements
Before running this example, prepare:
+ Two development boards supported by this example ([Supported Platforms](#Platform_peri))

### Menuconfig Configuration
1. Enable Bluetooth (`BLUETOOTH`):\
![BLUETOOTH](./assets/bluetooth.png)
2. Enable GAP, GATT Client, BLE connection manager:\
![BLE MIX](./assets/gap_gatt_ble_cm.png)
3. Enable NVDS:\
![NVDS](./assets/bt_nvds.png)

### Compilation and Flashing
Navigate to the example's project/common directory and run the scons command to compile:
```c
> scons --board=eh-lb525 -j32
```
Navigate to the example's `project/common/build_xx` directory, run `uart_download.bat`, and select the port as prompted to download:
```c
$ ./uart_download.bat

     Uart Download

please input the serial port num:5
```
For detailed steps on compilation and downloading, please refer to the relevant section in the [Quick Start Guide](/quickstart/get-started.md).

## Expected Results
<!-- Explain the expected results of the example, such as which LEDs will light up, what logs will be printed, so users can determine if the example is running correctly. The results can be explained step by step in combination with the code -->
After the example starts:
1. After executing the finsh command, it will connect to another development board and send data. After transmission is complete, it will print information such as transfer speed.

## Troubleshooting
1. After executing the finsh command and connecting to another development board, the connection immediately disconnects with reason code 62.
When a disconnection with code 0x3e occurs immediately after connection, simply retry the test.

## Reference Documentation
<!-- For rt_device examples, the RT-Thread official documentation provides detailed explanations. You can add web links here, for example, refer to RT-Thread's [RTC Documentation](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Update History
|Version |Date   |Release Notes |
|:---|:---|:---|
|0.0.1 |02/2025 |Initial version |
| | | |
| | | |