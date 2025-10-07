# BLE iBeacon Advertisement Example

Source code path: example/ble/iBeacon

<a id="Platform_iBeacon_adv"></a>
## Supported Platforms
<!-- Which boards and chip platforms are supported -->
+ eh-lb52x
+ eh-lb56x
+ eh-lb58x

## Overview
<!-- Example introduction -->
This example demonstrates the usage of iBeacon advertisement.


## Usage Instructions
<!-- Explain how to use the example, such as which hardware pins to connect to observe waveforms, compilation and flashing can reference related documentation.
For rt_device examples, also list the configuration switches used in this example, for example PWM example uses PWM1, need to enable PWM1 in onchip menu -->
1. After booting, the iBeacon broadcast will be enabled. You can refer to the implementation of ble_app_ibeacon_advertising_start(). The default iBeacon broadcast content is UUID: 95d0b422-a4bd-45d4-9920-576bc6632372, Major: 256, Minor: 258, RSSI at 1m: -50 dBm.
2. Use the finsh command "cmd_diss adv_update [UUID] [Major] [Minor] [RSSI_at_1m]" to modify the broadcast content. The UUID format should be like "12345678-1234-1234-1234-123456789abc"; Major value range is 0–65535; Minor value range is 0–65535; RSSI_at_1m value range is -128 to 127.
3. Use the finsh commands "cmd_diss adv_start" and "cmd_diss adv_stop" to enable and stop the iBeacon broadcast, respectively.


### Hardware Requirements
Before running this example, prepare:
+ One development board supported by this example ([Supported Platforms](#Platform_iBeacon_adv)).
+ Mobile device.

### menuconfig Configuration
1. Enable Bluetooth (`BLUETOOTH`):
    - Path: Sifli middleware → Bluetooth
    - Enable: Enable bluetooth
        - Macro switch: `CONFIG_BLUETOOTH`
        - Description: Enables Bluetooth functionality
2. Enable NVDS:
    - Path: Sifli middleware → Bluetooth → Bluetooth service → Common service
    - Enable: Enable NVDS synchronous
        - Macro switch: `CONFIG_BSP_BLE_NVDS_SYNC`
        - Description: Bluetooth NVDS synchronization. When Bluetooth is configured to HCPU, BLE NVDS can be accessed synchronously, so enable this option; when Bluetooth is configured to LCPU, this option needs to be disabled.

### Compilation and Flashing
Switch to the example project directory and run the scons command to compile:
```c
> scons --board=eh-lb525 -j32
```
Switch to the example `project/build_xx` directory and run `uart_download.bat`, then select the port as prompted to download:
```c
$ ./uart_download.bat

     Uart Download

please input the serial port num:5
```
For detailed compilation and download steps, please refer to the [Quick Start Guide](/quickstart/get-started.md).

## Expected Results
<!-- Describe the expected results of running the example, such as which LEDs will light up, what logs will be printed, to help users determine if the example is running normally. Results can be explained step by step in conjunction with the code -->
After the example starts:
1. It can perform iBeacon advertisement and modify advertisement content.


## Troubleshooting


## Reference Documentation
<!-- For rt_device examples, RT-Thread official documentation provides detailed explanations. Web links can be added here, for example, refer to RT-Thread's [RTC Documentation](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Update History
|Version |Date   |Release Notes |
|:---|:---|:---|
|0.0.1 |09/2025 |Initial version |
| | | |
| | | |