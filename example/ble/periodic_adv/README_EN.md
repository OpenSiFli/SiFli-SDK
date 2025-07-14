# BLE Periodic Advertisement Example

Source code path: example/ble/periodic_adv

(Platform_peri_adv)=
## Supported Platforms
<!-- Which boards and chip platforms are supported -->
+ eh-lb52x
+ eh-lb56x
+ eh-lb58x

## Overview
<!-- Example introduction -->
This example demonstrates the usage of periodic advertisement.


## Usage Instructions
<!-- Explain how to use the example, such as which hardware pins to connect to observe waveforms, compilation and flashing can reference related documentation.
For rt_device examples, also list the configuration switches used in this example, for example PWM example uses PWM1, need to enable PWM1 in onchip menu -->
1. Periodic advertisement starts after boot, refer to the implementation of ble_app_peri_advertising_start(). The default periodic advertisement content is 80 bytes of all-zero data.
2. Use finsh command "cmd_diss keep_per start [change_period] [len]" to change the periodic advertisement content. The change_period value range is 20-255 in milliseconds; len value range is 0-100 in bytes.
    1) After starting, the periodic advertisement will update content according to the set change period. The content consists of repeated digits of the specified length, and this digit increments with each periodic update, cycling between 0-255.
    2) Use finsh command "cmd_diss keep_per stop" to stop updating the periodic advertisement content. Note that this command only stops updating the content but does not stop the periodic advertisement.
3. Use finsh commands "cmd_diss adv_start" and "cmd_diss adv_stop" to enable and stop periodic advertisement.


### Hardware Requirements
Before running this example, prepare:
+ One development board supported by this example ([Supported Platforms](#Platform_peri_adv)).
+ Mobile device.

### menuconfig Configuration
1. Enable Bluetooth (`BLUETOOTH`):\
![BLUETOOTH](./assets/bluetooth.png)
2. Enable NVDS:\
![NVDS](./assets/bt_nvds.png)


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
1. It can perform periodic advertisement and modify advertisement content.


## Troubleshooting


## Reference Documentation
<!-- For rt_device examples, RT-Thread official documentation provides detailed explanations. Web links can be added here, for example, refer to RT-Thread's [RTC Documentation](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Update History
|Version |Date   |Release Notes |
|:---|:---|:---|
|0.0.1 |01/2025 |Initial version |
| | | |
| | | |