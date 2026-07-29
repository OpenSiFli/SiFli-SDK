# BT/BLE UART HCI Transceiver Example

Source code path: example/bt/HCI_over_uart/src

## Supported Platforms
<!-- Which development boards and chip platforms are supported -->
+ eh-lb52x
+ eh-lb56x
+ eh-lb58x

## Overview
<!-- Example introduction -->
This example demonstrates HCI communication with BT/BLE controller through uart1.

## Example Usage
<!-- Instructions on how to use the example, such as which hardware pins to connect to observe waveforms, compilation and flashing can reference related documentation.
For rt_device examples, the configuration switches used in this example also need to be listed, for example, PWM examples use PWM1, which needs to be enabled in the onchip menu -->

After the example starts, HCI commands can be sent to the controller through uart1, and HCI events will also be sent back to uart1.

The default console log port for 52x is uart3 (see project/sf32lb52x/proj.conf)

The default console log port for 56x/58x is uart4 (see project/sf32lb56x/proj.conf) (see project/sf32lb58x/proj.conf)

### Hardware Requirements
Before running this example, you need to prepare:
+ One development board supported by this example ([Supported Platforms](#Platform_music_sink)).

### menuconfig Configuration

1. None

### Compilation and Flashing
Switch to the example project directory and run the scons command to compile:
```c
> scons --board=eh-lb525 -j32
```
Switch to the example `project/build_xx` directory and run `uart_download.bat`, select the port as prompted to download:
```c
$ ./uart_download.bat

     Uart Download

please input the serial port num:5
```
For detailed compilation and download steps, please refer to the relevant introduction in [Quick Start](/quickstart/get-started.md).

## Expected Results
<!-- Describe the example execution results, such as which LEDs will light up, what logs will be printed, so users can determine if the example is running normally. Results can be explained step by step combined with code -->
After the example starts:
HCI commands can be sent to the controller through uart1, and HCI events will also be sent back to uart1.

For example: Send hexadecimal 01 03 0C 00 (need to add carriage return and line feed) through uart1, uart1 will receive hexadecimal 04 0E 04 06 03 0C 00

## Exception Diagnosis

## Reference Documentation
<!-- For rt_device examples, RT-Thread official website documentation provides detailed explanations, web links can be added here, for example, refer to RT-Thread's [RTC Documentation](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Update History
|Version |Date   |Release Notes |
|:---|:---|:---|
|0.0.1 |01/2025 |Initial version |
| | | |
| | | |