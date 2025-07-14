# BLE ANCS Example

Source code path: example/ble/ancs_dualcore

(Platform_ancs)=
## Supported Platforms
<!-- Which boards and chip platforms are supported -->
+ eh-lb55x

## Overview
<!-- Example introduction -->
This example demonstrates how to trigger Apple ANCS (Apple Notification Center Service) protocol subscription and simple handling of corresponding events.
ANCS is a messaging protocol provided by Apple that allows access to all messages received in the iOS device's notification bar and control of some special messages.


## Usage Instructions
<!-- Explain how to use the example, such as which hardware pins to connect to observe waveforms, compilation and flashing can reference related documentation.
For rt_device examples, also list the configuration switches used in this example, for example PWM example uses PWM1, need to enable PWM1 in onchip menu -->
1. The example starts advertising upon boot with the name SIFLI_APP-xx-xx-xx-xx-xx-xx, where xx represents the Bluetooth address of the device. This can be obtained using the finsh command "nvds get_mac".
2. Use BLE software (LightBlue, nRF Connect, etc.) on iOS devices (iPhone or iPad) to connect to this device. Note that ANCS requires pairing to complete, so you must accept when the pairing dialog appears on the iOS device.
    2) Higher iOS versions may also show a system notification sharing confirmation dialog in addition to the pairing dialog, which can also be toggled in the iOS Bluetooth settings for the corresponding device.
3. When messages are received, this example will print them through HCPU logs.
    1) Related protocol reference: [ANCS Official Documentation](https://developer.apple.com/library/archive/documentation/CoreBluetooth/Reference/AppleNotificationCenterServiceSpecification/Specification/Specification.html)


### Hardware Requirements
Before running this example, prepare:
+ One development board supported by this example ([Supported Platforms](#Platform_ancs)).
+ iOS device.

### menuconfig Configuration

- Platform
    - Select board peripherials->Select board: Select the running platform, please choose LB55XXX series according to the actual board. This option needs to be selected in both CPU corresponding options and ensure they are the same platform.
- HCPU
    - RTOS->On-chip Peripherial Drivers->Enable LCPU Patch[*]:
    - RTOS->On-chip Peripherial Drivers->Enable LCPU image[*]: Enable LCPU Patch and image for LCPU to run normally.
    - Sifli Middleware->Enable Data service[*]: Enable Data service, which provides a data interaction mechanism between data providers and data subscribers.
        - Enable ble nvds service[*]: Enable NVDS data service in Data service. This service provides asynchronous NVDS operations for BLE stack/service running on LCPU to store data to HCPU flash.
    - Third party packages->FlashDB[*]: Enable FlashDB to provide flash access interface, NVDS requires this service to be enabled.
- LCPU
    - Sifli Middleware->Enable BLE service[*]: Enable BLE service, which provides BLE GAP/GATT/COMMON services.
        - Enable BLE ANCS[*]: Enable BLE ANCS service, which provides access to iOS ANCS service, obtains notifications and provides them to users.
    - Sifli Middleware->Enable BLE stack[*]: Enable BLE protocol stack.
    - Sifli Middleware->Enable Data service[*]: Enable Data service, which provides a data interaction mechanism between data providers and data subscribers. ANCS uses this mechanism mainly to separate data and UI.
        - Enable ANCS service[*]: Enable ANCS data service in Data service. This service configures and enables BLE ANCS service and processes received notifications, providing processed data to subscribers of this service.


### Compilation and Flashing
Switch to the example project/lcpu/xx directory and run the scons command to compile:
```c
> scons -j8
```
Switch to the example project/hcpu/xx directory and run the scons command to compile:
```c
> scons -j8
```
Switch to the example `project/hcpu/xx/build_xx` directory and use JLink software commands to download the corresponding software:
```c
loadbin project/hcpu/xx/build_xx/xx.bin addr
loadfile project/hcpu/xx/build_xx/xx.hex
```
For detailed compilation and download steps, please refer to the [Quick Start Guide](/quickstart/get-started.md).

## Expected Results
<!-- Describe the expected results of running the example, such as which LEDs will light up, what logs will be printed, to help users determine if the example is running normally. Results can be explained step by step in conjunction with the code -->
After the example starts:
1. It can be connected and successfully paired by BLE software on iOS (such as LightBlue, nRF Connect).
2. When iOS receives messages, the device displays related information through logs.

## Troubleshooting


## Reference Documentation
<!-- For rt_device examples, RT-Thread official documentation provides detailed explanations. Web links can be added here, for example, refer to RT-Thread's [RTC Documentation](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Update History
|Version |Date   |Release Notes |
|:---|:---|:---|
|0.0.1 |01/2025 |Initial version |
| | | |
| | | |