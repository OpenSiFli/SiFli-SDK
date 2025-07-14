# BLE HID Example

Source code path: example/ble/hid

(Platform_hid)=
## Supported Platforms
<!-- Which boards and chip platforms are supported -->
All platforms

## Overview
<!-- Example introduction -->
1. This example demonstrates how this platform implements BLE HID.
2. The example implements two types of HID types: HID_KEYBOARD and HID_CONSUMER.
    1) main.c defines HID_CONSUMER by default. You can switch the compiled supported type by undefining HID_CONSUMER and defining HID_KEYBOARD.
    ![HID_TYPE_CHOOSE](./assets/hid_type_choose.png)


## Example Usage
<!-- Instructions on how to use the example, such as which hardware pins to connect to observe waveforms, compilation and programming can reference related documentation.
For rt_device examples, also need to list the configuration switches used by this example, such as PWM examples using PWM1, need to enable PWM1 in the onchip menu -->
1. This example starts advertising upon boot, with the broadcast name SIFLI_APP-xx-xx-xx-xx-xx-xx, where xx represents the Bluetooth address of this device. You can connect using a BLE APP on your phone.
2. After the phone connects to this device, use the finsh command "test_hids [key] [p|r]" to send control commands to the phone. Where p|r represents press|release, which need to be used in pairs; key has different values depending on whether the HID type is keyboard or consumer:
    1) Keyboard type key value range:
    ![keyboard_value](./assets/keyboard_value.png)
    2) Consumer type key value range:
    ![consumer_value](./assets/consumer_value.png)
    3) For example, in consumer type, entering "test_hids 0 p" represents the press operation for the key value corresponding to PLAY.


### Hardware Requirements
Before running this example, you need to prepare:
+ A development board supported by this example ([Supported Platforms](#Platform_hid)).
+ A mobile device.

### menuconfig Configuration
1. Enable Bluetooth (`BLUETOOTH`):
![BLUETOOTH](./assets/bluetooth.png)
2. Enable GAP, GATT Client, BLE connection manager:
![BLE MIX](./assets/gap_gatt_ble_cm.png)
3. Enable NVDS:
![NVDS](./assets/bt_nvds.png)


### Compilation and Programming
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
For detailed steps on compilation and downloading, please refer to the relevant introduction in [Quick Start](/quickstart/get-started.md).

## Expected Results
<!-- Describe the example execution results, such as which LEDs will light up, what logs will be printed, so users can determine if the example is running normally. Results can be explained step by step combined with code -->
After the example starts:
1. It can be discovered and connected by a BLE APP on the phone.
2. The phone can be controlled through finsh commands.

## Exception Diagnosis


## Reference Documents
<!-- For rt_device examples, the RT-Thread official website documentation provides more detailed descriptions. Web page links can be added here, for example, refer to RT-Thread's [RTC documentation](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Update Log
|Version |Date   |Release Notes |
|:---|:---|:---|
|0.0.1 |01/2025 |Initial version |
| | | |
| | | |