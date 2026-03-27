# BLE AMS Example

Source code path: example/ble/ams

(Platform_ams)=
## Supported Platforms
<!-- 支持哪些板子和芯片平台 -->
All platforms

## Overview
<!-- 例程简介 -->
This example demonstrates how to trigger Apple AMS (Apple Media Service)
protocol subscription and simple handling of corresponding events. AMS is an
audio control protocol provided by Apple that allows access to the current iOS
device's audio playback status and control of music play, pause, etc.


## Usage Instructions
<!-- 说明如何使用例程，比如连接哪些硬件管脚观察波形，编译和烧写可以引用相关文档。
对于rt_device的例程，还需要把本例程用到的配置开关列出来，比如PWM例程用到了PWM1，需要在onchip菜单里使能PWM1 -->
1. The example starts advertising upon boot with the name
   SIFLI_APP-xx-xx-xx-xx-xx-xx, where xx represents the Bluetooth address of the
   device. This can be obtained using the finsh command "nvds get_mac".
2. Use BLE software (LightBlue, nRF Connect, etc.) on iOS devices (iPhone or
   iPad) to connect to this device. Note that AMS requires pairing to complete,
   so you must accept when the pairing dialog appears on the iOS device.
3. Play music on the iOS device. This example has registered AMS
   Player/queue/track, and you can see related content in the logs.
    1) Related protocol reference: [AMS Official
       Documentation](https://developer.apple.com/library/archive/documentation/CoreBluetooth/Reference/AppleMediaService_Reference/Specification/Specification.html)

### Hardware Requirements
Before running this example, prepare:
+ One development board supported by this example ([Supported
  Platforms](#Platform_ams)).
+ iOS device.

### menuconfig Configuration
1. Enable Bluetooth (`BLUETOOTH`):
    - Path: Sifli middleware → Bluetooth
    - Enable: Enable bluetooth
        - Macro switch: `CONFIG_BLUETOOTH`
        - Description: Enable Bluetooth functionality
2. Enable GAP, GATT Client, BLE connection manager, and AMS:
    - Path: Sifli middleware → Bluetooth → Bluetooth service → BLE service
    - Enable: Enable BLE GAP central role
        - Macro switch: `CONFIG_BLE_GAP_CENTRAL`
        - Description: Switch for BLE CENTRAL (central device). When enabled,
          provides scanning and active connection initiation with peripherals.
    - Enable: Enable BLE GATT client
        - Macro switch: `CONFIG_BLE_GATT_CLIENT`
        - Description: Switch for GATT CLIENT. When enabled, can actively search
          for and discover services, read/write data, and receive notifications.
    - Enable: Enable BLE connection manager
        - Macro switch: `CONFIG_BSP_BLE_CONNECTION_MANAGER`
        - Description: Provides BLE connection control management, including
          multi-connection management, BLE pairing, link connection parameter
          updates, etc.
    - Enable: Enable BLE AMS
        - Macro switch: `CONFIG_BSP_BLE_AMS`
        - Description: Apple Media Service. After registering AMS, provides iOS
          device playback control, playback synchronization, volume control,
          etc.
3. Enable AMS in Data service:
    - Path: Sifli middleware → Enable Data service
    - Enable: Enable AMS Service
        - Macro switch: `CONFIG_BSP_USING_AMS_SVC`
        - Description: When using Data Service to register AMS, this option
          needs to be enabled
4. Enable NVDS:
    - Path: Sifli middleware → Bluetooth → Bluetooth service → Common service
    - Enable: Enable NVDS synchronous
        - Macro switch: `CONFIG_BSP_BLE_NVDS_SYNC`
        - Description: Bluetooth NVDS synchronization. When Bluetooth is
          configured to HCPU, BLE NVDS can be accessed synchronously, so enable
          this option; when Bluetooth is configured to LCPU, this option needs
          to be disabled.

### Compilation and Flashing
Switch to the example project directory and run the scons command to compile:
```c
&gt; scons --board=eh-lb525 -j32
```
Switch to the example `project/build_xx` directory and run `uart_download.bat`,
then select the port as prompted to download:
```c
$ ./uart_download.bat

     UART Download

Please input the serial port number: 5
```
For detailed compilation and download steps, please refer to the [Quick Start
Guide](/quickstart/get-started.md).

## Expected Results
<!-- 说明例程运行结果，比如哪几个灯会亮，会打印哪些log，以便用户判断例程是否正常运行，运行结果可以结合代码分步骤说明 -->
After the example starts:
1. It can be connected and successfully paired by BLE software on iOS (such as
   LightBlue, nRF Connect).
2. When the iOS device plays music, the device displays related information
   through logs.

## Troubleshooting


## Reference Documentation
<!-- 对于rt_device的示例，rt-thread官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考RT-Thread的[RTC文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Update History
| Version | Date    | Release Notes   |
| ------- | ------- | --------------- |
| 0.0.1   | 01/2025 | Initial version |
|         |         |                 |
|         |         |                 |
