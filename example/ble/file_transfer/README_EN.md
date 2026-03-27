# BLE File Transfer Example

Source code path: example/ble/file_transfer (Platform_file_rec)=

(Platform_file_rec)=
## Supported Platforms
<!-- 支持哪些板子和芯片平台 -->
All platforms

## Overview
<!-- 例程简介 -->
This example demonstrates how to implement a GAP peripheral role on this
platform to receive files sent from a mobile device.

## Example Usage
<!-- 说明如何使用例程，比如连接哪些硬件管脚观察波形，编译和烧写可以引用相关文档。
对于rt_device的例程，还需要把本例程用到的配置开关列出来，比如PWM例程用到了PWM1，需要在onchip菜单里使能PWM1 -->
1. The Finsh commands for this example can be displayed by entering `diss help`
   to show command usage.
2. When operating as a peripheral device, it will start advertising on boot with
   the name format: SIFLI_APP-xx-xx-xx-xx-xx-xx, where xx represents the
   device's Bluetooth address. You can connect using a mobile BLE app.
3. The sifli ble serial transport protocol can be used to send files from the
   mobile device to the target device, supporting one or multiple files in a
   single transfer.

### Hardware Requirements
Before running this example, you need to prepare:
+ A development board supported by this example ([Supported
  Platforms](#Platform_file_rec)).
+ A mobile device.

### menuconfig Configuration
1. Enable Bluetooth(`BLUETOOTH`):
    - Path: Sifli middleware → Bluetooth
    - Enable: Enable bluetooth
        - Macro switch: `CONFIG_BLUETOOTH`
        - Description: Enable Bluetooth functionality
2. Enable GAP, GATT Client, BLE connection manager:
    - Path: Sifli middleware → Bluetooth → Bluetooth service → BLE service
    - Enable: Enable BLE GAP central role
        - Macro switch: `CONFIG_BLE_GAP_CENTRAL`
        - Description: Switch for BLE CENTRAL (central device). When enabled, it
          provides scanning and active connection initiation with peripherals.
    - Enable: Enable BLE GATT client
        - Macro switch: `CONFIG_BLE_GATT_CLIENT`
        - Description: Switch for GATT CLIENT. When enabled, it can actively
          search and discover services, read/write data, and receive
          notifications.
    - Enable: Enable BLE connection manager
        - Macro switch: `CONFIG_BSP_BLE_CONNECTION_MANAGER`
        - Description: Provides BLE connection control management, including
          multi-connection management, BLE pairing, link connection parameter
          updates, etc.
3. Enable NVDS:
    - Path: Sifli middleware → Bluetooth → Bluetooth service → Common service
    - Enable: Enable NVDS synchronous
        - Macro switch: `CONFIG_BSP_BLE_NVDS_SYNC`
        - Description: Bluetooth NVDS synchronization. When Bluetooth is
          configured to HCPU, BLE NVDS can be accessed synchronously, so enable
          this option; when Bluetooth is configured to LCPU, this option needs
          to be disabled
4. Enable transmission related macros:
    - Path: Sifli middleware → Bluetooth → Bluetooth service → BLE service
    - Enable: Enable BLE serial transmission
        - Macro switch: `CONFIG_BSP_BLE_SERIAL_TRANSMISSION`
        - Description: Serial transmission protocol. When enabled, it provides a
          simple serial transmission protocol that allows users to transmit
          content to the development board via BLE.
    - Enable: Enable watch face downloaded via BLE
        - Macro switch: `CONFIG_BSP_BLE_WATCH_FACE`
        - Description: Watch face transmission protocol. Depends on serial
          transmission. When enabled, files can be transferred to the
          development board via BLE using the encapsulated mobile SDK.

### Compilation and Flashing
Navigate to the example's project/common directory and run the scons command to
compile:
```c
&gt; scons --board=eh-lb525 -j32
```
Navigate to the example's `project/common/build_xx` directory and run
`uart_download.bat`, then select the port as prompted to download:
```c
$ ./uart_download.bat
     Uart Download
please input the serial port num:5
```
For detailed compilation and downloading steps, please refer to the relevant
sections in [Quick Start](/quickstart/get-started.md).

## Expected Results
<!-- 说明例程运行结果，比如哪几个灯会亮，会打印哪些log，以便用户判断例程是否正常运行，运行结果可以结合代码分步骤说明 -->
After the example starts:
1. It can be discovered and connected by a mobile BLE app.
2. Compress the files to be transferred into a single .zip file and transfer
   this file to your mobile device.
3. Use the SIFLI BLE APP to search, connect the device, then go to the watchface
   interface, select the custom file, and then choose this file. This file can
   then be transferred to the file system of the development board.
4. Mobile app operation steps:
- Enable location permission for opening the feature ![APP1](./assets/app0.png)
- Click the query button at the bottom right corner to enter the Bluetooth
  scanning interface. ![APP1](./assets/app1.png)
- Click on "SCAN" to start the scan. ![APP2](./assets/app2.png)
- After finding the device, click on "SORT", and the broadcast named
  "SIFLI_APP-xx-xx-xx-xx-xx-xx" will be placed at the top.
  ![APP3](./assets/app3.png)
- Click on the broadcast name "SIFLI_APP-xx-xx-xx-xx-xx-xx" to enter the device
  interface. ![APP4](./assets/app4.png)
- Click on the upper right corner to enter the operation interface.
  ![APP4](./assets/app5.png)
- Click "CONNECT" to ensure that the State is "CONNECT"
  ![APP4](./assets/app6.png)
- Swipe up the last box, find File transfer WFPUSH2 and click to enter the file
  transfer interface ![APP4](./assets/app7.png)
- Click on "SELECT FILE" to select the pre-prepared zip file from your phone,
  and set the "File Type" to 3. Then, check the "With Byte Align" option, and
  finally click "START". ![APP4](./assets/app8.png)

- After the transmission is completed, execute the command in the console window
  to enter the "file_recv" folder, and then execute the "ls" command to view the
  list of files. Among them, the content of the zip file will be displayed.
  ![APP4](./assets/app9.png)

## Access to the mobile APP and DEMO project for use
### Download address of Android's free software application
https://www.pgyer.com/gurSBc

### Android demo project
https://github.com/OpenSiFli/SiFli_OTA_APP\
The corresponding part is "3. SiFli-SDK OTA"

### iOS Demo Project
https://github.com/OpenSiFli/SiFli_OTA_APP_IOS\
The corresponding part is "SiFli-SDK OTA (Nor Offline)" SiFli-SDK file transfer

## Troubleshooting
When the development board returns an error, check the ble_watchface_status_id_t
enumeration in bf0_sibles_watchface.h

## Reference Documentation
<!-- 对于rt_device的示例，rt-thread官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考RT-Thread的[RTC文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Update History
| Version | Date    | Release Notes   |
| ------- | ------- | --------------- |
| 0.0.1   | 07/2025 | Initial version |
|         |         |                 |
|         |         |                 |
