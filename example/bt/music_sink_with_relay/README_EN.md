# BT Music Sink Relay Example

Source code path: example/bt/music_sink_with_relay

{#Platform_music_sink}
## Supported Platforms
<!-- 支持哪些板子和芯片平台 -->
+ eh-lb52J

## Overview
<!-- 例程简介 -->
This example demonstrates a music_sink-based implementation that requires two
boards to download this project, enabling simultaneous music playback from a
mobile phone on both devices.


## Example Usage
<!-- 说明如何使用例程，比如连接哪些硬件管脚观察波形，编译和烧写可以引用相关文档。
对于rt_device的例程，还需要把本例程用到的配置开关列出来，比如PWM例程用到了PWM1，需要在onchip菜单里使能PWM1 -->
The example consists of two parts:
1. One part involves operations between the device and mobile phone, same as
   music_sink. The project enables Bluetooth Inquiry scan and page scan on
   startup, allowing A2DP source devices like mobile phones to discover this
   device and initiate connections. After connection, mobile phone music can be
   played.
2. The other part involves operations between two boards. The two boards need to
   complete pairing to achieve music forwarding. Only one board can connect to
   the mobile phone at a time. The board not connected to the mobile phone will
   receive forwarded music information.
3. The default Bluetooth name of this device is sifli_music_sink.

### Hardware Requirements
Before running this example, you need to prepare:
+ Two development boards supported by this example ([Supported
  Platforms](#Platform_music_sink)).
+ Speakers.

### menuconfig Configuration
1. Enable AUDIO CODEC and AUDIO PROC:
    - Path: On-chip Peripheral RTOS Drivers
    - Enable: Enable Audio Process driver
        - Macro switch: `CONFIG_BSP_ENABLE_AUD_PRC`
        - Description: Enable Audio process device, mainly used for audio data
          processing (including resampling, volume adjustment, etc.)
    - Enable: Enable Audio codec driver
        - Macro switch: `CONFIG_BSP_ENABLE_AUD_CODEC`
        - Description: Enable Audio codec device, mainly used for DAC conversion
2. Enable AUDIO(`AUDIO`):
    - Path: Sifli middleware
    - Enable: Enable Audio
        - Description: Enable audio configuration options
3. Enable AUDIO MANAGER(`AUDIO_USING_MANAGER`):
    - Path: Sifli middleware → Enable Audio
    - Enable: Enable audio manager
        - Macro switch: `CONFIG_AUDIO_USING_MANAGER`
        - Description: Use audio manager module for audio process handling
4. Enable Bluetooth(`BLUETOOTH`):
    - Path: Sifli middleware → Bluetooth
    - Enable: Enable bluetooth
        - Macro switch: `CONFIG_BLUETOOTH`
        - Description: Enable Bluetooth functionality
5. Enable A2DP SNK and AVRCP:
    - Path: Sifli middleware → Bluetooth → Bluetooth service → Classic BT
      service
    - Enable: Enable BT finsh (Optional)
        - Macro switch: `CONFIG_BT_FINSH`
        - Description: Enable finsh command line for Bluetooth control
    - Enable: Manually select profiles
        - Macro switch: `CONFIG_BT_PROFILE_CUSTOMIZE`
        - Description: Manually select profiles to enable
    - Enable: Enable A2DP
        - Macro switch: `CONFIG_CFG_AV`
        - Description: Enable A2DP
    - Enable: Enable A2DP sink profile
        - Macro switch: `CONFIG_CFG_AV_SNK`
        - Description: Enable A2DP SINK ROLE
    - Enable: Enable AVRCP
        - Macro switch: `CONFIG_CFG_AVRCP`
        - Description: Enable AVRCP profile
6. Enable BT connection manager:
    - Path: Sifli middleware → Bluetooth → Bluetooth service → Classic BT
      service
    - Enable: Enable BT connection manager
        - Macro switch: `CONFIG_BSP_BT_CONNECTION_MANAGER`
        - Description: Use connection manager module to manage BT connections
7. Enable NVDS:
    - Path: Sifli middleware → Bluetooth → Bluetooth service → Common service
    - Enable: Enable NVDS synchronous
        - Macro switch: `CONFIG_BSP_BLE_NVDS_SYNC`
        - Description: Bluetooth NVDS synchronization. When Bluetooth is
          configured to HCPU, BLE NVDS can be accessed synchronously, enable
          this option; when Bluetooth is configured to LCPU, this option needs
          to be disabled

### Compilation and Flashing
Switch to the example project directory and run the scons command to compile:
```c
&gt; scons --board=eh-lb525 -j32
```
Switch to the example `project/build_xx` directory and run `uart_download.bat`,
select the port as prompted to download:
```c
$ ./uart_download.bat

     UART Download

Please input the serial port number: 5
```
For detailed compilation and download steps, please refer to the relevant
introduction in [Quick Start](/quickstart/get-started.md).

## Expected Results
<!-- 说明例程运行结果，比如哪几个灯会亮，会打印哪些log，以便用户判断例程是否正常运行，运行结果可以结合代码分步骤说明 -->
After the example starts: Mobile phone music forwarding can be achieved with
synchronized sound between forwarding devices without stuttering.

## Exception Diagnosis


## Reference Documentation
<!-- 对于rt_device的示例，rt-thread官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考RT-Thread的[RTC文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Update History
| Version | Date    | Release Notes   |
| ------- | ------- | --------------- |
| 0.0.1   | 01/2025 | Initial version |
|         |         |                 |
|         |         |                 |
