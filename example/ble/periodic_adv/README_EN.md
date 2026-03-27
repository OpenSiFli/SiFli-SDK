# BLE Periodic Advertisement Example

Source code path: example/ble/periodic_adv

(Platform_peri_adv)=
## Supported Platforms
<!-- 支持哪些板子和芯片平台 -->
All platforms


## Overview
<!-- 例程简介 -->
This example demonstrates the usage of periodic advertisement.


## Usage Instructions
<!-- 说明如何使用例程，比如连接哪些硬件管脚观察波形，编译和烧写可以引用相关文档。
对于rt_device的例程，还需要把本例程用到的配置开关列出来，比如PWM例程用到了PWM1，需要在onchip菜单里使能PWM1 -->
1. Periodic advertisement starts after boot, refer to the implementation of
   ble_app_peri_advertising_start(). The default periodic advertisement content
   is 80 bytes of all-zero data.
2. Use finsh command "cmd_diss keep_per start [change_period] [len]" to change
   the periodic advertisement content. The change_period value range is 20-255
   in milliseconds; len value range is 0-100 in bytes.
    1) After starting, the periodic advertisement will update content according
       to the set change period. The content consists of repeated digits of the
       specified length, and this digit increments with each periodic update,
       cycling between 0-255.
    2) Use finsh command "cmd_diss keep_per stop" to stop updating the periodic
       advertisement content. Note that this command only stops updating the
       content but does not stop the periodic advertisement.
3. Use finsh commands "cmd_diss adv_start" and "cmd_diss adv_stop" to enable and
   stop periodic advertisement.

### Hardware Requirements
Before running this example, prepare:
+ One development board supported by this example ([Supported
  Platforms](#Platform_peri_adv)).
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
1. It can perform periodic advertisement and modify advertisement content.

## Troubleshooting


## Reference Documentation
<!-- 对于rt_device的示例，rt-thread官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考RT-Thread的[RTC文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Update History
| Version | Date    | Release Notes   |
| ------- | ------- | --------------- |
| 0.0.1   | 01/2025 | Initial version |
|         |         |                 |
|         |         |                 |
