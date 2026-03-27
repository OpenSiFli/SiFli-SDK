# BLE Periodic Advertisement Sync Example

Source code path: example/ble/periodic_adv_sync

(Platform_peri_adv_sync)=
## Supported Platforms
<!-- 支持哪些板子和芯片平台 -->
All platforms


## Overview
<!-- 例程简介 -->
This example demonstrates the usage of periodic advertisement synchronization.


## Usage Instructions
<!-- 说明如何使用例程，比如连接哪些硬件管脚观察波形，编译和烧写可以引用相关文档。
对于rt_device的例程，还需要把本例程用到的配置开关列出来，比如PWM例程用到了PWM1，需要在onchip菜单里使能PWM1 -->
1. After boot, use finsh command "diss sync create" to create periodic
   advertisement sync. If created successfully, you can see "PER_ADV_SYNC
   created [idx]" in the log, where idx is the allocated handle.
2. Use finsh command "diss scan start [is_dup] [scan_interval] [scan_window]
   [duration]" to start scan to search for periodic advertisements.
    1) Command example: "diss scan start 1 60 30 30000". This command will scan
       continuously for 30 seconds.
    2) If a periodic advertisement is found, it will print "Periodic adv found
       addr_type [type], addr:0xXX:XX:XX:XX:XX:XX, adv_sid:[adv_sid]".
3. To establish periodic advertisement synchronization, use finsh command "diss
   sync start [addr] [addr_type] [adv_sid] [sync_to]", where addr, addr_type and
   adv_sid are the address, address type and advertisement SID printed after
   finding the periodic advertisement; sync_to represents the sync timeout in
   units of 10ms
    1) Example: "diss sync start be:56:44:33:22:c2 0 0 80"
    2) Since establishing periodic advertisement sync requires scan to be
       enabled, the 30-second scan set by the previous command may have stopped
       when establishing periodic advertisement sync. If scan has stopped, you
       can see "Scan stopped" in the log. When scan is stopped, you need to
       restart scan.
    3) After periodic advertisement sync is established, it will print
       "PER_ADV_SYNC established(addr)" and "per_adv_data:"

### Hardware Requirements
Before running this example, prepare:
+ One development board supported by this example ([Supported
  Platforms](#Platform_peri_adv_sync)).
+ Mobile device.

### menuconfig Configuration
1. Enable Bluetooth (`BLUETOOTH`):
    - Path: Sifli middleware → Bluetooth
    - Enable: Enable bluetooth
        - Macro switch: `CONFIG_BLUETOOTH`
        - Description: Enables Bluetooth functionality
2. Enable GAP Central:
    - Path: Sifli middleware → Bluetooth → Bluetooth service → BLE service
  - Path: Sifli middleware → Bluetooth → Bluetooth service → BLE service
3. Enable NVDS:
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
1. It can search for periodic advertisements and establish periodic
   advertisement synchronization.

## Troubleshooting


## Reference Documentation
<!-- 对于rt_device的示例，rt-thread官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考RT-Thread的[RTC文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Update History
| Version | Date    | Release Notes   |
| ------- | ------- | --------------- |
| 0.0.1   | 01/2025 | Initial version |
|         |         |                 |
|         |         |                 |
