# BLE Periodic Advertisement Sync Example

Source code path: example/ble/periodic_adv_sync

(Platform_peri_adv_sync)=
## Supported Platforms
<!-- Which boards and chip platforms are supported -->
+ eh-lb52x
+ eh-lb56x
+ eh-lb58x

## Overview
<!-- Example introduction -->
This example demonstrates the usage of periodic advertisement synchronization.


## Usage Instructions
<!-- Explain how to use the example, such as which hardware pins to connect to observe waveforms, compilation and flashing can reference related documentation.
For rt_device examples, also list the configuration switches used in this example, for example PWM example uses PWM1, need to enable PWM1 in onchip menu -->
1. After boot, use finsh command "diss sync create" to create periodic advertisement sync. If created successfully, you can see "PER_ADV_SYNC created [idx]" in the log, where idx is the allocated handle.
2. Use finsh command "diss scan start [is_dup] [scan_interval] [scan_window] [duration]" to start scan to search for periodic advertisements.
    1) Command example: "diss scan start 1 60 30 30000". This command will scan continuously for 30 seconds.
    2) If a periodic advertisement is found, it will print "Periodic adv found addr_type [type], addr:0xXX:XX:XX:XX:XX:XX, adv_sid:[adv_sid]".
3. To establish periodic advertisement synchronization, use finsh command "diss sync start [addr] [addr_type] [adv_sid] [sync_to]", where addr, addr_type and adv_sid are the address, address type and advertisement SID printed after finding the periodic advertisement;
   sync_to represents the sync timeout in units of 10ms
    1) Example: "diss sync start be:56:44:33:22:c2 0 0 80"
    2) Since establishing periodic advertisement sync requires scan to be enabled, the 30-second scan set by the previous command may have stopped when establishing periodic advertisement sync. If scan has stopped, you can see "Scan stopped" in the log. When scan is stopped, you need to restart scan.
    3) After periodic advertisement sync is established, it will print "PER_ADV_SYNC established(addr)" and "per_adv_data:"



### Hardware Requirements
Before running this example, prepare:
+ One development board supported by this example ([Supported Platforms](#Platform_peri_adv_sync)).
+ Mobile device.

### menuconfig Configuration
1. Enable Bluetooth (`BLUETOOTH`):\
![BLUETOOTH](./assets/bluetooth.png)
2. Enable GAP Central: \
![GAP_CENTRAL](./assets/gap_central.png)
3. Enable NVDS:\
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
1. It can search for periodic advertisements and establish periodic advertisement synchronization.


## Troubleshooting


## Reference Documentation
<!-- For rt_device examples, RT-Thread official documentation provides detailed explanations. Web links can be added here, for example, refer to RT-Thread's [RTC Documentation](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Update History
|Version |Date   |Release Notes |
|:---|:---|:---|
|0.0.1 |01/2025 |Initial version |
| | | |
| | | |