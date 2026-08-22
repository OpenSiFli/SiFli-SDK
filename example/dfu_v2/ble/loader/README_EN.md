# DFU V2 BLE Loader Example

Source code path: example/dfu_v2/ble/loader

(Platform_dfu_v2_ble_loader)=
## Supported Platforms
<!-- Which boards and chip platforms are supported -->
The loader project does not hard-code a board; the target board is chosen at build time via `--board`, matching the companion BLE user application (verified on `sf32lb52-lcd_n16r8_hcpu`). Other SF32LB52X boards that provide a matching partition table work as well.

## Overview
<!-- Example introduction -->
This example is the DFU loader subprogram used when DFU V2 upgrades firmware over the BLE channel. It is flashed in and runs independently from the `DFU_V2_LOADER` partition, receives the new firmware pushed by a phone over BLE, and writes it directly into the target partition — it is the program that actually performs the erase/flash.

## Usage Instructions
<!-- Explain how to use the example, such as which hardware pins to connect to observe waveforms, compilation and flashing can reference related documentation.
For rt_device examples, also list the configuration switches used in this example, for example PWM example uses PWM1, need to enable PWM1 in onchip menu -->
You normally do not build this loader on its own. Through `AddDFU_BLE()` in `building.py`, the BLE user application's `SConstruct` pulls it in automatically as a child project via `AddChildProj`. In other words, building the application as usual also compiles the loader. To debug it in isolation, you can also enter this project directory and build it standalone.

### Hardware Requirements
Before running this example, prepare:
+ One development board supported by this example ([Supported Platforms](#Platform_dfu_v2_ble_loader)).
+ A phone with BLE and the companion upgrade app.

### menuconfig Configuration
Key configuration for this example (already enabled by default in proj.conf):
- Macro switch: `CONFIG_USING_DFU_V2`
    - Description: enable the DFU V2 middleware.
- Macro switch: `CONFIG_DFU_V2_USE_HOST_MODE`
    - Description: enable Host mode, where this side drives receiving and writing the firmware.
- Macro switch: `CONFIG_DFU_V2_USE_BLE_TRANSPORT`
    - Description: select BLE as the transport channel.
- Macro switch: `CONFIG_BLUETOOTH`
    - Description: enable the Bluetooth stack.
- Macro switch: `CONFIG_BSP_BLE_SERIAL_TRANSMISSION`
    - Description: enable the BLE serial transmission service used as the DFU data channel.
- Macro switch: `CONFIG_OTA_55X`
    - Description: enable OTA image layout support.
- Macro switch: `CONFIG_PKG_SIFLI_MBEDTLS_BOOT`, `CONFIG_PKG_USING_FLASHDB`
    - Description: dependency components required for DFU verification and storage.

### Compilation and Flashing
Switch to the BLE user application project directory and run the scons command to compile; building the application automatically includes this loader:
```c
> scons --board=sf32lb52-lcd_n16r8_hcpu -j8
```
To build the loader standalone (e.g. for isolated debugging), enter this project directory:
```c
> scons --board=sf32lb52-lcd_n16r8_hcpu -j8
```
Artifacts are generated under `build_<board_name>`. For detailed compilation and download steps, please refer to the [Quick Start Guide](/quickstart/get-started.md).

## Expected Results
<!-- Describe the expected results of running the example, such as which LEDs will light up, what logs will be printed, to help users determine if the example is running normally. Results can be explained step by step in conjunction with the code -->
1. After entering BLE upgrade mode, the device starts advertising with a device name like `SIFLI_DFU-xx-xx-xx-xx-xx-xx`, where xx represents the device's Bluetooth address, and waits for a phone to connect.
2. Once the phone app connects, it pushes the firmware over the BLE serial transmission service; the loader receives it and writes it directly to the target partition. During the transfer it reports the progress percentage, and the progress bar is shown when the UI is enabled.
3. To avoid getting stuck in upgrade mode, the loader has two built-in timeouts: a 5-minute global timeout (`DFU_MODE_TIMEOUT_MS`, the overall budget for the whole upgrade mode) and a 60-second inactivity timeout after connection (`DFU_INACTIVITY_TIMEOUT_MS`, reset on every progress message once BLE is connected). If either fires before the upgrade completes, the device reboots back into the user application.
4. After all firmware is written, the loader waits 3 seconds (`REBOOT_DELAY_MS`, to let the success popup display) and then reboots back into the new user application.
5. For debugging, type `dfu_ble_info` in the loader console to see BLE power-on status, MTU, and current progress.

## Troubleshooting


## Reference Documentation
<!-- For rt_device examples, RT-Thread official documentation provides detailed explanations. Web links can be added here, for example, refer to RT-Thread's [RTC Documentation](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Update History
|Version |Date   |Release Notes |
|:---|:---|:---|
|0.0.1 |06/2026 |Initial version |
| | | |
| | | |
