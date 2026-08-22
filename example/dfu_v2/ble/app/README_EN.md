# DFU V2 BLE User Application Example

Source code path: example/dfu_v2/ble/app

(Platform_dfu_v2_ble_app)=
## Supported Platforms
<!-- Which boards and chip platforms are supported -->
- `sf32lb52-lcd_n16r8_hcpu` — SF32LB52X, 16MB NOR Flash + 8MB PSRAM.

## Overview
<!-- Example introduction -->
This example is a standard BLE peripheral application that integrates the DFU V2 middleware. It can update non-self images (resources, fonts, loader, etc.) live while running, and when its own HCPU application needs updating it switches into the separate DFU loader subprogram to finish the job.

## Usage Instructions
<!-- Explain how to use the example, such as which hardware pins to connect to observe waveforms, compilation and flashing can reference related documentation.
For rt_device examples, also list the configuration switches used in this example, for example PWM example uses PWM1, need to enable PWM1 in onchip menu -->
1. After power-up the device starts BLE advertising. A phone / host connects over BLE and pushes firmware. When pushing non-HCPU images (resources, fonts, loader, etc.), the device writes them into their partitions live while running.
2. To update the HCPU application itself, since a running application cannot erase itself, run `cmd_diss ota_reboot` from the serial console to switch into the loader and finish the upgrade.
3. The phone App is unchanged from legacy BLE DFU, so no App-side changes are required.

### Hardware Requirements
Before running this example, prepare:
+ One development board supported by this example ([Supported Platforms](#Platform_dfu_v2_ble_app)).
+ A phone or host (acting as the BLE host to push firmware).

### menuconfig Configuration
Key configuration for this example (enabled by default in proj.conf):
- `CONFIG_USING_DFU_V2=y` — enables the DFU V2 middleware.
- `CONFIG_DFU_V2_USE_BLE_TRANSPORT=y` — selects BLE as the DFU transport channel.
- `CONFIG_BLUETOOTH=y` — enables the Bluetooth stack (required for the BLE peripheral).
- `CONFIG_BSP_BLE_SERIAL_TRANSMISSION=y` — enables the BLE serial transparent transmission service, used as the DFU data channel.
- `CONFIG_BT_CON_NUM_CUSTOMIZE=y` / `CONFIG_CFG_MAX_BT_ACL_NUM=2` — customize and cap the number of Bluetooth connections.
- `CONFIG_PKG_SIFLI_MBEDTLS_BOOT=y` — brings in mbedTLS for firmware verification.

### Compilation and Flashing
Switch to the example project directory and run the scons command to compile:
```c
> scons --board=sf32lb52-lcd_n16r8_hcpu -j8
```
When building the user application, the companion same-channel loader subprogram is built automatically as a child project via `AddDFU_BLE()`.

For detailed compilation and download steps, please refer to the [Quick Start Guide](/quickstart/get-started.md).

## Expected Results
<!-- Describe the expected results of running the example, such as which LEDs will light up, what logs will be printed, to help users determine if the example is running normally. Results can be explained step by step in conjunction with the code -->
After the example starts:
1. The serial console prints the startup banner `=== peripheral_with_ota V2 Start ===`, then the DFU V2 middleware initializes and BLE advertising starts with a name like `SIFLI_APP-xx-...`.
2. During init `dfu_mode_host_set_self_img_id(0)` tells Host mode to skip the HCPU image (`img_id=0`), because a running application cannot overwrite itself.
3. When the phone / host pushes non-HCPU images (resources, fonts, loader, etc.) over BLE, the serial console prints OTA progress logs (`OTA progress`, `OTA image N complete`, `OTA all complete`).
4. To update the HCPU application, after running `cmd_diss ota_reboot` the device reboots and jumps into the loader, which receives and directly writes the new HCPU firmware, then reboots back into the new application version.
5. At any time `cmd_diss ota_info` queries the current OTA progress, printing `OTA: idle` when idle.

## Troubleshooting


## Reference Documentation
<!-- For rt_device examples, RT-Thread official documentation provides detailed explanations. Web links can be added here, for example, refer to RT-Thread's [RTC Documentation](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Update History
|Version |Date   |Release Notes |
|:---|:---|:---|
|0.0.1 |06/2026 |Initial version |
| | | |
| | | |
