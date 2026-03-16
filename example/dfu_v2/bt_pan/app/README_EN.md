# DFU V2 PAN User Application Example

Source code path: example/dfu_v2/bt_pan/app

(Platform_dfu_v2_bt_pan_app)=
## Supported Platforms
<!-- Which boards and chip platforms are supported -->
- sf32lb52-lcd_n16r8_hcpu
- sf32lb52-lchspi-ulp_hcpu
- sf32lb52-nano_a128r16_hcpu

## Overview
<!-- Example introduction -->
This example demonstrates the user application side of the PAN (Bluetooth PAN network / HTTP) upgrade channel of DFU V2: the device gets online through a Bluetooth PAN link, actively queries an OTA server for the version, writes the pending firmware info to flash, then reboots and hands off to the loader subprogram for the actual download and installation.

## Usage Instructions
<!-- Explain how to use the example, such as which hardware pins to connect to observe waveforms, compilation and flashing can reference related documentation.
For rt_device examples, also list the configuration switches used in this example, for example PWM example uses PWM1, need to enable PWM1 in onchip menu -->
1. After flashing, the application brings up the Bluetooth stack, sets the local name to `sifli_pan` (or `BT_DEVICE_NAME`), and enables HID/PAN auto-reconnect. First pair the device with a phone so the device can get online through the phone's PAN network sharing; once HID is connected the application automatically initiates the PAN connection, and the serial log line `PAN connected` indicates the network channel is ready.
2. Drive the upgrade flow from the serial finsh console:
    1) `ota_version` — Show the current firmware version.
    2) `ota_check` — Register the device with the OTA server (`https://ota.sifli.com`) and query the latest version. If the server reports a newer version, the firmware list is parsed and written to flash.
    3) `ota_print` — Optional; print the firmware info stored in flash (name, URL, address, size, CRC32, etc.).
    4) `ota_go` — After confirming pending entries exist in flash, write the update flags, wait 2 seconds, then reboot.
3. Helper commands: `ota_clear` wipes the firmware info from flash; `pan_cmd` provides PAN debug subcommands such as `del_bond`, `conn_pan`, and `autoconnect`.

### Hardware Requirements
Before running this example, prepare:
+ One development board supported by this example ([Supported Platforms](#Platform_dfu_v2_bt_pan_app)).
+ A phone that can provide PAN network sharing.

### menuconfig Configuration
Key configuration of this example (enabled by default in proj.conf):
- `CONFIG_USING_DFU_V2=y` — Enables the DFU V2 middleware, providing the `dfu_fwinfo_*` firmware-info API.
- `CONFIG_RT_USING_BLUETOOTH=y`, `CONFIG_BLUETOOTH=y` — Enable the Bluetooth stack.
- `CONFIG_CFG_PAN=y`, `CONFIG_CFG_HID=y` — Enable PAN and HID; PAN provides the network channel, HID is used to trigger the PAN connection.
- `CONFIG_BT_PROFILE_CUSTOMIZE=y`, `CONFIG_BT_AUTO_CONNECT_LAST_DEVICE=y` — Customize the profile set and auto-reconnect the last device.
- `CONFIG_RT_USING_LWIP=y`, `CONFIG_RT_USING_LWIP212=y`, `CONFIG_LWIP_ALTCP=y`, `CONFIG_LWIP_ALTCP_TLS=y` — Enable the lwIP stack with TLS support for HTTPS downloads.
- `CONFIG_PKG_USING_WEBCLIENT=y`, `CONFIG_PKG_USING_CJSON=y`, `CONFIG_PKG_USING_MBEDTLS=y` — HTTP client, JSON parser, and TLS library for version query and firmware-info parsing.
- `CONFIG_BT_FINSH=y` — Enables the finsh console so the upgrade can be driven by commands.

### Compilation and Flashing
Switch to the example project directory and run the scons command to compile (`<board>` is one of the three boards above):
```c
> scons --board=<board> -j8
```
For example:
```c
> scons --board=sf32lb52-lcd_n16r8_hcpu -j8
```
Building the user application automatically pulls in the sibling loader subprogram as a child project via `AddDFU_PAN_V2`.
For detailed compilation and download steps, please refer to the [Quick Start Guide](/quickstart/get-started.md).

## Expected Results
<!-- Describe the expected results of running the example, such as which LEDs will light up, what logs will be printed, to help users determine if the example is running normally. Results can be explained step by step in conjunction with the code -->
After the example starts:
1. The application sets the local name to `sifli_pan`; after pairing with a phone it automatically initiates the PAN connection, the serial log shows `PAN connected`, and the network channel is ready.
2. Run `ota_check`; if the server has a newer version, the firmware info is written to flash.
3. After running `ota_go` the device reboots and hands off to the loader, which downloads the firmware over the PAN network, writes it directly to the target partition, then reboots back into the new user application.
4. Run `ota_version` again and see the version change to confirm a successful upgrade.

## Troubleshooting


## Reference Documentation
<!-- For rt_device examples, RT-Thread official documentation provides detailed explanations. Web links can be added here, for example, refer to RT-Thread's [RTC Documentation](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Update History
|Version |Date   |Release Notes |
|:---|:---|:---|
|0.0.1 |06/2026 |Initial version |
