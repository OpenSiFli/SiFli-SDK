# DFU V2 PAN Loader Example

Source code path: example/dfu_v2/bt_pan/loader

(Platform_dfu_v2_bt_pan_loader)=
## Supported Platforms
<!-- Which boards and chip platforms are supported -->
This loader does not keep its own partition table; it is compiled together with the board project of the companion PAN user application. The current PAN user-application project covers these SF32LB52X (NOR flash) boards:

- `sf32lb52-lcd_n16r8_hcpu`
- `sf32lb52-lchspi-ulp_hcpu`
- `sf32lb52-nano_a128r16_hcpu`

## Overview
<!-- Example introduction -->
This example is the DFU V2 loader subprogram for the Bluetooth PAN channel. It pulls the new firmware over the phone-shared PAN network, writes it directly into the target partition, and reboots back into the new user application.

## Usage Instructions
<!-- Explain how to use the example, such as which hardware pins to connect to observe waveforms, compilation and flashing can reference related documentation.
For rt_device examples, also list the configuration switches used in this example, for example PWM example uses PWM1, need to enable PWM1 in onchip menu -->
This loader normally does not need a separate build. The companion PAN user-application project pulls it in as a child project through `AddDFU_PAN_V2(SIFLI_SDK)` in its `SConstruct`, so building the app also produces the loader firmware. The usual flow is to build from the PAN user-application project directory; it can also be built standalone from this loader project directory.

### Hardware Requirements
Before running this example, prepare:
+ One development board supported by this example ([Supported Platforms](#Platform_dfu_v2_bt_pan_loader)).
+ A phone that provides a PAN network.

### menuconfig Configuration
Key configuration for this example (enabled by default in proj.conf):

- Macro switch: `CONFIG_USING_DFU_V2`
    - Description: Enable the DFU V2 middleware.
- Macro switch: `CONFIG_DFU_V2_USE_DEVICE_MODE`
    - Description: Run in device (target / updated side) mode.
- Macro switch: `CONFIG_DFU_V2_USE_PAN_TRANSPORT`
    - Description: Use PAN as the firmware transport channel.
- Macro switch: `CONFIG_BLUETOOTH`
    - Description: Enable Bluetooth functionality.
- Macro switch: `CONFIG_CFG_PAN`
    - Description: Enable the PAN profile.
- Macro switch: `CONFIG_CFG_HID`
    - Description: Enable the HID profile.
- Macro switch: `CONFIG_BT_AUTO_CONNECT_LAST_DEVICE`
    - Description: Automatically reconnect to the last connected device.
- Macro switch: `CONFIG_RT_USING_LWIP`
    - Description: Enable the LWIP network stack to fetch firmware over the PAN network.
- Macro switch: `CONFIG_LWIP_ALTCP_TLS`
    - Description: Enable LWIP ALTCP TLS support.
- Macro switch: `CONFIG_PKG_USING_MBEDTLS`
    - Description: Enable the mbedTLS crypto library.
- Macro switch: `CONFIG_PKG_USING_WEBCLIENT`
    - Description: Enable webclient to download firmware over the PAN network.
- Macro switch: `CONFIG_PKG_USING_LITTLEVGL2RTT`
    - Description: Enable the LVGL upgrade UI.
- Macro switch: `CONFIG_LVGL_V9`
    - Description: Use LVGL V9 to draw the upgrade UI.

### Compilation and Flashing
Switch to the PAN user-application project directory and run the scons command to compile; building the app automatically includes this loader:
```c
> scons --board=sf32lb52-lcd_n16r8_hcpu -j8
```
It can also be built standalone from this loader project directory:
```c
> scons --board=sf32lb52-lcd_n16r8_hcpu -j8
```
For detailed compilation and download steps, please refer to the [Quick Start Guide](/quickstart/get-started.md).

## Expected Results
<!-- Describe the expected results of running the example, such as which LEDs will light up, what logs will be printed, to help users determine if the example is running normally. Results can be explained step by step in conjunction with the code -->
After the example starts:
1. The local name is set to `sifli-pan`, and the loader waits for the phone to connect.
2. After the phone connects and joins the PAN network, the auto-update routine `execute_ota_update()` is triggered: it runs a DNS check for `ota.sifli.com` → reads the firmware info that the user application pre-wrote at `DFU_FWINFO_BASE_ADDR` in flash → calls the blocking `dfu_download()`.
3. On success it clears the firmware info in flash, the UI shows the progress, then it reboots into the new user application.

For debugging, the finsh command `ota_cmd` is available with subcommands: `del_bond`, `conn_pan`, `download`, `print`, `clear`, `test_flags`, `test_dns`.

## Troubleshooting


## Reference Documentation
<!-- For rt_device examples, RT-Thread official documentation provides detailed explanations. Web links can be added here, for example, refer to RT-Thread's [RTC Documentation](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Update History
|Version |Date   |Release Notes |
|:---|:---|:---|
|0.0.1 |06/2026 |Initial version |
| | | |
| | | |
