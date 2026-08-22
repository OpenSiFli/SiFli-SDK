# DFU V2 CDC Loader Example

Source code path: example/dfu_v2/cdc/loader

(Platform_dfu_v2_cdc_loader)=
## Supported Platforms
<!-- Which boards and chip platforms are supported -->
- `sf32lb52-lcd_n16r8_hcpu` — SF32LB52X, 16MB NOR flash (`n16r8`).

## Overview
<!-- Example introduction -->
This example is the DFU loader subprogram for the USB-CDC channel of DFU V2. It is flashed independently into the `DFU_V2_LOADER` partition; after the bootloader detects an update flag it jumps in here to receive the new firmware over USB-CDC, write it directly to the target partition, and optionally show the upgrade progress on screen.

## Usage Instructions
<!-- Explain how to use the example, such as which hardware pins to connect to observe waveforms, compilation and flashing can reference related documentation.
For rt_device examples, also list the configuration switches used in this example, for example PWM example uses PWM1, need to enable PWM1 in onchip menu -->
This loader usually does not need a separate build: the CDC user application's `SConstruct` pulls it in as a child project via `AddDFU_CDC(SIFLI_SDK)` (see `tools/build/building.py`), so building the user application also produces the loader firmware. It can also be built standalone.

### Hardware Requirements
Before running this example, prepare:
+ One development board supported by this example ([Supported Platforms](#Platform_dfu_v2_cdc_loader)).

### menuconfig Configuration
Key configuration of this example (enabled by default in proj.conf):
- `CONFIG_USING_DFU_V2=y` — enables the DFU V2 middleware.
- `CONFIG_DFU_V2_USE_CDC_TRANSPORT=y` — selects USB-CDC as the DFU transport.
- `CONFIG_PKG_USING_CHERRYUSB=y` / `CONFIG_PKG_CHERRYUSB_DEVICE=y` — enables the CherryUSB device stack.
- `CONFIG_PKG_CHERRYUSB_DEVICE_MUSB_SIFLI=y` — SiFli MUSB USB device controller driver.
- `CONFIG_PKG_CHERRYUSB_DEVICE_CDC_ACM=y` — CDC ACM device class.
- `CONFIG_PKG_USING_LITTLEVGL2RTT=y` / `CONFIG_LVGL_V9=y` — enables the LVGL (v9) upgrade UI; with these off the loader still performs the upgrade, just without the UI.
- `CONFIG_PKG_SIFLI_MBEDTLS_BOOT=y` — mbedTLS support needed for boot-time firmware verification.

### Compilation and Flashing
To build the CDC user application (the loader is included automatically), switch to the user application project directory and run the scons command to compile:
```c
> scons --board=sf32lb52-lcd_n16r8_hcpu -j8
```
To build this loader standalone, switch to this example project directory and run the scons command to compile:
```c
> scons --board=sf32lb52-lcd_n16r8_hcpu -j8
```
Artifacts are placed under the matching `build_<board>` directory. For detailed compilation and download steps, please refer to the [Quick Start Guide](/quickstart/get-started.md).

## Expected Results
<!-- Describe the expected results of running the example, such as which LEDs will light up, what logs will be printed, to help users determine if the example is running normally. Results can be explained step by step in conjunction with the code -->
1. After flashing, the board runs the user application (the screen shows `Ver: 1.0.9`).
2. The PC sends the 4-byte trigger magic `0xAA 0x55 0xDF 0x00` to the user application over USB-CDC; the app replies with the ACK `0xAA 0x55 0xDF 0x01`, then calls `dfu_enter_dfu_mode()` to write the update flag and reboot.
3. After the reboot the bootloader detects the update flag and jumps into this loader (`DFU_V2_LOADER` partition). In `main()` the loader first calls `dfu_init()` to bring up the middleware, then `dfu_check_install()` to handle any pending install, and waits for the USB-CDC connection.
4. The PC tool connects and streams the new firmware to the loader in chunks. The loader receives it via `dfu_process()` and writes it directly to the target partition; the `on_dfu_event()` callback feeds progress to the LVGL UI so the screen shows the percentage, and a success popup appears when all images are complete.
5. The loader finishes the install and reboots; the bootloader jumps back into the new user application. The changed version label on the screen confirms a successful upgrade.

For debugging, the `cdc_dfu_info` console command reports USB readiness and the current transfer progress.

## Troubleshooting


## Reference Documentation
<!-- For rt_device examples, RT-Thread official documentation provides detailed explanations. Web links can be added here, for example, refer to RT-Thread's [RTC Documentation](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Update History
|Version |Date   |Release Notes |
|:---|:---|:---|
|0.0.1 |06/2026 |Initial version |
| | | |
| | | |
