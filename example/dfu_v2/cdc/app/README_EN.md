# DFU V2 CDC User Application Example

Source code path: example/dfu_v2/cdc/app

(Platform_dfu_v2_cdc_app)=
## Supported Platforms
<!-- Which boards and chip platforms are supported -->
- `sf32lb52-lcd_n16r8_hcpu` — SF32LB52X chip, NOR Flash (16Mb NOR + 8MB PSRAM).

## Overview
<!-- Example introduction -->
This example demonstrates the user application side of DFU V2 firmware update over the USB-CDC (virtual serial) channel. The device enumerates as a CDC ACM serial port and shows its version on screen during normal operation; when the PC tool sends an upgrade trigger over that port, the device acknowledges and reboots into the DFU loader to perform the flashing.
The companion PC-side tool is `dfu_v2_cdc_tool.py`, which first sends the trigger to put the device into DFU mode, then uploads firmware over USB-CDC.

## Usage Instructions
<!-- Explain how to use the example, such as which hardware pins to connect to observe waveforms, compilation and flashing can reference related documentation.
For rt_device examples, also list the configuration switches used in this example, for example PWM example uses PWM1, need to enable PWM1 in onchip menu -->
After flashing the build artifacts to the device, the device runs normally and enumerates as a USB-CDC serial port showing its version. Prepare a new-version firmware, run `dfu_v2_cdc_tool.py` on the PC with `--trigger` to start the upgrade, and the device reboots into the loader to complete the flashing before returning to the new user application.

### Hardware Requirements
Before running this example, prepare:
+ One development board supported by this example ([Supported Platforms](#Platform_dfu_v2_cdc_app)).
+ A USB data cable to connect the development board to the PC.

### menuconfig Configuration
Key configuration for this example (enabled by default in proj.conf):
- `CONFIG_USING_DFU_V2=y` — Enable the DFU V2 middleware.
- `CONFIG_DFU_V2_USE_CDC_TRANSPORT=y` — Select USB-CDC as the DFU transport channel.
- `CONFIG_PKG_USING_CHERRYUSB=y` / `CONFIG_PKG_CHERRYUSB_DEVICE=y` — Enable the CherryUSB device stack.
- `CONFIG_PKG_CHERRYUSB_DEVICE_MUSB_SIFLI=y` — Use the SiFli MUSB device controller.
- `CONFIG_PKG_CHERRYUSB_DEVICE_CDC_ACM=y` — Enable the CDC ACM class (virtual serial port).
- `CONFIG_PKG_USING_LITTLEVGL2RTT=y` / `CONFIG_LVGL_V9=y` — Enable LVGL (v9) for the version-number UI.
- `CONFIG_RT_USING_MEMHEAP=y` — RT-Thread memheap required by the LVGL memory adapter.

### Compilation and Flashing
Switch to the example project directory and run the scons command to compile:
```c
> scons --board=sf32lb52-lcd_n16r8_hcpu -j8
```
Building the application automatically pulls in the sibling loader as a child project via `AddDFU_CDC` in building.py, so the loader does not need to be built separately.
For detailed compilation and download steps, please refer to the [Quick Start Guide](/quickstart/get-started.md).

## Expected Results
<!-- Describe the expected results of running the example, such as which LEDs will light up, what logs will be printed, to help users determine if the example is running normally. Results can be explained step by step in conjunction with the code -->
1. Flash the build artifacts to the device and reset. The device enumerates as a USB-CDC serial port and the screen shows the title `CDC DFU EXAMPLE`, the brand `SIFLI`, and the version `Ver: 1.0.9` (taken from `APP_VERSION` in `main.c`).
2. Prepare a "new version" firmware to verify the upgrade: change `APP_VERSION` in `src/main.c` to a new value (e.g. `1.1.0`) and rebuild to get `project/build_sf32lb52-lcd_n16r8_hcpu/main.bin`. Then on the PC, run the update tool with the serial port, this new firmware and the target partition address, adding `--trigger` so the tool sends the trigger first:

   ```
   python dfu_v2_cdc_tool.py --port COM3 --firmware project/build_sf32lb52-lcd_n16r8_hcpu/main.bin --addr 0x12218000 --trigger
   ```

   Here `0x12218000` is the `HCPU_FLASH_CODE` partition address (partition-table base `0x12000000` plus offset `0x00218000`).
3. Trigger flow: the tool sends the 4-byte magic `AA 55 DF 00` to the device; the `usbd_cdc_acm_bulk_out` callback in `main.c` recognizes the magic, replies with `AA 55 DF 01`, and sets `g_dfu_trigger_requested`. The main loop sees the flag and calls `dfu_enter_dfu_mode()`, which writes the update flag and reboots.
4. After the reboot the bootloader detects the update flag and jumps into the DFU loader. The tool reconnects automatically, uploads the firmware over the DFU V2 protocol, writes it directly to the target partition, and then resets the device.
5. The device reboots back into the new user application. The `Ver:` label on screen changes to the value from the new firmware, confirming the upgrade succeeded.

## Troubleshooting


## Reference Documentation
<!-- For rt_device examples, RT-Thread official documentation provides detailed explanations. Web links can be added here, for example, refer to RT-Thread's [RTC Documentation](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Update History
|Version |Date   |Release Notes |
|:---|:---|:---|
|0.0.1 |06/2026 |Initial version |
| | | |
| | | |
