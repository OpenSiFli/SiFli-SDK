# USB MTP Device Example (DFS + RAM DISK)

Source code path: example/cherryusb/device/mtp

## Supported Platforms
+ sf32lb52-lcd_n16r8_hcpu
+ sf32lb58-lcd_n16r32n1_dpi_hcpu

## Overview
This example demonstrates a CherryUSB MTP device implementation using RT-Thread DFS and POSIX APIs.
It creates a 64 KiB RAM block device (`mtp_ram`), formats it with the `elm` filesystem, and exposes it through MTP.

After connecting to a host PC, the device appears as `CherryUSB MTP DEMO`, and files can be browsed, copied, and deleted from the file manager.
By default, a sample file `cherryusb.txt` is created in the root directory.

## How to Use the Example

### Hardware Requirements
Before running this example, prepare the following:
+ A supported development board.
+ A USB cable with data transfer capability (a stable Type-C data cable is recommended).
+ A host that supports MTP (Windows/Linux/macOS).

### menuconfig Configuration
This example depends on the following key options:
+ `CONFIG_PKG_CHERRYUSB_DEVICE=y`
+ `CONFIG_PKG_CHERRYUSB_DEVICE_MTP=y`
+ `CONFIG_RT_USING_DFS_ELMFAT=y`

To adjust object count, pathname length, thread stack size, and related options, refer to:
+ `src/usb_config.h`

### Build and Flash
Switch to the example `project` directory and run scons:

> scons --board=sf32lb58-lcd_n16r32n1_dpi_hcpu -j8

Then switch to `project/build_xx` and run `uart_download.bat`. Select the serial port as prompted:

> .\uart_download.bat

For detailed build and flashing instructions, refer to [Quick Start](quick_start).

## Expected Result
After boot and USB connection, the expected behavior is:
+ The host detects an MTP device (product name: `CherryUSB MTP DEMO`).
+ The sample file `cherryusb.txt` is visible.
+ Basic file operations (read/write/delete/rename) work as expected.

## Troubleshooting
If the host cannot enumerate the device or file operations fail, check:
+ Whether `CONFIG_RT_USING_DFS_ELMFAT` is enabled. If not, errors like `File system (elm) was not found` may occur.
+ Whether the USB cable supports data transfer (charge-only cables will fail).
+ Whether MTP/DFS-related errors appear in board serial logs.

If you encounter issues, please submit an [issue](https://github.com/OpenSiFli/SiFli-SDK/issues) on GitHub.

## Reference Documentation
+ [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
+ [SiFli-SDK Development Guide](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/development/index.html)

## Changelog
| Version | Date    | Notes |
|:---|:---|:---|
| 0.0.1 | 06/2026 | Initial version with MTP (DFS + RAM Disk) example description |