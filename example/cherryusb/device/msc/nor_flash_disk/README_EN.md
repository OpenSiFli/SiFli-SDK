
# USB Mass Storage Device Example (NOR FLASH)

Source Path: example\cherryusb\device\nor_flash_disk

## Supported Platforms
<!-- Supported boards and chip platforms -->
+ sf32lb52-lcd_n16r8

## Overview
This example demonstrates implementing a virtual USB mass storage (U disk) device using CherryUSB MSC with NOR Flash as the backend storage. It includes:
+ A USB drive named "SiFli MSC DEMO" appears in the PC file manager.

## How to Use the Example
<!-- Describe how to use the example. For rt_device based samples, list required config switches if any. -->

### Hardware Requirements
Prepare the following before running the example:
+ A supported development board ([Supported Platforms](quick_start)).
+ A USB-A to Type-C data cable that supports data transfer.
+ A USB-capable host (e.g., PC).

### menuconfig Configuration
Configuration items follow the default settings for this example unless special customization is required. (If additional configuration options are needed, list them here.)

### Build and Flash
In the example `project` directory, run the following `scons` command to build:

> scons --board=sf32lb52-lcd_n16r8 -j32

Then switch to the generated `project/build_xx` directory, run `uart_download.bat`, and follow the prompt to select the serial port for downloading:

> .\uart_download.bat

Example interaction:
> Uart Download

> please input the serial port num:5

For detailed build and download steps, refer to the [Quick Start Guide](quick_start).

## Expected Results
After startup:
1. Connect the board to the host using the USB data cable.
2. The PC file manager shows a removable drive named "SiFli MSC DEMO".
3. In the OS Device Manager (under Universal Serial Bus controllers) a new "USB Mass Storage Device" entry appears.

## Troubleshooting
| Issue | Possible Cause | Suggested Action |
|-------|----------------|------------------|
| USB disk not enumerated | Cable is power-only | Use a proper data cable |
| Drive name not shown / incorrect | Descriptor not updated | Check product string in USB descriptor configuration |
| Flash read/write errors | NOR Flash init failure | Verify Flash wiring and initialization sequence |

## Reference Documents
Add related RT-Thread or CherryUSB documentation links here if needed.

## Revision History
| Version | Date   | Release Notes |
|:-------|:-------|:--------------|
| 0.0.1  | 09/2025 | Initial version |
|        |         |                |
|        |         |                |