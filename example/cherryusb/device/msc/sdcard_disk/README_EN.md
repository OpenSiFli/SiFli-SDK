# USB Mass Storage Device Example (SD Card)

Source Path: example\cherryusb\device\sdcard_disk

## Supported Platforms
<!-- List supported boards and chip platforms -->
+ sf32lb52-lcd_n16r8

## Overview
<!-- Brief introduction of the example -->
This example demonstrates implementing a virtual USB flash drive (MSC) using CherryUSB MSC with an SD card over SPI. It includes:
+ The PC can see a USB drive named "SiFli MSC DEMO" in the file manager.

## How to Use This Example
<!-- Explain how to use the example, such as which hardware pins to connect, how to compile and flash, and reference related documents.
For rt_device examples, also list the configuration switches used, e.g., if PWM1 is used, enable PWM1 in the onchip menu. -->

### Hardware Requirements
Before running this example, prepare:
+ A development board supported by this example ([Supported Platforms](quick_start)).
+ A USB-A to Type-C data cable with data transfer capability.
+ A USB-capable host (e.g. PC).

### menuconfig Configuration
1. Enable SPI1:
	 - Path: On-chip Peripheral RTOS Drivers → Enable SPI BUS
	 - Enable: Enable SPI1 BUS
		 - Macro: `BSP_USING_SPI1`
		 - Purpose: Use SPI1 as the SD card interface
	 - (Optional) Enable: Enable SPI1 TX DMA
		 - Macro: `BSP_SPI1_TX_USING_DMA`
		 - Purpose: Enable SPI TX DMA
	 - (Optional) Enable: Enable SPI1 RX DMA
		 - Macro: `BSP_SPI1_RX_USING_DMA`
		 - Purpose: Enable SPI RX DMA
2. Enable MSD (SD card over SPI) driver:
	 - Path: RTOS → RT-Thread Components → Device Drivers
	 - Enable: Using SD/TF card driver with spi
		 - Macro: `RT_USING_SPI_MSD`
		 - Purpose: Use SPI for SD (TF) card driver, required for MSC storage backend

### Compilation and Flashing
Switch to the example `project` directory and run the scons command to build:

> scons --board=sf32lb52-lcd_n16r8 -j32

Enter the example `project/build_xx` directory (actual build output folder) and run `uart_download.bat`, then follow the prompt to choose the serial port for download:

> ./uart_download.bat

> Uart Download

> please input the serial port num:5

For detailed compilation and download steps, refer to the [Quick Start Guide](quick_start).

## Expected Results
After startup:
The host connects to the board via the USB data cable. The PC file manager shows a USB drive named "SiFli MSC DEMO". In Device Manager under Universal Serial Bus controllers, a new device "USB Mass Storage Device" appears.

## Troubleshooting
TBD.

## Reference Documents
TBD.

## Revision History
|Version |Date   |Release Notes |
|:---|:---|:---|
|0.0.1 |09/2025 |Initial version |
| | | |
| | | |