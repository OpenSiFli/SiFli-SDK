# USB Mass Storage Device Example (EMMC)

Source Path: example\cherryusb\device\emmc_disk

## Supported Platforms
<!-- List supported boards and chip platforms -->
+ sf32lb52-lcd_n16r8

## Overview
<!-- Brief introduction of the example -->
This example demonstrates using CherryUSB MSC with eMMC (via the SDIO interface) to implement a virtual USB flash drive (U disk). It includes:
+ The PC can view a USB drive named "SiFli MSC DEMO" in the file manager.

## How to Use This Example
<!-- Explain how to use the example, such as which hardware pins to connect, how to compile and flash, and reference related documents.
For rt_device examples, also list the configuration switches used, e.g., if PWM1 is used, enable PWM1 in the onchip menu. -->

### Hardware Requirements
Before running this example, you need to prepare:
+ A development board supported by this example ([Supported Platforms](quick_start)).
+ A USB-A to Type-C data cable with data transfer capability.
+ A host device that supports USB.

### menuconfig Configuration
1. Enable SDIO:
	- Path: On-chip Peripheral RTOS Drivers → Enable SPI BUS
	- Enable: Enable SDIO
		- Macro: `BSP_USING_SDIO`
		- Purpose: Enable the SDIO driver.
2. Enable and configure SD device:
	- Path: RTOS → RT-Thread Components → Device Drivers
	- Enable: Using SD/MMC device drivers
		- Macro: `RT_USING_SDIO`
		- Purpose: Use SDIO as the underlying interface, so `SDIO` must be enabled.


### Compilation and Flashing
Switch to the project directory and run the scons command to compile:

> scons --board=sf32lb52-lcd_n16r8 -j32

Switch to the `project/build_xx` directory and run `uart_download.bat`. Follow the prompts to select the port for downloading:

 >./uart_download.bat

>Uart Download

>please input the serial port num:5

For detailed steps on compilation and downloading, please refer to the [Quick Start Guide](quick_start).

## Expected Results
<!-- Describe the expected results, such as which LEDs will light up, what logs will be printed, so that users can judge whether the example is running normally. You can also explain the results step by step with the code. -->
After the example starts:
The host connects to the board via the data cable. The PC can see a USB drive named "SiFli MSC DEMO" in the file manager. In Windows Device Manager, a new device appears under Universal Serial Bus controllers: USB Mass Storage Device.

## Troubleshooting


## Reference Documents
<!-- For rt_device examples, you can add links to detailed documentation on the RT-Thread official website, e.g., refer to RT-Thread's [RTC documentation](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Revision History
|Version |Date   |Release Notes |
|:---|:---|:---|
|0.0.1 |09/2025 |Initial version |
| | | |
| | | |