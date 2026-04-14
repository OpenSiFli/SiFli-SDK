# USB Mass Storage Device Example (RAM DISK)

Source Path: example\cherryusb\device\ram_disk

## Supported Platforms
<!-- Supported boards and chip platforms -->
+ sf32lb52-lcd_n16r8

## Overview
<!-- Example introduction -->
This example demonstrates the virtualization of a USB flash drive using RAM based on cherryusb MSC, including:
+ The PC can view a USB flash drive named SiFli MSC DEMO through the file manager.

## How to Use This Example
<!-- Instructions on how to use the example, such as which hardware pins to connect to observe waveforms, compilation and programming can refer to related documents.
For rt_device examples, also list the configuration switches used in this example, e.g., the PWM example uses PWM1, which needs to be enabled in the onchip menu. -->

### Hardware Requirements
Before running this example, prepare:
+ A development board supported by this example ([Supported Platforms](quick_start)).
+ A USB-A to Type-C data cable with data transfer capability.
+ A host device that supports USB.

### menuconfig Configuration


### Compilation and Programming
Switch to the example project directory and run the scons command to compile:

> scons --board=sf32lb52-lcd_n16r8 -j32

Switch to the example `project/build_xx` directory, run `uart_download.bat`, and follow the prompts to select the port for downloading:

 >./uart_download.bat

>Uart Download

>please input the serial port num:5

For detailed steps on compilation and downloading, please refer to the relevant introduction in [Quick Start](quick_start).

## Expected Results
<!-- Describe the expected results of running the example, such as which LEDs will light up, what logs will be printed, so users can judge whether the example is running normally. The results can be explained step by step in combination with the code. -->
After the example starts:
The host connects to the board via the data cable, and the PC can view a USB flash drive named SiFli MSC in the file manager. There is a README.TXT file in the drive with the content "cherryusb device msc_ram demo!".

## Troubleshooting


## Reference Documents
<!-- For rt_device examples, you can add web links to detailed instructions provided by the RT-Thread official documentation, e.g., refer to RT-Thread's [RTC documentation](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Change Log
|Version |Date   |Release Notes |
|:---|:---|:---|
|0.0.1 |09/2025 |Initial version |
| | | |
| | | |