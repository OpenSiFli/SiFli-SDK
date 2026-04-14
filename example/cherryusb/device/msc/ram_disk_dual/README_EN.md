# USB Mass Storage Device Example (RAM DISK Dual LUN)

Source Path: example\cherryusb\device\ram_disk_dual

## Supported Platforms
<!-- Supported boards and chip platforms -->
+ sf32lb52-lcd_n16r8

## Overview
<!-- Example introduction -->
This example demonstrates the implementation of a virtual U disk based on cherryusb MSC, including:
+ The PC can see two USB drives named SiFli MSC1 and SiFli MSC2 in the file manager.

## How to Use This Example
<!-- Instructions on how to use the example, such as which hardware pins to connect to observe waveforms, compilation and programming can refer to related documents.
For rt_device examples, also list the configuration switches used in this example. For example, if the PWM example uses PWM1, you need to enable PWM1 in the onchip menu. -->

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
The host connects to the board via the data cable, and the PC can see two USB drives in the file manager:
+ One named SiFli MSC1, whose README.TXT contains: cherryusb device msc_ram_dual demo! Lun: 1.
+ One named SiFli MSC2, whose README.TXT contains: cherryusb device msc_ram_dual demo! Lun: 2.

## Troubleshooting


## Reference Documents
<!-- For rt_device examples, you can add web links to detailed instructions provided by the RT-Thread official documentation, e.g., refer to RT-Thread's [RTC documentation](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Change Log
|Version |Date   |Release Notes |
|:---|:---|:---|
|0.0.1 |09/2025 |Initial version |
| | | |
| | | |