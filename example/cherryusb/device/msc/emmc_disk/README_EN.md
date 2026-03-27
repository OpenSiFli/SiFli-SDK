# USB Mass Storage Device Example (EMMC)

Source Path: example\cherryusb\device\emmc_disk

## Supported Platforms
<!-- 支持哪些板子和芯片平台 -->
sf32lb52-core_e8r16

## Overview
<!-- 例程简介 -->
This example demonstrates using CherryUSB MSC with eMMC (via the SDIO interface)
to implement a virtual USB flash drive (U disk). It includes:
+ The PC can view a USB drive named "SiFli MSC DEMO" in the file manager.

## How to Use This Example
<!-- 说明如何使用例程，比如连接哪些硬件管脚观察波形，编译和烧写可以引用相关文档。
对于rt_device的例程，还需要把本例程用到的配置开关列出来，比如PWM例程用到了PWM1，需要在onchip菜单里使能PWM1 -->

### Hardware Requirements
Before running this example, you need to prepare:
+ A development board supported by this example ([Supported
  Platforms](quick_start)).
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
        - Purpose: Use SDIO as the underlying interface, so `SDIO` must be
          enabled.

### Compilation and Flashing
Switch to the project directory and run the scons command to compile:

> scons --board=sf32lb52-lcd_n16r8 -j32

Switch to the `project/build_xx` directory and run `uart_download.bat`. Follow
the prompts to select the port for downloading:

> ./uart_download.bat

> UART Download

> Please enter the serial port number: 5

For detailed steps on compilation and downloading, please refer to the [Quick
Start Guide](quick_start).

## Expected Results
<!-- 说明例程运行结果，比如哪几个灯会亮，会打印哪些log，以便用户判断例程是否正常运行，运行结果可以结合代码分步骤说明 -->
After the example starts: The host connects to the board via the data cable. The
PC can see a USB drive named "SiFli MSC DEMO" in the file manager. In Windows
Device Manager, a new device appears under Universal Serial Bus controllers: USB
Mass Storage Device.

## Troubleshooting


## Reference Documents
<!-- 对于rt_device的示例，rt-thread官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考RT-Thread的[RTC文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Revision History
| Version | Date    | Release Notes   |
| ------- | ------- | --------------- |
| 0.0.1   | 09/2025 | Initial version |
|         |         |                 |
|         |         |                 |
