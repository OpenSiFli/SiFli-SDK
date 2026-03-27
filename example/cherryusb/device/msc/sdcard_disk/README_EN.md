# USB Mass Storage Device Example (SD Card)

Source Path: example\cherryusb\device\sdcard_disk

## Supported Platforms
<!-- 支持哪些板子和芯片平台 -->
+ sf32lb52-lcd_n16r8

## Overview
<!-- 例程简介 -->
This example demonstrates implementing a virtual USB flash drive (MSC) using
CherryUSB MSC with an SD card over SPI. It includes:
+ The PC can see a USB drive named "SiFli MSC DEMO" in the file manager.

## How to Use This Example
<!-- 说明如何使用例程，比如连接哪些硬件管脚观察波形，编译和烧写可以引用相关文档。
对于rt_device的例程，还需要把本例程用到的配置开关列出来，比如PWM例程用到了PWM1，需要在onchip菜单里使能PWM1 -->

### Hardware Requirements
Before running this example, prepare:
+ A development board supported by this example ([Supported
  Platforms](quick_start)).
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
        - Purpose: Use SPI for SD (TF) card driver, required for MSC storage
          backend

### Compilation and Flashing
Switch to the example `project` directory and run the scons command to build:

> scons --board=sf32lb52-lcd_n16r8 -j32

Enter the example `project/build_xx` directory (actual build output folder) and
run `uart_download.bat`, then follow the prompt to choose the serial port for
download:

> ./uart_download.bat

> UART Download

> Please enter the COM port number: 5

For detailed compilation and download steps, refer to the [Quick Start
Guide](quick_start).

## Expected Results
<!-- 说明例程运行结果，比如哪几个灯会亮，会打印哪些log，以便用户判断例程是否正常运行，运行结果可以结合代码分步骤说明 -->
After startup: The host connects to the board via the USB data cable. The PC
file manager shows a USB drive named "SiFli MSC DEMO". In Device Manager under
Universal Serial Bus controllers, a new device "USB Mass Storage Device"
appears.

## Troubleshooting


## Reference Documents
<!-- 对于rt_device的示例，rt-thread官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考RT-Thread的[RTC文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Revision History
| Version | Date    | Release Notes   |
| ------- | ------- | --------------- |
| 0.0.1   | 09/2025 | Initial version |
|         |         |                 |
|         |         |                 |
