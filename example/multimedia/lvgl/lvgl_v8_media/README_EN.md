# Local Video File Playback Example

Source Code Path: example/multimedia/lvgl/lvgl_v8_media


## Overview

This example demonstrates media playback. Users can overwrite the H264 format
MP4 file to disk/video_example.mp4, and it will play after compilation and
download. If the T-card file system is loaded successfully, it will
preferentially read video_example.mp4 from the T-card root directory

## Supported Development Boards

<!-- 支持哪些板子和芯片平台 -->
- eh-lb523
- sf32lb52-lcd_n16r8
- eh-lb523
- sf32lb52-lcd_n16r8

## Hardware Requirements

Before running this routine, you need to prepare:
+ A development board supported by this routine ([Supported
  Platforms](quick_start))
+ Screen

### Project Compilation and Programming:
Supported Boards
- Post-55x series boards, such as the 58x, 56x, and 52x series.

Projects located in the `project` directory can be compiled for a specific
target by specifying the `board` parameter.
- For example, to compile a project for the HDK 563, execute `scons
  --board=eh-lb563` to generate the project files.
- Firmware can be programmed using the `download.bat` script located in the
  `build` directory. To flash the 563 project generated in the previous step,
  run `.\build_eh-lb563\download.bat` to perform the download via J-Link.
- Note: For the SF32LB52x and SF32LB56x series, an additional
  `uart_download.bat` script is generated. Run this script and enter the
  appropriate COM port number to perform the download via UART.

## Project Compilation and Download

Supported boards
- Boards after 55x, such as 58x, 56x, 52x
- For example, to compile a project that can run on HDK 563, execute scons
  --board=eh-lb563 to generate the project

## Simulator Configuration

The simulator project is in the simulator directory,

## Troubleshooting
- [SiFli-SDK Quick Start
  Guided](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
