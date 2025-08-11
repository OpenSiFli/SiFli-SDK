# Local Video File Playback Example

Source Code Path: example/multimedia/lvgl/lvgl_v8_media

## Overview

This example demonstrates media playback. Users can overwrite the H264 format MP4 file to disk/video_example.mp4, and it will play after compilation and download.
If the T-card file system is loaded successfully, it will preferentially read video_example.mp4 from the T-card root directory

## Supported Development Boards

<!-- Which boards and chip platforms are supported -->
- eh-lb523
- sf32lb52-lcd_n16r8

## Hardware Requirements

Before running this routine, you need to prepare:
- A development board supported by this routine ([Supported Platforms](#supported-development-boards))
- Screen

## Project Compilation and Download

Supported boards
- Boards after 55x, such as 58x, 56x, 52x

The board project can be compiled in the project directory by specifying the board to adapt to the relative board project,
- For example, to compile a project that can run on HDK 563, execute scons --board=eh-lb563 to generate the project
- Download can be done through download.bat in the build directory. For example, to burn the 563 project generated in the previous step, you can execute .\build_eh-lb563\download.bat to download via jlink
- In particular, for SF32LB52x/SF32LB56x series, an additional uart_download.bat will be generated. You can execute this script and enter the download UART port number to download

## Simulator Configuration

The simulator project is in the simulator directory,
- Use scons to compile, the SiFli-SDK/msvc_setup.bat file needs to be modified accordingly to correspond to the local MSVC configuration
- You can also use scons --target=vs2017 to generate the MSVC project project.vcxproj, and compile it with Visual Studio.
    Note: If you are not using VS2017, such as VS2022, when loading the project, it will prompt to upgrade the MSVC SDK, and you can use it after the upgrade

## Troubleshooting

If you have any technical questions, please raise an [issue](https://github.com/OpenSiFli/SiFli-SDK/issues) on GitHub

## Reference Documentation
- [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)