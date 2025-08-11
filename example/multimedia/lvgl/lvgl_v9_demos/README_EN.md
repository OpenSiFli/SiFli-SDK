# LVGL v9 Demo

Source Code Path: example/multimedia/lvgl/lvgl_v9_demos
## Overview

This example demonstrates the official LVGL V9 demo applications. You can use menuconfig to select which demo applications to show. Included applications:
- Show some widget: Demonstrates the usage of lvgl widgets
    - Benchmark your system: Demonstrates benchmark (depends on "Show some widget")
- Demonstrate the usage of encoder and keyboard: Demonstrates keyboard
- Stress test for LVGL: Stress test
- Music player demo: Demonstrates music playback

## Supported Development Boards

<!-- Which boards and chip platforms are supported -->
- Boards after 55x series, such as 58x, 56x, 52x
- SF32LB52x series
- SF32LB56x series

## Hardware Requirements

Before running this example, you need to prepare:
- A development board supported by this example
- Screen

## Project Compilation and Download:
Board projects can be compiled by specifying the board in the project directory,
- For example, to compile a project that can run on HDK 563, execute scons --board=eh-lb563 to generate the project
- Download can be done through download.bat in the build directory. For example, to flash the 563 project generated in the previous step, execute .\build_eh-lb563\download.bat to download via jlink
- Special note: For SF32LB52x/SF32LB56x series, an additional uart_download.bat will be generated. You can execute this script and enter the download UART port number to perform the download
Simulator project is in the simulator directory,
- Compile using scons. The SiFli-SDK/msvc_setup.bat file needs to be modified accordingly to match the local MSVC configuration
- You can also use scons --target=vs2017 to generate the MSVC project project.vcxproj and compile using Visual Studio.

```{note}
If you are not using VS2017, such as VS2022, you will be prompted to upgrade the MSVC SDK when loading the project. After upgrading, you can use it.
```
## Troubleshooting

For any technical questions, please raise an [issue](https://github.com/OpenSiFli/SiFli-SDK/issues) on GitHub

## Reference Documentation
- [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)