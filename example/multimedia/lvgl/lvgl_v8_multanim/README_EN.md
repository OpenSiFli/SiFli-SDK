# LVGL v8 Multanim Example

Source path: `SiFli-SDK/example/multimedia/lvgl/lvgl_v8_multanim`

## Overview

This example demonstrates multiple transition effects provided by the `lvsf_multanim` widget.

## Supported Boards

<!-- List supported boards and chip platforms here -->
- sf32lb52-lcd_n16r8
- sf32lb52-lchspi-ulp

## Hardware Requirements

Before running this example, prepare:

+ One of the supported development boards ([supported platforms](quick_start)).
+ A display.

Build and download:

Supported boards:
- Boards after the 55x series, such as 58x, 56x, and 52x

The board project is located in the `project` directory. You can build a target project for a specific board by specifying `board`.

- For example, to build a project that runs on HDK 563, run `scons --board=eh-lb563`
- For downloading, use `download.bat` in the build directory. For example, to flash the 563 project generated above, run `.\build_eh-lb563\download.bat` to download it via J-Link
- Special note: for the SF32LB52x/SF32LB56x series, an additional `uart_download.bat` will be generated. You can run this script and enter the UART port number to perform the download

## Simulator Configuration

The simulator project is located in the `simulator` directory.

- Use `scons` to build it. The `SiFli-SDK/msvc_setup.bat` file needs to be adjusted to match the MSVC environment on your machine
- You can also run `scons --target=vs2017` to generate the MSVC project file `project.vcxproj`, then build it with Visual Studio
  Note: if you are not using VS2017, for example VS2022, Visual Studio may prompt you to upgrade the MSVC SDK when loading the project. After upgrading, the project can be used normally

## Runtime Description

After startup, the example creates a main animation area and generates control buttons at the bottom to switch between the following animation types:

- `Zoom`
- `3D`
- `Switch`
- `Turn`
- `Scale`
- `Fade`
- `Open`
- `Roll`
- `Book`
- `Shuttle`
- `Shutter`

Some of these effects depend on GPU or VGLite capabilities. Availability depends on the build configuration and the board capabilities.

- After the example starts, it automatically enters `lv_example_multanim()`
- The upper area of the screen is used to display animations
- The bottom buttons are used to switch between different transition effects
- The label area displays the current animation type and key parameters

## Troubleshooting

If you have any technical questions, please submit an [issue](https://github.com/OpenSiFli/SiFli-SDK/issues) on GitHub.

## Reference Documents

- [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
