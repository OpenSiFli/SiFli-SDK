# LVGL v8 Lottie Example

Source Path: example/multimedia/lvgl/lvgl_v8_lottie
```{warning}
Unverified
```
## Introduction

This example is used to test the LVGL V8 Lottie API.

## Usage

The following sections provide only essential information. For complete steps on configuring SiFli-SDK and building/running projects, please refer to the [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html).

### Supported Development Boards

This example can run on the following development boards:
- eh-lb563
- SF32LB52x series
- SF32LB56x series

## Project Compilation and Download:
The board project is located in the project directory. You can compile projects adapted to different boards by specifying the board parameter:
- To compile for HDK 563 development board, execute: scons --board=eh-lb563
- Download via JLink: Run .\build_<board_name>\download.bat
- For SF32LB52x/SF32LB56x series, an additional uart_download.bat is generated. Execute this script and enter the UART port number to download

The simulator project is located in the simulator directory:
- Compile with scons. The SiFli-SDK/msvc_setup.bat file needs to be modified accordingly to match the local MSVC configuration
- You can also generate an MSVC project using scons --target=vs2017 and compile with Visual Studio

```{note}
If you are not using VS2017 (e.g., using VS2022), you will be prompted to upgrade the MSVC SDK when loading the project. After upgrading, it can be used normally.
```

## Troubleshooting

If you have any technical questions, please submit an [issue](https://github.com/OpenSiFli/SiFli-SDK/issues) on GitHub.

## Reference Documentation

- [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)