# HAL Comprehensive Example

Source Code Path: example/hal_example

## Overview
The HAL comprehensive example demonstrates how to use the Hardware Abstraction Layer (HAL) features of SiFli-SDK. This example provides a series of test commands that allow users to run different module test cases through serial port interaction.

## Usage

### Supported Development Boards
- eh-lb525

### Hardware Requirements
No special hardware requirements; a standard development board is sufficient.

## Project Compilation and Download
The project can be compiled for specific boards by specifying the board name:
- Compilation command: `scons --board=eh-lb525`
- Download method: Execute `build_eh-lb525\download.bat` for J-Link download
- For SF32LB52x series, execute `uart_download.bat` and enter the UART port number for download

## Example Usage
Use the following commands in the serial port to interact:
1. `help` - Display all supported commands
2. `utest_list` - Show all available test cases
3. `utest_run example_xxx` - Run a specific module test case

## Troubleshooting
No specific troubleshooting information available. For issues, please submit an [issue](https://github.com/OpenSiFli/SiFli-SDK/issues) on GitHub.

## Reference Documentation
- [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)