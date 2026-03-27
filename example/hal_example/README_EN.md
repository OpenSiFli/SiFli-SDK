# HAL Comprehensive Example

Source Code Path: example/hal_example

## Overview
The HAL comprehensive example demonstrates how to use the Hardware Abstraction
Layer (HAL) features of SiFli-SDK. This example provides a series of test
commands that allow users to run different module test cases through serial port
interaction.

### Hardware Requirements
No special hardware is required; the project runs on standard development
boards.

## Usage
The project can be compiled for a specific target board by specifying the
`board` parameter.
- For example, to compile a project for the HDK 525, run `scons
  --board=eh-lb525` to generate the image file.
- Firmware can be flashed using the `download.bat` script located in the `build`
  directory. For instance, to flash the 525 project generated in the previous
  step via J-Link, execute `.`build_eh-lb525\download.bat``.
- Note: For the SF32LB52x series, an additional `uart_download.bat` script is
  generated. Execute this script and enter the designated UART port number to
  perform the download.

## Project Compilation and Download
The project can be compiled for specific boards by specifying the board name:
1. Compilation command: `scons --board=eh-lb525`
2. Download method: Execute `build_eh-lb525\download.bat` for J-Link download
3. For SF32LB52x series, execute `uart_download.bat` and enter the UART port
   number for download

## Example Usage
Use the following commands in the serial port to interact:

## Troubleshooting
- [SiFli-SDK Quick Start
  Guided](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
