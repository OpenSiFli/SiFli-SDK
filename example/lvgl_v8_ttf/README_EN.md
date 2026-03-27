# lvgl_v8_ttf_example

```{warning}
Unverified
```

Source Path: example/lvgl_v8_ttf

This example is used to test the API for using LVGL V8 TTF fonts, utilizing the
schrift TTF library. It demonstrates how to integrate and use TrueType font
rendering functionality in SiFli-SDK.

### Project Compilation and Download
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

#### Development Board Project
The board project is located in the project directory. You can compile for
specific development boards by specifying the board parameter:
- Compile for HDK 563: Execute `scons --board=eh-lb563` to generate the project
- Download method: Use download.bat in the build directory, e.g., flash eh-lb563
  project: `./build_eh-lb563/download.bat` (via J-Link)

## Supported Development Boards

This example supports the following development boards:
- eh-lb563 (HDK 563)
- SF32LB52x series
- SF32LB56x series

## Hardware Requirements

- The development board must be connected to the computer via USB for program
  download and debugging
- For UART download method, ensure the UART port of the development board is
  correctly connected and configured

## Example Output

No specific serial output information is provided for this example. When run, it
will display TTF font rendering test results, including text display effects
with different font sizes and styles.

## Troubleshooting

- [SiFli-SDK Quick Start
  Guided](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- [LVGL V8 Official Documentation](https://docs.lvgl.io/v8/)
- [schrift Font Library Documentation](https://github.com/turbolent/schrift)

