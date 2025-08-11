# lvgl_v8_ttf_example

```{warning}
Unverified
```

Source Path: example/lvgl_v8_ttf

This example is used to test the API for using LVGL V8 TTF fonts, utilizing the schrift TTF library. It demonstrates how to integrate and use TrueType font rendering functionality in SiFli-SDK.

### Project Compilation and Download

#### Development Board Project
The board project is located in the project directory. You can compile for specific development boards by specifying the board parameter:
- Compile for HDK 563: Execute `scons --board=eh-lb563` to generate the project
- Download method: Use download.bat in the build directory, e.g., flash eh-lb563 project: `./build_eh-lb563/download.bat` (via J-Link)
- Special note: For SF32LB52x/SF32LB56x series, an additional uart_download.bat will be generated. Execute this script and enter the UART port number for UART download

#### Simulator Project
The simulator project is located in the simulator directory:
- Compile with `scons` (requires modifying simulator/msvc_setup.bat to match local MSVC configuration first)
- Generate Visual Studio project: `scons --target=vs2017` to generate project.vcxproj, compile with Visual Studio
  > Note: If using a version other than VS2017 (e.g., VS2022), you'll be prompted to upgrade the MSVC SDK when loading the project. After upgrading, it can be used normally

## Supported Development Boards

This example supports the following development boards:
- eh-lb563 (HDK 563)
- SF32LB52x series
- SF32LB56x series

## Hardware Requirements

- The development board must be connected to the computer via USB for program download and debugging
- For UART download method, ensure the UART port of the development board is correctly connected and configured

## Example Output

No specific serial output information is provided for this example. When run, it will display TTF font rendering test results, including text display effects with different font sizes and styles.

## Troubleshooting

For any technical questions, please submit an [issue](https://github.com/OpenSiFli/SiFli-SDK/issues) on GitHub.

## Reference Documentation

- [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- [LVGL V8 Official Documentation](https://docs.lvgl.io/v8/)
- [schrift Font Library Documentation](https://github.com/turbolent/schrift)