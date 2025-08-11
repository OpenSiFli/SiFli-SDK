# LVGL Minesweeper Game

Source Code Path: example/multimedia/lvgl/games/minesweeper

This is a minesweeper game example implemented based on LVGL. All operations are performed through the LVGL graphics library without the need to interface with underlying hardware. This example demonstrates how to use LVGL to create interactive game applications, including graphical interface rendering, user input handling, and game logic implementation.
## Usage

The following sections provide only absolutely necessary information. For complete steps on configuring SiFli-SDK and using it to build and run projects, please refer to the [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)

## Supported Development Boards

This example can run on the following development boards:
- eh-lb563
- SF32LB52x series
- SF32LB56x series

This example demonstrates a minesweeper game where all operations are implemented based on LVGL without the need to interface with any underlying hardware.

## Project Compilation and Download:
Board projects in the project directory can be compiled for specific boards by specifying the board:
- To compile a project that can run on HDK 563, execute `scons --board=eh-lb563` to generate the project
- Download can be performed through download.bat in the build directory. For example, to flash the 563 project generated in the previous step, execute `build_eh-lb563\download.bat` for JLink download
- Special note: For SF32LB52x/SF32LB56x series, an additional uart_download.bat will be generated. You can execute this script and enter the download UART port number to perform the download
Simulator project is located in the simulator directory:
- Compile using scons. The simulator/msvc_setup.bat file needs to be modified accordingly to match your local MSVC configuration
- You can also use `scons --target=vs2017` to generate an MSVC project (project.vcxproj) for compilation with Visual Studio.

```{note}
If you are not using VS2017 (e.g., VS2022), you will be prompted to upgrade the MSVC SDK when loading the project. It can be used after upgrading.
```

## Troubleshooting

For any technical questions, please submit an [issue](https://github.com/OpenSiFli/SiFli-SDK/issues) on GitHub

## Reference Documentation
- [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)