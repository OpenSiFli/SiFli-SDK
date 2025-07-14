# LVGL v8 Demos
## Introduction
This example is used to demonstrate LVGL V8 official examples, using official demo applications.
You can use menuconfig to select the demo application to demonstrate. The included applications are:
- Show some widget demonstrates the usage of lvgl widgets
- Demonstrate the usage of encoder and keyboard demonstrates keyboard
- Benchmark your system demonstrates benchmark
- Stress test for LVGL stress testing
- Music player demo demonstrates music playback

## Project Compilation and Download:
Board projects in the project directory can be compiled by specifying the board to compile projects adapted to the corresponding board.
- For example, to compile a project that can run on HDK 563, execute scons --board=eh-lb563 to generate the project
- Download can be done through download.bat in the build directory. For example, to program the 563 project generated in the previous step, execute .\build_eh-lb563\download.bat to download via jlink
- Special note: For SF32LB52x/SF32LB56x series, an additional uart_download.bat will be generated. You can execute this script and input the download UART port number to perform download
Simulator projects are in the simulator directory.
- Use scons for compilation. The simulator/msvc_setup.bat file needs to be modified accordingly to match the local MSVC configuration
- You can also use scons --target=vs2017 to generate MSVC project project.vcxproj, and use Visual Studio for compilation.

```{note}
If not using VS2017, for example VS2022, when loading the project, it will prompt to upgrade the MSVC SDK. After upgrading, it can be used.
```