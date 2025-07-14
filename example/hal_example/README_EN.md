# HAL Comprehensive Example

## Introduction
Use commands in the serial port:
1. help can display all commands
2. utest_list can display all supported examples
3. utest_run example_xxx can run a specific module's example

## Project Compilation and Download:
Projects can be compiled by specifying the board to generate programs that can run on the corresponding board
- For example, to compile a project that can run on HDK 525, execute scons --board=eh-lb525 to generate the image file
- Download can be done through download.bat in the build directory. For example, to program the 525 project generated in the previous step, execute `build_eh-lb525\download.bat` to download via jlink
- Special note: For SF32LB52x series, an additional uart_download.bat will be generated. You can execute this script and input the download UART port number to perform download