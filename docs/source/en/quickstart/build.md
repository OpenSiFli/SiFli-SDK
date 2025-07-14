# Build and Download

## Building Project

First, install the dependency packages according to the environment installation requirements and configure the environment variables.

Execute the command `cd example\get-started\hello_world\rtt\project` to enter the project directory of the RT-Thread version hello_world example, run the command `scons --board=sf32lb52-lcd_n16r8 -j8` to compile the hello_world program that can run on the SF32LB52x-DevKit-LCD development board, where `sf32lb52-lcd_n16r8` is the name of the development board. Available board names can be found in [index.md](/supported_boards/index.md), you can replace sf32lb52-lcd_n16r8 with other development board names. The compiled files are stored in the `build_sf32lb52-lcd_n16r8_hcpu` directory.

```{note}
Build command format: `scons --board=<board_name> -jN`, where `<board_name>` is the board name. Available board names can be found in [index.md](/supported_boards/index.md). If board_name does not specify a core, the HCPU configuration is used by default for compilation, and <board_name> will be expanded to <board_name>_hcpu. `-jN` is the multi-threaded compilation parameter, where `N` is the number of threads. For example, in the above example, `-j8` means starting 8 threads for compilation.

The compiled files are stored in the `build_<board_name>` directory, which contains the binary files and download scripts needed for download, where `<board_name>` is the board name with the core as suffix, such as `build_sf32lb52-lcd_n16r8_hcpu`.
```

## Configuration Options

Different projects may have different customized configuration options. We use the menuconfig tool to configure projects. The method to open menuconfig is as follows:

Run the command `scons --board=<board_name> --menuconfig` in the project directory, a configuration interface will pop up, and you can modify configuration options as needed. After modification, press the `ESC` key twice, select save configuration and exit.

```{note}
`--board` is a required parameter that specifies the board name to be compiled. Available board names can be found in [](/supported_boards/index.md). If not specified, configuration cannot be performed normally.
```

## Download Program

Keep the USB connection between the development board and the computer, run `build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat` to download the program to the development board. When prompted `please input serial port number`, enter the actual serial port number of the development board, for example, if it's COM19, enter 19. After entering, press Enter to start downloading the program (it's recommended to restart the development board before pressing Enter to start downloading, and press Enter within 2 seconds after restarting the development board to start downloading). After completion, press any key as prompted to return to the command line prompt.

```{note}
Linux and macOS users use the script file `build_sf32lb52-lcd_n16r8_hcpu/uart_download.sh`, and the usage is the same as the Windows script. Note that macOS users should use device names starting with `/dev/cu.` for serial port numbers, such as `/dev/cu.usbserial-12345678`, rather than device names starting with `/dev/tty.`.
```

## Running Program

After the download is complete, a software reset will be automatically executed, or you can also press the RESET button on the development board. The program will run automatically, and the serial assistant will print hello world prompt information.
