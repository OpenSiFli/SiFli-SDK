# USB MSC Host Example

English | [简体中文](README.md)

Source Code Path: example/cherryusb/host/msc

## Overview
This example demonstrates how to implement USB Host Mass Storage Class (MSC) functionality using the CherryUSB stack. The example can recognize USB storage devices such as USB flash drives and perform file read/write operations.

Main Features:
- Initialize USB Host controller
- Detect and mount USB storage devices
- Read and print the content of README.TXT file from USB device
- Create and write to WRITE.TXT file on USB device

## Hardware Requirements
- SiFli development board with USB Host support (e.g., sf32lb52)
- USB cable
- USB flash drive or other USB mass storage device

**Note**: It is recommended to use this example together with the `example/cherryusb/device/msc/ram_disk` example. That example can simulate a USB storage device with a pre-configured README.TXT file, which is convenient for testing the USB Host's file reading functionality.

## Usage

### Supported Boards
This example can run on the following boards:
- sf32lb52-lcd_n16r8

### Build and Flash
Switch to the example project directory and run the scons command to build:

> scons --board=sf32lb52-lcd_n16r8 -j8

Switch to the `project/build_xx` directory and run `uart_download.bat`, then select the port as prompted:

 >./uart_download.bat

>Uart Download

>please input the serial port num:5

For detailed steps on building and flashing, please refer to the [Quick Start Guide](quick_start).

### Running the Example

**Method 1: Using Real USB Flash Drive**
1. Create a text file named `README.TXT` in the root directory of the USB drive (optional)
2. Flash the compiled firmware to the development board
3. Connect a serial terminal (baud rate is typically 1000000)
4. Reset the development board
5. The program will wait for a USB mass storage device to be connected
6. Insert the USB flash drive into the board's USB Host port
7. After the device is inserted, the program will automatically continue and perform file read/write operations
8. Observe the serial output

**Method 2: Testing with MSC Device Example (Recommended)**
1. Prepare two development boards
2. Flash this example (MSC Host) to one board
3. Flash `example/cherryusb/device/msc/ram_disk` example (MSC Device) to the other board
4. Start the Host board first, the program will wait for a USB mass storage device connection
5. Connect the two boards using a USB cable (Device board's USB Device port to Host board's USB Host port)
6. Then start the Device board
7. The Host board will automatically recognize the storage device emulated by the Device board and continue to read the README.TXT file and perform other operations

### Debug Functions
In addition to the auto-running example code, you can also manually operate the file system on the USB storage device through the finsh console:

- `ls` - List files and folders in the current directory
- `cat <filename>` - Display file content
- `echo <content> > <filename>` - Write content to file
- `mkdir <dirname>` - Create new directory
- `cd <dirname>` - Change directory
- `rm <filename>` - Delete file

Example operations:
```bash
msh />ls                          # View root directory
msh />cd /                        # Enter root directory
msh />cat README.TXT              # View file content
msh />echo Hello USB > test.txt  # Create and write to file
msh />mkdir newfolder             # Create new folder
```

## Example Output
If the example runs successfully, you will see the following output on the serial port:
```
cherryusb host demo!
Waiting for USB mass storage device connection...
msh />udisk: /dev/sda mount successfully
File content:
cherryusb device msc_ram demo!
```

## Troubleshooting
No specific troubleshooting information at this time. If you encounter any issues, please submit an [issue on GitHub](https://github.com/OpenSiFli/SiFli-SDK/issues).

## Reference Documentation
- [SiFli-SDK Quick Start Guide](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- [SiFli-SDK Development Guide](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/development/index.html)