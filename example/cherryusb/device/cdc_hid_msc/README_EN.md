# USB Composite Device Example Project

Source code path: example\cherryusb\device\cdc_hid_msc

## Supported Development Boards
- SF32LB52_LCD_N16R8

## Example Overview
This project demonstrates how to implement three different USB device functions simultaneously in a single USB connection:
- MSC (Mass Storage Class): RAM-based virtual USB drive
- CDC_ACM (Communications Device Class): Provides virtual serial port communication functionality
- HID (Human Interface Device): Simulates USB keyboard input functionality

## Features

### USB Composite Device Functions
- Virtual Disk: Access to device internal Flash storage with file read/write operations
- Virtual Serial Port: Data echo functionality for debugging and communication
- Virtual Keyboard: Triggered by key presses, sends text input to host

### Key Interaction Functions
- KEY1: Sends "SiFli" string
- KEY2: Sends "CherryUSB" string
- Supports key event detection and handling

## Compilation and Usage

### 1 Compilation
* Navigate to the example project directory and run the scons command to compile:
```
scons --board=sf32lb52-lcd_n16r8 -j8
```
* Run `build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat` and select the port as prompted for downloading:
```
build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat
Uart Download
please input the serial port num:5
```

### 2 Usage Instructions
1. Hardware Connection: Connect the host to the development board's UART port and USB port via USB data cable
2. Program Flashing: Flash the compiled firmware to the development board
3. Open Serial Port: Use serial port utility to open both USB virtual serial port and onboard physical serial port

### 3 Expected Behavior

1. When pressing keys, the serial port outputs the following, with corresponding text input of "CherryUSB" and "SiFli" in text boxes, indicating USB keyboard input functionality is working properly:
```
button:11,0
button:11,3
KEY2 clicked, sending 'CherryUSB'
HID: Sending string: CherryUSB
button:11,1
button:34,0
button:34,3
KEY1 clicked, sending 'SiFli'
HID: Sending string: SiFli
button:34,1
```

2. Opening File Explorer reveals a disk named "SiFli MSC" containing README.TXT. Opening this file shows the following content, indicating the RAM-based virtual USB drive functionality is working properly:
```
SiFli USB Composite Device Demo!
Features:
- MSC: Virtual Disk (RAM-based)
- CDC ACM: Virtual COM Port
- HID: Virtual Keyboard
```

3. Messages sent from the USB virtual serial port will be echoed back, while the physical UART will print the message length.
Example: Sending "SiFli" from the USB virtual serial port will display the following in the serial assistant opened on the physical UART COM port:
```
CDC actual out len:5
CDC actual in len:5
```
Meanwhile, the same text is sent back in the serial assistant opened on the USB virtual serial port:
```
SiFli
```
This indicates CDC_ACM is functioning properly.

### Common Issues

1. Device Not Recognized
   - Check USB connection
   - Verify VID/PID configuration
   - Check USB driver

2. Data Transfer Anomalies
   - Check UART pin configuration
   - Verify baud rate settings
   - Check hardware connections

3. Compilation Errors
   - Check RT-Thread configuration
   - Ensure CherryUSB component is enabled
   - Check header file paths

## Reference Documentation
* CDC ACM Device Class: https://cherryusb.readthedocs.io/zh_CN/latest/quick_start/cdc_acm.html
* CherryUSB Documentation: https://cherryusb.readthedocs.io/
* CherryUSB Official Repository: https://github.com/cherry-embedded/CherryUSB
* RT-Thread Official Website: https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/README

## Update Log
|Version |Date   |Release Notes |
|:---|:---|:---|
|0.0.1 |9/2025 |Initial version |