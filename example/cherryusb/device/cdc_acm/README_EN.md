# USB CDC ACM Example

This example demonstrates how to implement a CDC ACM (Communication Device Class - Abstract Control Model) device using the CherryUSB stack, which emulates a USB interface as a serial port device. The device can be recognized by the host as a virtual serial port (COM port) and supports bidirectional data transmission.

## Features

- **CDC ACM Device Class Implementation**: Complies with USB CDC ACM standard, supports virtual serial port communication
- **Data Transfer Functionality**: Supports bidirectional data transfer between USB and UART
- **DTR (Data Terminal Ready) Control**: Supports DTR signal detection and control
- **High-Speed Transfer**: Supports high-speed USB transfer with maximum packet length up to 512 bytes (high-speed mode)
- **Automatic Data Transmission**: Periodically sends test data to demonstrate data transfer functionality
- **Real-Time Operating System Integration**: Based on RT-Thread RTOS implementation
- **CherryUSB Stack**: Uses lightweight, high-performance CherryUSB protocol stack

## Hardware Connection

### Supported Development Boards
- **SF32LB52_LCD_N16R8**: LCD development board based on SF32LB52 chip

### USB Connection
- Connect the development board's USB interface to PC using a USB cable
- Ensure USB interface power supply is normal
- Supports USB 2.0 full-speed mode and high-speed mode (depending on configuration)

### Pin Configuration
This example uses the internal USB controller, mainly involving the following configurations:
- **USB VID**: 0x38f4 (SiFli Technologies)
- **USB PID**: 0xFFFF
- **Endpoint Configuration**:
  - CDC data input endpoint: 0x85
  - CDC data output endpoint: 0x02  
  - CDC interrupt endpoint: 0x86

## Compilation and Usage

### 1 Compilation

Switch to the example project directory and run the scons command to execute compilation:
```
scons --board=sf32lb52-lcd_n16r8 -j8
```
Run `build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat` and select the port as prompted to download:

```
build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat

Uart Download

please input the serial port num:5
```

### 2 Usage

#### 2.1 Running the Program
1. Download the compiled firmware to the development board
2. Reset or restart the development board
3. After program startup, it will print: "cherryusb device cdc acm demo!"

#### 2.2 Device Recognition
1. Connect the development board to PC using a USB cable
2. PC will recognize a new serial port device (COM port)
3. Device information:
   - Manufacturer: CherryUSB
   - Product Name: CherryUSB CDC DEMO
   - Serial Number: 2022123456

#### 2.3 Data Transfer Test
1. **Automatic Data Transmission**:
   - Program automatically sends 2048 bytes of test data every second (when DTR is enabled)
   - Data content: First 10 bytes are "1234567890", followed by letter 'a' padding

2. **Serial Tool Connection**:
   - Use serial debugging tool to connect to the virtual COM port
   - Any baud rate (CDC ACM does not depend on baud rate settings)
   - 8 data bits, 1 stop bit, no parity

3. **Data Reception**:
   - Can receive data sent from PC serial tool
   - Received data length will be output through USB LOG

#### 2.4 DTR Control
- Device supports DTR (Data Terminal Ready) signal control
- Device only automatically sends data when DTR signal is enabled
- Most serial tools automatically enable DTR when connected

## Technical Details

### Code Structure
- **main.c**: Main program entry, initializes CDC ACM device and starts data transmission loop
- **cdc_acm_template.c**: CDC ACM device implementation, including USB descriptors, endpoint configuration, and data processing
- **usb_config.h**: CherryUSB configuration file, defines various USB parameters and options

### Configuration Parameters
- **Buffer Size**: 2048 bytes (read/write buffers)
- **Maximum Packet Length**: 64 bytes (full-speed) / 512 bytes (high-speed)
- **USB Configuration**: Bus-powered, maximum power consumption 100mA
- **Endpoint Type**: Bulk transfer endpoints + interrupt endpoint

### CherryUSB Features
- Supports advanced descriptor registration API
- Supports USB 2.0 device qualifier descriptor
- Supports Zero Length Packet (ZLP) handling
- Supports endpoint status management and flow control

## Troubleshooting

### Common Issues

1. **Device Cannot Be Recognized**
   - Check USB cable and connection status
   - Verify VID/PID configuration (0x38f4/0xFFFF)
   - Check Windows Device Manager for unknown devices
   - Try reconnecting USB connection

2. **Virtual Serial Port Cannot Be Used**
   - Confirm device is correctly recognized as CDC device
   - Check if correct CDC ACM driver is installed
   - Windows 10/11 usually installs driver automatically

3. **Data Transfer Abnormal**
   - Check DTR signal status (view using serial tool)
   - Confirm serial tool is correctly connected and configured
   - Check USB log output to verify data transmission/reception

4. **Compilation or Download Failure**
   - Confirm SiFli SDK environment is configured correctly
   - Check toolchain path settings
   - Verify target board selection is correct

## Development Guide

### Custom Development
For custom development based on this example, you can modify the following:

1. **USB Descriptor Customization**:
   - Modify VID/PID (requires legitimate USB ID)
   - Change device string descriptors
   - Adjust endpoint configuration

2. **Data Processing Logic**:
   - Modify `usbd_cdc_acm_bulk_out()` function to handle received data
   - Modify `cdc_acm_data_send_with_dtr_test()` function to customize sent data
   - Add data format conversion or protocol processing

3. **Configuration Parameter Adjustment**:
   - Modify buffer size to fit application requirements
   - Adjust transmission frequency and data volume
   - Configure USB transfer mode (full-speed/high-speed)

### Related API Description
- `cdc_acm_init()`: Initialize CDC ACM device
- `usbd_ep_start_read()`: Start endpoint data reading
- `usbd_ep_start_write()`: Start endpoint data writing
- `usbd_cdc_acm_set_dtr()`: DTR status callback function

## Reference Documentation
* CDC ACM Device Class: https://cherryusb.readthedocs.io/zh_CN/latest/quick_start/cdc_acm.html
* CherryUSB Documentation: https://cherryusb.readthedocs.io/
* CherryUSB Official Repository: https://github.com/cherry-embedded/CherryUSB
* RT-Thread Official Website: https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/README

## Update Log
|Version |Date   |Release Notes |
|:---|:---|:---|
|1.0.0 |11/2025 |Enhanced feature description and usage instructions, added technical details |
|0.0.1 |9/2025 |Initial version |