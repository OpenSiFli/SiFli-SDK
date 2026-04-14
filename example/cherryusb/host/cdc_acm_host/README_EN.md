# USB CDC ACM Host Example

This example demonstrates how to implement a CDC ACM (Communication Device Class - Abstract Control Model) host using the CherryUSB stack to identify and connect to USB CDC ACM devices (such as virtual serial port devices). The host can enumerate CDC ACM devices, receive data from devices, and support bidirectional data transfer.

## Features

- **CDC ACM Host Class Implementation**: Compliant with USB CDC ACM standard, supporting host connection to CDC devices
- **Automatic Device Recognition**: Automatic enumeration and identification of connected CDC ACM devices
- **Data Reception Capability**: Support for receiving data from USB CDC devices with real-time printing
- **DTR/RTS Control**: Support for setting DTR and RTS control signals of CDC devices
- **Interrupt Reception Mode**: Using interrupt-driven data reception for improved response speed
- **Real-Time Operating System Integration**: Implemented based on RT-Thread RTOS
- **CherryUSB Stack**: Utilizing lightweight, high-performance CherryUSB host protocol stack



## Hardware Connection

### Supported Development Boards
- **SF32LB52_LCD_N16R8**: LCD development board based on SF32LB52 chip

### USB Connection
- Connect USB CDC ACM devices (such as virtual serial port devices, Arduino, etc.) to the development board's USB Host port
- Ensure the development board's USB Host port is properly powered and capable of supplying power to external devices
- Supports USB 2.0 full-speed mode (depending on configuration)

### Connection Topology
```
[PC] <--USB--> [Development Board (USB Host)] <--USB--> [CDC ACM Device]
   Debug Serial                                      Virtual Serial Device
```

### USB Host Configuration
This example uses the internal USB controller in host mode with the following main configurations:
- **USB Base Address**: USBC_BASE
- **CDC Device Name**: ttyACM0
- **Receive Buffer**: 4096 bytes
- **DTR Control**: Enabled (1)
- **RTS Control**: Disabled (0)

## Build and Usage

### 1  Build

Navigate to the example's project directory and run the scons command to build:

**Build for LCD development board:**
```bash
scons --board=sf32lb52-lcd_n16r8 -j8
```

### 2 Download

Run the corresponding download script for the development board and select the port as prompted:

**LCD development board download:**
```bash
build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat

Uart Download
please input the serial port num:5
```

### 3 Usage

#### 3.1 Running the Program
1. Download the compiled firmware to the development board
2. Reset or restart the development board
3. After the program starts, it will print: "cherryusb host demo!"
4. The system initializes the USB host controller

#### 3.2 Connecting CDC Device
1. Connect the CDC ACM device via USB cable to the development board's USB Host port
2. Wait approximately 2 seconds for the system to automatically enumerate the device
3. If the device is successfully recognized, it will print: "ttyACM0 found!"
4. If the device is not found, it will print: "ttyACM0 not found!"

#### 3.3 Data Reception Test
1. **Automatic Data Reception**:
   - When the CDC device sends data, the host automatically triggers the receive callback
   - Received data is immediately printed to the debug serial port
   - Each reception displays the data length and content

2. **Typical Output Example**:
   ```
   cherryusb host demo!
   ttyACM0 found!
   rx callback, size:2048
   receive data: 1234567890aaaaaa...
   rx callback, size:2048
   receive data: 1234567890aaaaaa...
   ```

3. **Testing with Device Side**:
   - You can use the CDC ACM Device example as the device side
   - The device side periodically sends 2048 bytes of data
   - The host side receives and prints this data

#### 3.4 DTR/RTS Control
- The host automatically sets the DTR signal to 1 after the device is opened
- RTS signal is set to 0
- These signals are passed to the CDC device and can be used to control device behavior
- For example, the CDC Device example only sends data when DTR is enabled



## Technical Details

### Code Structure
- **main.c**: Main program entry, initializes USB host controller and manages CDC ACM device connection
- **usb_config.h**: CherryUSB host configuration file, defines various USB parameters and options

### Configuration Parameters
- **Receive Buffer Size**: 4096 bytes
- **Device Enumeration Delay**: 2000 milliseconds (waiting for device enumeration to complete)
- **Device Name**: ttyACM0
- **Open Flags**: RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX
- **DTR State**: Enabled (1)
- **RTS State**: Disabled (0)

### CherryUSB Host Features
- Supports USB host controller initialization
- Supports automatic CDC ACM device identification and enumeration
- Supports interrupt reception mode with automatic callback triggering when data arrives
- Supports DTR/RTS control signal configuration
- Integrated with RT-Thread device driver framework

### Workflow
1. **Initialization Phase**:
   - Call `usbh_initialize()` to initialize the USB host controller
   - Configure USB base address and controller parameters

2. **Device Enumeration Phase**:
   - Delay 2 seconds to wait for device insertion and enumeration
   - USB host stack automatically identifies CDC ACM device
   - Create ttyACM0 device node

3. **Device Opening Phase**:
   - Use `rt_device_find()` to find the device
   - Use `rt_device_open()` to open the device
   - Register receive callback function `rx_cb()`
   - Set DTR/RTS control signals

4. **Data Reception Phase**:
   - Interrupt is triggered when CDC device sends data
   - Call `rx_cb()` callback function
   - Read data to receive buffer and print

## Troubleshooting

### Common Issues

1. **Device Cannot Be Recognized (prints "ttyACM0 not found!")**
   - Check if the CDC device is properly connected to the development board's USB Host port
   - Confirm that the CDC device is properly powered (some devices require external power)
   - Check USB cable quality and data line integrity
   - Increase enumeration delay time (currently 2 seconds)
   - Confirm that the CDC device itself is working properly (can test by connecting to PC first)
   - Review debug logs to confirm USB enumeration process

2. **Device Recognized But No Data Reception**
   - Confirm whether the CDC device is sending data
   - Check device-side DTR signal requirements (some devices only send data when DTR is enabled)
   - Verify that the receive callback function is properly registered
   - Check if USB host controller initialization was successful
   - Check if device open flags include INT_RX

3. **Incomplete Data Reception or Garbled Data**
   - Check receive buffer size (currently 4096 bytes)
   - Confirm if data transfer rate is too fast
   - Verify USB bus quality and signal integrity
   - Check for data overflow or loss

4. **System Instability**
   - Check if USB power supply capability is sufficient
   - Confirm that external device power consumption does not exceed USB Host power supply capability
   - Verify memory usage to avoid memory shortage
   - Check for stack overflow issues

5. **Build or Download Failure**
   - Confirm that SiFli SDK environment is properly configured
   - Check toolchain path settings
   - Verify that target board selection is correct
   - Confirm that project directory structure is complete

## Test Scenarios

### Testing with CDC ACM Device Example
1. Prepare two development boards: one running the CDC ACM Device example, one running this Host example
2. Connect the Device side via USB to the Host side's USB Host port
3. The Host side will automatically recognize the Device and start receiving data
4. You can see the Host side continuously printing test data sent by the Device

## Reference Documentation
* CDC ACM Host Class: https://cherryusb.readthedocs.io/zh_CN/latest/quick_start/cdc_acm.html
* CherryUSB Documentation: https://cherryusb.readthedocs.io/
* CherryUSB Official Repository: https://github.com/cherry-embedded/CherryUSB
* RT-Thread Official Website: https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/README
* USB CDC Specification: https://www.usb.org/document-library/class-definitions-communication-devices-12

## Development Guide

### Custom Development
If you need to perform custom development based on this example, you can modify the following:

1. **Data Reception Processing**:
   - Modify the `rx_cb()` function to implement custom data processing logic
   - Add data parsing, protocol processing, or storage functionality
   - Can forward data to other modules or peripherals

2. **Data Transmission Functionality**:
   - Add `rt_device_write()` calls to implement data transmission
   - Can implement bidirectional communication functionality
   - Support command transmission and response reception

3. **Multi-Device Support**:
   - Find device nodes like ttyACM1, ttyACM2, etc.
   - Support simultaneous connection of multiple CDC devices
   - Register independent callback functions for each device

4. **Configuration Parameter Adjustment**:
   - Modify receive buffer size (recv_buffer)
   - Adjust enumeration delay time to accommodate different devices
   - Configure DTR/RTS control signals

5. **Enhanced Error Handling**:
   - Add device disconnection detection and reconnection mechanism
   - Implement data transfer timeout handling
   - Add logging for exceptional situations

### API Reference

#### USB Host Initialization
- `usbh_initialize(busid, reg_base)`: Initialize USB host controller
  - busid: Bus ID (typically 0)
  - reg_base: USB controller base address

#### RT-Thread Device Operations
- `rt_device_find(name)`: Find device
  - Returns device handle, or RT_NULL on failure
  
- `rt_device_open(dev, flags)`: Open device
  - flags: RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX
  
- `rt_device_read(dev, pos, buffer, size)`: Read device data
  - pos: Offset position (typically 0 for CDC devices)
  - Returns actual number of bytes read
  
- `rt_device_write(dev, pos, buffer, size)`: Write device data
  - Used to send data to CDC device
  
- `rt_device_set_rx_indicate(dev, rx_ind)`: Set receive indication callback
  - rx_ind: Receive callback function pointer

#### CDC ACM Specific API
- `usbh_cdc_acm_set_line_state(cdc_acm, dtr, rts)`: Set line state
  - dtr: DTR signal (0 or 1)
  - rts: RTS signal (0 or 1)

### Data Transmission Example Code
```c
// Send data to CDC device
const char *send_data = "Hello CDC Device\n";
rt_size_t len = rt_device_write(ttyACM0, 0, send_data, strlen(send_data));
if (len > 0) {
    rt_kprintf("Sent %d bytes\n", len);
}
```

## Related Examples

- **CDC ACM Device Example**: Companion CDC ACM device-side example, can be used to test host functionality

## Version History
|Version |Date   |Release Notes |
|:---|:---|:---|
|1.0.0 |11/2025 |Enhanced feature description and usage instructions, added technical details and development guide |
|0.0.1 |11/2025 |Initial version, implemented basic CDC ACM host functionality |
