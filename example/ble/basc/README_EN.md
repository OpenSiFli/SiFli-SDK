# BLE BASC Example

Source path: example/ble/basc

## Supported Platforms
The example can run on the following development boards:
* sf32lb52-lcd series
* sf32lb56-lcd series
* sf32lb58-lcd series

## Overview
* This is an application example based on the Sifli BLE protocol stack that can read the microcontroller's battery level value. It includes the following functions: creation and registration of custom GATT services, BLE advertising (for Bluetooth connection), data read/write and notification mechanisms, and BASC functionality.

## GATT Protocol
GATT (Generic Attribute Profile, Generic Attribute Configuration File) is the core specification for low-power Bluetooth (BLE) communication, defining a unified framework and rules for data interaction between devices. It organizes data based on the attribute concept and is the foundation for BLE devices to implement service discovery, data read/write, real-time notifications, and other functions. GATT is essentially a data interaction protocol.
The GATT protocol includes the following types of data:

* Service: A collection of related characteristics, representing a complete function of the device.
* Characteristic: The most common interaction unit in GATT, essentially a combination of related attributes. For example: "Heart Rate Characteristic" contains "Heart Rate Value (characteristic value)" and "Heart Rate Measurement Range (descriptor)".
* Attribute: The smallest data unit in GATT. Each attribute contains 3 parts:
    1. UUID: Used to uniquely identify the attribute. Differentiates different types of data (e.g., the UUID for "Heart Rate Value" is 0x2A37, and for "Battery Level" is 0x2A19)
    2. Value: The actual stored data (e.g., heart rate value "65", battery level "80%")
    3. Permissions: Access permissions for the attribute value, such as: read-only, write-only, read-write, notification, etc.
* Profile: A combination of multiple services, defined for specific application scenarios. For example: "Heart Rate Profile (HRP)" specifies that heart rate monitoring must include "Heart Rate Service" and "Battery Service" (for users to check device battery level)

### Core Functions of GATT
Service and Characteristic Discovery
* After the client connects to the server, it first needs to "traverse" the services and characteristics in the server to understand what data the other party can provide (similar to "querying a directory").
* Process: Client sends "service discovery request" → Server returns all service UUIDs and ranges → Client then discovers the characteristics contained in each service.

## Using the Example

### Preparation Before Configuration
* Mobile phone recommendation: For iPhone, it is recommended to use third-party software LightBlue, and for Android, use nRF Connect for BLE testing.

### menuconfig Configuration
* The required configuration is already enabled by default
```c
menuconfig --board=board_nane
```
1. Enable Bluetooth
- Path: Sifli middleware → Bluetooth
    - Enable: Enable bluetooth
         - Macro switch: `CONFIG_BLUETOOTH`
         - Function: Enable Bluetooth functionality

2. Enable Bluetooth-related finsh (BT_FLASH)
- Path: Sifli middleware → Bluetooth → Bluetooth service → Classic BT service
    - Enable: Enable BT finsh
         - Macro switch: `CONFIG_BT_FINSH`
         - Function: Enable Bluetooth-related Finsh shell command support for convenient debugging

3. Used to activate the Bluetooth battery service client (BLE battery service client)
- Path: Sifli middleware → Bluetooth → Bluetooth service → BLE service
    - Enable: BLE battery service client
         - Macro switch: `CONFIG_BSP_BLE_BASC`
         - Function: Enable Bluetooth Battery Service Client (BASC) profile implementation, allowing the device to act as a client to read battery level information from remote Bluetooth devices
### Compilation and Flashing
Switch to the example project directory and run the scons command to compile:

```
scons --board=sf32lb52-lcd_n16r8 -j8
```

Run `build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat`, and select the port as prompted to download:

```
build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat

Uart Download

please input the serial port num:5
```
### Running Result Display

If you can see the following log information printed in the boot log, it indicates that the example is running successfully:
* Bluetooth protocol initialization successful
![alt text](assets/ble_power.png)
* Advertising started successfully
![alt text](assets/adv_seccess.png)

* Open the corresponding software downloaded above on your mobile phone and connect to the Bluetooth device named SIFLI_APP-xx-xx-xx-xx-xx-xx, as shown in the figure
![alt text](assets/connect.png)

* After successful connection, you can see the Bluetooth device named SIFLI_APP-xx-xx-xx-xx-xx-xx connected in the mobile phone interface, as shown in the figure
![alt text](assets/mac.png)
![alt text](assets/connect_success.jpg)

* You can read the microcontroller's battery level and write values through the mobile app
![alt text](assets/read_write_cccd.png)
![alt text](assets/write_data.jpg)

### Troubleshooting
* If you cannot achieve the steps shown above, please check if Bluetooth is turned on on the mobile phone and whether the phone has connected to the microcontroller.
* Whether the correct menu option has been selected.