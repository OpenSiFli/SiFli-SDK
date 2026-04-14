# BT PAN OTA Example

Source path: example/bt/pan_ota


## Supported Platforms
<!-- Supported boards and chip platforms -->
+ sf32lb52-lcd_n16r8
+ sf32lb52-lchspi-ulp
+ sf32lb52-nano_a128r16

## Overview
<!-- Example brief introduction -->
This example demonstrates connecting to a mobile phone's PAN protocol via Bluetooth, registering the device to a specific server through Finsh commands, retrieving firmware version information, and supports firmware updates via OTA.

## Adding CA Certificates
1. Storing signing authority root certificates
- The `external/mbedtls_288/certs/default` directory stores commonly used CA certificate files
- The `certs` directory stores user-added CA certificate files

If the `certs/default` directory does not contain the required CA root certificate file<br>
the user needs to copy their PEM format CA certificate to the "certs" root directory. (Only PEM format certificates are supported, DER format certificates are not supported)<br>
Add certificates stored alongside `DigiCert_Global_Root_CA2.crt`    
![alt text](./assets/list.png)

2. Certificate format description 
- `PEM Format Certificate`

    **PEM Format Certificate** usually ends with **.pem** and **.cer** suffixes.

    When opened with a text editor, the file content starts with `-----BEGIN CERTIFICATE-----` and ends with `-----END CERTIFICATE-----`.
- `DER Format Certificate`

    **DER Format Certificate** is a binary file type.<br>

3. View configuration

Check proj.conf; if PKG_USING_MBEDTLS_USER_CERTS is enabled, all files in the certs root directory will be merged into ports/src/tls_certificate.c
![alt text](./assets/proj.png)

## Usage of the Example
<!-- Explain how to use the example, such as which hardware pins to connect to observe waveforms, compilation and flashing can reference relevant documentation.
For rt_device examples, also list the configuration switches used by this example, for example, the PWM example uses PWM1, so PWM1 needs to be enabled in the onchip menu -->
1. Before connecting, it's best to ensure the mobile phone has network sharing enabled. If network sharing is turned on after BT connection, you can reconnect to the PAN and thus connect to the network using the finsh command "pan_cmd conn_pan"
    1) Enable network sharing on iOS. iOS needs to ensure a SIM card is installed, then simply turn on Personal Hotspot:\
    ![IOS_ENB](./assets/ios_enable_pan.png)
    2) Different Android phones have different paths to enable network sharing, but they all involve finding and enabling Bluetooth network sharing within personal hotspot sharing. Android can open Bluetooth network sharing while connected to WiFi:\
    ![ANDROID_ENB](./assets/android_enable_pan.png)
2. On startup, the example enables Bluetooth Inquiry scan and page scan, allowing devices like phones to discover and initiate connections to this device. The default Bluetooth name is sifli_pan.
3. PAN protocol will only connect successfully when the phone has network sharing enabled. You can find the print "pan connect successed" in the log. Ensure that the phone itself has internet access before executing commands.
4. This example supports OTA functionality, enabling remote firmware upgrades. Before using the OTA function, add the following configuration in the SConstruct file:
   ```
   if GetDepend('USING_DFU_PAN'):
       AddDFU_PAN(SIFLI_SDK)
   ```
   
   The OTA function provides the following commands:
   - `check_dynamic_cmd` - Register device to OTA server and check for new firmware versions, sets update flag if new version exists
   - `reset_to_dload_cmd` - Check update flag, restart device to enter OTA download mode
   - `show_version_cmd` - Display current firmware version information

### Hardware Requirements
Before running this example, prepare:
+ One development board supported by this example ([Supported Platforms](#Platform_pan)).
+ Mobile phone.
+ Version files deployed on the server, corresponding to the URL built by build_ota_query_url

### menuconfig Configuration

1. Enable Bluetooth ([BLUETOOTH]):
    - Path: Sifli middleware → Bluetooth
    - Enable: Enable bluetooth
        - Macro switch: `CONFIG_BLUETOOTH`
        - Function: Enable Bluetooth functionality
2. Enable PAN & A2DP, A2DP is to avoid IOS not supporting standalone PAN connection:
    - Path: Sifli middleware → Bluetooth → Bluetooth service → Classic BT service
    - Enable: Enable BT finsh (optional)
        - Macro switch: `CONFIG_BT_FINSH`
        - Function: Enable finsh command line for controlling Bluetooth
    - Enable: Manually select profiles
        - Macro switch: `CONFIG_BT_PROFILE_CUSTOMIZE`
        - Function: Manually select enabled profiles
    - Enable: Enable PAN
        - Macro switch: `CONFIG_CFG_PAN`
        - Function: Enable PAN protocol
3. Enable BT connection manager:
    - Path: Sifli middleware → Bluetooth → Bluetooth service → Classic BT service
    - Enable: Enable BT connection manager
        - Macro switch: `CONFIG_BSP_BT_CONNECTION_MANAGER`
        - Function: Use connection manager module to manage bt connections
4. Enable NVDS:
    - Path: Sifli middleware → Bluetooth → Bluetooth service → Common service
    - Enable: Enable NVDS synchronous
        - Macro switch: `CONFIG_BSP_BLE_NVDS_SYNC`
        - Function: Bluetooth NVDS synchronization. When Bluetooth is configured to HCUP, BLE NVDS can be accessed synchronously, enable this option; when Bluetooth is configured to LCPU, this option should be disabled
5. menuconfig options needed for Bluetooth auto connection:
    - Path: Sifli middleware → Bluetooth → Bluetooth service → Classic BT service
    - After enabling: Enable BT connection manager, Re-connect to last device if connection timeout happened or system power on will be enabled by default
        - Macro switch: `CONFIG_BT_AUTO_CONNECT_LAST_DEVICE`
        - Function: Enable automatic connection to the last connected device.  
    - Path: Third party packages
    - Enable: FlashDB: Lightweight embedded database, usually enabled by default
        - Macro switch: `CONFIG_PKG_USING_FLASHDB`
        - Function: Enable FlashDB database to retain important data after power off or reboot.
6. Enable DFU PAN functionality:
    - Path: Sifli middleware → Using dfu pan
    - Enable: Using dfu pan
        - Macro switch: `CONFIG_USING_DFU_PAN`
        
### Compilation and Flashing
Switch to the example project directory and run the scons command for compilation:
```c
> scons --board=eh-lb525 -j32
```
Switch to the example `project/build_xx` directory and run [uart_download.bat], select the port as prompted to proceed with downloading:
```c
$ ./uart_download.bat

     Uart Download

please input the serial port num:5
```
For detailed steps on compilation and downloading, please refer to the [Quick Start Guide](/quickstart/get-started.md).

## Expected Results of the Example
<!-- Explain example results, such as which LEDs light up, what logs print, so users can determine if the example is running normally. Results can be explained step-by-step combined with code -->
The example can connect to the mobile phone's PAN protocol, execute commands to properly register settings to the server, and query versions returning JSON formatted version information.
```json
Note: dfu_pan_query_latest_version() will retrieve JSON data returned from the server and write the firmware structure to flash address for direct download later. The JSON is as follows, where v1.1.bin is a placeholder file that needs to be placed together with the firmware file and match the current folder name.

{
  "result": 200,
  "message": "Success",
  "data": [
    {
      "name": "v1.1",
      "thumb": null,
      "zippath": "https://xxx.xxx.com/download?path=example/SF32LB52_LCD_N16R8_TFT_CO5300/sf32lb52-lcd-n16r8/v1.1/v1.1.bin",
      "files": [
        {
          "file_id": 391,
          "file_name": "main.bin",
          "file_size": 4087608,
          "crc32": "0x1026622b",
          "addr": "0x12460000",
          "region_size": "0x00680000",
          "note": null,
          "name": "main.bin",
          "url": "https://xxx.xxx.com/download?path=example/SF32LB52_LCD_N16R8_TFT_CO5300/sf32lb52-lcd-n16r8/v1.1/main.bin"
        },
        {
          "name": "v1.1.bin",
          "url": "https://xxx.xxx.com/download?path=example/SF32LB52_LCD_N16R8_TFT_CO5300/sf32lb52-lcd-n16r8/v1.1/v1.1.bin",
          "addr": "",
          "size": "",
          "crc32": "",
          "region_size": "",
          "note": ""
        }

      ]
    }
  ]
}
```
After connecting PAN properly:

Executing the "check_dynamic_cmd" command will print the following information:
![](./assets/dfu_pan1.png)

Executing the "reset_to_dload_cmd" command will print the following information:
![](./assets/dfu_pan2.png)
Upgrade progress can also be observed through screen UI
![](./assets/dfu_pan3.png)
Executing the "show_version_cmd" command will print the following information:

Indicating successful upgrade from v1.0 to v1.1
![](./assets/dfu_pan4.png)
## Troubleshooting


## Reference Documentation
<!-- For rt_device examples, RT-Thread official website documentation provides more detailed explanations, links can be added here, for example, refer to RT-Thread's [RTC documentation](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Update Log
|Version |Date   |Release Notes |
|:---|:---|:---|
|0.0.1 |12/2025 |Initial version |