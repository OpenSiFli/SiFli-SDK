# BT PAN Example

Source code path: example/bt/pan

(Platform_pan)=
## Supported Platforms
<!-- Which boards and chip platforms are supported -->
+ eh-lb52x
+ eh-lb56x
+ eh-lb58x
+ sf32lb52-lcd series
+ sf32lb56-lcd series
+ sf32lb58-lcd series


## Overview
<!-- Example introduction -->
This example demonstrates connecting to a phone's PAN protocol via Bluetooth, then using Finsh commands to retrieve current weather information from a specific website.

## Adding CA Certificates
1. Storing Root Certificates of Signing Authorities

- `external/mbedtls_288/certs/default` The directory stores the commonly used CA certificate files.

- `certs` The directory stores the CA certificate files added by the users.

If the required CA root certificate is not present in the `certs/default` directory,<br>
users must copy their PEM-format CA certificate to the root `certs` directory. (Only PEM format is supported; DER format is not accepted).<br>
Added certificates should be placed alongside `DigiCert_Global_Root_CA2.crt` in this directory.
![alt text](./assets/list.png)

2. Certificate Format Specifications

- `PEM` format Certificates

**PEM format certificate** use **.pem** or **.cer** file with suffixes at the end.

When opened in a text editor, content starts with `-----BEGIN CERTIFICATE-----` and ends with `-----END CERTIFICATE-----`.

- `DERformat Certificates`

    **The DER format certificate** is a binary file type.

3. Checking Configuration
Verify in `proj.conf`: If `PKG_USING_MBEDTLS_USER_CERTS` is enabled, all files in the root certs directory will be merged into `ports/src/tls_certificate.c` during compilation.
![alt text](./assets/proj.png)

## Example Usage
<!-- Instructions on how to use the example, such as connecting hardware pins to observe waveforms, compilation and flashing can reference related documentation.
For rt_device examples, you also need to list the configuration switches used in this example, such as PWM example uses PWM1, which needs to be enabled in the onchip menu -->
1. Before connecting, it's best to ensure the phone has already enabled network sharing. If network sharing is enabled after BT connection, you can reconnect to PAN using the finsh command "pan_cmd conn_pan" to connect to the network.
    1) iOS enables network sharing. iOS needs to ensure a SIM card is installed, then enable Personal Hotspot:\
    ![IOS_ENB](./assets/ios_enable_pan.png)
    2) Different Android devices have different paths to enable network sharing, but all can be found in Personal Hotspot sharing to find Bluetooth network sharing and enable it. Android can enable Bluetooth network sharing based on WiFi connection:\
    ![ANDRIOD_ENB](./assets/android_enable_pan.png)
2. The example will enable Bluetooth Inquiry scan and page scan at startup, allowing phones and other devices to discover and connect to this device. The default Bluetooth name is sifli_pan.
3. With the phone's network sharing enabled, the PAN protocol will connect successfully, and you can find "pan connect successed" in the log. With the phone itself having internet access, 
   input the finsh command "weather" to get current weather information. Successful output is shown below:\
   ![WEATHER_PRINT](./assets/weather_print.png)
4. By default, this example has OTA functionality enabled. Input the finsh command "pan_cmd ota_pan" to download and install the image specified by the URL in main.c via BT PAN. For OTA introduction, see the peripheral_with_ota project.
5. This example has added an autoconnect flag. You can enable it by entering the finsh command: pan_cmd set_retry_flag 1 / You can disable it by entering the command: pan_cmd set_retry_flag 0
6. This example has added the number of autoconnect retries. You can set the maximum retry count by entering the finsh command: pan_cmd set_retry_time 5 (number of retries)
7. Ensure that the phone has enabled network sharing. After the phone disconnects from pan, if you want to automatically reconnect, you can enter the finsh command: pan_cmd autoconnect

### Notes
- If weather retrieval fails, you need to first verify the validity of the local Bluetooth MAC address OUI: The first three bytes (OUI part) of the MAC address must be a valid value assigned by the IEEE. Using an unauthorized OUI may cause network issues.The address can be modified manually.
- MAC address get command format: nvds get_mac(Invalid address: xxxxxx52FD11)
- Command format for setting Bluetooth MAC: nvds update addr 6 xxxxxxYYYYYY
(where YYYYYY: OUI in byte‑reversed order (must be reversed); xxxxxx: user‑defined last 3 bytes)(Valid address: xxxxxx52FD5C)
- Example: A valid OUI E8:6A:64 → reversed to 646AE8
- Auto-update is enabled (CONFIG_NVDS_AUTO_UPDATE_MAC_ADDRESS), manual nvds command overridden.
- If nvds get_mac does not change after a reboot (i.e., the command to set the address fails):  Check if CONFIG_NVDS_AUTO_UPDATE_MAC_ADDRESS is enabled. If it is, disable this macro, then recompile and flash the firmware before attempting to manually set the MAC address again.

### Hardware Requirements
Before running this example, you need to prepare:
+ One development board supported by this example ([Supported Platforms](#Platform_pan)).
+ A phone.
+ A website that can provide weather information (default is api.seniverse.com).

### menuconfig Configuration

1. Enable Bluetooth (`BLUETOOTH`):
    - Path: Sifli middleware → Bluetooth
    - Enable: Enable bluetooth
        - Macro switch: `CONFIG_BLUETOOTH`
        - Description: Enable bluetooth function
2. Enable PAN & A2DP, A2DP is to avoid iOS not supporting standalone PAN connection:
    - Path: Sifli middleware → Bluetooth → Bluetooth service → Classic BT service
    - Enable: Enable BT finsh (optional)
        - Macro switch: `CONFIG_BT_FINSH`
        - Description: Enable finsh command line for Bluetooth control
    - Enable: Manually select profiles
        - Macro switch: `CONFIG_BT_PROFILE_CUSTOMIZE`
        - Description: Manually select profiles to enable
    - Enable: Enable PAN
        - Macro switch: `CONFIG_CFG_PAN`
        - Description: Enable PAN protocol
3. Enable BT connection manager:
    - Path: Sifli middleware → Bluetooth → Bluetooth service → Classic BT service
    - Enable: Enable BT connection manager
        - Macro switch: `CONFIG_BSP_BT_CONNECTION_MANAGER`
        - Description: Use connection manager module to manage BT connections
4. Enable NVDS:
    - Path: Sifli middleware → Bluetooth → Bluetooth service → Common service
    - Enable: Enable NVDS synchronous
        - Macro switch: `CONFIG_BSP_BLE_NVDS_SYNC`
        - Description: Bluetooth NVDS synchronization. When Bluetooth is configured to HCPU, BLE NVDS can be accessed synchronously, enable this option; when Bluetooth is configured to LCPU, this option needs to be disabled

5. Menuconfig for Bluetooth auto-connection:
    - Path: Sifli middleware → Bluetooth → Bluetooth service → Classic BT service
    - Enable: After enabling BT connection manager, Re-connect to last device if connection timeout happened or system power on will be enabled by default
        - Macro switch: `CONFIG_BT_AUTO_CONNECT_LAST_DEVICE`
        - Description: Enable auto-connection to the last connected device.
    - Path: Third party packages
    - Enable: FlashDB: Lightweight embedded database, usually enabled by default
        - Macro switch: `CONFIG_PKG_USING_FLASHDB`
        - Description: Enable FlashDB database to retain important data after power-off or restart

### Compilation and Flashing
Switch to the example project directory and run the scons command to compile:
```c
> scons --board=eh-lb525 -j32
```
Switch to the example `project/build_xx` directory and run `uart_download.bat`, then select the port as prompted to download:
```c
$ ./uart_download.bat

     Uart Download

please input the serial port num:5
```
For detailed steps on compilation and downloading, please refer to the related introduction in [Quick Start](/quickstart/get-started.md).

## Expected Results
<!-- Explain the example running results, such as which LEDs will light up, which logs will be printed, so users can judge whether the example is running normally. Results can be explained step by step combined with code -->
The example can retrieve weather information from a specific website by connecting to the phone's PAN protocol.

## Troubleshooting

## Reference Documentation
<!-- For rt_device examples, RT-Thread official documentation provides detailed explanations, you can add webpage links here, for example, refer to RT-Thread's [RTC Documentation](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Update History
|Version |Date   |Release Notes |
|:---|:---|:---|
|0.0.1 |01/2025 |Initial version |
|0.0.2 |04/2025 |Added OTA |
|0.0.3 |07/2025 |Add CA certificate |
| | | |