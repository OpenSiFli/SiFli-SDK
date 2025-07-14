# BT PAN Example

Source code path: example/bt/pan

(Platform_pan)=
## Supported Platforms
<!-- Which boards and chip platforms are supported -->
+ eh-lb52x
+ eh-lb56x
+ eh-lb58x

## Overview
<!-- Example introduction -->
This example demonstrates connecting to a phone's PAN protocol via Bluetooth, then using Finsh commands to retrieve current weather information from a specific website.

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

### Hardware Requirements
Before running this example, you need to prepare:
+ One development board supported by this example ([Supported Platforms](#Platform_pan)).
+ A phone.
+ A website that can provide weather information (default is api.seniverse.com).

### menuconfig Configuration

1. Enable Bluetooth (`BLUETOOTH`):
![BLUETOOTH](./assets/bluetooth.png)
2. Enable PAN & A2DP, A2DP is to avoid iOS not supporting standalone PAN connection:
![PAN & A2DP](./assets/bt_pan_a2dp.png)
3. Enable BT connection manager:
![BT CM](./assets/bt_cm.png)
4. Enable NVDS:
![NVDS](./assets/bt_nvds.png)

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
| | | |
| | | |