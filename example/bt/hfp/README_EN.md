# BT Hands-Free HF Role Example

Source code path: example/bt/hfp

{#Platform_bt_hfp}
## Supported Platforms
<!-- Which development boards and chip platforms are supported -->
+ eh-lb52x
+ eh-lb56x
+ eh-lb58x

## Overview
<!-- Example introduction -->
HFP_HF call status information acquisition demo:
+ Hands-free profile related AT command usage

## Example Usage
<!-- Instructions on how to use the example, such as which hardware pins to connect to observe waveforms, compilation and flashing can reference related documentation.
For rt_device examples, the configuration switches used in this example also need to be listed, for example, PWM examples use PWM1, which needs to be enabled in the onchip menu -->

The example enables Bluetooth by default on startup and is controlled through FINSH commands:

1. Search for Bluetooth devices
Use the command hfp_cmd start_inquiry to search for mobile phone Bluetooth devices.
Discovered devices will be printed in the form of logs "device [%s] searched" and "device COD is [%d], addr is xx:xx:xx:xx:xx:xx".

2. Stop searching for Bluetooth devices
Use the command hfp_cmd stop_inquiry to stop searching for mobile phone Bluetooth devices.

3. Connect to Bluetooth devices
Use the command hfp_cmd hfp_connect [addr] to connect, where addr should be copied from the device address (xx:xx:xx:xx:xx:xx) found above.
If you already know the mobile phone Bluetooth device address, you can connect directly without searching for Bluetooth devices.
Connection result print: "HFP HF connected"

4. Disconnect from Bluetooth devices
Use the command hfp_cmd hfp_disconnect [addr] to disconnect, where addr should be copied from the device address (xx:xx:xx:xx:xx:xx) found above.
Disconnection result print: "HFP HF disconnected"

5. Query local phone number
Use the command hfp_cmd local_phone_number to query the local phone number. When the local number is received, there will be a print "the remote phone local number + phone number".
Number acquisition completion print: "get remote local phone number complete"

6. Make a call
Use the command hfp_cmd make_call [phone_number] to make a call. Call result print: "make a call complete " + result

7. Call status notification
After a successful call, call status changes will be received, with corresponding prints: "the remote phone call status type " + type (callsetup) + status (2/3) outgoing call

8. Hang up call
Use the command hfp_cmd handup_call to hang up a call. Hang up result print: "hangup a call complete " + result, followed by call status notification

9. Answer call
Use the command hfp_cmd answer_call to answer a call. Answer result print: "answer a call complete " + result, followed by call status notification

10. Connect call audio
Use the command hfp_cmd audio_connect to connect call audio. Successful call audio establishment print: "HFP HF audio_connected"

11. Disconnect call audio
Use the command hfp_cmd audio_disconnect to disconnect call audio. Successful call audio disconnection print: "HFP HF audio_disconnected"

12. Adjust remote Bluetooth device volume
Use the command hfp_cmd volume_control [val] to implement this, where val is valid between 0-15. Call volume adjustment completion print: "change volume value complete"

### Hardware Requirements
Before running this example, you need to prepare:
+ One development board supported by this example ([Supported Platforms](#Platform_bt_hfp)).

### menuconfig Configuration
1. Enable Bluetooth:
![BLUETOOTH](./assets/mc_bluetooth.png)
2. Enable hands-free HF role:
![HandFree HF](./assets/mc_bt_handfree_hf.png)

### Compilation and Flashing
Switch to the example project directory and run the scons command to compile:
```c
> scons --board=eh-lb525 -j32
```
Switch to the example `project/build_xx` directory and run `uart_download.bat`, select the port as prompted to download:
```c
$ ./uart_download.bat

     Uart Download

please input the serial port num:5
```
For detailed compilation and download steps, please refer to the relevant introduction in [Quick Start](/quickstart/get-started.md).

## Expected Results
<!-- Describe the example execution results, such as which LEDs will light up, what logs will be printed, so users can determine if the example is running normally. Results can be explained step by step combined with code -->
After the example starts:
Mobile phone actively connects to device/device connects to mobile phone through commands. When mobile phone makes a call or device makes a call, mobile phone call status information can be obtained.

## Exception Diagnosis

## Reference Documentation
<!-- For rt_device examples, RT-Thread official website documentation provides detailed explanations, web links can be added here, for example, refer to RT-Thread's [RTC Documentation](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Update History
|Version |Date   |Release Notes |
|:---|:---|:---|
|0.0.1 |01/2025 |Initial version |
| | | |
| | | |