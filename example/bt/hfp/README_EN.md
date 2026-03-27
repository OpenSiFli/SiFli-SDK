# BT Hands-Free HF Role Example

Source code path: example/bt/hfp

{#Platform_bt_hfp}
## Supported Platforms
<!-- 支持哪些板子和芯片平台 -->
+ eh-lb52x
+ eh-lb56x
+ eh-lb58x
+ sf32lb52-lcd series
+ sf32lb56-lcd series
+ sf32lb58-lcd series

## Overview
<!-- 例程简介 -->
HFP_HF call status information acquisition demo:
+ Hands-free profile related AT command usage

## Example Usage
<!-- 说明如何使用例程，比如连接哪些硬件管脚观察波形，编译和烧写可以引用相关文档。
对于rt_device的例程，还需要把本例程用到的配置开关列出来，比如PWM例程用到了PWM1，需要在onchip菜单里使能PWM1 -->
The example enables Bluetooth by default on startup and is controlled through
FINSH commands:
1. Search for Bluetooth devices Use the command hfp_cmd start_inquiry to search
   for mobile phone Bluetooth devices. Discovered devices will be printed in the
   form of logs "device [%s] searched" and "device COD is [%d], addr is
   xx:xx:xx:xx:xx:xx".

2. Stop searching for Bluetooth devices Use the command hfp_cmd stop_inquiry to
   stop searching for mobile phone Bluetooth devices.

3. Connect to Bluetooth devices Use the command hfp_cmd hfp_connect [addr] to
   connect, where addr should be copied from the device address
   (xx:xx:xx:xx:xx:xx) found above. If you already know the mobile phone
   Bluetooth device address, you can connect directly without searching for
   Bluetooth devices. Connection result print: "HFP HF connected"

4. Disconnect from Bluetooth devices Use the command hfp_cmd hfp_disconnect
   [addr] to disconnect, where addr should be copied from the device address
   (xx:xx:xx:xx:xx:xx) found above. Disconnection result print: "HFP HF
   disconnected"

5. Query local phone number Use the command hfp_cmd local_phone_number to query
   the local phone number. When the local number is received, there will be a
   print "the remote phone local number + phone number". Number acquisition
   completion print: "get remote local phone number complete"

6. Make a call Use the command hfp_cmd make_call [phone_number] to make a call.
   Call result print: "make a call complete " + result

7. Call status notification After a successful call, call status changes will be
   received, with corresponding prints: "the remote phone call status type " +
   type (callsetup) + status (2/3) outgoing call

8. Hang up call Use the command hfp_cmd handup_call to hang up a call. Hang up
   result print: "hangup a call complete " + result, followed by call status
   notification

9. Answer call Use the command hfp_cmd answer_call to answer a call. Answer
   result print: "answer a call complete " + result, followed by call status
   notification

10. Connect call audio Use the command hfp_cmd audio_connect to connect call
    audio. Successful call audio establishment print: "HFP HF audio_connected"

11. Disconnect call audio Use the command hfp_cmd audio_disconnect to disconnect
    call audio. Successful call audio disconnection print: "HFP HF
    audio_disconnected"

12. Adjust remote Bluetooth device volume Use the command hfp_cmd volume_control
    [val] to implement this, where val is valid between 0-15. Call volume
    adjustment completion print: "change volume value complete"

### Hardware Requirements
Before running this example, you need to prepare:
+ One development board supported by this example ([Supported
  Platforms](#Platform_bt_hfp)).

### menuconfig Configuration
1. Enable Bluetooth (`BLUETOOTH`):
    - Path: Sifli middleware → Bluetooth
    - Enable: Enable bluetooth
        - Macro switch: `CONFIG_BLUETOOTH`
        - Description: Enables Bluetooth functionality
2. Enable hands-free HF role:
    - Path: Sifli middleware → Bluetooth → Bluetooth service → Classic BT
      service
    - Enable: Enable BT finsh (Optional)
        - Macro switch: `CONFIG_BT_FINSH`
        - Description: Enable finsh command line for Bluetooth control
    - Enable: Manually select profiles
        - Macro switch: `CONFIG_BT_PROFILE_CUSTOMIZE`
        - Description: Manually select the enabled profile
    - Enable: Enable Handsfree HF
        - Macro switch: `CONFIG_CFG_HFP_HF`
        - Description: Enable Hands free hf ROLE

### Compilation and Flashing
Switch to the example project directory and run the scons command to compile:
```c
&gt; scons --board=eh-lb525 -j32
```
Switch to the example `project/build_xx` directory and run `uart_download.bat`,
select the port as prompted to download:
```c
$ ./uart_download.bat

     UART Download

Please input the serial port number: 5
```
For detailed compilation and download steps, please refer to the relevant
introduction in [Quick Start](/quickstart/get-started.md).

## Expected Results
<!-- 说明例程运行结果，比如哪几个灯会亮，会打印哪些log，以便用户判断例程是否正常运行，运行结果可以结合代码分步骤说明 -->
After the example starts: Mobile phone actively connects to device/device
connects to mobile phone through commands. When mobile phone makes a call or
device makes a call, mobile phone call status information can be obtained.

## Exception Diagnosis


## Reference Documentation
<!-- 对于rt_device的示例，rt-thread官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考RT-Thread的[RTC文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Update History
| Version | Date    | Release Notes   |
| ------- | ------- | --------------- |
| 0.0.1   | 01/2025 | Initial version |
|         |         |                 |
|         |         |                 |
