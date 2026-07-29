# BT A2DP Sharing Example

Source code path: example/bt/a2dp_sharing

{#Platform_music_src}
## Supported Platforms
<!-- Which boards and chip platforms are supported -->
+ eh-lb525
+ eh-lb563
+ eh-lb567
+ eh-lb58x
+ sf32lb52-lcd series
+ sf32lb56-lcd series
+ sf32lb58-lcd series


## Overview
<!-- Example introduction -->
This example demonstrates A2DP music sharing and HFP call relay. The device works as a relay that connects to a mobile phone and a headset at the same time. Music played on the phone is shared to the headset through the relay device. Music control commands from the headset are also forwarded back to the phone (<span style="color: red;">volume adjustment is not included</span>).

This example also supports both HFP HF and HFP AG roles. The device acts as an HFP HF when connected to the phone, and acts as an HFP AG when connected to the Bluetooth headset. After both the phone and the headset are connected to the relay device, call status, incoming call number, signal/battery indicators, and other information from the phone side can be synchronized to the headset. HFP control requests from the headset side, such as dialing, answering, hanging up, DTMF, and call volume adjustment, can be forwarded to the phone.

## Example Usage
<!-- Instructions on how to use the example, such as connecting hardware pins to observe waveforms, compilation and flashing can reference related documentation.
For rt_device examples, you also need to list the configuration switches used in this example, such as PWM example uses PWM1, which needs to be enabled in the onchip menu -->
The example enables Bluetooth by default, and can accept a connection from a phone or actively initiate a connection to a headset. The phone side is used for A2DP sink/HFP HF connections, and the headset side is used for A2DP source/HFP AG connections.

1. Search for Bluetooth devices:
Use the command `a2dp_trans inquiry start` to search for headset-type Bluetooth devices. This command only reports devices with COD Major Class 0x000400 (Audio device).
Found devices are printed in the format "device [%s] searched" and "device COD is [%d], addr is xx:xx:xx:xx:xx:xx".

2. Connect to Bluetooth devices:
Use the command `a2dp_trans conn [addr]` to connect, where addr is the address (xx:xx:xx:xx:xx:xx) of the device found above - simply copy the printed value.
If you already know the address of a headset-type Bluetooth device, you can connect directly without searching.

3. Music sharing:
    1. Play music when only the phone is connected: the relay device makes no sound.
    2. Play music when only the headset is connected: the headset makes no sound.
    3. Play music when both the phone and the headset are connected: the headset makes sound, and the relay device makes no sound.
    4. While music is being shared, disconnect the headset: the relay device makes no sound.
    5. While music is being shared, disconnect the phone: the headset makes no sound.
    6. While music is being shared, disconnect the headset and then reconnect it: the headset makes sound again.
    7. By default, the relay device does not reconnect to the headset and the phone.

4. HFP call relay:
    1. After both the phone and the headset are connected to the relay device, incoming call, outgoing call, and call status changes on the phone side are synchronized to the headset side.
    2. When the headset side performs actions such as answering, hanging up, dialing, sending DTMF keys, or adjusting call volume, the relay device forwards the corresponding HFP control requests to the phone.
    3. HFP indicator information from the phone side, such as service status, signal strength, battery level, roaming status, incoming call number, local phone number, and current call information, is cached and replied to the headset.
    4. After the SCO call audio link is established on the phone side, the relay device tries to establish the SCO audio link on the headset side and uses `CONFIG_CFG_BT_VOICE_RELAY` for voice relay. When the SCO link on either side is disconnected, the corresponding voice relay link is closed accordingly.
    5. Typical logs after successful connection are "HFP HF connected" and "HFP AG connected". When disconnected, "HFP HF disconnected" and "HFP AG disconnected" are printed respectively.

### Hardware Requirements
Before running this example, you need to prepare:
+ One development board supported by this example ([Supported Platforms](#Platform_music_src)).
+ A mobile phone that supports HFP and A2DP.
+ A Bluetooth headset.

### menuconfig Configuration

1. This example needs to read and write files, so it requires a file system. Configure the `FAT` file system:
    - Path: RTOS → RT-Thread Components → Device virtual file system
    - Enable: Enable elm-chan fatfs
        - Macro switch: `CONFIG_RT_USING_DFS_ELMFAT`
        - Description: Enable fatfs file system
    ```{tip}
     mnt_init mounts the root partition.
    ```
2. Enable AUDIO CODEC and AUDIO PROC:
    - Path: On-chip Peripheral RTOS Drivers
    - Enable: Enable Audio Process driver
        - Macro switch: `CONFIG_BSP_ENABLE_AUD_PRC`
        - Description: Enable Audio process device, mainly used for audio data processing (including resampling, volume adjustment, etc.)
    - Enable: Enable Audio codec driver
        - Macro switch: `CONFIG_BSP_ENABLE_AUD_CODEC`
        - Description: Enable Audio codec device, mainly used for DAC conversion
3. Enable AUDIO(`AUDIO`):
    - Path: Sifli middleware
    - Enable: Enable Audio
        - Description: Enable audio configuration options
4. Enable AUDIO MANAGER(`AUDIO_USING_MANAGER`):
    - Path: Sifli middleware → Enable Audio
    - Enable: Enable audio manager
        - Macro switch: `CONFIG_AUDIO_USING_MANAGER`
        - Description: Use audio manager module for audio process handling
5. Enable local audio(`AUDIO_LOCAL_MUSIC`)
    - Path: Sifli middleware → Enable Audio
    - Enable: Enable local audio
        - Macro switch: `CONFIG_AUDIO_LOCAL_MUSIC`
        - Description: Enable local audio function
6. Pre-install the audio file by placing it in the `\disk\` directory for pre-install download.
    - The audio file is located at `music_source/disk/test.mp3`.
7. Enable Bluetooth(`BLUETOOTH`):
    - Path: Sifli middleware → Bluetooth
    - Enable: Enable bluetooth
        - Macro switch: `CONFIG_BLUETOOTH`
        - Description: Enable bluetooth function
8. Enable A2DP source, A2DP sink, AVRCP, and HFP relay:
    - Path: Sifli middleware → Bluetooth → Bluetooth service → Classic BT service
    - Enable: Enable BT finsh (optional)
        - Macro switch: `CONFIG_BT_FINSH`
        - Description: Enable finsh command line for Bluetooth control
    - Enable: Manually select profiles
        - Macro switch: `CONFIG_BT_PROFILE_CUSTOMIZE`
        - Description: Manually select profiles to enable
    - Enable: Enable A2DP
        - Macro switch: `CONFIG_CFG_AV`
        - Description: Enable A2DP
    - Enable: Enable A2DP source profile
        - Macro switch: `CONFIG_CFG_AV_SRC`
        - Description: Enable A2DP SOURCE ROLE
    - Enable: Enable A2DP share
        - Macro switch: `CONFIG_CFG_AV_SHARING`
        - Description: Enable A2DP music sharing function
    - Enable: Enable A2DP sink profile
        - Macro switch: `CONFIG_CFG_AV_SNK`
        - Description: Enable A2DP SINK ROLE
    - Enable: Enable AVRCP
        - Macro switch: `CONFIG_CFG_AVRCP`
        - Description: Enable AVRCP profile
    - Enable: Enable Handsfree HF
        - Macro switch: `CONFIG_CFG_HFP_HF`
        - Description: Enable the HFP HF role, used to connect to the phone-side HFP AG
    - Enable: Enable Handsfree AG
        - Macro switch: `CONFIG_CFG_HFP_AG`
        - Description: Enable the HFP AG role, used to accept the headset-side HFP HF connection
    - Enable: Enable BT voice relay
        - Macro switch: `CONFIG_CFG_BT_VOICE_RELAY`
        - Description: Enable HFP call voice relay capability
9. Enable BT connection manager:
    - Path: Sifli middleware → Bluetooth → Bluetooth service → Classic BT service
    - Enable: Enable BT connection manager
        - Macro switch: `CONFIG_BSP_BT_CONNECTION_MANAGER`
        - Description: Use connection manager module to manage BT connections
10. Enable NVDS:
    - Path: Sifli middleware → Bluetooth → Bluetooth service → Common service
    - Enable: Enable NVDS synchronous
        - Macro switch: `CONFIG_BSP_BLE_NVDS_SYNC`
        - Description: Bluetooth NVDS synchronization. When Bluetooth is configured to HCPU, BLE NVDS can be accessed synchronously, enable this option; when Bluetooth is configured to LCPU, this option needs to be disabled

### Compilation and Flashing
Switch to the example project directory and run the scons command to compile:
```bash
> scons --board=eh-lb525 -j32
```
Switch to the example `project/build_xx` directory and run `uart_download.bat`, then select the port as prompted to download:
```bash
$ ./uart_download.bat

     Uart Download

please input the serial port num:5
```
For detailed steps on compilation and downloading, please refer to the related introduction in [Quick Start](/quickstart/get-started.md).

## Expected Results
<!-- Explain the example running results, such as which LEDs will light up, which logs will be printed, so users can judge whether the example is running normally. Results can be explained step by step combined with code -->
After the example starts:
1. Play built-in music without a Bluetooth connection.
2. Can search for headset-type Bluetooth devices and play built-in music after connection.
3. After both the phone and the headset are connected, music played on the phone can be shared to the headset through the relay device.
4. After HFP connections are established with both the phone and the headset, call status and number information from the phone side can be synchronized to the headset. Controls from the headset side, such as answering, hanging up, dialing, DTMF, and call volume adjustment, can be forwarded to the phone.
5. When HFP connections are established, the serial log prints "HFP HF connected" and "HFP AG connected". When the call status changes, logs such as "the remote phone call_status", "callsetup_status", and "callheld_status" are printed.

## Troubleshooting

## Reference Documentation
<!-- For rt_device examples, RT-Thread official documentation provides detailed explanations, you can add webpage links here, for example, refer to RT-Thread's [RTC Documentation](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Update History
|Version |Date   |Release Notes |
|:---|:---|:---|
|0.0.1 |05/2026 |Initial version |
