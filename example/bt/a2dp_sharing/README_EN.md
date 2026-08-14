# BT A2DP Sharing Example

Source code path: example/bt/a2dp_sharing

{#Platform_music_src}
## Supported Platforms
<!-- Which boards and chip platforms are supported -->
+ eh-lb525, only one earphone can be connected for audio forwarding, call forwarding is not supported
+ eh-lb563, only one earphone can be connected for audio forwarding, call forwarding is not supported
+ eh-lb567, only one earphone can be connected for audio forwarding, call forwarding is not supported
+ eh-lb58x
+ eh-lb57x
+ sf32lb52-lcd series
+ sf32lb56-lcd series, only one earphone can be connected for audio forwarding, call forwarding is not supported
+ sf32lb58-lcd series

## Overview
<!-- Example introduction -->
This example demonstrates A2DP music sharing and HFP call relay: the device acts as a relay, connecting both a phone and an earphone simultaneously, sharing music played on the phone to the earphone, and forwarding music control commands from the earphone back to the phone. For earphones that support AVRCP absolute volume, the example also synchronizes the media volume set by the phone to the earphone.

This example also supports dual HFP HF and HFP AG roles: the device acts as HFP HF when connected to a phone, and as HFP AG when connected to a Bluetooth earphone. Once both the phone and earphone are connected to the relay device, call status, incoming call numbers, signal/battery level, and other information from the phone side can be synchronized to the earphone. HFP control requests initiated by the earphone side (such as dialing, answering, hanging up, DTMF, and call volume adjustment) can be forwarded to the phone.


## Example Usage
<!-- Instructions for using the example, such as which hardware pins to connect to observe waveforms. Refer to related documentation for compilation and flashing.
For rt_device examples, also list the configuration switches used in this example. For instance, a PWM example using PWM1 requires enabling PWM1 in the onchip menu. -->
After the example starts, Bluetooth is enabled by default with the name `sifli_a2dp_transfor`. It can accept connections from a phone or actively initiate connections to an earphone. The phone side is used for A2DP Sink/HFP HF connections, and the earphone side is used for A2DP Source/HFP AG connections.

1. Search for Bluetooth devices
Use the command `a2dp_trans inquiry start` to search for earphone-type Bluetooth devices. This command only reports devices whose COD Major Class is 0x000400 (Audio device).
Found devices are printed in the format `"device [%s] searched"` and `"device COD is [%d], addr is xx:xx:xx:xx:xx:xx"`.

2. Connect to Bluetooth devices
Use the command `a2dp_trans conn [addr]` to connect, where `addr` is copied from the address (xx:xx:xx:xx:xx:xx) printed above.
If the address of the earphone-type Bluetooth device is already known, you can connect directly without searching.

3. Music sharing
    1. With only the phone connected and music playing, the relay device produces no sound.
    2. With only the earphone connected and music playing, the earphone produces no sound.
    3. With both phone and earphone connected and music playing, the earphone produces sound while the relay device produces no sound.
    4. While music is being shared, disconnecting the earphone causes the relay device to produce no sound.
    5. While music is being shared, disconnecting the phone causes the earphone to produce no sound.
    6. While music is being shared, disconnecting the earphone and then reconnecting it causes the earphone to produce sound again.
    7. The relay device does not reconnect to the earphone or phone by default.
    8. Volume synchronization: when both a phone that supports absolute volume and an earphone that supports AVRCP absolute volume are connected, after the phone adjusts the media volume, the relay device synchronizes the volume value to the earphone.

4. HFP call relay
    1. After both the phone and earphone are connected to the relay device, incoming calls, outgoing calls, and call status changes on the phone side are synchronized to the earphone side.
    2. When the earphone side performs operations such as answering, hanging up, dialing, sending DTMF keys, or adjusting call volume, the relay device forwards the corresponding HFP control requests to the phone.
    3. HFP indicator information from the phone side, including carrier service status, signal strength, battery level, roaming status, incoming call number, local phone number, and current call information, is cached and replied to the earphone.
    4. Once the SCO call audio link is established on the phone side, the relay device attempts to establish the SCO audio link on the earphone side and relays call audio via `CONFIG_CFG_BT_VOICE_RELAY`. When the SCO link on either side is disconnected, the corresponding voice relay link is closed synchronously.
    5. Typical successful connection logs are `"HFP HF connected"` and `"HFP AG connected"`. When disconnected, `"HFP HF disconnected"` and `"HFP AG disconnected"` are printed respectively.


### Hardware Requirements
Before running this example, prepare:
+ A development board supported by this example ([Supported Platforms](#Platform_music_src)).
+ A phone that supports HFP and A2DP.
+ A Bluetooth earphone.

### menuconfig Configuration
1. This example requires reading and writing files, so a file system is needed. Configure the `FAT` file system:
    - Path: RTOS → RT-Thread Components → Device virtual file system
    - Enable: Enable elm-chan fatfs
        - Macro switch: `CONFIG_RT_USING_DFS_ELMFAT`
        - Description: Enable the fatfs file system
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
3. Enable AUDIO (`AUDIO`):
    - Path: Sifli middleware
    - Enable: Enable Audio
        - Description: Enable audio configuration options
4. Enable AUDIO MANAGER (`AUDIO_USING_MANAGER`):
    - Path: Sifli middleware → Enable Audio
    - Enable: Enable audio manager
        - Macro switch: `CONFIG_AUDIO_USING_MANAGER`
        - Description: Use the audio manager module for audio process handling
5. Enable local audio (`AUDIO_LOCAL_MUSIC`):
    - Path: Sifli middleware → Enable Audio
    - Enable: Enable local audio
        - Macro switch: `CONFIG_AUDIO_LOCAL_MUSIC`
        - Description: Enable local audio functionality
6. Pre-install audio files: place audio files in the `\disk\` directory for pre-installation download.
    - The audio file is located at `music_source/disk/test.mp3`.
7. Enable Bluetooth (`BLUETOOTH`):
    - Path: Sifli middleware → Bluetooth
    - Enable: Enable bluetooth
        - Macro switch: `CONFIG_BLUETOOTH`
        - Description: Enable Bluetooth functionality
8. Enable A2DP source, A2DP sink, AVRCP and HFP relay:
    - Path: Sifli middleware → Bluetooth → Bluetooth service → Classic BT service
    - Enable: Enable BT finsh (optional)
        - Macro switch: `CONFIG_BT_FINSH`
        - Description: Enable finsh command line for Bluetooth control
    - Enable: Manually select profiles
        - Macro switch: `CONFIG_BT_PROFILE_CUSTOMIZE`
        - Description: Manually select which profiles to enable
    - Enable: Enable A2DP
        - Macro switch: `CONFIG_CFG_AV`
        - Description: Enable A2DP
    - Enable: Enable A2DP source profile
        - Macro switch: `CONFIG_CFG_AV_SRC`
        - Description: Enable A2DP SOURCE ROLE
    - Enable: Enable A2DP share
        - Macro switch: `CONFIG_CFG_AV_SHARING`
        - Description: Enable A2DP music sharing functionality
    - Enable: Enable A2DP sink profile
        - Macro switch: `CONFIG_CFG_AV_SNK`
        - Description: Enable A2DP SINK ROLE
    - Enable: Enable AVRCP
        - Macro switch: `CONFIG_CFG_AVRCP`
        - Description: Enable AVRCP profile
    - Enable: Enable Handsfree HF
        - Macro switch: `CONFIG_CFG_HFP_HF`
        - Description: Enable HFP HF role for connecting to the phone-side HFP AG
    - Enable: Enable Handsfree AG
        - Macro switch: `CONFIG_CFG_HFP_AG`
        - Description: Enable HFP AG role for accepting the earphone-side HFP HF connection
    - Enable: Enable BT voice relay
        - Macro switch: `CONFIG_CFG_BT_VOICE_RELAY`
        - Description: Enable HFP call voice relay capability
9. Enable BT connection manager:
    - Path: Sifli middleware → Bluetooth → Bluetooth service → Classic BT service
    - Enable: Enable BT connection manager
        - Macro switch: `CONFIG_BSP_BT_CONNECTION_MANAGER`
        - Description: Use the connection manager module to manage BT connections
10. Enable NVDS:
    - Path: Sifli middleware → Bluetooth → Bluetooth service → Common service
    - Enable: Enable NVDS synchronous
        - Macro switch: `CONFIG_BSP_BLE_NVDS_SYNC`
        - Description: Bluetooth NVDS synchronization. When Bluetooth is configured to HCPU, BLE NVDS can be accessed synchronously — enable this option. When Bluetooth is configured to LCPU, this option must be disabled.

### Compilation and Flashing
Switch to the example project directory and run the scons command to compile:
```bash
> scons --board=eh-lb525 -j32
```
Switch to the example `project/build_xx` directory and run `uart_download.bat`. Select the port as prompted to download:
```bash
$ ./uart_download.bat

     Uart Download

please input the serial port num:5
```
For detailed compilation and download steps, refer to the [Quick Start](/quickstart/get-started.md) guide.

## Expected Results
<!-- Describe the expected example running results, such as which LEDs will light up and which logs will be printed, so users can verify the example is running correctly. Results can be explained step by step combined with the code. -->
After the example starts:
1. Built-in music plays without a Bluetooth connection.
2. Earphone-type Bluetooth devices can be searched and built-in music plays after connection.
3. With both phone and earphone connected, music played on the phone can be shared to the earphone through the relay device.
4. After HFP connections are established with both phone and earphone, call status and number information from the phone side can be synchronized to the earphone. Controls from the earphone side such as answering, hanging up, dialing, DTMF, and call volume adjustment can be forwarded to the phone.
5. When HFP connections are successful, the serial port prints `"HFP HF connected"` and `"HFP AG connected"`. When call status changes, logs such as `"the remote phone call_status"`, `"callsetup_status"`, and `"callheld_status"` are printed.

## Technical Notes

### AVRCP Role Configuration

In the A2DP music sharing scenario, the relay device must dynamically set the AVRCP role based on the type of the connected peer device.

#### AVRCP Role Definitions
- **AVRCP TG (Target)**: The audio source device — the party that plays music and is being controlled.
- **AVRCP CT (Controller)**: The control device — the party that sends control commands.

#### Necessity of Dynamic Role Assignment
In the `BT_NOTIFY_AVRCP_PROFILE_CONNECTED` event handler, the code dynamically sets the AVRCP role based on the A2DP role:

```c
avrcp_role = (inst->con[con_idx].cfg == AV_AUDIO_SRC) ? AVRCP_TG : AVRCP_CT;
```

- When the device acts as **A2DP Source** (connected to an earphone), it is assigned **AVRCP TG**.
- When the device acts as **A2DP Sink** (connected to a phone), it is assigned **AVRCP CT**.

**If all connections are incorrectly fixed to the CT role**, the following issues occur:
1. The AVRCP connection with the earphone cannot properly negotiate absolute volume functionality.
2. Volume change registration requests from the earphone will not be responded to.
3. Volume synchronization fails, returning a `BT_ERROR_UNSUPPORTED` error.
4. After the phone adjusts the volume, it cannot be forwarded to the earphone.

## Troubleshooting

## Reference Documentation
<!-- For rt_device examples, the RT-Thread official documentation provides detailed explanations. You can add webpage links here. For example, refer to RT-Thread's [RTC Documentation](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc). -->

## Update History
|Version |Date   |Release Notes |
|:---|:---|:---|
|0.0.1 |05/2026 |Initial version |
