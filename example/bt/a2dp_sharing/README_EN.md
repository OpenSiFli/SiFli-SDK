# BT Music Transfor Example

Source code path: example/bt/a2dp_transfor

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
This example demonstrates that as an a2dp relay device, the music played by the mobile phone can be forwarded to the headset through the relay device under the condition of connecting the mobile phone and the headset at the same time, and the music control command at the headset can also be forwarded to the mobile phone to realize music control(<span style="color: red;">excluding volume adjustment</span>).

## Example Usage
<!-- Instructions on how to use the example, such as connecting hardware pins to observe waveforms, compilation and flashing can reference related documentation.
For rt_device examples, you also need to list the configuration switches used in this example, such as PWM example uses PWM1, which needs to be enabled in the onchip menu -->
The example will enable Bluetooth by default, and can accept the connection of mobile phone or initiate the connection of headphones.

1. Search for Bluetooth devices:
Use the command `a2dp_trans inquiry start` to search for headphone-type Bluetooth devices. This command will only report devices with COD Major Class 0x000400 (Audio device).
Found devices will be printed in the format "device [%s] searched" and "device COD is [%d], addr is xx:xx:xx:xx:xx:xx".

2. Connect to Bluetooth devices:
Use the command `a2dp_trans conn [addr]` to connect, where addr is the address (xx:xx:xx:xx:xx:xx) of the device found above - simply copy the printed value.
If you already know the address of a headphone-type Bluetooth device, you can connect directly without searching.

3. Music transfor:
1.Play music when the mobile phone is connected alone, and the relay device will not make sound.
2.Play music when headphones are connected separately, and the headphones will not make sound.
3.Play music when the mobile phone and headphones are connected at the same time. The headset device will make sound, but the relay device will not make sound.
4.When the music is being forwarded, disconnect the earphone, and the relay device will not make sound.
5.When the mobile phone is disconnected while the music is being forwarded, the earphone device will not make sound.
6.In the case of forwarding music, disconnect the earphone and then reconnect it, and the earphone device will make sound.
7.By default, the relay device will not connect headphones and mobile phone devices back.

### Hardware Requirements
Before running this example, you need to prepare:
+ One development board supported by this example ([Supported Platforms](#Platform_music_src)).
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
6. Pre-install audio file by placing it in the following \disk\ directory for pre-installation download:  
* Audio file is located at music_source/disk/test.mp3
7. Enable Bluetooth(`BLUETOOTH`):
    - Path: Sifli middleware → Bluetooth
    - Enable: Enable bluetooth
        - Macro switch: `CONFIG_BLUETOOTH`
        - Description: Enable bluetooth function
8. Enable A2DP source and AVRCP:
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
    - Enable: Enable A2DP transfor
        - Macro switch: `CFG_AV_TRANSFOR`
        - Description: Enable A2DP TRANSFOR FUNCTION
    - Enable: Enable A2DP transfor
        - Macro switch: `CFG_AV_SNK`
        - Description: Enable A2DP SINK ROLE
    - Enable: Enable AVRCP
        - Macro switch: `CONFIG_CFG_AVRCP`
        - Description: Enable AVRCP profile
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
After the example starts:
1. Play built-in music without Bluetooth connection
2. Can search for headphone-type Bluetooth devices and play built-in music after connection

## Troubleshooting

## Reference Documentation
<!-- For rt_device examples, RT-Thread official documentation provides detailed explanations, you can add webpage links here, for example, refer to RT-Thread's [RTC Documentation](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Update History
|Version |Date   |Release Notes |
|:---|:---|:---|
|0.0.1 |05/2026 |Initial version |
| | | |
| | | |