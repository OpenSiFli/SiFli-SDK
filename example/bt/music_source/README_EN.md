# BT Music Source Example

Source code path: example/bt/music_source

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
This example demonstrates local music playback and audio streaming to A2DP SINK devices such as headphones after connection, including:
+ Pre-installed mp3 audio file in the root partition.

## Example Usage
<!-- Instructions on how to use the example, such as connecting hardware pins to observe waveforms, compilation and flashing can reference related documentation.
For rt_device examples, you also need to list the configuration switches used in this example, such as PWM example uses PWM1, which needs to be enabled in the onchip menu -->
The example will enable Bluetooth by default at startup and is controlled via FINSH commands:

1. Play local music:
Use the command `music play_default` to play the pre-installed .mp3 file. If no Bluetooth devices like headphones are connected, audio will be output directly from the speaker.
To play songs from the file system, use `music play [loop_time] [path]`, for example `music play 3 \A.mp3` will play A.mp3 from the root partition 4 times (loop_time+1) .

2. Search for Bluetooth devices:
Use the command `music inquiry start` to search for headphone-type Bluetooth devices. This command will only report devices with COD Major Class 0x000400 (Audio device).
Found devices will be printed in the format "device [%s] searched" and "device COD is [%d], addr is xx:xx:xx:xx:xx:xx".

3. Connect to Bluetooth devices:
Use the command `music conn [addr]` to connect, where addr is the address (xx:xx:xx:xx:xx:xx) of the device found above - simply copy the printed value.
If you already know the address of a headphone-type Bluetooth device, you can connect directly without searching.
When the A2DP profile is connected, the device's audio output will automatically switch from speaker to headphones, and switch back when disconnected.

4. Adjust remote Bluetooth device volume:
Use the command `music set_vol [val]` where val is valid between 0-127. This example currently only supports adjusting the volume of headphones that support absolute volume functionality. If the headphones don't support this feature, volume adjustment will not be possible.

### Hardware Requirements
Before running this example, you need to prepare:
+ One development board supported by this example ([Supported Platforms](#Platform_music_src)).
+ A speaker.

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
|0.0.1 |01/2025 |Initial version |
| | | |
| | | |