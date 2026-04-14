# Local Music Example for stereo speaker

Source code path: example/multimedia/audio/dual_adc_dac

## Supported Platforms
<!-- Which boards and chip platforms are supported -->
+ sf32lb58-lcd series

## Overview
<!-- Example introduction -->
This example demonstrates local music playback, including:
+ Preset a wav audio file in the root partition.
+ Play the preset wav file.

## Example Usage
<!-- Explain how to use the example, such as connecting which hardware pins to observe waveforms, compilation and programming can reference related documents.
For rt_device examples, also need to list the configuration switches used by this example, such as PWM example using PWM1, need to enable PWM1 in onchip menu -->

### Hardware Requirements
Before running this example, prepare:
+ A development board supported by this example ([Supported Platforms](quick_start)).
+ Speaker.

### menuconfig Configuration

1. This example needs to read and write files, so it needs to use a file system. Configure the `FAT` file system:
![RT_USING_DFS_ELMFAT](./assets/mc_fat.png)

     ```{tip}
     Mount root partition in mnt_init.
     ```
2. Enable AUDIO CODEC and AUDIO PROC:
![AUDIO CODEC & PROC](./assets/mc_audcodec_audprc.png)
3. Enable AUDIO(`AUDIO`):
![AUDIO](./assets/mc_audio.png)
4. Enable AUDIO MANAGER.(`AUDIO_USING_MANAGER`)
![AUDIO_USING_MANAGER](./assets/mc_audio_manager.png)
5. (`AUDIO_LOCAL_MUSIC`)
![AUDIO_LOCAL_MUSIC](./assets/mc_local_music.png)
6. Preset audio files (also supports MP3), place them in the \disk\ directory below for preset download:  
* Audio files are under multimedia/audio/local_music/disk

### Compilation and Programming
Switch to the example project directory and run the scons command to execute compilation:
```c
> scons --board=sf32lb58-lcd_n16r32n1_a1_dpi_hcpu -j8
```
Switch to the example `project/build_xx` directory and run `uart_download.bat`, select the port as prompted for download:
```c
$ ./uart_download.bat

     Uart Download

please input the serial port num:5
```
For detailed steps on compilation and download, please refer to the relevant introduction in [Quick Start](quick_start).

## Expected Results of Example
<!-- Explain example running results, such as which LEDs will light up, which logs will be printed, to help users judge whether the example is running normally, running results can be explained step by step combined with code -->
After the example starts:
Play the preset stereo.wav audio file once. Expected to different voice from two speakers.
input command by serial tools
mic2file
mic2speaker


## Exception Diagnosis


## Reference Documents
<!-- For rt_device examples, RT-Thread official website documentation provides detailed explanations, web links can be added here, for example, refer to RT-Thread's [RTC documentation](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Update History
|Version |Date   |Release Notes |
|:---|:---|:---|
|0.0.1 |10/2024 |Initial version |
| | | |
| | | |