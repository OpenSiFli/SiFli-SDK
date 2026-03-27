# MP3 Local Music Player

Source code path: example/multimedia/audio/mp3_sd_player

## Supported Platforms
<!-- 支持哪些板子和芯片平台 -->
+ sf32lb52-lchspi-ulp

## Overview
<!-- 例程简介 -->
This example will demonstrate playing MP3 or wav audio files in the `music`
directory on the SD card, and provides a shell interface for modifying playback
volume.

## Example Usage
<!-- 说明如何使用例程，比如连接哪些硬件管脚观察波形，编译和烧写可以引用相关文档。
对于 rt_device 的例程，还需要把本例程用到的配置开关列出来，比如 PWM 例程用到了 PWM1，需要在 onchip 菜单里使能 PWM1 -->

### Hardware Requirements
Before running this example, prepare:
+ A development board supported by this example ([Supported
  Platforms](quick_start)).
+ Speaker.

```{warning}
Note that on the `sf32lb52-lchspi-ulp` platform, an external battery is required, otherwise long button presses cannot be detected.
```

### menuconfig Configuration

1. This example needs to read and write files, so it needs to use a file system.
   Configure the `FAT` file system: ![RT_USING_DFS_ELMFAT](./assets/mc_fat.png)
1. This example uses SPI1 as the SD card interface, so SPI1 needs to be enabled:
   ![RT_USING_SPI1](./assets/mc_spi1.png)
1. The SD card in this example is driven using SPI, so the `MSD` driver needs to
   be enabled: ![RT_USING_MSD](./assets/mc_msd.png)
1. Enable AUDIO CODEC and AUDIO PROC: ![AUDIO CODEC &
   PROC](./assets/mc_audcodec_audprc.png)
1. Enable AUDIO(`AUDIO`): ![AUDIO](./assets/mc_audio.png)
1. Enable AUDIO MANAGER.(`AUDIO_USING_MANAGER`)
   ![AUDIO_USING_MANAGER](./assets/mc_audio_manager.png)
1. (`AUDIO_LOCAL_MUSIC`) ![AUDIO_LOCAL_MUSIC](./assets/mc_local_music.png)

### Compilation and Programming

Switch to the example project directory and run the scons command to execute
compilation:

```bash
scons --board=sf32lb52-lchspi-ulp -j32
```

Switch to the example `project/build_xx` directory and run `uart_download.bat`,
select the port as prompted for download:

```bash
$ ./uart_download.bat

     UART Download

Please input the serial port number: 5
```

For detailed steps on compilation and download, please refer to [Quick
Start](quick_start).

## Expected Results of Example
<!-- 说明例程运行结果，比如哪几个灯会亮，会打印哪些 log，以便用户判断例程是否正常运行，运行结果可以结合代码分步骤说明 -->
After the example runs, it will play MP3 or wav files in the `music` directory
on the SD card. After power-on, it is in stop playback state by default. When
the music in the list finishes playing, it will automatically loop from the
first song.

For the `sf32lb52-lchspi-ulp` platform, long press the `KEY1` button to start
playback. Long pressing `KEY1` during playback will stop playback, and the next
start will begin from the first song. Short press `KEY1` button to pause/resume
playback, short press `KEY2` button to switch to the next music.

The example also provides a shell interface. You can use the `volume` command to
get or set the volume. Enter the `volume` command to view the current volume,
enter `volume set <value>` to set the volume, where `<value>` ranges from 0-15.

![shell](./assets/mc_volume_shell.png)

## Exception Diagnosis

## Reference Documents
<!-- 对于 rt_device 的示例，rt-thread 官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考 RT-Thread 的 [RTC 文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Update History
| Version | Date      | Release Notes   |
| ------- | --------- | --------------- |
| 0.0.1   | June 2025 | Initial version |
|         |           |                 |
