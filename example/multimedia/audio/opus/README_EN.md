# Opus Example

Source code path: example/multimedia/audio/opus

## Supported Platforms
<!-- 支持哪些板子和芯片平台 -->
+ eh-lb525

## Overview
<!-- 例程简介 -->
This example demonstrates how to use the Opus audio codec library for recording,
encoding, decoding, and playback, including:

+ Record PCM audio data at a 16kHz sampling rate from the microphone via mic
  recording
+ Encoding: Compress PCM data using the Opus codec (10ms frame length,
  approximately 16kbps bitrate)
+ Decoding: Decompress audio data using the Opus decoder
+ Playback: Play the decoded audio data through the speakers

## Example Usage
<!-- 说明如何使用例程，比如连接哪些硬件管脚观察波形，编译和烧写可以引用相关文档。
对于rt_device的例程，还需要把本例程用到的配置开关列出来，比如PWM例程用到了PWM1，需要在onchip菜单里使能PWM1 -->

### Hardware Requirements
Before running this example, prepare:
+ A development board supported by this example ([Supported
  Platforms](quick_start)).
+ Speaker.

### menuconfig Configuration

1. This example needs to read and write files, so it needs to use a file system.
   Configure the `FAT` file system: ![RT_USING_DFS_ELMFAT](./assets/mc_fat.png)

     ```{tip}
     Mount root partition in mnt_init.
     ```
2. Enable AUDIO CODEC and AUDIO PROC: ![AUDIO CODEC &
   PROC](./assets/mc_audcodec_audprc.png)
3. Enable AUDIO(`AUDIO`): ![AUDIO](./assets/mc_audio.png)
4. Enable AUDIO MANAGER.(`AUDIO_USING_MANAGER`)
   ![AUDIO_USING_MANAGER](./assets/mc_audio_manager.png)

### **Routine Description**

If the `opus_test()` function contains `opus_encoder_ctl(encoder,
OPUS_SET_FORCE_MODE(MODE_SILK_ONLY));`,

define `OPUS_STACK_SIZE` as 20k.

If the `opus_test()` function does not contain `opus_encoder_ctl(encoder,
OPUS_SET_FORCE_MODE(MODE_SILK_ONLY));`,

define `OPUS_STACK_SIZE` as 200k.

### Compilation and Programming

Switch to the example project directory and run the scons command to execute
compilation:

> scons --board=eh-lb525 -j32

Switch to the example `project/build_xx` directory and run `uart_download.bat`,
select the port as prompted for download:

> ./uart_download.bat

> UART Download

> Please enter the serial port number: 5

For detailed steps on compilation and download, please refer to [Quick
Start](quick_start).

## Expected Results of Example
<!-- 说明例程运行结果，比如哪几个灯会亮，会打印哪些log，以便用户判断例程是否正常运行，运行结果可以结合代码分步骤说明 -->
After the example starts: Start recording for ten seconds, then automatically
play back after recording is complete. Expected successful recording and
automatic playback.

Manual Commands:
 *    opus: Record 10 seconds of audio to `/mic16k.pcm`, then play back after
      encoding and decoding.
 *    opus /mic16k.pcm: Read PCM data from the specified file, then play back
      after encoding and decoding.
 *    opus xxxxx: If the file does not exist, perform real-time recording and
      loopback playback (simultaneous record and play).

## Exception Diagnosis


## Reference Documents
<!-- 对于rt_device的示例，rt-thread官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考RT-Thread的[RTC文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Update History
| Version | Date    | Release Notes   |
| ------- | ------- | --------------- |
| 0.0.1   | 10/2024 | Initial version |
|         |         |                 |
|         |         |                 |
