# Opus 示例

源码路径：example/multimedia/audio/opus

## 支持的平台
<!-- 支持哪些板子和芯片平台 -->
+ eh-lb525

## 概述
<!-- 例程简介 -->
本示例演示如何使用 Opus 音频编解码库进行录音、编码、解码和播放，包含：

+ 通过mic录音，从麦克风录制 16kHz 采样率的 PCM 音频数据
+ 编码：使用 Opus 编码器将 PCM 数据压缩（10ms 帧长，约 16kbps 比特率）
+ 解码：使用 Opus 解码器解压缩音频数据
+ 播放：将解码后的音频数据通过扬声器播放


## 例程的使用
<!-- 说明如何使用例程，比如连接哪些硬件管脚观察波形，编译和烧写可以引用相关文档。
对于rt_device的例程，还需要把本例程用到的配置开关列出来，比如PWM例程用到了PWM1，需要在onchip菜单里使能PWM1 -->

### 硬件需求
运行该例程前，需要准备：
+ 一块本例程支持的开发板（[支持的平台](quick_start)）。
+ 喇叭。

### menuconfig配置

1. 本例程需要读写文件，所以需要用到文件系统，配置`FAT`文件系统：
![RT_USING_DFS_ELMFAT](./assets/mc_fat.png)

     ```{tip}
     mnt_init 中mount root分区。
     ```
2. 使能AUDIO CODEC 和 AUDIO PROC：
![AUDIO CODEC & PROC](./assets/mc_audcodec_audprc.png)
3. 使能AUDIO(`AUDIO`)：
![AUDIO](./assets/mc_audio.png)
4. 使能AUDIO MANAGER.(`AUDIO_USING_MANAGER`)
![AUDIO_USING_MANAGER](./assets/mc_audio_manager.png)

### 例程说明

​		如果`opus_test()`函数中有opus_encoder_ctl(encoder, OPUS_SET_FORCE_MODE(MODE_SILK_ONLY));

则定义`OPUS_STACK_SIZE`为20k。

​		如果`opus_test()`函数没有opus_encoder_ctl(encoder, OPUS_SET_FORCE_MODE(MODE_SILK_ONLY));

则定义`OPUS_STACK_SIZE`为200k。

### 编译和烧录

切换到例程project目录，运行scons命令执行编译：

> scons --board=eh-lb525 -j32

切换到例程`project/build_xx`目录，运行`uart_download.bat`，按提示选择端口即可进行下载：

 >./uart_download.bat

>Uart Download

>please input the serial port num:5

关于编译、下载的详细步骤，请参考[快速上手](quick_start)的相关介绍。

## 例程的预期结果
<!-- 说明例程运行结果，比如哪几个灯会亮，会打印哪些log，以便用户判断例程是否正常运行，运行结果可以结合代码分步骤说明 -->
例程启动后：
自动运行：程序启动后自动录音10秒，编码解码后播放

手动命令：
 *    opus             		     : 录音10秒到 /mic16k.pcm，编码解码后播放
 *    opus /mic16k.pcm  : 从指定文件读取 PCM 数据，编码解码后播放
 *    opus xxxxx               : 如果文件不存在，实时录音并回环播放10s（边录边播，喇叭和麦隔离下，不然有啸叫，可以把喇叭扣到耳朵上捂住）


## 异常诊断


## 参考文档
<!-- 对于rt_device的示例，rt-thread官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考RT-Thread的[RTC文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## 更新记录
|版本 |日期   |发布说明 |
|:---|:---|:---|
|0.0.1 |12/2025 |初始版本 |
| | | |
| | | |
