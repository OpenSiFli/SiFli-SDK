# Audprc Example

Source code path: example/rt_device/audprc

## Supported Platforms
<!-- Which boards and chip platforms are supported -->
+ eh-lb525
+ sf32lb52-lcd_n16r8
+ ec-lb587

## Overview
<!-- Example introduction -->
In this example, Audprc is used together with Audcodec, including:
+ Mic recording.
+ Playing recordings through speaker.
+ Loopback (simultaneous recording and playback).


## Example Usage
<!-- Explain how to use the example, such as connecting which hardware pins to observe waveforms, compilation and flashing can reference related documents.
For rt_device examples, you also need to list the configuration switches used in this example, such as PWM example using PWM1, need to enable PWM1 in onchip menu -->

### Hardware Requirements
Before running this example, you need to prepare:
+ A development board supported by this example ([Supported Platforms](quick_start)).
+ Speaker.

### menuconfig Configuration

1. Enable AUDIO CODEC and AUDIO PROC:
![AUDIO CODEC & PROC](./assets/mc_audcodec_audprc.png)

2. This example uses FINSH functionality, need to enable `RT_USING_FINSH`.
3. Enable peripherals (PA, select according to actual board model)
![PA AW8155](./assets/peripherals.png)
### Compilation and Flashing
Switch to the example project directory and run scons command to compile:
```c
> scons --board=eh-lb525 -j32
```
Switch to the example `project/build_xx` directory and run `uart_download.bat`, select the port as prompted to download:
```c
$ ./uart_download.bat

     Uart Download

please input the serial port num:5
```
For detailed steps on compilation and downloading, please refer to the relevant introduction in [Quick Start](quick_start).

## Expected Results of Example
<!-- Explain example running results, such as which lights will turn on, what logs will be printed, so users can judge whether the example is running normally, results can be explained step by step combined with code -->
In this example, recording and playback are operated through FINSH commands:
Purpose | Command | Example |Notes
|---|--|--|--|
Recording|audprc rx [channels:1/2] [sample rate] [sample bits]|`audprc rx 1 16000 16` |Recording stored in ram buffer (recording automatically stops after 1M)
Play recording|audprc tx [channels:1/2] [sample rate] [sample bits]|`audprc tx 1 16000 16`| Playback data from ram buffer (need to record first)
Loopback|audprc txrx [rx channels:1/2] [rx sample rate] [rx sample bits] [tx channels:1/2] [tx sample rate] [tx sample bits]|`audprc rxtx 1 16000 16 1 16000 16`|Simultaneous recording (mic) and playback (speaker) (Before executing recording and playback commands, if Loopback is running, need to stop first.)
Stop Loopback|audprc txrxstop|`audprc txrxstop`|Stop Loopback

Serial port output as follows:
```
// Input command, start recording
01-23 11:04:27:099 TX:audprc rx 1 16000 16
01-23 11:04:27:105    audprc_rx_entry
01-23 11:04:27:136    codec input parameter:sub_type=4 channels 1, rate 16000, bits 16mic input:rx channel 0, channels 1, rate 16000, bitwidth 16get_mic_volume = 0db
01-23 11:04:27:141    get_mic_volume = 0db
01-23 11:04:27:143    start_rx
01-23 11:04:27:147    [I/drv.audcodec] pll config state:1, samplerate:0 


// Recording ram buffer full 1M, stop recording
01-23 11:04:59:936    Record finished.
01-23 11:04:59:939    RX finished.
01-23 11:04:59:941    stop_rx
01-23 11:04:59:943    [I/drv.audcodec] bf0_audio_stop 0x401
01-23 11:04:59:954    [I/drv.audcodec] audcodec close adc
01-23 11:04:59:957    [I/drv.audcodec] bf0_audio_stop 0x401 done
01-23 11:04:59:958    [I/drv.audprc] bf0_audio_stop 0x1001
01-23 11:04:59:960    [I/drv.audprc] audprc close adc/dac
01-23 11:04:59:962    [I/drv.audprc] bf0_audio_stop 0x1001 done


// Input command, play recording file
01-23 11:05:38:083 TX:audprc tx 1 16000 16
01-23 11:05:38:096    audprc_tx_entry
01-23 11:05:38:097    prc_codec : sub_type=1 channel 1, samplerate 16000, bits 16speaker OUTPUTSRC channel=1 in_rate=16000 out_rate=16000speaker select=0x5050 mixer=0x5150tx[0]: sub_type 0, ch 1, samrate 16000, bits 16
01-23 11:05:38:099    init volume=-18[I/drv.audcodec] dac set volume: prc=0 codec=-18
01-23 11:05:38:100    start_tx
01-23 11:05:38:102    [I/drv.audcodec] codec dac mute set:1
01-23 11:05:38:103    [I/drv.audcodec] pll config state:1, samplerate:0 
01-23 11:05:38:105    [I/drv.audprc] samplerate = 16000, select voice eq
01-23 11:05:38:106    [I/drv.audprc] -----tx dma size = 320
01-23 11:05:38:108    sifli_aw8155 to do power handle 
01-23 11:05:38:110    sifli_aw8155_start,mode:1 10
01-23 11:05:38:126    [I/drv.audcodec] codec dac mute set:0


// Playback complete
01-23 11:06:10:858    Play finished.
01-23 11:06:10:861    Play finished.
01-23 11:06:10:863    stop_tx
01-23 11:06:10:864    [I/drv.audcodec] bf0_audio_stop 0x100
01-23 11:06:10:866    [I/drv.audcodec] audcodec close dac
01-23 11:06:10:867    [I/drv.audcodec] bf0_audio_stop 0x100 done
01-23 11:06:10:869    [I/drv.audprc] bf0_audio_stop 0x100
01-23 11:06:10:870    [I/drv.audprc] audprc close adc/dac
01-23 11:06:10:871    [I/drv.audprc] bf0_audio_stop 0x100 done
01-23 11:06:10:875    sifli_aw8155_stop 


// Input command, start loopback
01-23 11:06:14:524 TX:audprc txrx 1 16000 16 1 16000 16
01-23 11:06:14:533    example_audprc_tx_rx
01-23 11:06:14:540    audprc_rx_tx_entry
01-23 11:06:14:569    codec input parameter:sub_type=4 channels 1, rate 16000, bits 16mic input:rx channel 0, channels 1, rate 16000, bitwidth 16get_mic_volume = 0db
01-23 11:06:14:571    get_mic_volume = 0db
01-23 11:06:14:573    prc_codec : sub_type=1 channel 1, samplerate 16000, bits 16speaker OUTPUTSRC channel=1 in_rate=16000 out_rate=16000speaker select=0x5050 mixer=0x5150tx[0]: sub_type 0, ch 1, samrate 16000, bits 16
01-23 11:06:14:575    init volume=-18[I/drv.audcodec] dac set volume: prc=0 codec=-18
01-23 11:06:14:576    start_txrx
01-23 11:06:14:578    [I/drv.audcodec] codec dac mute set:1
01-23 11:06:14:579    [I/drv.audcodec] pll config state:1, samplerate:0 
01-23 11:06:14:594    [I/drv.audprc] samplerate = 16000, select voice eq
01-23 11:06:14:596    [I/drv.audprc] -----tx dma size = 320
01-23 11:06:14:598    sifli_aw8155 to do power handle 
01-23 11:06:14:598    sifli_aw8155_start,mode:1 10
01-23 11:06:14:623    [I/drv.audcodec] codec dac mute set:0


// Input command, stop loopback
01-23 11:06:21:132 TX:audprc txrxstop
01-23 11:06:21:141    example_audprc_tx_rx_stop
01-23 11:06:21:144    [I/drv.audcodec] bf0_audio_stop 0x502
01-23 11:06:21:146    [I/drv.audcodec] audcodec close dac
01-23 11:06:21:147    [I/drv.audcodec] audcodec close adc
01-23 11:06:21:149    [I/drv.audcodec] bf0_audio_stop 0x502 done
01-23 11:06:21:150    [I/drv.audprc] bf0_audio_stop 0x1102
01-23 11:06:21:152    [I/drv.audprc] audprc close adc/dac
01-23 11:06:21:153    [I/drv.audprc] bf0_audio_stop 0x1102 done
01-23 11:06:21:156    sifli_aw8155_stop 

```

## Exception Diagnosis


## Reference Documents
<!-- For rt_device examples, RT-Thread official website documentation provides detailed explanations, you can add webpage links here, for example, refer to RT-Thread's [RTC documentation](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Update History
|Version |Date   |Release Notes |
|:---|:---|:---|
|0.0.1 |12/2024 |Initial version |
| | | |
| | | |