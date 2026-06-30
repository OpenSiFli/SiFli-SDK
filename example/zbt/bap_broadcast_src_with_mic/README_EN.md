# MIC Record Player

Source path: example/zbt/bap_broadcast_src_with_mic

## Supported Platforms
<!-- Which boards and chip platforms are supported -->
+ sf32lb52 letter series

## Overview
<!-- Example introduction -->
This example demonstrates the BLE Audio Source side, flashed to sf32lb52 letter series boards, such as 52j or 52D. It cannot be flashed to 525 boards.
After turning on, this example will start recording. The local speaker won't make any sound, and the recorded audio will be streamed in real-time to another sink device's board for playback.
Another sink device can be found at example/zbt/bap_broadcast_sink. You need to compile the sink project and flash it to another device.


## Example Usage
<!-- Explain how to use the example, such as which hardware pins to connect to observe waveforms, compilation and flashing can reference related documentation.
For rt_device examples, you also need to list the configuration switches used by this example, such as PWM example using PWM1, which needs to be enabled in the onchip menu -->

### Hardware Requirements
Before running this example, you need to prepare:
+ One development board supported by this example ([Supported platforms](quick_start)).
+ Speaker.

```{warning}

```



### Compilation and Programming

Switch to the example project directory and run the scons command to execute compilation:

```bash
scons --board=sf32lb52-nano_52j -j8
```

Switch to the example `project/build_xx` directory and run `uart_download.bat`, select the port as prompted to download:

```bash
$ ./uart_download.bat

     Uart Download

please input the serial port num:5
```

For detailed steps on compilation and downloading, please refer to the relevant introduction in [Quick Start](quick_start).

## Expected Results
<!-- Explain the example running results, such as which LEDs will light up, what logs will be printed, so that users can judge whether the example is running normally. The running results can be explained step by step combined with the code -->
After the routine runs, the PCM audio data recorded from the mic will be transmitted via BLE to the connected BLE Audio sink device for playback, and the speaker on the local source device won't play it. If you want to stop broadcasting the mic recording, you can send the command on the source side: audio_src 0.



## Exception Diagnosis

## Reference Documentation
<!-- For rt_device examples, the RT-Thread official website documentation provides more detailed explanations, you can add webpage links here, for example, refer to RT-Thread's [RTC documentation](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Update Log
|Version |Date   |Release Notes |
|:---|:---|:---|
|0.0.1 |6/2026 |Initial version |
| | | |