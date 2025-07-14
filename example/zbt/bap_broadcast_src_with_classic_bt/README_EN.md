# BLE Audio Player

Source path: example/zbt/bap_broadcast_sink

## Supported Platforms
<!-- Which boards and chip platforms are supported -->
+ sf32lb52x

## Overview
<!-- Example introduction -->
This example demonstrates the BLE Audio Sink side, flashed to sf32lb52x boards.
It requires example/zbt/bap_broadcast_src or example/zbt/bap_broadcast_src_with_classic_bt project as the source side. The source side can only use 52j or 52d series (those ending with letters), and cannot use 525 as the source side.

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
scons --board=sf32lb52-lcd_n16r8 -j8
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
After the example runs, it will connect to the BLE Audio source device and receive audio data for playback.



## Exception Diagnosis

## Reference Documentation
<!-- For rt_device examples, the RT-Thread official website documentation provides more detailed explanations, you can add webpage links here, for example, refer to RT-Thread's [RTC documentation](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Update Log
|Version |Date   |Release Notes |
|:---|:---|:---|
|0.0.1 |6/2025 |Initial version |
| | | |