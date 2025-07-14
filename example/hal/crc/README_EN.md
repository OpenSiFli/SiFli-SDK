# CRC Example
Source code path: example/hal/crc
## Supported Platforms
<!-- Which boards and chip platforms are supported -->
+ sf32lb52-lcd_n16r8

## Overview
<!-- Example introduction -->
This example demonstrates using the CRC module for CRC checksum calculations, including:
+ CRC standard model calculations
     - Supported standard models are defined in the enumeration `HAL_CRC_Mode`.
+ Custom polynomial and initial value calculations

## Example Usage
<!-- Instructions on how to use the example, such as which hardware pins to connect to observe waveforms, compilation and programming can reference related documentation.
For rt_device examples, also need to list the configuration switches used by this example, such as PWM examples using PWM1, need to enable PWM1 in the onchip menu -->

### Hardware Requirements
Before running this example, you need to prepare a development board supported by this example

### menuconfig Configuration


### Compilation and Programming
Switch to the example project directory and run the scons command to compile:
```
scons --board=sf32lb52-lcd_n16r8 -j32
```
Run `build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat`, select the port as prompted to download:
```
$ ./uart_download.bat

     Uart Download

please input the serial port num:5
```
For detailed steps on compilation and downloading, please refer to the relevant introduction in [](/quickstart/get-started.md).

## Expected Results
<!-- Describe the example execution results, such as which LEDs will light up, what logs will be printed, so users can determine if the example is running normally. Results can be explained step by step combined with code -->
After the example starts, the serial port outputs the following:
1. Print raw data:
```c
10-20 20:35:31:631    Input raw data:
10-20 20:35:31:633     0x54, 0x68, 0x69, 0x73, 0x20, 0x69, 0x73, 0x20,
10-20 20:35:31:635     0x61, 0x20, 0x74, 0x65, 0x73, 0x74, 0x20, 0x64,
10-20 20:35:31:637     0x61, 0x74, 0x61, 0x20, 0x75, 0x73, 0x65, 0x64,
10-20 20:35:31:639     0x20, 0x66, 0x6f, 0x72, 0x20, 0x74, 0x65, 0x73,
10-20 20:35:31:641     0x74, 0x69, 0x6e, 0x67, 0x20, 0x63, 0x72, 0x63,
10-20 20:35:31:643     0x20, 0x61, 0x6c, 0x67, 0x6f, 0x72, 0x69, 0x74,
10-20 20:35:31:644     0x68, 0x6d, 0x20, 0x77, 0x69, 0x74, 0x68, 0x20,
10-20 20:35:31:693     0x74, 0x68, 0x65, 0x20, 0x63, 0x72, 0x63, 0x20,
10-20 20:35:31:698     0x68, 0x61, 0x72, 0x64, 0x77, 0x61, 0x72, 0x65,
10-20 20:35:31:704     0x20, 0x62, 0x6c, 0x6f, 0x63, 0x6b, 0x2e,
```
2. CRC-7/MMC calculation result:
```c
10-20 20:35:31:710    CRC Result(mode:0):
10-20 20:35:31:717    0x61
```
3. CRC_8 calculation result:
```c
10-20 20:35:31:722    CRC Result(mode:1):
10-20 20:35:31:726    0x62
```
4. CRC-8/ITU calculation result:
```c
10-20 20:35:31:730    CRC Result(mode:2):
10-20 20:35:31:734    0x37
```
5. CRC-8/ROHC calculation result:
```c
10-20 20:35:31:740    CRC Result(mode:3):
10-20 20:35:31:746    0xCD
```
6. CRC-8/MAXIM calculation result:
```c
10-20 20:35:31:750    CRC Result(mode:4):
10-20 20:35:31:754    0xBA
```
7. CRC-16/IBM calculation result:
```c
10-20 20:35:31:758    CRC Result(mode:5):
10-20 20:35:31:762    0x5263
```
8. CRC-16/MAXIM calculation result:
```c
10-20 20:35:31:767    CRC Result(mode:6):
10-20 20:35:31:775    0xAD9C
```
9. CRC-16/USB calculation result:
```c
10-20 20:35:31:781    CRC Result(mode:7):
10-20 20:35:31:785    0x72F2
```
10. CRC-16/MODBUS calculation result:
```c
10-20 20:35:31:794    CRC Result(mode:8):
10-20 20:35:31:799    0x8D0D
```
11. CRC-16/CCITT calculation result:
```c
10-20 20:35:31:806    CRC Result(mode:9):
10-20 20:35:31:810    0xFC57
```
12. CRC-16/CCITT-FALSE calculation result:
```c
10-20 20:35:31:814    CRC Result(mode:10):
10-20 20:35:31:819    0xBED
```
13. CRC-16/X25 calculation result:
```c
10-20 20:35:31:823    CRC Result(mode:11):
10-20 20:35:31:826    0xA52A
```
14. CRC-16/XMODEM calculation result:
```c
10-20 20:35:31:830    CRC Result(mode:12):
10-20 20:35:31:834    0x4A88
```
15. CRC-16/DNP calculation result:
```c
10-20 20:35:31:839    CRC Result(mode:13):
10-20 20:35:31:843    0xD423
```
16. CRC-32 calculation result:
```c
10-20 20:35:31:847    CRC Result(mode:14):
10-20 20:35:31:854    0x15B6B604
```
17. CRC-32/MPEG_2 calculation result:
```c
10-20 20:35:31:858    CRC Result(mode:15):
10-20 20:35:31:862    0x42F1290F
```
18. Custom polynomial and initialization calculation result:
```c
10-20 20:35:31:866    CRC Result(CRC_8 initial value:0xFF poly:0x1D):
10-20 20:35:31:869    0x45 
```

## Exception Diagnosis

## Reference Documents
<!-- For rt_device examples, the RT-Thread official website documentation provides more detailed descriptions. Web page links can be added here, for example, refer to RT-Thread's [RTC documentation](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Update Log
|Version |Date   |Release Notes |
|:---|:---|:---|
|0.0.1 |10/2024 |Initial version |
| | | |
| | | |