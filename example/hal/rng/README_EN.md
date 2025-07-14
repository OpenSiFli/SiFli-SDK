# RNG Example
Source code path: example/hal/rng
## Supported Platforms
<!-- Which boards and chip platforms are supported -->
+ sf32lb52-lcd_n16r8
+ sf32lb58-lcd_n16r64n4
## Overview
<!-- Example introduction -->
This example demonstrates using RNG (Random Number Generator) to generate random number seeds and random numbers.

## Example Usage
<!-- Instructions on how to use the example, such as which hardware pins to connect to observe waveforms, compilation and programming can reference related documentation.
For rt_device examples, also need to list the configuration switches used by this example, such as PWM examples using PWM1, need to enable PWM1 in the onchip menu -->

### Hardware Requirements
Before running this example, you need to prepare a development board supported by this example

<!-- ### menuconfig Configuration -->
 
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
1. Print a group (16) of random numbers every 500ms.  
    ```c
    SFBL
    RND_BUFFER:
    E8C706A8 9951352B F8E1DA37 9F1187FE 1F646D6F DD91FFC6 555F62E3 F0331F25 
    AB574A6E 458BF6B9 9232FD31 D2087C59 C7D34012 237536ED 642E65DD D85D20F7 
    RND_BUFFER:
    2B8EB6A6 D06F3E0F 4CA2B859 C24998F1 FEFB0DD9 1CD7F97E 1E5EECA4 D165FE0C 
    02AE7A9A 3CA3E015 2CB76E67 ABBD8780 D017F211 9DC5F365 846AA886 0C348503 
    RND_BUFFER:
    3892391B 0317442D 5D722D31 6A99CE87 D7DA4DBC CC08065B EBE93330 4929224A 
    53B59A81 28B15E6A 6BCB5A60 979D54B1 4D470A5E 001C491C F70875DA 8DD9FD34 
    RND_BUFFER:
    234A3E47 1AB20C8F 3D06FD8D DA68708D 7C463B6E C31EDA11 1BE0B91E E0A46D41 
    2D6E9838 74A6C7F1 68117920 BCCFB254 13F8BC97 E2AFB62B 7025ADBC D995DC5E 
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