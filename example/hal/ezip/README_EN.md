# EZIP Example

Source code path: example/hal/ezip

## Supported Platforms
<!-- Which boards and chip platforms are supported -->
+ sf32lb52-lcd_n16r8

## Overview
<!-- Example introduction -->
This example demonstrates EZIP usage, including:
+ Decompressing ezip format data.
+ Decompressing lz4 format data.
+ Decompressing gzip format data.


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
1. EZIP decompression (AHB output mode, polling mode), verify output results: 
    ```c
    11-16 16:37:14:846    [EZIP]EZIP initialization OK.
    11-16 16:37:14:847    [EZIP]EZIP AHB (polling mode).
    11-16 16:37:14:849    [EZIP]Output is correct.
    11-16 16:37:14:851    [EZIP]EZIP AHB (polling mode)  --- end.
    ```
2. EZIP decompression (AHB output mode, interrupt mode), verify output results: 
    ```c
    11-16 16:37:14:854    [EZIP]EZIP AHB (intrInterrupt mode).
    11-16 16:37:14:855    msh />[EZIP]ezip_done.
    11-16 16:37:14:857    [EZIP]Output is correct.
    11-16 16:37:14:858    [EZIP]EZIP AHB (intrInterrupt mode)  --- end.
    ```
3. LZ4 decompression (AHB output mode, polling mode), verify output results:
    ```c
    11-16 16:37:14:859    [EZIP]LZ4 AHB (polling mode).
    11-16 16:37:14:861    [EZIP]Output is correct.
    11-16 16:37:14:863    [EZIP]LZ4 AHB (polling mode)  --- end.
    ```
3. GZIP decompression (AHB output mode, polling mode), verify output results:
    ```c
    11-16 16:37:14:865    [EZIP]GZIP AHB (polling mode).
    11-16 16:37:14:867    [EZIP]Output is correct.
    11-16 16:37:14:868    [EZIP]GZIP AHB (polling mode)  --- end.
    ```
    ```{tip}
    If you have an LCD, you can enable the following configuration. In this example, the decompressed image will be sent to the LCD for display (steps 1, 2).  
    #define EXAMPLE_WITH_LCD 0 /* With LCD device. */
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