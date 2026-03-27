# EZIP Example

Source code path: example/hal/ezip

## Supported Platforms
<!-- 支持哪些板子和芯片平台 -->
+ sf32lb52-lcd series
+ sf32lb56-lcd series
+ sf32lb58-lcd series

## Overview
<!-- 例程简介 -->
This example demonstrates EZIP usage, including:
+ Decompressing ezip format data.
+ Decompressing lz4 format data.
+ Decompressing gzip format data.

## Example Usage
<!-- 说明如何使用例程，比如连接哪些硬件管脚观察波形，编译和烧写可以引用相关文档。
对于rt_device的例程，还需要把本例程用到的配置开关列出来，比如PWM例程用到了PWM1，需要在onchip菜单里使能PWM1 -->

### Hardware Requirements
Before running this example, you need to prepare a development board supported
by this example

### menuconfig Configuration


### Compilation and Programming
Switch to the example project directory and run the scons command to compile:
```
scons --board=sf32lb52-lcd_n16r8 -j32
```
Run `build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat`, select the port as
prompted to download:
```
$ ./uart_download.bat

     UART Download

Please input the serial port number: 5
```
For detailed steps on compilation and downloading, please refer to the relevant
introduction in [](/quickstart/get-started.md).

## Expected Results
<!-- 说明例程运行结果，比如哪几个灯会亮，会打印哪些log，以便用户判断例程是否正常运行，运行结果可以结合代码分步骤说明 -->
After the example starts, the serial port outputs the following:
1. EZIP decompression (AHB output mode, polling mode), verify output results:
    ```c
    11-16 16:37:14:846    [EZIP] EZIP initialization successful.
    11-16 16:37:14:847    [EZIP] EZIP AHB (polling mode).
    11-16 16:37:14:849    [EZIP] Output verified.
    11-16 16:37:14:851    [EZIP] EZIP AHB (polling mode) --- completed.
    ```
2. EZIP decompression (AHB output mode, interrupt mode), verify output results:
    ```c
    11-16 16:37:14:854    [EZIP] EZIP AHB (interrupt mode).
    11-16 16:37:14:855    msh /&gt;[EZIP] ezip_done.
    11-16 16:37:14:857    [EZIP] Output verified.
    11-16 16:37:14:858    [EZIP] EZIP AHB (interrupt mode) --- completed.
    ```
3. LZ4 decompression (AHB output mode, polling mode), verify output results:
    ```c
    11-16 16:37:14:859    [EZIP] LZ4 AHB (polling mode).
    11-16 16:37:14:861    [EZIP] Output verified.
    11-16 16:37:14:863    [EZIP] LZ4 AHB (polling mode) --- completed.
    ```
3. GZIP decompression (AHB output mode, polling mode), verify output results:
    ```c
    11-16 16:37:14:865    [EZIP] GZIP AHB (polling mode).
    11-16 16:37:14:867    [EZIP] Output verified.
    11-16 16:37:14:868    [EZIP] GZIP AHB (polling mode) --- completed.
    ```
    ```{tip}
    If you have an LCD, you can enable the following configuration. In this example, the decompressed image will be sent to the LCD for display (steps 1, 2).  
    #define EXAMPLE_WITH_LCD 0 /* With LCD device. */
    ```

## Exception Diagnosis


## Reference Documents
<!-- 对于rt_device的示例，rt-thread官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考RT-Thread的[RTC文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Update Log
| Version | Date    | Release Notes   |
| ------- | ------- | --------------- |
| 0.0.1   | 10/2024 | Initial version |
|         |         |                 |
|         |         |                 |
