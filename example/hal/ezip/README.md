# EZIP示例

源码路径：example/hal/ezip

## 支持的平台
<!-- 支持哪些板子和芯片平台 -->
+ sf32lb52-lcd_n16r8

## 概述
<!-- 例程简介 -->
本例程EZIP使用，包含：
+ 解压ezip格式数据。
+ 解压lz4格式数据。
+ 解压gzip格式数据。


## 例程的使用
<!-- 说明如何使用例程，比如连接哪些硬件管脚观察波形，编译和烧写可以引用相关文档。
对于rt_device的例程，还需要把本例程用到的配置开关列出来，比如PWM例程用到了PWM1，需要在onchip菜单里使能PWM1 -->

### 硬件需求
运行该例程前，需要准备一块本例程支持的开发板

### menuconfig配置


### 编译和烧录
切换到例程project目录，运行scons命令执行编译：
```
scons --board=sf32lb52-lcd_n16r8 -j32
```
运行`build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat`，按提示选择端口即可进行下载：
```
$ ./uart_download.bat

     Uart Download

please input the serial port num:5
```
关于编译、下载的详细步骤，请参考[](/quickstart/get-started.md)的相关介绍。

## 例程的预期结果
<!-- 说明例程运行结果，比如哪几个灯会亮，会打印哪些log，以便用户判断例程是否正常运行，运行结果可以结合代码分步骤说明 -->
例程启动后，串口输出如下：  
1. EZIP解压（AHB输出模式，轮询模式），校验输出结果： 
    ```c
    11-16 16:37:14:846    [EZIP]EZIP initialization OK.
    11-16 16:37:14:847    [EZIP]EZIP AHB (polling mode).
    11-16 16:37:14:849    [EZIP]Output is correct.
    11-16 16:37:14:851    [EZIP]EZIP AHB (polling mode)  --- end.
    ```
2. EZIP解压（AHB输出模式，中断模式），校验输出结果： 
    ```c
    11-16 16:37:14:854    [EZIP]EZIP AHB (intrInterrupt mode).
    11-16 16:37:14:855    msh />[EZIP]ezip_done.
    11-16 16:37:14:857    [EZIP]Output is correct.
    11-16 16:37:14:858    [EZIP]EZIP AHB (intrInterrupt mode)  --- end.
    ```
3. LZ4解压（AHB输出模式，轮询模式），校验输出结果：
    ```c
    11-16 16:37:14:859    [EZIP]LZ4 AHB (polling mode).
    11-16 16:37:14:861    [EZIP]Output is correct.
    11-16 16:37:14:863    [EZIP]LZ4 AHB (polling mode)  --- end.
    ```
3. GZIP解压（AHB输出模式，轮询模式），校验输出结果：
    ```c
    11-16 16:37:14:865    [EZIP]GZIP AHB (polling mode).
    11-16 16:37:14:867    [EZIP]Output is correct.
    11-16 16:37:14:868    [EZIP]GZIP AHB (polling mode)  --- end.
    ```
    ```{tip}
    如果有LCD，可以打开如下配置，本例程中会将解压后的图片送到LCD显示（步骤1，2）。  
    #define EXAMPLE_WITH_LCD 0 /* With LCD device. */
    ```

## 异常诊断


## 参考文档
<!-- 对于rt_device的示例，rt-thread官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考RT-Thread的[RTC文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## 更新记录
|版本 |日期   |发布说明 |
|:---|:---|:---|
|0.0.1 |10/2024 |初始版本 |
| | | |
| | | |
