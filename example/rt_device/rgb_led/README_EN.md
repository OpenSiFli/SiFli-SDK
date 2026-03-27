# RGBLED Example

Source path: example/rt_device/rgbled

## Supported Platforms
<!-- 支持哪些板子和芯片平台 -->
+ sf32lb52-lcd series
+ sf32lb56-lcd series
+ sf32lb58-lcd series
+ sf32lb58-lcd series
## Overview
<!-- 例程简介 -->
This example demonstrates rgbled driver based on rt-device (using rt-thread),
including:
+ rgb_ cycling RGB light display;

```{tip}
This example is based on HCPU and uses PWM DMA driver.
```

## Example Usage
<!-- 说明如何使用例程，比如连接哪些硬件管脚观察波形，编译和烧写可以引用相关文档。
对于rt_device的例程，还需要把本例程用到的配置开关列出来，比如PWM例程用到了PWM1，需要在onchip菜单里使能PWM1 -->

### Hardware Requirements
Before running this example, you need to prepare a development board supported
by this example

### menuconfig Configuration

Run the following command
```c
menuconfig --board=sf32lb52-lcd_n16r8
```
Enable PWM， 52x use PWM3 CHANNLE1, 58x use PWM4 CHANNLE4<br>\
![Enable rgbled:](./assets/menuconfig_pwm_52x.png)<br> ![Enable
rgbled:](./assets/menuconfig_pwm_58x.png)<br>

Enable RGBLED peripheral and corresponding peripheral pwm, Channel<br> ![Enable
rgbled:](./assets/Possible_error1.png) ![Enable rgbled:]{3}

**Note**: pwm setting already sets TIM configuration, check if Enable timer
configuration causes conflicts ![Enable timer:]<br>


### Compilation and Programming
Switch to the example project directory and run the scons command to execute
compilation:
```
scons -j8 --board=sf32lb52-lcd_n16r8
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
After the example starts, the serial port cyclically outputs color characters,
and the hardware rgbled also displays corresponding colors:
```
12-25 11:51:46:770    Start display color sequence!
12-25 11:51:47:266    -&gt; Black
12-25 11:51:49:261    -&gt; Blue
12-25 11:51:50:257    -&gt; Green
12-25 11:51:51:264    -&gt; Cyan
12-25 11:51:51:283    -&gt; Red
12-25 11:51:52:262    -&gt; Purple
12-25 11:51:54:278    -&gt; Yellow
12-25 11:51:55:275    -&gt; White
```



## Exception Diagnosis

1. Confirm PWM/DMA configuration status through PWM/DMA registers:
2. Enable rgb configuration error

GTIM2 register status: ![PWM_DMA]<br> DMAC1 register status:
![PWM_DMA](./assets/reg_rgled_gtim.png)<br>

## Reference Documentation
<!-- 对于rt_device的示例，rt-thread官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考RT-Thread的[RTC文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Update Log
| Version | Date    | Release Notes   |
| ------- | ------- | --------------- |
| 0.0.1   | 12/2024 | Initial version |
|         |         |                 |
|         |         |                 |
