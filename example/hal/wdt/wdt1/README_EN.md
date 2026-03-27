# WDT1 Example

Source code path: example/hal/wdt/wdt1

## Supported Platforms
<!-- 支持哪些板子和芯片平台 -->
+ sf32lb52-lcd series
+ sf32lb56-lcd series
+ sf32lb58-lcd series

## Overview
<!-- 例程简介 -->
This example demonstrates WDT1 usage, including:
+ WDT1 configuration and enabling.
+ WDT1 feeding.
+ WDT1 timeout response.

## Example Usage
<!-- 说明如何使用例程，比如连接哪些硬件管脚观察波形，编译和烧写可以引用相关文档。
对于rt_device的例程，还需要把本例程用到的配置开关列出来，比如PWM例程用到了PWM1，需要在onchip菜单里使能PWM1 -->

### Hardware Requirements
Before running this example, prepare a development board supported by this
example

### menuconfig Configuration


### Compilation and Programming
Switch to the example project directory and run the scons command to execute
compilation:
```
scons --board=sf32lb52-lcd_n16r8 -j32
```

Run `build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat`, select the port as
prompted for download:
```
$ ./uart_download.bat

     UART Download

Please input the serial port number: 5
```
For detailed steps on compilation and download, please refer to the relevant
introduction in [](/quickstart/get-started.md).

## Expected Results of Example
<!-- 说明例程运行结果，比如哪几个灯会亮，会打印哪些log，以便用户判断例程是否正常运行，运行结果可以结合代码分步骤说明 -->
After the example starts, the serial port outputs as follows:
1. WDT1 initialization configuration and enabling successful:
```c
10-28 21:27:41:364    WDT Example:
10-28 21:27:41:366    WDT initialized successfully. Timeout: 10(s) Reload2: 60(s)
10-28 21:27:41:368    WDT_CVR0:0x50000 WDT_CVR1:0x1E0000
```
```{tip}
In this example, WDT1 working mode (`respond mode`) is configured as `interrupt and reset`, counting two rounds, with first round counting timeout of 10 seconds and second round counting timeout of 60 seconds.
```
2. Feeding the dog (every 5 seconds):
```c
10-28 21:27:46:419    Feeding watchdog.
10-28 21:27:51:332    Feeding watchdog.
10-28 21:27:56:328    Feeding watchdog.
10-28 21:28:01:339    Feeding watchdog.
10-28 21:28:06:329    Feeding watchdog.
10-28 21:28:11:360    Feeding watchdog.
10-28 21:28:16:356    Feeding watchdog.
10-28 21:28:21:368    Feeding watchdog.
10-28 21:28:26:361    Feeding watchdog.
10-28 21:28:31:373    Feeding watchdog.
```
3. After stopping feeding the dog, the first round of counting ends (10s),
   generating an interrupt:
```c
10-28 21:28:31:373    Feeding watchdog.
10-28 21:28:43:793    WDT timeout. Interrupt triggered.
```
4. The second round of counting ends (60s), resetting the system:
```c
10-28 21:29:58:197    SFBL
10-28 21:29:59:394    Serial:c2, Chip:4, Package:3, Rev:2 Reason:00000002

10-28 21:29:59:398    NAND ID 0x7070cd
10-28 21:29:59:402    det bbm table with 1, 1, 2
```


## Exception Diagnosis

1. Confirm WDT configuration status (enable status, count configuration, working
   mode) through WDT registers: ![WDT regmap](./assets/wdt_regmap.png)

2. After WDT is configured correctly, WDT timeout cannot reset the system:\
   Need to confirm whether `reboot cause` is correctly configured,
   `HAL_PMU_SetWdt()` can be used to add the corresponding WDT's `reboot cause`:
    ```c
    /* Set watchdog as reboot cause. */
    HAL_PMU_SetWdt((uint32_t)hwdt.Instance);
    ```

## Reference Documents
<!-- 对于rt_device的示例，rt-thread官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考RT-Thread的[RTC文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Update History
| Version | Date    | Release Notes   |
| ------- | ------- | --------------- |
| 0.0.1   | 10/2024 | Initial version |
|         |         |                 |
|         |         |                 |
