# WDT1示例

源码路径：example/hal/wdt/wdt1

## 支持的平台
<!-- 支持哪些板子和芯片平台 -->
+ em-lb525
+ em-lb587

## 概述
<!-- 例程简介 -->
本例程演示WDT1使用，包含：
+ WDT1配置、使能。
+ WDT1喂狗。
+ WDT1超时响应。

## 例程的使用
<!-- 说明如何使用例程，比如连接哪些硬件管脚观察波形，编译和烧写可以引用相关文档。
对于rt_device的例程，还需要把本例程用到的配置开关列出来，比如PWM例程用到了PWM1，需要在onchip菜单里使能PWM1 -->

### 硬件需求
运行该例程前，需要准备一块本例程支持的开发板

### menuconfig配置


### 编译和烧录
切换到例程project目录，运行scons命令执行编译：
```
cons --board=em-lb525 -j32
```

运行`build_em-lb525_hcpu\uart_download.bat`，按提示选择端口即可进行下载：
```
$ ./uart_download.bat

     Uart Download

please input the serial port num:5
```
关于编译、下载的详细步骤，请参考[](/quickstart/get-started.md)的相关介绍。

## 例程的预期结果
<!-- 说明例程运行结果，比如哪几个灯会亮，会打印哪些log，以便用户判断例程是否正常运行，运行结果可以结合代码分步骤说明 -->
例程启动后，串口输出如下：
1. WDT1初始化配置、使能成功：
```c
10-28 21:27:41:364    WDT Example:
10-28 21:27:41:366    WDT init OK. Timeout: 10(s) Reload2: 60(s)
10-28 21:27:41:368    WDT_CVR0:0x50000 WDT_CVR1:0x1E0000
```
```{tip}
本例程中，配置了WDT1工作模式（`respond mode`）为`interrupt and reset`，计数两轮，第一轮计数超时10秒，第二轮段计数超时60秒。  
```
2. 喂狗（每5秒）：
```c
10-28 21:27:46:419    Watchdog feeding.
10-28 21:27:51:332    Watchdog feeding.
10-28 21:27:56:328    Watchdog feeding.
10-28 21:28:01:339    Watchdog feeding.
10-28 21:28:06:329    Watchdog feeding.
10-28 21:28:11:360    Watchdog feeding.
10-28 21:28:16:356    Watchdog feeding.
10-28 21:28:21:368    Watchdog feeding.
10-28 21:28:26:361    Watchdog feeding.
10-28 21:28:31:373    Watchdog feeding.
```
3. 停止喂狗后，第一轮计数结束（10s），产生中断：
```c
10-28 21:28:31:373    Watchdog feeding.
10-28 21:28:43:793    WDT timeout. Interrupt triggered.
```
4. 第二轮计数结束（60s），复位系统：
```c
10-28 21:29:58:197    SFBL
10-28 21:29:59:394    Serial:c2,Chip:4,Package:3,Rev:2  Reason:00000002

10-28 21:29:59:398    NAND ID 0x7070cd
10-28 21:29:59:402    det bbm table with 1, 1, 2
```


## 异常诊断

1. 通过WDT寄存器确认WDT配置状态（使能状态、计数配置、工作模式）：
![WDT regmap](./assets/wdt_regmap.png)

2. WDT配置正确后，WDT超时后不能复位系统：  
需确认是否有正确配置`reboot cause`，`HAL_PMU_SetWdt()`可用于添加对应WDT的`reboot cause`:
    ```c
    /* Add reboot cause for watchdog. */
    HAL_PMU_SetWdt((uint32_t)hwdt.Instance);
    ```

## 参考文档
<!-- 对于rt_device的示例，rt-thread官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考RT-Thread的[RTC文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## 更新记录
|版本 |日期   |发布说明 |
|:---|:---|:---|
|0.0.1 |10/2024 |初始版本 |
| | | |
| | | |
