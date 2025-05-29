# HWTIMER示例

源码路径：example/hal/hwtimer0

## 支持的平台
<!-- 支持哪些板子和芯片平台 -->
+ sf32lb52-lcd_n16r8


## 概述
<!-- 例程简介 -->
本例程使用GPTIM1、ATIM1和BTIM1演示HWTIMER的使用：
+ GPTIM 单次、重复中断。
+ ATIM 单次、多次中断（ATIM支持配置重复次数，完成配置次数后产生中断）。
+ BTIM 单次、重复中断。

## 例程的使用
<!-- 说明如何使用例程，比如连接哪些硬件管脚观察波形，编译和烧写可以引用相关文档。
对于rt_device的例程， 还需要把本例程用到的配置开关列出来，比如PWM例程用到了PWM1，需要在onchip菜单里使能PWM1 -->
      
### 硬件需求
运行该例程前，需要准备一块本例程 支持的开发板

### menuconfig配置


### 编译和烧录
切换到例程project目录，运行scons命令执行编译：
```
scons --board=sf32lb52-lcd_n16r8 -j8
```
`build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat`,按提示选择端口即可进行下载：
```
$ ./uart_download.bat

     Uart Download

please input the serial port num:5
```
关于编译、下载的详细步骤，请参考[](/quickstart/get-started.md)的相关介绍。

## 例程的预期结果
<!-- 说明例程运行结果，比如哪几个灯会亮，会打印哪些log，以便用户判断例程是否正常运行，运行结果可以结合代码分步骤说明 -->
例程启动后，串口输出如下：
1. GPTIM 单次模式，超时时间3.5s:
```c
12-05 21:47:16:619    GPTIM1: SINGLE. Timeout is 3.5 seconds
12-05 21:47:16:621    Prescaler:12000 PCLK:120000000 period:35000
12-05 21:47:16:623    Timer init ok
12-05 21:47:16:730    msh />
12-05 21:47:20:091    GPTIM1 timeout 3501
12-05 21:47:20:094    GPTIM1: SINGLE. END.
```

2. GPTIM 重复模式，每次超时时间3.5s，持续10次:
```c
12-05 21:47:20:097    GPTIM1: REPETITIVE. Timeout is 3.5 seconds * 10 times.
12-05 21:47:20:099    Prescaler:12000 PCLK:120000000 period:35000
12-05 21:47:20:101    Timer init ok
12-05 21:47:23:590    GPTIM1 timeout 3501
12-05 21:47:27:084    GPTIM1 timeout 3501
12-05 21:47:30:596    GPTIM1 timeout 3500
12-05 21:47:34:088    GPTIM1 timeout 3500
12-05 21:47:37:599    GPTIM1 timeout 3500
12-05 21:47:41:090    GPTIM1 timeout 3500
12-05 21:47:44:591    GPTIM1 timeout 3500
12-05 21:47:48:100    GPTIM1 timeout 3501
12-05 21:47:51:589    GPTIM1 timeout 3500
12-05 21:47:55:098    GPTIM1 timeout 3499
12-05 21:47:55:100    GPTIM1: REPETITIVE. END.
```

3. ATIM 单次模式，超时时间3.5s:
```c
12-05 21:47:55:103    ATIM1: SINGLE. Timeout is 3.5 seconds
12-05 21:47:55:104    Prescaler:12000 PCLK:120000000 period:35000
12-05 21:47:55:106    Timer init ok
12-05 21:47:58:594    ATIM1 timeout 3499
12-05 21:47:58:596    ATIM1: SINGLE. END.
```

4. ATIM 重复模式，每次超时时间3.5s，配置重复次数10，3.5s * 10次后产生中断:
```c
12-05 21:47:58:600    ATIM1: REPETITIVE. Timeout is (3.5 * 10) seconds.
12-05 21:47:58:645    Prescaler:12000 PCLK:120000000 period:35000
12-05 21:47:58:651    Timer init ok
12-05 21:48:37:107    ATIM1 timeout 38494
12-05 21:48:37:111    ATIM1: END.
```

5. BTIM 单次模式，超时时间3.5s:
```c
12-05 21:48:37:122    BTIM1: SINGLE. Timeout is 3.5 seconds
12-05 21:48:37:126    Prescaler:12000 PCLK:120000000 period:35000
12-05 21:48:37:132    Timer init ok
12-05 21:48:40:606    BTIM1 timeout 3499
12-05 21:48:40:611    BTIM1: SINGLE. END.
```

6. BTIM 重复模式，每次超时3.5s，持续10次：
```c
12-05 21:48:40:616    
12-05 21:48:40:622    BTIM1: REPETITIVE. Timeout is 3.5 seconds * 10 times.
12-05 21:48:40:627    Prescaler:12000 PCLK:120000000 period:35000
12-05 21:48:40:632    Timer init ok
12-05 21:48:44:104    BTIM1 timeout 3499
12-05 21:48:47:598    BTIM1 timeout 3499
12-05 21:48:51:113    BTIM1 timeout 3499
12-05 21:48:54:609    BTIM1 timeout 3499
12-05 21:48:58:102    BTIM1 timeout 3499
12-05 21:49:01:598    BTIM1 timeout 3500
12-05 21:49:05:110    BTIM1 timeout 3499
12-05 21:49:08:602    BTIM1 timeout 3499
12-05 21:49:12:097    BTIM1 timeout 3500
12-05 21:49:15:608    BTIM1 timeout 3500
12-05 21:49:15:612    BTIM1: REPETITIVE. END.
```


## 异常诊断
1. 计数不准
GPTIM/BTIM的预分频配置是16bit的，基于PCLK分频，需要确认预分频是否溢出16bit：
```c
tim->Init.Prescaler = HAL_RCC_GetPCLKFreq(core_id, 1) / FREQENCY; /*Prescaler is 16 bits, please select correct frequency*/
```
2. HWTIM不工作
确认RCC是否使能，比如：
```c
HAL_RCC_EnableModule(RCC_MOD_BTIM1); /* Enable btim1 rcc */
```

## 参考文档

## 更新记录

|版本 |日期 |发布说明|
|:---|:---|:---|
|0.0.1|12/2024|初始版本|
||||
||||