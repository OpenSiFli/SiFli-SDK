# HWTIMER Example

Source code path: example/hal/hwtimer0

## Supported Platforms
<!-- Which boards and chip platforms are supported -->
+ sf32lb52-lcd_n16r8


## Overview
<!-- Example introduction -->
This example uses GPTIM1, ATIM1, and BTIM1 to demonstrate HWTIMER usage:
+ GPTIM single-shot and repetitive interrupts.
+ ATIM single-shot and multiple interrupts (ATIM supports configurable repeat count, generating interrupts after completing the configured count).
+ BTIM single-shot and repetitive interrupts.

## Example Usage
<!-- Instructions on how to use the example, such as which hardware pins to connect to observe waveforms, compilation and programming can reference related documentation.
For rt_device examples, also need to list the configuration switches used by this example, such as PWM examples using PWM1, need to enable PWM1 in the onchip menu -->
      
### Hardware Requirements
Before running this example, you need to prepare a development board supported by this example

### menuconfig Configuration


### Compilation and Programming
Switch to the example project directory and run the scons command to compile:
```
scons --board=sf32lb52-lcd_n16r8 -j8
```
`build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat`, select the port as prompted to download:
```
$ ./uart_download.bat

     Uart Download

please input the serial port num:5
```
For detailed steps on compilation and downloading, please refer to the relevant introduction in [](/quickstart/get-started.md).

## Expected Results
<!-- Describe the example execution results, such as which LEDs will light up, what logs will be printed, so users can determine if the example is running normally. Results can be explained step by step combined with code -->
After the example starts, the serial port outputs the following:
1. GPTIM single-shot mode, timeout time 3.5s:
```c
12-05 21:47:16:619    GPTIM1: SINGLE. Timeout is 3.5 seconds
12-05 21:47:16:621    Prescaler:12000 PCLK:120000000 period:35000
12-05 21:47:16:623    Timer init ok
12-05 21:47:16:730    msh />
12-05 21:47:20:091    GPTIM1 timeout 3501
12-05 21:47:20:094    GPTIM1: SINGLE. END.
```

2. GPTIM repetitive mode, timeout time 3.5s each time, lasting 10 times:
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

3. ATIM single-shot mode, timeout time 3.5s:
```c
12-05 21:47:55:103    ATIM1: SINGLE. Timeout is 3.5 seconds
12-05 21:47:55:104    Prescaler:12000 PCLK:120000000 period:35000
12-05 21:47:55:106    Timer init ok
12-05 21:47:58:594    ATIM1 timeout 3499
12-05 21:47:58:596    ATIM1: SINGLE. END.
```

4. ATIM repetitive mode, timeout time 3.5s each time, configured repeat count 10, interrupt generated after 3.5s * 10 times:
```c
12-05 21:47:58:600    ATIM1: REPETITIVE. Timeout is (3.5 * 10) seconds.
12-05 21:47:58:645    Prescaler:12000 PCLK:120000000 period:35000
12-05 21:47:58:651    Timer init ok
12-05 21:48:37:107    ATIM1 timeout 38494
12-05 21:48:37:111    ATIM1: END.
```

5. BTIM single-shot mode, timeout time 3.5s:
```c
12-05 21:48:37:122    BTIM1: SINGLE. Timeout is 3.5 seconds
12-05 21:48:37:126    Prescaler:12000 PCLK:120000000 period:35000
12-05 21:48:37:132    Timer init ok
12-05 21:48:40:606    BTIM1 timeout 3499
12-05 21:48:40:611    BTIM1: SINGLE. END.
```

6. BTIM repetitive mode, timeout 3.5s each time, lasting 10 times:
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


## Exception Diagnosis
1. Counting inaccuracy
GPTIM/BTIM prescaler configuration is 16-bit, based on PCLK division. Need to confirm if the prescaler overflows 16-bit:
```c
tim->Init.Prescaler = HAL_RCC_GetPCLKFreq(core_id, 1) / FREQENCY; /*Prescaler is 16 bits, please select correct frequency*/
```
2. HWTIM not working
Confirm if RCC is enabled, for example:
```c
HAL_RCC_EnableModule(RCC_MOD_BTIM1); /* Enable btim1 rcc */
```

## Reference Documents

## Update Log

|Version |Date |Release Notes|
|:---|:---|:---|
|0.0.1|12/2024|Initial version|
||||
||||