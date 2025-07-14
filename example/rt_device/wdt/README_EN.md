# WDT Example

Source code path: example/rt_device/wdt/wdt

## Supported Platforms
<!-- Which boards and chip platforms are supported -->
+ sf32lb52-lcd_n16r8
+ sf32lb58-lcd_n16r64n4

## Overview
<!-- Example introduction -->
This example demonstrates WDT usage based on rt-device (using rt-thread), including:
+ WDT on/off.
+ WDT feeding.
+ WDT timeout response.
```{tip}
This example is based on HCPU, using IWDT and WDT1.
```

## Example Usage
<!-- Explain how to use the example, such as connecting which hardware pins to observe waveforms, compilation and programming can reference related documents.
For rt_device examples, also need to list configuration switches used in this example, such as PWM example using PWM1, need to enable PWM1 in onchip menu -->

### Hardware Requirements
Before running this example, you need to prepare a development board supported by this example

### menuconfig Configuration

1. BSP enable WDT (automatically configures `RT_USING_WDT`):
![BSP_USING_WDT](./assets/mc_bsp_using_wdt.png)
2. Configure WDT timeout time (`WDT_TIMEOUT`):
![BSP_USING_WDT](./assets/mc_wdt_timeout.png)

### Compilation and Programming
Switch to example project directory and run scons command to execute compilation:
```
scons --board=eh-lb525 -j32
```
Run `build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat`, select port as prompted to download:
```
$ ./uart_download.bat

     Uart Download

please input the serial port num:5
```
For detailed steps on compilation and downloading, please refer to [](/quickstart/get-started.md) related introduction.

## Expected Results of Example
<!-- Explain example running results, such as which lights will turn on, what logs will be printed, to help users determine if example is running normally, running results can be explained step by step with code -->
After example startup, serial port outputs as follows:
1. WDT off then on:
```c
11-01 11:05:24:772    WDT Example.
11-01 11:05:24:813    WDT off.
11-01 11:05:24:816    WDT on.
```
```{tip}
When using rt-thread, WDT initialization configuration and enable (rt_hw_watchdog_init) are already completed by default in startup process, no need to call initialization configuration and enable separately.
```
2. Cancel feeding in idle thread:
```c
11-01 11:05:24:820    Unregister idle hook.
```
```{tip}
rt_hw_watchdog_init registers idle hook for feeding. Cancel here to demonstrate manual feeding.
```
3. Feeding (every 5 seconds):
```c
11-01 11:05:29:709    watchdog feeding.
11-01 11:05:34:699    watchdog feeding.
11-01 11:05:39:668    watchdog feeding.
11-01 11:05:44:660    watchdog feeding.
11-01 11:05:49:629    watchdog feeding.
11-01 11:05:54:600    watchdog feeding.
11-01 11:05:59:594    watchdog feeding.
11-01 11:06:04:575    watchdog feeding.
11-01 11:06:09:558    watchdog feeding.
11-01 11:06:14:553    watchdog feeding.
```
3. After stopping feeding, WDT1 timeout (`WDT_TIMEOUT`, configured in menuconfig), generating interrupt. In interrupt (`WDT_IRQHandler`) WDT1/WDT2 will be disabled, IWDT refreshed, IWDT timeout time updated (`WDT_TIMEOUT`):
```c
11-01 11:06:14:557    Stop watchdog feeding.
11-01 11:06:51:813    WDT1 timeout occurs.
11-01 11:06:51:817    WDT reconfig:
11-01 11:06:51:821      WDT1 and WDT2 has been stopped.
11-01 11:06:51:825      IWDT refreshed and set timeout time to WDT_TIMEOUT.
```
```{tip}
Provides overridable function: wdt_store_exception_information, called by WDT_IRQHandler. Can be used to save scene, reboot system, etc.
```
4. IWDT timeout resets system:
```c
11-01 11:07:35:237    SFBL
11-01 11:07:37:445    Serial:c2,Chip:4,Package:3,Rev:2  Reason:00000000
```


## Exception Diagnosis

1. Confirm WDT configuration status (enable status, count configuration, working mode) through WDT registers:
![WDT regmap](./assets/wdt_regmap.png)


## Reference Documents
<!-- For rt_device examples, rt-thread official website documentation provides detailed explanations, web links can be added here, for example, refer to RT-Thread's [RTC documentation](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Update Log
|Version |Date   |Release Notes |
|:---|:---|:---|
|0.0.1 |10/2024 |Initial version |
| | | |
| | | |