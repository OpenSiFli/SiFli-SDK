# PWM Example

Source path: example\rt_device\pwm
## Supported Platforms
The example can run on the following development boards.
* sf32lb52-lcd_n16r8
* sf32lb58-lcd_n16r64n4
## Overview
* Includes examples of GPtimer outputting PWM via IO ports



## Example Usage
### Compilation and Programming
Switch to the example project directory and run the scons command to execute compilation:

```
scons --board=sf32lb52-lcd_n16r8 -j8
```

Run `build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat`, select the port as prompted to download:

```
build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat

Uart Download

please input the serial port num:5
```

For detailed steps on compilation and downloading, please refer to the relevant introduction in [](/quickstart/get-started.md).
### GPtimer Output PWM
#### Example Output Results Display:
* log output:
```
Start gtimer pwm demo!
[32m][2821] I/pwm: pwm_set:percentage:20,period:1000000,freq:1000hz
[0m][32m][2848] I/drv.pwm: psc 2, Period 60000,
[0m][32m][2866] I/drv.pwm: Pulse 12000
[0m]gtimer pwm demo end!
msh />
```
* PA09 outputs PWM waveform (default 200Hz, 20% duty cycle)

![alt text](assets/gptimer_pwm.jpg)

#### PWM Parameter Modification
* IO Output Modification

Physical position refers to the pin header position corresponding to the pin on the board

|Form Factor Name  | PWM      | CHX     | Pin (Physical Position)            |    
|--------|------------|---------------|-------------------|
|525    | GPTIM1     | CH1    | PA20 (10)                  |   
|587  | GPTIM1    | CH2  |PA51 (CONN2 28)                  |



```c

    #if defined(BSP_USING_BOARD_EM_LB525XXX)/* 52 series default PA20 (physical position 10) output */
    HAL_PIN_Set(PAD_PA20, GPTIM1_CH2, PIN_NOPULL, 1);/*GPTIM1_CH1 function*/
    #elif defined (BSP_USING_BOARD_EM_LB587XXX)/* 587 series default PA51 output */
    HAL_PIN_Set(PAD_PA51, GPTIM1_CH2, PIN_NOPULL, 1);/*GPTIM1_CH2 function*/
    #endif


```
**Note**: 
1. Except for 55x chips, can be configured to any IO with PA_TIM function to output PWM waveform
2. The last parameter of HAL_PIN_Set is hcpu/lcpu selection, 1: select hcpu, 0: select lcpu 
* PWM period, pulse width modification





## Exception Diagnosis
If the expected log and PWM waveform output don't appear, you can troubleshoot from the following aspects:
* Whether hardware connection is normal
* Whether pin configuration is correct 


## Reference Documentation
- For rt_device examples, the RT-Thread official website documentation provides more detailed explanations, you can add webpage links here, for example, refer to RT-Thread's [RTC documentation](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc)

## Update Log
|Version |Date   |Release Notes |
|:---|:---|:---|
|0.0.1 |10/2024 |Initial version |
|0.0.2 | 12/2024| 2.0|
| | | |