# PWM Example

Source path: example/rt_device/pwm/pwm
## Supported Platforms
The example can run on the following development boards.
* sf32lb52-nano series
* sf32lb52-lcd series
* sf32lb56-lcd series
* sf32lb58-lcd series
* dpi/spi-hdk_lb57x series

## Overview
* Includes examples of GPtimer outputting PWM via IO ports
* Includes examples of GPtimer input capture, collecting the PWM signal output by GPTIM1_CH2 through GPTIM2_CH1 (58x: GPTIM3_CH2/PB01) to verify period and duty cycle

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

For detailed steps on compilation and downloading, please refer to the relevant introduction in [Getting Started](/quickstart/get-started.md).
### GPtimer Output PWM
#### Example Output Results Display:
* log output:
```
07-31 14:21:42:818    Start gtimer pwm + input capture demo!
07-31 14:21:42:821    pwm_set:percentage:20,period:1000000,freq:1000hz
07-31 14:21:42:822    gtimer pwm + input capture demo started!
07-31 14:21:42:823    connect PWM output pin to INCAP input pin with jumper wire
07-31 14:21:42:828    msh />

```
Input capture needs to be started manually via MSH commands. After startup, the example automatically prints the latest captured HIGH/LOW pair once per second (two lines per pair; one pair is one complete period, with duty cycle):
```
07-31 14:21:46:087 TX:incap_start //start input capture
07-31 14:21:46:096    input capture started on incap2c1
07-31 14:21:47:104    captured: pulse=200 us, HIGH
07-31 14:21:47:104    captured: pulse=800 us, LOW,  duty=20%
07-31 14:21:48:106    captured: pulse=200 us, HIGH
07-31 14:21:48:106    captured: pulse=800 us, LOW,  duty=20%
...
07-31 14:21:51:441 TX:incap_stop //stop input capture
07-31 14:21:51:453    input capture stopped

07-31 14:22:00:145 TX:pwm_set 42 1000 //set PWM duty cycle to 42%, period 1000us
07-31 14:22:00:159    pwm_set:percentage:42,period:1000000,freq:1000hz
07-31 14:22:21:616 TX:incap_start //start input capture
07-31 14:22:21:629    input capture started on incap2c1
07-31 14:22:22:637    captured: pulse=420 us, HIGH
07-31 14:22:22:637    captured: pulse=580 us, LOW,  duty=42%
...
```
* Outputs PWM waveform (default 1000Hz, 20% duty cycle)

![alt text](assets/gptimer_pwm.jpg)

#### PWM Parameter Modification
* IO Output Modification

Physical position refers to the pin header position corresponding to the pin on the board

|Form Factor Name  | PWM      | CHX     | Pin (Physical Position)            |    
|--------|------------|---------------|-------------------|
|sf32lb52-nano  | GPTIM1     | CH2    | PA20 (Physical pin is on the back of the board, requires manual flying wire)    | 
|sf32lb52-lcd    | GPTIM1     | CH2    | PA20 (10)                  |   
|sf32lb58-lcd | GPTIM1    | CH2  |PA51 (CONN2 28)                  |
|sf32lb56-lcd | GPTIM1    | CH2  |PA36 (40)                 |
|dpi/spi-hdk_lb57x | GPTIM1    | CH2  |PA51                  |

```c

    #if defined(SF32LB52X)/* 52 series default PA20 (physical position 10) output */
    HAL_PIN_Set(PAD_PA20, GPTIM1_CH2, PIN_NOPULL, 1);
    #elif defined (SF32LB58X)/* 58 series default PA51 output */
    HAL_PIN_Set(PAD_PA51, GPTIM1_CH2, PIN_NOPULL, 1);
    #elif defined (SF32LB56X)/* 56 series default PA36 output */
    HAL_PIN_Set(PAD_PA36, GPTIM1_CH2, PIN_NOPULL, 1);
    #elif defined (SF32LB57X)/* 57 series default PA51 output */
    HAL_PIN_Set(PAD_PA51, GPTIM1_CH2, PIN_NOPULL, 1);
    #endif

```
**Note**: 
1. Except for 55x chips, can be configured to any IO with PA_TIM function to output PWM waveform
2. The last parameter of HAL_PIN_Set is hcpu/lcpu selection, 1: select hcpu, 0: select lcpu 
* PWM period, pulse width modification


### GPtimer Input Capture
Input capture uses GPTIM2_CH1 (58x: GPTIM3_CH2/PB01 because the GPTIM2_CH1 pin is not routed out) to collect the PWM signal output by GPTIM1_CH2. A jumper wire is needed to connect the two pins.

#### Hardware Connection

|Form Factor Name  | PWM Output (GPTIM1_CH2) | Input Capture (GPTIM2_CH1; 58x: GPTIM3_CH2) |
|--------|---------------------|----------------------|
|sf32lb52-nano  | PA20 (pin on the back, requires flying wire)    | PA37 |
|sf32lb52-lcd   | PA20 (10)                | PA37 |
|sf32lb58-lcd   | PA51 (CONN2 28)          | PB01 |
|sf32lb56-lcd   | PA36 (40)                | PA39 |
|dpi/spi-hdk_lb57x | PA51                  | PA21 |

**Use a jumper wire to connect the PWM output pin to the input capture pin.**

**Device name: 52x/56x/57x is `incap2c1`, 58x is `incap3c2`.**

#### MSH Commands

PWM output is automatically started after the example boots. Input capture needs to be manually controlled via MSH commands:

```
pwm_set <percentage> <period_us>   # Set PWM duty cycle and period (us), e.g.: pwm_set 50 1000 (50% duty, 1KHz)
incap_start                        # Start input capture
incap_stop                         # Stop input capture
```

> The valid range of `period_us` is 1~65000: the input capture driver measures pulse widths with a 1 MHz clock and a 16-bit counter, so only pulse widths below 65535us can be measured.

#### Input Capture Output Description

After input capture is started, the example prints the latest captured pair once per second (two lines per pair: one HIGH line and one LOW/duty line; one pair is one complete period):

- `pulse=200 us, HIGH` means the high level lasts 200 microseconds
- `pulse=800 us, LOW, duty=20%` means the low level lasts 800 microseconds, duty = 200/(200+800)=20%

The capture callback (`rx_indicate`) runs in the GPTIM interrupt context and only releases a semaphore to notify the main thread; draining the buffer and printing are done in thread context, so printing never delays the capture interrupts or loses edges.

If the jumper wire is not connected after starting the capture (or the PWM duty cycle is 0%/100%, producing no edges), the following is printed once per second:

```
no capture event, check the jumper wire!
```


## Exception Diagnosis
If the expected log and PWM waveform output don't appear, you can troubleshoot from the following aspects:
* Whether hardware connection is normal
* Whether pin configuration is correct
* No input capture data: check whether the jumper wire correctly connects the PWM output pin to the input capture pin, and confirm that `incap_start` has been executed
* Incomplete input capture data display: confirm the input capture config for the chip is enabled (52x/56x/57x: `CONFIG_BSP_USING_INPUT_CAPTURE_GPTIM2=y`, 58x: `CONFIG_BSP_USING_INPUT_CAPTURE_GPTIM3=y`, configured in the chip-level `project/sf32lb5xx/proj.conf`)
* Persistent `no capture event, check the jumper wire!` hint: confirm `incap_start` has been executed and the PWM signal is normal (0%/100% duty produces no edges)


## Reference Documentation
- For rt_device examples, the RT-Thread official website documentation provides more detailed explanations. For example, refer to RT-Thread's [PWM device documentation](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/pwm/pwm)

## Update Log
|Version |Date   |Release Notes |
|:---|:---|:---|
|0.0.1 |10/2024 |Initial version |
|0.0.2 |12/2024 |2.0|
|0.0.3 |9/2026 |Add input capture (GPTIM2_CH1; GPTIM3_CH2 on 58x) and 57x board support; move capture printing to thread context, printing the latest pair once per second |
