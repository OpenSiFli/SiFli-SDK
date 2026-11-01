# Input Capture Example

Source code path: example/hal/pulse_in
## Supported Platforms
The example can run on the following development boards:
* sf32lb52-lcd_n16r8
* sf32lb52-lcd_a128r16
* sf32lb52-nano_a128r16
* sf32lb52-nano_n16r16
* sf32lb56-lcd_a128r12n1
* sf32lb56-lcd_n16r12n1
* sf32lb58-lcd_a128r32n1_qspi
* sf32lb58-lcd_n16r32n1_qspi
## Overview
* Contains an example of GPT PWM input capture (Input Capture)
* The example first outputs a PWM waveform through a GPT as the signal source. A jumper wire then feeds that signal into the capture timer, which measures its period, frequency, pulse width and duty cycle
* PWM signal source: 52x/56x use GPTIM2_CH1, 58x uses GPTIM1_CH2
* Capture timer: 52x/56x use GPTIM1_CH1, 58x uses GPTIM3_CH2/PB01 (the GPTIM2 channel pins are not routed out on the 58x development board)
* Supports both interrupt acquisition (INPUT_CAPTURE_USE_IT=1, default) and polling acquisition (INPUT_CAPTURE_USE_IT=0)

## Example Usage
### Compilation and Programming
Switch to the example project directory and run the scons command to compile (`<board>` is the board name):
```
scons --board=sf32lb52-lcd_n16r8 -j8
```
`build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat`, select the port as prompted to download:

```
build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat

Uart Download

please input the serial port num:5
```

For detailed steps on compilation and downloading, please refer to the [Quick Start Guide](/quickstart/get-started.md).
### Configure the Project
The example needs no extra configuration; the default configuration is enough to build and run it. To inspect the project options, run:
```
sdk.py menuconfig --board=sf32lb52-lcd_n16r8
```
### Hardware Connection
After the example starts, it first outputs a PWM waveform as the signal source (default 200Hz, 20% duty cycle) and keeps it running. Short the PWM output pin to the input capture pin with a jumper wire to start measuring.

PWM output and capture pins are listed below:

|Board Name  | PWM Output | PWM Output Pin | Capture Timer | Capture Pin |
|--------|--------|---------|------------|---------|
|sf32lb52-nano    | GPTIM2_CH1 |    PA09 | GPTIM1_CH1 |    PA27 |
|sf32lb52-lcd     | GPTIM2_CH1 |    PA09 | GPTIM1_CH1 |    PA27 |
|sf32lb56-lcd     | GPTIM2_CH1 |    PA36 | GPTIM1_CH1 |    PA27 |
|sf32lb58-lcd     | GPTIM1_CH2 |    PA51 | GPTIM3_CH2 |    PB01 |

**Note**:
1. 58x uses GPTIM3_CH2 (PB01) in the LPSYS domain
2. The 4th parameter of `HAL_PIN_Set(pad, func, flags, hcpu)` is documented as the hcpu/lcpu selection, but in practice it is derived from `pad`: HPSYS pins (PA, `pad < PIN_PAD_MAX_H`) are forced to hcpu and LPSYS pins (PB, `pad >= PIN_PAD_MAX_H`) are forced to lcpu, so the value 0/1 passed in is ignored
3. Handling of the duplicate interrupt entry: the example drives the GPT through the HAL and does not use the hwtimer device, so the root `proj.conf` disables the hwtimer driver (`# CONFIG_BSP_USING_TIM is not set`). With that driver off, `drv_pwm_tim.c` takes over the `GPTIMx_IRQHandler` names (they are guarded by `!BSP_USING_TIM`), so the PWM instances that correspond to the capture timers have to be disabled as well: `PWMT1` maps to GPTIM1 and `PWMT3` maps to GPTIM3, while this example captures on GPTIM1 (52x/56x) and GPTIM3 (58x). The root `proj.conf` therefore turns `PWMT1` off (the 52x boards enable it by default, other chips leave it off, so it is a no-op for them); `PWMT3` only matters on 58x and is turned off in `sf32lb58x/proj.conf` (the 56x boards do enable `PWMT3`, but they capture on GPTIM1 and are unaffected). If you need the hwtimer device together with this example, use another timer for input capture
### Example Output Results Display
* Log output (interrupt mode, `INPUT_CAPTURE_USE_IT=1`, default):
```
SFBL
Start PWM signal source!
GPT_clock 24000000,psc 2, Period 60000,Pulse 12000
PWM signal source setup done!
Start input capture demo (interrupt mode)!
connect PWM output pin to capture input pin with a jumper wire
captured(IT): period=5000 us, freq=200 Hz, pulse=1000 us, duty=20%
captured(IT): period=5000 us, freq=200 Hz, pulse=1000 us, duty=20%
```
* Log output (polling mode, `INPUT_CAPTURE_USE_IT=0`):
```
SFBL
Start PWM signal source!
GPT_clock 24000000,psc 2, Period 60000,Pulse 12000
PWM signal source setup done!
Start input capture demo!
connect PWM output pin to capture input pin with a jumper wire
captured: period=5000 us, freq=200 Hz, pulse=1000 us, duty=20%
captured: period=5000 us, freq=200 Hz, pulse=1000 us, duty=20%
```
* The capture result is printed once per second: the signal source is a 200Hz/20% duty cycle PWM, corresponding to a period of 5000us and a pulse width of 1000us
* If the jumper wire is not connected properly, it prints once per second:
```
no capture event, check the jumper wire!
```
### Capture Principle
* PWM input capture mode: the period channel (rising edge, direct TIx capture) stores the period value, and the pulse channel (falling edge, indirect TIx capture, same pin as the period channel) stores the pulse width value
* Slave mode reset: the counter is cleared on the period edge, so CCR directly holds the period/pulse value of the current cycle, no need to compute the difference between two adjacent captures
* 52x/56x use TI1FP1 to trigger the slave mode reset (GPTIM1_CH1), 58x uses TI2FP2 (GPTIM3_CH2)
* The capture counter clock is `PCLK1/(CAPTURE_PRESCALER+1)` (currently `CAPTURE_PRESCALER=9`, i.e. divide by 10). The CCR is 16-bit, so the maximum measurable period is about `65535/(PCLK1/(CAPTURE_PRESCALER+1))`; if a higher `PCLK1` on the target chip overflows this range on long periods, increase the prescaler.
* Capture timer selection (macro definitions in main.c):
```c
#if defined(SF32LB52X) || defined(SF32LB56X)

#define CAPTURE_INSTANCE     hwp_gptim1
#define CAPTURE_CORE         CORE_ID_HCPU
#define CAPTURE_TRIGGER      GPT_TS_TI1FP1
#define CAPTURE_PERIOD_CH    GPT_CHANNEL_1
#define CAPTURE_PULSE_CH     GPT_CHANNEL_2
#define CAPTURE_PERIOD_FLAG  GPT_FLAG_CC1
#define CAPTURE_PULSE_FLAG   GPT_FLAG_CC2
#define CAPTURE_PERIOD_ACT   HAL_GPT_ACTIVE_CHANNEL_1
#define CAPTURE_PULSE_ACT    HAL_GPT_ACTIVE_CHANNEL_2
#define CAPTURE_IRQn         GPTIM1_IRQn
#define CAPTURE_IRQ_HANDLER  GPTIM1_IRQHandler

#elif defined(SF32LB58X)

#define CAPTURE_INSTANCE     hwp_gptim3
#define CAPTURE_CORE         CORE_ID_LCPU
#define CAPTURE_TRIGGER      GPT_TS_TI2FP2
#define CAPTURE_PERIOD_CH    GPT_CHANNEL_2
#define CAPTURE_PULSE_CH     GPT_CHANNEL_1
#define CAPTURE_PERIOD_FLAG  GPT_FLAG_CC2
#define CAPTURE_PULSE_FLAG   GPT_FLAG_CC1
#define CAPTURE_PERIOD_ACT   HAL_GPT_ACTIVE_CHANNEL_2
#define CAPTURE_PULSE_ACT    HAL_GPT_ACTIVE_CHANNEL_1
#define CAPTURE_IRQn         GPTIM3_IRQn
#define CAPTURE_IRQ_HANDLER  GPTIM3_IRQHandler

#endif
```
* Capture pin configuration (capture_pin_set in main.c):
```c
#if defined(SF32LB52X) || defined(SF32LB56X)
    HAL_PIN_Set(PAD_PA27, GPTIM1_CH1, PIN_NOPULL, 1);
#elif defined(SF32LB58X)
    HAL_PIN_Set(PAD_PB01, GPTIM3_CH2, PIN_NOPULL, 0);
#endif
```
### Switching between Polling and Interrupt Modes
Switch the acquisition mode with the `INPUT_CAPTURE_USE_IT` macro (at the top of main.c):
* `1` (default): interrupt mode, `HAL_GPT_IC_CaptureCallback` collects data in the interrupt, the main loop waits on a semaphore, and the log output format is `captured(IT): ...`
* `0`: polling mode, the main loop polls the capture flag register

Note: interrupt mode requires the example to define the interrupt entry of the capture timer (`CAPTURE_IRQ_HANDLER`). If the project also enables another driver that claims the same timer (such as a hwtimer instance), the interrupt entry names collide, see the notes under [Supported Platforms](#supported-platforms).

## Exception Diagnosis
If the expected logs and waveform output do not appear, troubleshooting can be performed from the following aspects:
* Whether the hardware connection is normal (whether the jumper wire correctly shorts the PWM output pin to the capture pin)
* Whether pin configuration is correct
* Whether the pin corresponds to the correct channel
* No capture data: confirm the PWM signal source is outputting normally and the jumper wire connection is firm

For any technical queries, please open an [issue](https://github.com/OpenSiFli/SiFli-SDK/issues) on GitHub.

## Reference Documents
* [SiFli-SDK Quick Start Guide](/quickstart/get-started.md)
* [GPT Peripheral Driver Guide](https://docs.sifli.com/projects/sdk/latest/en/hal/gpt.html)

## Update Log
|Version |Date   |Release Notes |
|:---|:---|:---|
|0.0.1 |9/2026 |Initial version (converted from example/hal/pwm to an input capture example) |
