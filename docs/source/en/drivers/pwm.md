# PWM Device

GPTim (General Purpose Timer), LPTim (Low Power Timer), and ATIM (Advanced
Timer) can work in PWM mode for hardware timers:
- General Purpose Timer (GPTim): GPTim implements a 16-bit counter for system
  PCLK, providing 1-65536 prescaler. It has 4 input/output channels. GPTim can
  be used for PWM to generate waveform signals or measure input signals.
- Low Power Timer (LPTim): LPTim implements a 16-bit counter for system PCLK or
  low power clock, providing 1-128 prescaler. LPTim can be used for PWM to
  generate waveform signals.
- Advanced Timer (ATIM): ATIM implements a 32-bit counter for system PCLK,
  providing 1-65536 prescaler. It has 6 channels, channels 1-3 can be configured
  as input or output mode respectively, where each channel can output two
  complementary PWM signals with dead-time protection, channel 4 can be
  configured as input or output mode and can output single PWM, channels 5-6 can
  be configured as output compare mode.

Hardware PWM driver includes two layers: hardware access layer (HAL) and
RT-Thread adaptation layer.<br> HAL provides basic APIs for accessing hardware
timer peripheral registers. For details, please refer to the API documentation
of hardware timer HAL.<br> The adaptation layer provides support for RT-Thread
driver framework. Users can use RT-Thread POSIX driver interface for PWM
programming. Device names like pwm3, pwm4, pwmlp1, and pwmlp3 are configured in
menuconfig.

## Driver Configuration

Select the PWM devices to use in the {menuselection}`On-Chip Peripheral RTOS
Drivers --> Enable pwm` menu.

The following macro switches indicate that PWM3, PWM4, LPTIM3, PWMA1 and other
devices are enabled:
```c
#define RT_USING_PWM
#define BSP_USING_PWM 1
#define BSP_USING_PWM3 1
#define BSP_USING_PWM4 1
#define BSP_USING_PWM_LPTIM3 1
#define BSP_USING_PWMA1 1
```

## Device Names
- `pwm<x>`: x is the device number, such as `pwm2`, `pwm3`. Note that device
  numbering starts from 2, device `pwm2` corresponds to peripheral `GPTIM1`,
  `pwm3` corresponds to peripheral `GPTIM2`, and so on
- `pwmlp<x>`: x is the device number, such as `pwmlp1`, `pwmlp2`, corresponding
  to peripherals `LPTIM1` and `LPTIM2`
- `pwma<x>`: x is the device number, such as `pwma1`, corresponding to
  peripheral `ATIM1`

## Example Code

```c
// Locate and open the device
rt_device_t rt_device_find(const char *name);
  name: pwmlp1 / pwmlp3 / pwm3 / pwm4 / pwma1 ...

// Configure PWM parameters
rt_err_t rt_pwm_set(struct rt_device_pwm *device, int channel, rt_uint32_t period, rt_uint32_t pulse);
  channel: 1-4 for GPTIM; ignored for LPTIM
  period: unit: ns (range: 1 ns to 4.29 s; frequency: 1 GHz to 0.23 Hz)
  pulse:  unit: ns (pulse &lt;= period)

// Enable and disable PWM
rt_err_t rt_pwm_enable(struct rt_device_pwm *device, int channel);
rt_err_t rt_pwm_disable(struct rt_device_pwm *device, int channel);

// Configure ATIMER dead-time and break functionality
rt_err_t rt_pwm_set_brk_dead(struct rt_device_pwm *device, rt_uint32_t *bkd, rt_uint32_t dt_ns);
bkd: Pointer to the struct rt_pwm_break_dead configuration structure
dt_ns: Dead-time duration (ns). If set to 0, the dead-time defaults to the bkd-&gt;dtg value (0–1023).

// Usage Example

{
    struct rt_device_pwm *device = RT_NULL;

    device = (struct rt_device_pwm *)rt_device_find("pwm3");
    if (!device)
    {
        return;
    }

    rt_pwm_set(device, 1, 500000000, 250000000);
    if (is_comp) // Check if PWM is in complementary mode
    {
        struct rt_pwm_break_dead bkd = {0};   // TODO: Initialize BKD structure parameters.
        rt_pwm_set_brk_dead(device, (rt_uint32_t *)&amp;bkd, 100);
        rt_pwm_enable2(device, channel, 1);
    }
    else
    {
        rt_pwm_enable(device, channel);
    }
    ......
    rt_pwm_disable(device, 1);
}
```

[PWM]:
https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/pwm/pwm

## RT-Thread Reference Documentation

- [PWM Device][PWM]
