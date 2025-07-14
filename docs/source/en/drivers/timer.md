# HWTIMER Device

There are 3 different types of hardware timers:
- General Purpose Timer (GPTim): BTim implements a 16-bit counter for system PCLK, providing 1-65536 prescaler. It has 4 input/output channels that can be independently configured as input/output mode. GPTim can be used for PWM to generate waveforms or measure input signals.
- Basic Timer (BTim): BTim implements a 32-bit counter for system PCLK, providing 1-65536 prescaler. It can be used as a timer or drive DAC through its trigger output.
- Low Power Timer (LPTim): LPTim implements a 16-bit counter for system PCLK or low power clock, providing 1-128 prescaler. Used for system sleep/wake-up, can operate independently of system clock and provide wake-up signal after system enters sleep state.

## Driver Configuration

Select the HWTIMER devices to use in the {menuselection}`On-Chip Peripheral RTOS Drivers --> Enable Timer` menu.

The following macro switches indicate that LPTIM1, BTIM1, GPTIM2, and GPTIM3 devices are registered:
```c
#define BSP_USING_TIM
#define BSP_USING_LPTIM1
#define BSP_USING_BTIM1
#define BSP_USING_GPTIM2
#define BSP_USING_GPTIM3
```

## Device Names
- `gptim<x>`: `x` is the device number, such as `gptim1` and `gptim2`, corresponding to peripherals `GPTIM1` and `GPTIM2` respectively
- `btim<x>`: `x` is the device number, such as `btim1`, `btim2`, corresponding to peripherals `BTIM1` and `BTIM2` respectively
- `lptim<x>`: `x` is the device number, such as `lptim1`, `lptim2`, corresponding to peripherals `LPTIM1` and `LPTIM2` respectively
- `atim<x>`: `x` is the device number, such as `atim1`, corresponding to peripheral `ATIM1`

## Example Code

```c
// Find and open device
rt_device_t timer_dev = rt_device_find("gptim2");
rt_err_t err = rt_device_open(timer_dev, RT_DEVICE_FLAG_RDWR);

// Configure Timer
int freq=1000000;
rt_device_control(timer_dev, HWTIMER_CTRL_FREQ_SET, (void *)&freq);	
int mode=HWTIMER_MODE_ONESHOT;
rt_device_control(timer_dev, HWTIMER_CTRL_MODE_SET, (void *)&mode);	

// Prepare for timeout
rt_device_set_rx_indicate(timer_dev, timeout_ind);

// Start timer
rt_hwtimerval_t t={3,500}; // 3 seconds and 500 microseconds
ret = rt_device_write(timer_dev, 0, &t, sizeof(t));

...

// Interrupt callback
static rt_err_t timeout_ind(rt_device_t dev, rt_size_t size)
{
    rt_kprintf("Timeout \n");
}

```

[hwtimer]: https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/hwtimer/hwtimer

## RT-Thread Reference Documentation

- [HWTIMER Device][hwtimer]