# HWTIMER设备

有 3 种不同类型的硬件定时器：
- 通用定时器 (GPTim) BTim 为系统 PCLK 实现一个 16 位计数器，提供 1-65536 分频器。具有4个输入/输出通道，可独立配置为输入/输出模式。 GPTim 可用于 PWM 以生成波或测量输入信号。
- Basic Timer (BTim) BTim 为系统 PCLK 实现了一个 32 位计数器，提供 1-65536 分频器。它可以用作定时器或通过其触发输出驱动 DAC。
- 低功耗定时器（LPTim） LPTim 为系统 PCLK 或低功耗时钟实现 16 位计数器，提供 1-128 分频器。用于系统休眠/唤醒，可独立于系统时钟运行，并在系统进入休眠状态后提供唤醒信号。

## 驱动配置

在{menuselection}`On-Chip Peripheral RTOS Drivers --> Enable Timer`菜单中选择要使用的HWTIMER设备

下面的宏开关表示注册了LPTIM1、BTIM1、GPTIM2 和 GPTIM3几个设备
```c
#define BSP_USING_TIM
#define BSP_USING_LPTIM1
#define BSP_USING_BTIM1
#define BSP_USING_GPTIM2
#define BSP_USING_GPTIM3
```

## 设备名称
- `gptim<x>`: `x`为设备编号，如`gptim1`和`gptim2`，分别对应外设`GPTIM1`和`GPTIM2`
- `btim<x>`：`x`为设备编号，如`btim1`、`btim2`，分别对应外设`BTIM1`和`BTIM2`
- `lptim<x>`: `x`为设备编号，如`lptim1`、`lptim2`，分别对应外设`LPTIM1`和`LPTIM2`
- `atim<x>`: `x`为设备编号，如`atim1`，对应外设`ATIM1`


## 示例代码

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
## RT-Thread参考文档

- [HWTIMER设备][hwtimer]