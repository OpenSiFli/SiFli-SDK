# PWM设备

GPTim（通用定时器）、LPTim（低功耗定时器）、ATIM(高级定时器)可以在 PWM 模式下工作，用于硬件定时器：
- 通用定时器（GPTim） GPTim 为系统 PCLK 实现了一个 16 位计数器，提供 1-65536 分频器。 它有4个输入/输出通道。 GPTim 可用于 PWM 以生成波形信号或测量输入信号。
- 低功耗定时器（LPTim） LPTim 为系统 PCLK 或低功耗时钟实现 16 位计数器，提供 1-128 分频器。 LPTim 可用于 PWM 以生成波形信号。
- 高级定时器（ATIM） ATIM 为系统 PCLK 实现了一个 32 位计数器，提供 1-65536 分频器。它有6个通道，通道1-3可分别配置为输入或输出模式，
其中每个通道可输出两路带死区保护的互补PWM，通道4可配置为输入或输出模式，可输出单路PWM，通道5~6可配置为输出比较模式。

硬件PWM驱动包括两层：硬件访问层（HAL）和RT-Thread的适配层。<br>
HAL 提供了用于访问硬件定时器外设寄存器的基本 API。 有关详细信息，请参阅硬件计时器 HAL 的 API 文档。<br>
适配层提供对 RT-Thread 驱动框架的支持。 用户可以使用 RT-Thread POSIX 驱动程序接口进行 PWM 编程。 pwm3、pwm4 和 pwmlp1 和 pwmlp3 等的设备名称在 menuconfig 中配置。 

## 驱动配置

在{menuselection}`On-Chip Peripheral RTOS Drivers --> Enable pwm`菜单中选择要使用的PWM设备。

下面的宏开关表示使能了PWM3、PWM4、LPTIM3、PWMA1等设备
```c
#define RT_USING_PWM
#define BSP_USING_PWM 1
#define BSP_USING_PWM3 1
#define BSP_USING_PWM4 1
#define BSP_USING_PWM_LPTIM3 1
#define BSP_USING_PWMA1 1
```

## 设备名称
- `pwm<x>`: x为设备编号，如`pwm2`、`pwm3`，需要注意的是设备编号从2开始，设备`pwm2`对应外设`GPTIM1`，`pwm3`对应外设`GPTIM2`，以此类推
- `pwmlp<x>`：x为设备编号，如`pwmlp1`、`pwmlp2`，对应外设`LPTIM1`和`LPTIM2`
- `pwma<x>`：x为设备编号，如`pwma1`，对应外设`ATIM1`


## 示例代码

```c
// Find and open device
rt_device_t rt_device_find(const char *name);
  name: pwmlp1 / pwmlp3 / pwm3 / pwm4 /pwma1 ...

// Set
rt_err_t rt_pwm_set(struct rt_device_pwm *device, int channel, rt_uint32_t period, rt_uint32_t pulse);
  channel: 1-4 for GPTim, ignored for LPTim
  period: unit:ns 1ns~4.29s:1Ghz~0.23hz
  pulse:  unit:ns (pulse<=period)

// Start & Stop
rt_err_t rt_pwm_enable(struct rt_device_pwm *device, int channel);
rt_err_t rt_pwm_disable(struct rt_device_pwm *device, int channel);

//atimer deadtime break set
rt_err_t rt_pwm_set_brk_dead(struct rt_device_pwm *device, rt_uint32_t *bkd, rt_uint32_t dt_ns);
bkd: struct rt_pwm_break_dead config value
dt_ns: dead time ,unit ns. if dt_ns is 0, dead time use bkd->dtg (0~1023).

// example

{
    struct rt_device_pwm *device = RT_NULL;
	
	device = (struct rt_device_pwm *)rt_device_find("pwm3");
    if (!device)
    {
        return;
    }

    rt_pwm_set(device, 1, 500000000, 250000000);
    if (is_comp) //is pwm complementary
    {
        struct rt_pwm_break_dead bkd = {0};   // TODO: set the BKD.
        rt_pwm_set_brk_dead(device, (rt_uint32_t *)&bkd, 100);
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

[pwm]: https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/pwm/pwm

## RT-Thread参考文档

- [PWM设备][pwm]
