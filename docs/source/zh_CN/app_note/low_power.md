# 低功耗使用指南

##  低功耗模式

Sifli 的两个CPU核， HCPU以及LCPU，可以分别处于不同的低功耗模式：

- `PM_SLEEP_MODE_IDLE`: <br>
    CPU 进 休闲(idle)模式，CPU停在WFI 或 WFE，系统中有高速时钟(HRC/HXT/DBLR/DLL)， 所有外设均可使能并产生中断。
- `PM_SLEEP_MODE_LIGHT`: <br>
    CPU进入浅睡(light)，CPU 停在 WFI 指令,系统中高速时钟关闭，CPU相关的外围设备停止工作,但是不掉电，系统切换到32K时钟。<br>
    可以由低功耗定时器(LPTIM)，RTC，BLE MAC(LCPU Only)，Mailbox(其他CPU)，或者特定的唤醒pin来唤醒。唤醒时间30us-100us。<br>
    唤醒之后继续在WFI之后的指令运行。    
- `PM_SLEEP_MODE_DEEP`: <br>
    和PM_SLEEP_MODE_LIGHT一样，但是系统的供电切换到RET_LDO，唤醒时间增加到100us-1ms。<br>
    唤醒之后继续在WFI之后的指令运行。
- `PM_SLEEP_MODE_STANDBY`: <br>
    CPU进入待机(standby)模式，系统中高速时钟关闭，CPU相关的外围设备掉电，RAM除了系统配置的部分，都停止供电，PIN的状态保持，系统的供电切换到RET_LDO。<br>
    可以由低功耗定时器，RTC，BLE MAC(LCPU Only)，Mailbox(其他CPU)，或者特定的唤醒pin来唤醒。唤醒时间1ms-2ms。<br>
    唤醒之后，系统会重新启动，根据AON寄存器中的低功耗模式指示，软件判断系统从standby模式启动，以便与冷启动区别。

除此之外系统还支持

- 休眠模式(hibernate) <br>
    进入这个模式，所有系统停止供电，系统切换到32K时钟，系统只能由特定的pin或者RTC来唤醒。唤醒时间 > 2ms。通常用户的关机可以使用这个模式，这样闹钟和按键可以唤醒系统。
    用户可以调用 pm_shutdown() 进入休眠模式。
- 关机模式(shutdown) <br>
    进入这个模式，所有系统停止供电，系统切换到RC10K时钟，系统只能由特定的pin来唤醒。唤醒时间 > 2ms。通常系统在运输的过程中，可以使用这个模式，这样可以达到最低功耗，按键可以唤醒系统。
    用户可以调用 HAL_PMU_EnterShutdown() 进入关机模式。
    
## 低功耗的实现

Sifli SDK在RT-Thread的PM模块基础上实现了低功耗策略，在系统的IDLE时间超过特定的设置后，可以进入不同的睡眠模式。

Sifli的HCPU和LCPU有不同的电源域，控制着各自使用的外设。为了共享CPU的外设资源，在LCPU在IDLE或者ACTIVE的情况下(未进入睡眠)，HCPU可以使用所有LCPU的外设，不过需要注意，需要避免HCPU和LCPU使用同一个外设，否则在HCPU和LCPU同时运行的时候，产生硬件资源的冲突。<br>
如果HCPU使用了LCPU的外设(这个是常用的场景)，HCPU不在低功耗的模式的时候，就必须将LCPU置于唤醒的状态，才能够正常的使用LCPU的外设。<br>
在这种情况下，如果LCPU进入待机模式, 其相关的外设都掉电，这限制了 HCPU的低功耗模式，HCPU也必须进入待机模式，从而芯片唤醒之后，首先唤醒LCPU，然后重新初始化所有的外设(包括LCPU的外设)，才能够正常使用。<br>

```{note} 
在待机模式下，HCPU会将RAM以及CPU寄存器保留在PSRAM中，唤醒以后，HCPU读取PSRAM保存的数据恢复睡眠前的运行现场。 LCPU则保留所有RAM的供电，将CPU的寄存器内容保留在RAM中，唤醒以后，LCPU读取RAM中保存的数据恢复现场。
```
 
用户可以参考 [](/middleware/pm.md) 中的configuration, 对工程打开低功耗的支持，使用不同的低功耗策略，可以进入不同的低功耗模式。系统目前默认的策咯是：
```c
const static pm_policy_t default_pm_policy[] =
{
    {15, PM_SLEEP_MODE_LIGHT},                  // 空闲超过15ms，进入浅睡模式
#ifdef PM_STANDBY_ENABLE
    {10000, PM_SLEEP_MODE_STANDBY},             // 空闲超过10s，进入待机模式
#endif /* PM_STANDBY_ENABLE */
};
```

## 外围设备的低功耗

因为在待机模式下，外围设备掉电，Sifli SDK在RT-Thread的驱动中实现了RT_DEVICE_CTRL_RESUME和RT_DEVICE_CTRL_SUSPEND，这样每个驱动都可以在进入待机模式前后保存/恢复设置。从而使外围设备可以进入低功耗，并且唤醒后，还能够正常使用原来的配置进行工作。
Sifli SDK同时还提供了接口，用来配置所有和外围设备连接的管脚，从而管理睡眠时，外围设备产生的漏电。
```c
void BSP_IO_Power_Down(int coreid);
```
这个函数在HCPU进入睡眠之前，会调用两次，第一次(coreid=CORE_ID_LCPU)在撤销LCPU的唤醒请求前（撤销唤醒请求后LCPU就可能进入低功耗模式，HCPU也就无法访问LCPU电源域的寄存器），用来关闭HCPU使用的LCPU接口管脚，第二次(coreid=CORE_ID_HCPU)在HCPU睡眠之前，关闭HCPU使用的其他管脚。
在LCPU的工程里面，这个函数在睡眠之前进调用一次，用来关闭LCPU使用的管脚。

不同的外围设备可能需要不同的管脚关闭的方式，管脚在低功耗的配置默认需要将上拉以及下拉电阻关闭，避免形成回路漏电。但是管脚的高低配置需要根据不同的设计改变。

