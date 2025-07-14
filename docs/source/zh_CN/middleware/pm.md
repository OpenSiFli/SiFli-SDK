# 电源管理

电源管理模块是基于RT-Thread电源管理框架开发的。 它有助于最大限度地减少用户实现低功耗功能的努力。 
除了 RT-Thread 电源管理 API，用户还可以使用该模块来实现自己的低功耗设计。 
Butterfli 中包含三个子系统（电源域），即 HPSYS（HCPU 内部）、LPSYS（LCPU 内部）和 BLESYS（BCPU 内部）。 每个子系统可以相互独立地进入自己的低功耗模式。

当前实现支持四种低功耗模式。 IDLE 是最低低功耗模式，STANDBY 是最高低功耗模式。 从 LIGHT 模式开始，subsys 在低功耗模式下无法访问。
- `PM_SLEEP_MODE_IDLE`：CPU 执行 WFE 指令，等待事件或 IRQ
- `PM_SLEEP_MODE_LIGHT`：子系统进入浅睡眠模式
- `PM_SLEEP_MODE_DEEP`：子系统进入深度睡眠模式
- `PM_SLEEP_MODE_STANDBY`：子系统进入待机模式

当进入空闲循环时，RT-Thread 电源管理框架会根据 CPU 处于空闲状态的时间来选择合适的低功耗模式。 
默认功耗模式选择方法使用 pm 策略表来决定低功耗模式。 可以覆盖默认的 pm 策略表。 并且默认的电源模式选择方法也可以由用户定义的算法代替。 
用户可以通过调用 `rt_pm_request` 来阻止系统进入高于 `PM_SLEEP_MODE_IDLE` 的低功耗模式，尽管 CPU 将长时间空闲，例如 DMA 传输正在进行，一个线程被完成信号挂起。 
如果请求 `PM_SLEEP_MODE_IDLE` 或 `PM_SLEEP_MODE_NONE`，进入空闲循环后系统将停留在 `PM_SLEEP_MODE_IDLE`。 如果请求 `PM_SLEEP_MODE_LIGHT`、`PM_SLEEP_MODE_DEEP` 或 `PM_SLEEP_MODE_STANDBY`，则不会影响低功耗模式选择结果。 
下面是空闲循环的时序图。
![图 1：空闲循环](../../assets/idle_loop.png)

pm_request 示例，演示了即使满足睡眠时间条件，如何禁止进入更高的低功耗模式。
```c
rt_device_t uart_device;
rt_err_t tx_done(rt_device_t dev, void *buffer)
{
    rt_sem_release(sema);
}

void init(void)
{
    uart_device = rt_device_find("uart1");
    rt_device_set_tx_complete(uart_device, tx_done);
    rt_device_open(uart_device, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_DMA_TX | RT_DEVICE_FLAG_DMA_RX);
}

void start_tx(void)
{
    rt_pm_request(PM_SLEEP_MODE_IDLE);
    /* start UART transmission in DMA mode */
    rt_device_write(uart_device, 0, buf, len);
    /* wait for tranmission complete 
     * though sleep time is very long (larger than the threshold of STANDBY mode), system will stay in IDLE mode 
     */
    rt_sem_take(sema, RT_WAITING_FOREVER);
    /* IDLE mode is released, then system has chance to enter higher low power mode */
    rt_pm_release(PM_SLEP_MODE_IDLE);
};
```

下面是默认的 pm 策略表。 阈值小于休眠时间的最高功耗模式被选为低功耗模式。 
休眠时间是根据操作系统计时器计算的，即查询最近的操作系统计时器超时滴答并将其与当前滴答进行比较（使用 rt_timer_next_timeout_tick() 和 rt_tick_get()）。 
例如，如果睡眠时间为 20ms，超过 15ms（PM_SLEEP_MODE_LIGHT 的阈值）但小于 25ms（PM_SLEEP_MODE_DEEP 的阈值），则选择 PM_SLEEP_MODE_LIGHT 作为低功耗模式。 
如果休眠时间为50ms，同时超过15ms和25ms，则进入`PM_SLEEP_MODE_DEEP`模式。
```c
const static pm_policy_t default_pm_policy[] =
{
    {15, PM_SLEEP_MODE_LIGHT},
    {25, PM_SLEEP_MODE_DEEP},    
    {10000, PM_SLEEP_MODE_STANDBY},
};
/** pm policy item structure */
typedef struct
{
    uint32_t    thresh;      /**< sleep time threshold in millisecond */
    uint8_t     mode;        /**< power mode is chosen if sleep time threshold is satisfied */
} pm_policy_t;
```


用于定制的 RT-Thread 电源管理 API
- `rt_pm_policy_register`：注册用户定义的策略以覆盖默认值
- `rt_pm_override_mode_select`: 注册用户定义的电源模式选择方法来覆盖默认的一种
- `rt_pm_request/rt_pm_release`：禁止进入LIGHT/DEEP/STANDBY模式
- `rt_application_get_power_on_mode`：获取系统开机模式，例如代码从待机模式启动或唤醒

PM模块API
- `pm_enable_pin_wakeup/pm_disable_pin_wakeup`：指定一个pin唤醒子系统
- `pm_get_power_mode`：获取上次低功耗模式
- `pm_get_wakeup_src`: 获取上次唤醒源
- `aon_irq_handler_hook`：AON_IRQHandler 钩子函数。用户可以重新实现这个钩子函数来执行自定义操作
- pm_shutdown：关闭系统

## 跨子系统访问
如果一个子系统处于 LIGHT、DEEP 或 STANDBY 低功耗模式，则其他子系统无法访问它。如果发生非法访问，则会触发硬故障异常。 <br>
要访问属于另一个处于低功耗模式的 subsys 的资源，它必须先唤醒另一个 subsys。
硬件邮箱可以自动唤醒目标子系统。它的驱动程序实现考虑了低功耗特性。
如果使用硬件邮箱驱动程序进行处理器间通信，则驱动程序可以确保在访问的子系统处于 ACTIVE 或 IDLE 模式时始终进行内存访问。
发送方驱动程序将数据写入自己的缓冲区并通知接收方。发送方子系统不会进入除空闲模式之外的任何低功耗模式，直到接收方消耗数据，即发送方的环形缓冲区为空。

## 配置
该模块可以在菜单 SIFLI Middleware 中启用，它会自动启用 RT_USING_PM。
```c
#define BSP_USING_PM
#define RT_USING_PM
```

![图 2：在中间件中启用低功耗支持](../../assets/pm_menu2.png)



## API参考
[bf0_pm.h](middleware-bf0_pm)
