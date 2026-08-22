# SDADC示例
源码路径：example/rt_device/adc/sdadc

## 概述
* 在 RT-Thread 操作系统下，SDADC 外设被虚拟成一个 `rt_device`，通过标准 RT ADC 框架
  （`rt_device_find` + `rt_adc_enable` + `rt_adc_read`）访问 `"sdadc"` 设备，实现电压采集。

* 本例程演示 **多通道扫描 + 定时器触发 + 中断通知（事件驱动）** 的完整链路：
  GPTIM3 周期性触发一轮扫描 → 硬件把各通道结果分别刷新到对应输出寄存器 → 转换完成中断触发 →
  驱动回调 `rx_indicate` 唤醒应用 → 应用逐通道读取最新电压。

## 支持的平台
例程可以运行在以下开发板。
+ sf32lb58 HDK

> **注意**：SDADC IP 仅在 SF32LB55X 与 SF32LB58X 芯片上可用，该例程基于58调试。

## SDADC 驱动支持的采集模式
SDADC 驱动支持两个相互正交的维度，通过 Kconfig 组合使用：

| 维度 | 选项 | 说明 |
|------|------|------|
| 通道数量 | `BSP_SDADC_SUPPORT_MULTI_CH_SAMPLING` | 关：单通道；开：多通道扫描（一次配置，多个通道分别采样） |
| 触发源 | `BSP_SDADC_USE_TIMER_TRIGGER` | 关：软件触发（读取时主动启动一次转换）；开：GPTIM3 定时器触发，并使能转换完成中断 |

四种组合均可用：单通道+软件触发、单通道+定时器触发、多通道+软件触发、多通道+定时器触发。
本例程默认采用 **多通道 + 定时器触发 + 中断通知**。


## 硬件连接
SDADC 通道与引脚的对应关系（驱动通道号 1~4）：

| 芯片 | CH1 | CH2 | CH3 | CH4 |
|------|-----|-----|-----|-----|
| SF32LB55X | PB_23 | PB_24 | PB_25 | PB_26 |
| SF32LB58X | PB_40 | PB_41 | PB_42 | PB_43 |

将目标 SDADC 引脚连接到外部电压源（0 ~ 参考电压），**GND 必须与开发板共地**。未接信号的
通道读数为悬空噪声，属正常现象。

## 例程的使用
### 编译和烧录
* 此例程用到了 SDADC RT 驱动，需要确认 `rtconfig.h` 中包含以下宏：

```c
#define BSP_USING_SDADC 1
#define RT_USING_ADC 1
```

只有包含了上述宏，`sifli_sdadc_init` 函数才会通过 `rt_hw_adc_register` 注册 `"sdadc"` 设备，
后续 `rt_device_find`、`rt_adc_read` 才能成功。

* 本例程的 `proj.conf` 已开启所需配置：

```
CONFIG_BSP_USING_SDADC=y
CONFIG_BSP_SDADC_USE_TIMER_TRIGGER=y       # 定时器触发 + 转换完成中断
CONFIG_BSP_SDADC_TIMER_PERIOD_MS=1000      # 定时周期（ms）
CONFIG_BSP_SDADC_SUPPORT_MULTI_CH_SAMPLING=y   # 多通道扫描
```

* 如需手动配置，可用 `menuconfig`（`your_board` 替换为你实际使用的 board 名）：

```
sdk.py menuconfig --board=your_board
```

按如图选项勾选 SDADC、多通道采样、定时器触发等选项，保存退出。
![alt text](assets/image.png)

* 切换到例程 project 目录，运行 scons 编译（以 ec-lb587 为例）：

```
scons --board=ec-lb587_hcpu -j8
```

* 运行 `build_ec-lb587_hcpu\uart_download.bat`，按提示选择串口下载。

### 例程输出结果展示
每收到一次转换完成中断通知（约等于定时周期），打印一组 4 通道电压值
（单位 0.1mV，`ch1: 6580 (658.0 mV)` 表示 658.0mV）：

```
======== SDADC Multi-Channel (IRQ-driven) Example ========
I/sdadc: SDADC ch1 enabled
I/sdadc: SDADC ch2 enabled
I/sdadc: SDADC ch3 enabled
I/sdadc: SDADC ch4 enabled
I/sdadc: SDADC multi-channel + timer trigger + interrupt, 4 channels
msh />
I/sdadc: ch1: 7025 (702.5 mV)
I/sdadc: ch2: 4761 (476.1 mV)
I/sdadc: ch3: 6618 (661.8 mV)
I/sdadc: ch4: 6391 (639.1 mV)

```

> 若持续打印 `SDADC conversion-complete notification timeout`，说明没有收到中断通知，
> 需检查定时器触发与 SDADC 中断是否正常。

## 例程流程
1. 对每个待采样通道配置 pinmux（`HAL_PIN_Set` + `HAL_PIN_Select` 切到 SDADC 模拟功能）
2. `rt_device_find("sdadc")` 查找 SDADC 设备
3. 对每个通道 `rt_adc_enable(dev, channel)` 使能对应采样槽。
4. 初始化信号量，并用 `rt_device_set_rx_indicate` 注册转换完成回调
5. 主循环 `rt_sem_take` 等待中断通知被唤醒，再对每个通道 `rt_adc_read` 读取电压


## 切换到其它采集模式
* **单通道**：在 `proj.conf` 去掉 `CONFIG_BSP_SDADC_SUPPORT_MULTI_CH_SAMPLING=y`，并把
  `main.c` 中 `g_sdadc_channels` 改为单个通道（如 `{3}`）。
* **软件触发**：去掉 `CONFIG_BSP_SDADC_USE_TIMER_TRIGGER=y`。此时没有定时器与转换完成中断，
  每次 `rt_adc_read` 会主动启动一次转换（阻塞到完成）；应用侧应改回 `rt_thread_mdelay` +
  `rt_adc_read` 的轮询方式，不要再等 `rx_indicate`。
* **只要定时器不要中断通知**：保留定时器触发，但不调用 `rt_device_set_rx_indicate`，主循环用
  `rt_thread_mdelay` + `rt_adc_read` 轮询即可。

## 异常诊断
* `find sdadc failed` — `BSP_USING_SDADC` 未启用，SDADC 驱动未编译，`"sdadc"` 设备未注册。
* `rt_adc_enable chN failed` — 通道号超出范围（有效 1~4），或 SDADC 硬件初始化失败。
* `SDADC conversion-complete notification timeout` — 没收到中断通知：确认已开启
  `BSP_SDADC_USE_TIMER_TRIGGER`（驱动会同时使能定时器触发与转换完成中断），并检查 GPTIM3 是否正常运行。
* 某通道读数恒为 0 或异常 —
  1. 检查该通道引脚是否接了信号、是否与板子共地；
  2. 检查该 pad 的 `HAL_PIN_Select` 功能选择值是否正确（本例程统一用 `10`，若某通道对不上
     请查引脚配置表确认）；
  3. 多通道模式下确认该通道已 `rt_adc_enable`（未使能的槽不会被扫描）。
  4. 确认该芯片的sdadc是否有做出厂校准，开机log有没有 `Get SDADC configure fail, use default calibration` 日志出现。

## 更新记录
|版本 |日期   |发布说明 |
|:---|:---|:---|
|0.0.1 |07/2026 |初始版本 |
