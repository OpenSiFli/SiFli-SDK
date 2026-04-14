# LL GPIO IRQ Example

源码路径: example/ll/gpio_irq

这是一个基于 LL（Low-Level）接口的 GPIO 中断示例。示例通过软件周期翻转 PA41 输出，并将 PA42 配置为双边沿中断输入，演示 GPIO 中断触发与状态读取。

该示例使用了 SiFli-SDK 的 LL GPIO 能力，核心接口位于 `drivers/ll/sf32lb52x/ll_gpio.h`，包括：
- `ll_gpio_pa_set_low()` / `ll_gpio_pa_toggle()`：基于绝对 PA 编号控制输出
- `ll_gpio_pa_config_irq_trigger()`：配置中断触发类型与极性
- `ll_gpio_pa_enable_irq()` / `ll_gpio_pa_clear_irq_pending()`：中断使能与清除
- `ll_gpio_pa_is_irq_pending()` / `ll_gpio_pa_is_input_high()`：中断与输入电平查询

基于此示例可以扩展出按键中断检测、脉冲计数、GPIO 事件采集等应用。

术语说明：LL 表示 Low-Level，指位于 HAL 与寄存器定义之间的轻量级、无状态、header-only 驱动接口。

## 用法

下面的小节仅提供绝对必要的信息。有关配置 SiFli-SDK 及使用其构建和运行项目的完整步骤，请参阅 [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)。

## 支持的开发板

此示例可在以下开发板上运行：
- sf32lb52-lcd_n16r8

### 硬件需求

- 一块 `sf32lb52-lcd_n16r8` 开发板
- 一根跳线

连接关系如下：

| 信号 | 引脚 | 说明 |
| --- | --- | --- |
| GPIO 输出 | `PA41` | 由示例每 1 秒翻转一次 |
| GPIO 输入中断 | `PA42` | 配置为双边沿中断 |
| 跳线连接 | `PA41` -> `PA42` | 通过输出翻转触发输入中断 |

### 软件需求 (可选)

- 无额外软件要求

### 配置项目

本示例默认配置即可运行，无需额外打开 menuconfig 选项。如需检查配置，可执行：

```bash
source ./export.sh
scons --board=sf32lb52-lcd_n16r8 --menuconfig
```

在 SDK 根目录执行以下命令编译：

```bash
source ./export.sh
cd example/ll/gpio_irq/project
scons --board=sf32lb52-lcd_n16r8 -j10
```

## 示例输出

如果您看到以下控制台输出，则示例应已正确运行：

```text
LL GPIO IRQ example started.
Please connect PA41 to PA42 with a jumper.
LL GPIO IRQ count=1, out=1, in=1
LL GPIO IRQ count=2, out=0, in=0
```

运行时 `count` 会持续累加，`out` 与 `in` 电平会随翻转交替变化。

## 异常诊断

- 中断计数不变化：确认 `PA41` 与 `PA42` 已正确短接。
- 输出和输入电平不一致：检查跳线接触是否稳定，避免虚接。
- 无日志输出：确认工程已正确下载并从串口查看系统日志。

如有任何技术疑问，请在 GitHub 上提出 [issue](https://github.com/OpenSiFli/SiFli-SDK/issues)。

## 参考文档

- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
