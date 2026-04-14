# LL USART Polling Example

源码路径: example/ll/usart_polling

这是一个基于 LL（Low-Level）接口的串口轮询收发示例。示例在 SF32LB52x 平台上使用 USART2 完成初始化、接收和回显发送。

该示例使用了 SiFli-SDK 的 LL USART 能力，核心接口位于 `drivers/ll/sf32lb52x/ll_usart.h`，包括：
- `ll_usart_config_frame()`：配置数据位/校验位/停止位
- `ll_usart_config_baudrate()`：配置波特率（SF32LB52x 固定 48MHz 外设时钟）
- `ll_usart_transmit_data8()` / `ll_usart_receive_data8()`：8bit 数据收发
- `ll_usart_is_active_flag_txe()` / `ll_usart_is_active_flag_rxne()`：轮询发送和接收状态

基于此示例可以快速搭建串口命令行、串口透传、调试日志输出等基础串口应用。

术语说明：LL 表示 Low-Level，指位于 HAL 与寄存器定义之间的轻量级、无状态、header-only 驱动接口。

## 用法

下面的小节仅提供绝对必要的信息。有关配置 SiFli-SDK 及使用其构建和运行项目的完整步骤，请参阅 [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)。

## 支持的开发板

此示例可在以下开发板上运行：
- sf32lb52-lcd_n16r8

### 硬件需求

- 一块 `sf32lb52-lcd_n16r8` 开发板
- 一路 3.3V USB 转串口工具
- 杜邦线若干

连接关系如下：

| 开发板引脚 | 串口工具引脚 | 说明 |
| --- | --- | --- |
| `PA27` (`USART2_TXD`) | `RX` | 板端发送、工具端接收 |
| `PA20` (`USART2_RXD`) | `TX` | 板端接收、工具端发送 |
| `GND` | `GND` | 共地 |

### 软件需求 (可选)

- 串口终端软件（如 SSCOM、PuTTY、MobaXterm 等）

### 配置项目

本示例默认配置即可运行，无需额外打开 menuconfig 选项。如需检查配置，可执行：

```bash
source ./export.sh
scons --board=sf32lb52-lcd_n16r8 --menuconfig
```

在 SDK 根目录执行以下命令编译：

```bash
source ./export.sh
cd example/ll/usart_polling/project
scons --board=sf32lb52-lcd_n16r8 -j10
```

## 示例输出

如果您看到以下控制台输出，则示例应已正确运行：

```text
LL USART polling example started.
Type any character, it will be echoed.
```

此时请将串口工具配置为 `115200 8N1`，发送任意字符，示例会回显相同字符。

## 异常诊断

- 串口无输出：检查 `PA27/PA20/GND` 连线是否正确，确认串口工具电平为 3.3V。
- 输出乱码：确认串口工具参数为 `115200 8N1`。
- 无回显：确认串口工具 `TX` 已接开发板 `PA20`，并且没有被其他串口程序占用。

如有任何技术疑问，请在 GitHub 上提出 [issue](https://github.com/OpenSiFli/SiFli-SDK/issues)。

## 参考文档

- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
