# LL CRC Example

源码路径: example/ll/crc

这是一个基于 LL（Low-Level）接口的 CRC 计算示例。示例在 SF32LB52x 平台上直接操作 CRC 外设，演示最小可运行流程：配置 `INIT/POL/CTRL`、执行 `reset`、分段喂数并读取结果。

该示例使用了 SiFli-SDK 的 LL CRC 能力，核心接口位于 `drivers/ll/sf32lb52x/ll_crc.h`，包括：
- `ll_crc_set_init()` / `ll_crc_set_poly()`：配置初值与多项式
- `ll_crc_config_ctrl()`：配置 `DATASIZE/POLYSIZE/REV_IN/REV_OUT`
- `ll_crc_reset()`：将计算状态复位到 `INIT`
- `ll_crc_push_data32()` / `ll_crc_push_data8()`：分段输入测试向量（覆盖 32-bit 主路径 + 尾字节）
- `ll_crc_read_result()` / `ll_crc_is_active_flag_done()`：读取结果与轮询完成标志

术语说明：LL 表示 Low-Level，指位于 HAL 与寄存器定义之间的轻量级、无状态、header-only 驱动接口。

## 用法

下面的小节仅提供绝对必要的信息。有关配置 SiFli-SDK 及使用其构建和运行项目的完整步骤，请参阅 [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)。

## 支持的开发板

此示例可在以下开发板上运行：
- sf32lb52-lcd_n16r8

### 硬件需求

- 一块 `sf32lb52-lcd_n16r8` 开发板
- 一路串口日志输出通道（用于查看 `rt_kprintf`）

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
cd example/ll/crc/project
scons --board=sf32lb52-lcd_n16r8 -j10
```

## 示例输出

如果您看到以下控制台输出，则示例应已正确运行：

```text
LL CRC example started.
CRC-32/MPEG-2('123456789') result=0x0376E6E7, expect=0x0376E6E7
LL CRC check PASS.
```

## 异常诊断

- `result` 与 `expect` 不一致：检查工程是否使用了最新 `drivers/ll/sf32lb52x/ll_crc.h`，并确认下载镜像正确。
- 输出 `overflow flag is set`：说明在 `DONE=0` 时过早写入了新数据，请检查喂数与轮询顺序。
- 无日志输出：确认串口日志端口连接、波特率及下载目标板选择正确。

如有任何技术疑问，请在 GitHub 上提出 [issue](https://github.com/OpenSiFli/SiFli-SDK/issues)。

## 参考文档

- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
