# PTM IO与FIFO示例
源码路径:example/hal/ptm/ptm_io
## 支持的平台
+ sf32lb57系列（PTM 仅在 57x 提供，且本 HAL 只在 HCPU 侧使能）

## 概述
本例程演示从 **CPU 侧**使用 PTM(Parallel Task Machine) 的 IO 通道与 FIFO，只在需要时启动核：
+ 像 GPIO 一样直接驱动输出通道：`HAL_PTM_SetOutputEnable()` 打开通道的输出使能，
  `HAL_PTM_SetOutput()` 写输出值，再用 `HAL_PTM_GetOutputEnable()`/`HAL_PTM_GetOutput()` 读回
+ 用 `HAL_PTM_CoreConfigIo()`（`PTM_IoFifoInitTypeDef`）描述一个核的 FIFO 与 IO 组接线：
  使能输出 FIFO、`oauto = 0`（不自动弹 FIFO，由程序自己去读）、IO 组从通道 0 开始（`obase`）
+ 用 `HAL_PTM_CoreConfig()`/`HAL_PTM_CoreLoadCode()`/`HAL_PTM_CoreStart()` 起一个不碰 FIFO 的核：
  FIFO 属于核的任务引擎，**核使能之前 FIFO 不接收数据**（只 Init 没 Start 的核被
  `HAL_PTM_CoreReset()` 写的复位位压着，写进去的数会丢），所以 CPU 侧写 FIFO 之前必须先把核跑起来
+ FIFO 是裸硬件队列：HAL 只负责读写，调用方先用 `HAL_PTM_CoreGetFifoStatus()` 查状态，
  再决定 `HAL_PTM_CorePushOFifo()` / `HAL_PTM_CorePopIFifo()`，与 UART FIFO 的约定一致；
  溢出错标志用 `HAL_PTM_CoreClearFifoFlag()`（`PTM_FIFO_FLAG_OFIFO_UNF`）清除
+ 输入 FIFO 由任务侧填充：`ififo_en` 使能 FIFO、`iauto` 让核把采样值自己推进去、`irsh` 右移采样位
  （参考 `drv_dcmi.c`）、`ilimit` 是这个自动入队的累计位阈值（**填 0 表示 32 位**，不是"每次采样都入队"）；
  CPU 只负责先查 `PTM_FIFO_FLAG_IFIFO_EMPTY` 再 `HAL_PTM_CorePopIFifo()`
+ 真正的批量传输把 FIFO 地址交给 DMA：`HAL_PTM_CoreGetFifoAddr()` 返回 `PTM_FIFO_OFIFO` 的地址
+ 引脚复用：通道要到达焊盘必须先 `HAL_PIN_Set(PAD_PAxx, PTM1_CHn, PIN_NOPULL, 1)`
+ 其他调用到的接口：`HAL_PTM_GetHandle()`、`HAL_PTM_Init()`/`HAL_PTM_DeInit()`（模块时钟）、
  `HAL_PTM_CoreInit()`、`HAL_PTM_CoreReset()`、`HAL_PTM_CoreDeInit()`；FIFO 状态位为
  `PTM_FIFO_FLAG_OFIFO_FULL`、`PTM_FIFO_FLAG_OFIFO_EMPTY`、`PTM_FIFO_FLAG_OFIFO_UNF`、
  `PTM_FIFO_FLAG_IFIFO_EMPTY`

## 例程的使用
### 硬件需求
运行该例程前，需要准备一块本例程支持的开发板（例如 spi-hdk_lb57bun3n6）。
如需在示波器上观察 IO 输出，需要先把对应引脚复用为 PTM 功能：例程第 0 步会调用
`HAL_PIN_Set()`，具体焊盘与通道由 `PTM_IO_PIN_SETUP`、`PTM_IO_PAD`、`PTM_IO_CHANNEL` 三个宏决定。
默认是 `PAD_PA35` → `PTM1_CH0`，请按自己的板子改成**空闲且支持该通道**的焊盘；如果该焊盘已被
别处占用（例如 spi-hdk 上 `PAD_PA04` 是 LCD SPI 时钟），就把 `PTM_IO_PIN_SETUP` 设为 0，
或直接在 `bsp_pinmux.c` 里复用。

### menuconfig配置
本示例只使用 PTM 内部的 IO 通道与 FIFO，不依赖外部器件，无需额外配置。

### 编译和烧录
切换到例程project目录，运行scons命令执行编译：
```
scons --board=spi-hdk_lb57bun3n6_hcpu -j32
```
运行`build_spi-hdk_lb57bun3n6_hcpu\uart_download.bat`，按提示选择端口即可进行下载：
```
$ ./uart_download.bat

     Uart Download

please input the serial port num:5
```
关于编译、下载的详细步骤，请参考[](/quickstart/get-started.md)的相关介绍。

## 例程的预期结果
例程启动后，串口输出如下（先看引脚复用与 CPU 驱动输出通道，再看两个 FIFO）：
```
PTM example: IO channels and FIFOs from the CPU side

[0] pin mux
    pad muxed to the PTM channel

[1] an output channel driven by the CPU
    enable = 0x1, value = 0x1
    value after the second write = 0x0

[2] the output FIFO
    pushed one word into OFIFO
    OFIFO is not empty and not full
    OFIFO address for the DMA: 0x500a1054

[3] the input FIFO
    read 0x0 from IFIFO

done
```
本例程是演示/参考代码：`enable`/`value` 是通道位图（`0x1` 表示 CH0）；OFIFO 的地址随核的偏移
变化（核 0 的 OFIFO = `PTM1_BASE + 0x30 + 0x24`）；第 3 步读到的值取决于采样到的引脚电平，
所以打印的值不一定是 `0x0`（引脚悬空/接地时常见 `0x0`）。关键判断点是 `pushed one word into OFIFO`
之后紧跟着 `OFIFO is not empty`，以及第 3 步能读到一行 `read ... from IFIFO`。

```{note}
第 3 步靠任务的**自动入队**把采样值推进 IFIFO，而 `ICRx.ILIMIT` 是"累计多少个采样位后自动入队"的
阈值，且**填 0 表示 32 位**（见 UM5701 第 18 章 ICRx 说明）：程序只采一次 8 通道（8 位）时永远达不到
32，于是什么都不入队，会打印 `IFIFO is empty: the task sampled nothing`。所以例程显式写
`io.ilimit = 8`，让"采 8 位就入队"这件事变成确定行为；若按参考驱动 `drv_dcmi.c` 的连续采样用法
（`ilimit = 0`，即 32 位一包），就应该等到攒够 4 次 8 位采样再读。
```

## 异常诊断

| 现象 | 原因与处理 |
| --- | --- |
| 打印 `CoreInit failed: is that core already taken?` | `core` 指定的核（这里固定为核 0）已被别的句柄占用。可把 `ptm_core.core` 设为 `PTM_CORE_ANY` 让 HAL 自动分配，或先用 `HAL_PTM_CoreDeInit()` 释放一个核。 |
| IO 通道没有输出 | 引脚需要由板级代码复用为 PTM 功能；CPU 侧输出使能要用 `HAL_PTM_SetOutputEnable()` 打开，且要先使能再写值（否则通道停在复位状态）。 |
| 没有打印 `pushed one word into OFIFO` | 输出 FIFO 已满（`PTM_FIFO_FLAG_OFIFO_FULL` 置位），`HAL_PTM_CorePushOFifo()` 被跳过：等 FIFO 有空间再写，或用 DMA 搬运。 |
| push 过了却仍打印 `OFIFO is empty` | FIFO 归属核的任务引擎：核没使能时（只 `HAL_PTM_CoreInit()` 没 `HAL_PTM_CoreStart()`，被 `HAL_PTM_CoreReset()` 写的复位位压着）FIFO 不接收数据；另外检查 `HAL_PTM_CoreConfigIo()` 里的 `ofifo_en` 是否置 1。 |
| 读不到 IFIFO 数据 | 读空 FIFO 不是定义好的操作，所以驱动先查 `HAL_PTM_CoreGetFifoStatus()`（`PTM_FIFO_FLAG_IFIFO_EMPTY` 清零）再读。数据要由任务填进来：`ififo_en` 使能 FIFO、`iauto` 让核自动推进采样值、`irsh`/`ishift`/`ilimit` 决定采样位怎么打包（8 通道采样参考 `drv_dcmi.c` 的 `irsh = 1`），或者交给 DMA。注意 `ilimit` 是自动入队的累计位阈值、**填 0 表示 32 位**：一次跑不到 32 位的程序在攒够（例如 4 次 8 位采样）之前什么都不入队，本例程用 `ilimit = 8` 让一次采样就够。 |
| OFIFO 溢出/欠载标志清不掉 | 标志必须用 `HAL_PTM_CoreClearFifoFlag()`（`PTM_FIFO_FLAG_OFIFO_UNF`）清除，该接口会按配置重建 ICR/OCR，不会丢掉配置位；直接写寄存器可能把配置位一起清掉。 |
| 编译报 `undefined reference`（`HAL_PTM_*` / `PTM1_BASE`） | 目标芯片不是 SF32LB57x：PTM 只在 57x 上提供，本 HAL 也只在 HCPU 侧使能。请用带 PTM 的板子编译本示例。 |

## 参考文档
+ PTM HAL 使用说明：[PTM](../../../../docs/source/zh_CN/hal/ptm.md)
+ UM5701 第 18 章（PTM），FIFO 位域见 ICRx/OCR 寄存器说明

## 更新记录

| 日期 | 变更 |
| --- | --- |
| 2026-05 | 首版，演示 CPU 侧驱动 IO 通道以及 OFIFO/IFIFO 的配置、读写与状态查询。 |
