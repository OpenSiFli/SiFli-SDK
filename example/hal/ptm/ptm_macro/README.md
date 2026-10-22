# PTM宏指令示例
源码路径:example/hal/ptm/ptm_macro
## 支持的平台
+ sf32lb57系列（PTM 仅在 57x 提供，且本 HAL 只在 HCPU 侧使能）

## 概述
本例程演示如何**不依赖汇编器**（不使用 `ptm_asm.h`）来构造 PTM(Parallel Task Machine) 任务程序：
程序就是一个普通的 `uint32_t` 数组，每条指令用 `ptm_ins.h` 里的宏拼出来，适合指令很短、
跳转偏移可以手算的场合；一旦程序里出现标号或前向引用，改用汇编器会更省事：汇编器就是
`ptm_asm.h`，用法见文档《指令与汇编器》，**例程里的参考程序是 `example/hal/ptm/ptm_asm`**
（片上自检在 `tests/ptm_selftest`，已不在例程里）。包含：
+ 直线程序：`PTM_SET()` 拼出两条指令，`CCR.END0` 指向末条指令，任务自然结束
+ 循环程序：`PTM_JMP()` 手写跳转偏移（偏移相对 JMP 自身），`JMP_NEQDEC` 做递减循环
+ 调用到的 HAL 接口：`HAL_PTM_CoreInit()`、`HAL_PTM_CoreConfig()`、`HAL_PTM_CoreLoadCode()`、
  `HAL_PTM_CoreStart()`（轮询模式）、`HAL_PTM_CoreWaitDone()`、`HAL_PTM_CoreGetReg()`、
  `HAL_PTM_CoreDeInit()`
+ 用到的指令宏与操作数：`PTM_SET()`、`PTM_JMP()`、`PTM_WAIT()`、`PTM_X`/`PTM_Y`/`PTM_B`、
  `OP_ADD`、`JMP_NEQDEC`，结果通过 `PTM_REG_X`/`PTM_REG_Y` 读回

## 例程的使用
### 硬件需求
运行该例程前，需要准备一块本例程支持的开发板（例如 spi-hdk_lb57bun3n6）。

### menuconfig配置
本示例只使用 PTM 内部的核与寄存器，不依赖外部器件，无需额外配置。

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
例程启动后，串口输出如下（两个程序各自一行结果，最后打印 `done`）：
```
PTM example: programs built with the instruction macros
(no assembler, ptm_asm.h is not used)

[1] straight line program, polling mode
    completed: X = 0x1234, Y = 0x1235

[2] loop with a hand written jump offset
    completed: X = 18 after 5 jumps

done
```
本例程是演示/参考代码：`X`、`Y` 的具体数值取决于芯片状态（例如核是否被别的程序占用、
TCM 里原有的内容），上面只是按代码逻辑推出的参考值；只要出现 `completed:` 就说明任务正常结束。

## 异常诊断

| 现象 | 原因与处理 |
| --- | --- |
| 打印 `CoreInit failed: is that core already taken?` | `core` 指定的核（这里固定为核 0）已被别的句柄占用。可把 `ptm_core.core` 设为 `PTM_CORE_ANY` 让 HAL 自动分配，或先用 `HAL_PTM_CoreDeInit()` 释放一个核。 |
| 打印 `CoreLoadCode failed`，返回 `HAL_ERROR` | 程序会覆盖别的核在 TCM 里的程序（超过 16 条指令就会侵占下一个核的窗口）。缩短程序，或放到系统总线上用 `PTM_JMP(JMP_X, ...)` 跳过去。 |
| 打印 `CoreLoadCode failed`，返回 `HAL_BUSY` | 核正在运行：程序要在启动之前装载。 |
| 打印 `the task did not complete` | `HAL_PTM_CoreWaitDone()` 超时，说明任务没有结束：请确认 `CCR.END0`/`END1` 指向了会被执行到的地址，或 `REP` 会归零。 |
| 编译报 `undefined reference`（`HAL_PTM_*` / `PTM1_BASE`） | 目标芯片不是 SF32LB57x：PTM 只在 57x 上提供，本 HAL 也只在 HCPU 侧使能。请用带 PTM 的板子编译本示例。 |

## 参考文档
+ PTM HAL 使用说明：[PTM](../../../../docs/source/zh_CN/hal/ptm.md)
+ UM5701 第 18 章（PTM）

## 更新记录

| 日期 | 变更 |
| --- | --- |
| 2026-05 | 首版，演示不使用汇编器、直接用指令宏构造 PTM 任务程序。 |
