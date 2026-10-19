# PTM多核与中断示例
源码路径:example/hal/ptm/ptm_multi
## 支持的平台
+ sf32lb57系列（PTM 仅在 57x 提供，且本 HAL 只在 HCPU 侧使能）

## 概述
本例程演示 PTM(Parallel Task Machine) HAL 的多核与中断用法：
+ 一个主核和一个辅助核用 `HAL_PTM_CoreStartMask()` 在同一个 hclk 周期里启动：`core_mask` 决定
  哪些核被启动，`it_mask` 决定哪些核向 CPU 上报（辅助核无人等待，保持静默）
+ 主核的完成事件走回调（`HAL_PTM_CoreRegisterCallback()`）而不是轮询，回调由
  `HAL_PTM_IRQHandler(core)` 从中断里分发；向量入口 `PTM1_CORE0_IRQHandler`/`PTM1_CORE1_IRQHandler`
  由应用自己提供，RT-Thread 版本里配对调用 `rt_interrupt_enter()`/`rt_interrupt_leave()`
+ 主核用 `PTM_WAIT(WAIT_EVT1, ...)` 等待辅助核通过 `PTM_EVTS` 置位的 EVT1，辅助核用
  `CCR.REP = 255`（永远重复）一直跑，最后由 CPU 用 `HAL_PTM_CoreStop()` 停掉并读 `PTM_REG_Y`
+ 出错后的恢复流程：程序用 `PTM_LDM(PTM_X, 0xDEADBEEF)` 读一个不在总线上的地址（总线错误），
  核中止后错误回调被调用，`HAL_PTM_CoreGetStatus()` 给出出错时的 `PC`；错误标志是粘滞的，
  必须 `HAL_PTM_CoreReset()` 清标志后再 `HAL_PTM_CoreStart_IT()` 重新启动
  + PTM 的 `ERRn` 只由**总线错误**（`LDM`/`STM` 访问非法地址）和**非法指令**触发；
    `JMP` 跳到不存在的地址**不会**报错，所以错误程序不能用跳转来制造
  + 出错时硬件会清掉 `CER.ENn`，核自己停下来，PC 停在出错的那条指令上
+ 其他调用到的接口：`HAL_PTM_GetHandle()`、`HAL_PTM_CoreInit()`、`HAL_PTM_CoreConfig()`、
  `HAL_PTM_CoreLoadCode()`、`HAL_PTM_CoreDeInit()`

## 例程的使用
### 硬件需求
运行该例程前，需要准备一块本例程支持的开发板（例如 spi-hdk_lb57bun3n6）。

### menuconfig配置
本示例只使用 PTM 内部的核与中断，不依赖外部器件，无需额外配置。

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
例程启动后，串口输出如下（先看多核协作，再看错误恢复）：
```
PTM example: several cores, interrupt reporting

[1] master and helper started in the same cycle
    the master completed and called its event callback
    helper stopped after Y = 4213 passes

[2] aborted task and recovery
    error callback called, PC = 0x0
    recovered: the core runs and completes again

done
```
本例程是演示/参考代码：辅助核的 `Y`（跑过的圈数）取决于 `HAL_PTM_CoreStop()` 的时机——本核在
"主核上报完成"之后才被停掉，而这段时间由 CPU 轮询/延时决定，所以实测值在数千到数万之间波动
（两次上板分别是 `4213` 和 `48689`），不必与上面的数字一致。关键判断点是
`the master completed...`、`error callback called...` 和 `recovered: ...` 三行都出现；
`PC` 是出错指令在核里的地址（本例出错指令是第 0 条，所以是 `0x0`）。

## 异常诊断

| 现象 | 原因与处理 |
| --- | --- |
| 打印 `CoreInit(master) failed` 或 `CoreInit(helper) failed` | 指定的核（核 0 / 核 1）已被别的句柄占用。可改用 `PTM_CORE_ANY`，或先用 `HAL_PTM_CoreDeInit()` 释放核。 |
| 打印 `CoreLoadCode failed`，返回 `HAL_ERROR` | 程序会覆盖别的核在 TCM 里的程序（超过 16 条指令就会侵占下一个核的窗口）。缩短程序，或放到系统总线上用 `PTM_JMP(JMP_X, ...)` 跳过去。 |
| 打印 `CoreLoadCode failed`，返回 `HAL_BUSY` | 核正在运行：程序要在启动之前装载。 |
| 打印 `CoreStartMask failed` | 启动掩码里有核当前不可用（已被占用或句柄未初始化）。先确认所有核都 `HAL_PTM_CoreInit()` 成功，再检查 `core_mask`/`it_mask` 是否与句柄一一对应。 |
| 打印 `no completion event within 1 s` 或 `no error reported within 1 s` | 1 s 内没等到回调：完成事件是用 `EVTn` 上报的，请用 `HAL_PTM_CoreStart()`/`HAL_PTM_CoreStart_IT()` 启动（它们会打开完成事件），并确认程序确实会结束（`CCR.END0`/`END1` 或 `REP`）。 |
| 错误程序不触发（打印 `no error reported within 1 s`） | `ERRn` 只由总线错误（`LDM`/`STM` 访问非法地址）和非法指令触发：越界的 `JMP`、跳到不存在的地址都不报警。写错误程序请照本例的 `PTM_LDM(PTM_X, 0xDEADBEEF)`，寄存器级用例见 `example/psram_test/src/csim/ctest/ptm/ptm1_err.c`。 |
| `END0` 指向新程序之外 | 装载程序不会清 TCM 的其余内容，`END0` 必须落在新程序内部，否则核会继续执行上一个程序留下的指令。 |
| 错误回调被反复进入 | 错误标志是电平式的，本 HAL 在调用错误回调之前已经屏蔽了该核的错误中断。要恢复运行，先 `HAL_PTM_CoreReset()` 清标志，再重新启动。 |
| 编译报 `undefined reference`（`HAL_PTM_*` / `PTM1_BASE`） | 目标芯片不是 SF32LB57x：PTM 只在 57x 上提供，本 HAL 也只在 HCPU 侧使能。请用带 PTM 的板子编译本示例。 |

## 参考文档
+ PTM HAL 使用说明：[PTM](../../../../docs/source/zh_CN/hal/ptm.md)
+ UM5701 第 18 章（PTM）

## 更新记录

| 日期 | 变更 |
| --- | --- |
| 2026-05 | 首版，演示多核同时启动、中断回调上报与出错后的恢复流程。 |
