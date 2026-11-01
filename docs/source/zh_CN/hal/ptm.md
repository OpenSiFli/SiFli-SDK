# PTM

PTM(Parallel Task Machine)是一个能够并行处理多个任务的小型处理器。PTM 内部包含 4 个独立运行的处理核，每个核能够从内置存储器(TCM)或系统总线获取自定义格式的指令并执行，实现总线地址读写、数据运算、判断跳转、IO 读写、等待触发、延时等功能，可用于自定义接口协议、自定义 PWM、内存数据批量运算、多模块互联调度、自动传感器采集等场景。

- 4 个处理核并行运行，两级流水线，1 个运行时钟执行 1 条基础指令
- 32 位自定义指令集：4 条基础指令(SET/WAIT/JMP/IO) + 4 条访存指令(LDM/LDR/STM/STR)
- TCM 共 64 条指令，CoreN 的地址 0 对应 TCM[16 x N]，因此每个核在 TCM 上最多 16 条指令；跳到总线地址执行时指令数无上限
- 每个核内置 2 个 32 位(X/Y)和 2 个 16 位(A/B)通用寄存器
- 每个核内置带 DMA 接口的 32 位收发 FIFO(IFIFO/OFIFO)，深度 4 或 8
- 16 个 IO 通道 CH0~CH15，每个核可同时处理 16 路 IO 输入输出，并单独控制其中 2 路
- 16 个事件通知 EVT0~EVT15 用于多核协同，其中 EVT0~EVT3 可产生 CPU 中断
- 每个核可独立复位、启动、停止、单步

```{note}
PTM 仅在 SF32LB57x 上提供，且本 HAL 只在 HCPU 侧使能（`bf0_hal_conf_hcpu.h`）：每核的中断号只定义在 HCPU 的 IRQn 表里（LCPU 侧没有）。
```

## 句柄模型

PTM 只有一个硬件实例、却可能被多个驱动共用，因此 HAL 分两层句柄：

| 句柄 | 谁定义 | 作用 |
| --- | --- | --- |
| `PTM_HandleTypeDef` | HAL 内部单例，用 `HAL_PTM_GetHandle()` 获取 | 时钟与复位的引用计数、核占用位图、全局 EVT/CH 接口 |
| `PTM_CoreHandleTypeDef` | 使用者自己定义（静态存储期，或嵌在 device 结构体里） | 一个核的配置、回调与状态 |

驱动通常只需要 `HAL_PTM_CoreInit()` / `HAL_PTM_CoreDeInit()` 这一对：它们内部会完成模块的引用计数与核的占用、复位。因此第二个使用者初始化时**不会**复位已经在运行的核。

```{warning}
核句柄必须有静态存储期（或随 device 对象一起存在）：HAL 与中断处理都保存着指向它的指针，不能是函数局部变量。
```

## 使用 PTM

```c
static PTM_CoreHandleTypeDef ptm_core;              /* 静态存储期 */

/* 程序：X 清零后自增，永久循环。
   与 tests/ptm_selftest 的 test_run_and_count 是同一段程序（那边用汇编器拼，
   这里直接用 ptm_ins.h 的宏写）。JMP 的相对偏移按"目标 = JMP 自身地址 + Tgt"
   自己算，这里的 -1 就是从 word 2 回到 word 1。 */
static const uint32_t ptm_code[] =
{
    PTM_SET(PTM_X, PTM_ZERO, OP_ADD, 0),            /* 0: X = 0 */
    PTM_SET(PTM_X, PTM_X, OP_ADD, 1),               /* 1: X++（循环体） */
    PTM_JMP(-1, PTM_ZERO, JMP_EQ, PTM_ZERO, 0),     /* 2: 0 == 0 恒成立，即无条件回到 1 */
};

void ptm_demo_start(void)
{
    PTM_CoreCfgTypeDef cfg = {0};

    /* 1. 占用并复位一个核：core 填核号，或填 PTM_CORE_ANY 由 HAL 自动分配 */
    ptm_core.core = 0;
    if (HAL_PTM_CoreInit(&ptm_core) != HAL_OK)
    {
        return;
    }

    /* 2. 装载到该核的 TCM 窗口（内部按 CoreN 地址 0 = TCM[16*N] 换算） */
    if (HAL_PTM_CoreLoadCode(&ptm_core, ptm_code,
                             sizeof(ptm_code) / sizeof(ptm_code[0])) != HAL_OK)
    {
        return;
    }

    /* 3. 配置 CCRx：分频、重复次数、结束地址。
       这段程序靠 JMP 自己维持循环，所以 REP 与 END0/END1 都保持 0
       （0 表示该地址不做结束判定：循环来自指令，不依赖 REP）。 */
    cfg.psc  = 255;                                 /* 分频：X 涨得慢一些，便于 CPU 读回观察 */
    cfg.rep  = 0;
    cfg.end0 = 0;
    cfg.end1 = 0;
    HAL_PTM_CoreConfig(&ptm_core, &cfg);

    /* 4. 启动（轮询模式；需要中断上报就用 HAL_PTM_CoreStart_IT()，见下一节） */
    HAL_PTM_CoreStart(&ptm_core);
}

/* 运行期间随时可读回：HAL_PTM_CoreGetReg(&ptm_core, PTM_REG_X)
   停止并释放：        HAL_PTM_CoreDeInit(&ptm_core);                        */
```

需要 IO 通道输出 / FIFO 收发的完整例子见 `example/hal/ptm/ptm_io`。

## 中断与事件

PTM 的中断路由是硬件固定的：**核 n 的 `IER.EVTIEx`(x = n) 使能后，`EVTn` 置位会在 `PTM1_COREn_IRQn` 上产生中断**。因此程序里用哪个 EVT 通知 CPU，就必须用哪个核来跑。

```c
/* 中断路由固定为 核n <-> EVTn，所以每个核有自己的向量入口 */
void PTM1_CORE0_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_PTM_IRQHandler(0);                          /* 清标志 + 回调 */
    rt_interrupt_leave();
}

static void ptm_event_cb(PTM_CoreHandleTypeDef *hcore, uint16_t evt_mask)
{
    /* EVT 标志已由 HAL 清除，这里只做业务处理 */
}

void ptm_demo_irq_start(void)
{
    HAL_PTM_CoreRegisterCallback(&ptm_core, ptm_event_cb, NULL);
    HAL_PTM_CoreStart_IT(&ptm_core);      /* 中断版启动：跑完/出错自动进回调 */
}
```

```{note}
`rt_interrupt_enter()` / `rt_interrupt_leave()` 是 RT-Thread 运行环境的要求（裸机环境不需要）。HAL 只提供分发函数 `HAL_PTM_IRQHandler(core)`，中断入口符号由使用者自己定义，这样同一个入口可以被多个模块共用。
```

- **启动方式二选一**：`HAL_PTM_CoreStart()` 是**轮询版**（只打开"任务结束自动置事件"，该核的中断全部关闭），`HAL_PTM_CoreStart_IT()` 是**中断版**（打开"完成事件 + 事件中断 + 错误中断"，并使能该核的 NVIC 线）。寄存器差异全部由 HAL 处理，使用者不需要碰 `IER`。
- **多核同时启动**用 `HAL_PTM_CoreStartMask(hptm, core_mask, it_mask)`：`core_mask` 里所有核由**一次 `CER` 写**在同一 hclk 周期启动，`it_mask` 选出其中哪些核走中断模式，其余走轮询模式。所以"主核 + 配合核"这种搭配可以直接表达——配合核不需要任何中断，就不会多进一次 ISR：
  ```c
  HAL_PTM_CoreStartMask(HAL_PTM_GetHandle(), (1U << 0) | (1U << 1), (1U << 0));
  /* core0/core1 同周期启动；只有 core0 用完成/事件/错误中断，core1 只置完成事件不产生中断 */
  ```
- 中断模式下，程序里任意时刻 `SET EVTS` 置位 `EVT<core>` 都会进回调（不区分"中途"还是"结束"）；若需要区分，在回调里用 `HAL_PTM_CoreIsRunning()` / `HAL_PTM_CoreGetStatus()` 看核是否已经停止。
- 程序侧也可以主动通知：`PTM_SET(PTM_EVTS, PTM_ZERO, OP_OR, 1 << 0)` 置 `EVT0`，`PTM_WAIT(WAIT_EVT0, 1, 0)` 等待 `EVT0`。
- CPU 侧：`HAL_PTM_GetEvent()` 读取全部事件，`HAL_PTM_ClearEvent()` 按位清标志；**CPU 不能置事件**（`ISR.EVTx` 只能写 1 清 0）。
- `EVT4~EVT15` 在 57x 上不能产生中断、也不会在任务结束时自动置位：57x 只实现 `EVTIE0~EVTIE3` 与 `EVTAUTO0~EVTAUTO3`（`IER` 的 bit[7:4] 与 bit[15:12] 是 RSVD；`ptm.h` 里的 `EVTIE4~EVTIE7` / `EVTAUTO4~EVTAUTO7` 是 IP 的通用定义，57x 不可用）。但标志位本身完全可用：可以给**核间同步/握手**（一个核 `SET EVTS` 置位，另一个核用 `PTM_WAIT(WAIT_EVTn, 1, 0)` 等待），也可以给 CPU 当状态位轮询（`HAL_PTM_GetEvent()` 读、`HAL_PTM_ClearEvent()` 清）。要在 CPU 侧等这些事件，就用 `HAL_PTM_GetEvent()` 自己判断。
- 事件中断必须显式使能：只置 `EVTn` 标志不会产生中断，要用 `HAL_PTM_CoreStart_IT()`（或 `HAL_PTM_CoreStartMask()` 的 `it_mask` 里带上该核）启动——它们会打开该核的 `IER.EVTIE<n>`；而指令里的 `PTM_WAIT(WAIT_EVTn, ...)` 不需要任何使能，它直接读标志位。
- 57x 每核的错误中断与事件中断共用同一条 NVIC 线，`HAL_PTM_CoreStart_IT()` 会一并打开错误中断；轮询模式则通过 `HAL_PTM_CoreGetStatus()` 的 `error` / `tcm_conflict` 发现错误。

### 轮询模式

用 `HAL_PTM_CoreStart()`（轮询版）启动后，不使用中断，可以用阻塞式轮询接口（内部用 `HAL_GetTick()` + `HAL_Delay(1)` 实现超时，只能在任务上下文调用）：

```c
/* 等本次运行结束：判据与中断模式完全一致（完成事件 / 错误标志） */
switch (HAL_PTM_CoreWaitDone(&ptm_core, 1000))
{
case HAL_OK:      break;      /* 正常完成（硬件置了 EVT<core>） */
case HAL_ERROR:   break;      /* 异常中止（ISR.ERRx / TCMx），明细用 CoreGetStatus() 取 */
case HAL_TIMEOUT: break;      /* 超时：程序还没结束，或被 CoreStop() 主动停了 */
default:          break;
}

/* 等程序中途发出的信号（成功后自动清标志） */
HAL_PTM_CoreWaitEvent(&ptm_core, 100);
```

`HAL_PTM_CoreWaitDone()` 轮询的是 `ISR.EVT<core>` 与 `ISR.ERRx/TCMx`，**不是** `CER.ENx`——因为 `EN=0` 有歧义（正常结束、被 `CoreStop()` 停、或压根没启动），而这两个标志才是硬件对"本次运行结果"的表达。它**不消费**事件位，所以之后仍可用 `HAL_PTM_CoreWaitEvent()`/`HAL_PTM_GetEvent()` 观察到同一次完成；同时 `HAL_PTM_CoreStart*()` 会先清掉本核残留的事件位，保证`WaitDone()` 不会被上一次运行的完成事件误触发。

需要等 FIFO 状态时用 `HAL_PTM_CoreGetFifoStatus()` 自己判断；更推荐的做法是让 DMA 搬数据，或让程序在数据就绪时用指令 `SET EVTS` 置事件，再用 `HAL_PTM_CoreWaitEvent()` 等（这样一次等待覆盖"数据好了"这个语义，而不用去猜 FIFO 的满/空标志）。

中断上下文或自己的循环里可以用非阻塞读接口：`HAL_PTM_GetEvent()`、`HAL_PTM_CoreGetFifoStatus()`、`HAL_PTM_CoreGetStatus()`、`HAL_PTM_CoreIsRunning()`、`HAL_PTM_CoreGetReg(&ptm_core, PTM_REG_Y)`。

两种模式可以混用：中断做通知、轮询读数据。注意轮询不受"核 n <-> EVTn"的限制，CPU 可以读取任意 EVT 位。

## 多核

4 个核各自一个核句柄，独立配置与启停，互不影响：

```c
static PTM_CoreHandleTypeDef ptm_core0;
static PTM_CoreHandleTypeDef ptm_core1;

ptm_core0.core = 0;
ptm_core1.core = PTM_CORE_ANY;                      /* 自动挑一个空闲核 */
HAL_PTM_CoreInit(&ptm_core0);
HAL_PTM_CoreInit(&ptm_core1);                       /* 不会复位 core0 */
```

**同时启动多个核**：用一次寄存器写使能多个核，让它们在同一个 hclk 周期起跑；若还需要更严格的对齐，可让两个核的程序都先等待同一个触发源：

```c
/* core0/core1 的程序首条: PTM_WAIT(PTM_TRIG_GPTIM1_UPDATE, 1, 0) */
HAL_PTM_CoreStartMask(HAL_PTM_GetHandle(), (1U << 0) | (1U << 1), (1U << 0));
```

多核使用时的约束：

- **TCM**：CoreN 的地址 0 对应 TCM[16 x N]，一个核的程序超过 16 条会侵占下一个核的地址 0，该核就不能再使用。HAL 会记录每个 TCM 槽位属于哪个核的哪段程序，**双向**拒绝会互相覆盖的装载：① 往一个已占用核的窗口上写；② 先装了长程序，之后又给被它侵占的核装码。这两种情况 `HAL_PTM_CoreLoadCode()` 都返回 `HAL_ERROR` 且不写入任何字。运行期还可用 `HAL_PTM_GetTcmConflict()` 检查硬件报出的冲突标志。需要更长的程序时，把它放在总线地址上执行，再通过 `PTM_JMP(JMP_X)` 跳过去。
- **IO 通道**：CH0~CH15 由所有核共享，用 `ICRx.IBASE` / `OCRx.OBASE` 为每个核做映射（`io.ibase` / `io.obase`），这样多个核可以运行同一份指令序列去操作不同的 IO。
- **事件**：EVT0~EVT15 是全局标志，但中断路由固定为核 n <-> EVTn。

## 指令

指令字由 `ptm_ins.h` 的宏生成，HAL 只负责装载与寄存器配置，不重新实现编码：

```c
static const uint32_t blink_code[] =
{
    PTM_SET(PTM_Y, PTM_ZERO, OP_ADD, 0),
    PTM_IO(IO_PUSH8, IO_OUT0_H, IO_NOP, 0),
    PTM_WAIT(0, 0, 100),                            /* 等待源 0 的条件恒成立，即纯延时 */
    PTM_JMP(-3, PTM_ZERO, JMP_EQ, PTM_ZERO, 0),     /* 相对偏移自己算 */
};
HAL_PTM_CoreLoadCode(&ptm_core, blink_code, 4);
```

```{note}
PTM 的相对跳转基准是 **JMP 指令自身的地址**，即 `目标 = PC + Tgt`（不是下一条指令），手写 `PTM_JMP()` 的相对偏移要按这个规则算，偏移范围为 -127~127。`Tgt` 的另外两种编码：`128~255` 是 TCM 绝对下标，`256` / `257`（`JMP_X` / `JMP_Y`）表示经 `X` / `Y` 跳转（把程序放在总线地址上执行时用）。
```

## 使用 FIFO 与 DMA

接收方向：IO 数据经 `IO` 指令进入 `Y`，在 `ICRx.IAUTO` 使能后自动压入 IFIFO，CPU 用 `HAL_PTM_CorePopIFifo()` 读取，或用 DMA 搬走：

```c
hdma.Init.Request = PTM1_IFIFO1_DMA_REQUEST;        /* 见板级 dma_config.h */
HAL_DMA_Start_IT(&hdma, HAL_PTM_CoreGetFifoAddr(&ptm_core1, PTM_FIFO_IFIFO),
                 (uint32_t)buffer, words);
```

发送方向：CPU/DMA 写入 OFIFO(`HAL_PTM_CorePushOFifo()`)，在 `OCRx.OAUTO` 使能后自动取出到 `X`，再由 `IO` 指令推送到 IO 通道。

```{note}
`HAL_PTM_CorePushOFifo()` / `HAL_PTM_CorePopIFifo()` 是"硬写/硬读"，**不检查 FIFO 状态**（和 UART 的收发 FIFO 用法一致）：写之前请用 `HAL_PTM_CoreGetFifoStatus()` 确认 `PTM_FIFO_FLAG_OFIFO_FULL` 未置位，读之前确认 `PTM_FIFO_FLAG_IFIFO_EMPTY` 未置位。FIFO 满时写入、空时读出，手册都没有定义行为，必须避免。

FIFO 属于核的任务引擎，**核启动过一次（脱离复位态）之后才接收数据**：只 `HAL_PTM_CoreInit()` 过、从没 `HAL_PTM_CoreStart()`/`HAL_PTM_CoreStart_IT()` 过的核处于 `CER.RSTx` 复位态，对 FIFO 的写入会被丢弃、状态位也不变。反过来**"核正在运行"并不是必要条件**：启动过一次之后，即使任务已经跑完、`CER.ENx` 被硬件清 0，FIFO 依然能收 CPU/DMA 写进来的字（两者都是上板实测）。所以 CPU 侧（含 DMA）要往 FIFO 写数据，先把这个核启动一次即可。
```

```{note}
`IER.EVTAUTOx` 的自动事件是在**任务真正完成**（`CCRx.REP` 递减到 0、核停止运行）时置位一次，**不是每跑一遍（每次命中 `END0`/`END1`）一次**：`REP = 2` 的程序会跑 3 遍，但 `EVTx` 只在最后结束时置位一次（手册 §18.1.4 的行为描述 + 上板实测 `REP=2` 只收到 1 次中断）。因此 `HAL_PTM_CoreWaitDone()` / `HAL_PTM_CoreWaitEvent()` 对"等任务结束"是精确的。
```

## 注意事项

- `CCRx.END0` / `END1` 是在**执行完该地址上的指令之后**才判定的：末条指令照常执行，然后任务结束（所以拿一条空闲指令当结束标记时，它的延时也生效）。置 0 表示该地址不做结束判定；结束地址如果填了超出程序范围的非法值，程序可能一直循环下去。执行地址为 0 时不做结束判定。
- `CER.RSTx` 会清除该核的通用寄存器、FIFO、PC 与错误标志，但不影响 TCM 内容，也不影响其它核。
- `ISR.ERRx`(总线错误/非法指令)与 `ISR.TCMx`(TCM 冲突)无法写清，只能通过 `HAL_PTM_CoreReset()` 复位对应核清除。
- 事件标志由 HAL 在中断回调之前按位清除（只清本核那一位），回调中不需要也不应该再清标志。
- 一个核的程序**用 `LDM`/`STM` 访问非法总线地址**或**执行非法指令**时会中止运行（硬件同时清掉 `CER.ENx`，核自己停下，`PC` 停在出错的那条指令上），可通过 `HAL_PTM_CoreGetStatus()` 读取状态与 `PC`。`JMP` **不是**错误条件：跳到不存在的地址不会置 `ERRx`，制造故障程序要用非法的 `LDM`/`STM` 或非法指令。若用了中断模式，HAL 在调用错误回调之前会先**屏蔽该核的错误中断**（`ERRx/TCMx` 是电平标志、只能靠 `HAL_PTM_CoreReset()` 清），回调里要恢复运行就得先 `HAL_PTM_CoreReset()` 再启动（启动会重新使能中断）。
- **线程安全**：模块级共享状态（时钟/复位引用计数、核占用位图、`IER` 的位读改写）在 HAL 内部用短临界区保护，因此可以从多个线程调用 `HAL_PTM_CoreInit/CoreDeInit/CoreAlloc` 等接口。仍由调用者负责：① 一个核句柄只归一个使用者，不要让两个线程共用一个 handle；② 临界区只关本 CPU 中断，若 HCPU 与 ACPU 都可能操作 PTM，需要在外部加锁。

## 相关例程

| 例程 | 演示内容 |
| --- | --- |
| `example/hal/ptm/ptm_macro` | **不用汇编器**：直接用 `ptm_ins.h` 的宏拼程序，轮询模式跑完并读回结果 |
| `example/hal/ptm/ptm_multi` | 多核：主核/配合核同周期启动、只让主核上报中断、出错后的恢复流程 |
| `example/hal/ptm/ptm_io` | CPU 侧 IO 输出控制、FIFO 接口（状态由调用者判断）与 DMA 的配合 |

## API参考

[](../api/hal/ptm.md)
