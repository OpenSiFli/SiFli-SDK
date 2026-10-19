# PTM

PTM (Parallel Task Machine) is a small processor able to run several tasks in parallel. It embeds 4 independent cores, each of them fetching and executing instructions from its internal memory (TCM) or from the system bus: bus read/write, arithmetic, compare and branch, IO read/write, waiting for a trigger and delays. It can be used to implement custom interface protocols, custom PWM, bulk memory computation, inter-module scheduling or automatic sensor acquisition.

- 4 cores running in parallel, two stage pipeline, one basic instruction per core clock
- 32 bit custom instruction set: 4 basic instructions (SET/WAIT/JMP/IO) and 4 memory instructions (LDM/LDR/STM/STR)
- 64 instruction TCM slots; core N address 0 maps to TCM[16 x N], so a TCM program is limited to 16 instructions per core. Executing from the system bus is not limited
- 2 x 32 bit (X/Y) and 2 x 16 bit (A/B) general purpose registers per core
- 32 bit input/output FIFO (IFIFO/OFIFO) with DMA interface per core, depth 4 or 8
- 16 IO channels CH0~CH15; each core handles up to 16 IO in/out and can drive 2 of them individually
- 16 event flags EVT0~EVT15 for inter-core cooperation; EVT0~EVT3 can raise a CPU interrupt
- Each core can be reset, started, stopped and single stepped independently

```{note}
PTM is only available on SF32LB57x, and this HAL is enabled for the HCPU side only (`bf0_hal_conf_hcpu.h`): the per core interrupt numbers exist in the HCPU IRQn table only (there are none for the LCPU).
```

## Handle model

PTM is a single hardware instance that may be shared by several drivers, so the HAL uses two levels of handles:

| Handle | Declared by | Purpose |
| --- | --- | --- |
| `PTM_HandleTypeDef` | a HAL internal singleton, obtained with `HAL_PTM_GetHandle()` | clock/reset reference count, core allocation bitmap, global EVT/CH helpers |
| `PTM_CoreHandleTypeDef` | the user (static storage, or inside its own device object) | configuration, callbacks and state of one core |

A driver normally only uses `HAL_PTM_CoreInit()` / `HAL_PTM_CoreDeInit()`: they take care of the module reference count and of claiming/resetting the core. Initializing a second core therefore never resets a core that is already running.

```{warning}
A core handle must have static storage duration (or live as long as its device object): the HAL and the interrupt handler keep a pointer to it. A function local variable is not allowed.
```

## Using PTM

```c
static PTM_CoreHandleTypeDef ptm_core;              /* static storage */

/* Program: X is cleared and then counts up forever. The relative offset of the JMP is
   computed by hand as "target = address of the JMP itself + Tgt", so -1 goes
   from word 2 back to word 1. */
static const uint32_t ptm_code[] =
{
    PTM_SET(PTM_X, PTM_ZERO, OP_ADD, 0),            /* 0: X = 0 */
    PTM_SET(PTM_X, PTM_X, OP_ADD, 1),               /* 1: X++ (loop body) */
    PTM_JMP(-1, PTM_ZERO, JMP_EQ, PTM_ZERO, 0),     /* 2: 0 == 0 is always true: jump to 1 */
};

void ptm_demo_start(void)
{
    PTM_CoreCfgTypeDef cfg = {0};

    /* 1. Claim and reset a core: use a core index or PTM_CORE_ANY */
    ptm_core.core = 0;
    if (HAL_PTM_CoreInit(&ptm_core) != HAL_OK)
    {
        return;
    }

    /* 2. Load it into the TCM window of that core (core N address 0 is TCM[16*N]) */
    if (HAL_PTM_CoreLoadCode(&ptm_core, ptm_code,
                             sizeof(ptm_code) / sizeof(ptm_code[0])) != HAL_OK)
    {
        return;
    }

    /* 3. Configure CCRx: clock divider, repeat count, end addresses.  This
          program keeps itself looping with the JMP, so REP and END0/END1 stay 0
          (0 means "no end check at that index": the loop comes from the
          instruction, it does not rely on REP). */
    cfg.psc  = 255;                                 /* divide so X grows slowly enough to read */
    cfg.rep  = 0;
    cfg.end0 = 0;
    cfg.end1 = 0;
    HAL_PTM_CoreConfig(&ptm_core, &cfg);

    /* 4. Start (polling mode; use HAL_PTM_CoreStart_IT() to report through
          interrupts, see the next section) */
    HAL_PTM_CoreStart(&ptm_core);
}

/* Read it back at any time: HAL_PTM_CoreGetReg(&ptm_core, PTM_REG_X);
   stop and release:         HAL_PTM_CoreDeInit(&ptm_core);                      */
```

See `example/hal/ptm/ptm_io` for a complete example of the IO channels and the FIFOs.

## Interrupts and events

The interrupt routing is fixed in hardware: once `IER.EVTIE<n>` is enabled, setting `EVTn` raises an interrupt on `PTM1_COREn_IRQn`. The EVT used by a program must therefore match the core that runs it.

```c
void PTM1_CORE0_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_PTM_IRQHandler(0);                          /* clears the flag, then calls the callbacks */
    rt_interrupt_leave();
}

static void ptm_event_cb(PTM_CoreHandleTypeDef *hcore, uint16_t evt_mask)
{
    /* the EVT flag has already been cleared by the HAL */
}

void ptm_demo_irq_start(void)
{
    HAL_PTM_CoreRegisterCallback(&ptm_core, ptm_event_cb, NULL);
    HAL_PTM_CoreStart_IT(&ptm_core);      /* interrupt mode: callbacks on completion or error */
}
```

```{note}
`rt_interrupt_enter()` / `rt_interrupt_leave()` are required by the RT-Thread environment (a bare metal build does not need them). The HAL only provides the dispatcher `HAL_PTM_IRQHandler(core)`; the vector entry itself is defined by the user, so that one entry can serve several modules.
```

- **Two start flavours**: `HAL_PTM_CoreStart()` is the **polling** one (it only enables the completion event and leaves every interrupt of that core off), `HAL_PTM_CoreStart_IT()` is the **interrupt** one (completion event + event interrupt + error interrupt, plus the NVIC line of the core). All the register level differences are handled by the HAL, the caller never touches `IER`.
- **Simultaneous start of several cores** uses `HAL_PTM_CoreStartMask(hptm, core_mask, it_mask)`: every core of `core_mask` is enabled by a **single `CER` write** (same hclk cycle) and `it_mask` selects which of them work in interrupt mode, the others in polling mode. A master plus a helper core is therefore expressed directly — the helper needs no interrupt at all, so it does not add an ISR entry:
  ```c
  HAL_PTM_CoreStartMask(HAL_PTM_GetHandle(), (1U << 0) | (1U << 1), (1U << 0));
  /* core0 and core1 start in the same cycle; only core0 uses completion/event/error interrupts */
  ```
- In interrupt mode, setting `EVT<core>` via `SET EVTS` at any point in the program will trigger the callback, regardless of whether it occurs mid-execution or at completion. If you need to distinguish between these cases, check whether the core has stopped by calling HAL_PTM_CoreIsRunning() or HAL_PTM_CoreGetStatus() in the callback.  
- The program may also signal on its own: `PTM_SET(PTM_EVTS, PTM_ZERO, OP_OR, 1 << 0)` sets `EVT0`, `PTM_WAIT(WAIT_EVT0, 1, 0)` waits for it.
- From the CPU: `HAL_PTM_GetEvent()` reads all the flags, `HAL_PTM_ClearEvent()` clears them bitwise. The CPU can only clear events (`ISR.EVTx` is write 1 clear).
- `EVT4~EVT15` neither raise an interrupt nor get asserted automatically on completion on the 57x: only `EVTIE0~EVTIE3` and `EVTAUTO0~EVTAUTO3` are implemented (`IER` bits [7:4] and [15:12] are RSVD; the `EVTIE4~EVTIE7` / `EVTAUTO4~EVTAUTO7` fields of `ptm.h` are the generic IP definition and are not usable on the 57x). The flags themselves are fully usable: for **inter-core handshaking** (one core sets them with `SET EVTS`, another waits with `PTM_WAIT(WAIT_EVTn, 1, 0)`) and for CPU polling (`HAL_PTM_GetEvent()` reads, `HAL_PTM_ClearEvent()` clears). To wait for them from the CPU, poll `HAL_PTM_GetEvent()` yourself.
- An event interrupt must be enabled explicitly: setting `EVTn` alone raises nothing, start with `HAL_PTM_CoreStart_IT()` (or list the core in the `it_mask` of `HAL_PTM_CoreStartMask()`), which enables `IER.EVTIE<n>` for that core. Instruction level waiting (`PTM_WAIT(WAIT_EVTn, ...)`) needs no enable at all, it reads the flag directly.
- On 57x the error interrupt of a core shares its NVIC line with the event interrupt and `HAL_PTM_CoreStart_IT()` turns it on as well; in polling mode errors are found through the `error` / `tcm_conflict` fields of `HAL_PTM_CoreGetStatus()`.

### Polling mode

After starting with `HAL_PTM_CoreStart()` (the polling flavour) and without interrupts, the blocking polling helpers can be used (they implement the timeout with `HAL_GetTick()` + `HAL_Delay(1)`, so they are for thread context only):

```c
/* wait for the end of the run: the criteria are the same as in interrupt mode */
switch (HAL_PTM_CoreWaitDone(&ptm_core, 1000))
{
case HAL_OK:      break;      /* completed (the hardware set EVT<core>) */
case HAL_ERROR:   break;      /* aborted (ISR.ERRx / TCMx), details in CoreGetStatus() */
case HAL_TIMEOUT: break;      /* not finished: still running, or stopped with CoreStop() */
default:          break;
}

/* wait for a signal raised in the middle of the program (consumed on success) */
HAL_PTM_CoreWaitEvent(&ptm_core, 100);
```

`HAL_PTM_CoreWaitDone()` polls `ISR.EVT<core>` and `ISR.ERRx/TCMx`, **not** `CER.ENx`: `EN = 0` is ambiguous (normal end, stopped with `CoreStop()`, or never started), while those flags are how the hardware reports the outcome of the run. It does **not** consume the event, so `HAL_PTM_CoreWaitEvent()`/`HAL_PTM_GetEvent()` can still observe the same completion; and `HAL_PTM_CoreStart*()` clears the event left over by an earlier run first, so a stale completion cannot make `WaitDone()` return immediately.

Waiting for a FIFO state is done with `HAL_PTM_CoreGetFifoStatus()` in a loop of your own. The preferred patterns are to move the data with DMA, or to have the program raise an event with `SET EVTS` once the data is ready and wait for it with `HAL_PTM_CoreWaitEvent()` — that covers "the data is ready" directly instead of guessing FIFO full/empty flags.

In interrupt context, or in a loop of your own, use the non blocking readers: `HAL_PTM_GetEvent()`, `HAL_PTM_CoreGetFifoStatus()`, `HAL_PTM_CoreGetStatus()`, `HAL_PTM_CoreIsRunning()` and `HAL_PTM_CoreGetReg(&ptm_core, PTM_REG_Y)`.

Both modes can be mixed: interrupts for notification, polling for reading the data. Polling is not restricted by the core n <-> EVTn routing, the CPU may read any EVT flag.

## Multiple cores

Each core has its own handle and can be configured and started independently:

```c
static PTM_CoreHandleTypeDef ptm_core0;
static PTM_CoreHandleTypeDef ptm_core1;

ptm_core0.core = 0;
ptm_core1.core = PTM_CORE_ANY;                      /* pick any free core */
HAL_PTM_CoreInit(&ptm_core0);
HAL_PTM_CoreInit(&ptm_core1);                       /* does not reset core 0 */
```

**Starting several cores at the same time**: one register write enables all of them in the same hclk cycle; for a tighter alignment let both programs wait for the same trigger first:

```c
/* first instruction of both programs: PTM_WAIT(PTM_TRIG_GPTIM1_UPDATE, 1, 0) */
HAL_PTM_CoreStartMask(HAL_PTM_GetHandle(), (1U << 0) | (1U << 1), (1U << 0));
```

Constraints to keep in mind:

- **TCM**: core N address 0 is TCM[16 x N], so a program longer than 16 instructions overruns into the address 0 of the next core, which then cannot be used. The HAL records which core owns which TCM slots and refuses **both** overlapping orders: (1) loading over the window of an already claimed core, and (2) loading a long program first and later loading code for a core it overran. In both cases `HAL_PTM_CoreLoadCode()` returns `HAL_ERROR` and writes nothing. At runtime the hardware collision flag is also readable with `HAL_PTM_GetTcmConflict()`. Longer programs must be placed on the system bus and reached with `PTM_JMP(JMP_X)`.
- **IO channels**: CH0~CH15 are shared by all the cores. Use `ICRx.IBASE` / `OCRx.OBASE` (`io.ibase` / `io.obase`) to map them per core, so that several cores can run the same instruction sequence on different IO.
- **Events**: EVT0~EVT15 are global flags, but the interrupt routing is fixed to core n <-> EVTn.

## Instructions

The instruction words come from the `ptm_ins.h` macros; the HAL only loads them and configures the registers, it does not implement the encoding again:

```c
static const uint32_t blink_code[] =
{
    PTM_SET(PTM_Y, PTM_ZERO, OP_ADD, 0),
    PTM_IO(IO_PUSH8, IO_OUT0_H, IO_NOP, 0),
    PTM_WAIT(0, 0, 100),                            /* wait source 0 always matches: a pure delay */
    PTM_JMP(-3, PTM_ZERO, JMP_EQ, PTM_ZERO, 0),     /* compute the relative offset yourself */
};
HAL_PTM_CoreLoadCode(&ptm_core, blink_code, 4);
```

```{note}
A PTM relative jump is based on the address of the JMP instruction itself, i.e. `target = PC + Tgt` (not the next instruction); hand written `PTM_JMP()` offsets must follow that rule. The offset range is -127~127. The other two encodings of `Tgt` are `128~255` for an absolute TCM index and `256` / `257` (`JMP_X` / `JMP_Y`) to jump through `X` / `Y`, which is what a program placed on the system bus uses.
```

## Using the FIFOs and DMA

Receive path: IO data sampled by the `IO` instruction lands in `Y`, is pushed into IFIFO automatically when `ICRx.IAUTO` is set, and can be read by the CPU with `HAL_PTM_CorePopIFifo()` or moved by DMA:

```c
hdma.Init.Request = PTM1_IFIFO1_DMA_REQUEST;        /* see the board dma_config.h */
HAL_DMA_Start_IT(&hdma, HAL_PTM_CoreGetFifoAddr(&ptm_core1, PTM_FIFO_IFIFO),
                 (uint32_t)buffer, words);
```

Transmit path: the CPU/DMA writes OFIFO (`HAL_PTM_CorePushOFifo()`), the data is popped into `X` automatically when `OCRx.OAUTO` is set, and pushed to the IO channels by the `IO` instruction.

```{note}
`HAL_PTM_CorePushOFifo()` / `HAL_PTM_CorePopIFifo()` write and read the FIFO directly and **do not look at its status** (the same convention as a UART FIFO): check with `HAL_PTM_CoreGetFifoStatus()` that `PTM_FIFO_FLAG_OFIFO_FULL` is clear before pushing and that `PTM_FIFO_FLAG_IFIFO_EMPTY` is clear before popping. Writing a full FIFO or reading an empty one has no behaviour defined by the manual and must be avoided.

A FIFO belongs to the task engine of a core, so **it only accepts data once that core has been started (out of reset)**: a core that was `HAL_PTM_CoreInit()`ed but never started is still in the `CER.RSTx` reset state, and the writes into its FIFO are dropped and leave the status bits unchanged. The opposite is not required: **the core does not have to be running** — after it has been started once, the FIFO keeps accepting words written by the CPU/DMA even when the task has already completed and the hardware has cleared `CER.ENx` (both were measured on the board). The CPU side (DMA included) therefore only has to start that core once before it feeds the FIFO.
```

```{note}
The automatic event of `IER.EVTAUTOx` is asserted **once, when the task really completes** (`CCRx.REP` counted down to 0 and the core stops) — **not once per pass** (per `END0`/`END1` hit): a program with `REP = 2` runs three passes but `EVTx` is asserted once, at the end (behaviour described in the manual §18.1.4, measured on the board with `REP = 2`: a single interrupt). `HAL_PTM_CoreWaitDone()` / `HAL_PTM_CoreWaitEvent()` are therefore exact for "wait until the task is over".
```

## Notes

- `CCRx.END0` / `END1` are tested **after the instruction at that address has run**: the last instruction executes as usual and the task ends right after it (so an idle instruction used as an end marker still spends its delay). A value of 0 means "no end check at that index". If neither is set the core may loop forever and overrun the instruction space of the other cores. Address 0 is never treated as an end address.
- `CER.RSTx` clears the general purpose registers, FIFO, PC and error flags of that core. It does not touch the TCM content nor the other cores.
- `ISR.ERRx` (bus error or illegal instruction) and `ISR.TCMx` (TCM collision) cannot be cleared by a write; reset the core with `HAL_PTM_CoreReset()` instead.
- Event flags are cleared bitwise by the HAL before the callback runs (only the bit of that core); the callback must not clear them again.
- A core stops when it **reads or writes an illegal bus address with `LDM`/`STM`**, or when it **executes an illegal instruction** (the hardware also clears `CER.ENx`, so the core stops by itself with the `PC` on the instruction that faulted). `HAL_PTM_CoreGetStatus()` reports the state and the `PC`. A `JMP` is **not** an error condition: jumping to an address that does not exist raises no `ERRx`, so a faulty program has to use an illegal `LDM`/`STM` or an illegal instruction. In interrupt mode the HAL **masks the error interrupt of that core** before calling the error callback (the `ERRx/TCMx` flags are level and are cleared by `HAL_PTM_CoreReset()` only), so a callback that wants to continue has to reset the core and start it again, which re-arms the interrupt.
- **Thread safety**: the module state shared by all the users (clock/reset reference count, core allocation bitmap and the read-modify-write of the `IER` bits) is updated inside short critical sections, so `HAL_PTM_CoreInit/CoreDeInit/CoreAlloc` and friends may be called from several threads. Two precautions stay on the caller side: (1) a core handle belongs to a single user, do not drive one handle from two threads; (2) the critical sections only mask this CPU, so if HCPU and ACPU can both drive PTM an external lock is required.

## Related examples

| Example | What it shows |
| --- | --- |
| `example/hal/ptm/ptm_macro` | **No assembler**: programs built from the `ptm_ins.h` macros, run in polling mode and read back |
| `example/hal/ptm/ptm_multi` | Multiple cores: master and helper started in the same cycle, only the master reports to the CPU, and how to recover from an aborted task |
| `example/hal/ptm/ptm_io` | CPU side IO output control, the FIFO interface (the caller checks the status) and how it fits with the DMA |

## API Reference

[](../api/hal/ptm.md)
