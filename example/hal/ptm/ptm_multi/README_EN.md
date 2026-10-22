# PTM Multi-core and Interrupt Example
Source code path: example/hal/ptm/ptm_multi
## Supported Platforms
+ sf32lb57 series (PTM is only available on 57x, and this HAL is enabled for the HCPU side only)

## Overview
This example shows the multi-core and interrupt side of the PTM (Parallel Task Machine) HAL:
+ A master core and a helper core started in the same hclk cycle with `HAL_PTM_CoreStartMask()`:
  `core_mask` selects the cores to start and `it_mask` selects the ones that report to the CPU,
  so the helper that nobody waits for stays silent
+ The completion of the master is delivered to a callback registered with
  `HAL_PTM_CoreRegisterCallback()` instead of being polled; the callback is dispatched from the
  interrupt by `HAL_PTM_IRQHandler(core)`, and the vector entries `PTM1_CORE0_IRQHandler` /
  `PTM1_CORE1_IRQHandler` are provided by the application, paired with
  `rt_interrupt_enter()`/`rt_interrupt_leave()` in an RT-Thread build
+ The master waits for the helper with `PTM_WAIT(WAIT_EVT1, ...)` while the helper sets EVT1 through
  `PTM_EVTS` and runs forever with `CCR.REP = 255`; the CPU finally stops it with
  `HAL_PTM_CoreStop()` and reads `PTM_REG_Y` back
+ The recovery flow after an error: a program reads a dead address with
  `PTM_LDM(PTM_X, 0xDEADBEEF)` (a bus error), the core aborts, the error callback is called and
  `HAL_PTM_CoreGetStatus()` returns the `PC` of the abort. The error flags are sticky, so the core
  has to be reset with `HAL_PTM_CoreReset()` before `HAL_PTM_CoreStart_IT()` can start it again
  + PTM raises `ERRn` for a **bus error** (`LDM`/`STM` through an invalid address) and for an
    **illegal instruction** only; a `JMP` to an address that does not exist is not a fault, so a
    faulty program cannot be built out of a jump
  + On an error the hardware clears `CER.ENn` (the core stops by itself) and the PC stays on the
    instruction that faulted
+ The other entry points it calls: `HAL_PTM_GetHandle()`, `HAL_PTM_CoreInit()`,
  `HAL_PTM_CoreConfig()`, `HAL_PTM_CoreLoadCode()` and `HAL_PTM_CoreDeInit()`

## Example Usage
### Hardware Requirements
A supported board is required, for example spi-hdk_lb57bun3n6.

### menuconfig Configuration
The example uses the cores and interrupts inside PTM only, so no extra configuration is needed.

### Compilation and Programming
Enter the project directory of the example and run scons:
```
scons --board=spi-hdk_lb57bun3n6_hcpu -j32
```
Then run `build_spi-hdk_lb57bun3n6_hcpu\uart_download.bat` and select the port:
```
$ ./uart_download.bat

     Uart Download

please input the serial port num:5
```
See [](/quickstart/get-started.md) for the detailed build and download steps.

## Expected Results
The serial output looks like this (multi-core cooperation first, then error recovery):
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
This example is demo/reference code: the number of passes of the helper (`Y`) follows when
`HAL_PTM_CoreStop()` is called. The helper is stopped only after the master reported completion,
and that interval is decided by CPU polling/delays, so the measured value varies between a few
thousand and a few tens of thousands (two board runs gave `4213` and `48689`); it does not have to
match the number above. The three lines that
matter are `the master completed...`, `error callback called...` and `recovered: ...`; `PC` is the
address of the faulting instruction inside the core (the faulting instruction here is the first
one, hence `0x0`).

## Exception Diagnosis

| Symptom | Cause and what to do |
| --- | --- |
| `CoreInit(master) failed` or `CoreInit(helper) failed` is printed | The requested core (core 0 / core 1) is taken by another handle. Use `PTM_CORE_ANY`, or release a core with `HAL_PTM_CoreDeInit()` first. |
| `CoreLoadCode failed` is printed and it returned `HAL_ERROR` | The program would overwrite the program of another core in the TCM (a program longer than 16 instructions overruns into the next core window). Keep it short or move it to the system bus and jump with `PTM_JMP(JMP_X, ...)`. |
| `CoreLoadCode failed` is printed and it returned `HAL_BUSY` | The core is running: load the program before starting the core. |
| `CoreStartMask failed` is printed | One of the cores in the start mask is not available (already claimed, or its handle was not initialized). Make sure every core was initialized by `HAL_PTM_CoreInit()` and that `core_mask`/`it_mask` match the handles. |
| `no completion event within 1 s` or `no error reported within 1 s` is printed | No callback arrived within 1 s: completion is reported through `EVTn`, so start the cores with `HAL_PTM_CoreStart()`/`HAL_PTM_CoreStart_IT()` (they enable it) and check that the program really ends (`CCR.END0`/`END1` or `REP`). |
| The faulty program never faults (`no error reported within 1 s`) | `ERRn` is raised by a bus error (`LDM`/`STM` through an invalid address) and by an illegal instruction only: an out of range `JMP`, or a jump to an address that does not exist, reports nothing. Build the faulty program like this example, with `PTM_LDM(PTM_X, 0xDEADBEEF)`; the register level case is `example/psram_test/src/csim/ctest/ptm/ptm1_err.c`. |
| `END0` points outside the new program | Loading a program does not clear the rest of the TCM, so `END0` has to be inside the new program, otherwise the core keeps running into the instructions the previous program left behind. |
| The error callback is entered over and over | The error flags are level sensitive, and this HAL masks the error interrupt of the core before calling the callback. To continue, clear the flags with `HAL_PTM_CoreReset()` and start the core again. |
| The build fails with `undefined reference` (`HAL_PTM_*` / `PTM1_BASE`) | The target chip is not an SF32LB57x: PTM exists on 57x only and this HAL is enabled for the HCPU side. Build this example for a board that has PTM. |

## Reference Documents
+ PTM HAL guide: [PTM](../../../../docs/source/en/hal/ptm.md)
+ UM5701 chapter 18 (PTM)

## Update Log

| Date | Change |
| --- | --- |
| 2026-05 | First version, showing several cores started together, interrupt callback reporting and the recovery after an aborted task. |
