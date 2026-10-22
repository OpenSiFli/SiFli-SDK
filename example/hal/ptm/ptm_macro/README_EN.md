# PTM Instruction Macro Example
Source code path: example/hal/ptm/ptm_macro
## Supported Platforms
+ sf32lb57 series (PTM is only available on 57x, and this HAL is enabled for the HCPU side only)

## Overview
This example shows how to build a PTM (Parallel Task Machine) task program **without the assembler**
(`ptm_asm.h` is not used): the program is a plain `uint32_t` array whose instructions are assembled
from the macros of `ptm_ins.h`. That is the simplest way to use PTM when the code is short and its
jump offsets are easy to count by hand; as soon as a program has labels or forward references, the
assembler is the better choice: that assembler is `ptm_asm.h`, its usage is described in the
"Instructions and assembler" section of the guide, and the reference program is the
`example/hal/ptm/ptm_asm` example (the on-chip self test lives in `tests/ptm_selftest`). It covers:
+ A straight line program: two instructions built with `PTM_SET()`, with `CCR.END0` pointing at the
  last one so the task ends by itself
+ A loop: a hand written jump offset with `PTM_JMP()` (the offset is relative to the JMP itself) and
  `JMP_NEQDEC` for a decrementing loop
+ The HAL entry points it calls: `HAL_PTM_CoreInit()`, `HAL_PTM_CoreConfig()`,
  `HAL_PTM_CoreLoadCode()`, `HAL_PTM_CoreStart()` (polling mode), `HAL_PTM_CoreWaitDone()`,
  `HAL_PTM_CoreGetReg()` and `HAL_PTM_CoreDeInit()`
+ The instruction macros and operands it uses: `PTM_SET()`, `PTM_JMP()`, `PTM_WAIT()`,
  `PTM_X`/`PTM_Y`/`PTM_B`, `OP_ADD` and `JMP_NEQDEC`, with the result read back through
  `PTM_REG_X`/`PTM_REG_Y`

## Example Usage
### Hardware Requirements
A supported board is required, for example spi-hdk_lb57bun3n6.

### menuconfig Configuration
The example uses the cores and registers inside PTM only, so no extra configuration is needed.

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
The serial output looks like this (one result line per program, then `done`):
```
PTM example: programs built with the instruction macros
(no assembler, ptm_asm.h is not used)

[1] straight line program, polling mode
    completed: X = 0x1234, Y = 0x1235

[2] loop with a hand written jump offset
    completed: X = 18 after 5 jumps

done
```
This example is demo/reference code: the exact values of `X` and `Y` depend on the state of the chip
(for instance on whether a core is already taken or on what the TCM held before), and the numbers
above are only the values the code logic implies. As long as a `completed:` line appears, the task
ended normally.

## Exception Diagnosis

| Symptom | Cause and what to do |
| --- | --- |
| `CoreInit failed: is that core already taken?` is printed | The core selected by `core` (core 0 here) is taken by another handle. Set `ptm_core.core` to `PTM_CORE_ANY` to let the HAL pick a free core, or release a core with `HAL_PTM_CoreDeInit()` first. |
| `CoreLoadCode failed` is printed and it returned `HAL_ERROR` | The program would overwrite the program of another core in the TCM (a program longer than 16 instructions overruns into the next core window). Keep it short or move it to the system bus and jump with `PTM_JMP(JMP_X, ...)`. |
| `CoreLoadCode failed` is printed and it returned `HAL_BUSY` | The core is running: load the program before starting the core. |
| `the task did not complete` is printed | `HAL_PTM_CoreWaitDone()` timed out, so the task never ended: check that `CCR.END0`/`END1` point at an address that is really reached, or that `REP` reaches zero. |
| The build fails with `undefined reference` (`HAL_PTM_*` / `PTM1_BASE`) | The target chip is not an SF32LB57x: PTM exists on 57x only and this HAL is enabled for the HCPU side. Build this example for a board that has PTM. |

## Reference Documents
+ PTM HAL guide: [PTM](../../../../docs/source/en/hal/ptm.md)
+ UM5701 chapter 18 (PTM)

## Update Log

| Date | Change |
| --- | --- |
| 2026-05 | First version, showing a PTM task program built directly from the instruction macros, without the assembler. |
