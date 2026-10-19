# PTM IO and FIFO Example
Source code path: example/hal/ptm/ptm_io
## Supported Platforms
+ sf32lb57 series (PTM is only available on 57x, and this HAL is enabled for the HCPU side only)

## Overview
This example shows how to use the IO channels and the FIFOs of the PTM (Parallel Task Machine) from
the **CPU side**, starting a core only where it is needed:
+ Driving an output channel like a GPIO: `HAL_PTM_SetOutputEnable()` enables the output of a channel,
  `HAL_PTM_SetOutput()` writes its value, and `HAL_PTM_GetOutputEnable()`/`HAL_PTM_GetOutput()` read
  them back
+ Describing the FIFO and IO group wiring of a core with `HAL_PTM_CoreConfigIo()`
  (`PTM_IoFifoInitTypeDef`): enable the output FIFO, keep `oauto = 0` so nothing pops the FIFO behind
  the program's back, and start the IO group at channel 0 (`obase`)
+ Starting a core that never touches the FIFO, with `HAL_PTM_CoreConfig()` /
  `HAL_PTM_CoreLoadCode()` / `HAL_PTM_CoreStart()`: a FIFO belongs to the task engine of a core, so
  **it does not accept data before that core is enabled** (a core that was initialized but not started
  is held by the reset written in `HAL_PTM_CoreReset()` and drops the writes). The core has to be
  running before the CPU writes into its FIFO
+ The FIFOs are raw hardware queues: the HAL only reads and writes them and the caller asks the status
  first with `HAL_PTM_CoreGetFifoStatus()` before calling `HAL_PTM_CorePushOFifo()` /
  `HAL_PTM_CorePopIFifo()`, the same convention as a UART FIFO; the overflow/underflow flags are
  cleared with `HAL_PTM_CoreClearFifoFlag()` (`PTM_FIFO_FLAG_OFIFO_UNF`)
+ The input FIFO is filled by the task: `ififo_en` enables the FIFO, `iauto` lets the core push the
  sampled value by itself, `irsh` shifts the sampled bits (see `drv_dcmi.c`) and `ilimit` is the
  accumulated bit threshold of that automatic push (0 means 32, not "every sample"); the CPU only
  checks `PTM_FIFO_FLAG_IFIFO_EMPTY` before calling `HAL_PTM_CorePopIFifo()`
+ Real bulk transfers hand the FIFO address to the DMA: `HAL_PTM_CoreGetFifoAddr()` returns the
  address of `PTM_FIFO_OFIFO`
+ Pin muxing: a channel only reaches a pad after `HAL_PIN_Set(PAD_PAxx, PTM1_CHn, PIN_NOPULL, 1)`
+ The other entry points it calls: `HAL_PTM_GetHandle()`, `HAL_PTM_Init()`/`HAL_PTM_DeInit()` (the
  module clock), `HAL_PTM_CoreInit()`, `HAL_PTM_CoreReset()` and `HAL_PTM_CoreDeInit()`; the FIFO
  status bits are `PTM_FIFO_FLAG_OFIFO_FULL`, `PTM_FIFO_FLAG_OFIFO_EMPTY`, `PTM_FIFO_FLAG_OFIFO_UNF`
  and `PTM_FIFO_FLAG_IFIFO_EMPTY`

## Example Usage
### Hardware Requirements
A supported board is required, for example spi-hdk_lb57bun3n6.
To watch the IO output on a scope, the related pins have to be muxed to the PTM function: step 0 of
the example calls `HAL_PIN_Set()`, with the pad and the channel selected by the `PTM_IO_PIN_SETUP`,
`PTM_IO_PAD` and `PTM_IO_CHANNEL` macros. The default is `PAD_PA35` → `PTM1_CH0`; adapt it to a **free**
pad of your board that carries that channel. If the pad is already driven by something else (on
spi-hdk, for example, `PAD_PA04` is the LCD SPI clock), set `PTM_IO_PIN_SETUP` to 0 or do the mux in
`bsp_pinmux.c` instead.

### menuconfig Configuration
The example uses the IO channels and FIFOs inside PTM only, so no extra configuration is needed.

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
The serial output looks like this (the pin mux and the CPU driven output channel first, then the two
FIFOs):
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
This example is demo/reference code: `enable`/`value` are channel bitmaps (`0x1` means CH0), the
OFIFO address follows the core offset (the OFIFO of core 0 is `PTM1_BASE + 0x30 + 0x24`), and the
value read in step 3 depends on the level sampled on the pins, so it is not necessarily `0x0`
(grounded or floating pins usually read `0x0`). The lines that matter are `OFIFO is not empty`
right after `pushed one word into OFIFO`, and a `read ... from IFIFO` line in step 3.

```{note}
Step 3 relies on the automatic push of the task to move the sampled value into the IFIFO, and
`ICRx.ILIMIT` is the number of accumulated input bits that triggers it — **0 is a special value
meaning 32** (see the ICRx description of UM5701 chapter 18). A program that samples 8 channels
once (8 bits) never reaches 32, so nothing is pushed and it prints
`IFIFO is empty: the task sampled nothing`. The example therefore sets `io.ilimit = 8` explicitly,
which makes "one 8 bit sample is pushed" deterministic; with the continuous sampling usage of the
reference driver `drv_dcmi.c` (`ilimit = 0`, i.e. 32 bits per packet) four 8 bit samples have to
accumulate before the CPU can read a word.
```

## Exception Diagnosis

| Symptom | Cause and what to do |
| --- | --- |
| `CoreInit failed: is that core already taken?` is printed | The core selected by `core` (core 0 here) is taken by another handle. Set `ptm_core.core` to `PTM_CORE_ANY` to let the HAL pick a free core, or release a core with `HAL_PTM_CoreDeInit()` first. |
| Nothing comes out of the IO channels | The pin has to be muxed to the PTM function by the board, and the CPU side output enable must be set with `HAL_PTM_SetOutputEnable()` before the value is written (otherwise the channel stays in its reset state). |
| `pushed one word into OFIFO` is not printed | The output FIFO is already full (`PTM_FIFO_FLAG_OFIFO_FULL` set), so `HAL_PTM_CorePushOFifo()` was skipped: wait until the FIFO has room, or move the data with the DMA. |
| The push happened but `OFIFO is empty` is still printed | A FIFO belongs to the task engine of a core: while the core is not enabled (it was `HAL_PTM_CoreInit()`ed but never `HAL_PTM_CoreStart()`ed, so it is held by the reset written in `HAL_PTM_CoreReset()`) the FIFO accepts nothing. Also check that `ofifo_en` is set in `HAL_PTM_CoreConfigIo()`. |
| Nothing can be read from the IFIFO | Reading an empty FIFO is not a defined operation, so a driver checks `HAL_PTM_CoreGetFifoStatus()` first (`PTM_FIFO_FLAG_IFIFO_EMPTY` clear). The data has to be pushed in by a task: `ififo_en` enables the FIFO, `iauto` lets the core push the sample by itself and `irsh`/`ishift`/`ilimit` describe how the sampled bits are packed (for 8 sampled channels see `irsh = 1` in `drv_dcmi.c`); the DMA can also feed it. Mind that `ilimit` is the automatic push threshold and **0 means 32 bits**, so a program sampling fewer than 32 bits per run pushes nothing until four 8 bit samples have accumulated — the example sets `ilimit = 8` to make one sample enough. |
| The OFIFO overflow/underflow flag cannot be cleared | The flag has to be cleared with `HAL_PTM_CoreClearFifoFlag()` (`PTM_FIFO_FLAG_OFIFO_UNF`), which rebuilds ICR/OCR from the configuration and loses no configuration bit; writing the register directly may clear configuration bits as well. |
| The build fails with `undefined reference` (`HAL_PTM_*` / `PTM1_BASE`) | The target chip is not an SF32LB57x: PTM exists on 57x only and this HAL is enabled for the HCPU side. Build this example for a board that has PTM. |

## Reference Documents
+ PTM HAL guide: [PTM](../../../../docs/source/en/hal/ptm.md)
+ UM5701 chapter 18 (PTM); the FIFO bit fields are described in the ICRx/OCR registers

## Update Log

| Date | Change |
| --- | --- |
| 2026-05 | First version, showing the CPU driven IO channels and the configuration, read/write and status of the OFIFO/IFIFO. |
