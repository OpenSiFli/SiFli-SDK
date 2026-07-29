# SiFli Crash Dump Architecture Notes

## Transport Modes

| Transport | Mechanism | Depends on J-Link? | Notes |
| --- | --- | --- | --- |
| `jlink` | `JLinkExe` runs generated `.jlink` script and writes `capture.log` | Yes | Can parse current core registers from J-Link log. |
| `uart` | Python `SifliUartDap` speaks SiFli UART DEBUG IP over serial | No | Must work on macOS without AssertDump, JLinkExe, or uart_server. |
| `uart_server`/AssertDump bridge | J-Link Commander connects to `127.0.0.1:19025`, bridge talks UART DEBUG IP | Yes for tooling | Hardware path resembles UART-DAP, but software dependency is not acceptable for pure UART. |

## UART DEBUG IP

The Python UART transport uses `tools/sdk_py_actions/sifli_uart_dap.py` and keeps one serial connection open. Avoid shelling out to `probe-rs read/write` because separate Enter/op/Exit sessions lose core-switch state.

SF32LB52 frame format:

```text
7E 79 | payload_len:u16 LE | 10 00 | payload
```

Commands:

- Enter: `41 54 53 46 33 32 05 21`, response `0xD1`
- Exit: `41 54 53 46 33 32 18 21`, response `0xD0`
- MEMRead: `40 72 addr:u32 words:u16`, response `0xD2 + data + 0x06`
- MEMWrite: `40 77 addr:u32 count:u16 data`, response `0xD3`

Core-switch setup before UART reads:

```python
dap.write32(0x5000B008, 1)
dap.write32(0x5000B008, 0)
dap.halt()
```

Then read current core registers through Cortex-M CoreDebug:

- DHCSR `0xE000EDF0`
- DCRSR `0xE000EDF4`
- DCRDR `0xE000EDF8`
- `S_REGRDY = bit16`
- `S_HALT = bit17`

## SF32LB52 Address Mapping

UART DEBUG IP physical mapping:

- `0x00000000-0x0000FFFF` -> `0xA0000000 + addr`
- `0xE0000000-0xEFFFFFFF` -> `(addr & 0x0FFFFFFF) | 0xF0000000`
- `0x10000000-0x1FFFFFFF` -> `(addr & 0x0FFFFFFF) | 0x60000000`
- Others pass through.

## Register Semantics

Always distinguish:

- `current_registers`: host-side current CPU registers from UART-DAP or J-Link log.
- `saved_registers`: firmware-saved frame decoded from `saved_stack_frame`.
- `saved_scb_reg`, `error_reason`, `saved_stack_pointer`: firmware-owned evidence used to judge whether `saved_registers` are valid.

Trust current registers for the current halt point. Trust saved registers for the original crash only when the saved frame is coherent.

Heuristics for bad saved frame:

- `saved_stack_pointer` is `0`, tiny, or outside RAM.
- `pc` decodes to RAM/data such as `0x2000dc08` rather than code.
- `lr` is `0`.
- `error_reason` remains `0` after an assert path.
- `rt_log_buf` reports a second assert such as `Function[rt_sem_take] shall not be used while interrupt is disabled`.

## Known Type-2 Heap Overflow Pattern

In the assert-dump test project, scenario 2 corrupts heap metadata and then asserts during `rt_free()`.

Observed sequence:

1. First assert enters `rt_assert_handler()`.
2. `assert_happened` becomes `1`.
3. `rt_assert_handler()` calls `rt_show_sys_info()` before `rt_hw_fatal_error()`.
4. `rt_show_sys_info()` or a list/FS helper may take a semaphore while interrupts are disabled.
5. That triggers a second assert.
6. Because `assert_happened == 1`, the second assert loops forever before `rt_hw_fatal_error()` saves context.

Result:

- Current registers point at `rt_assert_handler` loop, commonly near `PC=0x12041a8e`.
- `saved_stack_frame` may be zero or polluted.
- J-Link/uart current register reads are useful but represent the second assert halt point, not the original heap overflow site.

Host-side mitigation: save current registers at export time. Firmware-side fix: save fatal context before `rt_show_sys_info()` or skip lock-taking diagnostics after assert.

## Main Files

- `tools/sdk_py_actions/crash_dump.py`: package layout, `capture_live`, `write_core_elf`, `analyze_package`
- `tools/sdk_py_actions/sifli_uart_dap.py`: direct UART DEBUG IP implementation
- `tools/sdk_py_actions/crash_dump_ext.py`: CLI registration
- `tests/sdk_env/test_crash_dump.py`: focused coverage for parser, capture, analysis

## Commands

UART capture:

```bash
sdk.py crash-dump capture-live \
  --transport uart \
  --chip SF32LB52 \
  --chip-model LB525 \
  --probe '1a86:55d3-1:/dev/cu.wchusbserialXXXX' \
  --core hcpu \
  --output /tmp/heap-overflow \
  --elf build_sf32lb52-lcd_n16r8_hcpu/main.elf
```

Analysis:

```bash
sdk.py crash-dump analyze \
  --package /tmp/heap-overflow \
  --engine python \
  --elf build_sf32lb52-lcd_n16r8_hcpu/main.elf
```

Validation:

```bash
python3 -m unittest tests.sdk_env.test_crash_dump.AnalysisTests tests.sdk_env.test_crash_dump.BackendTests
python3 -m py_compile tools/sdk_py_actions/crash_dump.py tools/sdk_py_actions/sifli_uart_dap.py
```
