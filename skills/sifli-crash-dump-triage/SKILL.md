---
name: sifli-crash-dump-triage
description: Triage and modify SiFli SDK crash-dump capture/analyze workflows. Use when working on sdk.py crash-dump, capture-live, readcore, analyze, UART-DAP, J-Link/AssertDump exports, saved_stack_frame, current_registers vs saved_registers, zero-register analysis, heap-overflow/assert crash cases, or files under tools/sdk_py_actions/crash_dump.py and sifli_uart_dap.py in SiFli-SDK.
---

# SiFli Crash Dump Triage

## First Move

Start from the artifact, not the theory.

1. Inspect `sdk_manifest.json` first, then `manifest.json`.
2. Record `source`, `register_source`, `current_register_source`, `warnings`, and whether `current_registers` and `saved_registers` exist.
3. Use the package's own `hcpu.axf` or attached ELF for symbol addresses; do not reuse addresses from another build.
4. Check raw bytes for `saved_stack_frame`, `saved_stack_pointer`, `error_reason`, and `saved_scb_reg`.
5. Only then decide whether the problem is capture, firmware save order, register-source selection, or analyzer fallback.

Read `references/architecture.md` when you need transport details, register-source rules, or the known heap-overflow/type-2 failure pattern.

## Register Source Rules

Keep these sources separate:

- `current_registers`: CPU registers read at export time from UART-DAP or J-Link log. Trust this for where the chip is stopped now.
- `saved_registers`: registers decoded from firmware `saved_stack_frame`. Trust this for where the original exception/assert happened only if the saved frame and saved SP look sane.
- `registers`: compatibility field. It may point to current registers; do not assume it is the original crash frame.

For type-2 heap overflow/assert cases, expect current registers to point at a later assert loop such as `rt_assert_handler`, while `saved_stack_frame` may be zero, partial, or polluted.

## Triage Commands

Use small focused checks:

```bash
python3 - <<'PY'
import json
from pathlib import Path
p = Path("/tmp/heap-overflow/sdk_manifest.json")
if not p.exists():
    p = Path("/tmp/heap-overflow/manifest.json")
m = json.loads(p.read_text())
for k in ("source", "register_source", "current_register_source", "warnings"):
    print(k, "=", m.get(k))
print("current_registers =", json.dumps(m.get("current_registers") or m.get("registers"), indent=2))
PY
```

Find symbol addresses from the package ELF:

```bash
nm -S -n /tmp/heap-overflow/hcpu.axf | rg 'saved_scb_reg|error_reason|saved_stack_pointer|saved_stack_frame|assert_happened|rt_log_buf'
```

Read raw saved-context bytes:

```bash
python3 - <<'PY'
from pathlib import Path
base = 0x20000000
pkg = Path("/tmp/heap-overflow")
symbols = {
    "saved_scb_reg": (0x2000dc1c, 20),
    "error_reason": (0x2000dc30, 4),
    "saved_stack_pointer": (0x2000dc34, 4),
    "saved_stack_frame": (0x2000dc38, 0x48),
}
data = (pkg / "hcpu_ram.bin").read_bytes()
for name, (addr, size) in symbols.items():
    chunk = data[addr - base: addr - base + size]
    print(name, hex(addr), "all_zero=", all(b == 0 for b in chunk), chunk.hex())
PY
```

Replace addresses with the package's own `nm` output.

## Fix Strategy

Prefer the smallest fix at the shared source:

- If J-Link or UART-DAP can read current core registers, save them during `capture-live` as `current_registers`.
- Do not make UART depend on `JLinkExe`, AssertDump, or `uart_server`; macOS and pure USB-serial setups must still work.
- If `saved_stack_frame` is invalid but current registers are valid, preserve both and report the conflict.
- If firmware hits a second assert before `rt_hw_fatal_error()`, the host tool cannot recover the original assert frame. Fix firmware save order: save fatal context before `rt_show_sys_info()` or other lock-taking diagnostics.
- Keep old fields (`registers`, `register_source`) for compatibility, but add explicit fields rather than changing meanings silently.

## Verification

For code changes, run the focused tests first:

```bash
python3 -m unittest tests.sdk_env.test_crash_dump.AnalysisTests tests.sdk_env.test_crash_dump.BackendTests
python3 -m py_compile tools/sdk_py_actions/crash_dump.py tools/sdk_py_actions/sifli_uart_dap.py
```

If full `tests.sdk_env.test_crash_dump` fails only on a CLI default engine assertion, do not mix that unrelated failure into crash-register work unless the user asks.
