---
name: sifli-build-win
description: Build SiFli SDK projects on Windows using PowerShell. Use this skill when the user wants to compile/build SiFli SDK firmware, specify a board+example combo, or asks "编译", "build", "编译xxx例程", "用板子xxx编译". Handles environment setup via export.ps1, SCons build with --board option, and locating build artifacts.
---

# SiFli SDK Windows Build

Builds SiFli SDK example/project firmware on **Windows** using **PowerShell** (`powershell.exe`).

> **⚠️ Platform restriction**: This skill is for **Windows only**. Before proceeding, check the platform:
> - Run `uname -s` to determine the OS
> - If the result contains `MINGW`/`MSYS`/`CYGWIN` (Git Bash on Windows), proceed with the build.
> - If the result is `Linux` or `Darwin` (macOS), **do not proceed**. Instead, tell the user: "This build skill requires Windows (PowerShell). On Linux/macOS, please use `source ./export.sh` in bash, then `scons --board=<board> -j8`. See the CLAUDE.md for Linux/macOS build instructions."

## Environment

- Build tool: **SCons** (invoked via the SDK's environment setup)
- Environment setup: `export.ps1` (PowerShell) in SDK root directory
- Shell: `powershell.exe` — available via PATH on all Windows systems

## Build Command Template

Since we're in a bash shell context, use `powershell.exe -NoProfile -Command` to invoke the build:

```bash
powershell.exe -NoProfile -Command "
  cd <sdk_root>;
  .\\export.ps1;
  cd <project_dir>;
  scons --board=<board_name> -j8
" 2>&1
```

### Parameters

| Parameter | Description | Example |
|-----------|-------------|---------|
| `<sdk_root>` | Absolute path to SDK root directory | `E:\\code2\\SiFli-SDK-52S` |
| `<project_dir>` | Relative path (from SDK root) to the project containing `SConstruct` | `example\\get-started\\hello_world\\rtt\\project` |
| `<board_name>` | Board target name (with or without `_hcpu` suffix) | `sf32lb52-lcd_n16r8_hcpu` |

The `<sdk_root>` should be determined from the current working directory or the user's setup. If the user doesn't specify, use the working directory of the session.

## Finding Valid Board Names

List available boards in `customer/boards/`:

```bash
ls customer/boards/
```

Boards often have `_hcpu` or `_lcpu` subdirectories. The build target name includes the chip family prefix (e.g. `sf32lb52-`, `sf32lb56-`, `sf32lb58-`, `eh-lb5xx`, `ec-lb5xx`).

## Finding Example Projects

Examples are in `example/`. Each project directory should contain `SConstruct` (or `SConstruct` inside a `project/` subdirectory).

Common patterns:
- `example/get-started/hello_world/rtt/project/` — RT-Thread hello world
- `example/ble/<name>/project/hcpu/` — BLE examples (dual-core, HCPU side)
- `example/multimedia/lvgl/<name>/project/` — LVGL graphics examples

## Build Output

After a successful build, artifacts are in the project's `build_<board_name>/` directory:
- `main.elf` — ELF executable
- `main.asm` — Disassembly listing
- `output/` — Flashable binary/hex files
- `ftab.bin` — Flash partition table
- `download.bat` — Download script

A memory usage summary is printed at the end showing ROM/RAM utilization.

## Common Issues

1. **`scons: command not found`** — The SDK environment wasn't sourced properly. Make sure to run `export.ps1` first in the same PowerShell session.
2. **Board not found** — Verify the board name exists in `customer/boards/`. Try with and without `_hcpu`/`_lcpu` suffix.
3. **Permission denied** — If a build already exists, clean with `scons --board=<board> -c` first.
4. **Worktree dirty warning** — Non-blocking; the build will proceed.

## Example Usage

User: "编译 helloworld/rtt，板子 sf32lb52-lcd_n16r8"

→ `<sdk_root>` = session's working directory
→ Set `<project_dir>` = `example\\get-started\\hello_world\\rtt\\project`
→ Set `<board_name>` = `sf32lb52-lcd_n16r8_hcpu`
→ Run the build command template above.