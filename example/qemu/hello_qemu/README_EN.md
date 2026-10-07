# Hello QEMU Example (RT-Thread)

Source path: example/qemu/hello_qemu

## Overview
An RT-Thread example that runs on the `qemu_sf32lb52x` virtual board (QEMU
MPS2-AN505 machine) and prints "Hello SiFli on QEMU!" to the host terminal via
semihosting.

This is a pure RT-Thread application that does not use the SiFli HAL, so it runs
on any of the five chip-identified carriers (`qemu_sf32lb52x` through
`qemu_sf32lb58x`); `qemu_sf32lb52x` is the one used below.

## Usage

### Build
Switch to the example project directory and run:
```
scons --board=qemu_sf32lb52x -j8
```
Output: `build_qemu_sf32lb52x_hcpu/main.elf`

### Run
Requires QEMU (`qemu-system-arm`):
```
qemu-system-arm -machine mps2-an505 -cpu cortex-m33 -nographic \
    -semihosting -semihosting-config enable=on,target=native \
    -kernel build_qemu_sf32lb52x_hcpu/main.elf
```
Or via script:
```
python tools/qemu/qemu_smoke_test.py
```
Expected output: RT-Thread banner + `Hello SiFli on QEMU!` + `msh />` prompt.

### Debug
```
tools/qemu/qemu_debug.bat   # starts QEMU + GDB server (tcp:1234) + gdb
```
