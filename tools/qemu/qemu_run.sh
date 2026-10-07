#!/usr/bin/env bash
# Run a qemu_sf32lb52x build under QEMU MPS2-AN505.
#
# Usage: qemu_run.sh <path/to/main.elf>
#   default ELF: ../../example/qemu/hello_qemu/project/build_qemu_sf32lb52x_hcpu/main.elf
#
# The console is the emulated UART0, wired to this terminal by "-serial stdio".
# -display and -monitor are off so that stdio carries the console and nothing
# else; semihosting is not used -- its SYS_READC blocks the vCPU.
# QEMU version tested: 9.x / 11.x (machine map may differ per version).
# SIFLI_QEMU_EXE names the binary when it is not on PATH.

ELF="${1:-../../example/qemu/hello_qemu/project/build_qemu_sf32lb52x_hcpu/main.elf}"

if [ ! -f "$ELF" ]; then
    echo "ELF not found: $ELF"
    echo "Please build first: scons --board=qemu_sf32lb52x"
    exit 1
fi

exec "${SIFLI_QEMU_EXE:-qemu-system-arm}" -machine mps2-an505 -cpu cortex-m33 \
    -display none -monitor none -serial stdio \
    -kernel "$ELF"
