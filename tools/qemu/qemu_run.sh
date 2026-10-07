#!/usr/bin/env bash
# Run a qemu_sf32lb52x build under QEMU MPS2-AN505.
#
# Usage: qemu_run.sh <path/to/main.elf>
#   default ELF: ../../example/qemu/hello_qemu/project/build_qemu_sf32lb52x_hcpu/main.elf
#
# Console output goes to the host terminal via ARM semihosting (BKPT 0xAB).
# The console device requires "-semihosting-config target=native".
# QEMU version tested: 9.x / 11.x (machine map may differ per version).

ELF="${1:-../../example/qemu/hello_qemu/project/build_qemu_sf32lb52x_hcpu/main.elf}"

if [ ! -f "$ELF" ]; then
    echo "ELF not found: $ELF"
    echo "Please build first: scons --board=qemu_sf32lb52x"
    exit 1
fi

exec qemu-system-arm -machine mps2-an505 -cpu cortex-m33 \
    -nographic \
    -semihosting -semihosting-config enable=on,target=native \
    -kernel "$ELF"
