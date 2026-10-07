#!/usr/bin/env python3
"""QEMU smoke test for the qemu_sf32lb52x board.

Boots a qemu_sf32lb52x build under QEMU MPS2-AN505 and asserts that the
expected console strings appear within a timeout. Suitable for CI.

Usage:
    python qemu_smoke_test.py [<path/to/main.elf>] [--timeout 20]

Exit code 0 on success, 1 on failure.
"""

import argparse
import os
import subprocess
import sys
import time

DEFAULT_ELF = "../../example/qemu/hello_qemu/project/build_qemu_sf32lb52x_hcpu/main.elf"

# Strings that must appear on the console, in order
EXPECTED = [
    "SiFli Corporation",     # RT-Thread (SiFli) kernel banner
    "Hello SiFli on QEMU!",  # application output
]

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("elf", nargs="?", default=DEFAULT_ELF)
    parser.add_argument("--timeout", type=float, default=20.0)
    args = parser.parse_args()

    qemu_cmd = [
        os.environ.get("SIFLI_QEMU_EXE", "qemu-system-arm"),
        "-machine", "mps2-an505",
        "-cpu", "cortex-m33",
        "-display", "none",
        "-monitor", "none",
        "-serial", "stdio",
        "-kernel", args.elf,
    ]

    print("Running: " + " ".join(qemu_cmd))
    proc = subprocess.Popen(qemu_cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, bufsize=1)

    output = []
    deadline = time.time() + args.timeout
    seen = [False] * len(EXPECTED)

    try:
        while time.time() < deadline:
            line = proc.stdout.readline()
            if not line:
                break
            line = line.rstrip("\n")
            output.append(line)
            print(line)
            for i, want in enumerate(EXPECTED):
                if not seen[i] and want in line:
                    seen[i] = True
                    print("  [OK] found: %s" % want)
            if all(seen):
                break
    finally:
        if proc.poll() is None:
            proc.kill()

    ok = all(seen)
    if not ok:
        print("=== SMOKE TEST FAILED ===")
        for i, want in enumerate(EXPECTED):
            if not seen[i]:
                print("  missing: %s" % want)
        print("=== output (%d lines) ===" % len(output))
    else:
        print("=== SMOKE TEST PASSED ===")
    return 0 if ok else 1

if __name__ == "__main__":
    sys.exit(main())
