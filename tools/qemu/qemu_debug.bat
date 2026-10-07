@echo off
REM Debug a qemu_sf32lb52x build with GDB under QEMU MPS2-AN505.
REM
REM Starts QEMU with a GDB server on tcp:1234, then launches gdb.
REM
REM Usage: qemu_debug.bat <path\to\main.elf>
REM   default ELF: ..\..\example\qemu\hello_qemu\project\build_qemu_sf32lb52x_hcpu\main.elf

setlocal
if "%~1"=="" (
    set ELF=..\..\example\qemu\hello_qemu\project\build_qemu_sf32lb52x_hcpu\main.elf
) else (
    set ELF=%~1
)

if not exist "%ELF%" (
    echo ELF not found: %ELF%
    exit /b 1
)

start "qemu" qemu-system-arm -machine mps2-an505 -cpu cortex-m33 ^
    -nographic -semihosting -semihosting-config enable=on,target=native ^
    -kernel "%ELF%" -s -S

echo QEMU waiting for GDB on tcp:1234 ...
arm-none-eabi-gdb "%ELF%" -ex "target remote :1234"

endlocal
