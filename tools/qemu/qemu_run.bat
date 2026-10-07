@echo off
REM Run a qemu_sf32lb52x build under QEMU MPS2-AN505.
REM
REM Usage: qemu_run.bat <path\to\main.elf>
REM   default ELF: ..\..\example\qemu\hello_qemu\project\build_qemu_sf32lb52x_hcpu\main.elf
REM
REM Console output goes to the host terminal via ARM semihosting (BKPT 0xAB).
REM The console device requires "-semihosting-config target=native".
REM QEMU version tested: 9.x / 11.x (machine map may differ per version).

setlocal
if "%~1"=="" (
    set ELF=..\..\example\qemu\hello_qemu\project\build_qemu_sf32lb52x_hcpu\main.elf
) else (
    set ELF=%~1
)

if not exist "%ELF%" (
    echo ELF not found: %ELF%
    echo Please build first: scons --board=qemu_sf32lb52x
    exit /b 1
)

qemu-system-arm -machine mps2-an505 -cpu cortex-m33 ^
    -nographic ^
    -semihosting -semihosting-config enable=on,target=native ^
    -kernel "%ELF%"

endlocal
