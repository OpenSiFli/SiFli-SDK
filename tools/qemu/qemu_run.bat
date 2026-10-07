@echo off
REM Run a qemu_sf32lb52x build under QEMU MPS2-AN505.
REM
REM Usage: qemu_run.bat <path	o\main.elf>
REM   default ELF: ..\..\example\qemu\hello_qemu\project\build_qemu_sf32lb52x_hcpu\main.elf
REM
REM The console is the emulated UART0, wired to this terminal by "-serial stdio".
REM -display and -monitor are off so that stdio carries the console and nothing
REM else; semihosting is not used -- its SYS_READC blocks the vCPU.
REM QEMU version tested: 9.x / 11.x (machine map may differ per version).

setlocal
if defined SIFLI_QEMU_EXE (
    set QEMU=%SIFLI_QEMU_EXE%
) else (
    set QEMU=qemu-system-arm
)
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

"%QEMU%" -machine mps2-an505 -cpu cortex-m33 ^
    -display none -monitor none -serial stdio ^
    -kernel "%ELF%"

endlocal
