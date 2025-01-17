@REM usage: set_env [toolchain]
@REM toolchain could be: keil | gcc | iar
@echo off
set SIFLI_SDK=%~dp0
set PYTHONPATH= %PYTHONPATH%;%SIFLI_SDK%tools\build;%SIFLI_SDK%tools\build\default;
set SIFLI_SDK=%SIFLI_SDK:\=/%
set PATH=%SIFLI_SDK%tools\menuconfig\dist;%PATH%;%SIFLI_SDK%tools\scripts;

REM Use keil by default
set RTT_CC=keil
if "%REG_KEIL_PATH%" NEQ "" (
    set RTT_EXEC_PATH=%REG_KEIL_PATH%
) else (
    set RTT_EXEC_PATH=C:/Keil_v5
)    

if "%1"=="gcc" goto :SET_GCC
if "%1"=="iar" goto :SET_IAR
if "%1"=="keil" goto :CHECK
if "%1"==""    goto :CHECK

echo Unsupported toolchain: %1.
echo Supported toolchain: keil, iar, gcc.
goto :END


:SET_GCC
set RTT_CC=gcc
REM @if "%REG_GCC_PATH%"=="" goto :PRINT_GCC_PATH_ERROR
REM @set RTT_EXEC_PATH=%REG_GCC_PATH%

if "%REG_GCC_PATH%" NEQ "" (
    set RTT_EXEC_PATH=%REG_GCC_PATH%
) else (
    if "%RTT_GCC_EXEC_PATH%" NEQ "" (
        set RTT_EXEC_PATH=%RTT_GCC_EXEC_PATH%
    ) else (
        set RTT_EXEC_PATH=%ENV_ROOT%\tools\gnu_gcc\arm_gcc\bin
    )
)
goto :CHECK

:SET_IAR
set RTT_CC=iar
REM replace with your IAR install path
if "%REG_IAR_PATH%" NEQ "" (
    set RTT_EXEC_PATH=%REG_IAR_PATH%
) else (
    set RTT_EXEC_PATH=C:/PROGRA~2/IARSYS~1/EMBEDD~1.2
)    
goto :CHECK

:CHECK
if not exist "%RTT_EXEC_PATH%" goto :PRINT_GCC_PATH_ERROR
goto :END

:PRINT_GCC_PATH_ERROR
echo ERROR: "%RTT_EXEC_PATH%" not exists. Please use SiFli env version above v1.1.1 and set correct toolchain path
exit /b 1

:END
echo set_env DONE.

