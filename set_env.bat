@REM usage: set_env [toolchain]
@REM toolchain could be: keil | gcc | iar
@echo off
set SIFLI_SDK=%~dp0
set SIFLI_SDK_PATH=%~dp0
set PKGS_DIR=%PKGS_ROOT%

if not defined ORG_PATH (
    echo Please upgrate env to v1.1.2 or greater
    echo set_env FAIL
    exit /b 1
)

REM Use keil by default
set RTT_CC=keil
if defined REG_KEIL_PATH (
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

set RTT_EXEC_PATH=%ENV_ROOT%\tools\gnu_gcc\arm_gcc_14.2.1\bin
goto :CHECK

:SET_IAR
set RTT_CC=iar
REM replace with your IAR install path
if defined REG_IAR_PATH (
    set RTT_EXEC_PATH=%REG_IAR_PATH%
) else (
    set RTT_EXEC_PATH=C:/PROGRA~2/IARSYS~1/EMBEDD~1.2
)    
goto :CHECK

:CHECK
REM change default python2.7 to python3.11
set PYTHONPATH=%ENV_ROOT%\tools\python-3.11.9-amd64
set PYTHONHOME=%ENV_ROOT%\tools\python-3.11.9-amd64
set SCONS=%PYTHONPATH%\Scripts
set PATH=%ENV_ROOT%\tools\git\Git\bin;%ORG_PATH%
set PATH=%ENV_ROOT%\tools\bin;%PATH%
set PATH=%RTT_EXEC_PATH%;%PATH%
set PATH=%PYTHONHOME%;%PATH%
set PATH=%PYTHONPATH%;%PATH%
set PATH=%SCONS%;%PATH%
set PATH=%ENV_ROOT%\tools\qemu\qemu32;%PATH%

set PYTHONPATH= %PYTHONPATH%;%SIFLI_SDK%tools\build;%SIFLI_SDK%tools\build\default;
set SIFLI_SDK=%SIFLI_SDK:\=/%
set PATH=%SIFLI_SDK%tools\menuconfig\dist;%PATH%;%SIFLI_SDK%tools\scripts;
set LEGACY_ENV=1

if not exist "%RTT_EXEC_PATH%" goto :PRINT_GCC_PATH_ERROR
goto :END

:PRINT_GCC_PATH_ERROR
echo ERROR: "%RTT_EXEC_PATH%" not exists. Please use SiFli env version above v1.1.1 and set correct toolchain path
exit /b 1

:END
echo set_env DONE.

