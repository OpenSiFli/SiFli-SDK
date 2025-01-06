@REM usage: set_env [toolchain]
@REM toolchain could be: keil | gcc | iar
@set SIFLI_SDK=%~dp0
@set PYTHONPATH= %PYTHONPATH%;%SIFLI_SDK%tools\build;%SIFLI_SDK%tools\build\default;
@set SIFLI_SDK=%SIFLI_SDK:\=/%
@set PATH=%SIFLI_SDK%tools\menuconfig\dist;%PATH%;%SIFLI_SDK%tools\scripts;

@REM Use keil by default
@set RTT_CC=keil
@set RTT_EXEC_PATH=C:/Keil_v5

@if "%1"=="gcc" goto :SET_GCC
@if "%1"=="iar" goto :SET_IAR
@if "%1"=="keil" goto :END
@if "%1"==""    goto :END

@echo Unsupported toolchain: %1.
@echo Supported toolchain: keil, iar, gcc.
@goto :END


:SET_GCC
@set RTT_CC=gcc
@if "%REG_GCC_PATH%"=="" goto :PRINT_GCC_PATH_ERROR
@set RTT_EXEC_PATH=%REG_GCC_PATH%
@REM %SIFLI_SDK%..\rt-thread\env\env\tools\gnu_gcc\arm_gcc\bin
@goto :END

:SET_IAR
@set RTT_CC=iar
@REM replace with your IAR install path
@set RTT_EXEC_PATH=C:/PROGRA~2/IARSYS~1/EMBEDD~1.2
@goto :END

:PRINT_GCC_PATH_ERROR
@echo ERROR: REG_GCC_PATH not defined. Please use SiFli env version >=v1.1.1
@exit /b 1

:END
@echo set_env DONE.

