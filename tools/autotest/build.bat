REM Install python package in \\ftpserver\Software2\regression\gitroot\wvt_v10\utilities\python packages\graphical module(s)\colorama-0.3.9
REM Install serial port driver in \\ftpserver\Software2\ganghe\LCUS-2型双路USB 继电器\驱动
REM Add D:\Users\regression\gitroot\rt-thread\env\env\tools\Python27\Scripts to PATH
REM echo on

set SIFLI_SDK=%cd%

set TOOLS_ROOT=%cd%/tools

call %ENV_ROOT%\tools\ConEmu\ConEmu\CmdInit.cmd

dir
call set_env.bat gcc
cd %1

date /t && time /t

if exist resources (
    cd resources
    call scons --res=%1
    if %errorlevel% neq 0 exit /b %errorlevel%
    cd ..
)

if "%2"=="" goto :BUILD_WITHOUT_ARG
if "%2"=="--coremark" goto :BUILD_COREMARK
if "%2"=="--resolution" goto :GEN_RESOURCE
if "%2"=="--resource" goto :GEN_RESOURCE2
if "%2"=="--board" goto :BUILD_COMMON_PRJ

goto :BUILD_WITHOUT_ARG


:GEN_RESOURCE2
call scons --resource
goto :BUILD_WITHOUT_ARG

:GEN_RESOURCE
set resolution=%3
%TOOLS_ROOT%/sifli_develop/png2ezip/ezip -dir ..\..\resource\images -cfile 1 %resolution%

mkdir ..\..\resource\strings\output
%TOOLS_ROOT%/sifli_develop/language_gen/lang_gen.exe ..\..\resource\strings  %TOOLS_ROOT%/sifli_develop/language_scan/lang.exe  ..\..\..

:BUILD_WITHOUT_ARG
call scons -j8
set build_result=%errorlevel%
goto :BUILD_END

:BUILD_COREMARK
call scons -j8 %2=%3
set build_result=%errorlevel%
goto :BUILD_END

:BUILD_COMMON_PRJ
call scons %2=%3 -j8 
set build_result=%errorlevel%
goto :BUILD_END

:BUILD_END
date /t && time /t
cd %SIFLI_SDK%
exit /b %build_result%
