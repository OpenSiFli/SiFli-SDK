@echo off
title=uart download
set WORK_PATH=%~dp0
set CURR_PATH=%cd%
cd %WORK_PATH%
:start
echo,
echo      Uart Download
echo,
set /p input=please input the serial port num:
goto download
:download
echo com%input%
ImgDownUart.exe --func 0 --port com%input% --baund 3000000 --loadram 1 --postact 1 --device SF32LB52X_NAND --file ImgBurnList.ini --log ImgBurn.log
if %errorlevel%==0 (
    echo Download Successful
)else (
    echo Download Failed
    echo logfile:%WORK_PATH%ImgBurn.log
)
cd %CURR_PATH%
pause

