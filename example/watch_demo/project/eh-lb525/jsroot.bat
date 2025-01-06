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

cd build

..\..\..\..\..\tools\mkfatimg\mkfatimg_nand\Release\mkfatimg.exe ..\..\jsroot jsroot.bin 8192 2048
echo [FILEINFO] >jsroot.ini
echo FILE0=jsroot.bin >> jsroot.ini
echo ADDR0=0x63060000 >> jsroot.ini
echo NUM=1 >> jsroot.ini




..\..\..\..\..\tools\uart_download\ImgDownUart.exe --func 0 --port com%input% --baund 3000000 --loadram 1 --postact 1 --device SF32LB52X_NAND --file jsroot.ini --log jsrootImgBurn.log
if %errorlevel%==0 (
    echo Download Successful
)else (
    echo Download Failed
    echo logfile:%WORK_PATH%jsrootImgBurn.log
)
cd %CURR_PATH%
pause
