set WORK_PATH=%~dp0
set CURR_PATH=%cd%
cd %WORK_PATH%
jlink.exe -device SF32LB56X_NAND -CommandFile download_fs.jlink
cd %CURR_PATH%
