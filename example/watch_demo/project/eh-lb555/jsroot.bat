REM WF4 has apng which is not supported by SF32LB55X
ren ..\jsroot\JW_wf4 _JW_wf4
..\..\..\..\tools\mkfatimg\mkfatimg.exe ..\jsroot build\jsroot.bin 1024 8192
ren ..\jsroot\_JW_wf4 JW_wf4
echo loadbin build\jsroot.bin 0x65000000 > build\jsroot.jlink
echo exit >> build\jsroot.jlink
jlink.exe -device SF32LB55X -if SWD -speed 10000 -autoconnect 1 -CommandFile build\jsroot.jlink

