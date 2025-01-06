..\..\..\..\tools\mkfatimg\mkfatimg.exe ..\jsroot build\jsroot.bin 2048 4096
echo loadbin build\jsroot.bin 0x14600000 > build\jsroot.jlink
echo exit >> build\jsroot.jlink
jlink.exe -device SF32LB58X -if SWD -speed 10000 -autoconnect 1 -CommandFile build\jsroot.jlink
