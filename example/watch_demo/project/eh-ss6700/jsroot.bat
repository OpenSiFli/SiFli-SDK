..\..\..\..\tools\mkfatimg\mkfatimg.exe ..\jsroot build\jsroot.bin 1024 4096
echo loadbin build\jsroot.bin 0x14580000 > build\jsroot.jlink
echo exit >> build\jsroot.jlink
jlink.exe -device SF32LB56X -if SWD -speed 10000 -autoconnect 1 -CommandFile build\jsroot.jlink
