..\..\..\..\tools\mkfatimg\mkfatimg_nand\Release\mkfatimg.exe ..\jsroot build\jsroot.bin 4096 2048
echo loadbin build\jsroot.bin 0x64400000 > build\jsroot.jlink
echo exit >> build\jsroot.jlink
jlink.exe -device SF32LB56X_NAND -if SWD -speed 10000 -autoconnect 1 -CommandFile build\jsroot.jlink