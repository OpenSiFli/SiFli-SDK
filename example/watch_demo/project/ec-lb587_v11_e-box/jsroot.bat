copy ..\videos\*.* ..\jsroot /Y
rm build\jsroot.bin
..\..\..\..\tools\mkfatimg\mkfatimg_nand\Release\mkfatimg.exe ..\jsroot build\jsroot.bin 32768 2048
echo loadbin build\jsroot.bin 0x68800000 > build\jsroot.jlink
echo exit >> build\jsroot.jlink
jlink.exe -device SF32LB58X_NAND -if SWD -speed 10000 -autoconnect 1 -CommandFile build\jsroot.jlink
