..\..\..\..\tools\mkfatimg\mkfatimg.exe .\disk build\disk.bin 1024 4096
echo loadbin build\disk.bin 0x64800000 > build\disk.jlink
echo exit >>build\disk.jlink
jlink.exe -device SF32LB55X -if SWD -speed 10000 -autoconnect 1 -CommandFile build\disk.jlink