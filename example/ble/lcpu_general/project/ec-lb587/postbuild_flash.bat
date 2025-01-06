fromelf --bin %1 --output build\bf0_ap.bin
fromelf --i32 %1 --output build\bf0_ap.hex
fromelf --text -c %1 > build\keil\Obj\bf0_ap.lst

xcopy %1 build\ /y
xcopy build\keil\obj\bf0_ap.lst build\ /y
xcopy build\keil\list\bf0_ap.map build\ /y

del build\bf0_ap.bin\*.bin
del build\bf0_ap.hex\*.hex
xcopy  build\bf0_ap.bin\* build\bf0_ap.bin\*.bin /y
xcopy  build\bf0_ap.hex\* build\bf0_ap.hex\*.hex /y

python ../../../../../tools/patch/gen_src.py lcpu build/bf0_ap.bin/ER_IROM1.bin ../../../../rom_bin/lcpu_general_ble_img/
move ..\..\..\..\rom_bin\lcpu_general_ble_img\lcpu_img.c ..\..\..\..\rom_bin\lcpu_general_ble_img\lcpu_pro.c 
copy build\keil\list\bf0_ap.map build
copy build\keil\Obj\bf0_ap.axf build
copy build\keil\Obj\bf0_ap.lst build
copy build\bf0_ap.axf ..\..\..\..\rom_bin\lcpu_general_ble_img\lcpu_general_pro.axf