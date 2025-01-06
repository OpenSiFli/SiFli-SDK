fromelf --bin %1 --output build\bf0_ap.bin
fromelf --i32 %1 --output build\bf0_ap.hex
fromelf --text -c %1 > build\keil\Obj\bf0_ap.lst
python ../../../../../tools/patch/gen_src.py lcpu build/bf0_ap.bin ../../../../rom_bin/lcpu_general_ble_img/
move ..\..\..\..\rom_bin\lcpu_general_ble_img\lcpu_img.c ..\..\..\..\rom_bin\lcpu_general_ble_img\lcpu_lb557.c 
copy build\keil\list\bf0_ap.map build
copy build\keil\Obj\bf0_ap.axf build
copy build\keil\Obj\bf0_ap.lst build
copy build\bf0_ap.axf ..\..\..\..\rom_bin\lcpu_general_ble_img\lcpu_general_557.axf