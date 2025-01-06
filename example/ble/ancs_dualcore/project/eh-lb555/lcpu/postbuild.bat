fromelf --bin %1 --output  build\bf0_ap.bin
fromelf --i32 %1 --output  build\bf0_ap.hex
fromelf --text -c %1 > build\keil\Obj\bf0_ap.lst
copy build\keil\list\lcpu.map build
copy build\keil\Obj\lcpu.axf build
python ../../../../../../tools/patch/gen_src.py lcpu build\bf0_ap.bin  ../../../hcpu 


