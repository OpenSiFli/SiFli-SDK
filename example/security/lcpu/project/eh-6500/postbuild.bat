fromelf --bin %1 --output build\bf0_ap.bin
fromelf --i32 %1 --output build\bf0_ap.hex
fromelf --text -c %1 > build\keil\Obj\bf0_ap.lst
rem python ../../../../../tools/patch/gen_src.py lcpu build/bf0_ap.bin ../hcpu/applications/
rem python ../../../../../tools/patch/gen_src.py lcpu build/bf0_ap.bin ../hcpu_v5/applications/
rem python ../../../../../tools/patch/gen_src.py lcpu build/bf0_ap.bin ../hcpu_big/applications/
copy build\keil\list\bf0_ap.map build
copy build\keil\Obj\bf0_ap.axf build
copy build\keil\Obj\bf0_ap.lst build