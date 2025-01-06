fromelf --bin %1 --output  build\lcpu.bin
fromelf --i32 %1 --output  build\lcpu.hex
fromelf --text -c %1 > build\keil\Obj\lcpu.lst
copy build\keil\list\lcpu.map build
copy build\keil\Obj\lcpu.axf build
::mkdir ../hcpu/build/lcpu/
python ../../../../../tools/patch/gen_src.py lcpu build\lcpu.bin ../hcpu/build/lcpu/
ren ..\hcpu\build\lcpu\lcpu_img.c lcpu.c


