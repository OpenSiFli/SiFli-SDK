fromelf --bin %1 --output build\dfu.bin
fromelf --i32 %1 --output build\dfu.hex
fromelf --text -c %1 > build\keil\Obj\dfu.lst

copy build\keil\list\dfu.map build
copy build\keil\Obj\dfu.axf build
copy build\keil\Obj\dfu.lst build

set bin_path=..\..\..\rom_bin\hcpu_dfu
copy build\dfu.axf %bin_path%\dfu_micro.axf /y
copy build\dfu.bin %bin_path%\dfu_micro.bin /y
copy build\dfu.hex %bin_path%\dfu_micro.hex /y