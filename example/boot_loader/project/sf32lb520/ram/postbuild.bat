fromelf --bin %1 --output build\bootloader.bin
fromelf --i32 %1 --output build\bootloader.hex
fromelf --text -c %1 > build\keil\Obj\bootloader.lst

copy build\keil\list\bootloader.map build
copy build\keil\Obj\bootloader.axf build
copy build\keil\Obj\bootloader.lst build

set bin_path=..\..\..\rom_bin\hcpu_boot_loader
copy build\bootloader.axf %bin_path%\bootloader_micro.axf /y
copy build\bootloader.bin %bin_path%\bootloader_micro.bin /y
copy build\bootloader.hex %bin_path%\bootloader_micro.hex /y