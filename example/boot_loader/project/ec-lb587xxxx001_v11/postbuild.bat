REM set bin_path=..\..\..\rom_bin\hcpu_boot_loader
fromelf --bin %1 --output bootloader.bin
fromelf --i32 %1 --output bootloader.hex
fromelf --text -c %1 > build\keil\Obj\bootloader.lst

REM echo d| xcopy build\bootloader.symdefs %bin_path% /y
REM xcopy build\keil\obj\bootloader.axf %bin_path%\ /y
REM xcopy build\keil\obj\bootloader.lst %bin_path%\ /y
REM xcopy build\keil\list\bootloader.map %bin_path%\ /y
REM xcopy .\bootloader.bin %bin_path%\ /y
REM xcopy .\bootloader.hex %bin_path%\ /y