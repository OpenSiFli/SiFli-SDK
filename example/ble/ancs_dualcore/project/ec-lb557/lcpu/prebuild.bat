set rom_sym=rom.sym
set bin_path=..\..\..\..\..\rom_bin\lcpu_boot_loader
echo f|xcopy %bin_path%\%rom_sym% .\rom.sym /y