set rom_sym=rom_lite.sym
set bin_path=..\..\..\..\..\example\rom_bin\lcpu_boot_loader
echo f|xcopy %bin_path%\%rom_sym% build\lcpu_rom\rom_lite.sym /y
python filter.py --src=build/lcpu_rom/rom_lite.sym --dst=build/lcpu_rom/rom_lite.lib

