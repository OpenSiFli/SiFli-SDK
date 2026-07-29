@echo Download Boot patch sigkey
@python download.py boot_patch_sigkey --key=%1sig --port=%2
@echo Download SEC BOOT patch image part 1 to flash id 9
@python download.py img --eimg=boot_ram_patch_sec.bin --bksize=512 --flashid=9 --verbose=2 --port=%2