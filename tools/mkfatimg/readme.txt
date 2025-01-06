Usage:
1. Generate readonly disk image, disk size is autodetected
mkfatimg.exe -t resource resource.bin 0 8192

2. Generate RW disk image, disk size is autodetected, memory space is reserved
mkfatimg.exe resource resource.bin 0 8192

3. Generate RW disk image, disk size is specified by imgsize(sector number), if given disk size is smaller than required, disk image generation would fail.
mkfatimg.exe -t resource resource.bin imgsize 8192

imgsize=disk_size/8192, disk_size is expected image size in bytes.
sector_size is change to 8192



