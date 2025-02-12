从手表读文件
FsrwTool\FsrwTool.exe --rwtype 0 --port COM7 --baund 1000000 --dutfile ramfs/img_reset.bin  --pcfile d:\21.bin

往手表写文件
FsrwTool\FsrwTool.exe --rwtype 1 --port COM7 --baund 1000000 --dutfile ramfs/ttt.bin  --pcfile d:\21.bin

读取手表帧buffer
FsrwTool\FsrwTool.exe --rwtype 2 --port COM7 --baund 1000000 --pcfile d:\framebuf.bin
