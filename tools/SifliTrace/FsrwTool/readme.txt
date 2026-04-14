命令行调用:

读取终端文件到PC
FsrwTool.exe  --rwtype 0  --port COM3  --baund 1000000  --hide 1  --pcfile  D:\2.txt  --dutfile 1.txt
echo %errorlevel%

传输文件到终端
FsrwTool.exe  --rwtype 1  --port COM3  --baund 1000000  --hide 1  --pcfile  D:\2.txt  --dutfile 1.txt
echo %errorlevel%

读取手表帧buffer
FsrwTool.exe  --rwtype 2  --port COM3  --baund 1000000  --pcfile d:\framebuf.bin

--hide 1 表示隐藏工具窗口，无该参数会显示工具窗口
返回0表示成功