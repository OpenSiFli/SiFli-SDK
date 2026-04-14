# FatFs NAND 示例
源码路径：example/storage/fatfs/nand

## 支持的开发板
- sf32lb52-lcd_a128r16
- sf32lb56-lcd_a128r12n1
- sf32lb58-lcd_a128r32n1


## 使用指南
本示例演示FatFs的文件系统功能，文件系统采用FAT格式，在UART console可以调用常用的文件命令，如:

```
df               - Disk free
mountfs          - Mount device to file system
mkfs             - Format disk with file system
mkdir            - Create the DIRECTORY.
pwd              - Print the name of the current working directory.
cd               - Change the shell working directory.
rm               - Remove(unlink) the FILE(s).
cat              - Concatenate FILE(s)
mv               - Rename SOURCE to DEST.
cp               - Copy SOURCE to DEST.
ls               - List information about the FILEs.

```
### 性能测试
本示例包含文件系统性能测试功能，可通过`fs_test`命令进行测试:
- 使用`fs_test [size_kb] [buffer_kb]`进行指定大小数据的读写性能测试
- 支持不同buffer大小(1K, 2K, 4K, 8K, 16K, 32K)的测试
- 测试内容包括写入速度和读取速度

测试预期结果如下：
```c
TX:fs_test
Creating test file with 1024 KB data...
Written 0 KB...
Written 100 KB...
Written 200 KB...
Written 300 KB...
Written 400 KB...
Written 500 KB...
Written 600 KB...
Written 700 KB...
Written 800 KB...
Written 900 KB...
Written 1000 KB...
Write test completed
Write Speed: 1721799 bytes/sec (1681.44 KB/s)
Write Ops/sec: 1681 ops/sec
Reading test file...
Read 0 KB...
Read 100 KB...
Read 200 KB...
Read 300 KB...
Read 400 KB...
Read 500 KB...
Read 600 KB...
Read 700 KB...
Read 800 KB...
Read 900 KB...
Read 1000 KB...
Read test completed
Read Speed: 5518821 bytes/sec (5389.47 KB/s)
Read Ops/sec: 5389 ops/sec
```

### 文件系统打包

默认的编译脚本没有下载文件系统分区镜像文件，因此第一次运行程序如果mount失败会自动格式化分区，具体实现见`main.c`的函数`mnt_init`。SDK还提供了打包指定目录下的文件并生成文件系统镜像文件的功能，可以放开`SConstruct`里的如下代码，这段代码的作用是在编译时打包disk目录下的文件，在build目录下生成`fs_root.bin`文件，如果`ptab.json`的分区表中定义了`img`属性为`fs_root`的分区，下载脚本就会同时下载该bin文件。
      
```
# fs_bin=FileSystemBuild( "../disk", env)
# AddCustomImg("fs_root",bin=[fs_bin])
```
## 例程的使用说明
### 硬件需求
1、运行例程的前提，需要拥有一块支持该例程的开发板
2、一根具备数据传输能力的USB数据线
### menuconfig配置
```
 //执行指令
 menuconfig --board=em-lb561
```  
1、首先需要在menuconfig里面进行开启MTD Dhara Nand Flash device

![alt text](assets/file_system_1.png)
2、使用设备虚拟文件系统

![alt text](assets/file_system_2.png)
3、选择HAL Assert类型

![alt text](assets/file_system_3.png)

## 工程说明
- 编译方法：进入project目录执行命令`scons --board=<board_name> -j8`， 其中board_name为板子名称，例如编译eh-lb561板子，完整命令为`scons --board=eh-lb561 -j8`编译生成的image文件存放在HCPU的build_<board_name>目录下，common工程的用法参考通<<用工程构建方法>>
- 下载方法： 进入project目录执行命令`build_<board_name>_hcpu\download.bat(uart_download.bat)` ,其中board_name为板子名称，后面download前面带了uart是通过串口进行程序烧入，不带是进行jlink烧入（这个得根本板子型号是否支持jlink烧入），例如编译eh-lb561板子，完整命令为`build_en-lb561_hcpu\download.bat`

## 例程输出结果展示
下面结果展示了例程在开发板运行起来后的log。如果看不到这些log，就说明例程没能按预期运行成功，需要进行原因排查。
```
mount fs on flash root success//说明文件系统挂载成功
```
1、串口发送ls查看根目录下的文件。

2、在输入mkdir test1，创建test1文件夹（目录）。

3、再次输入cd+目录名，到XXX目录下，输入pwd进行查看当前工作路径是否在cd到的目录下 。

4、可在该工作目录下再次进行创建mkdir test2，ls查看该目录下是否进行创建成功。
![alt text](assets/file_system_log_1.png)
### 故障排查
如果log未能出现预期的log和现象，可以从已下几个方面进行故障排查：
* 硬件连接是否正常
* 检查USB线是否具备数据传输功能
* 上面的menu是否配置对了（特别板子flash型号）
