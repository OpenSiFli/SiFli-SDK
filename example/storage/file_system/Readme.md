<<<<<<< PATCH SET (283343 [int][bug][edut file_system] Split file_system)
=======
# file_system示例
@使用指南
    @介绍
        file_system应用会在板子flash上面验证文件系统的功能，文件系统采用FAT格式，在UART console可以调用常用的文件命令，如:
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
   @文件系统打包
      默认的编译脚本没有下载文件系统分区镜像文件，因此第一次运行程序如果mount失败会自动格式化分区，具体实现见`main.c`的函数`mnt_init`。SDK还提供了打包指定目录下的文件并生成文件系统镜像文件的功能，可以放开`SConstruct`里的如下代码，这段代码的作用是在编译时打包disk目录下的文件，在build目录下生成`fs_root.bin`文件，如果`ptab.json`的分区表中定义了`img`属性为`fs_root`的分区，下载脚本就会同时下载该bin文件。
      ```
      # fs_bin=FileSystemBuild( "../disk", env)
      # AddCustomImg("fs_root",bin=[fs_bin])
      ```
      
 @工程说明
        - 编译方法: 进入project目录执行命令`scons --board=<board_name> -j8`， 其中board_name为板子名称，例如编译eh-lb561板子，完整命令为`scons --board=eh-lb561 -j8`
          编译生成的image文件存放在HCPU的build_<board_name>目录下，common工程的用法参考通<<用工程构建方法>>
>>>>>>> BASE      (87f518 [opt][board] Change rc32k freq to 32000Hz and remove unused )
