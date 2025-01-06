# micropython示例
@使用指南
    @介绍
        micropython应用会在板子flash上面验证mycropython的功能，文件系统采用FAT格式，在UART console可以调用常用的文件命令，如:
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
        python           - run python script or start with python REPL.

      
 @工程说明
        - 编译方法: 进入project目录执行命令`scons --board=<board_name> -j8`， 其中board_name为板子名称，例如编译eh-lb561板子，完整命令为`scons --board=eh-lb561 -j8`
          编译生成的image文件存放在HCPU的build_<board_name>目录下，common工程的用法参考通<<用工程构建方法>>
        - 模拟器编译方法：进入simulator目录执行命令 'scons -j8', 运行时, 在 simulator目录执行 build\bf0_ap.exe
        