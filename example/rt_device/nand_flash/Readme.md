# nand_flash
@使用指南
    @介绍
        nand_flash 用来展示rt-thread 层的NAND读写擦功能接口。
        在编译前需要把对用的控制器配置成NAND接口，系统启动时会初始化NAND并进行坏块管理信息的初始化，这部分应用可以不用关注。
        NAND的擦操作必须要按照块对齐操作（包括地址和长度），块的大小可以通过接口获取。
        NAND写操作必须按照页对齐操作（包括地址和长度），页的大小可以通过接口获取。读操作建议按照页对齐操作。

      
 @工程说明
        - 编译方法: 进入project目录执行命令`scons --board=<board_name> -j8`， 其中board_name为板子名称，例如编译eh-lb563板子，完整命令为`scons --board=eh-lb563 -j8`
          编译生成的image文件存放在HCPU的build_<board_name>目录下，common工程的用法参考通<<用工程构建方法>>