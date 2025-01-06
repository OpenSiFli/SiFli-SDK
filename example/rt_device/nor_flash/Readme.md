# nor_flash
@使用指南
    @介绍
        nor_flash 用来展示rt-thread 层的NOR FLASH读写擦功能接口。
        在编译前需要把对用的控制器配置成Nor接口，系统启动时会初始化NAND并进行坏块管理信息的初始化，这部分应用可以不用关注。
        NOR FLASH的擦操作需要以4KB对齐（包括地址和长度），读写没有要求。

      
 @工程说明
        - 编译方法: 进入project目录执行命令`scons --board=<board_name> -j8`， 其中board_name为板子名称，例如编译eh-lb561板子，完整命令为`scons --board=eh-lb561 -j8`
          编译生成的image文件存放在HCPU的build_<board_name>目录下，common工程的用法参考通<<用工程构建方法>>