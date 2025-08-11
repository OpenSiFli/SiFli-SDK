# 分区表
`partition_table`（分区表）模块是一个编译脚本工具，用于解析分区表描述文件`ptab.json`，生成头文件`ptab.h`，`ptab.h`中包含了与分区信息相关的一系列宏定义，该头文件可被工程引用以获取分区信息。
分区表可以描述所有memory的地址规划，包括NOR Flash、NAND Flash、eMMC、TF卡、PSRAM和片内SRAM等。每个板子会有一个`ptab.json`，描述了该板子的分区信息，使用该板子编译的工程都遵循这个分区表定义的地址规划，
工程也可以使用自定义的`ptab.json`以覆盖板子的默认配置。

## 使能分区表
选中middleware中的`Use partition table to manage all memory layout`使能分区表功能
![](../../assets/partition_table/ptab_menuconfig.png)

同时还需要在工程的`Kconfig.proj`中定义如下开关，所有示例工程的`Kconfig.proj`都已包含了这段代码
```kconfig
#APP specific configuration.
config CUSTOM_MEM_MAP
    bool 
	select custom_mem_map
	default y if !SOC_SIMULATOR
```


## 分区表语法
分区表描述文件`ptab.json`是一个json格式的文本文件，遵循json语法，可以使用任意文本编辑器编辑。分区表的语法有1.0和2.0两个版本，具体语法参考[](partition_table_v1.md)和[](partition_table_v2.md)。


```{toctree}
:hidden:

partition_table_v1.md
partition_table_v2.md
```