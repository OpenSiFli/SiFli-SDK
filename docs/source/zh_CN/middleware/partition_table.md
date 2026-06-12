# 分区表
`partition_table`（分区表）模块是一个编译脚本工具，用于解析分区表描述文件并生成头文件`ptab.h`，`ptab.h`中包含了与分区信息相关的一系列宏定义，该头文件可被工程引用以获取分区信息。
分区表可以描述所有memory的地址规划，包括NOR Flash、NAND Flash、eMMC、TF卡、PSRAM和片内SRAM等。每个板子会有自己的分区表描述，使用该板子编译的工程都遵循对应分区表定义的地址规划；
工程也可以使用自定义 PTAB 覆盖板子的默认配置。

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
SDK 目前同时支持两类 PTAB 语法：

- v1/v2：使用 `ptab.json`
- v3：使用 `ptab.yaml`

其中，v3 提供了更清晰的 YAML 结构、芯片内部分区建模和工程级 `ptab.overlay.yaml` 支持。新项目推荐优先采用 v3。

具体语法参考：

- [](partition_table_v1.md)
- [](partition_table_v2.md)
- [](partition_table_v3.md)

## 查看 ftab.bin

工程构建完成后，构建目录中会生成 `ftab.bin`，这是由分区表生成的二进制 Flash Table。排查启动地址、镜像大小、XIP 地址或 DFU 镜像标志时，可以使用 `sdk.py ftab-dump` 将该文件解析为可读内容。

使用前需要先初始化 SDK 环境：

```bash
source ./export.sh
```

Windows PowerShell 下执行：

```powershell
./export.ps1
```

在工程目录下执行以下命令，会自动查找该工程目录中的第一个 `build_*/ftab.bin` 并输出：

```bash
sdk.py ftab-dump
```

如果同一个工程目录中存在多个板子的构建目录，建议显式指定文件路径，避免查看到非预期的构建产物：

```bash
sdk.py ftab-dump --path build_<board_name>/ftab.bin
```

默认输出为表格格式；如果需要给脚本处理，可以输出 JSON：

```bash
sdk.py ftab-dump --path build_<board_name>/ftab.bin --format json
```

参数说明：

| 参数 | 说明 |
|------|------|
| `-p, --path <path>` | `ftab.bin` 文件路径。省略时，`sdk.py` 会在当前工程目录中查找 `build_*/ftab.bin`。 |
| `-f, --format <format>` | 输出格式：`table` 或 `json`，默认为 `table`；需要机器可读输出时使用 `json`。 |

表格输出包含三部分：

| 部分 | 说明 |
|------|------|
| Partition Table Entries | Flash Table 分区条目，包括镜像存储基地址、分区大小、XIP 或执行基地址以及 flags。 |
| Image Description Table | DFU 和启动流程使用的镜像元信息，包括镜像长度、块大小、`AUTO`/`COMPRESS` 等 flags，以及条目是否有效。 |
| Image Index Table | 各 core 镜像槽位的运行镜像指针，例如 HCPU、LCPU、bootloader 和 boot image。 |


```{toctree}
:hidden:

partition_table_v1.md
partition_table_v2.md
partition_table_v3.md
```
