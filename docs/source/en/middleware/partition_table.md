# Partition Table
The `partition_table` module is a build script tool that parses the partition
table description file `ptab.json` and generates the header file `ptab.h`. This
header file contains a series of macro definitions related to partition
information, which can be included in projects to obtain partition details.

## Enabling the Partition Table
Select `Use partition table to manage all memory layout` in the middleware to
enable the partition table feature:
![](../../assets/partition_table/ptab_menuconfig.png)

You also need to define the following switch in your project’s `Kconfig.proj`.
This snippet is already included in all example projects.
```kconfig
# Application-specific configuration.
config CUSTOM_MEM_MAP
    bool 
	select custom_mem_map
	default y if !SOC_SIMULATOR
```


## Partition Table Syntax
The partition table description file `ptab.json` is a text file in JSON format.
It follows standard JSON syntax and can be edited with any text editor. There
are two versions of the partition table syntax: 1.0 and 2.0. The 2.0 version
simplifies the syntax compared to 1.0, making it easier to understand and
maintain. It is recommended that new projects adopt the 2.0 syntax. For detailed
syntax, refer to [](partition_table_v1.md) and [](partition_table_v2.md).


```{toctree}
:hidden:

partition_table_v1.md
partition_table_v2.md
```
