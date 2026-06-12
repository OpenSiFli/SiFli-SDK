# Partition Table

The `partition_table` module is a build script tool that parses the partition table description file `ptab.json` and generates the header file `ptab.h`. This header file contains a series of macro definitions related to partition information, which can be included in projects to obtain partition details.

The partition table can describe the address mapping for all memory types, including NOR Flash, NAND Flash, eMMC, TF cards, PSRAM, and on-chip SRAM. Each board has its own `ptab.json` that describes the partition information for that board. Any project compiled for that board must follow the address mapping defined by the partition table. Projects can also use a custom `ptab.json` to override the board’s default configuration.

## Enabling the Partition Table

Select `Use partition table to manage all memory layout` in the middleware to enable the partition table feature:
![](../../assets/partition_table/ptab_menuconfig.png)

You also need to define the following switch in your project’s `Kconfig.proj`. This snippet is already included in all example projects.

```kconfig
#APP specific configuration.
config CUSTOM_MEM_MAP
    bool 
	select custom_mem_map
	default y if !SOC_SIMULATOR
```

## Partition Table Syntax

The partition table description file `ptab.json` is a text file in JSON format. It follows standard JSON syntax and can be edited with any text editor. There are two versions of the partition table syntax: 1.0 and 2.0. The 2.0 version simplifies the syntax compared to 1.0, making it easier to understand and maintain. It is recommended that new projects adopt the 2.0 syntax. For detailed syntax, refer to [](partition_table_v1.md) and [](partition_table_v2.md).

## Dumping ftab.bin

After a project is built, the build directory contains `ftab.bin`, the binary Flash Table generated from the partition table. Use `sdk.py ftab-dump` to inspect this file in a readable form when checking boot addresses, image sizes, XIP addresses, or DFU image flags.

Run the command in an exported SDK environment:

```bash
source ./export.sh
```

On Windows PowerShell, run:

```powershell
./export.ps1
```

From the project directory, dump the first `build_*/ftab.bin` found under that project:

```bash
sdk.py ftab-dump
```

To avoid ambiguity when multiple board build directories exist, pass the file path explicitly:

```bash
sdk.py ftab-dump --path build_<board_name>/ftab.bin
```

The default output is a table view. JSON output is also available for scripts:

```bash
sdk.py ftab-dump --path build_<board_name>/ftab.bin --format json
```

Options:

| Option | Description |
|--------|-------------|
| `-p, --path <path>` | Path to `ftab.bin`. If omitted, `sdk.py` searches for `build_*/ftab.bin` in the current project directory. |
| `-f, --format <format>` | Output format: `table` or `json`. The default is `table`; use `json` for machine-readable output. |

The table output contains three sections:

| Section | Description |
|---------|-------------|
| Partition Table Entries | Flash Table partition entries, including storage base address, partition size, XIP or execution base address, and flags. |
| Image Description Table | Image metadata used by DFU and boot flows, including image length, block size, flags such as `AUTO` or `COMPRESS`, and whether the entry is valid. |
| Image Index Table | Running image pointers for each core image slot, such as HCPU, LCPU, bootloader, and boot image. |

```{toctree}
:hidden:

partition_table_v1.md
partition_table_v2.md
```
