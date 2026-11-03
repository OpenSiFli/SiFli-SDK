# crash-dump 命令行工具

`crash-dump` 是 sdk.py 中用于保存、打包和分析死机现场的命令组，适合没有 Trace32/AssertDump 图形界面的环境（例如 CI、Linux/macOS 开发机、远程调试）。本工具抓取的现场仍可直接交给 Trace32/AssertDump 打开，也支持在 PC 上离线分析。

```bash
sdk.py crash-dump --help
sdk.py crash-dump <command> --help
```

## 1. 命令概览

| 命令 | 作用 | 输入 | 主要输出 |
| --- | --- | --- | --- |
| `capture-live` | 通过 J-Link 或 UART DEBUG IP 直接从运行中的芯片读内存 | 芯片内存、`.jlink` 脚本 | 各 region 的 `*.bin` + 元数据（目录本身即崩溃包） |
| `export-coredump` | 读取 flash 上的 coredump 分区，或打包现成的 coredump 文件 | coredump 分区 / `coredump.bin` | 解析后的崩溃包 |
| `readcore` | 把崩溃包转换成 GDB 可用的 core ELF | 崩溃包 + 程序 ELF | `coredump.elf` |
| `analyze` | 离线分析崩溃包或 core ELF | 崩溃包 / core ELF + 程序 ELF | `analysis.json` |

典型流程：

```text
capture-live ─┐
              ├─> 崩溃包 ─> readcore ─> coredump.elf ─> analyze ─> analysis.json
export-coredump ┘                └─────────> analyze（直接分析包）
```

`capture-live` 和 `export-coredump` 产出的崩溃包都可以直接交给 `analyze`；`readcore` 只在需要用 GDB 或第三方工具打开现场时才需要执行。

本文中的几个术语：

- **崩溃包**：包含现场内存和元数据的目录，是本工具链各命令之间的交换格式；
- **region**：`.jlink` 脚本中一条 `savebin` 对应的内存块（如 `hcpu_ram.bin`）；
- **AssertDump / Trace32**：PC 端查看现场的图形工具，可以直接打开崩溃包目录；
- **UART DEBUG IP**：SiFli 芯片内置的串口调试协议，`--transport uart` 通过它读写内存；
- **saved_stack_frame**：固件在异常处理中保存的寄存器帧，用于还原异常发生瞬间的现场。

## 2. 崩溃包结构

两种抓取命令的输出目录都是崩溃包，常见内容如下：

| 文件/目录 | 说明 |
| --- | --- |
| `*.bin` | 各 region 的现场内存 |
| `sdk_manifest.json` | 完整 SDK 元数据，包含 `segments`、`current_registers`、`legacy` 和附件信息；工具优先读取它 |
| `manifest.json` | AssertDump 兼容元数据（`items`/`files`）；`sdk.py` 在生成兼容目录时会重写该文件 |
| `attachments/` | `--elf`、`--log` 的副本 |
| `hcpu.axf`、`log.txt` | AssertDump/Trace32 使用的程序镜像和日志（`--elf` 的副本；`log.txt` 在 `capture-live --transport jlink` 时来自 `capture.log`，在 `export-coredump` 时来自 `--log`） |
| `coredump.bin`、`segments/`、`ramlogs/` | 只有 `export-coredump` 才会产生 |
| `capture.jlink`、`capture.log` | 只有 `capture-live --transport jlink` 才会产生 |
| `coredump.elf` | `readcore` 生成的 GDB core 文件 |

> 注意：`manifest.json` 和 `sdk_manifest.json` 的职责不同。完整元数据在 `sdk_manifest.json`；`validate_package()` 优先读它，只有不存在时才回退到 `manifest.json`（例如未生成兼容目录的包）。

## 3. 前置条件

- 已执行 `./export.ps1`（Windows）或 `source ./export.sh`（Linux/macOS），`sdk.py` 可用；该环境已包含 pyserial、pyelftools 以及 GCC 工具链中的 `arm-none-eabi-gdb`，无需额外安装；
- 准备好与死机固件**完全匹配**的 ELF/AXF（`build_<board>/main.elf` 等），版本不一致会导致符号和地址错误；
- `--transport jlink` 需要安装 J-Link 软件（Windows 下为 `JLink.exe`，其他平台为 `JLinkExe`），也可以通过 `--jlink` 指定完整路径；
- 示例使用相对路径，默认**在工程目录**（含 `build_<board>` 的目录）执行；在其他目录执行时请改成绝对路径，`<SDK_ROOT>` 表示 SDK 根目录。

## 4. capture-live

### 4.1 传输方式

| `--transport` | 连接方式 | 依赖 | 芯片支持 | 当前寄存器来源 |
| --- | --- | --- | --- | --- |
| `jlink`（默认） | J-Link 仿真器（USB），或 SifliUsartServer 的 IP 服务 | J-Link 软件 | 全部系列 | J-Link 日志 |
| `uart` | 直连串口，使用 SiFli UART DEBUG IP 协议 | pyserial | 仅 SF32LB52X / SF32LB56X | UART-DAP 寄存器读取 |

`uart` 方式不需要 J-Link 硬件、AssertDump.exe 或 `uart_server`，适合纯 USB 串口环境（Windows/macOS/Linux）；但它仍会使用 `tools/AssertDump/` 下的 `.jlink` 脚本定义要抓取的 region。

### 4.2 参数

| 参数 | 说明 |
| --- | --- |
| `--transport` | `jlink`（默认）或 `uart` |
| `--chip` | 必填，芯片系列，如 `SF32LB52`、`SF32LB56X`；即使指定了 `--script` 也仍然必填 |
| `--output` | 必填，输出目录；建议使用空目录，或先清理旧文件 |
| `--probe` | `uart` 必填，串口：`/dev/ttyUSB0`、`/dev/cu.*`、`COM5`，或 probe-rs 选择器 `VID:PID-index:COM5` |
| `--jlink` | J-Link 可执行文件路径，默认按平台选择 |
| `--jlink-ip` | 通过 IP 连接 J-Link/usart_server，如 `127.0.0.1:19025` |
| `--core` | `hcpu` 或 `lcpu`；按 region 文件名过滤，并移除脚本中的切核命令 |
| `--script` | 自定义 `.jlink` 脚本，覆盖自动选择的抓取范围 |
| `--chip-model` | 具体料号（如 `LB525`、`SF32LB57BUN3N6`），用于查 `tools/AssertDump/AssertDump.ini` 选择脚本；不能与 `--psram-size` 同时使用 |
| `--include-psram` | 使用料号对应的 PSRAM 脚本；必须与 `--chip-model` 一起使用，单独使用会报错 |
| `--psram-size` | 按容量选择 PSRAM 脚本，如 `8MB`；不能与 `--chip-model` 同时使用 |
| `--elf` | 程序 ELF，会复制到包内 `attachments/`，并用于离线分析 |
| `--log` | 串口/调试日志，会复制到包内，供 `analyze` 回退解析寄存器 |
| `--json` | 输出 manifest JSON 而不是输出目录路径；进度信息仍输出到 stderr |

> `--probe-rs` 选项当前仅为兼容保留，未使用；指定非默认值时会在 stderr 输出提示。

`.jlink` 脚本的选择优先级：

1. `--script` 指定的脚本；
2. `--chip-model` + `--include-psram`：使用 `AssertDump.ini` 中该料号映射的脚本；
3. `--chip-model`（无 `--include-psram`）：使用 `<chip>_psramless.jlink`；
4. `--psram-size`：使用 `<chip>_psram_<size>.jlink`；
5. 以上参数都不提供：使用 `<chip>_psramless.jlink`。

以上优先级只适用于能通过参数校验的组合：只给 `--include-psram`（没有 `--chip-model`）会在选脚本前报错；`--script` 也不能与 `--chip-model`、`--psram-size`、`--include-psram` 叠加。

`--chip-model` 与 `--psram-size` 不能同时使用（会直接报错）。需要手动指定容量时，请去掉 `--chip-model`。

> **58X 说明**：默认使用 `sf32lb58x_psramless.jlink`，只抓 RAM；需要 PSRAM 时使用 `--chip-model <料号> --include-psram`。58X 的 PSRAM 是双 bank 布局（脚本名为 `sf32lb58x_psram_<size>_<size>.jlink`），`--psram-size` 只适用于单容量脚本，在 58X 上会报 `AssertDump script not found`，请改用料号或 `--script`。

### 4.3 示例

UART（Windows）：

```bash
sdk.py crash-dump capture-live \
  --transport uart \
  --probe COM5 \
  --chip SF32LB52 \
  --chip-model LB525 \
  --include-psram \
  --output ./live-crash \
  --elf build_sf32lb52-lcd_n16r8_hcpu/main.elf
```

UART（Linux）：

```bash
sdk.py crash-dump capture-live \
  --transport uart \
  --probe /dev/ttyUSB0 \
  --chip SF32LB52 \
  --chip-model LB525 \
  --output ./live-crash \
  --elf build_sf32lb52-lcd_n16r8_hcpu/main.elf
```

J-Link USB：

```bash
sdk.py crash-dump capture-live \
  --transport jlink \
  --chip SF32LB57 \
  --chip-model SF32LB575UCNN6 \
  --core hcpu \
  --output ./live-crash \
  --elf build_dpi-hdk_lb575ucnn6_a128_qspi_hcpu/main.elf
```

J-Link over IP（SifliUsartServer）：

```bash
sdk.py crash-dump capture-live \
  --transport jlink \
  --jlink-ip 127.0.0.1:19025 \
  --chip SF32LB52 \
  --output ./live-crash \
  --elf build_sf32lb52-lcd_n16r8_hcpu/main.elf
```

J-Link（58X，抓 PSRAM 时需带料号和 `--include-psram`）：

```bash
sdk.py crash-dump capture-live \
  --transport jlink \
  --chip SF32LB58 \
  --chip-model LB583 \
  --include-psram \
  --output ./live-crash \
  --elf build_ec-lb583_hcpu/main.elf
```

58X 也可以显式指定脚本：

```bash
sdk.py crash-dump capture-live \
  --transport jlink \
  --chip SF32LB58 \
  --script <SDK_ROOT>/tools/AssertDump/sf32lb58x_psram_8MB_8MB.jlink \
  --output ./live-crash \
  --elf build_ec-lb583_hcpu/main.elf
```

### 4.4 输出

- `.jlink` 脚本中每个 `savebin` 条目对应的 `*.bin`（如 `hcpu_ram.bin`、`psram.bin`）；
- `sdk_manifest.json`：完整 SDK 元数据，包含 `segments`、`current_registers`、`current_register_source`、`warnings`；
- `manifest.json`：AssertDump 兼容元数据；
- `hcpu.axf`：`--elf` 的副本（传入时才有）；
- `capture.jlink`、`capture.log`：`jlink` 方式生成的脚本和 J-Link 日志（`capture.log` 会再复制为 `log.txt`）；
- 输出目录可直接作为 Trace32/AssertDump 的现场目录使用。

该输出目录同时就是一个崩溃包，可以直接作为 `readcore --package` 和 `analyze --package` 的输入。

## 5. export-coredump

用于固件已经把死机现场写入 coredump 分区的情况，或把已有的 `coredump.bin` 转成崩溃包。

### 5.1 启用 coredump 分区

**固件侧**（`middleware/coredump/Kconfig`，可在 menuconfig 中配置）：

- 打开 `Core Dump for Offline Debugging`（`USING_COREDUMP=y`）；
- Backend 选择 `Partition Backend`；
- 分区名默认为 `coredump`；
- 可选打开 `Enable minidump`，它需要另一个独立分区（默认名 `mindump`）。

**分区表侧**：在 ptab 对应区域上打标签：

```json
{
  "offset": "0x00BA0000",
  "max_size": "0x00100000",
  "tags": ["COREDUMP_REGION"]
},
{
  "offset": "0x00B20000",
  "max_size": "0x00080000",
  "tags": ["COREDUMP_MIN_REGION"]
}
```

`--ptab` 只接受旧版 **JSON 列表**格式（`base` + `offset` + `max_size` + `tags`），不支持 `ptab.yaml`。可用的完整模板见 `example/system/coredump/project/sf32lb52-lcd_n16r8_hcpu/ptab.json`：full 区域实际为 `0x12BA0000` / `0x100000`，minimum 为 `0x12B20000` / `0x80000`。

**FAL 分区表**：固件通过 `fal_partition_find()` 按名字查找分区，所以还需要在工程的 `custom_mem_map.h`（或板级同名文件）里把分区名映射到上面两个区域（flash 设备名按实际配置调整）：

```c
#define FAL_PART_TABLE \
{ \
    {FAL_PART_MAGIC_WORD, "mindump",  NOR_FLASH2_DEV_NAME, COREDUMP_MIN_REGION_OFFSET, COREDUMP_MIN_REGION_SIZE, 0}, \
    {FAL_PART_MAGIC_WORD, "coredump", NOR_FLASH2_DEV_NAME, COREDUMP_REGION_OFFSET,     COREDUMP_REGION_SIZE,     0}, \
}
```

分区名需要与 menuconfig 中的 `COREDUMP_PARTITION_NAME` / `COREDUMP_MINIDUMP_PART_NAME` 一致。完整示例见 `example/system/coredump/project/sf32lb52-lcd_n16r8_hcpu/custom_mem_map.h`。

> SDK 自带的 `customer/boards/**/ptab.json` 默认**不带**这些标签。直接复用会报 `PTAB tag not found: COREDUMP_REGION`；此时需要先按上面的方式加标签，或改用 `--address/--size`。

### 5.2 两种输入方式

| 输入 | 需要的参数 | 说明 |
| --- | --- | --- |
| flash 上的 coredump 分区 | `--chip` + `--ptab`，或 `--chip` + `--address`/`--size` | 通过 `sftool` 或 J-Link 读取 |
| 现成的 coredump 文件 | `--input <file>` | 芯片型号和时间戳从文件头解析；`--transport`、`--port`、`--address`、`--size`、`--ptab` 等读取参数被忽略 |

coredump 文件头格式为 `<52X|55X|56X|57X|58X>_<时间戳>`（24 字节）。文件头不匹配时会报 `invalid coredump header`。

`--kind` 用于选择分区表中的区域，且只在 `--ptab` 模式下生效：

- `full`（默认）：查找 `COREDUMP_REGION` 标签；
- `minimum`：查找 `COREDUMP_MIN_REGION` 标签。

### 5.3 参数

| 参数 | 说明 |
| --- | --- |
| `--output` | 必填，输出目录；建议使用空目录，或先清理旧文件 |
| `--input` | 已有的 coredump 文件 |
| `--transport` | `uart`（默认，走 `sftool read_flash`）或 `jlink`（走 `savebin`） |
| `--chip` | 芯片系列；`--input` 模式下不需要 |
| `--ptab` | 旧版 ptab.json（JSON 列表格式），用于自动查找 coredump 区域 |
| `--address`、`--size` | 直接指定 coredump 区域，与 `--ptab` 二选一 |
| `--kind` | `full`（默认）或 `minimum`，仅与 `--ptab` 配合使用 |
| `--memory` | `sftool` 的存储器类型，默认 `nor` |
| `--port` | `uart` 必填，串口名；Windows 下可直接写 `COM5` |
| `--baud` | 串口波特率，默认 `1000000` |
| `--sftool` | `sftool` 可执行文件路径，默认 `sftool` |
| `--jlink` | J-Link 可执行文件路径 |
| `--script`、`--psram-size`、`--chip-model`、`--include-psram` | 仅用于重建 AssertDump 兼容目录时的 region 划分（选择规则同 §4.2；58X 需料号 + `--include-psram` 才能覆盖 PSRAM region） |
| `--elf`、`--log` | 附带程序 ELF 和日志 |
| `--json` | 输出 manifest JSON |

### 5.4 示例

按分区表读取（UART + sftool）：

```bash
sdk.py crash-dump export-coredump \
  --transport uart \
  --chip SF32LB52 \
  --ptab <SDK_ROOT>/example/system/coredump/project/sf32lb52-lcd_n16r8_hcpu/ptab.json \
  --port COM5 \
  --output ./coredump-pkg
```

按地址/大小读取（J-Link，地址取自上面的模板）：

```bash
sdk.py crash-dump export-coredump \
  --transport jlink \
  --chip SF32LB52 \
  --address 0x12BA0000 \
  --size 0x100000 \
  --output ./coredump-pkg
```

读取 minidump（只在 `--ptab` 模式下）：

```bash
sdk.py crash-dump export-coredump \
  --chip SF32LB52 \
  --ptab <SDK_ROOT>/example/system/coredump/project/sf32lb52-lcd_n16r8_hcpu/ptab.json \
  --kind minimum \
  --port COM5 \
  --output ./coredump-pkg
```

打包现成文件：

```bash
sdk.py crash-dump export-coredump \
  --input ./coredump.bin \
  --output ./coredump-pkg \
  --elf build_sf32lb52-lcd_n16r8_hcpu/main.elf
```

### 5.5 输出

- `coredump.bin`：原始 coredump；
- `segments/<序号>-0x<地址>.bin`：按 coredump 块拆出的内存段；
- `ramlogs/minimum.bin`、`ramlogs/remaining.bin`：固件 RAM log（如果 dump 中包含）；
- `sdk_manifest.json`：包含 `raw`、`segments`、`ramlogs`、`chip`、`warnings` 的完整元数据；
- `manifest.json`：AssertDump 兼容元数据；
- AssertDump 兼容目录：按 `.jlink` 脚本的 region 从 `segments` 中切片重建的 `*.bin`、`sdk_manifest.json`，以及传入 `--elf`/`--log` 时的 `hcpu.axf`、`log.txt`；coredump 未覆盖的 region 会记录在 `sdk_manifest.json` 的 `legacy.missing` 中。

## 6. readcore

把崩溃包转换成 GDB 可用的 core ELF。

```bash
sdk.py crash-dump readcore \
  --package ./live-crash \
  --elf build_sf32lb52-lcd_n16r8_hcpu/main.elf \
  --output ./live-crash/coredump.elf
```

| 参数 | 说明 |
| --- | --- |
| `--package` | 必填，`capture-live` 或 `export-coredump` 的输出目录 |
| `--elf` | 程序 ELF；如果包内已有附件 ELF 可省略；包内、外同时存在时会校验 sha256 |
| `--output` | 输出路径，默认 `<package>/coredump.elf` |

生成的 ELF 会为每个内存段建立 `PT_LOAD`，并携带寄存器（`PT_NOTE`）。寄存器来源依次为：包内 `sdk_manifest.json` 的 `registers`（`capture-live` 时即当前寄存器）、包内 J-Link 日志、`saved_stack_frame` 符号。

```bash
arm-none-eabi-gdb build_sf32lb52-lcd_n16r8_hcpu/main.elf \
  -ex "core-file ./live-crash/coredump.elf" \
  -ex "bt"
```

## 7. analyze

```bash
sdk.py crash-dump analyze --package ./live-crash --engine python
```

```bash
sdk.py crash-dump analyze \
  --core ./live-crash/coredump.elf \
  --elf build_sf32lb52-lcd_n16r8_hcpu/main.elf
```

| 参数 | 说明 |
| --- | --- |
| `--package` | 崩溃包目录 |
| `--core` | core ELF；必须同时提供 `--elf`；该模式固定使用 GDB，`--engine` 会被忽略 |
| `--elf` | 程序 ELF；`--package` 模式下包内已有附件时可省略 |
| `--engine` | `python`（默认，只依赖 pyelftools）、`gdb`（只依赖 GDB）、`auto`（先试 GDB，失败回退 Python 并合并部分结果） |
| `--output` | 输出路径，默认为包目录（或 core ELF 所在目录）下的 `analysis.json` |
| `--json` | 把分析结果 JSON 打印到 stdout；仍会写出分析文件 |

引擎行为细节：

- `python`：只用 pyelftools 和崩溃包数据；
- `gdb`：`--package` 模式下若没有 `coredump.elf`，会先自动调用 `readcore` 生成；
- `auto`：只有包内已存在 `coredump.elf` 时才尝试 GDB，否则直接使用 Python；GDB 部分失败时会回退到 Python 并保留 `gdb_partial`。

分析结果中的寄存器分为两类，含义不同：

- `current_registers` / `current_register_source`：抓现场时调试器读到的**当前** CPU 寄存器，适合判断芯片停在何处；
- `saved_registers` / `saved_register_source`：从固件 `saved_stack_frame` 解码出的**异常发生瞬间**寄存器；当 `saved_stack_pointer`、`error_reason` 明显异常时，应优先参考当前寄存器。

此外结果中还会包含 `pc`/`lr` 的符号、`scb`、`error_reason`、`stack_pointer` 和 `stack_candidates`。

## 8. 常见问题

| 现象 | 原因与处理 |
| --- | --- |
| `--elf is required when the package has no attached ELF` | 抓现场时没有传 `--elf`，包内也没有附件；给 `readcore`/`analyze` 补上 `--elf` |
| `ELF sha256 does not match the crash package` | 提供的 ELF 与抓现场时使用的固件不一致；`readcore` 和 `analyze --package` 都会校验 |
| `Invalid value for '--elf': Path ... does not exist` | 相对路径按当前目录解析；请在工程目录执行或改用绝对路径 |
| `--probe is required for --transport uart` | `uart` 方式必须指定串口 |
| `Cannot extract serial port from probe spec` | `--probe` 不是 `/dev/...` 或 `COMx` 形式 |
| `UART live capture only supports SF32LB52X and SF32LB56X` | `uart` 抓现场只支持 52X/56X；其他芯片请用 `--transport jlink` |
| `AssertDump script not found` | 缺少对应的 `.jlink` 脚本，检查 `tools/AssertDump/`；58X 的 PSRAM 是双 bank 布局，需配合 `--chip-model` + `--include-psram`，或用 `--script` |
| `AssertDump chip model not found` | `--chip-model` 不在 `AssertDump.ini` 中 |
| `JLinkExe`/`JLink.exe` 找不到 | 安装 J-Link 软件，或用 `--jlink` 指定路径 |
| `arm-none-eabi-gdb was not found` | 未 export GCC 工具链环境，或改用 `--engine python` |
| `--chip and either --ptab or both --address/--size are required` | 分区读取模式缺少芯片型号或区域信息 |
| `--port is required for UART coredump export` | `export-coredump --transport uart` 必须指定串口 |
| `--chip-model and --psram-size are mutually exclusive` | 手动指定容量时去掉 `--chip-model` |
| `--include-psram requires --chip-model` | `--include-psram` 只能与料号一起使用 |
| `--script cannot be combined with --chip-model, --psram-size, or --include-psram` | 自定义脚本已定义抓取范围，不能再叠加脚本选择项 |
| `legacy PTAB must be a JSON list` | `--ptab` 只接受旧版 JSON 列表格式，不支持 `ptab.yaml` |
| `PTAB tag not found: COREDUMP_REGION` | 分区表没有打 coredump 标签；参考 §5.1 添加，或改用 `--address/--size` |
| `invalid coredump header` | `--input` 文件不是固件生成的 coredump，或文件头损坏 |
| manifest 中出现 `missing output file` 警告 | `.jlink` 脚本的 region 与实际读到的文件不匹配，例如 PSRAM 容量选错 |

## 9. 相关文档

- [死机分析指南](crash_analysis.md)：Trace32/AssertDump 保存与恢复现场的完整流程；
- [dump 死机调试指南](../app_development/debuging/dump_assert_debug.md)：Trace32 解析现场的操作步骤；
- `example/system/coredump`：coredump 固件侧功能与分区模式示例；
- `tools/AssertDump/AssertDump.ini`：料号与 `.jlink` 脚本的映射表。
