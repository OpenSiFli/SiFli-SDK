# 分区表语法3.0版本

本文详细介绍 SiFli SDK 中 分区表v3(即ptab v3)格式的设计、使用方法和迁移指南。

## 概述

ptab v3 是 SiFli SDK 中新一代分区表格式，采用 YAML 格式，相比 v1/v2 的 JSON 格式具有以下优点：

- **更清晰的语义**：使用 `type`/`subtype` 替代复杂的 `tags` 机制
- **抽象的存储层**：通过 `region` 字段引用逻辑存储区域，与芯片拓扑解耦
- **统一的执行地址**：通过 `exec: {region, offset}` 明确指定执行地址/搬运落点
- **自动宏生成**：从 `name` 字段自动生成 C 宏定义
- **更好的验证**：内置验证工具检查分区配置正确性

和 patb v1/v2 不同的是，ptab v3 与构建系统紧密结合，直接生成`ftab.bin`、`ptab.h`，以及使用基于 jinja2 的链接脚本片段等。

## 文件格式

### 基本结构

下面的 YAML 示例是“完整展开形式”，用于说明每一种分区的语义。实际项目里的 `ptab.yaml` 可以省略能够由构建系统稳定推导出的默认分区；只有当布局偏离默认值时，才需要把这些分区显式写出来。

```yaml
version: 3
chip: SF32LB525UC6

memory:
  - mpi: mpi2
    type: nand
    size: 16777216 # 16MB

partitions:
  # Flash Table (ftab)
  - name: flash_table
    type: ftab
    region: mpi2
    offset: 0
    size: 32KB

  # Calibration data
  - name: calibration
    type: data
    subtype: calibration
    region: mpi2
    offset: 0x8000
    size: 8KB

  # Bootloader （存储在 Flash，执行在 RAM)
  - name: bootloader
    type: bootloader
    region: mpi2
    offset: 0x80000
    size: 64KB
    exec:
      region: hpsys_ram
      offset: 0x20000
    core: HCPU

  # Main application
  - name: hcpu_flash_code
    type: app
    subtype: factory
    region: mpi2
    offset: 0xA0000
    size: 4MB
    exec:
      region: mpi1
      offset: 0
    core: HCPU

  # DFU application
  - name: dfu_flash_code
    type: app
    subtype: dfu
    region: mpi2
    offset: 0x4A0000
    size: 512KB
    exec:
      region: mpi1
      offset: 0
    core: HCPU

  # DFU info region
  - name: dfu_info_region
    type: data
    subtype: raw
    region: mpi2
    offset: 0x520000
    size: 128KB

  # DFU download region
  - name: dfu_download_region
    type: data
    subtype: raw
    region: mpi2
    offset: 0x780000
    size: 1152KB

  # Filesystem region
  - name: fs_region
    type: data
    subtype: filesystem
    region: mpi2
    offset: 0x8A0000
    size: 4MB

  # KVDB for DFU
  - name: dfu
    type: data
    subtype: flashdb_kv
    region: mpi2
    offset: 0xCA0000
    size: 16KB

  # KVDB for BLE
  - name: ble
    type: data
    subtype: flashdb_kv
    region: mpi2
    offset: 0xCA4000
    size: 16KB

  # PSRAM data region
  - name: psram_data
    type: data
    subtype: ram
    region: mpi1
    offset: 0x400000
    size: 4MB

  # HCPU RAM data (shared with bootloader exec)
  - name: hcpu_ram_data
    type: data
    subtype: ram
    region: hpsys_ram
    offset: 0
    size: 128KB

  # Bootloader RAM data
  - name: bootloader_ram_data
    type: data
    subtype: ram
    region: hpsys_ram
    offset: 0x40000
    size: 64KB

  # LPSYS RAM
  - name: lpsys_ram
    type: data
    subtype: ram
    region: lpsys_ram
    offset: 0
    size: 24KB
```

### 顶层字段

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `version` | int | 是 | 固定为 `3` |
| `chip` | string | 是 | 芯片型号，如 `SF32LB525UC6`、`SF32LB52JUD6`等 |
| `memory` | list | 否 | 存储区域定义列表 |
| `partitions` | list | 是 | 分区列表 |

## memory 定义

memory 用于定义外挂的存储器的熟悉，如 NAND Flash、NOR Flash 等。

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `name` | string | 是 | MPI 接口或者 SCMMC 接口名称，如 `mpi2`、`sdmmc1`等 |
| `type` | string | 是 | 存储器类型，可选为 `nand`、`nor`、`sd` |
| `size` | string | 是 | 存储器大小，支持 `0x` 前缀和 `KB`/`MB` 后缀 |

如果是外挂的 SDMMC 存储器（eMMC/SD 卡），还需要在 `memory` 中定义对应的存储器，并且在分区中通过 `region` 字段引用。

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `sdmmc` | string | 是 | SDMMC 存储器名称，如 `sdmmc1` |
| `size` | string | 是 | 存储器大小，支持 `0x` 前缀和 `KB`/`MB` 后缀 |

### 分区字段

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `name` | string | 是 | 分区名称，用于生成 C 宏 |
| `type` | string | 是 | 分区类型 |
| `subtype` | string | 否 | 分区子类型 |
| `region` | string | 是 | 存储区域（逻辑名称） |
| `offset` | int/string | 是 | 区域内偏移，支持 `0x` 前缀和 `KB`/`MB` 后缀 |
| `size` | int/string | 是 | 分区大小 |
| `exec` | dict | 否 | 执行地址定义 |
| `core` | string | 否 | 运行核心：`HCPU` 或 `LCPU` |
| `attrs` | dict | 否 | 自定义属性 |

#### exec 字段

exec 字段用于指定代码不能在 XiP 的时候实际的执行地址（搬运到 psram 或者 sram 中执行）。

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `region` | string | 是 | 执行区域（逻辑名称） |
| `offset` | int/string | 是 | 执行区域内偏移，支持 `0x` 前缀和 `KB`/`MB` 后缀 |

例如，假设我们外挂了一个 NAND Flash，并且主程序存储在 NAND Flash 中，但由于 NAND Flash 不能 XiP，所以我们需要把主程序搬运到 PSRAM 中执行，那么就可以这样定义：

```yaml
- name: hcpu_flash_code
  type: app
  subtype: factory
  region: mpi2
  offset: 0xA0000
  size: 4MB
  exec:
    region: mpi1
    offset: 0
  core: HCPU
```

## 分区类型定义

### type 字段

| 值 | 说明 |
|------|------|
| `ftab` | Flash Table（分区表自身） |
| `bootloader` | 引导加载程序 |
| `app` | 应用程序 |
| `data` | 数据分区 |

### subtype 字段

**当 type=app 时：**

| 值 | 说明 |
|------|------|
| `factory` | 出厂主程序 |
| `dfu` | DFU 升级程序，需要注意的是 dfu 也是一个独立运行的程序 |

**当 type=data 时：**

| 值 | 说明 |
|------|------|
| `flashdb_kv` | FlashDB KV 数据库（自动生成 `KVDB_*` 宏，并参与 `FAL_PART_TABLE`） |
| `filesystem` | 文件系统（生成 `FS_REGION_*` 宏，并参与 `FAL_PART_TABLE`） |
| `littlefs` | LittleFS 文件系统，暂时无实际作用 |
| `fat` / `fatfs` | FAT 文件系统，暂时无实际作用 |
| `ram` | RAM 数据区，对于 hpsys_ram 和 lpsys_ram 来说，没有额外改动的情况下可以省略 |
| `calibration` | 校准数据 |
| `raw` | 原始数据区，用于存放由应用程序自行分配的区域 |
| `int_res` | 内置资源，与 raw 不同的是，int_res 是会被链接到最终的 elf 中的，然后我们再通过段名进行切割 |

## 默认分区推导

ptab v3 在解析阶段会自动补齐一部分“系统默认分区”。这意味着板级 `ptab.yaml` 不需要把所有默认布局都重复写一遍，只需要保留和默认值不同的分区。

- `SF32LB52`
  - 当启动存储位于 NOR / NAND / SDMMC 上时，`flash_table` 和 `bootloader` 会根据 HCPU factory app 的存储 `region` 自动推导。
  - 当启动存储位于 NAND 上时，会自动保留 `factory_data` 分区，别名为 `FACTORY_DATA`，默认 `offset = 2 * block_size`、`size = block_size`。未显式配置 `memory.block_size` 时，NAND block size 默认为 128KB；显式配置 256KB block size 时，`FACTORY_DATA` 默认偏移和大小也会同步调整。
  - 当启动存储位于 SDMMC 上时，会自动保留 `mbr` 分区（别名 `MBR`，`offset=0x0`，`size=0x1000`）和 `factory_data` 分区（别名 `FACTORY_DATA`，`offset=0x41000`，`size=0x20000`）。
  - 在保持默认 RAM 布局的情况下，`hcpu_ram_data`、`bootloader_ram_data`、`lpsys_ram` 可以省略。
- `SF32LB56` / `SF32LB58`
  - 位于内部 `mpi5` 上的 `flash_table` 和 `bootloader` 可以省略。
- `psram_data` / `psram_data2`
  - 片内 SiP PSRAM 的数据窗口会根据 SiliconSchema 与 `exec` 预留区自动生成，不需要在板级 YAML 中重复描述。

需要注意：

- 默认推导只会在分区缺失时生效；显式写出来的分区仍然优先生效。
- 如果某个工程对 RAM 布局做了定制，例如显式缩小 `hcpu_ram_data` 以匹配特定链接脚本，那么该分区不能删除。
- 工程级 `ptab.overlay.yaml` 仍然可以覆盖这些默认分区；即使板级 YAML 中省略了它们，overlay 也会先看到补齐后的默认分区再执行 `override`。

## 存储区域 (region)

### 支持的区域名称

| 区域 | 说明 |
|------|------|
| `外置存储` | 通过 `memory` 定义的外挂存储区域，如 `mpi1`、`mpi2`、`sdmmc1` 等 |
| `hpsys_ram` | HPSYS RAM |
| `lpsys_ram` | LPSYS RAM |

### 地址解析

每个 region 对应两个地址：

- **SBUS 地址（存储地址）**：用于 DMA 访问、下载烧录
- **CBUS 地址（XIP 地址）**：用于 CPU 直接执行

地址映射从 `tools/SiliconSchema/common/mpi/<chip>/mpi.yaml` 加载。

## 宏生成规则

### 基本宏

每个分区自动生成以下宏（`<NAME>` 为大写的 `name` 字段）：

```c
#define <NAME>_START_ADDR    (0x...)  // SBUS 地址
#define <NAME>_SIZE          (0x...)  // 分区大小
#define <NAME>_OFFSET        (0x...)  // 区域内偏移
```

当分区所在 region 的存储类型可识别时，还会额外生成：

```c
#define <NAME>_MEM_TYPE      HAL_MEM_TYPE_...
```

目前可生成的内存类型常量为：

- `HAL_MEM_TYPE_NOR_FLASH`
- `HAL_MEM_TYPE_NAND_FLASH`
- `HAL_MEM_TYPE_SDMMC_STORAGE`
- `HAL_MEM_TYPE_PSRAM`

对于片上 RAM 等没有对应 HAL memory type 的区域，不会生成 `<NAME>_MEM_TYPE`。

### 特殊宏

**CODE_START 分区：**

当 `type=app` 且 `subtype=factory` 时生成：

```c
#define CODE_START_ADDR  (0x...)
#define CODE_SIZE        (0x...)
```

需要注意的是，`CODE_START_ADDR` 对应 `exec` 字段指定的执行地址。如果没有指定 `exec`，则使用存储的 CBUS 地址。

对于 `HCPU_FLASH_CODE`、`FLASH_BOOT_LOADER` 这类由执行地址兼容出来的宏，若同时生成 `_MEM_TYPE`，其取值也跟随最终执行 region，而不是存储 region。若执行 region 为片上 RAM，则不会保留对应的 `_MEM_TYPE` 定义。

**文件系统分区：**

当 `type=data` 且 `subtype` 为 `filesystem`/`littlefs`/`fat`/`fatfs`/`flashdb` 时生成：

```c
#define FS_REGION_START_ADDR  (0x...)
#define FS_REGION_SIZE        (0x...)
#define FS_REGION_OFFSET      (0x...)
#define FS_REGION_MEM_TYPE    HAL_MEM_TYPE_...
```

**FlashDB KV 分区：**

当 `type=data` 且 `subtype=flashdb_kv` 时生成 KVDB 相关宏（其中 `<NAME>` 为分区 `name` 的大写形式；该 `name` 同时也是 FlashDB 数据库名和 FAL 分区名，例如 `dfu`、`ble`）：

```c
#define KVDB_<NAME>_REGION_START_ADDR  (0x...)
#define KVDB_<NAME>_REGION_OFFSET  (0x...)
#define KVDB_<NAME>_REGION_SIZE    (0x...)
#define KVDB_<NAME>_REGION_MEM_TYPE HAL_MEM_TYPE_...
```

**FAL_PART_TABLE：**

`ptab.h` 会自动生成 `FAL_PART_TABLE`，收录以下分区：

- `type=data, subtype=flashdb_kv`
- `type=data, subtype=filesystem`
- `type=app` 且 `subtype != ex`

不会自动收录以下分区：

- `type=ftab`
- `type=bootloader`
- `type=data, subtype=raw/ram/calibration/int_res`
- `type=app, subtype=ex`

FAL 分区名始终使用 partition 的 `name`，不会自动映射成旧工程里的 `main`、`dfu_code`、`fs_root` 等 legacy 名称。

生成示例：

```c
#define FAL_PART_TABLE \
{ \
    {FAL_PART_MAGIC_WORD, "hcpu_flash_code", NOR_FLASH5_DEV_NAME, 0x00020000, 0x00200000, 0}, \
    {FAL_PART_MAGIC_WORD, "dfu_flash_code", NOR_FLASH5_DEV_NAME, 0x00220000, 0x00080000, 0}, \
    {FAL_PART_MAGIC_WORD, "fs_region", NOR_FLASH5_DEV_NAME, 0x00600000, 0x00100000, 0}, \
    {FAL_PART_MAGIC_WORD, "<name>", NOR_FLASHx_DEV_NAME, <offset>, <size>, 0}, \
    ...
}
```

其中设备名根据存储 `region` 确定，而不是根据 `exec` 确定：

- `mpi1..mpi5` → `NOR_FLASH1_DEV_NAME` .. `NOR_FLASH5_DEV_NAME`
- `sdmmc1` → `SDMMC1_DEV_NAME`
- `sdmmc2` → `SDMMC2_DEV_NAME`
## 构建系统集成

### 自动检测

构建系统会自动检测 ptab 格式：

1. 优先查找 `ptab.yaml`（v3 格式）
2. 若不存在则使用 `ptab.json`（v1/v2 格式）

搜索路径优先级：
1. `$BSP_ROOT/<board>/`
2. `$BSP_ROOT/<chip>/`
3. `$BSP_ROOT/`
4. `customer/boards/<board>/`

### 工程级分区 overlay

如果工程只想修改少量分区参数，或者新增少量分区，不建议复制整份板级 `ptab.yaml`。ptab v3 支持工程级 overlay：

- `<project>/<chip>/ptab.overlay.yaml`
- `<project>/<board>/ptab.overlay.yaml`

其中：

- `<chip>` 使用板级 `rtconfig.py` 中的 `CHIP.lower()`，例如 `sf32lb52x`
- `<board>` 使用当前板名全名，带 core 后缀，例如 `sf32lb52-lcd_n16r8_hcpu`
- `board overlay` 优先级高于 `chip overlay`

overlay 文件只允许包含 `partitions` 顶层字段，例如：

```yaml
partitions:
  - op: override
    name: fs_region
    size: 5MB

  - op: add
    name: log_region
    type: data
    subtype: raw
    region: mpi2
    offset: 0x00F00000
    size: 64KB
```

`op` 是**可选字段**，但**建议显式写出**，便于 review 和定位错误。

- `op: override` 表示修改已有分区
- `op: add` 表示新增分区
- 如果省略 `op`：
  - 当 `name` 已存在时，自动按 `override` 处理
  - 当 `name` 不存在时，自动按 `add` 处理

限制如下：

- overlay 只支持“分区级”修改，不支持 `memory`、`chip`、`version`
- overlay 不支持删除分区
- overlay 只能作用于板级 `ptab.yaml`（v3）；不能叠加到 `ptab.json`
- 如果板级当前仍使用 `ptab.json`（v1/v2），且工程目录中没有命中任何完整 `ptab.yaml`，构建系统会忽略 `ptab.overlay.yaml`，输出 warning，并继续按非 overlay PTAB 路径处理
- 如果工程已经提供完整 `ptab.yaml` / `ptab.json`，则不能再同时使用 `ptab.overlay.yaml`

处理 overlay 时，构建系统会用 `rich` 输出重载摘要，并在 build 目录下生成最终生效的 `ptab.effective.yaml` 供检查。

```{tip}
当 overlay 因板级仍为 `ptab.json`（v1/v2）而被忽略时，不会生成 `ptab.effective.yaml`。这类 warning 仅表示当前构建继续沿用原有 v1/v2 PTAB 行为，并不表示 overlay 已生效。
```

### 生成产物

| 产物 | 说明 |
|------|------|
| `ptab.h` | C 头文件，包含所有分区宏定义 |
| `ftab.bin` | 二进制 Flash Table，用于 bootloader |
| `link_copy.lds` / `link_copy.sct` | GCC / Keil 链接脚本 |
| `ptab.effective.yaml` | overlay 实际作用于板级 `ptab.yaml`（v3）时生成的最终 PTAB V3 YAML |

### ptab v3 与 custom_mem_map.h

ptab v3 工程**不需要**在板级目录额外维护 `custom_mem_map.h`。推荐直接使用构建系统生成的 `ptab.h` 作为分区宏来源。

迁移到 ptab v3 时建议按下面顺序处理：

1. 将板级 `ptab.json` 迁移为 `ptab.yaml`。
2. 删除板级 `custom_mem_map.h`（如 `customer/boards/<board>/{hcpu,lcpu,acpu}/custom_mem_map.h`）。
3. 清理对应的 `build_xxx` 构建目录后重新构建。

注意：

- 不要在板级头文件中手工定义 `FAL_PART_TABLE`，该宏由 `ptab.h` 根据 `flashdb_kv` / `filesystem` / `app(subtype != ex)` 分区自动生成。
- 如果板级 `custom_mem_map.h` 未删除，构建系统仍可能继续生成 `build_xxx/custom_mem_map.h`，并出现 `FAL_PART_TABLE redefined` 告警。
- 如果只是删除了板级文件，但历史构建目录中的 `build_xxx/custom_mem_map.h` 仍然残留，也会继续触发旧逻辑。

### app/ex 资源分割产物（ptab v3 + GCC/Keil）

ptab v3 使用 `type: app` + `subtype: ex` 描述资源分区。构建系统会按分区动态生成链接脚本片段：GCC 为每个资源分区分配专用 `MEMORY` 并生成输出段 `.<name>`（全小写），Keil 生成对应的 `LR_<NAME>` / `ER_<NAME>` 区域。两种工具链最终都会归一化为同一套 `output/` 产物命名。

GCC 默认的段收集规则为：

```ld
.<name> : { *(.<NAME>*) } > <NAME>
```

如果分区声明了 `sections`，则会在默认规则之外额外收集这些输入 section 或 `object + section` 选择器。

构建产物会统一输出到各工程 build 目录下的 `output/`：

- 代码镜像（从 ELF 导出并排除所有 `.<resource_name>` 段）：
  - 如果最终只有一个 bin/hex 输出：
    `build_xxx/output/main.bin`、`build_xxx/output/main.hex`
  - 如果最终同时存在代码镜像和一个或多个 `app/ex` 资源镜像：
    `build_xxx/output/main.app.bin`、`build_xxx/output/main.app.hex`
  - 其它工程同理，基名使用工程名，例如：
    `build_xxx/<proj>/output/<proj>.bin`
    或
    `build_xxx/<proj>/output/<proj>.app.bin`
- 资源镜像（每个 `app/ex` 分区一个文件）：
  - 文件名后缀直接使用 `partition.name` 原文，不再转为大写短名
  - 例如：
    `build_xxx/output/main.hcpu_flash2_img.bin`
    `build_xxx/output/main.hcpu_flash2_font.bin`
    以及对应的 `.hex`

如果某个 `app/ex` 分区在 ELF 中没有对应输出段（或段为空），则会跳过生成该分区的 `main.<partition_name>.bin/.hex`（不报错）。

另外，`app/ex` 分区名应避免与代码镜像后缀 `app` 冲突，以避免在大小写不敏感文件系统下发生文件名覆盖，例如不要让某个资源分区命名为 `app`。

### ftab.bin 生成

v3 格式下，`ftab.bin` 直接通过 Python 脚本生成，不再编译 ftab 子工程：

```
Generating build_xxx/ftab.bin ...
Generated ftab.bin: build_xxx/ftab.bin (11280 bytes)
```

### jinja2 链接脚本模板

在 patb v1/v2 中，链接脚本使用编译器的 c 风格预处理器生成。

在 ptab v3 中：

- GCC 链接脚本使用 jinja2 模板，默认模板位于 `drivers/cmsis/<chip>/Templates/gcc/<core>/link.jinja2`
- Keil `.sct` 的 jinja2 模板也需要手工维护，默认模板位于 `drivers/cmsis/<chip>/Templates/arm/<core>/link.sct.jinja2`

需要注意：

- `tools/build/migrate_ptab_to_v3.py` 只负责迁移 `ptab.json -> ptab.yaml`，**不会自动迁移任何链接脚本**
- 如果工程或板级目录有自定义链接脚本，需要手工迁移对应的 `link.jinja2` 或 `link.sct.jinja2`
- ptab v3 必须使用 jinja2 模板，不可以继续依赖 c 风格预处理器生成链接脚本
- ptab v1/v2 也不支持 jinja2 模板

### validate_ptab_v3.py

验证 ptab v3 文件的正确性：

```bash
python tools/build/validate_ptab_v3.py customer/boards/<board>/ptab.yaml
```

验证项目包括：
- 分区名称格式
- 类型/子类型有效性
- 区域名称有效性
- 分区重叠检测
- `bootloader` 分区唯一性

### sdk.py ptab-export

如果需要检查“最终生效”的 PTAB V3 YAML，可以在工程目录执行：

```bash
sdk.py ptab-export --board=<board_name>
```

常用形式：

```bash
sdk.py ptab-export --board=sf32lb52-lcd_n16r8_hcpu
sdk.py ptab-export --board=sf32lb52-lcd_n16r8_hcpu --output /tmp/ptab.effective.yaml
sdk.py ptab-export --strict
```

说明：

- 默认把最终 YAML 输出到标准输出
- `--output` 可将最终 YAML 写入指定文件
- `--strict` 会把 warning 也视为失败
- 该命令与实际构建使用同一套 PTAB 解析和 overlay 合并逻辑

## 常见问题

### Q: 如何添加自定义宏？

使用 `attrs` 字段：

```yaml
- name: my_partition
  type: data
  subtype: flashdb_kv
  region: mpi2
  offset: 0x100000
  size: 64KB
  attrs:
    MY_CUSTOM_MACRO: 0x12345678
```

将生成：
```c
#define MY_CUSTOM_MACRO  (0x12345678)
```

### Q: v1/v2 的 tags 在 v3 中如何映射？

| v1/v2 tags | v3 对应 |
|------------|---------|
| `FLASH_TABLE` | `type: ftab` |
| `FLASH_BOOT_LOADER` | `type: bootloader` |
| `FS_REGION` | `type: data, subtype: filesystem` |
| `KVDB_*` | `type: data, subtype: flashdb_kv` |
| `app_img` | `type: app` |
| `app_exec` | 使用 `exec: {region, offset}`（当不能 XIP 或需要固定执行地址时） |

## 变更历史

| 版本 | 日期 | 说明 |
|------|------|------|
| v3.4 | 2026-04 | `FAL_PART_TABLE` 自动收录 `flashdb_kv`、`filesystem` 和 `app(subtype != ex)` 分区，并补充 `mpi5` / `sdmmc` 映射规则 |
| v3.3 | 2026-03 | 补充默认分区自动推导规则，支持省略默认 `flash_table` / `bootloader` / 部分 RAM 分区 |
| v3.2 | 2026-03 | 新增工程级 `ptab.overlay.yaml`、`sdk.py ptab-export` 和最终 `ptab.effective.yaml` 导出 |
| v3.1 | 2026-03 | 补充 ptab v3 与 `custom_mem_map.h` 的关系、`FAL_PART_TABLE redefined` 告警原因与处理 |
| v3.0 | 2024-12 | 初始版本，YAML 格式，移除 ftab 子工程 |
