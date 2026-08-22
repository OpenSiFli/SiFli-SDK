# Partition Table Syntax Version 3.0

This document describes the design, usage, and migration guide for the partition table v3 (in short ptab v3) format in the SiFli SDK.

## Overview

ptab v3 is the next-generation partition table format in the SiFli SDK and uses YAML. Compared with the v1/v2 JSON formats, it offers several advantages:

- Clearer semantics: uses `type`/`subtype` instead of the more complex `tags` mechanism
- Abstract storage regions: references logical storage regions via the `region` field, decoupling layout from chip topology
- Unified execution address: specifies execution/copy destination clearly via `exec: {region, offset}`
- Automatic macro generation: generates C macro definitions from `name` fields
- Improved validation: built-in validators check partition correctness

Unlike ptab v1/v2, ptab v3 is closely integrated with the build system and directly generates `ftab.bin`, `ptab.h`, and jinja2-based link script fragments.

## File format

### Basic structure

The YAML example below shows the fully expanded form to illustrate the semantics for each partition type. In real projects, `ptab.yaml` can omit partitions that the build system can deterministically infer; only partitions that differ from defaults need to be explicitly declared.

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

  # Bootloader (stored in Flash, executed in RAM)
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

### Top-level fields

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `version` | int | yes | Must be `3` |
| `chip` | string | yes | Chip model, e.g. `SF32LB525UC6`, `SF32LB52JUD6` |
| `memory` | list | no | Storage region definitions |
| `partitions` | list | yes | Partition list |

## Memory definitions

`memory` defines external storage devices such as NAND or NOR.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `name` | string | yes | MPI or SDMMC interface name, e.g. `mpi2`, `sdmmc1` |
| `type` | string | yes | Storage type: `nand`, `nor`, `sd` |
| `size` | string | yes | Size; supports `0x` prefix and `KB`/`MB` suffix |

For SD/MMC (eMMC/SD card) define the storage in `memory` and reference it in partitions via `region`.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `sdmmc` | string | yes | SDMMC storage name, e.g. `sdmmc1` |
| `size` | string | yes | Storage size; supports `0x` prefix and `KB`/`MB` suffix |

### Partition fields

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `name` | string | yes | Partition name, used for generating C macros |
| `type` | string | yes | Partition type |
| `subtype` | string | no | Partition subtype |
| `region` | string | yes | Storage region (logical name) |
| `offset` | int/string | yes | Region offset; supports `0x` and `KB`/`MB` suffixes |
| `size` | int/string | yes | Partition size |
| `exec` | dict | no | Execution address definition |
| `core` | string | no | Running core: `HCPU` or `LCPU` |
| `attrs` | dict | no | Custom attributes |

#### exec field

`exec` specifies where code should be executed from if XIP is not possible (copy-to-execute).

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `region` | string | yes | Execution region (logical name) |
| `offset` | int/string | yes | Execution region offset; supports `0x` and `KB`/`MB` suffixes |

Example: if the main program is stored in NAND and must be copied to PSRAM for execution:

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

## Partition type definitions

### `type` values

| Value | Meaning |
|-------|---------|
| `ftab` | Flash Table (the partition table itself) |
| `bootloader` | Bootloader |
| `app` | Application |
| `data` | Data partition |

### `subtype` values

When `type=app`:

| Value | Meaning |
|-------|---------|
| `factory` | Factory (primary) application |
| `dfu` | DFU upgrade application (runs independently) |

When `type=data`:

| Value | Meaning |
|-------|---------|
| `flashdb_kv` | FlashDB KV database (generates `KVDB_*` macros and participates in `FAL_PART_TABLE`) |
| `filesystem` | Filesystem (generates `FS_REGION_*` macros and participates in `FAL_PART_TABLE`) |
| `littlefs` | LittleFS (for future uses) |
| `fat` / `fatfs` | FAT filesystem (for future uses) |
| `ram` | RAM data area (can be omitted for default `hpsys_ram`/`lpsys_ram`) |
| `calibration` | Calibration data |
| `raw` | Raw data area for application-managed storage |
| `int_res` | Internal resources: linked into the final ELF and sliced by section names |

## Default partition inference

The ptab v3 parser will auto-fill certain system default partitions. This lets board-level `ptab.yaml` omit defaults and only declare differences.

- SF32LB52
  - When boot storage is NOR/NAND/SDMMC, `flash_table` and `bootloader` are inferred from the HCPU factory app storage `region`.
  - When boot storage is NAND, a `factory_data` partition (alias `FACTORY_DATA`) is reserved by default: `offset = 2 * block_size`, `size = block_size`. If `memory.block_size` is not explicitly set, NAND block size defaults to 128KB; if explicitly set to 256KB the defaults follow accordingly.
  - When boot storage is SDMMC, `mbr` (alias `MBR`, `offset=0x0`, `size=0x1000`) and `factory_data` (alias `FACTORY_DATA`, `offset=0x41000`, `size=0x20000`) are reserved.
  - If RAM layout is unchanged, `hcpu_ram_data`, `bootloader_ram_data`, and `lpsys_ram` can be omitted.
- SF32LB56 / SF32LB58
  - `flash_table` and `bootloader` located on internal `mpi5` can be omitted.
- psram_data / psram_data2
  - On-chip SiP PSRAM windows are generated automatically from the SiliconSchema and `exec` reserved regions and need not be declared in board YAML.

Notes:

- Defaults only apply when a partition is missing; explicitly declared partitions always take precedence.
- If a project customizes RAM layout (e.g. shrinks `hcpu_ram_data` to match a linker script), that partition must be kept.
- Project-level `ptab.overlay.yaml` can still override defaults; overlay sees the filled-default set before applying overrides.

## Regions (`region`)

### Supported region names

| Region | Description |
|--------|-------------|
| External storage | Defined in `memory`, e.g. `mpi1`, `mpi2`, `sdmmc1` |
| `hpsys_ram` | HPSYS RAM |
| `lpsys_ram` | LPSYS RAM |

### Address resolution

Each region maps to two addresses:

- SBUS address (storage address): used for DMA and flashing
- CBUS address (XIP address): used for CPU execution

Address mapping comes from `tools/SiliconSchema/common/mpi/<chip>/mpi.yaml`.

## Macro generation rules

### Basic macros

Each partition automatically generates the following macros (where `<NAME>` is the partition name uppercased):

```c
#define <NAME>_START_ADDR    (0x...)  // SBUS address
#define <NAME>_SIZE          (0x...)  // partition size
#define <NAME>_OFFSET        (0x...)  // region offset
```

If the partition region maps to a known storage type, an additional macro is generated:

```c
#define <NAME>_MEM_TYPE      HAL_MEM_TYPE_...
```

Possible memory type constants:

- `HAL_MEM_TYPE_NOR_FLASH`
- `HAL_MEM_TYPE_NAND_FLASH`
- `HAL_MEM_TYPE_SDMMC_STORAGE`
- `HAL_MEM_TYPE_PSRAM`

For on-chip RAM regions that have no HAL memory type, `<NAME>_MEM_TYPE` is omitted.

### Special macros

CODE_START partitions:

When `type=app` and `subtype=factory`, the generator will emit:

```c
#define CODE_START_ADDR  (0x...)
#define CODE_SIZE        (0x...)
```

`CODE_START_ADDR` corresponds to the execution address specified by `exec`. If `exec` is absent, the storage CBUS address is used.

If macros such as `HCPU_FLASH_CODE` or `FLASH_BOOT_LOADER` are generated from execution-address compatibility rules and also include `_MEM_TYPE`, the `_MEM_TYPE` follows the final execution region rather than the storage region. If the execution region is on-chip RAM, the `_MEM_TYPE` will not be generated.

Filesystem partitions:

When `type=data` and `subtype` is one of `filesystem`/`littlefs`/`fat`/`fatfs`/`flashdb`, the following macros are emitted:

```c
#define FS_REGION_START_ADDR  (0x...)
#define FS_REGION_SIZE        (0x...)
#define FS_REGION_OFFSET      (0x...)
#define FS_REGION_MEM_TYPE    HAL_MEM_TYPE_...
```

FlashDB KV partitions:

When `type=data` and `subtype=flashdb_kv`, KVDB macros are generated (where `<NAME>` is the uppercase partition `name`, which also becomes the FlashDB database name and FAL partition name, e.g. `dfu`, `ble`):

```c
#define KVDB_<NAME>_REGION_START_ADDR  (0x...)
#define KVDB_<NAME>_REGION_OFFSET  (0x...)
#define KVDB_<NAME>_REGION_SIZE    (0x...)
#define KVDB_<NAME>_REGION_MEM_TYPE HAL_MEM_TYPE_...
```

FAL_PART_TABLE:

`ptab.h` will auto-generate `FAL_PART_TABLE` containing the following partitions:

- `type=data, subtype=flashdb_kv`
- `type=data, subtype=filesystem`
- `type=app` where `subtype != ex`

Excluded from auto-inclusion:

- `type=ftab`
- `type=bootloader`
- `type=data` where `subtype` is `raw`/`ram`/`calibration`/`int_res`
- `type=app` where `subtype=ex`

FAL partition names use the partition `name` directly and do not map legacy names like `main`, `dfu_code`, or `fs_root`.

Example generation:

```c
#define FAL_PART_TABLE \\
{ \\
    {FAL_PART_MAGIC_WORD, "hcpu_flash_code", NOR_FLASH5_DEV_NAME, 0x00020000, 0x00200000, 0}, \\
    {FAL_PART_MAGIC_WORD, "dfu_flash_code", NOR_FLASH5_DEV_NAME, 0x00220000, 0x00080000, 0}, \\
    {FAL_PART_MAGIC_WORD, "fs_region", NOR_FLASH5_DEV_NAME, 0x00600000, 0x00100000, 0}, \\
    {FAL_PART_MAGIC_WORD, "<name>", NOR_FLASHx_DEV_NAME, <offset>, <size>, 0}, \\
    ... \\
}
```

Device names are determined by the storage `region`, not by `exec`:

- `mpi1..mpi5` → `NOR_FLASH1_DEV_NAME` .. `NOR_FLASH5_DEV_NAME`
- `sdmmc1` → `SDMMC1_DEV_NAME`
- `sdmmc2` → `SDMMC2_DEV_NAME`

## Build system integration

### Auto-detection

The build system detects ptab format in this order:

1. Look for `ptab.yaml` (v3)
2. If absent, fall back to `ptab.json` (v1/v2)

Search priority:
1. `$BSP_ROOT/<board>/`
2. `$BSP_ROOT/<chip>/`
3. `$BSP_ROOT/`
4. `customer/boards/<board>/`

### Project-level partition overlays

If a project only needs small adjustments, use `ptab.overlay.yaml` rather than copying the full board `ptab.yaml`.

Overlay filenames supported:

- `<project>/<chip>/ptab.overlay.yaml`
- `<project>/<board>/ptab.overlay.yaml`

Notes:

- `<chip>` uses `CHIP.lower()` from board `rtconfig.py`, e.g. `sf32lb52x`
- `<board>` uses the full board name including core suffix, e.g. `sf32lb52-lcd_n16r8_hcpu`
- Board overlay has higher priority than chip overlay

Overlay files should only contain a top-level `partitions` field. Example:

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

`op` is optional but recommended for clarity. Meaning:

- `op: override` modifies an existing partition
- `op: add` adds a new partition
- If `op` is omitted: existing `name` → `override`, missing `name` → `add`

Restrictions:

- Overlay supports only partition-level changes; it cannot change `memory`, `chip`, or `version`
- Overlay cannot delete partitions
- Overlay applies only to board-level `ptab.yaml` (v3); it cannot be layered over `ptab.json`
- If the board still uses `ptab.json` (v1/v2) and no full `ptab.yaml` is found in the project, the build system will ignore `ptab.overlay.yaml`, emit a warning, and continue with legacy PTAB behavior
- If a project provides a full `ptab.yaml` / `ptab.json`, it must not also use `ptab.overlay.yaml`

When processing overlays the build system prints a summary via `rich` and writes `ptab.effective.yaml` in the build directory for inspection.

Note:

If an overlay is ignored because the board uses `ptab.json`, `ptab.effective.yaml` will not be generated. That warning means the build falls back to v1/v2 behavior, not that the overlay has been applied.

### Generated artifacts

| Artifact | Description |
|---------|-------------|
| `ptab.h` | C header with partition macros |
| `ftab.bin` | Binary Flash Table used by the bootloader |
| `link_copy.lds` / `link_copy.sct` | GCC / Keil link scripts |
| `ptab.effective.yaml` | Final effective PTAB YAML when overlay is applied |

### ptab v3 and `custom_mem_map.h`

Projects using ptab v3 do not need to maintain board-level `custom_mem_map.h`. Prefer using the generated `ptab.h` as the source of partition macros.

Migration steps to ptab v3:

1. Convert board `ptab.json` to `ptab.yaml`.
2. Remove board-level `custom_mem_map.h` (e.g. `customer/boards/<board>/{hcpu,lcpu,acpu}/custom_mem_map.h`).
3. Clean any existing `build_xxx` directories and rebuild.

Notes:

- Do not manually define `FAL_PART_TABLE` in board headers; `ptab.h` auto-generates it based on `flashdb_kv`/`filesystem`/`app(subtype != ex)` partitions.
- If the board `custom_mem_map.h` is not removed, the build may still generate `build_xxx/custom_mem_map.h`, leading to `FAL_PART_TABLE redefined` warnings.
- Deleting board headers while leaving old `build_xxx/custom_mem_map.h` files may also trigger legacy warnings; clean build directories to avoid this.

### app/ex resource partition artifacts (ptab v3 + GCC/Keil)

ptab v3 uses `type: app` with `subtype: ex` to describe resource partitions. The build system generates linker script fragments per resource partition: GCC assigns each resource partition a `MEMORY` region and emits an output section `.<name>` (lowercase), Keil produces `LR_<NAME>` / `ER_<NAME>` areas. Both toolchains normalize artifact names under `output/`.

GCC default collection rule:

```ld
.<name> : { *(.<NAME>*) } > <NAME>
```

If a partition declares `sections`, the build will also collect the specified input sections or `object + section` selectors in addition to the default rule.

Output files under each build directory's `output/`:

- Code image (ELF-exported code excluding all `.<resource_name>` sections):
  - Single final binary: `build_xxx/output/main.bin`, `build_xxx/output/main.hex`
  - When code image + one or more `app/ex` resource images are present: `build_xxx/output/main.app.bin`, `build_xxx/output/main.app.hex`
  - Project-named variants: `build_xxx/<proj>/output/<proj>.bin` or `.../<proj>.app.bin`
- Resource images (one file per `app/ex` partition):
  - Filenames use the partition `name` verbatim (not uppercased)
  - Examples:
    - `build_xxx/output/main.hcpu_flash2_img.bin`
    - `build_xxx/output/main.hcpu_flash2_font.bin`
  - Corresponding `.hex` files are also produced

If a resource partition has no corresponding input section or the section is empty, the build skips generating that partition's `main.<partition_name>.bin/.hex` without error.

Avoid naming `app/ex` partitions `app` to prevent filename collisions on case-insensitive filesystems.

### ftab.bin generation

In v3, `ftab.bin` is generated directly by a Python script rather than building an ftab subproject:

```
Generating build_xxx/ftab.bin ...
Generated ftab.bin: build_xxx/ftab.bin (11280 bytes)
```

### jinja2 link script templates

In ptab v1/v2, link scripts were produced using the compiler's C-style preprocessor.

In ptab v3:

- GCC link scripts use jinja2 templates located by default at `drivers/cmsis/<chip>/Templates/gcc/<core>/link.jinja2`
- Keil `.sct` jinja2 templates are located at `drivers/cmsis/<chip>/Templates/arm/<core>/link.sct.jinja2` and need manual maintenance

Notes:

- `tools/build/migrate_ptab_to_v3.py` only migrates `ptab.json -> ptab.yaml`; it does not migrate link scripts.
- If a project or board uses custom link scripts, those must be manually migrated to `link.jinja2` or `link.sct.jinja2`.
- ptab v3 requires jinja2 templates; it cannot use the C preprocessor-based link script generation.
- ptab v1/v2 do not support jinja2 templates.

### `validate_ptab_v3.py`

Validate a ptab v3 file for correctness:

```bash
python tools/build/validate_ptab_v3.py customer/boards/<board>/ptab.yaml
```

Validation checks include:
- Partition name format
- Type/subtype validity
- Region name validity
- Partition overlap detection
- `bootloader` partition uniqueness

### `sdk.py ptab-export`

To inspect the final effective PTAB YAML for a project run:

```bash
sdk.py ptab-export --board=<board_name>
```

Common usage:

```bash
sdk.py ptab-export --board=sf32lb52-lcd_n16r8_hcpu
sdk.py ptab-export --board=sf32lb52-lcd_n16r8_hcpu --output /tmp/ptab.effective.yaml
sdk.py ptab-export --strict
```

Notes:

- By default, the final YAML is printed to stdout.
- `--output` writes the effective YAML to a file
- `--strict` treats warnings as failures
- This command uses the same PTAB parsing and overlay merge logic as the build

## FAQ

### Q: How do I add custom macros?

Use the `attrs` field:

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

This generates:

```c
#define MY_CUSTOM_MACRO  (0x12345678)
```

### Q: How do ptab v1/v2 `tags` map to v3?

| v1/v2 tags | v3 mapping |
|------------|-----------|
| `FLASH_TABLE` | `type: ftab` |
| `FLASH_BOOT_LOADER` | `type: bootloader` |
| `FS_REGION` | `type: data, subtype: filesystem` |
| `KVDB_*` | `type: data, subtype: flashdb_kv` |
| `app_img` | `type: app` |
| `app_exec` | use `exec: {region, offset}` when XIP is not possible or a fixed exec address is required |

## Change history

| Version | Date | Notes |
|---------|------|-------|
| v3.4 | 2026-04 | `FAL_PART_TABLE` automatically includes `flashdb_kv`, `filesystem` and `app(subtype != ex)` partitions, and adds `mpi5` / `sdmmc` mapping rules |
| v3.3 | 2026-03 | Added default partition inference rules; allowed omission of default `flash_table` / `bootloader` / some RAM partitions |
| v3.2 | 2026-03 | Added project-level `ptab.overlay.yaml`, `sdk.py ptab-export`, and `ptab.effective.yaml` export |
| v3.1 | 2026-03 | Clarified relationship between ptab v3 and `custom_mem_map.h`, and `FAL_PART_TABLE redefined` warning reasons and fixes |
| v3.0 | 2024-12 | Initial release; YAML format; removed ftab subproject |
