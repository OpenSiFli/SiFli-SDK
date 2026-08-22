---
name: sf32lb57-add-part-number
description: Add a new SF32LB57X chip part number to the SiFli SDK. Use this skill when the user wants to add a new chip part number, 添加新料号, or add a new part number. Guides through modifying Kconfig_soc.sf32lb57x, Kconfig_soc.sf32lb57x.v1, Kconfig_soc.sf32lb57x.common, and Kconfig_drv with the correct MPI mode, pinmap mode, package type, and memory size settings.
---

# Add New SF32LB57X Part Number

Adds a new chip part number to the SF32LB57X family in the SiFli SDK.

## Before You Start

**IMPORTANT: You MUST ask the user for ALL of the following information before making any changes. Do NOT infer or guess any values from the part number.** If the user forgets to provide any of these, ask them explicitly.

Collect the following information from the user:

1. **Part number string** (e.g., `SF32LB573UB7N6`)
2. **Package type**: `QFN68`, `QFN80`, or `BGA112` — can be inferred from the part number's package letter (see table below), but still confirm with the user
3. **MPI1 (PSRAM1) settings**:
   - Primary supplier (pkg_type0) MPI mode and **pinmap mode** — **ask for both, do not assume defaults**
   - Secondary supplier (pkg_type1) MPI mode and **pinmap mode** (if applicable)
   - Memory size (MB)
4. **MPI2 (PSRAM2) settings**:
   - Primary supplier (pkg_type0) MPI mode and **pinmap mode**
   - Secondary supplier (pkg_type1) MPI mode and **pinmap mode** (if applicable)
   - Memory size (MB)
5. **MPI3 settings** (if NOR Flash is connected):
   - MPI mode (typically `BSP_MPI3_MODE_0` for NOR)
   - Memory size (MB)

### Reference: Package Type Mapping from Part Number

The package letter (e.g., `U` in `SF32LB577**U**DNN6`) indicates the package type:

| Letter | Package Type |
|--------|-------------|
| U | QFN68 |
| Y | QFN80 |
| V | BGA112 |

### MPI Mode Reference

| Value | Mode |
|-------|------|
| 0 | NOR |
| 1 | NAND |
| 2 | PSRAM |
| 3 | OPSRAM (Xccela) |
| 4 | HPSRAM |
| 5 | LEGACY_PSRAM |
| 6 | HYPERBUS_PSRAM |

### Pinmap Mode Reference

| Value | Meaning |
|-------|---------|
| 1 | Specific pin mapping for PSRAM type (defined per board) |
| 2 | Default/secondary pin mapping |
| 3 | Specific pin mapping for certain package types |

## Modification Steps

### Step 1: `customer/boards/Kconfig_soc.sf32lb57x`

Add a simple config symbol (auto-selected by board Kconfig). Use `select SOC_PACKAGE_*` based on package type:

```kconfig
config SOC_SF32LB57xxxN6
    bool
    select SOC_PACKAGE_QFN68   # or QFN80 / BGA112
```

- Ensure the config name follows the pattern `SOC_SF32LB57` + chip suffix (e.g., `SOC_SF32LB573UB7N6`)
- Add it in alphabetical order within the existing list

### Step 2: `customer/boards/Kconfig_soc.sf32lb57x.v1`

Add to the `SOC_SF32LB57X_PART` choice block for manual user selection:

```kconfig
config SOC_SF32LB57xxxN6
    bool "SF32LB57xxxN6"
    select SOC_PACKAGE_QFN68   # or QFN80 / BGA112
```

- Add it in alphabetical order within the choice block

### Step 3: `customer/boards/Kconfig_soc.sf32lb57x.common`

Add PSRAM MPI mode and pinmap mode defaults. Use `depends on BSP_USING_PSRAM1` or `BSP_USING_PSRAM2` respectively.

For PSRAM1 (MPI1):
```kconfig
config BSP_PSRAM1_PKG_TYPE0_MPI_MODE
    int
    depends on BSP_USING_PSRAM1
    default <n> if SOC_SF32LB57xxxN6
    ...

config BSP_PSRAM1_PKG_TYPE0_PINMAP_MODE
    int
    depends on BSP_USING_PSRAM1
    default <n> if SOC_SF32LB57xxxN6
    ...

config BSP_PSRAM1_PKG_TYPE1_MPI_MODE
    int
    depends on BSP_USING_PSRAM1
    default <n> if SOC_SF32LB57xxxN6
    ...

config BSP_PSRAM1_PKG_TYPE1_PINMAP_MODE
    int
    depends on BSP_USING_PSRAM1
    default <n> if SOC_SF32LB57xxxN6
    ...
```

For PSRAM2 (MPI2): use `BSP_PSRAM2_PKG_TYPE*` equivalents.

- Add the `if SOC_SF32LB57xxxN6` condition to existing `default` lines for the corresponding config
- If the part does not use PSRAM on a given MPI, no changes needed (falls back to existing defaults)

### Step 4: `customer/boards/Kconfig_drv`

Three places to modify:

#### 4a. Enable MPI controllers

```kconfig
config BSP_ENABLE_MPI1
    default y if SOC_SF32LB57xxxN6

config BSP_ENABLE_MPI2
    default y if SOC_SF32LB57xxxN6

config BSP_ENABLE_MPI3
    default y if SOC_SF32LB57xxxN6   # only if NOR Flash connected
```

#### 4b. Select default MPI mode in the choice blocks

For MPI1 (inside `BSP_ENABLE_MPI1` → `choice` block):
```kconfig
default BSP_MPI1_MODE_<n> if SOC_SF32LB57xxxN6
```

For MPI2 (inside `BSP_ENABLE_MPI2` → `choice` block):
```kconfig
default BSP_MPI2_MODE_<n> if SOC_SF32LB57xxxN6
```

For MPI3 (inside `BSP_ENABLE_MPI3` → `choice` block):
```kconfig
default BSP_MPI3_MODE_<n> if SOC_SF32LB57xxxN6
```

**Important**: The MPI mode selected here determines whether PSRAM or NOR Flash is used. If this is a PSRAM mode (2/3/4/5/6), Kconfig will automatically `select BSP_USING_PSRAM` and enable `BSP_USING_PSRAM1/2`, and then the values from `Kconfig_soc.sf32lb57x.common` take effect.

#### 4c. Set memory size

```kconfig
config BSP_QSPI1_MEM_SIZE
    default <MB> if SOC_SF32LB57xxxN6

config BSP_QSPI2_MEM_SIZE
    default <MB> if SOC_SF32LB57xxxN6

config BSP_QSPI3_MEM_SIZE
    default <MB> if SOC_SF32LB57xxxN6   # only if MPI3 enabled
```

## Verification

After making all changes, verify by:
1. Checking that the board's `Kconfig.board` selects the correct `SOC_SF32LB57xxxN6` symbol
2. Building a project that uses this board to ensure Kconfig resolves correctly
3. Running `sdk.py menuconfig --board=<board_name>` to verify defaults are applied

## Example

If the part is `SF32LB579V6EN6` (BGA112, PSRAM1 on MPI1 in OPSRAM mode, PSRAM2 on MPI2 in OPSRAM mode):

**Kconfig_soc.sf32lb57x**: `select SOC_PACKAGE_BGA112`

**Kconfig_soc.sf32lb57x.v1**: `bool "SF32LB579V6EN6"` + `select SOC_PACKAGE_BGA112`

**Kconfig_soc.sf32lb57x.common**:
- `BSP_PSRAM1_PKG_TYPE0_MPI_MODE default 3 if SOC_SF32LB579V6EN6`
- `BSP_PSRAM1_PKG_TYPE0_PINMAP_MODE default 3 if SOC_SF32LB579V6EN6`
- `BSP_PSRAM2_PKG_TYPE0_MPI_MODE default 3 if SOC_SF32LB579V6EN6`
- `BSP_PSRAM2_PKG_TYPE0_PINMAP_MODE default 3 if SOC_SF32LB579V6EN6`

**Kconfig_drv**:
- `BSP_ENABLE_MPI1 default y if SOC_SF32LB579V6EN6`
- `BSP_ENABLE_MPI2 default y if SOC_SF32LB579V6EN6`
- `default BSP_MPI1_MODE_3 if SOC_SF32LB579V6EN6` (in MPI1 choice)
- `default BSP_MPI2_MODE_3 if SOC_SF32LB579V6EN6` (in MPI2 choice)
- `BSP_QSPI1_MEM_SIZE default 8 if SOC_SF32LB579V6EN6`
- `BSP_QSPI2_MEM_SIZE default 32 if SOC_SF32LB579V6EN6`