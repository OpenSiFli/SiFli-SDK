---
name: sf32lb57-add-part-number
description: Add a new SF32LB57X chip part number to the SiFli SDK. Use this skill when the user wants to add a new chip part number, 添加新料号, or add a new part number. Guides through Kconfig_soc.sf32lb57x, Kconfig_soc.sf32lb57x.v1 and a new soc/sf32lb57x/Kconfig.<part> file with the correct MPI mode, pinmap mode, package type, and memory size settings.
---

# Add New SF32LB57X Part Number

Adds a new chip part number to the SF32LB57X family in the SiFli SDK.

Every per-part-number default lives in **one file per part number** under
`soc/sf32lb57x/`. `customer/boards/Kconfig_drv` and
`Kconfig_soc.sf32lb57x.common` hold only part-number-independent fallbacks and
must **not** be edited when adding a part number.

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

For **every** MPI controller the part enables, the mode must be stated
explicitly — including NOR (`BSP_MPIx_MODE_0`). Never leave it to the shared
fallback: a later change to that fallback would then silently change this part.

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

### Step 3: new file `soc/sf32lb57x/Kconfig.<part>`

This is the only place for the part's defaults. The file name is `Kconfig.`
followed by the part number in lowercase (`Kconfig.sf32lb57eybbn6`,
`Kconfig.sf32bprtyb3n6`), and it must end with CRLF line endings like the rest
of the tree.

```kconfig
if SOC_SF32LB57xxxN6
configdefault BSP_ENABLE_MPI1
    default y

choice BSP_MPI1_MODE_CHOICE
    default BSP_MPI1_MODE_<n>
endchoice

configdefault BSP_QSPI1_MEM_SIZE
    default <MB>

configdefault BSP_ENABLE_MPI2
    default y

choice BSP_MPI2_MODE_CHOICE
    default BSP_MPI2_MODE_<n>
endchoice

configdefault BSP_QSPI2_MEM_SIZE
    default <MB>

# only when MPI1 runs a PSRAM mode (2/3/4/5/6)
configdefault BSP_PSRAM1_PKG_TYPE0_MPI_MODE
    default <n>

configdefault BSP_PSRAM1_PKG_TYPE0_PINMAP_MODE
    default <n>

# only when the part has a secondary PSRAM1 supplier: mode and pinmap together
configdefault BSP_PSRAM1_PKG_TYPE1_MPI_MODE
    default <n>

configdefault BSP_PSRAM1_PKG_TYPE1_PINMAP_MODE
    default <n>

# only when MPI2 runs a PSRAM mode (2/3/4/5/6)
configdefault BSP_PSRAM2_PKG_TYPE0_MPI_MODE
    default <n>

configdefault BSP_PSRAM2_PKG_TYPE0_PINMAP_MODE
    default <n>
endif
```

Which symbol carries which setting:

| Setting | Entry in the part file |
|---|---|
| MPI controller enable | `configdefault BSP_ENABLE_MPI<n>` / `default y` |
| MPI mode | `choice BSP_MPI<n>_MODE_CHOICE` / `default BSP_MPI<n>_MODE_<m>` |
| Memory size (MB) | `configdefault BSP_QSPI<n>_MEM_SIZE` / `default <MB>` |
| PSRAM pkg type MPI mode | `configdefault BSP_PSRAM<n>_PKG_TYPE<m>_MPI_MODE` |
| PSRAM pkg type pinmap | `configdefault BSP_PSRAM<n>_PKG_TYPE<m>_PINMAP_MODE` |

Rules:

- A `configdefault` block may contain **only `default` statements** — no
  prompt, `depends on`, `select`, `imply` or `range` (kconfiglib rejects them).
  Conditions go on the default itself: `default <v> if <cond>`.
- Write the entries for a controller together, in this order: enable, mode,
  memory size. Omit a controller entirely when the part does not enable it.
- State a PSRAM's `PKG_TYPE0` mode and pinmap **only when the matching MPI
  controller runs a PSRAM mode (2/3/4/5/6)** — that is when the PSRAM is
  selected and the values are actually read. For NOR (mode 0) or a controller
  the part does not enable, leave them out. Never omit a value the part *does*
  use: the shared fallback would then decide it silently.
  (`BSP_PSRAM1_PKG_TYPE1_*` describes a secondary PSRAM1 supplier: for a part
  that populates one, state **both** its mode and its pinmap (e.g.
  `default 6` and `default 2`) — never leave the type-1 pinmap to the shared
  default. TYPE2/TYPE3 are not used yet.)
- The MPI mode decides whether PSRAM or NOR is used. For a PSRAM mode
  (2/3/4/5/6) Kconfig automatically `select`s `BSP_USING_PSRAM` and enables
  `BSP_USING_PSRAM1/2`, which is what makes the `BSP_PSRAM*_PKG_TYPE*_*`
  values take effect.

### Step 4: board `Kconfig.board` (for boards that use the new part)

Each core's board file selects the part symbol:

```kconfig
config BSP_USING_BOARD_XXX
    bool
    select SOC_SF32LB57X
    select SOC_SF32LB57xxxN6
    select BF0_HCPU        # BF0_LCPU / BF0_ACPU on the other cores
    default y
```

### Do not edit these when adding a part number

`customer/boards/Kconfig_drv` and
`customer/boards/Kconfig_soc.sf32lb57x.common` — they keep only the
part-number-independent fallbacks (`default n`, `default BSP_MPIx_MODE_0`,
default memory sizes, default pinmap modes).

`Kconfig_soc.sf32lb57x.common` pulls the part files in with

```kconfig
source "$SIFLI_SDK/soc/sf32lb57x/Kconfig.*"
```

and that line must stay **before** the `config` declarations in that file and
before `Kconfig_drv` is parsed. `configdefault` inserts its defaults at the
position where it was parsed, and kconfiglib picks the first default whose
condition holds, so a part file that is sourced later would be silently
overridden by the unconditional fallback defaults. Keep the line where it is.

## Verification

After making all changes, verify by:

1. Checking that the board's `Kconfig.board` selects the correct
   `SOC_SF32LB57xxxN6` symbol
2. Building a project that uses this board — a Kconfig warning aborts the
   build, so a successful build means the part file parses cleanly:

   ```
   scons --board=<board_name> -j8
   ```

3. Checking the generated `build_<board_name>/<core>/rtconfig.h` for the
   expected values, e.g. for the part above:

   ```
   #define BSP_ENABLE_MPI1 1
   #define BSP_MPI1_MODE_5 1
   #define BSP_QSPI1_MEM_SIZE 4
   ```

   Re-running the build with the part file removed must change these values —
   if it does not, the file is not being sourced.
4. Running `sdk.py menuconfig --board=<board_name>` to verify the MPI mode
   choices show the intended selection.

Note: adding a part file shifts the line order of a few `#define`s in the
generated `.config` / `rtconfig.h` (the symbols' first menu node moves into the
early-sourced part file). The values are unchanged; do not chase that diff.

## Example

Part `SF32LB579V6EN6` (BGA112, MPI1 as 8 MB NOR flash, MPI2 in OPSRAM mode with
PSRAM2, 32 MB). Note there is no `BSP_PSRAM1_*` entry: MPI1 runs NOR, so PSRAM1
is never selected:

**`customer/boards/Kconfig_soc.sf32lb57x`**: `select SOC_PACKAGE_BGA112`

**`customer/boards/Kconfig_soc.sf32lb57x.v1`**: `bool "SF32LB579V6EN6"` +
`select SOC_PACKAGE_BGA112`

**`soc/sf32lb57x/Kconfig.sf32lb579v6en6`**:

```kconfig
if SOC_SF32LB579V6EN6
configdefault BSP_ENABLE_MPI1
    default y

choice BSP_MPI1_MODE_CHOICE
    default BSP_MPI1_MODE_0
endchoice

configdefault BSP_QSPI1_MEM_SIZE
    default 8

configdefault BSP_ENABLE_MPI2
    default y

choice BSP_MPI2_MODE_CHOICE
    default BSP_MPI2_MODE_3
endchoice

configdefault BSP_QSPI2_MEM_SIZE
    default 32

configdefault BSP_PSRAM2_PKG_TYPE0_MPI_MODE
    default 3

configdefault BSP_PSRAM2_PKG_TYPE0_PINMAP_MODE
    default 3
endif
```
