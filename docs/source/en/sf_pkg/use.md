# Using SiFli Package Registry in Your Project

The following steps explain how to install and use SiFli Package Registry dependencies in an existing project.

```{note}
Some screenshots on this page were captured with the old CLI format. If a screenshot differs from the text, follow the current `sdk.py sf-pkg ...` commands in the document.
```

## Initialize Dependencies (sf-pkg init)

```bash
sdk.py sf-pkg init
```

Upon successful execution, a `sf-pkg.yaml` declaration file will be generated in the `project` directory (YAML, comment-friendly). For example:

```yaml
# Project level fixed dependencies: each entry is <package>/<version>@<username>
requires:
  - sht30/0.0.4@caisong123                 # exact version
  - battery_calculator/[>=1.2 <2.0]@sifli  # version range (Conan syntax, see "Version Range Syntax" below)
  - legacy-lib/[*]@acme                    # any version

# Optional: SDK version range this project supports (npm semver syntax, different from requires)
# When omitted, no SDK version check is done, equivalent to "*"
# support_sdk_version: ">=2.4 <3"
```

An empty `requires` means no dependencies. Add packages as `<package>/<version>@<username>`, e.g. the SHT30 sensor driver: `sht30/0.0.4@caisong123`. The version can also be a range; see [Version Range Syntax](#version-range-syntax) below.

```{note}
Older SDK versions declared dependencies through `conanfile.py`. If a project root contains a `conanfile.py` but no `sf-pkg.yaml`, `sf-pkg install` treats it as "advanced mode" and runs `conan install` on that file directly (legacy behavior, without module-level dependency aggregation). New projects are encouraged to use `sf-pkg.yaml`.
```

### sf-pkg.yaml Field Reference

`sf-pkg.yaml` is used both at the project root and by SDK built-in modules:

- `requires` (list, optional): external components needed by this project/module. Each entry is `<package>/<version>@<username>`; version ranges are allowed (Conan syntax), e.g. `sht30/0.0.4@caisong123` or `battery_calculator/[>=1.2 <2.0]@sifli`. Empty/absent means no dependencies.
- `support_sdk_version` (string, optional): required SDK version range in npm-style semver (e.g. `>=2.4 <3`, `>=2.4.1 <2.5.0`; the shorthand forms `^2.4` and `~2.4.1` also work, and note the syntax differs from `requires`); unchecked when absent, which is equivalent to `"*"`. During install/build it is validated against the `SIFLI_SDK_VERSION` environment variable and fails before any network access, naming the source file. The project root and each module declare and are checked separately.
- `enable` (list, optional, **module manifests only**): the module's Kconfig enable symbols. If **any** of them is `y`, the module is considered to participate in the build and its `requires` are pulled; otherwise they are not. These are the real symbol names as defined in Kconfig (kconfig only prepends a `CONFIG_` prefix when writing `.config`; if a symbol's own name already starts with `CONFIG_`, write it as is). When absent/empty the module is treated as "always participating", which is only suitable for fixed modules without a switch.

`enable` has no effect at the project root: the root is a fixed dependency and only `requires` (and optionally `support_sdk_version`) belong there.

### Version Range Syntax

`requires` and `support_sdk_version` use **two different version syntaxes**; mixing them up leads to parse failures or unresolved packages:

| Field | Parser | Syntax | Example |
|---|---|---|---|
| `requires` | Conan 2 | conditions inside brackets, separated by spaces | `foo/[>=1.2 <2.0]@sifli` |
| `support_sdk_version` | npm semver | no brackets, space separated | `>=2.4 <3` |

#### requires: Conan Version Ranges

The format is `<package>/[<range>]@<username>[/<channel>]`; the brackets wrap only the version part. Omitting the brackets means an exact version (e.g. `sht30/0.0.4@caisong123`).

**Writing the lower and upper bounds explicitly is recommended** — it is the least ambiguous:

| Syntax | Meaning |
|---|---|
| `[>=1.2 <2.0]` | 1.2 or newer, below 2.0 (includes 1.2.0, excludes 2.0.0) |
| `[>=1.2.3 <1.5.0]` | 1.2.3 or newer, below 1.5.0 |
| `[>=1.2]` | 1.2 or newer, no upper bound |
| `[<2.0]` | below 2.0 |
| `[1.2.3]` | exactly 1.2.3 |
| `[*]` | any version (still excludes pre-releases) |

Rules:

- Multiple conditions are ANDed, separated by **spaces**; use `||` between condition groups, e.g. `[>=1.0 <2.0 || >=3.0 <4.0]`.
- **A comma is not "and"**; it separates range options such as `include_prerelease`. Writing `>=1.2,<2.0` fails with `"<2.0" in version range ">=1.2,<2.0" is not a valid option`.
- `>=` includes the lower bound and `<` excludes the upper bound; use `<=` if the upper bound should be included.
- Pre-releases (e.g. `1.3.0-beta1`) do not match by default; add `, include_prerelease` inside the range to allow them.
- At install time Conan picks the **highest** version available from the remote that satisfies the range; ranges declared for the same package by several modules are intersected.

Shorthand forms (optional; if unfamiliar with them, just use the explicit form above):

| Syntax | Equivalent to | Notes |
|---|---|---|
| `[^1.2]` | `>=1.2 <2.0` | locks the left-most non-zero component, i.e. the major stays |
| `[~1.2]` | `>=1.2 <1.3` | locks the minor version |
| `[~1.2.3]` | `>=1.2.3 <1.3` | locks the minor version, starting at 1.2.3 |
| `[1.2.*]` | `1.2.x` | prefix wildcard |

Note that for 0.x versions `^` behaves counter-intuitively: `[^0.2]` is `>=0.2 <0.3` and `[^0.0.3]` is `>=0.0.3 <0.0.4`. In such cases write `>= x < y` explicitly.

Example:

```yaml
requires:
  - sht30/0.0.4@caisong123                  # exact version
  - battery_calculator/[>=1.2 <2.0]@sifli   # 1.2 ~ 1.9.x
  - mesh-lib/[>=1.3 <2.0]@acme              # explicit bounds
  - codec/[>=1.4.2 <1.5.0]@sifli            # only 1.4.x, at least 1.4.2
  - compat/[>=1.0 <2.0 || >=3.0 <4.0]@acme  # 1.x or 3.x
  - legacy-lib/[*]@acme                     # any version
```

#### support_sdk_version: npm semver Ranges

The SDK version comes from the `SIFLI_SDK_VERSION` environment variable, which `export.ps1` / `set_env.bat` sets to `<major>.<minor>` (e.g. `2.5`); it is treated as `2.5.0` during validation.

**Writing the bounds explicitly is recommended here as well**:

| Syntax | Meaning |
|---|---|
| `>=2.4 <3` | 2.4 or newer, below 3.0.0 (recommended) |
| `>=2.4.0 <2.6.0` | full three-part form, more explicit |
| `>=2.4` | 2.4 or newer, no upper bound |
| `*` | any version |

Rules:

- Separate conditions with **spaces**. **Do not use commas**: `>=2.4,<3` cannot be parsed and fails with `Invalid NPM block`.
- Omitted components default to 0, so `>=2.4` equals `>=2.4.0`; an SDK version such as `2.5` is also compared as `2.5.0`.
- When `support_sdk_version` is absent no version check is performed, which is equivalent to writing `*`; `*` can be written explicitly but usually there is no need.
- On mismatch the operation fails before any network access and names the `sf-pkg.yaml` that declared the range.

Shorthand forms (optional):

| Syntax | Equivalent to | Notes |
|---|---|---|
| `^2.4` | `>=2.4.0 <3.0.0` | the major stays |
| `~2.4` | `>=2.4.0 <2.5.0` | the minor stays |
| `~2.4.1` | `>=2.4.1 <2.5.0` | only patch upgrades |
| `2.4.x` | `>=2.4.0 <2.5.0` | prefix wildcard |

Example:

```yaml
support_sdk_version: ">=2.4 <3"           # SDK 2.4.0 or newer, below 3.0.0 (recommended)
# support_sdk_version: ">=2.4.1 <2.5.0"   # only 2.4.x, at least 2.4.1
# support_sdk_version: "*"                # any version (same as omitting it)
```

### Module Level External Dependencies

SDK built-in modules (e.g. `middleware/*`, board drivers, ...) can each carry a `sf-pkg.yaml` declaring external components that are only fetched when that module participates in the build:

```yaml
# middleware/<module>/sf-pkg.yaml
# Must live in the same directory as the Kconfig that defines the module's
# enable symbols
enable:
  - BLE_STACK            # Kconfig enable symbol; any 'y' means "participates"
requires:
  - mesh-lib/1.3.0@acme  # conan reference, same format as the project root
support_sdk_version: ">=2.4 <3"   # optional, same meaning as the project root
```

- The manifest must sit **next to the module's Kconfig**, and that Kconfig must be parsed in the current board's config tree; otherwise the manifest is not discovered (no participation, no fetching).
- When a module is enabled, also `select` the package's enable symbol declared in `Kconfig.conandeps` from the module's own Kconfig so the package actually links into the build.
- See the [module level dependencies design](design/module_deps.md) for the full mechanism and conventions.

## Search for Available Packages

If you are unsure about the package name or version, you can search for it:

```bash
sdk.py sf-pkg search <package_name>
```

Example:

```bash
sdk.py sf-pkg search sht30
```
You can also search directly on the official website of the SiFli Package Registry: Click here to visit [SiFli Package Registry](https://packages.sifli.com/)


## Install Dependencies (sf-pkg install)

Execute the following command in the `project` directory of your project:

```bash
sdk.py sf-pkg install
```

- Without `--board`: installs only the project root `sf-pkg.yaml` `requires`; module-level dependencies are not merged.
- With `--board <board>`: additionally merges the external components declared by the modules enabled in that board configuration. The board config is resolved automatically from `board.conf` + `proj.conf` (generating `kconfiglist`/`.config` in the build dir), so no prior compilation is needed.
- With `--board-search-path <dir>`: adds a board search directory on top of the SDK `customer/boards` (same as scons `--board_search_path`; the `SIFLI_SDK_BOARD_SEARCH_PATH` environment variable works too).

![Install Dependencies](./assets/sf-pkg-install.png)

After successful installation, an `sf-pkgs` folder will be generated in the `project` directory, containing the installed packages.

### Automatic Installation During Build

As long as the project root has an `sf-pkg.yaml`, running `scons --board=<b>` automatically detects whether the aggregated dependency set of the modules enabled for that board changed and, if so, installs it before continuing to compile (idempotent; skipped when unchanged). To disable automatic installation:

```bash
SIFLI_SF_PKG_OFFLINE=1 scons --board=<b>     # or: scons --no-sf-pkg
```

## Using the Driver

Once installation is complete, you can use the driver directly:

- You can compile immediately
- When including header files, there is no need to specify absolute paths—Conan will automatically handle path configuration

### Kconfig Configuration Notes

- `menuconfig` will automatically integrate all `Kconfig` files under the `sf-pkgs` folder
- These configuration options will appear in the **SiFli External Components** menu within `menuconfig`
