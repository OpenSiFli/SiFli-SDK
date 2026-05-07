# LVGL Multilingual Example
## Introduction
This example demonstrates the basic multilingual flow of `lv_ext_resouce` in the current SDK, including:

- Builtin language-pack loading
- External language-pack installation and loading
- Runtime locale switching
- Filesystem installer-directory rescan
- Current locale persistence through `share_prefs`
- Conversion flows from both `json` and `xlsx` language sources

The main related code paths are:

- Example entry: `example/multimedia/lvgl/Multilingual/src/app_utils/main.c`
- Builtin/external language resource scripts: `example/multimedia/lvgl/Multilingual/src/resource/strings/`
- Multilingual middleware: `middleware/lvgl/lv_ext_resouce/`

## Build and Download
The board project is located in the `project` directory. You can generate a board-specific build by selecting a `board`.

- For example, to build for `sf32lb52-lchspi-ulp`:

```bash
scons --board=sf32lb52-lchspi-ulp -j8
```

- After the build completes, you can use the download script in the build directory:

```bash
build_sf32lb52-lchspi-ulp_hcpu/uart_download.sh
```

Or on Windows:

```bat
build_sf32lb52-lchspi-ulp_hcpu\uart_download.bat
```

If JLink is available, you can also use:

```bat
build_sf32lb52-lchspi-ulp_hcpu\download.bat
```

## Supported Platform

- sf32lb52-lchspi-ulp

## Overview
After boot, this example creates a scrollable page that shows:

- The current active locale
- The locale stored in `share_prefs`
- The registered language-pack nodes and locale list
- The external language-pack list in the filesystem installer directory
- Builtin locale switch buttons
- External locale switch button
- Installer rescan button

The default locale actions validated in `main.c` are:

- Builtin: `en_us`
- Builtin: `zh_cn`
- External: `zh_tw`
- Rescan external installer directory: `/ex/resource/lang/installer`

## Directory Layout
The key multilingual-related directories are:

```text
Multilingual/
├── project/
├── disk/
│   └── ex/resource/lang/installer/
├── src/
│   ├── app_utils/main.c
│   └── resource/
│       └── strings/
│           ├── SConscript
│           ├── in_lang_packs/
│           │   ├── SConscript
│           │   ├── json/
│           │   └── xlsx/
│           └── ex_lang_packs/
│               ├── SConscript
│               ├── json/
│               └── xlsx/
```

Where:

- `in_lang_packs` contains builtin language-pack sources
- `ex_lang_packs` contains external language-pack sources
- `disk/ex/resource/lang/installer/` is the installation directory for external `.bin` language packs

## Builtin and External Language Packs
### Builtin Language Packs
Builtin language packs are converted into `lang_pack.c/.h` during the build and compiled into the firmware.

The default generated output paths are:

- `project/build_<board>_hcpu/src/resource/strings/lang_pack.h`
- `project/build_<board>_hcpu/src/resource/strings/lang_pack.c`

Builtin packs have the following characteristics:

- They are linked into the firmware
- They are available immediately after boot
- They are suitable as the default language set
- This example uses them to validate `en_us` and `zh_cn`

### External Language Packs
External language packs are converted into `.bin` files during the build and placed into the filesystem installer directory:

- `disk/ex/resource/lang/installer/*.bin`

These `.bin` files are then packed into the filesystem image, for example:

- `project/build_<board>_hcpu/fs_root.bin`

At runtime, they are loaded by the following flow:

- `app_lang_load_pack_list()` scans the installer directory
- `app_lang_install_pack()` installs the selected language pack
- `app_locale_lang_update()` switches the current locale

External packs have the following characteristics:

- They can be replaced without rebuilding the main firmware
- They are suitable for OTA, Bluetooth, or file-based update scenarios
- This example uses "Rescan installer" to simulate rescanning and reloading

## Conversion Flows from Different Sources
This example supports two types of source inputs:

- `json`
- `xlsx`

Builtin and external sources can be selected independently. They do not have to use the same source type.

For example, the following combinations are all valid:

- Builtin `json`, external `json`
- Builtin `xlsx`, external `xlsx`
- Builtin `json`, external `xlsx`
- Builtin `xlsx`, external `json`

### Builtin Language-Pack Conversion
Builtin language packs support two conversion flows:

- `json -> lang_pack.c/.h`
- `xlsx -> lang_pack.c/.h`

The controlling script is:

- `src/resource/strings/in_lang_packs/SConscript`

The current default configuration is:

```python
lang_mode = 'xlsx'
lang_sources = Glob('xlsx/*.xlsx')
```

If you want to switch back to the classic `json` mode, change it to:

```python
# lang_mode = 'xlsx'
# lang_sources = Glob('xlsx/*.xlsx')

lang_mode = 'json'
lang_sources = Glob('json/*.json')
```

### External Language-Pack Conversion
External language packs also support two conversion flows:

- `json -> .bin`
- `xlsx -> .bin`

The controlling script is:

- `src/resource/strings/ex_lang_packs/SConscript`

The current default configuration is:

```python
lang_mode = 'xlsx'
lang_sources = Glob('xlsx/*.xlsx')
```

If you want to switch to `json` mode, change it to:

```python
# lang_mode = 'xlsx'
# lang_sources = Glob('xlsx/*.xlsx')

lang_mode = 'json'
lang_sources = Glob('json/*.json')
```

### Current Default Behavior
The current default behavior in `src/resource/strings/SConscript` is:

- Builtin: `xlsx -> c/h`
- External: `xlsx -> bin`

The default external language-pack version is:

```python
external_pack_version = 'v1.0.4'
```

## xlsx Table Format Requirements
The `xlsx` language table must follow these basic rules:

- Row 1 is the header row
- Column A is the string key
- Column B is the description or default note
- Starting from column C, each column represents one language

An example of the table format is shown below:

![xlsx language table](src/assets/image_xlsx.png)

The language header supports two formats:

- `zh_tw`
- `zh_tw(zh_tw)`

In the current script, both formats are parsed to the same result:

- `stem = zh_tw`
- `locale = zh_tw`

You only need the parenthesized form when you want the generated name and the runtime locale name to be different, for example:

- `zh_tw(Traditional Chinese)`

In this case:

- `zh_tw` is used for generated file names and variable names
- `Traditional Chinese` is used as the runtime locale string

Notes:

- `stem` must follow C identifier rules
- Language columns in the same table must be unique
- Keys in the same table must be unique
- External language packs and builtin language packs should use the same key schema

## How to Use the Example
### Build and Flash
Enter the `project` directory and run:

```bash
scons --board=sf32lb52-lchspi-ulp -j8
```

After flashing and booting the example, the UI will show:

- Current locale
- Stored locale
- Registered locale list
- Installer locale list
- Locale switching buttons

### Builtin Locale Validation
Tap the following buttons to validate builtin locale switching:

- `Switch en_us`
- `Switch zh_cn`

Validation points:

- Page text switches immediately
- `Current locale` is updated
- `Stored locale` is updated

### External Locale Validation
Tap the following button to validate external locale switching:

- `Switch zh_tw`

Precondition:

- A matching `zh_tw.bin` already exists in the filesystem installer directory

Validation points:

- `Installer locale list` shows `zh_tw`
- After switching, the page text changes to the external-language content

### NVM Persistence Validation
This project enables the following option in `proj.conf`:

- `CONFIG_BSP_SHARE_PREFS=y`

Therefore, the current locale is written into `share_prefs` after switching.

Validation steps:

1. Switch to `en_us`, `zh_cn`, or `zh_tw`
2. Record the `Stored locale` shown on the page
3. Reboot the device
4. Check the startup language and `Stored locale` again

If the startup locale matches the stored value, the persistence flow is working correctly.

## Example Configuration Flow
This example depends on the following key configurations:

- `CONFIG_LV_EXT_RES_NON_STANDALONE=y`
- `CONFIG_RT_USING_DFS_ELMFAT=y`
- `CONFIG_BSP_SHARE_PREFS=y`
- `CONFIG_LV_USING_FREETYPE_ENGINE=y`

Their roles are:

- Enable the non-standalone external resource manager mode
- Enable the filesystem so external language-pack directories can be scanned
- Enable `share_prefs` to save the current locale
- Enable FreeType font support to improve multilingual glyph coverage

## Notes
- `lang_pack.h` is an application-side generated resource header, not a fixed hand-written header. It is typically located in `src/resource/strings/` under the build output directory
- If locale switching succeeds but text appears empty, corrupted, or as boxes, check font coverage before suspecting the language-pack switching logic
- The default external locale validated by this example is `zh_tw`; if you add Thai, Khmer, or other languages, you must also provide font support for those scripts
- After switching between `json` and `xlsx` source modes, a full rebuild is recommended to avoid stale generated files

## Troubleshooting
### Symptom: No external locale appears in the UI
Possible causes:

- No matching `.bin` exists under `disk/ex/resource/lang/installer/`
- The filesystem is not mounted correctly
- `RT_USING_DFS_ELMFAT` was not enabled during the build

### Symptom: Locale switch button is clickable, but text does not change
Possible causes:

- The current locale is not in the registered or installed language-pack list
- The external language pack does not match the key schema of builtin `lang_pack.h`

### Symptom: Locale switching succeeds, but characters display incorrectly
Possible causes:

- The current font does not support the character set of that language
- FreeType or font resources are not configured correctly
