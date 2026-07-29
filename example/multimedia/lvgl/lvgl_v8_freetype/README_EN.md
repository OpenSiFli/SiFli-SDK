# LVGL v8 FreeType Example

Source path: example/multimedia/lvgl/lvgl_v8_freetype

## Overview
This example shows how to use LVGL's FreeType font engine on RT-Thread to read TTF fonts from the file system and switch them at runtime. It supports:
- Loading a default font at boot, falling back to the built-in font when it is missing or broken
- Scanning the font directory, listing the available fonts and switching on a tap
- Adjusting the font size from 12 to 60, scrolling the preview when the text no longer fits
- Validating a font as it is loaded: an oversized font or an unsupported format is reported and the UI keeps its current font
- Keeping a single font resident: the previous font is released once the switch completes, so memory does not grow with the number of switches

## Features
- **Fonts from the file system**: fonts are flashed as files instead of being converted to C arrays, so replacing a font does not mean rebuilding the firmware
- **Switching at runtime**: both the font and its size change without a restart
- **Resource guard**: creating a font object is gated by a minimum-free-heap floor, so a font cannot load successfully only for the system to die later
- **Safe unloading**: a font that is still referenced by the UI is kept instead of being freed
- **Shell control**: finsh commands to restart the example, verify and enable a font

## Usage

### Supported Platforms
The example has been verified on:
* sf32lb52-lchspi-ulp

Other boards need two things: an `FS_REGION` partition in the partition table (the example mounts it as the root directory), and enough system heap for the fonts in use (see [Notes](#notes)).

### Building and Flashing
Switch to the project directory of the example and run scons to build:
```
scons --board=sf32lb52-lchspi-ulp -j8
```
Replace `--board` for any other supported board.

Run the download script:
```
build_sf32lb52-lchspi-ulp_hcpu\uart_download.bat
```
Select the serial port when prompted:
```none
please input the serial port num:5
```

```{note}
The fonts are flashed as part of the file system image `fs_root.bin`. After changing anything under `disk/`, rebuild and flash **the full image** - flashing `main.bin` alone will not update the fonts on the board.
```

### Preparing Font Files
Everything under `disk/` is packed into the file system image, and the fonts under `disk/font/` are the ones available on the board. The example ships with:

| Font file | Description |
|---|---|
| `DroidSansFallback_Simplified.ttf` | Default font, ~1.0 MB after subsetting, covers all GB2312 hanzi and the digits |
| `A-SourceHanSansCN-M_Simplified.ttf` | Source Han Sans, ~2.0 MB after subsetting and conversion to TrueType outlines |
| `FontSwitchDemoCN.ttf` | Small test font with few glyphs, useful to see the fallback for missing characters |

Any `.ttf` / `.otf` file placed in `disk/font/` shows up in the font list without a code change. To change the default font, edit the macro in `src/font_switch_demo.c`:
```c
#define FONT_DEMO_DEFAULT_FONT "/font/DroidSansFallback_Simplified.ttf"
```

Two limits of this SDK matter when preparing your own font:

* **TrueType outlines only**. The FreeType build registers the TrueType driver alone, so an OTF with CFF outlines cannot be opened (the serial log says `unsupported outline format`). Convert it to TrueType outlines with [fontTools](https://github.com/fonttools/fonttools).
* **Full CJK fonts must be subset**. An open font keeps its glyph index tables resident, and those tables alone take hundreds of KB for a font with fifty thousand glyphs - more than a small board can spare. Subset the font before putting it in `disk/font/`:
  ```
  pip install fonttools
  pyftsubset YourFont.ttf --output-file=YourFont_Simplified.ttf \
      --unicodes=U+0020-007E,U+00A0-00FF,U+2000-206F,U+3000-303F,U+FF00-FFEF \
      --text-file=gb2312.txt --drop-tables+=GSUB,GPOS,GDEF
  ```
  where `gb2312.txt` lists the hanzi to keep.

### Key Configuration
The options below are already set in `project/proj.conf`; carry them over when porting the code into your own project:

| Option | Value | Description |
|---|---|---|
| `CONFIG_LV_USING_FREETYPE_ENGINE` | y | Enable the FreeType font engine |
| `CONFIG_FREETYPE_FONT_BPP_8` | y | Render glyphs as 8bpp gray bitmaps |
| `CONFIG_LV_FREETYPE_CACHE_FT_FACES` | 4 | Maximum number of font files open at once |
| `CONFIG_LV_FREETYPE_CACHE_FT_SIZES` | 16 | Maximum number of cached sizes |
| `CONFIG_LVSF_FONT_MIN_FREE_HEAP` | 32768 | System heap that must remain after creating a font object; below this the font is refused |
| `CONFIG_RT_USING_DFS_ELMFAT` | y | Fonts are read from a FAT file system |

## Example Output
When the example runs successfully, the serial output looks like this:
```
(...system initialization omitted...)

[lvsf_font] freetype init cache=64000 faces=4 sizes=16
mount fs on flash to root success
[font_demo] start
[font_demo] load: path=/font/DroidSansFallback_Simplified.ttf name=DroidSansFallback_Simplified
[lvsf_font] create ok name=DroidSansFallback_Simplified size=24 font=0x200498ac dsc=0x200498ec
[font_demo] enable/order ok: DroidSansFallback_Simplified
[font_demo] mem demo ready total=343704 used=183540 free=160164 max=183540
[font_demo] ready
```
The LCD shows the preview text, the current font name and size, with the size buttons and the font list button at the bottom.

Selecting another font prints the switch and the release of the previous one:
```
[font_demo] list select: /font/A-SourceHanSansCN-M_Simplified.ttf
[lvsf_font] create ok name=A-SourceHanSansCN-M_Simplified size=24 ...
[font_demo] enable/order ok: A-SourceHanSansCN-M_Simplified
[font_demo] mem after unload previous font total=343704 used=... free=...
```

## Using finsh Commands

| Command | Description |
|---|---|
| `lv_example_font_switch_demo` | Restart the example (rebuild the UI and release the font of the previous run) |
| `font_demo_enable_font` | Enable the selected font and move it to the front of the priority list |
| `font_demo_verify_font` | Create a font object at the current size to check that the font is usable |

```{note}
These commands only post a request from the finsh thread; the actual UI and font work runs in the LVGL thread from a timer owned by the example. LVGL is not thread-safe, and applications should do the same.
```

## API Reference

### lvsf_font_load_ex
```c
int lvsf_font_load_ex(char *font_path, uint16_t *size);
```
Register a font from the file system and create its font objects. With `size` set to `NULL` one object is created at the default size, which validates the file; alternatively pass a zero-terminated array of sizes to create several at once.

**Return**: 1 on success, -1 on failure (file cannot be opened, unsupported format, or not enough memory).

### lvsf_get_font_by_name
```c
lv_font_t *lvsf_get_font_by_name(char *font_name, int size);
```
Get the `lv_font_t` of a font (named after the file without its extension) at the given size, ready to be set on an LVGL style. Size objects are created on demand.

**Return**: the font pointer on success, `NULL` on failure - fall back to another font in that case.

### lvsf_font_set_enable / lvsf_font_set_order
```c
int  lvsf_font_set_enable(char *font_name, int enable);
void lvsf_font_set_order(char **font_name, uint16_t font_num);
```
Enable or disable a font and set the priority used when a font is looked up by size.

### lvsf_font_unload_ex
```c
int lvsf_font_unload_ex(char *font_path);
```
Unload the font registered for this path, releasing its font objects and the memory they keep resident.

**Parameter**: `font_path` - path of the font file

**Return**: 0 when everything is released, -1 when a font is still displayed and had to be kept

```{warning}
Re-point every style that selects the font to another font first (this example gives the UI back to the built-in font before unloading). A font that is still displayed is kept and reported on the serial port instead of being freed.
```

## Notes
1. **Re-point before unloading**: the font manager hands out raw pointers that the application stores in LVGL styles, so unloading a font that is still displayed means using freed memory. The SDK checks for references and refuses to free such a font, but re-pointing remains the application's job.
2. **Set a font memory floor for the board**: an open font keeps its index tables resident. With `CONFIG_LVSF_FONT_MIN_FREE_HEAP` set, creating a font fails outright when the remaining heap would fall below the floor, so the application can fall back to another font instead of dying later on an unrelated allocation.
3. **Font format**: TrueType outlines only; an OTF with CFF outlines has to be converted first.
4. **File system size**: the total size of `disk/` is bounded by the `FS_REGION` partition (4 MB by default); packing the image fails when it does not fit.
5. **Sizes and memory**: every size creates a font object and fills the glyph cache, so browsing many sizes raises the heap watermark noticeably.

## Troubleshooting
* **`Default font not found` on the serial port**: the default font is missing from `disk/font/`, or `fs_root.bin` was not updated when flashing.
* **`unsupported outline format: convert CFF/OTF fonts to TrueType`**: the font has CFF outlines and must be converted to TrueType outlines.
* **`leaves less than 32768 bytes of heap, refused`**: the font is too large for this board - subset it, or move to a board with PSRAM.
* **The old font is still shown after switching**: check for two fonts with the same name (identical once the extension is stripped); the manager refuses the duplicate registration and reports it on the serial port.
* **Some characters render as boxes or noticeably smaller**: they are not in the character set of the current font and fall back to the built-in font - include them when subsetting.

For any other problem, please open an [issue](https://github.com/OpenSiFli/SiFli-SDK/issues) on GitHub.

## References
- [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- [SiFli-SDK Development Guide](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/development/index.html)
- [LVGL Documentation](https://docs.lvgl.io/)
- [FreeType Documentation](https://freetype.org/freetype2/docs/documentation.html)
