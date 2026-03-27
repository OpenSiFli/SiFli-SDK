# Watch Interface

Using LVGL v8, the included interfaces are:

## Usage

## Supported Boards

This example can be executed on the following development boards:
- SF32LB52-LCD Series
- sf32lb56-lcd series
- sf32lb58-lcd series
- sf32lb52-lchspi-ulp

```{note}
- 520-HDK is not supported
```

### Hardware Requirements

- SiFli development boards with LCD support

## 1. Specifying Fonts

For example, if `font_name` is set to `DroidSansFallback`, it is equivalent to
adding the following macro definition:
```c
#define FREETYPE_FONT_NAME   DroidSansFallback
```

During compilation, the system searches the `freetype` subdirectory for font
files with the `.ttf` extension and converts them into C files for inclusion in
the build process:
```python
objs = Glob('freetype/{}.ttf'.format(font_name))
objs = Env.FontFile(objs)
```

Once compilation is complete, the TTF file is converted into a C file located in
the `build_xxx_hcpu/src/resource/fonts/freetype` directory with the filename
`{font_name}.c`. This C file invokes a font registration macro to register the
font with LVGL, making it available for use:
```c
LVSF_FREETYPE_FONT_REGISTER(tiny55_full)
```

Macros such as `FREETYPE_TINY_FONT_FULL` are defined in the `Kconfig.proj` file
within the project directory as follows:
```kconfig
config FREETYPE_TINY_FONT_FULL
    bool
    default y
```

## 2. Complete `src/resource/fonts/SConscript` Example (supporting multiple custom fonts)

```python
CPPDEFINES = []

font_name = ''  # Default font option, defined in [Kconfig.proj] under the project directory.
font_name2 = 'SourceHanSansCN_Normal'  # Custom TTF font filename. Place the file in the `src/resource/fonts/freetype` directory.

if GetDepend('FREETYPE_TINY_FONT_FULL'):
    font_name = 'tiny55_full'
elif GetDepend('FREETYPE_TINY_FONT_LITE'):
    font_name = 'tiny55_lite'
elif GetDepend('FREETYPE_HANSANS_FONT'):
    font_name = 'SourceHanSansCN_Normal'
elif GetDepend('FREETYPE_ARIAL_FONT'):
    font_name = 'arial'
else:
    font_name = 'DroidSansFallback'

objs = Glob('freetype/{}.ttf'.format(font_name))
objs = Env.FontFile(objs)

objs2 = Glob('freetype/{}.ttf'.format(font_name2))  # Locate the custom font file.
objs += Env.FontFile(objs2)  # Convert the custom TTF file to a C source file.
```

## Function Usage
```c
// Interface for using the default configured font (font_name)
void lv_ext_set_local_font(lv_obj_t *obj, uint16_t size, lv_color_t color)

// Interface for using a custom font by specifying the registered font name
void lv_ext_set_font_local_by_name(lv_obj_t *obj, uint16_t size, lv_color_t color, char *fontname)
```
## Troubleshooting
When multiple fonts are used, the total size of the TTF files may exceed the
pre-allocated memory capacity. If a compilation error such as the one shown
below occurs, you must increase the `max_size` for `"tags":
["HCPU_FLASH2_FONT"]` in `project/xxx_hcpu/ptab.json`. Note that when adjusting
this size, ensure that the addresses of adjacent segments remain contiguous and
consider the maximum memory capacity supported by the specific development
board.
```
region 'ROM' overflowed by 7880732 bytes
```
## Sample Output

Upon successful execution, the development board's screen will display the main
smartwatch interface, including a honeycomb menu and watch face. Users can
navigate between different interfaces via touch or physical buttons.

## References

- [SiFli-SDK Quick Start
  Guided](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- [LVGL Official Documentation](https://docs.lvgl.io/8.3/)

