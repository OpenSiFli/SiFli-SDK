# Watch Interface

Using LVGL v8, the included interfaces are:
- Honeycomb main menu
- Watch face
- Cube left-right rotation (not supported on SF32lb55x series chips)

```{note}
- Not supported on 520-hdk
```

## Specifying Fonts
Refer to `src/resource/fonts/SConscript`. By adding the `FREETYPE_FONT_NAME` macro definition in CPPDEFINES, you can register the corresponding TTF font to LVGL
```python
CPPDEFINES += ["FREETYPE_FONT_NAME={}".format(font_name)]
```

If `font_name` is `DroidSansFallback`, it is equivalent to adding the following macro definition
```c
#define FREETYPE_FONT_NAME   DroidSansFallback
```

During compilation, it will search for font files with `.ttf` suffix in the `freetype` subdirectory and convert them to C files for compilation

```python
objs = Glob('freetype/{}.ttf'.format(font_name))
objs = Env.FontFile(objs)
```

Macros like `FREETYPE_TINY_FONT_FULL` are defined in `Kconfig.proj` in the project directory, similar to the following:

```kconfig
config FREETYPE_TINY_FONT_FULL
    bool
    default y
```