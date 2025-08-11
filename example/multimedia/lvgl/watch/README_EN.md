# Watch Interface

Using LVGL v8, the included interfaces are:
- Honeycomb main menu
- Watch face
- Cube left-right rotation (not supported on SF32lb55x series chips)
```{note}
- Not supported on 520-hdk
```

# Watch Interface

Source Code Path: example/multimedia/lvgl/watch
This is a smartwatch interface example implemented based on LVGL v8, featuring various interactive interfaces and font configuration capabilities. It demonstrates how to use the LVGL graphics library component of SiFli-SDK to build user interfaces for embedded devices.
Developers can use this example as a foundation to build UI interfaces for various smart wearable devices such as sports watches and health monitoring devices.

## Supported Boards

This example can run on the following development boards:
- sf32lb52-lcd_n16r8
- sf32lb52-lchspi-ulp
```{note}
- Not supported on 520-hdk
```

### Hardware Requirements

- SiFli development board with LCD display support

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

## Example Output

After successful execution, the development board screen will display the watch main interface, including a honeycomb menu and watch face display. You can switch between different interfaces via touch or buttons.

## Reference Documentation

- [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- [LVGL Official Documentation](https://docs.lvgl.io/8.3/)