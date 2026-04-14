# 手表界面v8

源码路径: example/multimedia/lvgl/watch
这是一个基于LVGL v8实现的智能手表界面示例，包含多种交互界面和字体配置功能。该示例展示了如何使用SiFli-SDK的LVGL图形库组件构建嵌入式设备的用户界面。
基于此示例，开发者可以构建各种智能穿戴设备的UI界面，如运动手表、健康监测设备等。

## 用法

## 支持的开发板

此示例可在以下开发板上运行：
- sf32lb52-lcd系列
- sf32lb56-lcd系列
- sf32lb58-lcd系列
- sf32lb52-lchspi-ulp

```{note}
- 不支持520-hdk
```

### 硬件需求

- 支持LCD显示的SiFli开发板

## 1. 指定字体

例如当 `font_name` 是 `DroidSansFallback` 时，则相当于添加了如下宏定义：
```c
#define FREETYPE_FONT_NAME   DroidSansFallback
```

编译时会在 `freetype` 子目录里查找以 `.ttf` 为后缀的字体文件，并将其转换为 C 文件加入编译过程：
```python
objs = Glob('freetype/{}.ttf'.format(font_name))
objs = Env.FontFile(objs)
```

编译完成后，TTF 文件会被转换成 C 文件，生成于 `build_xxx_hcpu/src/resource/fonts/freetype` 目录下，文件名为 `{font_name}.c`。该 C 文件中调用了字体注册宏，将字体注册进 LVGL，从而可以在 LVGL 中使用：
```c
LVSF_FREETYPE_FONT_REGISTER(tiny55_full)
```

像 `FREETYPE_TINY_FONT_FULL` 这样的宏是在工程目录下的 `Kconfig.proj` 中定义的，形式如下：
```kconfig
config FREETYPE_TINY_FONT_FULL
    bool
    default y
```

## 2. 完整 `src/resource/fonts/SConscript` 实例（支持自定义多字体使用）

```python
CPPDEFINES = []

font_name = ''  # 默认字体选项，根据工程目录下的 [Kconfig.proj] 中定义
font_name2 = 'SourceHanSansCN_Normal'  # 自定义 TTF 字体文件名，需放置在 `src/resource/fonts/freetype` 目录下

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

objs2 = Glob('freetype/{}.ttf'.format(font_name2))  # 寻找自定义字体文件
objs += Env.FontFile(objs2)  # 将自定义 TTF 文件转为 C 文件
```

## 函数使用
```c
//使用默认配置font_name字体接口
void lv_ext_set_local_font(lv_obj_t *obj, uint16_t size, lv_color_t color)

//根据配置注册字体名称，使用自定义字体接口
void lv_ext_set_font_local_by_name(lv_obj_t *obj, uint16_t size, lv_color_t color, char *fontname)

```
## 异常诊断
在多字体使用时，ttf文件总和可能过大，超出事先预准备的内存大小，例如以下编译报错，此时需要修改`project/xxx_hcpu/ptab.json`中 `"tags": ["HCPU_FLASH2_FONT"]`的 `max_size` 大小，注意修改大小需要保证前后段的地址连续，另外还需考虑不同开发板所支持的内存大小。
```
region'ROM' overflowed by 7880732 bytes
```
## 示例输出

成功运行后，开发板屏幕将显示手表主界面，包含蜂窝菜单和表盘显示。通过触摸或按键可以在不同界面间切换。

## 参考文档

- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- [LVGL官方文档](https://docs.lvgl.io/8.3/)

