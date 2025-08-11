# 手表界面

源码路径: example/multimedia/lvgl/watch
这是一个基于LVGL v8实现的智能手表界面示例，包含多种交互界面和字体配置功能。该示例展示了如何使用SiFli-SDK的LVGL图形库组件构建嵌入式设备的用户界面。
基于此示例，开发者可以构建各种智能穿戴设备的UI界面，如运动手表、健康监测设备等。

## 用法

## 支持的开发板

此示例可在以下开发板上运行：
- sf32lb52-lcd_n16r8
- sf32lb52-lchspi-ulp
```{note}
- 不支持520-hdk
```

### 硬件需求

- 支持LCD显示的SiFli开发板

## 指定字体

参考`src/resource/fonts/SConscript`，通过在CPPDEFINES中添加`FREETYPE_FONT_NAME`宏定义，可以注册对应TTF字体到LVGL中
```python
CPPDEFINES += ["FREETYPE_FONT_NAME={}".format(font_name)]
```
如果`font_name`是`DroidSansFallback`，相当于添加了如下宏定义
```c
#define FREETYPE_FONT_NAME   DroidSansFallback
```
编译时会在`freetype`子目录里查找以`.ttf`为后缀的字体文件，将其转换为C文件加入编译
```python
objs = Glob('freetype/{}.ttf'.format(font_name))
objs = Env.FontFile(objs)
```
`FREETYPE_TINY_FONT_FULL`这些宏是在工程目录下的`Kconfig.proj`中定义，类似下面这样
```kconfig
config FREETYPE_TINY_FONT_FULL
    bool
    default y
```

## 示例输出

成功运行后，开发板屏幕将显示手表主界面，包含蜂窝菜单和表盘显示。通过触摸或按键可以在不同界面间切换。

## 参考文档

- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- [LVGL官方文档](https://docs.lvgl.io/8.3/)
