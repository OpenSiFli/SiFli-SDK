# 手表界面

使用LVGL v8，包含的界面有：
- 蜂窝主菜单
- 表盘
- 立方体左右旋转 (不支持SF32lb55x系列芯片)

```{note}
- 不支持520-hdk
```

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
