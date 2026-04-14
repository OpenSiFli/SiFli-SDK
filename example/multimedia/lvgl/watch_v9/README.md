# 手表界面v9

源码路径: example/multimedia/lvgl/watch_v9

该示例使用LVGL v9实现智能手表界面，包含以下主要功能界面：
- 蜂窝主菜单
- 表盘
- 立方体左右旋转 (不支持SF32lb55x系列芯片)

## 用法

下面的小节仅提供绝对必要的信息。有关配置SiFli-SDK及使用其构建和运行项目的完整步骤，请参阅 [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)。

### 支持的开发板
+ sf32lb52-lcd系列
+ sf32lb56-lcd系列
+ sf32lb58-lcd系列

```{note}
+ 不支持520-hdk
```

## 自定义指定字体
通过在resource\fonts中使用`SConscript`文件，将字体文件转换成C文件，并添加到编译中:
- 将`.ttf`后缀的字体文件放在`resource/fonts`目录下
- Env.ConvertFont(font_file, 'DroidSansFallback')
    - `font_file`: ttf文件路径
    - `DroidSansFallback`: C文件名


## 一次性转多ttf文件为C文件

在SConscript中添加如下代码：

```python
font_objs1 = Env.ConvertFont(font_file1, 'DroidSansFallback')
font_objs2 = Env.ConvertFont(font_file2, 'DroidSansFallback_other_name')
font_objs = font_objs1 + font_objs2
```
