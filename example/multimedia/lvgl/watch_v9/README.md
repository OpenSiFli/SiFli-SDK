# 手表界面
(有关示例及其用法的总体概述，请参阅上级 "examples" 目录中的 `README.md` 文件。)

源码路径: example/multimedia/lvgl/watch_v9

该示例使用LVGL v9实现智能手表界面，包含以下主要功能界面：
- 蜂窝主菜单
- 表盘
- 立方体左右旋转 (不支持SF32lb55x系列芯片)

## 用法

下面的小节仅提供绝对必要的信息。有关配置SiFli-SDK及使用其构建和运行项目的完整步骤，请参阅 [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)。

### 支持的开发板
```{note}
- 不支持520-hdk
```

### 硬件需求
本示例无需特殊硬件，可直接在支持的开发板上运行。

### 配置项目

#### 指定字体
参考`src/resource/fonts/SConscript`，通过添加使用字体的.c文件，进而在使用的地方extern声明

## 异常诊断
用户在使用此示例时可能会遇到以下兼容性问题：
- 在520-hdk开发板上运行会失败
- SF32lb55x系列芯片不支持立方体旋转功能

如有任何技术疑问，请在GitHub上提出 [issue](https://github.com/OpenSiFli/SiFli-SDK/issues)。
