# 手表界面

使用LVGL v9，包含的界面有：
- 蜂窝主菜单
- 表盘
- 立方体左右旋转 (不支持SF32lb55x系列芯片)

```{note}
- 不支持520-hdk
```

## 指定字体
参考`src/resource/fonts/SConscript`，通过添加使用字体的.c文件，进而在使用的地方extern声明
