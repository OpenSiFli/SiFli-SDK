# 键盘和编码器演示

## 概述

LVGL允许您使用键盘和/或编码器控制小部件，而无需触摸板。
此演示显示如何在没有触摸板的情况下处理按钮、下拉列表、滚轮、滑块、开关和文本输入。
了解更多关于LVGL无触摸板使用的信息[这里](https://docs.lvgl.io/master/overview/indev.html#keypad-and-encoder)。

![LVGL嵌入式GUI库中的键盘和编码器导航](screenshot1.gif)

## 运行演示
- 在 `lv_conf.h` 或等效位置设置 `LV_USE_DEMO_KEYPAD_AND_ENCODER 1`
- 在 `lv_init()` 和初始化驱动程序后调用 `lv_demo_keypad_encoder()`
