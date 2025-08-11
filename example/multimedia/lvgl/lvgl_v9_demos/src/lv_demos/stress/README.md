# 压力测试演示

## 概述

LVGL 的压力测试。
它包含大量对象创建、删除、动画、样式使用等。如果在大量使用期间出现内存损坏或任何内存泄漏，可以使用它。

![使用 LVGL 嵌入式 GUI 库的压力测试演示](screenshot1.gif)

## 运行演示
- 在 `lv_conf.h` 或等效位置设置 `LV_USE_DEMO_STRESS 1`
- 在 `lv_conf.h` 中启用所有控件（`LV_USE_BTN 1`）和动画（`LV_USE_ANIMATION 1`）
- 在 `lv_init()` 和初始化驱动程序后调用 `lv_demo_stress()`
