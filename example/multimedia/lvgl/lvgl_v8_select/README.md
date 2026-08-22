# LVGL v8 Select 示例

源码路径: `SiFli-SDK/example/multimedia/lvgl/lvgl_v8_select`

## 概述

本例程演示 `lvsf_select` 选择列表控件：每一行在选中时显示"选中"图标、否则显示"未选中"图标。单选模式(`LV_SELECT_TYPE_SINGLE`)下同时只有一行被选中，点击某一行即把选中态移到该行。设好元素个数和尺寸、选中/未选中图片以及初始状态即可。本例程是 3 行单选，初始选中第一行。

要点：
- `lv_select_set_ele_num()` 设行数，`lv_select_set_ele_size()` 设每行尺寸(据此排布各行)。
- `lv_select_set_check_src()` / `lv_select_set_uncheck_src()` 分别设选中/未选中图标，`lv_select_set_ele_state()` 设某行的初始状态。
- select 构造时会清掉自身的 `LV_OBJ_FLAG_CLICKABLE`，需重新加上，点击才能下达到各行(行内置的点击回调会移动单选选中态)。

## 用法（关键 API）

```c
lv_obj_t *sel = lv_select_create(parent);
lv_obj_add_flag(sel, LV_OBJ_FLAG_CLICKABLE);   /* 构造会清掉, 点击需要它 */
lv_select_set_type(sel, LV_SELECT_TYPE_SINGLE); /* 单选 */
lv_select_set_ele_num(sel, 3);                  /* 行数 */
lv_select_set_ele_size(sel, 220, 40);           /* 每行尺寸 */
lv_select_set_check_src(sel, &check_img_dsc);   /* 选中图标 */
lv_select_set_uncheck_src(sel, &uncheck_img_dsc);/* 未选中图标 */
lv_select_set_ele_state(sel, 0, LV_SELECT_STATE_CHECK);  /* 第 0 行初始选中 */

/* 读回用户的选择(行点击会冒泡到 select, 可在 select 上挂回调读取): */
uint16_t idx = lv_select_get_select_idx(sel);            /* 单选: 当前选中行 */
/* 多选时逐行查: lv_select_get_ele_state(sel, i) == LV_SELECT_STATE_CHECK */
```

## 支持的开发板

- sf32lb52-lcd_n16r8
- sf32lb52-lchspi-ulp

## 编译和烧录

板子工程在 `project` 目录下，通过指定 board 编译对应板子的工程：
- 例如编译 sf32lb52-lcd_n16r8：在 `project` 目录执行 `scons --board=sf32lb52-lcd_n16r8 -j8`
- 烧录通过 build 目录下的 `download.bat`（或 `uart_download.bat`）进行
- SF32LB52x/SF32LB56x 系列会额外生成 `uart_download.bat`，执行后输入下载 UART 端口号即可

## 模拟器

在 `project` 目录执行 `scons --board=pc_hcpu -j8`，生成 `build_pc_hcpu/main.exe` 直接运行即可看到窗口。
- 需要先按本机 MSVC 配置改好 `SiFli-SDK/msvc_setup.bat`。

## 运行说明

例程启动后屏幕中央有 3 行条目，分别标着 `Option A/B/C`，每行右侧一个小图标：第一行绿色(选中)，另外两行深色(未选中)；下方一行 `Selected: Option A` 显示当前选择。**点击其它行**，绿色"选中"图标会移到被点的那一行，同时下方的 `Selected:` 文字实时更新成被选项 —— 体现了 app 读回用户选择并据此响应。

## 异常诊断

如有任何技术疑问，请在 GitHub 上提出 [issue](https://github.com/OpenSiFli/SiFli-SDK/issues)

## 参考文档
- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
