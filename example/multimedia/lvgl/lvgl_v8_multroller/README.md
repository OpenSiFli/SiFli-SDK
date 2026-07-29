# LVGL v8 Multroller 示例

源码路径: `SiFli-SDK/example/multimedia/lvgl/lvgl_v8_multroller`

## 概述

本例程演示 `lvsf_multroller` 控件：一个简单、声明式的滚轮选择器(基于 `lvsf_multlist` 实现)。只需给它一个用 `'\n'` 分隔的选项字符串，它就排出一个**可循环、自动吸附**的列表，并在中间的焦点区把当前选中项高亮出来；用 `lv_multroller_get_selected()` 读回选中项。它和 `lvsf_mulroller` 互补：mulroller 是底层、回调驱动、可深度定制的轮子，而 multroller 只需设置选项/可见数量/焦点区即可，适合"给一组文字选项、选一个"的常见场景。本例程是一个月份选择器。

要点：
- 控件默认铺满全屏且背景不透明，所以要先 `lv_obj_set_size()` 设定大小、并把背景设为透明。
- 选项文字使用控件自身的字体与颜色样式，请在 `set_options()` **之前**用 `lv_obj_set_style_text_font()`/`set_style_text_color()` 设好；中间焦点区内的选中项会以高亮色显示。
- 调用顺序：设大小 + 字体/颜色 → `set_show_cnt()` → `set_options()` → `set_focus_param()`。
- 选项字符串按 `'\n'` 切分；**每个选项(包括最后一个)都要以 `'\n'` 结尾**，即字符串以一个空行收尾，否则最后一项会被丢掉。
- `set_show_cnt(cnt)` 设可见选项数(最小 3)；每项尺寸由控件大小按可见数量等分得到。
- `set_focus_param(color, w, h)` 在中间标出一个 `w×h` 的焦点区，区内的选项被高亮。控件默认开启循环，列表滚到末尾会绕回开头。
- `get_selected()` 返回居中(选中)项的索引；`set_selected(idx, anim_time)` 可编程选中(`anim_time` 为 0 即立即定位)。本例用一个小定时器轮询 `get_selected()`，把选中项同步到下方读数。
- 该控件基于 `lvsf_multlist` 并使用 app 框架内存，本例程的 `proj.conf` 已开启 `CONFIG_GUI_APP_FRAMEWORK`。

## 用法（关键 API）

```c
/* 选项字符串：按 '\n' 切分，末尾留一个空行确保最后一项也被收入 */
static const char *OPTIONS =
    "January\nFebruary\nMarch\nApril\nMay\nJune\n"
    "July\nAugust\nSeptember\nOctober\nNovember\nDecember\n\n";

lv_obj_t *roller = lv_multroller_create(parent);
lv_obj_set_size(roller, 240, 240);
lv_obj_set_style_bg_opa(roller, LV_OPA_TRANSP, 0);               /* 默认不透明，改透明 */
lv_obj_set_style_text_font(roller, &lv_font_montserrat_24, 0);   /* 选项字体 */
lv_obj_set_style_text_color(roller, lv_color_hex(0xBDBDBD), 0);  /* 未选中：灰 */

lv_multroller_set_show_cnt(roller, 5);                           /* 可见 5 项 */
lv_multroller_set_options(roller, OPTIONS);
lv_multroller_set_focus_param(roller, lv_palette_main(LV_PALETTE_RED), 240, 48); /* 中间焦点区高亮 */

/* 读取/设置选中项 */
uint16_t sel = lv_multroller_get_selected(roller);      /* 当前居中项索引 */
lv_multroller_set_selected(roller, 3, 200);             /* 选中第 3 项，200ms 动画定位 */
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

例程启动后屏幕中央是一个月份滚轮，一次显示 5 个月份，居中那项以高亮色(红)显示、其余为灰。用鼠标(模拟器)或手指(板子)上下拖动滚动：列表循环(December 之后绕回 January)，松手后吸附到最近一项。下方读数 `selected: <月份>` 跟随当前选中项更新。

## 异常诊断

如有任何技术疑问，请在 GitHub 上提出 [issue](https://github.com/OpenSiFli/SiFli-SDK/issues)

## 参考文档
- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
