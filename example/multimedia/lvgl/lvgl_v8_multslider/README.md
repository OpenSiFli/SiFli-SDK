# LVGL v8 Multslider 示例

源码路径: `SiFli-SDK/example/multimedia/lvgl/lvgl_v8_multslider`

## 概述

本例程演示 `lvsf_multslider` 控件。它和原生 `lv_slider`/`lv_bar` 的区别在于：**圆形手柄上会显示读数** —— 默认把当前值格式化成数字画在手柄上，调用 `lv_multslider_set_txt()` 则改为显示一段固定短文本。控件可拖动，拖动时手柄和读数随之移动。

要点：
- `LV_PART_MAIN` 是轨道，`LV_PART_INDICATOR` 是填充，`LV_PART_KNOB` 是手柄 —— 三者都要设 `bg_color`+`bg_opa` 才会绘制；**特别是手柄不设样式就不画，手柄上的读数也跟着不显示**。
- 手柄上读数的颜色取自 `LV_PART_MAIN` 的 `text_color`。
- 手柄是直径约等于条高的圆，自定义文本要短才放得下。

## 用法（关键 API）

```c
lv_obj_t *ms = lv_multslider_create(parent);

/* 样式三个部件：MAIN=轨道, INDICATOR=填充, KNOB=手柄(必设, 否则手柄和读数都不画) */
lv_obj_set_style_bg_color(ms, lv_color_hex(0xD0D4DA), LV_PART_MAIN);       /* 轨道 */
lv_obj_set_style_bg_opa(ms, LV_OPA_COVER, LV_PART_MAIN);
lv_obj_set_style_bg_color(ms, lv_palette_main(LV_PALETTE_BLUE), LV_PART_INDICATOR); /* 填充 */
lv_obj_set_style_bg_opa(ms, LV_OPA_COVER, LV_PART_INDICATOR);
lv_obj_set_style_bg_color(ms, lv_color_white(), LV_PART_KNOB);             /* 手柄 */
lv_obj_set_style_bg_opa(ms, LV_OPA_COVER, LV_PART_KNOB);
lv_obj_set_style_text_color(ms, lv_color_hex(0x303030), LV_PART_MAIN);     /* 手柄读数颜色 */

lv_multslider_set_range(ms, 0, 100);
lv_multslider_set_value(ms, 60, LV_ANIM_OFF);  /* 手柄显示实时数值, 拖动跟随 */
/* lv_multslider_set_txt(ms, "Vol");           // 可选: 改为固定标签, 不随值变化 */
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

例程启动后在屏幕上显示两条 multslider：
- 第一条（蓝色填充）：默认模式，手柄上显示**实时当前值** `60`，鼠标/手指拖动改值时数字跟着变。
- 第二条（绿色填充）：通过 `lv_multslider_set_txt()` 在手柄上显示**固定标签** `Vol`。拖动时手柄会移动，但文本固定不变 —— `set_txt` 设的是静态文本，不随当前值变化。

## 异常诊断

如有任何技术疑问，请在 GitHub 上提出 [issue](https://github.com/OpenSiFli/SiFli-SDK/issues)

## 参考文档
- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
