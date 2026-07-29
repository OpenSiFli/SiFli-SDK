# LVGL v8 Basechart 示例

源码路径: `SiFli-SDK/example/multimedia/lvgl/lvgl_v8_basechart`

## 概述

本例程演示 `lvsf_basechart` 图表控件，用来绘制一条或多条数据序列(折线/柱状)。先设图表类型和点数、数值范围和网格，再添加序列(指定颜色)、往序列里推入数值，最后刷新即可。本例程画两条折线序列。

要点：
- 部件样式：`LV_PART_ITEMS` 是序列线条(`line_width`)，`LV_PART_INDICATOR` 是数据点标记(`width`/`height`)。
- 每条序列由 `lv_basechart_add_series()` 创建并返回句柄，数值通过 `lv_basechart_set_next_value()` 逐点推入。
- basechart 基于 LVGL chart，需在配置中开启 `CONFIG_LV_USE_CHART=y`(本例程 `proj.conf` 已开)。

## 用法（关键 API）

```c
lv_obj_t *chart = lv_basechart_create(parent);
lv_obj_set_size(chart, 280, 180);
lv_obj_set_style_line_width(chart, 3, LV_PART_ITEMS);    /* 序列线宽 */
lv_obj_set_style_width(chart, 5, LV_PART_INDICATOR);     /* 数据点大小 */
lv_obj_set_style_height(chart, 5, LV_PART_INDICATOR);

lv_basechart_set_type(chart, LV_CHART_TYPE_LINE);        /* 折线; 也可 BAR/SCATTER */
lv_basechart_set_point_count(chart, 10);                 /* 每序列点数 */
lv_basechart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, 0, 100);
lv_basechart_set_div_line_count(chart, 5, 6);            /* 网格行列数 */

lv_chart_series_t *s = lv_basechart_add_series(chart, lv_palette_main(LV_PALETTE_RED),
                                               LV_CHART_AXIS_PRIMARY_Y);
for (int i = 0; i < 10; i++)
    lv_basechart_set_next_value(chart, s, vals[i]);      /* 逐点推入数值 */
lv_basechart_refresh(chart);
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

例程启动后屏幕中央显示一个白底图表：带网格线，上面是**两条折线**(红、蓝)，每条 10 个数据点，纵轴范围 0~100。

## 异常诊断

如有任何技术疑问，请在 GitHub 上提出 [issue](https://github.com/OpenSiFli/SiFli-SDK/issues)

## 参考文档
- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
