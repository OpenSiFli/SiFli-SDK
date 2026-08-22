# LVGL v8 Basechart Example

Source path: `SiFli-SDK/example/multimedia/lvgl/lvgl_v8_basechart`

## Overview

This example demonstrates the `lvsf_basechart` widget, used to plot one or more data series (line/bar). Set the chart type and point count, a value range and grid, add a series (with a color), push values into it, then refresh. This example draws two line series.

Notes:
- Part styles: `LV_PART_ITEMS` is the series line (`line_width`), `LV_PART_INDICATOR` is the point markers (`width`/`height`).
- Each series is created by `lv_basechart_add_series()` which returns a handle; values are pushed point by point with `lv_basechart_set_next_value()`.
- basechart is built on the LVGL chart, so enable `CONFIG_LV_USE_CHART=y` (already set in this example's `proj.conf`).

## Usage (key API)

```c
lv_obj_t *chart = lv_basechart_create(parent);
lv_obj_set_size(chart, 280, 180);
lv_obj_set_style_line_width(chart, 3, LV_PART_ITEMS);    /* series line width */
lv_obj_set_style_width(chart, 5, LV_PART_INDICATOR);     /* point marker size */
lv_obj_set_style_height(chart, 5, LV_PART_INDICATOR);

lv_basechart_set_type(chart, LV_CHART_TYPE_LINE);        /* line; also BAR/SCATTER */
lv_basechart_set_point_count(chart, 10);                 /* points per series */
lv_basechart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, 0, 100);
lv_basechart_set_div_line_count(chart, 5, 6);            /* grid rows, cols */

lv_chart_series_t *s = lv_basechart_add_series(chart, lv_palette_main(LV_PALETTE_RED),
                                               LV_CHART_AXIS_PRIMARY_Y);
for (int i = 0; i < 10; i++)
    lv_basechart_set_next_value(chart, s, vals[i]);      /* push values point by point */
lv_basechart_refresh(chart);
```

## Supported Boards

- sf32lb52-lcd_n16r8
- sf32lb52-lchspi-ulp

## Build and Download

The board project lives under `project`. Build for a specific board by passing its name:
- e.g. for sf32lb52-lcd_n16r8: run `scons --board=sf32lb52-lcd_n16r8 -j8` in `project`
- Download via `download.bat` (or `uart_download.bat`) under the build directory
- The SF32LB52x/SF32LB56x series also generate `uart_download.bat`; run it and enter the download UART port

## Simulator

Run `scons --board=pc_hcpu -j8` in `project` to produce `build_pc_hcpu/main.exe`, then run it to open the window.
- First adjust `SiFli-SDK/msvc_setup.bat` to match your local MSVC installation.

## Run Description

On startup a white chart is shown in the center: with grid lines and **two line series** (red, blue), each with 10 data points, on a 0–100 Y range.

## Troubleshooting

For technical questions, please open an [issue](https://github.com/OpenSiFli/SiFli-SDK/issues) on GitHub.

## References
- [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
