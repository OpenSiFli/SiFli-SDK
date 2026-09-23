# LVGL v8 基础图表（Basechart）

`lvsf_basechart` 是 SiFli 基于 LVGL v8 原生 `lv_chart` 封装的自定义图表控件，用于在屏幕上绘制一条或多条数据序列（折线、柱状、散点等）。它在保留 LVGL chart 全部常用接口的同时，扩展了柱状图颜色分区、多端点柱状图和散点柱状等定制绘制能力。

- 主要头文件：_middleware/lvgl/lvsf/gui_widgets/lvsf_basechart.h_
- 依赖控件：LVGL chart（`LV_USE_CHART`）
- 示例工程：_example/multimedia/lvgl/lvgl_v8_basechart_

```{note}
`lvsf_basechart` 继承自 `lv_chart_t`（结构体首成员为 `lv_chart_t bg`），因此除下文列出的封装接口外，控件仍遵循 LVGL chart 的样式与事件模型。`LV_PART_ITEMS` 对应序列线条，`LV_PART_INDICATOR` 对应数据点标记。
```

## 功能简介

- 支持折线（LINE）、柱状（BAR）、散点（SCATTER）等多种图表类型，接口与 LVGL `lv_chart` 一一对应。
- 支持多条数据序列，每条序列可单独指定颜色、坐标轴、显示/隐藏状态。
- 支持柱状图颜色分区：按数值区间映射不同颜色（`lv_basechart_color_t`）。
- 支持三种柱状图形态：普通柱状、双端点柱状（每柱两个数据）、散点柱状（相连数据点合并为一个柱子）。
- 支持光标（cursor）定位、轴刻度、X/Y 轴缩放、网格分割线等常规图表能力。
- 支持定时器定时刷新（`lv_basechart_refresh_timer()`），便于滚动刷新的实时数据展示。

## 使用场景

- 实时数据曲线展示，如心率、电量、传感器波形。
- 柱状统计图表，如电量分段、数值区间分布。
- 需要按数值区间着色的仪表盘、健康数据图表。
- 滚动更新的流式数据序列（配合 `set_next_value` 与定时器刷新）。

## 支持的开发板

- sf32lb52-lcd_n16r8
- sf32lb52-lchspi-ulp

## 配置与初始化

`lvsf_basechart` 依赖 LVGL chart 组件。在 menuconfig 中确认已开启图表控件：

```none
CONFIG_LV_USE_CHART=y
```

BSP 中通过 `LVSF_USE_BASECHART` 宏控制该控件的编译（位于 lvsf 配置）。示例工程在 `proj.conf` 中已默认开启。

创建对象并配置的最小流程：

```c
#include "lvsf_basechart.h"

lv_obj_t *chart = lv_basechart_create(lv_scr_act());
lv_obj_set_size(chart, 280, 180);
```

## API 说明

### 控件创建与类型扩展

| 接口函数 | 功能说明 | 参数/返回值说明 |
| --- | --- | --- |
| `lv_obj_t *lv_basechart_create(lv_obj_t *parent)` | 创建基础图表对象并初始化 | `parent`：父对象指针；成功返回图表对象指针，失败返回 `NULL` |
| `void lv_basechart_refresh_timer(lv_timer_t *timer)` | 图表刷新定时器回调 | `timer`：定时器对象指针；由定时器定期调用，用于更新图表内容 |
| `void lv_basechart_set_column_type(lv_obj_t *chart, lv_basechart_column_type_t column_type, uint8_t column_mult_num)` | 设置柱状图类型 | `chart`：图表对象；`column_type`：柱状图类型（`lv_basechart_column_type_t`）；`column_mult_num`：每个柱子的最大数据数量（用于 `LV_BASECHART_COLUMN_TYPE_MULT` 类型） |
| `void lv_basechart_set_color(lv_obj_t *chart, uint8_t num, lv_basechart_color_t *color)` | 设置颜色分区 | `chart`：图表对象；`num`：颜色分区数量；`color`：颜色分区数组指针，不同值范围显示不同颜色 |
| `void lv_basechart_add_simple_multiple(lv_obj_t *chart, uint8_t simple_multiple)` | 添加简单倍数 | `chart`：图表对象；`simple_multiple`：每条数据源的数据取值间隔 |

柱状图类型枚举（`lv_basechart_column_type_t`）：

| 取值 | 含义 |
| --- | --- |
| `LV_BASECHART_COLUMN_TYPE_COMM` | 正常的柱状图 |
| `LV_BASECHART_COLUMN_TYPE_DOUBLE` | 有两个端点的柱状图，每个柱子需要两个数据 |
| `LV_BASECHART_COLUMN_TYPE_MULT` | 柱状图上的散点，相连数据点连成一个柱子 |

### 图表属性设置（封装自 lv_chart）

| 接口函数 | 功能说明 | 参数/返回值说明 |
| --- | --- | --- |
| `void lv_basechart_set_type(lv_obj_t *obj, lv_chart_type_t type)` | 设置图表类型（折线/柱状等） | `obj`：图表对象；`type`：图表类型 |
| `void lv_basechart_set_point_count(lv_obj_t *obj, uint16_t cnt)` | 设置每个数据系列的点数量 | `cnt`：点的数量 |
| `void lv_basechart_set_range(lv_obj_t *obj, lv_chart_axis_t axis, lv_coord_t min, lv_coord_t max)` | 设置坐标轴范围 | `axis`：X 或 Y 轴；`min`/`max`：最小/最大值 |
| `void lv_basechart_set_update_mode(lv_obj_t *obj, lv_chart_update_mode_t update_mode)` | 设置图表更新模式 | `update_mode`：更新模式 |
| `void lv_basechart_set_div_line_count(lv_obj_t *obj, uint8_t hdiv, uint8_t vdiv)` | 设置网格分割线数量 | `hdiv`/`vdiv`：水平/垂直分割线数量 |
| `void lv_basechart_set_zoom_x(lv_obj_t *obj, uint16_t zoom_x)` | 设置 X 轴缩放比例 | `zoom_x`：X 轴缩放比例 |
| `void lv_basechart_set_zoom_y(lv_obj_t *obj, uint16_t zoom_y)` | 设置 Y 轴缩放比例 | `zoom_y`：Y 轴缩放比例 |
| `uint16_t lv_basechart_get_zoom_x(const lv_obj_t *obj)` | 获取 X 轴缩放比例 | 返回 X 轴缩放比例 |
| `uint16_t lv_basechart_get_zoom_y(const lv_obj_t *obj)` | 获取 Y 轴缩放比例 | 返回 Y 轴缩放比例 |
| `void lv_basechart_set_axis_tick(lv_obj_t *obj, lv_chart_axis_t axis, lv_coord_t major_len, lv_coord_t minor_len, lv_coord_t major_cnt, lv_coord_t minor_cnt, bool label_en, lv_coord_t draw_size)` | 设置轴刻度 | `axis`：坐标轴；`major_len`/`minor_len`：主/次刻度长度；`major_cnt`/`minor_cnt`：主/次刻度数量；`label_en`：是否显示标签；`draw_size`：绘制大小 |
| `lv_chart_type_t lv_basechart_get_type(const lv_obj_t *obj)` | 获取图表类型 | 返回图表类型 |
| `uint16_t lv_basechart_get_point_count(const lv_obj_t *obj)` | 获取点数量 | 返回点数量 |
| `uint16_t lv_basechart_get_x_start_point(const lv_obj_t *obj, lv_chart_series_t *ser)` | 获取 X 轴起始点索引 | `ser`：数据系列；返回起始点索引 |
| `void lv_basechart_get_point_pos_by_id(lv_obj_t *obj, lv_chart_series_t *ser, uint16_t id, lv_point_t *p_out)` | 获取指定 ID 点的坐标位置 | `id`：点 ID；`p_out`：输出坐标 |
| `void lv_basechart_refresh(lv_obj_t *obj)` | 强制刷新图表显示 | 无返回值 |

### 数据系列

| 接口函数 | 功能说明 | 参数/返回值说明 |
| --- | --- | --- |
| `lv_chart_series_t *lv_basechart_add_series(lv_obj_t *obj, lv_color_t color, lv_chart_axis_t axis)` | 添加数据系列 | `color`：系列颜色；`axis`：坐标轴；返回数据系列指针 |
| `void lv_basechart_remove_series(lv_obj_t *obj, lv_chart_series_t *series)` | 删除数据系列 | `series`：要删除的数据系列 |
| `void lv_basechart_hide_series(lv_obj_t *chart, lv_chart_series_t *series, bool hide)` | 隐藏/显示数据系列 | `hide`：`true` 隐藏，`false` 显示 |
| `void lv_basechart_set_series_color(lv_obj_t *chart, lv_chart_series_t *series, lv_color_t color)` | 设置数据系列颜色 | `color`：颜色值 |
| `void lv_basechart_set_x_start_point(lv_obj_t *obj, lv_chart_series_t *ser, uint16_t id)` | 设置 X 轴起始点 | `id`：起始点 ID |
| `lv_chart_series_t *lv_basechart_get_series_next(const lv_obj_t *chart, const lv_chart_series_t *ser)` | 获取下一个数据系列 | `ser`：当前数据系列；返回下一个系列指针 |
| `void lv_basechart_set_all_value(lv_obj_t *obj, lv_chart_series_t *ser, lv_coord_t value)` | 将系列所有点设为同一值 | `value`：值 |
| `void lv_basechart_set_next_value(lv_obj_t *obj, lv_chart_series_t *ser, lv_coord_t value)` | 推入下一个点的值 | 常用于流式滚动更新 |
| `void lv_basechart_set_next_value2(lv_obj_t *obj, lv_chart_series_t *ser, lv_coord_t x_value, lv_coord_t y_value)` | 推入下一个点的 X/Y 值 | `x_value`/`y_value`：X/Y 值 |
| `void lv_basechart_set_value_by_id(lv_obj_t *obj, lv_chart_series_t *ser, uint16_t id, lv_coord_t value)` | 设置指定 ID 点的值 | `id`：点 ID |
| `void lv_basechart_set_value_by_id2(lv_obj_t *obj, lv_chart_series_t *ser, uint16_t id, lv_coord_t x_value, lv_coord_t y_value)` | 设置指定 ID 点的 X/Y 值 | — |
| `void lv_basechart_set_ext_y_array(lv_obj_t *obj, lv_chart_series_t *ser, lv_coord_t array[])` | 设置外部 Y 轴数组 | `array`：外部 Y 数组 |
| `void lv_basechart_set_ext_x_array(lv_obj_t *obj, lv_chart_series_t *ser, lv_coord_t array[])` | 设置外部 X 轴数组 | `array`：外部 X 数组 |
| `lv_coord_t *lv_basechart_get_y_array(const lv_obj_t *obj, lv_chart_series_t *ser)` | 获取 Y 轴数组指针 | 返回 Y 数组指针 |
| `lv_coord_t *lv_basechart_get_x_array(const lv_obj_t *obj, lv_chart_series_t *ser)` | 获取 X 轴数组指针 | 返回 X 数组指针 |
| `uint32_t lv_basechart_get_pressed_point(const lv_obj_t *obj)` | 获取用户按下的点 ID | 返回按下的点 ID |

### 光标

| 接口函数 | 功能说明 | 参数/返回值说明 |
| --- | --- | --- |
| `lv_chart_cursor_t *lv_basechart_add_cursor(lv_obj_t *obj, lv_color_t color, lv_dir_t dir)` | 添加光标 | `color`：光标颜色；`dir`：光标方向；返回光标指针 |
| `void lv_basechart_set_cursor_pos(lv_obj_t *chart, lv_chart_cursor_t *cursor, lv_point_t *pos)` | 设置光标位置 | `pos`：位置坐标 |
| `void lv_basechart_set_cursor_point(lv_obj_t *chart, lv_chart_cursor_t *cursor, lv_chart_series_t *ser, uint16_t point_id)` | 设置光标指向的数据点 | `point_id`：点 ID |
| `lv_point_t lv_basechart_get_cursor_point(lv_obj_t *chart, lv_chart_cursor_t *cursor)` | 获取光标指向的点坐标 | 返回光标指向的点坐标 |

## 典型用法

下面示例绘制两条折线序列（红色、蓝色），每条 10 个数据点，纵轴范围 0~100：

```c
#include "lvsf_basechart.h"

void demo_basechart_init(void)
{
    lv_obj_t *scr = lv_scr_act();

    lv_obj_t *chart = lv_basechart_create(scr);
    lv_obj_set_size(chart, 280, 180);
    lv_obj_align(chart, LV_ALIGN_CENTER, 0, 10);
    lv_obj_set_style_bg_color(chart, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(chart, LV_OPA_COVER, 0);

    /* 网格颜色与线宽 */
    lv_obj_set_style_line_color(chart, lv_palette_lighten(LV_PALETTE_GREY, 2), LV_PART_MAIN);
    lv_obj_set_style_line_width(chart, 1, LV_PART_MAIN);
    /* 序列线宽、数据点大小 */
    lv_obj_set_style_line_width(chart, 3, LV_PART_ITEMS);
    lv_obj_set_style_width(chart, 5, LV_PART_INDICATOR);
    lv_obj_set_style_height(chart, 5, LV_PART_INDICATOR);

    lv_basechart_set_type(chart, LV_CHART_TYPE_LINE);
    lv_basechart_set_point_count(chart, 10);
    lv_basechart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, 0, 100);
    lv_basechart_set_div_line_count(chart, 5, 6);

    static const lv_coord_t v1[10] = {10, 45, 30, 70, 50, 92, 60, 80, 40, 78};
    static const lv_coord_t v2[10] = {80, 60, 75, 40, 55, 20, 48, 35, 65, 30};
    lv_chart_series_t *s1 = lv_basechart_add_series(chart, lv_palette_main(LV_PALETTE_RED),
                                                    LV_CHART_AXIS_PRIMARY_Y);
    lv_chart_series_t *s2 = lv_basechart_add_series(chart, lv_palette_main(LV_PALETTE_BLUE),
                                                    LV_CHART_AXIS_PRIMARY_Y);
    for (int i = 0; i < 10; i++)
    {
        lv_basechart_set_next_value(chart, s1, v1[i]);
        lv_basechart_set_next_value(chart, s2, v2[i]);
    }
    lv_basechart_refresh(chart);
}
```

## 效果展示

运行 `lvgl_v8_basechart` example 可查看实际效果：屏幕中央显示一个白底图表，带网格线，上面是两条折线（红、蓝），每条 10 个数据点，纵轴范围 0~100。

## 参考文档

- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- [LVGL v8 Chart 文档](https://docs.lvgl.io/8.3/widgets/chart.html)
- 源码路径：`middleware/lvgl/lvsf/gui_widgets/lvsf_basechart.h`
- 示例路径：`example/multimedia/lvgl/lvgl_v8_basechart`
