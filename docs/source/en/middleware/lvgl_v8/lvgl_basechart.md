# LVGL v8 Basechart

`lvsf_basechart` is a custom chart widget that SiFli wraps around LVGL v8's native `lv_chart`. It draws one or more data series (line, bar, scatter, and so on) on the screen. While keeping every commonly used LVGL chart interface, it adds custom drawing capabilities such as colored bar segments, dual-endpoint bars, and scatter bars.

- Main header: _middleware/lvgl/lvsf/gui_widgets/lvsf_basechart.h_
- Dependent widget: LVGL chart (`LV_USE_CHART`)
- Example project: _example/multimedia/lvgl/lvgl_v8_basechart_

```{note}
`lvsf_basechart` inherits from `lv_chart_t` (its first struct member is `lv_chart_t bg`). Therefore, in addition to the wrapper interfaces listed below, the widget still follows the LVGL chart styling and event model. `LV_PART_ITEMS` corresponds to series lines, and `LV_PART_INDICATOR` corresponds to data-point markers.
```

## Features

- Supports multiple chart types such as line (LINE), bar (BAR), and scatter (SCATTER), with a one-to-one mapping to the LVGL `lv_chart` interfaces.
- Supports multiple data series; each series can have its own color, axis, and show/hide state.
- Supports colored bar segments: different colors are mapped to numeric ranges (`lv_basechart_color_t`).
- Supports three bar forms: normal bars, dual-endpoint bars (two data values per bar), and scatter bars (connected data points merged into one bar).
- Provides common chart capabilities such as cursor positioning, axis ticks, X/Y axis zoom, and grid dividers.
- Supports periodic refresh via a timer (`lv_basechart_refresh_timer()`), which is convenient for scroll-refreshed real-time data display.

## Use Cases

- Real-time data curves, such as heart rate, battery level, and sensor waveforms.
- Bar statistics, such as segmented battery level and numeric-range distribution.
- Dashboards and health-data charts that need coloring by numeric range.
- Streaming data series that update by scrolling (used together with `set_next_value` and timer refresh).

## Supported Boards

- sf32lb52-lcd_n16r8
- sf32lb52-lchspi-ulp

## Configuration and Initialization

`lvsf_basechart` depends on the LVGL chart component. In menuconfig, confirm that the chart widget is enabled:

```none
CONFIG_LV_USE_CHART=y
```

In the BSP, the `LVSF_USE_BASECHART` macro (under the lvsf configuration) controls whether this widget is compiled. The example project enables it by default in `proj.conf`.

Minimum flow to create and configure an object:

```c
#include "lvsf_basechart.h"

lv_obj_t *chart = lv_basechart_create(lv_scr_act());
lv_obj_set_size(chart, 280, 180);
```

## API Reference

### Widget creation and type extensions

| Function | Description | Parameters / Returns |
| --- | --- | --- |
| `lv_obj_t *lv_basechart_create(lv_obj_t *parent)` | Creates and initializes a basechart object | `parent`: parent object pointer; returns the chart object pointer on success, or `NULL` on failure |
| `void lv_basechart_refresh_timer(lv_timer_t *timer)` | Chart refresh timer callback | `timer`: timer object pointer; called periodically by the timer to update the chart content |
| `void lv_basechart_set_column_type(lv_obj_t *chart, lv_basechart_column_type_t column_type, uint8_t column_mult_num)` | Sets the bar type | `chart`: chart object; `column_type`: bar type (`lv_basechart_column_type_t`); `column_mult_num`: maximum number of data values per bar (used with the `LV_BASECHART_COLUMN_TYPE_MULT` type) |
| `void lv_basechart_set_color(lv_obj_t *chart, uint8_t num, lv_basechart_color_t *color)` | Sets colored segments | `chart`: chart object; `num`: number of color segments; `color`: pointer to the color segment array; different value ranges are shown in different colors |
| `void lv_basechart_add_simple_multiple(lv_obj_t *chart, uint8_t simple_multiple)` | Adds a simple multiple | `chart`: chart object; `simple_multiple`: the value step between consecutive data points of each data source |

Bar type enumeration (`lv_basechart_column_type_t`):

| Value | Meaning |
| --- | --- |
| `LV_BASECHART_COLUMN_TYPE_COMM` | Normal bar chart |
| `LV_BASECHART_COLUMN_TYPE_DOUBLE` | Bar chart with two endpoints; each bar requires two data values |
| `LV_BASECHART_COLUMN_TYPE_MULT` | Scatter points on a bar chart; connected data points are joined into one bar |

### Chart property settings (wrapped from lv_chart)

| Function | Description | Parameters / Returns |
| --- | --- | --- |
| `void lv_basechart_set_type(lv_obj_t *obj, lv_chart_type_t type)` | Sets the chart type (line/bar/etc.) | `obj`: chart object; `type`: chart type |
| `void lv_basechart_set_point_count(lv_obj_t *obj, uint16_t cnt)` | Sets the number of points per data series | `cnt`: number of points |
| `void lv_basechart_set_range(lv_obj_t *obj, lv_chart_axis_t axis, lv_coord_t min, lv_coord_t max)` | Sets the axis range | `axis`: X or Y axis; `min`/`max`: minimum/maximum value |
| `void lv_basechart_set_update_mode(lv_obj_t *obj, lv_chart_update_mode_t update_mode)` | Sets the chart update mode | `update_mode`: update mode |
| `void lv_basechart_set_div_line_count(lv_obj_t *obj, uint8_t hdiv, uint8_t vdiv)` | Sets the number of grid dividers | `hdiv`/`vdiv`: number of horizontal/vertical dividers |
| `void lv_basechart_set_zoom_x(lv_obj_t *obj, uint16_t zoom_x)` | Sets the X-axis zoom ratio | `zoom_x`: X-axis zoom ratio |
| `void lv_basechart_set_zoom_y(lv_obj_t *obj, uint16_t zoom_y)` | Sets the Y-axis zoom ratio | `zoom_y`: Y-axis zoom ratio |
| `uint16_t lv_basechart_get_zoom_x(const lv_obj_t *obj)` | Gets the X-axis zoom ratio | Returns the X-axis zoom ratio |
| `uint16_t lv_basechart_get_zoom_y(const lv_obj_t *obj)` | Gets the Y-axis zoom ratio | Returns the Y-axis zoom ratio |
| `void lv_basechart_set_axis_tick(lv_obj_t *obj, lv_chart_axis_t axis, lv_coord_t major_len, lv_coord_t minor_len, lv_coord_t major_cnt, lv_coord_t minor_cnt, bool label_en, lv_coord_t draw_size)` | Sets axis ticks | `axis`: axis; `major_len`/`minor_len`: major/minor tick length; `major_cnt`/`minor_cnt`: major/minor tick count; `label_en`: whether to show labels; `draw_size`: draw size |
| `lv_chart_type_t lv_basechart_get_type(const lv_obj_t *obj)` | Gets the chart type | Returns the chart type |
| `uint16_t lv_basechart_get_point_count(const lv_obj_t *obj)` | Gets the point count | Returns the point count |
| `uint16_t lv_basechart_get_x_start_point(const lv_obj_t *obj, lv_chart_series_t *ser)` | Gets the X-axis start-point index | `ser`: data series; returns the start-point index |
| `void lv_basechart_get_point_pos_by_id(lv_obj_t *obj, lv_chart_series_t *ser, uint16_t id, lv_point_t *p_out)` | Gets the coordinate position of the point with the given ID | `id`: point ID; `p_out`: output coordinates |
| `void lv_basechart_refresh(lv_obj_t *obj)` | Forces a chart refresh | No return value |

### Data series

| Function | Description | Parameters / Returns |
| --- | --- | --- |
| `lv_chart_series_t *lv_basechart_add_series(lv_obj_t *obj, lv_color_t color, lv_chart_axis_t axis)` | Adds a data series | `color`: series color; `axis`: axis; returns the data series pointer |
| `void lv_basechart_remove_series(lv_obj_t *obj, lv_chart_series_t *series)` | Removes a data series | `series`: data series to remove |
| `void lv_basechart_hide_series(lv_obj_t *chart, lv_chart_series_t *series, bool hide)` | Hides/shows a data series | `hide`: `true` hides, `false` shows |
| `void lv_basechart_set_series_color(lv_obj_t *chart, lv_chart_series_t *series, lv_color_t color)` | Sets the data series color | `color`: color value |
| `void lv_basechart_set_x_start_point(lv_obj_t *obj, lv_chart_series_t *ser, uint16_t id)` | Sets the X-axis start point | `id`: start-point ID |
| `lv_chart_series_t *lv_basechart_get_series_next(const lv_obj_t *chart, const lv_chart_series_t *ser)` | Gets the next data series | `ser`: current data series; returns the next series pointer |
| `void lv_basechart_set_all_value(lv_obj_t *obj, lv_chart_series_t *ser, lv_coord_t value)` | Sets all points of a series to the same value | `value`: value |
| `void lv_basechart_set_next_value(lv_obj_t *obj, lv_chart_series_t *ser, lv_coord_t value)` | Pushes the value of the next point | Commonly used for streaming scroll updates |
| `void lv_basechart_set_next_value2(lv_obj_t *obj, lv_chart_series_t *ser, lv_coord_t x_value, lv_coord_t y_value)` | Pushes the X/Y values of the next point | `x_value`/`y_value`: X/Y value |
| `void lv_basechart_set_value_by_id(lv_obj_t *obj, lv_chart_series_t *ser, uint16_t id, lv_coord_t value)` | Sets the value of the point with the given ID | `id`: point ID |
| `void lv_basechart_set_value_by_id2(lv_obj_t *obj, lv_chart_series_t *ser, uint16_t id, lv_coord_t x_value, lv_coord_t y_value)` | Sets the X/Y values of the point with the given ID | — |
| `void lv_basechart_set_ext_y_array(lv_obj_t *obj, lv_chart_series_t *ser, lv_coord_t array[])` | Sets an external Y array | `array`: external Y array |
| `void lv_basechart_set_ext_x_array(lv_obj_t *obj, lv_chart_series_t *ser, lv_coord_t array[])` | Sets an external X array | `array`: external X array |
| `lv_coord_t *lv_basechart_get_y_array(const lv_obj_t *obj, lv_chart_series_t *ser)` | Gets the Y array pointer | Returns the Y array pointer |
| `lv_coord_t *lv_basechart_get_x_array(const lv_obj_t *obj, lv_chart_series_t *ser)` | Gets the X array pointer | Returns the X array pointer |
| `uint32_t lv_basechart_get_pressed_point(const lv_obj_t *obj)` | Gets the point ID pressed by the user | Returns the pressed point ID |

### Cursor

| Function | Description | Parameters / Returns |
| --- | --- | --- |
| `lv_chart_cursor_t *lv_basechart_add_cursor(lv_obj_t *obj, lv_color_t color, lv_dir_t dir)` | Adds a cursor | `color`: cursor color; `dir`: cursor direction; returns the cursor pointer |
| `void lv_basechart_set_cursor_pos(lv_obj_t *chart, lv_chart_cursor_t *cursor, lv_point_t *pos)` | Sets the cursor position | `pos`: position coordinates |
| `void lv_basechart_set_cursor_point(lv_obj_t *chart, lv_chart_cursor_t *cursor, lv_chart_series_t *ser, uint16_t point_id)` | Sets the data point the cursor points to | `point_id`: point ID |
| `lv_point_t lv_basechart_get_cursor_point(lv_obj_t *chart, lv_chart_cursor_t *cursor)` | Gets the coordinates of the point the cursor points to | Returns the coordinates of the point the cursor points to |

## Typical Usage

The following example draws two line series (red and blue), each with 10 data points, and a vertical-axis range of 0~100:

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

    /* Grid color and line width */
    lv_obj_set_style_line_color(chart, lv_palette_lighten(LV_PALETTE_GREY, 2), LV_PART_MAIN);
    lv_obj_set_style_line_width(chart, 1, LV_PART_MAIN);
    /* Series line width and data-point size */
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

## Demo

Run the `lvgl_v8_basechart` example to see the result: a white-background chart with grid lines is shown in the center of the screen, drawing two polylines (red and blue), each with 10 data points, and a vertical-axis range of 0~100.

## See Also

- [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- [LVGL v8 Chart documentation](https://docs.lvgl.io/8.3/widgets/chart.html)
- Source path: `middleware/lvgl/lvsf/gui_widgets/lvsf_basechart.h`
- Example path: `example/multimedia/lvgl/lvgl_v8_basechart`
