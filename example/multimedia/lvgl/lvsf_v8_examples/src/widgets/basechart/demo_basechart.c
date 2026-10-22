/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*********************
 *      INCLUDES
 *********************/
#include "rtconfig.h"
#include "lvgl.h"
#include "lvsf.h"               /* pulls lvsf_conf_internal.h -> LVSF_USE_* macros */
#include "lvsf_basechart.h"
#include "demo_basechart.h"

/**********************
 *  GLOBAL FUNCTIONS
 **********************/

/**
 * @brief Demonstrate lvsf_basechart.
 *
 * basechart plots one or more data series. Set the chart type and point count,
 * give it a value range and grid, add a series (with a color) and push values
 * into it, then refresh. Style parts: LV_PART_ITEMS is the series line,
 * LV_PART_INDICATOR is the point markers. This demo draws two line series.
 */
void demo_basechart_init(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "basechart");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 20);

    lv_obj_t *chart = lv_basechart_create(scr);
    lv_obj_set_size(chart, 280, 180);
    lv_obj_align(chart, LV_ALIGN_CENTER, 0, 10);
    lv_obj_set_style_bg_color(chart, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(chart, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(chart, 1, 0);
    lv_obj_set_style_line_color(chart, lv_palette_lighten(LV_PALETTE_GREY, 2), LV_PART_MAIN); /* grid color */
    lv_obj_set_style_line_width(chart, 1, LV_PART_MAIN);    /* grid line width */
    lv_obj_set_style_line_width(chart, 3, LV_PART_ITEMS);   /* series line */
    lv_obj_set_style_width(chart, 5, LV_PART_INDICATOR);    /* point marker w */
    lv_obj_set_style_height(chart, 5, LV_PART_INDICATOR);   /* point marker h */

    lv_basechart_set_type(chart, LV_CHART_TYPE_LINE);
    lv_basechart_set_point_count(chart, 10);
    lv_basechart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, 0, 100);
    lv_basechart_set_div_line_count(chart, 5, 6);

    static const lv_coord_t v1[10] = {10, 45, 30, 70, 50, 92, 60, 80, 40, 78};
    static const lv_coord_t v2[10] = {80, 60, 75, 40, 55, 20, 48, 35, 65, 30};
    lv_chart_series_t *s1 = lv_basechart_add_series(chart, lv_palette_main(LV_PALETTE_RED),  LV_CHART_AXIS_PRIMARY_Y);
    lv_chart_series_t *s2 = lv_basechart_add_series(chart, lv_palette_main(LV_PALETTE_BLUE), LV_CHART_AXIS_PRIMARY_Y);
    for (int i = 0; i < 10; i++)
    {
        lv_basechart_set_next_value(chart, s1, v1[i]);
        lv_basechart_set_next_value(chart, s2, v2[i]);
    }
    lv_basechart_refresh(chart);
}
