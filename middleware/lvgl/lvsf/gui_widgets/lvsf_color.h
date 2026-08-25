/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __LVSF_COLOR_H
#define __LVSF_COLOR_H

#include <stdint.h>
#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

static inline uint8_t lvsf_color_get_depth(void)
{
    uint32_t true_color_size = lv_img_buf_get_img_size(1, 1,
                                                        LV_IMG_CF_TRUE_COLOR);
    uint32_t true_color_alpha_size = lv_img_buf_get_img_size(
                                         1, 1, LV_IMG_CF_TRUE_COLOR_ALPHA);

    if ((true_color_size == 3) && (true_color_alpha_size == 4))
        return 24;
    if ((true_color_size == 4) && (true_color_alpha_size == 4))
        return 32;

    return 16;
}

#define LVSF_COLOR_FLAG_24BIT (1UL << 1)
#define LVSF_IS_24BIT_COLOR (lvsf_color_get_depth() == 24)

#define LVSF_COLOR32(a, r, g, b) ((((uint32_t)(a) & 0xFF) << 24) | \
                                  (((uint32_t)(r) & 0xFF) << 16) | \
                                  (((uint32_t)(g) & 0xFF) << 8) | \
                                  ((uint32_t)(b) & 0xFF))
#define LVSF_COLOR_RGB(r, g, b) LVSF_COLOR32(0xFF, r, g, b)
#define LVSF_COLOR_BLACK LVSF_COLOR_RGB(0x00, 0x00, 0x00)
#define LVSF_COLOR_WHITE LVSF_COLOR_RGB(0xFF, 0xFF, 0xFF)
#define LVSF_COLOR_RED LVSF_COLOR_RGB(0xFF, 0x00, 0x00)
#define LVSF_COLOR_GREEN LVSF_COLOR_RGB(0x00, 0x80, 0x00)
#define LVSF_COLOR_YELLOW LVSF_COLOR_RGB(0xFF, 0xFF, 0x00)
#define LVSF_COLOR_CYAN LVSF_COLOR_RGB(0x00, 0xFF, 0xFF)

static inline uint16_t lvsf_rgb565_from_color32(uint32_t color)
{
    uint8_t r = (color >> 16) & 0xFF;
    uint8_t g = (color >> 8) & 0xFF;
    uint8_t b = color & 0xFF;

    return (uint16_t)((r & 0xF8) << 8) |
           (uint16_t)((g & 0xFC) << 3) |
           (uint16_t)(b >> 3);
}

static inline uint32_t lvsf_color_hsv_to_rgb32(uint16_t h, uint8_t s, uint8_t v)
{
    h = (uint32_t)((uint32_t)h * 255) / 360;
    s = (uint16_t)((uint16_t)s * 255) / 100;
    v = (uint16_t)((uint16_t)v * 255) / 100;

    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t region;
    uint8_t remainder;
    uint8_t p;
    uint8_t q;
    uint8_t t;

    if (s == 0)
        return LVSF_COLOR_RGB(v, v, v);

    region = h / 43;
    remainder = (h - (region * 43)) * 6;
    p = (v * (255 - s)) >> 8;
    q = (v * (255 - ((s * remainder) >> 8))) >> 8;
    t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;

    switch (region)
    {
    case 0:
        r = v;
        g = t;
        b = p;
        break;
    case 1:
        r = q;
        g = v;
        b = p;
        break;
    case 2:
        r = p;
        g = v;
        b = t;
        break;
    case 3:
        r = p;
        g = q;
        b = v;
        break;
    case 4:
        r = t;
        g = p;
        b = v;
        break;
    default:
        r = v;
        g = p;
        b = q;
        break;
    }

    return LVSF_COLOR_RGB(r, g, b);
}

static inline uint32_t lvsf_raw_color_from_color32(uint32_t color)
{
    uint8_t r = (color >> 16) & 0xFF;
    uint8_t g = (color >> 8) & 0xFF;
    uint8_t b = color & 0xFF;
    uint8_t a = (color >> 24) & 0xFF;
    uint8_t depth = lvsf_color_get_depth();

    if (depth == 24)
        return ((uint32_t)b) | ((uint32_t)g << 8) | ((uint32_t)r << 16);
    if (depth == 32)
        return ((uint32_t)b) | ((uint32_t)g << 8) |
               ((uint32_t)r << 16) | ((uint32_t)a << 24);

    return lvsf_rgb565_from_color32(color);
}

static inline lv_style_value_t lvsf_style_color_value_from_color32(uint32_t color)
{
    lv_style_value_t value = {0};

    value.num = (int32_t)lvsf_raw_color_from_color32(color);
    return value;
}

static inline void lvsf_obj_set_style_color32(lv_obj_t *obj,
                                               lv_style_prop_t prop,
                                               uint32_t color,
                                               lv_style_selector_t selector)
{
    lv_style_value_t value = lvsf_style_color_value_from_color32(color);
    lv_obj_set_local_style_prop(obj, prop, value, selector);
}

static inline void lvsf_obj_set_style_bg_color32(lv_obj_t *obj, uint32_t color,
                                                  lv_style_selector_t selector)
{
    lvsf_obj_set_style_color32(obj, LV_STYLE_BG_COLOR, color, selector);
}

static inline void lvsf_obj_set_style_bg_grad_color32(
    lv_obj_t *obj, uint32_t color, lv_style_selector_t selector)
{
    lvsf_obj_set_style_color32(obj, LV_STYLE_BG_GRAD_COLOR, color, selector);
}

static inline void lvsf_obj_set_style_bg_img_recolor32(
    lv_obj_t *obj, uint32_t color, lv_style_selector_t selector)
{
    lvsf_obj_set_style_color32(obj, LV_STYLE_BG_IMG_RECOLOR, color, selector);
}

static inline void lvsf_obj_set_style_border_color32(
    lv_obj_t *obj, uint32_t color, lv_style_selector_t selector)
{
    lvsf_obj_set_style_color32(obj, LV_STYLE_BORDER_COLOR, color, selector);
}

static inline void lvsf_obj_set_style_outline_color32(
    lv_obj_t *obj, uint32_t color, lv_style_selector_t selector)
{
    lvsf_obj_set_style_color32(obj, LV_STYLE_OUTLINE_COLOR, color, selector);
}

static inline void lvsf_obj_set_style_shadow_color32(
    lv_obj_t *obj, uint32_t color, lv_style_selector_t selector)
{
    lvsf_obj_set_style_color32(obj, LV_STYLE_SHADOW_COLOR, color, selector);
}

static inline void lvsf_obj_set_style_img_recolor32(
    lv_obj_t *obj, uint32_t color, lv_style_selector_t selector)
{
    lvsf_obj_set_style_color32(obj, LV_STYLE_IMG_RECOLOR, color, selector);
}

static inline void lvsf_obj_set_style_line_color32(
    lv_obj_t *obj, uint32_t color, lv_style_selector_t selector)
{
    lvsf_obj_set_style_color32(obj, LV_STYLE_LINE_COLOR, color, selector);
}

static inline void lvsf_obj_set_style_arc_color32(
    lv_obj_t *obj, uint32_t color, lv_style_selector_t selector)
{
    lvsf_obj_set_style_color32(obj, LV_STYLE_ARC_COLOR, color, selector);
}

static inline void lvsf_obj_set_style_text_color32(
    lv_obj_t *obj, uint32_t color, lv_style_selector_t selector)
{
    lvsf_obj_set_style_color32(obj, LV_STYLE_TEXT_COLOR, color, selector);
}

#ifdef __cplusplus
}
#endif

#endif /* __LVSF_COLOR_H */
