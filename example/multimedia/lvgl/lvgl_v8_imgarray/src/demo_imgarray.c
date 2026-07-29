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
#include "lvsf_baseimg.h"       /* BASEIMG_TYPE_* */
#include "lvsf_imgarray.h"
#include "demo_imgarray.h"

/*********************
 *      DEFINES
 *********************/
#define GLYPH_W  34             /* one glyph image size */
#define GLYPH_H  54
#define N_GLYPH  12             /* "0".."9", ".", "%" */
#define IDX_POINT 10
#define IDX_UNIT  11
#define INT_DIGITS   2
#define FLOAT_DIGITS 1

/**********************
 *  STATIC VARIABLES
 **********************/
static const char *GLYPH[N_GLYPH] =
{
    "0", "1", "2", "3", "4", "5", "6", "7", "8", "9", ".", "%"
};
/* glyph images, rendered once; imgarray indexes into them for digits/point/unit. */
static lv_color_t        glyph_buf[N_GLYPH][GLYPH_W * GLYPH_H];
static lv_img_dsc_t      glyph_dsc[N_GLYPH];
static const lv_img_dsc_t *glyph_arr[N_GLYPH];

/**********************
 *  STATIC FUNCTIONS
 **********************/

/* Render each glyph onto its buffer with a temporary canvas, then wrap an image
 * descriptor over the buffer so imgarray can use them as digit/point/unit sources. */
static void build_glyph_images(lv_obj_t *parent)
{
    lv_obj_t *canvas = lv_canvas_create(parent);
    lv_draw_label_dsc_t ld;
    lv_draw_label_dsc_init(&ld);
    ld.font = &lv_font_montserrat_36;
    ld.color = lv_color_white();
    ld.align = LV_TEXT_ALIGN_CENTER;

    for (int i = 0; i < N_GLYPH; i++)
    {
        lv_canvas_set_buffer(canvas, glyph_buf[i], GLYPH_W, GLYPH_H, LV_IMG_CF_TRUE_COLOR);
        lv_canvas_fill_bg(canvas, lv_color_hex(0x202830), LV_OPA_COVER);
        lv_canvas_draw_text(canvas, 0, (GLYPH_H - 36) / 2, GLYPH_W, &ld, GLYPH[i]);

        glyph_dsc[i].header.cf = LV_IMG_CF_TRUE_COLOR;
        glyph_dsc[i].header.always_zero = 0;
        glyph_dsc[i].header.reserved = 0;
        glyph_dsc[i].header.w = GLYPH_W;
        glyph_dsc[i].header.h = GLYPH_H;
        glyph_dsc[i].data_size = (uint32_t)(GLYPH_W * GLYPH_H) * sizeof(lv_color_t);
        glyph_dsc[i].data = (const uint8_t *)glyph_buf[i];
        glyph_arr[i] = &glyph_dsc[i];
    }
    lv_obj_del(canvas);
}

/* Sweep a value (in tenths) over 20.0..80.0 and back, feeding it to the imgarray
 * as Q24.8 so the displayed number changes with one decimal place. */
static void ramp_cb(lv_timer_t *t)
{
    lv_obj_t *ia = (lv_obj_t *)t->user_data;
    static int tenths = 200;   /* 20.0 */
    static int step = 5;       /* 0.5 per tick */
    tenths += step;
    if (tenths >= 800) { tenths = 800; step = -5; }
    else if (tenths <= 200) { tenths = 200; step = 5; }
    lv_imgarray_set_value(ia, tenths * 256 / 10);   /* tenths/10 in Q24.8 */
}

/**********************
 *  GLOBAL FUNCTIONS
 **********************/

/**
 * @brief Demonstrate lvsf_imgarray (built on lvsf_baseimg).
 *
 * imgarray renders a numeric value out of per-glyph images: digits, an optional
 * decimal point, a unit, and a sign. Give it a glyph array (0..9 plus extras),
 * the integer/decimal digit counts, the indices of the point/unit glyphs, and a
 * value; it lays out [sign][int].[frac][unit] from those images. With
 * BASEIMG_TYPE_ARRAY_Q248 the value is Q24.8, so it can show decimals. This demo
 * shows a measurement like "36.5%" sweeping over 20.0..80.0.
 */
void demo_imgarray_init(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "imgarray");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 20);

    build_glyph_images(scr);

    lv_obj_t *ia = lv_imgarray_create(scr);
    lv_imgarray_set_img_type(ia, BASEIMG_TYPE_ARRAY_Q248);  /* value is Q24.8 -> supports decimals */
    lv_imgarray_set_src_array(ia, glyph_arr, 0, N_GLYPH - 1); /* glyphs (set before the counts) */
    lv_imgarray_set_interval(ia, 2);
    lv_imgarray_set_leading_zero(ia, true);
    lv_imgarray_set_trailing_zero(ia, true);
    lv_imgarray_set_int_num(ia, INT_DIGITS);                /* 2 integer digits */
    lv_imgarray_set_float_num(ia, FLOAT_DIGITS);            /* 1 decimal digit  */
    lv_imgarray_set_point_idx(ia, IDX_POINT);               /* "." glyph (after src_array) */
    lv_imgarray_set_unit_idx(ia, IDX_UNIT);                 /* "%" glyph */
    lv_obj_center(ia);
    lv_imgarray_set_value(ia, 365 * 256 / 10);              /* 36.5 in Q24.8 */

    lv_timer_create(ramp_cb, 120, ia);
}
