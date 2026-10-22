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
#include "lvsf_imgbar.h"
#include "lvsf_obj_ext.h"       /* lv_obj_set_range_value */
#include "demo_imgbar.h"

/*********************
 *      DEFINES
 *********************/
#define BAR_W 220               /* bar (and foreground image) width  */
#define BAR_H 28                /* bar (and foreground image) height */

/**********************
 *  STATIC FUNCTIONS
 **********************/

/* Fill a buffer with a solid color and wrap a true-color image descriptor over
 * it -- lets the demo build the foreground image at runtime without an asset. */
static void make_solid_img(lv_img_dsc_t *dsc, lv_color_t *buf, lv_coord_t w, lv_coord_t h, lv_color_t color)
{
    for (int i = 0; i < (int)(w * h); i++)
        buf[i] = color;
    dsc->header.cf = LV_IMG_CF_TRUE_COLOR;
    dsc->header.always_zero = 0;
    dsc->header.reserved = 0;
    dsc->header.w = w;
    dsc->header.h = h;
    dsc->data_size = (uint32_t)(w * h) * sizeof(lv_color_t);
    dsc->data = (const uint8_t *)buf;
}

/* Sweep the value 0..100 and back so the fill grows then shrinks. Each tick
 * re-clips the foreground image to the new value and redraws. */
static void imgbar_ramp_cb(lv_timer_t *t)
{
    lv_obj_t *imgbar = (lv_obj_t *)t->user_data;
    static int v = 0;
    static int step = 2;
    v += step;
    if (v >= 100) { v = 100; step = -2; }
    else if (v <= 0) { v = 0; step = 2; }
    lv_imgbar_set_value(imgbar, v);
}

/**********************
 *  GLOBAL FUNCTIONS
 **********************/

/**
 * @brief Demonstrate lvsf_imgbar.
 *
 * imgbar shows a foreground image clipped to a value, like a progress bar whose
 * fill is an image. Create a foreground lv_img, hand it to the imgbar with
 * lv_imgbar_set_img_fg (this sizes the imgbar to the image), pick a fill
 * direction, set the value range, then drive the value. A separate grey bar
 * behind it shows the unfilled track. The value is swept by a timer.
 */
void demo_imgbar_init(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "imgbar");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 20);

    /* grey track behind the bar (the unfilled part) */
    lv_obj_t *track = lv_obj_create(scr);
    lv_obj_remove_style_all(track);
    lv_obj_set_size(track, BAR_W, BAR_H);
    lv_obj_center(track);
    lv_obj_set_style_bg_color(track, lv_color_hex(0xD0D4DA), 0);
    lv_obj_set_style_bg_opa(track, LV_OPA_COVER, 0);

    /* imgbar with a blue foreground image, placed over the track */
    static lv_color_t fg_buf[BAR_W * BAR_H];
    static lv_img_dsc_t fg_dsc;
    make_solid_img(&fg_dsc, fg_buf, BAR_W, BAR_H, lv_palette_main(LV_PALETTE_BLUE));

    lv_obj_t *imgbar = lv_imgbar_create(scr);
    lv_obj_clear_flag(imgbar, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t *fg = lv_img_create(imgbar);
    lv_img_set_src(fg, &fg_dsc);
    lv_obj_refr_size(fg);
    lv_imgbar_set_img_fg(imgbar, fg);          /* sizes imgbar to the fg image */
    /* set_img_fg wraps the fg in an internal container (the fg's new parent);
     * clear its default padding/border/scrolling and pin the fg to the top-left
     * so the fill is flush and stays in view. */
    lv_obj_t *fg_box = lv_obj_get_parent(fg);
    lv_obj_set_style_pad_all(fg_box, 0, 0);
    lv_obj_set_style_border_width(fg_box, 0, 0);
    lv_obj_clear_flag(fg_box, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_pos(fg, 0, 0);
    lv_imgbar_set_dir(imgbar, BAR_DIR_LEFT_TO_RIGTH);
    lv_obj_set_range_value(imgbar, 0, 100);
    lv_obj_center(imgbar);                      /* align over the track */
    lv_imgbar_set_value(imgbar, 0);

    lv_timer_create(imgbar_ramp_cb, 60, imgbar);
}
