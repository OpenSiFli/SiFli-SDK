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
#include "lvsf_multslider.h"
#include "demo_multslider.h"

/**********************
 *  STATIC HELPERS
 **********************/

/* Style and place one multslider. The widget draws nothing until the app styles
 * its parts: LV_PART_MAIN is the track, LV_PART_INDICATOR is the value fill --
 * both need a bg color + opacity. The MAIN text color is the on-bar label color. */
static lv_obj_t *make_slider(lv_obj_t *parent, lv_coord_t y, lv_color_t fill, int32_t value)
{
    lv_obj_t *ms = lv_multslider_create(parent);
    lv_obj_set_size(ms, 240, 40);
    lv_obj_align(ms, LV_ALIGN_TOP_MID, 0, y);
    lv_obj_set_style_bg_color(ms, lv_color_hex(0xD0D4DA), LV_PART_MAIN);     /* track */
    lv_obj_set_style_bg_opa(ms, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_text_color(ms, lv_color_hex(0x303030), LV_PART_MAIN);   /* knob label color */
    lv_obj_set_style_bg_color(ms, fill, LV_PART_INDICATOR);                  /* fill */
    lv_obj_set_style_bg_opa(ms, LV_OPA_COVER, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(ms, lv_color_white(), LV_PART_KNOB);           /* knob: required, the
                                                                             * value/text draws on it */
    lv_obj_set_style_bg_opa(ms, LV_OPA_COVER, LV_PART_KNOB);
    lv_multslider_set_range(ms, 0, 100);
    lv_multslider_set_value(ms, value, LV_ANIM_OFF);
    return ms;
}

static lv_obj_t *make_caption(lv_obj_t *parent, lv_coord_t y, const char *text)
{
    lv_obj_t *l = lv_label_create(parent);
    lv_label_set_text(l, text);
    lv_obj_align(l, LV_ALIGN_TOP_MID, 0, y);
    return l;
}

/**********************
 *  GLOBAL FUNCTIONS
 **********************/

/**
 * @brief Demonstrate lvsf_multslider.
 *
 * Unlike the plain lv_slider / lv_bar, multslider draws a text label on its
 * circular knob: with no custom text it auto-formats the current value as a
 * number, and lv_multslider_set_txt() replaces it with a fixed short string.
 * The knob (LV_PART_KNOB) must be styled with a bg, otherwise the knob -- and
 * the text on it -- is not drawn. It is draggable: press and move along the bar
 * to change the value and the readout follows.
 */
void demo_multslider_init(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "multslider");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 20);

    /* (1) value mode: with no custom text the knob shows the LIVE value, so the
     *     number updates as you drag. */
    make_caption(scr, 70, "auto value (follows drag)");
    make_slider(scr, 96, lv_palette_main(LV_PALETTE_BLUE), 60);

    /* (2) label mode: set_txt pins a FIXED string on the knob. Dragging still
     *     moves the knob, but the text stays put -- it does NOT track the value. */
    make_caption(scr, 156, "set_txt: fixed label (stays on drag)");
    lv_obj_t *ms2 = make_slider(scr, 182, lv_palette_main(LV_PALETTE_GREEN), 75);
    lv_multslider_set_txt(ms2, "Vol");
}
