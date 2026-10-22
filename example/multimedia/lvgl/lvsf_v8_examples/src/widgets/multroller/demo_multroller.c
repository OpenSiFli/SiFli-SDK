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
#include "lvsf_multroller.h"
#include "demo_multroller.h"

/**********************
 *  STATIC VARIABLES
 **********************/

static const char *const MONTHS[12] = {
    "January", "February", "March", "April", "May", "June",
    "July", "August", "September", "October", "November", "December"
};

/* The options string. multroller splits on '\n'; every option (the last one
 * included) must be followed by '\n', so the string ends with a blank line. */
static const char *MONTH_OPTIONS =
    "January\nFebruary\nMarch\nApril\nMay\nJune\n"
    "July\nAugust\nSeptember\nOctober\nNovember\nDecember\n\n";

static lv_obj_t *s_roller;
static lv_obj_t *s_readout;
static uint16_t  s_last_sel = 0xFFFF;

/**********************
 *  STATIC FUNCTIONS
 **********************/

/* Poll the centered (selected) option and mirror it into the read-out. The
 * roller snaps on release, so the selection settles a moment after the drag. */
static void sel_poll_cb(lv_timer_t *timer)
{
    LV_UNUSED(timer);
    uint16_t sel = lv_multroller_get_selected(s_roller);
    if (sel != s_last_sel && sel < 12)
    {
        s_last_sel = sel;
        lv_label_set_text_fmt(s_readout, "selected:  %s", MONTHS[sel]);
    }
}

/**********************
 *  GLOBAL FUNCTIONS
 **********************/

/**
 * @brief Demonstrate lvsf_multroller.
 *
 * multroller is the easy, declarative roller (a picker built on lvsf_multlist):
 * give it a single '\n'-separated options string and it lays out a looping,
 * snapping list with a center focus area that re-colors the selected option.
 * Unlike lvsf_mulroller (a low-level, callback-driven wheel), here you only set
 * options / visible count / focus, and read the choice back with get_selected().
 * This demo is a month picker.
 */
void demo_multroller_init(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "multroller");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 16);

    /* the roller. It defaults to full-screen / opaque, so size it and make it
     * transparent first; item text uses the roller's own font and color. */
    s_roller = lv_multroller_create(scr);
    lv_obj_set_size(s_roller, 240, 240);
    lv_obj_set_style_bg_opa(s_roller, LV_OPA_TRANSP, 0);
    lv_obj_set_style_text_font(s_roller, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(s_roller, lv_color_hex(0xBDBDBD), 0);   /* unselected: grey */
    lv_obj_center(s_roller);

    lv_multroller_set_show_cnt(s_roller, 5);                 /* 5 options visible */
    lv_multroller_set_options(s_roller, MONTH_OPTIONS);
    /* center focus area: the option inside it is highlighted (red) */
    lv_multroller_set_focus_param(s_roller, lv_palette_main(LV_PALETTE_RED), 240, 48);

    /* read-out below the roller, kept in sync with get_selected() */
    s_readout = lv_label_create(scr);
    lv_obj_set_style_text_color(s_readout, lv_palette_main(LV_PALETTE_RED), 0);
    lv_label_set_text(s_readout, "selected:  --");
    lv_obj_align(s_readout, LV_ALIGN_BOTTOM_MID, 0, -12);

    lv_timer_create(sel_poll_cb, 150, NULL);
}
