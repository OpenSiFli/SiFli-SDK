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
#include "lvsf_mulroller.h"
#include "demo_mulroller.h"

/**********************
 *  STATIC VARIABLES
 **********************/

/* Combined read-out: "<weekday>  HH:MM", driven by each wheel's middle_cb. */
static lv_obj_t *s_readout;
static const char *const WD[7] = {"Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"};
static int s_wd;   /* 0..6 */
static int s_hh;   /* 0..23 */
static int s_mm;   /* 0..59 */

/**********************
 *  STATIC FUNCTIONS
 **********************/

static void readout_refresh(void)
{
    if (s_readout)
    {
        lv_label_set_text_fmt(s_readout, "%s  %02d:%02d", WD[s_wd], s_hh, s_mm);
    }
}

/* appear_cb supplies the content of a slot as it scrolls into a data index.
 * Numbers are formatted "%02d"; weekdays index a name table and wrap (the
 * weekday wheel is an infinite loop, so idx can be any integer). */
static bool num_appear_cb(lv_obj_t *label, int16_t idx)
{
    lv_label_set_text_fmt(label, "%02d", idx);
    return true;
}

static bool wd_appear_cb(lv_obj_t *label, int16_t idx)
{
    lv_label_set_text(label, WD[((idx % 7) + 7) % 7]);
    return true;
}

/* middle_cb fires when a wheel stops, with the now-centered (selected) index. */
static bool hh_middle_cb(lv_obj_t *label, int16_t idx) { LV_UNUSED(label); s_hh = idx; readout_refresh(); return true; }
static bool mm_middle_cb(lv_obj_t *label, int16_t idx) { LV_UNUSED(label); s_mm = idx; readout_refresh(); return true; }
static bool wd_middle_cb(lv_obj_t *label, int16_t idx) { LV_UNUSED(label); s_wd = ((idx % 7) + 7) % 7; readout_refresh(); return true; }

/* One reusable wheel builder: every wheel here is a LABEL roller, differing only
 * in direction, range, looping and the callbacks. */
typedef struct
{
    lv_mulroller_dir_t          dir;
    uint8_t                     ele_num;
    lv_coord_t                  w, h, ele_w, ele_h;
    lv_mulroller_appear_cb      appear;
    lv_mulroller_appear_cb      middle;
    lv_mulroller_circle_mode_t  circle;
    int16_t                     min, max;     /* used only for CIRCLE_NORMAL */
    uint16_t                    mid_zoom, side_zoom;
} wheel_cfg_t;

static lv_obj_t *make_wheel(lv_obj_t *parent, const wheel_cfg_t *c)
{
    lv_obj_t *r = lv_mulroller_create(parent);
    lv_obj_set_size(r, c->w, c->h);
    lv_mulroller_set_obj_type(r, MULROLLER_TYPE_LABEL);
    lv_mulroller_create_element(r, c->ele_num, c->ele_w, c->ele_h);

    /* drop the default lv_obj card (border/background/scrollbar) on each holder */
    lv_mulroller_t *ext = (lv_mulroller_t *)r;
    for (uint8_t i = 0; i < ext->ele_num; i++)
    {
        lv_obj_remove_style_all(ext->ele_tab[i].bg_obj);
        lv_obj_set_size(ext->ele_tab[i].bg_obj, c->ele_w, c->ele_h);
        lv_obj_clear_flag(ext->ele_tab[i].bg_obj, LV_OBJ_FLAG_SCROLLABLE);
    }

    lv_mulroller_set_appear_cb(r, c->appear);
    lv_mulroller_set_middle_cb(r, c->middle);

    lv_mulroller_set_dir(r, c->dir);
    lv_mulroller_set_align(r, MULROLLER_ALIGN_CENTER);
    lv_mulroller_set_layout_mode(r, MULROLLER_LAYOUT_MID);
    lv_mulroller_set_circle_mode(r, c->circle);
    if (c->circle == MULROLLER_CIRCLE_NORMAL)
    {
        lv_mulroller_set_circle_range(r, c->min, c->max);
    }

    /* emphasise the centered element: bigger + red; neighbours small + grey,
     * and (bounded wheels only) faded. */
    lv_mulroller_set_zoom_range(r, c->mid_zoom, c->side_zoom);
    lv_mulroller_set_color_mode(r, MULROLLER_COLOR_POS);
    lv_mulroller_set_color_range(r, 0xFF0000, 0x9E9E9E);
    if (c->circle == MULROLLER_CIRCLE_NORMAL)   /* opa range is ignored in infinite mode */
    {
        lv_mulroller_set_opa_mode(r, MULROLLER_OPA_MID);
        lv_mulroller_set_opa_range(r, 255, 130);
    }

    lv_mulroller_validate(r);
    return r;
}

/**********************
 *  GLOBAL FUNCTIONS
 **********************/

/**
 * @brief Demonstrate lvsf_mulroller.
 *
 * mulroller is a wheel/drum picker: scroll a strip of elements, snap to the
 * centered one as the selection, with the center emphasised over its neighbours.
 * This demo combines three wheels to cover the widget's range:
 *   - a horizontal, infinitely-looping weekday wheel (text content that wraps);
 *   - two vertical, bounded number wheels forming an HH:MM time picker.
 * Slot content comes from appear_cb, and each wheel reports its settled value
 * through middle_cb into one combined read-out below.
 */
void demo_mulroller_init(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "mulroller");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 6);

    /* (1) horizontal, infinite-loop weekday wheel -- text content, wraps forever */
    wheel_cfg_t wd = {
        .dir = MULROLLER_DIR_HOR, .ele_num = 5, .w = 360, .h = 58, .ele_w = 96, .ele_h = 58,
        .appear = wd_appear_cb, .middle = wd_middle_cb,
        .circle = MULROLLER_CIRCLE_INFINITE, .mid_zoom = 36, .side_zoom = 20,
    };
    lv_obj_t *wd_roller = make_wheel(scr, &wd);
    lv_obj_align(wd_roller, LV_ALIGN_TOP_MID, 0, 44);

    /* (2) two vertical, bounded number wheels -> HH:MM time picker */
    wheel_cfg_t num = {
        .dir = MULROLLER_DIR_VER, .ele_num = 5, .w = 78, .h = 156, .ele_w = 78, .ele_h = 52,
        .appear = num_appear_cb, .circle = MULROLLER_CIRCLE_NORMAL,
        .mid_zoom = 36, .side_zoom = 24,
    };

    wheel_cfg_t hh = num; hh.middle = hh_middle_cb; hh.min = 0; hh.max = 23;
    lv_obj_t *hh_roller = make_wheel(scr, &hh);
    lv_obj_align(hh_roller, LV_ALIGN_CENTER, -70, 18);

    wheel_cfg_t mm = num; mm.middle = mm_middle_cb; mm.min = 0; mm.max = 59;
    lv_obj_t *mm_roller = make_wheel(scr, &mm);
    lv_obj_align(mm_roller, LV_ALIGN_CENTER, 70, 18);

    lv_obj_t *colon = lv_label_create(scr);
    lv_obj_set_style_text_font(colon, &lv_font_montserrat_36, 0);
    lv_obj_set_style_text_color(colon, lv_color_hex(0x9E9E9E), 0);
    lv_label_set_text(colon, ":");
    lv_obj_align(colon, LV_ALIGN_CENTER, 0, 14);

    /* combined read-out, updated by each wheel's middle_cb on settle */
    s_readout = lv_label_create(scr);
    lv_obj_set_style_text_color(s_readout, lv_palette_main(LV_PALETTE_BLUE), 0);
    lv_obj_align(s_readout, LV_ALIGN_BOTTOM_MID, 0, -8);
    readout_refresh();   /* initial "Mon  00:00" */
}
