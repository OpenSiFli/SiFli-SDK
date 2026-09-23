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
#include "lvsf_select.h"
#include "demo_select.h"

/*********************
 *      DEFINES
 *********************/
#define ICON_SZ  26
#define ROW_W    220
#define ROW_H    40
#define ROW_NUM  3

/**********************
 *  STATIC VARIABLES
 **********************/
static const char *OPTS[ROW_NUM] = {"Option A", "Option B", "Option C"};
static lv_obj_t *g_status;       /* label that echoes the current selection */

/**********************
 *  STATIC FUNCTIONS
 **********************/

/* Fill a buffer with a solid color and wrap a true-color image descriptor over
 * it -- lets the demo build the check/uncheck icons at runtime without assets. */
static void make_solid_img(lv_img_dsc_t *dsc, lv_color_t *buf, lv_coord_t sz, lv_color_t color)
{
    for (int i = 0; i < (int)(sz * sz); i++)
        buf[i] = color;
    dsc->header.cf = LV_IMG_CF_TRUE_COLOR;
    dsc->header.always_zero = 0;
    dsc->header.reserved = 0;
    dsc->header.w = sz;
    dsc->header.h = sz;
    dsc->data_size = (uint32_t)(sz * sz) * sizeof(lv_color_t);
    dsc->data = (const uint8_t *)buf;
}

/* A row's SHORT_CLICKED bubbles up to the select after the built-in handler has
 * moved the selection, so reading lv_select_get_select_idx here gives the new
 * choice -- which is the point of select: the app reads back what the user picked. */
static void sel_changed_cb(lv_event_t *e)
{
    lv_obj_t *sel = (lv_obj_t *)lv_event_get_user_data(e);
    uint16_t idx = lv_select_get_select_idx(sel);
    lv_label_set_text_fmt(g_status, "Selected: %s", OPTS[idx]);
}

/**********************
 *  GLOBAL FUNCTIONS
 **********************/

/**
 * @brief Demonstrate lvsf_select.
 *
 * select is a choice control, not just a display: each row holds a state
 * (check / uncheck / disable) and the user taps to choose. LV_SELECT_TYPE_SINGLE
 * keeps one row selected (radio style); LV_SELECT_TYPE_MULTI lets several be
 * checked (checkbox style). The app reads the choice back with
 * lv_select_get_select_idx() (single) or lv_select_get_ele_state() (per row).
 * This demo is a 3-row single select; the label below echoes the current pick.
 */
void demo_select_init(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "select");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 20);

    static lv_color_t chk_buf[ICON_SZ * ICON_SZ];
    static lv_color_t unchk_buf[ICON_SZ * ICON_SZ];
    static lv_img_dsc_t chk_dsc, unchk_dsc;
    make_solid_img(&chk_dsc, chk_buf, ICON_SZ, lv_palette_main(LV_PALETTE_GREEN)); /* selected   */
    make_solid_img(&unchk_dsc, unchk_buf, ICON_SZ, lv_color_hex(0x404040));        /* unselected */

    lv_obj_t *sel = lv_select_create(scr);
    lv_obj_add_flag(sel, LV_OBJ_FLAG_CLICKABLE);   /* constructor clears it; taps need it */
    lv_select_set_type(sel, LV_SELECT_TYPE_SINGLE);
    lv_select_set_ele_num(sel, ROW_NUM);
    lv_select_set_ele_size(sel, ROW_W, ROW_H);     /* lays out the rows */
    lv_select_set_check_src(sel, &chk_dsc);
    lv_select_set_uncheck_src(sel, &unchk_dsc);
    lv_select_set_ele_state(sel, 0, LV_SELECT_STATE_CHECK);  /* row 0 selected initially */
    lv_obj_align(sel, LV_ALIGN_TOP_MID, 0, 70);

    /* put a text label on each row so it reads like a real choice list */
    for (uint16_t i = 0; i < ROW_NUM; i++)
    {
        lv_obj_t *row = lv_select_get_ele(sel, i);
        lv_obj_t *l = lv_label_create(row);
        lv_label_set_text(l, OPTS[i]);
        lv_obj_set_style_text_color(l, lv_color_white(), 0);
        lv_obj_align(l, LV_ALIGN_LEFT_MID, 14, 0);
    }

    /* echo the current selection; updated whenever a row is tapped */
    g_status = lv_label_create(scr);
    lv_label_set_text_fmt(g_status, "Selected: %s", OPTS[0]);
    lv_obj_align(g_status, LV_ALIGN_TOP_MID, 0, 230);
    lv_obj_add_event_cb(sel, sel_changed_cb, LV_EVENT_SHORT_CLICKED, sel);
}
