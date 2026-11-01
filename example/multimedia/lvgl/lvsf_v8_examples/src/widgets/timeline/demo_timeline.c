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
#include "lvsf_timeline.h"
#include "demo_timeline.h"

/*********************
 *      DEFINES
 *********************/
#define BOX_X0    40            /* box start x */
#define BOX_Y0    110           /* box y (fixed) */
#define BOX_SZ0   40            /* box start size */
#define BOX_X1    200           /* box far x */
#define BOX_SZ1   70            /* box grown size */

/**********************
 *  STATIC FUNCTIONS
 **********************/
/* Two exec callbacks, each applies the timeline value to a different property of
 * the same box. Signature: void(*)(var, value, user_data). */
static void tl_move_x(void *var, int32_t value, void *ud)
{
    (void)var;
    lv_obj_set_x((lv_obj_t *)ud, (lv_coord_t)value);
}
static void tl_set_size(void *var, int32_t value, void *ud)
{
    (void)var;
    lv_obj_set_size((lv_obj_t *)ud, (lv_coord_t)value, (lv_coord_t)value);
}

/* "Run" click: reset the box, then play one timeline. The timeline runs to its
 * total time then self-deletes, so a fresh one is created per click. */
static void run_cb(lv_event_t *e)
{
    lv_obj_t *box = (lv_obj_t *)lv_event_get_user_data(e);
    lv_obj_set_pos(box, BOX_X0, BOX_Y0);
    lv_obj_set_size(box, BOX_SZ0, BOX_SZ0);

    lv_obj_t *tl = lv_timeline_create(lv_scr_act());
    lv_obj_set_size(tl, 0, 0);
    /* (start_value, end_value, start_time, end_time, exec_cb, ready_cb, var)
     * Three phases on one clock; the last two elements run in parallel to show
     * the timeline can coordinate animations at the same time, not just in turn. */
    lv_timeline_add_element(tl, BOX_X0, BOX_X1, 0, 600, tl_move_x, NULL, box);        /* 0..600    move right */
    lv_timeline_add_element(tl, BOX_SZ0, BOX_SZ1, 600, 1200, tl_set_size, NULL, box); /* 600..1200 grow */
    lv_timeline_add_element(tl, BOX_X1, BOX_X0, 1200, 1900, tl_move_x, NULL, box);    /* 1200..1900 move back  | */
    lv_timeline_add_element(tl, BOX_SZ1, BOX_SZ0, 1200, 1900, tl_set_size, NULL, box);/* 1200..1900 shrink back |- parallel */
    lv_timeline_set_time(tl, 1900);
    lv_timeline_start(tl);
}

/**********************
 *  GLOBAL FUNCTIONS
 **********************/

/**
 * @brief Demonstrate lvsf_timeline.
 *
 * lvsf_timeline is a generic animation-sequencing engine: each element maps a
 * time window [start_time, end_time] to a value range [start_value, end_value]
 * and feeds the interpolated value to an exec callback, which can apply it to
 * any property of any object. Several elements share one clock, so animations
 * are choreographed -- both in sequence and in parallel. Here a box moves right,
 * grows, then simultaneously shrinks and moves back; press "Run" to replay.
 */
void demo_timeline_init(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "timeline");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 20);

    lv_obj_t *cap = lv_label_create(scr);
    lv_label_set_text(cap, "press Run: move -> grow -> return");
    lv_obj_align(cap, LV_ALIGN_TOP_MID, 0, 56);

    lv_obj_t *box = lv_obj_create(scr);
    lv_obj_remove_style_all(box);          /* clean solid square, no theme shadow/border */
    lv_obj_set_size(box, BOX_SZ0, BOX_SZ0);
    lv_obj_set_pos(box, BOX_X0, BOX_Y0);
    lv_obj_set_style_bg_color(box, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_set_style_bg_opa(box, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(box, 6, 0);

    lv_obj_t *btn = lv_btn_create(scr);
    lv_obj_set_size(btn, 130, 40);
    lv_obj_align(btn, LV_ALIGN_TOP_MID, 0, 210);
    lv_obj_add_event_cb(btn, run_cb, LV_EVENT_CLICKED, box);
    lv_obj_t *bl = lv_label_create(btn);
    lv_label_set_text(bl, "Run");
    lv_obj_center(bl);
}
