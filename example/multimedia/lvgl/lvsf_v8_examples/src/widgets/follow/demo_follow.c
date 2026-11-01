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
#include "lvsf_follow.h"
#include "demo_follow.h"

/*********************
 *      DEFINES
 *********************/
#define ICON_NUM 8

/**********************
 *  STATIC FUNCTIONS
 **********************/

/* item callback: return one icon object per element. Here a solid colored
 * circle; in a real app this would be an app launcher icon image. */
static lv_obj_t *follow_item_cb(lv_obj_t *parent, uint16_t index, uint16_t type, void *user_data)
{
    (void)type;
    (void)user_data;
    static const lv_palette_t pal[ICON_NUM] =
    {
        LV_PALETTE_RED, LV_PALETTE_BLUE, LV_PALETTE_GREEN, LV_PALETTE_ORANGE,
        LV_PALETTE_PURPLE, LV_PALETTE_CYAN, LV_PALETTE_PINK, LV_PALETTE_TEAL,
    };
    lv_obj_t *ic = lv_obj_create(parent);
    lv_obj_remove_style_all(ic);
    lv_obj_set_size(ic, 46, 46);
    lv_obj_set_style_radius(ic, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(ic, lv_palette_main(pal[index % ICON_NUM]), 0);
    lv_obj_set_style_bg_opa(ic, LV_OPA_COVER, 0);
    return ic;
}

static void follow_delete_cb(lv_follow_item_info_t *item)
{
    (void)item;
}

/**********************
 *  GLOBAL FUNCTIONS
 **********************/

/**
 * @brief Demonstrate lvsf_follow.
 *
 * follow is a physics "gravity" icon menu: icons arrange in concentric rings and
 * move under gravity. On a board a g-sensor drives the gravity; in the PC
 * simulator the widget's own path turns a mouse click into a gravity vector
 * (click the center to re-pack the icons, click off-center to pull them that
 * way), and a long-press drag moves an icon.
 *
 * cfg->custom_align must be true so the per-layer ring layout below (offset_r =
 * ring radius, target_r = icon radius, gap_angle -> icons per ring) is honored;
 * with false the widget auto-sizes icons from the widget width.
 */
void demo_follow_init(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "follow");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 16);

    lv_obj_t *follow = lv_follow_create(scr);
    lv_obj_set_size(follow, LV_PCT(100), 360);
    lv_obj_align(follow, LV_ALIGN_TOP_MID, 0, 50);

    lv_follow_cfg_t *cfg = lv_follow_get_cfg_param(follow);
    cfg->collision_type = FOLLOW_TYPE_STANDARDS;
    cfg->speed_ratio = 0.9f;
    cfg->black_ratio = 0.8f;
    cfg->hor_rate = 1.2f;
    cfg->ver_rate = 3.0f;
    cfg->margin = 4;
    cfg->gravity = 0.01f;
    cfg->friction = 0.2f;
    cfg->icon_r = 23;
    cfg->v_max = 3;
    cfg->target_r[0] = 23; cfg->target_r[1] = 19; cfg->target_r[2] = 14;  /* icon radius per ring */
    cfg->offset_r[0] = 0;  cfg->offset_r[1] = 70; cfg->offset_r[2] = 125; /* ring radius */
    cfg->start_angle[0] = 0; cfg->start_angle[1] = 0; cfg->start_angle[2] = 0;
    cfg->gap_angle[0] = 360; cfg->gap_angle[1] = 60; cfg->gap_angle[2] = 40; /* 360/gap = icons per ring */
    cfg->custom_align = true;   /* required: honor the explicit ring layout above */
    cfg->is_square = false;

    lv_follow_set_item_cb(follow, follow_item_cb, follow_delete_cb);
    for (int i = 0; i < ICON_NUM; i++)
        lv_follow_add_item_info(follow, 0, NULL);
    lv_follow_on_start(follow);
    lv_follow_enter_order_status(follow);
}
