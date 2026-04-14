/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*********************
 *      INCLUDES
 *********************/

#include "global.h"

#define _MODULE_NAME_ "multlist"
#include "app_module.h"

static void demo_multlist_back_event_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    gui_app_goback();
}

lv_obj_t *demo_multlist_create_back_button(lv_obj_t *parent)
{
    lv_obj_t *btn;
    lv_obj_t *label;

    btn = lv_btn_create(parent);
    lv_obj_set_size(btn, 72, 36);
    lv_obj_align(btn, LV_ALIGN_TOP_LEFT, 12, 12);
    lv_obj_clear_flag(btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_radius(btn, 18, 0);
    lv_obj_set_style_bg_color(btn, lv_color_make(32, 41, 56), 0);
    lv_obj_set_style_bg_opa(btn, LV_OPA_80, 0);
    lv_obj_set_style_border_width(btn, 0, 0);
    lv_obj_add_event_cb(btn, demo_multlist_back_event_cb, LV_EVENT_CLICKED, NULL);

    label = lv_label_create(btn);
    lv_label_set_text(label, "Back");
    lv_obj_center(label);

    lv_obj_move_foreground(btn);
    return btn;
}
