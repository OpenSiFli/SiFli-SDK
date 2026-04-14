/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*********************
 *      INCLUDES
 *********************/

#include "global.h"
#include "demo_multlist.h"

#define _MODULE_NAME_ "multlist"
#include "app_module.h"

static lv_obj_t *p_launcher = NULL;

static void demo_main_open_multlist(lv_event_t *e)
{
    LV_UNUSED(e);
    gui_app_run(DEMO_MULTLIST_ID);
}

static void on_start(void)
{
    lv_obj_t *parent;
    lv_obj_t *card;
    lv_obj_t *icon;
    lv_obj_t *label;
    lv_obj_t *tip;

    parent = lv_scr_act();
    p_launcher = lv_obj_create(parent);
    lv_obj_remove_style_all(p_launcher);
    lv_obj_set_size(p_launcher, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_set_style_bg_color(p_launcher, lv_color_make(82, 93, 118), 0);
    lv_obj_set_style_bg_opa(p_launcher, LV_OPA_COVER, 0);
    lv_obj_center(p_launcher);

    card = lv_btn_create(p_launcher);
    lv_obj_set_size(card, LV_MIN(LV_HOR_RES_MAX - 40, 180), LV_MIN(LV_VER_RES_MAX - 80, 180));
    lv_obj_center(card);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_radius(card, 24, 0);
    lv_obj_set_style_bg_color(card, lv_color_make(189, 189, 189), 0);
    lv_obj_set_style_bg_opa(card, LV_OPA_100, 0);
    lv_obj_set_style_border_width(card, 0, 0);
    lv_obj_add_event_cb(card, demo_main_open_multlist, LV_EVENT_CLICKED, NULL);

    icon = lv_img_create(card);
    lv_img_set_src(icon, APP_GET_IMG(menu_multlist));
    lv_obj_align(icon, LV_ALIGN_TOP_MID, 0, 18);

    label = lv_label_create(card);
    lv_label_set_text(label, "Multlist");
    lv_obj_align(label, LV_ALIGN_BOTTOM_MID, 0, -18);

    tip = lv_label_create(p_launcher);
    lv_label_set_text(tip, "Tap icon to enter");
    lv_obj_align_to(tip, card, LV_ALIGN_OUT_BOTTOM_MID, 0, 16);
}

static void on_resume(void)
{
}

static void on_pause(void)
{
}

static void on_stop(void)
{
    p_launcher = NULL;
}

APPLICATION_REGISTER_HIDDEN(LV_EXT_STR_ID(multlist), DEMO_MULTLIST_MAIN_ID, 0);
